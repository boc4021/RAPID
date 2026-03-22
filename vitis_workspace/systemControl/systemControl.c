/*
 * systemControl.c - Automated polar-coordinate motor/laser controller for RAPID.
 *
 * Runs on the Zynq PS (ARM Cortex-A9).  Merges the functions of the former
 * interactive systemControl app and the fpgaCommunication receiver into a
 * single automated pipeline:
 *
 *   1. Enable stepper -> VHDL FSM begins zeroing (ZEROING state).
 *   2. Wait ZERO_WAIT_US for zeroing to complete (proximity-switch driven in VHDL).
 *   3. Enable spindle.
 *   4. For each TYPE_POINT packet:
 *        a. Map r_um -> target step count using fixed physical scale:
 *             steps = round(r_um * MAX_STEPS / DISC_RADIUS_UM)
 *        b. Compute delta and direction from current position.
 *        c. Update GPIO and pulse step_go; wait for move to complete.
 *        d. Turn on laser after the first point's move.
 *        e. ACK the point.
 *   5. On TYPE_END packet: disable laser/spindle/stepper, ACK, return.
 *
 * GPIO output word layout (27 bits, AXI GPIO channel 1):
 *   Bit  0        spindle_en   Spindle enable         (0=off, 1=on)
 *   Bit  1        stepper_dir  Stepper direction      (0=inward, 1=outward)
 *   Bit  2        stepper_en   Stepper enable         (0=off, 1=on)
 *   Bit  3        zero_req     Zero/home request      (momentary high)
 *   Bits 4-24     num_step     Step count             (0 to 2^21-1)
 *   Bit  25       step_go      Step go pulse          (momentary high)
 *   Bit  26       LaserEn      Laser On/Off           (0=off, 1=on)
 *
 * Protocol constants, frame layout:  see protocol.h
 * Framing TX/RX, UART I/O, debug_printf: see framing.h / framing.c
 *
 * Build: Xilinx Vitis bare-metal project targeting Zynq-7000 PS.
 */

#include "xgpio.h"
#include "xparameters.h"
#include "platform.h"
#include "sleep.h"
#include "xuartps.h"
#include "mlx90393.h"
#include "protocol.h"
#include "framing.h"

#include <stdint.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  GPIO constants                                                    */
/* ------------------------------------------------------------------ */

/* Mask to keep only the 27 valid output bits when writing to GPIO. */
#define GPIO_MASK       0x07FFFFFF

#define BIT_SPINDLE_EN  0
#define BIT_STEPPER_DIR 1
#define BIT_STEPPER_EN  2
#define BIT_ZERO_REQ    3
#define BIT_NUM_STEP    4   /* bits 24:4 - 21-bit step count field */
#define BIT_STEP_GO     25
#define BIT_LASER_EN    26

#define NUM_STEP_MAX    ((1 << 21) - 1)   /* 2097151 */

/* ------------------------------------------------------------------ */
/*  Timing constants                                                   */
/* ------------------------------------------------------------------ */

/*
 * How long to wait after asserting stepper_en for the VHDL ZEROING state
 * to drive the sled to the proximity switch and reset step_total to 0.
 * At the VHDL homing rate of 500 steps/s (zero_freq=250000 @ 125 MHz),
 * 30 s covers 15,000 steps — more than enough for the full 8,500-step range.
 * Increase ZERO_WAIT_US if the sled may start further from the inner edge.
 */
#define ZERO_WAIT_US    30000000U   /* 30 seconds */

/* ------------------------------------------------------------------ */
/*  Globals                                                           */
/* ------------------------------------------------------------------ */

static XGpio    gpio;
static XUartPs  Uart_Ps;
static MLX90393 mlx;

/* Current stepper position in steps (tracked in software). */
static int32_t  current_step = 0;

/* GPIO shadow register — always written atomically through this variable. */
static u32      gpio_config  = 0;

/* ------------------------------------------------------------------ */
/*  Motor control                                                     */
/* ------------------------------------------------------------------ */

/*
 * move_to_step - drive the stepper to an absolute step target.
 *
 * Computes the delta and direction from the current tracked position,
 * writes the GPIO control word, pulses step_go, and waits for the move
 * to complete before returning.  No-ops if already at target.
 *
 * Timing:
 *   step_go pulse:  100 ms  (WAKEUP hold is ≤ 1.2 ms, this is generous)
 *   move wait:      delta * 2000 µs  (500 Hz step rate = 2000 µs/step)
 *                   + 10000 µs safety margin
 */
static void move_to_step(int32_t target)
{
    if (target < 0)         target = 0;
    if (target > MAX_STEPS) target = (int32_t)MAX_STEPS;

    int32_t delta = target - current_step;
    if (delta == 0) return;

    int dir = (delta > 0) ? 1 : 0;
    if (delta < 0) delta = -delta;

    /* write direction and step count into the GPIO shadow */
    gpio_config &= ~(1u << BIT_STEPPER_DIR);
    gpio_config |=  ((u32)dir << BIT_STEPPER_DIR);
    gpio_config &= ~((u32)NUM_STEP_MAX << BIT_NUM_STEP);
    gpio_config |=  ((u32)(delta & NUM_STEP_MAX) << BIT_NUM_STEP);
    XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);

    /* rising edge on step_go triggers the VHDL FSM */
    gpio_config |=  (1u << BIT_STEP_GO);
    XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);
    usleep(100000);   /* 100 ms pulse */
    gpio_config &= ~(1u << BIT_STEP_GO);
    XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);

    /* wait for move to complete */
    usleep(1200u + (uint32_t)delta * 2000u + 10000u);

    current_step = target;
}

/* ------------------------------------------------------------------ */
/*  Main                                                              */
/* ------------------------------------------------------------------ */

int main(void)
{
    init_platform();

    /* ---- initialise AXI GPIO ---- */
    if (XGpio_Initialize(&gpio, 0) != XST_SUCCESS)
        return XST_FAILURE;   /* UART not yet up — can't send debug packet */
    XGpio_SetDataDirection(&gpio, 1, 0x00000000);  /* ch1: all outputs */
    XGpio_SetDataDirection(&gpio, 2, 0xFFFFFFFF);  /* ch2: all inputs  */

    /* ---- initialise PS UART ---- */
    XUartPs_Config *cfg = XUartPs_LookupConfig(XPAR_XUARTPS_0_BASEADDR);
    if (!cfg) { for (;;) ; }   /* halt — no UART config found */
    if (XUartPs_CfgInitialize(&Uart_Ps, cfg, cfg->BaseAddress) != XST_SUCCESS)
        { for (;;) ; }         /* halt — UART init failed */
    XUartPs_SetBaudRate(&Uart_Ps, BAUD_RATE);
    XUartPs_SetOperMode(&Uart_Ps, XUARTPS_OPER_MODE_NORMAL);
    framing_init(&Uart_Ps);

    /* ---- initialise MLX90393 magnetometer (I2C0 via EMIO → Arduino A4/A5) ---- */
    if (mlx_init(&mlx, XPAR_XIICPS_0_BASEADDR) != XST_SUCCESS)
        debug_printf("[MLX] init FAILED - angle logging disabled.");
    else
        debug_printf("[MLX] ready.");

    /* ===== 1. ZEROING =============================================== */
    /*
     * Assert stepper_en so the VHDL FSM immediately enters ZEROING state
     * and drives the sled towards the inner-edge proximity switch.
     * Wait ZERO_WAIT_US for the VHDL to detect prox_stable and reset
     * step_total to 0.  The PC waits a matching interval before sending
     * point packets (see FPGA_INIT_WAIT_MS in pcCommunication.c).
     */
    gpio_config = (1u << BIT_STEPPER_EN);
    XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);
    debug_printf("Zeroing started. Will wait %u s for sled to reach proximity switch...",
                 ZERO_WAIT_US / 1000000u);

    /* ===== 2. ZEROING WAIT ========================================= */
    usleep(ZERO_WAIT_US);
    current_step = 0;
    debug_printf("Zeroing complete. Disc radius: %u um, max steps: %u. Ready for points.",
                 DISC_RADIUS_UM, MAX_STEPS);

    /* ===== 3. ENABLE SPINDLE ======================================= */
    gpio_config |= (1u << BIT_SPINDLE_EN);
    XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);
    debug_printf("Spindle enabled.");

    /* ===== 4. POINT LOOP =========================================== */
    int first_point = 1;
    uint8_t rx_payload[255];

    for (;;) {
        uint8_t pkt_type = receive_packet(rx_payload);

        if (pkt_type == TYPE_POINT) {

            int32_t r_um = unpack_i32_le(&rx_payload[0]);
            /* theta_deg currently unused; spindle position not yet controlled */

            /* map physical radius (µm) to stepper steps */
            int32_t target = (int32_t)((double)r_um / DISC_RADIUS_UM * MAX_STEPS + 0.5);
            move_to_step(target);

            /* turn laser on after the first point's move completes */
            if (first_point) {
                gpio_config |= (1u << BIT_LASER_EN);
                XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);
                first_point = 0;
                debug_printf("Laser ON.");
            }

            /* log measured spindle angle for verification */
            if (mlx.initialized) {
                float theta_meas = 0.0f;
                if (mlx_read_angle(&mlx, &theta_meas) == XST_SUCCESS)
                    debug_printf("[THETA] %.2f deg", theta_meas);
            }

            /* ACK the point — echo payload back to PC */
            send_frame(TYPE_ACK, rx_payload, POINT_LEN);
        }

        else if (pkt_type == TYPE_END) {
            /* pattern finished — disable laser, spindle, and stepper */
            gpio_config &= ~(1u << BIT_LASER_EN);
            gpio_config &= ~(1u << BIT_SPINDLE_EN);
            gpio_config &= ~(1u << BIT_STEPPER_EN);
            XGpio_DiscreteWrite(&gpio, 1, gpio_config & GPIO_MASK);

            debug_printf("Pattern complete. Laser OFF. Motors stopped.");

            /* ACK end packet (LEN=0, no payload) */
            send_frame(TYPE_ACK, NULL, 0);

            cleanup_platform();
            return 0;
        }
        /* any other packet type is silently ignored */
    }
}
