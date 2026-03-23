/*
 * main.c - Automated polar-coordinate motor/laser controller for RAPID.
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

/*
 * RAPID_ENABLE_MLX — set to 0 to compile out the magnetometer driver entirely
 * (e.g. when the MLX90393 is not fitted on the board).
 */
#ifndef RAPID_ENABLE_MLX
#  define RAPID_ENABLE_MLX 1
#endif

#include "xgpio.h"
#include "xparameters.h"
#include "platform.h"
#include "sleep.h"
#include "xuartps.h"
#if RAPID_ENABLE_MLX
#  include "mlx90393.h"
#endif
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
#define ZERO_WAIT_US        30000000U   /* 30 seconds */

/* move_to_step() timing — see comment block above the function. */
#define STEP_GO_PULSE_US    100000U     /* 100 ms — generous for DRV8834 wake */
#define WAKEUP_HOLD_US      1200U       /* DRV8834 SLEEP→active settling      */
#define US_PER_STEP         2000U       /* 500 Hz step rate = 2000 us/step    */
#define MOVE_MARGIN_US      10000U      /* safety margin after computed move  */

/* ------------------------------------------------------------------ */
/*  Globals                                                           */
/* ------------------------------------------------------------------ */

static XGpio    gpio;
static XUartPs  Uart_Ps;
#if RAPID_ENABLE_MLX
static MLX90393 mlx;
#endif

/* Current stepper position in steps (tracked in software). */
static int32_t  current_step = 0;

/* GPIO shadow register — always written atomically through this variable. */
static u32      gpio_config  = 0;

/* ------------------------------------------------------------------ */
/*  GPIO wrapper  (S-DIP-02)                                          */
/* ------------------------------------------------------------------ */

/** Write value to AXI GPIO channel 1, masking to the 27 valid bits. */
static void gpio_write(u32 value)
{
    XGpio_DiscreteWrite(&gpio, 1, value & GPIO_MASK);
}

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
    gpio_write(gpio_config);

    /* rising edge on step_go triggers the VHDL FSM */
    gpio_config |=  (1u << BIT_STEP_GO);
    gpio_write(gpio_config);
    usleep(STEP_GO_PULSE_US);
    gpio_config &= ~(1u << BIT_STEP_GO);
    gpio_write(gpio_config);

    /* wait for move to complete */
    usleep(WAKEUP_HOLD_US + (uint32_t)delta * US_PER_STEP + MOVE_MARGIN_US);

    current_step = target;
}

/* ------------------------------------------------------------------ */
/*  Halt helper                                                       */
/* ------------------------------------------------------------------ */

/*
 * FPGA_HALT - disable all actuators then spin forever.
 * Writes 0 to GPIO so laser, spindle, and stepper are all off.
 * Only safe to call after XGpio_Initialize has succeeded.
 */
#define FPGA_HALT() \
    do { gpio_write(0); for (;;) ; } while (0)

/* ------------------------------------------------------------------ */
/*  Point loop — packet handlers  (S-SRP-03)                         */
/* ------------------------------------------------------------------ */

/** Loop context shared by all packet handlers. */
typedef struct {
    int first_point;   /* 1 until the laser has been turned on */
    int done;          /* set to 1 by on_end / on_timeout to exit loop */
} LoopCtx;

/** Callback type for the dispatch table. */
typedef void (*PktFn)(const uint8_t *payload, uint8_t len, void *ctx);

/** Log the measured spindle angle via the magnetometer (no-op if disabled). */
static void log_angle(void)
{
#if RAPID_ENABLE_MLX
    if (mlx.initialized) {
        float theta_meas = 0.0f;
        if (mlx_read_angle(&mlx, &theta_meas) == XST_SUCCESS)
            debug_printf("[THETA] %.2f deg", theta_meas);
    }
#endif
}

static void on_point(const uint8_t *payload, uint8_t len, void *ctx_v)
{
    LoopCtx *ctx = (LoopCtx *)ctx_v;
    if (len != POINT_LEN) {
        debug_printf("[ERR] TYPE_POINT wrong len=%u (expected %u)", len, POINT_LEN);
        return;
    }

    int32_t r_um   = unpack_i32_le(&payload[0]);
    /* theta_deg currently unused; spindle position not yet controlled */

    int32_t target = (int32_t)((double)r_um / DISC_RADIUS_UM * MAX_STEPS + 0.5);
    move_to_step(target);

    /* turn laser on after the first point's move completes */
    if (ctx->first_point) {
        gpio_config |= (1u << BIT_LASER_EN);
        gpio_write(gpio_config);
        ctx->first_point = 0;
        debug_printf("Laser ON.");
    }

    log_angle();

    /* ACK the point — echo payload back to PC */
    send_frame(TYPE_ACK, payload, POINT_LEN);
}

static void on_end(const uint8_t *payload, uint8_t len, void *ctx_v)
{
    (void)payload; (void)len;
    LoopCtx *ctx = (LoopCtx *)ctx_v;

    /* pattern finished — disable laser, spindle, and stepper */
    gpio_config &= ~(1u << BIT_LASER_EN);
    gpio_config &= ~(1u << BIT_SPINDLE_EN);
    gpio_config &= ~(1u << BIT_STEPPER_EN);
    gpio_write(gpio_config);

    debug_printf("Pattern complete. Laser OFF. Motors stopped.");
    send_frame(TYPE_ACK, NULL, 0);
    ctx->done = 1;
}

static void on_timeout(const uint8_t *payload, uint8_t len, void *ctx_v)
{
    (void)payload; (void)len;
    LoopCtx *ctx = (LoopCtx *)ctx_v;

    /* No byte arrived within ~0.4 s — PC likely disconnected mid-run.
     * Shut down all actuators before halting so the disc stops safely. */
    gpio_config &= ~(1u << BIT_LASER_EN);
    gpio_config &= ~(1u << BIT_SPINDLE_EN);
    gpio_config &= ~(1u << BIT_STEPPER_EN);
    gpio_write(gpio_config);
    debug_printf("[ERR] UART receive timeout — safe shutdown, halting.");
    ctx->done = 1;
    FPGA_HALT();
}

/* Dispatch table — add a row here to support a new packet type.
 * run_point_loop itself never needs to change.  (S-OCP-02)          */
static const struct { uint8_t type; PktFn fn; } pkt_table[] = {
    { TYPE_POINT,   on_point   },
    { TYPE_END,     on_end     },
    { TYPE_TIMEOUT, on_timeout },
};
#define PKT_TABLE_COUNT (sizeof(pkt_table) / sizeof(pkt_table[0]))

/* ------------------------------------------------------------------ */
/*  Point loop — dispatch loop only  (S-OCP-02)                      */
/* ------------------------------------------------------------------ */

/*
 * run_point_loop - receive packets until TYPE_END or TYPE_TIMEOUT,
 * dispatching each via pkt_table[].  Driving the stepper, laser, and
 * ACK logic lives in the individual handler functions above.
 */
static void run_point_loop(void)
{
    LoopCtx ctx = { 1, 0 };
    uint8_t rx_payload[255];
    uint8_t rx_len = 0;

    for (;;) {
        rx_len = 0;
        uint8_t pkt_type = receive_packet(rx_payload, &rx_len);
        for (size_t i = 0; i < PKT_TABLE_COUNT; i++) {
            if (pkt_table[i].type == pkt_type) {
                pkt_table[i].fn(rx_payload, rx_len, &ctx);
                break;
            }
        }
        if (ctx.done) return;
        /* unknown packet types are silently ignored */
    }
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
    if (!cfg) { FPGA_HALT(); }   /* halt — no UART config found */
    if (XUartPs_CfgInitialize(&Uart_Ps, cfg, cfg->BaseAddress) != XST_SUCCESS)
        { FPGA_HALT(); }         /* halt — UART init failed */
    if (XUartPs_SetBaudRate(&Uart_Ps, BAUD_RATE) != XST_SUCCESS)
        { FPGA_HALT(); }         /* halt — baud rate out of range for system clock */
    XUartPs_SetOperMode(&Uart_Ps, XUARTPS_OPER_MODE_NORMAL);
    framing_init(&Uart_Ps);

#if RAPID_ENABLE_MLX
    /* ---- initialise MLX90393 magnetometer (I2C0 via EMIO → Arduino A4/A5) ---- */
    if (mlx_init(&mlx, XPAR_XIICPS_0_BASEADDR) != XST_SUCCESS)
        debug_printf("[MLX] init FAILED - angle logging disabled.");
    else
        debug_printf("[MLX] ready.");
#endif

    /* ===== 1. ZEROING =============================================== */
    /*
     * Assert stepper_en so the VHDL FSM immediately enters ZEROING state
     * and drives the sled towards the inner-edge proximity switch.
     * Wait ZERO_WAIT_US for the VHDL to detect prox_stable and reset
     * step_total to 0.  The PC waits a matching interval before sending
     * point packets (see FPGA_INIT_WAIT_MS in src/main.c).
     */
    gpio_config = (1u << BIT_STEPPER_EN);
    gpio_write(gpio_config);
    debug_printf("Zeroing started. Will wait %u s for sled to reach proximity switch...",
                 ZERO_WAIT_US / 1000000u);

    /* ===== 2. ZEROING WAIT ========================================= */
    usleep(ZERO_WAIT_US);
    current_step = 0;
    debug_printf("Zeroing complete. Disc radius: %u um, max steps: %u. Ready for points.",
                 DISC_RADIUS_UM, MAX_STEPS);

    /* ===== 3. ENABLE SPINDLE ======================================= */
    gpio_config |= (1u << BIT_SPINDLE_EN);
    gpio_write(gpio_config);
    debug_printf("Spindle enabled.");

    /* ===== 4. POINT LOOP =========================================== */
    run_point_loop();

    cleanup_platform();
    return 0;
}
