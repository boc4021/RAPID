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
 * Wire format (shared with pcCommunication.c):
 *   SOF (0xAA 0x55) | TYPE (1 B) | LEN (1 B) | PAYLOAD (LEN B) | CRC8
 *
 * Incoming packet types:
 *   TYPE 0x01, LEN 0x08 - polar point: r_um (int32 LE) + theta_deg (float32 LE)
 *   TYPE 0x03, LEN 0x00 - end of sequence
 *
 * Outgoing:
 *   TYPE 0x81 - ACK, LEN = echo of incoming LEN, PAYLOAD = echo of incoming payload
 *
 * Build: Xilinx Vitis bare-metal project targeting Zynq-7000 PS.
 */

#include "xgpio.h"
#include "xparameters.h"
#include "platform.h"
#include "sleep.h"
#include "xuartps.h"
#include "xuartps_hw.h"
#include "mlx90393.h"

#include <stdio.h>
#include <stdarg.h>
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
/*  UART / protocol constants                                         */
/* ------------------------------------------------------------------ */

#define UART_BASEADDR   XPAR_XUARTPS_0_BASEADDR
#define BAUD_RATE       115200

#define SOF_BYTE_1      0xAA
#define SOF_BYTE_2      0x55
#define TYPE_POINT      0x01
#define TYPE_END        0x03
#define TYPE_ACK        0x81
#define TYPE_DEBUG      0xF0
#define POINT_LEN       0x08
#define MAX_PAYLOAD     255

/* ------------------------------------------------------------------ */
/*  Motor constants                                                   */
/* ------------------------------------------------------------------ */

/* Full stepper range: inner edge (home / step 0) to outer edge. */
#define MAX_STEPS        8500

/* Physical disc radius in micrometres (33 mm standard CD).
 * 8500 steps spans this full range: steps = round(r_um * MAX_STEPS / DISC_RADIUS_UM). */
#define DISC_RADIUS_UM   33000

/*
 * How long to wait after asserting stepper_en for the VHDL ZEROING state
 * to drive the sled to the proximity switch and reset step_total to 0.
 * At the VHDL homing rate of ~50 steps/s, 30 s covers 1500 steps.
 * Increase ZERO_WAIT_US if the sled may start further from the inner edge.
 */
#define ZERO_WAIT_US    30000000U   /* 30 seconds */

/* ------------------------------------------------------------------ */
/*  Globals                                                           */
/* ------------------------------------------------------------------ */

static XGpio   gpio;
static XUartPs Uart_Ps;
static MLX90393 mlx;

/* ------------------------------------------------------------------ */
/*  CRC / serialisation helpers                                       */
/* ------------------------------------------------------------------ */

static uint8_t crc8_xor(const uint8_t *data, unsigned len)
{
    uint8_t c = 0;
    for (unsigned i = 0; i < len; i++)
        c ^= data[i];
    return c;
}

static int32_t unpack_i32_le(const uint8_t b[4])
{
    return (int32_t)(
        ((uint32_t)b[0])       |
        ((uint32_t)b[1] << 8)  |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24)
    );
}

/* ------------------------------------------------------------------ */
/*  Low-level UART I/O (polled, no interrupts)                        */
/* ------------------------------------------------------------------ */

static void uart_send(const uint8_t *buf, unsigned len)
{
    UINTPTR base = Uart_Ps.Config.BaseAddress;
    for (unsigned i = 0; i < len; i++)
        XUartPs_SendByte(base, buf[i]);
}

static void uart_flush_tx(void)
{
    UINTPTR base = Uart_Ps.Config.BaseAddress;
    while (!(XUartPs_ReadReg(base, XUARTPS_SR_OFFSET) & XUARTPS_SR_TXEMPTY))
        ;
}

static uint8_t uart_recv_byte(void)
{
    uint8_t b;
    while (XUartPs_Recv(&Uart_Ps, &b, 1) != 1)
        ;
    return b;
}

/* ------------------------------------------------------------------ */
/*  Framed packet TX                                                  */
/* ------------------------------------------------------------------ */

/*
 * send_frame - build and transmit a framed response packet.
 *
 * Frame layout: SOF(2) | type(1) | len(1) | payload(len) | CRC8(1)
 * CRC is computed over [type, len, payload...].
 * Pass payload=NULL or len=0 for empty-payload frames.
 */
static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t hdr[4] = { SOF_BYTE_1, SOF_BYTE_2, type, len };

    uint8_t crc = type ^ len;
    for (uint8_t i = 0; i < len; i++)
        crc ^= payload[i];

    uart_send(hdr, sizeof(hdr));
    if (len && payload)
        uart_send(payload, len);
    uart_send(&crc, 1);
    uart_flush_tx();
}

/* ------------------------------------------------------------------ */
/*  Debug output via TYPE_DEBUG packet                               */
/* ------------------------------------------------------------------ */

/*
 * debug_printf - send a formatted string to the PC as a TYPE_DEBUG (0xF0)
 * framed packet.  The PC reader thread prints these as "[FPGA] <msg>".
 * Use this instead of xil_printf so debug output stays within the framed
 * protocol and is visible in the PC console / GUI while the port is open.
 */
static void debug_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    uint8_t len = 0;
    while (buf[len] && len < 255) len++;
    send_frame(TYPE_DEBUG, (const uint8_t *)buf, len);
}

/* ------------------------------------------------------------------ */
/*  Framed packet RX                                                  */
/* ------------------------------------------------------------------ */

/*
 * receive_packet - block until one valid framed packet is received.
 *
 * Fills out_payload (caller must provide MAX_PAYLOAD bytes) and returns
 * the TYPE byte.  Frames with CRC mismatches are silently discarded.
 */
static uint8_t receive_packet(uint8_t *out_payload)
{
    for (;;) {
        /* synchronise on SOF */
        if (uart_recv_byte() != SOF_BYTE_1) continue;
        if (uart_recv_byte() != SOF_BYTE_2) continue;

        uint8_t type = uart_recv_byte();
        uint8_t len  = uart_recv_byte();

        uint8_t payload[MAX_PAYLOAD];
        for (uint8_t i = 0; i < len; i++)
            payload[i] = uart_recv_byte();

        uint8_t rx_crc = uart_recv_byte();

        /* verify CRC over [TYPE, LEN, PAYLOAD...] */
        uint8_t chk[2 + MAX_PAYLOAD];
        chk[0] = type;
        chk[1] = len;
        for (uint8_t i = 0; i < len; i++)
            chk[2 + i] = payload[i];

        if (crc8_xor(chk, 2u + len) != rx_crc)
            continue;   /* CRC mismatch - drop frame and resync */

        for (uint8_t i = 0; i < len; i++)
            out_payload[i] = payload[i];

        return type;
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
        return XST_FAILURE;   /* UART not yet up - can't send debug packet */
    XGpio_SetDataDirection(&gpio, 1, 0x00000000);  /* ch1: all outputs */
    XGpio_SetDataDirection(&gpio, 2, 0xFFFFFFFF);  /* ch2: all inputs  */

    /* ---- initialise PS UART ---- */
    XUartPs_Config *cfg = XUartPs_LookupConfig(UART_BASEADDR);
    if (!cfg) { for (;;) ; }   /* halt - no UART config found */
    if (XUartPs_CfgInitialize(&Uart_Ps, cfg, cfg->BaseAddress) != XST_SUCCESS)
        { for (;;) ; }         /* halt - UART init failed */
    XUartPs_SetBaudRate(&Uart_Ps, BAUD_RATE);
    XUartPs_SetOperMode(&Uart_Ps, XUARTPS_OPER_MODE_NORMAL);

    /* ---- initialise MLX90393 magnetometer (I2C0 via EMIO → Arduino A4/A5) ---- */
    if (mlx_init(&mlx, XPAR_XIICPS_0_BASEADDR) != XST_SUCCESS)
        debug_printf("[MLX] init FAILED - angle logging disabled.");
    else
        debug_printf("[MLX] ready.");

    u32 config = 0;

    /* ===== 1. ZEROING =============================================== */
    /*
     * Assert stepper_en so the VHDL FSM immediately enters ZEROING state
     * and drives the sled towards the inner-edge proximity switch.
     * Wait ZERO_WAIT_US for the VHDL to detect prox_stable and reset
     * step_total to 0.  The PC waits a matching interval before sending
     * point packets (see FPGA_INIT_WAIT_MS in pcCommunication.c).
     */
    config = (1u << BIT_STEPPER_EN);
    XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);
    debug_printf("Zeroing started. Will wait %u s for sled to reach proximity switch...",
                 ZERO_WAIT_US / 1000000u);

    uint8_t rx_payload[MAX_PAYLOAD];
    uint8_t pkt_type;

    /* ===== 2. ZEROING WAIT ========================================= */
    usleep(ZERO_WAIT_US);
    debug_printf("Zeroing complete. Disc radius: %u um, max steps: %u. Ready for points.",
                 DISC_RADIUS_UM, MAX_STEPS);

    /* ===== 3. ENABLE SPINDLE ======================================= */
    config |= (1u << BIT_SPINDLE_EN);
    XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);
    debug_printf("Spindle enabled.");

    /* ===== 4. POINT LOOP =========================================== */
    int32_t current_step = 0;
    int     first_point  = 1;

    for (;;) {
        pkt_type = receive_packet(rx_payload);

        if (pkt_type == TYPE_POINT) {

            int32_t r_um = unpack_i32_le(&rx_payload[0]);
            /* theta_deg currently unused; spindle position not yet controlled */

            /* --- map physical radius (µm) to stepper steps ---
             *   steps = round( r_um / DISC_RADIUS_UM * MAX_STEPS )
             * 0 µm = home (inner edge, step 0); DISC_RADIUS_UM = outer edge (MAX_STEPS). */
            int32_t target_step = (int32_t)((double)r_um / DISC_RADIUS_UM * MAX_STEPS + 0.5);
            if (target_step < 0)         target_step = 0;
            if (target_step > MAX_STEPS) target_step = MAX_STEPS;

            int32_t delta = target_step - current_step;
            if (delta < 0) delta = -delta;
            int dir = (target_step >= current_step) ? 1 : 0;

            /* --- issue move (skip if already at target) --- */
            if (delta > 0) {
                /* update direction and step count in the GPIO word */
                config &= ~(1u << BIT_STEPPER_DIR);
                config |=  ((u32)dir << BIT_STEPPER_DIR);
                config &= ~((u32)NUM_STEP_MAX << BIT_NUM_STEP);
                config |=  ((u32)(delta & NUM_STEP_MAX) << BIT_NUM_STEP);
                XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);

                /* pulse step_go for 100 ms - rising edge triggers VHDL FSM */
                config |=  (1u << BIT_STEP_GO);
                XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);
                usleep(100000);   /* 100 ms pulse */
                config &= ~(1u << BIT_STEP_GO);
                XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);

                /*
                 * Wait for the move to finish:
                 *   1200 us   WAKEUP hold (150,000 cycles @ 125 MHz)
                 *   delta*125 RUNNING at 8 kHz step rate (125 us/step)
                 *   10000 us  safety margin
                 * (The 100 ms step_go pulse already elapsed above.)
                 */
                usleep(1200u + (uint32_t)delta * 125u + 10000u);
            }

            current_step = target_step;

            /* turn laser on after the first point's move completes */
            if (first_point) {
                config |= (1u << BIT_LASER_EN);
                XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);
                first_point = 0;
                debug_printf("Laser ON.");
            }

            /* log measured spindle angle for verification */
            if (mlx.initialized) {
                float theta_meas = 0.0f;
                if (mlx_read_angle(&mlx, &theta_meas) == XST_SUCCESS)
                    debug_printf("[THETA] %.2f deg", theta_meas);
            }

            /* ACK the point - echo payload back to PC */
            send_frame(TYPE_ACK, rx_payload, POINT_LEN);
        }

        else if (pkt_type == TYPE_END) {
            /* pattern finished - disable laser, spindle, and stepper */
            config &= ~(1u << BIT_LASER_EN);
            config &= ~(1u << BIT_SPINDLE_EN);
            config &= ~(1u << BIT_STEPPER_EN);
            XGpio_DiscreteWrite(&gpio, 1, config & GPIO_MASK);

            debug_printf("Pattern complete. Laser OFF. Motors stopped.");

            /* ACK end packet (LEN=0, no payload) */
            {
                uint8_t empty = 0;
                send_frame(TYPE_ACK, &empty, 0);
            }

            cleanup_platform();
            return 0;
        }
        /* any other packet type is silently ignored */
    }
}
