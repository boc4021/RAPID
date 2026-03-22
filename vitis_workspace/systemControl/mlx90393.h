/*
 * mlx90393.h - Bare-metal I2C driver for the MLX90393 magnetometer.
 *
 * Communicates over PS I2C0 (EMIO → Arduino-compatible header A4/A5).
 * Reads XY magnetic field axes and accumulates spindle angle for
 * logging/verification.  No closed-loop control — angle is emitted
 * via debug_printf after each stepper move.
 *
 * Wiring (Arty Z7-20 Arduino-compatible header):
 *   SCL → A5 (FPGA P15)      SDA → A4 (FPGA P16)
 *   VCC → 3.3 V              GND → GND
 *   A0/A1 → GND  (I2C addr 0x0C)
 *   External 4.7 kΩ pull-ups on SCL and SDA to 3.3 V required.
 *
 * Vitis 2025.1 SDT note: use XPAR_XIICPS_0_BASEADDR (not DEVICE_ID).
 */

#ifndef MLX90393_H
#define MLX90393_H

#include "xiicps.h"
#include <stdint.h>

/* ---- I2C bus parameters ---- */
#define MLX90393_I2C_ADDR    0x0C    /* A0/A1 tied to GND */
#define MLX90393_SCL_HZ      100000U /* standard-mode 100 kHz */

/* ---- Command opcodes ---- */
#define MLX_CMD_EX           0x80u   /* Exit — return to idle */
#define MLX_CMD_RT           0xF0u   /* Reset — restore OTP defaults */
/*
 * Axis bits (in SM/RM opcode low nibble): T=0x01, X=0x02, Y=0x04, Z=0x08.
 * Reading T+X+Y (mask 0x07) produces a 7-byte RM response:
 *   [status, T_H, T_L, X_H, X_L, Y_H, Y_L]
 * T is read but discarded; its presence is required to keep X/Y at the
 * expected byte offsets (3 and 5).  Using X+Y only (mask 0x06) would return
 * only 5 bytes and shift X/Y to offsets 1 and 3 — mismatching the parser.
 */
#define MLX_CMD_SM_XY        0x37u   /* Start single measurement: T+X+Y axes (mask 0x07) */
#define MLX_CMD_RM_XY        0x47u   /* Read measurement: T+X+Y — returns 7 bytes */
#define MLX_CMD_WR           0x60u   /* Write register */

/* ---- Configuration register addresses ---- */
#define MLX_REG_CONF1        0x00u
#define MLX_REG_CONF3        0x02u

/*
 * CONF1 (reg 0x00): gain only.
 *   GAIN_SEL[6:4] = 5  → 1.667x gain  (5 << 4 = 0x50)
 *   Bits [3:0] left 0 (OTP reset defaults).
 *
 * CONF3 (reg 0x02): filter, oversampling, resolution.
 *   DIG_FILT[4:2] = 3  (3 << 2 = 0x0C)
 *   OSR[1:0]      = 3  (        0x03)
 *   RES_X/Y/Z     = 0  (16-bit, default)
 *   Combined: 0x0C | 0x03 = 0x000F
 *
 * Note: DIG_FILT and OSR belong in CONF3, not CONF1.
 * Conversion time at DIG_FILT=3, OSR=3 is ~6 ms (see datasheet Table 18);
 * mlx_read_angle waits 10 ms to include margin.
 */
#define MLX_CONF1_VALUE      0x0050u  /* GAIN_SEL[6:4]=5 */
#define MLX_CONF3_VALUE      0x000Fu  /* DIG_FILT[4:2]=3, OSR[1:0]=3, RES=0 */

/* ---- Calibration — identical to tests/anglemeasure.ino ---- */
#define MLX_X_OFFSET         (-47.5498f)
#define MLX_Y_OFFSET         (-23.2500f)
#define MLX_X_SCALE          (0.00010226f)
#define MLX_Y_SCALE          (0.00010226f)

/* Mechanical coupling factor from the original sketch */
#define MLX_ANGLE_DIVISOR    (1.54f)

/* ---- Driver instance ---- */
typedef struct {
    XIicPs  iic;               /* Xilinx PS I2C driver instance */
    float   prev_x_cal;        /* calibrated X from previous read */
    float   prev_y_cal;        /* calibrated Y from previous read */
    float   accumulated_angle; /* total rotation (degrees) */
    int     initialized;       /* non-zero after successful mlx_init() */
    int     first_read;        /* 1 until first sample taken */
} MLX90393;

/*
 * mlx_init - initialise PS I2C0 and configure the MLX90393.
 *
 * @dev:          caller-allocated MLX90393 (need not be zero-init; mlx_init
 *                does a full memset internally)
 * @iic_baseaddr: XIicPs base address — pass XPAR_XIICPS_0_BASEADDR
 *                (Vitis 2025.1 SDT flow: no DEVICE_ID)
 *
 * Returns XST_SUCCESS on success; XST_FAILURE on any I2C or sensor error.
 */
int mlx_init(MLX90393 *dev, u32 iic_baseaddr);

/*
 * mlx_read_angle - take one XY measurement and update accumulated angle.
 *
 * @dev:       initialised MLX90393 instance
 * @angle_out: set to current accumulated angle in degrees
 *
 * Returns XST_SUCCESS on success; XST_FAILURE on I2C error (*angle_out
 * is unchanged on failure).
 */
int mlx_read_angle(MLX90393 *dev, float *angle_out);

#endif /* MLX90393_H */
