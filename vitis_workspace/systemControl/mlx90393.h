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
#define MLX_CMD_SM_XY        0x36u   /* Start single measurement: X+Y axes (mask 0x06) */
#define MLX_CMD_RM_XY        0x46u   /* Read measurement: X+Y */
#define MLX_CMD_WR           0x60u   /* Write register (OR with reg<<2) */

/* ---- Configuration register addresses ---- */
#define MLX_REG_CONF1        0x00u
#define MLX_REG_CONF3        0x02u

/*
 * CONF1: GAIN_SEL[6:4]=5, DIG_FILT[3:2]=3, OSR[1:0]=3
 * = (5<<4)|(3<<2)|3 = 0x50|0x0C|0x03 = 0x5F
 * Matches Adafruit_MLX90393 library defaults.
 */
#define MLX_CONF1_VALUE      0x005Fu

/* CONF3: RES_X[11:10]=0, RES_Y[9:8]=0, RES_Z[7:6]=0 — max 16-bit resolution */
#define MLX_CONF3_VALUE      0x0000u

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
