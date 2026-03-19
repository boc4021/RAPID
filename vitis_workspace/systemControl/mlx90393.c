/*
 * mlx90393.c - Bare-metal I2C driver for the MLX90393 magnetometer.
 *
 * See mlx90393.h for wiring and usage notes.
 *
 * Protocol summary (Melexis MLX90393 datasheet):
 *   All commands: master writes 1-byte opcode; sensor replies with 1-byte status.
 *   WR command:   [0x60|(reg<<2), data_hi, data_lo, reg<<2] then read 1 status byte.
 *   SM command:   [0x36] then read 1 status byte; wait for conversion (~6 ms).
 *   RM command:   [0x46] then read 7 bytes: [status, T_H, T_L, X_H, X_L, Y_H, Y_L].
 *   Data is signed 16-bit big-endian.
 *
 * Angle tracking algorithm mirrors tests/anglemeasure.ino:
 *   - Apply offset + scale calibration to raw XY values.
 *   - Compute angular delta via cross/dot product of consecutive XY vectors
 *     (numerically stable through wrap-around, unlike raw atan2 subtraction).
 *   - Divide by MLX_ANGLE_DIVISOR (1.54) for mechanical coupling.
 *   - Accumulate into dev->accumulated_angle.
 */

#include "mlx90393.h"
#include "xparameters.h"
#include "sleep.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                  */
/* ------------------------------------------------------------------ */

/*
 * mlx_cmd - send a single command byte, read back the 1-byte status.
 *
 * Returns status byte on success, 0xFF if the I2C transaction failed.
 * Caller should treat any status with bit[4] set (ERROR) as an error.
 */
static uint8_t mlx_cmd(MLX90393 *dev, uint8_t cmd)
{
    uint8_t buf = cmd;

    if (XIicPs_MasterSendPolled(&dev->iic, &buf, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return 0xFF;
    while (XIicPs_BusIsBusy(&dev->iic)) {}

    buf = 0;
    if (XIicPs_MasterRecvPolled(&dev->iic, &buf, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return 0xFF;

    return buf;
}

/*
 * mlx_write_reg - write a 16-bit value to an MLX90393 configuration register.
 *
 * WR transaction: [0x60|(reg<<2), data_high, data_low, reg<<2]
 * Then read back 1 status byte.
 * Returns status byte, or 0xFF on I2C error.
 */
static uint8_t mlx_write_reg(MLX90393 *dev, uint8_t reg, uint16_t val)
{
    uint8_t buf[4] = {
        (uint8_t)(MLX_CMD_WR | (uint8_t)(reg << 2)),
        (uint8_t)(val >> 8),
        (uint8_t)(val & 0xFFu),
        (uint8_t)(reg << 2)
    };
    uint8_t status;

    if (XIicPs_MasterSendPolled(&dev->iic, buf, 4, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return 0xFF;
    while (XIicPs_BusIsBusy(&dev->iic)) {}

    if (XIicPs_MasterRecvPolled(&dev->iic, &status, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return 0xFF;

    return status;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

int mlx_init(MLX90393 *dev, u32 iic_baseaddr)
{
    XIicPs_Config *cfg;

    memset(dev, 0, sizeof(*dev));
    dev->first_read = 1;

    /*
     * Initialise XIicPs.
     * Vitis 2025.1 SDT flow: XIicPs_LookupConfig takes a u32 base address.
     * XPAR_XIICPS_0_DEVICE_ID does not exist in this toolchain version.
     */
    cfg = XIicPs_LookupConfig(iic_baseaddr);
    if (!cfg) return XST_FAILURE;

    if (XIicPs_CfgInitialize(&dev->iic, cfg, cfg->BaseAddress) != XST_SUCCESS)
        return XST_FAILURE;

    if (XIicPs_SelfTest(&dev->iic) != XST_SUCCESS)
        return XST_FAILURE;

    if (XIicPs_SetSClk(&dev->iic, MLX90393_SCL_HZ) != XST_SUCCESS)
        return XST_FAILURE;

    /* EX — exit any in-progress measurement and return to idle */
    mlx_cmd(dev, MLX_CMD_EX);
    usleep(10000);   /* 10 ms */

    /* RT — full reset, restore OTP defaults (datasheet: max ~5 ms) */
    mlx_cmd(dev, MLX_CMD_RT);
    usleep(10000);   /* 10 ms */

    /*
     * WR CONF1: GAIN_SEL=5 [6:4], DIG_FILT=3 [3:2], OSR=3 [1:0] = 0x5F
     * ERROR bit is bit[4] of the returned status byte.
     */
    if (mlx_write_reg(dev, MLX_REG_CONF1, MLX_CONF1_VALUE) & 0x10u)
        return XST_FAILURE;

    /* WR CONF3: RES_X/Y/Z = 0 (16-bit resolution for all axes) */
    if (mlx_write_reg(dev, MLX_REG_CONF3, MLX_CONF3_VALUE) & 0x10u)
        return XST_FAILURE;

    dev->initialized = 1;
    return XST_SUCCESS;
}

int mlx_read_angle(MLX90393 *dev, float *angle_out)
{
    uint8_t cmd;
    uint8_t status;
    uint8_t buf[7];
    int16_t raw_x, raw_y;
    float   cal_x, cal_y;

    if (!dev->initialized) return XST_FAILURE;

    /* ---- Start single measurement (XY axes) ---- */
    cmd = MLX_CMD_SM_XY;
    if (XIicPs_MasterSendPolled(&dev->iic, &cmd, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return XST_FAILURE;
    while (XIicPs_BusIsBusy(&dev->iic)) {}
    if (XIicPs_MasterRecvPolled(&dev->iic, &status, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return XST_FAILURE;

    /* Fixed conversion wait — OSR=3/DIG_FILT=3 takes ~6 ms; 10 ms has margin */
    usleep(10000);

    /* ---- Read measurement ---- */
    /* RM returns: status(1) + T_H + T_L + X_H + X_L + Y_H + Y_L = 7 bytes */
    cmd = MLX_CMD_RM_XY;
    if (XIicPs_MasterSendPolled(&dev->iic, &cmd, 1, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return XST_FAILURE;
    while (XIicPs_BusIsBusy(&dev->iic)) {}
    memset(buf, 0, sizeof(buf));
    if (XIicPs_MasterRecvPolled(&dev->iic, buf, 7, MLX90393_I2C_ADDR) != XST_SUCCESS)
        return XST_FAILURE;

    /* Parse signed 16-bit big-endian values */
    raw_x = (int16_t)(((uint16_t)buf[3] << 8) | buf[4]);
    raw_y = (int16_t)(((uint16_t)buf[5] << 8) | buf[6]);

    /* Apply calibration (offset then scale) */
    cal_x = ((float)raw_x + MLX_X_OFFSET) * MLX_X_SCALE;
    cal_y = ((float)raw_y + MLX_Y_OFFSET) * MLX_Y_SCALE;

    /* On first read, seed the previous-sample state and return 0 degrees */
    if (dev->first_read) {
        dev->prev_x_cal = cal_x;
        dev->prev_y_cal = cal_y;
        dev->first_read  = 0;
        *angle_out = 0.0f;
        return XST_SUCCESS;
    }

    /*
     * Angular delta via cross/dot product of consecutive calibrated vectors.
     * This correctly handles the 0°/360° wrap-around that raw atan2 subtraction
     * would get wrong.
     *
     *   delta_rad = atan2(prev × cur, prev · cur)
     *
     * Divide by MLX_ANGLE_DIVISOR to account for mechanical coupling between
     * the magnet's rotation and one full spindle revolution.
     */
    float cross = dev->prev_x_cal * cal_y - dev->prev_y_cal * cal_x;
    float dot   = dev->prev_x_cal * cal_x + dev->prev_y_cal * cal_y;
    float delta = (atan2f(cross, dot) * 180.0f / 3.14159265f) / MLX_ANGLE_DIVISOR;

    dev->accumulated_angle += delta;
    dev->prev_x_cal = cal_x;
    dev->prev_y_cal = cal_y;

    *angle_out = dev->accumulated_angle;
    return XST_SUCCESS;
}
