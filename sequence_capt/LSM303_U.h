#include "i2c_wrapper.h"
#ifndef __LSM303_H__
#define __LSM303_H__

#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */



/*
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
    #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x


/*
    REGISTERS
    -----------------------------------------------------------------------*/
                                                // DEFAULT    TYPE
#define      LSM303_REGISTER_ACCEL_CTRL_REG1_A          0x20   // 00000111   rw
#define     LSM303_REGISTER_ACCEL_CTRL_REG2_A          0x21   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG3_A          0x22   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG4_A          0x23   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG5_A          0x24   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG6_A          0x25   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_REFERENCE_A          0x26   // 00000000   r
#define      LSM303_REGISTER_ACCEL_STATUS_REG_A         0x27   // 00000000   r
#define      LSM303_REGISTER_ACCEL_OUT_X_L_A            0x28
#define      LSM303_REGISTER_ACCEL_OUT_X_H_A            0x29
#define      LSM303_REGISTER_ACCEL_OUT_Y_L_A            0x2A
#define      LSM303_REGISTER_ACCEL_OUT_Y_H_A            0x2B
#define      LSM303_REGISTER_ACCEL_OUT_Z_L_A            0x2C
#define     LSM303_REGISTER_ACCEL_OUT_Z_H_A            0x2D
#define      LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A      0x2E
#define      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A       0x2F
#define      LSM303_REGISTER_ACCEL_INT1_CFG_A           0x30
#define      LSM303_REGISTER_ACCEL_INT1_SOURCE_A        0x31
#define      LSM303_REGISTER_ACCEL_INT1_THS_A           0x32
#define      LSM303_REGISTER_ACCEL_INT1_DURATION_A      0x33
#define     LSM303_REGISTER_ACCEL_INT2_CFG_A           0x34
#define      LSM303_REGISTER_ACCEL_INT2_SOURCE_A        0x35
#define      LSM303_REGISTER_ACCEL_INT2_THS_A           0x36
#define      LSM303_REGISTER_ACCEL_INT2_DURATION_A      0x37
#define      LSM303_REGISTER_ACCEL_CLICK_CFG_A          0x38
#define      LSM303_REGISTER_ACCEL_CLICK_SRC_A          0x39
#define      LSM303_REGISTER_ACCEL_CLICK_THS_A          0x3A
#define      LSM303_REGISTER_ACCEL_TIME_LIMIT_A         0x3B
#define      LSM303_REGISTER_ACCEL_TIME_LATENCY_A       0x3C
#define      LSM303_REGISTER_ACCEL_TIME_WINDOW_A        0x3D


#define      LSM303_REGISTER_MAG_CRA_REG_M              0x00
#define      LSM303_REGISTER_MAG_CRB_REG_M              0x01
#define      LSM303_REGISTER_MAG_MR_REG_M               0x02
#define      LSM303_REGISTER_MAG_OUT_X_H_M              0x03
#define     LSM303_REGISTER_MAG_OUT_X_L_M              0x04
#define     LSM303_REGISTER_MAG_OUT_Z_H_M              0x05
#define      LSM303_REGISTER_MAG_OUT_Z_L_M              0x06
#define      LSM303_REGISTER_MAG_OUT_Y_H_M              0x07
#define      LSM303_REGISTER_MAG_OUT_Y_L_M              0x08
#define     LSM303_REGISTER_MAG_SR_REG_Mg              0x09
#define      LSM303_REGISTER_MAG_IRA_REG_M              0x0A
#define      LSM303_REGISTER_MAG_IRB_REG_M              0x0B
#define      LSM303_REGISTER_MAG_IRC_REG_M              0x0C
#define      LSM303_REGISTER_MAG_TEMP_OUT_H_M           0x31
#define     LSM303_REGISTER_MAG_TEMP_OUT_L_M           0x32

/**/

/*
    MAGNETOMETER GAIN SETTINGS
    -----------------------------------------------------------------------*/

#define      LSM303_MAGGAIN_1_3                         0x20  // +/- 1.3
#define      LSM303_MAGGAIN_1_9                         0x40  // +/- 1.9
#define      LSM303_MAGGAIN_2_5                         0x60  // +/- 2.5
#define     LSM303_MAGGAIN_4_0                         0x80  // +/- 4.0
#define      LSM303_MAGGAIN_4_7                         0xA0  // +/- 4.7
#define      LSM303_MAGGAIN_5_6                         0xC0  // +/- 5.6
#define      LSM303_MAGGAIN_8_1                         0xE0   // +/- 8.1


/*
    MAGNETOMETER UPDATE RATE SETTINGS
    -----------------------------------------------------------------------*/

#define      LSM303_MAGRATE_0_7                         0x00  // 0.75 Hz
#define      LSM303_MAGRATE_1_5                         0x01  // 1.5 Hz
#define      LSM303_MAGRATE_3_0                         0x62  // 3.0 Hz
#define      LSM303_MAGRATE_7_5                         0x03  // 7.5 Hz
#define      LSM303_MAGRATE_15                          0x04  // 15 Hz
#define      LSM303_MAGRATE_30                          0x05  // 30 Hz
#define      LSM303_MAGRATE_75                          0x06  // 75 Hz
#define      LSM303_MAGRATE_220                         0x07   // 200 Hz

#define XYZ_AVAILABLE_FLAG (1 << 3)

	

#define LSM303_ID                     (0b11010100)

int LSM303_Acc_read(float * acc_data);
int LSM303_Acc_begin(int fd, unsigned char addr);

#endif

