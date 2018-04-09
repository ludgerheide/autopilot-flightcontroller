//
// Created by Ludger Heide on 27.03.18.
//

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START    unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif

#ifndef _unused
#define _unused(x) ((void)x)
#endif


#include <stdbool.h>
#include <assert.h>
#include <util/delay.h>
#include <string.h>
#include "bno055.h"
#include "../avrlib/i2c.h"
#include "../avrlib/timer.h"

static u08 bno055Read8(u08 reg) {
    u08 i2cstat = i2cMasterSendNI(BNO055_ADDRESS, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte;
    i2cstat = i2cMasterReceiveNI(BNO055_ADDRESS, 1, &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    return outByte;
}

static void bno055Write8(u08 reg, u08 value) {
    //Combine reg and value in a 16 byt variable
    u08 transmission[2] = {reg, value};

    u08 i2cstat = i2cMasterSendNI(BNO055_ADDRESS, 2, transmission);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);
}

static void bno055Read48(u08 reg, u08 *outBytes) {
    u08 i2cstat = i2cMasterSendNI(BNO055_ADDRESS, 1, &reg);
    assert(i2cstat == I2C_OK);

    i2cstat = i2cMasterReceiveNI(BNO055_ADDRESS, 6, outBytes);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);
}


static void bno055SetMode(bno055_opmode_t mode) {
    bno055Write8(BNO055_OPR_MODE_ADDR, mode);
    _delay_ms(30);
}

bool bno055Init(void) {
    //Check if it is responding on the correct address
    u08 chipId = bno055Read8(BNO055_CHIP_ID_ADDR);
    if (chipId != BNO055_ID) {
        //_delay_ms(250); // hold on for boot
        chipId = bno055Read8(BNO055_CHIP_ID_ADDR);
        if (bno055Read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            //return false;  // still not? ok bail
        }
    }

    /* Switch to config mode (just in case since this is the default) */
    bno055SetMode(OPERATION_MODE_CONFIG);

    /* Reset */
    //We can't use the default write8 because the reset means the I2c bus is not exited clearly
    u08 transmission[2] = {BNO055_SYS_TRIGGER_ADDR, 0x20};
    i2cMasterSendNI(BNO055_ADDRESS, 2, (u08 *) &transmission);
    u08 i2cstat;
    while (i2cstat != I2C_OK) {
        _delay_ms(10);
        i2cstat = i2cMasterSendNI(BNO055_ADDRESS, 1, BNO055_CHIP_ID_ADDR);
        if (i2cstat != I2C_OK) {
            continue;
        } else {
            u08 outByte;
            i2cstat = i2cMasterReceiveNI(BNO055_ADDRESS, 1, &outByte);
        }
    }
    _delay_ms(50);

    /* Set to normal power mode */
    bno055Write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    _delay_ms(10);

    /* Set the output units */
    /* See table 3-11 of the datasheet */
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                      (0 << 4) | // Temperature = Celsius
                      (0 << 2) | // Euler = Degrees
                      (1 << 1) | // Gyro = Rad/s
                      (0 << 0);  // Accelerometer = m/s^2
    bno055Write8(BNO055_UNIT_SEL_ADDR, unitsel);


    /* Configure axis mapping (see section 3.4) */
    /*
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
    delay(10);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
    delay(10);
    */

    //Clear all reset and self-test triggers
    bno055Write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    _delay_ms(10);

    //Set it to NDOF mode
    bno055SetMode(OPERATION_MODE_NDOF);
    _delay_ms(20);

    return true;
}

void bno055GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error) {
    //Switch to configuration data page
    bno055Write8(BNO055_PAGE_ID_ADDR, 0);

    /* System Status (see section 4.3.58)
       ---------------------------------
       0 = Idle
       1 = System Error
       2 = Initializing Peripherals
       3 = System Iniitalization
       4 = Executing Self-Test
       5 = Sensor fusio algorithm running
       6 = System running without fusion algorithms */
    *system_status = bno055Read8(BNO055_SYS_STAT_ADDR);

    /* Self Test Results (see section )
       --------------------------------
       1 = test passed, 0 = test failed
       Bit 0 = Accelerometer self test
       Bit 1 = Magnetometer self test
       Bit 2 = Gyroscope self test
       Bit 3 = MCU self test
       0x0F = all good! */
    *self_test_result = bno055Read8(BNO055_SELFTEST_RESULT_ADDR);

    /* System Error (see section 4.3.59)
       ---------------------------------
       0 = No error
       1 = Peripheral initialization error
       2 = System initialization error
       3 = Self test result failed
       4 = Register map value out of range
       5 = Register map address out of range
       6 = Register map write error
       7 = BNO low power mode not available for selected operat ion mode
       8 = Accelerometer power mode not available
       9 = Fusion algorithm configuration error
       A = Sensor configuration error */
    *system_error = bno055Read8(BNO055_SYS_ERR_ADDR);
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
void bno055GetRevInfo(bno055_rev_info_t *info) {
    uint8_t a, b;

    memset(info, 0, sizeof(bno055_rev_info_t));

    /* Check the accelerometer revision */
    info->accel_rev = bno055Read8(BNO055_ACCEL_REV_ID_ADDR);

    /* Check the magnetometer revision */
    info->mag_rev = bno055Read8(BNO055_MAG_REV_ID_ADDR);

    /* Check the gyroscope revision */
    info->gyro_rev = bno055Read8(BNO055_GYRO_REV_ID_ADDR);

    /* Check the SW revision */
    info->bl_rev = bno055Read8(BNO055_BL_REV_ID_ADDR);

    a = bno055Read8(BNO055_SW_REV_ID_LSB_ADDR);
    b = bno055Read8(BNO055_SW_REV_ID_MSB_ADDR);
    info->sw_rev = (((uint16_t) b) << 8) | ((uint16_t) a);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void bno055GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) {
    uint8_t calData = bno055Read8(BNO055_CALIB_STAT_ADDR);
    if (sys != NULL) {
        *sys = (calData >> 6) & 0x03;
    }
    if (gyro != NULL) {
        *gyro = (calData >> 4) & 0x03;
    }
    if (accel != NULL) {
        *accel = (calData >> 2) & 0x03;
    }
    if (mag != NULL) {
        *mag = calData & 0x03;
    }
}


void bno055GetDataBlocking(attitude_struct *attitude, xyz *angularVelocity, xyz *gravity, xyz *linearAcceleration) {
    u08 buffer[6];
    s16 x, y, z;

    u64 now = micros64();

    //Get the attitude(euler angles)
    /* 1 degree = 16 LSB  means that we must multiply by 4 to get to our usueal values*/
    bno055Read48(VECTOR_EULER, buffer);
    x = ((s16) buffer[0]) | (((s16) buffer[1]) << 8);
    y = ((s16) buffer[2]) | (((s16) buffer[3]) << 8);
    z = ((s16) buffer[4]) | (((s16) buffer[5]) << 8);

    attitude->timestamp = now;
    attitude->courseMagnetic = x * 4;
    attitude->pitch = y * 4;
    attitude->roll = z * 4;

    //Get the angular velocity (rad/s)
    /* 1dps = 16 LSB meaning (rad/s) * 16 */
    bno055Read48(VECTOR_GYROSCOPE, buffer);
    x = ((s16) buffer[0]) | (((s16) buffer[1]) << 8);
    y = ((s16) buffer[2]) | (((s16) buffer[3]) << 8);
    z = ((s16) buffer[4]) | (((s16) buffer[5]) << 8);

    angularVelocity->timestamp = now;
    angularVelocity->x = x;
    angularVelocity->y = y;
    angularVelocity->z = z;

    //Get the gravity vector (m/s^2)
    //* 1m/s^2 = 100 LSB meaning cm/s^2 */
    bno055Read48(VECTOR_GRAVITY, buffer);
    x = ((s16) buffer[0]) | (((s16) buffer[1]) << 8);
    y = ((s16) buffer[2]) | (((s16) buffer[3]) << 8);
    z = ((s16) buffer[4]) | (((s16) buffer[5]) << 8);

    gravity->timestamp = now;
    gravity->x = x;
    gravity->y = y;
    gravity->z = z;

    //Get the linear acceleration vector (m/s^2)
    //* 1m/s^2 = 100 LSB meaning cm/s^2 */
    bno055Read48(VECTOR_LINEARACCEL, buffer);
    x = ((s16) buffer[0]) | (((s16) buffer[1]) << 8);
    y = ((s16) buffer[2]) | (((s16) buffer[3]) << 8);
    z = ((s16) buffer[4]) | (((s16) buffer[5]) << 8);

    linearAcceleration->timestamp = now;
    linearAcceleration->x = x;
    linearAcceleration->y = y;
    linearAcceleration->z = z;
}