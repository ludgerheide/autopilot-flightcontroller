//
//  bmp.c
//  avr - flyByWire
//
//  Created by Ludger Heide on 27.09.15.
//  Copyright © 2015 LH Technologies. All rights reserved.
//

#include "bmp180.h"
#include "../avrlib/i2c.h"

#include <assert.h>
#include <avr/io.h>
#include <math.h>
#include "../avrlib/timer.h"

#ifndef _unused
#define _unused(x) ((void)x)
#endif

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START    unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif

/*=========================================================================
 CALIBRATION DATA
 -----------------------------------------------------------------------*/
typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} BMP180_calib_data;
/*=========================================================================*/

//Variables
static BMP180_calib_data calibData;
bmp180State myBmp180State;
volatile tempPressRawData myBmp180RawData;
static const BMP180_mode_t selectedMode = BMP180_MODE_ULTRAHIGHRES;
u32 bmp180LastStateChange;

//static void bmpWrite8(u08 reg, u08 value) {
//    //Combine reg and value in a 16 byt variable
//    u08 transmission[2] = {reg, value};
//    
//    u08 i2cstat = i2cMasterSendNI(BMP180_ADDRESS, 2, (u08*)&transmission);
//    assert(i2cstat == I2C_OK);
//}

static u08 bmp180Read8(u08 reg) {
    u08 i2cstat = i2cMasterSendNI(BMP180_ADDRESS, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte;
    i2cstat = i2cMasterReceiveNI(BMP180_ADDRESS, 1, &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    return outByte;
}

static u16 bmp180ReadU16(u08 reg) {
    u08 i2cstat = i2cMasterSendNI(BMP180_ADDRESS, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte[2];
    i2cstat = i2cMasterReceiveNI(BMP180_ADDRESS, 2, (u08 *) &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    //Combine the 8-bit values into one 16-bit value
    return ((outByte[0] << 8) | outByte[1]);
}

static s16 bmp180ReadS16(u08 reg) {
    u08 i2cstat = i2cMasterSendNI(BMP180_ADDRESS, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte[2];
    i2cstat = i2cMasterReceiveNI(BMP180_ADDRESS, 2, (u08 *) &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    //Combine the 8-bit values into one 16-bit value
    return ((outByte[0] << 8) | outByte[1]);
}

static void readCoefficients(void) {
    calibData.ac1 = bmp180ReadS16(BMP180_REGISTER_CAL_AC1);
    calibData.ac2 = bmp180ReadS16(BMP180_REGISTER_CAL_AC2);
    calibData.ac3 = bmp180ReadS16(BMP180_REGISTER_CAL_AC3);

    calibData.ac4 = bmp180ReadU16(BMP180_REGISTER_CAL_AC4);
    calibData.ac5 = bmp180ReadU16(BMP180_REGISTER_CAL_AC5);
    calibData.ac6 = bmp180ReadU16(BMP180_REGISTER_CAL_AC6);

    calibData.b1 = bmp180ReadS16(BMP180_REGISTER_CAL_B1);
    calibData.b2 = bmp180ReadS16(BMP180_REGISTER_CAL_B2);
    calibData.mb = bmp180ReadS16(BMP180_REGISTER_CAL_MB);
    calibData.mc = bmp180ReadS16(BMP180_REGISTER_CAL_MC);
    calibData.md = bmp180ReadS16(BMP180_REGISTER_CAL_MD);
}

static int32_t computeB5(int32_t ut) {
    int32_t X1 = (ut - (int32_t) calibData.ac6) * ((int32_t) calibData.ac5) >> 15;
    int32_t X2 = ((int32_t) calibData.mc << 11) / (X1 + (int32_t) calibData.md);
    return X1 + X2;
}

BOOL bmp180Init() {
    u08 deviceId = bmp180Read8(BMP180_REGISTER_CHIPID);
    if (deviceId != 0x55) {
        return FALSE;
    }

    readCoefficients();

    myBmp180State = BMP180_IDLE;

    return TRUE;
}

//Start the temp capture
void bmp180StartTemperatureCapture(void) {
    const u08 toSend[2] = {BMP180_REGISTER_CONTROL, BMP180_REGISTER_READTEMPCMD};
    i2cMasterSend(BMP180_ADDRESS, 2, (u08 *) &toSend);
}

//Send the "get temperature" request
void bmp180StartReceivingTemperature(void) {
    assert(myBmp180State == BMP180_TEMPERATURE_READY);
    u08 toSend = BMP180_REGISTER_TEMPDATA;
    i2cMasterSend(BMP180_ADDRESS, 1, &toSend);
}

void bmp180StartPressureCapture(void) {
    const u08 toSend[2] = {BMP180_REGISTER_CONTROL, BMP180_REGISTER_READPRESSURECMD_UHR};
    i2cMasterSend(BMP180_ADDRESS, 2, (u08 *) &toSend);
}

//Send the "get pressure" request
void bmp180StartReceivingPressure(void) {
    assert(myBmp180State == BMP180_PRESSURE_READY);
    u08 pressReadCommand = BMP180_REGISTER_PRESSUREDATA;
    i2cMasterSend(BMP180_ADDRESS, 1, &pressReadCommand);
}

//Copy the data out of the receiving buffer into our struct
void bmp180GetPressDataFromI2cBuffer(void) {
    //The buffer should be three bytes long and full
    assert(I2cReceiveDataLength == 3);
    assert(I2cReceiveDataIndex == 3);

    myBmp180RawData.presshi = I2cReceiveData[0];
    myBmp180RawData.presslo = I2cReceiveData[1];
    myBmp180RawData.pressxlo = I2cReceiveData[2];

    myBmp180RawData.timestamp = micros();
}

void bmp180GetTempDataFromI2cBuffer(void) {
    //The buffer should be two bytes long and full
    assert(I2cReceiveDataLength == 2);
    assert(I2cReceiveDataIndex == 2);

    myBmp180RawData.temphi = I2cReceiveData[0];
    myBmp180RawData.templo = I2cReceiveData[1];
}

//Gets the altitude from the bmp
void bmp180GetData(pressureEvent *myEvent) {
    // ----- DON'T ASK HOW THIS IS DONE! -----

    CRITICAL_SECTION_START;
    if (myEvent->timestamp == myBmp180RawData.timestamp) {
        CRITICAL_SECTION_END;
        return;
    }
    myEvent->timestamp = myBmp180RawData.timestamp;

    //Maybe: Generate raw pressure int
    s32 rawPress;
    rawPress = ((u32) myBmp180RawData.presshi << 16) | ((u32) myBmp180RawData.presslo << 8) |
               ((u32) myBmp180RawData.pressxlo);
    rawPress = rawPress >> (8 - selectedMode);

    //Maybe: Generate raw temperature int
    s32 rawTemp = ((myBmp180RawData.temphi) << 8) | (myBmp180RawData.templo);
    CRITICAL_SECTION_END;

    int32_t ut = 0, up = 0, compp = 0;
    int32_t x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;

    ut = rawTemp;
    up = rawPress;

    /* Temperature compensation */
    b5 = computeB5(ut);

    /* Pressure compensation */
    b6 = b5 - 4000;
    x1 = (calibData.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (calibData.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) calibData.ac1) * 4 + x3) << selectedMode) + 2) >> 2;
    x1 = (calibData.ac3 * b6) >> 13;
    x2 = (calibData.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (calibData.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (up - b3) * (50000 >> selectedMode));

    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    myEvent->pressure = compp / 100.0; //Divide by 100 to convert to hPa

    //Temperature output (?)
    int32_t UT, B5;     // following ds convention
    float t;

    UT = rawTemp;

    B5 = computeB5(UT);
    t = (B5 + 8) >> 4;
    t /= 10;

    myEvent->temperature = t;
}

float pressureToAltitude(float atmosphericPressure, float theSeaLevelPressure) {
    // Equation taken from BMP180 datasheet (page 16):
    //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

    // Note that using the equation from wikipedia can give bad results
    // at high altitude.  See this thread for more information:
    //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

    return 44330.0 * (1.0 - pow(atmosphericPressure / theSeaLevelPressure, 0.1903));
}