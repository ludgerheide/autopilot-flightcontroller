//
//  bmp.h
//  avr - flyByWire
//
//  Created by Ludger Heide on 27.09.15.
//  Copyright © 2015 LH Technologies. All rights reserved.
//

#ifndef bmp180_h
#define bmp180_h

#include <stdio.h>
#include "../avrlib/avrlibtypes.h"

/*=========================================================================
 I2C ADDRESS/BITS
 -----------------------------------------------------------------------*/
#define BMP180_ADDRESS                (0x77)
/*=========================================================================*/

#define BMP180_TEMPERATURE_DURATION 5
#define BMP180_PRESSURE_DURATION 26

/*=========================================================================
 REGISTERS
 -----------------------------------------------------------------------*/
enum {
    BMP180_REGISTER_CAL_AC1 = 0xAA,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_AC2 = 0xAC,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_AC3 = 0xAE,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_AC4 = 0xB0,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_AC5 = 0xB2,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_AC6 = 0xB4,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_B1 = 0xB6,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_B2 = 0xB8,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_MB = 0xBA,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_MC = 0xBC,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CAL_MD = 0xBE,  // R   Calibration data (16 bits)
    BMP180_REGISTER_CHIPID = 0xD0,
    BMP180_REGISTER_VERSION = 0xD1,
    BMP180_REGISTER_SOFTRESET = 0xE0,
    BMP180_REGISTER_CONTROL = 0xF4,
    BMP180_REGISTER_TEMPDATA = 0xF6,
    BMP180_REGISTER_PRESSUREDATA = 0xF6,
    BMP180_REGISTER_READTEMPCMD = 0x2E,
    BMP180_REGISTER_READPRESSURECMD = 0x34,
    BMP180_REGISTER_READPRESSURECMD_UHR = 0xF4   //Ultra High-res
};
/*=========================================================================*/

//Structs
/*=========================================================================
 MODE SETTINGS
 -----------------------------------------------------------------------*/
typedef enum {
    BMP180_MODE_ULTRALOWPOWER = 0,
    BMP180_MODE_STANDARD = 1,
    BMP180_MODE_HIGHRES = 2,
    BMP180_MODE_ULTRAHIGHRES = 3
} BMP180_mode_t;
/*=========================================================================*/

typedef struct {
    u64 timestamp; //microseconds

    s32 temperature; //Celsius * 100
    u32 pressure;  //Pa * 256
} pressureEvent;

typedef struct tempPressRawData {
    u08 templo;
    u08 temphi;
    u08 presshi;
    u08 presslo;
    u08 pressxlo;
    u64 timestamp;
} tempPressRawData;

typedef struct altitudeData {
    u64 timestamp; //Microseconds

    s32 altitude; //centimeters (relative to barometer calibration)
    s32 rate_of_climb; //cm/s
} altitudeData;

typedef enum {
    BMP180_IDLE,
    BMP180_TEMPERATURE_MEASURING,
    BMP180_TEMPERATURE_READY,
    BMP180_TEMPERATURE_RECEIVING,
    BMP180_PRESSURE_MEASURING,
    BMP180_PRESSURE_READY,
    BMP180_PRESSURE_RECEIVING,
} bmp180State;

typedef struct seaLevelPressure {
    u64 timestamp;
    float slp;
} seaLevelPressure_struct;

//Variables
extern bmp180State myBmp180State;
extern volatile tempPressRawData myBmp180RawData;
extern u32 bmp180LastStateChange;

seaLevelPressure_struct mySeaLevelPressure;

//Initialize the BMP
//Returns true if the initializationw as successfull, false otherwise
BOOL bmp180Init(void);

//Start the temperature capture
void bmp180StartTemperatureCapture(void);

//Send the "get temperature" request
void bmp180StartReceivingTemperature(void);

//Start the pressure capture
void bmp180StartPressureCapture(void);

//Send the "get pressure" request
void bmp180StartReceivingPressure(void);

//Copy the data out of the receiving buffer into our struct
void bmp180GetPressDataFromI2cBuffer(void);

//Copy the data out of the receiving buffer into our struct
void bmp180GetTempDataFromI2cBuffer(void);

//Gets the altitude from the bmp
void bmp180GetData(pressureEvent *myEvent);

//Updates the altitude struct
void updateAltitudeData(pressureEvent *staticPressure, altitudeData *myAltitudeData);

#endif /* bmp180_h */