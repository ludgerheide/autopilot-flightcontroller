//
// Created by ludger on 05.01.18.
//
// Sourced from adafruit's axcellent BMP280 library: https://github.com/adafruit/Adafruit_BMP280_Library

#ifndef FLIGHTCONTROLLER_BMP280_H
#define FLIGHTCONTROLLER_BMP280_H

#include <stdio.h>
#include "../avrlib/avrlibtypes.h"
#include "bmp180.h"

/*=========================================================================
    I2C ADDRESS/BITS/SETTINGS
    -----------------------------------------------------------------------*/
#define BMP280_ADDRESS1               (0x77)
#define BMP280_ADDRESS2               (0x76)
#define BMP280_CHIPID                 (0x58)
/*=========================================================================*/


/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
enum {
    BMP280_REGISTER_DIG_T1 = 0x88,
    BMP280_REGISTER_DIG_T2 = 0x8A,
    BMP280_REGISTER_DIG_T3 = 0x8C,

    BMP280_REGISTER_DIG_P1 = 0x8E,
    BMP280_REGISTER_DIG_P2 = 0x90,
    BMP280_REGISTER_DIG_P3 = 0x92,
    BMP280_REGISTER_DIG_P4 = 0x94,
    BMP280_REGISTER_DIG_P5 = 0x96,
    BMP280_REGISTER_DIG_P6 = 0x98,
    BMP280_REGISTER_DIG_P7 = 0x9A,
    BMP280_REGISTER_DIG_P8 = 0x9C,
    BMP280_REGISTER_DIG_P9 = 0x9E,

    BMP280_REGISTER_CHIPID = 0xD0,
    BMP280_REGISTER_VERSION = 0xD1,
    BMP280_REGISTER_SOFTRESET = 0xE0,

    BMP280_REGISTER_CAL26 = 0xE1,  // R calibration stored in 0xE1-0xF0

    BMP280_REGISTER_STATUS = 0xF3,
    BMP280_REGISTER_CONTROL = 0xF4,
    BMP280_REGISTER_CONFIG = 0xF5,
    BMP280_REGISTER_PRESSUREDATA = 0xF7,
    BMP280_REGISTER_TEMPDATA = 0xFA,
};

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} BMP280_calib_data;
/*=========================================================================*/

/*=========================================================================
 MODE SETTINGS
 -----------------------------------------------------------------------*/
typedef enum {
    BMP280_MODE_OFF = 0,
    BMP280_MODE_1X = 1,
    BMP280_MODE_2X = 2,
    BMP280_MODE_4X = 3,
    BMP280_MODE_8X = 4,
    BMP280_MODE_16X = 5
} BMP280_mode_t;
/*=========================================================================*/

typedef enum {
    BMP280_IDLE,
    BMP280_MEASURING,
    BMP280_READY,
    BMP280_RECEIVING,
} bmp280State;

typedef struct {
    u64 timestamp; //Microseconds

    u32 speed;
} airspeed_struct;


typedef struct {
    u08 address;
    volatile bmp280State state;
    volatile u32 lastStateChange;
    BMP280_calib_data calib_data;
    //Default to "high-res mode", according to page 18 of the datasheet it's
    BMP280_mode_t temperature_mode;
    BMP280_mode_t pressure_mode;
    u08 measurement_time;
    volatile s32 adc_P;
    volatile s32 adc_T;
    volatile u32 timestamp;
} bmp280_configuration;

bmp280_configuration bmp280_0x76;

//Returns a default bmp280 configuration
bmp280_configuration bmp280_defaults(void);

//Returns true if the initializationw as successfull, false otherwise
BOOL bmp280Init(bmp280_configuration *configuration);

//Start the capture
void bmp280StartCapture(bmp280_configuration *configuration);

//Start Receiving data
void bmp280StartReceiving(bmp280_configuration *configuration);

//Copy the data out of the receiving buffer into our struct
void bmp280GetDataFromI2cBuffer(bmp280_configuration *configuration);

//Gets the altitude from the bmp
void bmp280GetData(pressureEvent *myEvent, bmp280_configuration *configuration);

// Measures the pressure compensation
// ALERT: Blocks for ~500ms, do not use inflight
void calibratePressureCompensation(void);

//Calculates airspeed
void calculateAirspeed(pressureEvent *pitotPressure, pressureEvent *staticPressure, airspeed_struct *airspeed);

#endif //FLIGHTCONTROLLER_BMP280_H
