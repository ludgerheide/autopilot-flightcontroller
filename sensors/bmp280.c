//
// Created by ludger on 05.01.18.
//

#include "bmp280.h"
#include "../avrlib/i2c.h"
#include "../avrlib/timer.h"
#include "../utils.h"

#include <assert.h>
#include <math.h>

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START    unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif

#ifndef _unused
#define _unused(x) ((void)x)
#endif

//Stores the pressure difference (static - pitot) for zero airspeed
static s32 pitotPressureDifference;

//Filter coefficients
static u32 sumPressure;
static const u08 alpha = 2; //Increase to make filter answer slower, decrease to make it faster


static u08 bmp280Read8(u08 reg, bmp280_configuration *configuration) {
    assert(configuration->address == BMP280_ADDRESS1 || configuration->address == BMP280_ADDRESS2);
    u08 i2cstat = i2cMasterSendNI(configuration->address, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte;
    i2cstat = i2cMasterReceiveNI(configuration->address, 1, &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    return outByte;
}

static u16 bmp280ReadU16(u08 reg, bmp280_configuration *configuration) {
    assert(configuration->address == BMP280_ADDRESS1 || configuration->address == BMP280_ADDRESS2);
    u08 i2cstat = i2cMasterSendNI(configuration->address, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte[2];
    i2cstat = i2cMasterReceiveNI(configuration->address, 2, (u08 *) &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    //Combine the 8-bit values into one 16-bit value
    return ((outByte[1] << 8) | outByte[0]);
}

static s16 bmp280ReadS16(u08 reg, bmp280_configuration *configuration) {
    assert(configuration->address == BMP280_ADDRESS1 || configuration->address == BMP280_ADDRESS2);
    u08 i2cstat = i2cMasterSendNI(configuration->address, 1, &reg);
    assert(i2cstat == I2C_OK);

    u08 outByte[2];
    i2cstat = i2cMasterReceiveNI(configuration->address, 2, (u08 *) &outByte);
    assert(i2cstat == I2C_OK);
    _unused(i2cstat);

    //Combine the 8-bit values into one 16-bit value
    return ((outByte[1] << 8) | outByte[0]);
}

static void readCoefficients(bmp280_configuration *configuration) {
    configuration->calib_data.dig_T1 = bmp280ReadU16(BMP280_REGISTER_DIG_T1, configuration);
    configuration->calib_data.dig_T2 = bmp280ReadS16(BMP280_REGISTER_DIG_T2, configuration);
    configuration->calib_data.dig_T3 = bmp280ReadS16(BMP280_REGISTER_DIG_T3, configuration);

    configuration->calib_data.dig_P1 = bmp280ReadU16(BMP280_REGISTER_DIG_P1, configuration);
    configuration->calib_data.dig_P2 = bmp280ReadS16(BMP280_REGISTER_DIG_P2, configuration);
    configuration->calib_data.dig_P3 = bmp280ReadS16(BMP280_REGISTER_DIG_P3, configuration);
    configuration->calib_data.dig_P4 = bmp280ReadS16(BMP280_REGISTER_DIG_P4, configuration);
    configuration->calib_data.dig_P5 = bmp280ReadS16(BMP280_REGISTER_DIG_P5, configuration);
    configuration->calib_data.dig_P6 = bmp280ReadS16(BMP280_REGISTER_DIG_P6, configuration);
    configuration->calib_data.dig_P7 = bmp280ReadS16(BMP280_REGISTER_DIG_P7, configuration);
    configuration->calib_data.dig_P8 = bmp280ReadS16(BMP280_REGISTER_DIG_P8, configuration);
    configuration->calib_data.dig_P9 = bmp280ReadS16(BMP280_REGISTER_DIG_P9, configuration);
}

//COPY-PASTED FROM THE MANUAL - I DON'T KNOW WHAT IT DOES
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
static s32 t_fine;

static s32 bmp280_compensate_T_int32(s32 adc_T, bmp280_configuration *configuration) {
    s32 var1, var2, T;
    var1 = ((((adc_T >> 3) - ((s32) configuration->calib_data.dig_T1 << 1))) * ((s32) configuration->calib_data.dig_T2))
            >> 11;
    var2 = (((((adc_T >> 4) - ((s32) configuration->calib_data.dig_T1)) *
              ((adc_T >> 4) - ((s32) configuration->calib_data.dig_T1))) >> 12) *
            ((s32) configuration->calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

//COPY-PASTED FROM THE MANUAL - I DON'T KNOW WHAT IT DOES
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static u32 bmp280_compensate_P_int64(s32 adc_P, bmp280_configuration *configuration) {
    s64 var1, var2, p;
    var1 = ((s64) t_fine) - 128000;
    var2 = var1 * var1 * (s64) configuration->calib_data.dig_P6;
    var2 = var2 + ((var1 * (s64) configuration->calib_data.dig_P5) << 17);
    var2 = var2 + (((s64) configuration->calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (s64) configuration->calib_data.dig_P3) >> 8) +
           ((var1 * (s64) configuration->calib_data.dig_P2) << 12);
    var1 = (((((s64) 1) << 47) + var1)) * ((s64) configuration->calib_data.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((s64) configuration->calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((s64) configuration->calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((s64) configuration->calib_data.dig_P7) << 4);
    return (u32) p;
}

//Returns a default bmp280 configuration
bmp280_configuration bmp280_defaults(void) {
    bmp280_configuration config = {.address = BMP280_ADDRESS2,
            .state = BMP280_IDLE,
            .lastStateChange = 0,
            //Default to "high-res mode", according to page 18 of the datasheet it's
            .temperature_mode = BMP280_MODE_2X,
            .pressure_mode = BMP280_MODE_16X,
            .measurement_time = 39};
    return config;
}

BOOL bmp280Init(bmp280_configuration *configuration) {
    u08 deviceId = bmp280Read8(BMP280_REGISTER_CHIPID, configuration);
    if (deviceId != BMP280_CHIPID) {
        return FALSE;
    }
    pitotPressureDifference = readPressureCompensationFromEEPROM();

    readCoefficients(configuration);

    configuration->state = BMP280_IDLE;

    return TRUE;
}

//Start the capture
void bmp280StartCapture(bmp280_configuration *configuration) {
    //We write to the control register:
    //A "forced mode" status of 0x01
    //Our chosen oversamplings
    const u08 toSend[2] = {BMP280_REGISTER_CONTROL, ((u08) 0x01 | (u08) configuration->pressure_mode << 2 |
                                                     (u08) configuration->temperature_mode << 5)};
#ifdef BMP280_DEBUG
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

    printf("Sending "BYTE_TO_BINARY_PATTERN" to register 0x%x\r\n", BYTE_TO_BINARY(toSend[1]), toSend[0]);
#endif


    i2cMasterSend(configuration->address, 2, (u08 *) &toSend);
}

//Send the "get data" request
void bmp280StartReceiving(bmp280_configuration *configuration) {
    assert(configuration->state == BMP280_READY);
    u08 toSend = BMP280_REGISTER_PRESSUREDATA;
    i2cMasterSend(configuration->address, 1, &toSend);
}

//Copy the data out of the receiving buffer into our struct
void bmp280GetDataFromI2cBuffer(bmp280_configuration *configuration) {
    //The buffer should be six bytes long and full
    assert(I2cReceiveDataLength == 6);
    assert(I2cReceiveDataIndex == 6);

    configuration->adc_P = I2cReceiveData[0];
    configuration->adc_P <<= 8;
    configuration->adc_P |= I2cReceiveData[1];
    configuration->adc_P <<= 8;
    configuration->adc_P |= I2cReceiveData[2];

    configuration->adc_T = I2cReceiveData[3];
    configuration->adc_T <<= 8;
    configuration->adc_T |= I2cReceiveData[4];
    configuration->adc_T <<= 8;
    configuration->adc_T |= I2cReceiveData[5];

#ifdef BMP280_DEBUG
    printf_P(PSTR("Raw data is %02x, %02x, %02x, temp: %02x, %02x, %02x\r\n"), I2cReceiveData[0], I2cReceiveData[1], I2cReceiveData[2],I2cReceiveData[3], I2cReceiveData[4],I2cReceiveData[5]);
    printf_P(PSTR("Actual: %08lx, %08lx\r\n"), configuration->adc_T, configuration->adc_P);
#endif

    configuration->timestamp = micros64();
}

//Gets the altitude from the bmp
void bmp280GetData(pressureEvent *myEvent, bmp280_configuration *configuration) {
    CRITICAL_SECTION_START;
    if (myEvent->timestamp == configuration->timestamp) {
        CRITICAL_SECTION_END;
        return;
    }
    //Copy the data from the buffer to temp variables in a critical section so it doesn't get updated by the ISR in between
    //See datasheet page 26 for shifts etc
#ifdef BMP280_DEBUG
    //printf("Raw data is %x %x %x %x %x %x\r\n", configuration->pressure_data[0], configuration->pressure_data[1], configuration->pressure_data[2], configuration->temperature_data[0], configuration->temperature_data[1], configuration->temperature_data[2]);
#endif
    s32 adc_P_temp = configuration->adc_P >> 4;
    s32 adc_T_temp = configuration->adc_T >> 4;

    myEvent->timestamp = configuration->timestamp;
    CRITICAL_SECTION_END;

    myEvent->temperature = bmp280_compensate_T_int32(adc_T_temp, configuration);

    //Get the "raw" new pressure
    u32 newPressure = bmp280_compensate_P_int64(adc_P_temp, configuration);
    u32 curPressure = myEvent->pressure;  //Current filter output

    sumPressure = sumPressure - curPressure + newPressure;
    myEvent->pressure = (sumPressure + (1 << (alpha - 1))) >> (alpha);
}

// Measures the pressure compensation
// ALERT: Blocks for ~5000ms, do not use inflight
void calibratePressureCompensation(void) {
    //Since this takes so long,w e need to change the watchdog timeout
    wdt_disable();
    wdt_enable(WDTO_8S);

    const u08 samplingCount = 100;
    u32 pitotSamples[samplingCount];
    u32 staticSamples[samplingCount];

    pressureEvent pitotEvent, staticEvent;
    s08 pitotCount = -1; //Disregard the first one sample, because it can be strange after a reset
    s08 staticCount = -1;
    do {
        u64 lastPitot = pitotEvent.timestamp;
        u64 lastStatic = staticEvent.timestamp;

        bmp280GetData(&pitotEvent, &bmp280_0x76);
        bmp180GetData(&staticEvent);

        if (pitotEvent.timestamp != lastPitot && pitotCount < samplingCount) {
            pitotSamples[pitotCount] = pitotEvent.pressure;
            pitotCount++;
        }

        if (staticEvent.timestamp != lastStatic && staticCount < samplingCount) {
            staticSamples[staticCount] = staticEvent.pressure;
            staticCount++;
        }
    } while (pitotCount < samplingCount || staticCount < samplingCount);

    //Calculate the average of each and the difference
    u64 pitotAvg = 0;
    u64 staticAvg = 0;
    for (u08 i = 0; i < samplingCount; i++) {
        pitotAvg += pitotSamples[i];
        staticAvg += staticSamples[i];
    }

    //Average, difference
    pitotAvg /= samplingCount;
    staticAvg /= samplingCount;

    pitotPressureDifference = (s32) (staticAvg - pitotAvg);
    writePressureCompensationToEEPROM(pitotPressureDifference);

    //reenable the watchdog with the proper timeout;
    wdt_disable();
    wdt_enable(WDTO_DEFAULT);
}

void calculateAirspeed(pressureEvent *pitotPressure, pressureEvent *staticPressure, airspeed_struct *airspeed) {
    //Since the event comes from two different sensors, its timestamp shall be the average of those
    u64 timestamp = (pitotPressure->timestamp + staticPressure->timestamp) / 2;

    if (airspeed->timestamp == timestamp) {
        return;
    }

    //Calculated using the ideal gas law https://en.wikipedia.org/wiki/Density_of_air
    //We use the pitot tube's temperature since the static sensor actually gets quite warm in use
    float rho = (staticPressure->pressure / 256.0) / (287.058 * ((pitotPressure->temperature / 100.0) + 273.15));

    //equtation from https://en.wikipedia.org/wiki/Pitot_tube, adapted a little
    //We would have to diveide pred
    u32 v = 100 *
            sqrt(2 * (((s64) pitotPressure->pressure + pitotPressureDifference) - (s64) staticPressure->pressure) /
                 (256 * rho));

    airspeed->timestamp = timestamp;
    airspeed->speed = v;
}