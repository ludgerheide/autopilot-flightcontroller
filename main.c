

#include <avr/io.h>
#include <avr/wdt.h>
#include "sensors/gyro.h"
#include "sensors/accelMag.h"
#include "sensors/bmp180.h"

#include "utils.h"
#include "avrlib/timer.h"
#include "sensors/gps.h"
#include "sensors/MadgwickAHRS.h"
#include "comms/communicationsHandler.h"
#include "comms/raspiComms.h"
#include "avrlib/servo.h"

#include "sensors/i2cperipherals.h"
#include "control/flightController.h"

int main(void) __attribute__ ((noreturn));

static void initIO(void) {
    wdt_enable(WDTO_250MS);
    raspiInit();
    printfAttachToUart();
    readSlpFromEEPROM();
    readHomeBaseFromEEPROM();
    servoInit();
    timerInit();
    IMUinit();
    gpsInit();
    commsInit();
    batteryInit();
    flightControllerInit();
    wdt_disable();
    wdt_enable(WDTO_DEFAULT);
}

//Sensor update rates
const u32 periodGyro = 1000000 / 190; //Microseconds
const u32 periodAccel = 1000000 / 200;
const u32 periodMag = 1000000 / 220;
const u32 periodStatic = 1000 * (BMP180_TEMPERATURE_DURATION + BMP180_PRESSURE_DURATION);
const u32 periodPitot = (u32) 1000 *
                        39; //Constant value because we can't use the value from the config definition because it isn't const itself.

static void updateSensors(void) {
    u32 now = micros();

    //Get the current sensor events and validate their times, otherwise, reste i2c and exit from the method
    if (theFlags.gyroEnabled) {
        gyroGetData(&curGyro);
        if ((u32) curGyro.timestamp < now && now - (u32) curGyro.timestamp > 2 * periodGyro) {
            doReset();
        }
    }

    if (theFlags.accelEnabled) {
        accelGetData(&curAccel);
        if ((u32) curAccel.timestamp < now && now - (u32) curAccel.timestamp > 2 * periodAccel) {
            doReset();
        }
    }

    if (theFlags.magEnabled) {
        magGetData(&uncompensatedMag);
        if ((u32) uncompensatedMag.timestamp < now && now - (u32) uncompensatedMag.timestamp > 4 *
                                                                                               periodMag) {
            //4X Because the magnetometer likes to skip a sample every now and then and I don't really care about that
            doReset();
        }

        magCompensate(&uncompensatedMag, &curMag);
    }

    if (theFlags.accelEnabled && theFlags.gyroEnabled) {
        //Update the madgwick algorithm
        if (theFlags.magEnabled) {
            MadgwickAHRSupdate(micros(), curGyro.x, curGyro.y, curGyro.z, curAccel.x, curAccel.y, curAccel.z, curMag.x,
                               curMag.y, curMag.z);
        } else {
            MadgwickAHRSupdateIMU(micros(), curGyro.x, curGyro.y, curGyro.z, curAccel.x, curAccel.y, curAccel.z);
        }
        getYawPitchRollDegrees(&currentAttitude.courseMagnetic, &currentAttitude.pitch, &currentAttitude.roll);
        currentAttitude.timestamp = micros64();
    }

    //Since the madgwick algorithm took a while, update our current time
    now = micros();

    if (theFlags.bmp180Enabled) {
        bmp180GetData(&bmp180staticPressure);
        if ((u32) bmp180staticPressure.timestamp < now &&
            now - (u32) bmp180staticPressure.timestamp > 2 * periodStatic) {
            doReset();
        }
        updateAltitudeData(&bmp180staticPressure, &myAltitudeData);
    }

    if (theFlags.bmp280Enabled) {
        bmp280GetData(&bmp280pitotPressure, &bmp280_0x76);
        if ((u32) bmp280pitotPressure.timestamp < now && now - (u32) bmp280pitotPressure.timestamp > 2 * periodPitot) {
            doReset();
        }
    }

    if (theFlags.bmp180Enabled && theFlags.bmp280Enabled) {
        calculateAirspeed(&bmp280pitotPressure, &bmp180staticPressure, &myAirspeed);
    }

    batteryGetData(&curBattery);
}

//Status variables for the main loop
u08 nextTask = 1;

int main(void) {
    initIO();

    while (1) {
        //Get current sensor data
        updateSensors();

        //Check if we have a new raspi and xbee available
        if (xBeeNewMessageReady) {
            xBeeHandleMessage();
        }
        if (raspiNewMessageReady) {
            raspiHandleMessage();
        }

        //Update the flight controllers
        updateFlightControls();

        switch (nextTask) {
            case 1:
                nextTask = 2;
                if (gpsCheck()) {
                    gpsUpdate();
                    break;
                }

            case 2:
                nextTask = 3;
                if (commsCheckAndSendTelemetry() != 0) { //Nonzero return indicates something was done
                    break;
                }

            case 3:
                nextTask = 1;
                commsCheckAndSendLogging();
                break;

            default:
                break;
        }


        //Finally, reset the watchdog
        wdt_reset();
    }
}