

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
    wdt_disable();
    wdt_enable(WDTO_DEFAULT);
}

//Status variables for the sensors
const u32 periodGyro = 1000000 / 190; //Microseconds
const u32 periodAccel = 1000000 / 200;
const u32 periodMag = 1000000 / 220;
const u32 periodStatic = 1000 * (BMP180_TEMPERATURE_DURATION + BMP180_PRESSURE_DURATION);
const u32 periodPitot = (u32) 1000 * 39;


static void updateSensors(void) {
    u32 now = micros();

    //Get the current sensor events and validate their times, otherwise, reste i2c and exit from the method
    gyroGetData(&curGyro);
    if ((u32) curGyro.timestamp < now && now - (u32) curGyro.timestamp > 2 * periodGyro) {
        IMUinit();
        return;
    }

    accelGetData(&curAccel);
    if ((u32) curAccel.timestamp < now && now - (u32) curAccel.timestamp > 2 * periodAccel) {
        IMUinit();
        return;
    }

    magGetData(&uncompensatedMag);
    if ((u32) uncompensatedMag.timestamp < now && now - (u32) uncompensatedMag.timestamp > 4 *
                                                                                           periodMag) { //4X Because the magnetometer likes to skip a sample every now and then and I don't really care about that
        IMUinit();
        return;
    }

    magCompensate(&uncompensatedMag, &curMag);

    //Update the madgwick algorithm
    MadgwickAHRSupdate(micros(), curGyro.x, curGyro.y, curGyro.z, curAccel.x, curAccel.y, curAccel.z, curMag.x,
                       curMag.y, curMag.z);
    getYawPitchRollDegrees(&currentAttitude.courseMagnetic, &currentAttitude.pitch, &currentAttitude.roll);
    currentAttitude.timestamp = micros64();

    //Since the madgwick algorithm took a while, update our current time
    now = micros();

    bmp180GetData(&bmp180staticPressure);
    if ((u32) bmp180staticPressure.timestamp < now && now - (u32) bmp180staticPressure.timestamp > 2 * periodStatic) {
        IMUinit();
    }
    updateAltitudeData(&bmp180staticPressure, &myAltitudeData);

    bmp280GetData(&bmp280pitotPressure, &bmp280_0x76);
    if ((u32) bmp280pitotPressure.timestamp < now && now - (u32) bmp280pitotPressure.timestamp > 2 * periodPitot) {
        IMUinit();
    }
    calculateAirspeed(&bmp280pitotPressure, &bmp180staticPressure, &myAirspeed);

    batteryGetData(&curBattery);
}

//Status variables for the main loop
u08 nextTask = 1;

int main(void) {
    initIO();

    while (1) {
        updateSensors();

        //Check if we have a new raspi and xbee available
        if (xBeeNewMessageReady) {
            xBeeHandleMessage();
        }
        if (raspiNewMessageReady) {
            raspiHandleMessage();
        }

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

        //Check the I2C sensor status, reset the i2c bus if stuff looks strange


        //Finally, reset the watchdog
        wdt_reset();
    }
}