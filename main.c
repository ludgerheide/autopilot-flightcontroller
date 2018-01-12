

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
}

static void updateSensors(void) {
    //Get the current sensor events
    gyroGetData(&curGyro);
    accelGetData(&curAccel);

    magGetData(&uncompensatedMag);
    magCompensate(&uncompensatedMag, &curMag);

    //Update the madgwick algorithm
    MadgwickAHRSupdate(micros(), curGyro.x, curGyro.y, curGyro.z, curAccel.x, curAccel.y, curAccel.z, curMag.x,
                       curMag.y, curMag.z);
    getYawPitchRollDegrees(&currentAttitude.courseMagnetic, &currentAttitude.pitch, &currentAttitude.roll);
    currentAttitude.timestamp = micros64();

    bmp180GetData(&bmp180staticPressure);
    updateAltitudeData(&bmp180staticPressure, &myAltitudeData);

    bmp280GetData(&bmp280pitotPressure, &bmp280_0x76);
    calculateAirspeed(&bmp280pitotPressure, &bmp180staticPressure, &myAirspeed);

    batteryGetData(&curBattery);
}

int main(void) {
    initIO();
    u08 nextTask = 1;

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
    }
}