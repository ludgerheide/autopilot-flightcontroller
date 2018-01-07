

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

    while (1) {
        updateSensors();

        //TODO: Only do one of those per cycle (or so)
        //Check if we have a new GPS and xbee available
        if (xBeeNewMessageReady) {
            xBeeHandleMessage();
        }
        if (raspiNewMessageReady) {
            raspiHandleMessage();
        }
        if (gpsCheck()) {
            gpsUpdate();
        }

        //Send telemetry
        commsCheckAndSendTelemetry();
        commsCheckAndSendLogging();
        //printf("Static: %.3f hPa, %.3f °C\r\n", bmp180staticPressure.pressure/25600.0, bmp180staticPressure.temperature/100.0);
        //printf("Pitot: %.3f hPa, %.3f °C\r\n", bmp280pitotPressure.pressure/25600.0, bmp280pitotPressure.temperature/100.0);
        //printf("Altitude: %.3f m, Airspeed: %.3f m/s\r\n", myAltitudeData.altitude/100.0, myAirspeed.speed/100.0);
        //_delay_ms(100);
    }
}