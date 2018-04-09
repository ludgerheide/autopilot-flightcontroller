#include <avr/io.h>
#include <util/delay.h>
#include "sensors/bmp180.h"

#include "utils.h"
#include "avrlib/timer.h"
#include "sensors/gps.h"
#include "comms/raspiComms.h"
#include "avrlib/servo.h"
#include "avrlib/uart4.h"

int main(void) __attribute__ ((noreturn));

static void initIO(void) {
    raspiInit();
    printfAttachToUart();
    timerInit();
    //i2cInit();
    //bno055Init();
    batteryInit();
    servoInit();

    sbi(DDRF, 2);
    uartInit(GPS_UART);
    uartSetBaudRate(GPS_UART, 9600);
}

int main(void) {
    initIO();

    while (1) {
        /*u08 system_status, self_test_result, system_error;
        bno055GetSystemStatus(&system_status, &self_test_result, &system_error);
        printf("Status: System %u Selftest %u Error %u\r\n", system_status, self_test_result, system_error);

        u08 sys, gyro, accel, mag;
        bno055GetCalibration(&sys, &gyro, &accel, &mag);
        printf("Calib: sys %u, gyro %u, accel %u, mag %u\r\n", sys, gyro, accel, mag);

        bno055GetDataBlocking(&currentAttitude, &angularVelocity, &gravity, &linearAcceleration);
        printf("\t attidude (deg) \t gyro (rad/s) \t gravity (m/s^2) \t linear acc. (m/s^2)\r\n");
        printf("x \t %5.1f \t\t\t %5.1f \t\t\t %5.1f \t\t\t %5.1f\r\n", currentAttitude.courseMagnetic/64.0, angularVelocity.x/16.0, gravity.x/100.0, linearAcceleration.x/100.0);
        printf("y \t %5.1f \t\t\t %5.1f \t\t\t %5.1f \t\t\t %5.1f\r\n", currentAttitude.pitch/64.0, angularVelocity.y/16.0, gravity.y/100.0, linearAcceleration.y/100.0);
        printf("z \t %5.1f \t\t\t %5.1f \t\t\t %5.1f \t\t\t %5.1f\r\n", currentAttitude.roll/64.0, angularVelocity.z/16.0, gravity.z/100.0, linearAcceleration.z/100.0);
        printf("\r\n");
        _delay_ms(100);*/

        batteryGetData(&curBattery);
        //printf("%.1f A0: %.2f, A1:%.2f\r\n", curBattery.timestamp/1000000.0, curBattery.current/1000.0, curBattery.voltage/1000.0);
        u08 pos = round(mapfloat(curBattery.current, 0, 2560, 0, 255));
        for (u08 i = 0; i < 5; i++) {
            servoSetPosition(i, pos);
        }
        PORTF ^= 1 << 2;
        cBuffer *gps = uartGetRxBuffer(GPS_UART);
        while (gps->datalength > 0) {
            printf("%c", bufferGetFromFront(gps));
        }

    }
}