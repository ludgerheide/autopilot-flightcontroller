//
//  imuBoard.c
//  avr - flyByWire
//
//  Created by Ludger Heide on 25.09.15.
//  Copyright © 2015 LH Technologies. All rights reserved.
//

#include "i2cperipherals.h"

#ifdef IMU_DEBUG
#include <stdio.h>
#endif

#include "../avrlib/avrlibdefs.h"
#include <avr/io.h>
#include "../avrlib/i2c.h"

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START    unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif

//Initialize "enabled" as false so the timer ISR (and other methods) don't communicate with the devices until they're ready
volatile imuFlags theFlags = {.imuEnabled = FALSE, .bmp280_0x76_enabled = FALSE, .bmp280_0x77_enabled=FALSE};

bmp280_configuration bmp280_0x76;
bmp280_configuration bmp280_0x77;

//Initialize all sensors on the IMU board
void I2CInit() {
    //Initialize the I2C bus
    i2cInit();

    //Disable interrupts while everything is being set up
    CRITICAL_SECTION_START;

    /*//Create the configuration for a bmp280 on 0x76
    bmp280_0x76 = bmp280_defaults();
    bmp280_0x76.address = 0x76;

    //Initialize the BMP280 pressure/temp sensor on 0x76
    if (bmp280Init(&bmp280_0x76)) {
        theFlags.bmp280Enabled = TRUE;

        //Attach the interrupt handler for the timer ticks
        timerSetInterruptCallback(imuTimerTick);

        //Start a measurement to set everything off
        const u08 toSend[2] = {BMP280_REGISTER_CONTROL, ((u08) 0x01 | (u08) bmp280_0x76.pressure_mode << 2 |
                                                         (u08) bmp280_0x76.temperature_mode << 5)};
        i2cMasterSendNI(bmp280_0x76.address, 2, (u08 *) &toSend);
    } else {
#ifdef BMP280_DEBUG
        printf_P(PSTR("Error intitializing the BMP280!\r\n"));
#endif
        theFlags.bmp280Enabled = FALSE;
    }*/

    //TODO: BMP280_0x77, both in continuous mode




    //Attach the custom stop handler after all sensors are initialized
    i2cSetStopHandler(customStopHandler);

    //Re-enable the interrupts
    CRITICAL_SECTION_END;
}

//Prototype for the last step of the following method
static void checkForPendingTransmissionsOrStop(void);
/*
//Gets called when an I2C transmission is over
void customStopHandler(u08 statusReg, u08 deviceaddress) {
    //Set the state to idle so the i2c.c methods work
    I2cState = I2C_IDLE;

    //First, find out what just happened
    if (statusReg == TW_MT_DATA_ACK) {
#ifdef IMU_DEBUG
        printf_P(PSTR("SH: Transmission complete from 0x%x\r\n"), deviceaddress);
#endif

        //Now we should receive data for the correct device
        switch (deviceaddress) {
            //TODO: BMP280_Address1

            case BMP280_ADDRESS2:
                assert(bmp280_0x76.state == BMP280_MEASURING || bmp280_0x76.state == BMP280_READY);

                //Check the state
                switch (bmp280_0x76.state) {
                    case BMP280_MEASURING:
#ifdef BMP280_DEBUG
                        printf_P(PSTR("BMP280: in progress\r\n"));
#endif

                        //We have not decided what to do on the bus yet, so decide it
                        checkForPendingTransmissionsOrStop();

                        break;
                    case BMP280_READY:
#ifdef BMP280_DEBUG
                        printf_P(PSTR("BMP280: ready\r\n"));
#endif

                        assert(theFlags.bmp280Ready);
                        assert(millis() - bmp280_0x76.lastStateChange >= bmp280_0x76.measurement_time);

                        theFlags.bmp280Ready = FALSE;

                        //The received data will be six bytes long
                        i2cMasterStartReceiving(bmp280_0x76.address, 6);
                        bmp280_0x76.state = BMP280_RECEIVING;
                        break;
                    default:
                        break;
                }
                break;

            default:
#ifdef IMU_DEBUG
                printf_P(PSTR("Invalid address at line %i"), __LINE__);
#endif
                break;
        }

    } else if (statusReg == TW_MR_DATA_NACK) {
#ifdef IMU_DEBUG
        printf_P(PSTR("SH: Reception complete\r\n"));
#endif

        //First, get the data out of the RX buffer
        switch (deviceaddress) {
            //TODO: BMP280_ADDRESS1

            case BMP280_ADDRESS2:
#ifdef BMP280_DEBUG
                printf_P(PSTR("BMP280: finished receiving\r\n"));
#endif
                assert(bmp280_0x76.state == BMP280_RECEIVING);
                bmp280GetDataFromI2cBuffer(&bmp280_0x76);

                //Now start the next measurement
                bmp280StartCapture(&bmp280_0x76);
                bmp280_0x76.state = BMP280_MEASURING;
                bmp280_0x76.lastStateChange = millis();
                break;



            default:
#ifdef IMU_DEBUG
                printf_P(PSTR("Invalid address at line %i\r\n"), __LINE__);
#endif
                break;
        }

    } else {
#ifdef IMU_DEBUG
        printf_P(PSTR("SH: Error!\r\n"));
#endif

        //Reset the bus
        // reset internal hardware and release bus
        outb(TWCR, (inb(TWCR) & TWCR_CMD_MASK) | BV(TWINT) | BV(TWSTO) | BV(TWEA));
        // set state
        I2cState = I2C_IDLE;
    }
}

static void checkForPendingTransmissionsOrStop() {
    //TODO: BMP280_ADDRESS1
    if (theFlags.bmp280Ready) {
        //Assert there is actually something ready
        assert(bmp280_0x76.state == BMP280_READY);
#ifdef BMP280_DEBUG
        printf_P(PSTR("Starting temp/press (BMP280) data request! @l: %i\r\n"), __LINE__);
#endif
        bmp280StartReceiving(&bmp280_0x76);
    } else {
#ifdef IMU_DEBUG
        printf_P(PSTR("IMU: Sending stop!\r\n"));
#endif
        //Nothing to be done. Send a STOP condition
        outb(TWCR, (inb(TWCR) & TWCR_CMD_MASK) | BV(TWINT) | BV(TWEA) | BV(TWSTO));
    }
}

void imuTimerTick(u32 millis) {
    //Only do stuff if the bmp is actually enabled
    if (theFlags.bmp280Enabled) {
        if (bmp280_0x76.state == BMP280_MEASURING &&
            millis - bmp280_0x76.lastStateChange > bmp280_0x76.measurement_time) {
#ifdef BMP280_DEBUG
            printf_P(PSTR("BMP280: Data ready (Timer ISR)\r\n"));
#endif
            bmp280_0x76.state = BMP280_READY;
            theFlags.bmp280Ready = TRUE;

            //If I2C is idle, transmit the temp register request, else just set the flag
            if (I2cState == I2C_IDLE) {
                bmp280StartReceiving(&bmp280_0x76);
            }
        }
    }
#ifdef IMU_DEBUG
    else {
        printf_P(PSTR("BMP disabled at line %i"), __LINE__);
    }
#endif

}
 */