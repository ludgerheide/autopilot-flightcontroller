//
//  communicationsHandler.c
//  autopilot
//
//  Created by Ludger Heide on 06.10.15.
//  Copyright © 2015 Ludger Heide. All rights reserved.
//

#include "communicationsHandler.h"

#include "../avrlib/nmea.h"

#include "../control/flightControllerTypesAndGlobals.h"
#include "../utils.h"

#include "../setup/pinSetup.h"
#include "../avrlib/uart4.h"
#include "../avrlib/timer.h"
#include <assert.h>
#include <avr/eeprom.h>

#ifndef CRITICAL_SECTION_START
#define CRITICAL_SECTION_START    unsigned char _sreg = SREG; cli()
#define CRITICAL_SECTION_END    SREG = _sreg
#endif

typedef enum {
    telemetry,
    logging
} messagePurpose;

static char messageBuffer[255];

//This holds the ID and time of the last transmission. We use it to refreain from sending until the last one has gone through
//As well as limiting the telemetry frequency
static BOOL lastTxAcked;
static u32 lastTelemetryTxTime; //actually milliseconds, since it's just for measuring a differnce in millliseconds
static const u16 telemetryDelay = 250; //The delay between each telemetry message in milliseconds

//These hold the timestamps for the last packets that were
// a) transmitted over the telemetry
// b) send thtough the serial (logging port)
//TODO Actually, we should be able to store those values as u64 because we don't care about the rollovers
//TODO And then compare (u32) timestamp to (u32) telPosTime...
static u64 telAirSpeedTime, telPosTime, telAltTime, telAttitudeTime, telBatteryTime;
static u64 logGpsSpeedTime, logAirSpeedTime, logPosTime, logAltTime, logAttitudeTime, logStaticPressureTime, logPitotPressureTime, logRawGyroTime, logRawMagTime, logRawAccelTime, logBatteryTime, logOutCommandSetTime, logCommandUpdateTime, logHomeBaseTime, logSlpTime;
static DroneMessage outgoingMsg, incomingMsg; //Allocate it here so it gets added to the compile-time memory usage stat (stack vs heap etc).

//Function n that creates a message and puts it into the message buffer
static u08 createProtobuf(messagePurpose thePurpose, u08 *messageLength) {
    assert(thePurpose == logging || thePurpose == telemetry);

    //Logging gets everything we have, telemtry gets less
    //Create a new message
    outgoingMsg = (DroneMessage) DroneMessage_init_zero;

    //Message creation timestamp for logging
    if (thePurpose == logging) {
        outgoingMsg.has_timestamp = true;
        outgoingMsg.timestamp = micros64();
    }

    //FlighMode just for logging, but then always
    if (thePurpose == logging) {
        outgoingMsg.has_current_mode = true;
        outgoingMsg.current_mode = currentFlightMode;
    }

    //Add GpsVelocity to logging, but not to telemetry, only if there is a new measurement available
    if (thePurpose == logging && GpsInfo.VelHS.timestamp - logGpsSpeedTime) {
        outgoingMsg.has_current_groundspeed = true;

        outgoingMsg.current_groundspeed.timestamp = GpsInfo.VelHS.timestamp;
        outgoingMsg.current_groundspeed.speed = GpsInfo.VelHS.speed;
        outgoingMsg.current_groundspeed.course_over_ground = GpsInfo.VelHS.heading;

        logGpsSpeedTime = GpsInfo.VelHS.timestamp;
    }

    //Add airspeed to both telemetry and logging output, but minize the amount of data in telemetry
    if ((thePurpose == logging && myAirspeed.timestamp - logAirSpeedTime) ||
        (thePurpose == telemetry && myAirspeed.timestamp - telAirSpeedTime)) {
        outgoingMsg.has_current_airspeed = true;

        //For telemetry as well as logging, set the auspeed
        outgoingMsg.current_airspeed.speed = myAirspeed.speed;

        //Only for logging, give it a timestamp
        if (thePurpose == logging) {
            outgoingMsg.current_airspeed.has_timestamp = true;
            outgoingMsg.current_airspeed.timestamp = myAirspeed.timestamp;
        }

        //Update the "last sent" times
        if (thePurpose == logging) {
            logAirSpeedTime = myAirspeed.timestamp;
        } else {
            telAirSpeedTime = myAirspeed.timestamp;
        }
    }

    //Position for both, but with the correct last update
    if ((thePurpose == logging && GpsInfo.PosLLA.timestamp - logPosTime) ||
        (thePurpose == telemetry && GpsInfo.PosLLA.timestamp - telPosTime)) {
        outgoingMsg.has_current_position = true;

        //For both, add lat/lon
        outgoingMsg.current_position.latitude = GpsInfo.PosLLA.lat;
        outgoingMsg.current_position.longitude = GpsInfo.PosLLA.lon;

        //For logging, add real time, timestamp, gps altitude and number of satellites
        if (thePurpose == logging) {
            outgoingMsg.current_position.has_timestamp = true;
            outgoingMsg.current_position.timestamp = GpsInfo.PosLLA.timestamp;

            outgoingMsg.current_position.has_real_time = true;
            outgoingMsg.current_position.real_time = GpsInfo.PosLLA.TimeOfFix;

            outgoingMsg.current_position.has_gps_altitude = true;
            outgoingMsg.current_position.gps_altitude = GpsInfo.PosLLA.alt;

            outgoingMsg.current_position.has_number_of_satellites = true;
            outgoingMsg.current_position.number_of_satellites = GpsInfo.numSVs;
        }

        //Now update the time
        if (thePurpose == logging) {
            logPosTime = GpsInfo.PosLLA.timestamp;
        } else {
            telPosTime = GpsInfo.PosLLA.timestamp;
        }
    }

    //Altitude for both, extened if we're logging
    if ((thePurpose == logging && myAltitudeData.timestamp - logAltTime) ||
        (thePurpose == telemetry && myAltitudeData.timestamp - telAltTime)) {
        outgoingMsg.has_current_altitude = true;

        //For both, add altitude
        outgoingMsg.current_altitude.altitude = myAltitudeData.altitude;

        //For logging, add timestamp and rate of climb
        if (thePurpose == logging) {
            outgoingMsg.current_altitude.has_timestamp = true;
            outgoingMsg.current_altitude.timestamp = myAltitudeData.timestamp;
            outgoingMsg.current_altitude.has_rate_of_climb = true;
            outgoingMsg.current_altitude.rate_of_climb = myAltitudeData.rate_of_climb;
        }
        //Now update the time
        if (thePurpose == logging) {
            logAltTime = myAltitudeData.timestamp;
        } else {
            telAltTime = myAltitudeData.timestamp;
        }
    }

    //Attitude for both, as above
    if ((thePurpose == logging && currentAttitude.timestamp - logAttitudeTime) ||
        (thePurpose == telemetry && currentAttitude.timestamp - telAttitudeTime)) {
        outgoingMsg.has_current_attitude = true;

        outgoingMsg.current_attitude.course_magnetic = currentAttitude.courseMagnetic;
        outgoingMsg.current_attitude.pitch = currentAttitude.pitch;
        outgoingMsg.current_attitude.roll = currentAttitude.roll;

        //For logging with timestamp
        if (thePurpose == logging) {
            outgoingMsg.current_attitude.has_timestamp = true;
            outgoingMsg.current_attitude.timestamp = currentAttitude.timestamp;
        }

        //Now update the time
        if (thePurpose == logging) {
            logAttitudeTime = currentAttitude.timestamp;
        } else {
            telAttitudeTime = currentAttitude.timestamp;
        }
    }

    //Barodata (static) just for logging
    if (thePurpose == logging && bmp180staticPressure.timestamp - logStaticPressureTime) {
        outgoingMsg.has_static_pressure = true;

        outgoingMsg.static_pressure.timestamp = bmp180staticPressure.timestamp;
        outgoingMsg.static_pressure.pressure = bmp180staticPressure.pressure;
        outgoingMsg.static_pressure.temperature = bmp180staticPressure.temperature;

        //Now update the time
        logStaticPressureTime = bmp180staticPressure.timestamp;
    }

    //Barodata (pitot) just for logging
    if (thePurpose == logging && bmp280pitotPressure.timestamp - logPitotPressureTime) {
        outgoingMsg.has_pitot_pressure = true;

        outgoingMsg.pitot_pressure.timestamp = bmp280pitotPressure.timestamp;
        outgoingMsg.pitot_pressure.pressure = bmp280pitotPressure.pressure;
        outgoingMsg.pitot_pressure.temperature = bmp280pitotPressure.temperature;

        //Now update the time
        logPitotPressureTime = bmp280pitotPressure.timestamp;
    }

    //Gyrodata just for logging
    if (thePurpose == logging && curGyro.timestamp - logRawGyroTime) {
        outgoingMsg.has_gyro_raw = true;

        outgoingMsg.gyro_raw.timestamp = curGyro.timestamp;
        outgoingMsg.gyro_raw.x = curGyro.x;
        outgoingMsg.gyro_raw.y = curGyro.y;
        outgoingMsg.gyro_raw.z = curGyro.z;

        //Now update the time
        logRawGyroTime = curGyro.timestamp;
    }

    //MagData just for logging
    if (thePurpose == logging && curMag.timestamp - logRawMagTime) {
        outgoingMsg.has_mag_raw = true;

        outgoingMsg.mag_raw.timestamp = curMag.timestamp;
        outgoingMsg.mag_raw.x = curMag.x;
        outgoingMsg.mag_raw.y = curMag.y;
        outgoingMsg.mag_raw.z = curMag.z;

        //Time update
        logRawMagTime = curMag.timestamp;
    }

    //AccelData just for logging
    if (thePurpose == logging && curAccel.timestamp - logRawAccelTime) {
        outgoingMsg.has_accel_raw = true;

        outgoingMsg.accel_raw.timestamp = curAccel.timestamp;
        outgoingMsg.accel_raw.x = curAccel.x;
        outgoingMsg.accel_raw.y = curAccel.y;
        outgoingMsg.accel_raw.z = curAccel.z;

        //Time update
        logRawAccelTime = curAccel.timestamp;
    }


    //Battery for both, as way above
    if ((thePurpose == logging && curBattery.timestamp - logBatteryTime) ||
        (thePurpose == telemetry && curBattery.timestamp - telBatteryTime)) {
        outgoingMsg.has_current_battery_data = true;

        outgoingMsg.current_battery_data.voltage = curBattery.voltage; // in mV
        outgoingMsg.current_battery_data.current = curBattery.current; // in mA

        //For logging, add timestamp
        if (thePurpose == logging) {
            outgoingMsg.current_battery_data.has_timestamp = true;
            outgoingMsg.current_battery_data.timestamp = curBattery.timestamp;
        }

        //Now update the time
        if (thePurpose == logging) {
            logBatteryTime = curBattery.timestamp;
        } else {
            telBatteryTime = curBattery.timestamp;
        }
    }

    //Output Command set
    if (thePurpose == logging && outputCommandSet.timestamp - logOutCommandSetTime) {
        outgoingMsg.has_output_command_set = true;

        outgoingMsg.output_command_set.yaw = outputCommandSet.yaw;
        outgoingMsg.output_command_set.pitch = outputCommandSet.pitch;
        outgoingMsg.output_command_set.thrust = outputCommandSet.thrust;

        //Time update
        logOutCommandSetTime = outputCommandSet.timestamp;
    }

    //In logging mode, also log inputs

    //CommandUpdate that is in use
    if (thePurpose == logging && commandUpdate.has_timestamp == true &&
        commandUpdate.timestamp - logCommandUpdateTime) {
        outgoingMsg.has_current_command = true;
        outgoingMsg.current_command = commandUpdate;

        //Time update
        logCommandUpdateTime = commandUpdate.timestamp;
    }

    //SLP
    if (thePurpose == logging && mySeaLevelPressure.timestamp - logSlpTime) {
        outgoingMsg.has_sea_level_pressure = true;
        outgoingMsg.sea_level_pressure = mySeaLevelPressure.slp;

        //Time update
        logSlpTime = mySeaLevelPressure.timestamp;
    }

    //home base
    if (thePurpose == logging && homeBase.timestamp - logHomeBaseTime) {
        outgoingMsg.has_home_base = true;

        outgoingMsg.home_base.has_timestamp = true;
        outgoingMsg.home_base.timestamp = homeBase.timestamp;

        outgoingMsg.home_base.latitude = homeBase.latitude;
        outgoingMsg.home_base.longitude = homeBase.longitude;
        outgoingMsg.home_base.altitude = homeBase.altitude * 100;

        //Time update
        logHomeBaseTime = homeBase.timestamp;
    }

    //Now create the buffer and write the message out
    pb_ostream_t stream = pb_ostream_from_buffer((u08 *) messageBuffer, sizeof(messageBuffer));

    /* Now we are ready to encode the message! */
    bool status = pb_encode(&stream, DroneMessage_fields, &outgoingMsg);
    *messageLength = stream.bytes_written;

#ifdef COMMS_DEBUG
    printf_P(PSTR("Created message of size %i for purpose %i \r\n"), *messageLength, thePurpose);
#endif

    /* Then just check for any errors.. */
    if (!status) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Encoding failed: %s\r\n"), PB_GET_ERROR(&stream));
#endif
        return 1;
    }
    return 0;
}

//Initializes the communications (the xBee serial and the RTS output pin)
void commsInit(void) {
    xBeeInit();
    xBeeAttachTxStatusHandler(txStatusHandler);
}

void commsProcessMessage(char *message, u08 size) {
#ifdef COMMS_DEBUG
    printf_P(PSTR("Processing msg!"));
#endif

    assert(size <= 100);
    u64 now = micros64();

    //Zero out the message so we only get valid data
    incomingMsg = (DroneMessage) DroneMessage_init_zero;

    //Set up an input stream
    pb_istream_t inStream = pb_istream_from_buffer((u08 *) message, size);

    BOOL status = pb_decode(&inStream, DroneMessage_fields, &incomingMsg);

    if (!status) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Decoding failed!"));
#endif
        return;
    }

    //Now do stuff according to the content of the decoded message
    if (incomingMsg.has_input_command) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Protobuf: Have inputCommand!\r\n"));
#endif
        commandUpdate = incomingMsg.input_command;
        commandUpdate.timestamp = now;
    }

    if (incomingMsg.has_sea_level_pressure) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Protobuf: Have sea level pressure!\r\n"));
#endif
        //Check if value is sane and update ram and eeprom
        if (incomingMsg.sea_level_pressure <= 1100 && incomingMsg.sea_level_pressure >= 850) {
            mySeaLevelPressure.slp = incomingMsg.sea_level_pressure;
            mySeaLevelPressure.timestamp = now;
            writeSlpToEEPROM();
        }

    }

    if (incomingMsg.has_home_base) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Protobuf: Have home base!\r\n"));
#endif

        if (incomingMsg.home_base.latitude <= 90 && incomingMsg.home_base.latitude >= -90 &&
            incomingMsg.home_base.longitude <= 180 && incomingMsg.home_base.longitude >= -180 &&
            incomingMsg.home_base.altitude <= (s32) 10000 * (s32) 100 &&
            incomingMsg.home_base.altitude >= (s32) -418 * (s32) 100) {

            homeBase.timestamp = now;
            homeBase.latitude = incomingMsg.home_base.latitude;
            homeBase.longitude = incomingMsg.home_base.longitude;
            homeBase.altitude = incomingMsg.home_base.altitude;

            writeHomeBaseToEEPROM();
        }
    }
}

//Check if the time has come to send a new message, then send it
//Return 0 if nothing was done
int commsCheckAndSendTelemetry(void) {
    u32 now = millis();
    u08 telemetryLength = 0;

    //Check if the serial buffer is empty, we should send a new msg
    //and the last transmission has been acked (disreagars the ack check if more than one second has passed since the last transmission
    if (uartReadyTx[XBEE_UART] && (now - lastTelemetryTxTime > telemetryDelay) &&
        (lastTxAcked || now - lastTelemetryTxTime > 1000)) {
        if (createProtobuf(telemetry, &telemetryLength)) {
#ifdef COMMS_DEBUG
            printf_P(PSTR("Creating protobuf failed @ %i\r\n"), __LINE__);
#endif
            return -1; //To indiucate error
        } else {
            xBeeSendPayload(messageBuffer, telemetryLength, false, 0xAB);
            lastTxAcked = FALSE;
            lastTelemetryTxTime = now;
            return 1; //To indicate sucess
        }
    } else {
        return 0; //To indicate we have done nothing
    }
}

//Check if the time has come to send a new message, then send it
//Return 0 if nothing was done
int commsCheckAndSendLogging(void) {
    u08 loggingLength = 0;

    //Check if the serial buffer is empty
    if (uartReadyTx[RASPI_UART]) {
        if (createProtobuf(logging, &loggingLength)) {
            //Nonzero return code indicates failure
#ifdef COMMS_DEBUG
            printf_P(PSTR("Creating protobuf failed @ %i\r\n"), __LINE__);
#endif
            return -1;
        }

        //If we get here creating the protobuf succeeded. Put the magic number in the buffer first, then the length, then the message
        uartAddToTxBuffer(RASPI_UART, 's');
        uartAddToTxBuffer(RASPI_UART, 't');
        uartAddToTxBuffer(RASPI_UART, 'a');
        uartAddToTxBuffer(RASPI_UART, 'r');
        uartAddToTxBuffer(RASPI_UART, 't');
        uartAddToTxBuffer(RASPI_UART, loggingLength);

        u08 checksum = 0;
        //Add the data and add a checksum
        for (u08 i = 0; i < loggingLength; i++) {
            uartAddToTxBuffer(RASPI_UART, messageBuffer[i]);
            checksum += messageBuffer[i];
        }
        uartAddToTxBuffer(RASPI_UART, checksum);
#ifdef COMMS_DEBUG
        for(u08 i = 0; i < uartGetTxBuffer(RASPI_UART)->datalength; i++) {
            printf("%02x ", uartGetTxBuffer(RASPI_UART)->dataptr[i]);
        }
        printf("\r\n");
#endif
        uartSendTxBuffer(RASPI_UART);
        return 1;
    } else {
        return 0;
    }
}

void txStatusHandler(uint8_t frameID, __attribute__ ((unused)) uint8_t retryCount,
                     __attribute__ ((unused)) uint8_t txStatus) {
    if (frameID == 0xAB) {
#ifdef COMMS_DEBUG
        printf_P(PSTR("Tx Ack for fram %02X, retryCount %02X, status %02X\r\n"), frameID, retryCount, txStatus);
#endif
        lastTxAcked = TRUE;
    }
#ifdef COMMS_DEBUG
    else{
        printf_P(PSTR("Invalid msg @%i\r\n"), __LINE__);
    }
#endif
}