//
//  flightController.c
//  autopilot
//
//  Created by Ludger Heide on 13.11.15.
//  Copyright © 2015 Ludger Heide. All rights reserved.
//


#include "flightController.h"
#include "pidController.h"
#include "../avrlib/timer.h"
#include "../sensors/i2cperipherals.h"
#include "../setup/global.h"
#include "../utils.h"
#include "../avrlib/servo.h"
#include "../setup/pinSetup.h"

#include <math.h>

//Static method definitions
static void updateHNAV(void);

static void updateVNAV(void);

static void updateThrust(void);

static pidData rateOfClimbPid, pitchPid, yawPid;

//Constants
static const float maximumRateOfTurn = (M_PI / 6.0); //Rate of turn corresponding to maximum goal (in rad/s)
static const s16 maximumAltitudeError = 10000; //Altitude error in centimeters for which we command max climb (in cm)
static const s16 maximumRateOfClimb = 200; //Maximum commanded rate of limb (in cm/s)
static const s16 maximumPitch = 64 * 20;
static const u32 maximumCommandSetAge = 1000000 / 2; //0.5 seconds

void checkSensors(void) {
}

void flightControllerInit(void) {
    currentFlightMode = DroneMessage_FlightMode_m_off;
    //TODO: Normalize error values wo we can normalize controller parameters?
    init_pid(&yawPid, 1.0 / maximumRateOfTurn, 1.0 / maximumRateOfTurn, 1.0 / maximumRateOfTurn, maximumRateOfTurn);

    init_pid(&rateOfClimbPid, 1.0 / maximumRateOfClimb, 1.0 / maximumRateOfClimb, 0, maximumRateOfClimb);
    init_pid(&pitchPid, 1.0 / maximumPitch, 1.0 / maximumPitch, 1.0 / maximumPitch, maximumPitch);

    outputCommandSet.yaw = UINT8_MAX / 2;
    outputCommandSet.pitch = UINT8_MAX / 2;
    outputCommandSet.thrust = 0;

    servoSetPosition(YAW_SERVO_CHAN, outputCommandSet.yaw);
    servoSetPosition(PITCH_SERVO_CHAN, outputCommandSet.pitch);
    servoSetPosition(THRUST_SERVO_CHAN, outputCommandSet.thrust);

}

void updateFlightControls(void) {
    //Check the age of the command set
    u64 now = micros64();
    if (now - commandUpdate.timestamp > maximumCommandSetAge) {
        //We have an expired command set, do a zero-power no-turn descent
        //TODO: Change this to attempted RTB for x minutes
        currentFlightMode = DroneMessage_FlightMode_m_degraded;
        commandUpdate.which_horizontalCommand = DroneMessage_CommandUpdate_rate_of_turn_tag;
        commandUpdate.horizontalCommand.rate_of_turn = 0;

        commandUpdate.which_verticalCommand = DroneMessage_CommandUpdate_pitch_angle_tag;
        commandUpdate.verticalCommand.pitch_angle = 0;

        commandUpdate.which_SpeedCommand = DroneMessage_CommandUpdate_throttle_tag;
        commandUpdate.SpeedCommand.throttle = 0;
    } else {
        currentFlightMode = DroneMessage_FlightMode_m_flybywire;
    }

    //Now set the controllers for the current command set
    updateHNAV();
    updateVNAV();
    updateThrust();

    //Now update the actual control surfaces
    servoSetPosition(YAW_SERVO_CHAN, outputCommandSet.yaw);
    servoSetPosition(PITCH_SERVO_CHAN, outputCommandSet.pitch);
    servoSetPosition(THRUST_SERVO_CHAN, outputCommandSet.thrust);
}

static void updateHNAV() {
    s08 targetRateOfTurn;

    switch (commandUpdate.which_horizontalCommand) {
        case DroneMessage_CommandUpdate_heading_tag:
            //Maintaining a heading requires a working magnetometer, then it's the same as maintainign a rate of turn
            if (!theFlags.magEnabled) {
                //If we do not have a magnetometer, go to degraded mode and try to maintain zero rate of turn
                currentFlightMode = DroneMessage_FlightMode_m_degraded;
                targetRateOfTurn = 0;
            } else {
                int32_t currentHeading = currentAttitude.courseMagnetic; //64*degrees
                int32_t headingError = commandUpdate.horizontalCommand.heading - currentHeading;

                const s32 oneEightyDegrees = 64 * 180;

                if (headingError < -oneEightyDegrees) {
                    headingError += oneEightyDegrees;
                } else if (headingError > oneEightyDegrees) {
                    headingError -= oneEightyDegrees;
                }
                //TODO: Do we need a real controller here?
                targetRateOfTurn = maps32(headingError, -oneEightyDegrees, oneEightyDegrees, INT8_MIN, INT8_MAX);
            }
            break;

        case DroneMessage_CommandUpdate_rate_of_turn_tag:
            targetRateOfTurn = commandUpdate.horizontalCommand.rate_of_turn;
            break;

        default:
            targetRateOfTurn = 0;
            break;
    }

    //Now that we have the target rate of turn, try to turn it into a yaw reading
    //First, check if we have a gyro. If not, go degraded to zero rudder and hope for the best
    if (theFlags.gyroEnabled) {
        float currentYawRate = curGyro.z; //Positive value means turning to the right (clockwise viewd from top), negative left (counteclockwise)
        float wantedYawRate = mapfloat(targetRateOfTurn, INT8_MIN, INT8_MAX, -maximumRateOfTurn, maximumRateOfTurn);
        float rudderValue = pidUpdate(&yawPid, wantedYawRate, currentYawRate, micros());
        outputCommandSet.yaw = mapfloat(rudderValue, -1, 1, 0, UINT8_MAX);
    } else {
        currentFlightMode = DroneMessage_FlightMode_m_degraded;
        outputCommandSet.yaw = UINT8_MAX / 2;
    }
}

static void updateVNAV(void) {
    bool pitchInUse = false;
    s16 targetPitch;
    s16 targetRateOfClimb;

    switch (commandUpdate.which_verticalCommand) {
        case DroneMessage_CommandUpdate_altitude_tag:
            //If we are maintaining altitude, we need a working static pressure
            //Else go to degraded and try to maintain zero pitch
            if (theFlags.bmp180Enabled) { //BMP180 provides static pressure
                s32 targetAltitude = commandUpdate.verticalCommand.altitude; //centimeters
                s32 realAltitude = myAltitudeData.altitude;
                s32 altitudeError = targetAltitude - realAltitude;

                targetRateOfClimb = maps32(altitudeError, -maximumAltitudeError, maximumAltitudeError,
                                           -maximumRateOfClimb, maximumRateOfClimb);
            } else {
                pitchInUse = true;
                targetPitch = 0;
            }
            break;

        case DroneMessage_CommandUpdate_rate_of_climb_tag:
            //To maintain a rate of climb, we need a working altimeter
            if (theFlags.bmp180Enabled) {
                targetRateOfClimb = commandUpdate.verticalCommand.rate_of_climb;
                if (targetRateOfClimb > maximumRateOfClimb) {
                    targetRateOfClimb = maximumRateOfClimb;
                } else if (targetRateOfClimb < -maximumRateOfClimb) {
                    targetRateOfClimb = -maximumRateOfClimb;
                }
            } else {
                pitchInUse = true;
                targetPitch = 0;
            }
            break;

        case DroneMessage_CommandUpdate_pitch_angle_tag:
            pitchInUse = true;
            targetPitch = mapfloat(commandUpdate.verticalCommand.pitch_angle, INT8_MIN, INT8_MAX, -maximumPitch,
                                   maximumPitch);
            break;

        default:
            targetRateOfClimb = 0;
    }

    //If we are not directly using pitch, we need to translate the target rate of climb to target pitch using the climb controller
    u32 now = micros();
    if (!pitchInUse) {
        float targetPitchNormalized = pidUpdate(&rateOfClimbPid, targetRateOfClimb, myAltitudeData.rate_of_climb, now);
        targetPitch = mapfloat(targetPitchNormalized, -1, 1, -maximumPitch, maximumPitch);
    }

    //Now, go from target pitch to control surface
    float elevatorValue = pidUpdate(&pitchPid, targetPitch, currentAttitude.pitch, now);
    outputCommandSet.pitch = mapfloat(elevatorValue, -1, 1, 0, UINT8_MAX);
}

static void updateThrust(void) {
    outputCommandSet.thrust = 0;
}