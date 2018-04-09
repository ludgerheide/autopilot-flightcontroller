//
// Created by Ludger Heide on 27.03.18.
//

#include "bno055-info.h"
#include "../setup/global.h"

#ifndef FLIGHTCONTROLLER_BNO055_H
#define FLIGHTCONTROLLER_BNO055_H

#endif //FLIGHTCONTROLLER_BNO055_H

typedef enum {
    BNO055_IDLE,
    BNO055_EULER,
    BNO055_ANGULAR_VELOCITY,
    BNO055_LINEARACCEL,
    BNO055_GRAVITY,
} bno055State;

//Initializes the sensor. Quite slow, only run it once on startup
bool bno055Init(void);

//Get the software and hardware revision info. Not really useful
void bno055GetRevInfo(bno055_rev_info_t *info);

//Get the system status. Also not useful if everything was set up properly
void bno055GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error);

//Gets the calibration status. Prbably called every second after startuop until the calibration is complete
void bno055GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

//Start receiving a set of data
//void bno055StartReceiving(void);

//Copy the data out of the receiving buffer into our struct
//This is still inside the ISR, so it just copies data and doesn't do the floating-point scaling yet
//void bno055StartReceiving(void);

//This finally fills the vector
//void bno055GetData(attitude_struct *attitude, xyz* angularVelocity, xyz* gravity, xyz* linearAcceleration);

//Gets the data from the sensor, blocking for the whole transmission
void bno055GetDataBlocking(attitude_struct *attitude, xyz *angularVelocity, xyz *gravity, xyz *linearAcceleration);