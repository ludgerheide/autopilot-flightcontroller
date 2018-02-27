//
// Created by ludger on 27.02.18.
//

#include <stdfix.h>
#include "../avrlib/avrlibtypes.h"

#ifndef FLIGHTCONTROLLER_PIDCONTROLLER_H
#define FLIGHTCONTROLLER_PIDCONTROLLER_H

typedef struct {
    float kp;
    float ki;
    float kd;

    float lastY;
    float integratorValue;
    float integratorMax;
    u32 lastUpdate;
} pidData;

void init_pid(pidData *thePidData, float kp, float ki, float kd, float integratorMax);

float pidUpdate(pidData *thePidData, float w, float y, u32 t);

float pidUpdateSeparateD(pidData *thePidData, float w, float y, float dy, u32 t);

#endif //FLIGHTCONTROLLER_PIDCONTROLLER_H
