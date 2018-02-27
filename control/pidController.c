//
// Created by ludger on 27.02.18.
//

#include "pidController.h"

void init_pid(pidData *thePidData, float kp, float ki, float kd, float integratorMax) {
    thePidData->kp = kp;
    thePidData->ki = ki;
    thePidData->kd = kd;
    thePidData->integratorMax = integratorMax;

    thePidData->lastUpdate = 0;
    thePidData->lastY = 0;
    thePidData->integratorValue = 0;
}

float pidUpdate(pidData *thePidData, float w, float y, u32 t) {
    //Calculate d, then do to separateD code
    float dy;
    if (thePidData->lastUpdate == 0) {
        dy = 0;
    } else {
        dy = ((y - thePidData->lastY) / (t - thePidData->lastUpdate)) * 1000000.0;
    }
    return pidUpdateSeparateD(thePidData, w, y, dy, t);
}

float pidUpdateSeparateD(pidData *thePidData, float w, float y, float dy, u32 t) {
    //Calculate the error
    float e = w - y;

    //Calculate the integral part
    if (thePidData->lastUpdate != 0) {
        float dt = (t - thePidData->lastUpdate) / 1000000.0;
        thePidData->integratorValue += e * dt;
        if (thePidData->integratorValue > thePidData->integratorMax) {
            thePidData->integratorValue = thePidData->integratorMax;
        } else if (thePidData->integratorValue < -thePidData->integratorMax) {
            thePidData->integratorValue = -thePidData->integratorMax;
        }
    }
    thePidData->lastUpdate = t;

    //we use -dy because it is equivalent to -de for constant w
    return thePidData->kp * e + thePidData->ki * thePidData->integratorValue + thePidData->kd * -dy;
}
