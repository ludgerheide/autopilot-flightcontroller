//
//  battery.h
//  autopilot
//
//  Created by Ludger Heide on 07.11.15.
//  Copyright © 2015 Ludger Heide. All rights reserved.
//

#ifndef battery_h
#define battery_h

#include "../avrlib/avrlibtypes.h"

//Structs
typedef struct {
    u64 timestamp; //microseconds

    u16 voltage; //mV
    u16 current; //mA
} batteryEvent;

//Initializes the ADC. Setd everything up and starts the first volatage measurement
void batteryInit(void);

//fills a batterys event withg current and voltage
void batteryGetData(batteryEvent *event);

#endif /* battery_h */
