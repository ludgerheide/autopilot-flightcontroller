/*! \file global.h \brief AVRlib project global include. */
//*****************************************************************************
//
// File Name	: 'global.h'
// Title		: AVRlib project global include 
// Author		: Pascal Stang - Copyright (C) 2001-2002
// Created		: 7/12/2001
// Revised		: 9/30/2002
// Version		: 1.1
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
//	Description : This include file is designed to contain items useful to all
//					code files and projects.
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#ifndef GLOBAL_H
#define GLOBAL_H

// global AVRLIB defines
#include "../avrlib/avrlibdefs.h"
// global AVRLIB types definitions
#include "../avrlib/avrlibtypes.h"

#include "../sensors/bmp180.h"
#include "../sensors/battery.h"
#include "../sensors/bmp280.h"

#include <avr/wdt.h>

// project/system dependent defines

// CPU clock speed
//#define F_CPU        16000000               		// 16MHz processor
//#define F_CPU        14745000               		// 14.745MHz processor
//#define F_CPU        8000000               		// 8MHz processor
//#define F_CPU        7372800               		// 7.37MHz processor
//#define F_CPU        4000000               		// 4MHz processor
//#define F_CPU        3686400               		// 3.69MHz processor
#define CYCLES_PER_US ((F_CPU+500000)/1000000)    // cpu cycles per microsecond

#define WDTO_DEFAULT WDTO_30MS

typedef struct {
    u64 timestamp;

    u16 courseMagnetic; //64*degrees
    s16 pitch;
    s16 roll;
} attitude_struct;
attitude_struct currentAttitude;

typedef struct {
    u64 timestamp;

    s16 x;
    s16 y;
    s16 z;
} xyz;

attitude_struct currentAttitude;
xyz gravity, linearAcceleration, angularVelocity;

pressureEvent staticPressure, pitotPressure;
altitudeData myAltitudeData;
airspeed_struct myAirspeed;
batteryEvent curBattery;
#endif
