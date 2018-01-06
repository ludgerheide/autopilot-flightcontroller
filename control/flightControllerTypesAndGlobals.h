//
//  flightControllerTypes.h
//  autopilot
//
//  Created by Ludger Heide on 08.04.16.
//  Copyright © 2016 Ludger Heide. All rights reserved.
//

#ifndef flightControllerTypes_h
#define flightControllerTypes_h

#include "../avrlib/avrlibtypes.h"
#include "../protobuf/communicationProtocol.pb.h"

typedef struct {
    u64 timestamp;

    u08 yaw;
    u08 pitch;
    u08 thrust;
} commandSet_struct;
commandSet_struct outputCommandSet;

DroneMessage_CommandUpdate commandUpdate;

typedef struct {
    u64 timestamp;

    float latitude;
    float longitude;
    s32 altitude; //CentiMeters ASL
} waypoint;
waypoint homeBase;

DroneMessage_FlightMode currentFlightMode;

#endif /* flightControllerTypes_h */
