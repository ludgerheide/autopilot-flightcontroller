//
//  flightController.h
//  autopilot
//
//  Created by Ludger Heide on 13.11.15.
//  Copyright © 2015 Ludger Heide. All rights reserved.
//

#ifndef flightController_h
#define flightController_h

#include <stdio.h>
#include "../protobuf/communicationProtocol.pb.h"

void flightControllerInit(void);

DroneMessage_FlightMode checkSensorsAndSetFlightMode(void);

void updateFlightControls(void);

#endif /* flightController_h */
