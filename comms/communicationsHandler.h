//
//  communicationsHandler.h
//  autopilot
//
//  Created by Ludger Heide on 06.10.15.
//  Copyright © 2015 Ludger Heide. All rights reserved.
//

#ifndef communicationsHandler_h
#define communicationsHandler_h

#include <stdio.h>
#include "../avrlib/avrlibtypes.h"

#include "../protobuf/pb_decode.h"
#include "../protobuf/pb_encode.h"
#include "../protobuf/pb_common.h"

#include "xBee.h"
#include "../protobuf/communicationProtocol.pb.h"

//Initializes the communications (the xBee serial and the RTS output pin)
void commsInit(void);

void commsProcessMessage(char *message, u08 size);

//Telemetry goes out over the radio and is a small packet with posoition, velocity and attitude
int commsCheckAndSendTelemetry(void);

//Logging goes out over the serial port and contains *a lot* of data
int commsCheckAndSendLogging(void);

//Called when a status is received
void txStatusHandler(uint8_t frameID, __attribute__ ((unused)) uint8_t retryCount,
                     __attribute__ ((unused)) uint8_t txStatus);


#endif /* communicationsHandler_h */
