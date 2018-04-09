//
//  pinSetup.h
//  avr - flyByWire
//
//  Created by Ludger Heide on 23.09.15.
//  Copyright © 2015 LH Technologies. All rights reserved.
//

#ifndef pinSetup_h
#define pinSetup_h

//UART Numbers and ther functions
#define XBEE_UART 3
#define UART3_TX_BUFFER_SIZE (100) //Those messages should be kinda short
#define UART3_RX_BUFFER_SIZE (100)
#define GPS_UART 0
#define UART0_TX_BUFFER_SIZE (82) //An NMEA msg is 82 chars long, we may have three in the buffer
#define UART0_RX_BUFFER_SIZE (82*3)
#define RASPI_UART 1
#define UART1_TX_BUFFER_SIZE (512+7)
#define UART1_RX_BUFFER_SIZE (128)

#define UART2_TX_BUFFER_SIZE (1) //This uart is not in use
#define UART2_RX_BUFFER_SIZE (1)

//The Servo numbers

#define YAW_SERVO_PORT PORTK
#define YAW_SERVO_DDR DDRK
#define YAW_SERVO_PIN 0

#define PITCH_SERVO_PORT PORTK
#define PITCH_SERVO_DDR DDRK
#define PITCH_SERVO_PIN 1

#define THRUST_SERVO_PORT PORTK
#define THRUST_SERVO_DDR DDRK
#define THRUST_SERVO_PIN 2

#define AILERON_R_SERVO_PORT PORTK
#define AILERON_R_SERVO_DDR DDRK
#define AILERON_R_SERVO_PIN 3

#define AILERON_L_SERVO_PORT PORTK
#define AILERON_L_SERVO_DDR DDRK
#define AILERON_L_SERVO_PIN 4


//The adc channels
#define VOLTAGE_CHANNEL 1
#define CURRENT_CHANNEL 0

#endif /* pinSetup_h */