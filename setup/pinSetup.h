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
#define GPS_UART 2
#define UART2_TX_BUFFER_SIZE (82) //An NMEA msg is 82 chars long, we may have three in the buffer
#define UART2_RX_BUFFER_SIZE (82*3)
#define RASPI_UART 0
#define UART0_TX_BUFFER_SIZE (512+7)
#define UART0_RX_BUFFER_SIZE (128)

#define UART1_TX_BUFFER_SIZE (1) //This uart is not in use
#define UART1_RX_BUFFER_SIZE (1)

//The Servo numbers
#define SERVO1_PORT PORTB
#define SERVO2_PORT PORTB
#define SERVO3_PORT PORTB

#define YAW_SERVO_PORT PORTB
#define YAW_SERVO_DDR DDRB
#define YAW_SERVO_PIN 6
#define YAW_SERVO_CHAN 0

#define PITCH_SERVO_PORT PORTB
#define PITCH_SERVO_DDR DDRB
#define PITCH_SERVO_PIN 5
#define PITCH_SERVO_CHAN 1

#define THRUST_SERVO_PORT PORTL
#define THRUST_SERVO_DDR DDRL
#define THRUST_SERVO_PIN 6
#define THRUST_SERVO_CHAN 2

//The adc channels
#define VOLTAGE_CHANNEL 1
#define CURRENT_CHANNEL 8

#endif /* pinSetup_h */