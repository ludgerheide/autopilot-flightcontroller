/*! \file i2c.c \brief I2C interface using AVR Two-Wire Interface (TWI) hardware. */
//*****************************************************************************
//
// File Name	: 'i2c.c'
// Title		: I2C interface using AVR Two-Wire Interface (TWI) hardware
// Author		: Pascal Stang - Copyright (C) 2002-2003
// Created		: 2002.06.25
// Revised		: 2003.03.02
// Version		: 0.9
// Target MCU	: Atmel AVR series
// Editor Tabs	: 4
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>
#include <avr/interrupt.h>

#include "i2c.h"

#ifdef I2C_DEBUG
#include <stdio.h> // include printf function library
#endif

// Standard I2C bit rates are:
// 100KHz for slow speed
// 400KHz for high speed

//#define I2C_DEBUG

// I2C state and address variables
volatile eI2cStateType I2cState;
static u08 I2cDeviceAddrRW;
// send/transmit buffer (outgoing data)
static u08 I2cSendData[I2C_SEND_DATA_BUFFER_SIZE];
static u08 I2cSendDataIndex;
static u08 I2cSendDataLength;
// receive buffer (incoming data)
volatile u08 I2cReceiveData[I2C_RECEIVE_DATA_BUFFER_SIZE];
volatile u08 I2cReceiveDataIndex;
volatile u08 I2cReceiveDataLength;

// function pointer to i2c receive routine
//! I2cSlaveReceive is called when this processor
// is addressed as a slave for writing
static void (*i2cSlaveReceive)(u08 receiveDataLength, u08* recieveData);
//! I2cSlaveTransmit is called when this processor
// is addressed as a slave for reading
static u08 (*i2cSlaveTransmit)(u08 transmitDataLengthMax, u08* transmitData);
//! I2cSTopHandler is called when a transmission is completed (after sending the stop)
static void (*i2cStopHandler)(u08 statusRegister, u08 deviceAddress);

// functions
void i2cInit(void) {

#ifdef I2C_USE_INT_PULLUP_RESISTORS
	// set pull-up resistors on I2C bus pins
#if (defined (__AVR_ATmega64C1__) || defined (__AVR_ATmega64M1__) ||\
	defined (__AVR_ATmega128__) || defined (__AVR_ATmega1280__) ||\
	defined (__AVR_ATmega1281__) || defined (__AVR_ATmega1284P__) ||\
	defined (__AVR_ATmega128RFA1__) || defined(__AVR_ATmega2560__))

	sbi(PORTD, 0);	// i2c SCL on ATmega128,64
	sbi(PORTD, 1);	// i2c SDA on ATmega128,64
#elif (defined (__AVR_ATmega8__) || defined (__AVR_ATmega8A__))
    sbi(PORTC, 5); // i2c SCL on ATmega8
    sbi(PORTC, 4); // i2c SDA on ATmega8
#else
	sbi(PORTC, 0);	// i2c SCL on ATmega163,323,16,32,etc
	sbi(PORTC, 1);	// i2c SDA on ATmega163,323,16,32,etc
#endif
#endif

	// clear SlaveReceive and SlaveTransmit handler to null
	i2cSlaveReceive = 0;
	i2cSlaveTransmit = 0;
    i2cStopHandler = 0;
	// set i2c bit rate to 100KHz
	i2cSetBitrate(100);
	// enable TWI (two-wire interface)
	sbi(TWCR, TWEN);
	// set state
	I2cState = I2C_IDLE;
	// enable TWI interrupt and slave address ACK
	sbi(TWCR, TWIE);
	sbi(TWCR, TWEA);
	//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	// enable interrupts
	sei();
}

void i2cSetBitrate(u16 bitrateKHz) {
	u08 bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	#ifdef TWPS0
		// for processors with additional bitrate division (mega128)
		// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
		// set TWPS to zero
		cbi(TWSR, TWPS0);
		cbi(TWSR, TWPS1);
	#endif
	// calculate bitrate division	
	bitrate_div = ((F_CPU/1000l)/bitrateKHz);
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;
	outb(TWBR, bitrate_div);
}

void i2cSetLocalDeviceAddr(u08 deviceAddr, u08 genCallEn) {
	// set local device address (used in slave mode only)
	outb(TWAR, ((deviceAddr<<1) | (genCallEn?1:0)) );
}

void i2cSetSlaveReceiveHandler(void (*i2cSlaveRx_func)(u08 receiveDataLength, u08* recieveData)) {
	i2cSlaveReceive = i2cSlaveRx_func;
}

void i2cSetSlaveTransmitHandler(u08 (*i2cSlaveTx_func)(u08 transmitDataLengthMax, u08* transmitData)) {
	i2cSlaveTransmit = i2cSlaveTx_func;
}

void i2cSetStopHandler(void (*i2cStopHandler_func)(u08 statusRegister, u08 deviceAddress)) {
    i2cStopHandler = i2cStopHandler_func;
}

inline void i2cSendStart(void) {
	// send start condition
	outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWSTA));
}

inline void i2cSendStop(void) {
    //If a custom stop handler is defined, call it
    if(i2cStopHandler) {
        u08 status = inb(TWSR) & TWSR_STATUS_MASK;
        u08 deviceAddress = I2cDeviceAddrRW >> 1; //Lowest bit removed
        i2cStopHandler(status, deviceAddress);
    } else {
        // transmit stop condition
        // leave with TWEA on for slave receiving
        outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA)|BV(TWSTO));
        
        // set state
        I2cState = I2C_IDLE;
    }
}

inline void i2cWaitForComplete(void) {
	// wait for i2c interface to complete operation
	while( !(inb(TWCR) & BV(TWINT)) );
}

inline void i2cSendByte(u08 data) {
	// save data to the TWDR
	outb(TWDR, data);
	// begin send
	outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
}

inline void i2cReceiveByte(u08 ackFlag) {
	// begin receive over i2c
	if( ackFlag ) {
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else {
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}

inline u08 i2cGetReceivedByte(void) {
	// retieve received data byte from i2c TWDR
	return( inb(TWDR) );
}

inline u08 i2cGetStatus(void) {
	// retieve current i2c status from i2c TWSR
	return( inb(TWSR) );
}

void i2cMasterSend(u08 deviceAddr, u08 length, u08* data) {
	u08 i;
	// wait for interface to be ready
	while(I2cState);
	// set state
	I2cState = I2C_MASTER_TX;
	// save data
	I2cDeviceAddrRW = ((deviceAddr << 1) & 0xFE);	// RW cleared: write operation
	for(i=0; i<length; i++)
		I2cSendData[i] = *data++;
	I2cSendDataIndex = 0;
	I2cSendDataLength = length;
	// send start condition
	i2cSendStart();
}

void i2cMasterReceive(u08 deviceAddr, u08 length, u08* data) {
	u08 i;
	// wait for interface to be ready
	while(I2cState);
	// set state
	I2cState = I2C_MASTER_RX;
	// save data
	I2cDeviceAddrRW = ((deviceAddr << 1)|0x01);	// RW set: read operation
	I2cReceiveDataIndex = 0;
	I2cReceiveDataLength = length;
	// send start condition
	i2cSendStart();
	// wait for data
	while(I2cState);
	// return data
	for(i=0; i<length; i++)
		*data++ = I2cReceiveData[i];
}

void i2cMasterStartReceiving(u08 deviceAddr, u08 length) {
    // wait for interface to be ready
    while(I2cState);
    // set state
    I2cState = I2C_MASTER_RX;
    // save data
    I2cDeviceAddrRW = ((deviceAddr << 1)|0x01);	// RW set: read operation
    I2cReceiveDataIndex = 0;
    I2cReceiveDataLength = length;
    // send start condition
    i2cSendStart();
}

u08 i2cMasterSendNI(u08 deviceAddr, u08 length, u08* data) {
	u08 retval = I2C_OK;

	// disable TWI interrupt
	cbi(TWCR, TWIE);

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// send device address with write
	i2cSendByte((deviceAddr << 1) & 0xFE );
	i2cWaitForComplete();

	// check if device is present and live
	if( inb(TWSR) == TW_MT_SLA_ACK) {
		// send data
		while(length) {
			i2cSendByte( *data++ );
			i2cWaitForComplete();
			length--;
		}
	}
    else {
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();
	while( !(inb(TWCR) & BV(TWSTO)) );

	// enable TWI interrupt
	sbi(TWCR, TWIE);

	return retval;
}

u08 i2cMasterReceiveNI(u08 deviceAddr, u08 length, u08 *data) {
	u08 retval = I2C_OK;

	// disable TWI interrupt
	cbi(TWCR, TWIE);

	// send start condition
	i2cSendStart();
	i2cWaitForComplete();

	// send device address with read
	i2cSendByte((deviceAddr << 1) | 0x01 );
	i2cWaitForComplete();

	// check if device is present and live
	if( inb(TWSR) == TW_MR_SLA_ACK)
	{
		// accept receive data and ack it
		while(length > 1)
		{
			i2cReceiveByte(TRUE);
			i2cWaitForComplete();
			*data++ = i2cGetReceivedByte();
			// decrement length
			length--;
		}

		// accept receive data and nack it (last-byte signal)
		i2cReceiveByte(FALSE);
		i2cWaitForComplete();
		*data++ = i2cGetReceivedByte();
	}
	else
	{
		// device did not ACK it's address,
		// data will not be transferred
		// return error
		retval = I2C_ERROR_NODEV;
	}

	// transmit stop condition
	// leave with TWEA on for slave receiving
	i2cSendStop();

	// enable TWI interrupt
	sbi(TWCR, TWIE);

	return retval;
}

//! I2C (TWI) interrupt service routine
ISR(TWI_vect) {
	// read status bits
	u08 status = inb(TWSR) & TWSR_STATUS_MASK;

	switch(status) {
	// Master General
	case TW_START:						// 0x08: Sent start condition
	case TW_REP_START:					// 0x10: Sent repeated start condition
		#ifdef I2C_DEBUG
		printf("I2C: M->START\r\n");
		#endif
		// send device address
		i2cSendByte(I2cDeviceAddrRW);
		break;
	
	// Master Transmitter & Receiver status codes
	case TW_MT_SLA_ACK:					// 0x18: Slave address acknowledged
	case TW_MT_DATA_ACK:				// 0x28: Data acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MT->SLA_ACK or DATA_ACK\r\n");
		#endif
		if(I2cSendDataIndex < I2cSendDataLength) {
			// send data
			i2cSendByte( I2cSendData[I2cSendDataIndex++] );
		}
		else {
			// transmit stop condition, enable SLA ACK
			i2cSendStop();
		}
		break;
	case TW_MR_DATA_NACK:				// 0x58: Data received, NACK reply issued
		#ifdef I2C_DEBUG
		printf("I2C: MR->DATA_NACK\r\n");
		#endif
		// store final received data byte
		I2cReceiveData[I2cReceiveDataIndex++] = inb(TWDR);
		// continue to transmit STOP condition
	case TW_MR_SLA_NACK:				// 0x48: Slave address not acknowledged
	case TW_MT_SLA_NACK:				// 0x20: Slave address not acknowledged
	case TW_MT_DATA_NACK:				// 0x30: Data not acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MTR->SLA_NACK or MT->DATA_NACK\r\n");
		#endif
		// transmit stop condition, enable SLA ACK
		i2cSendStop();
		break;
	case TW_MT_ARB_LOST:				// 0x38: Bus arbitration lost
	//case TW_MR_ARB_LOST:				// 0x38: Bus arbitration lost
		#ifdef I2C_DEBUG
		printf("I2C: MT->ARB_LOST\r\n");
		#endif
		// release bus
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
		// set state
		I2cState = I2C_IDLE;
		// release bus and transmit start when bus is free
		//outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWSTA));
		break;
	case TW_MR_DATA_ACK:				// 0x50: Data acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MR->DATA_ACK\r\n");
		#endif
		// store received data byte
		I2cReceiveData[I2cReceiveDataIndex++] = inb(TWDR);
		// fall-through to see if more bytes will be received
	case TW_MR_SLA_ACK:					// 0x40: Slave address acknowledged
		#ifdef I2C_DEBUG
		printf("I2C: MR->SLA_ACK\r\n");
		#endif
		if(I2cReceiveDataIndex < (I2cReceiveDataLength-1))
			// data byte will be received, reply with ACK (more bytes in transfer)
			i2cReceiveByte(TRUE);
		else
			// data byte will be received, reply with NACK (final byte in transfer)
			i2cReceiveByte(FALSE);
		break;

	// Misc
	case TW_NO_INFO:					// 0xF8: No relevant state information
		// do nothing
		#ifdef I2C_DEBUG
		printf("I2C: NO_INFO\r\n");
		#endif
		break;
	case TW_BUS_ERROR:					// 0x00: Bus error due to illegal start or stop condition
		#ifdef I2C_DEBUG
		printf("I2C: BUS_ERROR\r\n");
		#endif
		// reset internal hardware and release bus
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWSTO)|BV(TWEA));
		// set state
		I2cState = I2C_IDLE;
		break;
	}
}

eI2cStateType i2cGetState(void) {
	return I2cState;
}