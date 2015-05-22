/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            30
#define NUM_ANALOG_INPUTS           3
#define analogInputToDigitalPin(p)  (((p)>=14 && (p) <= 15) ? (p) + 13 : ((p == 17) ? 29 : -1))
#define digitalPinHasPWM(p)         (((p) >= 2 && (p) <= 5) || ((p) >= 10 && (p)<= 13))

static const uint8_t SS   = 23;
static const uint8_t MOSI = 19;
static const uint8_t MISO = 18;
static const uint8_t SCK  = 22;

static const uint8_t SDA = 20;
static const uint8_t SCL = 21;
#define LED_BUILTIN 13

// Digital Pins: 27,28,29. Analog pin value is kept as 14,15,17 inorder to match with Arduino core definition
// See wiring_analog.c
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 17;

// Pins: 0,10, 11, 12, 18, 19, 22

#define digitalPinToPCICR(p)    ( (((p) == 0)) || \
                                  (((p) >= 10) && ((p) <= 12)) || \
                                  (((p) >= 18) && ((p) <= 19)) || \
                                  (((p) == 22)) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 18) && ((p) <= 19)) || \
                                  (((p) >= 22) && ((p) <= 23))? 0 : \
                                ( (((p) == 0)) ? 1 : 0 ) )

#define digitalPinToPCMSK(p)    ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 18) && ((p) <= 19)) || \
                                  (((p) >= 22) && ((p) <= 23)) ? (&PCMSK0) : \
                                ( (((p) == 0)) ? (&PCMSK1) : \
                                ((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) ( (((p) >= 22) && ((p) <= 23)) ? ((p) - 22) : \
                                ( (((p) >= 18) && ((p) <= 19)) ? (21 - (p)) : \
                                ( (((p) >= 11) && ((p) <= 12)) ? ((p) - 6) : \
                                ( ((p) == 10) ? 7 : \
                                ( ((p) == 13) ? 4 : \
                                ( ((p) == 0) ? 0 : \
                                0 ) ) ) ) ) )

 // Pins: 21,20,17,16,2,3,7,8
#define digitalPinToInterrupt(p) ( (p) >= 20 && (p) <= 21 ? 21 - (p) : \
                                   ((p) >= 16 && (p) <= 17 ? 19 - (p) : \
								   ((p) >= 2 && (p) <= 3 ? (p) + 2 : \
								   ((p) >= 7 && (p) <= 8 ? (p) - 1 : NOT_AN_INTERRUPT))))

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	NOT_A_PORT,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	NOT_A_PORT,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	NOT_A_PIN,
	(uint16_t) &PINB,
	NOT_A_PIN,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PE	, // PE 0 ** 0 ** USART0_RX	
	PE	, // PE 1 ** 1 ** USART0_TX	
	PE	, // PE 4 ** 2 ** PWM0	
	PE	, // PE 5 ** 3 ** PWM1	
	PG	, // PG 5 ** 4 ** PWM2	
	PE	, // PE 3 ** 5 ** PWM3	
	PE	, // PE 2 ** 6 ** 	
	PE	, // PE 6 ** 7 ** 	
	PE	, // PE 7 ** 8 ** 	
	PD	, // PD 4 ** 9 ** 	
	PB	, // PB 7 ** 10 ** PWM4	
	PB	, // PB 5 ** 11 ** PWM5	
	PB	, // PB 6 ** 12 ** PWM6	
	PB	, // PB 4 ** 13 ** PWM7	
	PD	, // PD 5 ** 14 ** 
	PD	, // PD 6 ** 15 ** 	
	PD	, // PD 3 ** 16 ** USART1_TX	
	PD	, // PD 2 ** 17 ** USART1_RX	
	PB	, // PB 3 ** 18 ** SPI_MISO	
	PB	, // PB 2 ** 19 ** SPI_MOSI	
	PD	, // PD 1 ** 20 ** I2C_SDA	
	PD	, // PD 0 ** 21 ** I2C_SCL	
	PB	, // PB 1 ** 22 ** SPI_SCK	
	PB	, // PB 0 ** 23 ** SPI_SS	
	PG	, // PG 2 ** 24 ** 	
	PG	, // PG 0 ** 25 ** 	
	PD	, // PD 7 ** 26 ** 		
	PF	, // PF 0 ** 27 ** A0	
	PF	, // PF 1 ** 28 ** A1	
	PF	, // PF 3 ** 29 ** A2		
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PE 0 ** 0 ** USART0_RX	
	_BV( 1 )	, // PE 1 ** 1 ** USART0_TX	
	_BV( 4 )	, // PE 4 ** 2 ** PWM	
	_BV( 5 )	, // PE 5 ** 3 ** PWM	
	_BV( 5 )	, // PG 5 ** 4 ** PWM	
	_BV( 3 )	, // PE 3 ** 5 ** PWM	
	_BV( 2 )	, // PE 2 ** 6 ** 	
	_BV( 6 )	, // PE 6 ** 7 ** 	
	_BV( 7 )	, // PE 7 ** 8 ** 	
	_BV( 4 )	, // PD 4 ** 9 ** 	
	_BV( 7 )	, // PB 7 ** 10 ** PWM	
	_BV( 5 )	, // PB 5 ** 11 ** PWM	
	_BV( 6 )	, // PB 6 ** 12 ** PWM	
	_BV( 4 )	, // PB 4 ** 13 ** PWM	
	_BV( 5 )	, // PD 5 ** 14 ** 
	_BV( 6 )	, // PD 6 ** 15 ** 	
	_BV( 3 )	, // PD 3 ** 16 ** USART1_TX	
	_BV( 2 )	, // PD 2 ** 17 ** USART1_RX	
	_BV( 3 )	, // PB 3 ** 18 ** SPI_MISO	
	_BV( 2 )	, // PB 2 ** 19 ** SPI_MOSI	
	_BV( 1 )	, // PD 1 ** 20 ** I2C_SDA	
	_BV( 0 )	, // PD 0 ** 21 ** I2C_SCL	
	_BV( 1 )	, // PB 1 ** 22 ** SPI_SCK	
	_BV( 0 )	, // PB 0 ** 23 ** SPI_SS	
	_BV( 2 )	, // PG 2 ** 24 ** 		
	_BV( 0 )	, // PG 0 ** 25 ** 	
	_BV( 7 )	, // PD 7 ** 26 ** 		
	_BV( 0 )	, // PF 0 ** 27 ** A0	
	_BV( 1 )	, // PF 1 ** 28 ** A1	
	_BV( 3 )	, // PF 3 ** 29 ** A2	
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------
	NOT_ON_TIMER	, // PE 0 ** 0 ** USART0_RX	
	NOT_ON_TIMER	, // PE 1 ** 1 ** USART0_TX	
	TIMER3B	, // PE 4 ** 2 ** PWM	
	TIMER3C	, // PE 5 ** 3 ** PWM	
	TIMER0B	, // PG 5 ** 4 ** PWM	
	TIMER3A	, // PE 3 ** 5 ** PWM	
	NOT_ON_TIMER	, // PE 2 ** 6 ** 	
	NOT_ON_TIMER	, // PE 6 ** 7 ** 	
	NOT_ON_TIMER	, // PE 7 ** 8 ** 	
	NOT_ON_TIMER	, // PD 4 ** 9 ** 	
	TIMER0A	, // PB 7 ** 10 ** PWM	
	TIMER1A	, // PB 5 ** 11 ** PWM	
	TIMER1B	, // PB 6 ** 12 ** PWM	
	TIMER2A	, // PB 4 ** 13 ** PWM	
	NOT_ON_TIMER	, // PD 5 ** 14 ** 
	NOT_ON_TIMER	, // PD 6 ** 15 ** 	
	NOT_ON_TIMER	, // PD 3 ** 16 ** USART1_TX	
	NOT_ON_TIMER	, // PD 2 ** 17 ** USART1_RX	
	NOT_ON_TIMER	, // PB 3 ** 18 ** SPI_MISO	
	NOT_ON_TIMER	, // PB 2 ** 19 ** SPI_MOSI	
	NOT_ON_TIMER	, // PD 1 ** 20 ** I2C_SDA	
	NOT_ON_TIMER	, // PD 0 ** 21 ** I2C_SCL	
	NOT_ON_TIMER	, // PB 1 ** 22 ** SPI_SCK	
	NOT_ON_TIMER	, // PB 0 ** 23 ** SPI_SS	
	NOT_ON_TIMER	, // PG 2 ** 24 ** 		
	NOT_ON_TIMER	, // PG 0 ** 25 ** 	
	NOT_ON_TIMER	, // PD 7 ** 26 ** 		
	NOT_ON_TIMER	, // PF 0 ** 27 ** A0	
	NOT_ON_TIMER	, // PF 1 ** 28 ** A1	
	NOT_ON_TIMER	, // PF 3 ** 29 ** A2
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial1
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#define HAL_ATMEGA256RFR2
#define PHY_ATMEGARFR2
#define PLATFORM_XPLAINED_PRO_ATMEGA256RFR2
#endif
