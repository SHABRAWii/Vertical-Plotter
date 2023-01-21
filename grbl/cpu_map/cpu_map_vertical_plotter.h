/*
  cpu_map_custom.h - CPU and pin mapping configuration file
  Part of Vertical_Plotter

  Copyright (c) 2023 Youssef El Shabrawii

  Vertical Plotter is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Vertical Plotter is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Grbl officially supports the Arduino Uno, but the other supplied pin mappings are
   supplied by users, so your results may vary. This cpu_map file serves as a central
   pin mapping settings file for AVR 328p used on the Arduino Uno.  */
   
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "Vertical_Plotter"

// Define serial port pins and interrupt vectors.
#define SERIAL_RX     USART_RX_vect // (USART, Rx Complete): Name of ISR in the AVR microcontroller architecture that is triggered when a byte of data is received by the UART module.
#define SERIAL_UDRE   USART_UDRE_vect // (USART, Data Register Empty): Name of ISR in the AVR microcontroller architecture that is triggered when the UDR (USART Data Register) is empty and ready to receive new data for transmission.

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.
#define STEP_DDR        DDRD
#define STEP_PORT       PORTD
#define X_STEP_BIT      3  // Digital Pin 3
#define Y_STEP_BIT      5  // Digital Pin 5
#define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_DDR     DDRD
#define DIRECTION_PORT    PORTD
#define X_DIRECTION_BIT   4  // Digital Pin 4
#define Y_DIRECTION_BIT   6  // Digital Pin 6
#define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)) // All direction bits

#define Servo_DDR     DDRB
#define Servo_PORT    PORTB
#define Servo_BIT     3  // Digital Pin 11
