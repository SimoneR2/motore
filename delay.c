/*******************************************************************************

Author : Mauro Laurenti
Version : 1.4

Created on Date : 04/09/2006
Last update     : 15/12/2014

CopyRight 2006-2014 all rights are reserved

********************************************************
SOFTWARE LICENSE AGREEMENT
********************************************************

The usage of the supplied software imply the acceptance of the following license.

The software supplied herewith by Mauro Laurenti (the Author) is intended for
use solely and exclusively on Microchip PIC Microcontroller (registered mark).
The software is owned by the Author, and is protected under applicable
copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to civil liability
for the breach of the terms and conditions of this license.
Commercial use is forbidden without a written acknowledgement with the Author.
Personal or educational use is allowed if the application containing the
following software doesn't aim to commercial use or monetary earning of any kind.

THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE AUTHOR SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

*******************************************************************************/


#ifdef __XC8
    #include <xc.h>
#endif

#ifndef __XC8
  #include <delays.h>
#endif


#include "delay.h"

volatile unsigned char delay_quartz_frequency_value = 8;
volatile unsigned int clock_counter_reference = 0;

#ifdef DELAY_INTERRUPT_BLOCKING
    volatile unsigned char delay_flag_GIE = 0;
    volatile unsigned char delay_flag_PEIE = 0;
#endif

//************************************************************
//             delay_ms function implementation
//************************************************************

void delay_ms (unsigned int value_ms) {

        #ifdef DELAY_INTERRUPT_BLOCKING
            delay_flag_GIE = INTCONbits.GIE;
            // Disable Interrupt
            INTCONbits.GIE = 0;

            delay_flag_PEIE = INTCONbits.PEIE;
            // Disable Interrupt
            INTCONbits.PEIE = 0;
	#endif	     

	clock_counter_reference = value_ms * delay_quartz_frequency_value;

        while (clock_counter_reference) {
            #ifndef __XC8
                //1ms Delay at 1MHz trimmed down to 24 instead of 25
                //to compensate the main loop
                Delay10TCYx (24);
            #endif

            #ifdef __XC8
                //1ms Delay at 1MHz trimmed down to 240 instead of 250
                //to compensate the main loop
                _delay(240);
            #endif

            clock_counter_reference--;
	}

        #ifdef DELAY_INTERRUPT_BLOCKING
            // Reload old settings
            INTCONbits.GIE = delay_flag_GIE;
            INTCONbits.PEIE = delay_flag_PEIE;
        #endif
}

//************************************************************
//             delay_s function implementation
//************************************************************

void delay_s (unsigned char value_s) {

	unsigned char repeat_loop;

	for (repeat_loop = 0; repeat_loop < value_s; repeat_loop++)
		delay_ms (1000);
}


//************************************************************
//             setQuartz function implementation
//************************************************************

void delay_set_quartz (unsigned char frequency) {

	delay_quartz_frequency_value = frequency;
	
} 

