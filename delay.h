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


********************************************************
PURPOSES
********************************************************


 This library allows to execute long delay independently from the used clock.
 The clock can be set at run time (no compiling time).

*******************************************************************************/



#ifndef DELAY_H
#define DELAY_H

#ifdef __XC8
#include <xc.h>
#endif

// Remove the comment to the DELAY_INTERRUPT_BLOCKING definition if the delay
// should lock the interrupt.
// This feature make the delay function "almost" thread safe, in case they are
// used within the main function and within the ISR at the same time.

//#define DELAY_INTERRUPT_BLOCKING

/**
 * This function creates a delay in ms up to 1 second.
 *
 * @param value_ms Delay expressed in ms [min = 1, max = 1000]  
 *
 * @return void
 *
 * @note This is a blocking function.
 * @note Typical error is between 1%-2% depending on the compiler optimization.
 *       Tests have been done using 20MHz and the Free version of C18 and XC8 compilers.
 */

void delay_ms (unsigned int value_ms);


/**
 * This function creates a delay in seconds, up to 255s.
 *
 * @param value_s Delay expressed in s [min = 1, max = 255]  
 *
 * @return void
 *
 * @note This is a blocking function.
 * @note Typical error is between 1%-2% depending on the compiler optimization.
 *       Tests have been done using 20MHz and the Free version of C18 and XC8 compilers.
 */
void delay_s (unsigned char value_s);


/**
 * This function sets the frequency value used to run the CPU.
 *
 * @param frequency Frequency expressed in MHz (integer)
 *
 * @return void
 *
 * @note If this function is not called, the frequency default value is 20MHz.
 * @note Fractional values are not supported.
 */

void delay_set_quartz (unsigned char frequency);
#define setQuartz delay_set_quartz

#endif