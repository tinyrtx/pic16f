//*******************************************************************************
// tinyRTX Filename: sisd.c (System Interrupt Service Director)
//
// Copyright 2014 Sycamore Software, Inc.  ** www.tinyRTX.com **
// Distributed under the terms of the GNU Lesser General Purpose License v3
//
// This file is part of tinyRTX. tinyRTX is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// version 3 as published by the Free Software Foundation.
//
// tinyRTX is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General Public License
// (filename copying.lesser.txt) and the GNU General Public License (filename
// copying.txt) along with tinyRTX.  If not, see <http://www.gnu.org/licenses/>.
//
// Revision history:
//  06Oct15 Stephen_Higgins@KairosAutonomi.com
//              Convert from pic18 C18 to pic16 XC8.
//
//*******************************************************************************

#include    "ucfg.h"    // Configure board and proc, #include <proc.h>
#include    "srtx.h"
#include    "susr.h"
#include    "ssio.h"
//#include    "si2c.h"

//void SISD_Interrupt( void );

//*******************************************************************************
//
// SISD service variables.
//
// Interrupt Service Routine context save/restore variables.
//
// LINKNOTE: SISD_UdataShrSec must be placed in data space shared across all banks.
//          This is because to save/restore STATUS register properly, we can't control
//          RP1 and RP0 bits.  So any values in RP1 and RP0 must be valid.  In order to
//          allow this we need memory which accesses the same across all banks.

//*******************************************************************************

void interrupt SISD_Interrupt( void )
{
//*******************************************************************************
//
// SISD: System Interrupt Service Director.
//
// 5 possible sources of interrupts (in "priority" order):
//   a) RS-232 Receive byte. (Move byte from HW to receive buffer.)
//   b) RS-232 Transmit byte. (Move byte (if there is one) from transmit buffer to HW.)
//   c) I2C event completed. (Multiple I2C events to transmit ASCII.)
//   d) A/D conversion completed. (Convert reading to ASCII, initiate PWM.)
//   e) Timer0 expires. (Finish PWM.)
//   f) Timer1 expires. (Initiate A/D, new Timer1.)
//
//*******************************************************************************
//
// Test for completion of SIO receive event.
//
//   NOTE: Because transmitting device not looking for flow control, this interrupt has to be
//   checked first and finish fast.
//
    if (PIR1bits.RCIF && PIE1bits.RCIE) // If RC int flag set and RC int is enabled..
    {
        PIR1bits.RCIF = 0;              // ..then clear RC interrupt flag.
        SSIO_GetByteFromRcHW();         // System ISR handling when SIO_Rx event.
    }
    else
    {
//
// Test for completion of SIO transmit event.
//
    if (PIR1bits.TXIF && PIE1bits.TXIE) // If TX int flag set and TX int is enabled..
    {
        PIR1bits.TXIF = 0;              // ..then clear TX interrupt flag.
        SSIO_PutByteIntoTxHW();         // System ISR handling when SIO_Tx event.
    }
    else
    {
//
// Test for completion of I2C event.
//
    if (PIR1bits.SSP1IF)                // If I2C interrupt flag set..
    {
        PIR1bits.SSP1IF = 0;            // ..then clear I2C interrupt flag.
//        SI2C_Tbl_HwState();             // System ISR handling when I2C event.
    }
    else
    {
//
// Test for completion of A/D conversion.
//
    if (PIR1bits.ADIF)                      // If A/D interrupt flag set..
    {
        PIR1bits.ADIF = 0;                  // ..then clear A/D interrupt flag.
        PIE1bits.ADIE = 0;                  // Disable A/D interrupts.
        if( ++SRTX_Sched_Cnt_TaskADC == 0)  // Schedule ADC task, and..
            --SRTX_Sched_Cnt_TaskADC;       // ..max it at 0xFF if it rolls over.
    }
    else
    {
//
// Test for Timer0 rollover.
//
    if (INTCONbits.TMR0IF)              // If Timer0 interrupt flag set..
    {
        INTCONbits.TMR0IF = 0;          // ..then clear Timer0 interrupt flag.
        SUSR_Timer0_Expired();          // Schedule user tasks when timebase interrupts.
    }
    else
    {
//
// Test for Timer1 rollover.
//
    if (PIR1bits.TMR1IF)                // If Timer1 interrupt flag set..
    {
        PIR1bits.TMR1IF = 0;            // ..then clear Timer1 interrupt flag.
        SUSR_Timer1_Expired();          // User re-init of timebase interrupt.
        SRTX_Scheduler();               // Schedule user tasks when timebase interrupts.
    }
    else
    {
//
// Unknown interupt, can set breakpoint here.
//
    #asm
    nop
    nop
    nop
    #endasm
    } // if (PIR1bits.RCIF && PIE1bits.RCIE).
    } // if (PIR1bits.TXIF && PIE1bits.TXIE).
    } // if (PIR1bits.SSPIF).
    } // if (PIR1bits.ADIF).
    } // if (INTCONbits.TMR0IF).
    } // if (PIR1bits.TMR1IF).
}
