//*******************************************************************************
// tinyRTX Filename: uadc.c (User Analog to Digital Conversion routines)
//
// Copyright 2015 Sycamore Software, Inc.  ** www.tinyRTX.com **
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
//  23Oct15 Stephen_Higgins@KairosAutonomi.com
//              Convert from pic18 assembler to pic16 XC8.
//  05Nov15 Stephen_Higgins@KairosAutonomi.com
//              Add UCFG_KA027C defines.
//
//*******************************************************************************

#include    "ucfg.h"    // Configure board and proc, #include <proc.h>

//  Internal prototypes.

//  Variable Definitions

//*******************************************************************************

#if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010

    //   UCFG_PD2P_2010 OR UCFG_PD2P_2002
    //   ********************************

    #define UADC_ADCON0_VAL     0x01

    //  A/D channel = 0; no conversion active; A/D on.

    //  bit 7 : n/a   : 0 : (unused, don't care)
    //  bit 6 : CHS4  : 0 : Channel Select, 0b00000 -> AN0
    //  bit 5 : CHS3  : 0 : Channel Select, 0b00000 -> AN0
    //  bit 4 : CHS2  : 0 : Channel Select, 0b00000 -> AN0
    //  bit 3 : CHS1  : 0 : Channel Select, 0b00000 -> AN0
    //  bit 2 : CHS0  : 0 : Channel Select, 0b00000 -> AN0
    //  bit 1 : GO    : 0 : A/D Conversion Status (0 = not in progress)
    //  bit 0 : ADON  : 1 : A/D converter module is powered up
    
#endif

#if UCFG_BOARD==UCFG_KA027C

    //   UCFG_KA027C
    //   ***********

    #define UADC_ADCON0_VAL     0x19

    //  A/D channel = 0; no conversion active; A/D on.

    //  bit 7 : n/a   : 0 : (unused, don't care)
    //  bit 6 : CHS4  : 0 : Channel Select, 0b00110 -> AN6
    //  bit 5 : CHS3  : 0 : Channel Select, 0b00110 -> AN6
    //  bit 4 : CHS2  : 1 : Channel Select, 0b00110 -> AN6
    //  bit 3 : CHS1  : 1 : Channel Select, 0b00110 -> AN6
    //  bit 2 : CHS0  : 0 : Channel Select, 0b00110 -> AN6
    //  bit 1 : GO    : 0 : A/D Conversion Status (0 = not in progress)
    //  bit 0 : ADON  : 1 : A/D converter module is powered up
    
#endif

#define UADC_ADCON1_VAL  0xA0

//  Right-justified 10-bit A/D result; Vss on Vref-; Vdd on Vref+.
//  A/D conversion clock is 32MHz/32 = 1.0 us.
//    Tad must be >= 1.0 us (Tacq must be >= 5.0 us)

//  bit 7 : ADFM    : 1 : Right-justified 10-bit A/D result
//  bit 6 : ADCS2   : 0 : Clock Select, 0b010 -> Fosc/8)
//  bit 5 : ADCS2   : 1 : Clock Select, 0b010 -> Fosc/8)
//  bit 4 : ADCS2   : 0 : Clock Select, 0b010 -> Fosc/8)
//  bit 3 : n/a     : 0 : (unused, don't care)
//  bit 2 : ADNREF  : 0 : A/D Neg Ref, Vref- is connected to Vss
//  bit 1 : ADPREF1 : 0 : A/D Pos Ref, 0b00 -> Vref+ is connected to Vdd
//  bit 0 : ADPREF0 : 0 : A/D Pos Ref, 0b00 -> Vref+ is connected to Vdd

//*******************************************************************************
//
// UADC_Init: Initialize ADC registers.
//
//*******************************************************************************

void UADC_Init( void )
{
    ADCON0 = UADC_ADCON0_VAL;   //  A/D channel = 0; no conversion active; A/D on.
    ADCON1 = UADC_ADCON1_VAL;   //  Right-justified result; Vss on Vref-; Vdd on Vref+.
}

//*******************************************************************************
//
// UADC_Trigger: Trigger an AD conversion.
//
//*******************************************************************************

void UADC_Trigger( void )
{
    //  Delay for A/D acquisition time if processor only supports manual acquisition.
    //  (STRICTLY SPEAKING, THIS IS ONLY NEEDED IF CHANGING ACTIVE A/D PORT SELECTION.)
    //
    //  If changing ADC channel, delay 5us for A/D sample setup.
    //      (Only needed if processor only supports manual acquisition.)
    //  __delay_ms(5);

    PIE1bits.ADIE = 1;      //  Enable A/D interrupts. (BEFORE setting GO flag!)
    ADCON0bits.ADGO = 1;    //  Trigger A/D conversion.
}

//*******************************************************************************
//
// UADC_Raw10Bit:  Return 10-bit right-justified A/D result.
//
//*******************************************************************************

unsigned short UADC_Raw10Bit( void )
{
    return ADRES;
}
