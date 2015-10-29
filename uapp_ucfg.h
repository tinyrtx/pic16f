//*******************************************************************************
// tinyRTX Filename: uapp_ucfg.h (User APPlication User ConFiGuration)
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
// ***********************************************************************
//
// Revision history:
//  02Oct15 Stephen_Higgins@KairosAutonomi.com
//              Created from pic18 version.
//
//*******************************************************************************
// NOTE: This file must be (manually) kept in synch with uapp_ucfg.inc !!!
//*******************************************************************************

#include "ucfg.h"          // includes processor definitions.

//*******************************************************************************
//
//   Set CONFIG bits. Valid values found in <"processor".h>, e.g. <p16f877.h>.
//
//*******************************************************************************

#if UCFG_PROC==UCFG_16F1847
    #warning Processor defined: UCFG_16F1847

    //  UCFG_16F1847 specified.
    //  ***********************

    #if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010 || UCFG_BOARD==UCFG_KA027C

        //  UCFG_PD2P_2002 OR UCFG_PD2P_2010 OR UCFG_KA027C specified.
        //  **********************************************************

        #pragma config FOSC = INTOSC    // INTOSC oscillator: I/O on CLKIN/CLKOUT pins.
        #pragma config PLLEN = ON       // 4x PLL enabled
    #else
        #warning *** Unrecognized UCFG_BOARD ***
    #endif

    //      UCFG_16F1847 common to all boards.
    //      **********************************

    #pragma config CLKOUTEN = OFF      // CLKOUT function disabled. I/O or osc on CLKOUT pin
    #pragma config FCMEN = OFF         // Fail-Safe Clock Monitor disabled
    #pragma config IESO = OFF          // Oscillator Switchover mode disabled
    #pragma config BOREN = ON          // Brown-out Reset enabled
    #pragma config BORV = HI           // Brown-out Reset Voltage (Vbor), high trip point selected.
    #pragma config MCLRE = ON          // MCLR pin enabled; RE3 input pin disabled
    #pragma config STVREN = ON         // Stack full/underflow will cause Reset
#endif

//*******************************************************************************
//
//  CONFIG common to all processors and all boards.
//  ***********************************************

#pragma config PWRTE = OFF         // PWRT disabled
#pragma config WDTE = OFF          // WDT disabled
#pragma config LVP = OFF           // High-voltage on MCLR/VPP must be used for programming
#pragma config CP = OFF            // Program memory code protection is disabled
#pragma config CPD = OFF           // Data memory code protection is disabled
#pragma config WRT = OFF           // Write protection off

//*******************************************************************************
