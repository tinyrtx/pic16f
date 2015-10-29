//*******************************************************************************
// tinyRTX Filename: ucfg.h (User ConFiGuration)
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
//*******************************************************************************
//
// Revision history:
//  16Sep15 Stephen_Higgins@KairosAutonomi.com
//              Created from pic18 version.
//
//*******************************************************************************
// NOTE: This file MUST be (manually) kept in synch with ucfg.inc !!!
//*******************************************************************************
//
//   Define all supported board configurations.
//
#define UCFG_PD2P_2002      0x01    // Microchip PICDEM2PLUS 2002 (has ext 4MHz)
#define UCFG_PD2P_2010      0x02    // Microchip PICDEM2PLUS 2010 (no ext 4MHz)
#define UCFG_KA027C         0x10    // Kairos Autonomi 107I: Quad and Video Mux
//
//   Define all supported processors.
//
#define UCFG_16F877     0x01        // 40 pins. (unsupported)
#define UCFG_16F1847    0x02        // 18 pins.
//
//*******************************************************************************
//
//  Include appropriate processor definition file, define UCFG_PROC.
//  Define UCFG_BOARD, set UCFG_SSIO_EOMC.
//  Output message of configuration of board and processor.
//
    #include    <xc.h>
    #ifdef __16F1847
        #define UCFG_PROC   UCFG_16F1847    // Allows logical expressions.
        #warning Processor defined: UCFG_16F1847
    #endif       

    #ifdef _UCFG_PD2P02
        #define UCFG_BOARD  UCFG_PD2P_2002  // Allows logical expressions.
//        #define UCFG_SSIO_EOMC  0x0d        // End Of Msg Char = <CR>
        #define UCFG_SSIO_EOMC  0x5d        // End Of Msg Char = "]"
        #warning Board defined: UCFG_PD2P_2002
    #endif       
    #ifdef _UCFG_PD2P10
        #define UCFG_BOARD  UCFG_PD2P_2010  // Allows logical expressions.
        #define UCFG_SSIO_EOMC  0x5d        // End Of Msg Char = "]"
        #warning Board defined: UCFG_PD2P_2010
    #endif       
    #ifdef _UCFG_KA027C
        #define UCFG_BOARD  UCFG_KA027C     // Allows logical expressions.
        #define UCFG_SSIO_EOMC  0x5d        // End Of Msg Char = "]"
        #warning Board defined: UCFG_KA027C
     #endif
