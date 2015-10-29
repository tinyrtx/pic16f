//*******************************************************************************
// tinyRTX Filename: sutl.c (System UTiLity services)
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
//  22Oct15 Stephen_Higgins@KairosAutonomi.com
//              Convert from pic18 sbcd.asm to pic16 XC8 sutl.c.
//
//*******************************************************************************

#include    "ucfg.h"    // Configure board and proc, #include <proc.h>
#include    "sutl.h"

//  Internal prototypes.

//  Variable Definitions

//*******************************************************************************
//
// SUTL_EngToBCD_16U: 16-bit Unsigned Fixed Point Conversion to BCD
//      Input:  16 bit unsigned integer
//      Output: 5 BCD digit result (high nibble 0)
//
//  NOTE: This is done with repeated subtractions, as that should be quicker
//      than divisions.
//
//*******************************************************************************

unsigned short long SUTL_EngToBCD_16u( unsigned int SUTL_InputWord )
{
unsigned short long SUTL_Result;
SUTL_Result = 0;

    while( SUTL_InputWord  >= 10000 )
        {
        SUTL_InputWord -= 10000;
        SUTL_Result += 0x10000;
        }

    while( SUTL_InputWord  >= 1000 )
        {
        SUTL_InputWord -= 1000;
        SUTL_Result += 0x1000;
        }

    while( SUTL_InputWord  >= 100 )
        {
        SUTL_InputWord -= 100;
        SUTL_Result += 0x100;
        }

    while( SUTL_InputWord  >= 10 )
        {
        SUTL_InputWord -= 10;
        SUTL_Result += 0x10;
        }

    SUTL_Result += (unsigned int) SUTL_InputWord;
    return SUTL_Result;
}
