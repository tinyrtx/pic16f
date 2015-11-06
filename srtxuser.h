//*******************************************************************************
// tinyRTX Filename: srtxuser.h (System Real Time eXecutive to USER interface)
//
// Copyright 2014 Sycamore Software, Inc.  ** www.tinyRTX.com **
// Distributed under the terms of the GNU Lesser General Purpose License v3
//
// This file is part of tinyRTX. tinyRTX is free software: you can redistribute
// it and/or modify it under the terms of the GNU Lesser General Public License
// version 3 as published by the Free Software Foundation.
//
// tinyRTX is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY// without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General Public License
// (filename copying.lesser.txt) and the GNU General Public License (filename
// copying.txt) along with tinyRTX.  If not, see <http://www.gnu.org/licenses/>.
//
// Revision history:
//  04Sep15 Stephen_Higgins@KairosAutonomi.com
//              Change from pic18f to pic16f1847.
//  04Sep15 Stephen_Higgins@KairosAutonomi.com
//              Change from pic18f to pic16f1847.
//
//*******************************************************************************
#include    "ucfg.h"

// Counts to load task timer at initialization (must be non-zero.)
// Each task will be first scheduled at the SRTX-timer event which
//  occurs as equated below.  However, please note that immediately
//  after initialization is complete, a single "faux" SRTX-timer
//  event occurs, which allows all tasks equated to "1" below to run.
//  Allowed range is (1 - 255).

#define SRTX_CNT_INIT_TASK1 1
#define SRTX_CNT_INIT_TASK2 1
#define SRTX_CNT_INIT_TASK3 1

// Counts to reload task timers each expiration.  This is the number of
//  SRTX timer events which must occur before the task is again scheduled.
//  SRTX timer events occur every 10 or 20 or 100 ms, depending on board.
//  Search on definition of UAPP_TMR1L_VAL and UAPP_TMR1H_VAL (usually in 
//  uapp_XXX.c) for the timer base interval.
//  Allowed range is (0-255).

#if (UCFG_BOARD == UCFG_PD2P_2002) || (UCFG_BOARD == UCFG_PD2P_2010) || (UCFG_BOARD == UCFG_KA027C)
    #define	SRTX_CNT_RELOAD_TASK1	0x01    //   1 =    10 ms  (for example)
    #warning SRTX CNT RELOAD TASK1: 1
    #define	SRTX_CNT_RELOAD_TASK2	0x0A    //  10 =   100 ms  (for example)
    #define	SRTX_CNT_RELOAD_TASK3	0x64    // 100 = 1.000 sec (for example)
#else
    #warning *** Unrecognized UCFG_BOARD ***
#endif

