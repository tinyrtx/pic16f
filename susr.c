//******************************************************************************
//  tinyRTX Filename: susr.c (System USeR interface)
//  
//  Copyright 2015 Sycamore Software, Inc.  ** www.tinyRTX.com **
//  Distributed under the terms of the GNU Lesser General Purpose License v3
//  
//  This file is part of tinyRTX. tinyRTX is free software: you can redistribute
//  it and/or modify it under the terms of the GNU Lesser General Public License
//  version 3 as published by the Free Software Foundation.
//  
//  tinyRTX is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY//without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//  A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
//  details.
//  
//  You should have received a copy of the GNU Lesser General Public License
//  (filename copying.lesser.txt) and the GNU General Public License (filename
//  copying.txt) along with tinyRTX.  If not, see <http://www.gnu.org/licenses/>.
//  
//  Revision History:
//    02Oct15 Stephen_Higgins@KairosAutonomi.com  
//                Created from pic18 susr.asm as converting to pic16 XC8.
//  
//******************************************************************************

        #include "ucfg.h"          //includes processor definitions.
        #include "uapp.h"
        #include "usio.h"
//        #include "strc.h"

//    IF UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
//        #include <ui2c.h>
//    ENDIF

//******************************************************************************
//
//  SUSR: System to User Redirection.
//  
//  These routines provide the interface from the SRTX (System Real Time eXecutive)
//    and SISD (Interrupt Service Routine Director) to user application code.

//******************************************************************************
//
//  SUSR_POR_PhaseA: User initialization at Power-On Reset Phase A.
//      Application time-critical initialization, no interrupts.
//
void SUSR_POR_PhaseA(void)
{
    UAPP_POR_Init_PhaseA();     //  User app POR time-critical init.
}

//******************************************************************************
//
//  SUSR_POR_PhaseB: User initialization at Power-On Reset Phase B.
//      Application non-time critical initialization.
//
void SUSR_POR_PhaseB(void)
{
    UAPP_POR_Init_PhaseB();     //  User app POR Init. (Enables global and peripheral ints.)
}

//******************************************************************************
//
//  SUSR_Timer0_Expired: User completion of PWM.
//
#pragma interrupt_level 1
void SUSR_Timer0_Expired(void)
{
    UAPP_Timer0_Expired();      //  Finish PWM.
}

//******************************************************************************
//
//  SUSR_Timer1_Expired: User init of timebase timer and its interrupt.
//
#pragma interrupt_level 1
void SUSR_Timer1_Expired(void)
{
    UAPP_Timer1_Init();         //  Re-init Timer1 so new Timer1 int in 100ms.
                                //  UAPP_Timer1_Init enabled Timer1 interrupts.
}

//******************************************************************************
//
//  SUSR_BkgdTask: Background task measures PWM.
//      User initialization of Timer0 (no interrupts.)
//
void SUSR_BkgdTask(void)
{
    UAPP_BkgdTask();            //  User background task.
}

//******************************************************************************
//
//  SUSR_Task1: User interface to Task1.
//
void SUSR_Task1(void)
{
    UAPP_Task1();
}

//******************************************************************************
//
//  SUSR_Task2: User interface to Task2.
//
void SUSR_Task2(void)
{
    UAPP_Task2();
}

//******************************************************************************
//
//  SUSR_Task3: User interface to Task3.
//
void SUSR_Task3(void)
{
    UAPP_Task3();
}

//******************************************************************************
//
//  SUSR_TaskADC: User interface to TaskADC.
//
void SUSR_TaskADC(void)
{
    UAPP_TaskADC();     //  Convert A/D result and do something with it.
}
//
//******************************************************************************
//
//  SUSR_TaskI2C: User interface to TaskI2C.
//
//    IF UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
//        call    UI2C_MsgTC74ProcessData //Process data from TC74 message.
//    ENDIF
//
void SUSR_TaskI2C(void)
{
//  UI2C_MsgTC74ProcessData();  //  Process data from TC74 message.
}

//******************************************************************************
//
//  SUSR_TaskSIO_MsgRcvd: User interface to TaskSIO.
//
void SUSR_TaskSIO_MsgRcvd(void)
{
    USIO_MsgReceived();         //  Process SIO received msg.
}

//******************************************************************************
//
//  SUSR_TaskI2C_MsgDone: User handling when I2C message completed.
//
//    IF UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
//        goto    UI2C_MsgDone
//    ENDIF
//
void SUSR_TaskI2C_MsgDone(void)
{
//    UI2C_MsgDone();
}
