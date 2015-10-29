//******************************************************************************
// tinyRTX Filename: usio.c (User Serial I/O communication routines)
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
//  16Oct15 Stephen_Higgins@KairosAutonomi.com
//              Convert from pic18 assembler to pic16 XC8.
//
//******************************************************************************

#include    "ucfg.h"    // Configure board and proc, #include <proc.h>
#include    "srtx.h"
#include    "ssio.h"
#include    "uapp.h"

//#include    "susr.h"
//        #include    <slcd.inc>
//        #include    <slcduser.inc>

//  Internal prototypes.

//  Variable Definitions

//******************************************************************************

//  User SIO defines.

#if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010 || UCFG_BOARD==UCFG_KA027C

//  UCFG_PD2P_2002 or UCFG_PD2P_2010 or UCFG_KA027C specified.
//  **********************************************************

    #define USIO_SPBRGH_VAL 0
    #define USIO_SPBRGL_VAL 68

//  Baud rate for 32 Mhz clock, BRGH = 1, BRG16 = 1
//    SPBRG  832 =   9.6K    
//    SPBRG  416 =  19.2K
//    SPBRG  138 =  57.6K
//    SPBRG  68  = 115.2K (USIO_SPBRGH_VAL = 0, USIO_SPBRG_VAL = 68)

#else
    #warning *** Unrecognized UCFG_BOARD ***
#endif

#define USIO_TXSTA_VAL  0x24

//  bit 7 : CSRC  : 0 : Don't care (Asynch mode)
//  bit 6 : TX9   : 0 : 8-bit transmission
//  bit 5 : TXEN  : 1 : Transmit enabled
//  bit 4 : SYNC  : 0 : Asynchronous mode
//  bit 3 : SENDB : 0 : Sync Break transmission completed
//  bit 2 : BRGH  : 1 : High speed Baud rate
//  bit 1 : TRMT  : 0 : Transmit Shift Register full
//  bit 0 : TX9D  : 0 : Don't care (9th bit data)

#define USIO_RCSTA_VAL  0x90

//  bit 7 : SPEN  : 1 : Serial port enabled (RX/DT and TX/CK used)
//  bit 6 : RX9   : 0 : 8-bit reception
//  bit 5 : SREN  : 0 : Don't care (Asynch mode)
//  bit 4 : CREN  : 1 : Continuous Receive enabled
//  bit 3 : ADDEN : 0 : Don't care (Asynch mode, RX9=0)
//  bit 2 : FERR  : 0 : No framing error
//  bit 1 : OERR  : 0 : No overrun error
//  bit 0 : RX9D  : 0 : Don't care (9th bit data)

#define USIO_BAUDCON_VAL    0x48

//  bit 7 : ABDOVF: 0 : No BRG rollover
//  bit 6 : RCIDL : 1 : Receive operation idle
//  bit 5 : -     : 0 : Not implemented
//  bit 4 : SCKP  : 0 : Non-inverted data to the TX/CK pin
//  bit 3 : BRG16 : 1 : 16-bit, uses SPBRG and SPBRGH
//  bit 2 : -     : 0 : Not implemented
//  bit 1 : WUE   : 0 : Receiver operating normally
//  bit 0 : ABDEN : 0 : Autobaud disabled or completed

//******************************************************************************
//
//  USIO_Init: Init SIO hardware.  Assumes GIE and PEIE already enabled.
//
//******************************************************************************

void USIO_Init( void )
{
    SPBRGL = USIO_SPBRGL_VAL;           //  Set baud rate.
    SPBRGH = USIO_SPBRGH_VAL;
    BAUDCON = USIO_BAUDCON_VAL;

    TXSTA = USIO_TXSTA_VAL;             //  Enable transmission and high baud rate.
    RCSTA = USIO_RCSTA_VAL;             //  Enable serial port and reception and 16-bit BRG.

    SSIO_InitFlags();                   //  Initialize SSIO internal flags.
    SSIO_InitTxBuffer();                //  Initialize transmit buffer.
    SSIO_InitRcBuffer();                //  Initialize receive buffer.

    PIE1bits.RCIE = 1;                  //  Enable RC interrupts forever.

    //  Don't set TXIE until something is actually written to the TX buffer.
    //  Otherwise we're going to get a TX int right now because at init the TX hardware
    //  is empty so therefore TXIF is set.
}

//******************************************************************************
//  
//   USIO_TxLCDMsgToSIO is called from SUSR_TaskADC when an A/D conversion completes. 
//   It moves the message from the (unused) SLCD buffer to the SIO transmit buffer,
//   effectively replacing the LCD function with a transmit over the SIO (RS-232).
//  
//   Note that we use FSR0 and we depend on any routine which might be called
//     from an interrupt to protect FSR0. 
//  
//         GLOBAL  USIO_TxLCDMsgToSIO
//  SIO_TxLCDMsgToSIO
//  
//         movlw   SLCD_BUFFER_LINE_SIZE       ; Size of source buffer.
//         banksel USIO_DataXferCnt
//         movwf   USIO_DataXferCnt            ; Size saved in data transfer counter.
//  
//         lfsr    0, SLCD_BufferLine2         ; Data pointer gets source start address.
//  
//  SIO_TxLCDMsgToSIO_NextByte
//  
//         movf    POSTINC0, W                 ; Get char from source LCD buffer.
//         call    SSIO_PutByteTxBuffer        ; Move char to dest SIO Tx buffer.
//  
//         banksel USIO_DataXferCnt
//         decfsz  USIO_DataXferCnt, F         ; Dec count of data to copy, skip if all done.
//         bra     USIO_TxLCDMsgToSIO_NextByte ; More data to copy.
//  
//         movlw   0x0d
//         call    SSIO_PutByteTxBuffer        ; Move <CR> to dest SIO Tx buffer.
//         movlw   0x0a
//         call    SSIO_PutByteTxBuffer        ; Move <LF> to dest SIO Tx buffer.
//         return
//  
//******************************************************************************

//******************************************************************************
//  
//  USIO_MsgReceived is called from SSIO/SUSR when an SIO message completes.
//  
//  Move the message from the system receive buffer to the USER APPLICATION
//    RECEIVE buffer, allowing it to be parsed.
//  
//******************************************************************************

void USIO_MsgReceived( void )
{
char c;

    do  {
        c = SSIO_GetByteRcBuffer();         //  Get data from receive buffer.
        UAPP_PutByteRcBuffer( c );          //  Put data in UAPP Rc buffer.

        } while( c != UCFG_SSIO_EOMC );     //  For all chars until we copy an <EOMC>.

    UAPP_ParseRcMsg();                      //  Parse command string.
}