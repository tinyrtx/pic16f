//*******************************************************************************
// tinyRTX Filename: ssio.c (System Serial I/O communication services)
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
//  06Oct15 Stephen_Higgins@KairosAutonomi.com
//              Convert from pic18 assembler to pic16 XC8.
//
//*******************************************************************************

#include    "ucfg.h"    // Configure board and proc, #include <proc.h>
#include    "srtx.h"
#include    "susr.h"

//  Internal prototypes.

void SSIO_PutByteRcBuffer( char );
char SSIO_GetByteTxBuffer( void );
void SSIO_PutByteTxBuffer( char );

//  Variable Definitions

struct
{
    unsigned char SSIO_TxBufFull    : 1;    //  Tx buffer is full.
    unsigned char SSIO_TxBufEmpty   : 1;    //  Tx buffer is empty.
    unsigned char SSIO_RcBufFull    : 1;    //  Rc buffer is full.
    unsigned char SSIO_RcBufEmpty   : 1;    //  Rc buffer is empty.
    unsigned char SSIO_Verifying    : 1;    //  For verification.
    unsigned char unused            : 3;
} SSIO_Flags;

unsigned char SSIO_RcCntOERR;       //  Count of Overrun errors.
unsigned char SSIO_RcCntFERR;       //  Count of Framing errors.
unsigned char SSIO_RcCntBufOver;    //  Count of RX Buffer Overrun errors.
unsigned char SSIO_TxCntBufOver;    //  Count of TX Buffer Overrun errors.

char *SSIO_TxHeadPtr;               //  Transmit buffer data head pointer.
char *SSIO_TxTailPtr;               //  Transmit buffer data tail pointer.
char *SSIO_TxPrevTailPtr;           //  Transmit buffer data PREV tail pointer.
char *SSIO_RcHeadPtr;               //  Receive buffer data head pointer.
char *SSIO_RcTailPtr;               //  Receive buffer data tail pointer.
char *SSIO_RcPrevTailPtr;           //  Receive buffer data PREV tail pointer.

#define SSIO_TX_BUF_LEN 0x70        //  Define transmit buffer size.
#define SSIO_RC_BUF_LEN 0x70        //  Define receive buffer size.

char SSIO_TxBuffer[ SSIO_TX_BUF_LEN ];  //  Transmit data buffer.
char SSIO_RcBuffer[ SSIO_RC_BUF_LEN ];  //  Receive data buffer.

//*******************************************************************************
//
// SSIO_InitFlags: Initialize flags and error counters.
//
//*******************************************************************************

void SSIO_InitFlags( void )
{
    SSIO_Flags.SSIO_Verifying = 0;      //  Clear all flags.

    SSIO_RcCntOERR = 0;                 //  Clear all error counters.
    SSIO_RcCntFERR = 0;
    SSIO_RcCntBufOver = 0;
    SSIO_TxCntBufOver = 0;
}

//*******************************************************************************
//
// SSIO_InitTxBuffer: Initialize Tx buffer.
//
//*******************************************************************************

void SSIO_InitTxBuffer( void )
{
    SSIO_TxHeadPtr = SSIO_TxBuffer;     // Init Tx head ptr to Tx buffer start.
    SSIO_TxTailPtr = SSIO_TxBuffer;     // Init Tx tail ptr to Tx buffer start.

    // Init Tx prev tail ptr to last element in Tx buffer.
    SSIO_TxPrevTailPtr = &SSIO_TxBuffer[ SSIO_TX_BUF_LEN-1 ];

    SSIO_Flags.SSIO_TxBufFull = 0;      //  Tx buffer is not full.
    SSIO_Flags.SSIO_TxBufEmpty = 1;     //  Tx buffer is empty.
}

//*******************************************************************************
//
// SSIO_InitRcBuffer: Initialize Rc buffer.
//
//*******************************************************************************

void SSIO_InitRcBuffer( void )
{
    SSIO_RcHeadPtr = SSIO_RcBuffer;     // Init Rc head ptr to Rc buffer start.
    SSIO_RcTailPtr = SSIO_RcBuffer;     // Init Rc tail ptr to Rc buffer start.

    // Init Rc prev tail ptr to last element in Rc buffer.
    SSIO_RcPrevTailPtr = &SSIO_RcBuffer[ SSIO_RC_BUF_LEN-1 ];

    SSIO_Flags.SSIO_RcBufFull = 0;      //  Rc buffer is not full.
    SSIO_Flags.SSIO_RcBufEmpty = 1;     //  Rc buffer is empty.
}

//*******************************************************************************
//
// SSIO_PutByteIntoTxHW: Read data from Tx buffer and put into USART transmit register.
//
//*******************************************************************************

void SSIO_PutByteIntoTxHW( void )
{
    if( SSIO_Flags.SSIO_TxBufEmpty )
        PIE1bits.TXIE = 0;              //  Tx buffer empty so disable Tx interrupt.
    else
        TXREG = SSIO_GetByteTxBuffer(); //  Put data in HW transmit register.
}

//*******************************************************************************
//
// SSIO_GetByteFromRcHW: Get data from USART Rc register and write it into Rc buffer.
//
//*******************************************************************************

void SSIO_GetByteFromRcHW( void )
{
char c;

    //  Check for serial errors and handle them if found.

    if( RCSTAbits.OERR )                //  Overrun error handling.
        {
        RCSTAbits.CREN = 0;             //  Reset the receiver logic.
        RCSTAbits.CREN = 1;             //  Enable reception again. ENSURE NOT OPTIMIZED!!
        if( ++SSIO_RcCntOERR == 0 )     //  Increment overrun error count.
            --SSIO_RcCntOERR;           //  Ensure error count doesn't wrap.
        return;                         //  No further handling, just return.
        }

    if( RCSTAbits.FERR )                //  Framing error handling.
        {
        c = RCREG;                      //  Discard received data that has error.
        if( ++SSIO_RcCntFERR == 0 )     //  Increment framing error count.
            --SSIO_RcCntFERR;           //  Ensure error count doesn't wrap.
        return;                         //  No further handling, just return.
        }

    if( SSIO_Flags.SSIO_RcBufFull )     //  Receive buffer overrun error.
        if( ++SSIO_RcCntBufOver == 0 )  //  Increment buffer overrun error count.
            --SSIO_RcCntBufOver;        //  Ensure error count doesn't wrap.
   
    //  Put good or overrun data into receive buffer.

    c = RCREG;                          //  Get received data from Rc hardware.
    SSIO_PutByteRcBuffer( c );          //  Put data in Rc buffer.

    //  If we found <EOMC> then schedule user task to process data.
    //  Then interrupt can exit.
    //  SRTX Dispatcher will find task scheduled and invoke SUSR_TaskSIO.

    if( c == UCFG_SSIO_EOMC )               //  If data == <EOMC>..
        if( ++SRTX_Sched_Cnt_TaskSIO == 0 ) //  ..then increment task schedule count.
            --SRTX_Sched_Cnt_TaskSIO;       //  Ensure task schedule count doesn't wrap.
}

//*******************************************************************************
//
// SSIO_PutByteTxBuffer: Add a byte to tail of transmit buffer.
//
//  NOTE: If we are storing a byte and the buffer is already full, don't store
//  at current tail pointer nor update tail.  Instead store at previous
//  tail pointer.  Continue to overwrite the last location in the buffer.
//
//*******************************************************************************

void SSIO_PutByteTxBuffer( char c )
{
    PIE1bits.TXIE = 0;                      //  Disable Tx interrupt.

    if( SSIO_Flags.SSIO_TxBufFull )         //  If Tx buffer is full..
        {
        if( ++SSIO_TxCntBufOver == 0 )      //  ..then increment overrun error count.
            --SSIO_TxCntBufOver;            //  Ensure overrun error count doesn't wrap.

        *SSIO_TxPrevTailPtr = c;            //  Overwrite data at last tail pointer.
        return;                             //  No further handling, just return.
        }

    SSIO_TxPrevTailPtr = SSIO_TxTailPtr;    //  Save PREV tail pointer.
    *SSIO_TxTailPtr++ = c;                  //  Save data at current tail pointer.

    if( SSIO_TxTailPtr > &SSIO_TxBuffer[ SSIO_TX_BUF_LEN-1 ] )  //  If Tx tail ptr past buffer..
        SSIO_TxTailPtr = SSIO_TxBuffer;     //  ..then re-set Tx tail ptr to Tx buffer start.

    if( SSIO_TxTailPtr == SSIO_TxHeadPtr )  //  If Tx tail ptr now == Tx head ptr..
        SSIO_Flags.SSIO_TxBufFull = 1;      //  ..flag Tx buffer full.

    SSIO_Flags.SSIO_TxBufEmpty = 0;         //  Flag Tx buffer not empty.
//    if( !SSIO_Flags.Verifying )             //  If not verifying..
    PIE1bits.TXIE = 1;                      //  Enable Tx interrupt.
}

//*******************************************************************************
//
//  SSIO_PutByteRcBuffer: Add a byte to tail of receive buffer.
//  
//  NOTE: This routine is time critical.  It barely runs in the time allocated
//  for a 4 MHz clock at 115.2K baud.  It runs fine at 57.6K baud.  Consideration
//  should be given to making this a high priority interrupt, so fast receives
//  can be accommodated.
//
//  NOTE: If we are storing a byte and the buffer is already full, don't store
//  at current tail pointer nor update tail.  Instead store at previous
//  tail pointer.  Continue to overwrite the last location in the buffer.
//  So when <EOMC> is received and stored, other routines will find it.
//
//  NOTE: no disabling/re-enabling of RX interrupt because this routine called from Rc ISR.

void SSIO_PutByteRcBuffer( char c )
{
    if( SSIO_Flags.SSIO_RcBufFull )         //  If Rc buffer is full..
        {
        if( ++SSIO_RcCntBufOver == 0 )      //  ..then increment overrun error count.
            --SSIO_RcCntBufOver;            //  Ensure overrun error count doesn't wrap.

        *SSIO_RcPrevTailPtr = c;            //  Overwrite data at last tail pointer.
        return;                             //  No further handling, just return.
        }

    SSIO_RcPrevTailPtr = SSIO_RcTailPtr;    //  Save PREV tail pointer.
    *SSIO_RcTailPtr++ = c;                  //  Save data at current tail pointer.

    if( SSIO_RcTailPtr > &SSIO_RcBuffer[ SSIO_RC_BUF_LEN-1 ] )  //  If Rc tail ptr past buffer..
        SSIO_RcTailPtr = SSIO_RcBuffer;     //  ..then re-set Rc tail ptr to Rc buffer start.

    if( SSIO_RcTailPtr == SSIO_RcHeadPtr )  //  If Rc tail ptr now == Rc head ptr..
        SSIO_Flags.SSIO_RcBufFull = 1;      //  ..flag Rc buffer full.

    SSIO_Flags.SSIO_RcBufEmpty = 0;         //  Flag Rc buffer not empty.
}

//*******************************************************************************
//
//  SSIO_GetByteTxBuffer: Remove and return the byte at head of transmit buffer.
//
//*******************************************************************************

char SSIO_GetByteTxBuffer( void )
{
char c;

    if( SSIO_Flags.SSIO_TxBufEmpty )        //  If buffer empty..
        return '\0';                        //  .. just leave, returning null.

    c = *SSIO_TxHeadPtr++;                  //  Copy data, increment head pointer.

    if( SSIO_TxHeadPtr > &SSIO_TxBuffer[ SSIO_TX_BUF_LEN-1 ] )  //  If Tx head ptr past buffer..
        SSIO_TxHeadPtr = SSIO_TxBuffer;     //  ..then re-set Tx head ptr to Tx buffer start.

    if( SSIO_TxHeadPtr == SSIO_TxTailPtr )  //  If Tx head ptr now == Tx tail ptr..
        SSIO_Flags.SSIO_TxBufEmpty = 1;     //  ..flag Tx buffer empty.

    SSIO_Flags.SSIO_TxBufFull = 0;          //  Buffer cannot be full.
    return c;                               //  Return data.
}

//*******************************************************************************
//
//  SSIO_GetByteRcBuffer: Remove and return the byte at head of receive buffer.
//
//*******************************************************************************

char SSIO_GetByteRcBuffer( void )
{
char c;

    if( SSIO_Flags.SSIO_RcBufEmpty )        //  If buffer empty..
        return '\0';                        //  .. just leave, returning null.

    c = *SSIO_RcHeadPtr++;                  //  Copy data, increment head pointer.

    if( SSIO_RcHeadPtr > &SSIO_RcBuffer[ SSIO_RC_BUF_LEN-1 ] )  //  If Rc head ptr past buffer..
        SSIO_RcHeadPtr = SSIO_RcBuffer;     //  ..then re-set Rc head ptr to Rc buffer start.

    if( SSIO_RcHeadPtr == SSIO_RcTailPtr )  //  If Rc head ptr now == Rc tail ptr..
        SSIO_Flags.SSIO_RcBufEmpty = 1;     //  ..flag Rc buffer empty.

    SSIO_Flags.SSIO_RcBufFull = 0;          //  Buffer cannot be full.
    return c;                               //  Return data.
}

//*******************************************************************************
//
//  SSIO_PutStringTxBuffer: Add a data string to tail of transmit buffer.
//
//  NOTES: No error checking if buffer is full, just try to write the whole string.
//      String must be terminated C-style by "\0" (null) character.
//
//*******************************************************************************

void SSIO_PutStringTxBuffer( char *SSIO_InputString )
{
char c;

    while( ( c = *SSIO_InputString++ ) != '\0' )    //  For all chars until "\0"..
        SSIO_PutByteTxBuffer( c );                  //  ..copy char to Tx buffer.    
}
