//*******************************************************************************
// tinyRTX Filename: uapp_ka027c.c (User APPlication for KA 027C board)
//
// Copyright 2016 Sycamore Software, Inc.  ** www.tinyRTX.com **
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
//  05Nov15 Stephen_Higgins@KairosAutonomi.com
//              Created from uapp_pd2p.c, converted to Kairos 027C.
//  01Apr16 Stephen_Higgins@KairosAutonomi.com
//              Init c='X' in UAPP_Task3() to fix bug.
//  12Apr16 Stephen_Higgins@KairosAutonomi.com
//              Change Task1 from 20ms to 5ms, use Task2 at 20 ms.
//              If UAPP_Flags.UAPP_100PctActive NOT TRUE then function as before,
//              but calling UADC_Trigger() from Task2 (now 20ms) instead of Task1.
//              Also set Timer0 for 8us/count as before.
//              If it is TRUE, then call UADC_Trigger() from Task1 (now 5ms).
//              Also set Timer0 for 32us/count, so can get max PWM width
//              of 8ms.  This lets us create a 5ms PWM at 5ms period for 100% on.
//              Add new messages and use Task3 to call UAPP_CheckDiscrete_100PctActive().
//
//*******************************************************************************
//
// Complete PIC16F1847 (18-pin device) pin assignments for PICDEM 2 Plus (2002) Demo Board
//   OR PICDEM 2 Plus (2010) Demo Board:
//
//  1) RA2/AN2           = Discrete In: unused
//  2) RA3/AN3           = Discrete In: unused
//  3) RA4               = Discrete In:  Pushbutton S2 (active low, no debounce H/W)
//  4) RA5/MCLR*/Vpp     = Reset/Programming connector(1): (active low, with debounce H/W)
//  5) Vss               = Programming connector(3) (Ground)
//  6) RB0               = Discrete Out: LED RB0 (when J6 in place)
//                       = (Discrete In: RB0/INT also Pushbutton S3 (active low, with debounce H/W))
//  7) RB1/(RX)/SDA1     = Discrete Out: LED RB1 (when J6 in place)
//  8) RB2/(TX)/RX/SDA2  = Discrete In, USART RX (RS-232), USART control requires pin as Discrete In.
//  9) RB3               = Discrete Out: LED RB3 (when J6 in place)
// 10) RB4/SCL1          = No Connect: (configured as Discrete In)
// 11) RB5/TX/SCL2       = Discrete In, USART TX (RS-232), USART control requires pin as Discrete In.
// 12) RB6/PGC           = Programming connector(5) (PGC) ICD2 control requires pin as Discrete In.
// 13) RB7/PGD           = Programming connector(4) (PGD) ICD2 control requires pin as Discrete In.
// 14) Vdd               = Programming connector(2) (+5 VDC)
// 15) RA6/OSC2          = Discrete Out: unused
    //
    //   External 4 MHz crystal installed in Y2, PICDEM 2 Plus (2002) (IGNORED).
    //
    // 16) RA7/OSC1/CLKIN    = Discrete In: unused, ignore external crystal (or remove it)
    //
    //   External 4 MHz crystal NOT installed in Y2, PICDEM 2 Plus (2010) PN 02-01630-1.
    //   Note that Jumper J7 should not be connected.
    //
    // 16) RA7/OSC1/CLKIN    = Discrete In: unused
    //
// 17) RA0/AN0           = Analog In: Potentiometer Voltage
// 18) RA1/AN1           = Discrete Out: Servo PWM, 20 ms period, 900-2040 us pulse width.
//
//  NOTE: Both versions of the PICDEM 2 Plus Demo Board have RX/TX routed to 
//      RC7/RC6.  Therefore 18-pin parts do not have access to these signals.
//      With the usage of APFCON0/1 the chip RX/TX are routed to RB2/RB5 (same
//      pins as RX/TX on the Nemesis 16F88).  So for the chip to use the RS-232
//      port it is necessary to jumper chip RX RB2 to board RX RC7, and also
//      jumper chip TX RB5 to board TX RC6.
//
//*******************************************************************************
//
//  Complete PIC16F1847 (18-pin device) pin assignments for KA 027C
//      Pot and Encoder Board.  Note that PGC and PGD are not
//      supported by this board, so the chip must be programmed in a board that
//      supports the ICD3, then moved to the target 027C board.
//
//  NOTE: This software may also be compiled for PICDEM2+ 20012 and 2010 boards.
//      See accompanying ReadMeToConfigure.txt text file for instructions.
//      Also RX/TX lines will need to be jumpered for 18-pin parts on PICDEM2+.
//
//  1) RA2/AN2           = Discrete In: unused
//  2) RA3/AN3           = Discrete Out: State of Health LED
//  3) RA4               = Discrete In: unused
//  4) RA5/MCLR*/Vpp     = MCLR*: unused
//  5) Vss               = Ground
//  6) RB0               = Discrete Out: unused
//  7) RB1/(RX)/SDA1     = Discrete Out: unused
//  8) RB2/(TX)/RX/SDA2  = Discrete In: USART RX (RS-232), USART requires pin DI.
//  9) RB3               = Discrete Out: unused
// 10) RB4/SCL1          = Discrete In: Quad Encoder B input: unused
// 11) RB5/TX/SCL2       = Discrete In: USART TX (RS-232), USART requires pin DI.
// 12) RB6/PGC           = Discrete in: Quad Encoder A input: unused
// 13) RB7/PGD           = Analog In: Potentiometer Voltage, 0-5V
// 14) Vdd               = +5 VDC
// 15) RA6/OSC2          = Discrete Out: unused
// 16) RA7/OSC1/CLKIN    = Discrete In: unused
// 17) RA0/AN0           = Discrete In: 100% PWM is inactive if pulled high, active if GND.
// 18) RA1/AN1           = Discrete Out: Servo PWM, 20 ms period, 900-2040 us pulse width.
//
//*******************************************************************************

#include "ucfg.h"           // includes processor definitions.
#include "uapp_ucfg.h"      // includes #config definitions, only include in uapp*.c.
#include "usio.h"
#include "uadc.h"
#include "ssio.h"
#include "sutl.h"

//  Internal prototypes.

void UAPP_ClearRcBuffer( void );
void UAPP_Timer0_Init( unsigned char );
void UAPP_CheckDiscrete_100PctActive( void );

//  String literals.

const char UAPP_MsgInit[] = "[V: KA-027C 16F1847 v2.1.0 20160412]\n\r";
const char UAPP_MsgHelp[] = "[H: H-elp V-ersion E-cho R-eport P-ercent PWM]\n\r";
const char UAPP_MsgEchoActive[] = "[E: Message echo activated]\n\r";
const char UAPP_MsgEchoInactive[] = "[E: Message echo deactivated]\n\r";
const char UAPP_MsgReportActive[] = "[R: Reporting activated]\n\r";
const char UAPP_MsgReportInactive[] = "[R: Reporting deactivated]\n\r";
const char UAPP_Msg100PctActive[] = "[P: 100 percent PWM activated]\n\r";
const char UAPP_Msg100PctInactive[] = "[P: 100 percent PWM deactivated]\n\r";
const char UAPP_MsgNotImplemented[] = "[?: Not implemented]\n\r";
const char UAPP_MsgNotRecognized[] = "[?: Not recognized]\n\r";
const char UAPP_Nibble_ASCII[] = "0123456789ABCDEF";

// Internal variables.

#define UAPP_BUFFERLENGTH 40

unsigned char UAPP_BufferRc[UAPP_BUFFERLENGTH];
unsigned char UAPP_IndexRc;
unsigned char UAPP_ADC_PWM_Msg[UAPP_BUFFERLENGTH];

struct
{
    unsigned char UAPP_EchoActive:    1;    //  Echo incoming Rc msg to Tx.
    unsigned char UAPP_ReportActive:  1;    //  Report data in Task 3.
    unsigned char UAPP_100PctActive:  1;    //  0 -> 20ms PWM period, 900us-2040us width, Timer0  8us.
                                            //  1 ->  5ms PWM period,        0-5ms width, Timer0 32us.
    unsigned char unused            : 5;
} UAPP_Flags;

//*******************************************************************************
//
//  The Internal Oscillator Block can be used with the 4X PLL associated with the
//  External Oscillator Block to produce a 32 MHz internal system clock source.
//  The following settings are required to use the 32 MHz internal clock source:
//      The FOSC bits in Configuration Words must be set to use the INTOSC source
//          as the device system clock (FOSC<2:0> = 100).
//      The SCS bits in the OSCCON register must be cleared to use the clock
//          determined by FOSC<2:0> in Configuration Words (SCS<1:0> = 00).
//      The IRCF bits in the OSCCON register must be set to the 8 MHz HFINTOSC set
//          to use (IRCF<3:0> = 1110).
//      The SPLLEN bit in the OSCCON register must be set to enable the 4xPLL, or
//          the PLLEN bit of the Configuration Words must be programmed to a 1.
//      When using the PLLEN bit of the Configuration Words, the 4xPLL cannot
//          be disabled by software and the 8 MHz HFINTOSC option will no longer
//          be available.
//  The 4xPLL is not available for use with the internal oscillator when the SCS
//      bits of the OSCCON register are set to 1x. The SCS bits must be set to 00
//      to use the 4xPLL with the internal oscillator.

// User APP defines.

#define UAPP_OSCCON_VAL  0x70

// 32 MHz internal clock; ignore external clock input.

// bit 7 : SPLLEN:  : 1 : 4x PLL is enabled (ignored if #pragma config PLLEN = ON)
// bit 6 : IRCF3    : 1 : Internal Oscillator Frequency Select, 0b1110 -> 8 Mhz
// bit 5 : IRCF2    : 1 : Internal Oscillator Frequency Select, 0b1110 -> 8 Mhz
// bit 4 : IRCF1    : 1 : Internal Oscillator Frequency Select, 0b1110 -> 8 Mhz
// bit 4 : IRCF0    : 0 : Internal Oscillator Frequency Select, 0b1110 -> 8 Mhz
// bit 2 : n/a      : 0 : (unused, don't care)
// bit 1 : SCS1     : 0 : System Clock Select, 0b00 -> Clock determined by FOSC<2:0> in Config Words.
// bit 0 : SCS0     : 0 : System Clock Select, 0b00 -> Clock determined by FOSC<2:0> in Config Words.

#define UAPP_T1CON_VAL  0x30

// TMR1 clock source is instruction clock Fosc/4; Timer1 off;
// 1:8 pre-scaler; T1 oscillator disabled; T1SYNC* ignored;

// bit 7 : TMR1CS1 : 0 : Timer1 Input Clock Source Select, 0b00 -> instruction clock (Fosc/4)
// bit 6 : TMR1CS0 : 0 : Timer1 Input Clock Source Select, 0b00 -> instruction clock (Fosc/4)
// bit 5 : T1CKPS1 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
// bit 4 : T1CKPS0 : 1 : Timer1 Input Clock Prescale Select, 0b11 -> 1:8 Prescale value
// bit 3 : T1OSCEN : 0 : T1 oscillator disabled
// bit 2 : T1SYNC  : 0 : IT1SYNC* ignored
// bit 1 : n/a     : 0 : Unimplemented, ignored
// bit 0 : TMR1ON  : 0 : Timer1 disabled

#if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010

    //   UCFG_PD2P_2010 OR UCFG_PD2P_2002
    //   ********************************

    // Hardware: PICdem 2 Plus 2002 board with 4 MHz external crystal (IGNORED).
    // *OR*
    // Hardware: PICdem 2 Plus 2010 board with no external crystal.

    // Functions:
    //  1) Read 1 A/D channel, convert A/D signal to PWM servo signal.
    //  2) Send status messages to RS-232 port.  Receive RS-232 commands.

    #define UAPP_PORTA_VAL  0x00

    // PORTA cleared so any bits later programmed as output initialized to 0.

    // bit 7 : RA7/OSC1      : 0 : Discrete In: unused (don't care)
    // bit 6 : RA6/OSC2      : 0 : Discrete In: unused (don't care)
    // bit 5 : RA5/MCLR*/Vpp : 0 : Discrete In: unused (don't care)
    // bit 4 : RA4           : 0 : Discrete In: unused (don't care)
    // bit 3 : RA3/AN3       : 0 : Discrete In: unused (don't care)
    // bit 2 : RA2/AN2       : 0 : Discrete In: unused (don't care)
    // bit 1 : RA1/AN1       : 0 : Discrete Out: PWM
    // bit 0 : RA0/AN0       : 0 : Analog In: (don't care)

    #define UAPP_TRISA_VAL  0xFD

    // Set all PORTA to inputs, except for PWM.

    // bit 7 : TRISA7 : 1 : Input (tri-stated)
    // bit 6 : TRISA6 : 1 : Input (tri-stated)
    // bit 5 : TRISA5 : 1 : Input (tri-stated)
    // bit 4 : TRISA4 : 1 : Input (tri-stated)
    // bit 3 : TRISA3 : 1 : Input (tri-stated)
    // bit 2 : TRISA2 : 1 : Input (tri-stated)
    // bit 1 : TRISA1 : 0 : Output (PWM)
    // bit 0 : TRISA0 : 1 : Input (tri-stated)

    #define UAPP_ANSELA_VAL  0x01

    // Set PORTA (when pin used as input) to either digital or analog inputs.

    // bit 7 : n/a   : 0 : Unimplemented, ignored
    // bit 6 : n/a   : 0 : Unimplemented, ignored
    // bit 5 : n/a   : 0 : Unimplemented, ignored
    // bit 4 : ANSA4 : 0 : Digital In
    // bit 3 : ANSA3 : 0 : Digital In
    // bit 2 : ANSA2 : 0 : Digital In
    // bit 1 : ANSA1 : 0 : Digital Out: PWM (don't care)
    // bit 0 : ANSA0 : 1 : Analog In

    #define UAPP_PORTB_VAL  0x00

    // PORTB cleared so any bits later programmed as output initialized to 0.

    // bit 7 : RB7/PGD          : 0 : Discrete In: (don't care)
    // bit 6 : RB6/PGC          : 0 : Discrete In: (don't care)
    // bit 5 : RB5/TX/SCL2      : 0 : Discrete In: Nemesis TX (don't care)
    // bit 4 : RB4/SCL1         : 0 : Discrete In: (don't care)
    // bit 3 : RB3              : 0 : Discrete Out: Init to 0.
    // bit 2 : RB2/(TX)/RX/SDA2 : 0 : Discrete In: Nemesis RX (don't care)
    // bit 1 : RB1/(RX)/SDA1    : 0 : Discrete Out: Init to 0.
    // bit 0 : RB0              : 0 : Discrete Out: Init to 0.

    #define UAPP_TRISB_VAL  0xF4

    // Set TRISB RB0, RB1, RB3 to outputs for LED's.
    // Set TRISB RB2, RB4. RB5, RB6, RB7 to inputs.
    //      (PGC and PGD need to be configured as high-impedance inputs.)

    // bit 7 : DDRB7  : 1 : Input (tri-stated) (sometimes PGD)
    // bit 6 : DDRB6  : 1 : Input (tri-stated) (sometimes PGC)
    // bit 5 : DDRB5  : 1 : Input (tri-stated) USART TX requires input.
    // bit 4 : DDRB4  : 1 : Input (tri-stated): unused
    // bit 3 : DDRB3  : 0 : Discrete Out: LED 4
    // bit 2 : DDRB2  : 1 : Input (tri-stated): USART RX requires input.
    // bit 1 : DDRB1  : 0 : Discrete Out: LED 2
    // bit 0 : DDRB0  : 0 : Discrete Out: LED 1

    #define UAPP_ANSELB_VAL  0x00

    // Set PORTB (when pin used as input) to either digital or analog inputs.

    // bit 7 : ANSB7 : 0 : Digital In
    // bit 6 : ANSB6 : 0 : Digital In
    // bit 5 : ANSB5 : 0 : Digital In
    // bit 4 : ANSB4 : 0 : Digital In
    // bit 3 : ANSB3 : 0 : Digital In
    // bit 2 : ANSB2 : 0 : Digital In
    // bit 1 : ANSB1 : 0 : Digital In
    // bit 0 : n/a   : 0 : Unimplemented, ignored

#endif

#if UCFG_BOARD==UCFG_KA027C

    //   UCFG_KA027C
    //   ***********

    // Hardware: Kairos Autonomi 027C Pot and Quad Encoder board.

    // Functions:
    //  1) Read 1 A/D channel, convert A/D signal to PWM servo signal.
    //  2) Send status messages to RS-232 port.  Receive RS-232 commands.

    #define UAPP_PORTA_VAL  0x00

    // PORTA cleared so any bits later programmed as output initialized to 0.

    // bit 7 : RA7/OSC1      : 0 : Discrete In: unused (don't care)
    // bit 6 : RA6/OSC2      : 0 : Discrete In: unused (don't care)
    // bit 5 : RA5/MCLR*/Vpp : 0 : Discrete In: unused (don't care)
    // bit 4 : RA4           : 0 : Discrete In: unused (don't care)
    // bit 3 : RA3/AN3       : 0 : Discrete Out: LED: init to 0.
    // bit 2 : RA2/AN2       : 0 : Discrete In: unused (don't care)
    // bit 1 : RA1/AN1       : 0 : Discrete Out: PWM: init to 0.
    // bit 0 : RA0/AN0       : 0 : Discrete In: unused (don't care)

    #define UAPP_TRISA_VAL  0xF5

    // Set all PORTA to inputs, except for PWM.

    // bit 7 : TRISA7 : 1 : Input (tri-stated)
    // bit 6 : TRISA6 : 1 : Input (tri-stated)
    // bit 5 : TRISA5 : 1 : Input (tri-stated)
    // bit 4 : TRISA4 : 1 : Input (tri-stated)
    // bit 3 : TRISA3 : 0 : Output: LED
    // bit 2 : TRISA2 : 1 : Input (tri-stated)
    // bit 1 : TRISA1 : 0 : Output: PWM
    // bit 0 : TRISA0 : 1 : Input (tri-stated)

    #define UAPP_ANSELA_VAL  0x00

    // Set PORTA (when pin used as input) to either digital or analog inputs.

    // bit 7 : n/a   : 0 : Unimplemented, ignored
    // bit 6 : n/a   : 0 : Unimplemented, ignored
    // bit 5 : n/a   : 0 : Unimplemented, ignored
    // bit 4 : ANSA4 : 0 : Digital In
    // bit 3 : ANSA3 : 0 : Digital Out: LED (don't care)
    // bit 2 : ANSA2 : 0 : Digital In
    // bit 1 : ANSA1 : 0 : Digital Out: PWM (don't care)
    // bit 0 : ANSA0 : 0 : Digital In

    #define UAPP_PORTB_VAL  0x00

    // PORTB cleared so any bits later programmed as output initialized to 0.

    // bit 7 : RB7/PGD/AN6      : 0 : Analog In: (don't care)
    // bit 6 : RB6/PGC          : 0 : Discrete In: (don't care)
    // bit 5 : RB5/TX/SCL2      : 0 : Discrete In: TX (don't care)
    // bit 4 : RB4/SCL1         : 0 : Discrete In: (don't care)
    // bit 3 : RB3              : 0 : Discrete In: (don't care) (Discrete Out: Init to 0.)
    // bit 2 : RB2/(TX)/RX/SDA2 : 0 : Discrete In: RX (don't care)
    // bit 1 : RB1/(RX)/SDA1    : 0 : Discrete In: (don't care) (Discrete Out: Init to 0.)
    // bit 0 : RB0              : 0 : Discrete In: (don't care) (Discrete Out: Init to 0.)

    #define UAPP_TRISB_VAL  0xFF

    // Set TRISB RB? to output for LED's.
    // Set TRISB for the rest of RBx to inputs.

    // bit 7 : DDRB7  : 1 : Input (tri-stated)
    // bit 6 : DDRB6  : 1 : Input (tri-stated)
    // bit 5 : DDRB5  : 1 : Input (tri-stated) USART TX requires input.
    // bit 4 : DDRB4  : 1 : Input (tri-stated): unused
    // bit 3 : DDRB3  : 1 : Input (tri-stated) (Discrete Out: LED 4)
    // bit 2 : DDRB2  : 1 : Input (tri-stated): USART RX requires input.
    // bit 1 : DDRB1  : 1 : Input (tri-stated) (Discrete Out: LED 2)
    // bit 0 : DDRB0  : 1 : Input (tri-stated) (Discrete Out: LED 1)

    #define UAPP_ANSELB_VAL  0x80

    // Set PORTB (when pin used as input) to either digital or analog inputs.

    // bit 7 : ANSB7 : 1 : Analog In
    // bit 6 : ANSB6 : 0 : Digital In
    // bit 5 : ANSB5 : 0 : Digital In
    // bit 4 : ANSB4 : 0 : Digital In
    // bit 3 : ANSB3 : 0 : Digital In
    // bit 2 : ANSB2 : 0 : Digital In
    // bit 1 : ANSB1 : 0 : Digital In
    // bit 0 : n/a   : 0 : Unimplemented, ignored

#endif

//*******************************************************************************

//#define UAPP_TMR1L_VAL_20MS  0xE0
//#define UAPP_TMR1H_VAL_20MS  0xB1
//
//// 32 MHz Fosc/4 is base clock = 8 MHz = 0.125 us per clock.
//// 1:8 prescale = 0.125 * 8 = 1.0 us per clock.
//// 20,000 counts * 1.0us/clock = 20,000 us/rollover = 20ms/rollover.
//// Timer preload value = 65,536 - 20,000 = 45,536 = 0xB1E0.

#define UAPP_TMR1L_VAL_5MS  0x78
#define UAPP_TMR1H_VAL_5MS  0xEC

// 32 MHz Fosc/4 is base clock = 8 MHz = 0.125 us per clock.
// 1:8 prescale = 0.125 * 8 = 1.0 us per clock.
// 5,000 counts * 1.0us/clock = 5,000 us/rollover = 5ms/rollover.
// Timer preload value = 65,536 - 5,000 = 61,536 = 0xEC78.

#define UAPP_APFCON0_VAL  0x80

// bit 7 : RXDTSEL : 1 : RX/DT function is on RB2
// bit 6 : SDO1SEL : 0 : SDO1 function is on RB2 (unused)
// bit 5 : SS1SEL  : 0 : SS1 function is on RB5 (unused)
// bit 4 : P2BSEL  : 0 : P2B function is on RB7 (unused)
// bit 3 : CCP2SEL : 0 : CCP2/P2A function is on RB6 (unused)
// bit 2 : P1DSEL  : 0 : P1D function is on RB7 (unused)
// bit 1 : P1CSEL  : 0 : P1C function is on RB6 (unused)
// bit 0 : CCP1SEL : 0 : CCP1/P1A function is on RB3 (unused)

#define UAPP_APFCON1_VAL  0x01

// bit 7 : n/a     : 0 : Unimplemented, ignored
// bit 6 : n/a     : 0 : Unimplemented, ignored
// bit 5 : n/a     : 0 : Unimplemented, ignored
// bit 4 : n/a     : 0 : Unimplemented, ignored
// bit 3 : n/a     : 0 : Unimplemented, ignored
// bit 2 : n/a     : 0 : Unimplemented, ignored
// bit 1 : n/a     : 0 : Unimplemented, ignored
// bit 0 : TXCKSEL : 1 : TX/CK function is on RB5

#define UAPP_PIE1_VAL  0x00

// TMR1GIE, ADIE, RCIE, TXIE, SSPIE, CCP1IE, TMR2IE, TMR1IE disabled.

// bit 7 : TMR1GIE : 0 : Disables the Timer1 Gate interrupt
// bit 6 : ADIE    : 0 : Disables the A/D interrupt
// bit 5 : RCIE    : 0 : Disables the USART receive interrupt
// bit 4 : TXIE    : 0 : Disables the USART transmit interrupt
// bit 3 : SSP1IE  : 0 : Disables the MSSP1 interrupt
// bit 2 : CCP1IE  : 0 : Disables the CCP1 interrupt
// bit 1 : TMR2IE  : 0 : Disables the TMR2 to PR2 match interrupt
// bit 0 : TMR1IE  : 0 : Disables the TMR1 overflow interrupt

#define UAPP_PIR1_VAL  0x00

// TMR1GIF, ADIF, RCIF, TXIF, SSPIF, CCP1IF, TMR2IF, TMR1IF cleared.

// bit 7 : TMR1GIF : 0 : No Timer1 Gate interrupt has occurred
// bit 6 : ADIF    : 0 : The A/D conversion is not complete
// bit 5 : RCIF    : 0 : The EUSART receive buffer is empty
// bit 4 : TXIF    : 0 : The EUSART transmit buffer is full
// bit 3 : SSPI1F  : 0 : Waiting to transmit/receive
// bit 2 : CCP1IF  : 0 : No TMR1 register capture occurred
// bit 1 : TMR2IF  : 0 : No TMR2 to PR2 match occurred
// bit 0 : TMR1IF  : 0 : TMR1 register did not overflow

#define UAPP_INTCON_VAL  0xC0

// INTCON changed: GIE, PEIE enabled; TMR0IE, INTIE, IOCE disabled; TMR0IF, INTF, IOCF cleared.
//
// bit 7 : GIE/GIEH  : 1 : Enables all unmasked interrupts
// bit 6 : PEIE/GIEL : 1 : Enables all unmasked peripheral interrupts
// bit 5 : TMR0IE    : 0 : Disables the TMR0 overflow interrupt
// bit 4 : INTIE     : 0 : Disables the INT external interrupt
// bit 3 : IOCE      : 0 : Disables the Interrupt-On-Change interrupt
// bit 2 : TMR0IF    : 0 : TMR0 register did not overflow
// bit 1 : INTF      : 0 : The INT external interrupt did not occur
// bit 0 : IOCF      : 0 : None of the Interrupt-On-Change pins have changed state

#define UAPP_OPTION_REG_VAL_8us  0xD5

// TMR0 clock source is instruction clock Fosc/4;
// pre-scaler is used; 1:64 pre-scaler.

// bit 5 : WPUEN   : 1 : All weak pull-ups are disabled (except MCLR, if it is enabled)
// bit 4 : INTEDG  : 1 : Interrupt on rising edge of RB0/INT pin
// bit 5 : TMR0CS  : 0 : Internal instruction cycle clock (Fosc/4)
// bit 4 : TMR0SE  : 1 : Increment on high-to-low transition on RA4/T0CKI pin
// bit 3 : PSA     : 0 : Prescaler is used by the Timer0 module
// bit 2 : PS2     : 1 : Timer0 Prescale Select, 0b101 -> 1:64 Prescale value
// bit 1 : PS1     : 0 : Timer0 Prescale Select, 0b101 -> 1:64 Prescale value
// bit 0 : PS0     : 1 : Timer0 Prescale Select, 0b101 -> 1:64 Prescale value

// 32 MHz Fosc/4 is base clock = 8 MHz = 0.125 us per clock.
// 1:64 prescale = 0.125 * 64 = 8.0 us per clock.
// 255 counts * 8.0us/clock = 2.040 ms max pulse width.

#define UAPP_OPTION_REG_VAL_32us  0xD7

// TMR0 clock source is instruction clock Fosc/4;
// pre-scaler is used; 1:256 pre-scaler.

// bit 5 : WPUEN   : 1 : All weak pull-ups are disabled (except MCLR, if it is enabled)
// bit 4 : INTEDG  : 1 : Interrupt on rising edge of RB0/INT pin
// bit 5 : TMR0CS  : 0 : Internal instruction cycle clock (Fosc/4)
// bit 4 : TMR0SE  : 1 : Increment on high-to-low transition on RA4/T0CKI pin
// bit 3 : PSA     : 0 : Prescaler is used by the Timer0 module
// bit 2 : PS2     : 1 : Timer0 Prescale Select, 0b111 -> 1:256 Prescale value
// bit 1 : PS1     : 1 : Timer0 Prescale Select, 0b111 -> 1:256 Prescale value
// bit 0 : PS0     : 1 : Timer0 Prescale Select, 0b111 -> 1:256 Prescale value

// 32 MHz Fosc/4 is base clock = 8 MHz = 0.125 us per clock.
// 1:256 prescale = 0.125 * 256 = 32.0 us per clock.
// 255 counts * 32.0us/clock = 8.160 ms max pulse width.

//*******************************************************************************
//
// UAPP_POR_Init_PhaseA: User application Power-On Reset initialization.

void UAPP_POR_Init_PhaseA( void )
{
    OSCCON = UAPP_OSCCON_VAL;   // Configure Fosc. Note relation to CONFIG1H.
}

//*******************************************************************************
//
// UAPP_POR_Init_PhaseB: User application Power-On Reset initialization.

void UAPP_POR_Init_PhaseB( void )
{
    PORTA = UAPP_PORTA_VAL;     // Clear initial data values in port.
    PORTB = UAPP_PORTB_VAL;

    TRISA = UAPP_TRISA_VAL;     // Set port bits function and direction.
    TRISB = UAPP_TRISB_VAL;

    ANSELA = UAPP_ANSELA_VAL;   // Set input pins to digital function or analog.
    ANSELB = UAPP_ANSELB_VAL;

    APFCON0 = UAPP_APFCON0_VAL; // Alternate pin mapping moves RX/TX functions.
    APFCON1 = UAPP_APFCON1_VAL;

    // PIE1 changed: TMR1GIE, ADIE, RCIE, TXIE, SSP1IE, CCP1IE, TMR2IE, TMR1IE disabled.

    PIE1 = UAPP_PIE1_VAL;

    // PIR1 changed: TMR1GIF, ADIF, RCIF, TXIF, SSP1IF, CCP1IF, TMR2IF, TMR1IF cleared.

    PIR1 = UAPP_PIR1_VAL;

    // INTCON changed: GIE, PEIE enabled; TMR0IE, INT0IE, IOCE disabled; TMR0IF, INT0IF, IOCF cleared.

    INTCON = UAPP_INTCON_VAL;

    //   Global interrupts enabled. The following routines
    //   may enable additional specific interrupts.

    UADC_Init();                        // User ADC hardware init.
    UAPP_ClearRcBuffer();               // Clear Rx buffer before messages can arrive.
    USIO_Init();                        // User Serial I/O hardware init.

    UAPP_Flags.UAPP_EchoActive = 0;     // Do not echo received msgs.
    UAPP_Flags.UAPP_ReportActive = 1;   // Do report data.

    UAPP_Flags.UAPP_100PctActive = 0;   // Default to 20ms period, 900-2040us width.
    UAPP_CheckDiscrete_100PctActive();  // Check for override to 5ms period, 0-5ms width.

    //  Now before we can place chars in the TX buffer, which will cause the USART
    //      HW to start transmitting, we need to ensure that the 4X PLL is running,
    //      and that the HFINTOSC is at least 0.5% accurate.
    //  Otherwise the USART clocks derived from the system clock are not accurate
    //      and this initial message will be trash.
    //  Also note that any characters received before the below conditions are met
    //      are likely to be trashed as well.  Hopefully any device sending us a
    //      message waits to see our init message firest.

    // Spin until PLLR == 1 (4X PLL ready) AND HFIOFS == 1 (HF int osc accurate).
    while( !OSCSTATbits.PLLR || !OSCSTATbits.HFIOFS );

    SSIO_PutStringTxBuffer( (char*) UAPP_MsgInit );     // Version message.
}

//*******************************************************************************
//
// UAPP_Timer0_Init: Init Timer0 module to generate PWM for Timer0_Duration.
//
//  Timer0_Duration: the number of 8us (or 32us) counts for PWM active high.
//
//  When Timer0 expires, SISD is going to get an int,
//  then PWM will be finished in UAPP_Timer0_Expired().

#pragma interrupt_level 1
void UAPP_Timer0_Init( unsigned char Timer0_Duration )
{
unsigned char Timer0_PreLoad;

    if( UAPP_Flags.UAPP_100PctActive )
        OPTION_REG = UAPP_OPTION_REG_VAL_32us;  // Initialize Timer0 to 32us/count.
    else
        OPTION_REG = UAPP_OPTION_REG_VAL_8us;   // Initialize Timer0 to  8us/count.

    if( Timer0_Duration > 0 )           // Only create pulse for non-zero duration.
        {
        //  Form pre-load value from requested duration.
        Timer0_PreLoad = (unsigned char) 0xFF - Timer0_Duration;

        TMR0 = Timer0_PreLoad;          // Load Timer0 to get desired duration.

        LATAbits.LATA1 = 1;             // PWM signal high begins the pulse.

        INTCONbits.TMR0IF = 0;          // Clear Timer0 interrupt flag.
        INTCONbits.TMR0IE = 1;          // Enable Timer0 interrupts.
        }
}

//*******************************************************************************
//
// Timer0 has expired, finish the PWM.

#pragma interrupt_level 1
void UAPP_Timer0_Expired( void )
{
    LATAbits.LATA1 = 0;     // PWM signal low completes the pulse.
}

//*******************************************************************************
//
// UAPP_Timer1_Init: Init Timer1 module to generate timer interrupt every XX ms.

#pragma interrupt_level 1
void UAPP_Timer1_Init( void )
{
    T1CON = UAPP_T1CON_VAL;         // Initialize Timer1 but don't start it.
    TMR1L = UAPP_TMR1L_VAL_5MS;     // Timer1 pre-load value,  5ms, low byte.
    TMR1H = UAPP_TMR1H_VAL_5MS;     // Timer1 pre-load value,  5ms, high byte.

    PIE1bits.TMR1IE = 1;      // Enable Timer1 interrupts.
    T1CONbits.TMR1ON = 1;     // Turn on Timer1 module.
}

//*******************************************************************************
//
// UAPP_Task1: Task1

void UAPP_Task1( void )
{
    #if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
        LATBbits.LATB0 ^= 1;            //  Toggle LED 1.
    #elif UCFG_BOARD==UCFG_KA027C
        LATAbits.LATA3 ^= 1;            //  Toggle LED 2.
    #endif

    // If 100% mode active then start ADC and PWM pulse every 5ms. */
    if( UAPP_Flags.UAPP_100PctActive )
        UADC_Trigger();                 //  Trigger an A/D conversion.
                                        //  UADC_Trigger enabled ADC interrupts.
}

//*******************************************************************************
//
// UAPP_Task2: Task2.

void UAPP_Task2( void )
{
    #if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
        LATBbits.LATB1 = LATBbits.LATB1^1;  // Toggle LED 2.
    #endif

    // If 100% mode NOT active then start ADC and PWM pulse every 20ms. */
    if( !UAPP_Flags.UAPP_100PctActive )
        UADC_Trigger();                 //  Trigger an A/D conversion.
                                        //  UADC_Trigger enabled ADC interrupts.
}

//*******************************************************************************
//
// UAPP_Task3: Task3.

void UAPP_Task3( void )
{
unsigned char i;
char c;

    // Check for override to 5ms period, 0-5ms width.
    UAPP_CheckDiscrete_100PctActive();

    #if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
        LATBbits.LATB3 = LATBbits.LATB3^1;  //  Toggle LED 4.
    #endif

    //  Copy TaskADC (5 or 20ms) message to output buffer if reporting active.
    if( UAPP_Flags.UAPP_ReportActive )
        for( i = 0, c = 'X'; (i < UAPP_BUFFERLENGTH) && (c != '\0'); i++ )
            {
            c = UAPP_ADC_PWM_Msg[ i ];
            SSIO_PutByteTxBuffer( c );
            }
}

//*******************************************************************************
//
// UAPP_TaskADC: Convert A/D result, output it, then convert to 0 to 5v and
//      output that.  Then start PWM pulse high by calling UAPP_Timer0_Init().

void UAPP_TaskADC( void )
{
SUTL_Word       UAPP_RawADC;
SUTL_ShortLong  UAPP_ResultBCD;
SUTL_Long       UAPP_Intermediate;
unsigned char i;

    UAPP_RawADC.word = UADC_Raw10Bit();         // Get raw A/D result.

    //  Copy raw A/D result to buffer where it can be output at lower frequency.
    UAPP_ADC_PWM_Msg[i=0] = '[';
    UAPP_ADC_PWM_Msg[++i] = 'R';
    UAPP_ADC_PWM_Msg[++i] = ':';

    //  UAPP_RawADC.word contains right-justified 10-bit result, upper 6 bits are 0.
    //  0x0000 = 0.00 Vdc, 0x03ff = 5.00 Vdc.

    //  Convert raw A/D to engineering units, assuming 1023 counts = 0x3ff = 5 volts.
    //  Nadc(computer counts) = Eadc(volts) * (1023 counts/5.0 volts) so Nadc = Eadc * 204.6
    //  (With Nadc = Eadc * 204.6, Eadc = Nadc/204.6, or
    //   Eadc(volts) = Nadc * 0.00489 (each Nadc is 4.89mV) )

    //  For display purposes, we desire E = N * .001 (each N is 1.0 mV), or N = E * 1000.
    //  Therefore we rescale from N = E * 204.6 to N = E * 1000, so mult by 1000/204.6 (or 4.89).
    //  Since we cannot do integer multiply by 4.89 directly, we use an extra 8 bits of precision
    //  in the multiplier and then discard the extra 8 bits in the result.

    //  Begin with N = E * 204.6.
    //  Multiply by ((1000/204.6)*256) = 1251, now N = E * 1000 * 256.
    //  Dropping lower 8 bits of result is like dividing by 256, now N = E * 1000.

    //  32-bit result.
    UAPP_Intermediate.long0 = (unsigned long) UAPP_RawADC.word * (unsigned long) 1251;

    //  Byte reassignment implements / 256.
    UAPP_Intermediate.byte0 = UAPP_Intermediate.byte1;
    UAPP_Intermediate.byte1 = UAPP_Intermediate.byte2;

    //  Convert from 16-bit (N = E * 1000) to 6 nibble BCD.
    UAPP_ResultBCD.shortLong = SUTL_EngToBCD_16u( UAPP_Intermediate.word0 );

    //  Format 4 non-zero nibbles as ASCII and copy to buffer.
    UAPP_ADC_PWM_Msg[++i] = ' ';
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble3 ];
    UAPP_ADC_PWM_Msg[++i] = '.';
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble2 ];
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble1 ];
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble0 ];
    UAPP_ADC_PWM_Msg[++i] = 'V';

    if( !UAPP_Flags.UAPP_100PctActive)
        {
        //  0 -> 20ms PWM period, 900us-2040us width.

        //  UAPP_RawADC.word contains right-justified 10-bit result, upper 6 bits are 0.
        //  0x0000 = 0.00 Vdc, 0x03ff = 5.00 Vdc.

        //  The count passed to UAPP_Timer0_Init is in 8us increments.
        //  Convert raw A/D to PWM 8us counts, using desired mapping for servos...
        //  ... Npwm = ((Nadc / 8) + 90) * 10/8.
        //  Here's the mapping for several voltages, and the resulting 8us counts.
        //  Npwm(Nadc=0)    = (  0 + 90) * 10us/8us = 108 8us counts =  0x6c =  900us PWM (lower limit)
        //  Npwm(Nadc=912)  = (114 + 90) * 10us/8us = 255 8us counts =  0xff = 2040us PWM (upper limit)
        //  Npwm(Nadc=1023) = (127 + 90) * 10us/8us = 271 8us counts = 0x10f = 2170us PWM (limit at 0xff)
        //  271 won't fit in 8 bits, so max Npwm at 255, then 255 Npwm/8us ===> 2040us PWM max width.

        //  For PWM purposes, since each Nadc ranges 0-1023 and Npwm ranges 0-127, Npwm = Nadc / 8.
        //  Here's the fixed point math we'll use to convert from Nadc to Npwm.
        //  Npwm = ((Nadc / 8) + 90) * 10 / 8
        //  Npwm =  (Nadc / 8) * 10 / 8 + (90 * 10 / 8)
        //  Npwm =  (Nadc * 10 / 64)    + (90 * 10 * 8 / 64)
        //  Npwm = ((Nadc * 10 )        + (90 * 10 * 8)) / 64
        //  Npwm = ((Nadc * 10 )        + 7200)          / 64

        //  Therefore we multiply Nadc by 10, add 7200, and shift right by 6 bits to / 64.
        //  After this we must ensure that there is no overflow past 255.

        UAPP_RawADC.word *= 10;         //  Multiply Nadc by 10.
        UAPP_RawADC.word += 7200;       //  Add 7200 offset to get proper range.
        UAPP_RawADC.word >>= 6;         //  Shift Npwm 6 bits right to /64.

        if( UAPP_RawADC.word > 255 )    //  If overflows 8-bit byte..
            UAPP_RawADC.word = 255;     //  ..then max it at 255.

        //  Multiply number of 8us counts by 8 to get units us, 16-bit result, for serial message.
        //  Compiler should recognise this as left shift by 3 bits.
        UAPP_Intermediate.word0 = (unsigned char) UAPP_RawADC.byte0 * (unsigned char) 8;

        }   // ( !UAPP_Flags.UAPP_100PctActive)
    else
        {
        //  1 -> 5ms PWM period, 0-5ms width.

        //  UAPP_RawADC.word contains right-justified 10-bit result, upper 6 bits are 0.
        //  0x0000 = 0.00 Vdc, 0x03ff = 5.00 Vdc.

        //  The count passed to UAPP_Timer0_Init is in 32us increments.
        //  Convert raw A/D to PWM 32us counts, using desired mapping for 0-100%, Npwm = (Nadc * 40 / 256).
        //  Here's the mapping for several voltages, and the resulting 32us counts.
        //  Npwm(Nadc=0)    = (    0 * 40 / 256 =    0 32us counts =  0x00 =    0us PWM (lower limit)
        //  Npwm(Nadc=1023) = ( 1023 * 40 / 256 =  159 32us counts =  0x9f = 5088us PWM (upper limit)

        //  Here's the fixed point math we'll use to convert from Nadc to Npwm.
        //  Npwm = (Nadc * 40) / 256

        //  Therefore we multiply Nadc by 40, and shift right by 8 bits to / 256.

        UAPP_RawADC.word *= 40;         //  Multiply Nadc by 40.
        UAPP_RawADC.word >>= 8;         //  Shift Npwm 6 bits right to / 256.

        //  Multiply number of 32us counts by 32 to get units us, 16-bit result, for serial message.
        //  Compiler should recognise this as left shift by 5 bits.
        UAPP_Intermediate.word0 = (unsigned char) UAPP_RawADC.byte0 * (unsigned char) 32;

        }   // ( UAPP_Flags.UAPP_100PctActive)

    //  Convert from 16-bit count of us to 6 nibble BCD.
    UAPP_ResultBCD.shortLong = SUTL_EngToBCD_16u( UAPP_Intermediate.word0 );

    //  Format 4 lower non-zero nibbles as ASCII and copy to buffer.
    UAPP_ADC_PWM_Msg[++i] = ' ';
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble3 ];
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble2 ];
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble1 ];
    UAPP_ADC_PWM_Msg[++i] = UAPP_Nibble_ASCII[ UAPP_ResultBCD.nibble0 ];
    UAPP_ADC_PWM_Msg[++i] = 'u';
    UAPP_ADC_PWM_Msg[++i] = 's';
    UAPP_ADC_PWM_Msg[++i] = ']';
    UAPP_ADC_PWM_Msg[++i] = '\n';
    UAPP_ADC_PWM_Msg[++i] = '\r';
    UAPP_ADC_PWM_Msg[++i] = '\0';

    //  Pass number of 8us (or 32us) counts to PWM routine.
    UAPP_Timer0_Init( UAPP_RawADC.byte0 );
}

//*******************************************************************************
//
// Background Task.  UNUSED.

void UAPP_BkgdTask( void )
{
}

//*******************************************************************************
//
//  Clear UAPP Rc buffer.
//
void UAPP_ClearRcBuffer( void )
{
    UAPP_IndexRc = 0;
}

//*******************************************************************************
//
//  Copy input byte to UAPP Rc buffer.
//
void UAPP_PutByteRcBuffer( char RcChar )
{
    UAPP_BufferRc[UAPP_IndexRc++] = RcChar;
}

//*******************************************************************************
//
//  Parse command string in UAPP Rx buffer.
//
void UAPP_ParseRcMsg( void )
{
unsigned char i;

    if( UAPP_Flags.UAPP_EchoActive )                // Echo msg if flag active.
        {
        for( i=0; i<UAPP_IndexRc; i++ )
            if( UAPP_BufferRc[i] != 0x0a && UAPP_BufferRc[i] != 0x0d )
                SSIO_PutByteTxBuffer( UAPP_BufferRc[i] );

        SSIO_PutByteTxBuffer( '\n' );
        SSIO_PutByteTxBuffer( '\r' );
        }

    if( UAPP_BufferRc[0] == '[' )
    {
        switch( UAPP_BufferRc[1] )
        {
        case 'H':
            SSIO_PutStringTxBuffer( (char*) UAPP_MsgHelp ); // Help message.
            break;  // case 'H'

         case 'V':
            SSIO_PutStringTxBuffer( (char*) UAPP_MsgInit ); // Version message.
            break;  // case 'V'

        case 'E':
            UAPP_Flags.UAPP_EchoActive ^= 1;                // Toggle whether to echo msgs.
            if( UAPP_Flags.UAPP_EchoActive )                // Echo msg if flag active.
                SSIO_PutStringTxBuffer( (char*) UAPP_MsgEchoActive );   // Now echoing.
            else
                SSIO_PutStringTxBuffer( (char*) UAPP_MsgEchoInactive ); // Quit echoing.
            break;  // case 'E'

        case 'P':
            UAPP_Flags.UAPP_100PctActive ^= 1;  // Toggle whether to use 100% PWM.
            UAPP_CheckDiscrete_100PctActive();  // Check for override to 5ms period, 0-5ms width.
            if( UAPP_Flags.UAPP_100PctActive )  // Respond with confirmation message.
                SSIO_PutStringTxBuffer( (char*) UAPP_Msg100PctActive );   // Now 100% PWM.
            else
                SSIO_PutStringTxBuffer( (char*) UAPP_Msg100PctInactive ); // Quit 100% PWM.
            break;  // case 'P'

        case 'R':
            UAPP_Flags.UAPP_ReportActive ^= 1;          // Toggle whether to report data.
            if( UAPP_Flags.UAPP_ReportActive )          // Respond with confirmation message.
                SSIO_PutStringTxBuffer( (char*) UAPP_MsgReportActive );     // Now reporting.
            else
                SSIO_PutStringTxBuffer( (char*) UAPP_MsgReportInactive );   // Quit reporting.
            break;  // case 'R'

        case 'B':
            SSIO_PutStringTxBuffer( (char*) UAPP_MsgNotImplemented );   // Msg not implmented.
            //SUTL_InvokeBootloader(); // Not implemented yet.
            break;  // case 'B'

        default:
            SSIO_PutStringTxBuffer( (char*) UAPP_MsgNotRecognized );    // Msg not recognized.
            break;
        };  // switch( UAPP_BufferRc[1] )
    }

    UAPP_ClearRcBuffer();
}

//*******************************************************************************
//
//  Set UAPP_Flags.UAPP_100PctActive if discrete input pulled low.
//
//  NOTE: This overrides UAPP_Flags.UAPP_100PctActive that may have been set
//          in initialization, or from serial messages.  Also, if the discrete
//          input is left floating high, then no effect here, so init or msgs
//          control UAPP_Flags.UAPP_100PctActive.
//
void UAPP_CheckDiscrete_100PctActive( void )
{
#if UCFG_BOARD==UCFG_PD2P_2002 || UCFG_BOARD==UCFG_PD2P_2010
    if( !PORTAbits.RA4 )                    // If discrete input pulled low..
        UAPP_Flags.UAPP_100PctActive = 1;   // ..then 5ms period, 0-5ms width.
#elif UCFG_BOARD==UCFG_KA027C
    if( !PORTAbits.RA0 )                    // If discrete input not pulled low..
        UAPP_Flags.UAPP_100PctActive = 1;   // ..else 5ms period, 0-5ms width.
#endif
}

