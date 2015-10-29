//*******************************************************************************
// tinyRTX Filename: ssio.h (System Serial I/O communication services)
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
//  14Oct15 Stephen_Higgins@KairosAutonomi.com
//              Created from pic18 ssio.h.
//
//*******************************************************************************

// External type definitions.

// External variables.

// External prototypes.

void    SSIO_InitFlags( void );
void    SSIO_InitTxBuffer( void );
void    SSIO_InitRcBuffer( void );
void    SSIO_PutByteIntoTxHW( void );
void    SSIO_GetByteFromRcHW( void );
void    SSIO_PutByteTxBuffer( char );
void    SSIO_PutByteRcBuffer( char );
char    SSIO_GetByteTxBuffer( void );
char    SSIO_GetByteRcBuffer( void );
void    SSIO_PutStringTxBuffer( char* );
