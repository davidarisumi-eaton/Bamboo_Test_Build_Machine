//------------------------------------------------------------------------------------------------------------
//                      Eaton Corporation
//
//                      Proprietary Information
//                      (C) Copyright 2015
//                      All rights reserved
//
//                      PXR35 Electronic Trip Unit
//
//------------------------------------------------------------------------------------------------------------
//  AUTHORS:            Daniel A. Hosko         (412)893-2834
//                      Eaton Corporation
//                      1000 Cherrington Parkway
//                      Moon Twp, PA  15108-4312
//                      (412)893-3300
//
//------------------------------------------------------------------------------------------------------------
//  PRODUCT:            PXR35       Trip unit for air circuit and molded-case circuit breakers
//
//  FIRMWARE DRAWING:   ????????    This drawing combines the unprogrammed STM32F4207 with the code's
//                                  flash programming file to produce an "assembly group" that is the
//                                  programmed device.
//
//  PROCESSOR:          ST Micro STM32F407
//
//  COMPILER:           IAR C/C++ Compiler for ARM - v8.40.1.21539
//                      IAR Embedded Workbench from IAR Systems
//
//  MODULE NAME:        CAMCom_def.h
//
//  MECHANICS:          This is the definitions file for the CAMCom.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.15   160718  DAH - Added CAMPORTVARS and CAM.Status flag defs to support basic CAM port testing
//   0.16   160818  DAH - Renamed NumChars to TxNumChars in struct CAMPORTVARS
//
//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
//                    Includes and Declarations
// Path for <>:
//   1) Directories with the -I option
//   2) Directories specified using the C_INCLUDE environment variable, if any
//   3) The automatically set up library system include directories
// Path for "":
//   1) Directory of the source file in which the #include statement occurs
//   2) Directories with the -I option
//   3) Directories specified using the C_INCLUDE environment variable, if any
// Path for library files:
//   IAR Systems\Embedded Workbench 8.3\arm\inc\c
//
//------------------------------------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------------------------------------
//    Constants
//------------------------------------------------------------------------------------------------------------

#define CAMPORT_RXBUFSIZE   96
#define CAMPORT_TXBUFSIZE   52




//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct CAMPORTVARS
{
   uint8_t          TxState;
   uint8_t          RxState;
   uint8_t          RxCharsLeft;
   uint8_t          Temp;
   uint8_t          RxNdxOut;
   uint8_t          Status;
   uint8_t          TxNumChars;
   uint8_t          Spare;                  // Spare to keep alignment ok
   uint8_t          RxBuf[CAMPORT_RXBUFSIZE];
   char             TxBuf[CAMPORT_TXBUFSIZE];
   uint32_t         DMA_HISR_TCIF_FlagMask;
};


// CAM.Status flag definitions
// b7..b4 - unused
#define CAM_FIRST_SAMPLE    0x08            // First sample
#define CAM_ERROR           0x04            // Error
#define CAM_XMIT            0x02            // Transmit command issued
#define CAM_TYPE_SAMPLE     0x01            // Sampled-value CAM

