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
//  MODULE NAME:        Ovrcom_def.h
//
//  MECHANICS:          This is the definitions file for the Ovrcom.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   54     230803  BP  - Added structure for I2C Override comms
//----------------------------------------------------------------
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

#define OVR_I2C_RXLEN           10
#define OVR_I2C_TXLEN           14               
                                        

//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct OVR_I2C_VARS 
{
    uint8_t TxNdx;
    uint8_t RxNdx;
    uint8_t Status;
    uint8_t ErrCount;
    uint8_t RxBuf[OVR_I2C_RXLEN];
    uint8_t TxBuf[OVR_I2C_TXLEN];
}; 
