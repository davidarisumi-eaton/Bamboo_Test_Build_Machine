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
//  MODULE NAME:        RealTime_def.h
//
//  MECHANICS:          This is the definitions file for the RealTime.c module
//
//  TARGET HARDWARE:    PXR35 Rev 4 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.42   201202  DAH File Creation.  Code moved from Iod_def.h
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
// Definitions

// SysTick Definitions
#define     COUNT_16MHZ_10MSEC      (uint32_t)19999             // Count for 10msec tick at 16MHz
                                                                //   16MHz/8 * 10msec - 1 = 1999
#define     COUNT_120MHZ_10MSEC     (uint32_t)149999            // Count for 10msec tick at 120MHz
                                                                //   120MHz/8 * 10msec - 1 = 149999


//
//------------------------------------------------------------------------------------------------------------
//    Constants
//------------------------------------------------------------------------------------------------------------




//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct SYSTICK_TIME
{
    uint32_t cnt_sec;                   // Seconds past January 1, 2000
    uint32_t cnt_10msec;                // Ten-millisecond counter, resets at one second (100)
};

struct INTERNAL_TIME
{
    uint32_t Time_secs;                 // Seconds past January 1, 2000
    uint32_t Time_nsec;                 // Nanoseconds past the last second
};
    


