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
//  MODULE NAME:        Intr_def.h
//
//  MECHANICS:          This is the definitions file for the Intr.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.06   151111  DAH - Added user-initiated waveform request flag for the test port
//   0.15   160718  DAH - Added struct BB_TIME_STAMP, BB_TS_XMIT_SIZE, and BB_TIME_STAMP Flags to support
//                        storing waveform samples in the Black Box FRAM
//                      - Added struct SAMPLE_PACKET and SAMPLE_PKT_XMIT_SIZE to support storing waveform
//                        samples in the Black Box FRAM
//   0.21   180301  DAH - Renamed the voltages in struct SAMPLE_PACKET from Vxn1 and Vxn2 to VxnAFE and
//                        VxnADC
//                      - Added user-initiated waveform request/acknowledge flags
//                      - Added TOTAL_SAMPLE_SETS
//                      - Added struct RAM_SAMPLES
//   0.23   180504  DAH - Added struct USER_SAMPLES
//   0.26   180622  DAH - Cleaned up struct SAMPLE_PACKET definition (use RAM_SAMPLES instead of separate
//                        identical definitions)
//   0.27   181001  DAH - Rewrote user waveform flag definitions
//   0.31   190506  DAH - Removed Harmonics computation from User Waveform captures.  They are now
//                        completely independent
//                          - Deleted USR_TYPE_TP from the capture request types.  This was a special
//                            capture request that requested a single cycle of all waveforms without a
//                            harmonics computation.  Since harmonics will no longer be computed when a user
//                            capture is done, this request is the same as USER_TYPE_ALL.
//   0.59   220831  DAH - Deleted BB_TS_XMIT_SIZE, BB_TIME_STAMP, and ROLLED_OVER
//   42     230623  MAG - Added TIM4_FREQ, CLOSE_TIME_MS, OPEN_TIME_MS, CLOSE_TIME_CNT, and OPEN_TIME_CNT.
//   94     231010  DAH - Deleted struct USER_SAMPLES definition (moved to Meter_def.h)
//                      - Deleted user capture source definitions since no longer used
//                      - Deleted user capture type definitions (moved to Meter_def.h)
//                      - Deleted struct SAMPLE_PACKET definition since it is no longer used
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

// Number of RAM sample sets
//   Total number of sample sets = 39 cycles x 80 sample sets / cycle
// Need 39 sets so can overwrite the last earliest 3 sets while transferring the 36 sets to Flash (this
//   provides 3 x 1/60sec = 50msec to transfer a trip waveform capture to Flash)
#define TOTAL_SAMPLE_SETS       3120

// Breaker open/close time for advance time calculations based on 35ms open, 40ms close times. 
// May need to adjust these to account for external relay times, GOOSE messaging, etc.
#define TIM4_FREQ               1500000
#define CLOSE_TIME_MS           40
#define OPEN_TIME_MS            35
#define CLOSE_TIME_CNT          TIM4_FREQ * CLOSE_TIME_MS / 1000 
#define OPEN_TIME_CNT           TIM4_FREQ * OPEN_TIME_MS / 1000 

#define COUNT_10_MIN            60000


//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct RAM_SAMPLES
{
    float Ia;
    float Ib;
    float Ic;
    float In;
    float Igsrc;
    float Igres;
    int16_t VanAFE;
    int16_t VbnAFE;
    int16_t VcnAFE;
    int16_t VanADC;
    int16_t VbnADC;
    int16_t VcnADC;
};

