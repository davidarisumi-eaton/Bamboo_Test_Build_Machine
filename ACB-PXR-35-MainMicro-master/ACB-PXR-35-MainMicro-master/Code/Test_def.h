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
//  MODULE NAME:        Test_def.h
//
//  MECHANICS:          This is the definitions file for the Test.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.04   150928  DAH - Increased size of TxValBuf[] in TESTPORTVARS structure from 32 to 85
//   0.05   151021  DAH - Added Temp, *ValPtr1, and *ValPtr2 to struct TESTPORTVARS
//   0.09   160224  DAH - Added SubState to struct TESTPORTVARS
//                      - Rearranged definitions in TESTPORTVARS to group uint8_t vars all together
//   0.10   160310  DAH - Added FTemp and FTemp1 to struct TESTPORTVARS
//   0.12   160502  DAH - Added *ValPtr3 to struct TESTPORTVARS
//   0.13   160512  DAH - Added *ValPtr4 to struct TESTPORTVARS
//   0.14   160620  DAH - Added support for test-injection
//                          - struct TEST_INJECTION and struct TESTINJ_CAL added
//                          - TEST_INJ_ON, TEST_INJ_INIT_ON, TEST_INJ_INIT_OFF, and TEST_INJ_CHANGE_A1 added
//   0.16   160818  DAH - Replaced TX_STRING with TP_TX_STRING
//                      - Replaced TX_VALUE with TP_TX_VALUE
//                      - Added *ValPtr5 to struct TESTPORTVARS to support DM Test Port command
//   0.19   170223  DAH - Added Channel to struct TEST_INJECTION
//   0.25   180621  DAH - Added support for DA command to display Alarm waveforms
//                          - In struct TESTPORTVARS, changed FTemp and FTemp1 definitions from floats to
//                            unions with uint32_t and float for greater flexibility and renamed to Tmp1 and
//                            Tmp2
//                          - Added union tp_buf definition
//                          - Increased TxValBuf (in TESTPORTVARS) size from 85 to 100
//   0.26   180627  DAH - Added TestPort_States (now global).  Also added TP_INIT to TestPort_States
//   0.27   181001  DAH - Added TP_EA_WFDONE to TestPort_States
//   0.29   190122  DAH - Added TEMP_MEAS_ON definition to test injection flags to support coil temperature
//                        measurement
//   0.34   191002  DAH - Deleted TP_EA_DONE from TestPort_States as it is no longer used
//   0.35   191118  DAH - Added TP_GF and TP_NF to TestPort_States
//   0.45   210504  DAH - Added TP_INIT1 to TestPort_States
//   0.46   210521  DAH - unt16_t u16val[] changed to int16_t i16val[] in union tp_buf to support display of
//                        voltage waveform samples.  u16val[] was previously not used
//   0.48   210730  DAH - Added TP_MT to TestPort_States to support temporary test port command
//   0.60   220928  DAH - Added TP_DV to TestPort_States to support displaying the summary logs ("DV" cmnd)
//   0.61   221001  DB  - Added TP_DI and TP_DJ to TestPort_States to support displaying trip and time
//                        adjustment event logs
//   0.63   221111  DB  - Added TP_DN thru TP_LW to TestPort_States to support displaying the event logs
//   0.67   221209  DAH - Added TP_TV to TestPort_States to support testing the protection availability LEDs
//   0.72   220320  DB  - Added TP_DZ to TestPort_States
//    72    230906  DAH - Added TP_DQ to TestPort States
//    94    231010  DAH - Deleted TP_EA_WFDONE from TestPort_States as it is no longer used
//                      - Added CAL2 to Cal_States
//   108    231108  DAH - Added CAL6 to Cal_States
//   142    240119  DAH - In struct EXACTVARS, changed target definition from uint32_t to float
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

enum TestPort_States
{
  TP_INIT = 0,
  TP_INIT1,
  TP_IDLE, TP_CURSOR, TP_CHECK_FOR_CMND, TP_PARSECMNDSTRING,
  TP_TL,
  TP_TI,
  TP_TV,
  TP_EA,
  TP_DR0, TP_DR1, TP_DR2, TP_DR3, TP_DR4, TP_DR5,
  TP_DE,
  TP_DQ,
  TP_MC,
  TP_MT,                                // *** DAH 210730   ADDED FOR TEST PURPOSES ONLY - DELETE LATER
  TP_TM,
  TP_DW,
  TP_TH,
  TP_OC,
  TP_GC,
  TP_PC,
  TP_IOC,
  TP_IGC,
  TP_EE,
  TP_AG,
  TP_IT,
  TP_CC,
  TP_DS,
  TP_DM,
  TP_TR,
  TP_RU,
  TP_DD,
  TP_WR_CAL,
  TP_WR_CAL1,
  TP_DA,
  TP_DX,
  TP_DV,    // Summary Event Log
  TP_DI,    // Trip Event log
  TP_DF,    // Test Trip Event log
  TP_DN,    // Alarm Event Log
  TP_DJ,    // Time Adjustment Event log
  TP_DZ,    // Disturbance capture log
  TP_LS,
  TP_LT,
  TP_LR,
  TP_LM,
  TP_LD,
  TP_LU,
  TP_LV,
  TP_LW,
  TP_GF,
  TP_NF,
  TP_MX,    //Extended 6s OneCycle Capture Snaphsots
  TP_MZ     //Extended 60s 200mCycle Capture Snaphsots
};



//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct TESTPORTVARS
{
   uint8_t              State;
   uint8_t              SubState;
   uint8_t              RxNdxIn;
   uint8_t              RxNdxOut;
   uint8_t              Status;
   uint8_t              Temp;
   uint8_t              TxValNdx;
   uint8_t              NumChars;
   uint8_t              RxBuf[32];
   char                 TxValBuf[100];
   const uint8_t        *StrPtr;
   float                *ValPtr1;
   float                *ValPtr2;
   uint8_t              *ValPtr3;
   unsigned long long   *ValPtr4;
   void                 *ValPtr5;
   union f_u
   {
     float f;
     uint32_t u;
     uint8_t b[4];
   } Tmp1, Tmp2;
};

struct EXACTVARS
{
   uint8_t              State;
   uint8_t              SubState;
   uint8_t              Temp;
   float                *mPtr;
   float                *calPtr;
   float                *calPtrArray[5];
   union f_un
   {
     float f;
     uint32_t u;
     uint8_t b[4];
   } Tmp2, Tmp1, Temp1[5];
   uint8_t              channel;
   float                target;
   uint8_t              channels_amount;
   uint16_t             LED_image;
   uint16_t             Relay_image;
};

#define NO_MANUF_TEST 0xFFFF  //No Munufacturing test is currently required

// TP.Status flag definitions
// b7..b2 - unused
#define TP_TX_STRING    0x02            // Transmitting string constants
#define TP_TX_VALUE     0x01            // Transmitting characters from RAM



struct TEST_INJECTION                   // Structure for on-board test injection
{
  uint8_t Flags;
  uint8_t CosIndx;
  uint8_t Channel;
  float Amplitude;
  float MidPoint;
};

// On-board test injection flags
#define TEST_INJ_ON             0x0001
#define TEST_INJ_INIT_ON        0x0002
#define TEST_INJ_INIT_OFF       0x0004
#define TEST_INJ_CHANGE_A1      0x0008
#define COIL_MEAS_ON            0x0010


struct TESTINJ_CAL
{
  float midpoint_ph;
  float midpoint_gnd;
  float m_dc[5];
  float b_dc[5];
  float m_sine[5];
  float b_sine[5];
  uint32_t chk;
  uint32_t cmp;
};

union tp_buf
{
  struct ENERGY_DEMAND_STRUCT d;
  float fval[24];
  int16_t i16val[48];
  struct FLASH_INT_REQ a;
};

enum Cal_States
{
  CAL0, CAL1, CAL2, CAL3, CAL4, CAL5, CAL6
};
  

enum ExActStateMachine_States
{
  IDLE, GCAFE, OCAFE, GCH, OCH, GCL, OCL, WRCAL, LED_TEST, RELAY_TEST, OCSI, GCSI 
};


