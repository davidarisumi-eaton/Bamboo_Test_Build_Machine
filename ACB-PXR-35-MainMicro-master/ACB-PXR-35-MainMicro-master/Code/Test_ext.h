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
//  MODULE NAME:        Test_ext.h
//
//  MECHANICS:          This is the declarations file for the Test.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150617  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.05   151021  DAH - Added TP_AFECommsOff and TP_AFEIntOff
//   0.14   160620  DAH - Added support for on-board test-injection
//                          - struct TestInj and struct TestInjCal added
//   0.19   170223  DAH - Added TestInjCur()
//   0.25   180621  DAH - Added support for DA command to display Alarm waveforms
//                          - union tbuf added
//   0.26   180627  DAH - Deleted Test_VarInit()
//   0.93   231010  BP  - Added TestInjCur_OvrMicro()
//    94    231011  DAH - Deleted TP_AFECommsOff as it is no longer used
//   108    231108  DAH - Added Test_VarInit()
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
//                    Global Variable Declarations
//------------------------------------------------------------------------------------------------------------
//
extern struct TESTPORTVARS TP;
extern struct EXACTVARS    ExAct;
extern struct TEST_INJECTION TestInj;
extern struct TESTINJ_CAL TestInjCal;
extern union tp_buf tbuf;

extern uint8_t TP_AFEIntOff;                   // *** DAH TEST



//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Test_VarInit();
extern void TP_Top(void);
extern void ExAct_Top(void);
extern uint8_t Load_ExecuteAction_struct(uint8_t bid, uint8_t *msg);
extern void TestInjCur(uint8_t channel, float testcurrent);
extern void TestInjCur_OvrMicro(uint8_t channel, float testcurrent);
extern void Write_Default_Cal(uint16_t cal_type);


