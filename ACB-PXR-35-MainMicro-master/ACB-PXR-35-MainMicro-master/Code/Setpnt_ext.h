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
//  MODULE NAME:        Setpnt_ext.h
//
//  MECHANICS:          This is the declarations file for the Setpnt.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150617  DAH - File Creation
//   0.60   220928  DAH - Added setpoints declarations (initial code)
//   0.62   221019  DAH - Added SetpActiveSet, Checksum8_16()
//   0.64   221118  DAH - Revised Get_Setpoints() declaration to include the setpoints set
//   0.70   230224   BP - Added Groups 6 thru 10 Defs
//    49    230706  DAH - Added Groups 11 and 12 declarations
//    58    230810  DAH - Added Load_SetpGr0_Gr1() and Load_SetpGr2_LastGr() declarations
//   129    231213  MAG - Added Setpoints13 and changed Verify_Setpoints() to pass in buffer pointer
//   133    231219  DAH - Revised Load_SetpGr2_LastGr() declaration
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
//                    Global Variable Declarations
//------------------------------------------------------------------------------------------------------------
//
extern union STP_GRP0_DEF Setpoints0;
extern union STP_GRP1_DEF Setpoints1;
extern union STP_GRP2_DEF Setpoints2;
extern union STP_GRP3_DEF Setpoints3;
extern union STP_GRP4_DEF Setpoints4;
extern union STP_GRP5_DEF Setpoints5;
extern union STP_GRP6_DEF Setpoints6;
extern union STP_GRP7_DEF Setpoints7;
extern union STP_GRP8_DEF Setpoints8;
extern union STP_GRP9_DEF Setpoints9;
extern union STP_GRP10_DEF Setpoints10;
extern union STP_GRP11_DEF Setpoints11;
extern union STP_GRP12_DEF Setpoints12;
extern union STP_GRP13_DEF Setpoints13;

extern uint32_t SetpointsStat;
extern uint16_t SetpScratchBuf[];
extern uint8_t SetpChkGrp;
extern uint8_t SetpActiveSet;

extern uint16_t * const SETP_GR_DATA_ADDR[];
extern const uint16_t SETP_GR_SIZE[];
extern const uint32_t SETP_GR_FRAM_ADDR[];



//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern uint16_t Checksum8_16(uint8_t *addr, uint16_t length);
extern void Setp_VarInit(void);
extern uint8_t Get_Setpoints(uint8_t set, uint8_t group, uint8_t *good_copy);
extern void Check_Setpoints(uint8_t group);
extern uint8_t Verify_Setpoints(uint8_t group, uint16_t *stp_ptr);
extern void Load_SetpGr0_Gr1(void);
extern uint8_t Load_SetpGr2_LastGr(void);
extern void Stp_to_Default(void);


