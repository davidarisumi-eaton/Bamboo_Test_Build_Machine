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
//  MODULE NAME:        DispComm_ext.h
//
//  MECHANICS:          This is the declarations file for the Display.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150617  DAH File Creation
//   0.16   160818  DAH - Added support for Display communications
//                          - Added struct Disp
//                          - Added Disp_VarInit() and Disp_Tx()
//   0.19   170223  DAH - Added Disp_Rx()
//   0.28   181030  DAH - Added SendCurrents to initiate sending the currents to the display processor
//   0.32   190726  DAH - Added CalcCRC() for use with the Modbus message handlers
//   0.36   200210  DAH - Renamed file from Display_ext.h to DispComm_ext.h
//                      - Disp_Rx() renamed to DispComm_Rx()
//                      - Disp_Tx() renamed to DispComm_Tx()
//                      - Disp_VarInit() renamed to DispComm_VarInit()
//                      - struct DISPLAYVARS Disp renamed to struct DISPCOMMVARS DPComm
//                      - Deleted SendCurrents, as RTD buffers are only sent when requested by the Display
//                        Processor
//                      - Deleted CalcCRC() as it was moved to Modbus.c
//                      - Added CRC_TABLE1[] for Modbus.c
//   0.37   200604  DAH - Added AssembleAck1()
//   0.43   210115  DAH - Added struct DISPCOMM61850VARS DPComm61850, DispComm61850_Tx() and
//                        DispComm61850_Rx() to support 61850 communications with the Display Processor
//    25    230403  KT  - Added Pub_Goose_Capture_Pkt declaration
//    69    230828  DAH - Added struct INTERNAL_TIME IntSyncTime declaration
//    82    230928  DAH - Added DispProc_FW_Rev, DispProc_FW_Ver, DispProc_FW_Build declarations
//   133    231219  DAH - Added Reset_to_PLL_Count and maxlooptime declarations
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
extern struct DISPCOMMVARS DPComm;
extern struct DISPCOMM61850VARS DPComm61850;
extern struct INTERNAL_TIME IntSyncTime;

extern const uint16_t CRC_TABLE1[];
extern struct PUB_GOOSE_CBS PXR35_CB_Publish_Data;
extern uint32_t ResetMinMaxFlags;

extern uint8_t DispProc_FW_Rev;
extern uint8_t DispProc_FW_Ver;
extern uint16_t DispProc_FW_Build;

extern uint8_t RelayFlagStp;
extern struct TESTINJ_VARS TestInjVars;

extern uint8_t Reset_to_PLL_Count;
extern uint32_t maxlooptime;




//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void DispComm_VarInit(void);
extern void DispComm_Tx(void);
extern void DispComm_Rx(void);
extern void AssembleAck1(uint8_t ack_val);
extern void DispComm61850_Tx(void);
extern void DispComm61850_Rx(void);

