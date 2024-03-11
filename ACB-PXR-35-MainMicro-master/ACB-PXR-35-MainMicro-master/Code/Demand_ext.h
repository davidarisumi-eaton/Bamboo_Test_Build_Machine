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
//  MODULE NAME:        Demand_ext.h
//
//  MECHANICS:          This is the declarations file for the Demand.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.23   180504  DAH File Creation
//                          - Dmnd_I_VarInit() renamed to Dmnd_VarInit()
//                          - Renamed Calc_I_Demand() to Calc_Demand()
//   0.32   190726  DAH - Added struct IDmnd, PDmnd to support min and max demands
//   0.51   220304  DAH - Added Calc_5minAverages(), Res5min_Avg, Res5min_MinMax, Sum5min_MinMax to support
//                        the new 5-minute values
//   0.52   220309  DAH - Deleted Sum5min_MinMax as it is not used globally
//   0.72   230320  DAH - Added Dmnd_Type_Window for proc-proc comms
//   149    240131  DAH - Deleted CalcNextAnniversaries() as it is not used globally
//                      - Added Dmnd_Setp_DemandWindow, Dmnd_Setp_DemandInterval,
//                        Dmnd_Setp_DemandLogInterval
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
extern struct ENERGY_DEMAND_STRUCT EngyDmnd[2];
extern struct ENERGY_DEMAND_SAVE_EVENT Dmnd_SaveEvent;
extern uint32_t Next_SI_Time, Next_DW_Time, Next_LA_Time;
extern uint8_t Dmnd_ForceLog;
extern uint8_t Dmnd_Setp_DemandWindow;
extern uint8_t Dmnd_Setp_DemandInterval;
extern uint8_t Dmnd_Setp_DemandLogInterval;
extern struct DEMAND_I_MIN_MAX_STRUCT IDmnd;
extern struct DEMAND_P_MAX_STRUCT PDmnd;

extern struct FIVEMIN_AVGVALS_STRUCT Res5min_Avg;
extern struct FIVEMIN_MINMAXVALS_STRUCT Res5min_MinMax;
extern uint32_t Dmnd_Type_Window;



//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Dmnd_VarInit(void);
extern void Calc_Demand(void);
extern void CalcNextAnniversaries(uint32_t time_secs, uint32_t *next_si, uint32_t *next_dw, uint32_t *next_la);
extern void Calc_5minAverages(void);

