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
//  MODULE NAME:        Intr_ext.h
//
//  MECHANICS:          This is the declarations file for the Intr.c module
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
//   0.04   150928  DAH Added CurHalfCyc, CurHalfCycImax, Sample_State, OneSecAnniv, OneCycAnniv, and
//                      Intr_VarInit()
//   0.05   151021  DAH - Replaced CurHalfCycImax with CurHalfCycSOSmax
//                      - Replaced CurHalfCyc with CurHalfCycSOS_Sum
//                      - Added NewSample and CurOneCycSOSmax
//   0.06   151111  DAH - Added UserWF_Req, UserWF_Ack, UserSamples[][];
//   0.07   151216  DAH - Renamed .sram1 to .sram2 for readability
//   0.08   160122  DAH - Structure CURRENTS_WITH_G renamed to CUR_WITH_G_F
//                      - CurHalfCycSOS_Sum renamed to CurHalfCycSOS_SumF
//   0.09   160224  DAH - Added Getting_AFE_Seed to support basic seeding of the AFE integrator
//                      - Added TA_Timer to support the tripping function
//                      - Added CurHCSOS_Fmax, CurHCSOS_Fmin, CurOCSOS_Fmax, CurOCSOS_Fmin, FreezeMinMax,
//                        ResetMinMaxValues() to support min and max acquisition
//   0.10   160310  DAH - Added SD_ProtOn and TestSamples[][] to support short delay protection control and
//                        waveform capture via the Test Port
//                      - OneSecAnniv renamed to msec200Anniv
//   0.12   160502  DAH - Deleted Sample_State since it is only used in Intr.c
//                      - Deleted Getting_AFE_Seed since it is only used in Meter.c
//   0.15   160718  DAH - Added struct BB_SamplePkt, struct BB_TimeStamp, and BB_Frozen to support storing
//                        waveform samples in the Black Box FRAM
//   0.16   160818  DAH - Added min5Anniv flag to mark 5 minute anniversaries
//   0.21   180301  DAH - Changed UserWF_Req and UserWF_Ack from uint16 to uint8
//                      - Added UserWF_Flags, UserWF_CaptureCode, and UserWF_InService
//                      - Added SampleBuf[] and HarmSampleStartNdx
//   0.23   180504  DAH - Corrected declaration of UserSamples
//   0.27   181001  DAH - Revised User Waveform (UserWF_xx) declarations
//   0.29   190122  DAH - Added coil_temp_samples[] declaration
//   0.31   190506  DAH - Deleted HarmSampleStartNdx (moved to Meter.c)
//                      - Added SampleIndex
//   0.32   190726  DAH - Added SIN_COEFF[] for THD and Displacement PF computation support
//   0.46   210521  DAH - Added Prot_Timer
//   0.59   220831  DAH - Deleted BB_Frozen, BB_SamplePkt, and BB_TimeStamp
//   0.70   230224   BP - Added One Sec Anniv, Half Cyc Anniv, Igres_Protsample, CurOneCycSOS_SumF
//                      - SD_ProtOn renamed to Prot_Enabled
//   0.72   230320  DAH - Deleted HalfCycAnniv
//    36    230428  DAH - Added Trip_WF_OffsetTime, Alarm_WF_OffsetTime, and Ext_WF_OffsetTime to support
//                        waveform capture changes
//    93    231010   BP - Added SampleCounter
//    94    231010  DAH - Deleted UserWF_xx declarations (moved to Meter_ext.h)
//    98    231017  DAH - Added AlarmHoldOffTmr declaration
//    135   231221  DAH - Added SampleBufFilled declaration
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
extern struct CUR_WITH_G_F CurHalfCycSOS_SumF @".sram2";
extern struct CUR_WITH_G_F    CurOneCycSOS_SumF @".sram2";
extern float CurHalfCycSOSmax @".sram2";
extern float CurOneCycSOSmax @".sram2";
extern float NewSample @".sram2";
extern uint8_t msec200Anniv, OneCycAnniv, OneSecAnniv, min5Anniv;
extern uint8_t TA_Timer;

extern struct CUR_WITH_G_F CurHCSOS_Fmax;
extern struct CUR_WITH_G_F CurHCSOS_Fmin;
extern struct CUR_WITH_G_F CurOCSOS_Fmax;
extern struct CUR_WITH_G_F CurOCSOS_Fmin;
extern uint8_t FreezeMinMax;

extern uint8_t Prot_Enabled;
extern uint8_t GF_Enabled;
extern float TestSamples[4][800];

extern struct RAM_SAMPLES SampleBuf[] @ ".sram1";
extern uint16_t SampleIndex @".sram2";

extern uint16_t coil_temp_samples[200];

extern const float SIN_COEFF[];

extern uint8_t Prot_Timer;
extern uint32_t Trip_WF_OffsetTime, Alarm_WF_OffsetTime, Ext_WF_OffsetTime;
extern float Igres_Protsample;

extern uint16_t Sync_Phase;

extern uint32_t SampleCounter;

extern uint8_t AlarmHoldOffTmr;

extern uint16_t Admin_Verified_Tmr;
extern uint16_t User_Verified_Tmr;
extern uint16_t Pswd_Rejection_Tmr;
extern uint8_t Pswd_attempt;
extern uint8_t SampleBufFilled;

//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Intr_VarInit(void);
extern void ResetMinMaxValues(void);

