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
//  MODULE NAME:        Events_ext.h
//
//  MECHANICS:          This is the declarations file for the Events.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150617  DAH File Creation
//   0.22   180420  DAH Added declarations to support the Events Manager
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Added Energy_Log_Add
//                      - Added support for Alarm waveform capture events
//                          - Added InsertNewEvent()
//   0.37   200604  DAH - Added FRAM_EV_ADD[ ], MAX_NUM_EVENTS[ ], and FRAM_EV_INFO[ ]
//                      - Added GetEVInfo()
//                      - Revised Demand and Energy logging variables to be compatible with the other event
//                        variables
//                          - Replaced Energy_Log_Add with struct EV_Dmnd
//   0.59   220831  DAH - Added IntEventxxxx variables
//   0.61   221001  DB  - Added EventSummaryWrite() declaration
//   0.63   221111  DB  - Added EventLookUp() declaration
//   0.64   221118  DAH - Added ClearLogFRAM() declaration
//   0.65   221201  DB  - EID parameter in EventLookUp() definition changed from uint16_t to uint32_t
//   0.69   230220  DAH - Revised GetEVInfo() declaration
//                      - Deleted FRAM_EV_ADD[], MAX_NUM_EVENTS[], FRAM_EV_INFO[], and EventState
//                        declarations as they are only used in Events.c
//                  DB  - Added support for extended captures and disturbance captures
//   24     230323  DAH - Revised extended captures to lock out new events after a capture has been made
//                        until the extended capture has been completed (on an individual event basis)
//                          - ExtCap_Flag renamed to ExtCap_ReqFlag
//   25     230403  DAH - Added struct EV_SUM EV_Sum declaration
//                      - Added EventSummaryRead() and EventGetWfEIDs() declarations
//                      - Added RAM_EV_ADD[], FRAM_EV_ADD[], FRAM_EV_INFO[], and EV_SIZE[] declarations to
//                        support proc-proc waveform reads
//   27     230404  DAH - Added EventDisturbanceRead() declaration
//   28     230406  DAH - Added Dist_GOOSE flag declaration
//   37     230516  DAH - Corrected ExtCap_ReqFlag declaration from word to byte
//                      - Added struct DIST_VALUES sListOfDist[]
//                      - Added ExtCap_AckFlag declaration
//   40     230531  DAH - Added EventSnapshotSummaryRead() to support reading the snapshot summaries
//                      - Added EventSnapshotRead() to support reading the snapshots
//                      - Revised EventDisturbanceRead() declaration (added total_logs)
//   44     230623  DAH - EventGetWfEIDs() declaration was changed
//   46     230703  DAH - Added MAX_NUM_EVENTS[] declaration
//   54     230801  DAH - Added support for SDPU and GF disturbance captures
//                          - Dist_Flag_SD, Dist_Flag_GF, Dist_Flag_Cancel_SD, Dist_Flag_Cancel_GF
//                            declarations added
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
extern struct NEWEVENTFIFO NewEventFIFO[];
extern uint8_t NewEventInNdx;
extern uint32_t EventMasterEID;
extern struct EV_DMND EV_Dmnd;
extern struct NEWEVENTFIFO IntEventBuf[];
extern uint8_t NewEventInNdx;
extern uint8_t IntEventInNdx, IntEventOutNdx, IntEventBufFull;
extern uint8_t Dist_GOOSE;

extern struct EV_SUM EV_Sum;

extern struct EXTCAP_SNAPSHOT_METER ExtCapSnapshotMeteredValues;

extern uint32_t Dist_Flag;
extern uint8_t Dist_Flag_SD;
extern uint8_t Dist_Flag_GF;
extern uint32_t Dist_Flag_Cancel;
extern uint8_t Dist_Flag_Cancel_SD;
extern uint8_t Dist_Flag_Cancel_GF;
extern struct DIST_VALUES sListOfDist[];

extern struct EXCTAP_STATE sExtCapState;
extern uint8_t ExtCap_ReqFlag;
extern uint8_t ExtCap_AckFlag;
extern uint8_t Goose_Capture_Code;



//------------------------------------------------------------------------------------------------------------
//                    Global Constant Declarations
//------------------------------------------------------------------------------------------------------------
//
extern struct EV_ADD* const RAM_EV_ADD[];
extern const uint32_t FRAM_EV_ADD[];
extern const uint32_t FRAM_EV_INFO[];
extern const uint32_t EV_SIZE[];
extern const uint16_t MAX_NUM_EVENTS[];


//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Event_VarInit(void);
extern void EventManager(void);
extern void ClearEvents(void);
extern uint32_t InsertNewEvent(uint8_t EventType);
extern uint8_t GetEVInfo(uint8_t ev_type);
extern void EventSummaryWrite(void);
extern int16_t EventLookUp(uint32_t EIDLookUp, uint8_t EventType);
extern void ClearLogFRAM(uint8_t EventType);
extern void ExtendedCapture(uint8_t TypeOfRecord);
extern void DisturbanceCapture(void);
extern void EventSummaryRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num);
extern void EventDisturbanceRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs);
extern uint8_t EventGetWfEIDs(uint8_t evtype, uint32_t srch_eid, uint8_t *match_ndx);
extern void EventSnapshotSummaryRead(uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs, uint16_t bid);
extern void EventSnapshotRead(uint8_t *bufinfoptr, uint16_t bid);

