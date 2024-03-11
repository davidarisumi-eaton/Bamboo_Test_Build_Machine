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
//  MODULE NAME:        RealTime_ext.h
//
//  MECHANICS:          This is the declarations file for the RealTime.c module
//
//  TARGET HARDWARE:    PXR35 Rev 4 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.42   201202  DAH File Creation.  Code moved from Iod_ext.h
//                      - Added IntRTC_Read() and Check_IntRTC() to support the internal RTC
//                      - Deleted RTC_Read() and RTC_Write() as they are no longer used
//                      - Added InternalTimeRead() and IntTimeBuf() to support reading the internal time
//                      - Added IntRTC_Update() and RTC_State to adjust the RTC to internal time
//                      - Revised declaration of RTC_to_SysTickTime() to include input pointer
//                      - Deleted InternalTime_to_RTC() as it is no longer used
//                      - RTC_buf[] size reduced from 18 to 12
//   0.51   220304  DAH - Added InternalTimeToMBTime()
//   0.58   220829  DAH - Added RTC_InitState
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
extern uint8_t RTC_buf[];
extern uint8_t IntTimeBuf[];
extern uint8_t RTC_State;
extern uint8_t RTC_InitState;
extern struct SYSTICK_TIME SysTickTime;




//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void RT_VarInit(void);
extern void Get_InternalTime(struct INTERNAL_TIME *ts_ptr);
extern void RTC_to_SysTickTime(struct SYSTICK_TIME *systck_ptr);
extern void InternalTimeRead(void);
extern uint8_t Check_IntRTC(void);
extern uint8_t IntRTC_Read(void);
extern void IntRTC_Update(void);
extern uint64_t InternalTimeToMBTime(struct INTERNAL_TIME *inttime_ptr);


