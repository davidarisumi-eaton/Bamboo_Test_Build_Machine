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
//  AUTHORS:            Xavier Pacheco         (809)748-8609
//                      Eaton Corporation
//                      1000 Cherrington Parkway
//                      Moon Twp, PA  15108-4312
//                      (412)893-3300
//
//------------------------------------------------------------------------------------------------------------
//  PRODUCT:            PXR35       Trip unit for air circuit and molded-case circuit breakers
//
//  FIRMWARE DRAWING:   ????????    This drawing combines the unprogrammed STM32F407IGT7 with the code's
//                                  flash programming file to produce an "assembly group" that is the
//                                  programmed device.
//
//  PROCESSOR:          ST Micro STM32F407IGT7
//
//  COMPILER:           IAR C/C++ Compiler for ARM - v8.50.4.26143
//                      IAR Embedded Workbench from IAR Systems
//
//  MODULE NAME:        can_driver_ext.h
//
//  MECHANICS:          This module contains the external declarations for the can_driver.c module
//
//  TARGET HARDWARE:    PXR35 Rev ? and later boards
//
//------------------------------------------------------------------------------------------------------------



#ifndef CAN_DRIVER_EXT_H
#define CAN_DRIVER_EXT_H

extern void checkCanStatus(void);
extern void increment_TME_Counters(void);

#endif /*CAN_DRIVER_EXT_H*/

/*********************************** end of can_driver_ext.h **************************************/
