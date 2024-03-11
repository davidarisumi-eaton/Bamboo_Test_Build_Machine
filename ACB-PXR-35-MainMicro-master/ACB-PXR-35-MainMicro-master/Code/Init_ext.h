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
//  MODULE NAME:        Init_ext.h
//
//  MECHANICS:          This is the declarations file for the Init.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.06   151110  DAH - Deleted Init_FSMC()
//                      - Added Init_RTC()
//                  BP  - Added Init_DMAController2()
//   0.12   160502  DAH - Added Init_TIM1(), Init_TIM3(), and Init_TIM9()
//                      - Added SysClk_120MHz
//   0.15   160718  DAH - Added Init_SPI1(), Init_CAM1_DMA_Streams(), and Init_CAM2_DMA_Streams()
//   0.18   161115  DAH - Init_UART3() deleted (Modbus moved to the display processor)
//                      - Added Init_I2C3()
//   0.22   180420  DAH - Added support for energy and demand logging
//                          - Added InitFlashChip()
//   0.25   180621  DAH - Added Init_TIM4() to support SPI2 and Events handler on 1.5msec interrupts
//   0.29   190122  DAH - Added Init_TIM5() and Init_TIM8() to support coil temperature measurement
//                      - Init_ADC3() revised to support coil temperature measurement
//   0.32   190726  DAH - Added Init_TIM2() to support Modbus messaging
//   0.34   191002  DAH - Revised frequency measurement code for rev 4 boards.  Frequency measurement moved
//                        from Timer 9 to Timers 10 and 11.
//                          - Deleted Init_TIM9()
//                          - Added Init_TIM10() and Init_TIM11()
//   0.37   200604  DAH - Deleted Init_USB() since it is not used
//   0.39   200729  DAH - Added Init_Int_RTC() to support testing the internal RTC
//   0.42   201202  DAH - Renamed Init_Int_RTC() to Init_IntRTC()
//                      - Deleted Init_RTC() as the external RTC is no longer used
//   0.43   210115  DAH - Added Init_UART3() to support 61850 communications with the Display Processor
//   0.49   220131  DAH - Revisions to get DMA working for Modbus transmissions
//                          - Input parameter added to Init_UART6() to determine whether Rx interrupts are
//                            enabled
//   0.59   220831  DAH - Init_SPI1() declaration modified (config added)
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
extern uint8_t SysClkState;
extern uint8_t SysClk_120MHz;

//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Init_FlashConfig(void);
extern void SysClkHandler(void);
extern void Init_GPIO(void);
extern void Init_ADC1(void);
extern void Init_ADC2(void);
extern void Init_ADC3(uint8_t normal);
extern void Init_DAC(void);
extern void Init_SPI1(uint8_t config);
extern void Init_SPI2(uint8_t Device);
extern void Init_SPI3(void);
extern void Init_I2C2(void);
extern void Init_I2C3(void);
extern void Init_UART1(void);
extern void Init_UART2(void);
extern void Init_UART3(void);
extern void Init_UART6(uint8_t rx_int_enabled);
extern void Init_InterruptStruct(void);
extern void AFE_Init(void);
extern void Update_AFE_Sync_Regs(void);
extern void Init_DMAController1(void);
extern void Init_DMAController2(void);
extern void Init_CAM1_DMA_Streams(void);
extern void Init_CAM2_DMA_Streams(void);
extern void Init_TIM1(void);
extern void Init_TIM2(void);
extern void Init_TIM3(void);
extern void Init_TIM4(void);
extern void Init_TIM5(void);
extern void Init_TIM8(void);
extern void Init_TIM10(void);
extern void Init_TIM11(void);
extern void InitFlashChip(void);
extern void Init_IntRTC(void);

