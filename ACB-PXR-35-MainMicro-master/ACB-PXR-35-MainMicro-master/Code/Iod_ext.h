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
//  MODULE NAME:        Iod_ext.h
//
//  MECHANICS:          This is the declarations file for the Iod.c module
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
//   0.04   150928  DAH Deleted AFE_Process_Samples() and related variables (moved to Meter.c)
//   0.05   151021  DAH - Added FRAM_Read() and FRAM_Write()
//   0.06   151110  DAH - Added RTC_Write(), RTC_Read(), SPI2_Manager(), ComputeChksum32(), and Flash_Read()
//                      - Added RTC_Buf[] and struct SPI_2
//                      - Deleted FRAM_Write()
//   0.07   151116  DAH - Modified FRAM_Read() definition to include the device parameter
//                      - Added struct FrameEEPOT
//                      - Added FRAM_Write() for test purposes
//   0.09   160224  DAH - Deleted  FrameEEPOT
//                      - Added struct TH_SENSOR THSensor
//   0.10   160310  DAH - Added SystemFlags, ReadAFECalConstants(), ReadADCHCalConstants(), and
//                        ReadADCLCalConstants()
//   0.13   160512  DAH - Added FRAM_ReadEnergy()
//   0.14   160620  DAH - Re-added support for writing Frame EEPOT settings
//                          - Added struct FrameEEPOT
//                      - Added support for Test Injection
//                          - Added ReadInjCalConstants()
//   0.15   160718  DAH - Deleted TP_LEDCode3 as it is not used
//                      - Added ReadSwitches() to support reading the Clear LEDs pushbutton and the Breaker
//                        Closed switch
//                      - Added WriteCauseOfTripLEDS() and struct COT_AUX to support setting the Cause of
//                        Trip LEDs
//                      - Added BB_FRAM_Read() to support waveform sample storage in the Black Box FRAM
//   0.16   160818  DAH - Added support for Startup Time measurement
//                          - Added StartUpADC, struct StartupTime
//                          - Added StartupTimeCal()
//                      - Added support for thermal memory voltage ADC conversion
//                          - Added ThermalMemoryADC
//   0.18   161115  DAH - Added struct HLTH_I2C
//                      - Added ReadTHSensor()
//                      - Modified FRAM_Write() and FRAM_Read() declarations for 24-bit address for 2Mb FRAM
//   0.22   180420  DAH - Added support for internal real time and time-stamping
//                          - Added struct SysTickTime and functions Get_InternalTime(),
//                            RTC_to_SysTickTime(), and InternalTime_to_RTC()
//                          - Revised RTC_Read() declaration
//                      - Added support for energy and demand logging
//                          - Added Flash_PageWrite(), Flash_EraseSector(), and ProcessTimeAdjustment()
//                          - Added Dmnd_ForceLog and Dmnd_200ms_Per_SubInt_Adj
//   0.23   180504  DAH - Removed Dmnd_ForceLog (moved to Demand_ext.h) and Dmnd_200ms_Per_SubInt_Adj
//   0.24   180517  DAH - Added FRAM_Write(), FRAM_Stat_Write(), EEPOT_Write(), EEPOT_Xfer(),
//                        FRAM_WriteEnergy(), and StoreCalFlash()
//                      - Added FlashCtrl
//                      - Deleted SPI2_Manager()
//                      - Deleted struct SPI_2
//                      - Added struct Alarm_WF_Capture
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Added struct S2NF, struct SPI2Flash, SPI2_NoFlash_Manager(), and
//                            SPI2_Flash_Manager()
//                          - Deleted StoreCalFlash() and FRAM_WriteEnergy()
//                          - Deleted FlashCtrl
//                      - Deleted Flash_EraseSector()
//   0.25   180627  DAH - Deleted FRAM_ReadEnergy()
//   0.27   181001  DAH - Added support for Trip and Strip-Chart waveform captures
//                          - Added structure Trip_WF_Capture and Chart_WF_Capture
//   0.37   200604  DAH - Added SPI2_buf[] to support delayed read in display processor communications
//   0.38   200708  DAH - Added test_setpoints[] to support write setpoints test commands
//   0.42   201202  DAH - Added IntRTC_Read() and Check_IntRTC() to support the internal RTC
//                      - Deleted RTC_Read() and RTC_Write() as they are no longer used
//                      - Added InternalTimeRead() and IntTimeBuf() to support reading the internal time
//                      - Added IntRTC_Update() and RTC_State to adjust the RTC to internal time
//                      - Revised declaration of RTC_to_SysTickTime() to include input pointer
//                      - Deleted InternalTime_to_RTC() as it is no longer used
//                      - RTC_buf[] size reduced from 18 to 12
//                      - Deleted RTC and Internal Time declarations (moved to RealTime_ext.h)
//   0.58   220829  DAH - Frame_FRAM_Read() added to support reading setpoints from the Frame FRAM
//   0.59   220831  DAH - FRAM_ReadEnergy(), FRAM_WriteEnergy(), and FRAM_WriteMinMax() added
//                      - struct SPI2VARS S2NF, SPI2_NoFlash_Manager() declarations deleted
//   0.60   220928  DAH - Revised FRAM_Read() to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//   0.62   221027  DAH - Deleted test_setpoints[] as it is no longer used
//                      - Added Frame_FRAM_Write()
//   0.67   221209  DAH - SPI2_Flash_Manager() renamed to SPI1_Flash_Manager() as the serial Flash is now on
//                        SPI1
//                      - struct SPI2VARS SPI2Flash renamed to struct SPI1VARS SPI1Flash
//   0.69   230220  DAH - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - Chart_WF_Capture deleted
//                  DB      - ExtCapt_FRAM_Read(), ExtCapt_FRAM_Write(), FRAM_Clean() added
//   0.72   230320  BP  - Added HighLoadLED_BlinkCtr and Service_NonCOT_Leds()
//    25    230403  DAH - Added WF_ADDRESS[] (moved from Test.c)
//    54    230802  BP  - Added MM_Status_Update()
//   108    231108  DAH - Corrected SPI2_buf[] declaration (deleted size - it is not needed, and can cause a
//                        problem if it does not match the definition)
//   142    240119  DAH - Added ReadAFECalConstants1(), ReadADCHCalConstants1(), and ReadADCLCalConstants1()
//                        declarations
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
extern uint8_t StatusLED_BlinkCtr, HighLoadLED_BlinkCtr;
extern uint8_t TP_LEDCode;
extern struct SPI1VARS SPI1Flash;
extern uint8_t SPI2_buf[];
extern struct TH_SENSOR THSensor;
extern uint16_t SystemFlags;
extern struct FRAME_EEPOT FrameEEPOT;
extern struct SWITCHES_PUSHBUTTONS COT_AUX;
extern uint16_t StartUpADC, ThermalMemoryADC;
extern struct STARTUP_TIME_VARS StartupTime;
extern struct HLTH_I2C_VARS HLTH_I2C;
extern struct FLASH_INT_REQ Trip_WF_Capture, Alarm_WF_Capture, Ext_WF_Capture;
extern union FLASH_ID_UNION Flash_ID;


//------------------------------------------------------------------------------------------------------------
//                    Global Constant Declarations
//------------------------------------------------------------------------------------------------------------
//
extern const uint16_t WF_ADDRESS[36][4];



//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void IO_VarInit(void);
extern void Service_Status_Led(void);
extern void Service_NonCOT_Leds(void);
extern uint16_t AFE_SPI_Xfer(uint16_t WrData);
extern void FRAM_Read(uint32_t fram_address, uint16_t length, uint16_t *outptr);
extern uint32_t ComputeChksum32(uint32_t *dptr, uint16_t len);
extern void Flash_Read(uint32_t flash_address, uint16_t length, uint16_t *outptr);
extern void Flash_Read_ID(void);
extern void ReadAFECalConstants(void);
extern void ReadADCHCalConstants(void);
extern void ReadADCLCalConstants(void);
extern void ReadInjCalConstants(void);

extern void ReadSwitches(uint8_t first_use);
extern void WriteCauseOfTripLEDS(void);
extern void ReadAFECalConstants1(uint8_t *dest);
extern void ReadADCHCalConstants1(uint8_t *dest);
extern void ReadADCLCalConstants1(uint8_t *dest);

extern void FRAM_Clean(uint16_t device, uint32_t fram_address, uint16_t length);
extern void ExtCapt_FRAM_Read(uint16_t fram_address, uint16_t length, uint16_t *outptr);
extern void ExtCapt_FRAM_Write(uint32_t fram_address, uint16_t length, uint16_t *inptr);

extern void StartupTimeCal(void);
extern void ReadTHSensor(void);
extern void ReadTHSensorConfig(void);
extern uint8_t Flash_PageWrite(uint16_t flash_addr, uint8_t num_words, uint16_t *dataptr);
extern void ProcessTimeAdjustment(struct INTERNAL_TIME *old_time_ptr, struct INTERNAL_TIME *new_time_ptr);
extern void FRAM_Write(uint16_t device, uint32_t fram_address, uint16_t length, uint16_t *inptr);
extern void FRAM_Stat_Write(uint16_t device, uint8_t status);
extern uint8_t EEPOT_Write(void);
extern uint16_t EEPOT_Xfer(uint16_t WrData);
extern void SPI1_Flash_Manager(void);
extern void Frame_FRAM_Read(uint16_t fram_address, uint16_t length, uint8_t *outptr);
extern void FRAM_ReadEnergy(uint8_t device);
extern void FRAM_WriteEnergy(uint8_t device);
extern void FRAM_WriteMinMax(uint32_t fram_address, uint16_t length, uint16_t *inptr);
extern void Frame_FRAM_Write(uint16_t fram_address, uint16_t length, uint8_t *inptr);
extern void MM_Status_Update(void);
extern void RelayManagement(void);
extern void InitRelays(void);

