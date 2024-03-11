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
//  MODULE NAME:        Prot_ext.h
//
//  MECHANICS:          This is the declarations file for the Prot.c module
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
//   0.04   150928  DAH - Added Prot_VarInit() and Instantaneous_Prot()
//   0.05   151021  DAH - Added ShortDelay_Prot()
//   0.16   160818  DAH - Added PickupFlags
//   0.45   210514  BP  - Changes and additions from Bert Popovich
//                          - Added LongDelay_Prot() and Long_IEE_IEC_Prot()
//   0.54   220330  DAH - Added LD_Bucket, LD_BucketMax, LD_BucketMaxTS for future thermal overload warning
//   0.55   220420  DAH - Added SD_BucketMax, SD_BucketMaxTS for future short-delay overload logging
//   0.70   230223  BP  - Added all trip and alarm function declarations after Long, Short, and Inst
//   0.72   230320  DAH - Added Break_Config_config_Rating
//                  BP  - Added UpdateThermalMemory()         
//    24    230323  DAH - Added support for displaying long delay time to trip
//                          - Added LD_TimeToTrip
//    44    230323  DAH - Added OverrideTrip() declaration
//    54    230801  DAH - Added trip capacity declarations (OvTripBucket, UvTripBucket, etc.)
//                      - Added MaxPwrOneCycRevW, MaxPwrOneCycRevVar, MaxPwrOneCycW, MaxPwrOneCycVar,
//                        MaxPwrOneCycVA, MinProtPF
//          230802  BP  - Added  Short_Interlock_Out
//                      - Added Override and Breaker Config variables
//    58    230810  DAH - Added Gen_Values() declaration
//    93    231010  BP  - Added declarations for Firmware Simulated test and Hardware Sec Inj Test
//    101   231027  DAH - Added Inst_StartupSampleCnt and SD_StartupSampleCnt declarations
//                      - Added Hardware_SecInj_Test(void) declaration
//    122   231204  BP  - Added declaration for Coil Detection
//    133   231219  DAH - Added Get_Critical_BrkConfig_PXR25(), Get_Critical_BrkConfig_PXR35(), and
//                        Brk_Config_DefaultInit() declarations
//                      - Deleted Write_DefaultCriticalBrkConfig(), Write_DefaultBrkConfig(),
//                        Get_Critical_BrkConfig(), and Save_BrkConfig() declarations
//                      - Deleted Break_Config_Default, SD_StartupSampleCnt, and GF_StartupSampleCnt
//                        declarations
//                      - Added struct EventCurOneCyc and float EventCurOneCycIg declarations
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
extern union WORD_BITS Trip_Flags0;
extern union WORD_BITS Trip_Flags1;

extern union WORD_BITS TripPuFlags;

extern union WORD_BITS Alarm_Flags0;
extern union WORD_BITS Alarm_Flags1;
extern union WORD_BITS Alarm_Flags2;

extern union WORD_BITS Flags0;
extern union WORD_BITS Flags1;
extern union WORD_BITS Flags2;

extern struct FW_SIMULATED_TEST FW_SimulatedTest;
extern struct HW_SEC_INJ_TEST HW_SecInjTest;
extern struct COIL_DETECT CoilDetect;

extern uint16_t PickupFlags;
extern float ScaledCurOneCycSOS_SumF_In;
extern float MaxPwrOneCycRevW, MaxPwrOneCycRevVar, MaxPwrOneCycW, MaxPwrOneCycVar, MaxPwrOneCycVA;
extern float MinProtPF;
extern uint16_t OvTripBucket, UvTripBucket, VuTripBucket, CuTripBucket, RevWTripBucket, RevVarTripBucket;
extern uint16_t PlTripBucket, OfTripBucket, UfTripBucket, OvrWTripBucket, OvrVarTripBucket, OvrVATripBucket;
extern uint16_t PFTripBucket, GFTripBucket, SDTripBucket;

extern float LD_Bucket;
extern float LD_BucketMax;
extern struct INTERNAL_TIME LD_BucketMaxTS;
extern float LD_TimeToTrip;
extern float SD_BucketMax;
extern struct INTERNAL_TIME SD_BucketMaxTS;
extern struct CUR_WITHOUT_G_F EventCurOneCyc;
extern float EventCurOneCycIg;
extern uint16_t PickupFlags;
extern float PhaseLossAlmTmr;
extern float OVR_Threshold;
extern float MCR_Threshold;
extern float MM_Threshold;
extern float DB_Threshold;
extern uint8_t ST_MM_On;
extern uint8_t MM_HiLo_Gain;
extern uint8_t DB_HiLo_Gain;

extern union CRITICAL_BREAKER_CONFIG Break_Config;

extern uint8_t IecSel;                        // *** BP -added this until Factory Options are defined. Used in GF code.
extern uint8_t TU_State_TestUSBMode;          // TU_State_Test.USBMode in the future
extern uint8_t TU_State_TestMode;             // TU_State.TestMode in the future
extern uint8_t TU_State_PowerUp;              // TU_State.PowerUp in the furure
extern uint8_t TU_State_OpenedByComm;         // TU_State.OpenedByComm in the future
extern uint8_t TU_State_ClosedByComm;         // TU_State.ClosedByComm in the future

extern uint8_t Inst_StartupSampleCnt;



//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Prot_VarInit(void);
extern void Instantaneous_Prot(void);
extern void ShortDelay_Prot(void);
extern void LongDelay_Prot(void);
extern void Long_IEE_IEC_Prot(void);

extern void Ground_Fault_Prot(void);

extern void CurUnbalance_Prot(void);
extern void PhaseLoss_Prot(void);
extern void PhaseRotation_Prot(void);

extern void OverVoltage_Prot(void);
extern void UnderVoltage_Prot(void);
extern void OverFreq_Prot(void);
extern void UnderFreq_Prot(void);
extern void VoltUnbalance_Prot(void);

extern void OverRealPower_Prot(void);
extern void OverReactivePower_Prot(void);
extern void OverApparentPower_Prot(void);
void RevActivePower_Prot(void);
extern void RevReactivePower_Prot(void);
extern void PF_Prot(void);
extern void Sneakers_Prot(void);
extern void Temp_Prot(void);

extern void Ground_Fault_PreAlarm(void);
extern void Ground_Fault_Alarm(void);
extern void Sneakers_Alarm(void);
extern void Highload_Alarm(void);
extern void Thermal_Mem_Alarm(void);
extern uint8_t Wrong_Sensor_Alarm_Curr_Condition(void);
extern void WrongSensor_Alarm(void);
extern void BreakerHealth_Alarm(void);
extern void BatteryVolt_Alarm(void);

extern void OverVoltage_Alarm(void);
extern void UnderVoltage_Alarm(void);
extern void OverFreq_Alarm(void);
extern void UnderFreq_Alarm(void);
extern void VoltUnbalance_Alarm(void);
extern void CurUnbalance_Alarm(void);
extern void PhaseLoss_Alarm(void);
extern void PhaseRotation_Alarm(void);
extern void OverRealPower_Alarm(void);
extern void OverReactivePower_Alarm(void);
extern void OverApparentPower_Alarm(void);
extern void RevActivePower_Alarm(void);        // (This is RevPower_Prot() in our other products)
extern void RevReactivePower_Alarm(void);     // (This is not in our other products)
extern void PF_Alarm(void);
extern void Neutral_Alarm(void);
extern void THDCurrent_Alarm(void);
extern void THDVoltage_Alarm(void);
extern void TA_Alarm(void);
extern void ElecMech_Alarm(void);
extern void PKEOverload_Warning(void);

extern void TripFlagsReset(void);
extern void UpdateThermalMemory(void);
extern void Reset_ThermalMemory(uint16_t ThermalMemRst_item);
extern void OverrideTrip(void);
extern void Short_Interlock_Out(void);
extern void Gen_Values(void);

extern void Firmware_Simulated_Test(void);
extern void Hardware_SecInj_Test(void); 
extern void Coil_Detection(void); 

extern uint8_t Get_BrkConfig(void);
extern void Save_Critical_BrkConfig(void);
extern void Write_FrameFRAM_Default_Section(uint16_t address, uint16_t length, uint8_t copies);
extern uint8_t Get_Critical_BrkConfig_PXR25(void);
extern uint8_t Get_Critical_BrkConfig_PXR35(void);
extern void Brk_Config_DefaultInit(void);



