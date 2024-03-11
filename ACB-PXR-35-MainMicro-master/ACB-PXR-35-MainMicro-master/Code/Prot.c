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
//  MODULE NAME:        Prot.c
//
//  MECHANICS:          Program module containing the functions for protection
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.04   150928  DAH Added Prot_VarInit() and Instantaneous_Prot()
//   0.05   151021  DAH - Modified Instantaneous_Prot() to use half-cycle sums of squares instead of the RMS
//                        currents
//                          - CurHalfCyc replaced with CurHalfCycSOS_Sum
//                          - CurHalfCycInstsav replaced with CurHalfCycInstSOSsav
//                          - CurHalfCycImax replaced with CurHalfCycSOSmax
//                      - Added short delay protection
//                          - Added ShortDelay_Prot()
//                          - Added SD_FLAT, SD_I2T, SD_MIN_PASSES_FOR_TRIP
//                          - Added SD_HalfCycPickup, SD_OneCycPickup, SD_Tally, SD_TripThresholdI2t,
//                            SD_Passes, SD_TripThresholdFlat, SD_State, SD_Style
//                          - Modified Prot_VarInit() to initialize SD_HalfCycPickup and SD_Style
//   0.07   151216  DAH - Renamed .sram1 to .sram2 for readability
//   0.08   160122  DAH - In Instantaneous_Prot(), CurHalfCycSOS_Sum renamed to CurHalfCycSOS_SumF
//                      - Structure CURRENTS_WITHOUT_G renamed to CUR_WITHOUT_G_F
//   0.09   160224  DAH - For test purposes, added code to support Short Delay Flat Trip at 10000A (2.5pu in
//                        an R-Frame) and 50msec
//                          - In Prot_VarInit(), changed pickup level for Instantaneous (Inst_Pickup) to
//                            20000A, changed pickup level for Short Delay (SD_HalfCycPickup,
//                            SD_OneCycPickup) to 10000A, changed trip style (SD_Style) to Flat, and added
//                            code to initialize flat trip time (SD_TripThresholdFlat) to 45msec and I2t
//                            trip time (SD_TripThresholdI2t) to 800msec
//                          - In ShortDelay_Prot(), added code to turn on the trip indicator LEDs and to
//                            issue a trip signal if the trip threshold is reached
//   0.10   160310  DAH - Added code to support short delay protection control via the Test Port
//                          - Modified Prot_VarInit() to set to Flat 150msec at 600A pickup.  This provides
//                            nine cycles of capture
//                          - Modified ShortDelay_Prot() to set SD_ProtOn to False when tripping occurs.
//                            This stops the waveform capture after 150msec
//   0.16   160818  DAH - Added PickupFlags and modified Instantaneous_Prot() and ShortDelay_Prot() to
//                        update the flags
//                          - Added include of Prot_def.h
//   0.18   161115  DAH - Revised cause of trip LEDs interface code in ShortDelay_Prot() to match the rev 3
//                        board
//   0.19   170223  DAH - In ShortDelay_Prot(), replaced code to turn on the SD cause of trip LED with a
//                        call to WriteCauseOfTripLEDS()
//   0.22   180420  DAH - Swapped the order of "include "Iod_def.h" " and "#include "Meter_def.h" " because
//                        Meter_def.h uses INTERNAL_TIME, which is defined in Iod_def.h
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added include of RealTime_def.h
//   0.45   210514  BP  - Changes and additions from Bert Popovich
//                          - Added standard (iT) long delay protection functions
//                              - LDxxx variables and constants added
//                              - Prot_VarInit() revised to initialize new variables and "hard-code"
//                                protection settings, and call to Gen_Values() added to initialize
//                                protection variables based on the settings
//                              - Gen_Values(), LongDelay_Prot() and Long_IEE_IEC_Prot() added
//                          - Added settings for Short Delay and Instantaneous protection.  Revised code to
//                            compute internal protection values from the settings (were hard-coded)
//                          - Added ZSI to short-delay protection
//                      - Added initial trip waveform capture to instantaneous protection for arc flash demo
//                        and fixed up instantaneous protection function
//                          - Includes of Events_def.h and FRAM_Flash_def.h added
//                          - Breaker rating set to 200A and pickup set to 3x for initial testing
//                          - Revised Instantaneous_Prot() to turn on instantaneous cause of trip led,
//                            activate TA, and set waveform capture request flag when tripping
//   0.46   210521  DAH - Revised Instantaneous_Prot() to turn off protection and start the protection
//                        timer.  This disables protection for about 250msec after the TA is fired so that
//                        the TA is not fired continuously during the arc event
//   0.48   210715  DAH - In Instantaneous_Prot(), added code to reset state when trip is issued
//   0.54   220330  DAH - Added LD_Bucket, LD_BucketMax, LD_BucketMaxTS for future thermal overload warning
//                          - Prot_VarInit() revised
//   0.55   220420  DAH - Added SD_BucketMax, SD_BucketMaxTS for future short-delay overload logging
//                          - Prot_VarInit() revised
//   0.59   220831  DAH - Revised Instantaneous_Prot() to place the Trip Event into the interrupt queue
//   0.60   220928  DAH - In Instantaneous_Prot(), added test code to force an instantaneous trip.  Code is
//                        presently commented out
//   0.69   230220  DAH - In Instantaneous_Prot(), NewEventFIFO[].Type renamed to NewEventFIFO[].Code
//
//   0.70   230223  BP  - Added all trip and alarm functions after Long, Short, and Inst.
//   0.72   230320  DAH - Moved Break_Config_config_Rating from local to global section
//                      - In Prot_VarInit(), commented out code to initialize Break_Config_config_Rating
//                        This was moved to the initialization code in main
//                      - In Gen_Values(), added code to set the rating setpoints to the frame rating read
//                        from the configuration
//                      - In Gen_Values(), added code to initialize Dmnd_Type_Window
//                  BP  - Corrected minor bug in Highload_Alarm()
//                      - Added UpdateThermalMemory() and added Thermal Memory code
//                      - Added code to turn on Trip LED for L,S,I,G trips
//                      - Moved LD Bucket calculation
//                      - Clear Trip and Alarm flags in Prot_VarInit()
//   24     230323  DAH - Modified Highload_Alarm() to handle delay timer setpoint in increments of 0.1s
//                      - Modified Highload_Alarm() to support alarm and extended capture events
//   25     230403  DAH - Fixed bug inserting alarm event in Highload_Alarm()
//   27     230404  DAH - Added disturbance capture initiation to Highload_Alarm()
//   28     230406  DAH - Added GOOSE Capture request to Highload_Alarm()
//   29     230406  DAH - Added code to initialize TU_State_TestUSBMode
//   36     230428  DAH - Added Trip_WF_OffsetTime, Alarm_WF_OffsetTime, and Ext_WF_OffsetTime to
//                        Gen_Values() to support computing the time of the first sample in waveform
//                        captures
//   37     230516  DAH - Revised Highload_Alarm()
//                          - added High Load 1 event support
//                          - added code to save the EID of the Alarm event in case it initiates a
//                            disturbance capture
//                          - cleaned up GOOSE Global Capture command handling
//                          - added support for test events to Highload_Alarm()
//                      - Added disturbance captures and GOOSE global captures to LongDelay_Prot()
//                      - Added event support to the remaining alarm and protection functions
//   38     230518  DAH - Revised OverVoltage_Prot() and UnderVoltage_Prot() to insert an exit event if
//                        trip pickup is exited without a disturbance capture
//                      - Fixed bug in OverVoltage_Alarm() and UnderVoltage_Alarm() to ensure GOOSE "end
//                        capture" message is transmitted whenever alarm condition is exited
//   44     230323  DAH - Added OverrideTrip()
//   54     230801  DAH - Added support for short delay disturbance captures
//                          - ShortDelay_Prot() modified to initiate summary and disturbance events when
//                            entering pickup
//                      - Added Alarm event logging to Ground_Fault_PreAlarm()
//                      - Added Alarm and disturbance captures to Ground_Fault_Alarm() and
//                        Ground_Fault_Prot()
//                      - Corrected event capture in LongDelay_Prot().  Changed capture from interrupt queue
//                        to normal queue
//                      - Corrected how EID is saved for disturbance captures in Highload_Alarm()
//                      - In numerous subroutines, revised code setting sListOfDist[].EIDofStartEvnt to the
//                        EID of the inserted event (InsertNewEvent() now returns the EID)
//                      - Revised LongDelay_Prot() to clamp the tally register to the trip threshold so the
//                        thermal capacity never exceeds 100%.  The concern is if a large current spike
//                        occurs, the thermal memory will be very high and the unit will keep tripping on
//                        when the breaker is closed.
//                      - Fixed bug in OverVoltage_Prot() and UnderVoltage_Prot().  Added condition when
//                        inserting an exit event if trip pickup is exited without a disturbance capture.
//                        Without the conditional check, we were getting continuous exit pickup events
//                      - Changed trip and alarm timer (OvTripTmr, UvTripTmr, etc.) definitions from float
//                        to uint16_t
//                      - Revised OverVoltage_Prot(), UnderVoltage_Prot(), VoltUnbalance_Prot(),
//                        CurUnbalance_Prot(), RevActivePower_Prot(), RevReactive_Power_Prot(),
//                        PhaseLoss_Prot(), OverFreq_Prot(), UnderFreq_Prot(), OverRealPower_Prot(),
//                        OverReactivePower_Prot(), OverApparentPower_Prot() to add trip capacity
//                        computation for disturbance captures.  Values are in xxxxTripBucket variables
//                      - Revised RevActivePower_Alarm(), RevActivePower_Prot(), RevReactivePower_Alarm(),
//                        RevReactivePower_Prot() to compute the max reverse power for disturbance captures
//                      - Revised OverRealPower_Prot(), OverReactivePower_Prot(), OverRealPower_Alarm(),
//                        OverReactivePower_Alarm(), OverApparentPower_Prot(), OverApparentPower_Alarm() to
//                        compute the max reverse power for disturbance captures
//                      - Revised PF_Prot() to use the active power factor readings rather than the captured
//                        min readings
//          230802  BP  - Added Short_Interlock_Out for ZSI update
//                      - Added Overide and Breaker Config variables
//                      - Added 50 Hz code
//                      - Added threshold calcualtions for Override micro protection
//                      - Added GF trip time compensation
//                      - Fixed UV and OV pickup in trip and alarm functions. Need to accout for setpoints being x10.
//                      - Changed PF_Trip() and PF_Alarm() to account for a PF of 0 (NAN).
//   58     230810  DAH - Moved Gen_Values() from local to global
//   93     231010  BP  - Added code for Firmware Simulated Test and Hardware Sec Inj Test
//   98     231017  DAH - Commented out code in Reset_ProtRelated_Alarm() so that the subroutine does
//                        nothing.  The alarm flags are cleared in the main loop when the alarm holdoff
//                        timer is active.  The flags are also cleared when the alarm condition is no longer
//                        present.  This ensures only a single alarm is generated when the alarm is followed
//                        by a trip.  Note, this matches the NRX1150 operation (but not the PXR25 operation).
//                      - Added code to initialize the alarm holdoff timer, AlarmHoldOffTmr, to 2sec in all
//                        protection functions
//   99     231019  BP  - Added No Trip for Secondary Injection and added code to Sec Inj results
//   101    231027  DAH - Added startup time compensation to Instantaneous_Prot() and ShortDelay_Prot()
//   104    231102  BP  - Added TripFlagsReset() before setting Alarm flags high
//                      - Added startup time compensation to Ground_Fault_Prot()
//                      - Added TimeToTripFlag and ThermCapacityFlg
//   108    231108  DAH - Minor revisions to Get_BrkConfig(), Save_BrkConfig(), Save_Critical_BrkConfig(),
//                        and Write_FrameFRAM_Default_Section() to eliminate compiler warnings
//   110    231114  DAH - Revised OverVoltage_Prot(), UnderVoltage_Prot(), VoltUnbalance_Prot(),
//                        CurUnbalance_Prot(), OverFreq_Prot(), UnderFreq_Prot(), PhaseLoss_Prot(),
//                        OverRealPower_Prot(), OverReactivePower_Prot(), OverApparentPower_Prot(),
//                        PF_Prot(), RevActivePower_Prot(), RevReactivePower_Prot() to check whether
//                        protection is enabled before executing
//                      - Revised OverVoltage_Alarm(), UnderVoltage_Alarm(), VoltUnbalance_Alarm(),
//                        CurUnbalance_Alarm(), OverFreq_Alarm(), UnderFreq_Alarm(), PhaseLoss_Alarm(),
//                        PhaseRotation_Alarm(), OverRealPower_Alarm(), OverReactivePower_Alarm(),
//                        OverApparentPower_Alarm(), PF_Alarm(), RevActivePower_Alarm(),
//                        RevReactivePower_Alarm(), Ground_Fault_PreAlarm(), Sneakers_Alarm() to check
//                        whether the alarm is enabled and not held off before executing
//                      - Corrected check of Trip_Prot_Action setpoint in OverFreq_Prot() and
//                        UnderFreq_Prot()
//   111    231117  BP  - Added code for calculation of Ir_02 for IEC Long Delay in Gen_Values() 
//   113    231128  BP  - Added code for Ground Fault Digitization and 1200A limit
//                      - Added Sec Inj code for Long Delay IEEE and IEC slopes
//                      - Added Time to Trip for Long Delay IEEE and IEC slopes
//   122    231204  BP  - Added Coil Detection code
//                      - Fixed bug that prevented the reading of Neutral current for Rogowski input
//                        by linking the Group 0 Neutral Sensor setpoint to the port pin that controls
//                        the measurement circuitry.  Added this code to Gen_Values() so that it gets 
//                        executed no matter where the setpoints get changed from.
//   125    231212  BP  - Fixed Reverse Active Power protection bug
//                      - Changed Flat GF Trip Threshold multliplier by 10% for settings of 0.4 to 1.0
//                      - Added voltage and current conditions for running UnderPF protection
//                      - Added Setpoints5.stp.ExtProtEnable as a condition for running Motor Protection
//                        functions
//   129    231213  MAG - Consolidated handling of Group0/Group1 read-only setpoints into GetSetpoints()
//                      - Updates due to split of Setpoints10 to Setpoints10 and Setpoints13
//   133    231219  DAH - In Gen_Values() reduced SD and GF 50ms - 90ms trip times multiplier from 0.7 to
//                        0.5 to ensure trip times are met at minimum currents (200A Frame, 0.4 dial-down)
//                      - Fixed bugs in PF_Prot() and PF_Alarm().  Need to divide PU settings by 100
//                      - Revised breaker configuration handling
//                          - Non-critical breaker config values are not kept in RAM.  They are read and
//                            written to FRAM as required
//                          - A new section was added to Frame FRAM to support a larger amount of critical
//                            breaker config values for the PXR35.  The process for retrieving the critical
//                            breaker config values was changed to be backward-compatible with PXR25 Frame
//                            modules and also support the new Frame modules
//                          - Deleted Write_DefaultCriticalBrkConfig(), Write_DefaultBrkConfig(),
//                            Get_Critical_BrkConfig(), and Save_BrkConfig() as they are no longer used
//                          - Deleted Break_Config_Default as it is no longer used
//                          - Added Get_Critical_BrkConfig_PXR25() and Get_Critical_BrkConfig_PXR35()
//                          - Moved Brk_Config_DefaultInit() to the global section and revised it to handle
//                            both critical and non-critical breaker config values
//                          - Changed union Break_Config from hpolding all config values to just holding the
//                            critical config values (as defined by union CRITICAL_BREAKER_CONFIG)
//                          - Fixed bugs retrieving existing breaker config data from Frame FRAM.  Spacing
//                            bytes between copies was not accounted for
//                          - Revised Write_FrameFRAM_Default_Section() to account for spacing bytes between
//                            copies in Frame FRAM
//                      - Removed startup time compensation from Short Delay and Ground Fault protection.
//                        If the startup time is greater than 8.3msec (one-half cycle), it will not work
//                        properly, because the one-cycle currents have not been computed when they are
//                        checked in the protection routine.  Instantaneous protection is ok, because only
//                        half-cycle currents are used.
//                          - ShortDelay_Prot() and Ground_Fault_Prot() revised
//                      - Added code to compute and save the RMS currents if tripping in Instantaneous,
//                        Short Delay, and Ground Fault protection.  These currents are used for the trip
//                        snapshot logs, in case Calc_Prot_Current() isn't called prior to the event log.
//                          - Added struct CUR_WITHOUT_G_F EventCurOneCyc and float EventCurOneCycIg
//                          - Instantaneous_Prot(), ShortDelay_Prot(), Ground_Fault_Prot() revised
//                      
//   139    240104  BP  - Fixed GF ZSI
//                      - Fixed Under PF bug
//   141    240115  BP  - Set/Clear T_Forbid bit based on current levels for Sec Inj and Coil Detection
//   145    240124  BP  - Changed Over/UnderFreq Prot and Alarm routines to divide the pickup setpoint by 1000.
//                      - Fixed copy/paste bugs in UF Alarm routine
//                      - Voltage must be at least 10% of System Voltage to run UV, UF, and OF Alarm code.
//                      - Fixed scaling on Voltage Unbalance by remove the multiply by 100.
//   148    240131  BP  - Fixed UF Alarm bug
//                      - Changed scaling for Short Delay I2t at 0.05 setting
//   150    240202  DAH - Eliminated code that clears the trip flag in PhaseRotation_Prot().  Flag shouldn't
//                        be cleared, unless the reset button is pressed.
//                      - Added code to insert an event when a trip occurs
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
//                   Definitions
//------------------------------------------------------------------------------------------------------------
//
//      Global Definitions from external files...
//
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <math.h>
#include "Prot_def.h"
#include "Intr_def.h"
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Meter_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "DispComm_def.h"
#include "Test_def.h"
#include "Flags_def.h"

//
//      Local Definitions used in this module...
//
#define SD_FLAT                 0
#define SD_I2T                  1
#define SD_MIN_PASSES_FOR_TRIP  240       // 240 samples = 3 cycles

#define LD_I05T                 0         // These are from Randy's spreadsheet (08-02-22)
#define LD_I1T                  1
#define LD_I2T                  2
#define LD_I4T                  3
#define LD_IEEE_MI              4
#define LD_IEEE_VI              5
#define LD_IEEE_EI              6
#define LD_IEC_A                7
#define LD_IEC_B                8
#define LD_IEC_C                9

#define LD_MIN_PASSES_FOR_TRIP  15        // 15 cycles = 0.25s

#define GF_FLAT                 0
#define GF_I2T                  1
#define GF_MIN_PASSES_FOR_TRIP  480       // 480 samples = 0.1s


//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Intr_ext.h"
#include "Iod_ext.h"
#include "Events_ext.h"
#include "RealTime_ext.h"
#include "Setpnt_ext.h"
#include "Meter_ext.h"
#include "Intr_ext.h"
#include "Demand_ext.h"
#include "DispComm_ext.h"
#include "Ovrcom_ext.h"
#include "Test_ext.h"

//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Prot_VarInit(void);
void Instantaneous_Prot(void);
void ShortDelay_Prot(void);
void LongDelay_Prot(void);
void Long_IEE_IEC_Prot(void);
void Ground_Fault_Prot(void);

void CurUnbalance_Prot(void);
void PhaseLoss_Prot(void);
void PhaseRotation_Prot(void);

void OverVoltage_Prot(void);
void UnderVoltage_Prot(void);
void OverFreq_Prot(void);
void UnderFreq_Prot(void);
void VoltUnbalance_Prot(void);

void OverRealPower_Prot(void);
void OverReactivePower_Prot(void);
void OverApparentPower_Prot(void);
void RevActivePower_Prot(void);
void RevReactivePower_Prot(void);
void PF_Prot(void);
void Sneakers_Prot(void);
void Temp_Prot(void);

void Ground_Fault_PreAlarm(void);
void Ground_Fault_Alarm(void);
void Sneakers_Alarm(void);
void Highload_Alarm(void);
void Thermal_Mem_Alarm(void);
uint8_t Wrong_Sensor_Alarm_Curr_Condition(void);
void WrongSensor_Alarm(void);
void BreakerHealth_Alarm(void);
void BatteryVolt_Alarm(void);

void OverVoltage_Alarm(void);
void UnderVoltage_Alarm(void);
void OverFreq_Alarm(void);
void UnderFreq_Alarm(void);
void VoltUnbalance_Alarm(void);
void CurUnbalance_Alarm(void);
void PhaseLoss_Alarm(void);
void PhaseRotation_Alarm(void);
void OverRealPower_Alarm(void);
void OverReactivePower_Alarm(void);
void OverApparentPower_Alarm(void);
void RevActivePower_Alarm(void);        // (This is RevPower_Prot() in our other products)
void RevReactivePower_Alarm(void);     // (This is not in our other products)
void PF_Alarm(void);
void Neutral_Alarm(void);
void THDCurrent_Alarm(void);
void THDVoltage_Alarm(void);
void TA_Alarm(void);
void ElecMech_Alarm(void);
void PKEOverload_Warning(void);

void TripFlagsReset(void);
void Reset_ProtRelated_Alarm(void);
void UpdateThermalMemory(void);
void Reset_ThermalMemory(uint16_t ThermalMemRst_item);

void OverrideTrip(void);
void Short_Interlock_Out(void);
void Gen_Values(void);

void Firmware_Simulated_Test(void);
void FW_SimulatedTestResults(void);
void Hardware_SecInj_Test(void);
void HW_SecInjTestResults(void);
void OvrMicro_SecInjTestResults(void);
void Coil_Detection(void);

uint8_t Get_BrkConfig(void);
void Save_Critical_BrkConfig(void);
void Write_FrameFRAM_Default_Section(uint16_t address, uint16_t length, uint8_t copies);
uint8_t Get_Critical_BrkConfig_PXR25(void);
uint8_t Get_Critical_BrkConfig_PXR35(void);
void Brk_Config_DefaultInit(void);


//      Local Function Prototypes (These functions are called only within this module)
//



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
union WORD_BITS Trip_Flags0;
union WORD_BITS Trip_Flags1;

union WORD_BITS Alarm_Flags0;
union WORD_BITS Alarm_Flags1;
union WORD_BITS Alarm_Flags2;

union WORD_BITS Flags0;
union WORD_BITS Flags1;
union WORD_BITS Flags2;


float ScaledCurOneCycSOS_SumF_In;
float MaxPwrOneCycRevW, MaxPwrOneCycRevVar, MaxPwrOneCycW, MaxPwrOneCycVar, MaxPwrOneCycVA, MinProtPF;
uint16_t OvTripBucket, UvTripBucket, VuTripBucket, CuTripBucket, RevWTripBucket, RevVarTripBucket;
uint16_t PlTripBucket, OfTripBucket, UfTripBucket, OvrWTripBucket, OvrVarTripBucket, OvrVATripBucket;
uint16_t PFTripBucket, GFTripBucket, SDTripBucket;

uint8_t Inst_StartupSampleCnt;

//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
// Variables located in .sram2 are only used during the DMA1_Stream0_IRQHandler() interrupt.  There is no
//   conflict with the associated DMA stream, since it is not be enabled during this interrupt
struct CUR_WITHOUT_G_F CurHalfCycInstSOSsav @".sram2";
float Inst_Pickup;
float SD_HalfCycPickup, SD_OneCycPickup, SD_OneCyc_8x;
float SD_InterlockPickup;
float SD_Tally @".sram2";
float SD_TripThresholdI2t @".sram2";
float GF_HalfCycPickup, GF_OneCycPickup, GF_OneCyc_8x;
float GF_Tally @".sram2";
float GF_TallyDecr @".sram2";
float GF_OneCyc_0p625 @".sram2";
float GF_TripThresholdI2t @".sram2";
float LD_OneCycPickup;
float LD_Tally @".sram2";
float LD_TripThreshold @".sram2";
float LD_TallyIncr @".sram2";
float LD_TallyDecr @".sram2";
float PA_TripThreshold @".sram2";
float PB_TripThreshold @".sram2";
float PBdelay_Trip;
float Ir @".sram2";
float Ir_02 @".sram2";
float GF_In @".sram2";

float LD_Bucket;
float LD_BucketMax;
struct INTERNAL_TIME LD_BucketMaxTS;
float LD_TimeToTrip;
float SD_BucketMax;
struct INTERNAL_TIME SD_BucketMaxTS;
struct CUR_WITHOUT_G_F EventCurOneCyc;
float EventCurOneCycIg;

struct FW_SIMULATED_TEST FW_SimulatedTest;
struct HW_SEC_INJ_TEST HW_SecInjTest;
struct COIL_DETECT CoilDetect;

uint32_t SD_Passes @".sram2";         // uint32 needed for I2t
uint16_t LD_Passes @".sram2";
uint32_t GF_Passes @".sram2";         // uint32 needed for I2t
uint16_t PB_Passes @".sram2";
uint16_t SD_TripThresholdFlat @".sram2";
uint16_t GF_TripThresholdFlat @".sram2";
uint8_t Inst_State @".sram2";
uint8_t Inst_SamplePasses @".sram2";
uint8_t Inst_HalfCycPasses @".sram2";
uint8_t SD_State @".sram2";
uint8_t SD_Slope @".sram2";
uint8_t LD_State @".sram2";
uint8_t LD_Slope @".sram2";
uint8_t GF_State @".sram2";
uint8_t GF_Slope @".sram2";
uint8_t ST_MM_On @".sram2";
uint8_t MM_HiLo_Gain @".sram2";
uint8_t DB_HiLo_Gain @".sram2"; 

uint8_t SD_StartupSampleCnt, GF_StartupSampleCnt;


// ---------------------------------- *** BP - added these to compile until code is written for them
uint8_t IecSel;                        // *** BP -added this until Factory Options are defined. Used in GF code.
uint8_t TU_State_TestUSBMode;          // TU_State_Test.USBMode in the future
uint8_t TU_State_TestMode;             // TU_State.TestMode in the future
uint8_t TU_State_PowerUp;              // TU_State.PowerUp in the furure
uint8_t TU_State_OpenedByComm;         // TU_State.OpenedByComm in the future
uint8_t TU_State_ClosedByComm;         // TU_State.ClosedByComm in the future
uint8_t A_Scope_Req;
uint8_t Break_Config_config_Poles;     // Break_Config.config.Poles in the future
uint16_t Diag_Data_data_Life_Point;     // Diag_Data.data.Life_Point in the future
//-----------------------------------

uint16_t PickupFlags;
uint16_t SneakCount;
uint16_t SneakAlmTmr;
uint16_t BadTaAlmTmr;
uint16_t RevSeqAlmTmr; 
uint16_t RevSeqTripTmr;
uint16_t HlAlm1Tmr;
uint16_t HlAlm2Tmr;
uint16_t THDCurrAlmTmr;
uint16_t THDVoltAlmTmr;
uint16_t RevPwrAlmTmr;
uint16_t RevReacPwrAlmTmr;


uint16_t CurUnbalTripTmr;            // *** BP - initialize these ***
uint16_t VoltUnbalTripTmr;
uint16_t OvTripTmr;
uint16_t UvTripTmr;
uint16_t OfTripTmr;
uint16_t UfTripTmr;
uint16_t RealPwrTripTmr;
uint16_t ReacPwrTripTmr;
uint16_t AppPwrTripTmr;
uint16_t UnderPFTripTmr;
uint16_t PhaseLossTripTmr;

uint16_t CurUnbalAlmTmr;
uint16_t PhaseLossTripTmr;
uint16_t VoltUnbalAlmTmr;
uint16_t OvAlmTmr;
uint16_t UvAlmTmr;
uint16_t OfAlmTmr;
uint16_t UfAlmTmr;
uint16_t RealPwrAlmTmr;
uint16_t ReacPwrAlmTmr;
uint16_t AppPwrAlmTmr;
uint16_t RevPwrTripTmr;
uint16_t RevReacPwrTripTmr;
uint16_t UnderPFAlmTmr;
uint16_t PhaseLossAlmTmr;
uint16_t TherMemAlmTmr;
uint16_t NeutAlmTmr;

union WORD_BITS TripPuFlags;
float OVR_Threshold;
float MCR_Threshold;
float MM_Threshold;
float DB_Threshold;

const float MM_TripLevel[] = {1.5, 2.5, 4.0, 6.0, 8.0, 10.0};

extern struct EV_SUM EV_Sum;          // *** DAH TEST TO INSERT 500 EVENTS

uint8_t dfred_on;  // *** DAH TEST
float dfred_test;
union CRITICAL_BREAKER_CONFIG Break_Config;


//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//


//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Prot_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Protection Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Protection Module.
//                      The following variables do not need initialization:
//                          SD_Tally (initialized in state 0 of ShortDelay_Prot())
//                          Inst_HalfCycPasses (initialized in state 0 of Instantaneous_Prot())
//                          Inst_SamplePasses (initialized in state 0 of Instantaneous_Prot())
//                          SD_Passes (initialized in State 0 of ShortDelay_Prot())
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            Inst_State, SD_State, PickupFlags, LD_Tally, PB_Passes, LD_Bucket, LD_BucketMax,
//                      LD_BucketMaxTS, SD_BucketMax, SD_BucketMaxTS, Trip_WF_OffsetTime,
//                      Alarm_WF_OffsetTime, Ext_WF_OffsetTime, TripPuFlags, OvTripBucket, UvTripBucket,
//                      VuTripBucket, CuTripBucket, RevWTripBucket, RevVarTripBucket, Inst_StartupSampleCnt,
//                      SD_StartupSampleCnt, GF_StartupSampleCnt
//
//  ALTERS:             None
//
//  CALLS:              Gen_Values()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code, no FRAM reads): 2.2usec
//
//------------------------------------------------------------------------------------------------------------

void Prot_VarInit(void)
{
  Inst_State = 0;
  SD_State = 0;
//  SD_Passes = 0;                                        // Initialized in State 0 of ShortDelay_Prot()
  PickupFlags = 0;

  LD_Tally = 0.0;                                         // Clear long delay tally register
  LD_Bucket = 0.0;
  LD_BucketMax = 0.0;                                     // *** DAH  NEED TO RETRIEVE FROM FRAM
  LD_BucketMaxTS.Time_secs = 0;                           // *** DAH  NEED TO RETRIEVE FROM FRAM
  LD_BucketMaxTS.Time_nsec = 0;                           // *** DAH  NEED TO RETRIEVE FROM FRAM
  SD_BucketMax = 0.0;                                     // *** DAH  NEED TO RETRIEVE FROM FRAM
  SD_BucketMaxTS.Time_secs = 0;                           // *** DAH  NEED TO RETRIEVE FROM FRAM
  SD_BucketMaxTS.Time_nsec = 0;                           // *** DAH  NEED TO RETRIEVE FROM FRAM
  PB_Passes = 0;

  // Clear trip timers
  OvTripTmr = 0;
  UvTripTmr = 0;
  OfTripTmr = 0;
  UfTripTmr = 0;
  SneakCount = 0;
  CurUnbalTripTmr = 0;
  PhaseLossTripTmr = 0;
  VoltUnbalTripTmr = 0;
  RealPwrTripTmr = 0;
  ReacPwrTripTmr = 0;
  AppPwrTripTmr = 0;
  RevPwrTripTmr = 0;
  RevReacPwrTripTmr = 0;
  RevSeqTripTmr = 0;
  UnderPFTripTmr = 0;
  
  // Clear alarm timers
  SneakAlmTmr = 0;
  TherMemAlmTmr = 0;
  HlAlm1Tmr = 0;
  HlAlm2Tmr = 0;
  OvAlmTmr = 0;
  UvAlmTmr = 0;
  OfAlmTmr = 0;
  UfAlmTmr = 0;
  CurUnbalAlmTmr = 0;
  PhaseLossAlmTmr = 0;
  VoltUnbalAlmTmr = 0;
  RealPwrAlmTmr = 0;
  ReacPwrAlmTmr = 0;
  AppPwrAlmTmr = 0;
  RevPwrAlmTmr = 0;
  RevReacPwrAlmTmr = 0;
  UnderPFAlmTmr = 0;
  NeutAlmTmr = 0;
  THDCurrAlmTmr = 0;
  THDVoltAlmTmr = 0;
  BadTaAlmTmr = 0;
  BatteryValid = 0;
  
  // Clear Trip and Alarm flags
  Trip_Flags0.all = 0;
  Trip_Flags1.all = 0;
  Alarm_Flags0.all = 0;
  Alarm_Flags1.all = 0;
  Alarm_Flags2.all = 0;

  TripPuFlags.all = 0;

  OvTripBucket = 0;
  UvTripBucket = 0;
  VuTripBucket = 0;
  CuTripBucket = 0;
  RevWTripBucket = 0;
  RevVarTripBucket = 0;
  PlTripBucket = 0;
  OfTripBucket = 0;
  UfTripBucket = 0;
  OvrWTripBucket = 0;
  OvrVarTripBucket = 0;
  OvrVATripBucket = 0;
  PFTripBucket = 0;
  GFTripBucket = 0;
  SDTripBucket = 0;
  Flags0.all = 0;
  Flags1.all = 0;
  Flags2.all = 0;
  
  Diag_Data_data_Life_Point = 0;                      // *** BP until Breaker Health code is written
  
  GF_Enabled = FALSE;                                 // will be updated in Gen_Values based on Setpoints1 Style_2

  Gen_Values();                                           // Generate protection limits based on setpoints
  dfred_on = 0;  // *** DAH TEST
  dfred_test = 0;

  TU_State_TestUSBMode = FALSE;

  Trip_WF_OffsetTime = 0;
  Alarm_WF_OffsetTime = 0;
  Ext_WF_OffsetTime = 0;
  
  FW_SimulatedTest.State = 0;
  FW_SimulatedTest.Enable = 0;
  FW_SimulatedTest.Phase = TEST_IA;
  FW_SimulatedTest.StartingSampleCounter = 0;
  FW_SimulatedTest.EndingSampleCounter = 0;
  FW_SimulatedTest.TestCurrent = 0;
  FW_SimulatedTest.TestResultTime = 0;
  FW_SimulatedTest.TestResultCurrent = 0;
  
  HW_SecInjTest.State = 0;
  HW_SecInjTest.Enable = 0;
  HW_SecInjTest.Phase = TEST_IA;
  HW_SecInjTest.Inst_Trip_StartingSampleCounter = 0;
  HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged = FALSE;
  HW_SecInjTest.EndingSampleCounter = 0;
  HW_SecInjTest.TestCurrent = 0;
  HW_SecInjTest.TestResultTime = 0;
  HW_SecInjTest.TestResultCurrent = 0;
  HW_SecInjTest.OvrMicroTrip = 0;
  
  CoilDetect.State = 0;
  CoilDetect.Enable = 0;
  CoilDetect.SampleCount = 0;
  CoilDetect.Avg_Ia = 0;
  CoilDetect.Avg_Ib = 0;
  CoilDetect.Avg_Ic = 0;
  CoilDetect.Avg_In = 0;
  CoilDetect.Result = 0;
  CoilDetect.TestAllowedThreshold = 10;     // Can't run Coil Detection if >10A present
  CoilDetect.TestInProgress = 0;
  T_Forbid_Coil = 0;
  T_Forbid_SecInj = 0;
  

  Inst_StartupSampleCnt = 40;               // Default is one half-cycle
  SD_StartupSampleCnt = 40;                 // One half-cycle
  GF_StartupSampleCnt = 40;                 // One half-cycle

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Prot_VarInit()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Gen_Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Generate Protection values
//
//  MECHANICS:          This subroutine calculates the pickup values and trip threshold values
//                      used in the Protection Module.
//                      Note: all values used in protection should be generated in this subroutine, even        *** DAH - NOTE THIS!!!
//                      values that are just a copy of a setpoint.  The setpoint itself should NOT be used,
//                      because there could be tearing of values in the protection subroutine.  Remember,
//                      the protection subroutines are called through an interrupt.  If the setpoint is
//                      used before Gen_Values() is called, the setpoint would be updated but not any
//                      internal values.  When setpoints are updated, either interrupts must be disabled
//                      around a call to Gen_Values() in the main loop, or else a flag must be set to cause
//                      Gen_Values() to be called in the sampling interrupt before the protection
//                      subroutines are called.
//
//                      BP - For now, the Short Delay trip threshold values are based on +0%/-20%.
//                           (This will change when Jim Lagree defines the spec ranges)
//
//  CAVEATS:            Call during initialization and whenever the Protection (Group 1) setpoints
//                      are changed.
//
//  INPUTS:             None
//
//  OUTPUTS:            Inst_Pickup, SD_HalfCycPickup, SD_OneCycPickup, SD_OneCyc_8x, LD_OneCycPickup,
//                      SD_Slope, LD_Slope, LD_TripThreshold, SD_TripThriesholdFlat, SD_TripThresholdI2t
//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     Measured on 230809 (rev 56 code): 80usec max with (interrupts enabled, so includes
//                              the sample interrupt time)
//
//------------------------------------------------------------------------------------------------------------

void Gen_Values(void)
{
  float fLD_Tt;                         // float version of integer (as x10) setpoint

  Dmnd_Type_Window = Setpoints0.stp.DemandInterval + (((uint32_t)Setpoints0.stp.DemandWindow) << 16);



  Ir = (float)Break_Config.config.Rating * Setpoints1.stp.Ld_Pu / 100.0;   // In x LDPU (where LDPU integer value is x100)

  //------------------------------------ Instantaneous Protection Values -----------------------------------
  //
  // Internal instantaneous pickup value.  This is the min half-cycle sum-of-squares total to enter pickup
  // Value is (setting x rating)^2 * 40 (number of half-cycle samples)
  Inst_Pickup = (float)Setpoints1.stp.Inst_Pu / 10.0;                  // Setting is x10
  Inst_Pickup = (Break_Config.config.Rating * Inst_Pickup) *           // Compute internal value
                 (Break_Config.config.Rating * Inst_Pickup) * 4E1;
  //--------------------------------------------------------------------------------------------------------


  //------------------------------------- Short Delay Protection Values ------------------------------------
  //
  SD_Slope = Setpoints1.stp.Sd_Slp;

  // Internal short delay half-cycle pickup value.  This is the min half-cycle sum-of-squares total to enter
  // pickup.  Value is (setting x Ir)^2 * 40 (number of half-cycle samples)
  SD_HalfCycPickup = (float)Setpoints1.stp.Sd_Pu / 10.0;               // integer value of x10 to float
  SD_HalfCycPickup = (SD_HalfCycPickup * Ir) * (SD_HalfCycPickup * Ir) * 4E1;

  // Internal one-cycle pickup value
  SD_OneCycPickup = SD_HalfCycPickup * 2;

  // Internal short delay one-cycle 8x sum-of-squares.  Threshold for switching from I2t to Flat
  SD_OneCyc_8x = (8.0 * Ir) * (8.0 * Ir) * 80.0;

  
  // Internal short delay interlock pickup for ZSI out
  // The interlock pickup value is calculated as the lesser of 1.5x because that is the lowest SD setting that an upstream
  // breaker can have
  SD_InterlockPickup = (1.5 * Ir) * (1.5 * Ir) * 40.0;

  // Internal flat short delay trip time.  This number is the trip time setting converted to the number of
  //   sample times the unit is above the pickup level (since SD protection is executed each sample)
  // Make sure computation stays within a uint16 - max setting is about 13.5sec
  //      (80 sa/cyc * Setpoints0.stp.Freq)  = 4800 sa/sec for 60Hz
  //      80 sa/cyc * Setpoints0.stp.Freq * Setpoints1.stp.Sd_Tt hundredths-sec / 100 hundredths-sec/sec ---> xxxx samples
  // BP - changed from 60 cyc/sec to use Freq setpoint to account for 50Hz
  SD_TripThresholdFlat = (uint16_t)(((uint32_t)Setpoints1.stp.Sd_Tt * (uint32_t)Setpoints0.stp.Freq * 80)/100);
  
  // Compensate for slow trip times and different tolerances  (Sd_Tt setpoint is x100)
  if (Setpoints1.stp.Sd_Tt > 19)                         // 200ms - 500ms    (Trip tolerance = +0%/-20)
  {
    SD_TripThresholdFlat = (uint16_t)(SD_TripThresholdFlat * 0.8);
  }
  else if (Setpoints1.stp.Sd_Tt > 15)                    // 160ms - 190ms    (Trip tolerance = +0%/-30)
  {
    SD_TripThresholdFlat = (uint16_t)(SD_TripThresholdFlat * 0.7);
  }    
  else if (Setpoints1.stp.Sd_Tt > 9)                     // 100ms - 150ms    (Trip tolerance = +0%/-40)
  {
    SD_TripThresholdFlat = (uint16_t)(SD_TripThresholdFlat * 0.7);
  }    
  else                                                   // 50ms - 90ms     (Trip tolerance = +20%/-50)
  {
    SD_TripThresholdFlat = (uint16_t)(SD_TripThresholdFlat * 0.5);
  }   
  
  
  // Internal I2t short delay trip setting.  This number is computed from the trip time setting and Ir
  //   rating.  SD curves are based on 8x Ir setting.  The value is the 8x Ir current squared multiplied by
  //   the number of samples in the time setting (I^2 * time setting).
  // This is a floating point value since it contains both current and time
  //      (80 sa/cyc * 60 cyc/sec) = 4800 sa/sec for 60Hz
  //      80 sa/cyc * (8 * Ir)^2 * Setpoints0.stp.Freq * Setpoints1.stp.Sd_Tt hundredths-sec / 100 hun-sec/sec ---> xxxx setting
  // BP - changed from 60 cyc/sec to use Freq setpoint to account for 50Hz
  SD_TripThresholdI2t = ((80.0 * 64.0) * (Ir * Ir) * (float)Setpoints0.stp.Freq * (float)Setpoints1.stp.Sd_Tt)/100.0;
  
  // Compensate for slow trip times and different tolerances  (Sd_Tt setpoint is x100)
  if (Setpoints1.stp.Sd_Tt > 30)                         // 310ms - 500ms    (Trip tolerance = +0%/-20)
  {
    SD_TripThresholdI2t = SD_TripThresholdI2t * 0.8;
  }
  else if (Setpoints1.stp.Sd_Tt > 15)                    // 160ms - 300ms    (Trip tolerance = +0%/-30)
  {
    SD_TripThresholdI2t = SD_TripThresholdI2t * 0.7;
  }    
  else if (Setpoints1.stp.Sd_Tt > 9)                     // 100ms - 150ms    (Trip tolerance = +0%/-40)
  {
    SD_TripThresholdI2t = SD_TripThresholdI2t * 0.7;
  }    
  else                                                   // 50ms - 90ms     (Trip tolerance = +0%/-50)
  {
    SD_TripThresholdI2t = SD_TripThresholdI2t * 0.6;
  }   
  //--------------------------------------------------------------------------------------------------------


  //------------------------------------- Long Delay Protection Values -------------------------------------
  //
  LD_Slope = Setpoints1.stp.Ld_Slp;
  fLD_Tt = (float)Setpoints1.stp.Ld_Tt / 100.0;                         // integer value of x100 to float

  // Internal long delay one-cycle pickup value.  This is the min one-cycle sum-of-squares total to enter
  // pickup.  Value is (1.1 x Ir)^2 * 80 (number of one-cycle samples)
  LD_OneCycPickup = (1.1 * Ir) * (1.1 * Ir) * 80;                      // 110%, 80 samples

  // Internal long delay trip setting.  This number is computed from the slope, trip time setting, and Ir   
  //   rating.  This is a floating point value since it contains both current and time.
  // LD curves are based on 6x Ir setting.
  // 0.85 multiplier is used because spec, for now, is +0/-30%
  if (LD_Slope == LD_I05T)                          // I0.5T at 6x  (80 samples, 60Hz, 0.85 multiplier)
  {
    LD_TripThreshold =  2.9907 * sqrtf(6 * Ir) * fLD_Tt * (float)Setpoints0.stp.Freq * .85;   // 2.99 is sqrt(sqrt(80))
  }
  else if (LD_Slope == LD_I1T)                      // IT at 6x  (80 samples, 60Hz, 0.85 multiplier)
  {
    LD_TripThreshold = 8.944 * (6 * Ir) * fLD_Tt * (float)Setpoints0.stp.Freq * .85;          // 8.944 is sqrt(80)
  }
  else if (LD_Slope == LD_I4T)                      // I4T at 6x  (80 samples, 60Hz, 0.85 multiplier)
  {
    LD_TripThreshold = (8E1 * (6 * Ir) * (6 * Ir)) * (8E1 * (6 * Ir) * (6 * Ir))  * fLD_Tt * (float)Setpoints0.stp.Freq * .85;
  }
  else                                              // I2T at 6x  (80 samples, 60Hz, 0.85 multiplier)
  {
    LD_TripThreshold = 8E1 * (6 * Ir) * (6 * Ir) * fLD_Tt * (float)Setpoints0.stp.Freq * .85;    
  }


  if ( LD_Slope == LD_IEEE_MI)                                             // IEEE Moderately Inverse (I.02T)
  {
    Ir_02 = pow(Ir, 0.02);                                                          // raise to power of 0.02
    PA_TripThreshold = 8E1 * .0515 * Ir_02 * fLD_Tt * (float)Setpoints0.stp.Freq;   // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = .114 * fLD_Tt * (float)Setpoints0.stp.Freq;                  // "B" delay timeout (60 Hz)
  }
  else if ( LD_Slope == LD_IEEE_EI)                                        // IEEE Extremely Inverse (I2T)
  {
    PA_TripThreshold = 8E1 * 28.3 * Ir * Ir * fLD_Tt * (float)Setpoints0.stp.Freq;  // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = .1217 * fLD_Tt * (float)Setpoints0.stp.Freq;                 // "B" delay timeout (60 Hz)
  }
  else if ( LD_Slope == LD_IEEE_VI)                                        // IEEE Very Inverse (I2T)
  {
    PA_TripThreshold = 8E1 * 19.61 * Ir * Ir * fLD_Tt * (float)Setpoints0.stp.Freq; // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = .491 * fLD_Tt * (float)Setpoints0.stp.Freq;                  // "B" delay timeout (60 Hz)
  }
  else if (LD_Slope == LD_IEC_B)                                           // IECB Very Inverse (IT)
   {
    PA_TripThreshold = 8E1 * 13.5 * Ir * fLD_Tt * (float)Setpoints0.stp.Freq;       // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = 0;                                                           // "B" delay timeout (60 Hz)
  }
  else if (LD_Slope == LD_IEC_A)                                           // IECA Normal Inverse (I.02T)
   {
    Ir_02 = pow(Ir, 0.02);  
    PA_TripThreshold = 8E1 * 0.14 * Ir_02 * fLD_Tt * (float)Setpoints0.stp.Freq;    // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = 0;                                                           // "B" delay timeout (60 Hz)
  }   
  else                                                                              // IECC Extremely Inverse (I2T)
  {
    PA_TripThreshold = 8E1 * 80 * Ir * Ir * fLD_Tt * (float)Setpoints0.stp.Freq;    // "A" delay timeout (80 samples, 60 Hz)
    PB_TripThreshold = 0;                                                           // "B" delay timeout (60 Hz)
  }

 
  //------------------------------------- Ground Fault Protection Values ---------------------------------------
  //
  //        Setpoints1 Style2 is used for Ground Fault Digitization
  //
  //        Setpoints1.stp.Style_2   Bit0     GF Option (0 = disabled, 1 = enabled)
  //                                 Bit1     UL 1200A GF limit   0 = for In>1200A, GF_In = 1200A  
  //                                                              1 = for In>1200A, GF_In = In
  //                                          Note: This scaling is only for Residual Ground (for Trip only, not Alarm)
  //                                          - For In < 1200A, GF_In = In
  
  GF_Enabled = Setpoints1.stp.Style_2 & BIT0;     // GF Enable is bit 0

  if ((Break_Config.config.Rating > 1200) && ((Setpoints1.stp.Style_2 & BIT1) == 0) && (Setpoints1.stp.Gnd_Type == 0) && (Setpoints1.stp.Gf_T_A_OFF == 0))
  {                                                                               // Style2, bit1 not set and Residual and Trip
    GF_In = 1200;             // limit to 1200A for all calculations
  }
  else
  {
    GF_In = (float)Break_Config.config.Rating;
  }

  GF_Slope = Setpoints1.stp.Gf_Slp;

  // Internal ground fault half-cycle pickup value.  This is the min half-cycle sum-of-squares total to enter
  // pickup.  Value is (setting x Ir)^2 * 40 (number of half-cycle samples)
  GF_HalfCycPickup = (float)Setpoints1.stp.Gf_Pu / 100.0;               // integer value of x10 to float
  GF_HalfCycPickup = (GF_HalfCycPickup * GF_In) * (GF_HalfCycPickup * GF_In) * 4E1;

  // Internal one-cycle pickup value
  GF_OneCycPickup = GF_HalfCycPickup * 2;

  // Internal ground fault one-cycle 0.625x sum-of-squares.  Threshold for switching from I2t to Flat
  GF_OneCyc_0p625 = (0.625 * GF_In) * (0.625 * GF_In) * 80.0;


  //  Ground fault decrement value of 0.25pu
  GF_TallyDecr = 0.25 * GF_In * GF_In ;


  // Internal flat ground fault trip time.  This number is the trip time setting converted to the number of
  //   sample times the unit is above the pickup level (since GF protection is executed each sample)
  // Make sure computation stays within a uint16 - max setting is about 13.5sec
  //      (80 sa/cyc * Setpoints0.stp.Freq) = 4800 sa/sec for 60 Hz
  //      80 sa/cyc * Setpoints0.stp.Freq * Setpoints1.stp.Sd_Tt hundredths-sec / 100 hundredths-sec/sec ---> xxxx samples
  // BP - changed from 60 cyc/sec to use Freq setpoint to account for 50Hz
  GF_TripThresholdFlat = (uint16_t)(((uint32_t)Setpoints1.stp.Gf_Tt * (uint32_t)Setpoints0.stp.Freq * 80)/100);
     
  // Compensate for slow trip times and different tolerances  (Gf_Tt setpoint is x100)
  if (Setpoints1.stp.Gf_Tt > 39)                         // 400ms - 1s      (Trip tolerance = +0%/-20)
  {
    GF_TripThresholdFlat = (uint16_t)(GF_TripThresholdFlat * 1.0);
  }
  if (Setpoints1.stp.Gf_Tt > 19)                         // 200ms - 390ms   (Trip tolerance = +0%/-20)
  {
    GF_TripThresholdFlat = (uint16_t)(GF_TripThresholdFlat * 0.8);
  }
  else if (Setpoints1.stp.Gf_Tt > 15)                    // 160ms - 190ms   (Trip tolerance = +0%/-30)
  {
    GF_TripThresholdFlat = (uint16_t)(GF_TripThresholdFlat * 0.7);
  }    
  else if (Setpoints1.stp.Gf_Tt > 9)                     // 100ms - 150ms   (Trip tolerance = +0%/-40)
  {
    GF_TripThresholdFlat = (uint16_t)(GF_TripThresholdFlat * 0.7);
  }    
  else                                                   // 50ms - 90ms     (Trip tolerance = +20%/-40)
  {
    GF_TripThresholdFlat = (uint16_t)(GF_TripThresholdFlat * 0.5);
  } 

  // Internal I2t ground fault trip setting.  This number is computed from the trip time setting and Ir
  //   rating.  GF curves are based on 0.625x In setting.  The value is the 0.625x In current squared multiplied by
  //   the number of samples in the time setting (I^2 * time setting).
  // This is a floating point value since it contains both current and time
  //      (80 sa/cyc * Setpoints0.stp.Freq) = 4800 sa/sec for 60 Hz
  //      80 sa/cyc * Setpoints0.stp.Freq * (0.625 * In)^2 * Setpoints1.stp.Gf_Tt hundredths-sec / 100 hun-sec/sec ---> xxxx setting
  // BP - changed from 60 cyc/sec to use Freq setpoint to account for 50Hz
  GF_TripThresholdI2t = ((80.0 * 0.390625) * (float)Setpoints0.stp.Freq * (GF_In *  GF_In) * (float)Setpoints1.stp.Gf_Tt)/100.0;
  
  // Compensate for slow trip times and different tolerances  (Gf_Tt setpoint is x100)
  if (Setpoints1.stp.Gf_Tt > 30)                         // 310ms - 1s       (Trip tolerance = +0%/-20)
  {
    GF_TripThresholdI2t = GF_TripThresholdI2t * 0.8;
  }
  else if (Setpoints1.stp.Gf_Tt > 19)                    // 200ms - 300ms    (Trip tolerance = +0%/-20)
  {
    GF_TripThresholdI2t = GF_TripThresholdI2t * 0.8;
  }    
  else if (Setpoints1.stp.Gf_Tt > 15)                    // 160ms - 190ms    (Trip tolerance = +0%/-30)
  {
    GF_TripThresholdI2t = GF_TripThresholdI2t * 0.7;
  }    
   else if (Setpoints1.stp.Gf_Tt > 9)                    // 100ms - 150ms    (Trip tolerance = +0%/-40)
  {
    GF_TripThresholdI2t = GF_TripThresholdI2t * 0.7;
  }    
  else                                                   // 50ms - 90ms     (Trip tolerance = +20%/-40)
  {
    GF_TripThresholdI2t = GF_TripThresholdI2t * 0.5;
  }   

  // Set up the port pin outputs for Neutral and Ground Measurement circuitry based on these setpoints  
  if (Setpoints0.stp.Neutral_Sensor == 0)     // Rogowski
  {
    IN_ROGO_ACTIVE;                    // Set up IN_USING_ROGOWSKI port pin for Rogowski measurement circuity 
  }
  else                                 // CT
  {
    IN_ROGO_INACTIVE;                  // Use CT measurement circuitry
  }
  
  if (Setpoints1.stp.SG_Sensor == 0)   // Rogowski
  {
    GF_ROGO_ACTIVE;                    // Set up GF_USING_ROGOWSKI port pin for Rogowski measurement circuity 
  }
  else                                 // CT
  {
    GF_ROGO_INACTIVE;                  // Use CT measurement circuitry
  } 

  
  // ---------------------------- Override Micro Protection Values -----------------------------------------------------------
  //
  // The Override micro handles the MCR, Override, ARMs, and Digital Bypass protections.  The Protection processor calculates the 
  // trip thresholds for these protections and sends them to the Override micro via I2C communications.  The Override micro compares
  // its raw ADC samples for Ia, Ib, and Ic to these thresholds.  Three consecutuve samples over a threshold will cause a trip.
  //
  // High Gain/Low Gain is implemented to get the most signal into the Override micro without saturating the ADC 2048 because of 
  // the 1.25V offset.  The lower Maintanence mode trip settings and the lower Digital Bypass trip levels will use high gain current
  // values.
  //
  // MM High Gain/Low Gain gets sent to the Override micro via I2C.  The Override micro controls the mux to select the High Gain current
  // or the Low Gain current.
  //
  // The scaling is shown below:
  //
  //
  //                     ---------- Current scaling circuit --------------------------------   --- Override ADC ----------
  //              +----------+   +---------+   +------------+    +-------+
  //              | Rogowski |   | Voltage |   | Integrator |    | Low   |                         +----------+
  //  Current --->|   di/dt  |-->| Divider |-->|            |--->| Gain  |--------+--------------->|          |
  //              |  Sensor  |   |  x0.5   |   |  x0.0603   |    |  x1   |        |                | Override |
  //              +----------+   +---------+   +------------+    +-------+        |   +------+     |   ADC    |--> Current
  //              <------------------------------------------------------ >       |   | High |     |          |    Samples
  //  Magnum Std/Double: 0.166mV/A                        |                       +-->| Gain |---->|  12-bit  |
  //  Magnum Nar/Double: 0.208mV/A                        |                           |      |     |          |
  //                                            Magnum Standard/Double: 5.00uV/A      | x51  |     | unsigned |
  //                                            Magnum Narrow/Double:   6.27uV/A      +------+     +----------+
  //                                                                                                 4095/2.5V
  //                                    (Note:  Override and MCR only use Low Gain current values)
  //
  //
  //  Override Trip threshold (Std/DW) = Break_Config.config.OvrWithstand(kA) * 1000 * 5.00E-6 * 4095/2.5  (only low gain)
  //  Override Trip threshold (Nar/DN) = Break_Config.config.OvrWithstand(kA) * 1000 * 6.27E-6 * 4095/2.5  (only low gain)  
  //  MCR Trip threshold (Std/DW) = Break_Config.config.MCR(kA) * 1000 * 5.00E-6 * 4095/2.5  (only low gain)
  //  MCR Trip threshold (Nar/DN) = Break_Config.config.MCR(kA) * 1000 * 6.27E-6 * 4095/2.5  (only low gain) 
  
  //Break_Config.config.Rating = Break_Config_config_Rating;   // *** BP for testing
  
  switch (Break_Config.config.BreakerFrame)                 
  {
    case MAGNUM_STD:                          
    case MAGNUM_STD_DW:  
    default:
      OVR_Threshold = (float)Break_Config.config.OvrWithstand * 8.19;  // for low gain (no high gain for OVR or MCR)
      MCR_Threshold = (float)Break_Config.config.MCR * 8.19;           // for low gain
      break;                                

    case MAGNUM_NRW:
    case MAGNUM_NRW_DW: 
      OVR_Threshold = (float)Break_Config.config.OvrWithstand * 10.27; // for low gain (no high gain for OVR or MCR)
      MCR_Threshold = (float)Break_Config.config.MCR * 10.27;          // for low gain
      break;
  }   
          
  // Maintenance Mode: Setpoints0.stp.MM_Enable    b8 = ARMS mode status    b7 = Local ON    b0 = Remote COMM channel
  // Maintenance Mode: Setpoints0.stp.MaintLevel   0 = 1.5*In, 1 = 2.5*In, 2 = 4.0*In, 3 = 6.0*In, 4 = 8.0*In, 5 = 10.0*In 
       
  //  MM Trip threshold (Std/DW) = Break_Config.config.Rating * 1.414 * MM_TripLevel[Setpoints0.stp.MaintLevel] * 5.00E-6 * 4095/2.5    (low gain)
  //  MM Trip threshold (Std/DW) = Break_Config.config.Rating * 1.414 * MM_TripLevel[Setpoints0.stp.MaintLevel] * 255.25E-6 * 4095/2.5  (high gain)    
  //  MM Trip threshold (Nar/DN) = Break_Config.config.Rating * 1.414 * MM_TripLevel[Setpoints0.stp.MaintLevel] * 6.27E-6 * 4095/2.5    (low gain)
  //  MM Trip threshold (Nar/DN) = Break_Config.config.Rating * 1.414 * MM_TripLevel[Setpoints0.stp.MaintLevel] * 319.83E-6 * 4095/2.5  (high gain) 
  switch (Break_Config.config.BreakerFrame)                 
  {                      
    case MAGNUM_STD:        
    case MAGNUM_STD_DW:               
    default:          
      if ((float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] > 3200)                   // so Override ADC doesn't saturate
      {
        MM_HiLo_Gain = 0;                                                                                       // low gain
        MM_Threshold = (float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] * 0.01158;   // for low gain
      }
      else
      {
        MM_HiLo_Gain = 1;                                                                                       // high gain
        MM_Threshold = (float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] * 0.5913;    // for high gain
      }          
      break;                                

    case MAGNUM_NRW:                                  
    case MAGNUM_NRW_DW: 
      if ((float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] > 2520)                   // so Override ADC doesn't saturate
      {
        MM_HiLo_Gain = 0;                                                                                       // low gain
        MM_Threshold = (float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] * 0.01452;   // for low gain
      }
      else
      {
        MM_HiLo_Gain = 1;                                                                                       // high gain
        MM_Threshold = (float)Break_Config.config.Rating * MM_TripLevel[Setpoints0.stp.MaintLevel] * 0.7409;    // for high gain
      }                  
      break;
  }
    
  if (Setpoints0.stp.MM_Enable > 0)                               // Maintenance Mode is turned on
  {
    ST_MM_On = 1;                                                 // gets sent to Override micro
  }  
  else                                                            // Maintenance Mode is turned off
  {  
    ST_MM_On = 0;                                                 // turn off MM Mode protection in Override micro
  }
  
 
  // Digtal Bypass is implemented by the Override micro after 6 seconds of bad comms.  The DB Threshold is 1.5 x In.
  // The ST Micro sends down the correct DB Threshold that will be used and also a DB High/Low Gain flag so that the
  // Override micro knows which gain to implement based on the breaker rating.
  //    DB Trip threshold (Std/DW) = Break_Config.config.Rating * 1.414 * 1.5 * 5.00E-6 * 4095/2.5    (low gain)
  //    DB Trip threshold (Std/DW) = Break_Config.config.Rating * 1.414 * 1.5 * 255.25E-6 * 4095/2.5  (high gain)    
  //    DB Trip threshold (Nar/DN) = Break_Config.config.Rating * 1.414 * 1.5 * 6.27E-6 * 4095/2.5    (low gain)
  //    DB Trip threshold (Nar/DN) = Break_Config.config.Rating * 1.414 * 1.5 * 319.83E-6 * 4095/2.5  (high gain) 
  switch (Break_Config.config.BreakerFrame)                 
  {
    case MAGNUM_STD:                          
    case MAGNUM_STD_DW:  
    default:
      if (Break_Config.config.Rating > 2000)                            // so Override ADC doesn't saturate
      {
        DB_HiLo_Gain = 0;                                               // low gain
        DB_Threshold = (float)Break_Config.config.Rating * 0.01737;     // for low gain   
      }
      else
      {
        DB_HiLo_Gain = 1;                                               // high gain
        DB_Threshold = (float)Break_Config.config.Rating * 0.8869;      // for high gain   
      }
      break;                                

    case MAGNUM_NRW:
    case MAGNUM_NRW_DW: 
      if (Break_Config.config.Rating > 1600)                            // so Override ADC doesn't saturate
      {
        DB_Threshold = (float)Break_Config.config.Rating * 0.02179;     // for low gain 
      }
      else
      {
        DB_Threshold = (float)Break_Config.config.Rating * 1.1113;      // for high gain   
      }         
      break;
  }   
                
 
  //--------------------------------------- Waveform Capture Values ----------------------------------------
  //
  // Compute the time from the first sample to the present sample in nsec
  // This is simply the number of pre-cycles * 1/60 sec/cyc * 1000000000 nsec/sec
  // Combine so no overflow, but save divide by 3 for last for maximum accuracy
  Ext_WF_OffsetTime = (50000000 * Setpoints0.stp.WFM_ExtCapPreCyc)/3;
  Alarm_WF_OffsetTime = (50000000 * Setpoints0.stp.WFM_AlarmPreCyc)/3;
  Trip_WF_OffsetTime = (50000000 * Setpoints0.stp.WFM_TripPreCyc)/3;
  
  //------------------------------------- Secondary Injection Testing values -----------------------------
  FW_SimulatedTest.TestAllowedThreshold = (uint32_t)(Break_Config.config.Rating * 0.05);  // 5% of Breaker Rating
  HW_SecInjTest.TestAllowedThreshold = (uint32_t)(Break_Config.config.Rating * 0.05);     // 5% of Breaker Rating
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Gen_Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        OverrideTrip()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Override Trip Processing
//
//  MECHANICS:          This subroutine processes and Override Trip:
//                          - Instantaneous Trip LED is turned on (INST)
//                          - Maintenance Mode, MCR, or Override trip flag is set
//                          - Bell trip flag is set
//                          - Protection related alarms are reset
//                          - Trip event is entered
//                          - Trip waveform capture is initiated
//                          - Protection is disabled and protection timer is turned on
//
//  CAVEATS:            None
//
//  INPUTS:             Trip_WF_Capture.InProg, Prot_Timer
//
//  OUTPUTS:            COT_AUX.COT_Code, IntEventBuf[], IntEventInNdx, IntEventBufFull, McrHiTripFlg,
//                      BellTripFlg, Prot_Timer, Trip_WF_Capture.Req, Prot_Enabled, AlarmHoldOffTmr
//
//  ALTERS:             None
//
//  CALLS:              WriteCauseOfTripLEDS(), Get_InternalTime(), TripFlagsReset(),
//                      Reset_ProtRelated_Alarm(), Get_InternalTime()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void OverrideTrip(void)
{
  COT_AUX.COT_Code = 'I';                                   // Turn on the instantaneous cause of
  WriteCauseOfTripLEDS();                                   //   trip LED
  TRIP_LED_ON;
  TripFlagsReset();
  Reset_ProtRelated_Alarm();
  // Put an entry into the new event FIFO (for interrupts)
  if (Prot_Timer == 0)
  {
    if (Setpoints0.stp.MM_Enable > 0)                     // Maintenance mode is on
    {
      MM_TripFlg = 1;
      IntEventBuf[IntEventInNdx].Code = TRIP_MAINTANENCE_MODE_ARMS;
    }
    else if (OpenFlg == 1)                                // Breaker was open or closing into a fault
    {
      McrTripFlg = 1;
      IntEventBuf[IntEventInNdx].Code = TRIP_MCR;
    }
    else                                                  // Override trip
    {
      HWInstTripFlg = 1;
      IntEventBuf[IntEventInNdx].Code = TRIP_OVERRIDE;
    }

    Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
    IntEventInNdx &= 0x07;
    if (IntEventInNdx == IntEventOutNdx)
    {
      IntEventBufFull |= TRUE;
    }                                        // *** BP - determine trip flags from Ovr Micro comms
  }
//  TripReqFlg  = 1;
  BellTripFlg = 1;                            // Set general trip cause flag
  Prot_Timer = 25;                            // hold off next trip for 250ms
  AlarmHoldOffTmr = 200;                      // Hold off alarm functions for 2s
  if (!Trip_WF_Capture.InProg)                // Set trip waveform capture req flag
  {
    Trip_WF_Capture.Req = TRUE;
  }
  Prot_Enabled = FALSE;                       // Turn off protection for 250msec (Prot_Timer = 25)
  
  if (HW_SecInjTest.Enable == TRUE)
  { 
    HW_SecInjTest.OvrMicroTrip = 1;
    HW_SecInjTest.State = 1;                               // Ending/Result state for HW Sec Inj test
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          OverrideTrip()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Instantaneous_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Instantaneous Protection
//
//  MECHANICS:          This subroutine performs instantaneous protection every sample period (208usec at
//                      60Hz) as follows:
//                      State 0:
//                        - Check the maximum 1/2-cycle current sum of squares for pickup
//                              - If max current sum of squares >= pickup
//                                      - Initialize number of sample passes and half-cycles
//                                      - Save 1/2-cycle current sums of squares
//                                      - State = 1
//                              - If max current < pickup, remain in State 0
//                      State 1:
//                        - On 1/2-cycle anniversary...
//                              - Compute 1/2-cycle current sum of squares using present and previous
//                                1/2-cycle current sum of squares
//                              - Save 1/2-cycle current sum of squares
//                              - Compute the maximum 1/2-cycle current sum of squares
//                              - if max current sum of squares >= pickup, increment number of passes
//                                      - if number of passes >= 3, trip
//                              - if max current sum of squares < pickup, State = 0
//                      Note, after a reset, the number of passes is set to 1 to compensate for the power-on
//                      reset delay.
//                      In State 0, the maximum of the half-cycle current sum of squares is checked for
//                      pickup.  This current is computed in the interrupt and is the instantaneous
//                      half-cycle current, calculated each sample period.  The intent is to recognize a
//                      pickup condition as quickly as possible.  This maximum current is not compensated
//                      for DC offsets; that is, no averaging is done with the previous half-cycle current
//                      sum of squares.  This is not necessary, as we are looking to enter pickup and will
//                      enter on the large half-cycle regardless of averaging - the averaging only affects a
//                      small half-cycle.  Typically, a DC offset occurs when the fault occurs, so the unit
//                      will already be in pickup (State 1) when the offset occurs.  If it is not, there
//                      still is not a problem.  If the current is large due to the DC offset, the averaging
//                      algorithm would have kept the larger value and we would enter pickup anyway (there
//                      is no difference in the operation).  If the current is small due to the offset, it
//                      wouldn't enter pickup with or without averaging, because the previous half-cycle was
//                      also below pickup (since it wasn't in pickup to begin with).
//                      In State 1, we are already in pickup.  Here, we use the averaging algorithm to
//                      generate the half-cycle currents used to get the maximum current.  This eliminates
//                      the effects of DC offset currents.
//                      
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             CurHalfCycSOSmax, CurHalfCycSOS_SumF.Ix, Inst_Pickup, CurHalfCycInstSOSsav.Ix,
//                      IntEventOutNdx
//                      
//  OUTPUTS:            IntEventBuf[], IntEventBufFull, Trip_WF_Capture.Req, AlarmHoldOffTmr,
//                      EventCurOneCyc.Ix
//                      
//  ALTERS:             Inst_State, Inst_SamplePasses, Inst_HalfCycPasses, CurHalfCycInstSOSsav.Ix,
//                      IntEventInNdx
//                      
//  CALLS:              Get_InternalTime()
//                      
//------------------------------------------------------------------------------------------------------------

void Instantaneous_Prot(void)
{
  float temp, tempmax;

  // *** DAH  PER DISCUSSIONS WITH RANDY, TED, BERT, WE WILL USE THE ONE-CYCLE VALUES TO GENERATE THE SNAPSHOT VALUES FOR EVENTS.  WE DON'T NEED
  //   TO USE THE ACTUAL HALF-CYCLE INSTANTANEOUS AND SHORT DELAY VALUES
  // half-cyc - used for inst, short delay, ground protection
  // one-cyc used for long delay, other protections, snapshots for events, waveform for user capture
  // one-sec - used for metering (need to add filter)

  if (Inst_State != 1)                  // Default to State 0
  {
    if (CurHalfCycSOSmax >= Inst_Pickup)              // If above pickup level..                         *** DAH TO FORCE A TRIP EVENT, BREAK EMULATOR HERE, SET PICKUP = 0 AND UNCOMMENT
//    if (CurHalfCycSOSmax < Inst_Pickup)              // If above pickup level.. *** DAH TEST FOR TIMING                   LINE BELOW THAT SETS PICKUP BACK TO 4E9.  THIS WILL CAUSE A SINGLE TRIP EVENT
    {     
      if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged == FALSE))
      {
        HW_SecInjTest.Inst_Trip_StartingSampleCounter = SampleCounter;
        HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged = TRUE;
      }
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
        TESTPIN_A3_LOW;                      // *** DAH TEST FOR COLD START TESTING
#endif

      InstpuFlg = 1;   //PickupFlags |= INST_PICKUP;                           // Set pickup flag
//      Inst_SamplePasses = 0;                                // Initialize sample passes
//      Inst_HalfCycPasses = 1;                               // Initialize half-cycle passes
      // If this is the first time in (cold start closing into a fault), Inst_StartupSampleCnt will be
      //   non-zero (between 26 and 48), containing the time (in number of passes; 1 pass = 208usec) that it
      //   took to start up
      // Reduce the trip time by this amount, by initializing Inst_SamplePasses and Inst_HalfCycPasses
      //   accordingly
      if (Inst_StartupSampleCnt > 39)
      {
        Inst_SamplePasses = Inst_StartupSampleCnt - 40;
        Inst_HalfCycPasses = 2;
      }
      else
      {
        Inst_SamplePasses = Inst_StartupSampleCnt;
        Inst_HalfCycPasses = 1;
      }
      Inst_State = 1;                                       // Set state to 1
      CurHalfCycInstSOSsav.Ia = CurHalfCycSOS_SumF.Ia;      // Save the present half-cycle sums of squares
      CurHalfCycInstSOSsav.Ib = CurHalfCycSOS_SumF.Ib;
      CurHalfCycInstSOSsav.Ic = CurHalfCycSOS_SumF.Ic;
      CurHalfCycInstSOSsav.In = CurHalfCycSOS_SumF.In;
    }                                               // If below pickup level, just return
    Inst_StartupSampleCnt = 0;                      // Clear startup compensation count since only used once
  }
  else                                 // State 1
  {
    Inst_SamplePasses++;                            // Increment the sample pass count
    if (Inst_SamplePasses >= 40)                    // If 40 samples since last half-cycle anniversary,
    {                                               //   it is a new anniversary
      Inst_SamplePasses = 0;                                    // Clear the sample passcount
      // Compute the present 1/2-cyc sums of squares.  To compensate for DC offset in currents under some
      //   fault conditions, the following algorithm (taken from the NRX1150) is used:
      //     - If the new 1/2-cyc sum of squares is > 50% of the previous 1/2-cyc sum of squares, use the
      //       new 1/2-cyc sum of squares
      //     - Otherwise average the two 1/2-cyc sum of squares
      // Also compute RMS current based on these half-cycle SOS values.  These will be used for the event
      //   capture if a trip event occurs
      if (CurHalfCycSOS_SumF.Ia > (CurHalfCycInstSOSsav.Ia/2))
      {
        tempmax = CurHalfCycSOS_SumF.Ia;
      }
      else
      {
        tempmax = (CurHalfCycSOS_SumF.Ia + CurHalfCycInstSOSsav.Ia)/2;
      }
      EventCurOneCyc.Ia = sqrtf(tempmax/40);
      if (CurHalfCycSOS_SumF.Ib > (CurHalfCycInstSOSsav.Ib/2))
      {
        temp = CurHalfCycSOS_SumF.Ib;
      }
      else
      {
        temp = (CurHalfCycSOS_SumF.Ib + CurHalfCycInstSOSsav.Ib)/2;
      }
      EventCurOneCyc.Ib = sqrtf(temp/40);
      if (temp > tempmax)
      {
        tempmax = temp;
      }
      if (CurHalfCycSOS_SumF.Ic > (CurHalfCycInstSOSsav.Ic/2))
      {
        temp = CurHalfCycSOS_SumF.Ic;
      }
      else
      {
        temp = (CurHalfCycSOS_SumF.Ic + CurHalfCycInstSOSsav.Ic)/2;
      }
      EventCurOneCyc.Ic = sqrtf(temp/40);
      if (temp > tempmax)
      {
        tempmax = temp;
      }
      if (CurHalfCycSOS_SumF.In > (CurHalfCycInstSOSsav.In/2))
      {
        temp = CurHalfCycSOS_SumF.In;
      }
      else
      {
        temp = (CurHalfCycSOS_SumF.In + CurHalfCycInstSOSsav.In)/2;
      }
      EventCurOneCyc.In = sqrtf(temp/40);
      EventCurOneCycIg = 0;                     // Ig is not used in instantaneous protection
      if (temp > tempmax)
      {
        tempmax = temp;
      }
      CurHalfCycInstSOSsav.Ia = CurHalfCycSOS_SumF.Ia;          // Save present half-cycle sums of squares
      CurHalfCycInstSOSsav.Ib = CurHalfCycSOS_SumF.Ib;
      CurHalfCycInstSOSsav.Ic = CurHalfCycSOS_SumF.Ic;
      CurHalfCycInstSOSsav.In = CurHalfCycSOS_SumF.In;
      if (tempmax >= Inst_Pickup)                               // If above the pickup level..
//      if ( (tempmax >= 0) && (EV_Sum.NextEvntNdx != 523) )     // *** DAH TEST TO INSERT 500 SUMMARY EVENTS (FILL THE EVENT BUFFER)  NEED TO USE EMULATOR TO SET INST_PICKUP TO 0 AND TO GET FIRST EVENT IN
      {
        Inst_HalfCycPasses++;                                       // Inc the number of half-cycle passes
        if (Inst_HalfCycPasses > 2)                                 // If three consecutive passes, trip
        {
          //NeuTripFlg = NmaxFlg;         //          Set neutral trip flag if neutral was maximum current
          COT_AUX.COT_Code = 'I';                                   // Turn on the instantaneous cause of
          WriteCauseOfTripLEDS();                                   //   trip LED
          TRIP_LED_ON;                                              // Turn on Trip LED *** DAH  NOT SURE WHETHER THIS SHOULD BE TURNED ON HERE
          TripFlagsReset();
          Reset_ProtRelated_Alarm();
          if (FW_SimulatedTest.Enable == TRUE)   
          {  
            FW_SimulatedTest.State = 2;                             // Ending/Result state for FW Simulated test
          }
          if (HW_SecInjTest.Enable == TRUE)
          {  
            HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
          }    
          if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
              ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
          {
                 // Don't activate TA for No Trip tests
          }
          else
          {  
            TA_TRIP_ACTIVE;
          }
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
          TESTPIN_A3_HIGH;                      // *** DAH TEST FOR COLD START TESTING
#endif
          // Initialize TA active timer to about 5msec ((24 sa)/(80 sa/cyc * 60 cyc/sec) = 5msec)
          TA_Timer = 24;
          AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
          // Put an entry into the new event FIFO (for interrupts)
          IntEventBuf[IntEventInNdx].Code = TRIP_INSTANTANEOUS;
          Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
          IntEventInNdx &= 0x07;
          if (IntEventInNdx == IntEventOutNdx)
          {
            IntEventBufFull |= TRUE;
          }
//          Inst_Pickup = 4.0E9;      // *** DAH ADDED FOR TEST
          InstTripFlg = 1;
          TripReqFlg  = 1;
          BellTripFlg = 1;                            // Set general trip cause flag
          Prot_Timer = 25;                            // hold off next trip for 250ms
          if (!Trip_WF_Capture.InProg)                    // Set trip waveform capture req flag
          {
            Trip_WF_Capture.Req = TRUE;
          }
          Inst_HalfCycPasses = 0;
          Inst_State = 0;
          Prot_Enabled = FALSE;                       // Turn off protection for 250msec (Prot_Timer = 25)
          Prot_Timer = 25;                                          // *** DAH  COMMENT THIS OUT FOR DESK TESTING WITH TEST INJECTION    *** DAH 210714
        }                                                           //          AND UNCOMMENT THE LINE IN TestInj_Handler() IN INTR.C
      }                                                             //          THAT INITIALIZES THE TIMER WHEN TEST INJECTION IS TURNED OFF
      else                                            // If below pickup level, clear the pickup flag
      {                                               //   and reset the state
        InstpuFlg = 0;                                      // PickupFlags &= (~INST_PICKUP);
        Inst_State = 0;
        
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged == TRUE))
        {
          HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged = FALSE;
        }
      }
    }                                           // If not a new half-cycle anniversary, just return

  }

}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Instantaneous_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Short_Interlock_Out()
//------------------------------------------------------------------------------------------------------------
// FUNCTION:           Short Delay interlock out
//
// MECHANICS:          If CurHalfCycSOSmax >= SD_InterlockPickup, set Zout and SdintpuFlg.
//                     Note here that SD_InterlockPickup is 1.5pu because that is the lowest SD setting that an 
//                     upstream breaker can have.
//
//                     else, clear SdintpuFlg & Zin_Latched.
//
//  CAVEATS:           None
//                      
//  INPUTS:            CurHalfCycSOSmax, SD_InterlockPickup, GfpuFlg 
//                      
//  OUTPUTS:           ZSI output   
//                      
//  ALTERS:            SdintpuFlg, Zin_Latched      
//                      
//  CALLS:             None 
//
//------------------------------------------------------------------------------------------------------------

void Short_Interlock_Out(void)
{                                                
  if (CurHalfCycSOSmax >= SD_InterlockPickup)   // Above short delay interlock pickup?
  {
    SdintpuFlg = 1;                             // set short delay interlock flag                                    
    SET_ZOUT;                                   // set ZSI output pin high
  }
  else                                          // Below short delay interlock pickup?
  {
    SdintpuFlg = 0;                             // clear short delay interlock flag
    if (GfpuFlg == 0)
    {
      CLEAR_ZOUT;                               // clear ZSI output if both GfpuFlg & SdpuFlg are clear
      Zin_Latched = 0;                          // clear the latched interlock in flag
    }                                           // Zin_Latched is set in ISR according to ZIN status and is a latched flag
  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Short_Interlock_Out()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        ShortDelay_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Short Delay Protection
//
//  MECHANICS:          This subroutine performs short delay protection every sample period (208usec at
//                      60Hz) as follows:
//                      State 0:
//                        - Check the maximum 1/2-cycle current sum of squares for pickup
//                              - If max 1/2-cycle sum of squares >= pickup
//                                      - Initialize number of passes to 40 (one half-cycle)
//                                      - Initialize I2t tally register to the max 1/2-cycle sum of squares
//                                      - Save 1/2-cycle current sums of squares
//                                      - State = 1
//                              - If max current < pickup, remain in State 0
//                      State 1:
//                        - Increment the number of passes
//                        - On 1-cycle anniversary (number of passes == 120)...
//                              - Adjust the number of passes so we can fall into State 2:
//                                  - Set the number of passes back to 119                           
//                              - Increment the I2T tally register with the max 1-cycle sum of squares
//                              - Adjust the I2t tally register so we can fall into State 2:
//                                  - Subtract the square of the latest sample of the max 1-cyc SOS phase 
//                              - Fall into State 2
//                      State 2:
//                        - If in pickup (max 1-cycle sum of squares > pickup threshold)
//                              - Increment number of passes
//                              - If ZSI enabled...
//                                  - If the number of passes is greater than 160 (4 half-cycles), set the
//                                    trip flag
//                              - If Flat Short Delay Protection...
//                                  - If the number of passes is greater than the trip threshold, set the
//                                    trip flag
//                              - If I2t Short Delay Protection...
//                                  - Increment the I2T tally register with the square of the latest sample
//                                    of the max 1-cyc SOS phase 
//                                  - If the number of passes exceeds the minimum trip threshold AND the I2t
//                                    tally register exceeds the trip threshold, set the trip flag
//                                  - If the current is 8x or above, the SD protection will change from I2t
//                                    to Flat
//                        - If not in pickup, set the state to 0
//                      In State 0, the maximum of the half-cycle current sum of squares is checked for
//                      pickup.  This current is computed in the interrupt and is the instantaneous
//                      half-cycle current, calculated each sample period.  The intent is to recognize a
//                      pickup condition as quickly as possible.  This maximum current is not compensated
//                      for DC offsets; that is, no averaging is done with the previous half-cycle current
//                      sum of squares.  This is not necessary, as we are looking to enter pickup and will
//                      enter on the large half-cycle regardless of averaging - the averaging only affects a
//                      small half-cycle.  Typically, a DC offset occurs when the fault occurs, so the unit
//                      will already be in pickup (State 1) when the offset occurs.  If it is not, there
//                      still is not a problem.  If the current is large due to the DC offset, the averaging
//                      algorithm would have kept the larger value and we would enter pickup anyway (there
//                      is no difference in the operation).  If the current is small due to the offset, it
//                      wouldn't enter pickup with or without averaging, because the previous half-cycle was
//                      also below pickup (since it wasn't in pickup to begin with).
//                      In State 1, we wait for one cycle to elapse.  We then assume we are still in pickup,
//                      and increment the number of passes by one cycle (80 counts) and the I2t tally
//                      register by the max cycle's sum of squares, adjusted to allow us to share code with
//                      State 2 and just fall into this state.  State 2 consists of performing protection on
//                      a per-sample basis, so the adjustment consists of reducing the number of passes by
//                      one (since it is incremented by one in State 2), and reducing the I2t tally register
//                      by the square of the latest sample of the max 1-cycle phase (since this will be
//                      added to the tally register in State 2).
//                      As mentioned previously, in State 2, we perform the protection function on a
//                      per-sample basis.  The max 1-cycle phase is checked to determine whether we are
//                      still in pickup.  If so, depending on the protection style, either the number of
//                      passes is incremented by one pass or the I2t tally register is incremented by one
//                      sample - the sample of the present one-cycle maximum rms value.
//                      The intent of this protection algorithm is to: 1) recognize an overcurrent as
//                      quickly as possible (within 1/2-cycle), and 2) trip as precisely as possible (within
//                      one sample period).
//
//
//  CAVEATS:            There is no I2t slope for Short Delay if I4t or any of the IEEE or IEC LD slopes are selected.
//                      Short Delay will only be flat.
//
//  INPUTS:             SD_Pickup, CurHalfCycSOSmax, CurOneCycSOSmax, NewSample, SD_Slope,
//                      SD_Trip_Threshold_Flat, SD_Trip_Threshold_I2t
//                      
//  OUTPUTS:            AlarmHoldOffTmr
//                      
//  ALTERS:             SD_State, SD_Passes, SD_Tally, SDTripBucket
//                      
//  CALLS:              None
//                      
//------------------------------------------------------------------------------------------------------------

void ShortDelay_Prot(void)
{
  float CurIgOneCycSOS;
  
  //uint32_t SD_Passes_emulator;                                    // *** BP Test code for emulator

  // *** DAH  PER DISCUSSIONS WITH RANDY, TED, BERT, WE WILL USE THE ONE-CYCLE VALUES TO GENERATE THE SNAPSHOT VALUES FOR EVENTS.  WE DON'T NEED
  //   TO USE THE ACTUAL HALF-CYCLE INSTANTANEOUS AND SHORT DELAY VALUES
  // half-cyc - used for inst, short delay, ground protection
  // one-cyc used for long delay, other protections, snapshots for events, waveform for user capture
  // one-sec - used for metering (need to add filter)

  switch (SD_State)
  {
    case 0:                             // Default state is 0
    default:
      if (CurHalfCycSOSmax >= SD_HalfCycPickup) // If above pickup level..
      {        
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
        TESTPIN_A3_LOW;                      // *** DAH TEST FOR COLD START TESTING
#endif
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.SD_Trip_StartingSampleCounter_Logged == FALSE))
        {
          HW_SecInjTest.SD_Trip_StartingSampleCounter = SampleCounter;
          HW_SecInjTest.SD_Trip_StartingSampleCounter_Logged = TRUE;
        }
        // Since this is the initial entry into pickup (otherwise we would be in State 1), initiate summary
        //   and disturbance capture action if SD event captures are enabled
        if (Setpoints1.stp.SD_EventAction == 1)
        {
          // Put an entry into the new event FIFO (for interrupts)
          IntEventBuf[IntEventInNdx].Code = SDPU_ENTRY;
          Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
          IntEventInNdx &= 0x07;
          if (IntEventInNdx == IntEventOutNdx)
          {
            IntEventBufFull |= TRUE;
          }         
          Dist_Flag_SD = TRUE;                             // Initiate a disturbance capture
          // ***DAH  NOT SURE WHETHER WE WANT TO INITIATE A GOOSE GLOBAL CAPTURE
        }
        SdpuFlg = 1;   //PickupFlags |= SD_PICKUP;               // Set pickup flag
        SD_Passes = 40;                         // Initialize number of passes to one-half cycle
        SD_Tally = CurHalfCycSOSmax;            // Initialize I2t tally register to the max 1/2-cycle's SOS

        // If this is the first time in (we are closing into a fault), SD_StartupSampleCnt is 40.  This will
        //   reduce the trip time by one-half cycle (40 samples), by adjusting SD_Passes and SD_Tally
        //   accordingly
        SD_Passes += SD_StartupSampleCnt;
        // For i2T, assume max current was there for the entire startup time and add proportional amount to
        //   the bucket
        SD_Tally += (CurHalfCycSOSmax * SD_StartupSampleCnt)/40.0;

        SD_State = 1;
      }                                         // If below pickup level, just return

      SD_StartupSampleCnt = 0;                // Clear startup compensation count since only used once
      break;

    case 1:                             // State 1
      SD_Passes++;                              // Check for one-cycle anniversary
      if (SD_Passes >= 120)                     // If one cycle anniversary...
      {                                             // Set up for State 2:
        SD_Passes = 119;                                          // Set number of passes to 120 - 1
        SD_Tally += CurOneCycSOSmax - (NewSample * NewSample);    // Update SD_Tally with one cycle SOS
        SD_State = 2;                                             //   less the new sample SOS
      }
      else
      {
        break;
      }

    case 2:                             // State 2
      if (CurOneCycSOSmax >= SD_OneCycPickup)       // If above the pickup level...
//      if (CurOneCycSOSmax < SD_OneCycPickup)       // If above the pickup level...   *** DAH TEST FOR TIMING
      {
        SD_Passes++;                                    // Increment number of passes

        // BP - add ZSI/Goose-initiated
        if ((Setpoints1.stp.ZSI == 1) && (Zin_Latched == 0))   // trip initiated after two full cycles above
        {                                                      //   pickup independent of the tally value
          if (SD_Passes >= 160)                                // if it is the 4th half-cycle, trip...
          {
            COT_AUX.COT_Code = 'S';                               // Turn on short delay cause of trip LED
            WriteCauseOfTripLEDS();
            TRIP_LED_ON;                                              // Turn on Trip LED
            SD_Passes = 0;
            SD_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)                      
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
            TESTPIN_A3_HIGH;                      // *** DAH TEST FOR COLD START TESTING
#endif
            // Note, we don't need to compute event currents here because, we will have one-cycle values,
            //   since the trip time is 2 cycles
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_SHORT_DELAY;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            }
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            SdTripFlg = 1;
            TripReqFlg  = 1;
            BellTripFlg = 1;                            // Set general trip cause flag
            Prot_Enabled = FALSE;                       // Turn off protection
            Prot_Timer = 25;                            // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;               // Set trip waveform capture req flag
            }
            Dist_Flag_Cancel_SD = TRUE;                 // Cancel the disturbance capture
            //NeuTripFlg = NmaxFlg;                     // Set neutral trip flag if neutral was max current
          }
        }
        // If Flat protection or if above 8x...
        else if ((SD_Slope == SD_FLAT) || (CurOneCycSOSmax >= SD_OneCyc_8x))
        {
          if (SD_Passes >= SD_TripThresholdFlat)        // If tripping...      *** DAH  ADD COMPENSATION FOR STARTUP TIME IF FIRST TIME AFTER RESET
          {
            COT_AUX.COT_Code = 'S';                         // Turn on the short delay cause of trip LED
            WriteCauseOfTripLEDS();
            //SD_Passes_emulator = SD_Passes;                 // *** BP test code for emulator to read SD trip time
            SD_Passes = 0;
            SD_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }  
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {
              TA_TRIP_ACTIVE;
            }

            // Compute RMS current based on the one-cycle SOS values.  These will be used for the trip event
            //   capture in case the one-cycle currents haven't been computed yet
            EventCurOneCyc.Ia = sqrtf(CurOneCycSOS_SumF.Ia/80);
            EventCurOneCyc.Ib = sqrtf(CurOneCycSOS_SumF.Ib/80);
            EventCurOneCyc.Ic = sqrtf(CurOneCycSOS_SumF.Ic/80);
            EventCurOneCyc.In = sqrtf(CurOneCycSOS_SumF.In/80);
            CurIgOneCycSOS = (Setpoints1.stp.Gnd_Type == 0)
                                    ? CurOneCycSOS_SumF.Igres : CurOneCycSOS_SumF.Igsrc;
            EventCurOneCycIg = sqrtf(CurIgOneCycSOS/80);
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
            TESTPIN_A3_HIGH;                      // *** DAH TEST FOR COLD START TESTING
#endif
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_SHORT_DELAY;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            }
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            SdTripFlg = 1;
            TripReqFlg  = 1;
            BellTripFlg = 1;                            // Set general trip cause flag
            Prot_Enabled = FALSE;                       // Turn off protection
            Prot_Timer = 25;                            // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;               // Set trip waveform capture req flag
            }
            Dist_Flag_Cancel_SD = TRUE;                 // Cancel the disturbance capture
            //NeuTripFlg = NmaxFlg;                     // Set neutral trip flag if neutral was max current
          }
        }
        else                                        // Otherwise I2t protection...
        {
          SD_Tally += (NewSample * NewSample);          // Update SD_Tally with the latest sample SOS
          if ( (SD_Passes >= SD_MIN_PASSES_FOR_TRIP)    // Cannot trip unless in pickup for a minimum
            && (SD_Tally >= SD_TripThresholdI2t) )      //   number of passes and I2t tally exceeded
          {
            COT_AUX.COT_Code = 'S';                         // Turn on the short delay cause of trip LED
            WriteCauseOfTripLEDS();
            //SD_Passes_emulator = SD_Passes;                 // *** BP test code for emulator to read SD trip time
            SD_Passes = 0;                              // Not necessary since initialized in state 0
            SD_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                         // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)                      
            {  
              HW_SecInjTest.State = 2;                            // Ending/Result state for HW Sec Inj test
            }
                        if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }

            // Compute RMS current based on the one-cycle SOS values.  These will be used for the trip event
            //   capture in case the one-cycle currents haven't been computed yet
            EventCurOneCyc.Ia = sqrtf(CurOneCycSOS_SumF.Ia/80);
            EventCurOneCyc.Ib = sqrtf(CurOneCycSOS_SumF.Ib/80);
            EventCurOneCyc.Ic = sqrtf(CurOneCycSOS_SumF.Ic/80);
            EventCurOneCyc.In = sqrtf(CurOneCycSOS_SumF.In/80);
            CurIgOneCycSOS = (Setpoints1.stp.Gnd_Type == 0)
                                    ? CurOneCycSOS_SumF.Igres : CurOneCycSOS_SumF.Igsrc;
            EventCurOneCycIg = sqrtf(CurIgOneCycSOS/80);
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
            TESTPIN_A3_HIGH;                      // *** DAH TEST FOR COLD START TESTING
#endif
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_SHORT_DELAY;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            }
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            SdTripFlg = 1;
            TripReqFlg  = 1;
            BellTripFlg = 1;                            // Set general trip cause flag
            Prot_Enabled = FALSE;                       // Turn off protection
            Prot_Timer = 25;                            // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;               // Set trip waveform capture req flag
            }
            //NeuTripFlg = NmaxFlg;                     // Set neutral trip flag if neutral was max current
            Dist_Flag_Cancel_SD = TRUE;                 // Cancel the disturbance capture
          }
        }
      }
      else                                      // If not in pickup, clear the pickup flag and set next the
      {                                         //   state to 0 to start over
        SdpuFlg = 0;  //PickupFlags &= (~SD_PICKUP);
        if (SD_Passes >= (80 * 8))              // Must be in pickup for at least 8 cycles to do a
        {                                       //   disturbance capture
          // Compute how close we came to tripping.  There are three cases:
          //   (1) ZSI - we aren't doing a disturbance capture (too short)
          //   (2) Flat - this is just the number of passes divided by the threshold
          //   (3) I2t - this is the 
          if ((Setpoints1.stp.ZSI == 1) && (Zin_Latched == 0))
          {
            SDTripBucket = 0;
          }
          else if ((SD_Slope == SD_FLAT) || (CurOneCycSOSmax >= SD_OneCyc_8x))
          {
            SDTripBucket = (uint16_t)((SD_Passes * 1000)/SD_TripThresholdFlat);
          }
          else
          {
            SDTripBucket = (uint16_t)((SD_Tally * 1000.0)/SD_TripThresholdI2t);
            // If SD_Tally reached the threshold, we didn't trip because the minimum passes wasn't reached,
            //   so use the number of passes to compute how close we came to tripping
            if (SDTripBucket >= 1000.0)
            {
              SDTripBucket = (uint16_t)((SD_Passes * 1000)/SD_MIN_PASSES_FOR_TRIP);
            }
          }
          Dist_Flag_SD = FALSE;                 // Clear the Dist_Flag so the disturbance capture is
        }                                       //   completed (if one had been started)
        else
        {
          Dist_Flag_Cancel_SD = TRUE;           // Otherwise cancel the disturbance capture
        }
        SD_State = 0;
        
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.SD_Trip_StartingSampleCounter_Logged == TRUE))
        {
          HW_SecInjTest.SD_Trip_StartingSampleCounter_Logged = FALSE;
        }
      }
      break;

  }

}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             ShortDelay_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        LongDelay_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Long Delay Protection
//
//  MECHANICS:          This subroutine performs long delay protection every cycle (16.6ms at 60Hz)
//                      as follows:
//                      State 0:
//                        - Check the maximum 1-cycle current sum of squares for pickup
//                              - If max 1-cycle sum of squares >= pickup
//                                      - Initialize tally register to the max 1-cycle sum of squares
//                                      - Save 1-cycle current sums of squares
//                                      - State = 1
//                              - If max current < pickup, remain in State 0
//                              - If the tally is >0, clear it if thermal memory is off or if the LD slope
//                                is I05T or I1T. (Per Jim Lagree, I05T and I1T always get cleared)
//                              - if the tally is >0, thermal memory is on, and the LD slope is I2T or I4T,
//                                decrement the tally at a decrement rate of 36x the LD Time setting going
//                                from 100% to 0%. (Keep in mind that when we trip, the tally will start
//                                decrementing from 85%, not 100%)
//                      State 1:
//                        - If in pickup (max 1-cycle sum of squares > pickup threshold)
//                              - Increment the tally register with the appropriate increment amount based
//                                 on the LD slope (I05T, I1T, I2T, or I4t)
//                              - If the number of passes exceeds the minimum trip threshold AND the tally
//                                register exceeds the trip threshold, set the trip flag
//                        - If not in pickup, set the state to 2
//                      State 2:
//                        - Check for a second consecutive max1 1-cycle sum of squares below pickup.  If so,
//                          set the state to 0.  If it is above pickup, set the state to 1.
//
//                      In State 0, the maximum of the one-cycle current sum of squares is checked for
//                      pickup.  This current is computed in the interrupt and the accumulating
//                      one-cycle current is calculated each sample period. The current samples are digitally
//                      integrated and then converted to engineering units.  After 80 samples have been
//                      processed in the interrupt routine, the OneCycAnniv flag is set.
//                      The Long Delay Protection routine is called from the main loop every one-cycle anniversary.
//                      In State 1, we wait for one cycle to elapse.  We then assume we are still in pickup,
//                      and increment the number of passes by one cycle (80 counts) and the tally register
//                      by the appropriate amount based on the LD slope.  State 1 consists of performing protection on
//                      a per-cycle basis.
//
//
//  CAVEATS:            None
//
//  INPUTS:             LD_OneCycPickup, CurOneCycSOSmax, NewSample, LD_Style, LD_Trip_Threshold, LD_Slope,
//                      LD_TallyDecr, Setpoints1.stp.ThermMem
//
//  OUTPUTS:            AlarmHoldOffTmr
//
//  ALTERS:             LD_State, LD_Passes, LD_Tally, LD_TallyIncr
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void LongDelay_Prot(void)
{
  uint32_t temp2[2];
  float temp;

  if (LD_Slope == LD_I05T)                      // Calculate Incr and Decr values based on LD slope
  {
    temp = sqrtf(CurOneCycSOSmax);
    LD_TallyIncr = sqrtf(temp);
  }
  else if (LD_Slope == LD_I1T)
  {
    LD_TallyIncr = sqrtf(CurOneCycSOSmax);
  }
  else if (LD_Slope == LD_I4T)
  {
    LD_TallyIncr = CurOneCycSOSmax * CurOneCycSOSmax;
    LD_TallyDecr = 36 * 80 * 80 * Ir * Ir * Ir * Ir;
  }
  else                // LD_I2T
  {
    LD_TallyIncr = CurOneCycSOSmax;
    LD_TallyDecr = 80 * Ir * Ir;
  }

  switch (LD_State)
  {
    case 0:                             // Default state is 0
    default:
    if (CurOneCycSOSmax >= LD_OneCycPickup)   // If above pickup level..
//      if (CurOneCycSOSmax < LD_OneCycPickup)  // If above pickup level..  *** DAH TEST FOR TIMING
    {
      // Since this is the initial entry into pickup (otherwise we would be in State 1), initiate summary
      //   and disturbance capture action if LD event captures are enabled
      if (Setpoints1.stp.LD_EventAction == 1)
      {
        // Save the EID of the event that started the disturbance capture, and insert an entry event
        sListOfDist[DS_LD_PICKUP].EIDofStartEvnt = InsertNewEvent(LDPU_ENTRY);
        Dist_Flag |= DS_LD;                              // Initiate a disturbance capture
        // Set the flag to initiate a GOOSE capture message if one is not already in progress.
        //   Goose_Capture_Code stores which process initiated the capture, so that the command to terminate
        //   the capture will only come from the same process.
        // Note, the GOOSE Rx subroutine in the sampling interrupt could also set a capture flag. It
        //   shouldn't matter if this flag is missed, because we only do one extended capture anyway, so we
        //   don't need to disable interrupts around this statement
        if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code > DS_LAST)
          && (Setpoints13.stp.GC_GlobalCapEnable & 0x08) )  // *** DAH  MAKE "0x08" A CONSTANT
        {
          PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 1;
          DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;          // *** DAH  WE WILL LIKELY NEED TO INCLUDE WHICH PARAMETER TO MONITOR FOR THE DISTURBANCE CAPTURE!
          Goose_Capture_Code = DS_LD_PICKUP;                      //          MAYBE ALSO SAY WHETHER DO DISTURBANCE CAP + EXTENDED CAP OR JUST ONE OF THE TWO
        }
      }
      TripFlagsReset();                                   // to allow alarm to be logged
      LdPuAlmFlg = 1;      //PickupFlags |= LD_PICKUP;               // Set pickup flag
      LD_Passes = 1;                          // Initialize number of passes to one cycle
      LD_Tally += LD_TallyIncr;               // Initialize tally register to the max 1-cycle's SOS (to the correct power for the LD slope)
      LD_State = 1;
    }
    else                                      // If below pickup level, either clear or decrement LD_Tally
    {
      LdPuAlmFlg = 0;     //PickupFlags &= (~LD_PICKUP);
      TimeToTripFlg = 0;                     // Not in pickup so don't display Time to Trip
      if ((Setpoints1.stp.ThermMem == 0) || (LD_Slope == LD_I05T) || (LD_Slope == LD_I1T))
      {
        LD_Tally = 0.0;
        LD_Passes = 0;
        ThermCapacityFlg = 0;
      }
      else                                    // Decrement LD_Tally for I2T and I4T if Thermal Memory is on
      {
        LD_Tally -= LD_TallyDecr;
        ThermCapacityFlg = 1;

        if (LD_Tally < 0)                      // Limit LD_Tally to zero.
        {
          LD_Tally = 0.0;
          LD_Passes = 0;
          ThermCapacityFlg = 0;
        }
       }
    }
    break;



    case 1:                             // State 1
      if (CurOneCycSOSmax >= LD_OneCycPickup)         // If above the pickup level...
      {
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged == FALSE))
        {
          HW_SecInjTest.LD_Trip_StartingSampleCounter = SampleCounter;
          HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged = TRUE;
        }
        
        LD_Passes++;                                  // Increment number of passes

        PKEOverload_Warning();                        // Calculate the time to trip
        TimeToTripFlg = 1;                            // We're in pickup so display Time to Trip

        LD_Tally += LD_TallyIncr;                     // Update LD_Tally with the latest SOS (to the correct power for the LD slope)
        if ( (LD_Passes >= LD_MIN_PASSES_FOR_TRIP)    // Cannot trip unless in pickup for a minimum
          && (LD_Tally >= LD_TripThreshold) )         //   number of passes and I2t tally exceeded
        {
          // Clamp LD_Tally to the trip threshold value so thermal capacity never exceeds 100%
          LD_Tally = LD_TripThreshold;
          COT_AUX.COT_Code = 'L';                     // Turn on the long delay cause of trip LED
          WriteCauseOfTripLEDS();
          TRIP_LED_ON;                                              // Turn on Trip LED
          LD_Passes = 0;
          LD_State = 0;
          TripFlagsReset();
          Reset_ProtRelated_Alarm();
          if (FW_SimulatedTest.Enable == TRUE)                      
          {  
            FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
          }
          if (HW_SecInjTest.Enable == TRUE)
          {  
            HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
          }
          if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
              ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
          {
                 // Don't activate TA for No Trip tests
          }
          else
          {  
            TA_TRIP_ACTIVE;
          }
          // Put an entry into the new event FIFO
          InsertNewEvent(TRIP_LONG_DELAY);
          TA_Timer = 24;
          AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
          LdTripFlg = 1;
          TripReqFlg  = 1;
          BellTripFlg = 1;                              // Set general trip cause flag
          Prot_Enabled = FALSE;                         // Turn off protection
          Prot_Timer = 25;                              // Hold off next trip for 250ms
          if (!Trip_WF_Capture.InProg)
          {
            Trip_WF_Capture.Req = TRUE;                 // Set trip waveform capture req flag
          }
          // If LD event captures are enabled, cancel the disturbance capture since we tripped, and send a
          //   GOOSE capture message to end the capture if LD pickup initiated it
          if (Setpoints1.stp.LD_EventAction == 1)
          {
            Dist_Flag_Cancel |= DS_LD;      // Trip event will set the cancel flags anyway
            if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code == DS_LD_PICKUP)
              && (Setpoints13.stp.GC_GlobalCapEnable & 0x08) )  // *** DAH  MAKE "0x08" A CONSTANT
            {
              PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
              DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
              Goose_Capture_Code = DS_LAST + 1;
            }
          }
          //NeuTripFlg = NmaxFlg;                       // Set neutral trip flag if neutral was max current
        }
      }
      else                                      // If not in pickup, set the state to 2
      {                                         //   where we wait for one more reading below pickup
        LD_State = 2;                           //   before going back to state 0
      }
      break;



    case 2:                             // State 2 - check for a second consecutive value below pickup
      if (CurOneCycSOSmax >= LD_OneCycPickup)         // If above the pickup level...
      {
        LD_Passes++;                                  // Increment number of passes
        LD_Tally += LD_TallyIncr;                     // Update LD_Tally with the latest SOS (to the correct power for the LD slope)
        LD_State = 1;
      }
      else                                      // If not in pickup, clear the pickup flag and set next the
      {                                         //   state to 0 to start over
        LdPuAlmFlg = 0;         //PickupFlags &= (~LD_PICKUP);
        // If LD event captures are enabled, end the disturbance capture since we are out of pickup, and
        //   send a GOOSE capture message to end the capture if we initiated it
        if (Setpoints1.stp.LD_EventAction == 1)
        {
          // Clear the Dist_Flag so the disturbance capture is completed (if one had been started)
          Dist_Flag &= (DS_LD ^ 0xFFFFFFFF);
          if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code == DS_LD_PICKUP)
            && (Setpoints13.stp.GC_GlobalCapEnable & 0x08) )  // *** DAH  MAKE "0x08" A CONSTANT
          {
            PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
            DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
            Goose_Capture_Code = DS_LAST + 1;
          }
        }
        LD_State = 0;
        
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged == TRUE))
        {
          HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged = FALSE;
        }
      }
      break;

  }

  LD_Bucket =  (LD_Tally / LD_TripThreshold) * 100;      // LD_Bucket is the Thermal Capacity%
  
  // 4) Store the new LD_Tally bucket percentage in FRAM
  LD_Bucket =  (LD_Tally / LD_TripThreshold) * 100;  
  
  temp2[0] = (uint16_t) LD_Bucket;
  temp2[1] = (uint16_t)(~temp2[0]);

  FRAM_Write(DEV_FRAM2, TM_CAPACITY, 2, (uint16_t *)(&temp2[0]));
  FRAM_Write(DEV_FRAM2, TM_CAPACITY_COMP, 2, (uint16_t *)(&temp2[1]));

}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             LongDelay_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Long_IEE_IEC_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Long Delay IEEE and IEC Protection.
//
//  MECHANICS:          Here are the following overload curve selections:
//
//                            LD_Slope          CURVE CHARACTERISTIC     P         A       B
//
//                         IEEE CURVES
//                                LD_IEEE_MI     MODERATELY INVERSE      0.02    0.0515  0.114
//                                LD_IEEE_VI     VERY INVERSE            2       19.61   0.491
//                                LD_IEEE_EI     EXTREMELY INVERSE       2       28.2    0.1217
//
//                         IEC CURVES
//                                LD_IEC_A       NORMAL INVERSE          0.02    0.14    0
//                                LD_IEC_B       VERY INVERSE            1       13.5    0
//                                LD_IEC_C       EXTREMELY INVERSE       2       80      0
//
//
//                         If CurOneCycSOSmax >= LD_OneCycPickup, in pickup...
//                            - Set LdpuFlg & increment long delay pickup count
//
//                            - For I^2t, Calculate temp32= Imax_Sum64/256 - IPUSq_P, limit zero.
//                                  Calculate Ltally = Ltally + temp32.
//
//                            - For It, Calculate i02 = Sqrt(&Imax_Sum64)
//                                  Calculate   temp32 = i02 - IPU_P, limit zero.
//                                  Calculate Ltally = Ltally + temp32.
//
//                            - For I^0.02t, Calculate i02= (&Imax_Sum64)^0.02
//                                  Calculate   temp32 = i02 - IPU02_P, limit zero.
//                                  Calculate Ltally = Ltally + temp32.
//                            - If Ltally >= the Ptally_Trip trip value, enter the B delay
//                                  timeout routine.
//                         If below pickup...
//                            - Reset the long delay pickup flag.
//                            - Reset the long delay pickup count.
//                            - Reset the B delay counter.
//                            - Reset the B delay flag.
//                            - No reset characteristic is implemented.
//
//                      Ref NRX1150 Prot.c
//
//  CAVEATS:            There is no I2t slope for Short Delay if any of these IEEE or IEC slopes are selected. Short will only be flat.
//
//                      3505 cycles + 2 calls to Latched_Leds() to trip on IECB, It, Ld_Slp=Pslope=0x02
//                      This case calls sqrt32() which take the longest.
//
//  INPUTS:             Parameters: None.
//                      Memory:  LD_Slope, LD_OneCycPickup, CurOneCycSOSmax, LD_Tally, Ptally_Trip,
//                               TestMode, PBldFlg, LdpuFlg, IPUSq_P, IPU_P, IPU02_P,
//
//  OUTPUTS:            TripReqFlg , LdTripFlg, PIEEIECTripFlg, AlarmHoldOffTmr
//
//  ALTERS:             LD_Passes, LdpuFlg, LdTripFlg, PBldFlg, LD_Tally, PIEEIECTripFlg, NeuTripFlg
//
//  CALLS:              TripFlagsReset(), Latched_Leds(), Get_I02(), Sqrt32()
//
//------------------------------------------------------------------------------------------------------------
//
void Long_IEE_IEC_Prot(void)
{
  volatile float temp32;                                   // Temps.
  float i02;                                      // = 1000H * ((ImaxSum64/256)^2)^0.01


                                                  // In pickup?
  if (CurOneCycSOSmax >= LD_OneCycPickup)         // If above the pickup level...
  {                                               // Yes - In pickup
                                                  //    To avoid tail activation, do not modify existing trip flags or leds
                                                  //    if the breaker is open or on the first and second passes above ldpu.
                                                  //    Otherwise, reset historic trip flags, and service the leds.
                                                  //    Also, from ACB1250P main.abs pg 109, if TestMode==1, do not erase
                                                  //    prior trip flags.

    if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged == FALSE))
    {
      HW_SecInjTest.LD_Trip_StartingSampleCounter = SampleCounter;
      HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged = TRUE;
    }

    LD_Passes++;                                  //    Increment pu count & prevent rollover

    PKEOverload_Warning();                        // Calculate the time to trip
    TimeToTripFlg = 1;                            // We're in pickup so display Time to Trip

    TripFlagsReset();                             // to allow alarm to be logged
    LdPuAlmFlg = 1;                               // Set pickup flag

    
    if  (PBldFlg == 0)                            //    If the B delay is in progress, skip to TIMOLD aka B_Delay. ref pg 6.
    {                                             //    Otherwise, do "A" delay timeout.
      switch ( LD_Slope )
       {
         case LD_IEC_B:                          //    IECB SLOPE = it
                            
            LD_TallyIncr = 80 * (sqrtf(CurOneCycSOSmax/80) - Ir);               //       factor in the 80 samples/cycle
            LD_TallyIncr = ( LD_TallyIncr < 0 ) ? 0 : LD_TallyIncr;             //       limit subtraction to zero
            break;     

         case LD_IEEE_MI:                             //    MOD SLOPE= i^0.02t
         case LD_IEC_A:                               //    IECA SLOPE= i^0.02t
                                                      //       i02 returns with (CurOneCycSOSmax)^0.01 but the factor of 80 samples is removed first 
                                                 
            i02 = pow((CurOneCycSOSmax/80), 0.01);    //       raise to power of 0.01 because we are starting with a squared value
            LD_TallyIncr = 80 * (i02 - Ir_02);        //       factor in the 80 samples/cycle
            LD_TallyIncr = ( LD_TallyIncr < 0 ) ? 0 : LD_TallyIncr;     // limit subtraction to zero
            break;         

         case LD_IEEE_VI:
         case LD_IEEE_EI:                                //  (IEEE_VI,IEEE_EI) use i^2t.
         case LD_IEC_C:
                                                 //       limit subtraction to zero
            LD_TallyIncr = CurOneCycSOSmax - 80*(Ir * Ir);    // because 80 samples/cycle
            LD_TallyIncr = ( LD_TallyIncr < 0 ) ? 0 : LD_TallyIncr;
            break;
            
            
         default:                                //    All others (IEEE_VI,IEEE_EI) use i^2t.
                                                 //       limit subtraction to zero
            LD_TallyIncr = CurOneCycSOSmax - 80*(Ir * Ir);    // because CurOneCycSOSmax is 80 samples/cycle
            LD_TallyIncr = ( LD_TallyIncr < 0 ) ? 0 : LD_TallyIncr;
            break;  
            
         }                                       // end of switch

         LD_Tally += LD_TallyIncr;

         if (LD_Tally > PA_TripThreshold)
         {
           PBldFlg = 1;                          // set flag to enter the B_Delay timeout routine
         }
      }                                          // end of "if ( PBldFlg == 0 )"

      if (PBldFlg == 1)                          // The A delay timeout is complete.
      {                                                    //    Conduct the B delay timeout...
        if ( PB_Passes++ >= PB_TripThreshold )
        {                                                  //       Both the A and B delay timeouts are complete.
                                                           //       If TestMode==0, erase prior trip flags
          LdTripFlg = 1;                           
          COT_AUX.COT_Code = 'L';                          // Turn on the long delay cause of trip LED
          WriteCauseOfTripLEDS();
          TRIP_LED_ON;                                              // Turn on Trip LED
          LD_Passes = 0;
          PB_Passes = 0;
          PBldFlg = 0;
          LD_State = 0;
          TripFlagsReset();
          Reset_ProtRelated_Alarm();
          if (FW_SimulatedTest.Enable == TRUE)                      
          {  
            FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
          }
          if (HW_SecInjTest.Enable == TRUE)
          {  
            HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
          }
          if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
              ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
          {
                 // Don't activate TA for No Trip tests
          }
          else
          {  
            TA_TRIP_ACTIVE;
          }
          // Put an entry into the new event FIFO (for interrupts)
          IntEventBuf[IntEventInNdx].Code = TRIP_LONG_DELAY;
          Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
          IntEventInNdx &= 0x07;
          if (IntEventInNdx == IntEventOutNdx)
          {
            IntEventBufFull |= TRUE;
          }   
          TA_Timer = 24;          
          AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
          LdTripFlg = 1;
          TripReqFlg  = 1;
          BellTripFlg = 1;                              // Set general trip cause flag
          Prot_Enabled = FALSE;                         // Turn off protection
          Prot_Timer = 25;                              // Hold off next trip for 250ms
          if (!Trip_WF_Capture.InProg)
          {
            Trip_WF_Capture.Req = TRUE;                 // Set trip waveform capture req flag
          }
          //NeuTripFlg = NmaxFlg;                       // Set neutral trip flag if neutral was max current
        }
      }                                          // End of "if ( PBldFlg == 1 )"
   }
   else
   {                                                    // Below pickup
     LdPuAlmFlg = 0;                                    // reset the long delay pickup flag.
     LD_Passes = 0;                                     // reset the long delay pickup count.
     PB_Passes = 0;
     PBldFlg = 0;                                       // reset the B delay flag.
     LD_Tally = 0;                                      // no reset characteristic is implemented.
     TimeToTripFlg = 0;                                 // Not in pickup so don't display Time to Trip
     
     if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged == TRUE))
     {
       HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged = FALSE;
     }
   }


}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Long_IEE_IEC_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Ground_Fault_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Ground Fault Protection.
//
//
//  MECHANICS:          This function performs ground fault protection every sample period (208usec at 60Hz).
//
//                      If above pickup (ig_sum32 >= Gfpu_Val) set zone out pin & GfpuFlg.
//                      With no interlock in, trip after 4 successive passes.
//
//                      With an interlock in:
//                         For a flat response or i2t response above 0.625pu, increment the number of passes
//                         For an i2t response below 0.625pu, calculate GF_Tally = GF_Tally plus the latest sample squared.
//                            If GF_Tally >= the tally trip value, set TripReqFlg  and GndTripFlg.
//
//                      For ratings < 1200A, use the existing rating and per unit value
//                      For ratings > 1200A, use 1200A as the scaling factor.  Therefore, change the scaling to
//                          (1200A/rating)^2
//                             Note: This scaling is only for UL/ANSI and Residual Ground
//
//                      On trip a memory effect "cooling" is implemented by loading GF_Tally with the trip
//                      threshold value.  Below pickup, GF_Tally is decreased by 0.25pu each pass.
//
//  CAVEATS:            Called from DMA1_Stream0_IRQHandler() in Intr.c if Setpoints1.stp.Gf_T_A_OFF is set
//                      to 0 (trip)
//
//
//  INPUTS:             Setpoints1.stp.Gnd_Type: [0=residual, 1=source]
//                      Setpoints1.stp.Gf_Slp: [0=flat, 1=I2t]
//                      Setpoints1.stp.Gf_Pu: [0.2 to 1.0, step=0.01]
//                      Setpoints1.stp.Gf_Tt: [Flat: 0.05 to 1.0, step=0.01] sec
//                                            [I2t:  0.1 to 1.0, step=0.01] sec
//                      Setpoints1.stp.ThermMem: [0=off, 1=on]
//                      GF_HalfCycPickup, GF_OneCyc_0p625, GF_TallyDecr
//                      GF_Tally, GF_TripThresholdFlat, GF_TripThresholdI2t
//                      CurHalfCycSOS_SumF.Igres, CurHalfCycSOS_SumF.Igsrc
//                      CurOneCycSOS_SumF.Igres, CurOneCycSOS_SumF.Igsrc
//                      Igres, AFE_new_samples[4]
//
//  OUTPUTS:            GndTripFlg, TripReqFlg, ZOUT, GFTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             GfpuFlg
//
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void Ground_Fault_Prot(void)
{
  float CurIgHalfCycSOS;
  float CurIgOneCycSOS;
  float NewSampleIg;
  
  //uint32_t GF_Passes_emulator;          //*** BP test
  
                                                // do GF protection on either residual ground or source ground
  CurIgHalfCycSOS = (Setpoints1.stp.Gnd_Type == 0) ? CurHalfCycSOS_SumF.Igres : CurHalfCycSOS_SumF.Igsrc;
  CurIgOneCycSOS = (Setpoints1.stp.Gnd_Type == 0) ? CurOneCycSOS_SumF.Igres : CurOneCycSOS_SumF.Igsrc;
  NewSampleIg = (Setpoints1.stp.Gnd_Type == 0) ? Igres_Protsample : AFE_new_samples[4];


  switch (GF_State)
  {
    case 0:                             // Default state is 0
    default:
      if (CurIgHalfCycSOS >= GF_HalfCycPickup) // If above pickup level..
      {
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.GF_Trip_StartingSampleCounter_Logged == FALSE))
        {
          HW_SecInjTest.GF_Trip_StartingSampleCounter = SampleCounter;
          HW_SecInjTest.GF_Trip_StartingSampleCounter_Logged = TRUE;
        }
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
        TESTPIN_A3_LOW;                      // *** DAH TEST FOR COLD START TESTING
#endif
        
        GfpuFlg = 1;   //PickupFlags |= GF_PICKUP;               // Set pickup flag
        GF_Passes = 40;                         // Initialize number of passes to one-half cycle
        GF_Tally = CurIgHalfCycSOS;             // Initialize I2t tally register to the max 1/2-cycle's SOS
        Dist_Flag_GF = TRUE;                    // Initiate a disturbance capture

        // If this is the first time in (cold start closing into a fault), GF_StartupSampleCnt will be
        //   40, containing the time (in number of passes; 1 pass = 208usec) that it took to start up (about
        //   one-half cycle)
        // Reduce the trip time by this amount, by adjusting GF_Passes and GF_Tally accordingly
        GF_Passes += GF_StartupSampleCnt;
        // For i2T, assume max current was there for the entire startup time and add proportional amount to
        //   the bucket
        GF_Tally += (CurHalfCycSOSmax * GF_StartupSampleCnt)/40.0;

        GF_State = 1;
      }
      else                                      // If below pickup level, either clear or decrement GF_Tally
      {
        GfpuFlg = 0;   //PickupFlags &= (~GF_PICKUP);

        if (Setpoints1.stp.GF_ThermalMem == 0)
        {
          GF_Tally = 0.0;
          GF_Passes = 0;
        }
        else                                    // Decrement GF_Tally Thermal Memory is on
        {
          GF_Tally -= GF_TallyDecr;

          if (GF_Tally < 0)                      // Limit GF_Tally to zero.
          {
            GF_Tally = 0.0;
            GF_Passes = 0;
          }
        }
      }
      GF_StartupSampleCnt = 0;                // Clear startup compensation count since only used once
      break;

    case 1:                             // State 1
      GF_Passes++;                              // Check for one-cycle anniversary
      if (GF_Passes >= 120)                     // If one cycle anniversary...
      {                                             // Set up for State 2:
        GF_Passes = 119;                                              // Set number of passes to 120 - 1
        GF_Tally += CurIgOneCycSOS - (NewSampleIg * NewSampleIg);     // Update GF_Tally with one cycle SOS
        GF_State = 2;                                                 //   less the new sample SOS
        // If still in pickup, enter a pickup event.  Note, this is placed after the one-cycle anniversary
        //   rather than at the half-cycle anniversary in State 0, because I was afraid we would get too
        //   many events due to noise
        if (CurIgOneCycSOS >= GF_OneCycPickup)
        {
          // Put an entry into the new event FIFO (for interrupts)
          IntEventBuf[IntEventInNdx].Code = GF_PICKUP;
          Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
          IntEventInNdx &= 0x07;
          if (IntEventInNdx == IntEventOutNdx)
          {
            IntEventBufFull |= TRUE;
          }         
        }
      }
      else
      {
        break;
      }

    case 2:                             // State 2
      if (CurIgOneCycSOS >= GF_OneCycPickup)            // If above the pickup level...
      {
        GF_Passes++;                                    // Increment number of passes       
        if (Setpoints1.stp.GF_ZSI == 1)
        {
          SET_ZOUT;
        }
 
        // BP - add ZSI/Goose-initiated
        if ((Setpoints1.stp.GF_ZSI == 1) && (Zin_Latched == 0))   // trip initiated after two full cycles above
        {                                                      //   pickup independent of the tally value
          if (GF_Passes >= 160)                                // if it is the 4th half-cycle, trip...
          {
            COT_AUX.COT_Code = 'G';                               // Turn on ground fault cause of trip LED
            WriteCauseOfTripLEDS();
            TRIP_LED_ON;                                          // Turn on Trip LED
            GF_Passes = 0;
            GF_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }             
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }
#ifndef ENABLE_GOOSE_COMM_SPEED_TEST
        TESTPIN_A3_HIGH;                      // *** DAH TEST FOR COLD START TESTING
#endif
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_GROUND_FAULT;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            }            
            GndTripFlg = 1;
            TripReqFlg  = 1;
            BellTripFlg = 1;                             // Set general trip cause flag
            Prot_Enabled = FALSE;                        // Turn off protection
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;                // Set trip waveform capture req flag
            }
            Prot_Timer = 25;                             // Hold off next trip for 250ms
            //NeuTripFlg = NmaxFlg;                      // Set neutral trip flag if neutral was max current
            Dist_Flag_Cancel_GF = TRUE;                  // Cancel the disturbance capture
          }
        }
        // If Flat protection or if above 0.625x...
        else if ((GF_Slope == GF_FLAT) || (CurIgOneCycSOS >= GF_OneCyc_0p625))
        {
          if (GF_Passes >= GF_TripThresholdFlat)        // If tripping...      *** DAH  ADD COMPENSATION FOR STARTUP TIME IF FIRST TIME AFTER RESET
          {
            COT_AUX.COT_Code = 'G';                         // Turn on the short delay cause of trip LED
            WriteCauseOfTripLEDS();
            //GF_Passes_emulator = GF_Passes;                 //*** BP test to measure trip time
            GF_Passes = 0;
            GF_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }             
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }

            // Compute RMS current based on the one-cycle SOS values.  These will be used for the trip event
            //   capture in case the one-cycle currents haven't been computed yet
            EventCurOneCyc.Ia = sqrtf(CurOneCycSOS_SumF.Ia/80);
            EventCurOneCyc.Ib = sqrtf(CurOneCycSOS_SumF.Ib/80);
            EventCurOneCyc.Ic = sqrtf(CurOneCycSOS_SumF.Ic/80);
            EventCurOneCyc.In = sqrtf(CurOneCycSOS_SumF.In/80);
            EventCurOneCycIg = sqrtf(CurIgOneCycSOS/80);

            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_GROUND_FAULT;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            } 
            GndTripFlg = 1;
            BellTripFlg = 1;                             // Set general trip cause flag
            TripReqFlg = 1;                              // *** BP may not need this
            Prot_Enabled = FALSE;                        // Turn off protection
            Prot_Timer = 25;                             // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;                // Set trip waveform capture req flag
            }
            //NeuTripFlg = NmaxFlg;                      // Set neutral trip flag if neutral was max current
            Dist_Flag_Cancel_GF = TRUE;                  // Cancel the disturbance capture
          }
        }
        else                                        // Otherwise I2t protection...
        {
          GF_Tally += (NewSampleIg * NewSampleIg);      // Update GF_Tally with the latest sample SOS
          if ( (GF_Passes >= GF_MIN_PASSES_FOR_TRIP)    // Cannot trip unless in pickup for a minimum
            && (GF_Tally >= GF_TripThresholdI2t) )      //   number of passes and I2t tally exceeded
          {
            COT_AUX.COT_Code = 'G';                         // Turn on the ground fault cause of trip LED
            WriteCauseOfTripLEDS();
            //GF_Passes_emulator = GF_Passes;                 //*** BP test to measure trip time
            GF_Passes = 0;
            GF_State = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            if (FW_SimulatedTest.Enable == TRUE)                      
            {  
              FW_SimulatedTest.State = 2;                            // Ending/Result state for FW Simulated test
            }
            if (HW_SecInjTest.Enable == TRUE)
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }             
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }

            // Compute RMS current based on the one-cycle SOS values.  These will be used for the trip event
            //   capture in case the one-cycle currents haven't been computed yet
            EventCurOneCyc.Ia = sqrtf(CurOneCycSOS_SumF.Ia/80);
            EventCurOneCyc.Ib = sqrtf(CurOneCycSOS_SumF.Ib/80);
            EventCurOneCyc.Ic = sqrtf(CurOneCycSOS_SumF.Ic/80);
            EventCurOneCyc.In = sqrtf(CurOneCycSOS_SumF.In/80);
            EventCurOneCycIg = sqrtf(CurIgOneCycSOS/80);

            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            // Put an entry into the new event FIFO (for interrupts)
            IntEventBuf[IntEventInNdx].Code = TRIP_GROUND_FAULT;
            Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
            IntEventInNdx &= 0x07;
            if (IntEventInNdx == IntEventOutNdx)
            {
              IntEventBufFull |= TRUE;
            } 
            GndTripFlg = 1;
            BellTripFlg = 1;                             // Set general trip cause flag
            TripReqFlg = 1;                              // *** BP may not need this
            Prot_Enabled = FALSE;                        // Turn off protection
            Prot_Timer = 25;                             // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)
            {
              Trip_WF_Capture.Req = TRUE;                // Set trip waveform capture req flag
            }
            //NeuTripFlg = NmaxFlg;                      // Set neutral trip flag if neutral was max current
            Dist_Flag_Cancel_GF = TRUE;                  // Cancel the disturbance capture
          }
        }
      }
      else                                      // If not in pickup, clear the pickup flag and set next the
      {                                         //   state to 0 to start over
        GfpuFlg = 0;           
        if (SdintpuFlg == 0)                    // Clear zone out if both SdpuFlg & GfpuFlg are zero
        { 
          CLEAR_ZOUT;
          Zin_Latched = 0;                      // clear the latched interlock in flag
        }             
        if (GF_Passes >= (80 * 8))              // Must be in pickup for at least 8 cycles to do a
        {                                       //   disturbance capture
          // Compute how close we came to tripping.  There are three cases:
          //   (1) ZSI - we aren't doing a disturbance capture (too short)
          //   (2) Flat - this is just the number of passes divided by the threshold
          //   (3) I2t - this is the 
          if ((Setpoints1.stp.GF_ZSI == 1) && (Zin_Latched == 0))   // trip initiated after two full cycles above
          {
            GFTripBucket = 0;
          }
          else if ((GF_Slope == GF_FLAT) || (CurIgOneCycSOS >= GF_OneCyc_0p625))
          {
            GFTripBucket = (uint16_t)((GF_Passes * 1000)/GF_TripThresholdFlat);
          }
          else
          {
            GFTripBucket = (uint16_t)((GF_Tally * 1000.0)/GF_TripThresholdI2t);
            // If GF_Tally reached the threshold, we didn't trip because the minimum passes wasn't reached,
            //   so use the number of passes to compute how close we came to tripping
            if (GFTripBucket >= 1000.0)
            {
              GFTripBucket = (uint16_t)((GF_Passes * 1000)/GF_MIN_PASSES_FOR_TRIP);
            }
          }
          Dist_Flag_GF = FALSE;                 // Clear the Dist_Flag so the disturbance capture is
        }                                       //   completed (if one had been started)
        else
        {
          Dist_Flag_Cancel_GF = TRUE;           // Otherwise cancel the disturbance capture
        }
        GF_State = 0;
        if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.GF_Trip_StartingSampleCounter_Logged == TRUE))
        {
          HW_SecInjTest.GF_Trip_StartingSampleCounter_Logged = FALSE;
        }
        
      }
      break;

  }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Ground_Fault_Prot()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Sneakers_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Sneakers Protective function
//
//  MECHANICS:          A sneakers condition is defined as an incomplete closure (i.e. a closure with
//                      inadequate contact pressure). This can cause arcing and overheating. This condition
//                      can be detected because the aux switch will continue to indicate that the breaker is
//                      open while current is flowing.
//
//                      If the aux switch indicates the breaker is open...
//                         Above Ldpu...
//                            trip after 0.5 second and flash the Short Delay LED.
//
//                      This routine is called every one cycle anniversary.  A 0.5 sec timeout
//                      will take 0.5 x 60 = 30 passes at 60Hz, or 25 passes at 50Hz.
//
//  CAVEATS:
//
//
//  INPUTS:             LdpuFlg, OpenFlg
//
//  OUTPUTS:            Sneakers_TripFlg, BellTripFlg, TripReqFlg, AlarmHoldOffTmr
//
//  ALTERS:             SneakCount
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void Sneakers_Prot(void)
{
   uint16_t passtime;
                                            // adjust sneakers trip time according to frequency to make sure 0.5s delay
   passtime = (Setpoints0.stp.Freq == 60) ? 30 : 25;
                                            // run sneakers protection only if aux switch indicates breaker open
   if (OpenFlg == 1)                        // OpenFlg defined in Prot_def.h as part of Flags6
   {
      if (LdPuAlmFlg == 1)                     // above LD pick up...
      {
         if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Sneaker_Trip_StartingSampleCounter_Logged == FALSE))
         {
           HW_SecInjTest.Sneaker_Trip_StartingSampleCounter = SampleCounter;
           HW_SecInjTest.Sneaker_Trip_StartingSampleCounter_Logged = TRUE;
         }

         if (SneakCount < passtime)
         {
            SneakCount++;                   // not timed-out yet
         }
         else
         {
            SneakCount = 0;
            TripFlagsReset();               // time out, trip
            Reset_ProtRelated_Alarm();
            if (HW_SecInjTest.Enable == TRUE)                      
            {  
              HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
            }
            if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
                ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
            {
                   // Don't activate TA for No Trip tests
            }
            else
            {  
              TA_TRIP_ACTIVE;
            }
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            SneakersTripFlg = 1;            // Set cause of trip
            BellTripFlg = 1;                // Set general trip cause flag
            TripReqFlg = 1;                 // *** BP may not need this
            Prot_Enabled = FALSE;           // Turn off protection
            Prot_Timer = 25;                // Hold off next trip for 250ms
         }
      }
      else                                  // below LD pick up...
      {
        SneakCount = 0;                     //  reset sneakers counter
                                            //  service to secondary injection feature if it is active there

      }
    }
   else                                    // breaker closed...
   {
      SneakCount = 0;                      // clear sneakers counter
      if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Sneaker_Trip_StartingSampleCounter_Logged == TRUE))
      {
        HW_SecInjTest.Sneaker_Trip_StartingSampleCounter_Logged = FALSE;
      }
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Sneakers_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        CurUnbalance_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Current Unbalance Trip Protection.
//
//  MECHANICS:          Current unbalance trip protection is active only when enabled by the
//                      user by setting Setpoints5.stp.CurUnb_T_OFF to 1 (enabled).
//
//                      Unbalance protection will not start until there is 50% of Ir current.
//
//                      Current Unbalance protection is called at the one cycle anniversary.
//                      This routine uses the CurUnbalTot value provided by the Calc_Prot_Current()
//                      function called before this function.
//
//                      A current unbalance trip occurs when the CurUnbalTot is greater than the percentage
//                      specified by the Current Unbalance pickup setpoint, Setpoints5.stp.CurUnb_T_Pu.
//                      CurUnbalTot = ((imax - imin)/imax) * 100.0.
//                      This condition must exist for 1 to 300 seconds specified by Setpoints5.stp.CurUnb_Tt.
//
//                      This routine is called every one cycle anniversary.  A 60 sec timeout
//                      will take 60 x 60 = 3600 passes at 60hz, or 3000 passes at 50hz.
//
//
//  CAVEATS:            Called only if Setpoints5.stp.CurUnb_T_OFF is set for Trip
//
//
//  INPUTS:             Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.CurUnb_T_OFF: [0=disabled, 1=enabled]
//                      Setpoints5.stp.CurUnb_T_Pu: [2 to 90, step=1] %
//                      Setpoints5.stp.CurUnb_T_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurUnbalTot, CurOneCycMax, Ir
//
//  OUTPUTS:            CurrUnbTripFlg, TripReqFlg, AlarmHoldOffTmr
//
//  ALTERS:             CurUnbalTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void CurUnbalance_Prot(void)
{

    if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.CurUnb_T_OFF != 1))   // If Current Unbalance protection is not enabled, 
    {                                        //   reset the timer and pickup flag, and return
      CurUnbalTripTmr = 0;
      CuTripPuFlg = FALSE;
      return;
    }

    // Current unbalance protection is enabled

    if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter_Logged == FALSE))
    {
      HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter = SampleCounter;
      HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter_Logged = TRUE;
    } 
  
    if ( (CurUnbalTot > Setpoints5.stp.CurUnb_T_Pu) && (CurOneCycMax >= (0.5 * Ir)) )
    {
      CurUnbalTripTmr++;

      // If timer has expired, trip
      if ( CurUnbalTripTmr >= (Setpoints5.stp.CurUnb_T_Tt * Setpoints0.stp.Freq / 100) )
      {
        CurUnbalTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();          // clear trip flags if a trip is not in progress
        if (HW_SecInjTest.Enable == TRUE)                      
        {  
          HW_SecInjTest.State = 2;                               // Ending/Result state for HW Sec Inj test
        }
        if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
            ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
        {
              // Don't activate TA for No Trip tests
        }
        else
        {  
          TA_TRIP_ACTIVE;
        }
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        CurrUnbTripFlg = 1;                 // Set cause of trip
        TripReqFlg = 1;                     // Firmware trip request 1 => trip
        BellTripFlg = 1;                    // Set general trip cause flag
        Prot_Enabled = FALSE;               // Turn off protection
        Prot_Timer = 25;                    // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)        // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_CURRENT_UNBAL);
      }
      // Otherwise if we have just entered pickup, insert an entry event and initiate a disturbance capture
      // Note, the disturbance capture is started immediately, regardless of the delay timer, so that
      //   measurements are taken over the entire disturbance.  The capture is abandoned if the unit trips
      //   or the delay time does not reach 50% of the delay time setting
      else if (CuTripPuFlg == FALSE)        // If we have just entered pickup...
      {
        // Save the EID of the event that started the disturbance capture, and insert an entry event
        sListOfDist[DS_CUR_UNBAL].EIDofStartEvnt = InsertNewEvent(CU_PICKUP);
        Dist_Flag |= DS_CU;                     // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
                                                //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
                                                //          THEN NEED TO CLEAR FLAGS IF FEATURE IS UNLOCKED
        CuTripPuFlg = TRUE;                     // Set flag to True
      }                                        

    }
    else                                    // Unbalance below pickup
    {
      if (Dist_Flag & DS_CU)                    // If a disturbance capture was initiated...
      {
        // If the delay time never reached 50% of the setting, insert an exit event and abort the
        //   disturbance capture.
        if ( CurUnbalTripTmr < ((Setpoints5.stp.CurUnb_T_Tt * Setpoints0.stp.Freq) / 200) )
        {
          InsertNewEvent(CURRENT_UNBAL_EXIT);
          Dist_Flag_Cancel |= DS_CU;
        }
        // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
        //   came to tripping.  This is just the delay time divided by the threshold time
        //   The value is computed in tenths of a percent and is clamped at 100%
        else
        {
          CuTripBucket = (uint16_t)( ((uint32_t)CurUnbalTripTmr * 1000) /
              ((uint32_t)Setpoints5.stp.CurUnb_T_Tt * (uint32_t)Setpoints0.stp.Freq / 100) );
          CuTripBucket = ( (CuTripBucket > 1000) ? 1000 : CuTripBucket);
        }
        // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
        //   This will also cause an exit event to be generated
        Dist_Flag &= (DS_CU ^ 0xFFFFFFFF);
      }
//    CurrUnbTripFlg = 0;                     // *** DAH  ISN'T THIS RESET ONLY IN TripFlagsReset()?
      CurUnbalTripTmr = 0;
      CuTripPuFlg = FALSE;                  // Reset pickup flag

      if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter_Logged == TRUE))
      {
        HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter_Logged = FALSE;
      }
    }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             CurUnbalance_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PhaseLoss_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Current Based Phase Loss Protection.
//
//  MECHANICS:          Phase Loss protection is used for the complete loss of one or two phases. If
//                      the difference between the maximum and minimum of any of the three phase
//                      currents is greater than 75% for the time delay, then the protection action
//                      will be taken.
//
//                      Phase Loss protection will not be active unless at least one phase current is
//                      greater than 50% of Ir.
//
//                      Phase Loss protection is called at the one cycle anniversary.
//                      This routine uses the CurUnbalTot value provided by the Calc_Prot_Current()
//                      function called before this function.
//
//                      A phase loss trip occurs when the CurUnbalTot is greater than 75%.
//                      This condition must exist for 1 to 240 seconds specified by Setpoints5.stp.PhaseLoss_Tt.
//
//                      This routine is called every one cycle anniversary.  A 240 sec timeout will take
//                      240 x 60 = 14400 passes at 60hz, or 12000 passes at 50hz.
//
//
//  CAVEATS:            Phase loss trip protection is active only when enabled by the user by setting
//                      PhaseLoss_T_A_OFF to 0 (trip).
//
//  INPUTS:             Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.PhaseLoss_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.PhaseLoss_Tt: [1 to 240, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurUnbalTot, CurOneCycMax
//
//  OUTPUTS:            PhaseLossTripFlg, TripReqFlg, PlTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             PhaseLossTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------

void PhaseLoss_Prot(void)
{
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.PhaseLoss_T_A_OFF != 0))   // If Phase Loss protection is not enabled, 
  {                                            //  reset the timer and pickup flag, and return
    PhaseLossTripTmr = 0;
    PlTripPuFlg = FALSE;
    return;
  }

  // Phase loss protection is enabled

  // If above threshold, check time
  if ( (CurUnbalTot > 75) && (CurOneCycMax >= (0.5 * Ir)) )
  {
     if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter_Logged == FALSE))
     {
       HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter = SampleCounter;
       HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter_Logged = TRUE;
     } 
     PhaseLossTripTmr++;

     // If timer expired, trip
     if ( PhaseLossTripTmr >= (Setpoints5.stp.PhaseLoss_Tt * Setpoints0.stp.Freq) )
     {
       PhaseLossTripTmr = 0;
       TripFlagsReset();                                       // clear trip flags if a trip is not in progress
       Reset_ProtRelated_Alarm();
       PhaseLossTripFlg = 1;                                   // set cause of trip
       if (HW_SecInjTest.Enable == TRUE)                      
       {  
         HW_SecInjTest.State = 2;                              // Ending/Result state for HW Sec Inj test
       }
       if (((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.Trip_NoTrip == NO_TRIP)) || 
           ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)))
       {
               // Don't activate TA for No Trip tests
       }
       else
       {  
         TA_TRIP_ACTIVE;
       }
       TA_Timer = 24;
       AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
       TripReqFlg = 1;                     // Firmware trip request
       BellTripFlg = 1;                    // Set general trip cause flag
       Prot_Enabled = FALSE;               // Turn off protection
       Prot_Timer = 25;                    // Hold off next trip for 250ms
       if (!Trip_WF_Capture.InProg)        // Set the trip waveform capture req flag
       {
         Trip_WF_Capture.Req = TRUE;
       }
       InsertNewEvent(TRIP_PHASE_LOSS);
     }
     // Otherwise if we have just entered pickup, insert an entry event and initiate a disturbance capture
     // Note, the disturbance capture is started immediately, regardless of the delay timer, so that
     //   measurements are taken over the entire disturbance.  The capture is abandoned if the unit trips
     //   or the delay time does not reach 50% of the delay time setting
     else if (PlTripPuFlg == FALSE)        // If we have just entered pickup...
     {
       // Save the EID of the event that started the disturbance capture, and insert an entry event
       sListOfDist[DS_PHASE_LOSS].EIDofStartEvnt = InsertNewEvent(PL_PICKUP);
       Dist_Flag |= DS_PL;
       PlTripPuFlg = TRUE;                     // Set flag to True
     }                                        
   }
   else
   {                                     // below PL threshold
     if (Dist_Flag & DS_PL)                    // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( PhaseLossTripTmr < ((Setpoints5.stp.PhaseLoss_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(PHASE_LOSS_EXIT);
         Dist_Flag_Cancel |= DS_PL;
       }
        // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
        //   came to tripping.  This is just the delay time divided by the threshold time
        //   The value is computed in tenths of a percent and is clamped at 100%
        else
        {
          PlTripBucket = (uint16_t)( ((uint32_t)PhaseLossTripTmr * 1000) /
              (Setpoints5.stp.PhaseLoss_Tt * Setpoints0.stp.Freq) );
          PlTripBucket = ( (PlTripBucket > 1000) ? 1000 : PlTripBucket);
        }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_PL ^ 0xFFFFFFFF);
     }
     PlTripPuFlg = FALSE;                  // Reset pickup flag
     PhaseLossTripTmr = 0;                 // Reset timer

     if ((HW_SecInjTest.Enable == TRUE) && (HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter_Logged == TRUE))
     {
       HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter_Logged = FALSE;
     }
   }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PhaseLoss_Prot
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverVoltage_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Overvoltage Trip Protection function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any AFE line-to-line Vab_Rms or Vbc_Rms or Vca_Rms is above the OverVolt_T_Pu setpoint
//                      for a duration in excess of the OverVolt_T_Tt, set TripReqFlg and OvTripFlg.
//
//                      Do Overvoltage protection only if the breaker is closed (OpenFlg==0).
//
//                      Below OverVolt_T_Pu, clear the OvTripTmr.
//
//  CAVEATS:            Vll_max is determined in Calc_Prot_AFE_Voltage(). One-cycle AFE voltages are used.
//
//  INPUTS:             Setpoints5.stp.OverVolt_T_OFF: [0=disabled, 1=enabled]
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.OverVolt_T_Pu: [102 to 150, step=0.1] %
//                      Setpoints5.stp.OverVolt_T_Tt: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_max, OvTripTmr
//
//  OUTPUTS:            OvTripFlg, TripReqFlg, OvTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             OvTripTmr
//
//  CALLS:              TripFlagsReset();
//
//------------------------------------------------------------------------------------------------------------
//

void OverVoltage_Prot(void)
{

  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.OverVolt_T_OFF != 1))   // If Overvoltage protection is not enabled, 
  {                                         //  reset the timer and pickup flag, and return
    OvTripTmr = 0;
    OvTripPuFlg = FALSE;
    return;
  }

  // Overvoltage protection is enabled
   
  if ((Vll_max >= Setpoints5.stp.OverVolt_T_Pu * Setpoints0.stp.SystemVoltage / 1000) && (OpenFlg == 0))
  {                                // Above voltage pickup
    // The multiplication can result in a U32.  Standard C apparently casts a U16 x U16 to an int, which in
    //   our 32-bit micro is a 32-bit number, so we are ok.  If this code is ever used in a 16-bit micro,
    //    there may be an issue without the casting.  It is asumed that after the division by 100, the
    //    result is back down to a U16
    if ( OvTripTmr >= ( (uint16_t)((uint32_t)Setpoints5.stp.OverVolt_T_Tt * (uint32_t)Setpoints0.stp.Freq / 100) ) )
    {                              // Trip delay expired - Trip
       OvTripTmr = 0;
       TripFlagsReset();
       Reset_ProtRelated_Alarm();
       TA_TRIP_ACTIVE;
       TA_Timer = 24;
       AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
       OvTripFlg = 1;
       TripReqFlg  = 1;
       BellTripFlg = 1;                 // Set general trip cause flag
       Prot_Enabled = FALSE;            // Turn off protection
       Prot_Timer = 25;                 // Hold off next trip for 250ms
       if (!Trip_WF_Capture.InProg)     // Set the trip waveform capture req flag
       {
         Trip_WF_Capture.Req = TRUE;
       }
       InsertNewEvent(TRIP_OVERVOLTAGE);
    }
    else                           // Under trip delay
    {
       OvTripTmr++;                // Increment trip timer
       // If we have just entered pickup, insert an entry event and initiate a disturbance capture
       // Note, the disturbance capture is started immediately, regardless of the delay timer, so that
       //   measurements are taken over the entire disturbance.  The capture is abandoned if the unit trips
       //   or the delay time does not reach 50% of the delay time setting
       if (OvTripPuFlg == FALSE)             // If we have just entered pickup...
       {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_OVER_VOLT].EIDofStartEvnt = InsertNewEvent(OV_PICKUP);
          Dist_Flag |= DS_OV;
          OvTripPuFlg = TRUE;                      // Set flag to True
       }
    }
  }
  else                             // Below voltage pickup
  {
    if (Dist_Flag & DS_OV)              // If a disturbance capture was initiated...
    {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture
       if ( OvTripTmr < ((Setpoints5.stp.OverVolt_T_Tt * Setpoints0.stp.Freq) / 200) )
       {
          InsertNewEvent(OVERVOLTAGE_EXIT);
          Dist_Flag_Cancel |= DS_OV;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         OvTripBucket = (uint16_t)( ((uint32_t)OvTripTmr * 1000) /
             ((uint32_t)Setpoints5.stp.OverVolt_T_Tt * (uint32_t)Setpoints0.stp.Freq / 100) );
         OvTripBucket = ( (OvTripBucket > 1000) ? 1000 : OvTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_OV ^ 0xFFFFFFFF);
    }
    else if (OvTripPuFlg == TRUE)           // If a disturbance capture wasn't initiated but we entered
    {                                       //  pickup (shouldn't happen under normal circumstances), just
       InsertNewEvent(OVERVOLTAGE_EXIT);    //  insert an exit event
    }
    OvTripTmr = 0;                      // Reset timer
    OvTripPuFlg = FALSE;                // Reset pickup flag
  }

}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverVoltage_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        UnderVoltage_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Undervoltage Trip Protection Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any AFE line-to-line Vab_Rms or Vbc_Rms or Vca_Rms is below the UnderVolt_T_Pu
//                      for a duration in excess of the UnderVolt_T_Tt, set TripReqFlg and UvTripFlg.
//
//                      Do Undervoltage protection only if the breaker is closed (OpenFlg==0).
//
//                      Above UnderVolt_T_Pu, clear the UvTripTmr.
//
//
//  CAVEATS:            Vll_min is determined in Calc_Prot_AFE_Voltage(). One-cycle AFE voltages are used.
//
//  INPUTS:             Setpoints5.stp.UnderVolt_T_OFF: [0=disabled, 1=enabled]
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.UnderVolt_T_Pu: [50 to 98, step=0.1] %
//                      Setpoints5.stp.UnderVolt_T_Tt: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_min, UvTripTmr
//
//  OUTPUTS:            UvTripFlg, TripReqFlg, UvTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             UvTripTmr
//
//  CALLS:              TripFlagsReset();
//
//------------------------------------------------------------------------------------------------------------

void UnderVoltage_Prot(void)
{

  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.UnderVolt_T_OFF != 1))  // If Undervoltage protection is not enabled, 
  {                                         //  reset the timer and pickup flag, and return
    UvTripTmr = 0;
    UvTripPuFlg = FALSE;
    return;
  }

  // Undervoltage protection is enabled

  if ((Vll_min < Setpoints5.stp.UnderVolt_T_Pu * Setpoints0.stp.SystemVoltage / 1000)  && (OpenFlg == 0))
  {                                // Below voltage pickup
    if (UvTripTmr >= (Setpoints5.stp.UnderVolt_T_Tt * Setpoints0.stp.Freq / 100))
    {                             // Trip delay expired - Trip
       UvTripTmr = 0;
       TripFlagsReset();
       Reset_ProtRelated_Alarm();
       UvTripFlg = 1;
       TA_TRIP_ACTIVE;
       TA_Timer = 24;
       AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
       TripReqFlg  = 1;
       BellTripFlg = 1;                 // Set general trip cause flag
       Prot_Enabled = FALSE;            // Turn off protection
       Prot_Timer = 25;                 // Hold off next trip for 250ms
       if (!Trip_WF_Capture.InProg)     // Set the trip waveform capture req flag
       {
         Trip_WF_Capture.Req = TRUE;
       }
       InsertNewEvent(TRIP_UNDERVOLTAGE);
    }
    else
    {
       UvTripTmr++;
       // If we have just entered pickup, insert an entry event and initiate a disturbance capture
       // Note, the disturbance capture is started immediately, regardless of the delay timer, so that
       //   measurements are taken over the entire disturbance.  The capture is abandoned if the unit trips
       //   or the delay time does not reach 50% of the delay time setting
       if (UvTripPuFlg == FALSE)            // If we have just entered pickup...
       {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_UNDER_VOLT].EIDofStartEvnt = InsertNewEvent(UV_PICKUP);
          Dist_Flag |= DS_UV;
          UvTripPuFlg = TRUE;                   // Set flag to True
       }
    }                                        
  }
  else                            // Above threshold
  {
    if (Dist_Flag & DS_UV)              // If a disturbance capture was initiated...
    {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture
       if ( UvTripTmr < ((Setpoints5.stp.UnderVolt_T_Tt * Setpoints0.stp.Freq) / 200) )
       {
          InsertNewEvent(UNDERVOLTAGE_EXIT);
          Dist_Flag_Cancel |= DS_UV;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         UvTripBucket = (uint16_t)( ((uint32_t)UvTripTmr * 1000) /
             ((uint32_t)Setpoints5.stp.UnderVolt_T_Tt * (uint32_t)Setpoints0.stp.Freq / 100) );
         UvTripBucket = ( (UvTripBucket > 1000) ? 1000 : UvTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_UV ^ 0xFFFFFFFF);
    }
    else if (UvTripPuFlg == TRUE)           // If a disturbance capture wasn't initiated but we entered
    {                                       //  pickup (shouldn't happen under normal circumstances), just
       InsertNewEvent(UNDERVOLTAGE_EXIT);    //  insert an exit event
    }
    UvTripTmr = 0;                          // Reset timer
    UvTripPuFlg = FALSE;                    // Reset pickup flag
  }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             UnderVoltage_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        VoltUnbalance_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Voltage Unbalance Trip Protection Function.
//
//  MECHANICS:          Voltage Unbalance trip protection is active only when enabled by the
//                      user by setting Setpoints5.stp.VoltUnb_T_OFF to 1 (enabled).
//
//                      Unbalance protection will not start until there is a minimum l-l voltage (Vll_min > 84).
//
//                      Voltage Unbalance protection is called at the one cycle anniversary.
//                      This routine uses the VolUnbalTot value provided by the Calc_Prot_AFE_Voltage()
//                      function called before this function.
//
//                      A voltage unbalance trip occurs when the VolUnbalTot is greater than the percentage
//                      specified by the Voltage Unbalance pickup setpoint, Setpoints5.stp.VoltUnb_T_Pu;.
//                      VolUnbalTot = ((vmax - vmin)/vmax) * 100.0.
//                      This condition must exist for 1 to 60 seconds specified by Setpoints5.stp.VoltUnb_T_Tt.
//
//                      This routine is called every one cycle anniversary.  A 60 sec timeout
//                      will take 60 x 60 = 3600 passes at 60hz, or 3000 passes at 50hz.
//
//                      If the duration exceeds the VunbalTripTime, set TripReqFlg  and VolUnbalTot.
//
//                      Do Voltage Unbalance protection only if the breaker is closed (OpenFlg==0).
//
//                      Below VoltUnb_T_Pu;, clear the VunbalTripTmr.
//
//
//  CAVEATS:            Setpoints5.stp.VoltUnb_T_OFF must be set to 1 (enabled)
//
//  INPUTS:             Setpoints5.stp.VoltUnb_T_OFF: [0=disabled, 1=enabled]
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.VoltUnb_T_Pu;: [2 to 90] %
//                      Setpoints5.stp.VoltUnb_T_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_max, Vll_min, VoltUnbalTripTmr
//
//  OUTPUTS:            VoltUnbalTripFlg, TripReqFlg, VuTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             VoltUnbalTripTmr
//
//  CALLS:              TripFlagsReset();
//
//------------------------------------------------------------------------------------------------------------

void VoltUnbalance_Prot(void)
{

  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.VoltUnb_T_OFF != 1))  // If Voltage Unbalance protection is not enabled, 
  {                                        //  reset the timer and pickup flag, and return
    VoltUnbalTripTmr = 0;
    VuTripPuFlg = FALSE;
    return;
  }

  // Voltage unbalance protection is enabled

  // If unbalance above pickup, insert event and check delay time
  if ( (VolUnbalTot > Setpoints5.stp.VoltUnb_T_Pu)  && (Vll_min > 84)  && (OpenFlg == 0))
  {
    VoltUnbalTripTmr++;

    // If timer expired, trip
    if ( VoltUnbalTripTmr >= (Setpoints5.stp.VoltUnb_T_Tt * Setpoints0.stp.Freq / 100) )
    {
      VoltUnbalTripTmr = 0;
      TripFlagsReset();                                       // clear trip flags if a trip is not in progress
      Reset_ProtRelated_Alarm();
      VoltUnbalTripFlg = 1;
      TA_TRIP_ACTIVE;
      TA_Timer = 24;
      AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
      TripReqFlg = 1;                       // Firmware trip request 1 => trip
      BellTripFlg = 1;                      // Set general trip cause fla
      Prot_Enabled = FALSE;                 // Turn off protection
      Prot_Timer = 25;                      // Hold off next trip for 250ms
      if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
      {
        Trip_WF_Capture.Req = TRUE;
      }
      InsertNewEvent(TRIP_VOLTAGE_UNBAL);
    }
    // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance capture
    // Note, the disturbance capture is started immediately, regardless of the delay timer, so that
    //   measurements are taken over the entire disturbance.  The capture is abandoned if the unit trips or
    //   the delay time does not reach 50% of the delay time setting
    else if (VuTripPuFlg == FALSE)
    {
      // Save the EID of the event that started the disturbance capture, and insert an entry event
      sListOfDist[DS_VOLT_UNBAL].EIDofStartEvnt = InsertNewEvent(VU_PICKUP);
      Dist_Flag |= DS_VU;
      VuTripPuFlg = TRUE;                   // Set flag to True
    }                                        
  }
  else                                      // Unbalance below pickup
  {
    if (Dist_Flag & DS_VU)                      // If a disturbance capture was initiated...
    {
      // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
      //   capture.
      if ( VoltUnbalTripTmr < ((Setpoints5.stp.VoltUnb_T_Tt * Setpoints0.stp.Freq) / 200) )
      {
        InsertNewEvent(VOLTAGE_UNBAL_EXIT);
        Dist_Flag_Cancel |= DS_VU;
      }
      // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
      //   came to tripping.  This is just the delay time divided by the threshold time
      //   The value is computed in tenths of a percent and is clamped at 100%
      else
      {
        VuTripBucket = (uint16_t)( ((uint32_t)VoltUnbalTripTmr * 1000) /
            ((uint32_t)Setpoints5.stp.VoltUnb_T_Tt * (uint32_t)Setpoints0.stp.Freq / 100) );
        VuTripBucket = ( (VuTripBucket > 1000) ? 1000 : VuTripBucket);
      }
      // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
      //   This will also cause an exit event to be generated
      Dist_Flag &= (DS_VU ^ 0xFFFFFFFF);
    }
    else if (VuTripPuFlg == TRUE)           // If a disturbance capture wasn't initiated but we entered
    {                                       //  pickup (shouldn't happen under normal circumstances), just
       InsertNewEvent(VOLTAGE_UNBAL_EXIT);  //  insert an exit event
    }
//    VoltUnbalTripFlg = 0;                     // *** DAH  ISN'T THIS RESET ONLY IN TripFlagsReset()?
    VuTripPuFlg = FALSE;                  // Reset pickup flag
    VoltUnbalTripTmr = 0;                 // Reset timer
  }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             VoltUnbalance_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverFreq_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Overfrequency Trip Protection function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary if frequency protection is
//                      enabled: Setpoints4.stp.Freq_Prot_Enable == 1.  And also if the trip frequency protection
//                      setpoint is set to enabled: Setpoints4.stp.OF_Trip_Prot_Action == 1
//
//                      If FreqLine.FreqVal is above the OF_Trip_Prot_Pickup for a duration in excess of the
//                      OF_Trip_Prot_Time, set TripReqFlg  and OfTripFlg.
//
//                      To prevent frequency trips and alarms in the absence of line voltage, skip
//                      frequency protection if the measured frequency is marked invalid:
//                      (FreqLine.FreqVal = NAN).  The Calc_Freq() routine will set the measured
//                      frequency to invalid if inadequate voltage exists to properly determine the
//                      zero-crossing point of Van.
//
//                      Do Overfrequency protection only if the breaker is closed (OpenFlg==0).
//
//                      Below the Setpoints4.stp.OF_Trip_Prot_Pickup, clear the OfTripTmr.
//
//  CAVEATS:            This function is called from Main() if the Freq Protection setpoint is set to Enable
//
//  INPUTS:             FreqLine.FreqVal = The measured line frequency in Hz
//                      Setpoints4.stp.Freq_Prot_Enable
//                      Setpoints4.stp.OF_Trip_Prot_Action: [0=disabled, 1=enabled]
//                      Setpoints4.stp.OF_Trip_Prot_Pickup: [100.1 - 110.0, step=0.1] %
//                      Setpoints4.stp.OF_Trip_Prot_Time: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      OfTripTmr
//
//  OUTPUTS:            OfTripFlg, TripReqFlg, OfTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             OfTripTmr
//
//  CALLS:              TripFlagsReset();
//
//------------------------------------------------------------------------------------------------------------

void OverFreq_Prot(void)
{
   // If overfrequency protection is not enabled, reset the timer and pickup flag, and return
   if ( (Setpoints4.stp.Freq_Prot_Enable != 1) || (Setpoints4.stp.OF_Trip_Prot_Action != 1) )
   {
     OfTripTmr = 0;
     OfTripPuFlg = FALSE;
     return;
   }

   // Over-frequency protection is enabled

   // Is there a valid Freq value?
   if ((FreqLine.FreqVal != NAN) && (OpenFlg == 0))
   {
      // If frequency above pickup, insert event and check delay time
      // Multiply FreqVal by 1000 instead of dividing integer Pickup value by 1000
      if (1000*FreqLine.FreqVal >= Setpoints4.stp.OF_Trip_Prot_Pickup * Setpoints0.stp.Freq)
      {
         // If timer expired, trip
         if (OfTripTmr >= (Setpoints4.stp.OF_Trip_Prot_Time * Setpoints0.stp.Freq /100) )
         {
            OfTripTmr = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            TA_TRIP_ACTIVE;
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            OfTripFlg = 1;
            TripReqFlg  = 1;
            BellTripFlg = 1;                // Set general trip cause flag
            Prot_Enabled = FALSE;           // Turn off protection
            Prot_Timer = 25;                // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)    // Set the trip waveform capture req flag
            {
              Trip_WF_Capture.Req = TRUE;
            }
            InsertNewEvent(TRIP_OVERFREQUENCY);
         }
         // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
         //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
         //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
         //   unit trips or the delay time does not reach 50% of the delay time setting
         else                           // Trip timer not timed-out
         {
            if (OfTripPuFlg == FALSE)
            {
              // Save the EID of the event that started the disturbance capture, and insert an entry event
              sListOfDist[DS_OVER_FREQ].EIDofStartEvnt = InsertNewEvent(OF_PICKUP);
              Dist_Flag |= DS_OF;
              OfTripPuFlg = TRUE;                   // Set flag to True
            }                                        
            OfTripTmr++;
         }
      }
      else                              // Frequency not over limit...
      {
         if (Dist_Flag & DS_OF)             // If a disturbance capture was initiated...
         {
            // If the delay time never reached 50% of the setting, insert an exit event and abort the
            //   disturbance capture
            if ( OfTripTmr < ((Setpoints4.stp.OF_Trip_Prot_Time * Setpoints0.stp.Freq) / 200) )
            {
              InsertNewEvent(OVERFREQUENCY_EXIT);
              Dist_Flag_Cancel |= DS_OF;
            }
            // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
            //   came to tripping.  This is just the delay time divided by the threshold time
            //   The value is computed in tenths of a percent and is clamped at 100%
            else
            {
              OfTripBucket = (uint16_t)( ((uint32_t)OfTripTmr * 1000) /
                  ((uint32_t)Setpoints4.stp.OF_Trip_Prot_Time * (uint32_t)Setpoints0.stp.Freq / 100) );
              OfTripBucket = ( (OfTripBucket > 1000) ? 1000 : OfTripBucket);
            }
            // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
            //   This will also cause an exit event to be generated
            Dist_Flag &= (DS_OF ^ 0xFFFFFFFF);
         }
         OfTripPuFlg = FALSE;           // Reset pickup flag
         OfTripTmr = 0;                 // Reset timer
      }
   }
   else                                 // Trip protection turned off or invalid frequency value
   {
     if (Dist_Flag & DS_OF)             // If a disturbance capture was initiated...
     {
         // If the delay time never reached 50% of the setting, insert an exit event and abort the
         //   disturbance capture
         if ( OfTripTmr < ((Setpoints4.stp.OF_Trip_Prot_Time * Setpoints0.stp.Freq) / 200) )
         {
           InsertNewEvent(OVERFREQUENCY_EXIT);
           Dist_Flag_Cancel |= DS_OF;
         }
         // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
         //   This will also cause an exit event to be generated
         Dist_Flag &= (DS_OF ^ 0xFFFFFFFF);
      }
      OfTripPuFlg = FALSE;           // Reset pickup flag
      OfTripTmr = 0;                 // Reset timer
   }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverFreq_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        UnderFreq_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Underfrequency Trip Protection function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary if frequency protection is
//                      enabled: Setpoints4.stp.Freq_Prot_Enable == 1.  And also if the trip frequency protection
//                      setpoint is set to enable: Setpoints4.stp.UF_Trip_Prot_Action == 1
//
//                      If FreqLine.FreqVal is below the UF_Trip_Prot_Pickup for a duration in excess of the
//                      UF_Trip_Prot_Time, set TripReqFlg and OfTripFlg.
//
//                      To prevent frequency trips and alarms in the absence of line voltage, skip
//                      frequency protection if the measured frequency is marked invalid:
//                      (FreqLine.FreqVal = NAN).  The Calc_Freq() routine will set the measured
//                      frequency to invalid if inadequate voltage exists to properly determine the
//                      zero-crossing point of Van.
//
//                      Do Underfrequency protection only if the breaker is closed (OpenFlg==0).
//
//                      Above the Setpoints4.stp.UF_Trip_Prot_Pickup, clear the UfTripTmr.
//
//  CAVEATS:            This function is called from Main() if the Freq trip Protection setpoint is set to Enable
//
//  INPUTS:             FreqLine.FreqVal = The measured line frequency in Hz
//                      Setpoints4.stp.Freq_Prot_Enable
//                      Setpoints4.stp.UF_Trip_Prot_Action: [0=disabled, 1=enabled]       <--used in main()
//                      Setpoints4.stp.UF_Trip_Prot_Pickup: [90.0 - 99.9, step=0.1] %
//                      Setpoints4.stp.UF_Trip_Prot_Time: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      UfTripTmr
//
//  OUTPUTS:            UfTripFlg, TripReqFlg, UfTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             UfTripTmr
//
//  CALLS:              TripFlagsReset();
//
//------------------------------------------------------------------------------------------------------------

void UnderFreq_Prot(void)
{
   // If underfrequency protection is not enabled, reset the timer and pickup flag, and return
   if ( (Setpoints4.stp.Freq_Prot_Enable != 1) || (Setpoints4.stp.UF_Trip_Prot_Action != 1) )
   {
     UfTripTmr = 0;
     UfTripPuFlg = FALSE;
     return;
   }

   // Under-frequency protection is enabled

   // Is there a valid Freq value?
   if ((FreqLine.FreqVal != NAN) && (OpenFlg == 0))
   {
      // If frequency under pickup, insert event and check delay time
      // Multiply FreqVal by 1000 instead of dividing integer Pickup value by 1000
      if (1000*FreqLine.FreqVal < Setpoints4.stp.UF_Trip_Prot_Pickup * Setpoints0.stp.Freq)
      {
         // If timer expired, trip
         if (UfTripTmr >= (Setpoints4.stp.UF_Trip_Prot_Time * Setpoints0.stp.Freq / 100) )
         {
            UfTripTmr = 0;
            TripFlagsReset();
            Reset_ProtRelated_Alarm();
            UfTripFlg = 1;
            TA_TRIP_ACTIVE;
            TA_Timer = 24;
            AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
            TripReqFlg  = 1;
            BellTripFlg = 1;                // Set general trip cause flag
            Prot_Enabled = FALSE;           // Turn off protection
            Prot_Timer = 25;                // Hold off next trip for 250ms
            if (!Trip_WF_Capture.InProg)    // Set the trip waveform capture req flag
            {
              Trip_WF_Capture.Req = TRUE;
            }
            InsertNewEvent(TRIP_UNDERFREQUENCY);
         }
         // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
         //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
         //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
         //   unit trips or the delay time does not reach 50% of the delay time setting
         else                           // Trip timer not timed-out
         {
            if (UfTripPuFlg == FALSE)
            {
              // Save the EID of the event that started the disturbance capture, and insert an entry event
              sListOfDist[DS_UNDER_FREQ].EIDofStartEvnt = InsertNewEvent(UF_PICKUP);
              Dist_Flag |= DS_UF;
              UfTripPuFlg = TRUE;                   // Set flag to True
            }                                        
            UfTripTmr++;
         }
      }
      else                              // Frequency not under the limit...
      {
         if (Dist_Flag & DS_UF)             // If a disturbance capture was initiated...
         {
            // If the delay time never reached 50% of the setting, insert an exit event and abort the
            //   disturbance capture
            if ( UfTripTmr < ((Setpoints4.stp.UF_Trip_Prot_Time * Setpoints0.stp.Freq) / 200) )
            {
              InsertNewEvent(UNDERFREQUENCY_EXIT);
              Dist_Flag_Cancel |= DS_UF;
            }
            // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
            //   came to tripping.  This is just the delay time divided by the threshold time
            //   The value is computed in tenths of a percent and is clamped at 100%
            else
            {
              UfTripBucket = (uint16_t)( ((uint32_t)UfTripTmr * 1000) /
                  ((uint32_t)Setpoints4.stp.UF_Trip_Prot_Time * (uint32_t)Setpoints0.stp.Freq / 100) );
              UfTripBucket = ( (UfTripBucket > 1000) ? 1000 : UfTripBucket);
            }
            // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
            //   This will also cause an exit event to be generated
            Dist_Flag &= (DS_UF ^ 0xFFFFFFFF);
         }
         UfTripPuFlg = FALSE;           // Reset pickup flag
         UfTripTmr = 0;                 // Reset timer
      }
   }
   else                                 // Trip protection turned off
   {
     if (Dist_Flag & DS_UF)             // If a disturbance capture was initiated...
     {
         // If the delay time never reached 50% of the setting, insert an exit event and abort the
         //   disturbance capture
         if ( UfTripTmr < ((Setpoints4.stp.UF_Trip_Prot_Time * Setpoints0.stp.Freq) / 200) )
         {
           InsertNewEvent(UNDERFREQUENCY_EXIT);
           Dist_Flag_Cancel |= DS_UF;
         }
         // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
         //   This will also cause an exit event to be generated
         Dist_Flag &= (DS_UF ^ 0xFFFFFFFF);
      }
      UfTripPuFlg = FALSE;          // Reset pickup flag
      UfTripTmr = 0;                // Reset timer
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             UnderFreq_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverRealPower_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverRealPower_Prot Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the real power values are above the RealPwr_Pu setpoint for a duration
//                      in excess of the RealPwr_Tt, set TripReqFlg and RealPwrTripFlg.
//                      NOTE: this applies to forward power only!!  If reverse feed is enabled, the sign is
//                      flipped
//
//                      Below RealPwr_Pu, clear the RealPwrTripTmr.
//
//                      One-cycle real power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Real Power T_A_Off setpoint is set to
//                      Trip.
//
//  INPUTS:             Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.RealPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.RealPwr_Pu: [1 to 65550, step=1] kW
//                      Setpoints6.stp.RealPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.Pa, PwrOneCyc.Pb, PwrOneCyc.Pc
//
//  OUTPUTS:            RealPwrTripFlg, TripReqFlg, OvrWTripBucket, MaxPwrOneCycW, AlarmHoldOffTmr
//
//  ALTERS:             RealPwrTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void OverRealPower_Prot(void)
{
   float tempPa, tempPb, tempPc;

   // If Over Real Power protection is not enabled, reset the timer and pickup flag, and return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.RealPwr_T_A_OFF != 0) )
   {
     RealPwrTripTmr = 0;
     OkWTripPuFlg = FALSE;
     return;
   }

   // Over Real Power protection is enabled

   // For regular feed, positive power is "forward", so the sign is ok as is.
   // For reverse feed, negative power is "forward", so flip the sign
   tempPa = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pa : PwrOneCyc.Pa;
   tempPb = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pb : PwrOneCyc.Pb;
   tempPc = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pc : PwrOneCyc.Pc;

   // Compute the absolute value of the maximum forward power and save it in tempPa
   tempPa = ( (tempPa > tempPb) ? tempPa : tempPb);
   tempPa = ( (tempPa > tempPc) ? tempPa : tempPc);

   // Save the max forward power for disturbance capture
   MaxPwrOneCycW = ( (Setpoints0.stp.RevFeed == 1) ? (-tempPa) : tempPa);

   // Now do protection using the absolute value of the max forward power
   if (tempPa >= Setpoints6.stp.RealPwr_Pu * 1000)
   {                                // Above real power pickup

      if (RealPwrTripTmr >= (Setpoints6.stp.RealPwr_Tt * Setpoints0.stp.Freq))
      {                             // Trip delay expired - Trip
        RealPwrTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        RealPwrTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_OVERKW);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (OkWTripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_OVER_W].EIDofStartEvnt = InsertNewEvent(OW_PICKUP);
          Dist_Flag |= DS_OW;
          OkWTripPuFlg = TRUE;                  // Set flag to True
        }                                        
        RealPwrTripTmr++;               // Increment trip timer
      }
   }
   else                                 // Below real power pickup
   {
     if (Dist_Flag & DS_OW)                     // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( RealPwrTripTmr < ((Setpoints6.stp.RealPwr_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(OVERKW_EXIT);
         Dist_Flag_Cancel |= DS_OW;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         OvrWTripBucket = (uint16_t)( ((uint32_t)RealPwrTripTmr * 1000) /
                (Setpoints6.stp.RealPwr_Tt * Setpoints0.stp.Freq) );
         OvrWTripBucket = ( (OvrWTripBucket > 1000) ? 1000 : OvrWTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_OW ^ 0xFFFFFFFF);
     }
     RealPwrTripTmr = 0;                // Reset timer
     OkWTripPuFlg = FALSE;              // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverRealPower_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverReactivePower_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverReactivePower_Prot Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the reactive power values are above the ReacPwr_Pu setpoint for a duration
//                      in excess of the ReacPwr_Tt, set TripReqFlg and ReacPwrTripFlg.
//
//                      Below ReacPwr_Pu, clear the ReacPwrTripTmr.
//
//                      One-cycle reactive power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reactive Power T_A_Off setpoint is set to Trip.
//
//  INPUTS:             Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.ReacPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.ReacPwr_Pu: [1 to 65550, step=1] kVar
//                      Setpoints6.stp.ReacPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.RPa, PwrOneCyc.RPb, PwrOneCyc.RPc
//
//  OUTPUTS:            ReacPwrTripFlg, TripReqFlg, OvrVarTripBucket, MaxPwrOneCycVar, AlarmHoldOffTmr
//
//  ALTERS:             ReacPwrTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void OverReactivePower_Prot(void)
{
   float tempRPa, tempRPb, tempRPc;

   // If Over Reactive Power protection is not enabled, reset the timer and pickup flag, and return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.ReacPwr_T_A_OFF != 0) )
   {
     ReacPwrTripTmr = 0;
     OkVarTripPuFlg = FALSE;
     return;
   }

   // Over Reactive Power protection is enabled

   // For regular feed, positive power is "forward", so the sign is ok as is.
   // For reverse feed, negative power is "forward", so flip the sign
   tempRPa = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPa : PwrOneCyc.RPa;
   tempRPb = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPb : PwrOneCyc.RPb;
   tempRPc = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPc : PwrOneCyc.RPc;

   // Compute the absolute value of the maximum forward power and save it in tempRPa
   tempRPa = ( (tempRPa > tempRPb) ? tempRPa : tempRPb);
   tempRPa = ( (tempRPa > tempRPc) ? tempRPa : tempRPc);

   // Save the max forward power for disturbance capture
   MaxPwrOneCycVar = ( (Setpoints0.stp.RevFeed == 1) ? (-tempRPa) : tempRPa);

   // Now do protection using the absolute value of the max forward power
   if (tempRPa >= Setpoints6.stp.ReacPwr_Pu * 1000)
   {                                // Above reactive power pickup
      if (ReacPwrTripTmr >= (Setpoints6.stp.ReacPwr_Tt * Setpoints0.stp.Freq))
      {                             // Trip delay expired - Trip
        ReacPwrTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        ReacPwrTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_OVERKVAR);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (OkVarTripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_OVER_VAR].EIDofStartEvnt = InsertNewEvent(OVAR_PICKUP);
          Dist_Flag |= DS_OVAR;
          OkVarTripPuFlg = TRUE;                // Set flag to True
        }                                        
        ReacPwrTripTmr++;               // Increment trip timer
      }
   }
   else                                 // Below reactive power pickup
   {
     if (Dist_Flag & DS_OVAR)                   // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( ReacPwrTripTmr < ((Setpoints6.stp.ReacPwr_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(OVERKVAR_EXIT);
         Dist_Flag_Cancel |= DS_OVAR;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         OvrVarTripBucket = (uint16_t)( ((uint32_t)ReacPwrTripTmr * 1000) /
                (Setpoints6.stp.ReacPwr_Tt * Setpoints0.stp.Freq) );
         OvrVarTripBucket = ( (OvrVarTripBucket > 1000) ? 1000 : OvrVarTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_OVAR ^ 0xFFFFFFFF);
     }
     ReacPwrTripTmr = 0;                // Reset timer
     OkVarTripPuFlg = FALSE;            // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverReactivePower_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverApparentPower_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverApparentPower_Prot Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the apparent power values are above the AppPwr_Pu setpoint for a duration
//                      in excess of the AppPwr_Tt, set TripReqFlg and AppPwrTripFlg.
//
//                      Below AppPwr_Pu, clear the AppPwrTripTmr.
//
//                      One-cycle apparent power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Apparent Power T_A_Off setpoint is set to Trip.
//
//  INPUTS:             Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.AppPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.AppPwr_Pu: [1 to 65550, step=1] kVA
//                      Setpoints6.stp.AppPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCycApp.AppPa, PwrOneCycApp.AppPb, PwrOneCycApp.AppPc
//
//  OUTPUTS:            AppPwrTripFlg, TripReqFlg, OvrVATripBucket, MaxPwrOneCycVA, AlarmHoldOffTmr
//
//  ALTERS:             AppPwrTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void OverApparentPower_Prot(void)
{
   // If Over Apparent Power protection is not enabled, reset the timer and pickup flag, and return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.AppPwr_T_A_OFF != 0) )
   {
     AppPwrTripTmr = 0;
     OkVATripPuFlg = FALSE;
     return;
   }

   // Over Apparent Power protection is enabled

   // Compute one-cycle max power (of the three phases) for disturbance capture info
   MaxPwrOneCycVA = ((PwrOneCycApp.AppPa > PwrOneCycApp.AppPb) ? PwrOneCycApp.AppPa : PwrOneCycApp.AppPb);
   MaxPwrOneCycVA = ((MaxPwrOneCycVA > PwrOneCycApp.AppPc) ? MaxPwrOneCycVA : PwrOneCycApp.AppPc);

   if (MaxPwrOneCycVA >= Setpoints6.stp.AppPwr_Pu * 1000)
   {                                // Above apparent power pickup
      if (AppPwrTripTmr >= (Setpoints6.stp.AppPwr_Tt * Setpoints0.stp.Freq))
      {                             // Trip delay expired - Trip
        AppPwrTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        AppPwrTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_OVERKVA);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (OkVATripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_OVER_VA].EIDofStartEvnt = InsertNewEvent(OVA_PICKUP);
          Dist_Flag |= DS_OVA;
          OkVATripPuFlg = TRUE;                 // Set flag to True
        }                                        
        AppPwrTripTmr++;                // Increment trip timer
      }
   }
   else                                 // Below apparent power pickup
   {
     if (Dist_Flag & DS_OVA)                    // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( AppPwrTripTmr < ((Setpoints6.stp.AppPwr_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(OVERKVA_EXIT);
         Dist_Flag_Cancel |= DS_OVA;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         OvrVATripBucket = (uint16_t)( ((uint32_t)AppPwrTripTmr * 1000) /
                (Setpoints6.stp.AppPwr_Tt * Setpoints0.stp.Freq) );
         OvrVATripBucket = ( (OvrVATripBucket > 1000) ? 1000 : OvrVATripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_OVA ^ 0xFFFFFFFF);
     }
     AppPwrTripTmr = 0;                                 // Reset timer
     OkVATripPuFlg = FALSE;                             // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverApparentPower_Prot()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        RevActivePower_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reverse Active Power Prot Function
//                      (called Reverse Power Protection in our other products)
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the real power values are above the RevPwr_Pu setpoint for a duration
//                      in excess of the RevPwr_Tt, set TripReqFlg and RevPwrTripFlg.
//
//                      If Setpoints0.stp.RevFeed == 1, power direction is reversed.
//
//                      Below RevPwr_Pu, clear the RevPwrTripTmr.
//
//                      One-cycle real power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reverse Power T_A_Off setpoint is set to Trip.
//
//  INPUTS:             Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints5.stp.RevPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevPwr_Pu: [1 to 65500, step=1] kW
//                      Setpoints5.stp.RevPwr_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.RevFeed: [0=forward, 1=reverse]
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.Pa, PwrOneCyc.Pb, PwrOneCyc.Pc
//
//  OUTPUTS:            RevPwrTripFlg, TripReqFlg, RevWTripBucket, MaxPwrOneCycRevW, AlarmHoldOffTmr
//
//  ALTERS:             RevPwrTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void RevActivePower_Prot(void)
{
   float tempPa, tempPb, tempPc;

   // If Reverse Active Power protection is not enabled, reset the timer and pickup flag, and return
   if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints5.stp.RevPwr_T_A_OFF != 0) )
   {
     RevPwrTripTmr = 0;
     RevWTripPuFlg = FALSE;
     return;
   }

   // Reverse Active Power protection is enabled


   // For regular feed, negative power is "reverse" and positive power is "forward".  In this case, we need
   //   to flip the sign of the power.  This way, "reverse" power is positive, and "forward" power is
   //   negative.  The setpoint is positive, so "forward" power will never trigger.  We are only checking
   //   for high values of reverse power. 
   // For reverse feed, negative power is "forward", and positive power is "reverse", so the sign is ok as
   // is.
   tempPa = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pa : -PwrOneCyc.Pa;
   tempPb = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pb : -PwrOneCyc.Pb;
   tempPc = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pc : -PwrOneCyc.Pc;

   // Compute max absolute value of reverse power.  Store value in tempPa.  Store signed value in
   //   MaxPwrOneCycRevW (for disturbance capture)
   MaxPwrOneCycRevW = PwrOneCyc.Pa;
   if (tempPb > tempPa)
   {
     tempPa = tempPb;
     MaxPwrOneCycRevW = PwrOneCyc.Pb;
   }
   if (tempPc > tempPa)
   {
     tempPa = tempPc;
     MaxPwrOneCycRevW = PwrOneCyc.Pc;
   }

   // Now do protection using the absolute value of the max reverse power
   if (tempPa >= Setpoints5.stp.RevPwr_Pu * 1000)
   {                                // Above reverse power pickup
      if (RevPwrTripTmr >= (Setpoints5.stp.RevPwr_Tt * Setpoints0.stp.Freq / 100))
      {                             // Trip delay expired - Trip
        RevPwrTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        RevPwrTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms  
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_REVERSE_KW);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (RevWTripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_REV_W].EIDofStartEvnt = InsertNewEvent(REVW_PICKUP);
          Dist_Flag |= DS_RKW;
          RevWTripPuFlg = TRUE;                 // Set flag to True
        }                                        
        RevPwrTripTmr++;                // Increment trip timer
      }
   }
   else                                 // Below reverse power pickup
   {
     if (Dist_Flag & DS_RKW)                    // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( RevPwrTripTmr < ((Setpoints5.stp.RevPwr_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(REVERSE_KW_EXIT);
         Dist_Flag_Cancel |= DS_RKW;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         RevWTripBucket = (uint16_t)( ((uint32_t)RevPwrTripTmr * 1000) /
             ((uint32_t)Setpoints5.stp.RevPwr_Tt * (uint32_t)Setpoints0.stp.Freq) );
         RevWTripBucket = ( (RevWTripBucket > 1000) ? 1000 : RevWTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_RKW ^ 0xFFFFFFFF);
     }
     RevPwrTripTmr = 0;                 // Reset timer
     RevWTripPuFlg = FALSE;             // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             RevActivePower_Prot()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        RevReactivePower_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           RevReactivePower_Prot Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the reactive power values are above the RevReacPwr_Pu setpoint for a duration
//                      in excess of the RevReacPwr_Tt, set TripReqFlg and RevReacPwrTripFlg.
//
//                      If Setpoints0.stp.RevFeed == 1, power direction is reversed.
//
//                      Below RevReacPwr_Pu, clear the RevReacPwrTripTmr.
//
//                      One-cycle reactive power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reverse Reactive Power T_A_Off setpoint is set to Trip.
//
//  INPUTS:             Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints5.stp.RevReactPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevReactPwr_Pu: [1 to 65550, step=1] kW
//                      Setpoints5.stp.RevReactPwr_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.RevFeed: [0=forward, 1=reverse]
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.RPa, PwrOneCyc.RPb, PwrOneCyc.RPc
//
//  OUTPUTS:            RevReacPwrTripFlg, TripReqFlg, RevVarTripBucket, MaxPwrOneCycRevVar, AlarmHoldOffTmr
//
//  ALTERS:             RevReacPwrTripTmr
//
//  CALLS:              TripFlagsReset()

//
//------------------------------------------------------------------------------------------------------------
//
void RevReactivePower_Prot(void)
{
   float tempRPa, tempRPb, tempRPc;

   // If Reverse Rective Power protection is not enabled, reset the timer and pickup flag, and return
   if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints5.stp.RevReactPwr_T_A_OFF != 0) )
   {
     RevReacPwrTripTmr = 0;
     RevVarTripPuFlg = FALSE;
     return;
   }

   // Reverse Reactive Power protection is enabled

   // For regular feed, negative power is "reverse" and positive power is "forward".  In this case, we need
   //   to flip the sign of the power.  This way, "reverse" power is positive, and "forward" power is
   //   negative.  The setpoint is positive, so "forward" power will never trigger.  We are only checking
   //   for high values of reverse power. 
   // For reverse feed, negative power is "forward", and positive power is "reverse", so the sign is ok as
   // is.
   tempRPa = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPa : -PwrOneCyc.RPa;
   tempRPb = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPb : -PwrOneCyc.RPb;
   tempRPc = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPc : -PwrOneCyc.RPc;

   // Compute max absolute value of reverse power.  Store value in tempRPa.  Store signed value in
   //   MaxPwrOneCycRevVar (for disturbance capture)
   MaxPwrOneCycRevVar = PwrOneCyc.RPa;
   if (tempRPb > tempRPa)
   {
     tempRPa = tempRPb;
     MaxPwrOneCycRevVar = PwrOneCyc.RPb;
   }
   if (tempRPc > tempRPa)
   {
     tempRPa = tempRPc;
     MaxPwrOneCycRevVar = PwrOneCyc.RPc;
   }

   // Now do protection using the absolute value of the max reverse power
   if (tempRPa >= Setpoints5.stp.RevReactPwr_Pu * 1000)
   {                                // Above reverse power pickup
      if (RevReacPwrTripTmr >= (Setpoints5.stp.RevReactPwr_Tt * Setpoints0.stp.Freq / 100))
      {                             // Trip delay expired - Trip
        RevReacPwrTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        RevReacPwrTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_REVERSE_KVAR);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (RevVarTripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_REV_VAR].EIDofStartEvnt = InsertNewEvent(REVVAR_PICKUP);
          Dist_Flag |= DS_RKVAR;
          RevVarTripPuFlg = TRUE;               // Set flag to True
        }                                        
        RevReacPwrTripTmr++;            // Increment trip timer
      }
   }
   else                                 // Below reverse power pickup
   {
     if (Dist_Flag & DS_RKVAR)                  // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( RevReacPwrTripTmr < ((Setpoints5.stp.RevPwr_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(REVERSE_KVAR_EXIT);
         Dist_Flag_Cancel |= DS_RKVAR;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         RevVarTripBucket = (uint16_t)( ((uint32_t)RevReacPwrTripTmr * 1000) /
             ((uint32_t)Setpoints5.stp.RevReactPwr_Tt * (uint32_t)Setpoints0.stp.Freq) );
         RevVarTripBucket = ( (RevVarTripBucket > 1000) ? 1000 : RevVarTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_RKVAR ^ 0xFFFFFFFF);
     }
     RevReacPwrTripTmr = 0;             // Reset timer
     RevVarTripPuFlg = FALSE;           // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             RevReactivePower_Prot()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PhaseRotation_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Phase Rotation Prot function (Reverse Sequence trip)
//
//  MECHANICS:          This routine examines the rotation direction flags set by the Line-Neutral Voltage
//                      Sequence components.  If the Positive Sequence component is greater than the Neqative
//                      Sequence component, the rotation is ABC.  Otherwise, it is CBA. This phase rotation is
//                      compared to the RevSeq_Type setpoint value.
//
//                      If the rotation sequence matches the trip setpoint value, the RevSeqTripTmr starts.  If
//                      it times out based on a fixed delay of 10 cycles, a trip will be initiated.
//
//                      Perform this routine if the breaker is open or closed since correct phase rotation is a
//                      permissive condition for closing a breaker.
//
//                      A minimum line-to-line voltage of 84vac is required for this protection to be active.
//                      If this conditions is not met, RevSeqTripFlg and RevSeqTripTmr are cleared.  One-cycle
//                      AFE voltages are used.
//
//
//  CAVEATS:            This function is called from Main() every one cycle if the Phase Rotation T_A_Off setpoint is
//                      set to Trip.
//
//  INPUTS:             Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.RevSeq_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevSeq_Type: [0=ABC, 1=CBA]    <--(trips on these sequences)
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_min, SeqComp.V_PosMag, SeqComp.V_NegMag
//
//  OUTPUTS:            RevSeqTripFlg, AlarmHoldOffTmr
//
//  ALTERS:             ABC_Rotation, ACB_Rotation, RevSeqTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void PhaseRotation_Prot(void)
{
   // If Phase Rotation protection is not enabled, reset the timer and return
   //   Note, don't clear the trip flag in case a trip had occurred and hasn't been cleared yet
   if ((Setpoints5.stp.ExtProtEnable != 1) || ( Setpoints5.stp.RevSeq_T_A_OFF != 0)) 
   {
      RevSeqTripTmr = 0;
      return;
   }

   // Phase Rotation protection is enabled

   if (Vll_min > 84)
   {
      if (SeqComp.V_PosMag > SeqComp.V_NegMag)
      {
         ABC_Rotation = 1;
         ACB_Rotation = 0;
      }
      else
      {
         ABC_Rotation = 0;
         ACB_Rotation = 1;
      }

      switch (Setpoints5.stp.RevSeq_Type)        // 0=ABC, 1=CBA
      {
         case 0:                                 // Set to trip on ABC
            if (ABC_Rotation == 1)               // Wrong rotation: increment timer
            {
               RevSeqTripTmr++;
            }
            else if (ACB_Rotation == 1)          // Correct rotation: clear timer
            {
               RevSeqTripTmr = 0;
            }
            break;

         case 1:                                 // Set to trip on ACB/CBA
            if (ACB_Rotation == 1)               // Wrong rotation: increment timer
            {
               RevSeqTripTmr++;
            }
            else if (ABC_Rotation == 1)          // Correct rotation: clear timer
            {
               RevSeqTripTmr = 0;
            }
            break;

         default:
            RevSeqTripTmr = 0;
          break;
      }


      if (RevSeqTripTmr >= 10 * Setpoints0.stp.Freq)
      {                                         // Trip delay timed out after 10 cycles - Trip
        RevSeqTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        InsertNewEvent(TRIP_REVERSE_SEQUENCE);  // Put an entry into the new event FIFO
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;                  // Hold off alarm functions for 2s
        RevSeqTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                        // Set general trip cause flag
        Prot_Enabled = FALSE;                   // Turn off protection
        Prot_Timer = 25;                        // Hold off next trip for 250ms
      }
      else
      {
        RevSeqTripTmr++;                        // Increment trip timer
      }
   }
   else                                         // Voltage too low to run protection
   {                                            //   Note, don't clear the trip flag in case a trip had
      RevSeqTripTmr = 0;                        //     occurred and hasn't been cleared yet
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PhaseRotation_Prot()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PF_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Power Factor Protection Function
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If the Power Factor value is below the UnderPF_Pu setpoint for a duration
//                      in excess of the UnderPF_Tt, set TripReqFlg and PFTripFlg.
//                      NOTE: The power factors may be signed values.  This subroutine uses the magnitude
//                      (absolute value) of each power factor!!
//
//                      Above UnderPF_Pu, clear the UnderPFTripTmr.
//
//                      200ms Power Factor values are used.  The power factors for each phase are calculated
//                      in Calc_AppPF().
//
//  CAVEATS:            This function is called from Main() if the Under PF T_A_Off setpoint is set to Trip.
//
//  INPUTS:             Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.UnderPF_T_A_OFF
//                      Setpoints6.stp.UnderPF_Pu: [0.20 to 0.95, step=0.05]
//                      Setpoints6.stp.UnderPF_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PF.App[0..2]
//
//  OUTPUTS:            PFTripFlg, TripReqFlg, PFTripBucket, AlarmHoldOffTmr
//
//  ALTERS:             UnderPFTripTmr
//
//  CALLS:              TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------
//
void PF_Prot(void)
{
  float tempa, tempb, tempc;

  // If Power Factor protection is not enabled, reset the timer and pickup flag, and return
  if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.UnderPF_T_A_OFF != 0) || (Vll_min < 84) || (CurOneCycMax < 40))
  {
    UnderPFTripTmr = 0;
    UPFTripPuFlg = FALSE;
    return;
  }

   // Power Factor protection is enabled

  // Take absolute value of the power factors
  tempa = ( (PF.App[0] < 0) ? (-PF.App[0]) : PF.App[0]);
  tempb = ( (PF.App[1] < 0) ? (-PF.App[1]) : PF.App[1]);
  tempc = ( (PF.App[2] < 0) ? (-PF.App[2]) : PF.App[2]);

  // Store minimum power factor (signed value) in MinProtPF and absolute value in tempa
  MinProtPF = PF.App[0];
  if (tempb < tempa)
  {
    MinProtPF = PF.App[1];
    tempa = tempb;
  }
  if (tempc < tempa)
  {
    MinProtPF = PF.App[2];
    tempa = tempc;
  }

  tempa = 100 * tempa;                // to account for pickup setpoint being X100
  
  // Do protection function.  Check whether min PF is less than or equal to the setting
  if ((tempa <= Setpoints6.stp.UnderPF_Pu) && (tempa != NAN))  
  {                                // Below Power Factor pickup
      if (UnderPFTripTmr >= (Setpoints6.stp.UnderPF_Tt * Setpoints0.stp.Freq))
      {                             // Trip delay expired - Trip
        UnderPFTripTmr = 0;
        TripFlagsReset();
        Reset_ProtRelated_Alarm();
        TA_TRIP_ACTIVE;
        TA_Timer = 24;
        AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
        PFTripFlg = 1;
        TripReqFlg  = 1;
        BellTripFlg = 1;                // Set general trip cause flag
        Prot_Enabled = FALSE;           // Turn off protection
        Prot_Timer = 25;                // Hold off next trip for 250ms
        if (!Trip_WF_Capture.InProg)          // Set the trip waveform capture req flag
        {
          Trip_WF_Capture.Req = TRUE;
        }
        InsertNewEvent(TRIP_UNDERPF);
      }
      else                              // Under trip delay.
      {
        // Otherwise, if we have just entered pickup, insert an entry event and initiate a disturbance
        //   capture.  Note, the disturbance capture is started immediately, regardless of the delay timer,
        //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
        //   unit trips or the delay time does not reach 50% of the delay time setting
        if (UPFTripPuFlg == FALSE)
        {
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_UNDER_PF].EIDofStartEvnt = InsertNewEvent(UPF_PICKUP);
          Dist_Flag |= DS_UPF;
          UPFTripPuFlg = TRUE;                  // Set flag to True
        }                                        
        UnderPFTripTmr++;               // Increment trip timer
      }
   }
   else                                 // Above PF pickup
   {
     if (Dist_Flag & DS_UPF)                    // If a disturbance capture was initiated...
     {
       // If the delay time never reached 50% of the setting, insert an exit event and abort the disturbance
       //   capture.
       if ( UnderPFTripTmr < ((Setpoints6.stp.UnderPF_Tt * Setpoints0.stp.Freq) / 200) )
       {
         InsertNewEvent(UNDERPF_EXIT);
         Dist_Flag_Cancel |= DS_UPF;
       }
       // Otherwise we will do a disturbance capture entry - compute the trip bucket value (how close we
       //   came to tripping.  This is just the delay time divided by the threshold time
       //   The value is computed in tenths of a percent and is clamped at 100%
       else
       {
         PFTripBucket = (uint16_t)( ((uint32_t)RevReacPwrTripTmr * 1000) /
             (Setpoints6.stp.UnderPF_Tt * Setpoints0.stp.Freq) );
         PFTripBucket = ( (PFTripBucket > 1000) ? 1000 : PFTripBucket);
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if it is not aborted)
       //   This will also cause an exit event to be generated
       Dist_Flag &= (DS_UPF ^ 0xFFFFFFFF);
     }
     UnderPFTripTmr = 0;                            // Reset timer
     UPFTripPuFlg = FALSE;                          // Reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PF_Prot()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Temp_Prot()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Perform temperature protection
//
//  MECHANICS:          If the measured temperature (THSensor.Temperature) is above the trip limit (105 degC)
//                      in Prot_def.h , trip the breaker & set the appropriate trip flags.
//
//                      The alarm threshold is programmed as a setpoint.  There is a 5 degC hysteresis built in
//                      for clearing the alarm.
//
//                      There is a Display temperature alarm (70 degC) which defined in Prot_def.h based on the
//                      maximum operating temperature of the Display.  A flag will be set to turn off the display
//                      and provide a warning.  An output relay can be mapped to this warning.  There is a 5 degC
//                      hysteresis built in for clearing the alarm.
//
//  CAVEATS:            This function is called from Main() every second.  Trip protection is always enabled, as is the display
//                      temperature warning.  The alarm function is enabled/disabled by the Setpoints9.stp.OverTempAlm
//                      setpoint.
//
//  INPUTS:             Setpoints9.stp.OverTempAlm: [0 = Disabled, 1 = Enabled]
//                      Setpoints9.stp.OverTempAlm_Pu: [85 - 105, step 5] degrees C
//                      THSensor.Temperature
//
//  OUTPUTS:            TempTripFlg, BellTripFlg, TripReqFlg, TempAlmFlg, DispTempAlmFlg, AlarmHoldOffTmr
//
//  ALTERS:
//
//  CALLS:              TripFlagsReset(), InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------

void Temp_Prot(void)
{
   if (THSensor.Temperature >= TEMP_TRIP_THRESHOLD)     // If trip threshold reached...
   {
      TripFlagsReset();
      Reset_ProtRelated_Alarm();
      TA_TRIP_ACTIVE;
      TA_Timer = 24;
      AlarmHoldOffTmr = 200;        // Hold off alarm functions for 2s
      TempTripFlg = 1;
      BellTripFlg = 1;
      TripReqFlg = 1;
      Prot_Enabled = FALSE;             // Turn off protection
      Prot_Timer = 25;                  // Hold off next trip for 250ms
      // Insert event into queue.  Note, we will not initiate a waveform capture for overtemperature trips,
      //   just summary and snapshots
      InsertNewEvent(TRIP_OVERTEMPERATURE);
   }
   else                                                 // Otherwise check for alarm...
   {
     if ((THSensor.Temperature >= Setpoints9.stp.OverTempAlm_Pu) && (TempAlmFlg == 0)
      && (Setpoints9.stp.OverTempAlm == 1))
     {
        TripFlagsReset();                                   // to allow alarm to be logged
        TempAlmFlg = 1;
        if (TU_State_TestUSBMode == 1)      // if in test mode, insert minor alarm event, set the secondary
        {                                   //   as 0x03 -- test mode
           InsertNewEvent(ALARM_OVERTEMP_TEST);
        }
        else                                // Otherwise generate alarm and disturbance events but no
        {                                   //   waveform capture
          // Save the EID of the event that started the disturbance capture, and insert an entry event
           sListOfDist[DS_HIGH_TEMP].EIDofStartEvnt = InsertNewEvent(ALARM_OVERTEMPERATURE_ENTRY);

           // Initiate a disturbance capture
           Dist_Flag |= DS_HT;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
        }
     }
                                                                                         // 5 degC hysteresis
     if ((THSensor.Temperature < Setpoints9.stp.OverTempAlm - 5) && (TempAlmFlg == 1)
      && (Setpoints9.stp.OverTempAlm == 1))
     {
       // Clear the Dist_Flag so the disturbance capture is completed (if one had been started)
       Dist_Flag &= (DS_HT ^ 0xFFFFFFFF);
       TempAlmFlg = 0;
     }

     if (THSensor.Temperature >= DISPLAY_TEMP_WARNING_SET)
     {
        TripFlagsReset();                                            // to allow alarm to be logged
        DispTempAlmFlg = 1;                                          // set flag which provide warning and turns off display
     }

     if (THSensor.Temperature < DISPLAY_TEMP_WARNING_CLEAR)
     {
        DispTempAlmFlg = 0;                                          // clear flag which clears warning and turns on display
     }
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Temp_Prot();
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//  START OF FUNCTION               Ground_Fault_PreAlarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Ground Fault Pre-alarm function
//
//  MECHANICS:          If the ground current is above the GF_Pre_Alarm setpoint:
//                        - Set the GfPreAlarmFlg.  This will energize the Alarm relay if it it programmed
//                          for GF Pre-alarm.
//                          Note: the 1200A limitation doesn't apply to Ground Fault Pre-alarm
//                        - Insert either a summary event (test mode) or alarm (otherwise) event
//
//                      There is 5% hysteresis for turning off the alarm. There is no delay.
//
//  CAVEATS:            This function is called from Main() every half-cycle if the GF T_A_Off setpoint is
//                      set to Trip or Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints1.stp.Gf_T_A_OFF
//                      Setpoints6.stp.GF_Pre_Alarm: [50 to 100, step=5] %
//                      CurOneCycIg, IntEventOutNdx
//
//  OUTPUTS:            GfPreAlarmFlg, A_Scope_Req, IntEventBuf[], IntEventInNdx, IntEventBufFull
//
//  ALTERS:
//
//  CALLS:              TripFlagsReset(), Get_InternalTime()
//
//------------------------------------------------------------------------------------------------------------

void Ground_Fault_PreAlarm(void)
{
  // If Ground Fault Pre-alarm is not enabled or is being held off, reset the alarm flag and return
  if ((!GF_Enabled) || (Setpoints1.stp.Gf_T_A_OFF > 1) || (AlarmHoldOffTmr > 0) )
  {
    GfPreAlarmFlg = 0;
    return;
  }

  // Ground Fault Pre-alarm function is enabled and not held off

  // GF Pre-alarm pickup based on In
  if (CurOneCycIg >= Break_Config.config.Rating * Setpoints1.stp.GF_Pre_Alarm / 100)
  {
    if (GfPreAlarmFlg == 0)
    {
      TripFlagsReset();
      GfPreAlarmFlg = 1;
      if (TU_State_TestUSBMode == 1)               // If in test mode, insert summary event
      {
        // Put an entry into the new event FIFO (for interrupts)
        IntEventBuf[IntEventInNdx].Code = ALARM_GND_FAULT_PRE_TEST;
        Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
        IntEventInNdx &= 0x07;
        if (IntEventInNdx == IntEventOutNdx)
        {
          IntEventBufFull |= TRUE;
        }         
      }
      else                                         // Otherwise it is an alarm event, so initiate
      {                                            //   capture and insert event
        A_Scope_Req = 1;
        // Put an entry into the new event FIFO (for interrupts)
        IntEventBuf[IntEventInNdx].Code = ALARM_GND_FAULT_PRE;
        Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
        IntEventInNdx &= 0x07;
        if (IntEventInNdx == IntEventOutNdx)
        {
          IntEventBufFull |= TRUE;
        }         
      }
    }
  }
  else if ( (GfPreAlarmFlg == 1)
        && (CurOneCycIg <= Break_Config.config.Rating * (Setpoints1.stp.GF_Pre_Alarm - 5) / 100) )
  {
    GfPreAlarmFlg = 0;
  }
}

//------------------------------------------------------------------------------
//  END OF FUNCTION         Ground_Fault_PreAlarm()
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------
//  START OF FUNCTION       Ground_Fault_Alarm()
//------------------------------------------------------------------------------
//
//  FUNCTION:           Ground Fault Alarm
//
//
//  MECHANICS:          This subroutine performs ground fault alarm protection every sample period (208usec at 60Hz).
//
//                      If above pickup (ig_sum32 >= Gfpu_Val) set zone out pin & GfpuFlg.
//                      With no interlock in, trip after 4 successive passes.
//
//                      With an interlock in:
//                         For a flat response or i2t response above 0.625pu, increment the number of passes
//                         For an i2t response below 0.625pu, calculate GF_Tally = GF_Tally plus the latest sample squared.
//                            If GF_Tally >= the tally trip value, set TripReqFlg  and GndTripFlg.
//
//                              Note: the 1200A limitation doesn't apply to Ground Fault Alarm
//
//                      On trip a memory effect "cooling" is implemented by loading GF_Tally with the trip
//                      threshold value.  Below pickup, GF_Tally is decreased by 0.25pu each pass.
//
//  CAVEATS:            Called from DMA1_Stream0_IRQHandler() in Intr.c if Setpoints1.stp.Gf_T_A_OFF is set
//                      to 1 (alarm)
//
//
//  INPUTS:             Setpoints1.stp.Gnd_Type: [0=residual, 1=source]
//                      Setpoints1.stp.Gf_Slp: [0=flat, 1=I2t]
//                      Setpoints1.stp.Gf_Pu: [0.2 to 1.0, step=0.01]
//                      Setpoints1.stp.Gf_Tt: [Flat: 0.05 to 1.0, step=0.01] sec
//                                            [I2t:  0.1 to 1.0, step=0.01] sec
//                      Setpoints1.stp.ThermMem: [0=off, 1=on]
//                      GF_HalfCycPickup, GF_OneCyc_0p625, GF_TallyDecr
//                      GF_Tally, GF_TripThresholdFlat, GF_TripThresholdI2t
//                      CurHalfCycSOS_SumF.Igres, CurHalfCycSOS_SumF.Igsrc
//                      CurOneCycSOS_SumF.Igres, CurOneCycSOS_SumF.Igsrc
//                      Igres, AFE_new_samples[4]
//                      IntEventOutNdx
//
//  OUTPUTS:            Dist_Flag_GF, Dist_Flag_Cancel_GF, IntEventBuf[], IntEventInNdx, IntEventBufFull
//
//  ALTERS:             GfpuFlg, GndAlmFlg
//
//  CALLS:              Get_InternalTime()
//
//------------------------------------------------------------------------------------------------------------
//
void Ground_Fault_Alarm(void)
{
  float CurIgHalfCycSOS;
  float CurIgOneCycSOS;
  float NewSampleIg;
                                                // do GF protection on either residual ground or source ground
  CurIgHalfCycSOS = (Setpoints1.stp.Gnd_Type == 0) ? CurHalfCycSOS_SumF.Igres : CurHalfCycSOS_SumF.Igsrc;
  CurIgOneCycSOS = (Setpoints1.stp.Gnd_Type == 0) ? CurOneCycSOS_SumF.Igres : CurOneCycSOS_SumF.Igsrc;
  NewSampleIg = (Setpoints1.stp.Gnd_Type == 0) ? Igres_Protsample : AFE_new_samples[4];


  switch (GF_State)
  {
    case 0:                             // Default state is 0
    default:
      if (CurIgHalfCycSOS >= GF_HalfCycPickup) // If above pickup level..
      {
        GfpuFlg = 1;   //PickupFlags |= GF_PICKUP;               // Set pickup flag
        GF_Passes = 40;                         // Initialize number of passes to one-half cycle
        GF_Tally = CurIgHalfCycSOS;             // Initialize I2t tally register to the max 1/2-cycle's SOS
        // Initiate a disturbance capture.  Note, this is started immediately so that measurements are taken
        //   over the entire disturbance.  The capture is abandoned if an alarm is not generated
        Dist_Flag_GF = TRUE;
        GF_State = 1;
      }
      else                                      // If below pickup level, either clear or decrement GF_Tally
      {
        GfpuFlg = 0;   //PickupFlags &= (~GF_PICKUP);
        if (Setpoints1.stp.ThermMem == 0)
        {
          GF_Tally = 0.0;
          GF_Passes = 0;
        }
        else                                    // Decrement GF_Tally Thermal Memory is on
        {
          GF_Tally -= GF_TallyDecr;

          if (GF_Tally < 0)                      // Limit GF_Tally to zero.
          {
            GF_Tally = 0.0;
            GF_Passes = 0;
          }
        }
      }

      break;

    case 1:                             // State 1
      GF_Passes++;                              // Check for one-cycle anniversary
      if (GF_Passes >= 120)                     // If one cycle anniversary...
      {                                             // Set up for State 2:
        GF_Passes = 119;                                              // Set number of passes to 120 - 1
        GF_Tally += CurIgOneCycSOS - (NewSampleIg * NewSampleIg);     // Update GF_Tally with one cycle SOS
        GF_State = 2;                                                 //   less the new sample SOS
      }
      else
      {
        break;
      }

    case 2:                             // State 2
      if (CurIgOneCycSOS >= GF_OneCycPickup)            // If above the pickup level...
      {
        GF_Passes++;                                    // Increment number of passes


        // If Flat protection or if above 0.625x...
        if ((GF_Slope == GF_FLAT) || (CurIgOneCycSOS >= GF_OneCyc_0p625))
        {
          if (GF_Passes >= GF_TripThresholdFlat)        // If tripping...      *** DAH  ADD COMPENSATION FOR STARTUP TIME IF FIRST TIME AFTER RESET
          {
            if (GndAlmFlg == 0)
            {
              TripFlagsReset();                                   // to allow alarm to be logged
              GndAlmFlg = 1;
              GF_Passes = 0;
              GF_State = 0;
              A_Scope_Req = 1;        // Start oscillographic capture
              // Put an entry into the new event FIFO (for interrupts)
              IntEventBuf[IntEventInNdx].Code = ALARM_GROUND_FAULT;
              Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
              IntEventInNdx &= 0x07;
              if (IntEventInNdx == IntEventOutNdx)
              {
                IntEventBufFull |= TRUE;
              }         
            }
          }
        }
        else                                        // Otherwise I2t protection...
        {
          GF_Tally += (NewSampleIg * NewSampleIg);      // Update GF_Tally with the latest sample SOS
          if ( (GF_Passes >= GF_MIN_PASSES_FOR_TRIP)    // Cannot trip unless in pickup for a minimum
            && (GF_Tally >= GF_TripThresholdI2t) )      //   number of passes and I2t tally exceeded
          {
            if (GndAlmFlg == 0)
            {
              TripFlagsReset();                                   // to allow alarm to be logged
              GndAlmFlg = 1;
              GF_Passes = 0;
              GF_State = 0;
              A_Scope_Req = 1;        // Start oscillographic capture
              // Put an entry into the new event FIFO (for interrupts)
              IntEventBuf[IntEventInNdx].Code = ALARM_GROUND_FAULT;
              Get_InternalTime(&IntEventBuf[IntEventInNdx++].TS);
              IntEventInNdx &= 0x07;
              if (IntEventInNdx == IntEventOutNdx)
              {
                IntEventBufFull |= TRUE;
              }         
            }
          }
        }
      }
      else                                      // If not in pickup, clear the pickup flag and set next the
      {                                         //   state to 0 to start over
        // If a disturbance capture was initiated...
        if (Dist_Flag_GF)
        {
          // If the alarm flag is not set, abort the the disturbance capture because no alarm event occurred
          //   (the delay time wasn't met).  If the alarm flag is set, we will end the capture and log the
          //   results
          if (!GndAlmFlg)
          {
            Dist_Flag_Cancel_GF = TRUE;
          }
          // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
          Dist_Flag_GF = FALSE;
        }
        GndAlmFlg = 0;
        GfpuFlg = 0;   //PickupFlags &= (~GF_PICKUP);
        GF_State = 0;
      }
      break;

  }

}
//------------------------------------------------------------------------------
//  END OF FUNCTION                    Ground_Fault_Alarm()
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Sneakers_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Sneakers Alarm Function.
//                      A sneakers condition is defined as an incomplete closure (i.e. a closure with
//                      inadequate contact pressure). This can cause arcing and overheating. This condition
//                      can be detected because the aux switch will continue to indicate that the breaker is
//                      open while current is flowing.
//
//  MECHANICS:          If the aux switch indicates the breaker is open...
//                            CurOneCycSOSmax > (0.0625 Ldpu), the sneakers alarm is in pickup.  After 2 secounds,
//                                  set the SneakersAlmFlg.
//                            CurOneCycSOSmax < (0.0625 Ldpu), clear SneakersAlmFlg and reset the pickup timer.
//
//                      Note: 0.0625 Ldpu which is Ldpu/16 is used in previous products.  Keep in mind that these
//                            values are squared so the current really needs to be above 25% of it's pickup value
//
//                      This routine is called every one cycle anniversary.  A 2 sec timeout will take
//                      2 x 60 = 120 passes at 60Hz, or 100 passes at 50Hz.
//
//  CAVEATS:
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints9.stp.SneakersAlm
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurOneCycSOSmax, LD_OneCycPickup, OpenFlg
//
//  OUTPUTS:            Nothing
//
//  ALTERS:             SneakAlmTmr, SneakersAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void Sneakers_Alarm(void)
{
   // If Sneakers alarm is not enabled or is being held off, reset the timer and alarm flag, and return
   if ( (Setpoints9.stp.SneakersAlm != 1) || (AlarmHoldOffTmr > 0) )
   {
     SneakAlmTmr = 0;
     SneakersAlmFlg = 0;
     return;
   }

   // Sneakers alarm function is enabled and not held off

   if (OpenFlg == 1)                // Run sneakers protection only if aux switch indicates breaker open
   {                                   // Breaker open...
      if (CurOneCycSOSmax >= LD_OneCycPickup/16)   // Imax > 0.0625 Ldpu
      {
                                       // Delay timer (2 sec) timed out yet?
         if (SneakAlmTmr > (Setpoints0.stp.Freq * 2))
         {
            if (SneakersAlmFlg == 0)
            {
               SneakAlmTmr = 0;
               TripFlagsReset();                                   // to allow alarm to be logged
               SneakersAlmFlg = 1;
               A_Scope_Req = 1;        // Start oscillographic capture
      //         InsertNewEvent(ALARM, ((Uint32)CAUSE_MECH << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
            }
         }
         else                          // Delay timer running...
         {
            SneakAlmTmr++;
         }
      }
      else
      {                                // CurOneCycSOSmax < 0.0625 Ldpu - clear sneakers indication
         if (SneakersAlmFlg == 1)
         {
            SneakersAlmFlg = 0;
         }
         SneakAlmTmr = 0;              // Reset timer
      }
   }
   else                                // Breaker closed...
   {
      if (SneakersAlmFlg == 1)
      {
         SneakersAlmFlg = 0;
      }
      SneakAlmTmr = 0;                 // Reset timer
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Sneakers_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//  START OF FUNCTION       Thermal_Mem_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Thermal Memory Alarm function
//
//  MECHANICS:          A delicated RC circuit is used to provide unpowered thermal memory to the trip unit.
//                      The voltage on the capacitor is regulated such that it represents the percentage of the
//                      long-delay trip value (Ltally_Trip) presently in the long-delay trip accumulator.
//
//                      LD_Tally is updated in UpdateThermalMemory() at power-up if Thermal Memory is enabled. 
//
//                      The alarm is settable with the Thermal Memory Alarm setpoint if Thermal Memory is enabled.
//                      There is 5% hysteresis for turning off the alarm.
//
//                      This routine is called every one cycle anniversary.
//
//  CAVEATS:
//
//
//  INPUTS:             Setpoints1.stp.ThermMem_Alarm: [50 to 100, step=5] %
//                      Setpoints1.stp.ThermMem: [0=disabled, 1=enabled]
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      LD_Tally, LD_TripThreshold
//
//  OUTPUTS:            TherMemAlmFlg
//
//  ALTERS:             TherMemAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void Thermal_Mem_Alarm(void)
{
  if ((Setpoints1.stp.ThermMem == 1) && (LdPuAlmFlg == 0))          // Only when we're not in pickup and setpoint is enabled
  {
     if (LD_Tally >= (LD_TripThreshold * Setpoints1.stp.ThermMem_Alarm / 100))           // Thermal Memory Alarm
     {
        if (TherMemAlmTmr >= (THER_MEM_ALM_TIME * Setpoints0.stp.Freq))
        {
           if (TherMemAlmFlg == FALSE)
           {
              TherMemAlmTmr = 0;
              TripFlagsReset();                                   // to allow alarm to be logged
              TherMemAlmFlg = 1;
              if (TU_State_TestUSBMode == TRUE)
              {
    //            InsertNewEvent(MINOR_ALARM, (((uint32_t)CAUSE_T_MEM) << 16) + 0x0300 + ((OpenFlg == 1)? 1 : 2));
              }
              else
              {
                A_Scope_Req = 1;
      //          InsertNewEvent(ALARM, (((uint32_t)CAUSE_T_MEM) << 16) + 0x0800 + ((OpenFlg == 1)? 1 : 2));
              }
           }
        }
        else                          // Under alarm delay.
        {
           TherMemAlmTmr++;           // Increment alarm timer
        }
     }
     else if (LD_Tally < (LD_TripThreshold * (Setpoints1.stp.ThermMem_Alarm - 5) / 100))     // below hysteresis level
     {
        TherMemAlmTmr = 0;
        TherMemAlmFlg = 0;
     }
  }
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         Thermal_Mem_Alarm()
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  START OF FUNCTION       Highload_Alarm()
//------------------------------------------------------------------------------
//  FUNCTION:           Highload Alarm function
//
//  MECHANICS:          There are two High Load setpoints: 1) High Load Alarm 1: 50% to (HL2-5%) of Ir
//                                                         2) High Load Alarm 2: (HL1+5%) to 120% of Ir
//
//                      The alarm is active when the maximum one cycle current is above the setpoint for a programmable delay.
//                      Hysteresis will turn the alarm off at 5% below the setpoint value.
//
//                      This routine is called every one cycle anniversary.
//
//  CAVEATS:
//
//
//  INPUTS:             
//                      Setpoints1.stp.HL1_Action: [0=alarm, 1=alarm with extended capture, 2=off]
//                      Setpoints1.stp.HL2_Action: [0=alarm, 1=alarm with extended capture, 2=off]
//                      Setpoints1.stp.HL_Alarm_1: [50 to (HL2-5), step=1] %
//                      Setpoints1.stp.HL_Alarm_2: [(HL+5) to 120, step=1] %
//                      Setpoints1.stp.HL1_Time: [1 to 60, step=1] sec
//                      Setpoints1.stp.HL2_Time: [1 to 60, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurOneCycMax, Ir
//
//
//  OUTPUTS:            HlAlm1Flg, HlAlm2Flg
//
//  ALTERS:             HlAlm1Tmr, HlAlm2Tmr
//
//  CALLS:              InsertNewEvent(), TripFlagsReset()
//
//------------------------------------------------------------------------------------------------------------

void Highload_Alarm(void)
{
/*  if (dfred_on)                                  // *** DAH TEST START - FORCES HL1 OR HL2 OR BOTH
  {                                                // DON'T FORGET TO RESET DFRED_ON, EITHER IN THE HL1 OR HL2 CODE BELOW
    dfred_test += 1.0f;
  }
  else
  {
    dfred_test = ((dfred_test > 0) ? (dfred_test - 1.0f) : 0);
  }
  CurOneCycMax = dfred_test;             //      *** DAH END OF TEST CODE   */

  //--------------------------------------------------------------------------------------------------------
  //                                           High Load 1
  //
  if ( (CurOneCycMax >= Ir * Setpoints1.stp.HL_Alarm_1 / 100) && (Setpoints1.stp.HL1_Action < 2) )      // above High Load 1 pick up?
  {
     if (HlAlm1Tmr >= (Setpoints1.stp.HL1_Time * Setpoints0.stp.Freq) )
     {
        if (HlAlm1Flg == 0)
        {
           HlAlm1Tmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           HlAlm1Flg = 1;
           if (TU_State_TestUSBMode == TRUE)
           {
             InsertNewEvent(ALARM_HL1_ENTRY_TEST);                       // Insert event into queue
           }
           else if (Setpoints1.stp.HL1_Action == 0)             // If alarm on High Load 1...
           {
             if (!Alarm_WF_Capture.InProg)                               // Set alarm waveform capture req
             {                                                           //   flag
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_HIGHLOAD_1].EIDofStartEvnt = InsertNewEvent(ALARM_HIGHLOAD1_ENTRY);
           }
           else if (Setpoints1.stp.HL1_Action == 1)             // If extended capture on High Load 1...
           {
             // Set the flag to do an extended capture.  Note, this will cause an extended capture event to
             //   be inserted into the queue in ExtendedCapture()
             ExtCap_ReqFlag |= EC_HL1;                          // set flag for extended capture
             // Set the flag to initiate a GOOSE capture message if one is not already in progress.
             //   Goose_Capture_Code stores which process initiated the capture, so that the command to
             //   terminate the capture will only come from the same process.
             // Note, the GOOSE Rx subroutine in the sampling interrupt could also set the flag. It
             //   shouldn't matter if this is missed, because we only do one extended capture anyway, so we
             //   don't need to disable interrupts around this statement
             if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code > DS_LAST)
               && (Setpoints13.stp.GC_GlobalCapEnable & 0x01) )
             {
               PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 1;
               DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;           // *** DAH  WE WILL LIKELY NEED TO INCLUDE WHICH PARAMETER TO MONITOR FOR THE DISTURBANCE CAPTURE!
               Goose_Capture_Code = DS_HIGHLOAD_1;
             }
           }
//           dfred_on = 0;  // *** DAH TEST CODE
        }
     }
     else                          // Under alarm delay
     {
        HlAlm1Tmr++;               // Increment alarm timer
     }

     // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
     //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the delay
     //   time is not reached
     Dist_Flag |= DS_HL1;          // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
  }
  else if ( (CurOneCycMax <= Ir * (Setpoints1.stp.HL_Alarm_1 - 5) / 100) || (Setpoints1.stp.HL1_Action == 2) )     // below hysteresis level
  {
     if (Dist_Flag & DS_HL1)                // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the disturbance capture because no alarm event occurred (the delay
       //   time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if ((!HlAlm1Flg) && (Setpoints1.stp.HL1_Action < 2))
       {
         Dist_Flag_Cancel |= DS_HL1;
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if one had been started)
       Dist_Flag &= (DS_HL1 ^ 0xFFFFFFFF);
       // Set the flag to send a GOOSE capture message to terminate the capture if this process is in control 
       if (Goose_Capture_Code == DS_HIGHLOAD_1)
       {
         PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
         DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
         Goose_Capture_Code = DS_LAST + 1;
       }
     }
     HlAlm1Tmr = 0;
     HlAlm1Flg = 0;
  }
  //
  //                                         End High Load 1
  //--------------------------------------------------------------------------------------------------------



  //--------------------------------------------------------------------------------------------------------
  //                                           High Load 2
  //
  if ( (CurOneCycMax >= Ir * Setpoints1.stp.HL_Alarm_2 / 100) && (Setpoints1.stp.HL2_Action < 2) )      // above High Load 2 pick up?
  {
     if (HlAlm2Tmr >= (Setpoints1.stp.HL2_Time * Setpoints0.stp.Freq) )
     {
        if (HlAlm2Flg == 0)
        {
           HlAlm2Tmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           HlAlm2Flg = 1;
           if (TU_State_TestUSBMode == TRUE)
           {
             InsertNewEvent(ALARM_HL2_ENTRY_TEST);                       // Insert event into queue
           }
           else if (Setpoints1.stp.HL2_Action == 0)             // If alarm on High Load 2...
           {
             if (!Alarm_WF_Capture.InProg)                               // Set alarm waveform capture req
             {                                                           //   flag
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_HIGHLOAD_2].EIDofStartEvnt = InsertNewEvent(ALARM_HIGHLOAD2_ENTRY);
           }
           else if (Setpoints1.stp.HL2_Action == 1)             // If extended capture on High Load 2...
           {
             // Set the flag to do an extended capture.  Note, this will cause an extended capture event to
             //   be inserted into the queue in ExtendedCapture()
             ExtCap_ReqFlag |= EC_HL2;
             // Set the flag to initiate a GOOSE capture message if one is not already in progress.
             //   Goose_Capture_Code stores which process initiated the capture, so that the command to
             //   terminate the capture will only come from the same process.
             // Note, the GOOSE Rx subroutine in the sampling interrupt could also set the flag. It
             //   shouldn't matter if this is missed, because we only do one extended capture anyway, so we
             //   don't need to disable interrupts around this statement
             if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code > DS_LAST)
               && (Setpoints13.stp.GC_GlobalCapEnable & 0x02) )
             {
               PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 1;
               DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;           // *** DAH  WE WILL LIKELY NEED TO INCLUDE WHICH PARAMETER TO MONITOR FOR THE DISTURBANCE CAPTURE!
               Goose_Capture_Code = DS_HIGHLOAD_2;
             }
//             dfred_on = 0;  // *** DAH TEST CODE
           }
        }
     }
     else                          // Under alarm delay
     {
        HlAlm2Tmr++;               // Increment alarm timer
     }

     // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
     //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the delay
     //   time is not reached
     Dist_Flag |= DS_HL2;               // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
                                        //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT THEN NEED TO
  }                                     //          CLEAR FLAGS IF FEATURE IS UNLOCKED
  else if ( (CurOneCycMax <= Ir * (Setpoints1.stp.HL_Alarm_2 - 5) / 100) || (Setpoints1.stp.HL2_Action == 2) )     // below hysteresis level
  {
     if (Dist_Flag & DS_HL2)                // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the disturbance capture because no alarm event occurred (the delay
       //   time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if ((!HlAlm2Flg) && (Setpoints1.stp.HL2_Action < 2))
       {
         Dist_Flag_Cancel |= DS_HL2;
       }
       // Clear the Dist_Flag so the disturbance capture is completed (if one had been started)
       Dist_Flag &= (DS_HL2 ^ 0xFFFFFFFF);
       // Set the flag to send a GOOSE capture message to terminate the capture if this process is in control 
       if (Goose_Capture_Code == DS_HIGHLOAD_2)
       {
         PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
         DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
         Goose_Capture_Code = DS_LAST + 1;
       }
     }
     HlAlm2Tmr = 0;
     HlAlm2Flg = 0;
  }
  //
  //                                         End High Load 2
  //--------------------------------------------------------------------------------------------------------

}
//------------------------------------------------------------------------------
//  END OF FUNCTION         Highload_Alarm()
//------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
//  START OF FUNCTION       Wrong_Sensor_Alarm_Curr_Condition()
//--------------------------------------------------------------------------------------------------
//  FUNCTION:           Wrong Sensor Alarm Current Condition function
//
//  MECHANICS:          This function determines if we have a high Neutral current and provides the
//                      condtion for the Wrong_Sensor_Alarm() function
//
//                      This routine is called every one cycle anniversary.
//
//  CAVEATS:
//
//  INPUTS:             CurOneCyc.Ia, CurOneCyc.Ib, CurOneCyc.Ic, CurOneCyc.In
//
//  OUTPUTS:
//
//  ALTERS:
//
//  CALLS:
//
//------------------------------------------------------------------------------------------------------------

uint8_t Wrong_Sensor_Alarm_Curr_Condition (void)
{
    if (((CurOneCyc.Ia > 0) && (CurOneCyc.In > 10*CurOneCyc.Ia)) ||
         ((CurOneCyc.Ib > 0) && (CurOneCyc.In > 10*CurOneCyc.Ib)) ||
          ((CurOneCyc.Ic > 0) && (CurOneCyc.In > 10*CurOneCyc.Ic))
            &&  CurOneCyc.In > HIGH_N_CURRENT)
    {
       return TRUE;
    }
    else
    {
       return FALSE;
    }
}

//-------------------------------------------------------------------------------------------------
//  END OF FUNCTION         Wrong_Sensor_Alarm_Curr_Condition()
//-------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------
//  START OF FUNCTION       WrongSensor_Alarm()
//--------------------------------------------------------------------------------------------------
//  FUNCTION:           Wrong Sensor Alarm function
//
//  MECHANICS:          The alarm is active when the Neutral current is excessively high for a 3-pole or 6-pole breaker
//                      and the Neutral Sensor setpoint is set to Rogowski.
//
//                      This routine is called every one cycle anniversary.
//
//  CAVEATS:            The Wrong_Sensor_Alarm_Curr_Condition() needs to be true first.
//
//
//  INPUTS:             Setpoints0.stp.Neutral_Sensor: [0, 1]
//                      Break_Config.config.Poles
//
//
//  OUTPUTS:            WrongSensorAlmFlg
//
//  ALTERS:             Setpoints0.stp.Neutral_Sensor
//
//  CALLS:              Wrong_Sensor_Alarm_Curr_Condition(), InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------

void WrongSensor_Alarm(void)
{                                                                                                                          // Rogowski
    if (Wrong_Sensor_Alarm_Curr_Condition() && ((Break_Config_config_Poles == 3) || (Break_Config_config_Poles == 6)) && Setpoints0.stp.Neutral_Sensor == 0)
    {
        TripFlagsReset();                                   // to allow alarm to be logged 
        WrongSensorAlmFlg = 1;

        if (TU_State_TestUSBMode    == TRUE)
        {
    //        InsertNewEvent(MINOR_ALARM, (((uint32_t)CAUSE_WRONGSENSOR) << 16) + 0x0300 + ((OpenFlg == 1)? PSTATUS_OPEN : PSTATUS_CLOSED));
        }
        else
        {
            A_Scope_Req = 1;
    //        InsertNewEvent(ALARM, (((uint32_t)CAUSE_WRONGSENSOR) << 16) + 0x0800 + ((OpenFlg == 1)? PSTATUS_OPEN : PSTATUS_CLOSED));
        }

        Setpoints0.stp.Neutral_Sensor = 1;                      // set to CT
    }

    WrongSensorAlmFlg = 0;

}
//
//------------------------------------------------------------------------------
//  END OF FUNCTION         WrongSensor_Alarm()
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  START OF FUNCTION       BreakerHealth_Alarm()
//------------------------------------------------------------------------------
//
//  FUNCTION:           Breaker Health Alarm
//
//  MECHANICS:          The Breaker Health Alarm is activated when the remaining health
//                      percentage is below the setpoint value.  The remaining health is 0
//                      when the Life Points reach 10000.
//
//                      The Life Points are a sum of:  Mech Wear points
//                                                     Contact Wear points
//                                                     Time/Temperature points
//
//  CAVEATS:
//
//
//  INPUTS:             Setpoints0.stp.BrkHealth_Level: [0 to 50, step=1] %
//                      Diag_Data.data.Life_Point
//
//
//  OUTPUTS:            BrkHealthAlmFlg
//
//  ALTERS:
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//

void BreakerHealth_Alarm(void)
{
    if (Diag_Data_data_Life_Point >= (10000 - Setpoints0.stp.BrkHealth_Level * 100))
    {
        if (BrkHealthAlmFlg == 0)
        {
            TripFlagsReset();                                   // to allow alarm to be logged
            BrkHealthAlmFlg = 1;
   //         InsertNewEvent(MINOR_ALARM, (((uint32_t)CAUSE_BRK_HEALTH) << 16) + 0x0800 + ((OpenFlg == 1)? 1 : 2));
        }
    }
    else
    {
        if (BrkHealthAlmFlg == 1)
        {
            BrkHealthAlmFlg = 0;
        }
    }
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         BreakerHealth_Alarm()
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//  START OF FUNCTION       BatteryVolt_Alarm()
//------------------------------------------------------------------------------
//
//  FUNCTION:           Battery Voltage Alarm
//
//  MECHANICS:          The battery voltage is measured and calculated once per second
//                      in Calc_BatteryVolt(). The Low Battery alarm will only be logged
//                      if no other alarms are present.  There is no delay time.
//
//                      A Power-up alarm is logged in this function the first time through
//                      after a power-up.
//
//  CAVEATS:
//
//
//  INPUTS:             Vbattery
//                      Alarm_Flags0, Alarm_Flags1, Alarm_Flags2
//                      Flags2.Battery_Valid
//
//  OUTPUTS:            LowBatAlmFlg
//
//  ALTERS:             Flags2.Battery_Valid
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
void BatteryVolt_Alarm(void)
{
  if (Vbattery < BATTERY_LOWVOLT_THRESHOLD)
  {
    // Added below line to generate battary low status and alarm only when no any other active alarm,
    // generated battary alarm then. the pop-up list uses LIFO sequence. so don't want battary alarm
    // to overwrite other alarm on screen side.
    if (((Alarm_Flags0.all & ALARM_MASK_FLG0) == 0) && ((Alarm_Flags1.all & ALARM_MASK_FLG1) == 0) && ((Alarm_Flags2.all & ALARM_MASK_FLG2) == 0))
    {
       if (LowBatAlmFlg == 0)
       {
          TripFlagsReset();                                   // to allow alarm to be logged
          LowBatAlmFlg = 1;
    //      InsertNewEvent(MINOR_ALARM, (((uint32_t)CAUSE_BATT) << 16) + 0x0800 + ((OpenFlg == 1)? 1 : 2));
       }

      // According to discussion agreement for PD's, when battery <2.5V, LCD will display empty      ***BP - double-check this
      // battery icon, latch battery voltage metering value as 0V to keep alignment.
      Vbattery = 0;
    }
  }
  else
  {
     LowBatAlmFlg = 0;
  }

  if (BatteryValid == FALSE)                    // if the 1st battery sampling, log power up event
  {
    if (LowBatAlmFlg == 1)
    {
  //    InsertNewEvent(POWER_UP, TYPEMOD_PWR_NOBAT);        // *** DAH  DO WE NEED TO LOG THIS? WE ALREADY LOG POWER UP EVENTS BASED ON WHETHER THE CLOCK IS GOOD
    }
    else
    {  
 //     InsertNewEvent(POWER_UP, TYPEMOD_PWR_BAT);
    }
  }

  BatteryValid = TRUE;                          // 1st battery voltage reading done, value valid now

}
//------------------------------------------------------------------------------
//  END OF FUNCTION         BatteryVolt_Alarm()
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Overvoltage Alarm function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any AFE line-to-line Vab_Rms or Vbc_Rms or Vca_Rms is above the OverVolt_A_Pu setpoint
//                      for a duration in excess of the OverVolt_A_Tt, set OvAlmFlg.
//
//                      Below OverVolt_A_Pu, clear the OvAlmTmr.
//
//  CAVEATS:            Vll_max is determined in Calc_Prot_AFE_Voltage(). One-cycle AFE voltages are used.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable
//                      Setpoints5.stp.OverVolt_A_EC_OFF: [0=alarm, 1=alarm with extended capture, 2=off]
//                      Setpoints5.stp.OverVolt_A_Pu: [102 to 150, step=0.1] %
//                      Setpoints5.stp.OverVolt_A_Tt: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_max, OvAlmTmr
//
//  OUTPUTS:            OvAlmFlg
//
//  ALTERS:             OvAlmTmr, OvAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void OverVoltage_Alarm(void)
{
  // If Over Voltage alarm is not enabled or is being held off, reset the timer and alarm flag, and return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.OverVolt_A_EC_OFF > 1) || (AlarmHoldOffTmr > 0) )
  {
    OvAlmTmr = 0;
    OvAlmFlg = 0;
    return;
  }

  // Over Voltage alarm function is enabled and not held off

  if (Vll_max >= Setpoints5.stp.OverVolt_A_Pu * Setpoints0.stp.SystemVoltage / 1000)
  {                                // Above voltage alarm threshold
     if (OvAlmTmr >= (Setpoints5.stp.OverVolt_A_Tt * Setpoints0.stp.Freq / 100))
     {                             // Alarm delay expired
        if (OvAlmFlg == FALSE)
        {
           OvAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           OvAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)                // If test, just insert event for summary log
           {
              InsertNewEvent(ALARM_OV_ENTRY_TEST);
           }
           else if (Setpoints5.stp.OverVolt_A_EC_OFF == 0)  // If overvoltage alarm...
           {
             if (!Alarm_WF_Capture.InProg)                          // Set alarm waveform capture req flag
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_OVER_VOLT].EIDofStartEvnt = InsertNewEvent(ALARM_OVERVOLTAGE_ENTRY);
           }
           else if (Setpoints5.stp.OverVolt_A_EC_OFF == 1)  // If extended capture on overvoltage...
           {
             // Set the flag to do an extended capture.  Note, this will cause an extended capture event to
             //   be inserted into the queue in ExtendedCapture()
             ExtCap_ReqFlag |= EC_OV;
             // Set the flag to initiate a GOOSE capture message if one is not already in progress.
             //   Goose_Capture_Code stores which process initiated the capture, so that the command to
             //   terminate the capture will only come from the same process.
             // Note, the GOOSE Rx subroutine in the sampling interrupt could also set the flag. It
             //   shouldn't matter if this is missed, because we only do one extended capture anyway, so we
             //   don't need to disable interrupts around this statement
             if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code > DS_LAST)
               && (Setpoints13.stp.GC_GlobalCapEnable & 0x10) )
             {
               PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 1;
               DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;           // *** DAH  WE WILL LIKELY NEED TO INCLUDE WHICH PARAMETER TO MONITOR FOR THE DISTURBANCE CAPTURE!
               Goose_Capture_Code = DS_OVER_VOLT;
             }
           }
        }
     }
     else                          // Under alarm delay.
     {
        OvAlmTmr++;                // Increment alarm timer
     }

     // If the OV trip function is not enabled, initiate a disturbance capture.  Note, this is started
     //   immediately, regardless of the delay timer, so that measurements are taken over the entire
     //   disturbance.  The capture is abandoned if the delay time is not reached
     if (Setpoints5.stp.OverVolt_T_OFF == 0)
     {
       Dist_Flag |= DS_OV;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
     }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
  }                                  //          THEN NEED TO CLEAR FLAGS IF FEATURE IS UNLOCKED
  else                             // Below voltage alarm threshold
  {
     if (Setpoints5.stp.OverVolt_T_OFF == 0)    // If the OV trip function is not enabled, complete the
     {                                          //   disturbance capture
       if (Dist_Flag & DS_OV)                       // If a disturbance capture was initiated...
       {
         // If alarm flag is not set, abort the the disturbance capture because no alarm or extended capture
         //   event occurred (the delay time wasn't met).  If the alarm flag is set, we will end the capture
         //   and log the results
         if (!OvAlmFlg)                         // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
         {
           Dist_Flag_Cancel |= DS_OV;
         }
         // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
         Dist_Flag &= (DS_OV ^ 0xFFFFFFFF);
       }
     }
     // Set the flag to send a GOOSE message to terminate the capture if this process is in control
     if (Goose_Capture_Code == DS_OVER_VOLT)
     {
       PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
       DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
       Goose_Capture_Code = DS_LAST + 1;
     }
     OvAlmTmr = 0;                 // reset timer
     OvAlmFlg = 0;             // reset flag
  }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        UnderVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Undervoltage Alarm function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any AFE line-to-line Vab_Rms or Vbc_Rms or Vca_Rms is below the UnderVolt_A_Pu setpoint
//                      for a duration in excess of the UnderVolt_Tt, set UnAlmFlg.
//
//                      Above UnderVolt_A_Pu, clear the UvAlmTmr.
//
//  CAVEATS:            Vll_max is determined in Calc_Prot_AFE_Voltage(). One-cycle AFE voltages are used.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable
//                      Setpoints5.stp.UnderVolt_A_EC_OFF: [0=alarm, 1=alarm with extended capture, 2=off]
//                      Setpoints5.stp.UnderVolt_A_Pu: [102 to 150, step=0.1] %
//                      Setpoints5.stp.UnderVolt_A_Tt: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_max, UvAlmTmr
//
//  OUTPUTS:            UvAlmFlg
//
//  ALTERS:             UvAlmTmr, UvAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void UnderVoltage_Alarm(void)
{
  // If Under Voltage alarm is not enabled or is being held off or if voltage is less than 10% of System Voltage,
  // reset the timer and alarm flag, and return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.UnderVolt_A_EC_OFF > 1) || (AlarmHoldOffTmr > 0)
       || (Vll_max < Setpoints0.stp.SystemVoltage / 10))
  {
    UvAlmTmr = 0;
    UvAlmFlg = 0;
    return;
  }

  // Under Voltage alarm function is enabled and not held off

  if (Vll_min < Setpoints5.stp.UnderVolt_A_Pu * Setpoints0.stp.SystemVoltage / 1000)
  {                                // Below voltage alarm threshold
     if (UvAlmTmr >= (Setpoints5.stp.UnderVolt_A_Tt * Setpoints0.stp.Freq / 100))
     {                             // Alarm delay expired
        if (UvAlmFlg == FALSE)
        {
           UvAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           UvAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)                // If test, just insert event for summary log
           {
              InsertNewEvent(ALARM_UV_ENTRY_TEST);
           }
           else if (Setpoints5.stp.UnderVolt_A_EC_OFF == 0) // If undervoltage alarm...
           {
             if (!Alarm_WF_Capture.InProg)                          // Set alarm waveform capture req flag
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_UNDER_VOLT].EIDofStartEvnt = InsertNewEvent(ALARM_UNDERVOLTAGE_ENTRY);
           }
           else if (Setpoints5.stp.UnderVolt_A_EC_OFF == 1) // If extended capture on undervoltage...
           {
             // Set the flag to do an extended capture.  Note, this will cause an extended capture event to
             //   be inserted into the queue in ExtendedCapture()
             ExtCap_ReqFlag |= EC_UV;
             // Set the flag to initiate a GOOSE capture message if one is not already in progress.
             //   Goose_Capture_Code stores which process initiated the capture, so that the command to
             //   terminate the capture will only come from the same process.
             // Note, the GOOSE Rx subroutine in the sampling interrupt could also set the flag. It
             //   shouldn't matter if this is missed, because we only do one extended capture anyway, so we
             //   don't need to disable interrupts around this statement
             if ( (Setpoints0.stp.IEC61850_Config == 1) && (Goose_Capture_Code > DS_LAST)
               && (Setpoints13.stp.GC_GlobalCapEnable & 0x20) )
             {
               PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 1;
               DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;           // *** DAH  WE WILL LIKELY NEED TO INCLUDE WHICH PARAMETER TO MONITOR FOR THE DISTURBANCE CAPTURE!
               Goose_Capture_Code = DS_UNDER_VOLT;
             }
           }
        }
     }
     else                          // Under alarm delay.
     {
        UvAlmTmr++;                // Increment alarm timer
     }

     // If the UV trip function is not enabled, initiate a disturbance capture.  Note, this is started
     //   immediately, regardless of the delay timer, so that measurements are taken over the entire
     //   disturbance.  The capture is abandoned if the delay time is not reached
     if (Setpoints5.stp.UnderVolt_T_OFF == 0)
     {
       Dist_Flag |= DS_UV;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
     }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
  }                                  //          THEN NEED TO CLEAR FLAGS IF FEATURE IS UNLOCKED
  else                             // Below voltage alarm threshold
  {
     if (Setpoints5.stp.UnderVolt_T_OFF == 0)   // If the UV trip function is not enabled, complete the
     {                                          //   disturbance capture
       if (Dist_Flag & DS_UV)                       // If a disturbance capture was initiated...
       {
         // If alarm flag is not set, abort the the disturbance capture because no alarm or extended capture
         //   event occurred (the delay time wasn't met).  If the alarm flag is set, we will end the capture
         //   and log the results
         if (!UvAlmFlg)                         // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
         {
           Dist_Flag_Cancel |= DS_UV;
         }
         // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
         Dist_Flag &= (DS_UV ^ 0xFFFFFFFF);
       }
     }
     // Set the flag to send a GOOSE message to terminate the capture if this process is in control
     if (Goose_Capture_Code == DS_UNDER_VOLT)
     {
       PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = 0;
       DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
       Goose_Capture_Code = DS_LAST + 1;
     }
     UvAlmTmr = 0;                 // reset timer
     UvAlmFlg = 0;                 // reset flag
  }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             UnderVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        CurUnbalance_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Current Unbalance Alarm
//
//  MECHANICS:          Current unbalance alarm protection is active only when enabled by the
//                      user by setting Setpoints5.stp.CurUnb_A_OFF to 1 (enabled).
//
//                      Unbalance protection will not start until there is 50% of Ir current.
//
//                      Current Unbalance protection is called at the one cycle anniversary.
//                      This routine uses the CurUnbalTot value provided by the Calc_Prot_Current()
//                      function called before this function.
//
//                      A current unbalance alarm occurs when the CurUnbalTot is greater than the percentage
//                      specified by the Current Unbalance pickup setpoint, Setpoints5.stp.CurUnb_A_Pu.
//                      CurUnbalTot = ((imax - imin)/imax) * 100.0.
//                      This condition must exist for 1 to 60 seconds specified by Setpoints5.stp.CurUnb_Tt.
//
//                      This routine is called every one cycle anniversary.  A 60 sec timeout
//                      will take 60 x 60 = 3600 passes at 60hz, or 3000 passes at 50hz.
//
//
//  CAVEATS:            Called only if Setpoints5.stp.CurUnb_T_A_OFF = 1
//
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]                     
//                      Setpoints5.stp.CurUnb_A_OFF [0=disable, 1=enable]
//                      Setpoints5.stp.CurUnb_A_Pu: [2 to 90, step=1] %
//                      Setpoints5.stp.CurUnb_A_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurUnbalTot, CurOneCycMax
//
//  OUTPUTS:            CurrUnbAlmFlg
//
//  ALTERS:             CurUnbalAlmTmr, CurrUnbAlmFlg
//
//  CALLS:
//
//------------------------------------------------------------------------------------------------------------
//
void CurUnbalance_Alarm(void)
{
  // If Current Unbalance alarm is not enabled or is being held off, reset the timer and alarm flag, and
  //   return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.CurUnb_A_OFF != 1) || (AlarmHoldOffTmr > 0) )
  {
    CurUnbalAlmTmr = 0;
    CurrUnbAlmFlg = FALSE;
    return;
  }

  // Current Unbalance alarm function is enabled and not held off

  // If above unbalance pickup level and min current requirement is met...
  if ( (CurUnbalTot > Setpoints5.stp.CurUnb_A_Pu) && (CurOneCycMax >= 0.5 * Ir) )
  {
    CurUnbalAlmTmr++;                       // Increment delay timer

    // If delay time is exceeded, event has occurred
    if ( CurUnbalAlmTmr >= (Setpoints5.stp.CurUnb_A_Tt * Setpoints0.stp.Freq / 100) )
    {
      if (CurrUnbAlmFlg == FALSE)           // If first time in...
      {
        CurUnbalAlmTmr = 0;
        TripFlagsReset();                          // to allow alarm to be logged
        CurrUnbAlmFlg = 1;
        if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
        {
          InsertNewEvent(ALARM_CU_ENTRY_TEST);
        }
        else                                       // Insert alarm event with wf capture if not in test mode
        {
          if (!Alarm_WF_Capture.InProg)
          {
            Alarm_WF_Capture.Req = TRUE;
          }
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_CUR_UNBAL].EIDofStartEvnt = InsertNewEvent(ALARM_CURRENT_UNBALANCE_ENTRY);
        }
      }
    }
    // If the CU trip function is not enabled, initiate a disturbance capture.  Note, this is started
    //   immediately, regardless of the delay timer, so that measurements are taken over the entire
    //   disturbance.  The capture is abandoned if the delay time is not reached
    if (Setpoints5.stp.CurUnb_T_OFF == 0)
    {
      Dist_Flag |= DS_CU;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
    }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
  }
  else                                      // Below current unbalance alarm threshold
  {
    if (Setpoints5.stp.CurUnb_T_OFF == 0)           // If the CU trip function is not enabled, complete the
    {                                              //   disturbance capture
      if (Dist_Flag & DS_CU)                           // If a disturbance capture was initiated...
      {
        // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
        //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
        if (!CurrUnbAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
        {
          Dist_Flag_Cancel |= DS_CU;
        }
        // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
        Dist_Flag &= (DS_CU ^ 0xFFFFFFFF);
      }
    }
    CurUnbalAlmTmr = 0;
    CurrUnbAlmFlg = FALSE;
  }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             CurUnbalance_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PhaseLoss_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Current Based Phase Loss Alarm.
//
//
//  MECHANICS:          Phase Loss protection is used for the complete loss of one or two phases. If
//                      the difference between the maximum and minimum of any of the three phase
//                      currents is greater than 75% for the time delay, then the protection action
//                      will be taken.
//
//                      Phase Loss protection will not be active unless at least one phase current is
//                      greater than 50% of the LDPU setting.
//
//                      Phase Loss protection is called at the one cycle anniversary.
//                      This routine uses the CurUnbalTot value provided by the Calc_Prot_Current()
//                      function called before this function.
//
//                      A phase loss alarm occurs when the CurUnbalTot is greater than 75%.
//                      This condition must exist for 1 to 240 seconds specified by Setpoints5.stp.PhaseLoss_Tt.
//
//                      This routine is called every one cycle anniversary.  A 240 sec timeout will take
//                      240 x 60 = 14400 passes at 60hz, or 12000 passes at 50hz.
//
//
//  CAVEATS:            Phase loss alarm protection is active only when enabled by the user by setting
//                      PhaseLoss_T_A_OFF to 1 (alarm).
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]                      
//                      Setpoints5.stp.PhaseLoss_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.PhaseLoss_Tt: [1 to 240, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      CurUnbalTot, CurOneCycMax
//
//  OUTPUTS:            PhaseLossAlmFlg
//
//  ALTERS:             PhaseLossAlmTmr, PhaseLossAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------

void PhaseLoss_Alarm(void)
{
  // If Phase Loss alarm is not enabled or is being held off, reset the timer and alarm flag, and return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.PhaseLoss_T_A_OFF != 1) || (AlarmHoldOffTmr > 0) )
  {
    PhaseLossAlmTmr = 0;
    PhaseLossAlmFlg = FALSE;
    return;
  }

  // Phase Loss alarm function is enabled and not held off

  // If above the threshold, check time and insert event
  if ( (CurUnbalTot > 75) && (CurOneCycMax >= 0.5 * Ir) )
  {
     PhaseLossAlmTmr++;

     if ( PhaseLossAlmTmr >= (Setpoints5.stp.PhaseLoss_Tt * Setpoints0.stp.Freq) )
     {
       PhaseLossAlmTmr = 0;
       TripFlagsReset();                    // to allow alarm to be logged
       PhaseLossAlmFlg = 1;
       if (TU_State_TestUSBMode == TRUE)    // Insert summary event only if in test mode
       {
         InsertNewEvent(ALARM_PH_LOSS_ENTRY_TEST);
       }
       else                                 // Insert alarm event with wf capture if not in test mode
       {
         if (!Alarm_WF_Capture.InProg)
         {
           Alarm_WF_Capture.Req = TRUE;
         }
         // Save the EID of the event that started the disturbance capture, and insert an entry event
         sListOfDist[DS_PHASE_LOSS].EIDofStartEvnt = InsertNewEvent(ALARM_PH_LOSS_ENTRY);
       }
       // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
       //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
       //   delay time is not reached
       Dist_Flag |= DS_PL;        // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED

      }
   }
   else                                 // below PL threshold
   {
     if (Dist_Flag & DS_PL)                 // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!PhaseLossAlmFlg)        // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_PL;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_PL ^ 0xFFFFFFFF);
     }
     PhaseLossAlmTmr = 0;                       // reset timer
     PhaseLossAlmFlg = FALSE;                   // reset flag
   }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PhaseLoss_Alarm
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        VoltUnbalance_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Voltage Unbalance Alarm function
//
//  MECHANICS:          Voltage Unbalance alarm protection is active only when enabled by the
//                      user by setting Setpoints5.stp.VoltUnb_A_OFF to 1 (enabled).
//
//                      Unbalance protection will not start until there is a minimum l-l voltage (Vll_min > 84).
//
//                      Voltage Unbalance protection is called at the one cycle anniversary.
//                      This routine uses the VolUnbalTot value provided by the Calc_Prot_AFE_Voltage()
//                      function called before this function.
//
//                      A current unbalance alarm occurs when the VolUnbalTot is greater than the percentage
//                      specified by the Voltage Unbalance pickup setpoint, Setpoints5.stp.VoltUnb_A_Pu.
//                      VolUnbalTot = ((vmax - vmin)/vmax) * 100.0.
//                      This condition must exist for 1 to 300 seconds specified by Setpoints5.stp.VoltUnb_A_Tt.
//
//                      This routine is called every one cycle anniversary.  A 60 sec timeout
//                      will take 60 x 60 = 3600 passes at 60hz, or 3000 passes at 50hz.
//
//                      If the duration exceeds the delay time, set VoltUnbalAlmFlg.
//
//                      Below VoltUnb_A_Pu, clear the VunbalTripTmr.
//
//
//  CAVEATS:            Setpoints5.stp.VoltUnb_A_OFF must be set to 1 (enabled)
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable
//                      Setpoints5.stp.VoltUnb_A_OFF
//                      Setpoints5.stp.VoltUnb_A_Pu: [2 to 90] %
//                      Setpoints5.stp.VoltUnb_A_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_max, Vll_min, VoltUnbalAlmTmr
//
//  OUTPUTS:            Returns: Nothing.
//                      Memory:  VoltUnbalAlmFlg
//
//  ALTERS:             VoltUnbalAlmTmr
//
//  CALLS:

//------------------------------------------------------------------------------------------------------------

void VoltUnbalance_Alarm(void)
{
  // If Voltage Unbalance alarm is not enabled or is being held off, reset the timer and alarm flag, and
  //   return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.VoltUnb_A_OFF != 1) || (AlarmHoldOffTmr > 0) )
  {
    VoltUnbalAlmTmr = 0;
    VoltUnbalAlmFlg = 0;
    return;
  }

  // Voltage Unbalance alarm function is enabled and not held off

  // If above unbalance pickup level and min voltage requirement is met...
  if ( (VolUnbalTot > Setpoints5.stp.VoltUnb_A_Pu)  && (Vll_min > 84) )
  {
    VoltUnbalAlmTmr++;                         // Increment delay timer

    // If delay time is exceeded, event has occurred
    if ( VoltUnbalAlmTmr >= (Setpoints5.stp.VoltUnb_A_Tt * Setpoints0.stp.Freq / 100) )
    {
      if (VoltUnbalAlmFlg == FALSE)            // If first time in...
      {
        VoltUnbalAlmTmr = 0;
        TripFlagsReset();                                   // to allow alarm to be logged
        VoltUnbalAlmFlg = 1;
        if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
        {
          InsertNewEvent(ALARM_VU_ENTRY_TEST);
        }
        else                                       // Insert alarm event with wf capture if not in test mode
        {
          if (!Alarm_WF_Capture.InProg)
          {
            Alarm_WF_Capture.Req = TRUE;
          }
          // Save the EID of the event that started the disturbance capture, and insert an entry event
          sListOfDist[DS_VOLT_UNBAL].EIDofStartEvnt = InsertNewEvent(ALARM_VOLTAGE_UNBALANCE_ENTRY);
        }
      }
    }
    // If the VU trip function is not enabled, initiate a disturbance capture.  Note, this is started
    //   immediately, regardless of the delay timer, so that measurements are taken over the entire
    //   disturbance.  The capture is abandoned if the delay time is not reached
    if (Setpoints5.stp.VoltUnb_T_OFF == 0)
    {
      Dist_Flag |= DS_VU;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
    }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
  }                                 //          THEN NEED TO CLEAR FLAGS IF FEATURE IS UNLOCKED
  else                                      // Below voltage unbalance alarm threshold
  {
    if (Setpoints5.stp.VoltUnb_T_OFF == 0)         // If the VU trip function is not enabled, complete the
    {                                              //   disturbance capture
      if (Dist_Flag & DS_VU)                           // If a disturbance capture was initiated...
      {
        // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
        //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
        if (!VoltUnbalAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
        {
          Dist_Flag_Cancel |= DS_VU;
        }
        // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
        Dist_Flag &= (DS_VU ^ 0xFFFFFFFF);
      }
    }
    VoltUnbalAlmTmr = 0;
    VoltUnbalAlmFlg = 0;
  }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             VoltUnbalance_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverFreq_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Overfrequency Alarm function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary if frequency protection is
//                      enabled: Setpoints4.stp.Freq_Prot_Enable == 1.  And also if the frequency protection
//                      alarm setpoint is set to enable: Setpoints4.stp.OF_Alarm_Prot_Action = 1
//
//                      If FreqLine.FreqVal is above the OF_Prot_Pickup for a duration in excess of the
//                      OF_Prot_Time, set OfAlmFlg.
//
//                      To prevent frequency trips and alarms in the absence of line voltage, skip
//                      frequency protection if the measured frequency is marked invalid:
//                      (FreqLine.FreqVal = NAN).  The Calc_Freq() routine will set the measured
//                      frequency to invalid if inadequate voltage exists to properly determine the
//                      zero-crossing point of Van.
//
//                      Below the Setpoints4.stp.OF_Alarm_Prot_Pickup, clear the OfTripTmr.
//
//  CAVEATS:            This function is called from Main() if the Freq Protection setpoint is set to Enable
//
//  INPUTS:             Setpoints4.stp.Freq_Prot_Enable
//                      AlarmHoldOffTmr
//                      Setpoints4.stp.OF_Alarm_Prot_Action
//                      FreqLine.FreqVal = The measured line frequency in Hz
//                      Setpoints4.stp.OF_Alarm_Prot_Action: [0=disabled, 1=enabled]
//                      Setpoints4.stp.OF_Alarm_Prot_Pickup: [100.1 - 110.0, step=0.1] %   <- Setpoint = 1001 to 1100
//                      Setpoints4.stp.OF_Alarm_Prot_Time: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      OfAlmTmr
//                      Vll_max
//
//  OUTPUTS:            OfAlmFlg
//
//  ALTERS:             OfAlmTmr, OfAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//-------------------------------------------------------------------------

void OverFreq_Alarm(void)
{

   // If Over Frequency alarm is not enabled or is being held off, reset the timer and alarm flag, and return
   if ( (Setpoints4.stp.Freq_Prot_Enable != 1) || (Setpoints4.stp.OF_Alarm_Prot_Action != 1)
     || (AlarmHoldOffTmr > 0) || (Vll_max < Setpoints0.stp.SystemVoltage / 10) )
   {
     OfAlmTmr = 0;
     OfAlmFlg = 0;
     return;
   }

   // Over Frequency alarm function is enabled and not held off

   // Is Freq protection set for Alarm and valid Freq value?
   if ( (Setpoints4.stp.OF_Alarm_Prot_Action == 1) && (FreqLine.FreqVal != NAN) )
   {  // Frequency over setpoint?
      // Multiply FreqVal by 1000 instead of dividing integer Pickup value by 1000
      if (1000*FreqLine.FreqVal >= Setpoints4.stp.OF_Alarm_Prot_Pickup * Setpoints0.stp.Freq)
      {  // Yes- check alarm delay timer
         if (OfAlmTmr >= (Setpoints4.stp.OF_Alarm_Prot_Time * Setpoints0.stp.Freq / 100) )
         {  // Alarm timer timed-out - alarm
            if (OfAlmFlg == FALSE)              // If first time in...
            {
              OfAlmTmr = 0;
              TripFlagsReset();                                   // to allow alarm to be logged
              OfAlmFlg = 1;
              if (TU_State_TestUSBMode == TRUE)    // Insert summary event only if in test mode
              {
                InsertNewEvent(ALARM_OF_ENTRY_TEST);
              }
              else                                 // Insert alarm event with wf capture if not in test mode
              {
                if (!Alarm_WF_Capture.InProg)
                {
                  Alarm_WF_Capture.Req = TRUE;
                }
                // Save the EID of the event that started the disturbance capture, and insert an entry event
                sListOfDist[DS_OVER_FREQ].EIDofStartEvnt = InsertNewEvent(ALARM_OVERFREQUENCY_ENTRY);
              }
            }
         }
         else                          // Alarm timer not timed-out
         {
            OfAlmTmr++;
         }
         // If the OF trip function is not enabled, initiate a disturbance capture.  Note, this is started
         //   immediately, regardless of the delay timer, so that measurements are taken over the entire
         //   disturbance.  The capture is abandoned if the delay time is not reached
         if (Setpoints4.stp.OF_Trip_Prot_Action == 0)
         {
           Dist_Flag |= DS_OF;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
         }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
      }
      else                             // Frequency not over limit...
      {
         if (Setpoints4.stp.OF_Trip_Prot_Action == 0)   // If the OF trip function is not enabled, complete
         {                                              //   the disturbance capture
           if (Dist_Flag & DS_OF)                       // If a disturbance capture was initiated...
           {
             // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred
             //   (the delay time wasn't met).  If the alarm flag is set, we will end the capture and log
             //   the results
             if (!OfAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
             {
               Dist_Flag_Cancel |= DS_OF;
             }
             // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't
             //   aborted)
             Dist_Flag &= (DS_OF ^ 0xFFFFFFFF);
           }
         }
         OfAlmTmr = 0;                 // reset timer
         OfAlmFlg = 0;             // reset flag
      }
   }
   else                                // Alarm protection turned off or bad Freq value
   {
      OfAlmTmr = 0;                    // reset timer
      OfAlmFlg = 0;                // reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverFreq_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        UnderFreq_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Underfrequency Alarm function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary if frequency protection is
//                      enabled: Setpoints4.stp.Freq_Prot_Enable == 1.  And also if the frequency protection
//                      alarm setpoint is set to enable: Setpoints4.stp.UF_Alarm_Prot_Action = 1
//
//                      If FreqLine.FreqVal is below the UF_Alarm_Prot_Pickup for a duration in excess of the
//                      UF_Alarm_Prot_Time, set UfAlmFlg.
//
//                      To prevent frequency trips and alarms in the absence of line voltage, skip
//                      frequency protection if the measured frequency is marked invalid:
//                      (FreqLine.FreqVal = NAN).  The Calc_Freq() routine will set the measured
//                      frequency to invalid if inadequate voltage exists to properly determine the
//                      zero-crossing point of Van.
//
//                      Above the Setpoints4.stp.UF_Alarm_Prot_Pickup, clear the UfTripTmr.
//
//  CAVEATS:            This function is called from Main() if the Freq Protection setpoint is set to Enable
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints4.stp.Freq_Prot_Enable
//                      Setpoints4.stp.UF_Alarm_Prot_Action
//                      FreqLine.FreqVal = The measured line frequency in Hz
//                      Setpoints4.stp.UF_Alarm_Prot_Action: [0=disabled, 1=enabled]       <--used in main()
//                      Setpoints4.stp.UF_Alarm_Prot_Pickup: [90.0 - 99.9, step=0.1] %    <- Setpoint = 900 to 999
//                      Setpoints4.stp.UF_Alarm_Prot_Time: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Setpoints0.stp.SystemVoltage: [90 to 690, step=1] V
//                      UfAlmTmr
//
//  OUTPUTS:            UfAlmFlg
//
//  ALTERS:             UfAlmTmr, UfAlmFlg
//
//  CALLS:              InsertNewEvent()
//
//-------------------------------------------------------------------------

void UnderFreq_Alarm(void)
{

   // If Under Frequency alarm is not enabled or is being held off, reset the timer and alarm flag, and return
   if ( (Setpoints4.stp.Freq_Prot_Enable != 1) || (Setpoints4.stp.UF_Alarm_Prot_Action != 1)
     || (AlarmHoldOffTmr > 0) || (Vll_max < Setpoints0.stp.SystemVoltage / 10) )
   {
     UfAlmTmr = 0;
     UfAlmFlg = 0;
     return;
   }

   // Under Frequency alarm function is enabled and not held off

   // Is Freq protection set for Alarm and valid Freq value?
   if ( (Setpoints4.stp.UF_Alarm_Prot_Action == 1) && (FreqLine.FreqVal != NAN) )
   {  // Frequency under setpoint?
      // Multiply FreqVal by 1000 instead of dividing integer Pickup value by 1000
      if (1000*FreqLine.FreqVal <= Setpoints4.stp.UF_Alarm_Prot_Pickup * Setpoints0.stp.Freq)
      {  // Yes- check alarm delay timer
         if (UfAlmTmr >= (Setpoints4.stp.UF_Alarm_Prot_Time * Setpoints0.stp.Freq / 100) )
         {  // Alarm timer timed-out - alarm
            if (UfAlmFlg == FALSE)              // If first time in...
            {
              UfAlmTmr = 0;
              TripFlagsReset();                                   // to allow alarm to be logged
              UfAlmFlg = 1;
              if (TU_State_TestUSBMode == TRUE)    // Insert summary event only if in test mode
              {
                InsertNewEvent(ALARM_UF_ENTRY_TEST);
              }
              else                                 // Insert alarm event with wf capture if not in test mode
              {
                if (!Alarm_WF_Capture.InProg)
                {
                  Alarm_WF_Capture.Req = TRUE;
                }
                // Save the EID of the event that started the disturbance capture, and insert an entry event
                sListOfDist[DS_UNDER_FREQ].EIDofStartEvnt = InsertNewEvent(ALARM_UNDERFREQUENCY_ENTRY);
              }
            }
         }
         else                          // Alarm timer not timed-out
         {
            UfAlmTmr++;
         }
         // If the UF trip function is not enabled, initiate a disturbance capture.  Note, this is started
         //   immediately, regardless of the delay timer, so that measurements are taken over the entire
         //   disturbance.  The capture is abandoned if the delay time is not reached
         if (Setpoints4.stp.UF_Trip_Prot_Action == 0)
         {
           Dist_Flag |= DS_UF;           // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
         }                               //          COULD ALSO PUT CHECK IN THE MAIN LOOP AND NOT CALL DisturbanceCapture() BUT
      }
      else                             // Frequency not under limit...
      {
         if (Setpoints4.stp.UF_Trip_Prot_Action == 0)   // If the UF trip function is not enabled, complete
         {                                              //   the disturbance capture
           if (Dist_Flag & DS_UF)                       // If a disturbance capture was initiated...
           {
             // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred
             //   (the delay time wasn't met).  If the alarm flag is set, we will end the capture and log
             //   the results
             if (!UfAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
             {
               Dist_Flag_Cancel |= DS_UF;
             }
             // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't
             //   aborted)
             Dist_Flag &= (DS_UF ^ 0xFFFFFFFF);
           }
         }
         UfAlmTmr = 0;                 // reset timer
         UfAlmFlg = 0;             // reset flag
      }
   }
   else                                // Alarm protection turned off or bad Freq value
   {
      UfAlmTmr = 0;                    // reset timer
      UfAlmFlg = 0;                // reset flag
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             UnderFreq_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverRealPower_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverRealPower_Alarm Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the real power values are above the RealPwr_Pu setpoint for a duration
//                      in excess of the RealPwr_Tt, set RealPwrAlmFlg.
//
//                      Below RealPwr_Pu, clear the RealPwrAlmTmr.
//
//                      One-cycle real power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Real Power T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.RealPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.RealPwr_Pu: [1 to 65550, step=1] kW
//                      Setpoints6.stp.RealPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.Pa, PwrOneCyc.Pb, PwrOneCyc.Pc
//
//  OUTPUTS:            RealPwrAlmFlg, MaxPwrOneCycW
//
//  ALTERS:             RealPwrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//---------------------------------------------------------------------------------------------------------
//
void OverRealPower_Alarm(void)
{
   float tempPa, tempPb, tempPc;

   // If Over Real Power alarm is not enabled or is being held off, reset the timer and alarm flag, and
   //   return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.RealPwr_T_A_OFF != 1)
     || (AlarmHoldOffTmr > 0) )
   {
     RealPwrAlmTmr = 0;
     RealPwrAlmFlg = FALSE;
     return;
   }

   // Over Real Power alarm function is enabled and not held off

   // For regular feed, positive power is "forward", so the sign is ok as is.
   // For reverse feed, negative power is "forward", so flip the sign
   tempPa = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pa : PwrOneCyc.Pa;
   tempPb = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pb : PwrOneCyc.Pb;
   tempPc = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.Pc : PwrOneCyc.Pc;

   // Compute the absolute value of the maximum forward power and save it in tempPa
   tempPa = ( (tempPa > tempPb) ? tempPa : tempPb);
   tempPa = ( (tempPa > tempPc) ? tempPa : tempPc);

   // Save the max forward power for disturbance capture
   MaxPwrOneCycW = ( (Setpoints0.stp.RevFeed == 1) ? (-tempPa) : tempPa);

   // Now do alarm using the absolute value of the max forward power
   if (tempPa >= Setpoints6.stp.RealPwr_Pu * 1000)
   {                                // Above real power pickup
      if (RealPwrAlmTmr >= (Setpoints6.stp.RealPwr_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired - alarm
         if (RealPwrAlmFlg == FALSE)
         {
           RealPwrAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           RealPwrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_OVERKW_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_OVER_W].EIDofStartEvnt = InsertNewEvent(ALARM_OVERKW_ENTRY);
           }
         }
      }
      else                          // Under alarm delay.
      {
        RealPwrAlmTmr++;            // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_OW;          // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                            // Below real power pickup
   {
     if (Dist_Flag & DS_OW)                           // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!RealPwrAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_OW;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_OW ^ 0xFFFFFFFF);
     }
     RealPwrAlmTmr = 0;              // reset timer
     RealPwrAlmFlg = FALSE;          // reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverRealPower_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverReactivePower_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverReactivePower_Alarm Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the reactive power values are above the ReacPwr_Pu setpoint for a duration
//                      in excess of the ReacPwr_Tt, set ReacPwrAlmFlg.
//
//                      Below ReacPwr_Pu, clear the ReacPwrAlmTmr.
//
//                      One-cycle reactive power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reactive Power T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.ReacPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.ReacPwr_Pu: [1 to 65550, step=1] kVar
//                      Setpoints6.stp.ReacPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.Pa, PwrOneCyc.Pb, PwrOneCyc.Pc
//
//  OUTPUTS:            ReacPwrAlmFlg, MaxPwrOneCycVar
//
//  ALTERS:             ReacPwrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void OverReactivePower_Alarm(void)
{
   float tempRPa, tempRPb, tempRPc;

   // If Over Reactive Power alarm is not enabled or is being held off, reset the timer and alarm flag, and
   //   return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.ReacPwr_T_A_OFF != 1)
     || (AlarmHoldOffTmr > 0) )
   {
     ReacPwrAlmTmr = 0;
     ReacPwrAlmFlg = FALSE;
     return;
   }

   // Over Reactive Power alarm function is enabled and not held off

   // For regular feed, positive power is "forward", so the sign is ok as is.
   // For reverse feed, negative power is "forward", so flip the sign
   tempRPa = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPa : PwrOneCyc.RPa;
   tempRPb = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPb : PwrOneCyc.RPb;
   tempRPc = (Setpoints0.stp.RevFeed == 1) ? -PwrOneCyc.RPc : PwrOneCyc.RPc;

   // Compute the absolute value of the maximum forward power and save it in tempRPa
   tempRPa = ( (tempRPa > tempRPb) ? tempRPa : tempRPb);
   tempRPa = ( (tempRPa > tempRPc) ? tempRPa : tempRPc);

   // Save the max forward power for disturbance capture
   MaxPwrOneCycVar = ( (Setpoints0.stp.RevFeed == 1) ? (-tempRPa) : tempRPa);

   // Now do alarm using the absolute value of the max forward power
   if (tempRPa >= Setpoints6.stp.ReacPwr_Pu * 1000)
   {                                // Above reactive power pickup
      if (ReacPwrAlmTmr >= (Setpoints6.stp.ReacPwr_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired - alarm
         if (ReacPwrAlmFlg == FALSE)
         {
           ReacPwrAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           ReacPwrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_OVERKVAR_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_OVER_VAR].EIDofStartEvnt = InsertNewEvent(ALARM_OVERKVAR_ENTRY);
           }
         }
      }
      else                          // Under alarm delay.
      {
        ReacPwrAlmTmr++;            // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_OVAR;         // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                             // Below reactive power pickup
   {
     if (Dist_Flag & DS_OVAR)               // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!ReacPwrAlmFlg)           // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_OVAR;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_OVAR ^ 0xFFFFFFFF);
     }
     ReacPwrAlmTmr = 0;             // reset timer
     ReacPwrAlmFlg = FALSE;         // reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverReactivePower_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OverApparentPower_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           OverApparentPower_Alarm Function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the apparent power values are above the AppPwr_Pu setpoint for a duration
//                      in excess of the AppPwr_Tt, set AppPwrAlmFlg.
//
//                      Below AppPwr_Pu, clear the AppPwrAlmTmr.
//
//                      One-cycle apparent power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Apparent Power T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.AppPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.AppPwr_Pu: [1 to 65550, step=1] kA
//                      Setpoints6.stp.AppPwr_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCycApp.AppPa, PPwrOneCycApp.AppPb, PwrOneCycApp.AppPc
//
//  OUTPUTS:            AppPwrAlmFlg, MaxPwrOneCycVA
//
//  ALTERS:             AppPwrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void OverApparentPower_Alarm(void)
{

   // If Over Apparent Power alarm is not enabled or is being held off, reset the timer and alarm flag, and
   //   return
   if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.AppPwr_T_A_OFF != 1)
     || (AlarmHoldOffTmr > 0) )
   {
     AppPwrAlmTmr = 0;
     AppPwrAlmFlg = FALSE;
     return;
   }

   // Over Apparent Power alarm function is enabled and not held off

   // Compute one-cycle max power (of the three phases) for disturbance capture info
   MaxPwrOneCycVA = ((PwrOneCycApp.AppPa > PwrOneCycApp.AppPb) ? PwrOneCycApp.AppPa : PwrOneCycApp.AppPb);
   MaxPwrOneCycVA = ((MaxPwrOneCycVA > PwrOneCycApp.AppPc) ? MaxPwrOneCycVA : PwrOneCycApp.AppPc);

   if (MaxPwrOneCycVA >= Setpoints6.stp.AppPwr_Pu * 1000)
//   if ( (PwrOneCycApp.AppPa >= Setpoints6.stp.AppPwr_Pu * 1000) || (PwrOneCycApp.AppPb >= Setpoints6.stp.AppPwr_Pu * 1000) || (PwrOneCycApp.AppPc >= Setpoints6.stp.AppPwr_Pu * 1000) )
   {                                // Above apparent power pickup
      if (AppPwrAlmTmr >= (Setpoints6.stp.AppPwr_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired - alarm
         if (AppPwrAlmFlg == FALSE)
         {
           AppPwrAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           AppPwrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_OVERKVA_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_OVER_VA].EIDofStartEvnt = InsertNewEvent(ALARM_OVERKVA_ENTRY);
           }
         }
      }
      else                          // Under alarm delay.
      {
        AppPwrAlmTmr++;             // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_OVA;          // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                             // Below apparent power pickup
   {
     if (Dist_Flag & DS_OVA)                // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!AppPwrAlmFlg)            // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_OVA;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_OVA ^ 0xFFFFFFFF);
     }
     AppPwrAlmTmr = 0;              // reset timer
     AppPwrAlmFlg = FALSE;          // reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OverApparentPower_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        RevActivePower_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reverse Active Power Alarm function
//                      (called Reverse Power Alarm in our other products)
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the real power values are above the RevPwr_Pu setpoint for a duration
//                      in excess of the RevPwr_Tt, set RevPwrAlmFlg.
//
//                      If Setpoints0.stp.RevFeed == 1, power direction is reversed.
//
//                      Below RevPwr_Pu, clear the RevPwrAlmTmr.
//
//                      One-cycle real power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reverse Power T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints5.stp.RevPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevPwr_Pu: [1 to 65500, step=1] kW
//                      Setpoints5.stp.RevPwr_Tt: [1 to 300, step=0.05] sec
//                      Setpoints0.stp.RevFeed: [0=forward, 1=reverse]
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.Pa, PwrOneCyc.Pb, PwrOneCyc.Pc
//
//  OUTPUTS:            RevPwrAlmFlg, MaxPwrOneCycRevW
//
//  ALTERS:             RevPwrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void RevActivePower_Alarm(void)
{
   float tempPa, tempPb, tempPc;

   // If Reverse Active Power alarm is not enabled or is being held off, reset the timer and alarm flag, and
   //   return
   if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints5.stp.RevPwr_T_A_OFF != 1)
     || (AlarmHoldOffTmr > 0) )
   {
     RevPwrAlmTmr = 0;
     RevPwrAlmFlg = FALSE;
     return;
   }

   // Reverse Active Power alarm function is enabled and not held off

   // For regular feed, negative power is "reverse" and positive power is "forward".  In this case, we need
   //   to flip the sign of the power.  This way, "reverse" power is positive, and "forward" power is
   //   negative.  The setpoint is positive, so "forward" power will never trigger.  We are only checking
   //   for high values of reverse power. 
   // For reverse feed, negative power is "forward", and positive power is "reverse", so the sign is ok as
   // is.
   tempPa = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pa : -PwrOneCyc.Pa;
   tempPb = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pb : -PwrOneCyc.Pb;
   tempPc = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.Pc : -PwrOneCyc.Pc;

   // Compute max absolute value of reverse power.  Store value in tempPa.  Store signed value in
   //   MaxPwrOneCycRevW (for disturbance capture)
   MaxPwrOneCycRevW = PwrOneCyc.Pa;
   if (tempPb > tempPa)
   {
     tempPa = tempPb;
     MaxPwrOneCycRevW = PwrOneCyc.Pb;
   }
   if (tempPc > tempPa)
   {
     tempPa = tempPc;
     MaxPwrOneCycRevW = PwrOneCyc.Pc;
   }

   // Now do alarm using the absolute value of the max reverse power
   if (tempPa >= Setpoints5.stp.RevPwr_Pu * 1000)
   {                                // Above reverse power pickup
      if (RevPwrAlmTmr >= (Setpoints5.stp.RevPwr_Tt * Setpoints0.stp.Freq / 100))
      {                             // Alarm delay expired
         if (RevPwrAlmFlg == FALSE)
         {
           RevPwrAlmTmr = 0;
           TripFlagsReset();                          // to allow alarm to be logged
           RevPwrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_REV_KW_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_REV_W].EIDofStartEvnt = InsertNewEvent(ALARM_REVERSE_KW_ENTRY);
           }
         }
      }
      else                          // Under alarm delay
      {
        RevPwrAlmTmr++;             // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_RKW;          // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                             // Below reverse power pickup
   {
     if (Dist_Flag & DS_RKW)                          // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!RevPwrAlmFlg)            // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_RKW;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_RKW ^ 0xFFFFFFFF);
     }
     RevPwrAlmTmr = 0;              // reset timer
     RevPwrAlmFlg = FALSE;          // reset flag
   }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             RevActivePower_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        RevReactivePower_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reverse Reactive Power Alarm function
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the reactive power values are above the RevReacPwr_Pu setpoint for a duration
//                      in excess of the RevReacPwr_Tt, set RevReacPwrAlmFlg.
//
//                      If Setpoints0.stp.RevFeed == 1, power direction is reversed.
//
//                      Below RevReacPwr_Pu, clear the RevReacPwrAlmTmr.
//
//                      One-cycle reactive power values are used.  They are calculated in Calc_Prot_Power().
//
//  CAVEATS:            This function is called from Main() if the Reverse Reactive Power T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints5.stp.RevReacPwr_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevReacPwr_Pu: [1 to 65550, step=1] kW
//                      Setpoints5.stp.RevReacPwr_Tt: [0.05 to 300, step=0.05] sec
//                      Setpoints0.stp.RevFeed: [0=forward, 1=reverse]
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PwrOneCyc.RPa, PwrOneCyc.RPb, PwrOneCyc.RWPc
//
//  OUTPUTS:            RevReacPwrAlmFlg, MaxPwrOneCycRevVar
//
//  ALTERS:             RevReacPwrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void RevReactivePower_Alarm(void)
{
   float tempRPa, tempRPb, tempRPc;

   // If Reverse Reactive Power alarm is not enabled or is being held off, reset the timer and alarm flag,
   //   and return
   if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints5.stp.RevReactPwr_T_A_OFF != 1)
     || (AlarmHoldOffTmr > 0) )
   {
     RevReacPwrAlmTmr = 0;
     RevReacPwrAlmFlg = FALSE;
     return;
   }

   // Reverse Reactive Power alarm function is enabled and not held off

   // For regular feed, negative power is "reverse" and positive power is "forward".  In this case, we need
   //   to flip the sign of the power.  This way, "reverse" power is positive, and "forward" power is
   //   negative.  The setpoint is positive, so "forward" power will never trigger.  We are only checking
   //   for high values of reverse power. 
   // For reverse feed, negative power is "forward", and positive power is "reverse", so the sign is ok as
   // is.
   tempRPa = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPa : -PwrOneCyc.RPa;
   tempRPb = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPb : -PwrOneCyc.RPb;
   tempRPc = (Setpoints0.stp.RevFeed == 1) ? PwrOneCyc.RPc : -PwrOneCyc.RPc;

   // Compute max absolute value of reverse power.  Store value in tempRPa.  Store signed value in
   //   MaxPwrOneCycRevVar (for disturbance capture)
   MaxPwrOneCycRevVar = PwrOneCyc.RPa;
   if (tempRPb > tempRPa)
   {
     tempRPa = tempRPb;
     MaxPwrOneCycRevVar = PwrOneCyc.RPb;
   }
   if (tempRPc > tempRPa)
   {
     tempRPa = tempRPc;
     MaxPwrOneCycRevVar = PwrOneCyc.RPc;
   }

   // Now do alarm using the absolute value of the max reverse power
   if (tempRPa >= Setpoints5.stp.RevReactPwr_Pu * 1000)
   {                                // Above reverse power pickup
      if (RevReacPwrAlmTmr >= (Setpoints5.stp.RevReactPwr_Tt * Setpoints0.stp.Freq) / 100)
      {                             // Alarm delay expired
         if (RevReacPwrAlmFlg == FALSE)
         {
           RevReacPwrAlmTmr = 0;
           TripFlagsReset();                          // to allow alarm to be logged
           RevReacPwrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_REV_KVAR_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_REV_VAR].EIDofStartEvnt = InsertNewEvent(ALARM_REVERSE_KVAR_ENTRY);
           }
         }
      }
      else                          // Under alarm delay.
      {
        RevReacPwrAlmTmr++;         // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_RKVAR;        // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                             // Below reverse power pickup
   {
     if (Dist_Flag & DS_RKVAR)                        // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!RevReacPwrAlmFlg)        // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_RKVAR;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_RKVAR ^ 0xFFFFFFFF);
     }
     RevReacPwrAlmTmr = 0;              // reset timer
     RevReacPwrAlmFlg = FALSE;          // reset flag
   }
}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             RevReactivePower_Alarm()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PF_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Power Factor Alarm function
//
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If the Power Factor value is below the UnderPF_Pu setpoint for a duration
//                      in excess of the UnderPF_Tt, set PfAlmFlg.
//
//                      Above UnderPF_Pu, clear the UnderPFAlmTmr.
//
//                      200ms Power Factor values are used.  The minimums for each phase are calculated in Calc_AppPF().
//
//  CAVEATS:            This function is called from Main() if the Under PF T_A_Off setpoint is set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints6.stp.Power_Prot_Enable
//                      Setpoints6.stp.UnderPF_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints6.stp.UnderPF_Pu: [0.20 to 0.95, step=0.05]
//                      Setpoints6.stp.UnderPF_Tt: [1 to 300, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      PF.App[0..2]
//
//  OUTPUTS:            PFAlmFlg
//
//  ALTERS:             UnderPFAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void PF_Alarm(void)
{
  float tempa, tempb, tempc;

  // If Power Factor alarm is not enabled or is being held off, reset the timer and alarm flag, and return
  if ( (Setpoints6.stp.Power_Prot_Enable != 1) || (Setpoints6.stp.UnderPF_T_A_OFF != 1)
    || (AlarmHoldOffTmr > 0) || (Vll_min < 84) || (CurOneCycMax < 40))
  {
    UnderPFAlmTmr = 0;
    PFAlmFlg = FALSE;
    return;
  }

  // Power Factor alarm function is enabled and not held off

  // Take absolute value of the power factors
  tempa = ( (PF.App[0] < 0) ? (-PF.App[0]) : PF.App[0]);
  tempb = ( (PF.App[1] < 0) ? (-PF.App[1]) : PF.App[1]);
  tempc = ( (PF.App[2] < 0) ? (-PF.App[2]) : PF.App[2]);

  // Store minimum power factor (signed value) in MinProtPF and absolute value in tempa
  MinProtPF = PF.App[0];
  if (tempb < tempa)
  {
    MinProtPF = PF.App[1];
    tempa = tempb;
  }
  if (tempc < tempa)
  {
    MinProtPF = PF.App[2];
    tempa = tempc;
  }

  tempa = 100 * tempa;                // to account for pickup setpoint being X100
  
  // Do alarm function.  Check whether min PF is less than or equal to the setting
  if ((tempa <= Setpoints6.stp.UnderPF_Pu) && (tempa != NAN))
  {                                // Below Power Factor pickup
      if (UnderPFAlmTmr >= (Setpoints6.stp.UnderPF_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired
         if (PFAlmFlg == 0)
         {
           UnderPFAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           PFAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)          // Insert summary event only if in test mode
           {
             InsertNewEvent(ALARM_UNDERPF_ENTRY_TEST);
           }
           else                                       // Insert alarm event with wf capture if not in
           {                                          //   test mode
             if (!Alarm_WF_Capture.InProg)
             {
               Alarm_WF_Capture.Req = TRUE;
             }
             // Save the EID of the event that started the disturbance capture, and insert an entry event
             sListOfDist[DS_UNDER_PF].EIDofStartEvnt = InsertNewEvent(ALARM_UNDERPF_ENTRY);
           }
         }
      }
      else                          // Under alarm delay.
      {
        UnderPFAlmTmr++;            // Increment alarm timer
      }
      // Initiate a disturbance capture.  Note, this is started immediately, regardless of the delay timer,
      //   so that measurements are taken over the entire disturbance.  The capture is abandoned if the
      //   delay time is not reached
      Dist_Flag |= DS_UPF;          // *** DAH  MAY ALSO NEED TO CHECK CONFIG FLAG TO SEE IF DISTURBANCE CAPTURES ARE ENABLED
   }
   else                             // Above PF pickup
   {
     if (Dist_Flag & DS_UPF)                // If a disturbance capture was initiated...
     {
       // If alarm flag is not set, abort the the disturbance capture because no alarm event occurred (the
       //   delay time wasn't met).  If the alarm flag is set, we will end the capture and log the results
       if (!PFAlmFlg)                   // *** DAH  WE PROBABLY WANT TO SEND A DIFFERENT GOOSE MESSAGE HERE TO ABORT THE DISTURBANCE CAPTURE
       {
         Dist_Flag_Cancel |= DS_UPF;
       }
       // Clear the Dist_Flag so the capture is completed (if one had been started and it wasn't aborted)
       Dist_Flag &= (DS_UPF ^ 0xFFFFFFFF);
     }
     UnderPFAlmTmr = 0;                         // reset timer
     PFAlmFlg = FALSE;                          // reset flag
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PF_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PhaseRotation_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Phase Rotation Alarm function (Reverse Sequence)
//
//  MECHANICS:          This routine examines the rotation direction flags set by the Line-Neutral Voltage
//                      Sequence components.  If the Positive Sequence component is greater than the Neqative
//                      Sequence component, the rotation is ABC.  Otherwise, it is CBA. This phase rotation is
//                      compared to the RevSeq_Type setpoint value.
//
//                      If the rotation sequence matches the trip setpoint value, the RevSeqTripTmr starts.  If
//                      it times out based on a fixed delay of 10 cycles, an alarm will be initiated.
//
//                      Perform this routine if the breaker is open or closed since correct phase rotation is a
//                      permissive condition for closing a breaker.
//
//                      A minimum line-to-line voltage of 84vac is required for this protection to be active.
//                      If this conditions is not met, RevSeqTripFlg and RevSeqTripTmr are cleared.  One-cycle
//                      AFE voltages are used.
//
//
//  CAVEATS:            This function is called from Main() every one cycle if the Phase Rotation T_A_Off setpoint is
//                      set to Alarm.
//
//  INPUTS:             AlarmHoldOffTmr
//                      Setpoints5.stp.ExtProtEnable: [0=disabled, 1=enabled]
//                      Setpoints5.stp.RevSeq_T_A_OFF: [0=trip, 1=alarm, 2=off]
//                      Setpoints5.stp.RevSeq_Type: [0=ABC, 1=CBA]    <--(alarms on these sequences)
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      Vll_min, SeqComp.V_PosMag, SeqComp.V_NegMag
//
//  OUTPUTS:            RevSeqAlmFlg
//
//  ALTERS:             ABC_Rotation, ACB_Rotation, RevSeqAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void PhaseRotation_Alarm(void)
{
  // If Phase Rotation alarm is not enabled or is being held off, reset the timer and alarm flag, and return
  if ((Setpoints5.stp.ExtProtEnable != 1) || (Setpoints5.stp.RevSeq_T_A_OFF != 1) || (AlarmHoldOffTmr > 0) )
  {
    RevSeqAlmFlg = 0;
    RevSeqAlmTmr = 0;
    return;
  }

  // Phase Rotation alarm function is enabled and not held off


   if (Vll_min > 84)
   {
      if (SeqComp.V_PosMag > SeqComp.V_NegMag)
      {
         ABC_Rotation = 1;
         ACB_Rotation = 0;
      }
      else
      {
         ABC_Rotation = 0;
         ACB_Rotation = 1;
      }

      switch (Setpoints5.stp.RevSeq_Type)        // 0=ABC, 1=CBA
      {
         case 0:                                 // Set to alarm on ABC
            if (ABC_Rotation == 1)               // Wrong rotation: increment timer
            {
               RevSeqAlmTmr++;
            }
            else if (ACB_Rotation == 1)          // Correct rotation: clear timer
            {
               RevSeqAlmTmr = 0;
            }
          break;

          case 1:                                 // Set to alarm on ACB/CBA
            if (ACB_Rotation == 1)               // Wrong rotation: increment timer
            {
               RevSeqAlmTmr++;
            }
            else if (ABC_Rotation == 1)          // Correct rotation: clear timer
            {
               RevSeqAlmTmr = 0;
            }
          break;

         default:
            RevSeqAlmFlg = 0;
            RevSeqAlmTmr = 0;
          break;
      }


      if (RevSeqAlmTmr >= (10 * Setpoints0.stp.Freq))
      {                                          // Alarm delay timed out after 10 cycles - Alarm
        if (RevSeqAlmFlg == 0)
        {
           RevSeqAlmTmr = 0;
           TripFlagsReset();                     // to allow alarm to be logged
           RevSeqAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)
           {
    //         InsertNewEvent(MINOR_ALARM, (((uint32_t)CAUSE_REVS) << 16) + 0x0300 + ((OpenFlg == 1)? 1 : 2));
           }
           else
           {
             A_Scope_Req = 1;
    //         InsertNewEvent(ALARM, (((uint32_t)CAUSE_REVS) << 16) + 0x0800 + ((OpenFlg == 1)? 1 : 2));
           }
        }
      }
      else
      {
        RevSeqAlmTmr++;                         // Increment alarm timer
      }
   }
   else                                          // Voltage too low to run protection
   {
      RevSeqAlmFlg = 0;
      RevSeqAlmTmr = 0;
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             PhaseRotation_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Neutral_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Neutral Alarm Function.
//
//  MECHANICS:          The neutral alarm function is active if the unit is not in a test (TestMode = 0).
//
//                      If the scaled Neutral One Cycle SOS is above the pickup setpoint for Setpoints1.stp.NeutAlmTime,
//                      set NeutAlmTmr.  The scaling is based on the 0,60,100% neutral ratio setpoint.
//                      The scaling is done in Intrinline_def.h.
//
//
//  CAVEATS:            Called every one cycle only if enabled (Setpoints1.stp.NeutAlmPu != 0)
//
//
//  INPUTS:             Setpoints1.stp.NeutAlmPu [0=off, 10 to 100] %
//                      Setpoints1.stp.NeutAlmTime: [1 to 60] sec
//                      TestMode, NmaxFlg, ScaledCurOneCycSOS_SumF_In
//
//  OUTPUTS:            NeutAlmFlg
//
//  ALTERS:             NeutAlmTmr
//
//  CALLS:              TripFlagsReset(), InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------

void Neutral_Alarm(void)
{
   if (TU_State_TestMode == 0)
   {                                                                // Above neutral pickup & alarm enabled
      if ((Setpoints1.stp.NeutAlmPU > 0) && (ScaledCurOneCycSOS_SumF_In >= Ir * Setpoints1.stp.NeutAlmPU / 100))
      {
         if (NeutAlmTmr >= (Setpoints1.stp.NeutAlmTime * Setpoints0.stp.Freq))
         {
            if (NeutAlmFlg == 0)
            {
               NeutAlmTmr = 0;
               TripFlagsReset();                                   // to allow alarm to be logged
               NeutAlmFlg = 1;
               A_Scope_Req = 1;
    //           InsertNewEvent(ALARM, ((Uint32)CAUSE_LDNEUT << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
            }
         }
         else
         {
            NeutAlmTmr++;
         }
      }
      else                                                       // Below neutral pickup or disabled
      {
         NeutAlmTmr = 0;
         NeutAlmFlg = 0;
      }
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Neutral_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        THDCurrent_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           THD Current Alarm function.
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the Current THD values are above the CurrentTHD_Pu setpoint for a duration
//                      in excess of the CurrentTHD_Tt, set THDCurrAlmFlg.
//
//                      Below CurrentTHD_Pu, clear the THDCurrAlmTmr.
//
//                      One-cycle Current THD values are used.  They are calculated in Calc_DispPF_THD().
//
//  CAVEATS:            If Setpoints9.stp.CurrentTHD_PU = 0, this function is disabled.
//
//  INPUTS:             Setpoints9.stp.CurrentTHD_Pu: [0= off, 10 to 30, step=1] %
//                      Setpoints9.stp.CurrentTHD_Tt: [1 to 60, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      THDminmax[0].THDmax, THDminmax[1].THDmax, THDminmax[2].THDmax   <-(these are in %)
//
//  OUTPUTS:            THDCurrAlmFlg
//
//  ALTERS:             THDCurrAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void THDCurrent_Alarm(void)
{
  if (( (THDminmax[0].THDmax >= Setpoints9.stp.CurrentTHD_Pu) || (THDminmax[1].THDmax >= Setpoints9.stp.CurrentTHD_Pu) || (THDminmax[2].THDmax >= Setpoints9.stp.CurrentTHD_Pu) )
         && (Setpoints9.stp.CurrentTHD_Pu > 0))
   {                                // Above THD pickup
      if (THDCurrAlmTmr >= (Setpoints9.stp.CurrentTHD_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired - alarm
         if (THDCurrAlmFlg == FALSE)
         {
           THDCurrAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           THDCurrAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)
           {
    //         InsertNewEvent(ALARM, ((Uint32)CAUSE_THDC << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
           }
           else                                       // *** BP - do we need separate causes for current and voltage THD
           {
             A_Scope_Req = 1;
     //        InsertNewEvent(ALARM, ((Uint32)CAUSE_THDC << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
           }
         }
      }
      else                          // Under alarm delay.
      {
        THDCurrAlmTmr++;              // Increment alarm timer
      }
   }
   else                             // Below pickup
   {
     THDCurrAlmTmr = 0;             // reset timer
     THDCurrAlmFlg = 0;
   }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             THDCurrent_Alarm()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        THDVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           THD Voltage Alarm function
//
//  MECHANICS:          This routine is executed at the 1 cycle anniversary.
//                      If any of the Voltage THD values are above the Voltage THD_Pu setpoint for a duration
//                      in excess of the VoltTHD_Tt, set THDVoltAlmFlg.
//
//                      Below VoltTHD_Pu, clear the THDVoltAlmTmr.
//
//                      One-cycle Voltage THD values are used.  They are calculated in Calc_DispPF_THD().
//
//  CAVEATS:            If Setpoints9.stp.VoltTHD_Pu = 0, this function is disabled.
//
//  INPUTS:             Setpoints9.stp.VoltTHD_Pu: [0= off, 10 to 30, step=1] %
//                      Setpoints9.stp.VoltTHD_Tt: [1 to 60, step=1] sec
//                      Setpoints0.stp.Freq: [50, 60] Hz
//                      THDminmax[4].THDmax, THDminmax[5].THDmax, THDminmax[6].THDmax   <-(these are in %)
//
//  OUTPUTS:            THDVoltAlmFlg
//
//  ALTERS:             THDVoltAlmTmr
//
//  CALLS:              InsertNewEvent()
//
//------------------------------------------------------------------------------------------------------------
//
void THDVoltage_Alarm(void)
{
   if (( (THDminmax[4].THDmax >= Setpoints9.stp.VoltTHD_Pu) || (THDminmax[5].THDmax >= Setpoints9.stp.VoltTHD_Pu) || (THDminmax[6].THDmax >= Setpoints9.stp.VoltTHD_Pu) )
         && (Setpoints9.stp.VoltTHD_Pu > 0))
   {                                // Above THD pickup
      if (THDVoltAlmTmr >= (Setpoints9.stp.VoltTHD_Tt * Setpoints0.stp.Freq))
      {                             // Alarm delay expired - alarm
         if (THDVoltAlmFlg == FALSE)
         {
           THDVoltAlmTmr = 0;
           TripFlagsReset();                                   // to allow alarm to be logged
           THDVoltAlmFlg = 1;
           if (TU_State_TestUSBMode == TRUE)
           {
      //       InsertNewEvent(ALARM, ((Uint32)CAUSE_THDV << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
           }
           else                                        // *** BP - do we need separate causes for current and voltage THD
           {
             A_Scope_Req = 1;
       //      InsertNewEvent(ALARM, ((Uint32)CAUSE_THDV << 16) | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
           }
         }
      }
      else                          // Under alarm delay.
      {
        THDVoltAlmTmr++;              // Increment alarm timer
      }
   }
   else                             // Below pickup
   {
     THDVoltAlmTmr = 0;             // reset timer
   }


}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             THDVoltage_Alarm()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        TA_Alarm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test the TA for an open / disconnected condition.
//
//  MECHANICS:          This function checks to see if the OpenTA_Voltage flag from TA_Volt_Monitoring()
//                      is set meaning the TA Sense voltage is below the Open TA voltage threshold.
//
//                      If so, begin a one second time delay before setting the BadTaFlg in the Alarm flags.
//
//                      If above the threshold, reset the delay.
//
//  CAVEATS:            This routine is executed at the 1 cycle anniversary if a trip is not in progress
//                      (Delay_ProtHo == 0).
//
//  INPUTS:             Setpoints0.stp.Freq
//                      OpenTA_Voltage
//
//  OUTPUTS:            BadTaFlg
//
//  ALTERS:             BadTaAlmTmr
//
//  CALLS:              InsertNewEvent
//
//------------------------------------------------------------------------------------------------------------
void TA_Alarm(void)
{
   if (OpenTA_Voltage == 1)                            // we are below the Open TA voltage threshold
   {
      if (BadTaAlmTmr > (Setpoints0.stp.Freq * 1))     // Delay timer (1 sec) timed out yet?
      {
         if (BadTaFlg == 0)
         {
             BadTaAlmTmr = 0;
             TripFlagsReset();                         // to allow alarm to be logged
             BadTaFlg = 1;
       //      InsertNewEvent(MINOR_ALARM, ((uint32_t)CAUSE_COIL) << 16 | (0x0800 + ((OpenFlg == 1) ? 1 : 2)));
         }
      }
      else
      {
         BadTaAlmTmr++;
      }
  }
  else                                                 // we are above the Open TA voltage threshold
  {
     BadTaFlg = 0;
     BadTaAlmTmr = 0;
  }
}

//------------------------------------------------------------------------------
//  END OF FUNCTION         TA_Alarm()
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        PKEOverload_Warning()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           PKE Overload Warning Function
//
//  MECHANICS:          This function will calculate the time before tripping on Long Delay (count down timer).
//                      This countdown timer will be shown on the display.  The countdown is active when we are
//                      in Long Delay Pickup.
//
//                      The calculation works for I0.5t, It, I2t, and I4t.  And also for 50 Hz or 60 Hz.
//
//                      After a trip or when the high load condition goes away we can show the thermal capacity
//                      remaining in the breaker.  It is still hot and will not trip at 100% capacity. "Thermal Time Warning"
//
//                      This routine is executed at the 1 cycle anniversary.
//
//  CAVEATS:            This function is only called if we are in Long Delay Pickup.
//
//  INPUTS:             LD_Tally, LD_TripThreshold, LD_TallyIncr, LdPuAlmFlg
//
//  OUTPUTS:            LD_Bucket, LD_TimeToTrip
//
//  ALTERS:
//
//  CALLS:
//
//------------------------------------------------------------------------------------------------------------
//
void PKEOverload_Warning(void)
{
   if (LD_Slope < 4)                               // I0.5t, It, I2t, I4t
   {  
     LD_TimeToTrip =  ((LD_TripThreshold - LD_Tally) / LD_TallyIncr) / Setpoints0.stp.Freq;    // in seconds
   }
   
   else if (LD_Slope < 7)                          // IEEE MI, VI, EI
   {  
     LD_TimeToTrip =  ((PA_TripThreshold - LD_Tally) / LD_TallyIncr)  + (PB_TripThreshold - PB_Passes);
     LD_TimeToTrip =  LD_TimeToTrip / Setpoints0.stp.Freq;                                     // in seconds
   }
   
   else
   {  
     LD_TimeToTrip =  ((PA_TripThreshold - LD_Tally) / LD_TallyIncr) / Setpoints0.stp.Freq;    // in seconds
   }
   
   if (LD_TimeToTrip < 0)                        // IEC A, B, C
   {
      LD_TimeToTrip = 0;
   }

}
//------------------------------------------------------------------------------
//  END OF FUNCTION         PKEOverload_Warning()
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        TripFlagsReset()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Flags reset routine.
//
//
//  MECHANICS:          Clear trip flags if a trip is not in progress (TripReqFlg == 0).
//        
//  CAVEATS:      
//        
//  INPUTS:             TripReqFlg.
//        
//  OUTPUTS:            None.
//        
//  ALTERS:             Flags0...2
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void TripFlagsReset(void)
{
   if (TripReqFlg == 0)                // Clear trip flags only if no trip in progress...
   {
      Trip_Flags0.all = 0;             //    Clear trip indication flags
      Trip_Flags1.all = 0;                  
      Flags1.all = 0;                  //    Clear aux trip flags
      Flags2.all = 0;                  //    Clear aux trip flags
   }
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             TripFlagsReset()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  START OF FUNCTION       Reset_ProtRelated_Alarm()
//------------------------------------------------------------------------------
//
//  FUNCTION:           Resets alarm flags & timers.
//
//  MECHANICS:          Clears all alarm flags and sets alarm counters and timers to initial vlaues.
//        
//  CAVEATS:      
//        
//  INPUTS:             None.
//        
//  OUTPUTS:            None.
//        
//  ALTERS:             See below...
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------

void Reset_ProtRelated_Alarm(void)                   // *** BP - revisit to add all alarms and timers
{
                                    //reset protection related alarm flags (belong to Flags3)...

/*    if (HlAlm1Flg == 1)             // *** DAH  ARE WE SURE WE WANT TO RESET THE HIGH LOAD FLAGS?  IF WE RESET THEM AND HIGH LOAD ALARM IS ENABLED,
    {                               //          YOU WILL GET ANOTHER HIGH LOAD ALARM EVENT (IN Highload_Alarm()) IF THE CURRENT IS STILL ABOVE PICKUP
        HlAlm1Flg = 0;              //          BECAUSE THE FLAG WAS CLEARED
    }
    if (HlAlm2Flg== 1)
    {
        HlAlm2Flg = 0;                  
    }
    if (GndAlmFlg == 1)
    {
        GndAlmFlg = 0;
    }
    if (GfPreAlarmFlg == 1)
    {
        GfPreAlarmFlg = 0;                   
    }
    if (SneakersAlmFlg == 1)
    {
        SneakersAlmFlg = 0;
    }
    if (OvAlmFlg == 1)
    {
        OvAlmFlg = 0;
    }
    if (UvAlmFlg == 1)
    {
        UvAlmFlg = 0;
    }
    if (CurrUnbAlmFlg == 1)
    {
        CurrUnbAlmFlg = 0;
    }
    if (VoltUnbalAlmFlg == 1)
    {
        VoltUnbalAlmFlg = 0;
    }
    if (RevPwrAlmFlg == 1)
    {
        RevPwrAlmFlg = 0;
    }
    if (PhaseLossAlmFlg == 1)
    {
        PhaseLossAlmFlg = 0;
    }
    if (WrongSensorAlmFlg == 1)
    {
        WrongSensorAlmFlg = 0;
    }             */

 
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         Reset_ProtRelated_Alarm()
//------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        UpdateThermalMemory()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Thermal Memory Update on power-up routine.
//
//
//  MECHANICS:          Procedure when handling the thermal memory on power-up:
//                           1) Read the thermal memory capcacitor voltage
//                           2) Read the LD Bucket% value from FRAM
//                           3) Compute the new LD tally register value from the FRAM value and the cap value
//                           4) Store the new LD tally register value in FRAM
//                           5) Then finally, turn on the thermal memory cap charger
//
//                      This way, if there is an intermmittent power-up (wherre the micro goes in and out of reset),
//                      the cap voltage won't be recharged unless the tally register has been updated
//        
//  CAVEATS:            This is called from main() if Setpoints1.stp.ThermMem = 1
//        
//  INPUTS:             
//        
//  OUTPUTS:            LD_Tally
//        
//  ALTERS:             Vcap_pct, LD_Bucket
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void UpdateThermalMemory(void)
{
  
  uint32_t temp2[2];      
  uint16_t temp3[2];       
  float Vcap_pct;
  
  // 1) Read the thermal memory capacitor voltage
  //   Measured the execution time on 181026 (rev 0.28 code): negligible
  ADC3->CR2 |= ADC_CR2_SWSTART;         // Start the conversion
  while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
  {
  }
  ThermalMemoryADC = ((uint16_t)ADC3->DR & 0x0FFF);        
  Vcap_pct = (float)ThermalMemoryADC/VCAP_MAX;       // % of charge on Therm Mem cap at power-up
   
  // 2) Read LD Bucket from FRAM.  If it is invalid, set the number to 0
  FRAM_Read(TM_CAPACITY, 2, &temp3[0]);
  FRAM_Read(TM_CAPACITY_COMP, 2, &temp3[1]);

  if ((temp3[0] ^ temp3[1]) == 0xFFFF)
  {
    LD_Bucket = temp3[0];      // stored percentage (0 - 100), of last bucket value at power-down
  }   
  else
  {
    LD_Bucket = 0;
  }
  
  // 3) Calculate the new LD_Tally value based on stored bucket value and Therm Memory capacitor reading
  LD_Tally = LD_TripThreshold * Vcap_pct * LD_Bucket/100;
  
  if (LD_Tally >= LD_TripThreshold)
  {
     LD_Tally = LD_TripThreshold - 1;
  }
  
  // 4) Store the new LD_Tally bucket percentage in FRAM
  LD_Bucket =  (LD_Tally / LD_TripThreshold) * 100;  
  
  temp2[0] = (uint16_t) LD_Bucket;
  temp2[1] = (uint16_t)(~temp2[0]);

  FRAM_Write(DEV_FRAM2, TM_CAPACITY, 2, (uint16_t *)(&temp2[0]));
  FRAM_Write(DEV_FRAM2, TM_CAPACITY_COMP, 2, (uint16_t *)(&temp2[1]));
                       
  // 5) Turn on the thermal memory cap charger
  TH_MEM_CHARGE_EN;  
 
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             UpdateThermalMemory()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Reset_ThermalMemory()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Thermal Memory Reset.
//
//
//  MECHANICS:          When Thermal Memory Reset Command is received via comms LD_Tally and/or GF_Tally is set to 0.
//        
//  CAVEATS:            This is called from DispComm.c if Thermal Memory Reset command is received
//        
//  INPUTS:             
//        
//  ALTERS:             LD_Tally, GF_Tally
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void Reset_ThermalMemory(uint16_t ThermalMemRst_item)
{
  switch(ThermalMemRst_item)
  {
    case 1:
      LD_Tally = 0;
      break;
    case 2:
      GF_Tally = 0;
      break;
     case 3:
      LD_Tally = 0;
      GF_Tally = 0;
      break;
     default:
      break;    
  }
 
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Reset_ThermalMemory()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Firmware_Simulated_Test()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Firmware Simulated Test routine.
//
//
//  MECHANICS:          
//
//        
//  CAVEATS:            This is called from DispComm_Rx() and also from the Long Delay, Short Delay, Instantaneous,
//                      and Ground Fault routines.
//        
//  INPUTS:             
//        
//  OUTPUTS:            
//        
//  ALTERS:             
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void Firmware_Simulated_Test(void)
{
  
  switch (FW_SimulatedTest.State)            
  {   
    case 0:                                                   // Start test
            
      if (FW_SimulatedTest.Enable == TRUE)
      {  
        FW_SimulatedTest.StartingSampleCounter = SampleCounter;
        FW_SimulatedTest.State++;
        
        FW_SimulatedTest.TestTimeoutTimer = HW_FW_SECINJ_TEST_TIMEOUT;    // Set 2 hour timeout
      }
      break;

      
    case 1:                                                   // Test in process
       
      if (FW_SimulatedTest.TestTimeoutTimer == 0)             // End test after 2 hours
      {
        FW_SimulatedTest.State++;                             // stop injection and report no trip
      }  
      
      
      
      break;    
      
      
      
    case 2:                                                   // Test is finished
      FW_SimulatedTest.EndingSampleCounter = SampleCounter;
      
      FW_SimulatedTestResults();     // <--see Tokyo in Prot.c
      
      FW_SimulatedTest.Enable = FALSE;
      FW_SimulatedTest.State = 0;
      FW_SimulatedTest.TestCurrent = 0; 
      break;       
      
      
  }


}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Firmware_Simulated_Test()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        FW_SimulatedTestResults()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Firmware Simulated Test routine.
//
//
//  MECHANICS:          
//
//        
//  CAVEATS:            This is called from Firmware_Simulated_Test() when the test ends.  The test result time
//                      is calculated and the Breaker Mech time is added to this value.  The one-cycle current 
//                      is recorded for the phase that was tested.
//        
//  INPUTS:             FW_SimulatedTest.StartingSampleCounter, FW_SimulatedTest.EndingSampleCounter, 
//                      Break_Config.config.BreakerFrame
//        
//  OUTPUTS:            FW_SimulatedTest.TestResultTime            
//        
//  ALTERS:             DeltaSampleCounter 
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void FW_SimulatedTestResults(void)
{
  uint32_t DeltaSampleCounter;
  
  
  if (FW_SimulatedTest.EndingSampleCounter > FW_SimulatedTest.StartingSampleCounter)
  {
    DeltaSampleCounter = FW_SimulatedTest.EndingSampleCounter - FW_SimulatedTest.StartingSampleCounter;
  }
  else
  {
    DeltaSampleCounter = FW_SimulatedTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - FW_SimulatedTest.StartingSampleCounter);     
  }
  
  // Convert from sample intervals to milliseconds
  if (Setpoints0.stp.Freq == 60)
  {
    FW_SimulatedTest.TestResultTime = (uint32_t)(DeltaSampleCounter * DELTA_SAMPLES_TO_MSEC_60HZ);
  }
  else
  {
    FW_SimulatedTest.TestResultTime = (uint32_t)(DeltaSampleCounter * DELTA_SAMPLES_TO_MSEC_50HZ);
  }  
    
  
  // Add the Breaker Mech times (from Tokyo)
  switch (Break_Config.config.BreakerFrame)            
  {   
    case MAGNUM_STD:                    
      FW_SimulatedTest.TestResultTime += MAGNUM_STD_MECH_CLEARING_TIME;           
      break;
      
    case MAGNUM_NRW:                    
      FW_SimulatedTest.TestResultTime += MAGNUM_NRW_MECH_CLEARING_TIME;           
      break;
     
    case MAGNUM_STD_DW:                    
      FW_SimulatedTest.TestResultTime += MAGNUM_STD_DW_MECH_CLEARING_TIME;           
      break;
      
    case MAGNUM_NRW_DW:                    
      FW_SimulatedTest.TestResultTime += MAGNUM_NRW_DW_MECH_CLEARING_TIME;           
      break;
      
    default:                    
      FW_SimulatedTest.TestResultTime += MAGNUM_STD_MECH_CLEARING_TIME;           
      break;
  }  

  
  // Save the test current that was measured
  switch (FW_SimulatedTest.Phase)            
  {   
    case TEST_IA:                                                   // Phase A test     
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCyc.Ia;           
      break;
    
    case TEST_IB:                                                   // Phase B test
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCyc.Ib;
      break;    
   
    case TEST_IC:                                                   // Phase C test     
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCyc.Ic;           
      break;
    
    case TEST_IN:                                                   // Phase N test
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCyc.In;
      break; 
      
    case TEST_IG:                                                   // Ig test     
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCycIg;           
      break;
      
    default:                                                        // Phase A test     
      FW_SimulatedTest.TestResultCurrent = (uint32_t)CurOneCyc.Ia;           
      break;    
  }   

  // Fill TesInjVars with the results
  TestInjVars.Current = FW_SimulatedTest.TestResultCurrent;  
  TestInjVars.TestTime = FW_SimulatedTest.TestResultTime;    
                                               
  // Make sure Secondary status gets set to Test (it gets cleared when trip is detected)
  if (FW_SimulatedTest.Trip_NoTrip == NO_TRIP)
  {
    TestNoTrip = 1;
    FW_SimulatedTest.Trip_NoTrip = TRIP;           // remove the No Trip condition
  }
  else
  {
    TestTrip = 1;
  }
  Update_PSC(&StatusCode);
  TestInjVars.PriSecCos = StatusCode;
  
  if (FW_SimulatedTest.TestTimeoutTimer == 0)                // Test timed out
  {
     TestInjVars.Status = TESTINJ_STAT_COMPFAIL;  
  }
  else
  {
     TestInjVars.Status = TESTINJ_STAT_COMPPASS;  
  } 
  
  // Cancel Test case
  if (TestInjVars.Type == TESTINJ_TYPE_CANCELSW)
  {
    TestInjVars.Status = TESTINJ_STAT_COMPFAIL;
    TestInjVars.Current = 0;
    TestInjVars.TestTime = 0;   
  }    
   
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             FW_SimulatedTestResults()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Hardware_SecInj_Test()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Hardware Secondary Injection Test routine.
//
//
//  MECHANICS:          
//
//        
//  CAVEATS:            This is called from DispComm_Rx() and also from the Long Delay, Short Delay, Instantaneous,
//                      and Ground Fault routines.
//        
//  INPUTS:             
//        
//  OUTPUTS:            
//        
//  ALTERS:             
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void Hardware_SecInj_Test(void)
{
  
  switch (HW_SecInjTest.State)            
  {   
    case 0:                                                   // Start test
   
      if (HW_SecInjTest.Enable == TRUE)
      {              
        HW_SecInjTest.TestTimeoutTimer = HW_FW_SECINJ_TEST_TIMEOUT;    // Set 2 hour timeout
        
        HW_SecInjTest.LD_Trip_StartingSampleCounter_Logged = FALSE;        
        HW_SecInjTest.SD_Trip_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.Inst_Trip_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.GF_Trip_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.GF_Alarm_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.Sneaker_Trip_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter_Logged = FALSE;
        HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter_Logged = FALSE;
        
        HW_SecInjTest.State++;                                      // BP - start Test Injection signal here  
        
      }
      break;

      
    case 1:                                                   // Test in process
      
      if (HW_SecInjTest.OvrMicroTrip == 1)                    // wait for cause of trip from I2C comms (every 10ms)
      {
        if ((MM_TripFlg == 1) || (HWInstTripFlg == 1) || (McrTripFlg == 1) || (DigBypassTripFlg == 1))  
        {
          HW_SecInjTest.State++;
        }          
      }
      
      if (HW_SecInjTest.TestTimeoutTimer == 0)                // End test after 2 hours
      {              
        HW_SecInjTest.State++;                                // stop injection and report no trip
      }  
      
      break;    
      
      
      
    case 2:                                                   // Test is finished
      HW_SecInjTest.EndingSampleCounter = SampleCounter;
      
      if (HW_SecInjTest.OvrMicroTrip == 1)
      {
        OvrMicro_SecInjTestResults();                         // for Override micro trips
      }
      else
      {
        HW_SecInjTestResults();                               // for Protection processor trips
      }
      
      if (TestInj.Flags & TEST_INJ_ON)                        // Only if test has been running
      {
        TestInj.Flags |= TEST_INJ_INIT_OFF;                   // Turn off Sec Inj signal
      }  
      HW_SecInjTest.Enable = FALSE;
      HW_SecInjTest.State = 0;
      break;       
      
      
  }


}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Hardware_SecInj_Test()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        HW_SecInjTestResults()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Hardware Secondary Injection Test Results for Protection processor trips
//
//
//  MECHANICS:          
//
//        
//  CAVEATS:            This is called from Hardware_SecInj_Test() when the test ends.  This is for all trips
//                      by the Protection processor.  The test result time is calculated and the Breaker Mech time 
//                      is added to this value.  The one-cycle current is recorded for the phase that was tested.   
//                      
//        
//  INPUTS:             
//        
//  OUTPUTS:            
//        
//  ALTERS:             
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void HW_SecInjTestResults(void)
{
  uint32_t DeltaSampleCounter;
                                    // *** BP - Starting Counter values will be Protection-dependent (LD, SD, Inst, GF, etc)
  if (LdTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.LD_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.LD_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.LD_Trip_StartingSampleCounter);     
    }
  }
  
  if (SdTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.SD_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.SD_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.SD_Trip_StartingSampleCounter);     
    }
  }
  
  if (InstTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.Inst_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.Inst_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.Inst_Trip_StartingSampleCounter);     
    }
  }
   
  if (GndTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.GF_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.GF_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.GF_Trip_StartingSampleCounter);     
    }
  }
  
  if (SneakersTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.Sneaker_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.Sneaker_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.Sneaker_Trip_StartingSampleCounter);     
    }
  }
  
  if (CurrUnbTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.CurUnbal_Trip_StartingSampleCounter);     
    }
  }  
  
  if (PhaseLossTripFlg == 1)
  {
    if (HW_SecInjTest.EndingSampleCounter > HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter)
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter - HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter;
    }
    else
    {
      DeltaSampleCounter = HW_SecInjTest.EndingSampleCounter + ((uint32_t)0xFFFFFFFF - HW_SecInjTest.PhaseLoss_Trip_StartingSampleCounter);     
    }
  }  
  
  
  // Convert from sample intervals to milliseconds
  if (Setpoints0.stp.Freq == 60)
  {
    HW_SecInjTest.TestResultTime = (uint32_t)(DeltaSampleCounter * DELTA_SAMPLES_TO_MSEC_60HZ);
  }
  else
  {
    HW_SecInjTest.TestResultTime = (uint32_t)(DeltaSampleCounter * DELTA_SAMPLES_TO_MSEC_50HZ);
  }  
    
  
  // Add the Breaker Mech times (from Tokyo)
  switch (Break_Config.config.BreakerFrame)            
  {   
    case MAGNUM_STD:                    
      HW_SecInjTest.TestResultTime += MAGNUM_STD_MECH_CLEARING_TIME;           
      break;
      
    case MAGNUM_NRW:                    
      HW_SecInjTest.TestResultTime += MAGNUM_NRW_MECH_CLEARING_TIME;           
      break;
     
    case MAGNUM_STD_DW:                    
      HW_SecInjTest.TestResultTime += MAGNUM_STD_DW_MECH_CLEARING_TIME;           
      break;
      
    case MAGNUM_NRW_DW:                    
      HW_SecInjTest.TestResultTime += MAGNUM_NRW_DW_MECH_CLEARING_TIME;           
      break;
      
    default:                    
      HW_SecInjTest.TestResultTime += MAGNUM_STD_MECH_CLEARING_TIME;           
      break;
  }  
  
  // Set the Test Result Time to 0 for an Override Micro trip (MM, Override, MCR, Digital Bypass)
  if (HW_SecInjTest.OvrMicroTrip == 1)
  {
    HW_SecInjTest.TestResultTime = 0;
    HW_SecInjTest.OvrMicroTrip = 0;
  }  
  
  // Save the test current that was measured
  switch (HW_SecInjTest.Phase)            
  {   
    case TEST_IA:                                                   // Phase A test     
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCyc.Ia;           
      break;
    
    case TEST_IB:                                                   // Phase B test
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCyc.Ib;
      break;    
   
    case TEST_IC:                                                   // Phase C test     
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCyc.Ic;           
      break;
    
    case TEST_IN:                                                   // Phase N test
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCyc.In;
      break; 
      
    case TEST_IG:                                                   // Ig test     
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCycIg;           
      break;
      
    default:                                                       // Phase A test     
      HW_SecInjTest.TestResultCurrent = (uint32_t)CurOneCyc.Ia;           
      break;    
  }   
  
  // Fill TesInjVars with the results
  TestInjVars.Current = HW_SecInjTest.TestResultCurrent;       
  TestInjVars.TestTime = HW_SecInjTest.TestResultTime;        


  // Make sure Secondary status gets set to Test (it gets cleared when trip is detected)
  if (HW_SecInjTest.Trip_NoTrip == NO_TRIP)
  {
    TestNoTrip = 1;
    HW_SecInjTest.Trip_NoTrip = TRIP;           // remove the No Trip condition
  }
  else
  {
    TestTrip = 1;
  }
  Update_PSC(&StatusCode);
  TestInjVars.PriSecCos = StatusCode;
  
  if (HW_SecInjTest.TestTimeoutTimer == 0)                // Test timed out
  {
     TestInjVars.Status = TESTINJ_STAT_COMPFAIL;  
  }
  else
  {
     TestInjVars.Status = TESTINJ_STAT_COMPPASS;  
  } 
  
  
  // Cancel Test case
  if (TestInjVars.Type == TESTINJ_TYPE_CANCELHW)
  {
    TestInjVars.Status = TESTINJ_STAT_COMPFAIL;
    TestInjVars.Current = 0;
    TestInjVars.TestTime = 0;  
  }  
}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             HW_SecInjTestResults()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        OvrMicro_SecInjTestResults()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Hardware Secondary Injection Test Results for Override Micro trips
//
//
//  MECHANICS:          
//
//        
//  CAVEATS:            This is called from Hardware_SecInj_Test() when the test ends.  This is for all trips
//                      by the Override micro.    
//                      
//        
//  INPUTS:             
//        
//  OUTPUTS:            
//        
//  ALTERS:             
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void OvrMicro_SecInjTestResults(void)
{
  // Set the Test Result Time to 0 for an Override Micro trip (MM, Override, MCR)
  HW_SecInjTest.TestResultTime = 0;

    
  // Test Result Current is the trip threshold value
  if (MM_TripFlg == 1)
  {
    HW_SecInjTest.TestResultCurrent = (uint32_t)Break_Config.config.Rating * (uint32_t)MM_TripLevel[Setpoints0.stp.MaintLevel];
  }
  if (HWInstTripFlg == 1)
  {
    HW_SecInjTest.TestResultCurrent = (uint32_t)Break_Config.config.OvrWithstand * 1000;
  } 
  if (McrTripFlg == 1)
  {
    HW_SecInjTest.TestResultCurrent = (uint32_t)Break_Config.config.MCR * 1000;
  } 
 
  // Fill TesInjVars with the results
  TestInjVars.Current = HW_SecInjTest.TestResultCurrent;     
  TestInjVars.TestTime = HW_SecInjTest.TestResultTime;  
  TestInjVars.Status = TESTINJ_STAT_COMPPASS;     
  
  // Make sure Secondary status gets set to Test (it gets cleared when trip is detected)
  if (HW_SecInjTest.Trip_NoTrip == NO_TRIP)
  {
    TestNoTrip = 1;
    HW_SecInjTest.Trip_NoTrip = TRIP;           // remove the No Trip condition
  }
  else
  {
    TestTrip = 1;
  }
  Update_PSC(&StatusCode);
  TestInjVars.PriSecCos = StatusCode;
  
  HW_SecInjTest.OvrMicroTrip = 0;

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             OvrMicro_SecInjTestResults()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Coil_Detection()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Coil Detection routine
//
//
//  MECHANICS:          A 4800Hz square wave from Timer8 is injected into each phasae individually.  The 
//                      one-cycle current value from Calc_Prot_Current is used.  240 samples are taken and 
//                      averaged for 4 seconds of data.  If the avg current is above the COIL_DETECT_THRESHOLD,
//                      the Rogowski is not connected.
//
//                      Rogowski coil impedance for Magnum Standard and Narrow is 40 ohms.
//                      Double-wide and Double-narrow are wired in seires.  Coil impedance is 80 ohms.
//                      These impedances attenuate the current.
//        
//  CAVEATS:            This is called from DispComm_Rx() 
//        
//  INPUTS:             CoilDetect.Avg_Ia, CoilDetect.Avg_Ib, CoilDetect.Avg_Ic, CoilDetect.Avg_In, CoilDetect.SampleCount 
//        
//  OUTPUTS:            CoilDetect.Result_PhA, CoilDetect.Result_PhB, CoilDetect.Result_PhC, CoilDetect.Result_PhN          
//        
//  ALTERS:             CoilDetect.Avg_Ia, CoilDetect.Avg_Ib, CoilDetect.Avg_Ic, CoilDetect.Avg_In
//        
//  CALLS:              Nothing
//        
//------------------------------------------------------------------------------------------------------------
                
void Coil_Detection(void)
{
  
  switch (CoilDetect.State)            
  {   
    case 0:                                                   // Start test
      if (CoilDetect.Enable == TRUE)
      {                     
        // Turn off test injection only if it was on.  Don't set the flag to turn it off if test
        //   injection is already off, because it will then turn off test injection the next time it is
        //   turned on.
        if (TestInj.Flags & TEST_INJ_ON)
        {
          TestInj.Flags |= TEST_INJ_INIT_OFF;
          while (TestInj.Flags & TEST_INJ_INIT_OFF)   // Wait for test injection to be turned off
          {
          }
        }
     
        COIL_TEMP_OFF;                                // Not using measurement multiplexer or ADC3 
        CoilDetect.SampleCount = 0;
        CoilDetect.Avg_Ia = 0;
        CoilDetect.Result = 0;
        CoilDetect.State++;                                               
      }
      break;

      
    case 1:                                           // Test Phase A       
      TIM8->CR1 |= 0x0001;                            // Enable the Timer8 clock which starts the 4800Hz square wave output
      TSTINJ_PHA_ON;                                  // Turn on the test channel
      
      COIL_TEMP_4800_ON;                              // Turn on the 4800Hz coil excitation signal

      if (CoilDetect.SampleCount == 239)              // Finished calculating average of 4 seconds of one-cycle currents
      {
        CoilDetect.Avg_Ia = CoilDetect.Avg_Ia/240;     // 240 readings were summed in Calc_Prot_Current() 
        if (CoilDetect.Avg_Ia > COIL_DETECT_THRESHOLD)
        {
          CoilDetect.Result |= BIT0;                  // Rogowski is connected for this phase - Open
        }
        
        TSTINJ_PHA_OFF;                               // Turn off the test channel
        CoilDetect.SampleCount = 0;
        CoilDetect.State++;
      }   
      break;    
      
    case 2:                                           // Test Phase B       
      TSTINJ_PHB_ON;                                  // Turn on the test channel

      if (CoilDetect.SampleCount == 239)              // Finished calculating average of 4 seconds of one-cycle currents
      {
        CoilDetect.Avg_Ib = CoilDetect.Avg_Ib/240;     // 240 readings were summed in Calc_Prot_Current() 
        if (CoilDetect.Avg_Ib > COIL_DETECT_THRESHOLD)
        {
          CoilDetect.Result |= BIT1;                  // Rogowski is not connected for this phase - Open
        }
                
        TSTINJ_PHB_OFF;                               // Turn off the test channel
        CoilDetect.SampleCount = 0;
        CoilDetect.State++;
      }   
      break;
       
    case 3:                                           // Test Phase C       
      TSTINJ_PHC_ON;                                  // Turn on the test channel

      if (CoilDetect.SampleCount == 239)              // Finished calculating average of 4 seconds of one-cycle currents
      {
        CoilDetect.Avg_Ic = CoilDetect.Avg_Ic/240;     // 240 readings were summed in Calc_Prot_Current()        
        if (CoilDetect.Avg_Ic > COIL_DETECT_THRESHOLD)
        {
          CoilDetect.Result |= BIT2;                  // Rogowski is not connected for this phase - Open
        }
        
        TSTINJ_PHC_OFF;                               // Turn off the test channel
        CoilDetect.SampleCount = 0;
        
        if ((Break_Config.config.Poles == 4) ||(Break_Config.config.Poles == 8))  
        {  
          CoilDetect.State++;                         // Test Neutral phase
        }  
        else
        {
          CoilDetect.Result |= BIT3;                  // Rogowski is not connected for this phase - Open
          CoilDetect.State = 5;                       // No Neutral phase
        }
      }   
      break;     
      
    case 4:                                           // Test Phase N       
      TSTINJ_PHN_ON;                                  // Turn on the test channel

      if (CoilDetect.SampleCount == 239)              // Finished calculating average of 4 seconds of one-cycle currents
      {
        CoilDetect.Avg_In = CoilDetect.Avg_In/240;     // 240 readings were summed in Calc_Prot_Current()               
        if (CoilDetect.Avg_In > COIL_DETECT_THRESHOLD) 
        {
          CoilDetect.Result |= BIT3;                  // Rogowski is not connected for this phase - Open
        }
        
        TSTINJ_PHN_OFF;                               // Turn off the test channel
        CoilDetect.SampleCount = 0;
        CoilDetect.State++;
      }   
      break;         
      
    case 5:                                           // Test is finished
      COIL_TEMP_4800_OFF;                             // Turn off the 4800Hz excitation signal
      TIM8->CR1 &= 0xFFFE;                            // Disable the Timer8 clock which stops the 4800Hz square wave output
      TSTINJ_PHA_OFF;                                 // Turn off the test channels
      TSTINJ_PHB_OFF;
      TSTINJ_PHC_OFF;
      TSTINJ_PHN_OFF;
            
      TestInj.Flags |= TEST_INJ_INIT_OFF;             // Turn off Coil Inj signal
      CoilDetect.Enable = FALSE;
      CoilDetect.State = 0;
      CoilDetect.TestInProgress = 0;                 
      break;       
      
      
  }


}

//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             Coil_Detection()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Brk_Config_DefaultInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Initialize PXR35 Breaker Configuration to Default Values
//
//
//  MECHANICS:          Write default PXR35 breaker configuration into Frame FRAM.  This initializes the
//                      following sections:
//                          - 403-byte section beginning at 0x0030 (3 copies)
//                          -  18-byte section beginning at 0x1FC0 (3 copies)
//        
//  CAVEATS:            None
//        
//  INPUTS:             DFLT_BKRCFG_xxx, BRKCONFIG_BLOCK_SIZE 
//        
//  OUTPUTS:            Break_Config.config.xxx
//        
//  ALTERS:             SPI2_buf[]
//        
//  CALLS:              Frame_FRAM_Write(), Checksum8_16(), Save_Critical_BrkConfig()
//        
//------------------------------------------------------------------------------------------------------------

void Brk_Config_DefaultInit(void)
{
  uint16_t i, ckSum_cal;

  // Clear SPI2_buf[] since most of the values are 0
  for (i = 0; i < BRKCONFIG_BLOCK_SIZE; i++)
  {
    SPI2_buf[i] = 0;
  }
  // Now fill in the default values
  SPI2_buf[0] = (uint8_t)DFLT_BKRCFG_RATING;
  SPI2_buf[1] = (uint8_t)(((uint16_t)DFLT_BKRCFG_RATING) >> 8);
  SPI2_buf[2] = (uint8_t)DFLT_BKRCFG_FRAME;
  SPI2_buf[3] = (uint8_t)(((uint16_t)DFLT_BKRCFG_FRAME) >> 8);
  SPI2_buf[234] = (uint8_t)DFLT_BKRCFG_POLES;
  SPI2_buf[235] = (uint8_t)(((uint16_t)DFLT_BKRCFG_POLES) >> 8);
  SPI2_buf[236] = (uint8_t)DFLT_BKRCFG_STANDARD;
  SPI2_buf[237] = (uint8_t)(((uint16_t)DFLT_BKRCFG_STANDARD) >> 8);
  SPI2_buf[238] = (uint8_t)DFLT_BKRCFG_DEV_TYPE;
  SPI2_buf[239] = (uint8_t)(((uint16_t)DFLT_BKRCFG_DEV_TYPE) >> 8);
  SPI2_buf[240] = (uint8_t)DFLT_BKRCFG_DC_RATING_CAP;
  SPI2_buf[241] = (uint8_t)(((uint16_t)DFLT_BKRCFG_DC_RATING_CAP) >> 8);
  SPI2_buf[242] = (uint8_t)DFLT_BKRCFG_VOLT_MTR_SRC;
  SPI2_buf[243] = (uint8_t)(((uint16_t)DFLT_BKRCFG_VOLT_MTR_SRC) >> 8);
  SPI2_buf[244] = (uint8_t)DFLT_BKRCFG_MAXFRM_IEC;
  SPI2_buf[245] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAXFRM_IEC) >> 8);
  SPI2_buf[246] = (uint8_t)DFLT_BKRCFG_MAXFRM_UL489;
  SPI2_buf[247] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAXFRM_UL489) >> 8);
  SPI2_buf[248] = (uint8_t)DFLT_BKRCFG_MAXFRM_UL1066;
  SPI2_buf[249] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAXFRM_UL1066) >> 8);
  SPI2_buf[250] = (uint8_t)DFLT_BKRCFG_FRAME_AMPS;
  SPI2_buf[251] = (uint8_t)(((uint16_t)DFLT_BKRCFG_FRAME_AMPS) >> 8);
  SPI2_buf[252] = (uint8_t)DFLT_BKRCFG_MIN_IN_SETTING;
  SPI2_buf[253] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MIN_IN_SETTING) >> 8);
  SPI2_buf[254] = (uint8_t)DFLT_BKRCFG_MAX_WITHSTAND;
  SPI2_buf[255] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAX_WITHSTAND) >> 8);
  SPI2_buf[256] = (uint8_t)DFLT_BKRCFG_OVR_WITHSTAND;
  SPI2_buf[257] = (uint8_t)(((uint16_t)DFLT_BKRCFG_OVR_WITHSTAND) >> 8);
  SPI2_buf[258] = (uint8_t)DFLT_BKRCFG_MCR;
  SPI2_buf[259] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MCR) >> 8);
  SPI2_buf[260] = (uint8_t)DFLT_BKRCFG_MAX_GF;
  SPI2_buf[261] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAX_GF) >> 8);
  SPI2_buf[262] = (uint8_t)DFLT_BKRCFG_MAX_INTERRUPT;
  SPI2_buf[263] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAX_INTERRUPT) >> 8);
  SPI2_buf[264] = (uint8_t)DFLT_BKRCFG_MAX_INT_LABEL;
  SPI2_buf[265] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAX_INT_LABEL) >> 8);
  SPI2_buf[266] = (uint8_t)DFLT_BKRCFG_MAX_INST_TRIP;
  SPI2_buf[267] = (uint8_t)(((uint16_t)DFLT_BKRCFG_MAX_INST_TRIP) >> 8);
  SPI2_buf[293] = (uint8_t)DFLT_BKRCFG_IN_REPROGRAM;
  SPI2_buf[294] = (uint8_t)(((uint32_t)DFLT_BKRCFG_IN_REPROGRAM) >> 8);
  SPI2_buf[295] = (uint8_t)(((uint32_t)DFLT_BKRCFG_IN_REPROGRAM) >> 16);
  SPI2_buf[296] = (uint8_t)(((uint32_t)DFLT_BKRCFG_IN_REPROGRAM) >> 24);
  SPI2_buf[397] = (uint8_t)DFLT_BKRCFG_CT_VERSION;
  SPI2_buf[398] = (uint8_t)(((uint16_t)DFLT_BKRCFG_CT_VERSION) >> 8);

  // Compute the checksum and then write the default values into Frame FRAM
  ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), BRKCONFIG_BLOCK_SIZE);
  SPI2_buf[BRKCONFIG_BLOCK_SIZE + 0] = (uint8_t)ckSum_cal;
  SPI2_buf[BRKCONFIG_BLOCK_SIZE + 1] = (uint8_t)(ckSum_cal >> 8);
  SPI2_buf[BRKCONFIG_BLOCK_SIZE + 2] = (uint8_t)(ckSum_cal ^ 0xFFFF);
  SPI2_buf[BRKCONFIG_BLOCK_SIZE + 3] = (uint8_t)((ckSum_cal ^ 0xFFFF) >> 8);
                            
  // First "+4" to include the checksum and checksum complement also
  // Second "+4" in address computation because there are 4 spare bytes between copies
  for (i = 0; i < 3; i++)
  {
    Frame_FRAM_Write((BRKCONFIG_FADDR + (i * (BRKCONFIG_BLOCK_SIZE+4+4))), (BRKCONFIG_BLOCK_SIZE+4),
                            &SPI2_buf[0]);
  }
  
  // Load the default values into critical breaker values
  Break_Config.config.Rating = DFLT_BKRCFG_RATING;
  Break_Config.config.BreakerFrame = DFLT_BKRCFG_FRAME;
  Break_Config.config.Standard = DFLT_BKRCFG_STANDARD;
  Break_Config.config.MaxInstTripSetting = DFLT_BKRCFG_MAX_INST_TRIP;
  Break_Config.config.Poles = DFLT_BKRCFG_POLES;
  Break_Config.config.MCR = DFLT_BKRCFG_MCR;
  Break_Config.config.OvrWithstand = DFLT_BKRCFG_OVR_WITHSTAND;

  // Finally, save the critical breaker values in Frame FRAM
  Save_Critical_BrkConfig();

}

//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION          Brk_Config_DefaultInit()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Get_BrkConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Breaker Configuration From PXR35 Frame
//
//
//  MECHANICS:          Retrieve breaker config values from Frame FRAM into SPI2_buf[].  There are 3 copies
//                      of the config data.  The code reads each one in order.  As soon as a valid block is
//                      read, those values are used and the function returns True.  If no block is valid
//                      (all three copies have bad checksums), the function returns False.
//        
//  CAVEATS:            None
//        
//  INPUTS:             BRKCONFIG_FADDR, BRKCONFIG_BLOCK_SIZE
//        
//  OUTPUTS:            SPI2_buf[]
//                      The function returns True if the values were retrieved successfully, False otherwise
//        
//  ALTERS:             None
//        
//  CALLS:              Frame_FRAM_Read(), Checksum8_16()
//        
//------------------------------------------------------------------------------------------------------------

uint8_t Get_BrkConfig(void)
{
  uint16_t ckSum_cal, ckSum_read, ckSumNot_read;
  uint16_t i;
  uint8_t valid = FALSE;

  for (i = 0; i < 3; ++i)
  {
  // First "+4" to include the checksum and checksum complement also
  // Second "+4" in address computation because there are 4 spare bytes between copies
    Frame_FRAM_Read((BRKCONFIG_FADDR + (i * (BRKCONFIG_BLOCK_SIZE+4+4))), (BRKCONFIG_BLOCK_SIZE+4),
                            &SPI2_buf[0]);

    ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), BRKCONFIG_BLOCK_SIZE);
    ckSum_read = SPI2_buf[BRKCONFIG_BLOCK_SIZE + 0] + (((uint16_t)SPI2_buf[BRKCONFIG_BLOCK_SIZE + 1]) << 8);
    ckSumNot_read = SPI2_buf[BRKCONFIG_BLOCK_SIZE + 2]
                        + (((uint16_t)SPI2_buf[BRKCONFIG_BLOCK_SIZE + 3]) << 8);

    if ((ckSum_cal == ckSum_read) && ((ckSum_cal ^ 0xFFFF)  == ckSumNot_read))
    {
      valid = TRUE;
      break;
    }
  }

  return valid;   
}

//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION          Get_BrkConfig()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Get_Critical_BrkConfig_PXR35()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Critical Breaker Configuration From PXR35 Frame
//
//
//  MECHANICS:          Retrieve critical breaker config values from the corresponding PXR35 section in
//                      Frame FRAM.  There are 3 copies of the config data.  The code reads each one in
//                      order.  As soon as a valid block is read, those values are used and the function
//                      returns True.  If no block is valid (all three copies have bad checksums), the
//                      function returns False.
//        
//  CAVEATS:            None
//        
//  INPUTS:             CRIT_BRKCONFIG_FADDR_35, CRIT_BRKCONFIG_BLOCK_SIZE_35
//        
//  OUTPUTS:            Break_Config.config.Rating, Break_Config.config.BreakerFrame,
//                      Break_Config.config.Standard, Break_Config.config.MaxInstTripSetting, 
//                      Break_Config.config.Poles, Break_Config.config.MCR, Break_Config.config.OvrWithstand
//                      The function returns True if the values were retrieved successfully, False otherwise
//        
//  ALTERS:             SPI2_buf[]
//        
//  CALLS:              Frame_FRAM_Read(), Checksum8_16()
//        
//------------------------------------------------------------------------------------------------------------

uint8_t Get_Critical_BrkConfig_PXR35(void)
{
  uint16_t ckSum_cal, ckSum_read, ckSumNot_read;
  uint8_t i;

  uint8_t valid = FALSE;

  for (i = 0; i < 3; ++i)
  {
    // "+4" to read the checksum and checksum complement also
    Frame_FRAM_Read((CRIT_BRKCONFIG_FADDR_35 + (i * (CRIT_BRKCONFIG_BLOCK_SIZE_35+4))),
                        (CRIT_BRKCONFIG_BLOCK_SIZE_35+4), &SPI2_buf[0]);

    ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), CRIT_BRKCONFIG_BLOCK_SIZE_35);
    ckSum_read = SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 0]
                    + (((uint16_t)SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 1]) << 8);
    ckSumNot_read = SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 2]
                    + (((uint16_t)SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 3]) << 8);

    if ((ckSum_cal == ckSum_read) && ((ckSum_cal ^ 0xFFFF) == ckSumNot_read))
    {
      Break_Config.config.Rating = ( SPI2_buf[0] + (((uint16_t)SPI2_buf[1]) << 8) );
      Break_Config.config.BreakerFrame = ( SPI2_buf[2] + (((uint16_t)SPI2_buf[3]) << 8) );
      Break_Config.config.Standard = ( SPI2_buf[4] + (((uint16_t)SPI2_buf[5]) << 8) );
      Break_Config.config.MaxInstTripSetting = ( SPI2_buf[6] + (((uint16_t)SPI2_buf[7]) << 8) );
      Break_Config.config.Poles = ( SPI2_buf[8] + (((uint16_t)SPI2_buf[9]) << 8) );
      Break_Config.config.MCR = ( SPI2_buf[10] + (((uint16_t)SPI2_buf[11]) << 8) );
      Break_Config.config.OvrWithstand = ( SPI2_buf[12] + (((uint16_t)SPI2_buf[13]) << 8) );
      valid = TRUE;
      break;
    }
  }

  return valid;   
}

//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION          Get_Critical_BrkConfig_PXR35()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Get_Critical_BrkConfig_PXR25()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Critical Breaker Configuration From PXR25 Frame
//
//
//  MECHANICS:          Retrieve critical breaker config values from the corresponding PXR25 section in
//                      Frame FRAM.  There are 3 copies of the config data.  The code reads each one in
//                      order.  As soon as a valid block is read, those values are used and the function
//                      returns True.  If no block is valid (all three copies have bad checksums), the
//                      function returns False.
//        
//  CAVEATS:            None
//        
//  INPUTS:             CRIT_BRKCONFIG_FADDR_35, CRIT_BRKCONFIG_BLOCK_SIZE_35
//        
//  OUTPUTS:            Break_Config.config.Rating, Break_Config.config.BreakerFrame,
//                      Break_Config.config.Standard, Break_Config.config.MaxInstTripSetting 
//                      The function returns True if the values were retrieved successfully, False otherwise
//        
//  ALTERS:             SPI2_buf[]
//        
//  CALLS:              Frame_FRAM_Read(), Checksum8_16()
//        
//------------------------------------------------------------------------------------------------------------

uint8_t Get_Critical_BrkConfig_PXR25(void)
{
  uint16_t ckSum_cal, ckSum_read, ckSumNot_read;
  uint8_t i;

  uint8_t valid = FALSE;

  for (i = 0; i < 3; ++i)
  {
    // First "+4" to read the checksum and checksum complement also
    // Second "+4" in address computation because there are 4 spare bytes between copies
    Frame_FRAM_Read((CRIT_BRKCONFIG_FADDR_25 + (i * (CRIT_BRKCONFIG_BLOCK_SIZE_25+4+4))),
                        (CRIT_BRKCONFIG_BLOCK_SIZE_25+4), &SPI2_buf[0]);

    ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), CRIT_BRKCONFIG_BLOCK_SIZE_25);
    ckSum_read = SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_25 + 0]
                    + (((uint16_t)SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_25 + 1]) << 8);
    ckSumNot_read = SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_25 + 2]
                    + (((uint16_t)SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_25 + 3]) << 8);

    if ((ckSum_cal == ckSum_read) && ((ckSum_cal ^ 0xFFFF)  == ckSumNot_read))
    {
      // Use PXR25 settings where available
      Break_Config.config.Rating = ( SPI2_buf[0] + (((uint16_t)SPI2_buf[1]) << 8) );
      Break_Config.config.BreakerFrame = ( SPI2_buf[2] + (((uint16_t)SPI2_buf[3]) << 8) );
      Break_Config.config.Standard = ( SPI2_buf[4] + (((uint16_t)SPI2_buf[5]) << 8) );
      Break_Config.config.MaxInstTripSetting = ( SPI2_buf[6] + (((uint16_t)SPI2_buf[7]) << 8) );
      valid = TRUE;
      break;
    }
  }

  return valid;   
}

//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION          Get_Critical_BrkConfig_PXR25()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Save_Critical_BrkConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Critical Breaker Configuration Into PXR35 Frame
//
//
//  MECHANICS:          Write critical breaker config values into the corresponding PXR35 section in Frame
//                      FRAM.  There are 3 copies of the config data.  The code reads each one in order.
//        
//  CAVEATS:            None
//        
//  INPUTS:             CRIT_BRKCONFIG_FADDR_35, CRIT_BRKCONFIG_BLOCK_SIZE_35, Break_Config.buf[]
//        
//  OUTPUTS:            None
//        
//  ALTERS:             SPI2_buf[]
//        
//  CALLS:              Frame_FRAM_Write(), Checksum8_16()
//
//  EXECUTION TIME:     Measured on 240109: (rev 140 code): 103usec
//        
//------------------------------------------------------------------------------------------------------------

void Save_Critical_BrkConfig(void)
{
  uint16_t i, ckSum_cal;

  for (i = 0; i < CRIT_BRKCONFIG_BLOCK_SIZE_35; i++)
  {
    SPI2_buf[i] = Break_Config.buf[i];
  }
  ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), CRIT_BRKCONFIG_BLOCK_SIZE_35);
  SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 0] = (uint8_t)ckSum_cal;
  SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 1] = (uint8_t)(ckSum_cal >> 8);
  SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 2] = (uint8_t)(ckSum_cal ^ 0xFFFF);
  SPI2_buf[CRIT_BRKCONFIG_BLOCK_SIZE_35 + 3] = (uint8_t)((ckSum_cal ^ 0xFFFF) >> 8);
  
  // "+4" to account for checksum and checksum complement (note, there are no bytes between copies)
  for (i = 0; i < 3; i++)
  {
    Frame_FRAM_Write((CRIT_BRKCONFIG_FADDR_35 + (i * (CRIT_BRKCONFIG_BLOCK_SIZE_35+4))),
                        (CRIT_BRKCONFIG_BLOCK_SIZE_35+4), &SPI2_buf[0]);
  }
}

//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION          Save_Critical_BrkConfig()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Write_FrameFRAM_Default_Section()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Default Configuration (All Zeroes) Frame FRAM
//
//
//  MECHANICS:          Write critical breaker config values into the corresponding PXR35 section in Frame
//                      FRAM.  There are 3 copies of the config data.  The code reads each one in order.
//        
//  CAVEATS:            It is assumed the default configuration is all zeroes
//                      It is assumed there are 4 spacer bytes between Frame FRAM copies
//        
//  INPUTS:             address - starting address of the first copy
//                      length - length of the data in bytes, NOT INCLUDING THE CHECKSUM AND SPACING BYTES!!
//                      copies - number of copies to be written
//        
//  OUTPUTS:            None
//        
//  ALTERS:             SPI2_buf[]
//        
//  CALLS:              Frame_FRAM_Write(), Checksum8_16()
//        
//------------------------------------------------------------------------------------------------------------

void Write_FrameFRAM_Default_Section(uint16_t address, uint16_t length, uint8_t copies)
{
  uint16_t ckSum_cal, i; 

  for(i = 0; i < length; i++)
  {
    SPI2_buf[i] = 0;  
  }                 

  ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), length);
  SPI2_buf[length + 0] = (uint8_t)ckSum_cal;
  SPI2_buf[length + 1] = (uint8_t)(ckSum_cal >> 8);
  SPI2_buf[length + 2] = (uint8_t)(ckSum_cal ^ 0xFFFF);
  SPI2_buf[length + 3] = (uint8_t)((ckSum_cal ^ 0xFFFF) >> 8);

  // First "+4" to include the checksum and checksum complement also
  // Second "+4" in address computation because there are 4 spare bytes between copies
  for (i = 0; i < copies; i++)
  {
    Frame_FRAM_Write((address + (i * (length+4+4))), (length+4), &SPI2_buf[0]);
  }
}

//------------------------------------------------------------------------------
//  END OF FUNCTION     Write_FrameFRAM_Default_Section()
//------------------------------------------------------------------------------


