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
//  MODULE NAME:        Prot_def.h
//
//  MECHANICS:          This is the definitions file for the Prot.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.16   160818  DAH - Added Pickup Flag definitions
//   0.45   210514  BP  - Changes and additions from Bert Popovich
//                          - Added LD_PICKUP and PBLD_PICKUP to support long delay protection
//   0.70   230223  BP  - Added Overtemp definitions
//                      - Added Trip and Alarm flags
//   0.72   230320  BP  - Changed Trip and Alarm masks
//                      - Added VCAP_MAX
//    37    230516  DAH - Added trip pickup flag definitions (xxTripPuFlg)
//    44    230623  BP  - Added breaker frame and breaker configuration definitions
//    54    230802  BP  - Added Digital Bypass trip and alarm flags
//    93    231010  BP  - Added Breaker clearing times for Sec Injection
//                      - Added/modified the Secondary Injection structures
//    99    231019  BP  - Added Trip/No Trip for Secondary Injection
//   104    231102  BP  - Added TimeToTripFlag and ThermCapacityFlg 
//   122    231204  BP  - Added Coil Detect stucture and detection threshold
//   125    231212  BP  - Changed name of Breaker Config Fault
//                      - Added Test in Progress flag for Coil Detection
//   133    231219  DAH - Renamed CRITICAL_BRKCONFIG_FADDR to CRIT_BRKCONFIG_FADDR_25 and added
//                        CRIT_BRKCONFIG_FADDR_35 to support new PXR35 config info
//                      - Renamed CRITICAL_BRKCONFIG_BLOCK_SIZE to CRIT_BRKCONFIG_BLOCK_SIZE_25 and
//                        hard-coded it to 8
//                      - Added CRIT_BRKCONFIG_BLOCK_SIZE_35 to support the new PXR35 config info
//                      - Added default breaker configuration constants DFLT_BKRCFG_xxx
//                      - Deleted struct BRKCONFIG_DEF and union BREAKER_CONFIG
//   141    240115  BP  - Added T_forbid flag for Sec Inj and Coil Detection
//                      - Changed some of the Auxiliary (Electrical) alarms 
//   148    240132  BP  - Added AuxVoltValid flag bit
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
//    Constants
//------------------------------------------------------------------------------------------------------------

// Pickup flag definitions
#define INST_PICKUP         0x0001
#define SD_PICKUP           0x0002
#define LD_PICKUP           0x0004
#define PBLD_PICKUP         0x0008  


//threshold definitions for over temperature
#define TEMP_TRIP_THRESHOLD               105               // 105 degC

#define DISPLAY_TEMP_WARNING_SET           70               // 70 degC   (Max operating temperature for Display)
#define DISPLAY_TEMP_WARNING_CLEAR         65               // 65 degC


#define OP_COUNT_THRESHOLD              10000               // operations count (Breaker Health) alarm

#define BATTERY_LOWVOLT_THRESHOLD         2.5               // Low Battery voltage alarm

#define THER_MEM_ALM_TIME                   2               // Thermal Memory Alarm time - 2 sec

#define HIGH_N_CURRENT                  10000

#define VCAP_MAX                         3328               // 2.5V charge voltage minus diode drop
                                                            //     (3328 x 2.5V/4095 = 2.03V)

// Breaker frame definitions
#define NRX_NF                           0x00
#define NRX_RF                           0x01
#define MAGNUM_STD                       0x02               // Magnum Standard
#define MAGNUM_NRW                       0x03               // Magnum Narrow
#define MAGNUM_STD_DW                    0x04               // Magnum Standard Double Wide
#define MAGNUM_NRW_DW                    0x05               // Magnum Narrow Double Wide

// Breaker Clearing Times for Secondary Injection (from Tokyo)    
#define NF_MECH_CLEARING_TIME              25
#define RF_MECH_CLEARING_TIME              30
#define MAGNUM_STD_MECH_CLEARING_TIME      25               // Magnum Standard
#define MAGNUM_NRW_MECH_CLEARING_TIME      25               // Magnum Narrow
#define MAGNUM_STD_DW_MECH_CLEARING_TIME   25               // Magnum Standard Double Wide            
#define MAGNUM_NRW_DW_MECH_CLEARING_TIME   25               // Magnum Narrow Double Wide

// Breaker Test Phases for Secondary Injection
#define TEST_IA                          0x00
#define TEST_IB                          0x01
#define TEST_IC                          0x02
#define TEST_IN                          0x03
#define TEST_IG                          0x04

#define TRIP                             0x00                // want default to be Trip
#define NO_TRIP                          0x01            

// Sec Inj Conversion constants for converting Delta Sampling Counter to miliseconds
//   60Hz:  16.66ms/80 samples = 208.333us between samples
//                             = 0.208333ms between samples
//   50Hz:  20ms/80 samples = 250.0us between samples
//                          = 0.25ms between samples
#define DELTA_SAMPLES_TO_MSEC_60HZ       0.208333
#define DELTA_SAMPLES_TO_MSEC_50HZ       0.2500

#define HW_FW_SECINJ_TEST_TIMEOUT     720000       // 2 hour timeout using 10ms timer
#define COIL_DETECT_THRESHOLD            140       // Above 140A means Rogowski is not connected

// Frame FRAM address
#define BRKCONFIG_FADDR             0x30
#define FRAMECONFIG_FADDR           0x660
#define CRIT_BRKCONFIG_FADDR_25     0x068A
#define CRIT_BRKCONFIG_FADDR_35     0x1FC0
#define VDBCONFIG_LINE_FADDR        0x76A
#define VDBCONFIG_LOAD_FADDR        0x794

// Default breaker configuration
#define DFLT_BKRCFG_RATING              800
#define DFLT_BKRCFG_FRAME               MAGNUM_NRW
#define DFLT_BKRCFG_POLES               4             // 3=3 pole breaker; 4=4 pole breaker
#define DFLT_BKRCFG_STANDARD            2             // 1=IEC; 2=UL1066/ANSI; 3=UL489; 4=CCC
#define DFLT_BKRCFG_DEV_TYPE            0
#define DFLT_BKRCFG_DC_RATING_CAP       0
#define DFLT_BKRCFG_VOLT_MTR_SRC        0             // 1=external voltage sensing
#define DFLT_BKRCFG_MAXFRM_IEC          0             // frame max allowable continuous IEC RMS Amps
#define DFLT_BKRCFG_MAXFRM_UL489        0             // frame physical max allowable continuous UL489 Amps
#define DFLT_BKRCFG_MAXFRM_UL1066       0             // frame physical max allowable cont. ANSI/UL1066 Amps
#define DFLT_BKRCFG_FRAME_AMPS          630           // customer purchased frame amps RMS
#define DFLT_BKRCFG_MIN_IN_SETTING      200
#define DFLT_BKRCFG_MAX_WITHSTAND       0             // withstand limit Amps (Icw 1/2s, 1s) max capability
                                                      //   (not used in circuit)
#define DFLT_BKRCFG_OVR_WITHSTAND       90            // override pk current used for High Inst. EEPOT steps
                                                      //   setting (lowest override trip level for NF brkr)
#define DFLT_BKRCFG_MCR                 36            // MCR peak current used for MCR EEPOT steps setting
                                                      //   (lowest MCR trip level for NF breaker)
#define DFLT_BKRCFG_MAX_GF              100
#define DFLT_BKRCFG_MAX_INTERRUPT       0
#define DFLT_BKRCFG_MAX_INT_LABEL       42
#define DFLT_BKRCFG_MAX_INST_TRIP       15
#define DFLT_BKRCFG_IN_REPROGRAM        0
#define DFLT_BKRCFG_CT_VERSION          1             //   0 = RF sensor, 1 = Magnum sensor



//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct BITS
{
    uint16_t b0:  1;                  // LSB
    uint16_t b1:  1;
    uint16_t b2:  1;
    uint16_t b3:  1;
    uint16_t b4:  1;
    uint16_t b5:  1;
    uint16_t b6:  1;
    uint16_t b7:  1;
    uint16_t b8:  1;
    uint16_t b9:  1;
    uint16_t b10: 1;
    uint16_t b11: 1;
    uint16_t b12: 1;
    uint16_t b13: 1;
    uint16_t b14: 1;
    uint16_t b15: 1;                  // MSB
};
union WORD_BITS
{
    uint16_t    all;
    struct BITS bit;
};

//------------------------------------------------------------------------------------------------------------
//          Trip Flags...
//------------------------------------------------------------------------------------------------------------

// Trip Flags. The following flags are cleared by TripFlagsReset.
#define LdTripFlg          Trip_Flags0.bit.b0        // Long Delay Trip
#define SdTripFlg          Trip_Flags0.bit.b1        // Short Delay Trip              *** MAY NEED TO MOVE TO A SEPARATE WORD SINCE SOME ARE CHANGED IN BACKGROUND (INTERRUPT)
#define InstTripFlg        Trip_Flags0.bit.b2        // Instantaneous Trip
#define GndTripFlg         Trip_Flags0.bit.b3        // Ground Trip
#define PhaseLossTripFlg   Trip_Flags0.bit.b4        // Current-based Phase Loss Trip
#define CurrUnbTripFlg     Trip_Flags0.bit.b5        // Current Unbalance Trip
#define UvTripFlg          Trip_Flags0.bit.b6        // UV Trip
#define OvTripFlg          Trip_Flags0.bit.b7        // OV Trip
#define VoltUnbalTripFlg   Trip_Flags0.bit.b8        // Voltage Unbalance Trip
#define SneakersTripFlg    Trip_Flags0.bit.b9        // Sneakers Trip
#define UfTripFlg          Trip_Flags0.bit.b10       // Under Frequency Trip
#define OfTripFlg          Trip_Flags0.bit.b11       // Over Frequency Trip
#define TempTripFlg        Trip_Flags0.bit.b12       // Over Temperature Trip
//                         Trip_Flags0.bit.b13
#define ComOpenFlg         Trip_Flags0.bit.b14       // Opened by communication
#define MM_TripFlg         Trip_Flags0.bit.b15       // Maintainance Mode Trip  - from Override micro

#define NeuTripFlg         Trip_Flags1.bit.b0        // Neutral Trip                   *** DAH  CHECK THESE ALSO FOR POSSIBLE FOREGROUND/BACKGROUND ISSUES
#define McrTripFlg         Trip_Flags1.bit.b1        // MCR Trip - from Override micro 
#define BellTripFlg        Trip_Flags1.bit.b2        // Bell: Set on any trip
#define TripReqFlg         Trip_Flags1.bit.b3        // Set on any trip
#define HWInstTripFlg      Trip_Flags1.bit.b4        // HW Instantaneous (Override) Trip  - from Override micro 
#define RealPwrTripFlg     Trip_Flags1.bit.b5        // Over Real Power Trip
#define ReacPwrTripFlg     Trip_Flags1.bit.b6        // Over Reactive Power Trip
#define AppPwrTripFlg      Trip_Flags1.bit.b7        // Over Apparent Power Trip
#define PFTripFlg          Trip_Flags1.bit.b8        // Under Power Factor Trip
#define RevPwrTripFlg      Trip_Flags1.bit.b9        // Reverse Power Trip
#define RevReacPwrTripFlg  Trip_Flags1.bit.b10       // Reverse Reactive Power Trip
#define RevSeqTripFlg      Trip_Flags1.bit.b11       // Reverse Sequence (Phase Rotation) Trip
#define DigBypassTripFlg   Trip_Flags1.bit.b12       // Digital Bypass Trip - from Override micro
//                         Trip_Flags1.bit.b13
//                         Trip_Flags1.bit.b14
//                         Trip_Flags1.bit.b15

#define TRIP_MASK_FLG0     0xDFFF
#define TRIP_MASK_FLG1     0x1FFF


// Trip function pickup flag definitions
#define UvTripPuFlg        TripPuFlags.bit.b0        // Undervoltage
#define OvTripPuFlg        TripPuFlags.bit.b1        // Overvoltage
#define VuTripPuFlg        TripPuFlags.bit.b2        // Voltage unbalance
#define CuTripPuFlg        TripPuFlags.bit.b3        // Current unbalance
#define RevWTripPuFlg      TripPuFlags.bit.b4        // Reverse active power
#define RevVarTripPuFlg    TripPuFlags.bit.b5        // Reverse reactive power
#define PlTripPuFlg        TripPuFlags.bit.b6        // Phase loss
#define OfTripPuFlg        TripPuFlags.bit.b7        // Overfrequency
#define UfTripPuFlg        TripPuFlags.bit.b8        // Underfrequency
#define OkWTripPuFlg       TripPuFlags.bit.b9        // Over kW
#define OkVarTripPuFlg     TripPuFlags.bit.b10       // Over kVar
#define OkVATripPuFlg      TripPuFlags.bit.b11       // Over kVA
#define UPFTripPuFlg       TripPuFlags.bit.b12       // Under PF

#define TRIP_PU_MASK       0x1FFF


//------------------------------------------------------------------------------------------------------------
//          Alarm Flags...
//------------------------------------------------------------------------------------------------------------

// Alarm Flags.
#define LdPuAlmFlg         Alarm_Flags0.bit.b0        // Long Delay Pickup Alarm
#define HlAlm1Flg          Alarm_Flags0.bit.b1        // High load Alarm 1 active flag
//                         Alarm_Flags0.bit.b2        
#define GndAlmFlg          Alarm_Flags0.bit.b3        // Ground Alarm
#define RevSeqAlmFlg       Alarm_Flags0.bit.b4        // Phase Rotation Alarm
#define CurrUnbAlmFlg      Alarm_Flags0.bit.b5        // Current Unbalance Alarm
#define UvAlmFlg           Alarm_Flags0.bit.b6        // UV Alarm
#define OvAlmFlg           Alarm_Flags0.bit.b7        // OV Alarm
#define VoltUnbalAlmFlg    Alarm_Flags0.bit.b8        // Voltage Unbalance Alarm
#define HlAlm2Flg          Alarm_Flags0.bit.b9        // High load Alarm 2 active flag
#define UfAlmFlg           Alarm_Flags0.bit.b10       // UF Alarm
#define OfAlmFlg           Alarm_Flags0.bit.b11       // OF Alarm
//                         Alarm_Flags0.bit.b12       
#define KWDmdAlmFlg        Alarm_Flags0.bit.b13       // KW Demand Alarm
#define KVADmdAlmFlg       Alarm_Flags0.bit.b14       // KVA Demand Alarm
#define GfPreAlarmFlg      Alarm_Flags0.bit.b15       // Ground Fault Pre-alarm

#define NeutAlmFlg         Alarm_Flags1.bit.b0        // Neutral Alarm
#define THDAlmFlg          Alarm_Flags1.bit.b1        // THD Alarm
#define RevPwrAlmFlg       Alarm_Flags1.bit.b2        // Reverse Power Alarm
#define MechAlmFlg         Alarm_Flags1.bit.b3        // Mechanical Alarm
#define ElectAlmFlg        Alarm_Flags1.bit.b4        // Electronic Alarm
#define RealPwrAlmFlg      Alarm_Flags1.bit.b5        // Over Real Power Alarm
#define ReacPwrAlmFlg      Alarm_Flags1.bit.b6        // Over Reactive Power Alarm
#define AppPwrAlmFlg       Alarm_Flags1.bit.b7        // Over Apparent Power Alarm
#define PFAlmFlg           Alarm_Flags1.bit.b8        // Under Power Factor Alarm
#define SneakersAlmFlg     Alarm_Flags1.bit.b9        // Sneakers Alarm
#define TempAlmFlg         Alarm_Flags1.bit.b10       // Over Temperature Alarm
#define PhaseLossAlmFlg    Alarm_Flags1.bit.b11       // Phase Loss Alarm
#define BrkHealthAlmFlg    Alarm_Flags1.bit.b12       // Breaker Health Alarm
#define THDCurrAlmFlg      Alarm_Flags1.bit.b13       // Current THD Alarm
#define THDVoltAlmFlg      Alarm_Flags1.bit.b14       // Voltage THD Alarm
#define TherMemAlmFlg      Alarm_Flags1.bit.b15       // Thermal Memory Alarm

#define LowBatAlmFlg       Alarm_Flags2.bit.b0        // Low Battery Alarm
#define LowControlVoltage  Alarm_Flags2.bit.b1        // Control Voltage below operating threshold    *** BP - same asAuxPower_LowVolt?
#define WrongSensorAlmFlg  Alarm_Flags2.bit.b2        // Wrong Neutral Sensor Alarm
#define DispTempAlmFlg     Alarm_Flags2.bit.b3        // Display Over Temperature Warning
#define RevReacPwrAlmFlg   Alarm_Flags2.bit.b4        // Reverse Reactive Power Alarm
#define DigiBypassAlmFlg   Alarm_Flags2.bit.b5        // Digital Bypass Alarm (OVR I2C comms)
#define TimeToTripFlg      Alarm_Flags2.bit.b6        //new add one  -- time to trip flag 



#define ALARM_MASK_FLG0    0xEFFB
#define ALARM_MASK_FLG1    0xFFFF
#define ALARM_MASK_FLG2    0x007F

#define TestAlm_MaskF3     0xFFD4               // Enable mask for Flags3 used to exit test mode
#define TestAlm_MaskF4     0xFFFF               // Enable mask for Flags4 used to exit test mode

#define Alarm_MaskF3       0x7FFE               // Mask from Flags3 to determine secondary status = alarm
#define Alarm_MaskF4       0xFFFF               // Mask from Flags4 to determine secondary status = alarm

//#define ElectAlmFlg_Mask   0x1EF1             // Mask for Flags0 that cause ElectAlmFlg to be set    <--(handled individually)
#define MechAlmFlg_Mask    0xF000               // Mask for Flag05 that cause MechAlmFlg to be set
#define NvmFault_Mask      0x0C30               // Mask for Flags0 setting NVMem cause code     <--(non-protection related)

                     // Auxiliary Alarm definition flags.
#define BrkrConfigFault    Flags0.bit.b0        // Bad Breaker Config.  Default Breaker Config used - (Status LED red)
#define StpFault           Flags0.bit.b1        // Bad Setpoint Image.  Default Setpoints Used - (Status LED red)
#define ETUCalFault        Flags0.bit.b2        // Bad ETU Calibration Image.  Default ETU Calibration used - (Status LED red)
#define RtcFault           Flags0.bit.b3        // Unable to recover real time  ???(In Tokyo, this is Low Battery and No Aux Power)
#define MaxMinFault        Flags0.bit.b4        // Bad max / min image.  (part of NV Memory fault)
#define EnergyFault        Flags0.bit.b5        // Bad energy image in FRAM   (part of NV Memory fault)
#define RogoCalFault       Flags0.bit.b6        // Bad Rogo Calibration Image.  Default Rogo Calibration used - (Status LED red)
#define AFE_Error          Flags0.bit.b7        // AFE error bit is set - (Status LED red)
#define WdgFault           Flags0.bit.b8        // Watchdog operation - (Status LED red)
#define OscFault           Flags0.bit.b9        // Set if we are not running at 120MHz - (Status LED red)
#define BrkHealthCfgFault  Flags0.bit.b10       // Bad Breaker Health Config. Default Breaker Health Config used  (part of NV Memory fault)
#define BrkHealthDataFault Flags0.bit.b11       // Bad Breaker Health data in FRAM   (part of NV Memory fault)
#define LowTA_Boost_Voltage Flags0.bit.b12      // USB boost is not capable of providing TA voltage to trip  <--do we have this?
#define BadTaFlg           Flags0.bit.b13       // Bad (open) trip actuator - (Status LED red)
#define FrameVerFault      Flags0.bit.b14       // Frame board version 2.x connected - (not used)
//                         Flags0.bit.b15

//------------------------------------------------------------------------------------------------------------
//          Protection Flags1
//------------------------------------------------------------------------------------------------------------
#define OpenFlg             Flags1.bit.b0       //same as 1150 -- open breaker = 1
#define InstpuFlg           Flags1.bit.b1       //same as 1150 -- instantaneous pickup flag
#define AuxPower_LowVolt    Flags1.bit.b2       //new add one  -- set when low aux-power detected there.
#define NmaxFlg             Flags1.bit.b3       //same as 1150 -- Neutral One Cycle current is highest of the 4 phases
#define LowBattery_Voltage  Flags1.bit.b4       //new add one  -- set when battery in low voltage status
#define PwrUpTrip_Check     Flags1.bit.b5       //new add one  -- set flag when finish trip checking on power up
#define Zin_Latched         Flags1.bit.b6       //same as 1150 -- latched ZIN flag
#define SdpuFlg             Flags1.bit.b7       //same as 1150 -- short delay pick up flag
#define SdintpuFlg          Flags1.bit.b8       //new add one  -- short delay interlock pick up 1 => active

#define PSdpuFlg            Flags1.bit.b9       //new add one  -- phantom short delay pick up flag
#define OpenTA_Voltage      Flags1.bit.b10      //new add one  -- set then TA drain voltage < 18.0V
#define GfpuFlg             Flags1.bit.b11      //same as 1150 -- ground fault pick up 1 => active
#define TAVoltValid         Flags1.bit.b12      // set after reading the TA Sense voltage for the first time                 Flags1.bit.b12      
#define LowTA_Voltage       Flags1.bit.b13      //same as 1150 -- set when TA drain voltage < 38.0V
#define PBldFlg             Flags1.bit.b14      // B delay flag for IEEE_IEC long delay 
#define SdponFlg            Flags1.bit.b15      //new add one  -- for short delay trip time depression under cold start

#define MM_Active           Flags2.bit.b0       // Maintenance Mode Active
#define ABC_Rotation        Flags2.bit.b1       // ABC rotation
#define ACB_Rotation        Flags2.bit.b2       // ACB rotation
#define ThermCapacityFlg    Flags2.bit.b3       //new add one  -- thermal capacity flag  
#define BatteryValid        Flags2.bit.b4       // set after reading the battery voltage for the first time  
#define AuxVoltValid        Flags2.bit.b5      // set after reading the Aux Power voltage for the first time
#define TestTrip            Flags2.bit.b8       //set by test trip initiated from USB/UART test port
#define TestNoTrip          Flags2.bit.b9       //set by test without trip initiated from USB/UART test port
#define TestLCD             Flags2.bit.b10      //set by local test initiated from LCD side to open breaker directly
#define RemoteCtrlFlag1     Flags2.bit.b11        //new add one  -- Remot Control Flag
#define RemoteCtrlFlag2     Flags2.bit.b12        //new add one  -- Remot Control Flag
#define RemoteCtrlFlag3     Flags2.bit.b13        //new add one  -- Remot Control Flag
#define T_Forbid_Coil       Flags2.bit.b14      // used to allow Coil Detection testing based on current levels
#define T_Forbid_SecInj     Flags2.bit.b15      // used to allow HW or FW Sec Injection testing based on current levels


struct FW_SIMULATED_TEST
{     
    uint32_t TestAllowedThreshold;      // Max current threshold to permit Sec Inj testing (5% * Breaker Rating) 
    uint8_t State;
    uint8_t Enable;
    uint8_t Phase;                      // which phase gets tested
    uint32_t TestCurrent;               // test current in engineering units
    uint8_t  Trip_NoTrip;               // default is 0 for Trip
    uint32_t StartingSampleCounter;  
    uint32_t EndingSampleCounter;   
    uint32_t TestTimeoutTimer;    
    uint32_t TestResultTime;            // test result in msec
    uint32_t TestResultCurrent;         // test result one-cycle current in engineering units
    uint32_t Test_result_psc;           // test result: captured trip/alarm cause

 //  uint32_t Trip_Alarm_current;      //MCU1, using for trip event log after test finished
};




struct HW_SEC_INJ_TEST           
{
    uint32_t TestAllowedThreshold;      // Max current threshold to permit Sec Inj testing (5% * rating current)
    uint8_t State;
    uint8_t Enable;
    uint8_t Phase;                      // which phase gets tested
    uint32_t TestCurrent;               // test current in engineering units
    uint8_t  Trip_NoTrip;               // default is 0 for Trip
    uint32_t StartingSampleCounter;  
    uint32_t EndingSampleCounter;   
    uint32_t TestTimeoutTimer;    
    uint32_t TestResultTime;            // test result in msec
    uint32_t TestResultCurrent;         // test result one-cycle current in engineering units
    

    uint32_t LD_Trip_StartingSampleCounter;                     // Starting Sample Counter is set when we go into pickup.
    uint8_t  LD_Trip_StartingSampleCounter_Logged;              // This allows for the time that it takes the Sec Inj signal
    uint32_t SD_Trip_StartingSampleCounter;                     // to get to its value.
    uint8_t  SD_Trip_StartingSampleCounter_Logged;
    uint32_t Inst_Trip_StartingSampleCounter;
    uint8_t  Inst_Trip_StartingSampleCounter_Logged;
    uint32_t GF_Trip_StartingSampleCounter;
    uint8_t  GF_Trip_StartingSampleCounter_Logged;
    uint32_t GF_Alarm_StartingSampleCounter;
    uint8_t  GF_Alarm_StartingSampleCounter_Logged;
    uint32_t Sneaker_Trip_StartingSampleCounter;
    uint8_t  Sneaker_Trip_StartingSampleCounter_Logged;
    uint32_t PhaseLoss_Trip_StartingSampleCounter;
    uint8_t  PhaseLoss_Trip_StartingSampleCounter_Logged;
    uint32_t CurUnbal_Trip_StartingSampleCounter;
    uint8_t  CurUnbal_Trip_StartingSampleCounter_Logged;
    
    uint8_t  OvrMicroTrip;                                      // This gets set when we get a trip (MM, Override, MCR, Digital Bypass)
                                                                // from the Override micro.  We set the TestResultTime to 0, like Tokyo.

};


struct COIL_DETECT           
{
    uint16_t TestAllowedThreshold;      // Max current threshold to permit Sec Inj testing (5% * rating current)
    uint8_t  State;
    uint8_t  Enable;
    uint16_t SampleCount;
    float    Avg_Ia;  
    float    Avg_Ib; 
    float    Avg_Ic;  
    float    Avg_In; 
    uint8_t  Result;
    uint8_t  TestInProgress;
 
};

// Breaker Configuration size in frame module FRAM
//
#define  BRKCONFIG_BLOCK_SIZE      399  // Does not include checksum and checksum complement 

//critical breaker configuration structure definition, stored in frame module FRAM
struct CRITICAL_BRKCONFIG_DEF           //stored in FRAM on frame module
{
    uint16_t Rating;                    
    uint16_t BreakerFrame;              
    uint16_t Standard;                  
    uint16_t MaxInstTripSetting;
    uint16_t Poles;
    uint16_t MCR;
    uint16_t OvrWithstand;
};
#define  CRIT_BRKCONFIG_BLOCK_SIZE_35   (sizeof(struct CRITICAL_BRKCONFIG_DEF))
union CRITICAL_BREAKER_CONFIG
{
   uint8_t                          buf[CRIT_BRKCONFIG_BLOCK_SIZE_35];
   struct CRITICAL_BRKCONFIG_DEF    config;
};

// This is for the old PXR25 critical breaker config info
#define  CRIT_BRKCONFIG_BLOCK_SIZE_25       8


//frame module structure definition, stored in frame module FRAM
//
struct FRAMEMODULECONFIG_DEF
{
    uint16_t        FrameModule_HW_Rev;
    uint32_t        FrameModule_SerialNum;

    uint16_t        Cksum;
    uint16_t        Cksum_not;
};
#define  FRAMEMODULECONFIG_BLOCK_SIZE  (sizeof(struct FRAMEMODULECONFIG_DEF))
union FRAMEMODULE_CONFIG
{
   uint8_t                              buf[FRAMEMODULECONFIG_BLOCK_SIZE];
   struct FRAMEMODULECONFIG_DEF         config;
};

//Voltage Divider Board structure definition store in Frame module FRAM
//
struct VDB_CONFIG_DEF
{
    uint16_t    VDB_HW_Rev;
    uint32_t    VDB_SerialNum;
    
    uint16_t    Cksum;
    uint16_t    Cksum_not;
};
#define VDB_CONFIG_BLOCK_SIZE   (sizeof(struct VDB_CONFIG_DEF))
union VDB_CONFIG 
{
    uint8_t                 buf[VDB_CONFIG_BLOCK_SIZE];
    struct VDB_CONFIG_DEF   config;
};


