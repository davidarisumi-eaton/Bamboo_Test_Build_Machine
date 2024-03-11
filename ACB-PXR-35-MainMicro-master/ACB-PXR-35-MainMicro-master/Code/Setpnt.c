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
//  MODULE NAME:        Setpnt.c
//
//  MECHANICS:          Program module containing the subroutines that handle setpoints communications and
//                      processing
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.60   220928  DAH - Added setpoints definitions and functions (initial code)
//                      - FRAM_Read() revised to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//   0.62   221027  DAH - Added SetpActiveSet
//                      - Revised Setp_VarInit() to read SetpActiveSet from Frame FRAM
//                      - Revised Get_Setpoints() to handle multiple setpoints sets (with SetpActiveSet)
//                      - Revised Check_Setpoints() to handle multiple setpoints sets (with SetpActiveSet)
//                      - Moved Checksum8_16() from local to global
//                      - Corrected bugs in Check_Setpoints().  Replaced FRAM_Write() with
//                        Frame_FRAM_Write().  Added check of the checksum values
//   0.64   221118  DAH - Revised SETP_GR0_DEFAULT[] (WFM_BBCapConfig changed to G0_Spare4)
//                      - Added items to SETP_GR8_DEFAULT[]
//                      - In Check_Setpoints(), added num_sp to hold the number of setpoints, instead of
//                        recalculating it each time through the loop that checks the setpoints
//                      - In Check_Setpoints(), added code to overwrite the Group1 read-only setpoints with
//                        corresponding Group0 setpoints when retrieving Group1 setpoints
//                      - Revised GEt_Setpoints() to include the setpoints set when retrieving the setpoints
//   0.69   220220  DAH - Revised Groups 0, 1, 4, 5, and 6 setpoints
//   0.71   220228  DAH - Added LD_EventAction to SETP_GR1_DEFAULT[]
//                      - Demand power times (xxxxDmdPwr_Tt) deleted from SETP_GR6_DEFAULT[]
//                      - Added Group 11 setpoints
//   0.72   220320  DAH - Fixed bug with Group 11 setpoints
//                          - Get_Setpoints() modified
//    38    230518  DAH - Corrected Overvoltage Alarm and Undervoltage Alarm defaults (2 = OFF)
//    49    230706  DAH - Added Setpoints Group 12
//    53    230725  VD  - Added setpoint range constants and Verify_Setpoints for Modbus Setpoint write
//                        support
//    54    230801  DAH - Revised SETP_GR1_DEFAULT[] to match Frame FRAM and to add SD Disturbance setting
//    58    230810  DAH - Revised Check_Setpoints() to generate an event if there is a Frame module error
//                      - Revised Get_Setpoints() to set the Group 0 active setpoints set to SetpActiveSet
//                      - Added support for setpoint set changes
//                          - Added Load_SetpGr0_Gr1() and Load_SetpGr2_LastGr()
//    98    231017  DAH - In Load_SetpGr2_LastGr(), corrected error determining good setpoints (checking
//                        SetpointsStat)
//   111    231117  BP  - Added code in Load_SetpGr0_Gr1() for Breaker Frame
//   113    231128  BP  - Setpoints1 Style2 is now writeable for Digitization
//   116    231129  MAG - Changed several values in MODB_SETP_GRP0_MIN/MAX and MODB_SETP_GRP2_MAX
//                      - Added handling of Maintenance Mode changes (hope this is the right place)
//   125    231212  BP  - Changed default setpoints for Neutral Sensor and Source Ground Sensor to CT
//   129    231213  MAG - Changed Verify_Setpoints() to pass in a pointer to the setpoints buffer
//                      - Numerous changes to handle splitting setpoints Group 10 to Groups 10 and 13
//                      - Comment changes describing various setpoints to match setpoints document
//                      - Default, Min, and Max setpoint changes to match setpoints document, eliminate bugs
//                      - Fixed typo/bug in Group 0 reference in MODB_SETP_MAX_ADDR[]
//                      - Renamed SETP_ACTIVE_SET to SETP_ACTIVE_SET_ADDR to match naming conventions
//                      - Consolidated handling of Group0/Group1 read-only setpoints into GetSetpoints()
//                      - Many changes in Verify_Setpoints to handle passed-in setpoints buffer pointer and
//                        fix numerous bugs.
//                      - Tweaked Maintenance Mode handling to match other occurrences
//   133    231219  DAH - In Load_SetpGr2_LastGr(), removed code that enters an event and added code to
//                        return a status code.  The event will be entered in the calling subroutine
//   138    240102  MAG - In Verify_Setpoints() change all Long Delay Times to 100x and swap miminimum
//                        values for Ground Fault Time dependant on Flat or I2t slope.
//   146    240124  MAG - In Verify_Setpoints() fix bugs restricting upper limit for Ground Fault Pre-Alarm,
//                        Thermal Memory Alarm, Neutral Alarm Pickup, and Freq. Protection Trip/Alarm Delays.
//                        Also added checks for number of poles and LDPU for Neutral Protection Ratio.
//                        Increased Long Delay Time default to 200 to account for 100x change in rev 138.
//   149    240131  DAH - In Verify_Setpoints() revised check of Demand Logging Interval to distinguish
//                        between fixed and sliding windows
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
#include "Setpnt_def.h"
#include "RealTime_def.h"               // Needed for Iod_def.h
#include "Events_def.h"                 // FRAM_Flash_def.h must be preceded by Events_def.h, Setpnt_def.h, 
#include "Demand_def.h"                 //   Demand_def.h, and Meter_def.h!!
#include "Meter_def.h"
#include "FRAM_Flash_def.h"
#include "Iod_def.h"
#include "Prot_def.h"
#include "Flags_def.h"

//
//      Local Definitions used in this module...
//


//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Iod_ext.h"
#include "Prot_ext.h"
#include "Events_ext.h"
#include "RealTime_ext.h"
#include "Ovrcom_ext.h"
#include "Demand_ext.h"



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Setp_VarInit(void);
uint8_t Get_Setpoints(uint8_t set, uint8_t group, uint8_t *good_copy);
void Load_SetpGr0_Gr1(void);
uint8_t Load_SetpGr2_LastGr(void);
uint16_t Checksum8_16(uint8_t *addr, uint16_t length);
void Check_Setpoints(uint8_t group);
//uint8_t Verify_Setpoints(uint8_t group);
uint8_t Verify_Setpoints(uint8_t group, uint16_t *stp_ptr);



//      Local Function Prototypes (These functions are called only within this module)
//


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
union STP_GRP0_DEF Setpoints0;
union STP_GRP1_DEF Setpoints1;
union STP_GRP2_DEF Setpoints2;
union STP_GRP3_DEF Setpoints3;
union STP_GRP4_DEF Setpoints4;
union STP_GRP5_DEF Setpoints5;
union STP_GRP6_DEF Setpoints6;
union STP_GRP7_DEF Setpoints7;
union STP_GRP8_DEF Setpoints8;
union STP_GRP9_DEF Setpoints9;
union STP_GRP10_DEF Setpoints10;
union STP_GRP11_DEF Setpoints11;
union STP_GRP12_DEF Setpoints12;
union STP_GRP13_DEF Setpoints13;
uint32_t SetpointsStat;
uint16_t SetpScratchBuf[SETP_SCRATCHBUF_SIZE/2];
uint8_t SetpChkGrp;
uint8_t SetpActiveSet;


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//


//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//


//------------------------------------------------------------------------------------------------------------
// Setpoints Default Values
//------------------------------------------------------------------------------------------------------------

const uint16_t SETP_GR0_DEFAULT[] =
{
  1600,                                 // Rating (1600A) (read-only) Copied from Config at starup
  0,                                    // Breakframe (???) (read-only) Copied from Config at starup
  0x21,                                 // Style (0x20 - no BT, 0x21 - with BT)  Read from Group 1 at startup
  0x00,                                 // Style_2 (digitization feature enable bitfield) Read from Group 1 at startup
  0x00,                                 // MM (disabled)
  1,                                    // MaintLevel (2.5*In)
  60,                                   // Freq (60Hz)
  0,                                    // Power Feed (forward/top)
  0,                                    // SignConv (???)
  0,                                    // Demand Window (fixed) used by display code for both power and current
  15,                                   // Demand Interval (15 minute) used by display code for both power and current
  0,                                    // Language (English)
  0,                                    // LCD_Rotation (unused)
  0,                                    // Relay_Config1 (unused, moved to group 12)
  0,                                    // Relay_Config2 (unused, moved to group 12)
  0,                                    // Relay_Config3 (unused, moved to group 12)
  0,                                    // Pole_A_Location (unused)
  0,                                    // Was CurrentWindow in PXR25 (unused)
  0,                                    // Was CurrentInterval in PXR25 (unused)
  0,                                    // BrkHealth_Level (0)
  480,                                  // SystemVoltage (480)
  1,                                    // Neutral_Sensor (CT)
  801,                                  // SG_Sensor (CT 400:1) Read from Group 1 at startup

  1,                                    // SetSelectMethod (Comms)
  0,                                    // ActiveSet (Read-only)
  0,                                    // PTModule2 (None)
  0,                                    // PhaseLabel (ABCN)
  10,                                   // WFM_TripPreCyc (10)
  10,                                   // WFM_AlarmPreCyc (10)
  10,                                   // WFM_ExtCapPreCyc(10)
  0,                                    // IEC61850_Config (Disabled)
  0,                                    // DemandLogInterval (Off)
  0,                                    // Voltage Transformer Configuration (wye)
  2,                                    // G0_Spare2 (unused)
  3                                     // G0_Spare3 (unused)
};


const uint16_t SETP_GR1_DEFAULT[] =
{
  1600,                                 // Rating (1600A) (read-only) Copied from Config at starup
  0,                                    // Breakframe (???) (read-only) Copied from Config at starup
  0x21,                                 // Style (0x20 - no BT, 0x21 - with BT) (read-only)
  0x00,                                 // Style_2 (digitization feature enable bitfield) (read-only)
  0,                                    // Thermal Memory (enabled)
  0,                                    // ZSI (disabled)
  2,                                    // Ld_Slp (I2t)
  40,                                   // Ld_Pu (0.4, 10x)
  200,                                  // LD_Tt (2sec, 100x)
  50,                                   // HL_Alarm_1 (50)
  0,                                    // Sd_Slp (flat)
  20,                                   // Sd_Pu (2.0 * Ir)
  5,                                    // Sd_Tt (50msec)
  40,                                   // Inst_Pu (4 * In)
  0,                                    // Gnd_Type (residual)
  0,                                    // Gf_T_A_OFF (Trip)
  0,                                    // Gf_Slp (flat)
  20,                                   // Gf_Pu (0.20)
  30,                                   // Gf_Tt (300msec)
  1,                                    // GF_ThermalMem (enabled)
  100,                                  // NeutRatio (100%)
  120,                                  // HL_Alarm_2 (120)
  50,                                   // GF_Pre_Alarm (50%)
  0,                                    // ThermMem_Alarm (Off)

  30,                                   // HL1_Time (30sec)
  0,                                    // GF_ZSI (disabled)
  0,                                    // NeutAlmPU (off)
  30,                                   // NeutAlmTime (30sec)

  30,                                   // HL2_Time (30sec)
  2,                                    // HL1_Action (OFF)
  2,                                    // HL2_Action (OFF)
  0,                                    // LD_EventAction (no events on LDPU)
  801,                                  // SG_Sensor (CT 400:1)
  0,                                    // SD_EventAction (no events on SDPU)
  12,                                   // G1_Spare2 (unused)
  13                                    // G1_Spare3 (unused)
};


const uint16_t SETP_GR2_DEFAULT[] =
{
  7,                                    // RTU device address (7)
  2,                                    // RTU baud rate (9600)
  0,                                    // RTU parity (Even)
  1,                                    // RTU stop bits (1)
  0,                                    // RTU inv obj handling (return NaN or 0)
  1,                                    // RTU float word order (low-order word is first)
  1,                                    // RTU fixed word order (low-order word is first)
  0,                                    // RTU Enable (disabled)
  0,                                    // On-board Modbus TCP invalid object handling (return NaN or 0)
  1,                                    // On-board Modbus TCP float word order (low-order word is first)
  1,                                    // On-board Modbus TCP fixed word order
  311,                                  // On-board Modbus TCP comms loss timeout (*** DAH TEST VALUE)
  412,                                  // On-board Modbus TCP IP filter enable (*** DAH TEST VALUE)
  513                                   // On-board Modbus TCP TCP enable/disable (*** DAH TEST VALUE)
};


const uint16_t SETP_GR3_DEFAULT[] =
{
  // these setpoints are not used for initial release
  3,                                    // CAM device address (*** DAH TEST VALUE)
  13,                                   // CAM baud rate (*** DAH TEST VALUE)
  23,                                   // CAM parity (*** DAH TEST VALUE)
  33,                                   // CAM stop bits (*** DAH TEST VALUE)
  43,                                   // CAM INCOM address  (*** DAH TEST VALUE)
  53,                                   // CAM INCOM baud rate (*** DAH TEST VALUE)
  63,                                   // CAM Ethernet DHCP enable (*** DAH TEST VALUE)
  73,                                   // CAM Ethernet IP Address 0 (*** DAH TEST VALUE)
  83,                                   // CAM Ethernet IP Address 1 (*** DAH TEST VALUE)
  93,                                   // CAM Ethernet IP Address 2 (*** DAH TEST VALUE)
  103,                                  // CAM Ethernet IP Address 3 (*** DAH TEST VALUE)
  113,                                  // CAM Ethernet submask (*** DAH TEST VALUE)
  123,                                  // CAM Ethernet DF gateway 0 (*** DAH TEST VALUE)
  133,                                  // CAM Ethernet DF gateway 1 (*** DAH TEST VALUE)
  143,                                  // CAM Ethernet reset pin (*** DAH TEST VALUE)
  153                                   // CAM Profibus address (*** DAH TEST VALUE)
};


const uint16_t SETP_GR4_DEFAULT[] =
{
  0,                                    // Frequency protection enabled (disabled)
  0,                                    // OF Trip protection action (Off)
  1050,                                 // OF Trip protection pickup (105%)
  100,                                  // OF Trip protection time (1sec)
  0,                                    // UF Trip protection action (Off)
  950,                                  // UF Trip protection pickup (95%)
  100,                                  // UF Trip protection time (1sec)
  0,                                    // OF Alarm protection action (Off)
  1050,                                 // OF Alarm protection pickup (105%)
  6000,                                 // OF Alarm protection time (60sec)
  0,                                    // UF Alarm protection action (Off)
  900,                                  // UF Alarm protection pickup (90%)
  6000,                                 // UF Alarm protection time (60sec)
  189,                                  // G4_Spare1 (*** DAH TEST VALUE)       
  190,                                  // G4_Spare2 (*** DAH TEST VALUE)       
  191                                   // G4_Spare3 (*** DAH TEST VALUE)       
};


const uint16_t SETP_GR5_DEFAULT[] =
{
  0,                                    // Over Voltage Trip feature (off)
  1200,                                 // Over Voltage Trip Pickup (120% of nominal)
  6000,                                 // Over Voltage Trip Time (60sec)
  0,                                    // Under Voltage Trip feature (off)
  900,                                  // Under Voltage Trip Pickup (90% of nominal)
  6000,                                 // Under Voltage Trip Time (60sec)
  0,                                    // Voltage Unbalance Trip feature (off)
  50,                                   // Voltage Unbalance Trip Pickup (50% max phase voltage)
  3000,                                 // Voltage Unbalance Trip Time (30sec)
  0,                                    // Current Unbalance Trip feature (off)
  5,                                    // Current Unbalance Trip Pickup (5% max phase current)
  1000,                                 // Current Unbalance Trip Time (10sec)
  2,                                    // Reverse Power feature (off)
  1200,                                 // Reverse Power Pickup (1200kW)
  3000,                                 // Reverse Power Time (30sec)
  0,                                    // Sensing (ABC)
  2,                                    // Phase Reverse feature (off)
  2,                                    // Phase Loss feature (off)
  1,                                    // Phase Loss Time (1sec)

  195,                                  // Reserved_1
  196,                                  // Reserved_2
  197,                                  // Reserved_3
  198,                                  // Reserved_4
  199,                                  // Reserved_5

  2,                                    // Over Voltage Alarm feature (off)
  1500,                                 // Over Voltage Alarm Pickup (150% of nominal)
  6000,                                 // Over Voltage Alarm Time (60 sec)
  0,                                    // Extended protection settings enabled/disabled (disabled)
  2,                                    // Reverse Reactive Power feature (off)
  1000,                                 // Reverse Reactive Power Pickup (1000kVar)
  3000,                                 // Reverse Reactive Power Time (30sec)                                        
  6000,                                 // Phase Rotation Time (60sec)
  1,                                    // OV_Phase_Num (1)
  1,                                    // UV_Phase_Num (1)  
  2,                                    // Under Voltage Alarm feature (off)
  500,                                  // Under Voltage Alarm Pickup (50% of nominal)
  6000,                                 // Under Voltage Alarm Time (60sec)
  0,                                    // Voltage Unbalance Alarm feature (off)
  50,                                   // Voltage Unbalance Alarm Pickup (50% max phase voltage)
  3000,                                 // Voltage Unbalance Alarm Time (30sec)
  0,                                    // Current Unbalance Alarm feature (off)
  90,                                   // Current Unbalance Alarm Pickup (90% max phase current)
  1000,                                 // Current Unbalance Alarm Time (100sec)
  223,                                  // G5_Spare1
  224                                   // G5_Spare2
};


const uint16_t SETP_GR6_DEFAULT[] =
{
  0,                                    // Power Protection Enable/Disable (Disabled)
  2,                                    // Real Power feature (Off)
  1000,                                 // Real Power Pickup (1000kW)
  30,                                   // Real Power Time (30sec)
  2,                                    // Reactive Power feature (Off)
  1000,                                 // Reactive Power Pickup (1000kW)
  30,                                   // Reactive Power Time (30sec)
  2,                                    // Apparent Power feature (Off)
  1000,                                 // Apparent Power Pickup (1000kW)
  30,                                   // Apparent Power Time (30sec)
  2,                                    // Under Power Factor feature (Off)
  95,                                   // Under Power Factor Pickup (.95)
  30,                                   // Under Power Factor Time (30sec)
  0,                                    // Real Demand Power feature (Off)
  1000,                                 // Real Demand Power Pickup (1000kW)
  0,                                    // Reactive Demand Power feature (Off)
  1000,                                 // Reactive Demand Power Pickup (1000kVar)
  0,                                    // Apparent Demand Power feature (Off)
  1000,                                 // Apparent Demand Power Pickup (1000kVA)
  190,                                  // G6_Spare1
  191,                                  // G6_Spare2
  192                                   // G6_Spare3
};


const uint16_t SETP_GR7_DEFAULT[] =
{
  0,                                    // Sync Check Enable/Disable (Disabled)
  65,                                   // SyncMinLiveVolt1 (3% of nominal)
  3,                                    // SyncMaxDeadVolt1 (65% of nominal)
  65,                                   // SyncMinLiveVolt2 (3% of nominal)
  3,                                    // SyncMaxDeadVolt2 (65% of nominal)
  5,                                    // SyncMaxVoltDiff (5% of nominal)
  20,                                   // SyncMaxSlipFreq (0.2Hz) 
  10,                                   // SyncMaxAngleDiff (10degrees) 
  0,                                    // SyncDeadV1DeadV2_Enable/Disable (Disabled)
  0,                                    // SyncDeadV1LiveV2_Enable/Disable (Disabled)
  0,                                    // SyncLiveV1DeadV2_Enable/Disable (Disabled)
  50,                                   // SyncChkDead_Time (50sec)
  431,                                  // G7_Spare1
  432,                                  // G7_Spare2
  433                                   // G7_Spare3
};


const uint16_t SETP_GR8_DEFAULT[] =
{
  0,                                    // ATS_Func_Enable/Disabled (0 = Disabled)
  1,                                    // Number of Generators (1)
  0,                                    // Phase Sequence (0 = ABC)
  0,                                    // Manual Retransfer: (0 = Disabled/Auto)
  0,                                    // Commit to Transfer (0 = Disabled/No)
  0,                                    // Closed Transition Enable (0 = Disabled)
  2,                                    // Closed Transition Voltage Difference (2% of nominal)
  3,                                    // Closed Transition Frequency difference (0.3Hz)
  0,                                    // InPhase Transition Enable (Disabled)
  10,                                   // In-phase Transition Frequency difference (1Hz)
  5,                                    // In-phase/Closed Synchronication timer (5min)
  0,                                    // Load Decay Threshold (0 = Off)
  0,                                    // Open Transition - Time delay, neutral (0sec)
  0,                                    // Time delay, normal -> emergency (0sec)
  300,                                  // Time delay, emergency -> normal (300sec)
  1,                                    // Time delay, pre-transfer (1sec)
  10,                                   // Time delay, post-transfer (10sec)
  3,                                    // Time delay, engine start 2 (3sec)
  3,                                    // Time delay, engine start 1 (3sec))
  300,                                  // Time delay, engine cooldown (300sec)
  6,                                    // Time delay, emergency fail (6sec)
  0xFFFF,                               // Time delay, emergency disconnect (Off)
  45,                                   // Time delay, emergency reconnect (45sec)
  20,                                   // Time delay, normal fail (20sec)
  0xFFFF,                               // Time delay, normal disconnect (Off)
  45,                                   // Time delay, normal reconnect (45sec)
  30,                                   // Engine (ATS) test duration (30min)
  115,                                  // OV dropout (115% of nominal)
  110,                                  // OV pickup (110% of nominal)
  80,                                   // UV dropout (80% of nominal)
  90,                                   // UV pickup (90% of nominal)
  106,                                  // OF dropout (106% of nominal)
  104,                                  // OF pickup (104% of nominal)
  94,                                   // UF dropout (94% of nominal)
  96,                                   // UF pickup (96% of nominal)
  12,                                   // Voltage unbal dropout (12% of nominal)
  10,                                   // Voltage unbal pickup (10% of nominal)
  30,                                   // Voltage unbal Delay (30sec)
  552,                                  // G8_Spare1
  553,                                  // G8_Spare2
  554,                                  // G8_Spare3
  555,                                  // G8_Spare4
  556,                                  // G8_Spare5
  557,                                  // G8_Spare6
  558,                                  // G8_Spare7
  559,                                  // G8_Spare8
  560,                                  // G8_Spare9
  561,                                  // G8_Spare10
  562,                                  // G8_Spare11
  563,                                  // G8_Spare12
  564,                                  // G8_Spare13
  565,                                  // G8_Spare14
  566                                   // G8_Spare15
};


const uint16_t SETP_GR9_DEFAULT[] =
{
  0,                                    // Over Temperature Alarm (Disabled)
  85 ,                                  // Over Temperature Alarm Pickup (85 Degrees C)
  0,                                    // Current THD Alarm Pickup (Off)
  1,                                    // Current THD Alarm Time (1 second) 
  0,                                    // Voltage THD Alarm Pickup (Off)
  1,                                    // Voltage THD Alarm Time (1 second) 
  0,                                    // Breaker Operations Count Alarm (Off)
  0,                                    // Health Alarm Pickup (Off)
  0,                                    // Sneakers Alarm (Disabled)
  0,                                    // Internal Alarm (Disabled)
  673,                                  // G9_Spare1
  674,                                  // G9_Spare2
  675,                                  // G9_Spare3
  676,                                  // G9_Spare4
  677                                   // G9_Spare5
};


const uint16_t SETP_GR10_DEFAULT[] =
{
  1,                                    // GC_DeviceName (1)
  0,                                    // Downstream Breaker 1 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 2 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 3 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 4 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 5 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 6 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 7 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 8 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 9 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 10 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 11 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 12 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 13 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 14 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 15 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 16 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 17 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 18 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 19 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 20 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 21 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 22 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 23 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 24 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 25 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 26 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 27 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 28 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 29 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 30 for ZSI function (0 =  Unassigned)
  0,                                    // Downstream Breaker 31 for ZSI function (0 =  Unassigned)
  0,                                    // Trip Fail Protection Enable/Disable (Disable)
  0,                                    // Downstream Breaker 1 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 2 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 3 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 4 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 5 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 6 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 7 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 8 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 9 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 10 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 11 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 12 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 13 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 14 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 15 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 16 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 17 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 18 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 19 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 20 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 21 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 22 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 23 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 24 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 25 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 26 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 27 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 28 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 29 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 30 for Trip Failure function (0 =  Unassigned)
  0,                                    // Downstream Breaker 31 for Trip Failure function (0 =  Unassigned)
  0,                                    // Setpoints Set Control Enable/Disable (Disabled)
  0,                                    // Setpoints Set Controller Device 1 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 2 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 3 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 4 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 5 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 6 (0 =  Unassigned)
  0,                                    // Setpoints Set Controller Device 7 (0 =  Unassigned)
  0,                                    // Transmit Metered Values Enable/Disable (Disabled)
  50,                                   // GC_CurrentDeadband (50%)
  50,                                   // GC_VoltageDeadband (50%)
  50,                                   // GC_RealPowerDeadband (50%)
  50,                                   // GC_ReacPowerDeadband (50%)
  50,                                   // GC_AppPowerDeadband (50%)
  50,                                   // GC_FreqDeadband (50%)
  50,                                   // GC_PFDeadband (50%)
  512,                                  // G10_Spare1
  513,                                  // G10_Spare2
  514,                                  // G10_Spare3
  515,                                  // G10_Spare4
  516                                   // G10_Spare5
};


const uint16_t SETP_GR11_DEFAULT[] =
{
  4,                                    // CAM device address (*** DAH TEST VALUE)
  14,                                   // CAM baud rate (*** DAH TEST VALUE)
  24,                                   // CAM parity (*** DAH TEST VALUE)
  34,                                   // CAM stop bits (*** DAH TEST VALUE)
  44,                                   // CAM INCOM address  (*** DAH TEST VALUE)
  54,                                   // CAM INCOM baud rate (*** DAH TEST VALUE)
  64,                                   // CAM Ethernet DHCP enable (*** DAH TEST VALUE)
  74,                                   // CAM Ethernet IP Address 0 (*** DAH TEST VALUE)
  84,                                   // CAM Ethernet IP Address 1 (*** DAH TEST VALUE)
  94,                                   // CAM Ethernet IP Address 2 (*** DAH TEST VALUE)
  104,                                  // CAM Ethernet IP Address 3 (*** DAH TEST VALUE)
  114,                                  // CAM Ethernet submask (*** DAH TEST VALUE)
  124,                                  // CAM Ethernet DF gateway 0 (*** DAH TEST VALUE)
  134,                                  // CAM Ethernet DF gateway 1 (*** DAH TEST VALUE)
  144,                                  // CAM Ethernet reset pin (*** DAH TEST VALUE)
  154                                   // CAM Profibus address (*** DAH TEST VALUE)
};


const uint16_t SETP_GR12_DEFAULT[] =
{
  0,                                    // Relay 1 configuration bits (Off)
  0,
  0,
  0,
  0,                                    // Relay 2 configuration bits (Off)
  0,
  0,
  0,
  0,                                    // Relay 3 configuration bits (Off)
  0,
  0,
  0
};


const uint16_t SETP_GR13_DEFAULT[] =
{
  0,                                    // Global Capture Enable/Disable (Disabled)
  0,                                    // Global Capture Command Breaker 1 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 2 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 3 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 4 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 5 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 6 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 7 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 8 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 9 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 10 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 11 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 12 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 13 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 14 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 15 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 16 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 17 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 18 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 19 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 20 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 21 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 22 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 23 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 24 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 25 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 26 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 27 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 28 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 29 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 30 (0 =  Unassigned)
  0,                                    // Global Capture Command Breaker 31 (0 =  Unassigned)
  0,                                    // Transfer Application (none)
  0,                                    // Goose Transfer Breaker Type (Preferred/Tie/Preferred)
  0,                                    // Transfer Breaker 1 (0 =  Unassigned)
  0,                                    // Transfer Breaker 2 (0 =  Unassigned)
  0,                                    // Transfer Breaker 3 (0 =  Unassigned)
  0,                                    // Transfer Breaker 4 (0 =  Unassigned)
  0,                                    // Transfer Breaker 5 (0 =  Unassigned)
  0,                                    // Transfer Breaker 6 (0 =  Unassigned)
  0,                                    // Transfer Breaker 7 (0 =  Unassigned)
  0,                                    // Transfer Breaker 8 (0 =  Unassigned)
  612,                                  // G13_Spare1
  613,                                  // G13_Spare2
  614,                                  // G13_Spare3
  615,                                  // G13_Spare4
  616                                   // G13_Spare5
};


// The following arrays provide Minimum and Maximum setpoint validation thresholds.
// 0xFFFF is reserved for setpoints that are read-only (typically factory set) or do
// not need to be checked (typically unused). Both should ignore whatever is in the 
// setpoint change buffer and retain the current setpoint value.

// 0xFFFX is used for setpoints that don't have a simple min, max, step size of 1
// or have some other special requirements, such as "Factory Setting Only".
// 0xFFFFx (x=0-F) must match in MIN and MAX arrays for a given setpoint.
// Setpoint index in the arrays will match up with the order of setpoints in Setpnt_def.h

const uint16_t MODB_SETP_GRP0_MIN[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFE, 0, 0xFFFD, 0,
  0, 0, 0xFFFC, 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0, 90, 0, 0xFFFF, 0,
  0xFFFF, 0xFFFF, 0, 8, 8, 8, 0, 0xFFFB,
  0, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP0_MAX[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFE, 5, 0xFFFD, 1,
  2, 1, 0xFFFC, 5, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 100, 690, 1, 0xFFFF, 1,
  0xFFFF, 0xFFFF, 1, 28, 28, 28, 2, 0xFFFB,
  1, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP1_MIN[] = 
{
  0xFFFF, 0xFFFF, 0xFFFE, 0xFFFD, 0, 0, 0, 40,
  0xFFFC, 0xFFFB, 0, 15, 5, 0xFFFA, 0, 0,
  0, 20, 0xFFF9, 0, 0xFFF8, 0xFFF7, 0xFFF6, 0xFFF5,
  1, 0, 0xFFF4, 1, 1, 0, 0, 0,       
  0xFFF3, 0, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP1_MAX[] = 
{
  0xFFFF, 0xFFFF, 0xFFFE, 0xFFFD, 1, 3, 9, 100,
  0xFFFC, 0xFFFB, 1, 140, 50, 0xFFFA, 1, 2,
  1, 100, 0xFFF9, 1, 0xFFF8, 0xFFF7, 0xFFF6, 0xFFF5,
  60, 3, 0xFFF4, 60, 60, 2, 2, 1,
  0xFFF3, 1, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP2_MIN[] = 
{
  1, 0, 0, 1, 0, 0, 0, 0,
  0, 0, 0, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP2_MAX[] = 
{
  247, 4, 2, 2, 1, 1, 2, 1,
  1, 1, 1, 0xFFFF, 0xFFFF, 0xFFFF
};

// Group 3 - CAM 1 Setpoints are not currently used
const uint16_t MODB_SETP_GRP3_MIN[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP3_MAX[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP4_MIN[] = 
{
  0, 0, 1001, 0xFFFE, 0, 900, 0xFFFE, 0,
  1001, 0xFFFE, 0, 900, 0xFFFE, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP4_MAX[] = 
{
  1, 1, 1100, 0xFFFE, 1, 999, 0xFFFE, 1,
  1100, 0xFFFE, 1, 999, 0xFFFE, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP5_MIN[] = 
{
  0, 1020, 0xFFFE, 0, 500, 0xFFFE, 0, 2,
  0xFFFD, 0, 2, 0xFFFD, 0, 1, 0xFFFD, 0,
  0, 0, 1, 0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
  0, 1020, 0xFFFE, 0, 0, 1, 0xFFFD, 0xFFFE,
  1, 1, 0, 500, 0xFFFE, 0, 2, 0xFFFD,
  0, 2, 0xFFFD, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP5_MAX[] = 
{
  1, 1500, 0xFFFE, 1, 980, 0xFFFE, 1, 90,
  0xFFFD, 1, 90, 0xFFFD, 2, 65500, 0xFFFD, 1,
  2, 2, 240,0xFFFF,0xFFFF,0xFFFF,0xFFFF,0xFFFF,
  2, 1500, 0xFFFE, 1, 2, 65500, 0xFFFD, 0xFFFE,
  3, 3, 2, 980, 0xFFFE, 1, 90, 0xFFFD,
  1, 90, 0xFFFD, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP6_MIN[] = 
{
  0, 0, 1, 1, 0, 1, 1, 0,
  1, 1, 0, 0xFFFE, 1, 0, 1, 0,
  1, 0, 1, 0xFFFF, 0xFFFF, 0xFFFF 
};

const uint16_t MODB_SETP_GRP6_MAX[] = 
{
  1, 2, 65500, 300, 2, 65500, 300, 2,
  65500, 300, 2, 0xFFFE, 300, 1, 65500, 1,
  65500, 1, 65500, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP7_MIN[] = 
{
  0, 10, 1, 10, 1, 1, 1, 1,
  0, 0, 0, 0, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP7_MAX[] = 
{
  1, 130, 100, 130, 100, 5, 200, 60,
  1, 1, 1, 300, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP8_MIN[] = 
{
  0, 0, 0, 0, 0, 0, 1, 0,
  0, 0, 1, 0xFFFE, 0, 0, 0, 1, 
  1, 0, 0, 0, 0, 0xFFFD, 0, 0,
  0xFFFD, 0, 0, 0xFFFC, 0xFFFB, 70, 0xFFFA, 0xFFF9,
  0xFFF8, 0xFFF7, 0xFFF6, 0xFFF5, 0xFFF4, 0, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP8_MAX[] = 
{
  1, 2, 1, 1, 1, 2, 5, 30,
  2, 30, 60, 0xFFFE, 600, 9999, 9999, 600,
  600, 120, 120, 9999, 60, 0xFFFD, 60, 120,
  0xFFFD, 60, 600, 0xFFFC, 0xFFFB, 97, 0xFFFA, 0xFFF9,
  0xFFF8, 0xFFF7, 0xFFF6, 0xFFF5, 0xFFF4, 30, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP9_MIN[] = 
{
  0, 0xFFFE, 0xFFFD, 1, 0xFFFD, 1, 0xFFFF, 0,
  0, 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP9_MAX[] = 
{
  1, 0xFFFE, 0xFFFD, 60, 0xFFFD, 60, 0xFFFF, 50,
  1, 1, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP10_MIN[] = 
{
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 10, 10, 10, 10, 10, 10, 10,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP10_MAX[] = 
{
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  1, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  1, 511, 511, 511, 511, 511, 511, 511,
  1, 50, 50, 50, 50, 50, 50, 50,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};
  
// Group 11 - CAM 2 Setpoints are not currently used
const uint16_t MODB_SETP_GRP11_MIN[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP11_MAX[] = 
{
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,
  0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

// Group 12 - Relay Configuration Setpoints require special rules, no MIN/MAX required.
const uint16_t MODB_SETP_GRP12_MIN[] = 
{
  0 // placeholder for MODB_SETP_MIN_ADDR
};
  
const uint16_t MODB_SETP_GRP12_MAX[] = 
{
  0 // placeholder for MODB_SETP_MAX_ADDR
};

const uint16_t MODB_SETP_GRP13_MIN[] = 
{
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};

const uint16_t MODB_SETP_GRP13_MAX[] = 
{
  1, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  511, 511, 511, 511, 511, 511, 511, 511,
  4, 7, 511, 511, 511, 511, 511, 511,
  511, 511, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF
};


const uint16_t* const MODB_SETP_MIN_ADDR[] =
{
  &MODB_SETP_GRP0_MIN[0], &MODB_SETP_GRP1_MIN[0], &MODB_SETP_GRP2_MIN[0], 
  &MODB_SETP_GRP3_MIN[0], &MODB_SETP_GRP4_MIN[0], &MODB_SETP_GRP5_MIN[0], 
  &MODB_SETP_GRP6_MIN[0], &MODB_SETP_GRP7_MIN[0], &MODB_SETP_GRP8_MIN[0], 
  &MODB_SETP_GRP9_MIN[0], &MODB_SETP_GRP10_MIN[0], &MODB_SETP_GRP11_MIN[0],
  &MODB_SETP_GRP12_MIN[0], &MODB_SETP_GRP13_MIN[0]
};

const uint16_t* const MODB_SETP_MAX_ADDR[] =
{
  &MODB_SETP_GRP0_MAX[0], &MODB_SETP_GRP1_MAX[0], &MODB_SETP_GRP2_MAX[0], 
  &MODB_SETP_GRP3_MAX[0], &MODB_SETP_GRP4_MAX[0], &MODB_SETP_GRP5_MAX[0], 
  &MODB_SETP_GRP6_MAX[0], &MODB_SETP_GRP7_MAX[0], &MODB_SETP_GRP8_MAX[0], 
  &MODB_SETP_GRP9_MAX[0], &MODB_SETP_GRP10_MAX[0], &MODB_SETP_GRP11_MAX[0],
  &MODB_SETP_GRP12_MAX[0], &MODB_SETP_GRP13_MAX[0]
};

const uint32_t SETP_GR_FRAM_ADDR[] = 
{
  SETP_SETA_GR0_COPY1_ADDR,  SETP_SETA_GR0_COPY2_ADDR,  SETP_SETA_GR1_COPY1_ADDR,  SETP_SETA_GR1_COPY2_ADDR,
  SETP_SETA_GR2_COPY1_ADDR,  SETP_SETA_GR2_COPY2_ADDR,  SETP_SETA_GR3_COPY1_ADDR,  SETP_SETA_GR3_COPY2_ADDR,
  SETP_SETA_GR4_COPY1_ADDR,  SETP_SETA_GR4_COPY2_ADDR,  SETP_SETA_GR5_COPY1_ADDR,  SETP_SETA_GR5_COPY2_ADDR,
  SETP_SETA_GR6_COPY1_ADDR,  SETP_SETA_GR6_COPY2_ADDR,  SETP_SETA_GR7_COPY1_ADDR,  SETP_SETA_GR7_COPY2_ADDR,
  SETP_SETA_GR8_COPY1_ADDR,  SETP_SETA_GR8_COPY2_ADDR,  SETP_SETA_GR9_COPY1_ADDR,  SETP_SETA_GR9_COPY2_ADDR,
  SETP_SETA_GR10_COPY1_ADDR, SETP_SETA_GR10_COPY2_ADDR, SETP_SETA_GR11_COPY1_ADDR, SETP_SETA_GR11_COPY2_ADDR,
  SETP_SETA_GR12_COPY1_ADDR, SETP_SETA_GR12_COPY2_ADDR, SETP_SETA_GR13_COPY1_ADDR, SETP_SETA_GR13_COPY2_ADDR
};

const uint16_t SETP_GR_SIZE[] = 
{
  STP_GRP0_SIZE, STP_GRP1_SIZE, STP_GRP2_SIZE, STP_GRP3_SIZE,  STP_GRP4_SIZE,  STP_GRP5_SIZE, STP_GRP6_SIZE,
  STP_GRP7_SIZE, STP_GRP8_SIZE, STP_GRP9_SIZE, STP_GRP10_SIZE, STP_GRP11_SIZE, STP_GRP12_SIZE, STP_GRP13_SIZE
};

uint16_t * const SETP_GR_DATA_ADDR[] =
{
  &Setpoints0.stp.Rating,            &Setpoints1.stp.Rating,           &Setpoints2.stp.Modbus_Port_Addr,
  &Setpoints3.stp.CAM_Modbus_Addr,   &Setpoints4.stp.Freq_Prot_Enable, &Setpoints5.stp.OverVolt_T_OFF,
  &Setpoints6.stp.Power_Prot_Enable, &Setpoints7.stp.Sync_Chk_Enable,  &Setpoints8.stp.ATS_Func_Enable,
  &Setpoints9.stp.OverTempAlm,       &Setpoints10.stp.GC_DeviceName,   &Setpoints11.stp.CAM_Modbus_Addr,
  &Setpoints12.stp.Relay1_Cfg1, &Setpoints13.stp.GC_GlobalCapEnable
};

const uint16_t * const SETP_GR_DEFAULT_ADDR[] =
{  
  &SETP_GR0_DEFAULT[0], &SETP_GR1_DEFAULT[0], &SETP_GR2_DEFAULT[0],  &SETP_GR3_DEFAULT[0],
  &SETP_GR4_DEFAULT[0], &SETP_GR5_DEFAULT[0], &SETP_GR6_DEFAULT[0],  &SETP_GR7_DEFAULT[0],
  &SETP_GR8_DEFAULT[0], &SETP_GR9_DEFAULT[0], &SETP_GR10_DEFAULT[0], &SETP_GR11_DEFAULT[0],
  &SETP_GR12_DEFAULT[0], &SETP_GR13_DEFAULT[0]
};

const uint32_t SETP_GR_PXR25_FRAM_ADDR[] = 
{
  FADDR_STP_0_CPY_1, FADDR_STP_0_CPY_2, FADDR_STP_0_CPY_3,
  FADDR_STP_1_CPY_1, FADDR_STP_1_CPY_2, FADDR_STP_1_CPY_3,
  FADDR_STP_1_CPY_1, FADDR_STP_1_CPY_2, FADDR_STP_1_CPY_3,      // Groups 2 through 4 are not supported by
  FADDR_STP_1_CPY_1, FADDR_STP_1_CPY_2, FADDR_STP_1_CPY_3,      //   PXR25 Frame Module, so these are just
  FADDR_STP_1_CPY_1, FADDR_STP_1_CPY_2, FADDR_STP_1_CPY_3,      //   dummy variables so that Group 5 is in
  FADDR_STP_5_CPY_1, FADDR_STP_5_CPY_2, FADDR_STP_5_CPY_3       //   the proper index
};

const uint16_t SETP_GR_PXR25_SIZE[] =                           // See comment directly above
{
  (STP_0_FRAM_SIZE - 4), (STP_1_FRAM_SIZE - 4), (STP_1_FRAM_SIZE - 4), (STP_1_FRAM_SIZE - 4),
  (STP_1_FRAM_SIZE - 4), (STP_5_FRAM_SIZE - 4)
};





//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Setp_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Setpoint Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Setpoints Module.
//                      The following variables do not need initialization.  They are initialized in the
//                      initialization code in main
//                          Setpoints0
//                          Setpoints1
//                          SetpointsStat
//                          SetpScratchBuf[]
//
//  CAVEATS:            Call only during initialization
//
//  INPUTS:             None
//
//  OUTPUTS:            SetpChkGrp, SetpActiveSet
//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Setp_VarInit(void)
{
  uint8_t temp[2];

  SetpChkGrp = 0;

  // Read active set from Frame FRAM.  If it is invalid, set the number to an invalid value (255)
  //   This will cause default settings to be loaded
  Frame_FRAM_Read(SETP_ACTIVE_SET_ADDR, 2, &temp[0]);
  if ( ((temp[0] ^ temp[1]) == 0xFF) && (temp[0] < 4) )
  {
    SetpActiveSet = temp[0];
  }
  else
  {
//    SetpActiveSet = 0xFF;
    SetpActiveSet = 0x00;  // *** DAH  JUST SET TO ZERO FOR NOW IF INVALID SINCE DON'T HAVE WRITE MECHANISM IN YET
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Setp_VarInit()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Get_Setpoints()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reads setpoints group from Frame FRAM
//
//  MECHANICS:          This subroutine retrieves a setpoints group from Frame FRAM.  The setpoints group is
//                      a calling parameter and the setpoints are stored in SetpScratchBuf[].
//                      The Frame may be configured as a PXR35 or a PXR25.
//                      For a PXR25, the FRAM contains setpoints Groups 0, 1, and 5 (3 copies of each
//                      group).
//                      For a PXR35, there are four sets of PXR35 setpoints in the FRAM for groups 0 - 9,
//                      a single set for groups 10, 11, and 12.  There two copies of of each group in each
//                      set of setpoints.  Global variable SetpActiveSet (read from the Frame FRAM)
//                      determines set is to read.
//                      The following algorithm is used to retrieve the setpoints:
//                          - If SetpActiveSet is valid, assume we have a PXR35 frame:
//                              - Read the first copy of the PXR35 setpoints.  If the checksum matches, use
//                                the setpoints and return 0
//                              - Read the second copy of the PXR35 setpoints.  If the checksum matches, use
//                                the setpoints and return 0
//                          - At this point, either SetpActiveSet is invalid and/or the PXR35 setpoints are
//                            invalid, so we assume we have a PXR25 frame:
//                              - If the setpoints are not common to the PXR25 (i.e., Group 0, 1, or 5), use
//                                defaults and return 2
//      `                       - Read the first copy of the PXR25 setpoints.  If the checksum matches, use
//                                the PXR25 setpoints where appropriate, and fill the remaining setpoints
//                                with defaults.  Return 1
//          `                   - Read the second copy of the PXR25 setpoints.  If the checksum matches, use
//                                the PXR25 setpoints where appropriate, and fill the remaining setpoints
//                                with defaults.  Return 1
//      `                       - Read the third copy of the PXR25 setpoints.  If the checksum matches, use
//                                the PXR25 setpoints where appropriate, and fill the remaining setpoints
//                                with defaults.  Return 1
//                          - Use defaults.  Return 2
//  `                   Note, the setpoints in RAM and FRAM are periodically checked in the main loop.  If
//                      there is a checksum error in one of the FRAM copies, it will be rewritten if the
//                      setpoints status for the group is 0 or 1.  Therefore, the FRAM does not need to be
//                      corrected in this subroutine.  Variable *good_copy is used for this purpose.
//                      Several read-only setpoints in Group 0 and Group 1 are loaded from their source.
//
//  CAVEATS:            The location pointed to by setp_ptr must have enough space allocated to handle the
//                      requested setpoints
//                      This function assumes the SPI2 bus is free
//
//  INPUTS:             group - the group number of the requested setpoints
//                      set - the setpoints set
//                      SetpActiveSet
// 
//  OUTPUTS:            SetpScratchBuf[] - where the setpoints are stored
//                      The function returns a code indicating the status of the setpoints:
//                          0 - all setpoints were retrieved successfully from the Frame FRAM (at least one
//                              of the two PXR35 copies were good)
//                          1 - the PXR25 setpoints were retrieved successfully for the group (at least one
//                              of the three PXR25 copies were good), but default values are used for the
//                              PXR35-only setpoints
//                          2 - the default setpoints are used (none of the setpoints are valid in FRAM)
//                          3 - all setpoints were retrieved successfully from the on-board FRAM (at least
//                              one of the two PXR35 copies were good)
//                       0xFF - Invalid Group requested
//                      *good_copy - the copy in FRAM that was used (i.e., had a good checksum)
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Read(), Frame_FRAM_Read()
//
//  EXECUTION TIME:     
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Get_Setpoints(uint8_t set, uint8_t group, uint8_t *good_copy)
{
  uint16_t sum;
  const uint16_t *sptr;
  uint16_t i, faddr;
  uint16_t chksum_rd, chksumnot_rd;
  uint8_t done, j, ret_code;

  if ((group >= NUM_STP_GROUPS) || (set >= NUM_STP_SETS))
  {
    return(0xFF); // If group/set parameter is out of range, return 0xFF
  }  
  done = FALSE;

  // If the set number is valid, assume it is a PXR35 and retrieve the setpoints from FRAM
  //   In addition, Groups 2, 3, and 11 are stored in on-board FRAM, not the Frame Module, so the active set
  //   doesn't matter.
  if ( (set < 4) || (group == 2) || (group == 3) || (group == 11) )
  {
    *good_copy = 10;                    // Set good copy to an invalid value in case all copies are bad
    // Read the setpoints, assuming it is a PXR35 Frame Module.  Groups 2, 3, and 14 are read from the
    //   on-board FRAM.  All other groups are read from the Frame Module.
    // All groups except groups 2, 3, 10, 11, 12, and 13 (i.e., 0-1, 4-9) have 4 sets of setpoints.
    //   Thus, the address must be offset by the active set (in SetpActiveSet)
    for (j = 0; j < 2; ++j)
    {
      if ( (group == 2) || (group == 3) || (group == 11) )
      {
        FRAM_Read(SETP_GR_FRAM_ADDR[(group << 1) + j], (SETP_GR_SIZE[group] >> 1), &SetpScratchBuf[0]);
        ret_code = 3;                                                   // Assume setpoints are good
      }
      else
      {
        faddr = ( (group < 10) ?
                        (SETP_GR_FRAM_ADDR[(group << 1) + j] + (SETP_SET_ADDRESS_OFFSET * set))
                      : (SETP_GR_FRAM_ADDR[(group << 1) + j]) );
        Frame_FRAM_Read(faddr, SETP_GR_SIZE[group], (uint8_t *)(&SetpScratchBuf[0]));
        ret_code = 0;                                                   // Assume setpoints are good
      }
      // If the checksum is good, set the return code to 0, and set done to True
      // Note, setpoint words are stored with the ls byte in the lower address and the ms byte in the higher
      //   address - this is compatible with the PXR25.  The checksum is done on a byte (not word) basis
      sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_SIZE[group] - 4));
      chksum_rd = SetpScratchBuf[(SETP_GR_SIZE[group] >> 1) - 2];       // Size buffers are in bytes
      chksumnot_rd = SetpScratchBuf[(SETP_GR_SIZE[group] >> 1) - 1];
      if ( (sum == chksum_rd) && (sum == (chksumnot_rd ^ 0xFFFF)) )
      {
        *good_copy = j;
        done = TRUE;
        break;
      }
    }
  }
  // At this point, if "done" is False.  The set number is invalid and/or the PXR35 setpoints are invalid,
  //  so assume it is a PXR25 frame

  // If not done, and it is setpoints Group 0, 1, or 5, read the setpoints group assuming it is a PXR25
  //   Frame Module
  if ( (done == FALSE) && ((group <= 1) || (group == 5)) )
  {
    for (j = 0; j < 3; ++j)
    {
      Frame_FRAM_Read(SETP_GR_PXR25_FRAM_ADDR[(group * 3) + j], SETP_GR_PXR25_SIZE[group],
                        (uint8_t *)(&SetpScratchBuf[0]));
      // If the checksum is good, use the PXR25 setpoints plus defaults for the PXR35-only setpoints
      sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_PXR25_SIZE[group] - 4));
      chksum_rd = SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 2];
      chksumnot_rd = SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 1];
      if ( (sum == chksum_rd) && (sum == (chksumnot_rd ^ 0xFFFF)) )
      {
        // Move default values into the PXR35-only setpoints for this group
        //   Source pointer (sptr) is set to the beginning of the defaults
        //   The index for the first default value ("i") is immediately after the last PXR25 setpoint
        //     (i.e., the index of the checksum)
        //   The end number is size/2 - 2 because we are loading words, not bytes (divide by 2) and we
        //     don't include the two checksum words (subtract 2).
        sptr = SETP_GR_DEFAULT_ADDR[group];
        i = (SETP_GR_PXR25_SIZE[group] >> 1) - 2;
        while (i < ((SETP_GR_SIZE[group] >> 1) - 2))
        {
          SetpScratchBuf[i] = sptr[i];
          i += 1;
        }
        ret_code = 1;
        *good_copy = j;
        done = TRUE;
        break;
      }
    }
  }

  // If not done, we cannot get any info from the Frame FRAM.  Load default values into the setpoints
  if (done == FALSE)
  {
    // Load in the default values
    //   Source pointer (sptr) is set to the beginning of the defaults
    //   The end number is size/2 - 2 because we are loading words, not bytes (divide by 2) and we don't
    //     include the two checksum words (subtract 2).
    sptr = SETP_GR_DEFAULT_ADDR[group];
    for (i = 0; i < ((SETP_GR_SIZE[group] >> 1) - 2); i++)
    {
      SetpScratchBuf[i] = sptr[i];
    }
    ret_code = 2;
    done = TRUE;                    // Leave valid flag at False, but set done flag
  }

  // For Group 0 and Group 1 the Rating and Breaker Frame setpoints are read-only duplicates of the
  // corresponding configuration data, so load these into the appropriate setpoint locations.
  if (group < 2)
  {
  SetpScratchBuf[0] = Break_Config.config.Rating;
  SetpScratchBuf[1] = Break_Config.config.BreakerFrame;
  }

  
  if (group == 0)
  {
    // Group 0 has read-only setpoints for style, style2, and Source Ground Sensor that are read
    // from Group 1.  Load them here, but it's up to the calling function to ensure Group 1 is read
    // first so that these are valid.
    SetpScratchBuf[2] = Setpoints1.stp.Style;
    SetpScratchBuf[3] =Setpoints1.stp.Style_2;
    SetpScratchBuf[22] = Setpoints1.stp.SG_Sensor;
    
    // Also for Group 0 load the active setpoints set
    SetpScratchBuf[24] = SetpActiveSet;
  }

  return(ret_code);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Get_Setpoints()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Load_SetpGr0_Gr1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Load setpoint groups 0 and 1
//
//  MECHANICS:          This subroutine loads setpoints group 0 and 1 into working RAM.  It is called after
//                      a reset and whenever the setpoints set is changed.
//
//  CAVEATS:            Load Group 1 setpoints first so that read-only setpoints copied from Group 1 to
//                      Group 0 are valid.
//
//  INPUTS:             SetpActiveSet, SETP_GR_DATA_ADDR[], SETP_GR_SIZE[]
// 
//  OUTPUTS:            SetpointsStat, Setpoints0.xx, Setpoints1.xx
//                      SetpScratchBuf[] - where the setpoints are stored
//                      The function returns a code indicating the status of the setpoints:
//                          0 - all setpoints were retrieved successfully from the Frame FRAM (at least one
//                              of the two PXR35 copies were good)
//                          1 - the PXR25 setpoints were retrieved successfully for the group (at least one
//                              of the three PXR25 copies were good), but default values are used for the
//                              PXR35-only setpoints
//                          2 - the default setpoints are used (none of the setpoints are valid in FRAM)
//                          3 - all setpoints were retrieved successfully from the on-board FRAM (at least
//                              one of the two PXR35 copies were good)
//                      *good_copy - the copy in FRAM that was used (i.e., had a good checksum)
//
//  ALTERS:             None
// 
//  CALLS:              Get_Setpoints(), Frame_FRAM_Read()
//
//  EXECUTION TIME:     Measured execution time on 220928 (rev 0.60 code):
//                                          532.1usec if using a PXR25 frame module
//                                          204.1usec if using a PXR35 frame module
// 
//------------------------------------------------------------------------------------------------------------

void Load_SetpGr0_Gr1(void)
{
  uint16_t *dptr;
  uint16_t temp, temp1;
  uint8_t i, j, k;

  //   Get_Setpoints() returns a status code as follows:
  //     0: all setpoints were retrieved successfully from the Frame FRAM
  //     1: only the PXR25 setpoints were retrieved successfully, remainder are default (this only applies
  //        to groups 0, 1, and 5)
  //     2: no setpoints were retrieved successfully; defaults are used
  //     3: all setpoints were retrieved successfully from the on-board FRAM
  //   SetpointsStat retains this status for each group in a bit-map:
  //     b0, 1: Group 0        b2, 3: Group 1        b4, 5: Group 2    ...    bn, n+1: Group n/2
  // Setpoints are stored in SetpScratchBuf[]
  // Note, SetpActiveSet has already been initialized from FRAM by Setp_VarInit()
  // Measured execution time on 220928 (rev 0.60 code): 532.1usec if using a PXR25 frame module
  //                                                    204.1usec if using a PXR35 frame module
  SetpointsStat = 0;                    // Setpoints status bits - initialize to all ok

  // Read Group 1, then 0 to ensure read-only setpoints in Group 0 are copied properly from Group 1
  i = 1;
  for (k = 0; k < 2; k++)
  {           
    SetpointsStat |= (Get_Setpoints(SetpActiveSet, i, &j) << (i << 1));
    // Move the setpoints from the scratchpad buffer into the appropriate setpoints structure
    //   Source pointer (sptr) is set to the beginning of the setpoints structure
    //   The end number is size/2 - 2 because we are loading words, not bytes (divide by 2) and we don't
    //     include the two checksum words (subtract 2).
    dptr = SETP_GR_DATA_ADDR[i];
    temp1 = (SETP_GR_SIZE[i] >> 1) - 2;
    for (temp = 0; temp < temp1; temp++)
    {
      *dptr++ = SetpScratchBuf[temp];
    }
    i--;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Load_SetpGr0_Gr1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Load_SetpGr2_LastGr()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Load setpoint groups 2 through the remaining setpoint groups
//
//  MECHANICS:          This subroutine loads setpoints group 2 thru the remaining setpoint groups into
//                      working RAM.  It is called after a reset and whenever the setpoints set is changed.
//                      This subroutine also examines the status of the Frame FRAM reads.  If there is an
//                      error, an event is entered
//
//  CAVEATS:            It is assumed that Load_SetpGr0_Gr1() has already been called to load in the first
//                      two setpoints groups
//
//  INPUTS:             SetpActiveSet, SETP_GR_DATA_ADDR[], SETP_GR_SIZE[]
// 
//  OUTPUTS:            SetpointsStat, Setpoints0.xx, Setpoints1.xx
//                      SetpScratchBuf[] - where the setpoints are stored
//                      SetpointsStat contains a code indicating the status of the setpoints:
//                          0 - all setpoints were retrieved successfully from the Frame FRAM (at least one
//                              of the two PXR35 copies were good)
//                          1 - the PXR25 setpoints were retrieved successfully for the group (at least one
//                              of the three PXR25 copies were good), but default values are used for the
//                              PXR35-only setpoints
//                          2 - the default setpoints are used (none of the setpoints are valid in FRAM)
//                          3 - all setpoints were retrieved successfully from the on-board FRAM (at least
//                              one of the two PXR35 copies were good)
//                      *good_copy - the copy in FRAM that was used (i.e., had a good checksum)
//                      The function returns a code indicating the frame status:
//                          0 - OK
//                          1 - Frame mismatch (configured for a PXR25)
//                          2 - Frame error (cannot retrieve setpoints)
//
//  ALTERS:             None
// 
//  CALLS:              Get_Setpoints(), Frame_FRAM_Read()
//
//  EXECUTION TIME:     Measured execution time on 220928 (rev 0.60 code):
//                                          2.2msec if using a PXR25 frame module
//                                          982usec if using a PXR35 frame module
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Load_SetpGr2_LastGr(void)
{
  uint16_t *dptr;
  uint16_t temp, temp1;
  uint8_t i, j;

  //   Get_Setpoints() returns a status code as follows:
  //     0: all setpoints were retrieved successfully from the Frame FRAM
  //     1: only the PXR25 setpoints were retrieved successfully, remainder are default (this only applies
  //        to groups 0, 1, and 5)
  //     2: no setpoints were retrieved successfully; defaults are used
  //     3: all setpoints were retrieved successfully from the on-board FRAM (Groups 2, 3, 11)
  //   SetpointsStat retains this status for each group in a bit-map:
  //     b0, 1: Group 0        b2, 3: Group 1        b4, 5: Group 2    ...    bn, n+1: Group n/2
  // Setpoints are stored in SetpScratchBuf[]
  for (i = 2; i < NUM_STP_GROUPS; i++)
  {
    SetpointsStat |= (Get_Setpoints(SetpActiveSet, i, &j) << (i << 1));
    // Move the setpoints from the scratchpad buffer into the appropriate setpoints structure
    //   Source pointer (sptr) is set to the beginning of the setpoints structure
    //   The end number is size/2 - 2 because we are loading words, not bytes (divide by 2) and
    //   we don't include the two checksum words (subtract 2).
    dptr = SETP_GR_DATA_ADDR[i];
    temp1 = (SETP_GR_SIZE[i] >> 1) - 2;
    for (temp = 0; temp < temp1; temp++)
    {
      *dptr++ = SetpScratchBuf[temp];
    }
    if (i == 12)
    {
      i++; // skip past group 13
    }
  }
  // 
  if (SetpointsStat == 0x00C000F0)      // If no error retrieving setpnts (Gr2, 3, 11 return 3), return
  {                                     //   status code = 0 (no error)
    i = 0;
  }                                     // If PXR25 setpoints ok, return status code = 1 (frame mismatch)
  else if ( (SetpointsStat & SETP_PXR25_GROUPS_STATMASK) == SETP_PXR25_GROUPS_OK)
  {
    i = 1;
  }
  else                                  // Otherwise error so return status code = 2
  {
    i = 2;
  }

  return (i);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Load_SetpGr2_LastGr()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Checksum8_16()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute checksum
//
//  MECHANICS:          This subroutine computes the checksum of a block of memory beginning at *addr.  The
//                      number of bytes is contained in length.
//                      A word (16-bit) checksum is computed by summing the bytes.
//
//  CAVEATS:            None
//
//  INPUTS:             *addr - the first byte to be computed in the checksum calculation
//                      length - the number of bytes in the checksum
// 
//  OUTPUTS:            The function returns the 16-bit checksum
//
//  ALTERS:             None
// 
//  CALLS:              None
//
//  EXECUTION TIME:     
// 
//------------------------------------------------------------------------------------------------------------

uint16_t Checksum8_16(uint8_t *addr, uint16_t length)
{
  uint16_t i, checksum;

  checksum = 0;
  if (length > 0)
  {
    for (i = 0; i < length; i++)
    {
      checksum += *addr++;
    }
  }
  return(checksum);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Checksum8_16()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Stp_to_Default()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Write Setpoints Message Subroutine
//
//  MECHANICS:          This subroutine handles a command to write setpoints from the display processor.
//                      It
//                          - parses the write request message contained in DPComm.RxMsg[]
//                          - stores the setpoints in the appropriate FRAM, if the message is valid
//                          - assembles the appropriate response (ack or nak) into DPComm.TxBuf[]
//
//  CAVEATS:            Note: Protection is disabled while the new setpoints are being written.  This only
//                      impacts the protection that is being done in the sampling interrupt, because the
//                      write is completed before this subroutine is exited
//
//  INPUTS:             bufid
//                      DPComm.RxMsg[]
// 
//  OUTPUTS:            Prot_Enabled
//
//  ALTERS:             DPComm.AckNak
//
//  CALLS:              Checksum8_16(), FRAM_Write(), Frame_FRAM_Write(), Gen_Values()
//
//------------------------------------------------------------------------------------------------------------

void Stp_to_Default(void)
{
  uint16_t i, set ,group;
  uint16_t chksum, chksumnot;
  const uint16_t *sptr;
  uint16_t *active_sptr;
  
  for(group = 0; group < NUM_STP_GROUPS; group++)
  {
    //fill SetpScratchBuf with default values
    sptr = SETP_GR_DEFAULT_ADDR[group];
    for (i = 0; i < ((SETP_GR_SIZE[group] >> 1) - 2); i++)
    {
      SetpScratchBuf[i] = sptr[i];
    }
    //calculate checksum
    chksum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_SIZE[group] - 4));
    chksumnot = (chksum ^ 0xFFFF);
    SetpScratchBuf[(SETP_GR_SIZE[group] >> 1) - 2] = chksum;       // Size buffers are in bytes
    SetpScratchBuf[(SETP_GR_SIZE[group] >> 1) - 1] = chksumnot;

    //Save SetpScratchBuf filled with default values to FRAM
    //group 2, 3, 11 are stored in On-Board FRAM
    if(group == 2 || group == 3 || group == 11)
    {
      FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(group << 1)], (SETP_GR_SIZE[group] >> 1),
                (uint16_t *)(&SetpScratchBuf[0]));
      FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(group << 1) + 1], (SETP_GR_SIZE[group] >> 1),
                (uint16_t *)(&SetpScratchBuf[0]));
    }
    //other groups stored in FRAME FRAM
    else
    {
      for(set = 0; set < 4; set++)
      {     
        Frame_FRAM_Write((SETP_GR_FRAM_ADDR[(group << 1)] + (SETP_SET_ADDRESS_OFFSET * set)),
                          SETP_GR_SIZE[group], (uint8_t *)(&SetpScratchBuf[0]));
        Frame_FRAM_Write((SETP_GR_FRAM_ADDR[(group << 1)  + 1] + (SETP_SET_ADDRESS_OFFSET * set)),
                          SETP_GR_SIZE[group], (uint8_t *)(&SetpScratchBuf[0]));      
      }
    }
    
    //fill active setpoints Set with default values
    active_sptr = SETP_GR_DATA_ADDR[group];
    for (i = 0; i < ((SETP_GR_SIZE[group] >> 1) - 2); i++)
    {
      *active_sptr++ = SetpScratchBuf[i];
    }
    if (group == 11)
    {
      group += 2; // skip groups 12 and 13
    }
  }

  Gen_Values();
  // If the demand setpoints changed, we need to restart logging with the new variables
  //   Since this is a factory command, we will not change the EID or log an event
  if ( (Dmnd_Setp_DemandWindow != (uint8_t)Setpoints0.stp.DemandWindow)
    || (Dmnd_Setp_DemandInterval != (uint8_t)Setpoints0.stp.DemandInterval)
    || (Dmnd_Setp_DemandLogInterval = (uint8_t)Setpoints0.stp.DemandLogInterval) )
  {
    Dmnd_VarInit();
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Stp_to_Default()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Check_Setpoints()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Check a setpoints group
//
//  MECHANICS:          This subroutine checks the integrity of an existing setpoints group (SetpointsN) by
//                      comparing it to the corresponding contents in FRAM.
//                          - Read setpoints group from FRAM
//                          - Check returned setpoints status code versus the existing status
//                          - If the returned status is different than the existing status, the FRAM has
//                            changed somehow (this should not happen).  In this case, load the issue an
//                            alarm (one time only) that indicates a Frame FRAM issue.  Don't change the RAM
//                            setpoints.
//                          - If the returned status is the same as the existing status (this should always
//                            be the case), compare the two sets of setpoints.  If there is a mismatch,
//                            correct the RAM copy, but don't alarm.
//                            In addition, perform a check of the copies in FRAM.  If there is a bad or
//                            outdated copy of the setpoints in FRAM, rewrite the good copy into this copy.
//
//  CAVEATS:            SetpScratchBuf[] AND spi2_BUF[] must have enough space allocated to handle the
//                      requested setpoints
//                      This function assumes the SPI2 bus is free
//
//  INPUTS:             group - the group number of the setpoints to be checked
//                      SystemFlags.FRAME_FRAM_ERR, SetpActiveSet
// 
//  OUTPUTS:            SetpointsN (where N is denoted by the group)
//                      Frame FRAM setpoints contents
//
//  ALTERS:             SetpScratchBuf[], SPI2_buf[], SystemFlags.FRAME_FRAM_ERR
// 
//  CALLS:              Get_Setpoints(), Frame_FRAM_Read(), FRAM_Write()
//
//  EXECUTION TIME:     Measured on 220928 (rev 0.60 code): 1.05msec with Gr10 setpoints and rewriting the
//                      second copy
// 
//------------------------------------------------------------------------------------------------------------

void Check_Setpoints(uint8_t group)
{
  uint16_t *setp_dptr;
  uint8_t fram_stat, goodcopy, i, j, temp1[2], num_sp;
  uint16_t sum, temp, k, faddr;

  // Check the active setpoints set value versus the FRAM value
  // Read active set from Frame FRAM.  If it is valid, check it versus the active set
  // If it's invalid, don't perform the check (maybe the Frame is a PXR25 FRAME or it is bad and we don't
  //   want to change the active set if it has been reloaded but cannot be stored in the Frame).  Also,
  //   don't check the other setpoints if the active set is bad
  Frame_FRAM_Read(SETP_ACTIVE_SET_ADDR, 2, &temp1[0]);
  if ( ((temp1[0] ^ temp1[1]) == 0xFF) && (temp1[0] < 4) )
  {
    if (SetpActiveSet != temp1[0])
    {
      SetpActiveSet = temp1[0];          // *** DAH SHOULD WE ALARM HERE?
    }
  }
  else
  {
    return;
  }

  // Retrieve the setpoints
  //   Setpoints are returned in SetpScratchBuf[]
  //   fram_stat has the status of the setpoints in FRAM:
  //     0 - at least one of the two PXR35 copies in Frame FRAM were good
  //     1 - no PXR35 setpoints are good; at least one of the PXR25 setpoints are good
  //     2 - none of the setpoints are good; defaults only
  //     3 - at least one of the two PXR35 copies in on-board FRAM were good
  //   goodcopy has the number of the good copy (0 or 1 for PXR35 setpoints; 0, 1, or 2 for PXR25 setpoints)
  fram_stat = Get_Setpoints(SetpActiveSet, group, &goodcopy);
  // For group1 setpoints, the following setpoints are read-only and are duplicates of the corresponding
  //   group0 setpoints:
  //        Rating    Frame    Style    Style_2    SG Sensor
  // These must be overwritten with the Group0 values
  if (group == 1)
  {
    SetpScratchBuf[0] = Setpoints0.stp.Rating;
    SetpScratchBuf[1] = Setpoints0.stp.Breakframe;
    SetpScratchBuf[2] = Setpoints0.stp.Style;
    SetpScratchBuf[3] = Setpoints0.stp.Style_2;
    SetpScratchBuf[32] = Setpoints0.stp.SG_Sensor;
  }

  // Compare the status returned (framstat) to the existing status (in 2 bits of SetpointsStat, based on the
  //   group).  If the status matches, check the setpoints
  if (fram_stat == ((SetpointsStat >> (group << 1)) & 0x0003))
  {
    // Check each setpoint in the RAM working copy to the copy read from FRAM
    setp_dptr = SETP_GR_DATA_ADDR[group];
    num_sp = (SETP_GR_SIZE[group] >> 1) - 2;
    for (i = 0; i < num_sp; i++)
    {
      if (*setp_dptr != SetpScratchBuf[i])                           // If mismatch, correct the setpoint
      {                                                              //   (no alarm)
        *setp_dptr = SetpScratchBuf[i];                              // Note, could just rewrite, but leave
      }                                                              //   it this way for now in case we
      setp_dptr++;                                                   //   need to add an additional action
    }
    // Check and correct the other FRAM copies if necessary
    switch (fram_stat)                  // Depend on what setpoints we are using (PXR35, PXR25, or defaults)
    {
      case 0:                               // Using PXR35 setpoints (Frame FRAM) - only have two copies
        // If first copy is good, check the second copy
        if (goodcopy == 0)
        {
          // Read the setpoints into SPI2_buf[].  Since SPI2_buf[] is a byte buffer, we need to assemble the
          //   16-bit values to do the comparison.
          faddr = ( (group != 10) ?
                          (SETP_GR_FRAM_ADDR[(group << 1) + 1] + (SETP_SET_ADDRESS_OFFSET * SetpActiveSet))
                        : (SETP_GR_FRAM_ADDR[(group << 1) + 1]) );
          Frame_FRAM_Read(faddr, SETP_GR_SIZE[group], (uint8_t *)(&SPI2_buf[0]));
          k = 0;
          num_sp = (SETP_GR_SIZE[group] >> 1);
          for (i = 0; i < num_sp; i++)
          {
            temp = SPI2_buf[k] + ((uint16_t)SPI2_buf[k+1] << 8);
            if (temp != SetpScratchBuf[i])                                  // If mismatch, rewrite the
            {                                                               //   second copy and break
              Frame_FRAM_Write(faddr, SETP_GR_SIZE[group], (uint8_t *)(&SetpScratchBuf[0]));
              break;
            }
            k += 2;
          }
        }
        // If second copy is good, first copy must be bad, so just rewrite it
        else
        {
          faddr = ( (group != 10) ?
                          (SETP_GR_FRAM_ADDR[(group << 1) + 0] + (SETP_SET_ADDRESS_OFFSET * SetpActiveSet))
                        : (SETP_GR_FRAM_ADDR[(group << 1) + 0]) );
          Frame_FRAM_Write(faddr, SETP_GR_SIZE[group], (uint8_t *)(&SetpScratchBuf[0]));
        }
        break;

      case 1:                               // Using PXR25 setpoints - three copies
        // Check the other two copies (only the PXR25 setpoints) versus the good copy
        // Compute the checksum for the good PXR25 setpoints.  Note, it is not kept when the setpoints are
        //   retrieved, because it is overwritten by the additional default PXR35 setpoints
        sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_PXR25_SIZE[group] - 4));
        for (i = 0; i < 3; ++i)
        {
          if (goodcopy != i)
          {
            Frame_FRAM_Read(SETP_GR_PXR25_FRAM_ADDR[(group * 3) + i], SETP_GR_PXR25_SIZE[group],
                              (uint8_t *)(&SPI2_buf[0]));
            k = 0;
            num_sp = (SETP_GR_PXR25_SIZE[group] >> 1) - 2;
            for (j = 0; j < num_sp; j++)
            {
              temp = SPI2_buf[k] + ((uint16_t)SPI2_buf[k+1] << 8);
              if (temp != SetpScratchBuf[j])                                // If mismatch, rewrite the
              {                                                             //   second copy and break
                SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 2] = sum;
                SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 1] = (sum ^ 0xFFFF);
                Frame_FRAM_Write(SETP_GR_PXR25_FRAM_ADDR[(group * 3) + i], SETP_GR_PXR25_SIZE[group],
                                    (uint8_t *)(&SetpScratchBuf[0]));
                break;
              }
              k += 2;
            }
            if (j >= ((SETP_GR_PXR25_SIZE[group] >> 1) - 2))        // If setpoints ok so far, check the
            {                                                       //   checksum
              if (sum != (SPI2_buf[k] + ((uint16_t)SPI2_buf[k+1] << 8)) )   // If mismatch, rewrite the
              {                                                             //   second copy
                SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 2] = sum;
                SetpScratchBuf[(SETP_GR_PXR25_SIZE[group] >> 1) - 1] = (sum ^ 0xFFFF);
                Frame_FRAM_Write(SETP_GR_PXR25_FRAM_ADDR[(group * 3) + i], SETP_GR_PXR25_SIZE[group],
                                    (uint8_t *)(&SetpScratchBuf[0]));
              }
            }
          }
        }
        break;

      case 3:                               // Using PXR35 setpoints (on-board FRAM) - only have two copies
        // If first copy is good, check the second copy
        if (goodcopy == 0)
        {
          // Read the setpoints into SPI2_buf[].  Since SPI2_buf[] is a byte buffer, we need to assemble the
          //   16-bit values to do the comparison.
          FRAM_Read(SETP_GR_FRAM_ADDR[(group << 1) + 1], (SETP_GR_SIZE[group] >> 1),
                        (uint16_t *)(&SPI2_buf[0]));
          k = 0;
          num_sp = (SETP_GR_SIZE[group] >> 1);
          for (i = 0; i < num_sp; i++)
          {
            temp = SPI2_buf[k] + ((uint16_t)SPI2_buf[k+1] << 8);
            if (temp != SetpScratchBuf[i])                                  // If mismatch, rewrite the
            {                                                               //   second copy and break
              FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(group << 1) + 1],
                                (SETP_GR_SIZE[group] >> 1), &SetpScratchBuf[0]);
              break;
            }
            k += 2;
          }
        }
        // If second copy is good, first copy must be bad, so just rewrite it
        else
        {
          FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(group << 1)+ 0],
                                (SETP_GR_SIZE[group] >> 1), &SetpScratchBuf[0]);
        }
        break;

      default:                              // Using defaults - no other copies to check
        break;
    }
  }
  // If the status doesn't match, the Frame module has somehow changed.  Send an alarm if one hasn't been
  //   sent already
  else if ( (SystemFlags & FRAME_FRAM_ERR) == 0)
  {
    SystemFlags |= FRAME_FRAM_ERR;
    NewEventFIFO[NewEventInNdx].Code = STP_ERROR;
    Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
    NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
    NewEventInNdx &= 0x0F;                                        // Note, FIFO size must be 16!!
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Check_Setpoints()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Verify_Setpoints()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Verify the values of all setpoints in a group
//
//  MECHANICS:          This subroutine verifies that all of the setpoints in a group are within acceptable
//                      limits.  This is typically done prior to storing a group of setpoints that have been
//                      changed via Modbus, display, USB, etc.
//                        - For simple monotonic ranges the minimum and maximum values are specified in
//                          MODB_SETP_GRPx_MIN[] and MODB_SETP_GRPx_MAX[] where "x" is the setpoint group.
//                        - For read-only and unused setpoints the values in MODB_SETP_GRPx_MIN[] and
//                          MODB_SETP_GRPx_MAX[] are 0xFFFF. Copy current value into SetpScratchBuf[]
//                        - More complex setpoint ranges have values of 0xFFFx in MODB_SETP_GRPx_MIN[] and
//                          MODB_SETP_GRPx_MAX[], requiring special processing based on the limits for that
//                          particular setpoint.
//                        - Each setpoint in the group is verified based on the assosciated value in
//                          MODB_SETP_GRPx_MIN[] and MODB_SETP_GRPx_MAX[] described above.
//
//  CAVEATS:            This function assumes all setpoints to verify are correctly placed in SetpScratchBuf[]
//
//  INPUTS:             group - group number of the setpoints to be verified
//                      SETP_GR_SIZE[] - number of bytes in the setpoint group including 4 checksum bytes
//                      stp_ptr -  - address in memory of setpoint values to be verified
// 
//  OUTPUTS:            Returns 1 if all setpoints in the group are valid, 0 otherwise.
//
//  ALTERS:             Read-only setpoint values
// 
//  CALLS:              Nothing
//
//  EXECUTION TIME:     ???
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Verify_Setpoints(uint8_t group, uint16_t *stp_ptr)
{
  const uint16_t* setp_min, *setp_max;
  uint16_t i, numsetp;
  uint8_t setpoints_ok;
  setpoints_ok = 1; // assume all are OK
  numsetp = ((SETP_GR_SIZE[group] >> 1) - 2);
  setp_min = MODB_SETP_MIN_ADDR[group];
  setp_max = MODB_SETP_MAX_ADDR[group];

  // The following implementation has some repeated cases due to common ranges between
  // setpoint groups. Maybe not the most efficient, but readable and maintainable.
  // This may be simplified by removing the switch(group) and handle all groups
  // in a single "for" loop. To do this all unique/special ranges require unique 0xFFxx
  // values. More difficult to maintain if ranges change in the future, and more special
  // cases before the default "normal" range, so probably longer execution time.
  switch (group)
  {
    // Group 0 - System
    case 0:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints0.buf[(i * 2) + 1] << 8) + Setpoints0.buf[i * 2]);
            break;
          case 0xFFFE:
            // Maintenance Mode:
            // b08: MM state
            // b07: MM local ON
            // b06: MM local setting invalid (not used for PXR35?)
            // b01: MM signal from secondary pin (not used for PXR35, combined with b07)
            // b00: MM remote com channel
            // Test for invalid combos:
            //   - local and remote OFF but state ON
            //   - local or remote ON but state OFF
            if ((((*stp_ptr & BIT7) == 0) && ((*stp_ptr & BIT0) == 0) &&
                 ((*stp_ptr & BIT8) == BIT8 )) ||
                ((((*stp_ptr & BIT7) == 1) || ((*stp_ptr & BIT0) == 1)) &&
                 ((*stp_ptr & BIT8) == 0 )))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFD:
            // Line Frequency: value can only be 0, 50, 60, or 400
            if ((*stp_ptr != 0) && (*stp_ptr != 50) && (*stp_ptr != 60) && (*stp_ptr != 400))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFC:
            // Demand Interval:
            // value can only be 0 (off) or 5 to 60 in steps of 1
            if ((*stp_ptr != 0) && ((*stp_ptr < 5) || (*stp_ptr > 60)))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFB:
            // Demand Logging Interval: 0 (off), 5, 10, 15, 20, 30, 60
            // From setpoints spreadsheet " Also, this depends on the demand interval. For fixed
            //   windows, logging interval must be >= demand interval; not so for sliding windows.
            //   If demand window is off, can still log energy, but not demands.  If logging
            //   interval is off, can compute demands but not log them."
            // First make sure value is valid in general
            if ((*stp_ptr != 0) && (*stp_ptr!= 5) && (*stp_ptr != 10) && (*stp_ptr!= 15) &&
                (*stp_ptr!= 20) && (*stp_ptr != 30) && (*stp_ptr != 60))
            {
              setpoints_ok = 0;
            }
            // We could combine the following test for "logging interval must be >= demand interval"
            // with the previous tests, but sometimes compilers get confused by large combinational
            // logic.  Also we may want to remove this if we don't really need to verify internally?

            // Demand Interval is 21 setpoints before Demand Logging Interval
            // Demand Type (fixed = 0, sliding = 1) is 22 setpoints before Demand Logging Interval
            // Now check validity if the window is fixed:
            //   invalid if Logging Interval < Demand Interval for Fixed Demands
//            else if ((*stp_ptr != 0) && (*stp_ptr < *(stp_ptr - 21)))
            else if ((*stp_ptr != 0) && (*stp_ptr < *(stp_ptr - 21)) && (*(stp_ptr - 22) == 0))
            {
              setpoints_ok = 0;
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || (*stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        // we could technically break out of the loop if a bad value is detected, but why bother?
        stp_ptr++;  // next setpoint
      }
      break;
    
    // Group 1 - Current Protection
    case 1:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints1.buf[(i * 2) + 1] << 8) + Setpoints1.buf[i * 2]);
            break;
          case 0xFFFE:
            // Trip unit style - writable in manufacturing mode, else read-only
            // Valid values are 0x20 and 0x21
            if (Manufacture_Mode)
            {
              if ((*stp_ptr != 0x20) && (*stp_ptr != 0x21))
              {
                setpoints_ok = 0;
              }
            }
            else
            {
              *stp_ptr = Setpoints1.stp.Style; // use existing value
            }
            break;
          case 0xFFFD:
            // Trip unit style 2 - digitization: writable in manufacturing mode, else read-only
            // At this time bits 0 and 1 are defined but others are expected.  For now allow all
            if (!Manufacture_Mode)
            {
              *stp_ptr = Setpoints1.stp.Style_2; // use existing value
            }
            else
            {
            // all are OK for now
// TODO: MAG - Fill in the "else" with allowable bits when defined
            }
            break;
          case 0xFFFC:
            // Long Delay Time: Dependent on Long Delay Slope
            // Long Delay slope is 2 setpoints before Long Delay Time
            switch (*(stp_ptr - 2))
            {
              case 0: // I0.5t slope: 50 to 2400, steps of 50
              case 1: // It slope: 50 to 2400, steps of 50
                if ((*stp_ptr < 50) || (*stp_ptr > 2400) || (*stp_ptr %50 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 2: // I2t slope: 50 to 2400, steps of 10
                if ((*stp_ptr < 50) || (*stp_ptr > 2400) || (*stp_ptr %10 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 3: // I4t slope: 50 to 700, steps of 10
                if ((*stp_ptr < 50) || (*stp_ptr > 700) || (*stp_ptr %10 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 4: // IEEE Moderately Inverse slope: 10 to 500, steps of 10
                if ((*stp_ptr < 10) || (*stp_ptr > 500) || (*stp_ptr %10 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 5: // IEEE Very Inverse slope: 20 to 500, steps of 10
              case 6: // IEEE Extrememly Inverse slope: 20 to 500, steps of 10
                if ((*stp_ptr < 20) || (*stp_ptr > 500) || (*stp_ptr %10 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 7: // IEC A: 5 to 100, steps of 5
                if ((*stp_ptr < 5) || (*stp_ptr > 100) || (*stp_ptr %5 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              case 8: // IEC B: 10 to 100, steps of 5
                if ((*stp_ptr < 10) || (*stp_ptr > 100) || (*stp_ptr %5 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
              default: // really case 9 - IEC C: 20 to 100, steps of 5
                if ((*stp_ptr < 20) || (*stp_ptr > 100) || (*stp_ptr %5 != 0))
                {
                  setpoints_ok = 0;
                }
                break;
            }
            break;
          case 0xFFFB:
            // High Load Alarm 1: 50 to (HL2 - 1)
            // High Load Alarm 2 is 12 setpoints after High Load Alarm 1
            if ((*stp_ptr < 50) || (*stp_ptr > (*(stp_ptr + 12) - 1)))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFA:
            // Instantaneous Pickup: 0 = off, 20 to 150
            if (((*stp_ptr < 20) && (*stp_ptr != 0)) || (*stp_ptr > 150))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF9:
            // Ground Fault Time: Dependent on Ground Fault Slope
            // Flat slope: 5 to 100
            // Ground Fault Slope is 2 setpoints before Ground Fault Time
            if (*(stp_ptr - 2) == 0)
            {
              if ((*stp_ptr < 5) || (*stp_ptr > 100))
              {
                setpoints_ok = 0;
              }
            }
            // I2t slope: 10 to 100
            else // I2t (invalid slope setpoint will be caught when testing that setpoint)
            {
              if ((*stp_ptr < 10) || (*stp_ptr > 100))
              {
                setpoints_ok = 0;
              }
            }
            break;
          case 0xFFF8:
            // Neutral Protection Ratio: 0, 60, 100, or 200 for 3-pole with LDPU <= 50%
            //                           0, 60, 100 for 4-pole or 3-pole with LDPU > 50%
            if ((*stp_ptr == 200) && ((Break_Config.config.Poles |= 3) || (*(stp_ptr - 13) > 50)))
            {
              setpoints_ok = 0;
            }
            else if ((*stp_ptr != 0) && (*stp_ptr != 60) && (*stp_ptr != 100))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFF7:
            // High Load Alarm 2: (HL1 + 1) to 120
            // High Load Alarm 1 is 12 setpoints before High Load Alarm 2
            if ((*stp_ptr < (*(stp_ptr - 12) + 1)) || (*stp_ptr > 120))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFF6:
            // Ground Pre-Alarm Pickup: 50 to 100, steps of 5
            if (((*stp_ptr < 50) || (*stp_ptr > 100)) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFF5:
            // Thermal Alarm: 0 = off, 50 to 100, steps of 5
            if (*stp_ptr != 0)
            {
              if (((*stp_ptr < 50) || (*stp_ptr > 100)) || (*stp_ptr %5 != 0))
              {
                setpoints_ok = 0;
              }
            }
            break;
          case 0xFFF4:
            // Neutral Alarm Pickup: 0 = off, 10 to 100, steps of 5
            if (*stp_ptr != 0)
            {
              if (((*stp_ptr < 10) || (*stp_ptr > 100)) || (*stp_ptr %5 != 0))
              {
                setpoints_ok = 0;
              }
            }
            break;
          case 0xFFF3:
            // SG Sensor - Valid values are 0, 21, 51, 101, 201, 401, 801, 1601, 2001, and 2401
            if ((*stp_ptr != 0) && (*stp_ptr != 21) && (*stp_ptr != 51) && (*stp_ptr != 101) &&
                (*stp_ptr != 201) && (*stp_ptr != 401) && (*stp_ptr != 801) &&
                (*stp_ptr != 1601) && (*stp_ptr != 2001) && (*stp_ptr != 2401))
            {
              setpoints_ok = 0;
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || ( *stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Groups 2 - Modbus, and 7 - Sync Check have no special ranges (0xFFFx) to check
    case 2:
    case 7:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            if (group == 2)
            {
              *stp_ptr = ((Setpoints2.buf[(i * 2) + 1] << 8) + Setpoints2.buf[i * 2]);
            }
            else
            {
              *stp_ptr = ((Setpoints7.buf[(i * 2) + 1] << 8) + Setpoints7.buf[i * 2]);
            }
            break;
          // no special ranges
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || ( *stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;
    
    // Group 4 - Frequency Protection
    case 4:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints4.buf[(i * 2) + 1] << 8) + Setpoints4.buf[i * 2]);
            break;
          case 0xFFFE:
            // Trip/Alarm Time Delays: 5 to 30000, steps of 5
            if (((*stp_ptr < 5) || (*stp_ptr > 30000)) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || ( *stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Group 5 - Extended Protection
    case 5:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints5.buf[(i * 2) + 1] << 8) + Setpoints5.buf[i * 2]);
            break;
          case 0xFFFE:
            // Trip/Alarm Time Delays: 5 to 30000, steps of 5
            if ((*stp_ptr < 5) || (*stp_ptr > 30000) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFD:
            // Trip/Alarm Time Delays: 100 to 30000, steps of 5
            if ((*stp_ptr < 100) || (*stp_ptr > 30000) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || (*stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Group 6 - Power Protection
    case 6:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints6.buf[(i * 2) + 1] << 8) + Setpoints6.buf[i * 2]);
            break;
          case 0xFFFE:
            // Under Power Factor  Pickup: 20 to 95, steps of 5
            if ((*stp_ptr < 20) || (*stp_ptr > 95) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || (*stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Group 8 - ATS
    case 8:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints8.buf[(i * 2) + 1] << 8) + Setpoints8.buf[i * 2]);
            break;
          case 0xFFFE:
            // Load Voltage Decay: 0 = off, 2 to 30
            if (((*stp_ptr < 2) && (*stp_ptr != 0)) || (*stp_ptr > 30))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFFD:
            // Time Delay Emergency Disconnect: 0xFFFF = off, 0 to 10
            // Time Delay Normal Disconnect: 0xFFFF = off, 0 to 10
            if ((*stp_ptr > 10) && (*stp_ptr != 0xFFFF))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFFC:
            // Overvoltage Dropout: 0 = disabled, 105 to 120
            if (((*stp_ptr < 105) && (*stp_ptr != 0)) || (*stp_ptr > 120))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFFB:
            // Overvoltage Pickup: 0 = disabled, 103 to (OVDO - 2)
            // Overvoltage Dropout is 1 setpoints before Overvoltage Pickup
            if (((*stp_ptr < 103) && (*stp_ptr != 0)) || (*stp_ptr > (*(stp_ptr - 1) - 2)))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFFA:
            // Undervoltage Pickup: (UVDO + 2) to 97
            // Undervoltage Dropout is 1 setpoints before Undervoltage Pickup
            if ((*stp_ptr < (*(stp_ptr - 1) + 2)) || (*stp_ptr > 97))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF9:
            // Overfrequency Dropout: 0 = disabled, 103 to 110
            if (((*stp_ptr < 103) && (*stp_ptr != 0)) || (*stp_ptr > 110))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF8:
            // Overfrequency Pickup: 0 = disabled, 101 to (OFDO - 2)
            // Overfrequency Dropout is 1 setpoints before Overfrequency Pickup
            if (((*stp_ptr < 103) && (*stp_ptr != 0)) ||(*stp_ptr > (*(stp_ptr - 1) - 2)))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF7:
            // Underfrequency Dropout: 0 = disabled, 90 to 97
            if (((*stp_ptr < 90) && (*stp_ptr != 0)) || (*stp_ptr > 97))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF6:
            // Underfrequency Pickup: 0 = disabled, (UFDO + 2) to 99
            // Underfrequency Dropout is 1 setpoints before Underfrequency Pickup
            if (((*stp_ptr < (*(stp_ptr - 1) + 2)) && (*stp_ptr != 0)) || (*stp_ptr > 99))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF5:
            // Voltage Unbalance Dropout: 0 = disabled, 5 to 60
            if (((*stp_ptr < 5) && (*stp_ptr != 0)) || (*stp_ptr > 60))
              {
                setpoints_ok = 0;
              }
            break;
          case 0xFFF4:
            // Voltage Unbalance Pickup: 0 = disabled, 3 to (VUNBALDO - 2)
            // Voltage Unbalance Dropout is 1 setpoints before Voltage Unbalance Pickup
            if (((*stp_ptr < 3) && (*stp_ptr != 0)) || (*stp_ptr > (*(stp_ptr - 1) - 2)))
              {
                setpoints_ok = 0;
              }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || (*stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Group 9 - Other
    case 9:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            *stp_ptr = ((Setpoints9.buf[(i * 2) + 1] << 8) + Setpoints9.buf[i * 2]);
            break;
          case 0xFFFE:
            // Overtemperature Alarm:  85 to 105, steps of 5
            if (((*stp_ptr < 85) || (*stp_ptr >= 105)) || (*stp_ptr %5 != 0))
            {
              setpoints_ok = 0;
            }
            break;
          case 0xFFFD:
            // Current THD Alarm Pickup: 0 = off, 10 to 30
            // Voltage THD Alarm Pickup: 0 = off, 10 to 30
            if (((*stp_ptr < 10) && (*stp_ptr != 0)) || (*stp_ptr > 30))
              {
                setpoints_ok = 0;
              }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || ( *stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Groups 10 and 13 - GOOSE
    case 10:
    case 13:
      for (i = 0; i < numsetp; i++)
      {
        switch(setp_min[i])
        {
          case 0xFFFF:
            // read-only, but copy existing value into scratch buffer
            if (group == 10)
            {
              *stp_ptr = ((Setpoints10.buf[(i * 2) + 1] << 8) + Setpoints10.buf[i * 2]);
            }
            else
            {
              *stp_ptr = ((Setpoints13.buf[(i * 2) + 1] << 8) + Setpoints13.buf[i * 2]);
            }
            break;
          default:
            //Continuous range with step size of 1
            if ((*stp_ptr < setp_min[i]) || ( *stp_ptr > setp_max[i]))
            {
              setpoints_ok = 0;
            }
            break;
        }
        stp_ptr++;  // next setpoint
      }
      break;

    // Group 12 - Relays
    case 12:
      // three sets of 64-bit bitfields. At this time any combination is allowed!
      break;
      
    // Groups 3 - CAM 1, and 11 - CAM 2 are currently unused, anything else is undefined
    default:
      setpoints_ok = 0;
      break;
  }
  return (setpoints_ok);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Verify_Setpoints()
//------------------------------------------------------------------------------------------------------------


