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
//  MODULE NAME:        Events_def.h
//
//  MECHANICS:          This is the definitions file for the Events.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.22   180420  DAH Added definitions to support the Events Manager
//   0.23   180504  DAH Added EM_ENERGYLOG3 and EM_ENERGYLOG4 definition to EventManager_States
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Deleted EM_ENERGYLOG2, EM_ENERGYLOG3, and EM_ENERGYLOG4 as they are no longer
//                            used
//                          - Added MAXEVENTCODE
//                      - Added support for Alarm waveform capture events
//                          - Added the event ID (EID) to struct NEWEVENTFIFO.  This enables the EID to be
//                            in the order of the events (like the time tag)
//   0.37   200604  DAH - Added definitions and structures to support indexed Flash waveform storage
//                          - Added EVENT_TYPES definitions
//                          - Added struct EV_SUM and struct EV_ADD
//                      - Revised Demand and Energy logging variables to be compatible with the other event
//                        variables
//                          - Added struct EV_DMND
//   0.61   221001  DB  - Added support for RTC events to EVENT_TYPES and EventManager_States
//                      - Revised Summary Events Codes list
//                      - Added struct DATE_TIME and struct TIMEADJ
//   0.69   230220  DAH - Deleted Minor Alarm events.  These are handled by Basic events with the expanded
//                        Summary Log codes
//                          - EM_MINORALARM and EM_MINORALARM1 deleted from EventManager_States
//                          - Deleted EV_TYPE_MINALARM from EVENT_TYPES
//                          - Renamed EV_TYPE_MAJALARM to EV_TYPE_ALARM in EVENT_TYPES
//                          - Revised the summary event codes, primarily moving and renaming minor alarm
//                            codes into the summary (basic) event codes
//                      - Deleted EV_TYPE_RTD from EVENT_TYPES
//                      - In struct NEWEVENTFIFO, renamed Type to Code
//                      - Revised and renumbered event codes.  Added codes for disturbance events
//                      - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          EV_TYPE_CHARTWF removed from EVENT_TYPES[]
//                      - Deleted EM_TRIP_NO_CAPTURE, EM_TRIP1_NO_CAPTURE, EM_TRIP3, EM_TRIP4, EM_TRIP5 from
//                        EventManager_States
//                  DB  - Added ExtCapMeasuredVals constants for extended captures
//   0.71   230228  DAH - Revised order of alarm events.  Demand overpower events do not initiate a
//                        disturbance capture
//                      - Deleted EM_ALARM1, EM_ALARM2, and EM_ALARM_WF_CAPTURE from EventManager_States
//                  DB -  Added support for extended captures and disturbance captures
//   24     230323  DAH - Added saving the EID for the extended capture log entries
//                          - EID added to struct EXCTAP_STATE
//   25     230403  DAH - Added disturbance and extended capture snapshots to EVENT_TYPES
//                      - Added EV_TYPE_LAST
//   27     230404  DAH - Added DS_ALL
//   31     230414  DAH - Added Extended Capture event codes for events without snapshots or extended
//                        captures, just summary logs.  These handle events that occur while an extended
//                        capture is already occurring.  A Summary event is entered, but no other entries
//                        (i.e., Snapshots or Captures) are made
//   37     230516  DAH - Revised extended capture definitions from words to bytes
//                      - Added disturbance capture codes and revised the way the disturbance capture flags
//                        are defined
//                      - Added EIDofStartEvnt to struct DIST_VALUES and struct DIST_CAPTURE
//                      - Deleted struct DIST_CAPTURE
//                      - In struct DIST_VALUES, added ValueCode and rearranged the elements so that they
//                        align with the events values saved in FRAM
//                      - Added DS_WAIT to DisturbanceStatus states
//                      - Added DIST_EVCODE_START
//                      - Added ALARM_xxx_ENTRY_TEST definitions to support Test alarms
//                      - Added xx_PICKUP definitions to support disturbance captures
//   54     230801  DAH - Added support for short delay disturbance captures
//                          - DS_SD_PICKUP and DS_SD added to disturbance codes and flags
//                          - Added SDPU_ENTRY and SDPU_EXIT to event codes
//                      - Added ALARM_GND_FAULT_PRE_TEST to event codes
//   58     230810  DAH - Added STP_FRAME_MISMATCH and STP_ERROR to event codes
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
//


// Definitions

//---------------------------------------------- Event types -----------------------------------------------
enum EVENT_TYPES
{
  EV_TYPE_SUMMARY = 1,
  EV_TYPE_TIMEADJ,
  EV_TYPE_TRIP,
  EV_TYPE_TESTTRIP,
  EV_TYPE_ALARM,
  EV_TYPE_DEMAND,
  EV_TYPE_TRIPWF,
  EV_TYPE_ALARMWF,
  EV_TYPE_EXTCAPWF,
  EV_TYPE_DIST,
  EV_TYPE_EXTCAP
};
#define EV_TYPE_LAST    EV_TYPE_EXTCAP
//---------------------------------------------- Event types -----------------------------------------------





// Event manager states.  Note, some of these states are also used as event type definitions
enum EventManager_States
{
  EM_IDLE=0,                            // No Event
  EM_SUMMARY,
  EM_ENERGYLOG,                         // Demand/Energy Logging Event - normal (use existing EID)
  EM_ENERGYLOG1,
  EM_ENERGYLOG2,
  EM_TRIP,
  EM_TRIP1,
  EM_TRIP2,
  EM_ALARM,
  EM_ALARM1,
  EM_ALARM2,                            // 10
  EM_RTD,
  EM_EXT_CAPT,
  EM_EXT_CAPT1,
  EM_EXT_CAPT2,
  EM_DISTURBANCE,
  EM_DISTURBANCE1,
  EM_FINISH                             // 17
};


// Event Codes for Summary Logs

#define NO_EVENT                  0

// Basic Event
#define PWRUP_RTC_GOOD                        1
#define PWRUP_RTC_BAD                         2
#define STP_DWNLD_MODBUS_RTU                  3
#define STP_DWNLD_CAM                         4
#define STP_DWNLD_MODBUS_TCP                  5
#define STP_DWNLD_DISPLAY                     6
#define STP_DWNLD_USB                         7
#define STP_DWNLD_BLUETOOTH                   8
#define STP_SET_CHANGE_CAM                    9
#define STP_SET_CHANGE_DISPLAY               10
#define STP_SET_CHANGE_MODBUS_RTU            11
#define STP_SET_CHANGE_MODBUS_TCP            12
#define STP_SET_CHANGE_USB                   13
#define STP_SET_CHANGE_GOOSE                 14
#define STP_SET_CHANGE_BLUETOOTH             15
#define STP_FRAME_MISMATCH                   16
#define STP_ERROR                            17
#define ENTER_MAINTENANCE_MODE               18
#define EXIT_MAINTENANCE_MODE                19
#define OPENEDBY_MODBUS_RTU                  20
#define OPENEDBY_CAM                         21
#define OPENEDBY_DISPLAY                     22
#define OPENEDBY_MODBUS_TCP                  23
#define OPENEDBY_USB                         24
#define OPENEDBY_BLUETOOTH                   25
#define OPENEDBY_GOOSE                       26
#define CLOSEDBY_MODBUS_RTU                  27
#define CLOSEDBY_CAM                         28
#define CLOSEDBY_DISPLAY                     29
#define CLOSEDBY_MODBUS_TCP                  30
#define CLOSEDBY_USB                         31
#define CLOSEDBY_BLUETOOTH                   32
#define CLOSEDBY_GOOSE                       33
#define ENTER_TEST_MODE                      34
#define EXIT_TEST_MODE_ABRT                  35
#define TEST_MODE_CMPLT                      36
#define ADMIN_PWD_CHANGED                    37
#define ADMIN_PWD_ENTERED                    38
#define USER_PWD_CHANGED                     39
#define USER_PWD_ENTERED                     40
#define WRONG_PWD_ENTERED                    41
#define TA_COIL_FAIL                         42
#define WATCHDOG_RESTART                     43
#define CAL_CONSTANTS_ERR                    44
#define SETPOINTS_FAULT                      45
#define FRAME_MODULE_FAULT                   46
#define NV_MEMORY_ERROR1                     47
#define NV_MEMORY_ERROR2                     48
#define NV_MEMORY_ERROR3                     49
#define BREAKER_HEALTH                       50
#define AUX_POWER_FAIL                       51
#define BATTERY_FAIL                         52
#define RTC_ERROR                            53
#define WRONG_SENSOR                         54      // *** DAH  THIS IS A MAJOR ALARM IN V3.0.  SHOULD IT BE?
#define DIGITAL_BYPASS                       55      // Override micro comms error
#define OPERATIONS_COUNT                     56
#define MECHANICAL2                          57      // Reserved for health bus - slow opening time
#define DISPLAY_OVERTEMPERATURE              58      // Display turned off due to overtemperature
#define SYSTEM_CHANGE                        59      // System configuration changed (automatically or also by user?)
#define SENSOR_ERROR                         60      // This is detected using the old coil temperature circuit and algorithm
#define ZSI_IN_ACTIVE                        61
#define ZSI_OUT_ACTIVE                       62
#define LDPU_ENTRY                           63      // Also initiate disturbance capture
#define LDPU_NEUTRAL_ENTRY                   64
#define SDPU_ENTRY                           65      // Also initiate disturbance capture

#define ALARM_OV_ENTRY_TEST                  66
#define ALARM_UV_ENTRY_TEST                  67
#define ALARM_HL1_ENTRY_TEST                 68
#define ALARM_HL2_ENTRY_TEST                 69
#define ALARM_OVERTEMP_TEST                  70
#define ALARM_VU_ENTRY_TEST                  71
#define ALARM_CU_ENTRY_TEST                  72
#define ALARM_REV_KW_ENTRY_TEST              73
#define ALARM_REV_KVAR_ENTRY_TEST            74
#define ALARM_PH_LOSS_ENTRY_TEST             75
#define ALARM_OF_ENTRY_TEST                  76
#define ALARM_UF_ENTRY_TEST                  77
#define ALARM_OVERKW_ENTRY_TEST              78
#define ALARM_OVERKVAR_ENTRY_TEST            79
#define ALARM_OVERKVA_ENTRY_TEST             80
#define ALARM_UNDERPF_ENTRY_TEST             81
#define ALARM_GND_FAULT_PRE_TEST             82
  
#define UV_PICKUP                            83
#define OV_PICKUP                            84
#define VU_PICKUP                            85
#define CU_PICKUP                            86
#define REVW_PICKUP                          87
#define REVVAR_PICKUP                        88
#define PL_PICKUP                            89
#define OF_PICKUP                            90
#define UF_PICKUP                            91
#define OW_PICKUP                            92
#define OVAR_PICKUP                          93
#define OVA_PICKUP                           94
#define UPF_PICKUP                           95
#define GF_PICKUP                            96
 
#define SUMMARY_END               GF_PICKUP



// Trip events
#define TRIP_LONG_DELAY                      98
#define TRIP_SHORT_DELAY                     99
#define TRIP_INSTANTANEOUS                  100
#define TRIP_OVERRIDE                       101
#define TRIP_LD_NEUTRAL                     102
#define TRIP_SD_NEUTRAL                     103
#define TRIP_OVERIDE_NEUTRAL                104
#define TRIP_MECHANICAL1                    105
#define TRIP_OVERTEMPERATURE                106
#define TRIP_OTHER                          107

#define TRIP_OVERVOLTAGE                    108
#define TRIP_UNDERVOLTAGE                   109
#define TRIP_VOLTAGE_UNBAL                  110
#define TRIP_CURRENT_UNBAL                  111
#define TRIP_REVERSE_KW                     112
#define TRIP_REVERSE_KVAR                   113
#define TRIP_REVERSE_SEQUENCE               114
#define TRIP_PHASE_LOSS                     115
#define TRIP_OVERFREQUENCY                  116
#define TRIP_UNDERFREQUENCY                 117
#define TRIP_GOOSE                          118
#define TRIP_MAINTANENCE_MODE_ARMS          119
#define TRIP_OVERKW                         120
#define TRIP_OVERKVAR                       121
#define TRIP_OVERKVA                        122
#define TRIP_UNDERPF                        123
#define TRIP_OVERDMNDKW                     124
#define TRIP_OVERDMNDKVAR                   125
#define TRIP_OVERDMNDKVA                    126
#define TRIP_GROUND_FAULT                   127
#define TRIP_MCR                            128

#define TRIPS_END                 TRIP_MCR



// Alarm Events
#define ALARM_MECHANICAL1                   130      // Sneakers
#define ALARM_GND_FAULT_PRE                 131
#define ALARM_GROUND_FAULT                  132
#define ALARM_NEUTRAL_CURRENT               133
#define ALARM_THD_CURRENT                   134
#define ALARM_THD_VOLTAGE                   135
#define ALARM_OVERDMNDKW                    136
#define ALARM_OVERDMNDKVAR                  137
#define ALARM_OVERDMNDKVA                   138
#define THERMAL_MEMORY                      139

#define ALARM_OVERTEMPERATURE_ENTRY         140
#define ALARM_OVERVOLTAGE_ENTRY             141
#define ALARM_UNDERVOLTAGE_ENTRY            142
#define ALARM_VOLTAGE_UNBALANCE_ENTRY       143
#define ALARM_CURRENT_UNBALANCE_ENTRY       144
#define ALARM_REVERSE_KW_ENTRY              145
#define ALARM_REVERSE_KVAR_ENTRY            146
#define ALARM_REVERSE_SEQUENCE              147
#define ALARM_PH_LOSS_ENTRY                 148
#define ALARM_OVERFREQUENCY_ENTRY           149
#define ALARM_UNDERFREQUENCY_ENTRY          150
#define ALARM_OVERKW_ENTRY                  151
#define ALARM_OVERKVAR_ENTRY                152
#define ALARM_OVERKVA_ENTRY                 153
#define ALARM_UNDERPF_ENTRY                 154
#define ALARM_HIGHLOAD1_ENTRY               155
#define ALARM_HIGHLOAD2_ENTRY               156
#define ALARM_GOOSE_CAPTURE_ENTRY           157

#define ALARMS_END                ALARM_GOOSE_CAPTURE_ENTRY
 


// Extended Capture events
#define EXTCAP_GOOSE_CAPTURE_ENTRY          180
#define EXTCAP_OV_ENTRY                     181
#define EXTCAP_UV_ENTRY                     182
#define EXTCAP_HIGHLOAD1_ENTRY              183
#define EXTCAP_HIGHLOAD2_ENTRY              184     // *** DAH  MAY WANT TO ADD SOME BUFFER SPACE HERE FOR MORE CODES
#define EXTCAP_GOOSE_CAPTURE_ENTRY_NO_SNAP  185
#define EXTCAP_OV_ENTRY_NO_SNAP             186
#define EXTCAP_UV_ENTRY_NO_SNAP             187
#define EXTCAP_HIGHLOAD1_ENTRY_NO_SNAP      188
#define EXTCAP_HIGHLOAD2_ENTRY_NO_SNAP      189
#define EXTCAP_SNAP_NOSNAP_DIFF             5       // This must be maintained - ExtendedCapture() uses this

#define EXTCAP_END                EXTCAP_HIGHLOAD2_ENTRY_NO_SNAP


// Disturbance events
#define LDPU_EXIT                           200
#define HIGH_TEMPERATURE_EXIT               201
#define OVERVOLTAGE_EXIT                    202
#define UNDERVOLTAGE_EXIT                   203
#define VOLTAGE_UNBAL_EXIT                  204
#define CURRENT_UNBAL_EXIT                  205
#define REVERSE_KW_EXIT                     206
#define REVERSE_KVAR_EXIT                   207
#define PHASE_LOSS_EXIT                     208
#define OVERFREQUENCY_EXIT                  209
#define UNDERFREQUENCY_EXIT                 210
#define OVERKW_EXIT                         211
#define OVERKVAR_EXIT                       212
#define OVERKVA_EXIT                        213
#define UNDERPF_EXIT                        214
#define HIGHLOAD1_EXIT                      215
#define HIGHLOAD2_EXIT                      216
#define GND_FAULT_EXIT                      217
#define GOOSE_CAPTURE_EXIT                  218
#define SDPU_EXIT                           219

#define DIST_EVCODE_START                   LDPU_EXIT
#define DISTCAP_END                         SDPU_EXIT






#define LONG_DELAY_PICKUP_TM   242                // From NRX1150, PXR25 does not appear to have LD pickup alarm in test mode       In PXR25 only generate Alarm snapshot - no summary, no waveform
#define GROUND_FAULT_TM        243                // From NRX1150, PXR25 does not appear to have ground fault in test mode alarm    In PXR25 only generate Alarm snapshot - no summary, no waveform



// Summary real time data
#define TRIP_TEST                         235
#define SD_HIGH_LOAD_ALARM_CAPT           236

#define DEMAND_EVENT                      245

#define DISTURBANCE_EXIT                  246



#define SUMMARY_RTD               SD_HIGH_LOAD_ALARM_CAPT

#define SUMMARY_DEMAND            DEMAND_EVENT

#define SUMMARY_DISTURBANCE       DISTURBANCE_EXIT

#define MAXEVENTCODE              SUMMARY_DISTURBANCE

#define DISTURBANCEONEMIN         3599

//------------------------------------------------------------------------------------------------------------
//    Constants
//------------------------------------------------------------------------------------------------------------

//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

// New events structure
struct NEW_EVENT_PARAMS
{
  uint32_t p_one;
  uint32_t p_two;
};

union NEWEVENTPARAM
{
  struct SYSTICK_TIME NewTime;              // For time adjustment events
  struct NEW_EVENT_PARAMS param;            // For trip and alarm events
};
 
struct NEWEVENTFIFO
{
  struct INTERNAL_TIME TS;                  // Time Stamp
  union NEWEVENTPARAM Aux;                  // Data parameter
  uint32_t EID;                             // Event ID
  uint16_t Code;                            // Event Code (0..255 - see definitions above)
};

struct EV_SUM
{
  uint16_t NextEvntNdx;
  uint16_t Num_Events;
};
struct EV_ADD
{
  uint8_t NextEvntNdx;
  uint8_t Num_Events;
};
struct EV_DMND
{
  uint16_t NextDmndAdd;
  uint16_t Num_Entries;
};

struct DATE_TIME
{
  uint16_t Year;
  uint8_t Month;
  uint8_t Day;
  uint8_t Hour;
  uint8_t Minute;
  uint8_t Second;
};

struct TIMEADJ
{
  uint8_t SourceChange;                     // where the time adj happened
  uint8_t Spare;                            // even boundaries
  struct INTERNAL_TIME NewTime;             // adjusted time;
};

// Extended Capture Flags
#define EC_GCE               0x01           // 1
#define EC_OV                0x02           // 2
#define EC_UV                0x04           // 3
#define EC_HL1               0x08           // 4
#define EC_HL2               0x10           // 5

#define EXTCAP_NUM           5

enum ExtCapStatus
{
  EC_IDLE = 0,
  EC_START,
  EC_IN_PROGRESS,
  EC_CANCEL,
  EC_END
};


enum ExtCapTypeOfRecord
{
  OneCycle = 0,
  TwoHundred
};

struct EXCTAP_STATE
{
  uint8_t State;
  uint8_t Cause;
  uint16_t TypeOfRecord;
  uint16_t OneCycleCounter;
  uint16_t TwoHundredCounter;
  uint16_t OneCycleNumVals;
  uint16_t TwoHundredNumVals;
  uint32_t EID;
};


struct EXTCAP_SNAPSHOT_METER
{
  // Currents
  float Ia_Rms;           //phase A current
  float Ib_Rms;           //phase B current
  float Ic_Rms;           //phase C current
  float In_Rms;           //phase N current
  float Ig_Rms;           //ground fault current

  //Line-to-Neutral Voltages
  float Van1_Rms;          //phase A to Neutral
  float Vbn1_Rms;          //phase A to Neutral
  float Vcn1_Rms;          //phase A to Neutral

  //Line-to-Neutral Voltages
  float Van2_Rms;          //phase A to Neutral
  float Vbn2_Rms;          //phase A to Neutral
  float Vcn2_Rms;          //phase A to Neutral
};

enum DisturbanceStatus
{
  DS_IDLE = 0,
  DS_START,
  DS_IN_PROGRESS,
  DS_END,
  DS_WAIT,
  DS_CANCEL
};

// Disturbance Codes and Flags
//   Note1: the causes shown below MUST align with aDistCause[] in Events.c!
//   Note2: the first 17 codes/flags (from DS_LD_PICKUP thru DS_HIGHLOAD_2) are set and cleared in the
//          foreground, and they are processed in the foreground (no issues).
//          However, the the subroutines that generate the disturbance events for the last three codes/flags
//          (DS_GOOSE thru DS_SD_PICKUP) are in the sampling interrupt, so separate byte flags are used.
//          This ensures there is no data tearing and interrupts do not need to be disabled when setting and
//          clearing the flags
#define DS_LD_PICKUP        0x00                                // Long Delay Pickup
#define DS_LD               (((uint32_t)1) << DS_LD_PICKUP)
#define DS_HIGH_TEMP        0x01                                // High Temperature
#define DS_HT               (((uint32_t)1) << DS_HIGH_TEMP)
#define DS_OVER_VOLT        0x02                                // Overvoltage
#define DS_OV               (((uint32_t)1) << DS_OVER_VOLT)
#define DS_UNDER_VOLT       0x03                                // Undervoltage
#define DS_UV               (((uint32_t)1) << DS_UNDER_VOLT)
#define DS_VOLT_UNBAL       0x04                                // Voltage unbalance
#define DS_VU               (((uint32_t)1) << DS_VOLT_UNBAL)
#define DS_CUR_UNBAL        0x05                                // Current unbalance
#define DS_CU               (((uint32_t)1) << DS_CUR_UNBAL)
#define DS_REV_W            0x06                                // Reverse real power
#define DS_RKW              (((uint32_t)1) << DS_REV_W)
#define DS_REV_VAR          0x07                                // Reverse reactive power
#define DS_RKVAR            (((uint32_t)1) << DS_REV_VAR)
#define DS_PHASE_LOSS       0x08                                // Phase loss
#define DS_PL               (((uint32_t)1) << DS_PHASE_LOSS)
#define DS_OVER_FREQ        0x09                                // Overfrequency
#define DS_OF               (((uint32_t)1) << DS_OVER_FREQ)
#define DS_UNDER_FREQ       0x0A                                // Underfrequency
#define DS_UF               (((uint32_t)1) << DS_UNDER_FREQ)
#define DS_OVER_W           0x0B                                // Overpower (real)
#define DS_OW               (((uint32_t)1) << DS_OVER_W)
#define DS_OVER_VAR         0x0C                                // Overpower (reactive)
#define DS_OVAR             (((uint32_t)1) << DS_OVER_VAR)
#define DS_OVER_VA          0x0D                                // Overpower (apparent)
#define DS_OVA              (((uint32_t)1) << DS_OVER_VA)
#define DS_UNDER_PF         0x0E                                // Under PF
#define DS_UPF              (((uint32_t)1) << DS_UNDER_PF)
#define DS_HIGHLOAD_1       0x0F                                // High Load 1
#define DS_HL1              (((uint32_t)1) << DS_HIGHLOAD_1)
#define DS_HIGHLOAD_2       0x10                                // High Load 2
#define DS_HL2              (((uint32_t)1) << DS_HIGHLOAD_2)
#define DS_GOOSE            0x11                                // GOOSE capture
#define DS_GSE              (((uint32_t)1) << DS_GOOSE)
#define DS_GND_FAULT        0x12                                // Ground Fault Protection
#define DS_GF               (((uint32_t)1) << DS_GND_FAULT)
#define DS_SD_PICKUP        0x13                                // Short Delay Pickup
#define DS_SD               (((uint32_t)1) << DS_SD_PICKUP)

#define DS_LAST             DS_SD_PICKUP

#define DIST_NUM            DS_SD_PICKUP + 1

#define DS_ALL              0xFFFFF



// Note, values are arranged so that they align with the values written into FRAM when a disturbance capture
//   event is stored.  There should be no padding bytes, and the size, from sEntryTS through EIDofStartEvnt
//   MUST be 32, so tht the total number of bytes written to FRAM is 44 (EID + Event Timestamp adds 12 more
//   bytes).  Reference DIST_EVENT_SIZE in FRAM_Flash_def.h
struct DIST_VALUES
{
  struct INTERNAL_TIME sEntryTS;
  float Duration;
  uint16_t ValueCode;               // Value of interest *** DAH  must have separate codes for OV and UV
  uint16_t Spare;                   // Value of interest *** DAH  must have separate codes for OV and UV
  float MaxMinVal;
  float AvgVal;
  float ThermalCap;
  uint32_t EIDofStartEvnt;          // EID of the event that initiated the capture
  float *pDistValue;
  uint16_t SubCounter;
  uint8_t State;
  uint8_t Cause;
  uint32_t MinsCounter;
  float SubAvg;
};

