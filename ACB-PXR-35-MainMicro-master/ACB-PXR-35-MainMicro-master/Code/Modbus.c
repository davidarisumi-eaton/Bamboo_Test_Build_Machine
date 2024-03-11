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
//  MODULE NAME:        Modbus.c
//
//  MECHANICS:          Program module containing the Modbus interface subroutines
//
//  TARGET HARDWARE:    PXR35 Rev 4 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.32   190726  DAH - File Creation
//   0.36   200210  DAH - Display_ext.h renamed to DispComm_ext.h
//                      - Added CalcCRC() (moved from DispComm.c)
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added include of RealTime_def.h and deleted include of Iod_def.h
//   0.49   220131  DAH - Revisions to get DMA working for Modbus transmissions
//                          - struct ModB moved from CCM RAM to sram2
//                          - revised ModB_SlaveComm() to check input message length before computing CRC
//                          - revised ProcFC0304Msg() to hard-code a response for address 3000
//   0.50   220203  DAH - Added code to process Read requests for real-time values
//   0.51   220304  DAH - Added more code to process Read requests for real-time values
//                          - Per-phase current and voltage unbalance values were added
//                      - Min/max current values have switched from 200msec values to one-cycle values
//                          - MODB_OBJECT_ADDR_GR12[] and MODB_OBJECT_ADDR_GR16[] revised
//                            (Cur200msFltr_max.Ix and Cur200msFltr_min.Ix replaced with CurOneCyc_max.Ix
//                            and CurOneCyc_min.Ix)
//                      - Current unbalance value changed from 200msec value to one-cycle value
//                          - In MODB_OBJECT_ADDR_GR12[], replaced Cur200msFltrUnbal with CurOneCycUnbal
//                      - Registers 4698 - 4713 were changed to not supported
//                      - Min/max voltage values changed from 200msec values to one-cycle values
//                          - In MODB_OBJECT_ADDR_GR12[] and MODB_OBJECT_ADDR_GR16[], replaced
//                            VolAFE200msFltr_min.Vxx and VolAFE200msFltr_max.Vxx with VolAFEOneCyc_min.Vxx
//                            VolAFEOneCyc_max.Vxx
//                          - In MODB_OBJECT_ADDR_GR15[] and MODB_OBJECT_ADDR_GR16[], replaced
//                            VolADC200ms_min.Vxx and VolADC200ms_max.Vxx with VolADCOneCyc_min.Vxx
//                            VolADCOneCyc_max.Vxx
//                      - Voltage unbalance value changed from 200msec value to one-cycle value
//                          - In MODB_OBJECT_ADDR_GR12[], replaced VolAFE200msFltrUnbal with
//                            VolAFEOneCycUnbal
//   0.52   220309  DAH - Added real-time data set 9, 5-minute values
//                          - ProcFC0304Msg() revised
//                      - In MODB_OBJECT_ADDR_GR18[], replaced EngyDmnd[1].TotSumWHr with TotWHr and
//                        replaced EngyDmnd[1].TotSumVarHr with NetWHr
//                      - Added net real energy (WHr) and total reactive energy (VarHr)
//                      - Corrected bug in energy conversions
//                          - Format codes for energies changed
//                          - Modb_FormatStoreVal() revised
//                      - Maximum temperature removed from real-time data set 2, and temperature moved to
//                        address 4764/6300, to align with the PXR25
//                          - MODB_GROUP_START_ADD[], MODB_GROUP_END_ADD[], MODB_OBJECT_ADDR_GR11[],
//                            MODB_OBJECT_CONV_GR11[] revised
//   0.53   220325  DAH - Added min/max THD values to the Modbus map by inserting them after the voltage
//                        unbalance min/max values and shifting the remaining values down.  This keeps the
//                        number of groups the same, and just changes the starting and ending addresses of
//                        the groups
//                      - Corrected bug in Modb_FormatStoreVal() in computing the word order
//                      - Added Harmonics
//                          - Added MB_Harmonics_Selection and MODB_OBJECT_ADDR_GR14[]
//                          - Revised Modb_VarInit() to initialize MB_Harmonics_Selection
//                          - Revised ProcFC0304Msg() to process groups 14 and 23 (harmonics) of the Modbus
//                            map
//                          - Revised ProcFC06Msg() to handle writes to the harmonics selection register
//                            (MB_Harmonics_Selection)
//   0.54   220330  DAH - CurOneCycUnbal renamed to CurUnbalTot in MODB_OBJECT_ADDR_GR12[]
//                      - VolAFEOneCycUnbal renamed to VolUnbalTot in MODB_OBJECT_ADDR_GR12[]
//                      - Added PKE (Thermal) Overload Warning to the modbus map by adding LD_Bucket to
//                        MODB_OBJECT_ADDR_GR12[] and MODB_OBJECT_CONV_GR12[] (it was not supported)
//                      - Added max current total unbalance, max voltage total unbalance, and max Thermal
//                        Overload Warning (CurUnbalTotMax, VolUnbalTotMax, LD_BucketMax) to the Modbus map
//                        by inserting them after the THD min/max values and shifting the remaining values
//                        down.  This keeps the number of groups the same, and just changes the starting and
//                        ending addresses of the groups
//   0.55   220420  DAH - Added short-delay overload values to the Modbus map by inserting the max value
//                        after the PKE (Thermal) Overload max value and timestamp, shifting the 5-minute
//                        values down, and moving the Net Real Power and Total Reactive Power past the
//                        Remote Control registers.  This keeps the number of groups the same, and just
//                        changes the starting and ending addresses of the groups
//                      - Added StatusCode for address 1200/1800 in MODB_OBJECT_ADDR_GR10[]
//                          - Added format codes 6 and 26 (U32 --> U32) to Modb_FormatStoreVal()
//   0.56   220526  DAH - Fixed minor bug setting function code for NAK response in ProcFC06Msg()
//                      - Added ProcFC16Msg() to support harmonics freeze/thaw commands
//   0.58   220829  DAH - Revisions to support changes to the Modbus map.  The map was revised to add the
//                        Time of Last Reset to the min/max values.  These registers were inserted to the
//                        end of the min/max group, and the subsequent values were slid down.
//                          - Modbus groups (MODB_GROUP_START_ADD[], MODB_GROUP_END_ADD[],
//                            MODB_NUM_REGS_PER_DATA_OBJECT[]) were modified.  The addresses were changed to
//                            reflect the new map.  Additional groups were added, because each min/max set
//                            must be a separate group, ending with the Time of Last Reset.
//                          - ProcFC06Msg(), ProcFC16Msg() revised (states changed to reflect the new
//                            groups)
//    53    230725  VD  - Added support for setpoint read and writes
//                          - ProcFC0304Msg(), ProcFC06Msg(), ProcFC16Msg() revised
//                      - Fixed bug in ProcFC16Msg().  Received message length check was wrong
//   108    231108  DAH - Added include of Flags_def.h for BITxx definitions
//   116    231129  MAG - Modified communications parameter handling including:
//                        - Replaced MB_Float_Order, MB_Fixed_Order, and MB_Unforgiving with their
//                          Setpoints2.stp.x values to eliminate added variables and code.
//                        - Added reset of Modbus UART when communications parameters are changed and
//                          modified Modb_VarInit() to provide power-up initialization of all variables
//                          while retaining some variables during in-operation reset of Modbus UART
//                        - Added handling of "Group 8" communication parameters
//                      - Fixed an indexing bug in FC16 setpoint handling
//                      - Added more handling of setpoint set 0xFF, active set
//                      - Corrected FC16 error code return value from 0x96 to 0x90
//                      - Fixed setting of RelayFlagStp for setpoint group 12
//   129    231213  MAG - Wait until transmit is complete before handling ModB.Reset_Req to more gracefully
//                        handle changes in Modbus communications parameters
//                      - Increment offset to fix bug reading Modbus parameters in ProcFC0304Msg()
//                      - Consolidated handling of Group0/Group1 read-only setpoints into GetSetpoints()
//                      - Fixed several calls to Get_Setpoints that used 0xFF instead of SetpActiveSet
//                      - Updated calls to Verify_Setpoints() to pass setpoints buffer pointer
//                      - Fixed incorrect num_to_add when calling Modb_CheckRegAddress in ProcFC16Msg()
//                      - Fixed incorrect math testing for exceeding MODB_GROUP_END_ADD in ProcFC16Msg()
//                      - Fixed faddr calculation for Groups 10, 12, and new Group 13
//                      - Tweaked Maintenance Mode handling to match other occurrences
//   149    240131  DAH - Modified Modb_Save_Setpoints() to add event insertion
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
#include "RealTime_def.h"
#include "Modbus_def.h"
#include "Meter_def.h"
#include "Demand_def.h"
#include "Iod_def.h"
#include "Setpnt_def.h"
#include "Prot_def.h"
#include "Flags_def.h"
#include "Intr_def.h"
#include "Events_def.h"



//
//      Local Definitions used in this module...
//
enum ModB_SlaveComm_States
{
  MBS_IDLE, MBS_PROCREQ, MBS_START_TX, MBS_TX, MBS_DONE
};

uint16_t ModB_CurSetGrp;



//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Init_ext.h"
#include "DispComm_ext.h"
#include "Meter_ext.h"
#include "Demand_ext.h"
#include "Iod_ext.h"
#include "RealTime_ext.h"
#include "Prot_ext.h"
#include "Setpnt_ext.h"
#include "Intr_ext.h"
#include "Events_ext.h"



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Modb_VarInit(uint8_t init_all);
void ModB_SlaveComm(void);




//      Local Function Prototypes (These functions are called only within this module)
//
void ProcFC0102Msg(uint8_t length, uint8_t fc);
void ProcFC0304Msg(uint8_t length, uint8_t fc);
void ProcFC06Msg(uint8_t length);
void ProcFC16Msg(uint8_t length);
void Modb_CheckRegAddress(uint8_t *group, uint16_t *offset, uint8_t *num_bad, uint8_t *num_to_add,
                                    uint16_t start_start_reg_add, uint8_t num_reg);
void Modb_FormatStoreVal(uint8_t format_code, void *val_in, uint8_t *val_out);
uint16_t CalcCRC(uint8_t *msg_ptr, uint16_t len);
void Modb_Save_Setpoints(uint8_t CurSetpSet, uint8_t SetpGrpNum);
uint8_t Modb_Remote_Control(uint8_t control_group, uint16_t sub_code);


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct MODB_PORT ModB @".sram2";




//------------------------------------------------------------------------------------------------------------
//                   Global Constants used in this module and other modules
//------------------------------------------------------------------------------------------------------------
//



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
float mb_reg_not_supported;
uint16_t MB_Harmonics_Selection;


//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
// TODO:  The following is for initial debugging, remove when actual Password storage it completed
#define DEBUG_PASSWORD 1234
//
//------------------------------------------------------------------------------------------------------------
//                   PXR35 Modbus Address Map and Decoding
//------------------------------------------------------------------------------------------------------------
//
// The Modbus Map is defined in document 66A7918


// Function Code 1
// At the initial writing of this function there are no Function Code 1 registers suppported.
//  const uint16_t MODB_FC1_START_ADD = ???? future use;
//  const uint16_t MODB_FC1_END_ADD = ???? future use;


// Function Code 2
// supported registers:
// 3E8 Status - Breaker Closed
// 3E9 Status - Unacknowledged trip
// 3EA Status - Unacknowledged alarm
// 3EB undefined - Always 0
// 3EC Status - Maintenance mode
// 3ED Status - Test mode
// 3EE undefined - Always 0
// 3EF undefined - Always 0
// 3F0 Status - Phase rotation is ABC
// 3F1 Status - Long delay pickup
// 3F2 Status - Zone interlock active
// 3F3 undefined - Always 0
// 3F4 Status - "Ground" is source gnd
// 3F5 Status - Breaker connected - Not implemented - Always 0
// 3F6 Status - Spring charged - Not implemented - Always 0
// 3F7 undefined - Always 0
// 3F8 Validity - Breaker Closed - Always 1
// 3F9 Validity - Unacknowledged trip - Always 1
// 3FA Validity - Unacknowledged alarm - Always 1
// 3FB undefined - Always 0
// 3FC Validity - Maintenance mode - Always 1
// 3FD Validity - Test mode - Always 1
// 3FE undefined - Always 0
// 3FF undefined - Always 0
// 400 Validity - Phase rotation is ABC - Always 1
// 401 Validity - Long delay pickup - Always 1
// 402 Validity - Zone interlock active - Always 1
// 403 undefined
// 404 Validity - "Ground" is source gnd - Always 1
// 405 Validity - Breaker connected - Not implemented - Always 0
// 406 Validity - Spring charged - Not implemented - Always 0
// 407 undefined - Always 0
  uint16_t const MODB_FC2_START_ADD = 1000;
  uint16_t const MODB_FC2_END_ADD = 1031;
  

// This mapping applies to register addresses for Function Codes 3, 4, 6, and 16
// Note, in order for the mapping algorithm to work properly, all data objects in the group must use the
// same number of registers!  They do not have to be the same type (e.g., U32 vs S32), but they MUST be the
// same size!!
// 
// Address Range    Address Range    Group      Description
//  (decimal)          (hex)
// 1000 - 1199      x03E8 - x04AF      0        Preset mapping assignment registers
// 1200 - 1999      x04B0 - x07CF      1        Preset mapping data registers
// 2000 - 2002      x07D0 - x07D2      2        Modbus register configuration registers
// 2900 - 2902      x0B54 - x0B56      3        Remote control registers
// 2910 - 2911      x0B5E - x0B5F      4        Supervisory broadcast registers
// 2920 - 2927      x0B68 - x0B6F      5        Time (old format)
// 2940 - 2943      x0B7C - x0B7F      6        Time (new format)
// 2999 - 3199      x0BB7 - x0C7F      7        Setpoints
// 3999 - 4002      x0F9F - x0FA2      8        Modbus communications configuration
// 4496 - 4607      x1190 - x11FF      9        Device Info
// 4608 - 4721      x1200 - x1271     10        Floating point real-time data set 1 - aligns with Group 21
// 4764 - 4765      x129A - x129D     11        Floating point real-time data set 2 - aligns with Group 23
// 4796 - 4951      x12BC - x1357     12        Floating point real-time data set 3 - aligns with Group 25
// 4954 - 5015      x135A - x1397     13        Miscellaneous registers Group 1
// 5024 - 5105      x13A0 - x13F1     14        Floating point Harmonics -            aligns with Group 27
// 5600 - 5781      x15E0 - x1695     15        Floating point real-time data set 4 - aligns with Group 28
// 5782 - 5845      x1696 - x16D5     16        Floating point real-time data set 5 - aligns with Group 29
// 5846 - 5885      x16D6 - x16FD     17        Floating point real-time data set 6 - aligns with Group 30
// 5886 - 5925      x16FE - x1725     18        Floating point real-time data set 7 - aligns with Group 31
// 5926 - 5965      x1726 - x174D     19        Floating point real-time data set 8 - aligns with Group 32
// 5966 - 6005      x174E - x1775     20        Floating point real-time data set 9 - aligns with Group 33
// 6144 - 6257      x1800 - x1871     21        Fixed point real-time data set 1 - aligns with Group 10
// 6258 - 6271      x1872 - x187F     22        Fixed point 32-bit energy values (kWHr, kVarhr, kVA)
// 6300 - 6301      x189C - x189D     23        Fixed point real-time data set 2 - aligns with Group 11
// 6304 - 6331      x18A0 - x18BB     24        Fixed point 64-bit energy values (WHr, Varhr, VA)
// 6332 - 6487      x18BC - x1957     25        Fixed point real-time data set 3 - aligns with Group 12
// 6490 - 6551      x195A - x1997     26        Miscellaneous registers Group 1
// 6560 - 6641      x19A0 - x19F1     27        Fixed point Harmonics - aligns with Group 14
// 7136 - 7317      x1BE0 - x1C95     28        Fixed point real-time data set 4 - aligns with Group 15
// 7318 - 7381      x1C96 - x1CD5     29        Fixed point real-time data set 5 - aligns with Group 16
// 7382 - 7421      x1CD6 - x1CFD     30        Fixed point real-time data set 6 - aligns with Group 17
// 7422 - 7461      x1CFE - x1D25     31        Fixed point real-time data set 7 - aligns with Group 18
// 7462 - 7501      x1D26 - x1D4D     32        Fixed point real-time data set 8 - aligns with Group 19
// 7502 - 7541      x1D4E - x1D75     33        Fixed point real-time data set 9 - aligns with Group 20
// 8192 - 9660      x2000 - x25BC     34        Events
// 20480 - 20679    x5000 - x50E7     35        Preset mapping assignment registers
// 20736 - 21535    x5100 - x541F     36        Preset mapping data registers
// 24576 - 24627    x6000 - x6033     37        Floating point real-time data set 10 - aligns with Group 55
// 24628 - 24679    x6034 - x6067     38        Floating point real-time data set 11 - aligns with Group 56
// 24680 - 24707    x6068 - x6083     39        Floating point real-time data set 12 - aligns with Group 57
// 24708 - 24759    x6084 - x60B7     40        Floating point real-time data set 13 - aligns with Group 58
// 24760 - 24781    x60B8 - x60CD     41        Floating point real-time data set 14 - aligns with Group 59
// 24782 - 24807    x60CE - x60E7     42        Floating point real-time data set 15 - aligns with Group 60
// 24808 - 24847    x60E8 - x610F     43        Floating point real-time data set 16 - aligns with Group 61
// 24848 - 24875    x6110 - x612B     44        Floating point real-time data set 17 - aligns with Group 62
// 24876 - 24909    x612C - x614D     45        Floating point real-time data set 18 - aligns with Group 63
// 24910 - 24961    x614E - x6181     46        Floating point real-time data set 19 - aligns with Group 64
// 24962 - 25001    x6182 - x61A9     47        Floating point real-time data set 20 - aligns with Group 65
// 25002 - 25041    x61AA - x61D1     48        Floating point real-time data set 21 - aligns with Group 66
// 25042 - 25051    x61D2 - x61DB     49        Floating point real-time data set 22 - aligns with Group 67
// 25052 - 25061    x61DC - x61E5     50        Floating point real-time data set 23 - aligns with Group 68
// 25088 - 25091    x6200 - x6203     51        Remote control registers
// 25344 - 25346    x6300 - x6302     52        Modbus register configuration registers
// 25856 - 26003    x6500 - x6593     53        Floating point real-time data set 24 - aligns with Group 71
// 26004 - 26075    x6594 - x65DB     54        Floating point real-time data set 25 - aligns with Group 72
// 49152 - 49203    xC000 - xC033     55        Fixed point real-time data set 10 - aligns with Group 37
// 49204 - 49255    xC034 - xC067     56        Fixed point real-time data set 11 - aligns with Group 38
// 49256 - 49283    xC068 - xC083     57        Fixed point real-time data set 12 - aligns with Group 39
// 49284 - 49335    xC084 - xC0B7     58        Fixed point real-time data set 13 - aligns with Group 40
// 49336 - 49357    xC0B8 - xC0CD     59        Fixed point real-time data set 14 - aligns with Group 41
// 49358 - 49383    xC0CE - xC0E7     60        Fixed point real-time data set 15 - aligns with Group 42
// 49384 - 49423    xC0E8 - xC10F     61        Fixed point real-time data set 16 - aligns with Group 43
// 49424 - 49451    xC110 - xC12B     62        Fixed point real-time data set 17 - aligns with Group 44
// 49452 - 49485    xC12C - xC14D     63        Fixed point real-time data set 18 - aligns with Group 45
// 49486 - 49537    xC14E - xC181     64        Fixed point real-time data set 19 - aligns with Group 46
// 49538 - 49577    xC182 - xC1A9     65        Fixed point real-time data set 20 - aligns with Group 47
// 49578 - 49617    xC1AA - xC1D1     66        Fixed point real-time data set 21 - aligns with Group 48
// 49618 - 49627    xC1D2 - xC1DB     67        Fixed point real-time data set 22 - aligns with Group 49
// 49628 - 49637    xC1DC - xC1E5     68        Fixed point real-time data set 23 - aligns with Group 50 
// 49680 - 49683    xC210 - xC213     69        Fixed point 32-bit energy values (kWHr, kVarhr)
// 49684 - 49691    xC214 - xC21B     70        Fixed point 64-bit energy values (WHr, Varhr)
// 50432 - 50579    xC500 - xC593     71        Fixed point real-time data set 24 - aligns with Group 53
// 50580 - 50651    xC594 - xC5DB     72        Fixed point real-time data set 25 - aligns with Group 54




//
const uint16_t MODB_GROUP_START_ADD[] =
{
  1000,  1200,  2000,  2900,  2910,  2920,  2940,  2999,  3999,  4496,  4608,  4764,  4796,  4954,  5024,
  5600,  5782,  5846,  5886,  5926,  5966,  6144,  6258,  6300,  6304,  6332,  6490,  6560,  7136,  7318,
  7382,  7422,  7462,  7502,  8192,  20480, 20736, 24576, 24628, 24680, 24708, 24760, 24782, 24808, 24848,
  24876, 24910, 24962, 25002, 25042, 25052, 25088, 25344, 25856, 26004, 49152, 49204, 49256, 49284, 49336,
  49358, 49384, 49424, 49452, 49486, 49538, 49578, 49618, 49628, 49680, 49684, 50432, 50580
};

const uint16_t MODB_GROUP_END_ADD[] =
{
  1199,  1999,  2002,  2902,  2911,  2927,  2943,  3199,  4002,  4607,  4721,  4765,  4951,  5015,  5105,
  5781,  5845,  5885,  5925,  5965,  6005,  6257,  6271,  6301,  6331,  6487,  6551,  6641,  7317,  7381,
  7421,  7461,  7501,  7541,  9660,  20679, 21535, 24627, 24679, 24707, 24759, 24781, 24807, 24847, 24875,
  24909, 24961, 25001, 25041, 25051, 25061, 25091, 25346, 26003, 26075, 49203, 49255, 49283, 49335, 49357,
  49383, 49423, 49451, 49485, 49537, 49577, 49617, 49627, 49637, 49683, 49691, 50579, 50651
};

const uint8_t MODB_NUM_REGS_PER_DATA_OBJECT[] =
{
     1,     1,     1,     1,     1,     1,     1,     1,     1,     1,     2,     2,     2,     2,     2,
     2,     6,     6,     6,     6,     6,     2,     2,     2,     4,     2,     2,     2,     2,     6,
     6,     6,     6,     6,     2,     2,     2,     6,     6,     6,     6,     6,     2,     6,     6,
     6,     6,     6,     6,     6,     6,     2,     2,     6,     2,     6,     6,     6,     6,     6,
     2,     6,     6,     6,     6,     6,     6,     6,     6,     2,     4,     6,     2
};

#define MODB_NUM_GROUPS (sizeof(MODB_GROUP_END_ADD)/2)

// Modbus Real Time Data Objects set 1 Data Address Table
//   This handles both Group 10 and Group 21 above
void * const MODB_OBJECT_ADDR_GR10[] =
{                                                           // Modbus Float Addr/Modbus Fixed Addr (hex)
  &StatusCode,                                              // 1200/1800 - status presently not supported
  &Cur200msFltr.Ia,             &Cur200msFltr.Ib,           // 1202/1802, 1204/1804
  &Cur200msFltr.Ic,             &Cur200msFltrIg,            // 1206/1806, 1208/1808
  &Cur200msFltr.In,             &Cur200msIavg,              // 120A/180A, 120C/180C
  &VolAFE200msFltr.Vab,         &VolAFE200msFltr.Vbc,       // 120E/180E, 1210/1810
  &VolAFE200msFltr.Vca,         &VolAFE200msFltrVllavg,     // 1212/1812, 1214/1814
  &VolAFE200msFltr.Van,         &VolAFE200msFltr.Vbn,       // 1216/1816, 1218/1818
  &VolAFE200msFltr.Vcn,         &VolAFE200msFltrVlnavg,     // 121A/181A, 121C/181C
  &mb_reg_not_supported,                                    // 121E/181E  Vng not supported
  &IDmnd.IaMax,                 &IDmnd.IbMax,               // 1220/1820, 1222/1822
  &IDmnd.IcMax,                                             // 1224/1824
  &mb_reg_not_supported,                                    // 1226/1826  Ig demand not supported
  &IDmnd.InMax,                 &Pwr200msec.Ptot,           // 1228/1828, 122A/182A
  &Pwr200msec.Rtot,             &Pwr200msecApp.Apptot,      // 122C/182C, 122E/182E
  &PF.Disp[3],                  &PF.App[3],                 // 1230/1830, 1232/1832
  &FreqLoad.FreqVal,                                        // 1234/1834
  &mb_reg_not_supported,                                    // 1236/1836  Unused
  &IDmnd.IaMin,                 &IDmnd.IbMin,               // 1238/1838, 123A/183A
  &IDmnd.IcMin,                                             // 123C/183C
  &mb_reg_not_supported,                                    // 123E/183E  Ig demand not supported
  &IDmnd.InMin,                 &FreqLine.FreqVal,          // 1240/1840, 1242/1842
  &FreqLine.FreqVal,            &FreqLine.MinFreqVal,       // 1244/1844, 1246/1846
  &FreqLine.MaxFreqVal,                                     // 1248/1848
  &mb_reg_not_supported,                                    // 124A/184A  Unused
  &PF.Disp[0],                  &PF.Disp[1],                // 124C/184C, 124E/184E
  &PF.Disp[2],                  &PF.App[0],                 // 1250/1850, 1252/1852
  &PF.App[1],                   &PF.App[2],                 // 1254/1854, 1256/1856
  &PDmnd.TotWMax,               &mb_reg_not_supported,      // 1258/1858, 125A/185A
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 125C/185C, 125E/185E
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1260/1860, 1262/1862
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1264/1864, 1266/1866
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1268/1868, 126A/186A
  &mb_reg_not_supported,                                    // 126C/186C
  &mb_reg_not_supported,                                    // 126E/186E  Product ID presently not supported
  &FreqLoad.FreqVal                                         // 1270/1870
};

// Modbus Real-Time Data Objects set 1 Conversion Type Table
//   Note, the table is for Fixed Point addresses, but also handles floating point addresses.
//      Fixed Point Address Groups              Floating Point Address Groups
//                 21                                         10
//                 22  (Energy)
//                 23                                         11
//                 24  (Energy)
//                 25                                         12
//                 27                                         14
//                 28                                         15
//                 29                                         16
//                 30                                         17
//                 31                                         18
//                 32                                         19
//                 33                                         20
//                 55                                         37
//                 56                                         38
//                 57                                         39
//                 58                                         40
//                 59                                         41
//                 60                                         42
//                 61                                         43
//                 62                                         44
//                 63                                         45
//                 64                                         46
//                 65                                         47
//                 66                                         48
//                 67                                         49
//                 68                                         50
//                 69  (Energy)
//                 70  (Energy)
//                 71                                         53
//                 72                                         54

//  For all objects except energy and timestamps, float conversion = (fixed conversion + 20).  Energy and
//  time objects are handled separately.
//              
//   format_code         Conversion
//        0              float --> U32
//        1              float * 10 --> U32
//        2              float * 100 --> U32
//        3              float --> S32
//        4              float * 10 --> S32
//        5              float * 100 --> S32
//        6              U32 --> U32
//        20             float --> float positive
//        21             float --> float positive
//        22             float --> float positive
//        23             float --> float
//        24             float --> float
//        25             float --> float
//        26             U32 --> U32
//        35             U64/1000 --> U32
//        36             S64/1000 --> S32
//        50, 70         internal time --> Modbus time
//        62             U64 --> U64
//        63             S64 --> S64
//        99, 119        invalid register, fill with zeros
const uint8_t MODB_OBJECT_CONV_GR10[] =
{                                                           // Modbus Fixed Addr (hex)
  6,                                                        // 1800: status presently not supported
  1, 1, 1, 1, 1, 1,                                         // 1802 thru 180C: Ia thru Iavg RMS
  1, 1, 1, 1,                                               // 180E thru 1814: Vab thru Vllavg RMS
  1, 1, 1, 1,                                               // 1816 thru 181C: Van thru Vlnavg RMS
  99,                                                       // 181E: Vng not supported
  1, 1, 1,                                                  // 1820 thru 1824: Peak Demand Ia thru Ic
  99,                                                       // 1826: Peak Demand Ig not supported
  1,                                                        // 1828: Peak Demand In
  3, 3,                                                     // 182A thru 122C/182C: Power W, Var
  0,                                                        // 182E: Power VA
  5, 5,                                                     // 1830 thru 1832: PF_Disp, PF_App
  1,                                                        // 1834: Load frequency
  99,                                                       // 1836: Unused
  1, 1, 1,                                                  // 1838 thru 183C: Min Demand Ia thru Ic
  99,                                                       // 183E: Min Demand Ig not supported
  1,                                                        // 1840: Min Demand In
  1,                                                        // 1842: f2 * 10
  2, 2, 2,                                                  // 1844 thru 1848: (f2, f2min, f2max) * 100
  99,                                                       // 184A: unused
  5, 5, 5,                                                  // 184C thru 1850: PF_Disp Pha, b, c
  5, 5, 5,                                                  // 1852 thru 1856: PF_App Pha, b, c
  3,                                                        // 1858: Peak Demand W
  99, 99, 99, 99, 99, 99, 99, 99, 99, 99,                   // 185A thru 186C: unused
  99,                                                       // 186E: Product ID presently not supported
  2                                                         // 1870: f1
};



// Modbus Real Time Data Objects set 2 Data Address Table
//   This handles both Group 11 and Group 23 above
void * const MODB_OBJECT_ADDR_GR11[] =
{                                                           // Modbus Address (hex)
  &THSensor.Temperature                                     // 189C: Temperature
};

// Modbus Real-Time Data Objects set 2 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 23 above), but also handles floating point
//     addresses (Group 11 above).  Group 11 conversion = Group 23 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR11[] =
{                                                           // Modbus Fixed Addr (hex)
  3                                                         // 189C: Temperature
};



// Modbus Real Time Data Objects set 3 Data Address Table
//   This handles both Group 12 and Group 25 above
void * const MODB_OBJECT_ADDR_GR12[] =
{                                                           // Modbus Float Addr/Modbus Fixed Addr (hex)
  &PDmnd.TotVarMax,             &PDmnd.TotVAMax,            // 12BC/18BC, 12BE/18BE: Peak Demand Var, VA
  &Pwr200msec.Pa,               &Pwr200msec.Pb,             // 12C0/18C0, 12C2/18C2: Pha W, Phb W
  &Pwr200msec.Pc,                                           // 12C4/18C4: Phc W
  &Pwr200msec.RPa,              &Pwr200msec.RPb,            // 12C6/18C6, 12C8/18C8: Pha Var, Phb Var
  &Pwr200msec.RPc,                                          // 12CA/18CA: Phc Var
  &Pwr200msecApp.AppPa,         &Pwr200msecApp.AppPb,       // 12CC/18CC, 12CE/18CE: Pha VA, Phb VA
  &Pwr200msecApp.AppPc,                                     // 12D0/18D0: Phc VA
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 12D2/18D2, 12D4/18D4: Unused
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 12D6/18D6, 12D8/18D8: Unused
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 12DA/18DA, 12DC/18DC: Unused
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 12DE/18DE, 12E0/18E0: Unused
  &EngyDmnd[1].DmndIa,          &EngyDmnd[1].DmndIb,        // 12E2/18E2, 12E4/18E4: Demand Ia, Ib
  &EngyDmnd[1].DmndIc,                                      // 12E6/18E6: Demand Ic
  &mb_reg_not_supported,                                    // 12E8/18E8: Unused
  &EngyDmnd[1].DmndIn,                                      // 12EA/18EA: Demand In
  &EngyDmnd[1].DmndTotW,        &EngyDmnd[1].DmndTotVar,    // 12EC/18EC, 12EE/18EE: Demand W, Var
  &EngyDmnd[1].DmndTotVA,                                   // 12F0/18F0: Demand VA
  &CurOneCyc_min.Ia,            &CurOneCyc_max.Ia,          // 12F2/18F2, 12F4/18F4: Ia min, Ia max
  &CurOneCyc_min.Ib,            &CurOneCyc_max.Ib,          // 12F6/18F6, 12F8/18F8: Ib min, Ib max
  &CurOneCyc_min.Ic,            &CurOneCyc_max.Ic,          // 12FA/18FA, 12FC/18FC: Ic min, Ic max
  &CurOneCyc_min.Ig,            &CurOneCyc_max.Ig,          // 12FE/18FE, 1300/1900: Ig min, Ig max
  &CurOneCyc_min.In,            &CurOneCyc_max.In,          // 1302/1902, 1304/1904: In min, In max
  &VolAFEOneCyc_min.Vab,        &VolAFEOneCyc_max.Vab,      // 1306/1906, 1308/1908: Vab min, Vab max
  &VolAFEOneCyc_min.Vbc,        &VolAFEOneCyc_max.Vbc,      // 130A/190A, 130C/190C: Vbc min, Vbc max
  &VolAFEOneCyc_min.Vca,        &VolAFEOneCyc_max.Vca,      // 130E/190E, 1310/1910: Vca min, Vca max
  &VolAFEOneCyc_min.Van,        &VolAFEOneCyc_max.Van,      // 1312/1912, 1314/1914: Van min, Van max
  &VolAFEOneCyc_min.Vbn,        &VolAFEOneCyc_max.Vbn,      // 1316/1916, 1318/1918: Vbn min, Vbn max
  &VolAFEOneCyc_min.Vcn,        &VolAFEOneCyc_max.Vcn,      // 131A/191A, 131C/191C: Vcn min, Vcn max
  &LD_Bucket,                   &mb_reg_not_supported,      // 131E/191E, 1320/1920: Unused
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1322/1922, 1324/1924: Unused
  &PF.MinApp[3],                &PF.MaxApp[3],              // 1326/1926, 1328/1928: PF App min, max
  &FreqLoad.MinFreqVal,         &FreqLoad.MaxFreqVal,       // 132A/192A, 132C/192C: f1 min, f1 max
  &CurUnbalTot,                                             // 132E/192E: Current Unbalance
  &VolUnbalTot,                                             // 1330/1930: Voltage Unbalance
  &THD[0],                      &THD[1],                    // 1332/1932, 1334/1934: THD Ia, Ib
  &THD[2],                                                  // 1336/1936: THD Ic
  &mb_reg_not_supported,                                    // 1338/1938: Unused
  &THD[3],                                                  // 133A/193A: THD In
  &THD[7],                      &THD[8],                    // 133C/193C, 133E/193E: THD Vab, Vbc
  &THD[9],                                                  // 1340/1940: THD Vca
  &THD[4],                      &THD[5],                    // 1342/1942, 1344/1944: THD Van, Vbn
  &THD[6],                                                  // 1346/1946: THD Vcn
  &CF.Ia,                       &CF.Ib,                     // 1348/1948, 134A/194A: CF Ia, Ib
  &CF.Ic,                                                   // 134C/194C: CF Ic
  &mb_reg_not_supported,                                    // 134E/194E: Unused
  &CF.In,                                                   // 1350/1950: CF In
  &KF_Val.Ia,                   &KF_Val.Ib,                 // 1352/1952, 1354/1954: K-Factor Ia, Ib
  &KF_Val.Ic                                                // 1356/1956: K-Factor Ic
};

// Modbus Real-Time Data Objects set 3 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 25 above), but also handles floating point
//     addresses (Group 12 above).  Group 12 conversion = Group 25 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR12[] =
{                                                           // Modbus Fixed Addr (hex)
  3, 0,                                                     // 18BC thru 18BE: Peak Demand Var, VA
  3, 3, 3,                                                  // 18C0 thru 18C4: Pha W, Phb W, Phc W
  3, 3, 3,                                                  // 18C6 thru 18CA: Pha Var, Phb Var, Phc Var
  3, 3, 3,                                                  // 18CC thru 18D0: Pha VA, Phb VA, Phc VA
  99, 99, 99, 99, 99, 99, 99, 99,                           // 18D2 thru 18E0: Unused
  1, 1, 1,                                                  // 18E2 thru 18E6: Demand Ia, Ib, Ic
  99,                                                       // 18E8: Unused
  1,                                                        // 18EA: Demand In
  3, 3,                                                     // 18EC thru 18EE: Demand W, Var
  0,                                                        // 18F0: Demand VA
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1,                             // 18F2 thru 1904: Ia min/max - In min/max
  1, 1, 1, 1, 1, 1,                                         // 1906 thru 1910: Vab min/max - Vca min/max
  1, 1, 1, 1, 1, 1,                                         // 1912 thru 191C: Van min/max - Vcn min/max
  0,                                                        // 191E: PKE (Thermal) Overload
  99, 99, 99,                                               // 1920 thru 1924: Unused
  2, 2,                                                     // 1926 thru 1928: PF App min, max
  2, 2,                                                     // 192A thru 192C: f1 min, f1 max
  2,                                                        // 192E: Current Unbalance
  2,                                                        // 1930: Voltage Unbalance
  2, 2, 2,                                                  // 1932 thru 1936: THD Ia, Ib, Ic
  99,                                                       // 1938: Unused
  2,                                                        // 193A: THD In
  2, 2, 2,                                                  // 193C thru 1940: THD Vab, Vbc, Vca
  2, 2, 2,                                                  // 1942 thru 1946: THD Van, Vbn, Vcn
  2, 2, 2,                                                  // 1948 thru 194C: CF Ia, Ib, Ic
  99,                                                       // 194E: Unused
  2,                                                        // 1950: CF In
  0, 0, 0                                                   // 1952 thru 1956: K-Factor Ia, Ib, Ic
};



// Modbus Harmonics Objects Data Address Table
//   This handles both Group 14 and Group 27 above
// Note, this table is different the other tables.  It is just the address of the first harmonic for each
//   object.  Note, there is a duplicate In entry for each set, because the user selection register is as
//   follows:
//        0=Ia    1=Ib    2=Ic    4=In    5=Vab    6=Vbc    7=Vca    8=Van    9=Vbn   10=Vcn
//   (3 is skipped)
uint16_t * const MODB_OBJECT_ADDR_GR14[] =
{
  &MB_Harmonics_Selection,
  &HarmonicsAgg.Ia[0],  &HarmonicsAgg.Ib[0],  &HarmonicsAgg.Ic[0],  &HarmonicsAgg.In[0],  &HarmonicsAgg.In[0],
  &HarmonicsAgg.Vab[0], &HarmonicsAgg.Vbc[0], &HarmonicsAgg.Vca[0],
  &HarmonicsAgg.Van[0], &HarmonicsAgg.Vbn[0], &HarmonicsAgg.Vcn[0],
  &HarmonicsCap.Ia[0],  &HarmonicsCap.Ib[0],  &HarmonicsCap.Ic[0],  &HarmonicsCap.In[0],  &HarmonicsCap.In[0],
  &HarmonicsCap.Vab[0], &HarmonicsCap.Vbc[0], &HarmonicsCap.Vca[0],
  &HarmonicsCap.Van[0], &HarmonicsCap.Vbn[0], &HarmonicsCap.Vcn[0]
};



// Modbus Real Time Data Objects set 4 Data Address Table
//   This handles both Group 15 and Group 28 above
void * const MODB_OBJECT_ADDR_GR15[] =
{                                                           // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolADC200ms.Vab,             &VolADC200ms.Vbc,           // 15E0/1BE0, 15E2/1BE2
  &VolADC200ms.Vca,             &VolADC200msVllavg,         // 15E4/1BE4, 15E6/1BE6
  &VolADC200ms.Van,             &VolADC200ms.Vbn,           // 15E8/1BE8, 15EA/1BEA
  &VolADC200ms.Vcn,             &VolADC200msVlnavg,         // 15EC/1BEC, 15EE/1BEE
  &SeqComp.V_PosMag,            &SeqComp.V_PosPh,           // 15FO/1BFO, 15F2/1BF2
  &SeqComp.V_NegMag,            &SeqComp.V_NegPh,           // 15F4/1BF4, 15F6/1BF6
  &SeqComp.V_ZeroMag,           &SeqComp.V_ZeroPh,          // 15F8/1BF8, 15FA/1BFA
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 15FC/1BFC, 15FE/1BFE
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1600/1C00, 1602/1C02
  &mb_reg_not_supported,        &mb_reg_not_supported,      // 1604/1C04, 1606/1C06
  &VolADCOneCyc_min.Vab,        &VolADCOneCyc_max.Vab,      // 1608/1C08, 160A/1C0A
  &VolADCOneCyc_min.Vbc,        &VolADCOneCyc_max.Vbc,      // 160C/1C0C, 160E/1C0E
  &VolADCOneCyc_min.Vca,        &VolADCOneCyc_max.Vca,      // 1610/1C10, 1612/1C12
  &VolADCOneCyc_min.Van,        &VolADCOneCyc_max.Van,      // 1614/1C14, 1616/1C16
  &VolADCOneCyc_min.Vbn,        &VolADCOneCyc_max.Vbn,      // 1618/1C18, 161A/1C1A
  &VolADCOneCyc_min.Vcn,        &VolADCOneCyc_max.Vcn,      // 161C/1C1C, 161E/1C1E
  &Pwr200msecMin.Pa,            &Pwr200msecMax.Pa,          // 1620/1C20, 1622/1C22
  &Pwr200msecMin.Pb,            &Pwr200msecMax.Pb,          // 1624/1C24, 1626/1C26
  &Pwr200msecMin.Pc,            &Pwr200msecMax.Pc,          // 1628/1C28, 162A/1C2A
  &Pwr200msecMin.Ptot,          &Pwr200msecMax.Ptot,        // 162C/1C2C, 162E/1C2E
  &Pwr200msecMin.RPa,           &Pwr200msecMax.RPa,         // 1630/1C30, 1632/1C32
  &Pwr200msecMin.RPb,           &Pwr200msecMax.RPb,         // 1634/1C34, 1636/1C36
  &Pwr200msecMin.RPc,           &Pwr200msecMax.RPc,         // 1638/1C38, 163A/1C3A
  &Pwr200msecMin.Rtot,          &Pwr200msecMax.Rtot,        // 163C/1C3C, 163E/1C3E
  &Pwr200msecAppMin.AppPa,      &Pwr200msecAppMax.AppPa,    // 1640/1C40, 1642/1C42
  &Pwr200msecAppMin.AppPb,      &Pwr200msecAppMax.AppPb,    // 1644/1C44, 1646/1C46
  &Pwr200msecAppMin.AppPc,      &Pwr200msecAppMax.AppPc,    // 1648/1C48, 164A/1C4A
  &Pwr200msecAppMin.Apptot,     &Pwr200msecAppMax.Apptot,   // 164C/1C4C, 164E/1C4E
  &PhAngles1[0],                &PhAngles1[1],              // 1650/1C50, 1652/1C52
  &PhAngles1[2],                &PhAngles1[3],              // 1654/1C54, 1656/1C56
  &PhAngles1[4],                &PhAngles1[5],              // 1658/1C58, 165A/1C5A
  &PhAngles1[6],                &PhAngles1[7],              // 165C/1C5C, 165E/1C5E
  &PhAngles1[8],                &PhAngles2[0],              // 1660/1C60, 1662/1C62
  &PhAngles2[1],                &PhAngles2[2],              // 1664/1C64, 1666/1C66
  &PhAngles2[3],                &PhAngles2[4],              // 1668/1C68, 166A/1C6A
  &PhAngles2[5],                &SeqComp.I_PosMag,          // 166C/1C6C, 166E/1C6E
  &SeqComp.I_NegMag,            &SeqComp.I_ZeroMag,         // 1670/1C70, 1672/1C72
  &SeqComp.I_PosPh,             &SeqComp.I_NegPh,           // 1674/1C74, 1676/1C76
  &SeqComp.I_ZeroPh,            &PF.MinDisp[3],             // 1678/1C78, 167A/1C7A
  &PF.MaxDisp[3],               &PF.MinApp[0],              // 167C/1C7C, 167E/1C7E
  &PF.MaxApp[0],                &PF.MinApp[1],              // 1680/1C80, 1682/1C82
  &PF.MaxApp[1],                &PF.MinApp[2],              // 1684/1C84, 1686/1C86
  &PF.MaxApp[2],                &PF.MinDisp[0],             // 1688/1C88, 168A/1C8A
  &PF.MaxDisp[0],               &PF.MinDisp[1],             // 168C/1C8C, 168E/1C8E
  &PF.MaxDisp[1],               &PF.MinDisp[2],             // 1690/1C80, 1682/1C82
  &PF.MaxDisp[2]                                            // 1694/1C84
};

// Modbus Real-Time Data Objects set 4 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 28 above), but also handles floating point
//     addresses (Group 15 above).  Group 15 conversion = Group 28 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR15[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 1, 1, 1,                                               // 1BE0-1BE6: ADC Vab-Vllavg
  1, 1, 1, 1,                                               // 1BE8-1BEE: ADC Van-Vlnavg
  1, 1, 1, 1, 1, 1,                                         // 1BFO-1BFA: AFE VLN Sequence Components
  99, 99, 99, 99, 99, 99,                                   // 1BFC-1C06: Unused
  1, 1, 1, 1, 1, 1,                                         // 1C08-1C12: ADC min/max Vab-Vca
  1, 1, 1, 1, 1, 1,                                         // 1C14-1C1E: ADC min/max Van-Vcn
  3, 3, 3, 3, 3, 3, 3, 3,                                   // 1C20-1C2E: min/max Pa-Ptot
  3, 3, 3, 3, 3, 3, 3, 3,                                   // 1C30-1C3E: min/max Vara-Vartot
  0, 0, 0, 0, 0, 0, 0, 0,                                   // 1C40-1C4E: min/max VAa-VAtot
  2, 2, 2,                                                  // 1C50-1C54: Phase Angle IA-IC
  2, 2, 2,                                                  // 1C56-1C5A: Phase Angle AFE Van-Vcn
  2, 2, 2,                                                  // 1C5C-1C60: Phase Angle AFE Vab-Vca
  2, 2, 2,                                                  // 1C62-1C66: Phase Angle ADC Van-Vcn
  2, 2, 2,                                                  // 1C68-1C6C: Phase Angle ADC Vab-Vca
  1, 1, 1, 1, 1, 1,                                         // 1C6E-1678: Current Sequence Components
  2, 2,                                                     // 1C7A-1C7C: min/max PF Disp total
  2, 2, 2, 2, 2, 2,                                         // 1C7E-1C88: min/max PF App Pha-Phc
  2, 2, 2, 2, 2, 2                                          // 1C8A-1C94: min/max PF Disp Pha-Phc
};



// Modbus Real Time Data Objects set 5 Data Address Table
//   This handles both Group 16 and Group 29 above
void * const MODB_OBJECT_ADDR_GR16[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &CurOneCyc_min.Ia,            &CurOneCyc_min.IaTS,            // 1696/1C96, 1698/1C98
  &CurOneCyc_max.Ia,            &CurOneCyc_max.IaTS,            // 169C/1C9C, 169E/1C9E
  &CurOneCyc_min.Ib,            &CurOneCyc_min.IbTS,            // 16A2/1CA2, 16A4/1CA4
  &CurOneCyc_max.Ib,            &CurOneCyc_max.IbTS,            // 16A8/1CA8, 16AA/1CAA
  &CurOneCyc_min.Ic,            &CurOneCyc_min.IcTS,            // 16AE/1CAE, 16B0/1CB0
  &CurOneCyc_max.Ic,            &CurOneCyc_max.IcTS,            // 16B4/1CB4, 16B6/1CB6
  &CurOneCyc_min.Ig,            &CurOneCyc_min.IgTS,            // 16BA/1CBA, 16BC/1CBC
  &CurOneCyc_max.Ig,            &CurOneCyc_max.IgTS,            // 16C0/1CC0, 16C2/1CC2
  &CurOneCyc_min.In,            &CurOneCyc_min.InTS,            // 16C6/1CC6, 16C8/1CC8
  &CurOneCyc_max.In,            &CurOneCyc_max.InTS,            // 16CC/1CCC, 16CE/1CCE
  &CurOneCyc_max.InTS                                           // 16D2/1CD2                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 5 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 29 above), but also handles floating point
//     addresses (Group 16 above).  Group 16 conversion = Group 29 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR16[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50,                                                 // 1C96-1C9E: Ia min/max plus Timestamps
  1, 50, 1, 50,                                                 // 1CA2-1CAA: Ib min/max plus Timestamps
  1, 50, 1, 50,                                                 // 1CAE-1CB6: Ic min/max plus Timestamps
  1, 50, 1, 50,                                                 // 1CBA-1CC2: Ig min/max plus Timestamps
  1, 50, 1, 50,                                                 // 1CC6-1CCE: In min/max plus Timestamps
  50                                                            // 1CD2: Time of last reset
};



// Modbus Real Time Data Objects set 6 Data Address Table
//   This handles both Group 17 and Group 30 above
void * const MODB_OBJECT_ADDR_GR17[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolAFEOneCyc_min.Vab,        &VolAFEOneCyc_min.VabTS,        // 16D6/1CD6, 16D8/1CD8
  &VolAFEOneCyc_max.Vab,        &VolAFEOneCyc_max.VabTS,        // 16DC/1CDC, 16DE/1CDE
  &VolAFEOneCyc_min.Vbc,        &VolAFEOneCyc_min.VbcTS,        // 16E2/1CE2, 16E4/1CE4
  &VolAFEOneCyc_max.Vbc,        &VolAFEOneCyc_max.VbcTS,        // 16E8/1CE8, 16EA/1CEA
  &VolAFEOneCyc_min.Vca,        &VolAFEOneCyc_min.VcaTS,        // 16EE/1CEE, 16F0/1CF0
  &VolAFEOneCyc_max.Vca,        &VolAFEOneCyc_max.VcaTS,        // 16F4/1CF4, 16F6/1CF6
  &VolAFEOneCyc_max.VcaTS                                       // 16FA/1CFA                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 6 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 30 above), but also handles floating point
//     addresses (Group 17 above).  Group 17 conversion = Group 30 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR17[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50,                                                 // 1CD6-1CDE: AFE Vab min/max plus TS's
  1, 50, 1, 50,                                                 // 1CE2-1CEA: AFE Vbc min/max plus TS's
  1, 50, 1, 50,                                                 // 1CEE-1CF6: AFE Vca min/max plus TS's
  50                                                            // 1CFA: Time of last reset
};



// Modbus Real Time Data Objects set 7 Data Address Table
//   This handles both Group 18 and Group 31 above
void * const MODB_OBJECT_ADDR_GR18[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolAFEOneCyc_min.Van,        &VolAFEOneCyc_min.VanTS,        // 16FE/1CFE, 1700/1D00
  &VolAFEOneCyc_max.Van,        &VolAFEOneCyc_max.VanTS,        // 1704/1D04, 1706/1D06
  &VolAFEOneCyc_min.Vbn,        &VolAFEOneCyc_min.VbnTS,        // 170A/1D0A, 170C/1D0C
  &VolAFEOneCyc_max.Vbn,        &VolAFEOneCyc_max.VbnTS,        // 1710/1D10, 1712/1D12
  &VolAFEOneCyc_min.Vcn,        &VolAFEOneCyc_min.VcnTS,        // 1716/1D16, 1718/1D18
  &VolAFEOneCyc_max.Vcn,        &VolAFEOneCyc_max.VcnTS,        // 171C/1D1C, 171E/1D1E
  &VolAFEOneCyc_max.VcnTS                                       // 1722/1D22                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 7 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 31 above), but also handles floating point
//     addresses (Group 18 above).  Group 18 conversion = Group 31 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR18[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50,                                                 // 1CFE-1D06: AFE Van min/max plus TS's
  1, 50, 1, 50,                                                 // 1D0A-1D12: AFE Vbn min/max plus TS's
  1, 50, 1, 50,                                                 // 1D16-1D1E: AFE Vcn min/max plus TS's
  50                                                            // 1D22: Time of last reset
};



// Modbus Real Time Data Objects set 8 Data Address Table
//   This handles both Group 19 and Group 32 above
void * const MODB_OBJECT_ADDR_GR19[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolADCOneCyc_min.Vab,        &VolADCOneCyc_min.VabTS,        // 1726/1D26, 1728/1D28
  &VolADCOneCyc_max.Vab,        &VolADCOneCyc_max.VabTS,        // 172C/1D2C, 172E/1D2E
  &VolADCOneCyc_min.Vbc,        &VolADCOneCyc_min.VbcTS,        // 1732/1D32, 1734/1D34 
  &VolADCOneCyc_max.Vbc,        &VolADCOneCyc_max.VbcTS,        // 1738/1D38, 173A/1D3A 
  &VolADCOneCyc_min.Vca,        &VolADCOneCyc_min.VcaTS,        // 173E/1D3E, 1740/1D40
  &VolADCOneCyc_max.Vca,        &VolADCOneCyc_max.VcaTS,        // 1744/1D44, 1746/1D46
  &VolADCOneCyc_max.VcaTS                                       // 174A/1D4A                *** DAH   REPLACE WITH TIME OF LAST RESET  
};

// Modbus Real-Time Data Objects set 8 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 32 above), but also handles floating point
//     addresses (Group 19 above).  Group 19 conversion = Group 32 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR19[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50,                                                 // 1D26-1D2E: ADC Vbc min/max plus TS's
  1, 50, 1, 50,                                                 // 1D32-1D3A: ADC Vca min/max plus TS's
  1, 50, 1, 50,                                                 // 1D3E-1D46: ADC Van min/max plus TS's
  50                                                            // 1D4A: Time of last reset
};



// Modbus Real Time Data Objects set 9 Data Address Table
//   This handles both Group 20 and Group 33 above
void * const MODB_OBJECT_ADDR_GR20[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolADCOneCyc_min.Van,        &VolADCOneCyc_min.VanTS,        // 174E/1D4E, 1750/1D40
  &VolADCOneCyc_max.Van,        &VolADCOneCyc_max.VanTS,        // 1754/1D54, 1756/1D56
  &VolADCOneCyc_min.Vbn,        &VolADCOneCyc_min.VbnTS,        // 175A/1D5A, 175C/1D5C
  &VolADCOneCyc_max.Vbn,        &VolADCOneCyc_max.VbnTS,        // 1760/1D60, 1762/1D62
  &VolADCOneCyc_min.Vcn,        &VolADCOneCyc_min.VcnTS,        // 1766/1D66, 1768/1D68
  &VolADCOneCyc_max.Vcn,        &VolADCOneCyc_max.VcnTS,        // 176C/1D6C, 176E/1D6E
  &VolADCOneCyc_max.VcnTS                                       // 1772/1D72                *** DAH   REPLACE WITH TIME OF LAST RESET  
};

// Modbus Real-Time Data Objects set 9 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 33 above), but also handles floating point
//     addresses (Group 20 above).  Group 20 conversion = Group 33 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR20[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50,                                                 // 1D4E-1D56: ADC Van min/max plus TS's
  1, 50, 1, 50,                                                 // 1D5A-1D62: ADC Vbn min/max plus TS's
  1, 50, 1, 50,                                                 // 1D66-1D6E: ADC Vcn min/max plus TS's
  50                                                            // 1D72: Time of last reset
};



// Modbus Objects Data Address Table
//   This handles Groups 22 and 24 above
void * const MODB_OBJECT_ADDR_GR22[] =
{                                                           // Modbus Address (hex)
  &EngyDmnd[1].TotFwdWHr,                                   // 1872: Energy Real Forward
  &EngyDmnd[1].TotRevWHr,                                   // 1874: Energy Real Reverse
  &TotWHr,                                                  // 1876: Energy Real Total
  &EngyDmnd[1].TotLeadVarHr,                                // 1878: Energy Reactive Leading
  &EngyDmnd[1].TotLagVarHr,                                 // 187A: Energy Reactive Lagging
  &NetVarHr,                                                // 187C: Energy Reactive Net
  &EngyDmnd[1].TotVAHr                                      // 187E: Energy Apparent
};

// Modbus Group 22 Real-Time Data Objects Conversion Type Table
//   This handles Group 22 above.  Objects are the same as for 24, but different scaling
const uint8_t MODB_OBJECT_CONV_GR22[] =
{                                                           // Modbus Fixed Addr (hex)
  35, 35,                                                   // 1872-1874: Whr Forward, Reverse
  36,                                                       // 1876: Whr Total
  35, 35,                                                   // 1878-187A: VarHr Leading, Lagging
  36,                                                       // 187C: Varhr Net
  35                                                        // 187E: VAhr
};

// Modbus Group 24 Real-Time Data Objects Conversion Type Table
//   This handles Group 24 above.  Objects are the same as for 22, but different scaling
const uint8_t MODB_OBJECT_CONV_GR24[] =
{                                                           // Modbus Fixed Addr (hex)
  62, 62,                                                   // 1872-1874: Whr Forward, Reverse
  63,                                                       // 1876: Whr Total
  62, 62,                                                   // 1878-187A: VarHr Leading, Lagging
  63,                                                       // 187C: Varhr Net
  62                                                        // 187E: VAhr
};



// Modbus Real Time Data Objects set 10 Data Address Table
//   This handles both Group 37 and Group 55 above
void * const MODB_OBJECT_ADDR_GR37[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &PF.MinApp[0],                &PF.MinApp_TS[0],               // 6000/C000, 6002/C002
  &PF.MaxApp[0],                &PF.MaxApp_TS[0],               // 6006/C006, 6008/C008
  &PF.MinApp[1],                &PF.MinApp_TS[1],               // 600C/C00C, 600E/C00E
  &PF.MaxApp[1],                &PF.MaxApp_TS[1],               // 6012/C012, 6014/C014
  &PF.MinApp[2],                &PF.MinApp_TS[2],               // 6018/C018, 601A/C01A
  &PF.MaxApp[2],                &PF.MaxApp_TS[2],               // 601E/C01E, 6020/C020
  &PF.MinApp[3],                &PF.MinApp_TS[3],               // 6024/C024, 6026/C026
  &PF.MaxApp[3],                &PF.MaxApp_TS[3],               // 602A/C02A, 602C/C02C
  &PF.MaxApp_TS[3]                                              // 6030/C030                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 10 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 37 above), but also handles floating point
//     addresses (Group 55 above).  Group 55 conversion = Group 37 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR37[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50,       // C000-C02C: PF App min/max plus Timestamps
  50                                                            // C030: Time of last reset
};



// Modbus Real Time Data Objects set 11 Data Address Table
//   This handles both Group 38 and Group 56 above
void * const MODB_OBJECT_ADDR_GR38[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &PF.MinDisp[0],               &PF.MinDisp_TS[0],              // 6034/C034, 6036/C036
  &PF.MaxDisp[0],               &PF.MaxDisp_TS[0],              // 603A/C03A, 603C/C03C
  &PF.MinDisp[1],               &PF.MinDisp_TS[1],              // 6040/C040, 6042/C042
  &PF.MaxDisp[1],               &PF.MaxDisp_TS[1],              // 6046/C046, 6048/C048
  &PF.MinDisp[2],               &PF.MinDisp_TS[2],              // 604C/C04C, 604E/C04E
  &PF.MaxDisp[2],               &PF.MaxDisp_TS[2],              // 6052/C052, 6054/C054
  &PF.MinDisp[3],               &PF.MinDisp_TS[3],              // 6058/C058, 605A/C05A
  &PF.MaxDisp[3],               &PF.MaxDisp_TS[3],              // 605E/C05E, 6060/C060
  &PF.MaxDisp_TS[3]                                             // 6064/C064                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 11 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 38 above), but also handles floating point
//     addresses (Group 56 above).  Group 56 conversion = Group 38 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR38[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50,       // C034-C060: PF Disp min/max plus TS's
  50                                                            // C064: Time of last reset
};



// Modbus Real Time Data Objects set 12 Data Address Table
//   This handles both Group 39 and Group 57 above
void * const MODB_OBJECT_ADDR_GR39[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &FreqLoad.MinFreqVal,         &FreqLoad.MinFreqVal_TS,        // 6068/C068, 606A/C06A
  &FreqLoad.MaxFreqVal,         &FreqLoad.MaxFreqVal_TS,        // 606E/C06E, 6070/C070
  &FreqLine.MinFreqVal,         &FreqLine.MinFreqVal_TS,        // 6074/C074, 6076/C076
  &FreqLine.MaxFreqVal,         &FreqLine.MaxFreqVal_TS,        // 607A/C07A, 607C/C07C
  &FreqLine.MaxFreqVal_TS                                       // 6080/C080                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 12 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 39 above), but also handles floating point
//     addresses (Group 57 above).  Group 57 conversion = Group 39 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR39[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50,                                                 // C068-C070: f load min/max + Timestamps
  2, 50, 2, 50,                                                 // C074-C07C: f line min/max + Timestamps
  50                                                            // C080: Time of last reset
};



// Modbus Real Time Data Objects set 13 Data Address Table
//   This handles both Group 40 and Group 58 above
void * const MODB_OBJECT_ADDR_GR40[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &IDmnd.IaMin,                 &IDmnd.IaMinTS,                 // 6084/C084, 6086/C086
  &IDmnd.IaMax,                 &IDmnd.IaMaxTS,                 // 608A/C08A, 608C/C08C
  &IDmnd.IbMin,                 &IDmnd.IbMinTS,                 // 6090/C090, 6092/C092
  &IDmnd.IbMax,                 &IDmnd.IbMaxTS,                 // 6096/C096, 6098/C098
  &IDmnd.IcMin,                 &IDmnd.IcMinTS,                 // 609C/C09C, 609E/C09E
  &IDmnd.IcMax,                 &IDmnd.IcMaxTS,                 // 60A2/C0A2, 60A4/C0A4
  &IDmnd.InMin,                 &IDmnd.InMinTS,                 // 60A8/C0A8, 60AA/C0AA
  &IDmnd.InMax,                 &IDmnd.InMaxTS,                 // 60AE/C0AE, 60B0/C0B0
  &IDmnd.InMaxTS                                                // 60B4/C0B4                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 13 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 40 above), but also handles floating point
//     addresses (Group 58 above).  Group 58 conversion = Group 40 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR40[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 50, 1, 50, 1, 50, 1, 50, 1, 50, 1, 50, 1, 50, 1, 50,       // C084-C0B0: Demand I Pha-Phn plus TS's
  50                                                            // C0B4: Time of last reset
};



// Modbus Real Time Data Objects set 14 Data Address Table
//   This handles both Group 41 and Group 59 above
void * const MODB_OBJECT_ADDR_GR41[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &PDmnd.TotWMax,               &PDmnd.TotWMaxTS,               // 60B8/C0B8, 60BA/C0BA
  &PDmnd.TotVarMax,             &PDmnd.TotVarMaxTS,             // 60BE/C0BE, 60C0/C0C0
  &PDmnd.TotVAMax,              &PDmnd.TotVAMaxTS,              // 60C4/C0C4, 60C6/C0C6
  &PDmnd.TotVAMaxTS                                             // 60CA/C0CA                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 14 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 41 above), but also handles floating point
//     addresses (Group 59 above).  Group 59 conversion = Group 41 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR41[] =
{                                                           // Modbus Fixed Addr (hex)
  3, 50,                                                        // C0B8-C0BA: Demand Wtot plus Timestamp
  3, 50,                                                        // C0BE-C0C0: Demand Vartot plus Timestamp
  0, 50,                                                        // C0C4-C0C6: Demand VAtot plus Timestamp
  50                                                            // C0CA: Time of last reset
};



// Modbus Real Time Data Objects set 15 Data Address Table
//   This handles both Group 42 and Group 60 above
void * const MODB_OBJECT_ADDR_GR42[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &CurUnbal.Ia,                 &CurUnbal.Ib,                   // 60CE/C0CE, 60D0/C0D0
  &CurUnbal.Ic,                 &CurUnbal.In,                   // 60D2/C0D2, 60D4/C0D4
  &CurUnbalMax,                                                 // 60D6/C0D6
  &VolUnbal.Van,                &VolUnbal.Vbn,                  // 60D8/C0D8, 60DA/C0DA
  &VolUnbal.Vcn,                                                // 60DC/C0DC
  &VolUnbalMaxLN,                                               // 60DE/C0DE
  &VolUnbal.Vab,                &VolUnbal.Vbc,                  // 60E0/C0E0, 60E2/C0E2
  &VolUnbal.Vca,                                                // 60E4/C0E4
  &VolUnbalMaxLL                                                // 60E6/C0E6
};

// Modbus Real-Time Data Objects set 15 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 42 above), but also handles floating point
//     addresses (Group 60 above).  Group 60 conversion = Group 42 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR42[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 2, 2, 2, 2,                                                // C0CE-C0D6: Current Unbalance Ia - Imax
  2, 2, 2, 2,                                                   // C0D8-C0DE: Voltage Unbalance Van - MaxLN
  2, 2, 2, 2                                                    // C0E0-C0E6: Voltage Unbalance Vab - MaxLL
};



// Modbus Real Time Data Objects set 16 Data Address Table
//   This handles both Group 43 and Group 61 above
void * const MODB_OBJECT_ADDR_GR43[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &CurUnbalPh_max.Ia,           &CurUnbalPh_max.IaTS,           // 60E8/C0E8, 60EA/C0EA
  &CurUnbalPh_max.Ib,           &CurUnbalPh_max.IbTS,           // 60EE/C0EE, 60F0/C0F0
  &CurUnbalPh_max.Ic,           &CurUnbalPh_max.IcTS,           // 60F4/C0F4, 60F6/C0F6
  &CurUnbalPh_max.In,           &CurUnbalPh_max.InTS,           // 60FA/C0FA, 60FC/C0FC
  &CurUnbalAllMax,              &CurUnbalAllMaxTS,              // 6100/C100, 6102/C102
  &CurUnbalTotMax,              &CurUnbalTotMaxTS,              // 6106/C106, 6108/C108
  &CurUnbalTotMaxTS                                             // 610C/C10C                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 16 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 43 above), but also handles floating point
//     addresses (Group 61 above).  Group 61 conversion = Group 43 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR43[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50,                                   // C0E8-C0FC: Ix unbal max plus Timestamps
  2, 50,                                                        // C100-C102: Iall unbal max plus Timestamp
  2, 50,                                                        // C106-C108: Itot unbal max plus Timestamp
  50                                                            // C10C: Time of last reset
};



// Modbus Real Time Data Objects set 17 Data Address Table
//   This handles both Group 44 and Group 62 above
void * const MODB_OBJECT_ADDR_GR44[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolUnbalPh_max.Van,          &VolUnbalPh_max.VanTS,          // 6110/C110, 6112/C112
  &VolUnbalPh_max.Vbn,          &VolUnbalPh_max.VbnTS,          // 6116/C116, 6118/C118
  &VolUnbalPh_max.Vcn,          &VolUnbalPh_max.VcnTS,          // 611C/C11C, 611E/C11E
  &VolUnbalAllMaxLN,            &VolUnbalAllMaxLNTS,            // 6122/C122, 6124/C124
  &VolUnbalAllMaxLNTS                                           // 6128/C128                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 17 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 44 above), but also handles floating point
//     addresses (Group 62 above).  Group 62 conversion = Group 44 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR44[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50,                                          // C110-C11E: Vln unbal max + Timestamps
  2, 50,                                                        // C122-C124: Vln all unbal max + Timestamp
  50                                                            // C128: Time of last reset
};



// Modbus Real Time Data Objects set 18 Data Address Table
//   This handles both Group 45 and Group 63 above
void * const MODB_OBJECT_ADDR_GR45[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &VolUnbalPh_max.Vab,          &VolUnbalPh_max.VabTS,          // 612C/C12C, 612E/C12E
  &VolUnbalPh_max.Vbc,          &VolUnbalPh_max.VbcTS,          // 6132/C132, 6134/C134
  &VolUnbalPh_max.Vca,          &VolUnbalPh_max.VcaTS,          // 6138/C138, 613A/C13A
  &VolUnbalAllMaxLL,            &VolUnbalAllMaxLLTS,            // 613E/C13E, 6140/C140
  &VolUnbalTotMax,              &VolUnbalTotMaxTS,              // 6144/C144, 6146/C146
  &VolUnbalTotMaxTS                                             // 614A/C14A                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 18 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 45 above), but also handles floating point
//     addresses (Group 63 above).  Group 63 conversion = Group 45 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR45[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50,                                          // C12C-C13A: Vll unbal max + Timestamps
  2, 50,                                                        // C13E-C140: Vll all unbal max + Timestamp
  2, 50,                                                        // C144-C146: Vll tot unbal max + Timestamp
  50                                                            // C14A: Time of last reset
};



// Modbus Real Time Data Objects set 19 Data Address Table
//   This handles both Group 46 and Group 64 above
void * const MODB_OBJECT_ADDR_GR46[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &THDminmax[0].THDmin,         &THDminmax[0].THDminTS,         // 614E/C14E, 6150/C150
  &THDminmax[0].THDmax,         &THDminmax[0].THDmaxTS,         // 6154/C154, 6156/C156
  &THDminmax[1].THDmin,         &THDminmax[1].THDminTS,         // 615A/C15A, 615C/C15C
  &THDminmax[1].THDmax,         &THDminmax[1].THDmaxTS,         // 6160/C160, 6162/C162
  &THDminmax[2].THDmin,         &THDminmax[2].THDminTS,         // 6166/C166, 6168/C168
  &THDminmax[2].THDmax,         &THDminmax[2].THDmaxTS,         // 616C/C16C, 616E/C16E
  &THDminmax[3].THDmin,         &THDminmax[3].THDminTS,         // 6172/C172, 6174/C174
  &THDminmax[3].THDmax,         &THDminmax[3].THDmaxTS,         // 6178/C178, 617A/C17A
  &THDminmax[3].THDmaxTS                                        // 617E/C17E                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 19 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 46 above), but also handles floating point
//     addresses (Group 64 above).  Group 64 conversion = Group 46 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR46[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50,       // C14E-C17A: THD Ix min/max + Timestamps
  50                                                            // C17E: Time of last reset
};



// Modbus Real Time Data Objects set 20 Data Address Table
//   This handles both Group 47 and Group 65 above
void * const MODB_OBJECT_ADDR_GR47[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &THDminmax[7].THDmin,         &THDminmax[7].THDminTS,         // 6182/C182, 6184/C184
  &THDminmax[7].THDmax,         &THDminmax[7].THDmaxTS,         // 6188/C188, 618A/C18A
  &THDminmax[8].THDmin,         &THDminmax[8].THDminTS,         // 618E/C18E, 6190/C190
  &THDminmax[8].THDmax,         &THDminmax[8].THDmaxTS,         // 6194/C194, 6196/C196
  &THDminmax[9].THDmin,         &THDminmax[9].THDminTS,         // 619A/C19A, 619C/C19C
  &THDminmax[9].THDmax,         &THDminmax[9].THDmaxTS,         // 61A0/C1A0, 61A2/C1A2
  &THDminmax[9].THDmaxTS                                        // 61A6/C1A6                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 20 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 47 above), but also handles floating point
//     addresses (Group 65 above).  Group 65 conversion = Group 47 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR47[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50,                     // C182-C1A2: THD Vll min/max + Timestamps
  50                                                            // C1A6: Time of last reset
};



// Modbus Real Time Data Objects set 21 Data Address Table
//   This handles both Group 48 and Group 66 above
void * const MODB_OBJECT_ADDR_GR48[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &THDminmax[4].THDmin,         &THDminmax[4].THDminTS,         // 61AA/C1AA, 61AC/C1AC
  &THDminmax[4].THDmax,         &THDminmax[4].THDmaxTS,         // 61BO/C1BO, 61B2/C1B2
  &THDminmax[5].THDmin,         &THDminmax[5].THDminTS,         // 61B6/C1B6, 61B8/C1B8
  &THDminmax[5].THDmax,         &THDminmax[5].THDmaxTS,         // 61BC/C1BC, 61BE/C1BE
  &THDminmax[6].THDmin,         &THDminmax[6].THDminTS,         // 61C2/C1C2, 61C4/C1C4
  &THDminmax[6].THDmax,         &THDminmax[6].THDmaxTS,         // 61C8/C1C8, 61CA/C1CA
  &THDminmax[6].THDmaxTS                                        // 61CE/C1CE                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 21 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 48 above), but also handles floating point
//     addresses (Group 66 above).  Group 66 conversion = Group 48 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR48[] =
{                                                           // Modbus Fixed Addr (hex)
  2, 50, 2, 50, 2, 50, 2, 50, 2, 50, 2, 50,                     // C1AA-C1CA: THD Vln min/max + Timestamps
  50                                                            // C1CE: Time of last reset
};



// Modbus Real Time Data Objects set 22 Data Address Table
//   This handles both Group 49 and Group 67 above
void * const MODB_OBJECT_ADDR_GR49[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &LD_BucketMax,                &LD_BucketMaxTS,                // 61D2/C1D2, 61D4/C1D4
  &LD_BucketMaxTS                                               // 61D8/C1D8                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 22 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 49 above), but also handles floating point
//     addresses (Group 67 above).  Group 67 conversion = Group 49 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR49[] =
{                                                           // Modbus Fixed Addr (hex)
  0, 50, 0, 50,                                                 // C1D2-C1D4: Max Overload + Timestamps
  50                                                            // C1D8: Time of last reset
};



// Modbus Real Time Data Objects set 23 Data Address Table
//   This handles both Group 50 and Group 68 above
void * const MODB_OBJECT_ADDR_GR50[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &SD_BucketMax,                &SD_BucketMaxTS,                // 61DC/C1DC, 61DE/C1DE
  &SD_BucketMaxTS                                               // 61E2/C1E2                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 23 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 50 above), but also handles floating point
//     addresses (Group 68 above).  Group 68 conversion = Group 50 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR50[] =
{                                                           // Modbus Fixed Addr (hex)
  0, 50, 0, 50,                                                 // C1DC-C1DE: Max Overload + Timestamps
  50                                                            // C1E2: Time of last reset
};



// Modbus Real Time Data Objects set 24 Data Address Table
//   This handles both Group 53 and Group 71 above
void * const MODB_OBJECT_ADDR_GR53[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &Pwr200msecMin.Pa,            &Pwr200msecMin.PaTS,            // 6500/C500, 6502/C502
  &Pwr200msecMax.Pa,            &Pwr200msecMax.PaTS,            // 6506/C506, 6508/C508
  &Pwr200msecMin.Pb,            &Pwr200msecMin.PbTS,            // 650C/C50C, 650E/C50E
  &Pwr200msecMax.Pb,            &Pwr200msecMax.PbTS,            // 6512/C512, 6514/C514
  &Pwr200msecMin.Pc,            &Pwr200msecMin.PcTS,            // 6518/C518, 651A/C51A
  &Pwr200msecMax.Pc,            &Pwr200msecMax.PcTS,            // 651E/C51E, 6520/C520
  &Pwr200msecMin.Ptot,          &Pwr200msecMin.PtotTS,          // 6524/C524, 6526/C526
  &Pwr200msecMax.Ptot,          &Pwr200msecMax.PtotTS,          // 652A/C52A, 652C/C52C
  &Pwr200msecMin.RPa,           &Pwr200msecMin.RPaTS,           // 6530/C530, 6532/C532
  &Pwr200msecMax.RPa,           &Pwr200msecMax.RPaTS,           // 6536/C536, 6538/C538
  &Pwr200msecMin.RPb,           &Pwr200msecMin.RPbTS,           // 653C/C53C, 653E/C53E
  &Pwr200msecMax.RPb,           &Pwr200msecMax.RPbTS,           // 6542/C542, 6544/C544
  &Pwr200msecMin.RPc,           &Pwr200msecMin.RPcTS,           // 6548/C548, 654A/C54A
  &Pwr200msecMax.RPc,           &Pwr200msecMax.RPcTS,           // 654E/C54E, 6550/C550
  &Pwr200msecMin.Rtot,          &Pwr200msecMin.RtotTS,          // 6554/C554, 6556/C556
  &Pwr200msecMax.Rtot,          &Pwr200msecMax.RtotTS,          // 655A/C55A, 655C/C55C
  &Pwr200msecAppMin.AppPa,      &Pwr200msecAppMin.AppPaTS,      // 6560/C560, 6562/C562 
  &Pwr200msecAppMax.AppPa,      &Pwr200msecAppMax.AppPaTS,      // 6566/C566, 6568/C568 
  &Pwr200msecAppMin.AppPb,      &Pwr200msecAppMin.AppPbTS,      // 656C/C56C, 656E/C56E 
  &Pwr200msecAppMax.AppPb,      &Pwr200msecAppMax.AppPbTS,      // 6572/C572, 6574/C574 
  &Pwr200msecAppMin.AppPc,      &Pwr200msecAppMin.AppPcTS,      // 6578/C578, 657A/C57A 
  &Pwr200msecAppMax.AppPc,      &Pwr200msecAppMax.AppPcTS,      // 657E/C57E, 6580/C580 
  &Pwr200msecAppMin.Apptot,     &Pwr200msecAppMin.ApptotTS,     // 6584/C584, 6586/C586 
  &Pwr200msecAppMax.Apptot,     &Pwr200msecAppMax.ApptotTS,     // 658A/C58A, 658C/C58C 
  &Pwr200msecAppMax.ApptotTS                                    // 6590/C590                *** DAH   REPLACE WITH TIME OF LAST RESET
};

// Modbus Real-Time Data Objects set 24 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 53 above), but also handles floating point
//     addresses (Group 71 above).  Group 71 conversion = Group 53 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR53[] =
{                                                           // Modbus Fixed Addr (hex)
  3, 50, 3, 50,                                                 // C500-C508: W Pha min/max plus Timestamps
  3, 50, 3, 50,                                                 // C50C-C514: W Phb min/max plus Timestamps
  3, 50, 3, 50,                                                 // C518-C520: W Phc min/max plus Timestamps
  3, 50, 3, 50,                                                 // C524-C52C: W tot min/max plus Timestamps
  3, 50, 3, 50,                                                 // C530-C538: Var Pha min/max plus TS's
  3, 50, 3, 50,                                                 // C53C-C544: Var Phb min/max plus TS's
  3, 50, 3, 50,                                                 // C548-C550: Var Phc min/max plus TS's
  3, 50, 3, 50,                                                 // C554-C55C: Var tot min/max plus TS's
  0, 50, 0, 50,                                                 // C560-C568: VA Pha min/max plus Timestamps
  0, 50, 0, 50,                                                 // C56C-C574: VA Phb min/max plus Timestamps
  0, 50, 0, 50,                                                 // C578-C580: VA Phc min/max plus Timestamps
  0, 50, 0, 50,                                                 // C584-C58C: VA tot min/max plus Timestamps
  50                                                            // C590: Time of last reset
};



// Modbus Real Time Data Objects set 25 Data Address Table
//   This handles both Group 54 and Group 72 above
void * const MODB_OBJECT_ADDR_GR54[] =
{                                                               // Modbus Float Addr/Modbus Fixed Addr (hex)
  &Res5min_Avg.Ia,              &Res5min_Avg.Ib,                // 6594/C594, 6596/C596
  &Res5min_Avg.Ic,              &Res5min_Avg.Iavg,              // 6598/C598, 659A/C59A
  &Res5min_Avg.Pa,              &Res5min_Avg.Pb,                // 659C/C59C, 659E/C59E
  &Res5min_Avg.Pc,              &Res5min_Avg.Ptot,              // 65A0/C5A0, 65A2/C5A2
  &Res5min_Avg.AppPa,           &Res5min_Avg.AppPb,             // 65A4/C5A4, 65A6/C5A6
  &Res5min_Avg.AppPc,           &Res5min_Avg.AppPtot,           // 65A8/C5A8, 65AA/C5AA
  &Res5min_MinMax.Iamax,        &Res5min_MinMax.Iamin,          // 65AC/C5AC, 65AE/C5AE
  &Res5min_MinMax.Ibmax,        &Res5min_MinMax.Ibmin,          // 65B0/C5B0, 65B2/C5B2
  &Res5min_MinMax.Icmax,        &Res5min_MinMax.Icmin,          // 65B4/C5B4, 65B6/C5B6
  &Res5min_MinMax.Iavgmax,      &Res5min_MinMax.Iavgmin,        // 65B8/C5B8, 65BA/C5BA
  &Res5min_MinMax.Pamax,        &Res5min_MinMax.Pamin,          // 65BC/C5BC, 65BE/C5BE
  &Res5min_MinMax.Pbmax,        &Res5min_MinMax.Pbmin,          // 65C0/C5C0, 65C2/C5C2
  &Res5min_MinMax.Pcmax,        &Res5min_MinMax.Pcmin,          // 65C4/C5C4, 65C6/C5C6
  &Res5min_MinMax.Ptotmax,      &Res5min_MinMax.Ptotmin,        // 65C8/C5C8, 65CA/C5CA
  &Res5min_MinMax.AppPamax,     &Res5min_MinMax.AppPamin,       // 65CC/C5CC, 65CE/C5CE
  &Res5min_MinMax.AppPbmax,     &Res5min_MinMax.AppPbmin,       // 65D0/C5D0, 65D2/C5D2
  &Res5min_MinMax.AppPcmax,     &Res5min_MinMax.AppPcmin,       // 65D4/C5D4, 65D6/C5D6
  &Res5min_MinMax.AppPtotmax,   &Res5min_MinMax.AppPtotmin      // 65D8/C5D8, 65DA/C5DA
};

// Modbus Real-Time Data Objects set 25 Conversion Type Table
//   Note, the table is for Fixed Point addresses (Group 54 above), but also handles floating point
//     addresses (Group 72 above).  Group 72 conversion = Group 54 conversion + 20
const uint8_t MODB_OBJECT_CONV_GR54[] =
{                                                           // Modbus Fixed Addr (hex)
  1, 1, 1, 1,                                                   // C594-C59A: 5-min Ia, Ib, Ic, Iavg
  3, 3, 3, 3,                                                   // C59C-C5A2: 5-min Pa, Pb, Pc, Ptot
  0, 0, 0, 0,                                                   // C5A4-C5AA: 5-min AppPa - AppPtot
  1, 1, 1, 1, 1, 1, 1, 1,                                       // C5AC-C5BA: 5-min Ia - Iavg min/max
  3, 3, 3, 3, 3, 3, 3, 3,                                       // C5BC-C5CA: 5-min Pa - Ptot min/max
  0, 0, 0, 0, 0, 0, 0, 0                                        // C5CC-C5DA: 5-min AppPa - AppPtot min/max
};



// Modbus Objects Data Address Table
//   This handles Groups 69 and 70 above
void * const MODB_OBJECT_ADDR_GR69[] =
{                                                           // Modbus Address (hex)
  &NetWHr,                                                  // C210/C214: Energy Real Net
  &TotVarHr                                                 // C212/C218: Energy Reactive Total
};

// Modbus Group 39 Real-Time Data Objects Conversion Type Table
//   This handles Group 69 above.  Objects are the same as for 70, but different scaling
const uint8_t MODB_OBJECT_CONV_GR69[] =
{                                                           // Modbus Fixed Addr (hex)
  36,                                                       // C210: Whr Net
  36                                                        // C212: VarHr Total
};

// Modbus Group 40 Real-Time Data Objects Conversion Type Table
//   This handles Group 70 above.  Objects are the same as for 69, but different scaling
const uint8_t MODB_OBJECT_CONV_GR70[] =
{                                                           // Modbus Fixed Addr (hex)
  63,                                                       // C214: Whr Net
  63                                                        // C214: VarHr Total
};



// Note, not all arrays exist or are required.  Those that are not required have a duplicate array to fill
//   the location
const uint8_t * const MODB_OBJECT_CONV_ADDR[] =
{
  &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0],
  &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0],
  &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR11[0],
  &MODB_OBJECT_CONV_GR12[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR15[0],
  &MODB_OBJECT_CONV_GR16[0], &MODB_OBJECT_CONV_GR17[0], &MODB_OBJECT_CONV_GR18[0], &MODB_OBJECT_CONV_GR19[0],
  &MODB_OBJECT_CONV_GR20[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR22[0], &MODB_OBJECT_CONV_GR11[0],
  &MODB_OBJECT_CONV_GR24[0], &MODB_OBJECT_CONV_GR12[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0],
  &MODB_OBJECT_CONV_GR15[0], &MODB_OBJECT_CONV_GR16[0], &MODB_OBJECT_CONV_GR17[0], &MODB_OBJECT_CONV_GR18[0],
  &MODB_OBJECT_CONV_GR19[0], &MODB_OBJECT_CONV_GR20[0], &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR10[0],
  &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR37[0], &MODB_OBJECT_CONV_GR38[0], &MODB_OBJECT_CONV_GR39[0],
  &MODB_OBJECT_CONV_GR40[0], &MODB_OBJECT_CONV_GR41[0], &MODB_OBJECT_CONV_GR42[0], &MODB_OBJECT_CONV_GR43[0],
  &MODB_OBJECT_CONV_GR44[0], &MODB_OBJECT_CONV_GR45[0], &MODB_OBJECT_CONV_GR46[0], &MODB_OBJECT_CONV_GR47[0],
  &MODB_OBJECT_CONV_GR48[0], &MODB_OBJECT_CONV_GR49[0], &MODB_OBJECT_CONV_GR50[0], &MODB_OBJECT_CONV_GR10[0],
  &MODB_OBJECT_CONV_GR10[0], &MODB_OBJECT_CONV_GR53[0], &MODB_OBJECT_CONV_GR54[0], &MODB_OBJECT_CONV_GR37[0],
  &MODB_OBJECT_CONV_GR38[0], &MODB_OBJECT_CONV_GR39[0], &MODB_OBJECT_CONV_GR40[0], &MODB_OBJECT_CONV_GR41[0],
  &MODB_OBJECT_CONV_GR42[0], &MODB_OBJECT_CONV_GR43[0], &MODB_OBJECT_CONV_GR44[0], &MODB_OBJECT_CONV_GR45[0],
  &MODB_OBJECT_CONV_GR46[0], &MODB_OBJECT_CONV_GR47[0], &MODB_OBJECT_CONV_GR48[0], &MODB_OBJECT_CONV_GR49[0],
  &MODB_OBJECT_CONV_GR50[0], &MODB_OBJECT_CONV_GR69[0], &MODB_OBJECT_CONV_GR70[0], &MODB_OBJECT_CONV_GR53[0],
  &MODB_OBJECT_CONV_GR54[0]
};

void * const * const MODB_OBJECT_ADDR[] =
{
  &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0],
  &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0],
  &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR11[0],
  &MODB_OBJECT_ADDR_GR12[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR15[0],
  &MODB_OBJECT_ADDR_GR16[0], &MODB_OBJECT_ADDR_GR17[0], &MODB_OBJECT_ADDR_GR18[0], &MODB_OBJECT_ADDR_GR19[0],
  &MODB_OBJECT_ADDR_GR20[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR22[0], &MODB_OBJECT_ADDR_GR11[0],
  &MODB_OBJECT_ADDR_GR22[0], &MODB_OBJECT_ADDR_GR12[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0],
  &MODB_OBJECT_ADDR_GR15[0], &MODB_OBJECT_ADDR_GR16[0], &MODB_OBJECT_ADDR_GR17[0], &MODB_OBJECT_ADDR_GR18[0],
  &MODB_OBJECT_ADDR_GR19[0], &MODB_OBJECT_ADDR_GR20[0], &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR10[0],
  &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR37[0], &MODB_OBJECT_ADDR_GR38[0], &MODB_OBJECT_ADDR_GR39[0],
  &MODB_OBJECT_ADDR_GR40[0], &MODB_OBJECT_ADDR_GR41[0], &MODB_OBJECT_ADDR_GR42[0], &MODB_OBJECT_ADDR_GR43[0],
  &MODB_OBJECT_ADDR_GR44[0], &MODB_OBJECT_ADDR_GR45[0], &MODB_OBJECT_ADDR_GR46[0], &MODB_OBJECT_ADDR_GR47[0],
  &MODB_OBJECT_ADDR_GR48[0], &MODB_OBJECT_ADDR_GR49[0], &MODB_OBJECT_ADDR_GR50[0], &MODB_OBJECT_ADDR_GR10[0],
  &MODB_OBJECT_ADDR_GR10[0], &MODB_OBJECT_ADDR_GR53[0], &MODB_OBJECT_ADDR_GR54[0], &MODB_OBJECT_ADDR_GR37[0],
  &MODB_OBJECT_ADDR_GR38[0], &MODB_OBJECT_ADDR_GR39[0], &MODB_OBJECT_ADDR_GR40[0], &MODB_OBJECT_ADDR_GR41[0],
  &MODB_OBJECT_ADDR_GR42[0], &MODB_OBJECT_ADDR_GR43[0], &MODB_OBJECT_ADDR_GR44[0], &MODB_OBJECT_ADDR_GR45[0],
  &MODB_OBJECT_ADDR_GR46[0], &MODB_OBJECT_ADDR_GR47[0], &MODB_OBJECT_ADDR_GR48[0], &MODB_OBJECT_ADDR_GR49[0],
  &MODB_OBJECT_ADDR_GR50[0], &MODB_OBJECT_ADDR_GR69[0], &MODB_OBJECT_ADDR_GR69[0], &MODB_OBJECT_ADDR_GR53[0],
  &MODB_OBJECT_ADDR_GR54[0]
};


// Function Codes 5 and 15
// At the initial writing of this function there are no Function Code 5 or 15 registers suppported.
//  const uint16_t MODB_FC0515_START_ADD = ???? future use;
//  const uint16_t MODB_FC0515_END_ADD = ???? future use;


//------------------------------------------------------------------------------------------------------------
//                   Modbus Interface Operational Overview
//------------------------------------------------------------------------------------------------------------
//
// The PXR35 contains a single Modbus RTU slave port.  The port is driven by the microprocessor's UART6
// peripheral.
//
// *** Reference "MODBUS over Serial Line Specification and Implementation Guide", V1.02, 12/20/2006 ***
//
// Modbus is a half-duplex master-slave interface.  All characters are typically 11 bits in length:
//      Start bit, 8 data bits, parity bit, stop bit
//   or Start bit, 8 data bits, stop bit, stop bit  (if no parity, character size is still 11 bits)
// On rare occasions, there is parity + 2 stop bits, and the character size is 12 bits.
// Messages are framed by the idle time between characters, as described below:
//      Idle Time < 1.5 char times: Character is part of the existing message
//      1.5 char times < Idle Time < 3.5 char times: Illegal character, abort message
//      3.5 char times < Idle Time: Character is the beginning of a new message
// Due to these strict timing requirements between characters, interrupts are used to receive the
// characters, and to measure the time interval between characters.
// Not all Modbus masters meet the timing requirements, ao we are operating with a relaxed specification.
// We just check whether the idle time is greater than 3.5 char times.  If it is, the character is the
// beginning of a new message.  If it is not, it is appended to the existing message.
//
// A basic modbus RTU message from a master to a slave device consists of the following:
//      | Slave address | Function code |     Data      | CRC low byte | CRC high byte |
//      |    1 byte     |    1 byte     | 0 - 252 bytes |    1 byte    |    1 byte     |  (CRC is 2 bytes)
//   Slave address - 0: broadcast address (receive only, no response)
//                   1 - 247: normal addresses
//   Function code - 1: Read coil status - discrete outputs ("Read" is from the master's perspective)
//                   2: Read input status - discrete inputs
//                   3: Read holding registers - fixed values and setpoints
//                   4: Read input registers - Actual values and events
//                   6: Write preset single register - configuration
//                   8: Read diagnostics
//                  15: Write multiple coils - discrete outputs
//                  16: Write preset multiple registers - configuration and setpoints
//
// The basic Modbus operation is described below:
//   - The Modbus communications process is handled by a single foreground subroutine, ModB_SlaveComm, and
//     one interrupt subroutine, UART6_IRQHandler()
//   - The Modbus communications process is defined as being in one of three states:
//       - MODB_IDLE: The processor is neither transmitting nor receiving characters (i.e., no character has
//           been received for more than 3.5 character times)
//       - MODB_RECEIVING: The processor is receiving characters
//       - MODB_TRANSMITTING: The processor is transmitting characters
//     The state transition diagram is shown below:
//
//                                                   +-------+
//                                                   | Reset |
//                                                   +-------+
//                                                       |
//                                                       V
//                                                 +-----------+
//            +----------------------------------->| MODB_IDLE |<-------------------------------+
//            |                                    +-----------+                                |
//            |                                       |    |                                    |
//            |                  Character Received   |    | Start Transmissions                |
//            |                (Process_Modb_RxIRQ()) |    |  (ModB_SlaveComm())                |
//            |                          +------------+    +----------+                         |
//            |                          |                            |                         |
//            |                          |                            |                         |
//            |                          V                            V                         |
//            |                  +----------------+          +-------------------+              |
//            |                  | MODB_RECEIVING |          | MODB_TRANSMITTING |              |
//            |                  +----------------+          +-------------------+              |
//            |                          |                            |                         |
//            |  End of Message Frame    |                            | Last Char Sent          |
//            |    (Process_Modb_RxIRQ() |                            |   (UART6_IRQHandler())  |
//            |     or ModB_SlaveComm()) |                            |                         |
//            |                          |                            |                         |
//            +--------------------------+                            +-------------------------+
//
//     Note:
//       - As shown above, the decision to start transmissions and enter the TRANSMITTING state is made in
//         the foreground, while the decision to enter the RECEIVING state is made in the interrupt.  Once
//         the decision is made to transmit, it is not reversed.  It is possible for a character to be
//         received just as the TRANSMITTING state is entered.  If this occurs, the character and subsequent
//         message is dropped and a collision will likely occur.  Note that this is an error on the part of
//         the Modbus master, because it did not allow enough time for the PXR35 to respond.
//
//   - Basic Receiving Operation (greater details are in the comments for the individual subroutines):
//       - Characters are received, processed, and stored in Process_ModB_RxIRQ(), which is called from the
//         ISR.  The characters are stored in a linear buffer, ModB.RxMsgBuf[ModB.RxMsgNdx].  The Modbus
//         state is changed from MODB_IDLE to MODB_RECEIVING when the first character is received.
//       - When receptions end, the idle time is exceeded.  This is typically detected in the foreground,
//         although if a second message was begun immediately after the first message, it is detected in
//         Process_ModB_RxIRQ().  At this point, the receive buffer is frozen - no more characters will be
//         accepted until the message is processed.
//       - ModB.RxMsgNdx is the index of the next character to be received.  Hence, it is the length of a
//         received message.
//       - Frame breaks are detected in the MBS_IDLE state in ModB_SlaveComm().  The subroutine then
//         advances to MBS_PROCREQ, where the received message is parsed and decoded, and the response is
//         generated.
//
//   - Basic Transmitting Operation (greater details are in the comments for the individual subroutines):
//       - ModB.TxMsgBuf[] is a linear buffer containing the message to be transmitted.  ModB.TxMsgBuf[] is
//         large enough to accomodate the largest message to be transmitted.
//       - ModB.CharsToTx is the number of characters in the message to be transmitted
//       - Messages start at index 0
//       - DMA is used for message transmissions
//       - Transmit messages are assembled in the MBS_PROCREQ state in ModB_SlaveComm().  The state then
//         advances to MBS_START_TX, where the DMA is set up and initiated.  The subroutine then advances to
//         MBS_TX, where the DMA process is checked for completion.  When it is done, the subroutine
//         advances to MBS_DONE, where the transceivers are set back to receive mode.
//
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Modb_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Modbus Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Modbus interface subroutines.
//                      The following variables do not need initialization:
//                          ModB.RxMsgBuf[ ]                ModB.TxMsgBuf[ ]
//                          ModB.CharsToTx                  ModB.Comm_Timer
//
//  CAVEATS:            MB_Harmonics_Selection and ModB_CurSetGrp need intialized on power-up, but
//                      should not change if something like baud/parity/stop bits setpoints change,
//                      requiring UART initialization. 
//
//  INPUTS:             init_all
//
//  OUTPUTS:            ModB.xxx, mb_reg_not_supported, MB_Harmonics_Selection, ModB_CurSetGrp, 
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Modb_VarInit(uint8_t init_all)
{
  uint8_t i;

  ModB.CommState = MODB_IDLE;
  ModB.RxIabort = FALSE;
  ModB.RxIFrameBreak = FALSE;
  ModB.Reset_Req = FALSE;
  ModB.RxMsgNdx = 0;
  for (i=0; i<8; ++i)
  {
    ModB.MsgStat_Counter[i] = 0;
  }
  // This is not used.  It is only used if checking interval times between 1.5 and 3.5 char times
//  ModB.T1P5 = 0;
  ModB.State = 0;

  mb_reg_not_supported = 0;

  if (init_all)
  {
    MB_Harmonics_Selection = 0;
    ModB_CurSetGrp = 0x00FF; // start atGroup zero of the Active set for accessing setpoints
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Modb_VarInit()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Modb_FormatStoreVal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Modbus Format and Store Value
//
//  MECHANICS:          This subroutine converts the internal value into the format specified for the Modbus
//                      register.  The required conversion is specified by input variable format_code:
//                          format_code         Conversion
//                               0              float --> U32
//                               1              float * 10 --> U32
//                               2              float * 100 --> U32
//                               3              float --> S32
//                               4              float * 10 --> S32
//                               5              float * 100 --> S32
//                               6              U32 --> U32
//                               20             float --> float positive
//                               21             float --> float positive
//                               22             float --> float positive
//                               23             float --> float
//                               24             float --> float
//                               25             float --> float
//                               26             U32 --> U32
//                               35             U64/1000 --> U32
//                               36             S64/1000 --> S32
//                               50, 70         internal time --> Modbus time
//                               62             U64 --> U64
//                               63             S64 --> S64
//                               99, 119        invalid register, fill with zeros
//                      It then stores the value in the Modbus transmit register (ModB.TxMsgBuf[] at the
//                      location pointed to by *val_out
//
//  CAVEATS:            None
//
//  INPUTS:             format_code: conversion format
//                      *val_in: pointer to input value
//                      *val_out: pointer to output array (ModB.TxMsgBuf[])
//
//  OUTPUTS:            ModB.TxMsgBuf[]
//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Modb_FormatStoreVal(uint8_t format_code, void *val_in, uint8_t *val_out)
{
  union VAL_HOLDER
  {
    unsigned long long uval64;
    signed long long   sval64;
    float fval;
    uint32_t uval;
    int32_t sval;
  } temp_val;
  uint16_t reg_order;

  switch (format_code)
  {
    case 0:                             // float --> U32
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // Value should not be negative, so clamp it to zero if negative.  Note, this is an error condition
      //   that shouldn't happen.
      temp_val.fval = ((temp_val.fval < 0) ? (0) :(temp_val.fval));
      // If converting to U32, 0.5 is added for correct roundoff - casting drops the fractional portion
      temp_val.uval = (uint32_t)(temp_val.fval + 0.5F);
      break;

    case 1:                             // float * 10 --> U32
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // Value should not be negative, so clamp it to zero if negative.  Note, this is an error condition
      //   that shouldn't happen.
      temp_val.fval = ((temp_val.fval < 0) ? (0) :(temp_val.fval));
      // If converting to U32, multiply by 10 and 0.5 is added for correct roundoff - casting drops the
      //   fractional portion
      temp_val.fval = (temp_val.fval * 10);
      temp_val.uval = (uint32_t)(temp_val.fval + 0.5F);
      break;

    case 2:                             // float * 100 --> U32
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // Value should not be negative, so clamp it to zero if negative.  Note, this is an error condition
      //   that shouldn't happen.
      temp_val.fval = ((temp_val.fval < 0) ? (0) :(temp_val.fval));
      // If converting to U32, multiply by 10 and 0.5 is added for correct roundoff - casting drops the
      //   fractional portion
      temp_val.fval = (temp_val.fval * 100);
      temp_val.uval = (uint32_t)(temp_val.fval + 0.5F);
      break;

    case 3:                             // float --> S32
    case 23:                            // float --> float
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // If converting to S32, 0.5 is added for correct roundoff - casting drops the fractional portion
      //   (Note, must account for positive and negative numbers)
      if (format_code == 3)
      {
        temp_val.fval = ( (temp_val.fval < 0) ? (temp_val.fval - 0.5F) : (temp_val.fval + 0.5F) );
        temp_val.sval = (int32_t)(temp_val.fval);
      }
      break;

    case 4:                             // float * 10 --> S32
    case 24:                            // float --> float
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // If converting to S32, multiply by 10 and add 0.5 for correct roundoff - casting drops the
      //   fractional portion.  (Note, must account for positive and negative numbers)
      if (format_code == 4)
      {
        temp_val.fval = (temp_val.fval * 10);
        temp_val.fval = ( (temp_val.fval < 0) ? (temp_val.fval - 0.5F) : (temp_val.fval + 0.5F) );
        temp_val.sval = (int32_t)(temp_val.fval);
      }
      break;

    case 5:                             // float * 100--> S32
    case 25:                            // float --> float
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // If converting to S32, multiply by 100 and add 0.5 for correct roundoff - casting drops the
      //   fractional portion.  (Note, must account for positive and negative numbers)
      if (format_code == 5)
      {
        temp_val.fval = (temp_val.fval * 100);
        temp_val.fval = ( (temp_val.fval < 0) ? (temp_val.fval - 0.5F) : (temp_val.fval + 0.5F) );
        temp_val.sval = (int32_t)(temp_val.fval);
      }
      break;

    case 6:                             // U32 --> U32
    case 26:                            // U32 --> U32
      temp_val.uval = *((uint32_t *)val_in);    // Move input value to temporary uint32 value
      break;

    case 20:                            // float --> float positive
    case 21:                            // float --> float positive
    case 22:                            // float --> float positive
      temp_val.fval = *((float *)val_in);       // Move input value to temporary floating point value
      // Value should not be negative, so clamp it to zero if negative.  Note, this is an error condition
      //   that shouldn't happen.
      temp_val.fval = ((temp_val.fval < 0) ? (0) :(temp_val.fval));
      break;

    case 35:                            // U64/1000 --> U32
    case 62:                            // U64 --> U64
      temp_val.uval64 = *((unsigned long long *)val_in);
      if (format_code == 35)
      {
        temp_val.uval = (uint32_t)(temp_val.uval64/1000);
      }
      break;

    case 36:                            // S64/1000 --> S32
    case 63:                            // S64 --> S64
      temp_val.sval64 = *((signed long long *)val_in);
      if (format_code == 36)
      {
        temp_val.sval = (int32_t)(temp_val.sval64/1000);
      }
      break;

    case 50:                            // internal time --> Modbus Time
    case 70:                            // internal time --> Modbus Time
      temp_val.uval64 = InternalTimeToMBTime((struct INTERNAL_TIME *)val_in);
      break;

    case 99:                            // invalid register
    case 119:                           // invalid register
    default:
      temp_val.uval = 0;                    // Set output value to 0
      break;                                // Don't care about word order

  }
  // Store the value in the order specified by the setpoint:
  //   setpoint > 0 (factory default):
  //        Register n: b15..b0 of floating point value
  //        Register n+1: b31..b16 of floating point value
  //   setpoint = 0:
  //        Register n: b31..b16 of floating point value
  //        Register n+1: b15..b0 of floating point value
  // *** DAH  Replace this with the associated setpoint ((floating point word order in Modbus register 2002)
  if (format_code < 40)
  {
    reg_order = ( ((format_code < 6) || (format_code > 30)) ?
                   Setpoints2.stp.Modbus_RTU_Fixed_Pt_Word_Order :
                   Setpoints2.stp.Modbus_RTU_Floating_Pt_Word_Order);
    if (reg_order > 0)                                // Codes 0 - 25
    {
      val_out[0] =  (uint8_t)(temp_val.uval >> 8);    // low-order word first (ms byte always first)
      val_out[1] =  (uint8_t)(temp_val.uval);
      val_out[2] =  (uint8_t)(temp_val.uval >> 24);   // high-order word second (ms byte always first)
      val_out[3] =  (uint8_t)(temp_val.uval >> 16);
    }
    else
    {
      val_out[0] =  (uint8_t)(temp_val.uval >> 24);   // high-order word first (ms byte always first)
      val_out[1] =  (uint8_t)(temp_val.uval >> 16);
      val_out[2] =  (uint8_t)(temp_val.uval >> 8);    // low-order word second (ms byte always first)
      val_out[3] =  (uint8_t)(temp_val.uval);
    }
  }
  else                                                // Codes 50, 70, 62, 63
  {
    if ( (format_code == 50) || (format_code == 70) )   // Time is always returned in a specific order
    {
      reg_order = 0;
    }
    else
    {
      reg_order = Setpoints2.stp.Modbus_RTU_Fixed_Pt_Word_Order;
    }
    if (reg_order > 0)
    {
      val_out[0] =  (uint8_t)(temp_val.uval64 >> 8);    // low-order word first (ms byte always first)
      val_out[1] =  (uint8_t)(temp_val.uval64);
      val_out[2] =  (uint8_t)(temp_val.uval64 >> 24);
      val_out[3] =  (uint8_t)(temp_val.uval64 >> 16);
      val_out[4] =  (uint8_t)(temp_val.uval64 >> 40);
      val_out[5] =  (uint8_t)(temp_val.uval64 >> 32);
      val_out[6] =  (uint8_t)(temp_val.uval64 >> 56);
      val_out[7] =  (uint8_t)(temp_val.uval64 >> 48);
    }
    else
    {
      val_out[0] =  (uint8_t)(temp_val.uval64 >> 56);   // high-order byte first (ms byte always first)
      val_out[1] =  (uint8_t)(temp_val.uval64 >> 48);
      val_out[2] =  (uint8_t)(temp_val.uval64 >> 40);
      val_out[3] =  (uint8_t)(temp_val.uval64 >> 32);
      val_out[4] =  (uint8_t)(temp_val.uval64 >> 24);
      val_out[5] =  (uint8_t)(temp_val.uval64 >> 16);
      val_out[6] =  (uint8_t)(temp_val.uval64 >> 8);
      val_out[7] =  (uint8_t)(temp_val.uval64);
    }
  }

      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Modb_FormatStoreVal()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        ModB_SlaveComm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Communications processor for Modbus Slave
// 
//  MECHANICS:          This subroutine is the top-level communications processor for a Modbus Slave.  It
//                      basically operates as follows:
//                          - Check for a received message
//                          - Process the request
//                          - Assemble the reponse
//                          - Initiate transmissions
//                          - Wait for transmissions to end
//
//                      Modbus Slave Communications Processor Flowchart:
//
//                                                         +-----------------------------------------------+
//                                                         |                                               |
//                                                         V                                               |
//                                  +------------------------------------------------------+               |
//                                  |                IDLE STATE                            |               |
//                                  | Check for received message or timeout                |               |
//                                  | If comm state is Receiving and frame break received, |               |
//                                  |   change comm state to Idle                          |               |
//                                  +------------------------------------------------------+               |
//                                           |             |               |                               |
//                          Message Received |             |               | No message                    |
//                                           |             |               |      AND                      |
//                                           |             |               | No timeout                    |
//                                           |             |               +------------------------------>+
//                                           |             |                                               ^
//                                           |             | No frame break                                |
//                                           |             |     AND                                       |
//                                           |             |   Timeout      +-----------------------+      |
//                                           |             +--------------->| Set Reset Modbus flag |----->+
//                                           |                              +-----------------------+      ^
//                                           V                                                             |
//                         +------------------------------------+                                          |
//                         |        PROCESS REQUEST STATE       |                                          |
//                         | Process the request                |                                          |
//                         | Remove message from the queue      |                                          |
//                         | Assemble the response              |                                          |
//                         | Check whether receiving            |                                          |
//                         +------------------------------------+                                          |
//                                  |                  |                                                   |
//                        Receiving |                  | Not receiving                                     |
//                     +------------+                  +-------+                                           |
//                     |                                       |                                           |
//                     |                                       V                                           |
//                     |                      +------------------------------------+                       |
//                     |                      |           START TX STATE           |                       |
//                     |                      | Set the comm state to transmitting |                       |
//                     |                      | Initialize transmit buffer index   |                       |
//                     |                      | Enable the transmit DMA            |                       |
//                     |                      +------------------------------------+                       |
//                     |                                        |                                          |
//                     |                                        V                                          |
//                     |                                        +---+<--------------------------------+    |
//                     |                                        |                                     |    |
//                     |                                        V                                     |    |
//                     |                          +----------------------------+                      |    |
//                     |                          |          TX STATE          |                      |    |
//                     |                          | Check whether transmitting |                      |    |
//                     |                          | Check for timeout          |                      |    |
//                     |                          +----------------------------+                      |    |
//                     |                               |         |         |                          |    |
//                     |              Not transmitting |         |         | Transmitting, no timeout |    |
//                     |                               |         |         +--------------------------+    |
//                     |                               |         |                                         |
//                     +-------------->+<--------------+         | Transmitting, timeout                   |
//                                     |                         +------+                                  |
//                                     |                                |                                  |
//                                     V                                V                                  |
//                    +-------------------------------+       +---------------------+                      |
//                    |         DONE STATE            |       | Set Reset UART flag |                      |
//                    | Remove message from the queue |       +---------------------+                      |
//                    | Clear the frame break flag    |                 |                                  |
//                    | Set comm state to Idle        |                 |                                  |
//                    +-------------------------------+                 +--------------------------------->+
//
//
//  CAVEATS:            None
//
//  INPUTS:             ModB.CommState, TIM2->CNT, ModB.RxIFrameBreak, ModB.RxMsgBuf[], ModB.Comm_Timer,
//                      ModB.RxMsgNdx
// 
//  OUTPUTS:            ModB.CommState, ModB.RxMsgNdx
// 
//  ALTERS:             ModB.Reset_Req, ModB.State, ModB.MsgStat_Counter[], ModB.Comm_Timer,
//                      ModB.RxIFrameBreak
//
//  CALLS:              Init_UART6(), Modb_VarInit(), Init_TIM2(), CalcCRC()
// 
//  EXECUTION TIME:     Measured on 220304 (rev 0.51 code): 152usec with a Modbus master reading 125
//                      Group 36-37 registers (starting register = 49362).  Note, this is with interrupts
//                      disabled around the subroutine call.
//
//------------------------------------------------------------------------------------------------------------
//

void ModB_SlaveComm(void)
{
  uint8_t mbs_exit, len, ok, respond;
  uint16_t i;

  // A Modbus initialization request occurs on power-up and if the idle timer times out
  if ((ModB.Reset_Req) && (ModB.State == MBS_IDLE))
  {
    Init_UART6(FALSE);                  // This disables UART6 Rx interrupt
    Modb_VarInit(FALSE);                // This initializes the idle timer
    Init_TIM2();
    USART6->CR1 |= USART_CR1_RXNEIE;    // Enable UART6 receive interrupts
    ModB.Reset_Req = 0;                 // Clear for next time
    return;
  }
                                        // No reset request so enter the state machine

  mbs_exit = FALSE;                     // Initialize exit flag

  while (!mbs_exit)
  {
    switch (ModB.State)
    {
      case MBS_IDLE:                    // Idle State
        // Timer 2 is a downcounter that is intialized at t3.5.  If it reaches 0, the event flag
        //  (TIM_SR_UIF) is set and the counter stops.  Here, we don't want to clear the flag or restart the
        //  timer, because when the next character is received, we will detect a frame break, but there
        //  won't be any characters in the previous frame, so the interrupt will just save the character as
        //  the start of a new message, and then restart the timer.
        // If comm state is Receiving and either t3.5 expired or the Frame Break flag is True, we have
        //   reached the end of a message frame
        if ( (ModB.CommState == MODB_RECEIVING)
          && ((TIM2->SR & TIM_SR_UIF) || ModB.RxIFrameBreak) )
        {
          // Set comm state to Idle (shouldn't receive any more chars).  We will check this to make sure the
          //   Master isn't transmitting before we transmit our response
          ModB.CommState = MODB_IDLE;
          //   There is a message to process if there are chars in the buffer
          if (ModB.RxMsgNdx > 0)                         // If there is a message, increment the message
          {                                              //   counter, reset the idle timer, and fall into
            if (ModB.MsgStat_Counter[MSGCTR] < 0xFFFF)   //   the next state to process the message
            {
              ModB.MsgStat_Counter[MSGCTR]++;
            }
            ModB.Comm_Timer = MODB_IDLE_TIMEOUT;
            ModB.State = MBS_PROCREQ;
//            break;                                        Fall into the next state
          }
        }

        // No new message
        else
        {
          if (ModB.Comm_Timer == 0)                      // If idle timer timed out, set request to reset
          {
//            ModB.Reset_Req = TRUE;            *** DAH TEST SKIP RESETTING FOR NOW
          }
          mbs_exit = TRUE;                               // Remain in this state and exit
          break;
        }

      case MBS_PROCREQ:                 // Process Message State
        // Modbus message format:
        //   RxMsgBuf Index       Description
        //         0              Device Address
        //         1              Function Code
        //        2..N            Data field, e.g.:
        //              2              Register Address high byte
        //              3              Register Address low byte
        //              4              Number of Registers high byte
        //              5              Number of Registers low byte
        //        N+1             CRC low byte
        //        N+2             CRC high byte

        // Check device address.  The device address is always byte 0 of the message.  A valid address is
        //   either my address or 0 (the broadcast address).
        if ( (ModB.RxMsgBuf[0] == Setpoints2.stp.Modbus_Port_Addr)
          || (ModB.RxMsgBuf[0] == 0) )
        {                               //   Set ok to True and increment the slave
          ok = TRUE;                    //   message counter
          if (ModB.MsgStat_Counter[SLAVEMSGCTR] < 0xFFFF)
          {
            ModB.MsgStat_Counter[SLAVEMSGCTR]++;
          }
        }
        else                                                // Not a valid device address: set ok to False
        {
          ok = FALSE;
        }
                                        
        if (ModB.RxMsgBuf[0] == Setpoints2.stp.Modbus_Port_Addr) // If my device address, ok to respond
        {
          respond = TRUE;
        }
        else                                                // Otherwise, don't respond and increment No
        {                                                   //   Response Counter
          respond = FALSE;
          if (ModB.MsgStat_Counter[NORESPCTR] < 0xFFFF)
          {
            ModB.MsgStat_Counter[NORESPCTR]++;
          }
        }
                                            
        len = ModB.RxMsgNdx;                // Retrieve the message length
        
        // Check the CRC
        // Compute the CRC of the received msg and store in temp1.
        //   Note, the CRC is received low byte, then high byte.
        if (len > 2)
        {
          i = CalcCRC(&ModB.RxMsgBuf[0], (len - 2));
          if ( (ModB.RxMsgBuf[len - 2] != (i & 0x00FF))    // Low byte of CRC
            || (ModB.RxMsgBuf[len - 1] != (i >> 8)) )      // High byte of CRC
          {                                                // If CRC error, increment Communication Error
            if (ModB.MsgStat_Counter[ERRCTR] < 0xFFFF)     //   Counter and set ok to False
            {
              ModB.MsgStat_Counter[ERRCTR]++;
            }
            ok = FALSE;
          }
        }
        else                                             // If invalid message length, increment
        {                                                //   Communication Error Counter and set ok to
          if (ModB.MsgStat_Counter[ERRCTR] < 0xFFFF)     //   False
          {
            ModB.MsgStat_Counter[ERRCTR]++;
          }
          ok = FALSE;
        }
                                            
        if (ok)                             // If the message is ok so far, process it according to the
        {                                   //   function code
          switch (ModB.RxMsgBuf[1])    
          {
            case 0x01:                          // Function Code = 1: Read Coils/Outputs - only do if responding
            case 0x02:                          // Function Code = 2: Read Inputs - only do if responding
              if (respond == TRUE)
              {
                ProcFC0102Msg(len, ModB.RxMsgBuf[1]);
              }
              break;

            case 0x03:                          // Function Code = 3: Read holding register - only do if responding
            case 0x04:                          // Function Code = 4: Read input register - only do if responding
              if (respond == TRUE)
              {
                ProcFC0304Msg(len, ModB.RxMsgBuf[1]);
              }
              break;

/*            case 0x05:                          // Function Code = 5: Execute Action / Write Relays
              port->XmitNdx = ProcFC05Msg(port, 0, len);
              break;       */

            case 0x06:                          // Function Code = 6: Write
              // If respond = True, write is to my address, so ok to process.  If respond = False, write is
              //   to broadcast address, which is illegal.  Ignore it.  Don't NAK, because we don't respond
              //   to a broadcast message
              if (respond == TRUE)
              {
                ProcFC06Msg(len);
              }
              break;

/*            case 0x08:                          // Function Code = 8: Read Diagnostic Counter - only do if
                                                //   responding
              if (respond == TRUE)
              {
                port->XmitNdx = ProcFC08Msg(port, 0, len);
              }
              break;      */

            case 0x10:                          // Function Code = 16: Write Setpoints, Execute Actions
              // If respond = True, write is to my address, so ok to process.  If respond = False, write is
              //   to broadcast address, which is illegal.  Ignore it.  Don't NAK, because we don't respond
              //   to a broadcast message
              if (respond == TRUE)
              {
                ProcFC16Msg(len);
              }
              break;

            default:                            // Invalid Function Code - exception response
              if (respond == TRUE)
              {
                ModB.TxMsgBuf[0] = Setpoints2.stp.Modbus_Port_Addr; // Char 0: Device Address
                ModB.TxMsgBuf[1] = (ModB.RxMsgBuf[1] | 0x80);   // Char 1: Function Code with msb set
                ModB.TxMsgBuf[2] = NAK_ILLEGAL_FUNCTION;        // Char 2: Exception Code
                // Compute CRC of message and load into transmit buffer.  Note, the CRC is transmitted low
                //   byte, then high byte.
                i = CalcCRC(&ModB.TxMsgBuf[0], 3);
                ModB.TxMsgBuf[3] = (uint8_t)(i);
                ModB.TxMsgBuf[4] = (uint8_t)(i>>8);
                ModB.CharsToTx = 5;
                if (ModB.MsgStat_Counter[EXCTR] < 0xFFFF)   // Increment the exception error counter
                {
                  ModB.MsgStat_Counter[EXCTR]++;
                }
              }
              break;
          }
        }                                   // If the message is not ok (not my address or invalid CRC),
                                            //   ignore it

        // If the frame break break came in the interrupt, another character was received, so someone else
        //   is transmitting.  Therefore, we can't transmit any response to this message - just go back to
        //   the Idle state.  Note, this is either:  1) an error on the part of the Modbus master, because
        //   it transmitted the start of another message before waiting the Response Time or Turnaround Time
        //   or  2) the master is communicating with a different device
        //   Also, if the received message is bad or does not require a response, just go back to the Idle
        //   state
        if ( (ModB.RxIFrameBreak)
          || (ok == FALSE)
          || (respond == FALSE) )
        {
          ModB.State = MBS_DONE;
          ModB.Comm_Timer = MODB_IDLE_TIMEOUT;   // Reset the timeout for the Idle state
          break;
        }
        // Otherwise it is ok to transmit the response, so fall into the next state
        else
        {
          ModB.State = MBS_START_TX;
//          break;
        }

      case MBS_START_TX:                // Start Transmissions State
        // Comm State should be set to Transmitting before we switch the transceiver and initiate
        //   transmissions to ensure the transmit ISR operates correctly
        ModB.CommState = MODB_TRANSMITTING;         // Set the comm state to Transmitting
        ModB.Comm_Timer = MODB_XMIT_TIMEOUT;        // Initialize the internal transmit timer
        // Switch the transceiver from receiving to transmitting.  Per SN65HVD70 data sheet, must delay at
        //   least 8usec for the receiver to be disabled and the transmitter to be enabled
        CAM2_RCVR_DISABLED;
        CAM2_DRVR_ENABLED;
        i = 250;                            // DAH: Measured delay on 220131 - 12.7usec min
        while (i > 0)
        {
          --i;
        }
        // Set up DMA channel to transmit: stream 6, Modbus TxBuf --> UART6 TX
        USART6->SR &= (~USART_SR_TC);             // Clear TC bit in UART6
        DMA2_Stream6->CR &= 0xFFFFFFFE;           // Make sure DMA channel is disabled before configuring it
        while (DMA2_Stream6->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
        {
        }
        // Must clear all event flags before initiating a DMA operation
        DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                                + DMA_HIFCR_CFEIF6);
        DMA2_Stream6->M0AR = (uint32_t)((uint8_t *)(&ModB.TxMsgBuf[0]));
        DMA2_Stream6->NDTR &= 0xFFFF0000;
        DMA2_Stream6->NDTR |= ModB.CharsToTx;     // Number of bytes in the Modbus transmit message
        DMA2_Stream6->CR |= 0x00000001;           // Initiate the DMA to transmit the data
        ModB.State = MBS_TX;
        mbs_exit = TRUE;
        break;

      case MBS_TX:                      // Transmitting State
/*      if (USART6->SR & USART_SR_TC)
      {
        TESTPIN_D1_HIGH;
      }
      else
      {
        TESTPIN_D1_LOW;
      }   */
        // Check whether done transmitting
        if ( (DMA2->HISR & DMA_HISR_TCIF6)
          && (USART6->SR & USART_SR_TC) )
        {
          // Reset the transfer complete interrupt flag
          // Note, we don't need to clear the enable bit.  It is cleared automatically, when the DMA has
          //   operation has been completed
          DMA2->HIFCR |= DMA_HISR_TCIF6;
          ModB.State = MBS_DONE;
//          TESTPIN_A3_HIGH;
        }
        else
        {
//          TESTPIN_A3_LOW;
          mbs_exit = TRUE;
          break;
        }

      case MBS_DONE:
        // Remove the message from the queue by resetting the character buffer index.  Note, other message
        //   setup (such as clearing flags, etc.) occurs in the interrupt when the first character is
        //   received
        ModB.RxMsgNdx = 0;
        ModB.RxIFrameBreak = FALSE;         // Clear the Frame Break flag
        ModB.State = MBS_IDLE;
        ModB.CommState = MODB_IDLE;
        CAM2_DRVR_DISABLED;                 // Set the transceiver to receive
        CAM2_RCVR_ENABLED;
        mbs_exit = TRUE;
        break;

      default:                          // Deafult state: this should never be entered
        ModB.Reset_Req = TRUE;
        mbs_exit = TRUE;
        break;

    }

  }

}
//------------------------------------------------------------------------------------------------------------
//            END FUNCTION             ModB_SlaveComm()
//------------------------------------------------------------------------------------------------------------






//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Modb_CheckRegAddress()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Check Modbus Register Address
//
//  MECHANICS:          This subroutine checks a Modbus register address range (given by the starting
//                      address and number of registers) in an incoming message to determine whether it is
//                      valid and what address group it is in.  It returns the address group (the table ID
//                      that contains the starting address in the message), the  offset in the table, the
//                      number of invalid addresses in the incoming address range before the first address
//                      match, and the number of valid addresses.
//                                          Example 1: Normal
//                      Starting address (start_reg_add) = 1202, Number of registers (num_reg) = 10
//                          group = 1, offset = 1, num_bad = 0, num_to_add = 10
//                      At this point, the calling subroutine can add 10 registers to the transmit buffer
//                      using the group 1 table.  It then should decrease the number of registers by 10, and
//                      increase the register address by 10.  Since the number of registers is now 0, we are
//                      finished.
//
//                                          Example 2: Crossing contiguous table boundaries
//                      Starting address (start_reg_add) = 4600, Number of registers (num_reg) = 30
//                          group = 9, offset = 104, num_bad = 0, num_to_add = 8
//                      At this point, the calling routine should add the eight registers to the transmit
//                      buffer using the group 9 table.  It then decreases the number of registers by 8
//                      (num_reg = 22), and increases the register address by 8 (start_reg_add = 4608).
//                      This subroutine is again called:
//                      Starting address (start_reg_add) = 4608, Number of registers (num_reg) = 22
//                          group = 10, offset = 0, num_bad = 0, num_to_add = 22
//                      At this point, the calling routine should add the 22 registers to the transmit
//                      buffer using the group 10 table.  It then decreases the number of registers by 22
//                      (num_reg = 0), and increases the register address by 22 (start_reg_add = 4630).
//                      Since the number of registers is now 0, we are finished.
//
//                                          Example 3: Crossing non-contiguous table boundaries
//                      Starting address (start_reg_add) = 6126, Number of registers (num_reg) = 38
//                          group = 16, offset = 526, num_bad = 0, num_to_add = 4
//                      At this point, the calling routine should add the four registers to the transmit
//                      buffer using the group 16 table.  It then decreases the number of registers by 4
//                      (num_reg = 34), and increases the register address by 4 (start_reg_add = 6130).
//                      This subroutine is again called:
//                      Starting address (start_reg_add) = 6130, Number of registers (num_reg) = 36
//                          group = 17, offset = 0, num_bad = 14, num_to_add = 20
//                      At this point, the calling routine should first place 14 zeros in the transmit
//                      buffer (assuming it is configured to do this) to cover the 14 bad addresses from
//                      6130 - 6143.  It then will add the 20 registers to the transmit buffer using the
//                      group 17 table.  It then decreases the number of registers by 34 (14 + 20)
//                      (num_reg = 0), and increases the register address by 20 (start_reg_add = 6150).
//                      Since the number of registers is now 0, we are finished.
//
//  CAVEATS:            None
//
//  INPUTS:             start_reg_add - starting register address
//                      num_reg - number of requested registers
//
//  OUTPUTS:            group - the table number that has the data addresses for the requested registers
//                      offset - the offset in the table of the data address for the first register
//                      num_bad - the number of invalid register addresses before the first valid address
//                      num_to_add - the number of registers to add from the table
//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Modb_CheckRegAddress(uint8_t *group, uint16_t *offset, uint8_t *num_bad, uint8_t *num_to_add,
                                    uint16_t start_reg_add, uint8_t num_reg)
{
  uint16_t end_reg_add;
  uint8_t i;

  end_reg_add = start_reg_add + num_reg - 1;
  // Step through the address groups in ascending order.  Check each group to find the lowest address group
  // that contains the address
  for (i=0; i < MODB_NUM_GROUPS; ++i)
  {
    // If some portion of the input address range is in the address group...
    if ( (start_reg_add <= MODB_GROUP_END_ADD[i]) && (end_reg_add >= MODB_GROUP_START_ADD[i]) )
    {
      *group = i;                                               // Save group number
      // If the start address is in the group:
      //   the offset is the (register address minus the group's starting address)/(num regs per object).
      //     The division compensates for entries that use more than one register (for example, each float
      //     data object uses two registers, so the offset is (reg_start - table_start)/2
      //   the number of invalid register addresses is zero
      //   the number of registers to add depends on the ending register address:
      //     if the ending address is in the group, it is the number of requested registers (num_reg)
      //     if not, then it is the number of registers from the starting address to the ending address
      if (start_reg_add >= MODB_GROUP_START_ADD[i])
      {
        *offset = (start_reg_add - MODB_GROUP_START_ADD[i])/MODB_NUM_REGS_PER_DATA_OBJECT[i];
        *num_bad = 0;
        *num_to_add = ( (end_reg_add <= MODB_GROUP_END_ADD[i]) ?
                            num_reg : (MODB_GROUP_END_ADD[i] - start_reg_add + 1) );
      }
      // If the start address is not in the group:
      //   the offset is zero
      //   the number of invalid register addresses is the number of registers from the starting address to
      //     the starting address in the group
      //   the number of registers to add depends on the ending register address:
      //     if the ending address is in the group, it is the number of registers from the group's starting
      //       address to the requested ending address
      //     if not, then it is the number of registers in the group
      else
      {
        *offset = 0;
        *num_bad = MODB_GROUP_START_ADD[i] - start_reg_add;
        *num_to_add = ( (end_reg_add <= MODB_GROUP_END_ADD[i]) ?
                            (end_reg_add - MODB_GROUP_START_ADD[i] + 1) : (MODB_GROUP_END_ADD[i] - MODB_GROUP_START_ADD[i] + 1) );
      }
      break;
    }
  }

  // If the address isn't in any of the groups:
  if (i == MODB_NUM_GROUPS)
  {
    *group = 255;                         // Set group to invalid number
    *offset = 0;                          // Set offset to 0
    *num_bad = 0;                         // Set the number of invalid registers to 0
    *num_to_add = 0;                      // Set the number of registers to add to 0
  }
        
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Modb_CheckRegAddress()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcFC0102Msg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Function Code 1 and 2 Messages
//
//  MECHANICS:          This subroutine processes received messages with function code 1 or 2.  These are
//                      Read messages.  The subroutine parses and decodes the message, checks for valid
//                      address range, and then assembles the response.
//
//  CAVEATS:            At the time of initial writing the PXR only has function 2 address, but since FC1
//                      and FC2 messages share the same format both are handled here.
//
//  INPUTS:             length: length of the input message
//                      fc: function code of the input message (x01 or x02)
//                      ModB.RxMsgBuf: input message
//                      Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling
//                              0  : insert 0 for invalid values (register not supported, for example)
//                              > 0: NAK if invalid value encountered
//                      MODB_OBJECT_ADDR_GRxx: table of variable addresses for a particular register group
//                      MODB_OBJECT_CONV_GRxx: table of conversion codes showing how a particular variable
//                              must be converted to be read by Modbus
//
//  OUTPUTS:            ModB.TxMsgBuf[]: output message
//                      ModB.CharsToTx: output message length
//
//  ALTERS:             None
//
//  CALLS:              Modb_CheckRegAddress(), Modb_FormatStoreVal(), CalcCRC()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ProcFC0102Msg(uint8_t length, uint8_t fc)                    
{
  uint16_t start_reg_add, end_reg_add, offset, ndx, num_reg;
  uint8_t  errcode;

  // Modbus input message (ModB.RxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x01 or 0x04
  //   Data Field:
  //     [2]: Register Address high byte
  //     [3]: Register Address low byte
  //     [4]: Number of Registers high byte (registers are outputs or inputs)
  //     [5]: Number of Registers low byte (registers are outputs or inputs)
  //   [6]: CRC low byte
  //   [7]: CRC high byte

  // Modbus response message (ModB.TxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x01 or 0x02
  //   Data Field:
  //     [2]: Byte count (number of registers/8, if remainder is non-zero add 1)
  //     [3]: up to 8 input/output bits
  //        ............
  //     [3+ (byte count - 1)]: up to 8 additional input/output bits
  //   [3+byte count]: CRC low byte
  //   [3+byte count+1]: CRC high byte

  // Initialize working variables
  start_reg_add = ( (((uint16_t)ModB.RxMsgBuf[2]) << 8) | ModB.RxMsgBuf[3] );   // Register address
  num_reg = ( (((uint16_t)ModB.RxMsgBuf[4]) << 8) | ModB.RxMsgBuf[5] );         // Assemble the number of regs 
  end_reg_add = start_reg_add + num_reg - 1;
  errcode = 0;                          // Initialize errcode to no error

  // Initialize first few registers assuming the message is good
  ModB.TxMsgBuf[0] = Setpoints2.stp.Modbus_Port_Addr;  // Char 0: Device Address (char index = 1)
  ModB.TxMsgBuf[1] = fc;                  // Char 1: Function Code
  ModB.TxMsgBuf[2] = num_reg / 8;         // Char 2: Byte Count (char index = 3)
  if ((num_reg % 8) != 0)                 // not a multiple of 8?
  {
    ModB.TxMsgBuf[2] += 1;               // need another byte for remaining bits
  }

  ndx = 3;                                // Chars 3..N: Data bytes (begin at char index = 4)

  // Check number of registers and length
  if ( (num_reg > 125) || (num_reg == 0) || (length != 8) )
  {
    errcode = NAK_ILLEGAL_DATA_VAL;
  }

  // Verify register addresses are in range. This is a pared-down version of Mdob_CheckRegAddress()
  // specific for functions codes 1 and 2 because there are a limited number of contiguous addresses
  // at the time of implementation. If more registers are added to complicate the process this may be
  // broken out into a subroutine.

// TODO: Verify that we want "forgiving" to include allowing FC1 to respond for FC2 register range
  // first handle Function Code 1. At initial writing there are no function code 1 registers defined
  // so if "unforgiving" we'll NAK.
  else if ((fc == 1) && (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling |= 0))
  {
    errcode = NAK_ILLEGAL_DATA_ADDR;
  }
  // next check if ALL requested registers are outside the supported range and NAK if so
  else if ((start_reg_add > MODB_FC2_END_ADD) || (end_reg_add < MODB_FC2_START_ADD))
  {
    errcode = NAK_ILLEGAL_DATA_ADDR;
  }
  // for "unforgiving" check if ANY requested registers are outside the supported range and NAK if so
  else if ((Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling |= 0) &&
           ((start_reg_add < MODB_FC2_START_ADD) || (end_reg_add > MODB_FC2_END_ADD)))
  {
    errcode = NAK_ILLEGAL_DATA_ADDR;
  }
  else // everything is OK so far, generate the response
  {
    uint8_t bit_val = 0;
    uint8_t bit_cnt = 0;
    uint16_t curr_add = start_reg_add;
    uint16_t index = 0;
    ModB.TxMsgBuf[ndx] = 0; // clear the first response byte
    while (num_reg > 0)
    {
      // outside supported addresses?
      if ((curr_add < MODB_FC2_START_ADD) || (curr_add > MODB_FC2_END_ADD))
      {
        bit_val = 0;
      }
      else
      {
        index = curr_add - MODB_FC2_START_ADD; // index into lookup tables
        switch (index)
        {
          case 0:  // Status - Breaker Closed
            bit_val = (OpenFlg == 0) ? 1 : 0;
            break;
          case 1:  // Status - Unacknowledged trip
            bit_val = ((Trip_Flags0.all & TRIP_MASK_FLG0) ||
                       (Trip_Flags1.all & TRIP_MASK_FLG1))? 1 : 0;
            break;
          case 2:  // Status - Unacknowledged alarm
            bit_val = ((Alarm_Flags0.all & ALARM_MASK_FLG0) ||
                       (Alarm_Flags1.all & ALARM_MASK_FLG1) ||
                       (Alarm_Flags2.all & ALARM_MASK_FLG2) )? 1 : 0;
            break;
          case 4:  // Status - Maintenance mode
            bit_val = ((Setpoints0.stp.MM_Enable & BIT8) == 0) ? 0 : 1;
            break;
          case 5:  // Status - Test mode
            bit_val = ((TU_State_TestMode == 1) || (TU_State_TestUSBMode == 1));
            break;
          case 8:  // Status - Phase rotation is ABC
            bit_val = (ABC_Rotation == 0) ? 0 : 1;
            break;
          case 9:  // Status - Long delay pickup
            bit_val = ((TripPuFlags.all & TRIP_PU_MASK) || LdPuAlmFlg) ? 1 : 0;
            break;
          case 10:  // Status - Zone interlock active
            bit_val = Zin_Latched;
            break;
          case 12:  // Status - "Ground" is source gnd
            bit_val = (Setpoints1.stp.Gnd_Type == 0)? 0 : 1;
            break;
          case 16:  // Validity - Breaker Closed - Always 1
          case 17:  // Validity - Unacknowledged trip - Always 1
          case 18:  // Validity - Unacknowledged alarm - Always 1
          case 20:  // Validity - Maintenance mode - Always 1
          case 21:  // Validity - Test mode - Always 1
          case 24:  // Validity - Phase rotation is ABC - Always 1
          case 25:  // Validity - Long delay pickup - Always 1
          case 26:  // Validity - Zone interlock active - Always 1
          case 28:  // Validity - "Ground" is source gnd - Always 1
            bit_val = 1;
            break;
          default:  // undefined or unimplemented - Always 0
            bit_val = 0;
            break;
        }
      }
      // put the bit value into the byte to transmit in the appropriate position
      ModB.TxMsgBuf[ndx] |= (bit_val << bit_cnt);
      num_reg--;
      curr_add++;
      if (num_reg > 0)  // more to go?
      {
        // increment to next bit position
        if (bit_cnt < 7)
        {
          bit_cnt++;
        }
        else
        {
          // go to lsb of next byte
          bit_cnt = 0;
          ndx++;
          ModB.TxMsgBuf[ndx] = 0; // clear the next response byte
        }
      }
    }
  }
  ndx++; // increment index since we completed all data
  // If errcode is nonzero, respond with a NAK
  if (errcode != 0)
  {                                         // Char 0: Device Address (already loaded)
    ModB.TxMsgBuf[1] |= 0x80;               // Char 1: Function Code (already loaded) with msb set
    ModB.TxMsgBuf[2] = errcode;             // Char 2: Exception Code
    ndx = 3;
  }
  // Compute CRC of message and load into transmit buffer.  Note, the CRC is transmitted low byte, then high
  //   byte.
  offset = CalcCRC(&ModB.TxMsgBuf[0], ndx);         // offset used as temp to hold CRC
  ModB.TxMsgBuf[ndx++] = (uint8_t)(offset);
  ModB.TxMsgBuf[ndx++] = (uint8_t)(offset>>8);

  // Save number of chars to transmit
  ModB.CharsToTx = ndx;
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ProcFC0102Msg()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcFC0304Msg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Function Code 3 and 4 Messages
//
//  MECHANICS:          This subroutine processes received messages with function code 3 or 4.  These are
//                      Read messages.  The subroutine parses and decodes the message, and then assembles
//                      the response.
//
//  CAVEATS:            None
//
//  INPUTS:             length: length of the input message
//                      fc: function code of the input message (x03 or x04)
//                      ModB.RxMsgBuf: input message
//                      Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling
//                              0  : insert 0 for invalid values (register not supported, for example)
//                              > 0: NAK if invalid value encountered
//                      MODB_OBJECT_ADDR_GRxx: table of variable addresses for a particular register group
//                      MODB_OBJECT_CONV_GRxx: table of conversion codes showing how a particular variable
//                              must be converted to be read by Modbus
//
//  OUTPUTS:            ModB.TxMsgBuf[]: output message
//                      ModB.CharsToTx: output message length
//
//  ALTERS:             None
//
//  CALLS:              Modb_CheckRegAddress(), Modb_FormatStoreVal(), CalcCRC()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ProcFC0304Msg(uint8_t length, uint8_t fc)                    
{
  uint16_t start_reg_add, offset, ndx, num_reg;
  uint16_t tmp16;
  uint8_t  errcode, endcnt, i;
  uint8_t group, num_bad, num_to_add;
  uint8_t conversion_code;
  uint8_t numsetp;                                      //VARUN
  uint16_t *sptr;                                       //VARUN
  uint8_t SetpGrpNum;
  uint8_t CurSetpSet;

  void * const *objaddr_ptr;
  uint8_t const *cc_ptr;
  uint16_t *harmonic_valptr;
  union float_u32
  {
    float fval;
    uint32_t uval;
  } harmonic_tmpval;

  // Modbus input message (ModB.RxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x03 or 0x04
  //   Data Field:
  //     [2]: Register Address high byte
  //     [3]: Register Address low byte
  //     [4]: Number of Registers high byte
  //     [5]: Number of Registers low byte
  //   [6]: CRC low byte
  //   [7]: CRC high byte

  // Modbus response message (ModB.TxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x03 or 0x04
  //   Data Field:
  //     [2]: Byte count
  //     [3]: Register Address low byte
  //     [4]: Register n data high byte
  //     [5]: Register n data low byte
  //        ............
  //     [3+num]: Register n+num-1 data high byte
  //     [3+num+1]: Register n+num-1 data low byte
  //   [3+num+2]: CRC low byte
  //   [3+num+3]: CRC high byte

  
  // Initialize working variables
  start_reg_add = ( (((uint16_t)ModB.RxMsgBuf[2]) << 8) | ModB.RxMsgBuf[3] );   // Register address
  num_reg = ( (((uint16_t)ModB.RxMsgBuf[4]) << 8) | ModB.RxMsgBuf[5] );         // Assemble the number of regs 
  errcode = 0;                          // Initialize errcode to no error

  // Initialize first few registers assuming the message is good
  ModB.TxMsgBuf[0] = Setpoints2.stp.Modbus_Port_Addr; // Char 0: Device Address (char index = 1)
  ModB.TxMsgBuf[1] = fc;                  // Char 1: Function Code
  ModB.TxMsgBuf[2] = num_reg * 2;         // Char 2: Byte Count (char index = 3)

  ndx = 3;                                // Chars 3..N: Data bytes (begin at char index = 4)

  // Check number of registers and length
  if ( (num_reg > 125) || (num_reg == 0) || (length != 8) )
  {
    errcode = NAK_ILLEGAL_DATA_VAL;
  }

  // Assemble the response until we are done (num_reg == 0) or we decide to send a NAK (errcode != 0)
  while ( (num_reg > 0) && (errcode == 0x00) )
  {
    // Call subroutine to translate the device address and number of registers into a Modbus Data Object
    // Table location, along with other Tx buffer loading information
    // Function inputs:
    //   start_reg_add - starting register address
    //   num_reg - total number of requested registers
    // Function outputs:
    //   group - the table number that has the data addresses for the requested registers
    //   offset - the offset in the table of the data address for the first register
    //   num_bad - the number of invalid register addresses before the first valid address
    //   num_to_add - the number of registers to add from the table
    Modb_CheckRegAddress(&group, &offset, &num_bad, &num_to_add, start_reg_add, num_reg);
    // If there are invalid addresses before the first valid address, either
    //   - Forgiving Mode: fill the locations with 0
    //   - Unforgiving Mode: set errcode to NAK_ILLEGAL_ADDRESS (we will NAK)
    if (num_bad > 0)
    {
      if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill with zeros
      {
        endcnt = num_bad * 2;               // Multiply by 2 because each register is two bytes
        for (i=0; i<endcnt; ++i)
        {
          ModB.TxMsgBuf[ndx++] = 0;
        }
      }
      else                                // Unforgiving mode - set errcode for illegal address
      {                                   
        errcode = NAK_ILLEGAL_DATA_ADDR;
        break;                              // Break out of the loop since ew are going to NAK
      }
    }
    // Update number of registers left and starting register address
    //   Note, per Modb_CheckRegAddress(), num_reg is always >= (num_bad + num_to_add)
    num_reg -= num_bad;
    start_reg_add += num_bad;

    objaddr_ptr = MODB_OBJECT_ADDR[group];
    cc_ptr = MODB_OBJECT_CONV_ADDR[group];
    // Switch into the group that has the data addresses for the registers
    switch (group)
    {
      case 2:  // Modbus object handling configuration registers
      case 8:  // Modbus communications configuration registers
//      case 52:                            // Modbus object handling configuration registers   *** DAH  SHOULD ALTERNATE ADDRESS BE SUPPORTED?
        // These Modbus configuration registers are implemented as Group 2 setpoints in the PXR35
        // (coincidence that Modbus address group 2 contains data for Setpoints Group 2).
        // If there are objects to add (num_to_add > 1), add the data objects into the transmit
        // buffer. All objects in this group are U16 objects, so no conversion is necessary. There is one register per object.
        for (i = 0; i < num_to_add; ++i)
        {
          if (group == 2)
          {
            switch (offset)
            {
              case 0:
                tmp16 = Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling;
                break;
              case 1:
                tmp16 = Setpoints2.stp.Modbus_RTU_Floating_Pt_Word_Order;
                break;
              default:
                tmp16 = Setpoints2.stp.Modbus_RTU_Fixed_Pt_Word_Order;
                break;
            }
          }
          else
          {
            switch (offset)
            {
              case 0:
                tmp16 = Setpoints2.stp.Modbus_Port_Addr;
                break;
              case 1:
                tmp16 = Setpoints2.stp.Modbus_Port_Baudrate;
                break;
              case 2:
                tmp16 = Setpoints2.stp.Modbus_Port_Parity;
                break;
              default:
                tmp16 = Setpoints2.stp.Modbus_Port_Stopbit;
                break;
            }
          }
          ModB.TxMsgBuf[ndx++] = (uint8_t)(tmp16 >> 8); // High byte
          ModB.TxMsgBuf[ndx++] = (uint8_t)(tmp16);        // Low byte
          offset++;
        }
        // Update register counts and starting register address
        num_reg -= num_to_add;
        start_reg_add += num_to_add;
        num_to_add = 0;
        break;
      case 7:                             // Setpoints
        SetpGrpNum = (uint8_t)(ModB_CurSetGrp >> 8);        // Group Number stored in high 8 bits
        CurSetpSet = (uint8_t)(ModB_CurSetGrp);             // Set Number stored in low 8 bits
// TODO: Need to validate SetpGrpNum BEFORE using it???
        numsetp = ((SETP_GR_SIZE[SetpGrpNum] >> 1) - 2);
        tmp16 = start_reg_add % 0x0BB9;
        // Set the readable set to the active set if an invalid set is there
        CurSetpSet = ( (CurSetpSet < 4) ? (CurSetpSet) : SetpActiveSet);
        if(SetpGrpNum >= NUM_STP_GROUPS) {                  // If the group number is out of bounds, return
          ModB.TxMsgBuf[ndx++] = 0;                         //   current set only
          ModB.TxMsgBuf[ndx++] = CurSetpSet;
          num_to_add--;
          num_reg--;
          start_reg_add++;
          offset++;
        }
        else {
          if(tmp16 == 0x0BB7)                               // Password register, show 0
          {
            ModB.TxMsgBuf[ndx++] = 0;
            ModB.TxMsgBuf[ndx++] = 0;
            tmp16++;
            num_to_add--;
            num_reg--;
            start_reg_add--;
            offset++;
          }
          if(tmp16 == 0x0BB8 && num_to_add > 0)             // Setpoint group/set register, return group and
          {                                                 //   set
            ModB.TxMsgBuf[ndx++] = SetpGrpNum;
            ModB.TxMsgBuf[ndx++] = CurSetpSet;
            tmp16 = 0;
            num_to_add--;
            num_reg--;
            start_reg_add++;
            offset++;
          } 
          if (tmp16 >= numsetp)                             // Check if requested offset is larger than
                                                            //   number of setpoints in the setpoint group
          {                                   
            if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0)                  
            {
              while (num_to_add > 0)
              {
                ModB.TxMsgBuf[ndx++] = 0;               
                ModB.TxMsgBuf[ndx++] = 0;
                num_to_add--;                           
                num_reg--;                              
                start_reg_add++;                        
              }
              offset++;                                 
            }                                           
            else                                      
            {  
              errcode = NAK_ILLEGAL_DATA_ADDR;
              break;
            }
          }
          i = 0;
          if(CurSetpSet == SetpActiveSet)
          {
            sptr = (SETP_GR_DATA_ADDR[SetpGrpNum]);
          }
          else
          {
            Get_Setpoints(CurSetpSet, (uint8_t)SetpGrpNum, (uint8_t *)(&i));
            sptr = &SetpScratchBuf[0];
          }
          while(num_to_add > 0 && tmp16 < numsetp)                      // read the requested amount of
          {                                                             //   setpoints within the number of
            ModB.TxMsgBuf[ndx++] = (uint8_t)((sptr[tmp16]) >> 8);     //   setpoints
            ModB.TxMsgBuf[ndx++] = (uint8_t)(sptr[tmp16++]);
            num_to_add--;
            num_reg--;
            start_reg_add++;
          }
          offset++;
        }
        if (num_to_add > 0)                 // If there are still registers left, they are illegal (a
        {                                   //  partial object).  Either fill with zeros or NAK
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while (num_to_add > 0)
            {
              ModB.TxMsgBuf[ndx++] = 0;               // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                           // Decrement the number of regs to add in this pass
              num_reg--;                              // Decrement the total number of registers left to add
              start_reg_add++;                        // Increment the starting register number
            }
            offset++;                                 // Increment the table index of the first reg to add
          }                                           //   since its data is torn and we are filling with 0
          else                                        // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
          }
        }
        break;
      case 21:                            // Real-time data values - fixed point
      case 23:                            // Real-time data values - fixed point
      case 25:                            // Real-time data values - fixed point
      case 28:                            // Real-time data values - fixed point
      case 60:                            // Real-time data values - fixed point
      case 72:                            // Real-time data values - fixed point
      case 24:                            // Real-time data values - fixed point 64-bit energy
      case 70:                            // Real-time data values - fixed point 64-bit energy
      case 10:                            // Real-time data values - floating point
      case 11:                            // Real-time data values - floating point
      case 12:                            // Real-time data values - floating point
      case 15:                            // Real-time data values - floating point
      case 42:                            // Real-time data values - floating point
      case 54:                            // Real-time data values - floating point
      case 22:                            // Real-time data values - fixed point 32-bit energy
      case 69:                            // Real-time data values - fixed point 32-bit energy
        // Check starting address validity.  It must be on the boundary of an object.  For 32-bit objects,
        //   (object size = 2 regs) that means it must be even.  For 64-bit objects (object size = 4 regs),
        //   it must be a multiple of 4.
        //   If we are in forgiving mode, we will fill the invalid registers with zero, and adjust the
        //   number of requested registers and starting address accordingly.  If in unforgiving mode and
        //   there is an invalid register, just NAK
        tmp16 = ( (MODB_NUM_REGS_PER_DATA_OBJECT[group] == 4) ? 0x0003 : 0x0001);   // bitmask to check addr
        if ((start_reg_add & tmp16) != 0x0000)   // If the starting address is invalid 
        {
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while ( ((start_reg_add & tmp16) != 0x0000) && (num_to_add > 0) )
            {
              ModB.TxMsgBuf[ndx++] = 0;               // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                           // Decrement the number of regs to add in this pass
              num_reg--;                              // Decrement the total number of registers left to add
              start_reg_add++;                        // Increment the starting register number
            }
            offset++;                                 // Increment the table index of the first reg to add
          }                                           //   since its data is torn and we are filling with 0
          else                                      // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
            break;                                    // We are done so exit
          }
        }
        // We now have a valid starting register address.  If there are still objects to add
        //   (num_to_add > 1), add the data objects into the transmit buffer using the Group x table,
        //   beginning at "offset".
        // Initialize endcnt to the number of objects to place in the Tx buffer.  This is the number of 
        //   registers to add in this pass divided by the number of registers per object.  If there are
        //   objects left over, the last object will not be added.  We will either fill the remaining
        //   registers with zeros or NAK.
        // Remember, all data objects in a group of registers are the same size!
        endcnt = num_to_add/MODB_NUM_REGS_PER_DATA_OBJECT[group];
        for (i = 0; i < endcnt; ++i)
        {
          // Conversion code calls for scaling multiplication and a format change only if fixed point
          //   register is requested.  Otherwise no format change
          // Conversion codes for fixed versus floating point are aligned,  Floating point conversion
          //   codes = fixed point conversion code + 20
          if ( (group < 16) || (group == 42) || (group == 54) )
          {
            conversion_code = cc_ptr[i+offset] + 20;
          }
          else
          {
            conversion_code = cc_ptr[i+offset];
          }
          // If requested register is not supported and in unforgiving mode, NAK and abort
          if ( ((conversion_code == 99) || (conversion_code == 119)) &&
                (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling != 0) )
          {
            errcode = NAK_ILLEGAL_DATA_ADDR;
            break;
          }  
          Modb_FormatStoreVal(conversion_code, objaddr_ptr[i+offset], &ModB.TxMsgBuf[ndx]);
          ndx += (MODB_NUM_REGS_PER_DATA_OBJECT[group] * 2);
        }
        // Update the number of registers left to add in this pass, the total number of registers left to
        //   add, and the starting register address
        num_to_add -= (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);
        num_reg -= (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);
        start_reg_add += (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);

        if (num_to_add > 0)                 // If there are still registers left, they are illegal (a
        {                                   //  partial object).  Either fill with zeros or NAK
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while (num_to_add > 0)
            {
              ModB.TxMsgBuf[ndx++] = 0;               // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                           // Decrement the number of regs to add in this pass
              num_reg--;                              // Decrement the total number of registers left to add
              start_reg_add++;                        // Increment the starting register number
            }
            offset++;                                 // Increment the table index of the first reg to add
          }                                           //   since its data is torn and we are filling with 0
          else                                      // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
          }
        }
        break;

      case 29:                            // Real-time data values with time stamp - fixed point      
      case 30:                            // Real-time data values with time stamp - fixed point      
      case 31:                            // Real-time data values with time stamp - fixed point      
      case 32:                            // Real-time data values with time stamp - fixed point      
      case 33:                            // Real-time data values with time stamp - fixed point      
      case 71:                            // Real-time data values with time stamp - fixed point      
      case 55:                            // Real-time data values with time stamp - fixed point      
      case 56:                            // Real-time data values with time stamp - fixed point      
      case 57:                            // Real-time data values with time stamp - fixed point      
      case 58:                            // Real-time data values with time stamp - fixed point      
      case 59:                            // Real-time data values with time stamp - fixed point      
      case 61:                            // Real-time data values with time stamp - fixed point      
      case 62:                            // Real-time data values with time stamp - fixed point      
      case 63:                            // Real-time data values with time stamp - fixed point      
      case 64:                            // Real-time data values with time stamp - fixed point      
      case 65:                            // Real-time data values with time stamp - fixed point      
      case 66:                            // Real-time data values with time stamp - fixed point      
      case 67:                            // Real-time data values with time stamp - fixed point      
      case 68:                            // Real-time data values with time stamp - fixed point      
      case 16:                            // Real-time data values with time stamp - floating point
      case 17:                            // Real-time data values with time stamp - floating point
      case 18:                            // Real-time data values with time stamp - floating point
      case 19:                            // Real-time data values with time stamp - floating point
      case 20:                            // Real-time data values with time stamp - floating point
      case 53:                            // Real-time data values with time stamp - floating point
      case 37:                            // Real-time data values with time stamp - floating point
      case 38:                            // Real-time data values with time stamp - floating point
      case 39:                            // Real-time data values with time stamp - floating point
      case 40:                            // Real-time data values with time stamp - floating point
      case 41:                            // Real-time data values with time stamp - floating point
      case 43:                            // Real-time data values with time stamp - floating point
      case 44:                            // Real-time data values with time stamp - floating point
      case 45:                            // Real-time data values with time stamp - floating point
      case 46:                            // Real-time data values with time stamp - floating point
      case 47:                            // Real-time data values with time stamp - floating point
      case 48:                            // Real-time data values with time stamp - floating point
      case 49:                            // Real-time data values with time stamp - floating point
      case 50:                            // Real-time data values with time stamp - floating point
        // These groups are special cases.  Each object is considered to be a value plus its associated
        //   timestamp.  As far as Modb_CheckRegAddress() is concerned, these are single objects that are 6
        //   bytes long.  In addition, when Modb_CheckRegAddress() returns "offset", the offset in the data
        //   table of the address for the first register, it will be off by a factor of two, because the
        //   data table contains two addresses per object (value and timestamp), and Modb_CheckRegAddress()
        //   believes there are single 6-byte objects.
        // This code uses "6" instead of MODB_NUM_REGS_PER_DATA_OBJECT[group] to eliminate code and reduce
        //   execution time.  Every object (data and timestamp) is 6 bytes long
        //
        // Check starting address validity.  It must be on the boundary of an object.  An object here is
        //   considered to be a value plus its associated timestamp.  Therefore, the starting address must
        //   be on a 6-byte boundary.
        //   If we are in forgiving mode, we will fill the invalid registers with zero, and adjust the
        //   number of requested registers and starting address accordingly.  If in unforgiving mode and
        //   there is an invalid register, just NAK
        tmp16 = start_reg_add - MODB_GROUP_START_ADD[group];       // tmp16 gets offset of starting address
        offset = (offset << 1);             // Multiply offset by two since two table entries per object
        // i has the number of registers beyond a valid address -  0 < i < 5.  "0" - starting address is ok
        i = tmp16 % 6;
        if (i > 0)                                                 // If the starting address is invalid
        {
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid registers with zeros
          {
            // i now has the number of registers to fill with zero before we a reach valid address
            i = 6 - i;
            // Fill invalid regs with zero until either no more invalid or number of regs to add is reached
            while ( (i > 0) && (num_to_add > 0) )
            {
              ModB.TxMsgBuf[ndx++] = 0;             // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                         // Decrement the number of regs to add in this pass
              num_reg--;                            // Decrement the total number of registers left to add
              start_reg_add++;                      // Increment the starting register number
              tmp16++;                              // Increment the offset
              --i;
            }
            offset += 2;                            // Move the offset to the next object
          }
          else                              // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
            break;                                    // We are done so exit
          }
        }
        // We now have a valid starting register address.  If there are still objects to add
        //   (num_to_add > 1), add the data objects into the transmit buffer using the Group x data table,
        //   beginning at "offset".
        // Initialize endcnt to the number of objects to place in the Tx buffer.  This is the number of 
        //   registers to add in this pass divided by the number of registers per object.  If there are
        //   objects left over, the last object will not be added.  We will either fill the remaining
        //   registers with zeros or NAK.
        // Remember, all data objects in a group of registers are the same size (6 registers: 2 for the
        //   value plus 4 for the timestamp)!
        // For these groups, an object is the value plus timestamp.  We need to make (2 * endcnt) passes
        //   because each object needs two retrievals
        endcnt = num_to_add/6;
        for (i = 0; i < (endcnt * 2); ++i)
        {
          // Conversion code calls for scaling multiplication and a format change only if fixed point
          //   register is requested.  Otherwise no format change
          // Conversion codes for fixed versus floating point are aligned,  Floating point conversion
          //   codes = fixed point conversion code + 20
          // This comparison statement is written to use as few checks as possible.  It includes groups that
          //   are not in the floating point groups.  That is ok, because the switch has already weeded out
          //   those groups.  As long as it doesn't include any of ht efixed point groups, it is ok
          if ( (group <= 20) || ((group >= 37) && (group <= 53)) )
          {
            conversion_code = cc_ptr[i+offset] + 20;
          }
          else
          {
            conversion_code = cc_ptr[i+offset];
          }
          // If requested register is not supported and in unforgiving mode, NAK and abort
          if ( ((conversion_code == 99) || (conversion_code == 119)) &&
                (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling > 0) )
          {
            errcode = NAK_ILLEGAL_DATA_ADDR;
            break;
          }  
          Modb_FormatStoreVal(conversion_code, objaddr_ptr[i+offset], &ModB.TxMsgBuf[ndx]);
          if ( (conversion_code == 50) || (conversion_code == 70) )
          {
            ndx += 8;
          }
          else
          {
            ndx += 4;
          }
        }
        // Update the number of registers left to add in this pass, the total number of registers left to
        //   add, and the starting register address
        num_to_add -= (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);
        num_reg -= (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);
        start_reg_add += (MODB_NUM_REGS_PER_DATA_OBJECT[group] * endcnt);

        // At this point, we have entered all value/timestamp pairs (6-register sets)
        // Check whether we need to add the Time of Last Reset.  This will be the case if we are at the end
        //   of the group and we have 4 or more registers to add
        if ( (num_reg >= 4) && (start_reg_add == (MODB_GROUP_END_ADD[group] - 3)) )
        {
          Modb_FormatStoreVal(50, objaddr_ptr[i+offset], &ModB.TxMsgBuf[ndx]);
          ndx += 8;
          // Update everything for the new entry
          num_to_add -= 4;
          num_reg -= 4;
          start_reg_add += 4;
        }

        if (num_to_add > 0)                 // If there are still registers left, they are illegal (a
        {                                   //  partial object).  Either fill with zeros or NAK
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while (num_to_add > 0)
            {
              ModB.TxMsgBuf[ndx++] = 0;               // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                           // Decrement the number of regs to add in this pass
              num_reg--;                              // Decrement the total number of registers left to add
              start_reg_add++;                        // Increment the starting register number
            }
            offset++;                                 // Increment the table index of the first reg to add
          }                                           //   since its data is torn and we are filling with 0
          else                                    // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
          }
        }
        break;

      case 14:                            // Floating point Harmonics
      case 27:                            // Fixed point Harmonics
// TODO: make a major part of this a subroutine that differentiates fixed/floating for MB_Harmonics_Selection
        // Check starting address validity.  It must be on the boundary of an object.  For Harmonics,
        //   (object size = 2 regs) that means it must be even.
        //   If we are in forgiving mode, we will fill the invalid registers with zero, and adjust the
        //   number of requested registers and starting address accordingly.  If in unforgiving mode and
        //   there is an invalid register, just NAK
        if ((start_reg_add & 0x0001) != 0x0000)   // If the starting address is invalid 
        {
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while ( ((start_reg_add & 0x001) != 0x0000) && (num_to_add > 0) )
            {
              ModB.TxMsgBuf[ndx++] = 0;               // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                           // Decrement the number of regs to add in this pass
              num_reg--;                              // Decrement the total number of registers left to add
              start_reg_add++;                        // Increment the starting register number
            }
            offset++;                                 // Increment the table index of the first reg to add
          }                                           //   since its data is torn and we are filling with 0
          else                                      // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;          // Set error code to NAK
            break;                                    // We are done so exit
          }
        }
        // We now have a valid starting register address.  If there are still objects to add
        //   (num_to_add > 1), add the data objects into the transmit buffer using the Group x table,
        //   beginning at "offset".
        // Initialize endcnt to the number of objects to place in the Tx buffer.  This is the number of 
        //   registers to add in this pass divided by the number of registers per object.  If there are
        //   objects left over, the last object will not be added.  We will either fill the remaining
        //   registers with zeros or NAK.
        // Remember, all data objects in a group of registers are the same size!
        endcnt = num_to_add/2;
        if (endcnt > 0)                 // If there are objects to return
        {
          // First check whether we need to return the selection register value.  This is a special case.  The
          // value is return as a U32 regardless of whether the fixed or floating register was requested.  It
          // is converted to a float value so that it can be stored via Modb_FormatStoreVal()
          // Note, since this code is custom for the harmonics we use "2" instead of
          // MODB_NUM_REGS_PER_DATA_OBJECT[group]
          if (offset == 0)                  // If offset is zero, return the selection register value
          {
            harmonic_tmpval.fval = (float)MB_Harmonics_Selection;
            Modb_FormatStoreVal(0, &harmonic_tmpval.fval, &ModB.TxMsgBuf[ndx]);
            ndx += 4;
            num_to_add -= 2;                  // Decrement the number of regs to add in this pass
            endcnt--;                         // Decrement the number of objects to place in the Tx buffer
            num_reg -=2;                      // Decrement the total number of registers left to add
            start_reg_add += 2;               // Increment the starting register address
          }
          // If the first register is not the selection register, decrement offset because offset includes
          // the selection register in the register map, and we want the offset beginning with the first
          // harmonic
          else
          {
            offset--;
          }
        }
        // The harmonics objects to be returned depend on the selection register value:
        //     0 - 10: Active Harmonics (HarmonicsAgg)
        //        0=Ia    1=Ib    2=Ic    4=In    5=Vab    6=Vbc    7=Vca    8=Van    9=Vbn   10=Vcn
        //     20 - 30: Captured Harmonics (HarmonicsCap)
        //       20=Ia   21=Ib   22=Ic   24=In   25=Vab   26=Vbc   27=Vca   28=Van   29=Vbn   30=Vcn
        // The MODB_OBJECT_ADDR_GR14 table includes MB_Harmonics_Selection register, so the aggregated
        //   harmonics addresses are at index 1-11.  Therefore, increment selection by 1.  The captured
        //   harmonics addresses are at index 12-22, so decrement selection by 8.
        // Note, if MB_Harmonics_Selection = 3 or 23, In will be used.  This is an error condition that
        //   should never occur
        if (MB_Harmonics_Selection < 11)
        {
          harmonic_valptr = MODB_OBJECT_ADDR_GR14[MB_Harmonics_Selection + 1];
        }
        else if (MB_Harmonics_Selection < 31)
        {
          harmonic_valptr = MODB_OBJECT_ADDR_GR14[MB_Harmonics_Selection - 8];
        }
        else
        {
          errcode = NAK_ILLEGAL_DATA_ADDR;
          break;
        }

        // Now insert the harmonics.  "Offset" contains the offset in the array of the first object
        for (i = 0; i < endcnt; ++i)
        {
        // For fixed point values, the format is % x 100 (hundredths of a percent, 10000 = 100%).  The
        // internal values are in tenths of a per cent, so we need to multiply the values by 10.
        // For floating point values, the format is %, so we need to divide the values by 10.
          harmonic_tmpval.fval = (float)(harmonic_valptr[i+offset]);
          if (group == 14)              // Floating point format
          {
            harmonic_tmpval.fval = harmonic_tmpval.fval/10.0;
            Modb_FormatStoreVal(23, &harmonic_tmpval.fval, &ModB.TxMsgBuf[ndx]);
          }
          else                          // Fixed point format
          {
            harmonic_tmpval.fval = harmonic_tmpval.fval * 10.0;
            Modb_FormatStoreVal(0, &harmonic_tmpval.fval, &ModB.TxMsgBuf[ndx]);
          }
          ndx += 4;
        }
        // Update the number of registers left to add in this pass, the total number of registers left to
        //   add, and the starting register address   ("2" is the number of registers per object)
        num_to_add -= (2 * endcnt);
        num_reg -= (2 * endcnt);
        start_reg_add += (2 * endcnt);
        // If there are still registers left, they are illegal (a partial object).  Either fill with zeros
        // or NAK
        if (num_to_add > 0)
        {
          if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill invalid regs with zeros
          {
            while (num_to_add > 0)
            {
              ModB.TxMsgBuf[ndx++] = 0;             // Fill data buffer with 0
              ModB.TxMsgBuf[ndx++] = 0;
              num_to_add--;                         // Decrement the number of regs to add in this pass
              num_reg--;                            // Decrement the total number of registers left to add
              start_reg_add++;                      // Increment the starting register number
            }
            offset++;                               // Increment the table index of the first reg to add
          }                                         //   since its data is torn and we are filling with 0
          else                                  // Unforgiving Mode - set errcode for illegal address
          {  
            errcode = NAK_ILLEGAL_DATA_ADDR;        // Set error code to NAK
          }
        }
        break;

      default:                            // Invalid address
        // If the address is invalid, either:
        //   - Forgiving Mode: fill the locations with 0
        //   - Unforgiving Mode: set errcode to NAK_ILLEGAL_ADDRESS (we will NAK)
        // Note, num_bad and num_to_add are both set to zero in this case
        if (Setpoints2.stp.Modbus_RTU_Inval_Obj_Handling == 0) // Forgiving Mode - fill reg with zeros
        {
          for (i=0; i<num_reg; ++i)
          {
            ModB.TxMsgBuf[ndx++] = 0;
            ModB.TxMsgBuf[ndx++] = 0;
          }
          num_reg = 0;
        }
        else 
        {
          errcode = NAK_ILLEGAL_DATA_ADDR;
        }
        break;
    }
  }

  // If errcode is nonzero, respond with a NAK
  if (errcode != 0)
  {                                         // Char 0: Device Address (already loaded)
    ModB.TxMsgBuf[1] |= 0x80;               // Char 1: Function Code (already loaded) with msb set
    ModB.TxMsgBuf[2] = errcode;             // Char 2: Exception Code
    ndx = 3;
  }
  // Compute CRC of message and load into transmit buffer.  Note, the CRC is transmitted low byte, then high
  //   byte.
  offset = CalcCRC(&ModB.TxMsgBuf[0], ndx);         // offset used as temp to hold CRC
  ModB.TxMsgBuf[ndx++] = (uint8_t)(offset);
  ModB.TxMsgBuf[ndx++] = (uint8_t)(offset>>8);

  // Save number of chars to transmit
  ModB.CharsToTx = ndx;
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ProcFC0304Msg()
//------------------------------------------------------------------------------------------------------------



// TODO: Combine this as a special case with FC16Msg
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcFC06Msg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Function Code 6 Messages
//
//  MECHANICS:          This subroutine processes received messages with function code 6.  These are Write
//                      Single Register messages.  The subroutine parses and decodes the message, stores the
//                      value in the appropriate location, and then assembles the response.
//
//  CAVEATS:            This subroutine ignores "unforgiving" and NAKs any incorrect address/data. This
//                      seems safest for writing data but can be changed in the future if others disagree.
//
//  INPUTS:             length: length of the input message
//                      ModB.RxMsgBuf: input message
//                      MODB_OBJECT_ADDR_GRxx: table of variable addresses for a particular register group
//
//  OUTPUTS:            ModB.TxMsgBuf[]: output message
//                      ModB.CharsToTx: output message length
//                      *MODB_OBJECT_ADDR_GRxx: locations where values are stored
//                      MB_Harmonics_Selection
//
//  ALTERS:             None
//
//  CALLS:              Modb_CheckRegAddress(), CalcCRC()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ProcFC06Msg(uint8_t length)                    
{
  uint16_t start_reg_add, offset, val;
  uint8_t  errcode;
  uint8_t  i, copy;
  uint8_t group, num_bad, num_to_add;
  uint8_t numsetp;
  uint8_t SetpGrpNum;
  uint8_t CurSetpSet;
  uint16_t tmp16;

  // Modbus input message (ModB.RxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x06
  //   Data Field:
  //     [2]: Register Address high byte
  //     [3]: Register Address low byte
  //     [4]: Data to Be Written high byte
  //     [5]: Data to Be Written low byte
  //   [6]: CRC low byte
  //   [7]: CRC high byte

  // Modbus response message (ModB.TxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x06
  //   Data Field:
  //     [2]: Register Address high byte
  //     [3]: Register Address low byte
  //     [4]: Data to Be Written high byte
  //     [5]: Data to Be Written low byte
  //   [6]: CRC low byte
  //   [7]: CRC high byte
  
  // Initialize working variables
  start_reg_add = ( (((uint16_t)ModB.RxMsgBuf[2]) << 8) | ModB.RxMsgBuf[3] );   // Register address
  errcode = 0;                          // Initialize errcode to no error

  // Check message length
  if (length != 8)
  {
    errcode = NAK_ILLEGAL_DATA_VAL;
  }

  if (errcode == 0)                     // If message is ok so far...
  {
    // Call subroutine to translate the device address into a Modbus Data Object Table location
    // Function inputs:
    //   start_reg_add - starting register address
    //   num_reg - number of requested registers = 1
    // Function outputs:
    //   group - the table number that has the data addresses for the requested registers
    //   offset - the offset in the table of the data address for the first register
    //   num_bad - the number of invalid register addresses before the first valid address (not used)
    //   num_to_add - the number of data objects to add from the table (not used)
    Modb_CheckRegAddress(&group, &offset, &num_bad, &num_to_add, start_reg_add, 1);
    // get the value to be written, common for all groups
    val = ( (((uint16_t)ModB.RxMsgBuf[4]) << 8) | (ModB.RxMsgBuf[5]) );
    // Switch into the group that has the data addresses for the registers
    switch (group)
    {
      case 2:  // Modbus object handling configuration registers
      case 8:  // Modbus communications configuration registers
//      case 52:  // Modbus object handling configuration registers   *** DAH  SHOULD ALTERNATE ADDRESS BE SUPPORTED?
        // These Modbus configuration registers are implemented as Group 2 setpoints in the PXR35
        // (coincidence that Modbus address group 2 contains data for Setpoints Group 2). We may
        // want to put these under Password control in the future, but for now treat them as normal
        // register writes.
        // Start by getting the current Group 2 setpoints.
        // For Group 2 Setpoints, the Setpoints Set doesn't matter so just ask for the Active set.
        copy = 0; // keep the code analyzers happy
        Get_Setpoints(SetpActiveSet, 0x02, &copy);  // places Group 2 setpoints in SetpScratchBuf[]
        // Next move the value from the Modbus buffer to the appropriate setpoint locations
        // The four communications configuration registers (group 8) are at setpoint Group 2
        // locations 0-3 while the three object handling configuration registers (group 2)
        // are at locations 4-6.
        if (group == 2)
        {
          offset += 4;
        }
        SetpScratchBuf[offset] = val; // enter the new setpoint value
        // verify the change is valid
        if (Verify_Setpoints(2, (uint16_t *)(&SetpScratchBuf[0])) == 1)
        {
          Modb_Save_Setpoints(0xFF, 2); // store change if valid
        }
        break;
        
      case 7:  // Setpoint registers
        // calculate the offset from the start of this register group (group 7)
        tmp16 = start_reg_add - MODB_GROUP_START_ADD[7];
        
        if (tmp16 == 0) // Password register
        {
          // verify value written matches passord to enable setpoint changes
          if (val == DEBUG_PASSWORD)
          {
            // start setpoints write enable timer
          }
        }

        else if (tmp16 == 1) // Group/Set register, range check and update it
        {
          SetpGrpNum = (uint8_t)(val >> 8);
          CurSetpSet = (uint8_t)(val); // note this does NOT select the ACTIVE set
// TODO: verify 0xFF is OK?
          if (((CurSetpSet >= 4) && (CurSetpSet != 0xFF)) || (SetpGrpNum >= NUM_STP_GROUPS))
          {
            errcode = NAK_ILLEGAL_DATA_VAL; // not really correct usage of this error code!
          }
          else
          {
// TODO: The following will keep the 0xFF for Active Set request if we support this
            ModB_CurSetGrp = val;
          }
        }

        else // possibly an actual setpoint!
        {
          // get the Setpoints Set and Group that we're working with
          SetpGrpNum = (uint8_t)(ModB_CurSetGrp >> 8);                            
          CurSetpSet = (uint8_t)(ModB_CurSetGrp);
          // in an older version of this function the previous two variables were checked for
          // valid ranges.
          // They're coming from Modb_CurSetGrp, which is a global/static RAM variable that is
          // initialized on reset to a valid value and any updates are range checked, so it
          // seems unnecessary to range check here again.
          
// TODO:  Make sure this is what we want to do!!!
          // handle special case of Active set
          if (CurSetpSet == 0xFF)
          {
            CurSetpSet = (uint8_t)Setpoints0.stp.ActiveSet;
          }
          
          // get to actual setpoint offset
          tmp16 -= 2; 
          // get the number of setpoints in the current group.  SETP_GR_SIZE is in bytes, including
          // 16 bit checksum and checksum complement, so divide by two and subract 2 for number of 
          // actual 16-bit setpoints in the structure
          numsetp = ((SETP_GR_SIZE[SetpGrpNum] >> 1) - 2);
                  
          // check if requested offset is larger than number of setpoints in the setpoint group
          if (tmp16 >= numsetp)
          {                                   
            errcode = NAK_ILLEGAL_DATA_ADDR;
          }

          else  // valid setpoint number so far
          {
// TODO: need to add functionality to check if password has been properly written within appropriate time

            // get the current setpoint - - setpoints returned in SetpScratchBuf[]
            copy = 0; // keep the code analyzers happy
            Get_Setpoints(CurSetpSet, (uint8_t)SetpGrpNum, (uint8_t *)(&copy));
            SetpScratchBuf[tmp16] = val; // enter the new setpoint value
            
            if (Verify_Setpoints(SetpGrpNum, (uint16_t *)(&SetpScratchBuf[0])) == 1)
            {
              Modb_Save_Setpoints(CurSetpSet, SetpGrpNum);
            }
            // else??? technically shouldn't NAK for wrong setpoint data

// TODO: Modb_Save_Setpoints() return a success/fail indication?  If any of these steps validating/saving setpoints fail what should the NAK error code be?
          }
        }
        break;
        
/* Cases 14 and 27 were originally in FC06, but the Harmonics Selection Registers are 32-bit values
   so they cannot be written with a single register write.  The value is limited to <= 30, so we
   could accept a 16-bit value and extend it to 32-bit with leading zeros if desired.
   Similar for setting Event type/ID just in case we want to send that info via RTU in the future?
*/

// TODO: Verify there aren't any other valid address groups.  Groups 14 and 27 were originally here but they are 32 bit values that can't be sent with a single register write!

      default:                            // Invalid address
        // Address is invalid, so set errcode to NAK_ILLEGAL_ADDRESS (we will NAK)
        errcode = NAK_ILLEGAL_DATA_ADDR;
        break;
    }
  }

  // If errcode is nonzero, respond with a NAK
  if (errcode != 0)
  {
    ModB.TxMsgBuf[0] = Setpoints2.stp.Modbus_Port_Addr; // Char 0: Device Address (char index = 1)
    ModB.TxMsgBuf[1] = 0x86;                // Char 1: Function Code
    ModB.TxMsgBuf[2] = errcode;             // Char 2: Exception Code
    i = 3;
  }
  // Otherwise respond with duplicate of the input message
  else
  {
    for (i = 0; i < 6; ++i)
    {
      ModB.TxMsgBuf[i] = ModB.RxMsgBuf[i];
    }
  }
  // Compute CRC of message and load into transmit buffer.  Note, the CRC is transmitted low byte, then high
  //   byte.
  val = CalcCRC(&ModB.TxMsgBuf[0], i);
  ModB.TxMsgBuf[i++] = (uint8_t)(val);
  ModB.TxMsgBuf[i++] = (uint8_t)(val>>8);

  // Save number of chars to transmit
  ModB.CharsToTx = i;
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ProcFC06Msg()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcFC16Msg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Function Code 16 Messages
//
//  MECHANICS:          This subroutine processes received messages with function code 16.  These are Write
//                      Multiple Registers messages.  The subroutine parses and decodes the message, stores
//                      the values in the appropriate locations, and then assembles the response.
//
//  CAVEATS:            This subroutine ignores "unforgiving" and NAKs any incorrect address/data. This
//                      seems safest for writing data but can be changed in the future if others disagree.
//
//  INPUTS:             length: length of the input message
//                      ModB.RxMsgBuf: input message
//                      MODB_OBJECT_ADDR_GRxx: table of variable addresses for a particular register group
//
//  OUTPUTS:            ModB.TxMsgBuf[]: output message
//                      ModB.CharsToTx: output message length
//                      HarmFrozen: value may be changed depending on the execute action command
//
//  ALTERS:             None
//
//  CALLS:              Modb_CheckRegAddress(), CalcCRC()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ProcFC16Msg(uint8_t length)                    
{
  uint16_t start_reg_add, offset, val, sub_code;
  uint8_t copy;
  uint8_t  errcode, i;
  uint8_t ndx, ndx2;
  uint8_t group, num_bad, num_to_add;
  uint8_t numsetp;
  uint8_t SetpGrpNum;
  uint8_t CurSetpSet;
  uint16_t tmp16;

  // Modbus input message (ModB.RxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x10
  //   Data Field:
  //     [2]: Register Address ("X") high byte
  //     [3]: Register Address ("X") low byte
  //     [4]: Number of Registers to be Written ("num") high byte
  //     [5]: Number of Registers to be Written ("num") low byte
  //     [6]: Number of Bytes to be Written ("n")
  //     [7]: Data to Be Written into Register X high byte
  //     [8]: Data to Be Written into Register X low byte
  //         .....
  //     [7+n-2]: Data to Be Written into Register X+num high byte
  //     [7+n-1]: Data to Be Written into Register X+num low byte
  //   [7+n]: CRC low byte
  //   [7+n+1]: CRC high byte

  // Modbus response message (ModB.TxMsgBuf[]) format:
  //   [0]: Device Address
  //   [1]: Function Code = 0x16
  //   Data Field:
  //     [2]: Starting Register Address ("X") high byte
  //     [3]: Starting Register Address ("X") low byte
  //     [4]: Number of Registers Written ("num") high byte
  //     [5]: Number of Registers Written ("num") low byte
  //   [6]: CRC low byte
  //   [7]: CRC high byte
  
  // Initialize working variables
  start_reg_add = ( (((uint16_t)ModB.RxMsgBuf[2]) << 8) | ModB.RxMsgBuf[3] );   // Register address
  errcode = 0;                          // Initialize errcode to no error

  // Check message length and number of registers versus number of bytes to write
  //   Message length should be num bytes (ModB.RxMsgBuf[6]) + 7 (7 bytes before the data bytes) + 2 (CRC)
  if ( (length != (ModB.RxMsgBuf[6]+9))
    || (ModB.RxMsgBuf[4] != 0)                          // High byte of number of regs to write should be 0
    || (ModB.RxMsgBuf[5] != (ModB.RxMsgBuf[6]/2)) )     // Low byte of number of regs to write should be
  {                                                     //   (num bytes to write)/2  (reg is 2 bytes long)
    errcode = NAK_ILLEGAL_DATA_VAL;
  }

  if (errcode == 0)                     // If message is ok so far...
  {
    // Call subroutine to translate the device address into a Modbus Data Object Table location
    // Function inputs:
    //   start_reg_add - starting register address
    //   num_reg - number of requested registers = 1
    // Function outputs:
    //   group - the table number that has the data addresses for the requested registers
    //   offset - the offset in the table of the data address for the first register
    //   num_bad - the number of invalid register addresses before the first valid address
    //   num_to_add - the number of data objects to add from the table
    Modb_CheckRegAddress(&group, &offset, &num_bad, &num_to_add, start_reg_add, ModB.RxMsgBuf[5]);

    // At the initial writing of this code there are no contiguous register address groups for Function
    // Code 16 and we aren't "forgiving", so NAK if any registers outside the current group.
    if ((num_bad > 0) || ((start_reg_add + num_to_add - 1) > MODB_GROUP_END_ADD[group]))
    {
      errcode = NAK_ILLEGAL_DATA_ADDR;
    }
    else // so far so good...
    {

      // Switch into the group that has the data addresses for the registers
      switch (group)
      {
        case 2:  // Modbus object handling configuration registers
        case 8:  // Modbus communications configuration registers
//        case 52:  // Modbus object handling configuration registers   *** DAH  SHOULD ALTERNATE ADDRESS BE SUPPORTED?
          // These Modbus configuration registers are implemented as Group 2 setpoints in the PXR35
          // (coincidence that Modbus address group 2 contains data for Setpoints Group 2). We may
          // want to put these under Password control in the future, but for now treat them as normal
          // register writes.
          // Start by getting the current Group 2 setpoints.
          // For Group 2 Setpoints, the Setpoints Set doesn't matter so just ask for the Active set.
          copy = 0; // keep the code analyzers happy
          Get_Setpoints(SetpActiveSet, 0x02, &copy);  // places Group 2 setpoints in SetpScratchBuf[]
          // next move the value(s) from the Modbus buffer to the appropriate memory locations
          ndx = 7; // index to the first data value in the Modbus buffer
          // The four communications configuration registers (group 8) are at setpoint Group 2
          // locations 0-3 while the three object handling configuration registers (group 2)
          // are at locations 4-6.
          if (group == 2)
          {
            offset += 4;
          }
          while (num_to_add > 0)
          {
            // get the value from the received message buffer
            val = (uint16_t)ModB.RxMsgBuf[ndx++] << 8;
            val |= ModB.RxMsgBuf[ndx++];
            SetpScratchBuf[offset] = val; // enter the new setpoint value
            num_to_add--;
            if (num_to_add > 0)
            {
              offset++; // next place to store
            }
          }  
          // verify the change is valid
          if (Verify_Setpoints(2, (uint16_t *)(&SetpScratchBuf[0])) == 1)
          {
            // store the Group 2 setpoints
            Modb_Save_Setpoints(0xFF, 2);
          }
          break;

        case 3:  // Remote control registers
          // Execute action is a three-byte hex number, byte2...byte0, followed by its one's complement
          // The message format is shown below.  Remember that data is sent in 2-byte pairs - the high byte
          //   followed by the low byte:
            // Modbus input message (ModB.RxMsgBuf[]) format:
            //   [0]: Device Address
            //   [1]: Function Code = 0x10
            //   Data Field:
            //     [2]: Register Address high byte = x0B or x62
            //     [3]: Register Address low byte = x54 or x00
            //     [4]: Number of Registers to be Written high byte = x00
            //     [5]: Number of Registers to be Written low byte = x03
            //     [6]: Number of Bytes to be Written  = x06
            //     [7]: Execute action byte1
            //     [8]: Execute action byte0
            //     [9]: Execute action complement byte0
            //     [10]: Execute action byte2
            //     [11]: Execute action complement byte2
            //     [12]: Execute action complement byte1
            //   [13]: CRC low byte
            //   [14]: CRC high byte
  
            // Modbus response message (ModB.TxMsgBuf[]) format:
            //   [0]: Device Address
            //   [1]: Function Code = 0x16
            //   Data Field:
            //     [2]: Starting Register Address high byte = x0B or x62
            //     [3]: Starting Register Address low byte = x54 or x00
            //     [4]: Number of Registers Written high byte = x00
            //     [5]: Number of Registers Written low byte = x03
            //   [6]: CRC low byte
            //   [7]: CRC high byte
  
          if (offset != 0) // Starting address must be the first address in the group
          {
            errcode = NAK_ILLEGAL_DATA_ADDR;
          }
// TODO: If we aren't going to NAK bad data, change this to looking for valid values rather than invalid
          else if ( ((ModB.RxMsgBuf[8] + ModB.RxMsgBuf[9]) != 0xFF)       // Execute action format must have
                 || ((ModB.RxMsgBuf[7] + ModB.RxMsgBuf[12]) != 0xFF)      //   correct one's complement
                 || ((ModB.RxMsgBuf[10] + ModB.RxMsgBuf[11]) != 0xFF)
                 || (ModB.RxMsgBuf[6] != 6) )                             // Must write 6 bytes (3 registers)
          {
            // errcode = NAK_ILLEGAL_DATA_VAL; // not really the correct use of this error code
          }
          else  // Message format is ok - decode
          {                                                               //   the command
            // Assemble the execute action group and subcode
            group = ModB.RxMsgBuf[10];
            sub_code = ( ((uint16_t)ModB.RxMsgBuf[8]) + (((uint16_t)ModB.RxMsgBuf[7]) << 8) );
            errcode = Modb_Remote_Control(group, sub_code);
          }
          break;
  
        case 7:  // Setpoints Password, Set/Group, and setting registers
// TODO: Is a user allowed to send password, set/group select, and setpoint changes all in one message?  For now assume they can!
          // Work through the possible registers, Check Password, then Set/Group,
          // then loop through settings as needed
// TODO: verify that we really need tmp16, or just use start_reg_add
          tmp16 = start_reg_add; // get the register address
          ndx = 7; // index to the first data value in the Modbus buffer
          if(tmp16 == 0x0BB7)  // Password register, verify and enable setpoint changes
          {
            // get the value from the received message buffer
            val = (uint16_t)ModB.RxMsgBuf[ndx++] << 8;
            val |= ModB.RxMsgBuf[ndx++];
            if (val == DEBUG_PASSWORD)
            {
              // start setpoints write enable timer
            }
            else
            {
              // should we NAK a bad password?  I don't think so, but maybe zero the timer?
            }
            tmp16++; // next register
            ndx += 2; // index to next data in RX buffer
            num_to_add--;
          }
          if((num_to_add > 0) && (tmp16 == 0x0BB8))
          {
            // Setpoint group/set register, range check and update.
            // get the value from the received message buffer
            val = (uint16_t)ModB.RxMsgBuf[ndx++] << 8;
            val |= ModB.RxMsgBuf[ndx++];
            SetpGrpNum = (uint8_t)(val >> 8);
            CurSetpSet = (uint8_t)(val);
            if (((CurSetpSet >= 4) && (CurSetpSet != 0xFF)) || (SetpGrpNum >= NUM_STP_GROUPS))
            {
              errcode = NAK_ILLEGAL_DATA_VAL; // not really the correct use of this error code
            }
            else
            {
              ModB_CurSetGrp = val;
            }
            tmp16++; // next register
            ndx += 2; // index to next data in RX buffer
            num_to_add--;
          }
          if (num_to_add > 0)
          {
            // Either we've just processed Password/Set/Group register or are starting in Setpoints values.
            // Get the Setpoints Set and Group that we're working with (can't count on the previous register
            // being written as part of this command)
            SetpGrpNum = (uint8_t)(ModB_CurSetGrp >> 8); //Setpoint group is stored in the high 8 bits
            CurSetpSet = (uint8_t)(ModB_CurSetGrp); //Setpoint set is stored in the low 8 bits
            // in an older version of this function the previous two variables were checked for
            // valid ranges.
            // They're coming from Modb_CurSetGrp, which is a global/static RAM variable that is
            // initialized on reset to a valid value and any updates are range checked, so it
            // seems unnecessary to range check here again.
            
            if (CurSetpSet == 0xFF) // 0xFF indicates Active Set
            {
// TODO: make sure this is safe, that CurSetpSet always gets set from ModB_CurSetGrp so that we don't use a "stale" Active Set.
              CurSetpSet = (uint8_t)Setpoints0.stp.ActiveSet;
            }

            // get the number of setpoints in the current group.  SETP_GR_SIZE is in bytes, including
            // 16 bit checksum and checksum complement, so divide by two and subract 2 for number of 
            // actual 16-bit setpoints in the structure
            numsetp = ((SETP_GR_SIZE[SetpGrpNum] >> 1) - 2);
            // Check if requested offset is larger than number of setpoints in the setpoint group.
            // tmp16 was previously set to the first setpoint to change, 3001 is first setpoint address,
// TODO: verify the corner cases, maybe need a "+1"???
            if ((tmp16+ num_to_add) > 3001 + numsetp)
            {                                   
              errcode = NAK_ILLEGAL_DATA_ADDR;
            }
            else  // valid setpoint number so far
            {
              // get the current setpoints for the selected set/group ( in SetpScratchBuf[])
              copy = 0; // keep the code analyzers happy
              Get_Setpoints(CurSetpSet, (uint8_t)SetpGrpNum, (uint8_t *)(&copy));
              ndx2 = tmp16 - 3001;  // index into the setpoints scratch buffer
              while (num_to_add > 0)  // add in the setpoint values
              {
                // get the value to save from the received message buffer
                val = (uint16_t)ModB.RxMsgBuf[ndx++] << 8;
                val |= ModB.RxMsgBuf[ndx++];
                // and store it in the setpoints scratch buffer
                SetpScratchBuf[ndx2] = val;
                ndx2++;
                num_to_add--;
              }
              // all requested changes are in the scratch buffer, now verify they are within allowed ranges
              if (Verify_Setpoints(SetpGrpNum, (uint16_t *)(&SetpScratchBuf[0])) == 1)
              {
                Modb_Save_Setpoints(CurSetpSet, SetpGrpNum);
              }
              // else??? technically shouldn't NAK for wrong setpoint data
            }
          }
          break;
        
/* Cases 14 and 27 were originally in FC06, but the Harmonics Selection Registers are 32-bit values
   so they cannot be written with a single register write.  Moved the cases here, but need to verify
   the actual code. Also maybe add cases for setting Event type/ID just in case we want to send that
   info via RTU?  Similarly add cases to ProcFC0304Msg() for Event data?
*/
          
  
        case 14:                            // Floating point Harmonics Selection Register
        case 27:                            // Fixed point Harmonics Selection Register
          // If offset isn't 0, it isn't writing to the selection register and so it is a bad address
          if (offset != 0)
          {
            errcode = NAK_ILLEGAL_DATA_ADDR;
            break;
          }
          // Address is ok, now check the value.  Valid values are:
          //    0 - IaAgg         1 - IbAgg          2 - IcAgg           4 - InAgg          5 - VabAgg
          //    6 - VbcAgg        7 - VcaAgg         8 - VanAgg          9 - VbnAgg        10 - VcnAgg
          //   20 - IaCap        21 - IbCap         22 - IcCap          24 - InCap         25 - VabCap
          //   26 - VbcCap       27 - VcaCap        28 - VanCap         29 - VbnCap        30 - VcnCap
          val = ( (((uint16_t)ModB.RxMsgBuf[4]) << 8) | (ModB.RxMsgBuf[5]) );
          if ( (val == 3) || ((val > 10) && (val < 20)) || (val == 23) || (val > 30) )
          {
            errcode = NAK_ILLEGAL_DATA_VAL; // not really the correct use of this error code
          }
          else
          {
// TODO: maybe create separate fixed/floating point MB_Harmonics_Selection since they are technically different registers
            // Value is ok so just store it
            MB_Harmonics_Selection = val;
          }
          break;
  
  //      case 51:  Should we support alternate address?  Not yet!!!
        
        default:                            // Invalid address
          // Address is invalid, so set errcode to NAK_ILLEGAL_ADDRESS (we will NAK)
          errcode = NAK_ILLEGAL_DATA_ADDR;
          break;
      }
    }
  }
  // If errcode is nonzero, respond with a NAK
  if (errcode != 0)
  {
    ModB.TxMsgBuf[0] = Setpoints2.stp.Modbus_Port_Addr; // Char 0: Device Address (char index = 1)
    ModB.TxMsgBuf[1] = 0x90;                // Char 1: Function Code
    ModB.TxMsgBuf[2] = errcode;             // Char 2: Exception Code
    i = 3;
  }
  // Otherwise respond with duplicate of the input message
  else
  {
    for (i = 0; i < 6; ++i)
    {
      ModB.TxMsgBuf[i] = ModB.RxMsgBuf[i];
    }
  }
  // Compute CRC of message and load into transmit buffer.  Note, the CRC is transmitted low byte, then high
  //   byte.
  val = CalcCRC(&ModB.TxMsgBuf[0], i);
  ModB.TxMsgBuf[i++] = (uint8_t)(val);
  ModB.TxMsgBuf[i++] = (uint8_t)(val>>8);

  // Save number of chars to transmit
  ModB.CharsToTx = i;
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ProcFC16Msg()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        CalcCRC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate CRC-16 of Specified Message Subroutine
//
//  MECHANICS:          This subroutine calculates the CRC-16 of a message stored in either a transmit or
//                      receive buffer, pointed to by *msg_ptr.  
//                      The CRC-16 generator polynomial is x^16 + x^15 + x^2 + 1, and is computed using
//                      a 256-word look-up-table method.
//
//  CAVEATS:            The subroutine assumes that *(msg_ptr + len) is within the buffer size
//
//  INPUTS:             Parameters: msg_ptr - starting address of the message buffer
//                                  len - number of (byte) characters in the buffer
// 
//  OUTPUTS:            CRC-16
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

uint16_t CalcCRC(uint8_t *msg_ptr, uint16_t len)
{
  uint16_t crc, nxt_char, i;           // temps

  crc = 0xFFFF;                        // Initialize crc
  if (len > 0)
  {
    for (i=0; i<len; i++)
    {
      nxt_char = *msg_ptr++;                    // Load next char from the buffer and increment the pointer
      nxt_char = (crc ^ nxt_char) & 0x00FF;
      crc = ((crc ^ CRC_TABLE1[nxt_char]) >> 8) + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
    }
  }
  return(crc);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CalcCRC()
//------------------------------------------------------------------------------------------------------------
            
            
            
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Modb_Save_Setpoints()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Common function to save setpoints for Modbus Function Codes 6 and 16
//
//  MECHANICS:          This subroutine saves setpoints to onboard FRAM or Frame FRAM depending
//                      on setpoint group.  It also loads the RAM copy when appropriate.
// TODO: maybe improve this desription?
//
//  CAVEATS:            The subroutine assumes the setpoints change password has been received/verified
//                      and the new setpoints have been verified and are at &SetpScratchBuf
//
//  INPUTS:             Parameters: CurSetpSet - 
//                                  SetpGrpNum - 
//                                  SetpScratchBuf[] - 
//                                  len - 
//                                  len - 
// 
//  OUTPUTS:            ???
//
//  ALTERS:             SetpScratchBuf[]
//
//  CALLS:              Checksum8_16(), FRAM_Write(), Frame_FRAM_Write(), Gen_Values(), Get_Setpoints()
//
//------------------------------------------------------------------------------------------------------------

void Modb_Save_Setpoints(uint8_t CurSetpSet, uint8_t SetpGrpNum)
{
  uint32_t newEID;
  uint16_t sum;
  uint16_t faddr;
  uint16_t *sptr;
  uint16_t *sptr2;
  uint16_t numsetp;
  uint16_t j;
  uint8_t copy;
  // calculate checksum and complement
  sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_SIZE[SetpGrpNum] - 4));
  SetpScratchBuf[(SETP_GR_SIZE[SetpGrpNum] >> 1) - 2] = sum;
  sum = (sum ^ 0xFFFF);
  SetpScratchBuf[(SETP_GR_SIZE[SetpGrpNum] >> 1) - 1] = sum;
  if ( (SetpGrpNum == 2) || (SetpGrpNum == 3) || (SetpGrpNum == 11) )  // Gr 2, 3, and 11 are stored
  {                                                                    //   in on-board FRAM
    FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(SetpGrpNum << 1)], 
              (SETP_GR_SIZE[SetpGrpNum] >> 1), (uint16_t *)(&SetpScratchBuf[0]));
    FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(SetpGrpNum << 1) + 1], 
              (SETP_GR_SIZE[SetpGrpNum] >> 1), (uint16_t *)(&SetpScratchBuf[0]));
  }
  else                                      // All other groups are in the Frame FRAM
  {
    // Groups 0, 1, and 4-9 have one of four possible sets, groups 10 and above only have 1
    
    faddr = ( (SetpGrpNum < 10) ?
       (SETP_GR_FRAM_ADDR[(SetpGrpNum << 1)] + (SETP_SET_ADDRESS_OFFSET * CurSetpSet))
     : (SETP_GR_FRAM_ADDR[(SetpGrpNum << 1)]) );
    Frame_FRAM_Write(faddr, SETP_GR_SIZE[SetpGrpNum], (uint8_t *)&SetpScratchBuf[0]);
    faddr = ( (SetpGrpNum < 10) ?
       (SETP_GR_FRAM_ADDR[(SetpGrpNum << 1) + 1] + (SETP_SET_ADDRESS_OFFSET * CurSetpSet))
     : (SETP_GR_FRAM_ADDR[(SetpGrpNum << 1) + 1]) );
    Frame_FRAM_Write(faddr, SETP_GR_SIZE[SetpGrpNum], (uint8_t *)&SetpScratchBuf[0]);
  }

  // If this is Group 2 (Modbus), check for changes that require resetting the UART
  if ((SetpGrpNum == 2) && 
        ((SetpScratchBuf[1] != Setpoints2.stp.Modbus_Port_Baudrate) ||
         (SetpScratchBuf[2] != Setpoints2.stp.Modbus_Port_Parity) ||
         (SetpScratchBuf[3] != Setpoints2.stp.Modbus_Port_Stopbit)))
  {
     ModB.Reset_Req = TRUE;
  }
  
  // If this is the active setpoints set or Group 2, 3 or 11, also write to the active
  //   setpoints, and update the setpoints status
  if ((CurSetpSet == SetpActiveSet) || (SetpGrpNum == 2) || (SetpGrpNum == 3) || (SetpGrpNum == 11) )
  {
// TODO: verify that we need to retrieve the setpoints again.  Dan thinks this is required but possibly just a throwback to older design?  Also should we check return value or "i" for errors?
    copy = 0; // keep the code analyzers happy
    Get_Setpoints(CurSetpSet, (uint8_t)SetpGrpNum, (uint8_t *)(&copy));
    sptr = SETP_GR_DATA_ADDR[SetpGrpNum];                  // Set pointer to the first setpoint
    sptr2 = (uint16_t *)&SetpScratchBuf[0];
    // SETP_GR_SIZE[ ] is the structure size.  Divide by 2 because each setpoint is a
    //   word.  Subtract 2 because the structure includes the checksum and checksum
    //   complement
    numsetp = ((SETP_GR_SIZE[SetpGrpNum] >> 1) - 2);
    // Disable protection while setpoints are being changed.  This only impacts the
    //   protection that is being done in the sampling interrupt
    // This takes about 92usec max (including interrupts)
    Prot_Enabled = FALSE;
    for (j = 0; j < numsetp; j++)
    {
      sptr[j] = sptr2[j];
    }
    Gen_Values();                     // Generate new protection values
    Prot_Enabled = TRUE;
    newEID = InsertNewEvent(STP_DWNLD_MODBUS_RTU);
    // If the demand setpoints changed, we need to restart logging with the new variables
    if ( (Dmnd_Setp_DemandWindow != (uint8_t)Setpoints0.stp.DemandWindow)
      || (Dmnd_Setp_DemandInterval != (uint8_t)Setpoints0.stp.DemandInterval)
      || (Dmnd_Setp_DemandLogInterval = (uint8_t)Setpoints0.stp.DemandLogInterval) )
    {
      Dmnd_VarInit();
      EngyDmnd[1].EID = newEID;
    }
    // Update setpoints status byte to "ok" for the group that was written
    if ( (SetpGrpNum == 2) || (SetpGrpNum == 3) || (SetpGrpNum == 11) )
    {
      // If group 2, 3, or 11, set status to 3: retrieved successfully from on-board FRAM
      SetpointsStat |= (3 << (SetpGrpNum << 1));
    }
    else                                                   
    {
      // Any other group, set status to 0: retrieved successfully from Frame FRAM as a PXR35
      SetpointsStat &= ~((3 << (SetpGrpNum << 1)));             //  
    }
    if (SetpGrpNum == 12) RelayFlagStp = TRUE;
    Admin_Verified_Tmr = (Admin_Verified_Tmr) ? (COUNT_10_MIN) : (Admin_Verified_Tmr);
    User_Verified_Tmr = (User_Verified_Tmr) ? (COUNT_10_MIN) : (User_Verified_Tmr);
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Modb_Save_Setpoints()
//------------------------------------------------------------------------------------------------------------
            
            
            
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Modb_Remote_Control()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Handles remote control commands as received through Function Code 16, just
//                      broken out as a subroutine since it is so long.
//
//  MECHANICS:          This subroutine parses the three-byte remote control/execute action commands
//                      and performs the appropriate control/action.
//
//  CAVEATS:            The subroutine assumes the three bytes have been validated by checking the
//                      associated 1s complement.
//
//  INPUTS:             Parameters: control_group - basically Slave action Byte 2
//                                  sub_code - basically Slave action Bytes 1 (msb) and 0 (lsb)
//                                  SetpScratchBuf[] - 
// 
//  OUTPUTS:            errocde (not sure if we'll use this???)
//
//  ALTERS:             ??? depends on control/action
//
//  CALLS:              ??? depends on control/action
//
//------------------------------------------------------------------------------------------------------------

uint8_t Modb_Remote_Control(uint8_t control_group, uint16_t sub_code)
{
  uint8_t errcode;
  uint16_t mm_setting_updated;

  errcode = 0;  // initialize to no error

  switch (control_group)  // execute action Byte 2, Control Group
  {
    case 0:  // resets
      switch (sub_code)
      {
        case 0x0001:  // Reset alarm
          break;
        case 0x0002:  // Reset trip
          break;
        case 0x0003:  // Reset powered-up indication
          ResetMinMaxFlags |=  RESET_PWRUP_FLGS_FLAG;
          break;
        case 0x0004:  // Reset demand power
          ResetMinMaxFlags |=  RESET_PEAK_DMNPWR_FLAG;
          break;
        case 0x0008:  // Reset energy (kWhr)
          ResetMinMaxFlags |=  RESET_ACC_ENG_FLAG;
          break;
        case 0x0010:  // Reset device software
          break;
        case 0x0020:  // Reset time-stamped event data buffers
          break;
        case 0x0040:  // Reset (synchronize) demand watts window
          ResetMinMaxFlags |= RESET_ALLDMNDW_FLAG;
          break;
        case 0x0080:  // Reset snapshot command
          break;
        case 0x0101:  // Reset demand - currents
          ResetMinMaxFlags |=  RESET_PEAK_DMNCURR_FLAG;
          break;
        case 0x0102:  // Reset operations count (or trigger counters)
          ResetMinMaxFlags |=  RESET_OOPTS_FLAG;
          break;
        case 0x0103:  // Reset run time
          ResetMinMaxFlags |=  RESET_RUNTIME_FLAG;
          break;
        case 0x0104:  // Reset all min/max values
          break;
        case 0x0105:  // Unlock waveform buffer (clear upload-in-progress) 
          break;
        case 0x0106:  // Reset discrete input counters
          break;
        case 0x010D:  // Reset min/max currents
          ResetMinMaxFlags |=  RESET_MINMAX_CURR_FLAG;
          break;
        case 0x010E:  // Reset min/max L-L voltages
          ResetMinMaxFlags |=  RESET_MINMAX_LL_VOLT_FLAG;
          break;
        case 0x010F:  // Reset min/max L-N voltages
          ResetMinMaxFlags |=  RESET_MINMAX_LN_VOLT_FLAG;
          break;
        case 0x0110:  // Reset min/max PF - apparent
          ResetMinMaxFlags |=  RESET_MINMAX_PF_FLAG;
          break;
        case 0x0111:  // Reset min/max PF - displacement
          break;
        case 0x0112:  // Reset min/max power
          ResetMinMaxFlags |=  RESET_MINMAX_PWR_FLAG;
          break;
        case 0x0113:  // Reset min/max current THD
          break;
        case 0x0114:  // Reset min/max voltage THD
          break;
        case 0x0116:  // Reset min/max per-phase power
          break;
        case 0x0117:  // Reset min/max frequency
          ResetMinMaxFlags |=  RESET_MINMAX_FREQ_FLAG;
          break;
        case 0x0118:  // Reset min/max current unbalances
          break;
        case 0x0119:  // Reset min/max L-L voltage unbalances
          break;
        case 0x011A:  // Reset min/max L-N voltage unbalances
          break;
        case 0x011B:  // Reset min/max current THD
          break;
        case 0x011C:  // Reset min/max L-L voltage THD
          break;
        case 0x011D:  // Reset min/max L-N voltage THD
          break;
        case 0x011E:  // Reset max PKE Overload Warning
          break;
        case 0x011F:  // Reset max Short Delay Overload Warning
          break;
        case 0x0124:  // Reset op count, runtime, and override count
          break;
        case 0x0125:  // Reset motor data maximum values
          break;
        case 0x0126:  // Reset motor trip and alarm counters
          break;
  // TODO: determine how many trigger numbers and decide best way to handle this
  //                  case 0x02X   // Reset locked-trigger #X (X = trigger number)
  //                    break;
        case 0x0301:  // Reset Source 1 available time
          break;
        case 0x0302:  // Reset Source 1 connect time
          break;
        case 0x0303:  // Reset Source 1 run time
          break;
        case 0x0304:  // Reset Source 2 available time
          break;
        case 0x0305:  // Reset Source 2 connect time
          break;
        case 0x0306:  // Reset Source 2 run time
          break;
        case 0x0307:  // Reset load energized time
          break;
        case 0x0308:  // Reset transfer status
          break;
  // TODO: determine how many sensors and decide best way to handle this
  //                  case 0x04X:  // Reset tamper flag for sensor #X
  //                    break;
        case 0x0501:  // Reset trip count
          ResetMinMaxFlags |=  RESET_TRIP_CNTR_FLAG;
          break;
        case 0x0502:  // Reset temperature
          ResetMinMaxFlags |=  RESET_TEMP_FLAG;
          break;
        case 0x0503:  // Reset all diagnostics information
          break;
        default:  // Invalid execute action command, ignore or NAK
         // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
          break;
      }
      break;
    case 1:  // Circuit Breaker Open/Close
      switch (sub_code)
      {
        case 0x0000:  // Open request
          break;
        case 0x0001:  // Close request
          break;
        case 0x0002:  // Trip request
          break;
        case 0x0008:  // Enable maintenance mode
          // b08: MM state
          // b07: MM local ON
          // b06: MM local setting invalid
          // b01: MM signal from secondary pin
          // b00: MM remote com channel
          if ((Setpoints0.stp.MM_Enable & BIT0) == 0)
          {
            // if MM remote com channel status is off, turn it on and ensure MM state is ON
            Setpoints0.stp.MM_Enable |= (BIT0 | BIT8);
            mm_setting_updated = TRUE;
          }
          break;
        case 0x0009:  // Disable maintenance mode
          // if MM remote com channel status is on, turn it off
          if ((Setpoints0.stp.MM_Enable & BIT0) != 0)
          {
            // if MM local is also off, ensure all MM bits are cleared
            if ((Setpoints0.stp.MM_Enable & BIT7) == 0)
            {
              Setpoints0.stp.MM_Enable = 0;
            }
            else // otherwise just clear MM remote com channel
            {
              Setpoints0.stp.MM_Enable &= ~BIT0;
            }
            mm_setting_updated = TRUE;
          }
          break;
        default:  // Invalid execute action command, ignore or NAK
         // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
          break;
        }
      break;
    case 2:  // Motor Start/Stop (not used)
      // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
      break;
    case 3:  // System Control
      switch (sub_code)
      {
        case 0x0000:  // Release time-stamped event buffer
          break;
        case 0x0001:  // Capture waveform
          break;
        case 0x0002:  // Reset INCOM slave-interface statistics
          break;
        case 0x0003:  // Reset product-specific statistics
          break;
        case 0x0004:  // Acknowledge triggered event(s)
          break;
        case 0x0005:  // Reset sun-network master INCOM statistics
          break;
        case 0x0006:  // Acknowledge energy reset
          break;
        case 0x0007:  // Save setpoints change
          break;
        case 0x0008:  // Release time-stamped minor event buffer
          break;
        case 0x0009:  // Release time-stamped motor start profile buffer
          break;
        case 0x000A:  // Abort setpoints change
          break;
  // TODO:  Decide if we keep the following two commands added by Dan for customers to stop/starts harmonics calculations
        case 0x000B:  // Capture harmonics
          HarmFrozen = FALSE;
          break;
        case 0x000C:  // Freeze captured harmonics
          HarmFrozen = TRUE;
          break;
        default:  // Invalid execute action command, ignore or NAK
         // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
          break;
      }
      break;
    case 4:  // relay control (not used?)
      switch (sub_code)
      {
  // TODO: determine how many relays, if any, and decide best way to handle this
  //                  case 0x01X:  // Activate relay output #X (X = relay number 0-255)
  //                    break;
  //                  case 0x02X:  // De-activate relay output #X (X = relay number 0-255)
  //                    break;
        default:  // Invalid execute action command, ignore or NAK
          // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
          break;
        }
      break;
    case 5:  // Automatic Transfer Switch Control
       switch (sub_code)
       {
         case 0x0001:  // Open request
           break;
         case 0x0002:  // Close request
           break;
         case 0x0003:  // Trip request
           break;
         case 0x0004:  // Enable maintenance mode
          break;
         case 0x0005:  // Disable maintenance mode
           break;
         case 0x0006:  // Enable maintenance mode
          break;
         case 0x0007:  // Disable maintenance mode
           break;
         default:  // Invalid execute action command, ignore or NAK
          // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
           break;
         }
      break;
    default:  // Invalid execute action command, ignore or NAK
      // errcode = NAK_ILLEGAL_DATA_ADDR; // not really the correct use of this error code
      break;
  }
  // save group 0 setpoints if Maintenance Mode has changed
  if (mm_setting_updated == TRUE)
  {
    uint8_t copy;
    mm_setting_updated = FALSE;
  
    // get the current setpoints for group 0 - setpoints returned in SetpScratchBuf[]
    Get_Setpoints(SetpActiveSet, 0, (uint8_t *)(&copy));
  
    SetpScratchBuf[4] = Setpoints0.stp.MM_Enable;
    // typically we validate setpoints before saving them, but since the only thing we've changed
    // is MM_enable we don't need to take that extra step, simply save Group 0 of the active set to FRAM.
    Modb_Save_Setpoints(SetpActiveSet, 0);
  }
  return (errcode);
}
            
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Modb_Remote_Control()
//------------------------------------------------------------------------------------------------------------


