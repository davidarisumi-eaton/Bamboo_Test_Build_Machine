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
//  MODULE NAME:        Diag_def.h
//
//  MECHANICS:          This is the definitions file for the Diag.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//    64    230823  DAH File Creation
//   133    231219  DAH - Original HEALTH_DAT structure was used to compute the number of bytes in the
//                        health data section in Frame FRAM.  This is an error, because there are two
//                        padding bytes added in the structure definition (the computed length was 48 - it
//                        should be 46, including the checksum and checksum complement bytes).
//                          - Renamed HEALTH_DAT_SIZE to FRAME_FRAM_HEALTH_DAT_SIZE and hard-coded it to 42.
//                            It does not include the checksum and spacing bytes between Frame FRAM copies
//                          - Revised struct HEALTH_DAT to only include the bytes that are in use
//                          - Deleted union HEALTH_DEF
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
// Definitions
// Diagnostics data size.  Note, this is in bytes and includes the four checksum bytes.  It does not include
//   the extra four padding bytes (after the checksum bytes)
#define DIAG_DATA_SIZE                  56



//
//------------------------------------------------------------------------------------------------------------
//    Constants
//------------------------------------------------------------------------------------------------------------

//Frame FRAM address
#define HEALTHDATA_FADDR  0x706
#define INTERNAL_HEALTHDATA_FADDR 0x738
#define HEALTHCONFIG_FADDR  0x6BA



//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct HEALTH_CON
{
    uint16_t    Mech_Wear_Ref_1;
    uint16_t    Rate_1;
    uint16_t    Current_Slope_1;
    uint16_t    Current_Mech_Eq_Contact_1;
    uint16_t    Volt_slope_1;
    uint16_t    System_Voltage_1;
    uint16_t    Mech_Wear_Ref_2;
    uint16_t    Rate_2;
    uint16_t    Current_Slope_2;
    uint16_t    Current_Mech_Eq_Contact_2;
    uint16_t    Volt_slope_2;
    uint16_t    System_Voltage_2;
    uint16_t    Mech_Wear_Ref_3;
    uint16_t    Rate_3;
    uint16_t    Current_Slope_3;
    uint16_t    Current_Mech_Eq_Contact_3;
    uint16_t    Volt_slope_3;
    uint16_t    System_Voltage_3;
    uint16_t    MCR_Multiplier;
    uint16_t    Time_Temp_Adder_0;
    uint16_t    Time_Temp_Adder_1;
    uint16_t    Time_Temp_Adder_2;
    uint16_t    Time_Temp_Adder_3;
    uint16_t    Time_Temp_Adder_4;
    uint16_t    Display_Points_Per_Bar_0;
    uint16_t    Display_Points_Per_Bar_1;
    uint16_t    Display_Points_Per_Bar_2;
    uint16_t    Display_Points_Per_Bar_3;
    uint16_t    Display_Points_Per_Bar_4;
    
    uint16_t    Reserved_1;
    uint16_t    Reserved_2;
    uint16_t    Reserved_3;
    uint16_t    Reserved_4;
    uint16_t    Reserved_5;

    uint16_t    Cksum;
    uint16_t    Cksum_not;    
};
#define HEALTH_CON_BLOCK_SIZE sizeof(struct HEALTH_CON)
union HEALTH_CONFIG
{
    uint8_t             buf[HEALTH_CON_BLOCK_SIZE];
    struct HEALTH_CON   data;

};

struct HEALTH_DAT
{
    uint16_t    Operations_Cntr;
    uint32_t    Min_since_Contact_Wear_Rst; //Minutes since Last Contact Wear Reset
    uint32_t    Min_since_Mech_Wear_Rst;    //Minutes since Last Mech Wear Reset
    uint32_t    Min_since_Time_Temp_Rst;    //Minutes since Last Time/Temp Reset

    uint16_t    Life_Points_Contact_Wear;   
    uint16_t    Life_Points_Mech_Wear;
    uint16_t    Life_points_Time_Temp;

    uint16_t    Life_Points_Rst_Num;        //Number of Life Points Resets
};

// This is the number of bytes reserved for health data in Frame FRAM.  It does NOT include the four
//   checksum and checksum complement bytes, nor does it include the 4 bytes between copies
#define FRAME_FRAM_HEALTH_DAT_SIZE 42


