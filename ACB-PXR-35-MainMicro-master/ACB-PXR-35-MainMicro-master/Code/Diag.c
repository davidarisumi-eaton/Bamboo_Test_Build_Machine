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
//  MODULE NAME:        Diag.c
//
//  MECHANICS:          Program module containing the health and diagnostics subroutines
//                      Note: This code is based on the Diag.c module in the V3.0 PXR25 code
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//    64    230823  DAH File Creation
//    108   231108  DAH - Minor revisions to Get_BrkHealthConfig() and Save_BrkHealthConfig() to eliminate
//                        compiler warnings
//    133   231219  DAH - Replaced Health_Data and Internal_Health_Data definitions from union HEALTH_DEF to
//                        struct HEALTH_DEF.  Unions cannot be used to transfer data into Frame FRAM because
//                        there are padding bytes in the structure for address alignment
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
#include "Diag_def.h"
#include "RealTime_def.h"               // Needed for Iod_ext.h

//
//      Local Definitions used in this module...
//
// Frame FRAM diagnostics data address definitions
//
#define DIAG_FRAM_SIZE                  60              // Note, this is in bytes and includes padding bytes
#define FADDR_CUST_DIAG1                0x4F8
#define FADDR_CUST_DIAG2                (FADDR_CUST_DIAG1 + DIAG_FRAM_SIZE)
#define FADDR_CUST_DIAG3                (FADDR_CUST_DIAG2 + DIAG_FRAM_SIZE)

#define FADDR_INT_DIAG1                 0x5AC
#define FADDR_INT_DIAG2                 (FADDR_CUST_DIAG1 + DIAG_FRAM_SIZE)
#define FADDR_INT_DIAG3                 (FADDR_CUST_DIAG2 + DIAG_FRAM_SIZE)



//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Iod_ext.h"
#include "Setpnt_ext.h"


// Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Diag_VarInit(void);
void Clear_Diag(void);
uint16_t Get_Diag(uint8_t int_cust);
void Write_Diag(uint8_t int_cust);
void Write_DefaultBrkHealthConfig(void);
uint8_t Get_BrkHealthConfig(void);
void Save_BrkHealthConfig(void);


//      Local Function Prototypes (These functions are called only within this module)
//
void Brk_HealthConfig_DefaultInit(void);



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
union HEALTH_CONFIG Health_Config;
union HEALTH_CONFIG Health_Config_Default;

struct HEALTH_DAT    Health_Data;
struct HEALTH_DAT    Internal_Health_Data;


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
// Array of starting addresses in Frame FRAM for diagnostics data
const uint16_t Faddr_Cust_Diag[3] = {FADDR_CUST_DIAG1, FADDR_CUST_DIAG2, FADDR_CUST_DIAG3};
const uint16_t Faddr_Int_Diag[3] = {FADDR_INT_DIAG1, FADDR_INT_DIAG2, FADDR_INT_DIAG3};


//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Diag_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Health and Diagnostics Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Health and Diagnostics Module.
//                      The following variables do not need initialization:
//
//  CAVEATS:            Call only during initialization
//
//  INPUTS:             None
//
//  OUTPUTS:            
//
//  ALTERS:             None
//
//  CALLS:              
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Diag_VarInit(void)
{

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Diag_VarInit()
//------------------------------------------------------------------------------------------------------------


//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Clear_Diag()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Health and Diagnostics Variable clearing
//
//  MECHANICS:          This subroutine resets  the variables used in the Health and Diagnostics Module 
//                      to default value.
//                      The following variables do not need initialization:
//
//  CAVEATS:            
//
//  INPUTS:             None
//
//  OUTPUTS:            
//
//  ALTERS:             None
//
//  CALLS:              
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Clear_Diag(void)
{

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Clear_Diag()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Get_Diag()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Diagnostics Info From Frame FRAM
//
//  MECHANICS:          This subroutine retrieves the health and diagnostics data from Frame FRAM.  The data
//                      is stored in SPI2_buf[].  The subroutine operates as follows:
//                          - Read first copy of diag data into SPI2_buf[]
//                          - If checksum is good, return True
//                          - Otherwise read second copy of diag data into SPI2_buf[]
//                          - If checksum is good, return True
//                          - Otherwise read third copy of diag data into SPI2_buf[]
//                          - If checksum is good, return True
//                      Any bad copies of the FRAM data is NOT corrected.  However, when diagnostics data is
//                      written, all three copies are updated and (presumably) corrected
//
//  CAVEATS:            None
//
//  INPUTS:             int_cust: 0 = internal diagnostics; otherwise customer diagnostics
//
//  OUTPUTS:            SPI2_buf[]
//                      The function returns True if a valid copy is retrieved; False otherwise
//
//  ALTERS:             None
//
//  CALLS:              Frame_FRAM_Read(), Checksum8_16()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

uint16_t Get_Diag(uint8_t int_cust)
{
  uint8_t i, valid;
  uint16_t checksum, checksum_read, checksumnot_read;

  valid = FALSE;

  // Read until a valid copy is found (up to 3 copies).  Contents placed in SPI2_buf[]
  // Note, DIAG_DATA_SIZE includes the checksum bytes
  for (i = 0; i < 3; i++)
  {
    if (int_cust == 0)
    {
      Frame_FRAM_Read(Faddr_Int_Diag[i], DIAG_DATA_SIZE, &SPI2_buf[0]);
    }
    else
    {
      Frame_FRAM_Read(Faddr_Cust_Diag[i], DIAG_DATA_SIZE, &SPI2_buf[0]);
    }
    checksum = Checksum8_16(&SPI2_buf[0], (DIAG_DATA_SIZE - 4) );
    checksum_read = (((uint16_t)SPI2_buf[DIAG_DATA_SIZE - 3]) << 8) + SPI2_buf[DIAG_DATA_SIZE - 4];
    checksumnot_read = (((uint16_t)SPI2_buf[DIAG_DATA_SIZE - 1]) << 8) + SPI2_buf[DIAG_DATA_SIZE - 2];
                                                         //check correctness of checksum
    if ((checksum == checksum_read) && ((checksum ^ 0xFFFF) == checksumnot_read))
    {
      valid = TRUE;
      break;
    }
  }

  // If no valid copy found, fill values with zeros (default values)
  if (valid == FALSE)
  {
    for (i = 0; i < (DIAG_DATA_SIZE - 4); i++)      // "- 4" because don't need checksum bytes
    {
      SPI2_buf[i] = 0;
    }
  }

  return (valid);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Get_Diag()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Write_Diag()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Diagnostics Info Into Frame FRAM
//
//  MECHANICS:          This subroutine writes the health and diagnostics data into Frame FRAM.  The data is
//                      assumed to be in SPI2_buf[].  It is assumed that the data was previously read into
//                      SPI2_buf[] and then modified.  The subroutine operates as follows:
//                          - Generate checksum and save in SPI_buf[]
//                          - Write first copy of diag data into Frame FRAM
//                          - Write second copy of diag data into Frame FRAM
//                          - Write third copy of diag data into Frame FRAM
//
//  CAVEATS:            None
//
//  INPUTS:             int_cust: 0 = internal diagnostics; otherwise customer diagnostics
//                      SPI2_buf[]
//
//  OUTPUTS:            None
//
//  ALTERS:             None
//
//  CALLS:              Checksum8_16(), Frame_FRAM_Write()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void Write_Diag(uint8_t int_cust)
{
  uint16_t checksum;
  uint8_t i;

  // Note, DIAG_DATA_SIZE includes the checksum bytes
  checksum = Checksum8_16(&SPI2_buf[0], (DIAG_DATA_SIZE - 4) );
  SPI2_buf[DIAG_DATA_SIZE - 4] = (uint8_t)checksum;
  SPI2_buf[DIAG_DATA_SIZE - 3] = (uint8_t)(checksum >> 8);
  checksum = checksum ^ 0xFFFF;                                 // Checksum complement
  SPI2_buf[DIAG_DATA_SIZE - 2] = (uint8_t)checksum;
  SPI2_buf[DIAG_DATA_SIZE - 1] = (uint8_t)(checksum >> 8);

  for (i = 0; i < 3; i++)
  {
    if (int_cust == 0)
    {
      Frame_FRAM_Write(Faddr_Int_Diag[i], DIAG_DATA_SIZE, &SPI2_buf[0]);
    }
    else
    {
      Frame_FRAM_Write(Faddr_Cust_Diag[i], DIAG_DATA_SIZE, &SPI2_buf[0]);
    }
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Write_Diag()
//------------------------------------------------------------------------------------------------------------

//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION   Brk_HealthConfig_DefaultInit()
//------------------------------------------------------------------------------
// FUNCTION:            initialization to breaker configuration default
//
void Brk_HealthConfig_DefaultInit(void)
{
  Health_Config_Default.data.Mech_Wear_Ref_1              = 2000;
  Health_Config_Default.data.Rate_1                       = 2;  
  Health_Config_Default.data.Current_Slope_1              = 2;
  Health_Config_Default.data.Current_Mech_Eq_Contact_1    = 2000;
  Health_Config_Default.data.Volt_slope_1                 = 1;
  Health_Config_Default.data.System_Voltage_1             = 480;   
  Health_Config_Default.data.Mech_Wear_Ref_2              = 0;
  Health_Config_Default.data.Rate_2                       = 0;
  Health_Config_Default.data.Current_Slope_2              = 0;
  Health_Config_Default.data.Current_Mech_Eq_Contact_2    = 0;
  Health_Config_Default.data.Volt_slope_2                 = 0;
  Health_Config_Default.data.System_Voltage_2             = 0;
  Health_Config_Default.data.Mech_Wear_Ref_3              = 0;
  Health_Config_Default.data.Rate_3                       = 0;
  Health_Config_Default.data.Current_Slope_3              = 0;
  Health_Config_Default.data.Current_Mech_Eq_Contact_3    = 0;
  Health_Config_Default.data.Volt_slope_3                 = 0;
  Health_Config_Default.data.System_Voltage_3             = 0;
  Health_Config_Default.data.MCR_Multiplier               = 1;       
  Health_Config_Default.data.Time_Temp_Adder_0            = 0;
  Health_Config_Default.data.Time_Temp_Adder_1            = 5;
  Health_Config_Default.data.Time_Temp_Adder_2            = 10;
  Health_Config_Default.data.Time_Temp_Adder_3            = 25;
  Health_Config_Default.data.Time_Temp_Adder_4            = 50;
  Health_Config_Default.data.Display_Points_Per_Bar_0     = 8750;
  Health_Config_Default.data.Display_Points_Per_Bar_1     = 6250;
  Health_Config_Default.data.Display_Points_Per_Bar_2     = 3750;
  Health_Config_Default.data.Display_Points_Per_Bar_3     = 1250;
  Health_Config_Default.data.Display_Points_Per_Bar_4     = 0;
  
}
//------------------------------------------------------------------------------
//  END OF FUNCTION     Brk_HealthConfig_DefaultInit()
//------------------------------------------------------------------------------


//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION   Write_DefaultBrkHealthConfig()
//------------------------------------------------------------------------------
// FUNCTION:            write default breaker Health Config information into FRAM.
//
void Write_DefaultBrkHealthConfig(void)
{
  uint16_t i;

  Brk_HealthConfig_DefaultInit();

  for(i =0; i<HEALTH_CON_BLOCK_SIZE - 4; i++)
  {
    Health_Config.buf[i] = Health_Config_Default.buf[i];  
  }

  Save_BrkHealthConfig();
}
//------------------------------------------------------------------------------
//  END OF FUNCTION     Write_DefaultBrkHealthConfig()
//------------------------------------------------------------------------------


//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION   Get_BrkHealth()
//------------------------------------------------------------------------------
// FUNCTION:            Read breaker Health Config information from FRAM.
//
uint8_t Get_BrkHealthConfig(void)
{
  uint16_t ckSum_cal, ckSum_read, ckSumNot_read;
  uint16_t j;
  uint8_t valid = FALSE;

  Frame_FRAM_Read(HEALTHCONFIG_FADDR , HEALTH_CON_BLOCK_SIZE, &SPI2_buf[0]);

  ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), HEALTH_CON_BLOCK_SIZE-4);
  ckSum_read = SPI2_buf[HEALTH_CON_BLOCK_SIZE - 4] + (((uint16_t)SPI2_buf[HEALTH_CON_BLOCK_SIZE - 3]) << 8);
  ckSumNot_read = SPI2_buf[HEALTH_CON_BLOCK_SIZE - 2] + (((uint16_t)SPI2_buf[HEALTH_CON_BLOCK_SIZE - 1]) << 8);

  if ((ckSum_cal == ckSum_read) && ((ckSum_cal ^ 0xFFFF)  == ckSumNot_read))
  {
    for(j =0; j< HEALTH_CON_BLOCK_SIZE - 4; j++)
    {
      Health_Config.buf[j] = SPI2_buf[j];  
    }
    valid = TRUE;
  }


  return valid;   
}
//------------------------------------------------------------------------------
//  END OF FUNCTION     Get_BrkHealthConfig()
//------------------------------------------------------------------------------

//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION   Save_BrkHealthConfig()
//------------------------------------------------------------------------------
// FUNCTION:            Save breaker Health Config information into FRAM.
//
void Save_BrkHealthConfig(void)
{
  uint16_t ckSum_cal, i;              
                            
  for (i = 0; i < (HEALTH_CON_BLOCK_SIZE - 4); i++)
  {
    SPI2_buf[i] = Health_Config.buf[i];
  }
  ckSum_cal = Checksum8_16((uint8_t *)(&SPI2_buf[0]), (HEALTH_CON_BLOCK_SIZE - 4));
  SPI2_buf[HEALTH_CON_BLOCK_SIZE - 4] = (uint8_t)ckSum_cal;
  SPI2_buf[HEALTH_CON_BLOCK_SIZE - 3] = (uint8_t)(ckSum_cal >> 8);
  SPI2_buf[HEALTH_CON_BLOCK_SIZE - 2] = (uint8_t)(ckSum_cal ^ 0xFFFF);
  SPI2_buf[HEALTH_CON_BLOCK_SIZE - 1] = (uint8_t)((ckSum_cal ^ 0xFFFF) >> 8);
                            
  Frame_FRAM_Write(HEALTHCONFIG_FADDR, HEALTH_CON_BLOCK_SIZE, &SPI2_buf[0]);

}
//------------------------------------------------------------------------------
//  END OF FUNCTION     Save_BrkHealthConfig()
//------------------------------------------------------------------------------


