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
//  MODULE NAME:        Ovrcom.c
//
//  MECHANICS:          Program module containing the subroutines to handle communications with the override
//                      microprocessor
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//    54    230802  BP  - Added the I2C Override comms code
//    93    231010  BP  - Added HW_SecInjTest.Enable condition when sending thresholds
//                        to the Override micro
//    108   231108  DAH - Minor revisions to Ovr_Micro_Comm() to eliminate compiler warnings
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
#include "Iod_def.h"
#include "Ovrcom_def.h"
#include "Init_def.h"
#include "Meter_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Intr_def.h"
#include "DispComm_def.h"
#include "Prot_def.h"
//
//      Local Definitions used in this module...
//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Init_ext.h"
#include "Meter_ext.h"
#include "Prot_ext.h"
#include "Events_ext.h"
#include "Demand_ext.h"
#include "Intr_ext.h"
#include "DispComm_ext.h"
#include "RealTime_ext.h"
#include "Setpnt_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Ovr_VarInit(void);
void Ovr_Micro_Comm(void);

//      Local Function Prototypes (These functions are called only within this module)
//


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct OVR_I2C_VARS OVR_I2C; 

uint8_t Manufacture_Mode;           // Manufacture Mode is used by the factory for things like Configuration and calibration.      
                                    // The IES Testers use Manufacture Mode at the Assembly station, Calibration Tester and 
                                    // End of Line Tester.

                                    // Breaker tripping is disabled when in Manufacture Mode.

                                    // We will need to implement a 20 minute time-out in case someone forgets to exit Manufacture Mode.

uint8_t Ovr_FW_Version;             // This is read from the Override micro

//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint16_t OVR_I2C_ErrCount;

//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Ovr_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Override Comms Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Ovrcom Module.
//                      The following variables do not need initialization:
//
//  CAVEATS:            Call only during initialization.  
//
//  INPUTS:             None
//
//  OUTPUTS:            OVR_I2C_ErrCount
//
//  ALTERS:             None
//
//  CALLS:              
//
//  EXECUTION TIME:    
//
//------------------------------------------------------------------------------------------------------------

void Ovr_VarInit(void)
{
  OVR_I2C_ErrCount = 0;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Ovr_VarInit()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//            START OF FUNCTION        Ovr_Micro_Comm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Communications processor for Override micro
// 
//  MECHANICS:          This subroutine is the top-level communications processor for an I2C Slave.  It
//                      basically operates as follows:
//                          - Check for a received message
//                          - Process the request
//                          - Assemble the response
//                          - Initiate transmissions
//                          - Wait for transmissions to end
//
//                      I2C Received Data: OVR_I2C.RxBuf[0] = Overide Micro firmware version
//                                         OVR_I2C.RxBuf[1] = trip reason
//                                         OVR_I2C.RxBuf[2] = Ia Trip Sample value (high-byte)
//                                         OVR_I2C.RxBuf[3] = Ia Trip Sample value (low-byte)  
//                                         OVR_I2C.RxBuf[4] = Ib Trip Sample value (high-byte)
//                                         OVR_I2C.RxBuf[5] = Ib Trip Sample value (low-byte) 
//                                         OVR_I2C.RxBuf[6] = Ic Trip Sample value (high-byte)
//                                         OVR_I2C.RxBuf[7] = Ic Trip Sample value (low-byte) 
//                                         OVR_I2C.RxBuf[8] = checksum high byte
//                                         OVR_I2C.RxBuf[9] = checksum low byte
//
//                      Trip reasons:  MM trip              0x01 / 0x02 / 0x03 - phase A B C
//                                     Override trip        0x04 / 0x05 / 0x06 - phase A B C
//                                     Digital bypass trip  0x07 / 0x08 / 0x09 - phase A B C
//                                     MCR trip             0x0A / 0x0B / 0x0C - phase A B C
//
//                      I2C Transmit Data: OVR_I2C.TxBuf[0] = Digital Bypass threshold (high-byte)
//                                         OVR_I2C.TxBuf[1] = Digital Bypass threshold (low-byte)
//                                         OVR_I2C.TxBuf[2] = MM/ARMS threshold (high-byte)
//                                         OVR_I2C.TxBuf[3] = MM/ARMS threshold (low-byte)
//                                         OVR_I2C.TxBuf[4] = Override threshold (high-byte)
//                                         OVR_I2C.TxBuf[5] = Override threshold (low-byte)
//                                         OVR_I2C.TxBuf[6] = MCR threshold (high-byte)
//                                         OVR_I2C.TxBuf[7] = MCR threshold (low-byte)
//                                         OVR_I2C.TxBuf[8] = Maintenance Mode On/Off from Main micro
//                                         OVR_I2C.TxBuf[9] = Maintenance Mode Hi/Lo Gain
//                                         OVR_I2C.TxBuf[10] = Digital Bypass Hi/Lo Gain
//                                         OVR_I2C.TxBuf[11] = Trip Reason
//                                         OVR_I2C.TxBuf[12] = Checksum (high-byte)
//                                         OVR_I2C.TxBuf[13] = Checksum (low-byte)
//                          
//  CAVEATS:            
//                      
//  INPUTS:             OVR_I2C.RxBuf[], MM_Threshold, OVR_Threshold, MCR_Threshold, DB_Threshold, 
//                      Setpoints0.stp.MM_Enable, Break_Config.config.OvrWithstand, Manufacture_Mode  
//                      
//  OUTPUTS:            OVR_I2C.TxBuf[], MM_TripFlg, HWInstTripFlg, McrTripFlg, DigBypassTripFlg, DigiBypassAlmFlg,
//                      Firmware_Ver
//                      
//  ALTERS:             OVR_I2C_ErrCount
//                      
//  CALLS:              Init_I2C2()      
// 
//------------------------------------------------------------------------------------------------------------

void Ovr_Micro_Comm(void)
{
  uint16_t temp;
  uint16_t checksum;
  
  uint8_t Rx_pic_version;
  uint8_t Rx_tripcause;
  uint8_t Rx_Ia_trip_sample_high;                   // *** BP - test variables
  uint8_t Rx_Ia_trip_sample_low;
  uint8_t Rx_Ib_trip_sample_high;
  uint8_t Rx_Ib_trip_sample_low;
  uint8_t Rx_Ic_trip_sample_high;
  uint8_t Rx_Ic_trip_sample_low;
  uint8_t Rx_cksm_high;
  uint8_t Rx_cksm_low;  
  
  uint8_t transmit_mm_threshold_high;     
  uint8_t transmit_mm_threshold_low; 
  uint8_t transmit_ovr_threshold_high;
  uint8_t transmit_ovr_threshold_low;
  uint8_t transmit_mcr_threshold_high;
  uint8_t transmit_mcr_threshold_low;
  uint8_t transmit_db_threshold_high;
  uint8_t transmit_db_threshold_low; 
  uint8_t transmit_feedback_tripcause;
  


  // decode received message - 10 bytes
  Rx_pic_version = OVR_I2C.RxBuf[0];           // 1st byte: Override Micro firmware version
  Rx_tripcause   = OVR_I2C.RxBuf[1];           // 2nd byte: trip reason

  Rx_Ia_trip_sample_high = OVR_I2C.RxBuf[2];   // 3rd byte   
  Rx_Ia_trip_sample_low  = OVR_I2C.RxBuf[3];   // 4th byte  
  Rx_Ib_trip_sample_high = OVR_I2C.RxBuf[4];   // 5th byte   
  Rx_Ib_trip_sample_low  = OVR_I2C.RxBuf[5];   // 6th byte   
  Rx_Ic_trip_sample_high = OVR_I2C.RxBuf[6];   // 7th byte
  Rx_Ic_trip_sample_low  = OVR_I2C.RxBuf[7];   // 8th byte

  Rx_cksm_high = OVR_I2C.RxBuf[8];              // 9th byte:  checksum high byte
  Rx_cksm_low  = OVR_I2C.RxBuf[9];              // 10th byte: checksum low byte
  
  
  // 2.2 check if received messsage checksum is correct
  checksum = 0;
  for(temp = 0; temp < 8; temp++)                    // calculate checksum
  {
    checksum += OVR_I2C.RxBuf[temp];
  }
  checksum = (checksum ^ 0xFFFF);
    
  if ((Rx_cksm_high == (uint8_t)(checksum >> 8)) && (Rx_cksm_low == (uint8_t)checksum))  // Rx checksum good
  {
    Ovr_FW_Version = Rx_pic_version;
    OVR_I2C_ErrCount = 0;                           // reset error count
  
  }
  else                                              // Rx checksum bad
  {
    OVR_I2C_ErrCount++;
    if (OVR_I2C_ErrCount >= 300)                    // 3 seconds of bad comms so set Alarm flag
    {
      OVR_I2C_ErrCount = 0;
      Init_I2C2();                                  // re-initialize I2C
      if (DigiBypassAlmFlg == 0)
      {
        TripFlagsReset();                                   // to allow alarm to be logged
        DigiBypassAlmFlg = 1;
                                    // *** BP - Insert Alarm event here
      }      
    }  
  }

  // 2.3 check if received trip cause buffer in valid range
  if ((Rx_tripcause >= 0x01) && (Rx_tripcause <= 0x0C))
  {    
    // *** BP - cause of trip.  Do we want to store the PIC trip sample in FRAM for troubleshooting?
    
    switch (Rx_tripcause)
    {
      case 0x01:                                            // Maintenance Mode trip
      case 0x02:
      case 0x03:       
       MM_TripFlg = 1;   
       HWInstTripFlg = 0;
       DigBypassTripFlg = 0;
       McrTripFlg = 0;      
       break; 
        
      case 0x04:                                            // Override trip
      case 0x05:
      case 0x06:       
       MM_TripFlg = 0;   
       HWInstTripFlg = 1;
       DigBypassTripFlg = 0;
       McrTripFlg = 0;    
       break; 
        
      case 0x07:                                            // Digital Bypass trip
      case 0x08:
      case 0x09:
       MM_TripFlg = 0;   
       HWInstTripFlg = 0;
       DigBypassTripFlg = 1;
       McrTripFlg = 0; 
       break; 
        
      case 0x0A:                                            // MCR trip
      case 0x0B:
      case 0x0C:        
       MM_TripFlg = 0;   
       HWInstTripFlg = 0;
       DigBypassTripFlg = 0;
       McrTripFlg = 1;    
       break; 
                 
    }  
    
  }
  
  
  //-------------------------------------------------------------------------------------------------------
  // 3. package buffer for next transmission
  //-------------------------------------------------------------------------------------------------------
  
  
  // Build ARMS trip threshold for Override micro.  Threshold is calculated in Gen_Values().
  if ((Setpoints0.stp.MM_Enable > 0) && ((Manufacture_Mode == FALSE) || (HW_SecInjTest.Enable == TRUE)))
  {
    transmit_mm_threshold_high = (uint16_t)MM_Threshold >> 8;         
    transmit_mm_threshold_low  = (uint16_t)MM_Threshold & 0x00FF;
  }
  else
  {
    transmit_mm_threshold_high  = 0;                                             // turn off ARMS protection in the Override micro
    transmit_mm_threshold_low   = 0;
  }
  
  
  // Build Override trip threshold for Override micro.  Threshold is calculated in Gen_Values().
  if ((Break_Config.config.OvrWithstand < 999) && ((Manufacture_Mode == FALSE) || (HW_SecInjTest.Enable == TRUE)))       // 999 is Override disabled setting
  {          
    transmit_ovr_threshold_high = (uint16_t)OVR_Threshold >> 8;         
    transmit_ovr_threshold_low  = (uint16_t)OVR_Threshold & 0x00FF;
  }
  else
  {
    transmit_ovr_threshold_high = 0;
    transmit_ovr_threshold_low  = 0;
  }
  
  
  // Build MCR trip threshold for Override micro.  Threshold is calculated in Gen_Values().
  if ((Manufacture_Mode == FALSE) || (HW_SecInjTest.Enable == TRUE))  
  {          
    transmit_mcr_threshold_high = (uint16_t)MCR_Threshold >> 8;     
    transmit_mcr_threshold_low  = (uint16_t)MCR_Threshold & 0x00FF;
  }
  else
  {
    transmit_mcr_threshold_high = 0;
    transmit_mcr_threshold_low  = 0;
  }
  
  
  // Build Digital Bypass trip threshold for Override micro.  Threshold is calculated in Gen_Values().
  if (Manufacture_Mode == FALSE)  
  {          
    transmit_db_threshold_high = (uint16_t)DB_Threshold >> 8;             
    transmit_db_threshold_low  = (uint16_t)DB_Threshold & 0x00FF;
  }
  else
  {
    transmit_db_threshold_high = 0;
    transmit_db_threshold_low  = 0;
  }  
  
  
  transmit_feedback_tripcause = Rx_tripcause;
  
  // Build message to be transmitted into buffer
  OVR_I2C.TxBuf[0] = transmit_db_threshold_high;                         // high-byte digital bypass threshold    
  OVR_I2C.TxBuf[1] = transmit_db_threshold_low;                          // low-byte digital bypass threshold        
  OVR_I2C.TxBuf[2] = transmit_mm_threshold_high;                         // high-byte MM threshold    
  OVR_I2C.TxBuf[3] = transmit_mm_threshold_low;                          // low-byte MM threshold     
  OVR_I2C.TxBuf[4] = transmit_ovr_threshold_high;                        // high-byte Override threshold  
  OVR_I2C.TxBuf[5] = transmit_ovr_threshold_low;                         // low-byte Override threshold   
  OVR_I2C.TxBuf[6] = transmit_mcr_threshold_high;                        // high-byte MCR threshold 
  OVR_I2C.TxBuf[7] = transmit_mcr_threshold_low;                         // low-byte MCR threshold       
  OVR_I2C.TxBuf[8] = ST_MM_On;                                           // MM on/off from Main micro - deternined in Gen_Values()
  OVR_I2C.TxBuf[9] = MM_HiLo_Gain;                                       // MM Hi/Lo gain - deternined in Gen_Values()
  OVR_I2C.TxBuf[10] = DB_HiLo_Gain;                                      // Digital Bypass Hi/Lo gain - deternined in Gen_Values()
  OVR_I2C.TxBuf[11] = transmit_feedback_tripcause;       
  

  // build checksum of Tx message   
  checksum = 0;
  for(temp = 0; temp < 12; temp++)
  {
    checksum += OVR_I2C.TxBuf[temp];
  }
  checksum = (checksum ^ 0xFFFF);
  
  OVR_I2C.TxBuf[12] = (checksum >> 8);                                   // checksum high-byte
  OVR_I2C.TxBuf[13] = (checksum & 0x00FF);                               // checksum low byte

  
}
//------------------------------------------------------------------------------------------------------------
//            END OF FUNCTION        Ovr_Micro_Comm()
//------------------------------------------------------------------------------------------------------------
