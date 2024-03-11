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
//  MODULE NAME:        CAMCom.c
//
//  MECHANICS:          Program module containing the subroutines to handle CAM communications
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.15   160718  DAH - Added support for basic CAM port testing
//                          - struct CAM1, struct CAM2, CAM_VarInit(), CAM_Rx(), and CAM_Tx() added
//   0.16   160818  DAH - Renamed CAMx.NumChars to CAMx.TxNumChars
//                      - Added headers to the subroutines
//   0.22   180420  DAH - Added include of Iod_def.h for compilation purposes (for Iod_ext.h)
//   0.37   200604  DAH - Added include of Events_def.h for compilation purposes (for FRAM_Flash_def.h)
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added include of RealTime_def.h and deleted includes of Iod_def.h and
//                            Iod_ext.h
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
#include "CAMCom_def.h"
#include "RealTime_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"

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



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void CAM_VarInit(struct CAMPORTVARS *port);
void CAM_Tx(struct CAMPORTVARS *port, DMA_Stream_TypeDef *DMA_Stream);
void CAM_Rx(struct CAMPORTVARS *port, DMA_Stream_TypeDef *DMA_Stream);


//      Local Function Prototypes (These functions are called only within this module)
//


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct CAMPORTVARS CAM1 @".sram2", CAM2 @".sram2";

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



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        CAM_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           CAM Port Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used for CAM communications.
//                      The following variables do not need initialization:
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             port - pointer to either CAM1 or CAM2
//
//  OUTPUTS:            CAMx.TxState, CAMx.RxState, CAMx.RxCharsLeft, CAMx.Status,
//                      CAMx.DMA_HISR_TCIF_FlagMask, CAMx.Temp, CAMx.RxNdxOut
//
//  ALTERS:             None
//
//  CALLS:              ReadInjCalConstants()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 2usec
//
//------------------------------------------------------------------------------------------------------------

void CAM_VarInit(struct CAMPORTVARS *port)
{

  port->TxState = 0;                        // Initialize CAM port Tx state to idle
  port->RxState = 0;                        // Initialize CAM port Rx state to idle
  port->RxCharsLeft = CAMPORT_RXBUFSIZE;    // Initialize number of chars in buffer to buffer size
  if (port == &CAM1)                        // Initialize TCIF flag mask
  {
    port->Status = CAM_FIRST_SAMPLE + CAM_TYPE_SAMPLE;    // Clear all status flags except first sample,
                                                          //   CAM1 is sampled-value type
    port->DMA_HISR_TCIF_FlagMask = DMA_HISR_TCIF7;
  }
  else
  {
    port->Status = CAM_FIRST_SAMPLE;                      // Clear all status flags except first sample,
                                                          //   CAM2 is normal (GOOSE) type
    port->DMA_HISR_TCIF_FlagMask = DMA_HISR_TCIF6;
  }
  port->Temp = 0;
  port->RxNdxOut = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CAM_VarInit()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        CAM_Tx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           CAM Port Top-Level Transmit Subroutine
//
//  MECHANICS:          This subroutine handles requests for CAM transmission, presently from the Test Port
//                      only.
//
//  CAVEATS:            None
//
//  INPUTS:             port - pointer to either CAM1 or CAM2
//                      DMA_Stream - pointer to the transmit DMA stream registers associated with either
//                                   CAM1 or CAM2
//                      CAMx.TxState, port->TxNumChars
//
//  OUTPUTS:            DMA2 Registers
//
//  ALTERS:             CAMx.TxState, port->Status
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void CAM_Tx(struct CAMPORTVARS *port, DMA_Stream_TypeDef *DMA_Stream)
{

  switch (port->TxState)
  {
    case 0:                           // Idle State
    default:
      if (port->Status & CAM_XMIT)        // Check flag to transmit
      {
        DMA_Stream->NDTR &= 0xFFFF0000;
        DMA_Stream->NDTR |= port->TxNumChars;
        // Must clear all event flags before initiating a DMA operation
        // *** DAH  THIS MUST BE EITHER STREAM 7 OR STREAM 6 - PROBABLY NEED TO ADD A PORT NUMBER AND IF STATEMENT WITH THE CORRECT MASKS!!!!!
//        DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
//                                + DMA_HIFCR_CFEIF6);
        DMA_Stream->CR |= 0x00000001;           // Initiate the DMA to transmit the data
        port->TxState = 1;
      }
      break;

    case 1:
      // Check whether done transmitting.  Note, for both CAM1 (Stream7) and CAM2 (Stream6), the TCIF flag
      //   is in the HISR register.  Flag mask is stored in DMA_HISR_TCIF_FlagMask
      if (DMA2->HISR & port->DMA_HISR_TCIF_FlagMask)
      {
        DMA2->HIFCR |= port->DMA_HISR_TCIF_FlagMask;    // Reset the transfer complete interrupt flag
        port->Status &= (~(CAM_ERROR + CAM_XMIT));      // Reset error and transmit flags
        port->TxState = 0;
      }
      break;                                            // Exit the routine in any event
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CAM_Tx()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        CAM_Rx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           CAM Port Top-Level Receiver Subroutine
//
//  MECHANICS:          This subroutine handles messages received over the CAM ports
//
//  CAVEATS:            None
//
//  INPUTS:             port - pointer to either CAM1 or CAM2
//                      DMA_Stream - pointer to the transmit DMA stream registers associated with either
//                                   CAM1 or CAM2
//                      CAMx.RxCharsLeft, port->RxState
//
//  OUTPUTS:            None
//
//  ALTERS:             CAMx.RxCharsLeft, port->RxNdxOut
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void CAM_Rx(struct CAMPORTVARS *port, DMA_Stream_TypeDef *DMA_Stream)
{
  uint8_t i;
  uint8_t charcount;

  // Capture the number of chars left in the buffer - need to take a snapshot in case this changes due to
  //   more characters being received while we are in this subroutine
  i = DMA_Stream->NDTR;

  // Compute the number of chars to process.  We must take into account when the buffer circles back around
  if (i <= port->RxCharsLeft)                   // No wraparound
  {
    charcount = port->RxCharsLeft - i;
  }
  else                                          // Buffer has gone past the end
  {
    charcount = (CAMPORT_RXBUFSIZE + port->RxCharsLeft) - i;
  }

  if (charcount == 0)                   // If no chars are received since the last time we were here, exit
  {
    return;
  }

  port->RxCharsLeft = i;                // Update the number of chars left in the buffer

  // Characters have been received - fall into the state machine to parse the message
  switch (port->RxState)
  {
    case 0:                             // Check for command
    default:
      for (i=0; i<charcount; ++i)
      {
        if (port->RxNdxOut >= (CAMPORT_RXBUFSIZE - 1))
        {
          port->RxNdxOut = 0;
        }
        else
        {
          port->RxNdxOut++;
        }
      }
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CAM_Rx()
//------------------------------------------------------------------------------------------------------------

