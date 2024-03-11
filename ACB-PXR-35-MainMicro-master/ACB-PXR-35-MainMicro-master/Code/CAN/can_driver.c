//------------------------------------------------------------------------------------------------------------
//                      Eaton Corporation
//
//                      Proprietary Information
//                      (C) Copyright 2023
//                      All rights reserved
//
//                      PXR35 Electronic Trip Unit
//
//------------------------------------------------------------------------------------------------------------
//  AUTHORS:            Xavier Pacheco         
//                      Eaton Corporation
//                      Santo Domingo Design Center
//                      Av. Los Proceres, Santo Domingo 10604, DR
//
//------------------------------------------------------------------------------------------------------------
//  PRODUCT:            PXR35       Trip unit for air circuit and molded-case circuit breakers
//
//  FIRMWARE DRAWING:   ????????    This drawing combines the unprogrammed STM32F407IGT7 with the code's
//                                  flash programming file to produce an "assembly group" that is the
//                                  programmed device.
//
//  PROCESSOR:          ST Micro STM32F407IGT7
//
//  COMPILER:           IAR C/C++ Compiler for ARM - v8.50.4.26143
//                      IAR Embedded Workbench from IAR Systems
//
//  MODULE NAME:        can_driver.c
//
//  MECHANICS:          This module contains the CAN controller drivers.
//
//  TARGET HARDWARE:    PXR35 Rev ? and later boards
//
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//                   Global Definitions from external files...
//------------------------------------------------------------------------------------------------------------
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include <stdbool.h>
#include "pxcan_def.h"
#include "can_driver_def.h"
#include "RealTime_def.h"
#include "Iod_def.h"

//------------------------------------------------------------------------------------------------------------
//                   Global Declarations from external files...
//------------------------------------------------------------------------------------------------------------
#include "can_tasks_ext.h"
#include "pxcan_ext.h"

//--------------------------------------------------------------------------------------------------
// External (Visible) Function Prototypes
//--------------------------------------------------------------------------------------------------
void checkCanStatus(void);
void increment_TME_Counters(void);

//--------------------------------------------------------------------------------------------------
// Local data declarations
//--------------------------------------------------------------------------------------------------

static CAN_HandleTypeDef   canHandle;
static uint32_t TME0_Counter;
static uint32_t TME1_Counter;
static uint32_t TME2_Counter;
static uint32_t MultiFrameCounter;
static uint32_t lastFifoStatus;
static uint32_t lastBusOffStatus;
static uint32_t lastErrorPassiveStatus;

//--------------------------------------------------------------------------------------------------
// Local (Private) Function Prototypes
//--------------------------------------------------------------------------------------------------

static void CAN_FilterConfig(CanAddress_t addr);

//--------------------------------------------------------------------------------------------------
// CAN Timing definitions
//--------------------------------------------------------------------------------------------------
/*
CAN Bit timing calculation 
Source: http://www.bittiming.can-wiki.info/
Parameters:
Clock rate = 30 MHz
Sample-point = 87.5%
Synchronization Jump Width (SJW) = 1
Bit rate = 125 kbps
 *
 *   --------------------------------------------------------------------------------------------
 *   | Bit rate | Prescaler |  Number of |            Seg 1         |   Seg 2    |  Sample point |
     |          |           | time quanta|  (Prop_Seg+Phase_Seg1)   |            |       %      |
 *   --------------------------------------------------------------------------------------------
 *   |   125    |     8     |     16     |            13            |      2     |      87.5     |  
 *   --------------------------------------------------------------------------------------------
 *
*/

static const uint32_t can_prescaler = 15;
static const uint32_t can_tseg1     = 13;
static const uint32_t can_tseg2     = 2;
static const uint32_t can_sjw       = 1;

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CanInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Initializes the CAN controller in polling mode and synchronizes it to the CAN bus.
//
//  MECHANICS:        This subroutine initializes the CAN controller hardware and performs the
//                    following actions:
//                         - Configures the CAN reception filter
//                         - Starts the CAN module
//
//  CAVEATS:          None 
//
//  INPUTS:           None
//
//  OUTPUTS:          None
//
//  ALTERS:           CAN1 registers
// 
//  CALLS:            CAN_FilterConfig()
//
//  EXECUTION TIME:   None   
// 
//------------------------------------------------------------------------------------------------------------

void CanInit(CanAddress_t addr)
{
   
  // CAN1 clock enable 
  CAN1_CLK_ENABLE();
  
  //---------------- CAN master control register ----------------
  // b31..17 = 0: Reserved, must be kept at reset value.
  // b16     = 0: Debug freeze - CAN working during debug
  // b15     = 0: RESET - Normal operation.
  // b14..8  = 0: Reserved, must be kept at reset value.
  // b7      = 0: Time Triggered Communication mode disabled.
  // b6      = 1: The Bus-Off state is left on hardware request
  // b5      = 0: The Sleep mode is left on software request
  // b4      = 0: Set the automatic retransmission
  // b3      = 1: Enable receive FIFO locked mode
  // b2      = 0: Priority driven by the identifier of the message
  // b1      = 0: Sleep mode request
  // b0      = 1: Initialization request
  
  // Request initialization 
  CAN1->MCR |= CAN_MCR_INRQ;
 
  // Exit from sleep mode
  CAN1->MCR &= ~CAN_MCR_SLEEP;
  
  //Enable Automatic Bus-off recovery
  CAN1->MCR |= CAN_MCR_ABOM;
  
  CAN1->MCR &= ~(CAN_MCR_DBF | CAN_MCR_TTCM | CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_TXFP);
  
  //Receive FIFO locked against overrun. Once a receive FIFO is full the next incoming
  //message will be discarded.
  CAN1->MCR |= CAN_MCR_RFLM;
  
  // Set the bit timing register 
  CAN1->BTR =  (uint32_t)(CAN_MODE_NORMAL                      | \
                         ((can_sjw - 1U) << CAN_BTR_SJW_Pos)   | \
                         ((can_tseg1 - 1U) << CAN_BTR_TS1_Pos) | \
                         ((can_tseg2 - 1U) << CAN_BTR_TS2_Pos) | \
                         ((can_prescaler - 1U) << CAN_BTR_BRP_Pos));

  
 // Configures the CAN reception filter. 
 CAN_FilterConfig(addr);

 // Request leave initialization and start the CAN module
 CAN1->MCR &= ~CAN_MCR_INRQ;
 
 // Initialize bus status variables
 TME0_Counter = 0;
 TME1_Counter = 0;
 TME2_Counter = 0;
 MultiFrameCounter = 0;
 lastFifoStatus = CAN_TSR_TME;
 lastBusOffStatus = CAN_ESR_BOFF;
 lastErrorPassiveStatus = CAN_ESR_EPVF;
}  


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CanInit()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CAN_FilterConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Configures the CAN reception filter
//
//  MECHANICS:        Initializes and configure CAN filter
//                    This is how the CAN filter works:
//                       Start with a Message ID. Then XOR it with the Filter ID. The result is AND with
//                       the Mask. If the result is ZERO, then the message is accepted.
//
//  CAVEATS:          None
//
//  INPUTS:           addr - CAN node address. 
//
//  OUTPUTS:          None
//
//  ALTERS:           CAN1 registers
// 
//  CALLS:            None
//
//  EXECUTION TIME:   None   
// 
//------------------------------------------------------------------------------------------------------------

static void CAN_FilterConfig(CanAddress_t addr)
{
  CAN_FilterTypeDef   filterConfig;
  uint32_t filternbrbitpos;
  uint32_t  rxFilterId, rxFilterMask;
  
  // Fixed filter parameters
  rxFilterMask = (uint32_t)(PXCAN_TARGET_MASK << 3) | CAN_RI0R_IDE;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // Initialization mode for the filter
  CAN1->FMR |= CAN_FMR_FINIT;
  
    // Configure first filter
  filterConfig.FilterBank = 0;
  rxFilterId = (uint32_t)((addr << PXCAN_TARGET_POS) << 3) | CAN_RI0R_IDE;
  
  // Convert filter number into bit position
  filternbrbitpos = (uint32_t)1 << (filterConfig.FilterBank & 0x1FU);
  
  // Filter Deactivation 
  CAN1->FA1R &= ~filternbrbitpos;
  
   // 32-bit scale for the filter 
  CAN1->FS1R |= filternbrbitpos;
  
  // 32-bit identifier or First 32-bit identifier 
  CAN1->sFilterRegister[filterConfig.FilterBank].FR1 = rxFilterId;

  // 32-bit mask or Second 32-bit identifier 
  CAN1->sFilterRegister[filterConfig.FilterBank].FR2 = rxFilterMask;
 
  // Mask mode for the filter
  CAN1->FM1R &= ~filternbrbitpos;
  
  // FIFO 0 assignation for the filter 
  CAN1->FFA1R &= ~filternbrbitpos;
  
  // Filter activation 
  CAN1->FA1R |= filternbrbitpos;
  
  // Configure second filter
  filterConfig.FilterBank++;
  rxFilterId = (uint32_t)((BROADCAST_ADDR << PXCAN_TARGET_POS) << 3) | CAN_RI0R_IDE;
  
  // Convert filter number into bit position
  filternbrbitpos = (uint32_t)1 << (filterConfig.FilterBank & 0x1FU);
  
  // Filter Deactivation 
  CAN1->FA1R &= ~filternbrbitpos;
  
   // 32-bit scale for the filter 
  CAN1->FS1R |= filternbrbitpos;
  
  // 32-bit identifier or First 32-bit identifier 
  CAN1->sFilterRegister[filterConfig.FilterBank].FR1 = rxFilterId;

  // 32-bit mask or Second 32-bit identifier 
  CAN1->sFilterRegister[filterConfig.FilterBank].FR2 = rxFilterMask;
  
  // Mask mode for the filter
  CAN1->FM1R &= ~filternbrbitpos;
  
  // FIFO 1 assignation for the filter 
  CAN1->FFA1R |= filternbrbitpos;
  
  // Filter activation 
  CAN1->FA1R |= filternbrbitpos;
  
   // Leave the initialization mode for the filters
  CAN1->FMR &= ~CAN_FMR_FINIT;
  
} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CAN_FilterConfig()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CanTransmitPacket()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Transmist a CAN frame
//
//  MECHANICS:        Add a message to the first free Tx mailbox and activate the
//                    corresponding transmission request
//
//  CAVEATS:          1. Only 3 Transmission Mailboxes available.
//                    Needs to make sure there's always a free Tx mailbox. If transmission requests
//                    are done too fast (< 1ms), the CAN controller may not have enough time to release the mailboxes.
//                    2. This functions works if there's has been a Tx request prior to calling it.
//
//  INPUTS:           pCanTx - pointer to CanTxMsg_t structure
//
//  OUTPUTS:          Transmission status, PXCAN_OK or PXCAN_ERROR
//                    PXCAN_OK: Transmission successful
//                    PXCAN_ERROR: Transmission mailboxes are full. This can happen if no devices have ACK the message
//
//  ALTERS:           CAN1 registers
// 
//  CALLS:            None
//
//  EXECUTION TIME:   None  
// 
//------------------------------------------------------------------------------------------------------------

uint8_t CanTransmitPacket(CanTxMsg_t *pCanTx)
{
  //TESTPIN_D1_HIGH;
  uint32_t transmitmailbox;
  uint32_t tsr = CAN1->TSR;
  uint8_t status = PXCAN_ERROR;
  
  // Check that any of Tx mailboxes is not full 
   if  (((tsr & CAN_TSR_TME0) != 0U) ||
        ((tsr & CAN_TSR_TME1) != 0U) ||
        ((tsr & CAN_TSR_TME2) != 0U))  
    {
      status = PXCAN_OK;   
      // Select an empty transmit mailbox 
      transmitmailbox = (tsr & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
      
      if (transmitmailbox > 2U)
       {
        transmitmailbox = 2U;
        status = PXCAN_ERROR;
       }
  
     if(status == PXCAN_OK)
     {     

       // configure the Extended CAN frame     
       CAN1->sTxMailBox[transmitmailbox].TIR = ((pCanTx->Id.all << CAN_TI0R_EXID_Pos) |
                                                                           CAN_ID_EXT |
                                                                         CAN_RTR_DATA);
       // Set Data Length
       CAN1->sTxMailBox[transmitmailbox].TDTR = pCanTx->DataLen;
      
       // Set up the data field 
       CAN1->sTxMailBox[transmitmailbox].TDHR = 
                (((uint32_t)pCanTx->Data[7] << CAN_TDH0R_DATA7_Pos)|
                ((uint32_t)pCanTx->Data[6] << CAN_TDH0R_DATA6_Pos) |
                ((uint32_t)pCanTx->Data[5] << CAN_TDH0R_DATA5_Pos) |
                ((uint32_t)pCanTx->Data[4] << CAN_TDH0R_DATA4_Pos));
       CAN1->sTxMailBox[transmitmailbox].TDLR =
                (((uint32_t)pCanTx->Data[3] << CAN_TDL0R_DATA3_Pos)|
                ((uint32_t)pCanTx->Data[2] << CAN_TDL0R_DATA2_Pos) |
                ((uint32_t)pCanTx->Data[1] << CAN_TDL0R_DATA1_Pos) |
                ((uint32_t)pCanTx->Data[0] << CAN_TDL0R_DATA0_Pos));
      
       // Request transmission 
       CAN1->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;   
       
      //TESTPIN_D1_LOW;
    }
   }
   else
   { // Somehow the buffer is full. This should not happen  
     status = PXCAN_ERROR;
     CanSystemErrorReg |= (CAN_TX_FIFO_FULL);
   }
  return status; 
 }   
 
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CanTransmitPacket()
//------------------------------------------------------------------------------------------------------------

// This routine must be called in 1 sec anniversary 
void checkCanStatus(void)
{ 
  
  uint32_t TxAbortRequest = 0;
  
  if ((CAN1->TSR & CAN_TSR_TME) != lastFifoStatus)
  {
    lastFifoStatus = (CAN1->TSR & CAN_TSR_TME);
    
    if (lastFifoStatus == 0)
      { 
        CanSystemErrorReg |= (CAN_TX_FIFO_FULL);  
      } 
    else if (lastFifoStatus == CAN_TSR_TME)
      {
        CanSystemErrorReg &= ~(CAN_TX_FIFO_FULL);  
      } 
  }
  
  if((CAN1->ESR & CAN_ESR_BOFF) != lastBusOffStatus)
  {
    lastBusOffStatus = (CAN1->ESR & CAN_ESR_BOFF);
    
    if (lastBusOffStatus == 0)
      { 
        CanSystemErrorReg &= ~(CAN_BUS_OFF);  
      } 
    else if (lastBusOffStatus == CAN_ESR_BOFF)
      {
        CanSystemErrorReg |= (CAN_BUS_OFF);  
      } 
   }
  
  if((CAN1->ESR & CAN_ESR_EPVF) != lastErrorPassiveStatus)
   {
    lastErrorPassiveStatus = (CAN1->ESR & CAN_ESR_EPVF);
    
    if (lastErrorPassiveStatus == 0)
      { 
        CanSystemErrorReg &= ~(CAN_ERROR_PASSIVE);  
      } 
    else if (lastErrorPassiveStatus == CAN_ESR_EPVF)
      {
        CanSystemErrorReg |= (CAN_ERROR_PASSIVE);  
      } 
   }
  
  // Check Tx Mailbox 0 status
  if ((CAN1->TSR & CAN_TSR_TME0) == 0U) // Message pending
    {
      if(TME0_Counter > CAN_TX_MAILBOX_TIMEOUT_SEC)
       {
        // Add cancellation request for Tx Mailbox 0 
        CAN1->TSR |= CAN_TSR_ABRQ0; 
        TxAbortRequest = 1;
        TME0_Counter = 0;
       }
    }
  else {TME0_Counter = 0;}
  
  // Check Tx Mailbox 1 status 
  if ((CAN1->TSR & CAN_TSR_TME1) == 0U) // Message pending
    {
      if(TME1_Counter > CAN_TX_MAILBOX_TIMEOUT_SEC)
      {
       // Add cancellation request for Tx Mailbox 1
       CAN1->TSR |= CAN_TSR_ABRQ1; 
       TxAbortRequest = 1;
       TME1_Counter = 0;
      } 
    }
  else {TME1_Counter = 0;}
  
 // Check Tx Mailbox 2 status 
  if ((CAN1->TSR & CAN_TSR_TME2) == 0U) // Message pending
    {
     if(TME2_Counter > CAN_TX_MAILBOX_TIMEOUT_SEC)
      {
        // Add cancellation request for Tx Mailbox 2
        CAN1->TSR |= CAN_TSR_ABRQ2; 
        TxAbortRequest = 1;
        TME2_Counter = 0;
      }
    } 
  else {TME2_Counter = 0;}
  
  if(NoMultiFrameGoing() != true)
  {
    if (MultiFrameCounter > CAN_TX_MAILBOX_TIMEOUT_SEC)
    {
      TxAbortRequest = 1;
      MultiFrameCounter = 0;     
    }
  }
  else {MultiFrameCounter = 0;}

  if ((CanSystemErrorReg != 0) || (TxAbortRequest != 0))
  {
   // Make sure to cancel any request
   CAN1->TSR |= (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2); 
   // reset multi-frame flags
   ResetMultiFrameFlags();
   //reset Copy Process flags
   SetpointsCopyProcess = 0;
  }
}

void increment_TME_Counters(void)
{ 
  TME0_Counter++;
  TME1_Counter++;
  TME2_Counter++;
  MultiFrameCounter++;
}

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CanReceivePacket()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Receive a CAN frame
//
//  MECHANICS:        Get an CAN frame from the Rx FIFO zone into the message RAM.
//
//  CAVEATS:          None
//
//  INPUTS:           pCanRx - pointer to CanRxMsg_t structure
//                    RxLocation - FIFO zone to check
//
//  OUTPUTS:          Reception status, PXCAN_OK or PXCAN_ERROR
//                    PXCAN_OK: Message successfully retrieved
//                    PXCAN_ERROR: No message available
//                    PXCAN_INVALID_PARAMETER: Invalid FIFO number
//
//  ALTERS:           None
// 
//  CALLS:            None
//
//  EXECUTION TIME:   None 
// 
//------------------------------------------------------------------------------------------------------------

uint8_t CanReceivePacket(CanRxMsg_t *pCanRx, uint32_t RxLocation)

{

 uint32_t RxFifo = RxLocation;
 uint8_t status = PXCAN_ERROR;
 
 if (RxLocation == CAN_RX_FIFO0) /* Rx element is assigned to the Rx FIFO 0 */
   {   
        //Check that the Rx FIFO 0 is not empty
      if ( (CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
      {
        status = PXCAN_OK;
      }
   }
  
 else if (RxLocation == CAN_RX_FIFO1) /* Rx element is assigned to the Rx FIFO 1 */
   {  
        //Check that the Rx FIFO 1 is not empty
      if ((CAN1->RF1R & CAN_RF1R_FMP1) != 0U)
      {
       status = PXCAN_OK;
      }
   }
 else
  {
    status = PXCAN_INVALID_PARAMETER;
  }
 
 if (status == PXCAN_OK)
 {
     /* Get the header */ 
    pCanRx->Id.all = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[RxFifo].RIR) >> CAN_RI0R_EXID_Pos;
    pCanRx->DataLen = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[RxFifo].RDTR) >> CAN_RDT0R_DLC_Pos;
     /* Get the data */
    pCanRx->Data[0] = (uint8_t)((CAN_RDL0R_DATA0 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA0_Pos);
    pCanRx->Data[1] = (uint8_t)((CAN_RDL0R_DATA1 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA1_Pos);
    pCanRx->Data[2] = (uint8_t)((CAN_RDL0R_DATA2 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA2_Pos);
    pCanRx->Data[3] = (uint8_t)((CAN_RDL0R_DATA3 & CAN1->sFIFOMailBox[RxFifo].RDLR) >> CAN_RDL0R_DATA3_Pos);
    pCanRx->Data[4] = (uint8_t)((CAN_RDH0R_DATA4 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA4_Pos);
    pCanRx->Data[5] = (uint8_t)((CAN_RDH0R_DATA5 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA5_Pos);
    pCanRx->Data[6] = (uint8_t)((CAN_RDH0R_DATA6 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA6_Pos);
    pCanRx->Data[7] = (uint8_t)((CAN_RDH0R_DATA7 & CAN1->sFIFOMailBox[RxFifo].RDHR) >> CAN_RDH0R_DATA7_Pos);
   
    if (RxLocation == CAN_RX_FIFO0)
    {
   /* Release RX FIFO 0 */
      CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
    else if (RxLocation == CAN_RX_FIFO1)
    {
      /* Release RX FIFO 1 */
      CAN1->RF1R |= CAN_RF1R_RFOM1;  
    }
 }
 return status;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CanReceivePacket()
//------------------------------------------------------------------------------------------------------------


/*********************************** end of can_driver.c *********************************/
