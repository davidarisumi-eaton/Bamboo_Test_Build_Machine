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
//  MODULE NAME:        can_tasks.c
//
//  MECHANICS:          This module contains the CAN bus tasks.
//
//  TARGET HARDWARE:    PXR35 Rev ? and later boards
//
//------------------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Include header files for external module definitions & declarations
//--------------------------------------------------------------------------------------------------
#include "stm32f407xx.h"
#include "stm32f4xx.h"
#include <stdbool.h>
#include "pxcan_def.h"
#include "can_driver_def.h"
#include "can_setpoints_def.h"
#include "RealTime_def.h"
#include "can_tasks_def.h"
#include "Flags_def.h"
#include "Setpnt_def.h"

//------------------------------------------------------------------------------------------------------------
//                   Global Declarations from external files...
//------------------------------------------------------------------------------------------------------------
#include "Setpnt_ext.h"
#include "pxcan_ext.h"
#include "can_setpoints_ext.h"
#include "Iod_ext.h"

//--------------------------------------------------------------------------------------------------
// External (Visible) Function Prototypes
//--------------------------------------------------------------------------------------------------
void Init_Can(void);
void CanTasks(void);
void CopyToCassetteFlags(void);

//--------------------------------------------------------------------------------------------------
// Local function prototypes
//--------------------------------------------------------------------------------------------------
static void CopySetpointsToCassette (CanRxMsg_t const * const pCanRx);
static void CopySetpointsProcess (void);
static void IOBlock_ProcessPublication(CanRxMsg_t const * const pCanRx);

//--------------------------------------------------------------------------------------------------
// Global data declarations
//--------------------------------------------------------------------------------------------------
bool     protection_processor;
volatile uint8_t  IOBlock_Inputs;
volatile uint32_t CanSystemErrorReg;
uint8_t SetpointsCopyProcess;
//--------------------------------------------------------------------------------------------------
// Local data declarations
//--------------------------------------------------------------------------------------------------
static const uint8_t Protection_GlobalString[DATA_BYTES_8] = {'P', 'R', 'O', 'T', 'P', 'R', 'O', 'C'};
static uint8_t Set;
static uint8_t Group;
static uint8_t CopytoCassetteCmdReceived;
static CanRxMsg_t RxInfo;

//--------------------------------------------------------------------------------------------------
// Strong function definitions - see the Weak ones in pxcan.c module
//--------------------------------------------------------------------------------------------------

void FRAMx_Read(uint16_t read_addr, uint8_t *read_buff, uint16_t bytes)
{
 Frame_FRAM_Read(read_addr, bytes, read_buff);
}
void FRAMx_Write(uint16_t write_addr, uint8_t *write_buff, uint16_t bytes)
{
 Frame_FRAM_Write(write_addr, bytes, write_buff);
}

//------------------------------------------------------------------------------------------------------------
//              START OF PROTECTION PROCESSOR COMMAND TABLE
//------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// Protection Processor Array of all of the supported commands and the function that should
// be executed when the command is received.
//--------------------------------------------------------------------------------------------------
const Command_RxCmdListType ProtectionCommandsList[] =
{
  // Setpoints R/W commands (from 0x14 to 0x2F)
  {READ_SETPOINTS,              Read_Setpoints_Cmd_Rx     },
  {WRITE_SETPOINTS,             Write_Setpoints_Cmd_Rx    },
  {COPY_SETPOINTS_TO_CASSETTE,  CopySetpointsToCassette   },
  // Available for further commands
  {END_OF_COMMANDS,             0x00                      },       	                   
};

//------------------------------------------------------------------------------------------------------------
//              END OF PROTECTION PROCESSOR COMMAND TABLE
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//              START OF CASSETTE PROCESSOR RESPONSES TO PROTECTION PROCESSOR   
//------------------------------------------------------------------------------------------------------------
// Defines an array of all of the supported commands and the function that should
// be executed when the command is received.

const Command_RxCmdListType CassetteResponseList[] =
{
  // Commands                   // Functions calls
  // Setpoints R/W commands (from 0x14 to 0x2F)
  {WRITE_SETPOINTS,             ProcessWriteCmdResponse  },
  {END_OF_COMMANDS,             0x00                     },
};

//------------------------------------------------------------------------------------------------------------
//              END OF CASSETTE PROCESSOR RESPONSES TO PROTECTION PROCESSOR 
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Init_Can()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Protection processor CAN bus initialization
//
//  MECHANICS:        Init CAN driver and PXCAN protocol
//                    Init setpoints flags 
//
//  CAVEATS:          None
//
//  INPUTS:           None
//
//  OUTPUTS:          None
//
//  ALTERS:           None
// 
//  CALLS:            PxCanInit()
//
//  EXECUTION TIME:   None  
// 
//------------------------------------------------------------------------------------------------------------

void Init_Can(void)
{ 
  CanSystemErrorReg = 0;
  
  PxCanInit(PROTECTION_ADDR);
  
  protection_processor = true;
  cassette_processor = false;
  // Reset Copy Process flags
  SetpointsCopyProcess = 0;
  // Set Write Ready Flags indicating there's no a Write process pending 
  SetpointsWriteReady = ((uint64_t)0xFFFFFFFFFFFFFFFF);
  // Initialize set-group 
  Set = SET_A;
  Group = GROUP0;
  IOBlock_Inputs = 0;
  CopytoCassetteCmdReceived = false;
}


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CanTasks()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:          Protection processor CAN bus tasks
//
//  MECHANICS:         Checks for Rx FIFO0 and Rx FIFO1 for incoming messages
//                     Monitors Copy Setpoints to Cassette routine
//
//  CAVEATS:           None
//
//  INPUTS:            None
//
//  OUTPUTS:           None
//
//  ALTERS:            None
// 
//  CALLS:             PxCan_CommandProcess(), PxCan_ProcessCmdResponse(), CopySetpointsProcess()
//
//  EXECUTION TIME:    None
// 
//------------------------------------------------------------------------------------------------------------

void CanTasks(void)

{
  // Rx data objects
  CanRxMsg_t ProtectionRx;
  CanRxMsg_t BroadcastRx;
  const Command_RxCmdListType * CmdResponseListPtr = _NULL;
  const Command_RxCmdListType * CmdListPtr = _NULL;

  //Poll CAN Rx FIFO0 
  if (CanReceivePacket(&ProtectionRx, CAN_RX_FIFO0) == PXCAN_OK)  
  {
    CanRxMsg_t const * const pCanRx0 = &ProtectionRx;
    
    if ((pCanRx0->Id.TargetAddr == PROTECTION_ADDR) && ((pCanRx0->Id.SrcAddr == DISPLAY_ADDR)  || 
                                                      (pCanRx0->Id.SrcAddr == CASSETTE_ADDR)  ||
                                                      (pCanRx0->Id.SrcAddr == PC_MASTER_ADDR)))
      {

         if (pCanRx0->Id.PacketType == PXCAN_CMD)
         { 
           
           if(pCanRx0->Id.CmdId < MAX_GLOBAL_COMMANDS)
            {
             CmdListPtr = GlobalCmdListPtr;
            }
           else
            {
             CmdListPtr = ProtectionCommandsList;
            }
           
            if(CmdListPtr != _NULL)
             {
               PxCan_CommandProcess(pCanRx0, CmdListPtr);
             }            
         }      
        else
          {
           switch (pCanRx0->Id.SrcAddr)
            {
              case CASSETTE_ADDR:
                CmdResponseListPtr = CassetteResponseList; 
              break;
            }  
           
            if(CmdResponseListPtr != _NULL)
             {
               PxCan_ProcessCmdResponse(pCanRx0, CmdResponseListPtr);
             }           
          }
       }
  }   
    //Poll CAN Rx FIFO1 for Broadcast messages
  if (CanReceivePacket(&BroadcastRx, CAN_RX_FIFO1) == PXCAN_OK)  
  {
    CanRxMsg_t const * const pCanRx1 = &BroadcastRx;
    
    if(pCanRx1->Id.TargetAddr == BROADCAST_ADDR)
    {     
      if(((pCanRx1->Id.SrcAddr == PC_MASTER_ADDR) || (pCanRx1->Id.SrcAddr == DISPLAY_ADDR)) && 
          (pCanRx1->Id.PacketType == PXCAN_CMD) &&
          (pCanRx1->Id.Priority == PXCAN_LOW_PRIORITY))
      {
       PxCan_CommandProcess(pCanRx1, GlobalCmdListPtr);
      }
      else if(pCanRx1->Id.PacketType == PXCAN_PUBLICATION)
      {
       switch (pCanRx1->Id.SrcAddr)
        {            
           case IOBLOCK_ADDR:
                IOBlock_ProcessPublication(pCanRx1);
           break;
        }              
      }
    }
  }
  // Check Setpoints Copy Process
  CopySetpointsProcess();
  
  // Check bus status
  checkCanStatus();
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CanTasks()
//------------------------------------------------------------------------------------------------------------

static void IOBlock_ProcessPublication(CanRxMsg_t const * const pCanRx)

{
  
  switch (pCanRx->Id.CmdId)
    {     
     case IOBLOCK_INPUTS:
           IOBlock_Inputs = pCanRx->Data[0];
     break;    
    }  
       
}

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       SetGlobalStringResponse()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:          Send String Response
//
//  MECHANICS:         This function responds to the "READ GLOBAL STRING" command. It returns a
//                     specific 8-bytes string 
//                     
//  CAVEATS:           None
//
//  INPUTS:            pCanRx - const pointer to const CanRxMsg_t structure
//
//  OUTPUTS:           None
//
//  ALTERS:            None
// 
//  CALLS:             NoMultiFrameGoing(), PxCan_Set_CmdResponse()
//
//  EXECUTION TIME:    None
// 
//------------------------------------------------------------------------------------------------------------

void SetGlobalStringResponse(CanRxMsg_t const * const pCanRx)
{
  // Only accept this command if there are no on-going multi-frame processes
  if ( NoMultiFrameGoing() )
   {
      switch(pCanRx->Id.FrameType)
      {
        case PXCAN_SINGLE_FRAME:    
            PxCan_Set_CmdResponse(pCanRx, (uint8_t *)Protection_GlobalString, DATA_BYTES_8);                    
        break;   
      } 
   }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          SetGlobalStringResponse()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CopySetpointsToCassette()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:          Copy Setpoints to Cassette CAN command
//
//  MECHANICS:         This function receives the CAN command "COPY_SETPOINTS_TO_CASSETTE"
//                     It resets the SetpointsWriteReady and SetpointsCopyProcess flags to initiate a copy to cassette
//
//  CAVEATS:           None
//
//  INPUTS:            pCanRx - const pointer to const CanRxMsg_t structure
//
//  OUTPUTS:           None
//
//  ALTERS:            SetpointsWriteReady, SetpointsCopyProcess
// 
//  CALLS:             NoMultiFrameGoing()
//
//  EXECUTION TIME:     
// 
//------------------------------------------------------------------------------------------------------------

static void CopySetpointsToCassette(CanRxMsg_t const * const pCanRx)

{
 // WE ARE EXPECTING TO RECEIVE 5 BYTES OF DATA WHICH MEANS 54 BITS/FLAGS, ACCOUNTING FOR SETPOINTS GROUPS UP TO 12
 // IF HIGHER GROUPS ARE NEEDED, THIS ROUTINE NEEDS TO BE ADJUSTED ACCORDINGLY.
 // IF NO FLAGS ARE SPECIFIED (DATA SIZE == 0), THEN ALL SET-GROUPS WILL BE COPIED
 // THE DATA FIELD CONTAINS THE MATCH FLAGS WITH THE FOLLOWING ORDER:
 // 1: The Set-Group matches
 // 0: The Set-Group doesn't match. The corresponding SetpointsWriteReady flag will be cleared.
 // Group0SetA_MatchFlag   SetpointsMatch.b0
 // Group0SetB_MatchFlag   SetpointsMatch.b1
 // Group0SetC_MatchFlag   SetpointsMatch.b2
 // Group0SetD_MatchFlag   SetpointsMatch.b3
 // ........
 // Group1SetA_MatchFlag   SetpointsMatch.b4
 // Group1SetB_MatchFlag   SetpointsMatch.b5
 // .........
  
  if ( NoMultiFrameGoing())
   {
      switch(pCanRx->Id.FrameType)
      {
        case PXCAN_SINGLE_FRAME:    
               
          if((SetpointsCopyProcess == 0) || (SetpointsCopyProcess & COPY_PROCESS_DONE))
          { 
            if(pCanRx->DataLen == DATA_BYTES_7)
            {
              SetpointsWriteReady =  (uint64_t)pCanRx->Data[0];
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[1] << (uint64_t)8);
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[2] << (uint64_t)16);
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[3] << (uint64_t)24);
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[4] << (uint64_t)32);
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[5] << (uint64_t)40);
              SetpointsWriteReady |= ((uint64_t)pCanRx->Data[6] << (uint64_t)48);
            }
            else if (pCanRx->DataLen == DATA_BYTES_0)
            {
              SetpointsWriteReady = SETPOINT_WRITE_READY_DEFAULT;     
            }
            
            SetpointsCopyProcess = START_COPY_PROCESS;  
            
            CopytoCassetteCmdReceived = true;
            RxInfo.Id = pCanRx->Id;
          }
                 
        break;   
      } 
   }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CopySetpointsToCassette()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       SetCopyToCassetteFlags()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:         Set Copy-to-Cassette flags 
//
//  MECHANICS:        Set flags to initiate a Copy of Setpoints to cassette.
//
//  CAVEATS:          None
//
//  INPUTS:           None
//
//  OUTPUTS:          None
//
//  ALTERS:           SetpointsWriteReady, SetpointsCopyProcess, Set, Group
// 
//  CALLS:            ResetMultiFrameFlags()
//
//  EXECUTION TIME:   None  
// 
//------------------------------------------------------------------------------------------------------------

void CopyToCassetteFlags(void)
{
  // Just set the flags to default to trigger the copy process
  SetpointsWriteReady = SETPOINT_WRITE_READY_DEFAULT; 
  // Start the process
  SetpointsCopyProcess = START_COPY_PROCESS;  
  Set = SET_A;
  Group = GROUP0;
  
  if (get_MultiFrameProcessTypeFlag() & MULTIFRAME_WRITE)
  {
    ResetMultiFrameFlags();
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CopyToCassetteFlags()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CopySetpointsProcess()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:          Copy Setpoints Process
//
//  MECHANICS:         None
//
//  CAVEATS:           This routines relys on properly setting the SetpointsCopyProcess flags. They define
//                     the states of the "State Machine"
//                     
//  INPUTS:            None
//
//  OUTPUTS:           None
//
//  ALTERS:            Set, Group, SetpointsCopyProcess
//                     
// 
//  CALLS:             getSetpoints(), write_MultiFrameProcessTypeFlag(), prepareMultiFrameTxMsg()
//                     sendMultiFrameWriteCmd(), NoMultiFrameGoing()
//
//  EXECUTION TIME:    None
// 
//------------------------------------------------------------------------------------------------------------

void CopySetpointsProcess(void)

{
  
  uint64_t setpoint_mask = ((uint64_t)1 << (((uint64_t)Group*4) + (uint64_t)Set));

  switch (SetpointsCopyProcess)
  
   {
      case NO_COPY_PROCESS_STARTED:
        // No copy process has been initiated
      break;
      // Start Copy Process
      case START_COPY_PROCESS:

        // If current set/group Copy Process is done or matches, go to next one on next iteration
        if((SetpointsWriteReady & setpoint_mask) && ((SetpointsCopyProcess & COPY_PROCESS_DONE) == 0))
         {
           Set++;
       
           if (Set > SET_D)
            {
             // Go to the next group and reset Set to A
             Group++;
             Set = SET_A; 
            
             switch(Group)
              {
               case GROUP2:
               Group = GROUP4;
               break;
               
               case GROUP7:
               Group = GROUP9;
               break;
               
               case GROUP10:
               Group = GROUP12;
               break;
               
               case GROUP13:
               Group = GROUP0;
               break;
              }
             }
          }
          
        // At this point, the current set/group doesn't match
        // Start Copy Process if a current process has not been initiated
        else if(((SetpointsCopyProcess & COPY_PROCESS_DONE) == 0)  && 
                ((SetpointsWriteReady & setpoint_mask) == 0) && 
                NoMultiFrameGoing())
          { 
            uint8_t tempBuf[PXCAN_MAX_BLOCK_SIZE];
            uint8_t stp_copy = 0;
            
            getSetpoints(Set, Group, tempBuf, &stp_copy);   
                
            prepareMultiFrameTxMsg(tempBuf, SETP_GR_SIZE[Group]);
            
            // Set Command ID
            CanIdUnion_t txCmd = {0};
            txCmd.CmdId = (Group*2) + WRITE_SETPOINTS;
            txCmd.CmdParam = Set;
            txCmd.FrameType = PXCAN_FIRST_FRAME;
            txCmd.TargetAddr = CASSETTE_ADDR;
            // Sent First Frame
            sendMultiFrameWriteCmd(&txCmd);  
            // Set Write process flag
            set_MultiFrameFlag(setpoint_mask);        
            // Clear start flag and set Process on-going flag
            SetpointsCopyProcess = COPY_PROCESS_ONGOING;
            
            write_MultiFrameProcessTypeFlag(MULTIFRAME_WRITE);
       }
      
       if(SetpointsWriteReady == (uint64_t)0xFFFFFFFFFFFFFFFF)
        {
          // Set the CopyProcessDoneFlg 
          SetpointsCopyProcess = COPY_PROCESS_DONE;
        }
      
      break;
      
      // Copy Process Ongoing
      case COPY_PROCESS_ONGOING:
        // Check if set/group copy has been completed
        if ((SetpointsWriteReady & setpoint_mask) && NoMultiFrameGoing())
         {
          // Go back to step 1 to check for next set/group
          SetpointsCopyProcess = START_COPY_PROCESS;   
         }
      break;
      
      // Copy Process Done
      case COPY_PROCESS_DONE:
        
        if (CopytoCassetteCmdReceived)
        {
          CanRxMsg_t const * const pCanRx = &RxInfo;
          // Send success confirmation only.
          PxCan_Send_Async_Response(pCanRx, _NULL, 0);
          CopytoCassetteCmdReceived = false; 
        }
        // Nothing to do
      break;
   }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          CopySetpointsProcess()
//------------------------------------------------------------------------------------------------------------

/*********************************** end of can_tasks.c *********************************/
