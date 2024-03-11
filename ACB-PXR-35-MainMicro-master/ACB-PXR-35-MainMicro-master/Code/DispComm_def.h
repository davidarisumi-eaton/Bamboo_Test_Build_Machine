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
//  MODULE NAME:        DispComm_def.h
//
//  MECHANICS:          This is the definitions file for the Display.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.16   160818  DAH - Added struct DISPLAYVARS and Status Flags definitions to support basic display
//                        communications
//   0.19   170223  DAH - Added DISP_RXBUFSIZE, DISP_TXBUFSIZE, and RXNDX_INCMASK
//                      - Revised struct DISPLAYVARS because requirements changed due to using DMA instead
//                        of interrupts, and because receptions were added
//                      - Added more Disp.Status flag definitions to handle receptions
//   0.28   181030  DAH - Modified the communications protocol for the planned new high-speed CAM protocol.
//                          - Increased DISP_RXBUFSIZE and DISP_TXBUFSIZE from 64 to 256
//                          - Revised struct DISPLAYVARS to accomodate the new protocol
//                          - Revised  Disp.Status flag definitions to support the new protocol
//                          - Added START_OF_PKT and END_OF_PKT definitions
//   0.36   200210  DAH - Renamed file from Display_def.h to DispComm_def.h
//                      - Renamed struct DISPLAYVARS to struct DISPCOMMVARS
//                      - Renamed const DISP_RXBUFSIZE to const DISPCOMM_RXBUFSIZE
//                      - Renamed const DISP_TXBUFSIZE to const DISPCOMM_TXBUFSIZE
//                      - Added RTD buffers to the display processor communications
//                          - In struct DISPCOMMVARS, renamed RxState (used in AssembleRxPkt()) to
//                            AssRxPktState
//                          - In struct DISPCOMMVARS, added RxState (used in Disp_Rx()), Addr, SeqNum,
//                            RcvReqBitMsk, and AckNak
//                          - In struct DISPCOMMVARS, renamed Status to Flags and revised the definitions
//   0.37   200604  DAH - In struct DISPCOMMVARS, renamed XmitReqBitMsk to XmitReqClrMsk
//                      - In struct DISPCOMMVARS, added RxMsgSav[]
//                      - In struct DISPCOMMVARS, deleted RcvReqBitMsk as it is not used
//                      - Added DEL_MSG_INPROG and GEN_WRITERESP to DPComm.Flags definitions
//                      - Added command definitions, buffer type definitions, and ACK/NAK code definitions
//   0.38   200708  DAH - Added NUM_RTD_BUFFERS definition
//                      - Added DP_CMND_WRWACK, DP_CMND_EXWACK, and DP_CMND_EXWCHK definitions
//                      - In struct DISPCOMMVARS, added RTD_TmrNdx, RTD_XmitTimer[], SeqNumSaved,
//                        TurnaroundTimer, and Check_Status.  Deleted XmitReqClrMsk
//                      - Added CHK_STAT_COMPLETED and CHK_STAT_IN_PROGRESS definitions
//                      - Added DP_BUFTYPE_CHECK definition
//   0.39   200729  DAH - In struct DISPCOMMVARS, renamed TurnaroundTimer to XmitWaitTimer and added
//                        WaitTime
//                      - Commented out definitions WAITING_FOR_WRITERESP and RESPONSE_RECEIVED as these are
//                        not used
//   0.43   210115  DAH - Added support for 61850 communications with the Display Processor
//                          - Added struct DISPCOMM61850VARS, struct DPCOMMTXVARS, and struct DPCOMMRXVARS
//                          - Added DISPCOMM_61850RXBUFSIZE, DISPCOMM_61850TXBUFSIZE, RXNDX_61850INCMASK,
//                            NUM_61850_REQUESTS, and NUM_61850_REQMASK definitions
//                          - Added GOOSE request type definitions (DP61850_TYPE_xxx)
//                          - Added DP_BUFTYPE_GOOSE definition
//   0.44   210324  DAH - Additional support for 61850 communications with the Display Processor
//                          - Added CharCount to struct DPCOMMRXVARS
//                          - Deleted RxSeqNum[] from struct DISPCOMM61850VARS
//   0.55   220420  DAH - Increased transmit buffer size for normal communications (DISPCOMM_TXBUFSIZE) from
//                        256 to 350.  This allows one cycle of a waveform capture (320 bytes) to be
//                        transmitted, along with harmonics Ia thru In (also 320 bytes)
//                      - Changed NUM_RTD_BUFFERS from 1 to 10
//   0.56   220526  DAH - DISPCOMM_TXBUFSIZE increased to 350 for harmonics support
//   0.59   220831  DAH - Added TxDelMsgBuf[] to struct DISPCOMMVARS to hold delayed messages to be
//                        transmitted
//   0.62   221027  DAH - Corrected NUM_RTD_BUFFERS (changed from 10 to 11)
//                      - Increased DISPCOMM_RXBUFSIZE from 256 to 512 to handle largest setpoints buffer
//                      - Changed RxBufNdx from uint8_t to uint16_t
//   0.72   230320  DAH - Added DP_BUFTYPE_FACTORY to support factory test commands
//    25    230403  KT  - Added DP61850_TYPE_CAPT_ENABLE and DP61850_TYPE_CAPT_DISABLE declarations
//                      - Added struct GOOSECAPTURECMD
//                      - In struct DISPCOMM61850VARS, changed Req[] definition from uint8_t to uint16_t
//    40    230531  DAH - Added Event Buffer ID definitions
//    66    230825  DAH - Added DPTxReqFlags definitions
//    69    230828  DAH - Added TX_TIME to DPComm.Flags definitions
//    82    230928  DAH - Added DP_VERREV_BUF_REQ to DPTxReqFlags definitions
//    94    231010  DAH - Added more execute action type definitions
//                      - Added DP_BUFTYPE_DIAG to buffer type definitions
//                      - Deleted ACT_TYPE_xxx constants as they are duplicates of DP_EATYPE_xxx constants
//    99    231019  BP  - Moved Secondary Injection definitions to here and TESTINJ_VARS structure
//   149    240131  DAH - Renamed NUM_RTD_BUFFERS to NUM_RTD_TIMESLICES and set value to 7
//                      - Added display processor port addresses
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
//    Constants
//------------------------------------------------------------------------------------------------------------

// Display Processor Port Addresses
#define DP_DISP_ADD         0x04
#define DP_BT_ADD           0x07
#define DP_USB_ADD          0x08
#define DP_ETH1_ADD         0x09
#define DP_ETH2_ADD         0x0A
#define DP_ETH3_ADD         0x0B
#define DP_ETH4_ADD         0x0C
#define DP_ETH5_ADD         0x0D


// Message definitions
#define START_OF_PKT        0x00            // Start of Packet
#define END_OF_PKT          0x01            // End of Packet

// Command definitions
#define DP_CMND_RDIMM       0x02
#define DP_CMND_RDDEL       0x03
#define DP_CMND_WRRESP      0x04
#define DP_CMND_WRWACK      0x06
#define DP_CMND_EXWACK      0x09
#define DP_CMND_EXWCHK      0x0A
#define DP_CMND_ACK         0x0E

// Buffer type definitions
#define DP_BUFTYPE_RTDATA   3
#define DP_BUFTYPE_DIAG     4
#define DP_BUFTYPE_SETP     5
#define DP_BUFTYPE_EVENT    7
#define DP_BUFTYPE_CHECK    8
#define DP_BUFTYPE_FACTORY  13
#define DP_BUFTYPE_GOOSE    20

// Execute Action type definitions
#define DP_EATYPE_SETP      0
#define DP_EATYPE_RESET     1
#define DP_EATYPE_TU        2
#define DP_EATYPE_CAP       3
#define DP_EATYPE_DIAG      4
#define DP_EATYPE_TIME      5
#define DP_EATYPE_OTHERS    6
#define DP_EATYPE_PWD       7
#define DP_EATYPE_FACTORY   13

#define NUM_RTD_TIMESLICES  7

// Buffer ID definitions
// Events
#define DP_EVENT_SUMMARY        16
#define DP_EVENT_TRIP_SNAP      18
#define DP_EVENT_TESTTRIP_SNAP  19
#define DP_EVENT_ALARM_SNAP     20
#define DP_EVENT_EXTCAP_SNAP    21
#define DP_EVENT_TRIP_SUM       22
#define DP_EVENT_TESTTRIP_SUM   23
#define DP_EVENT_ALARM_SUM      24
#define DP_EVENT_EXTCAP_SUM     25

// Action ID definitions
// EA Type = Time
#define DP_EAID_WRITETIME       1

// ACK/NAK codes
#define DP_ACK              0x00
#define DP_NAK_GENERAL      0x01
#define DP_NAK_BUFINVALID   0x02
#define DP_NAK_BUFTYPEINV   0x03
#define DP_NAK_NOTAVAIL     0x04
#define DP_NAK_NOTSTATE     0x05
#define DP_NAK_DATACRC      0x06
#define DP_NAK_DATARANGE    0x07
#define DP_NAK_NOSESSION    0x08
#define DP_NAK_NOSESINPROG  0x09
#define DP_NAK_INCSESSION   0x0A
#define DP_NAK_BUFOVRFLOW   0x0B
#define DP_NAK_CMDFORMAT    0x0C
#define DP_NAK_CMDINVALID   0x0D

#define DISPCOMM_RXBUFSIZE  512                     // Must be binary number!
#define DISPCOMM_TXBUFSIZE  350
#define RXNDX_INCMASK       (DISPCOMM_RXBUFSIZE - 1)

#define DISPCOMM_61850RXBUFSIZE  128                 // Count must be Multiples of byte
#define DISPCOMM_61850TXBUFSIZE  128
#define RXNDX_61850INCMASK  (DISPCOMM_61850RXBUFSIZE - 1)

// DPComm.Flags flag definitions
#define TX_ACK                  0x01            // Request to transmit an ACK message
#define READ_REQ_RCVD           0x02            // Read request message received
#define DEL_MSG_INPROG          0x04            // Read Delayed message processing is in progress
#define TX_TIME                 0x08            // Transmit set time execute action
//#define RESPONSE_RECEIVED       0x10            // Write response message received
//#define WAITING_FOR_ACK         0x20            // Some process is waiting for an ACK response
#define ACK_RECEIVED            0x40            // ACK response received
#define GEN_WRITERESP           0x80            // Generate a write response message

// DPTxReqFlags: Transmit buffer request flags (for non-periodic buffers)
#define DP_EXTDIAG_BUF_REQ      0x01
#define DP_5MINAVG_BUF_REQ      0x02
#define DP_VERREV_BUF_REQ       0x04
#define DP_TXREQFLAGS_ALL       0x07

// Factory Buffer ID Definitions
#define DP_FAC_BID_DISPFWVER    100
#define DP_FAC_BID_PROTFWVER    99
#define DP_FAC_BID_NV           42
#define DP_FAC_BID_STYLE_2      41
#define DP_FAC_BID_BIO          45
#define DP_FAC_BID_STYLE        22

// DPComm.Check_Status definitions
#define CHK_STAT_COMPLETED      0x00
#define CHK_STAT_IN_PROGRESS    0x01
#define CHK_STAT_FAILED         0x02

// High-Speed (GOOSE) Request Types
//  0 = ACK, 1 = CB Status(CB position, Breaker Failure, ZSI, Power Status, Capture cmd, 
//  CB position control block(upto 8 CBs)), Setpoint Set Selection, 2 = Meter values,
//  3 = Goose Connectivity
#define DP61850_TYPE_ACK                0
#define DP61850_TYPE_GOCB_STATUS_CTRL   1
#define DP61850_TYPE_VALS               2
#define DP61850_TYPE_GO_CON             3

#define NUM_61850_REQUESTS              4
#define NUM_61850_REQMASK               0x0E        // Ack request shouldn't come from Type ACK!


//New EAG Test Commands
#define TEST_DP61850_TYPE_CB_POS_INTER      100 // 100 - CB position is INTERMEDIATE_STATE
#define TEST_DP61850_TYPE_CB_POS_OFF        101 // 101 - CB position is OFF_STATE
#define TEST_DP61850_TYPE_CB_POS_ON         102 // 102 - CB position is ON_STATE
#define TEST_DP61850_TYPE_CB_POS_BAD        103 // 103 - CB position is BAD_STATE

#define TEST_DP61850_TYPE_TRIP_SUCCESS      110 // 110 - CB Trip is Success or Not Tripped
#define TEST_DP61850_TYPE_TRIP_FAIL         111 // 111 - CB Trip Failed during tripping
          
#define TEST_DP61850_TYPE_ZSI_NOPICKUP      120 // 120 - XSI signal for No pickup
#define TEST_DP61850_TYPE_ZSI_PICKUP_STARTS 121 // 121 - XSI signal for pickup Started
          
#define TEST_DP61850_TYPE_PWR_SOURCE_BAD    130 // 130 - Power Source Is bad
#define TEST_DP61850_TYPE_PWR_SOURCE_GOOD   131 // 131 - Power Source Is Good
          
#define TEST_DP61850_TYPE_CAPT_STOP         140 // 140 - Capture Disable
#define TEST_DP61850_TYPE_CAPT_START        141 // 141 - Capture Enable

#define TEST_DP61850_TYPE_CSWI_ALL_OFF      150 // 150 - All 8 CSWI will be OFF
#define TEST_DP61850_TYPE_CSWI_ALL_ON       151 // 151 - All 8 CSWI will be ON   

// Secondary Injection
#define TESTINJ_TYPE_NOTEST     0
#define TESTINJ_TYPE_HWTRIP     1
#define TESTINJ_TYPE_HWNOTRIP   2
#define TESTINJ_TYPE_SWTRIP     3
#define TESTINJ_TYPE_SWNOTRIP   4
#define TESTINJ_TYPE_CANCELHW   5
#define TESTINJ_TYPE_CANCELSW   6

#define TESTINJ_STAT_NONE       0x10     // No test has been conducted since powering up
#define TESTINJ_STAT_INPROG     0x20
#define TESTINJ_STAT_COMPPASS   0x00
#define TESTINJ_STAT_COMPFAIL   0xFF


#define TESTINJ_PHASE_IA        0
#define TESTINJ_PHASE_IB        1
#define TESTINJ_PHASE_IC        2
#define TESTINJ_PHASE_IN        3
#define TESTINJ_PHASE_IG        4

//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct DISPCOMMVARS
{
   uint8_t              RxBuf[DISPCOMM_RXBUFSIZE];
   uint8_t              TxBuf[DISPCOMM_TXBUFSIZE];
   uint8_t              TxDelMsgBuf[DISPCOMM_TXBUFSIZE];
   uint8_t              RxMsg[DISPCOMM_RXBUFSIZE];
   uint8_t              RxMsgSav[13];
   uint8_t              TxState;
   uint8_t              TxSegCharCnt;
   uint8_t              Flags;
   uint16_t             RxBufNdx;
   uint8_t              AssRxPktState;
   uint8_t              RxSegCode;
   uint8_t              RxSegOffset;
   uint8_t              RxState;
   uint8_t              Addr;
   uint8_t              SeqNum;
   uint8_t              SeqNumSaved;
   uint8_t              AckNak;
   uint8_t              Check_Status;
   uint8_t              RTD_TmrNdx;
   uint16_t             RTD_XmitTimer[NUM_RTD_TIMESLICES];
   uint16_t             XmitWaitTimer;
   uint16_t             WaitTime;
   uint16_t             RxCharsLeft;
   uint16_t             RxMsgNdx;
   uint16_t             RxCRC;
   uint16_t             TxSegNdx;
   uint16_t             TxNdx;
   uint16_t             TxCRC;
};

struct DPCOMMTXVARS
{
   uint16_t             TxNdx;
   uint16_t             TxSegNdx;
   uint16_t             TxCRC;
   uint8_t              TxSegCharCnt;
   uint8_t              TxBuf[DISPCOMM_61850TXBUFSIZE];
};

struct DPCOMMRXVARS
{
   uint16_t             RxCharsLeft;
   uint16_t             RxMsgNdx;
   uint16_t             RxCRC;
   uint8_t              RxBufNdx;
   uint8_t              AssRxPktState;
   uint8_t              RxSegCode;
   uint8_t              RxSegOffset;
   uint8_t              CharCount;
   uint8_t              RxMsg[DISPCOMM_61850RXBUFSIZE];
   uint8_t              RxBuf[DISPCOMM_61850RXBUFSIZE];
};

struct DISPCOMM61850VARS
{
   uint8_t              TxState;
   uint16_t             Req[NUM_61850_REQUESTS];
   uint8_t              SeqNum;
   uint8_t              TxSeqNum[NUM_61850_REQUESTS];
   uint16_t             XmitWaitTimer[NUM_61850_REQUESTS];
   struct DPCOMMTXVARS  TxVars;
   struct DPCOMMRXVARS  RxVars;
};

/**
 * @brief Enum data for Double point control/status
 */
typedef enum
{
    INTERMEDIATE_STATE = 0,
    OFF = 1,
    ON = 2,
    BAD_STATE = 3
} enum_dp_t;

/**
 * @brief Enum data for ATS Device list
 */
typedef enum
{
    CB_NONE = 0,
    //MM, MTM1, MTM2 Circuit breakers
    CB_T1 = 1,
    CB_M1 = 2,
    CB_M2 = 3,
    // Microgrid Circuit breakers
    CB_G1 = 1,
    CB_G2,
    CB_G3,
    CB_G4,
    CB_G5,
    CB_G6,
    CB_G7,
    CB_G8,
    // Subscribed or Downstream Circuit breakers
    CB_1 = 1,
    CB_2,
    CB_3,
    CB_4,
    CB_5,
    CB_6,
    CB_7,
    CB_8,
    CB_9,
    CB_10,
    CB_11,
    CB_12,
    CB_13,
    CB_14,
    CB_15,
    CB_16,
    CB_17,
    CB_18,
    CB_19,
    CB_20,
    CB_21,
    CB_22,
    CB_23,
    CB_24,
    CB_25,
    CB_26,
    CB_27,
    CB_28,
    CB_29,
    CB_30,
    CB_31,
    CB_32
} enum_cblist_t;

/**
 * @brief Enum data for Fault List bit Assignment
 */
typedef enum
{
    NO_FAULT = 0,
    HIGH_LOAD_1 = 1,
    HIGH_LOAD_2,
    SD_PICKUP_FAULT,
    LD_PICKUP_FAULT,
    INST_PICKUP_FAULT,
    OVERVOLTAGE,
    UNDERVOLTAGE,
    MODBUS_TCP,
    MODBUS_RTU,
    GOOSE_CMD,
    BLUETOOTH,
} enum_Fault_list_t;

/**
 * @brief Enum data for Setpoint Set
 */
typedef enum
{
    SP_SET1 = 1,
    SP_SET2 = 2,
    SP_SET3 = 3,
    SP_SET4 = 4
} enum_setpointset_t;

#define DEFAULT_SETPOINT_SET    SP_SET1

#define MAX_NUM_ATS_CB_CNT      CB_G8
#define MAX_NUM_CB_CNT          CB_32
#define MAX_GOOSE_SUB           PXR35_LD_MAX_COUNT //Defualt Maximum Subscriber can be handler per PXR35 Trip Unit (This can be configured form SCD/CID files)

#define LD_NAME_SIZE_MAX_ED2    64 // As per the IEC61850-7.2 Ed2 Logical node name maximum size

/**
 * @brief Enum data for ATS Application types
 */
typedef enum
{
    ATSAPP_NONE = 0,    // None
    ATSAPP_MM,          // MM1 - Main to main 1
    ATSAPP_MTM1,        // MTM1 - Main tie main1
    ATSAPP_MTM2,        // MTM2 - Main tie main2
    ATSAPP_MG           // Microgrid - max 8 CBs
} enum_xferapp_t;

/******* In below Packet format, Device_ID is only for subscribtion perpose. For publishing it is don't care variable  ******/
// Status and control dataset
struct STATUSCTRL_CB
{
    uint16_t    Device_ID;
    uint8_t     CB_Pos_Status;
    uint8_t     Trip_Failure;
    uint8_t     ZSI_Status;
    uint8_t     Capture_State;
    uint8_t     SPSet_Select;
    uint8_t     Source_Status;
    uint8_t     CTRL_CB1_Open_Op;   //Tie/Main CB1 Open/Close Operation
    uint8_t     CTRL_CB1_Close_Op;
    uint8_t     CTRL_CB2_Open_Op;   //Main CB2 Open/Close Operation
    uint8_t     CTRL_CB2_Close_Op;
    uint8_t     CTRL_CB3_Open_Op;   //Main CB3 Open/Close Operation
    uint8_t     CTRL_CB3_Close_Op;
    uint8_t     CTRL_CB4_Open_Op;   //Main CB4 Open/Close Operation
    uint8_t     CTRL_CB4_Close_Op;
    uint8_t     CTRL_CB5_Open_Op;   //Main CB5 Open/Close Operation
    uint8_t     CTRL_CB5_Close_Op;
    uint8_t     CTRL_CB6_Open_Op;   //Main CB6 Open/Close Operation
    uint8_t     CTRL_CB6_Close_Op;
    uint8_t     CTRL_CB7_Open_Op;   //Main CB7 Open/Close Operation
    uint8_t     CTRL_CB7_Close_Op;
    uint8_t     CTRL_CB8_Open_Op;   //Main CB8 Open/Close Operation
    uint8_t     CTRL_CB8_Close_Op;  
};

// Meter value dataset
struct METERVALUE_CB
{
    uint16_t    Device_ID;
    uint32_t    Meter_Readings[13]; // 0 = TotW.mag.f, 1 = TotVAr.mag.f, 2 = TotVA.mag.f, 3 = TotPF.mag.f, 4 = Hz.mag.f, 
                                    // 5 = PhV.phsA.cVal.mag.f, 6 = PhV.phsB.cVal.mag.f, 7 = PhV.phsC.cVal.mag.f,
                                    // 8 = PhV.neut.cVal.mag.f, 9 = A.phsA.cVal.mag.f, 10 = A.phsB.cVal.mag.f,
                                    // 11 = A.phsC.cVal.mag.f, 12 = A.neut.cVal.mag.f
};

// Status and Control Susbscription structure
struct SUB_CBSTATUSCTRL
{
    struct STATUSCTRL_CB CB_Status_Ctrl;
    union {
        unsigned char Subscriptions;// What are all features are subscribed by this set's Device ID
                                    // (bit 0 -> ZSI, 1 -> Trip_Failure,
                                    // 2 -> Global_Capture, 3 -> ATS, 4 -> SPSET_SEL
        struct {
            unsigned char ZSI:1;
            unsigned char Trip_Failure:1;
            unsigned char Global_Capture:1;
            unsigned char ATS:1;
            unsigned char SPSET_SEL:1;
        };
    };
    uint8_t Goose_Connection;
};

// Status, Control & Meter value Publish structure
struct PUB_GOOSE_CBS
{
    struct STATUSCTRL_CB CB_Status_Ctrl;
    struct METERVALUE_CB CB_Meter_values;
};

struct TESTINJ_VARS
{
  uint8_t Type;
  uint8_t Status;
  uint8_t Phase;            // Ia, Ib, Ic, In?, Ig?
  uint32_t Current;         // Can use for both input and result
  uint32_t TestTime;        // In msec
  uint32_t PriSecCos;
};  

struct PASSWORD
{
    uint8_t admin_num[4];
    uint8_t user_num[4];
    uint8_t user_info;    // b0: User exists
                          // b1: User has extended rights
    uint8_t reserved;
    uint16_t Cksum;
    uint16_t Cksum_not;
};
#define PASSWORD_SIZE   sizeof(struct PASSWORD)
union PASSWORD_DEF
{
    uint8_t             buf[PASSWORD_SIZE];
    struct  PASSWORD    data;
};


