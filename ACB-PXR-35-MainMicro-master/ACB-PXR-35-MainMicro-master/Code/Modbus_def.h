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
//  MODULE NAME:        Meter_def.h
//
//  MECHANICS:          This is the definitions file for the Modbus.c module
//
//  TARGET HARDWARE:    PXR35 Rev 4 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   190726  DAH File Creation
//   0.50   220203  DAH - Added Modbus ACK/NAK definitions
//                      - Deleted T1P5 from struct MODB_PORT as it is no longer used
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

enum ModB_Comm_States
{
  MODB_IDLE,
  MODB_RECEIVING,
  MODB_TRANSMITTING
};


// Diagnostics counters definitions
#define MSGCTR                0                       // Message Counter
#define ERRCTR                1                       // Communication Error Counter
#define EXCTR                 2                       // Exception Error Counter
#define SLAVEMSGCTR           3                       // Slave Message Counter
#define NORESPCTR             4                       // No Response Counter
#define OVRRUNCTR             7                       // Overrun Counter

#define MODBUS_RX_BUFSIZE   256                       // Receive buffer size (in bytes)
#define MODBUS_TX_BUFSIZE   256                       // Transmit buffer size (in bytes)

#define MODB_XMIT_TIMEOUT   20
#define MODB_IDLE_TIMEOUT   300


// ACK/NAK Codes
#define NAK_ILLEGAL_FUNCTION    1
#define NAK_ILLEGAL_DATA_ADDR   2
#define NAK_ILLEGAL_DATA_VAL    3
#define NAK_SLAVE_FAILURE       4
#define ACK                     5
#define NAK_DEVICE_BUSY         6
#define NAK_MEMORY_PAR_ERR      8
#define NAK_GATEWAY_UNAVAIL     10
#define NAK_GATEWAY_NO_RESP     11


// These are timer2 initialization constants for different character times
// Used test code to double-check the times on 190610
// Each character is 11 bits, so the ideal times are:
//   3.5 char, 1200 baud - 32.08msec
//   1.5 char, 1200 baud - 13.75msec
//   3.5 char, 4800 baud - 8.02msec
//   1.5 char, 4800 baud - 3.44msec
//   3.5 char, 9600 baud - 4.01msec
//   1.5 char, 9600 baud - 1.72msec
//   3.5 char, 19200 baud - 2.01msec
//   1.5 char, 19200 baud - 860usec
//
#define TIM2_MODB_3P5_1200      31875               // Need to Measure ***MAG
#define TIM2_MODB_1P5_1200      15000               // Need to Measure ***MAG
#define TIM2_MODB_3P5_4800      31875               // Need to Measure ***MAG
#define TIM2_MODB_1P5_4800      15000               // Need to Measure ***MAG
#define TIM2_MODB_3P5_9600      31875               // Measured 4.249msec
#define TIM2_MODB_1P5_9600      15000               // Measured 2.019msec
#define TIM2_MODB_3P5_19200     15938               // Measured 2.129msec
#define TIM2_MODB_1P5_19200      7500               // Measured 1.004msec


// Modbus Map definitions
//   "Start" is the first assigned address
//   "End" is the last assigned address
#define MODB_INPUT_BITS_START       0x03E8          // Start of input bits registers (FC = 2)
#define MODB_INPUT_BITS_END         0x0407          // End of input bits registers (FC = 2)

#define MODB_MAP_ASSIGN_START_1     0x03E8          // Start of mapping assignment registers
#define MODB_MAP_ASSIGN_END_1       0x04AF          // End of mapping assignment registers

#define MODB_MAP_DATA_START_1       0x04B0          // Start of mapping data registers
#define MODB_MAP_DATA_END_1         0x07CF          // End of mapping data registers

#define MODB_CONFIG_START_1         0x07D0          // Start of configuration registers
#define MODB_CONFIG_END_1           0x07D2          // End of configuration registers

#define MODB_TIME1_START            0x0B68          // Start of time registers (old format)
#define MODB_TIME1_END              0x0B6F          // End of time registers (old format)

#define MODB_TIME2_START            0x0B7C          // Start of time registers (new format)
#define MODB_TIME2_END              0x0B80          // End of time registers (new format)

#define MODB_SETPOINTS_START        0x0BB7          // Start of setpoints registers
#define MODB_SETPOINTS_END          0x0D00          // End of setpoints registers

#define MODB_PARAM_START            0x0F9F          // Start of Modbus communications parameters registers
#define MODB_PARAM_END              0x0FA2          // End of Modbus communications parameters registers

#define MODB_RTD1_FLOAT_START       0x1200          // Start of floating-point real-time data section 1
#define MODB_RTD1_FLOAT_END         0x17F1          // End of floating-point real-time data section 1

#define MODB_RTD1_FIXED_START       0x1800          // Start of fixed-point real-time data section 1
#define MODB_RTD1_FIXED_END         0x18F1          // End of fixed-point real-time data section 1

#define MODB_EVENTS_START           0x2000          // Start of events
#define MODB_EVENTS_END             0x25BD          // End of events

#define MODB_MAP_ASSIGN_START_2     0x5000          // Start of mapping assignment regs (same as above)
#define MODB_MAP_ASSIGN_END_2       0x50C7          // End of mapping assignment registers (same as above)

#define MODB_MAP_DATA_START_2       0x5100          // Start of mapping data registers (same as above)
#define MODB_MAP_DATA_END_2         0x541F          // End of mapping data registers (same as above)

#define MODB_RTD2_FLOAT_START       0x6000          // Start of floating-point real-time data section 2
#define MODB_RTD2_FLOAT_END         0x60B9          // End of floating-point real-time data section 2

#define MODB_CONFIG_START_2         0x6300          // Start of configuration registers (same as above)
#define MODB_CONFIG_END_2           0x6302          // End of configuration registers (same as above)

#define MODB_RTD2_FIXED_START       0xC000          // Start of fixed-point real-time data section 2
#define MODB_RTD2_FIXED_END         0xC0B9          // End of fixed-point real-time data section 2




//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct MODB_PORT
{
  uint8_t  CommState;
  uint8_t  RxIabort;
  uint8_t  RxIFrameBreak;
  uint8_t  State;
  uint8_t  Reset_Req;
  uint8_t  RxMsgBuf[MODBUS_RX_BUFSIZE];
  uint8_t  TxMsgBuf[MODBUS_TX_BUFSIZE];
  uint8_t  CharsToTx;
  uint16_t RxMsgNdx;
//  uint16_t T1P5;                        // Count for 1.5 character times (not used)
  uint16_t Comm_Timer;
  uint16_t MsgStat_Counter[8];          // Message diagnostics counters
                                            // [0] - Bus Total Message Count
                                            // [1] - Bus Communications Error Count
                                            // [2] - Slave Exception Error Count
                                            // [3] - Slave Message Count (my messages)
                                            // [4] - Slave No Response Count
                                            // [5] - Slave NAK Count
                                            // [6] - Slave Busy Count
                                            // [7] - Bus Character Overrun Count
};

struct TS60870
{
  uint16_t Year;                            // b15..7: Reserved = 0  b6..0: Year (2000 - 2127)
  uint16_t Month_Day;                       // b15..12: Reserved = 0   b11..8: Month (1=Jan, 12=Dec)
                                            // b7..5: Reserved = 0   b4..0: Day (1-31)
  uint16_t Min_Hr;                          // b15..13: Reserved = 0   b12..8: Hour (0-23)
                                            // b7..6 Reserved = 0   b5..0: Minutes (0-59)
  uint16_t msec;                            // b15..0: milliseconds (0-59,999)
};

