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
//  MODULE NAME:        Iod_def.h
//
//  MECHANICS:          This is the definitions file for the Iod.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.04   150928  DAH Code development
//   0.05   151021  DAH - Added FRAM interface definitions
//   0.06   151110  DAH - Added RTC and Flash interface definitions
//                      - Added SPI2 request flags and SPI2 structure definition
//                  BP  - Added TRIP_LED_TOGGLE for debugging
//                      - Added Analog MUX interface definition for reading temperature
//   0.07   151216  DAH - Added Frame FRAM interface definitions
//                      - Added NUM_SPI2_REQUESTS (moved from Iod.c)
//                      - Added EEPOT constants and structure
//                      - Added display interface definitions
//   0.08   160122  DAH - Added Modbus interface definitions
//   0.09   160224  DAH - Changed TRIP_LED_x definitions from GPIOA-8 to GPIOG-11 for rev 2 hardware
//                      - Renamed FRAM_CSN_ACTIVE to FRAM2_CSN_ACTIVE and renamed FRAM_CSN_INACTIVE to
//                        FRAM2_CSN_INACTIVE for rev 2 hardware
//                      - Renamed DEV_ONBOARD to DEV_FRAM2
//                      - Added FRAM1_CSN_ACTIVE and FRAM1_CSN_INACTIVE
//                      - Changed Modbus Enable definitions from GPIOH-7 to GPIOG-14 for rev 2 hardware
//                      - Added MODBUS_PWR_ENABLE and MODBUS_PWR_DISABLE
//                      - Renamed the test injection control signals
//                      - Deleted all EEPOT constants and definitions, as the EEPOT has been eliminated from
//                        rev 2 boards
//                      - Added SPI2_THSENSOR_READ,  THSSENSOR_CSN_ACTIVE, THSSENSOR_CSN_INACTIVE, and
//                        struct TH_SENSOR to support the HIH6030 temperature/humidity sensor
//                      - Added TRIP_LED_CLR_xx and TRIP_LED_STROBE_xx
//                      - Added Cause of Trip LED definitions
//                      - Added ADC Multiplexer Control and TA Trip definitions
//   0.10   160310  DAH - Added system flag definitions
//                      - Corrected NUM_SPI2_REQUESTS (changed from 6 to 5)
//   0.12   160502  DAH - Renamed VAL200msec to VAL200MSEC
//          160502  BP  - Added spare test pins A3 (PE1) and D1 (PC13) for testing.
//   0.13   160512  DAH - Added SPI2_ENERGY_WRBDFRAM to support writing energy to on-board FRAM
//   0.14   160620  DAH - Re-added support for writing Frame EEPOT settings
//                          - Added SPI2_RD_EEPOT AND SPI2_WR_EEPOT
//                          - Added struct Frame_EEPOT
//                          - Added EEPOT_CSN_ACTIVE and EEPOT_CSN_INACTIVE
//                          - EEPOT command definitions added
//                      - Added support for Test Injection calibration
//                          - Added SPI2_INJ_WRITEFRAM and SPI2_INJ_WRITEFLASH
//                          - Added INJ_FRAM_ERR and INJ_FLASH_ERR
//                          - Added TSTINJ_SEL_GND and TSTINJ_SEL_PH
//   0.15   160718  DAH - Added support for TLA command
//                          - Added ALARM_LED_OFF and ALARM_LED_ON definitions
//                      - Added struct SWITCHES_PUSHBUTTONS and to support setting the Cause of Trip LEDs
//                        and reading the breaker status 
//                      - Added COT_RESET_HIGH to support reading the Clear LEDs pushbutton
//                      - Added BREAKER_CLOSED to support reading the Breaker Closed switch
//                      - Added support for waveform sample storage in the Black Box FRAM
//                          - Added FM25CL64 FRAM Instructions (moved from Iod.c)
//   0.16   160818  DAH - Added support for Startup Time measurement
//                          - Added ST_DISCHARGE_OFF, ST_DISCHARGE_ON, struct STARTUP_TIME_VARS
//                          - Added SysTick Definitions (moved from Init.c)
//                          - Added GET_STARTUPTIME to System Flags
//                      - Added ADC3 channel definitions to support moving ADC3 conversions from the
//                        DMA2_Stream0_IRQHandler() (interrupt) to the foreground
//   0.18   161115  DAH - Deleted TRIP_LED_CLR_OFF and TRIP_LED_CLR_ON definitions as the LED circuit was
//                        modified on the rev 3 boards
//                      - Deleted ADC3_USBPWR_CH definition as the USB was moved to the display processor
//                      - Deleted SPI2_THSENSOR_READ as the Temperature/Humidity sensor was moved to the I2C
//                      - Renamed THSSENSOR_CSN_ACTIVE and THSSENSOR_CSN_INACTIVE to HIGHLOAD_LED_ON and
//                        HIGHLOAD_LED_OFF as the Temperature/Humidity sensor was moved to the I2C bus and a
//                        high load LED indicator was added to the rev 3 boards
//                      - Added TH_MEM_CHARGE_EN and TH_MEM_CHARGE_DIS for thermal memory support on rev 3
//                        boards
//                      - Revised MODBUS_PWR_ENABLE and MODBUS_PWR_DISABLE definitions to support the new
//                        rev 3 board Modbus power circuit
//                      - Renamed MODBUS_ENABLE and MODBUS_DISABLE to MODBUS_TXEN_RXDIS and
//                        MODBUS_TXDIS_RXEN to match the rev 3 board
//                      - Added Health Bus definitions and structure (HLTH_I2C_VARS)
//                      - Changed temperature and humidity data types from uint16_t to float in struct
//                        TH_SENSOR
//                      - Renamed PI6 to TRIP_LED_STROBE_LOW and TRIP_LED_STROBE_HIGH.  Changed the
//                        definitions from controlling the output (the ODR register) to controlling the Mode
//                        register.  For low, make mode input without pull-down (there is an external
//                        pull-down on the board).  For high, switch the mode to output (the ODR is
//                        initialized for high output).  Want normal state to be input so that when reset
//                        button is pushed, the battery will not be drained through the port line
//   0.19   170223  DAH - Added auxiliary relay definitions
//                      - Deleted MODBUS_TXDIS_RXEN and MODBUS_TXDIS_RXEN as the Modbus was moved to the
//                        display processor
//                      - Revised Cause of Trip strobe definitions
//   0.22   180420  DAH - Added support for internal real time and time-stamping
//                          - Added structure definitions SYSTICK_TIME and INTERNAL_TIME
//                          - Added RTC_ERR to System Flag (SystemFlags) definitions
//                      - Revised for Flash support (energy and demand logging)
//                          - Deleted FLASH_SECTOR_1 definition (moved to Flash_def.h)
//                          - Added SSt26VF064B Flash Instructions (moved from Iod.c since used globally)
//   0.23   180504  DAH - Added FLASH_IN_USE definition to SystemFlags to support arbitrating between Flash
//                        reads and writes
//   0.24   180517  DAH - Deleted SPI2 request flags as they are no longer used
//                      - Deleted FLASH_IN_USE flag from SystemFlags (moved to FRAM_Flash_def.h)
//                      - Deleted struct SPI2VARS
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Added struct SPI2VARS and added Ack double word to the structure
//                          - Added SPI2 non-Flash request and acknowledge flags back in
//                          - Added SPI2 Flash access flag definitions
//                      - Added EVENT_FF_FULL to System Flag (SystemFlags) definitions
//                      - Added support for Alarm waveform capture events
//                          - Added FLASH_BE for Flash block erases
//                          - Added ALARM_WF_FRAM_ERR to System Flag (SystemFlags) definitions
//   0.26   180627  DAH - In the System Flag (SystemFlags) definitions, renamed GET_STARTUPTIME to
//                        FIRST_PASS
//                      - Added S2NF_ENERGY_RD to the SPI2 Non-Flash Access Flags definitions
//                      - Added S2F_CHK_CAL to the SPI2 Flash Access Flags definitions
//   0.27   181001  DAH - Added support for Trip waveform captures
//                          - Added SPI2 Flash access flags for trip waveforms (S2F_TRIP_WF_xxx)
//                          - Added TRIP_WF_FRAM_ERR to System Flag (SystemFlags) definitions
//                      - Changed the priority of the Flash requests for waveforms so that erase operations
//                        are higher than write operations.  The blocks must be erased before a new waveform
//                        is written, so the erases should be a higher priority.
//                          - S2F_NEXT_STATE[ ] revised
//                      - Added support for Strip-Chart waveform captures
//                          - Added SPI2 Flash access flags for strip-chart waveforms (S2F_CHART_WF_xxx)
//                          - Added CHART_WF_FRAM_ERR to System Flag (SystemFlags) definitions
//   0.29   190122  DAH - Added ADC3_COILTEMP_CH definition to support coil temperature measurement
//                      - Added coil temperature multiplexer control definitions (COIL_TEMP_xx)
//   0.32   190726  DAH - Added S2NF_IDMND_WR and S2NF_PDMND_WR definitions to support min and max demand
//                        current and power storage
//   0.34   191002  DAH - Revised pin definitions for rev 4 boards:
//                          - FRAM1_CSN_ACTIVE and FRAM1_CSN_INACTIVE changed from PE2 to PE7
//                          - RELAY_DR2 changed from PH13 to PH10
//                          - Revised coil temperature controls (COIL_TEMP_x_ON, COIL_TEMP_OFF) because
//                            multiplexer enable is now active LOW, not active HIGH
//                          - Added TSTINJ_GND_ON and TSTINJ_GND_OFF
//                          - Deleted TSTINJ_SEL_GND and TSTINJ_DEL_PH.  These are no longer used
//                          - Added TSTINJ_EN and TSTINJ_DIS
//                          - Added CAM1 and CAM2 controls
//  0.35    191118  DAH - Added CAM2_UART_MODE and CAM2_OUTPUT_MODE definitions
//                      - Added GF_USING_ROGOWSKI and IN_USING_ROGOWSKI definitions
//  0.37    200604  DAH - Added support for reading waveforms for display processor communications
//                          - Added S2F_RD_WF to Flash access flags
//                          - Added S2NF_DP_REQ to FRAM access flags
//  0.38    200708  DAH - Added support for writing setpoints (as a test)
//                          - Added S2F_SETP_WR1 and S2F_SETP_WR2 to Flash access flags
//  0.42    201202  DAH - Deleted S2NF_RTC_RD and S2NF_RTC_WR as the external RTC is no longer used
//                      - Deleted RTC and Internal Time constants and definitions (moved to RealTime_def.h)
//  0.45    210504  DAH - Changes for H743 board revision
//                          - Swapped STATUS_LED_TOGGLE_RED and STATUS_LED_TOGGLE_GRN definitions
//                          - Changed ALARM_LED_xxx definitions from PH9 to PA11
//                      - Added S2F_INJ_RD to SPI2 Flash Access Flags
//   0.50   220203  DAH - Added TemperatureMax to struct TH_SENSOR
//   0.53   220325  DAH - Added TemperatureMaxTS, TemperatureMaxInt, TemperatureMaxIntTS to struct TH_SENSOR
//   0.58   220829  DAH - Fixed ST_DISCHARGE_ON and ST_DISCHARGE_OFF definitions
//   0.59   220831  DAH - Deleted SPI2 Non-Flash Access Flag defs (S2NF_xx) as they are no longer used
//                      - Deleted S2F_CLEAR_EVENTS from SPI2 Flash Access Flag defs as it is no longer used
//   0.60   220928  DAH - Added FRAME_FRAM_ERR to SystemFlags definitions
//   0.62   221027  DAH - Deleted S2F_SETP_WR1 and S2F_SETP_WR2 as they are no longer used
//   0.67   221209  DAH - Renamed S2F_xx flags to S1F_xx flags as the serial Flash is now on SPI1
//                      - Renamed struct SPI2VARS to SPI1VARS
//                      - Changed Alarm LED definitions for the new hardware (was PA11, now is PA9)
//                      - Added digital I/O definitions (DIO1 - DIO4)
//                      - Added protection availability LED definitions (L_LED_xx, S_LED_xx, I_LED_xx,
//                        G_LED_xx)
//   0.69   230220  DAH - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - CHART_WF_FRAM_ERR replaced with EXTCAP_WF_FRAM_ERR
//                          - Strip-chart flags replaced with extended-capture flags in SPI1 Access Flags
//                            (S1F_EXT_xx)
//   0.72   230320  DAH - Changed neutral sense control from PC9 to PD14
//                  BP  - Added High Load LED toggle and blink rate setting
//   54     230802  BP  - Fixed definition for ADC3_BATSENSE_CH
//                      - Added Battery Sense enable/disable
//                      - Added ZSI definitions
//   69     230828  DAH - Added time sync pin control definitions (TIME_SYNC_xx)
//   108    231108  DAH - Deleted duplicate definitions of TRIP_LED_STROBE_xxx
//                      - Deleted BITxx definitions.  These are defined in Flags_def.h and caused compiler
//                        warnings
//   122    231204  BP  - Added definitions for controlling the Rogo and GF selection pins
//   133    231219  DAH - Added ONE_CYC_VALS_DONE to system flag definitions
//   142    240119  DAH - Added S1F_AFECAL_RD1, S1F_ADCHCAL_RD1, and S1F_ADCLCAL_RD1 definitions to SPI1
//                        access flags
//                      - Added SKIP_LOOPTIME_MEAS to System Flag (SystemFlags) definitions
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
//#define ENABLE_GOOSE_COMM_SPEED_TEST // This enable the Tracepoint Test pins for measuring the Goose Comm Time.
//#define ENABLE_GOOSE_COMM_AUTOSEND // This enables the auto send(Same as EAG77 command)

//   Note: For BSRR registers, BSRRH (bits 16-31 of BSRR register) resets the corresponding ODR bits
//     BSRRL (bits 0-15 of BSRR register) sets the corresponding ODR bits
// Status LED
#define STATUS_OK_RATE          50          // Number of 10msec ticks for 1/2-sec on, 1/2-sec off blink
#define STATUS_ERR_RATE         13          // Number of 10msec ticks for 1/8-sec on, 1/8-sec off blink

#define STATUS_LED_OFF                  GPIOI->BSRRH = 0x0003;
#define STATUS_LED_ON_RED               GPIOI->BSRRL = 0x0001;
#define STATUS_LED_ON_GRN               GPIOI->BSRRL = 0x0002;

#define STATUS_LED_TOGGLE_RED           GPIOI->BSRRH = 0x0002;   \
                                        GPIOI->ODR ^= 0x0001;
#define STATUS_LED_TOGGLE_GRN           GPIOI->BSRRH = 0x0001;   \
                                        GPIOI->ODR ^= 0x0002;

// LDPU LED
#define LDPU_LED_OFF                    GPIOI->BSRRH = 0x0080;
#define LDPU_LED_ON                     GPIOI->BSRRL = 0x0080;

// TRIP LED
#define TRIP_LED_OFF                    GPIOG->BSRRH = 0x0800;
#define TRIP_LED_ON                     GPIOG->BSRRL = 0x0800;
#define TRIP_LED_TOGGLE                 GPIOG->ODR ^= 0x0800;

// ALARM LED
#define ALARM_LED_OFF                   GPIOA->BSRRH = 0x0200;
#define ALARM_LED_ON                    GPIOA->BSRRL = 0x0200;

// HIGHLOAD LED
#define HIGHLOAD_BLINK_RATE         13          // Number of 10msec ticks for 1/8-sec on, 1/8-sec off blink                                        
                                        
#define HIGHLOAD_LED_OFF                GPIOG->BSRRH = 0x0001;
#define HIGHLOAD_LED_ON                 GPIOG->BSRRL = 0x0001;
#define HIGHLOAD_LED_TOGGLE             GPIOG->ODR ^= 0x0001;                                        

// Cause of Trip LEDs strobe (TRIP_LED_STROBE) 
#define TRIP_LED_STROBE_OFF             GPIOI->MODER &= 0xFFFFCFFF;
#define TRIP_LED_STROBE_ON              GPIOI->MODER |= 0x00001000;
#define TRIP_LED_STROBE_LOW             GPIOI->BSRRH = 0x0040;
#define TRIP_LED_STROBE_HIGH            GPIOI->BSRRL = 0x0040;

// Cause of Trip LEDs
#define TRIP_LEDS_OFF                   GPIOI->BSRRH = 0x0F00;          // PI11, 10, 9, 8 = 0
#define SD_LED_ON                       GPIOI->BSRRH = 0x0E00;   \
                                        GPIOI->BSRRL = 0x0100;          // PI11, 10, 9 = 0  PI8 = 1
#define GND_HL_LED_ON                   GPIOI->BSRRH = 0x0D00;   \
                                        GPIOI->BSRRL = 0x0200;          // PI11, 10, 8 = 0  PI9 = 1
#define INST_LED_ON                     GPIOI->BSRRH = 0x0B00;   \
                                        GPIOI->BSRRL = 0x0400;          // PI11, 9, 8 = 0  PI10 = 1
#define LONG_LED_ON                     GPIOI->BSRRH = 0x0700;   \
                                        GPIOI->BSRRL = 0x0800;          // PI10, 9, 8 = 0  PI11 = 1

// Cause of Trip LEDs Pushbutton Input (SW_SYS_RST)
#define COT_RESET_HIGH                  ((GPIOG->IDR & 0x0100) == 0x0100)   // Button pressed

// Breaker Closed Input (uC_BREAKER_CLOSED)
#define BREAKER_CLOSED                  ((GPIOG->IDR & 0x0080) == 0x0080)   // Breaker closed
                                        
// Maintenance Mode Switch Input (uC_MM). This includes both the switch and the external input
#define MM_SWITCH_VALUE                    ((GPIOB->IDR & 0x1000) == 0x1000)   // Maintenance Mode switch (0 = MM switch on, 1 = MM switch off)                                        
                                        
// ZSI Input and Output                                       
#define ZIN                             ((GPIOG->IDR & 0x0008) == 0x0008)   // ZSI input high                                      
#define SET_ZOUT                        GPIOG->BSRRH = 0x1000;
#define CLEAR_ZOUT                      GPIOG->BSRRL = 0x1000;                                      

// AFE Interface
#define AFE_START_HIGH                  GPIOE->BSRRL = 0x0001;
#define AFE_START_LOW                   GPIOE->BSRRH = 0x0001;

#define AFE_RESETN_INACTIVE             GPIOF->BSRRL = 0x0004;
#define AFE_RESETN_ACTIVE               GPIOF->BSRRH = 0x0004;

#define AFE_CSN_INACTIVE                GPIOG->BSRRL = 0x0020;
#define AFE_CSN_ACTIVE                  GPIOG->BSRRH = 0x0020;


// Battery Sense Enable
#define BATTERY_SENSE_EN                GPIOG->BSRRL = 0x0010;              // Hi = enable (turns on FET)
#define BATTERY_SENSE_DIS               GPIOG->BSRRH = 0x0010;                                        

// Ground Fault and Neutral Sensing Control (These port pins control the measurement circuitry on the board)
#define GF_USING_ROGOWSKI               ((GPIOD->ODR & 0x0001) == 0x0001)   // PD0 (GF_CT_EN) = 1: Rogowski
#define IN_USING_ROGOWSKI               ((GPIOD->ODR & 0x4000) == 0x4000)   // PD14 (N_CT_EN) = 1: Rogowski

// These will determined by Setpoints0.stp.Neutral_Sensor                                        
#define IN_ROGO_ACTIVE                  GPIOD->BSRRL = 0x4000;              // Hi = Rogo 
#define IN_ROGO_INACTIVE                GPIOD->BSRRH = 0x4000;              // Lo = CT
                                        
// These will determined by Setpoints0.stp.SG_Sensor                                        
#define GF_ROGO_ACTIVE                  GPIOD->BSRRL = 0x0001;              // Hi = Rogo 
#define GF_ROGO_INACTIVE                GPIOD->BSRRH = 0x0001;              // Lo = CT
                                        
                                         

// FRAM1 (Extended Capture FRAM) Interface
#define FRAM1_CSN_INACTIVE              GPIOE->BSRRL = 0x0080;
#define FRAM1_CSN_ACTIVE                GPIOE->BSRRH = 0x0080;

// FRAM2 Interface
#define FRAM2_CSN_INACTIVE              GPIOH->BSRRL = 0x0004;
#define FRAM2_CSN_ACTIVE                GPIOH->BSRRH = 0x0004;

// RTC Interface
#define RTC_CSN_INACTIVE                GPIOH->BSRRL = 0x0008;
#define RTC_CSN_ACTIVE                  GPIOH->BSRRH = 0x0008;

// FLASH Interface
#define FLASH_CSN_INACTIVE              GPIOG->BSRRL = 0x0040;
#define FLASH_CSN_ACTIVE                GPIOG->BSRRH = 0x0040;

// Frame FRAM Interface
#define FRAME_CSN_INACTIVE              GPIOI->BSRRL = 0x0010;
#define FRAME_CSN_ACTIVE                GPIOI->BSRRH = 0x0010;

// Frame EEPOT Interface
#define EEPOT_CSN_INACTIVE              GPIOI->BSRRL = 0x0008;
#define EEPOT_CSN_ACTIVE                GPIOI->BSRRH = 0x0008;

// Display Interface
#define DISPLAY_ENABLE                  GPIOF->BSRRL = 0x0002;
#define DISPLAY_DISABLE                 GPIOF->BSRRH = 0x0002;

// Modbus Interface
#define MODBUS_PWR_ENABLE               GPIOH->BSRRL = 0x0800;
#define MODBUS_PWR_DISABLE              GPIOH->BSRRH = 0x0800;

// Termal Memory Interface
#define TH_MEM_CHARGE_EN                GPIOG->BSRRH = 0x2000
#define TH_MEM_CHARGE_DIS               GPIOG->BSRRL = 0x2000



// Test Injection Controls
#define TSTINJ_PHA_OFF                  GPIOF->BSRRL = 0x0800;          // SELx_y=1: STG3692 Dx=xS1, Dy=yS1
#define TSTINJ_PHA_ON                   GPIOF->BSRRH = 0x0800;          // SELx_y=0: STG3692 Dx=xS2, Dy=yS2                    

#define TSTINJ_PHB_OFF                  GPIOF->BSRRL = 0x1000;
#define TSTINJ_PHB_ON                   GPIOF->BSRRH = 0x1000;

#define TSTINJ_PHC_OFF                  GPIOF->BSRRL = 0x2000;
#define TSTINJ_PHC_ON                   GPIOF->BSRRH = 0x2000;

#define TSTINJ_PHN_OFF                  GPIOF->BSRRL = 0x4000;
#define TSTINJ_PHN_ON                   GPIOF->BSRRH = 0x4000;

#define TSTINJ_GND_OFF                  GPIOE->BSRRH = 0x0100;          // ADG782 INx = 0: switch open
#define TSTINJ_GND_ON                   GPIOE->BSRRL = 0x0100;          // ADG782 INx = 1: switch closed

// Test injection enable: turn on injection bypass enable (PE13 = 1) and test injection enable (PI5 = 1)
#define TSTINJ_EN                       GPIOE->BSRRL = 0x2000;      \
                                        GPIOI->BSRRL = 0x0020;

// Test injection disable: turn off injection bypass enable (PE13 = 0) and test injection enable (PI5 = 0)
#define TSTINJ_DIS                      GPIOE->BSRRH = 0x2000;      \
                                        GPIOI->BSRRH = 0x0020;


// ADC Multiplexer Control
#define LOW_GAIN_INPUTS                 GPIOC->BSRRL = 0x0100;
#define HIGH_GAIN_INPUTS                GPIOC->BSRRH = 0x0100;
#define USING_HIGH_GAIN                 ((GPIOC->ODR & 0x0100) == 0x0000)

// TA Trip
#define TA_TRIP_ACTIVE                  GPIOE->BSRRL = 0x8000;
#define TA_TRIP_INACTIVE                GPIOE->BSRRH = 0x8000;

// Startup Time Circuit
                                        // Make open-drain input
#define ST_DISCHARGE_OFF                GPIOG->OTYPER |= 0x00000400; \
                                        GPIOG->MODER &= 0xFFEFFFFF;
                                        // Make open-drain, clear output bit, make pin output
#define ST_DISCHARGE_ON                 GPIOG->OTYPER |= 0x00000400; \
                                        GPIOG->BSRRH = 0x0400;      \
                                        GPIOG->MODER |= 0x00100000;

// CAM1 and CAM2 controls
#define CAM1_RCVR_ENABLED               GPIOD->BSRRH = 0x1000;
#define CAM1_RCVR_DISABLED              GPIOD->BSRRL = 0x1000;

#define CAM1_DRVR_ENABLED               GPIOD->BSRRL = 0x2000;
#define CAM1_DRVR_DISABLED              GPIOD->BSRRH = 0x2000;

#define CAM2_RCVR_ENABLED               GPIOD->BSRRH = 0x0400;
#define CAM2_RCVR_DISABLED              GPIOD->BSRRL = 0x0400;

#define CAM2_DRVR_ENABLED               GPIOD->BSRRL = 0x0800;
#define CAM2_DRVR_DISABLED              GPIOD->BSRRH = 0x0800;

#define CAM2_UART_MODE                  GPIOC->MODER &= 0xFFFF0FFF; \
                                        GPIOC->MODER |= 0x0000A000;
// Used if 5V_AUX (Modbus power) is turned off.  Note, the ODR for the Tx and Rx pins is already low
#define CAM2_OUTPUT_MODE                GPIOC->MODER &= 0xFFFF0FFF; \
                                        GPIOC->MODER |= 0x00005000;


// Auxiliary Relays
#define RELAY1_OPEN                     GPIOH->BSRRH = 0x1000;
#define RELAY1_CLOSE                    GPIOH->BSRRL = 0x1000;

#define RELAY2_OPEN                     GPIOH->BSRRH = 0x0400;
#define RELAY2_CLOSE                    GPIOH->BSRRL = 0x0400;

#define RELAY3_OPEN                     GPIOH->BSRRH = 0x4000;
#define RELAY3_CLOSE                    GPIOH->BSRRL = 0x4000;

#define RELAY1_NUM      1
#define RELAY2_NUM      2
#define RELAY3_NUM      3


// Digital I/O
// DIO1 = PD1 = input
#define DIO1_IN_HIGH                    ((GPIOD->IDR & 0x0002) == 0x0002)

// DIO2 = PD4 = input
#define DIO2_IN_HIGH                    ((GPIOD->IDR & 0x0010) == 0x0010)

// DIO3 = PD7 = output
#define DIO3_OUT_LOW                    GPIOD->BSRRH = 0x0080;
#define DIO3_OUT_HIGH                   GPIOD->BSRRL = 0x0080;

// DIO4 = PD9 = output
#define DIO4_OUT_LOW                    GPIOD->BSRRH = 0x0200;
#define DIO4_OUT_HIGH                   GPIOD->BSRRL = 0x0200;


// Protection Availability LEDs
// LLED = PE3
#define L_LED_OFF                       GPIOE->BSRRH = 0x0008;
#define L_LED_ON                        GPIOE->BSRRL = 0x0008;

// SLED = PE4
#define S_LED_OFF                       GPIOE->BSRRH = 0x0010;
#define S_LED_ON                        GPIOE->BSRRL = 0x0010;

// ILED = PE5
#define I_LED_OFF                       GPIOE->BSRRH = 0x0020;
#define I_LED_ON                        GPIOE->BSRRL = 0x0020;

// GLED = PE6
#define G_LED_OFF                       GPIOE->BSRRH = 0x0040;
#define G_LED_ON                        GPIOE->BSRRL = 0x0040;


// Coil Temperature Multiplexer
// PE12 = EN (Active Low), PE11 = A1, PE10 = A0
#define COIL_TEMP_A_ON                  GPIOE->BSRRL = 0x1000;   \
                                        GPIOE->BSRRH = 0x0C00;   \
                                        GPIOE->BSRRH = 0x1000; 
#define COIL_TEMP_B_ON                  GPIOE->BSRRL = 0x1000;   \
                                        GPIOE->BSRRH = 0x0C00;   \
                                        GPIOE->BSRRL = 0x0400;   \
                                        GPIOE->BSRRH = 0x1000; 
#define COIL_TEMP_C_ON                  GPIOE->BSRRL = 0x1000;   \
                                        GPIOE->BSRRH = 0x0C00;   \
                                        GPIOE->BSRRL = 0x0800;   \
                                        GPIOE->BSRRH = 0x1000; 
#define COIL_TEMP_N_ON                  GPIOE->BSRRL = 0x1000;   \
                                        GPIOE->BSRRL = 0x0C00;   \
                                        GPIOE->BSRRH = 0x1000; 
#define COIL_TEMP_OFF                   GPIOE->BSRRL = 0x1000;

// Coil Temperature 4800Hz control 
#define COIL_TEMP_4800_OFF              GPIOE->BSRRH = 0x4000;
#define COIL_TEMP_4800_ON               GPIOE->BSRRL = 0x4000;

// Time Sync Line Control
// Time Sync pin control - ODR is already set to 0 for low output and PUPDR is set to 1 for pullup
#define TIME_SYNC_OUTLOW                GPIOH->MODER |= 0x00040000;     // PH9 Mode = 1 (Output)
#define TIME_SYNC_INHIGH                GPIOH->MODER &= 0xFFF3FFFF;     // PH9 Mode = 0 (Input)


// Spare Port Pins for Testing
#define TESTPIN_A3_HIGH                 GPIOE->BSRRL = 0x0002;       // PE1
#define TESTPIN_A3_LOW                  GPIOE->BSRRH = 0x0002;
#define TESTPIN_A3_TOGGLE               GPIOE->ODR ^=  0x0002;                                        
                                        
#define TESTPIN_D1_HIGH                 GPIOC->BSRRL = 0x2000;       // PC13
#define TESTPIN_D1_LOW                  GPIOC->BSRRH = 0x2000;                                        
#define TESTPIN_D1_TOGGLE               GPIOC->ODR ^=  0x2000; 



// FM25CL64 FRAM Instructions:
#define FRAM_WRSR       0x01            // Write status register instruction
#define FRAM_WRITE      0x02            // Write memory instruction
#define FRAM_READ       0x03            // Read memory instruction
#define FRAM_WRDI       0x04            // Write disable instruction
#define FRAM_RDSR       0x05            // Read status register instruction
#define FRAM_WREN       0x06            // Write enable instruction



// FRAM device definitions
#define DEV_FRAME       0x0000     
#define DEV_FRAM2       0x0001



// System Flag (SystemFlags) Definitions
//   Note, these must only be altered in the foreground (not in any interrupts)!!  *** DAH  MAKE SURE THIS IS TRUE
#define CAL_FRAM_ERR        0x0001
#define CAL_FLASH_ERR       0x0002
#define VALONECYC           0x0004
#define VAL200MSEC          0x0008
#define INJ_FRAM_ERR        0x0010
#define INJ_FLASH_ERR       0x0020
#define FIRST_PASS          0x0040
#define RTC_ERR             0x0080
#define EVENT_FF_FULL       0x0100
#define ALARM_WF_FRAM_ERR   0x0200
#define TRIP_WF_FRAM_ERR    0x0400
#define EXTCAP_WF_FRAM_ERR  0x0800
#define FRAME_FRAM_ERR      0x1000
#define ONE_CYC_VALS_DONE   0x2000
#define SKIP_LOOPTIME_MEAS  0x4000




// EEPOT Commands
//  | b15 | b14 | b13 | b12 | b11 | b10 | b9  | b8  | b7  | b6  | b5  | b4  | b3  | b2  | b1  | b0  |
//  |Add3 |Add2 |Add1 |Add0 | C1  | C0  | D9  | D8  | D7  | D6  | D5  | D4  | D3  | D2  | D1  | D0  |

// Addresses:                               Commands:
//   Volatile Wiper 0 -    0000               Write - 00
//   Volatile Wiper 1 -    0001               Read -  11
//   NonVolatile Wiper 0 - 0010
//   NonVolatile Wiper 1 - 0011
//   TCON Register -       0100
//   Status Register -     0101

#define EEPOT_RD_W0     0x0C00
#define EEPOT_WR_W0     0x0000
#define EEPOT_RD_W1     0x1C00
#define EEPOT_WR_W1     0x1000
#define EEPOT_RD_NVW0   0x2C00
#define EEPOT_WR_NVW0   0x2000
#define EEPOT_RD_NVW1   0x3C00
#define EEPOT_WR_NVW1   0x3000
#define EEPOT_RD_TCON   0x4C00
#define EEPOT_WR_TCON   0x4000
#define EEPOT_RD_STATUS 0x5C00

#define EEPOT_NVW0_ADD  0x2000
#define EEPOT_NVW1_ADD  0x3000



// ADC3 Channel Definitions
//   The ADC3 channels are:
//      ADC3_IN9   TA Sense
//      ADC3_IN14  Battery Sense
//      ADC3_IN15  Aux Power Sense
//      ADC3_IN5   Not used
//      ADC3_IN6   4.5V Supply
//      ADC3_IN7   Start Time

#define ADC3_TASENSE_CH     0x00000009
#define ADC3_BATSENSE_CH    0x0000000E
#define ADC3_AUXPWR_CH      0x0000000F
#define ADC3_4P5VPWR_CH     0x00000006
#define ADC3_STARTUP_CH     0x00000007
#define ADC3_THERMMEM_CH    0x00000003
#define ADC3_COILTEMP_CH    0x00000002


// Health Bus interface definitions
#define TEMP_SENSOR_DRDYN   ((GPIOI->IDR & 0x0004) == 0x0000)   // Temp/humidity sensor data is ready


#define HLTH_I2C_RXLEN      8
#define HLTH_I2C_TXLEN      8

#define ON_BD_SENSORADD     0x40

#define PTR_TEMP            0x01
#define PTR_HUM             0x02

#define PTR_ADD_TEMP        0x00
#define PTR_ADD_HUM         0x01
#define PTR_ADD_CONFIG      0x02
#define PTR_ADD_MANUFID     0xFE


// Flag definitions
#define READ_SENSOR     0x01
#define I2C_WRITE_REG   0x02
#define I2C_READ_SENSOR 0x03
#define I2C_PROC_VAL    0x04

#define I2C_ERROR       0x08

// Interrupt States
#define I2C3_IDLE       0x00
#define I2C3_WRITECONF  0x01
#define I2C3_WRITEREG   0x02
#define I2C3_READSENS   0x03


// SSt26VF064B Flash Instructions:
#define FLASH_WREN      0x06            // Write enable
#define FLASH_ULBPR     0x98            // Global block protection unlock
#define FLASH_WBPR      0x42            // Write block protection register
#define FLASH_PP        0x02            // Page program
#define FLASH_SE        0x20            // Sector erase
#define FLASH_BE        0xD8            // Block erase
#define FLASH_RDSR      0x05            // Read status register
#define FLASH_RD        0x03            // Read data
#define FLASH_SFDP      0x5A            // Read Serial Flash Discoverable Parameters


// SPI1 Flash Access Flags
// Note, these must line up with the states listed in S1F_NEXT_STATE[ ] in Iod.c!!
#define S1F_TRIP_WF_ERASE       0x00000001          // Erase must be higher priority than write so that
#define S1F_TRIP_WF_ERASE1      0x00000002          //   next block will be erased before a new waveform is
#define S1F_TRIP_WF_WR          0x00000004          //   written
#define S1F_ALARM_WF_ERASE      0x00000008
#define S1F_ALARM_WF_ERASE1     0x00000010
#define S1F_ALARM_WF_WR         0x00000020
#define S1F_EXT_WF_ERASE        0x00000040
#define S1F_EXT_WF_ERASE1       0x00000080
#define S1F_EXT_WF_WR           0x00000100
#define S1F_RD_WF               0x00000200
#define S1F_CAL_WR              0x00000400
#define S1F_INJ_WR              0x00000800
#define S1F_DMND_WR             0x00001000
#define S1F_DMND_ERASE          0x00002000
#define S1F_TP_RD_REQ           0x00004000         // *** DAH  MAY NEED TO REARRANGE FOR PRIORITIES (LOWER THE BIT POSITION, HIGHER THE PRIORITY)!!!  DEMAND SHOULD PROBABLY BE LOWER
#define S1F_CHK_CAL             0x00008000
#define S1F_INJ_RD              0x00010000
#define S1F_AFECAL_RD           0x00020000
#define S1F_ADCHCAL_RD          0x00040000
#define S1F_ADCLCAL_RD          0x00080000
#define S1F_AFECAL_RD1          0x00100000
#define S1F_ADCHCAL_RD1         0x00200000
#define S1F_ADCLCAL_RD1         0x00400000

#define NUM_SPI1FLASH_REQ       23

//#define BIT0                    0x0001           // BITxx definitions are defined in Flags_def.h
//#define BIT1                    0x0002
//#define BIT7                    0x0080
//#define BIT8                    0x0100                                          

//
//------------------------------------------------------------------------------------------------------------
//    Constants
//------------------------------------------------------------------------------------------------------------




//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct SPI1VARS
{
   uint8_t  State;
   uint8_t  ReqNum;
   uint32_t Req;
   uint32_t Ack;
};


struct TH_SENSOR
{
   float Humidity;
   float Temperature;
   float TemperatureMax;
   struct INTERNAL_TIME TemperatureMaxTS;
   float TemperatureMaxInt;
   struct INTERNAL_TIME TemperatureMaxIntTS;
};


struct FRAME_EEPOT
{
   uint16_t RdVal;
   uint16_t WrVal;
   uint16_t State;
};


struct SWITCHES_PUSHBUTTONS
{
   uint8_t  COT_SwitchState;
   uint8_t  COT_Code;
   uint32_t COT_Debounce;

   uint8_t  AUX_OpenFlg;
   uint8_t  AUX_Cycle_Ctr;
   uint32_t AUX_Debounce;
   
   uint8_t  MM_SwitchState;
   uint8_t  MM_Cycle_Ctr;
   uint32_t MM_Debounce;
};


struct STARTUP_TIME_VARS
{
    uint8_t State;
    uint8_t DoCalFlag;
    float OnTime;
    float OffTime;
    float Scale;
    float Time;
};

struct HLTH_I2C_VARS
{
    uint8_t TxNdx;
    uint8_t RxNdx;
    uint8_t Status;
    uint8_t ErrCount;
    uint8_t TxNumChars;
    uint8_t RxNumChars;
    uint8_t State;
    uint8_t Flags;
    uint8_t RxBuf[HLTH_I2C_RXLEN];
    uint8_t TxBuf[HLTH_I2C_TXLEN];
    uint8_t IntState;
};

union FLASH_ID_UNION
{
  uint32_t wd[2];
  uint8_t by[8];
};



//------------------------------------------------------------------------------
//          Relay Configuration
//------------------------------------------------------------------------------
//#define RELAY_OFF               0
#define OVERLOAD_TRIP           0
#define NEUTRAL_TRIP            1
#define SHORT_DELAY_TRIP        2
#define SHORT_CIRC_TRIP         3
#define INSTANT_TRIP            4
#define GROUND_TRIP             5
#define MMODE_TRIP              6

#define MECHANISM_TRIP          7
#define DIR_SHORT_CIRC          8
#define OVER_VOLTAGE            9
#define UNDER_VOLTAGE           10

#define OVER_FREQ               11
#define UNDER_FREQ              12
#define VOLTAGE_UNBALANCE       13
#define CURRENT_UNBALANCE       14

#define REVERS_ACTIVE_PWR       15
#define PHASE_ROTATION          16
#define PHASE_LOSS              17
#define ALL_TRIPS               18
//#define SPARE                   19

//#define SPARE                   20
#define HIGH_LOAD1_ALARM        21
#define HIGH_LOAD2_ALARM        22
#define HIGH_TEMP_ALARM         23
#define GND_FAULT_PREALARM      24
#define THERMAL_MEMORY_ALARM    25
#define WTCHDOG_AUX_PWR_ALARM   26
#define LOW_BATTERY_ALARM       27
#define INTERNAL_ALARM          28

#define STP_MISMATCH_ALARM      29
#define HEALTH_WARNING_ALARM    30
#define COMM_FAULT_ALARM        31
#define FAULT_PRESENT_ALARM     32
#define OV_ALARM                33

#define UV_ALARM                34
#define OVER_FREQ_ALARM         35
#define UNDER_FREQ_ALARM        36
#define VOLTAGE_UNBALANCE_ALARM 37

#define CURRENT_UNBALANCE_ALARM 38
#define REVERS_ACTIVE_PWR_ALARM 39
#define PHASE_ROTATION_ALARM    40
#define PHASE_LOSS_ALARM        41
#define ALL_ALARM               42
//#define SPARE                   43

#define REV_REACT_PWR           44
#define THD_DISTORTION          45
#define UNDER_PWR               46
#define REAL_PWR                47

#define REACT_PWR               48
#define APP_PWR                 49
#define PWR_DEMAND              50
//#define SPARE                   51
//#define SPARE                   52
#define AUX_CONTACT             53
#define BELL_CONTACT            54
#define MM_ACTIVE               55
#define ZSI_ACTIVE              56

#define ZSI_INPUT_RECEIVED      57
#define ZSI_OUTPUT_SENT         58
#define OPEN_BREAKER_PULSED     59
#define CLOSE_BREAKER_PULSED    60
#define REMOTE_CONTROL          61
#define SYNC_CHECK              62
//#define SPARE                   63
//#define SPARE                   64

#define MAX_RELAY_ASS           SYNC_CHECK

#define RELAY_CLOSE       1
#define RELAY_OPEN        0

