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
//  MODULE NAME:        Init_def.h
//
//  MECHANICS:          This is the definitions file for the Init.c module
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
//   0.04   150928  DAH Changed PG7 (uC_Breaker_Closed) from Input with Pull-down to Input with Pull-up
//   0.05   151021  DAH - Added GPIOx_ODR_DEF (GPIO output register) definitions
//                      - Corrected RTC chip enable pin (PH3).  It is active high, not active low
//                      - Replaced DEV_FRAM_FLASH with DEV_FRAM_FLASH8 and DEV_FRAM_FLASH16 in
//                        SPI2_DEVICE_CODES to support both 8 and 16-bit transactions
//   0.06   151110  DAH - Added GPIOH_ODR_DEF for DS1390 RTC instead of DS1305 RTC.  PH3 is active low for
//                        the DS1390, not active high for the DS1305  These definitions are commented out
//                        for now - the DS1305 definitions will still be used until the rev 02 boards arrive
//                      - Revised GPIOE_ODR_DEF to make the initial state of AFE_START high instead of low
//   0.07   151216  DAH - Corrected GPIOH_TYPE_DEF
//   0.08   160122  DAH - Removed pull-up from PH7 (corrected GPIOH_PUPDR_DEF)
//   0.09   160224  DAH - Revised pin definitions for Rev 2 board
//                      - Eliminated EEPOT constants and added temperature/humidity sensor constant in
//                        SPI2_DEVICE_CODES
//                      - Changed port C ADC current input mux control (PC8) to initialize for high-gain
//                        inputs.  The high-gain inputs will be used initially for seeding.
//   0.12   160502  DAH - Changed PA8 definitions from unused GPIO to Timer1 output for AFE clock
//                      - Changed PC13 and PE1 from inputs to outputs 
//   0.14   160620  DAH - Re-added support for writing Frame EEPOT settings
//                          - DEV_EEPOT_8 and DEV_EEPOT_16 added
//   0.16   160818  DAH - Corrected PG10 Type and Mode register definitions and Port H Type reg definitions
//   0.18   161115  DAH - Renamed PA1 from Vn_Load to Vn_Line to match rev 3 board and schematics
//                      - Changed PA9, PA11, PA12, PF7 from USB signals to NC to match rev 3 board and
//                        schematics (USB moved to display processor)
//                      - Renamed PB0 from Vc_Load to Vc_Line to match rev 3 board and schematics
//                      - Changed PB10 and PB11 configuration from UART to NC to match rev 3 board and
//                        schematics (Modbus moved to display processor)
//                      - Renamed PC4 from Va_Load to Va_Line to match rev 3 board and schematics
//                      - Renamed PC5 from Vb_Load to Vb_Line to match rev 3 board and schematics
//                      - Changed PF0 configuration from GPIO Output to GPIO Input to match rev 3 board and
//                        schematics (signal changed from Clear LEDs to AFE Alert)
//                      - Changed PG0 description from temp/humidity enable to highload alarm LED and
//                        initial output state from high to low to match rev 3 board and schematics
//                      - Changed PG13 from unused to TH_MEM_CHARGEN for the new (corrrected) thermal memory
//                        interface on the rev 3 boards
//                      - Changed PH11 configuration from active low to active high to match rev 3 board
//                      - Renamed PI2 from NC to TEMP_SENSOR_DRDYN to match rev 3 board
//                      - Changed PI6 (LED_SELECT1) from push-pull output to input without pullup for rev 3
//                        board (there is an external pull-down on the board).  Want normal state to be
//                        input so that when reset button is pushed, the battery will not be drained through
//                        the port line.  Also modified initial ODR value for the port pin to be high, so
//                        that when the mode is switched to output, it will be high.  This way, can toggle
//                        the line by just switching the mode.
//                      - Changed PG14 to an input.  The Modbus Tx/Rx control was moved to the display
//                        processor
//   0.29   190122  DAH - Revised PH13 configuration to support coil temperature measurement
//                      - Revised PE12 configuration so that the coil temperature mux is OFF initially
//   0.34   191002  DAH - Revised pin definitions for rev 4 boards:
//                           Pin    Rev 3                   Rev 4
//                           PI5    TEST_ D/A OUT SEL       TEST_INJ_EN
//                           PH13   RELAY_DR2               4P8K_INJ
//                           PH10   ARCON_SHORTING_BAR      RELAY_DR2
//                           PH6    NC                      SPI2_SB_MICRO_EN
//                           PG15   ARCON_LIGHT_INPUT       NC
//                           PG14   MODBUS_DE/REN           SPI2_SB_MICRO_EN
//                           PF7    NC                      uC_Freq_Line
//                           PE14   NC                      4P8K_INJ_EN
//                           PE13   NC                      INJ_BYPASS_EN
//                           PE3    TA_ACT                  TRACE D0
//                           PE8    NC                      GND_INJ_EN
//                           PE7    NC                      SPI1_2M_FRAM_EN
//                           PE6    Load Freq               TRACE D3
//                           PE5    Line Freq               TRACE D2
//                           PE4    SPI2_DAC_CEN            TRACE D1
//                           PE2    SPI1_FRAM_CEN           TRACE CLK
//                           PD15   NC                      PING_OVR
//                           PD13   NC                      CAM1_DE
//                           PD12   NC                      CAM1_RE
//                           PD11   NC                      CAM2_DE
//                           PD10   NC                      CAM2_RE
//                           PD8    NC                      AFE_DATA_READYN
//                           PD3    NC                      TA_ACT
//                           PD0    NC                      GF_CT_EN
//                           PB11   NC                      UART3 RX (Display processor)
//                           PB10   NC                      UART3 TX (Display processor)
//                           PB8    AFE_DATA_READYN         uC_Freq_Load
//                      - Revised configuration for PE12 because it is now active low, not active high
//                      - Changed pin numbers in the descriptions for the new micro package
//                      - Changed PC6 and PC7 configuration on power-up to low outputs so pins do not power
//                        the Modbus transceiver chip
//   0.35   191118  DAH - Changed PC9 from NC to neutral sense
//   0.42   201202  DAH - Added RTC_BKUP_CODE1 and RTC_BKUP_CODE2 to support the internal RTC
//   0.44   210324  DAH - Added code to make PC9 low on power up, so AFE is on.  Circuit was added for the
//                        new rev 6 (with H7 display processor) boards.  This change does not affect the old
//                        rev 4 boards (PC9 was TP7, unused)
//   0.45   210504  DAH - Changes for H743 board revision
//                          - Alarm LED moved from PH9 to PA11 (PA11 was NC)
//                          - PH9 changed from Alarm LED to DP_TIME_SYNC
//                          - OVR_TRIPPED moved from PB2 to PB7 (PB7 was NC)
//                          - PB2 changed to PP_BOOT1
//                          - PB9 changed from BATTERY SENSE EN to uC_Freq_Line
//                          - PF7 changed from uC_Freq_Line to Frame_Bd_ID
//                          - PD1 changed from NC to DIN2
//                          - PD4 changed from NC to DIN3
//                          - PD7 changed from NC to DIN4
//                          - PD9 changed from NC to DIN5
//                          - PD14 changed from NC to NEU_CT_EN
//                          - PF15 changed from NC to USB_Power_On
//                          - PG4 changed from NC to BAT_SENSE_EN
//                          - PH6 changed from SPI2_SB_MICRO_EN (same as PG14) to USB_BOOST_EN
//                          - Swapped LED_STATUS_GRN and LED_STATUS_RED
//                      - Changed AF register for PB8 and PB9 from 0 to 3
//   0.49   220131  DAH - Changed GPIOC_MODE_DEF to come up with P6 and P7 configured for UART6 operation to
//                        allow Modbus to work.  This will will eventually need to be changed back to output
//                        low so Modbus is not powered without aux power  *** DAH
//   0.56   220526  DAH - Revised GPIOH_ODR_DEF to activate PH6 to turn on USB Boost Enable (for John Downs
//                        testing).  This will have to be modified for final production *** DAH
//   0.67   221209  DAH - Revised pin definitions for new hardware revision:
//                           Pin    Old                     New
//                           PA9    SB SYNC (Not used)      Alarm LED
//                           PA11   Alarm LED               CAN_Rx
//                           PA12   TP24                    CAN_Tx
//                           PD1    DIN2                    DIO1 (input)
//                           PD4    DIN3                    DIO2 (input)
//                           PD7    DIN4                    DIO3 (output)
//                           PD9    DIN5                    DIO4 (output)
//                           PE3    TRACED0                 TRACED0_LLED
//                           PE4    TRACED1                 TRACED1_SLED
//                           PE5    TRACED2                 TRACED2_ILED
//                           PE6    TRACED3                 TRACED3_GLED
//                           PE7    SPI1_2M_FRAM_EN         ECAP_FRAM_EN
//                           PG2    TP32                    Alarm LED (not used - same as PA9)
//                           PG6    SPI2_FLASH_CEN          SPI1_FLASH_CEN
//                           PG9    EXT_SETPT_GP2           NC
//                           PG14   PP_SPI2_SB_MICRO_EN     NC
//
//   42     230623  MAG - Revised Alternate Function assignments for PB8 and PB9 
//
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
//                                I/O Pin Definitions
//  Reference Table 9 - Alternate Function Mapping - of the STM32F407 data sheet (DocID022152)
//    Mode register: b0-1: Px0, b2-3: Px1, ..., b30-31: Px15
//    Speed register: b0-1: Px0, b2-3: Px1, ..., b30-31: Px15
//    Type register: b0: Px0, b1: Px1, ..., b15: Px15     b16-31: Not used
//    PUPDR register: b0-1: Px0, b2-3: Px1, ..., b30-31: Px15
//    Input register: b0: Px0, b1: Px1, ..., b15: Px15     b16-31: Not used
//    Output register: b0: Px0, b1: Px1, ..., b15: Px15     b16-31: Not used
//    AFRLOW register: b0-3: Px0, b4-7: Px1, ..., b28-31: Px7
//    AFRHIGH register: b0-3: Px8, b4-7: Px9, ..., b28-31: Px15

//                                   Chart Key
//                   Mode: 0=Input, 1=Output, 2=Alternate, 3=Analog
//                   Speed: 0=Low, 1=Medium, 2=Fast, 3=High
//                   Type: 0=Push-Pull (for outputs) or not applicable (for inputs), 1=Open-Drain
//                   PUPDR: 0=None, 1=Pull-up, 2=Pull-down, 3=Reserved

//                                  Port A Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//         138   PA15       AF0 - JTDI          2      0     0     0    SYS JTDI
//         137   PA14       AF0 - JTCK_SWCLK    2      0     0     0    SYS JTCK SWCLK
//         124   PA13       AF0 - JTMS_SWDIO    2      0     0     0    SYS JTMS SWDIO
//         123   PA12       AF9 - CAN1_TX       2      0     0     0    CAN_TX
//         122   PA11       AF9 - CAN1_RX       2      0     0     0    CAN_RX
//         121   PA10       AF7 - USART1 RX     2      0     0     0    UART1 RX (CAM1)
//         120   PA9        GPIO - OUT_PP_NP    1      0     0     0    Alarm LED
//         119   PA8        AF1 - TIM1 CH1 OUT  2      0     0     0    AFE CLOCK
//          53   PA7        AF5 - SPI1 MOSI     2      0     0     0    SPI1_MOSI (Flash for events)
//          52   PA6        AF5 - SPI1 MISO     2      0     0     0    SPI1_MISO (Flash for events)
//          51   PA5        AF5 - SPI1 SCK      2      0     0     0    SPI1 SCLK (Flash for events)
//          50   PA4        DAC - OUT1          3      0     0     0    DAC1 Analog Output
//          47   PA3        ADC3_IN3            3      0     0     0    Thermal Memory Analog Input
//          42   PA2        ADC3_IN2            3      0     0     0    COIL TEMP Analog input
//          41   PA1        ADC1_IN1            3      0     0     0    Vn_Line Analog Input
//          40   PA0        GPIO - IN_PU        0      0     0     1    PWR RESET
#define GPIOA_MODE_DEF      0xAAA6ABFC
#define GPIOA_SPEED_DEF     0x00000000
//#define GPIOA_SPEED_DEF     0x00030000    If bringing clock out on PA8, must set pin speed to very fast
#define GPIOA_TYPE_DEF      0x00000000 
#define GPIOA_PUPDR_DEF     0x00000001      // No need pulls on CAN lines 
#define GPIOA_AFRLOW_DEF    0x55500000 
#define GPIOA_AFRHI_DEF     0x00099701
//#define GPIOA_AFRHI_DEF     0x00000700              // *** DAH PA8 remapped to be PLL clock output - AF0
#define GPIOA_ODR_DEF       0x00000000 
 
//                                  Port B Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//          95   PB15       AF5 - SPI2 MOSI     2      0     0     0    SPI2 MOSI (GENERAL)
//          94   PB14       AF5 - SPI2 MISO     2      0     0     0    SPI2 MISO (GENERAL)
//          93   PB13       AF5 - SPI2 SCK      2      0     0     0    SPI2 SCLK (GENERAL)
//          92   PB12       GPIO - IN_NP        0      0     0     0    uC MM (has ext pull-up)
//          80   PB11       AF7 - USART3_RX     2      0     0     1    UART3 RX (Display processor)   *** DAH  NEED TO CONFIGURE AS GPIO OUTPUT LOW SO THEY DO NOT POWER THE DISPLAY PROCESSOR WHEN IT IS UNPOWERED
//          79   PB10       AF7 - USART3_TX     2      0     0     1    UART3 TX (Display processor)            THEN SWITCH TO UART WHEN WE POWER THE DISPLAY MICRO!!!!
//         168   PB9        AF2 - TMR4 CH4      2      0     0     0    uC_Freq_Line
//         167   PB8        AF2 - TMR4 CH3      2      0     0     0    uC_Freq_Load
//         165   PB7        GPIO - IN_NP        0      0     0     0    OVR_TRIPPED
//         164   PB6        AF7 - USART1 TX     2      0     0     0    UART1 TX (CAM1)
//         163   PB5        AF6 - SPI3 MOSI     2      2     0     0    SPI3 MOSI (AFE)
//         162   PB4        AF0 - NJTRST        2      0     0     0    SYS_JTRST
//         161   PB3        AF0 - JTDO          2      0     0     0    SYS_JTDO_SW0
//          58   PB2        GPIO - IN_NP        0      0     0     0    PP_BOOT1 (has ext pull-down)
//          57   PB1        ADC2_IN9            3      0     0     0    1.25V Reference Analog Input
//          56   PB0        ADC2_IN8            3      0     0     0    Vc_Line Analog Input
#define GPIOB_MODE_DEF      0xA8AA2A8F
#define GPIOB_SPEED_DEF     0x00000800 
#define GPIOB_TYPE_DEF      0x00000000 
#define GPIOB_PUPDR_DEF     0x00500000 
#define GPIOB_AFRLOW_DEF    0x07600000 
#define GPIOB_AFRHI_DEF     0x55507722 
#define GPIOB_ODR_DEF       0x00000000 

//                                  Port C Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//          10   PC15       GPIO - IN_PU        0      0     0     1    NC
//           9   PC14       GPIO - IN_PU        0      0     0     1    NC
//           8   PC13       GPIO - OUT_PP_PU    1      0     0     1    UC_D1 Test pin   *** DAH  RENAME THIS TP5 TEST PIN AFTER WE BRING IN THE GOOSE TEST CODE
//         141   PC12       AF8 - UART5 TX      2      0     0     1    UART5 TX (TEST PORT)
//         140   PC11       AF6 - SPI3 MISO     2      2     0     0    SPI3 MISO (AFE)
//         139   PC10       AF6 - SPI3 SCK      2      2     0     0    SPI3 SCK (AFE)
//         118   PC9        GPIO - OUT_PP_NP    1      0     0     0    AFE_POWER_ONN  (has ext pull-up)
//         117   PC8        GPIO - OUT_PP_NP    1      0     0     0    ST_Low_Gain (has ext pull-up)
//         116   PC7        AF8 - USART6 RX     2      0     0     0    UART6 RX (CAM2) Normal Operation
//         116   PC7        GPIO - OUT_PP_NP    1      0     0     0    UART6 RX (CAM2) Power up so pin cannot power xcvr chip
//         115   PC6        AF8 - USART6 TX     2      0     0     0    UART6 TX (CAM2) Normal Operation
//         115   PC6        GPIO - OUT_PP_NP    1      0     0     0    UART6 TX (CAM2) Power up so pin cannot power xcvr chip
//              Note, on power-up PC6 and PC7 are initialized as GPIO outputs set low so that they do not power the
//                    Modbus or CAM2 transceiver chips
//          55   PC5        ADC2_IN15           3      0     0     0    Vb_Line Analog Input
//          54   PC4        ADC2_IN14           3      0     0     0    Va_Line Analog Input
//          35   PC3        ADC1_IN13           3      0     0     0    In Analog Input
//          34   PC2        ADC1_IN12           3      0     0     0    Ic Analog Input     
//          33   PC1        ADC1_IN11           3      0     0     0    Ib Analog Input
//          32   PC0        ADC1_IN10           3      0     0     0    Ia Analog Input
#define GPIOC_MODE_DEF      0x06A5AFFF       // Normal Operation       *** DAH 220128  Enable pins for uart operation for modbus
//#define GPIOC_MODE_DEF      0x06A55FFF      // Power up
#define GPIOC_SPEED_DEF     0x00A00000 
#define GPIOC_TYPE_DEF      0x00000000 
#define GPIOC_PUPDR_DEF     0x55000000 
#define GPIOC_AFRLOW_DEF    0x88000000 
#define GPIOC_AFRHI_DEF     0x00086600 
#define GPIOC_ODR_DEF       0x00000000      // Initialize for high-gain current inputs, PC6 and PC7 low
                                            //   PC9 low - AFE On

//                                  Port D Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//         105   PD15       GPIO - OUT_PP_NP    1      0     0     0    PING_OVR
//         104   PD14       GPIO - OUT_PP_NP    1      0     0     0    NEU_CT_EN (active low)
//         101   PD13       GPIO - OUT_PP_PD    1      0     0     2    CAM1_DE
//         100   PD12       GPIO - OUT_PP_PU    1      0     0     2    CAM1_REN (active low)
//          99   PD11       GPIO - OUT_PP_PD    1      0     0     2    CAM2_DE
//          98   PD10       GPIO - OUT_PP_PD    1      0     0     2    CAM2_REN (active low)
//          97   PD9        GPIO - OUT_PP_NP    1      0     0     0    DIO4 (output)
//          96   PD8        GPIO - IN_PU        0      0     0     1    AFE_DATA_READYN
//         151   PD7        GPIO - OUT_PP_NP    1      0     0     0    DIO3 (output)
//         150   PD6        AF7 - USART2 RX     2      0     0     1    UART2 RX (DISPLAY)     *** DAH  NEED TO CONFIGURE AS GPIO OUTPUT LOW SO THEY DO NOT POWER THE DISPLAY PROCESSOR WHEN IT IS UNPOWERED
//         147   PD5        AF7 - USART2_TX     2      0     0     1    UART2 TX (DISPLAY)
//         146   PD4        GPIO - IN_NP        0      0     0     0    DIO2 (input with external pull-up)
//         145   PD3        GPIO - IN_NP        0      0     0     0    TA_ACT (has weak ext pull-down)  *** DAH NO INTERRUPT?
//         144   PD2        AF8 - UART5 RX      2      0     0     1    UART5 RX (TEST PORT)     
//         143   PD1        GPIO - IN_NP        0      0     0     0    DIO1 (input with external pull-up)
//         142   PD0        GPIO - OUT_PP_NP    1      0     0     0    GF_CT_EN
#define GPIOD_MODE_DEF      0x55546821
#define GPIOD_SPEED_DEF     0x00000000 
#define GPIOD_TYPE_DEF      0x00000000 
#define GPIOD_PUPDR_DEF     0x0AA11410 
#define GPIOD_AFRLOW_DEF    0x07700800
#define GPIOD_AFRHI_DEF     0x00000000 
#define GPIOD_ODR_DEF       0x00000000          // CAM1 and CAM2 rcvr enabled, driver disabled
        // Note, the Modbus isolator is not powered initially, so the CAM2 RE and DE signals should be low
        //   so that the isolator does not draw current

//                                  Port E Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//          78   PE15       GPIO - OUT_PP_NP    1      0     0     0    ST_TA_TRIP (has ext pull-down)
//          77   PE14       GPIO - OUT_PP_PD    1      0     0     2    4P8K_INJ_EN
//          76   PE13       GPIO - OUT_PP_PD    1      0     0     2    INJ_BYPASS_EN
//          75   PE12       GPIO - OUT_PP_PU    1      0     0     1    TMP_CH_EN (active low)
//          74   PE11       GPIO - OUT_PP_PD    1      0     0     2    TMP_CH_A1
//          73   PE10       GPIO - OUT_PP_PD    1      0     0     2    TMP_CH_A0
//          70   PE9        GPIO - IN_PU        0      0     0     1    RTC_PULSEIN
//          69   PE8        GPIO - OUT_PP_PD    1      0     0     2    GND_INJ_EN
//          68   PE7        GPIO - OUT_PP_NP    1      0     0     0    ECAP_FRAM_EN (has ext pull-up)
//           5   PE6        AF0  - TRACE D3     2      0     0     0    TRACED3_GLED  CONFIG FOR TRACED3
//           5   PE6        GPIO - OUT_PP_NP    1      0     0     0    TRACED3_GLED  CONFIG FOR GLED
//           4   PE5        AF0  - TRACE D2     2      0     0     0    TRACED2_ILED  CONFIG FOR TRACED2
//           4   PE5        GPIO - OUT_PP_NP    1      0     0     0    TRACED2_ILED  CONFIG FOR ILED
//           3   PE4        AF0  - TRACE D1     2      0     0     0    TRACED1_SLED  CONFIG FOR TRACED1
//           3   PE4        GPIO - OUT_PP_NP    1      0     0     0    TRACED1_SLED  CONFIG FOR SLED
//           2   PE3        AF0  - TRACE D0     2      0     0     0    TRACED0_LLED  CONFIG FOR TRACED0
//           2   PE3        GPIO - OUT_PP_NP    1      0     0     0    TRACED0_LLED  CONFIG FOR LLED
//           1   PE2        AF0  - TRACE CLK    2      0     0     0    TRACE CLK
//         170   PE1        GPIO - OUT_PP_PU    1      0     0     1    UC_A3 Test pin    *** DAH  RENAME THIS TP6 TEST PIN AFTER WE BRING IN THE GOOSE TEST CODE
//         169   PE0        GPIO - OUT_PP_PU    1      0     0     1    AFE_STARTN
#define GPIOE_MODE_DEF      0x55515565
#define GPIOE_SPEED_DEF     0x00000000 
#define GPIOE_TYPE_DEF      0x00000000 
#define GPIOE_PUPDR_DEF     0x29A60005 
#define GPIOE_AFRLOW_DEF    0x00000000
#define GPIOE_AFRHI_DEF     0x00000000
#define GPIOE_ODR_DEF       0x00001099

//                                  Port F Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//          65   PF15       GPIO - IN_NP        0      0     0     0    USB_Power_On (has ext divider)
//          64   PF14       GPIO - OUT_PP_PU    1      0     0     1    PHN_SEL
//          63   PF13       GPIO - OUT_PP_PU    1      0     0     1    PHC_SEL
//          60   PF12       GPIO - OUT_PP_PU    1      0     0     1    PHB_SEL
//          59   PF11       GPIO - OUT_PP_PU    1      0     0     1    PHA_SEL
//          28   PF10       ADC3_IN8            3      0     0     0    1.25V CHECK Analog Input
//          27   PF9        ADC3_IN7            3      0     0     0    START TIME Analog Input
//          26   PF8        ADC3_IN6            3      0     0     0    4.5V SUPPLY SENSE Analog Input
//          25   PF7        GPIO - IN_PU        0      0     0     1    Frame_Bd_ID
//          24   PF6        ADC3_IN4            3      0     0     0    DISPLAY_3.3V_MONITOR Analog Input
//          21   PF5        ADC3_IN15           3      0     0     0    AUX PWR SENSE Analog Input
//          20   PF4        ADC3_IN14           3      0     0     0    BATTERY SENSE Analog Input
//          19   PF3        ADC3_IN9            3      0     0     0    TA_SENSE Analog Input
//          18   PF2        GPIO - OUT_PP_PU    1      0     0     0    AFE_RESETN (has ext pull-down)
//          17   PF1        GPIO - OUT_PP_NP    1      0     0     0    DISPLAY_SHUTDOWN (external pull-down)
//          16   PF0        GPIO - IN_NP        0      0     0     0    AFE ALERT (has ext pull-down)
#define GPIOF_MODE_DEF      0x157F3FD4
#define GPIOF_SPEED_DEF     0x00000000 
#define GPIOF_TYPE_DEF      0x00000000 
#define GPIOF_PUPDR_DEF     0x15404000 
#define GPIOF_AFRLOW_DEF    0x00000000 
#define GPIOF_AFRHI_DEF     0x00000000 
#define GPIOF_ODR_DEF       0x00007800 

//                                  Port G Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//         160   PG15       GPIO - IN_PU        0      0     0     1    TP33 (not used)
//         157   PG14       GPIO - IN_PU        0      0     0     1    NC
//         156   PG13       GPIO - OUT_OD_NP    1      0     1     0    TH_MEM_CHARGEN (has ext pull-up)
//         155   PG12       GPIO - OUT_PP_NP    1      0     0     0    ZOUT (has ext pull-down)
//         154   PG11       GPIO - OUT_PP_NP    1      0     0     0    Trip LED
//         153   PG10       GPIO - OUT_IN_NP    0      0     0     0    START_TIME_DIS (has ext pull-down)
//              Note, make input after reset so startup time cap is not affected when measuring the time.
//                    Pin is changed to open-drain output after the startup time has been measured
//         152   PG9        GPIO - IN_PU        0      0     0     1    NC
//         112   PG8        GPIO - IN_NP        0      0     0     0    LED_RST (has ext pull-down)
//         111   PG7        GPIO - IN_NP        0      0     0     0    uC_BREAKER_CLOSED (has ext pull-up)
//         110   PG6        GPIO - OUT_PP_NP    1      0     0     0    SPI1_FLASH_CEN (has ext pull-up)
//         109   PG5        GPIO - OUT_PP_PU    1      0     0     1    SPI_AFE_CSN
//         108   PG4        GPIO - OUT_PP_NP    1      0     0     0    BAT_SENSE_EN
//         107   PG3        GPIO - IN_NP        0      0     0     0    ZIN_COMP (has ext pull-down)
//         106   PG2        GPIO - IN_PU        0      0     0     1    Alarm LED (not used - same as PA9)
//          67   PG1        GPIO - IN_PU        0      0     0     1    AFE DOUT0
//          66   PG0        GPIO - OUT_PP_NP    1      0     0     0    HIGHLOAD LED
#define GPIOG_MODE_DEF      0x05401501
#define GPIOG_SPEED_DEF     0x00000000 
#define GPIOG_TYPE_DEF      0x00002000 
#define GPIOG_PUPDR_DEF     0x50040414 
#define GPIOG_AFRLOW_DEF    0x00000000 
#define GPIOG_AFRHI_DEF     0x00000000 
#define GPIOG_ODR_DEF       0x00002060

//                                  Port H Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//         130   PH15       GPIO - OUT_PP_PU    1      0     0     1    TRIP_REQUEST
//         129   PH14       GPIO - OUT_PP_PD    1      0     0     2    RELAY_DR3
//         128   PH13       AF3 - TIM8 CH1 OUT  2      0     0     0    4P8K_INJ
//          89   PH12       GPIO - OUT_PP_PD    1      0     0     2    RELAY_DR1
//          88   PH11       GPIO - OUT_PP_NP    1      0     0     0    5V_AUX PWR ENABLE (has ex pull-down)
//          87   PH10       GPIO - OUT_PP_PD    1      0     0     2    RELAY_DR2
//          86   PH9        GPIO - IN_PU        0      0     0     1    DP_TIME_SYNC
//          85   PH8        AF4 - I2C3_SDA      2      0     1     0    I2C3_SDA (Health Bus)
//          84   PH7        AF4 - I2C3_SCL      2      0     1     0    I2C3_SCL (Health Bus)
//          83   PH6        GPIO - OUT_PP_NP    1      0     0     0    USB_BOOST_EN (has ext pull-down)
//          46   PH5        AF4 - I2C2_SDA      2      0     1     0    I2C2_SDA (Override Micro)
//          45   PH4        AF4 - I2C2_SCL      2      0     1     0    I2C2_SCL (Override Micro)
//          44   PH3        GPIO - OUT_PP_NP    1      0     0     0    SPI2_RTC_CEN (has ext pull-up)
//          43   PH2        GPIO - OUT_PP_PU    1      0     0     0    SPI2_FRAM_CEN (has ext pull-up)
//          30   PH1        OSC_OUT             0      0     0     0    RCC_OSC_OUT
//          29   PH0        OSC_IN              0      0     0     0    RCC_OSC_IN
//#define GPIOH_MODE_DEF      0x55568A50                FOR RELAY_DR2
#define GPIOH_MODE_DEF      0x59529A50
#define GPIOH_SPEED_DEF     0x00000000 
#define GPIOH_TYPE_DEF      0x000001B0 
#define GPIOH_PUPDR_DEF     0x62240000
#define GPIOH_AFRLOW_DEF    0x40440000 
#define GPIOH_AFRHI_DEF     0x00300004
#define GPIOH_ODR_DEF       0x0000004C


//                                  Port I Configuration
//         Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//          13   PI11       GPIO - OUT_PP_PD    1      0     0     0    LD LED (ext pull-down) 
//          12   PI10       GPIO - OUT_PP_PD    1      0     0     0    INST LED (ext pull-down) 
//          11   PI9        GPIO - OUT_PP_PD    1      0     0     0    GND LED (ext pull-down) 
//           7   PI8        GPIO - OUT_PP_PD    1      0     0     0    SD LED (ext pull-down) 
//         176   PI7        GPIO - OUT_PP_NP    1      0     0     0    LDPU_LED
//         175   PI6        GPIO - IN_NP        0      0     0     0    LED_SELECT1 (external pull-down)
//         174   PI5        GPIO - OUT_PP_PU    1      0     0     2    TEST_INJ_EN
//         173   PI4        GPIO - OUT_PP_NP    1      0     0     0    SPI2_FRAME_CEN (external pull-up)
//         134   PI3        GPIO - OUT_PP_PU    1      0     0     1    SPI2_MCR_CEN
//         133   PI2        GPIO - OUT_IN_PU    0      0     0     1    TEMP_SENSOR_DRDYN
//         132   PI1        GPIO - OUT_PP_PD    1      0     0     0    LED_STATUS_GRN
//         131   PI0        GPIO - OUT_PP_PD    1      0     0     0    LED_STATUS_RED
#define GPIOI_MODE_DEF      0x00554545
#define GPIOI_SPEED_DEF     0x00000000 
#define GPIOI_TYPE_DEF      0x00000000 
#define GPIOI_PUPDR_DEF     0x00000850 
#define GPIOI_AFRLOW_DEF    0x00000000 
#define GPIOI_AFRHI_DEF     0x00000000
#define GPIOI_ODR_DEF       0x00000058 


enum SPI2_DEVICE_CODES
{
    DEV_RTC,
    DEV_TH_SENSOR8,
    DEV_TH_SENSOR16,
    DEV_FRAM_FLASH8,
    DEV_FRAM_FLASH16,
    DEV_DAC,
    DEV_EEPOT_8,
    DEV_EEPOT_16
};


// RTC memory codes for configuration check
#define RTC_BKUP_CODE1  0x55AD2794
#define RTC_BKUP_CODE2  0xAA46C163

