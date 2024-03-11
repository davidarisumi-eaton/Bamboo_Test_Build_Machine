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
//  MODULE NAME:        Init.c
//
//  MECHANICS:          Program module containing the functions for hardware initialization
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
//   0.04   150928  DAH Eliminated PF9 (Trip Status) interrupt structure.  Init_InterruptStruct() revised
//                      Revised AFE_Init() to set up AFE for 4800 samples/sec and Low Power Mode
//   0.05   151021  DAH - Added code to Init_GPIO() to initialize the port output registers
//                      - Revised Init_SPI2() to set the slave select management bits (b8 and 9) in SPI2->CR
//   0.06   151110  DAH - Deleted Init_FSMC() and references to Flexible Static Memory Controller
//                      - Revised Init_SPI2() for DS1390 RTC instead of DS1305 RTC
//                      - Added Init_RTC()
//                      - Revised Init_InterruptStruct() to only set PF8 to falling edge (FTSR register)
//                  BP  - Revised Init_ADC1().
//                      - Added Init_TIM3() and Init_DMAController2().
//   0.07   151216  DAH - Modified Init_SPI2() to reduce SPI clock speed for EEPOT configuration
//                      - Added code to set the ACK bit in I2C2->CR1 after the peripheral is enabled in
//                        EnDisPeripherals()
//   0.08   160122  DAH - Revised Init_UART6() to correct the character length
//                      - Revised EnDisPeripherals() to enable and disable the Modbus interface (RS485) chip
//                      - Modified AFE_Init() to enable and disable the appropriate on-chip diagnostic
//                        tests.  Only those tests that indicate a "broken" chip are enabled.
//                      - Modified AFE_Init() to have the sampling decimation registers assume an 8.192MHz
//                        crystal.  The old decimation register values were for an 8.000MHz crystal.  Also
//                        added comments and cleaned up the code
//   0.09   160224  DAH - Swapped UART3 (CAM2 to Modbus) and UART6 (Modbus to CAM2) for rev 2 hardware
//                      - Moved AFE_DATA_READYN signal from PF8 to PB8 for rev 2 hardware
//                      - Revised comment in Init_InterruptStruct().  PF8 changed to PB8 for rev 2 hardware
//                      - Removed EEPOT (no longer used) and added HIH6030 temperature/humidity sensor for
//                        rev 2 hardware
//                      - Revised AFE_Init() for 4.096MHz crystal instead of 8.192MHz crystal
//                      - Moved ADC1, ADC2, and ADC3 enabling from the initialization functions to
//                        EnDisPeripherals().  The ADCs are now enabled after the 120MHz clock is present.
//   0.11   160317  DAH - Corrected error in Init_InterruptStruct().  The NVIC Interrupt Priority was
//                        configured for DMA1 Stream 5 instead of DMA1 Stream 0.
//   0.12   160502  DAH - Added Init_TIM1() for the AFE clock
//                      - Corrected bug in Init_UART5().  Corrected baud rate register (UART5->BRR) setting
//                        (114 changed to 104) was corrected for 16MHz operation.
//                      - Revised AFE_Init() for 4MHz clock instead of 4.096MHz external crystal
//                      - Removed Init_TIM9() from SysClkHandler() (added to initialization code in the
//                        main loop)
//                      - Modified Init_TIM3() to initialize for 120MHz only.  ADC readings will only be
//                        taken when the system clock is 120MHz.  Otherwise the unit is broken.  Removed
//                        Init_TIM3() from SysClkHandler() (added to initialization code in the main loop)
//                      - Removed the enabling of UART5 and SysTick from EnDisPeripherals() and added the
//                        the enabling into the respective initialization subroutines (Init_SysTick() and
//                        Init_UART5() ).  The peripherals are always enabled, so it did not make sense
//                        have them in EnDisPeripherals().
//                      - Added code to initialize the AFE Sync Offset registers for phase compensation in
//                        AFE_Init()
//                      - Added code to support AFE-ADC sample synchronization
//                          - Modified Init_TIM3() to initialize the counter to 6249.  This is half the
//                            auto-reload value.  Since the timer is started in the AFE interrupt, the first
//                            ADC interrupt will occur 1/2 sample time after the AFE interrupt.  Subsequent
//                            interrupts will be 1 sample time later, since the counter resets when it
//                            reaches the compare value.  This delays the ADC samples by one half-sample
//                            time.
//                          - Removed code that starts the ADC timer (Timer 3) from EnDisPeripherals().
//                            Timer 3 is started in the AFE ISR
//   0.14   160620  DAH - Re-added support for writing Frame EEPOT settings
//                          - EEPOT SPI initializations added to Init_SPI2()
//                      - Revised Init_DAC()
//   0.15   160718  DAH - Fixed minor bug in Init_SPI2()
//                      - Added support for the Black Box (SPI1) FRAM
//                          - Added Init_SPI1()
//                          - Revised Init_DMAController2() to add initializing Streams 2 and 3 for writing
//                            the samples to the Black Box FRAM via SPI1
//                      - Added support for basic CAM port testing
//                          - Revised Init_DMAController2() to add initializing Streams 5 and 7 for
//                            transmitting and receiving the samples to CAM1 via UART1
//                          - Revised Init_DMAController2() to add initializing Streams 1 and 6 for
//                            transmitting and receiving values to CAM2 via UART6
//                          - Added Init_CAM1_DMA_Streams() and Init_CAM2_DMA_Streams()
//                          - Revised Init_UART1() and Init_UART6() to change to 3.75Mbps, no interrupts,
//                            and DMA
//                          - Revised Init_InterruptStruct() to eliminate UART1 (CAM1) and UART6 (CAM2)
//                            interrupts, since DMA is used instead
//   0.16   160818  DAH - Added support for Startup Time measurement
//                          - Revised Init_ADC3() to enable the ADC
//                          - Removed ADC3 from EnDisPeripherals() since it is now always enabled
//                          - Deleted SysTick definitions (moved to Iod_def.h)
//   0.17   161031  DAH - Removed Timer 3 and ADC complete interrupts as they are no longer used (DMA is
//                        used instead).  Removed Modbus Rx and Tx interrupts as they are no longer used
//                        (Modbus will be moved to the Display Processor)
//                          - Revised Init_InterruptStruct()
//                      - Revised SPI2 to support the switch from the DS1390 RTC to the MCP7951 RTC
//                          - Revised Init_SPI2() and Init_RTC()
//   0.18   161115  DAH - Init_UART3() deleted (Modbus moved to the display processor).  Also removed Modbus
//                        control from EnDisPeripherals()
//                      - Added I2C3 code to support the Health Bus on rev 3 boards
//                          - Init_I2C3() added
//                          - Modified Init_InterruptStruct() to add I2C3 Rx/Tx and Error interrupts
//                      - Revised some comments (mainly subsystems and pin definitions) to match the rev 3
//                        board
//   0.19   170223  DAH - Revised Init_UART2() to increase speed from 9600 to 3.75M for Display comms
//                      - Revised UART2 (Display) communications to use DMA instead of interrupts
//                          - Init_UART2() revised
//                          - Init_InterruptStruct() revised to eliminate UART2 interrupts
//                          - Init_DMAController1() revised to move the AFE SPI3 Tx DMA stream from
//                            DMA1_Stream5 to DMA1_Stream7 to accomodate the UART2 Rx DMA stream and added
//                            added DMA1_Stream5 and DMA1_Stream6 for the UART2 (Display) interface
//   0.22   180420  DAH - Added support for energy and demand logging
//                          - Added InitFlashChip() to remove write protection from the demand sectors
//   0.24   180517  DAH - Updated InitFlashChip() to use defined constants for the write protection data
//   0.25   180621  DAH - Added Init_TIM4() to support SPI2 and Events handler on 1.5msec interrupts
//                      - Revised Init_InterruptStruct() to add TIM4 interrupt
//   0.29   190122  DAH - Added Init_TIM5() and Init_TIM8() to support coil temperature measurement
//                      - Revised Init_ADC3() to allow for initialization for both coil temperature
//                        measurement (multiple conversions driven by Timer 5) and other "one-time"
//                        conversions
//                      - Revised Init_ADC1() and Init_ADC2() to remove coil temperature measurement from
//                        the ADC1/ADC2 sequence
//                      - Revised Init_DMAController2() to reduce number of values transferred from 6 to 5
//                        since coil temperature no longer is read here (now read using ADC3)
//                      - Revised Init_TIM1(), Init_TIM3(), Init_TIM4(), and Init_TIM9() to clean up the
//                        register initialization
//   0.32   190726  DAH - Added Init_TIM2() to support Modbus messaging
//                      - Minor modification to Init_Tim1(), Init_Tim5(), Init_Tim8(), and Init_Tim9()
//   0.33   190823  DAH - Added support for frequency measurement
//                          - Revised Init_Tim9() to run on 1.5MHz clock (was 2MHz) to expand lower end of
//                            the frequency range.  Also changed to run on only with 120MHz system clock.
//                            Also changed to enable capture interrupts
//                          - Revised Init_InterruptStruct() to add Timer 9 capture interrupts
//                      - Minor modification to Init_Tim5() and Init_Tim8()
//   0.34   191002  DAH - Revised frequency measurement code for rev 4 boards.  Frequency measurement moved
//                        from Timer 9 to Timers 10 and 11.
//                          - Deleted Init_TIM9()
//                          - Added Init_TIM10() and Init_TIM11()
//                          - Revised EnDisPeripherals() to replace Timer 9 with Timer 10 and Timer 11
//                          - Revised Init_InterruptStruct() to replace Timer 9 with Timer 10 and Timer 11
//                      - Revised Init_TIM8() to leave clock disabled after it is set up.  Clock will be
//                        enabled when the temperature measurement is performed.
//                      - Revised Init_InterruptStruct() for rev 4 boards.  AFE Data Ready pin changed from
//                        PB8 to PD8.  Interrupt changed accordingly
//                      - Fixed bug in timer initialization subroutines (Init_TIMx()).  The counter values
//                        must be set after the update event is generated because the update event clears
//                        the counter!  In addition, must wait for the update event to complete before
//                        writing the counter.  If the counter is written before the update event completes,
//                        it will be cleared when the update event completes.
//                      - Revised pin documentation to match the rev 4 board's microprocessor package
//                      - Added Init_UART3()
//   0.35   191118  DAH - Revised Init_UART3() and Init_UART2() to change the baud rate from 3.75Mbps to
//                        3.0Mbps
//                      - Corrected bug in AFE_Init() so that phase calibration constants were written to
//                        the AFE
//   0.36   200210  DAH - Display_def.h renamed to DispComm_def.h
//                      - Display_ext.h renamed to DispComm_ext.h
//                      - In Init_DMAController1(), constant DISP_RXBUFSIZE renamed to DISPCOMM_RXBUFSIZE
//   0.37   200604  DAH - Removed ARCON Light interrupt from the interrupt structure.  PF10 was the input
//                        signal; it has long been repurposed.
//                          - Init_InterruptStruct() revised
//                      - Deleted Init_USB() since it is not used
//                      - Added include of Events_def.h for compilation purposes (for FRAM_Flash_def.h)
//   0.39   200729  DAH - Added test code to begin testing the internal RTC
//                          - Added matt_test[] and Init_Int_RTC()
//   0.42   201202  DAH - Cleaned up internal RTC code
//                          - Renamed Init_Int_RTC() to Init_IntRTC() and cleaned up the code
//                      - Deleted Init_RTC() as the external RTC is no longer used
//                      - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added include of RealTime_def.h
//   0.43   210115  DAH - Added support for 61850 communications with the Display Processor
//                          - Added initialization of Streams 1 and 3 to Init_DMAController1()
//                          - Added USART3 to EnDisPeripherals()
//                      - Revised Init_DMAController1() and Init_DMAController2() to clear the corresponding
//                        event flags (per RM0090 Reference Manual) before enabling the stream
//   0.49   220131  DAH - Revisions to get DMA working for Modbus transmissions
//                          - Revised Init_UART6() to enable DMA mode for transmission
//                          - Added input parameter to Init_UART6() to determine whether Rx interrupts are
//                            enabled or disabled
//                          - Added UART6 Rx interrupt to Init_InterruptStruct()
//   0.58   220829  DAH - Revisions to move Init_IntRTC() from the initialization code to the main loop
//                          - Revised Init_IntRTC() to use a state machine, as the watch crystal clock takes
//                            about 500msec to be stable
//   0.59   220831  DAH - Revised Init_SPI1() for operation with the serial Flash chip
//                      - Revised InitFlashChip() to use SPI1 instead of SPI2
//   42     230623  MAG - Revised Init_TIM4(), InitTIM10(), Init_InterruptStruct(), and EnDisPeripherals()
//                      - Deleted Init_TIM11()
//   44     230623  DAH - Deleted PG14 interrupt (was to be used for the Override interrupt).  An Override
//                        interrupt is detected in the AFE sampling interrupt, which occurs every 208usec.
//                        We do not need a separate interrupt for this input
//                          - Init_InterruptStruct() was revised
//   54     230802  BP  - Revised Init_I2C2() for Override micro comms
//   66     230825  DAH - Minor revision to Update_AFE_Sync_Regs() to eliminate compiler warning
//   69     230828  DAH - Added Time Sync (PH9) pin toggle interrupt to Init_InterruptStruct()
//  116     231129  MAG - Modified Init_UART6() with correct Modbus baud rates
//  117     231129  DAH - Revised Init_InterruptStruct() to change the AFE DRDY and Time Sync interrupt
//                        priority from 2 to 1 (highest priority)
//                      - Corrected bug in Init_InterruptStruct() for I2C3 interupt positions
//  142      240119 DAH - Revised AFE_Init() to support 50Hz and 60Hz phase cal constants when initializing
//                        the sync offset registers
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
//                                  System Initialization and Startup
//
// Reset --> Initialize with 16MHz internal clock:
//             Flash memory - instruction prefetch, instruction cache, data cache enabled
//                 (Init_FlashConfig()).  This provides high-speed performance, especially at the (eventual)
//                 120MHz system clock (HCLK).  Although there may not be much of an advantage to enabling
//                 the prefetch and caches with the 16MHz HSI clock, there does not appear to be a
//                 disadvantage to it.  Zero wait states.
//             System clocks - 16MHz HSI clock used for system clock.  HSE clock (25MHz) turned on and PLL
//                 set up, but not turned on (SysClkHandler()).  At the end of initialization, the HSE ready
//                 bit is checked.  According to Table 31 in the STM32F407xx data sheet (DocID022152), it
//                 takes about 2msec typ for the oscillator to come up.  However, we need to be able to run
//                 protection at 16MHz, in case the oscillator and PLL take longer to come up or don't come
//                 up at all.
//                      PCLK1 = 16MHz, APB1 timer clocks = 16MHz
//                      PCLK2 = 16MHz, APB2 timer clocks = 16MHz
//             ADC's - initialized with clock speed at PCLK/2.  This assumes 120MHz operation (PCLK2=30MHz).
//                  Will run slower at 16MHz (PCLK2=16MHz), but that is ok.
//             DAC - channels 1 and 2 are software triggered, but disabled (both are used to generate test
//                  signals).
//             I2C2 - used to communicate with the override micro.  Initialized assuming 120MHz system
//                  clock, but port is disabled.  Slave, 400KHz speed, interrupts enabled.
//             I2C3 - used for health bus communications.  Communicates with the on-board temp/humidity
//                    sensor HDC1010.  Initialized assuming 120MHz system clock, but port is disabled.
//                    Slave, 400KHz speed, interrupts enabled.
//             SPI1 - used to communicate with the serial Flash.  Initialized at fastest available speed -
//                  PCLK2/2 = 15MHz.  DMA operation enabled.
//             SPI2 - used to communicate to various peripheral ICs.  Configuration is device-dependent
//             SPI3 - used to communicate with the AFE.  Initialized at fastest available speed -
//                  PCLK1/2 = 15MHz.  DMA operation enabled.
//             UART1 - used to communicate with the CAM.  Initialized assuming 120MHz system clock, but UART
//                  is disabled
//             UART2 - used to communicate with the display processor.  Initialized assuming 120MHz system
//                  clock, but UART is disabled
//             UART3 - used to communicate with the display processor.  Initialized assuming 120MHz system
//                  clock, but UART is disabled
//             UART5 - used to provide test port communications.  9600 baud, no parity, one stop bit.  RXE
//                  interrupt enabled.
//             UART6 - used to communicate with the CAM.  Initialized assuming 120MHz system clock, but UART
//                  is disabled
//             Timer1 - used to drive the AFE clock input
//             Timer2 - used to measure the time between Modbus characters
//             Timer3 - used to trigger STM32's ADC sampling
//             Timer4 - used to trigger an interrupt for SPI2 event handling 1.5msec
//             Timer5 - used to trigger an ADC3 conversion every 250usec for coil temperature measurement
//             Timer8 - used to generate an external 4800Hz square wave for coil temperature measurement
//             Timer10 - used to measure load frequency
//             Timer11 - used to measure line frequency
//             SysTick - generic timer, initialized for 10msec tick

//  If it is ready, turn on the PLL with the HSE as the clock input.  If it is
//                 not, we will check later.
//                                  System Clock Configuration
// The HSE is enabled and the PLL is set up to run from the HSE clock.  The PLL is set to operate at 120MHz.
// Until the PLL is ready, the system operates in a "bare-bones" state:
//   80 samples per cycle
//   Basic overcurrent protection - instantaneous, short-delay, and long-delay
//   No metering or advanced protection
//   LED interface and switches
//   No USB or other communications
//   No display
// Once the PLL is ready, the system reconfigures to operate at 120MHz.  The APB1 and APB2 peripheral clocks
// are set to 30MHz, and therefore the APB1 and APB2 timer clocks are 60MHz.  The internal flash is set for
// 3 wait states.
// After the switch to 120MHz is made, the system operates in normal operation:
//   80 samples per cycle
//   All protection functions
//   Metering
//   LED interface and switches
//   USB and all other communications
//   Display
// The PLL should be ready no longer than 10msec after it is enabled.  If the PLL is not ready after 20msec,
// the RED error LED is flashed, but the system continues to operate in bare-bones mode.






//------------------------------------------------------------------------------------------------------------
//                   Definitions
//------------------------------------------------------------------------------------------------------------
//
//      Global Definitions from external files...
//
//
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "Init_def.h"
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Meter_def.h"
#include "Intr_def.h"
#include "CAMCom_def.h"
#include "DispComm_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Modbus_def.h"


//
//      Local Definitions used in this module only...
//
//      Flash Memory Controller
#define ACR_BYTE0_ADDRESS           ((uint32_t)0x40023C00) 
#define __HAL_FLASH_SET_LATENCY(__LATENCY__) (*(__IO uint8_t *)ACR_BYTE0_ADDRESS = (uint8_t)(__LATENCY__))

#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field. */



//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Iod_ext.h"
#include "Meter_ext.h"
#include "Intr_ext.h"
#include "CAMCom_ext.h"
#include "DispComm_ext.h"
#include "RealTime_ext.h"
#include "Setpnt_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Init_FlashConfig(void);
void SysClkHandler(void);
void Init_GPIO();
void Init_ADC1(void);
void Init_ADC2(void);
void Init_ADC3(uint8_t normal);
void Init_DAC(void);
void Init_SPI1(uint8_t config);
void Init_SPI2(uint8_t Device);
void Init_SPI3(void);
void Init_I2C2(void);
void Init_I2C3(void);
void Init_UART1(void);
void Init_UART2(void);
void Init_UART3(void);
void Init_UART6(uint8_t rx_int_enabled);
void Init_InterruptStruct(void);
void AFE_Init(void);
void Init_DMAController1(void);
void Init_DMAController2(void);
void Init_CAM1_DMA_Streams(void);
void Init_CAM2_DMA_Streams(void);
void Init_TIM1(void);
void Init_TIM2(void);
void Init_TIM3(void);
void Init_TIM4(void);
void Init_TIM5(void);
void Init_TIM8(void);
void Init_TIM10(void);
void InitFlashChip(void);
void Init_IntRTC(void);



//      Local Function Prototypes (These functions are called only within this module)
//
void Init_UART5(void);
void Init_SysTick(void);
void EnDisPeripherals(void);

#define __GPIOI_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOIEN))
#define __GPIOF_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOFEN))
#define __GPIOG_CLK_ENABLE()            (RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOGEN))
#define __USART3_CLK_ENABLE()           (RCC->APB1ENR |= (RCC_APB1ENR_USART3EN))
#define __UART5_CLK_ENABLE()            (RCC->APB1ENR |= (RCC_APB1ENR_UART5EN))
#define __DAC_CLK_ENABLE()              (RCC->APB1ENR |= (RCC_APB1ENR_DACEN))
#define __ADC2_CLK_ENABLE()             (RCC->APB2ENR |= (RCC_APB2ENR_ADC2EN))
#define __ADC3_CLK_ENABLE()             (RCC->APB2ENR |= (RCC_APB2ENR_ADC3EN))


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
uint8_t SysClkState;                    // Note, this is initialized in the startup code (Reset_Handler in 
                                        //   startup_stm32f407xx.s)
uint8_t SysClk_120MHz;                  // Note, this is initialized in State 0 of SysClkHandler() and so
                                        //   does not need to be initialized

//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t fred1[101];     // *** DAH TEST TO READ BACK AFE REGISTERS

//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
#define ADC_CLOCKPRESCALER_PCLK_DIV2    ((uint32_t)0x00000000)
#define ADC_RESOLUTION12b               ((uint32_t)0x00000000)

const uint16_t SPI3_READAFE = 0x8000;


//
//                                  Subsystems and Pin Definitions
//
//                             Chart Key
//             Mode: 0=Input, 1=Output, 2=Alternate, 3=Analog
//             Speed: 0=Low, 1=Medium, 2=Fast, 3=High
//             Type: 0=Push-Pull, 1=Open-Drain
//             PUPDR: 0=None, 1=Pull-up, 2=Pull-down, 3=Reserved
//

// Subsystem: AFE Sampling Conversions
// Purpose:   Convert phase current and line voltage signals
// Resource - Initialization subroutine - Function:
//    Timer1 - Init_TIM1() - Generates the AFE clock
//    SPI3 - Init_SPI3() - SPI interface used to communicate with the AFE (analog front end)
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       119   PA8        AF1 - TIM1 CH1 OUT  2      0     0     0    AFE CLOCK
//
//       163   PB5        AF6 - SPI3 MOSI     2      2     0     0    SPI3 MOSI (AFE)
//       140   PC11       AF6 - SPI3 MISO     2      2     0     0    SPI3 MISO (AFE)
//       139   PC10       AF6 - SPI3 SCK      2      2     0     0    SPI3 SCK (AFE)
//       169   PE0        GPIO - OUT_PP_PU    1      0     0     1    AFE_STARTN
//        96   PD8        GPIO - IN_PU        0      0     0     1    AFE_DATA_READYN
//        18   PF2        GPIO - OUT_PP_PU    1      0     0     0    AFE_RESETN (has ext pull-down)


// Subsystem: ADC Sampling Conversions
// Purpose:   Convert phase current and load voltage signals
// Resource - Initialization subroutine - Function:
//    ADC1 - Init_ADC1() - Converts phase current and load voltage signals
//    ADC2 - Init_ADC2() - Converts phase current and load voltage signals
//    Timer3 - Init_TIM3() - Triggers ADC1 and ADC2 conversions
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        41   PA1        ADC1_IN1            3      0     0     0    Vn_Line Analog Input
//        35   PC3        ADC1_IN13           3      0     0     0    In Analog Input
//        34   PC2        ADC1_IN12           3      0     0     0    Ic Analog Input     
//        33   PC1        ADC1_IN11           3      0     0     0    Ib Analog Input
//        32   PC0        ADC1_IN10           3      0     0     0    Ia Analog Input
//
//        56   PB0        ADC2_IN8            3      0     0     0    Vc_Line Analog Input
//        55   PC5        ADC2_IN15           3      0     0     0    Vb_Line Analog Input
//        54   PC4        ADC2_IN14           3      0     0     0    Va_Line Analog Input
//        57   PB1        ADC2_IN9            3      0     0     0    1.25V Reference Analog Input
//        47   PA3        ADC2_IN3            3      0     0     0    Thermal Memory Analog Input
//
//       117   PC8        GPIO - OUT_PP_NP    1      0     0     0    ST_Low_Gain (has ext pull-up)


// Subsystem: Coil Temperature Measurement
// Purpose:   Generate coil temperature measurement excitation signal and measure the input signal
// Resource - Initialization subroutine - Function:
//    Timer8 - Init_TIM8() - Generate an external 4800Hz square wave that is used in measuring the Rogowski
//         coil temperature
//    ADC3 - Init_ADC3() - Converts coil temperature input signal
//    Timer5 - Init_TIM5() - Triggers ADC3 conversion
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       128   PH13       AF3 - TIM8 CH1 OUT  2      0     0     0    4P8K_INJ
//        77   PE14       GPIO - OUT_PP_PD    1      0     0     2    4P8K_INJ_EN
//        76   PE13       GPIO - OUT_PP_PD    1      0     0     2    INJ_BYPASS_EN
//
//        75   PE12       GPIO - OUT_PP_PU    1      0     0     1    TMP_CH_EN
//        74   PE11       GPIO - OUT_PP_PD    1      0     0     2    TMP_CH_A1
//        73   PE10       GPIO - OUT_PP_PD    1      0     0     2    TMP_CH_A0
//
//        42   PA2        ADC3_IN2            3      0     0     0    Coil Temp Input


// Subsystem: Miscellaneous ADC Conversions
// Purpose:   Convert miscellaneous voltage inputs
// Resource - Initialization subroutine - Function:
//    ADC3 - Init_ADC3() - Converts miscellaneous analog inputs
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        47   PA3        ADC3_IN3            3      0     0     0    Thermal Memory Analog Input
//        19   PF3        ADC3_IN9            3      0     0     0    TA_SENSE Analog Input
//        20   PF4        ADC3_IN14           3      0     0     0    BATTERY SENSE Analog Input
//        21   PF5        ADC3_IN15           3      0     0     0    AUX PWR SENSE Analog Input
//        24   PF6        ADC3_IN4            3      0     0     0    DISPLAY_2.8V_MONITOR Analog Input
//        26   PF8        ADC3_IN6            3      0     0     0    4.5V SUPPLY SENSE Analog Input
//        27   PF9        ADC3_IN7            3      0     0     0    START TIME Analog Input
//        28   PF10       ADC3_IN8            3      0     0     0    1.25V CHECK Analog Input


// Subsystem: On-board Test Injection
// Purpose:   Generates test signals to simulate trip conditions for on-board trip testing
// Resource - Initialization subroutine - Function:
//    DAC - Init_DAC() - Generate test injection signal
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        50   PA4        DAC - OUT1          3      0     0     0    DAC1 Analog Output


// Subsystem: Miscellaneous Peripheral Communications
// Purpose:   Communicate with various on-board peripheral chips
// Resource - Initialization subroutine - Function:
//    DAC - Init_DAC() - SPI interface used to communicate with various peripherals
//    Timer4 - Init_TIM4() - Trigger 1.5msec interrupts for SPI2 and event handling
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        95   PB15       AF5 - SPI2 MOSI     2      0     0     0    SPI2 MOSI (GENERAL)
//        94   PB14       AF5 - SPI2 MISO     2      0     0     0    SPI2 MISO (GENERAL)
//        93   PB13       AF5 - SPI2 SCK      2      0     0     0    SPI2 SCLK (GENERAL)
//       157   PG14       GPIO - OUT_PP_PU    1      0     0     1    SPI2_SB_MICRO_EN
//       110   PG6        GPIO - OUT_PP_NP    1      0     0     0    SPI2_FLASH_CEN (has ext pull-up)
//        43   PH2        GPIO - OUT_PP_PU    1      0     0     0    SPI2_FRAM_CEN (has ext pull-up)
//        44   PH3        GPIO - OUT_PP_NP    1      0     0     0    SPI2_RTC_CEN (has ext pull-up)
//       173   PI4        GPIO - OUT_PP_NP    1      0     0     0    SPI2_FRAME_CEN (external pull-up)
//       134   PI3        GPIO - OUT_PP_PU    1      0     0     1    SPI2_MCR_CEN


// Subsystem: CAM1 Communications
// Purpose:   Communicate with a high-speed communications adapter module (CAM)
// Resource - Initialization subroutine - Function:
//    UART1 - Init_UART1() - Communicate with a high-speed communications adapter module (CAM)
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       121   PA10       AF7 - USART1 RX     2      0     0     0    UART1 RX (CAM1)
//       164   PB6        AF7 - USART1 TX     2      0     0     0    UART1 TX (CAM1)


// Subsystem: CAM2/Modbus Communications
// Purpose:   Communicate with a high-speed communications adapter module (CAM) or Modbus master
// Resource - Initialization subroutine - Function:
//    UART6 - Init_UART6() - CAM or Modbus communications
//    Timer2 - Init_TIM2() - Measure the time between received Modbus characters
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       116   PC7        AF8 - USART6 RX     2      0     0     0    UART6 RX (CAM2)
//       115   PC6        AF8 - USART6 TX     2      0     0     0    UART6 TX (CAM2)


// Subsystem: Display Microprocessor Communications
// Purpose:   Communicate with the display microprocessor
// Resource - Initialization subroutine - Function:
//    UART2 - Init_UART2() - Normal communications with the display microprocessor
//    UART3 - Init_UART3() - GOOSE or Sampled Value communications with the display microprocessor
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       150   PD6        AF7 - USART2 RX     2      0     0     1    UART2 RX (DISPLAY)
//       147   PD5        AF7 - USART2_TX     2      0     0     1    UART2 TX (DISPLAY)
//
//        80   PB11       AF7 - USART3_RX     2      0     0     1    UART3 RX (Display processor)
//        79   PB10       AF7 - USART3_TX     2      0     0     1    UART3 TX (Display processor)


// Subsystem: Override Microprocessor Communications
// Purpose:   Communicate with the override microprocessor
// Resource - Initialization subroutine - Function:
//    I2C - Init_I2C2() - Communicate with the override microprocessor
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        46   PH5        AF4 - I2C2_SDA      2      0     1     0    I2C2_SDA (Override Micro)
//        45   PH4        AF4 - I2C2_SCL      2      0     1     0    I2C2_SCL (Override Micro)


// Subsystem: Health Bus Communications
// Purpose:   Communicate with the health bus devices
// Resource - Initialization subroutine - Function:
//    I2C - Init_I2C3() - Communicate with the health bus microprocessor
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        85   PH8        AF4 - I2C3_SDA      2      0     1     0    I2C3_SDA (Health Bus)
//        84   PH7        AF4 - I2C3_SCL      2      0     1     0    I2C3_SCL (Health Bus)


// Subsystem: Test Port Communications
// Purpose:   Communicate with a dumb terminal (such as Hyperterminal on a PC) for internal testing
// Resource - Initialization subroutine - Function:
//    UART5 - Init_UART5() - Test port communications
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       141   PC12       AF8 - UART5 TX      2      0     0     1    UART5 TX (TEST PORT)
//       144   PD2        AF8 - UART5 RX      2      0     0     1    UART5 RX (TEST PORT)     


// Subsystem: Load Frequency Measurement
// Purpose:   Measure the load frequency
// Resource - Initialization subroutine - Function:
//    Timer10 - Init_TIM10() - Measure the load frequency
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//       167   PB8        AF3 - TMR10 CH1     2      0     0     0    uC_Freq_Load


// Subsystem: Line Frequency Measurement
// Purpose:   Measure the line frequency
// Resource - Initialization subroutine - Function:
//    Timer11 - Init_TIM11() - Measure the line frequency
// Pin Resources:
//       Pin   Resource   Micro Func        Mode  Speed  Type  PUPDR  PXR35 Function
//        25   PF7        AF3 - TMR11 CH1     2      0     0     0    uC_Freq_Line


// Subsystem: Internal Timekeeping
// Purpose:   Internal generic timekeeping (non-critical, low-accuracy functions)
// Resource - Initialization subroutine - Function:
//    SysTick - Init_SysTick() - Generic system timer
// Pin Resources:
//       None


// *** DAH MAKE SURE ALL PERIPHERALS ARE DISABLED BEFORE CHANGING THEM - WE MAY CALL THIS INITIALIZATION ROUTINE
//     IN THE CODE IN CASE REINITIALIZING THE PORT DUE TO ERRORS, ALSO MAY WANT TO TURN THE CLOCKS ON IN THE ROUTINES
//     FOR THE SAME REASON



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_FlashConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Flash Configuration Initialization
// 
//  MECHANICS:          This subroutine enables the Flash prefetch, instruction cache, and data cache to
//                      speed system performance.
//
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             Flash access control register
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_FlashConfig(void)
{
  // Configure the Flash prefetch, instruction cache, and data cache
  FLASH->ACR |= (FLASH_ACR_ICEN + FLASH_ACR_DCEN + FLASH_ACR_PRFTEN);
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_FlashConfig()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        SysClkHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           System Clock Handler
// 
//  MECHANICS:          This subroutine initializes the system clock from a power up or reset, then checks
//                      the clock to ensure it is operating properly.
//                      After a reset, the state variable (SysClkState) is set to 0.  The HSI clock is
//                      automatically used as the system clock (SYSCLK).  In State 0, the clock system is
//                      initialized.
//                        State 0:
//                          System clock is set to the HSI (internal 16MHz clock)
//                          SYSCLK operates at 16MHz
//                          Internal flash has zero wait states
//                          HCLK is 16MHz
//                          48MHz CLOCK IS NOT AVAILABLE - USB AND RANDOM NUMBER GENERATOR WILL NOT WORK!!!
//                          Peripheral and timer clocks (PCLK1 and PCLK2) are set to 16MHz (the maximum
//                              frequency at HCLK = 16MHz)
//                          HSE (external 25MHz clock) is enabled, but the system clock is NOT set to use it
//                          PLL is set up to run from the HSE clock.  The PLL is set for 120MHz.  The PLL is
//                              NOT turned on, because the HSE is not ready and it will take longer for the
//                              PLL to track the HSE as it comes up.
//                          The state variable (SysClkState is set to 1
//                      In State 1, the subroutine checks the status of the HSE.
//                        State 1:
//                          If the HSE is ready, turn on the PLL and advance the state
//                          Otherwise remain in this state
//                      At this point, the 120MHz PLL is coming up, but we are still operating from the
//                      16MHz HSI clock.  In State 2, the subroutine checks the status of the PLL.  If it is
//                      ready, switch to this clock.
//                        State 2:
//                          If the PLL is ready, switch from the HSI clock to the PLL clock
//                          
//                      Reference Section 7.3 of the Programmer's Reference Manual (RM0090), and Section 4
//                      of the Programming Manual (PM0214).
//                          
//  CAVEATS:            It is assumed interrupts are disabled when this subroutine is called
// 
//  INPUTS:             SysClkState
// 
//  OUTPUTS:            SysClk_120MHz - set to TRUE if system clock is 120MHz (PCLK2 = 30MHz)
//                                    - set to FALSE if system clock is 16MHz (PCLK2 = 16MHz)
//
//  ALTERS:             Reset and clock control registers, SysClkState
// 
//  CALLS:              Init_UART5(), Init_SysTick()
// 
//------------------------------------------------------------------------------------------------------------

void SysClkHandler(void)
{
  uint8_t SCH_exit;

  SCH_exit = FALSE;

  while (!SCH_exit)
  {
    SCH_exit = TRUE;                    // Normally will exit after one case statement is executed

    switch (SysClkState)
    {
      case 0:                           // Reset state - Initialize using HSI.  Note, this state is entered
      default:                          //   either after a reset or if the PLL drops out
        // Initialize the floating point unit (coprocessor) to full access (ref PM0214 section 4.6.1)
        // b31..24 = 0: Reserved
        // b23..22 = 3: Full access for coprocessor 1
        // b21..20 = 3: Full access for coprocessor 0
        // b19..0 = 0: Reserved
        SCB->CPACR = 0x00F00000;

        // Initialize the vector table offset register (ref PM0214 section 4.4.4)
        // b31..30 = 0: Reserved must be 0
        // b29 = 0: Vector table is in program memory
        // b28..9: Bits 28:9 of the offset (from 0) address of the vector table (bits 8..0 are 0)
        // b8..0 = 0: Reserved must be 0
        SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */

        // Enable the power interface
        RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
    
        // Initialize the power control register
        // b31..15 = 0: Reserved
        // b14 = 1: Scale 1 mode (normal)
        // b13..10 = 0: Reserved
        // b9 = 0: Flash memory is not in power-down mode when the device is in Stop mode
        // b8 = 0: Access to RTC and RTC Backup registers is disabled
        // b7..5 = 0: PVD level is 2.0V (PVD is not used)
        // b4 = 0: PVD is disabled
        // b3 = 0: Clear standby flag is not active
        // b2 = 0: Clear wakeup flag is not active
        // b1 = 0: Enter Stop mode when CPU enters deep sleep
        // b0 = 0: Voltage regulator is ON during Stop mode
        PWR->CR = 0x00004000;
    
        // Turn on HSI and HSE oscillators.  Turn off other sources.  Make sure trim values are unchanged
        // b31..28 = 0: Reserved, must be kept at reset value (per RM0090)
        // b27 = 0: PLLI2S unlocked flag (read-only)
        // b26 = 0: PLLI2S off
        // b25 = 0: PLL unlocked flag (read-only)
        // b24 = 0: PLL off
        // b23..20 = 0: Reserved, must be kept at reset value (per RM0090)
        // b19 = 0: Clock security system off
        // b18 = 0: HSE oscillator not bypassed
        // b17 = 0: HSE ready flag (read-only)
        // b16 = 1: HSE oscillator on
        // b15..8 = x: HSI calibration, Initialized automatically at startup (read-only)
        // b7..3 = x10: Reset value of HSI trim
        // b2 = 0: Reserved, must be kept at reset value (per RM0090)
        // b1 = 1: HSI oscillator is ready flag (read-only)
        // b0 = 1: HSI oscillator is on (reset value)
        RCC->CR &= 0xFAF2FFFF;                        // Make sure b26, 24, 19, 18 are 0
        RCC->CR |= (RCC_CR_HSEON | RCC_CR_HSION);     // Make sure b16, 0 are 1
    
        // Configure the PLL.  PLL clock is HSE/25*240/2 for SYSCLK, HSE/25*240/5 for 48MHz clock
        // b31..28 = 0: Reserved
        // b27..24 = 5: PLLQ is 5
        // b23 = 0: Reserved
        // b22 = 1: PLL source input is HSE clock
        // b21..18 = 0: Reserved
        // b17..16 = 0: PLLP is 2
        // b15 = 0: Reserved
        // b14..6 = 240: PLLN is 240
        // b5..0 = 25: PLLM is 25
        RCC->PLLCFGR = 0x05403C19;
    
        // Configure the Clock.  Clock source is HSI.  APB1 and APB2 peripheral clocks are 16MHz.  APB1 and
        //   APB2 timer clocks are 32MHz.
        // b31..30 = 0: Microcontroller clock output 2 is SYSCLK
        // b29..27 = 0: MCO2 prescaler is no division
        // b26..24 = 0: MCO1 prescaler is no division
        // b23 = 0: PLLI2Sis I2S source (not used)
        // b22..21 = 0: Microcontroller clock output 1 is HSI
        // b20..16 = 0: No division for RTC clock (not used)
        // b15..13 = 0: APB2 prescaler is 1 (no division)
        // b12..10 = 0: APB1 prescaler is 1 (no division)
        // b9..8 = 0: Reserved
        // b7..4 = 0: AHB prescaler is 1 (no division)
        // b3..2 = 0: SYSCLK source (read-only)
        // b1..0 = 0: SYSCLK is HSI
        RCC->CFGR = 0x00000000;
    
        // Set the number of wait states  to 0 since clock speed is now 16MHz
        __HAL_FLASH_SET_LATENCY(0);
    
        // The following interfaces are initialized differently depending on the system clock (SYSCLK) value
        //   For SYSCLK = 16MHz, PCLK1 and PCLK2 are 16MHz.  For SYSCLK = 120MHz, PCLK1 and PCLK2 are 30MHz.
        //     UART5 - interface to test port, setup is dependent on the clock speeds
        //     SysTick - generic system timer, setup is dependent on the clock speeds
        // These peripherals are presently disabled.  We do not want pending interrupts until the system is
        //   prepared to handle them.
        SysClk_120MHz = FALSE;                        // Set flag indicating system clock speed is 16MHz
        Init_UART5();
        Init_SysTick();
        // Enable and disable appropriate peripherals based on SYSCLK.  Remember, we may have entered this
        //   state from State 2, and the PLL has quit operating.  Therefore, we need to make sure the
        //   120MHz-only peripherals are disabled
        EnDisPeripherals();
    
        SysClkState++;
//        break;                             Fall into the next state to turn on the HSE
    
      case 1:                             // Check whether HSE is ready
        RCC->CR |= RCC_CR_HSEON;                          // Make sure the HSE is on
        if ((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY)   // If HSE is ready, turn on the PLL and advance the
        {                                                 //   state
          RCC->CR |= RCC_CR_PLLON;
          SysClkState++;
        }                                                 // Otherwise remain in this state
        break;                                            // Exit in either case
    
      case 2:                             // Check whether PLL is ready
        RCC->CR |= (RCC_CR_PLLON | RCC_CR_HSEON);     // Make sure the PLL and HSE are on
    
        // If PLL is ready, switch SYSCLK from the HSI (16MHz) to the PLL (120MHz)
        if ((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY)
        {
          // First disable the peripherals that are running and are configured for 16MHz operation.  This
          //   ensures these peripherals will not operate at the wrong frequency after the clock is changed
          UART5->CR1 &= 0xFFFFDFFF;
          SysTick->CTRL &= 0xFFFFFFFE;
    
          // Now switch to the PLL...
    
          SysClk_120MHz = TRUE;                       // Assume the switch to the PLL will be successful
          SysClkState = 3;
    
          __HAL_FLASH_SET_LATENCY(3);                 // Set the number of wait states  to 3 (per Table 10 in
                                                      //   RM0090 - assumes Vdd = 2.7 - 3.6V)
          if ((FLASH->ACR & FLASH_ACR_LATENCY) != 3)  // If the wait states didn't take, abort - this was
          {                                           //   recommended in Section 3.5.1 of RM0090.  Don't set
                                                      //   latency back to 0.  Hopefully, it will eventually
                                                      //   switch to 3
            // *** DAH NEED TO INCREMENT ERROR COUNTER AND/OR SET FLAG TO LIGHT RED ERROR LED
            SysTick->CTRL |= 0x00000001;              // Reenable the peripherals that were turned off      
            UART5->CR1 |= 0x00002000;

            SysClk_120MHz = FALSE;                    // Cannot switch to PLL so set flag back to 16MHz
            SysClkState = 2;                          //   operation and remain in this state
          }
    
          if (SysClk_120MHz == TRUE)      // If ok so far...
          {
            // Configure the Clock.  Clock source is PLL.  APB1 and APB2 peripheral clocks are 30MHz.  APB1
            //   and APB2 timer clocks are 60MHz.
            // b31..30 = 0: Microcontroller clock output 2 is SYSCLK
            // b29..27 = 0: MCO2 prescaler is no division
            // b26..24 = 0: MCO1 prescaler is no division
            // b23 = 0: PLLI2Sis I2S source (not used)
            // b22..21 = 3: Microcontroller clock output 1 is PLL
            // b20..16 = 0: No division for RTC clock (not used)
            // b15..13 = 101b: APB2 prescaler is 4
            // b12..10 = 101b: APB1 prescaler is 4
            // b9..8 = 0: Reserved
            // b7..4 = 0: AHB prescaler is 1 (no division)
            // b3..2 = 0: SYSCLK source (read-only)
            // b1..0 = 2: SYSCLK is PLL
            RCC->CFGR = 0x0060B402;
            // If clock source is not PLL, abort - this was recommended in Section 3.5.1 of RM0090.  Note,
            //   according to the ST Micro FAE (contacted on 150701), it should take less than a clock cycle for
            //   the actual switch of the clock sources to occur.  Therefore, there doesn't need to be a delay
            // between the command to switch the clock source, and the command to check the clock source.
            if ((RCC->CFGR & 0x0000000C) != 0x00000008)
            {
              RCC->CFGR = 0x00000000;                           // Switch back to the HSI
              SysClk_120MHz = FALSE;                            // Cannot switch to PLL so set flag back to
              SysClkState = 2;                                  //   16MHz operation and remain in this state
              // *** DAH NEED TO INCREMENT ERROR COUNTER AND/OR SET FLAG TO LIGHT RED ERROR LED
            }
            if (SysClk_120MHz == TRUE)                    // If SYSCLK switched to 120MHz, reinitialize the
            {                                             //   the peripherals that had been configured for
              Init_UART5();                               //   for SYSCLK = 16MHz
              Init_SysTick();
            }
            EnDisPeripherals();                           // Enable and disable appropriate peripherals based
          }                                               //   on SYSCLK
        }                                 // If PLL is not ready, remain in this state and exit
        break;
    
      case 3:                             // Check to make sure PLL is still operating
        if ((RCC->CFGR & 0x0000000C) != 0x00000008)       // If clock source is not PLL, abort - jump back to
        {                                                 //   state 0 to switch back to the HSI clock
          // *** DAH NEED TO SET FLAG TO LIGHT RED ERROR LED
          SysClkState = 0;
          SCH_exit = FALSE;
        }
        break;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        SysClkHandler()
//------------------------------------------------------------------------------------------------------------



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_GPIO()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           GPIO Pins Initialization
// 
//  MECHANICS:          This subroutine initializes the general-purpose I/O port pins.  This subroutine also
//                      enables the clocks for the GPIO ports.
//
//                      Reference Section 8.4 of the Programmer's Reference Manual (RM0090).
//                      Reference Init_def.h for the I/O configuration.
//                          
//  CAVEATS:            Call only during initialization.
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             GPIO configuration registers for ports A, B, D, E, F, G, H, I
//                      RCC clock configuration registers for the GPIO clocks
// 
//  CALLS:              None
//
//  EXECUTION TIME:     Measured on 180627 (rev 0.25 code): 251 cyc/16MHz = 15.7usec
// 
//------------------------------------------------------------------------------------------------------------

void Init_GPIO()
{

  // Enable the GPIO clocks
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOI_CLK_ENABLE();

  // Configure the ports

  //--------------------------------------------------------------------------------------------------------
  // REFERENCE INIT_DEF.H FOR THE PIN CONFIGURATION CHART
  //--------------------------------------------------------------------------------------------------------

  // Write ODR register before mode register so desired output state is set before switching pin to an output
  GPIOA->ODR = GPIOA_ODR_DEF;
  GPIOA->MODER = GPIOA_MODE_DEF;
  GPIOA->OSPEEDR = GPIOA_SPEED_DEF;
  GPIOA->OTYPER = GPIOA_TYPE_DEF;
  GPIOA->PUPDR = GPIOA_PUPDR_DEF;
  GPIOA->AFR[0] = GPIOA_AFRLOW_DEF;
  GPIOA->AFR[1] = GPIOA_AFRHI_DEF;

  GPIOB->ODR = GPIOB_ODR_DEF;
  GPIOB->MODER = GPIOB_MODE_DEF;
  GPIOB->OSPEEDR = GPIOB_SPEED_DEF;
  GPIOB->OTYPER = GPIOB_TYPE_DEF;
  GPIOB->PUPDR = GPIOB_PUPDR_DEF;
  GPIOB->AFR[0] = GPIOB_AFRLOW_DEF;
  GPIOB->AFR[1] = GPIOB_AFRHI_DEF;

  GPIOC->ODR = GPIOC_ODR_DEF;
  GPIOC->MODER = GPIOC_MODE_DEF;
  GPIOC->OSPEEDR = GPIOC_SPEED_DEF;
  GPIOC->OTYPER = GPIOC_TYPE_DEF;
  GPIOC->PUPDR = GPIOC_PUPDR_DEF;
  GPIOC->AFR[0] = GPIOC_AFRLOW_DEF;
  GPIOC->AFR[1] = GPIOC_AFRHI_DEF;

  GPIOD->ODR = GPIOD_ODR_DEF;
  GPIOD->MODER = GPIOD_MODE_DEF;
  GPIOD->OSPEEDR = GPIOD_SPEED_DEF;
  GPIOD->OTYPER = GPIOD_TYPE_DEF;
  GPIOD->PUPDR = GPIOD_PUPDR_DEF;
  GPIOD->AFR[0] = GPIOD_AFRLOW_DEF;
  GPIOD->AFR[1] = GPIOD_AFRHI_DEF;

  GPIOE->ODR = GPIOE_ODR_DEF;
  GPIOE->MODER = GPIOE_MODE_DEF;
  GPIOE->OSPEEDR = GPIOE_SPEED_DEF;
  GPIOE->OTYPER = GPIOE_TYPE_DEF;
  GPIOE->PUPDR = GPIOE_PUPDR_DEF;
  GPIOE->AFR[0] = GPIOE_AFRLOW_DEF;
  GPIOE->AFR[1] = GPIOE_AFRHI_DEF;

  GPIOF->ODR = GPIOF_ODR_DEF;
  GPIOF->MODER = GPIOF_MODE_DEF;
  GPIOF->OSPEEDR = GPIOF_SPEED_DEF;
  GPIOF->OTYPER = GPIOF_TYPE_DEF;
  GPIOF->PUPDR = GPIOF_PUPDR_DEF;
  GPIOF->AFR[0] = GPIOF_AFRLOW_DEF;
  GPIOF->AFR[1] = GPIOF_AFRHI_DEF;

  GPIOG->ODR = GPIOG_ODR_DEF;
  GPIOG->MODER = GPIOG_MODE_DEF;
//  GPIOH->MODER |= (1<<15);
//  GPIOH->MODER |= (1<<17);

  GPIOG->OSPEEDR = GPIOG_SPEED_DEF;
  GPIOG->OTYPER = GPIOG_TYPE_DEF;
  GPIOG->PUPDR = GPIOG_PUPDR_DEF;
  GPIOG->AFR[0] = GPIOG_AFRLOW_DEF;
  GPIOG->AFR[1] = GPIOG_AFRHI_DEF;

  GPIOH->ODR = GPIOH_ODR_DEF;
  GPIOH->MODER = GPIOH_MODE_DEF;
  GPIOH->OSPEEDR = GPIOH_SPEED_DEF;
  GPIOH->OTYPER = GPIOH_TYPE_DEF;
//  GPIOH->OTYPER |= (1<<7);
//  GPIOH->OTYPER |= (1<<8);

  GPIOH->PUPDR = GPIOH_PUPDR_DEF;
  GPIOH->PUPDR |= (1<<14);
  GPIOH->PUPDR |= (1<<16);
  GPIOH->AFR[0] = GPIOH_AFRLOW_DEF;
  GPIOH->AFR[1] = GPIOH_AFRHI_DEF;

  GPIOI->ODR = GPIOI_ODR_DEF;
  GPIOI->MODER = GPIOI_MODE_DEF;
  GPIOI->OSPEEDR = GPIOI_SPEED_DEF;
  GPIOI->OTYPER = GPIOI_TYPE_DEF;
  GPIOI->PUPDR = GPIOI_PUPDR_DEF;
  GPIOI->AFR[0] = GPIOI_AFRLOW_DEF;
  GPIOI->AFR[1] = GPIOI_AFRHI_DEF;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_GPIO()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_ADC1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           ADC1 Initialization
// 
//  MECHANICS:          This subroutine initializes Analog to Digital Converter #1
//                      ADC1 and ADC2 are configured in dual simultaneous mode (ref AN3116, March 2010,
//                      from ST Micro).  In this mode, the two ADCs are synchronized to perform two
//                      conversions simultaneously.  Each ADC converts a channel sequence as follows:
//                                        ADC1                                 ADC2
//                          Seq Num   Pin   Input     Channel          Pin  Input    Channel
//                             1      PC0   Ia        ADC1_IN10        PC4  Va_Line  ADC2_IN14
//                             2      PC1   Ib        ADC1_IN11        PC5  Vb_Line  ADC2_IN15
//                             3      PC2   Ic        ADC1_IN12        PB0  Vc_Line  ADC2_IN8
//                             4      PC3   In        ADC1_IN13        PB1  1.25Vref ADC2_IN9
//                             5      PA1   Vn_Line   ADC1_IN1         PA3  Therm Mem  ADC2_IN3
//                      ADC1 is configured as follows:
//                          Resolution = 12 bits
//                          Clock is PCLK2/2
//                                  = 30MHz/2 = 15MHz for PCLK2 = 30MHz (SYSCLK = 120MHz) 
//                                  = 16MHz/2 = 8MHz for PCLK2 = 16MHz (SYSCLK = 16MHz) 
//                          Conversion time = 3 (sampling) + 12 (conversion) = 15 cycles
//                                  = 1.0usec/channel for PCLK2 = 30MHz (SYSCLK = 120MHz)
//                                  = 1.875usec/channel for PCLK2 = 16MHz (SYSCLK = 16MHz)
//                      Reference Section 13.13 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             ADC1 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_ADC1(void)
{

  __ADC1_CLK_ENABLE();                      // Enable the clock

  ADC1->CR2 &= 0xFFFFFFFE;                  // Make sure ADC is off before configuring

  // b31..24 = 0: Reserved
  // b23 = 0: Temperature sensor and Vrefint is disabled
  // b22 = 0: Vbat channel is disabled
  // b21..18 = 0: Reserved
  // b17..16 = 00: Clock is PCLK2/2
  // b15..13 = 101: DMA Mode 2 is enabled
  // b12 = 0: Reserved
  // b11..8 = 0: Delay is not used in simultaneous mode
  // b7..5 = 0: Reserved
  // b4..0 = 6: Regular simultaneous mode for ADC1 and ADC2; ADC3 independent
  ADC->CCR = ADC_CLOCKPRESCALER_PCLK_DIV2 | ADC_DUALMODE_REGSIMULT | ADC_CCR_DMA_1 | ADC_CCR_DDS;
  
  // b31..27 = 0: Reserved
  // b26 = 0: Overrun interrupt is disabled
  // b25..24 = 0: 12-bit resolution
  // b23 = 0: Analog watchdog on injected channels is disabled
  // b22 = 0: Analog watchdog on regular channels is disabled
  // b21..16 = 0: Reserved
  // b15..13 = 0: Discontinuous mode is not used
  // b12 = 0: Discontinuous mode on injected channels is disabled
  // b11 = 0: Discontinuous mode on regular channels is disabled
  // b10 = 0: Automatic injected group conversion is disabled
  // b9 = 0: Analog watchdog is not used
  // b8 = 1: Scan mode is enabled
  // b7 = 0: No interrupt on injected channels
  // b6 = 0: No interrupt on analog watchdog
  // b5 = 0: No interrupt on end of conversion
  // b4..0 = 0: Analog watchdog is not used
  ADC1->CR1 = ADC_RESOLUTION12b | ADC_CR1_SCAN;

  // b31 = 0: Reserved
  // b30 = 0: No start of conversion for regular channels
  // b29..28 = 01: External trigger for regular channels is enabled.  Rising edge trigger.
  // b27..24 = 1000: Timer 3 TRGO event to trigger start of conversion
  // b23 = 0: Reserved
  // b22 = 0: No start of conversion for injected channels
  // b21..20 = 0: External trigger for injected channels is disabled
  // b19..16 = 0: Injected group is not used
  // b15..12 = 0: Reserved
  // b11 = 0: Data is right-aligned
  // b10 = 0: EOC bit is set at the end of each sequence of regular conversions.  No overrun detection
  // b9 = 0: DMA mode is disabled (for single ADC mode)
  // b8 = 0: DMA mode is disabled (for single ADC mode)
  // b7..2 = 0: Reserved
  // b1 = 0: Single conversion mode
  // b0 = 0: ADC is off (but will be turned on after ADC initialization is complete)
  ADC1->CR2 = ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_3;           // Timer 3 TRGO event trigger
  
  // b31..24 = 0: Reserved
  // b23..20 = 4: Total number of conversions in the regular channel conversion sequence is 5 (1 + 4)
  // b19..0 = 0: 16th - 13th conversions in the sequence are not used
  ADC1->SQR1 = 0x00400000;                // for 5 dual channels
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: 12th - 7th conversions in the sequence are not used
  ADC1->SQR2 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..25 = 0: 6th conversion in the sequence is not used
  // b24..20 = 1: 5th conversion in the sequence is channel 1
  // b19..15 = 13: 4th conversion in the sequence is channel 13
  // b14..10 = 12: 3rd conversion in the sequence is channel 12
  // b9..5 = 11: 2nd conversion in the sequence is channel 11
  // b4..0 = 10: 1st conversion in the sequence is channel 10
  ADC1->SQR3 = 0x0016B16A;
  
  // b31..27 = 0: Reserved
  // b26..0 = 0: SMP18.. SMP10 are set for 3 sampling cycles
  ADC1->SMPR1 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: SMP9.. SMP0 are set for 3 sampling cycles
  ADC1->SMPR2 = 0x00000000;
  
  // b31..6 = 0: Reserved
  // b5 = 0: Overrun flag cleared
  // b4 = 0: Regular channel conversion started flag cleared
  // b3 = 0: Injected channel start of conversion flag cleared
  // b2 = 0: Injected channel end of conversion flag cleared
  // b1 = 0: Regular channel end of conversion flag cleared
  // b0 = 0: Analog watchdog flag cleared
  ADC1->SR = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 1           // *** DAH  MAY WANT TO INVESTIGATE USING THESE
  ADC1->JOFR1 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 2
  ADC1->JOFR2 = 0x00000000;
  
  // b31..12 = 0: Reserved 
  // b11..0 = 0: Data offset for injected channel 3
  ADC1->JOFR3 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 4
  ADC1->JOFR4 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = xFFF: Analog watchdog higher threshold
  ADC1->HTR = 0x00000FFF;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Analog watchdog lower threshold
  ADC1->LTR = 0x00000000;
  
  // b31..22 = 0: Reserved
  // b21..0 = 0: Injected sequences are not used
  ADC1->JSQR = 0x00000000;
  
  // b31..22 = 0: Reserved
  // b21: ADC3 overrun flag
  // b20: ADC3 regular channel conversion started flag
  // b19: ADC3 injected channel start of conversion flag
  // b18: ADC3 injected channel end of conversion flag
  // b17: ADC3 regular channel end of conversion flag
  // b16: ADC3 analog watchdog flag
  // b15..14 = 0: Reserved
  // b13: ADC2 overrun flag
  // b12: ADC2 regular channel conversion started flag
  // b11: ADC2 injected channel start of conversion flag
  // b10: ADC2 injected channel end of conversion flag
  // b9: ADC2 regular channel end of conversion flag
  // b8: ADC2 analog watchdog flag
  // b7..6 = 0: Reserved
  // b5 = 0: ADC1 overrun flag cleared
  // b4 = 0: ADC1 regular channel conversion started flag cleared
  // b3 = 0: ADC1 injected channel start of conversion flag cleared
  // b2 = 0: ADC1 injected channel end of conversion flag cleared
  // b1 = 0: ADC1 regular channel end of conversion flag cleared
  // b0 = 0: ADC1 analog watchdog flag cleared
  ADC->CSR &= 0x003F3F00;

  // ADC will be enabled in EnDisPeripherals() when 120MHz is stable
  // ADC1->CR2 |= 0x00000001;
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_ADC1()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_ADC2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           ADC2 Initialization
// 
//  MECHANICS:          This subroutine initializes Analog to Digital Converter #2
//                      ADC1 and ADC2 are configured in dual simultaneous mode (ref AN3116, March 2010,
//                      from ST Micro).  In this mode, the two ADCs are synchronized to perform two
//                      conversions simultaneously.  Each ADC converts a channel sequence as follows:
//                                        ADC1                                 ADC2
//                          Seq Num   Pin   Input     Channel          Pin  Input    Channel
//                             1      PC0   Ia        ADC1_IN10        PC4  Va_Line  ADC2_IN14
//                             2      PC1   Ib        ADC1_IN11        PC5  Vb_Line  ADC2_IN15
//                             3      PC2   Ic        ADC1_IN12        PB0  Vc_Line  ADC2_IN8
//                             4      PC3   In        ADC1_IN13        PB1  1.25Vref   ADC2_IN9
//                             5      PA1   Vn_Line   ADC1_IN1         PA3  Therm Mem  ADC2_IN3
//                      ADC2 is configured as follows:
//                          Resolution = 12 bits
//                          Clock is PCLK2/2
//                                  = 30MHz/2 = 15MHz for PCLK2 = 30MHz (SYSCLK = 120MHz) 
//                                  = 16MHz/2 = 8MHz for PCLK2 = 16MHz (SYSCLK = 16MHz) 
//                          Conversion time = 3 (sampling) + 12 (conversion) = 15 cycles
//                                  = 1.0usec/channel for PCLK2 = 30MHz (SYSCLK = 120MHz)
//                                  = 1.875usec/channel for PCLK2 = 16MHz (SYSCLK = 16MHz)
//                      Reference Section 13.13 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             ADC2 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_ADC2(void)
{

  __ADC2_CLK_ENABLE();                      // Enable the clock

  ADC2->CR2 &= 0xFFFFFFFE;                  // Make sure ADC is off before configuring

  // b31..27 = 0: Reserved
  // b26 = 0: Overrun interrupt is disabled
  // b25..24 = 0: 12-bit resolution
  // b23 = 0: Analog watchdog on injected channels is disabled
  // b22 = 0: Analog watchdog on regular channels is disabled
  // b21..16 = 0: Reserved
  // b15..13 = 0: Discontinuous mode is not used
  // b12 = 0: Discontinuous mode on injected channels is disabled
  // b11 = 0: Discontinuous mode on regular channels is disabled
  // b10 = 0: Automatic injected group conversion is disabled
  // b9 = 0: Analog watchdog is not used
  // b8 = 1: Scan mode is enabled
  // b7 = 0: No interrupt on injected channels
  // b6 = 0: No interrupt on analog watchdog
  // b5 = 0: No interrupt on end of conversion
  // b4..0 = 0: Analog watchdog is not used
  ADC2->CR1 = ADC_RESOLUTION12b | ADC_CR1_SCAN;

  // b31 = 0: Reserved
  // b30 = 0: No start of conversion for regular channels
  // b29..28 = 0: External trigger for regular channels is disabled (ADC2 is externally triggered via ADC1)
  // b27..24 = 0: No event to trigger start of conversion
  // b23 = 0: Reserved
  // b22 = 0: No start of conversion for injected channels
  // b21..20 = 0: External trigger for injected channels is disabled
  // b19..16 = 0: Injected group is not used
  // b15..12 = 0: Reserved
  // b11 = 0: Data is right-aligned
  // b10 = 0: EOC bit is set at the end of each sequence of regular conversions.  No overrun detection
  // b9 = 0: DMA mode is disabled (for single ADC mode)
  // b8 = 0: DMA mode is disabled (for single ADC mode)
  // b7..2 = 0: Reserved
  // b1 = 0: Single conversion mode
  // b0 = 0: ADC is off (but will be turned on after ADC initialization is complete)
  ADC2->CR2 = 0x00000000;
  
  // b31..24 = 0: Reserved
  // b23..20 = 5: Total number of conversions in the regular channel conversion sequence is 5 (1 + 4)
  // b19..0 = 0: 16th - 13th conversions in the sequence are not used
  ADC2->SQR1 = 0x00400000;
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: 12th - 7th conversions in the sequence are not used
  ADC2->SQR2 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..25 = 0: 6th conversion in the sequence is not used
  // b24..20 = 3: 5th conversion in the sequence is channel 3
  // b19..15 = 9: 4th conversion in the sequence is channel 9
  // b14..10 = 8: 3rd conversion in the sequence is channel 8
  // b9..5 = 15: 2nd conversion in the sequence is channel 15
  // b4..0 = 14: 1st conversion in the sequence is channel 14
  ADC2->SQR3 = 0x0034A1EE;
  
  // b31..27 = 0: Reserved
  // b26..0 = 0: SMP18.. SMP10 are set for 3 sampling cycles
  ADC2->SMPR1 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: SMP9.. SMP0 are set for 3 sampling cycles
  ADC2->SMPR2 = 0x00000000;
  
  // b31..6 = 0: Reserved
  // b5 = 0: Overrun flag cleared
  // b4 = 0: Regular channel conversion started flag cleared
  // b3 = 0: Injected channel start of conversion flag cleared
  // b2 = 0: Injected channel end of conversion flag cleared
  // b1 = 0: Regular channel end of conversion flag cleared
  // b0 = 0: Analog watchdog flag cleared
  ADC2->SR = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 1           // *** DAH  MAY WANT TO INVESTIGATE USING THESE
  ADC2->JOFR1 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 2
  ADC2->JOFR2 = 0x00000000;
  
  // b31..12 = 0: Reserved 
  // b11..0 = 0: Data offset for injected channel 3
  ADC2->JOFR3 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 4
  ADC2->JOFR4 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = xFFF: Analog watchdog higher threshold
  ADC2->HTR = 0x00000FFF;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Analog watchdog lower threshold
  ADC2->LTR = 0x00000000;
  
  // b31..22 = 0: Reserved
  // b21..0 = 0: Injected sequences are not used
  ADC2->JSQR = 0x00000000;
  
  // b31..22 = 0: Reserved
  // b21: ADC3 overrun flag
  // b20: ADC3 regular channel conversion started flag
  // b19: ADC3 injected channel start of conversion flag
  // b18: ADC3 injected channel end of conversion flag
  // b17: ADC3 regular channel end of conversion flag
  // b16: ADC3 analog watchdog flag
  // b15..14 = 0: Reserved
  // b13 = 0: ADC2 overrun flag cleared
  // b12 = 0: ADC2 regular channel conversion started flag cleared
  // b11 = 0: ADC2 injected channel start of conversion flag cleared
  // b10 = 0: ADC2 injected channel end of conversion flag cleared
  // b9 = 0: ADC2 regular channel end of conversion flag cleared
  // b8 = 0: ADC2 analog watchdog flag cleared
  // b7..6 = 0: Reserved
  // b5: ADC1 overrun flag
  // b4: ADC1 regular channel conversion started flag
  // b3: ADC1 injected channel start of conversion flag
  // b2: ADC1 injected channel end of conversion flag
  // b1: ADC1 regular channel end of conversion flag
  // b0: ADC1 analog watchdog flag
  ADC->CSR &= 0x003F003F;

  // ADC will be enabled in EnDisPeripherals() when 120MHz is stable
  // ADC2->CR2 |= 0x00000001;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_ADC2()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_ADC3()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           ADC3 Initialization
// 
//  MECHANICS:          This subroutine initializes Analog to Digital Converter #3
//                      ADC3 is configured as an independent ADC in single-conversion mode (ref AN3116,
//                      March 2010, from ST Micro).  In this mode, the ADC converts the following channels:
//                                         ADC3
//                          Pin    Input                  Channel
//                          PF5    AuxPower_Sense         ADC3_IN15
//                          PF4    Battery Sense          ADC3_IN14
//                          PF3    TA_Sense               ADC3_IN9
//                          PF10   1.25Vref Check         ADC3_IN8    
//                          PF9    Start_Time             ADC3_IN7
//                          PF8    4.5V Supply Sense      ADC3_IN6
//                          PF6    Display 3.3V monitor   ADC3_IN4
//                          PA3    Thermal Memory         ADC3_IN3
//                          PA2    Coil Temp              ADC3_IN2
//
//                      ADC3 is configured as follows:
//                          Resolution = 12 bits
//                          Clock is PCLK2/2
//                                  = 30MHz/2 = 15MHz for PCLK2 = 30MHz (SYSCLK = 120MHz) 
//                                  = 16MHz/2 = 8MHz for PCLK2 = 16MHz (SYSCLK = 16MHz) 
//                          Conversion time = 3 (sampling) + 12 (conversion) = 15 cycles
//                                  = 1.0usec/channel for PCLK2 = 30MHz (SYSCLK = 120MHz)
//                                  = 1.875usec/channel for PCLK2 = 16MHz (SYSCLK = 16MHz)
//                      Reference Section 13.13 of the Programmer's Reference Manual (RM0090).
//
//                      The ADC operates in two modes, normal and coil-temp.  In normal mode, the ADC is
//                      under control of the calling subroutine.  The calling subroutine sets the channel to
//                      be read, initiates the conversion, waits for the conversion to complete, and reads
//                      the result.  In coil_temp mode, the ADC is configured to continuously read the coil
//                      temperature channel at a 4KHz rate.  Conversions are triggered by Timer 5, and the
//                      results are read in the sampling interrupt.
//                          
//  CAVEATS:            None
// 
//  INPUTS:             normal: True if initialization is for single conversions (everything except coil
//                                  temperature)
//                              False if initialization is for coil temperature measurement
// 
//  OUTPUTS:            None
//
//  ALTERS:             ADC3 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_ADC3(uint8_t normal)
{

  __ADC3_CLK_ENABLE();                      // Enable the clock

  ADC3->CR2 &= 0xFFFFFFFE;                  // Make sure ADC is off before configuring

  // b31..27 = 0: Reserved
  // b26 = 0: Overrun interrupt is disabled
  // b25..24 = 0: 12-bit resolution
  // b23 = 0: Analog watchdog on injected channels is disabled
  // b22 = 0: Analog watchdog on regular channels is disabled
  // b21..16 = 0: Reserved
  // b15..13 = 0: Discontinuous mode is not used
  // b12 = 0: Discontinuous mode on injected channels is disabled
  // b11 = 0: Discontinuous mode on regular channels is disabled
  // b10 = 0: Automatic injected group conversion is disabled
  // b9 = 0: Analog watchdog is not used
  // b8 = 0: Scan mode is disabled
  // b7 = 0: No interrupt on injected channels
  // b6 = 0: No interrupt on analog watchdog
  // b5 = 0: No interrupt on end of conversion
  // b4..0 = 0: Analog watchdog is not used
  ADC3->CR1 = ADC_RESOLUTION12b;

  // b31 = 0: Reserved
  // b30 = 0: No start of conversion for regular channels
  // b29..28
  //   Normal:
  //     = 00: External trigger for regular channels is disabled (ADC3 is externally triggered)
  //   Coil temperature:
  //     = 11: External trigger for regular channels is enabled.  Trigger on both rising and falling edges
  // b27..24
  //   Normal:
  //     = 0000: No event to trigger start of conversion
  //   Coil temperature:
  //     = 1010: Timer 5 CC1 event to trigger start of conversion
  // b23 = 0: Reserved
  // b22 = 0: No start of conversion for injected channels
  // b21..20 = 0: External trigger for injected channels is disabled
  // b19..16 = 0: Injected group is not used
  // b15..12 = 0: Reserved
  // b11 = 0: Data is right-aligned
  // b10 = 0: EOC bit is set at the end of each sequence of regular conversions.  No overrun detection
  // b9 = 0: DMA mode is disabled (for single ADC mode)
  // b8 = 0: DMA mode is disabled (for single ADC mode)
  // b7..2 = 0: Reserved
  // b1 = 0: Single conversion mode
  // b0 = 0: ADC is off (but will be turned on after ADC initialization is complete)
  ADC3->CR2 = (normal == TRUE) ?
                (0x00000000) : (ADC_CR2_EXTEN_0 | ADC_CR2_EXTEN_1 | ADC_CR2_EXTSEL_3 | ADC_CR2_EXTSEL_1);
  
  
  // b31..24 = 0: Reserved
  // b23..0 = 0: Sequenced conversions not used
  ADC3->SQR1 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: Sequenced conversions not used
  ADC3->SQR2 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..5 = 0: Single conversion mode
  // b4..0 = ADC3_THERMMEM_CH: Get ready to convert thermal memory voltage
  ADC3->SQR3 = (normal == TRUE) ? (ADC3_THERMMEM_CH) : (ADC3_COILTEMP_CH);
  
  // b31..27 = 0: Reserved
  // b26..0 = 0: SMP18.. SMP10 are set for 3 sampling cycles
  ADC3->SMPR1 = 0x00000000;
  
  // b31..30 = 0: Reserved
  // b29..0 = 0: SMP9.. SMP0 are set for 3 sampling cycles
  ADC3->SMPR2 = 0x00000000;
  
  // b31..6 = 0: Reserved
  // b5 = 0: Overrun flag cleared
  // b4 = 0: Regular channel conversion started flag cleared
  // b3 = 0: Injected channel start of conversion flag cleared
  // b2 = 0: Injected channel end of conversion flag cleared
  // b1 = 0: Regular channel end of conversion flag cleared
  // b0 = 0: Analog watchdog flag cleared
  ADC3->SR = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 1           // *** DAH  MAY WANT TO INVESTIGATE USING THESE
  ADC3->JOFR1 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 2
  ADC3->JOFR2 = 0x00000000;
  
  // b31..12 = 0: Reserved 
  // b11..0 = 0: Data offset for injected channel 3
  ADC3->JOFR3 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Data offset for injected channel 4
  ADC3->JOFR4 = 0x00000000;
  
  // b31..12 = 0: Reserved
  // b11..0 = xFFF: Analog watchdog higher threshold
  ADC3->HTR = 0x00000FFF;
  
  // b31..12 = 0: Reserved
  // b11..0 = 0: Analog watchdog lower threshold
  ADC3->LTR = 0x00000000;
  
  // b31..22 = 0: Reserved
  // b21..0 = 0: Injected sequences are not used
  ADC3->JSQR = 0x00000000;

  // ADC is enabled now so that the Startup Time can be measured before entering the main loop
  ADC3->CR2 |= 0x00000001;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_ADC3()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_DAC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DAC Initialization
// 
//  MECHANICS:          This subroutine initializes the Digital to Analog Converter.  The DAC is used to
//                      inject test signals for on-board trip tests.
//                          Channel 2 is used to inject a ground fault signal
//                          Channel 1 is used to inject a phase fault signal
//                          Resolution = 12 bits
//                      Reference Section 14.5 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             DAC configuration register
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_DAC(void)
{


  RCC->APB1RSTR |= (RCC_APB1RSTR_DACRST);   // Reset the DAC
  RCC->APB1RSTR &= ~(RCC_APB1RSTR_DACRST);
  __DAC_CLK_ENABLE();

  // b31..30 = 0: Reserved
  // b29 = 0: Channel 2 DMA underrun interrupt is disabled
  // b28 = 0: Channel 2 DMA mode is disabled
  // b27..24 = 0: Channel 2 mask/amplitude selector is not used
  // b23..22 = 0: Channel 2 wave generation is disabled
  // b21..19 = 0: Trigger is not used to load DAC2 (automatically loads)
  // b18 = 0: Channel 2 trigger is disabled
  // b17 = 1: Channel 2 output buffer is disabled
  // b16 = 0: Channel 2 is disabled
  // b15..14 = 0: Reserved
  // b13 = 0: Channel 1 DMA underrun interrupt is disabled
  // b12 = 0: Channel 1 DMA mode is disabled
  // b11..8 = 0: Channel 1 mask/amplitude selector is not used
  // b7..6 = 0: Channel 1 wave generation is disabled
  // b5..3 = 0: Trigger is not used to load the DAC1 (automatically loads)
  // b2 = 0: Channel 1 trigger is disabled
  // b1 = 0: Channel 1 output buffer is enabled - tested on 160525: buffer must be enabled to get the full
  //                    output range.  Otherwise, the test voltages do not swing the full range because the
  //                    10K load resistors present too much of a load
  // b0 = 0: Channel 1 is disabled
  DAC->CR = 0x00020000;

  // b31..2 = 0: Reserved
  // b1 = 0: Channel 2 software trigger is disabled
  // b0 = 0: Channel 1 software trigger is disabled
  DAC->SWTRIGR = 0x00000000;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_DAC()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_SPI1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           SPI1 Initialization
// 
//  MECHANICS:          This subroutine initializes SPI #1.  This SPI bus is used to communicate with the
//                      Serial Flash (SST26VF064B), used to store waveform captures and extended captures.
//                      The Flash operates at 0-40MHz, Modes 0 and 3.
//                      Reference Section 28.5 of the Programmer's Reference Manual (RM0090).
//                      Note, the initialization is the same regardless of the system clock (and PCLK2)
//                      speed.  Therefore, at 16MHz, the interface will run about half as fast as at 120MHz.
//                          
//  CAVEATS:            None
// 
//  INPUTS:             config - the configuration for the SPI.  The Flash uses both 8-bit and 16-bit modes
// 
//  OUTPUTS:            None
//
//  ALTERS:             SPI1_CR1 and _CR2 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_SPI1(uint8_t config)
{

  __SPI1_FORCE_RESET();
  __SPI1_RELEASE_RESET();
  __SPI1_CLK_ENABLE();

  // b15 = 0: Two-line unidirectional data mode
  // b14 = 0: Output enable in bidirectional mode (not used)
  // b13 = 0: CRC calculation disabled
  // b12 = 0: No CRC phase (not used)
  // b11    : 0 = 8-bit data frame
  // b10 = 0: Full duplex (not used)
  // b9 = 1: Software slave select management is enabled
  // b8 = 1: Internal slave select is active
  // b7 = 0: MSB transmitted first
  // b6 = 0: SPI is disabled
  // b5..3 : 0 = Baud rate is fPCLK2/2
  // b2 = 1: SPI master
  // b1 = 1: CPOL, clock is high when idle
  // b0 = 1: CPHA, data is captured on the second clock transition
  // 8-bit or 16-bit data frame, clock idle is HIGH, data captured on the second clock transition (rising
  //   edge for CPOL=1), clock = 8MHz or 15MHz, depending on SYSCLK
  SPI1->CR1 = ( (config == DEV_FRAM_FLASH8) ? 0x0307 : 0x0B07);
                                            
  // b15..8 = 0: Reserved
  // b7 = 0: TX interrupt is disabled
  // b6 = 0: RX interrupt is disabled
  // b5 = 0: Error interrupt is disabled
  // b4 = 0: Frame format is Motorola mode
  // b3 = 0: Reserved
  // b2 = 0: Slave select output is disabled
  // b1 = 0: Tx buffer DMA is disabled
  // b0 = 0: Rx buffer DMA is disabled
  SPI1->CR2 = 0x0000;

  // Enable the SPI
  SPI1->CR1 |= 0x0040;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_SPI1()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_SPI2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           SPI2 Initialization
// 
//  MECHANICS:          This subroutine initializes SPI #2.  This SPI bus is used to communicate with the:
//                          (1) Serial FRAM (256KBit)
//                          (2) Serial FRAM (32Kbit)
//                          (3) Serial DAC (AD5624)
//                          (4) Temperature/Humidity Sensor (HIH6030)
//                          (5) The Frame Module FRAM
//                      The FRAMs (items 1 and 6), operate at 0-30MHz, Modes 0 and 3
//                      The DAC (item 3) operates at 0-50MHz, Mode 1
//                      The RTC (item 4) operates at 0-4MHz, Modes 1 or 3
//                      The HIH6030 (item 5) operates at 0-800KHz, Mode 0
//                      Reference Section 28.5 of the Programmer's Reference Manual (RM0090).
//                      Note, the initialization is the same regardless of the system clock (and PCLK1)
//                      speed.  Therefore, at 16MHz, the interface will run about half as fast as at 120MHz.
//                      Since very little data is being transferred when the clock is 16MHz, this should not
//                      be an issue.
//                          
//  CAVEATS:            None
// 
//  INPUTS:             Device - the device that the SPI needs to interface with
// 
//  OUTPUTS:            None
//
//  ALTERS:             SPI2_CR1 and _CR2 registers
// 
//  CALLS:              None
//
//  EXECUTION TIME:     The execution time was measured on 180614.  It is about 600nsec
// 
//------------------------------------------------------------------------------------------------------------

void Init_SPI2(uint8_t Device)
{

  __SPI2_FORCE_RESET();
  __SPI2_RELEASE_RESET();
  __SPI2_CLK_ENABLE();

  // b15 = 0: Two-line unidirectional data mode
  // b14 = 0: Output enable in bidirectional mode (not used)
  // b13 = 0: CRC calculation disabled
  // b12 = 0: No CRC phase (not used)
  // b11    : 0 = 8-bit data frame
  //          1 = 16-bit data frame
  // b10 = 0: Full duplex (not used)
  // b9 = 1: Software slave select management is enabled
  // b8 = 1: Internal slave select is active
  // b7 = 0: MSB transmitted first
  // b6 = 0: SPI is disabled
  // b5..3 : 0 = Baud rate is fPCLK1/2
  //         1 = Baud rate is fPCLK1/4
  //         2 = Baud rate is fPCLK1/8
  //         3 = Baud rate is fPCLK1/16
  //         4 = Baud rate is fPCLK1/32
  //         5 = Baud rate is fPCLK1/64
  // b2 = 1: SPI master
  // b1    : 0 = CPOL, clock is low when idle
  //         1 = CPOL, clock is high when idle
  // b0    : 0 = CPHA, data is captured on the first clock transition
  //         1 = CPHA, data is captured on the second clock transition
  if (Device == DEV_FRAM_FLASH8)
  {                                         // 8-bit data frame, clock idle is HIGH, data captured on the
    SPI2->CR1 = 0x0307;                     //   second clock transition (rising edge for CPOL=1),
  }                                         //   clock = 8MHz or 15MHz, depending on SYSCLK
  else if (Device == DEV_FRAM_FLASH16)
  {                                         // 16-bit data frame, clock idle is HIGH, data captured on the
    SPI2->CR1 = 0x0B07;                     //   second clock transition (rising edge for CPOL=1),
  }                                         //   clock = 8MHz or 15MHz, depending on SYSCLK
  else if (Device == DEV_TH_SENSOR8)
  {                                         // 8-bit data frame, clock idle is LOW, data captured on the
    SPI2->CR1 = 0x032C;                     //   first clock transition (rising edge for CPOL=0),       
  }                                         //   clock = 250KHz or 469KHz, depending on SYSCLK
  else if (Device == DEV_TH_SENSOR16)
  {                                         // 16-bit data frame, clock idle is LOW, data captured on the
    SPI2->CR1 = 0x0B2C;                     //   first clock transition (rising edge for CPOL=0),       
  }                                         //   clock = 250KHz or 469KHz, depending on SYSCLK
  else if (Device == DEV_EEPOT_8)
  {                                         // 8-bit data frame, clock idle is HIGH, data captured on the
    SPI2->CR1 = 0x0317;                     //   second clock transition (rising edge for CPOL=1),       
  }                                         //   clock = 2MHz or 3.75MHz, depending on SYSCLK             
  else if (Device == DEV_EEPOT_16)
  {                                         // 16-bit data frame, clock idle is HIGH, data captured on the
    SPI2->CR1 = 0x0B17;                     //   second clock transition (rising edge for CPOL=1),       
  }                                         //   clock = 2MHz or 3.75MHz, depending on SYSCLK             
  else if (Device == DEV_DAC)
  {                                         // 8-bit data frame, clock idle is LOW, data captured on the
    SPI2->CR1 = 0x0305;                     //   second clock transition (falling edge for CPOL=0),       
  }                                         //   clock = 8MHz or 15MHz, depending on SYSCLK             
  else                                  // Default to RTC
  {                                         // 8-bit data frame, clock idle is LOW, data captured on the
    SPI2->CR1 = 0x0314;                     //   first clock transition (rising edge for CPOL=0),
  }                                         //   clock = 2MHz or 3.75Hz, depending on SYSCLK             
                                            
  // b15..8 = 0: Reserved
  // b7 = 0: TX interrupt is disabled
  // b6 = 0: RX interrupt is disabled
  // b5 = 0: Error interrupt is disabled
  // b4 = 0: Frame format is Motorola mode
  // b3 = 0: Reserved
  // b2 = 0: Slave select output is disabled
  // b1 = 0: Tx buffer DMA is disabled
  // b0 = 0: Rx buffer DMA is disabled
  SPI2->CR2 = 0x0000;

  // Enable the SPI
  SPI2->CR1 |= 0x0040;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_SPI2()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_SPI3()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           SPI3 Initialization
// 
//  MECHANICS:          This subroutine initializes SPI #3.  This SPI bus is used to communicate with the
//                      AFE.  The SPI is configured as follows:
//                          16-bit transactions
//                          Clock speed = fPCLK1/2 (fastest speed possible)
//                                      = 16MHz/2 = 8MHz for SYSCLK = 16MHz (HSI)
//                                      = 30MHz/2 = 15MHz for SYSCLK = 120MHz (PLL)
//                          Mode = 3 (Clock idles HIGH; data changes on falling edge, captured on rising
//                                    edge)
//                      Reference Section 28.5 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             SPI3_CR1 and _CR2 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_SPI3(void)
{

  __SPI3_FORCE_RESET();
  __SPI3_RELEASE_RESET();
  __SPI3_CLK_ENABLE();

  // b15 = 0: Two-line unidirectional data mode
  // b14 = 0: Output enable in bidirectional mode (not used)
  // b13 = 0: CRC calculation disabled
  // b12 = 0: No CRC phase (not used)
  // b11 = 1: 16-bit data frame (for ADE7779: 1 R/W bit, 7 address bits, 8 data bits)
  // b10 = 0: Full duplex
  // b9 = 1: Software slave select management is enabled
  // b8 = 1: Internal slave select is active
  // b7 = 0: MSB transmitted first
  // b6 = 0: SPI is disabled
  // b5..3 = 0: Baud rate is fPCLK1/2
  // b2 = 1: SPI master
  // b1 = 1: CPOL, clock is high when idle
  // b0 = 1: CPHA, data is captured on the second clock transition (rising edge for CPOL=1)
  SPI3->CR1 = 0x0B07;

  // b15..8 = 0: Reserved
  // b7 = 0: TX interrupt is disabled
  // b6 = 0: RX interrupt is disabled
  // b5 = 0: Error interrupt is disabled
  // b4 = 0: Frame format is Motorola mode
  // b3 = 0: Reserved
  // b2 = 0: Slave select output is disabled
  // b1 = 1: Tx buffer DMA is enabled
  // b0 = 1: Rx buffer DMA is enabled
  SPI3->CR2 = 0x0003;

  // Enable the SPI
  SPI3->CR1 |= 0x0040;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_SPI3()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_I2C2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I2C2 Initialization
// 
//  MECHANICS:          This subroutine initializes I2C #2.  This I2C bus is used to communicate with the
//                      override microprocessor.  The I2C is configured as follows:
//                          Slave Mode - FM
//                          Slave Address = 86 (x56)
//                          Speed = 400KHz for SYSCLK = 120MHz (PCLK1 = 30MHz)
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 27 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK1 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             I2C2 configuration registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_I2C2(void)
{

  __I2C2_FORCE_RESET();
  __I2C2_RELEASE_RESET();
  __I2C2_CLK_ENABLE();

  // b15..13 = 0: Reserved
  // b12 = 0: Next DMA EOT is not the last transfer (not used)
  // b11 = 0: DMA requests disabled
  // b10 = 1: TXE or RXNE generates an interrupt
  // b9 = 1: Event interrupt enabled
  // b8 = 0: Error interrupt disabled
  // b7..6 = 0: Reserved
  // b5..0 = 8: PCLK1 = 30MHz  (33.3ns)
  I2C2->CR2 = 0x0608;

  // b15 = 1: FM Mode
  // b14 = 0: Tlow/Thigh = 2
  // b13..12 = 0: Reserved
  // b11..0 = 150: Tlow = 150 * 0.033us = 5us, Thigh = 150 * 0.033us = 5us,
  //              f = 1/(5usec +5usec) = 1/10usec = 100KHz
  I2C2->CCR = 0x0096;

  // b15..6 = 0: Reserved
  // b5..0 =  31: Max rise time = [1000nsec (from I2C spec in SM mode) * 30MHz (PCLK1)] + 1
  I2C2->TRISE = 0X001F;

  // b15..5 = 0: Reserved
  // b4 = 1: Analog noise filter is disabled
  // b3..0 = 6: Digital noise filter is enabled and will suppress up to 6/30MHz (PCLK1) = 200nsec
  // I2C2->FLTR = 0x0006;                     // *** DAH CHECK THIS NOT AVAILABLE ON 32F407 CHECK THE DIASSEMBLY
                                              // *** BP CHECKED AND IT IS NOT AVAILABLE
  // b15 = 0: 7-bit slave address
  // b14 = 1: Reserved = 1 (must be 1)
  // b13..10 = 0: Reserved = 0
  // b9..8 = 0: Don't care for 7-bit slave address
  // b7..1 = 43: Slave address = 43 (x2B)
  // b0 = 0: Don't care for 7-bit slave address
  I2C2->OAR1 = 0x404A;

  // b15..8 = 0: Reserved = 0
  // b7..1 = 0: Don't care since not using dual addressing mode
  // b0 = 0: Not using dual addressing mode - only OAR1 is recognized
  I2C2->OAR2 = 0x0000;

  // b15 = 0: Peripheral is not under software reset
  // b14 = 0: Reserved = 0
  // b13 = 0: SMBus alert is not used
  // b12 = 0: No packet error checking transfer
  // b11 = 0: Clear POS flag
  // b10 = 0: No acknowledge returned
  // b9 = 0: No Stop generation
  // b8 = 0: No Start generation
  // b7 = 0: Clock stretching is enabled
  // b6 = 0: General call is disabled (Address x00 is NACKed)
  // b5 = 0: Packet error checking is disabled
  // b4 = 0: ARP is disabled
  // b3 = 0: SMBus device
  // b2 = 0: Reserved = 0
  // b1 = 0: I2C mode
  // b0 = 0: Peripheral is disabled
  I2C2->CR1 = 0x0000;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_I2C2()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_I2C3()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I2C3 Initialization
// 
//  MECHANICS:          This subroutine initializes I2C #3.  This I2C bus is used to communicate with the
//                      HDC1010 temperature/humidity sensor, and to provide health bus communications.  The
//                      I2C is configured as follows:
//                          Master Mode - FM
//                          Speed = 400KHz for SYSCLK = 120MHz (PCLK1 = 30MHz)
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 27 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK1 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             I2C3 configuration registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_I2C3(void)
{

  __I2C3_FORCE_RESET();
  __I2C3_RELEASE_RESET();
  __I2C3_CLK_ENABLE();

  // b15..13 = 0: Reserved
  // b12 = 0: Next DMA EOT is not the last transfer
  // b11 = 0: DMA requests disabled
  // b10 = 1: TXE or RXNE generates an interrupt
  // b9 = 1: Event interrupt enabled
  // b8 = 1: Error interrupt enabled
  // b7..6 = 0: Reserved
  // b5..0 = 30: PCLK1 = 30MHz
  I2C3->CR2 = 0x070F;

  // b15 = 1: FM Mode
  // b14 = 0: Tlow/Thigh = 2
  // b13..12 = 0: Reserved
  // b11..0 = 25: Tlow = 2*25/30MHz=1.67usec, Thigh = 25/30MHz=.83usec,
  //              f = 1/(1.67usec+.83usec)=1/2.5usec=400KHz
  I2C3->CCR = 0x82A0;       // 21kHz
  I2C3->CCR &= ~(1<<15);

  // b15..6 = 0: Reserved
  // b5..0 =  9: Max rise time = 267nsec (from I2C spec in FM mode) * 30MHx (PCLK1) + 1
  I2C3->TRISE = 0x0009;

  // b15..5 = 0: Reserved
  // b4 = 1: Analog noise filter is disabled
  // b3..0 = 6: Digital noise filter is enabled and will suppress up to 6/30MHz (PCLK1) = 200nsec
  I2C3->FLTR = 0x0006;

  // b15 = 0: Peripheral is not under software reset
  // b14 = 0: Reserved = 0
  // b13 = 0: SMBus alert is not used
  // b12 = 0: No packet error checking transfer
  // b11 = 0: Clear POS flag
  // b10 = 0: No acknowledge returned
  // b9 = 0: No Stop generation
  // b8 = 0: No Start generation
  // b7 = 0: Clock stretching is enabled
  // b6 = 0: General call is disabled (Address x00 is NACKed)
  // b5 = 0: Packet error checking is disabled
  // b4 = 0: ARP is disabled
  // b3 = 0: SMBus device
  // b2 = 0: Reserved = 0
  // b1 = 0: I2C mode
  // b0 = 0: Peripheral is disabled
  I2C3->CR1 = 0x0000;
  
  NVIC_EnableIRQ(I2C3_EV_IRQn);
  NVIC_EnableIRQ(I2C3_ER_IRQn);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_I2C3()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART1 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #1.  This UART is used for high-speed CAM
//                      communications.  The UART is configured as follows:
//                          3.75 Mbps
//                          no parity
//                          one stop bit
//                          no hardware flow control
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK2 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART1 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_UART1(void)
{
  volatile uint16_t temp;

  USART1->CR1 &= 0xFFFFDFFF;                    // Make sure the UART is disabled

  __USART1_FORCE_RESET();
  __USART1_RELEASE_RESET();
  __USART1_CLK_ENABLE();

  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  USART1->CR2 = 0x00000000;

  // b31..b16: Reserved = 0
  // b15 = 0: Oversampling by 16
  // b15 = 1: Oversampling by 8
  // b14 = 0: Reserved = 0
  // b13 = 0: UART1 is disabled
  // b12 = 0: 8-bit character length
  // b11 = 0: Idle Line wakeup method
  // b10 = 0: No parity bit
  // b9 = 0: Even parity (not used)
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = 0: Receive interrupt is disabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted
  USART1->CR1 = 0x0000800C;             // 8-bit chars, oversampling by 8 (for 3.75Mbps), no interrupts
                                        //   (using DMA)

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 1: DMA mode is enabled for transmission
  // b6 = 1: DMA mode is enabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  USART1->CR3 = 0x000000C0;

  // Set the baud rate register for 3.75Mbps.  PCLK = 30MHz. Oversampling is 8.
  //   Reference Table 139 in RM0090
  //       f = PCLK2/(8 x USARTDIV) --> USARTDIV = 30MHz/(8 x 3.75M) = 1
  // b31..b16: Reserved = 0
  // b15..4 = 1: Mantissa of USARTDIV
  // b3..0 = 0: Fraction of USARTDIV (must be 0 if oversampling by 8)
  USART1->BRR = 0x00000010;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  USART1->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = USART1->SR;
  temp = USART1->DR;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART1()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART2 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #2.  This UART is used for normal communications
//                      with the display processor.  The UART is configured as follows:
//                          3.0M baud
//                          no parity
//                          one stop bit
//                          no hardware flow control
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                      Note, 3.0Mbps is chosen because it is compatible with the display processor
//                      frequency.  The display processor operates at 168MHz, and 3MHz can be derived on
//                      both processors
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK1 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART2 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_UART2(void)
{
  volatile uint16_t temp;

  USART2->CR1 &= 0xFFFFDFFF;                    // Make sure the UART is disabled

  __USART2_FORCE_RESET();
  __USART2_RELEASE_RESET();
  __USART2_CLK_ENABLE();

  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  USART2->CR2 = 0x00000000;

  // b31..b16: Reserved = 0
  // b15 = 1: Oversampling by 8
  // b14 = 0: Reserved = 0
  // b13 = 0: UART2 is disabled
  // b12 = 0: 8-bit character length
  // b11 = 0: Idle Line wakeup method
  // b10 = 0: No parity bit
  // b9 = 0: Even parity (not used)
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = 0: Receive interrupt is disabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted
  USART2->CR1 = 0x0000800C;

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 1: DMA mode is enabled for transmission
  // b6 = 1: DMA mode is enabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  USART2->CR3 = 0x000000C0;

  // Set the baud rate register for 3.0Mbps.  PCLK = 30MHz. Oversampling is 8.
  //   Reference Table 139 in RM0090
  //       f = PCLK1/(8 x USARTDIV) --> USARTDIV = 30MHz/(8 x 3.0M) = 1.25
  // b31..b16: Reserved = 0
  // b15..4 = 1: Mantissa of USARTDIV
  // b3..0 = 2: Fraction of USARTDIV (2/8 = .25)
  USART2->BRR = 0x00000012;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  USART2->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = USART2->SR;
  temp = USART2->DR;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART2()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART3()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART3 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #3.  This UART is used for GOOSE and/or Sampled
//                      Value communications with the display processor.  The UART is configured as follows:
//                          3.0M baud
//                          no parity
//                          one stop bit
//                          no hardware flow control
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                      Note, 3.0Mbps is chosen because it is compatible with the display processor
//                      frequency.  The display processor operates at 168MHz, and 3MHz can be derived on
//                      both processors
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK1 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART3 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_UART3(void)
{
  volatile uint16_t temp;

  USART3->CR1 &= 0xFFFFDFFF;                    // Make sure the UART is disabled

  __USART3_FORCE_RESET();
  __USART3_RELEASE_RESET();
  __USART3_CLK_ENABLE();

  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  USART3->CR2 = 0x00000000;

  // b31..b16: Reserved = 0
  // b15 = 1: Oversampling by 8
  // b14 = 0: Reserved = 0
  // b13 = 0: UART2 is disabled
  // b12 = 0: 8-bit character length
  // b11 = 0: Idle Line wakeup method
  // b10 = 0: No parity bit
  // b9 = 0: Even parity (not used)
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = 0: Receive interrupt is disabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted
  USART3->CR1 = 0x0000800C;

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 1: DMA mode is enabled for transmission
  // b6 = 1: DMA mode is enabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  USART3->CR3 = 0x000000C0;

  // Set the baud rate register for 3.0Mbps.  PCLK = 30MHz. Oversampling is 8.
  //   Reference Table 139 in RM0090
  //       f = PCLK1/(8 x USARTDIV) --> USARTDIV = 30MHz/(8 x 3.0M) = 1.25
  // b31..b16: Reserved = 0
  // b15..4 = 1: Mantissa of USARTDIV
  // b3..0 = 2: Fraction of USARTDIV (2/8 = 0.25)
  USART3->BRR = 0x00000012;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  USART3->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = USART3->SR;
  temp = USART3->DR;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART3()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART5()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART5 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #5.  This UART is used for (internal) Test Port
//                      communications.  The UART is configured as follows:
//                          9600 baud
//                          no parity
//                          one stop bit
//                          no hardware flow control
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            None
// 
//  INPUTS:             SysClk_120MHz - TRUE if system clock is 120MHz (PCLK1 = 30MHz)
//                                    - FALSE if system clock is 16MHz (PCLK1 = 16MHz)
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART5 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_UART5(void)
{
  volatile uint16_t temp;

  UART5->CR1 &= 0xFFFFDFFF;                     // Make sure the UART is disabled

  __UART5_FORCE_RESET();
  __UART5_RELEASE_RESET();
  __UART5_CLK_ENABLE();

  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  UART5->CR2 = 0x00000000;

  // b31..b16: Reserved = 0
  // b15 = 0: Oversampling by 16
  // b14 = 0: Reserved = 0
  // b13 = 0: UART5 is disabled
  // b12 = 0: 8-bit character length
  // b11 = 0: Idle Line wakeup method
  // b10 = 0: No parity bit
  // b9 = 0: Even parity (not used)
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = 1: Receive interrupt is enabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted
  UART5->CR1 = 0x0000002C;

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 0: DMA mode is disabled for transmission
  // b6 = 0: DMA mode is disabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  UART5->CR3 = 0x00000000;

  // Set the baud rate register for 9600.  PCLK = 30MHz. Oversampling is 16.
  //   Reference Table 139 in RM0090
  //       f = PCLK1/(8 x 2 x USARTDIV) --> USARTDIV = 30MHz/(16 x 9600) = 195.3125
  //                                        USARTDIV = 16MHz/(16 x 9600) = 104.1667
  // b31..b16: Reserved = 0
  // b15..4: Mantissa of USARTDIV  For SYSCLK = 120MHz (PCLK1 = 30MHz), 195
  //                               For SYSCLK = 16MHz (PCLK1 = 16MHz), 104
  // b3..0: Fraction of USARTDIV   For SYSCLK = 120MHz, (PCLK1 = 30MHz), 0.3125 = 5/16
  //                               For SYSCLK = 16MHz, (PCLK1 = 16MHz), 0.1667 ~ 3
  UART5->BRR = (SysClk_120MHz == TRUE) ? 0x00000C35 : 0x00000683;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  UART5->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = UART5->SR;
  temp = UART5->DR;

  UART5->CR1 |= 0x00002000;                     // b13 = 1: UART5 is enabled

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART5()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART6()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART6 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #6.  This UART is used for either high-speed CAM
//                      communications or Modbus communications.  The UART is configured as follows:
//                          3.75 Mbps
//                          no parity
//                          one stop bit
//                          no hardware flow control
//                      Note, the peripheral is disabled when the subroutine exits.
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK2 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART6 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

/*void Init_UART6(void)
{
  uint16_t temp;

  USART6->CR1 &= 0xFFFFDFFF;                    // Make sure the UART is disabled

  __USART6_FORCE_RESET();
  __USART6_RELEASE_RESET();
  __USART6_CLK_ENABLE();

  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  USART6->CR2 = 0x00000000;

  // b31..b16: Reserved = 0
  // b15 = 1: Oversampling by 8
  // b14 = 0: Reserved = 0
  // b13 = 0: UART6 is disabled
  // b12 = 0: 8-bit character length
  // b11 = 0: Idle Line wakeup method
  // b10 = 0: No parity bit
  // b9 = 0: Even parity (not used)
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = 0: Receive interrupt is disabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted
  USART6->CR1 = 0x0000800C;             // 8-bit chars, oversampling by 8 (for 3.75Mbps), no interrupts
                                        //   (using DMA)

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 1: DMA mode is enabled for transmission
  // b6 = 1: DMA mode is enabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  USART6->CR3 = 0x000000C0;

  // Set the baud rate register for 3.75Mbps.  PCLK = 30MHz. Oversampling is 8.
  //   Reference Table 139 in RM0090
  //       f = PCLK2/(8 x USARTDIV) --> USARTDIV = 30MHz/(8 x 3.75M) = 1
  // b31..b16: Reserved = 0
  // b15..4 = 1: Mantissa of USARTDIV
  // b3..0 = 0: Fraction of USARTDIV (must be 0 if oversampling by 8)
  USART6->BRR = 0x00000010;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  USART6->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = USART6->SR;
  temp = USART6->DR;

}*/

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART6()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_UART6()     THIS NEEDS TO BE INTEGRATED IN WITH THE CAM UART INITIALIZATION
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART6 Initialization
// 
//  MECHANICS:          This subroutine initializes UART #6.  This UART is used for Modbus communications.
//                      The Modbus port is a slave.  Characters are 8 bits.  The baud rate, number of stop
//                      bits, and parity are setpoint-programmable:
//                        Setpoints2.stp.Modbus_Port_Baudrate
//                          0 - 1200 bps
//                          1 - 4800 bps
//                          2 - 9600 bps
//                          3 - 19200 bps
//                        Setpoints2.stp.Modbus_Port_Parity
//                          0 - None
//                          1 - Odd
//                          2 - Even
//                        Setpoints2.stp.Modbus_Port_Stopbit
//                          1 or 2
//
//                      Note, the peripheral is enabled when the subroutine exits.
//                      Reference Section 30 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            This is initialized assuming the system clock (SYSCLK) is 120MHz, and PCLK1 is 30MHz
//                      If we are operating at 16MHz, the interface should be disabled
// 
//  INPUTS:             rx_int_enabled: True - enable RX interrupts.  False - do not enable Rx interrupts
// 
//  OUTPUTS:            None
//
//  ALTERS:             UART6 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_UART6(uint8_t rx_int_enabled)
{
  uint32_t temp;

  USART6->CR1 &= 0xFFFFDFFF;                    // Make sure the UART is disabled

  __USART6_FORCE_RESET();
  __USART6_RELEASE_RESET();
  __USART6_CLK_ENABLE();


  // set CR2, CR1, and BRR based on setpoints

  // CR2
  // b31..b15: Reserved = 0
  // b14 = 0: LIN mode disabled
  // b13..12 = 0: 1 Stop bit
  // b12 = 0: No CRC phase (not used)
  // b11 = 0: SCLK pin disabled
  // b10 = 0: SCLK polarity (not used)
  // b9 = 0: SCLK phase (not used)
  // b8 = 0: Last bit clock pulse (not used)
  // b7 = 0: Reserved = 0
  // b6 = 0: LIN break interrupt is disabled
  // b5 = 0: LIN break detection length (not used)
  // b4 = 0: Reserved = 0
  // b3..0 = 0: UART address (not used) 
  switch (Setpoints2.stp.Modbus_Port_Stopbit)
  {
    case 1:
      USART6->CR2 = 0x00000000; // b13..12 = 0: 1 stop bit
      break;
    default:
      USART6->CR2 = 0x00002000; // b13..12 = 2: 2 stop bits
      break;
  }

  //CR1
  // b31..b16: Reserved = 0
  // b15 = 0: Oversampling by 16
  // b14 = 0: Reserved = 0
  // b13 = 0: UART6 is disabled
  // b12 = x: 9-bit character length (9 if using parity, 8 otherwise)
  // b11 = 0: Idle Line wakeup method
  // b10 = x: Parity bit is used
  // b9 = x: Even parity
  // b8 = 0: Parity interrupt is disabled
  // b7 = 0: Transmitter empty interrupt is disabled
  // b6 = 0: Transmit complete interrupt is disabled
  // b5 = x: Receive interrupt is enabled
  // b4 = 0: IDLE interrupt is disabled
  // b3 = 1: Transmitter is enabled
  // b2 = 1: Receiver is enabled
  // b1 = 0: Receiver is in active mode
  // b0 = 0: No break character is transmitted

  // build the CR2 value
  // always enable Transmitter(b3) and Receiver(b2)
  // enable Receiver Interrupt(b5) based on received value
  temp = (rx_int_enabled ? (0x0000002C) : (0x0000000C));
  // add in the Parity configuration
  switch (Setpoints2.stp.Modbus_Port_Parity)
  {
    case 0:
      // no change to CR1 for No Parity
      break;
    case 1:
      temp |= 0x00001600; // b12=1, b10=1, b9=1: Odd Parity
      break;
    default:
      temp |= 0x00001400; // b12=1, b10=1, b9=0: Even Parity
      break;
  }
  USART6->CR1 = temp; // set the configuration register

  // BRR
  // Set the baud rate register for 9600.  PCLK = 30MHz. Oversampling is 16.
  //   Reference Table 139 in RM0090
  //       f = PCLK1/(8 x 2 x USARTDIV) --> USARTDIV = 30MHz/(16 x 9600) = 195.3125
  // b31..b16: Reserved = 0
  // b15..4 = 195: Mantissa of USARTDIV
  // b3..0 = 0.3125: Fraction of USARTDIV (fraction * 16 and round/carry to mantissa if applicable)
  switch (Setpoints2.stp.Modbus_Port_Baudrate)
  {
    case 0: // 9600 baud
      // 30MHz/(16 * 9600) = 195.3125, 195 = 0xC3
      // 0.3125 * 16 = 5 = 0x5
      USART6->BRR = 0x00000C35;
      break;
    case 1: // 19200 baud
      // 30MHz/(16 * 19200) = 97.65625, 97 = 0x61
      // 0.65625 * 16 = 10.5, round to 11 = 0xB
      USART6->BRR = 0x0000061B;
      break;
    case 2: // 38400 baud
      // 30MHz/(16 * 38400) = 48.828125, 48 = 0x30
      // 0.828125 * 16 = 13.25, round to 13 = 0xD
      USART6->BRR = 0x0000030D;
      break;
      case 3: // 57600 baud
        // 30MHz/(16 * 57600) = 32.5520833, 32 = 0x20
        // 0.5520833 * 16 = 8.833, round to 9 = 0x9
        USART6->BRR = 0x00000209;
        break;
    default: // 115200 baud
      // 30MHz/(16 * 115200) = 16.27604166, 16 = 0x10
      // 0.27604166 * 16 = 4.4166, round to 4 = 0x4
      USART6->BRR = 0x00000104;
      break;
  }

  // b31..b12: Reserved = 0
  // b11 = 0: Three sample bit method
  // b10 = 0: CTS interrupt is inhibited
  // b9 = 0: CTS hardware flow control is disabled
  // b8 = 0: RTS hardware flow control is disabled
  // b7 = 1: DMA mode is enabled for transmission
  // b6 = 0: DMA mode is disabled for reception
  // b5 = 0: Smartcard mode is disabled
  // b4 = 0: Smartcard NACK is disabled (not used)
  // b3 = 0: Single-wire half-duplex mode is not selected
  // b2 = 0: Normal mode (not low-power IrDA mode)
  // b1 = 0: IrDA is disabled
  // b0 = 0: Error interrupt is inhibited
  USART6->CR3 = 0x00000080;

  // Make sure transmission complete (TC) and read data register not empty (RXNE) flags are clear
  USART6->SR &= 0xFFFFFF9F;

  // Clear overrun error (ORE), noise detected (NF), framing error, and parity error (PE) flags by
  //   doing read of SR register followed by read of data register
  temp = USART6->SR;
  temp = USART6->DR;
  USART6->CR1 |= 0x00002000;         // ENABLE THE UART

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_UART6()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer1 Initialization
//
//  MECHANICS:          This subroutine initializes Timer1. It is used to generate a 4MHz square wave (when
//                      SYSCLK = 120MHz) that becomes the clock source for the AFE.  Timer1 runs off of the
//                      APB2 timer clock (2 x APB2 Clk = 60MHz).  It is configured as an upcounter in
//                      edge-aligned PWM mode.  The autoreload value (in TIM1->ARR) is 14.  When the count
//                      reaches 14, on the next clock pulse, the counter rolls over to 0 and the output
//                      toggles.  The frequency is therefore 60MHz/15 = 4MHz.
//                      The capture/compare value (in TIM->CCR1) is 8.  This determines the high time of the
//                      output.  When the count is less than 8, the output is high.  When it is greater than
//                      or equal to 8, the output is low.
//                      Thus, the counter works as follow:
//                          count:   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14  0  1  2  3 ...
//                          output:  H  H  H  H  H  H  H  H  L  L  L  L  L  L  L  H  H  H  H ...
//                      The output is brought out on PA8.
//
//                      Reference Section 17 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer1 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM1(void)
{

  TIM1->CR1 &= 0xFFFFFFFE;                  // Make sure the timer is disabled

  __TIM1_FORCE_RESET();
  __TIM1_RELEASE_RESET();
  __TIM1_CLK_ENABLE();                      // Enable clock to Timer1

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: PWM edge-aligned mode
  // b4 = 0: Counter direction is up
  // b3 = 0: Counter is not stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled.  This bit must be cleared to allow updates to the PSC and ARR
  //        registers.  Otherwise, the values remain in the respective preload registers.  I verified this
  //        with the emulator on 190118.  It also must be clear if you want an update event to generate an
  //        interrupt.  Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM1->CR1 = 0x0000;

  // b15 = 0: Reserved
  // b14..8 = 0: Output idle state = 0
  // b7 = 0: TI1 input connected to TIM1_CH1 pin
  // b6..4 = 000: Master Mode selection - Trigger output is not used so keep at reset value
  // b3 = 0: Capture/compare DMA selection (not used)
  // b2 = 0: Capture/compare control update selection - not used so keep at reset value
  // b1 = 0: Reserved
  // b0 = 0: Capture/compare bits are not preloaded
  TIM1->CR2 = 0x0000;

  // b15..0 = 0: This is the slave mode control register - not used so keep at reset values
  //             (slave mode is disabled)
  TIM1->SMCR = 0x0000;

  // b15..0 = 0: All interrupts disabled
  TIM1->DIER = 0x0000;

  //   Timer1 runs off of 2 x APB2 which is 60 Mhz.  Since we want a 4MHz output, we need to divide by 15.
  //   The ARR register gets set to one less than this value - 14.
  TIM1->ARR = 14;
  // This determines the high time: 8/60MHz = 133.3nsec.  The low time is (15-8)/60MHz = 116.7nsec
  TIM1->CCR1 = 8;

  TIM1->PSC = 0;                            // Prescalar is not needed

  // b15..7 = 0: Reserved
  // b6 = 0: No trigger event is generated
  // b5..3 = 0: Reserved
  // b2 = 0: No capture/compare 2 event is generated
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM1->EGR = TIM_EGR_UG;

  // Wait for the update event to complete
  while ((TIM1->SR & 0x0001) == 0x0000)
  {
  }

  // b15..8 = 0: Channel 2 is not used so keep at reset values
  // b7 = 0: External trigger (ETRF) is not used
  // b6..4 = 110: PWM mode 1
  // b3 = 0: TIM1_CCR1 can be written any time
  // b2 = 0: Output compare 1 fast enable is not used
  // b1..0 = 0: CC1 channel is an output
  TIM1->CCMR1 = 0x0060;

  // b15..0 = 0: Channels 3 and 4 are not used so keep at reset values
  TIM1->CCMR2 = 0x0000;

  // b15 = 1: Main output is enabled (needed to turn on the PA8 output)
  // b14..0 = 0: Keep at reset values
  TIM1->BDTR = 0x8000;

  // b15..1 = 0: Active high outputs, OC4..2 disabled
  // b0 = 1: OC1 enabled
  TIM1->CCER = 0x0001;

  TIM1->CR1 |= 0x0002;                      // Disable update events  *** DAH  IF WE ADD CODE TO CHANGE THE
                                            //   FREQUENCY BASED ON TRACKING THE LINE FREQUENCY, WE WILL
                                            //   KEEP THIS BIT CLEAR!!!!

  TIM1->CR1 |= 0x0001;                      // Enable the AFE clock (Timer1)

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer2 Initialization
//
//  MECHANICS:          This subroutine initializes Timer2. It is used to check the time between characters
//                      in a Modbus (received) message.
//                      Timer2 is configured as a downcounter.  It is preloaded with the time for 3.5
//                      characters (signifying the end of a message).  It stops counting whenever the count
//                      reaches zero.  The timer is used when receiving as follows:
//                        In the UART6 receive character interrupt, if the count is zero, the end of a
//                          message was reached, so set a flag and discard the new character.  Otherwise if
//                          the count indicates greater than 1.5 character times has elapsed, the new
//                          character is invalid, so set an error flag, discard the character, and reload
//                          the timer.  Otherwise the time is ok, so store the character and reload the
//                          timer.
//                        In the main loop Modbus routine, if receiving, if the timer is 0,the end of a
//                          message was reached.  Turn off the timer, parse the message, assemble the
//                          response and begin transmitting the response.
//                      Timer2 runs off of the APB1 timer clock.
//
//                      Reference Section 18 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer2 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM2(void)
{

  TIM2->CR1 &= 0xFFFE;                         // Make sure the timer is disabled

  __TIM2_FORCE_RESET();
  __TIM2_RELEASE_RESET();
  __TIM2_CLK_ENABLE();                         // Enable clock to Timer5

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: PWM edge-aligned mode
  // b4 = 1: Counter direction is down
  // b3 = 1: Counter is stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled.  This bit must be cleared to allow updates to the PSC and ARR
  //        registers.  Otherwise, the values remain in the respective preload registers.  I verified this
  //        with the emulator on 190118.  It also must be clear if you want an update event to generate an
  //        interrupt.  Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM2->CR1 = 0x0018;

  // b15..b8 = 0: Reserved
  // b7 = 0: TI1 input connected to TIM4_CH1 pin
  // b6..4 = 000: The UG bit is used as the trigger output (TRGO) (not used)
  // b3 = 0: Capture/compare DMA selection
  // b2..b0 = 0: Reserved
  TIM2->CR2 = 0x0000;



// divide by 30 for 1200 baud, 
// divide by 7 for 4800 baud

/* Dan's original MAG***
  // Timer2 runs off of 2 x APB1 which is 60MHz.  Since we need to measure 3.5 character times at 9600bps,
  // the timer must measure up to about 4msec (11 bits/character).  Since it is a 16-bit counter, set the
  // prescale value to 8, so that the clock is essentially 7.5MHz.
  // Initialize for 3.5 character times at 9600 baud
  TIM2->PSC = 7;                                //   Prescalar = (PSC + 1) = 8
  TIM2->ARR = TIM2_MODB_3P5_9600;
End Dan's original MAG***/

  // set PreSCale and AutoReload Register based on baud rate
  // Prescaler is selected such that 3.5 characters reaches ~full count for 19200 baud,
  // then us increments of the prescaler to use the same AutoReload value.
// TODO: MAG This is a good starting point, but need to measure and adjust as Dan did for 9600
  switch (Setpoints2.stp.Modbus_Port_Baudrate)
  {
    case 0: // 1200 baud
      TIM2->PSC = 31;
      TIM2->ARR = 60156;
      break;
    case 1: // 4800 baud
      TIM2->PSC = 7;
      TIM2->ARR = 60156;
      break;
    case 2: // 9600 baud
      TIM2->PSC = 3;
      TIM2->ARR = 60156;
      break;
    default: // 19200 baud
      TIM2->PSC = 1;
      TIM2->ARR = 60156;
      break;
  }




  // The capture/compare register needs to measure 1.5 bit times.  At 9600bps, this is 1.5625msec
  //  Note, must factor in the time it takes to enter the UART6 receive character interrupt, since this
  //  interrupt is a low priority.   Assume 200usec for now, but *** DAH will need to check this by adding
  //  all the interrupts that could be in front of UART6.  Count is 13220.  Therefore, the character is
  //  valid if the down counter is greater than (27500 - 13220) = 14280.  
//  TIM2->CCR1 = 11720;

  // b15..7 = 0: Reserved
  // b6 = 0: No trigger event is generated
  // b5..3 = 0: Reserved
  // b2 = 0: No capture/compare 2 event is generated
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM2->EGR = TIM_DIER_UIE;

  // Wait for the update event to complete
  while ((TIM2->SR & 0x0001) == 0x0000)
  {
  }

  // b15..8 = 0: Reserved
  // b7 = 0: Master/Slave mode - External event synchronization is not used
  // b6..4 = 0: Internal Trigger 0 is used to synchronize the counter (not used)
  // b3 = 0: Reserved
  // b2..0 = 0: Slave mode is disabled.  The prescaler is clocked directly with the clock (2 x PCLK2)
  TIM2->SMCR = 0x0000;

  // b15..7 = 0: Reserved
  // b6 = 0: Trigger interrupt is disabled
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt is disabled
  // b1 = 0: Capture/Compare 1 interrupt is disabled
  // b0 = 0: Update interrupt is disabled
  TIM2->DIER = 0;

  // b15..11 = 0: Reserved
  // b10 = 0: Clear the capture/compare 2 overcapture flag
  // b9 = 0: Clear the capture/compare 1 overcapture flag
  // b8..7 = 0: Reserved
  // b6 = 0: Clear the trigger interrupt flag
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt flag (read only)
  // b1 = 0: Capture/Compare 1 interrupt flag (read only)
  // b0 = 0: Clear the update interrupt flag
  TIM2->SR = 0x0000;

  // b15..8 = 0: Channel 2 is not used so keep at reset values
  // b7 = 0: External trigger (ETRF) is not used
  // b6..4 = 000: Output compare mode is frozen (not used)
  // b3 = 0: TIM1_CCR1 can be written any time
  // b2 = 0: Output compare 1 fast enable is not used
  // b1..0 = 0: CC1 channel is an output
  TIM2->CCMR1 = 0x0000;

  // b15..8 = 0: Capture/compare channel 4 is not used
  // b7..0 = 0: Capture/compare channel 3 is not used
  TIM2->CCMR2 = 0x0000;

  // b15..8 = 0: Reserved
  // b7..0 = 0: Capture/compare is not used
  TIM2->CCER = 0x0000;

  TIM2->SR &= (~TIM_SR_UIF);                // Clear event flag

  TIM2->CR1 |= 0x0001;                      // Enable the timer

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM2()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM3()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer3 Initialization
//
//  MECHANICS:          This subroutine initializes Timer3. It is used to trigger the ADC conversions about
//                      every 208usec at 60Hz).  Timer3 runs off of the APB1 Timer Clock
//                      (2 x APB1 Clk = 60MHz).  It is configured as an upcounter.  Once the autoreload
//                      value is reached (in Tim3->ARR), the count is set to 0 and an event is generated.
//                      This event triggers ADC conversions.  Interrupts are NOT generated.
//
//                      Reference Section 18 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer3 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM3(void)
{

  TIM3->CR1 &= 0xFFFE;                         // Make sure the timer is disabled

  __TIM3_FORCE_RESET();
  __TIM3_RELEASE_RESET();
  __TIM3_CLK_ENABLE();                         // Enable clock to Timer3

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: Edge triggered mode with counting based on direction bit
  // b4 = 0: Counter direction is up
  // b3 = 0: Counter is not stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled - the TRG0 event is used to trigger an ADC1/2 conversion
  //        This bit must be clear to allow updates to the PSC and ARR registers.  Otherwise, the values
  //        remain in the respective preload registers.  I verified this with the emulator on 190118.
  //        It also must be clear if you want an update event to generate an interrupt.
  //        Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM3->CR1 = 0x0000;

  // b15..b8 = 0: Reserved
  // b7 = 0: TI1 input connected to TIM3_CH1 pin
  // b6..4 = 010: Master Mode selection - Update event selected as trigger output (TRGO)
  // b3 = 0: Capture/compare DMA selection
  // b2..b0 = 0: Reserved
  TIM3->CR2 = 0x0020;                                // an update is selected as the trigger event

  //   Timer3 runs off of 2 x APB1 which is 60 Mhz.  Since we want the ADC's to sample at 4800 Hz, we need a
  //   period of 12500.  The ARR register gets set to one less than this value.
  //   For SYSCLK = 120MHz, TIM3->ARR = 12499;         // Autoreload value (12500 - 1)
  TIM3->ARR = 12499;

  TIM3->PSC = 0;                                //   Prescalar is not needed

  // b15..7 = 0: Reserved
  // b6 = 0: No trigger event is generated
  // b5..3 = 0: Reserved
  // b2 = 0: No capture/compare 2 event is generated
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM3->EGR = TIM_EGR_UG;

  // Wait for update event to complete.  This ensures the counter is not erroneously cleared after it has
  //   been written
  while ((TIM3->SR & 0x0001) == 0x0000)
  {
  }

  // Set initial counter value to half sample time so that the first ADC sample is one-half the sample time
  //   after the AFE sample.  Subsequent samples will be one sample time later (since the counter will reset
  //   to zero each time), so all ADC sampels will be spaced half-way between the AFE samples.  Add 360 to
  //   the initial counter value - this was obtained experimentally (160420) using a scope to check when the
  //   the samples were occurring.  The 360 value is required because there appears to be a delay from when
  //   the timer is enabled to when the timer starts counting, along with interrupt latency, etc.  This
  //   shortens the first ADC sample by about 6usec.
  // Note: Make sure this is after the update event because the update event clears the counter!!!
  TIM3->CNT = (6249 + 360);


  // b15..8 = 0: Reserved
  // b7 = 1: Master/Slave mode - External event synchronization is used (for ADC's) *** DAH NOT SURE IF THIS SHOULD BE SET
  // b6..4 = 0: Internal Trigger 0 is used to synchronize the counter (not used)
  // b3 = 0: Reserved
  // b2..0 = 0: Slave mode is disabled.  The prescaler is clocked directly with the clock (2 x PCLK2)
  TIM3->SMCR = TIM_SMCR_MSM;

  // b15..7 = 0: Reserved
  // b6 = 0: Trigger interrupt is disabled
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt is disabled
  // b1 = 0: Capture/Compare 1 interrupt is disabled
  // b0 = 0: Update interrupt is disabled   (enabled for test case to measure Timer3 timing)
  //TIM3->DIER = TIM_DIER_UIE;   //(DEBUG until DMA interrupt is working)
  TIM3->DIER = 0x0000;


  // b15..11 = 0: Reserved
  // b10 = 0: Clear the capture/compare 2 overcapture flag
  // b9 = 0: Clear the capture/compare 1 overcapture flag
  // b8..7 = 0: Reserved
  // b6 = 0: Clear the trigger interrupt flag
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt flag (read only)
  // b1 = 0: Capture/Compare 1 interrupt flag (read only)
  // b0 = 0: Clear the update interrupt flag
  TIM3->SR = 0x0000;


  TIM3->CCER &= 0xFFEE;                     // Disable channel 1 and channel 2 captures

  // b15..8 = 0: Capture/compare channel 2 is not used
  // b7..0 = 0: Capture/compare channel 1 is not used
  TIM3->CCMR1 = 0x0000;

  // b15..8 = 0: Capture/compare channel 4 is not used
  // b7..0 = 0: Capture/compare channel 3 is not used
   TIM3->CCMR2 = 0x0000;

  // b15..8 = 0: Reserved
  // b7 = 0: For Input 2, capture is triggered on the rising edge of the input (b7 & b5 = 0)
  // b6 = 0: Reserved
  // b5 = 0: For Input 2, capture is triggered on the rising edge of the input (b7 & b5 = 0)
  // b4 = 0: For Input 2, capture is enabled
  // b3 = 0: For Input 1, capture is triggered on the rising edge of the input (b3 & b1 = 0)
  // b2 = 0: Reserved
  // b1 = 0: For Input 1, capture is triggered on the rising edge of the input (b3 & b1 = 0)
  // b0 = 0: For Input 1, capture is enabled
  TIM3->CCER = 0x0000;

  // Timer will be enabled in EnDisPeripherals() when 120MHz is stable
  //  TIM3->CR1 |= 0x0001;                      // Enable the timer

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM3()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM4()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer4 Initialization
//
//  MECHANICS:          This subroutine initializes Timer4. Timer 4 is a 16-bit counter/timer with up to
//                      four I/O channels to support capture/compare operations.  Channels 3 and 4 are
//                      to measure the load frequency and line frequency respectively as well as phase
//                      difference/sync between line and load.  logic-level square waves corresponding
//                      to the phase A load and line voltages are brought into the T3 and T4 inputs
//                      respectively. Timer4 is configured as an upcounter.  The autoreload value is 0xFFFF,
//                      so basically it operates as a free-running counter.  Channels 3 and 4 are configured
//                      for Input Capture mode.  Each channel will capture the counter value on each rising
//                      edge of its corresponding input input.
//                      The clock frequency is set to 1.5MHz (at 120MHz, 2 x APB2 is 60MHz, the timer input
//                      clock is 60MHz/40 = 1.5MHz).  This provides a resolution of 667nsec.  At 60Hz, the
//                      accuracy should be about .0024Hz.
//                      A slight filter is applied to the inputs - four consecutive readings must be
//                      obtained in order for a transition to be recognized.  The firmware could average
//                      multiple readings to eliminate any inaccuracies due to this and to improve the
//                      accuracy.
//
//                      Reference Section 18 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer4 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM4(void)
{

  TIM4->CR1 &= 0xFFFE;                         // Make sure the timer is disabled

  __TIM4_FORCE_RESET();
  __TIM4_RELEASE_RESET();
  __TIM4_CLK_ENABLE();                         // Enable clock to Timer4

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: Edge triggered mode with counting based on direction bit
  // b4 = 0: Counter direction is up
  // b3 = 0: Counter is not stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled - TRGO is used to generate an interrupt
  //        This bit must be clear to allow updates to the PSC and ARR registers.  Otherwise, the values
  //        remain in the respective preload registers.  I verified this with the emulator on 190118.
  //        It also must be clear if you want an update event to generate an interrupt.
  //        Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM4->CR1 = 0x0000;

  // not using Master/Slave, so don't care about this register
  // b15..b8 = 0: Reserved
  // b7 = 0: TI1 input connected to TIM4_CH1 pin
  // b6..4 = 010: Master Mode selection - Update event selected as trigger output (TRGO)
  // b3 = 0: Capture/compare DMA selection
  // b2..b0 = 0: Reserved
  TIM4->CR2 = 0x0020;

  // not using Master/Slave, so disable in this register
  // b15..8 = 0: Reserved
  // b7 = 0: Master/Slave mode - External event synchronization is not used
  // b6..4 = 0: Internal Trigger 0 is used to synchronize the counter (not used)
  // b3 = 0: Reserved
  // b2..0 = 0: Slave mode is disabled.  The prescaler is clocked directly with the clock (2 x PCLK2)
  TIM4->SMCR = 0x0000;
  
  // b15, 13, 7, and 5 = 0: Reserved
  // b14, 12, 11, 10, 9, and 8 = 0, no DMA requests enabled
  // b6 = 0: Trigger interrupt is disabled
  // b4 = 1: Capture/Compare 4 interrupt is enabled
  // b3 = 1: Capture/Compare 3 interrupt is enabled
  // b2 = 0: Capture/Compare 2 interrupt is disabled
  // b1 = 0: Capture/Compare 1 interrupt is disabled
  // b0 = 0: Update interrupt is disabled
  TIM4->DIER = 0x0018;

  // b15..13 = 0: Reserved
  // b12, 11, 10, and 9 = 0: Clear all capture/compare overcapture flags
  // b8..7 = 0: Reserved
  // b6 = 0: Clear the trigger interrupt flag
  // b5 = 0: Reserved
  // b4, 3, 2, and 1 = 0: Clear all capture/Compare interrupt flags
  // b0 = 0: Clear the update interrupt flag
  TIM4->SR = 0x0000;

  // b15..8 = 0: Capture/compare channel 2 is not used
  // b7..0 = 0: Capture/compare channel 1 is not used
  TIM4->CCMR1 = 0x0000;

  // b15..12 = 0100: Input Capture 4 filter set for four consecutive reading
  // B11..10 = 00: no prescaler, capture every time
  // B9..8 = 01: Capture/compare channel 4 is configured as an input, IC4 is mapped to TI4
  // b7..4 = 0100: Input Capture 3 filter set for four consecutive reading
  // B3..2 = 00: no prescaler, capture every time
  // B1..0 = 01: Capture/compare channel 3 is configured as an input, IC3 is mapped to TI3
  TIM4->CCMR2 = 0x4141;

  // b15 = 0: Capture/compare 4 configured for noninverted/rising edge input
  // b14 = 0: Reserved
  // b13 = 0: Capture/compare 4 configured for noninverted/rising edge input
  // b12 = 1: Capture/compare 4 capture enabled
  // b11 = 0: Capture/compare 3 configured for noninverted/rising edge input
  // b10 = 0: Reserved
  // b9 = 0: Capture/compare 3 configured for noninverted/rising edge input
  // b8 = 1: Capture/compare 3 capture enabled
  // b7..0 = 0: Capture/compare 2 and 1 are not used
  TIM4->CCER = 0x1100;

  TIM4->CNT = 0;

  // Counter frequency should be as fast as possible for greatest resolution while not overrunning the
  //   16-bit counter at the minimum frequency.  Minimum frequency in the requirements is 45Hz.  We also
  //   will use the generic 10msec timer to detect an overrun.  This can be set for 40msec (if the 10msec
  //   count is greater than or equal to 4, there is an overrun).  Depending on when the counter is cleared,
  //   the overrun can occur at 40msec (25Hz) or 30msec (33Hz).  The counter must be able to handle 25Hz, so
  //   the frequency must be less than 25Hz * 65535, or 1.64MHz.  We will set the frequency to 1.5MHz.
  //   At this frequency, the minimum frequency is 23Hz (although the overrun counter limits the value to
  //   25Hz).  The resolution is 667nsec.  At 60Hz, the resolution is .004%.  Even at 440Hz, the resolution
  //   is .03%., which is still less than the accuracy requirement of .2Hz.
  // Set prescalar value.  Clock input is 2 x PCLK2.
  // For SYSCLK = 120MHz, 2 x APB2 = 60MHz, so divide by 40
  // Note, the divisor is (PSC + 1)
  TIM4->PSC = 39;

  TIM4->ARR = 0xFFFF;  // for free-running 16 bit counter

// b15..7 = 0: Reserved
// b6 = 0: No trigger event is generated
// b5 = 0: Reserved
// b4 = 1: Capture/compare 4 event is generated
// b3 = 1: Capture/compare 3 event is generated
// b2 = 0: No capture/compare 2 event is generated
// b1 = 0: No capture/compare 1 event is generated
// b0 = 1: Generate update event to load PSC and ARR
TIM4->EGR = 0x0019;

  // Wait for update event to complete
  while ((TIM4->SR & 0x0001) == 0x0000)
  {
  }

  // TIM4->CCR1 Capture/compare 1 not used
  // TIM4->CCR2 Capture/compare 2 not used
  // TIM4->CCR3 Capture 3 is read-only
  // TIM4->CCR4 Capture 4 is read-only

  // DMA Control not used
  TIM4->DCR = 0;
  // TIM4->DMAR DMA Control not used

  // Timer will be enabled in EnDisPeripherals()
  //  TIM4->CR1 |= 0x0001;                      // Enable the timer
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM4()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM5()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer5 Initialization
//
//  MECHANICS:          This subroutine initializes Timer5. It is used to trigger an ADC3 conversion on
//                      input 2 (coil temperature).  Conversions are to occur at 4000Hz, so the timer
//                      triggers every 250usec.  Note, the timer output is actually a 2KHz signal (250usec
//                      High, 250usec Low).  The ADC is configured to have conversions occur on both the
//                      rising and falling edge.
//                      Timer5 runs off of the APB1 Timer Clock (2 x 30MHz = 60MHz).  It is configured as an
//                      upcounter.  Once the autoreload value is reached (in Tim5->ARR), the count is set to
//                      0 and an interrupt is generated.
//
//                      Reference Section 18 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer5 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM5(void)
{

  TIM5->CR1 &= 0xFFFE;                         // Make sure the timer is disabled

  __TIM5_FORCE_RESET();
  __TIM5_RELEASE_RESET();
  __TIM5_CLK_ENABLE();                         // Enable clock to Timer5

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: PWM edge-aligned mode
  // b4 = 0: Counter direction is up
  // b3 = 0: Counter is not stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled.  This bit must be cleared to allow updates to the PSC and ARR
  //        registers.  Otherwise, the values remain in the respective preload registers.  I verified this
  //        with the emulator on 190118.  It also must be clear if you want an update event to generate an
  //        interrupt.  Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM5->CR1 = 0x0000;

  // b15..b8 = 0: Reserved
  // b7 = 0: TI1 input connected to TIM4_CH1 pin
  // b6..4 = 100: Master Mode selection - Compare event selected as trigger output (TRGO)
  // b3 = 0: Capture/compare DMA selection
  // b2..b0 = 0: Reserved
  TIM5->CR2 = 0x0040;

  // Timer5 runs off of 2 x APB1 which is 60MHz.  Since we want an event every 250usec, the divisor must
  // be 15,000.  We will use a prescale value of 6, so that the clock is essentially 10MHz, and the
  // divisor is 2,500.  The ARR register gets set to one less than this value.
  // For SYSCLK = 120MHz, TIM5->ARR = 2499;          // Autoreload value (2500 - 1)
  TIM5->ARR = 2499;

  TIM5->CCR1 = 2499;

  TIM5->PSC = 5;                                //   Prescalar = (PSC + 1) = 6

  // b15..7 = 0: Reserved
  // b6 = 0: No trigger event is generated
  // b5..3 = 0: Reserved
  // b2 = 0: No capture/compare 2 event is generated
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM5->EGR = TIM_EGR_UG;

  // Wait for update event to complete
  while ((TIM5->SR & 0x0001) == 0x0000)
  {
  }

  TIM5->CNT = 0;

  // b15..8 = 0: Reserved
  // b7 = 0: Master/Slave mode - External event synchronization is not used
  // b6..4 = 0: Internal Trigger 0 is used to synchronize the counter (not used)
  // b3 = 0: Reserved
  // b2..0 = 0: Slave mode is disabled.  The prescaler is clocked directly with the clock (2 x PCLK2)
  TIM5->SMCR = 0x0000;

  // b15..7 = 0: Reserved
  // b6 = 0: Trigger interrupt is disabled
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt is disabled
  // b1 = 0: Capture/Compare 1 interrupt is disabled
  // b0 = 0: Update interrupt is disabled
  TIM5->DIER = 0;

  // b15..11 = 0: Reserved
  // b10 = 0: Clear the capture/compare 2 overcapture flag
  // b9 = 0: Clear the capture/compare 1 overcapture flag
  // b8..7 = 0: Reserved
  // b6 = 0: Clear the trigger interrupt flag
  // b5..3 = 0: Reserved
  // b2 = 0: Capture/Compare 2 interrupt flag (read only)
  // b1 = 0: Capture/Compare 1 interrupt flag (read only)
  // b0 = 0: Clear the update interrupt flag
  TIM5->SR = 0x0000;

  // b15..8 = 0: Channel 2 is not used so keep at reset values
  // b7 = 0: External trigger (ETRF) is not used
  // b6..4 = 011: Toggle mode
  // b3 = 0: TIM1_CCR1 can be written any time
  // b2 = 0: Output compare 1 fast enable is not used
  // b1..0 = 0: CC1 channel is an output
  TIM5->CCMR1 = 0x0030;

  // b15..8 = 0: Capture/compare channel 4 is not used
  // b7..0 = 0: Capture/compare channel 3 is not used
  TIM5->CCMR2 = 0x0000;

  // b15..8 = 0: Reserved
  // b7..0 = 1: OC1 output is active to trigger ADC3
  TIM5->CCER = 0x0001;

  TIM5->CR1 |= 0x0002;                      // Disable update events

  TIM5->CR1 |= 0x0001;                      // Enable the timer

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM5()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM8()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer8 Initialization
//
//  MECHANICS:          This subroutine initializes Timer8. It is used to generate an external 4800Hz square
//                      wave that is used to measure the Rogowski coil temperatures.
//                      Timer8 runs off of the APB2 Timer Clock (2 x APB2 = 60MHz).  It is configured as an
//                      upcounter in edge-aligned PWM mode.  The autoreload value (in TIM8->ARR) is 12499.
//                      When the count reaches 12499, on the next clock pulse, the counter rolls over to 0
//                      and an event is generated.  The frequency is therefore 60MHz/12500 = 4800.
//                      Interrupts are NOT generated.
//
//                      Reference Section 17 of the Programmer's Reference Manual (RM0090).
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             Timer8 registers
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_TIM8(void)
{

  TIM8->CR1 &= 0xFFFFFFFE;                  // Make sure the timer is disabled

  RCC->APB2RSTR |= (RCC_APB2RSTR_TIM8RST);
//  __TIM8_FORCE_RESET();
  RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM8RST);
//  __TIM8_RELEASE_RESET();
  RCC->APB2ENR |= (RCC_APB2ENR_TIM8EN);
//  __TIM8_CLK_ENABLE();                      // Enable clock to Timer8

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..5 = 0: PWM edge-aligned mode
  // b4 = 0: Counter direction is up
  // b3 = 0: Counter is not stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled.  This bit must be cleared to allow updates to the PSC and ARR
  //        registers.  Otherwise, the values remain in the respective preload registers.  I verified this
  //        with the emulator on 190118.  It also must be clear if you want an update event to generate an
  //        interrupt.  Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM8->CR1 = 0x0000;

  // b15 = 0: Reserved
  // b14..8 = 0: Output idle state = 0
  // b7 = 0: TI1 input connected to TIM8_CH1 pin
  // b6..4 = 000: Master Mode selection - Trigger output is not used so keep at reset value
  // b3 = 0: Capture/compare DMA selection (not used)
  // b2 = 0: Capture/compare control update selection - not used so keep at reset value
  // b1 = 0: Reserved
  // b0 = 0: Capture/compare bits are not preloaded
  TIM8->CR2 = 0x0000;

  // Timer8 runs off of 2 x APB2 which is 60MHz.  Since we want a 4800Hz output, we need to divide by 12500.
  // The ARR register gets set to one less than this value - 12499.
  TIM8->ARR = 12499;

  // This determines the high time: 6250/60MHz = 104usec.  The low time is (12499-6250)/60MHz = 104usec
  TIM8->CCR1 = 6250;

  TIM8->PSC = 0;                            // Prescalar is not needed

  // b15..0 = 0: This is the slave mode control register - not used so keep at reset values
  //             (slave mode is disabled)
  TIM8->SMCR = 0x0000;

  // b15..0 = 0: All interrupts disabled
  TIM8->DIER = 0x0000;

  // b15..7 = 0: Reserved
  // b6 = 0: No trigger event is generated
  // b5..3 = 0: Reserved
  // b2 = 0: No capture/compare 2 event is generated
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM8->EGR = TIM_EGR_UG;

  // Wait for update event to complete
  while ((TIM8->SR & 0x0001) == 0x0000)
  {
  }

  // Initialize counter
  TIM8->CNT = 0;

  // b15..8 = 0: Channel 2 is not used so keep at reset values
  // b7 = 0: External trigger (ETRF) is not used
  // b6..4 = 110: PWM mode 1
  // b3 = 0: TIM8_CCR1 can be written any time
  // b2 = 0: Output compare 1 fast enable is not used
  // b1..0 = 0: CC1 channel is an output
  TIM8->CCMR1 = 0x0060;

  // b15..8 = 0: Capture/compare channel 4 is not used
  // b7..0 = 0: Capture/compare channel 3 is not used
  TIM8->CCMR2 = 0x0000;

  // b15 = 1: Main output is enabled (needed to turn on the PA8 output)
  // b14..0 = 0: Keep at reset values
  TIM8->BDTR = 0x8000;

  // b15..1 = 0: Active high outputs, OC4..2 disabled
  // b4 = 1: OC1N enabled
  TIM8->CCER = 0x0004;

  TIM8->CR1 |= 0x0002;                      // Disable update events

//  TIM8->CR1 |= 0x0001;                    // Enable the clock (Timer8) Only turn on when measuring

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM8()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_TIM10()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer10 Initialization
// 
//  MECHANICS:          This subroutine initializes Timer10. It is used to trigger an interrupt for SPI2 and
//                      event handling.  The interrupt is triggered every 1.5msec.  Timer10 runs off of the
//                      APB1 Timer Clock (2 x APB1 = 60MHz).  It is configured as an upcounter.  Once the
//                      autoreload value is reached (in Tim10->ARR), the count is set to 0 and an interrupt
//                      is generated.

//                      Reference Section 19 of the Programmer's Reference Manual (RM0090).
//                          
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             Timer10 registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_TIM10(void)
{

  TIM10->CR1 &= 0xFFFE;                          // Make sure the timer is disabled

  __TIM10_FORCE_RESET();
  __TIM10_RELEASE_RESET();
  __TIM10_CLK_ENABLE();

  // b15..b10 = 0: Reserved
  // b9..8 = 0: Sampling clock for the digital filters = timer clock
  // b7 = 0: Auto-reload register is not buffered
  // b6..4 = 0: Reserved = 0
  // b3 = 1: Counter is stopped on the update event
  // b2 = 0: Counter overflow or UG bit causes an update interrupt event
  // b1 = 0: Update events are enabled.  This bit must be cleared to allow updates to the PSC and ARR
  //        registers.  Otherwise, the values remain in the respective preload registers.  I verified this
  //        with the emulator on 190118.  It also must be clear if you want an update event to generate an
  //        interrupt.  Reference Section 18.3.1 in RM0090.
  // b0 = 0: Disable the counter (for now)
  TIM10->CR1 = 0x0008;

  // b15..2 = 0: Reserved
  // b1 = 0: Capture/Compare 1 interrupt is disabled
  // b0 = 1: Update interrupt is enabled
  TIM10->DIER = 0x0001;

  // b15..10 = 0: Reserved
  // b9 = 0: Clear the capture/compare 1 overcapture flag
  // b8..2 = 0: Reserved
  // b1 = 0: Capture/Compare 1 interrupt flag
  // b0 = 0: Clear the update interrupt flag
  TIM10->SR = 0x0000;

  // Capture/Compare is not used
  // b15..8 = 0: Reserved
  // b7..4 = 0: No filter
  // b3..2 = 0: No prescaler
  // b1..0 = 0: Capture/compare channel 1 is configured as an output
  TIM10->CCMR1 = 0x0000;

  // Capture/Compare is not used
  // b15..4 = 0: Reserved
  // b3 = 0: For Input 1, capture is triggered on the rising edge of the input (b3 & b1 = 0)
  // b2 = 0: Reserved
  // b1 = 0: For Input 1, capture is triggered on the rising edge of the input (b3 & b1 = 0)
  // b0 = 0: For Input 1, capture is disabled
  TIM10->CCER = 0x0000;

  TIM10->CNT = 0x0000;
  
  // Timer10 runs off of 2 x APB1 which is 60MHz.  Since we want an interrupt every 1.5msec, the divisor must
  // be 90,000.  We will use a prescale value of 6, so that the clock is essentially 10MHz, and the
  // divisor is 15,000.  The ARR register gets set to one less than this value.
  // For SYSCLK = 120MHz, TIM4->ARR = 14999;         // Autoreload value (15000 - 1)
  // Add 2% margin = 15300, so use 15299
  TIM10->ARR = 15299;

  TIM10->PSC = 5;                                //   Prescalar = (PSC + 1) = 6

  // TIM10->CCR1 Capture/compare 1 is not used

  // b15..2 = 0: Reserved
  // b1 = 0: No capture/compare 1 event is generated
  // b0 = 1: Generate update event to load PSC and ARR
  TIM10->EGR = TIM_EGR_UG;

  // Wait for update event to complete
  while ((TIM10->SR & 0x0001) == 0x0000)
  {
  }
  
  TIM10->CR1 |= 0x0001;                      // Enable the timer

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_TIM10()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_SysTick()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           System Timer Initialization
// 
//  MECHANICS:          This subroutine initializes the system timer.  This timer is used for general-purpose
//                      timing functions, such as blinking LEDs, events time-stamps, etc.  An interrupt is
//                      generated every 10msec.
//                      The system timer is a standard ARM-4 peripheral.  It contains four registers:
//                          0xE000E010      CTRL    Control and Status
//                          0xE000E014      LOAD    Reload Value
//                          0xE000E018      VAL     Present value
//                          0xE000E01C      CALIB   Calibration
//                      The input clock is HCLK/8.  HCLK is the HSI clock after initialization (16MHz) or
//                      120MHz (PLLCLK running off of a 25MHz HSE clock).  The counter simply decrements each
//                      cycle.  When it reaches 0, an interrupt is generated and the counter is reinitialized
//                      with the reload value.
//                      Reference ARM-4 documentation for further information.
//                          
//  CAVEATS:            None
// 
//  INPUTS:             SysClk_120MHz - TRUE if system clock is 120MHz (PCLK2 = 30MHz)
//                                    - FALSE if system clock is 16MHz (PCLK2 = 16MHz)
// 
//  OUTPUTS:            None
//
//  ALTERS:             SysTick registers, SCB->SHP[11] register
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_SysTick(void)
{

  // b31..b17: Reserved = 0
  // b16:      Read-only flag.  Set when counter reaches 0.  Cleared when register is read or present value
  //           (VAL) register is cleared
  // b15..3:   Reserved = 0
  // b2 = 0:   Core clock/8 is used
  // b1 = 1:   SysTick interrupt is enabled
  // b0 = 0:   Timer is disabled
  SysTick->CTRL  = 0x00000002;
  // Load up reload and counter registers with appropriate values for 10msec tick, depending on the system
  //   clock
  if (SysClk_120MHz == TRUE)                    // SYSCLK = 120MHz, (input clock is 15MHz)
  {
    SysTick->LOAD  = COUNT_120MHZ_10MSEC;              // Initialize reload value
    SysTick->VAL   = COUNT_120MHZ_10MSEC;              // Initialize present value
  }
  else                                          // SYSCLK = 16MHz (input clock is 2MHz)
  {
    SysTick->LOAD  = COUNT_16MHZ_10MSEC;               // Initialize reload value
    SysTick->VAL   = COUNT_16MHZ_10MSEC;               // Initialize present value
  }

  // b31..28 = 0xF: SysTick priority = 15 (Group=3, Sub=3)   (ref Section 4.4.8 in PM0214)
  // b27..24:       Not used = 0
  // b23..20:       PendSV interrupt not used
  // b19..16:       Not used = 0
  // b15..0:        Reserved = 0
  SCB->SHP[11] = 0xF0;

  SysTick->CTRL |= 0x00000001;                  // Enable SysTick

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_SysTick()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_InterruptStruct()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Interrupt Structure Initialization
// 
//  MECHANICS:          This subroutine initializes the interrupt structure.  The STM32F407 is configured
//                      for 4 Group Priorities and 4 Sub Priorities.  The lower the number, the higher the
//                      priority.  A higher-priority-group interrupt can interrupt a lower-priority group
//                      interrupt service routine.  Within the same group, a higher-subpriority interrupt
//                      will NOT interrupt a lower-subpriority interrupt.  However, it will be serviced
//                      before the lower-subpriority interrupt, if both interrupts are pending.
//                      The following interrupts are used:
//
//                        Interrupt       Description                                Position Priority Level
//                                                                                              Group/Sub
//                        SysTick Timer   General-purpose timer tick every 10msec        -         3/3
//                        UART6Rx         Modbus receiver full                          71         3/3
//                        UART5Rx         Test port receiver full                       53         3/3
//                        TIM10           Timer 10 (SPI2/Events Manager)                25         3/2
//                        TIM4            Timer 4 (Load/Line Freq/Sync Measurement)     30         3/3
//                        I2C2Rx          Override micro interface receiver full        33         3/0
//                        I2C2Tx          Override micro interface transmitter empty    33         3/0
//                        I2C3Rx          Health Bus interface receiver full            72         3/0
//                        I2C3Tx          Health Bus interface transmitter empty        72         3/0
//                        I2C3Error       Health Bus interface error                    73         3/0
//                        DMA2 Stream 0   DMA for ADC1/ADC2                             56         2/1
//                        DMA1 Stream 0   DMA for SPI3 (AFE) Rx complete                11         2/1
//                        PD8             AFE Data Ready                                23         1/0
//                        PH9             Time Sync                                     23         1/0
//
//                      Note, the "Position" determines the register and bit-field location of the data that
//                      configures the interrupt.  It is taken from Table 61 in RM0090 (the STM32F407
//                      Reference Manual).
//                      Also note, the SysTick Timer interrupt is handled in Init_SysTick().
//
//                      The peripherals that generate the interrupts are configured in separate subroutines.
//
//                      Reference ARM-4 documentation and PM0214, Section 4.4 for further information.
//                          
//  CAVEATS:            Interrupts should be disabled before calling this subroutine
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             SYSCFG registers, EXTI registers,
//                      SCB->AIRCR register
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_InterruptStruct(void)
{
  // Disable (turn off) all interrupts
  NVIC->ICER[0] = 0xFFFFFFFF;               // Interrupts are disabled by writing a one to the location
  NVIC->ICER[1] = 0xFFFFFFFF;
  NVIC->ICER[2] = 0xFFFFFFFF;

  // *** DAH  ADD CODE TO CLEAR ALL INTERRUPTS IN THE REGISTERS!!
  
  // Configure the interrupt priority scheme - 4 Group (preempt) priorities, 4 subgroup priorities

  // b31..b16: Register key = 0x05FA in order for write to be accepted
  // b15:      Read-only bit.  0 = Little-endian data
  // b14..11:  Reserved = 0
  // b10..8:   Priority group setting = 5 - Group priorities=4, Subpriorities=4 (Table 51, PM0214)
  // b7..3:    Reserved = 0
  // b2 = 0:   No system reset request
  // b1 = 0:   Reserved for debug use (must be 0)
  // b0 = 0:   Reserved for debug use (must be 0)
  SCB->AIRCR =  0x05FA0500;

  // Set up external interrupts (from GPIO pins)...

  // Map the GPIO pin to the external interrupt (ref Section 9 of RM0090)
  __SYSCFG_CLK_ENABLE();                    // Enable the clock

  // b15..12 = 0: EXTI3 = PA3 (not used)
  // b11..8 = 0: EXTI2 = PA2 (not used)
  // b7..4 = 0: EXTI1 = PA1 (not used)
  // b3..0 = 0: EXTI0 = PA0 (not used)
  SYSCFG->EXTICR[0] = 0x0000;

  // b15..12 = 0: EXTI7 = PA7 (not used)
  // b11..8 = 0: EXTI6 = PA6 (not used)
  // b7..4 = 0: EXTI5 = PA5 (not used)
  // b3..0 = 0: EXTI4 = PA4 (not used)
  SYSCFG->EXTICR[1] = 0x0000;

  // b15..12 = 0: EXTI11 = PA11 (not used)
  // b11..8 = 5: EXTI10 = PA10 (not used)
  // b7..4 = 7: EXTI9 = PH9
  // b3..0 = 3: EXTI8 = PD8
  SYSCFG->EXTICR[2] = 0x0073;

  // b15..12 = 0: EXTI15 = PA15 (not used)
  // b11..8 = 6: EXTI14 = PA14 (not used)
  // b7..4 = 0: EXTI13 = PA13 (not used)
  // b3..0 = 0: EXTI12 = PA12 (not used)
  SYSCFG->EXTICR[3] = 0x0000;

  // Unmask the external interrupts in the interrupt mask register (ref Section 12.3 in RM0090)
  EXTI->IMR = 0x00000300;                   // PD8, PH9
  EXTI->EMR = 0x00000000;                   // Events are not used, so clear all bits

  // Initialize the edge detection registers.  PD8, PH9 = falling edge
  EXTI->RTSR = 0x00000000;                  // Rising edge register
  EXTI->FTSR = 0x00000300;                  // Falling edge register

  // Clear pending external interrupts.  This also clears software interrupts in SWIER register
  EXTI->PR = 0xFFFFFFFF;                    // Bits are cleared by writing a one to the location


  // Set up the interrupt priorities...  Note, NVIC->IP is a byte array, so the position can be used as the
  //   index without having to divide by four.  Also, SysTick is handled in Init_SysTick()
  // Reference Section 4.3.7 and Table 51 of PM0214
  // Bits 7..4 hold the priority.  Bits 3..0 are not used.
  NVIC->IP[71] = 0xF0;                      // UART6 Group = 3, Subgroup = 3
  NVIC->IP[53] = 0xF0;                      // UART5 Group = 3, Subgroup = 3
  NVIC->IP[25] = 0xE0;                      // TIM10, Group = 3, Subgroup = 2
  NVIC->IP[33] = 0xC0;                      // I2C2 Rx/Tx  Group = 3, Subgroup = 0
  NVIC->IP[72] = 0xC0;                      // I2C3 Rx/Tx Group = 3, Subgroup = 0
  NVIC->IP[73] = 0xC0;                      // I2C3 Error  Group = 3, Subgroup = 0
  NVIC->IP[56] = 0x90;                      // DMA2  Group = 2, Subgroup = 1 (for ADC1/ADC2)
  NVIC->IP[11] = 0x90;                      // DMA1  Group = 2, Subgroup = 1
  NVIC->IP[23] = 0x40;                      // PD8, PH9  Group = 1, Subgroup = 0
  NVIC->IP[30] = 0xF0;                      // TIM4  Group = 3, Subgroup = 3


  // Set up the peripheral interrupts...

  // Enable interrupts.  Note, this assumes 120MHz operation.  If SYSCLK is 16MHz, the 120MHz peripherals *** DAH LEAVE DISABLED FOR NOW
  //   will be off, so no interrupts will occur
  NVIC->ISER[0] = 0x46800800;               // DMA1 Stream 0 (position 11 = b11), PD8, PH9 (pos 23 = b23),
                                            //   TIM10 (pos 25 = b25), TIM11 (pos 26 = b26),
                                            //   TIM4 (pos 30 = b30)
  NVIC->ISER[1] = 0x01200002;               // I2C2 (pos 33 = b1), UART5 (pos 53 = b21),
                                            //   DMA2 Stream 0 (position 56 = b24)
  NVIC->ISER[2] = 0x00000380;               // UART 6 (POS 71 =b7), I2C3 Rx/Tx, Error (pos 72, 73 = b8, 9)
                                            //   

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_InterruptStruct()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        EnDisPeripherals()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Enable and Disable Peripherals
// 
//  MECHANICS:          This subroutine enables and disables SYSCLK-dependent peripherals based on whether
//                      the system clock (SYSCLK) is operating at 120MHz (PLL) or 16MHz (HSI).
//
//                      The following is a list of the SYSCLK-dependent peripherals:
//                        Peripheral      Description                                   Enabled?
//                                                                                   16MHz   120MHz
//                        Timer3          Used to trigger the STM32 ADC's              N       N
//                        Timer4          Used to measure load/line frequency          N       Y
//                        UART2           Display port interface                       N       Y
//                        I2C2            Override micro interface                     N       Y
//                        UART1           CAM1 port interface                          N       Y
//                        UART6           CAM2 port interface                          N       Y
//                        ADC1            A/D converter                                N       Y
//                        ADC2            A/D converter                                N       Y
//
//                      Note, Timer 3 is disabled if we do not have 120MHz.  It is enabled, however, in the
//                      AFE Sampling ISR so that the ADC and AFE samples are aligned.
//                          
//  CAVEATS:            This subroutine does not perform any other configuration of the peripherals.  It
//                      merely enables or disables the peripherals.  It is assumed the peripherals have
//                      already been configured.
// 
//  INPUTS:             SysClk_120MHz - TRUE if system clock is 120MHz (PCLK2 = 30MHz)
//                                    - FALSE if system clock is 16MHz (PCLK2 = 16MHz)
// 
//  OUTPUTS:            None
//
//  ALTERS:             Enable peripheral bit in CR1 for I2C2, USART1, USART2, USART3, USART6, Timer3,
//                      Timer9, ADC1, and ADC2
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void EnDisPeripherals(void)
{
  
  // These peripherals are enabled if SYSCLK is 120MHz PLL, disabled if SYSCLK is 16MHz HSI
  if (SysClk_120MHz == TRUE)
  {
    I2C2->CR1 |= 0x0001;                  // b0 = 1: I2C peripheral is enabled
    I2C2->CR1 |= 0x0400;                  // b10 = 1: Send ACK when char received - peripheral must be enabled first
    I2C3->CR1 |= 0x0001;                  // b0 = 1: I2C peripheral is enabled
//    I2C3->CR1 |= 0x0400;                  // b10 = 1: Send ACK when char received - peripheral must be enabled first

    USART1->CR1 |= 0x00002000;            // b13 = 1: UART1 is enabled
    USART2->CR1 |= 0x00002000;            // b13 = 1: UART2 is enabled
    USART3->CR1 |= 0x00002000;            // b13 = 1: UART3 is enabled
    USART6->CR1 |= 0x00002000;            // b13 = 1: UART6 is enabled
    ADC1->CR2 |= 0x00000001;              // Turn on the ADCs
    ADC2->CR2 |= 0x00000001;
    TIM4->CR1 |= 0x0001;                 // b0 = 1: Enable the counter
  }
  else
  {
    I2C2->CR1 &= 0xFFFE;                  // b0 = 0: I2C peripheral is disabled
    I2C3->CR1 &= 0xFFFE;                  // b0 = 0: I2C peripheral is disabled

    USART1->CR1 &= 0xFFFFDFFF;            // b13 = 0: UART1 is disabled
    USART2->CR1 &= 0xFFFFDFFF;            // b13 = 0: UART2 is disabled
    USART3->CR1 |= 0xFFFFDFFF;            // b13 = 1: UART3 is enabled
    USART6->CR1 &= 0xFFFFDFFF;            // b13 = 0: UART6 is disabled
    ADC1->CR2 &= 0xFFFFFFFE;              // Turn off the ADCs
    ADC2->CR2 &= 0xFFFFFFFE;
    TIM3->CR1 &= 0xFFFE;                  // b0 = 0: Disable the counter
    TIM4->CR1 &= 0xFFFE;                 // b0 = 0: Disable the counter
  }


}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        EnDisPeripherals()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AFE_Init()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           AFE Initialization
// 
//  MECHANICS:          This subroutine initializes the AFE as follows:
//                          Low Power Mode
//                          Output data rate = 4800 samples per second
//                          
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              AFE_SPI_Xfer()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 121usec
// 
//------------------------------------------------------------------------------------------------------------

void AFE_Init(void)
{
  volatile uint16_t temp;
  uint16_t i;
  uint16_t addr;

  //--------------------------------------------------------------------------------------------------------
  //                                    AFE Registers
  //
  //  Address       Name           Reset Val   Written Val    Comments
  // x00 - x07   CHx_CONFIG           x00      Not written    Use reset val: AFE gain = 1, Diagnostic mux disabled
  // x08         CH_DISABLE           x00      Not written    Use reset val: All channels enabled
  // x09 - x10   CHx_SYNC_OFFSET      x00          TBD        Used to adjust phase between current and voltage channels
  //                                                            Will get values during calibration
  // x11         USER_CONFIG1         x24          x00        Low power mode, VCM and Ref Out buffers OFF, SAR OFF, OSC OFF
  // x12         USER_CONFIG2         x09          x09        Use reset val: 1 fixed notch filter, sync start value
  //                                                            from mm is high
  // x13         USER_CONFIG3         x80          x90        Use reset val: SAR not used so keep reset values, enable
  //                                                            SPI mode, no clock delay on any channels *** DAH MAY CHANGE THIS FOR SYNCING
  // x14         DOUT_FORMAT          x20          x00        Use status header has status bits, not CRC header
  // x15         ADC_MUX_CONFIG       x00      Not written    External reference, SAR input mux is off (SAR is not used)
  // x16         GLOBAL_MUX_CONFIG    x00      Not written    Use reset val: diagnostics not used so just keep reset vals
  // x17         GPIO_CONFIG          x00      Not written    Use reset val: GPIO configured as inputs (ext pull-downs)
  // x18         GPIO_DATA            x00      Not written    Use reset val: not used so just keep reset vals
  // x19         BUFFER_CONFIG1       x38          x00        Reference buffers in Bypass mode
  // x1A         BUFFER_CONFIG2       xC0          x00        Reference buffers in Bypass mode.  Use reset values for other bits
  // x1C - x1E   CH0_OFFSET           x00      Not written    Will probably write in at calibration *** DAH  NEED TO SEE WHETHER ADI FACTORY CALIBRATES
  // x1F - x21   CH0_GAIN             x00      Not written    May write in at calibration - looks like the ADI factory writes values to this reg
  // x22 - x4B   CHx_OFFSET, _GAIN    x00      Not written      *** DAH NEED TO DETERMINE HOW CALIBRATION WILL BE DONE
  // x4C - x53   CHx_ERR              x00      Not written    All undervoltage, overvoltage, and reference errors cleared
  // x54 - x57   CHx_x_SAT_ERR        x00      Not written    All modulator, filter, and ADC saturation errors cleared
  // x58         CH_ERR_REG_EN        xFE          x01        Disable output, filter, modulator saturation tests.
  //                                                            Disable over- and under-voltage tests.  Enable external ref test
  // x59         GEN_ERR_REG_1        x00      Not written    MEMMAP_CRC, ROM_CRC, SPI error register
  // x5A         GEN_ERR_REG_1_EN     x3E          x30        MEMMAP_CRC and ROM_CRC enabled, SPI error checks disabled
  // x5B         GEN_ERR_REG_2        x00      Not written    RESET_DETECTED, EXT_MCLK_SWITCH_ERR, PSM_ERRs
  // x5C         GEN_ERR_REG_2_EN     x3C          x1C        RESET_DETECTED disabled, voltage regulator error checks enabled
  // x5D - x5F   STATUS_REG_x         x00      Not written    Location of error bit - not used
  // x60 - x61   SRC_N_MSB, _LSB     x0000        x0068       104 for 60Hz (4.000MHz/(8*80*60) = 104.166
  //                                              x007D       125 for 50Hz (4.000MHz/(8*80*50) = 125.000
  // x62 - x63   SRC_IF_MSB, _LSB    x0000        x2AAB       0.1667 for 60Hz (x2AAB = 10923  10923/65536 = 0.1667)
  //                                              x0000       0 for 50Hz
  // x64         SRC_UPDATE           x00         x01         Use software to load an SRC update.  Set b0 to load new SRC value

  // AFE SPI protocol:
  //   b15 = R/W: 0 = Write, 1 = Read
  //   b14..8 = Address A6..A0
  //   b7..0 = Data D7..0
  
  // General Configuration 1 Register
  // b7 = 0: Keep at reset values
  // b6 = 0: Low Power Mode
  // b5 = 0: VCM Buffer is OFF
  // b4 = 0: Reference Output Buffer is OFF
  // b3 = 0: SAR is OFF
  // b2 = 0: Internal oscillator is OFF (only used by the internal logic to initialize regs after power up)
  // b1..0 = 00: No effect
  temp = AFE_SPI_Xfer(0x1100);
  
  // General Configuration 2 Register
  // b7..0: Keep at reset values
  temp = AFE_SPI_Xfer(0x1209);
  
  // Data Output Format Register
  // b7..6 = 0: 4 DOUT lines (not used)
  // b5 = 0: Put status bits in header (not CRC)
  // b4 = 0: Reserved = 0
  // b3..1 = 0: Divide DCLK by 1
  // b0 = 0: Reserved = 0
  temp = AFE_SPI_Xfer(0x1400);
  
  // For Output Data Rate = 4800 samples per second (80 samples per cycle x 60 cycles per second),
  //   4.000MHz external clock:
  // 4.000MHz/(8*4800) = 104.1667                   Low Power Mode
  // SCR_N = 104 = x0068                            Low Power Mode
  // SCR_IF = .1667 x 65536 = 10923 = 0x2AAB        Low Power Mode

  // For Output Data Rate = 4000 samples per second (80 samples per cycle x 50 cycles per second),
  //   4.000MHz external clock:
  // 4.000MHz/(8*4000) = 125.0000      Low Power Mode
  // SCR_N = 125 = 0x007D              Low Power Mode
  // SCR_IF = 0 x 65536 = 0 = 0x0000   Low Power Mode

  // *** DAH NEED TO ADD CODE FOR 50HZ OPERATION
  // b7..4 = 0: Not used
  // b3..0 = 0: Decimation register (N) b11..8 = 0
  temp = AFE_SPI_Xfer(0x6000);        // Low Power Mode

  // b7..0 = 213: Decimation register (N) b7..0 = 104 (x68)
  temp = AFE_SPI_Xfer(0x6168);          // Low Power Mode

  // b7..0 = 0xAA: Decimation register (fraction) high byte = 0x2A
  temp = AFE_SPI_Xfer(0x622A);        // Low Power Mode

  // b7..0 = 0xAB: Decimation register (fraction) low byte = 0xAB
  temp = AFE_SPI_Xfer(0x63AB);        // Low Power Mode

  // b7 = 0: Use software to update the Decimation registers
  // b6..1 = 0: Unused
  // b0 = 1: Load the new decimation rate
  temp = AFE_SPI_Xfer(0x6401);
  temp = AFE_SPI_Xfer(0x6400);                  // Lower the bit again (not sure if necessary)

  // b7 = 0: Reserved=0
  // b6 = 0: 1/4 buffer bpower (default)
  // b5 = 0: AFE buffer disabled
  // b4..3 = 0: Reference buffers disabled (bypass mode)
  // b2..0 = 0: Don't care.  If b4..3 = 0, the AFE is automatically in bypass mode
  temp = AFE_SPI_Xfer(0x1900);

  // b7..6 = 0: Don't care.  If b4..3 of x19 = 0, the AFE is automatically in bypass mode
  // b5..3 = 0: Reserved=0
  // b2..0 = 0: Keep at reset values
  temp = AFE_SPI_Xfer(0x1A00);

  // b7 = 0: Output saturated test disabled
  // b6 = 0: Filter saturated test disabled
  // b5 = 0: Modulator saturated test disabled
  // b4 = 0: AINM undervoltage test disabled
  // b3 = 0: AINM overvoltage test disabled
  // b2 = 0: AINP undervoltage test disabled
  // b1 = 0: AINP overvoltage test disabled
  // b0 = 1: Reference detection test enabled
  temp = AFE_SPI_Xfer(0x5801);

  // b7..6 = 0: Reserved=0
  // b5 = 1: Memory map CRC error enabled
  // b4 = 1: Fuse CRC test enabled
  // b3..0 = 0: SPI errors disabled
  temp = AFE_SPI_Xfer(0x5A30);

  // b7..6 = 0: Reserved=0
  // b5 = 0: Rest detected disabled
  // b4 = 1: Clock error detection enabled
  // b3..2 = 3: LDO error detection enabled for all internal LDOs
  // b1..0 = 0: No trip detect test enabled
  temp = AFE_SPI_Xfer(0x5C1C);
  
  // Sync Offset Registers
  if (Setpoints0.stp.Freq == 50)
  {
    for (i=0; i<3; ++i)
    {
      addr = 0x0900 + (i << 8) + AFEcal.phase[i+6];
      temp = AFE_SPI_Xfer(addr);
      addr = 0x0905 + (i << 8) + AFEcal.phase[i+9];
      temp = AFE_SPI_Xfer(addr);
    }
  }
  else
  {
    for (i=0; i<3; ++i)
    {
      addr = 0x0900 + (i << 8) + AFEcal.phase[i];
      temp = AFE_SPI_Xfer(addr);
      addr = 0x0E00 + (i << 8) + AFEcal.phase[i+3];
      temp = AFE_SPI_Xfer(addr);
    }
  }

  // *** DAH TEST Code to read back the AFE registers
  for (i=0; i<101; ++i)
  {
  addr = 0x8000 + (i << 8);
  fred1[i] = AFE_SPI_Xfer(addr);
  }
  // End Test Code
  
  // General Configuration 3 Register
  // b7..5: Keep at reset values
  // b4 = 1: Enable SPI slave mode to read ADC data over the SPI
  // b3..0: Keep at reset values
  temp = AFE_SPI_Xfer(0x1390);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        AFE_Init()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Update_AFE_Sync_Regs()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           AFE Sync registers update for phase cal adjustment
// 
//  MECHANICS:          Program sync offset regs in AFE  with latest phase input.
//                      Updates take effect when AFE chip SYNC OUT line is toggled.
//                          
//  CAVEATS:            None
// 
//  INPUTS:             AFEcal.phase[0..7]
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              AFE_SPI_Xfer()
//
//  EXECUTION TIME:     
// 
//------------------------------------------------------------------------------------------------------------
void Update_AFE_Sync_Regs(void)
{
  volatile uint16_t temp;
  uint16_t addr, i;
  
  
  // Sync Offset Registers
  if (Setpoints0.stp.Freq == 50)
  {
    for (i=0; i<3; ++i)
    {
      addr = 0x0900 + (i << 8) + AFEcal.phase[i+6];
      temp = AFE_SPI_Xfer(addr);
      addr = 0x0905 + (i << 8) + AFEcal.phase[i+9];
      temp = AFE_SPI_Xfer(addr);
    }
  }
  else
  {
    for (i=0; i<3; ++i)
    {
      addr = 0x0900 + (i << 8) + AFEcal.phase[i];
      temp = AFE_SPI_Xfer(addr);
      addr = 0x0E00 + (i << 8) + AFEcal.phase[i+3];
      temp = AFE_SPI_Xfer(addr);
    }
  }
  
  
 // General Configuration 2 Register
    
  temp = AFE_SPI_Xfer(0x1209);      // b0 - SYNC OUT is High
  
  temp = AFE_SPI_Xfer(0x1208);      // b0 - SYNC OUT is Low
  for (i=0; i<100; ++i)             // need hold time of >244nS; 11uS measured for count 100
  {
  }

  temp = AFE_SPI_Xfer(0x1209);      // b0 - SYNC OUT is High
  
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Update_AFE_Sync_Regs()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_DMAController1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA Channels Initialization
// 
//  MECHANICS:          This subroutine initializes the DMA1 as follows:
//
//                                      AFE Interface via SPI3 - Streams 0 and 7
//                          The DMA is used to read the eight channels of the AFE and to transfer the 32-bit
//                          values into SRAM2 memory.  Each value is read as two 16-bit words, so a total of
//                          16 words are read.
//                          SPI3 Rx:
//                              Stream 0
//                              Channel 0
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Very high priority
//                              Peripheral Address - SPI3 DR register, fixed address (not incrementing)
//                              Memory Address - &AFE_single_capture[0], auto-incrementing
//                          SPI3 Tx
//                              Stream 7
//                              Channel 0
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Very high priority
//                              Peripheral Address - SPI3 DR register, fixed address (not incrementing)
//                              Memory Address - &SPI3_READAFE (constant), fixed address (not incrementing)
//
//                                      Display Processor Normal Comms Interface via UART2 - Streams 5 and 6
//                          The DMA is used to interface with the display processor via UART2.  It is used
//                          for both transmissions and receptions.  The character length is 8 bits.  Message
//                          lengths may vary.  Transmissions are initiated in the foreground (main loop)
//                          Disp_Tx() subroutine.  The receive FIFO is checked and emptied in the foreground
//                          Disp_Rx() subroutine (via AssembleRxPkt() ).
//                          UART2 Rx:
//                              Stream 5
//                              Channel 4
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART2 DR register, fixed address (not incrementing)
//                              Memory Address - &DPComm.RxBuf[0], auto-incrementing, circular mode
//                          UART2 Tx
//                              Stream 6
//                              Channel 4
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART2 DR register, fixed address (not incrementing)
//                              Memory Address - &DPComm.TxBuf[0], auto-incrementing
//
//                                      Display Processor 61850 Interface via UART3 - Streams 1 and 3
//                          The DMA is used to interface with the display processor via UART3.  It is used
//                          for both transmissions and receptions.  The character length is 8 bits.  Message
//                          lengths may vary slightly, but all messages are designed to be the about the
//                          same length so that the Display Processor can use a timer interrupt to mark when
//                          the DMA has completed, and the message will be finished and can be processed in
//                          the timer ISR.  This provides error detection in case the message is broken.
//                          The timer interrupt will always occur.
//                          Transmissions are requested in both the foreground (main loop) and background
//                          (interrupt), and are serviced in Disp_61850_Tx() subroutine, which is called
//                          from the sampling interrupt (every 217usec).  Messages are designed to be
//                          completed within 200usec, so we should be able to transmit every 200usec.  The
//                          receive FIFO is checked and emptied in Disp_61850_Rx(), which is also called
//                          from the sampling interrupt.  The DMA does NOT generate an interrupt.
//                          UART3 Rx:
//                              Stream 1
//                              Channel 4
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              High priority
//                              Peripheral Address - UART3 DR register, fixed address (not incrementing)
//                              Memory Address - &DPComm61850.RxBuf[0], auto-incrementing, circular mode
//                          UART3 Tx
//                              Stream 3
//                              Channel 4
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              High priority
//                              Peripheral Address - UART3 DR register, fixed address (not incrementing)
//                              Memory Address - &DPComm61850.TxBuf[0], auto-incrementing
//                          
//  CAVEATS:            This subroutine assumes there are no active DMA streams.  That is, it assumes it is
//                      called only after a reset!
//
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             DMA1_Stream0, 1, 3, 5, 6, and 7 configuration registers
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Init_DMAController1(void)
{

  __DMA1_CLK_ENABLE();

  //---------------- AFE SPI3 Rx  Stream 0 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 0: Channel 0 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 3: Priority is Very High
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 1: Memory data size is word
  // b12..11 = 1: Peripheral data size is word
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 1:      Transfer complete interrupt is enabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream0->CR &= 0xF0000000;          // Make sure DMA channel is disabled before configuring it
  DMA1_Stream0->CR |= 0x00032C10;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream0->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 16:  Transferring 16 words from the AFE to memory
  DMA1_Stream0->NDTR &= 0xFFFF0000;
  DMA1_Stream0->NDTR |= 16;

  // b31..0:      Peripheral address is initialized to SPI3 data register
  DMA1_Stream0->PAR = (uint32_t)(&(SPI3->DR));

  DMA1_Stream0->M0AR = (uint32_t)(&AFE_single_capture[0]);

  //-------------- END AFE SPI3 Rx Stream 0 --------------



  //---------------- AFE SPI3 Tx Stream 7 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 0: Channel 0 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 3: Priority is Very High
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 1: Memory data size is word
  // b12..11 = 1: Peripheral data size is word
  // b10 = 0:     Memory address is fixed
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 1:   Direction is memory to peripheral
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream7->CR &= 0xF0000000;          // Make sure DMA channel is disabled before configuring it
  DMA1_Stream7->CR |= 0x00032840;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream7->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 16:  Transferring 16 words from the AFE to memory
  DMA1_Stream7->NDTR &= 0xFFFF0000;
  DMA1_Stream7->NDTR |= 16;

  // b31..0:      Peripheral address is initialized to SPI3 data register
  DMA1_Stream7->PAR = (uint32_t)(&(SPI3->DR));

  DMA1_Stream7->M0AR = (uint32_t)(&SPI3_READAFE);

  //-------------- END AFE SPI3 Tx Stream 7 --------------






  //---------------- Display UART2 Rx Stream 5 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 1:      Circular mode is enabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream5->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA1_Stream5->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA1->HIFCR |= DMA_HISR_TCIF5;                // Reset the transfer complete interrupt flag
  DMA1_Stream5->CR |= 0x08000500;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream5->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Receiving DISPCOMM_RXBUFSIZE bytes
  DMA1_Stream5->NDTR &= 0xFFFF0000;
  DMA1_Stream5->NDTR |= DISPCOMM_RXBUFSIZE;

  // b31..0:      Peripheral address is initialized to UART2 data register
  DMA1_Stream5->PAR = (uint32_t)(&(USART2->DR));

  DMA1_Stream5->M0AR = (uint32_t)((uint8_t *)(&DPComm.RxBuf[0]));

  // Must clear all event flags before initiating a DMA operation
  DMA1->HIFCR |= (DMA_HIFCR_CTCIF5 + DMA_HIFCR_CHTIF5 + DMA_HIFCR_CTEIF5 + DMA_HIFCR_CDMEIF5
                        + DMA_HIFCR_CFEIF5);
  DMA1_Stream5->CR |= 0x00000001;           // Initiate the DMA to receive the data

  //-------------- END Display UART2 Rx Stream 5 --------------



  //---------------- Display UART2 Tx Stream 6 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 1:   Direction is memory to peripheral
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream6->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA1_Stream6->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA1->HIFCR |= DMA_HISR_TCIF6;                // Reset the transfer complete interrupt flag
  DMA1_Stream6->CR |= 0x08000440;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream6->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Transmitting 36 bytes
  DMA1_Stream6->NDTR &= 0xFFFF0000;
  DMA1_Stream6->NDTR |= 36;

  // b31..0:      Peripheral address is initialized to UART2 data register
  DMA1_Stream6->PAR = (uint32_t)(&(USART2->DR));

  // Just initialize as if a normal CAM - this will be initialized in DMA1_Stream0_IRQHandler() before the
  //   DMA operation is started
  DMA1_Stream6->M0AR = (uint32_t)((uint8_t *)(&DPComm.TxBuf[0]));

  //-------------- END Display UART1 Tx Stream 6 --------------






  //---------------- Display UART3 Rx Stream 1 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 2: Priority is High
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 1:      Circular mode is enabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream1->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA1_Stream1->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA1->LIFCR |= DMA_LIFCR_CTCIF1;                // Reset the transfer complete interrupt flag
  DMA1_Stream1->CR |= 0x08020500;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream1->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Receiving DISPCOMM_RXBUFSIZE bytes
  DMA1_Stream1->NDTR &= 0xFFFF0000;
  DMA1_Stream1->NDTR |= DISPCOMM_61850RXBUFSIZE;

  // b31..0:      Peripheral address is initialized to UART3 data register
  DMA1_Stream1->PAR = (uint32_t)(&(USART3->DR));

  DMA1_Stream1->M0AR = (uint32_t)((uint8_t *)(&DPComm61850.RxVars.RxBuf[0]));

  // Must clear all event flags before initiating a DMA operation
  DMA1->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1 + DMA_LIFCR_CTEIF1 + DMA_LIFCR_CDMEIF1
                        + DMA_LIFCR_CFEIF1);
  DMA1_Stream1->CR |= 0x00000001;           // Initiate the DMA to receive the data

  //-------------- END Display UART3 Rx Stream 1 --------------



  //---------------- Display UART3 Tx Stream 3 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 2: Priority is High
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 1:   Direction is memory to peripheral
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA1_Stream3->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA1_Stream3->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA1->LIFCR |= DMA_LIFCR_CTCIF3;                // Reset the transfer complete interrupt flag
  DMA1_Stream3->CR |= 0x08020440;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA1_Stream3->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Transmitting 36 bytes
  DMA1_Stream3->NDTR &= 0xFFFF0000;
  DMA1_Stream3->NDTR |= 36;

  // b31..0:      Peripheral address is initialized to UART2 data register
  DMA1_Stream3->PAR = (uint32_t)(&(USART3->DR));

  // Just initialize as if a normal CAM - this will be initialized in DMA1_Stream0_IRQHandler() before the
  //   DMA operation is started
  DMA1_Stream3->M0AR = (uint32_t)((uint8_t *)(&DPComm61850.TxVars.TxBuf[0]));

  //-------------- END Display UART1 Tx Stream 3 --------------

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_DMAController1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_DMAController2()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA Channels Initialization
//
//  MECHANICS:          This subroutine initializes the DMA2 as follows:
//
//                                      ADC1 and ADC2 - Stream 0
//                          The DMA is used to read the currents and voltages from ADC1 and ADC2 and to transfer
//                          the 32-bit values into SRAM2 memory.  Each value is read as a 32-bit integer.  A total
//                          of 6 32-bit values are read from each ADC.
//                          ADC1:
//                              Stream 0
//                              Channel 0
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Very high priority
//                              Peripheral Address - ADC CDR register, fixed address (not incrementing)
//                              Memory Address - &ADC_single_capture[0], auto-incrementing
//                          ADC2:
//                              is part of the dual data streaming
//
//                                      CAM1 Interface via UART1 - Streams 5 and 7
//                          The DMA is used to interface with CAM1 via UART1.  It is used for both 61850
//                          communications and standard communications (including GOOSE messaging).
//                          For sample-value communications, samples are transmitted as byte (8-bit) values,
//                          so a total of 36 bytes are transmitted.  Transmissions are initiated in the AFE
//                          sampling interrupt subroutine.
//                          For standard communications, values are transmitted as byte (8-bit) values.  The
//                          length is variable.  Transmissions are initiated in the foreground (main loop)
//                          CAM_Tx() subroutine.
//                          Reception is the same for both modes of communications.  Bytes are received into
//                          a circular buffer, and the buffer is checked and emptied in the CAM_Rx()
//                          subroutine.
//                          For now, the default configuration is sample-value communications.
//                          UART1 Rx:
//                              Stream 5
//                              Channel 4
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM1.RxBuf[0], auto-incrementing, circular mode
//                          UART1 Tx
//                              Stream 7
//                              Channel 4
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &BB_SamplePkt.Ia, auto-incrementing
//
//                                      CAM2 Interface via UART6 - Streams 1 and 6
//                          The DMA is used to interface with CAM2 via UART6.  It is used for both 61850
//                          communications and standard communications (including GOOSE messaging).
//                          For sample-value communications, samples are transmitted as byte (8-bit) values,
//                          so a total of 36 bytes are transmitted.  Transmissions are initiated in the AFE
//                          sampling interrupt subroutine.
//                          For standard communications, values are transmitted as byte (8-bit) values.  The
//                          length is variable.  Transmissions are initiated in the foreground (main loop)
//                          CAM_Tx() subroutine.
//                          Reception is the same for both modes of communications.  Bytes are received into
//                          a circular buffer, and the buffer is checked and emptied in the CAM_Rx()
//                          subroutine.
//                          For now, the default configuration is standard communications.
//                          UART6 Rx:
//                              Stream 1
//                              Channel 5
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART6 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM2.RxBuf[0], auto-incrementing, circular mode
//                          UART6 Tx
//                              Stream 6
//                              Channel 5
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM2.TxBuf[0], auto-incrementing
//
//  CAVEATS:            This subroutine assumes there are no active DMA streams.  That is, it assumes it is
//                      called only after a reset!
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             DMA2_Streams 0-3, 5-7 CR, FCR, NDTR, PAR, and MOAR registers.
//
//  CALLS:              Init_CAM1_DMA_Streams(), Init_CAM2_DMA_Streams()
//
//------------------------------------------------------------------------------------------------------------

void Init_DMAController2(void)
{

  __DMA2_CLK_ENABLE();

  //---------------- ADC1 Stream 0 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 0: Channel 0 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 3: Priority is Very High
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 2: Memory data size is double-word (32-bit)
  // b12..11 = 2: Peripheral data size is double-word (32-bit)
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 1:      Circular mode is enabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 1:      Transfer complete interrupt is enabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA2_Stream0->CR &= 0xF0000000;          // Make sure DMA channel is disabled before configuring it
  DMA2_Stream0->CR |= 0x00035510;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA2_Stream0->FCR &= 0xFFFFFF40;

  // b31..16:     Reserved must be kept at reset value
  // b15..0 = 5:  Transferring 5 32-bit values from ADC1 & ADC2 to memory
  DMA2_Stream0->NDTR &= 0xFFFF0000;
  DMA2_Stream0->NDTR |= 5;            // for 5 dual channels

  // b31..0:      Peripheral address is initialized to ADC common regular data register for dual mode
  DMA2_Stream0->PAR = (uint32_t)(&(ADC->CDR));

  DMA2_Stream0->M0AR = (uint32_t)(&ADC_single_capture[0]);

  // Must clear all event flags before initiating a DMA operation
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF0 + DMA_LIFCR_CHTIF0 + DMA_LIFCR_CTEIF0 + DMA_LIFCR_CDMEIF0
                        + DMA_LIFCR_CFEIF0);
  // Start the DMA data stream.  Stream 0   Received words are stored in ADC_single_capture[0..9].
  DMA2_Stream0->CR |= 0x00000001;

  //-------------- END ADC1 Stream 0 --------------



  //------ CAM1 UART1 Rx Stream 5, Tx Stream 7 -------

  Init_CAM1_DMA_Streams();

  //----- END CAM1 UART1 Rx Stream 5, Tx Stream 7 ----



  //------ CAM2 UART6 Rx Stream 1, Tx Stream 6 -------

  Init_CAM2_DMA_Streams();

  //----- END CAM2 UART6 Rx Stream 1, Tx Stream 6 ----

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_DMAController2()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_CAM1_DMA_Streams()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA CAM1 Streams Initialization
//
//  MECHANICS:          This subroutine initializes the DMA2 CAM1 streams as follows:
//
//                                      CAM1 Interface via UART1 - Streams 5 and 7
//                          The DMA is used to interface with CAM1 via UART1.  It is used for both 61850
//                          communications and standard communications (including GOOSE messaging).
//                          For sample-value communications, samples are transmitted as byte (8-bit) values,
//                          so a total of 36 bytes are transmitted.  Transmissions are initiated in the AFE
//                          sampling interrupt subroutine.
//                          For standard communications, values are transmitted as byte (8-bit) values.  The
//                          length is variable.  Transmissions are initiated in the foreground (main loop)
//                          CAM_Tx() subroutine.
//                          Reception is the same for both modes of communications.  Bytes are received into
//                          a circular buffer, and the buffer is checked and emptied in the CAM_Rx()
//                          subroutine.
//                          For now, the default configuration is sample-value communications.
//                          UART1 Rx:
//                              Stream 5
//                              Channel 4
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM1.RxBuf[0], auto-incrementing, circular mode
//                          UART1 Tx
//                              Stream 7
//                              Channel 4
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &BB_SamplePkt.Ia, auto-incrementing
//
//                          Since this subroutine may be called after the DMA streams have been operating,
//                          it must ensure the streams are disabled before reconfiguring them.
//
//                          Reference Section 10.3.17 of the Programmer's Reference Manual
//                          (RM0090 - DocID018909 Rev 7)
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             DMA2_Streams 5 and 7 CR, FCR, NDTR, PAR, and MOAR registers.
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_CAM1_DMA_Streams(void)
{

  __DMA2_CLK_ENABLE();

  //---------------- CAM1 UART1 Rx Stream 5 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 1:      Circular mode is enabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA2_Stream5->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA2_Stream5->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA2->HIFCR |= DMA_HISR_TCIF5;                // Reset the transfer complete interrupt flag
  DMA2_Stream5->CR |= 0x08000500;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA2_Stream5->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Receiving CAMPORT_RXBUFSIZE bytes
  DMA2_Stream5->NDTR &= 0xFFFF0000;
  DMA2_Stream5->NDTR |= CAMPORT_RXBUFSIZE;

  // b31..0:      Peripheral address is initialized to UART1 data register
  DMA2_Stream5->PAR = (uint32_t)(&(USART1->DR));

  DMA2_Stream5->M0AR = (uint32_t)((uint8_t *)(&CAM1.RxBuf[0]));

  // Must clear all event flags before initiating a DMA operation
  DMA2->HIFCR |= (DMA_HIFCR_CTCIF5 + DMA_HIFCR_CHTIF5 + DMA_HIFCR_CTEIF5 + DMA_HIFCR_CDMEIF5
                        + DMA_HIFCR_CFEIF5);
  DMA2_Stream5->CR |= 0x00000001;           // Initiate the DMA to receive the data

  //-------------- END CAM1 UART1 Rx Stream 5 --------------



  //---------------- CAM1 UART1 Tx Stream 7 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 4: Channel 4 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 1:   Direction is memory to peripheral
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA2_Stream7->CR &= 0xF0000000;          // Make sure DMA channel is disabled before configuring it
  while (DMA2_Stream7->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA2->HIFCR |= DMA_HISR_TCIF7;                // Reset the transfer complete interrupt flag
  DMA2_Stream7->CR |= 0x08000440;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA2_Stream7->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Transmitting 36 bytes
  DMA2_Stream7->NDTR &= 0xFFFF0000;
  DMA2_Stream7->NDTR |= 36;

  // b31..0:      Peripheral address is initialized to UART1 data register
  DMA2_Stream7->PAR = (uint32_t)(&(USART1->DR));

  // Just initialize as if a normal CAM - this will be initialized in DMA1_Stream0_IRQHandler() before the
  //   DMA operation is started
  DMA2_Stream7->M0AR = (uint32_t)((uint8_t *)(&CAM1.TxBuf[0]));

  //-------------- END CAM1 UART1 Tx Stream 7 --------------

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_CAM1_DMA_Streams()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Init_CAM2_DMA_Streams()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA CAM2 Streams Initialization
//
//  MECHANICS:          This subroutine initializes the DMA2 CAM2 streams as follows:
//
//                                      CAM2 Interface via UART6 - Streams 1 and 6
//                          The DMA is used to interface with CAM2 via UART6.  It is used for both 61850
//                          communications and standard communications (including GOOSE messaging).
//                          For sample-value communications, samples are transmitted as byte (8-bit) values,
//                          so a total of 36 bytes are transmitted.  Transmissions are initiated in the AFE
//                          sampling interrupt subroutine.
//                          For standard communications, values are transmitted as byte (8-bit) values.  The
//                          length is variable.  Transmissions are initiated in the foreground (main loop)
//                          CAM_Tx() subroutine.
//                          Reception is the same for both modes of communications.  Bytes are received into
//                          a circular buffer, and the buffer is checked and emptied in the CAM_Rx()
//                          subroutine.
//                          For now, the default configuration is standard communications.
//                          UART6 Rx:
//                              Stream 1
//                              Channel 5
//                              Peripheral-to-memory, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART6 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM2.RxBuf[0], auto-incrementing, circular mode
//                          UART6 Tx
//                              Stream 6
//                              Channel 5
//                              Memory-to-peripheral, DMA is the flow controller
//                              Direct mode (FIFO not used)
//                              Low priority
//                              Peripheral Address - UART1 DR register, fixed address (not incrementing)
//                              Memory Address - &CAM2.TxBuf[0], auto-incrementing
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             DMA2_Streams 1 and 6 CR, FCR, NDTR, PAR, and MOAR registers.
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Init_CAM2_DMA_Streams(void)
{

  __DMA2_CLK_ENABLE();

  //---------------- CAM2 UART6 Rx Stream 1 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 5: Channel 5 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 1:      Circular mode is enabled
  // b7..6 = 0:   Direction is peripheral to memory
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA2_Stream1->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA2_Stream1->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA2->LIFCR |= DMA_LISR_TCIF1;                // Reset the transfer complete interrupt flag
  DMA2_Stream1->CR |= 0x0A000500;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA2_Stream1->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Receiving CAMPORT_RXBUFSIZE bytes
  DMA2_Stream1->NDTR &= 0xFFFF0000;
  DMA2_Stream1->NDTR |= CAMPORT_RXBUFSIZE;

  // b31..0:      Peripheral address is initialized to UART1 data register
  DMA2_Stream1->PAR = (uint32_t)(&(USART6->DR));

  DMA2_Stream1->M0AR = (uint32_t)((uint8_t *)(&CAM2.RxBuf[0]));

  // Must clear all event flags before initiating a DMA operation
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF1 + DMA_LIFCR_CHTIF1 + DMA_LIFCR_CTEIF1 + DMA_LIFCR_CDMEIF1
                        + DMA_LIFCR_CFEIF1);
  DMA2_Stream1->CR |= 0x00000001;           // Initiate the DMA to receive the data

  //-------------- END CAM2 UART6 Rx Stream 1 --------------



  //---------------- CAM2 UART6 Tx Stream 6 ----------------

  // b31..28:     Reserved must be kept at reset value
  // b27..25 = 5: Channel 5 selected
  // b24..23 = 0: Single transfer (direct mode) for memory
  // b22..21 = 0: Single transfer (direct mode) for peripheral
  // b20:         Reserved must be kept at reset value
  // b19 = 0:     Current target memory = 0 (not used as not using double buffer mode)
  // b18 = 0:     Double buffer mode is disabled
  // b17..16 = 0: Priority is Low
  // b15 = 0:     Peripheral increment offset size is linked to PSIZE (not used because peripheral address
  //              will not be incremented)
  // b14..13 = 0: Memory data size is byte
  // b12..11 = 0: Peripheral data size is byte
  // b10 = 1:     Memory address is incremented after each transfer according to MSIZE
  // b9 = 0:      Peripheral address is fixed
  // b8 = 0:      Circular mode is disabled
  // b7..6 = 1:   Direction is memory to peripheral
  // b5 = 0:      The DMA is the flow controller
  // b4 = 0:      Transfer complete interrupt is disabled
  // b3 = 0:      Half-transfer complete interrupt is disabled
  // b2 = 0:      Transfer error interrupt is disabled
  // b1 = 0:      Direct mode error interrupt is disabled
  // b0 = 0:      DMA stream is disabled
  DMA2_Stream6->CR &= 0xF0000000;           // Make sure DMA channel is disabled before configuring it
  while (DMA2_Stream6->CR & 0x00000001)     // Wait for stream to be disabled before configuring it
  {
  }
  DMA2->HIFCR |= DMA_HISR_TCIF6;                // Reset the transfer complete interrupt flag
  DMA2_Stream6->CR |= 0x0A000440;

  // b31..8:      Reserved must be kept at reset value
  // b7 = 0:      FIFO error interrupt is disabled
  // b6:          Reserved must be kept at reset value
  // b5..3:       FIFO status (read-only)
  // b2 = 0:      Direct mode is enabled
  // b1..0 = 0:   FIFO threshold level is 1/4 full (FIFO is not used)
  DMA2_Stream6->FCR &= 0xFFFFFF40;

  // b31..16:      Reserved must be kept at reset value
  // b15..0 = 36:  Transmitting 36 bytes
  DMA2_Stream6->NDTR &= 0xFFFF0000;
  DMA2_Stream6->NDTR |= 36;

  // b31..0:      Peripheral address is initialized to UART6 data register
  DMA2_Stream6->PAR = (uint32_t)(&(USART6->DR));

  DMA2_Stream6->M0AR = (uint32_t)((uint8_t *)(&CAM2.TxBuf[0]));

  //-------------- END CAM2 UART6 Tx Stream 7 --------------

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Init_CAM2_DMA_Streams()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Init_IntRTC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Initialize the Internal RTC
//
//  MECHANICS:          The STM32F407 contains an internal, battery-backed RTC.  This subroutine configures
//                      the RTC to operate using the external crystal (32.768KHz watch crystal) and loads a
//                      default time of 1/1/2000 into it.  This time setting matches the internal default
//                      power-up time.  It is assumed that time will eventually be written to the unit.
//                      Note, the RTC is configured to have time read directly from the RTC registers, NOT
//                      from the shadow registers.  Initial testing showed that the time was not accurate
//                      when reading the shadow registers immediately after writing the time.  I couldn't
//                      find the problem.  I did clear the RSF bit and wait for it to be set before reading
//                      the time (as prescribed by the APP Note), but I still saw the issue.
//
//                      REFERENCES: STM32F407 Reference Manual RM0090 (DocID018909) Rev 7
//                                  RTC Application Note AN3371
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             None
//                      
//  OUTPUTS:            SystemFlags
//                      
//  ALTERS:             RTC register contents
//                      RTC_InitState
//
//  CALLS:              None
//
//  EXECUTION TIME:     ?
// 
//------------------------------------------------------------------------------------------------------------

void Init_IntRTC(void)
{
  uint8_t i;

  switch (RTC_InitState)
  {
    case 0:                             // Turn on LSE clock
      PWR->CR |= 0x00000100;                // Set DBP bit to enable write access to RTC registers
      for (i=0; i<5; ++i)                   // Add slight delay loop just to be safe
      {
      }
      RCC->BDCR |= 0x00000001;              // Turn on the LSE clock
      RTC_InitState = 1;
      break;

    case 1:                             // Wait for LSE clock to be ready
      if ((RCC->BDCR & 0x00000002) == 0x00000002)   // Measured on 220810.  After a power-up, it takes about
      {                                             //   500msec for the LSE clock to come up!
        RCC->BDCR |= 0x00000100;                        // Select LSE clock for RTC clock source
        RCC->BDCR |= 0x00008000;                        // Turn on the RTC clock

        RTC->WPR = 0xCA;                                // Key to unlock registers for access
        for (i=0; i<5; ++i)                             // Add slight delay loop just to be safe
        {
        }
        RTC->WPR = 0x53;
        for (i=0; i<5; ++i)                             // Add slight delay loop just to be safe
        {
        }
        RTC->ISR |= RTC_ISR_INIT;                       // Enter initialization mode
        RTC_InitState = 2;
      }
      break;

    case 2:                             // Wait for initialization mode to be entered
      if ((RTC->ISR & RTC_ISR_INITF) == RTC_ISR_INITF)
      {
        // Note: the RSF bit is cleared when initialization is entered

        // CR Register:
        //   b31..24: Reserved
        //   b23 = 0: Calibration output is disabled
        //   b22..21 = 0: RTC alarm output is disabled
        //   b20 = 0: Alarm output polarity (don't care since disabled)
        //   b19 = 0: Calibration output is 512Hz (not used)
        //   b18 = 0: Backup, used to save whether daylight's savings change was made (not used)
        //   b17 = 0: Do not subtract one hour from the hours (for winter time)
        //   b16 = 0: Do not add one hour to the hours (for summer time)
        //   b15 = 0: Timestamp interupt is disabled
        //   b14 = 0: Wakeup timer interupt is disabled
        //   b13 = 0: Alarm B interupt is disabled
        //   b12 = 0: Alarm A interupt is disabled
        //   b11 = 0: Timestamp is disabled
        //   b10 = 0: Wakeup timer is disabled
        //   b9 = 0: Alarm B is disabled
        //   b8 = 0: Alarm A is disabled
        //   b7 = 0: Coarse calibration is disabled
        //   b6 = 0: 24-hour format
        //   b5 = 1: Time and date values are taken directly from the time registers
        //   b4 = 0: Reference clock detection is disabled
        //   b3 = 0: Timestamp rising edge generates and event (not used)            
        //   b2..0 = 000: Wakeup clock = RTC clock/16 (not used)
        RTC->CR = 0x00000020;

        // TR Register:
        //   b31..23: Reserved
        //   b22 = 0: 24-hour format
        //   b21..20: Hours tens in BCD format (0 - 2)
        //   b19..16: Hours ones in BCD format (0 - 9)
        //   b15: Reserved
        //   b14..12: Minutes tens in BCD format (0 - 5)
        //   b11..8: Minutes ones in BCD format (0 - 9)
        //   b7: Reserved
        //   b6..4: Seconds tens in BCD format (0 - 5)                                                                          
        //   b3..0: Seconds ones in BCD format (0 - 9)                                                                          
        RTC->TR = 0x00000000;                   // Set time to midnight in the shadow register

        // DR Register:
        //   b31..24: Reserved
        //   b23..20: Year tens in BCD format (0 - 9)
        //   b19..16: Year ones in BCD format (0 - 9)
        //   b15..13: Weekday (0 = N/A, 1 = Monday, ..., 7 = Sunday)
        //   b12: Month tens (0 - 1)
        //   b11..8: Month ones in BCD format (0 - 9)
        //   b7..6: Reserved
        //   b5..4: Date tens in BCD format (0 - 3)
        //   b3..0: Date ones in BCD format (0 - 9)
        RTC->DR = 0x0000C101;                   // Set date to 01/01/2000 (Saturday)

        RTC->ISR &= (uint32_t)(~RTC_ISR_INIT);  // Exit initialization mode to load values in

        RTC_InitState = 3;
      }
      break;

    case 3:                             // Wait for initialization mode to be exited
    if ((RTC->ISR & RTC_ISR_INITF) == 0)
    {
      // Write special codes into two of the backup registers as a marker to show the RTC was configured
      RTC->BKP0R = RTC_BKUP_CODE1;
      RTC->BKP1R = RTC_BKUP_CODE2;
      RTC->WPR = 0xFF;                      // Lock them up again
      PWR->CR &= 0xFFFFFEFF;                // Clear DBP bit to safeguard registers

      SystemFlags &= (~RTC_ERR);            // Clear RTC error flag

      RTC_InitState = 0;
    }
    break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Init_IntRTC()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    InitFlashChip()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Initialize Flash Memory Chip
//                      
//  MECHANICS:          This subroutine sets up the Flash memory chip so that it is able to correctly store
//                      trip log waveforms, alarm log waveforms, and demand data.  The following actions are
//                      taken:
//                        Set up write protection for only the calibration constants section.  Other blocks
//                        are unprotected.
//                          - Issue Write Enable (WREN) instruction
//                               - Allows a write command to have affect
//                          - Issue Write Block Protection Register (WBPR) instruction
//                               - Write protects the calibration data sector only
//                               - Write disable is automatically activated when this instruction is
//                                    finished
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             None
//                      
//  OUTPUTS:            Flash memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI1()
// 
//  EXECUTION TIME:     Measured on 160625 (rev 0.25 code): 71usec
//
//------------------------------------------------------------------------------------------------------------

void InitFlashChip(void)
{
  uint8_t i;
  volatile uint8_t temp;

  Init_SPI1(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

  // Issue write enable command
  FLASH_CSN_ACTIVE;
  SPI1->DR = FLASH_WREN;
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;
  FLASH_CSN_INACTIVE;
  i = 1;
  while (i > 0)                         //   Measured on 151110: 96nsec
  {
    --i;
  }

  // Issue the write block protection command and the data (18 bytes)
  FLASH_CSN_ACTIVE;
  SPI1->DR = FLASH_WBPR;                // Send out write protection block command
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;

  for (i=0; i<18; ++i)                  // Send out write protection block data
  {                                     //  Data is bit 143 first .. bit 0 last
    if (i == BLK_PROT_BYTENUM)          //  All data is 0 except block 0 (sector 1) is write protected
    {                                   //    Block 0 protection is bits 129 (read) and 128 (write)
      SPI1->DR = BLK_PROT_VAL;          //    This is bits 1..0 of byte 16
    }
    else
    {
      SPI1->DR = 0x00;
    }
    while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI1->DR;
  }
  FLASH_CSN_INACTIVE;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      InitFlashChip()
//------------------------------------------------------------------------------------------------------------

