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
//  FIRMWARE DRAWING:   ????????    This drawing combines the unprogrammed STM32F407 with the code's
//                                  flash programming file to produce an "assembly group" that is the
//                                  programmed device.
//
//  PROCESSOR:          ST Micro STM32F407
//
//  COMPILER:           IAR C/C++ Compiler for ARM - v8.40.1.21539
//                      IAR Embedded Workbench from IAR Systems
//
//  MODULE NAME:        Main.c
//
//  MECHANICS:          Program module containing the primary foreground execution routines
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150316  DAH - File Creation
//   0.01   150818  DAH - Code development
//   0.02   150919  DAH - Code development
//   0.03   150918  DAH - Code development
//   0.04   150928  DAH - Added calls to Intr_VarInit(), Prot_VarInit(), Meter_VarInit(), Calc_Current(),
//                        and Calc_Current1()
//   0.05   151021  DAH - Revised instantaneous protection to use half-cycle sums of squares rather than RMS
//                        currents
//                          - Intr.c, Prot.c, Test.c, Intr_ext.h revised
//                      - Revised the sampling interrupt to capture one-cycle sums of squares (needed for
//                        short-delay protection)
//                          - Intr.c, Intr_ext.h revised
//                      - Added short delay protection
//                          - Intr.c, Prot.c, Prot_ext.h revised
//                      - Corrected some bugs with the port pins
//                          - Init.c and Init_def.h revised
//                      - Added test port command to turn off AFE communications.  This was added for power
//                        consumption testing and will eventually be deleted
//                          - Intr.c, Test.c, Test_ext.h revised
//                      - Added test port command to read and modify the offset and gain calibration
//                        constants for the AFE samples
//                          - Test.c and Test_def.h revised
//                          - Iod.c, Iod_def.h, Iod_ext.h modified to add FRAM_Read() and FRAM_Write()
//                          - Init.c modified to fix bug in Init_SPI2() to get it to work with the FRAM
//                      - Added test port commands to turn the ART accelerator on and off.  These were added
//                        for power consumption testing and will eventually be deleted
//                          - Test.c revised
//                      - Added test port command to turn off the integration of the AFE current samples
//                          - Test.c and Test_ext.h revised
//                      - Replaced constant AFE gain and offset cal constants with RAM constants that are
//                        loaded from FRAM and entered via a test port command
//                          - Meter.c revised
//                      - Revised AFE_Process_Samples() to skip integrating the current samples if flag is
//                        set via a test port command
//                          - Meter.c and Meter_ext.h revised
//                      - Changed AFE offset and gain calibration of samples to multiply by the gain
//                        constant instead of divide by the gain constant
//                          - Meter.c revised
//                      - Commented out some test code
//   0.06   151110  DAH - Deleted Flexible Static Memory Controller initialization, as the external RAM will
//                        not be used
//                          - Deleted call to Init_FSMC() in the initialization portion of main()
//                          - Init.c and Init_ext.h revised
//                      - Deleted Init_SPI2() from initialization code.  It is not required, because SPI2 is
//                        initialized for the specific device each time it is used
//                      - Added support for DS1390 real time clock.  This replaces the DS1305 RTC
//                          - Init.c, Init_def.h, Init_ext.h, Iod.c, Iod_def.h, Iod_ext.h revised
//                      - Added Test port command to read and modify real time
//                          - Test.c revised
//                      - Added support for MicroChip SST26VF064B Flash chip
//                          - Iod.c, Iod_def.h, Iod_ext.h revised
//                      - Added code to store calibration constants (AFEcal.x) in both FRAM and Flash
//                          - Iod.c, Meter.c, Meter_def.h, Test.c revised
//                      - Added SPI2_Manager() to manage and service various SPI2 requests (presently
//                        support SPI2 accesses to RTC, FRAM, and Flash)
//                          - Call to SPI2_Manager() added to main loop
//                          - Iod.c, Iod_def.h, Iod_ext.h, Meter.c, Meter_ext.h revised
//                      - Added Test port commands to initiate a user waveform capture and to display a user
//                        waveform
//                          - Intr.c, Intr_def.h, Intr_ext.h, Test.c revised
//                      - Corrected code to generate the start pulse for the AFE
//                          - Init_def.h, main.c revised
//                      - Added Test port command to reset the AFE
//                          - Test.c revised
//                   BP - Added a call to Init_DMAController2()
//                      - Added a call to Calc_Temperature()
//                      - Set up Analog Mux to select Temp Sensor
//   0.07   151216  DAH - Added support for the Frame FRAM
//                          - Iod.c, Iod_def.h, Iod_ext.h, Meter.c modified
//                      - Added Test Port command to modify Frame EEPOT settings and corrected the SPI clock
//                        speed
//                          - Test.c, Iod.c, Iod_def.h, Iod_ext.h, Init.c revised
//                      - Added test code to read and verify Frame FRAM values
//                          - Meter.c revised
//                      - Added I2C communications (for communications with the override micro)
//                          - Intr.c, Init.c, startip_stm32f407xx.s revised
//                      - Revised the handling of the AFE reset line.  Don't need to activate it.  It is
//                        already active when the port is initialized in Init_GPIO()
//                      - Added test code to turn on the display
//                      - Renamed .sram1 to .sram2 so that the actual memory allocation in the data sheets
//                        matches the names
//                          - Intr.c, Meter.c, Prot.c, Intr_ext.h, Meter_ext.h
//   0.08   160122  DAH - Added test code and driver for the Modbus port (USART6).  This is just a loopback
//                        test of the RS485 transceiver and a basic UART interface test.
//                          - startup_stm32f407xx.s, Init.c, Init_def.h, Intr.c, Iod_def.h, Modbus.c,
//                            Modbus_def.h, Modbus_ext.h, Test.c revised
//                          - Added MB_VarInit() to the initialization code and MB_Top() to the main loop
//                      - Revised the AFE initialization for an 8.192MHz crystal.  The previous
//                        initialization was for an 8.000MHz crystal
//                          - Init.c revised
//                      - Revised the AFE initialization to enable and disable the appropriate on-chip
//                        diagnostic tests.  Only those tests that indicate a "broken" chip are enabled.
//                          - Init.c revised
//                      - Corrected user waveform capture bug.  Waveform capture request flag was not
//                        cleared when the capture was completed, causing a second capture to occur once the
//                        acknowledge flag was cleared (immediately after a waveform was read).  The request
//                        flag is now cleared once the capture is completed in the sampling interrupt.
//                          - Intr.c revised
//                      - Corrected error in Calc_Current1()
//                          - Meter.c revised
//                      - Changed the number format for the 1/2-cycle and 1-cycle sample buffers from
//                        floating point to unsigned integer.
//                        The basic algorithm - keep a running total of the sums of squares, subtract the
//                        earliest sample, and add the latest sample - has not changed.  This method has the
//                        advantage of fast execution speed, since there are not many math operations to
//                        perform (as opposed to adding the entire buffer contents with each new sample).
//                        A problem was seen with this method, where the sums of squares could go negative.
//                        This could occur due to imprecision in the floating point operations, or due to
//                        lack of resolution (roundoff error).  For example:
//                          sample 0 = 10E7    SOS = 10E14
//                          samples 1 thru 39 = 10E2     SOS = 10E14    (FP rounds off)
//                          new sample 0 = 1E1   SOS = 10E14 - 10E14 + 1E2 = 1E2
//                          new sample 1 = 1E1   SOS = 1E2 - 1E4 + 1E2 < 0
//                        Since the 1/2-cycle and 1-cycle sample buffers re only used for instantaneous and
//                        short delay protection, the history is now stored as 64-bit integers multiplied by
//                        10 (for 0.1A resolution) instead of floating point numbers.  The sums of squares
//                        are converted back to floating point before the protection subroutines are called.
//                        Since integer values are used, there is no issue with resolution error.  Although
//                        the new method takes a little more execution time, it is safer.
//                          - Intr.c revised
//                      - Added code to set the Status LED to the error state if the AFE CHIP ERROR but is
//                        set
//                          - Meter.c revised
//   0.09   160224  DAH - Deleted code in initialization to turn off the test injection.  This is handled
//                        automatically in Init_GPIO()
//                      - Deleted code in initialization to set up the analog mux for the temp sensor.  The
//                        mux has been deleted from rev 2 boards
//                      - Revised code for rev 2 hardware
//                          - Init.c, Init_def.h, Intr.c, Iod.c, Iod_def.h, Iod_ext.h, Meter.c,
//                            Test.c revised
//                      - Added code to support the tripping function
//                          - Intr.c, Intr_ext.h, Iod_def.h revised
//                      - Added support for basic seeding of the AFE integrator.  After a reset, the AFE
//                        integrator is seeded with the ADC input.  Samples are not yet aligned, but this
//                        should give us an idea of how well the basic seeding works.
//                          - Intr.c, Intr_ext.h, Meter.c revised
//                      - For test purposes, added code to support Short Delay Flat Trip at 10000A (2.5pu in
//                        an R-Frame) and 50msec
//                          - Prot.c revised
//                      - Disabled calls to instantaneous and short delay protection so protection doesn't
//                        interfere with metering testing
//                          - Intr.c revised
//                      - Moved ADC1, ADC2, and ADC3 enabling from the initialization functions to
//                        EnDisPeripherals().  The ADCs are now enabled after the 120MHz clock is present.
//                        The ADC timing is and alignment with the AFE samples depends on the system clock.
//                        Waiting for the 120MHz clock simplifies the process and means the initialization
//                        does not need to be done twice.  If the 120MHz does not come up, we assume the
//                        board is broken and run basic protection with the override micro.
//                          - Init.c revised
//                      - Changed port C ADC current input mux control (PC8) to initialize for high-gain
//                        inputs.  The high-gain inputs will be used initially for seeding.
//                          - Init_def.h revised
//                      - Added half-cycle, one-cycle, and one-second min/max data acquisition and display
//                        over the test port
//                          - Intr.c, Intr_ext.h, and Test.c revised
//                      - Updated digital integrator coefficients per input from George Gao
//                          - Meter.c revised
//                      - Added a separate state variable, TP.SubState, for the test command subroutines to
//                        reduce the number of states in TP_Top().  The intent is to keep this subroutine as
//                        small and "clean" as possible for readability.
//                          - Test.c and Test_def.h revised
//                      - Added ability to force ADC samples to be used via the Test Port, and the ability
//                        to control whether the low-gain or high-gain ADC input is used
//                          - Test.c, Meter.c, Meter_ext.h revised
//                      - Added Test Port command to restore AFE calibration constants to the default values
//                          - Test.c, Meter.c, and Meter_def.h revised
//   0.10   160310  DAH - Finished cleaning up TP_Top()
//                          - Test.c revised
//                      - Added Test Port command to turn on short delay protection.  Also added code to
//                        initiate a 10-cycle waveform capture when the unit is in pickup.  The capture ends
//                        when the unit trips.  Since pickup is 600A, we can examine the current samples as
//                        soon as primary current is turned on.  This is used for test purposes to see the
//                        initial response of the integrator.  Also added a Test Port command to display the
//                        test waveforms.
//                          - Test.c, Prot.c, Meter.c, Meter_ext.h, Intr.c, and Intr_ext.h revised
//                      - Added DC filtering to the samples for metered values.  This filter, designed by
//                        George Gao, replaces the simple one-second averaging filter.
//                          - Meter.c, Meter_ext.h, Intr.c revised
//                      - Changed the integration time for the metered values from one second to 200msec
//                          - OneSecAnniv renamed to msec200Anniv
//                          - Intr.c, Intr_ext.h, Meter.c, Meter_ext.h revised
//                      - Added the 200msec currents to the min and max calculations and display
//                          - Meter.c, Meter_ext.h, Intr.c, and Test.c revised
//                      - Finalized the ADC cal constants.  Created separate constants for the high-gain
//                        and low-gain circuits and moved them to FRAM and Flash.  Began setting up SPI2
//                        FRAM map
//                          - FRAM_def.h added
//                          - Iod.c, Iod_def.h, Iod_ext.h, Meter.c, Meter_ext.h, Meter_def.h revised
//                      - Added Test Port calibration commands OCxx and OGxx
//                          - SystemFlags.VALONECYC added to main loop to indicate new one-cycle values
//                          - SystemFlags.VAL200msec added to main loop to indicate new 200msec values
//                          - Test.c, Test_def.h, Iod.c, Iod_def.h, Iod_ext.h revised
//                      - Modified the digital integrator per input from George Gao
//                          - Meter.c, Meter_def.h revised
//                      - Added block write protection to the calibration constants in FRAM
//                          - Iod.c revised
//   0.11   160317  DAH - Renamed MCF test port command to MCA to make it consistent with the OCx and OGx
//                        commands, and added ADC cal constants to the command
//                          - Test.c revised
//                      - Corrected error in interrupt structure initialization.
//                          - Init.c revised
//   0.12   160502  DAH - Added voltage metering and default calibration.  Metered voltages can be displayed
//                        via the test port.
//                          - Added calls to Calc_Voltage() and Calc_Voltage1()
//                          - Meter.c, Meter_ext.h, iod.c, Test.c revised
//                      - Added gain and offset calibration for the AFE voltage channels
//                          - Test.c revised
//                      - Added AFE clock generation from Timer 1
//                          - Added Init_Tim1() to initialization code in main loop
//                          - Init.c, Init_def.h revised
//                      - Corrected baud rate register (UART5->BRR) setting for 16MHz operation so the Test
//                        Port will work when running on the internal clock
//                          - Init.c revised
//                      - Added Init_Tim3() and Init_Tim9() to initialization code in the main loop (moved
//                        from SysClkHandler() )
//                      - Added a one sample delay to the integrator output to compensate for the lead
//                        introduced by the digital integrator
//                          - Meter.c revised
//                      - Added power calculations and display
//                          - Calc_Power1() and Calc_Power() added to the main loop
//                          - Intr.c, Meter.c, Meter_def.h, Meter_ext.h, and Test.c revised
//                      - Added phase calibration
//                          - Test.c, Test_def.h, Init.c, Iod.c, Meter_def.h, FRAM_def.h revised
//                      - VAL200msec renamed VAL200MSEC
//                      - Added sample synchronization between the AFE and the ADC
//                          Init.c, Init_ext.h, Intr.c, and Meter.c revised
//                      - Finalized AFE integrator seeding
//                          Intr.c, Intr_ext.h, and Meter.c revised
//                      - Added EAP9 and EAP10 commands to turn on and off Modbus chip power
//                          - Test.c revised
//                      - Corrected In sum of squares calculation (half-cycle In was off by a factor of 10)
//                          - Intr.c revised
//          160502   BP - Removed call to Calc_Temperature()from 200 msec anniversary for Rev 2 hardware.
//                      - Revised ADC sampling and conversions for the Rev 2 hardware
//                          - Meter.c revised
//   0.13   160512  DAH - Finalized AFE seeding by adding code to handle transitioning between AFE and ADC
//                        current readings
//                          - Meter.c revised
//                      - Corrected bugs in ReadADCHCalConstants() and ReadADCLCalConstants()
//                          - Iod.c revised
//                      - Added Test Port commands to restore default ADC current cal constants
//                          - Test.c revised
//                      - Added energy metering and storage in on-board FRAM
//                          - FRAM_ReadEnergy() added to initialization
//                          - Calc_Energy() added to the main loop
//                          - Meter.c, Meter_def.h, Meter_ext.h, Iod.c, Iod_def.h, Iod_ext.h, FRAM_def.h
//                            revised
//                      - Added test port commands to display energy and reset energy registers
//                          - Test.c, Test_def.h, Meter.c, Meter_ext.h revised
//                      - Fixed bug computing Phase B reactive power
//                          - Intr.c revised
//   0.14   160620  DAH - Changed code so unit is set up for an N-Frame instead of an R-Frame
//                          - Meter.c and Meter_def.h revised
//                      - Modified the way apparent power is computed.  Changed to multiplying rms current
//                        and voltage instead of taking the rms of the real and reactive power
//                          - Meter.c revised
//                      - Added min and max power computations and test port command to display min and max
//                        power
//                          - Meter.c, Meter_ext.h, Test.c revised
//                      - Re-added support for the Frame EEPOT
//                          - Iod.c, Iod_def.h. Iod_ext.h, Test.c, Init.c, Init_def.h revised
//                      - Fixed minor bugs in gain and phase calibration
//                          - Test.c revised
//                      - Added Test Injection and associated calibration
//                          - Intr.c, Test.c, Test_def.h, Test_ext.h, Iod.c, Iod_def.h, Iod_ext.h,
//                            Meter.c, Meter_def.h, Meter_ext.h, FRAM_def.h revised
//                      - Added Test Port command to set the AFE PGA gain
//                          - Test.c revised
//                      - Corrected bug displaying min/max variance for currents
//                          - Test.c revised
//                      - Fixed Pha and Phb apparent power calculations
//                          - Meter.c revised
//   0.15   160718  DAH - Added test command to test the Cause of Trip LEDs
//                          - Added call to ReadSwitches() in initialization
//                          - Iod.c, Iod_def.h, Iod_ext.h, Test.c revised
//                      - Added test command to test the Alarm LED
//                          - Test.c revised
//                      - Added reading the Clear LEDs pushbutton and the Breaker Closed switch
//                          - Iod.c, Iod_def.h, Iod_ext.h, Intr.c revised
//                      - Added storing waveform samples in the Black Box (SPI1) FRAM
//                          - Added call  to Init_SPI1() in initialization
//                          - Init.c, Init_ext.h, Intr.c, Intr_def.h, Intr_ext.h, Iod.c, Iod_def.h,
//                            Iod_ext.h, FRAM_def.h revised
//                      - Added storing the waveform samples in RAM for Trip event, Alarm event, and
//                        User-initiated captures
//                          - Intr.c revised
//                      - Change Modbus port from USART6 to USART3
//                          - Modbus.c revised
//                      - Added code for CAM port (hardware) testing, sample-value CAM communications, and
//                        normal (GOOSE) CAM communications
//                          - Added includes of CAMCom_def.h and CAMCom_ext.h
//                          - Added calls to Comm_VarInit() and Init_SPI1() in initialization
//                          - Added calls to CAM_Tx() and CAM_Rx() in the main loop
//                          - CAMCom.c, CAMCom_def.h, CAMCom_ext.h, Init.c, Init_ext.h, Intr.c revised
//                      - Added test command to change the CAM type (sampled-value or GOOSE)
//                          - Test.c revised
//                      - Added test command to initiate a (GOOSE) CAM test transmission
//                          - Test.c revised
//   0.16   160818  DAH - Added basic display communications
//                          - Added call to Disp_VarInit() in initialization
//                          - Added code assembling message and call to Disp_Tx() in the main loop
//                          - Added include of Display_ext.h
//                          - Display.c, Display_def.h, Display_ext.h, Intr.c revised
//                      - Corrected PG10 Type register definition and Port H Type register definitions
//                          - Init_def.h revised
//                      - Removed ADC3 conversions from the ADC DMA interrupt handler
//                       (DMA2_Stream0_IRQHandler()).  The conversions will be done in the individual
//                       foreground subroutines as needed
//                          - Intr.c, Iod_def.h revised
//                      - Added pickup flags
//                          - Prot.c, Prot_def.h, Prot_ext.h revised
//                      - Added Startup Time measurement
//                          - Added code in initialization to read the Startup capacitor voltage
//                          - Added 5 minute anniversary code in the main loop.  This initiates startup
//                            time calibration
//                          - Added call to StartupTimeCal() in the main loop
//                          - Iod.c, Iod_ext.h, Iod_def.h, Intr.c, Init.c, Init_def.h revised
//                      - Added thermal memory voltage ADC conversion
//                          - Added code in initialization to read the thermal memory capacitor voltage
//                          - Iod.c, Iod_ext.h, Iod_def.h, Init.c revised
//                      - Added test command to display the Startup Time and thermal memory values
//                          - Test.c revised
//                      - Added test command to display memory
//                          - Test.c, Test_def.h revised
//   0.17   161031  DAH - Removed Timer 3 and ADC complete interrupts as they are no longer used (DMA is
//                        used instead)
//                          - Init.c and Intr.c revised
//                      - Removed all Modbus code as Modbus will be moved to the Display Processor
//                          - Deleted call to MB_VarInit() from the initialization code
//                          - Deleted call to MB_Top() from the main loop
//                          - Init.c, Intr.c, Test.c revised.  Modbus.c, Modbus_def.h, Modbus_ext.h removed
//                            from the project
//                      - Revised the RTC code to switch from the Maxim DS1390 to the MCP7951 because the
//                        Maxim part was losing the time completely when the aux power connector was
//                        installed or removed
//                          - Init.c, Iod.c, and Test.c revised
//                      - Added test commands to turn off and turn on display power
//                          - Test.c revised
//   0.18   161115  DAH - Revised code for new rev 3 boards
//                          - Eliminated call to Init_UART3() and deleted Init_UART3() as the Modbus port
//                            moved to the display processor
//                              - Init.c, Init_def.h, and Init_ext.h revised
//                          - Revised PI6 (LED_SELECT1) configuration for rev 3 board
//                              - Init_def.h, Iod_def.h, and Prot.c modified
//                          - Added support for the Health Bus (via I2C3) and revised code to read the
//                            temperature/humidity sensor as it was moved from the SPI bus to the new I2C
//                            health bus
//                              - Added call to Init_I2C3() to the initialization code
//                              - Added ReadTHSensor() to the main loop
//                              - Iod.c, Iod_def.h, Iod_ext.h, Intr.c, Init.c, Init_def.h, Init_ext.h, and
//                                startup_stm32f407xx.h, revised
//                          - Revised code to support the new (corrected) thermal memory interface
//                              - In initialization code, changed MODBUS_PWR_ENABLE (which was retrofitted
//                                to the thermal memory circuit) to TH_MEM_CHARGE_EN
//                              - Init_def.h and Iod_def.h revised
//                          - Revised Modbus Power Enable signal (active low to active high)
//                              - Init_def.h and Iod_def.h revised
//                          - Added support for HiLoad LED
//                              - Test.c revised
//                          - Revised PG14 (MODBUS_DE/REN) configuration from output to input for the rev 3
//                            board.  This control signal must be driven by the display processor.  Deleted
//                            MODBUS_TXDIS_RXEN and MODBUS_TXDIS_RXEN
//                              - Init_def.h, Iod_def.h modified
//                          - Modified FRAM_Write() and FRAM_Read() to handle the 256k x 8 FRAM (3-byte
//                            address), which replaced the 32k x 8 FRAM (2=byte address)
//   0.19   170223  DAH - Revised the display communications to use DMA instead of interrupts
//                          - DMA1 Streams 5 and 6 are required, so had to move AFE SPI3 Tx from Stream 5 to
//                            Stream 7
//                          - Display.c, Display_def.h. Intr.c, Init.c, and startup_stm32f407xx.s revised
//                      - Revised display communications protocol to add CRC, message length, escape chars
//                          - Display.c, Display_def.h revised
//                      - Revised display communications message body to send 200msec and one-cycle currents
//                        instead of a test string
//                          - Deleted test code from initialization section and from the main loop
//                          - Display.c revised
//                      - Increased display communications bit rate from 9600 bps to 3.75M bps
//                          - Init.c revised
//                      - Added display receptions.  Presently supports just simple commands for test
//                        injection
//                          - Call to Disp_Rx() added to the main loop
//                          - Display.c, Display_def.h, Display_ext.h, Intr.c revised
//                      - In ShortDelay_Prot(), replaced code to turn on the SD cause of trip LED with a
//                        call to WriteCauseOfTripLEDS()
//                          - Prot.c revised
//                      - Revised code to write cause of trip LEDs for the rev 3 hardware
//                          - Iod.c, Iod_def.h revised
//                      - Added "TR" command to test the auxiliary relays and "EAD" command to support
//                        display communications testing
//                          - Test.c revised
//                      - Added "EAB" command.  This command causes a command to be sent to the display
//                        processor to start or stop Modbus communications (square wave output for power
//                        consumption testing)
//                          - Test.c revised
//                      - In TP_TestInjOffsetCal(), increased starting value from 2300 to 2375
//                          - Test.c revised
//                      - Corrected bugs in TP_GainCal(), TP_TestInjGainCal(), and TP_TestInj()
//                          - Test.c revised
//                      - Revised test injection to allow control from both the test port and from display
//                        communications
//                          - Test.c, Test_def.h, Test_ext.h, Display.c revised
//   0.20   180222  DAH - Not released
//   0.21   180301  DAH - Cleaned up sampling interrupt code
//                          - Intr.c, Intr_def.h, Test.c revised
//                      - Added code to compute ADC (line-side) voltages
//                          - Calc_Voltage2() and Calc_Voltage3() added to the main loop
//                          - Intr.c, Meter.c, Meter_ext.h revised
//                      - Added harmonics computations and revised user waveform captures so that it can be
//                        either a single cycle of 11 parameters or 12 cycles of one parameter
//                          - Calc_Harmonics() added to the main loop
//                          - Meter.c, Test.c, Intr.c, Meter_ext.h, Intr_def.h revised
//   0.22   180420  DAH - Added code for internal real time
//                          - Added code to initialize the internal time from the RTC
//                          - Intr.c, Test.c, Iod.c, Iod_def.h, and Iod_ext.h revised
//                      - Revised SPI2 FRAM definitions for Rev 3 hardware
//                          - FRAM_def.h revised
//                      - Added "f" to constants definitions to ensure they are interpreted as floats
//                          - Meter_def.h revised
//                      - Added support for demand computations
//                          - Added Calc_I_Demand() to the main loop
//                          - Meter.c, Meter_ext.h revised
//                      - Added energy and demand logging
//                          - Added calls to InitFlashChip(), Dmnd_I_VarInit(), Event_VarInit(), and
//                            EventManager()
//                          - Meter.c, Events.c, Test.c, Prot.c, Iod.c, Display.c, Init.c, FRAM_def.h,
//                            Events_def.h, Events_ext.h, Meter_def.h, Meter_ext.h, Iod_def.h, Iod_ext.h,
//                            Init_ext.h revised
//                          - Flash_def.h added to the project
//   0.23   180504  DAH - Revised the Demand function so that internal time is used to determine when
//                        anniversaries occur, not the sub-interval counters, although the 200msec
//                        anniversary still kicks everything off.
//                          - Iod.c, Iod_ext.h, Demand.c revised
//                      - Improved readability
//                          - Added Harm_Table.h to the project
//                          - Added Demand.c, Demand_def.h, and Demand_ext.h to the project
//                          - Revised Meter.c, Iod.c, Iod_ext.h, Meter_def.h, Meter_ext.h
//                      - Corrected bug in offset calibration and displaying waveforms (UserSamples
//                        definition changed)
//                          - Intr.c, Intr_def.h, Intr_ext.h, Test.c revised
//                      - Added flag (FLASH_IN_USE) when the Flash is being written to. The intent is to
//                        eventually eliminate SPI2_Manager(), and just use SPI2 in the various subroutines
//                        as needed.  The only time SPI2 needs to be "locked" is when writing to Flash (or
//                        if SPI2 is used with a DMA)
//                          - Events.c, Events_def.h, Iod.c, Iod_def.h revised
//                      - Added event and demand logging for major time changes via the Test port
//                          - Test.c revised
//                      - Corrected bug when converting internal time to RTC time
//                          - Iod.c revised
//                      - Added display demand values (DD) test port command
//                          - Test.c revised
//                      - Added voltage and power demands
//                          - In intialization code, Dmnd_I_VarInit() renamed to Dmnd_VarInit()
//                          - In the main loop, renamed Calc_I_Demand() to Calc_Demand()
//                          - Demand.c, Demand_ext.h, Events.c revised
//                      - Fixed bug that caused demand event processing to freeze whenever the next sector
//                        in Flash needed to be erased
//                          - Events.c revised
//   0.24   180517  DAH - Eliminated SPI2_Manager() and added Flash usage arbitration
//                          - SPI2_Manager() deleted from the main loop
//                          - Events.c, Iod.c, Test.c, Meter.c, Iod_def.h, Iod_ext.h revised
//                      - Corrected energy read and writes to FRAM
//                          - Iod.c, FRAM_def.h revised
//                      - Combined Flash_def.h and FRAM_def.h
//                          - Deleted Flash_def.h
//                          - Replaced FRAM_def.h with FRAM_Flash_def.h
//                          - Demand.c, Events.c, Intr.c, Iod.c, Test.c revised
//                      - Moved FLASH_IN_USE from SystemFlags to FlashCtrl flags
//                          - Events.c, Iod.c, Test.c, Iod_def.h, Iod_ext.h revised
//                      - Added Flash storage for metering and test injection calibration constants
//                          - Iod.c, Init.c, Test.c, Iod_ext.h revised
//                      - Revised FRAM_ReadEnergy() and FRAM_WriteEnergy() to work with the FM25V20 FRAM
//                        (they had been designed for the FM25V05)
//                          - Iod.c revised
//                      - Began code for alarm waveform captures.  EAW3 test port command added.
//                          - Test.c, Intr.c, Iod.c, Iod_ext.h, FRAM_Flash_def.h
//   0.25   180621  DAH - Moved SPI2 management into a 1.5msec interrupt subroutine.  This is necessary
//                        because waveform captures must be written to Flash every 1.5msec (so the samples
//                        are written before the buffer overruns) - an interrupt is necessary.
//                        Other SPI2 operations are done in between the Flash page program operations.
//                          - Init_TIM4() added to initialization
//                          - ManageSPI2Flags() added to main loop
//                          - Demand.c, Init.c, Init_ext.h, Test.c, Iod.c, Iod_def.h, Iod_ext.h, Meter.c,
//                            Meter_ext.h, Events.c, Events_def.h, Events_ext.h, Intr.c, FRAM_Flash_def.h,
//                            startup_stm32f407xx.s revised
//                      - Added writing the startup time measurement scaling constant to FRAM
//                          - In the main loop, replaced the fixed value (2E-5) with the stored scaling
//                            constant (StartupTime.Scale) when calculating the startup time
//                          - Iod.c, Iod_def.h, FRAM_Flash_def.h revised
//                      - Added alarm waveform captures and display via the test port
//                          - Intr.c, Events.c, Events_def.h, Events_ext.h, Meter.c, Test.c, Test_def.h,
//                            Test_ext.h, Iod.c, Iod_def.h, FRAM_Flash_def.h revised
//   0.26   180829  DAH - Modifications to reduce the initialization time.  Before changes, initialization
//                        code took about 3.52msec.  After changes, about 1.843sec.
//                          - Replaced call to Test_VarInit() with statement initializing TP_State to
//                            TP_INIT.  Test_VarInit() is now called the first time TP_Top() is invoked.
//                          - Renamed SystemFlags GET_STARTUPTIME to FIRST_PASS, as this flag now causes
//                            multiple one-time tasks to be performed the first time through the main loop,
//                            not just reading the startup time.
//                          - Replaced the call to FRAM_ReadEnergy() in the initialization code with setting
//                            the S2NF_ENERGY_RD request flag in the main loop under the FIRST_PASS tasks
//                          - Modified ReadAFECalConstants(), ReadADCHCalConstants(), ReadADCLCalConstants()
//                            to only read the Flash if the FRAM constants have a checksum error.  Added
//                            code in the FIRST_PASS tasks to read the Flash values for diagnostics purposes
//                          - Test.c, Test_def.h, Test_ext.h, Iod.c, Iod_def.h, Iod_ext.h, Meter.c revised
//                      - Added support for Strip-Chart waveforms in Flash memory and changed Flash memory
//                        allocation
//                          - Iod.c, FRAM_Flash_def.h revised
//                      - Code cleanup
//                          - Intr_def.h revised
//   0.27   181001  DAH - Modified the User Waveform captures to support requests from the display and all
//                        of the communications ports
//                          - Intr.c, Intr_def.h, Intr_ext.h, Meter.c, Meter_ext.h, Test.c, Test_def.h
//                            revised
//                      - Added Trip waveform captures and revised the waveform capture writes to Flash to
//                        improve the performance and correct some bugs
//                          - Intr.c, Meter.c, Iod.c, Iod_def.h, Iod_ext.h, FRAM_Flash_def.h revised
//                      - Revised "DA" command to display the number of samples
//                          - Test.c revised
//                      - Added Strip-Chart waveform captures
//                          - Intr.c, Meter.c, Iod.c, Iod_def.h, Iod_ext.h, FRAM_Flash_def.h revised
//                      - Added clearing Trip, Alarm, and Strip-Chart waveforms to the "RU" command
//                          - Events.c revised
//                      - Changed EAW3 command from alarm waveform capture to strip-chart waveform capture
//                        and created EAW4 command for alarm waveform capture
//                          - Test.c revised
//                      - Added "DT" and "DC" commands to display the Trip and Stip-Chart waveforms
//                          - Test.c revised
//   0.28   181030  DAH - Modified the communications protocol for the planned new high-speed CAM protocol.
//                        This protocol is based on the Consistent Overhead Byte Stuffing (COBS) technique
//                        and is an 8-bit protocol
//                          - Display.c, Display_def.h, Display_ext.h, Intr.c revised
//                      - Modifications to reduce the startup time
//                          - The hardware reset chips driving the PDR pin (U76) and WKUP pin (U93) were
//                            changed to TPS3702.  This reduced the reset delay from 3msec max to 300usec
//                            max.  In addition, the 1.2V reset signal (WKUP) and 3.3V reset signal (PDR)
//                            are connected together to that both reset signals are released only when both
//                            the 1.2V and 3.3V supplies are valid
//                          - The 27pF crystal loading capacitors, C429 and C430 were changed to 12pF.  The
//                            crystal is spec'ed for 20pF load capacitance, and the micro data sheet states
//                            that the PCB pins and traces add about 10pF.  This reduced the startup time of
//                            the crystal from about 2.6msec to about 1.9msec
//                          - Moved the call to Dmnd_VarInit() in front of the call to Meter_VarInit() so
//                            that enough time has elapsed that the HSE is ready and we can switch to the
//                            120MHz PLL.  This means the SPI operations in Meter_VarInit() will run at
//                            15MHz instead of 8MHz.  Added code to switch to the PLL after Dmnd_VarInit()
//                          - Modified delay code for AFE reset inactive to call to AFE_Init() and for AFE
//                            start pulse to compensate for 120MHz clock instead of 16MHz clock
//   0.29   190122  DAH - Reduced the HEAP size in the stm32f407xG.icf from x800 to x200.  Testing showed
//                        the HEAP is not used at all, and no memory is allocated to it.  In addition, there
//                        is not enough room in the CCM memory for a HEAP that is x800 bytes.  Will keep the
//                        size at x200 in case it is used in the future.
//                          - stm32f407xG.icf revised
//                      - Added code for Rogowski coil temperature measurement
//                          - Added calls to Init_TIM5() and Init_TIM8()
//                          - Commented out code in initialization that sets ADC3 channel to startup cap.
//                            Also commented out call to StartupTimeCal() in the main loop so the ADC3
//                            conversion does not interfere with the coil temperature conversions - we will
//                            need to add some top-level code to manage the ADC3 conversions
//                          - Revised call to Init_ADC3() to initialize for coil temperature measurements.
//                            This has to be changed back to initializing for normal measurements after
//                            testing has been completed.
//                          - Init.c, Init_def.h, Init_ext.h, Intr.c, Intr_ext.h, Iod_def.h, Meter.c,
//                            Test.c, Test_def.h revised
//   0.30   190314  DAH - Added support for an Initiate Test Injection Execute Action command from the
//                        display, and to turn off Test Injection with the pushbutton switch
//                          - Display.c, Iod.c revised
//                      - Corrected bug in TP_TestInjOffsetCal() and TP_TestInjGainCal()
//                          - Test.c revised
//                      - Added support for coil temperature measurements
//                          - Meter.c, Meter_ext.h, Test.c revised
//   0.31   190506  DAH - Revised harmonics computations to decouple them from waveform captures and to meet
//                        IEC61000-4-7 and IEC61000-4-30
//                          - Harm_Tables.h, Intr.c, Intr_def.h, Intr_ext.h, Meter.c, Meter_ext.h, Test.c
//                            revised
//   0.32   190726  DAH - Deleted voltages from the demand calculations
//                          - Demand.c, Demand_def.h, Test.c, FRAM_Flash_def.h revised
//                      - Added code to initialize demands to NaN
//                          - Demand.c revised
//                      - Added min and max demands
//                          - Demand.c, Demand_def.h, Demand_ext.h, Iod.c, Iod_def.h, FRAM_Flash_def.h,
//                            Meter.c revised
//                      - Improved the readability of Intr.c
//                          - Intr.c revised
//                          - IntrInline_def.h added
//                      - Added Modbus RTU communications drivers.  Note, the code is designed for the Rev 4
//                        boards and is not fully operational presently
//                          - Added calls to Init_Tim2() and ModB_VarInit() to the initialization code
//                          - Added call to ModB_SlaveComm() to the main loop
//                          - Modbus.c, Modbus_def.h, and Modbus_ext.h added to the project
//                          - Display.c, Display_ext.h, Init.c, Init_ext.h, Intr.c, Meter.c, Meter_ext.h
//                            revised
//                      - Added THD and Displacement Power Factor
//                          - Added calls to Calc_AppPF() and Calc_DispPF_THD() to the main loop
//                          - Intr.c, Intr_ext.h, Meter.c, Meter_def.h, Meter_ext.h revised
//                      - Renamed and combined metering variable structures for better readability
//                          - Intr.c, Meter.c, Meter_def.h, Meter_ext.h, Test.c revised
//                      - Renamed Calc_Power() to Calc_Meter_Power() and revised the method for computing
//                        reactive power.  Renamed Calc_Power1() to Calc_Prot_Power() and revised the method
//                        for computing reactive power
//                          - Meter.c, Meter_ext.h revised
//                      - Added total real, reactive, and apparent powers
//                          - Meter.c, Meter_def.h revised
//   0.33   190823  DAH - Calc_Current renamed to Calc_Meter_Current()
//                      - Calc_Current1() renamed to Calc_Prot_Current()
//                      - Calc_Voltage() renamed to Calc_Meter_AFE_Voltage()
//                      - Calc_Voltage2() renamed to Calc_Meter_ADC_Voltage()
//                      - Calc_Voltage1() renamed to Calc_Prot_AFE_Voltage()
//                      - Added time stamps to meter currents, voltages, and powers
//                          - Meter.c, Meter_def.h, Meter_ext.h revised
//                      - Added Crest Factor
//                          - Added call to Calc_CF() to the main loop
//                          - Intr.c, IntrInline_def.h, Meter.c, Meter_ext.h revised
//                      - Added K-Factor, Sequence Components, and Phase Angles
//                          - Added calls to Calc_KFactor() and Calc_SeqComp_PhAng() to the main loop
//                          - Meter.c, Meter_ext.h, Harm_Tables.h revised
//                      - Added Voltage and Current Unbalance
//                          - Meter.c revised
//                      - Added Frequency measurements
//                          - Added calls to Calc_Freq() for both line and load to the main loop
//                          - Init.c, Intr.c, startup_stm32f407xx.s, Meter.c, Meter_def.h,
//                            Meter_ext.h revised
//   0.34   191002  DAH - Added code to initialize some variables
//                          - Meter.c revised
//                      - Revised code for rev 4 boards and switched to version 8.4 compiler
//                          - Timer 9 was replaced by Timer 10 and Timer 11 to measure line and load
//                            frequencies.  The frequency input pins changed from PE5 and PE6
//                            (rev 3 boards) to PF7 and PB8 (rev 4 boards)
//                              - In initialization, Init_TIM9() replaced with Init_TIM10() and Init_TIM11()
//                              - Init.c, Init_def.h, Init_ext.h, Intr.c, meter.c, startup_stm32f407xx.s
//                                revised
//                          - Coil temperature circuit and pin definitions were modified, so pin controls
//                            were modified accordingly
//                              - Test.c, Iod_def.h, Init.c, Init_def.h revised
//                          - AFE Data Ready pin changed
//                              - Init.c, Init_def.h revised
//                          - Miscellaneous pins were changed
//                              - Iod_def.h, Init_def.h revised
//                      - Fixed bug in timer initialization subroutines
//                          - Init.c revised
//                      - Added Init_UART3() for future use for GOOSE display communications
//                          - Init.c revised
//                      - Deleted EAB test port command as it is no longer used
//                          - Test.c, Test_def.h revised
//                      - Added EAC3 thru EAC6 test port commands for CAM testing
//                          - Test.c revised
//   0.35   191118  DAH - Display processor communications changed from 3.75Mbps to 3.0Mbps because the
//                        display processor now runs at 168MHz, and 3.0MHz is the maximum frequency that
//                        both the display processor and protection processor (120MHz) can derive
//                          - Init.c revised
//                      - Fixed CAM2 test operation so that when Modbus power is turned on, the UART Rx and
//                        Tx pins are configured for UART operation.  When Modbus power is turned off, the
//                        pins are set as GPIO low outputs
//                          - Test.c, Iod_def.h revised
//                      - Added one-cycle and 200msec ADC voltages to the DR2 command
//                          - Test.c revised
//                      - Fixd bug in computing line (ADC) voltages
//                          - Meter.c revised
//                      - Reduced size of ADC cal constants arrays in struct ADC_CAL.  Extra constants were
//                        not used
//                          - Iod.c, Test.c, Meter_def.h, FRAM_Flash_def.h revised
//                      - Changed default scaling gain constants to reflect the rev 4 schematic
//                          - Meter_def.h revised
//                      - Added support for ADC voltages
//                          - Test.c revised
//                      - Added support for Igsrc and In using both a Rogowski coil and a CT
//                          - Iod.c, Test.c, Meter.c, Test_def.h Iod_def.h, Meter_def.h, FRAM_Flash_def.h,
//                            Init_def.h revised
//                      - Fixed bug in loading the AFE with the cal constants.  The phase cal constants were
//                        not being written to the AFE
//                          - Init.c revised
//   0.36   200210  DAH - Display processor communications variables and file renamed from Dispxxx to
//                        DispCommxxx
//                          - Display.c renamed to DispComm.c
//                          - Display_def.h renamed to DispComm_def.h
//                          - Display_ext.h renamed to DispComm_ext.h
//                          - Disp_VarInit() renamed to DispComm_VarInit()
//                          - Disp_Rx() renamed to DispComm_Rx()
//                          - Disp_Tx() renamed to DispComm_Tx()
//                          - DispComm.c, DispComm_def.h, DispComm_ext.h, Init.c, Intr.c, Modbus.c, Test.c
//                            revised
//                      - The Display Processor communications was cleaned up per the spec.  Real-Time Data
//                        Buffer 0 was defined and added.  RTD buffers are now only sent when requested by
//                        the Display Processor, rather than sent as unsolicited Writes.
//                          - DispComm.c, Dispcomm_def.h, DispComm_ext.h, Intr.c, Test.c revised
//                      - Eliminated call to CalcCRC() for Display Processor received messages, as the CRC
//                        is computed as the message is assembled.  Moved CalcCRC() from DispComm.c to
//                        Modbus.c
//                          - DispComm.c, DispComm_ext.h, Modbus.c revised
//                      - Corrected bug in encoding Display Processor transmit packets
//                          - DispComm.c revised
//   0.37   200604  DAH - Removed ARCON Light interrupt from the interrupt structure.  PF10 was the input
//                        signal; it has long been repurposed.
//                          - Init.c revised
//                      - Deleted Init_USB().  USB was moved to the Display Processor
//                          - Deleted Init_USB() from the initialization code
//                          - Init.c, Init_ext.h revised
//                      - Corrected bug in processor-processor communications
//                          - DispComm.c revised
//                      - Additional firmware development of the events code.  Memory allocation in FRAM and
//                        Flash was firmed up, and the basic method for retrieving events information has
//                        been implemented.  This has been done in support of the proc-proc communications,
//                        and was implemented for the USB PXPM events support.  Code to retrieve the
//                        earliest and latest Event IDs was written.
//                          - Events.c, Events_def.h, Events_ext.h, Iod.c, FRAM_Flash_def.h, Demand_def.h,
//                            Test.c revised
//                      - Additional firmware development of the processor-processor comms.  Support for
//                        Read Delayed Requests was added.  These will be used for setpoints and events.  A
//                        command to read the earliest and latest EIDs was written.  Support for Write
//                        Response Requests from an outside process was added.  Usually the outside process
//                        is a Flash or FRAM data retrieval for setpoints or events in response to a Read
//                        Delayed Request.
//                          - DispComm.c, DispComm_def.h, DispComm_ext.h, Iod.c, Iod_def.h, Iod_ext.h
//                            revised
//                      - Added include of Events_def.h for compilation purposes (for FRAM_Flash_def.h)
//   0.38   200708  DAH - Additional firmware development of the processor-processor comms.
//                          - The RTD buffers (stored in the Display Processor's DCI) are now "pushed up" by
//                            the Protection Processor rather than "pulled up" by the Display Processor.
//                            Note, we still support (for now) Read Immediates of the Real-Time Data buffers
//                          - Added code to wait for the Turnaround Time or an Ack after  transmitting a msg
//                          - Corrected the sequence number used in AssembleAck1()
//                          - The test injection command was changed to an Execute Action With Acknowledge
//                            instead of an Execute Action Without Acknowledge
//                          - Added test code to test receiving Write With Ack and Execute Action With Check
//                            commands
//                          - Deleted support for Write Without Acknowledge commands.  They are not used
//                          - Debugged Read Delayed receptions
//                          - DispComm.c, DispComm_def.h, FRAM_Flash_def.h, Intr.c, Iod.c, Iod_def.h,
//                            Iod_ext.h, Meter.c revised
//   0.39   200729  DAH - Additional firmware development of the processor-processor comms
//                          - Modifications to ensure proper full-duplex operation
//                          - Corrected bugs when receiving messages and waiting until response is
//                            transmitted
//                          - General code cleanup
//                          - DispComm.c, DispComm_def.h, Intr.c revised
//                      - Added test code to begin testing the internal RTC
//                          - Added call to Init_Int_RTC() in the initialization code
//                          - Init.c, Init_ext.h revised
//   0.40   200923  DAH - Migrated to IAR 8.5 compiler
//                          - Added cmsis_iccarm.h to project.  This is a copy of the library file in
//                            \Program Files (x86)\IAR Systems\Embedded Workbench 8.4\arm\CMSIS\Core\Include
//                            It had to be modified because there were duplicate definitions in
//                            \Program Files (x86)\IAR Systems\Embedded Workbench 8.4\arm\inc\c\cmsis_iar.h
//                            Both files are included in arm_math.h, which is required in Meter.c for the
//                            harmonics calculations
//                            This file is use instead of the library file because of the search path order
//   0.41   201028  DAH - Revised processor-processor communications based on testing
//                          - DispComm.c revised
//                      - Corrected error in retrieving the earliest and latest Event IDs
//                          - Events.c revised
//                      - Added test command to retrieve setpoints for proc-proc comms
//                          - Iod.c revised
//   0.42   201202  DAH - Cleaned up internal RTC code and replaced external RTC with internal RTC
//                          - Init_Int_RTC() renamed to Init_IntRTC()
//                          - Revised call to RTC_to_SysTickTime() to include pointer to SysTickTime
//                          - Added Check_IntRTC() to the initialization code
//                          - Deleted call to Init_RTC() and associated code for the external RTC from the
//                            initialization code
//                          - Added setting RTC_State to 1 in the 5-minute anniversary task list in the main
//                            loop, and added call to IntRTC_Update() in the main loop
//                          - Iod.c, Iod_ext.h, Init.c, Init_def.h, Init_ext.h revised
//                      - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - RealTime.c, RealTime_def.h, and RealTime_ext.h created and added to the
//                            project
//                          - Deleted RTC and Internal Time subroutines and associated variables from Iod.c,
//                            Iod_def.h, and Iod_ext.h and added to RealTime.c, RealTime_def.h, and
//                            RealTime_ext.h
//                          - Added call to RT_VarInit() in the initialization code
//                          - Added include of RealTime_def.h
//                          - Iod.c, Iod_def.h, Iod_ext.h, Intr.c, Events.c, Meter.c, Init.c, Test.c,
//                            CAMCom.c, Prot.c, Demand.c, Modbus.c, DispComm.c revised.  RealTime.c,
//                            RealTime_def.h, and RealTime_ext.h added
//                      - Revised "TM" test port command to accomodate the internal RTC instead of the
//                        external RTC and added displaying the system time in addition to the internal
//                        RTC time
//                          - Test.c revised
//   0.43   210115  DAH - Added high-speed processor-processor communications for GOOSE testing
//                          - Call to Init_UART3() added to the initialization code
//                          - Added EAG test command to initiate one or more consecutive GOOSE transmissions
//                            to the Display Processor - FOR TESTING ONLY
//                          - Init.c, Init_ext.h, Intr.c, DispComm.c, DispComm_def.h, DispComm_ext.h, Test.c
//                            revised
//                      - Corrected possible bug in enabling DMA streams.  According to the STM32F407
//                        Reference Manual (RM0090), the event flags for the DMA stream must be cleared
//                        before enabling the stream.  A problem was seen in the Display Processor where
//                        the bit could not be set without doing this, so the code was added for all
//                        streams in both processors.
//                          - DispComm.c, Init.c, Intr.c revised
//   0.44   210324  DAH - Additional support for high-speed proc-proc communications for GOOSE testing
//                          - Added more GOOSE message types for transmitting and receiving
//                          - Modified sequence number handling
//                          - Added processing of "stacked" received messages
//                          - Revised EAG test port command to send out different types of GOOSE messages
//                            instead of multiple requests of the same GOOSE message.
//                          - Added test command to send out consecutive GOOSE messages (Trip, ZSI, XFER) on
//                            a one-second rate
//                          - Revised transmit timing to ensure display processor has time to process an
//                            incoming message
//                          - Test.c, main.c, DispComm.c, DispComm_def.h revised
//                      - Added code to turn on AFE on power up for new rev 6 (with H7 display processor)
//                        boards.  This change does not affect the old rev 4 boards (pin was TP7, unused)
//                          - Init_def.h revised
//                      - Added test code to test the high-speed comms DMA operation in the H7 display
//                        processor.  Code is presently commented out.  FOR H7 TESTING ONLY!!
//                          - DispComm.c revised
//   0.45   210504  DAH - Revisions for H743 board revision
//                  BP      - Iod_def.h, Init_def.h revised
//                      - Corrected problem with retrieving test injection cal constants.  This was
//                        incorrectly being done in the main loop (TP_Top()).  All SPI2 accesses must be
//                        done through the SPI2 managers, because SPI2 is activated in the 1.5msec timer
//                        interrupt.  SPI2 was getting corrupted and was hanging.
//                          - Iod.c, Iod_def.h, Test.c, Test_def.h, Iod.c revised
//                      - (BP) Added long delay protection, added settings for instantaneous and short delay
//                        protection, and added ZSI to short delay protection
//                          - Prot.c, Prot_def.h, Prot_ext.h revised
//                      - (BP) Moved Prot_VarInit() to execute after SYSCLK is 120MHz
//                      - (BP) Added calls to LongDelay_Prot() and Long_IEE_IEC_Prot() to main loop
//                      - Activated instantaneous tripping function for arc flash demo
//   0.46   210521  DAH - Revisions to correct bugs and add enhancements to waveform captures for the arc
//                        flash demo unit
//                          - Events.c, Iod.c, Test.c, Test_def.h revised
//                      - Added code to turn off protection for 250msec after the TA is fired so TA is not
//                        continuously fired during the arc
//                          - Intr.c, Intr_ext.h, Prot.c revised
//   0.47   210525  DAH - Corrected bugs in ADC voltage calibration
//                          - Test.c revised
//   0.48   210730  DAH - Corrected minor bug in test port command to display waveforms
//                          - Test.c revised
//                      - Corrected minor bug in instantaneous protection
//                          - Prot.c revised
//                      - Revised unbalance calculations
//                          - Meter.c revised
//                      - Corrected bug when switching from ADC to AFE samples
//                          - Meter.c revised
//                      - Added temporary test port command, "MT" to change instantaneous protection pickup
//                        level.  Reference *** DAH 210730
//                          - Test.c, Test_def.h revised
//   0.49   220131  DAH - Revised code to get DMA working for Modbus transmissions
//                        NOTE: Modbus is now hard-coded for 9600bps, even parity, 1 stop bit,
//                        device address = 7.  In addition PC6 and PC7 are configured for UART6 Rx and Tx
//                        operation out of a reset.   *** DAH
//                          - Modbus.c, Init.c, Init_ext.h, Init_def.h, Intr.c, startup_stm32f407xx.s
//                            revised
//                          - Input parameter added to call to Init_UART6()
//   0.50   220203  DAH - Commented out calls to CAM_Rx() and CAM_Tx() for CAM2 port.  It is now
//                        "hard-coded" for Modbus
//                      - Added some metered values to make Modbus messaging simpler
//                          - Meter.c, Iod_def.h, Meter_def.h, Meter_ext.h, Demand_def.h, Iod_def.h revised
//                      - Continued Modbus development.  Added code to process Read requests for real-time
//                        values
//                          - Modbus.c, Modbus_def.h revised
//   0.51   220304  DAH - Continued Modbus development.  Added more code to process Read requests for
//                        real-time values
//                          - Modbus.c, Meter.c, Meter_def.h, Meter_ext.h, RealTime.c, RealTime_ext.h
//                            revised
//                      - Min/max current values have switched from 200msec values to one-cycle values
//                          - Intr.c, Meter.c, Meter_ext.h, Modbus.c, Test.c revised
//                      - Added per-phase current unbalance with min/max
//                          - Meter.c. Meter_def.h, Meter_ext.h, Modbus.c revised
//                      - Current unbalance has switched from 200msec value to one-cycle value
//                          - Meter.c, Meter_ext.h, Modbus.c, DispComm.c revised
//                      - Min/max voltage values changed from 200msec values to one-cycle values
//                          - Intr.c, Meter.c, Meter_ext.h, Modbus.c, Test.c revised
//                      - Voltage unbalance has switched from 200msec value to one-cycle value
//                          - Meter.c, Meter_ext.h, Modbus.c, DispComm.c revised
//                      - Added per-phase voltage unbalance with min/max
//                          - Meter.c, Meter_ext.h, Modbus.c revised
//                      - Calc_Meter_ADC_Voltage() renamed to Calc_ADC_200ms_Voltage()
//                      - Calc_Voltage3() renamed to Calc_ADC_OneCyc_Voltage()
//                      - Min/max ADC voltage values have switched from 200msec values to one-cycle values
//                          - Meter.c, Meter_ext.h, Modbus.c, Intr.c revised
//                      - Added 5-minute anniversary values
//                          - Calc_5minAverages() added to 200msec function calls
//                          - Demand.c, Demand_def.h, Demand_ext.h revised
//   0.52   220309  DAH - Completed 5-minute anniversary values
//                          - Demand.c, Demand_def.h, Demand_ext.h revised
//                      - Minor enhancement to power metering code
//                          - Meter.c revised
//                      - Added 5-minute anniversary values to Modbus
//                      - Slight revision to Modbus map with temperature objects to align with PXR25
//                      - Corrected bug in energy conversions
//                          - Modbus.c revised
//                      - Added net real energy (WHr) and total reactive energy (VarHr)
//                          - Meter.c, Meter_ext.h, Demand_def.h, Modbus.c revised
//                      - Revised demand computation to make logging anniversaries completely independent of
//                        demand window anniversaries, even for fixed windows
//                          - Demand.c revised
//   0.53   220325  DAH - Added timestamp to maximum temperature and added internal max temperature
//                          - Iod.c, Iod_def.h revised
//                      - Added min/max THD values
//                          - Meter.c, Meter_def.h, Meter_ext.h, Intr.c, Modbus.c revised
//                      - Corrected bug with Modbus word order
//                          - Modbus.c revised
//                      - Eliminated instantaneous harmonics (no longer required),added 40th harmonic, and
//                        cleaned up the code
//                          - Meter.c revised
//                      - Added Modbus support for harmonics
//                          - Meter.c, Meter_def.h, Meter_ext.h, Modbus.c revised
//   0.54   220330  DAH - Bug fix and enhancements to per-phase current and voltage computations
//                          - Meter.c revised
//                      - Renamed CurOneCycUnbal to CurUnbalTot and added max total current unbalance
//                      - Renamed VolAFEOneCycUnbal to VolUnbalTot and added max total voltage unbalance
//                          - DispComm.c, Modbus.c, Meter.c, Meter_ext.h revised
//                      - Added LD_Bucket, LD_BucketMax, LD_BucketMaxTS for future thermal overload warning
//                          - Prot.c, Prot_ext.h, Modbus.c revised
//   0.55   220420  DAH - Added short-delay overload values (SD_BucketMax, SD_BucketMaxTS) for future
//                        short-delay overload logging.  Note, these values still need to be added to the
//                        metering code
//                          - Prot.c, Prot_ext.h, Modbus.c revised
//                      - Added Real-Time Data buffer processor-processor communications
//                          - DispComm.c, DispComm_def.h, Meter.c, Meter_ext.h revised
//   0.56   220526  DAH - Fixed minor bugs
//                          - Iod.c, Modbus.c revised
//                      - Added Modbus RTU freeze/thaw harmonics commands
//                          - Modbus.c, Meter_ext.h revised
//                      - Added harmonics to the processor-processor communications
//                          - DispComm.c, DispComm_def.h revised
//                      - Changed USB Boost Enable to be active after a reset
//                          - Init_def.h revised
//   0.57   220628  DAH - Reversed Ig and In in the proc-proc communications
//                          - DispComm.c, Meter_ext.h revised
//   0.58   220829  DAH - Fixed bug in test port manager when retrieving test injection cal constants
//                          - Test.c revised
//                      - Fixed startup circuit pin control definitions
//                          - Iod_def.h revised
//                      - Fixed bug in internal RTC initialization
//                          - In initialization code, if RTC is bad and needs to be initialized, moved call
//                            to Init_IntRTC() into the main loop.  The 32.768KHz crystal is not up yet, and
//                            it takes about 500msec before it is ready, so the RTC cannot be initialized in
//                            the initialization code.
//                          - RealTime.c, RealTime_ext.h, Init.c revised
//                      - Added Time of Last Reset to the Modbus map
//                          - Modbus.c revised
//                      - Added Setpoints Group 0 support
//                          - Added call to Get_Setpoints() in the initialization code to retrieve the
//                            setpoints from Frame FRAM
//                          - Setpnt.c, Setpnt_def.h, Setpnt_ext.h added to the project
//                          - Iod.c, Iod_ext.h revised
//  0.59    220831  DAH - Switched Black Box FRAM and Flash chips, so that the Flash Chip is now on SPI1 and
//                        the Black Box FRAM is on SPI2.  The Black Box function (10-cycle waveform capture
//                        without aux power) has been eliminated.  All SPI2 operations now occur in the main
//                        loop functions, not in the interrupt.  Therefore, the SPI2 manager has been
//                        eliminated, and the corresponding SPI2 functions (mainly FRAM reads and writes)
//                        have been moved to where the request flags were set.
//                        FRAM functions that were in the Flash handlers have been moved to the main loop
//                        functions.
//                        The Flash waveform writes (now on SPI1) are still handled by the Timer 4 interrupt
//                          - In the main loop, replaced code that set S2NF_ENERGY_RD with a call to
//                            FRAM_ReadEnergy()
//                          - Intr.c, Intr_ext.h, Intr_def.h, FRAM_Flash_def.h, IntrInline_def.h, Iod.c,
//                            Iod_def.h, Iod_ext.h, Meter.c, Demand.c, Test.c, DispComm.c, DispComm_def.h,
//                            Events.c, Event_ext.h, Init.c, Init_ext.h revised
//                      - Revised Trip event handling per the new SPI2 FRAM and Flash write procedures
//                          - Prot.c, Event.c, Event_ext.h revised
//                      - In the processor-processor communications, cleaned up the code to generate Write
//                        Response messages to Delayed Read Requests
//                          - DispComm.c, DispComm_def.h revised
//  0.60    220928  DAH - Added summary log entries for Trip events
//                          - Events.c revised
//                      - Added test code (presently commented out) to force an instantaneous trip event
//                          - Prot.c revised
//                      - Added "DV" command to display summary event logs
//                          - Test.c, Test_def.h revised
//                      - Added setpoints definitions, initialization, and checking
//                          - Added call to Setp_VarInit() in the initialization code
//                          - Added calls to Get_Setpoints() in the initialization code
//                          - Added call to Check_Setpoints() in the 5-minute anniversary in the main loop
//                          - Setpnt.c, Setpnt_def.h, Setpnt_ext.h, FRAM_Flash_def.h revised
//  0.61    221001  DB  - Events development
//                          - Added test code to initialization to insert summary logs
//                          - Events_def.h, Events_ext.h, FRAM_Flash_def.h, Meter.c, Meter_def.h,
//                            Meter_ext.h, Test.c, Test_def.h revised
//  0.62    221027  DAH - Corrected bug in processor-processor comms
//                          - DispComm_def.h revised
//                      - Added setpoint buffers to the processor-processor comms
//                          - Setpnt.c, Setpnt_ext.h, DispComm.c, DispComm_def.h, Iod.c, Iod_def.h,
//                            Iod_ext.h, Meter.c revised
//                      - Added support for multiple setpoints sets
//                          - Setpnt.c, Setpnt_def.h, Setpnt_ext.h revised
//                      - Corrected bugs in checking setpoints
//                          - Setpnt.c revised
//                      - Moved retrieving the non-protection setpoints into the first-pass portion of the
//                        main loop after the startup time is computed
//                      - Revised FRAM_Read() to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//                          - Iod.c, Iod_ext.h, Events.c, Setpnt.c, Test.c revised
//   0.63   221111  DB  - Continued events development
//                          - Events.c, Events_ext.h, Meter.c, Test.c, Test_def.h revised
//   0.64   221118  DAH - Continued events development
//                          - Events.c, Events_ext.h, Test.c revised
//                      - Minor setpoints modifications
//                          - Setpnt.c, Setpnt_def.h revised
//                      - In initialization and main loop, added temp1 to hold the number of setpoints,
//                        instead of recalculating each time through the loop that loads the setpoints into
//                        RAM
//                      - Revised group1 setpoints to handle the read-only values, which are duplicates of
//                        the corresponding group0 values
//                          - In the initialization code, added code to overwrite the 5 read-only group1
//                            values with the corresponding group0 values
//                          - Setpnt.c, DispComm.c revised
//                      - Fixed bug in setpoints comms to enable a setpoint set to be retrieved that is not
//                        the active set
//                          - Calls to Get_Setpoints() now include the setpoints set
//                          - DispComm.c, Setpnt.c, Setpnt_ext.h revised
//                      - Fixed bug in setpoints when writing setpoints
//                          - DispComm.c revised
//   0.65   221201  DB  - Events EID bug fix
//                          - Events.c, Events_ext.h, Test.c revised
//   0.66   221206  DAH - Events bug fixes
//                          - Events.c revised
//                      - Enhanced Events search code to reduce the execution time
//                          - Events.c revised
//   0.67   221209  DAH - Variables and functions renamed from "SPI2" to "SPI1" for the serial Flash chips
//                          - Iod.c, Iod_ext.h, Iod_def.h, Intr.c, Events.c, Test.c, Meter.c revised
//                          - ManageSPI2Flags() renamed to ManageSPI1Flags()
//                      - Revised code for new hardware revision
//                          - Init_def.h, Iod_def.h revised
//                      - Added TV command to test the protection availability LEDs
//                          - Test.c, Test_def.h revised
//   0.68   230124  DB    calling extended capture routines when the flags are set
//                        - for Ext Cap OneCycle Aniversary
//                        - for Ext Cap 200ms Aniversary
//                        - Waveform Ext Capture added into IntEventBuf[]
//   0.69   230220  DAH - Cleaned up some of the events code
//                          - Deleted Strip-Chart and Minor Alarm events
//                          - Added support for Extended Captures and Disturbance Captures
//                            (code not completed yet)
//                          - Revised the event definition codes based on requirements discussions
//                              - Events.c, Intr.c, Iod.c, Meter.c, Test.c, Events_def.h, Events_ext.h,
//                                FRAM_Flash_def.h, Iod_def.h, Iod_ext.h revised
//                          - In struct NEWEVENTFIFO, renamed Type to Code
//                              - Demand.c, Events.c, Proct.c revised
//                      - Began revising the proc-proc comms code for events.  Most events do not have to be
//                        delayed reads (code not yet complete)
//                              - DispComm.c, Events.c, Events_ext.h revised
//                      - Revised setpoints per new requirements
//                          - Setpnt.c, Setpnt_def.h revised
//                  DB  - Continued extended capture events development - disturbance captures added
//                              - Events.c, Events_def.h, Events_ext.h, Iod.c, Iod_ext.h, Meter.c,
//                                Meter_def.h, Meter_ext.h, Test.c, Test_def.h revised
//   0.70   230224  BP  - Added calls to the trip and alarm functions in the half cycle, one cycle and one
//                        second anniversaries.
//   0.71   230228  DAH - Revised alarm events.  Demand overpower events do not initiate a disturbance
//                        capture
//                              - Events_def.h revised
//                      - Setpoints revised and Group 11 added
//                              - Setpnt.c, Setpnt_def.h, FRAM_Flash_def.h revised
//   0.72   230320  DAH - Fixed neutral coil control
//                          - Test.c, Iod_def.h revised
//                      - Changed default current scaling constants from N-Frame to Magnum Standard
//                          - Meter_def.h revised
//                      - Fixed bug in Instantaneous protection
//                          - Intr.c revised
//                      - BreakerHealth_Alarm() and Ground_Fault_PreAlarm() moved from half-cycle
//                        anniversary to one-cycle anniversary (main loop is too long to support half-cycle
//                        anniversary functions and these two functions are ok executing each cycle)
//                          - Intr.c, Intr_ext.h revised
//                      - Added read/write breaker rating factory command to processor-processor comms
//                          - DispComm.c, DispComm_def.h, Prot.c, Prot_ext.h revised
//                      - Added code to initialization to retrieve breaker frame rating from Frame FRAM
//                      - Moved call of Prot_VarInit() to after the Group 0 and 1 setpoints are retrieved
//                      - Added code to set the read-only frame rating setpoints to the rating read from the
//                        config data in Frame FRAM
//                          - Prot.c revised
//                      - Added thermal memory storage to FRAM
//                          - FRAM_Flash_def.h revised
//                      - Added demand window type and interval, and Time of Last Reset for voltage and
//                        current min/max to demand real time data buffer
//                          - DispComm.c, Demand.c, Demand_def.h, Demand_ext.h revised
//                      - Fixed bugs with Group 11 setpoints
//                          - Setpnt.c, DispComm.c revised
//                  DAH - Revised Events flags operation to ensure foreground cannot overwrite a flag being
//                  DB  - altered in an interrupt.  With the revision, the events control flags for writing
//                        waveforms to Flash (SPI1_Flash.req, SPI1_Flash.Ack) are only changed in either the
//                        foreground (SPI1_Flash.Req) or the 1.5msec timer interrupt (SPI1_Flash.Ack)
//                          - Events.c, Iod.c, Meter.c revised
//                      - Fixed bug in extended capture writes
//                          - Iod.c revised
//                  BP  - Added blinker function for high load alarm
//                          - Intr.c, Iod_def.h, Iod_ext.h revised
//                      - Updated Cause of Trip function LEDs
//                          - Iod.c, Iod_ext.h revised
//                  BP  - Added calls to UpdateThermalMemory(), TA_Volt_Monitoring(), Calc_BatteryVolt(),
//                        and Service_NonCOT_Leds()
//   24     230323  DAH - Added initial support for PriSecCause status for upcoming EMC testing
//                          - Added call to Update_PSC() to the main loop
//                          - DispComm.c, Meter.c, Meter_def.h, Meter_ext.h revised
//                      - Added support for displaying long delay time to trip
//                          - DispComm.c, Prot_ext.h revised
//                      - Modified Highload_Alarm() to handle delay time setpoints in 0.1s resolution
//                      - Added alarms and extended captures to Highload_Alarm()
//                          - Prot.c revised
//                      - Revised extended captures to handle multiple events
//                          - Events.c, Events_def.h, FRAM_Flash_def.h revised
//                      - Fixed bug in extended waveform capture
//                          - Iod.c revised
//   25     230403  DAH - Fixed PriSecCause status
//                          - Meter.c revised
//                      - Fixed bug in Extended Capture indices initialization
//                          - Events.c revised
//                      - Added code to initialization to insert power-up summary events
//                      - Eliminating clearing FRAM when restoring unit test command is issued
//                          - Test.c revised
//                      - Modified DV test command to show the number of Summary Logs
//                          - Test.c revised
//                      - Added reading Event Summary logs to proc-proc comms
//                          - Events.c, Events_ext.h, DispComm.c revised
//                      - Fixed bug inserting High Load Alarm 2 event
//                          - Prot.c revised
//                      - Began adding support for reading waveforms via proc-proc comms
//                          - DispComm.c, DispComm_def.h, Iod.c, Iod_ext.h, Meter.c, Meter_ext.h revised
//                      - Fixed bug with extended captures
//                          - Event.c revised
//                      - Added support for OpenFlg when reading the switches.  OpenFlg is used in the
//                        protection functions
//                          - Iod.c revised
//                  KT  - Added 61850 GOOSE Capture command and IEC61850 GOOSE development
//                          - DispComm.c, DispComm_def.h, DispComm_ext.h, Test.c revised
//                          - Added GOOSE capture test code to main loop
//   26     230403  DAH - Added extended capture one-cyc and 200msec values to proc-proc comms
//                          - DispComm.c, Events.c, FRAM_Flash_def.h revised
//   27     230404  DAH - Added support for Disturbance Capture Read Commands
//                          - DispComm.c, Event.c, Event_ext.h revised
//                      - Cleaned up Disturbance Captures
//                          - Replaced multiple calls to DisturbanceCapture() with a single call in the
//                            one-cycle anniversary after all protection and metering has been completed
//                          - Event.c, Event_def.h revised
//                      - Added code to initialization to insert a power-up summary event
//                      - Added temporary patch for AE demo that places LDPU flag in Pri/Sec/Cause status
//                          - Meter.c revised
//   28     230406  DAH - Fixed bug in proc-proc comms waveform reads
//                          - Iod.c, DispComm.c
//                      - Fixed bug in retrieving events info
//                          - Events.c revised
//                      - Added GOOSE capture request to high load alarm 2 for demo
//                          - Prot.c revised
//                      - Added GOOSE capture command support
//                          - Disp_Comm.c, Events,c, Events_ext.h revised
//                      - Added temporary code to clear events if reset switch pressed for 10 seconds
//                        This code will be removed after the demo
//                          - Iod.c revised
//   29     230406  DAH - Fixed bug clearing extended capture events
//                          - Events.c revised
//                      - Added code to initialize TU_State_TestUSBMode to False.  It was preventing high
//                        load alarms from occurring because it was sometimes coming up True
//                          - Prot.c revised
//                      - Fixed bugs in retrieving extended capture 6s and 60s values
//                          - DispComm.c revised
//                      - Corrected bugs in calling AssembleTxPkt() with message lengths greater than 126.
//                        The source data buffer cannot be TxBuf[] if the message length is greater than
//                        126.  This can cause the buffer to be corrupted.
//                          - DispComm.c revised
//                      - Corrected bugs retrieving waveforms
//                          - DispComm.c, Iod.c revised
//   30     230412  DAH - Corrected bug encoding proc-proc comms transmit packets
//                          - DispComm.c revised
//                      - Corrected bugs retrieving 6sec and 60sec values for proc-proc comms
//                          - DispComm.c revised
//                      - Corrected bug capturing 60sec values
//                          - Events.c, Meter.c, Meter_ext.c revised
//   31     230414  DAH - Revised code to handle instances where multiple extended capture events occur.  In
//                        this case, the initial extended capture event initiates Summary, Snapshot,
//                        Waveform, and RMS capture entries.  Subsequent events only cause Summary events to
//                        be made until the Extended Capture is completed (after 60 seconds).
//                          - Events.c, Events_def.h revised
//                      - Adjusted waveform capture starting index (added one cycle) so that the correct
//                        number of pre- and post-event cycles are captured.
//                      - Corrected the timestamp for extended capture waveform captures.  Note, this still
//                        needs to be done for Alarm and Trip waveform captures
//                          - Intr.c revised
//   34     230424  DAH - Fixed minor bug causing startup calibration to be called incorrectly
//                          - Intr.c revised
//   36     230428  DAH - Revised events code to decouple waveform captures from snapshot and summary
//                        captures, so that snapshot and summary entries will be logged immediately for new
//                        events even as a previous event's waveform capture is still being stored
//                          - Events.c, FRAM_Flash_def.h revised
//                      - Corrected time stamps for waveform captures.  The time stamp is the time of the
//                        first sample
//                          - Intr.c, Intr_ext.h, Prot.c revised
//                      - Adjusted waveform capture starting index back to the way it was (removed one
//                        cycle) so that the correct number of pre- and post-event cycles are captured
//                          - Intr.c revised
//                      - Cleaned up the waveform capture process.  Deleted code that clears
//                        Trip_WF_Capture.Req in DMA1_Stream0_IRQHandler().  The flag is cleared in
//                        EventManager() when capture processing begins
//                          - Intr.c revised
//                      - Replaced the hard-coded number of post-cycles with the appropriate setpoint
//                          - Intr.c revised
//                      - Revised the waveform capture process to make extended captures (almost) the same
//                        priority as alarm captures.  Alarm waveform requests will no longer abort an
//                        extended capture waveform capture
//                          - Iod.c revised
//   37     230516  DAH - Added High Load 1 support
//                          - Prot.c revised
//                      - Revised extended Capture process to couple waveform and RMS profile captures.  RMS
//                        profile captures will not be done if waveforms are not captured (due to an alarm
//                        trip capture in progress).  Also did other extended capture code cleanup
//                          - Events.c, Events_ext.h, Setpnt_def.h revised
//                          - Revised calls to ExtendedCapture() to occur if ExtCap_ReqFlag or
//                            ExtCap_AckFlag, since Req flag is now cleared when Ack flag is set in
//                            ExtendedCapture()
//                      - Added code to set the extended capture EID to 0 when events are cleared.  This is
//                        an invalid vallue for an Extended Capture EID
//                          - Events.c revised
//                      - For Disturbance Captures, added code to wait until the event has been written
//                        before allowing another event of the same type to begin (to ensure no data is
//                        overwritten)
//                          - Events.c, Events_def.h revised
//                      - Revised Disturbance Captures to allow multiple disturbance capture types to be
//                        processed "simultaneously" and independently.
//                          - Events.c, Events_def.h, Events_ext.h revised
//                      - Added capturing the EID of the event that initiates a Disturbance Capture to the
//                        Disturbance Capture event logs.  This can be used to tie the Disturbance Capture
//                        information to the associated Alarm, Extended Capture, and/or Summary Event
//                          - Prot.c, Events.c, FRAM_Flash_def.h revised
//                      - Fixed ExtCap_ReqFlag external declaration
//                          - Events_ext.h revised
//                      - Added disturbance captures and GOOSE global captures to LongDelay_Prot() and
//                        cleaned up the global captures in Highload_Alarm()
//                          - Prot.c, Events.c, Events_ext.h
//                      - Added Alarm, Test Alarm, and Disturbance capture event initiation to numerous
//                        protection and alarm functions
//                          - Events.c, Prot.c, Events_def.h revised
//   38     230518  DAH - Fixed bugs with disturbance capture processing
//                          - Prot.c revised
//                      - Fixed bug in read response for disturbance capture logs
//                          - DispComm.c revised
//                      - Corrected Overvoltage and Undervoltage Alarm defaults
//                          - Setpnt.c revised
//                      - Fixed minor bug in extended capture processing when capturing the EID for
//                        disturbance captures
//                          - Events.c revised
//   40     230531  DAH - Revised snapshot captures (added values) to snapshot captures
//                          - Events.c, Meter.c, Meter_def.h, Meter_ext.h, FRAM_Flash_def.h revised
//                      - Fixed bug in saving EID for extended captures
//                          - Events.c, FRAM_Flash_def.h revised
//                      - Added proc-proc comms support for reading Snapshot Summaries and Snapshot logs
//                          - Events.c, Events_ext.h, DispComm.c, DispComm_def.h revised
//                      - Revised Disturbance Read proc-proc comms (eliminated EID information)
//                          - Events.c, Events_ext.h, DispComm.c revised
//                      - Added ADC voltages to "DWU" test port command
//                          - Test.c revised
//                      - Cleaned up events proc-proc comms code
//                          - DispComm.c, DispComm_def.h revised
//   42     230623  MAG - Revised frequency measurement code to improve phase difference measurement.
//                        Frequency/Phase measurement moved from Timers 10 and 11 to Timer 4 and 1.5 ms
//                        Timer/Interrupt for SPI2 Event handler was moved to Timer 10.
//                          - Init.c, Intr.c, Init_def.h, Intr_def.h revised
//                      - Deleted call to Init_TIM11() which is no longer used.
//   44     230623  DAH - Fixed bugs when reading disturbance capture and summary events.  Message was not
//                        encoded properly when there were zero events to return.
//                          - DispComm.c, Events.c revised
//                      - Revised EventGetWfEIDs() to also return whether a requested EID is present
//                          - Events.c, DispComm.c, Events_ext.h revised
//                      - Fixed bugs reading snapshot events
//                          - Events.c revised
//                      - Revised the way disturbance captures are read, when reading by EID (as opposed to
//                        by index).  The EID that INITIATED the capture (not the EID of the actual event
//                        entry), is used.  This EID is stored as data in the log.  It is more useful to use
//                        this EID because it ties the disturbance capture to the other event log entries
//                        for the Event
//                          - Events.c revised
//                      - Added support for Override Trips.  Deleted the override interrupt.  Overrides are
//                        detected in the sampling interrupt, which occurs every 208usec.  This delay is
//                        acceptable.  We do not need a separate interrupt.
//                          - Intr.c, Prot.c, Prot_ext.h revised
//                  BP  - Added breaker frame and breaker configuration definitions
//                          - Prot_def.h revised
//   46     230703  DAH - Added protection enable qualifier around all protection functions (disables
//                        protection for 250msec after trip is initiated)
//                          - In main loop, added Prot_Enabled qualifier around Trip routines
//                      - Added proc-proc comms support for reading Waveform Capture EIDs
//                          - DispComm.c, Events.c, Events_ext.h revised
//   49     230706  DAH - Added Setpoints Group 12
//                          - Setpnt.c, Setpnt_ext.h, can_setpoints.c, Setpnt_def.h revised
//   53     230725  VD  - Added Modbus RTU setpoint reads and writes
//                      - Fixed bug with Modbus write multiple register command
//                          - Modbus.c, Setpnt.c revised
//   54     230801  DAH - Revised Group1 setpoints definitions for short delay disturbance captures and to
//                        match the Frame FRAM
//                          - Setpnt.c, Setpnt_def.h revised
//                      - Added SDPU and GF disturbance captures
//                          - Events.c, Events_def.h, Events_ext.h, Prot.c revised
//                      - Added code to compute how close we came to tripping for all disturbance captures
//                          - Events.c, Prot.c, Prot_ext.h revised
//                      - Added ground fault pre-alarm events
//                          - Prot.c revised
//                      - Corrected event capture for long delay protection
//                          - Prot.c revised
//                      - Revised long delay protection to clamp the tally register to the trip threshold so
//                        the thermal capacity never exceeds 100%.  The concern is if a large current spike
//                        occurs, the thermal memory will be very high and the unit will keep tripping on
//                        when the breaker is closed
//                          - Prot.c revised
//                      - Fixed bug with disturbance capture processing
//                          - Prot.c revised
//                      - Changed trip and alarm timer definitions from float to uint16_t to save RAM space
//                          - Prot.c revised
//                      - Revised reverse power disturbance captures to use max reverse power as the
//                        critical value
//                          - Prot.c, Prot_ext.h, Event.c revised
//                      - Revised over power disturbance captures to use max over power as the
//                        critical value
//                          - Prot.c, Prot_ext.h, Event.c revised
//                      - Corrected bug in power factor protection
//                          - Prot.c revised
//           230802 BP  - Added calls to Ovr_VarInit() and MM_Status_Update()
//                      - Added Setpoints5.stp.RevSeq_T_A_OFF for calling Phase Rotation protection
//   58     230810  DAH - Minor revision to battery voltage subroutine
//                          - Meter.c revised
//                      - Added calls to Gen_Values() (updates the protection parameters) whenever new
//                        setpoints are written
//                          - DispComm.c, Prot.c, Prot_ext.h, Events_def.h revised
//                      - Added support for setpoint set changes
//                          - DispComm.c, Setpnt.c, Setpnt_ext.h, Events_def.h revised
//                      - Replaced code in initialiation that loads Gr0 and Gr1 setpoints with call to
//                        Load_SetpGr0_Gr1().  This subroutine is also used when setpoint set is changed.
//                      - Replaced code in main loop that loads remaining setpoint groups with call to
//                        Load_SetpGr2_LastGr().  This subroutine is also used when setpoint set is changed.
//                      - Fixed minor bug reading Group 0 setpoints
//                          - Setpnt.c revised
//                      - Added event if a Frame module error is found when checking setpoints
//                          - Setpnt.c revised
//                      - Added Vbattery to proc-proc comms
//                          - DispComm.c revised
//   66     230825  DAH - Added initial support for diagnostics counters
//                          - Diag.c, Diag_def.h, Diag_ext.h added to the project
//                          - DispComm.c, DispComm_def.h revised
//                      - Corrected minor bug in FRAM map
//                          - FRAM_Flash_def.h revised
//   69     230828  DAH - Added support for transmitting time to the display processor and receiving time
//                        from the display processor
//                          - Added code to initialization to set flag to initiate sending time to the
//                            display processor
//                          - DispComm.c, DispComm_def.h, DispComm_ext.h, Iod_def.h, Test.c, Init.c,
//                            Intr.c revised
//   72     230906  DAH - Added test port DQ command to display the firmware revision info for all three
//                        processors.  Note, both the protection and display micro revisions are stored as
//                        constants in Test.c.  This will likely be changed in the future.  Ther override
//                        micro revision is received from the override micro (this is ok)
//                          - Test.c, Test_def.h, DispComm.c revised
//                      - Fixed bug in transmitting time to the display processor after a reset.
//                          - Added one-second delay in the main loop before setting the flag to initiate
//                            sending time to the display processor.  The display processor was not ready to
//                            receive time immediately after the protection processor powered up
//                      - Commented out code that writes firmware revision, catalog number, etc. into FRAM
//   82     230928  DAH - General code cleanup
//                          - DispComm.c revised
//                      - Finalized the code to handle firmware version info
//                          - DispComm.c, DispComm_def.h, DispComm_ext.h, Test.c, FRAM_Flash_def.h revised
//   93     231010   BP - Added Manufacture Mode checks for calling protection routines
//                      - Added code for the Firmware Simulated Test and the Hardware Sec Injection Test
//   94     231010  DAH - Moved user waveform captures out of the proc-proc comms processing subroutine and
//                        into a separate subroutine and revised user waveforms to handle multiple capture
//                        requests
//                          - Intr.c, Intr_def.h, Intr_ext.h, Meter.c, Meter_def.h, DispComm.c,
//                            DispComm_def.h, Test.c, Test_def.h revised
//                      - Added proc-proc comms support for test injection commands
//                          - DispComm.c, DispComm_def.h revised
//                      - Moved Temperature data object from proc-proc comms buffer 0 to buffer 3 (this is
//                        updated less frequently, and added one-cycle currents to buffer 0
//                          - DispComm.c revised
//   95     231011  DAH - Fixed bug that delayed AFE comms after a cold start for ~120msec.  Bug was due to
//                        a variable being initialized in the main loop.  Entire code was eliminated as it
//                        is no longer used
//                          - Intr.c, Test.c, Test_ext.h revised
//                      - Moved TP_AFEIntOff initialization from Test_VarInit() to Meter_VarInit() to ensure
//                        it is initialized before sampling starts
//                          - Test.c, Meter.c revised
//   96     231012  DAH - Fixed bug to display one-cycle currents when test injection is performed
//                          - DispComm.c revised
//                      - Revised test injection processing when command received
//                          - DispComm.c revised
//   98     231017  DAH - In the main loop, split calls to Update_Std_Status() and Update_PSC() into
//                        separate statements to ensure both subroutines are called
//                      - Revised trip event handling to ensure binary and PSC status are updated before
//                        storing snapshot values
//                          - Events.c revised
//                      - Corrected bug where test injection variables are not initialized
//                          - DispComm.c revised
//                      - Eliminated code that resets alarm flags when the unit trips.  The alarm flag is
//                        only cleared when the alarm condition is removed
//                          - Prot.c revised
//                      - In main loop and sampling interrupt, added check of AlarmHoldOffTmr before calling
//                        any alarm functions.  AlarmHoldOffTmr prevents alarm functions from being called
//                        for 2sec after tripping.  This allows any current "tails" to decay before
//                        reenabling trips.  The timer is initialized in all protection functions when
//                        tripping is initiated
//                          - Intr.c, Intr_ext.h, Prot.c revised
//                      - Corrected bug checking whether setpoints are good
//                          - Setpnt.c revised
//   99      231019 BP  - Added the rest of the Secondary Injection code
//   101     231027 DAH - Enhanced startup time measurement accuracy and calibration, and added startup time
//                        compensation to instantaneous and short delay protection
//                          - Revised "first-pass" portion of the main loop to enhance the startup time
//                            accuracy, and to convert the time to number of passes for the instantaneous
//                            and short delay protection subroutines
//                          - Iod.c, Prot.c, Prot_ext.h revised
//                      - Fixed bug causing code to hang in offset calibration
//                          - Test.c revised
//   104     231102 BP  - Added TripFlagsReset() call before setting Alarm flags high
//                      - Fixed Neutral protection bug
//                      - Added TimeToTripFlg and ThermCapacityFlg
//                      - Added cold start compensation to Ground Fault protection
//   108     231108 DAH - Added call to Test_VarInit() into the initialization.  Test injection flags need
//                        to be cleared before interrupts are enabled, and this subroutine takes very little
//                        execution time.  Deleted initialization of TP.State.  It is initialized in
//                        Test_VarInit()
//                          - Test.c, Test_ext.h revised
//                      - Revised test port and USB secondary injection calibration subroutines to ensure no
//                        alarms or trips occur when calibrating
//                          - Test.c revised
//                      - Moved one-time tasks into the initialization section after sampling is started
//                      - Added Get_BrkConfig() code to the one-time tasks to keep initialization time
//                        before sampling starts to a minimum
//                      - Fixed minor bug in USB secondary injection gain calibration subroutine
//                          - Test.c revised
//                      - Minor revisions to eliminate compiler warnings
//                          - Diag.c, Ovrcom.c, Prot.c, DispComm.c, Meter.c, Modbus.c, Iod.c, Iod_ext.h,
//                            Iod_def.h revised
//                      - Minor code cleanup
//                          - DispComm.c, Iod_def.h revised
//   110     231114 DAH - Removed check of setpoints to see whether protection function is enabled for the
//                        following one-cycle protection subroutines: OverVoltage_Prot(),
//                        UnderVoltage_Prot(), VoltUnbalance_Prot(), CurUnbalance_Prot(), OverFreq_Prot(),
//                        UnderFreq_Prot(), PhaseLoss_Prot(), PhaseRotation_Prot(), OverRealPower_Prot(),
//                        OverReactivePower_Prot(), OverApparentPower_Prot(), PF_Prot(),
//                        RevActivePower_Prot(), RevReactivePower_Prot()
//                        The check is now performed in the actual protection function.  This cleans up the
//                        main loop by grouping all of the one-cycle subroutines in one tight if-statement.
//                          - Prot.c revised
//                      - Removed check of setpoints to see whether alarm function is enabled for the
//                        following one-cycle alarm subroutines: OverVoltage_Alarm(), UnderVoltage_Alarm(),
//                        VoltUnbalance_Alarm(), CurUnbalance_Alarm(), OverFreq_Alarm(), UnderFreq_Alarm(),
//                        PhaseLoss_Alarm(), PhaseRotation_Alarm(), OverRealPower_Alarm(), PF_Alarm(),
//                        OverReactivePower_Alarm(), OverApparentPower_Alarm(), Sneakers_Alarm(),
//                        RevActivePower_Alarm(), RevReactivePower_Alarm(), Ground_Fault_PreAlarm()
//                          - Prot.c revised
//                      - Corrected bug calling frequency protection and alarm protection.  Removed check of
//                        AlarmHoldOffTmr from protection calls and added to alarm calls
//                      - Corrected bug in OverFreq_Prot() and UnderFreq_Prot() that prevented code from
//                        executing
//                          - Prot.c revised
//   111     231117 BP  - Fixed bug in Long Delay IEC-A in Gen_Values in Prot.c
//                      - Added code to Setpnt.c in Load_SetpGr0_Gr1() for Breaker Frame
//                      - Fixed bug that was recently introduced with Breaker Rating being incorrect 
//                        in Setpoints0 and Setpoints1 after a power-up. For now, added a read of Critical  
//                        Breaker Config in Main.c after reading Breaker Config because we are presently only   
//                        writing the Critical Breaker Config Rating to FRAM and not the Breaker Config Rating.
//   116     231120 MAG - Pass TRUE to Modb_VarInit() for full initialization on power-up/reset
//                      - Mila's change in DispComm.c/AssembleFactoryBuffer() to fix USB lockup
//                      - Modified Maintenance Mode handling is DispComm.c, IOD.c, and Setpnt.c
//                      - Fixed various Modbus bugs (recorded in JIRA) in Modbus.c
//   117     231129 DAH - Increased Time Sync / AFE DRDY interrupt priority
//                          - Init.c revised
//                      - Deleted Read_ThermMem().  It is not used
//                          - Deleted call to Read_ThermMem() in 1-sec anniversary functions
//                          - Meter.c, Meter_ext.h revised
//                      - Corrected bug initializing I2C3 interrupts
//                          - Init.c revised
//   118     231129 DAH - Fixed firmware version
//                          - test.c revised
//   121     231204 DAH - Moved Modbus and Display power enable from initialization to one-sec anniversary.
//                        This is a temporary fix so that it does not cause issues with instantaneous and
//                        short delay cold-start testing.  Eventually need to turn on only if we have aux
//                        power
//   122     231204 BP  - Added first part of Coil Detection code that works with emulator in place of USB
//                        command.
//                      - Fixed bug that prevented the reading of Neutral current for Rogowski input
//                        by linking the Group 0 Neutral Sensor setpoint to the port pin that controls
//                        the measurement circuitry.
//                      - Made the same change for GF Sensor Group 1 setpoint and port pin.
//   125     231212 BP  - Added Ashley's fix for Breaker Config write
//                      - Fixed Reverse Active Power protection bug
//                      - Changed Flat GF Trip Threshold multliplier by 10% for settings of 0.4 to 1.0
//                      - Added voltage and current conditions for running UnderPF protection
//                      - Added Setpoints5.stp.ExtProtEnable as a condition for running Motor Protection
//                        functions
//   129     231213 MAG - Tweaked Maintenance Mode handling in various places to use similar code
//                      - Added Setpoints Verification to changes from the display processor
//                      - Split GOOSE setpoints Group 10 into Groups 10 and 12
//                      - Fixed numerous Modbus/Setpoints bugs
//                      - Consolidated handling of read-only configuration setpoints into GetSetpoints()
//                      - Added UART reset for Modbus communication setpoint changes
//   131     231214 RPS - Added update to Gp0 and Gp1 rating values in ProcWrFactoryConfig for Jira item 1753
//   133     231219 DAH - In Load_SetpGr2_LastGr(), removed code that enters an event and added code to
//                        return a status code.
//                          - Setpnt.c, Setpnt_ext.h, DispComm.c revised
//                      - In initialization, added code to check the status code from the call to
//                        Load_SetpGr2_LastGr() and enter an event if necessary
//                      - Revised temperature sensor code to not update readings if an I2C error occurs.
//                        This was causing temperature trips to occur
//                      - Added code to initialize THSensor.xx so we don't generate a false trip due to
//                        garbage values in the temperature register
//                          - Iod.c revised
//                      - Reduced SD and GF 50ms - 90ms trip time settings for 50msec times to ensure trip
//                        times are met at minimum currents (200A Frame, 0.4 dial-down)
//                          - Prot.c revised
//                      - Reenabled temperature protection in the one-second anniversary
//                      - Revised breaker configuration handling
//                          - Non-critical breaker config values are not kept in RAM.  They are read and
//                            written to FRAM as required
//                          - A new section was added to Frame FRAM to support a larger amount of critical
//                            breaker config values for the PXR35.  The process for retrieving the critical
//                            breaker config values was changed to be backward-compatible with PXR25 Frame
//                            modules and also support the new Frame modules
//                          - Initialization code in this module revised
//                          - DispComm.c, Prot.c, Prot_def.h, Prot_ext.h revised
//                      - Removed startup time compensation from Short Delay and Ground Fault protection.
//                        If the startup time is greater than 8.3msec (one-half cycle), it will not work
//                        properly.
//                          - SD_StartupSampleCnt and GF_StartupSampleCnt removed from initialization code
//                          - Prot.c revised
//                      - Added code to compute the one-cycle RMS currents if tripping in Instantaneous,
//                        Short Delay, and Ground Fault protection.  These routines are run in the sampling
//                        interrupt, and a trip event on a cold start may be logged before the one-cycle
//                        values are computed.  This ensures the most recent current values are captured.
//                        Also added ONE_CYC_VALS_DONE flag to note whether one-cycle values have been
//                        computed.  If not, the other snapshot values in the trip event log are set to 0.
//                          - Added code to set SystemFlags.ONE_CYC_VALS_DONE after one-cycle metering has
//                            been completed
//                          - Prot.c, Prot_ext.h, Prot_def.h, Event.c, Iod_def.h revised
//                      - Added proc-proc comms support to read internal time parameters
//                          - Moved maxlooptime to DispComm.c
//                          - Replaced temporary variable with Reset_to_PLL_Count in loop counter for PLL
//                            ready check
//                          - DispComm.c, DispComm_ext.h revised
//   135     231221 DAH - Added code to fill waveform pre-cycles with zeros if they have not been written.
//                        This may occur when closing into a fault from a cold start.
//                          - Intr.c, Intr_ext.h, Iod.c revised
//   137     240102 DAH - Added internal timing values to "DS" test port command
//                          - Test.c revised
//   138     240102 MAG - Fixed bugs in verification of Long Delay and Ground Fault Time setpoints
//                          - Setpnt.c revised
//   139     240104  BP - Fixed GF ZSI
//                      - Fixed Under PF bug
//   141     240115 BP  - Fixed Cancel Test bug for HW Sec Injection
//                      - Added T_forbid to Sec Injection and Coil Detection for when currents are too high
//   142     240119 DAH - Fixed bug in USB breaker configuration write Frame command, added cal constants to
//                        the configuration USB comms code, and cleaned up the code
//                      - Added code to the initialization to write critical breaker info into the PXR35
//                        section of Frame FRAM if it is configured as a PXR25 Frame.  This is necessary so
//                        that the startup time is in spec on subsequent power-ups
//                      - Major revisions to of factory command handling code to process calibration values
//                        correctly if stored in both FRAM and Flash
//                      - Added factory buffers 43 and 44 to support reading and writing the Standard and
//                        Max Instantaneous Trip Setting configuration values to PXR25 and PXR35 critical
//                        breaker config sections in Frame FRAM.  This command is needed to ensure that the
//                        PXR25 critical config section always matches the PXR35 critical config section.
//                        If the PXR35 critical config section failed, the PXR25 section would be used on
//                        the next power-up
//                          - DispComm.c, FRAM_Flash_def.h, Iod.c, Iod_def.h, Iod_ext.h, Meter_def.h revised
//                      - Added code to support 50Hz and 60Hz phase calibration constants
//                          - Meter_def.h, Init.c, Test.c revised
//                      - Added call to Flash_Read_ID() in initialization
//                      - Revised max loop time measurement in main loop to skip if SKIP_LOOPTIME_MEAS flag
//                        set.  Flag is set if time is changed.  A bug was found where if time is a bad
//                        reading is generated
//                          - Iod.c, Iod_def.h revised
//                      - Fixed minor bugs in USB calibration subroutines
//                          - Test.c, Test_def.h revised
//   143     240122 DAH - Fixed bug with ETU manufacturing location and date object (factory config
//                        proc-proc comms buffer 30)
//                          - DispComm.c, FRAM_Flash_def.h revised
//   144     240123 DAH - Fixed bugs with writing calibration constants (forgot to update checksum)
//                          - DispComm.c, Iod.c revised
//   145     240124 BP  - Added T_Forbids for Coil Detection and Sec Inj allowable current thresholds in Meter.c
//                      - Changed Over/UnderFreq Prot and Alarm routines to divide the pickup setpoint by 1000.
//                      - Fixed copy/paste bugs in UF Alarm routine
//                      - Added code so that voltage must be at least 10% of System Voltage to run  to run 
//                        UV, UF, and OF Alarm code. 
//                      - Fixed scaling on Voltage Unbalance by remove the multiply by 100.
//   146    240124  MAG - Fixed several setpoints validation bugs and LD Time default.  Stpnt.c revised.
//   147    240126  DAH - Fixed bug in GF protection.  Trip and Alarm flags were cleared if GF protection is
//                        disabled.  This can cause issues if GF protection is disabled when the unit is
//                        tripped due to GF
//                          - Intr.c revised
//   148    240131  BP  - Fixed UF Alarm bug
//                      - Changed scaling for Short Delay I2t at 0.05 setting
//                      - Added Aux Power measurement code with provisions to hold off protection when Aux or USB
//                        power is applied to prevent nuisance tripping.  
//                      - Sprinkled calls to TA_Volt_Monitoring() and AuxPower_Monitoring() in the main loop.
//   149    240131  DAH - Revised RTD buffer transmissions to free up time for other transmissions.  A bug
//                        was seen (JIRA item 1899) where under certain conditions, RTD buffer comms was
//                        locking out all other comms
//                          - DispComm.c, Intr.c, Dispcomm_def.h revised
//                      - Fixed minor bugs in ADC offset calibration subroutines
//                          - Test.c revised
//                      - Integrated setpoints into the demand computations and fixed bug
//                          - Demand.c, Setpnt.c revised
//                      - Added event generation to setpoint downloads and active set change
//                          - DispComm.c, Modbus.c, DispComm_def.h revised
//                      - In initialization code, moved call to Dmnd_VarInit() to after Group 0 setpoints
//                        have been read.  Some demand variables depend on the setpoints
//                      - In initialization code, added code to initialize the demand logging EID to the
//                        power-up EID
//   150    240202  DAH - Revised Calc_Demand() to handle cases where demand calculations and/or demand
//                        logging may be disabled
//                          - Demand.c revised
//                      - Recommented out code that clears GF trip and alarm flags when GF protection is
//                        disabled (was put back in for testing)
//                          - Intr.c revised
//                      - Fixed bugs in Phase Rotation Protection (JIRA item 1947)
//                          - Prot.c revised
//
//     *** DAH  NEED TO ADD SUPPORT FOR EXECUTE ACTION THAT RESETS THE ENERGY REGISTERS - SEE MINUTES FROM
//              MODBUS AND METERING DESIGN REVIEW ON 220405.  OPERATION SHOULD BE SIMILAR TO WHAT IS IN THE
//              PXR25.  NOT NECESSARY TO HAVE NON-RESETTABLE ENERGY REGISTERS, BUT DOUBLE-CHECK WHETHER ANYONE
//              WANTS THESE AND IF SO, HOW THEY SHOULD BE ACCESSED.
//
//      *** DAH  SEE *** DAH TEST  220207 FOR MODBUS TEST CODE
//
// SEE COMMENTS MARKED  *** DAH 210714 FOR DESKTOP TESTING WITH TEST INJECTION
//          (ATTEMPT TO DEBUG ISSUE WITH CORRUPTION OF FIRST CYCLE DURING TRIP WAVEFORM CAPTURE)
// MODBUS IS CONFIGURED FOR OPERATION OUT OF A RESET - SEE NOTE UNDER REV .49 ABOVE
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


//-------------------------------- Includes ------------------------------------------------------------------
#include "stdint.h"
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include "core_cm4.h"
#include "Init_def.h"
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"
#include "Meter_def.h"
#include "Prot_def.h"
#include "CAMCom_def.h"
#include "DispComm_def.h"
#include "Modbus_def.h"
#include <stdbool.h>
//#include "pxcan_def.h" - not used here for now

#include "Init_ext.h"
#include "Iod_ext.h"
#include "Test_ext.h"
#include "Intr_ext.h"
#include "Meter_ext.h"
#include "Prot_ext.h"
#include "CAMCom_ext.h"
#include "DispComm_ext.h"
#include "Events_ext.h"
#include "Demand_ext.h"
#include "Modbus_ext.h"
#include "RealTime_ext.h"
#include "Setpnt_ext.h"
#include "can_tasks_ext.h"
#include "can_driver_ext.h"
//#include "pxcan_ext.h" - not used here for now.
#include "Ovrcom_ext.h"



struct INTERNAL_TIME starttime, endtime;
uint32_t looptime;

extern uint8_t gtest;         // *** DAH TEST  210420
extern uint8_t gtestcnt;      // *** DAH TEST  210420

uint8_t DS_OneTimeTest;

uint32_t DistCounter;
uint8_t displayOFFTIM;

                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING START
    extern void Load_MB_Test_Vals(void);
                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING END



// -------------------------------------- Startup Timing ---------------------------------------------------
//
// Description:
//   For the worst-case analysis, assume the lowest firmware instantaneous overcurent.  This is
//   .4 (Ir dial-down) x 200A (min frame size) x 1.5 (minimum instantaneous setting) = 120A.
//   So if you close into this fault current, time starts at that point.  First, the TA caps must be charged
//   before the trip signal is issued.  This is a function of the hardware.  The minimum instantaneous time
//   is 3 half-cycles, so the TA caps have roughly this much time from the onset of the overcurrent to
//   charge - 50msec trip time: 3 half-cycles (25msec) + 4msec mech time + ~1.25 cycles clearing time
//   (21msec) = 50msec.
//   In parallel with this, the microprocessor must sample a minimum of two half-cycles before issuing the
//   trip signal - we don't want less than 2 half-cycles so that we don't get nuisance tripping.  First, the
//   power supply must come up to generate Vdd.  This is a function of the hardware.  When Vdd reaches the
//   operating threshold (about 2.93V), the reset chip (TPS3702) will release the PDR reset signal to the
//   micro.  When the 1.2V supply reaches the operating threshold (about 1.1V), a second TPS3702 reset chip
//   (TPS3702) will release the PA0 WKUP reset signal to the micro.
//
//   Note, the 1.2V supply comes up AFTER the 3.3V supply, so the 1.2V reset signal is released AFTER the
//   Vdd reset signal is released.
//
//   There are delays, however, in the reset chip and the microprocessor.  Code execution then begins.  The
//   micro initially operates at 16MHz (the internal HSI clock).  The startup code from STMIcro is executed,
//   then the initialization code begins to be executed.  Partway through this execution, the HSE clock is
//   available, so the micro switches over from the HSI clock to the 120MHz PLL, and the remaining
//   initialization code is executed.  Sampling then begins.
//   The PLL is based on the external 25MHz crystal.  After the two resets are released, the crystal starts
//   to come up.  There is a delay before the crystal is stable.  This occurs while the initialization code
//   is executing at 16MHz.  The delay was measured to be 1.868msec from the reset release to the HSE being
//   available.
//
// Startup timing:
//    + Min Overcurrent --> Valid TA Voltage:                                              TBD
//    |                             TOTAL:                                                 TBD
//    |
//    + Min Overcurrent (.4 x 200A x 1.5 = 120A) --> Vdd (hardware):                        TBD
//          + Vdd --> 1.2V reset released (TPS3702 chip):                                  992usec
//          +---+ Reset Released --> Micro begins execution (Reset Temporization Time):    1.6msec typ, 3msec max
//              +---+ Code exec to main (startup code from ST Micro - 879cyc/16MHz):       55usec
//                  +---+ main to sampling (initialization code + SysClkHandler()):        1.136msec
//                                  TOTAL:                                           TBD + 3.79msec typ, TBD + 5.19msec max
//    Testing 231024
//    |
//    + Min Overcurrent (.4 x 200A x 1.5 = 120A) --> Vdd (hardware) -->
//          1.2V reset released (TPS3702 chip) -->
//          Micro begins execution (Reset Temporization Time) -->
//          Code exec to main (startup code from ST Micro - 879cyc/16MHz) -->
//          Init_GPIO() completed:                                                         4.2msec
//    +---+ Init_GPIO() completed --> sampling (initialization code + SysClkHandler()):    2.4msec
//        +---+ sampling --> sync + seeding completed (6 samples):                    :    1.25msec
//                                  TOTAL:                                                 7.85msec typ



int main(void)
{
  uint8_t cfgstat;
  uint16_t temp;
  uint32_t temp2[2];

  // Note, the reset vector sets SysClkState equal to 0, then calls SysClkHandler().  This will:
  //   - disable global interrupts (done before calling SysClkHandler())
  //   - configure the system clock (SYSCLK)
  //   - initialize the peripherals whose configuration and enable/disable status are dependent on the state
  //     of SYSCLK
  //   - configure the system for HSI (16MHz) operation and get the HSE started
  // Reference startup_stm32f407xx.s

  // Configure the Flash memory
  Init_FlashConfig();

  // Set up the interrupt structure before initializing the peripherals
  Init_InterruptStruct();

  // Initialize all peripherals that are not dependent on SYSCLK.  In other words, these peripherals are
  //   configured the same way regardless of the state of SYSCLK.
  // These peripherals either do not generate interrupts or have their interrupts disabled until the proper
  // clock is set up.
  // Note, some of these peripherals are enabled in EnDisPeripherals() when the system clock is stable,
  //   others are always enabled, still others (the DACs) are only enabled when they are used.  The DAC is
  //   used to inject test currents and so will only be enabled when a trip test is occurring.
  Init_GPIO();
TESTPIN_D1_HIGH;                      // *** DAH TEST
TESTPIN_A3_HIGH;

  Init_ADC1();                          // Measured execution time for these subroutines on 180625
  Init_ADC2();                          //   (rev 0.25 code): 35.1usec total
  Init_ADC3(TRUE);                      // Init_ADC3() is set up to read the Startup Time
  Init_DAC();
  Init_SPI3();
  Init_TIM1();                          // Note, Init_TIM2() was moved to after Setpoints group 2 are read
  Init_TIM3();                          //   to initialize the timer based on the Modbus baud rate
  Init_TIM4();
  Init_TIM5();                          // *** DAH NEED TO MEASURE TIME AGAIN
  Init_TIM10();

  // Initialize peripherals that only operate when SYSCLK is 120MHz (PCLK1 and PCLK2 are 30MHz).  These
  //   peripherals are configured assuming PCLK is 30MHz.  They are disabled if SYSCLK is 16MHz, and are
  //   only enabled when SYSCLK is 120MHz.  These peripherals are disabled by the initialization subroutines
  //   since at this point, SYSCLK is 16MHz.
  //   These peripherals generate interrupts.  The initialization subroutine enables the interrupts, but no
  //   interrupts will occur since the peripherals are disabled.  The peripherals will be enabled in
  //   SysClkHandler() when the system switches from 16MHz to 120MHz.
  //
  Init_I2C2();                          // Measured execution time for these subroutines on 180625
  Init_I2C3();                          //   (rev 0.25 code): 54.3usec total
  Init_UART1();                         // *** DAH  NEED TO REMEASURE SINCE ADDED INIT_UART3()
  Init_UART2();                         // Note, Init_UART6() was moved to after Setpoints group 2 are read
  Init_UART3();                         //   to initialize the UART based on the Modbus baud rate
  Init_SPI1(DEV_FRAM_FLASH8);           // *** DAH initialized here for now, may be able to delete if only used with serial Flash part
  Init_DMAController1();
  Init_DMAController2();
  Init_Can();                           // Execution time = 16.3 usec (XIP)
                        
  IO_VarInit();                         // This must precede Event_VarInit()!!
                                        // Execution time = 56.1usec (rev 0.25 code)
  Test_VarInit();
  Intr_VarInit();                       // Execution time = 18usec (rev 0.25 code)
  Setp_VarInit();
  Ovr_VarInit();
  InitRelays();

  // At this point, approximately 273usec have elapsed since the end of Init_GPIO().  The HSE clock should
  //   be ready.  Analysis and testing conducted on 181026 showed that it is 1.66msec from reset release to
  //   the end of Init_GPIO(), so it is 1.93msec from reset release to this point in the code.  Testing with
  //   an oscilloscope showed that it is 1.87msec from reset release to the HSE clock up and running.
  // Therefore, we can switch to the 120MHz PLL with minimal delay.  Try 100 times as a precaution.  Testing
  //   on 181029 showed that it takes 35 loops until the switch is made, this is the delay for the PLL to be
  //   ready
  Reset_to_PLL_Count = 0;
  while ( (SysClk_120MHz != TRUE) && (Reset_to_PLL_Count++ < 100) )
  {
    SysClkHandler();
  }

  // The remaining code now runs at 120MHz - the SPI operations will run at 15MHz instead of 8MHz
  CAM_VarInit(&CAM1);                   // Measured execution time for these subroutines on 181026
  CAM_VarInit(&CAM2);                   //   (rev 0.28 code): 50.4usec for CAM_VarInit() (two calls),
  DispComm_VarInit();                   //   DispComm_VarInit(), Event_VarInit(), InitFlashChip(), and
  Event_VarInit();                      //   ReadSwitches()
                                        // Event_VarInit() must be called after IO_VarInit()!!
  RT_VarInit();
  Modb_VarInit(TRUE);

  InitFlashChip();                      // Execution time = 71usec (rev 0.25 code)

  // Meter_VarInit() must follow InitFlashChip() because it reads the Flash chip!!
  Meter_VarInit();                      // Execution time = 286usec (rev 0.28 code, no error)  *** DAH  RECALCULATE THIS TIME

  Init_TIM8();                          // *** DAH - Need to measure the execution time

  ReadSwitches(TRUE);                   // Read switches to initialize the status

  // Measured the execution time on 220928 (rev 0.60 code): 486usec to initialize and start the AFE
  //   (from this point in the code to AFE_START_HIGH)
  // Initialize the Analog Front End (AFE)
  //   The AFE reset line is already active (low).  The port line is set low when the GPIO is initialized in
  //   Init_GPIO()
  AFE_RESETN_INACTIVE;

  // Retrieve the breaker configuration information.  First retrieve it assuming the Frame module supports a
  //   PXR35.  If this is successful, we are done.  If it is unsuccessful (False is returned), we will try
  //   assuming the Frame module only supports a PXR25.
  // Note, if it is a PXR25 frame, the execution time to retrieve the parameters is extended by about 1.7msec
  //   We have no choice, because we need the Poles, MCR, and OVR Withstand values and these are only in the
  //   large (403-byte) section of Frame FRAM
  cfgstat = 0;                                      // Initialize status to ok
  if (Get_Critical_BrkConfig_PXR35() == FALSE)
  {
    if ( Get_Critical_BrkConfig_PXR25() )           // We got the PXR25 critical parameters.  Now retrieve
    {                                               //   the remaining 3 parameters for the PXR35
      if (Get_BrkConfig())                          // If successful, store the 3 parameters
      {
        Break_Config.config.Poles = ( SPI2_buf[234] + (((uint16_t)SPI2_buf[235]) << 8) );
        Break_Config.config.MCR = ( SPI2_buf[258] + (((uint16_t)SPI2_buf[259]) << 8) );
        Break_Config.config.OvrWithstand = ( SPI2_buf[256] + (((uint16_t)SPI2_buf[257]) << 8) );
        cfgstat = 1;                                // Set status code to 1
      }
      else                                          // If unsuccessful, load last three values with defaults
      {
        Break_Config.config.Poles = DFLT_BKRCFG_POLES;
        Break_Config.config.MCR = DFLT_BKRCFG_MCR;
        Break_Config.config.OvrWithstand = DFLT_BKRCFG_OVR_WITHSTAND;
        cfgstat = 2;                                // Set status code to 2
        TP_LEDCode = 2;                // flash the Status LED red
      }
    }
    else                                            // If unsuccessful retrieving as a PXR25 frame, the
    {                                               //   frame is bad, so load in defaults
      Break_Config.config.Rating = DFLT_BKRCFG_RATING;
      Break_Config.config.BreakerFrame = DFLT_BKRCFG_FRAME;
      Break_Config.config.Standard = DFLT_BKRCFG_STANDARD;
      Break_Config.config.MaxInstTripSetting = DFLT_BKRCFG_MAX_INST_TRIP;
      Break_Config.config.Poles = DFLT_BKRCFG_POLES;
      Break_Config.config.MCR = DFLT_BKRCFG_MCR;
      Break_Config.config.OvrWithstand = DFLT_BKRCFG_OVR_WITHSTAND;
      cfgstat = 2;                                  // Set status code to 2
      TP_LEDCode = 2;                // flash the Status LED red
    }
  }
  
  // Retrieve the Gr0 and Gr1 setpoints from the Frame FRAM - these are required to initialize the AFE and
  //   for protection.  To keep startup time to a minimum, the remaining setpoints will be retrieved after we
  //   have begun sampling
  Load_SetpGr0_Gr1();

  // Load Flash test values into buffer
  Flash_Read_ID();                      // Execution time = 15.35usec (Rev 142 code) 

  // Note, this subroutine calls Gen_Values() - it uses setpoints, so it must be called after setpoints have
  //   been retrieved!
  Prot_VarInit();                       // Execution time = __ usec (rev 0.45 code)
  // This must be called after Group 0 setpoints have been read!
  Dmnd_VarInit();                       // Execution time = 108usec (rev 0.25 code)


  // Contacted ADI on 150825 after seeing errors in the setup when not delaying between the reset going
  //   inactive and calling the routine to initialize the chip.  According to ADI, must wait at least
  //   120usec after reset is brought inactive before writing to the chip.  Data sheet mentions 225usec
  //   from reset to first DRDY
  // We have already waited a minimum of 204usec retrieving the setpoints (see above).  Need to wait about
  //   another 50usec (put some margin in)
  temp = 1000;                          // Measured time on 220928: 50.1usec
  while (temp > 0)                      //   Note, this is running with the 120MHz clock.
  {
    --temp;
  }

  if (Setpoints1.stp.ThermMem == 1)
  {
    UpdateThermalMemory();             // Read the Therm Mem cap voltage and read the stored LD Bucket% from FRAM
  }
  else                                 // LD_Tally and LD_Bucket are already zeroed in Prot_VarInit()
  {
    temp2[0] = 0;                      // Write a 0 to FRAM for LD Bucket value
    temp2[1] = (uint16_t)(~temp2[0]);

    FRAM_Write(DEV_FRAM2, TM_CAPACITY, 2, (uint16_t *)(&temp2[0]));
    FRAM_Write(DEV_FRAM2, TM_CAPACITY_COMP, 2, (uint16_t *)(&temp2[1]));
  }


  // Measured total time from AFE_RESETN_INACTIVE to AFE_Init() on 220928: 253.1usec min (typical case:
  //   PXR35 frame module and first copy of setpoints are good)
  AFE_Init();                           // Next, set up the registers in the chip

  ReadTHSensorConfig();

  AFE_START_LOW;                        // Finally, send a pulse to start the chip
  temp = 20;                            // Delay at least 250nsec.  Measured on 151029: 1.09usec
  while (temp > 0)                      //   Note, this is running with the 120MHz clock.
  {
    --temp;
  }
  AFE_START_HIGH;
  displayOFFTIM = 0;//***ALG for functionality testing
  // Turn on the display *** DAH NEED TO ADD CODE TO CHECK WHETHER WE HAVE AUX POWER - FOR NOW, JUST TURN ON
//  DISPLAY_ENABLE;                       // *** DAH TURNED ON FOR ENGINEERING DEMO
//  MODBUS_PWR_ENABLE;                    // *** DAH  NEED TO PLACE IN SUBROUTINE IN MAIN LOOP THAT ENABLES POWER IF AUX VOLTAGE
                                        //          IS GREATER THAN 16, AND DISABLES POWER IF AUX VOLTAGE IS LESS THAN 15

  ADC3->SQR3 = ADC3_STARTUP_CH;         // Set the ADC3 channel for the startup time cap voltage to get
                                        //   ready to convert
//  SystemFlags |= FIRST_PASS;            // Set flag to indicate first pass through the main loop - not needed any more   *** DAH  DELETE THIS AFTER WE TEST OK  (11/7/23)

  // Measured time on ???: ??usec for Check_IntRTC(), IntRTC_Read(), and RTC_to_SysTickTime()
  if ( !(Check_IntRTC()) )              // Check the RTC for valid configuration.  IO_VarInit() must be
  {                                     //   called before this code because SystemFlags is initialized
    SystemFlags |= RTC_ERR;             //   there.  If RTC has not been configured, set error flag, set
    SysTickTime.cnt_sec = 0;            //   internal time to default, initialize the RTC, and insert
    SysTickTime.cnt_10msec = 0;         //   RTC Bad Power Up event
    NewEventFIFO[NewEventInNdx].Code = PWRUP_RTC_BAD;             // Insert event into queue
    Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
    NewEventFIFO[NewEventInNdx++].EID = EventMasterEID;
    NewEventInNdx &= 0x0F;                                        // Note, FIFO size must be 16!!
  }
  else                                  // RTC appears to be ok.  Read the RTC time.  If it is ok, set
  {                                     //   internal time to RTC time, leave error flag clear, and insert
    // IntRTC_Read() takes approximately ?usec to complete (measured on ???).  Remember, we are running
    //   on the 16MHz SYSCLK.  This is small enough that it is not necessary to adjust the time loaded into
    //   the internal timer.
    RTC->ISR &= 0xFFFFFFDF;             // Must clear RSF flag after a wakeup before reading it
    if (IntRTC_Read() )
    {
      RTC_to_SysTickTime(&SysTickTime);
         // Insert event into queue.  Note, cannot use call to InsertNewEvent() because it disables, then
         //   enables interrupts and we don't want interrupts enabled yet
      NewEventFIFO[NewEventInNdx].Code = PWRUP_RTC_GOOD;
      Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
      NewEventFIFO[NewEventInNdx++].EID = EventMasterEID;
      NewEventInNdx &= 0x0F;                                        // Note, FIFO size must be 16!!
    }
    else                                // If invalid time, set error flag, set internal time to default,
    {                                   //   and insert RTC Bad Power Up event
      SystemFlags |= RTC_ERR;
      SysTickTime.cnt_sec = 0;
      SysTickTime.cnt_10msec = 0;
      NewEventFIFO[NewEventInNdx].Code = PWRUP_RTC_BAD;             // Insert event into queue
      Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
      NewEventFIFO[NewEventInNdx++].EID = EventMasterEID;
      NewEventInNdx &= 0x0F;                                        // Note, FIFO size must be 16!!
    }
  }

  // Set the Demand Logging EID to the power up EID and increment the master EID
  EngyDmnd[1].EID = EventMasterEID++;

  maxlooptime = 0;                      // Used to measure the maximum main loop time

  // Make sure the switch to the PLL has been made.  The switch should have been done already
  //   Try 50 times, then give up in case there is a problem (will run at 16MHz so test port works)  *** DAH  BETTER TO SWITCH TO INTERNAL OSCILLATOR AND CONFIGURE THE PLL FOR 120MHZ.
  temp = 0;                                                                                        //   ADD AN ALARM HERE ALSO
  while ( (SysClk_120MHz != TRUE) && (temp++ < 50) )
  {
    SysClkHandler();
  }

  // Enable interrupts
  __enable_irq();

  // At this point, sampling has started.  Now do a couple of one-time tasks before entering the main loop
  //   Execution time measured on 231108: 2.55msec
  //   The first six samples are used to sync the AFE and ADC samples and to seed the digital integrator, so
  //   6 x 208.3usec = 1.25msec.  This means we will accumulate about 1.3msec of samples (about 6 samples)
  //   before entering the main loop
  // This should not be a problem
  TESTPIN_D1_LOW;                      // *** DAH TEST FOR COLD START TESTING
  if (SysClk_120MHz == TRUE)
  {
    // Measure the startup time at the point where we are running at 120MHz.  This is when sampling begins
    ADC3->CR2 |= ADC_CR2_SWSTART;         // Start the conversion
    while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
    {
    }
    StartUpADC = ((uint16_t)ADC3->DR & 0x0FFF);
    // Computed value has 1.25msec added to compensate for the 6 samples that are not used while seeding
    //   and AFE/ADC synchronization occurs
    StartupTime.Time =  (float)StartUpADC * StartupTime.Scale + 1.25E-3;
    // If startup time out of range, set to one-half cycle (40 samples)           *** DAH  MUST CHANGE FOR 50HZ OPERATION!!
    //   Note: Do not change this range without examining the Instantaneous_Prot() and ShortDelay_Prot()
    //         in Prot.c!!
    StartupTime.Time = ( (StartupTime.Time > 10E-3) ? (8.34E-3) : (StartupTime.Time) );
    StartupTime.Time = ( (StartupTime.Time < 5.5E-3) ? (8.34E-3) : (StartupTime.Time) );
    Inst_StartupSampleCnt = (uint8_t)(StartupTime.Time * 4800);
    ST_DISCHARGE_ON;
//TESTPIN_D1_LOW;                      // *** DAH TEST

    // Retrieve the remaining setpoints from the Frame FRAM.  Save return status in temp
    temp = Load_SetpGr2_LastGr();

    // Enter an event if there was any problem (Frame mismatch or Frame error) retrieving the configuration
    //   or the setpoints.  Configuration status is in cfgstat; temp has the setpoints status
    if ( (temp == 2) || (cfgstat == 2) )                // Status = 2 in either section: Frame Error
    {
      NewEventFIFO[NewEventInNdx].Code = STP_ERROR;
      Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
      NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
      NewEventInNdx &= 0x0F;                                        // Note, FIFO size must be 16!!
    }
    else if ( (temp == 1) || (cfgstat == 1) )           // Status = 1 in either section: Frame Mismatch
    {
      NewEventFIFO[NewEventInNdx].Code = STP_FRAME_MISMATCH;
      Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
      NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
      NewEventInNdx &= 0x0F;              // Note, FIFO size must be 16!!
    }

    // Initialize Timer 2 based on Modbus baud rate from Group 2 setpoints
    Init_TIM2();
    // Initialize the Modbus UART with baud, parity, stop bits from Group 2 setpoints
    Init_UART6(TRUE);

    // Initialize the energy registers from FRAM.  This only has to be done before the 200msec anniversary
    FRAM_ReadEnergy(DEV_FRAM2);

   
    SPI1Flash.Req |= S1F_CHK_CAL;         // Set the flag to check the Flash cal constants

    // If the PXR35 critical configuration in the Frame FRAM was retrieved from a PXR25 Frame, save the
    //   values so next time we power up, we can quickly retrieve them.  We won't do this if the Frame is
    //   a PXR35 (cfgstat == 0, everything ok), or if the Frame is not configured (cfgstat == 2, frame bad).
    //   If Frame is bad, we want to alarm every time we power up
    if (cfgstat == 1)
    {
      Save_Critical_BrkConfig();        // Measured execution time on 240109: 103usec
    }
  }
  else              // *** DAH  WHAT DO WE DO IF WE AREN'T RUNNING AT 120MHZ?
  {
    OscFault = TRUE;
    TP_LEDCode = 2;                // flash the Status LED red
  }


gtestcnt = 0;  // *** DAH TEST  210420
gtest = 0;     // *** DAH TEST  210420

DS_OneTimeTest = 0;

DistCounter = 0;

temp = 5;

Manufacture_Mode = FALSE;

AuxPower_Monitoring();             // has nuisance trip code

// Capture time for max loop time measurement.  We want to want to measure the first time through, because
//   this could be the worst-case condition
__disable_irq();                    // Disable interrupts when checking the system clock
Get_InternalTime(&starttime);       // *** DAH ADDED TO MEASURE MAIN LOOP TIME
__enable_irq();


  //-------------------------------- Start of Main Loop ----------------------------------------------------
  //

  while (1)
  {
//     TESTPIN_A3_TOGGLE;

    // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple times
    //   in the main loop  *** DAH check timing
    ManageSPI1Flags();
    TA_Volt_Monitoring();
    AuxPower_Monitoring();     
    
    //------------------------------ One-Cycle Anniversary Subroutines -------------------------------------
    //
    if (OneCycAnniv)                        // *** DAH DO WE WANT TO RUN IF NOT 120MHZ?
    {
      Calc_Prot_Current();
      Calc_Prot_AFE_Voltage();
      Calc_ADC_OneCyc_Voltage();
      Calc_Prot_Power();

      // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple
      //   times in the main loop  *** DAH check timing
      ManageSPI1Flags();

      Calc_CF();
      Calc_Freq(&FreqLoad, VolAFE200msFltr.Van);
      Calc_Freq(&FreqLine, VolADC200ms.Van);

      SystemFlags |= ONE_CYC_VALS_DONE;                     // Set flag indicating we have one-cycle values
      
      TA_Volt_Monitoring();
      AuxPower_Monitoring(); 

      // One-cycle protection routines
      if ((Prot_Enabled) && (!Manufacture_Mode))            // Run protection if it is enabled
      {
        if (Setpoints1.stp.Ld_Slp < 4)
        {
           LongDelay_Prot();
        }
        else
        {
           Long_IEE_IEC_Prot();
        }

        Sneakers_Prot();                        // *** DAH MEASURE EXECUTION TIMES MAY NEED TO ADD CALL TO ManageSPI1Flags()
        OverVoltage_Prot();
        UnderVoltage_Prot();
        VoltUnbalance_Prot();
        CurUnbalance_Prot();
        OverFreq_Prot();
        UnderFreq_Prot();
        PhaseLoss_Prot();
        PhaseRotation_Prot();
        OverRealPower_Prot();
        OverReactivePower_Prot();
        OverApparentPower_Prot();
        PF_Prot();
        RevActivePower_Prot();
        RevReactivePower_Prot();
      }

      // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple
      //   times in the main loop  *** DAH check timing
      ManageSPI1Flags();

      // One-cycle alarm routines
      OverVoltage_Alarm();                         // *** DAH MEASURE EXECUTION TIMES MAY NEED TO ADD CALL TO ManageSPI1Flags()
      UnderVoltage_Alarm();
      VoltUnbalance_Alarm();
      CurUnbalance_Alarm();
      OverFreq_Alarm();
      UnderFreq_Alarm();
      PhaseLoss_Alarm();
      PhaseRotation_Alarm();
      OverRealPower_Alarm();
      OverReactivePower_Alarm();
      OverApparentPower_Alarm();
      PF_Alarm();
      RevActivePower_Alarm();
      RevReactivePower_Alarm();
      Ground_Fault_PreAlarm();
      Sneakers_Alarm();

      // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple
      //   times in the main loop  *** DAH check timing
      ManageSPI1Flags();

      // More one-cycle Alarm routines
      if (AlarmHoldOffTmr == 0)
      {
        Highload_Alarm();                        // *** DAH MEASURE EXECUTION TIMES MAY NEED TO ADD CALL TO ManageSPI1Flags()
        Thermal_Mem_Alarm();
        Wrong_Sensor_Alarm_Curr_Condition();
        WrongSensor_Alarm();
        THDCurrent_Alarm();
        THDVoltage_Alarm();
        Neutral_Alarm();                  
        TA_Alarm();               
        BreakerHealth_Alarm();
      }

      // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple
      //   times in the main loop  *** DAH check timing
      ManageSPI1Flags();

      // Extended Capture Snapshot Values OneCycle for 6s
        // *** DAH  NEED TO STORE INITIAL TIME STAMP + EVENT ID (SAME EID AS FOR WAVEFORMS)
        //  ALSO MAY NEED TO TRANSFER THIS TO FLASH (NOT SURE YET)
      if ((ExtCap_ReqFlag) || (ExtCap_AckFlag))
      {
        ExtendedCapture(OneCycle);
      }

      // Run subroutine on one-cycle anniversary after protection and alarm functions have completed
      DisturbanceCapture();

      OneCycAnniv = FALSE;
      SystemFlags |= VALONECYC;
    }
    //
    //--------------------------- End Of One-Cycle Anniversary Subroutines ---------------------------------


    //------------------------------- 200msec Anniversary Subroutines --------------------------------------
    //
    if (msec200Anniv)
    {
      Calc_Meter_Current();
      Calc_Meter_AFE_Voltage();
      Calc_ADC_200ms_Voltage();
      Calc_Meter_Power();               // Must follow Calc_Meter_Current() and Calc_Meter_AFE_Voltage()

      // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple
      //   times in the main loop  *** DAH check timing
      ManageSPI1Flags();

      Calc_Energy();                    // This must follow Calc_Meter_Power()
      Calc_Demand();                    // Must follow Calc_Meter_Current() and Calc_Meter_AFE_Voltage()
      Calc_AppPF();                     // This must follow Calc_Meter_Power()
      Calc_DispPF_THD();
      Calc_SeqComp_PhAng();
      Calc_5minAverages();
      
      // Extended Capture Snaphsot Values 200ms for 60s
      if ((ExtCap_ReqFlag) || (ExtCap_AckFlag))
      {
        ExtendedCapture(TwoHundred);
      }
      
      msec200Anniv = FALSE;
      SystemFlags |= VAL200MSEC;
      // Test code to support EAG77 command and send out 3 consecutive GOOSE messages every second
      if (gtest && (++gtestcnt >= 5))  // *** DAH TEST 210420
      {
        gtestcnt = 0;
        __disable_irq();
//        TESTPIN_A3_HIGH;   // *** DAH TEST  210421  Publisher
//        DPComm61850.Req[DP61850_TYPE_TRIP] = 1;
//        DPComm61850.Req[DP61850_TYPE_ZSI] = 1;
//        DPComm61850.Req[DP61850_TYPE_XFER] = 1;
        
        DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
        DPComm61850.Req[DP61850_TYPE_VALS] = 1;
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
        TESTPIN_A3_HIGH;
#endif
        __enable_irq();
      }
#ifdef ENABLE_GOOSE_COMM_AUTOSEND
      else if((!gtest) && (++gtestcnt >= 20))
      {
        gtest = TRUE;
      }
#endif

      if (temp > 0)             // Wait one second, then send time to display processor
      {
        if (--temp == 0)
        {
          // Set flag to send time up to display micro - note this should be tied to DISPLAY_ENABLE   *** DAH - THIS MUST BE TIED TO WHEN THE DISPLAY PROCESSOR IS TURNED ON - ADD DELAY TO GIVE DISPLAY PROCESSOR TIME TO COME UP
          DPComm.Flags |= TX_TIME;
        }
      }
      if (displayOFFTIM)        // Display CPU disabled
      {
        displayOFFTIM--;
        if (!displayOFFTIM)
        {
          DISPLAY_ENABLE;
        }
      }
    }     // end of 200ms anniversary
    //
    //---------------------------- End Of 200msec Anniversary Subroutines ----------------------------------


    //--------------------------------- 1sec Anniversary Subroutines ---------------------------------------
    //
    if (OneSecAnniv)
    {
      DISPLAY_ENABLE;                       // *** DAH TURNED ON FOR ENGINEERING DEMO
      MODBUS_PWR_ENABLE;                    // *** DAH  NEED TO PLACE IN SUBROUTINE IN MAIN LOOP THAT ENABLES POWER IF AUX VOLTAGE

      ReadTHSensor();

      Temp_Prot();
      Calc_BatteryVolt();
      //BatteryVolt_Alarm();    // *** BP comment out for now

      // Increment CAN TME counters
      increment_TME_Counters();
      OneSecAnniv = FALSE;
    }
    //
    //------------------------------ End Of 1sec Anniversary Subroutines -----------------------------------


    // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple times
    //   in the main loop  *** DAH check timing
    ManageSPI1Flags();
    TA_Volt_Monitoring();
    AuxPower_Monitoring(); 

    //--------------------------------- 5min Anniversary Subroutines ---------------------------------------
    //
    // Don't allow diagnostics until after we measure the startup time so that the cap isn't discharged in
    //   StartupTimeCal().  The 5-minute anniversary counter must be initialized to a value large enough to
    //   ensure an anniversary does not occur until the startup time has been measured.
    if (min5Anniv)
    {
      RTC_State = 1;                        // Set state to 1 to initiate RTC update
      StartupTime.DoCalFlag = TRUE;
      Check_Setpoints(SetpChkGrp);
      SetpChkGrp = ((SetpChkGrp >= (NUM_STP_GROUPS - 1)) ? 0 : (SetpChkGrp + 1));
//      DPComm61850.Req[DP61850_TYPE_ZSI] = TRUE;            // *** DAH  ADDED FOR TEST  201207
      min5Anniv = FALSE;
    }
    //
    //------------------------------ End Of 5min Anniversary Subroutines -----------------------------------

    // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple times
    //   in the main loop  *** DAH check timing
    ManageSPI1Flags();
    TA_Volt_Monitoring();
    AuxPower_Monitoring(); 

    if ((SystemFlags & RTC_ERR) == RTC_ERR)         // Internal RTC: either initialize it if it didn't come
    {                                               //   up right, or update it if it is running
      Init_IntRTC();
    }
    else
    {
      IntRTC_Update();
    }

    // These functions are executed each time through the main loop

    // Call subroutines to update the binary status and Pri/Sec/Cause status.  If true is returned, the
    //   status has changed, so clear the timer to transmit status immediately
    // Note, two separate "if" statements are used, as opposed to a single "if" statement with an "or" of
    //   the two functions, because we want both subroutines to be called.  If an "or" statement is used,
    //   the second subroutine will not be called if the first one returns True
    if (Update_Std_Status(&TU_BinStatus))
    {
      DPComm.RTD_XmitTimer[0] = 0;
    }
    if (Update_PSC(&StatusCode))
    {
      DPComm.RTD_XmitTimer[0] = 0;
    }
    Calc_Harmonics();
    Calc_KFactor();
    if(Manufacture_Mode == FALSE || ExAct.LED_image == NO_MANUF_TEST)
    {
      Service_Status_Led();               // Service status LED
      Service_NonCOT_Leds();              // Service all other LEDs except for Cause-of-Trip LEDs
    }

    if(Manufacture_Mode == FALSE || ExAct.Relay_image == NO_MANUF_TEST)
    {
      RelayManagement();            // Measured execution time on 231129: 1.3msec max
    }
    TP_Top();
    ExAct_Top();
    CAM_Tx(&CAM1, DMA2_Stream7);
//    CAM_Tx(&CAM2, DMA2_Stream6);        // *** DAH 220128 disabled for modbus operation
    CAM_Rx(&CAM1, DMA2_Stream5);
//    CAM_Rx(&CAM2, DMA2_Stream1);
                      // *** DAH TEST  220207 ADDED FOR MODBUS AND DISPLAY COMMS DEBUGGING START
//    Load_MB_Test_Vals();            // Takes about 335usec with present code

                      // *** DAH TEST  220207 ADDED FOR MODBUS AND DISPLAY COMMS DEBUGGING END
//    __disable_irq();                    // Disable interrupts when checking timing

    // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple times
    //   in the main loop  *** DAH check timing
    ManageSPI1Flags();
    TA_Volt_Monitoring();
    AuxPower_Monitoring(); 

    // XIP -- Call CAN routines here
    //TESTPIN_D1_HIGH;  // *** XIP testing Can tasks timing
    CanTasks(); // ~400 ns - need to measure again as command table grows
    //TESTPIN_D1_LOW;  // *** XIP testing Can tasks timing
    ModB_SlaveComm();
    DispComm_Rx();                      // Rx routine should be called before the Tx routine
    DispComm_Tx();
    StartupTimeCal();                    //*** DAH TEST  COMMENT OUT FOR COIL TEMPERATURE  TESTING - SEE REV 29 COMMENT.  WILL NEED TO ADD AND ADC3 MANAGER
    SystemFlags &= (~(VALONECYC + VAL200MSEC));
    EventManager();

    Firmware_Simulated_Test();             
    Hardware_SecInj_Test(); 
    Coil_Detection();
                    
    // This services the 1.5msec timer flags and so must be updated ~every 3msec or so.  Call multiple times
    //   in the main loop  *** DAH check timing
    ManageSPI1Flags();
    TA_Volt_Monitoring();
    AuxPower_Monitoring(); 

    MM_Status_Update();

    // resetting of min/max values and functions
    ResetMinMax();

    // relay management of assignment
//    if (Trip_Flags0.all || Trip_Flags1.all || Alarm_Flags0.all || Alarm_Flags1.all || Alarm_Flags2.all || RelayFlagStp)
//    {
//      RelayManagement();      //*** DAH to measure time
//      RelayFlagStp = FALSE;
//    }

    // Capture time for max loop time measurement
    __disable_irq();                    // Disable interrupts when checking the system clock
    Get_InternalTime(&endtime);
    __enable_irq();
    if (!(SystemFlags & SKIP_LOOPTIME_MEAS))        // If skip flag is clear, measure the loop time
    {
      looptime = ((endtime.Time_secs == starttime.Time_secs) ?
          (endtime.Time_nsec - starttime.Time_nsec) :
                ( ((endtime.Time_secs - starttime.Time_secs) * 1000000000)
                  + endtime.Time_nsec - starttime.Time_nsec));
      if (looptime > maxlooptime)         // Max loop time = maxlooptime in nanoseconds
      {
        maxlooptime = looptime;
      }
    }
    // Set start time to end time for next measurement
    starttime.Time_secs = endtime.Time_secs;
    starttime.Time_nsec = endtime.Time_nsec;
                    // MEASURED MAIN LOOP TIME ON 180221: ~355USEC MAX WITHOUT HARMONICS CALCULATIONS, DISPLAY NOT CONNECTED, NO CAM COMMS GOING ON
                    // MEASURED MAIN LOOP TIME ON 180625: ~4.6msec MAX WITH HARMONICS CALCULATIONS, DISPLAY NOT CONNECTED, NO CAM COMMS GOING ON
                    // MEASURED MAIN LOOP TIME ON 190718: ~4.9msec MAX WITH HARMONICS CALCULATIONS, DISPLAY NOT CONNECTED, NO CAM COMMS GOING ON
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        main()
//------------------------------------------------------------------------------------------------------------


