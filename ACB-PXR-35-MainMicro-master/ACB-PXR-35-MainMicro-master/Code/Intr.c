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
//  MODULE NAME:        Intr.c
//
//  MECHANICS:          Program module containing the interrupt service routines
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.04   150928  DAH - Added Intr_VarInit()
//                      - Continued code development in DMA1_Stream0_IRQHandler().  Added sample processing
//                        and metering.
//   0.05   151021  DAH - Revised the sampling interrupt, DMA1_Stream0_IRQHandler(), to eliminate computing
//                        the half-cycle RMS current (half-cycle sums of squares is still computed).
//                          - CurHalfCycImax replaced by CurHalfCycSOSmax
//                          - CurHalfCyc replaced with CurHalfCycSOS_Sum
//                          - Deleted code to compute the half-cycle RMS currents
//                      - Revised the sampling interrupt, DMA1_Stream0_IRQHandler(), to capture the
//                        one-cycle sums of squares.  These are needed for i2T short-delay protection.
//                          - CurOneCycSOSmax and OneCycSamples[][] added
//                          - OneCycCtr renamed to OneCycInd
//                          - Added NewSample to capture the new sample corresponding to the phase that is
//                            the maximum one-cycle RMS value
//                          - Added states 4 and 5 to process one-cycle samples and sums of squares
//                      - Added short delay protection
//                          - Revised the sampling interrupt, DMA1_Stream0_IRQHandler(), to call
//                            ShortDelay_Prot()
//                      - Revised EXTI9_5_IRQHandler() to support test command to turn off AFE
//                        communications.  This was added for power consumption testing and will eventually
//                        be deleted
//   0.06   151111  DAH - Added support for user-initiated waveform captures
//                          - Added UserWF_Req, UserWF_Ack
//                          - Moved UserSamples[][] from the local to the global section
//                          - Revised DMA1_Stream0_IRQHandler() to start capturing samples when requested
//                            and stop capturing when the buffer is filled
//                  BP  - Added DMA2_Stream0_IRQHandler() for the ADC1/ADC2 data.
//                      - Added isr's for TIM3 and ADC for debug purposes.  They will eventually be deleted.
//   0.07   151216  DAH - Added I2C communications
//                          - Added I2C interface constants
//                          - Added struct OVR_I2C
//                          - Added I2C2_EV_IRQHandler()
//                      - Renamed .sram1 to .sram2 for readability
//   0.08   160122  DAH - Added USART6_IRQHandler()
//                      - Revised DMA1_Stream0_IRQHandler() to clear the UserWF_Req flag when the user
//                        waveform capture has been completed
//                      - Revised DMA1_Stream0_IRQHandler() to change the number format for the 1/2-cycle
//                        and 1-cycle sample buffers from floating point to unsigned integer.
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
//                        Since the 1/2-cycle and 1-cycle sums of squares are only used for instantaneous
//                        short delay protection, the history is now stored as 64-bit integers multiplied by
//                        10 (for 0.1A resolution) instead of floating point numbers.  The sums of squares
//                        are converted back to floating point before the protection subroutines are called.
//                        Since integer values are used, there is no issue with resolution error.  Although
//                        the new method takes a little more execution time, it is safer.
//                          - Renamed structure CURRENTS_WITH_G to CUR_WITH_G_F (for floating point)
//                          - Renamed CurHalfCycSOS_Sum to CurHalfCycSOS_SumF
//                          - Renamed CurOneCycSOS_Sum to CurOneCycSOS_SumF
//                          - Added structures CUR_WITH_G_I CurHalfCycSOS_SumI and CurOneCycSOS_SumI that
//                            are integer images of the floating point structures x 100
//                          - Renamed structure CURRENTS_WITHOUT_G to CUR_WITHOUT_G_F
//                          - Redefined HalfCycSamples[][] and OneCycSamples[][] from float to
//                            unsigned long long and reduced size.  These arrays only handle currents.  Also
//                            renamed the arrays to HalfCycSamplesSq and OneCycSamplesSq since the arrays
//                            now hold the squared sample values
//   0.09   160224  DAH - Revised comment in EXTI9_5_IRQHandler().  PF8 changed to PB8 for rev 2 hardware
//                      - Renamed USART6_IRQHandler() to USART3_IRQHandler() and changed the routine to
//                        process UART3 instead of UART6.  The Modbus port was moved from UART6 to UART3 on
//                        the rev 2 boards.
//                      - Added support for basic seeding of the AFE integrator.  After a reset, the AFE
//                        integrator is seeded with the ADC input.  Samples are not yet aligned, but this
//                        should give us an idea of how well the basic seeding works.
//                          - Added Getting_AFE_Seed
//                          - Modified DMA1_Stream0_IRQHandler() to exit without processing any samples
//                            until the integrator is seeded.  This takes two ADC samples (one from the high
//                            gain circuit and one from the low gain circuit), so this results in an
//                            additional delay of about 412usec.
//                      - Added TA_Timer to support the tripping function.  Timer is decremented in the
//                        AFE sampling interrupt (DMA1_Stream0_IRQHandler()) and the TA drive signal is
//                        turned off after 5msec
//                      - Fixed bug in ADC_IRQHandler() that prevented the DMA from working
//                      - In DMA1_Stream0_IRQHandler(), commented out calls to instantaneous and short delay
//                        protection subroutines so protection doesn't interfere with metering testing
//                      - Added min and max half-cycle, one-cycle, and one-second data acquisition
//                          - In DMA1_Stream0_IRQHandler(), added code to compute the half-cycle and
//                            one-cycle min and max values for internal testing
//                          - Added ResetMinMaxValues() to reset the min and max values
//   0.10   160310  DAH - Added code to support short delay protection control and waveform capture via the
//                        Test Port
//                          - Added SD_ProtOn and TestSamples[][]
//                          - Revised DMA1_Stream0_IRQHandler() to call short delay protection if SD_ProtOn
//                            flag is True
//                      - Updated the metered values.  Filtering added and integration time changed from one
//                        second to 200msec
//                          - Renamed structure CurOneSecSOS_Sum to Cur200msecSOS_Sum
//                          - Structure CurOneSecSOS_Sav renamed to Cur200msecSOS_Sav
//                          - Structure VolAFEOneSecLNSOS_Sum renamed to VolAFE200msecLNSOS_Sum
//                          - Structure VolAFEOneSecLLSOS_Sum renamed to VolAFE200msecLLSOS_Sum
//                          - Structure VolAFEOneSecLLSOS_Sav renamed to VolAFE200msecLLSOS_Sav
//                          - Structure VolAFEOneSecLNSOS_Sav renamed to VolAFE200msecLNSOS_Sav
//                          - Renamed structure PwrOneSecSOS_Sum to Pwr200msecSOS_Sum
//                          - Structure PwrOneSecSOS_Sav renamed to Pwr200msecSOS_Sav
//                          - Renamed OneSecAnniv to msec200Anniv
//                          - Renamed OneSecCtr to msec200Ctr
//                          - In DMA1_Stream0_IRQHandler(), changed anniversary count from 4800 to 960 
//                          - In DMA1_Stream0_IRQHandler(), the newly-named 200msec current, voltage, and
//                            power values are now updated with the filtered samples
//                      - Added the 200msec min and max values to ResetMinMaxValues()
//   0.12   160502  DAH - Added one-cycle and 200msec LN and LL voltage values to ResetMinMaxValues()
//                      - Added reactive power calculations to DMA1_Stream0_IRQHandler()
//                          - Structures DelayedVolts_OC and DelayedVolts_200msec added
//                          - DelayedVoltsNdx added
//                      - Added code to support AFE-ADC sample synchronization
//                      - Revised DMA1_Stream0_IRQHandler() to use switch-case construct for the state
//                        machine
//                          - Modified DMA1_Stream0_IRQHandler() to turn on the ADC timer (Timer 3)
//                          - Replaced variable initialization code (in old State 0) with a subroutine call
//                            (AFEISR_VarInit)
//                          - Renamed Sample_State to AFE_SampleState
//                          - Deleted Getting_AFE_Seed (moved to Meter.c)
//                          - Added States 0 thru 5 to DMA1_Stream0_IRQHandler() to handle the AFE-ADC
//                            sample alignment and the AFE sample integration seeding.  Old States 0 - 5
//                            are now States 5 - 9
//                      - Revised DMA1_Stream0_IRQHandler() to correct the calculation of In sum of squares
//                        (added division by 100)
//   0.13   160512  DAH - Updated and revised comments and updated the execution time for
//                        DMA1_Stream0_IRQHandler()
//                      - In DMA1_Stream0_IRQHandler(), corrected bug computing Phase B reactive power
//                      - Added power min and max values to ResetMinMaxValues()
//   0.14   160620  DAH - Added support for on-board test-injection
//                          - DMA1_Stream0_IRQHandler() modified to output cosine waveform or DC signal via
//                            DAC1 if test injection is on
//   0.15   160718  DAH - Added support for reading the Clear LEDs pushbutton
//                          - Added call to ReadSwitches() in DMA1_Stream0_IRQHandler()
//                      - In DMA1_Stream0_IRQHandler(), moved test injection code to a new subroutine,
//                        TestInj_Handler(),for readability
//                      - Added storing waveform samples in the Black Box FRAM
//                          - Added include of FRAM_def.h
//                          - Added BB_Frozen flag to allow sample storage to be halted
//                          - Revised DMA1_Stream0_IRQHandler() to store the current and voltage samples in
//                            BB_SamplePkt.  Added state machine to initiate a DMA operation to write the
//                            sample packet to the Black Box FRAM when ten sets of samples have been
//                            accumulated.
//                          - Added struct BB_SamplePkt, struct BB_TimeStamp, BB_State, and BB_SampleCtr
//                      - Added storing the waveform samples in RAM for Trip event, Alarm event, and
//                        User-initiated captures
//                          - Added struct SampleBuf[] and SampleIndex to store the samples
//                          - Revised DMA1_Stream0_IRQHandler() to store the current and voltage samples in
//                            SampleBuf[].
//                      - Added support for sampled-value CAM communications
//                          - Added includes of CAMCom_def.h and CAMCom_ext.h
//                          - Revised DMA1_Stream0_IRQHandler() to initiate DMA transfer of the present RAM
//                            samples (in SampleBuf[SampleIndex]) to the CAM UART
//                      - Deleted USART6_IRQHandler() since CAM UARTs use DMA instead of interrupts
//   0.16   160818  DAH - Added support for basic display communications
//                          - Added USART2_IRQHandler()
//                      - In UART5_IRQHandler(), replaced TX_STRING with TP_TX_STRING and TX_VALUE with
//                        TP_TX_VALUE
//                      - In USART3_IRQHandler(), replaced TX_VALUE with MB_TX_VALUE
//                      - In DMA1_Stream0_IRQHandler(), fixed error in storing residual gnd in SampleBuf[]
//                      - Fixed minor bug in DMA2_Stream0_IRQHandler()
//                      - Added 5 minute anniversary markers to support Startup Time calibration
//                          - Added Anniv_5minCtr, min5Anniv flag, and COUNT_5MIN
//                          - Added Anniv_5minCtr to Intr_VarInit()
//                          - Revised SysTick_Handler() to generate 5 minute anniversaries
//                      - Removed ADC3 conversions from DMA2_Stream0_IRQHandler().  The conversions will be
//                        done in the foreground as needed
//                          - Deleted const ADC3_CHANNEL[] (replaced with definitions in Iod_def.h)
//                          - Deleted ch_select and ADC3_result[]
//   0.17   161031  DAH - Deleted TIM3_IRQHandler() and ADC_IRQHandler() as Timer 3 and ADC complete
//                        interrupts are no longer used (DMA is used instead).  Deleted USART3_IRQHandler()
//                        as Modbus will be moved to the Display Processor
//   0.18   161115  DAH - Added I2C3_EV_IRQHandler() and I2C3_ER_IRQHandler() to support the Health Bus for
//                        rev 3 boards
//   0.19   170223  DAH - Revised EXTI9_5_IRQHandler() and DMA1_Stream0_IRQHandler() to change DMA1_Stream5
//                        to DMA1_Stream7
//                      - Deleted USART2_IRQHandler() as display processor communications are now handled
//                        using DMA
//                      - Replaced TP.Temp with TestInj.Channel in TestInj_Handler() so it can support both
//                        Test Port and display communications requests for test injection
//   0.21   180301  DAH - Replaced cosine table COS[] with SIN_COEFF[].  This table is now used for both
//                        test injection and THD.
//                      - Cleaned up DMA1_Stream0_IRQHandler() by replacing numerous sections of code with
//                        inline functions
//                      - Added ADC voltage sums of squares structures to support line-side voltages
//                      - Renamed the voltages in struct RAM_SAMPLES from Vxn1 and Vxn2 to VxnAFE and VxnADC
//                      - Changed SampleBuf[] size from 2880 (36 cycles) to 3120 (39 cycles).  This allows
//                        36 cycles to be stored on a trip or alarm, and gives 50msec (3 cycles) to write
//                        the waveforms to Flash before they are overwritten.  Also changed the size to
//                        constant TOTAL_SAMPLE_SETS
//                      - Added ADC min and max voltages to ResetMinMaxValues()
//                      - Revised the user waveform captures so that it can be either a single cycle of 11
//                        parameters or 12 cycles of one parameter.  Decoupled the waveform capture for
//                        display purposes from the waveform capture for harmonics calculation.  Harmonics
//                        now uses the sample buffer, SampleBuf[], and are computed using 12 cycles.
//                          - UserWF_Req and UserWF_Ack are now a set of bit flags used to request and
//                            acknowledge captures.  There is a dedicated bit corresponding to the requestor
//                            of the capture
//                          - Added UserWF_CaptureCode to hold the type of capture requested
//                          - Added UserWF_Flags to hold the status of the capture and associated harmonics
//                            computation
//                          - Added HarmSampleStartNdx to hold the first sample in the 12-cycle harmonics
//                            window 
//                          - Revised DMA1_Stream0_IRQHandler() for user waveform captures and harmonics as
//                            described above
//                      - Revised SysTick_Handler() to cause harmonics waveform capture every 10 seconds.
//                        This is test code that should eventually be deleted.
//   0.22   180420  DAH - Added support for internal real time
//                          - Revised SysTick_Handler() to increment internal time counters
//                            (SysTickTime.cnt_10msec and SysTickTime.cnt_sec)
//   0.23   180504  DAH - Moved USER_SAMPLES structure definition to intr_def.h because UserSamples is used
//                        externally
//   0.24   180517  DAH - Combined Flash_def.h and FRAM_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//                      - Added alarm events to the sampling interrupt (DMA1_Stream0_IRQHandler())
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (so waveform
//                        writes to Flash can be done as quickly as possible)
//                          - TIM4_IRQHandler() added
//   0.27   181001  DAH - Modified the User Waveform capture portion of DMA1_Stream0_IRQHandler() to expand
//                        the requesting threads to all of the communications ports, the display, etc.
//                          - Replaced UserWF_Req and UserWF_CaptureCode with UserWF_CaptureType[].  This is
//                            an array that holds the request type, including No Request, for each source of
//                            a user waveform capture request.  UserWF_CaptureType[] serves as both a
//                            request flag and a description of the type of capture (one-cyc all, etc.)
//                          - Replaced UserWF_InService with UserWF_Src to hold the source of the capture
//                            request
//                          - Replaced UserWF_Flags with HarmReq and HarmAck (defined in Meter.c).  These
//                            are separate harmonic request and acknowledge flags to ensure they are written
//                            either only in the foreground or only in the background
//                      - Modified Intr_VarInit() to initialize the new variables
//                      - Modified SysTick_Handler() to support the new user waveform capture flags
//                      - Fixed bug in DMA1_Stream0_IRQHandler() with alarm waveform capture requests ("&"
//                        changed to "&&")
//                      - Added support for Trip and Strip-Chart waveform captures to
//                        DMA1_Stream0_IRQHandler()
//   0.28   181030  DAH - Revised SysTick_Handler() to set SendCurrents flag every second so currents are
//                        transmitted to the display processor
//   0.29   190122  DAH - Revised SysTick_Handler() to no longer initiate a user waveform capture every ten
//                        seconds for harmonics.  This could potentially interfere with test port user
//                        waveform capture requests for calibration
//                      - Fixed a bug in the user waveform capture portion of DMA1_Stream0_IRQHandler().
//                        Added code to ensure that a single cycle of all of the waveforms is captured for
//                        both USR_TYPE_ALL captures and USR_TYPE_TP captures
//                      - In user waveform capture portion of DMA1_Stream0_IRQHandler(), changed indices of
//                        ADC_samples[] for the ADC voltage samples.  The size of ADC_samples[] was reduced
//                        because the coil temperature reading was moved to ADC3
//                      - In buffer_samples(), update_VlnADC_SOS(), and update_VllADC_SOS(), changed indices
//                        of ADC_samples[].  The size of ADC_samples[] was reduced because the coil
//                          temperature reading was moved to ADC3
//                      - Added support for coil temperature measurement
//                          - Added coil_temp_samples[] and coil_temp_index
//                          - Added code in DMA1_Stream0_IRQHandler() to capture the coil temperature
//                            samples
//   0.31   190506  DAH - Removed Harmonics computation from User Waveform captures.  They are now
//                        completely independent
//                          - Eliminated USR_TYPE_TP from the capture request types.  This was a special
//                            capture request that requested a single cycle of all waveforms without a
//                            harmonics computation.  Since harmonics will no longer be computed when a user
//                            capture is done, this request is the same as USER_TYPE_ALL.
//                            In DMA1_Stream0_IRQHandler(), replaced USR_TYPE_TP with USR_TYPE_ALL
//                          - In DMA1_Stream0_IRQHandler(), eliminated request for harmonics computation at
//                            the end of a user waveform capture (no longer set HarmReq flag)
//                          - Renamed HarmSampleCtr to UsrWF_SampleCtr
//                          - In DMA1_Stream0_IRQHandler(), removed initializing HarmSampleStartNdx when a
//                            user waveform capture begins (moved to the start of a harmonics computation
//                            in Calc_Harmonics()).  Also deleted HarmSampleStartNdx (moved to Meter.c)
//                      - Harmonics captures now occur every 600msec
//                          - Renamed TestHarmonicsTimer to HarmonicsTimer and reduced delay after reset
//                            from 10 seconds to 3 seconds 
//                          - Revised SysTick_Handler() to set the harmonics request flag (HarmReq) every
//                            600msec
//   0.32   190726  DAH - Moved all inline functions to a separate file, IntrInline_def.h, to improve the
//                        readability
//                      - Added Moodbus support
//                          - Added Process_ModB_RxIRQ() for Modbus Rx interrupt handling
//                          - Added ModB.Comm_Timer, the Modbus communications watchdog timer, to
//                            SysTick_Handler()
//                          - Added UART6_IRQHandler() to handle CAM or Modbus communications interrupts
//                          - Added Process_ModB_RxIRQ() to handle Modbus Rx interrupts
//                      - Added THD and Displacement PF support
//                          - Renamed struct Cur200msecSOS_Sum to Cur200msFltrSOS_Sum to distinguish it from
//                            the unfiltered current variables
//                          - Added struct Cur200msNoFltrSOS_Sum.Ix, replaced CurOneCycSinSOS_Sum with
//                            Cur200msNoFltrSinSOS_Sum and CurOneCycCosSOS_Sum with Cur200msNoFltrCosSOS_Sum
//                            These are used to generate the RMS, Sin component, and cosine component of the
//                            unfiltered currents Ia thru In (don't need Ig) over 200msec, instead of one
//                            cycle, for increased accuracy (matches the harmonic analysis)
//                          - Added struct VolAFE200msNoFltrSOS_Sum.Vxx to generate the unfiltered AFE
//                            voltages Van thru Vcn and Vab thru Vca over 200msec
//                          - Added struct VolAFE200msNoFltrSinSOS_Sum and VolAFE200msNoFltrCosSOS_Sum to
//                            generate the sin and cosine components of the unfiltered AFE voltages Van thru
//                            Vcn over 200msec (Vab thru Vca are handled in Calc_DispPF_THD())
//                          - SIN_COEFF[] moved to global section for use in Calc_DispPF_THD()
//                      - Combined struct VolAFE200msecLNSOS_Sum and VolAFE200msecLLSOS_Sum into one
//                        structure, VolAFE200msFltrSOS_Sum
//                      - Combined struct VolAFEOneCycLNSOS_Sum and VolAFEOneCycLLSOS_Sum into one
//                        structure, VolAFEOneCycSOS_Sum
//                      - Combined struct VolADCOneCycLNSOS_Sum and VolADCOneCycLLSOS_Sum into one
//                        structure, VolADCOneCycSOS_Sum
//                      - Combined struct VolADC200msecLNSOS_Sum and VolADC200msecLLSOS_Sum into one
//                        structure, VolADC200msSOS_Sum
//                      - Cur200msec_max renamed to Cur200msFltr_max and Cur200msec_min to Cur200msFltr_min
//                      - struct VolAFEOneCycLNmax and VolAFEOneCycLLmax combined into VolAFEOneCyc_max
//                      - struct VolAFEOneCycLNmin and VolAFEOneCycLLmin combined into VolAFEOneCyc_min
//                      - struct VolAFE200msecLNmax and VolAFE200msecLLmax combined into VolAFE200msFltr_max
//                      - Combined struct VolAFE200msecLNmin and VolAFE200msecLLmin into VolAFE200msFltr_min
//                      - Combined struct VolADCOneCycLNmax and VolADCOneCycLLmax into VolADCOneCyc_max
//                      - Combined struct VolADCOneCycLNmin and VolADCOneCycLLmin into VolADCOneCyc_min
//                      - Combined struct VolADC200msecLNmax and VolADC200msecLLmax into VolADC200ms_max
//                      - Combined struct VolADC200msecLNmin and VolADC200msecLLmin into VolADC200ms_min
//   0.33   190823  DAH - In DMA1_Stream0_IRQHandler(), moved the code to increment the 200msec counter
//                        (msec200Ctr) into inc_buf_indices() for readablility
//                      - Modified AFEISR_VarInit() to initialize VolAFE200msNoFltrCosSOS_Sum.Vxx
//                      - Added Crest Factor support
//                          - Added CurOneCycPeak (moved from Meter.c)
//                          - In DMA1_Stream0_IRQHandler(), added call to save_OC_peaks()
//                      - Added Frequency measurement support
//                          - Added TIM1_BRK_TIM9_IRQHandler()
//                          - Added MAX_PERIOD
//                          - Added FreqLine.OvrTimer and FreqLoad.OvrTimer to SysTick_Handler()
//   0.34   191002  DAH - Revised frequency measurement code for rev 4 boards.  Frequency measurement moved
//                        from Timer 9 to Timers 10 and 11.
//                          - Deleted TIM1_BRK_TIM9_IRQHandler()
//                          - Added TIM1_UP_TIM10_IRQHandler() and TIM1_TRG_COM_TIM11_IRQHandler()
//                      - Revised test injection for rev 4 boards
//                          - In TestInj_Handler(), replaced TSTINJ_SEL_GND with TSTINJ_GND_ON to turn on
//                            ground test injection.  Added TSTINJ_EN to enable test injection.  Added
//                            TSTINJ_DIS to turn off test injection.  Replaced TSTINJ_SEL_PH with
//                            TSTINJ_GND_OFF  to turn off ground test injection.
//   0.36   200210  DAH - Display_def.h renamed to DispComm_def.h
//                      - Display_ext.h renamed to DispComm_ext.h
//                      - Revised SysTick_Handler() to delete the SendCurrents flag
//   0.37   200604  DAH - In struct FLASH_INT_REQ, StartIndex was renamed SampleStartIndex so it is not
//                        confused with the Event storage index
//                      - Added include of Events_def.h for compilation purposes (for FRAM_Flash_def.h)
//   0.38   200708  DAH - In SysTick_Handler(), added support for display comms timers
//                        DPComm.RTD_XmitTimer[i] and DPComm.TurnaroundTimer
//   0.39   200729  DAH - DPComm.TurnaroundTimer renamed to DPComm.XmitWaitTimer
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added includes of RealTime_def.h and RealTime_ext.h
//   0.43   210115  DAH - Revised DMA1_Stream0_IRQHandler() to call DispComm61850_Rx() and
//                        DispComm61850_Tx() to support 61850 communications with the Display Processor
//                      - Revised EXTI9_5_IRQHandler(), DMA1_Stream0_IRQHandler() and
//                        DMA2_Stream0_IRQHandler() to clear the corresponding event flags (per RM0090
//                        Reference Manual) before enabling the stream
//                      - Added test code to measure the execution time of the 61850 comms routines
//                       (maxlooptime1, etc.).  This code should eventually be deleted.
//   0.46   210521  DAH - Added Prot_Timer to SysTick_Handler() to turn on protection 250msec after the TA
//                        is fired
//   0.49   220131  DAH - Revised code to get DMA working for Modbus transmissions
//                          - Renamed UART6_IRQHandler to USART6_IRQHandler() and corrected pointer (changed
//                            from UART5 to UART6)
//                          - In Process_ModB_RxIRQ(), eliminated code that checks for times between 1.5 and
//                            character times.  We only check whether the time is less than or greater than
//                            3.5 character times.  Also fixed the code that restarts Timer 2
//   0.51   220304  DAH - Min/max current values have switched from 200msec values to one-cycle values
//                          - ResetMinMaxValues() revised (Cur200msFltr_max.Ix and Cur200msFltr_min.Ix
//                            replaced with CurOneCyc_max.Ix and CurOneCyc_min.Ix).  Also, initialization
//                            values changed from min and max extremes to NANs
//                          - Replaced CurOneCyc_min.Igsrc and Igres with CurOneCyc_min.Ig
//                          - Replaced CurOneCyc_max.Igsrc and Igres with CurOneCyc_max.Ig
//                      - Min/max ADC voltage values have switched from 200msec values to one-cycle values
//                          - Deleted VolADC200msec_min.Vxx and VolADC200msec_max.Vxx from
//                            ResetMinMaxValues()
//                          - Changed initialization values for VolADCOneCyc_max.Vxx and
//                            VolADCOneCyc_min.Vxx from extremes to NANs in ResetMinMaxValues()
//   0.53   220325  DAH - Added THD min and max values to ResetMinMaxValues()
//   0.59   220831  DAH - Deleted Black Box FRAM function (10-cycle waveform capture without aux power)
//                          - Deleted "BB_" variables
//                          - Modified DMA1_Stream0_IRQHandler() to delete Black Box sample storage
//                      - In TIM4_IRQHandler(), deleted call to SPI2_NoFlash_Manager().  SPI2 accesses are
//                        are now done in the foreground
//   0.67   221209  DAH - S2F_xx flags renamed to S1F_xx flags
//                      - SPI2Flash.xx renamed to SPI1Flash.xx
//   0.69   230220  DAH - Strip-chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - DMA1_Stream0_IRQHandler() revised
//   0.70   230224  BP  - Added call to Instantaneous and Ground Fault protections based on their configuration
//                        setpoint
//   0.72   230320  DAH - Deleted redundant call to Instantaneous_Prot() in DMA1_Stream0_IRQHandler()
//                      - Deleted HalfCycAnniv as it is no longer used
//                      - Deleted code that sets SPI1Flash.Req in the DMA1_Stream0_IRQHandler()
//                      - Deleted code disabling and enabling interrupts around the call to
//                        Get_InternalTime() in DMA1_Stream0_IRQHandler()
//                  BP  - Added HighLoadLED_BlinkCtr code
//   31     230414  DAH - Fixed waveform capture starting index (added one cycle) so that the correct number
//                        of pre- and post-event cycles are captured.
//                          - DMA1_Stream0_IRQHandler() revised
//                      - For extended captures, added code to adjust the timestamp to the time of the first
//                        sample.  It had been the time of the existing sample.  Note, this needs to be
//                        added to the Trip and Alarm waveforms also.
//                          - DMA1_Stream0_IRQHandler() revised
//   34     230424  DAH - Added min5Anniv to Intr_VarInit()
//   35     230428  DAH - Completed fixing the time stamps for waveform captures.  The time stamp is the
//                        time of the first sample.  Removed the offset computation in the ISR and replaced
//                        it with a global variable that is computed when the setpoints are read.  The
//                        time adjustment is based entirely on the number of pre-cycles (setpoints)
//                          - Added Trip_WF_OffsetTime, Alarm_WF_OffsetTime, Ext_WF_OffsetTime
//                          - Revised DMA1_Stream0_IRQHandler() to fix the timestamps
//                      - Adjusted waveform capture starting index back to the way it was (removed one
//                        cycle) so that the correct number of pre- and post-event cycles are captured
//                          - DMA1_Stream0_IRQHandler() revised
//                      - Cleaned up the waveform capture process.  Deleted code that clears
//                        Trip_WF_Capture.Req and Ext_WF_Capture.Req in DMA1_Stream0_IRQHandler().  The
//                        flags are cleared in EventManager() when capture processing begins
//                      - In DMA1_Stream0_IRQHandler(), replaced the hard-coded number of post-cycles with
//                        the appropriate setpoint
//   42     230623  MAG - Revised TIM4_IRQHandler() and TIM1_UP_TIM10_IRQHandler()
//                      - Deleted TIM1_TRG_COM_TIM11_IRQHandle()
//   44     230623  DAH - Revised EXTI9_5_IRQHandler() to handle an Override Trip
//   54     230802  BP  - Updated for ZSI
//                      - Added call to Ovr_Micro_Comm() in I2C2_EV_IRQHandler()
//   69     230828  DAH - Revised EXTI9_5_IRQHandler() to include Time Sync (PH9) interrupt
//   93     231010  BP  - Added SampleCounter for Sec Injection timing.
//                      - Added Test Timout timer code to SysTick timer for Sec Injection timout 
//   94     231010  DAH - User waveform captures moved to the foreground and out of the sampling interrupt
//                        (DMA1_Stream0_IRQHandler())
//                          - Deleted all UserWF_xx variables (moved to Meter.c)
//   95     231011  DAH - Revised EXTI9_5_IRQHandler() to delete test variable (TP_AFECommsOff) that
//                        controlled whether the DMA SPI comms with the AFE were turned on
//   98     231017  DAH - Added AlarmHoldOffTmr to hold off alarm functions for 2sec after a trip (allows
//                        any current tails to decay).  SysTick_Handler() revised
//                      - Revised DMA1_Stream0_IRQHandler() to only call Ground_Fault_Alarm() if
//                        AlarmHoldOffTmr is inactive (0)
//   113    231128  BP  - Added GF_Enabled for Ground Fault Digitization
//   122    231204  BP  - Commented out ADC3 code for Coil Detection measurement
//   135    231221  DAH - Added SampleBufFilled flag to indicate when the waveform sample buffer is filled
//   147    240126  DAH - In DMA1_Stream0_IRQHandler(), commented out code that clears GF trip and alarm
//                        flags when GF protection is disabled
//   149    240131  DAH - Renamed NUM_RTD_BUFFERS to NUM_RTD_TIMESLICES
//   150    240202  DAH - In DMA1_Stream0_IRQHandler(), recommented out code that clears GF trip and alarm
//                        flags when GF protection is disabled (was put back in for testing)
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
#include <math.h>
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Demand_def.h"
#include "Intr_def.h"
#include "Meter_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"
#include "CAMCom_def.h"
#include "DispComm_def.h"
#include "Modbus_def.h"
#include "Prot_def.h"
#include "Ovrcom_def.h"

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
#include "Iod_ext.h"
#include "RealTime_ext.h"
#include "Test_ext.h"
#include "Meter_ext.h"
#include "Prot_ext.h"
#include "Init_ext.h"
#include "CAMCom_ext.h"
#include "DispComm_ext.h"
#include "Modbus_ext.h"
#include "Setpnt_ext.h"
#include "Ovrcom_ext.h"

//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Intr_VarInit(void);
void ResetMinMaxValues(void);
void SysTick_Handler(void);                  // Only used in startup_stm32f407xx.s
void UART5_IRQHandler(void);                 // Only used in startup_stm32f407xx.s
void EXTI9_5_IRQHandler(void);               // Only used in startup_stm32f407xx.s
void DMA1_Stream0_IRQHandler(void);          // Only used in startup_stm32f407xx.s
void DMA2_Stream0_IRQHandler(void);          // Only used in startup_stm32f407xx.s
void I2C2_EV_IRQHandler(void);               // Only used in startup_stm32f407xx.s
void I2C3_EV_IRQHandler(void);               // Only used in startup_stm32f407xx.s
void I2C3_ER_IRQHandler(void);               // Only used in startup_stm32f407xx.s
void TIM4_IRQHandler(void);                  // Only used in startup_stm32f407xx.s
void TIM1_UP_TIM10_IRQHandler(void);         // Only used in startup_stm32f407xx.s
void USART6_IRQHandler(void);                // Only used in startup_stm32f407xx.s



//      Local Function Prototypes (These functions are called only within this module)
//
void AFEISR_VarInit(void);
void TestInj_Handler(void);
void Process_ModB_RxIRQ(uint16_t status_reg, uint16_t rx_data_reg);


//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants and definitions used in this module
//------------------------------------------------------------------------------------------------------------
//
//
// I2C Interface Definitions
#define I2C2_IDLE               0x00
#define I2C2_TX                 0x01
#define I2C2_RX                 0x02
#define OVR_I2C_STAT_CHANGE     0x01

// Timer anniversary counters
#define     COUNT_5MIN          30000           // Count for 5 minutes at 10msec SysTick

// Max allowable period for frequency measurement
#define     MAX_PERIOD          4

// Process_ModB_RxIRQ() states
enum Process_ModB_RxIRQ_States
{
  CHECK_INTERVAL, FRAME_BREAK, CHAR_RECEIVED
};


//------------------------------------------------------------------------------------------------------------
//                   Global Constants
//------------------------------------------------------------------------------------------------------------

// Sine table for Test Injection (offset by 20 to generate cosine value), displacement power factor, and THD
const float SIN_COEFF[80] =
{  
  //     0 -   0.0deg           1 -   4.5deg           2 -   9.0deg           3 -  13.5deg
     0.0E+00,               7.84590957278449E-02,  1.56434465040231E-01,  2.33445363855905E-01,
  
  //     4 -  18.0deg           5 -  22.5deg           3 -  27.0deg           7 -  31.5deg
     3.09016994374947E-01,  3.82683432365090E-01,  4.53990499739547E-01,  5.22498564715949E-01,
  
  //     8 -  36.0deg           9 -  40.5deg          10 -  45.0deg          11 -  49.5deg
     5.87785252292473E-01,  6.49448048330184E-01,  7.07106781186547E-01,  7.60405965600031E-01,

  //    12 -  54.0deg          13 -  58.5deg          14 -  63.0deg          15 -  67.5deg
     8.09016994374947E-01,  8.52640164354092E-01,  8.91006524188368E-01,  9.23879532511287E-01,

  //    16 -  72.0deg          17 -  76.5deg          18 -  81.0deg          19 -  85.5deg
     9.51056516295154E-01,  9.72369920397677E-01,  9.87688340595138E-01,  9.96917333733128E-01,

  //    20 -  90.0deg          21 -  94.5deg          22 -  99.0deg          23 - 103.5deg
     1.00000000000000E+00,  9.96917333733128E-01,  9.87688340595138E-01,  9.72369920397677E-01,

  //    24 - 108.0deg          25 - 112.5deg          26 - 117.0deg          27 - 121.5deg
     9.51056516295154E-01,  9.23879532511287E-01,  8.91006524188368E-01,  8.52640164354092E-01,

  //    28 - 126.0deg          29 - 130.5deg          30 - 135.0deg          31 - 139.5deg
     8.09016994374947E-01,  7.60405965600031E-01,  7.07106781186548E-01,  6.49448048330184E-01,

  //    32 - 144.0deg          33 - 148.5deg          34 - 153.0deg          35 - 157.5deg
     5.87785252292473E-01,  5.22498564715949E-01,  4.53990499739547E-01,  3.82683432365090E-01,

  //    36 - 162.0deg          37 - 166.5deg          38 - 171.0deg          39 - 175.5deg
     3.09016994374948E-01,  2.33445363855906E-01,  1.56434465040231E-01,  7.84590957278451E-02,

  //    40 - 180.0deg          41 - 184.5deg          42 - 189.0deg          43 - 193.5deg
     0.0E+00,              -7.84590957278448E-02, -1.56434465040231E-01, -2.33445363855906E-01,

  //    44 - 198.0deg          45 - 202.5deg          46 - 207.0deg          47 - 211.5deg
    -3.09016994374948E-01, -3.82683432365089E-01, -4.53990499739546E-01, -5.22498564715949E-01,

  //    48 - 216.0deg          49 - 220.5deg          50 - 225.0deg          51 - 229.5deg
    -5.87785252292473E-01, -6.49448048330184E-01, -7.07106781186547E-01, -7.60405965600031E-01,

  //    52 - 234.0deg          53 - 238.5deg          54 - 243.0deg          55 - 247.5deg
    -8.09016994374947E-01, -8.52640164354092E-01, -8.91006524188368E-01, -9.23879532511287E-01,

  //    56 - 252.0deg          57 - 256.5deg          58 - 261.0deg          59 - 265.5deg
    -9.51056516295154E-01, -9.72369920397677E-01, -9.87688340595138E-01, -9.96917333733128E-01,

  //    60 - 270.0deg          61 - 274.5deg          62 - 279.0deg          63 - 283.5deg
    -1.00000000000000E+00, -9.96917333733128E-01, -9.87688340595138E-01, -9.72369920397677E-01,

  //    64 - 288.0deg          65 - 292.5deg          66 - 297.0deg          67 - 301.5deg
    -9.51056516295154E-01, -9.23879532511287E-01, -8.91006524188368E-01, -8.52640164354092E-01,

  //    68 - 306.0deg          69 - 310.5deg          70 - 315.0deg          71 - 319.5deg
    -8.09016994374948E-01, -7.60405965600031E-01, -7.07106781186548E-01, -6.49448048330184E-01,

  //    72 - 324.0deg          73 - 328.5deg          74 - 333.0deg          75 - 337.5deg
    -5.87785252292473E-01, -5.22498564715949E-01, -4.53990499739547E-01, -3.82683432365090E-01,

  //    76 - 342.0deg          77 - 346.5deg          78 - 351.0deg          79 - 355.5deg
    -3.09016994374948E-01, -2.33445363855906E-01, -1.56434465040231E-01, -7.84590957278456E-02
};
//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct CUR_WITH_G_F CurHalfCycSOS_SumF @".sram2";   // Variables located in .sram2 are only used in
struct CUR_WITH_G_F    CurOneCycSOS_SumF @".sram2";
float CurHalfCycSOSmax @".sram2";                   //   subroutines that are called from                  
float CurOneCycSOSmax @".sram2";                    //   DMA1_Stream0_IRQHandler().  There is no conflict   
float NewSample @".sram2";                          //   with the associated DMA stream, since the stream
                                                    //   will not be active at this time
uint8_t msec200Anniv, OneCycAnniv, OneSecAnniv, min5Anniv;

uint8_t TA_Timer;

struct CUR_WITH_G_F CurHCSOS_Fmax;
struct CUR_WITH_G_F CurHCSOS_Fmin;
struct CUR_WITH_G_F CurOCSOS_Fmax;
struct CUR_WITH_G_F CurOCSOS_Fmin;
uint8_t FreezeMinMax;

uint8_t Prot_Enabled;
uint8_t GF_Enabled;

float TestSamples[4][400];              // *** DAH Added for test
                                        // TestSamples[0][xxx] - scaled AFE samples before digital
                                        //                       integrator
                                        // TestSamples[1][xxx] - scaled AFE samples after digital integrator
                                        //                       and before digital filter
                                        // TestSamples[2][xxx] - scaled ADC samples
                                        // TestSamples[3][xxx] - scaled AFE samples after digital filter
                                        // All waveforms are Phase A current (channel 0)

struct RAM_SAMPLES SampleBuf[TOTAL_SAMPLE_SETS] @ ".sram1";
uint16_t SampleIndex @".sram2";

uint16_t coil_temp_samples[200];
//uint16_t coil_temp_samples[600];      // *** DAH  FOR LEO 190605

float Igres_Protsample;

uint16_t Sync_Phase, TransferNow;

uint32_t SampleCounter;

uint8_t AlarmHoldOffTmr;

uint16_t Admin_Verified_Tmr;
uint16_t User_Verified_Tmr;
uint16_t Pswd_Rejection_Tmr;
uint8_t  Pswd_attempt;

uint8_t SampleBufFilled;


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
unsigned long long HalfCycSamplesSq[6][40] @".sram2";
unsigned long long OneCycSamplesSq[6][80] @".sram2";
struct CUR_WITH_G_I    CurHalfCycSOS_SumI @".sram2";
struct CUR_WITH_G_F    CurOneCycSOS_SumF @".sram2";
struct CUR_WITH_G_I    CurOneCycSOS_SumI @".sram2";
struct CUR_WITH_G_F    Cur200msFltrSOS_Sum @".sram2";
struct CUR_WITHOUT_G_F Cur200msNoFltrSOS_Sum @".sram2";
struct CUR_WITHOUT_G_F Cur200msNoFltrSinSOS_Sum @".sram2";
struct CUR_WITHOUT_G_F Cur200msNoFltrCosSOS_Sum @".sram2";
struct CUR_WITHOUT_G_F CurOneCycPeak;
struct VOLTAGES        VolADCOneCycSOS_Sum @".sram2";
struct VOLTAGES        VolAFEOneCycSOS_Sum @".sram2";
struct VOLTAGES        VolADC200msSOS_Sum @".sram2";
struct VOLTAGES        VolAFE200msFltrSOS_Sum @".sram2";
struct VOLTAGES        VolAFE200msNoFltrSOS_Sum @".sram2";
struct VOLTAGES_LN     VolAFE200msNoFltrSinSOS_Sum @".sram2";
struct VOLTAGES_LN     VolAFE200msNoFltrCosSOS_Sum @".sram2";
struct POWERS          PwrOneCycSOS_Sum @".sram2";
struct POWERS          Pwr200msecSOS_Sum @".sram2";

struct DELAY_VOLTS_OC
{
    float Van[20];
    float Vbn[20];
    float Vcn[20];
} DelayedVolts_OC @".sram2";

struct DELAY_VOLTS_200msec
{
    float Van[20];
    float Vbn[20];
    float Vcn[20];
} DelayedVolts_200msec @".sram2";

uint16_t msec200Ctr @".sram2";
uint16_t OneSecCtr  @".sram2";
uint8_t HalfCycInd @".sram2";
uint8_t OneCycInd @".sram2";
uint8_t AFE_SampleState @".sram2";                    
uint8_t DelayedVoltsNdx @".sram2";

unsigned long long ulltemp[6];       // This has to be defined globally in order to use inline functions
float Igres, Igres_mtr;              // These have to be defined globally in order to use inline functions

uint16_t Anniv_5minCtr;
uint16_t HarmonicsTimer;


uint8_t Prot_Timer;

uint32_t Trip_WF_OffsetTime, Alarm_WF_OffsetTime, Ext_WF_OffsetTime;

uint8_t coil_temp_index;
//uint16_t coil_temp_index;         // *** DAH  FOR LEO 190605

// uint16_t freddah, freddah1;                  //  *** DAH  UNCOMMENT FOR HARMONICS TEST
struct INTERNAL_TIME starttime1, endtime1;                // *** DAH ADDED TO MEASURE THE HIGH-SPEED COMMS TIME
uint32_t looptime1, maxlooptime1;                         // *** DAH ADDED TO MEASURE THE HIGH-SPEED COMMS TIME

// This file only contains in-line subroutines used in this module only.  The subroutines were placed in a
//   separate file for readability purposes only.  The file must be included here, after all of the variable
//   and constant definitions, so that the variables are defined for the in-line routines 
#include "IntrInline_def.h"



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Intr_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Interrupt Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Interrupt Module.
//                      The following variables do not need to be initialized in this subroutine:
//                        CurHalfCycSOS_SumF.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        CurHalfCycSOS_SumI.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        CurHalfCycSOSmax (does not need to be initialized)
//                        CurOneCycSOS_SumF.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        CurOneCycSOS_SumI.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        CurOneCycSOSmax (does not need to be initialized)
//                        NewSample (does not need to be initialized)
//                        Cur200msFltrSOS_Sum.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        Cur200msNoFltrSinSOS_Sum.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        Cur200msNoFltrCosSOS_Sum.Ix (initialized in DMA1_Stream0_IRQHandler())
//                        VolAFEOneCycSOS_Sum.Vxx (initialized in DMA1_Stream0_IRQHandler())
//                        VolAFE200msFltrSOS_Sum.Vxx (initialized in DMA1_Stream0_IRQHandler())
//                        VolADCOneCycSOS_Sum.Vxx (initialized in DMA1_Stream0_IRQHandler())
//                        VolADC200msSOS_Sum.Vxx (initialized in DMA1_Stream0_IRQHandler())
//                        PwrOneCycSOS_Sum.X (initialized in DMA1_Stream0_IRQHandler())
//                        Pwr200msecSOS_Sum.X (initialized in DMA1_Stream0_IRQHandler())
//                        HalfCycSamplesSq[][] (does not need to be initialized)
//                        OneCycSamplesSq[][] (does not need to be initialized)
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            AFE_SampleState, HalfCycInd, msec200Ctr, OneCycInd, msec200Anniv, OneCycAnniv,
//                      min5Anniv, OVR_I2C.Status, OVR_I2C.TxNdx, OVR_I2C.RxNdx, SampleIndex,
//                      coil_temp_index, Prot_Timer, OneSecCtr, OneSecAnniv, TA_Timer, Prot_Enabled,
//                      Anniv_5minCtr, HarmonicsTimer, AlarmHoldOffTmr, SampleBufFilled
//
//  ALTERS:             None
//
//  CALLS:              ResetMinMaxValues()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code, no FRAM reads): 18usec
//
//------------------------------------------------------------------------------------------------------------

void Intr_VarInit(void)
{
  HalfCycInd = 0;
  OneCycInd = 0;
  msec200Ctr = 0;
  OneSecCtr = 0;
  AFE_SampleState = 0;
  msec200Anniv = FALSE;
  OneCycAnniv = FALSE;
  OneSecAnniv = FALSE;
  min5Anniv = FALSE;
  
  OVR_I2C.Status = I2C2_IDLE;
  OVR_I2C.TxNdx = 0;
  OVR_I2C.RxNdx = 0;

  HLTH_I2C.Status = I2C3_IDLE;
  HLTH_I2C.TxNdx = 0;
  HLTH_I2C.RxNdx = 0;
  HLTH_I2C.ErrCount = 0;

  TA_Timer = 0;
  SampleIndex = 0;
  AlarmHoldOffTmr = 0;

  Admin_Verified_Tmr = 0;
  User_Verified_Tmr = 0;
  Pswd_Rejection_Tmr = 0;
  Pswd_attempt = 0;

  ResetMinMaxValues();

  Prot_Enabled = TRUE;                  // Protection is enabled
  
  Anniv_5minCtr = 10;                   // Initialize to 10 so startup discharge cap is calibrated quickly.
                                        //   However, the value must be large enough to ensure the startup
                                        //   time is measured (when 120MHz is running and sampling has
                                        //   begun) before the cap is discharged.

  // Initialize to 3 seconds to ensure currents are stable and sample buffer is filled (39 cycles)
  HarmonicsTimer = 300;

  coil_temp_index = 0;

  Prot_Timer = 0;
  SampleCounter = 0;

  SampleBufFilled = FALSE;
  
//  freddah = 0;                //  *** DAH  UNCOMMENT FOR HARMONICS TEST
//  freddah1 = 0;                //  *** DAH  UNCOMMENT FOR HARMONICS TEST
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Intr_VarInit()
//------------------------------------------------------------------------------------------------------------






//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AFEISR_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           AFE Sample Interrupt Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the AFE Sample Interrupt
//                      routine.
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            CurHalfCycSOS_SumF.Ix, CurHalfCycSOS_SumI.Ix, CurOneCycSOS_SumF.Ix,
//                      CurOneCycSOS_SumI.Ix, CurOneCycPeak.Ix, Cur200msFltrSOS_Sum.Ix,
//                      Cur200msNoFltrSinSOS_Sum.Ix, Cur200msNoFltrCosSOS_Sum.Ix, VolAFEOneCycSOS_Sum.Vx,
//                      VolAFE200msFltrSOS_Sum.Vx, VolADCOneCycSOS_Sum.Vx, VolADC200msSOS_Sum.Vx,
//                      PwrOneCycSOS_Sum.Px, PwrOneCycSOS_Sum.RPx, Pwr200msecSOS_Sum.Px,
//                      Pwr200msecSOS_Sum.RPx, DelayedVolts_OC.Vx[], DelayedVolts_200msec.Vx[],
//                      DelayedVoltsNdx, Cur200msNoFltrSOS_Sum.Ix, VolAFE200msNoFltrSOS_Sum.Vxx,
//                      VolAFE200msNoFltrSinSOS_Sum.Vxx, VolAFE200msNoFltrCosSOS_Sum.Vxx
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void AFEISR_VarInit(void)
{
  uint8_t i;

  CurHalfCycSOS_SumF.Ia = 0;
  CurHalfCycSOS_SumF.Ib = 0;
  CurHalfCycSOS_SumF.Ic = 0;
  CurHalfCycSOS_SumF.In = 0;
  CurHalfCycSOS_SumF.Igsrc = 0;
  CurHalfCycSOS_SumF.Igres = 0;
  CurHalfCycSOS_SumI.Ia = 0;
  CurHalfCycSOS_SumI.Ib = 0;
  CurHalfCycSOS_SumI.Ic = 0;
  CurHalfCycSOS_SumI.In = 0;
  CurHalfCycSOS_SumI.Igsrc = 0;
  CurHalfCycSOS_SumI.Igres = 0;
  CurOneCycSOS_SumF.Ia = 0;
  CurOneCycSOS_SumF.Ib = 0;
  CurOneCycSOS_SumF.Ic = 0;
  CurOneCycSOS_SumF.In = 0;
  CurOneCycSOS_SumF.Igsrc = 0;
  CurOneCycSOS_SumF.Igres = 0;
  CurOneCycSOS_SumI.Ia = 0;
  CurOneCycSOS_SumI.Ib = 0;
  CurOneCycSOS_SumI.Ic = 0;
  CurOneCycSOS_SumI.In = 0;
  CurOneCycSOS_SumI.Igsrc = 0;
  CurOneCycSOS_SumI.Igres = 0;
  CurOneCycPeak.Ia = 0;
  CurOneCycPeak.Ib = 0;
  CurOneCycPeak.Ic = 0;
  CurOneCycPeak.In = 0;
  Cur200msFltrSOS_Sum.Ia = 0;
  Cur200msFltrSOS_Sum.Ib = 0;
  Cur200msFltrSOS_Sum.Ic = 0;
  Cur200msFltrSOS_Sum.In = 0;
  Cur200msFltrSOS_Sum.Igsrc = 0;
  Cur200msFltrSOS_Sum.Igres = 0;
  Cur200msNoFltrSOS_Sum.Ia = 0;
  Cur200msNoFltrSOS_Sum.Ib = 0;
  Cur200msNoFltrSOS_Sum.Ic = 0;
  Cur200msNoFltrSOS_Sum.In = 0;
  Cur200msNoFltrSinSOS_Sum.Ia = 0;
  Cur200msNoFltrSinSOS_Sum.Ib = 0;
  Cur200msNoFltrSinSOS_Sum.Ic = 0;
  Cur200msNoFltrSinSOS_Sum.In = 0;
  Cur200msNoFltrCosSOS_Sum.Ia = 0;
  Cur200msNoFltrCosSOS_Sum.Ib = 0;
  Cur200msNoFltrCosSOS_Sum.Ic = 0;
  Cur200msNoFltrCosSOS_Sum.In = 0;
  VolAFEOneCycSOS_Sum.Van = 0;
  VolAFEOneCycSOS_Sum.Vbn = 0;
  VolAFEOneCycSOS_Sum.Vcn = 0;
  VolAFEOneCycSOS_Sum.Vab = 0;
  VolAFEOneCycSOS_Sum.Vbc = 0;
  VolAFEOneCycSOS_Sum.Vca = 0;
  VolAFE200msFltrSOS_Sum.Van = 0;
  VolAFE200msFltrSOS_Sum.Vbn = 0;
  VolAFE200msFltrSOS_Sum.Vcn = 0;
  VolAFE200msFltrSOS_Sum.Vab = 0;
  VolAFE200msFltrSOS_Sum.Vbc = 0;
  VolAFE200msFltrSOS_Sum.Vca = 0;
  VolAFE200msNoFltrSOS_Sum.Van = 0;
  VolAFE200msNoFltrSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrSOS_Sum.Vcn = 0;
  VolAFE200msNoFltrSOS_Sum.Vab = 0;
  VolAFE200msNoFltrSOS_Sum.Vbc = 0;
  VolAFE200msNoFltrSOS_Sum.Vca = 0;
  VolAFE200msNoFltrSinSOS_Sum.Van = 0;
  VolAFE200msNoFltrSinSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrSinSOS_Sum.Vcn = 0;
  VolAFE200msNoFltrCosSOS_Sum.Van = 0;
  VolAFE200msNoFltrCosSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrCosSOS_Sum.Vcn = 0;
  VolADCOneCycSOS_Sum.Van = 0;
  VolADCOneCycSOS_Sum.Vbn = 0;
  VolADCOneCycSOS_Sum.Vcn = 0;
  VolADCOneCycSOS_Sum.Vab = 0;
  VolADCOneCycSOS_Sum.Vbc = 0;
  VolADCOneCycSOS_Sum.Vca = 0;
  VolADC200msSOS_Sum.Van = 0;
  VolADC200msSOS_Sum.Vbn = 0;
  VolADC200msSOS_Sum.Vcn = 0;
  VolADC200msSOS_Sum.Vab = 0;
  VolADC200msSOS_Sum.Vbc = 0;
  VolADC200msSOS_Sum.Vca = 0;
  PwrOneCycSOS_Sum.Pa = 0;
  PwrOneCycSOS_Sum.Pb = 0;
  PwrOneCycSOS_Sum.Pc = 0;
  PwrOneCycSOS_Sum.RPa = 0;
  PwrOneCycSOS_Sum.RPb = 0;
  PwrOneCycSOS_Sum.RPc = 0;
  Pwr200msecSOS_Sum.Pa = 0;
  Pwr200msecSOS_Sum.Pb = 0;
  Pwr200msecSOS_Sum.Pc = 0;
  Pwr200msecSOS_Sum.RPa = 0;
  Pwr200msecSOS_Sum.RPb = 0;
  Pwr200msecSOS_Sum.RPc = 0;
  for (i=0; i<20; ++i)
  {
    DelayedVolts_OC.Van[i] = 0;
    DelayedVolts_OC.Vbn[i] = 0;
    DelayedVolts_OC.Vcn[i] = 0;
    DelayedVolts_200msec.Van[i] = 0;
    DelayedVolts_200msec.Vbn[i] = 0;
    DelayedVolts_200msec.Vcn[i] = 0;
  }
  DelayedVoltsNdx = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AFEISR_VarInit()
//------------------------------------------------------------------------------------------------------------






//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ResetMinMaxValues()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reset Min/Max Values
//
//  MECHANICS:          This subroutine resets the min and max values for the half-cycle, one-cycle, and
//                      one-second sums of squares.  It also resets FreezeMinMax, which allows min and max
//                      acquisition to begin
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            CurHCSOS_Fmax.Ix, CurHCSOS_Fmin.Ix, CurOCSOS_Fmax.Ix, CurOCSOS_Fmin.Ix,
//                      FreezeMinMax, CurOneCyc_max.Ix, CurOneCyc_min.Ix, VolAFEOneCyc_max.Vxx,
//                      CurOneCycIgmax, CurOneCycIgmin, VolAFEOneCyc_min.Vxx, VolAFE200msFltr_max.Vxx,
//                      VolAFE200msFltr_min.Vxx, VolADCOneCyc_max.Vxx, VolADCOneCyc_min.Vxx,
//                      PwrOneCycMax.Xxx, PwrOneCycMin.Xxx, PwrOneCycAppMax.Xxx, PwrOneCycAppMin.Xxx,
//                      Pwr200msecMax.Xxx, Pwr200msecMin.Xxx, Pwr200msecAppMax.Xxx, Pwr200msecAppMin.Xxx
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void ResetMinMaxValues(void)
{
  uint8_t  i;

  CurHCSOS_Fmax.Ia = 0;
  CurHCSOS_Fmax.Ib = 0;
  CurHCSOS_Fmax.Ic = 0;
  CurHCSOS_Fmax.In = 0;
  CurHCSOS_Fmax.Igsrc = 0;
  CurHCSOS_Fmax.Igres = 0;
  CurHCSOS_Fmin.Ia = 1E36;
  CurHCSOS_Fmin.Ib = 1E36;
  CurHCSOS_Fmin.Ic = 1E36;
  CurHCSOS_Fmin.In = 1E36;
  CurHCSOS_Fmin.Igsrc = 1E36;
  CurHCSOS_Fmin.Igres = 1E36;

  CurOCSOS_Fmax.Ia = 0;
  CurOCSOS_Fmax.Ib = 0;
  CurOCSOS_Fmax.Ic = 0;
  CurOCSOS_Fmax.In = 0;
  CurOCSOS_Fmax.Igsrc = 0;
  CurOCSOS_Fmax.Igres = 0;
  CurOCSOS_Fmin.Ia = 1E36;
  CurOCSOS_Fmin.Ib = 1E36;
  CurOCSOS_Fmin.Ic = 1E36;
  CurOCSOS_Fmin.In = 1E36;
  CurOCSOS_Fmin.Igsrc = 1E36;
  CurOCSOS_Fmin.Igres = 1E36;

  CurOneCyc_max.Ia = NAN;
  CurOneCyc_max.Ib = NAN;
  CurOneCyc_max.Ic = NAN;
  CurOneCyc_max.In = NAN;
  CurOneCyc_max.Ig = NAN;
  CurOneCyc_min.Ia = NAN;
  CurOneCyc_min.Ib = NAN;
  CurOneCyc_min.Ic = NAN;
  CurOneCyc_min.In = NAN;
  CurOneCyc_min.Ig = NAN;

  VolAFEOneCyc_max.Van = NAN;
  VolAFEOneCyc_max.Vbn = NAN;
  VolAFEOneCyc_max.Vcn = NAN;
  VolAFEOneCyc_min.Van = NAN;
  VolAFEOneCyc_min.Vbn = NAN;
  VolAFEOneCyc_min.Vcn = NAN;

  VolAFEOneCyc_max.Vab = NAN;
  VolAFEOneCyc_max.Vbc = NAN;
  VolAFEOneCyc_max.Vca = NAN;
  VolAFEOneCyc_min.Vab = NAN;
  VolAFEOneCyc_min.Vbc = NAN;
  VolAFEOneCyc_min.Vca = NAN;

  VolADCOneCyc_max.Van = NAN;
  VolADCOneCyc_max.Vbn = NAN;
  VolADCOneCyc_max.Vcn = NAN;
  VolADCOneCyc_min.Van = NAN;
  VolADCOneCyc_min.Vbn = NAN;
  VolADCOneCyc_min.Vcn = NAN;

  VolADCOneCyc_max.Vab = NAN;
  VolADCOneCyc_max.Vbc = NAN;
  VolADCOneCyc_max.Vca = NAN;
  VolADCOneCyc_min.Vab = NAN;
  VolADCOneCyc_min.Vbc = NAN;
  VolADCOneCyc_min.Vca = NAN;

  PwrOneCycMax.Pa = 0;
  PwrOneCycMax.Pb = 0;
  PwrOneCycMax.Pc = 0;
  PwrOneCycMax.RPa = 0;
  PwrOneCycMax.RPb = 0;
  PwrOneCycMax.RPc = 0;
  PwrOneCycAppMax.AppPa = 0;
  PwrOneCycAppMax.AppPb = 0;
  PwrOneCycAppMax.AppPc = 0;
  PwrOneCycMin.Pa = 1E36;
  PwrOneCycMin.Pb = 1E36;
  PwrOneCycMin.Pc = 1E36;
  PwrOneCycMin.RPa = 1E36;
  PwrOneCycMin.RPb = 1E36;
  PwrOneCycMin.RPc = 1E36;
  PwrOneCycAppMin.AppPa = 1E36;
  PwrOneCycAppMin.AppPb = 1E36;
  PwrOneCycAppMin.AppPc = 1E36;

  Pwr200msecMax.Pa = 0;
  Pwr200msecMax.Pb = 0;
  Pwr200msecMax.Pc = 0;
  Pwr200msecMax.RPa = 0;
  Pwr200msecMax.RPb = 0;
  Pwr200msecMax.RPc = 0;
  Pwr200msecAppMax.AppPa = 0;
  Pwr200msecAppMax.AppPb = 0;
  Pwr200msecAppMax.AppPc = 0;
  Pwr200msecMin.Pa = 1E36;
  Pwr200msecMin.Pb = 1E36;
  Pwr200msecMin.Pc = 1E36;
  Pwr200msecMin.RPa = 1E36;
  Pwr200msecMin.RPb = 1E36;
  Pwr200msecMin.RPc = 1E36;
  Pwr200msecAppMin.AppPa = 1E36;
  Pwr200msecAppMin.AppPb = 1E36;
  Pwr200msecAppMin.AppPc = 1E36;

  for (i=0; i<10; ++i)
  {
    THDminmax[i].THDmin = 1E36;
    THDminmax[i].THDmax = 0;
    THDminmax[i].THDminTS.Time_secs = 0;
    THDminmax[i].THDminTS.Time_nsec = 0;
    THDminmax[i].THDmaxTS.Time_secs = 0;
    THDminmax[i].THDmaxTS.Time_nsec = 0;
  }

  FreezeMinMax = FALSE;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ResetMinMaxValues()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        TestInj_Handler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Handler
//
//  MECHANICS:          This subroutine handles test injection.  Three flags in TestInj.Flags control the
//                      test injection process:
//                          TEST_INJ_ON:
//                              True - test injection is active
//                              False - test injection is not active
//                          TEST_INJ_INIT_ON:
//                              True - initialize the test injection variables and hardware (muxes and DAC)
//                              False - do not perform initialization
//                          TEST_INJ_INIT_OFF:
//                              True - turn off test injection
//                              False - output the test injection sample
//                      The test injection signal is driven by the microprocessor's DAC.  The DAC output
//                      code consists of two parts:
//                          TestInj.Amplitude: the amplitude of a cosine waveform
//                          TestInj.MidPoint: a fixed dc offset
//                      The amplitude is multiplied by a cosine factor, SIN_COEFF[TestInj.CosIndx].  This
//                      cosine factor is updated each sample time and is equal to
//                      Sin(360deg/80 * TestInj.CosIndx).  Note, TestInj.CosIndx is initialized to 20, so
//                      that the Sin value is offset by +90 degrees, which is equal to the cosine.
//                      When lower test currents are desired, TestInj.Amplitude is nonzero and
//                      TestInj.MidPoint is equal to the midpoint of the cosine waveform - this value is
//                      obtained during calibration - it is the point where the dc offset is zero.
//                      When higher currents are desired, TestInj.Amplitude is zero and TestInj.MidPoint is
//                      a value greater than the calibration value.  The value is proportional to the
//                      requested current and additional calibration (conversion) values.  In this case, the
//                      DAC output is a fixed dc signal.  The integrator gain is significantly higher for a
//                      dc input and so higher currents may be obtained.
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//                      
//  CAVEATS:            This should only be active if the primary currents are low!!
//                      TEST_INJ_ON must be the last flag set in the main loop (set the other flags before
//                      this one to ensure there are no conflicts with the interrupt) when turning on test
//                      injection!!
//
//
//  INPUTS:             TestInj.x, SIN_COEFF[]
//
//  OUTPUTS:            None
//
//  ALTERS:             TestInj.Flags
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void TestInj_Handler(void)
{
  // - If test injection is on (active), there are three states:
  //      - State 0: Initialization (TEST_INJ_INIT_ON is set, TEST_INJ_INIT_OFF is clear)
  //          - Turn on the DAC
  //          - Enable the proper channel for the test injection signal
  //          - Change the integrator constant for the appropriate channel.  Note, this will affect the seed
  //            for the next sample, which is what we want
  //          - Clear flag for initialization on (TEST_INJ_INIT_ON)
  //          - Write the DAC value and increment the index
  //      - State 1: Outputting the test inj signal (TEST_INJ_INIT_OFF is clear)
  //          - Write the DAC value and increment the index
  //      - State 2: Turn off (TEST_INJ_INIT_OFF is set)
  //          - Turn off the DAC
  //          - Disable all of the channels for the test injection signal
  //          - Change all integrator constants back to normal.  Note, this will affect the seed for the
  //            next sample, which is what we want
  //          - Clear all test injection flags, including the active flag.  This turns off test injection
  if (TestInj.Flags & TEST_INJ_ON)
  {
    if (TestInj.Flags & TEST_INJ_INIT_ON)
    {
      DAC->CR |= 0x00000001;                    // Turn on the DAC
      if (TestInj.Channel == 0)                 // Turn on the test channel
      {
        TSTINJ_PHA_ON;
      }
      else if (TestInj.Channel == 1)
      {
        TSTINJ_PHB_ON;
      }
      else if (TestInj.Channel == 2)
      {
        TSTINJ_PHC_ON;
      }
      else if (TestInj.Channel == 3)
      {
        TSTINJ_PHN_ON;
      }
      else
      {
        TSTINJ_GND_ON;
      }
      if (TestInj.Channel < 4)                      // For the phase currents (TestInj.Channel < 4), if the
      {                                             //   flag is set, change the integrator constant for the
        if (TestInj.Flags & TEST_INJ_CHANGE_A1)     //   corresponding phase to the low-current test
        {                                           //   injection constant
          INT_a1[TestInj.Channel] = TESTINJ_a1;
        }
        else                                        // If the flag is clear, change the constant back to
        {                                           //   normal in case it was changed previously and test
          INT_a1[TestInj.Channel] = NORMAL_a1;      //   injection is still on
        }
      }
      TSTINJ_EN;                                    // Enable test injection
      TestInj.Flags &= (~TEST_INJ_INIT_ON);     // Clear flag to initialize test injection
    }
    if (TestInj.Flags & TEST_INJ_INIT_OFF)
    {
      TSTINJ_DIS;
      DAC->DHR12R1 = 0;                         // Write 0 to the DAC
      DAC->CR &= 0xFFFFFFFE;                    // Turn off the DAC
      TSTINJ_PHA_OFF;                           // Turn off the test channels
      TSTINJ_PHB_OFF;
      TSTINJ_PHC_OFF;
      TSTINJ_PHN_OFF;
      TSTINJ_GND_OFF;
      INT_a1[0] = NORMAL_a1;                    // Change all integrator constants back to normal
      INT_a1[1] = NORMAL_a1;
      INT_a1[2] = NORMAL_a1;
      INT_a1[3] = NORMAL_a1;
      TestInj.Flags = 0;                        // Clear all test injection flags
//      Prot_Timer = 25;                       // *** DAH  UNCOMMENT THIS FOR DESKTOP TESTING WITH TEST INJECTION     *** DAH 210714
                                                //          THIS TURNS PROTECTION BACK ON 250MSEC AFTER THE TEST CURRENTS ARE TURNED OFF
    }
    else
    {
      DAC->DHR12R1 = (uint16_t)((SIN_COEFF[TestInj.CosIndx++] * TestInj.Amplitude) + TestInj.MidPoint);
      if (TestInj.CosIndx >= 80)
      {
        TestInj.CosIndx = 0;
      }
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          TestInj_Handler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        SysTick_Handler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           System Timer Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the SysTick (system timer) interrupt.  It
//                      occurs every 10msec, and updates the following timer/counters:
//                          StatusLED_BlinkCtr - decrements if nonzero, used to blink the Status LED
//                          HighLoadLED_BlinkCtr - decrements  if nonzero, used to blink the High Load LED
//                          Anniv_5minCtr - decrements if nonzero, used to initiate diagnostics
//                          ModB.Comm_Timer - watchdog timer for the Modbus
//                          Prot_Timer - keeps protection off for ~250msec after TA is fired
//                          HW_SecInjTest.TestTimeoutTimer - decrements if nozero, used to time out the test if
//                                                           we don't trip in 2 hours
//                          FW_SimulatedTest.TestTimeoutTimer - decrements if nozero, used to time out the test if
//                                                           we don't trip in 2 hours
//
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            StatusLED_BlinkCtr, HarmReq, ModB.Comm_Timer, SD_ProtOn, HW_SecInjTest.TestTimeoutTimer,
//                      FW_SimulatedTest.TestTimeoutTimer
//
//  ALTERS:             Prot_Timer
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void SysTick_Handler(void)
{
  uint8_t i;

  if (StatusLED_BlinkCtr > 0)
  {
    --StatusLED_BlinkCtr;
  }
  if ((HlAlm2Flg == 1) && (HighLoadLED_BlinkCtr > 0))
  {
    --HighLoadLED_BlinkCtr;
  }
  if (Anniv_5minCtr > 0)
  {
    --Anniv_5minCtr;
  }
  else
  {
//    Anniv_5minCtr = COUNT_5MIN;
    Anniv_5minCtr = COUNT_5MIN/5;           // *** DAH MAKE 1 MINUTE FOR NOW FOR TESTING
    min5Anniv = TRUE;
  }
  if (++SysTickTime.cnt_10msec > 99)
  {
    SysTickTime.cnt_10msec = 0;
    ++SysTickTime.cnt_sec;
  }

  if (HarmonicsTimer > 0)                   // Set flag for harmonics computation every 600msec
  {
    --HarmonicsTimer;
  }
  else
  {
    HarmReq = TRUE;
    HarmonicsTimer = 60;
  }

  if (ModB.Comm_Timer > 0)
  {
    --ModB.Comm_Timer;
  }

  if (FreqLine.OvrTimer < MAX_PERIOD)
  {
    ++FreqLine.OvrTimer;
  }
  if (FreqLoad.OvrTimer < MAX_PERIOD)
  {
    ++FreqLoad.OvrTimer;
  }

  for (i=0; i<NUM_RTD_TIMESLICES; ++i)
  {
    if (DPComm.RTD_XmitTimer[i] > 0)
    {
      --DPComm.RTD_XmitTimer[i];
    }
  }
  if (DPComm.XmitWaitTimer > 0)
  {
    --DPComm.XmitWaitTimer;
  }

  if (Prot_Timer > 0)
  {
    if (--Prot_Timer == 0)
    {
      Prot_Enabled = TRUE;                  
    }
  }

  if (AlarmHoldOffTmr > 0)
  {
    --AlarmHoldOffTmr;
  }

  if (Admin_Verified_Tmr > 0)
  {
    --Admin_Verified_Tmr;
  }

  if (User_Verified_Tmr > 0)
  {
    --User_Verified_Tmr;
  }

  if (Pswd_Rejection_Tmr > 0)
  {
    --Pswd_Rejection_Tmr;
  }
  
  if ((FW_SimulatedTest.Enable == 1) && (FW_SimulatedTest.TestTimeoutTimer > 0))
  {
    --FW_SimulatedTest.TestTimeoutTimer;
  }  
  
  if ((HW_SecInjTest.Enable == 1) && (HW_SecInjTest.TestTimeoutTimer > 0))
  {
    --HW_SecInjTest.TestTimeoutTimer;
  }   

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        SysTick_Handler()
//------------------------------------------------------------------------------------------------------------


//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        UART5_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART5 (Test Port) Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the UART5 (test port) interrupt.  It handles
//                      transmissions and receptions over the test port as follows:
//                          - Check whether a character has been received without error.  If so, store the
//                            character in the Rx buffer.  If not, continue.  Don't store the character if
//                            there is a reception error (Overrun, Framing, Parity, or Noise).  There is no
//                            need to set a flag.  Dropping the character will foul the message anyway
//                          - If the transmitter is empty, load another character to transmit if there are
//                            still characters to transmit.  If there are no more characters to transmit,
//                            disable the tranmitter empty interrupt.
//                      Note, the transmit complete interrupt is not used.  The TC flag is not affected in
//                      this interrupt handler.  It is checked in the foreground routine to determine
//                      whether transmissions have been completed, then cleared.
//
//  CAVEATS:            None
// 
//  INPUTS:             TP.Status, TP.StrPtr, TP.TxValBuf[], TP.TxValNdx, TP.NumChars
// 
//  OUTPUTS:            TP.RxBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxIn, TP.StrPtr, TP.TxValNdx
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void UART5_IRQHandler(void)
{
  uint16_t tmp_sr, tmp_dr;
  
  // Read the status and data registers into temporaries.  Aside from holding the data, this sequence:
  //   - clears the RXNE (read data register not empty) flag in the SR register
  //   - clears the ORE (overrun), NF (noise detected), FE (framing), and PE (parity) error flags in the SR
  //     register
  tmp_sr = UART5->SR;                   // Read the status register
  tmp_dr = UART5->DR;                   // Read the data register
  // If we have received a character without error, store it in the buffer
  if ((tmp_sr & USART_SR_RXNE) && (!(tmp_sr & (USART_SR_PE + USART_SR_FE + USART_SR_ORE))) )
  {
    TP.RxBuf[TP.RxNdxIn] = tmp_dr;
    TP.RxNdxIn = (TP.RxNdxIn + 1) & 0x1F;
  }
  // If the transmitter is empty, check flags to see whether we are transmitting and transmit the next
  //   character from either Flash or RAM
  if (tmp_sr & USART_SR_TXE)
  {
    // If transmitting a string from Flash (constants), transmit the next char and advance the pointer.  If
    //   it is a null, the end of the string has been reached, so end the transmission
    if (TP.Status & TP_TX_STRING)
    {
      UART5->DR = *TP.StrPtr++;                 // Transmit char and advance string pointer to the next char
      if (*TP.StrPtr == 0)                      // If that was the last character in the string, we're done
      {                                         //   transmitting.  Disable transmitter empty interrupts and
        UART5->CR1 &= (~(USART_CR1_TXEIE));     //   clear transmitting flags
        TP.Status &= (~(TP_TX_STRING + TP_TX_VALUE));
      }
    }
    // Otherwise if transmitting a string from RAM (values), transmit the next char and increment the index.
    //   If the index equals the number of chars to send, we are done
    else if (TP.Status & TP_TX_VALUE)
    {
      UART5->DR = TP.TxValBuf[TP.TxValNdx++];   // Output char and advance the buffer index
      if (TP.TxValNdx == TP.NumChars)           // If that was the last character, we're done
      {                                         //   transmitting.  Disable transmitter empty interrupts and
        UART5->CR1 &= (~(USART_CR1_TXEIE));     //   clear transmitting flags
        TP.Status &= (~(TP_TX_STRING + TP_TX_VALUE));
      }
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        UART5_IRQHandler()
//------------------------------------------------------------------------------------------------------------


//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        EXTI9_5_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           External Inputs 9..5 Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to whenever there is a falling edge on PB8 (AFE) or PH9
//                      (Time Sync).  Reference Init_InterruptStruct() in Init.c.
//                      The PB8 pin is driven by the DRDY output of the AFE whenever the AFE has new data;
//                      at 4800 samples/sec, an interrupt occurs every 208.33usec.
//                      This subroutine does the following:
//                          - Clear the interrupt
//                          - Activate the AFE chip select to prepare for SPI exchanges
//                          - Enable DMA streams.  We are only interested in the received data, but the
//                            transmit stream is used to drive the exchange.  Both streams are already
//                            initialized to read the 8 double-words of data.
//                      The PH9 pin is driven by the display processor whenever it is writing time into the
//                      protection processor.
//                      This subroutine does the following:
//                          - Clear the interrupt
//                          - Capture the time
//
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             AFE_CS pin (PG5), EXTI->PRNone, DMA1_Stream0->CR, DMA1_Stream7->CR
// 
//  CALLS:              Get_InternalTime
// 
//------------------------------------------------------------------------------------------------------------

void EXTI9_5_IRQHandler(void)
{
  // Check both interrupt sources (Time Sync and AFE)

  // Note: it is important to clear the pending flag before executing any other code.  Testing conducted on
  //   150825 showed that there must be a delay after the instruction to clear the flag and before exiting
  //   the ISR.  If there is not (i.e., if the instruction to clear the flag is the last instruction before
  //   exiting the routine), the ISR will occasionally be reentered immediately.  This appears to be a bug
  //   in the chip.

  if (EXTI->PR & 0x00000200)                // If time sync interrupt has occurred, process it
  {
    EXTI->PR = 0x00000200;                      // Bits are cleared by writing a one to the location
    Get_InternalTime(&IntSyncTime);             // Capture internal time to timestamp the sync toggle
  }

  if (EXTI->PR & 0x00000100)                // If AFE DRDY interrupt has occurred, process it
  {
    EXTI->PR = 0x00000100;                    // Bits are cleared by writing a one to the location

    // Check PB7 to see whether we have an override trip
    if ( ((GPIOB->IDR & 0x0080) == 0x0080) && (Prot_Enabled) )      // If TA Override active and not already tripping...
    {
      OverrideTrip();
      EXTI->PR = 0x00000080;                  // Bits are cleared by writing a one to the location
    }

    // Now process the AFE sample
    AFE_CSN_ACTIVE;                           // Activate the AFE chip select

    // Start the DMA data streams.  Stream 0 for Rx, Stream 7 for Tx.  Received words are stored in
    //   AFE_single_capture[0..15].
    // Must clear all event flags before initiating a DMA operation
    // Note, we don't need to reload the NDTR value.  According to AN4031, if NDTR is 0 when the Enable bit is
    //   set, it will automatically reload the last NDTR value
    DMA1->LIFCR |= (DMA_LIFCR_CTCIF0 + DMA_LIFCR_CHTIF0 + DMA_LIFCR_CTEIF0 + DMA_LIFCR_CDMEIF0
                      + DMA_LIFCR_CFEIF0);
    DMA1->HIFCR |= (DMA_HIFCR_CTCIF7 + DMA_HIFCR_CHTIF7 + DMA_HIFCR_CTEIF7 + DMA_HIFCR_CDMEIF7
                          + DMA_HIFCR_CFEIF7);
    DMA1_Stream0->CR |= 0x00000001;
    DMA1_Stream7->CR |= 0x00000001;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        EXTI9_5_IRQHandler()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//                                        AFE-ADC Sampling Timing
//------------------------------------------------------------------------------------------------------------
//
//    (Earliest)                                                                                    (Latest)
//       [0]           [1]            [2]          [3]           [4]           [5]
//        |<--1 sample->|           
//        +------+------+------+------+------+------+------+------+------+------+------------------> t
//        |      |      |      |      |      |      |      |      |      |      |
//    AFE[-2.5]  |  AFE[-1.5]  |  AFE[-0.5]  |  AFE[0.5]   |  AFE[1.5]   |  AFE[2.5]
//        |      |      |      |      |      |      |      |      |      |      |
//        |  ADC[0.5]   |  ADC[1.5]   |  ADC[2.5]   |  ADC[3.5]   |  ADC[4.5]   |
//        |      |      |      |      |             |             |             +-> This sample is used in
//        |      |      |      |      |             |             |                   the integrator and is
//        |      |      |      |      |             |             |                   the first sample used
//        |      |      |      |      |             |             |                   for metering
//        |      |      |      |      |             |             |
//        |      |      |      |      |             |             +--> This sample aligns with ADC[1.5].  It
//        |      |      |      |      |             |                    is used in the integrator only if
//        |      |      |      |      |             |                     ADC[0.5] was the seed value.  It
//        |      |      |      |      |             |                     is not used for metering.
//        |      |      |      |      |             |
//        |      |      |      |      |             +--> This sample aligns with ADC[0.5].  It is not used
//        |      |      |      |      |                    in the integrator and is not used for metering.
//        |      |      |      |      |
//        |      |      |      |      +--> Throw out this sample since no corresponding ADC sample.
//        |      |      |      |
//        |      |      |      +----> This is the low-gain circuit seeding sample.  It may or may not be
//        |      |      |               used in the integrator, depending on whether it is valid.
//        |      |      |
//        |      |      +----> Throw out this sample since no corresponding ADC sample.
//        |      |
//        |      +----> This is the high-gain circuit seeding sample.  It is used in the integrator for
//        |               seeding if it is valid.
//        |     
//        +-------> Throw out this sample because clock may have been transitioning to 120MHz.  AFE lags the
//                    input by 2.5 sample times.  Turn on the ADC timer.
//
//------------------------------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DMA1_Stream0_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA1 Stream 0 Complete Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to whenever Stream 0 of DMA Controller 1 has completed a
//                      transaction.  This occurs whenever the 8 32-bit data values have been read from the
//                      AFE.  The subroutine does the following:
//                          - Deactivate the AFE chip select
//                          - Reset the the transfer complete interrupt flags
//                          - Call AFE_Process_Samples() to process the current samples:
//                              - Digital integration to convert from di/dt to current
//                              - Gain and offset calibration
//                              - Scaling to convert to engineering units
//                              - If out of range, overwrite with internal ADC reading
//                          - Update peak values, sine and cosine component sums of squares, voltage sums of
//                            squares, and power sums of squares
//                          - Update 1/2-cycle and 1-cycle current sums of squares, depending on the state.
//                            The state is actually a function of how many samples have been read.
//                              - State 0:  Wait for 120MHz
//                                  - When 120MHz is established:
//                                      - Enable Timer 3 which triggers A/D conversions
//                                      - Initialize sampling variables
//                                      - Advance the state
//                              - State 1:  Discard Sample
//                              - State 2:  Discard Sample
//                                  - Discard the AFE samples because they are not aligned to any ADC
//                                    samples (they are 2 samples early)
//                                  - Advance the state
//                              - State 3:  Process Sample for Integrator
//                              - State 4:  Process Sample for Integrator
//                                  - This sample will be used with the high-gain ADC sample seed, if it is
//                                    valid (State 3), or the low-gain sample seed otherwise (State 4).
//                                    These samples are not used in any further metering/sampling
//                                    operations - they are only used for seeding.  If the high-gain sample
//                                    is used for seeding, the next sample is integrated, but is still not 
//                                    used in metering, so that the state machine is consistent.
//                              - State 5: Samples 1 - 39
//                                  - Store samples and squares of samples in buffers
//                                  - Update sin, cos, power, energy, etc. values
//                                  - Update 1/2-cycle sums of squares without removing oldest samples
//                                  - Update 1-cycle sums of squares without removing oldest samples
//                                  - Cannot calculate current or run protection
//                              - State 6: Sample 40
//                                  - Store samples and squares of samples in buffers
//                                  - Update sin, cos, power, energy, etc. values
//                                  - Update 1/2-cycle sums of squares without removing oldest samples
//                                  - Update 1-cycle sums of squares without removing oldest samples
//                                  - Find maximum 1/2-cycle sum of squares
//                                  - Call instantaneous and short-delay protection
//                              - State 7: Samples 41 - 79
//                                  - Store samples and squares of samples in buffers
//                                  - Update sin, cos, power, energy, etc. values
//                                  - Update 1/2-cycle sums of squares with removing oldest samples
//                                  - Update 1-cycle sums of squares without removing oldest samples
//                                  - Find maximum 1/2-cycle sum of squares
//                                  - Call instantaneous and short-delay protection
//                              - State 8: Sample 80
//                                  - Store samples and squares of samples in buffers
//                                  - Update sine, cos, power, energy, etc. values
//                                  - Update 1/2-cycle sums of squares with removing oldest samples
//                                  - Update 1-cycle sums of squares without removing oldest samples
//                                  - Find maximum 1/2-cycle sum of squares
//                                  - Find maximum 1-cycle sum of squares
//                                  - Call instantaneous and short-delay protection
//                              - State 9: Samples 81 and greater
//                                  - Store samples and squares of samples in buffers
//                                  - Update sine, cos, power, energy, etc. values
//                                  - Update 1/2-cycle sums of squares with removing oldest samples
//                                  - Update 1-cycle sums of squares with removing oldest samples
//                                  - Find maximum 1/2-cycle sum of squares
//                                  - Find maximum 1-cycle sum of squares
//                                  - Call instantaneous and short-delay protection
//                          - If one-cycle anniversary, save the sums of squares and reset the peak, sine
//                            and cosine, voltage, and power sum of square buffers
//                          - If one-sec anniversary, save the sums of squares and reset the sum of square
//                            buffers
//                          - Update the user waveform sample buffers with the new samples and inc the index
//                          - Update the sample state
//                          - Note, the 1/2-cycle and 1-cycle buffers hold the square of the samples as
//                            integers.  The 1-sec and user waveform buffers hold the samples as floats.
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[]
// 
//  OUTPUTS:            CurHalfCycSOS_SumF.Ix, CurHalfCycSOSmax, CurOneCycSOSmax, NewSample,
//                      AFE_SampleState, msec200Anniv, OneCycAnniv, CurOneCycPeak.Ix, CurOneCycSOS_Sav.Ix,
//                      CurVol200msNoFltrSinSOS_Sav[], CurVol200msNoFltrCosSOS_Sav.Ix,
//                      VolAFEOneCycSOS_Sav.Vxx, PwrOneCycSOS_Sav.Px, PwrOneCycSOS_Sav.RPx,
//                      Cur200msecSOS_Sav.Ix, VolAFE200msFltrSOS_Sav.Vxx, VolADC200msSOS_Sav.Vxx,
//                      Pwr200msecSOS_Sav.Px, Pwr200msecSOS_Sav.RPx
//
//  ALTERS:             AFE_CS pin (PG5), DMA1->HIFCR, DMA1->LIFCR
//                      CurHalfCycSOS_SumI.Ix, AFE_SampleState, HalfCycInd, OneCycInd, msec200Ctr,
//                      CurOneCycSOS_SumF.Ix, CurOneCycSOS_SumI.Ix, Cur200msFltrSOS_Sum.Ix,
//                      Cur200msNoFltrSinSOS_Sum.Ix, Cur200msNoFltrCosSOS_Sum.Ix, VolAFEOneCycSOS_Sum.Vxx,
//                      VolAFE200msFltrSOS_Sum.Vxx, VolADCOneCycSOS_Sum.Vxx, VolADC200msSOS_Sum.Vxx,
//                      PwrOneCycSOS_Sum.Px, PwrOneCycSOS_Sum.RPx, Pwr200msecSOS_Sum.Px,
//                      Pwr200msecSOS_Sum.RPx, HalfCycSamplesSq[][], OneCycSamplesSq[][]
// 
//  CALLS:              AFE_Process_Samples(), Instantaneous_Prot(), ShortDelay_Prot()
//
//  EXECUTION TIME:     Measured execution time on 160718 (Rev 00.15 code).
//                                37.8usec (with instantaneous and short-delay protection and both CAMs
//                                          configured as sampled-value CAMs)
//                      Measured execution time on 170213 (Rev 00.20 code).
//                                39.5usec (with instantaneous and short-delay protection and both CAMs
//                                          configured as sampled-value CAMs, intrinsic subroutines, line
//                                          voltages added)
//                      Measured execution time on 230303 (Rev 00.70 code).
//                                47usec (with instantaneous and short-delay protection and both CAMs
//                                          configured as GOOSE CAMs, no GOOSE code running!)
//                      Measured execution time on 230518 (Rev 00.xx code).
//                                ~60usec (with instantaneous and short-delay protection and GOOSE code running!)
//------------------------------------------------------------------------------------------------------------

void DMA1_Stream0_IRQHandler(void)
{
  uint8_t i;

// TESTPIN_D1_HIGH;                         // *** DAH TEST CODE FOR TIMING

  /*-------------------------- BP using one-cycle currents  -------
  // Capture the coil temperature sample if it is enabled
  if (TestInj.Flags & TEMP_MEAS_ON)
  {
    if ( (ADC3->SR & ADC_SR_EOC) == ADC_SR_EOC )
    {
      coil_temp_samples[coil_temp_index++] = ((uint16_t)ADC3->DR & 0x0FFF);
      if (coil_temp_index > 199)
      {
        coil_temp_index = 0;
      }
    }
  }
  -------------------------------------------------------------*/

  // Miscellaneous tasks performed each interrupt
  TestInj_Handler();                    // Test injection
  ReadSwitches(FALSE);                  // Read switches
  
  if (ZIN == 1)                         // update ZSI flag based on ZSI input
  {
    Zin_Latched = 1;
  }
  
  AFE_CSN_INACTIVE;                     // Deactivate the AFE chip select
//  TESTPIN_D1_LOW;                         // *** DAH TEST CODE FOR TIMING

  DMA1->HIFCR |=  DMA_HISR_TCIF7;       // Reset the DMA transfer complete interrupt flags
  DMA1->LIFCR |=  DMA_LISR_TCIF0;

  switch (AFE_SampleState)
  {
    case 0:                             // Wait for 120MHz
      if (SysClk_120MHz == TRUE)            // If we are operating at 120MHz...
      {
        // Turn on the ADC timer.  It has already been initialized with the half sample time for the first
        //   interrupt.  When it reaches the compare value, the counter will reset to zero.  All subsequent
        //   interrupts will be one sample time.  This delays the ADC samples by one half-sample time.  The
        //   ADC sample processing subroutine, ADC_Process_Samples(), delays the samples by another 2 sample
        //   times.  This compensates for the 2.5 sample delay in the AFE samples.
        TIM3->CR1 |= 0x0001;
        // Initialize the sums of squares and currents.  This is done here rather than in Intr_VarInit() to
        //   reduce the initialization time.
        AFEISR_VarInit();
        ++AFE_SampleState;
      }
      break;

    case 1:                             // Discard Sample
    case 2:                             // Discard Sample
      // Discard these two samples because they do not have a corresponding ADC sample, and we need to seed
      //   the integrator with an ADC sample that corresponds to the AFE sample.
      ++AFE_SampleState;
      break;

    case 3:                             // Process Sample for Integrator Only
    case 4:                             // Process Sample for Integrator Only
      // Call subroutine to process the AFE sampled data.  This sample aligns with the high-gain ADC seed
      //   value (case 3).  This sample is used for seeding if it is valid.  Otherwise the low-gain value
      //   (case 4) is used for seeding.  Neither sample is used in any metering calculations.
      AFE_Process_Samples();
      ++AFE_SampleState;
      break;

    default:                            // States 5 thru 9: Process Sample
      // Call subroutine to process the AFE sampled data.  The seeding has already been completed.  This is
      //   the first sample used in metering computations.  This ensures that the samples from all of the
      //   channels are aligned, regardless of whether different (low-gain versus high-gain) seeds were used
      //   by the different channels.
      AFE_Process_Samples();

      // Compute the unfiltered (Igres) and filtered (Igres_mtr) residual grounds for protection
      Igres = AFE_new_samples[0] + AFE_new_samples[1] + AFE_new_samples[2] + AFE_new_samples[3];
      Igres_mtr = MTR_new_samples[0] + MTR_new_samples[1] + MTR_new_samples[2] + MTR_new_samples[3];
      Igres_Protsample = Igres;

      // Store samples in the RAM buffer (SampleBuf[])
      //      SampleBuf[SampleIndex].xxxx <--- AFE_new_samples[x]
      // Note, the buffer index (SampleIndex) is not incremented until after the DMA is initiated to
      //   transmit the samples to the CAMs so that the memory address is correct for the DMA operation
      buffer_samples();

      // Transmit the sampled values out over the CAM ports, for sampled-value CAMs
      if (CAM1.Status & CAM_TYPE_SAMPLE)      // If CAM type is sampled-value, transmit the samples
      {
        if ( !(DMA2->HISR & DMA_HISR_TCIF7)           // If the previous DMA is not complete and this is not
          && !(CAM1.Status & CAM_FIRST_SAMPLE) )      //   the first sample, there is an error.  Log the
        {                                             //   error and reinitialize the DMA stream.
          // *** DAH   ADD CODE TO LOG SOME SORT OF ERROR
          CAM1.Status |= CAM_ERROR;
          Init_CAM1_DMA_Streams();
        }
        DMA2_Stream7->M0AR = (uint32_t)((uint8_t *)(&SampleBuf[SampleIndex].Ia));
        DMA2->HIFCR |= DMA_HISR_TCIF7;                // Reset the transfer complete interrupt flag
        // Must clear all event flags before initiating a DMA operation
        DMA2->HIFCR |= (DMA_HIFCR_CTCIF7 + DMA_HIFCR_CHTIF7 + DMA_HIFCR_CTEIF7 + DMA_HIFCR_CDMEIF7
                              + DMA_HIFCR_CFEIF7);
        DMA2_Stream7->CR |= 0x00000001;               // Initiate the DMA to transmit the data to CAM1
        CAM1.Status &= (~CAM_FIRST_SAMPLE);
      }
      if (CAM2.Status & CAM_TYPE_SAMPLE)      // If CAM type is sampled-value, transmit the samples
      {
        if ( !(DMA2->HISR & DMA_HISR_TCIF6)           // If the previous DMA is not complete and this is not
          && !(CAM2.Status & CAM_FIRST_SAMPLE) )      //   the first sample, there is an error.  Log the
        {                                             //   error and reinitialize the DMA stream.
          // *** DAH   ADD CODE TO LOG SOME SORT OF ERROR
          CAM2.Status |= CAM_ERROR;
          Init_CAM2_DMA_Streams();
        }
        DMA2_Stream6->M0AR = (uint32_t)((uint8_t *)(&SampleBuf[SampleIndex].Ia));
        DMA2->HIFCR |= DMA_HISR_TCIF6;                // Reset the transfer complete interrupt flag
        // Must clear all event flags before initiating a DMA operation
        DMA2->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                              + DMA_HIFCR_CFEIF6);
        DMA2_Stream6->CR |= 0x00000001;               // Initiate the DMA to transmit the data to CAM2
        CAM2.Status &= (~CAM_FIRST_SAMPLE);
      }

      // ------------------------------------- Trip Waveform Captures -------------------------------------
      //
      // Basic Trip waveform capture process (also applies to alarm and extended capture):
      //   Protection function:
      //     sets Trip_WF_Capture.Req
      //   This ISR (because Trip_WF_Capture.Req is set):
      //     sets Trip_WF_Capture.InProg
      //     computes SampleStartIndex
      //     computes timestamp of first sample
      //   ManageSPI1Flags() (because Trip_WF_Capture.InProg is set):
      //     sets S1F_TRIP_WF_WR in SPI1Flash.Req
      //   EventManager() (because Trip_WF_Capture.Req and Trip_WF_Capture.InProg are set):
      //     captures EID for waveform
      //     clears Trip_WF_Capture.Req
      //     waits for waveform capture to complete
      //   SPI1_Flash_Manager() (because S1F_TRIP_WF_WR in SPI1Flash.Req  is set):
      //     stores the waveform
      //     sets S1F_TRIP_WF_WR in SPI1Flash.Ack when done
      //   EventManager() (because S1F_TRIP_WF_WR in SPI1Flash.Ack  is set):
      //     clears S1F_TRIP_WF_WR in SPI1Flash.Req
      //     completes the waveform capture (writes header info)
      //   SPI1_Flash_Manager() (because S1F_TRIP_WF_WR in SPI1Flash.Req is clear):
      //     clears S1F_TRIP_WF_WR in SPI1Flash.Ack
      //   
      // Handle trip waveform capture before the sample index is incremented.  The trip waveform capture is
      // used when most trips occur.  The following waveforms are captured:
      //    Ia, Ib, Ic, In, Igsrc, Igres, VanAFE, VbnAFE, VcnAFE, VanADC, VbnACE, VcnADC
      // Thirty-six cycles are captured and written into Flash memory.  To save memory space, a separate
      // buffer is NOT used to hold the samples.  SampleBuf[] is used.  This buffer is also used for Alarm
      // and Extended-Capture waveform captures.  The Trip waveform captures are the highest priority and
      // will abort any other capture (only one captured waveform may be written into Flash at a time).
      // SampleBuf[] holds 39 cycles.  When a Trip capture is requested, we only need to capture the present
      // index of SampleBuf[], SampleIndex. The starting index for the captured waveform is then the
      // present index + the number of post-event cycles + 3 (39 - 36).  The extra 3 cycles (about 50msec
      // at 60Hz) allow us to finish any existing Flash access operations, such as a demand write and sector
      // erase.
      // Timing analysis:
      // There are two concerns:
      //   1) In the case of the minimum number of post-event cycles, the incoming samples (indexed by
      //      SampleIndex) do not overwrite samples that have yet to be written (indexed by TripWF_Index):
      //      SampleIndex does not catch up to SBout_Index
      //      The worst case condition for 1) is when the incoming samples are arriving as quickly as
      //      possible - this is at 60Hz (4800 sampling rate).  Samples arrive every 208.33usec.  Seven
      //      sample packets are stored in each page of Flash.  A waveform capture takes 412 pages of
      //      storage.  Thus, 7 sample packets arrive every 1.458msec (7 x 208.33usec).  In theory, seven
      //      sample packets are written to Flash every 1.5msec.  However, timing tests conducted on 180615
      //      showed that it takes 709msec to write all of the pages to Flash.  SBout_Index must start out
      //      far enough in front of SampleIndex so that SampleIndex cannot catch up.  To expand on this:
      //      Timing tests were conducted on 180615:
      //        A waveform captures takes 412 pages of storage.
      //        It takes 709msec to write all of the pages to Flash.
      //        Samples arrive at the rate of 16.67msec per cycle.  It takes 600msec to receive 36 cycles.
      //        We need a "head-start" of 709msec - 600msec = 109msec, or 6.54 cycles
      //        Test code was written that tracked the incoming and outgoing samples.  With a head start of
      //          7 cycles, the outgoing sample counter remained 50 samples (.625 cycles ahead).
      //        In addition to the 6.54 cycle head start, we have to allow for two Flash block erases to
      //          prepare for the next Trip waveform capture.  This takes 50msec, or 3 cycles.  Therefore,
      //          the minimum allowable head start is 3 + 7 = 10 cycles.
      //        The RAM buffer handles 39 cycles (rather than 36), so we can spec a minimum of 7 post-alarm
      //          cycles.
      //      The 7 post-event cycles give us enough of a head start to allow us to get the 36 cycles
      //      written before we overwrite the earliest cycle in SampleBuf[].  We will spec a minimum of 8
      //      post-event cycles.  (The number of post-event cycles is equal to the head-start, since these
      //      locations are stored in Flash after being written with new, post-event samples).
      //
      //      Conducted tests on 180914: Ran code with 8 post-event cycles and checked SampleIndex versus
      //      SBout_Index at the end of the writes to Flash.  SBout_Index was consistently ~4.6cycles (368
      //      samples) in front of SampleIndex.  This does not include the 3 cycles used if two block erases
      //      are required, so the results are as expected.
      //      Repeated the tests only this time manually set the two alarm block erase bits.  SBout_Index
      //      was consistently ~2cycles (168 samples) in front of SampleIndex.  This is better than
      //      expected, because the block erase takes less than 25msec to execute (measured at about 22msec)
      //
      //   2) In the case of the maximum number of post-event cycles, samples are not written before the new
      //      (post-event) samples are captured: SBout_Index does not catch up to SampleIndex
      //      The worst-case condition for 2) is when the incoming samples are arriving as slowly as
      //      possible - this is at 50Hz (4000 sampling rate).  Samples arrive every 250usec.  Seven sample
      //      packets arrive every 1.75msec.  For worst-case conditions, we will assume the samples are
      //      written as fast as possible, or every 1.5msec.  We must make sure that SBout_Index does not
      //      start out so far in front of SampleIndex that it can catch up to it.
      //        A waveform captures takes 412 pages of storage.
      //        It takes 618msec minimum to write all of the pages to Flash (1.5msec x 412)
      //        Samples arrive at the rate of 20msec per cycle.  It takes 720msec to receive 36 cycles.
      //        720msec - 618msec = 102msec, or 5.1 cycles.  For worst-case conditions, we will assume there
      //        are no pending Flash block erase cycles, so that the samples will immediately begin to be
      //        written to Flash.  We will spec a maximum of 28 post-alarm cycles.  This leaves an 8-cycle
      //        gap between SBout_Index and SampleIndex.  This provides a 2.9-cycle margin.
      //
      //      Conducted tests on 180914: Ran code with 28 post-event cycles and checked SampleIndex versus
      //      SBout_Index at the end of the writes to Flash.  SampleIndex was consistently ~7.3 cycles (587
      //      samples) in front of SBout_Index.  This better than expected, because the actual time to write
      //      all of the pages to Flash is actually 709msec, not 618msec.  The extra 91msec (approximately
      //      4.5 cycles at 50Hz) accounts for the 7.3cycle margin that was seen (2.9 + 4.5 = 7.4 cycles). 
      //      
      if (Trip_WF_Capture.Req && !Trip_WF_Capture.InProg)     // If there is a new Trip capture request...
      {
        Trip_WF_Capture.InProg = TRUE;                            // Set the capture in progress flag
        // Compute the starting index of the first sample.  This is 3 cycles (buffer is 39 cycles long and
        //   we are saving 36 cycles) plus the number of post event cycles after the present sample
        // The number of post-event cycles is (36 - Setpoints0.stp.WFM_TripPreCyc), where
        // 8 <= Setpoints0.stp.WFM_TripPreCyc <= 28
        Trip_WF_Capture.SampleStartIndex = SampleIndex + ((3 + 36 - Setpoints0.stp.WFM_TripPreCyc) * 80);
        if (Trip_WF_Capture.SampleStartIndex >= TOTAL_SAMPLE_SETS)
        {
          Trip_WF_Capture.SampleStartIndex -=  TOTAL_SAMPLE_SETS;
        }
        // Compute and store the time of the first sample.  Shouldn't need to disable interrupts since the
        //   sampling interrupt is pretty much the highest-priority interrupt
        // The offset amount, Trip_WF_OffsetTime, is computed in Gen_Values() based on
        //   Setpoints0.stp.WFM_TripPreCyc.  It is computed once in Gen_Values(), rather than computing it
        //   here, to save execution time in this ISR
        Get_InternalTime(&Trip_WF_Capture.TS);                      // Retrieve time of the present sample
        if (Trip_WF_OffsetTime > Trip_WF_Capture.TS.Time_nsec)      // Subtract the offset time from the
        {                                                           //   present time
          if (Trip_WF_Capture.TS.Time_secs > 0)
          {
            Trip_WF_Capture.TS.Time_nsec += (1000000000 - Trip_WF_OffsetTime);
            Trip_WF_Capture.TS.Time_secs--;
          }
          else
          {
            Trip_WF_Capture.TS.Time_nsec = 0;
            Trip_WF_Capture.TS.Time_secs = 0;
          }
        }
        else
        {
          Trip_WF_Capture.TS.Time_nsec -= Trip_WF_OffsetTime;
        }
      }

      //
      // --------------------------------- End of Trip Waveform Captures ----------------------------------



      // ------------------------------------- Alarm Waveform Captures -------------------------------------
      //
      // Handle alarm waveform captures identically as Trip waveforms (above).  However, since Trip waveform
      // captures are a higher priority, do not service an Alarm waveform request if a Trip capture is
      // requested.
      else if (Alarm_WF_Capture.Req && !Alarm_WF_Capture.InProg)   // If there is a new Alarm capture
      {                                                            //   request...
        Alarm_WF_Capture.InProg = TRUE;   // Set the capture in progress flag
        // Compute the starting index of the first sample (see comments in Trip waveform processing)
        Alarm_WF_Capture.SampleStartIndex = SampleIndex + ((3 + 36 - Setpoints0.stp.WFM_AlarmPreCyc) * 80);
        if (Alarm_WF_Capture.SampleStartIndex >= TOTAL_SAMPLE_SETS)
        {
          Alarm_WF_Capture.SampleStartIndex -=  TOTAL_SAMPLE_SETS;
        }
        // Compute and store the time of the first sample (see comments in Trip waveform processing)
        Get_InternalTime(&Alarm_WF_Capture.TS);                     // Retrieve time of the present sample
        if (Alarm_WF_OffsetTime > Alarm_WF_Capture.TS.Time_nsec)    // Subtract the offset time from the
        {                                                           //   present time
          if (Alarm_WF_Capture.TS.Time_secs > 0)
          {
            Alarm_WF_Capture.TS.Time_nsec += (1000000000 - Alarm_WF_OffsetTime);
            Alarm_WF_Capture.TS.Time_secs--;
          }
          else
          {
            Alarm_WF_Capture.TS.Time_nsec = 0;
            Alarm_WF_Capture.TS.Time_secs = 0;
          }
        }
        else
        {
          Alarm_WF_Capture.TS.Time_nsec -= Alarm_WF_OffsetTime;
        }
      }

      //
      // --------------------------------- End of Alarm Waveform Captures ----------------------------------



      // ------------------------------- Extended Capture Waveform Captures --------------------------------
      //
      // Handle extended waveform captures identically as Trip waveforms (above).  However, since Trip and
      // Alarm waveform captures are a higher priority, do not service an Extended Capture waveform request
      // if a Trip or Alarm capture is requested
      //      
      else if (Ext_WF_Capture.Req && !Ext_WF_Capture.InProg)
      {
        Ext_WF_Capture.InProg = TRUE;      // Set the capture in progress flag
        // Compute the starting index of the first sample (see comments in Trip waveform processing)
        Ext_WF_Capture.SampleStartIndex = SampleIndex + ((3 + 36 - Setpoints0.stp.WFM_ExtCapPreCyc) * 80);
        if (Ext_WF_Capture.SampleStartIndex >= TOTAL_SAMPLE_SETS)         // *** DAH  ALSO, IF CLOSING INTO A FAULT, WE CANNOT NECESSARILY GET
        {                                                                 //   ALL OF THE PRESAMPLES, UNLESS WE SET THEM TO 0 ON POWER UP.  OTHERWISE,
          Ext_WF_Capture.SampleStartIndex -=  TOTAL_SAMPLE_SETS;          //   WE CAN ONLY GO BACK AS FAR AS THE NUMBER OF SAMPLES WE HAVE READ.
        }
        // Compute and store the time of the first sample (see comments in Trip waveform processing)
        Get_InternalTime(&Ext_WF_Capture.TS);                   // Retrieve time of the present sample
        if (Ext_WF_OffsetTime > Ext_WF_Capture.TS.Time_nsec)    // Subtract the offset time from the present
        {                                                       //   time
          if (Ext_WF_Capture.TS.Time_secs > 0)
          {
            Ext_WF_Capture.TS.Time_nsec += (1000000000 - Ext_WF_OffsetTime);
            Ext_WF_Capture.TS.Time_secs--;
          }
          else
          {
            Ext_WF_Capture.TS.Time_nsec = 0;
            Ext_WF_Capture.TS.Time_secs = 0;
          }
        }
        else
        {
          Ext_WF_Capture.TS.Time_nsec -= Ext_WF_OffsetTime;
        }
      }
      //
      // ------------------------------ End of Extended Capture Waveform Captures -------------------------------


      // Now increment SampleIndex since the DMA has been set up
      if (++SampleIndex >= TOTAL_SAMPLE_SETS)
      {
        SampleIndex = 0;
        SampleBufFilled = TRUE;
      }
      
      SampleCounter++;                 // Used by Sec Inj to time the test

      // -------------------------------- Update Peaks and Sums of Squares ---------------------------------
      //
      // Finish updating the sums of squares for the metered currents (Ia thru Igres), load voltages (Van1
      //   thru Vcn1, Vab1 thru Vca1, Van2 thru Vcn2, Vab2 thru Vca2)
      //
      update_peaks();                       // Update peak values for crest factor calculations
      update_Ia_SOS();                      // Update the Ia 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_Ib_SOS();                      // Update the Ib 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_Ic_SOS();                      // Update the Ic 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_In_SOS();                  // Update the In 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_Igsrc_SOS();               // Update the Igsrc 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_Igres_SOS();               // Update the Igres 1/2-cycle, 1-cyc, and 200msec sums of squares
      update_VlnAFE_SOS();              // Update the AFE (load side) Vln 1-cyc and 200msec sums of squares
      update_VllAFE_SOS();              // Update the AFE (load side) Vll 1-cyc and 200msec sums of squares
      update_VlnADC_SOS();              // Update the ADC (line side) Vln 1-cyc and 200msec sums of squares
      update_VllADC_SOS();              // Update the ADC (line side) Vll 1-cyc and 200msec sums of squares
      update_Power_SOS();               // Update the AFE (load side) W and VAR 1-cyc and 200msec SOS's
      //
      //
      // At this point, the following sums of squares have been updated
      //    Cur200msFltrSOS_Sum.Ix -- floating point 200msec filtered current
      //    CurHalfCycSOS_SumI.Ix -- uint64 1/2-cycle current in tenths (squared)
      //    CurOneCycSOS_SumI.Ix -- uint64 1-cycle current in tenths (squared)
      //    VolAFE200msFltrSOS_Sum.Vxx -- floating point filtered 200ms l-n and l-l load-side voltages
      //    VolAFEOneCycSOS_Sum.Vxx -- floating point 1-cycle line-neutral and line-line load-side voltages
      //    VolADC200msSOS_Sum.Vxx -- floating point 200msec line-neutral and line-line line-side voltages
      //    VolADCOneCycSOS_Sum.Vxx -- floating point 1-cyc line-neutral and line-to-line line-side voltages
      //    PwrOneCycSOS_Sum.Px -- floating point one-cycle real power
      //    Pwr200msecSOS_Sum.Px -- floating point 200msec real power
      //    PwrOneCycSOS_Sum.RPx -- floating point one-cycle reactive power
      //    Pwr200msecSOS_Sum.RPx -- floating point 200msec reactive power
      //
      // ----------------------------- End of Finish Updating Sums of Squares ------------------------------


      // Update the sine and cosine components of the fundamental for THD
//TESTPIN_D1_LOW;
      update_THD_Sin_Cos_SOS();
//TESTPIN_D1_HIGH;

     
      // If state > 6, we have received more than one 1/2-cycle's worth of samples (40 samples = 1/2 cycle),
      //   so subtract the square of the oldest 1/2-cycle sample from the sums of squares for the 1/2-cycle
      //   currents
      if (AFE_SampleState > 6)
      {
        update_I_HC_SOS();
      }
     
      // If state >= 6, we have at least 40 samples, so convert the integer 1/2-cyc sum of squares to floats
      //   and calculate the maximum 1/2-cyc sum of squares
      if (AFE_SampleState > 5)
      {
        calc_max_I_HC_SOS();
        if (!FreezeMinMax)              // Compute max and min half cycle currents for internal testing if
        {                               //   not displaying the values
          calc_min_max_I_HC_SOS();
        }
      }
     
      // If state > 8, we have received more than one 1-cycle's worth of samples (80 samples = 1 cycle), so
      //   subtract the square of the oldest 1-cycle sample from the sums of squares for the 1-cycle currents
      if (AFE_SampleState > 8)
      {
        update_I_OC_SOS();
      }
     
      // If state >= 8, we have at least 80 samples, so calculate the maximum 1-cyc sum of squares and save the
      //   corresponding new sample
      //   Note, the maximum current is used only for short delay protection, so it does not include the ground
      //   currents
      if (AFE_SampleState > 7)
      {
        calc_max_I_OC_SOS();
        if (!FreezeMinMax)              // Compute max and min one cycle currents for internal testing if
        {                               //   not displaying the values
          calc_min_max_I_OC_SOS();
        }
      }
     
      // Check for 1-cycle anniversary
      if (OneCycInd >= 79)                  // If index is 79, this is the 80th sample, and therefore the
      {                                     //   one-cycle anniversary...
        save_OC_SOS();                          // Save the one cycle values and clear the sum registers
        save_OC_peaks();                        // Save the one cyc pk currents and clear the running values
        OneCycAnniv = TRUE;                     // Set the anniversary flag
      }                                     // Don't increment the one-cycle index yet!
     
      // Check for 200msec anniversary
      if (msec200Ctr >= 959)                // If index is 959, this is the 960th sample, and therefore the
      {                                     //   200msec anniversary...
        save_200msec_SOS();
        msec200Anniv = TRUE;
      }
      
      // Check for 1 sec anniversary
      if (OneSecCtr >= 4799)                // If index is 4799, this is the 4800th sample, and therefore the
      {                                     //   1 sec anniversary...
        OneSecAnniv = TRUE;
      }

      // Move the new squares of samples into the 1/2-cycle and 1-cycle buffers
      for (i=0; i<6; ++i)
      {
        HalfCycSamplesSq[i][HalfCycInd] = ulltemp[i];
        OneCycSamplesSq[i][OneCycInd] = ulltemp[i];
      }

      // Update the delayed voltage sample buffers.  These buffers are used to delay the voltage samples by
      //   90 degrees (20 samples), for use in reactive power calculations
      update_delayed_vbuf();
     
      // Increment the buffer indices
      inc_buf_indices();

      // Run Instantaneous and Short-Delay Protection
      if (AFE_SampleState > 5)          // If there are valid 1/2-cycle currents, run protection
      {
         if ((Prot_Enabled) && (!Manufacture_Mode))      // Run protection if it is enabled
         {
           if (Setpoints1.stp.Inst_Pu > 0)       // Check if Inst protection is enabled
           {
             Instantaneous_Prot();
           }

           Short_Interlock_Out();               // Update ZSI 
           ShortDelay_Prot();                   // Run Short Delay protection

           if ((GF_Enabled) && (Setpoints1.stp.Gf_T_A_OFF == 0))  // Ground Fault trip function enable
           {
              Ground_Fault_Prot();
           }
           else if ((GF_Enabled) && (Setpoints1.stp.Gf_T_A_OFF == 1)     // Ground Fault alarm function enable
                  && (AlarmHoldOffTmr == 0) )
           {
              Ground_Fault_Alarm();                             // *** DAH  I THINK THIS NEEDS TO BE MOVED TO THE MAIN LOOP PERHAPS TO PREVENT FLAG TEARING
           }
           else                                                 // Ground Fault function disable (2 = off)
           {
//            GndTripFlg = 0;                                       // Don't clear flags, because there may
//            GndAlmFlg = 0;                                        //   be an existing trip or alarm
           }

         }

      }

      // Update the state
      if (AFE_SampleState == 5)                                // For state 5, if next sample is the 40th
      {                                                        //   one (HalfCycInd = 39), we can compute
        if (HalfCycInd >= 39)                                  //   the max 1/2-cyc sums of squares, so set
        {                                                      //   the state to 6
          AFE_SampleState = 6;
        }
      }
      else if (AFE_SampleState == 7)                           // For state 7, if next sample is the 80th
      {                                                        //   one (OneCycInd = 79), we can compute the
        if (OneCycInd >= 79)                                   //   RMS current, so set the state to 8
        {
          AFE_SampleState = 8;
        }
      }
      else if ( (AFE_SampleState == 6)                         // States 6 and 8 handle conditions for one
             || (AFE_SampleState == 8) )                       //   sample only (the 40th and 80th samples),
      {                                                        //   so automatically go to the next state
        AFE_SampleState++;
      }

      if (TA_Timer > 0)
      {
        --TA_Timer;
      }
      else
      {
        TA_TRIP_INACTIVE;
        TripReqFlg = 0;                                         // clear trip request
      }

// TESTPIN_D1_LOW;                 // *** DAH TEST CODE FOR TIMING
      break;

  }

    Get_InternalTime(&starttime1);       // *** DAH ADDED TO MEASURE 61850 COMMUNICATIONS SUBROUTINES TIME
  DispComm61850_Rx();                   // *** DAH  MAYBE MOVE THESE TO THE ADC INTERRUPT SINCE IT IS ON A TIMER AND WON'T DEPEND ON THE EXTERNAL CHIP
  DispComm61850_Tx();                   // Call 61850 GOOSE transmission processing subroutine *** DAH MAYBE HAVE RX ROUTINE BEFORE PROTECTION AND TX ROUTINE AFTER PROTECTION
                                                                        // TO MINIMIZE DELAYS RECEIVING AND TRANSMITTING ZSI MESSAGES!
    Get_InternalTime(&endtime1);         // *** DAH ADDED TO MEASURE HIGH-SPEED COMMS TIME
    looptime1 = ((endtime1.Time_secs == starttime1.Time_secs) ?
                    (endtime1.Time_nsec - starttime1.Time_nsec) :
                    ( ((endtime1.Time_secs - starttime1.Time_secs) * 1000000000) + endtime1.Time_nsec - starttime1.Time_nsec));
    if (looptime1 > maxlooptime1)         // Max loop time = maxlooptime in nanoseconds
    {
      maxlooptime1 = looptime1;
    }
// TESTPIN_A3_HIGH;                       // *** DAH TEST CODE
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        DMA1_Stream0_IRQHandler()
//------------------------------------------------------------------------------------------------------------





//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DMA2_Stream0_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DMA2 Stream 0 Complete Interrupt Service Routine
//
//  MECHANICS:          This subroutine is branched to whenever Stream 0 of DMA Controller 2 has completed a
//                      transaction.  This occurs whenever the 12 16-bit data values have been read from the
//                      internal ADCs (ADC1 and ADC2).  The subroutine reinitializes DMA 2 Stream 0 for the 
//                      next sample, and calls ADC_Process_Samples() to process the readings.
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             DMA2->LIFCR, DMA2_Stream0->CR
//
//  CALLS:              ADC_Process_Samples()
//
//  EXECUTION TIME:     6.5usec max (measured on 190131 - rev 00.29 code)
//
//------------------------------------------------------------------------------------------------------------

void DMA2_Stream0_IRQHandler(void)
{

//  TESTPIN_A3_TOGGLE;                        // *** DAH TEST CODE FOR TIMING

  DMA2->LIFCR |= 0x00000020;                 // Reset the transfer complete interrupt flag for Stream0

  ADC_Process_Samples();

  // Re-start the DMA data stream.  Stream 0   Received words are stored in ADC_single_capture[0..9].
  // Must clear all event flags before initiating a DMA operation
  DMA2->LIFCR |= (DMA_LIFCR_CTCIF0 + DMA_LIFCR_CHTIF0 + DMA_LIFCR_CTEIF0 + DMA_LIFCR_CDMEIF0
                   + DMA_LIFCR_CFEIF0);
  DMA2_Stream0->CR |= 0x00000001;

// TESTPIN_A3_TOGGLE;                       // *** DAH TEST CODE FOR TIMING

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        DMA2_Stream0_IRQHandler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        I2C2_EV_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I2C2 (Override Microprocessor Interface) Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the I2C2 event interrupt.  The STM32F407 is a
//                      slave.  The override microprocessor is the master.  The basic algorithm for handling
//                      transmissions and receptions with the override micro are handled as follows.
//                          - Check whether the ADDR bit is set
//                              - If it is set, we are starting a new packet:
//                                  - Set the transmit and receive indices to 0
//                                  - Exit
//                              - If it is clear, continue
//                          - Check whether the STOPF bit is set
//                              - If it is set, we have received a stop from the Master:
//                                  - Set the message received flag if we have been receiving a message
//                                  - Exit
//                              - If it is clear, continue
//                          - Check whether the RXNE flag is set
//                              - If it is set, we have received data:
//                                  - If there are no errors, store the data byte in the buffer
//                                  - Exit
//                              - If it is clear, continue
//                          - Check whether the AF flag is set
//                              - If it is set, we are done transmitting:
//                                  - Exit
//                          - Check whether the TXE flag is set
//                              - If it is set, we have data to transmit:
//                                  - Transmit data from the buffer, if there is data left to transmit
//                                  - If it is the last byte to transmit, set the STOP flag to release the
//                                    bus
//                                  - Exit
//
//  CAVEATS:            None
// 
//  INPUTS:             I2C->SR1, I2C->DR, OVR_I2C.RxBuf[], OVR_I2C.RxNdxIn, OVR_I2C.TxValBuf[], OVR_I2C.TxValNdx,
//                      OVR_I2C.NumChars
// 
//  OUTPUTS:            I2C->DR, OVR_I2C.Status
//
//  ALTERS:             OVR_I2C.RxNdxIn, OVR_I2C.TxValNdx
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void I2C2_EV_IRQHandler(void)
{
  uint16_t tmp_sr1, tmp_sr2, tmp_dr;
  uint8_t i;
  
  tmp_sr1 = I2C2->SR1;                  // Read the status and data registers into temporaries.  Note, the
  tmp_sr2 = I2C2->SR2;                  //   first two reads (SR1 followed by SR2) clears the ADDR flag
  tmp_dr = I2C2->DR;                    // Read the data register - this clears the RXNE flag
  
  if (tmp_sr1 & I2C_SR1_ADDR)           // If address received, this is the start of a new message, so
  {                                     //   reset the indices and set the state to either receiver or
    OVR_I2C.TxNdx = 0;                  //   transmitter, based on the TRA flag
    OVR_I2C.RxNdx = 0;
    OVR_I2C.Status = (tmp_sr2 & I2C_SR2_TRA) ? I2C2_TX : I2C2_RX;
  }
  else if (tmp_sr1 & I2C_SR1_STOPF)     // If stop command received, the master is closing communications
  {                                     //   If we were receiving, check the received message
    if (OVR_I2C.Status == I2C2_RX)
    {
      if (OVR_I2C.RxNdx == OVR_I2C_RXLEN)
      {
        Ovr_Micro_Comm();                           // process receieved message from Override micro and set up transmit buffer
        for (i=0; i<(OVR_I2C_RXLEN-1); ++i)
        {
          if (OVR_I2C.RxBuf[i] != OVR_I2C.TxBuf[i])
          {
            // OVR_I2C.TxBuf[OVR_I2C_RXLEN - 1] |= OVR_I2C_STAT_CHANGE;           
            if (++OVR_I2C.ErrCount >= 50)
            {
              // *** DAH  SET ERROR FLAG
            }
          }
        }
      }
    }                                   // We don't need to do anything if we were transmitting
    OVR_I2C.Status = I2C2_IDLE;         // Set the state to idle in either case
  }
  else if (tmp_sr1 & I2C_SR1_RXNE)      // If a char has been received, store it if there are no errors and
  {                                     //   there is room
    if ( !(tmp_sr1 & (I2C_SR1_BERR + I2C_SR1_OVR + I2C_SR1_PECERR + I2C_SR1_TIMEOUT))
      && (OVR_I2C.RxNdx < OVR_I2C_RXLEN) )
    {
      OVR_I2C.RxBuf[OVR_I2C.RxNdx] = tmp_dr;
      ++OVR_I2C.RxNdx;
    }
  }
  else if (tmp_sr1 & I2C_SR1_AF)        // If the no acknowledge flag is set, the master doesn't want any
  {                                     //   more characters, so just exit
    OVR_I2C.Status = I2C2_IDLE;
  }
  else if (tmp_sr1 & I2C_SR1_TXE)       // If the transmitter is empty, send out a character
  {
    if (OVR_I2C.TxNdx < OVR_I2C_TXLEN)
    {
      I2C2->DR = OVR_I2C.TxBuf[OVR_I2C.TxNdx++];
    }
    if (OVR_I2C.TxNdx > OVR_I2C_TXLEN) // If that was the last character, we're done transmitting, so
    {                                   //   release the bus when done with this char and set the status to
      I2C2->CR1 |= I2C_CR1_STOP;        //   idle
      OVR_I2C.Status = I2C2_IDLE;
    }
  }

  tmp_sr1 = I2C2->CR1;                  // Dummy operations to make sure the STOPF flag is clear
  tmp_sr2 = I2C2->SR1;
  I2C2->CR1 = tmp_sr1;
                                        // Make sure the error flags are clear
  I2C2->SR1 &= (~(I2C_SR1_TIMEOUT + I2C_SR1_PECERR + I2C_SR1_OVR + I2C_SR1_AF + I2C_SR1_BERR));
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        I2C2_EV_IRQHandler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        I2C3_EV_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I2C3 (Health Bus Interface) Interrupt Service Routine - *** DAH  THIS IS JUST FOR THE TEMPERATURE SENSOR IT IS NO LONGER FOR THE HEALTH BUS
// 
//  MECHANICS:          This subroutine is branched to from the I2C3 event interrupt.  The STM32F407 is a
//                      master.  The health bus device is the slave.  The basic algorithm for handling
//                      transmissions and receptions with the health bus device are handled as follows.
//                      Stage I2C3_WRITECONF&I2C3_WRITEREG is reponsible for sending configuration onpackets (1byte Addr + 3bytes data) and 
//                      write into register what to read (1byte Addr + 1 byte data)
//
//                      Stage I2C3_READSENS is reponsobile to get 5 bytes of sensor data (1byte Addr + 4 bytes data) with NACK at the and
//
//
//  CAVEATS:            None
// 
//  INPUTS:             I2C3->SR1, I2C3->DR, HLTH_I2C.TxBuf[], HLTH_I2C.TxNdx, HLTH_I2C.TxBuf[],
//                      HLTH_I2C.IntState,
//                      OVR_I2C.NumChars
// 
//  OUTPUTS:            I2C->DR, HLTH_I2C.Status
//
//  ALTERS:             OVR_I2C.RxNdxIn, OVR_I2C.TxValNdx
// 
//  CALLS:              None+
// 
//------------------------------------------------------------------------------------------------------------

void I2C3_EV_IRQHandler(void)
{
  volatile uint16_t tmp_sr1, tmp_sr2, tmp_dr;
  tmp_sr1 = I2C3->SR1;                  // Read the status and data registers into temporaries.  Note, the
  tmp_sr2 = I2C3->SR2;                  //   first two reads (SR1 followed by SR2) clears the ADDR flag
  tmp_dr = I2C3->DR;                    // Read the data register - this clears the RXNE flag
  volatile uint8_t i;
  
  switch (HLTH_I2C.IntState)
  {
    case I2C3_WRITECONF:
    case I2C3_WRITEREG:
      tmp_sr1 = I2C3->SR1;
      if ((tmp_sr1 & I2C_SR1_SB) || (tmp_sr1 & I2C_SR1_ADDR))
      {
        I2C3->DR = HLTH_I2C.TxBuf[HLTH_I2C.TxNdx++];
        break;
      }
      else 
      {
        if (HLTH_I2C.TxNdx < HLTH_I2C.TxNumChars)
        {
          I2C3->DR = HLTH_I2C.TxBuf[HLTH_I2C.TxNdx++];
          break;
        }
        else if (HLTH_I2C.TxNdx == HLTH_I2C.TxNumChars)
        {
          I2C3->DR = HLTH_I2C.TxBuf[HLTH_I2C.TxNdx++];
          I2C3->CR1 |= I2C_CR1_STOP;
          if (HLTH_I2C.IntState == I2C3_WRITECONF) HLTH_I2C.Flags = I2C_WRITE_REG;
          else HLTH_I2C.Flags = I2C_READ_SENSOR;
          break;
        }
      }
      break;

    case I2C3_READSENS:
      tmp_sr1 = I2C3->SR1;
      if (tmp_sr1 & I2C_SR1_SB)
      {
        I2C3->DR = HLTH_I2C.TxBuf[HLTH_I2C.TxNdx++];
        break;
      }
      
      if (HLTH_I2C.RxNdx < HLTH_I2C.RxNumChars - 1)
      {
        HLTH_I2C.RxBuf[HLTH_I2C.RxNdx++] = I2C3->DR;
        I2C3->CR1 |= I2C_CR1_ACK;
      }
      else
      {
        HLTH_I2C.RxBuf[HLTH_I2C.RxNdx++] = I2C3->DR;
        I2C3->CR1 |= I2C_CR1_STOP;
        I2C3->CR1 &= ~I2C_CR1_ACK;
        HLTH_I2C.Flags = I2C_PROC_VAL;
      }
      break;

    default:                                // This state should never be entered
      HLTH_I2C.Flags =  I2C_ERROR;
      HLTH_I2C.IntState =  1;
      I2C3->CR1 |= I2C_CR1_STOP;
      break;
  }

  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        I2C3_EV_IRQHandler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        I2C3_ER_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I2C3 (Health Bus Interface) Error Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the I2C3 error interrupt.  It merely sets the
//                      bit to issue a stop condition, and then sets the error flag.  The flag will be read
//                      in the main loop and I2C system will be reset.
//
//  CAVEATS:            None
// 
//  INPUTS:             I2C3->SR1, I2C3->DR, HLTH_I2C.TxBuf[], HLTH_I2C.TxNdx, HLTH_I2C.TxBuf[],
//                      HLTH_I2C.IntState,
//                      OVR_I2C.NumChars
// 
//  OUTPUTS:            HLTH_I2C.Flags
//
//  ALTERS:             I2C->CR1
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void I2C3_ER_IRQHandler(void)
{
  
  HLTH_I2C.Flags =  I2C_ERROR;
  if (HLTH_I2C.ErrCount == UINT8_MAX) HLTH_I2C.ErrCount = 0;
  HLTH_I2C.ErrCount++;

  I2C3->CR1 &= (~I2C_CR1_PE);
  I2C3->CR1 |= I2C_CR1_STOP;
 
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        I2C3_ER_IRQHandler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        TIM4_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer 4 Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the TIM4 Channel 3 or channel 4 Input Capture
//                      interrupt. Channel 3 is used to measure the load frequency while channel 4 is used
//                      to measure the line frequency. The difference between channels 3 and 4 is used to
//                      measure the phase difference for sync check.  Logic-level square waves
//                      corresponding to the phase A load and line voltages are brought into the T3 and T4
//                      inputs respectively.
//                      Timer 4 is configured as an upcounter, with Channels 3 and 4 configured for Input
//                      Capture mode.  On each rising edge of the corresponding load or line frequency input
//                      the appropriate channel captures the counter value and generates an interrupt.
//                      The ISR operates as follows...
//                      1) The interrupt flags are captured and the interrupts are cleared
//                      2) Check whether the Channel 3 and/or 4 capture flag is set.  If set, the appropriate
//                         frequency/frequencies is/are processed.
//                              - Read the captured time (temp_captim)
//                              - First reading (Start is True):
//                                  - Save time in CapTim
//                                  - Done
//                              - Not first reading:
//                                  - Overrun (OvrTimer >= 4 ==> frequency less than 33Hz):
//                                      - Set delta time (temp_deltatim) to max value (xFFFF)
//                                  - No overrun (OvrTimer < 4 ==> frequency greater than 33Hz):
//                                      - Compute the delta time (temp_deltatim) from the new captured time
//                                        (temp_captim) and the previous captured time (CapTim)
//                                  - If the delta time is long enough (temp_deltatim > 750 ==> f < 2.0KHz):
//                                      - Store the captured time and delta time (CapTim and DeltaTim).
//                                        Note, this will also occur if an overrun occurs, since
//                                        temp_deltatim is set to xFFFF in this case
//                              - Clear the overrun timer (OvrTimer = 0)
//
//                      The overrun counter has a resolution of 10msec.  A count of 4 indicates anywhere
//                      from 30msec to 40msec has elapsed since the last capture.  Thus, any period longer
//                      than 30msec (33Hz) is considered invalid. The timer runs at 1.5MHz, with a maximum
//                      count of 65535.  At this count, a maximum period of 43.69msec can be measured, which
//                      is greater than the 40msec overrun time, so we are ok.
//
//                      If the delta time is too small (less than 0.5msec, which is equivalent to a
//                      frequency greater than 2KHz), it indicates that the zero-crossing comparator is
//                      "bouncing".  In this case, the crossing is essentially ignored.  Neither the
//                      captured time (CapTim) nor the delta time (DeltaTim) are updated.  This will
//                      effectively add the small period to the next period measurement.
//
//
//                      If we're trying to sync, ensure frequency/voltage difference is within range.
//                      Some of fhis may done in the main loop, but need to understand errors introduced
//                      by waiting for the main loop processing, especially at higher frequency
//                      differential.
//                      For initial development only perform sync check at the load frequency zero
//                      crossings. This is an arbitrary choice and could be changed to line frequency
//                      or we could check at both line and load.
//
//                      Calculate phase difference based on time difference between channel 3 and channel 4
//                      timer captures.  Convert this to degrees for human consumption.
//
//                      Determine phase difference at the time of breaker closing based on breaker operate
//                      time, frequency difference between the two sources, and phase difference now.  When
//                      within the desired difference (+/-8 degrees for initial development) intiate the
//                      trasnfer (or increment a counter if filtering potential noise.
//
//  CAVEATS:            It is possible, and in the case of dual utilities even likely, that the input
//                      captures will occur almost simultaneously.  This may be handled by checking and
//                      servicing both with one interrupt, or requiring a separate interrupt for each.
//                      To start we will take one snapshot of the status register and clear any flags
//                      pending at that time, handling both captures if they are present at that time.
//                      Any additional interrupts that occur while in this routine will be handled by a
//                      subsequent call to this ISR after the return from interrupt.
// 
//  INPUTS:             TIM4->SR, TIM4->CCR3, TIM4->CCR4
//                      FreqLoad.Start, FreqLoad.OvrTimer, FreqLoad.CapTim, 
//                      FreqLine.Start, FreqLine.OvrTimer, FreqLine.CapTim,
// 
//  OUTPUTS:            FreqLoad.DeltaTim, FreqLine.DeltaTim
//
//  ALTERS:             TIM4->SR
//                      FreqLoad.Start, FreqLoad.CapTim, FreqLoad.OvrTimer,
//                      FreqLine.Start, FreqLine.CapTim, FreqLine.OvrTimer
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     *** MAG  NEED TO REMEASURE AFTER MAJOR RESHUFFLE OF TIMERS 4 AND 10
// 
//------------------------------------------------------------------------------------------------------------

void TIM4_IRQHandler(void)
{
   uint16_t temp_sr;
   uint16_t temp_captim, temp_deltatim, temp_fdiff, temp_ph_diff, temp_limit;
   uint32_t temp_operate_time, temp_line_change, temp_load_change;

   __disable_irq();                    // Disable interrupts to avoid losing flags between read and write
   temp_sr = TIM4->SR;                // Capture the status
   TIM4->SR = 0x0000;                 // Clear all interrupt flags (is this needed?)
   __enable_irq();
   
   // Note, we don't need to worry about the overcapture flags.  This would happen due to noise around the
   // zero-crossing.  The small amount of jitter won't matter much to the frequency calculation, and will
   // either be added to the existing calculation or the next one anyway.
   
   // Check Channel 3 - Load Frequency
   if (temp_sr & TIM_SR_CC3IF)           // If channel 3 interrupt occurred...
   {
     temp_captim = TIM4->CCR3;              // Read the captured time
     if (FreqLoad.Start)                     // If first reading, just save the value
     {
       FreqLoad.CapTim = temp_captim;
       FreqLoad.Start = FALSE;
     }
     else                                    // Otherwise process the reading
     {
       if (FreqLoad.OvrTimer < MAX_PERIOD)       // If period is not too long, compute the delta time from
       {                                         //   the last rising edge (must account for rollover)
          temp_deltatim = ( (temp_captim >= FreqLoad.CapTim) ? 
                (temp_captim - FreqLoad.CapTim) : ((0x10000L - (uint32_t)FreqLoad.CapTim) + (uint32_t)temp_captim) );

// TODO: MAG is "Invalid" correct?  Will this negate Under Frequency Trip at very low frequencies (if that' s even realistic).
          if (temp_deltatim > 45000)             // If the period is greater than 30msec, set it to invalid
          {
            temp_deltatim = 0xFFFF;
            FreqLoad.DeltaTim = temp_deltatim;     // Save the delta time
          }
       }
       else
       {                                
          temp_deltatim = 0xFFFF;                // If period is too long, set the delta time to "invalid"
          FreqLoad.DeltaTim = temp_deltatim;     // Save the delta time
       }
       // Check the (valid) measured period.  If it is greater than 0.5msec, it is considered valid (not a "runt")
       //   and is ok to use.  If it is less than 0.5ms, don't update the previous captured value or set new
       //   public delta.  This will add the runt period to the next cycle's period
//       if (temp_deltatim > 748)              // If the period is long enough (allow for tolerance so use 748
//       {                                     //   counts instead of 750 to ensure 2.0KHz is accepted)...

         // there is noise/ringing at the falling edge of the detector, so limit the upper
         // frequency to 80 Hz
         if (( temp_deltatim != 0xFFFF ) && ( temp_deltatim > 18750 ))
         
         {
          FreqLoad.CapTim = temp_captim;         // Save the captured time
          FreqLoad.DeltaTim = temp_deltatim;     // Save the delta time

/* The following code determines phase difference between line and load and change rate of phase difference
   to handle advance time for in-phase and closed transfers.

   We can add tests to skip these calculations when they are not needed (no transfer in process, etc.)

   Some of this MAY be moved out of the ISR, but need to be careful that it is not held off too long.
*/
          if (FreqLine.DeltaTim != 0xFFFF)
          {
             // calculate frequency difference * 100  (this part could be done in main loop when frequency is calculated)
             temp_fdiff = ( temp_deltatim <= FreqLine.DeltaTim ) ?
                TIM4_FREQ / ( temp_deltatim - FreqLine.DeltaTim) :
                TIM4_FREQ / ( FreqLine.DeltaTim -  temp_deltatim);

             // validate frequency difference against in-phase or closed transfer setpoint
             // Don't calculate phase difference if the frequency difference is greater than the
             // setpoint. 
             // if (temp_fdiff <= appropriate setpoint)...
             
             // calculate the phase difference 
             if (temp_captim > FreqLine.CapTim )
             {
                temp_ph_diff = temp_captim - FreqLine.CapTim;
             }
             else  // handle rollover
             {
                temp_ph_diff = (uint32_t)temp_captim + 0x10000L - (uint32_t)FreqLine.CapTim;
             }
             // convert this to degrees difference and save as global, then do remaining sync stuff
             // this conversion is only required for HMI purposes and debugging.  If we aren't going
             // to display the phase difference we can remove this for the final release
             Sync_Phase = 360 * (uint32_t)temp_ph_diff / FreqLine.DeltaTim;


                // - counts/cycle / 360 degrees/cycle -> counts/degree
                // - operate time in counts / counts/degree -> degree change during operate time
                // - difference in degree change during operate time + current phase difference
                //   yields phase difference at time of closing

             // calculate operate time
             temp_operate_time = (uint32_t)CLOSE_TIME_CNT;
             // if (IN-PHASE TRANSFER), add open time
             temp_operate_time += (uint32_t)OPEN_TIME_CNT;

             // calculate degrees change in line and load during the operate time
             temp_load_change = temp_operate_time * 360 / temp_deltatim;
             temp_line_change = temp_operate_time * 360 / FreqLine.DeltaTim;

             // now difference between line and load change
             if ( temp_load_change <= temp_line_change )     // load freq is higher/equal
             {
                temp_ph_diff = temp_ph_diff + temp_load_change - temp_line_change;
             }
             else
             {
                temp_ph_diff = temp_ph_diff + temp_line_change - temp_load_change;
             }
             if (temp_ph_diff > temp_deltatim)
             {
                temp_ph_diff -= temp_deltatim; // subtract off one cycle time
             }
             
             // calculate 8 degree limit in counts
             // counts/cycle / 360degrees/cycle => counts/degree * 8 degrees 
             // temp_limit = (temp_deltatim / 360) * 8;
             temp_limit = temp_deltatim/ 45;
             // now check if within +/- 8 degrees
             if (temp_ph_diff <= temp_limit)
             {
                // start transfer (or increment a counter if we want to filter potentially noisy sources)
                TransferNow = 1;
             }

             // check on this side of zero crossing
             else if (temp_ph_diff >= ( temp_deltatim - temp_limit))
             {
                // start transfer (or increment a counter if we want to filter potentially noisy sources)
                TransferNow = 1;
             }
             else
             {
                TransferNow = 0;
             }
          }   
       }
     }
     FreqLoad.OvrTimer = 0;                  // Reset the load overrun timer
   }

   // as noted in function header, it's possible that both captures occur almost simultaneously, so need to
   // handle that possibility so that we don't miss one
   
   // Check Channel 4 - Line Frequency
   if (temp_sr & TIM_SR_CC4IF)           // If channel 4 interrupt occurred...
   {
     temp_captim = TIM4->CCR4;              // Read the captured time
     if (FreqLine.Start)                     // If first reading, just save the value
     {
       FreqLine.CapTim = temp_captim;
       FreqLine.Start = FALSE;
     }
     else                                    // Otherwise process the reading
     {
       if (FreqLine.OvrTimer < MAX_PERIOD)       // If period is not too long, compute the delta time from
       {                                         //   the last rising edge (must account for rollover)
          temp_deltatim = ( (temp_captim >= FreqLine.CapTim) ? 
                (temp_captim - FreqLine.CapTim) : ((0x10000L - (uint32_t)FreqLine.CapTim) + (uint32_t)temp_captim) );
          if (temp_deltatim > 45000)             // If the period is greater than 30msec, set it to invalid
          {
            temp_deltatim = 0xFFFF;
          }
       }
       else
       {                                
          temp_deltatim = 0xFFFF;                // If period is too long, set the delta time to "invalid"
       }
// TODO: MAG need to determine the max frequency, probably closer to 440 than 2KHz.  How do we handle noise (runts)?  Need to be sure greater than Max frequency will trigger OverFreq. Trip.
       // Check the measured period.  If it is greater than 0.5msec, it is considered valid (not a "runt")
       //   and is ok to use.  If it is less than 0.5ms, don't update the previous captured value or set new
       //   public delta.  This will add the runt period to the next cycle's period
//       if (temp_deltatim > 748)              // If the period is long enough (allow for tolerance so use 748
//       {                                     //   counts instead of 750 to ensure 2.0KHz is accepted)...

       // there is noise/ringing at the falling edge of the detector, so limit the upper
       // frequency to 80 Hz
       if ( temp_deltatim > 18750 )
       {
          FreqLine.CapTim = temp_captim;         // Save the captured time
          FreqLine.DeltaTim = temp_deltatim;     // Save the delta time

/*
We could do phase sync detection at both line and load zero crossings, but the extra code and execution
time provides minimal improvement in sync detection.  It MIGHT be helful when the two sources are at max
frequency difference, so drifting quickly in and back out of sync. This can be checked once we settle on
frequency limits and phase window.
*/
       
       }
     }
     FreqLine.OvrTimer = 0;                  // Reset the line overrun timer
   }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        TIM4_IRQHandler()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        TIM1_UP_TIM10_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Timer 1 Update and Timer 10 Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the TIM10 overflow interrupt.  Timer 10 is
//                      configured to overflow every 1.5msec.  The time is set to 1.5msec because that is
//                      the page programming time for the on-board Flash chip.  This allows waveform capture
//                      storage to occur at the maximum rate.
//                      This subroutine calls the SPI1 handler:
//                      SPI1_Flash_Manager() is used to process requests involving the Flash.  Most of these
//                      processes take more than one step to complete.

//                      
//
//  CAVEATS:            Timer 1 is not configured to generate interrupts and so it is assumed that this ISR
//                      is branched to only from Timer 10
// 
//  INPUTS:             None
// 
//  OUTPUTS:            None
//
//  ALTERS:             TIM10->SR, TIM10->CR1
// 
//  CALLS:              SPI1_Flash_Manager()
// 
//  EXECUTION TIME:     *** MAG  NEED TO REMEASURE AFTER MAJOR RESHUFFLE OF TIMERS 4 AND 10
//  EXECUTION TIME:     Measured the execution time on 180601: 616usec (both with using the scope and the    *** DAH REMEASURE THIS, SHOULD BE LOWER!!!  (I ALREADY UPDATED THE COMPUTATION)
//                      internal timer using Get_InternalTime()) max.  The worst-case time occurs when the
//                      demand is being written into Flash:
//                          S1F_DMND_WRITE1 FRAM_Read() - about 128 * 8/15 = 69usec
//                          S1F_DMND_WRITE2 FRAM_Write() - about 256 * 8/15 = 137usec
//                          3 x sampling interrupt (DMA1_Stream0_IRQHandler) - about 3 x 40usec = 120usec
//                          TOTAL: 326usec plus overhead, other instructions, etc.
// 
//------------------------------------------------------------------------------------------------------------

void TIM1_UP_TIM10_IRQHandler(void)
{
  TIM10->SR = 0x0000;                       // Clear the interrupt
  SPI1_Flash_Manager();
  TIM10->CR1 |= 0x0001;                     // Enable the timer
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        TIM1_UP_TIM10_IRQHandler()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        USART6_IRQHandler()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART6 Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the UART6 interrupt.  It checks whether the
//                      communications interface is Modbus or CAM_Comm, then calls the appropriate
//                      subroutine to process the interrupt.
//
//  CAVEATS:            None
// 
//  INPUTS:             CAM2 Communications Setpoint     *** DAH
// 
//  OUTPUTS:            None (see Process_Modb_IRQ() and Process_CAM_IRQ())
//
//  ALTERS:             None (see Process_Modb_IRQ() and Process_CAM_IRQ())
// 
//  CALLS:              Process_Modb_IRQ(), Process_CAM_IRQ()
// 
//------------------------------------------------------------------------------------------------------------

void USART6_IRQHandler(void)
{
  uint16_t tmp_sr, tmp_dr;
  
  // Read the status and data registers into temporaries.  Aside from holding the data, this sequence:
  //   - clears the RXNE (read data register not empty) flag in the SR register
  //   - clears the ORE (overrun), NF (noise detected), FE (framing), and PE (parity) error flags in the SR
  //     register
  tmp_sr = USART6->SR;                   // Read the status register
  tmp_dr = USART6->DR;                   // Read the data register

  // First check whether the communications are CAM_Commm or Modbus
//  if ( (CAM2_Comms == MODB_COMMS)
  {
    if (tmp_sr & USART_SR_RXNE)             // If receive interrupt, call subroutine to process it
    {
      Process_ModB_RxIRQ(tmp_sr, tmp_dr);
    }
  }
//  else
//  {
//    Process_CAM_IRQ();
//  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        USART6_IRQHandler()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Process_ModB_RxIRQ()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           UART6 Modbus Receive Interrupt Service Routine
// 
//  MECHANICS:          This subroutine is branched to from the UART6 (Modbus) interrupt.  It handles
//                      receptions over the Modbus port as follows:
//
//                      Receive Modbus Character Processing:
//
//                      Entry:
//                          Read the UART status and data registers.  These are used later if we want to
//                            store the data 
//                          Read the character interval timer (time from last character)
//                          Restart the character interval timer
//                          Check Comm_State.  If we are transmitting, we shouldn't have gotten this
//                            interrupt, so abort the reception.  If we are in the Idle state, or Receiving
//                            state, we are ok to receive a character.  If we are in the Idle state, this is
//                            the first character of a new message, so set Comm_State to Receiving.  Go on
//                            to the CHECK_INTERVAL state to check the time between characters
//                                  +----> Next state (CHECK_INTERVAL)
//
//                      CHECK_INTERVAL:
//                          Check character interval time and abort flag
//                          If time >= t3.5 (3.5 character times), this character is the start of a new
//                            frame, so go on to the FRAME_BREAK state to process the frame break
//                          Otherwise this character belongs with the existing frame so:
//                            If the Abort flag is TRUE, we have already decided to abort this message.
//                              Basically, we are just looking for a frame break, so ignore the character
//                                and exit
//                              Otherwise check the interval time.  If it is valid (less than 1.5 character
//                                times) go on to the CHAR_RECEIVED state to process the character
//                              Otherwise the interval time is invalid (between t1.5 and t3.5).  This is an
//                                error so go abort the reception (RxIabort = TRUE, RxMsgNdx = 0)
//                                  +----> Exit or next state (FRAME_BREAK, or CHAR_RECEIVED)
//
//                      FRAME_BREAK:
//                          If there are already is a message to process (the Rx buffer index RxMsgNdx > 0),
//                            we cannot process this new message, so ignore it and continue working on the
//                            existing message.  Set the Abort flag so any additional characters will be
//                            ignored.  Don't clear the RxMsgNdx, because this is the length of the existing
//                            message.  It will be cleared in the main loop when the existing message has
//                            been processed.  In addition, set the FrameBreak flag to signal the main loop
//                            to begin processing the existing message.
//                            Note:
//                              If this occurs, the Modbus master has sent a new message too soon, and did
//                              not wait the proper Response Time or Turnaround Time (whichever applies).
//                              We should have completed processing the previous message in the main loop,
//                              and removed this message (by setting RxMsgNdx to 0)
//                          Otherwise this is the start of a new message to process, so clear the Abort and
//                            FrameBreak flags and process the received character (state = CHAR_RECEIVED)
//                                  +----> Exit or next state (CHAR_RECEIVED)
//
//                      CHAR_RECEIVED:
//                          If there is a UART error (parity, framing, or overrun), the character is bad, so
//                            increment the error message counter and abort the message
//                          Otherwise the character is ok, so if there is room in the Rx Message Buffer,
//                            store the character and increment the index (RxMsgNdx)
//                          Otherwise there is no room in the buffer, so increment the overrun error counter
//                            and abort the message
//                                  +----> Exit
//
//                      Additional Note:
//                        RxMsgNdx is not just the index of the next open character in the Rx buffer.  It
//                        also indicates whether there is an existing message, and the length of said
//                        message.  For this reason, care must be taken to reset RxMsgNdx only when an
//                        existing message has been processed, or when an incoming message is aborted.  It
//                        It should NOT be reset if a new message is starting (i.e., a frame break occurs),
//                        but there is an existing message. In this case, RxMsgNdx indicates the length of
//                        the existing message and is needed to process the existing message.  It will be
//                        reset when we have finished processing the existing messsage.
//
//  CAVEATS:            None
// 
//  INPUTS:             status_reg - uart status register
//                      data_reg - uart data register
// 
//  OUTPUTS:            TP.RxBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxIn, TP.StrPtr, TP.TxValNdx
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Process_ModB_RxIRQ(uint16_t status_reg, uint16_t rx_data_reg)
{
  uint16_t tmp_tim2sr;
  uint8_t rxistate, rxiexit;

  // Timer 2 is a downcounter.  It is initialized to 3.5 character times.  It auto-reloads when the count
  // reaches 0.  When this occurs, an event is generated and the event flag (SR_UIF) is set in the SR
  // register.  Thus:
  //   (tmp_sr & TIM_SR_UIF) = TIM_SR_UIF: time is >= t3.5 so new frame
  //   timer 2 count >= ModB.T1P5: time is less than t1.5 and character is ok
  //   Otherwise time is between 1.5 and 3.5 character times and so character is invalid
  // Note, we are operating with a relaxed specification.  We will accept characters that arrive between
  // 1.5 and 3.5 character times, so the only thing we have to check for is time > t3.5, indicating a frame
  // break.
  tmp_tim2sr = TIM2->SR;
  TIM2->SR &= (~TIM_SR_UIF);            // Clear the underflow flag in case it was set
  TIM2->CNT = TIM2_MODB_3P5_9600;       // Reset the counter
  TIM2->CR1 |= 0x0001;                  // Restart the character timer in case an event occurred

  rxistate = CHECK_INTERVAL;            // Initialize state variable
  rxiexit = FALSE;                      // Initialize exit flag

  if (ModB.CommState == MODB_TRANSMITTING)   // If Comm State is Transmitting, this reception is an error,
  {                                          //   so abort the message
    ModB.RxIabort = TRUE;                        // Set Abort flag
    ModB.RxIFrameBreak = FALSE;                  // Clear the Frame Break flag
    ModB.RxMsgNdx = 0;                           // Empty the char buffer by setting the index to 0
    rxiexit = TRUE;                              // Exit
  }
  else                                       // Otherwise the Comm State is either Idle or Receiving.  Make
  {                                          //   sure it is Receiving
    ModB.CommState = MODB_RECEIVING;
  }

  while (!rxiexit)                      // Enter the state machine
  {
    switch (rxistate)
    {
      case CHECK_INTERVAL:              // Check Interval Between Characters
        if (tmp_tim2sr & TIM_SR_UIF)          // If interval > 3.5 char times, this is a message frame, so
        {                                   //   jump to handle the frame
          rxistate = FRAME_BREAK;
        }
        else                                // This is not a message frame
        {
          if (ModB.RxIabort == TRUE)            // If the message has already been aborted, ignore the
          {                                     //   character
            rxiexit = TRUE;
          }
          else // if (tmp_cnt >= ModB.T1P5)        // Otherwise ok so far.  If interval < 1.5 char times, char
          {                                     //   is part of existing frame, so jump to process the char
            rxistate = CHAR_RECEIVED;
          }
//          else                                  // Otherwise 1.5 bit times < interval < 3.5 bit times:
//          {                                     //   bad char so abort the message
//            ModB.RxIabort = TRUE;                   // Set Abort flag
//            ModB.RxMsgNdx = 0;                      // Empty the char buffer by setting the index to 0
//            rxiexit = TRUE;                         // Exit
//          }
        }
        break;

      case FRAME_BREAK:                 // End of Frame
        // Since we have detected an end of frame, see if there is an existing message to be processed.
        //   There is a message to process if there are chars in the buffer.  Note, this is an error by the
        //   Modbus Master, because it transmitted the start of another message before we finished
        //   processing and responding to (if not a broadcast message) the old message.  Basically, the
        //   master didn't wait the required Turnaround or Response Time.
        if (ModB.RxMsgNdx > 0)              // If there is a valid message, set the abort flag so no more
        {                                   //    chars are received.  This prevents the message that is
          ModB.RxIabort = TRUE;             //    being processed from getting corrupted
          ModB.RxIFrameBreak = TRUE;        // Set the Frame Break flag
          rxiexit = TRUE;
        }
        // Otherwise there is no existing message to process, so set up for a new message
        else
        {
          ModB.RxIabort = FALSE;            // Clear the Abort flag
          ModB.RxIFrameBreak = FALSE;       // Make sure the Frame Break flag is clear
          rxistate = CHAR_RECEIVED;         // Jump to process the first char in the new frame
        }
        break;

      case CHAR_RECEIVED:               // Character Received
        // If there is a UART reception error, the message is fouled, so increment the communication error
        //   counter and abort the message
        if ((status_reg & (USART_SR_PE + USART_SR_FE + USART_SR_ORE)) )
        {
          if (ModB.MsgStat_Counter[ERRCTR] < 0xFFFF)
          {
            ModB.MsgStat_Counter[ERRCTR]++;
          }
          ModB.RxIabort = TRUE;                   // Set Abort flag
          ModB.RxMsgNdx = 0;                      // Empty the char buffer by setting the index to 0
        }
        // Otherwise check whether there is room in the character buffer.  If there is no room, it is an
        //   overrun, so increment the overrun counter and abort the message
        else if (ModB.RxMsgNdx >= MODBUS_RX_BUFSIZE)
        {
          if (ModB.MsgStat_Counter[OVRRUNCTR] < 0xFFFF)
          {
            ModB.MsgStat_Counter[OVRRUNCTR]++;
          }
          ModB.RxIabort = TRUE;                   // Set Abort flag
          ModB.RxMsgNdx = 0;                      // Empty the char buffer by setting the index to 0
        }
        // There is room in the character buffer, so insert the char into the buffer, increment the index,
        //   and then exit
        else
        {
          ModB.RxMsgBuf[ModB.RxMsgNdx++] =  rx_data_reg;
        }
        rxiexit = TRUE;
        break;

      default:                      // Default State - this should not be entered
        rxiexit = TRUE;
        break;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        Process_ModB_RxIRQ()
//------------------------------------------------------------------------------------------------------------

