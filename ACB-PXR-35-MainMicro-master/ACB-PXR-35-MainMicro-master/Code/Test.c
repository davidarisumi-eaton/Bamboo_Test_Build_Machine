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
//  MODULE NAME:        Test.c
//
//  MECHANICS:          Program module containing the subroutines that handle the internal test port, user
//                      test, and internal diagnostic subroutines
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
//   0.04   150928  DAH - Added command to display real-time values (DR command).
//                          - TP_Top() revised
//                          - TP_DisplayRTValues() and TP_DisplayRT0Values() added
//   0.05   151001  DAH - Revised TP_DisplayRT0Values() to replace CurHalfCyc with CurHalfCycSOS_Sum
//                      - Added command to read and modify the AFE offset and gain calibration constants
//                          - Added stdlib.h and math.h to included files
//                          - TP_Top() revised
//                          - TP_ModifyCal() added
//                      - Added command to turn off the integration of the AFE current samples
//                          - TP_ExecuteAction() revised
//                          - TP_AFEIntOff added
//                      - Added command to turn off AFE communications.  This was added for power
//                        consumption testing and will eventually be deleted
//                          - TP_ExecuteAction() revised
//                          - TP_AFECommsOff added
//                      - Added commands to turn the ART accelerator on and off.  These were added for power
//                        consumption testing and will eventually be deleted
//                          - TP_ExecuteAction() revised
//                      - Cleaned up comments in TP_GetDecNum() and TP_GetHexNum()
//                      - Updated NAMECOPYRIGHTDATE[] string
//   0.06   151110  DAH - Added command to read and modify real time from the RTC
//                          - States TP_TM0 thru TP_TM5  added to TP_Top()
//                          - TP_RTC() and dhex_ascii() added
//                      - Updated NAMECOPYRIGHTDATE[] string
//                      - Modified MC (modify cal constants) command so that cal constants are written to
//                        FRAM and Flash in SPI2_Manager()
//                      - Added command to initiate a user waveform capture
//                          - TP_ExecuteAction() modified
//                      - Added command to display a user waveform
//                          - States TP_DW0 thru TP_DW2  added to TP_Top()
//                          - TP_DisplayWaveform() added
//                      - Added command to reset the AFE (EAM 3)
//                          - TP_ExecuteAction() revised
//   0.07   151216  DAH - Updated NAMECOPYRIGHTDATE[] string
//                      - Added command to modify Frame EEPOT settings ("EEO" and "EEM")
//                          - States TP_EE0 thru TP_EE2  added to TP_Top()
//                          - TP_EEPOT() and asciidecimal_hex() added
//   0.08   160122  DAH - Updated NAMECOPYRIGHTDATE[] string
//                      - Updated comments in asciidecimal_hex()
//                      - Added EAB command to perform Modbus loopback test
//                      - In TP_DisplayRT0Values(), CurHalfCycSOS_Sum renamed to CurHalfCycSOS_SumF
//   0.09   160224  DAH - Updated NAMECOPYRIGHTDATE[] string
//                      - Deleted EEPOT test commands, as EEPOT is no longer used on rev 2 boards
//                          - TP_EEPOT() deleted
//                          - Deleted "EE" command from TP_Top()
//                          - Deleted TP_EEx states from TP_Top()
//                      - Added "TH" command to read the temperature/humidity sensor and display the values
//                          - Added TP_THSensor() 
//                          - TP_THx states added to TP_Top()
//                      - Deleted asciidecimal_hex() as it is no longer used
//                      - Added commands to display half-cycle, one-cycle, and one-second min, max, and
//                        variance values
//                          - Revised TP_ExecuteAction() to add EAM4 thru EAM6 commands to freeze, thaw, and
//                            reset min/max data acquisition
//                          - Added DR1 command to TP_Top() and added TP_DisplayRT1Values() to display the
//                            min, max, and variance values
//                      - Added a separate state variable, TP.SubState, for the subroutines to reduce the
//                        number of states in TP_Top().  The intent is to keep this subroutine as small and
//                        "clean" as possible for readability.
//                          - Moved call to TP_DisplayRTValues() to state TP_PARSECMNDSTRING
//                          - Added states TP_DR0 and TP_DR1 to TP_Top()
//                          - Deleted TP_DRGR0_x and TP_DRGR1_x from TP_Top().  These are now the substates
//                            in TP_DisplayRT0Values() and TP_DisplayRT1Values()
//                          - Added state TP_MC to TP_Top()
//                          - Deleted TP_MCx from TP_Top().  These are now the substates in TP_ModifyCal()
//                          - Added state TP_TM to TP_Top()
//                          - Deleted TP_TMx from TP_Top().  These are now the substates in TP_RTC()
//                          - Added state TP_DW to TP_Top()
//                          - Deleted TP_DWx from TP_Top().  These are now substates in TP_DisplayWaveform()
//                      - Added EAM7 thru EAM10 commands to TP_ExecuteAction()
//                      - Added EAR1 command to TP_ExecuteAction()
//   0.10   160310  DAH - Reduced states used to read the temperature/humidity sensor in TP_Top()
//                          - Added state TP_TH to TP_Top()
//                          - Deleted TP_THx from TP_Top().  These are now substates in TP_THSensor()
//                      - Added 200msec min and max values to DR1 command
//                          - Modified TP_DisplayRT1Values() to include these values (states TP_DRG1_11 thru
//                            TP_DRG1_16 added)
//                      - In TP_ExecuteAction(), replaced if-else constructs with case-constructs.  Also
//                        revised code to ensure subroutine exits after a command is executed
//                      - Added code to support 10-cycle waveform captures via the test port during a trip,
//                        for test purposes only
//                          - Added EAW2 command to TP_ExecuteAction() to turn on short delay protection
//                          - Modified TP_DisplayWaveform() to display the test waveforms
//                      - Added offset and gain calibration
//                          - Revised TP_Top() to process 'OC' and 'GC' cmnds (added states TP_OC and TP_GC)
//                          - Added TP_OffsetCal() and TP_GainCal()
//   0.11   160317  DAH - Added ADC cal constants to TP_ModifyCal()
//                      - Renamed MCF to MCA so it is consistent with the OCA and OGA commands
//   0.12   160502  DAH - Added AFE voltage display
//                          - State TP_DR2 added to TP_Top()
//                          - State 2 added to TP_DisplayRTValues()
//                          - TP_DisplayRT2Values() added
//                      - Added separate default calibration constants for AFE currents and voltages.
//                          - Renamed AFE_CAL_DEFAULT_GAIN to AFE_CAL_DEFAULT_IGAIN
//                          - Renamed AFE_CAL_DEFAULT_OFFSET to AFE_CAL_DEFAULT_IOFFSET
//                          - Added AFE_CAL_DEFAULT_VGAIN and AFE_CAL_DEFAULT_VOFFSET
//                          - Modified EAR1 command in TP_ExecuteAction() to only restore the AFE current
//                            calibration constants to their default values
//                          - Added EAR2 command in TP_ExecuteAction() to restore the AFE voltage
//                            calibration constants to their default values
//                      - Corrected comments in TP_GainCal() and TP_OffsetCal()
//                      - Modified TP_GainCal() and TP_OffsetCal() to add gain and offset calibration for
//                        the AFE voltage channels
//                      - Added AFE min and max voltage display
//                          - State TP_DR3 added to TP_Top()
//                          - State 3 added to TP_DisplayRTValues()
//                          - TP_DisplayRT3Values() added
//                      - Added power display
//                          - State TP_DR4 added to TP_Top()
//                          - State 4 added to TP_DisplayRTValues()
//                          - TP_DisplayRT4Values() added
//                      - Added phase calibration
//                          - State TP_PC added to TP_Top()
//                          - TP_PhaseCal() added
//                          - Added EAR3 cmnd to TP_ExecuteAction() to restore default phase cal constants
//                          - Added phase cal constants to TP_ModifyCal()
//                      - VAL200msec renamed VAL200MSEC
//                      - Added EAP9 and EAP10 commands to turn on and off Modbus chip power
//                          - TP_ExecuteAction() revised
//   0.13   160512  DAH - Updated NAMECOPYRIGHTDATE[] string
//                      - Added EAR4 and EAR5 commands to restore default ADC current cal constants
//                          - TP_ExecuteAction() revised
//                      - Added DE command to display energy
//                          - TP_DisplayEnergy() added
//                          - TP_Top() revised
//                      - Added EAR6 command to reset energy registers
//                          - TP_ExecuteAction() revised
//   0.14   160620  DAH - Updated NAMECOPYRIGHTDATE[] string
//                      - Added DR5 command to display min and max powers
//                          - State TP_DR5 added to TP_Top()
//                          - State 5 added to TP_DisplayRTValues()
//                          - TP_DisplayRT5Values() added
//                      - Re-added command to modify Frame EEPOT settings ("EEO" and "EEM")
//                          - State TP_EE added to TP_Top()
//                          - TP_EEPOT() and asciidecimal_hex() added
//                      - Modified TP_GainCal() and TP_PhaseCal() to throw out the first reading after setup
//                        because the reading may contain samples from a different input circuit.  Note,
//                        TP_OffsetCal() does not need to be modified because the samples are all taken
//                        after setup has been completed.
//                      - Corrected bug displaying min/max variance in TP_DisplayRT1Values()
//                      - Added Test Injection offset and gain calibration commands and Execute Test
//                        Injection command
//                          - struct TestInj and struct TestInjCal added
//                          - Test_VarInit() revised to initialize test injection variables
//                          - States TP_IOC, TP_IGC, and TP_IT added to TP_Top()
//                          - Added TP_TestInjCal(), TP_TestInjOffsetCal(), TP_TestInjGainCal(), and
//                            TP_TestInj()
//                          - Revised Test_VarInit() to initialize Test Injection conversion scaling
//                            constants and variables
//                      - Added set AFE PGA gain command
//                          - TP_AFEcalgain[] and TP_AFEPGASetting added
//                          - Revised Test_VarInit() to initialize TP_AFEPGASetting
//                          - State TP_AG added to TP_Top()
//                          - TP_AFESetPGA() added
//   0.15   160718  DAH - Added TLA command to test the Alarm LED
//                          - TP_TestLEDs() revised
//                      - Renamed state TP_TL0 to state TP_TL for consistency in TP_Top()
//                      - Added TI command to test the Trip Indicator LEDs
//                          - Modified TP_Top() to add "TI" command
//                          - Added TP_TestIndicators()
//                      - Added CC command to change the CAM type (sampled-value or GOOSE)
//                          - Modified TP_Top() to add "CC" command
//                          - Added TP_ChangeCAMtype()
//                      - Added EAC command to initiate a CAM test transmission
//                          - Added CAM_TEST_STRING[]
//                          - Modified TP_ExecuteAction() (added case "C") to process the EAC command
//                      - Fixed minor bug in TP_EEPOT()
//   0.16   160818  DAH - Replaced TX_STRING with TP_TX_STRING and TX_VALUE with TP_TX_VALUE
//                      - In TP_ExecuteAction(), renamed CAM1.NumChars to CAM1.TxNumChars and CAM2.NumChars
//                        to CAM2.TxNumChars
//                      - Added DS command to display Startup time measurement values
//                          - Modified TP_Top() to add "DS" command
//                          - Added TP_DisplayStartup()
//                      - Added DM command to display memory
//                          - Modified TP_Top() to add "DM" command
//                          - Added TP_DisplayMemory()
//   0.17   161031  DAH - Removed "MB" (initiate Modbus transmission) command from TP_ExecuteAction() as
//                        Modbus has been moved to the display processor
//                      - Revised code to display RTC time to support the switch from the DS1390 RTC to the
//                        MCP7951 RTC
//                          - TP_RTC() revised
//                      - Added EAP10 and EAP11 commands to turn off and turn on display power
//                          - TP_ExecuteAction() revised
//   0.18   161115  DAH - Added "TLH" command to support HiLoad LED for rev 3 boards
//                          - TP_TestLEDs() revised
//                      - TP_THSensor() was modified to support the new temperature/humidity sensor on the
//   0.19   170223  DAH - Added "EAD" command to support display communications testing
//                          - TP_ExecuteAction() revised
//                      - Added TR command to test the auxiliary relays
//                          - TP_AuxRelays() added
//                      - Added "EAB" command.  This command causes a command to be sent to the display
//                        processor to start or stop Modbus communications (square wave output for power
//                        consumption testing)
//                          - TP_ExecuteAction() revised
//                      - Corrected bug in TP_GainCal() and TP_TestInj() where wraparound of the Rx buffer
//                        could cause the inputted current value to be incorrect
//                      - In TP_TestInjOffsetCal(), increased starting value from 2300 to 2375
//                      - Corrected bug in TP_TestInjGainCal().  Set flag to reduce dc filter gain when
//                        doing calibration to match what occurs in normal test injection operation.
//                        Testing found this is necessray to improve the test injection performance
//                      - Corrected bug in TP_TestInj().  Reinitialized integrator constants to normal for
//                        all of the channels before setting up the channel under test.  This handles the
//                        where a channel was under test, then a different channel is tested (without
//                        turning off test injection first)
//                      - Revised test injection to allow control from both the test port and from display
//                        communications
//                          - Moved the actual code that injects the current from TP_TestInj() to a separate
//                            subroutine, TestInjCur(), and added call to TestInjCur() in TP_TestInj().
//                          - TestInjCur() is also called from Disp_Rx() in Display.c
//   0.21   180301  DAH - Revised test injection subroutine to use a sine table instead of a cosine table.
//                        The index, TestInj.CosIndx, needs to be offset by 90 degrees (20), so revised
//                        Test_VarInit(), TP_TestInjOffsetCal(), TP_TestInjGainCal (), and TestInjCur() to
//                        initialize TestInj.CosIndx to 20 instead of 0
//                      - As part of the revision to user waveform captures, UserWF_Req and UserWF_Ack are
//                        now a set of bit flags used to request and acknowledge captures.  UserWF_Flags
//                        hold the status of the capture and associated harmonics computation.
//                          - Revised TP_DisplayWaveform(), TP_OffsetCal(), TP_ExecuteAction()
//   0.22   180420  DAH - Revised per changes to support internal real time and time-stamping
//                          - In TP_RTC(), added code to convert values in RTC_Buf[] to BCD when displaying
//                            the values (in state TP_TM2) because RTC_Read() now returns the time values in
//                            hex.  Also added code to store the values in RTC_Buf[] as hex numbers (not
//                            BCD) when reading new values (in state TP_TM3) because the values are written
//                            into the internal timer.  TP_GetDecNum() is now called instead of
//                            TP_GetHexNum().
//                          - In TP_RTC(), state TP_TM7, subroutine RTC_to_SysTickTime() is called to write
//                            the new time into the internal timer.  The new time is then written into the
//                            RTC chip from the internal timer via SPI2_Manager()
//                          - In TP_RTC(), revised which RTC flags are outputted for the new RTC chip
//                      - Revised per changes for energy and demand logging
//                          - Added include of Events_ext.h
//                          - In TP_DisplayEnergy(), changed EnergyAll.FwdWHr to EngyDmnd[1].TotFwdWHr
//                          - Added "Restore Unit" command.  Presently this is just used to clear events.
//                            In TP_Top(), added state TP_RU and call to ClearEvents()
//   0.23   180504  DAH - Fixed TP_OffsetCal() and TP_DisplayWaveform() for new UserSamples definition
//                      - In TP_RTC(), added calls to Get_InternalTime() and ProcessTimeAdjustment() so
//                        major time changes cause an event and are handled properly by the demand function
//                      - Added Display Demand values (DD) test port command
//                          - Added Demand_def.h, Flash_def.h, and Demand_ext.h includes
//                          - In TP_Top(), added state TP_DD and call to TP_DisplayDmnd()
//                          - TP_DisplayDmnd() added
//   0.24   180517  DAH - Revised TP_RTC() to read and write the RTC directly, as opposed to setting flags
//                        and using the SPI2 manager subroutine, which has been eliminated.
//                          - Deleted state TP_TM1 and renumbered states from TP_TM2 to TP_TM7
//                      - Revised TP_EEPOT() to read and write the EEPOT directly, as opposed to setting
//                        flags and using the SPI2 manager subroutine, which has been eliminated.
//                          - Added state TP_EE3
//                      - Revised TP_ModifyCal(), TP_OffsetCal(), TP_GainCal(), TP_ExecuteAction(),
//                        TP_TestInjOffsetCal(), and TP_TestInjGainCal() to write the calibration values
//                        into FRAM directly, as opposed to setting a flag and using the SPI2 manager
//                        subroutine, which has been eliminated
//                          - Added FRAM_WriteCal()
//                      - Combined Flash_def.h and FRAM_def.h
//                          - Deleted include of Flash_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//                      - In TP_DisplayDmnd(), added support for multiple Flash requestors
//                          - Replaced SystemFlags with FlashCtrl for FLASH_IN_USE arbitration
//                      - Added writing cal constants to Flash (TP_WR_CAL state) to TP_Top().  Modified
//                        TP_ExecuteAction(), TP_ModifyCal(), TP_OffsetCal(), TP_GainCal(),
//                        TP_TestInjOffsetCal(), and TP_TestInjGainCal() to set the next state to TP_WR_CAL
//                        when cal constants have changed so they are written to Flash
//                      - In TP_ExecuteAction(), added EAW3 command for alarm waveform captures
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Modified TP_RTC() to use the request and acknowledge flags when reading and
//                            writing the RTC
//                          - Modified TP_Top() (TP_WR_CAL and TP_WR_CAL1 states), TP_ExecuteAction()
//                            (Restore command), TP_ModifyCal(), TP_OffsetCal(), and TP_GainCal() to use the
//                            request and ack flags when writing the calibration constants to FRAM and Flash
//                          - Modified TP_TestInjOffsetCal() and TP_TestInjGainCal() to use the request and
//                            and ack flags when writing the test injection cal constants to FRAM and Flash
//                          - Deleted FRAM_WriteCal() as writing the cal constants was moved to Iod.c
//                          - Modified TP_EEPOT() to use the request and acknowledge flags when reading and
//                            writing the EEPOT
//                          - Changed tbuf from a structure to a union so that it can be used with several
//                            commands involving FRAM and Flash reads, and moved definition from local to
//                            global
//                          - Modified TP_DisplayDmnd() to use the request and acknowledge flags when
//                            reading the demand logs
//                          - Modified the Restore Unit command to use the request and acknowledge flags
//                            since FRAM and Flash are accessed
//                              - TP_Top() ("RU" states) modified.  Call to ClearEvents() was replaced with
//                                call to TP_RestoreUnit()
//                              - TP_RestoreUnit() added
//                      - Added support for Alarm waveform capture events
//                          - In TP_ExecuteAction(), revised EAW3 command to insert the alarm waveform
//                            capture event into the new event FIFO
//                          - Added include of Events_def.h
//                      - Added DA command to display Alarm waveforms
//                          - In TP_Top(), added TP_DA state
//                          - Added TP_DisplayAlarmLog()
//                          - Added constant array WF_ADDRESS[][]
//                      - Revised TP_DisplayDmnd() to use TP.Temp as an indicator to read demand logs and
//                        also to indicate which value pointer to use
//                      - Replaced TP.FTemp and TP.FTemp1 (floats) with unions TP.Tmp1 and TP.Tmp2 for
//                        greater versatility
//   0.26   180829  DAH - Deleted TestPort_States (moved to Test_def.h as now global)
//                      - Added TP_INIT state to TP_Top(), so Test_VarInit() is called the first time
//                        TP_Top() is called, instead of in the initialization code
//                      - Moved Test_VarInit() from the global section to the local section
//   0.27   181001  DAH - Corrected minor bug in TP_EA_DONE state in TP_Top()
//                      - Added TP_EA_WFDONE state to TP_Top() to support the modifications to the user
//                        waveform capture process
//                      - Revised TP_ExecuteAction() (EAW command) to support the modifications to the User
//                        Waveform capture control flags and process
//                      - Eliminated waveform capture control from TP_DisplayWaveform().  This was moved to
//                        the EAW commands
//                      - Revised TP_OffsetCal() to support the modifications to the user waveform capture
//                        control flags and process
//                      - Revised "DA" command (TP_DisplayAlarmLog()) to display the number of samples
//                      - Added "DT" and "DC" commands to display the Trip and Stip-Chart waveforms
//                          - Renamed TP_DisplayAlarmLog() to TP_DisplayWF() and revised the subroutine to
//                            support Alarm, Trip, and Strip-Chart waveforms
//                          - Added "DT" and "DC" commands to TP_Top()
//                      - Changed EAW3 command from alarm waveform capture to strip-chart waveform capture
//                        and created EAW4 command for alarm waveform capture
//                          - TP_ExecuteAction() revised
//   0.29   190122  DAH - Added "EAH" commands and "DWC" command for coil temperature measurement
//                          - TP_ExecuteAction() and TP_DisplayWaveform() revised
//   0.30   190314  DAH - Corrected bug in TP_TestInjOffsetCal() and TP_TestInjGainCal().  Modified the code
//                        to store the channel to be calibrated in TestInj.Channel instead of TP.Temp.  The
//                        test injection code in the sampling interrupt uses TestInj.Channel.
//                  LEB - Added support for coil temperature measurements on all phases
//                          - Revised TP_ExecuteAction() to change the EAH2 command to EAH0 for turning off
//                            coil temp sensing.  Also added EAH1, EAH2, EAH3, EAH4 commands to compute the
//                            coil RMS value for all phases and neutral
//                          - Added the RMS value to the coil temperature waveform display (DWC) in
//                            TP_DisplayWaveform()
//   0.31   190506  DAH - Removed Harmonics computation from User Waveform captures.  They are now
//                        completely independent
//                          - Eliminated USR_TYPE_TP from the capture request types.  This was a special
//                            capture request that requested a single cycle of all waveforms without a
//                            harmonics computation.  Since harmonics will no longer be computed when a user
//                            capture is done, this request is the same as USER_TYPE_ALL.
//                            In TP_OffsetCal(), replaced USR_TYPE_TP with USR_TYPE_ALL
//                          - In the TP_EA_WFDONE state of TP_Top(), eliminated check of HarmAck for when
//                            the waveform capture has been completed
//   0.32   190726  DAH - Demand voltages were deleted
//                          - In TP_DisplayDmnd(), adjusted the indexing of the demand logs to account for
//                            the deletion of the demand voltages
//                      - In TP_DisplayRT2Values(), struct VolAFEOneCycLN and VolAFEOneCycLL combined into
//                        VolAFEOneCyc, and struct VolAFE200msecLN and VolAFE200msecLL combined into
//                        VolAFE200msFltr
//                      - In TP_DisplayRT0Values(), Cur200msec renamed to Cur200msFltr
//                      - In TP_DisplayRT1Values, Cur200msec_max renamed to Cur200msFltr_max, and
//                        Cur200msec_min renamed to Cur200msFltr_min
//                      - In TP_DisplayRT3Values(),struct VolAFEOneCycLNmax and VolAFEOneCycLLmax combined
//                        into VolAFEOneCyc_max, struct VolAFEOneCycLNmin and VolAFEOneCycLLmin combined
//                        into VolAFEOneCyc_min, struct VolAFE200msecLNmax and VolAFE200msecLLmax combined
//                        into VolAFE200msFltr_max, and struct VolAFE200msecLNmin and VolAFE200msecLLmin
//                        combined into VolAFE200msFltr_min
//                      - In TP_GainCal(), struct Cur200msec renamed to Cur200msFltr and
//                        struct VolAFE200msecLN replaced by struct VolAFE200msFltr
//   0.34   191002  DAH - Coil Test code changed for Rev 4 boards
//                          - In TP_ExecuteAction(), revised coil temperature measurement commands (EAH0
//                            thru EAH4) to no longer change the 4800Hz coil drive pin configuration to
//                            start and stop the coil drive signal.  Instead, Timer8 is turned on and off,
//                            and the multiplexer is enabled and disabled
//                          - In TP_TestInjOffsetCal(), TP_TestInjGainCal(), and TestInjCur(), replaced
//                            TSTINJ_SEL_PH with TSTINJ_DIS to turn off test injection
//                      - Deleted EAB command as it is no longer needed (EAC command will work to test the
//                        port).  TP_ExecuteAction() revised to delete the command.  Also deleted TP_EA_DONE
//                        state from TP_Top() as it is no longer used.
//                      - TP_ExecuteAction() revised to add EAC3 thru EAC6 commands for CAM testing
//   0.35   191118  DAH - In TP_ExecuteAction() modified EAP9 and EAP10 to also set the CAM2 Rx and Tx pins
//                        to the appropriate mode (UART or Output)
//                      - Added one-cycle and 200msec ADC voltages to the DR2 command
//                          - Added states TP_DRGR2_3 thru TP_DRGR2_6 to TP_DisplayRT2Values()
//                      - Added gain and offset constants for Rogowski Igsrc and CT In into struct AFEcal
//                          - Revised EAR1 command in TP_ExecuteAction() accordingly
//                          - Revised MCA command in TP_ModifyCal() accordingly
//                      - Size of ADCcalLow.gain[], ADCcalLow.offset[], ADCcalHigh.gain[], and
//                        ADCcalHigh.offset[] reduced from 10 to 8
//                          - Revised MCH and MCL commands in TP_ModifyCal() accordingly
//                      - Added support for ADC voltages
//                          - Revised TP_ExecuteAction() to add command to restore default ADC voltage
//                            constants.  Command added as EAR6.  The existing EAR6 command (reset energy)
//                            was renamed to EAR7
//                          - Revised TP_OffsetCal() and TP_GainCal() to add ADC voltages to OC and GC
//                            commands
//                      - Added GF command to configure GF sensing for Rogowski or CT
//                          - Added GF command and TP_GF state to TP_Top()
//                          - Added TP_DisplayGF_CTstatus() and TP_SetReset_GF_CT_EN()
//                      - Added NF command to configure neutral sensing for Rogowski or CT
//                          - Added NF command and TP_NF state to TP_Top()
//                          - Added TP_DisplayN_CTstatus() and TP_SetReset_N_CT_EN()
//                      - Added support for Igsrc and In using a Rogowski coil or a CT
//                          - Revised EAR3, EAR4, and EAR5 commands in TP_ExecuteAction() accordingly
//                          - Revised EA5 command so that it no longer affects index = 4 (Vn).  This index
//                            location is now used to hold the In with CT cal constant
//                          - TP_OffsetCal(), TP_GainCal(), and TP_ModifyCal() revised
//                      - Modified test injection to support an input current of zero.  This connects the
//                        test injection circuit, but with no injection current.  Before this change, an
//                        input current of zero would disconnect the current injection current.
//                          - TestInjCur() revised
//   0.36   200210  DAH - Display_def.h renamed to DispComm_def.h
//                      - Display_ext.h renamed to DispComm_ext.h
//                      - "EAD1" command no longer works, as RTD buffers are only sent when requested by the
//                        display processor
//                          - TP_ExecuteAction() revised
//   0.37   200604  DAH - Revised TP_DisplayWF() based on changes to struct FLASH_INT_REQ and FRAM waveform
//                        storage definitions
//   0.42   201202  DAH - Revised "TM" command to accomodate the internal RTC instead of the external RTC
//                        and added displaying the system time in addition to the internal RTC time
//                          - TP_RTC() revised
//                      - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added includes of RealTime_def.h and RealTime_ext.h
//   0.43   210115  DAH - Added EAG command to support testing the 61850 communications
//                          - TP_ExecuteAction() revised
//   0.44   210324  DAH - Additional support for 61850 communications with the Display Processor
//                          - In TP_ExecuteAction(), revised EAG command to cause different GOOSE message
//                            types to be transmitted, instead of multiple requests of the same type.
//                            This is a more realistic test
//   0.45   210504  DAH - In TP_Top(), modified state TP_INIT to replace the call to Test_VarInit() with
//                        setting SPI2 request flag S2F_INJ_RD.  This moves the SPI2 read operations into
//                        the 1.5msec timer interrupt.  Also added state TP_INIT1 to check for completion of
//                        the reads.  The call in TP_Top() (main loop) was conflicting with calls in the
//                        1.5msec timer interrupt and causing SPI2 to hang.  All SPI2 accesses must be done
//                        in the initialization code before the interrupt is enabled, or through the SPI2
//                        managers.
//                          - TP_Top() revised per description above
//                          - Test_VarInit() revised to no longer call ReadInjCalConstants()
//   0.46   210521  DAH - Corrected bugs in TP_DisplayWF() when displaying header information.
//                          - Starting index was eliminated from display, and EID format was corrected
//                      - Enhanced display event waveform commands
//                          - Code added to show number of waveform captures and the index of the most
//                            recent capture (DT<crlf>, DA<crlf>, DC<crlf>)
//                          - Code added to enable the user to select which current or voltage waveform to
//                            display (previously was hard-coded for Ia)
//                              - TP_DisplayWF() revised
//   0.47   210525  DAH - Corrected bugs in TP_GainCal() for ADC voltages
//                          - TP.ValPtr1 was pointing to 200msec voltages and should have been one-cycle
//                            voltages
//                          - Index increment was wrong when setting TP.ValPtr1 (was j, should be (j - 5))
//   0.48   210730  DAH - Corrected minor bug in TP_DisplayWF() when displaying waveform summary information
//                      - Added temporary MT test port command to change the instantaneous protection pickup
//                        level.  Reference *** DAH 210730
//   0.51   220304  DAH - Min/max current values have switched from 200msec values to one-cycle values
//                          - TP_DisplayRT1Values() revised (Cur200msFltr_max.Ia and Cur200msFltr_min.Ia
//                            replaced with CurOneCyc_max.Ia and CurOneCyc_min.Ia)
//                      - Min/max voltage values changed from 200msec values to one-cycle values
//                          - TP_DisplayRT3Values() revised to delete VolAFE200msFltr.Vxx values
//   0.58   220829  DAH - Fixed bug in test port when retrieving test injection cal constants
//                          - In TP_Top(), added code to exit the state when waiting for the test injection
//                            cal constants to be retrieved
//   0.59   220831  DAH - Revised TP_EEPOT() to replace code that set S2NF_RD_EEPOT and S2NF_WR_EEPOT with
//                        calls to EEPOT_Xfer() and EEPOT_Write()
//                      - Revised TP_DisplayWF() to replace code that set S2NF_TP_RD_REQ with call to
//                        FRAM_Read()
//                      - Revised TP_RestoreUnit() to replace code that set S2F_CLR_EVENTS with call to
//                        ClearEvents()
//                      - In TP_Top(), state TP_WR_CAL1, added code to write cal constants to FRAM
//                      - In TP_Top(), state TP_INIT, added code to read the test injection cal constants
//                        from FRAM
//                      - In TP_TestInjOffsetCal() and TP_TestInjGainCal(), added code to write the test
//                        injection cal constants to FRAM
//   0.60   220928  DAH - Added "DV" command to display summary event logs
//                      - FRAM_Read() revised to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//   0.61   221001  DB - Added "DI" and "DJ" command to display trip and time adjustment event logs
//   0.63   221111  DB - Added all functions to look up index in Logs by EID, for all logs according to
//                       RAM_EV_ADD[] list
//                       Using enum of enum TestPort_States - TP_LS (Summary), TP_LT (Trip),
//                       TP_LR (TestTrip), TP_LM(MajAlarm), TP_LD(Demand), TP_LU(TripWF), TP_LV(AlarmWF),
//                       TP_LW(ChartWF) means when type LS1010 then you get index of Summary log of EID=1010
//   0.64   221118  DB - Added call to ClearLogFRAM()to zero the 256K FRAM (to clear out events) when the
//                       Restore Unit (RU) command is issued
//                          - TP_Top() revised
//   0.65   221201  DB  - EID temporary variables in TP_LookUpSumEvent(), TP_LookUpTripEvent(),
//                        TP_LookUpTestTripEvent(), TP_LookUpMajAlarmEvent(), TP_LookUpDemandEvent(),
//                        TP_LookUpTripWFEvent(), TP_LookUpAlarmWFEvent(), TP_LookUpCharTWFEvent() changed
//                        from uint16_t to uint32_t
//   0.67   221209  DAH - S2F_xx flags renamed to S1F_xx flags
//                      - SPI2Flash.xx renamed to SPI1Flash.xx
//   0.68   230124  DB  - TP_DisplayExtendedValues with MX and MZ keys that list the captured values from
//                        BB FRAM when Extended capture is in progress
//   0.69   230220  DAH - In TP_LookUpMajAlarmEvent(), EV_TYPE_MAJALARM renamed to EV_TYPE_ALARM
//                      - TP_LookUpMajAlarmEvent() renamed to TP_LookUpAlarmEvent()
//                      - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - Renamed TP_LookUpCharTWFEvent() to TP_LookUpExtCapWFEvent() and modified for
//                            extended waveform captures
//                          - Deleted EAW3 command from TP_ExecuteAction()
//                          - Deleted DC command
//   0.70   230221  DB  - TP_DisplayDisturbanceEvent that listed Disturbance structure from FRAM
//   0.72   230320  DAH - In TP_SetReset_N_CT_EN(), changed neutral sense control from PC9 to PD14
//    25    230403  DAH - Commented out call to ClearLogFRAM() in TP_RU state of TP_Top()
//                      - Revised TP_DisplaySummaryEvent() to display the number of summary logs
//                      - Deleted WF_ADDRESS[] (moved to Iod.c and Iod_ext.h)
//                  KT  - Added Capture GOOSE commands to EAGx command
//    40    230531  DAH - In TP_DisplayTripEvent(), replaced TRIP_EVENT_SIZE with SNAPSHOTS_EVENT_SIZE  
//                      - In TP_DisplayTestTripEvent(), replaced TESTTRIP_EVENT_SIZE with
//                        SNAPSHOTS_EVENT_SIZE
//                      - In TP_DisplayAlarmEvent(), replaced ALARM_EVENT_SIZE with SNAPSHOTS_EVENT_SIZE
//                      - Revised TP_DisplayWaveform() to add ADC voltages to "DWU" command
//    69    230828  DAH - Revised TP_RTC() to set flag to send time to the display processor if new time is
//                        entered
//    72    230906  DAH - Added DQ command to display firmware revisions of all three processors.  Note,
//                        presently both the protection processor and display processor revision info is
//                        stored as constants in this file (PROT_PROC_FW_VER, DISP_PROC_FW_VER, etc.).  This
//                        will need to be changed in the future
//                          - TP_DisplayFWversions() added
//                          - Display proc and protection proc FW rev constants added as described above
//                          - DQ command added to TP_Top()
//    82    230928  DAH - Revised TP_DisplayFWversions() to replace display processor firmware ver/rev
//                        constants with values received from the display micro
//                      - Deleted protection proc FW ver/rev constants.  Constants moved to main_def.h,
//                        which is now included in this file
//    93    231010  BP  - Added Test Current injection code for Override micro trips
//    94    231010  DAH - Modified TP_OffsetCal() per the new user waveform capture process
//                      - In TP_ExecuteAction(), modified EAW1 and EAW2 commands per the new user waveform
//                        capture process
//                      - Deleted state TP_EA_WFDONE from TP_Top() as it is no longer used
//                      - Modified Cal_Offset_AFE(), Cal_Offset_HG(), and Cal_Offset_LG() per the new user
//                        waveform capture process
//    95    231011  DAH - Revised TP_ExecuteAction() to delete EAP7 and EAP8 commands.  These commands were
//                        used in early power measurements to disable/enable SPI communications with the
//                        AFE.  TP_AFECommsOff was deleted as it is no longer used
//                      - Deleted TP_AFEIntOff from Test_VarInit().  It was moved to Meter_VarInit()
//    101   231027  DAH - Revised TP_OffsetCal() to ensure state machine is exited and does not hang
//    104   231102  BP  - Added scaling for Mag Narrow Sec Inj
//    108   231108  DAH - Revised TP_TestInjOffsetCal() and TP_TestInjGainCal() to disable protection when
//                        calibrating
//                      - Revised TP_TestInjOffsetCal(), TP_TestInjGainCal(), Cal_Offset_SI(), and
//                        Cal_Gain_SI() to disable thermal memory when calibrating, and added delay to
//                        ensure currents are back to normal before reenabling protection
//                      - Fixed minor bug in Cal_Gain_SI()
//                      - Call to Test_VarInit() was moved into initialization because test injection flags
//                        need to be cleared before we start sampling.  Deleted call to Test_VarInit() in
//                        TP_Top()
//    137   240102  DAH - Revised TP_DisplayStartup() to:
//                          - display the internal timing parameters in addition to the existing parameters
//                          - display the percent bucket value in addition to the ADC value if thermal
//                            memory is enabled
//                          - display a "thermal memory is disabled" string if thermal memory is disabled
//                          - display labels for the values
//    142   240119  DAH - Revised TP_ModifyCal(), TP_ExecuteAction(), and Write_Default_Cal() to support
//                        additional phase cal constants (50Hz and 60Hz supported)
//                      - Fixed minor bugs in Load_ExecuteAction_struct(), Cal_Gain_AFE(), Cal_Gain_HG(),
//                        and Cal_Gain_LG()
//    149   240131  DAH - Fixed minor bugs in Cal_Offset_HG() and Cal_Offset_LG()
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
#define USR_TP                  0        // *** DAH  HERE JUST SO WILL COMPILE FOR NOW
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Demand_def.h"
#include "Meter_def.h"
#include "Intr_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"
#include "CAMCom_def.h"
#include "DispComm_def.h"
#include "Init_def.h"
#include "main_def.h"
#include "Flags_def.h"
#include "Prot_def.h"


//
//      Local Definitions used in this module...
//
enum DisplayRT0_States
{
  TP_DRGR0_0, TP_DRGR0_1, TP_DRGR0_2, TP_DRGR0_3, TP_DRGR0_4
};

enum DisplayRT1_States
{
  TP_DRGR1_0,  TP_DRGR1_1,  TP_DRGR1_2,  TP_DRGR1_3,  TP_DRGR1_4,  TP_DRGR1_5,  TP_DRGR1_6,  TP_DRGR1_7,
  TP_DRGR1_8,  TP_DRGR1_9,  TP_DRGR1_10, TP_DRGR1_11, TP_DRGR1_12, TP_DRGR1_13, TP_DRGR1_14, TP_DRGR1_15,
  TP_DRGR1_16
};

enum DisplayRT2_States
{
  TP_DRGR2_0, TP_DRGR2_1, TP_DRGR2_2, TP_DRGR2_3, TP_DRGR2_4, TP_DRGR2_5, TP_DRGR2_6
};

enum DisplayRT3_States
{
  TP_DRGR3_0,  TP_DRGR3_1,  TP_DRGR3_2,  TP_DRGR3_3,  TP_DRGR3_4
};

enum DisplayRT4_States
{
  TP_DRGR4_0,  TP_DRGR4_1,  TP_DRGR4_2,  TP_DRGR4_3,  TP_DRGR4_4,  TP_DRGR4_5,  TP_DRGR4_6,  TP_DRGR4_7,
  TP_DRGR4_8,  TP_DRGR4_9,  TP_DRGR4_10, TP_DRGR4_11, TP_DRGR4_12
};

enum DisplayRT5_States
{
  TP_DRGR5_0,  TP_DRGR5_1,  TP_DRGR5_2,  TP_DRGR5_3,  TP_DRGR5_4,  TP_DRGR5_5,  TP_DRGR5_6,  TP_DRGR5_7,
  TP_DRGR5_8,  TP_DRGR5_9,  TP_DRGR5_10, TP_DRGR5_11, TP_DRGR5_12, TP_DRGR5_13, TP_DRGR5_14, TP_DRGR5_15,
  TP_DRGR5_16, TP_DRGR5_17, TP_DRGR5_18, TP_DRGR5_19, TP_DRGR5_20, TP_DRGR5_21, TP_DRGR5_22, TP_DRGR5_23,
  TP_DRGR5_24, TP_DRGR5_25, TP_DRGR5_26, TP_DRGR5_27, TP_DRGR5_28, TP_DRGR5_29, TP_DRGR5_30, TP_DRGR5_31,
  TP_DRGR5_32, TP_DRGR5_33, TP_DRGR5_34
};

enum DisplayEnergy_States
{
  TP_DE0, TP_DE1, TP_DE2, TP_DE3
};

enum ModifyCal_States
{
  TP_MC0,  TP_MC1,  TP_MC2,  TP_MC3,  TP_MC4,  TP_MC5,  TP_MC6,  TP_MC7,  TP_MC8,  TP_MC9,  TP_MC10,
  TP_MC11, TP_MC12, TP_MC13, TP_MC14
};

enum TP_ModifyProtSetting_States                                // *** DAH 210730  FOR TEST PURPOSES ONLY - DELETE LATER
{
  TP_MT0,  TP_MT1
};

enum RTC_States
{
  TP_TM0, TP_TM1, TP_TM2, TP_TM3, TP_TM4, TP_TM5, TP_TM6
};

enum DisplayWaveform_States
{
  TP_DW0, TP_DW1, TP_DW2, TP_DW3, TP_DW4
};

enum THSensor_States
{
  TP_TH0, TP_TH1
};

enum OffsetCal_States
{
  TP_OC0, TP_OC1, TP_OC2, TP_OC3, TP_OC4, TP_OC5, TP_OC6
};

enum GainCal_States
{
  TP_GC0, TP_GC1, TP_GC2, TP_GC3
};

enum PhaseCal_States
{
  TP_PC0, TP_PC1
};

enum TestInjectionOffsetCal_States
{
  TP_IOC0, TP_IOC1, TP_IOC2, TP_IOC3, TP_IOC4
};

enum TestInjectionGainCal_States
{
  TP_ISC0, TP_ISC1, TP_ISC2, TP_ISC3, TP_ISC4, TP_ISC5, TP_ISC6,
  TP_IDC0, TP_IDC1, TP_IDC2, TP_IDC3, TP_IDC4, TP_IDC5, TP_IDC6
};

enum TP_EEPOT_States
{
  TP_EE0, TP_EE1, TP_EE2, TP_EE3
};

enum TP_ChangeCAMtype_States
{
  TP_CC0, TP_CC1, TP_CC2
};

enum TP_DisplayInternalTiming_States
{
  TP_DS0, TP_DS1, TP_DS2, TP_DS3, TP_DS4
};

enum DisplayMemory_States 
{
  TP_DM0, TP_DM1_0, TP_DM1_1, TP_DM1_2, TP_DM1_3, TP_DM2_0, TP_DM2_1, TP_DM2_2, TP_DM2_3, TP_DM3_0,
  TP_DM3_1, TP_DM3_2, TP_DM3_3, TP_DM4_0, TP_DM4_1, TP_DM4_2, TP_DM4_3
}; 

enum DisplayDemandLog_States 
{
  TP_DD0, TP_DD1, TP_DD2, TP_DD3
};

enum TP_RestoreUnit_States
{
  TP_RU0, TP_RU1
};

enum TP_DisplayWF_States
{
  TP_DA0, TP_DA1, TP_DA2, TP_DA3, TP_DA4, TP_DA5, TP_DA6, TP_DA7, TP_DA8, TP_DA9, TP_DA10, TP_DA11, TP_DA12,
  TP_DA13, TP_DA14, TP_DA15, TP_DA16, TP_DA17, TP_DA18, TP_DA19, TP_DA20, TP_DA21, TP_DA22, TP_DA23, TP_DA24, TP_DA25,TP_DA26, TP_DA27
};

enum DisplayExtCap_States
{
  TP_MX_0, TP_MX_1, TP_MX_2, TP_MX_3, TP_MX_4, TP_MX_5, TP_MX_6
};

enum DisplayFWversion_States
{
  TP_DQ0, TP_DQ1, TP_DQ2, TP_DQ3, TP_DQ4, TP_DQ5, TP_DQ6, TP_DQ7, TP_DQ8, TP_DQ9, TP_DQ10, TP_DQ11, TP_DQ12,
  TP_DQ13, TP_DQ14, TP_DQ15, TP_DQ16, TP_DQ17, TP_DQ18, TP_DQ19, TP_DQ20, TP_DQ21, TP_DQ22, TP_DQ23,
  TP_DQ24, TP_DQ25, TP_DQ26, TP_DQ27, TP_DQ28
};



// Test Injection
#define INJ_CAL_DEFAULT_OFFSET      2441
#define INJ_CAL_DEFAULT_DCGAIN      0
#define INJ_CAL_DEFAULT_SINEGAIN    0


 


//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Iod_ext.h"
#include "RealTime_ext.h"
#include "Intr_ext.h"
#include "Meter_ext.h"
#include "Init_ext.h"
#include "CAMCom_ext.h"
#include "DispComm_ext.h"
#include "Events_ext.h"
#include "Demand_ext.h"
#include "Setpnt_ext.h"
#include "Ovrcom_ext.h"
#include "Setpnt_ext.h"
#include "Ovrcom_ext.h"
#include "Prot_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void TP_Top(void);
void ExAct_Top(void);
uint8_t Load_ExecuteAction_struct(uint8_t bid, uint8_t *msg);
void TestInjCur(uint8_t channel, float testcurrent);

void TestInjCur_OvrMicro(uint8_t channel, float testcurrent);

//      Local Function Prototypes (These functions are called only within this module)
//
void Test_VarInit(void);
uint8_t TP_ParseChars(void);
uint32_t TP_GetDecNum(void);
uint32_t TP_GetHexNum(void);
void TP_TestLEDs(void);
void TP_TestAvailabilityLEDs(void);
void TP_ExecuteAction(void);
void TP_DisplayRTValues(void);
void TP_DisplayRT0Values(void);
void TP_DisplayRT1Values(void);
void TP_DisplayRT2Values(void);
void TP_DisplayRT3Values(void);
void TP_DisplayRT4Values(void);
void TP_DisplayRT5Values(void);
void TP_ModifyCal(void);
uint16_t dhex_ascii(uint8_t num);
void TP_RTC(void);
void TP_DisplayWaveform(void);
void TP_THSensor(void);
void TP_OffsetCal(void);
void TP_GainCal(void);
void TP_PhaseCal(void);
void TP_TestInjCal(void);
void TP_TestInjOffsetCal(void);
void TP_TestInjGainCal(void);
void TP_TestInj(void);
void TP_DisplayEnergy(void);
void TP_EEPOT(void);
void TP_AFESetPGA(void);
void TP_TestIndicators(void);
void TP_ChangeCAMtype(void);
void TP_ParseUInt16(uint8_t TPindex, uint16_t SPIindex);
void TP_ParseUInt32(uint8_t TPindex, uint16_t SPIindex);
void TP_ParseFloat(uint8_t TPindex, uint16_t SPIindex);
uint16_t TP_PutSpace(uint8_t Num, uint16_t TP_Index);
void TP_ParseEventsTable(uint32_t index);
void TP_DisplayStartup(void);
void TP_DisplayMemory(void) ;
void TP_AuxRelays(void) ;
void TP_DisplayDmnd(void);
void TP_DisplayWF(void);
uint16_t asciidecimal_hex(uint8_t end_indx);
void TP_RestoreUnit(void);
void TP_DisplayGF_CTstatus(unsigned char input);
void TP_SetReset_GF_CT_EN(void);
void TP_DisplayN_CTstatus(unsigned char input);
void TP_SetReset_N_CT_EN(void);

void TP_ModifyProtSetting(void);            // *** DAH 210730  FOR TEST PURPOSES ONLY - DELETE LATER

void TP_DisplaySummaryEvent(void);

void TP_DisplayTripEvent(void);
void TP_DisplayTestTripEvent(void);
void TP_DisplayAlarmEvent(void);
void TP_DisplayTimeAdjEvent(void);
void TP_DisplayDisturbanceEvent(void);

void TP_LookUpEvent(void);

void TP_LookUpSumEvent(void);
void TP_LookUpTripEvent(void);
void TP_LookUpTestTripEvent(void);
void TP_LookUpAlarmEvent(void);
void TP_LookUpDemandEvent(void);
void TP_LookUpTripWFEvent(void);
void TP_LookUpAlarmWFEvent(void);
void TP_LookUpExtCapWFEvent(void);
void TP_DisplayExtendedValues(uint8_t TypeOfRecord);

void TP_DisplayFWversions(void);

void Cal_Gain_AFE(void);
void Cal_Offset_AFE(void);
void Cal_Gain_HG(void);
void Cal_Offset_HG(void);
void Cal_Gain_LG(void);
void Cal_Offset_LG(void);
void Cal_Offset_SI(void);
void Cal_Gain_SI(void);
uint8_t WriteCal(void);
void Write_Default_Cal(uint16_t cal_type);
void Service_LED_test(void);
void Service_Relay_test(void);




//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct TESTPORTVARS TP;
struct EXACTVARS    ExAct;


float TP_AFEcalgain[4];
struct TEST_INJECTION TestInj;
struct TESTINJ_CAL TestInjCal;
uint8_t TP_AFEIntOff;                   // *** DAH TEST
union tp_buf tbuf;

uint8_t gtest;         // *** DAH TEST  210420
uint8_t gtestcnt;      // *** DAH TEST  210420



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t TP_AFEPGASetting;
uint8_t DW_temp;                        // *** DAH THIS WILL EVENTUALLY BE DELETED

//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//
const unsigned char NAMECOPYRIGHTDATE[] = "PXR35 \n\rCopyright 2024 Eaton Corporation Pittsburgh, "
                                           "Pennsylvania\n\rAll rights reserved\n\r152 240207\n\r";
const unsigned char CURSOR[] = "\n\r> ";
const unsigned char GAIN[] = {'G', 'a', 'i', 'n'};
const unsigned char OFFSET[] = {'O', 'f', 'f', 's', 'e', 't'};
const unsigned char PHASE[] = {'P', 'h', 'a', 's', 'e'};

const unsigned char NUM_SAMPLES[] = {'\r', '\n', 'N', 'u', 'm', ' ', 'S', 'a', 'm', 'p', 'l', 'e', 's', ':', ' '};
const unsigned char TIME_STAMP[] = {'\r', '\n', 'T', 'S', ':', ' '};
const unsigned char EID_LABEL[] = {'\r', '\n', 'E', 'I', 'D', ':', ' '};
const unsigned char NUM_WFLOGS[] = {'\r', '\n', 'N', 'u', 'm', ' ', 'L', 'o', 'g', 's', ':', ' '};
const unsigned char MOST_RECENT_LOG[] = {'\r', '\n', 'L', 'a', 's', 't', ' ', 'L', 'o', 'g', ':', ' '};

const unsigned char PROT_PROC_FW_VER_STR[] = {"Prot Processor Firmware Version: "};
const unsigned char PROT_PROC_FW_REV_STR[] = {"Prot Processor Firmware Revision: "};
const unsigned char PROT_PROC_FW_BUILD_STR[] = {"Prot Processor Firmware Build: "};
const unsigned char DISP_PROC_FW_VER_STR[] = {"Disp Processor Firmware Version: "};
const unsigned char DISP_PROC_FW_REV_STR[] = {"Disp Processor Firmware Revision: "};
const unsigned char DISP_PROC_FW_BUILD_STR[] = {"Disp Processor Firmware Build: "};
const unsigned char OVR_MICRO_FW_VER_STR[] = {"Override Micro Firmware Version: "};

const unsigned char STARTUP_ADC_COUNT_STR[] = {'S', 't', 'a', 'r', 't', 'u', 'p', ' ', 'A', 'D', 'C', ' ',
                                               'V', 'a', 'l', ':', ' '};
const unsigned char STARTUP_TIME[] = {'S', 't', 'a', 'r', 't', 'u', 'p', ' ', 'T', 'i', 'm', 'e', ':', ' '};
const unsigned char STARTUP_SCALE_STR[] = {'S', 't', 'a', 'r', 't', 'u', 'p', ' ', 'S', 'c', 'a', 'l', 'e',
                                           ' ', 'V', 'a', 'l', ':', ' '};
const unsigned char THERM_ADC_COUNT_STR[] = {'T', 'h', 'e', 'r', 'm', ' ', 'm', 'e', 'm', ' ', 'A', 'D',
                                             'C', ' ', 'V', 'a', 'l', ':', ' '};
const unsigned char THERM_PCT_STR[] = {'T', 'h', 'e', 'r', 'm', ' ', 'm', 'e', 'm', ' ', 'P', 'e', 'r', 'c',
                                       'e', 'n', 't', ':', ' '};
const unsigned char THERM_MEM_DISABLED_STR[] = {'T', 'h', 'e', 'r', 'm', ' ', 'm', 'e', 'm', ' ', 'd', 'i',
                                                's', 'a', 'b', 'l', 'e', 'd', '\n', '\r'};
const unsigned char RESET_TO_PLL_COUNT_STR[] = {'R', 'e', 's', 'e', 't', ' ', 't', 'o', ' ', 'P', 'L', 'L',
                                                ' ', 'c', 'o', 'u', 'n', 't', ':', ' '};
const unsigned char MAX_LOOP_TIME_STR[] = {'M', 'a', 'x', ' ', 'l', 'o', 'o', 'p', ' ', 't', 'i', 'm', 'e',
                                           ':', ' '};

unsigned char const * const FW_STR_PTR[] =
{
        &PROT_PROC_FW_VER_STR[0], &PROT_PROC_FW_REV_STR[0], &PROT_PROC_FW_BUILD_STR[0],
        &DISP_PROC_FW_VER_STR[0], &DISP_PROC_FW_REV_STR[0], &DISP_PROC_FW_BUILD_STR[0],
        &OVR_MICRO_FW_VER_STR[0]
};

const uint32_t PROT_PROC_FW_VERREVBUILD[] =
{
        PROT_PROC_FW_VER, PROT_PROC_FW_REV, PROT_PROC_FW_BUILD
};

void * const DISP_OVR_PROC_FWREV[] =
{
        &DispProc_FW_Rev, &DispProc_FW_Ver, &DispProc_FW_Build, &Ovr_FW_Version
};



// CAM Test Transmission for EAC1 and EAC2 commands - must not exceed length of CAM Tx buffer!
const unsigned char CAM_TEST_STRING[] = "CAM TEST TRANSMISSION EXECUTED";


extern uint8_t dfred_on;   // *** DAH TEST

//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Test_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Port Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Test Module.
//                      The following variables do not need initialization:
//                          TP.RxBuf[]
//                          TP.TxValBuf[]
//                          *TP.StrPtr
//                          TP.NumChars
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            TP.State, TP.RxNdxIn, TP.RxNdxOut, TP.TxValNdx, TP.Status, TP_AFEPGASetting,
//                      TestInj.x
//
//  ALTERS:             None
//
//  CALLS:              ReadInjCalConstants()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 481usec
//
//------------------------------------------------------------------------------------------------------------

void Test_VarInit(void)
{
  TP.State = TP_INIT;                   // Initialize test port state to initialization state
  TP.RxNdxIn = 0;                       // Initialize indices to 0
  TP.RxNdxOut = 0;
  TP.TxValNdx = 0;
  TP.Status = 0;                        // Clear status flags

  TP_AFEPGASetting = 1;

//  ReadInjCalConstants();                // Read the Test Injection cal constants
  TestInj.Flags = 0;
  TestInj.CosIndx = 20;                 // Initialize cosine index to 20 (COS[n] = SIN[n+20])
  TestInj.Amplitude = 0;
  TestInj.MidPoint = TestInjCal.midpoint_ph;

  ExAct.State = IDLE;
  ExAct.LED_image = NO_MANUF_TEST;
  ExAct.Relay_image = NO_MANUF_TEST;
 
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Test_VarInit()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_Top()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Top-Level Subroutine for the Test and Calibration Port
//
//  MECHANICS:          This subroutine is the top-level subroutine for the test and calibration serial
//                      port.  It operates in either IDLE or ACTIVE state.
//                      In the IDLE state, the subroutine:
//                        - Checks to see whether any characters have been received
//                        - If a carriage return has been received, output the header information, and enter
//                          the active state.  Otherwise, ignore the character and remain in the IDLE state.
//                      In the ACTIVE state, the subroutine:
//                        - Parses any and all received characters, decodes the command, and assembles and 
//                          transmits the response
//                        - If not processing a command, see whether the idle timer has expired.  If it has,
//                          reset password protection and return to the IDLE state.
//                      Note, when transmitting a response, the transmit buffer is loaded with a character
//                      each time the subroutine is called until there are no more characters to transmit.
//                      There is no need to check whether the previous character has been transmitted before
//                      loading the next character.  At 9600 baud, a character will be sent in about 1msec. 
//                      This subroutine is called on the half-cycle anniversary, or every 8msec.  We can
//                      always assume the previous character has been transmitted and the transmit buffer is
//                      empty when the subroutine is called.
//
//  CAVEATS:            None
//
//  INPUTS:             TP_RxNdxIn, TP_RxBuf[]
// 
//  OUTPUTS:            TP.StrPtr, TP.Status
//
//  ALTERS:             TP_State, TP_RxNdxOut, StatusFlags
// 
//  CALLS:              TP_TestLEDs(), TP_ExecuteAction(), TP_DisplayRTValues(), TP_DisplayRT0Values(),
//                      TP_DisplayRT1Values(), TP_DisplayRT2Values(), TP_ModifyCal(), TP_RTC(),
//                      TP_DisplayWaveform(), TP_THSensor(), TP_OffsetCal(), TP_GainCal(),
//                      TP_TestIndicators(), TP_RestoreUnit(), TP_DisplayWF(), FRAM_Read(), ClearLogFRAM()
// 
//------------------------------------------------------------------------------------------------------------

void TP_Top(void)
{
  uint8_t i, TP_exit;
  uint16_t cmnd;
  uint32_t chk;

/*  if ( (TP_IdleTimer == 0)                      // Check for timeout.  If timeout, start over
    && (!(StatusFlags & TP_NO_IDLETIMEOUT)) )   //   If flag set, no idle timeout
  {
    TP_State = TP_IDLE;
  }     */

  TP_exit = FALSE;

  while (!TP_exit)
  {
    switch (TP.State)
    {
      case TP_INIT:                       // Initialization State
      default:
        // Read the test injection calibration constants, checksum, and checksum complement from FRAM.
        //   Store these in the permanent structure (TestInjCal).  We will assume these are good
        //   Note, the FRAM values ahould always be checked and used over the Flash values, because the
        //   values are written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most
        //   current.
        FRAM_Read(FRAM_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&TestInjCal.midpoint_ph));

        // Compute the checksum and complement values
        chk = ComputeChksum32((uint32_t *)(&TestInjCal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));

        // If there is a checksum error, set the error flag and initialize the cal constants to defaults
        //   This ensures defaults will be used if something goes wrong getting the Flash copies
        if ( (TestInjCal.chk != chk) || (TestInjCal.cmp != ~chk) )
        {
          SystemFlags |= INJ_FRAM_ERR;
          TestInjCal.midpoint_ph = INJ_CAL_DEFAULT_OFFSET;
          TestInjCal.midpoint_gnd = INJ_CAL_DEFAULT_OFFSET;
          for (i=0; i<4; i++)
          {
            TestInjCal.m_dc[i] = INJ_CAL_DEFAULT_DCGAIN;
            TestInjCal.b_dc[i] = INJ_CAL_DEFAULT_DCGAIN;
            TestInjCal.m_sine[i] = INJ_CAL_DEFAULT_SINEGAIN;
            TestInjCal.b_sine[i] = INJ_CAL_DEFAULT_SINEGAIN;
          }
        }
        // If the checksum is good, keep the cal constants that were read and clear the error flag
        else
        {
          SystemFlags &= (uint16_t)(~INJ_FRAM_ERR);
        }
        // Set flag to retrieve the test injection cal constants from Flash.  This routine will use the
        //   Flash cal constants if they are good and the Fram ones are bad.  This means:
        //      - If FRAM values are good, they are used
        //      - If FRAM values are bad and Flash values are good, Flash values are used
        //      - If both FRAM and Flash values are bad, default values are used
        // We want to read the Flash values even if the FRAM values are good because we check them and set
        // a flag if they are bad
        SPI1Flash.Req |= S1F_INJ_RD;            
        TP.State = TP_INIT1;
        break;
        
      case TP_INIT1:                      // Initialization State 1
        if (SPI1Flash.Ack & S1F_INJ_RD)               // If the reads are done, clear the request
        {
          SPI1Flash.Req &= (uint32_t)(~S1F_INJ_RD);
          TP.State = TP_IDLE;
        }
        TP_exit = TRUE;                             // Exit in any case
        break;

      case TP_IDLE:                       // Idle State
//      TP.Status &= (~PASSWORD_OK);                // Cancel password protection
//      TP_LEDCode = 0;                             // Cancel LED test
        while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a line feed is received
             && (TP.State == TP_IDLE) )             //   or there are no characters left to parse
        {
          if (TP.RxBuf[TP.RxNdxOut] == 0x0A)        // If LF received, set up to output header string
          {
            TP.StrPtr = &NAMECOPYRIGHTDATE[0];
            TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
            TP.Status |= TP_TX_STRING;              // Set flag to transmit string
            UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
            TP.State = TP_CURSOR;
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received valid char(s)
          }
          TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;   // Increment the index
        }
        TP_exit = TRUE;
        break;

      case TP_CURSOR:                     // Output Cursor
        // If done transmitting, output a cursor
        if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
        {
          TP.StrPtr = &CURSOR[0];
          TP.Status &= (~TP_TX_VALUE);              // Make sure flag to transmit values is clear
          TP.Status |= TP_TX_STRING;                // Set flag to transmit string
          UART5->CR1 |= USART_CR1_TXEIE;            // Enable transmit interrupts
          TP.RxNdxOut = TP.RxNdxIn;                 // Clear out any spurious characters
          TP.State = TP_CHECK_FOR_CMND;
//        TP_IdleTimer = TP_IDLETIMEOUT;            // Reset the idle timer - we have received valid char(s)
        }
        TP_exit = TRUE;
        break;                                      // Exit the routine in either case

      case TP_CHECK_FOR_CMND:             // Check For Command String
        i = TP.RxNdxOut;
        while ( (i != TP.RxNdxIn)                   // Check the Rx buffer for a line feed - this indicates
             && (TP.State == TP_CHECK_FOR_CMND) )   //   that a complete command string has been entered
        {
          if (TP.RxBuf[i] == 0x0A)                  // If LF received, go to next state to parse the command
          {
            TP.State = TP_PARSECMNDSTRING;
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          i = (i + 1) & 0x1F;                       // Increment the index
        }
        TP_exit = TRUE;
        break;

      case TP_PARSECMNDSTRING:            // Parse Command String
        // Parse the command string to get the two-char command.  Ignore all chars that are not letters.
        //   Once a letter is found, capture it and the next character.
        // TestPort.RxNdxIn is the location of the next character to receive - it is one plus the last
        //   received char location.  TestPort.RxNdxOut is the index of the first char in the command
        //   string.
        TP.State = TP_CURSOR;                       // Assume bad command - initialize next state to idle
        while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until two valid chars are
             && (TP.State == TP_CURSOR)             //   received, a line feed is received, or there are no
             && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   chars left to parse
        {
          i = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
          TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
          if ( (i >= 'A') && (i <= 'Z') )           // If valid, create command string from this char and
          {                                         //   the next char
            cmnd = ((uint16_t)(i) << 8) + (TP.RxBuf[TP.RxNdxOut] & 0xDF);
            TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
            switch (cmnd)
            {
              case ('T' * 256 + 'L'):               // Test LEDs command string
                TP.State = TP_TL;
                break;

              case ('T' * 256 + 'I'):               // Test Indicator LEDs command string
                TP.State = TP_TI;
                break;

              case ('T' * 256 + 'V'):               // Test Protection Availability LEDs command string
                TP.State = TP_TV;
                break;

              case ('E' * 256 + 'A'):               // Execute Action command string
                TP.State = TP_EA;
                break;

              case ('D' * 256 + 'R'):               // Display Real-Time values command string
                TP_DisplayRTValues();                   // Call subroutine to set the next state according
                break;                                  //   to the group number

              case ('D' * 256 + 'E'):               // Display Energy command string
                TP.State = TP_DE;
                TP.SubState = TP_DE0;
                break;

              case ('D' * 256 + 'Q'):               // Display Firmware Versions
                TP.State = TP_DQ;
                TP.SubState = TP_DQ0;
                break;

              case ('M' * 256 + 'C'):               // Modify Calibration Constants command string
                TP.State = TP_MC;
                TP.SubState = TP_MC0;
                break;

              case ('M' * 256 + 'T'):               // Modify Instantaneous Protection Setting   *** DAH 210730   FOR TEST PURPOSES ONLY - DELETE LATER
                TP.State = TP_MT;
                TP.SubState = TP_MT0;
                break;

              case ('T' * 256 + 'M'):               // Time command string
                TP.State = TP_TM;
                TP.SubState = TP_TM0;
                break;

              case ('D' * 256 + 'W'):               // Display Waveform command string
                TP.State = TP_DW;
                TP.SubState = TP_DW0;
                break;

              case ('T' * 256 + 'H'):               // Read Temperature/Humidity Sensor Command
                TP.State = TP_TH;
                TP.SubState = TP_TH0;
                break;

              case ('O' * 256 + 'C'):               // Offset Calibration Command
                TP.State = TP_OC;
                TP.SubState = TP_OC0;
                break;

              case ('G' * 256 + 'C'):               // Gain Calibration Command
                TP.State = TP_GC;
                TP.SubState = TP_GC0;
                break;

              case ('P' * 256 + 'C'):               // Phase Calibration Command
                TP.State = TP_PC;
                TP.SubState = TP_PC0;
                break;

              case ('I' * 256 + 'C'):               // Test Injection Calibration Command
                TP_TestInjCal();                        // Call subroutine to set the next state according
                break;                                  //   according to the next parameter

              case ('I' * 256 + 'T'):               // Test Injection Command
                TP.State = TP_IT;
                break;

              case ('E' * 256 + 'E'):               // Modify Frame EEPOT Settings command
                TP.State = TP_EE;
                TP.SubState = TP_EE0;
                break;

              case ('A' * 256 + 'G'):               // Set AFE PGA command
                TP.State = TP_AG;
                break;

              case ('C' * 256 + 'C'):               // Change CAM Style command
                TP.State = TP_CC;
                TP.SubState = TP_CC0;
                break;

              case ('D' * 256 + 'S'):               // Display Startup Values command string
                TP.State = TP_DS;
                TP.SubState = TP_DS0;
                break;

              case ('D' * 256 + 'M'):               // Display Memory Command 
                TP.State = TP_DM;
                TP.SubState = TP_DM0;
                break;

              case ('T' * 256 + 'R'):               // Test Aux Relays Command 
                TP.State = TP_TR;
                break;

              case ('R' * 256 + 'U'):               // Restore Unit Command 
                TP.State = TP_RU;
                TP.SubState = TP_RU0;
                break;

              case ('D' * 256 + 'D'):               // Display Demand and Energy Logs Command
                TP.State = TP_DD;
                TP.SubState = TP_DD0;
                break;

              case ('D' * 256 + 'A'):               // Display Alarm Waveform Command
                TP.State = TP_DA;
                TP.SubState = TP_DA0;
                break;

              case ('D' * 256 + 'T'):               // Display Trip Waveform Command
                TP.State = TP_DA;
                TP.SubState = TP_DA7;
                break;

              case ('D' * 256 + 'C'):               // Display Strip-Chart Waveform Command
                TP.State = TP_DA;
                TP.SubState = TP_DA14;
                break;

              case ('D' * 256 + 'X'):               // Display Extended Waveform Command
                TP.State = TP_DX;
                TP.SubState = TP_DA21;
                break;

              case ('D' * 256 + 'V'):               // Display Summary Event Command
                TP.State = TP_DV;
                TP.SubState = 0;
                break;

              case ('D' * 256 + 'I'):               // Display Trip Event Command
                TP.State = TP_DI;
                TP.SubState = 0;
                break;

              case ('D' * 256 + 'F'):               // Display Test Trip Event Command
                TP.State = TP_DF;
                TP.SubState = 0;
                break;

              case ('D' * 256 + 'N'):               // Display Alarm Event Command
                TP.State = TP_DN;
                TP.SubState = 0;
                break;

              case ('D' * 256 + 'J'):               // Display Time Adj Command
                TP.State = TP_DJ;
                TP.SubState = 0;
                break;
                
              case ('D' * 256 + 'Z'):               // Display Disturbance Capture log
                TP.State = TP_DZ;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'S'):               // Lookup index by EID in Summary Events
                TP.State = TP_LS;
                TP.SubState = 0;
                break;
                
              case ('L' * 256 + 'T'):               // Lookup index by EID in Trip Events
                TP.State = TP_LT;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'R'):               // Lookup index by EID in Trip Events
                TP.State = TP_LR;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'M'):               // Lookup index by EID in MajAlarm Events
                TP.State = TP_LM;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'D'):               // Lookup index by EID in Demand Events
                TP.State = TP_LD;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'U'):               // Lookup index by EID in TripWF Events
                TP.State = TP_LU;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'V'):               // Lookup index by EID in AlarmWF Events
                TP.State = TP_LV;
                TP.SubState = 0;
                break;

              case ('L' * 256 + 'W'):               // Lookup index by EID in CharTWF Events
                TP.State = TP_LW;
                TP.SubState = 0;
                break;

              case ('M' * 256 + 'X'):               // Extended Capture One Cycle values
                TP.State = TP_MX;
                TP.SubState = TP_DRGR0_0;
                break;

              case ('M' * 256 + 'Z'):               // Extended Capture One Cycle values
                TP.State = TP_MZ;
                TP.SubState = TP_DRGR0_0;
                break;

              case ('G' * 256 + 'F'):               // Display/Change Ground Fault Configuration Command
                TP_DisplayGF_CTstatus(TRUE);            // Display the existing GF configuration
                TP.RxNdxOut = 0;                        // Set RxNdxOut equal to RxNdxIn to ensure a new
                TP.RxNdxIn = 0;                         //   start to parsing
                TP.State = TP_GF;
                break;

              case ('N' * 256 + 'F'):               // Display/Change Neutral Sensing Configuration Command
                TP_DisplayN_CTstatus(TRUE);            // Display the existing neutral configuration
                TP.RxNdxOut = 0;                        // Set RxNdxOut equal to RxNdxIn to ensure a new
                TP.RxNdxIn = 0;                         //   start to parsing
                TP.State = TP_NF;
                break;

              default:                              // Invalid command string
                break;
            }
          }
        }
        TP_exit = TRUE;
        break;

      case TP_TL:                         // Test LEDs command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_TestLEDs();
        break;

      case TP_TI:                         // Test Indicator LEDs command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_TestIndicators();
        break;

      case TP_TV:                         // Test Protection Availability LEDs command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_TestAvailabilityLEDs();
        break;

      case TP_EA:                         // Execute Action command - get which command
        TP.State = TP_CURSOR;                 // Next state is idle, unless changed in the subroutine
        TP_ExecuteAction();
        break;

      case TP_DR0:                        // Display Group 0 Real-Time Values command
        TP_DisplayRT0Values();
        TP_exit = TRUE;
        break;

      case TP_DR1:                        // Display Group 1 Real-Time Values command
        TP_DisplayRT1Values();
        TP_exit = TRUE;
        break;

      case TP_DR2:                        // Display Group 2 Real-Time Values command
        TP_DisplayRT2Values();
        TP_exit = TRUE;
        break;

      case TP_DR3:                        // Display Group 3 Real-Time Values command
        TP_DisplayRT3Values();
        TP_exit = TRUE;
        break;

      case TP_DR4:                        // Display Group 4 Real-Time Values command
        TP_DisplayRT4Values();
        TP_exit = TRUE;
        break;

      case TP_DR5:                        // Display Group 5 Real-Time Values command
        TP_DisplayRT5Values();
        TP_exit = TRUE;
        break;

      case TP_DE:                         // Display Energy command
        TP_DisplayEnergy();
        TP_exit = TRUE;
        break;

      case TP_DQ:                         // Display Firmware Versions command
        TP_DisplayFWversions();
        TP_exit = TRUE;
        break;

      case TP_MC:                         // Modify Calibration Constants command
        TP_ModifyCal();
        TP_exit = TRUE;
        break;

      case TP_MX:
        TP_DisplayExtendedValues(OneCycle);
        TP_exit = TRUE;
        break;

      case TP_MZ:
        TP_DisplayExtendedValues(TwoHundred);
        TP_exit = TRUE;
        break;

      case TP_MT:                         // Modify Protection Setting command    *** DAH 210730  ADDED FOR TEST PURPOSES ONLY - DELETE LATER
        TP_ModifyProtSetting();
        TP_exit = TRUE;
        break;

      case TP_TM:                         // Test Real Time command
        TP_RTC();
        TP_exit = TRUE;
        break;

      case TP_DW:                         // Display Waveform command
        TP_DisplayWaveform();
        TP_exit = TRUE;
        break;

      case TP_TH:                         // Read Temperature/Humidity Sensor command
        TP_THSensor();
        TP_exit = TRUE;
        break;

      case TP_OC:                         // Offset Calibration command
        TP_OffsetCal();
        TP_exit = TRUE;
        break;

      case TP_GC:                         // Gain Calibration command
        TP_GainCal();
        TP_exit = TRUE;
        break;

      case TP_PC:                         // Phase Calibration command
        TP_PhaseCal();
        TP_exit = TRUE;
        break;

      case TP_IOC:                        // Test Injection Offset Calibration command
        TP_TestInjOffsetCal();
        TP_exit = TRUE;
        break;

      case TP_IGC:                        // Test Injection Gain Calibration command
        TP_TestInjGainCal();
        TP_exit = TRUE;
        break;

      case TP_EE:                         // EEPOT command
        TP_EEPOT();
        TP_exit = TRUE;
        break;

      case TP_AG:                         // Set AFE PGA command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_AFESetPGA();
        break;

      case TP_IT:                         // Test Injection command
        TP_TestInj();
        TP_exit = TRUE;
        break;

      case TP_CC:                         // Change CAM Style command
        TP_ChangeCAMtype();
        TP_exit = TRUE;
        break;

      case TP_DS:                         // Display Startup Values command
        TP_DisplayStartup();
        TP_exit = TRUE;
        break;

      case TP_DM:                         // Display Memory command
        TP_DisplayMemory(); 
        TP_exit = TRUE;
        break;

      case TP_DD:                         // Display Demand and Energy Logs command
        TP_DisplayDmnd();
        TP_exit = TRUE;
        break;

      case TP_DX:
      case TP_DA:                         // Display Alarm Waveform Log command
        TP_DisplayWF();
        TP_exit = TRUE;
        break;

      case TP_DV:                         // Display Summary Event Log command
        TP_DisplaySummaryEvent();
        TP_exit = TRUE;
        break;

      case TP_DI:                         // Display Trip Event Logcommand
        TP_DisplayTripEvent();
        TP_exit = TRUE;
        break;

      case TP_DF:                         // Display Test Trip Event Log command
        TP_DisplayTestTripEvent();
        TP_exit = TRUE;
        break;
        
      case TP_DN:                         // Display Alarm Event Log command
        TP_DisplayAlarmEvent();
        TP_exit = TRUE;
        break;

      case TP_DJ:                         // Display Time Adj Event Log command
        TP_DisplayTimeAdjEvent();
        TP_exit = TRUE;
        break;

      case TP_DZ:                         // Display Time Adj Event Log command
        TP_DisplayDisturbanceEvent();
        TP_exit = TRUE;
        break;

      case TP_LS:                         // Lookup index by EID in Summarry Events
        TP_LookUpSumEvent();
        TP_exit = TRUE;
        break;

      case TP_LT:                         // Lookup index by EID in Trip Events
        TP_LookUpTripEvent();
        TP_exit = TRUE;
        break;

      case TP_LR:                         // Lookup index by EID in TestTrip Events
        TP_LookUpTestTripEvent();
        TP_exit = TRUE;
        break;

      case TP_LM:                         // Lookup index by EID in Alarm Events
        TP_LookUpAlarmEvent();
        TP_exit = TRUE;
        break;

      case TP_LD:                         // Lookup index by EID in Demand Events
        TP_LookUpDemandEvent();
        TP_exit = TRUE;
        break;

      case TP_LU:                         // Lookup index by EID in TripWF Events
        TP_LookUpTripWFEvent();
        TP_exit = TRUE;
        break;

      case TP_LV:                         // Lookup index by EID in AlarmWF Events
        TP_LookUpAlarmWFEvent();
        TP_exit = TRUE;
        break;

      case TP_LW:                         // Lookup index by EID in ExtCapWF Events
        TP_LookUpExtCapWFEvent();
        TP_exit = TRUE;
        break;

      case TP_TR:                         // Test Auxiliary Relays command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_AuxRelays();
        break;

      case TP_RU:                         // Restore Unit command
        TP.State = TP_CURSOR;                 // Next state is idle
        TP_RestoreUnit();
//        ClearLogFRAM(0);
        break;

      case TP_WR_CAL:                     // Write (Metering) Calibration Constants to FRAM and Flash
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        TP.State = TP_WR_CAL1;                // Set next state to see if done
        TP_exit = TRUE;                       // Exit the subroutine
        break;

      case TP_WR_CAL1:                    // Wait for Writes to Complete
        if (SPI1Flash.Ack & S1F_CAL_WR)       // If the Flash writes are done, clear the request flag, write
        {                                     //   the cal constants to FRAM, and jump to the cursor state
          SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
          // Write cal constants to FRAM
          FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
          FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
          FRAM_Write(DEV_FRAM2, FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));
          FRAM_Write(DEV_FRAM2, FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));
          FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
          TP.State = TP_CURSOR;
        }
        TP_exit = TRUE;                       // Exit the subroutine
        break;

      case TP_GF:                         // Display/Change Ground Fault Configuration Command
        TP_SetReset_GF_CT_EN();
        TP_exit = TRUE;
        break;

      case TP_NF:                         // Display/Change Neutral Sensing Configuration Command
        TP_SetReset_N_CT_EN();
        TP_exit = TRUE;
        break;
    }
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          TP_Top()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_ParseChars()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Parse Characters in the Test Port Command String
//
//  MECHANICS:          This subroutine parses the command string and returns a code describing the type of
//                      value that has been entered.  Characters are examined until one of the following is
//                      found:
//                        1) A non-zero number - positive decimal number - code = 1
//                        2) A positive number equal to 0 - code = 1
//                        3) 0x or 0X - hexadecimal number - code = 2
//                        4) x or X - hexadecimal number - code = 2
//                        5) $ - hexadecimal number - code = 2
//                        6) " - string - code = 3
//                        7) A minus sign (-) followed by a number - negative decimal number - code = 4
//                        8) A <LF> is one of the first two characters found (special case) - code = 5
//                      Note, leading zeros are ignored, but if these are the only characters a code of 1 is
//                      returned.
//                      If no valid characters are found, the subroutine returns a code of 0.  Additionally,
//                      the subroutine modifies the index, so that it points to the first character in the
//                      number or string (first char after an "0x", for example) upon returning.  If the
//                      number is 0, the index points to the last 0 in the string.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP_RxNdxOut, TP.RxBuf[]
// 
//  OUTPUTS:            The subroutine returns the code indicating the type of value (see above).  It also
//                      returns the starting index of the number or string through *indx
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint8_t TP_ParseChars(void)
{
  uint8_t code, temp, flagminusfound, flagzerofound;

  // Check for a <CRLF>
  if ( (TP.RxBuf[TP.RxNdxOut] == 0x0A)
    || (TP.RxBuf[TP.RxNdxOut+1] == 0x0A) )
    return (5);
  // Parse the command string until we find either: 1) a non-zero number (decimal number), 2) a zero
  //   followed by a non-numeric number (decimal number equal to 0), 3) a minus sign followed by a number
  //   (negative decimal number), 3) a "0x" or "0X" (hexadecimal number), 4) a "$" (hexadecimal number), or
  //   5) a """ (string)
  // Ignore all chars until one of these chars are found
  code = 0;                                           // Initialize code to no valid char found
  flagminusfound = FALSE;                             // Initialize "found a minus sign" flag to False
  flagzerofound = FALSE;                              // Initialize "found a zero" flag to False
  while ( (TP.RxNdxOut != TP.RxNdxIn) && (code == 0) )
  {
    temp = TP.RxBuf[TP.RxNdxOut];                     // Set temp to next char in the command string
    // Examine the character
    if (flagminusfound)                               // If already found a "-", see if next char is a
    {                                                 //   decimal number
      if ( (temp >= '0') && (temp <= '9') )           // If next char is a number, number is a negative
      {                                               //   decimal
        code = 4;
      }
      else                                            // Otherwise reset the "found a minus" flag and inc
      {                                               //   indx to the next char
        flagminusfound = FALSE;
        TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
      }
    }
    else if  (temp == '-')                            // If char is "-", set "found a minus" flag to True and
    {                                                 //   inc index to the next char
      flagminusfound = TRUE;
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
    else if ( (temp > '0') && (temp <= '9') )         // If char is 1-9, number is a decimal
    {
      code = 1;
    }
    else if ( (temp == '$')                           // If char is a "$", "x", or "X", number is a hex, and
           || ((temp & 0xDF) == 'X') )                //   need to start at next char so inc index
    {
      code = 2;
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
    else if (temp == '\"')                            // If char is a """, it is a string, and need to
    {                                                 //   start at next char
      code = 3;
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
    else if (temp == '0')                             // If char is 0, set "found a zero" flag to True and
    {                                                 //   inc index to the next char
      flagzerofound = TRUE;
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
    else if (flagzerofound)                           // If non-numeric char found and zero was found
    {                                                 //   before, set code to 1 (decimal number) and
      code = 1;                                       //   decrement index so it points to the zero
      TP.RxNdxOut = (TP.RxNdxOut - 1) & 0x1F;
    }
    else                                              // Otherwise ignore the char and inc the index
    {
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
  }
  return (code);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_ParseChars()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_GetDecNum()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Get Decimal Number From the Command String
//
//  MECHANICS:          This subroutine parses the command string and returns the decimal number value of
//                      the characters beginning at TP.RxNdxOut.  Characters are examined until either a
//                      non-decimal number (i.e., not 0-9) is found, or the end of the command string is
//                      reached.
//                      
//  CAVEATS:            This subroutine assumes the next set of characters, beginning at *indx, comprise a
//                      decimal number.  If a non-decimal number character is the first character
//                      encountered, the subroutine returns 0.  Additionally, the result is returned in a
//                      32-bit integer.  This subroutine ignores overflows, and an incorrect value will
//                      result if an overflow occurs.
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            The subroutine returns the number that was converted from the number string (see
//                      above).  It also returns the index of the character that terminated the number
//                      string through TP.RxNdxOut.
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint32_t TP_GetDecNum(void)
{
  uint32_t res;
  uint8_t temp;

  res = 0;
  while (TP.RxNdxOut != TP.RxNdxIn)
  {
    temp = TP.RxBuf[TP.RxNdxOut];                       // Set temp to next char in the command string
    // Update the number with the char
    if ( (temp >= '0') && (temp <= '9') )               // If number is valid, update the result
    {
      res = (res * 10) + (temp - '0');
    }
    else                                                // Otherwise invalid char so done
    {
      break;
    }
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
  }
  return (res);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_GetDecNum()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_GetHexNum()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Get Hexadecimal Number From the Command String
//
//  MECHANICS:          This subroutine parses the command string and returns the hex number value of the
//                      characters beginning at *indx.  Characters are examined until either a non-hex
//                      number (i.e., not 0-9 or A-F) is found, or the end of TestPort.CmndString[] is
//                      reached.
//                      
//  CAVEATS:            This subroutine assumes the next set of characters, beginning at *indx, comprise a
//                      hexadecimal number.  If a non-hex number character is the first character
//                      encountered, the subroutine returns 0.  Additionally, the result is returned in a
//                      32-bit integer.  This subroutine ignores overflows, and an incorrect value will
//                      result if an overflow occurs.
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            The subroutine returns the number that was converted from the number string (see
//                      above).  It also returns the index of the character that terminated the number
//                      string through TP.RxNdxOut.
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint32_t TP_GetHexNum(void)
{
  uint32_t res;
  uint8_t temp, foundnonzero;

  res = 0;
  foundnonzero = FALSE;
  while (TP.RxNdxOut != TP.RxNdxIn)
  {
    temp = TP.RxBuf[TP.RxNdxOut];                       // Set temp to next char in the command string
    // Update the number with the char
    if (temp == '0')                                        // If char is a zero ...
    {
      if (foundnonzero )                                    // If not a leading zero, need to update the
        res = res * 16;                                     //   result.  Otherwise can ignore the zero
    }
    else if ( (temp > '0') && (temp <= '9') )               // Otherwise if number is valid, update the
    {                                                       //   result, and the foundnonzero flag
      res = (res * 16) + (temp - '0');
      foundnonzero = TRUE;
    }
    else if ( (temp >= 'A') && (temp <= 'F') )
    {
      res = (res *16) + (temp - 0x0037);
      foundnonzero = TRUE;
    }
    else if ( (temp >= 'a') && (temp <= 'f') )
    {
      res = (res *16) + (temp - 0x0057);
      foundnonzero = TRUE;
    }
    else                                                    // Otherwise invalid char so done
      break;
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
  }
  return (res);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_GetHexNum()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestLEDs()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test LEDs
//
//  MECHANICS:          This subroutine handles testing the LEDs
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            TP_LEDCode
//
//  ALTERS:             TP.RxNdxOut
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestLEDs(void)
{
  uint8_t i;

  while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a letter is found, a line
       && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   feed is found, or there are no chars left
  {
    i = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if ( (i >= 'A') && (i <= 'Z') )           // If valid, create command string from this char and
    {                                         //   the next char
      switch (i)
      {
        case ('S'):                           // Test Status LED command string
          i = TP_ParseChars();                    // Read next parameter - must be decimal number (return
          if (i == 1)                             //   code = 1)
          {
            TP_LEDCode = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            TP_LEDCode = 0;                       // If invalid parameter code, make normal
          }
          break;

        case ('L'):                           // Test LDPU LED command string
          i = TP_ParseChars();                    // Read next parameter - must be decimal number (return
          if (i == 1)                             //   code = 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else                                    // If invalid parameter, turn LED off
          {
            i = 0;
          }
          if (i == 0)
          {
            LDPU_LED_OFF;
          }
          else
          {
            LDPU_LED_ON;
          }
          break;
      
        case ('T'):                           // Test TRIP LED command string
          i = TP_ParseChars();                    // Read next parameter - must be decimal number (return
          if (i == 1)                             //   code = 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else                                    // If invalid parameter, turn LED off
          {
            i = 0;
          }
          if (i == 0)
          {
            TRIP_LED_OFF;
          }
          else
          {
            TRIP_LED_ON;
          }
          break;
      
        case ('A'):                           // Test ALARM LED command string
          i = TP_ParseChars();                    // Read next parameter - must be decimal number (return
          if (i == 1)                             //   code = 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else                                    // If invalid parameter, turn LED off
          {
            i = 0;
          }
          if (i == 0)
          {
            ALARM_LED_OFF;
          }
          else
          {
            ALARM_LED_ON;
          }
          break;
      
        case ('H'):                           // Test HILOAD LED command string
          i = TP_ParseChars();                    // Read next parameter - must be decimal number (return
          if (i == 1)                             //   code = 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else                                    // If invalid parameter, turn LED off
          {
            i = 0;
          }
          if (i == 0)
          {
            HIGHLOAD_LED_OFF;
          }
          else
          {
            HIGHLOAD_LED_ON;
          }
          break;
  
        default:                              // Invalid command string
          break;
      }
    }
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestLEDs()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestAvailabilityLEDs()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Protection Availability LEDs
//
//  MECHANICS:          This subroutine handles testing the Protection Availability LEDs
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            None
//
//  ALTERS:             TP.RxNdxOut
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestAvailabilityLEDs(void)
{
  uint8_t i, j;

  while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a letter is found, a line
       && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   feed is found, or there are no chars left
  {
    i = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if ( (i >= 'A') && (i <= 'Z') )           // If valid, create command string from this char and
    {                                         //   the next char
      j = TP_ParseChars();                    // Read next parameter - must be decimal number (return
      if (j == 1)                             //   code = 1)
      {
        j = (uint8_t)(TP_GetDecNum());
      }
      else                                    // If invalid parameter, turn LED off
      {
        j = 0;
      }
      switch (i)
      {
        case ('L'):                           // Test Long Delay Protection Available LED
          if (j == 1)
          {
            L_LED_ON;
          }
          else
          {
            L_LED_OFF;
          }
          break;
      
        case ('S'):                           // Test Short Delay Protection Available LED
          if (j == 1)
          {
            S_LED_ON;
          }
          else
          {
            S_LED_OFF;
          }
          break;
      
        case ('I'):                           // Test Instantaneous Protection Available LED
          if (j == 1)
          {
            I_LED_ON;
          }
          else
          {
            I_LED_OFF;
          }
          break;
      
        case ('G'):                           // Test Ground Protection Available LED
          if (j == 1)
          {
            G_LED_ON;
          }
          else
          {
            G_LED_OFF;
          }
          break;
  
        default:                              // Invalid command string - turn off all LEDs
          L_LED_OFF;
          S_LED_OFF;
          I_LED_OFF;
          G_LED_OFF;
          break;
      }
    }
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestAvailabilityLEDs()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestIndicators()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Trip Indicator LEDs
//
//  MECHANICS:          This subroutine handles testing the Trip Inidicator LEDs
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            None
//
//  ALTERS:             TP.RxNdxOut
// 
//  CALLS:              WriteCauseOfTripLEDS()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestIndicators(void)
{
  uint8_t i;

  while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a letter is found, a line
       && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   feed is found, or there are no chars left
  {
    i = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if ( (i >= 'A') && (i <= 'Z') )           // If valid, create command string from this char and
    {                                         //   the next char
      COT_AUX.COT_Code = i;
      WriteCauseOfTripLEDS();
    }
    break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestIndicators()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_ExecuteAction()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Execute Action
//
//  MECHANICS:          This subroutine handles execute action commands.  These are commands that cause a
//                      the unit to perform a specific function.  The commands are grouped into five
//                      categories:
//                          P - Power commands - deal with turning on or off on-board circuits or circuits
//                              within the microprocessor itself to test the power consumption
//                          M - Metering commands - deal with enabling and disabling different metering
//                              functions, and performing certain metering functions
//                          R - Restore commands - deal with restoring the unit to some pre-defined state
//                          W - Waveform capture commands - deal with initiating waveform captures
//                          B - Modbus command - used to initiate a Modbus transmission
//                          G - GOOSE Message command - used to initiate a GOOSE high-speed proc-proc
//                                transmission
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            UserWF_CaptureCode, FreezeMinMax, UseADCvals, MB.Status
//
//  ALTERS:             TP.RxNdxOut
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), AFE_SPI_Xfer(), AFE_Init(), ResetMinMaxValues()
// 
//------------------------------------------------------------------------------------------------------------

void TP_ExecuteAction(void)
{
  uint8_t i, k;
  uint16_t j;

  while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a letter is found, a line
       && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   feed is found, or there are no chars left
  {
    i = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if ( (i >= 'A') && (i <= 'Z') )           // If valid, create command string from this char and
    {                                         //   the next char
      switch (i)
      {
        case ('P'):                           // Power Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAP1 - Place AFE in standby mode (lowest pwr consumption)
              AFE_RESETN_ACTIVE;                  // Reset the AFE
              i = 5;                                // Delay at least 100nsec.  Measured on 150828: 175nsec
              while (i > 0)                         //   Note, this is running with the 120MHz clock.
              {
                --i;
              }
              AFE_RESETN_INACTIVE;                  // Must delay at least 120usec after reset is inactive
              j = 5800;                             //   before writing to the AFE
              while (j > 0)
              {
                --j;
              }
              __disable_irq();                    // Disable interrupts when turning off the AFE DRDY int
              EXTI->IMR &= 0xFFFFFEFF;            // Turn off DRDY interrupt
              __enable_irq();
              i = AFE_SPI_Xfer(0x08FF);           // Disable all of the sigma-delta channels
              i = AFE_SPI_Xfer(0x113C);           // Disable the VCM and references
              break;

            case 2:                             // EAP2 - Place microprocessor in Sleep Mode
              __disable_irq();                    // Disable interrupts so no events are generated to wake
                                                  //   up the micro
              __WFE();                            // Place micro in sleep mode, wake from event.  No events
              __WFE();                            //   are enabled, so this is permanent
              __enable_irq();
              break;

            case 3:                             // EAP3 - Place microprocessor in Stop Mode
              __disable_irq();                    // Disable interrupts so no events are generated to wake
                                                  //   up the micro
              PWR->CR &= (uint32_t)(~(PWR_CR_PDDS));
              PWR->CR |= (uint32_t)(PWR_CR_LPDS);
              SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
              __WFE();                            // Place micro in stop mode, wake from event.  No events
              __WFE();                            //   are enabled, so this is permanent
              __enable_irq();
              break;

            case 4:                             // EAP4 - Place microprocessor in Standby Mode
              __disable_irq();                    // Disable interrupts so no events are generated to wake
                                                  //   up the micro
              PWR->CR |= (uint32_t)(PWR_CR_PDDS);
              SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
              __WFI();                            // Place micro in standby mode, wake from interrupt.  No
                                                  //   interrupts are enabled, so this is permanent
              __enable_irq();
              break;

            case 5:                             // EAP5 - Turn off ART accelerator
              FLASH->ACR &= ~(FLASH_ACR_ICEN + FLASH_ACR_DCEN + FLASH_ACR_PRFTEN);
              break;

            case 6:                             // EAP6 - Turn on ART accelerator
              FLASH->ACR |= (FLASH_ACR_ICEN + FLASH_ACR_DCEN + FLASH_ACR_PRFTEN);
              break;

            case 9:                             // EAP9 - Turn on Modbus chip power
              MODBUS_PWR_ENABLE;
              CAM2_UART_MODE;                       // Set the Tx and Rx pins are for UART mode
              break;

            case 10:                            // EAP10 - Turn off Modbus chip power
              CAM2_OUTPUT_MODE;                     // Set the Rx and Tx pins to low outputs
              MODBUS_PWR_DISABLE;
              break;

            case 11:                            // EAP11 - Turn off the Display power (turns off display
              DISPLAY_DISABLE;                  //   processor and backlight)
              break;                            // *** DAH  MAY NEED TO ADD FLAG TO DISABLE THE POWER FROM BEING TURNED ON AND OFF
                                                //   AUTOMATICALLY IN THE MAIN LOOP WHEN POWER IS CHECKED - WILL ADD LATER WHEN THAT CODE
                                                //   IS WRITTEN

            case 12:                            // EAP12 - Turn on the Display power (turns on display
              DISPLAY_ENABLE;                   //   processor and backlight)
              break;

            default:                            // If unknown command, do nothing
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('M'):                           // Metering Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAM1 - Turn off integrator
              TP_AFEIntOff = TRUE;
              break;

            case 2:                             // EAM2 - Turn on integrator
              TP_AFEIntOff = FALSE;
              break;

            case 3:                             // EAM3 - Reset the AFE
              AFE_RESETN_ACTIVE;                  // First reset the chip
              j = 5;                              // Delay at least 100nsec.  Measured on 150820: 1.33usec
              while (j > 0)                       //   Note, this is running with the 16MHz clock.
              {                                   // Assume delay is at least 133nsec at 120MHz
                --j;
              }
              AFE_RESETN_INACTIVE;

              // Contacted ADI on 150825 after seeing errors in the setup when not delaying between the
              //   reset going inactive and calling the routine to initialize the chip.  According to ADI,
              //   must wait at least 120usec after reset is brought inactive before writing to the chip.
              j = 725 * 10;                       // Measured time on 150825: 137usec
              while (j > 0)                       //   Note, this is running with the 16MHz clock (j=725)
              {                                   //   At 120MHz, multiply by 10.
                --j;
              }

              AFE_Init();                         // Next, set up the registers in the chip     

              AFE_START_LOW;
              j = 5 * 4;                          // Delay at least 250nsec.  Measured on 150820: 1.33usec
              while (j > 0)                       //   Note, this is running with the 16MHz clock (j=5).
              {                                   // Assume delay is at least 266nsec at 120MHz if
                --j;                              //   multiplied by 2.  Multiply by 4 to play it safe.
              }
              AFE_START_HIGH;
              break;

            case 4:                             // EAM4 - Disable min/max acquisition
              FreezeMinMax = TRUE;
              break;

            case 5:                             // EAM5 - Enable min/max acquisition
              FreezeMinMax = FALSE;
              break;

            case 6:                             // EAM6 - Reset min/max values
              ResetMinMaxValues();
              break;

            case 7:                             // EAM7 - Set flag to force ADC samples to be used
              UseADCvals = TRUE;
              break;

            case 8:                             // EAM8 - Reset flag to force ADC samples to be used
              UseADCvals = FALSE;
              break;

            case 9:                             // EAM9 - Use ADC low gain circuit
              LOW_GAIN_INPUTS;
              break;

            case 10:                            // EAM10 - Use ADC high gain circuit
              HIGH_GAIN_INPUTS;
              break;

            default:                            // If unknown command, do nothing
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('R'):                           // Restore Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAR1 - Restore AFE current gain and offset cal constants
              for (i=0; i<5; ++i)               //        to default state
              {
                AFEcal.gain[i] = AFE_CAL_DEFAULT_IGAIN;
                AFEcal.offset[i] = AFE_CAL_DEFAULT_IOFFSET;
              }
              AFEcal.gain[4] = AFE_CAL_CT_IGAIN;          // For CT Igsrc
              AFEcal.gain[8] = AFE_CAL_DEFAULT_IGAIN;     // For Rogowski Igsrc
              AFEcal.offset[8] = AFE_CAL_DEFAULT_IOFFSET;
              AFEcal.gain[9] = AFE_CAL_CT_IGAIN;          // For CT In
              AFEcal.offset[9] = AFE_CAL_DEFAULT_IOFFSET;
                                                  // Compute new checksum and complement
              AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
              AFEcal.cmp = ~AFEcal.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 2:                             // EAR2 - Restore AFE voltage gain and offset cal constants
              for (i=5; i<8; ++i)               //        to default state
              {
                AFEcal.gain[i] = AFE_CAL_DEFAULT_VGAIN;
                AFEcal.offset[i] = AFE_CAL_DEFAULT_VOFFSET;
              }
                                                  // Compute new checksum and complement
              AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
              AFEcal.cmp = ~AFEcal.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 3:                             // EAR3 - Restore AFE phase cal constants to default state
              for (i=0; i<12; ++i)
              {
                AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
              }
                                                  // Compute new checksum and complement
              AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
              AFEcal.cmp = ~AFEcal.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 4:                             // EAR4 - Restore ADC low-gain current gain and offset cal
              for (i=0; i<4; ++i)               //          constants
              {
                ADCcalLow.gain[i] = ADC_CAL_DEFAULT_IGAIN_LOW;
                ADCcalLow.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
              }
              ADCcalLow.gain[4] = ADC_CAL_CT_IGAIN_LOW;                 // For CT In
              ADCcalLow.offset[4] = ADC_CAL_CT_IOFFSET;
                                                  // Compute new checksum and complement
              ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
              ADCcalLow.cmp = ~ADCcalLow.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 5:                             // EAR5 - Restore ADC high-gain current gain and offset cal
              for (i=0; i<4; ++i)               //          constants
              {
                ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_IGAIN_HIGH;
                ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
              }
              ADCcalHigh.gain[4] = ADC_CAL_CT_IGAIN_HIGH;               // For CT In
              ADCcalHigh.offset[4] = ADC_CAL_CT_IOFFSET;
                                                  // Compute new checksum and complement
              ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
              ADCcalHigh.cmp = ~ADCcalHigh.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 6:                             // EAR6 - Restore ADC voltage gain and offset cal constants
              for (i=5; i<8; ++i)
              {
                ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_VGAIN;
                ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
                ADCcalLow.gain[i] = ADC_CAL_DEFAULT_VGAIN;
                ADCcalLow.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
              }
                                                  // Compute new checksum and complement
              ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
              ADCcalHigh.cmp = ~ADCcalHigh.chk;
              ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
              ADCcalLow.cmp = ~ADCcalLow.chk;
              TP.State = TP_WR_CAL;                 // Set next state to write cal constants to FRAM and
              break;                                //   Flash

            case 7:                             // EAR7 - Restore Energy Registers
              ResetEnergy();
              // *** DAH ADD CODE TO STORE IN FRAM MAY NOT NEED SINCE FRAM UPDATED EVERY 200MSEC OR SO ANYWAY
              break;

            default:                            // If unknown command, do nothing
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('W'):                           // Waveform Capture Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAW1 - user-initiated waveform capture
              if (CaptureUserWaveform(USR_TYPE_ALL))
              {
                 sprintf(&TP.TxValBuf[0], "\tCapture done");
                 TP.NumChars = 13;
              }
              else
              {
                 sprintf(&TP.TxValBuf[0], "\tCapture failed");
                 TP.NumChars = 15;
              }
              TP.Status &= (~TP_TX_STRING);
              TP.Status |= TP_TX_VALUE;
              TP.TxValNdx = 0;
              UART5->CR1 |= USART_CR1_TXEIE;            // Enable tx interrupts - this begins the transmissions
              TP.State = TP_CURSOR;
              break;

            case 2:                             // EAW2 - trip-initiated waveform capture *** DAH TEMPORARY TEST COMMAND
              if (CaptureUserWaveform(USR_TYPE_IA))           // *** DAH  WHEN TRIP WAVEFORM INITIATED, REMOVE ANY ALARM WAVEFORM REQUEST
              {
                 sprintf(&TP.TxValBuf[0], "\tCapture done");
                 TP.NumChars = 13;
              }
              else
              {
                 sprintf(&TP.TxValBuf[0], "\tCapture failed");
                 TP.NumChars = 15;
              }
              TP.Status &= (~TP_TX_STRING);
              TP.Status |= TP_TX_VALUE;
              TP.TxValNdx = 0;
              UART5->CR1 |= USART_CR1_TXEIE;            // Enable tx interrupts - this begins the transmissions
              TP.State = TP_CURSOR;
//              SD_ProtOn = TRUE;                   // Turn on short delay protection.  This will cause a trip
              break;                              //   and will cause the waveforms to be captured in RAM

/*            case 3:                             // EAW3 - strip-chart waveform capture     - *** DAH  MAYBE CONVERT THIS TO EXTENDED CAPTURE REQUEST
              break;    */

            case 4:                             // EAW4 - alarm waveform capture  *** DAH  THIS CAN BE DELETED EVENTUALLY
              // If a trip or alarm capture is not in progress, set flag to initiate an alarm waveform
              //   capture, insert the event into the queue, save the EID.  Don't initiate an alarm capture
              //   if trip is in progress because the trip is a higher priority
              if ( (!Alarm_WF_Capture.InProg) && (!Trip_WF_Capture.Req)
                && (!Trip_WF_Capture.InProg) )
              {
                Alarm_WF_Capture.Req = TRUE;
                Alarm_WF_Capture.EID = InsertNewEvent(ALARM_MECHANICAL1);
                // Wait for the capture to begin before we send the request to start storing, so that we are
                //   certain the starting index and time stamp have been computed.
                // It is ok to use a while() loop here.  This is relying on the sampling interrupt.  If it
                //   is broken, we should reset anyway.
                while (!Alarm_WF_Capture.InProg)
                {
                }
              }
              else                                // If an alarm capture is already in progress, just insert
              {                                   //   the event into the queue    *** DAH  WILL NEED CODE TO GET THE ALARM EID
                Alarm_WF_Capture.EID = InsertNewEvent(ALARM_MECHANICAL1);
              }
              break;

            default:
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('C'):                           // CAM Transmit Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAC1 - Initiate CAM1 test transmission
              if (!(CAM1.Status & CAM_TYPE_SAMPLE))     // If GOOSE CAM, ok to execute
              {
                // Set k to the smaller of the test string size and the Tx buffer size to ensure we don't
                //   go past the buffer when filling it
                k = ((sizeof(CAM_TEST_STRING) > CAMPORT_TXBUFSIZE) ? CAMPORT_TXBUFSIZE : sizeof(CAM_TEST_STRING));
                for (i=0; i<k; ++i)                         // Fill the Tx buffer
                {
                  CAM1.TxBuf[i] = CAM_TEST_STRING[i];
                }
                CAM1.TxNumChars = k;
                CAM1.Status &= (~CAM_ERROR);                // Clear the error flag
                CAM1.Status |= CAM_XMIT;
              }
              break;

            case 2:                             // EAC2 - Initiate CAM2 test transmission
              if (!(CAM2.Status & CAM_TYPE_SAMPLE))     // If GOOSE CAM, ok to execute
              {
                // Set k to the smaller of the test string size and the Tx buffer size to ensure we don't
                //   go past the buffer when filling it
                k = ((sizeof(CAM_TEST_STRING) > CAMPORT_TXBUFSIZE) ? CAMPORT_TXBUFSIZE : sizeof(CAM_TEST_STRING));
                for (i=0; i<k; ++i)                         // Fill the Tx buffer
                {
                  CAM2.TxBuf[i] = CAM_TEST_STRING[i];
                }
                CAM2.TxNumChars = k;
                CAM2.Status &= (~CAM_ERROR);                // Clear the error flag
                CAM2.Status |= CAM_XMIT;
              }
              break;

            case 3:                             // EAC3 - Enable CAM1 driver - *** DAH CAN DELETE THIS EVENTUALLY
              CAM1_DRVR_ENABLED;
              break;

            case 4:                             // EAC4 - Disable CAM1 driver - *** DAH CAN DELETE THIS EVENTUALLY
              CAM1_DRVR_DISABLED;
              break;

            case 5:                             // EAC5 - Enable CAM2 driver - *** DAH CAN DELETE THIS EVENTUALLY
              CAM2_DRVR_ENABLED;
              break;

            case 6:                             // EAC6 - Disable CAM2 driver - *** DAH CAN DELETE THIS EVENTUALLY
              CAM2_DRVR_DISABLED;
              break;

            default:                            // If unknown command, do nothing
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('D'):                           // Display Communications Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          switch (i)
          {
            case 1:                             // EAD1 - Not used
              break;                            
            default:                            // If unknown command, do nothing
              break;
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('H'):                           // Health Command
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            i = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            i = 255;                            // If invalid parameter, set code to invalid
          }
          if (i == 0)                           // EAH0 - Turn off coil measurement
          {
            TestInj.Flags &= (~COIL_MEAS_ON);     // Clear flag so samples are frozen
            COIL_TEMP_4800_OFF;                   // Turn off the 4800Hz excitation signal
            COIL_TEMP_OFF;                        // Turn off the multiplexer
            // Disable the Timer8 clock which stops the 4800Hz square wave output
            TIM8->CR1 &= 0xFFFE;
            TSTINJ_PHA_OFF;                       // Turn off the test channels
            TSTINJ_PHB_OFF;
            TSTINJ_PHC_OFF;
            TSTINJ_PHN_OFF;
          }
          else if (i < 5)                       // EAH1 thru 4 - Initiate coil measurement on PHx
          {
            // Turn off test injection only if it was on.  Don't set the flag to turn it off if test
            //   injection is already off, because it will then turn off test injection the next time it is
            //   turned on.
            if (TestInj.Flags & TEST_INJ_ON)
            {
              TestInj.Flags |= TEST_INJ_INIT_OFF;
              while (TestInj.Flags & TEST_INJ_INIT_OFF)   // Wait for test injection to be turned off
              {
              }
            }
            // Enable the Timer8 clock which starts the 4800Hz square wave output
            TIM8->CR1 |= 0x0001;
            switch (i)
            {
              case 1:                             // EAH1 - Initiate coil measurement on PHA
                TSTINJ_PHA_ON;                        // Turn on the test channel
                COIL_TEMP_A_ON;                       // Set up the multiplexer
                break;
              case 2:                             // EAH2 - Initiate coil measurement on PHB
                TSTINJ_PHB_ON;                        // Turn on the test channel
                COIL_TEMP_B_ON;                       // Set up the multiplexer
                break;
              case 3:                             // EAH3 - Initiate coil measurement on PHC
                TSTINJ_PHC_ON;                        // Turn on the test channel
                COIL_TEMP_C_ON;                       // Set up the multiplexer
                break;
              default:                            // EAH4 - Initiate coil measurement on PHN
                TSTINJ_PHN_ON;                        // Turn on the test channel
                COIL_TEMP_N_ON;                       // Set up the multiplexer
                break;
            }
            // Turn on the 4800Hz coil temperature excitation signal
            COIL_TEMP_4800_ON;
            TestInj.Flags |= COIL_MEAS_ON;        // Set flag to capture samples
          }
          else if (i == 9)                       // EAH9 - Coil detection on all phases
          {  
            CoilDetect.State = 0;               // These are for the state machine in Coil_Detection()
            CoilDetect.Enable = 1;           
          } 
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

        case ('G'):                           // GOOSE Communications Commands     *** DAH ADDED FOR TEST USING TEST PORT  201219
          // i is the number of GOOSE transmissions (up to 255) to transmit consecutively
          i = TP_ParseChars();                  // Read next parameter - must be decimal number (return code
          if (i == 1)                           //   equals 1)
          {
            j = (uint16_t)(TP_GetDecNum());
          }
          else
          {
            j = 0;                              // If invalid parameter, set temp to 0
          }
          // Code < 3: single request - adjust j as follows:
          // j = 0: Goose Status Control Block & Control Switch of CB pos - Req index = 1 (skip ACK = 0)
          // j = 1: Meter Values - Req index = 2
          if (j < 3)
          {
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
            TESTPIN_A3_HIGH;
#endif
            DPComm61850.Req[j + 1] = TRUE;
          }
          else if(j >= 100)
          {
            if ((j/100) == DP61850_TYPE_GOCB_STATUS_CTRL)
            {
                j = j%100;
                if((j/10) <= 5)
                {
                  if((j/10) == 0)
                  {
                    if ((j%10) <= BAD_STATE)
                    {
                      PXR35_CB_Publish_Data.CB_Status_Ctrl.CB_Pos_Status = (j%10);
                    }
                  }
                  else if((j/10) == 1)
                  {
                    if ((j%10) <= 1)
                    {
                      PXR35_CB_Publish_Data.CB_Status_Ctrl.Trip_Failure = (j%10);
                    }
                  }
                  else if((j/10) == 2)
                  {
                    if ((j%10) <= 1)
                    {
                      PXR35_CB_Publish_Data.CB_Status_Ctrl.ZSI_Status = (j%10);
                    }
                  }
                  else if((j/10) == 3)
                  {
                    if ((j%10) <= 1)
                    {
                      PXR35_CB_Publish_Data.CB_Status_Ctrl.Source_Status = (j%10);
                    }
                  }
                  else if((j/10) == 4)
                  {
                    if ((j%10) <= 1)
                    {
                      PXR35_CB_Publish_Data.CB_Status_Ctrl.Capture_State = (j%10);
                    }
                  }
                  else if((j/10) == 5)
                  {
                    if ((j%10) <= 1)
                    {
                      if ((j%10) == 0)
                      {
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB1_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB2_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB3_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB4_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB5_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB6_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB7_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB8_Close_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB1_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB2_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB3_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB4_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB5_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB6_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB7_Open_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB8_Open_Op = 1;
                      }
                      else
                      {
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB1_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB2_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB3_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB4_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB5_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB6_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB7_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB8_Open_Op = 0;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB1_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB2_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB3_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB4_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB5_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB6_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB7_Close_Op = 1;
                        PXR35_CB_Publish_Data.CB_Status_Ctrl.CTRL_CB8_Close_Op = 1;
                      }
                    }
                  }
                  __disable_irq();
                  DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
                  TESTPIN_A3_HIGH;
#endif
                  __enable_irq();
                }
            }
          }
          else if (j == 77)   // *** DAH TEST  210420
          {
            gtest = TRUE;
            gtestcnt = 0;
          }
          else if (j == 78)   // *** DAH TEST  210420
          {
            gtest = FALSE;
          }
          else if (j == 80)
          {
            dfred_on = TRUE;                    // *** DAH TEST
          }
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;

/*        case ('Z'):                           // *** DAH TEST  210402 FOR DMA TESTING IN THE DISPLAY PROCESSOR
          DPComm.RTD_XmitTimer[0] = 57;
          break;   */


        default:                              // Invalid command string
          TP.RxNdxOut = TP.RxNdxIn;             // Command is completed, set NdxOut to NdxIn to force exit
          break;
      }
    }
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_ExecuteAction()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRTValues()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Real Time Values
//
//  MECHANICS:          This subroutine reads the command string to determine which real-time data values
//                      should be displayed,  and sets the two state variables accordingly
//                      
//  CAVEATS:            None
//
//  INPUTS:             None
// 
//  OUTPUTS:            TP.State, TP.SubState
//
//  ALTERS:             None
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), TP_GetHexNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRTValues(void)
{
  uint8_t i;

  i = TP_ParseChars();                      // Read next parameter - must be either decimal or hex number
  if (i == 1)
  {
    i = (uint8_t)(TP_GetDecNum());
  }
  else if (i == 2)
  {
    i = (uint8_t)(TP_GetHexNum());
  }
  else                                      // If invalid number, set i to 0xFF
  {
    i = 0xFF;
  }
  switch (i)                                // Set next state according to the group of values requested
  {
    case 0:                                 // Group 0 values - currents
      TP.State = TP_DR0;
      TP.SubState = TP_DRGR0_0;
      break;

    case 1:                                 // Group 1 values - min/max currents
      TP.State = TP_DR1;
      TP.SubState = TP_DRGR1_0;
      break;

    case 2:                                 // Group 2 values - voltages
      TP.State = TP_DR2;
      TP.SubState = TP_DRGR2_0;
      break;

    case 3:                                 // Group 3 values - min/max voltages
      TP.State = TP_DR3;
      TP.SubState = TP_DRGR3_0;
      break;

    case 4:                                 // Group 4 values - powers
      TP.State = TP_DR4;
      TP.SubState = TP_DRGR4_0;
      break;

    case 5:                                 // Group 5 values - min/max powers
      TP.State = TP_DR5;
      TP.SubState = TP_DRGR5_0;
      break;

    default:                                // Invalid command string
      break;
  }                                             // Leave next state at idle to ignore the command
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRTValues()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayFWversions()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Firmware Versions
//
//  MECHANICS:          This subroutine displays the firmware version, revision, and build number for the
//                      protection processor, display processor, and override micro
//                      
//  CAVEATS:            None
//
//  INPUTS:             Ovr_FW_Version
// 
//  OUTPUTS:            TP.State, TP.SubState
//
//  ALTERS:             None
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), TP_GetHexNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayFWversions(void)
{
  uint16_t temp;

  switch (TP.SubState)
  {
    case TP_DQ0:                                // Initialize string counter
      TP.Temp = 0;
      TP.SubState++;

    case TP_DQ1:                                // Output prot proc fw version string
    case TP_DQ5:                                // Output prot proc fw revision string
    case TP_DQ9:                                // Output prot proc fw build string
    case TP_DQ13:                               // Output disp proc fw version string
    case TP_DQ17:                               // Output disp proc fw revision string
    case TP_DQ21:                               // Output disp proc fw build string
    case TP_DQ25:                               // Output ovr micro fw build string
      TP.StrPtr = FW_STR_PTR[TP.Temp];
      TP.Status &= (~TP_TX_VALUE);              // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;                // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;            // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DQ2:                                // Wait for transmission to complete
    case TP_DQ4:                                // Wait for transmission to complete
    case TP_DQ6:                                // Wait for transmission to complete
    case TP_DQ8:                                // Wait for transmission to complete
    case TP_DQ10:                               // Wait for transmission to complete
    case TP_DQ12:                               // Wait for transmission to complete
    case TP_DQ14:                               // Wait for transmission to complete
    case TP_DQ16:                               // Wait for transmission to complete
    case TP_DQ18:                               // Wait for transmission to complete
    case TP_DQ20:                               // Wait for transmission to complete
    case TP_DQ22:                               // Wait for transmission to complete
    case TP_DQ24:                               // Wait for transmission to complete
    case TP_DQ26:                               // Wait for transmission to complete
      // Jump to next state when done transmitting.  Check both flags since state covers both string and
      //   value transmissions
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        TP.SubState++;
      }
      break;

    case TP_DQ3:                                // Output prot proc firmware version
    case TP_DQ7:                                // Output prot proc firmware revision
    case TP_DQ11:                               // Output proc firmware build number
      sprintf(&TP.TxValBuf[0], "%5u", PROT_PROC_FW_VERREVBUILD[TP.Temp++]);
      TP.TxValBuf[5] = '\n';
      TP.TxValBuf[6] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 7;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DQ15:                               // Output disp proc firmware version
    case TP_DQ19:                               // Output disp proc firmware revision
    case TP_DQ23:                               // Output disp firmware build number
    case TP_DQ27:                               // Output ovr micro firmware build number
      temp = *(uint16_t *)DISP_OVR_PROC_FWREV[TP.Temp - 3];
      if (TP.SubState != TP_DQ23)               // Some values are u8's - need to make sure high byte is 0
      {
        temp &= 0x00FF;
      }
      sprintf(&TP.TxValBuf[0], "%5u", ((uint32_t)temp));
      TP.Temp++;
      TP.TxValBuf[5] = '\n';
      TP.TxValBuf[6] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 7;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DQ28:                                // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting, set main state to output
      {                                             //   cursor
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayFWversions()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT0Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 0 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 0 real-time values:
//                          IaHC       IbHC       IcHC       InHC       IgsrcHC       IgresHC
//                          IaOC       IbOC       IcOC       InOC       IgsrcOC       IgresOC
//                          Ia200ms    Ib200ms    Ic200ms    In200ms    Igsrc200ms    Igres200ms
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, CurHalfCycSOS_SumF.Ix, CurOneCyc.Ix, Cur200msFltr.Ix
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sqrtf(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT0Values(void)
{
  float temp;

  switch (TP.SubState)
  {
    case TP_DRGR0_0:                          // Group 0 values - half-cycle currents
      temp = CurHalfCycSOS_SumF.Ia;                 // Since current is updated in the 200usec interrupt,
      temp = sqrtf(temp/40);                        //   store it in a temporary before processing
      sprintf(&TP.TxValBuf[0], "% .5E", temp);      // *** DAH MAKE SURE MOVE TO TEMPORARY IS ATOMIC AND PUT IN COMMENTS IF IT IS
      TP.TxValBuf[12] = ' ';
      temp = CurHalfCycSOS_SumF.Ib;
      temp = sqrtf(temp/40);
      sprintf(&TP.TxValBuf[13], "% .5E", temp);
      TP.TxValBuf[25] = ' ';
      temp = CurHalfCycSOS_SumF.Ic;
      temp = sqrtf(temp/40);
      sprintf(&TP.TxValBuf[26], "% .5E", temp);
      TP.TxValBuf[38] = ' ';
      temp = CurHalfCycSOS_SumF.In;
      temp = sqrtf(temp/40);
      sprintf(&TP.TxValBuf[39], "% .5E", temp);
      TP.TxValBuf[51] = ' ';
      temp = CurHalfCycSOS_SumF.Igsrc;
      temp = sqrtf(temp/40);
      sprintf(&TP.TxValBuf[52], "% .5E", temp);
      TP.TxValBuf[64] = ' ';
      temp = CurHalfCycSOS_SumF.Igres;
      temp = sqrtf(temp/40);
      sprintf(&TP.TxValBuf[65], "% .5E", temp);
      TP.TxValBuf[77] = '\n';
      TP.TxValBuf[78] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 79;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DRGR0_1;
      break;

    case TP_DRGR0_1:                          // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Fall into next state when done transmitting
      {
        TP.SubState = TP_DRGR0_2;
      }
      else
      {
        break;
      }

    case TP_DRGR0_2:                          // Group 0 values - one-cycle currents
      sprintf(&TP.TxValBuf[0], "% .5E", CurOneCyc.Ia);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", CurOneCyc.Ib);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", CurOneCyc.Ic);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", CurOneCyc.In);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", CurOneCyc.Igsrc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", CurOneCyc.Igres);
      TP.TxValBuf[77] = '\n';
      TP.TxValBuf[78] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 79;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DRGR0_3;
      break;

    case TP_DRGR0_3:                          // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Fall into next state when done transmitting
      {
        TP.SubState = TP_DRGR0_4;
      }
      else
      {
        break;
      }

    case TP_DRGR0_4:                          // Group 0 values - 200msec currents
      sprintf(&TP.TxValBuf[0], "% .5E", Cur200msFltr.Ia);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", Cur200msFltr.Ib);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", Cur200msFltr.Ic);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", Cur200msFltr.In);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", Cur200msFltr.Igsrc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", Cur200msFltr.Igres);
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 77;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.State = TP_CURSOR;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT0Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT1Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 1 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 1 real-time values.
//                        IaHCmin      IbHCmin      IcHCmin      InHCmin      IgsrcHCmin      IgresHCmin
//                        IaHCmax      IbHCmax      IcHCmax      InHCmax      IgsrcHCmax      IgresHCmax
//                        IaHCvar      IbHCvar      IcHCvar      InHCvar      IgsrcHCvar      IgresHCvar
//
//                        IaOCmin      IbOCmin      IcOCmin      InOCmin      IgsrcOCmin      IgresOCmin
//                        IaOCmax      IbOCmax      IcOCmax      InOCmax      IgsrcOCmax      IgresOCmax
//                        IaOCvar      IbOCvar      IcOCvar      InOCvar      IgsrcOCvar      IgresOCvar
//
//                        Ia200msmin   Ib200msmin   Ic200msmin   In200msmin   Igsrc200msmin   Igres200msmin
//                        Ia200msmax   Ib200msmax   Ic200msmax   In200msmax   Igsrc200msmax   Igres200msmax
//                        Ia200msvar   Ib200msvar   Ic200msvar   In200msvar   Igsrc200msvar   Igres200msvar
//
//                      The variance is computed as follows:
//                          (max - min)/(max + min) * 200%
//                      It is intended to provide a measure of how much deviation there is in the readings.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, CurHCSOS_Fmin.Ix, CurHCSOS_Fmax.Ix, CurOCSOS_Fmin.Ix, CurOCSOS_Fmax.Ix,
//                      Cur200msFltr_min.Ix, Cur200msFltr_max.Ix
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sqrtf(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT1Values(void)
{
  float temp, temp1;
  float *ptr, *ptr1;
  uint8_t ind, i;

  switch (TP.SubState)
  {
    case TP_DRGR1_0:                          // Group 1 values - half-cycle min currents
    case TP_DRGR1_2:                          // Group 1 values - half-cycle max currents
    case TP_DRGR1_6:                          // Group 1 values - one-cycle min currents (SOS)
    case TP_DRGR1_8:                          // Group 1 values - one-cycle max currents (SOS)
    case TP_DRGR1_12:                         // Group 1 values - one-cycle min currents
    case TP_DRGR1_14:                         // Group 1 values - one-cycle max currents
                                                    // Initialize the pointer
      if (TP.SubState == TP_DRGR1_0)
      {
        ptr = &CurHCSOS_Fmin.Ia;
      }
      else if (TP.SubState == TP_DRGR1_2)
      {
        ptr = &CurHCSOS_Fmax.Ia;
      }
      else if (TP.SubState == TP_DRGR1_6)
      {
        ptr = &CurOCSOS_Fmin.Ia;
      }
      else if (TP.SubState == TP_DRGR1_8)
      {
        ptr = &CurOCSOS_Fmax.Ia;
      }
      else if (TP.SubState == TP_DRGR1_12)
      {
        ptr = &CurOneCyc_min.Ia;
      }
      else
      {
        ptr = &CurOneCyc_max.Ia;
      }
      ind = 0;
      for (i=0; i<6; ++i)
      {
        temp = *ptr++;
        if (TP.SubState <= TP_DRGR1_2)
        {
          temp = sqrtf(temp/40);
        }
        else if (TP.SubState <= TP_DRGR1_8)
        {
          temp = sqrtf(temp/80);
        }
        sprintf(&TP.TxValBuf[ind], "% .5E", temp);
        ind += 12;
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[78] = '\n';
      TP.TxValBuf[79] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 80;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DRGR1_1:                          // Wait for transmission to complete
    case TP_DRGR1_3:                          // Wait for transmission to complete
    case TP_DRGR1_5:                          // Wait for transmission to complete
    case TP_DRGR1_7:                          // Wait for transmission to complete
    case TP_DRGR1_9:                          // Wait for transmission to complete
    case TP_DRGR1_11:                         // Wait for transmission to complete
    case TP_DRGR1_13:                         // Wait for transmission to complete
    case TP_DRGR1_15:                         // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Advance to next state when done transmitting
      {
        TP.SubState++;
      }
      break;

    case TP_DRGR1_4:                          // Group 1 values - half-cycle min/max variance
    case TP_DRGR1_10:                         // Group 1 values - one-cycle min/max variance (SOS)
    case TP_DRGR1_16:                         // Group 1 values - one-cycle min/max variance
      if (TP.SubState == TP_DRGR1_4)
      {
        ptr = &CurHCSOS_Fmin.Ia;
        ptr1 = &CurHCSOS_Fmax.Ia;
      }
      else if (TP.SubState == TP_DRGR1_10)
      {
        ptr = &CurOCSOS_Fmin.Ia;
        ptr1 = &CurOCSOS_Fmax.Ia;
      }
      else
      {
        ptr = &CurOneCyc_min.Ia;
        ptr1 = &CurOneCyc_max.Ia;
      }
      ind = 0;
      for (i=0; i<6; ++i)
      {
        temp = *ptr++;
        temp1 = *ptr1++;
        if (TP.SubState == TP_DRGR1_4)
        {
          temp = sqrtf(temp/40);
          temp1 = sqrtf(temp1/40);
        }
        else if (TP.SubState == TP_DRGR1_10)
        {
          temp = sqrtf(temp/80);
          temp1 = sqrtf(temp1/80);
        }
        temp = ((temp1 - temp)/(temp1 + temp)) * (float)(200.0);
        sprintf(&TP.TxValBuf[ind], "% .2E", temp);
        ind += 9;
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[78] = '\n';
      TP.TxValBuf[79] = '\r';
      TP.TxValBuf[80] = '\n';
      TP.TxValBuf[81] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 82;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      if (TP.SubState == TP_DRGR1_16)
      {
        TP.State = TP_CURSOR;
      }
      else
      {
        ++TP.SubState;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT1Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT2Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 2 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 2 real-time values.
//                        VanOCAFE      VbnOCAFE      VcnOCAFE      VabOCAFE      VbcOCAFE      VcaOCAFE
//                        Van200msAFE   Vbn200msAFE   Vcn200msAFE   Vab200msAFE   Vbc200msAFE   Vca200msAFE
//                        VanOCADC      VbnOCADC      VcnOCADC      VabOCADC      VbcOCADC      VcaOCADC
//                        Van200msADC   Vbn200msADC   Vcn200msADC   Vab200msADC   Vbc200msADC   Vca200msADC
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, VolAFEOneCyc.Vxx, VolAFE200msFltr.Vxx, VolADCOneCyc.Vxx,
//                      VolADC200ms.Vxx
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sqrtf(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT2Values(void)
{

  switch (TP.SubState)
  {
    case TP_DRGR2_0:                          // Group 2 values - one-cycle AFE voltages
      sprintf(&TP.TxValBuf[0], "% .5E", VolAFEOneCyc.Van);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", VolAFEOneCyc.Vbn);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", VolAFEOneCyc.Vcn);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", VolAFEOneCyc.Vab);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", VolAFEOneCyc.Vbc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", VolAFEOneCyc.Vca);
      TP.TxValBuf[77] = '\n';
      TP.TxValBuf[78] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 79;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DRGR2_1;
      break;


    case TP_DRGR2_1:                          // Wait for transmission to complete
    case TP_DRGR2_3:                          // Wait for transmission to complete
    case TP_DRGR2_5:                          // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Fall into next state when done transmitting
      {
        ++TP.SubState;
      }
      break;

    case TP_DRGR2_2:                          // Group 2 values - 200msec AFE voltages
      sprintf(&TP.TxValBuf[0], "% .5E", VolAFE200msFltr.Van);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", VolAFE200msFltr.Vbn);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", VolAFE200msFltr.Vcn);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", VolAFE200msFltr.Vab);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", VolAFE200msFltr.Vbc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", VolAFE200msFltr.Vca);
      TP.TxValBuf[77] = '\n';
      TP.TxValBuf[78] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 79;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DRGR2_3;
      break;

    case TP_DRGR2_4:                          // Group 2 values - one-cycle ADC voltages
      sprintf(&TP.TxValBuf[0], "% .5E", VolADCOneCyc.Van);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", VolADCOneCyc.Vbn);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", VolADCOneCyc.Vcn);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", VolADCOneCyc.Vab);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", VolADCOneCyc.Vbc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", VolADCOneCyc.Vca);
      TP.TxValBuf[77] = '\n';
      TP.TxValBuf[78] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 79;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DRGR2_5;
      break;

    case TP_DRGR2_6:                          // Group 2 values - 200msec ADC voltages
      sprintf(&TP.TxValBuf[0], "% .5E", VolADC200ms.Van);
      TP.TxValBuf[12] = ' ';
      sprintf(&TP.TxValBuf[13], "% .5E", VolADC200ms.Vbn);
      TP.TxValBuf[25] = ' ';
      sprintf(&TP.TxValBuf[26], "% .5E", VolADC200ms.Vcn);
      TP.TxValBuf[38] = ' ';
      sprintf(&TP.TxValBuf[39], "% .5E", VolADC200ms.Vab);
      TP.TxValBuf[51] = ' ';
      sprintf(&TP.TxValBuf[52], "% .5E", VolADC200ms.Vbc);
      TP.TxValBuf[64] = ' ';
      sprintf(&TP.TxValBuf[65], "% .5E", VolADC200ms.Vca);
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 77;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.State = TP_CURSOR;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT2Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT3Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 3 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 3 real-time values.
//                        AFE: VanOCmin     VbnOCmin     VcnOCmin     VabOCmin     VbcOCmin     VcaOCmin
//                             VanOCmax     VbnOCmax     VcnOCmax     VabOCmax     VbcOCmax     VcaOCmax
//                             VanOCvar     VbnOCvar     VcnOCvar     VabOCvar     VbcOCvar     VcaOCvar
//
//                      The variance is computed as follows:
//                          (max - min)/(max + min) * 200%
//                      It is intended to provide a measure of how much deviation there is in the readings.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, VolAFEOneCyc_max.Vxx, VolAFEOneCyc_min.Vxx
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT3Values(void)
{
  float temp, temp1;
  float *ptr, *ptr1, *ptr2, *ptr3;
  uint8_t ind, i;

  switch (TP.SubState)
  {
    case TP_DRGR3_0:                          // Group 3 values - one-cycle min AFE voltages
    case TP_DRGR3_2:                          // Group 3 values - one-cycle max AFE voltages
      if (TP.SubState == TP_DRGR3_0)                // Initialize the pointers
      {
        ptr = &VolAFEOneCyc_min.Van;
        ptr1 = &VolAFEOneCyc_min.Vab;
      }
      else
      {
        ptr = &VolAFEOneCyc_max.Van;
        ptr1 = &VolAFEOneCyc_max.Vab;
      }
      ind = 0;
      for (i=0; i<3; ++i)
      {
        temp = *ptr++;
        sprintf(&TP.TxValBuf[ind], "% .5E", temp);
        ind += 12;
        TP.TxValBuf[ind++] = ' ';
      }
      for (i=0; i<3; ++i)
      {
        temp = *ptr1++;
        sprintf(&TP.TxValBuf[ind], "% .5E", temp);
        ind += 12;
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[78] = '\n';
      TP.TxValBuf[79] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 80;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DRGR3_1:                          // Wait for transmission to complete
    case TP_DRGR3_3:                          // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Advance to next state when done transmitting
      {
        TP.SubState++;
      }
      break;

    case TP_DRGR3_4:                          // Group 3 values - one-cycle min/max variance
      ptr = &VolAFEOneCyc_min.Van;
      ptr1 = &VolAFEOneCyc_max.Van;
      ptr2 = &VolAFEOneCyc_min.Vab;
      ptr3 = &VolAFEOneCyc_max.Vab;
      ind = 0;
      for (i=0; i<3; ++i)
      {
        temp = *ptr++;
        temp1 = *ptr1++;
        temp = ((temp1 - temp)/(temp1 + temp)) * (float)(200.0);
        sprintf(&TP.TxValBuf[ind], "% .2E", temp);
        ind += 9;
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
      }
      for (i=0; i<3; ++i)
      {
        temp = *ptr2++;
        temp1 = *ptr3++;
        temp = ((temp1 - temp)/(temp1 + temp)) * (float)(200.0);
        sprintf(&TP.TxValBuf[ind], "% .2E", temp);
        ind += 9;
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[78] = '\n';
      TP.TxValBuf[79] = '\r';
      TP.TxValBuf[80] = '\n';
      TP.TxValBuf[81] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 82;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.State = TP_CURSOR;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT3Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT4Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 4 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 4 real-time values.
//                        PaOC             PbOC             PcOC
//                        RPaOC            RPbOC            RPcOC
//                        AppPaOC          AppPbOC          AppPcOC
//
//                        Pa200msec        Pb200msec          PcOC
//                        RPa200msec       RPb200msec         RPcOC
//                        AppPa200msec     AppPb200msec       AppPcOC
//
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, PwrOneCyc.Px, PwrOneCyc.RPx, PwrOneCycApp.AppPx, Pwr200msec.Px,
//                      Pwr200msec.RPx, Pwr200msecApp.AppPx
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT4Values(void)
{
  float temp;
  float *ptr;
  uint8_t ind, i;

  switch (TP.SubState)
  {
    case TP_DRGR4_0:                          // Group 4 values - one-cycle real powers
    case TP_DRGR4_2:                          // Group 4 values - one-cycle reactive powers
    case TP_DRGR4_4:                          // Group 4 values - one-cycle apparent powers
    case TP_DRGR4_8:                          // Group 4 values - 200msec real powers
    case TP_DRGR4_10:                         // Group 4 values - 200msec reactive powers
    case TP_DRGR4_12:                         // Group 4 values - 200msecaparent powers
                                                    // Initialize the pointer
      if (TP.SubState == TP_DRGR4_0)
      {
        ptr = &PwrOneCyc.Pa;
      }
      else if (TP.SubState == TP_DRGR4_2)
      {
        ptr = &PwrOneCyc.RPa;
      }
      else if (TP.SubState == TP_DRGR4_4)
      {
        ptr = &PwrOneCycApp.AppPa;
      }
      else if (TP.SubState == TP_DRGR4_8)
      {
        ptr = &Pwr200msec.Pa;
      }
      else if (TP.SubState == TP_DRGR4_10)
      {
        ptr = &Pwr200msec.RPa;
      }
      else
      {
        ptr = &Pwr200msecApp.AppPa;
      }
      ind = 0;
      for (i=0; i<3; ++i)
      {
        temp = *ptr++;
        sprintf(&TP.TxValBuf[ind], "% .5E", temp);
        ind += 12;
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[39] = '\n';
      TP.TxValBuf[40] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 41;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      if (TP.SubState == TP_DRGR4_12)
      {
        TP.State = TP_CURSOR;
      }
      else
      {
        ++TP.SubState;
      }
      break;

    case TP_DRGR4_1:                          // Wait for transmission to complete
    case TP_DRGR4_3:                          // Wait for transmission to complete
    case TP_DRGR4_5:                          // Wait for transmission to complete
    case TP_DRGR4_7:                          // Wait for transmission to complete
    case TP_DRGR4_9:                          // Wait for transmission to complete
    case TP_DRGR4_11:                         // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Advance to next state when done transmitting
      {
        TP.SubState++;
      }
      break;

    case TP_DRGR4_6:                          // Add spacing line
      TP.TxValBuf[0] = '\n';
      TP.TxValBuf[1] = '\r';
      TP.TxValBuf[2] = '\n';
      TP.TxValBuf[3] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 4;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT4Values()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayRT5Values()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Group 5 Real Time Values
//
//  MECHANICS:          This subroutine displays the group 5 real-time values.
//                        PaOCmin         PbOCmin         PaOCmin
//                        PaOCmax         PbOCmax         PcOCmax
//                        PaOCvar         PbOCvar         PbOCvar
//
//                        RPaOCmin        RPbOCmin        RPaOCmin
//                        RPaOCmax        RPbOCmax        RPcOCmax
//                        RPaOCvar        RPbOCvar        RPbOCvar
//
//                        AppPaOCmin      AppPbOCmin      AppPcOCmin
//                        AppPaOCmax      AppPbOCmax      AppPcOCmax
//                        AppPaOCvar      AppPbOCvar      AppPcOCvar
//
//                        Pa200msmin      Pb200msmin      Pc200msmin
//                        Pa200msmax      Pb200msmax      Pc200msmax
//                        Pa200msvar      Pb200msvar      Pc200msvar
//
//                        RPa200msmin     RPb200msmin     RPc200msmin
//                        RPa200msmax     RPb200msmax     RPc200msmax
//                        RPa200msvar     RPb200msvar     RPc200msvar
//
//                        AppPa200msmin   AppPb200msmin   AppPc200msmin
//                        AppPa200msmax   AppPb200msmax   AppPc200msmax
//                        AppPa200msvar   AppPb200msvar   AppPc200msvar
//
//                      The variance is computed as follows:
//                          (max - min)/(max + min) * 200%
//                      It is intended to provide a measure of how much deviation there is in the readings.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, PwrOneCycMin.Xx, PwrOneCycMax.Xx, PwrOneCycAppMin.AppPx,
//                      PwrOneCycAppMax.AppPx, Pwr200msecMin.Xx, Pwr200msecMax.Xx, Pwr200msecAppMin.AppPx,
//                      Pwr200msecAppMax.AppPx
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayRT5Values(void)
{
  float temp, temp1;
  float *ptr, *ptr1;
  uint8_t ind, i;

  switch (TP.SubState)
  {
    case TP_DRGR5_0:                          // Group 5 values - one-cycle min active powers
    case TP_DRGR5_2:                          // Group 5 values - one-cycle max active powers
    case TP_DRGR5_6:                          // Group 5 values - one-cycle min reactive powers
    case TP_DRGR5_8:                          // Group 5 values - one-cycle max reactive powers
    case TP_DRGR5_12:                         // Group 5 values - one-cycle min apparent powers
    case TP_DRGR5_14:                         // Group 5 values - one-cycle max apparent powers
    case TP_DRGR5_18:                         // Group 5 values - 200msec min active powers
    case TP_DRGR5_20:                         // Group 5 values - 200msec max active powers
    case TP_DRGR5_24:                         // Group 5 values - 200msec min reactive powers
    case TP_DRGR5_26:                         // Group 5 values - 200msec max reactive powers
    case TP_DRGR5_30:                         // Group 5 values - 200msec min apparent powers
    case TP_DRGR5_32:                         // Group 5 values - 200msec max apparent powers
                                                    // Initialize the pointer
      if (TP.SubState == TP_DRGR5_0)
      {
        ptr = &PwrOneCycMin.Pa;
      }
      else if (TP.SubState == TP_DRGR5_2)
      {
        ptr = &PwrOneCycMax.Pa;
      }
      else if (TP.SubState == TP_DRGR5_6)
      {
        ptr = &PwrOneCycMin.RPa;
      }
      else if (TP.SubState == TP_DRGR5_8)
      {
        ptr = &PwrOneCycMax.RPa;
      }
      else if (TP.SubState == TP_DRGR5_12)
      {
        ptr = &PwrOneCycAppMin.AppPa;
      }
      else if (TP.SubState == TP_DRGR5_14)
      {
        ptr = &PwrOneCycAppMax.AppPa;
      }
      else if (TP.SubState == TP_DRGR5_18)
      {
        ptr = &Pwr200msecMin.Pa;
      }
      else if (TP.SubState == TP_DRGR5_20)
      {
        ptr = &Pwr200msecMax.Pa;
      }
      else if (TP.SubState == TP_DRGR5_24)
      {
        ptr = &Pwr200msecMin.RPa;
      }
      else if (TP.SubState == TP_DRGR5_26)
      {
        ptr = &Pwr200msecMax.RPa;
      }
      else if (TP.SubState == TP_DRGR5_30)
      {
        ptr = &Pwr200msecAppMin.AppPa;
      }
      else
      {
        ptr = &Pwr200msecAppMax.AppPa;
      }
      ind = 0;
      for (i=0; i<3; ++i)
      {
        temp = *ptr++;
        sprintf(&TP.TxValBuf[ind], "% .5E", temp);
        ind += 12;
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[39] = '\n';
      TP.TxValBuf[40] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 41;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState++;
      break;

    case TP_DRGR5_1:                          // Wait for transmission to complete
    case TP_DRGR5_3:                          // Wait for transmission to complete
    case TP_DRGR5_5:                          // Wait for transmission to complete
    case TP_DRGR5_7:                          // Wait for transmission to complete
    case TP_DRGR5_9:                          // Wait for transmission to complete
    case TP_DRGR5_11:                         // Wait for transmission to complete
    case TP_DRGR5_13:                         // Wait for transmission to complete
    case TP_DRGR5_15:                         // Wait for transmission to complete
    case TP_DRGR5_17:                         // Wait for transmission to complete
    case TP_DRGR5_19:                         // Wait for transmission to complete
    case TP_DRGR5_21:                         // Wait for transmission to complete
    case TP_DRGR5_23:                         // Wait for transmission to complete
    case TP_DRGR5_25:                         // Wait for transmission to complete
    case TP_DRGR5_27:                         // Wait for transmission to complete
    case TP_DRGR5_29:                         // Wait for transmission to complete
    case TP_DRGR5_31:                         // Wait for transmission to complete
    case TP_DRGR5_33:                         // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Advance to next state when done transmitting
      {
        TP.SubState++;
      }
      break;

    case TP_DRGR5_4:                          // Group 5 values - one-cycle min/max variance
    case TP_DRGR5_10:                         // Group 5 values - one-cycle min/max variance
    case TP_DRGR5_16:                         // Group 5 values - one-cycle min/max variance
    case TP_DRGR5_22:                         // Group 5 values - 200msec min/max variance
    case TP_DRGR5_28:                         // Group 5 values - 200msec min/max variance
    case TP_DRGR5_34:                         // Group 5 values - 200msec min/max variance
      if (TP.SubState == TP_DRGR5_4)
      {
        ptr = &PwrOneCycMin.Pa;
        ptr1 = &PwrOneCycMax.Pa;
      }
      else if (TP.SubState == TP_DRGR5_10)
      {
        ptr = &PwrOneCycMin.RPa;
        ptr1 = &PwrOneCycMax.RPa;
      }
      else if (TP.SubState == TP_DRGR5_16)
      {
        ptr = &PwrOneCycAppMin.AppPa;
        ptr1 = &PwrOneCycAppMax.AppPa;
      }
      else if (TP.SubState == TP_DRGR5_22)
      {
        ptr = &Pwr200msecMin.Pa;
        ptr1 = &Pwr200msecMax.Pa;
      }
      else if (TP.SubState == TP_DRGR5_28)
      {
        ptr = &Pwr200msecMin.RPa;
        ptr1 = &Pwr200msecMax.RPa;
      }
      else
      {
        ptr = &PwrOneCycAppMin.AppPa;
        ptr1 = &PwrOneCycAppMax.AppPa;
      }
      ind = 0;
      for (i=0; i<3; ++i)
      {
        temp = *ptr++;
        temp1 = *ptr1++;
        temp = ((temp1 - temp)/(temp1 + temp)) * (float)(200.0);
        sprintf(&TP.TxValBuf[ind], "% .2E", temp);
        ind += 9;
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
        TP.TxValBuf[ind++] = ' ';
      }
      TP.TxValBuf[39] = '\n';
      TP.TxValBuf[40] = '\r';
      TP.TxValBuf[41] = '\n';
      TP.TxValBuf[42] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 43;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      if (TP.SubState == TP_DRGR5_34)
      {
        TP.State = TP_CURSOR;
      }
      else
      {
        ++TP.SubState;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayRT5Values()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayEnergy()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Energy
//
//  MECHANICS:          This subroutine displays the either the Pha, Phb, Phc, or Sum energy in the
//                      following format:
//                          Fwd WHr: Sum, Residual      Rev WHr: Sum, Residual
//                          Lag VarHr: Sum, Residual    Lead VarHr: Sum, Residual
//                          VAHr: Sum, Residual
//                      Which energy is displayed depends on the code after the DE command:
//                          0 - Pha, 1 - Phb, 2 - Phc, 3 - All (sum)
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, EnergyPha, EnergyPhb, EnergyPhc, EnergySum, ResidualPha, ResidualPhb,
//                      ResidualPhc,  ResidualSum
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sprintf(), TP_GetDecNum(), TP_GetHexNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayEnergy(void)
{
  float temp;
  uint8_t i;

  switch (TP.SubState)
  {
    case TP_DE0:                                // Active Energy
      i = TP_ParseChars();                          // Read next parameter - must be either decimal or hex
      if (i == 1)                                   //   number
      {
        i = (uint8_t)(TP_GetDecNum());
      }
      else if (i == 2)
      {
        i = (uint8_t)(TP_GetHexNum());
      }
      else                                          // If invalid number, set i to 0xFF
      {
        i = 0xFF;
      }

      if (i == 0)                               // DE0 - read Pa energy
      {
        TP.ValPtr4 = &EnergyPha.FwdWHr;
        TP.ValPtr1 = &ResidualPha.FwdWHr;
      }
      else if (i == 1)                          // DE1 - read Pb energy
      {
        TP.ValPtr4 = &EnergyPhb.FwdWHr;
        TP.ValPtr1 = &ResidualPhb.FwdWHr;
      }
      else if (i == 2)                          // DE2 - read Pc energy
      {
        TP.ValPtr4 = &EnergyPhc.FwdWHr;
        TP.ValPtr1 = &ResidualPhc.FwdWHr;
      }
      else if (i == 3)                          // DE3 - read All energy
      {
        TP.ValPtr4 = &EngyDmnd[1].TotFwdWHr;
        TP.ValPtr1 = &ResidualAll.FwdWHr;
      }
      else                                      // Invalid command - just return
      {
        TP.State = TP_CURSOR;
        break;
      }
      TP.SubState = TP_DE1;                     // If command is good, fall into next state

    case TP_DE1:                                // Active Energy
      temp = *TP.ValPtr4++;                         // Convert forward register to floating point to display
      sprintf(&TP.TxValBuf[0], "% .5E", temp);
      TP.TxValBuf[12] = ',';
      TP.TxValBuf[13] = ' ';
      sprintf(&TP.TxValBuf[14], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[26] = ' ';
      TP.TxValBuf[27] = ' ';
      TP.TxValBuf[28] = ' ';
      temp = *TP.ValPtr4++;                         // Convert reverse register to floating point to display
      sprintf(&TP.TxValBuf[29], "% .5E", temp);
      TP.TxValBuf[41] = ',';
      TP.TxValBuf[42] = ' ';
      sprintf(&TP.TxValBuf[43], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[55] = '\n';
      TP.TxValBuf[56] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 57;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DE2;
      break;

    case TP_DE2:                                // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // Fall into next state when done transmitting
      {
        TP.SubState = TP_DE3;
      }
      else
      {
        break;
      }

    case TP_DE3:                              // Reactive Energy
      temp = *TP.ValPtr4++;                         // Convert lagging register to floating point to display
      sprintf(&TP.TxValBuf[0], "% .5E", temp);
      TP.TxValBuf[12] = ',';
      TP.TxValBuf[13] = ' ';
      sprintf(&TP.TxValBuf[14], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[26] = ' ';
      TP.TxValBuf[27] = ' ';
      TP.TxValBuf[28] = ' ';
      temp = *TP.ValPtr4++;                         // Convert leading register to floating point to display
      sprintf(&TP.TxValBuf[29], "% .5E", temp);
      TP.TxValBuf[41] = ',';
      TP.TxValBuf[42] = ' ';
      sprintf(&TP.TxValBuf[43], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[55] = '\n';
      TP.TxValBuf[56] = '\r';
      temp = *TP.ValPtr4;                           // Convert lagging register to floating point to display
      sprintf(&TP.TxValBuf[57], "% .5E", temp);
      TP.TxValBuf[69] = ',';
      TP.TxValBuf[70] = ' ';
      sprintf(&TP.TxValBuf[71], "% .5E", *TP.ValPtr1);
      TP.TxValBuf[83] = '\n';
      TP.TxValBuf[84] = '\r';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 85;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.State = TP_CURSOR;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayEnergy()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayWaveform()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Waveform
//
//  MECHANICS:          This subroutine handles the states in TP_Top() that are used to display waveforms.
//                      The command format is as follows:
//                          DW[Type][Number]<CR>
//                          Type must be one of the following:
//                              U - User-initiated waveform capture (one-cycle only)
//                                  waveform held in UserSamples.OneCyc[Num][SampleNum]
//                                    Num - WF number  0..4: Ia..Igsrc
//                                                     5..7: VanAFE..VcnAFE
//                                    SampleNum - Sample number 0..79
//                              N - Non-Integrated Phase A waveform (10 cycles)
//                                  waveform held in TestSamples[0][SampleNum]
//                                    Num - Cycle number  0..9
//                                    SampleNum - Sample number 0..79
//                              I - Integrated Phase A waveform (10 cycles)
//                                  waveform held in TestSamples[1][SampleNum]
//                                    Num - Cycle number  0..9
//                                    SampleNum - Sample number 0..79
//                              A - ADC Phase A waveform (10 cycles)
//                                  waveform held in TestSamples[2][SampleNum]
//                                    Num - Cycle number  0..9
//                                    SampleNum - Sample number 0..79
//                              F - Filtered and Integrated Phase A waveform (10 cycles)
//                                  waveform held in TestSamples[3][SampleNum]
//                                    Num - Cycle number  0..9
//                                    SampleNum - Sample number 0..79
//                              C - Coil temperature measurement waveform (1 cycle)
//                                  waveform held in coil_temp_samples[SampleNum]
//                                    SampleNum - Sample number 0..79
//                      The subroutine always outputs 80 samples, or one complete cycle, for waveform types
//                      "U", "H", "I", "A",  and "F".  The parameters determine which cycle is sent.
//                      Waveform type "C" is a special case, and the subroutine outputs all 200 samples.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn, UserSamples[][]
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.SubState, TP.RxNdxOut, TP.ValPtr1, TP.Temp
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayWaveform(void)
{
  uint8_t i, j;
  uint16_t tndx;

  switch (TP.SubState)
  {
    case TP_DW0:                              // Set up
      TP.State = TP_CURSOR;                         // Assume bad command
      if (TP.RxNdxOut != TP.RxNdxIn)                // Get the next char in the cmnd - this defines the
      {                                             //   waveform buffer: user, non-integrated phase A, etc.
        i = TP.RxBuf[TP.RxNdxOut] & 0xDF;           // Convert char to upper-case
        TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
        j = TP_ParseChars();                        // Read next parameter - must be decimal number (return
        if (j == 1)                                 //   code equals 1).  This defines the waveform number:
        {                                           //   Ia ... Ig, Van ... Vcn
          j = (uint8_t)(TP_GetDecNum());
        }
        else
        {
          j = 255;                                  // If invalid parameter, set code to invalid
        }
        if (j < 11)                                 // If valid number ...
        {
          if (i == 'U')                                 // Set up for user buffer
          {                                             // Samples are displayed from index=0 to index=79
            if (j > 10)                                 //   Buffer never rolls over.  Set DW_temp to 100
            {                                           //   to ensure we don't reinitialize the pointer
              j = 10;
            }
            TP.ValPtr1 = &UserSamples.OneCyc[j][0];
            TP.Temp = 80;
            DW_temp = 100;
            TP.State = TP_DW;
            TP.SubState = TP_DW1;
          }
          else if (i == 'C')                            // Set up for coil temperature measurement buffer
          {                                             // Samples are displayed from index=0 to index=199
            TP.Temp = 200;                              //   Buffer never rolls over.
            DW_temp = 0;
            TP.State = TP_DW;                                                                            
            TP.SubState = TP_DW3;
          }
          else                                          // Set up for the test buffer
          {                                             // Samples are displayed from index=TestWFind to
//            tndx = (j * 80) + TestWFind;                //   index=((TestWFind+800) MOD 800).  Buffer will   *** DAH CHANGED TO JUST STARTING AT ZERO SINCE ON COLD START, IT IS NOT ROLLING OVER
            tndx = (j * 80);                            //   index=((TestWFind+800) MOD 800).  Buffer will
            if (tndx >= 800)                            //   likely roll over at some point.  Need to check
            {                                           //   for this by checking DW_temp.  Initialize
              tndx -= 800;                              //   DW_temp to the number of characters before the 
            }                                           //   rollover point.  When DW_temp reaches 0, we've
            if ((tndx + 80) > 800)                      //   rolled over, so reinitialize the pointer to the
            {                                           //   zeroth entry, which is held in ValPtr2, and set
              DW_temp = 800 - tndx;                     //   NumChars to 100.
            }                                           //   If we're not going to roll over with this group
            else                                        //   of samples, initialize DW_temp to 100.         
            {
              DW_temp = 100;
            }
            TP.Temp = 80;
            TP.State = TP_DW;
            TP.SubState = TP_DW1;
            if (i == 'N')
            {
              TP.ValPtr1 = &TestSamples[0][tndx];
              TP.ValPtr2 = &TestSamples[0][0];
            }
            else if (i == 'I')
            {
              TP.ValPtr1 = &TestSamples[1][tndx];
              TP.ValPtr2 = &TestSamples[1][0];
            }
            else if (i == 'A')
            {
              TP.ValPtr1 = &TestSamples[2][tndx];
              TP.ValPtr2 = &TestSamples[2][0];
            }
            else if (i == 'F')
            {
              TP.ValPtr1 = &TestSamples[3][tndx];
              TP.ValPtr2 = &TestSamples[3][0];
            }
            else
            {
              TP.State = TP_CURSOR;
            }
          }
        }
      }
      break;

    case TP_DW1:                              // Output set of values
      sprintf(&TP.TxValBuf[0], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[12] = '\n';
      TP.TxValBuf[13] = '\r';
      DW_temp--;
      if (DW_temp == 0)
      {
        DW_temp = 100;
        TP.ValPtr1 = TP.ValPtr2;
      }
      sprintf(&TP.TxValBuf[14], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[26] = '\n';
      TP.TxValBuf[27] = '\r';
      DW_temp--;
      if (DW_temp == 0)
      {
        DW_temp = 100;
        TP.ValPtr1 = TP.ValPtr2;
      }
      sprintf(&TP.TxValBuf[28], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[40] = '\n';
      TP.TxValBuf[41] = '\r';
      DW_temp--;
      if (DW_temp == 0)
      {
        DW_temp = 100;
        TP.ValPtr1 = TP.ValPtr2;
      }
      sprintf(&TP.TxValBuf[42], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[54] = '\n';
      TP.TxValBuf[55] = '\r';
      DW_temp--;
      if (DW_temp == 0)
      {
        DW_temp = 100;
        TP.ValPtr1 = TP.ValPtr2;
      }
      sprintf(&TP.TxValBuf[56], "% .5E", *TP.ValPtr1++);
      TP.TxValBuf[68] = '\n';
      TP.TxValBuf[69] = '\r';
      DW_temp--;
      if (DW_temp == 0)
      {
        DW_temp = 100;
        TP.ValPtr1 = TP.ValPtr2;
      }
      TP.Temp -= 5;
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 70;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DW2;
      break;

    case TP_DW2:                              // Wait for transmission to complete
    case TP_DW4:                              // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        if (TP.Temp > 0)                            // If there are more values to transmit, jump back to
        {                                           //   the previous state
          --TP.SubState;
        }
        else                                        // Otherwise set the state to output the cursor
        {
          TP.State = TP_CURSOR;
        }
      }
      break;

    case TP_DW3:                              // Output the coil measurement values
      sprintf(&TP.TxValBuf[0], "%4u", ((uint32_t)coil_temp_samples[DW_temp++]));
      TP.TxValBuf[4] = '\n';
      TP.TxValBuf[5] = '\r';
      sprintf(&TP.TxValBuf[6], "%4u", ((uint32_t)coil_temp_samples[DW_temp++]));
      TP.TxValBuf[10] = '\n';
      TP.TxValBuf[11] = '\r';
      sprintf(&TP.TxValBuf[12], "%4u", ((uint32_t)coil_temp_samples[DW_temp++]));
      TP.TxValBuf[16] = '\n';
      TP.TxValBuf[17] = '\r';
      sprintf(&TP.TxValBuf[18], "%4u", ((uint32_t)coil_temp_samples[DW_temp++]));
      TP.TxValBuf[22] = '\n';
      TP.TxValBuf[23] = '\r';
      sprintf(&TP.TxValBuf[24], "%4u", ((uint32_t)coil_temp_samples[DW_temp++]));
      TP.TxValBuf[28] = '\n';
      TP.TxValBuf[29] = '\r';
      TP.Temp -= 5;
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 30;
      // Display the RMS value of the waveform after the waveform samples
      if(TP.Temp == 0)
      {
        // RMS value is shown as a float limited to 4 digits plus the decimal point.
        float RMS = TP_CoilTempRMSavg();
        TP.TxValBuf[30] = '\n';
        TP.TxValBuf[31] = '\r';
        sprintf(&TP.TxValBuf[32], "%.1f", RMS);
        TP.TxValBuf[37] = '\n';
        TP.TxValBuf[38] = '\r';
        TP.NumChars = 39;
      }
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_DW4;
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayWaveform()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayStartup()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Startup Values
//
//  MECHANICS:          This subroutine displays the ADC reading of the startup capacitor voltage and the
//                      corresponding calculated startup time.
//                      The values are displayed as follows:
//                          Startup time ADC reading in decimal
//                          Startup time in floating point
//                          Startup time conversion costant in floating point
//                          Thermal memory time ADC reading in decimal
//                      
//  CAVEATS:            None
//
//  INPUTS:             StartUpADC, StartupTime.Time
// 
//  OUTPUTS:            TP.TxValBuf[], TP.Status, TP.TxValNdx, TP.NumChars
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayStartup(void)
{
  uint8_t i;
  float tVcap_pct;

  switch (TP.SubState)
  {
    case TP_DS0:                        // Display Startup Time Values
      for (i=0; i<17; ++i)                  // Assemble the following in the transmit buffer:
      {                                     //   "Startup Time ADC Val: "<Startup ADC Counts>
        TP.TxValBuf[i] = STARTUP_ADC_COUNT_STR[i];
      }
      sprintf(&TP.TxValBuf[17], "% .4u", (unsigned int)(StartUpADC));
      TP.TxValBuf[21] = '\n';
      TP.TxValBuf[22] = '\r';
      for (i=0; i<14; ++i)                  // Assemble the following in the transmit buffer:
      {                                     //   "Startup Time: "<Startup Time>
        TP.TxValBuf[i+23] = STARTUP_TIME[i];
      }
      sprintf(&TP.TxValBuf[37], "% .2E", StartupTime.Time);
      TP.TxValBuf[46] = '\n';
      TP.TxValBuf[47] = '\r';
      for (i=0; i<19; ++i)                  // Assemble the following in the transmit buffer:
      {                                     //   "Startup Time Scale Val: "<Startup Scale Factor>
        TP.TxValBuf[i+48] = STARTUP_SCALE_STR[i];
      }
      sprintf(&TP.TxValBuf[67], "% .5E", StartupTime.Scale);
      TP.TxValBuf[79] = '\n';
      TP.TxValBuf[80] = '\r';
      TP.Status &= (~TP_TX_STRING);         // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;             // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 81;
      UART5->CR1 |= USART_CR1_TXEIE;        // Enable transmit interrupts
      TP.SubState = TP_DS1;
      break;

    case TP_DS2:                        // Display Thermal Memory Values
      if (Setpoints1.stp.ThermMem == 1)     // If thermal memory is enabled, show the values
      {
        for (i=0; i<19; ++i)                    // Assemble the following in the transmit buffer:
        {                                       //   "Therm mem ADC Val: "<Thermal memory ADC Counts>
          TP.TxValBuf[i] = THERM_ADC_COUNT_STR[i];
        }
        sprintf(&TP.TxValBuf[19], "% .4u", (unsigned int)(ThermalMemoryADC));
        TP.TxValBuf[23] = '\n';
        TP.TxValBuf[24] = '\r';
        for (i=0; i<19; ++i)                    // Assemble the following in the transmit buffer:
        {                                       //   "Therm mem pct: "<Thermal memory per cent>
          TP.TxValBuf[i+25] = THERM_PCT_STR[i];
        }
        // Compute % of charge on Therm Mem cap at power-up
        tVcap_pct = ((float)ThermalMemoryADC/VCAP_MAX * 100.0);
        sprintf(&TP.TxValBuf[44], "% .3E", tVcap_pct);
        TP.TxValBuf[54] = '\n';
        TP.TxValBuf[55] = '\r';
        TP.NumChars = 56;
      }
      else                                  // Otherwise display string saying it is disabled
      {
        for (i=0; i<20; ++i)
        {
          TP.TxValBuf[i] = THERM_MEM_DISABLED_STR[i];
        }
        TP.NumChars = 20;
      }
      TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
      TP.TxValNdx = 0;
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = TP_DS3;
      break;

    case TP_DS4:                        // Display Internal Timing Values
      for (i=0; i<20; ++i)                  // Assemble the following in the transmit buffer:
      {                                     //   "Reset to PLL Count: "<Reset to PLL count>
        TP.TxValBuf[i] = RESET_TO_PLL_COUNT_STR[i];
      }
      sprintf(&TP.TxValBuf[20], "% .3u", (unsigned int)(Reset_to_PLL_Count));
      TP.TxValBuf[23] = '\n';
      TP.TxValBuf[24] = '\r';
      for (i=0; i<15; ++i)                  // Assemble the following in the transmit buffer:
      {                                     //   "Max Loop Time: "<Max Loop Time>
        TP.TxValBuf[i+25] = MAX_LOOP_TIME_STR[i];
      }
      // maxlooptime is a U32 in nsec.  Compute max loop time in sec and as a float
      tVcap_pct = ((float)maxlooptime)/1E9;
      sprintf(&TP.TxValBuf[40], "% .3e", tVcap_pct);
      TP.TxValBuf[50] = '\n';
      TP.TxValBuf[51] = '\r';
      TP.Status &= (~TP_TX_STRING);         // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;             // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 52;
      UART5->CR1 |= USART_CR1_TXEIE;        // Enable transmit interrupts
      TP.State = TP_CURSOR;
      break;

    case TP_DS1:                        // Wait until done transmitting
    case TP_DS3:                        // Wait until done transmitting
      if (!(TP.Status & TP_TX_VALUE))       // When done transmitting jump to next state
      {
        TP.SubState++;
      }
      break;                                // Otherwise remain in this state

    default:
      TP.State = TP_CURSOR;
      break;
 }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayStartup()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_ModifyCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Modify Calibration Constants Command
//
//  MECHANICS:          This subroutine performs the tasks associated with the Modify Calibration Constants
//                      (MC) command.  There are three sets of calibration constants.  One set is for the
//                      AFE, one for the high-gain ADC, and one for the low-gain ADC.  Each set of constants
//                      has eight channels (0 - 7), and each channel contains a gain and offset constant.
//                      The subroutine operates as follows:
//                          State 0:
//                            - Read the command modifier: "A" - AFE, "H" - high-gain ADC, "L" - low-gain
//                              ADC, else invalid
//                            - Set pointers to the corresponding constants in memory
//                          State 1:
//                            - Output the label ("Gain x: ", where x=0..7) and present gain value
//                          State 2:
//                            - Wait for user input, ending with a <LF>
//                            - If input is just a <CRLF>, skip to State 3 (no changes)
//                            - If input is a ".", quit without saving to FRAM.  Note, any modifications
//                              will still be present in RAM.  The old values will be retrieved from FRAM
//                              after a reset.
//                            - Any other input, convert to floating point and save in RAM
//                          State 3:
//                            - Output the label ("Offset x: ", where x=0..7) and present gain value
//                          State 4:
//                            - Wait for user input, ending with a <LF>
//                            - If input is just a <CRLF>, either skip back to State 1 (no changes) if there
//                              are more channels to look at.  If this is the last channel, save the values
//                              to FRAM and exit.
//                            - If input is a ".", quit without saving to FRAM.  Note, any modifications
//                              will still be present in RAM.  The old values will be retrieved from FRAM
//                              after a reset.
//                            - Any other input, convert to floating point and save in RAM.  If this is the
//                              last channel, save the values to FRAM and exit.  Otherwise, go back to
//                              State 1 to do the next channel.
//                      
//  CAVEATS:            The constants must be entered in floating point numbers
//
//                      
//  INPUTS:             TP.SubState, TP.RxNdxIn, TP.RxBuf[], AFEcal.x[], ADCcalHigh.x,  ADCcalLow.x
// 
//  OUTPUTS:            TP.State, AFEcal.x[], ADCcalHigh.x,  ADCcalLow.x, TP.TxValBuf[],TP.NumChars,
//                      TP.Status
//
//  ALTERS:             TP.SubState, TP.RxNdxOut, TP.RxNdxIn
// 
//  CALLS:              sprintf(), ComputeChksum32(), atof(), TP_GetDecNum()
//
//------------------------------------------------------------------------------------------------------------

void TP_ModifyCal(void)
{
  uint8_t i, MC_exit;
  uint16_t temp;

  MC_exit = FALSE;                      // Initialize exit flag to False

  while (!MC_exit)
  {
    switch (TP.SubState)
    {
      case TP_MC0:                      // Set up
        TP.State = TP_CURSOR;                       // Assume bad command
        if (TP.RxNdxOut != TP.RxNdxIn)              // Get the next character in the command, if there is
        {                                           //   one
           i = TP.RxBuf[TP.RxNdxOut] & 0xDF;        // Convert char to upper-case
           TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
           if (i == 'A')                            // Set up for AFE
           {
             TP.ValPtr1 = &AFEcal.gain[0];
             TP.ValPtr2 = &AFEcal.offset[0];
             TP.ValPtr3 = &AFEcal.phase[0];
             TP.Temp = 0;
             TP.State = TP_MC;
             TP.SubState = TP_MC1;
           }
           else if (i == 'H')                       // Set up for High-Gain ADC
           {
             TP.ValPtr1 = &ADCcalHigh.gain[0];
             TP.ValPtr2 = &ADCcalHigh.offset[0];
             TP.Temp = 0;
             TP.State = TP_MC;
             TP.SubState = TP_MC7;
           }
           else if (i == 'L')                       // Set up for Low-Gain ADC
           {
             TP.ValPtr1 = &ADCcalLow.gain[0];
             TP.ValPtr2 = &ADCcalLow.offset[0];
             TP.Temp = 0;
             TP.State = TP_MC;
             TP.SubState = TP_MC11;
           }
         }
         MC_exit = TRUE;
         break;

      case TP_MC1:                      // Output Gain Value
      case TP_MC7:                      // Output Gain Value
      case TP_MC11:                     // Output Gain Value
        for (i=0; i<4; ++i)                 // Assemble the following in the transmit buffer:
        {                                   //   "Gain <channel>: <gain constant> "
          TP.TxValBuf[i] = GAIN[i];
        }
        TP.TxValBuf[4] = ' ';
        TP.TxValBuf[5] = 0x30 + TP.Temp;
        TP.TxValBuf[6] = ':';
        TP.TxValBuf[7] = ' ';
        sprintf(&TP.TxValBuf[8], "%.5E", *TP.ValPtr1);
        TP.TxValBuf[20] = ' ';
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 21;                   //   21 chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted  value does not roll over past the
        //   end of TP.RxBuf[].  This will mess up the value returned from atof(), since it doesn't realize
        //   the buffer rolls over.  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.SubState++;
        MC_exit = TRUE;
        break;

      case TP_MC2:                      // Read New Gain Value
      case TP_MC8:                      // Read New Gain Value
      case TP_MC12:                     // Read New Gain Value
        i = TP.RxNdxOut;
        while (i != TP.RxNdxIn)             // Check the Rx buffer for a line feed - this indicates that a
        {                                   //   new value has been entered
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            i = TP.RxNdxIn;                         // Set i to TP.RxNdxIn to force exit from loop
            if (TP.RxBuf[TP.RxNdxOut] == '.')       // If first char is a ".", exit
            {
              TP.State = TP_CURSOR;
            }
            else                                    // Otherwise either skip this char or process the input
            {
              TP.SubState++;                        // Either way, next state is to do the offset
              if ( (TP.RxBuf[TP.RxNdxOut] != 0x0A)      // If first char is not a line feed or carriage
                && (TP.RxBuf[TP.RxNdxOut] != 0x0D) )    //   return, process the input
              {
                *TP.ValPtr1 = atof((char const *)(&TP.RxBuf[TP.RxNdxOut]));
              }                                         // Otherwise this constant stays as is
            }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          else                                      // If no line feed, increment index to check the next
          {                                         //   character
            i = (i + 1) & 0x1F;
          }
        }
        MC_exit = TRUE;
        break;

      case TP_MC3:                      // Output Offset Value
      case TP_MC9:                      // Output Offset Value
      case TP_MC13:                     // Output Offset Value
        for (i=0; i<6; ++i)                 // Assemble the following in the transmit buffer:
        {                                   //   "Offset <channel>: <offset constant> "
          TP.TxValBuf[i] = OFFSET[i];
        }
        TP.TxValBuf[6] = ' ';
        TP.TxValBuf[7] = 0x30 + TP.Temp;
        TP.TxValBuf[8] = ':';
        TP.TxValBuf[9] = ' ';
        sprintf(&TP.TxValBuf[10], "%.5E", *TP.ValPtr2);
        TP.TxValBuf[22] = ' ';
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 23;                   //   21 chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted  value does not roll over past the
        //   end of TP.RxBuf[].  This will mess up the value returned from atof(), since it doesn't realize
        //   the buffer rolls over.  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.SubState++;
        MC_exit = TRUE;
        break;

      case TP_MC4:                      // Read New Offset Value
      case TP_MC10:                     // Read New Offset Value
      case TP_MC14:                     // Read New Offset Value
        i = TP.RxNdxOut;
        while (i != TP.RxNdxIn)             // Check the Rx buffer for a line feed - this indicates that a
        {                                   //   new value has been entered
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            i = TP.RxNdxIn;                         // Set i to TP.RxNdxIn to force exit from loop
            if (TP.RxBuf[TP.RxNdxOut] == '.')       // If first char is a ".", exit
            {
              TP.State = TP_CURSOR;
            }
            else                                    // Otherwise either skip this char or process the input
            {                                       //   Next state is the same either way
              if ( (TP.RxBuf[TP.RxNdxOut] != 0x0A)      // If first char is not a line feed or carriage
                && (TP.RxBuf[TP.RxNdxOut] != 0x0D) )    //   return, process the input
              {
                *TP.ValPtr2 = atof((char const *)(&TP.RxBuf[TP.RxNdxOut]));
              }                                         // Otherwise this constant stays as is
              if (TP.SubState == TP_MC4)                // If doing AFE cal constants, set next state to do
              {                                         //   phase
                TP.SubState = TP_MC5;
              }
              else if ( (TP.Temp >= 7)                  // Otherwise if last character (char count = 0
                && (TP.SubState == TP_MC10) )           //   thru 7) for ADC high constants, done
              {
                                                            // Compute and save new checksum and complement
                ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
                ADCcalHigh.cmp = ~ADCcalHigh.chk;
                TP.State = TP_WR_CAL;                       // Set next state to write cal constants to FRAM
              }                                             //   and Flash
              else if ( (TP.Temp >= 7)                  // If last character (char count = 0 thru 7), for
                && (TP.SubState == TP_MC14) )           //   ADC low cal constants, done
              {
                                                            // Compute and save new checksum and complement
                ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
                ADCcalLow.cmp = ~ADCcalLow.chk;
                TP.State = TP_WR_CAL;                       // Set next state to write cal constants to FRAM
              }                                             //   and Flash
              else                                      // Otherwise not AFE and not done, so increment char
              {                                         //   count and jump back to do gain
                TP.Temp++;
                TP.ValPtr1++;
                TP.ValPtr2++;
                TP.SubState -= 3;
              }
            }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          else                                      // If no line feed, increment index to check the next
          {                                         //   character
            i = (i + 1) & 0x1F;
          }
        }
        MC_exit = TRUE;
        break;

      case TP_MC5:                      // Output Phase Value
        for (i=0; i<5; ++i)                 // Assemble the following in the transmit buffer:
        {                                   //   "Phase <channel>: <phase constant> "
          TP.TxValBuf[i] = PHASE[i];
        }
        TP.TxValBuf[i++] = ' ';
        if (TP.Temp <= 9)
        {
          TP.TxValBuf[i++] = 0x30 + TP.Temp;
        }
        else
        {
          TP.TxValBuf[i++] = 0x31;
          TP.TxValBuf[i++] = 0x30 + (TP.Temp - 10);
        }
        TP.TxValBuf[i++] = ':';
        TP.TxValBuf[i++] = ' ';
        temp = *TP.ValPtr3;                 // temp has phase compensation value.  Number is from 0 to 255
        if (temp > 99)                      // If three digits, output ms digit
        {
          TP.TxValBuf[i++] = (temp/100) + 0x30;
        }
        if (temp > 9)                       // If two or three digits, make sure third digit is shaved off
        {                                   //   then output second digit
          temp = temp % 100;
          TP.TxValBuf[i++] = (temp/10) + 0x30;
        }
        temp = temp % 10;                   // Make sure second digit is shaved off
        TP.TxValBuf[i++] = (temp) + 0x30;   // Output LS digit
        TP.TxValBuf[i++] = ' ';
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = i;                    //   Number of chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted  value does not roll over past the
        //   end of TP.RxBuf[].  This will mess up the value returned from atof(), since it doesn't realize
        //   the buffer rolls over.  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.SubState = TP_MC6;
        MC_exit = TRUE;
        break;

      case TP_MC6:                      // Read New Phase Value
        i = TP.RxNdxOut;
        while (i != TP.RxNdxIn)             // Check the Rx buffer for a line feed - this indicates that a
        {                                   //   new value has been entered
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            i = TP.RxNdxIn;                         // Set i to TP.RxNdxIn to force exit from loop
            if (TP.RxBuf[TP.RxNdxOut] == '.')       // If first char is a ".", exit
            {
              TP.State = TP_CURSOR;
            }
            else                                    // Otherwise either skip this char or process the input
            {
              if ( (TP.RxBuf[TP.RxNdxOut] != 0x0A)      // If first char is not a line feed or carriage
                && (TP.RxBuf[TP.RxNdxOut] != 0x0D) )    //   return, process the input
              {
                *TP.ValPtr3 = (uint8_t)(TP_GetDecNum());
              }                                         // Otherwise this constant stays as is
              if (TP.Temp >= 11)                        // If last character (char count = 0 thru 11), done
              {
                                                           // Compute and save new checksum and complement
                AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
                AFEcal.cmp = ~AFEcal.chk;
                
                Update_AFE_Sync_Regs();                 // Send new phase offset to AFE chip
                
                TP.State = TP_WR_CAL;                      // Set next state to write cal constants to FRAM
              }                                            //   and Flash
              else                                      // Otherwise increment character count and jump back
              {                                         //   to do next set of values.  If char count
                TP.Temp++;                              //   (TP.Temp) < 10, need to do gain, offset, and
                TP.ValPtr1++;                           //   phase so jump back to MC1.  If TP.Temp >= 10,
                TP.ValPtr2++;                           //   just need to do phase so jump back to MC5
                TP.ValPtr3++;
                TP.SubState = ((TP.Temp < 10) ? TP_MC1 : TP_MC5);
              }
            }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          else                                      // If no line feed, increment index to check the next
          {                                         //   character
            i = (i + 1) & 0x1F;
          }
        }
        MC_exit = TRUE;
        break;

      default:                          // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        MC_exit = TRUE;
        break;

    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_ModifyCal()
//------------------------------------------------------------------------------------------------------------



extern void Gen_Values(void);

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_ModifyProtSetting()       *** DAH 210730   ADDED FOR TEST PURPOSES ONLY - DELETE LATER
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Modify Protection Setting Command
//
//  MECHANICS:          This subroutine displays and allows a user to change the instantaneous pickup level.
//                      
//  CAVEATS:            The setting must be entered as a uint16
//
//                      
//  INPUTS:             TP.SubState, TP.RxNdxIn, TP.RxBuf[], Setpoints1.stp.Inst_Pu
// 
//  OUTPUTS:            TP.State, Setpoints1.stp.Inst_Pu
//
//  ALTERS:             TP.SubState, TP.RxNdxOut, TP.RxNdxIn
// 
//  CALLS:              sprintf(), TP_GetDecNum(), Gen_Values()
//
//------------------------------------------------------------------------------------------------------------

void TP_ModifyProtSetting(void)
{
  uint8_t i, MT_exit;

  MT_exit = FALSE;                      // Initialize exit flag to False

  while (!MT_exit)
  {
    switch (TP.SubState)
    {
      case TP_MT0:                      // Output Existing Value
        sprintf(&TP.TxValBuf[0], "%4u", ((uint32_t)Setpoints1.stp.Inst_Pu));
        TP.TxValBuf[4] = ' ';
        TP.TxValBuf[5] = ':';
        TP.TxValBuf[6] = ' ';
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 7;                   //   21 chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted  value does not roll over past the
        //   end of TP.RxBuf[].  This will mess up the value returned from atof(), since it doesn't realize
        //   the buffer rolls over.  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.SubState++;
        MT_exit = TRUE;
        break;

      case TP_MT1:                      // Read New Value
        i = TP.RxNdxOut;
        while (i != TP.RxNdxIn)             // Check the Rx buffer for a line feed - this indicates that a
        {                                   //   new value has been entered
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            i = TP.RxNdxIn;                         // Set i to TP.RxNdxIn to force exit from loop
            TP.State = TP_CURSOR;                   // Set next state to output cursor since done
            if (TP.RxBuf[TP.RxNdxOut] == '.')       // If first char is a ".", exit
            {
            }
            else                                    // Otherwise either skip this char or process the input
            {
              if (TP_ParseChars() == 1)             // Read next parameter.  If parameter is a decimal
              {                                     //   number, retrieve it
                Setpoints1.stp.Inst_Pu = (uint16_t)(TP_GetDecNum());
                Gen_Values();                               // Generate protection limits based on setpoints
              }                                     // Otherwise do nothing
            }
          }
          else                              // If no line feed, increment index to check the next character
          {
            i = (i + 1) & 0x1F;
          }
        }
        MT_exit = TRUE;
        break;

      default:                          // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        MT_exit = TRUE;
        break;

    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_ModifyProtSetting()
//------------------------------------------------------------------------------------------------------------






//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_RTC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           RTC Test Handler
//
//  MECHANICS:          This subroutine handles setting and testing the RTC and internal time.  The
//                      subroutine:
//                        1) Reads the existing RTC time
//                        2) Outputs this time and accepts changes from the user
//                        3) Prompts the user to either save the new time or cancel
//                        If the new time is to be saved, the subroutine:
//                          4) Captures the existing internal time
//                          5) Writes the new time into the internal timer
//                          6) Captures the new internal time
//                          7) Processes the time adjustment (for demand and energy logging)
//                          8) Writes the new time into the RTC
//                      
//  CAVEATS:            The subroutine relies on the fact that IntRTC_Update() is completed in one call, so
//                      we don't have to worry corrupting an update that is in progress.  RTC_Buf[] and
//                      IntTimeBuf[] are used in both subroutines, but the values are saved in TxValBuf[]
//                      before this subroutine is exited
//
//  INPUTS:             TP.SubState, TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState, TP.RxNdxOut
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), RTC_to_SysTickTime(), Get_InternalTime(),
//                      ProcessTimeAdjustment(), IntRTC_Read()
// 
//------------------------------------------------------------------------------------------------------------

void TP_RTC(void)
{
  struct INTERNAL_TIME oldtime, newtime;
  uint16_t temp;
  uint8_t i, TM_exit;

  TM_exit = FALSE;                      // Initialize exit flag to False

  while (!TM_exit)
  {
    switch (TP.SubState)
    {
      case TP_TM0:                      // Read Existing RTC Values
        if (IntRTC_Read())                  // Read RTC time.  If read is successful...
        {
          InternalTimeRead();                   // Read internal time also
          for (i=0; i<11; ++i)                  // Save the two times in the transmit buffer
          {                                     // RTC_buf[] and IntTimeBuf[] are also used IntRTC_Update()
            TP.TxValBuf[40+i] = RTC_buf[i];
            if (i < 8)
            {
              TP.TxValBuf[60+i] = IntTimeBuf[i];
            }
          }
          TP.Temp = 0;                          // Initialize temporary to be used as index
          TP.SubState = TP_TM1;                 // Fall into next state
        }
        else                                // If read is unsuccessful...
        {
          TP.SubState = TP_TM0;                 // Quit
          TP.State = TP_CURSOR;
          TM_exit = TRUE;
          break;
        }          

      case TP_TM1:                      // Output Existing Time Values
        // RTC Time:
        //   RTC_buf[0] = hundredths of a second in hexadecimal (0 - 99)
        //   RTC_buf[1] = seconds in hexadecimal (0 - 59)
        //   RTC_buf[2] = minutes in hexadecimal (0 - 59)
        //   RTC_buf[3] = hours in hexadecimal (0 - 23)
        //   RTC_buf[4] = day in hexadecimal (1 - 7, 1 = Sunday)
        //   RTC_buf[5] = date in hexadecimal (1 - 31)
        //   RTC_buf[6] = month in hexadecimal (1 - 12)
        //   RTC_buf[7] = year in hexadecimal (0 - 99)
        //   RTC_buf[8] = low byte of CR reg
        //                   b7..6 = 0
        //                   b5 = 1: Time and date values are taken directly from the registers
        //                   b4..0 = 0
        //   RTC_buf[9] = low byte of Marker1
        //   RTC_buf[10] = low byte of Marker2
        // Internal Time:
        //   IntTimeBuf[0]: hundredths of a second in hex
        //   IntTimeBuf[1]: seconds in hex (0 - 59)
        //   IntTimeBuf[2]: minutes in hex (0 - 59)
        //   IntTimeBuf[3]: hours in hex (0 - 23)
        //   IntTimeBuf[4]: weekday in hex (1 - 7, 1 = Sunday)
        //   IntTimeBuf[5]: date in hex (1 - 31)
        //   IntTimeBuf[6]: month in hex (1 - 12)
        //   IntTimeBuf[7]: year in hex (0 - 99)
        // Move value to temporary so when when converting to ASCII BCD will be using register
        //   (faster processing)
        i = TP.TxValBuf[40+TP.Temp];
        i = ( ((i/10) << 4) | (i % 10) );           // Convert to BCD
        TP.TxValBuf[0] = (i >> 4) + 0x30;
        TP.TxValBuf[1] = (i & 0x0F) + 0x30;
        TP.TxValBuf[2] = ' ';
        TP.TxValBuf[3] = ' ';
        TP.TxValBuf[4] = ' ';
        i = TP.TxValBuf[60+TP.Temp];
        i = ( ((i/10) << 4) | (i % 10) );           // Convert to BCD
        TP.TxValBuf[5] = (i >> 4) + 0x30;
        TP.TxValBuf[6] = (i & 0x0F) + 0x30;
        TP.TxValBuf[7] = ':';
        TP.TxValBuf[8] = ' ';
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 9;                    //   4 chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted value does not roll over past the
        //   end of TP.RxBuf[].  This will make it easier to process the input value.  Also set RxNdxOut
        //   equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.SubState = TP_TM2;
        TM_exit = TRUE;
        break;

      case TP_TM2:                      // Read New Time Value
        i = TP.RxNdxOut;
        while ( (i != TP.RxNdxIn)           // Check the Rx buffer for a line feed - this indicates that a
             && (TP.SubState == TP_TM2) )   //   that a new value has been entered
        {
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            if (TP.RxBuf[TP.RxNdxOut] == '.')       // If first char is a ".", exit
            {
              TP.State = TP_CURSOR;
              TP.SubState = TP_TM0;
            }
            else                                    // Otherwise either skip this char or process the input
            {                                       //   Next state is the same either way
              if ( (TP.RxBuf[TP.RxNdxOut] != 0x0A)      // If first char is not a line feed or carriage
                && (TP.RxBuf[TP.RxNdxOut] != 0x0D) )    //   return, process the input
              {
                // If decimal number, get it and store.  Use TP_GetDecNum() to get and convert the ASCII
                //   number since we want to convert it from ASCII-encoded decimal to binary-coded decimal
                if (TP_ParseChars() == 1)
                {
                  TP.TxValBuf[40+TP.Temp] = (uint8_t)(TP_GetDecNum());
                }
              }                                         // Otherwise the register will not be changed
              if (++TP.Temp > 7)                        // First eight registers (TP.Temp = 0..7) are
              {                                         //   handled this way.  If index is 8, done so jump
                TP.SubState = TP_TM3;                   //   to TM3 to output the control register bits
              }
              else                                      // Otherwise jump back to do the next register
              {                                         //   (still doing the first eight registers)
                TP.SubState = TP_TM1;
              }
            }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          i = (i + 1) & 0x1F;                       // Increment the index
        }
        TM_exit = TRUE;
        break;

      case TP_TM3:                      // Output Existing RTC Flags
        //  RTC_buf[8] = low byte of control reg:
        //    b5 is the only bit of interest - it should be 0.  The rest should also be 0
        TP.TxValBuf[0] = '0';
        TP.TxValBuf[1] = 'x';
        temp = dhex_ascii((uint8_t)TP.TxValBuf[48]);
        TP.TxValBuf[2] = (uint8_t)(temp >> 8);
        TP.TxValBuf[3] = (uint8_t)(temp);
        TP.TxValBuf[4] = 0x0A;
        TP.TxValBuf[5] = 0x0D;
        TP.Status &= (~TP_TX_STRING);       // Set up to transmit
        TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 6;                    //   6 chars
        UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
        // Reinitialize the output buffer indices so that any inputted value does not roll over past the
        //   end of TP.RxBuf[].  This will make it easier to process the input value.  Also set RxNdxOut
        //   equal to RxNdxIn to ensure a new start to parsing.
        TP.RxNdxOut = 0;
        TP.RxNdxIn = 0;
        TP.Temp++;                          // Increment buffer index
        TP.SubState = TP_TM4;
        TM_exit = TRUE;
        break;

      case TP_TM4:                      // Output Marker1 Value
      case TP_TM5:                      // Output Marker2 Value
        // If done transmitting, output the next value.  This is required to ensure the first value is done
        //   being transmitted before the next value is written into the Tx buffer
        if (!(TP.Status & TP_TX_VALUE))
        {
          //  RTC_buf[9] = location 0x20 = 0x55
          //  RTC_buf[10] = location 0x21 = 0xAD
          TP.TxValBuf[0] = '0';
          TP.TxValBuf[1] = 'x';
          temp = dhex_ascii(TP.TxValBuf[40+TP.Temp]);
          TP.TxValBuf[2] = (uint8_t)(temp >> 8);
          TP.TxValBuf[3] = (uint8_t)(temp);
          TP.TxValBuf[4] = 0x0A;
          TP.TxValBuf[5] = 0x0D;
          TP.Status &= (~TP_TX_STRING);       // Set up to transmit
          TP.Status |= TP_TX_VALUE;           //   Transmitting values, not string
          TP.TxValNdx = 0;
          if (TP.SubState == TP_TM4)
          {
            TP.NumChars = 6;                  // Six chars
            ++TP.Temp;                        // Increment index for next value
          }
          else
          {
            TP.TxValBuf[6] = '<';
            TP.TxValBuf[7] = 'C';
            TP.TxValBuf[8] = 'R';
            TP.TxValBuf[9] = 'L';
            TP.TxValBuf[10] = 'F';
            TP.TxValBuf[11] = '>';
            TP.TxValBuf[12] = '-';
            TP.TxValBuf[13] = 'S';
            TP.TxValBuf[14] = 'a';
            TP.TxValBuf[15] = 'v';
            TP.TxValBuf[16] = 'e';
            TP.TxValBuf[17] = ',';
            TP.TxValBuf[18] = ' ';
            TP.TxValBuf[19] = '"';
            TP.TxValBuf[20] = '.';
            TP.TxValBuf[21] = '"';
            TP.TxValBuf[22] = '-';
            TP.TxValBuf[23] = 'Q';
            TP.TxValBuf[24] = 'u';
            TP.TxValBuf[25] = 'i';
            TP.TxValBuf[26] = 't';
            TP.TxValBuf[27] = ':';
            TP.TxValBuf[28] = ' ';
            TP.NumChars = 29;                 //   29 chars
          }
          UART5->CR1 |= USART_CR1_TXEIE;      // Enable transmit interrupts - this begins the transmissions
          // Reinitialize the output buffer indices so that any inputted value does not roll over past the
          //   end of TP.RxBuf[].  This will make it easier to process the input value.  Also set RxNdxOut
          //   equal to RxNdxIn to ensure a new start to parsing.
          TP.RxNdxOut = 0;
          TP.RxNdxIn = 0;
          ++TP.SubState;
        }
        TM_exit = TRUE;
        break;

      case TP_TM6:                      // Read Save or Quit command
        i = TP.RxNdxOut;
        while ( (i != TP.RxNdxIn)           // Check the Rx buffer for a line feed - this indicates that a
             && (TP.SubState == TP_TM6) )   //   new value has been entered
        {
          if (TP.RxBuf[i] == 0x0A)          // If LF received, check whether it is a new value
          {
            if (TP.RxBuf[TP.RxNdxOut] != '.')       // If first char is not a ".", process the time
            {                                       //   adjustment
              __disable_irq();                             // capture the present time
              Get_InternalTime(&oldtime);
              __enable_irq();
              for (i=0; i<11; ++i)                         // Move values back into RTC_buf[] to write into
              {                                            //   the internal value
                RTC_buf[i] = TP.TxValBuf[40+i];
              }
              // Disable interrupts before writing the new values to internal time to prevent corruption by
              //   the SysTick interrupt
              __disable_irq();
              RTC_to_SysTickTime(&SysTickTime);            // write the new values to internal time
              Get_InternalTime(&newtime);                  // capture the new time
              __enable_irq();
              ProcessTimeAdjustment(&oldtime, &newtime);   // call subroutine to process the time adjustment
              // Set state to initiate an RTC time update.  Note, this relies on the fact that
              //   IntRTC_Update() is completed in one call to the subroutine, so we don't have to worry
              //   corrupting an update that is in progress.  Still, we will check to be sure it is ok.
              //   It is ok if RTC_State is already 1, but cannot be greater than that.
              if (RTC_State <= 1)
              {
                RTC_State = 1;
              }
              DPComm.Flags |= TX_TIME;             // Set flag to send time to the display processor
            }                                      // If char is a ".", just exit
            TP.SubState = TP_TM0;                  // Reset the substate
            TP.State = TP_CURSOR;                  // Jump to the cursor state
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
          }
          i = (i + 1) & 0x1F;                       // Increment the index
        }
        TM_exit = TRUE;
        break;

      default:                          // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        TM_exit = TRUE;
        break;
    }
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_RTC()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_THSensor()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Temperature/Humidity Sensor Test Handler
//
//  MECHANICS:          This subroutine handles the testing the HIH6030 Temperature/Humidity Sensor
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, THSensor.xx, HLTH_I2C.Flags
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status, TP.TxValNdx, TP.NumChars, HLTH_I2C.Flags
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_THSensor(void)
{
  uint8_t TH_exit;

  TH_exit = FALSE;                      // Initialize exit flag to False

  while (!TH_exit)
  {
    switch (TP.SubState)
    {
      case TP_TH0:                      // Read Sensor Values
        HLTH_I2C.Flags |= READ_SENSOR;                // Set flag to read the sensor
        TP.SubState = TP_TH1;
        TH_exit = TRUE;
        break;

      case TP_TH1:                      // Wait For Read to Complete
        if ((HLTH_I2C.Flags & READ_SENSOR) == 0)      // If the read has been completed, set up to display
        {                                             //   the values
          TP.TxValBuf[0] = 'T';
          TP.TxValBuf[1] = ':';
          TP.TxValBuf[2] = ' ';
          sprintf(&TP.TxValBuf[3], "% .5E", THSensor.Temperature);
          TP.TxValBuf[15] = '\n';
          TP.TxValBuf[16] = '\r';
          TP.TxValBuf[17] = 'H';
          TP.TxValBuf[18] = ':';
          TP.TxValBuf[19] = ' ';
          sprintf(&TP.TxValBuf[20], "% .5E", THSensor.Humidity);
          TP.TxValBuf[32] = '\n';
          TP.TxValBuf[33] = '\r';
          TP.Status &= (~TP_TX_STRING);               // Set up to transmit
          TP.Status |= TP_TX_VALUE;                   //   Transmitting values, not string
          TP.TxValNdx = 0;
          TP.NumChars = 34;                           //   22 chars
          UART5->CR1 |= USART_CR1_TXEIE;              // Enable xmit interrupts - this begins transmissions
          TP.State = TP_CURSOR;
        }
        TH_exit = TRUE;                               // Otherwise haven't gotten sensor values yet, just
        break;                                        //   exit

      default:                          // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        TH_exit = TRUE;
        break;
    }
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_THSensor()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_OffsetCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Offset Calibration
//
//  MECHANICS:          This subroutine handles offset calibration for the AFE, ADC high gain, and ADC low
//                      gain conversions.  The calibration is performed on a single channel.  Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated
//                          2) Set the offset for the channel being calibrated to 0.
//                          3) Set the hardware to the circuit being calibrated (AFE, ADC high gain, or ADC
//                             low gain)
//                          4) Initiate a user waveform capture.  A single cycle is captured.  Sum the 80
//                             samples - this is the DC offset for this cycle.
//                          5) Repeat this process 100 times, and keep a running total of the offset.
//                          6) When 100 cycles of offset has been accumulated, divide the result by
//                             (100 * 80 * gain) - 100 because 100 cycles, 80 because 80 samples.  The
//                             result is the DC offset per sample.
//                          7) Save the result in the offset for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      Valid commands are:
//                          OCA0 - Offset cal of AFE Ia     OCA1 - Offset cal of AFE Ib
//                          OCA2 - Offset cal of AFE Ic
//                          OCA3 - Offset cal of AFE In, Rogowski or CT, depending on configuration
//                          OCA4 - Offset cal of AFE Igsrc, Rogowski or CT, depending on configuration
//                          OCA5 - Offset cal of AFE Van    OCA6 - Offset cal of AFE Vbn
//                          OCA7 - Offset cal of AFE Vcn
//                          OCH0 - Offset cal of high-gain ADC Ia     OCH1 - Offset cal of high-gain ADC Ib
//                          OCH2 - Offset cal of high-gain ADC Ic
//                          OCH3, OCH4 - Offset cal of high-gain ADC In, Rogowski or CT, depending on config
//                          OCH5 - Offset cal of ADC Van    OCH6 - Offset cal of ADC Vbn
//                          OCH7 - Offset cal of ADC Vcn
//                          OCL0 - Offset cal of low-gain ADC Ia     OCL1 - Offset cal of low-gain ADC Ib
//                          OCL2 - Offset cal of low-gain ADC Ic
//                          OCL3, OCL4 - Offset cal of low-gain ADC In, Rogowski or CT, depending on config
//                          OCL5 - Offset cal of ADC Van    OCL6 - Offset cal of ADC Vbn
//                          OCL7 - Offset cal of ADC Vcn
//                      Note:
//                        TP.ValPtr2 points to the offset for the channel being calibrated
//                        *(TP.ValPtr2 - 10) for the AFE and *(TP.ValPtr2 - 8) for the ADC points to the
//                              gain for the channel being calibrated
//                        TP.ValPtr1 points to the user waveform samples for the channel being calibrated
//                        TP.Tmp1.f holds the running total of the offset for the 100 cycles
//                        TP.Temp holds the cycle count
//                      
//  CAVEATS:            The subroutine assumes the input to the channel (shorted input) has been properly
//                      set up.
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), CaptureUserWaveform()
// 
//------------------------------------------------------------------------------------------------------------

void TP_OffsetCal(void)
{
  uint8_t i, j, tp_ofc_exit;;

  tp_ofc_exit = FALSE;

  while (!tp_ofc_exit)
  {
    switch (TP.SubState)
    {
      case TP_OC0:                        // Read circuit and channel to be calibrated
        // Read the next parameter - circuit to be calibrated.  This must be a letter:
        //   a-AFE, h-ADC High Gain, l-ADC Low Gain
        TP.State = TP_CURSOR;                 // Assume bad command
        if (TP.RxNdxOut != TP.RxNdxIn)        // Get the next char in the cmnd - this defines the waveform
        {                                     //   buffer: user, half-cyc, one-cyc.  Store in i
          i = TP.RxBuf[TP.RxNdxOut] & 0xDF;   // Convert char to upper-case
          TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
          // Read the next parameter - channel to be calibrated.  This must be decimal number:
          //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igrsc (for OCA), Not Used (Vn) for OCL and OCH, 5-Van, 6-Vbn, 7-Vcn
          //   Store the channel in j
          j = TP_ParseChars();
          if (j == 1)                         // If valid, get the number
          {
            j = (uint8_t)(TP_GetDecNum());
          }
          else                                // If not decimal number, set to invalid code
          {
            j = 255;
          }
          if ( (j < 8) && ((i == 'A') || (i == 'H') || (i == 'L')) )
          {
            if (i == 'A')                     // Calibrating AFE circuit
            {
              UseADCvals = FALSE;                           // Make sure using AFE values
              if ( (j == 4) && (GF_USING_ROGOWSKI) )        // If Igsrc calibration and using a Rogowski coil,
              {                                             //   cal constant is held in index = 8
                TP.ValPtr2 = &AFEcal.offset[8];
                TP.RxNdxIn = 16;
              }
              else if ( (j == 3) && (!IN_USING_ROGOWSKI) )  // If In calibration and using a CT, cal constant
              {                                             //   is held in index = 9
                TP.ValPtr2 = &AFEcal.offset[9];
                TP.RxNdxIn = 12;
              }
              else                                          // Otherwise cal constant is in the entered index
              {                                             //   (0 - 7).  Igsrc with a CT is at index = 4,
                TP.ValPtr2 = &AFEcal.offset[j];             //   In with a Rogowski at index = 3
                TP.RxNdxIn = ( (j < 3) ? (j * 4) : (24 + ((j - 5) * 2)) );
              }
              *TP.ValPtr2 = 0.0;                       // Initialize offset for channel of interest to 0
              TP.ValPtr1 = &UserSamples.OneCyc[j][0];
              TP.SubState = TP_OC1;
            }
            else if (i == 'H')                // Calibrating High Gain circuit
            {
              if ((j == 3) || (j == 4))             // If calibrating In (j=3 or 4)...
              {
                if (!IN_USING_ROGOWSKI)                 // If using a CT, cal constant is stored at index = 4
                {
                  TP.ValPtr2 = &ADCcalHigh.offset[4];
                  ADCcalHigh.offset[4] = 0.0;           // Initialize offset for channel of interest to 0
                }
                else                                    // Otherwise using a Rogowski, so cal constant is
                {                                       //   stored at index = 3
                  TP.ValPtr2 = &ADCcalHigh.offset[3];
                  ADCcalHigh.offset[3] = 0.0;           // Initialize offset for channel of interest to 0
                }
                TP.RxNdxIn = 12;
              }
              else                                  // Otherwise cal constants are located at the index
              {
                TP.ValPtr2 = &ADCcalHigh.offset[j];
                ADCcalHigh.offset[j] = 0.0;
                TP.RxNdxIn = ( (j < 3) ? (j * 4) : (24 + ((j - 5) * 2)) );
              }
              TP.SubState = TP_OC3;
              if (j < 5)
              {
                UseADCvals = TRUE;
                if (j == 4)                                           // If j=4, calibrating In, so set
                {                                                     //   pointer to OneCyc[3][0]
                  TP.ValPtr1 = &UserSamples.OneCyc[3][0];
                }
                else                                                  // Otherwise use entered index
                {
                  TP.ValPtr1 = &UserSamples.OneCyc[j][0];
                }
                HIGH_GAIN_INPUTS;
              }
              else
              {
                TP.ValPtr1 = &UserSamples.OneCyc[j+3][0];
              }
            }
            else                              // Calibrating Low Gain circuit
            {
              if ((j == 3) || (j == 4))             // If calibrating In (j=3 or 4)...
              {
                if (!IN_USING_ROGOWSKI)                 // If using a CT, cal constant is stored at index = 4
                {
                  TP.ValPtr2 = &ADCcalLow.offset[4];
                  ADCcalLow.offset[4] = 0.0;            // Initialize offset for channel of interest to 0
                }
                else                                    // Otherwise using a Rogowski, so cal constant is
                {                                       //   stored at index = 3
                  TP.ValPtr2 = &ADCcalLow.offset[3];
                  ADCcalLow.offset[3] = 0.0;            // Initialize offset for channel of interest to 0
                }
                TP.RxNdxIn = 12;
              }
              else                                  // Otherwise cal constants are located at the index
              {
                TP.ValPtr2 = &ADCcalLow.offset[j];
                ADCcalLow.offset[j] = 0.0;
                TP.RxNdxIn = ( (j < 3) ? (j * 4) : (24 + ((j - 5) * 2)) );
              }
              TP.SubState = TP_OC5;
              if (j < 5)
              {
                UseADCvals = TRUE;
                if (j == 4)                                           // If j=4, calibrating In, so set
                {                                                     //   pointer to OneCyc[3][0]
                  TP.ValPtr1 = &UserSamples.OneCyc[3][0];
                }
                else                                                  // Otherwise use entered index
                {
                  TP.ValPtr1 = &UserSamples.OneCyc[j][0];
                }
                LOW_GAIN_INPUTS;
              }
              else
              {
                TP.ValPtr1 = &UserSamples.OneCyc[j+3][0];
              }
            }
            TP.Temp = 0;
            TP.Tmp1.f = 0;
            TP.State = TP_OC;
          }
        }
        break;
    
        
      case TP_OC1:                        // Accumulate readings - AFE
        // Unlock the waveform capture process.  This may mess up some other process that is using a
        //   captured waveform, but we are calibrating, so it shouldn't matter.
        UserWF.Locked = FALSE;
        // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
        //   other process is using a captured waveform - this should never happen (see above)
        if (CaptureUserWaveform(USR_TYPE_ALL))
        {
          for (i=0; i<80; ++i)                        // Add all values to temporary sum.  The sum of all
          {                                           //   of the values is the DC offset
            TP.Tmp1.f += TP.ValPtr1[i];
          }
          // If accumulated 100 readings, take average and set offset to the average (per sample average is
          // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
          // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 10)
          if (TP.Temp == 99)
          {
            *TP.ValPtr2 = TP.Tmp1.f/(8000.0f * *(TP.ValPtr2 - 10));
                                                            // Compute and save new checksum and complement
            AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
            AFEcal.cmp = ~AFEcal.chk;
            UserWF.Locked = FALSE;                          // Free up user captures
            TP.State = TP_WR_CAL;                           // Set next state to write cal constants to FRAM
          }                                                 //   and Flash
          else                                        // If not done accumulating readings, increment the
          {                                           //   counter, capture present 10msec count,and go to
            TP.Temp++;                                //    the delay state.  We will initiate a new request
            TP.SubState++;                            //    in about 30msec.  Note, TP.TxValNdx is used to
            TP.TxValNdx = SysTickTime.cnt_10msec;     //    hold the present 10msec count
          }
        }
        tp_ofc_exit = TRUE;
        break;
    
      case TP_OC3:                        // Accumulate readings - ADC High
        // Unlock the waveform capture process.  This may mess up some other process that is using a
        //   captured waveform, but we are calibrating, so it shouldn't matter.
        UserWF.Locked = FALSE;
        // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
        //   other process is using a captured waveform - this should never happen (see above)
        if (CaptureUserWaveform(USR_TYPE_ALL))
        {
          for (i=0; i<80; ++i)                        // Add all values to temporary sum.  The sum of all
          {                                           //   of the values is the DC offset
            TP.Tmp1.f += TP.ValPtr1[i];
          }
          // If accumulated 100 readings, take average and set offset to the average (per sample average is
          // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
          // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 8)
          if (TP.Temp == 99)
          {
            *TP.ValPtr2 = TP.Tmp1.f/(8000.0f * *(TP.ValPtr2 - 8));
                                                            // Compute and save new checksum and complement
            ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
            ADCcalHigh.cmp = ~ADCcalHigh.chk;
            UserWF.Locked = FALSE;                          // Free up user captures
            TP.State = TP_WR_CAL;                           // Set next state to write cal constants to FRAM
          }                                                 //   and Flash
          else                                        // If not done accumulating readings, increment the
          {                                           //   counter, capture present 10msec count,and go to
            TP.Temp++;                                //    the delay state.  We will initiate a new request
            TP.SubState++;                            //    in about 30msec.  Note, TP.TxValNdx is used to
            TP.TxValNdx = SysTickTime.cnt_10msec;     //    hold the present 10msec count
          }
        }
        tp_ofc_exit = TRUE;
        break;
    
      case TP_OC5:                        // Accumulate readings - ADC Low
        // Unlock the waveform capture process.  This may mess up some other process that is using a
        //   captured waveform, but we are calibrating, so it shouldn't matter.
        UserWF.Locked = FALSE;
        // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
        //   other process is using a captured waveform - this should never happen (see above)
        if (CaptureUserWaveform(USR_TYPE_ALL))
        {
          for (i=0; i<80; ++i)                        // Add all values to temporary sum.  The sum of all
          {                                           //   of the values is the DC offset
            TP.Tmp1.f += TP.ValPtr1[i];
          }
          // If accumulated 100 readings, take average and set offset to the average (per sample average is
          // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
          // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 8)
          if (TP.Temp == 99)
          {
            *TP.ValPtr2 = TP.Tmp1.f/(8000.0f * *(TP.ValPtr2 - 8));
                                                        // Compute and save new checksum and complement
            ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
            ADCcalLow.cmp = ~ADCcalLow.chk;
            UserWF.Locked = FALSE;                          // Free up user captures
            TP.State = TP_WR_CAL;                           // Set next state to write cal constants to FRAM
          }                                                 //   and Flash
          else                                        // If not done accumulating readings, increment the
          {                                           //   counter, capture present 10msec count,and go to
            TP.Temp++;                                //    the delay state.  We will initiate a new request
            TP.SubState++;                            //    in about 30msec.  Note, TP.TxValNdx is used to
            TP.TxValNdx = SysTickTime.cnt_10msec;     //    hold the present 10msec count
          }
        }
        tp_ofc_exit = TRUE;
        break;
    
    
      case TP_OC2:                        // Wait a minimum of 30msec
      case TP_OC4:                        // Wait a minimum of 30msec
      case TP_OC6:                        // Wait a minimum of 30msec
        // Compute elapsed time since last capture.  We need to wait at least 30msec to ensure we get new
        //   samples for the next capture (50Hz --> 20msec, 10msec resolution)
        // The counter rolls over to 0 at 100 (0 - 99)
        i = ( (SysTickTime.cnt_10msec >= TP.TxValNdx) ?
                  (SysTickTime.cnt_10msec - TP.TxValNdx) : ((100 + SysTickTime.cnt_10msec) - TP.TxValNdx) );
        if (i > 4)                        // Make 40msec to be safe
        {
          TP.SubState--;
        }
        else
        {
          tp_ofc_exit = TRUE;
        }
        break;
    
    
      default:                            // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        tp_ofc_exit = TRUE;
        break;
    
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_OffsetCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_GainCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the AFE, ADC high gain, and ADC low
//                      gain conversions.  The calibration is performed on a single channel.  Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated, along with the actual input
//                             current
//                          2) Set the hardware to the circuit being calibrated (AFE, ADC high gain, or ADC
//                             low gain)
//                          3) Wait for a new reading.  When the flag is set, add the new reading to the
//                             running total in Tmp1.f.
//                          4) Repeat this process 100 times.
//                          5) When 100 readings have been accumulated, divide the result by 100 to get the
//                             average reading.
//                          6) Multiply the existing gain constant by the actual input divided by the
//                             average reading
//                          7) Save the result in the gain for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      The 200msec reading is used for AFE calibration because this is the reading that is
//                      used for metering.  The one-cycle reading is used for ADC calibration because this
//                      is the reading that is used for protection.  When the ADC readings are used, the
//                      current is greater than the upper limit for revenue-grade metering.
//                      Valid commands are:
//                          GCA0 - Gain cal of AFE Ia
//                          GCA1 - Gain cal of AFE Ib
//                          GCA2 - Gain cal of AFE Ic
//                          GCA3 - Gain cal of AFE In, Rogowski or CT, depending on configuration
//                          GCA4 - Gain cal of AFE Igsrc, Rogowski or CT, depending on configuration
//                          GCA5 - Gain cal of AFE Van
//                          GCA6 - Gain cal of AFE Vbn
//                          GCA7 - Gain cal of AFE Vcn
//                          GCH0 - Gain cal of high-gain ADC Ia
//                          GCH1 - Gain cal of high-gain ADC Ib
//                          GCH2 - Gain cal of high-gain ADC Ic
//                          GCH3, GCH4 - Gain cal of high-gain ADC In, Rogowski or CT, depending on config
//                          GCH5 - Gain cal of ADC Van
//                          GCH6 - Gain cal of ADC Vbn
//                          GCH7 - Gain cal of ADC Vcn
//                          GCL0 - Gain cal of low-gain ADC Ia
//                          GCL1 - Gain cal of low-gain ADC Ib
//                          GCL2 - Gain cal of low-gain ADC Ic
//                          GCL3, GCL4 - Gain cal of low-gain ADC In, Rogowski or CT, depending on config
//                          GCL5 - Gain cal of ADC Van
//                          GCL6 - Gain cal of ADC Vbn
//                          GCL7 - Gain cal of ADC Vcn
//                      Note:
//                        TP.ValPtr2 points to the gain for the channel being calibrated
//                        TP.ValPtr1 points to the 200msec (for the AFE) or the one-cycle (for the ADC)
//                              reading
//                        TP.Tmp2.f holds the actual input current (enterd by the user)
//                        TP.Tmp1.f holds the running total of the offset for the 100 readings
//                        TP.Temp holds the cycle count
//                      Note:
//                        The ADC voltages are the same for both the high-gain and low-gain circuits.  In
//                        other words, the high gain and low-gain circuits only affect the currents.
//                        However, the high-gain and low-gain constants use the same storage structure, and
//                        are stored separately - the ADC voltage cal constants are kept in both the
//                        high-gain and the low-gain cal structures.
//                      
//  CAVEATS:            The subroutine assumes the input to the channel has been properly set up.
//                      Offset calibration should be performed first!!
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_GainCal(void)
{
  uint8_t i, j;
  uint8_t tempbuf[17];

  switch (TP.SubState)
  {
    case TP_GC0:                        // Read circuit and channel to be calibrated
      // Read the next parameter - circuit to be calibrated.  This must be a letter:
      //   a-AFE, h-ADC High Gain, l-ADC Low Gain
      TP.State = TP_CURSOR;                 // Assume bad command
      if (TP.RxNdxOut != TP.RxNdxIn)        // Get the next char in the cmnd - this defines the waveform
      {                                     //   buffer: user, half-cyc, one-cyc
        i = TP.RxBuf[TP.RxNdxOut] & 0xDF;   // Convert char to upper-case
        TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
        // Read the next parameter - channel to be calibrated.  This must be decimal number:
        //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igrsc (for GCA), Not Used (Vn) for GCL and GCH, 5-Van, 6-Vbn, 7-Vcn
        //   Store the channel in j
        j = TP_ParseChars();
        if (j == 1)                         // If valid, get the number
        {
          j = (uint8_t)(TP_GetDecNum());
        }
        else                                // If not decimal number, set to invalid code
        {
          j = 255;
        }
        // Save existing setup.  High nibble gets channel.  Low nibble gets high or low gain ADC inputs
//        TP.Temp = j << 4;                     
//        TP.Temp += (USING_HIGH_GAIN ? 1 : 0);

        // If circuit and channel are valid, continue with setup
        if ( (j < 8) && ((i == 'A') || (i == 'H') || (i == 'L')) )
        {
          if (i == 'A')                     // Calibrating AFE circuit
          {
            UseADCvals = FALSE;                      // Make sure using AFE values
            TP.ValPtr2 = &AFEcal.gain[j];            // Initialize ValPtr2
            if ( (j == 4) && (GF_USING_ROGOWSKI) )   // If Igsrc calibration and using a Rogowski coil, cal
            {                                        //   constant is held in index = 8
              TP.ValPtr2 = &AFEcal.gain[8];
              TP.ValPtr1 = &Cur200msFltr.Igsrc;
            }
            else if ( (j == 3) && (!IN_USING_ROGOWSKI) )  // If In calibration and using a CT, cal constant
            {                                             //   is held in index = 9
              TP.ValPtr2 = &AFEcal.gain[9];
              TP.ValPtr1 = &Cur200msFltr.In;
            }
            else if (j < 5)                          // If Ia, Ib, Ic, In, or Igsrc with CT, cal constant is
            {                                        //   at the entered index (0 - 4)
              TP.ValPtr1 = &Cur200msFltr.Ia + j;     // Note, j is promoted to float address and is
            }                                        //   multiplied by four by the compiler
            else                                     // Otherwise VanADC - Vcn_ADC
            {
              TP.ValPtr1 = &VolAFE200msFltr.Van + (j - 5);
            }
            TP.SubState = TP_GC1;
          }
          else if (i == 'H')                // Calibrating High Gain circuit
          {
            if ( ((j == 3) || (j == 4)) && (!IN_USING_ROGOWSKI) )   // If calibrating In (j=3 or 4) and
            {                                                       //   using a CT, cal constant is stored
              TP.ValPtr2 = &ADCcalHigh.gain[4];                     //   at index = 4
            }
            else                                                    // Otherwise cal constant is stored at
            {                                                       //   the entered index
              TP.ValPtr2 = &ADCcalHigh.gain[j];
            }
            TP.SubState = TP_GC2;
            if (j < 5)
            {
              UseADCvals = TRUE;
              if (j == 4)                                           // If j=4, calibrating In, so set
              {                                                     //   pointer to &CurOneCyc.In
                TP.ValPtr1 = &CurOneCyc.Ia + 3;
              }
              else                                                  // Otherwise use entered index
              {
                TP.ValPtr1 = &CurOneCyc.Ia + j;
              }
              HIGH_GAIN_INPUTS;
            }
            else
            {
              TP.ValPtr1 = &VolADCOneCyc.Van + (j - 5);
            }
          }
          else                              // Calibrating Low Gain circuit
          {
            if ( ((j == 3) || (j == 4)) && (!IN_USING_ROGOWSKI) )   // If calibrating In (j=3 or 4) and
            {                                                       //   using a CT, cal constant is stored
              TP.ValPtr2 = &ADCcalLow.gain[4];                      //   at index = 4
            }
            else                                                    // Otherwise cal constant is stored at
            {                                                       //   the entered index
              TP.ValPtr2 = &ADCcalLow.gain[j];
            }
            TP.SubState = TP_GC3;
            if (j < 5)
            {
              UseADCvals = TRUE;
              if (j == 4)                                           // If j=4, calibrating In, so set
              {                                                     //   pointer to &CurOneCyc.In
                TP.ValPtr1 = &CurOneCyc.Ia + 3;
              }
              else                                                  // Otherwise use entered index
              {
                TP.ValPtr1 = &CurOneCyc.Ia + j;
              }
              LOW_GAIN_INPUTS;
            }
            else
            {
              TP.ValPtr1 = &VolADCOneCyc.Van + (j - 5);
            }
          }
          // Move the characters comprising the input current entered by the user to a separate buffer to
          //   ensure there is no wraparound.  The atof() function does not handle wraparound of the buffer.
          for (i=0; i<16; ++i)
          {
            tempbuf[i] = TP.RxBuf[TP.RxNdxOut];
            TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
          }
          tempbuf[16] = 0x00;               // Make sure the string is terminated
          TP.Tmp2.f = atof((char const *)(&tempbuf[0]));
          if ((TP.Tmp2.f > 0) && (TP.Tmp2.f < 40E3))       // *** DAH THIS MAY NEED TO BE INCREASED
          {
            TP.Temp = 0;
            TP.Tmp1.f = 0;
            TP.State = TP_GC;
          }
        }
      }
      break;

    case TP_GC1:                        // Accumulate readings - AFE
      if (SystemFlags & VAL200MSEC)         // If new reading, add to temporary sum
      {
        if (TP.Temp > 0)                    // Throw out the first reading since it may include samples from
        {                                   //   the ADC circuit
          TP.Tmp1.f += *TP.ValPtr1;
        }
        // If accumulated 100 readings, compute the average reading (sum/100).  Adjust the gain by the ratio
        // of the ideal reading (in Tmp2.f) amd the average reading (in Tmp1.f).  Note, gain is referenced
        // by *TP.ValPtr2
        if (TP.Temp == 100)
        {
          TP.Tmp1.f = TP.Tmp1.f/100.0f;
          *TP.ValPtr2 = (*TP.ValPtr2 * TP.Tmp2.f)/TP.Tmp1.f;
                                                      // Compute and save new checksum and complement
          AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
          AFEcal.cmp = ~AFEcal.chk;
          TP.State = TP_WR_CAL;                         // Set next state to write cal constants to FRAM and
        }                                               //   Flash
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.Temp++;
        }
      }
      break;

    case TP_GC2:                         // Accumulate readings - ADC High
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        if (TP.Temp > 0)                    // Throw out the first reading since it may have samples from
        {                                   //   the AFE or ADC low-gain circuit.
          TP.Tmp1.f += *TP.ValPtr1;
        }
        if (TP.Temp == 100)                 // If accumulated 100 readings, compute average and set gain to
        {                                   //   gain * (ideal value)/(average value).
          TP.Tmp1.f = TP.Tmp1.f/100.0f;
          *TP.ValPtr2 = (*TP.ValPtr2 * TP.Tmp2.f)/TP.Tmp1.f;
                                                      // Compute and save new checksum and complement
          ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalHigh.cmp = ~ADCcalHigh.chk;
          TP.State = TP_WR_CAL;                         // Set next state to write cal constants to FRAM and
        }                                               //   Flash
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.Temp++;
        }
      }
      break;

    case TP_GC3:                         // Accumulate readings - ADC Low
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        if (TP.Temp > 0)                    // Throw out the first reading since it may have samples from
        {                                   //   the AFE or ADC high-gain circuit.
          TP.Tmp1.f += *TP.ValPtr1;
        }
        if (TP.Temp == 100)                 // If accumulated 100 readings, compute average and set gain to
        {                                   //   gain * (ideal value)/(average value).
          TP.Tmp1.f = TP.Tmp1.f/100.0f;
          *TP.ValPtr2 = (*TP.ValPtr2 * TP.Tmp2.f)/TP.Tmp1.f;
                                                      // Compute and save new checksum and complement
          ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalLow.cmp = ~ADCcalLow.chk;
          TP.State = TP_WR_CAL;                         // Set next state to write cal constants to FRAM and
        }                                               //   Flash
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.Temp++;
        }
      }
      break;
        
    default:                            // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_GainCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_PhaseCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Phase Calibration
//
//  MECHANICS:          This subroutine handles phase calibration between current and voltage for the AFE
//                      for power and energy conversions.  The calibration is performed on a single channel.
//                      Calibration is performed as follows:
//                          1) Set up the source with current in phase with voltage (PF = 1.0).  The current
//                             should be about 600A RMS.  The voltage should be about 120V RMS.
//                          2) Enter PCx, where x = channel to be calibrated
//                          3) The code then computes the average power over 100 200msec readings (about 20
//                             seconds total).  The power (P1.0) is returned.
//                          4) Set up the source with a 60 degree phase shift between current and voltage
//                             (PF = 0.5).  It shouldn't matter whether the current is leading or lagging.
//                             The amplitudes MUST be the same as for the previous setup.
//                          5) Enter PCx, where x = channel to be calibrated
//                          6) The code then computes the average power over 100 200msec readings (about 20
//                             seconds total).  The power (P0.5) is returned.
//                          7) The user or automated tester then computes the value to be written to the
//                             channel's Sync Offset Register as follows:
//                                  - phase error = (P0.5 - P1.0/2)/(P1.0/2)
//                                  - phase error in degrees = ARCSIN(phase error/SQRT(3))
//                                  - adjustment = phase error/.00432
//                                  - register value = 104 - adjustment
//                          8) The user or automated tester writes the register value to either the
//                             channel's current or voltage Sync Offset Register using the MCA command.  The
//                             current register is written if P0.5 is less than P1.0/2.  The voltage
//                             register is written if P0.5 is greater than P1.0/2.
//                             
//                      Note:
//                        TP.ValPtr1 points to the 200msec real power reading
//                        TP.Tmp1.f holds the running total of the power for the 100 readings
//                        TP.Temp holds the cycle count
//
//                      Reference the AD7779 data sheet
//                      
//  CAVEATS:            The subroutine assumes the input to the channel has been properly set up.
//
//  INPUTS:             TP.SubState, TP.RxNdxOut, TP.RxNdxIn, SystemFlags
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[]
//
//  ALTERS:             TP.SubState, UseADCvals, TP.ValPtr1, TP.Temp, TP.Tmp1.f, TP.Status, TP.NumChars,
//                      TP.TxValNdx
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_PhaseCal(void)
{
  uint8_t j;

  switch (TP.SubState)
  {
    case TP_PC0:                        // Read step and channel to be calibrated
      // Read the next parameter - channel to be calibrated.  This must be a decimal number:
      //   0-Ia, 1-Ib, 2-Ic
      TP.State = TP_CURSOR;                 // Assume bad command
      if (TP.RxNdxOut != TP.RxNdxIn)        // Get the next char in the cmnd - this defines the calibration
      {                                     //   step: 1, 2
        j = TP_ParseChars();
        if (j == 1)                         // If valid, get the number
        {
          j = (uint8_t)(TP_GetDecNum());
        }
        else                                // If not decimal number, set to invalid code
        {
          j = 255;
        }
        // If channel is valid, continue with setup
        if (j < 3)
        {
          UseADCvals = FALSE;                   // Make sure using AFE values
          // Set pointer to the power for the channel being calibrated.
          // Note, j is promoted to float address and is multiplied by four by the compiler
          TP.ValPtr1 = &Pwr200msec.Pa + j;
          TP.Temp = 0;
          TP.Tmp1.f = 0;
          TP.State = TP_PC;
          TP.SubState = TP_PC1;
        }
      }
      break;

    case TP_PC1:                        // Accumulate readings
      if (SystemFlags & VAL200MSEC)         // If new reading, add to temporary sum
      {
        if (TP.Temp > 0)                    // Throw out the first reading since it may include samples from
        {                                   //   the ADC circuit
          TP.Tmp1.f += *TP.ValPtr1;
        }
        // If accumulated 100 readings, compute and output the average reading (sum/100)
        if (TP.Temp == 100)
        {
          TP.Tmp1.f = TP.Tmp1.f/100.0f;
          sprintf(&TP.TxValBuf[0], "% .5E", TP.Tmp1.f);
          TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
          TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
          TP.TxValNdx = 0;
          TP.NumChars = 12;
          UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
          TP.State = TP_CURSOR;
          break;
        }
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.Temp++;
        }
      }
      break;
       
    default:                            // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_PhaseCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestInjCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Calibration
//
//  MECHANICS:          This subroutine reads the command string to determine whether to perform offset or
//                      gain calibration for the on-board test-injection circuit, and sets the two state
//                      variables accordingly
//                      
//  CAVEATS:            None
//
//  INPUTS:             None
// 
//  OUTPUTS:            TP.State, TP.SubState
//
//  ALTERS:             None
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), TP_GetHexNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestInjCal(void)
{
  uint8_t i;

  TP.State = TP_CURSOR;                     // Assume bad command
  if (TP.RxNdxOut != TP.RxNdxIn)            // Get the next character in the command, if there is one
  {
    i = TP.RxBuf[TP.RxNdxOut] & 0xDF;       // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if (i == 'O')                           // Set up for offset calibration
    {
      TP.State = TP_IOC;
      TP.SubState = TP_IOC0;
    }
    else if (i == 'S')                      // Set up for sine gain calibration
    {
      TP.State = TP_IGC;
      TP.SubState = TP_ISC0;
    }
    else if (i == 'D')                      // Set up for dc gain calibration
    {
      TP.State = TP_IGC;
      TP.SubState = TP_IDC0;
    }
  }                                         // Otherwise leave next state at idle to ignore the command
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestInjCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestInjOffsetCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Offset Calibration
//
//  MECHANICS:          This subroutine handles the offset calibration for the on-board test injection
//                      circuit.  Basically, this subroutine finds the ADC code that produces zero current
//                      (where V_TEST+ and V_TEST- are equal).  Calibration consists of the following steps:
//                          1) Set the amplitude to 0 so will only use the DC outputs
//                          2) Enable the test injection circuit and set the output to the nominal value
//                          3) Enable the test injection output
//                          4) Read the one-cycle current (cannot use the 200msec current because the DC
//                             filter blocks the signal).  Repeat this 10 times and compute the average
//                             reading
//                          5) If the new reading is less than the original reading, increment the count and
//                             repeat step 4.  If the new reading is greater than the original reading, set
//                             a flag and decrement the count.
//                          6) Continue incrementing the count until the minimum value is found or the count
//                             reaches 3000 (error condition).
//                      Note:
//                        The midpoint only needs to be found using one channel - channel A is used.  Tests
//                        conducted on 160616 showed that it is the same regardless of the channel.  This
//                        makes sense, since the resolution of the DAC is significantly less than the offset
//                        error.
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//                      
//  CAVEATS:            TEST_INJ_ON must be the last flag set (set the other flags before this one to ensure
//                      there are no conflicts with the interrupt) when turning on test injection!!
//
//  INPUTS:             TP.SubState, CurOneCyc.Ia, SystemFlags, SPI1Flash.Ack
// 
//  OUTPUTS:            TP.State, TestInj.x, TestInjCal.x, TP.Status, TP.TxValNdx, TP.NumChars, TP.State,
//                      TestInj.Channel, Manufacture_Mode
//                      SPI1Flash.Req (S1F_INJ_WR)
//
//  ALTERS:             TP.SubState, TP.ValPtr1, TP.NumChars, TP.Tmp1.f, TP.Tmp2
// 
//  CALLS:              sprintf(), FRAM_Stat_Write(), FRAM_Write()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestInjOffsetCal(void)
{
  uint8_t j;

  switch (TP.SubState)
  {
    case TP_IOC0:                       // Set up for initial readings
      // Read the next parameter - channel to be calibrated.  This must be a decimal number:
      //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igsrc.    Save the channel number in temporary
      TP.State = TP_CURSOR;                 // Assume bad command
      if (TP.RxNdxOut != TP.RxNdxIn)        // Get the next char in the cmnd
      {
        j = TP_ParseChars();
        if (j == 1)                         // If valid, get the number
        {
          j = (uint8_t)(TP_GetDecNum());
        }
        else                                // If not decimal number, set to invalid code
        {
          j = 255;
        }
        // If channel is valid, continue with setup
        if (j < 5)
        {
          TestInj.Amplitude = 0;                  // Set amplitude to zero so will be DC output
          TestInj.MidPoint = 2375;                // Set the midpoint to the nominal value  *** DAH MAY WANT TO INCREASE THIS TO REDUCE CAL TIME AFTER WE GET MORE READINGS FROM OTHER BOARDS
          TSTINJ_PHA_OFF;                         // Make sure all of the test channels are off to start
          TSTINJ_PHB_OFF;                         //   with
          TSTINJ_PHC_OFF;
          TSTINJ_PHN_OFF;
          TSTINJ_DIS;
          TestInj.CosIndx = 20;                   // Initialize cosine index (COS[n] = SIN[n+20])
          TestInj.Channel = j;                    // Save channel
          TP.ValPtr1 = &CurOneCyc.Ia + j;         // Initialize pointer to the current for the cal channel
          Manufacture_Mode = TRUE;                // Put into Manufacturing Mode so protection is disabled
          // Turn off thermal memory so the bucket is cleared when we are done calibrating
          TP.Temp = Setpoints1.stp.ThermMem;      // Save thermal memory setpoint
          Setpoints1.stp.ThermMem = 0;
          TestInj.Flags |= TEST_INJ_INIT_ON;      // Set flag to initialize test inj.  Note, set this flag
                                                  //   before setting flag to turn on test injection
          TestInj.Flags |= TEST_INJ_ON;           // Turn on test injection
          TP.NumChars = 0;                        // Initialize count.  NumChars is used as a temporary
          TP.Tmp2.f = 0;                          // Initialize sum of readings
          TP.Tmp1.f = 1E36;                       // Initialize initial sum of readings
          TP.State = TP_IOC;
          TP.SubState = TP_IOC1;
        }
      }
      break;

    case TP_IOC1:                       // Wait for settling time
    // Testing on 160526 showed that the analog front-end circuit and digital integrator takes about
    //   60 passes to settle from a cold start.  Wait 180 passes.
    if (SystemFlags & VALONECYC)            // Wait for a new reading...
    {
      TP.NumChars++;
      if (TP.NumChars >= 180)                   // If wait time is up, reset count and move on to get new
      {                                         //   readings                                          
        TP.NumChars = 0;
        TP.SubState = TP_IOC2;
      }
    }
    break;
      


    case TP_IOC2:                       // Accumulate one-cycle AFE current readings
      if (SystemFlags & VALONECYC)          // Wait for a new reading...
      {
        TP.Tmp2.f += *TP.ValPtr1;               // Add the reading to a running total
        if (TP.NumChars >= 9)                   // If accumulated 10 readings...
        {
          TP.Tmp2.f = TP.Tmp2.f/10.0;               // Compute the average current reading
          if (TP.Tmp2.f < TP.Tmp1.f)                // If new current is less than old current...
          {
            TestInj.MidPoint++;                         // Increment the midpoint
            if (TestInj.MidPoint > 2494)                // If midpoint reaches limit, error so done
            {
              TP.SubState = TP_IOC3;
              TestInj.Flags |= TEST_INJ_INIT_OFF;           // Set flag to turn off test injection
            }
            else                                        // If no error, reset count, save new current, and
            {                                           //   repeat this step with the new midpoint
              TP.NumChars = 120;                        // Since only changing one count, will only wait 60
              TP.Tmp1.f = TP.Tmp2.f;                    //   (180 - 120) passes.  Testing on 160526 showed
              TP.Tmp2.f = 0;                            //   it takes about 30 passes.
              TP.SubState = TP_IOC1;
            }
          }
          else                                      // If new current is greater than the old current...
          {                                         //   set the midpoint back to the old value, compute
            TestInj.MidPoint--;                     //   checksum for the cal constants, write cal constants
                                                    //   to FRAM, set flag to write the constants to Flash,
            if (TestInj.Channel < 4)                //   turn off test injection, and set state to output
            {                                       //   the new midpoint
              TestInjCal.midpoint_ph = TestInj.MidPoint;
            }
            else
            {
              TestInjCal.midpoint_gnd = TestInj.MidPoint;
            }
            TestInjCal.chk = ComputeChksum32((uint32_t *)(&TestInjCal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));
            TestInjCal.cmp = ~TestInjCal.chk;

            // Write test injection cal constants to FRAM
            FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
            FRAM_Write(DEV_FRAM2, FRAM_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&TestInjCal.midpoint_ph));
            FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
            
            SPI1Flash.Req |= S1F_INJ_WR;                  // Set flag to write constants to Flash
            TestInj.Flags |= TEST_INJ_INIT_OFF;           // Set flag to turn off test injection
            TP.SubState = TP_IOC3;
          }
        }
        else                                    // If not done accumulating readings, increment counter and
        {                                       //   remain in this state
          TP.NumChars++;
        }
      }
      break;

    case TP_IOC3:                       // Output the new midpoint value
      if ( (SPI1Flash.Ack & S1F_INJ_WR)             // If the FRAM and Flash writes are done and there are
        && (SystemFlags & VAL200MSEC) )             //   new readings (don't want to exit until all readings
      {                                             //   are back to normal), clear the request flag, output
        SPI1Flash.Req &= (uint32_t)(~S1F_INJ_WR);   //   the new midpoint value, and jump to the next state
        sprintf(&TP.TxValBuf[0], "% .4u", (unsigned int)(TestInj.MidPoint));
        TP.Status &= (~TP_TX_STRING);
        TP.Status |= TP_TX_VALUE;
        TP.TxValNdx = 0;
        TP.NumChars = 4;
        UART5->CR1 |= USART_CR1_TXEIE;              // Enable tx interrupts - this begins the transmissions
        TP.SubState++;
      }
      break;

    case TP_IOC4:                       // Wait For New 200msec Currents
      if (SystemFlags & VAL200MSEC)         // Wait once more for new 200msec currents.  This ensures all
      {                                     //   currents are back to 0 before enabling protection again
        Setpoints1.stp.ThermMem = TP.Temp;  // Restore thermal memory setpoint
        Manufacture_Mode = FALSE;
        TP.State = TP_CURSOR;
      }
      break;
        
    default:                            // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestInjOffsetCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestInjGainCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Sine Generator Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the Test Injection Sine Wave Generator.
//                      Calibration consists of the following steps:
//                          1) Enable the test injection circuit and set the amplitude for 100
//                          2) Accumulate 100 readings an compute the average
//                          3) Repeat the process with an amplitude of 1600
//                          4) Compute the slope and y-intercept
//                          5) Set the flags to write the results into FRAM and Flash
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//                      
//  CAVEATS:            The subroutine assumes that offset calibration has already been performed, that the
//                      TA is not connected, and that no external currents are being input to the unit
//                      TEST_INJ_ON must be the last flag set (set the other flags before this one to ensure
//                      there are no conflicts with the interrupt) when turning on test injection!!
//
//  INPUTS:             TP.SubState,  SPI1Flash.Ack
// 
//  OUTPUTS:            TP.State,  SPI1Flash.Req, TestInj.Flags, TestInj.Amplitude, TestInj.MidPoint,
//                      TestInj.CosIndx, TestInj.Channel, Manufacture_Mode
//
//  ALTERS:             TP.SubState, TP.NumChars
//
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), FRAM_Stat_Write(), FRAM_Write()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestInjGainCal(void)
{
  uint8_t j;

  switch (TP.SubState)
  {
    case TP_ISC0:                       // Read step and channel to be calibrated - sine
    case TP_IDC0:                       // Read step and channel to be calibrated - dc
      // Read the next parameter - channel to be calibrated.  This must be a decimal number:
      //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igsrc.    Save the channel number in temporary
      TP.State = TP_CURSOR;                 // Assume bad command
      if (TP.RxNdxOut != TP.RxNdxIn)        // Get the next char in the cmnd
      {
        j = TP_ParseChars();
        if (j == 1)                         // If valid, get the number
        {
          j = (uint8_t)(TP_GetDecNum());
        }
        else                                // If not decimal number, set to invalid code
        {
          j = 255;
        }
        // If channel is valid, continue with setup
        if (j < 5)
        {
          if (TP.SubState == TP_ISC0)           // If sine calibration...
          {                                       // Set amplitude to 100 and the midpoint to the zero point
            TestInj.Amplitude = 100;
            TestInj.MidPoint = ((j < 4) ? (TestInjCal.midpoint_ph) : (TestInjCal.midpoint_gnd));
          }
          else                                  // If dc calibration...
          {                                       // Set amplitude to 0 and the midpoint to the first cal
            TestInj.Amplitude = 0;                //   point (2550)
            TestInj.MidPoint = 2550;
          }
          // When sine wave is used for test injection, the dc gain is reduced by a factor of 3.  This
          //   reduces the error, but limits the max current to 65KA.  The dc gain is also reduced when dc
          //   is used for test injection and the current is less than 65kA.  Therefore, set the flag here
          //   to reduce the dc gain here also so the dc gain is also reduced when calibrating
          TestInj.Flags |= TEST_INJ_CHANGE_A1;
          TSTINJ_PHA_OFF;                       // Make sure all of the test channels are off to start with
          TSTINJ_PHB_OFF;
          TSTINJ_PHC_OFF;
          TSTINJ_PHN_OFF;
          TSTINJ_DIS;
          TestInj.CosIndx = 20;                 // Initialize cosine index (COS[n] = SIN[n+20])
          TestInj.Channel = j;                  // Save channel
          Manufacture_Mode = TRUE;              // Put into Manufacturing Mode so protection is disabled
          // Turn off thermal memory so the bucket is cleared when we are done calibrating
          TP.Temp = Setpoints1.stp.ThermMem;    // Save thermal memory setpoint
          Setpoints1.stp.ThermMem = 0;
          TestInj.Flags |= TEST_INJ_INIT_ON;    // Set flag to initialize test inj.  Note, this flag must
                                                //   be set before setting the flag to turn on test
                                                //   injection
          TestInj.Flags |= TEST_INJ_ON;         // Turn on test injection
          TP.NumChars = 0;                      // Initialize count.  NumChars is used as a temporary
          TP.Tmp1.f = 0;                        // Initialize sums of readings
          TP.Tmp2.f = 0;
          TP.ValPtr1 = &CurOneCyc.Ia + j;       // Initialize pointer to the current for the cal channel
          TP.State = TP_IGC;
          TP.SubState++;
        }
      }
      break;

    case TP_ISC1:                       // Wait for digital integrator to stabilize - sine
    case TP_IDC1:                       // Wait for digital integrator to stabilize - dc
    case TP_ISC3:
    case TP_IDC3:
      if (SystemFlags & VAL200MSEC)         // Wait 10 seconds for the integrator to stabilize - this is
      {                                     //   50 200msec anniversaries
        if (++TP.NumChars >= 50)            // Reinitialize counter and go to next state when wait period is
        {                                   //   over
          TP.NumChars = 0;
          TP.SubState++;
        }
      }
      break;                                // If no 200msec anniversary, just break

    case TP_ISC2:                       // Accumulate 100 readings - sine - amplitude = 100
    case TP_IDC2:                       // Accumulate 100 readings - dc - midpoint = 2550
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        TP.Tmp1.f += *TP.ValPtr1;
        if (TP.NumChars >= 99)              // If accumulated 100 readings, compute average, save in Tmp1.f,
        {                                   //   set up for the next output point, reinitialize the counter
          TP.Tmp1.f = TP.Tmp1.f/100.0;      //   and summation registers, and go on to the next state
          if (TP.SubState == TP_ISC2)           // If sine calibration...
          {
            TestInj.Amplitude = 1600;
          }
          else                                  // If dc calibration...
          {
            TestInj.MidPoint = 4000;
          }
          TP.NumChars = 0;
          TP.Tmp2.f = 0;
          TP.SubState++;
        }
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.NumChars++;
        }
      }
      break;

    case TP_ISC4:                       // Accumulate 100 readings - sine - amplitude = 1600
    case TP_IDC4:                       // Accumulate 100 readings - dc - midpoint = 4000
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        TP.Tmp2.f += *TP.ValPtr1;
        if (TP.NumChars >= 99)              // If accumulated 100 readings, compute average, save in Tmp2.f,
        {                                   //   and calculate the gain and offset values using the formulas
          TP.Tmp2.f = TP.Tmp2.f/100.0;      //     m = 1500/(Tmp2.f-Tmp1.f),  b = 100-(m*Tmp1.f) for sine
          if (TP.SubState == TP_ISC4)       //     m = 1450/(Tmp2.f-Tmp1.f),  b = 2550-(m*Tmp1.f) for dc
          {
            TestInjCal.m_sine[TestInj.Channel] = 1500.0/(TP.Tmp2.f - TP.Tmp1.f);
            TestInjCal.b_sine[TestInj.Channel] = 100.0 - (TestInjCal.m_sine[TestInj.Channel] * TP.Tmp1.f);
          }
          else
          {
            TestInjCal.m_dc[TestInj.Channel] = 1450.0/(TP.Tmp2.f - TP.Tmp1.f);
            TestInjCal.b_dc[TestInj.Channel] = 2550.0 - (TestInjCal.m_dc[TestInj.Channel] * TP.Tmp1.f);
          }
          TestInjCal.chk = ComputeChksum32((uint32_t *)(&TestInjCal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));
          TestInjCal.cmp = ~TestInjCal.chk;

          // Write test injection cal constants to FRAM
          FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
          FRAM_Write(DEV_FRAM2, FRAM_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&TestInjCal.midpoint_ph));
          FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection

          SPI1Flash.Req |= S1F_INJ_WR;          // Set flag to request write to Flash
          TestInj.Flags |= TEST_INJ_INIT_OFF;   // Set flag to turn off test injection
          TP.SubState++;                        // Set next state to wait for write completion
        }
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          TP.NumChars++;
        }
      }
      break;

    case TP_ISC5:                       // Wait For FRAM and Flash Writes to Complete
    case TP_IDC5:
      if ( (SPI1Flash.Ack & S1F_INJ_WR)             // If the FRAM and Flash writes are done and there are
        && (SystemFlags & VAL200MSEC) )             //   new readings (don't want to exit until all readings
      {                                             //   are back to normal), clear the request flag and
        SPI1Flash.Req &= (uint32_t)(~S1F_INJ_WR);   //   jump to the next state
        TP.SubState++;
      }
      break;

    case TP_ISC6:                       // Wait For New 200msec Currents
    case TP_IDC6:
      if (SystemFlags & VAL200MSEC)         // Wait once more for new 200msec currents.  This ensures all
      {                                     //   currents are back to 0 before enabling protection again
        Setpoints1.stp.ThermMem = TP.Temp;  // Restore thermal memory setpoint
        Manufacture_Mode = FALSE;
        TP.State = TP_CURSOR;
      }
      break;

    default:                            // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestInjGainCal()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_TestInj()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Port Test Injection
//
//  MECHANICS:          This subroutine injects an on-board test signal that was entered via the test port
//                      into one of the current channels.
//                      The test signal RMS amplitude is the equivalent of the primary current entered as
//                      part of the command string.  The subroutine performs the following steps:
//                          1) Read the channel on which the test signal will be injected, and the
//                             equivalent requested primary current
//                          2) Call the subroutine to perform test injection
//                      
//  CAVEATS:            This subroutine assumes test injection calibration has been performed
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[], Manufacture_Mode
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.RxNdxOut, TP.Tmp1.f
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), atof(), TestInjCur()
// 
//------------------------------------------------------------------------------------------------------------

void TP_TestInj(void)
{
  uint8_t i, j;
  uint8_t tempbuf[17];

  // Set the channel to an invalid code so test injection is turned off if there are no mode characters in
  //   the buffer
  j = 255;

  // Read the channel and the desired primary current
  if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the buffer...
  {
    // Read the next parameter - channel to be calibrated.  This must be decimal number:
    //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igsrc
    j = TP_ParseChars();
    if (j == 1)                             // If valid, get the number
    {
      j = (uint8_t)(TP_GetDecNum());
    }
    else                                    // If not decimal number, set to invalid code
    {
      j = 255;
    }
    // Read the next parameter - primary current
    // Move the characters comprising the current entered by the user to a separate buffer to ensure there
    //   is no wraparound.  The atof() function does not handle wraparound of the buffer.  Assume current
    //   isn't more than 16 chars (probably significantly less)
    i = 0;
    TP.Tmp1.f = 0;                          // Initialize value to 0 in case there are no characters
    while ((i < 16) && (TP.RxNdxOut != TP.RxNdxIn))
    {
      tempbuf[i] = TP.RxBuf[TP.RxNdxOut];
      ++i;
      TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    }
    tempbuf[i] = 0x00;                      // Make sure the string is terminated
    TP.Tmp1.f = atof((char const *)(&tempbuf[0]));
  }

  // Call subroutine to perform the test injection.  Note, if the input is invalid, test injection will be
  //   turned off
  if (Manufacture_Mode == FALSE)
  {
    TestInjCur(j, TP.Tmp1.f);              // for a Protection processor trip (LSIG)
  }
  else
  {
    TestInjCur_OvrMicro(j, TP.Tmp1.f);     // for an Override micro trip (MM, MCR, Override)
  }  
  
  //------------------ BP Test code -------------------------------
  HW_SecInjTest.Enable = TRUE;
  HW_SecInjTest.Phase = TEST_IA;       // BP Test code
  //---------------------------------------------------------------

  TP.State = TP_CURSOR;                 // Next state is cursor because we are done

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_TestInj()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TestInjCur()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection
//
//  MECHANICS:          This subroutine injects an on-board test signal into one of the current channels.
//                      The test signal RMS amplitude and the channel are input parameters.  The subroutine
//                      performs the following steps:
//                          1) Check the channel and testcurrent validity.  If valid, continue.  If invalid,
//                             quit.
//                          2) Compute the sine wave amplitude according to the sine wave injection
//                             conversion constants.
//                             If (amplitude + midpoint) is greater than 4095 (the max DAC code), use the dc
//                             signal for injection, and determine the code using the dc injection
//                             conversion constants.
//                             If the requested current is less than 65KA, set the flag to use the modified
//                             integrator constant.  This constant reduces the gain by a factor of 3, which
//                             reduces the error, but limits the max current to 65KA.
//                             If the requested current is greater than 65KA, do not set the flag.  Also,
//                             use the dc signal for injection.
//                          3) Turn on the test injection, and start the timer
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//
//                      This test signal goes through the AFE and digital integrator.  The trip is initiated
//                      by the Protection processor.
//
//                      This subroutine is called when a Hardware Sec Inj test is requested and we are not 
//                      in Manufacture Mode.
//                      
//  CAVEATS:            This subroutine assumes test injection calibration has been performed.
//                      TEST_INJ_ON must be the last flag set (set the other flags before this one to ensure
//                      there are no conflicts with the interrupt) when turning on test injection!!
//
//  INPUTS:             channel - the channel that the test current should be injected on
//                      testcurrent - the RMS amplitude of the injected current
//                      TestInjCal.midpoint_ph, TestInjCal.midpoint_gnd, TestInjCal.m_sine[],
//                      TestInjCal.b_sine[], TestInjCal.m_dc[], TestInjCal.b_dc[]
// 
//  OUTPUTS:            TestInj.Amplitude, TestInj.MidPoint, TestInj.CosIndx, TestInj.Flags, INT_a1[]
//                      Test injection port lines
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void TestInjCur(uint8_t channel, float testcurrent)
{
  float ftemp, ftemp1;

  // First, make sure all of the test channels are off and all of the integrator constants are normal.  This
  //   basically turns off test injection.  We need to do this if a valid test injection request is made,
  //   before switching to a new channel and/or value.  We also need to do this if an invalid test injection
  //   request is made, because in this case, we will turn off test injection.  Since it is done in both
  //   cases, do it here.
  TSTINJ_PHA_OFF;                         // Make sure all of the test channels are off
  TSTINJ_PHB_OFF;
  TSTINJ_PHC_OFF;
  TSTINJ_PHN_OFF;
  TSTINJ_DIS;
  INT_a1[0] = NORMAL_a1;                  // Make sure all integrator constants are normal
  INT_a1[1] = NORMAL_a1;
  INT_a1[2] = NORMAL_a1;
  INT_a1[3] = NORMAL_a1;

  // If current is nonzero and in range, and channel is valid, set up to output the test injection current
  if ((testcurrent >= 0) && (testcurrent <= 195E3) && (channel < 5))    // *** DAH   MAY NEED TO ADJUST THE 195E3 VALUE DOWNWARD TO ABOUT 148KA (SEE POWERPOINT)- NEED TO TEST WITH ROGOWSKI ATTACHED
  {
    // Save the channel.  It is needed in the interrupt when turning on test injection.
    TestInj.Channel = channel;
    // Set ftemp1 to the midpoint value
    ftemp1 = ((channel < 4) ? (TestInjCal.midpoint_ph) : (TestInjCal.midpoint_gnd));
    // Assume test injection is a sine wave.  Compute the amplitude and save it in ftemp
    ftemp = ((TestInjCal.m_sine[channel] * testcurrent) + TestInjCal.b_sine[channel]);
    if ( (testcurrent <= 65E3)              // If the desired current is not greater than 65KA and the
      && ((ftemp + ftemp1) <= 4095.0) )     //   amplitude is within the range of the DAC, it is ok to use a
    {                                       //   sine wave for the injected signal
      TestInj.Amplitude = ftemp;              // Save the amplitude and midpoint for the sine wave
      TestInj.MidPoint = ftemp1;
    }
    else                                    // Otherwise the amplitude is beyond DAC's range and/or the
    {                                       //   requested current is greater than 65KA - use dc signal
      TestInj.Amplitude = 0;                  // Amplitude is zero for dc signal
      // Compute the dc output and store it in MidPoint.  If the current is greater than 65KA, the
      //   integrator constant will not be changed, so reduce the dc output by a factor of 3 to compensate
      if ((Break_Config.config.BreakerFrame == MAGNUM_STD) || (Break_Config.config.BreakerFrame == MAGNUM_STD_DW))               
      {
        if (testcurrent <= 65E3)     // *** DAH   MAY NEED TO ADJUST DOWN TO 45KA (SEE POWERPOINT) - NEED TO TEST WITH ROGOWSKI COIL ATTACHED
        {
          ftemp = ((TestInjCal.m_dc[channel] * testcurrent) + TestInjCal.b_dc[channel] + 0.5);
        }
        else
        {
          ftemp = ((TestInjCal.m_dc[channel] * testcurrent)/3 + TestInjCal.b_dc[channel] + 0.5);
        }
      }
      else                                  // Magnum Narrow or Double-Narrow (change the scaling by .208/.166)
      {
        if (testcurrent <= 65E3)     // *** DAH   MAY NEED TO ADJUST DOWN TO 45KA (SEE POWERPOINT) - NEED TO TEST WITH ROGOWSKI COIL ATTACHED
        {
          ftemp = ((TestInjCal.m_dc[channel] * testcurrent * 1.253) + TestInjCal.b_dc[channel] + 0.5);
        }
        else
        {
          ftemp = ((TestInjCal.m_dc[channel] * testcurrent) *1.253 /3 + TestInjCal.b_dc[channel] + 0.5);
        }
      }        
      
      
      if (ftemp > 4095.0)                     // Clamp the midpoint to 4095 if it exceeds it
      {
        TestInj.MidPoint = 4095.0;
      }
      else
      {
        TestInj.MidPoint = ftemp;
      }
    }
    TestInj.CosIndx = 20;                       // Initialize cosine index (COS[n] = SIN[n+20])
    if (testcurrent <= 65E3)                    // If the requested current is less than 65KA, set flag to
    {                                           //   change the integrator constant.  This reduces the dc
      TestInj.Flags |= TEST_INJ_CHANGE_A1;      //   gain by a factor of 3.
    }
    else                                        // Otherwise clear the flag in case it was already set
    {
      TestInj.Flags &= (~TEST_INJ_CHANGE_A1);   // Note, Set the status of this flag before setting the flag
    }                                           //   to initialize and the flag to turn on injection
    TestInj.Flags |= TEST_INJ_INIT_ON;          // Set flag to initialize test inj.  Note, set this flag
                                                //   before setting the flag to turn on test injection
    TestInj.Flags |= TEST_INJ_ON;               // Turn on test injection

    // *** DAH  NEED TO ADD CODE TO START THE TIMER TO MEASURE BEFORE TRIPPING!!!
  }
  // If no input or invalid input, turn off test injection only if it was on.  Don't set the flag to turn it
  //   off if test injection is already off, because it will turn off test injection the next time it is
  //   turned on.
  else if (TestInj.Flags & TEST_INJ_ON)
  {
    TestInj.Flags |= TEST_INJ_INIT_OFF;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TestInjCur()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TestInjCur_OvrMicro()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection for Override Micro trips (Maintenance Mode, MCR, Override)
//
//  MECHANICS:          This subroutine injects an on-board test signal into one of the current channels.
//                      The test signal RMS amplitude and the channel are input parameters.  The subroutine
//                      performs the following steps:
//                          1) Check the channel and testcurrent validity.  If valid, continue.  If invalid,
//                             quit.
//                          2) Compute the sine wave amplitude according to the sine wave injection
//                             conversion constants.
//                             If (amplitude + midpoint) is greater than 4095 (the max DAC code), use the dc
//                             signal for injection, and determine the code using the dc injection
//                             conversion constants.
//                             If the requested current is greater than xxKA, use the dc signal for injection.
//                          3) Turn on the test injection.
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//
//                      This test signal goes through the analog integrator.  The trip is initiated by the 
//                      Override Micro when 3 consecutive samples are above the threshold.
//
//                      This subroutine is called when a Hardware Sec Inj test is requested and we are in 
//                      Manufacture Mode.  This also prevents LSIG trips when the test signal is ramping up.
//                      
//  CAVEATS:            This subroutine assumes test injection calibration has been performed.
//                      TEST_INJ_ON must be the last flag set (set the other flags before this one to ensure
//                      there are no conflicts with the interrupt) when turning on test injection!!
//
//  INPUTS:             channel - the channel that the test current should be injected on.  
//                                (The Override micro only protects on A,B,C phases)
//                      testcurrent - the RMS amplitude of the injected current
//                      TestInjCal.midpoint_ph, TestInjCal.midpoint_gnd, TestInjCal.m_sine[],
//                      TestInjCal.b_sine[], TestInjCal.m_dc[], TestInjCal.b_dc[]
// 
//  OUTPUTS:            TestInj.Amplitude, TestInj.MidPoint, TestInj.CosIndx, TestInj.Flags, INT_a1[]
//                      Test injection port lines
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void TestInjCur_OvrMicro(uint8_t channel, float testcurrent)
{
  float ftemp, ftemp1;

  // First, make sure all of the test channels are off and all of the integrator constants are normal.  This
  //   basically turns off test injection.  We need to do this if a valid test injection request is made,
  //   before switching to a new channel and/or value.  We also need to do this if an invalid test injection
  //   request is made, because in this case, we will turn off test injection.  Since it is done in both
  //   cases, do it here.
  TSTINJ_PHA_OFF;                         // Make sure all of the test channels are off
  TSTINJ_PHB_OFF;
  TSTINJ_PHC_OFF;
  TSTINJ_PHN_OFF;
  TSTINJ_DIS;

  // If current is nonzero and in range, and channel is valid, set up to output the test injection current
  if ((testcurrent >= 0) && (testcurrent <= 195E3) && (channel < 4))    // *** DAH   MAY NEED TO ADJUST THE 195E3 VALUE DOWNWARD TO ABOUT 148KA (SEE POWERPOINT)- NEED TO TEST WITH ROGOWSKI ATTACHED
  {
    // Save the channel.  It is needed in the interrupt when turning on test injection.
    TestInj.Channel = channel;
    // Set ftemp1 to the midpoint value
    ftemp1 = ((channel < 4) ? (TestInjCal.midpoint_ph) : (TestInjCal.midpoint_gnd));
    // Assume test injection is a sine wave.  Compute the amplitude and save it in ftemp
    ftemp = ((TestInjCal.m_sine[channel] * testcurrent) + TestInjCal.b_sine[channel]);
  

    if ( (testcurrent <= 65E3)              // If the desired current is not greater than 65KA and the
      && ((ftemp + ftemp1) <= 4095.0) )     //   amplitude is within the range of the DAC, it is ok to use a
    {                                       //   sine wave for the injected signal
      TestInj.Amplitude = ftemp;            // Save the amplitude and midpoint for the sine wave
      TestInj.MidPoint = ftemp1;
    }
    else                                    // Otherwise the amplitude is beyond DAC's range and/or the
    {                                       //   requested current is greater than 65KA - use dc signal
      TestInj.Amplitude = 0;                // Amplitude is zero for dc signal
      
      // Compute the dc output and store it in MidPoint       
      if ((Break_Config.config.BreakerFrame == MAGNUM_STD) || (Break_Config.config.BreakerFrame == MAGNUM_STD_DW))               
      {                      
        ftemp = ((TestInjCal.m_dc[channel] * testcurrent * 0.73) + TestInjCal.b_dc[channel] + 0.5);    
      }
      else                                                              // Magnum Narrow or Double-Narrow
      {
        ftemp = ((TestInjCal.m_dc[channel] * testcurrent * 0.915) + TestInjCal.b_dc[channel] + 0.5);      
      }
      
      if (ftemp > 4095.0)                     // Clamp the midpoint to 4095 if it exceeds it
      {
        TestInj.MidPoint = 4095.0;
      }
      else
      {
        TestInj.MidPoint = ftemp;
      }
    }
                                             
    TestInj.CosIndx = 20;                       // Initialize cosine index (COS[n] = SIN[n+20])
       
    TestInj.Flags |= TEST_INJ_INIT_ON;          // Set flag to initialize test inj.  Note, set this flag
                                                //   before setting the flag to turn on test injection
    TestInj.Flags |= TEST_INJ_ON;               // Turn on test injection
  }
  
  // If no input or invalid input, turn off test injection only if it was on.  Don't set the flag to turn it
  //   off if test injection is already off, because it will turn off test injection the next time it is
  //   turned on.
  else if (TestInj.Flags & TEST_INJ_ON)
  {
    TestInj.Flags |= TEST_INJ_INIT_OFF;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TestInjCur_OvrMicro()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_EEPOT()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Modify EEPOT Settings
//
//  MECHANICS:          This subroutine handles displaying and changing the Frame Module EEPOT settings.
//                      The Microchip Technology MCP4261 chip contains two potentiometers.  Command code "O"
//                      maps to potentiometer 0 in the EEPOT.  Command code "M" maps to potentiometer 1 in
//                      the EEPOT.
//                      The subroutine reads and displays the existing value for the referenced EEPOT.
//                      Note, the nonvolatile register is read.  It then accepts a new input (if desired)
//                      and writes the new input to both the volatile and nonvolatile registers. 
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState, FrameEEPOT.WrVal, FrameEEPOT.RdVal
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf(), EEPOT_Write(), EEPOT_Xfer()
// 
//------------------------------------------------------------------------------------------------------------

void TP_EEPOT(void)
{
  uint8_t i;

  switch (TP.SubState)
  {
    case TP_EE0:                            // Set up
      if (TP.RxNdxOut != TP.RxNdxIn)                // Get the next char in the cmnd - this defines the
      {                                             //   EEPOT: override ("O") or Maintenance Mode ("M")
        i = TP.RxBuf[TP.RxNdxOut] & 0xDF;           // Convert char to upper-case
        TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
        if ( (i == 'O') || (i == 'M') )             // If valid EEPOT setting, decode it
        {
          if (i == 'O')            
          {
            FrameEEPOT.WrVal = EEPOT_RD_W0;                     // Save read command
            TP.Temp = 0;                                        // Save pot address
          }
          else
          {
            FrameEEPOT.WrVal = EEPOT_RD_W1;
            TP.Temp = 1;
          }
            FrameEEPOT.RdVal = EEPOT_Xfer(FrameEEPOT.WrVal);    // Read the EEPOT
            TP.SubState = TP_EE1;
        }
        else                                        // If invalid EEPOT setting, quit
        {
          TP.State = TP_CURSOR;
        }
      }
      break;

    case TP_EE1:                            // Output existing value
      // Convert the EEPOT setting to ASCII and transmit it
      sprintf(&TP.TxValBuf[0], "% .3u", (unsigned int)(FrameEEPOT.RdVal & 0x01FF));
      TP.TxValBuf[3] = ' ';
      TP.Status &= (~TP_TX_STRING);
      TP.Status |= TP_TX_VALUE;
      TP.TxValNdx = 0;
      TP.NumChars = 4;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable tx interrupts - this begins the transmissions
      // Reinitialize the output buffer indices so that any inputted  value does not roll over past the end
      //   of TP.RxBuf[].  This will mess up the value returned from atof(), since it doesn't realize the
      //   buffer rolls over.  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
      TP.RxNdxOut = 0;
      TP.RxNdxIn = 0;
      TP.SubState = TP_EE2;
      break;

    case TP_EE2:                            // Process the new EEPOT value
      i = TP.RxNdxOut;
      while ( (i != TP.RxNdxIn)                     // Check the Rx buffer for a line feed - this indicates
           && (TP.State == TP_EE) )                 //   that a that a new value has been entered
      {
        if (TP.RxBuf[i] == 0x0A)                    // If LF received, check whether it is a new value
        {
          TP.State = TP_CURSOR;                         // Assume don't need to process an input
          if ( (TP.RxBuf[TP.RxNdxOut] != 0x0A)          // If first char is not a line feed or carriage
            && (TP.RxBuf[TP.RxNdxOut] != 0x0D) )        //   return, process the input and set up to write
          {                                             //   the new value to the EEPOT
//            FrameEEPOT.WrVal = ( atoi((char const *)(&TP.RxBuf[TP.RxNdxOut])) ) & 0x01FF;   Much less efficient
            FrameEEPOT.WrVal = asciidecimal_hex(i);
            if (FrameEEPOT.WrVal < 258)             // If valid value was entered (257 steps for the
            {                                       //   potentiometer), set up to write it
              FrameEEPOT.WrVal |= ((TP.Temp == 0) ? EEPOT_NVW0_ADD : EEPOT_NVW1_ADD);
              TP.SubState = TP_EE3;                 // Set substate to wait for write to complete
              TP.State = TP_EE;                     // Reset the main state so this subroutine is called
            }                                       // Otherwise just exit
          }
        }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
        i = (i + 1) & 0x1F;                         // Increment the index
      }
      break;

    case TP_EE3:                            // Write the new EEPOT value to the EEPOT
      // Write value to the Frame EEPOT.  The value and nonvolatile address is held in FrameEEPOT.WrVal.
      //   The function returns True when done
      if (EEPOT_Write())                        // If write is done, jump to the cursor state
      {
        TP.State = TP_CURSOR;
      }
      break;                                    // If write is not done, remain in this state

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_EEPOT()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_AFESetPGA()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Set AFE PGA Gain
//
//  MECHANICS:          This subroutine handles the AG command, which is used to set the gain of the AFE.
//                      The subroutine, resets the AFE, reinitializes the AFE with the new gain setting,
//                      restarts the AFE, then adjusts the gain calibration coefficients according to the
//                      new PGA setting.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn, TP.RxBuf[]
// 
//  OUTPUTS:            None
//
//  ALTERS:             TP.RxNdxOut
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), AFE_Init(), AFE_SPI_Xfer()
// 
//------------------------------------------------------------------------------------------------------------

void TP_AFESetPGA(void)
{
  uint8_t new_setting, AFEval, k;
  uint16_t j;

  new_setting = 255;
  if (TP_ParseChars() == 1)     // Read next parameter - must be dec number (return code equals 1)
  {
    new_setting = (uint8_t)(TP_GetDecNum());
  }
  if (new_setting == 1)         // Set new_setting to PGA value to write to AFE, based on the setting
  {                             //   entered by the user.  If number is not a valid setting, just return
    AFEval = 0x00;
  }
  else if (new_setting == 2)
  {
    AFEval = 0x40;
  }
  else if (new_setting == 4)
  {
    AFEval = 0x80;
  }
  else if (new_setting == 8)
  {
    AFEval = 0xC0;
  }
  else
  {
    new_setting = 255;
  }

  if (new_setting != 255)               // If valid input
  {
    for (k=0; k<4; k++)                     // Adjust the AFE gain cal constant for the new PGA setting
    {
      AFEcal.gain[k] = AFEcal.gain[k] * (float)TP_AFEPGASetting;
      AFEcal.gain[k] = AFEcal.gain[k]/(float)new_setting;
    }
    TP_AFEPGASetting = new_setting;         // Save the new setting

    AFE_RESETN_ACTIVE;                      // Reset the AFE chip
    j = 5;                                  // Delay at least 100nsec.  Measured on 150820: 1.33usec
    while (j > 0)                           //   Note, this is running with the 16MHz clock.
    {                                       // Assume delay is at least 133nsec at 120MHz
      --j;
    }
    AFE_RESETN_INACTIVE;

    // Contacted ADI on 150825 after seeing errors in the setup when not delaying between the
    //   reset going inactive and calling the routine to initialize the chip.  According to ADI,
    //   must wait at least 120usec after reset is brought inactive before writing to the chip.
    j = 725 * 10;                           // Measured time on 150825: 137usec
    while (j > 0)                           //   Note, this is running with the 16MHz clock (j=725)
    {                                       //   At 120MHz, multiply by 10.
      --j;
    }

    AFE_Init();                             // Next, set up the registers in the chip     
    j = AFE_SPI_Xfer(0x0000 | AFEval);      // Rewrite the PGA register for channels 0 - 3 (Ia thru In)
    j = AFE_SPI_Xfer(0x0100 | AFEval);
    j = AFE_SPI_Xfer(0x0200 | AFEval);
    j = AFE_SPI_Xfer(0x0300 | AFEval);

    AFE_START_LOW;                          // Restart the chip
    j = 5 * 4;                              // Delay at least 250nsec.  Measured on 150820: 1.33usec
    while (j > 0)                           //   Note, this is running with the 16MHz clock (j=5).
    {                                       // Assume delay is at least 266nsec at 120MHz if
      --j;                                  //   multiplied by 2.  Multiply by 4 to play it safe.
    }
    AFE_START_HIGH;
  }
      
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_AFESetPGA()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_ChangeCAMtype()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Change CAM Type
//
//  MECHANICS:          This subroutine reads the command string, displays the present CAM style for the
//                      referenced CAM, and changes the style if a valid new style is entered.
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.RxNdxOut, TP.RxNdxIn
// 
//  OUTPUTS:            CAMx.Status
//
//  ALTERS:             TP.Temp, TP.ValPtr3
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_ChangeCAMtype(void)
{
  uint8_t i, CAMtype;


  switch (TP.SubState)
  {
    case TP_CC0:                            // Read CAM port
      // Save CAM port in i
      i = 255;                              // Assume bad CAM number
      if (TP.RxNdxOut != TP.RxNdxIn)            // Get the next character in the command, if there is one
      {
        i = TP_ParseChars();
        if (i == 1)                                 // If valid, get the number
        {
          i = (uint8_t)(TP_GetDecNum());
        }
      }
      // Initialize pointer to CAM.Status according to the CAM number.  If invalid CAM number, quit
      TP.SubState = TP_CC1;                     // Assume valid CAM number
      if (i == 1)                               // If CAM1 ...
      {
        TP.Temp = 1;
        TP.ValPtr3 = &CAM1.Status;
      }
      else if (i ==  2)                         // If CAM2 ...
      {
        TP.Temp = 2;
        TP.ValPtr3 = &CAM2.Status;
      }
      else                                      // If invalid ...
      {
        TP.State = TP_CURSOR;
      }
      break;

    case TP_CC1:                            // Output existing CAM style
      TP.TxValBuf[0] = ((*TP.ValPtr3 & CAM_TYPE_SAMPLE) ? 'S' : 'G');
      TP.TxValBuf[1] = ' ';
      TP.Status &= (~TP_TX_STRING);
      TP.Status |= TP_TX_VALUE;
      TP.TxValNdx = 0;
      TP.NumChars = 2;
      UART5->CR1 |= USART_CR1_TXEIE;              // Enable transmit interrupts - this begins the transmissions
      // Reinitialize the output buffer indices so that any inputted  value does not roll over past the
      //   end of TP.RxBuf[].  Also set RxNdxOut equal to RxNdxIn to ensure a new start to parsing.
      TP.RxNdxOut = 0;
      TP.RxNdxIn = 0;
      TP.SubState = TP_CC2;
      break;

    case TP_CC2:                            // Read new CAM style
      i = TP.RxNdxOut;
      while ( (i != TP.RxNdxIn)                     // Check the Rx buffer for a line feed - this indicates
           && (TP.State == TP_CC) )                 //   that a that a new value has been entered
      {
        if (TP.RxBuf[i] == 0x0A)                    // If LF received, check whether it is a new value
        {
          CAMtype = TP.RxBuf[TP.RxNdxOut] & 0xDF;       // Convert received val to upper-case
          if (CAMtype == 'S')                           // If changing CAM to sampled-value type...
          {
            if (TP.Temp == 1)                           // Abort any existing streams to ensure transfer
            {                                           //   complete flag is clear
              Init_CAM1_DMA_Streams();
            }
            else
            {
              Init_CAM2_DMA_Streams();
            }
            *TP.ValPtr3  |= CAM_TYPE_SAMPLE;
          }
          else if (CAMtype == 'G')                      // If changing CAM to GOOSE type...
          {
            *TP.ValPtr3 &= (~CAM_TYPE_SAMPLE);          // First set flag so no more DMA operations will be
                                                        //   initiated in the sampling interrupt
            if (TP.Temp == 1)                           // Abort any existing streams to ensure transfer
            {                                           //   complete flag is clear
              Init_CAM1_DMA_Streams();
            }
            else
            {
              Init_CAM2_DMA_Streams();
            }
          }
          TP.State = TP_CURSOR;
        }
//          TP_IdleTimer = TP_IDLETIMEOUT;          // Reset the idle timer - we have received char(s)
        i = (i + 1) & 0x1F;                         // Increment the index
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_ChangeCAMtype()
//------------------------------------------------------------------------------------------------------------

#define TPFLOATSIZE       12
#define TPUINT32SIZE      8
#define TPUINT16SIZE      8
#define TPUINDEXSIZE      6

#define SPIUINT32SIZE     4
#define SPIUINT16SIZE     2


void TP_ParseUInt16(uint8_t TPindex, uint16_t SPIindex)
{
  uint32_t temp;
  temp = (SPI2_buf[SPIindex] + (((uint32_t)SPI2_buf[SPIindex+1]) << 8));
  sprintf(&TP.TxValBuf[TPindex], "%8u", temp);
}


void TP_ParseUInt32(uint8_t TPindex, uint16_t SPIindex)
{
  uint32_t temp;
  temp = (SPI2_buf[SPIindex] + (((uint32_t)SPI2_buf[SPIindex+1]) << 8) + (((uint32_t)SPI2_buf[SPIindex+2]) << 16) + (((uint32_t)SPI2_buf[SPIindex+3]) << 24) );
  sprintf(&TP.TxValBuf[TPindex], "%8u", temp);
}

void TP_ParseFloat(uint8_t TPindex, uint16_t SPIindex)
{
  float temp_f;
  memcpy(&temp_f, &SPI2_buf[SPIindex], sizeof(temp_f));
  sprintf(&TP.TxValBuf[TPindex], "% .5E", temp_f);
}


uint16_t TP_PutSpace(uint8_t Num, uint16_t TP_Index)
{
  uint8_t i;

  for (i = 0; i < Num; i++)
  {
    TP.TxValBuf[TP_Index++]= ' ';
  }

  return TP_Index;
}


void TP_ParseEventsTable(uint32_t index)
{

  uint16_t TP_Index;
  uint16_t SPI_Index;

  TP_Index = 0;
  SPI_Index = 0;

  // index
  sprintf(&TP.TxValBuf[TP_Index], "%5u", (index));
  TP_Index = TP_PutSpace(3, TP_Index += TPUINDEXSIZE);

  // EID
  TP_ParseUInt32(TP_Index++,SPI_Index);
  TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
  
  // TS sec
  TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
  TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
  
  // TS nanosec
  TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
  TP_Index = TP_PutSpace(2, TP_Index += TPUINT32SIZE);

  // Ia
  TP_ParseFloat(TP_Index++,SPI_Index += SPIUINT32SIZE);
  TP_Index = TP_PutSpace(2, TP_Index += TPFLOATSIZE);

  // Ib
  TP_ParseFloat(TP_Index++,SPI_Index += SPIUINT32SIZE);
  TP_Index = TP_PutSpace(11, TP_Index += TPFLOATSIZE);


  TP.TxValBuf[TP_Index++] = '\n';
  TP.TxValBuf[TP_Index++] = '\r';

  TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
  TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
  TP.TxValNdx = 0;
  TP.NumChars = TP_Index++;
  UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts

}



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayMemory()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Memory 
//
//  MECHANICS:          This subroutine displays the contents of the microprocessor's RAM memory as either
//                      a uint8, uint16, or a uint32.  The user enters the following parameters:
//                          1) output format (stored in temporary "i")
//                               0 - uint8
//                               1 - uint16
//                               2 - uint32
//                          2) starting address (stored in temporary "Addr" and TP.ValPtr5)
//                               Must be a hexadecimal number and a valid RAM address.  Certain areas of RAM
//                               are not accessable and the micro will hang if a read is attempted.  If an
//                               address is in one of these inaccessable regions, the subroutine merely
//                               returns
//                          3) number of lines of values (stored in TP.Temp)
//                              This is the number of lines of values that will be outputted.
//                              For uint8 format, there are 16 values per line
//                              For uint16 format, there are 8 values per line
//                              For uint32 format, there are 4 values per line
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[],TP.Status, TP.TxValNdx, TP.NumChars, UART5->CR1
//
//  ALTERS:             TP.ValPtr5, TP.SubState, TP.Temp 
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------
void TP_DisplayMemory(void) 
{
  uint8_t i, j, k, ind;
  uint32_t Addr;  
  
  Addr = (uint32_t)TP.ValPtr5;          // Initialize Addr to the present memory address to be displayed   

  switch (TP.SubState)
  {
    case TP_DM0:                        // Read in parameters
      i = TP_ParseChars();                      // Read next parameter - output format
      if (i == 1)                               // If parameter is a decimal number, retrieve it
      {
        i = (uint8_t)(TP_GetDecNum());
      }
      else                                      // If parameter is not a decimal number, set i to 0xFF
      {
        i = 0xFF;
      }
      if (i != 0xFF)                            // If command line is valid so far, read the next
      {                                         //   parameter - memory address
        j = TP_ParseChars();                    // If parameter is a hexadecimal number, retrieve it
        if (j == 2)
        {
          Addr = TP_GetHexNum(); 
        }
        else                                    // If parameter is not a decimal number, set i to 0xFF
        {
          i = 0xFF;
          break;
        }
      }  
      if (i != 0xFF)                            // If command line is valid so far, read the next
      {                                         //   parameter - number of lines to display
        k = TP_ParseChars();                    // If parameter is a decimal number, retrieve it
        if (k == 1)
        {
          TP.Temp = TP_GetDecNum(); 
        }
        else                                    // If parameter is not a decimal number, set i to the
        {                                       //   default value
          TP.Temp = 1; 
        }
      }  

      switch (i)                                // Set next state and initialize pointer according to the
      {                                         //   requested format
        case 0:                                     // Format 0 - uint8 
          TP.ValPtr5 = (uint8_t *)Addr;  
          TP.SubState = TP_DM1_0; 
          break;  
       
        case 1:                                     // Format 1 - uint16 
          TP.ValPtr5 = (uint16_t *)Addr;  
          TP.SubState = TP_DM1_1;  
          break;  
 
        case 2:                                     // Format 2 - uint32 
          TP.ValPtr5 = (uint32_t *)Addr;  
          TP.SubState = TP_DM1_2; 
          break; 

        case 3:                                     // Format 3 - float 
          TP.ValPtr5 = (float *)Addr;
          TP.SubState = TP_DM1_3;
          break;

        default:                                    // Invalid command string - quit
          TP.State = TP_CURSOR;  
          break;    
      } 
      break;

    case TP_DM1_0:                      // Output the address
    case TP_DM1_1:
    case TP_DM1_2:
    case TP_DM1_3:
      TP.TxValBuf[0] = '0';
      TP.TxValBuf[1] = 'x';
      sprintf(&TP.TxValBuf[2], "% 08X", Addr);
      TP.TxValBuf[11] = ':';
      TP.TxValBuf[12] = ' ';
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = 13;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts 
      TP.SubState += 4;                             // Increment the state
      break;
      
    case TP_DM2_0:                      // Wait until done transmitting
    case TP_DM2_1:
    case TP_DM2_2:
    case TP_DM2_3:
      if (!(TP.Status & TP_TX_VALUE))               // Increment the state when done transmitting
      {
        TP.SubState += 4;
      }
      break;  
      
    case TP_DM3_0:                      // Output values as uint8
      TP.TxValBuf[0] = ' ';
      ind = 1;
      for (i=0; i<16; i++)                  // 16 values per line 
      {
        // If address is in reserved area, quit.  Cannot read these areas, causes hard fault error interrupt
        if ( (Addr >= 0x00100000) && (Addr <= 0x07FFFFFF)
          || (Addr >= 0x08100000) && (Addr <= 0x0FFFFFFF)
          || (Addr >= 0x10010000) && (Addr <= 0x1FFEFFFF)
          || (Addr >= 0x1FFF7A10) && (Addr <= 0x1FFFBFFF)
          || (Addr >= 0x1FFFC008) && (Addr <= 0x1FFFFFFF)
          || (Addr >= 0x20020000) )
        {
          TP.Temp = 1;
          break;
        }
        sprintf(&TP.TxValBuf[ind], "% 02X", *(uint8_t *)TP.ValPtr5);
        ind += 2;
        TP.TxValBuf[ind++] = ' ';  
        Addr = (uint32_t)TP.ValPtr5;   
        Addr = Addr + 1;
        TP.ValPtr5 = (uint8_t *)Addr;
      }  
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValBuf[ind++] = '\n';
      TP.TxValBuf[ind++] = '\r';
      TP.TxValNdx = 0;
      TP.NumChars = ind;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState += 4;                             // Increment the state
      break;  

    
    case TP_DM3_1:                      // Output values as uint16
      TP.TxValBuf[0] = ' ';
      ind = 1;
      for (i=0; i<8; i++)                  // 8 values per line 
      {
        // If address is in reserved area, quit.  Cannot read these areas, causes hard fault error interrupt
        if ( (Addr >= (0x00100000 - 1)) && (Addr <= (0x07FFFFFF - 1))
          || (Addr >= (0x08100000 - 1)) && (Addr <= (0x0FFFFFFF - 1))
          || (Addr >= (0x10010000 - 1)) && (Addr <= (0x1FFEFFFF - 1))
          || (Addr >= (0x1FFF7A10 - 1)) && (Addr <= (0x1FFFBFFF - 1))
          || (Addr >= (0x1FFFC008 - 1)) && (Addr <= (0x1FFFFFFF - 1))
          || (Addr >= (0x20020000 - 1)) )
        {
          TP.Temp = 1;
          break;
        }
        sprintf(&TP.TxValBuf[ind], "% 04X", *(uint16_t *)TP.ValPtr5);
        ind += 4; 
        TP.TxValBuf[ind++] = ' '; 
        Addr = (uint32_t)TP.ValPtr5;          
        Addr = Addr + 2; 
        TP.ValPtr5 = (uint16_t *)Addr;
      }  
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValBuf[ind++] = '\n';
      TP.TxValBuf[ind++] = '\r';
      TP.TxValNdx = 0;
      TP.NumChars = ind;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState += 4;                             // Increment the state
      break;
    
    case TP_DM3_2:                      // Output values as uint32
      TP.TxValBuf[0] = ' ';
      ind = 1;
      for (i=0; i<4; i++)                  // 4 chars per line 
      {
        // If address is in reserved area, quit.  Cannot read these areas, causes hard fault error interrupt
        if ( (Addr >= (0x00100000 - 3)) && (Addr <= (0x07FFFFFF - 3))
          || (Addr >= (0x08100000 - 3)) && (Addr <= (0x0FFFFFFF - 3))
          || (Addr >= (0x10010000 - 3)) && (Addr <= (0x1FFEFFFF - 3))
          || (Addr >= (0x1FFF7A10 - 3)) && (Addr <= (0x1FFFBFFF - 3))
          || (Addr >= (0x1FFFC008 - 3)) && (Addr <= (0x1FFFFFFF - 3))
          || (Addr >= (0x20020000 - 3)) ) 
        {
          TP.Temp = 1;
          break;
        }
        sprintf(&TP.TxValBuf[ind], "% 08X", *(uint32_t *)TP.ValPtr5);
        ind = ind + 9; 
        TP.TxValBuf[ind++] = ' ';
        Addr = (uint32_t)TP.ValPtr5;  
        Addr = Addr + 4; 
        TP.ValPtr5 = (uint32_t *)Addr;
      }  
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValBuf[ind++] = '\n';
      TP.TxValBuf[ind++] = '\r';
      TP.TxValNdx = 0;
      TP.NumChars = ind;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts 
      TP.SubState += 4;                             // Increment the state
      break; 
      
    case TP_DM3_3:                      // Output values as floats
      TP.TxValBuf[0] = ' ';
      ind = 1;
      for (i=0; i<4; i++)                  // 4 chars per line 
      {
        // If address is in reserved area, quit.  Cannot read these areas, causes hard fault error interrupt
        if ( (Addr >= (0x00100000 - 3)) && (Addr <= (0x07FFFFFF - 3))
          || (Addr >= (0x08100000 - 3)) && (Addr <= (0x0FFFFFFF - 3))
          || (Addr >= (0x10010000 - 3)) && (Addr <= (0x1FFEFFFF - 3))
          || (Addr >= (0x1FFF7A10 - 3)) && (Addr <= (0x1FFFBFFF - 3))
          || (Addr >= (0x1FFFC008 - 3)) && (Addr <= (0x1FFFFFFF - 3))
          || (Addr >= (0x20020000 - 3)) ) 
        {
          TP.Temp = 1;
          break;
        }
        if ((*(float *)TP.ValPtr5) >= 0.0E0)        // Add extra space if positive or 0 so numbers are in
        {                                           //   line with negative values
          TP.TxValBuf[ind++] = ' ';
        }
        sprintf(&TP.TxValBuf[ind], "%.5E", *(float *)TP.ValPtr5);
        ind = ind + 12; 
        TP.TxValBuf[ind++] = ' ';
        Addr = (uint32_t)TP.ValPtr5;  
        Addr = Addr + 4; 
        TP.ValPtr5 = (float *)Addr;
      }  
      TP.Status &= (~TP_TX_STRING);                 // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;                     // Set flag to transmit values
      TP.TxValBuf[ind++] = '\n';
      TP.TxValBuf[ind++] = '\r';
      TP.TxValNdx = 0;
      TP.NumChars = ind;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts 
      TP.SubState += 4;                             // Increment the state
      break;
      
    case TP_DM4_0:                      // Wait until done transmitting
    case TP_DM4_1:
    case TP_DM4_2:
    case TP_DM4_3:
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        if (--TP.Temp == 0)                             // If no more lines to do, done
        {
          TP.State = TP_CURSOR;
        }
        else
        {                                               // Otherwise jump back 3 states to output the next
          TP.SubState -= (3 * 4);                       //   line, beginning with the address.
        }                                               //   3 - states, 4 - substates/state
      }
      break;  
      
    default: 
      TP.State = TP_CURSOR; 
      break;
      
  }                                            
}   
   
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayMemory()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayDmnd()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Demand and Energy Logs
//
//  MECHANICS:          This subroutine handles displaying the demand and energy logs
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayDmnd(void)
{
  uint32_t indx;
  uint8_t i, j;

  switch (TP.SubState)
  {
    case TP_DD0:                        // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                    // Read next parameter - value to retrieve
        if (j == 1)                                 // Must be a decimal number
        {
          i = (uint8_t)(TP_GetDecNum());
        }
        else
        {
          i = 255;                                  // If not a decimal number, set to invalid
        }
        if (i > 11)                                 // If invalid or out of range, set to display Ia demand
        {
          i = 0;
        }
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to invalid
        {
          indx = 65535;
        }
        if (indx > 12959)                           // If invalid or out of range, set to 0
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        i = 0;
        indx = 0;
      }

      // Initialize the pointer to the selected value and indicator to read demand
      // TP.Temp is used for two purposes.  In this subroutine, it indicates whether to use TP.ValPtr1 or
      //   TP.ValPtr4 as the value to display.  In the SPI1 Flash manager (SPI1_Flash_Manager()), it
      //   indicates whether to read the demand log or the energy log information.
      //        TP.Temp = 37: Use TP.ValPtr1 and read demand log
      //        TP.Temp = 38: Use TP.ValPtr4 and read energy log
      if (i < 7)
      {
        TP.ValPtr1 = &tbuf.d.DmndIa + i;
        TP.Temp = 37;
      }
      else
      {
        TP.ValPtr4 = &tbuf.d.TotFwdWHr + (i - 7);
        TP.Temp = 38;
      }
      // Save the index in TP.ValPtr5 (use this as a temporary)
      TP.ValPtr5 = (uint8_t *)indx;

      // Initialize the line count (use Tmp1.u as the count)
      TP.Tmp1.u = 0;

      TP.SubState = TP_DD1;                 // Fall into the next state
   // break;

    case TP_DD1:                        // Request Flash Read
      SPI1Flash.Req |= S1F_TP_RD_REQ;       // Set request flag from the test port
      TP.SubState = TP_DD2;
      break;

    case TP_DD2:                        // Wait for demand logs from Flash
      if (SPI1Flash.Ack & S1F_TP_RD_REQ)    // If read is done...
      {
        SPI1Flash.Req &= (uint32_t)(~S1F_TP_RD_REQ);    // Clear request
        // Set up the output buffer
        //   Index,  Address,  EID,  Time Stamp secs,  Time Stamp nsec,   Value
        sprintf(&TP.TxValBuf[0], "%5u", ((uint32_t)TP.ValPtr5));
        TP.TxValBuf[5]= ',';
        TP.TxValBuf[6]= ' ';
        sprintf(&TP.TxValBuf[7], "%8u", TP.Tmp2.u);
        TP.TxValBuf[15]= ',';
        TP.TxValBuf[16]= ' ';
        sprintf(&TP.TxValBuf[17], "%5u", tbuf.d.EID);
        TP.TxValBuf[22]= ',';
        TP.TxValBuf[23]= ' ';
        sprintf(&TP.TxValBuf[24], "%10u", tbuf.d.TS.Time_secs);
        TP.TxValBuf[34]= ',';
        TP.TxValBuf[35]= ' ';
        sprintf(&TP.TxValBuf[36], "%10u", tbuf.d.TS.Time_nsec);
        TP.TxValBuf[46]= ',';
        TP.TxValBuf[47]= ' ';
        if (TP.Temp == 37)
        {
          sprintf(&TP.TxValBuf[48], "% .5E", *TP.ValPtr1);
        }
        else
        {
          sprintf(&TP.TxValBuf[48], "% .5E", (float)(*TP.ValPtr4));
        }
        TP.TxValBuf[60] = ',';
        TP.TxValBuf[61] = ' ';
        sprintf(&TP.TxValBuf[62], "%1u", tbuf.d.Status);
        TP.TxValBuf[63] = '\n';
        TP.TxValBuf[64] = '\r';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 65;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = TP_DD3;                   // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case TP_DD3:                              // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        if (TP.Tmp1.u > 19)                             // If no more values to transmit, we are done, so
        {                                               //   set the state to output the cursor
          TP.State = TP_CURSOR;
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so 
          // Increment the index and save it in TP.ValPtr5 (use this as a temporary)
          indx = (uint32_t)TP.ValPtr5 + 1;
          TP.ValPtr5 = (uint8_t *)(indx);
          TP.SubState = TP_DD1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayDmnd()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayWF()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Trip, Alarm, or Extended-Capture Waveform Log
//
//  MECHANICS:          This subroutine handles displaying the trip, alarm, and strip-chart waveform log
//                      information
//                      Command formats:
//                        DT[log number]<space>[waveform]<space>[info code]  (trip)
//                          log number: 0-25
//                          waveform: 0-5: Ia - Igres, 6-8: VanAFE - VcnAFE, 9-11: VanADC - VcnADC
//                          info code: 0 = summary information for the log
//                                     1-36 = cycle
//                        DA[log number]<space>[waveform]<space>[info code]  (alarm)
//                          log number: 0-20
//                          waveform: 0-5: Ia - Igres, 6-8: VanAFE - VcnAFE, 9-11: VanADC - VcnADC
//                          info code: 0 = summary information for the log
//                                     1-36 = cycle
//                        DC[log number]<space>[waveform]<space>[info code]  (strip-chart)
//                          log number: 0-1
//                          waveform: 0-5: Ia - Igres, 6-8: VanAFE - VcnAFE, 9-11: VanADC - VcnADC
//                          info code: 0 = summary information for the log
//                                     1-36 = cycle
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf(), FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayWF(void)
{
  uint8_t i, j, da_exit;

  da_exit = FALSE;

  while (!da_exit)
  {
    switch (TP.SubState)
    {
      case TP_DA0:                      // Read parameters - Alarm Waveforms
      case TP_DA7:                      // Read parameters - Trip waveforms
      case TP_DA21:                     // Read parameters - Extended-capture waveforms
        if (TP.RxNdxOut != TP.RxNdxIn)      // If there are characters in the receive buffer...
        {
          // Read next parameter - log number (trip: 0 - 24, alarm: 0 - 20, strip-chart: 0 -1)
          i = TP_ParseChars();
          if (i == 1)                           // Must be a decimal number
          {
            TP.Tmp1.b[2] = (uint8_t)(TP_GetDecNum());
          }
          else
          {
            TP.Tmp1.b[2] = 255;                     // If not a decimal number, set to invalid
          }
          if (TP.SubState == TP_DA21)
          {
            j = NUM_EXTCAP_WAVEFORMS;
          }
          else
          {
            j = ((TP.SubState == TP_DA0) ? NUM_ALARM_WAVEFORMS : NUM_TRIP_WAVEFORMS);
          }
          if (TP.Tmp1.b[2] >= j)                    // If invalid or out of range, set to invalid
          {
            TP.Tmp1.b[2] = 255;
          }

          // Read next parameter - waveform (0-5: Ia - Igres, 6-8: VanAFE - VcnAFE, 9-11: VanADC - VcnADC)
          j = TP_ParseChars();
          if (j == 1)                       // Must be a decimal number
          {
            i = TP_GetDecNum();
          }
          else                                      // If not a decimal number, set to 0
          {
            i = 0;
          }
          if (i > 11)                         // If invalid or out of range, set to 0
          {
            i = 0;
          }
          // Set TP.Tmp1.b[3] equal to the byte offset within a sample set.  First 6 samples are floats
          //   (4 bytes); remaining samples are uint16s (2 bytes)
          TP.Tmp1.b[3] = ((i < 6) ? (i * 4) : (24 + ((i-6) * 2)));

          // Read next parameter - information indicator (0: header, 1 - 36 cycle number)
          j = TP_ParseChars();
          if (j == 1)                       // Must be a decimal number.  Save it in TP.Temp.  It is used
          {                                 //   to indicate what to read from Flash
            TP.Temp = TP_GetDecNum();
          }
          else                                      // If not a decimal number, set to invalid
          {
            TP.Temp = 255;
          }
          if (TP.Temp > 36)                         // If invalid or out of range, set to 0
          {
            TP.Temp = 0;
          }
        }
        else                                // If no characters in the receive buffer, set to defaults
        {
          TP.Temp = 0;
          TP.Tmp1.b[2] = 255;
          TP.Tmp1.b[3] = 0;
        }
        // If info indicator is 255, just return the index of the most recent waveform capture
        if (TP.Tmp1.b[2] == 255)
        {
          for (i=0; i<12; ++i)                      // "Num Logs: <number of events>"
          {
            TP.TxValBuf[i] = NUM_WFLOGS[i];
          }
          if (TP.SubState == TP_DA0)
          {
            i = ( (Alarm_WF_Capture.EV_Add.Num_Events < NUM_ALARM_WAVEFORMS) ?
                        (Alarm_WF_Capture.EV_Add.Num_Events) : (NUM_ALARM_WAVEFORMS - 1) );
            j = ( (Alarm_WF_Capture.EV_Add.NextEvntNdx == 0) ?
                        (NUM_ALARM_WAVEFORMS - 1) : (Alarm_WF_Capture.EV_Add.NextEvntNdx - 1) );
          }
          else if (TP.SubState == TP_DA7)
          {
            i = ( (Trip_WF_Capture.EV_Add.Num_Events < NUM_TRIP_WAVEFORMS) ?
                        (Trip_WF_Capture.EV_Add.Num_Events) : (NUM_TRIP_WAVEFORMS - 1) );
            j = ( (Trip_WF_Capture.EV_Add.NextEvntNdx == 0) ?
                        (NUM_TRIP_WAVEFORMS - 1) : (Trip_WF_Capture.EV_Add.NextEvntNdx - 1) );
          }
          else if (TP.SubState == TP_DA21)
          {
            i = ( (Ext_WF_Capture.EV_Add.Num_Events < NUM_EXTCAP_WAVEFORMS) ?
                        (Ext_WF_Capture.EV_Add.Num_Events) : (NUM_EXTCAP_WAVEFORMS - 1) );
            j = ( (Ext_WF_Capture.EV_Add.NextEvntNdx == 0) ?
                        (NUM_EXTCAP_WAVEFORMS - 1) : (Ext_WF_Capture.EV_Add.NextEvntNdx - 1) );
          }

          sprintf(&TP.TxValBuf[12], "%2u", ((uint32_t)i));
          for (i=0; i<12; ++i)                      // "Last Log: <log number>"
          {
            TP.TxValBuf[i+14] = MOST_RECENT_LOG[i];
          }
          sprintf(&TP.TxValBuf[26], "%2u", ((uint32_t)j));
          TP.Status &= (~TP_TX_STRING);                 // Set up to transmit
          TP.Status |= TP_TX_VALUE;                     //   Transmitting values, not string
          TP.TxValNdx = 0;
          TP.NumChars = 28;                             //   28 chars
          UART5->CR1 |= USART_CR1_TXEIE;                // Enable tx interrupts - this begins transmissions
          TP.State = TP_CURSOR;
          da_exit = TRUE;
        }
        // Otherwise if reading header, increment state by 1
        else if (TP.Temp == 0)
        {
          ++TP.SubState;
        }
        // Otherwise reading waveform - increment state by 3 
        else
        {
          DW_temp = 0;                      // Initialize the page offset
          TP.Tmp1.b[1] = 0;                 // Initialize the total number of samples read
          TP.SubState += 3;
        }
        break;

      case TP_DA1:                      // Request FRAM Read
      case TP_DA8:
      case TP_DA22:
        // Assemble the FRAM address and store in TP.Tmp2.u
        if (TP.SubState == TP_DA22)
        {
          TP.Tmp2.u = EXTCAP_WF_INFO;
        }
        else
        {
          TP.Tmp2.u = ((TP.SubState == TP_DA1) ? ALARM_WF_INFO : TRIP_WF_INFO);
        }
        TP.Tmp2.u += (TP.Tmp1.b[2] * WF_HEADER_SIZE);
        // Read the header info
        FRAM_Read(TP.Tmp2.u, (WF_HEADER_SIZE/2), (uint16_t *)(&tbuf.a.NumSamples));
        ++TP.SubState;
        da_exit = TRUE;
        break;

      case TP_DA2:                      // Output header info
      case TP_DA9:
      case TP_DA23:
        for (i=0; i<15; ++i)                          // "Num Samples: <number of samples>
        {
          TP.TxValBuf[i] = NUM_SAMPLES[i];
        }
        sprintf(&TP.TxValBuf[15], "%4u", ((uint32_t)tbuf.a.NumSamples));
        for (i=0; i<6; ++i)                           // "TS: <time stamp>
        {
          TP.TxValBuf[19+i] = TIME_STAMP[i];
        }
        sprintf(&TP.TxValBuf[25], "%10u", tbuf.a.TS.Time_secs);
        TP.TxValBuf[35]= ',';
        TP.TxValBuf[36]= ' ';
        sprintf(&TP.TxValBuf[37], "%10u", tbuf.a.TS.Time_nsec);
        for (i=0; i<7; ++i)
        {                                             // "EID: <event ID>"
          TP.TxValBuf[47+i] = EID_LABEL[i];
        }
        sprintf(&TP.TxValBuf[54], "%10u", tbuf.a.EID);
        TP.Status &= (~TP_TX_STRING);                 // Set up to transmit
        TP.Status |= TP_TX_VALUE;                     //   Transmitting values, not string
        TP.TxValNdx = 0;
        TP.NumChars = 64;                             //   64 chars
        UART5->CR1 |= USART_CR1_TXEIE;                // Enable tx interrupts - this begins transmissions
        TP.State = TP_CURSOR;
        da_exit = TRUE;
        break;

      case TP_DA3:                      // Request alarm waveform values
      case TP_DA10:                     // Request trip waveform values
      case TP_DA24:                     // Request ExtendedCapture waveform values
        // Waveform values are displayed one cycle (80 sample values) at a time.
        // Waveforms are stored 7 sample sets per page.  Table WF_ADDRESS[36][4] provides the page and
        // sample set offset of the first and last sample set of each of the 36 cycles.
        // When extracting the waveform, we will read the selected value from each sample set in  each
        // page.  We cannot just read the samples and increment the address by the sample set size, because
        // the sets are stored in pages (7 sets per page), and the address will be wrong when we cross a
        // page boundary.
        // The waveform that is displayed is a function of:
        //     the waveform log number, TP.Tmp1.b[2], a number from 0 - 20 (20 logs are stored)
        //     the cycle number, TP.Temp, a number from 0 - 35  (36 cycles per waveform are stored)
        //     the page offset, DW_temp, a number from 0 - 12
        // The Flash read address is saved in TP.Tmp2.u
        // The number of samples to read is held in TP.Tmp1.b[0]
        // The total number of samples that have been read is in TP.Tmp1.b[1]
        // THE WAVEFORM ITSELF IS HARD-CODED FOR NOW TO BE IA.  iF WE WANT TO CHANGE THIS, WE WOULD EITHER
        //   NEED TO ADD A PARAMETER TO THE COMMAND, OR ADD A COMMAND TO SET THE PARAMETER.  IN ADDITION, WE
        //   WOULD NEED TO CHANGE THE RETRIEVAL FOR VOLTAGES SINCE THEY ARE INT16 VALUES, NOT FLOATS

        // Assemble the Flash address of the alarm waveform
        //   Address is:
        //       starting address of the alarm waveforms logs
        //     + offset of the selected waveform log
        //     + address of the page of the first cycle within the selected log
        //     + offset of the page and byte of the samples within the cycle
        //     + offset of the selected value (hard-coded for Ia right now)
        if (TP.SubState == TP_DA24)                             // starting address of the ext cap wf logs
        {
          TP.Tmp2.u = ((uint32_t)EXTCAP_WAVEFORM_START << 12);
        }
        else
        {
          TP.Tmp2.u = ((TP.SubState == TP_DA3) ?
                  (((uint32_t)ALARM_WAVEFORMS_START) << 12) : (((uint32_t)TRIP_WAVEFORMS_START) << 12));
        }
        TP.Tmp2.u += ((TP.Tmp1.b[2] * WF_SIZE_IN_SECTORS)<< 12) // offset of the selected waveform log
                  + (WF_ADDRESS[TP.Temp - 1][0] << 8)           // starting page address of the first cycle
                  + TP.Tmp1.b[3];                               // offset of the selected value
        // Add the page and byte offset.  If it is the first set of samples (DW_temp = 0), the page offset
        //   is zero, and the byte offset is non-zero: the first sample is in the starting page.  If it is
        //   not the first sample (DW_temp > 0), the byte offset is zero, and the page offset is DW_temp:
        //   the samples start at the beginning of the page
        TP.Tmp2.u += ((DW_temp == 0) ?                      // offset of the page of within the cycle
                (WF_ADDRESS[TP.Temp - 1][1] * (sizeof(struct RAM_SAMPLES))) : (((uint32_t)DW_temp) << 8));
        // Compute the number of samples to read.  If it is the first set of samples to be read
        //   (DW_temp = 0), it is 7 (the maximum in a page) minus the starting offset of the cycle.
        //   If it is the last set of samples to be read
        //   (DW_temp = WF_ADDRESS[TP.Temp - 1][2] - WF_ADDRESS[TP.Temp - 1][0]), it is the number
        //   of samples sets in the last page.  Otherwise it is seven.
        if (DW_temp == 0)
        {
          TP.Tmp1.b[0] = 7 - WF_ADDRESS[TP.Temp - 1][1];
        }
        else if (DW_temp == (WF_ADDRESS[TP.Temp - 1][2] - WF_ADDRESS[TP.Temp - 1][0]))
        {
          TP.Tmp1.b[0] = WF_ADDRESS[TP.Temp - 1][3] + 1;
        }
        else
        {
          TP.Tmp1.b[0] = 7;
        }
        TP.Tmp1.b[1] += TP.Tmp1.b[0];        // Update the total number of samples that have been read
        SPI1Flash.Req |= S1F_TP_RD_REQ;     // Set request flag from the test port
        ++TP.SubState;
        da_exit = TRUE;
        break;

      case TP_DA4:                      // Wait for waveform samples from Flash
      case TP_DA11:
      case TP_DA25:
        if (SPI1Flash.Ack & S1F_TP_RD_REQ)    // If read is done...
        {
          SPI1Flash.Req &= (uint32_t)(~S1F_TP_RD_REQ);    // Clear request
          ++TP.SubState;                                  // Fall into next state
          // break;
        }
        else                                  // If read is not done, remain in this state and exit
        {
          da_exit = TRUE;
          break;
        }

      case TP_DA5:                      // Display waveform
      case TP_DA12:
      case TP_DA26:
        for (j=0; j<98; ++j)
        {
          TP.TxValBuf[j] = 0;
        }
        i = 0;
        // If outputting currents, floating point values
        if (TP.Tmp1.b[3] <= 20)
        {
          for (j=0; j<TP.Tmp1.b[0]; ++j)
          {
            sprintf(&TP.TxValBuf[i], "% .5E", tbuf.fval[j]);
            i += 12;
            TP.TxValBuf[i++] = '\n';
            TP.TxValBuf[i++] = '\r';
          }
        }
        // If outputting voltages, uint16 values
        else
        {
          for (j=0; j<TP.Tmp1.b[0]; ++j)
          {
            sprintf(&TP.TxValBuf[i], "%6i", ((int32_t)tbuf.i16val[j]));
            i += 6;
            TP.TxValBuf[i++] = '\n';
            TP.TxValBuf[i++] = '\r';
          }
        }
        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = i;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        ++TP.SubState;                          // Go on to next state
        da_exit = TRUE;
        break;

      case TP_DA6:                      // Wait for transmission to complete
      case TP_DA13:
      case TP_DA27:
        if (!(TP.Status & TP_TX_VALUE))     // When done transmitting...
        {
          // There are no more values to transmit when the page offset (in DW_temp) has reached the last
          //   page of the cycle.  If there are no more values to transmit, we are done, so set the state to
          //   output the cursor and exit
          if (DW_temp == (WF_ADDRESS[TP.Temp - 1][2] - WF_ADDRESS[TP.Temp - 1][0]))
          {
            TP.State = TP_CURSOR;
          }
          else                                  // Otherwise there are still alarm sample values to
          {                                     //   transmit, so increment the page offset and jump back to
            ++DW_temp;                          //   read more samples from Flash 
            TP.SubState -= 3;
          }
        }
        da_exit = TRUE;                     // Exit the subroutine in any event
        break;

      default:                          // Invalid state - this should never be entered
        TP.State = TP_CURSOR;
        da_exit = TRUE;
        break;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayWF()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayExtendedValues()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Extended Measured Valules from BB FRAM
//
//  MECHANICS:          This subroutine handles displaying the extended capture logs
//
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

const unsigned char ExtCapNumValsCharText[] = "Number of Values: ";


void TP_DisplayExtendedValues(uint8_t TypeOfRecord)
{
  uint16_t indx;
  uint8_t j;
  uint16_t ExtCapNumVals, NumLogs, LogStart, LogSize = 0;
  char ExtCapNumValsChar[6] = {0};
  uint16_t TP_Index;
  uint16_t SPI_Index;
  
  switch (TypeOfRecord)
  {
    case TwoHundred:
      NumLogs = EXTCAP_TWOHUNDRCYCLE_NUM_LOGS - 1;
      LogStart = EXTCAP_TWOHUNDRCYCLE_LOG_START;
      LogSize = EXTCAP_TWOHUNDRCYCLE_SIZE;
      ExtCapNumVals = sExtCapState.TwoHundredNumVals;
      break;
    
    default:
      NumLogs = EXTCAP_ONECYCLE_NUM_LOGS - 1;
      LogStart = EXTCAP_ONECYCLE_LOG_START;
      LogSize = EXTCAP_ONECYCLE_SIZE;
      ExtCapNumVals = sExtCapState.OneCycleNumVals;
      break;
  }
  
  switch (TP.SubState)
  {
    case TP_MX_0:
      
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > NumLogs)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      TP.StrPtr = &ExtCapNumValsCharText[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = TP_MX_1;
      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;
      
    case TP_MX_1:
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        TP.SubState = TP_MX_2;
      }
      else
      {
        break;
      }


    case TP_MX_2:
      sprintf(&ExtCapNumValsChar[0], "%d \n\r", ExtCapNumVals);
      memcpy (&TP.TxValBuf[0], ExtCapNumValsChar, sizeof(ExtCapNumValsChar));
    
      TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;               // Set flag to transmit values

      TP.TxValNdx = 0;
      TP.NumChars = sizeof(ExtCapNumValsChar);
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_MX_3;
      break;


    case TP_MX_3:
      if (!(TP.Status & TP_TX_VALUE))
      {
        TP.SubState = TP_MX_4;
      }
      else
      {
        break;
      }
      
    case TP_MX_4:
      ExtCapt_FRAM_Read(LogStart + LogSize*TP.Tmp2.u, (LogSize >> 1) , (uint16_t *)(&SPI2_buf[0]));

      TP_Index = 0;
      SPI_Index = 0;

      // Ia
      TP_ParseFloat(TP_Index, 0);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Ib
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Ic
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // In
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Ig
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = '\n';
      TP.TxValBuf[TP_Index++] = '\n';
      TP.TxValBuf[TP_Index++] = '\r';

      TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;               // Set flag to transmit values

      TP.TxValNdx = 0;
      TP.NumChars = TP_Index++;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.SubState = TP_MX_5;
      break;

    case TP_MX_5:
      if (!(TP.Status & TP_TX_VALUE))
      {
        TP.SubState = TP_MX_6;
      }
      else
      {
        break;
      }

    case TP_MX_6:
      TP_Index = 0;
      
      // Van1
      TP_ParseFloat(TP_Index, SPI_Index);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Vbn1
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Vcn1
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';
      
      // Van2
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';
      
      // Vbn2
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE] = ' ';

      // Vcn2
      TP_ParseFloat(TP_Index++, SPI_Index += SPIUINT32SIZE);

      TP.TxValBuf[TP_Index += TPFLOATSIZE] = '\n';
      TP.TxValBuf[TP_Index++] = '\r';

      TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;               // Set flag to transmit values

      TP.TxValNdx = 0;
      TP.NumChars = TP_Index++;
      UART5->CR1 |= USART_CR1_TXEIE;                // Enable transmit interrupts
      TP.State = TP_CURSOR;
      
    break;                                // If read is not done, remain in this state and exit
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayExtendedValues()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplaySummaryEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Summary Event Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//                      The format of the command is:
//                        DV <Starting index>
//                      Starting index is the first log to retrieve, 0-499
//                      For Summary logs, the subroutine returns 20 logs in sequence, beginning with
//                        starting index, in the following format:
//                      index    EID    code    timestamp sec    timestamp nsec
//                      Note, logs are returned by the index in their respective buffer, not by EID
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
// 
//------------------------------------------------------------------------------------------------------------

const unsigned char SUMMARY_EVENT_HEADER[] = "Index        EID      TS sec    TS nsec     Code\n\r";
const unsigned char EVENT_NUMLOGS[] = "Number of logs = ";

void TP_DisplaySummaryEvent(void)
{
  //uint32_t temp;
  uint16_t indx;
  uint8_t j;
  uint16_t TP_Index;
  uint16_t SPI_Index;

  TP_Index = 0;
  SPI_Index = 0;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if ( (indx+1) > EV_Sum.Num_Events)          // If index out of range, set to 0
        {                                           //   Compare this way in case num_events = 0
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;

      // Output number of logs string
      TP.StrPtr = &EVENT_NUMLOGS[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;
      break;

    case 1:
      // If done transmitting, output the number of summary logs
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        sprintf(&TP.TxValBuf[0], "%3u", (uint32_t)EV_Sum.Num_Events);
        TP.TxValBuf[3] = '\n';
        TP.TxValBuf[4] = '\r';
        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 5;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        if (EV_Sum.Num_Events > 0)              // If events to output, go on to state 2
        {
          TP.SubState = 2;
        }
        else                                    // Otherwise done
        {
          TP.State = TP_CURSOR;
        }
      }
      break;

    case 2:
      // If done transmitting, output the header
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Output header
        TP.StrPtr = &SUMMARY_EVENT_HEADER[0];
        TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
        TP.Status |= TP_TX_STRING;              // Set flag to transmit string
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 3;
      }
      break;

    case 3:                             // Wait for header
      // If done transmitting, output the logs
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((SUMMARY_LOG_START + (TP.Tmp2.u * SUMMARY_EVENT_SIZE)),
                    (SUMMARY_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer

        // index
        sprintf(&TP.TxValBuf[TP_Index], "%5u", TP.Tmp2.u);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINDEXSIZE);
        
        // EID
        TP_ParseUInt32(TP_Index++,SPI_Index);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
        
        // TS sec
        TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
        
        // TS nanosec
        TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(2, TP_Index += TPUINT32SIZE);

        // Code
        TP_ParseUInt16(TP_Index++, SPI_Index += SPIUINT32SIZE);
        TP.TxValBuf[TP_Index += TPUINT16SIZE]= '\n';
        TP.TxValBuf[TP_Index++]= '\n';
        TP.TxValBuf[TP_Index++]= '\r';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = TP_Index++;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 4;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 4:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u >= EV_Sum.Num_Events) )    // If no more values to transmit,
        {                                                              //   we are done, so set the state
          TP.State = TP_CURSOR;                                        //   to output the cursor
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = 3;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplaySummaryEvent()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpSumEvent()
//------------------------------------------------------------------------------------------------------------

const unsigned char LOOKUP_HEADER[] = "  EID   Index\n\r";

void TP_LookUpSumEvent(void)
{
  uint32_t LookUpEID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          LookUpEID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        LookUpEID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = LookUpEID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_SUMMARY);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpSumEvent()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpTripEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpTripEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_TRIP);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpTripEvent()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpTestTripEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpTestTripEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_TESTTRIP);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpTestTripEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpAlarmEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpAlarmEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_ALARM);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpAlarmEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpDemandEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpDemandEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_DEMAND);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpDemandEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpTripWFEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpTripWFEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_TRIPWF);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpTripWFEvent()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpAlarmWFEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpAlarmWFEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_ALARMWF);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpAlarmWFEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_LookUpExtCapWFEvent()
//------------------------------------------------------------------------------------------------------------


void TP_LookUpExtCapWFEvent(void)
{
  uint32_t EID;
  int16_t index;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          EID = TP_GetDecNum();
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        EID = 0;
      }

      // Output header
      TP.StrPtr = &LOOKUP_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = EID;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        index = EventLookUp(TP.Tmp2.u, EV_TYPE_EXTCAPWF);
        
        sprintf(&TP.TxValBuf[0], "%8u", ((uint32_t)TP.Tmp2.u));
        if (index < 0) sprintf(&TP.TxValBuf[8], "N/A ");
        else sprintf(&TP.TxValBuf[8], "%5u", ((uint32_t)index));
        TP.TxValBuf[13]= '\r';
        TP.TxValBuf[14]= '\n';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = 15;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        TP.State = TP_CURSOR;
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_LookUpExtCapWFEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayTripEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Trip Event Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//                      The format of the command is:
//                        DV <Starting index>
//                      Starting index is the first log to retrieve, 0-499
//                      For Summary logs, the subroutine returns 20 logs in sequence, beginning with
//                        starting index, in the following format:
//                      index    EID    code    timestamp sec    timestamp nsec
//                      Note, logs are returned by the index in their respective buffer, not by EID
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
//
//------------------------------------------------------------------------------------------------------------

const unsigned char EVENT_LOG_HEADER[] = "Index        EID      TS sec    TS nsec      Ia          Ib\n\r";

void TP_DisplayTripEvent(void)
{
//  uint32_t temp;
//  float temp_f;
  uint16_t indx;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > TRIP_NUM_LOGS - 1)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }         // Wait for header

      // Output header
      TP.StrPtr = &EVENT_LOG_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;

    case 1:
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((TRIP_LOG_START + (TP.Tmp2.u * SNAPSHOTS_EVENT_SIZE)),
                   (SNAPSHOTS_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer
        TP_ParseEventsTable((uint32_t)TP.Tmp2.u);
        
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u > TRIP_NUM_LOGS-1) )    // If no more values to transmit, we
        {                                                           //   are done, so set the state to
          TP.State = TP_CURSOR;                                     //   output the cursor
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = 1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayTripEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayTestTripEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Test Trip Event Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//                      The format of the command is:
//                        DV <Starting index>
//                      Starting index is the first log to retrieve, 0-499
//                      For Summary logs, the subroutine returns 20 logs in sequence, beginning with
//                        starting index, in the following format:
//                      index    EID    code    timestamp sec    timestamp nsec
//                      Note, logs are returned by the index in their respective buffer, not by EID
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
//
//------------------------------------------------------------------------------------------------------------

void TP_DisplayTestTripEvent(void)
{
//  uint32_t temp;
//  float temp_f;
  uint16_t indx;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > TESTTRIP_NUM_LOGS - 1)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      // Output header
      TP.StrPtr = &EVENT_LOG_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((TESTTRIP_LOG_START + (TP.Tmp2.u * SNAPSHOTS_EVENT_SIZE)),
                    (SNAPSHOTS_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer
        TP_ParseEventsTable((uint32_t)TP.Tmp2.u);

        TP.SubState = 2;                        // Go on to next state

      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u > TESTTRIP_NUM_LOGS - 1) )   // If no more values to transmit,
        {                                                                //   we are done, so set the state
          TP.State = TP_CURSOR;                                          //   to output the cursor
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = 1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayTestTripEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayAlarmEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Alarm Event Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//                      The format of the command is:
//                        DV <Starting index>
//                      Starting index is the first log to retrieve, 0-499
//                      For Summary logs, the subroutine returns 20 logs in sequence, beginning with
//                        starting index, in the following format:
//                      index    EID    code    timestamp sec    timestamp nsec
//                      Note, logs are returned by the index in their respective buffer, not by EID
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
//
//------------------------------------------------------------------------------------------------------------

void TP_DisplayAlarmEvent(void)
{
//  uint32_t temp;
//  float temp_f;
  uint16_t indx;
  uint8_t j;

  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > ALARM_NUM_LOGS - 1)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      // Output header
      TP.StrPtr = &EVENT_LOG_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((ALARM_LOG_START + (TP.Tmp2.u * SNAPSHOTS_EVENT_SIZE)),
                    (SNAPSHOTS_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer

        TP_ParseEventsTable((uint32_t)TP.Tmp2.u);

        TP.SubState = 2;                        // Go on to next state

      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u > ALARM_NUM_LOGS - 1) )   // If no more values to transmit, we
        {                                                             //   are done, so set the state to
          TP.State = TP_CURSOR;                                       //   output the cursor
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = 1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayAlarmEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION         TP_DisplayTimeAdjEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display TimeAdj Event Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//                      The format of the command is:
//                        DV <Starting index>
//                      Starting index is the first log to retrieve, 0-499
//                      For Summary logs, the subroutine returns 20 logs in sequence, beginning with
//                        starting index, in the following format:
//                      index    EID    code    timestamp sec    timestamp nsec
//                      Note, logs are returned by the index in their respective buffer, not by EID
//                      
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
//
//------------------------------------------------------------------------------------------------------------

const unsigned char TIMEADJ_EVENT_HEADER[] = "Index        EID      TS sec    TS nsec   Source      NewT[s]\n\r";

void TP_DisplayTimeAdjEvent(void)
{
  //uint32_t temp;
  uint16_t indx;
  uint8_t j;

  uint16_t TP_Index;
  uint16_t SPI_Index;

  TP_Index = 0;
  SPI_Index = 0;


  switch (TP.SubState)
  {
    case 0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > TIMEADJ_NUM_LOGS - 1)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      // Output header
      TP.StrPtr = &TIMEADJ_EVENT_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = 1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;

    case 1:                             // Wait for header
      // If done transmitting, output a cursor
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((TIMEADJ_LOG_START + (TP.Tmp2.u * TIMEADJ_EVENT_SIZE)),
                    (TIMEADJ_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer

        // index
        sprintf(&TP.TxValBuf[TP_Index], "%5u", TP.Tmp2.u);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINDEXSIZE);
        
        // EID
        TP_ParseUInt32(TP_Index++,SPI_Index);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
        
        // TS sec
        TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);
        
        // TS nanosec
        TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(2, TP_Index += TPUINT32SIZE);

        // Source
        TP_ParseUInt32(TP_Index++, SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);

        // New Time
        TP_ParseUInt32(TP_Index++, SPI_Index);
        TP.TxValBuf[TP_Index += TPUINT32SIZE]= '\n';
        TP.TxValBuf[TP_Index++]= '\n';
        TP.TxValBuf[TP_Index++]= '\r';

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = TP_Index++;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = 2;                        // Go on to next state
      }
      break;                                // If read is not done, remain in this state and exit

    case 2:                             // Wait for transmission to complete
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u > TIMEADJ_NUM_LOGS - 1) )   // If no more values to transmit, we
        {                                                               //   are done, so set the state to
          TP.State = TP_CURSOR;                                         //   output the cursor
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = 1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayTimeAdjEvent()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION         TP_DisplayDisturbanceEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Disturbance Capture Logs
//
//  MECHANICS:          This subroutine handles displaying the event logs
//
//  CAVEATS:            None
//
//  INPUTS:             TP.SubState, TP.RxNdxIn
// 
//  OUTPUTS:            TP.State, TP.TxValBuf[], TP.Status
//
//  ALTERS:             TP.RxNdxOut, TP.ValPtr1, TP.Temp, TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), sprintf()
//
//------------------------------------------------------------------------------------------------------------

//const unsigned char DISTURBANCE_EVENT_HEADER[] = "Index        EID      TS sec    TS nsec   Source      NewT[s]\n\r";
const unsigned char DISTURBANCE_EVENT_HEADER[] = "Index       EID     Exit[s]    Entry[s]       Duration       ValueCode      Spare        MaxMin        Avg        ThermalCap\n\r";

void TP_DisplayDisturbanceEvent(void)
{
  //uint32_t temp;
  uint16_t indx;
  uint8_t j;

  uint16_t TP_Index;
  uint16_t SPI_Index;

  switch (TP.SubState)
  {
    case TP_MX_0:                             // Read parameters
      if (TP.RxNdxOut != TP.RxNdxIn)        // If there are characters in the receive buffer...
      {
        j = TP_ParseChars();                // Read next parameter - index of the first log to read
        if (j == 1)                                 // Must be a decimal number
        {
          indx = TP_GetDecNum();
        }
        else                                        // If not a decimal number, set to 0
        {
          indx = 0;
        }
        if (indx > DIST_NUM_LOGS - 1)                             // If invalid or out of range, set to 0 *** DAH SHOULD BE MAX NUMBER OF LOGS
        {
          indx = 0;
        }
      }
      else                                  // If no characters in the receive buffer, set to defaults
      {
        indx = 0;
      }

      // Output header
      TP.StrPtr = &DISTURBANCE_EVENT_HEADER[0];
      TP.Status &= (~TP_TX_VALUE);            // Make sure flag to transmit values is clear
      TP.Status |= TP_TX_STRING;              // Set flag to transmit string
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = TP_MX_1;

      TP.Tmp1.u = 0;
      TP.Tmp2.u = indx;
      break;

    case TP_MX_1:                             // Wait for header
      // If done transmitting, output a cursor
      
      if (!(TP.Status & (TP_TX_STRING + TP_TX_VALUE)))
      {
        // Read the events from FRAM
        FRAM_Read((DIST_LOG_START + (TP.Tmp2.u * DIST_EVENT_SIZE)),
                    (DIST_EVENT_SIZE >> 1), (uint16_t *)(&SPI2_buf[0]));
        // Move the events into the Tx buffer
        TP_Index = 0;
        SPI_Index = 0;

        // index
        sprintf(&TP.TxValBuf[TP_Index], "%5u", TP.Tmp2.u);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINDEXSIZE);

        // EID
        TP_ParseUInt32(TP_Index++,SPI_Index);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);

        // Exit TS sec
        TP_ParseUInt32(TP_Index++,SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT32SIZE);

        // Entry time TS sec
        TP_ParseUInt32(TP_Index++,SPI_Index += 2*SPIUINT32SIZE);
        TP_Index = TP_PutSpace(6, TP_Index += TPUINT32SIZE);

        // Duration
        TP_ParseFloat(TP_Index++,SPI_Index += 2*SPIUINT32SIZE);
        TP_Index = TP_PutSpace(6, TP_Index += TPFLOATSIZE);

        // ValueCode
        TP_ParseUInt16(TP_Index++, SPI_Index += SPIUINT32SIZE);
        TP_Index = TP_PutSpace(3, TP_Index += TPUINT16SIZE);

        TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
        TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
        TP.TxValNdx = 0;
        TP.NumChars = TP_Index++;
        UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
        TP.SubState = TP_MX_2;
      }
      break;                                // If read is not done, remain in this state and exit

    case TP_MX_2:
      if (!(TP.Status & TP_TX_VALUE))
      {
        TP.SubState = TP_MX_3;
      }
      else
      {
        break;
      }

    case TP_MX_3:
      
      TP_Index = 0;

      // Spare
      TP_ParseUInt16(TP_Index, SPI_Index += 6*SPIUINT32SIZE + SPIUINT16SIZE);
      TP_Index = TP_PutSpace(3, TP_Index += TPUINT16SIZE);

      // MaxMin Value
      TP_ParseFloat(TP_Index++,SPI_Index += SPIUINT16SIZE);
      TP_Index = TP_PutSpace(2, TP_Index += TPFLOATSIZE);

      // Avg Value
      TP_ParseFloat(TP_Index++,SPI_Index +=SPIUINT32SIZE);
      TP_Index = TP_PutSpace(2, TP_Index += TPFLOATSIZE);
      
      // Thermal Capacitance
      TP_ParseFloat(TP_Index++,SPI_Index += SPIUINT32SIZE);
      TP.TxValBuf[TP_Index += TPFLOATSIZE]= '\n';
      
      TP.TxValBuf[TP_Index++]= '\n';
      TP.TxValBuf[TP_Index++]= '\r';

      TP.Status &= (~TP_TX_STRING);           // Make sure flag to transmit string is clear
      TP.Status |= TP_TX_VALUE;               // Set flag to transmit values
      TP.TxValNdx = 0;
      TP.NumChars = TP_Index++;
      UART5->CR1 |= USART_CR1_TXEIE;          // Enable transmit interrupts
      TP.SubState = TP_MX_4;
      
      break;                                // If read is not done, remain in this state and exit

    case TP_MX_4:
      if (!(TP.Status & TP_TX_VALUE))               // When done transmitting...
      {
        TP.Tmp1.u++;                                    // Increment the line count
        TP.Tmp2.u++;                                    // Increment the line count
        if ( (TP.Tmp1.u > 19) || (TP.Tmp2.u > DIST_NUM_LOGS-1) )    // If no more values to transmit, we are done, so
        {                                               //   set the state to output the cursor
          TP.State = TP_CURSOR;
        }
        else                                        // Otherwise there are still demand log values to
        {                                           //   transmit, so go back to read the next log 
          TP.SubState = TP_MX_1;
        }
      }
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayDisturbanceEvent()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_AuxRelays()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Auxiliary Relays
//
//  MECHANICS:          This subroutine reads the command string to determine which auxiliary relay to test,
//                      and whether to open or close the relay.  It then acts accordingly.
//                      
//  CAVEATS:            None
//
//  INPUTS:             None
// 
//  OUTPUTS:            Aux relay port control lines
//
//  ALTERS:             None
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------

void TP_AuxRelays(void)
{
  uint8_t i, j;

  i = TP_ParseChars();                      // Read next parameter - must be either decimal or hex number
  if (i == 1)
  {
    i = (uint8_t)(TP_GetDecNum());
  }
  else                                      // If invalid number, set i to 0xFF
  {
    i = 0xFF;
  }
  while ( (TP.RxNdxOut != TP.RxNdxIn)         // Parse the Rx buffer until a letter is found, a line
       && (TP.RxBuf[TP.RxNdxOut] != 0x0A) )   //   feed is found, or there are no chars left
  {
    j = TP.RxBuf[TP.RxNdxOut] & 0xDF;         // Convert char to upper-case
    TP.RxNdxOut = (TP.RxNdxOut + 1) & 0x1F;
    if ( (j >= 'A') && (j <= 'Z') )
    {
      break;
    }
  }
  
  switch (i)                                // Open or close the relay given by i based on the command
  {
    case 1:                                 // Relay 1
      if (j == 'C')                         // If not 'C' or 'O', don't do anything
      {
        RELAY1_CLOSE;
      }
      else if (j == 'O')
      {
        RELAY1_OPEN;
      }
      break;

    case 2:                                 // Relay 2
      if (j == 'C')
      {
        RELAY2_CLOSE;
      }
      else if (j == 'O')
      {
        RELAY2_OPEN;
      }
      break;

    case 3:                                 // Relay 3
      if (j == 'C')
      {
        RELAY3_CLOSE;
      }
      else if (j == 'O')
      {
        RELAY3_OPEN;
      }
      break;

    default:                                // Invalid relay number
      break;
  }                                             // Leave next state at idle to ignore the command
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_AuxRelays()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_RestoreUnit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Restore Unit
//
//  MECHANICS:          This subroutine restores the unit to its "out of the box condition".  Presently, it
//                      resets the events (reset the EID, set the demand/energy log back to zero).
//                      Other functions will be added later.
//                      
//  CAVEATS:            None
//
//  INPUTS:             SPI1Flash.Ack (S2F_CLR_EVENTS), TP.SubState
// 
//  OUTPUTS:            SPI1Flash.Req (S2F_CLR_EVENTS), TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              ClearEvents()
// 
//------------------------------------------------------------------------------------------------------------

void TP_RestoreUnit(void)
{

  switch (TP.SubState)
  {
    case TP_RU0:                            // Set request flag
      ClearEvents();                        // Note, this subroutine sets the S1F_DMND_ERASE request flag to
      TP.State = TP_CURSOR;                 //  erase the next Flash demand sector
      break;

    default:                                // Invalid state - this should never be entered
      TP.State = TP_CURSOR;
      break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_RestoreUnit()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayGF_CTstatus()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           This function displays the configuration of the ground circtuit. The circuit will
//                      either be configured for a CT input or a Rogowski input.
//
//  MECHANICS:          The function checks GPIOD pin 0.  If the pin is HIGH, the ground input is set for a
//                      Rogowski input and this is printed at the test port terminal.  If the pin is LOW,
//                      the ground input is set for a CT input and this is printed at the test port terminal
//                      
//  CAVEATS:            
//
//  INPUTS:             input - whether a user input is expected (if so print a ":"
// 
//  OUTPUTS:            TP.Status, TP.TxValBuf[ ], TP.NumChars
//
//  ALTERS:             None
// 
//  CALLS:              sprintf()    
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayGF_CTstatus(unsigned char input)
{
  if (GF_USING_ROGOWSKI)                    // If Rogowski is set as input
  {
    sprintf(&TP.TxValBuf[0], "\tGF is set to Rog input");
    TP.NumChars = 23;
  }
  else                                      // Otherwise CT is set as input
  {
    sprintf(&TP.TxValBuf[0], "\tGF is set to CT input");
    TP.NumChars = 22;
  }
  if (input)
  {
    sprintf(&TP.TxValBuf[TP.NumChars], ":  ");
    TP.NumChars += 3;
  }

  TP.Status &= (~TP_TX_STRING);
  TP.Status |= TP_TX_VALUE;
  TP.TxValNdx = 0;
  UART5->CR1 |= USART_CR1_TXEIE;            // Enable tx interrupts - this begins the transmissions
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayGF_CTstatus()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_SetReset_GF_CT_EN()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           A test port command GF0 resets GF_CT_EN (PD0) and configures the ground input for a
//                      Rogowski input.  A test port command GF1 sets GF_CT_EN and configures the ground
//                      input for a CT input.
//
//  MECHANICS:          This function reads the input to the test port and either sets or resets the
//                      GF_CT_EN bit. If the user enters "GF0", the bit is reset, if the user enters "GF1",
//                      the bit is set.  
//                      
//  CAVEATS:            
//
//  INPUTS:             TP.RxBuf[ ], TP.RxNdxIn, TP.RxNdxOut
// 
//  OUTPUTS:            none
//
//  ALTERS:            
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), TP_GetHexNum(), TP_DisplayGF_CTstatus()
// 
//------------------------------------------------------------------------------------------------------------

void TP_SetReset_GF_CT_EN(void)
{
  uint8_t i, j;

  // Wait for input code (0 = use Rogowski, 1 = use CT, anything else = stay as is and exit)
  j = TP.RxNdxOut;
  while (j != TP.RxNdxIn)               // Check the Rx buffer for a line feed - this indicates that a new
  {                                     //   code has been entered
    if (TP.RxBuf[j] == 0x0A)            // If LF received...
    {
      j = TP.RxNdxIn;                       // Set j to TP.RxNdxIn to force exit from loop
      i = TP_ParseChars();                  // Read next parameter - must be either decimal or hex number
      if (i == 1)
      {
        i = (uint8_t)(TP_GetDecNum());
      }
      else if (i == 2)
      {
        i = (uint8_t)(TP_GetHexNum());
      }
      else                                  // If invalid number, set i to 0xFF
      {
        i = 0xFF;
      }
      if (i == 0)                           // Using Rogowski:
      {
        GPIOD->BSRRH = 1;                       // Activate GF_CT_EN (active low)
        TP_DisplayGF_CTstatus(FALSE);           // Output the configuration
      }
      else if (i == 1)
      {
        GPIOD->BSRRL = 1;                       // Deactivate GF_CT_EN (active low)
        TP_DisplayGF_CTstatus(FALSE);           // Output the configuration
      }
      TP.State = TP_CURSOR;                 // Next state is idle
    }
    else                                // Otherwise increment the temporary Rx buffer index
    {
      j = (j + 1) & 0x1F;
    }
  }
     
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_SetReset_GF_CT_EN()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_DisplayN_CTstatus()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           This function displays the configuration of the neutral circtuit. The circuit will
//                      either be configured for a CT input or a Rogowski input.
//
//  MECHANICS:          The function checks GPIOD pin 14.  If the pin is HIGH, the neutral input is set for a
//                      Rogowski input and this is printed at the test port terminal.  If the pin is LOW,
//                      the neutral input is set for a CT input and this is printed at the test port
//                      terminal.
//                      
//  CAVEATS:            
//
//  INPUTS:             input - whether a user input is expected (if so print a ":"
// 
//  OUTPUTS:            TP.Status, TP.TxValBuf[ ], TP.NumChars
//
//  ALTERS:             None
// 
//  CALLS:              sprintf()    
// 
//------------------------------------------------------------------------------------------------------------

void TP_DisplayN_CTstatus(unsigned char input)
{
  if (IN_USING_ROGOWSKI)                    // If Rogowski is set as input
  {
    sprintf(&TP.TxValBuf[0], "\tNeutral input is set to Rog input");
    TP.NumChars = 34;
  }
  else                                      // Otherwise CT is set as input
  {
    sprintf(&TP.TxValBuf[0], "\tNeutral input is set to CT input");
    TP.NumChars = 33;
  }
  if (input)
  {
    sprintf(&TP.TxValBuf[TP.NumChars], ":  ");
    TP.NumChars += 3;
  }

  TP.Status &= (~TP_TX_STRING);
  TP.Status |= TP_TX_VALUE;
  TP.TxValNdx = 0;
  UART5->CR1 |= USART_CR1_TXEIE;            // Enable tx interrupts - this begins the transmissions
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_DisplayN_CTstatus()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_SetReset_N_CT_EN()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test port command GF0 resets N_CT_EN (PD14) and configures the neutral input for a
//                      Rogowski input.  Test port command GF1 sets N_CT_EN and configures the neutral input
//                      for a CT input.
//
//  MECHANICS:          This function reads the input to the test port and either sets or resets the
//                      N_CT_EN bit. If the user enters "GF0", the bit is reset, if the user enters "GF1",
//                      the bit is set.  
//                      
//  CAVEATS:            
//
//  INPUTS:             TP.RxBuf[ ], TP.RxNdxIn, TP.RxNdxOut
// 
//  OUTPUTS:            none
//
//  ALTERS:            
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), TP_GetHexNum(), TP_DisplayN_CTstatus()
// 
//------------------------------------------------------------------------------------------------------------

void TP_SetReset_N_CT_EN(void)
{
  uint8_t i, j;

  // Wait for input code (0 = use Rogowski, 1 = use CT, anything else = stay as is and exit)
  j = TP.RxNdxOut;
  while (j != TP.RxNdxIn)               // Check the Rx buffer for a line feed - this indicates that a new
  {                                     //   code has been entered
    if (TP.RxBuf[j] == 0x0A)            // If LF received...
    {
      j = TP.RxNdxIn;                       // Set j to TP.RxNdxIn to force exit from loop
      i = TP_ParseChars();                  // Read next parameter - must be either decimal or hex number
      if (i == 1)
      {
        i = (uint8_t)(TP_GetDecNum());
      }
      else if (i == 2)
      {
        i = (uint8_t)(TP_GetHexNum());
      }
      else                                  // If invalid number, set i to 0xFF
      {
        i = 0xFF;
      }
      if (i == 0)                           // Using Rogowski:
      {
        GPIOD->BSRRH = 0x4000;                  // Activate N_CT_EN (active low)
        TP_DisplayN_CTstatus(FALSE);            // Output the configuration
      }
      else if (i == 1)
      {
        GPIOD->BSRRL = 0x4000;                  // Deactivate N_CT_EN (active low)
        TP_DisplayN_CTstatus(FALSE);            // Output the configuration
      }
      TP.State = TP_CURSOR;                 // Next state is idle
    }
    else                                // Otherwise increment the temporary Rx buffer index
    {
      j = (j + 1) & 0x1F;
    }
  }
     
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_SetReset_N_CT_EN()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Load_ExecuteAction_struct()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Fill Execute action structure
//
//------------------------------------------------------------------------------------------------------------
uint8_t Load_ExecuteAction_struct(uint8_t bid, uint8_t *msg)
{
  uint8_t result = 1;
  uint32_t temp;

  ExAct.channel = *(msg + 8);
  if(bid == 12 || bid == 14 || bid == 16)//cal gain
  {    
    temp = ((uint32_t)(*(msg + 12))) << 24;
    temp |= ((uint32_t)(*(msg + 11))) << 16;
    temp |= ((uint32_t)(*(msg + 10))) << 8;
    temp |= ((uint32_t)(*(msg + 9)));
    ExAct.target = (float)temp;
    ExAct.target = ExAct.target/16.0; 
  }

  switch(bid)
  {
    case 11:  //cal offset AFE
      ExAct.State = OCAFE;                                                           
      ExAct.SubState = CAL0;
      break;
      
     case 12: // cal gain AFE
      ExAct.State = GCAFE;                                                           
      ExAct.SubState = CAL0;
      break;
      
     case 13: //cal Offset ADC Low Gain
      ExAct.State = OCL;                                                           
      ExAct.SubState = CAL0;
      break;
      
     case 14: //cal Gain ADC Low Gain
      ExAct.State = GCL;                                                           
      ExAct.SubState = CAL0;
      break;

     case 15: //cal Offset ADC High Gain
      ExAct.State = OCH;                                                           
      ExAct.SubState = CAL0;
      break;

     case 16: //cal Gain ADC High Gain
      ExAct.State = GCH;                                                           
      ExAct.SubState = CAL0;    
      break;

     case 19: //cal Offset Test Injection
      ExAct.State = OCSI;                                                           
      ExAct.SubState = CAL0;    
      break;

     case 20: //cal Gain Test Injection
      ExAct.State = GCSI;                                                           
      ExAct.SubState = CAL0;   
      ExAct.target = *(msg + 9);
      break;

     default:
      result = 0;
      break;
      
  }

  return result;
  
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_SetReset_N_CT_EN()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ExAct_Top()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Top-Level Subroutine for Execute action commands
//
//  MECHANICS:          This subroutine is the top-level subroutine for Execute Action commands.  It operates
//                      in either IDLE or ACTIVE state.
//                      In the IDLE state, the subroutine:
//                        - waits for State change by new execute action command
//                        - otherwise remains in the IDLE state.
//                      In the ACTIVE state, the subroutine:
//                        - Handles required Execute Action and returns to IDLE state
//
//------------------------------------------------------------------------------------------------------------
void ExAct_Top(void)
{
  switch(ExAct.State)
  {
        
    case IDLE:
      {
        ; //nothing to do
      }
      break;
    case OCAFE:
      Cal_Offset_AFE();
      break;
    case GCAFE:
      Cal_Gain_AFE();
      break;
    case OCH:
      Cal_Offset_HG();
      break;
    case GCH:
      Cal_Gain_HG();
      break;
    case OCL:
      Cal_Offset_LG();
      break;
    case GCL:
      Cal_Gain_LG();
      break; 
    case OCSI:
      Cal_Offset_SI();
      break;
    case GCSI:
      Cal_Gain_SI();
      break;
    case WRCAL:
    {
      if(WriteCal())
      {
        ExAct.State = IDLE;
      }
    }
    break;
    case LED_TEST:
      Service_LED_test(); 
      break;
    case RELAY_TEST:
      Service_Relay_test();
      break;

    default:
      break;      

  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ExAct_Top()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Gain_AFE()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           AFE Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the AFE
//                      gain conversions.  The calibration is performed on a single channel.  Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated, along with the actual input
//                             current
//                          2) Set the hardware to the circuit being calibrated 
//                          3) Wait for a new reading.  When the flag is set, add the new reading to the
//                             running total in Tmp1.f.
//                          4) Repeat this process 100 times.
//                          5) When 100 readings have been accumulated, divide the result by 100 to get the
//                             average reading.
//                          6) Multiply the existing gain constant by the actual input divided by the
//                             average reading
//                          7) Save the result in the gain for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      The 200msec reading is used for AFE calibration because this is the reading that is
//                      used for metering.  
//                      Note:
//                        ExAct.ValPtr2 points to the gain for the channel being calibrated
//                        ExAct.ValPtr1 points to the 200msec (for the AFE) or the one-cycle (for the ADC)
//                              reading
//                        ExAct.Tmp2.f holds the actual input current (enterd by the user)
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 readings
//                        ExAct.Temp holds the cycle count
//                      Note:
//                        The ADC voltages are the same for both the high-gain and low-gain circuits.  In
//                        other words, the high gain and low-gain circuits only affect the currents.
//                        However, the high-gain and low-gain constants use the same storage structure, and
//                        are stored separately - the ADC voltage cal constants are kept in both the
//                        high-gain and the low-gain cal structures.
//                      
//  CAVEATS:            The subroutine assumes the input to the channel has been properly set up.
//                      Offset calibration should be performed first!!
//
//------------------------------------------------------------------------------------------------------------

void Cal_Gain_AFE(void)
{
  uint8_t i;
  switch(ExAct.SubState)
  {
    case CAL0:
    {    
      UseADCvals = FALSE;                      // Make sure using AFE values
      ExAct.channels_amount = 1;
      ExAct.calPtrArray[0] = &AFEcal.gain[ExAct.channel];

      if ( (ExAct.channel == 4) && (GF_USING_ROGOWSKI) )   // If Igsrc calibration and using a Rogowski coil, cal
      {                                        //   constant is held in index = 8
        ExAct.calPtrArray[0] = &AFEcal.gain[8];
        ExAct.mPtr = &Cur200msFltr.Igsrc;
      }
      else if ( (ExAct.channel == 3) && (!IN_USING_ROGOWSKI) )  // If In calibration and using a CT, cal constant
      {                                             //   is held in index = 9
        ExAct.calPtrArray[0] = &AFEcal.gain[9];
        ExAct.mPtr = &Cur200msFltr.In;
      }
      else if (ExAct.channel < 5 )   // If Ia, Ib, Ic, In, or Igsrc with CT, cal constant is
      {  
         ExAct.calPtrArray[0] = &AFEcal.gain[ExAct.channel];
         ExAct.mPtr = &Cur200msFltr.Ia + ExAct.channel;     
      } 
      else if (ExAct.channel < 8)    // Otherwise VanADC - Vcn_ADC
      {
        ExAct.calPtrArray[0] = &AFEcal.gain[ExAct.channel];
        ExAct.mPtr = &VolAFE200msFltr.Van + (ExAct.channel - 5);
      }
      else if(ExAct.channel == 8)
      {
        //all currents
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &AFEcal.gain[i];
        }
        ExAct.calPtrArray[3] = (IN_USING_ROGOWSKI) ?  &AFEcal.gain[3] :  &AFEcal.gain[9];
        ExAct.calPtrArray[4] = (GF_USING_ROGOWSKI) ?  &AFEcal.gain[8] :  &AFEcal.gain[4];
        ExAct.mPtr = &Cur200msFltr.Ia;
        ExAct.channels_amount  = 5;        
      }
      else if (ExAct.channel == 9)
      {
        //all voltages
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &AFEcal.gain[5 + i];
        }
        ExAct.mPtr = &VolAFE200msFltr.Van;
        ExAct.channels_amount  = 3;
      }      
      else
      {
         //invalid for example GF cal but incorect GF setting
         ExAct.State = IDLE;
         break;
      }
      ExAct.Temp = 0;
      ExAct.SubState = CAL1;
      for(i=0; i<ExAct.channels_amount; i++)
      {
        ExAct.Temp1[i].f = 0;
      }
      

    }
    break;
    case CAL1:
    {
      if (SystemFlags & VAL200MSEC)         // If new reading, add to temporary sum
      {
        if (ExAct.Temp > 0)                    // Throw out the first reading since it may include samples from
        {                                   //   the ADC circuit
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f += *(ExAct.mPtr +i);
          }          
        }
        // If accumulated 100 readings, compute the average reading (sum/100).  Adjust the gain by the ratio
        // of the ideal reading (in Tmp2.f) amd the average reading (in Tmp1.f).  Note, gain is referenced
        // by *TP.ValPtr2
        if (ExAct.Temp == 100)
        {
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f = ExAct.Temp1[i].f/100.0f;
            *(ExAct.calPtrArray[i]) = (*(ExAct.calPtrArray[i]) * ExAct.target)/ExAct.Temp1[i].f; 
          } 
                                                      // Compute and save new checksum and complement
          AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
          AFEcal.cmp = ~AFEcal.chk;
          SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
          ExAct.State = WRCAL;                         // Set next state to write cal constants to FRAM and Flash
        }                                               
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          ExAct.Temp++;
        }
      }

    }
    break;

    default:
      break;    

  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Gain_AFE()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Offset_AFE()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Offset Calibration AFE
//
//  MECHANICS:          This subroutine handles offset calibration for the AFE. Calibration consists of the
//                      following steps:
//                          1) Read the circuit and channel being calibrated
//                          2) Set the offset for the channel being calibrated to 0.
//                          3) Set the hardware to the circuit being calibrated (AFE, ADC high gain, or ADC
//                             low gain)
//                          4) Initiate a user waveform capture.  A single cycle is captured.  Sum the 80
//                             samples - this is the DC offset for this cycle.
//                          5) Repeat this process 100 times, and keep a running total of the offset.
//                          6) When 100 cycles of offset has been accumulated, divide the result by
//                             (100 * 80 * gain) - 100 because 100 cycles, 80 because 80 samples.  The
//                             result is the DC offset per sample.
//                          7) Save the result in the offset for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      Note:
//                        ExAct.ValPtr2 points to the offset for the channel being calibrated
//                        ExAct.ValPtr1 points to the user waveform samples for the channel being calibrated
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 cycles
//                        ExAct.Temp holds the cycle count
//                      
//  CAVEATS:            The subroutine assumes the input to the channel (shorted input) has been properly
//                      set up.
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum()
// 
//------------------------------------------------------------------------------------------------------------
float tempp = 0;
void Cal_Offset_AFE(void)
{
  uint8_t i,j;
  switch (ExAct.SubState)
    {
      case CAL0:                        
        {       
          ExAct.channels_amount = 1;
          UseADCvals = FALSE; 
          if ( (ExAct.channel == 4) && (GF_USING_ROGOWSKI) )   // If Igsrc calibration and using a Rogowski coil, cal
          {                                        //   constant is held in index = 8
            ExAct.calPtrArray[0] = &AFEcal.offset[8];
            ExAct.mPtr = &UserSamples.OneCyc[4][0];
          }
          else if ( (ExAct.channel == 3) && (!IN_USING_ROGOWSKI) )  // If In calibration and using a CT, cal constant
          {                                             //   is held in index = 9
            ExAct.calPtrArray[0] = &AFEcal.offset[9];
            ExAct.mPtr = &UserSamples.OneCyc[3][0];
          }
          else if (ExAct.channel < 8)   
          {  
            ExAct.calPtrArray[0] = &AFEcal.offset[ExAct.channel];
            ExAct.mPtr = &UserSamples.OneCyc[ExAct.channel][0] ;     
          } 
          else if(ExAct.channel == 8)
          {
            //all currents
            for(i=0;i<3;i++)
            {
              ExAct.calPtrArray[i] = &AFEcal.offset[i];
            }
            float *channel_3 = (IN_USING_ROGOWSKI) ?  &AFEcal.offset[3] :  &AFEcal.offset[9];
            ExAct.calPtrArray[3] = channel_3;
            float *channel_4 = (GF_USING_ROGOWSKI) ?  &AFEcal.offset[8] :  &AFEcal.offset[4];
            ExAct.calPtrArray[4] = channel_4;
            
            ExAct.mPtr = &UserSamples.OneCyc[0][0];
            ExAct.channels_amount  = 5; 
          }
          else if (ExAct.channel == 9)
          {
            //all voltages
            for(i=0;i<3;i++)
            {
              ExAct.calPtrArray[i] = &AFEcal.offset[5 + i];
            }
            ExAct.mPtr = &UserSamples.OneCyc[5][0] ; 
            ExAct.channels_amount  = 3; 
          }
          else
          {
             //invalid for example GF cal but incorect GF setting
             ExAct.State = IDLE;
             break;
          }
          
          // Initialize offset for channel of interest to 0
          for(i=0; i<ExAct.channels_amount; i++)
          {
            *(ExAct.calPtrArray[i]) = 0.0;
          }
          ExAct.SubState = CAL1;
          ExAct.Temp = 0;
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f = 0;
          }
        }
        break;
  
      case CAL1:                        // Accumulate readings - AFE
        // Unlock the waveform capture process.  This may mess up some other process that is using a
        //   captured waveform, but we are calibrating, so it shouldn't matter.
        UserWF.Locked = FALSE;
        // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
        //   other process is using a captured waveform - this should never happen (see above)
        if (CaptureUserWaveform(USR_TYPE_ALL))
        {
          for(j = 0; j<ExAct.channels_amount; j++)
          {
            for (i=0; i<80; ++i)                           // Add all values to temporary sum.  The sum of all
            {                                              //   of the values should be the DC offset
              ExAct.Temp1[j].f += *(ExAct.mPtr + i + j*80); // 80 stands for size of UserSamples.OneCycycle[11][80]
            }
          }
          // If accumulated 100 readings, take average and set offset to the average (per sample average is
          // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
          // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 10)
          if (ExAct.Temp == 99)
          {
            for(i=0; i < ExAct.channels_amount; i++)
            {
              tempp= *(ExAct.calPtrArray[i] - 10);
              *(ExAct.calPtrArray[i]) = ExAct.Temp1[i].f/(8000.0f * tempp);
            }
                                                       // Compute and save new checksum and complement
            AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
            AFEcal.cmp = ~AFEcal.chk;
            UserWF.Locked = FALSE;                     // Free up user captures
            SPI1Flash.Req |= S1F_CAL_WR;               // Set flag to write the cal constants to Flash
            ExAct.State = WRCAL;                       // Set next state to write cal constants to FRAM
          }                                            //   and Flash   
          else                                         // If not done accumulating readings, increment the
          {                                            //   counter, capture present 10msec count,and go to
            ExAct.Temp++;                              //   the delay state.  We will initiate a new request
            ExAct.SubState++;                          //   in about 30msec.  Note, ExAct.Tmp2.b[0] is
            ExAct.Tmp2.b[0] = SysTickTime.cnt_10msec;  //   used to hold the present 10msec count
          }
        }
        break;

      case CAL2:                        // Wait a minimum of 30msec
        // Compute elapsed time since last capture.  We need to wait at least 30msec to ensure we get new
        //   samples for the next capture (50Hz --> 20msec, 10msec resolution)
        // The counter rolls over to 0 at 100 (0 - 99)
        i = ( (SysTickTime.cnt_10msec >= ExAct.Tmp2.b[0]) ?
                  (SysTickTime.cnt_10msec - ExAct.Tmp2.b[0]) : ((100 + SysTickTime.cnt_10msec) - ExAct.Tmp2.b[0]) );
        if (i > 4)                        // Make 40msec to be safe
        {
          ExAct.SubState--;
        }
        break;
  
      default:                            // Invalid state - this should never be entered
        break;
  
    }

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Offset_AFE()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Gain_HG()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           ADC High Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the ADC High Gain
//                      gain conversions.  The calibration is performed on a single channel.  Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated, along with the actual input
//                             current
//                          2) Set the hardware to the circuit being calibrated 
//                          3) Wait for a new reading.  When the flag is set, add the new reading to the
//                             running total in Tmp1.f.
//                          4) Repeat this process 100 times.
//                          5) When 100 readings have been accumulated, divide the result by 100 to get the
//                             average reading.
//                          6) Multiply the existing gain constant by the actual input divided by the
//                             average reading
//                          7) Save the result in the gain for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      The 200msec reading is used for AFE calibration because this is the reading that is
//                      used for metering.  
//                      Note:
//                        ExAct.ValPtr2 points to the gain for the channel being calibrated
//                        ExAct.ValPtr1 points to the 200msec (for the AFE) or the one-cycle (for the ADC)
//                              reading
//                        ExAct.Tmp2.f holds the actual input current (enterd by the user)
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 readings
//                        ExAct.Temp holds the cycle count
//                      Note:
//                        The ADC voltages are the same for both the high-gain and low-gain circuits.  In
//                        other words, the high gain and low-gain circuits only affect the currents.
//                        However, the high-gain and low-gain constants use the same storage structure, and
//                        are stored separately - the ADC voltage cal constants are kept in both the
//                        high-gain and the low-gain cal structures.
//                      
//  CAVEATS:            The subroutine assumes the input to the channel has been properly set up.
//                      Offset calibration should be performed first!!
//
//------------------------------------------------------------------------------------------------------------

void Cal_Gain_HG(void)
{
  uint8_t i;
  
  switch(ExAct.SubState)
  {
    case CAL0:
    {     
      ExAct.channels_amount = 1;
      if ( ((ExAct.channel == 3) || (ExAct.channel == 4)) && (!IN_USING_ROGOWSKI) )   // If calibrating In  and
      {                                                       //   using a CT, cal constant is stored
        ExAct.calPtrArray[0] = &ADCcalHigh.gain[4];                  //   at index = 4
        ExAct.mPtr = &CurOneCyc.In;
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else if (ExAct.channel < 5 )                            // Otherwise cal constant is stored at
      {                                                       //   the entered index
        ExAct.calPtrArray[0] = &ADCcalHigh.gain[ExAct.channel];
        ExAct.mPtr = &CurOneCyc.Ia + ExAct.channel;
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      
      else if (ExAct.channel < 8)                             // Otherwise VanADC - Vcn_ADC
      {
        ExAct.calPtrArray[0] = &ADCcalHigh.gain[ExAct.channel];
        ExAct.mPtr = &VolADCOneCyc.Van + (ExAct.channel - 5);
      }
      else if(ExAct.channel == 8)
      {
        //all current channels
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalHigh.gain[i];
        }
        ExAct.calPtrArray[3] = (IN_USING_ROGOWSKI) ?  &ADCcalHigh.gain[3] :  &ADCcalHigh.gain[4];
        
        ExAct.mPtr = &Cur200msFltr.Ia;
        ExAct.channels_amount  = 4;       
        
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else if (ExAct.channel == 9)
      {
        //all voltages
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalHigh.gain[5 + i];
        }
        ExAct.mPtr = &VolADCOneCyc.Van;
        ExAct.channels_amount = 3;
      }
      else
      {
        //invalid 
        ExAct.State = IDLE;
        break;
      }
      ExAct.SubState = CAL1;
      ExAct.Temp = 0;
      for(i=0; i<ExAct.channels_amount; i++)
      {
        ExAct.Temp1[i].f = 0;
      }
    }
    break;
    case CAL1:
    {
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        if (ExAct.Temp > 0)                    // Throw out the first reading since it may have samples from
        {                                   //   the AFE or ADC low-gain circuit.
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f += *(ExAct.mPtr +i);
          }           
        }
        if (ExAct.Temp == 100)                 // If accumulated 100 readings, compute average and set gain to
        {                                   //   gain * (ideal value)/(average value).
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f = ExAct.Temp1[i].f/100.0f;           
            *(ExAct.calPtrArray[i]) = (*(ExAct.calPtrArray[i]) * ExAct.target)/ExAct.Temp1[i].f; 
          }
                                                      // Compute and save new checksum and complement
          ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalHigh.cmp = ~ADCcalHigh.chk;
          SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
          ExAct.State = WRCAL;                         // Set next state to write cal constants to FRAM and Flash
        }                                               
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          ExAct.Temp++;
        }
      }

    }
    break;

    default:
      break;    

  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Gain_HG()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Offset_HG()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Offset Calibration HG
//
//  MECHANICS:          This subroutine handles offset calibration for ADC High Gain, Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated
//                          2) Set the offset for the channel being calibrated to 0.
//                          3) Set the hardware to the circuit being calibrated (AFE, ADC high gain, or ADC
//                             low gain)
//                          4) Initiate a user waveform capture.  A single cycle is captured.  Sum the 80
//                             samples - this is the DC offset for this cycle.
//                          5) Repeat this process 100 times, and keep a running total of the offset.
//                          6) When 100 cycles of offset has been accumulated, divide the result by
//                             (100 * 80 * gain) - 100 because 100 cycles, 80 because 80 samples.  The
//                             result is the DC offset per sample.
//                          7) Save the result in the offset for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      Note:
//                        ExAct.ValPtr2 points to the offset for the channel being calibrated
//                        ExAct.ValPtr1 points to the user waveform samples for the channel being calibrated
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 cycles
//                        ExAct.Temp holds the cycle count
//                      
//  CAVEATS:            The subroutine assumes the input to the channel (shorted input) has been properly
//                      set up.
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), CaptureUserWaveform()
// 
//------------------------------------------------------------------------------------------------------------

void Cal_Offset_HG(void)
{
  uint8_t i,j;
  switch (ExAct.SubState)
  {
    case CAL0:                        
    { 
      ExAct.channels_amount = 1;
      if((ExAct.channel == 3) || (ExAct.channel == 4))
      {
        if (!IN_USING_ROGOWSKI) //check if this sends right
        {
          ExAct.calPtrArray[0] = &ADCcalHigh.offset[4];
        }
        else
        {
          ExAct.calPtrArray[0] = &ADCcalHigh.offset[3];
        }
        ExAct.mPtr = &UserSamples.OneCyc[3][0];
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else if (ExAct.channel < 6)
      {
        ExAct.calPtrArray[0] = &ADCcalHigh.offset[ExAct.channel];
        ExAct.mPtr = &UserSamples.OneCyc[ExAct.channel][0];
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else if (ExAct.channel < 8)
      {
        ExAct.calPtrArray[0] = &ADCcalHigh.offset[ExAct.channel];
        ExAct.mPtr = &UserSamples.OneCyc[ExAct.channel + 3][0];
      }
      else if(ExAct.channel == 8)
      {
        //all currents
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalHigh.offset[i];
        }
        float *channel_3 = (IN_USING_ROGOWSKI) ?  &ADCcalHigh.offset[3] :  &ADCcalHigh.offset[4];
        ExAct.calPtrArray[3] = channel_3;
        
        ExAct.mPtr = &UserSamples.OneCyc[0][0];
        ExAct.channels_amount  = 4; 

        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else if (ExAct.channel == 9)
      {
        //all voltages
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalHigh.offset[5 + i];
        }
        ExAct.mPtr = &UserSamples.OneCyc[8][0];
        ExAct.channels_amount  = 3; 
        
        UseADCvals = TRUE;
        HIGH_GAIN_INPUTS;
      }
      else
      {
        ExAct.State = IDLE;
        break;
      }

      // Initialize offset for channel of interest to 0
      for(i=0; i<ExAct.channels_amount; i++)
      {
        *(ExAct.calPtrArray[i]) = 0.0;
      }
      ExAct.SubState = CAL1;
      ExAct.Temp = 0;
      for(i=0; i<ExAct.channels_amount; i++)
      {
        ExAct.Temp1[i].f = 0;
      }
    }
    break;

    case CAL1:  
      // Unlock the waveform capture process.  This may mess up some other process that is using a
      //   captured waveform, but we are calibrating, so it shouldn't matter.
      UserWF.Locked = FALSE;
      // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
      //   other process is using a captured waveform - this should never happen (see above)
      if (CaptureUserWaveform(USR_TYPE_ALL))
      {
        for(j = 0; j<ExAct.channels_amount; j++)
        {
          for (i=0; i<80; ++i)                           // Add all values to temporary sum.  The sum of all
          {                                              //   of the values should be the DC offset
            ExAct.Temp1[j].f += *(ExAct.mPtr + i + j*80); // 80 stands for size of UserSamples.OneCycycle[11][80]
          }
        }
        // If accumulated 100 readings, take average and set offset to the average (per sample average is
        // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
        // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 8)
        if (ExAct.Temp == 99)
        {
          for(i=0; i < ExAct.channels_amount; i++)
          {
            *(ExAct.calPtrArray[i]) = ExAct.Temp1[i].f/(8000.0f * *(ExAct.calPtrArray[i] - 8));//***ALG Does this calptrarray work like this? 
          }
                                                            // Compute and save new checksum and complement
          ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalHigh.cmp = ~ADCcalHigh.chk;
          UserWF.Locked = FALSE;                       // Free up user captures
          SPI1Flash.Req |= S1F_CAL_WR;                 // Set flag to write the cal constants to Flash
          ExAct.State = WRCAL;                         // Set next state to write cal constants to FRAM and Flash
        }                                                 
        else                                         // If not done accumulating readings, increment the
        {                                            //   counter, capture present 10msec count,and go to
          ExAct.Temp++;                              //   the delay state.  We will initiate a new request
          ExAct.SubState++;                          //   in about 30msec.  Note, ExAct.Tmp2.b[0] is
          ExAct.Tmp2.b[0] = SysTickTime.cnt_10msec;  //   used to hold the present 10msec count
        }
      }
      break;

      case CAL2:                        // Wait a minimum of 30msec
        // Compute elapsed time since last capture.  We need to wait at least 30msec to ensure we get new
        //   samples for the next capture (50Hz --> 20msec, 10msec resolution)
        // The counter rolls over to 0 at 100 (0 - 99)
        i = ( (SysTickTime.cnt_10msec >= ExAct.Tmp2.b[0]) ?
                  (SysTickTime.cnt_10msec - ExAct.Tmp2.b[0]) : ((100 + SysTickTime.cnt_10msec) - ExAct.Tmp2.b[0]) );
        if (i > 4)                        // Make 40msec to be safe
        {
          ExAct.SubState--;
        }
        break;

    default:                            // Invalid state - this should never be entered
      break;
  
  }

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Offset_HG()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Gain_LG()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           ADC Low Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the ADC Low Gain
//                      gain conversions.  The calibration is performed on a single channel.  Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated, along with the actual input
//                             current
//                          2) Set the hardware to the circuit being calibrated 
//                          3) Wait for a new reading.  When the flag is set, add the new reading to the
//                             running total in Tmp1.f.
//                          4) Repeat this process 100 times.
//                          5) When 100 readings have been accumulated, divide the result by 100 to get the
//                             average reading.
//                          6) Multiply the existing gain constant by the actual input divided by the
//                             average reading
//                          7) Save the result in the gain for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      The 200msec reading is used for AFE calibration because this is the reading that is
//                      used for metering.  
//                      Note:
//                        ExAct.ValPtr2 points to the gain for the channel being calibrated
//                        ExAct.ValPtr1 points to the 200msec (for the AFE) or the one-cycle (for the ADC)
//                              reading
//                        ExAct.Tmp2.f holds the actual input current (enterd by the user)
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 readings
//                        ExAct.Temp holds the cycle count
//                      Note:
//                        The ADC voltages are the same for both the high-gain and low-gain circuits.  In
//                        other words, the high gain and low-gain circuits only affect the currents.
//                        However, the high-gain and low-gain constants use the same storage structure, and
//                        are stored separately - the ADC voltage cal constants are kept in both the
//                        high-gain and the low-gain cal structures.
//                      
//  CAVEATS:            The subroutine assumes the input to the channel has been properly set up.
//                      Offset calibration should be performed first!!
//
//------------------------------------------------------------------------------------------------------------

void Cal_Gain_LG(void)
{
  uint8_t i;
  switch(ExAct.SubState)
  {
    case CAL0:
    {
      ExAct.channels_amount = 1;
      if ( ((ExAct.channel == 3) || (ExAct.channel == 4)) && (!IN_USING_ROGOWSKI) )   // If calibrating In (j=3 or 4) and
      {                                                       //   using a CT, cal constant is stored
        ExAct.calPtrArray[0] = &ADCcalLow.gain[4];                   //   at index = 4
        ExAct.mPtr = &CurOneCyc.In;
        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }
      else if (ExAct.channel < 5 )                            // Otherwise cal constant is stored at
      {                                                       //   the entered index
        ExAct.calPtrArray[0] = &ADCcalLow.gain[ExAct.channel];
        ExAct.mPtr = &CurOneCyc.Ia + ExAct.channel;
        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }      
      else if (ExAct.channel < 8)                             // Otherwise VanADC - Vcn_ADC
      {
        ExAct.calPtrArray[0] = &ADCcalLow.gain[ExAct.channel];
        ExAct.mPtr = &VolADCOneCyc.Van + (ExAct.channel - 5);
      }
      else if(ExAct.channel == 8)
      {
        //all current channels
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalLow.gain[i];
        }
        ExAct.calPtrArray[3] = (IN_USING_ROGOWSKI) ?  &ADCcalLow.gain[3] :  &ADCcalLow.gain[4];
        
        ExAct.mPtr = &CurOneCyc.Ia;
        ExAct.channels_amount  = 4; 
        
        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }
      else if (ExAct.channel == 9)
      {
        //all voltages
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalLow.gain[5 + i];
        }
        ExAct.mPtr = &VolADCOneCyc.Van;
        ExAct.channels_amount = 3;
      }
      else
      {
        //invalid for example GF cal but incorrect GF setting
        ExAct.State = IDLE;
        break;
      }
      ExAct.SubState = CAL1;
      ExAct.Temp = 0;
      for(i=0; i<ExAct.channels_amount; i++)
      {
        ExAct.Temp1[i].f = 0;
      }
    }
    break;
    case CAL1:
    {
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        if (ExAct.Temp > 0)                    // Throw out the first reading since it may have samples from
        {                                   //   the AFE or ADC low-gain circuit.
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f += *(ExAct.mPtr +i);
          }
        }
        if (ExAct.Temp == 100)                 // If accumulated 100 readings, compute average and set gain to
        {                                   //   gain * (ideal value)/(average value).
          for(i=0; i<ExAct.channels_amount; i++)
          {
            ExAct.Temp1[i].f = ExAct.Temp1[i].f/100.0f;           
            *(ExAct.calPtrArray[i]) = (*(ExAct.calPtrArray[i]) * ExAct.target)/ExAct.Temp1[i].f; 
          }
                                                      // Compute and save new checksum and complement
          ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalLow.cmp = ~ADCcalLow.chk;
          SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
          ExAct.State = WRCAL;                         // Set next state to write cal constants to FRAM and Flash
        }                                               
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          ExAct.Temp++;
        }
      }

    }
    break;

    default:
      break;    

  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Gain_LG()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Offset_LG()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Offset Calibration HG
//
//  MECHANICS:          This subroutine handles offset calibration for ADC Low Gain, Calibration
//                      consists of the following steps:
//                          1) Read the circuit and channel being calibrated
//                          2) Set the offset for the channel being calibrated to 0.
//                          3) Set the hardware to the circuit being calibrated (AFE, ADC high gain, or ADC
//                             low gain)
//                          4) Initiate a user waveform capture.  A single cycle is captured.  Sum the 80
//                             samples - this is the DC offset for this cycle.
//                          5) Repeat this process 100 times, and keep a running total of the offset.
//                          6) When 100 cycles of offset has been accumulated, divide the result by
//                             (100 * 80 * gain) - 100 because 100 cycles, 80 because 80 samples.  The
//                             result is the DC offset per sample.
//                          7) Save the result in the offset for this channel, and set the flag to write the
//                             result into FRAM and Flash
//                      Note:
//                        ExAct.ValPtr2 points to the offset for the channel being calibrated
//                        ExAct.ValPtr1 points to the user waveform samples for the channel being calibrated
//                        ExAct.Tmp1.f holds the running total of the offset for the 100 cycles
//                        ExAct.Temp holds the cycle count
//                      
//  CAVEATS:            The subroutine assumes the input to the channel (shorted input) has been properly
//                      set up.
//
//  INPUTS:             TP.SubState
// 
//  OUTPUTS:            TP.State
//
//  ALTERS:             TP.SubState
// 
//  CALLS:              TP_ParseChars(), TP_GetDecNum(), CaptureUserWaveform()
// 
//------------------------------------------------------------------------------------------------------------

void Cal_Offset_LG(void)
{
  uint8_t i,j;
  switch (ExAct.SubState)
  {
    case CAL0:                        
    { 
      ExAct.channels_amount = 1;
      if((ExAct.channel == 3) || (ExAct.channel == 4))
      {
        if (!IN_USING_ROGOWSKI) 
        {
          ExAct.calPtrArray[0] = &ADCcalLow.offset[4];
        }
        else
        {
          ExAct.calPtrArray[0] = &ADCcalLow.offset[3];
        }
        ExAct.mPtr = &UserSamples.OneCyc[3][0];
        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }
      else if (ExAct.channel < 5)
      {
        ExAct.calPtrArray[0] = &ADCcalLow.offset[ExAct.channel];
        ExAct.mPtr = &UserSamples.OneCyc[ExAct.channel][0];
        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }
      else if (ExAct.channel < 8)
      {
        ExAct.calPtrArray[0] = &ADCcalLow.offset[ExAct.channel];
        ExAct.mPtr = &UserSamples.OneCyc[ExAct.channel + 3][0];
      }
      else if(ExAct.channel == 8)
      {
        //all currents

        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalLow.offset[i];
        }
        float *channel_3 = (IN_USING_ROGOWSKI) ?  &ADCcalLow.offset[3] :  &ADCcalLow.offset[4];
        ExAct.calPtrArray[3] = channel_3;
        
        ExAct.mPtr = &UserSamples.OneCyc[0][0];
        ExAct.channels_amount  = 4; 

        UseADCvals = TRUE;
        LOW_GAIN_INPUTS;
      }
      else if (ExAct.channel == 9)
      {
        //all voltages
        for(i=0;i<3;i++)
        {
          ExAct.calPtrArray[i] = &ADCcalLow.offset[5 + i];
        }
        ExAct.mPtr = &UserSamples.OneCyc[8][0];
        ExAct.channels_amount  = 3; 

      }
      else
      {
        ExAct.State = IDLE;
        break;
      }
      
      // Initialize offset for channel of interest to 0
      for(i=0; i<ExAct.channels_amount; i++)
      {
        *(ExAct.calPtrArray[i]) = 0.0;
      }
      ExAct.SubState = CAL1;
      ExAct.Temp = 0;
      for(i=0; i<ExAct.channels_amount; i++)
      {
        ExAct.Temp1[i].f = 0;
      }
    }
    break;

    case CAL1:  
      // Unlock the waveform capture process.  This may mess up some other process that is using a
      //   captured waveform, but we are calibrating, so it shouldn't matter.
      UserWF.Locked = FALSE;
      // Perform a waveform capture.  True is returned if the capture is made.  If False is returned, some
      //   other process is using a captured waveform - this should never happen (see above)
      if (CaptureUserWaveform(USR_TYPE_ALL))
      {
        for(j = 0; j<ExAct.channels_amount; j++)
        {
          for (i=0; i<80; ++i)                           // Add all values to temporary sum.  The sum of all
          {                                              //   of the values should be the DC offset
            ExAct.Temp1[j].f += *(ExAct.mPtr + i + j*80); // 80 stands for size of UserSamples.OneCycycle[11][80]
          }
        }
        // If accumulated 100 readings, take average and set offset to the average (per sample average is
        // equal to sum/(80 samples * 100 readings) ) divided by the gain.  Note, offset is subtracted in
        // the scaling equation so don't negate it here.  Note, gain is referenced by *(TP.ValPtr2 - 8)
        if (ExAct.Temp == 99)
        {
          for(i=0; i < ExAct.channels_amount; i++)
          {
            *(ExAct.calPtrArray[i]) = ExAct.Temp1[i].f/(8000.0f * *(ExAct.calPtrArray[i] - 8));
          }
                                                            // Compute and save new checksum and complement
          ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
          ADCcalLow.cmp = ~ADCcalLow.chk;
          UserWF.Locked = FALSE;                       // Free up user captures
          SPI1Flash.Req |= S1F_CAL_WR;                 // Set flag to write the cal constants to Flash
          ExAct.State = WRCAL;                         // Set next state to write cal constants to FRAM and Flash
        }                                                 
        else                                         // If not done accumulating readings, increment the
        {                                            //   counter, capture present 10msec count,and go to
          ExAct.Temp++;                              //   the delay state.  We will initiate a new request
          ExAct.SubState++;                          //   in about 30msec.  Note, ExAct.Tmp2.b[0] is
          ExAct.Tmp2.b[0] = SysTickTime.cnt_10msec;  //   used to hold the present 10msec count
        }
      }
      break;

      case CAL2:                        // Wait a minimum of 30msec
        // Compute elapsed time since last capture.  We need to wait at least 30msec to ensure we get new
        //   samples for the next capture (50Hz --> 20msec, 10msec resolution)
        // The counter rolls over to 0 at 100 (0 - 99)
        i = ( (SysTickTime.cnt_10msec >= ExAct.Tmp2.b[0]) ?
                  (SysTickTime.cnt_10msec - ExAct.Tmp2.b[0]) : ((100 + SysTickTime.cnt_10msec) - ExAct.Tmp2.b[0]) );
        if (i > 4)                        // Make 40msec to be safe
        {
          ExAct.SubState--;
        }
        break;

    default:                            // Invalid state - this should never be entered
      break;
  
  }

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Offset_LG()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Offset_SI()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Offset Calibration
//
//  MECHANICS:          This subroutine handles the offset calibration for the on-board test injection
//                      circuit.  Basically, this subroutine finds the ADC code that produces zero current
//                      (where V_TEST+ and V_TEST- are equal).  Calibration consists of the following steps:
//                          1) Set the amplitude to 0 so will only use the DC outputs
//                          2) Enable the test injection circuit and set the output to the nominal value
//                          3) Enable the test injection output
//                          4) Read the one-cycle current (cannot use the 200msec current because the DC
//                             filter blocks the signal).  Repeat this 10 times and compute the average
//                             reading
//                          5) If the new reading is less than the original reading, increment the count and
//                             repeat step 4.  If the new reading is greater than the original reading, set
//                             a flag and decrement the count.
//                          6) Continue incrementing the count until the minimum value is found or the count
//                             reaches 3000 (error condition).
//                      Note:
//                        The midpoint only needs to be found using one channel - channel A is used.  Tests
//                        conducted on 160616 showed that it is the same regardless of the channel.  This
//                        makes sense, since the resolution of the DAC is significantly less than the offset
//                        error.
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//                      
// 
//------------------------------------------------------------------------------------------------------------

void Cal_Offset_SI(void)
{
  switch (ExAct.SubState)
  {
    case CAL0:                       // Set up for initial readings
      // Read the next parameter - channel to be calibrated.  This must be a decimal number:
      //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igsrc.    Save the channel number in Temp
      TestInj.Amplitude = 0;                  // Set amplitude to zero so will be DC output
      TestInj.MidPoint = 2375;                // Set the midpoint to the nominal value  *** DAH MAY WANT TO INCREASE THIS TO REDUCE CAL TIME AFTER WE GET MORE READINGS FROM OTHER BOARDS
      TSTINJ_PHA_OFF;                         // Make sure all of the test channels are off to start
      TSTINJ_PHB_OFF;                         //   with
      TSTINJ_PHC_OFF;
      TSTINJ_PHN_OFF;
      TSTINJ_DIS;
      TestInj.CosIndx = 20;                   // Initialize cosine index (COS[n] = SIN[n+20])
      TestInj.Channel = ExAct.channel;                    // Save channel
      ExAct.mPtr = &CurOneCyc.Ia + ExAct.channel;         // Initialize pointer to the current for the cal channel
      // Note, we are already in Manufacturing Mode.  This was checked in the Display Processor when the USB
      //   command was received
      // Turn off thermal memory so the bucket is cleared when we are done calibrating
      ExAct.Temp1[0].b[0] = Setpoints1.stp.ThermMem;      // Save thermal memory setpoint
      Setpoints1.stp.ThermMem = 0;
      TestInj.Flags |= TEST_INJ_INIT_ON;      // Set flag to initialize test inj.  Note, set this flag
                                              //   before setting flag to turn on test injection
      TestInj.Flags |= TEST_INJ_ON;           // Turn on test injection
      ExAct.Temp = 0;                        // Initialize count
      ExAct.Tmp2.f = 0;                          // Initialize sum of readings
      ExAct.Tmp1.f = 1E36;                       // Initialize initial sum of readings
      ExAct.SubState = CAL1;

      
      break;

    case CAL1:                       // Wait for settling time
    // Testing on 160526 showed that the analog front-end circuit and digital integrator takes about
    //   60 passes to settle from a cold start.  Wait 180 passes.
    if (SystemFlags & VALONECYC)            // Wait for a new reading...
    {
      ExAct.Temp++;
      if (ExAct.Temp >= 180)                   // If wait time is up, reset count and move on to get new
      {                                         //   readings                                          
        ExAct.Temp = 0;
        ExAct.SubState = CAL2;
      }
    }
    break;
      


    case CAL2:                       // Accumulate one-cycle AFE current readings
      if (SystemFlags & VALONECYC)          // Wait for a new reading...
      {
        ExAct.Tmp2.f += *ExAct.mPtr;               // Add the reading to a running total
        if (ExAct.Temp >= 9)                   // If accumulated 10 readings...
        {
          ExAct.Tmp2.f = ExAct.Tmp2.f/10.0;               // Compute the average current reading
          if (ExAct.Tmp2.f < ExAct.Tmp1.f)                // If new current is less than old current...
          {
            TestInj.MidPoint++;                         // Increment the midpoint
            if (TestInj.MidPoint > 2494)                // If midpoint reaches limit, error so done
            {
              ExAct.SubState = CAL3;
              TestInj.Flags |= TEST_INJ_INIT_OFF;           // Set flag to turn off test injection
            }
            else                                        // If no error, reset count, save new current, and
            {                                           //   repeat this step with the new midpoint
              ExAct.Temp = 120;                        // Since only changing one count, will only wait 60
              ExAct.Tmp1.f = ExAct.Tmp2.f;                    //   (180 - 120) passes.  Testing on 160526 showed
              ExAct.Tmp2.f = 0;                            //   it takes about 30 passes.
              ExAct.SubState = CAL1;
            }
          }
          else                                      // If new current is greater than the old current...
          {                                         //   set the midpoint back to the old value, compute
            TestInj.MidPoint--;                     //   checksum for the cal constants, write cal constants
                                                    //   to FRAM, set flag to write the constants to Flash,
            if (TestInj.Channel < 4)                //   turn off test injection, and set state to output
            {                                       //   the new midpoint
              TestInjCal.midpoint_ph = TestInj.MidPoint;
            }
            else
            {
              TestInjCal.midpoint_gnd = TestInj.MidPoint;
            }
            TestInjCal.chk = ComputeChksum32((uint32_t *)(&TestInjCal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));
            TestInjCal.cmp = ~TestInjCal.chk;

            // Write test injection cal constants to FRAM
            FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
            FRAM_Write(DEV_FRAM2, FRAM_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&TestInjCal.midpoint_ph));
            FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
            
            SPI1Flash.Req |= S1F_INJ_WR;                  // Set flag to write constants to Flash
            TestInj.Flags |= TEST_INJ_INIT_OFF;           // Set flag to turn off test injection
            ExAct.SubState = CAL3;
          }
        }
        else                                    // If not done accumulating readings, increment counter and
        {                                       //   remain in this state
          ExAct.Temp++;
        }
      }
      break;

    case CAL3:                       // Output the new midpoint value
      if ( (SPI1Flash.Ack & S1F_INJ_WR)             // If the FRAM and Flash writes are done and there are
        && (SystemFlags & VAL200MSEC) )             //   new readings (don't want to exit until all readings
      {                                             //   are back to normal), clear the request flag and
        SPI1Flash.Req &= (uint32_t)(~S1F_INJ_WR);   //   jump to the next state
        ExAct.SubState = CAL3;
      }
      break;

    case CAL4:                       // Wait For New 200msec Currents
      if (SystemFlags & VAL200MSEC)         // Wait once more for new 200msec currents.  This ensures all
      {                                     //   currents are back to 0 before enabling protection again
        Setpoints1.stp.ThermMem = ExAct.Temp1[0].b[0];  // Restore thermal memory setpoint
        ExAct.State = IDLE;
      }
      break;
        
    default:                            // Invalid state - this should never be entered
      ExAct.State = IDLE;
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Offset_SI()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Cal_Gain_SI()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Test Injection Sine Generator Gain Calibration
//
//  MECHANICS:          This subroutine handles gain calibration for the Test Injection Sine Wave Generator.
//                      Calibration consists of the following steps:
//                          1) Enable the test injection circuit and set the amplitude for 100
//                          2) Accumulate 100 readings an compute the average
//                          3) Repeat the process with an amplitude of 1600
//                          4) Compute the slope and y-intercept
//                          5) Set the flags to write the results into FRAM and Flash
//                      Note, interrupts do not need to be disabled when manipulating the test injection
//                      flags in the main loop, despite the fact that they are also altered in the sampling
//                      interrupt (in TestInj_Handler()).  The reason is that the the interrupt does not
//                      change any flags unless test injection is turned on (TEST_INJ_ON is set).  It then
//                      clears the init flag (TEST_INJ_INIT_ON) if it is set.  No other flags are changed
//                      until the main loop turns off test injection by setting TEST_INJ_INIT_OFF.
//                      It is important that the other flags are set up first, and that TEST_INJ_ON is set
//                      last!!
//                      
//  CAVEATS:            The subroutine assumes that offset calibration has already been performed, that the
//                      TA is not connected, and that no external currents are being input to the unit
//                      TEST_INJ_ON must be the last flag set (set the other flags before this one to ensure
//                      there are no conflicts with the interrupt) when turning on test injection!!
//
// 
//------------------------------------------------------------------------------------------------------------

void Cal_Gain_SI(void)
{

  switch (ExAct.SubState)
  {
    case CAL0:                       // Read step and channel to be calibrated - sine
    case TP_IDC0:                       // Read step and channel to be calibrated - dc
      // Read the next parameter - channel to be calibrated.  This must be a decimal number:
      //   0-Ia, 1-Ib, 2-Ic, 3-In, 4-Igsrc.    Save the channel number in Temp

      if (ExAct.target == 0)           // If sine calibration...
      {                                       // Set amplitude to 100 and the midpoint to the zero point
        TestInj.Amplitude = 100;
        TestInj.MidPoint = ((ExAct.channel < 4) ? (TestInjCal.midpoint_ph) : (TestInjCal.midpoint_gnd));
      }
      else                                  // If dc calibration...
      {                                       // Set amplitude to 0 and the midpoint to the first cal
        TestInj.Amplitude = 0;                //   point (2550)
        TestInj.MidPoint = 2550;
      }
      // When sine wave is used for test injection, the dc gain is reduced by a factor of 3.  This
      //   reduces the error, but limits the max current to 65KA.  The dc gain is also reduced when dc
      //   is used for test injection and the current is less than 65kA.  Therefore, set the flag here
      //   to reduce the dc gain here also so the dc gain is also reduced when calibrating
      TestInj.Flags |= TEST_INJ_CHANGE_A1;
      TSTINJ_PHA_OFF;                       // Make sure all of the test channels are off to start with
      TSTINJ_PHB_OFF;
      TSTINJ_PHC_OFF;
      TSTINJ_PHN_OFF;
      TSTINJ_DIS;
      TestInj.CosIndx = 20;                 // Initialize cosine index (COS[n] = SIN[n+20])
      TestInj.Channel = ExAct.channel;      // Save channel
      // Note, we are already in Manufacturing Mode.  This was checked in the Display Processor when the USB
      //   command was received
      // Turn off thermal memory so the bucket is cleared when we are done calibrating
      ExAct.Temp1[0].b[0] = Setpoints1.stp.ThermMem;   // Save thermal memory setpoint
      Setpoints1.stp.ThermMem = 0;
      TestInj.Flags |= TEST_INJ_INIT_ON;    // Set flag to initialize test inj.  Note, this flag must
                                            //   be set before setting the flag to turn on test
                                            //   injection
      TestInj.Flags |= TEST_INJ_ON;         // Turn on test injection
      ExAct.Temp = 0;                      // Initialize count.  NumChars is used as a temporary
      ExAct.Tmp1.f = 0;                        // Initialize sums of readings
      ExAct.Tmp2.f = 0;
      ExAct.mPtr = &CurOneCyc.Ia + ExAct.channel;       // Initialize pointer to the current for the cal channel
      ExAct.SubState++;

      break;

    case CAL1:                       // Wait for digital integrator to stabilize - sine
    case CAL3:

      if (SystemFlags & VAL200MSEC)         // Wait 10 seconds for the integrator to stabilize - this is
      {                                     //   50 200msec anniversaries
        if (++ExAct.Temp >= 50)            // Reinitialize counter and go to next state when wait period is
        {                                   //   over
          ExAct.Temp = 0;
          ExAct.SubState++;
        }
      }
      break;                                // If no 200msec anniversary, just break

    case CAL2:                       // Accumulate 100 readings - sine - amplitude = 100
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        ExAct.Tmp1.f += *ExAct.mPtr;
        if (ExAct.Temp >= 99)              // If accumulated 100 readings, compute average, save in Tmp1.f,
        {                                   //   set up for the next output point, reinitialize the counter
          ExAct.Tmp1.f = ExAct.Tmp1.f/100.0;      //   and summation registers, and go on to the next state
          if (ExAct.target == 0)           // If sine calibration...
          {
            TestInj.Amplitude = 1600;
          }
          else                                  // If dc calibration...
          {
            TestInj.MidPoint = 4000;
          }
          ExAct.Temp = 0;
          ExAct.Tmp2.f = 0;
          ExAct.SubState++;
        }
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          ExAct.Temp++;
        }
      }
      break;

    case CAL4:                       // Accumulate 100 readings - sine - amplitude = 1600
      if (SystemFlags & VALONECYC)          // If new reading, add to temporary sum
      {
        ExAct.Tmp2.f += *ExAct.mPtr;
        if (ExAct.Temp >= 99)              // If accumulated 100 readings, compute average, save in Tmp2.f,
        {                                   //   and calculate the gain and offset values using the formulas
          ExAct.Tmp2.f = ExAct.Tmp2.f/100.0;      //     m = 1500/(Tmp2.f-Tmp1.f),  b = 100-(m*Tmp1.f) for sine
          if (ExAct.target == 0)       //     m = 1450/(Tmp2.f-Tmp1.f),  b = 2550-(m*Tmp1.f) for dc
          {
            TestInjCal.m_sine[TestInj.Channel] = 1500.0/(ExAct.Tmp2.f - ExAct.Tmp1.f);
            TestInjCal.b_sine[TestInj.Channel] = 100.0 - (TestInjCal.m_sine[TestInj.Channel] * ExAct.Tmp1.f);
          }
          else
          {
            TestInjCal.m_dc[TestInj.Channel] = 1450.0/(ExAct.Tmp2.f - ExAct.Tmp1.f);
            TestInjCal.b_dc[TestInj.Channel] = 2550.0 - (TestInjCal.m_dc[TestInj.Channel] * ExAct.Tmp1.f);
          }
          TestInjCal.chk = ComputeChksum32((uint32_t *)(&TestInjCal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));
          TestInjCal.cmp = ~TestInjCal.chk;

          // Write test injection cal constants to FRAM
          FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
          FRAM_Write(DEV_FRAM2, FRAM_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&TestInjCal.midpoint_ph));
          FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection

          SPI1Flash.Req |= S1F_INJ_WR;          // Set flag to request write to Flash
          TestInj.Flags |= TEST_INJ_INIT_OFF;   // Set flag to turn off test injection
          ExAct.SubState++;                        // Set next state to wait for write completion
        }
        else                                // If not done accumulating readings, increment counter and
        {                                   //   remain in this state
          ExAct.Temp++;
        }
      }
      break;

    case CAL5:                       // Wait For FRAM and Flash Writes to Complete
      if ( (SPI1Flash.Ack & S1F_INJ_WR)             // If the FRAM and Flash writes are done and there are
        && (SystemFlags & VAL200MSEC) )             //   new readings (don't want to exit until all readings
      {                                             //   are back to normal), clear the request flag and
        SPI1Flash.Req &= (uint32_t)(~S1F_INJ_WR);   //   jump to the next state
        ExAct.SubState = CAL6;
      }
      break;

    case CAL6:                       // Wait For New 200msec Currents
      if (SystemFlags & VAL200MSEC)         // Wait once more for new 200msec currents.  This ensures all
      {                                     //   currents are back to 0 before enabling protection again
        Setpoints1.stp.ThermMem = ExAct.Temp1[0].b[0];  // Restore thermal memory setpoint
        ExAct.State = IDLE;
      }
      break;

    default:                            // Invalid state - this should never be entered
      ExAct.State = IDLE;
      break;

  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Cal_Gain_SI()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       WriteCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Calibration factors to FRAM
//
//  MECHANICS:          This subroutine writes calibrations factors to FRAM
//
//------------------------------------------------------------------------------------------------------------
uint8_t WriteCal(void)
{
  uint8_t writeFinished = 0;
  // Wait for Writes to Complete
  if (SPI1Flash.Ack & S1F_CAL_WR)       // If the Flash writes are done, clear the request flag, write
  {                                     //   the cal constants to FRAM, and jump to the cursor state
    SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
    // Write cal constants to FRAM
    FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
    FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
    FRAM_Write(DEV_FRAM2, FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));
    FRAM_Write(DEV_FRAM2, FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));
    FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
    writeFinished = 1;
  }
  return writeFinished;
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         WriteCal()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Write_Default_Cal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Default Calibration factors to FRAM
//
//  MECHANICS:          This subroutine writes default calibrations factors to FRAM
//
//------------------------------------------------------------------------------------------------------------
void Write_Default_Cal(uint16_t cal_type)
{
  uint8_t i;
  uint8_t restore_AFE = FALSE;
  uint8_t restore_ADC_High = FALSE;
  uint8_t restore_ADC_Low = FALSE;
  uint16_t rogo_cal[6];
  uint16_t vdb_line_cal[14];
  uint16_t vdb_load_cal[8];
  
  if(cal_type & BIT0)
  {
    for (i=0; i<5; i++)
    {
      AFEcal.gain[i] = AFE_CAL_DEFAULT_IGAIN;
      AFEcal.offset[i] = AFE_CAL_DEFAULT_IOFFSET;
      AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
    }
    for (i=0; i<12; i++)                        // Set all phase cal constants to default
    {
      AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
    }

    AFEcal.gain[8] = AFE_CAL_DEFAULT_IGAIN;     // For Rogowski Igsrc
    AFEcal.offset[8] = AFE_CAL_DEFAULT_IOFFSET;
    AFEcal.gain[9] = AFE_CAL_CT_IGAIN;          // For CT In
    AFEcal.offset[9] = AFE_CAL_DEFAULT_IOFFSET;

    restore_AFE = TRUE;

  }
  if(cal_type & BIT1)
  {
    for (i=5; i<8; i++)
    {
      AFEcal.gain[i] = AFE_CAL_DEFAULT_VGAIN;
      AFEcal.offset[i] = AFE_CAL_DEFAULT_VOFFSET;
    }
    for (i=0; i<12; i++)                        // Set all phase cal constants to default
    {
      AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
    }

    restore_AFE = TRUE;
  }
  if(cal_type & BIT2)
  {
    for (i=0; i<3; i++)
    {
      ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_IGAIN_HIGH;
      ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
    }
    ADCcalHigh.gain[4] = ADC_CAL_CT_IGAIN_HIGH;               // For CT In
    ADCcalHigh.offset[4] = ADC_CAL_CT_IOFFSET;

    restore_ADC_High = TRUE;
  }
  if(cal_type & BIT3)
  {
    for (i=0; i<3; i++)
    {
      ADCcalLow.gain[i] = ADC_CAL_DEFAULT_IGAIN_LOW;
      ADCcalLow.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
    }
    ADCcalLow.gain[4] = ADC_CAL_CT_IGAIN_LOW;                 // For CT In
    ADCcalLow.offset[4] = ADC_CAL_CT_IOFFSET;

    restore_ADC_Low = TRUE;
  }
  if(cal_type & BIT4)
  {
    for (i=5; i<8; i++)
    {
      ADCcalLow.gain[i] = ADC_CAL_DEFAULT_VGAIN;
      ADCcalLow.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
      ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_VGAIN;
      ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
    } 

    restore_ADC_High = TRUE;
    restore_ADC_Low = TRUE;
  }
  if(cal_type & BIT5)
  {
    for (i=0; i<sizeof(rogo_cal); i++)
    {
      rogo_cal[i] = ((uint16_t)1)  << 14;
    }

    rogo_cal[4] = Checksum8_16((uint8_t *)(&rogo_cal[0]), (sizeof(rogo_cal) - 4));
    rogo_cal[5] = (rogo_cal[4] ^ 0xFFFF);
    for (i = 0; i < 3; i++)
    {
      Frame_FRAM_Write(0x00 + i*sizeof(rogo_cal), sizeof(rogo_cal), (uint8_t *)(&rogo_cal[0]));
    }
    
  }
  if(cal_type & BIT6)
  {
    for (i=0; i<3; i++)
    {
      vdb_line_cal[i] = ((uint16_t)1)  << 14;
    }
    for (i=3; i<sizeof(vdb_line_cal); i++)
    {
      vdb_line_cal[i] = 0;
    }

    vdb_line_cal[12] = Checksum8_16((uint8_t *)(&vdb_line_cal[0]), (sizeof(vdb_line_cal) - 4));
    vdb_line_cal[13] = (vdb_line_cal[12] ^ 0xFFFF);
    for (i = 0; i < 3; i++)
    {
      Frame_FRAM_Write(0x812 + i*sizeof(vdb_line_cal), sizeof(vdb_line_cal), (uint8_t *)(&vdb_line_cal[0]));
    }
    
  }
  if(cal_type & BIT7)
  {
    for (i=0; i<3; i++)
    {
      vdb_load_cal[i] = ((uint16_t)1)  << 14;
    }
    for (i=3; i<sizeof(vdb_load_cal); i++)
    {
      vdb_load_cal[i] = 0;
    }

    vdb_load_cal[6] = Checksum8_16((uint8_t *)(&vdb_load_cal[0]), (sizeof(vdb_load_cal) - 4));
    vdb_load_cal[7] = (vdb_load_cal[6] ^ 0xFFFF);
    for (i = 0; i < 3; i++)
    {
      Frame_FRAM_Write(0x872 + i*sizeof(vdb_load_cal), sizeof(vdb_load_cal), (uint8_t *)(&vdb_load_cal[0]));
    }
    
  }

    

  if(restore_AFE)
  {
    AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
    AFEcal.cmp = ~AFEcal.chk;
    FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
  }
  if(restore_ADC_High)
  {
    ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
    ADCcalHigh.cmp = ~ADCcalHigh.chk;
    FRAM_Write(DEV_FRAM2, FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));
  }
  if(restore_ADC_Low)
  {
    ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
    ADCcalLow.cmp = ~ADCcalLow.chk;
    FRAM_Write(DEV_FRAM2, FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));
  }


}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Write_Default_Cal()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Service_LED_test()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Service LED Test
//
//  MECHANICS:          This subroutine updates the LED based on the test input:
//                      It truns OFF all LED and turns ON required LEDs
//
//
//-----------------------------------------------------------------------------------------------------
void Service_LED_test(void)
{
 
  STATUS_LED_OFF;
  ALARM_LED_OFF;
  HIGHLOAD_LED_OFF;
  TRIP_LED_OFF;
  LDPU_LED_OFF;
  L_LED_OFF;
  S_LED_OFF;
  I_LED_OFF;
  G_LED_OFF;

  if(ExAct.LED_image != NO_MANUF_TEST)
  {  
    if (ExAct.LED_image & BIT0)
      STATUS_LED_ON_GRN;

    if (ExAct.LED_image & BIT1)
      STATUS_LED_ON_RED;

    if (ExAct.LED_image & BIT2)
      ALARM_LED_ON;

    if (ExAct.LED_image & BIT3)
      HIGHLOAD_LED_ON;

    if (ExAct.LED_image & BIT4)
      TRIP_LED_ON;

    if (ExAct.LED_image & BIT5)
      LDPU_LED_ON;

    if (ExAct.LED_image & BIT6)
      L_LED_ON;

    if (ExAct.LED_image & BIT7)
      S_LED_ON;

    if (ExAct.LED_image & BIT8)
      I_LED_ON;

    if (ExAct.LED_image & BIT9)
      G_LED_ON;
  }
  
  ExAct.State = IDLE;

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Service_LED_test()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Service_Relay_test()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Service Relay Test
//
//  MECHANICS:          This subroutine updates the Relay based on the test input:
//                      It closes/opens all required Relays
//
//
//-----------------------------------------------------------------------------------------------------
void Service_Relay_test(void)
{
 
  if(ExAct.Relay_image != NO_MANUF_TEST)
  {  
    if (ExAct.Relay_image & BIT0)
    {
      RELAY1_CLOSE;
    }
    else
    {
      RELAY1_OPEN;
    }
    
    if (ExAct.Relay_image & BIT1)
    {
      RELAY2_CLOSE;
    }
    else
    {
      RELAY2_OPEN;
    }

    if (ExAct.Relay_image & BIT2)
    {
      RELAY3_CLOSE;
    }
    else
    {
      RELAY3_OPEN;
    }
  }
  
  ExAct.State = IDLE;

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Service_Relay_test()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       dhex_ascii()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Hex to ASCII Conversion Subroutine
//
//  MECHANICS:          This subroutine converts the hex number in num to two ASCII bytes as follows:
//                        num ms nibble --> result ms byte (ASCII encoded)
//                        num ls nibble --> result ls byte (ASCII encoded)
//                      
//  CAVEATS:            None
//
//  INPUTS:             num - the input byte to be converted
// 
//  OUTPUTS:            The ASCII-encoded value of num is return in a word, as described above.
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint16_t dhex_ascii(uint8_t num)
{
  uint16_t temp, temp1;

  temp = (num >> 4) & 0x000F;
  if (temp < 0x0A)
    temp = (temp + '0') * 256;
  else
    temp = (temp + 0x37) * 256;
  temp1 = (num & 0x000F);
  if (temp1 < 0x0A)
    temp += (temp1 + '0');
  else
    temp += (temp1 + 0x37);
  return (temp);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         dhex_ascii()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       asciidecimal_hex()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Decimal ASCII to Hex Conversion Subroutine
//
//  MECHANICS:          This subroutine converts the ASCII-encoded decimal number in TP.RxBuf[] to an
//                      unsigned hex number and returns that value.  Basically, this function is equivalent
//                      to:
//                          atoi((char const *)(&TP.RxBuf[TP.RxNdxOut])) ) & 0x01FF;
//                      This subroutine executes significantly faster, and has some additional validation
//                      checks.
//                      The subroutine parses the values in TP.RxBuf[] from 0 to end_indx, or until an
//                      invalid character (not an ASCII-encoded decimal number) is encountered
//                      
//  CAVEATS:            None
//
//  INPUTS:             end_indx - the termination index in TP.RxBuf[]
//                      TP.RxBuf[]
// 
//  OUTPUTS:            The ASCII-encoded value of num is return in a word, as described above.  If no valid
//                      number is found, the subroutine returns 0xFFFF
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint16_t asciidecimal_hex(uint8_t end_indx)
{
  uint8_t first, i, count;
  uint16_t val;


  val = 0;
  count = 0;
  i = 0;
  first = TRUE;

  // Parse the input buffer.  Skip characters until a valid decimal number is found.  Quit when either we
  // are out of characters, three characters have been parsed, or an invalid character is found
  while ( (i < end_indx) && (count < 3) )
  {
    if ( (TP.RxBuf[i] < 0x3A) && (TP.RxBuf[i] > 0x2F) )     // If valid character...
    {
      first = FALSE;                                            // Clear first time flag
      val = (val * 10) + (TP.RxBuf[i] - 0x30);                  // Update value with latest character
      ++count;                                                  // Increment the character count
    }
    else if (!first)                                        // If invalid character and we have already
    {                                                       //   received valid characters, done
      break;
    }
    ++i;                                                    // Update char index
  }

  if (!first)                                               // If at least one valid character was entered,
  {                                                         //   return the value.
    return (val);
  }
  else                                                      // Otherwise return an error code
  {
    return (0xFFFF);
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         asciidecimal_hex()
//------------------------------------------------------------------------------------------------------------

