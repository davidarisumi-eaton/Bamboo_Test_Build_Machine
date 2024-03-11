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
//  MODULE NAME:        Iod.c
//
//  MECHANICS:          Program module containing the I/O drivers and utilities subroutines
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
//   0.04   150928  DAH Deleted AFE_Process_Samples() and related variables (moved to Meter.c)
//   0.05   151021  DAH - Added FRAM_Read() and FRAM_Write()
//   0.06   151110  DAH - In FRAM_Write(), modified the CS deselect time delay (measured with scope)
//                      - Added RTC_Write(), RTC_Read(), SPI2_Manager(), ComputeChksum32(), and Flash_Read()
//                      - Added StoreAFEcalFlash()
//                      - Moved FRAM_Write() from global to local section
//                      - Added constants for Flash interface and for SPI2 manager
//   0.07   151216  DAH - Modified FRAM_Read() and FRAM_Write() to accomodate both the on-board FRAM and the
//                        Frame FRAM
//                      - Added support for reading and writing to the Frame EEPOT
//                          - Added struct Frame_EEPOT
//                          - Added EEPOT_Xfer() and EEPOT_Write()
//                          - Added states SM2_RD_EEPOT and SM2_WR_EEPOT to SPI2_Manager()
//                      - Deleted NUM_SPI2_REQUESTS (moved to Iod_def.h)
//   0.09   160224  DAH - In FRAM_Read() and FRAM_Write(), renamed FRAM_CSN_ACTIVE to FRAM2_CSN_ACTIVE and
//                        renamed FRAM_CSN_INACTIVE to FRAM2_CSN_INACTIVE
//                      - In SPI2_Manager(), renamed DEV_ONBOARD to DEV_FRAM2, and SM2_CAL_WRITE_FRAM to
//                        SM2_CAL_WRITE_FRAM2
//                      - Deleted EEPOT_Xfer(), EEPOT_Write(), FrameEEPOT, and EEPOT interface states from
//                        SPI2_Manager() as EEPOT is no longer used on the rev 2 boards
//                      - Added code to read the HIH6030 temperature/humidity sensor for rev 2 boards
//                          - Added SM2_RD_THSENSOR state to SPI2_Manager()
//                          - Added THSensorRd() to access the temperature/humidity sensor
//                          - Added struct TH_SENSOR THSensor
//   0.10   160310  DAH - Added SystemFlags, ReadAFECalConstants(), ReadADCHCalConstants(), and
//                        ReadADCLCalConstants()
//                      - Modified SPI2_Manager() to write the ADC cal constants to FRAM in addition to the
//                        AFE cal constants
//                      - Modified SPI2_Manager() to use constants for AFE and ADC cal constants parameters
//                        instead of hard-coded values
//                      - Added default state to SPI2_Manager()
//                      - Added FRAM_def.h
//                      - Modified StoreAFEcalFlash() to store the ADC cal constants in addition to the AFE
//                        cal constants
//                      - Added block write protection to the calibration constants in FRAM
//                          - FRAM_Stat_Write() added
//                          - SPI2_Manager() revised
//   0.12   160502  DAH - Corrected bug in StoreAFEcalFlash()
//                      - Renamed AFE_CAL_DEFAULT_GAIN to AFE_CAL_DEFAULT_IGAIN
//                      - Renamed AFE_CAL_DEFAULT_OFFSET to AFE_CAL_DEFAULT_IOFFSET
//                      - Modified ReadAFECalConstants() to use separate default constants for the current
//                        and voltage
//                      - Added phase calibration constants to ReadAFECalConstants()
//   0.13   160512  DAH - Corrected bugs in ReadADCHCalConstants() and ReadADCLCalConstants()
//                      - Corrected minor bugs in FRAM_Read() and FRAM_Write()
//                      - Added code to support writing energy to FRAM
//                          - Added FRAM_WriteEnergy() and FRAM_XferW()
//                          - Added state SM2_ENERGY_WRITE_FRAM2 to SPI2_Manager() to handle energy write
//                            requests
//                      - Added code to support reading energy from FRAM
//                          - Added FRAM_ReadEnergy() and FRAM_XferR()
//   0.14   160620  DAH - Re-added support for reading and writing to the Frame EEPOT
//                          - Added struct FrameEEPOT
//                          - Added EEPOT_Xfer() and EEPOT_Write()
//                          - Added states SM2_RD_EEPOT and SM2_WR_EEPOT to SPI2_Manager()
//                      - Added support for Test Injection
//                          - Added ReadInjCalConstants()
//                          - Added default test injection cal constants AFE_CAL_DEFAULT_OFFSET,
//                            AFE_CAL_DEFAULT_DCGAIN, AFE_CAL_DEFAULT_SINEGAIN
//                          - Added states SM2_INJ_WRITE_FRAM2 and SM2_INJ_WRITE_FLASH to SPI2_Manager() to
//                            handle requests to write Test Injection cal constants
//   0.15   160718  DAH - Deleted TP_LEDCode3 as it is not used
//                      - Added ReadSwitches() and struct COT_AUX to support reading the Clear LEDs
//                        pushbutton and the Breaker Closed switch
//                      - Added WriteCauseOfTripLEDS() and struct COT_AUX to support setting the Cause of
//                        Trip LEDs
//                      - Added support for waveform sample storage in the Black Box (SPI1) FRAM
//                          - Deleted FM25CL64 FRAM Instructions (moved to Iod_def.h)
//                          - Added BB_FRAM_Read()
//   0.16   160818  DAH - Added support for Startup Time measurement
//                          - Added StartUpADC, struct StartupTime
//                          - Added StartupTimeCal()
//                          - Added include of Prot_ext.h
//                      - Added support for thermal memory voltage ADC conversion
//                          - Added ThermalMemoryADC
//   0.17   161031  DAH - Revised RTC interface subroutines to support the switch from the DS1390 RTC to the
//                        MCP7951 RTC
//                          - Revised RTC_Write() and RTC_Read()
//   0.18   161115  DAH - Deleted THSensorRd() and added ReadTHSensor() as the temperature/humidity sensor
//                        was changed from the SPI bus to the I2C health bus.  Also deleted SM2_RD_THSENSOR
//                        state from SPI2_Manager()
//                      - Modified FRAM_Write() and FRAM_Read() to handle the 256k x 8 FRAM (3-byte
//                        address), which replaced the 32k x 8 FRAM (2-byte address)
//   0.19   170223  DAH - Revised WriteCauseOfTripLEDS() for the rev 3 hardware
//   0.22   180420  DAH - Added support for time-stamping with internal real time
//                          - Added struct SysTickTime
//                          - Added constant arrays Day_Offset_Reg[], Day_Offset_Leap[], Days_In_Period[]
//                          - Added Get_InternalTime(), RTC_to_SysTickTime(), and InternalTime_to_RTC()
//                          - Revised RTC_Read() to check the time for validity, return whether the time is
//                            valid, and to return the time values as hex numbers (not BCD)
//                          - Revised RTC_Write() for the new RTC chip (minor revisions)
//                          - Revised SPI2_Manager() to set the RTC_ERR in SystemFlags if RTC_Read() returns
//                            False 
//                          - Revised SPI2_Manager() to call InternalTime_to_RTC() instead of RTC_Write() so
//                            because the internal time is now written to the RTC instead of the values in
//                            RTC_buf[]
//                      - Added support for energy and demand logging
//                          - Added includes of Flash_def.h and Events_ext.h
//                          - struct EnergyAll was deleted.  The energy tally registers in this structure
//                            were moved into struct EngyDmnd.  EngyDmnd contains all of the variables used
//                            in demand logging, so that the structure can be written directly into FRAM and
//                            Flash.  FRAM_WriteEnergy() and FRAM_ReadEnergy() were revised to use the new
//                            structure
//                          - Added Flash_PageWrite() and Flash_EraseSector()
//                          - Deleted SSt26VF064B Flash Instructions (moved to Iod_def.h since used
//                            globally)
//                          - Add Dmnd_ForceLog and Dmnd_200ms_Per_SubInt_Adj
//                          - Added ProcessTimeAdjustment()
//                      - Corrected bug in State 4 of StoreAFEcalFlash() when setting the write block
//                        protection bits
//   0.23   180504  DAH - Moved Demand subroutines to Demand.c for readability
//                          - Removed Dmnd_ForceLog definition (moved to Demand.c)
//                          - Added Demand_def.h and Demand_ext.h includes
//                      - Revised ProcessTimeAdjustment() to eliminate computing Dmnd_200ms_Per_SubInt_Adj.
//                        The demand sub-intervals are no longer adjusted.  We are now using the real-time
//                        counter to determine the anniversary times
//                      - Removed Dmnd_200ms_Per_SubInt_Adj as it is no longer used
//                      - Corrected bug in InternalTime_to_RTC()
//                      - In SM2_CAL_WRITE_FLASH state of SPI2_Manager(), added FLASH_IN_USE (in
//                        SystemFlags) when writing to the Flash
//   0.24   180517  DAH - Deleted SPI2_Manager() and associated variables and constants.  All of the SPI2
//                        operations can be done immediately in the requesting subroutine, except for Flash
//                        accesses.  These need to be managed, because the Flash may be accessed for demand
//                        read/writes, setpoint read/writes, waveform read/writes, or cal constant
//                        read/writes.  The Flash management is handled in the calling subroutines using
//                        request and access flags (reference FRAM_Flash_def.h).
//                        Note, this SPI2 may be used this way provided it is not used with DAM or
//                        interrupts!  The longest write would be waveforms to Flash.  At 15MHz and 256
//                        bytes (the page size), this takes 136usec.  This is not worth using DMA.
//                          - Added FlashCtrl (flags word for Flash access control)
//                      - Renamed StoreAFEcalFlash() to StoreCalFlash() since it now is used to store AFE,
//                        ADC, and test injection calibration constants
//                      - Revised StoreCalFlash() to check whether the Flash is in use before proceeding and
//                        to handle both metering and test injection calibration constants.
//                      - Moved FRAM_Write(), FRAM_Stat_Write(), EEPOT_Write(), EEPOT_Xfer(),
//                        FRAM_WriteEnergy(), and StoreCalFlash() from the local definition section to the
//                        global definition section as these are now used globally
//                      - Revised FRAM_ReadEnergy() and FRAM_WriteEnergy() to work with the FM25V20 FRAM
//                        (they had been designed for the FM25V05).
//                      - Combined Flash_def.h and FRAM_def.h
//                          - Deleted include of Flash_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//                          - Revised StoreAFEcalFlash(), ReadAFECalConstants(), ReadADCHCalConstants(),
//                            ReadADCLCalConstants() with new Flash addresses
//                      - Modified Flash_PageWrite(), Flash_EraseSector(), and StoreCalFlash() to
//                        reinitialize SPI2 in each state that may be entered when the subroutine is
//                        invoked, because SPI2 may now be used in different threads (via the main loop)
//                        when Flash_PageWrite(), Flash_EraseSector(), and StoreCalFlash() are exited
//                      Added support for alarm waveform captures
//                          - Added struct Alarm_WF_Capture
//                          - Revised IO_VarInit() to initialize Alarm_WF_Capture.xxx
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Added SPI2_NoFlash_Manager() and struct S2NF.  These handle non-Flash SPI2
//                            access requests.  Modified IO_VarInit() to initialize the SPI_2 variables.
//                          - Added SPI2_Flash_Manager() and struct SPI2Flash.  These handle Flash SPI2
//                            access requests.  Modified IO_VarInit() to initialize the SPI2Flash variables.
//                          - Moved StoreCalFlash() and FRAM_WriteEnergy() from the global definition
//                            section to the local definition section as these are no longer used globally
//                          - Deleted FlashCtrl since it is no longer used
//                      - Fixed error in Get_InternalTime() (10msec counter was not converted correctly)
//                      - Fixed bug in Get_InternalTime() (SysTick->VAL could read as zero after rollover
//                        because the counter had not yet been reloaded with the initial count)
//                      - Added support for the startup time measurement function
//                          - Added ReadStartupScaleConstant()
//                          - Revised StartupTimeCal() to write the startup scaling constant to FRAM
//                      - Added support for Alarm waveform capture events
//                          - Added Flash_WriteWaveform()
//                          - Replaced Flash_EraseSector() with Flash_EraseSectorBlock().  The new routine
//                            is used to erase both sectors (for demand storage) and blocks (for waveform
//                            storage)
//                          - Modified IO_VarInit() to retrieve AlarmWF_Add from FRAM
//                      - Corrected bug in RTC_to_SysTickTime() (hundredths was incorrectly multiplied by
//                        10 before being saved in the 10msec count)
//   0.25   180627  DAH - In StartupTimeCal(), renamed SystemFlags GET_STARTUPTIME to FIRST_PASS
//                      - Added support for energy reads from FRAM
//                          - Added S2NF_ENERGY_READ_FRAM2 state to SPI2_NoFlash_Manager()
//                      - Moved FRAM_ReadEnergy() from the global section to the local section
//                      - Modified ReadAFECalConstants(), ReadADCHCalConstants(), ReadADCLCalConstants() to
//                        only read the Flash if the FRAM constants have a checksum error.  The Flash
//                        constants will be checked in the main loop.  This is done to reduce the execution
//                        time of the initialization code.
//                      - Added support for checking the Flash cal constants
//                          - Added S2F_CHECK_CAL_CONSTS state to SPI2_Flash_Manager()
//   0.26   180829  DAH - NUM_WAVEFORMS changed to NUM_ALARM_WAVEFORMS in IO_VarInit() (Flash memory mapping
//                        was modified)
//   0.27   181001  DAH - Added support for Trip waveform captures
//                          - Added structure Trip_WF_Capture
//                          - Revised IO_VarInit() to initialize Trip_WF_Capture.xxx
//                          - Added Trip waveform writes to S2F_NEXT_STATE[ ]
//                          - Revised S2F_ALARM_WF_WRITE state of SPI2_Flash_Manager() to handle aborting
//                            the alarm waveform capture if a trip capture is requested
//                          - Revised Flash_WriteWaveform() to handle Alarm, Trip, and Strip-Chart waveforms
//                          - Replaced AlarmWF_Add with Alarm_WF_Capture->FlashAdd
//                          - Added initialization of Trip_WF_Capture.FlashAdd to IO_VarInit()
//                      - Revised SPI2_Flash_Manager() so that all of the acknowledge bits are cleared if
//                        the corresponding request flag is clear
//                      - Changed the priority of the Flash requests for waveforms so that erase operations
//                        are higher than write operations.  The blocks must be erased before a new waveform
//                        is written, so erases should be a higher priority.
//                          - S2F_NEXT_STATE[ ] revised
//                      - Corrected bug in S2F_ALARM_WF_BLKERASE1 state of SPI2_Flash_Manager().
//                        S2F_ALARM_WF_WR was incorrectly set when the erase was completed.  This caused an
//                        additional unnecessary block erase to occur
//                      - Corrected bug in case 2 of Flash_WriteWaveform() when computing the FRAM address
//                        for the waveform header
//                      - Added support for Strip-Chart waveform captures
//                          - Added structure Chart_WF_Capture
//                          - Revised IO_VarInit() to initialize Chart_WF_Capture.xxx
//                          - Added Strip-Chart waveform writes to S2F_NEXT_STATE[ ]
//                          - Revised Flash_WriteWaveform() to handle Alarm, Trip, and Strip-Chart waveforms
//                          - Revised SPI2_Flash_Manager() to handle Strip-Chart waveform capture requests
//                            (S2F_CHART_WF_xxx states added)
//   0.30   190314  DAH - Added support to turn off Test Injection with the pushbutton switch
//                          - Added code to ReadSwitches() to set TEST_INJ_INIT_OFF flag in  TestInj.Flags
//                            if test injection is running and the Reset LEDs pushbutton is pressed
//   0.32   190726  DAH - Added code to write min and max demands into FRAM
//                          - Added FRAM_WriteMinMax()
//                          - Added S2NF_IDMND_WRITE and S2NF_PDMND_WRITE states to SPI2_NoFlash_Manager()
//                            to handle requests to write min and max demands into FRAM
//   0.35   191118  DAH - Size of ADCcalLow.gain[], ADCcalLow.offset[], ADCcalHigh.gain[], and
//                        ADCcalHigh.offset[] reduced from 10 to 8
//                          - Revised ReadADCHCalConstants() and ReadADCLCalConstants() accordingly
//                      - Added gain and offset constants for Rogowski Igsrc into struct AFEcal.  Added gain
//                        and offset constants for CT In into struct AFEcal
//                          - Revised ReadAFECalConstants() accordingly
//                      - Added gain and offset constants for CT In into struct ADCcalHigh and struct
//                        ADCcalLow
//                          - Revised ReadADCHCalConstants() and ReadADCLCalConstants() accordingly
//   0.37   200604  DAH - Added support for reading events (Read Delayed Requests) for display proc comms
//                          - Added SPI2_buf[]
//                          - Added S2NF_DP_REQUEST state to SPI2_NoFlash_Manager()
//                      - Began code to support reading waveforms for display processor communications
//                          - Added S2F_DP_READ_WF to S2F_NEXT_STATE[ ]
//                        This code may end up getting deleted
//                      - Moved events-related initialization out of IO_VarInit() (moved into
//                        Event_VarInit())
//                      - Revised Flash waveform storage algorithm to use an index to compute the Flash and
//                        FRAM addresses instead of the actual Flash address.  This is easier to manipulate,
//                        and provides commonality between the Flash and FRAM storage.
//                          - Added struct EV_ADD EV_Add to struct FLASH_INT_REQ
//                          - Revised Flash_WriteWaveform() to replace the Flash address with
//                            EV_Add.NextEvntNdx and EV_Add.Num_Events
//                          - Revised SPI2_Flash_Manager() to eliminate clearing the header information for
//                            waveforms when they are erased (not necessary), and added code to compute the
//                            Flash address from the index
//                          - In Flash_WriteWaveform(), for struct FLASH_INT_REQ, StartIndex was renamed
//                            SampleStartIndex so it is not confused with the Event storage index
//                      - Revised Flash waveform storage to only store the required header bytes in
//                        struct FLASH_INT_REQ instead of the entire structure.  The structure was modified
//                        to ensure there are no padding bytes.  This was done to reduce the read and write
//                        times for the header info, and to reduce the required FRAM storage space.
//                          - Revised Flash_WriteWaveform() and SPI2_NoFlash_Manager() (case S2NF_TP_READ)
//                            accordingly
//                      - The MasterEID storage in FRAM was altered to include a complement and second copy
//                          - Revised ProcessTimeAdjustment() and SPI2_NoFlash_Manager() (case
//                            S2NF_WRITE_EID) accordingly
//                      - The Startup Time Scale Constant storage in FRAM was altered.  The second copy was
//                        moved
//                          - Revised SPI2_NoFlash_Manager() (case S2NF_WRITE_STARTUP) and
//                            ReadStartupScaleConstant() accordingly
//                      - The waveform capture index and number of captures storage in FRAM was altered to
//                        include a complement and second copy
//                          - Modified Flash_WriteWaveform() accordingly
//                          - Added include of Events_def.h
//                      - Demand and Energy logging variables were revised to be compatible with the other
//                        event variables.  Added EV_Dmnd.Num_Entries to track how many demand and energy
//                        log entries were made and when the log has rolled over
//                          - Replaced Energy_Log_Add with EV_Dmnd.NextDmndAdd and EV_Dmnd.Num_Entries
//                          - Revised SPI2_Flash_Manager() to increment EV_Dmnd.Num_Entries when an entry is
//                            made and to reduce it when a sector is erased after it has rolled over
//                      - In S2F_TP_READ state of SPI2_Flash_Manager(), modified Flash address rollover
//                        check from "> ENERGY_SECTOR_END" to ">= ENERGY_SECTOR_END" because events
//                        are not stored in the ENERGY_SECTOR_END sector.  ENERGY_SECTOR_END marks the end
//                        of events storage
//   0.38   200708  DAH - Added support for writing setpoints (test purpose only) to Flash
//                          - Added test_setpoints[] to store the values written by the Display Processor
//                          - Added S2F_SETP_WRITE1, S2F_SETP_WRITE2, and S2F_SETP_WRITE3 states to
//                            SPI2_Flash_Manager().  These states erase the setpoints sector, then write an
//                            8-byte test code into the sector
//   0.41   201028  DAH - Added test code to retrieve setpoints for proc-proc comms
//                          - SPI2_NoFlash_Manager() (case S2NF_DP_REQUEST) and IO_VarInit() revised
//                          - Test variables MB_xx added
//   0.42   201202  DAH - Added Check_IntRTC(), IntRTC_Write(), and IntRTC_Read()to support the internal RTC
//                      - Revised RTC_to_SysTickTime() to work with the internal RTC and for more general
//                        use
//                      - In SPI2_NoFlash_Manager() deleted S2NF_RTC_READ and S2NF_RTC_WRITE states as the
//                        external RTC is no longer used
//                      - Deleted RTC_Read() and RTC_Write() as they are no longer used
//                      - Added InternalTimeRead() and IntTimeBuf() to support reading the internal time
//                      - Added IntRTC_Update(), RTC_State, and RTC_ShiftVal to adjust the
//                        RTC to internal time
//                      - Deleted InternalTime_to_RTC() as it has basically been replaced by calling
//                        InternalTimeRead() and IntRTC_Write() in IntRTC_Update()
//                      - RTC_buf[] size reduced from 18 to 12
//                      - Deleted RTC and Internal Time subroutines and associated variables (moved to
//                        RealTime.c).  Added include of RealTime_def.h
//   0.45   210504  DAH - Added S2F_INJ_READ_FLASH state to SPI2_Flash_Manager() to read the test injection
//                        constants
//   0.46   210521  DAH - Corrected bug in Flash_WriteWaveform().  The address in WF_Struct->FlashAdd needed
//                        to be shifted left 4 bits, so that it is in the format SSSP (sector address is in
//                        b15..b4, page address is in b3..0)
//                      - Corrected bug in SPI2_NoFlash_Manager().  In state S2NF_TP_READ, corrected the
//                        output buffer
//                      - Corrected bug in SPI2_Flash_Manager().  For Trip, Alarm, and Strip-Chart waveform
//                        block erases, the address in xxx.FlashAdd must be shifted left 4 bits, so that it
//                        is in the format SSSP (sector address is in b15..b4, page address is in b3..0)
//                      - Added code to Flash_WriteWaveform() to save the EID and increment it.  The code is
//                        temporary for the arc flash demo unit.  It should be reviewed and probably moved.
//                      - In state S2F_TP_READ in SPI2_Flash_Manager(), added code to support reading both
//                        voltages and currents.  Previous code only supported reading currents.
//   0.53   220325  DAH - Revised ReadTHSensor() to update both user and internal maximum temperatures
//   0.56   220526  DAH - Added include of RealTime_ext.h for call to Get_InternalTime() in ReadTHSensor()
//   0.58   220829  DAH - Frame_FRAM_Read() added to support reading setpoints from the Frame FRAM
//   0.59   220831  DAH - FRAM_ReadEnergy(), FRAM_WriteEnergy, and FRAM_WriteMinMax() moved from local to
//                        global section
//                      - SPI2_NoFlash_Manager() deleted as it is no longer used.  struct SPI2VARS S2NF
//                        deleted also
//                      - Revised StartupTimeCal() to replace code that set S2NF_WR_STARTUP with calls to
//                        FRAM_Write()
//                      - In Flash_WriteWaveform(), deleted the code that captured the EID and waveform
//                        header, and wrote the index and number of samples to FRAM.  This has all been
//                        moved to EventManager()
//                      - In ReadInjCalConstants(), deleted the code that reads the test injection cal
//                        constants from FRAM.  This has been moved to TP_Top()
//                      - Deleted test injection default cal constants (moved to Test.c)
//                      - In SPI2_Flash_Manager(), state S2F_INJ_WRITE_FLASH, deleted the code that writes
//                        the test injection cal constants to FRAM.  This has been moved to
//                        TP_TestInjOffsetCal() and TP_TestInjGainCal()
//                      - Revised StoreCalFlash(), Flash_PageWrite(), Flash_WriteWaveform(),
//                        Flash_EraseSectorBlock(), and Flash_Read() to use SPI1 instead of SPI2
//   0.60   220928  DAH - Revised FRAM_Read() to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//   0.62   221027  DAH - Deleted test code to write setpoints to Flash.  This is no longer needed.
//                        Setpoints are stored in Frame FRAM only
//                      - Deleted states to write setpoints to Flash from SPI2_Flash_Manager
//                      - Added Frame_FRAM_Write()
//   0.67   221209  DAH - Renamed SPI2_Flash_Manager() to SPI1_Flash_Manager() as the serial Flash is now on
//                        SPI1
//                      - Renamed SPI2_Flash_Manager_States to SPI1_Flash_Manager_States and state names
//                        from S2F_xx to S1F_xx
//                      - S2F_xx flags renamed to S1F_xx flags
//                      - Renamed struct SPI2VARS SPI2Flash to struct SPI1VARS SPI1Flash
//   0.68   230124  DB  - ExtCap_FRAM_Read and ExtCap_FRAM_Write added. These handle SPI2 writes/reads out
//                        in/from BB FRAM
//   0.69   230220  DAH - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - Chart_WF_Capture deleted
//                          - Flash_WriteWaveform(), SPI1_Flash_Manager() modified
//                          - Strip-chart waveform states removed from SPI1_Flash_Manager_States
//                          - Strip-chart waveform states replaced by extended-capture waveform states in
//                            S1F_NEXT_STATE[]
//                  DB  - Added FRAM_Clean()
//   0.72   230320  DAH - Revised  SPI1Flash.Ack and SPI1Flash.Req operation
//                      - In SPI1_Flash_Manager(), fixed bug in extended capture writes
//   0.72   230320  BP  - Added Service_NonCOT_Leds()
//                      - Turn off Protection Availability LEDs for John's EMC testing
//                      - Changed Cause of Trip LED order
//                      - Turn off Trip LED and reset trip flags when COT Reset button is pressed
//    24    230323  DAH - Fixed bug in SPI1_Flash_Manager()
//    25    230403  DAH - Revised SPI1_Flash_Manager() to support proc-proc comms read waveforms command
//                      - Added WF_ADDRESS[].  This was moved from Test.c
//                      - Added OpenFlg to ReadSwitches()
//    28    230406  DAH - In SPI1_Flash_Manager(), added code for the buffer length for waveform reads
//                        (state S1F_DP_READ_WF)
//                      - Added temporary code to clear events if reset switch pressed for 10 seconds
//                        This code will be removed after the demo
//                          - ReadSwitches() revised
//    29    230407  DAH - Corrected bug retrieving waveforms in S1F_DP_READ_WF state of SPI1_Flash_Manager()
//    36    230428  DAH - Revised SPI1_Flash_Manager() to make extended captures (almost) the same priority
//                        as alarm captures.  Alarm waveform requests will no longer abort an extended
//                        capture waveform capture
//    54    230802  BP  - Added the Maintenance Mode switch to ReadSwitches()and MM_Status_Update()
//    93    231010  BP  - Added Group 0 setpoint write when Maintenance Mode is changed
//    101   231027  DAH - Revised StartupTimeCal() to tighten reasonability check and add filter to the
//                        computed value.
//                      - Changed StartupTime.Scale default value in ReadStartupScaleConstant() from 2.06E-6
//                        to 2.60E-6
//   108    231108  DAH - Added include of Flags_def.h for BITxx definitions
//   116    231129  MAG - Modified MM_Status_Update() to removed commented out MM Comm_State code and change
//                        handling of bits 7/8 and setpoints save to match similar code in other functions
//   129    231213  MAG - Tweaked Maintenance Mode handling to match other occurrences.
//   133    231219  DAH - Eliminated code in ReadTHSensor() that sets temperature and humidity to large
//                        values if an error occurs.  This was causing temperature trips to occur.  The
//                        values are not updated if an error occurs.
//                      - Added code to initialize THSensor.xx so we don't generate a false trip
//   135    231221  DAH - Revised Flash_WriteWaveform() to fill waveform precycles with zero if they have
//                        not been written yet
//   141    20115   BP  - Added code to flash the Status LED red for an Energy fault
//   142    240119  DAH - Added code to support factory commands for changing calibration constants
//                          - Added ReadAFECalConstants1(), ReadADCHCalConstants1(), and
//                            ReadADCLCalConstants1()
//                          - Added S1F_READ_AFE_CALCONST1, S1F_READ_ADCH_CALCONST1, and
//                            S1F_READ_ADCL_CALCONST1 states to SPI1_Flash_Manager()
//                      - Revised Flash_Read_ID() to store value in union Flash_ID
//                      - Revised ProcessTimeAdjustment() to set SKIP_LOOPTIME_MEAS flag
//   144    240123  DAH - In ReadAFECalConstants1(), corrected bug checking the checksum when FRAM was read
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
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Init_def.h"
#include "Meter_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"
#include "Intr_def.h"
#include "DispComm_def.h"
#include "Prot_def.h"
#include "Flags_def.h"
#include "string.h"
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
#include "Test_ext.h"
#include "Init_ext.h"
#include "Meter_ext.h"
#include "Prot_ext.h"
#include "Events_ext.h"
#include "Demand_ext.h"
#include "Intr_ext.h"
#include "DispComm_ext.h"
#include "RealTime_ext.h"
#include "Setpnt_ext.h"
#include "can_tasks_ext.h"
#include "Modbus_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void IO_VarInit(void);
void Service_Status_Led(void);
void Service_NonCOT_Leds(void);

void ServiceRelays(uint16_t RelayConfig, uint8_t Relay);

uint16_t AFE_SPI_Xfer(uint16_t WrData);
void FRAM_Read(uint32_t fram_address, uint16_t length, uint16_t *outptr);
uint32_t ComputeChksum32(uint32_t *dptr, uint16_t len);
void Flash_Read(uint32_t flash_address, uint16_t length, uint16_t *outptr);
void Flash_Read_ID(void);

void ReadAFECalConstants(void);
void ReadADCHCalConstants(void);
void ReadADCLCalConstants(void);
void ReadInjCalConstants(void);
void ReadSwitches(uint8_t first_use);
void WriteCauseOfTripLEDS(void);
void ReadAFECalConstants1(uint8_t *dest);
void ReadADCHCalConstants1(uint8_t *dest);
void ReadADCLCalConstants1(uint8_t *dest);

void ExtCapt_FRAM_Read(uint16_t fram_address, uint16_t length, uint16_t *outptr);
void ExtCapt_FRAM_Write(uint32_t fram_address, uint16_t length, uint16_t *inptr);

void StartupTimeCal(void);

void ReadTHSensor(void);
void ReadTHSensorConfig(void);
void ReadTHSensorWriteReg(void);
void ReadTHSensorReadVal(void);
void ReadTHSensorProcValvoid(void);

uint8_t Flash_PageWrite(uint16_t flash_addr, uint8_t num_words, uint16_t *dataptr);
void ProcessTimeAdjustment(struct INTERNAL_TIME *old_time_ptr, struct INTERNAL_TIME *new_time_ptr);
void FRAM_Write(uint16_t device, uint32_t fram_address, uint16_t length, uint16_t *inptr);     // *** DAH  KEEP DEVICE PARAMETER IF WE WILL USE FOR BB_FRAM
void FRAM_Stat_Write(uint16_t device, uint8_t status);
uint8_t EEPOT_Write(void);
uint16_t EEPOT_Xfer(uint16_t WrData);
void SPI1_Flash_Manager(void);
void FRAM_Clean(uint16_t device, uint32_t fram_address, uint16_t length);
void Frame_FRAM_Read(uint16_t fram_address, uint16_t length, uint8_t *outptr);
void FRAM_ReadEnergy(uint8_t device);
void FRAM_WriteEnergy(uint8_t device);
void FRAM_WriteMinMax(uint32_t fram_address, uint16_t length, uint16_t *inptr);
void Frame_FRAM_Write(uint16_t fram_address, uint16_t length, uint8_t *inptr);
void MM_Status_Update(void);



//      Local Function Prototypes (These functions are called only within this module)
//
uint32_t FRAM_XferW(uint8_t len, uint16_t *dptr, uint32_t *chkptr);
uint32_t FRAM_XferR(uint8_t len, uint16_t *dptr, uint32_t *chkptr);
uint8_t StoreCalFlash(uint8_t mtr_cal);
void ReadStartupScaleConstant(void);
uint8_t Flash_WriteWaveform(struct FLASH_INT_REQ *WF_Struct, uint8_t WF_Abort);
uint8_t Flash_EraseBlock(uint16_t flash_addr);
uint8_t Flash_EraseSectorBlock(uint16_t flash_addr, uint8_t SectorErase);

void RelayManagement(void);
void InitRelays(void);
uint8_t IsRelayAssStpChanged(void);
uint8_t IsFlagAssChanged(void);


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
uint8_t StatusLED_BlinkCtr, HighLoadLED_BlinkCtr;
uint8_t TP_LEDCode;

struct SPI1VARS SPI1Flash;
uint8_t SPI2_buf[450];
struct TH_SENSOR THSensor;
uint16_t SystemFlags;
struct FLASH_INT_REQ Trip_WF_Capture, Alarm_WF_Capture, Ext_WF_Capture;
struct FRAME_EEPOT FrameEEPOT;
struct SWITCHES_PUSHBUTTONS COT_AUX;

uint16_t StartUpADC, ThermalMemoryADC;
struct STARTUP_TIME_VARS StartupTime;

struct HLTH_I2C_VARS HLTH_I2C;

union FLASH_ID_UNION Flash_ID;




//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t SCF_State;
uint8_t FPW_State;
uint8_t FSE_State;
uint8_t FSBE_State;
uint8_t FWW_State;
uint16_t SBout_Index;
uint16_t *SBout_ptr;

uint8_t MB_Addr;                     // *** DAH  ADDED FOR TEST
uint8_t MB_Par;
uint8_t MB_Stop;
uint16_t MB_Baud;

uint16_t sw_count;                  // *** DAH ADDED FOR DEMO TO RESET EVENTS!!
uint8_t oldstate;


uint8_t RelayStatusFlag;
uint32_t Relay1[2], Relay2[2], Relay3[2], Relay1Status[2], Relay2Status[2], Relay3Status[2];
uint32_t FlagTripAss, FlagTripAssStatus, RelayRegister[2];
uint64_t FlagAlarmAss, FlagAlarmAssStatus, FlagNoTripAlarmAss, FlagNoTripAlarmAssStatus;


//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//
// SPI1 interface
enum SPI1_Flash_Manager_States
{
  S1F_IDLE=0,
  S1F_TRIP_WF_BLKERASE,
  S1F_TRIP_WF_BLKERASE1,
  S1F_TRIP_WF_BLKERASE2,
  S1F_TRIP_WF_WRITE,
  S1F_ALARM_WF_BLKERASE,
  S1F_ALARM_WF_BLKERASE1,
  S1F_ALARM_WF_BLKERASE2,
  S1F_ALARM_WF_WRITE,
  S1F_EXT_WF_BLKERASE,
  S1F_EXT_WF_BLKERASE1,
  S1F_EXT_WF_BLKERASE2,
  S1F_EXT_WF_WRITE,
  S1F_DMND_WRITE1,
  S1F_DMND_WRITE2,
  S1F_DMND_WRITE3,
  S1F_CAL_WRITE_FLASH,
  S1F_DP_READ_WF,
  S1F_INJ_WRITE_FLASH,
  S1F_TP_READ,
  S1F_CLEAR_EVENTS,
  S1F_CHECK_CAL_CONSTS,
  S1F_INJ_READ_FLASH,
  S1F_READ_AFE_CALCONST,
  S1F_READ_ADCH_CALCONST,
  S1F_READ_ADCL_CALCONST,
  S1F_READ_AFE_CALCONST1,
  S1F_READ_ADCH_CALCONST1,
  S1F_READ_ADCL_CALCONST1
};


// Note, these must line up with the Flash access flags listed in Iod_def.h!!
const uint8_t S1F_NEXT_STATE[ ] = {S1F_TRIP_WF_BLKERASE,    S1F_TRIP_WF_BLKERASE2,  S1F_TRIP_WF_WRITE,
                                   S1F_ALARM_WF_BLKERASE,   S1F_ALARM_WF_BLKERASE2, S1F_ALARM_WF_WRITE,
                                   S1F_EXT_WF_BLKERASE,     S1F_EXT_WF_BLKERASE2,   S1F_EXT_WF_WRITE,
                                   S1F_DP_READ_WF,          S1F_CAL_WRITE_FLASH,    S1F_INJ_WRITE_FLASH,
                                   S1F_DMND_WRITE1,         S1F_DMND_WRITE2,        S1F_TP_READ,
                                   S1F_CHECK_CAL_CONSTS,    S1F_INJ_READ_FLASH,     S1F_READ_AFE_CALCONST,
                                   S1F_READ_ADCH_CALCONST,  S1F_READ_ADCL_CALCONST, S1F_READ_AFE_CALCONST1,
                                   S1F_READ_ADCH_CALCONST1, S1F_READ_ADCL_CALCONST1
                                  };


// Waveform Addressing - page address and offsets for the samples for each cycle in Flash
//                                 First Sample Page and              Last Sample Page and Offset    Cycle
//                                   Sample Set Offset                  Sample Set Offset
const uint16_t WF_ADDRESS[36][4] = {        0, 0,                             11, 2,               //  0
                                           11, 3,                             22, 5,               //  1
                                           22, 6,                             34, 1,               //  2
                                           34, 2,                             45, 4,               //  3
                                           45, 5,                             57, 0,               //  4
                                           57, 1,                             68, 3,               //  5
                                           68, 4,                             79, 6,               //  6
                                           80, 0,                             91, 2,               //  7
                                           91, 3,                            102, 5,               //  8
                                          102, 6,                            114, 1,               //  9
                                          114, 2,                            125, 4,               // 10
                                          125, 5,                            137, 0,               // 11
                                          137, 1,                            148, 3,               // 12
                                          148, 4,                            159, 6,               // 13
                                          160, 0,                            171, 2,               // 14
                                          171, 3,                            182, 5,               // 15
                                          182, 6,                            194, 1,               // 16
                                          194, 2,                            205, 4,               // 17
                                          205, 5,                            217, 0,               // 18
                                          217, 1,                            228, 3,               // 19
                                          228, 4,                            239, 6,               // 20
                                          240, 0,                            251, 2,               // 21
                                          251, 3,                            262, 5,               // 22
                                          262, 6,                            274, 1,               // 23
                                          274, 2,                            285, 4,               // 24
                                          285, 5,                            297, 0,               // 25
                                          297, 1,                            308, 3,               // 26
                                          308, 4,                            319, 6,               // 27
                                          320, 0,                            331, 2,               // 28
                                          331, 3,                            342, 5,               // 29
                                          342, 6,                            354, 1,               // 30
                                          354, 2,                            365, 4,               // 31
                                          365, 5,                            377, 0,               // 32
                                          377, 1,                            388, 3,               // 33
                                          388, 4,                            399, 6,               // 34
                                          400, 0,                            411, 2,               // 35
                                   };



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        IO_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           I/O Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the I/O Module.
//                      The following variables do not need initialization:
//
//  CAVEATS:            Call only during initialization.  This must be before Event_VarInit() because
//                      Event_VarInit() may add DMND_ERASE to SPI1Flash.Req.  This function assumes SPI2 is
//                      free to use!!
//
//  INPUTS:             None
//
//  OUTPUTS:            StatusLED_BlinkCtr, TP_LEDCode, SPI1Flash.xxx, SCF_State, SystemFlags, HLTH_I2C.xx,
//                      FrameEEPOT.State, StartupTime.xx, FPW_State, FSE_State, FSBE_State, FWW_State,
//                      HighLoadLED_BlinkCtr, THSensor.xx
//
//  ALTERS:             None
//
//  CALLS:              ReadStartupScaleConstant()
//
//  EXECUTION TIME:     Measured on 160625 (rev 0.25 code): 56.1usec    *** DAH  CAN REMEASURE SINCE SOME CODE REMOVED (SHOULD BE FASTER)
//
//------------------------------------------------------------------------------------------------------------

void IO_VarInit(void)
{
  StatusLED_BlinkCtr = 0;               // Initialize LED blink counter to 0
  HighLoadLED_BlinkCtr = 0;             // Initialize LED blink counter to 0
  TP_LEDCode = 0;                       // Initialize LED test inputs to 0
  SPI1Flash.State = S1F_IDLE;
  SPI1Flash.Req = 0;
  SPI1Flash.Ack = 0;
  SCF_State = 0;
  SystemFlags = 0;
  FrameEEPOT.State = 0;
  StartupTime.DoCalFlag = FALSE;
  StartupTime.State = 0;
  FPW_State = 0;
  FSE_State = 0;
  FSBE_State = 0;
  FWW_State = 0;

  HLTH_I2C.State = 0;
  HLTH_I2C.IntState = I2C3_IDLE;
  HLTH_I2C.RxNdx = 0;
  HLTH_I2C.Status = 0;
  HLTH_I2C.ErrCount = 0;
  HLTH_I2C.Flags = 0;
  memset (&HLTH_I2C.RxBuf[0], 0, HLTH_I2C_RXLEN);         // *** DAH  is it necessary to zero these out?
  memset (&HLTH_I2C.TxBuf[0], 0, HLTH_I2C_TXLEN);         // *** DAH  is it necessary to zero these out?

  THSensor.Humidity = 50;           // Initialize values to reasonable numbers so we don't generate a
  THSensor.Temperature = 25;        //   false trip due to garbage in the values

  ReadStartupScaleConstant();


  MB_Addr = 23;                     // *** DAH  ADDED FOR TEST
  MB_Par = 0;
  MB_Stop = 1;
  MB_Baud = 19200;

  
                                    // *** BP  TEMPORARY CODE
  // Turn off Protection Availability LEDs (for EMC testing per John Downs request)
  L_LED_OFF;
  S_LED_OFF;
  I_LED_OFF;
  G_LED_OFF;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          IO_VarInit()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Service_Status_Led()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Service Status LED
//
//  MECHANICS:          This subroutine updates the Status LED based on the following inputs:
//                          - The blink rate counter (StatusLED_BlinkCtr)
//                          - The Test LED input code (TP_LEDCode)
//                          - The system status flags (SystemFlags)
//
//  CAVEATS:            None
//
//  INPUTS:             StatusLED_BlinkCtr
// 
//  OUTPUTS:            None
//
//  ALTERS:             StatusLED_BlinkCtr
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Service_Status_Led(void)
{
  if (StatusLED_BlinkCtr == 0)               // If counter = 0, time to change state of LED (color and blink)
  {
    StatusLED_BlinkCtr = STATUS_OK_RATE;     // Initialize counter to ok rate
    switch (TP_LEDCode)                      // Determine LED color
    {
      case 0:                               // Not test mode
      default:
        STATUS_LED_TOGGLE_GRN;                  // Otherwise everything ok so flash ok green
        StatusLED_BlinkCtr = STATUS_OK_RATE;
        break;

      case 1:                               // Test code 1: LED off
        STATUS_LED_OFF;
        StatusLED_BlinkCtr = STATUS_OK_RATE;
        break;

      case 2:                               // Test code 2: STATUS LED = fast red
        STATUS_LED_TOGGLE_RED;
        StatusLED_BlinkCtr = STATUS_ERR_RATE;
        break;
    }

  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Service_Status_Led()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Service_NonCOT_Leds
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Service the Non-Cause-of-Trip LEDs
//
//  MECHANICS:          This subroutine updates all LEDs except the Status LED and Cause-of-Trip LEDs.
//
//                      The High Load LED is on solid for a High Load1 alarm and blinks for a High Load2 alarm.
//                      The LDPU Alarm LED is on when we are above Long Delay Pickup and off when below.
//                      The Trip LED is on for any trip.  It is turned off by pressing the Reset button
//
//                      Add LDPU, Alarm, and Trip LED's
//
//  CAVEATS:            None
//
//  INPUTS:             HighLoadLED_BlinkCtr, HlAlm1Flg, HlAlm2Flg, LdPuAlmFlg
// 
//  OUTPUTS:            None
//
//  ALTERS:             HighLoadLED_BlinkCtr
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Service_NonCOT_Leds(void)
{
  
  // High Load LED
  if (HlAlm2Flg == 1)                            // Blink the High Load LED for High Load Alarm2
  {
    if (HighLoadLED_BlinkCtr == 0)               // If counter = 0, time to blink
    {
      HighLoadLED_BlinkCtr = HIGHLOAD_BLINK_RATE;     // Initialize counter to blink rate
      HIGHLOAD_LED_TOGGLE;
    }
  }  
  else if (HlAlm1Flg == 1)                       // Turn on High Load LED solid for High Load Alarm1
  {
    HIGHLOAD_LED_ON;
    HighLoadLED_BlinkCtr = 0;
  }
  else                                           // Turn off High Load LED
  {
    HIGHLOAD_LED_OFF;
    HighLoadLED_BlinkCtr = 0;
  }
  
  
  // LDPU Alarm LED
  if (LdPuAlmFlg == 1)
  {
    LDPU_LED_ON;
  }
  else
  {
    LDPU_LED_OFF;    
  } 
  
  // Trip LED
  if (((Trip_Flags0.all & TRIP_MASK_FLG0) != 0) || ((Trip_Flags1.all & TRIP_MASK_FLG1) != 0))
  {
    TRIP_LED_ON;
  }

  // Alarm LED
  if (((Alarm_Flags0.all & ALARM_MASK_FLG0) != 0) || ((Alarm_Flags1.all & ALARM_MASK_FLG1) != 0) || ((Alarm_Flags2.all & ALARM_MASK_FLG2) != 0))
  {
    ALARM_LED_ON;
  }
    else
  {
    ALARM_LED_OFF;    
  } 
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Service_NonCOT_Leds()
//------------------------------------------------------------------------------------------------------------

//[]
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       InitRelays()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Relay Init
//
//  MECHANICS:          This subroutine sets all to zero
// 
//  INPUTS:             none

//  CALLS:              none
// 
//------------------------------------------------------------------------------------------------------------


void InitRelays(void)
{
  
  Relay1[0] = 0;
  Relay1[1] = 0;
  Relay2[0] = 0;
  Relay2[1] = 0;
  Relay3[0] = 0;
  Relay3[1] = 0;

  Relay1Status[0] = 0;
  Relay1Status[1] = 0;
  Relay2Status[0] = 0;
  Relay2Status[1] = 0;
  Relay3Status[0] = 0;
  Relay3Status[1] = 0;


  FlagTripAss = 0;
  FlagAlarmAss = 0;
  FlagAlarmAssStatus = 0;
  FlagTripAssStatus = 0;
  
  RelayStatusFlag = 0;

  RelayRegister[0] = 0;
  RelayRegister[1] = 0;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         InitRelays()
//------------------------------------------------------------------------------------------------------------


//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION       ActivateRelay
//------------------------------------------------------------------------------
// routine used to Activate/CLOSE Relay status
// ------------------------------------------------------------------------------
void ActivateRelay(uint8_t Relay)
{
    switch(Relay)
    {
      case 1:
        RELAY1_CLOSE;
      break;
      case 2:
        RELAY2_CLOSE;
      break;
      case 3:
        RELAY3_CLOSE;
      break;
    }
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         ActivateRelay
//------------------------------------------------------------------------------/

//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION       DeactivateRelay
//------------------------------------------------------------------------------
// routine used to DeActivate/OPEN Relay status
// ------------------------------------------------------------------------------

void DeactivateRelay(uint8_t Relay)
{
    switch(Relay)
    {
      case 1:
        RELAY1_OPEN;
      break;
      case 2:
        RELAY2_OPEN;
      break;
      case 3:
        RELAY3_OPEN;
      break;
    }
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         DeactivateRelay
//------------------------------------------------------------------------------/


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ServiceRelays()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Service Relays
//
//  MECHANICS:          This subroutine updates status of Relays according to trip/Alarm Flags and Relay Assignments
// 
//  INPUTS:             RelayConfig, Relay, State

//  CALLS:              ActivateRelay, DeactivateRelay
// 
//------------------------------------------------------------------------------------------------------------

// not implemented
//DIR_SHORT_CIRC
//INTERNAL_ALARM
//SYNC_CHECK

#define RELAY_MANAGEMENT  ActivateRelay(Relay):DeactivateRelay(Relay)

void ServiceRelays(uint16_t RelayStpConfig, uint8_t Relay)
{
  uint8_t State = 0;
  
  if(RelayStpConfig == OVERLOAD_TRIP)
  {
    ((LdTripFlg || TempTripFlg) && !NeuTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == NEUTRAL_TRIP)
  {
    (NeuTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == SHORT_DELAY_TRIP)
  {
    (SdTripFlg && !NeuTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == SHORT_CIRC_TRIP)
  {
//  if((SdTripFlg||InstTripFlg||HinstTripFlg) && !NeuTripFlg)
  ((SdTripFlg||InstTripFlg) && !NeuTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == INSTANT_TRIP)
  {
//        if((InstTripFlg || McrHiTripFlg) && !NeuTripFlg)
    (InstTripFlg && !NeuTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == GROUND_TRIP)
  {
    (GndTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == OVER_FREQ)
  {
    (OfTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == UNDER_FREQ)
  {
    (UfAlmFlg)?RELAY_MANAGEMENT;
  }       
  else if(RelayStpConfig == MMODE_TRIP)
  {
    (MM_TripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == ALL_TRIPS)
  {
    (BellTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == OV_ALARM && OvAlmFlg)
  {
    (OvAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == UV_ALARM && UvAlmFlg)
  {
    (UvAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == VOLTAGE_UNBALANCE_ALARM)
  {
    (VoltUnbalAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == CURRENT_UNBALANCE_ALARM)
  {
    (CurrUnbAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == HIGH_LOAD1_ALARM)
  {
    (HlAlm1Flg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == HIGH_LOAD2_ALARM)
  {
    (HlAlm2Flg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == HIGH_TEMP_ALARM)
  {
    (TempAlmFlg)?RELAY_MANAGEMENT;
  }               
  else if(RelayStpConfig == GND_FAULT_PREALARM)
  {
    (GfPreAlarmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == PHASE_LOSS_ALARM)
  {
    (PhaseLossAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == OVER_FREQ_ALARM)
  {
    (OfAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == UNDER_FREQ_ALARM)
  {
    (UfAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REVERS_ACTIVE_PWR_ALARM)
  {
    (RevReacPwrAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == THERMAL_MEMORY_ALARM)
  {
    (TherMemAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == WTCHDOG_AUX_PWR_ALARM)
  {
    (WdgFault)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == LOW_BATTERY_ALARM)
  {
    (LowBatAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == FAULT_PRESENT_ALARM)
  {
//        if(ETUBrkrConfigFault||BRKBrkrConfigFault||ETUCalFault||RtcFault||MaxMinFault||EnergyFault||SecondaryInj_CalFault||FrameModuleBrkrConfigFault||RogoCalFault||BadtaFlg)
    (RtcFault||MaxMinFault||EnergyFault)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == STP_MISMATCH_ALARM)
  {
    (StpFault)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == HEALTH_WARNING_ALARM)
  {
    (BrkHealthAlmFlg)?RELAY_MANAGEMENT;
  }
//  else if(RelayStpConfig == COMM_FAULT_ALARM)
//  {
//      if(ExternCommError_Flg)
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
  else if(RelayStpConfig == ALL_ALARM)
  {
//        if(ETUBrkrConfigFault||BRKBrkrConfigFault||ETUCalFault||RtcFault||MaxMinFault||EnergyFault||SecondaryInj_CalFault||FrameModuleBrkrConfigFault||RogoCalFault||BadtaFlg ||StpFault||BrkHealthAlmFlg||ExternCommError_Flg)
    (RtcFault||MaxMinFault||EnergyFault||StpFault||BrkHealthAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == THD_DISTORTION)
  {
    (THDAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == AUX_CONTACT)
  {
    (OpenFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == BELL_CONTACT)
  {
    if(BellTripFlg&&OpenFlg)
        ActivateRelay(Relay);
    else if (!BellTripFlg)
        DeactivateRelay(Relay);
  }       
  else if(RelayStpConfig == MM_ACTIVE)
  {
    (MM_Active)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == ZSI_ACTIVE)
  {
    (Setpoints1.stp.ZSI == 1)?RELAY_MANAGEMENT;
  }
//  else if(RelayStpConfig == ZSI_INPUT_RECEIVED)
//  {
//      if(ZSIRcvFlg)
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
//  else if(RelayStpConfig == ZSI_OUTPUT_SENT)
//  {
//      if(ZSISentFlg)
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
//  else if(RelayStpConfig == OPEN_BREAKER_PULSED)
//  {
//      if(Relay_PulseOpen_Flg)
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
//  else if(RelayStpConfig == CLOSE_BREAKER_PULSED)
//  {
//      if(Relay_PulseClose_Flg)
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
//  else if(RelayStpConfig == REMOTE_CONTROL)
//  {
//      if(((Relay1Output_Flg)&&(Relay==1))
//          ||((Relay2Output_Flg)&&(Relay==2))
//          ||((Relay3Output_Flg)&&(Relay==3)))
//          ActivateRelay(Relay);
//      else
//          DeactivateRelay(Relay);
//  }
  else if(RelayStpConfig == REMOTE_CONTROL && (Relay ==  1))
  {
    (RemoteCtrlFlag1)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REMOTE_CONTROL && (Relay == 2))
  {
    (RemoteCtrlFlag2)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REMOTE_CONTROL && (Relay == 3))
  {
    (RemoteCtrlFlag3)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == MECHANISM_TRIP)
  {
    (SneakersTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == PHASE_ROTATION)
  {
    (RevSeqTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == OVER_VOLTAGE)
  {
    (OvTripFlg || OvAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == UNDER_VOLTAGE)
  {
    (UvTripFlg || UvAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == VOLTAGE_UNBALANCE)
  {
    (VoltUnbalTripFlg || VoltUnbalAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == CURRENT_UNBALANCE)
  {
    (CurrUnbTripFlg || CurrUnbAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == PHASE_ROTATION_ALARM)
  {
    (RevSeqAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REVERS_ACTIVE_PWR)
  {
    (RevPwrTripFlg || RevPwrAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REV_REACT_PWR)
  {
    (RevReacPwrTripFlg || RevReacPwrAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REAL_PWR)
  {
    (RealPwrAlmFlg || RealPwrTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == REACT_PWR)
  {
    (ReacPwrTripFlg || ReacPwrAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == APP_PWR)
  {
    (AppPwrAlmFlg || AppPwrTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == UNDER_PWR)
  {
    (PFAlmFlg || PFTripFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == PWR_DEMAND)
  {
    (KWDmdAlmFlg || KVADmdAlmFlg)?RELAY_MANAGEMENT;
  }
  else if(RelayStpConfig == PHASE_LOSS)
  {
    (PhaseLossTripFlg || PhaseLossAlmFlg)?RELAY_MANAGEMENT;
  }
//    else if(RelayStpConfig == AQD_PREWRN_ALARM)
//    {
//        if (AQDPreWarAlmFlg)
//        {
//            ActivateRelay(Relay);
//        }
//        else
//        {
//            DeactivateRelay(Relay);
//        }
//    }
}


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ServiceRelays()
//------------------------------------------------------------------------------------------------------------


//[]
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       IsRelayAssChanged()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           IsRelayAssChanged
//
//  MECHANICS:          This subroutine checks whether the setpoints group 12 has been changed
// 
//  INPUTS:             none
//  
//  OUTPUTS:            1 if changed, 0 default not changed
//
//  CALLS:              ActivateRelay, DeactivateRelay
// 
//------------------------------------------------------------------------------------------------------------


uint8_t IsRelayAssStpChanged()
{
  uint8_t RelayChange = 0;

  Relay1[0] = (uint32_t) Setpoints12.stp.Relay1_Cfg1 | (uint32_t) Setpoints12.stp.Relay1_Cfg2 << 16;
  Relay1[1] = (uint32_t) Setpoints12.stp.Relay1_Cfg3 | (uint32_t) Setpoints12.stp.Relay1_Cfg4 << 16;

  Relay2[0] = (uint32_t) Setpoints12.stp.Relay2_Cfg1 | (uint32_t) Setpoints12.stp.Relay2_Cfg2 << 16;
  Relay2[1] = (uint32_t) Setpoints12.stp.Relay2_Cfg3 | (uint32_t) Setpoints12.stp.Relay2_Cfg4 << 16;

  Relay3[0] = (uint32_t) Setpoints12.stp.Relay3_Cfg1 | (uint32_t) Setpoints12.stp.Relay3_Cfg2 << 16;
  Relay3[1] = (uint32_t) Setpoints12.stp.Relay3_Cfg3 | (uint32_t) Setpoints12.stp.Relay3_Cfg4 << 16;

  if (Relay1Status[0] ^ Relay1[0])
  {
    RelayChange = 1;
    Relay1Status[0] = Relay1[0];
  }
  
  if (Relay1Status[1] ^ Relay1[1])
  {
    RelayChange = 1;
    Relay1Status[1] = Relay1[1];
  }

  if (Relay2Status[0] ^ Relay2[0])
  {
    RelayChange = 2;
    Relay2Status[0] = Relay2[0];
  }
  
  if (Relay2Status[1] ^ Relay2[1])
  {
    RelayChange = 2;
    Relay2Status[1] = Relay2[1];
  }

  if (Relay3Status[0] ^ Relay3[0])
  {
    RelayChange = 3;
    Relay3Status[0] = Relay3[0];
  }
  
  if (Relay3Status[1] ^ Relay3[1])
  {
    RelayChange = 3;
    Relay3Status[1] = Relay3[1];
  }

  return RelayChange;
  
}

//------------------------------------------------------------------------------
//  END OF FUNCTION         IsRelayAssChanged
//------------------------------------------------------------------------------

//[]
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       IsRelayAssChanged()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           IsFlagAssChanged
//
//  MECHANICS:          This subroutine checks whether the Flags of Trips/Alarams have been changed
// 
//  INPUTS:             none
//  
//  OUTPUTS:            1 if changed, 0 default not changed
//
//  CALLS:              ActivateRelay, DeactivateRelay
// 
//------------------------------------------------------------------------------------------------------------


uint8_t IsFlagAssChanged(void)
{
  uint8_t FlagAssChange = 0;

  FlagTripAss = (uint32_t)Trip_Flags0.all | (uint32_t)Trip_Flags1.all << 16;
  FlagAlarmAss = (uint64_t)Alarm_Flags0.all | (uint64_t)Alarm_Flags1.all << 16 | (uint64_t)Alarm_Flags2.all << 32;
  FlagNoTripAlarmAss = (uint64_t)Flags0.all | (uint64_t)Flags1.all << 16 | (uint64_t)Flags2.all << 32;
  
  if (FlagTripAssStatus ^ FlagTripAss || FlagAlarmAssStatus ^ FlagAlarmAss || FlagNoTripAlarmAssStatus ^ FlagNoTripAlarmAss)
  {
    FlagAssChange = 1;
    FlagTripAssStatus = FlagTripAss;
    FlagAlarmAssStatus = FlagAlarmAss;
    FlagNoTripAlarmAssStatus = FlagNoTripAlarmAss;
  }

  return FlagAssChange;
  
}

//------------------------------------------------------------------------------
//  END OF FUNCTION         IsFlagAssChanged
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       RelayManagement()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Realy Management
//
//  MECHANICS:          This subroutine parses from setpoints group12 the relay assignment according to specification
// 
//  INPUTS:             RelayConfig, Relay

//  CALLS:              ServiceRelays, IsFlagAssChanged, IsRelayAssChanged
// 
//------------------------------------------------------------------------------------------------------------


void RelayManagement(void)
{
  
  uint8_t i;

  Relay1[0] = (uint32_t) Setpoints12.stp.Relay1_Cfg1 | (uint32_t) Setpoints12.stp.Relay1_Cfg2 << 16;
  Relay1[1] = (uint32_t) Setpoints12.stp.Relay1_Cfg3 | (uint32_t) Setpoints12.stp.Relay1_Cfg4 << 16;

  Relay2[0] = (uint32_t) Setpoints12.stp.Relay2_Cfg1 | (uint32_t) Setpoints12.stp.Relay2_Cfg2 << 16;
  Relay2[1] = (uint32_t) Setpoints12.stp.Relay2_Cfg3 | (uint32_t) Setpoints12.stp.Relay2_Cfg4 << 16;

  Relay3[0] = (uint32_t) Setpoints12.stp.Relay3_Cfg1 | (uint32_t) Setpoints12.stp.Relay3_Cfg2 << 16;
  Relay3[1] = (uint32_t) Setpoints12.stp.Relay3_Cfg3 | (uint32_t) Setpoints12.stp.Relay3_Cfg4 << 16;

  if (IsFlagAssChanged() || IsRelayAssStpChanged())
  {
    for (i = 0; i < MAX_RELAY_ASS; i++)
    {
      if (i < 32)
      {
        RelayRegister[1] = 0;
        RelayRegister[0] = 1 << i;
      }
      else 
      {
        RelayRegister[0] = 0;
        RelayRegister[1] = 1 << (i-32);
      }
      
      if (Relay1[0] & RelayRegister[0] || Relay1[1] & RelayRegister[1])
      {
        ServiceRelays(i, RELAY1_NUM);
      }

      if (Relay2[0] & RelayRegister[0] || Relay2[1] & RelayRegister[1])
      {
        ServiceRelays(i, RELAY2_NUM);
      }

      if (Relay3[0] & RelayRegister[0] || Relay3[1] & RelayRegister[1])
      {
        ServiceRelays(i, RELAY3_NUM);
      }
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         RelayManagement()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       AFE_SPI_Xfer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           AFE SPI Transfer
//
//  MECHANICS:          This subroutine transmits the 16-bit data in TxData over the SPI3 bus, then reads
//                      and returns the data received in the same transaction.
//
//  CAVEATS:            No error checking is performed
//                      If the SPI transfer operation fails (which means there is a microprocessor failure,
//                        so it is a highly unlikely occurrence), a watchdog reset will occur
//
//  INPUTS:             TxData - the data word to be transmitted
// 
//  OUTPUTS:            Returns the data word that was received
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint16_t AFE_SPI_Xfer(uint16_t TxData)
{
  uint16_t temp;                        // Temporary

  AFE_CSN_ACTIVE;                       // Activate the AFE chip select
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the ADI AFE.
  SPI3->DR = TxData;
                                        // Wait for the transfer to complete
  while ( (SPI3->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI3->DR;                      // Clear RXNE flag by reading data reg and capture data
  AFE_CSN_INACTIVE;                     // Deactivate the AFE chip select
  return (temp);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         AFE_SPI_Xfer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_Clean()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Clear FRAM Contents    
//                      
//  MECHANICS:          This subroutine writes "0's" into either the on-board FRAM or the Frame FRAM at the
//                      location held in address
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS!
//                      The data is written in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: device - FRAME FRAM or on-board FRAM
//                                  fram_address - starting byte address in FRAM to write to
//                                  length - length (in words) to write
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_Clean(uint16_t device, uint32_t fram_address, uint16_t length)
{
  uint16_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

                                        // Send out a WRITE ENABLE instruction
  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send write enable opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else if( device == DEV_FRAM2)
  {
    FRAM2_CSN_INACTIVE;
  }

  i = 2;                                // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (i > 0)
  {
    --i;
  }

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_ACTIVE;
  }

                                        // Send write opcode and ms address byte
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_WRITE) << 8) | (uint16_t)((fram_address >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send the remaining address word
  SPI2->DR = (uint16_t)fram_address;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send out the write data
  for (i=0; i<length; ++i)
  {
    SPI2->DR = 0x00;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
  }
  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_INACTIVE;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         FRAM_Clean()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ExtCapt_FRAM_Read()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read data from the ExtCap FRAM
//                      
//  MECHANICS:          This subroutine reads data from the Extended Capture FRAM and places it in the
//                      location addressed by outptr.  Note, the SPI operations are performed in bytes, not
//                      words, because the data is written into the FRAM as bytes.
//                          
//                      REFERENCE: RAMTRON FM25V05, 512Kb Serial 3V F-RAM datasheet Revision 1.00 August 2008
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS
//                      The data is retrieved in bytes
//                      The subroutine assumes SPI1 has already been initialized
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to read from
//                                  length - length (in words) to read
//                                  outptr - word address in memory to place the data
//                      
//  OUTPUTS:            Location(s) pointed at by outptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void ExtCapt_FRAM_Read(uint16_t fram_address, uint16_t length, uint16_t *outptr)
{
  uint16_t i;
  volatile uint8_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes for Frame FRAM
  FRAM1_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send read opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (11nsec) of the FRAM.
  SPI2->DR = FRAM_READ;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  SPI2->DR = (fram_address >> 8);       // Send out the address high byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  SPI2->DR = fram_address;              // Send out the address low byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words     *** DAH  230118

                                        // Read the requested data words
  for (i=0; i<length; ++i)
  {
    SPI2->DR = 0;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *outptr++ = SPI2->DR;               // Clear RXNE flag by reading data reg and capture data
  }
  FRAM1_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ExtCapt_FRAM_Read()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ExtCapt_FRAM_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write data to the Extended Capture FRAM    
//                      
//  MECHANICS:          This subroutine writes the data addressed by inptr into the extended capture FRAM at
//                      the location held in address
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS!
//                      The data is written in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to write to
//                                  length - length (in words) to write
//                                  inptr - address in memory to retrieve the data from
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void ExtCapt_FRAM_Write(uint32_t fram_address, uint16_t length, uint16_t *inptr)
{
  uint16_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes
  FRAM1_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send out a WRITE ENABLE instruction
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  FRAM1_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)
  i = 2;                                // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (i > 0)
  {
    --i;
  }

  FRAM1_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send write opcode
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WRITE;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

                                        // Send address word
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
//  SPI2->DR = (fram_address >> 8);       // Send out the address high byte
  SPI2->DR = fram_address;              // Send out the 16-bit address
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send out the write data as words
  for (i=0; i<length; ++i)
  {
    SPI2->DR = *inptr++;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
  }
  FRAM1_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)           
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ExtCapt_FRAM_Write()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_Read()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read data from On-Board FRAM    
//                      
//  MECHANICS:          This subroutine reads data from the on-board FRAM and places it in the location
//                      addressed by outptr
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS!
//                      The data is retrieved in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to read from
//                                  length - length (in words) to read
//                                  outptr - word address in memory to place the data
//                      
//  OUTPUTS:            Location(s) pointed at by outptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_Read(uint32_t fram_address, uint16_t length, uint16_t *outptr)
{
  uint16_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  FRAM2_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send read opcode and ms byte of address
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (11nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_READ) << 8) | (uint16_t)((fram_address >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  SPI2->DR = (uint16_t)fram_address;              // Send out the remaining  address word
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

                                        // Read the requested data words
  for (i=0; i<length; ++i)
  {
    SPI2->DR = 0;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *outptr++ = SPI2->DR;               // Clear RXNE flag by reading data reg and capture data
  }

  FRAM2_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_Read()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Frame_FRAM_Read()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read data from the Frame FRAM
//                      
//  MECHANICS:          This subroutine reads data from the Frame FRAM and places it in the location
//                      addressed by outptr.  Note, the SPI operations are performed in bytes, not words.
//                      This routine is necessary to maintain compatibility with the PXR25.
//                      Note, the Frame FRAM is a smaller device than the on-board FRAMs and uses a 16-bit
//                      address.  The on-board FRAMs use 24-bit addresses
//                          
//                      REFERENCE: RAMTRON FM25640, 64Kb Serial 3V F-RAM datasheet Revision 1.00 August 2008
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS
//                      The data is retrieved in bytes, not words!
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to read from
//                                  length - length (in bytes) to read
//                                  outptr - word address in memory to place the data
//                      
//  OUTPUTS:            Location(s) pointed at by outptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2
// 
//------------------------------------------------------------------------------------------------------------

void Frame_FRAM_Read(uint16_t fram_address, uint16_t length, uint8_t *outptr)
{
  uint16_t i;
  volatile uint8_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes for Frame FRAM
  FRAME_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send read opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (11nsec) of the FRAM.
  SPI2->DR = FRAM_READ;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  SPI2->DR = (fram_address >> 8);       // Send out the address high byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  SPI2->DR = fram_address;              // Send out the address low byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

                                        // Read the requested data words
  for (i=0; i<length; ++i)
  {
    SPI2->DR = 0;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *outptr++ = SPI2->DR;               // Clear RXNE flag by reading data reg and capture data
  }
  FRAME_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Frame_FRAM_Read()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write data to FRAM    
//                      
//  MECHANICS:          This subroutine writes the data addressed by inptr into either the on-board FRAM or
//                      the Frame FRAM at the location held in address
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS!
//                      The data is written in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: device - FRAME FRAM or on-board FRAM
//                                  fram_address - starting byte address in FRAM to write to
//                                  length - length (in words) to write
//                                  inptr - address in memory to retrieve the data from
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_Write(uint16_t device, uint32_t fram_address, uint16_t length, uint16_t *inptr)
{
  uint16_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

                                        // Send out a WRITE ENABLE instruction
  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {                                     // If invalid value for device, no FRAM is selected
    FRAME_CSN_ACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send write enable opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else if( device == DEV_FRAM2)
  {
    FRAM2_CSN_INACTIVE;
  }

  i = 2;                                // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (i > 0)
  {
    --i;
  }

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_ACTIVE;
  }

                                        // Send write opcode and ms address byte
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_WRITE) << 8) | (uint16_t)((fram_address >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send the remaining address word
  SPI2->DR = (uint16_t)fram_address;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send out the write data
  for (i=0; i<length; ++i)
  {
    SPI2->DR = *inptr++;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
  }
  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else if(device == DEV_FRAM2)
  {
    FRAM2_CSN_INACTIVE;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_Write()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Frame_FRAM_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write data into the Frame FRAM
//                      
//  MECHANICS:          This subroutine writes the data addressed by inptr into either the Frame FRAM at the
//                      location held in address
//                          
//  MECHANICS:          This subroutine writes data into the Frame FRAM and places it in the location
//                      addressed by outptr.  Note, the SPI operations are performed in bytes, not words.
//                      This routine is necessary to maintain compatibility with the PXR25.
//                      Note, the Frame FRAM is a smaller device than the on-board FRAMs and uses a 16-bit
//                      address.  The on-board FRAMs use 24-bit addresses
//                          
//                      REFERENCE: RAMTRON FM25640, 64Kb Serial 3V F-RAM datasheet Revision 1.00 August 2008
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS
//                      The data is written in bytes
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to write to
//                                  length - length (in bytes) to write
//                                  inptr - address in memory to retrieve the data from
//                      
//  OUTPUTS:            Location(s) pointed at by outptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2
// 
//------------------------------------------------------------------------------------------------------------

void Frame_FRAM_Write(uint16_t fram_address, uint16_t length, uint8_t *inptr)
{
  uint16_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes
  FRAME_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send out a WRITE ENABLE instruction
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  FRAME_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)
  i = 2;                                // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (i > 0)
  {
    --i;
  }

  FRAME_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send write opcode
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WRITE;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send ms address byte
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (fram_address >> 8);       // Send out the address high byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
  SPI2->DR = fram_address;              // Send out the address low byte
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

                                        // Send out the write data
  for (i=0; i<length; ++i)
  {
    SPI2->DR = *inptr++;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
  }
  FRAME_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)
  // Trigger Setpoints Copy to Cassette process
  CopyToCassetteFlags();
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Frame_FRAM_Write()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_WriteMinMax()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Min and Max values with checksum to FRAM    
//                      
//  MECHANICS:          This subroutine writes the min and max data addressed by inptr into the on-board
//                      FRAM at the location held in address.  It also computes the checksum of the data and
//                      writes it and its complement at the end of the data.
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            FRAM addresses are in BYTES, not WORDS!
//                      The data is written in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: fram_address - starting byte address in FRAM to write to
//                                  length - length (in words) to write
//                                  dptr - address in memory of the data to be written
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2(), FRAM_XferW()
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_WriteMinMax(uint32_t fram_address, uint16_t length, uint16_t *dptr)
{
  uint32_t chk;
  uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

                                        // Send out a WRITE ENABLE instruction
  FRAM2_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send write enable opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  FRAM2_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)

  temp = 2;                             // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (temp > 0)
  {
    --temp;
  }
                                 
  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  FRAM2_CSN_ACTIVE;                     // Select the FRAM device (Chip select set-up time= 10ns)

                                        // Send write opcode and ms address byte
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_WRITE) << 8) | (uint16_t)((fram_address >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send the remaining address word
  SPI2->DR = (uint16_t)fram_address;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send out the write data
  chk = FRAM_XferW((length>>2), dptr, (uint32_t *)dptr);

                                        // Send out the checksum
  SPI2->DR = (uint16_t)(chk & 0x0000FFFF);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;
  SPI2->DR = (uint16_t)(chk >> 16);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;

                                        // Send out the checksum complement
  chk = ~chk;
  SPI2->DR = (uint16_t)(chk & 0x0000FFFF);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;
  SPI2->DR = (uint16_t)(chk >> 16);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;

  FRAM2_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time= 10ns)

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_WriteMinMax()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_Stat_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write FRAM Status Register    
//                      
//  MECHANICS:          This subroutine writes the status register in the FRAM from the passed parameter:
//                         Bit 7 (WPEN): 0 - Ignore /WP pin 
//                         Bits 6 - 4 (reserved): 000      
//                         Bits 3 - 2 (BP1, BP0): 00 - No blocks in FRAM are protected  
//                         Bits 3 - 2 (BP1, BP0): 01 - Upper block (addresses 0x30000 - 0x3FFFF) is protected
//                         Bits 3 - 2 (BP1, BP0): 10 - Upper two blocks (addr 0x20000 - 0x3FFFF) is protected
//                         Bits 3 - 2 (BP1, BP0): 11 - Entire memory (addr 0x00000 - 0x3FFFF) is protected
//                         Bit 1 (WEL): 0 (read-only bit)    
//                         Bit 0 (reserved): 0
//                         Sending the WREN op-code causes the internal Write Enable Latch (WEL) to be set.
//                         WEL = 1 indicates that writes are permitted.  Attempting to write the WEL bit in 
//                         the Status Register has no effect on the state of this bit – only the WREN op-code 
//                         can set this bit. The WEL bit will be automatically cleared on the rising edge of
//                         /CS following a WRDI, a WRSR, or a WRITE operation. This prevents further writes to 
//                         the Status Register or the FRAM array without another WREN command.
//                          
//                         REFERENCE: Cypress FM25V20A, 2Mb Serial 3V FRAM datasheet Doc 001-90261 Rev G June, 2017
//                      
//  CAVEATS:            Nonr
//                      
//  INPUTS:             Parameters: device - FRAME FRAM or on-board FRAM
//                                  status - 8-bits of status to write
//                      
//  OUTPUTS:            FRAM internal status register
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_Stat_Write(uint16_t device, uint8_t status)
{
  uint8_t i;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

  // Send out a WRITE ENABLE instruction
  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send write enable opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else
  {
    FRAM2_CSN_INACTIVE;
  }
  i = 2;                                // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (i > 0)
  {
    --i;
  }

  // Send out the WRITE STATUS instruction and the data 
  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else
  {
    FRAM2_CSN_ACTIVE;
  }
                                 
  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words
                                        // Send write opcode
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_WRSR) << 8) | status;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                        
  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else
  {
    FRAM2_CSN_INACTIVE;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_Stat_Write()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_WriteEnergy()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Energy to FRAM    
//                      
//  MECHANICS:          This subroutine writes the energy into either the on-board FRAM or the Frame FRAM.
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            This subroutine is written to execute quickly.  As a result, it does not use
//                      multiple calls to FRAM_Write().  The energy registers are transferred to FRAM in one
//                      continuous stream.  Because of this, the location of the registers in FRAM (i.e.,
//                      the FRAM addresses of the 8 energy register blocks), is set by the order that they
//                      are written in.  This MUST match up with the FRAM addresses in FRAM_ReadEnergy and
//                      in FRAM_def.h!
//                      The data is written in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: device - FRAME FRAM or on-board FRAM
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2(), FRAM_XferW()
//
//  EXECUTION TIME:     Measured on 180601.  Worst-case time is about 264usec
// 
//------------------------------------------------------------------------------------------------------------

void FRAM_WriteEnergy(uint8_t device)
{
  uint32_t chk;
  uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

                                        // Send out a WRITE ENABLE instruction
  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send write enable opcode
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = FRAM_WREN;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else
  {
    FRAM2_CSN_INACTIVE;
  }
  temp = 2;                             // Delay at least 50nsec deselect time.  Measured on 151026: 120nsec
  while (temp > 0)
  {
    --temp;
  }
                                 
  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send write opcode and ms address byte
  // Initiate the transfer.  Note, measured the time from CS low to first slock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_WRITE) << 8) | (uint16_t)((FRAM_ENERGY >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send the remaining address word
  SPI2->DR = (uint16_t)FRAM_ENERGY;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

                                            // Send out the first group of energy values
  chk = FRAM_XferW((RES_SIZE>>2), (uint16_t *)&ResidualPha.FwdWHr, (uint32_t *)&ResidualPha.FwdWHr);

                                            // Send out the second group of energy values
  chk += FRAM_XferW((NRG_SIZE>>2), (uint16_t *)&EnergyPha.FwdWHr, (uint32_t *)&EnergyPha.FwdWHr);

                                            // Send out the third group of energy values
  chk += FRAM_XferW((RES_SIZE>>2), (uint16_t *)&ResidualPhb.FwdWHr, (uint32_t *)&ResidualPhb.FwdWHr);

                                            // Send out the fourth group of energy values
  chk += FRAM_XferW((NRG_SIZE>>2), (uint16_t *)&EnergyPhb.FwdWHr, (uint32_t *)&EnergyPhb.FwdWHr);

                                            // Send out the fifth group of energy values
  chk += FRAM_XferW((RES_SIZE>>2), (uint16_t *)&ResidualPhc.FwdWHr, (uint32_t *)&ResidualPhc.FwdWHr);

                                            // Send out the sixth group of energy values
  chk += FRAM_XferW((NRG_SIZE>>2), (uint16_t *)&EnergyPhc.FwdWHr, (uint32_t *)&EnergyPhc.FwdWHr);

                                            // Send out the seventh group of energy values
  chk += FRAM_XferW((RES_SIZE>>2), (uint16_t *)&ResidualAll.FwdWHr, (uint32_t *)&ResidualAll.FwdWHr);

                                            // Send out the eighth group of energy values
  chk += FRAM_XferW((NRG_SIZE>>2), (uint16_t *)&EngyDmnd[1].TotFwdWHr, (uint32_t *)&EngyDmnd[1].TotFwdWHr);

  SPI2->DR = (uint16_t)(chk & 0x0000FFFF);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;
  SPI2->DR = (uint16_t)(chk >> 16);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;

  chk = ~chk;
  SPI2->DR = (uint16_t)(chk & 0x0000FFFF);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;
  SPI2->DR = (uint16_t)(chk >> 16);
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else
  {
    FRAM2_CSN_INACTIVE;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_WriteEnergy()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_ReadEnergy()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Energy from FRAM    
//                      
//  MECHANICS:          This subroutine reads the energy from either the on-board FRAM or the Frame FRAM.
//                          
//                      REFERENCE: RAMTRON FM25V20A, 2Mb Serial 3V F-RAM datasheet Revision F August 2015
//                      
//  CAVEATS:            This subroutine is written to execute quickly.  As a result, it does not use
//                      multiple calls to FRAM_Read().  The energy registers are transferred from FRAM in
//                      one stream after the starting address is entered.  Because of this, this subroutine
//                      must be identical to FRAM_WriteEnergy, regarding the FRAM locations and data sizes.
//                      This subroutine must also match up with the FRAM addresses in FRAM_def.h!
//                      The data is read in words (16-bit), not bytes!
//                      
//  INPUTS:             Parameters: device - FRAME FRAM or on-board FRAM
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2(), FRAM_XferR()
// 
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 569usec
//
//------------------------------------------------------------------------------------------------------------

void FRAM_ReadEnergy(uint8_t device)
{
  uint32_t chk;
  uint32_t fchk, fcmp;
  volatile uint16_t temp;

  Init_SPI2(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  if (device == DEV_FRAME)              // Select the FRAM device (Chip select set-up time= 10ns)
  {
    FRAME_CSN_ACTIVE;
  }
  else
  {
    FRAM2_CSN_ACTIVE;
  }
                                        // Send read opcode and ms byte of address
  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time was
  //   244nsec, so it easily meets the chip select setup time requirement (10nsec) of the FRAM.
  SPI2->DR = (((uint16_t)FRAM_READ) << 8) | (uint16_t)((FRAM_ENERGY >> 16) & 0x000000FF);
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg
                                 
                                        // Send the remaining address word
  SPI2->DR = (uint16_t)FRAM_ENERGY;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

                                        // Read the first group of energy values
  chk = FRAM_XferR((RES_SIZE>>2), (uint16_t *)&ResidualPha.FwdWHr, (uint32_t *)&ResidualPha.FwdWHr);

                                        // Read the second group of energy values
  chk += FRAM_XferR((NRG_SIZE>>2), (uint16_t *)&EnergyPha.FwdWHr, (uint32_t *)&EnergyPha.FwdWHr);

                                        // Read the third group of energy values
  chk += FRAM_XferR((RES_SIZE>>2), (uint16_t *)&ResidualPhb.FwdWHr, (uint32_t *)&ResidualPhb.FwdWHr);

                                        // Read the fourth group of energy values
  chk += FRAM_XferR((NRG_SIZE>>2), (uint16_t *)&EnergyPhb.FwdWHr, (uint32_t *)&EnergyPhb.FwdWHr);

                                        // Read the fifth group of energy values
  chk += FRAM_XferR((RES_SIZE>>2), (uint16_t *)&ResidualPhc.FwdWHr, (uint32_t *)&ResidualPhc.FwdWHr);

                                        // Read the sixth group of energy values
  chk += FRAM_XferR((NRG_SIZE>>2), (uint16_t *)&EnergyPhc.FwdWHr, (uint32_t *)&EnergyPhc.FwdWHr);

                                        // Read the seventh group of energy values
  chk += FRAM_XferR((RES_SIZE>>2), (uint16_t *)&ResidualAll.FwdWHr, (uint32_t *)&ResidualAll.FwdWHr);

                                        // Read the eighth group of energy values
  chk += FRAM_XferR((NRG_SIZE>>2), (uint16_t *)&EngyDmnd[1].TotFwdWHr, (uint32_t *)&EngyDmnd[1].TotFwdWHr);

                                        // Read the checksum and complement
  SPI2->DR = 0;
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  fchk = SPI2->DR;                          // Clear RXNE flag by reading data reg and capture data
  SPI2->DR = 0;
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  fchk += ((uint32_t)SPI2->DR) << 16;
  SPI2->DR = 0;
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  fcmp = SPI2->DR;
  SPI2->DR = 0;
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  fcmp += ((uint32_t)SPI2->DR) << 16;

  if (device == DEV_FRAME)              // Deselect the FRAM device (Chip select hold time= 10ns)
  {
    FRAME_CSN_INACTIVE;
  }
  else
  {
    FRAM2_CSN_INACTIVE;
  }

  if ( (fchk != chk) || fcmp != (~chk) )    // If checksum mismatch...
  {                                             // Set all energy registers to 0
    ResetEnergy();
    EnergyFault = TRUE;
    TP_LEDCode = 2;                         // flash the Status LED red
  }                                             // *** DAH ADD CODE TO GENERATE ALARM, LOG ERROR, BLINK LED RED?, ETC.

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_ReadEnergy()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_XferW()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Transfer Data to FRAM    
//                      
//  MECHANICS:          This subroutine transfers data to FRAM via SPI2.  It also computes the checksum of
//                      the data as it is written.
//                          
//                      REFERENCE: RAMTRON FM25V05, 512Kb Serial 3V F-RAM datasheet Revision 1.00 August 2008
//                      
//  CAVEATS:            It is assumed that the FRAM Write commands have already been sent to the FRAM, that
//                      the chip is enabled, and that SPI2 has been intialized properly.
//                      The data is written in words (16-bit), not bytes!
//                      The checksum is computed in double words
//                      
//  INPUTS:             Parameters: len - the number of double-words (32-bits) to transfer
//                                  *dptr - pointer to the first data word to write
//                                  *chkptr - pointer to the first uint32 for the checksum
//                      
//  OUTPUTS:            FRAM memory
//                      
//  ALTERS:             None
//                      
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint32_t FRAM_XferW(uint8_t len, uint16_t *dptr, uint32_t *chkptr)
{
  uint32_t chk;
  volatile uint16_t temp;
  uint8_t i;

  chk = 0;                              // Initialize the checksum
                                        // Send out the write data
  for (i=0; i<len; ++i)
  {
    SPI2->DR = *dptr++;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
    chk += *chkptr++;
    SPI2->DR = *dptr++;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    temp = SPI2->DR;                    // Clear RXNE flag by reading data reg
  }
  return (chk);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_XferW()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    FRAM_XferR()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Transfer Data from FRAM    
//                      
//  MECHANICS:          This subroutine transfers data from FRAM via SPI2.  It also computes the checksum of
//                      the data as it is read.
//                          
//                      REFERENCE: RAMTRON FM25V05, 512Kb Serial 3V F-RAM datasheet Revision 1.00 August 2008
//                      
//  CAVEATS:            It is assumed that the FRAM Read command has already been sent to the FRAM, that the
//                      chip is enabled, and that SPI2 has been intialized properly.
//                      The data is read in words (16-bit), not bytes!
//                      The checksum is computed in double words
//                      
//  INPUTS:             Parameters: len - the number of double-words (32-bits) to read
//                                  *dptr - pointer to the first location to store the data word
//                                  *chkptr - pointer to the first uint32 for the checksum
//                      FRAM memory
//                      
//  OUTPUTS:            RAM memory (pointed to by *dptr)
//                      
//  ALTERS:             None
//                      
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

uint32_t FRAM_XferR(uint8_t len, uint16_t *dptr, uint32_t *chkptr)
{
  uint32_t chk;
  uint8_t i;

  chk = 0;                              // Initialize the checksum

                                        // Read the requested data words
  for (i=0; i<len; ++i)
  {
    SPI2->DR = 0;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *dptr++ = SPI2->DR;                 // Clear RXNE flag by reading data reg and capture data
    SPI2->DR = 0;
    while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *dptr++ = SPI2->DR;                 // Clear RXNE flag by reading data reg and capture data
    chk += *chkptr++;
  }
  return (chk);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      FRAM_XferR()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ComputeChksum32()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute Checksum for 32-bit Data
//
//  MECHANICS:          This subroutine computes the checksum for a number of 32-bit data values.
//
//  CAVEATS:            None
//
//  INPUTS:             *dptr - the location of the first data double-word
//                      len - the number of double-words to compute the checksum on
//
//  OUTPUTS:            The subroutine returns the checksum as a uint32
//
//  ALTERS:             AFEcal.chk, AFEcal.cmp
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

uint32_t ComputeChksum32(uint32_t *dptr, uint16_t len)
{
  uint32_t chk;
  uint8_t i;

  chk = 0;
  for (i=0; i<len; ++i)
  {
    chk += *dptr++;
  }
  return (chk);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ComputeChksum32()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    StoreCalFlash()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Store Calibration Constants in Flash
//                      
//  MECHANICS:          This subroutine writes either:
//                        1) the AFE and ADC calibration constants into Flash, along with their respective
//                           checksums and checksum complements
//                        2) the test injection calibration constants into Flash, along with their
//                           respective checksums and checksum complements
//                      Input parameter mtr_cal determines which constants are written.
//                      The sequence to write the calibration data is as follows:
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Global Block-Protection Unlock (ULBPR) instruction
//                              - Removes write protection from all sectors
//                              - Write disable is automatically activated when this instruction is finished
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Sector Erase (SE) instruction
//                              - Erases the sector (sets all bits to "1")
//                          Issue Read Status Register instruction
//                              - Wait until the sector erase has been completed
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Page Program (PP) command
//                              - This programs the part with the new calibration data
//                              - Write disable is automatically activated when this instruction is finished
//                          Issue Read Status Register instruction
//                              - Wait until the page program has been completed
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Write Block Protection Register (WBPR) instruction
//                              - Write protects the calibration data sector
//                              - Write disable is automatically activated when this instruction is finished
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             mtr_cal - True is metering cal constants, False if test injection cal constants
//                      AFECal.x, ADCcalHigh.x, ADCcalLow.x
//                      
//  OUTPUTS:            Flash memory
//                      The subroutine returns True when it is finished writing the constants to Flash
//                        It returns False otherwise
//                      
//  ALTERS:             SCF_State
//                      
//  CALLS:              Init_SPI1()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t StoreCalFlash(uint8_t mtr_cal)
{
  uint8_t i;
  uint8_t done_status;
  uint16_t temp;
  uint16_t *wptr;

  done_status = FALSE;                  // Assume not done

  switch (SCF_State)
  {

    case 0:                             // State 0 - Initialization
      // Initialize SPI to output bytes
      Init_SPI1(DEV_FRAM_FLASH8);

      // Issue write enable command
      FLASH_CSN_ACTIVE;                     // Select the Flash device (Chip select set-up time= 8ns)
      // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time
      //   was 244nsec, so it easily meets the chip select setup time requirement (8nsec) of the Flash.
      SPI1->DR = FLASH_WREN;                // Output write enable
                                            // Wait for the transfer to complete
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Clear RXNE flag by reading data reg
      FLASH_CSN_INACTIVE;                   // Deselect the Flash device (Chip select high time = 25nsec)
      i = 1;                                // Delay at least 25nsec deselect time
      while (i > 0)                         //   Measured on 151110: 96nsec
      {
        --i;
      }

      // Issue the global block protection unlock command (same steps as above)
      FLASH_CSN_ACTIVE;
      SPI1->DR = FLASH_ULBPR;
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      FLASH_CSN_INACTIVE;
      i = 1;
      while (i > 0)
      {
        --i;
      }

      // Issue write enable command (same steps as above)
      FLASH_CSN_ACTIVE;
      SPI1->DR = FLASH_WREN;
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      FLASH_CSN_INACTIVE;                   // Don't need delay since Init_SPI1() is called next

      // Initialize SPI to output words
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue sector erase command
      FLASH_CSN_ACTIVE;
      if (mtr_cal)                          // Send out erase sector instruction and msb of sector address
      {
        SPI1->DR = (((uint16_t)FLASH_SE) << 8) + (FLASH_AFE_SECTOR >> 4);
      }
      else
      {
        SPI1->DR = (((uint16_t)FLASH_SE) << 8) + (FLASH_TSTINJ_SECTOR >> 4);
      }
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Clear RXNE flag by reading data reg
      if (mtr_cal)                          // Send out the lower nibble of the sector address, along with
      {                                     //   the page and byte address (don't care = 0x000)
        SPI1->DR = (FLASH_AFE_SECTOR << 12);
      }
      else
      {
        SPI1->DR = (FLASH_TSTINJ_SECTOR << 12);
      }
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Clear RXNE flag by reading data reg
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      SCF_State = 1;
      break;

    case 1:                             // State 1 - Wait for Sector Erase to Complete *** DAH ADD COUNTER HERE SO CANNOT HANG WAITING  IF NOT FINISHED AFTER x PASSES, SET HARDWARE ERROR FLAG
      // Initialize SPI to output words.  Must reinitialize SPI in case it was used by a different thread
      //   after exiting State 1 back to the main loop.
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue read status register command and dummy byte to retrieve the data
      FLASH_CSN_ACTIVE;
      SPI1->DR = ((uint16_t)FLASH_RDSR) << 8;   
                                            // Wait for the transfer to complete
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Save status and clear RXNE flag
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      if (!(temp & 0x0080))                 // b7 = busy bit.  If 0, erase is complete, so fall into the
      {                                     //   next state
        SCF_State = 2;
      }
      else                                  // Otherwise erase is still in progress, so just exit
      {
        break;
      }
      
    case 2:                                 // State 2 - Write the New Calibration Data
      // Initialize SPI to output bytes
      Init_SPI1(DEV_FRAM_FLASH8);

      // Issue write enable command
      FLASH_CSN_ACTIVE;
      SPI1->DR = FLASH_WREN;
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      FLASH_CSN_INACTIVE;                       // Don't need delay since Init_SPI1() is called next

      // Initialize SPI to output words
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue page program command and the address
      FLASH_CSN_ACTIVE;
      if (mtr_cal)                          // Send out page program cmnd and ms byte of sector address
      {
        SPI1->DR = (((uint16_t)FLASH_PP) << 8) + ((uint8_t)(FLASH_AFECAL >> 16));
      }
      else
      {
        SPI1->DR = (((uint16_t)FLASH_PP) << 8) + ((uint8_t)(FLASH_TESTINJ >> 16));
      }
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      if (mtr_cal)                          // Send out ls nibble of sector address, page address, and byte
      {                                     //   address
        SPI1->DR = (uint16_t)FLASH_AFECAL;
      }
      else
      {
        SPI1->DR = (uint16_t)FLASH_TESTINJ;
      }
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;

      // Send out the data
      if (mtr_cal)
      {
                                            // Send out the AFE calibration data structure as words
        wptr = (uint16_t *)(&AFEcal.gain[0]);         // Set pointer to address of first word to be written
        for (i=0; i < (AFE_CAL_SIZE >> 1); ++i)
        {
          SPI1->DR = *wptr++;
          while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
          {
          }
          temp = SPI1->DR;
        }
                                            // Send out the ADC High Gain cal data structure as words
        wptr = (uint16_t *)(&ADCcalHigh.gain[0]);     // Set pointer to address of first word to be written
        for (i=0; i < (ADC_CAL_SIZE >> 1); ++i)
        {
          SPI1->DR = *wptr++;
          while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
          {
          }
          temp = SPI1->DR;
        }
                                            // Send out the ADC Low Gain calibration data structure as words
        wptr = (uint16_t *)(&ADCcalLow.gain[0]);      // Set pointer to address of first word to be written
        for (i=0; i < (ADC_CAL_SIZE >> 1); ++i)
        {
          SPI1->DR = *wptr++;
          while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
          {
          }
          temp = SPI1->DR;
        }
      }
      else
      {
        wptr = (uint16_t *)(&TestInjCal.midpoint_ph); // Set pointer to address of first word to be written
        for (i=0; i < (TESTINJ_CAL_SIZE >> 1); ++i)
        {
          SPI1->DR = *wptr++;
          while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
          {
          }
          temp = SPI1->DR;
        }
      }
      FLASH_CSN_INACTIVE;
      SCF_State = 3;
      break;

    case 3:                             // State 3 - Wait for Page Program to Complete
      // Initialize SPI to output words.  Must reinitialize SPI in case it was used by a different thread
      //   after exiting State 1 back to the main loop.
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue read status register command and dummy byte to retrieve the data
      FLASH_CSN_ACTIVE;
      SPI1->DR = ((uint16_t)FLASH_RDSR) << 8;   
                                            // Wait for the transfer to complete
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Save status and clear RXNE flag
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      if (!(temp & 0x0080))                 // b7 = busy bit.  If 0, erase is complete, so fall into the
      {                                     //   next state
        SCF_State = 4;
      }
      else                                  // Otherwise erase is still in progress, so just exit
      {
        break;
      }

    case 4:                             // State 4 - Write Block Protection
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
      while (i > 0)
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

      done_status = TRUE;
      SCF_State = 0;                        // Reset the state
      break;
  }

  return (done_status);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      StoreCalFlash()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Flash_PageWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write A Page of Data to Flash
//                      
//  MECHANICS:          This subroutine writes data (up to 256 bytes) to a page of Flash.  The sequence is
//                      as follows:
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Page Program (PP) command
//                              - This programs the part with the new data
//                              - Write disable is automatically activated when this instruction is finished
//                          Issue Read Status Register instruction
//                              - Wait until the page program has been completed
//                      Notes:
//                          1) It is assumed that the data is in an unprotected section of Flash.  The write
//                             protection unlock command is NOT issued.
//                          2) The address is a page address.  The byte address is set to 0.  The address is
//                             formatted as 0xsssp, where sss is the sector and p is the page within the
//                             sector
//                          3) The length, num_words, is in words, so up to 128 words may be written.
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             flash_addr - the sector (b15..b4) and page (b3..0) address
//                      num_words - the number of words to be written to the page (128 max)
//                      *dataptr - pointer to the data to be writtem
//                      
//  OUTPUTS:            Flash memory
//                      The subroutine returns True when it is finished writing the data to Flash
//                        It returns False otherwise
//                      
//  ALTERS:             FPW_State
//                      
//  CALLS:              Init_SPI1()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Flash_PageWrite(uint16_t flash_addr, uint8_t num_words, uint16_t *dataptr)
{
  uint8_t i;
  uint16_t temp;

  switch (FPW_State)
  {

    case 0:                                 // State 0 - Write the Data
      Init_SPI1(DEV_FRAM_FLASH8);               // Initialize SPI to output bytes

      // Issue write enable command
      FLASH_CSN_ACTIVE;
      SPI1->DR = FLASH_WREN;
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      FLASH_CSN_INACTIVE;                       // Don't need delay since Init_SPI1() is called next

      Init_SPI1(DEV_FRAM_FLASH16);              // Initialize SPI to output words

      // Issue page program command and the data
      FLASH_CSN_ACTIVE;
                                                // Send out page program cmnd and ms byte of sector address
      SPI1->DR = (((uint16_t)FLASH_PP) << 8) + (flash_addr >> 8);
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
                                                // Send out ls nibble of sector address, page address, and
      SPI1->DR = (flash_addr << 8);             //   byte address.  Byte address is 0x00
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
                                                // Send out the data as words
      for (i=0; i < num_words; ++i)
      {
        SPI1->DR = *dataptr++;
        while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
        {
        }
        temp = SPI1->DR;
      }
      FLASH_CSN_INACTIVE;
      FPW_State = 1;
      break;

    case 1:                             // State 1 - Wait for Page Program to Complete
      // Initialize SPI to output words.  Must reinitialize SPI in case it was used by a different thread
      //   after exiting State 0 back to the main loop.
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue read status register command and dummy byte to retrieve the data
      FLASH_CSN_ACTIVE;
      SPI1->DR = ((uint16_t)FLASH_RDSR) << 8;   
                                            // Wait for the transfer to complete
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Save status and clear RXNE flag
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      if (!(temp & 0x0080))                 // b7 = busy bit.  If 0, page program is complete, so reset
      {                                     //   state and return True
        FPW_State = 0;
        return (TRUE);
      }
      break;                                // Otherwise page program is still in progress, so just exit

    default:                            // Default state - this should never be entered
      FPW_State = 0;                        // Reset the state
      break;
  }

  return (FALSE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Flash_PageWrite()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Flash_WriteWaveform()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Waveform to Flash
//                      
//  MECHANICS:          This subroutine writes a trip or alarm waveform into Flash.  The waveform is written
//                      a page at a time.  The subroutine works as follows:
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Page Program (PP) command
//                              - This programs the part with the new data
//                              - Write disable is automatically activated when this instruction is finished
//                          Issue Read Status Register instruction
//                              - Wait until the page program has been completed.  This should be on the
//                                call, since the interrupt is set to occur every 1.5msec
//                      Notes:
//                          1) It is assumed that the data is in an unprotected section of Flash.  The write
//                             protection unlock command is NOT issued.
//                          2) The length, num_words, is in words, so up to 128 words may be written.
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//
//                      Note, this code has been written to run as fast as possible, especially with the SPI
//                      transactions.  Timing tests were conducted on 180615.  Reference comments in
//                      FRAM_Flash_def.h and especially in Intr.c - DMA1_Stream0_IRQHandler(), under Trip
//                      Waveform Captures.
//                      
//  CAVEATS:            It is assumed that the data is in an unprotected part of Flash.  It is also assumed
//                      that the sections being written to have already been erased.
//                      
//  INPUTS:             FWW_State
//                      Alarm_WF_Capture.SampleStartIndex - the index in SampleBuf[] of the first set of
//                          samples to capture
//                      flash_addr - the sector (b15..b4) and page (b3..0) address
//                      num_words - the number of words to be written to the page (128 max)
//                      
//  OUTPUTS:            Flash memory
//                      The subroutine returns True when it is finished writing the data to Flash
//                        It returns False otherwise
//                      
//  ALTERS:             FWW_State
//                      SBout_Index - the index in SampleBuf[] of the present set of samples to capture
//                      *SBout_ptr - pointer to the data to be writtem
//                      WF_Struct.NumSamples - the number of sample sets that have been written
//                      
//  CALLS:              Init_SPI1(), FRAM_Write()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Flash_WriteWaveform(struct FLASH_INT_REQ *WF_Struct, uint8_t WF_Abort)
{
  union temp_bytes
  {
    uint8_t i;
    uint8_t num_waveforms;
  } t8;
  uint8_t num_words, num_words1, exit_flag;
  uint32_t temp[2];

  exit_flag = FALSE;

  while (!exit_flag)
  {
    switch (FWW_State)
    {
      case 0:                           // State 0: Set up
//TESTPIN_D1_HIGH;
        SBout_Index = WF_Struct->SampleStartIndex;
        SBout_ptr = (uint16_t *)(&SampleBuf[SBout_Index].Ia);
        WF_Struct->NumSamples = 0;
        // Compute the starting address in Flash for the write
        if (WF_Struct == &Trip_WF_Capture)
        {
          WF_Struct->FlashAdd = TRIP_WAVEFORMS_START;
          WF_Struct->WF_Type = EV_TYPE_TRIPWF;
        }
        else if (WF_Struct == &Alarm_WF_Capture)
        {
          WF_Struct->FlashAdd = ALARM_WAVEFORMS_START;
          WF_Struct->WF_Type = EV_TYPE_ALARMWF;
        }
        else
        {
          WF_Struct->FlashAdd = EXTCAP_WAVEFORM_START;
          WF_Struct->WF_Type = EV_TYPE_EXTCAPWF;
        }
        WF_Struct->FlashAdd += (WF_Struct->EV_Add.NextEvntNdx * WF_SIZE_IN_SECTORS);
        // Address is SSSP ("S" = sector, "P" = page), so need to shift left 4 to move sector to MS position
        WF_Struct->FlashAdd = (WF_Struct->FlashAdd << 4);

        // If sample buffer is filled, or starting index of samples to write is behind the index of the
        //   present sample, we can fill from SampleBuf[], so fall into State 1
        if ( (SampleBufFilled) || (SBout_Index < SampleIndex) )
        {
          FWW_State = 1;
//          break;
        }
        // Otherwise the sample buffer has not been filled, so there are garbage values in some of the
        //   precycles.  Jump to State 3 to fill these with zeros until we reach the first captured sample
        else
        {
          FWW_State = 3;
          break;
        }
    
      case 1:                           // State 1: Write the data
// TESTPIN_A3_TOGGLE;
        // Issue write enable command
        Init_SPI1(DEV_FRAM_FLASH8);             // Initialize SPI to transfer bytes
        FLASH_CSN_ACTIVE;
        SPI1->DR = FLASH_WREN;
        while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
        {
        }
        temp[0] = SPI1->DR;
        FLASH_CSN_INACTIVE;                     // Don't need delay since Init_SPI1() is called next
    
        // Issue page program command and the address
        Init_SPI1(DEV_FRAM_FLASH16);            // Initialize SPI to output words  (routine takes 600nsec)
        FLASH_CSN_ACTIVE;                       // Send out page program cmnd and ms byte of sector address
        SPI1->DR = (((uint16_t)FLASH_PP) << 8) + (WF_Struct->FlashAdd >> 8);
        while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
        {
        }
        SPI1->DR = (WF_Struct->FlashAdd << 8);                  // Send out ls nibble of sector address,
        while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)           //   page address, and byte address
        {                                                       //   Byte address is 0x00
        }
    
        // Send out the data as words
        // Store the number of sample sets to write in num_words1
        num_words1 = ((2880 - WF_Struct->NumSamples) > 7) ? 7 : (2880 - WF_Struct->NumSamples);
        WF_Struct->NumSamples += num_words1;                    // Update the count
    
        if ((SBout_Index + num_words1) > TOTAL_SAMPLE_SETS)     // If we are wrapping around SampleBuf[], we
        {                                                       //   need to split this into two writes
          num_words = (TOTAL_SAMPLE_SETS - SBout_Index);
          num_words1 -= num_words;
          num_words = num_words * (sizeof(struct RAM_SAMPLES))/2;
          for (t8.i=0; t8.i<num_words; ++t8.i)
          {
            SPI1->DR = *SBout_ptr++;
            while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
            {
            }
          }
          SBout_ptr = (uint16_t *)(&SampleBuf[0].Ia);
          num_words = num_words1 * (sizeof(struct RAM_SAMPLES))/2;
          for (t8.i=0; t8.i<num_words; ++t8.i)
          {
            SPI1->DR = *SBout_ptr++;
            while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
            {
            }
          }
          while ((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY)         // Make sure last char is finished before
          {                                                     //   proceeding
          }
          SBout_Index = num_words1;
        }
        else
        {
          num_words = num_words1 * (sizeof(struct RAM_SAMPLES))/2;
          for (t8.i=0; t8.i<num_words; ++t8.i)
          {
            SPI1->DR = *SBout_ptr++;
            while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
            {
            }
            temp[0] = SPI1->DR;
          }
          SBout_Index += num_words1;
          if (SBout_Index >= TOTAL_SAMPLE_SETS)  
          {
            SBout_Index = 0;
            SBout_ptr = (uint16_t *)(&SampleBuf[0].Ia);
          }
          while ((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY)         // Make sure last char is finished before
          {                                                     //   proceeding
          }
        }
        FLASH_CSN_INACTIVE;
        FWW_State = 2;
        exit_flag = TRUE;
        break;
    
      case 3:                           // State 1: Write zeros
// TESTPIN_A3_TOGGLE;
        // Issue write enable command
        Init_SPI1(DEV_FRAM_FLASH8);             // Initialize SPI to transfer bytes
        FLASH_CSN_ACTIVE;
        SPI1->DR = FLASH_WREN;
        while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
        {
        }
        temp[0] = SPI1->DR;
        FLASH_CSN_INACTIVE;                     // Don't need delay since Init_SPI1() is called next
    
        // Issue page program command and the address
        Init_SPI1(DEV_FRAM_FLASH16);            // Initialize SPI to output words  (routine takes 600nsec)
        FLASH_CSN_ACTIVE;                       // Send out page program cmnd and ms byte of sector address
        SPI1->DR = (((uint16_t)FLASH_PP) << 8) + (WF_Struct->FlashAdd >> 8);
        while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
        {
        }
        SPI1->DR = (WF_Struct->FlashAdd << 8);                  // Send out ls nibble of sector address,
        while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)           //   page address, and byte address
        {                                                       //   Byte address is 0x00
        }
    
        // Send out the data as words
        // Store the number of sample sets to write in num_words1.  We will write zeros until we reach
        //   TOTAL_SAMPLE_SETS
        num_words1 = ( ((SBout_Index + 7) > TOTAL_SAMPLE_SETS) ? (TOTAL_SAMPLE_SETS - SBout_Index) : 7);
        WF_Struct->NumSamples += num_words1;                    // Update the count
        num_words = num_words1 * (sizeof(struct RAM_SAMPLES))/2;
        for (t8.i=0; t8.i<num_words; ++t8.i)
        {
          SPI1->DR = 0;
          while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
          {
          }
          temp[0] = SPI1->DR;
        }
        // Update the output index in SampleBuf[].  If it has reached the end, the next sample to write is
        //   at index = 0.  We can start getting the samples from SampleBuf[] instead of writing zeros, so
        //   the set State to 2.  Otherwise set State to 4 to keep filling with zeros.
        SBout_Index += num_words1;
        if (SBout_Index >= TOTAL_SAMPLE_SETS)  
        {
          SBout_Index = 0;
          SBout_ptr = (uint16_t *)(&SampleBuf[0].Ia);
          // If we have written less than 7 words, we need to fill in the remaining words with data from
          //   SampleBuf[]
          if (num_words1 < 7)
          {
            num_words = (7 - num_words1) * (sizeof(struct RAM_SAMPLES))/2;
            for (t8.i=0; t8.i<num_words; ++t8.i)
            {
              SPI1->DR = *SBout_ptr++;
              while ((SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
              {
              }
              temp[0] = SPI1->DR;
            }
          }            
          FWW_State = 2;
        }
        else
        {
          FWW_State = 4;
        }
        while ((SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY)         // Make sure last char is finished before
        {                                                     //   proceeding
        }
        FLASH_CSN_INACTIVE;
        exit_flag = TRUE;
        break;
    
      case 2:                           // State 2 - Wait for Page Program to Complete
      case 4:                           // State 4 - Wait for Page Program to Complete
        // Initialize SPI to output words.  Must reinitialize SPI in case it was used by a different thread
        //   after exiting State 0 back to the main loop.
        Init_SPI1(DEV_FRAM_FLASH16);
    
        // Issue read status register command and dummy byte to retrieve the data
        FLASH_CSN_ACTIVE;
        SPI1->DR = ((uint16_t)FLASH_RDSR) << 8;   
                                            // Wait for the transfer to complete
        while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
        {
        }
        temp[0] = SPI1->DR;                 // Save status and clear RXNE flag
        FLASH_CSN_INACTIVE;                 // Deselect the Flash device (Chip select hold time = 8nsec)
        if (!(temp[0] & 0x00000080))        // b7 = busy bit.  If 0, page program is complete
        {
          ++(WF_Struct->FlashAdd);          // Increment the Flash address        *** DAH NEED TO CHECK FOR PARTIAL IN CASE CLOSING INTO A FAULT!!
          if ((WF_Struct->NumSamples >= 2880) || (WF_Abort))  // If number of sample sets written reaches
          {                                                   //   2880 (36 cycles) or aborting, we are done
            FWW_State = 0;                                    // Reset the state and return True
//TESTPIN_D1_LOW;
            return (TRUE);
          }                                       
          else                                              // Otherwise there are still more samples to
          {                                                 //   store, so jump back
            FWW_State--; 
          }
        }
        // If b7 = 1, page program is still going on, so remain in this state and exit.  This shouldn't
        //   happen, because the interrupts are spaced by 1.5msec, which is the max page program time
        else
        {
          exit_flag = TRUE;
        }
        break;
    
      default:                          // Default state - this should never be entered
        FPW_State = 0;                      // Reset the state
        exit_flag = TRUE;
        break;
    }

  }

  return (FALSE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Flash_WriteWaveform()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Flash_EraseSectorBlock()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Erase a Sector or Block in Flash
//                      
//  MECHANICS:          This subroutine erases a sector or a block in Flash, depending on input parameter
//                      SectorErase (True - Sector Erase, False - Block Erase).  The sequence is as follows:
//                          Issue Write Enable (WREN) instruction
//                              - Allows a write command to have affect
//                          Issue Sector Erase (SE) or Block Erase (BE) instruction
//                              - Erases the sector or Block (sets all bits to "1")
//                          Issue Read Status Register instruction
//                              - Wait until the sector or block erase has been completed
//                      Notes:
//                          1) It is assumed that the data is in an unprotected section of Flash.  The write
//                             protection unclock command is NOT issued.
//                          2) The address is formatted as follows: 0xsssp, where "sss" is the sector
//                             address ("bbs" where bb is the block address) and "p" is the page address.
//                             The page address value ("p") does not matter.  The byte address is set to 0
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             flash_addr - the sector or block address to be erased
//                      SectorErase - True = sector erase, False = block erase
//                      
//  OUTPUTS:            Flash memory
//                      The routine returns True when the sector or block erase is done.  It returns False
//                      otherwise
//                      
//  ALTERS:             FSBE_State
//                      
//  CALLS:              Init_SPI1()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Flash_EraseSectorBlock(uint16_t flash_addr, uint8_t SectorErase)
{
  uint16_t temp;

  switch (FSBE_State)
  {
    case 0:                             // State 0 - Initialization
      Init_SPI1(DEV_FRAM_FLASH8);           // Initialize SPI to output bytes

      // Issue write enable command (same steps as above)
      FLASH_CSN_ACTIVE;
      SPI1->DR = FLASH_WREN;
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;
      FLASH_CSN_INACTIVE;                   // Don't need delay since Init_SPI1() is called next

      Init_SPI1(DEV_FRAM_FLASH16);          // Initialize SPI to output words

      // Issue sector erase command
      FLASH_CSN_ACTIVE;
                                            // Send out erase instruction and msb of address
      SPI1->DR = (SectorErase) ? ( (((uint16_t)FLASH_SE) << 8) + (flash_addr >> 8) )
                               : ( (((uint16_t)FLASH_BE) << 8) + (flash_addr >> 8) );
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Clear RXNE flag by reading data reg
      SPI1->DR = (flash_addr << 8);         // Send out the lower nibble of the sector address, along with
                                            //   the page and byte address (don't care)
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Clear RXNE flag by reading data reg
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      FSBE_State = 1;
      break;

    case 1:                             // State 1 - Wait for Erase to Complete *** DAH ADD COUNTER HERE SO CANNOT HANG WAITING  IF NOT FINISHED AFTER x PASSES, SET HARDWARE ERROR FLAG
      // Initialize SPI to output words.  Must reinitialize SPI in case it was used by a different thread
      //   after exiting State 0 back to the main loop.
      Init_SPI1(DEV_FRAM_FLASH16);

      // Issue read status register command and dummy byte to retrieve the data
      FLASH_CSN_ACTIVE;
      SPI1->DR = ((uint16_t)FLASH_RDSR) << 8;   
                                            // Wait for the transfer to complete
      while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
      {
      }
      temp = SPI1->DR;                      // Save status and clear RXNE flag
      FLASH_CSN_INACTIVE;                   // Deselect the FRAM device (Chip select hold time = 8nsec)
      if (!(temp & 0x0080))                 // b7 = busy bit.  If 0, erase is complete, so reset state and
      {                                     //   return True
        FSBE_State = 0;
        return (TRUE);
      }
      break;                                // Otherwise erase is still in progress, so just exit

    default:                            // Default state - this should never be entered
      FSBE_State = 0;                        // Reset the state
      break;

  }
      
  return (FALSE);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Flash_EraseSectorBlock()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Flash_Read()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read data from Flash    
//                      
//  MECHANICS:          This subroutine reads data from Flash and places it in the location addressed by
//                      outptr
//                      Three parameters are passed:
//                         - flash_address: (byte address in Flash)
//                         - length: (in words)
//                         - outptr: address in memory to place the data
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            Flash addresses are in BYTES, not WORDS!
//                      The Flash address, flash_address, is a 32-bit parameter.  However, the most
//                      significant byte MUST be zero (the address in Flash is 3 bytes long
//                      
//  INPUTS:             Parameters: flash_address - starting byte address in Flash to read from
//                                  length - length (in words) to read
//                                  outptr - address in memory to place the data
//                      
//  OUTPUTS:            Location(s) pointed at by outptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI1()
// 
//------------------------------------------------------------------------------------------------------------

void Flash_Read(uint32_t flash_address, uint16_t length, uint16_t *outptr)
{
  uint8_t i;
  volatile uint16_t temp;

  Init_SPI1(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  // Issue read command + ms byte of sector address
  FLASH_CSN_ACTIVE;
                                            // Send out read instruction and ms byte of address
  SPI1->DR = (((uint16_t)FLASH_RD) << 8) + (uint16_t)(flash_address >> 16);
                                            // Wait for SPI to complete
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;                          // Clear RXNE flag by reading data reg

  // Issue the remainder of the address
  SPI1->DR = (uint16_t)(flash_address);
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;

  // Read the data words
  for (i=0; i<length; ++i)
  {
    SPI1->DR = 0;
    while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    *outptr++ = SPI1->DR;               // Clear RXNE flag by reading data reg and capture data
  }
  FLASH_CSN_INACTIVE;                    // Deselect the Flash device (Chip select high time= 25nsec)
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Flash_Read()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Flash_Read_ID()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Flash ID 
//                      
//  MECHANICS:          This subroutine reads ID of Flash and places it in Flash_ID
//                          
//                      REFERENCE: MicroChip SST26VF064B DS200051 9G 2015
//                      
//  CAVEATS:            This subroutine is intended to be called in the initialization to just retrieve the
//                      serial Flash ID for Factory Test.  If it is called during normal operation, it must
//                      go through SPI1_Flash_Manager()
//                      
//  INPUTS:             None
//                      
//  OUTPUTS:            Flash_ID
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI1()
//
//  EXECUTION TIME:     15.35usec (Rev 142 code) 
// 
//------------------------------------------------------------------------------------------------------------

void Flash_Read_ID(void)
{
  uint8_t i;
  volatile uint16_t temp;

  Init_SPI1(DEV_FRAM_FLASH16);          // Initialize SPI to output words

  // Issue read command + ms byte of sector address (address = 0)
  FLASH_CSN_ACTIVE;
                                            // Send out read instruction and ms byte of address
  SPI1->DR = (((uint16_t)FLASH_SFDP) << 8);
                                            // Wait for SPI to complete
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;                          // Clear RXNE flag by reading data reg

  // Issue the remainder of the address (address = 0)
  SPI1->DR = 0;
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;

  Init_SPI1(DEV_FRAM_FLASH8);          // Initialize SPI to output bytes

  // Read the data bytes - first one is a dummy byte
  SPI1->DR = 0;
  while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI1->DR;
  for (i=0; i<8; ++i)
  {
    SPI1->DR = 0;
    while ( (SPI1->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
    {
    }
    Flash_ID.by[i] = SPI1->DR;           // Clear RXNE flag by reading data reg and capture data
  }
  FLASH_CSN_INACTIVE;                    // Deselect the Flash device (Chip select high time= 25nsec)
}


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Flash_Read_ID()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    EEPOT_Xfer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Transfer data to/from the EEPOT
//                      
//  MECHANICS:          This subroutine performs a SPI read or write to the EEPOT, depending on the value in
//                      WrData.  If the operation is a write, it is assumed it is to one of the (volatile)
//                      registers; otherwise, EEPOT_Write would be used.
//                          
//                      REFERENCE: Microchip Technology MCP4261 2008 DS22059B
//
//                      Note, the data sheet (front page) indicates 10MHz SPI clock speed is supported.  
//                      However, the timing diagram shows that the SDO valid after clock edge time is
//                      70nsec.  The STM32F407 data input setup time is 6.5ns.  With a 50% duty cycle for
//                      the SPI clock, the maximum SPI clock frequency is actually 1/(76.5ns x 2) = 6.5MHz.
//                      Microchip was contacted on 3/17/10 (for the NRX project) and confirmed that the
//                      clock speed for write operations is slower.
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             WrData: the address, command, and data to be passed to the EEPOT
//                      
//  OUTPUTS:            The value read via the transfer is returned
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

uint16_t EEPOT_Xfer(uint16_t WrData)
{
  uint16_t temp;
  uint8_t i;

  Init_SPI2(DEV_EEPOT_16);              // Initialize SPI to output words

  EEPOT_CSN_ACTIVE;                     // Select the EEPOT device (Chip select set-up time= 60ns)

  // Initiate the transfer.  Note, measured the time from CS low to first clock pulse on 150820.  Time
  //   was 244nsec, so it easily meets the chip select setup time requirement (60nsec) of the EEPOT.
  SPI2->DR = WrData;
                                        // Wait for the transfer to complete
  while ( (SPI2->SR & (SPI_SR_RXNE + SPI_SR_TXE)) != (SPI_SR_RXNE + SPI_SR_TXE) )
  {
  }
  temp = SPI2->DR;                      // Clear RXNE flag by reading data reg

  i = 2;                                // Deselect hold time after last clock is 100ns
  while (i > 0)                         //   Measured on 151026: 120nsec
  {
    --i;
  }

  EEPOT_CSN_INACTIVE;                   // Deselect the EEPOT device (Chip select high time = 25nsec)

  // Must wait 50nsec before selecting the EEPOT again.  This is not a problem, since would have to go
  // around the main loop (16msec) before the EEPOT can be accessed again

  return (temp);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      EEPOT_Xfer()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EEPOT_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write EEPOT settings
//
//  MECHANICS:          This function writes a potentiometer wiper setting to the Frame EEPOT.  The value is
//                      written to both the nonvolatile and volatile registers.  It is assumed the address
//                      is for the nonvolatile memory; otherwise, EEPOT_xfer would be used.  The function
//                      first writes the value to the volatile register.  It then writes the value to the
//                      nonvolatile register, and waits for the write to complete.  The subroutine then
//                      checks the TCON register.  If the value is not what is expected, the subroutine
//                      rewrites this value.
//
//  CAVEATS:            None
//
//  INPUTS:             FrameEEPOT.WrVal - the address and value to be written
// 
//  OUTPUTS:            The function returns True when the write has been completed.  It returns False
//                      otherwise
//
//  ALTERS:             FrameEEPOT.State
// 
//  CALLS:              EEPOT_Xfer()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t EEPOT_Write(void)
{
  uint8_t i, retval;

  retval = FALSE;

  switch (FrameEEPOT.State)
  {
    case 0:                             // Write the new setting
      // First write the value to the Frame EEPOT volatile register,  The value is held in FrameEEPOT.WrVal.
      //   However, FrameEEPOT.WrVal has the address for the nonvolatile register.  Masking off b13 converts
      //   the address to the corresponding volatile register.
      EEPOT_Xfer(FrameEEPOT.WrVal & 0xDFFF);
      i = 1;                                // Must wait 50nsec before selecting the EEPOT again
      while (i > 0)                         //   Measured on 151110: 96nsec
      {
        --i;
      }
      // Now write the value to the volatile register
      EEPOT_Xfer(FrameEEPOT.WrVal);
      FrameEEPOT.State = 1;
      break;

    case 1:                             // Wait for write to finish
      if ( (EEPOT_Xfer(EEPOT_RD_STATUS) & 0x0010) == 0x0000)      // If Write is finished, go to next state
      {
        FrameEEPOT.State = 2;
      }                                                           // Otherwise remain in this state
      // Exit the subroutine either way.  This ensures the wait time after CS is inactive (50ns) is met.
      //   See comment at the end of EEPOT_Xfer()
      break;                                                      

    case 2:                             // Read TCON register and compare to expected value
      if ( (EEPOT_Xfer(EEPOT_RD_TCON) & 0x01FF) != 0x01FF)   // If TCON register not correct, rewrite it
      {
        EEPOT_Xfer(EEPOT_WR_TCON | 0x01FF);
        FrameEEPOT.State = 3;
      }
      else                                                   // If register is correct, we are done
      {
        FrameEEPOT.State = 0;                                       // Reset the state
        retval = TRUE;                                              // Return True
      }
      break;

    case 3:                             // Wait for write to finish
      if ( (EEPOT_Xfer(EEPOT_RD_STATUS) & 0x0010) == 0x0000)      // If Write is finished, we are done
      {
        FrameEEPOT.State = 0;                                       // Reset the state
        retval = TRUE;                                              // Return True
      }                                                           // Otherwise remain in this state
      break;                                                      // Exit the subroutine either way

    default:                            // This state should never be entered
      FrameEEPOT.State = 0;
      break;
  }
  return (retval);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EEPOT_Write()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadAFECalConstants()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read AFE calibration constants    
//                      
//  MECHANICS:          This subroutine reads the AFE calibration constants from FRAM and Flash.  The
//                      checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI1 is free for use (subroutine is only called in the
//                      initialization portion of the code)
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, AFEcal.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void ReadAFECalConstants(void)
{
  uint8_t fram_error, flash_error, i;
  uint32_t chk;
  struct AFE_CAL tempAFEcal;

  fram_error = FALSE;
  flash_error = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in the
  //   permanent structure (AFEcal).  We will assume these are good
  FRAM_Read(FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));

  // If there is a checksum error, set error flag
  if ( (AFEcal.chk != chk) || (AFEcal.cmp != ~chk) )
  {
    SystemFlags |= CAL_FRAM_ERR;
    fram_error = TRUE;
  }

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most recent.
  if (fram_error)
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in the
    //   temporary structure (tempAFEcal).
    Flash_Read( FLASH_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&tempAFEcal.gain[0]) );

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)(&tempAFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));

    // If there is a checksum error, set error flag
    if ( (tempAFEcal.chk != chk) || (tempAFEcal.cmp != ~chk) )
    {
      SystemFlags |= CAL_FLASH_ERR;
      flash_error = TRUE;
    }

    if (!flash_error)
    {
      for (i=0; i<10; ++i)
      {
        AFEcal.gain[i] = tempAFEcal.gain[i];
        AFEcal.offset[i] = tempAFEcal.offset[i];
        AFEcal.phase[i] = tempAFEcal.phase[i];
      }
      AFEcal.phase[10] = tempAFEcal.phase[10];
      AFEcal.phase[11] = tempAFEcal.phase[11];
    }
    else
    {
      for (i=0; i<5; ++i)
      {
        AFEcal.gain[i] = AFE_CAL_DEFAULT_IGAIN;
        AFEcal.offset[i] = AFE_CAL_DEFAULT_IOFFSET;
        AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
      }
      AFEcal.gain[4] = AFE_CAL_CT_IGAIN;          // For CT Igsrc
      for (i=5; i<8; ++i)
      {
        AFEcal.gain[i] = AFE_CAL_DEFAULT_VGAIN;
        AFEcal.offset[i] = AFE_CAL_DEFAULT_VOFFSET;
        AFEcal.phase[i] = AFE_CAL_DEFAULT_PHASE;
      }
      AFEcal.gain[8] = AFE_CAL_DEFAULT_IGAIN;     // For Rogowski Igsrc
      AFEcal.offset[8] = AFE_CAL_DEFAULT_IOFFSET;
      AFEcal.phase[8] = AFE_CAL_DEFAULT_PHASE;
      AFEcal.gain[9] = AFE_CAL_CT_IGAIN;          // For CT In
      AFEcal.offset[9] = AFE_CAL_DEFAULT_IOFFSET;
      AFEcal.phase[9] = AFE_CAL_DEFAULT_PHASE;
      AFEcal.phase[10] = AFE_CAL_DEFAULT_PHASE;
      AFEcal.phase[11] = AFE_CAL_DEFAULT_PHASE;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadAFECalConstants()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadAFECalConstants1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read AFE calibration constants    
//                      
//  MECHANICS:          This subroutine reads the AFE calibration constants from FRAM and Flash.  The
//                      checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI1 is free for use (subroutine is only called in the
//                      initialization portion of the code)
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, AFEcal.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              Init_SPI2()
// 
//------------------------------------------------------------------------------------------------------------

void ReadAFECalConstants1(uint8_t *dest)
{
  uint8_t i, err;
  uint32_t chk, chk_rd, chkcmp_rd;
  float *flt_ptr;

  err = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in dest[].
  FRAM_Read(FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)dest);

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)dest, ((AFE_CAL_SIZE >> 2)-2));
  chk_rd = (uint32_t)dest[AFE_CAL_SIZE - 8] + ((uint32_t)dest[AFE_CAL_SIZE - 7] << 8)
                    + ((uint32_t)dest[AFE_CAL_SIZE - 6] << 16) + ((uint32_t)dest[AFE_CAL_SIZE - 5] << 24);
  chkcmp_rd = (uint32_t)dest[AFE_CAL_SIZE - 4] + ((uint32_t)dest[AFE_CAL_SIZE - 3] << 8)
                    + ((uint32_t)dest[AFE_CAL_SIZE - 2] << 16) + ((uint32_t)dest[AFE_CAL_SIZE - 1] << 24);

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most recent.
  if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in dest[]
    Flash_Read(FLASH_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)dest);

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)dest, ((AFE_CAL_SIZE >> 2)-2));
    chk_rd = (uint32_t)dest[AFE_CAL_SIZE - 8] + ((uint32_t)dest[AFE_CAL_SIZE - 7] << 8)
                      + ((uint32_t)dest[AFE_CAL_SIZE - 6] << 16) + ((uint32_t)dest[AFE_CAL_SIZE - 5] << 24);
    chkcmp_rd = (uint32_t)dest[AFE_CAL_SIZE - 4] + ((uint32_t)dest[AFE_CAL_SIZE - 3] << 8)
                      + ((uint32_t)dest[AFE_CAL_SIZE - 2] << 16) + ((uint32_t)dest[AFE_CAL_SIZE - 1] << 24);

    // If there is a checksum error, set error flag to use defaults
    if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
    {
      err = TRUE;
    }
  }

  // If both FRAM and Flash are bad, use defaults
  if (err)
  {
    flt_ptr = (float *)(dest);
    for (i = 0; i < 4; ++i)
    {
      *flt_ptr++ = AFE_CAL_DEFAULT_IGAIN;
    }
    *flt_ptr++ = AFE_CAL_CT_IGAIN;
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = AFE_CAL_DEFAULT_VGAIN;
    }
    *flt_ptr++ = AFE_CAL_DEFAULT_IGAIN;
    *flt_ptr++ = AFE_CAL_CT_IGAIN;
    for (i = 0; i < 5; ++i)
    {
      *flt_ptr++ = AFE_CAL_DEFAULT_IOFFSET;
    }
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = AFE_CAL_DEFAULT_VOFFSET;
    }
    *flt_ptr++ = AFE_CAL_DEFAULT_IOFFSET;
    *flt_ptr++ = AFE_CAL_DEFAULT_IOFFSET;
    for (i = 0; i < 12; ++i)
    {
      *flt_ptr++ = AFE_CAL_DEFAULT_PHASE;
    }
  }
  // Otherwise use values that were read

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadAFECalConstants1()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadADCHCalConstants()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read ADC high gain circuit calibration constants    
//                      
//  MECHANICS:          This subroutine reads the ADC high gain  calibration constants from FRAM and Flash.
//                      The checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, ADCcalHigh.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read(), Flash_Read(), ComputeChksum32()
// 
//------------------------------------------------------------------------------------------------------------

void ReadADCHCalConstants(void)
{
  uint8_t fram_error, flash_error, i;
  uint32_t chk;
  struct ADC_CAL tempADCcal;

  fram_error = FALSE;
  flash_error = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in the
  //   permanent structure (AFEcal).  We will assume these are good
  FRAM_Read(FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));

  // If there is a checksum error, set error flag
  if ( (ADCcalHigh.chk != chk) || (ADCcalHigh.cmp != ~chk) )
  {
    SystemFlags |= CAL_FRAM_ERR;
    fram_error = TRUE;
  }

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most current.    */
  if (fram_error)
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in the
    //   temporary structure (tempADCcal).
    Flash_Read(FLASH_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&tempADCcal.gain[0]) );

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)(&tempADCcal.gain[0]), ((ADC_CAL_SIZE >> 2)-2));

    // If there is a checksum error, set error flag
    if ( (tempADCcal.chk != chk) || (tempADCcal.cmp != ~chk) )
    {
      SystemFlags |= CAL_FLASH_ERR;
      flash_error = TRUE;
    }

    if (!flash_error)
    {
      for (i=0; i<8; ++i)
      {
        ADCcalHigh.gain[i] = tempADCcal.gain[i];
        ADCcalHigh.offset[i] = tempADCcal.offset[i];
      }
    }
    else
    {
      for (i=0; i<4; ++i)
      {
        ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_IGAIN_HIGH;
        ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
      }
      ADCcalHigh.gain[4] = ADC_CAL_CT_IGAIN_HIGH;               // For CT In
      ADCcalHigh.offset[4] = ADC_CAL_CT_IOFFSET;
      for (i=5; i<8; ++i)
      {
        ADCcalHigh.gain[i] = ADC_CAL_DEFAULT_VGAIN;
        ADCcalHigh.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
      }
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadADCHCalConstants()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadADCHCalConstants1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read ADC high gain circuit calibration constants    
//                      
//  MECHANICS:          This subroutine reads the ADC high gain  calibration constants from FRAM and Flash.
//                      The checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, ADCcalHigh.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read(), Flash_Read(), ComputeChksum32()
// 
//------------------------------------------------------------------------------------------------------------

void ReadADCHCalConstants1(uint8_t *dest)
{
  uint8_t i, err;
  uint32_t chk, chk_rd, chkcmp_rd;
  float *flt_ptr;

  err = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in dest[].
  FRAM_Read(FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)dest);

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)dest, ((ADC_CAL_SIZE >> 2)-2));
  chk_rd = (uint32_t)dest[ADC_CAL_SIZE - 8] + ((uint32_t)dest[ADC_CAL_SIZE - 7] << 8)
                    + ((uint32_t)dest[ADC_CAL_SIZE - 6] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 5] << 24);
  chkcmp_rd = (uint32_t)dest[ADC_CAL_SIZE - 4] + ((uint32_t)dest[ADC_CAL_SIZE - 3] << 8)
                    + ((uint32_t)dest[ADC_CAL_SIZE - 2] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 1] << 24);

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most recent.
  if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in dest[]
    Flash_Read(FLASH_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)dest);

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)dest, ((ADC_CAL_SIZE >> 2)-2));
    chk_rd = (uint32_t)dest[ADC_CAL_SIZE - 8] + ((uint32_t)dest[ADC_CAL_SIZE - 7] << 8)
                      + ((uint32_t)dest[ADC_CAL_SIZE - 6] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 5] << 24);
    chkcmp_rd = (uint32_t)dest[ADC_CAL_SIZE - 4] + ((uint32_t)dest[ADC_CAL_SIZE - 3] << 8)
                      + ((uint32_t)dest[ADC_CAL_SIZE - 2] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 1] << 24);

    // If there is a checksum error, set error flag to use defaults
    if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
    {
      err = TRUE;
    }
  }

  // If both FRAM and Flash are bad, use defaults
  if (err)
  {
    flt_ptr = (float *)(dest);
    for (i = 0; i < 4; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_IGAIN_HIGH;
    }
    *flt_ptr++ = ADC_CAL_CT_IGAIN_HIGH;
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_VGAIN;
    }
    for (i = 0; i < 4; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_IOFFSET;
    }
    *flt_ptr++ = ADC_CAL_CT_IOFFSET;
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_VOFFSET;
    }
  }
  // Otherwise use values that were read

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadADCHCalConstants1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadADCLCalConstants()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read ADC low gain circuit calibration constants    
//                      
//  MECHANICS:          This subroutine reads the ADC low gain  calibration constants from FRAM and Flash.
//                      The checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, ADCcalLow.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read(), Flash_Read(), ComputeChksum32()
// 
//------------------------------------------------------------------------------------------------------------

void ReadADCLCalConstants(void)
{
  uint8_t fram_error, flash_error, i;
  uint32_t chk;
  struct ADC_CAL tempADCcal;

  fram_error = FALSE;
  flash_error = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in the
  //   permanent structure (AFEcal).  We will assume these are good
  FRAM_Read(FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));

  // If there is a checksum error, set error flag
  if ( (ADCcalLow.chk != chk) || (ADCcalLow.cmp != ~chk) )
  {
    SystemFlags |= CAL_FRAM_ERR;
    fram_error = TRUE;
  }

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most current.    */
  if (fram_error)
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in the
    //   temporary structure (tempADCcal).
    Flash_Read(FLASH_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&tempADCcal.gain[0]) );

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)(&tempADCcal.gain[0]), ((ADC_CAL_SIZE >> 2)-2));

    // If there is a checksum error, set error flag
    if ( (tempADCcal.chk != chk) || (tempADCcal.cmp != ~chk) )
    {
      SystemFlags |= CAL_FLASH_ERR;
      flash_error = TRUE;
    }

    if (!flash_error)
    {
      for (i=0; i<8; ++i)
      {
        ADCcalLow.gain[i] = tempADCcal.gain[i];
        ADCcalLow.offset[i] = tempADCcal.offset[i];
      }
    }
    else
    {
      for (i=0; i<3; ++i)
      {
        ADCcalLow.gain[i] = ADC_CAL_DEFAULT_IGAIN_LOW;
        ADCcalLow.offset[i] = ADC_CAL_DEFAULT_IOFFSET;
      }
      ADCcalLow.gain[4] = ADC_CAL_CT_IGAIN_LOW;                 // For CT In
      ADCcalLow.offset[4] = ADC_CAL_CT_IOFFSET;
      for (i=5; i<8; ++i)
      {
        ADCcalLow.gain[i] = ADC_CAL_DEFAULT_VGAIN;
        ADCcalLow.offset[i] = ADC_CAL_DEFAULT_VOFFSET;
      }
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadADCLCalConstants()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadADCLCalConstants1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read ADC low gain circuit calibration constants    
//                      
//  MECHANICS:          This subroutine reads the ADC low gain  calibration constants from FRAM and Flash.
//                      The checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, ADCcalLow.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read(), Flash_Read(), ComputeChksum32()
// 
//------------------------------------------------------------------------------------------------------------

void ReadADCLCalConstants1(uint8_t *dest)
{
  uint8_t i, err;
  uint32_t chk, chk_rd, chkcmp_rd;
  float *flt_ptr;

  err = FALSE;

  // Read the calibration constants, checksum, and checksum complement from FRAM - store these in dest[].
  FRAM_Read(FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)dest);

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)dest, ((ADC_CAL_SIZE >> 2)-2));
  chk_rd = (uint32_t)dest[ADC_CAL_SIZE - 8] + ((uint32_t)dest[ADC_CAL_SIZE - 7] << 8)
                    + ((uint32_t)dest[ADC_CAL_SIZE - 6] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 5] << 24);
  chkcmp_rd = (uint32_t)dest[ADC_CAL_SIZE - 4] + ((uint32_t)dest[ADC_CAL_SIZE - 3] << 8)
                    + ((uint32_t)dest[ADC_CAL_SIZE - 2] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 1] << 24);

  // If there is a fram_error, use the Flash values if they are good.  Otherwise use the default values.
  //   Note, the FRAM values ahould always be checked and used over the Flash values, because the values are
  //   written to FRAM first, then Flash.  The FRAM values are guaranteed to be the most recent.
  if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
  {
    // Read the calibration constants, checksum, and checksum complement from Flash - store these in dest[]
    Flash_Read(FLASH_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)dest);

    // Compute the checksum and complement values
    chk = ComputeChksum32((uint32_t *)dest, ((ADC_CAL_SIZE >> 2)-2));
    chk_rd = (uint32_t)dest[ADC_CAL_SIZE - 8] + ((uint32_t)dest[ADC_CAL_SIZE - 7] << 8)
                      + ((uint32_t)dest[ADC_CAL_SIZE - 6] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 5] << 24);
    chkcmp_rd = (uint32_t)dest[ADC_CAL_SIZE - 4] + ((uint32_t)dest[ADC_CAL_SIZE - 3] << 8)
                      + ((uint32_t)dest[ADC_CAL_SIZE - 2] << 16) + ((uint32_t)dest[ADC_CAL_SIZE - 1] << 24);

    // If there is a checksum error, set error flag to use defaults
    if ( (chk_rd != chk) || (chkcmp_rd != ~chk) )
    {
      err = TRUE;
    }
  }

  // If both FRAM and Flash are bad, use defaults
  if (err)
  {
    flt_ptr = (float *)(dest);
    for (i = 0; i < 4; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_IGAIN_LOW;
    }
    *flt_ptr++ = ADC_CAL_CT_IGAIN_LOW;
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_VGAIN;
    }
    for (i = 0; i < 4; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_IOFFSET;
    }
    *flt_ptr++ = ADC_CAL_CT_IOFFSET;
    for (i = 0; i < 3; ++i)
    {
      *flt_ptr++ = ADC_CAL_DEFAULT_VOFFSET;
    }
  }
  // Otherwise use values that were read

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadADCLCalConstants1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadInjCalConstants()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Test Injection calibration constants    
//                      
//  MECHANICS:          This subroutine reads the Test Injection calibration constants from FRAM and Flash.
//                      The checksum is computed and checked against the data, and status flags are set or
//                      cleared accordingly.
//                          
//                      REFERENCE: Cypress FM25V05, 512Kb Serial 3V FRAM datasheet Doc 001-84497 Rev E 08/15
//                      
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      
//  INPUTS:             dev - the device for which the cal constants are desired
//                      
//  OUTPUTS:            SystemFlags, AFEcal.x
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read(), Flash_Read(), ComputeChksum32()
// 
//------------------------------------------------------------------------------------------------------------

void ReadInjCalConstants(void)
{
  uint8_t i;
  uint32_t chk;
  struct TESTINJ_CAL tempInjcal;

  // Read the calibration constants, checksum, and checksum complement from Flash - store these in the
  //   temporary structure (tempInjcal).
  Flash_Read(FLASH_TESTINJ, (TESTINJ_CAL_SIZE >> 1), (uint16_t *)(&tempInjcal.midpoint_ph) );

  // Compute the checksum and complement values
  chk = ComputeChksum32((uint32_t *)(&tempInjcal.midpoint_ph), ((TESTINJ_CAL_SIZE >> 2)-2));

  // If there is a checksum error, set the error flag
  if ( (tempInjcal.chk != chk) || (tempInjcal.cmp != ~chk) )
  {
    SystemFlags |= INJ_FLASH_ERR;
  }
  // If the Flash values are good, check the existing values.  If there is a fram_error, use the Flash
  //   values.  Otherwise keep the existing default values.
  else if (SystemFlags & INJ_FRAM_ERR)
  {
    TestInjCal.midpoint_ph = tempInjcal.midpoint_ph;
    TestInjCal.midpoint_gnd = tempInjcal.midpoint_gnd;
    for (i=0; i<4; i++)                                       // *** DAH  DO WE HAVE TO SUPPORT Ig?
    {
      TestInjCal.m_dc[i] = tempInjcal.m_dc[i];
      TestInjCal.b_dc[i] = tempInjcal.b_dc[i];
      TestInjCal.m_sine[i] = tempInjcal.m_sine[i];
      TestInjCal.b_sine[i] = tempInjcal.b_sine[i];
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadInjCalConstants()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadSwitches()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Switches    
//                      
//  MECHANICS:          This subroutine reads and debounces the following switch inputs:
//                          Cause of Trip LED Reset (MICRO_RESET)
//                          Breaker Closed (BREAKER_CLOSED)
//                      Switch inputs are read once during initialization, then every sample interrupt
//                      during normal operation
//                      If "firstuse" is True (one-time read during initialization)...
//                        Set the switch and debounce states based only on the state of corresponding input
//                      If "firstuse" is False (during the sample interrupt)..
//                        Left-shift the debounce double-word and insert the current switch state into bit 0
//                        If the debounce double-word is all ones, set the switch state to one
//                        If the debounce doublw-word is all zeros, set the switch state to zero
//                        Otherwise do not change the switch state
//                      This provides a 32 * line-cycle/80 debounce time...
//                            6.7 ms @ 60Hz, 8.0ms @ 50Hz
//
//                      For the Cause of Trip reset switch:
//                          - If the pushbutton is pressed (0 to 1 transition), set the LED code
//                            (COT_AUX.COT_code) to 0 and set the flag to write the value to FRAM.  The LEDs
//                            do not need to be cleared by the micro, because the pushbutton does that in
//                            hardware.  Also set the flag to turn off test injection, if test injection is
//                            running
//
//  CAVEATS:            None
//                      
//  INPUTS:             firstuse - True = the first time the subroutine is called (during initialization)
//                      COT_AUX.AUX_Cycle_Ctr, TestInj.Flags
//                      
//  OUTPUTS:            COT_AUX.COT_code, TestInj.Flags, OpenFlg
//                      
//  ALTERS:             COT_AUX.COT_SwitchState, COT_AUX.COT_Debounce, COT_AUX.AUX_OpenFlg,
//                      COT_AUX.AUX_Debounce, COT_AUX.AUX_Cycle_Ctr
//                      
//  CALLS:              WriteCauseOfTripLEDS()
//
// EXECUTION TIME:      Measured on 180625 (rev 0.25 code): 1.8usec
//
//------------------------------------------------------------------------------------------------------------

void ReadSwitches(uint8_t first_use)
{

  if (first_use)                        // If this is the first call...
  {

    // Breaker closed switch (uC_BREAKER_CLOSED) - set flag and preload debounce double word according to
    //   the initial switch state
    if (BREAKER_CLOSED)                                // *** DAH  NOT SURE WHETHER WE NEED COT_AUX.AUX_OpenFlg ANYMORE, SINCE WE ARE USING OPENFLG
    {                                                  // ALSO MAY BE BETTER TO USE A COUNTER THAT COUNTS THE NUMBER OF CONSECUTIVE READINGS
      COT_AUX.AUX_OpenFlg = 0;                         // AND A "PRESENT STATE" VARIABLE INSTEAD OF A SHIFT REGISTER FOR DEBOUNCING
      COT_AUX.AUX_Debounce = 0xFFFFFFFF;               // THIS MAY BE MORE EFFICIENT RAM-WISE, PLUS WE CAN THEN DEBOUNCE TO A CUSTOM NUMBER OF
      OpenFlg = 0;                                     // PASSES (> 32 IF NECESSARY, WITH UINT8 COUNTER COULD BE AS HIGH AS 256)
    }
    else
    {
      COT_AUX.AUX_OpenFlg = 1;
      COT_AUX.AUX_Debounce = 0x00000000;
      OpenFlg = 1;
    }

    // Cause of Trip pushbutton (MICRO_RESET) - initialize to button not pushed
    COT_AUX.COT_Debounce = 0x00000000;
    COT_AUX.COT_SwitchState = 0;
    COT_AUX.AUX_Cycle_Ctr = 0;              // Also initialize debounce timer

    // *** DAH  CODE ADDED TO RESET EVENTS FOR DEMO
    sw_count = 0;                           // Initialize counter
    oldstate = COT_AUX.COT_SwitchState;     // Initialize old and new states
    // *** DAH  END OF CODE ADDED TO RESET EVENTS FOR DEMO
    
    
    // Maintenance Mode slide switch 
    if (MM_SWITCH_VALUE == 0)                    // indicates MM switch turn on
    {
      COT_AUX.MM_SwitchState = TRUE;
      COT_AUX.MM_Debounce = 0xFFFF;              // Preload the debounce word with all 1's
    }
    else
    {                                            // MM_SWITCH_VALUE=1 indicates MM switch turn off
      COT_AUX.MM_SwitchState = FALSE;
      COT_AUX.MM_Debounce = 0x0000;              // Preload the debounce word with all 0's
    }
        
  }
  else                                  // Not first call...
  {
    // Breaker closed switch (uC_BREAKER_CLOSED) - read and process the aux switch input
    COT_AUX.AUX_Debounce = (COT_AUX.AUX_Debounce << 1);     // Left shift debounce double word - clear lsb
                                                            //   (assume the switch is open)
    if (BREAKER_CLOSED == 1)                                // If aux switch is open, the breaker is closed,
    {                                                       //   so set bit 0 in the debounce double word
      COT_AUX.AUX_Debounce |= 0x00000001;
    }
    if (COT_AUX.AUX_Debounce == 0xFFFFFFFF)                 // Change OpenFlg only if all bits are the same
    {
/*      if ( COT_AUX.AUX_OpenFlg == 1)                      // *** DAH   Request status update on state change
      {
        TU_State.UpdateStatus = 1;
      }               */
      COT_AUX.AUX_OpenFlg = 0;   
      OpenFlg = 0;
    }
    else if (COT_AUX.AUX_Debounce == 0x00000000)            // If all bits are 0, the aux switch is closed,
    {                                                       //   and the breaker is open
      // Increment the operations counter on closed->open transitions if the open delay counter has timed
      //   out.  This counter prevents spurious cycle counts if the aux switch bounces.
      if ((COT_AUX.AUX_OpenFlg == 0) && (COT_AUX.AUX_Cycle_Ctr == 0))
      {
/*        DINT;
        Get_DateTime(&Health_Data.data.Open_DTS);
        EINT;
        Health_Data.data.Open_Count++;

        if ((InstTripFlg == 1) || (McrHiTripFlg == 1))      // Increment the high-current operations counter
        {                                                   //   if any of the following trip flags are set:
          Health_Data.data.HighTrip_Count++;                //     InstTripFlg, McrHiTripFlg
        }
        else if ( (Flags0.all & 0x8039)                     // Increment the low-current operations counter
              || (Flags1.all & 0x0001) )                    //   if any of the following trip flags are set:
        {                                                   //     LdTripFlg, GndTripFlg, PlossTripFlg,
          Health_Data.data.LowTrip_Count++;                 //     AunbalTripFlg, MM_TripFlg, NeuTripFlg
        }
        Cycle_Ctr = 100;                                    // Set operations ctr inhibit timer for 1 sec
        MFlags0.NewHealthFlg = 1;                           // Update FRAM copy of health info.
                                                            // Clear associated state change timers
        Cam_Port.RTData_Timers[8] = 0;
        Pnl_Port.RTData_Timers[8] = 0;   */
      }
      if (COT_AUX.AUX_OpenFlg == 0)             //    Request status update on state change
      {
/*        TU_State.UpdateStatus = 1; */
      }
      COT_AUX.AUX_OpenFlg = 1;   
      OpenFlg = 1;
    }

    // Cause of Trip pushbutton (MICRO_RESET) - read and process the pushbutton input
    COT_AUX.COT_Debounce = (COT_AUX.COT_Debounce << 1);     // Left shift debounce word - clear lsb (assume
                                                            //   button not pressed)
    if (COT_RESET_HIGH)                                     // If pushbutton pressed, set bit 0
    {
      COT_AUX.COT_Debounce |= 0x00000001;
    }
    if (COT_AUX.COT_Debounce == 0xFFFFFFFF)                 // If button debounced and pressed...,
    {
      // If button state changed from 0 to 1, we have just detected the pushbutton being pressed.  Clear the
      //   Cause of Trip LEDs nonvolatile mask.  Only do this on the transition, so that if the button is
      //   stuck on, we will not continuously write to FRAM.  The LEDs are cleared by the hardware.
      //   Finally, if Test Injection is on, set the flag to turn test injection off
      if (COT_AUX.COT_SwitchState == 0)                         // COT_SwitchState has old button state
      {
        COT_AUX.COT_Code = 0;                                   // *** DAH SET FLAG TO WRITE TO FRAM
        if (TestInj.Flags & TEST_INJ_ON)
        {
          TestInj.Flags |= TEST_INJ_INIT_OFF;
        }
      }
      TRIP_LED_OFF;                                             // Turn off the Trip LED
      TripFlagsReset();                                         // Clear the trip flags
      COT_AUX.COT_SwitchState = 1;                              // Update button state
    }
    else if (COT_AUX.COT_Debounce == 0x00000000)            // If button debounced and released...
    {
      COT_AUX.COT_SwitchState = 0;                              // Update button state
    }                                       // Otherwise button is bouncing - leave as is

    // *** DAH  CODE ADDED TO RESET EVENTS FOR DEMO
    if ( (oldstate == 0) && (COT_RESET_HIGH) )  // If button has been pushed, increment counter and check
    {                                           //   how long it has been pushed
      if (++sw_count >= 48000)                  // 80counts/cyc * 60cyc/sec * 10sec = 48000
      {                                         // If button pushed for 10sec
        ClearEvents();                          //   clear events
        sw_count = 0;                           //   reset count
        oldstate = 1;                           //   set old state to high
      }
    }
    else if ( (oldstate == 1) && (!COT_RESET_HIGH) )  // If button released, increment counter and check
    {                                                 //   how long it has been released
      if (++sw_count >= 4800)                         // 80counts/cyc * 60cyc/sec * 1sec
      {                                               // If button released for 1sec
        sw_count = 0;                                 //   reset count
        oldstate = 0;                                 //   set old state to low
      }
    }
    else                                        // Otherwise button matches the state, do nothing but
    {                                           //   make sure counter is clear
      sw_count = 0;
    }
    // *** DAH  END OF CODE ADDED TO RESET EVENTS FOR DEMO

   
    // Maintenance Mode switch
    COT_AUX.MM_Debounce = (COT_AUX.MM_Debounce << 1);    // Left shift debounce word - clear LSB

    if (MM_SWITCH_VALUE == 0)                        // if MM switch turn on, set bit 1
    {
      COT_AUX.MM_Debounce |= 0x0001;
    }
                                                          // Change MMSwitch_State only if all bits are the same
    if (COT_AUX.MM_Debounce == 0xFFFF)                    // only when debounce=0xFFFF indicates MM switch turn off there.
    {
      COT_AUX.MM_SwitchState = TRUE;
    }

    if (COT_AUX.MM_Debounce == 0x0000)                    // only when debounce=0x0000 indicates MM switch turn on there.
    {
      COT_AUX.MM_SwitchState = FALSE;
    }
    
  }
}                                      

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadSwitches()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       WriteCauseOfTripLEDS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Cause of Trip LEDs
//
//  MECHANICS:          This subroutine sets the state of the Cause of Trip LEDs according to
//                      COT_AUX.COT_Code.  The following procedure is followed:
//                          Entry state:
//                            LED data signals all low (off)
//                            LED strobe signal is configured as an input so when reset button is pushed,
//                              no current is drawn from the battery
//                            LED strobe signal output data register (ODR) is set high so signal will be
//                              high when strobe is configured to be an output
//                          (1) Turn on the LED signal according to COT_AUX.COT_Code.  Only one LED will be
//                              turned on.  If the code is invalid, all of the LEDs will be turned off
//                          (2) Turn on the strobe signal - this reconfigures the port line from an input to
//                              a push-pull output, and drives the port line high
//                          (3) Delay about 4usec to ensure the signal at the latch goes high, as there is
//                              an RC filter on the signal
//                          (4) Lower the strobe signal
//                          (5) Delay about 4usec to ensure the signal at the latch goes low, as there is an
//                              RC filter on the signal.  We want to make sure the signal is low before
//                              changing the inputs to the latch, in case the strobe signal has noise on it
//                              (don't want a false state to be written due to noise)
//                          (6) Switch the strobe signal back to an input
//                          (7) Set the strobe ODR register to high so the output will be high when the
//                              strobe port is reconfigured as an output
//                          (8) Lower all of the LED control lines so that if the reset pushbutton is
//                              pushed, the LEDs will be cleared
//                      
//  CAVEATS:            None
//
//  INPUTS:             COT_AUX.COT_Code
// 
//  OUTPUTS:            Cause of Trip LEDs
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void WriteCauseOfTripLEDS(void)
{
  uint8_t i;

  if (COT_AUX.COT_Code == 'I')          // Instantaneous Indicator
  {
    INST_LED_ON;
  }
  else if (COT_AUX.COT_Code == 'S')     // Short Delay Indicator
  {
    SD_LED_ON;
  }
  else if (COT_AUX.COT_Code == 'G')     // Ground/High Load Indicator
  {
    GND_HL_LED_ON;
  }
  else if (COT_AUX.COT_Code == 'L')     // Long Delay Indicator
  {
    LONG_LED_ON;
  }

  // Turn on the strobe signal.  Note, the signal will be high because the ODR has already been set to high
  TRIP_LED_STROBE_ON;

  i = 100;                              // Delay for RC filter on strobe signal
  while (i > 0)                         // Measured clock pulse width on 170214: 3.25usec (spec is only 6ns
  {                                     //   for min clock pulse width)
    --i;
  }
  // Lower the strobe signal.  Note, we will drive the signal low with the port pin to speed the fall time.
  //   There is an RC network and the capacitor must discharge
  TRIP_LED_STROBE_LOW;

  i = 100;                              // Delay for RC filter on strobe signal
  while (i > 0)                         // Measured on 161115: 4.25usec
  {                                     // Measured clock pulse fall time on 170214: 2.1usec
    --i;
  }

  // Switch the strobe signal back to an input.  The external pull-down resistor keeps the signal low
  TRIP_LED_STROBE_OFF;

  // Set the strobe signal output data register high to be ready for the next time
  TRIP_LED_STROBE_HIGH;

  TRIP_LEDS_OFF;                      // Turn the LEDs off when done so that if pushbutton is pressed, LEDs
                                      //   are cleared
}


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         WriteCauseOfTripLEDS()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       StartupTimeCal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calibrate Startup Time Circuit
//
//  MECHANICS:          This subroutine calibrates the startup time circuit
//                      
//  CAVEATS:            None
//
//  INPUTS:             StartupTime.State, StartupTime.DoCalFlag, SystemFlags, PickupFlags, SysClk_120MHz
// 
//  OUTPUTS:            Startup time scaling constant is written to FRAM
//
//  ALTERS:             StartupTime.State, StartupTime.OnTime, StartupTime.OffTime, StartupTime.DoCalFlag
// 
//  CALLS:              FRAM_Write()
// 
//------------------------------------------------------------------------------------------------------------

void StartupTimeCal(void)
{
  float ElapsedTime;
  union float_uint32
  {
    float flt_val;
    uint32_t u_val;
  } temp[2];

  switch (StartupTime.State)
  {
    case 0:                             // Idle State
      // If flag set to do calibration and we have already read the startup time, fall into State 1
      if ( (StartupTime.DoCalFlag) && (!(SystemFlags & FIRST_PASS)) )
      {
        StartupTime.State = 1;
      }
      // Otherwise remain in this state and exit
      else
      {
        break;
      }

    case 1:                             // Capture time and Turn on charging circuit
      if (PickupFlags == 0)                 // If not in pickup, ok to proceed
      {
        StartupTime.OnTime = (float)SysTick->VAL;    // Capture the time
//TESTPIN_D1_HIGH;
        ST_DISCHARGE_OFF;                            // Turn off the cap discharge
        StartupTime.State = 2;
      }
      break;

    case 2:                             // Check Time and Read Cap Voltage
      // Set the ADC3 channel for the startup time cap voltage to get ready to convert
      ADC3->SQR3 = ADC3_STARTUP_CH;

      // Capture the time
      StartupTime.OffTime = (float)SysTick->VAL;

      // Compute the elapsed time.  Remember, SysTick is a down counter!!
      if (StartupTime.OffTime <= StartupTime.OnTime)                // No rollover
      {
        ElapsedTime = StartupTime.OnTime - StartupTime.OffTime;
      }
      else                                                          // Rollover
      {
        ElapsedTime = (float)( (SysClk_120MHz == TRUE) ? (COUNT_120MHZ_10MSEC + 1) : (COUNT_16MHZ_10MSEC + 1) );
        ElapsedTime = (ElapsedTime + StartupTime.OnTime) - StartupTime.OffTime;
      }
      if (SysClk_120MHz == TRUE)
      {
        ElapsedTime = ElapsedTime * (8.0/120E6);
      }
      else
      {
        ElapsedTime = ElapsedTime * (8.0/16E6);
      }

      // If the elapsed time is between 5msec and 9msec, read the startup cap voltage and compute the
      //   scaling constant
      if ( (ElapsedTime >= 5.0E-3) && (ElapsedTime <= 9.0E-3) )
      {
        ADC3->CR2 |= ADC_CR2_SWSTART;         // Start the conversion
        while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
        {
        }
        StartUpADC = ((uint16_t)ADC3->DR & 0x0FFF);
//TESTPIN_D1_LOW;
        // Testing conducted on 160804 showed scale factor to be low by about 8%, probably because the
        //   3.3V supply ramps up, slowing the capacitor charging.
        temp[0].flt_val =  (ElapsedTime * 1.08)/((float)StartUpADC);
        // Scale factor should be about 2.60E-6, based on testing conducted on 231025
        //   Do reasonability check: +/-20%
        // If in range, adjust scale factor - use filter so no single reading has major affect
        if ((temp[0].flt_val > 2.2E-6) && (temp[0].flt_val < 3.3E-6))
        {
          StartupTime.Scale =  (StartupTime.Scale * 9.0E-1) + (temp[0].flt_val * 1.0E-1);
        }
        ST_DISCHARGE_ON;
        StartupTime.DoCalFlag = FALSE;
        // There are two copies.  Write each one separately in case reset occurs before one of the writes is
        //   completed and the value is corrupted
        temp[0].flt_val = StartupTime.Scale;            // Compute the complement
        temp[1].u_val = ~temp[0].u_val;
        FRAM_Write(DEV_FRAM2, STARTUP_SCALE, 4, (uint16_t *)(&temp[0].u_val));
        FRAM_Write(DEV_FRAM2, STARTUP_SCALE+SECONDBLK_OFFSET, 4, (uint16_t *)(&temp[0].u_val));

        StartupTime.State = 0;
      }                                 // If the elapsed time is not greater than 6msec, remain in this
      break;                            //   state and exit

    default:
      StartupTime.State = 0;
      break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         StartupTimeCal()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ReadTHSensorConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Config of sensor
//
//  MECHANICS:          This subroutine initiliaze the sensor
//                      
//  CAVEATS:            None
//
//------------------------------------------------------------------------------------------------------------

void ReadTHSensorConfig(void)
{
  I2C3->CR1 |= I2C_CR1_PE;
  HLTH_I2C.TxBuf[0] = (ON_BD_SENSORADD << 1) | 0x00;
  HLTH_I2C.TxBuf[1] = PTR_ADD_CONFIG;
  HLTH_I2C.TxBuf[2] = 0x10;
  HLTH_I2C.TxBuf[3] = 0x00;
  HLTH_I2C.TxNdx = 0;
  HLTH_I2C.TxNumChars = 4;
  HLTH_I2C.RxNumChars = 0;
  HLTH_I2C.IntState = I2C3_WRITECONF;
  HLTH_I2C.State = PTR_TEMP;
  I2C3->CR1 |= I2C_CR1_START;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadTHSensorConfig()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ReadTHSensorWriteReg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Requesting the reading out
//
//  MECHANICS:          This subroutine request reading out from sensor
//                      
//  CAVEATS:            None
//
//------------------------------------------------------------------------------------------------------------

void ReadTHSensorWriteReg(void)
{
  I2C3->CR1 |= I2C_CR1_PE;
  HLTH_I2C.TxBuf[0] = (ON_BD_SENSORADD << 1) | 0x00;
  HLTH_I2C.TxBuf[1] = PTR_ADD_TEMP;
  HLTH_I2C.TxNdx = 0;
  HLTH_I2C.TxNumChars = 1;
  HLTH_I2C.RxNumChars = 0;
  HLTH_I2C.IntState = I2C3_WRITEREG;
  I2C3->CR1 |= I2C_CR1_START;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadTHSensorWriteReg()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ReadTHSensorWriteReg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read the bytes
//
//  MECHANICS:          This subroutine reads out 4 bytes from sensor ended by NACKs
//                      
//  CAVEATS:            None
//
//------------------------------------------------------------------------------------------------------------

void ReadTHSensorReadVal(void)
{
  I2C3->CR1 |= I2C_CR1_PE;
  HLTH_I2C.TxBuf[0] = (ON_BD_SENSORADD << 1) | 0x01;
  HLTH_I2C.TxNdx = 0;
  HLTH_I2C.RxNdx = 0;
  HLTH_I2C.TxNumChars = 1;
  HLTH_I2C.RxNumChars = 4;
  HLTH_I2C.IntState = I2C3_READSENS;
  I2C3->CR1 |= I2C_CR1_START;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadTHSensorReadVal()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ReadTHSensorProcVal()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Config of sensor
//
//  MECHANICS:          This subroutine procces 4 bytes of received temp and hum into buffers 
//                      
//  CAVEATS:            None
//
//------------------------------------------------------------------------------------------------------------

void ReadTHSensorProcVal(void)
{
  uint16_t temp;
  struct INTERNAL_TIME presenttime;

  temp = HLTH_I2C.RxBuf[2] + (((uint16_t)HLTH_I2C.RxBuf[1]) << 8);

  THSensor.Temperature = ((float)temp * 165.0)/65535.0 - 40.0;
  if ( (THSensor.Temperature > THSensor.TemperatureMax )
    || (THSensor.Temperature > THSensor.TemperatureMaxInt) )
  {
    __disable_irq();
    Get_InternalTime(&presenttime);
    __enable_irq();
  }
  if (THSensor.Temperature > THSensor.TemperatureMax)
  {
    THSensor.TemperatureMax = THSensor.Temperature;
    THSensor.TemperatureMaxTS = presenttime;
    // *** DAH  NEED TO STORE IN FRAM - ADD FLAG TO UPDATE MIN/MAX VALUES IN FRAM
  }
  if (THSensor.Temperature > THSensor.TemperatureMaxInt)
  {
    THSensor.TemperatureMaxInt = THSensor.Temperature;
    THSensor.TemperatureMaxIntTS = presenttime;
    HLTH_I2C.State = PTR_HUM;
    // *** DAH  NEED TO STORE IN FRAM - ADD FLAG TO UPDATE MIN/MAX VALUES IN FRAM
  }
  
  temp = HLTH_I2C.RxBuf[4] + (((uint16_t)HLTH_I2C.RxBuf[3]) << 8);
  THSensor.Humidity = ((float)temp * 100.0)/65535.0;
  HLTH_I2C.State = PTR_TEMP;
  
  HLTH_I2C.Flags = I2C_WRITE_REG;
  HLTH_I2C.IntState = I2C3_WRITEREG;
  HLTH_I2C.TxNdx = 0;
  HLTH_I2C.RxNdx = 0;
  HLTH_I2C.TxNumChars = 0;
  HLTH_I2C.RxNumChars = 0;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadTHSensorProcVal()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadTHSensor()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Temperature/Humidity Sensor    
//                      
//  MECHANICS:          This subroutine reads the on-board HDC1010 Temperature/Humidity Sensor via the I2C3
//                      bus.  The following steps are taken to perform the read:
//                          State I2C_WRITE_REG:
//                                writting the registr and trigger the read request
//                          State I2C_READ_SENSOR: 
//                              When TEMP_SENSOR_DRDYN (sensor is sready to answer) then read. If TEMP_SENSOR_DRDYN
//                              failes  then restart the sensor and increas error count
//                          State I2C_PROC_VAL: 
//                              Compute and store the temperature and humidity and set the State to 3 to
//                                repeat the process (no longer need to configure the sensor)
//                      If there is an error when trying to read the sensor, the on-board I2C3 peripheral is
//                      reinitialized.
//
//  CAVEATS:            None
//                      
//  INPUTS:             HLTH_I2C.Flags, TEMP_SENSOR_DRDYN
//                      
//  OUTPUTS:            THSensor.Humidity, THSensor.Temperature, HLTH_I2C.TxBuf[], HLTH_I2C.TxNdx,
//                      HLTH_I2C.TxNumChars, HLTH_I2C.RxNumChars, HLTH_I2C.IntState, HLTH_I2C.Flags
//
//  ALTERS:             I2C3->CR1, HLTH_I2C.State
//
//  CALLS:              Init_I2C3(), ReadTHSensorConfig(), ReadTHSensorWriteReg(), ReadTHSensorReadVal(), ReadTHSensorProcVal()
// 
//------------------------------------------------------------------------------------------------------------

void ReadTHSensor(void)
{

  if (HLTH_I2C.Flags == I2C_ERROR)          // If error, just keep old values
  {
    HLTH_I2C.Flags &= (~READ_SENSOR);
    Init_I2C3();
    ReadTHSensorConfig();
    if (HLTH_I2C.ErrCount == UINT8_MAX)
    {
      HLTH_I2C.ErrCount = 0;
    }
  }

  switch (HLTH_I2C.Flags)
  {
    case I2C_WRITE_REG:
      ReadTHSensorWriteReg();
    break;
    
    case I2C_READ_SENSOR:
      if (TEMP_SENSOR_DRDYN) 
      {
        ReadTHSensorReadVal();
      }
      else
      {
        HLTH_I2C.Flags = I2C_ERROR;
        HLTH_I2C.ErrCount++;
      }
    break;
      
    case I2C_PROC_VAL:
      ReadTHSensorProcVal();
    break;

    default:
      HLTH_I2C.Flags = I2C_ERROR;
      HLTH_I2C.ErrCount++;
      break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadTHSensor()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ProcessTimeAdjustment()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Real Time Adjustment
//
//  MECHANICS:          This subroutine is called whenever time is received (from either of the CAMs, the
//                      display processor, or the test port).  It examines the time change to determine
//                      whether whether a forced demand logging event should occur and an event should be
//                      logged.
//                      An event is logged (the time adjustment is considered major) if time is changed by
//                      more than one second.  This is the minimum amount to cause a break in the demand log
//                          
//                      Note, the RTC is only read on power up and so this subroutine is not used - it is
//                      assumed the time adjustment is major (i.e., power was down for more than one second)
//
//  CAVEATS:            None
//
//  INPUTS:             *old_time_ptr - the existing time
//                      *new_time_ptr - the new time
// 
//  OUTPUTS:            Dmnd_SaveEvent.xx, Dmnd_ForceLog, SystemFlags (SKIP_LOOPTIME_MEAS)
//
//  ALTERS:             None 
// 
//  CALLS:              FRAM_Write()
// 
//------------------------------------------------------------------------------------------------------------

void ProcessTimeAdjustment(struct INTERNAL_TIME *old_time_ptr, struct INTERNAL_TIME *new_time_ptr)
{
  uint32_t diffsecs, uval[2];
  volatile uint32_t diffnsec;
  
  // Compute the magnitude and direction of the time change
  if ( (new_time_ptr->Time_secs > old_time_ptr->Time_secs)          // Forward adjustment
    || ( (new_time_ptr->Time_secs == old_time_ptr->Time_secs)
      && (new_time_ptr->Time_nsec >= old_time_ptr->Time_nsec) ) )
  {
    diffsecs = new_time_ptr->Time_secs - old_time_ptr->Time_secs;
    if (new_time_ptr->Time_nsec >= old_time_ptr->Time_nsec)
    {
      diffnsec = new_time_ptr->Time_nsec - old_time_ptr->Time_nsec;
    }
    else
    {
      --diffsecs;
      diffnsec = (1000000000L + new_time_ptr->Time_nsec) - old_time_ptr->Time_nsec;
    }
  }
  else                                                              // Backward adjustment
  {
    diffsecs = old_time_ptr->Time_secs - new_time_ptr->Time_secs;
    if (old_time_ptr->Time_nsec >= new_time_ptr->Time_nsec)
    {
      diffnsec = old_time_ptr->Time_nsec - new_time_ptr->Time_nsec;
    }
    else
    {
      --diffsecs;
      diffnsec = (1000000000L + old_time_ptr->Time_nsec) - new_time_ptr->Time_nsec;
    }
  }

  // If magnitude of the change is less than one second, we do not need to do anything.  This is considered
  // a minor time adjustment.
  // If the magnitude of the change is more than one second, it is considered a major time adjustment and
  // must be logged.  In addition, it affects demand logging and so the Dmnd_ForceLog flag is set to force a
  // demand window and logging anniversary.  The Event ID is incremented for the new demand logs - the new
  // EID matches the EID for the Time Adjustment event.  The time stamp of the forced demand log is the old
  // time at the time of the adjustment.
  if (diffsecs != 0)
  {
    Dmnd_SaveEvent.EID = EventMasterEID++;                                  // Save the EID amd old time for
    uval[0] = EventMasterEID;                                               //   the next demand log
    uval[1] = (~EventMasterEID);
    FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval[0]));
    FRAM_Write(DEV_FRAM2, (MASTER_EID+SECONDBLK_OFFSET), 4, (uint16_t *)(&uval[0]));
    Dmnd_SaveEvent.TS.Time_secs = old_time_ptr->Time_secs;               
    Dmnd_SaveEvent.TS.Time_nsec = old_time_ptr->Time_nsec;
    Dmnd_ForceLog = TRUE;                                                   // Set flag to force demand log

    // *** DAH  ENTER TIME CHANGE EVENT HERE - MAYBE NOT INCLUDE IN THE SUMMARY LOG UNLESS THE TIME CHANGE IS EVEN GREATER
    //          MAYBE MAKE A MAJOR CHANGE GREATER THAN ONE SECOND, BUT IF GREATER THAN ONE SECOND, FORCE A DEMAND LOG ENTRY - NO
    //          THIS WON'T WORK, BECAUSE WILL NEED TO KNOW WHY THE DEMAND LOG EID CHANGED, SO IT HAS TO STAY AT ONE SECOND AND MUST DO THE FORCE LOG
  }

  // Set flag to skip max main loop time measurement since we changed the time
  SystemFlags |= SKIP_LOOPTIME_MEAS;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ProcessTimeAdjustment()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    SPI1_Flash_Manager()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Top-Level Manager of Flash Requests to Access SPI1
//                      
//  MECHANICS:          This subroutine manages the operation of SPI1 for requests that involve the Flash
//                      chip.  These are prioritized requests, and typically require multiple steps to
//                      complete.  SPI1 is used to perform the following tasks:
//                           1) Write calibration constants to FLASH (USB, test port)
//                           2) Write the Test Injection cal constants to Flash
//                           3) Write demand and energy logs to FRAM or Flash
//                           4) Read demand and energy logs from Flash
//                           5) Write waveform to FLASH
//                           6) Read waveform from FLASH
//                           7) Read calibration constants from FLASH
//                           8) Retrieve the Test Injection cal constants
//                      
//                      Requests are made via bit-mapped flags in the variable SPI1Flash.Req.  Each request
//                      has an assigned priority that is based on its bit location in SPI1Flash.Req.  Higher
//                      priority requests are serviced before lower-priority requests.  Some tasks consist
//                      of more than one step, and so may require more than one flag.  The following
//                      describes the flags:
//                        Flag                  Value         Description                        Priority
//                                              (bit-mask)
//                        S1F_TRIP_WF_ERASE     0x00000001    Erase trip waveform block 1        0 (highest)
//                        S1F_TRIP_WF_ERASE1    0x00000002    Erase trip waveform block 2        1
//                        S1F_TRIP_WF_WR        0x00000004    Write trip waveforms               2
//                        S1F_ALARM_WF_ERASE    0x00000008    Erase alarm waveform block 1       3
//                        S1F_ALARM_WF_ERASE1   0x00000010    Erase alarm waveform block 2       4
//                        S1F_ALARM_WF_WR       0x00000020    Write alarm waveforms              5
//                        S1F_EXTCAP_WF_ERASE   0x00000040    Erase strip-chart waveform blk 1   3
//                        S1F_EXTCAP_WF_ERASE1  0x00000080    Erase strip-chart waveform blk 2   4
//                        S1F_EXTCAP_WF_WR      0x00000100    Write strip-chart waveforms        5
//                        S1F_RD_WF             0x00000200    Read waveform                      6
//                        S1F_CAL_WR            0x00000400    Write (metering) cal constants     6      // *** DAH  MAY NEED TO REARRANGE FOR PRIORITIES (LOWER THE BIT POSITION, HIGHER THE PRIORITY)!!!  DEMAND SHOULD PROBABLY BE LOWER
//                        S1F_INJ_WR            0x00000800    Write test inj cal constants       7
//                        S1F_DMND_WR           0x00001000    Write demand log (dmnd log event)  9
//                        S1F_DMND_ERASE        0x00002000    Erase demand sector                10
//                        S1F_TP_RD_REQ         0x00004000    Test port request for the Flash    11
//                        S1F_CHK_CAL           0x00008000    Check cal constants                12
//                        S1F_INJ_RD            0x00010000    Read test inj cal constants        13
//                        S1F_AFECAL_RD         0x00020000    Read AFE cal constants             14
//                        S1F_ADCHCAL_RD        0x00040000    Read ADCH cal constants            14
//                        S1F_ADCLCAL_RD        0x00080000    Read ADCL cal constants            14
//                        S1F_AFECAL_RD1        0x00100000    Read AFE cal consts into SPI2_buf  14
//                        S1F_ADCHCAL_RD1       0x00200000    Read ADCH cal consts into SPI2_buf 14
//                        S1F_ADCLCAL_RD1       0x00400000    Read ADCL cal consts into SPI2_buf 14
//
//                      When the requested task has been completed, the corresponding acknowledge flag is
//                      set.  The calling subroutine should then clear the request flag.  When the request
//                      flag is cleared, this subroutine then clears the corresponding acknowledge flag.
//                      The sequence is illustrated below:
//                        Foreground subroutine sets Req Flag
//                                  ISR (this subroutine) reads Req Flag
//                                  ISR begins begins requested task
//                        Foreground subroutine reads Ack Flag to see whether task has been completed
//                                  ISR (this subroutine) completes task and sets Ack Flag
//                                  ISR (this subroutine) does not act on Req Flag if Ack Flag is set
//                        Foreground reads Ack Flag, determines task has been completed, and clears Req Flag
//                                  ISR (this subroutine) reads Req Flag, sees it is clear, and clears Ack Flag
//
//  CAVEATS:            None
//                      
//  INPUTS:             FlashCtrlReq
//                      
//  OUTPUTS:            FlashCtrlAck
//                      
//  ALTERS:             SPI1Flash.State
//                      
//  CALLS:              FRAM_Write(), FRAM_Read(), Flash_PageWrite(), Flash_EraseSectorBlock(),
//                      StoreCalFlash(), RTC_Read(), FRAM_Stat_Write(), Flash_Read(), ClearEvents()
//
// *** DAH  MEASURE EXECUTION TIME FOR EACH STEP - REMEMBER, THIS GETS CALLED EVERY 1.5MSEC, SO IT COULD BE CALLED 8 TIMES IN A 12MSEC MAIN LOOP
//          WE MUST MAKE SURE IT DOESN'T EXTEND THE MAIN LOOP BY TOO MUCH!!!!
// 
//------------------------------------------------------------------------------------------------------------

void SPI1_Flash_Manager(void)
{
  uint32_t bmsk, indx, temp32;
  union temp_cal_structs
  {
    struct AFE_CAL AFEcal;
    struct ADC_CAL ADCcal;
  } temp;
  uint8_t S1F_exit;
  uint8_t i, j, num, cycnum;

  S1F_exit = FALSE;                     // Initialize exit flag to False

  while (!S1F_exit)
  {
    switch (SPI1Flash.State)
    {

      case S1F_IDLE:                    // Check for requests
        S1F_exit = TRUE;                    // Assume no requests, so will exit
        SPI1Flash.Ack &= SPI1Flash.Req;     // Clear any ack bits whose corresponding request bits are clear
        if (SPI1Flash.Req != 0)             // If we have a request, find it.  Always start at 0 (the
        {                                   //   highest priority)
          for (i=0; i<NUM_SPI1FLASH_REQ; ++i)
          {
            bmsk = (1<<i);
            if ( ((SPI1Flash.Req & bmsk) == bmsk) &&        // If request bit is set AND ack bit is clear,
                 ((SPI1Flash.Ack & bmsk) != bmsk) )         //   there is an unprocessed request (if ack is
            {                                               //   set the request has already been done)...
              SPI1Flash.State = S1F_NEXT_STATE[i];              // Set next state to the state corresponding
              S1F_exit = FALSE;                                 //   to the request, and jump to that state
              break;
            }
          }
        }
        break;

      case S1F_DMND_WRITE1:             // Demand/Energy Log Storage  - 1 (Flash write)
        // Write page in Flash (1.5msec to complete).  Subroutine returns True when done
        if (Flash_PageWrite((EV_Dmnd.NextDmndAdd >> 1), (sizeof(EngyDmnd) >> 1),
                    (uint16_t *)(&EngyDmnd[0].EID)) )
        {
          SPI1Flash.Ack |= S1F_DMND_WR;
          SPI1Flash.State = S1F_IDLE;
        }
        else                                    // If not done with page write, remain in this state and
        {                                       //   exit
          S1F_exit = TRUE;
          break;
        }
        break;

      case S1F_DMND_WRITE2:             // Demand/Energy Log Storage  - 2 (Flash sector erase)
        // Call subroutine to erase sector.  Subroutine returns True when done
        if (Flash_EraseSectorBlock((EV_Dmnd.NextDmndAdd >> 1), TRUE))
        {
          SPI1Flash.Ack |= S1F_DMND_ERASE;  //   because both processes are now complete
          SPI1Flash.State = S1F_IDLE;
        }                                   // If not done with the sector erase, remain in this state
        S1F_exit = TRUE;                    // Exit the subroutine in either case
        break;

      case S1F_CAL_WRITE_FLASH:         // Cal Constants Write to FRAM and Flash
        // Write cal constants to Flash.  Function returns True when done
        if (StoreCalFlash(TRUE))            // When done, set the acknowledge flag and set the state to idle
        {
          SPI1Flash.Ack |= S1F_CAL_WR;
          SPI1Flash.State = S1F_IDLE;
        }                                   // If not done, remain in this state
        S1F_exit = TRUE;                    // Exit in either case
        break;

      case S1F_INJ_WRITE_FLASH:         // Test Injection Cal Constants Write to FRAM and Flash
        // Write cal constants to Flash.  Function returns True when done
        if (StoreCalFlash(FALSE))           // When done, set the ack flag, and set the state to idle
        {                                   //  
          SPI1Flash.Ack |= S1F_INJ_WR;
          SPI1Flash.State = S1F_IDLE;
        }                                   // If not done, remain in this state
        S1F_exit = TRUE;                    // Exit in either case
        break;

      case S1F_TP_READ:                 // Test Port Read
        if ( (TP.Temp == 37) || (TP.Temp == 38) )   // If indicator = 37 or 38, demand read
        {
          indx = (uint32_t)TP.ValPtr5;              // Move the index into the temporary
          // Assemble the Flash address of the demand log
          //   Address is:
          //       starting address of the demand logs (ENERGY_SECTOR_START << 12)
          //     + page address (index / 2 * 256)
          //     + byte address (0 if index is even, DEMAND_SIZE if odd)
          temp32 = (indx/2 * 256) + ( ((uint32_t)(ENERGY_SECTOR_START)) << 12 );
          temp32 += (((indx & 0x00000001) == 1) ? DEMAND_SIZE : 0);
        
          // Note, events are not stored in the ENERGY_SECTOR_END sector.  ENERGY_SECTOR_END marks the end
          //   of events storage
          if (temp32 >= ((uint32_t)(ENERGY_SECTOR_END) << 12))
          {
            temp32 = ( ((uint32_t)(ENERGY_SECTOR_START)) << 12 );
            TP.ValPtr5 = 0;
          }
          TP.Tmp2.u = temp32;                       // Save address to display
          // Read the demand log from flash from the address in temp32.  Stored in tbuf[]
          Flash_Read( temp32, (DEMAND_SIZE>>1), (uint16_t *)(&tbuf.d.EID) );
          SPI1Flash.Ack |= S1F_TP_RD_REQ;
          SPI1Flash.State = S1F_IDLE;
          S1F_exit = TRUE;
        }
        else if (TP.Temp < 37)                      // If indicator 0 - 36, read waveform
        {
          // Read samples from the trip, alarm, or strp-chart waveform log from Flash starting with the
          //   address in TP.Tmp2.u.  Store in tbuf[]
          // TP.Temp1.b[3] holds the offset in the sample set.  If less than 20, we are reading currents,
          //   which are floats
          if (TP.Tmp1.b[3] <= 20)
          {
            for (i=0; i<TP.Tmp1.b[0]; ++i)
            {
              Flash_Read(TP.Tmp2.u, 2, (uint16_t *)(&tbuf.fval[i]));
              TP.Tmp2.u += (sizeof(struct RAM_SAMPLES));
            }
          }
          // Otherwise we are reading voltages, which are uint16s
          else
          {
            for (i=0; i<TP.Tmp1.b[0]; ++i)
            {
              Flash_Read(TP.Tmp2.u, 1, (uint16_t *)(&tbuf.i16val[i]));
              TP.Tmp2.u += (sizeof(struct RAM_SAMPLES));
            }
          }
          SPI1Flash.Ack |= S1F_TP_RD_REQ;
          SPI1Flash.State = S1F_IDLE;
          S1F_exit = TRUE;
        }
        else                                        // invalid indicator, just clear the request
        {
          SPI1Flash.Ack |= S1F_TP_RD_REQ;
          SPI1Flash.State = S1F_IDLE;
          S1F_exit = TRUE;
        }
        break;

      case S1F_EXT_WF_WRITE:
        // If a trip waveform capture is requested, abort this capture by setting the abort flag to True.
        //   "i" is used as a temporary abort flag
        i = ((SPI1Flash.Req & S1F_TRIP_WF_WR) == S1F_TRIP_WF_WR);
        // Write the waveform into Flash (1.5msec to complete for each page).  Subroutine returns
        //   True when the entire waveform has been written or the waveform capture has been aborted.
        //   When waveform write is complete, set the Ack flag, reset the state, and clear the capture in
        //   progress flag.
        // Note, the flags to erase the next blocks are set in ManageSPI1Flags().  These are handled as new
        //   requests and may be held off if there is a higher-priority request (like for a trip).  The in
        //   progress flag may be cleared here, and not after the next two blocks are erased, because the
        //   timing allows us to handle another alarm request even if we start storing after the next two
        //   blocks are erased.
        if (Flash_WriteWaveform(&Ext_WF_Capture, i))
        {
          SPI1Flash.Ack |= S1F_EXT_WF_WR;
          SPI1Flash.State = S1F_IDLE;
          Ext_WF_Capture.InProg = FALSE;
          // Make sure that any outstanding Alarm Capture request is cleared.  It is too late to service an
          //   alarm capture request.  The extended capture request was done instead.
          Alarm_WF_Capture.Req = FALSE;
          Alarm_WF_Capture.InProg = FALSE;
        }                                       // If not done with page write, remain in this state and
        S1F_exit = TRUE;                        //   exit
        break;

      case S1F_EXT_WF_BLKERASE:        // Extended-Capture Waveform Block Erase
        // Initialize the flash address
        Ext_WF_Capture.FlashAdd = EXTCAP_WAVEFORM_START
                                        + (Ext_WF_Capture.EV_Add.NextEvntNdx * WF_SIZE_IN_SECTORS);
        Ext_WF_Capture.FlashAdd = (Ext_WF_Capture.FlashAdd << 4);
        SPI1Flash.State = S1F_EXT_WF_BLKERASE1;
//        break;                                 Fall into the next state

      case S1F_EXT_WF_BLKERASE1:       // Extended-Capture Block Erase
        if (Flash_EraseSectorBlock(Ext_WF_Capture.FlashAdd, FALSE))    // Call routine to erase the first
        {                                                               //   block.  Subroutine returns True
          SPI1Flash.Ack |= S1F_EXT_WF_ERASE;                           //   when done
          SPI1Flash.State = S1F_IDLE;
        }                                       // If not done with the block erase, remain in this state
        S1F_exit = TRUE;                        // Exit the subroutine in either case
        break;

      case S1F_EXT_WF_BLKERASE2:       // Extended-Capture Block Erase
        if (Flash_EraseSectorBlock((Ext_WF_Capture.FlashAdd + 0x0100), FALSE) )   // Erase the second block
        {                                                             //   Subroutine returns True when done
          SPI1Flash.Ack |= S1F_EXT_WF_ERASE1;
          SPI1Flash.State = S1F_IDLE;
        }                                           // If not done with block erase, remain in this state
        S1F_exit = TRUE;                            // Exit the subroutine in either case
        break;


      case S1F_TRIP_WF_WRITE:           // Write Trip Waveform
        // Write the waveform into Flash (1.5msec to complete for each page).  Subroutine returns
        //   True when the entire waveform has been written
        if (Flash_WriteWaveform(&Trip_WF_Capture, FALSE))
        {
          SPI1Flash.Ack |= S1F_TRIP_WF_WR;
          SPI1Flash.State = S1F_IDLE;
          Trip_WF_Capture.InProg = FALSE;
          // Make sure that any outstanding Alarm  or Extended Capture requests are cleared.  It is too late
          //   to service either request.  The trip capture request was done instead.
          Alarm_WF_Capture.Req = FALSE;
          Alarm_WF_Capture.InProg = FALSE;
          Ext_WF_Capture.Req = FALSE;
          Ext_WF_Capture.InProg = FALSE;
        }                                       // If not done with page write, remain in this state and
        S1F_exit = TRUE;                        //   exit
        break;

      case S1F_TRIP_WF_BLKERASE:        // Trip Waveform Block Erase
        // Initialize the flash address
        Trip_WF_Capture.FlashAdd = TRIP_WAVEFORMS_START
                                        + (Trip_WF_Capture.EV_Add.NextEvntNdx * WF_SIZE_IN_SECTORS);
        Trip_WF_Capture.FlashAdd = (Trip_WF_Capture.FlashAdd << 4);
        SPI1Flash.State = S1F_TRIP_WF_BLKERASE1;
//        break;                                Fall into the next state

      case S1F_TRIP_WF_BLKERASE1:       // Trip Waveform Block Erase
        if (Flash_EraseSectorBlock(Trip_WF_Capture.FlashAdd, FALSE))    // Call routine to erase the first
        {                                                               //   block.  Subroutine returns True
          SPI1Flash.Ack |= S1F_TRIP_WF_ERASE;                           //   when done
          SPI1Flash.State = S1F_IDLE;
        }                                       // If not done with the block erase, remain in this state
        S1F_exit = TRUE;                        // Exit the subroutine in either case
        break;

      case S1F_TRIP_WF_BLKERASE2:       // Trip Waveform Block Erase
        if (Flash_EraseSectorBlock((Trip_WF_Capture.FlashAdd + 0x0100), FALSE) )   // Erase the second block
        {                                                             //   Subroutine returns True when done
          SPI1Flash.Ack |= S1F_TRIP_WF_ERASE1;
          SPI1Flash.State = S1F_IDLE;
        }                                           // If not done with block erase, remain in this state
        S1F_exit = TRUE;                            // Exit the subroutine in either case
        break;

      case S1F_ALARM_WF_WRITE:          // Write Alarm Waveform
        // If a trip waveform capture is requested, abort this capture by setting the abort flag to True.
        //   "i" is used as a temporary abort flag
        i = ((SPI1Flash.Req & S1F_TRIP_WF_WR) == S1F_TRIP_WF_WR);
        // Write the waveform into Flash (1.5msec to complete for each page).  Subroutine returns
        //   True when the entire waveform has been written or the waveform capture has been aborted.
        //   When waveform write is complete, set the Ack flag, reset the state, and clear the capture in
        //   progress flag.
        // Note, the flags to erase the next blocks are set in ManageSPI1Flags().  These are handled as new
        //   requests and may be held off if there is a higher-priority request (like for a trip).  The in
        //   progress flag may be cleared here, and not after the next two blocks are erased, because the
        //   timing allows us to handle another alarm request even if we start storing after the next two
        //   blocks are erased.
        if (Flash_WriteWaveform(&Alarm_WF_Capture, i))
        {
          SPI1Flash.Ack |= S1F_ALARM_WF_WR;
          SPI1Flash.State = S1F_IDLE;
          Alarm_WF_Capture.InProg = FALSE;
          // Make sure that any outstanding Extended Capture request is cleared.  It is too late to service
          //   an extended capture request.  The alarm capture request was done instead.
          Ext_WF_Capture.Req = FALSE;
          Ext_WF_Capture.InProg = FALSE;
        }                                       // If not done with page write, remain in this state and
        S1F_exit = TRUE;                        //   exit
        break;

      case S1F_ALARM_WF_BLKERASE:       // Alarm Waveform Block Erase
        // Initialize the flash address
        Alarm_WF_Capture.FlashAdd = ALARM_WAVEFORMS_START
                                        + (Alarm_WF_Capture.EV_Add.NextEvntNdx * WF_SIZE_IN_SECTORS);
        Alarm_WF_Capture.FlashAdd = (Alarm_WF_Capture.FlashAdd << 4);
        SPI1Flash.State = S1F_ALARM_WF_BLKERASE1;
//        break;                                Fall into the next state

      case S1F_ALARM_WF_BLKERASE1:      // Alarm Waveform Block Erase
        if (Flash_EraseSectorBlock(Alarm_WF_Capture.FlashAdd, FALSE))   // Call routine to erase the first
        {                                                               //  block.  Subroutine returns True
          SPI1Flash.Ack |= S1F_ALARM_WF_ERASE;                          //  when done
          SPI1Flash.State = S1F_IDLE;
        }                                       // If not done with the block erase, remain in this state
        S1F_exit = TRUE;                        // Exit the subroutine in either case
        break;

      case S1F_ALARM_WF_BLKERASE2:      // Alarm Waveform Block Erase
        if (Flash_EraseSectorBlock((Alarm_WF_Capture.FlashAdd + 0x0100), FALSE) )  // Erase the second block
        {                                                             //   Subroutine returns True when done
          SPI1Flash.Ack |= S1F_ALARM_WF_ERASE1;
          SPI1Flash.State = S1F_IDLE;
        }                                           // If not done with block erase, remain in this state
        S1F_exit = TRUE;                            // Exit the subroutine in either case
        break;

      case S1F_DP_READ_WF:              // Read Waveform for Display Processor Comms
        // Retrieval info is in the RxMsgSav buffer, and DPComm.TxDelMsgBuf[14]:
        //   DPComm.RxMsgSav[3..0]- base address of the first sample
        //   DPComm.RxMsgSav[4]- number of samples to retrieve (starting value is 80 max)
        //   DPComm.RxMsgSav[5]- Flash page count (starting value is 0)
        //   DPComm.RxMsgSav[6]- value offset, indicates whether voltages or currents are being read
        //   DPComm.RxMsgSav[8..7] - starting index to save the samples
        //   DPComm.TxDelMsgBuf[14] - cycle number (0 - 35)

        // Quick addressing overview and example using extended captures
        // Suppose we want Ic of cycle 15 of extended capture at index 3 (7 waveform capacity)
        // Flash memory, for the first sample:
        //   waveform type (trip, alarm, ext cap) ext cap --> starting addr of ext cap waveform captures
        //   EID --> index = 3 --> starting address of the requested waveform capture
        //   cycle number = 15 --> page address of the first sample of the requested cycle = 171
        //   page offset = 0 --> first sample is in the first page 
        //   sample set offset = 3 --> offset within the page of the sample set containing the first sample
        //   value offset = 2 x 4 = 8 --> offset within a sample set of the requested value
        // Flash memory, for later samples:
        //   waveform type (trip, alarm, ext cap) ext cap --> starting addr of ext cap waveform captures
        //   EID --> index = 3 --> starting address of the requested waveform capture
        //   cycle number = 15 --> page address of the first sample of the requested cycle = 171
        //   page offset = x --> offset from the first page of the subsequent samples
        //   sample set offset = 0 --> sample in subsequent pages all start in the first sample set
        //   value offset = 2 x 4 = 8 --> offset within a sample set of the requested value
 
        // Assemble the base address of the first sample
        //   This address accounts for the waveform type (trip, alarm, extended capture), index,
        //   cycle number, and value offset.  These values remain constant for all of the samples
        temp32 = ((uint32_t)DPComm.RxMsgSav[3] << 24) + ((uint32_t)DPComm.RxMsgSav[2] << 16)
               + ((uint32_t)DPComm.RxMsgSav[1] << 8) + ((uint32_t)DPComm.RxMsgSav[0]);
        // Save the page count in i, the cycle number in cycnum, and the starting index for the samples in
        //   indx
        i = DPComm.RxMsgSav[5];
        cycnum = DPComm.TxDelMsgBuf[14];
        indx = (((uint16_t)DPComm.RxMsgSav[8])<< 8) + DPComm.RxMsgSav[7];

        // Add the page offset and sample set offset
        //   If it is the first set of samples (i = 0), the page offset is zero, and the sample set offset
        //   may be non-zero: the first sample is somewhere in the starting page, not necessarily in the
        //   first sample set.  If it is not the first sample (i > 0), the sample set offset is zero, and
        //   the page offset is the page count (in i)
        temp32 += ((i == 0) ? (WF_ADDRESS[cycnum][1] * (sizeof(struct RAM_SAMPLES))) : ((uint32_t)i << 8));
        // Compute the number of samples to read, assuming the entire cycle is in the capture.  If it is the
        //   first set of samples to be read (i = 0), it is 7 (the maximum in a page) minus the starting
        //   offset of the cycle.  If it is the last set of samples to be read
        //   (i = WF_ADDRESS[cycnum][2] - WF_ADDRESS[cycnum][0]), it is the number of samples
        //   sets in the last page.  Otherwise it is seven.
        if (i == 0)
        {
          num = 7 - WF_ADDRESS[cycnum][1];
        }
        else if (i == (WF_ADDRESS[cycnum][2] - WF_ADDRESS[cycnum][0]))
        {
          num = WF_ADDRESS[cycnum][3] + 1;
        }
        else
        {
          num = 7;
        }
        // Clamp the number to read if limited by the number available
        num = ((num > DPComm.RxMsgSav[4]) ? (DPComm.RxMsgSav[4]) : (num));

        // Read samples from Flash
        if (DPComm.RxMsgSav[6] <= 20)       // Currents (4 bytes) are being read
        {
          for (j = 0; j < num; ++j)
          {
            Flash_Read(temp32, 2, (uint16_t *)(&DPComm.TxDelMsgBuf[indx]));
            temp32 += (sizeof(struct RAM_SAMPLES));
            indx += 4;
          }
        }
        else                                // Voltages (2 bytes) are being read
        {
          for (j = 0; j < num; ++j)
          {
            Flash_Read(temp32, 1, (uint16_t *)(&DPComm.TxDelMsgBuf[indx]));
            temp32 += (sizeof(struct RAM_SAMPLES));
            indx += 2;
          }
        }

        DPComm.RxMsgSav[4] -= num;      // Update the number of samples left to retrieve

        // There are no more values to transmit when the page offset (in i) has reached the last page of
        //   the cycle - this assumes the entire cycle was captured.  We are also done if the number of
        //   samples to retrieve (in DPComm.RxMsgSav[4]) has reached zero.
        if ( (i == (WF_ADDRESS[cycnum][2] - WF_ADDRESS[cycnum][0])) || (DPComm.RxMsgSav[4] == 0) )
        {
          SPI1Flash.Ack |= S1F_RD_WF;         // Set the Ack flag since done
          SPI1Flash.State = S1F_IDLE;
          DPComm.TxDelMsgBuf[8] = (uint8_t)(indx - 10);         // LS byte of buffer length
          DPComm.TxDelMsgBuf[9] = (uint8_t)((indx - 10) >> 8);  // MS byte of buffer length
        }
        // Otherwise there are still sample values to read, so increment the page count and save the
        //   parameters.  Remain in this state, but exit
        else
        {
          // Update and save the parameters
          //   DPComm.RxMsgSav[3..0]- base address of the first sample - doesn't change
          //   DPComm.RxMsgSav[4]- number of samples to retrieve - already updated
          //   DPComm.RxMsgSav[5]- Flash page count (starting value is 0) - updated and saved below
          //   DPComm.RxMsgSav[6]- value offset - doesn't change
          //   DPComm.RxMsgSav[8..7]- starting index to save the samples - saved below
          //   DPComm.TxDelMsgBuf[14] - cycle number (0 - 35) - doesn't change
          DPComm.RxMsgSav[5] = ++i;                     // Page count
          DPComm.RxMsgSav[7] = (uint8_t)indx;           // LS byte of index to save the samples
          DPComm.RxMsgSav[8] = (uint8_t)(indx >> 8);    // MS byte of index to save the samples
        }
        S1F_exit = TRUE;
        break;


      case S1F_CHECK_CAL_CONSTS:        // Check Calibration Constants
        // Read the AFE calibration constants, checksum, and checksum complement from Flash - store these in
        //   the temporary structure (temp.AFEcal).  Compute the checksum and complement values and set the
        //   error flag if there is a checksum error
        Flash_Read( FLASH_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&temp.AFEcal.gain[0]) );
        temp32 = ComputeChksum32((uint32_t *)(&temp.AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
        if ( (temp.AFEcal.chk != temp32) || (temp.AFEcal.cmp != ~temp32) )
        {
          SystemFlags |= CAL_FLASH_ERR;
        }
        // Read the high-gain ADC cal constants, checksum, and checksum complement from Flash - store these
        //   in the temporary structure (temp.ADCEcal).  Compute the checksum and complement values and set
        //   the error flag if therre is a checksum error
        Flash_Read(FLASH_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&temp.ADCcal.gain[0]) );
        temp32 = ComputeChksum32((uint32_t *)(&temp.ADCcal.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        if ( (temp.ADCcal.chk != temp32) || (temp.ADCcal.cmp != ~temp32) )
        {
          SystemFlags |= CAL_FLASH_ERR;
        }
        // Read the low-gain ADC cal constants, checksum, and checksum complement from Flash - store these
        //   in the temporary structure (temp.ADCcal).  Compute the checksum and complement values and set
        //   the error flag if therre is a checksum error
        Flash_Read(FLASH_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&temp.ADCcal.gain[0]) );
        temp32 = ComputeChksum32((uint32_t *)(&temp.ADCcal.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        if ( (temp.ADCcal.chk != temp32) || (temp.ADCcal.cmp != ~temp32) )
        {
          SystemFlags |= CAL_FLASH_ERR;
        }
        SPI1Flash.Ack |= S1F_CHK_CAL;               // Set the acknowledge flag
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_INJ_READ_FLASH:          // Test Injection Cal Constants Read
        ReadInjCalConstants();
        SPI1Flash.Ack |= S1F_INJ_RD;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_AFE_CALCONST:       // AFE Cal Constants Read
        ReadAFECalConstants();
        SPI1Flash.Ack |= S1F_AFECAL_RD;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_ADCH_CALCONST:      // ADCH Cal Constants Read
        ReadADCHCalConstants();
        SPI1Flash.Ack |= S1F_ADCHCAL_RD;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_ADCL_CALCONST:      // ADCL Cal Constants Read
        ReadADCLCalConstants();
        SPI1Flash.Ack |= S1F_ADCLCAL_RD;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_AFE_CALCONST1:      // AFE Cal Constants Read into SPI2_buf[]
        // It is ok to use SPI2_buf[] here because we set the request flag in ProcWrFactoryConfig() and then
        //   wait for it to complete.  We are not executing other code while the interrupt is occurring, so
        //   SPI2_buf[] is not in use
        ReadAFECalConstants1(&SPI2_buf[0]);
        SPI1Flash.Ack |= S1F_AFECAL_RD1;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_ADCH_CALCONST1:     // ADCH Cal Constants Read into SPI2_buf[]
        // It is ok to use SPI2_buf[] here because we set the request flag in ProcWrFactoryConfig() and then
        //   wait for it to complete.  We are not executing other code while the interrupt is occurring, so
        //   SPI2_buf[] is not in use
        ReadADCHCalConstants1(&SPI2_buf[0]);
        SPI1Flash.Ack |= S1F_ADCHCAL_RD1;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      case S1F_READ_ADCL_CALCONST1:     // ADCL Cal Constants Read into SPI2_buf[]
        // It is ok to use SPI2_buf[] here because we set the request flag in ProcWrFactoryConfig() and then
        //   wait for it to complete.  We are not executing other code while the interrupt is occurring, so
        //   SPI2_buf[] is not in use
        ReadADCLCalConstants1(&SPI2_buf[0]);
        SPI1Flash.Ack |= S1F_ADCLCAL_RD1;
        SPI1Flash.State = S1F_IDLE;
        S1F_exit = TRUE;
        break;

      default:                          // This state should never be entered
        SPI1Flash.State = S1F_IDLE;
        break;

    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      SPI1_Flash_Manager()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadStartupScaleConstant()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Startup Time Scaling Constant    
//                      
//  MECHANICS:          This subroutine reads the startup time scaling constant and its complement from
//                      FRAM.  If the complement is correct, the constant is stored in the variable
//                      StartupTime.Scale.  If the complement is not correct, the second copy is read.
//                      If it is correct, it is stored in StartupTime.Scale.  If neither copy is correct,
//                      StartupTime.Scale is set to the default value.
//                          
//  CAVEATS:            This subroutine should only be called during initialization.  It assumes SPI2 is
//                      free for use
//                      
//  INPUTS:             None
//                      
//  OUTPUTS:            StartupTime.Scale
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void ReadStartupScaleConstant(void)
{
  union valstruct
  {
    uint32_t uval;
    float fval;
  } val[2];

  // Read the scaling constants from FRAM and store in a temporary
  FRAM_Read(STARTUP_SCALE, 4, (uint16_t *)(&val[0].uval));

  // If the complement isn't correct, read the second copy
  if (val[0].uval != (~val[1].uval))
  {
    FRAM_Read((STARTUP_SCALE+SECONDBLK_OFFSET), 4, (uint16_t *)(&val[0].uval));
    // If the complement isn't correct, set scaling constant to the default value
    if (val[0].uval != (~val[1].uval))
    {
      StartupTime.Scale = 2.60E-6;
    }
    // If complement is correct, use this scaling constant
    else
    {
      StartupTime.Scale = val[0].fval;
    }
  }
  // If complement is correct, use this scaling constant
  else
  {
    StartupTime.Scale = val[0].fval;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadStartupScaleConstant()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//             START OF FUNCTION      MM_Status_Update()
//------------------------------------------------------------------------------
//
//  FUNCTION:           4 MM status defined as below:
//                         #define MMSwitch_State         State_Flags0.bit.b3
//                         #define MMComm_State           State_Flags0.bit.b4
//                         #define MM_Active              Flags2.bit.b0
//
//                      For PXR35, the local switch input is combined with the secondary
//                      pin input
//
//                      MM setpoint defined as below:
//                      uint16_t   MM_Enable;
//                         b08: MM status 1-enable / 0-disable
//                         b07: MM local switch ON / MM signal from secondary pin 
//                         b00: MM setting from remote com channel
//
//                      for MM setting through MM_Enable setpoint:
//                         * for USB/CAM/Modbus, when read MM_Enable setpoint, only
//                           display bit8/bit0 value -- final MM enable status & comm
//                           setting status;
//                           1) for CAM, by self-study setpoint description, always
//                              just send bit8/bit0 value to CAM for display;
//                           2) for Modbus, always mask by BIT8 | BIT0 before sending
//                              out MM_Enable setpoint;
//                           3) send out complete MM_Enable setpoint to PC tool, while
//                              PC tool only display bit8/bit0.
//
//                         * for USB/CAM/Modbus, when write MM_Enable setpoint, only
//                           care about bit0 setting and pass through to MM_Enable bit0,
//                           and ignore other bit setting, since all of them readonly.
//
//                      for MM enable/disable through remote control:
//                         when enable MM, set MM_Enable bit0;
//                         when disable MM, only validate when MM NOT enabled from
//                         local switch OR secondary terminals.
//
//  MECHANICS:          None
//
//  CAVEATS:            None
//
//  INPUTS:             None
//
//  OUTPUTS:            None
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------
void MM_Status_Update(void)
{
  uint16_t mm_setting_updated;
    
  mm_setting_updated = FALSE;

  // Local MM switch setting status (On-board switch or Secondaries)
  // if MM switch == TRUE, while MM setpoint -> switch status still FALSE...
  if ((COT_AUX.MM_SwitchState == TRUE) && ((Setpoints0.stp.MM_Enable & BIT7) == 0))
  {
    // enable MM setpoint -> switch status bit and ensure MM Setpoint -> status bit is set
    Setpoints0.stp.MM_Enable |= (BIT7 | BIT8);              
    //InsertNewEvent(MAINT_MODE, TYPEMOD_MMENTER);   // insert enter MM event
    mm_setting_updated = TRUE;
  }
  // if MM switch == FALSE, while MM setpoint -> switch status still ENABLE...
  else if ((COT_AUX.MM_SwitchState == FALSE) && ((Setpoints0.stp.MM_Enable & BIT7) != 0))
  {
    // if MM remote com channel is also off, ensure all MM bits are cleared
    if ((Setpoints0.stp.MM_Enable & BIT0) == 0)
    {
      Setpoints0.stp.MM_Enable = 0;
    }
    else // otherwise just clear MM local switch
    {
      Setpoints0.stp.MM_Enable &= ~BIT7;
    }
    mm_setting_updated = TRUE;
  }

  // save group 0 setpoints if Maintenance Mode has changed
  if (mm_setting_updated == TRUE)
  {
    uint8_t copy;
    mm_setting_updated = FALSE;
  
    // get the current setpoints for group 0 - setpoints returned in SetpScratchBuf[]
    Get_Setpoints(SetpActiveSet, 0, (uint8_t *)(&copy));
  
    SetpScratchBuf[4] = Setpoints0.stp.MM_Enable;     
    // typically we validate setpoints before saving them, but since the only thing we've changed
    // is MM_enable we don't need to take that extra step, simply save Group 0 of the active set to FRAM.
    Modb_Save_Setpoints(SetpActiveSet, 0);
  }

}
//------------------------------------------------------------------------------
//             END OF FUNCTION        MM_Status_Update()
//------------------------------------------------------------------------------
