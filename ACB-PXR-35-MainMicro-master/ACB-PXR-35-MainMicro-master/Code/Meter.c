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
//  MODULE NAME:        Meter.c
//
//  MECHANICS:          Program module containing the metering subroutines
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
//   0.04   150928  DAH Continued code development.  Added Meter_VarInit(), AFE_Process_Samples(),
//                      AFE_Cal_Scale(), Calc_Current(), and Calc_Current1()
//   0.05   151021  DAH - Replaced constant AFE gain and offset cal constants with RAM constants that are
//                        loaded from FRAM and entered via a test port command
//                          - Revised Meter_VarInit() to retrieve the cal constants from FRAM
//                          - Added default AFE offset and gain cal constants
//                          - Added StoreAFEcal() to write AFE gain and offset cal constants to FRAM
//                          - Added struct AFEcal to hold the cal constants
//                          - Revised AFE_Cal_Scale() to use the new gain and offset cal constants
//                      - Revised AFE_Process_Samples() to skip integrating the current samples if flag is
//                        set via a test port command
//                      - Revised AFE_Cal_Scale() to multiply the sample by the gain constant instead of
//                        divide by the gain constant
//   0.06   151110  DAH - Deleted struct AFE_CAL (moved to Meter_def.h)
//                      - Deleted StoreAFEcal() (replaced with ComputeChksum32() and FRAM_Write() in
//                        Iod.c)
//                      - Revised Meter_VarInit():
//                          - AFEcal structure now has a single checksum for both gain and offset, instead
//                            of separate checksums
//                          - AFEcal is now stored in both FRAM and Flash.  The subroutine now reads and
//                            checks the copies in both chips.  It uses the first good copy (FRAM is read
//                            first)
//                          - Revised AFE_Process_Samples() to eliminate checking the error bits in the
//                            data, except for the Chip_Error bit
//                  BP  - Added ADC_Process_Samples() and ADC_Cal_Scale()
//                      - Added Calc_Temperature()
//   0.07   151216  DAH - In Meter_VarInit(), modified call to FRAM_Read() to include the device parameter
//                      - In Meter_VarInit(), added call to FRAM_Read() to read some Frame FRAM values for
//                        test purposes only
//                      - Renamed .sram1 to .sram2 for readability
//   0.08   160122  DAH - Structure CURRENTS_WITH_G renamed to CUR_WITH_G_F
//                      - Structure CURRENTS_WITHOUT_G renamed to CUR_WITHOUT_G_F
//                      - Corrected error in Calc_Current1()
//                      - Added code in AFE_Process_Samples() set the Status LED to the error state if the
//                        CHIP ERROR bit is set in the AFE samples.  For now, the LED will remain in this
//                        state, even if the error goes away.  We may want to modify this later.
//   0.09   160224  DAH - In Meter_VarInit(), renamed DEV_ONBOARD to DEV_FRAM2
//                      - Changed AFE_CAL_DEFAULT_GAIN and AFE_CAL_DEFAULT_OFFSET for 60Hz R-Frame interface
//                      - Added support for basic seeding of the AFE integrator.  After a reset, the AFE
//                        integrator is seeded with the ADC input.  Samples are not yet aligned, but this
//                        should give us an idea of how well the basic seeding works.
//                          - Added AFE_seed_val[]
//                          - Replaced gain_ADC[] with gain_ADC_low[] and gain_ADC_high[] and for 60Hz
//                            R-Frame interface.  Both the high-gain and low-gain values are read when
//                            obtaining the AFE seed value
//                          - Modified AFE_Integrate_Sample() to use the AFE seed value to seed the
//                            integrator for the first reading
//                          - Modified ADC_Process_Samples() to obtain the seed values
//                          - In ADC_Process_Samples(), fixed the arrangement of the values that are stored
//                            in ADC_samples[]
//                          - Modified ADC_Cal_Scale() to handle both the low-gain and the high-gain inputs
//                      - Changed digital integrator per input from George Gao.  Also modified the scaling
//                        constants to reflect the new coefficients
//                          - AFE_Integrate_Sample() revised
//                      - Modified AFE_Process_Samples() to use the ADC values if the AFE value is invalid
//                        OR if UseADCvals is True.  This flag may be set using the Test Port
//                      - Deleted AFE_CAL_DEFAULT_GAIN and AFE_CAL_DEFAULT_OFFSET (moved to Meter_def.h)
//   0.10   160310  DAH - Added code to support 10-cycle waveform captures via the test port during a trip,
//                        for test purposes only
//                          - Added TestWFind
//                          - In AFE_Process_Samples(), added code to store the AFE pre-integrator sample,
//                            post-integrator sample, ADC sample, and AFE integrated and filtered sample in
//                            a buffer if flag SD_ProtOn is True
//                      - Added DC filtering of the samples for metered values.  This filter, designed by
//                        George Gao, replaces the simple one-second averaging filter.
//                          - Meter_Filter_Sample() added to perform the digital filtering function
//                          - MTR_new_samples[] added to hold the filtered sample values
//                          - MTR_ss[], MTR_b0, MTR_a1 added for the digital filter function
//                          - AFE_Process_Samples() modified to call Meter_Filter_Sample()
//                      - Integration time for metered values changed from one second to 200msec
//                          - CurOneSecSOS_Save renamed to Cur200msecSOS_Sav
//                          - CurOneSec renamed to Cur200msec
//                          - Calc_Current() modified to divide by 960 (12 cycles worth of samples, which
//                            is 200msec at 60Hz) instead of 4800 (one second at 60Hz)
//                          - Renamed structure VolAFEOneSecLLSOS_Sav to VolAFE200msecLLSOS_Sav
//                          - Renamed structure VolAFEOneSecLNSOS_Sav to VolAFE200msecLNSOS_Sav
//                          - Renamed structure PwrOneSecSOS_Sav to Pwr200msecSOS_Sav
//                      - Added min and max calculations for the 200msec values
//                          - Cur200msec_max and Cur200msec_min added
//                          - Modified Calc_Current() to compute the min and max values
//                      - Finalized the ADC cal constants.  Created separate constants for the high-gain
//                        and low-gain circuits and moved them to FRAM and Flash.
//                          - Deleted ADC default cal constants (moved to Meter_def.h)
//                          - Replaced gain_ADC_low[], gain_ADC_high[], offset_ADC_low[], and
//                            offset_ADC_high[] with ADC cal structures ADCcalHigh and ADCcalLow.  Modified
//                            ADC_Cal_Scale() accordingly. 
//                          - Added initialization of ADCcalHigh and ADCcalLow to Meter_VarInit()
//                          - In Meter_VarInit(), replaced code used to initialize AFE cal constants with
//                            subroutine calls and added code to initialize the ADC cal constants
//                      - Modified the digital integrator, AFE_Integrate_Sample(), per input from George Gao
//   0.12   160502  DAH - Added code to compute metered AFE one-cycle and 200msec voltages
//                          - Added structures VolAFEOneCycLN and VolAFEOneCycLL
//                          - Added Calc_Voltage() and Calc_Voltage1()
//                          - Increased size of MTR_ss[] from 4 to 8
//                      - Added code to compute min and max one-cycle and 200msec AFE voltages
//                          - Added structures VolAFEOneCycLNmax, VolAFEOneCycLNmin, VolAFEOneCycLLmax,
//                            VolAFEOneCycLLmin, VolAFE200msecLNmax, VolAFE200msecLNmin, VolAFE200msecLLmax,
//                            VolAFE200msecLLmin
//                          - Modified Calc_Voltage() and Calc_Voltage1() to compute min and max LN and LL
//                            AFE voltages
//                      - The integrator for AFE currents introduces a one-sample lead to the output versus
//                        the voltage samples (the current leads the voltage by one sample).  A one sample
//                        delay was introduced as a result
//                          - AFE_PrevSample[] was added to store the previous samples of AFE currents
//                          - Modified Meter_VarInit() to initialize AFE_PrevSample[]
//                          - Modified AFE_Process_Samples() to add a one sample delay to the AFE integrator
//                            output samples
//                      - Added power calculations
//                          - Added Calc_Power() and Calc_Power1()
//                          - Added struct PwrOneCyc, Pwr200msec, PwrOneCycApp, and Pwr200msecApp
//                      - Added a 2-sample delay to the ADC readings
//                          - ADC_PrevSample[][] was added to store the previous samples of ADC currents
//                          - Modified Meter_VarInit() to initialize ADC_PrevSample[][]
//                          - Modified ADC_Process_Samples() to add a two sample delay to the ADC samples
//                      - Finalized AFE integrator seeding
//                          - Added Getting_AFE_Seed (moved from Intr.c)
//                          - Added Seed_State[]
//                          - Deleted AFE_seed_val[] (use ADC_samples[] instead)
//                          - Revised AFE_Integrate_Sample() to use either the high-gain or low-gain ADC
//                            sample to seed the integrator on a per-sample basis.  ADC samples are aligned
//                            with the AFE samples in both cases.  
//                          - Eliminated AFE_seed_val[] from ADC_Process_Samples().  ADC_samples[] provides
//                            the seed value, and added Seed_State[] to capture which sample to use.  Moved
//                            call to scaling routine (ADC_Cal_Scale()) to before the 2-sample delay code.
//          160502  BP  - Replaced Calc_Temperature() with Read_ThermMem().  Temperature in Rev 2 and later 
//                        will be read from a separate sensor over SPI instead of through the ADC.
//                      - Replaced TempSensorReading with ThermMemReading
//                      - Increased size of ADC_single_capture[] from 5 to 6 and ADC_samples[] from 10 to 12
//                      - Revised ADC_Process_Samples() for rev 2 hardware
//   0.13   160512  DAH - Modified AFE_Process_Samples() to handle the transitions between using AFE and ADC
//                        samples.  Added SampleCntOK[] and AFE_State[]
//                      - Added energy metering
//                          - Structures ResidualXxx and EnergyXxx added
//                          - Calc_Energy() and ResetEnergy() added
//   0.14   160620  DAH - In ADC_Process_Samples(), replaced max seed value (hard-coded for R-Frame) with
//                        ADC_MAX_SEEDVAL_NFRAME so unit is set up for an N-Frame
//                      - Modified Calc_Power() and Calc_Power1() to compute apparent power by multiplying
//                        the rms current and voltage instead of taking the root mean square of the real
//                        and reactive power
//                      - Added min and max calculations for the one-cycle and 200msec power values
//                          - PwrOneCycMin, PwrOneCycMax, Pwr200msecMin, Pwr200msecMax, PwrOneCycAppMin,
//                            PwrOneCycAppMax, Pwr200msecAppMin, Pwr200msecAppMax added
//                      - Modified Calc_Power() and Calc_Power1() to compute the min and max values
//                      - Fixed Pha and Phb apparent power calculations
//                          - Calc_Power() and Calc_Power1() revised
//                      - Added support for Test Injection
//                          - Replaced constant a1 (which was fixed for normal integrator operation) with
//                            variable INT_a1[], which is loaded with either NORMAL_a1 (for normal
//                            integrator operation) or TESTINJ_a1 (for test injection)
//                          - Modified AFE_Integrate_Sample() to use INT_a1[]
//   0.21   180301  DAH - Added harmonics computations.  This code is based on an algorithm designed by
//                        George Gao
//                          - Added includes of arm_math.h and arm_const_structs.h
//                          - Added various constants for harmonics definitions
//                          - Added test inputs x_n_test[] and x_n_test1[].  These should be deleted
//                            eventually.
//                          - Added tables w_n[] and H_w[]
//                          - Added CH_State, x_n[], and g_n[]
//                          - Added Harmonics structure to hold the final harmonics values
//                          - Revised Meter_VarInit() to initialize CH_State
//                          - Added Calc_Harmonics() subroutine
//                      - Added code to support ADC (line-side) voltages
//                          - Added numerous "VolADCxxxx" structures
//                          - Added Calc_Voltage2() and Calc_Voltage3() to compute one-cycle and 200msec ADC
//                            (line) voltages
//                          - Revised Meter_VarInit() to initialize "VolADCxxxx" sums of squares variables
//                      - Deleted Sin1_CoefQ15[] and Cos1_CoefQ15[] tables as they are no longer used
//   0.22   180420  DAH - Added support for demand computations and logging
//                          - Added Dmnd_xxx and Dmnd_I_xxx variables
//                          - Added Dmnd_I_VarInit() to initialize the appropriate demand variables
//                          - Added Calc_I_Demand(), CalcNextLoggingTime(), and
//                            CalcTimesToNextAnniversaries()
//                          - Moved "include "Iod_def.h" " statement before "#include "Meter_def.h" "
//                            because Meter_def.h uses INTERNAL_TIME, which is defined in Iod_def.h
//                          - Added includes of Events_def.h and Events_ext.h
//                          - struct EnergyAll was deleted.  The energy tally registers in this structure
//                            were moved into struct EngyDmnd.  EngyDmnd contains all of the variables used
//                            in demand logging, so that the structure can be written directly into FRAM and
//                            Flash.   Calc_Energy() and ResetEnergy() were revised to use the new tally
//                            registers.
//   0.23   180504  DAH - Deleted the harmonics tables (moved to Harm_Tables.h) and added include of
//                        Harm_Tables.h for readability
//                      - Deleted Demand functions and variables (moved to Demand.c) to improve readability
//                          - Deleted Dmnd_I_VarInit(), Calc_I_Demand(), CalcTimesToNextAnniversaries(), and
//                            CalcNextLoggingTime()
//                          - Deleted EngDmnd[], Dmnd_I_WinSum, Dmnd_I_SubSum, Dmnd_LoggingTime,
//                            Dmnd_I_SubTots, Dmnd_200msecCnt, Dmnd_Num200ms_Per_SubInt,Dmnd_I_SubIntCnt_DW,
//                            Dmnd_Num200ms_Per_DmndWin, Dmnd_I_SubIntCnt_LA, Dmnd_NumSubInt_Per_DmndWin, 
//                            Dmnd_NumSubInt_Per_LogAnn, Dmnd_SaveEvent
//                          - Added includes of Demand_def.h and Demand_ext.h
//   0.24   180517  DAH - Revised Calc_Energy() to write the energy directly into the FRAM, as opposed to
//                        setting flags and using the SPI2 manager subroutine, which has been eliminated
//                      - Combined Flash_def.h and FRAM_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Modified Calc_Energy() to use the request flag to write energy to FRAM instead
//                            of calling FRAM_WriteEnergy() directly
//                          - Added ManageSPI2Flags()
//                      - Added support for Alarm waveform capture events
//                          - Added ManageSPI2Flags()
//   0.26   180627  DAH - Revised ManageSPI2Flags() to support SPI2 read energy requests and Flash cal
//                        constants integrity check
//   0.27   181001  DAH - Revised Calc_Harmonics() to support change in user waveform capture and harmonics
//                        control
//                          - Replaced UserWF_Ack and UserWF_Flags with HarmReq and HarmAck.  These are
//                            harmonic request and acknowledge flags.  HarmReq is written only in the
//                            background and is read in the foreground.  HarmAck is written and read only in
//                            the foreground
//                      - Added initialization of HarmReq and HarmAck to Meter_VarInit()
//                      - Added support for Trip and Strip-Chart waveform captures
//                          - Added trip and strip-chart waveform handling flags to ManageSPI2Flags()
//   0.29   190122  DAH - Reduced size of ADC_single_capture[] and ADC_test_samples[] from 6 to 5 since coil
//                        temperature is no longer read using ADC1 and ADC2 (it is now read separately using
//                        ADC3).  Also reduced of ADC_samples[] from 12 to 10
//                      - Revised ADC_Process_Samples() to handle the elimination of the coil temperature
//                        from ADC_samples[]
//                      - Revised Read_ThermMem() to get the thermal memory reading from ADC_samples[9]
//                        instead of ADC_samples[10] (since coil temperature was removed from ADC_samples[])
//   0.30   190314  DAH - Fixed bug in AFE_Process_Samples() that ensures ground currents and voltage
//                        samples are processed using the max values if there is an AFE chip error
//                      - Added  TP_CoilTempRMSavg() for coil temperature measurement
//   0.31   190506  DAH - Removed Harmonics computation from User Waveform captures.  They are now
//                        completely independent
//                          - Added HarmSampleStartNdx (moved from Intr.c)
//                          - In Calc_Harmonics(), added initializing HarmSampleStartNdx at the start of the
//                            harmonics computation process
//                          - Deleted HarmAck as it is no longer used
//                      - Renamed N to N_SAMPLES
//                      - Revised the harmonics computations in Calc_Harmonics() to meet IEC61000-4-7 and
//                        IEC61000-4-30 and still support our existing way of providing harmonics
//                          - There are two harmonics: aggregated (meets the IEC specs) and instantaneous
//                            (the existing way)
//                          - Interharmonics to 5Hz are still computed but are only used internally as part
//                            the computation of the primary harmonics.  Only multiples of the fundamental
//                            (i.e., the primary harmonics) are outputted.
//                          - Aggregation to 3 seconds and filtering, both per IEC61000-4-7, were added to
//                            the harmonics computation
//   0.32   190726  DAH - Added Cur200msFltrIg to support Modbus communications.  This is either Igsrc or
//                        Igres, depending on the setting
//                      - In Calc_Power() and Calc_Power1(), revised how reactive power is calculated.  It
//                        was computed by shifting the voltage samples 90 degrees and then taking the RMS of
//                        the current and voltage products.  It is now just the vector difference of the
//                        apparent and real powers.  The new approach is used because it includes harmonics,
//                        while the old approach only accounted for the fundamental frequency.
//                        Note, the shifted VI sample products are still used to determine the sign of the
//                        reactive power and the power factor
//                      - Renamed Calc_Power() to Calc_Meter_Power()
//                      - Renamed Calc_Power1() to Calc_Prot_Power()
//                      - Added Apparent PF, THD, and Displacement PF support
//                          - Added Power Factor and Reactive Power sign convention comments (taken from the
//                            NRX1150 and SR2 code)
//                          - Added struct PF and array THD[]
//                          - Added constant arrays t200msecW[], t200msecVA[], and t200msecAV[] 
//                          - Added Calc_AppPF() and Calc_DispPF_THD()
//                          - Renamed struct Cur200msecSOS_Sav to Cur200msFltrSOS_Sav to distinguish it from
//                            the unfiltered current variables
//                          - Added struct Cur200msNoFltrSOS_Sav to compute unfiltered currents Ia thru In
//                          - Replaced struct CurOneCycSinSOS_Sav with CurVol200msNoFltrSinSOS_Sav[] and
//                            CurOneCycCosSOS_Sav with CurVol200msNoFltrCosSOS_Sav.  These are used to
//                            generate the sin and cosine components of the unfiltered currents Ia thru In
//                            (don't need Ig) and the unfiltered AFE voltages Van thru Vcn  over 200msec,
//                            instead of one cycle, for increased accuracy (harmonic analysis is also
//                            computed over 200msec)
//                          - Added struct VolAFE200msNoFltrSOS_Sav.Vxx to generate the unfiltered AFE
//                            voltages Van thru Vcn and Vab thru Vca over 200msec
//                      - Combined struct VolADC200msecLN and VolADC200msecLL into one struct, VolADC200ms
//                        Cur200msec_min to Cur200msFltr_min to distinguish them from unfiltered currents
//                      - Combined struct VolAFEOneCycLNSOS_Sav and VolAFEOneCycLLSOS_Sav into one
//                        structure, VolAFEOneCycSOS_Sav
//                      - Combined struct VolAFE200msecLNSOS_Sav and VolAFE200msecLLSOS_Sav into one
//                        structure, VolAFE200msFltrSOS_Sav
//                      - Combined struct VolADCOneCycLNSOS_Sav and VolADCOneCycLLSOS_Sav into one
//                        structure, VolADCOneCycSOS_Sav
//                      - Combined struct VolADC200msecLNSOS_Sav and VolADC200msecLLSOS_Sav into one
//                        structure, VolADC200msSOS_Sav
//                      - Combined struct VolAFEOneCycLN and VolAFEOneCycLL into one structure, VolAFEOneCyc
//                      - Combined struct VolAFE200msecLN and VolAFE200msecLL into VolAFE200msFltr
//                      - Combined struct VolADCOneCycLN and VolADCOneCycLL into one structure, VolADCOneCyc
//                      - Combined struct VolAFEOneCycLNmax and VolAFEOneCycLLmax into VolAFEOneCyc_max
//                      - Combined struct VolAFEOneCycLNmin and VolAFEOneCycLLmin into VolAFEOneCyc_min
//                      - Combined struct VolAFE200msecLNmax and VolAFE200msecLLmax into VolAFE200msFltr_max
//                      - Combined struct VolAFE200msecLNmin and VolAFE200msecLLmin into VolAFE200msFltr_min
//                      - Combined struct VolADCOneCycLNmax and VolADCOneCycLLmax into VolADCOneCyc_max
//                      - Combined struct VolADCOneCycLNmin and VolADCOneCycLLmin into VolADCOneCyc_min
//                      - Combined struct VolADC200msecLNmax and VolADC200msecLLmax into VolADC200ms_max
//                      - Combined struct VolADC200msecLNmin and VolADC200msecLLmin into VolADC200ms_min
//                      - Renamed Cur200msec to Cur200msFltr, Cur200msec_max to Cur200msFltr_max, and
//                      - Added S2NF_IDMND_WRITE and S2NF_PDMND_WRITE flags to ManageSPI2Flags()
//   0.33   190823  DAH - Added Time Stamps to min and max 200msec currents, min and max 200msec AFE
//                        voltages, and min and max 200msec ADC voltages
//                          - Changed Cur200msFltr_max and Cur200msFltr_min definitions from
//                            struct CUR_WITH_G_F to struct MIN_MAX_CUR_WITH_G_F
//                          - Changed VolAFE200msFltr_max and VolAFE200msFltr_min definitions from
//                            struct VOLTAGES to struct MIN_MAX_VOLTAGES
//                          - Revised Calc_Current() and Calc_Voltage() to add time stamps
//                          - Changed VolADC200ms_max and VolADC200ms_min definitions from struct VOLTAGES
//                            to struct MIN_MAX_VOLTAGES
//                          - Revised Calc_Voltage2() to add time stamps
//                      - Added Time Stamps to min and max 200msec powers
//                          - Changed Pwr200msecMin and Pwr200msecMax from struct POWERS to
//                            struct MIN_MAX_POWERS
//                          - Changed Pwr200msecAppMin and Pwr200msecAppMax from struct APP_POWERS to
//                            struct MIN_MAX_APP_POWERS
//                          - Revised Calc_Meter_Power() to add time stamps
//                      - Revised Calc_Meter_Power() to add total powers to min and max computations
//                      - Renamed Calc_Current() to Calc_Meter_Current()
//                      - Renamed Calc_Current1() to Calc_Prot_Current()
//                      - Renamed Calc_Voltage() to Calc_Meter_AFE_Voltage()
//                      - Renamed Calc_Voltage2() to Calc_Meter_ADC_Voltage()
//                      - Renamed Calc_Voltage1() to Calc_Prot_AFE_Voltage()
//                      - Deleted struct CurOneCycPeak (moved to Intr.c)
//                      - Added Crest Factor computations
//                          - Added struct CF_PeakSav to save peak one-cycle currents, struct CF_Sum to
//                            store the one-cycle crest factor sums, and struct CF to store the final crest
//                            factor values
//                          - Added Calc_CF()
//                          - Modified Meter_VarInit() to initialize CF_Sum.Ix and CF.Ix
//                      - Added K-Factor computations
//                          - Added flag to request K-Factor computations, K_FactorReq
//                          - Added struct KF and struct KF_Val
//                          - Revised Calc_Harmonics() to set K_FactorReq when done with harmonics
//                          - Modified Meter_VarInit() to initialize K_FactorReq, KF.State, and KF_Val.Ix
//                          - Added Calc_KFactor()
//                      - Added Sequence Components and Phase Angles computations
//                          - Added struct SeqComp to save the current and AFE l-n sequence components
//                          - Added PhAngles1[] to save the currents and AFE voltage phase angles
//                          - Added PhAngles2[] to save the ADC voltage phase angles
//                          - Added Calc_SeqComp_PhAng()
//                          - Modified Meter_VarInit() to initialize SeqComp.xxx
//                      - Added voltage and current unbalance computations
//                          - Added Cur200msFltrUnbal and VolAFE200msFltrUnbal
//                          - Added unbalance computations to Calc_Meter_Current() and
//                            Calc_Meter_AFE_Voltage()
//                      - Added frequency computations
//                          - Added Calc_Freq() and struct FreqLoad, FreqLine
//                          - Revised Meter_VarInit() to initialize FreqLoad, FreqLine variables
//                      - Revised comments and fixed bugs in Calc_DispPF_THD()
//   0.34   191002  DAH - Added PF.App[], PF.MinApp[), and PF.MaxApp[] to Meter_VarInit()
//                      - Added PF.Disp[], PF.MinDisp[), and PF.MaxDisp[] to Meter_VarInit()
//                      - Added THD[] to Meter_VarInit()
//                      - Added PhAngles1[] and PhAngles2[] to Meter_VarInit()
//                      - Added FreqLine.FreqVal, .MinFreqVal, and .MaxFreqVal to Meter_VarInit()
//                      - Added FreqLoad.FreqVal, .MinFreqVal, and .MaxFreqVal to Meter_VarInit()
//                      - Added VolAFE200msFltrUnbal and Cur200msFltrUnbal to Meter_VarInit()
//                      - Revised frequency measurement code for rev 4 boards.  Frequency measurement moved
//                        from Timer 9 to Timers 10 and 11.
//                          - Comments updated in Calc_Freq()
//   0.35   191118  DAH - In ADC_Process_Samples() added code to subtract the neutral voltage from the line
//                        voltages to get the Vln voltages.  The voltage inputs are to the ADC are not
//                        differential; neutral is a separate input
//                      - In AFE_Process_Samples() and AFE_Integrate_Sample(), added code to process Igsrc
//                        for either a Rogowski coil or a CT
//                      - In ADC_Cal_Scale(), AFE_Process_Samples(), and AFE_Integrate_Sample(), added code
//                        to process In for either a Rogowski coil or a CT
//                      - Increased size of INT_a1[] and ss[] for Rogowski ground current sensing
//   0.38   200708  DAH - Added support for writing setpoints to Flash for testing purposes
//                          - Revised ManageSPI2Flags() to:
//                              - clear S2F_SETP_WR1 request and set S2F_SETP_WR2 request when S2F_SETP_WR1
//                                acknowledge is set
//                              - clear S2F_SETP_WR2 request and set DPComm.Check_Status to
//                                CHK_STAT_COMPLETED when S2F_SETP_WR2 acknowledge is set
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added includes of RealTime_def.h and RealTime_ext.h
//   0.48   210730  DAH - Revised Calc_Meter_Current() and Calc_Meter_AFE_Voltage() to compute unbalance in
//                        per cent
//                      - Revised AFE_Process_Samples() to increase the RMS half-cycle current when
//                        checking whether to switch to the AFE samples from the ADC samples
//   0.50   220203  DAH - Added Cur200msIavg, VolAFE200msFltrVllavg, VolAFE200msFltrVlnavg,
//                        VolADC200msVllavg, VolADC200msVlnavg to make Modbus messaging simpler
//                          - Calc_Meter_Current(), Calc_Meter_AFE_Voltage(), Calc_Meter_ADC_Voltage revised
//                      - Added EngyDmnd[1].TotSumWHr and EngyDmnd[1].TotSumVarHr to make Modbus messaging
//                        simpler
//                          - Calc_Energy() revised
//                      - Added Cur200msFltrIgmin and Cur200msFltrIgmax to support Modbus communications.
//                        These are computed from Cur200msFltrIg (either Igsrc or Igres, depending on the
//                        setting)
//                          - Calc_Meter_Current() revised
//   0.51   220304  DAH - Moved struct SEQ_COMP definition to Meter_def.h
//                      - Renamed Calc_Meter_ADC_Voltage() to Calc_ADC_200ms_Voltage()
//                      - Renamed Calc_Voltage3() to Calc_ADC_OneCyc_Voltage()
//                      - Min/max current changed from 200msec values to one-cycle values
//                          - Replaced Cur200msFltrIgmin and Cur200msFltrIgmax with CurOneCycIgmin and
//                            CurOneCycIgmax
//                          - Replaced Cur200msFltrIgminTS and Cur200msFltrIgmaxTS with CurOneCycIgminTS and
//                            CurOneCycIgmaxTS
//                          - Replaced Cur200msFltr_min.Ix and Cur200msFltr_max.Ix with CurOneCyc_min.Ix and
//                            CurOneCyc_max.Ix.  ResetMinMaxValues() revised 
//                          - In ResetMinMaxValues(), min/max initialization values changed from extremes to
//                            NANs
//                          - Deleted min/max and unbalance computations from Calc_Meter_Current()
//                          - Added min/max and unbalance computations to Calc_Prot_Current()
//                          - Replaced CurOneCyc_min.Igsrc and Igres with CurOneCyc_min.Ig
//                          - Replaced CurOneCyc_max.Igsrc and Igres with CurOneCyc_max.Ig
//                          - Replaced CurOneCyc_min.IgsrcTS and IgresTS with CurOneCyc_min.IgTS
//                          - Replaced CurOneCyc_max.IgsrcTS and IgresTS with CurOneCyc_max.IgTS
//                          - Moved min/max current code from Calc_Meter_Current() to Calc_Prot_Current()
//                      - Added per-phase current unbalance with min/max
//                          - Calc_Prot_Current() revised
//                      - Current unbalance changed from 200msec value to one-cycle value
//                          - Replaced Cur200msFltrUnbal with CurOneCycUnbal
//                          - Moved current unbalance code from Calc_Meter_Current() to Calc_Prot_Current()
//                      - Min/max voltage changed from 200msec values to one-cycle values
//                          - Deleted struct MIN_MAX_VOLTAGES VolAFE200msFltr_min and VolAFE200msFltr_max
//                          - VolAFEOneCyc_min and VolAFEOneCyc_max changed from struct VOLTAGES to
//                            struct MIN_MAX_VOLTAGES
//                          - Deleted min/max voltage code from Calc_Meter_AFE_Voltage()
//                          - Revised Calc_Prot_AFE_Voltage() to add timestamps to min/max values
//                          - Deleted struct MIN_MAX_VOLTAGES VolADC200ms_max and VolADC200ms_min
//                          - VolADCOneCyc_min and VolADCOneCyc_max changed from struct VOLTAGES to
//                            struct MIN_MAX_VOLTAGES
//                          - Deleted min/max code from Calc_ADC_200ms_Voltage()
//                          - Added min/max code to Calc_ADC_OneCyc_Voltage()
//                      - Voltage unbalance changed from 200msec value to one-cycle value
//                          - Replaced VolAFE200msFltrUnbal with VolAFEOneCycUnbal
//                          - Moved voltage unbalance code from Calc_Meter_AFE_Voltage() to
//                            Calc_Prot_AFE_Voltage()
//                      - Added per-phase voltage unbalance with min/max
//                          - Calc_Prot_AFE_Voltage() revised
//   0.52   220309  DAH - Deleted redundant code in Calc_Meter_Power()
//                      - EngyDmnd[1].TotSumWHr was replaced by TotWHr for total real energy and NetWHr was
//                        added for net real energy.  Cannot use EngyDmnd[1].TotSumWHr because it would get
//                        written to Flash as part of the energy logging, and we are not logging total and
//                        net energies (they can be derived from the forward and reverse energies).
//                        Similarly, EngyDmnd[1].TotSumVarHr was replaced by TotVarHr for total reactive
//                        energy and NetVarHr was added for net reactive energy.
//                          - Calc_Energy() revised
//   0.53   220325  DAH - Added min/max THD values
//                          - struct THD_MIN_MAX THDminmax[10] added
//                          - Calc_DispPF_THD() revised
//                      - Eliminated instantaneous harmonics.  Captured harmonics are now the aggregated
//                        harmonics
//                      - Added 40th harmonic
//                          - Calc_Harmonics() revised
//                          - struct HarmonicsInst deleted and replaced with HarmInst[]
//                          - HarmSeqNum deleted
//                      - Moved some harmonics variables to the proper global and local sections
//   0.54   220330  DAH - In Calc_Prot_Current(), added check for divide by zero in per-phase current
//                        unbalance computations
//                      - In Calc_Prot_AFE_Voltage(), added check for divide by zero in per-phase voltage
//                        unbalance computations and corrected bug in Vll per-phase unbalance computations
//                      - Added max total current unbalance, CurUnbalTotMax and CurUnbalTotMaxTS to
//                        Calc_Prot_Current()
//                      - Added max total voltage unbalance, VolUnbalTotMax and VolUnbalTotMaxTS to
//                        Calc_Prot_AFE_Voltage()
//   0.55   220420  DAH - Added CurOneCycIavg to support processor-processor communications
//                      - Added StatusCode for Modbus and proc-proc comms testing
//   0.59   220831  DAH - Revised Calc_Energy() to replace code that set S2NF_ENERGY_WR with a call to
//                        FRAM_WriteEnergy()
//                      - Revised ManageSPI2Flags() to eliminate the Non-Flash Access Flags (S2NF_xx) as
//                        they are no longer used
//   0.61   221001  DB  - Added SnapshotMeteredValues and SnapshotMeter()
//   0.62   221027  DAH - Deleted S2F_SETP_WR1 and S2F_SETP_WR2 from ManageSPI2Flags() as they are no longer
//                        used
//   0.63   221111  DB  - Fixed minor bug in SnapshotMeter()
//   0.67   221209  DAH - S2F_xx flags renamed to S1F_xx flags
//                      - SPI2Flash.xx renamed to SPI1Flash.xx
//                      - Renamed ManageSPI2Flags() to ManageSPI1Flags()
//   0.68   230124  DB  - ExtCapSnapshotMeter added. This function is called when Extended capture starts
//   0.69   230220  DAH - Strip chart waveform captures has been deleted and replaced with the extended
//                        captures
//                          - Strip-chart flags removed from ManageSPI1Flags()
//   0.70   230224  BP  - Added TA_Volt_Monitoring() and Calc_BatteryVolt()
//   0.72   230320  DAH - Revised  SPI1Flash.Ack and SPI1Flash.Req operation
//                          - ManageSPI1Flags() revised
//   24     230323  DAH - Added preliminary support for PriSecCause status
//                          - Update_PSC() added
//   25     230403  DAH - Revised Update_PSC() to move primary status to MS byte
//                      - Revised ManageSPI1Flags() to support proc-proc comms read waveforms command
//   27     230404  DAH - Added temporary patch for AE demo that places LDPU flag in Pri/Sec/Cause status
//                          - Update_PSC() revised
//   30     230412  DAH - Replaced ExtCapSnapshotMeter() with ExtCapSnapshotMeterOneCyc() and
//                        ExtCapSnapshotMeterTwoHundred(), as one-cycle values are captured for the 6sec
//                        readings and 200msec values are captured for the 60sec readings
//   40     230531  DAH - Added SummaryCode to SnapshotMeter() as an input parameter
//                      - Added values to SnapshotMeter()
//   54     230802  BP  - Revised battery code
//   58     230810  DAH - Revised Calc_BatteryVolt() to eliminate global var avg_vbat
//   93     231010  BP  - Added Battery Sense enable/disable around the Battery ADC sampling
//   94     231010  DAH - Added support for user waveform captures
//                          - CaptureUserWaveform() created from code moved from DispComm.c and revisions
//                            made to handle multiple requests
//                          - Added UserWF_xx variables (moved from Intr.c)
//   95     231011 DAH  - Added TP_AFEIntOff to Meter_VarInit() (moved from Test_VarInit())
//   108    231108 DAH  - Added include of Flags_def.h for BITxx definitions
//   117    231129 DAH  - Deleted Read_ThermMem() as it is not used
//   122    231204 BP   - Added Coil Detection measurement code
//   129    231213 MAG  - Changed how Update_Std_Status() determines Maintenance Mode to set status bit
//   141    240115 BP   - Added new_std.bit.T_Forbid
//   145    240124 BP   - Added T_Forbids for Coil Detection and Sec Inj
//   148    240131 BP   - Added Aux Power measurement code with provisions to hold off protection when Aux or USB
//                        power is applied to prevent nuisance tripping.
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
#include "Prot_def.h"
#include "Meter_def.h"
#include "Intr_def.h"
#include "Events_def.h"
#include "Demand_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "DispComm_def.h"
#include "Flags_def.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "Prot_ext.h"   // *** DAH  NEEDED FOR MODBUS TEST INITIALIZATION - CAN PROBABLY DELETE LATER




//
//      Local Definitions used in this module...
//

// Harmonics constants
#define FREQ_FUNDAMENTAL    60	                           // Fundamental frequency, e.g., 50, 60, 400 Hz
#define SAMPLING_RATE       80                             // 80 Samples per cycle
#define NUM_CYCLES          12                             // Number of cycles in a measurement period
#define N_SAMPLES           (NUM_CYCLES * SAMPLING_RATE)   // Number of samples in time domain
#define NFFT                2048	                       // Next power of 2 for FFT
#define INDEX_FUNDAMENTAL   NUM_CYCLES                     // Index in harmonic result array of fundamental
#define NUM_HARMONICS       (N_SAMPLES/2 - 1)              // Number of harmonics (excluding DC component)

#define HARM_LPF_BETA       7.012F
#define HARM_LPF_ALPHA      8.012F

// Tables for harmonics calculations
#include "Harm_Tables.h"

// Energy and power constants
#define MS200_TO_HRS            ((float)(0.2/(60 * 60)))



//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Intr_ext.h"
#include "Test_ext.h"
#include "Iod_ext.h"
#include "RealTime_ext.h"
#include "Events_ext.h"
#include "Demand_ext.h"
#include "DispComm_ext.h"
#include "Setpnt_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Meter_VarInit(void);
void AFE_Process_Samples(void);
void AFE_Cal_Scale(uint8_t index);
void Calc_Meter_Current(void);
void Calc_Prot_Current(void);
void Calc_Meter_AFE_Voltage(void);
void Calc_Prot_AFE_Voltage(void);
void Calc_ADC_200ms_Voltage(void);
void Calc_ADC_OneCyc_Voltage(void);
void Calc_Meter_Power(void);
void Calc_Prot_Power(void);
void Calc_Energy(void);
void Calc_AppPF(void);
void Calc_DispPF_THD(void);
void ADC_Process_Samples(void);
void ADC_Cal_Scale(uint8_t index);
void TA_Volt_Monitoring(void);
void AuxPower_Monitoring(void);
void Calc_BatteryVolt(void);
void ResetEnergy(void);
void Calc_Harmonics(void);
void ManageSPI1Flags(void);
float TP_CoilTempRMSavg(void);
void Calc_CF(void);
void Calc_KFactor(void);
void Calc_SeqComp_PhAng(void);
void Calc_Freq(struct FREQ_MEASURE_VARS *f_ptr, float v_xn);
uint8_t Update_PSC(uint32_t *pPriSecCause);
void ExtCapSnapshotMeterOneCyc();
void ExtCapSnapshotMeterTwoHundred();
void SnapshotMeter(uint16_t SummaryCode);
uint8_t CaptureUserWaveform(uint16_t type);


                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING START
    void Load_MB_Test_Vals(void);
                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING END



//      Local Function Prototypes (These functions are called only within this module)
//
void AFE_Integrate_Sample(uint8_t index);
void Meter_Filter_Sample(uint8_t index);

void ResetMinMax(void);
void ResetMinMaxBufID(uint32_t bufID);



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct CUR_WITH_G_F             CurOneCycSOS_Sav;
struct CUR_WITH_G_F             Cur200msFltrSOS_Sav;
struct CUR_WITHOUT_G_F          Cur200msNoFltrSOS_Sav;
float                           CurVol200msNoFltrSinSOS_Sav[7];
float                           CurVol200msNoFltrCosSOS_Sav[7];
struct CUR_WITH_G_F             CurOneCyc;
struct MIN_MAX_CUR_WITH_G_F     CurOneCyc_max;
struct MIN_MAX_CUR_WITH_G_F     CurOneCyc_min;
struct CUR_WITH_G_F             Cur200msFltr;
struct CUR_WITHOUT_G_F          CF_PeakSav;
struct CUR_WITHOUT_G_F          CF;
struct CUR_WITHOUT_G_F          CurUnbal;               // Per-phase current unbalance
struct MIN_MAX_CUR_WITHOUT_G_F  CurUnbalPh_max;
float                           CurUnbalMax;
float                           CurUnbalAllMax;
struct INTERNAL_TIME            CurUnbalAllMaxTS;
float                           CurUnbalTot;            // Total current unbalance
float                           CurUnbalTotMax;
struct INTERNAL_TIME            CurUnbalTotMaxTS;

struct VOLTAGES                 VolADCOneCycSOS_Sav;
struct VOLTAGES                 VolADC200msSOS_Sav;
struct VOLTAGES                 VolAFEOneCycSOS_Sav;
struct VOLTAGES                 VolAFE200msFltrSOS_Sav;
struct VOLTAGES                 VolAFE200msNoFltrSOS_Sav;
struct VOLTAGES                 VolADCOneCyc;
struct MIN_MAX_VOLTAGES         VolADCOneCyc_max;
struct MIN_MAX_VOLTAGES         VolADCOneCyc_min;
struct VOLTAGES                 VolADC200ms;
struct VOLTAGES                 VolAFEOneCyc;
struct MIN_MAX_VOLTAGES         VolAFEOneCyc_max;
struct MIN_MAX_VOLTAGES         VolAFEOneCyc_min;
struct VOLTAGES                 VolAFE200msFltr;
struct VOLTAGES                 VolUnbal;
float                           VolUnbalMaxLN, VolUnbalMaxLL;
struct MIN_MAX_VOLTAGES         VolUnbalPh_max;
float                           VolUnbalAllMaxLN, VolUnbalAllMaxLL;
struct INTERNAL_TIME            VolUnbalAllMaxLNTS, VolUnbalAllMaxLLTS;
float                           VolUnbalTot;            // Total voltage unbalance
float                           VolUnbalTotMax;
struct INTERNAL_TIME            VolUnbalTotMaxTS;

struct POWERS                   PwrOneCycSOS_Sav;
struct POWERS                   Pwr200msecSOS_Sav;
struct POWERS                   PwrOneCyc;
struct POWERS                   PwrOneCycMin;
struct POWERS                   PwrOneCycMax;
struct POWERS                   Pwr200msec;
struct MIN_MAX_POWERS           Pwr200msecMin;
struct MIN_MAX_POWERS           Pwr200msecMax;
struct APP_POWERS               PwrOneCycApp;
struct APP_POWERS               PwrOneCycAppMin;
struct APP_POWERS               PwrOneCycAppMax;
struct APP_POWERS               Pwr200msecApp;
struct MIN_MAX_APP_POWERS       Pwr200msecAppMin;
struct MIN_MAX_APP_POWERS       Pwr200msecAppMax;

struct ENERGY_FLOATS            ResidualPha;
struct ENERGY_UINT64            EnergyPha;
struct ENERGY_FLOATS            ResidualPhb;
struct ENERGY_UINT64            EnergyPhb;
struct ENERGY_FLOATS            ResidualPhc;
struct ENERGY_UINT64            EnergyPhc;
struct ENERGY_FLOATS            ResidualAll;
signed long long                TotWHr;
signed long long                NetWHr;
signed long long                TotVarHr;
signed long long                NetVarHr;

uint16_t AFE_single_capture[16] @".sram2";
uint32_t ADC_single_capture[5] @".sram2";
float AFE_new_samples[8] @".sram2";
float MTR_new_samples[8] @".sram2";
struct AFE_CAL AFEcal @".sram2";

struct HARMONICS_I_STRUCT HarmonicsAgg, HarmonicsCap;

uint8_t HarmReq, HarmFrozen;

struct K_FACTORS KF_Val;

struct SEQ_COMP SeqComp;

float PhAngles1[9];
float PhAngles2[6];

struct FREQ_MEASURE_VARS FreqLoad, FreqLine;

struct ADC_CAL ADCcalHigh;
struct ADC_CAL ADCcalLow;

float ThermMemReading;            // thermal memory value through ADC2
float INT_a1[5];

float Cur200msFltrIg;             // Ig for communications, either Igsrc or Igres
float CurOneCycIg;                // Ig for communications, either Igsrc or Igres
float CurOneCycIavg;              // One-cycle average current for communications
float Cur200msIavg;               // Iavg
float VolAFE200msFltrVllavg;      // AFE Vllavg
float VolAFE200msFltrVlnavg;      // AFE Vlnavg
float VolADC200msVllavg;          // ADC Vllavg
float VolADC200msVlnavg;          // ADC Vlnavg

struct PF_VALUES PF;
float THD[10];
struct THD_MIN_MAX THDminmax[10];

uint32_t StatusCode;
union WORD_BITS TU_BinStatus;

struct USER_WF_CAPTURE UserWF;
struct USER_SAMPLES UserSamples;                    // User waveform capture sample buffer


uint16_t TestWFind;             // *** DAH Added for test waveform capture
float seed_val;                 // *** DAH Added for seeding debug

struct SNAPSHOT_METER SnapshotMeteredValues; 

uint16_t temp_i;
float Vll_max;                  // BP added for Overvoltage protection
float Vll_min;                  // BP added for Undervoltage protection
float CurOneCycMax;             // BP added for Current Unbalance
float Vbattery;
float TA_Volt;
float AuxPower_Volt;


//------------------------------------------------------------------------------------------------------------
//                   Global Constants used in this module and other modules
//------------------------------------------------------------------------------------------------------------
//




//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
float ss[5] @".sram2";
float MTR_ss[8] @".sram2";
uint32_t ADC_test_samples[5] @".sram2";     // for DEBUG
float ADC_samples[10] @".sram2";
float AFE_PrevSample[5] @".sram2";
float ADC_PrevSample[8][2] @".sram2";
struct CUR_WITHOUT_G_F CF_Sum;
uint8_t Seed_State[4] @".sram2";
uint8_t AFE_State[4] @".sram2";
uint8_t SampleCntOK[4] @".sram2";
uint8_t UseADCvals @".sram2";
uint8_t Getting_AFE_Seed @".sram2";

float Max_Metering_Energy;
float Min_Metering_Energy;

struct HARMONICS_F_STRUCT               // Harmonics Intermediate Values (Filtered and Aggregation Sum)
{                                           // Harmonic analysis percent * 100 (100% = 10000)
   float Ia[40];                            // Phase A harmonics [0]=fundamental, [1]=2nd harmonic, etc.
   float Ib[40];                            // Phase B harmonics
   float Ic[40];                            // Phase C harmonics
   float In[40];                            // Neutral harmonics
   float Vab[40];                           // Vab harmonics
   float Vbc[40];                           // Vbc harmonics
   float Vca[40];                           // Vca harmonics
   float Van[40];                           // Van harmonics
   float Vbn[40];                           // Vbn harmonics
   float Vcn[40];                           // Vcn harmonics
} HarmonicsFil, HarmonicsSum;

struct HARMONICS_I_STRUCT HarmonicsTemp;

uint16_t HarmInst[40];
uint16_t HarmSampleStartNdx;
uint8_t HarmSumCount;

struct K_FACTOR_VARS
{
  float     DivisorSum;
  float     DividendSum;
  uint16_t  *HarmPtr;
  uint8_t   State;
} KF;

//uint8_t AFE_Error;       // added to Flags0 as part of the Electrical alarm
uint8_t K_FactorReq;
  
uint8_t CH_State;
float32_t g_n[2 * NFFT];
float32_t x_n[N_SAMPLES];

uint16_t fred_frame[10];  // *** DAH USED FOR DEBUG AND TEST ONLY

uint8_t USBStatus;        // *** BP - put here for now until USB code is written
uint32_t sum_Vbat;
uint32_t sum_TAVolt;
float avg_TAVolt;
uint32_t sum_AuxVolt;
float avg_AuxVolt;
uint8_t passed_1st_detection;

//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//

// Digital integrator coefficients
const float b0 = 1;
//const float a1 = 9.9691733373313E-1;
//const float a1 = 9.90742468983431E-1;                               // For test injection
const float a1_scaled = (9.9691733373313E-1/SCALE_FACTOR_b0);
//const float a1_scaled = (9.90742468983431E-1/SCALE_FACTOR_b0);      // For test injection

// DC filter coefficients
const float MTR_b0 = 9.9928408556729E-1;
const float MTR_a1 = 9.9609375000000E-1;

// Constants used in Calc_AppPF().  These are placed here so that they are computed once and are stored in
//   code space
float * const t200msecW[4] = {&Pwr200msec.Pa, &Pwr200msec.Pb, &Pwr200msec.Pc, &Pwr200msec.Ptot};
float * const t200msecVA[4] = {&Pwr200msecApp.AppPa, &Pwr200msecApp.AppPb, &Pwr200msecApp.AppPc,
                               &Pwr200msecApp.Apptot};

// Constants used in Calc_DispPF_THD(void).  These are placed here so that they are computed once and are
//   stored in code space
float * const t200msecAV[10] =
       {&Cur200msNoFltrSOS_Sav.Ia, &Cur200msNoFltrSOS_Sav.Ib, &Cur200msNoFltrSOS_Sav.Ic, &Cur200msNoFltrSOS_Sav.In,
        &VolAFE200msNoFltrSOS_Sav.Van, &VolAFE200msNoFltrSOS_Sav.Vbn, &VolAFE200msNoFltrSOS_Sav.Vcn,
        &VolAFE200msNoFltrSOS_Sav.Vab, &VolAFE200msNoFltrSOS_Sav.Vbc, &VolAFE200msNoFltrSOS_Sav.Vca};


// For test DC input R-Frame
/*const float gain_ADC_low[10] = {14, 14, 14, 14, 14, 14, 14, 14, 14, 14};
const float gain_ADC_high[10] = {274E-3, 274E-3, 274E-3, 274E-3, 274E-3, 274E-3, 274E-3, 274E-3, 274E-3, 274E-3};
const float offset_ADC[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};          // for now - BP*/

//


//------------------------------------------------------------------------------------------------------------
// PXR35 Phase Current (Ia, Ib, Ic, In) Measurement Signal Chain for the AFE
//
//    The 4 phase currents are measured by Rogowski current transformers (di/dt sensors).  The signal chain
//    is shown below:
//
//                              ---------- Input circuit and AFE ----------
//                +----------+              +--------+                +--------+
//                | Rogowski |              | Filter |                |  AFE   |
//    Current --->|   di/dt  |------------->|        |--------------->| 24-bit |---->SPI3 to Microprocessor
//                |  Sensor  | N: .345mV/A  |   /2   | N: 172.5uV/A   | signed |
//                +----------+ R: .086mV/A  +--------+ R: 43.125uV/A  +--------+
//
//                                 ---------- Microprocessor ----------
//                +---------+     +------------+     +-------------+     +-------------+
//    24-bit      | Convert |     |            |     |   Offset    |     | Convert to  |
//    Signed  --->|   to    |---->| Integrator |---->|  and gain   |---->| engineering |----> to selection
//    Integer     |  Float  |     |            |     | calibration |     | units       |
//                +---------+     +------------+     +-------------+     +-------------+
//
//                         ---------- Input circuit and Internal ADC ----------
//                +----------+        +--------+        +--------+ 10.3uV/A   
//                | Rogowski |        | Filter |        | Low    |  2.575uV/A                   +----------+
//    Current --->|   di/dt  |------->|        |------->| Gain   |-----------+----------------->|          |
//                |  Sensor  |        | /34.33 |        |  x2    |           |                  | Internal |
//                +----------+        +--------+        +--------+           |   +------+       |   ADC    |-->to micro
//                            N: .1725mV/A      N: 5.15uV/A                  |   | High |       |  12-bit  |
//                            R: .043mV/A       R: 1.29uV/A                  +-->| Gain |------>| unsigned |
//                                                                               |  x51 |       |          |
//                                                                               +------+       +----------+
//                                                                                       N: 525.3uV/A
//                                                                                       R: 131.326uV/A
//                                 ---------- Microprocessor ----------
//                 +-----------+     +---------+     +-------------+     +-------------+
//    12-bit       | Convert   |     | Subtract|     |   Offset    |     | Convert to  |
//    Unsigned --->|    to     |---->| 1.25V   |---->|  and gain   |---->| engineering |----> to selection
//    Integer      |  Float    |     | Offset  |     | calibration |     | units       |
//                 +-----------+     +---------+     +-------------+     +-------------+
//
//
//                                 ---------- Microprocessor ----------
//                            +--------+     
//    AFE Samples in -------->|        |               +------------------+ 
//    Eng Units               | Select |------+------->| Waveform Capture | 
//                       +--->|        |      |        +------------------+ 
//    ADC Samples in     |    +--------+      |                             
//    Eng Units      ----+                    |        +-------------+      
//                                            +------->| 61850 Comms |      
//                                            |        +-------------+      
//                                            |                             
//                                            |        +------------+       
//                                            +------->| Protection |       
//                                            |        +------------+       
//                                            |                             
//                                            |        +----------+         
//                                            +------->| Metering |         
//                                                     +----------+         
//
//
//
//------------------------------------------------------------------------------------------------------------
//          The Sign of Lagging VARS...
//------------------------------------------------------------------------------------------------------------
//
//          Engineers universally agree that an inductive load has lagging VARs.  Choosing a simple system
//          with a purely sinusoidal voltage source, a parallel resistor "R" and inductive "L" load, the
//          following statements can be made.  Power is voltage multiplied by current and current is voltage 
//          divided by the impedance.  Thses relationships are shown in the following equations:
//
//                   power = vi
//
//          If "v" and "i" are complex numbers then: complex power = P + jQ.  Where P is "real power" (Watts), 
//          Q is "reactive power" (VARs) and "j" is the square root of -1.
//
//          The load current is the sum of the resistor "R" current and the induction "L" current, or
//          i = v/R + (v/jwL), using "sinor" relationships.   Where "w" is "Ohmega".
//
//                   power = vi
//                         = v[(v/R) + (v/jwL)]
//                         = (v^2) +  [(v^2)/jwL][j/j]
//                         = (v^2) + j[-(v^2)/wL]
//                         = P  + jQ
//
//          Thus, for the lagging load of the example:
//                   P (Watts) = v^2
//                   Q (VARs)  = -(v^2)/wL
//          Note, the "Q" (VARs) is negative.  This result is not particularly attractive to power system
//          engineers and the explanation of why is best stated by William D. Stevenson, Jr, in his book
//          "Elements of Power system Analysis" (2nd Ed, 1962) on pg 136 & 137, quoted as follows:
//
//             "Engineers are not entirely in agreement on the sign of reactive power, but most power
//              system engineers use a positive sign to indicate lagging VARs, the VARs of an inductive
//              load.  With this convention, a capacitor receives negative VARs from the line.  Power
//              system engineers find it convenient to consider a capacitor as supplying positive VARs 
//              rather the receiving negative VARs.  This concept of the action of a capacitor is 
//              consistent with the adoption of the positive sign for the VARs received by an inductive
//              load."
//
//          This sign convention by power system engineers improves communication by improving the English
//          comprehension.  In an industry that literally powers cities and states, this is an important
//          aspect.  Having made this chioce for an inductive load being positive VARs, how is complex power
//          redefined?  Complex power, as used by power system engineers, is defined as the product of
//          "voltage" multiplied by the "conjugate of current":
//
//                   Complex Power = VI* = P +jQ, where I* is the "conjugate of current"
//          Note that the conjugate of a phasor is that same phasor with the sign of the angle reversed.
//
//          Neither VAR sign convention is wrong.  One comes from a straight forward mathematical
//          calculation.  The other comes from a desire to eliminate communication confusion and the
//          mathematical calculation is redefined to accomodate this result.
//
//
//------------------------------------------------------------------------------------------------------------
//                   Standardized Sign Conventions
//------------------------------------------------------------------------------------------------------------
//
//       Within industry, there are several sign conventions in use for reactive power (VAR), reactive
//       energy (VARh), and power factor measurements.  The NRX1150 supports three sign conventions for Power 
//       Factor, VARs, and Watts as determined by a group 0 setpoint: Setpoints0.stp.SignConv as follows...
//          - Setpoints0.stp.SignConv = 0: IEC
//          - Setpoints0.stp.SignConv = 1: IEEE
//          - Setpoints0.stp.SignConv = 2: IEEEalt
//
//       Additionally, a setpoint is available to indicate the breaker is reverse-fed (backfeed).  A backfeed
//       breaker effectively has its current sensors inverted (equivalent to reversing the current sensor 
//       connections).  This has the effect of interchanging quandrant 1 & 3, and 2 & 4 when the setpoint 
//       is a 1.  
//          - Setpoints0.stp.RevFeed = 0: normal
//          - Setpoints0.stp.RevFeed = 1: reverse feed
//
//       Both these setpoints effect only the "DISPLAY" of power and energy by the panel, CAM, and test
//       port.  Accumulation of energy is not effected by either setpoint.  Power factor computation uses
//       both setpoints to determine sign.
//       
//       
//       Note: The following diagrams show sign convention in several forms:
//
//          - Conventional phasor diagrams where the vertical axis is drawn with leading forward currents 
//            in quadrant 1 and lagging forward currents in quadrant 4.
//
//          - Power system network engineers prefer to use power flow diagrams which are basically phasor  
//            diagrams rotated 90 degrees ccw. In a power flow diagram, the real axis is vertical, with power 
//            flow into the load at the top.  The horizontal axis is reactive power flow, with lagging VARs  
//            consumed by the load on the right.  The voltage reference remains aligned with the positive real 
//            axis with the phase angle of current measured counter-clockwise for leading current.
//
//          - IEEE 1459-2010 Power Flow Diagrams are different from both of the above in that the phasor
//            is rotating clock-wise.  The horizontal axis is real power flowing into the load (+).  The
//            vertical axis represents reactive power with lagging current in quadrant 1 (upper right).
//
//       A helpful reference can be found in the IEEE 3399-1997 (Brown Book), Section 4.3...
//
//          The four expressions for power quantities given in the following table can be used to model
//          nonlinear elements.  Given any two of the four values, the remaining two can be defined.  Power
//          can also be expressed in polar form:  S = |S|angle(theta) which yields these relationships:
//          PF = cos(theta), P = S cos(theta), and Q = S sin(theta).  Theta is defined as the angular
//          displacement of leading current with respect to the voltage.  Note that the magnitude of the 
//          complex power must be used in the previous equations and in the relations of the table below:
//
//          Note also that the signs of P or Q may be positive or negative.  By convention, the positive sign
//          of Q is used for inductive loads: that is, the current will lag (180 < theta < 360) the voltage 
//          applied to a load that consumes VARs.  In this sense, it is said that capacitors generate VARs
//          (current leads the voltage) and that induction motor absorb VARs.
//
//          By convention also, the sign of P is positive for a load that consumes energy or a source that
//          generates energy.  Thus a load with a negative sign for P could be used to represent a generator,
//          and vice versa for a motor.  It should also be noted that the expressions below are appropriate
//          for fundamental frequency applications only; modifications are required for application in
//          harmonic systems.
//
//                               Table:  Four defining expressions for power quantities
//          __________________________________________________________________________________________________
//                     Name              Symbol         Unit                     Defining expression
//
//                complex power            S             VA  (voltampere)        S = P + jQ
//
//                active power             P             W   (Watt)              P = (S^2 - Q^2)^0.5
//
//                reactive power           Q             VAR                     Q = (S^2 - P^2)^0.5
//
//                power factor             PF            pu (per unit)           PF = P/S
//          __________________________________________________________________________________________________
//
//
//       Additional useful references:
//       -  "Power Flow Direction Definitions For Metering of Bidirectional Power",
//          Raymond H. Stevens, IEEE Transactions on Power Apparatus & Systems, Vol PAS-102, #9, Sep 1983.
//
//       -  "Understanding Power Flow and Naming Conventions In Bi-directional Metering Applications",
//          Michael Bearden, Landis+Gyr North America
//
//       -  "Power Flow Direction Definitions for Metering of Bi-directional Power"
//          Raymond H. Stevens, Sangamo Weston Inc., IEEE Power Engineering Review, September 1983, pp 34-35
//
//       -  "Electrical Transmission and Distribution Reference Book"
//          Westinghouse Electric Corp, 1964, Section 10.2, pp 291...
//
//       -  "IEEE Standard Definitions for the Measurement of Electric Power Quantities Under Sinusoidal and 
//           Nonsinusoidal Balanced or Unbalanced Conditions", IEEE Std 1459-2010, March 2010, Section 3.
//
//------------------------------------------------------------------------------------------------------------
//          IEEE Std 1459-2010
//------------------------------------------------------------------------------------------------------------
//
//       "IEEE standard Definitions for the Measurement of Electric Power Quantities Under Sinusoidal,
//          Nonsinusoidal, Balanced, or Unbalanced Conditions"
//
//          Power Flow Diagram:
//
//                                                     Reactive
//                                                      Power
//                                                     Delivered
//                                                      270 deg
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                                        |       (Lagging Current)
//                                                        |       (Inductive Loads)
//                       +Q (In, Delivered, Consumed)     |     +Q (In, Delivered, Consumed) 
//                       -P (Out, Received, Sourced)      |     +P (In, Delivered, Consumed)
//                                                        |     
//                                                        |
//       Active          <-Power Flowing into the Source  |  Power Flowing into the Load->        Active
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//       Received                                         |                                       Delivered
//                                                        |
//                       -P (Out, Received, Sourced)      |     +P (In, Delivered, Consumed)
//                       -Q (Out, Received, Sourced)      |     -Q (Out, Received, Sourced)
//                                                        |     
//                                                        |      (Capacitive Loads)
//                                                        |      (Leading Current)        Omega = cw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                       90 deg
//                                                     Reactive
//                                                      Power
//                                                     Received
//
//       Note: 1) This is not a conventional phasor diagram in that Omega is rotating clockwise.
//
//             2) Phase angle measures current relative to voltage:
//                   (i.e. 45 deg => current leads voltage by 45 deg)
//  
//             3) Terminology is from the utility's point of view:
//                   Real (Active) power flow (P = Watts) to passive loads:  "Delivered" = "In" = "Consumed"
//                   Reactive power flow (Q = VARs) to inductive loads:  "Delivered" = "In" = "Consumed"
//
//                   Real (Active) power flow (P = Watts) to generator loads: "Received" = "Out" = "Sourced"
//                   Reactive power flow (Q = VARs) to capactive loads: "Received" = "Out" = "Sourced"
//
//             3) VARs sign convention: S = P + jQ = VI*
//
//
//                              ***** IEEE Std 1459-2010 Power Flow Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//          IEEE Std 1459-2010 (Alternative)
//------------------------------------------------------------------------------------------------------------
//
//       This convention is similar to the IEEE Std 1459-2010 except the sign of VARs is reversed aligining
//       it with the electrical engineering / math convention
//
//          Power Flow Diagram:
//
//                                                     Reactive
//                                                      Power
//                                                     Delivered
//                                                      270 deg
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                                        |       (Lagging Current)
//                                                        |       (Inductive Loads)
//                       -Q (In, Delivered, Consumed)     |     -Q (In, Delivered, Consumed) 
//                       -P (Out, Received, Sourced)      |     +P (In, Delivered, Consumed)
//                                                        |     
//                                                        |
//       Active          <-Power Flowing into the Source  |  Power Flowing into the Load->        Active
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//       Received                                         |                                       Delivered
//                                                        |
//                       -P (Out, Received, Sourced)      |     +P (In, Delivered, Consumed)
//                       +Q (Out, Received, Sourced)      |     +Q (Out, Received, Sourced)
//                                                        |     
//                                                        |      (Capacitive Loads)
//                                                        |      (Leading Current)        Omega = cw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                       90 deg
//                                                     Reactive
//                                                      Power
//                                                     Received
//
//       Note: 1) This is not a conventional phasor diagram in that Omega is rotating clockwise.
//
//             2) Phase angle measures current relative to voltage:
//                   (i.e. 45 deg => current leads voltage by 45 deg)
//  
//             3) VARs sign convention: S = P + jQ = VI
//
//
//                              ***** Alternative IEEE Std 1459-2010 Power Flow Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//          Electrical Engineering / Math Convention
//------------------------------------------------------------------------------------------------------------
//
//                                                  (Imaginary Axis)
//                                                      90 deg
//                                                        +j
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                                        |       (Leading Current)
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     +Q (VARs)          |     +Q (VARs)
//                                                        |     
//                                                        |
//                       <-Power Flowing into the Source  |  Power Flowing into the Load->
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//                                                        |
//                                                        |
//                                     -Q (VARs)          |     -Q (VARs)
//                                     -P (Watts)         |     +P (Watts)
//                                                        |     
//                                                        |
//                                                        |      (Lagging Current)        Omega = ccw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                        -j
//                                                      270 deg
//
//       Note: 1) This is a conventional phasor diagram in that Omega is rotating CCW.
//
//             2) Phase angle measures current relative to voltage:
//                   (i.e. 45 deg => current leads voltage by 45 deg)
//  
//             3) VARs sign convention: S = P + jQ = VI
//
//                                      ***** Conventional Phasor Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//          Power System Network Relay Convention
//------------------------------------------------------------------------------------------------------------
//
//                                                   (Real Axis)
//                                          -Power Flowing into the Load-
//                                                      0 deg
//                                                       +P
//                                                        ^
//                                         II             |               I
//                                 (Leading Current)      |       (Lagging Current)
//                                                        |
//                                     +P (Watts)         |     +P (Watts)
//                                     -Q (VARs)          |     +Q (VARs)
//                                                        |     
//                           <- VARs sourced to the line  |  VARs consumed by the load ->
//                 90deg -Q ------------------------------+------------------------------> +Q 270deg
//                                                        |
//                                     -Q (VARs)          |     +Q (VARs)
//                                     -P (Watts)         |     -P (Watts)
//                                                        |
//                                                        |
//                                                        |
//                                         III            |              IV               Omega = ccw
//                                                        |
//                                                       -P
//                                                     180deg
//                                         -Power Flowing into the Source-
//
//       Note: 1) This is a conventional phasor diagram rotated 90deg CCW.  Omegs is rotating CCW.
//
//             2) Phase angle measures current relative to voltage:
//                   (i.e. 45 deg => current leads voltage by 45 deg)
//
//             3) VARs sign convention: S = P + jQ = VI*
//       
//
//                                            ***** MPCV Relay View *****
//
//
//------------------------------------------------------------------------------------------------------------
//          NRX1150 Sign Convention Setpoints...
//------------------------------------------------------------------------------------------------------------
//
//          The NRX1150 is capable of four-quadrant metering and has a setpoint (Setpoints0.stp.SignConv) that 
//          determines the sign convention used for reactive power and power factor.
//             Setpoints0.stp.SignConv has three settings:
//             - 0 IEC Convention:  This setting uses the power engineering convention where P = VI*.  
//                   Additionally, the sign of PF follows the mathmatical definition of PF(app) = P / VA.
//
//             - 1 IEEE Convention:  This setting uses the power engineering convention where P = VI*.  
//                   Additionally, the sign of PF is positive if the signs of Watts & VARs are the same.  A
//                   lagging PF is positive.
//
//             - 2 Alternate IEEE Convention:  This setting uses the direct mathematical convention where 
//                   P = VI.  Additionally, the sign of PF is positive if the signs of Watts & VARs are the 
//                   same.  A lagging PF is negative.
//
//          An additional setpoint (Setpoints0.stp.RevFeed) can be used to invert the phase of the current
//          when the breaker is wired in a back-feed cofiguration.  The effect of setting this setpoint to 1
//          is to interchange quadrants 1 & 3 and 2 & 4.
//
//          The phase angle between voltage and current is always the phase of current relative to voltage.  
//          A phase angle (theta) of +45 deg implies the fundamental current leads the voltage by 45 deg.  
//          The terms "leading" and "lagging" current are always with respect to the source of real power.  
//          If theta exceeds +/- 90 degrees, the load is supplying power and the current "leads" for phase 
//          angles 180 > theta > 270.  Quadrants 1 & 3 are "Leading" while quadrants 2 & 4 are "lagging".
//
//          Note: The sign convention setpoint duplicates operation of the equivalent setpoint in the
//          IQ Analyzer (ref TD17530) with the addition of the IEC option.  Selecting the "IEEE Convention"
//          will provide sign compatibility with other Eaton products and must be used if a ProfiBus CAM
//          is attached.            
//
//------------------------------------------------------------------------------------------------------------
//          IEC (Power Engineering) Convention:  P = VI*
//------------------------------------------------------------------------------------------------------------
//       
//          + Setpoints0.stp.SignConv = 0: IEC (Power Engineering) Convention  P = VI*
//             This setting uses the power engineering convention where power is the complex product of volts 
//             times the conjugate of current.  Positive Watts & VARs are delivered to an inductive load: 
//             - Watts are positive when the phase angle of current relative to voltage is -90 < theta < +90.
//             - Vars are positive when the phase angle of current relative to voltage is 180 < theta < 360.  
//               Under these conditions, current lags the voltage.
//             - The sign of PF follows the sign of Watts.  The follows the mathmatical definition of 
//               PF(app) = P / VA. 
//
//          + Setpoints0.stp.RevFeed = 1: Quadrants 1 & 3 and 2 & 4 are interchanged:
//             - Watts are inverted.
//             - VARs are inverted.
//             - PF is inverted.
//
//                                                  (Imaginary Axis)
//                                                      90 deg
//                                                        +j
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                (Lagging Current)       |       (Leading Current)
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     -Q (VARs)          |     -Q (VARs)
//                                     (-) PF             |     (+) PF
//                                                        |
//                       <-Power Flowing into the Source  |  Power Flowing into the Load->
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//                                                        |
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     +Q (VARs)          |     +Q (VARs)
//                                     (-) PF             |     (+) PF
//                                                        |
//                                 (Leading Current)      |      (Lagging Current)        Omega = ccw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                        -j
//                                                      270 deg
//
//                                      ***** Conventional Phasor Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//          IEEE (Power Engineering) Convention: P = VI*
//------------------------------------------------------------------------------------------------------------
//       
//          + Setpoints0.stp.SignConv = 1: IEEE (Power Engineering) Convention P = VI*
//             This setting uses the power engineering convention where power is the product of volts times
//             the complex conjugate of current.  Positive Watts & VARs are delivered to an inductive load: 
//             - Watts are positive when the phase angle of current relative to voltage is -90 < theta < +90.
//             - Vars are positive when the phase angle of current relative to voltage is 180 < theta < 360.  
//               Under these conditions, current lags the voltage.
//             - The sign of PF is positive if the signs of Watts & VARs are the same.  For those conditions 
//               where real power is flowing into the load, a lagging (inductive) PF is positive.
//
//          + Setpoints0.stp.RevFeed = 1: Quadrants 1 & 3 and 2 & 4 are interchanged:
//             - Watts are inverted.
//             - VARs are inverted.
//            
//
//                                                  (Imaginary Axis)
//                                                      90 deg
//                                                        +j
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                (Lagging Current)       |       (Leading Current)
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     -Q (VARs)          |     -Q (VARs)
//                                     (+) PF             |     (-) PF
//                                                        |
//                       <-Power Flowing into the Source  |  Power Flowing into the Load->
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//                                                        |
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     +Q (VARs)          |     +Q (VARs)
//                                     (-) PF             |     (+) PF
//                                                        |
//                                 (Leading Current)      |      (Lagging Current)        Omega = ccw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                        -j
//                                                      270 deg
//       
//                                      ***** Conventional Phasor Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//          Alternate IEEE (Direct Mathematical) Convention: P = VI
//------------------------------------------------------------------------------------------------------------
//
//          + Setpoints0.stp.SignConv = 2: Alternate IEEE (Direct Mathematical) Convention P = VI
//             This setting uses the direct mathematical convention where power is the complex product of volts 
//             times current.  Positive Watts & negative VARs are delivered to an inductive load: 
//             - Watts are positive when the phase angle of current relative to voltage is -90 < theta < +90.
//             - Vars are positive when the phase angle of current relative to voltage is 0 < theta < 180.  Under
//               these conditions, current leads the voltage.  This is the inversion of the IEEE setting
//             - The sign of PF is positive if the signs of Watts & VARs are the same.  For those conditions 
//               where real power is flowing into the load, a lagging (inductive) PF is negative.
//
//          + Setpoints0.stp.RevFeed = 1: Quadrants 1 & 3 and 2 & 4 are interchanged:
//             - Watts are inverted.
//             - VARs are inverted.
//            
//
//                                                  (Imaginary Axis)
//                                                      90 deg
//                                                        +j
//                                                        ^
//                                    Quadrant 2          |          Quadrant 1
//                                (Lagging Current)       |       (Leading Current)
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     +Q (VARs)          |     +Q (VARs)
//                                     (-) PF             |     (+) PF
//                                                        |
//                       <-Power Flowing into the Source  |  Power Flowing into the Load->
//       Power (W) 180 deg -------------------------------+-------------------------------> 0 deg Power (W)
//                                                        |
//                                                        |
//                                     -P (Watts)         |     +P (Watts)
//                                     -Q (VARs)          |     -Q (VARs)
//                                     (+) PF             |     (-) PF
//                                                        |
//                                 (Leading Current)      |      (Lagging Current)        Omega = ccw
//                                    Quadrant 3          |          Quadrant 4
//                                                        |
//                                                        -j
//                                                      270 deg
//       
//                                      ***** Conventional Phasor Diagram View *****
//
//------------------------------------------------------------------------------------------------------------
//       Power Sign Computation...
//------------------------------------------------------------------------------------------------------------
//
//       Real Power (Watts)
//
//          Real power is computed by integrating the product of the instantaneous phase current sample
//          and the corresponding line-to-neutral voltage sample (the current sample is actually delayed by
//          one sampling time to account for the phase-shift through the voltage channel) over a single line
//          cycle.  This calcualtion results in positive power calculation when the current and voltage are 
//          in-phase (Quadrant 1 & 4) and the breaker is fed from it's line-side.
//
//                - Setpoints0.stp.SignConv: This setpoint has no effect on the diplay of Real Power.          
//
//                - Setpoints0.stp.RevFeed: This setpoint inverts the sign of computed Real Power when set
//                  to a 1.
//
//
//       Reactive Power (VARs)
//
//          Reactive power is computed by integrating the product of the instantaneous phase current sample
//          and the corresponding line-to-neutral voltage sample delayed by 90degrees (16 samples) over a
//          single line cycle.  In this instance, the current sample is also delayed by one sampling time to 
//          account for the phase-shift through the voltage channel.  This calculation results in positive 
//          reactive power calculation when the load current lags the voltage by 0 to 180degrees 
//          (180 < theta < 360), assuming the breaker is fed from it's line-side (Quadrant 1 & 2).
//
//                - Setpoints0.stp.SignConv: This setpoint inverts the sign of computed Reactive Power when
//                  set to a 2 (IEEEalt).       
//
//                - Setpoints0.stp.RevFeed: This setpoint inverts the sign of computed Reactive Power when set
//                  to a 1. (Note the potential of a double sign inversion due to both setpoints)
//
//
//       Apparent Power (VA)
//
//          Apparent power is computed by multiplying each phase's rms voltage by the phase rms current.
//          Total apparent power is the sum of the three per-phase power.  This numbeer is always a positive
//          value and is not effected by either setpoint.
//
//                - Setpoints0.stp.SignConv: This setpoint has no effect on the diplay of Apparent Power.          
//
//                - Setpoints0.stp.RevFeed: This setpoint has no effect on the diplay of Apparent Power.
//
//------------------------------------------------------------------------------------------------------------
//       Energy Sign Computation...
//------------------------------------------------------------------------------------------------------------
//
//       Real Energy (kWh)
//
//          Real energy is computed by integrating the real power of each line cycle over time.  The 
//          computation is done using two volatile energy accumulators and two non-volatile energy tally 
//          registers: one pair for forward energy, the other for reverse energy.  The wiring of the current
//          and voltage sensors results in power & energy calculations using the IEEE/IEC sign convention: 
//          Watts are  positive in quadrants 1 & 4 when the breaker is delivering power to its load side.  
//          Every line cycle, the cycle's energy is distribuited as follows:
//             - Positive Watts are deposited in the forward Wh accumulator.
//             - Negative Watts are deposited in the reverse Wh accumulator.
//          Whenever any accumulator exceeds 1 kWh, 1 is added to the appropriate permanent energy tally 
//          register and 1 kWh is subtracted from the accumulator.  The accumulators are volatile over 
//          power-cycle, the tally registers are stored in FRAM.  Note, the accumulators and tally registers
//          are unsigned.  Sign conventions are applied to these energy registers only when they are 
//          communicated to the panel or test port.  Setpoints have no effect on the calculation of energy
//          accumulators.
//
//                - Setpoints0.stp.SignConv: This setpoint has no effect on the diplay of Real Energy.          
//
//                - Setpoints0.stp.RevFeed: This setpoint inverts the sign of displayed Real Energy when set
//                  to a 1.
//
//
//       Reactive Energy (kVARh)
//
//          Reactive energy is computed by integrating the reactive power of each line cycle over time.  The 
//          computation is done using two volatile energy accumulators and two non-volatile energy tally 
//          registers: one pair for positive (lagging) VARhs, the other for negitive (leading) VARhs.  The 
//          wiring of the current and voltage sensors results in power & energy calculations using the 
//          IEEE/IEC sign convention:  VARs are  positive in quadrants 1 & 4 when the breaker is delivering 
//          reactive power to its load side.  Every line cycle, the cycle's energy is distribuited as follows:
//             - Positive VARs are deposited in the posivive VARh accumulator.
//             - Negative VARs are deposited in the negative VARh accumulator.
//          Whenever any accumulator exceeds 1 kVARh, 1 is added to the appropriate permanent energy tally 
//          register and 1 kVARh is subtracted from the accumulator.  The accumulators are volatile over 
//          power-cycle, the tally registers are stored in FRAM.  Note, the accumulators and tally registers
//          are unsigned.  Sign conventions are applied to these energy registers only when they are 
//          communicated to the panel or test port.  Setpoints have no effect on the calculation of energy
//          accumulators.
//
//          Note: the terms "leading VARh" and "lagging VARh" are misleading.  Lagging forward VARs
//          (270 < theta < 360) and leading reverse VARs (180 < theta < 270) accumulate energy in the 
//          positive VARh accumulator.
//
//                - Setpoints0.stp.SignConv = 2 (IEEEalt): Positive and Negative VARh tallys are interchanged.   
//
//                - Setpoints0.stp.RevFeed = 1: Positive and Negative VARh tallys are interchanged.  (Note
//                  the potential for a double inversion if both setpoints are in the proper state.)   
//
//
//
//------------------------------------------------------------------------------------------------------------
//       Power Factor Sign Computation...
//------------------------------------------------------------------------------------------------------------
//
//       Power Factor (PF)
//
//          Power factor is computed based on the signs of Watts, VARs and Setpoints0.stp.SignConv.
//                - Setpoints0.stp.SignConv = 0 (IEC)
//                         PF is the sign of computed Real Power.
//
//                - Setpoints0.stp.SignConv = 1 (IEEE)
//                         PF is + if the signs of internal Real Power & Reactive Power are different.
//                         PF is - if the signs of internal Real Power & Reactive Power are the same.
//
//                - Setpoints0.stp.SignConv = 2 (Alternate IEEE)
//                         PF is + if the signs of internal Real Power & Reactive Power are the same.
//                         PF is - if the signs of internal Real Power & Reactive Power are different.
//








                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING START
void Load_MB_Test_Vals(void)
{
  struct SYSTICK_TIME tSysTickTime;

  Cur200msFltr.Ia = 0.1F;                  // 1802
  Cur200msFltr.Ib = 0.2F;                  // 1804
  Cur200msFltr.Ic = 0.3F;                  // 1806
  Cur200msFltr.Igres = 0.4F;
  Cur200msFltrIg = Cur200msFltr.Igres;     // 1808
  Cur200msFltr.In = 0.5F;                  // 180A
  Cur200msIavg = 0.6F;                     // 180C
  VolAFE200msFltr.Vab = 0.7F;              // 180E
  VolAFE200msFltr.Vbc = 0.8F;              // 1810
  VolAFE200msFltr.Vca = 0.9F;              // 1812
  VolAFE200msFltrVllavg = 1.0F;            // 1814
  VolAFE200msFltr.Van = 1.1F;              // 1816
  VolAFE200msFltr.Vbn = 1.2F;              // 1818
  VolAFE200msFltr.Vcn = 1.3F;              // 181A
  VolAFE200msFltrVlnavg = 1.4F;            // 181C
  // NOT SUPPORTED                         // 181E
//  IDmnd.IaMax = 1.5F;                      // 1820
//  IDmnd.IbMax = 1.6F;                      // 1822
//  IDmnd.IcMax = 1.7F;                      // 1824
  // NOT SUPPORTED                         // 1826
//  IDmnd.InMax = 1.8F;                      // 1828
  Pwr200msec.Ptot = 19.0F;                 // 182A
  Pwr200msec.Rtot = 20.0F;                 // 182C
  Pwr200msecApp.Apptot = 21.0F;            // 182E
  PF.Disp[3] = 0.22F;                      // 1830
  PF.App[3] = 0.23F;                       // 1832
//  FreqLoad.FreqVal = 2.4F;                 // 1834
  // NOT SUPPORTED                         // 1836
//  IDmnd.IaMin = 2.5F;                      // 1838
//  IDmnd.IbMin = 2.6F;                      // 183A
//  IDmnd.IcMin = 2.7F;                      // 183C
  // NOT SUPPORTED                         // 183E
//  IDmnd.InMin = 2.8F;                      // 1840
//  FreqLine.FreqVal = 2.9F;                 // 1842
  FreqLine.FreqVal = 0.30F;                // 1844
//  FreqLine.MinFreqVal = 0.31F;             // 1846
//  FreqLine.MaxFreqVal = 0.32F;             // 1848
  // NOT SUPPORTED                         // 184A
  PF.Disp[0] = .33F;                       // 184C
  PF.Disp[1] = .34F;                       // 184E
  PF.Disp[2] = .35F;                       // 1850
  PF.App[0] = .36F;                        // 1852
  PF.App[1] = .37F;                        // 1854
  PF.App[2] = .38F;                        // 1856
//  PDmnd.TotWMax = 39.0F;                   // 1858
  // NOT SUPPORTED                         // 185A
  // NOT SUPPORTED                         // 185C
  // NOT SUPPORTED                         // 185E
  // NOT SUPPORTED                         // 1860
  // NOT SUPPORTED                         // 1862
  // NOT SUPPORTED                         // 1864
  // NOT SUPPORTED                         // 1866
  // NOT SUPPORTED                         // 1868
  // NOT SUPPORTED                         // 186A
  // NOT SUPPORTED                         // 186C
  // NOT SUPPORTED                         // 186E
  FreqLoad.FreqVal = 0.40F;                // 1870

  THSensor.Temperature = 95.0F;
  THSensor.TemperatureMax = 250.0F;

  EngyDmnd[1].TotFwdWHr = (1000 * 100);
  EngyDmnd[1].TotRevWHr = (1000 * 200);
  TotWHr = (1000 * 300);
  EngyDmnd[1].TotLeadVarHr = (1000 * 400);
  EngyDmnd[1].TotLagVarHr = (1000 * 500);
  NetVarHr = -(1000 * 300);
  EngyDmnd[1].TotVAHr = (1000 * 600);

//  PDmnd.TotVarMax = 51;
//  PDmnd.TotVAMax = 52;
  Pwr200msec.Pa = 53;
  Pwr200msec.Pb = 54;
  Pwr200msec.Pc = 55;
  Pwr200msec.RPa = 56;
  Pwr200msec.RPb = 57;
  Pwr200msec.RPc = 58;
  Pwr200msecApp.AppPa = 59;
  Pwr200msecApp.AppPb = 60;
  Pwr200msecApp.AppPc = 61;
  EngyDmnd[1].DmndIa = 62.0/10;
  EngyDmnd[1].DmndIb = 63.0/10;
  EngyDmnd[1].DmndIc = 64.0/10;
  EngyDmnd[1].DmndIn = 65.0/10;
  EngyDmnd[1].DmndTotW = 66;
  EngyDmnd[1].DmndTotVar = 67;
  EngyDmnd[1].DmndTotVA = 68;
//  CurOneCyc_min.Ia = 69.0/10;
//  CurOneCyc_max.Ia = 70.0/10;
//  CurOneCyc_min.Ib = 71.0/10;
//  CurOneCyc_max.Ib = 72.0/10;
//  CurOneCyc_min.Ic = 73.0/10;
//  CurOneCyc_max.Ic = 74.0/10;
//  CurOneCyc_min.Ig = 75.0/10;
//  CurOneCyc_max.Ig = 76.0/10;
//  CurOneCyc_min.In = 77.0/10;
//  CurOneCyc_max.In = 78.0/10;
//  VolAFEOneCyc_min.Vab = 79.0/10;
//  VolAFEOneCyc_max.Vab = 80.0/10;
//  VolAFEOneCyc_min.Vbc = 81.0/10;
//  VolAFEOneCyc_max.Vbc = 82.0/10;
//  VolAFEOneCyc_min.Vca = 83.0/10;
//  VolAFEOneCyc_max.Vca = 84.0/10;
//  VolAFEOneCyc_min.Van = 85.0/10;
//  VolAFEOneCyc_max.Van = 86.0/10;
//  VolAFEOneCyc_min.Vbn = 87.0/10;
//  VolAFEOneCyc_max.Vbn = 88.0/10;
//  VolAFEOneCyc_min.Vcn = 89.0/10;
//  VolAFEOneCyc_max.Vcn = 90.0/10;
//  PF.MinApp[3] = 91.0/100;
//  PF.MaxApp[3] = 92.0/100;
//  FreqLoad.MinFreqVal = 93.0/100;
//  FreqLoad.MaxFreqVal = 94.0/100;
  CurUnbalTot = 95.0/100;
  VolUnbalTot = 96.0/100;
  THD[0] = 97.0/100;
  THD[1] = 98.0/100;
  THD[2] = 99.0/100;
  THD[3] = 100.0/100;
  THD[7] = 101.0/100;
  THD[8] = 102.0/100;
  THD[9] = 103.0/100;
  THD[4] = 104.0/100;
  THD[5] = 105.0/100;
  THD[6] = 106.0/100;
  CF.Ia = 107.0/100;
  CF.Ib = 108.0/100;
  CF.Ic = 109.0/100;
  CF.In = 110.0/100;
  KF_Val.Ia = 111;
  KF_Val.Ib = 112;
  KF_Val.Ic = 113;
  VolADC200ms.Vab = 150.0/10;
  VolADC200ms.Vbc = 151.0/10;
  VolADC200ms.Vca = 152.0/10;
  VolADC200msVllavg = 153.0/10;
  VolADC200ms.Van = 154.0/10;
  VolADC200ms.Vbn = 155.0/10;
  VolADC200ms.Vcn = 156.0/10;
  VolADC200msVlnavg = 157.0/10;
  SeqComp.V_PosMag = 158.0/10;
  SeqComp.V_PosPh = 159.0/10;
  SeqComp.V_NegMag = 160.0/10;
  SeqComp.V_NegPh = 161.0/10;
  SeqComp.V_ZeroMag = 162.0/10;
  SeqComp.V_ZeroPh = 163.0/10;
//  VolADCOneCyc_min.Vab = 164.0/10;
//  VolADCOneCyc_max.Vab = 165.0/10;
//  VolADCOneCyc_min.Vbc = 166.0/10;
//  VolADCOneCyc_max.Vbc = 167.0/10;
//  VolADCOneCyc_min.Vca = 168.0/10;
//  VolADCOneCyc_max.Vca = 169.0/10;
//  VolADCOneCyc_min.Van = 170.0/10;
//  VolADCOneCyc_max.Van = 171.0/10;
//  VolADCOneCyc_min.Vbn = 172.0/10;
//  VolADCOneCyc_max.Vbn = 173.0/10;
//  VolADCOneCyc_min.Vcn = 174.0/10;
//  VolADCOneCyc_max.Vcn = 175.0/10;
//  Pwr200msecMin.Pa = 176.0;
//  Pwr200msecMax.Pa = 177.0;
//  Pwr200msecMin.Pb = 178.0;
//  Pwr200msecMax.Pb = 179.0;
//  Pwr200msecMin.Pc = 180.0;
//  Pwr200msecMax.Pc = 181.0;
//  Pwr200msecMin.Ptot = 182.0;
//  Pwr200msecMax.Ptot = 183.0;
//  Pwr200msecMin.RPa = 184.0;
//  Pwr200msecMax.RPa = 185.0;
//  Pwr200msecMin.RPb = 186.0;
//  Pwr200msecMax.RPb = 187.0;
//  Pwr200msecMin.RPc = 188.0;
//  Pwr200msecMax.RPc = 189.0;
//  Pwr200msecMin.Rtot = 190.0;
//  Pwr200msecMax.Rtot = 191.0;
//  Pwr200msecAppMin.AppPa = 192.0;
//  Pwr200msecAppMax.AppPa = 193.0;
//  Pwr200msecAppMin.AppPb = 194.0;
//  Pwr200msecAppMax.AppPb = 195.0;
//  Pwr200msecAppMin.AppPc = 196.0;
//  Pwr200msecAppMax.AppPc = 197.0;
//  Pwr200msecAppMin.Apptot = 198.0;
//  Pwr200msecAppMax.Apptot = 199.0;
  PhAngles1[0] = 200.0/100;
  PhAngles1[1] = 201.0/100;
  PhAngles1[2] = 202.0/100;
  PhAngles1[3] = 203.0/100;
  PhAngles1[4] = 204.0/100;
  PhAngles1[5] = 205.0/100;
  PhAngles1[6] = 206.0/100;
  PhAngles1[7] = 207.0/100;
  PhAngles1[8] = 208.0/100;
  PhAngles2[0] = 209.0/100;
  PhAngles2[1] = 210.0/100;
  PhAngles2[2] = 211.0/100;
  PhAngles2[3] = 212.0/100;
  PhAngles2[4] = 213.0/100;
  PhAngles2[5] = 214.0/100;
  SeqComp.I_PosMag = 215.0/10;
  SeqComp.I_NegMag = 216.0/10;
  SeqComp.I_ZeroMag = 217.0/10;
  SeqComp.I_PosPh = 218.0/10;
  SeqComp.I_NegPh = 219.0/10;
  SeqComp.I_ZeroPh = 220.0/10;
//  PF.MinDisp[3] = 221.0/100;
//  PF.MaxDisp[3] = 222.0/100;
//  PF.MinApp[0] = 223.0/100;
//  PF.MaxApp[0] = 224.0/100;
//  PF.MinApp[1] = 225.0/100;
//  PF.MaxApp[1] = 226.0/100;
//  PF.MinApp[2] = 227.0/100;
//  PF.MaxApp[2] = 228.0/100;
//  PF.MinDisp[0] = 229.0/100;
//  PF.MaxDisp[0] = 230.0/100;
//  PF.MinDisp[1] = 231.0/100;
//  PF.MaxDisp[1] = 232.0/100;
//  PF.MinDisp[2] = 233.0/100;
//  PF.MaxDisp[2] = 234.0/100;


// *** DAH generate internal time for a canned time of 06/27/2022  11:00:00 and add 15 seconds each time
  RTC_buf[0] = 0;  // hundredths of a second in hexadecimal (0 - 99)
  RTC_buf[1] = 0;  // seconds in hexadecimal (0 - 59)
  RTC_buf[2] = 0;  // minutes in hexadecimal (0 - 59)
  RTC_buf[3] = 11; // hours in hexadecimal (0 - 23)
  RTC_buf[4] = 0;  // day in hexadecimal - not used here
  RTC_buf[5] = 27; // date in hexadecimal (1 - 31)
  RTC_buf[6] = 6;  // month in hexadecimal (1 - 12)
  RTC_buf[7] = 22; // year in hexadecimal (0 - 99)
  RTC_to_SysTickTime(&tSysTickTime);
// *** DAH
  

  CurOneCyc_min.Ia = 240.0/10.0;
//  CurOneCyc_min.IaTS.Time_secs = 0x295A6D00;
  CurOneCyc_min.IaTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_min.IaTS.Time_nsec = 1000000;
  CurOneCyc_max.Ia = 241.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_max.IaTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_max.IaTS.Time_nsec = 2000000;
  CurOneCyc_min.Ib = 242.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_min.IbTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_min.IbTS.Time_nsec = 3000000;
  CurOneCyc_max.Ib = 243.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_max.IbTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_max.IbTS.Time_nsec = 4000000;
  CurOneCyc_min.Ic = 244.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_min.IcTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_min.IcTS.Time_nsec = 5000000;
  CurOneCyc_max.Ic = 245.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_max.IcTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_max.IcTS.Time_nsec = 6000000;
  CurOneCyc_min.Ig = 246.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_min.IgTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_min.IgTS.Time_nsec = 7000000;
  CurOneCyc_max.Ig = 247.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_max.IgTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_max.IgTS.Time_nsec = 8000000;
  CurOneCyc_min.In = 248.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_min.InTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_min.InTS.Time_nsec = 9000000;
  CurOneCyc_max.In = 249.0/10.0;
  tSysTickTime.cnt_sec += 15;
  CurOneCyc_max.InTS.Time_secs = tSysTickTime.cnt_sec;
  CurOneCyc_max.InTS.Time_nsec = 10000000;
  VolAFEOneCyc_min.Vab = 250.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VabTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VabTS.Time_nsec = 11000000;
  VolAFEOneCyc_max.Vab = 251.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VabTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VabTS.Time_nsec = 12000000;
  VolAFEOneCyc_min.Vbc = 252.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VbcTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VbcTS.Time_nsec = 13000000;
  VolAFEOneCyc_max.Vbc = 253.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VbcTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VbcTS.Time_nsec = 14000000;
  VolAFEOneCyc_min.Vca = 254.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VcaTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VcaTS.Time_nsec = 15000000;
  VolAFEOneCyc_max.Vca = 255.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VcaTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VcaTS.Time_nsec = 16000000;
  VolAFEOneCyc_min.Van = 256.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VanTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VanTS.Time_nsec = 17000000;
  VolAFEOneCyc_max.Van = 257.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VanTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VanTS.Time_nsec = 18000000;
  VolAFEOneCyc_min.Vbn = 258.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VbnTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VbnTS.Time_nsec = 19000000;
  VolAFEOneCyc_max.Vbn = 259.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VbnTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VbnTS.Time_nsec = 20000000;
  VolAFEOneCyc_min.Vcn = 260.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_min.VcnTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_min.VcnTS.Time_nsec = 21000000;
  VolAFEOneCyc_max.Vcn = 261.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolAFEOneCyc_max.VcnTS.Time_secs = tSysTickTime.cnt_sec;
  VolAFEOneCyc_max.VcnTS.Time_nsec = 22000000;
  VolADCOneCyc_min.Vab = 262.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VabTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VabTS.Time_nsec = 23000000;
  VolADCOneCyc_max.Vab = 263.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VabTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VabTS.Time_nsec = 24000000;
  VolADCOneCyc_min.Vbc = 264.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VbcTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VbcTS.Time_nsec = 25000000;
  VolADCOneCyc_max.Vbc = 265.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VbcTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VbcTS.Time_nsec = 26000000;
  VolADCOneCyc_min.Vca = 266.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VcaTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VcaTS.Time_nsec = 27000000;
  VolADCOneCyc_max.Vca = 267.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VcaTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VcaTS.Time_nsec = 28000000;
  VolADCOneCyc_min.Van = 268.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VanTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VanTS.Time_nsec = 29000000;
  VolADCOneCyc_max.Van = 269.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VanTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VanTS.Time_nsec = 30000000;
  VolADCOneCyc_min.Vbn = 270.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VbnTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VbnTS.Time_nsec = 31000000;
  VolADCOneCyc_max.Vbn = 271.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VbnTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VbnTS.Time_nsec = 32000000;
  VolADCOneCyc_min.Vcn = 272.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_min.VcnTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_min.VcnTS.Time_nsec = 33000000;
  VolADCOneCyc_max.Vcn = 273.0/10.0;
  tSysTickTime.cnt_sec += 15;
  VolADCOneCyc_max.VcnTS.Time_secs = tSysTickTime.cnt_sec;
  VolADCOneCyc_max.VcnTS.Time_nsec = 34000000;
  Pwr200msecMin.Pa = -234.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.PaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.PaTS.Time_nsec = 35000000;
  Pwr200msecMax.Pa = -235.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.PaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.PaTS.Time_nsec = 36000000;
  Pwr200msecMin.Pb = -236.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.PbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.PbTS.Time_nsec = 37000000;
  Pwr200msecMax.Pb = -237.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.PbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.PbTS.Time_nsec = 38000000;
  Pwr200msecMin.Pc = -238.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.PcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.PcTS.Time_nsec = 39000000;
  Pwr200msecMax.Pc = -239.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.PcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.PcTS.Time_nsec = 40000000;
  Pwr200msecMin.Ptot = -240.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.PtotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.PtotTS.Time_nsec = 41000000;
  Pwr200msecMax.Ptot = -241.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.PtotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.PtotTS.Time_nsec = 42000000;
  Pwr200msecMin.RPa = -242.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.RPaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.RPaTS.Time_nsec = 43000000;
  Pwr200msecMax.RPa = -243.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.RPaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.RPaTS.Time_nsec = 44000000;
  Pwr200msecMin.RPb = -244.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.RPbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.RPbTS.Time_nsec = 45000000;
  Pwr200msecMax.RPb = -245.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.RPbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.RPbTS.Time_nsec = 46000000;
  Pwr200msecMin.RPc = -246.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.RPcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.RPcTS.Time_nsec = 47000000;
  Pwr200msecMax.RPc = -247.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.RPcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.RPcTS.Time_nsec = 48000000;
  Pwr200msecMin.Rtot = -248.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMin.RtotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMin.RtotTS.Time_nsec = 49000000;
  Pwr200msecMax.Rtot = -249.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecMax.RtotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecMax.RtotTS.Time_nsec = 50000000;
  Pwr200msecAppMin.AppPa = 350.0; 
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMin.AppPaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMin.AppPaTS.Time_nsec = 51000000;
  Pwr200msecAppMax.AppPa = 351.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMax.AppPaTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMax.AppPaTS.Time_nsec = 52000000;
  Pwr200msecAppMin.AppPb = 352.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMin.AppPbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMin.AppPbTS.Time_nsec = 53000000;
  Pwr200msecAppMax.AppPb = 353.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMax.AppPbTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMax.AppPbTS.Time_nsec = 54000000;
  Pwr200msecAppMin.AppPc = 354.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMin.AppPcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMin.AppPcTS.Time_nsec = 55000000;
  Pwr200msecAppMax.AppPc = 355.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMax.AppPcTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMax.AppPcTS.Time_nsec = 56000000;
  Pwr200msecAppMin.Apptot = 356.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMin.ApptotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMin.ApptotTS.Time_nsec = 57000000;
  Pwr200msecAppMax.Apptot = 357.0;
  tSysTickTime.cnt_sec += 15;
  Pwr200msecAppMax.ApptotTS.Time_secs = tSysTickTime.cnt_sec;
  Pwr200msecAppMax.ApptotTS.Time_nsec = 58000000;

// *** DAH generate internal time for a canned time of 11/24/2022  17:00:00 and add 5 seconds each time
  RTC_buf[0] = 0;  // hundredths of a second in hexadecimal (0 - 99)
  RTC_buf[1] = 0;  // seconds in hexadecimal (0 - 59)
  RTC_buf[2] = 0;  // minutes in hexadecimal (0 - 59)
  RTC_buf[3] = 17; // hours in hexadecimal (0 - 23)
  RTC_buf[4] = 0;  // day in hexadecimal - not used here
  RTC_buf[5] = 24; // date in hexadecimal (1 - 31)
  RTC_buf[6] = 11; // month in hexadecimal (1 - 12)
  RTC_buf[7] = 22; // year in hexadecimal (0 - 99)
  RTC_to_SysTickTime(&tSysTickTime);
// *** DAH

  PF.MinApp[0] = 50.0/100.0;
  PF.MinApp_TS[0].Time_secs = tSysTickTime.cnt_sec;
  PF.MinApp_TS[0].Time_nsec = 100000000;
  PF.MaxApp[0] = 51.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxApp_TS[0].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxApp_TS[0].Time_nsec = 101000000;
  PF.MinApp[1] = 52.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinApp_TS[1].Time_secs = tSysTickTime.cnt_sec;
  PF.MinApp_TS[1].Time_nsec = 102000000;
  PF.MaxApp[1] = 53.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxApp_TS[1].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxApp_TS[1].Time_nsec = 103000000;
  PF.MinApp[2] = 54.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinApp_TS[2].Time_secs = tSysTickTime.cnt_sec;
  PF.MinApp_TS[2].Time_nsec = 104000000;
  PF.MaxApp[2] = 55.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxApp_TS[2].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxApp_TS[2].Time_nsec = 105000000;
  PF.MinApp[3] = 56.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinApp_TS[3].Time_secs = tSysTickTime.cnt_sec;
  PF.MinApp_TS[3].Time_nsec = 106000000;
  PF.MaxApp[3] = 57.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxApp_TS[3].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxApp_TS[3].Time_nsec = 107000000;
  PF.MinDisp[0] = 58.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinDisp_TS[0].Time_secs = tSysTickTime.cnt_sec;
  PF.MinDisp_TS[0].Time_nsec = 108000000;
  PF.MaxDisp[0] = 59.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxDisp_TS[0].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxDisp_TS[0].Time_nsec = 109000000;
  PF.MinDisp[1] = 60.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinDisp_TS[1].Time_secs = tSysTickTime.cnt_sec;
  PF.MinDisp_TS[1].Time_nsec = 110000000;
  PF.MaxDisp[1] = 61.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxDisp_TS[1].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxDisp_TS[1].Time_nsec = 111000000;
  PF.MinDisp[2] = 62.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinDisp_TS[2].Time_secs = tSysTickTime.cnt_sec;
  PF.MinDisp_TS[2].Time_nsec = 112000000;
  PF.MaxDisp[2] = 63.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxDisp_TS[2].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxDisp_TS[2].Time_nsec = 113000000;
  PF.MinDisp[3] = 64.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MinDisp_TS[3].Time_secs = tSysTickTime.cnt_sec;
  PF.MinDisp_TS[3].Time_nsec = 114000000;
  PF.MaxDisp[3] = 65.0/100.0;
  tSysTickTime.cnt_sec += 5;
  PF.MaxDisp_TS[3].Time_secs = tSysTickTime.cnt_sec;
  PF.MaxDisp_TS[3].Time_nsec = 115000000;

  FreqLoad.MinFreqVal = 66.0/100.0;
  tSysTickTime.cnt_sec += 5;
  FreqLoad.MinFreqVal_TS.Time_secs = tSysTickTime.cnt_sec;
  FreqLoad.MinFreqVal_TS.Time_nsec = 116000000;
  FreqLoad.MaxFreqVal = 67.0/100.0;
  tSysTickTime.cnt_sec += 5;
  FreqLoad.MaxFreqVal_TS.Time_secs = tSysTickTime.cnt_sec;
  FreqLoad.MaxFreqVal_TS.Time_nsec = 117000000;
  FreqLine.MinFreqVal = 68.0/100.0;
  tSysTickTime.cnt_sec += 5;
  FreqLine.MinFreqVal_TS.Time_secs = tSysTickTime.cnt_sec;
  FreqLine.MinFreqVal_TS.Time_nsec = 118000000;
  FreqLine.MaxFreqVal = 69.0/100.0;
  tSysTickTime.cnt_sec += 5;
  FreqLine.MaxFreqVal_TS.Time_secs = tSysTickTime.cnt_sec;
  FreqLine.MaxFreqVal_TS.Time_nsec = 119000000;

  IDmnd.IaMin = 470.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IaMinTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IaMinTS.Time_nsec = 120000000;
  IDmnd.IaMax = 471.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IaMaxTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IaMaxTS.Time_nsec = 121000000;
  IDmnd.IbMin = 472.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IbMinTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IbMinTS.Time_nsec = 122000000;
  IDmnd.IbMax = 473.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IbMaxTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IbMaxTS.Time_nsec = 123000000;
  IDmnd.IcMin = 474.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IcMinTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IcMinTS.Time_nsec = 124000000;
  IDmnd.IcMax = 475.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.IcMaxTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.IcMaxTS.Time_nsec = 125000000;
  IDmnd.InMin = 476.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.InMinTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.InMinTS.Time_nsec = 126000000;
  IDmnd.InMax = 477.0/10.0;
  tSysTickTime.cnt_sec += 5;
  IDmnd.InMaxTS.Time_secs = tSysTickTime.cnt_sec;
  IDmnd.InMaxTS.Time_nsec = 127000000;
  PDmnd.TotWMax = 478.0;
  tSysTickTime.cnt_sec += 5;
  PDmnd.TotWMaxTS.Time_secs = tSysTickTime.cnt_sec;
  PDmnd.TotWMaxTS.Time_nsec = 128000000;
  PDmnd.TotVarMax = 479.0;
  tSysTickTime.cnt_sec += 5;
  PDmnd.TotVarMaxTS.Time_secs = tSysTickTime.cnt_sec;
  PDmnd.TotVarMaxTS.Time_nsec = 129000000;
  PDmnd.TotVAMax = 480.0;
  tSysTickTime.cnt_sec += 5;
  PDmnd.TotVAMaxTS.Time_secs = tSysTickTime.cnt_sec;
  PDmnd.TotVAMaxTS.Time_nsec = 130000000;

  CurUnbal.Ia = 481.0/100;
  CurUnbal.Ib = 482.0/100;
  CurUnbal.Ic = 483.0/100;
  CurUnbal.In = 484.0/100;
  CurUnbalMax = 485.0/100;
  VolUnbal.Van = 486.0/100;
  VolUnbal.Vbn = 487.0/100;
  VolUnbal.Vcn = 488.0/100;
  VolUnbalMaxLN = 489.0/100;
  VolUnbal.Vab = 490.0/100;
  VolUnbal.Vbc = 491.0/100;
  VolUnbal.Vca = 492.0/100;
  VolUnbalMaxLL = 493.0/100;
  CurUnbalPh_max.Ia = 494.0/100;
  tSysTickTime.cnt_sec += 5;
  CurUnbalPh_max.IaTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalPh_max.IaTS.Time_nsec = 131000000;
  CurUnbalPh_max.Ib = 495.0/100;
  tSysTickTime.cnt_sec += 5;
  CurUnbalPh_max.IbTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalPh_max.IbTS.Time_nsec = 132000000;
  CurUnbalPh_max.Ic = 496.0/100;
  tSysTickTime.cnt_sec += 5;
  CurUnbalPh_max.IcTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalPh_max.IcTS.Time_nsec = 133000000;
  CurUnbalPh_max.In = 497.0/100;
  tSysTickTime.cnt_sec += 5;
  CurUnbalPh_max.InTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalPh_max.InTS.Time_nsec = 134000000;
  CurUnbalAllMax = 498.0/100;
  tSysTickTime.cnt_sec += 5;
  CurUnbalAllMaxTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalAllMaxTS.Time_nsec = 135000000;
  VolUnbalPh_max.Van = 499.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VanTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VanTS.Time_nsec = 136000000;
  VolUnbalPh_max.Vbn = 500.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VbnTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VbnTS.Time_nsec = 137000000;
  VolUnbalPh_max.Vcn = 501.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VcnTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VcnTS.Time_nsec = 138000000;
  VolUnbalAllMaxLN = 502.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalAllMaxLNTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalAllMaxLNTS.Time_nsec = 139000000;
  VolUnbalPh_max.Vab = 503.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VabTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VabTS.Time_nsec = 140000000;
  VolUnbalPh_max.Vbc = 504.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VbcTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VbcTS.Time_nsec = 141000000;
  VolUnbalPh_max.Vca = 505.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalPh_max.VcaTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalPh_max.VcaTS.Time_nsec = 142000000;
  VolUnbalAllMaxLL = 506.0/100;
  tSysTickTime.cnt_sec += 5;
  VolUnbalAllMaxLLTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalAllMaxLLTS.Time_nsec = 143000000;

  Res5min_Avg.Ia = 600.0/10;
  Res5min_Avg.Ib = 601.0/10;
  Res5min_Avg.Ic = 602.0/10;
  Res5min_Avg.Iavg = 603.0/10;
  Res5min_Avg.Pa = 604.0;
  Res5min_Avg.Pb = 605.0;
  Res5min_Avg.Pc = 606.0;
  Res5min_Avg.Ptot = 607.0;
  Res5min_Avg.AppPa = 608.0;
  Res5min_Avg.AppPb = 609.0;
  Res5min_Avg.AppPc = 610.0;
  Res5min_Avg.AppPtot = 611.0;
  Res5min_MinMax.Iamax = 612.0/10;
  Res5min_MinMax.Iamin = 613.0/10;
  Res5min_MinMax.Ibmax = 614.0/10;
  Res5min_MinMax.Ibmin = 615.0/10;
  Res5min_MinMax.Icmax = 616.0/10;
  Res5min_MinMax.Icmin = 617.0/10;
  Res5min_MinMax.Iavgmax = 618.0/10;
  Res5min_MinMax.Iavgmin = 619.0/10;
  Res5min_MinMax.Pamax = 620.0;
  Res5min_MinMax.Pamin = 621.0;
  Res5min_MinMax.Pbmax = 622.0;
  Res5min_MinMax.Pbmin = 623.0;
  Res5min_MinMax.Pcmax = 624.0;
  Res5min_MinMax.Pcmin = 625.0;
  Res5min_MinMax.Ptotmax = 626.0;
  Res5min_MinMax.Ptotmin = 627.0;
  Res5min_MinMax.AppPamax = 628.0;
  Res5min_MinMax.AppPamin = 629.0;
  Res5min_MinMax.AppPbmax = 630.0;
  Res5min_MinMax.AppPbmin = 631.0;
  Res5min_MinMax.AppPcmax = 632.0;
  Res5min_MinMax.AppPcmin = 633.0;
  Res5min_MinMax.AppPtotmax = 634.0;
  Res5min_MinMax.AppPtotmin = 635.0;

  TotVarHr = (1000 * 0x1234);
  NetWHr = (1000 * 0x9876);

// *** DAH generate internal time for a canned time of 12/24/2022  23:00:00 and add 10 seconds each time
  RTC_buf[0] = 0;  // hundredths of a second in hexadecimal (0 - 99)
  RTC_buf[1] = 0;  // seconds in hexadecimal (0 - 59)
  RTC_buf[2] = 0;  // minutes in hexadecimal (0 - 59)
  RTC_buf[3] = 23; // hours in hexadecimal (0 - 23)
  RTC_buf[4] = 0;  // day in hexadecimal - not used here
  RTC_buf[5] = 24; // date in hexadecimal (1 - 31)
  RTC_buf[6] = 12; // month in hexadecimal (1 - 12)
  RTC_buf[7] = 22; // year in hexadecimal (0 - 99)
  RTC_to_SysTickTime(&tSysTickTime);
// *** DAH

  THDminmax[0].THDmin = 650.0/100;
  THDminmax[0].THDmax = 651.0/100;
  THDminmax[0].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[0].THDminTS.Time_nsec = 700000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[0].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[0].THDmaxTS.Time_nsec = 701000000;
  THDminmax[1].THDmin = 652.0/100;
  THDminmax[1].THDmax = 653.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[1].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[1].THDminTS.Time_nsec = 702000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[1].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[1].THDmaxTS.Time_nsec = 703000000;
  THDminmax[2].THDmin = 654.0/100;
  THDminmax[2].THDmax = 655.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[2].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[2].THDminTS.Time_nsec = 704000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[2].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[2].THDmaxTS.Time_nsec = 705000000;
  THDminmax[3].THDmin = 656.0/100;
  THDminmax[3].THDmax = 657.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[3].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[3].THDminTS.Time_nsec = 706000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[3].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[3].THDmaxTS.Time_nsec = 707000000;
  THDminmax[7].THDmin = 658.0/100;
  THDminmax[7].THDmax = 659.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[7].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[7].THDminTS.Time_nsec = 708000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[7].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[7].THDmaxTS.Time_nsec = 709000000;
  THDminmax[8].THDmin = 660.0/100;
  THDminmax[8].THDmax = 661.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[8].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[8].THDminTS.Time_nsec = 710000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[8].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[8].THDmaxTS.Time_nsec = 711000000;
  THDminmax[9].THDmin = 662.0/100;
  THDminmax[9].THDmax = 663.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[9].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[9].THDminTS.Time_nsec = 712000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[9].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[9].THDmaxTS.Time_nsec = 713000000;
  THDminmax[4].THDmin = 664.0/100;
  THDminmax[4].THDmax = 665.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[4].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[4].THDminTS.Time_nsec = 714000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[4].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[4].THDmaxTS.Time_nsec = 715000000;
  THDminmax[5].THDmin = 666.0/100;
  THDminmax[5].THDmax = 667.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[5].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[5].THDminTS.Time_nsec = 716000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[5].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[5].THDmaxTS.Time_nsec = 717000000;
  THDminmax[6].THDmin = 668.0/100;
  THDminmax[6].THDmax = 669.0/100;
  tSysTickTime.cnt_sec += 10;
  THDminmax[6].THDminTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[6].THDminTS.Time_nsec = 718000000;
  tSysTickTime.cnt_sec += 10;
  THDminmax[6].THDmaxTS.Time_secs = tSysTickTime.cnt_sec;
  THDminmax[6].THDmaxTS.Time_nsec = 719000000;

  CurUnbalTotMax = 670.0/100;
  tSysTickTime.cnt_sec += 10;
  CurUnbalTotMaxTS.Time_secs = tSysTickTime.cnt_sec;
  CurUnbalTotMaxTS.Time_nsec = 720000000;
  VolUnbalTotMax = 671.0/100;
  tSysTickTime.cnt_sec += 10;
  VolUnbalTotMaxTS.Time_secs = tSysTickTime.cnt_sec;
  VolUnbalTotMaxTS.Time_nsec = 721000000;
  LD_BucketMax = 672.0;
  tSysTickTime.cnt_sec += 10;
  LD_BucketMaxTS.Time_secs = tSysTickTime.cnt_sec;
  LD_BucketMaxTS.Time_nsec = 722000000;
  LD_Bucket = 673.0;
  SD_BucketMax = 775.0;
  tSysTickTime.cnt_sec += 10;
  SD_BucketMaxTS.Time_secs = tSysTickTime.cnt_sec;
  SD_BucketMaxTS.Time_nsec = 723000000;

  StatusCode = 0x003E0208;              // Pri=2: Closed  Sec=8: Alarm  Cause=x0044: Phase Rotation
//  StatusCode = 0x003E0303;              // Pri=3: Tripped  Sec=3: Test Mode  Cause=x003E: Short Delay
                                // *** DAH  NEED TO ADD METERING FOR STATUS AND BINARY STATUS (?)
//  BinaryStatus = 0x1655;                // b12: GF is source ground
// BinaryStatus = 0x1676;                 // b11: unknown
                                        // b10: ZIN active
                                        // b9: long delay pickup
                                        // b6: test mode is not enabled
                                        // b5: test mode is active
                                        // b4: MM active
                                        // b2: alarm active
                                        // b1: trip condition active
                                        // b0: breaker closed

}
                      // *** DAH TEST  220207 ADDED FOR MODBUS DEBUGGING END
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Meter_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Meter Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Metering Module.
//                      The following variables do not need to be initialized in this subroutine:
//                        AFE_new_samples[] (initialized in AFE_Process_Samples())
//                        CF_PeakSav does not need to be initialized (it is set before it is used)
//                        FreqLoad.CapTim (not used until set in the ISR)
//                        FreqLine.CapTim (not used until set in the ISR)
//                        UserWF.CapTime.xx (not used until set in CaptureUserWaveform)
//                        UserWF.Type (not used until set in CaptureUserWaveform)
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            AFEcal.gain[], AFEcal.offset[], CurOneCycSOS_Sav.Ix, CurVol200msNoFltrSinSOS_Sav.Ix,
//                      CurVol200msNoFltrCosSOS_Sav.Ix, VolAFEOneCycSOS_Sav.Vxx, PwrOneCycSOS_Sav.Px,
//                      PwrOneCycSOS_Sav.RPx, Cur200msFltrSOS_Sav.Ix, Cur200msNoFltrSOS_Sav.Ix,
//                      VolAFE200msFltrSOS_Sav.Vxx, Pwr200msecSOS_Sav.Px, VolADC200msSOS_Sav.Vxx,
//                      Pwr200msecSOS_Sav.RPx, AFE_PrevSample[], ADC_PrevSample[][], CH_State, HarmReq,
//                      HarmFrozen, HarmSumCount, HarmonicsFil.xx[], HarmonicsSum.xx[], CF_Sum.Ix, CF.Ix,
//                      K_FactorReq, KF.State, KF_Val.Ix, SeqComp.xxx, Freqxxxx.Start, Freqxxxx.OvrTimer,
//                      Freqxxxx.DeltaTim, Freqxxxx.FreqVal, Freqxxxx.MinFreqVal, Freqxxxx.MaxFreqVal,
//                      PF.App[], PF.MinApp[], PF.MaxApp[], PF.Disp[], PF.MinDisp[], PF.MaxDisp{}, THD[],
//                      PhAngles1[], PhAngles2[], VolUnbalTot, CurUnbalTot, StatusCode, UserWF.Locked

//
//  ALTERS:             AFEcal.x
//
//  CALLS:              FRAM_Read(), ReadAFECalConstants(), ReadADCHCalConstants(),. ReadADCLCalConstants()
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 1.44msec worst case, FRAM errors on all of the
//                          constants at 16MHz
//                      Measured on 180628 (rev 0.26 code): 793usec typical (no FRAM errors) at 16MHz
//                      Measured on 181026 (rev 0.28 code): 286usec typical (no FRAM errors) at 120MHz
//
//------------------------------------------------------------------------------------------------------------

void Meter_VarInit(void)
{
  uint8_t i;

  CurOneCycSOS_Sav.Ia = 0;
  CurOneCycSOS_Sav.Ib = 0;
  CurOneCycSOS_Sav.Ic = 0;
  CurOneCycSOS_Sav.In = 0;
  CurOneCycSOS_Sav.Igsrc = 0;
  CurOneCycSOS_Sav.Igres = 0;
  for (i=0; i<7; ++i)
  {
    CurVol200msNoFltrSinSOS_Sav[i] = 0;
    CurVol200msNoFltrCosSOS_Sav[i] = 0;
  }
  VolAFEOneCycSOS_Sav.Van = 0;
  VolAFEOneCycSOS_Sav.Vbn = 0;
  VolAFEOneCycSOS_Sav.Vcn = 0;
  VolAFEOneCycSOS_Sav.Vab = 0;
  VolAFEOneCycSOS_Sav.Vbc = 0;
  VolAFEOneCycSOS_Sav.Vca = 0;
  VolADCOneCycSOS_Sav.Van = 0;
  VolADCOneCycSOS_Sav.Vbn = 0;
  VolADCOneCycSOS_Sav.Vcn = 0;
  VolADCOneCycSOS_Sav.Vab = 0;
  VolADCOneCycSOS_Sav.Vbc = 0;
  VolADCOneCycSOS_Sav.Vca = 0;
  PwrOneCycSOS_Sav.Pa = 0;
  PwrOneCycSOS_Sav.Pb = 0;
  PwrOneCycSOS_Sav.Pc = 0;
  PwrOneCycSOS_Sav.RPa = 0;
  PwrOneCycSOS_Sav.RPb = 0;
  PwrOneCycSOS_Sav.RPc = 0;
  Cur200msFltrSOS_Sav.Ia = 0;
  Cur200msFltrSOS_Sav.Ib = 0;
  Cur200msFltrSOS_Sav.Ic = 0;
  Cur200msFltrSOS_Sav.In = 0;
  Cur200msFltrSOS_Sav.Igsrc = 0;
  Cur200msFltrSOS_Sav.Igres = 0;
  Cur200msNoFltrSOS_Sav.Ia = 0;
  Cur200msNoFltrSOS_Sav.Ib = 0;
  Cur200msNoFltrSOS_Sav.Ic = 0;
  Cur200msNoFltrSOS_Sav.In = 0;
  VolAFE200msFltrSOS_Sav.Van = 0;
  VolAFE200msFltrSOS_Sav.Vbn = 0;
  VolAFE200msFltrSOS_Sav.Vcn = 0;
  VolAFE200msFltrSOS_Sav.Vab = 0;
  VolAFE200msFltrSOS_Sav.Vbc = 0;
  VolAFE200msFltrSOS_Sav.Vca = 0;
  VolAFE200msNoFltrSOS_Sav.Van = 0;
  VolAFE200msNoFltrSOS_Sav.Vbn = 0;
  VolAFE200msNoFltrSOS_Sav.Vcn = 0;
  VolAFE200msNoFltrSOS_Sav.Vab = 0;
  VolAFE200msNoFltrSOS_Sav.Vbc = 0;
  VolAFE200msNoFltrSOS_Sav.Vca = 0;
  VolADC200msSOS_Sav.Van = 0;
  VolADC200msSOS_Sav.Vbn = 0;
  VolADC200msSOS_Sav.Vcn = 0;
  VolADC200msSOS_Sav.Vab = 0;
  VolADC200msSOS_Sav.Vbc = 0;
  VolADC200msSOS_Sav.Vca = 0;
  Pwr200msecSOS_Sav.Pa = 0;
  Pwr200msecSOS_Sav.Pb = 0;
  Pwr200msecSOS_Sav.Pc = 0;
  Pwr200msecSOS_Sav.RPa = 0;
  Pwr200msecSOS_Sav.RPb = 0;
  Pwr200msecSOS_Sav.RPc = 0;
  CF_Sum.Ia = 0;
  CF_Sum.Ib = 0;
  CF_Sum.Ic = 0;
  CF_Sum.In = 0;
  CF.Ia = NAN;
  CF.Ib = NAN;
  CF.Ic = NAN;
  CF.In = NAN;
  KF_Val.Ia = NAN;
  KF_Val.Ib = NAN;
  KF_Val.Ic = NAN;
  SeqComp.I_ZeroMag = NAN;
  SeqComp.I_ZeroPh = NAN;
  SeqComp.I_PosMag = NAN;
  SeqComp.I_PosPh = NAN;
  SeqComp.I_NegMag = NAN;
  SeqComp.I_NegPh = NAN;
  SeqComp.V_ZeroMag = NAN;
  SeqComp.V_ZeroPh = NAN;
  SeqComp.V_PosMag = NAN;
  SeqComp.V_PosPh = NAN;
  SeqComp.V_NegMag = NAN;
  SeqComp.V_NegPh = NAN;
  FreqLoad.FreqVal = NAN;
  FreqLine.FreqVal = NAN;
  FreqLoad.MinFreqVal = NAN;
  FreqLine.MinFreqVal = NAN;
  FreqLoad.MaxFreqVal = NAN;
  FreqLine.MaxFreqVal = NAN;
  VolUnbalTot = NAN;
  CurUnbalTot = NAN;
  for (i=0; i<4; ++i)
  {
    PF.App[i] = NAN;
    PF.MinApp[i] = NAN;
    PF.MaxApp[i] = NAN;
    PF.Disp[i] = NAN;
    PF.MinDisp[i] = NAN;
    PF.MaxDisp[i] = NAN;
  }
  for (i=0; i<10; ++i)
  {
    THD[i] = NAN;                       // *** DAH MAYBE CHANGE THESE TO 0, OTHERWISE NEED TO CHECK ISNAN IN MODBUS CONVERSION SUBROUTINES
  }
  for (i=0; i<9; ++i)
  {
    PhAngles1[i] = NAN;
  }
  for (i=0; i<6; ++i)
  {
    PhAngles2[i] = NAN;
  }

  Max_Metering_Energy = 1.25 * 7000 * 346 * MS200_TO_HRS;        // *** DAH  CHECK THIS NUMBER - MAY BE FRAME DEPENDENT
  Min_Metering_Energy = 3.0 * 100 * MS200_TO_HRS;                // *** DAH  CHECK THIS NUMBER - MAY BE FRAME DEPENDENT

  for (i=0; i<4; ++i)
  {
    AFE_PrevSample[i] = 0.0;            // Initialize delayed samples for AFE currents
    Seed_State[i] = 0;                  // Initialize seeding state
    AFE_State[i] = 0;                   // Initialize AFE sample state for currents
    INT_a1[i] = NORMAL_a1;              // Initialize integrator constant
  }
  INT_a1[4] = NORMAL_a1;                // Initialize integrator constant for Rogowski ground sensing
  ss[4] = 0;                            // Initialize integrator initial condition for Rogowski ground

  for (i=0; i<8; ++i)
  {
    MTR_ss[i] = 0;                      // Initialize filter state to 0
    ADC_PrevSample[i][0] = 0;           // Initialize delayed samples for ADC currents
    ADC_PrevSample[i][1] = 0;
  }

  SystemFlags = 0;                      // Clear all system flags
  ReadAFECalConstants();                // Call subroutines to retrieve the AFE and ADC cal constants
  ReadADCHCalConstants();               //   These subroutines will set the appropriate error flag if there
  ReadADCLCalConstants();               //   is an error

  if (SystemFlags & (CAL_FRAM_ERR + CAL_FLASH_ERR))
  {
    // *** DAH SET ERROR FLAG SO WE GENERATE AN ALARM LOG AND WE LIGHT THE RED LED (TALKED TO EVERYBODY ON 151108) - DON'T CORRECT
  }

  // Read the FRAME calibration constants, checksum, and checksum complement from FRAM - store these ??
  //   *** DAH NEED TO ADD CODE TO PROCESS THE FRAME CAL CONSTANTS AND COMBINE THEM WITH THE OTHER CAL CONSTANTS


  UseADCvals = FALSE;
  TestWFind = 0;
  Getting_AFE_Seed = TRUE;

  CH_State = 0;
  HarmReq = FALSE;
  HarmFrozen = FALSE;
  HarmSumCount = 0;                 // Measured execution time to initialize these variables on 190501
  for (i=0; i<39; ++i)              //   (rev 0.31 code): 10usec so not really worth placing this in
  {                                 //   the Calc_Harmonics() subroutine
    HarmonicsFil.Ia[i] = 0;
    HarmonicsFil.Ib[i] = 0;
    HarmonicsFil.Ic[i] = 0;
    HarmonicsFil.In[i] = 0;
    HarmonicsFil.Vab[i] = 0;
    HarmonicsFil.Vbc[i] = 0;
    HarmonicsFil.Vca[i] = 0;
    HarmonicsFil.Van[i] = 0;
    HarmonicsFil.Vbn[i] = 0;
    HarmonicsFil.Vcn[i] = 0;
    HarmonicsSum.Ia[i] = 0;
    HarmonicsSum.Ib[i] = 0;
    HarmonicsSum.Ic[i] = 0;
    HarmonicsSum.In[i] = 0;
    HarmonicsSum.Vab[i] = 0;
    HarmonicsSum.Vbc[i] = 0;
    HarmonicsSum.Vca[i] = 0;
    HarmonicsSum.Van[i] = 0;
    HarmonicsSum.Vbn[i] = 0;
    HarmonicsSum.Vcn[i] = 0;
  }
  KF.State = 0;
  K_FactorReq = FALSE;

  FreqLoad.Start = TRUE;                // Set first-time flag
  FreqLine.Start = TRUE;
  FreqLoad.OvrTimer = 0;                // Initialize overrun timer
  FreqLine.OvrTimer = 0;
  FreqLoad.DeltaTim = 0xFFFF;           // Set delta time to invalid
  FreqLine.DeltaTim = 0xFFFF;

  StatusCode = 0;
  TU_BinStatus.all = 0;

  ResetMinMaxFlags = 0;
  temp_i = 0;

  UserWF.Locked = FALSE;

  TP_AFEIntOff = FALSE;                 // *** DAH TEST DO WE WANT TO KEEP THIS FEATURE?
  
  TAVoltValid = FALSE;
  AuxVoltValid = FALSE;
  passed_1st_detection = FALSE;
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Meter_VarInit()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       AFE_Process_Samples()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process AFE Samples
//
//  MECHANICS:          This subroutine processes the samples from the DMA operation used to read the AFE.
//                      The AFE readings (one per channel, eight total) are received as two 16-bit words.
//                      Each reading is checked for validity (errors in the header and data overrange).  If
//                      the AFE reading is valid, the 24-bit AFE sample data is extracted from the reading
//                      and stored in floating point format in AFE_new_samples[].  If the AFE readings are
//                      invalid, the internal ADC data is stored in AFE_new_samples[].
//                        AFE_new_samples[0] - Ia: Rogowski input, integrated
//                        AFE_new_samples[1] - Ib: Rogowski input, integrated
//                        AFE_new_samples[2] - Ic: Rogowski input, integrated
//                        AFE_new_samples[3] - In: if CT input, not integrated
//                                                 if Rogowski input, integrated
//                        AFE_new_samples[4] - Igsrc: if CT input, not integrated
//                                                    if Rogowski input, integrated
//                        AFE_new_samples[5] - Va: voltage input, not integrated
//                        AFE_new_samples[6] - Vb: voltage input, not integrated
//                        AFE_new_samples[7] - Vc: voltage input, not integrated
//
//  CAVEATS:            None
//
//  INPUTS:             AFE_single_capture[] - the present set of values from the AFE
// 
//  OUTPUTS:            AFE_new_samples[] - the new sample values in floating point format
//
//  ALTERS:             None
// 
//  CALLS:              AFE_Integrate_Sample(), AFE_Cal_Scale(), Meter_Filter_Sample()
// 
//------------------------------------------------------------------------------------------------------------

void AFE_Process_Samples(void)
{
  uint32_t utemp32;
  int32_t stemp32;
  uint8_t i, j;
  float ftemp;
  float *HCSOS_ptr;

  // Channel values are double-word (32 bits) long.  AFE_single_capture[0, 2, ..., 14] are the
  //   most-significant words of the 32-bit value.  AFE_single_capture[1, 3, ..., 15] are the
  //   least-significant words of the 32-bit values.
  // The data format is as follows:
  //   b31 (AFE_single_capture[0, 2, 4, 6] b15):
  //       CHIP_ERROR - 1 if serious error in the AFE and a reset is required
  //                    0 if ok
  //   b30..28 (AFE_single_capture[0, 2, 4, 6] b14..12):
  //       CH ADDR - Channel address (0..7)
  //   b27 (AFE_single_capture[0, 2, 4, 6] b11):
  //       RESET_DETECTED - 1 if reset condition has occurred
  //                        0 otherwise
  //   b26 (AFE_single_capture[0, 2, 4, 6] b10):
  //       MODULATOR_SATURATED - 1 if the modulator has outputted 20 consecutive 0's or 1's
  //                             0 otherwise
  //   b25 (AFE_single_capture[0, 2, 4, 6] b9):
  //       FILTER_SATURATED - 1 if the filter output is out of bounds
  //                          0 otherwise
  //   b24 (AFE_single_capture[0, 2, 4, 6] b8):
  //       AIN_OV_ERROR - 1 if there has been an AINx over- or under-voltage condition on the input
  //   b23..b0 (AFE_single_capture[0, 2, 4, 6] b7..0, (AFE_single_capture[1, 3, 5, 7] b15..0)):
  //       Data reading

  // The eight AFE channels, from 0 thru 7, are: Ia, Ib, Ic, In, Igsrc, Va, Vb, Vc.  Channels 0 thru 2 come
  //   from Rogowski coils and must be integrated.

  // ************************* First process the Rogowski current channels, 0 - 2 **************************
  // Initialize temporary  pointer to the half-cyc sum of squares current value.  This is used to determine
  //   whether it is ok to go back to using the AFE samples, if we have switched to the ADC samples.
  HCSOS_ptr = &CurHalfCycSOS_SumF.Ia;
  for (i=0, j=0; j<3; i+=2, j++)
  {
    // Check for a chip error.  If the error bit is set, blink the error LED and set the sample magnitude
    //   (held in utemp32) to an overrange value.  This will force the state machine to switch to the ADC.
    //   We will reset the AFE anyway, and sampling will stop, but at least this sample will be handled ok
    if ( (AFE_single_capture[i] & 0x8000) == 0x8000)  // *** DAH  1) WE MAY WANT TO ENTER AN EVENT HERE
    {                                                 //          2) IS THIS HOW WE WANT TO HANDLE THE STATUS LED?
      TP_LEDCode = 2;                                 //          3) I THINK WE SHOULD HANDLE THIS AS A BROKEN CHIP AND WE SHOULD RESET THE AFE
      utemp32 = 0x7FFFFE;                             //          4) WE NEED TO TALK TO ADI TO SEE WHETHER THERE IS A LIKELIHOOD OF AN ERROR
      AFE_Error = TRUE;                               //          5) SHOULD WE HAVE AN ERROR COUNTER?  WE NEED A WAY TO ENSURE WE
    }                                                 //             KEEP USING THE ADC AND DON'T SWITCH BACK TO THE AFE
    // If no chip error, store the sample as a floating point.  Store the data value in stemp32 (not the
    //   header info) as a signed integer.  Set utemp32 to the magnitude of the value
    else
    {
      utemp32 = (((uint32_t)(AFE_single_capture[i] & 0x00FF)) << 16) + AFE_single_capture[i+1];
      if (utemp32 < 0x00800000)                 // Value is positive...
      {
        stemp32 = utemp32;
      }
      else                                      // Value is negative...
      {
        utemp32 |= 0xFF000000;                      // Sign extend the value
        stemp32 = (int32_t)utemp32;                 // Store the signed number in stemp
        utemp32 = (~(int32_t)utemp32) + 1;          // Set utemp to the magnitude
      }
    }
    // Enter the sample state machine.  Note, each channel has its own state.
    //   AFE State 0: AFE Operation
    //      - Check the AFE value for errors and range.
    //      - If there are no errors and the sample value is in range, use the AFE value.  The integration
    //        subroutine is called.  Note, seeding may occur in this state after a reset.
    //      - If the AFE value is invalid, fall into AFE State 1.
    //   AFE State 1: ADC Operation
    //      - Check the RMS current value and the number of consecutive valid AFE samples to determine
    //        whether we can switch back to AFE operation.
    //      - If we can switch back to AFE operation, set Seed_State[] to 1 and call AFE_Integrate_Sample()
    //        to reseed the integrator with the low-gain seed value.  Seed_State[] will then be set back to
    //        0.  The (low-gain) ADC value is still used for this sample.  Set AFE_State[] to 0 for when the
    //        next sample is processed.
    //      - If we cannot switch back to AFE operation, just use the (low-gain) ADC value
    //      - Note, because the multiplexer high-gain or low-gain setting applies to all of the inputs, it
    //        is simplest to just use the low-gain setting for reseeding, as opposed to also getting
    //        high-gain readings.  If we switched to the high-gain setting, we run the risk of the samples
    //        being clamped on both this channel and the other channels, and this sample is still used for
    //        metering and protection.  Using the low-gain samples for reseeding should be ok.  Since the
    //        transition is from the low-gain ADC value to the AFE, presumably we are going from a higher
    //        current to a lower (safer) current, and so the accuracy loss of using the lower-precision
    //        low-gain sample for seeding will not matter.
    switch (AFE_State[j])
    {
      case 0:                                       // State 0: AFE Operation
      default:
        if (utemp32 < 0x7FFF80)                         // If value is in range, use the AFE value and stay
        {                                               //   in this state
          if (!UseADCvals)                                // If test flag is False use AFE value and stay in
          {                                               //   this state *** DAH PUT TIMER ON THIS FLAG SO IT IS CANNOT BE ON INDEFINITELY
            AFE_new_samples[j] = (float)(stemp32);           // Store it as a floating point number
            // *** DAH TEST STORE THE RAW AND SCALED SAMPLES BEFORE INTEGRATION
            //   SPECIAL TEST CODE USES EAW2 TO INITIATE A 10-CYCLE CAPTURE INTO TESTSAMPLES USING SD_PROTON.  CODE SHOULD BE DELETED EVENTUALLY
//            if ((j == 0) && SD_ProtOn)        // *** DAH 190925 added
//            {
//              TestSamples[0][TestWFind] = (AFE_new_samples[0] - AFEcal.offset[0]) * AFEcal.gain[0];    // *** DAH 190925 replaced with directly below
//              TestSamples[0][TestWFind] = AFE_new_samples[2];                                        // *** DAH 190925 added   raw sample
//              TestSamples[1][TestWFind] = (AFE_new_samples[2] - AFEcal.offset[2]) * AFEcal.gain[2];  // *** DAH 190925 added   scaled before integration
//            }
            // *** DAH TEST END OF TEST CODE
//            AFE_Integrate_Sample(j);                         // Integrate the sample to convert the di/dt
//            ftemp = AFE_PrevSample[j];                       //   value to current.  Note, the output of the
//            AFE_PrevSample[j] = AFE_new_samples[j];          //   integrator leads the voltage by one
//            AFE_new_samples[j] = ftemp;                      //   sample, so use the previous sample

            // *** DAH TEST SHOULD THIS "IF" BE ELIMINATED EVENTUALLY USED TO BYPASS INTEGRATION IF DESIRED
            if (TP_AFEIntOff == FALSE)                         
            {
              AFE_Integrate_Sample(j);
              ftemp = AFE_PrevSample[j];
              AFE_PrevSample[j] = AFE_new_samples[j];
              AFE_new_samples[j] = ftemp;
            }
            // *** DAH TEST END OF TEST CODE
            AFE_Cal_Scale(j);                                // Convert to engineering units
          }
          else                                            // If test flag is True, use ADC value and stay in
          {                                               //   this state
            AFE_new_samples[j] = ADC_samples[j];
          }
          break;
        }
        else                                            // Value is out of range
        {                                                 // Set state to 1, initialize the number of
          AFE_State[j] = 1;                               //   consecutive good samples, and fall into
          SampleCntOK[j] = 0;                             //   State 1
//          break;
        }

      case 1:                                       // State 1: ADC Operation
        if (utemp32 < 0x7FFF80)                         // If value is in range, increment the number of
        {                                               //   conecutive good samples
          if (SampleCntOK[j] < 255)
          {
            SampleCntOK[j]++;
          }
        }
        else                                            // Otherwise reset the number of consecutive good
        {                                               //   samples
          SampleCntOK[j] = 0;
        }
        if ( (SampleCntOK[j] > 89)                      // If we are ok to switch back to the AFE, reseed       *** DAH  THE HALF-CYCLE RMS SOS VALUE IS SET FOR 75000A RIGHT NOW
          && (*HCSOS_ptr < 2.25E9) )                    //   the integrator using the existing (low-gain)                THIS IS ABOUT 75% OF THE MAXIMUM VALUE FOR THE AFE
        {                                               //   ADC sample.  This sample will also be used in               THIS MUST BE CHANGED TO A CONSTANT BASED ON THE FRAME SIZE
          Seed_State[j] = 1;                            //   protection and metering processing
          AFE_Integrate_Sample(j);
          AFE_State[j] = 0;                             //   Set state back to 0
        }
        AFE_new_samples[j] = ADC_samples[j];            // Use the ADC samples for metering and protection
        break;
    }

    Meter_Filter_Sample(j);                         // Filter the sample for metered values
    ++HCSOS_ptr;                                    // Increment the half-cyc pointer
  }
  // ************************* End of Rogowski current channels, 0 - 2 *************************************

  
  // ****************************** Now process the voltage channels, 5 - 7 ********************************
  for (i=10, j=5; j<8; i+=2, j++)
  {
    // Check for a chip error.  If the error bit is set, blink the error LED and set the sample magnitude     
    //   (held in utemp32) to the maximum value.
    if ( (AFE_single_capture[i] & 0x8000) == 0x8000)  // *** DAH  1) WE MAY WANT TO ENTER AN EVENT HERE
    {                                                 //          2) IS THIS HOW WE WANT TO HANDLE THE STATUS LED?
      TP_LEDCode = 2;                                 //          3) I THINK WE SHOULD HANDLE THIS AS A BROKEN CHIP AND WE SHOULD RESET THE AFE
      utemp32 = 0x7FFFFE;                             //          4) WE NEED TO TALK TO ADI TO SEE WHETHER THERE IS A LIKELIHOOD OF AN ERROR
      AFE_Error = TRUE;                               //          5) SHOULD WE HAVE AN ERROR COUNTER?  WE NEED A WAY TO ENSURE WE
    }                                                 //             KEEP USING THE ADC AND DON'T SWITCH BACK TO THE AFE
    // If no chip error, assemble the value as an unsigned integer
    else
    {
      utemp32 = (((uint32_t)(AFE_single_capture[i] & 0x00FF)) << 16) + AFE_single_capture[i+1];
    }
    // Store the sample as a floating point.  Store the data value in stemp32 (not the header info) as a
    //   signed integer.  Set utemp32 to the magnitude of the value
    if (utemp32 < 0x00800000)                 // Value is positive...
    {
      stemp32 = utemp32;
    }
    else                                      // Value is negative...
    {
      utemp32 |= 0xFF000000;                      // Sign extend the value
      stemp32 = (int32_t)utemp32;                 // Store the signed number in stemp
      utemp32 = (~(int32_t)utemp32) + 1;          // Set utemp to the magnitude
    }
    if (!UseADCvals)                            // If test flag is False use AFE value
    {
      AFE_new_samples[j] = (float)(stemp32);        // Store it as a floating point number
      AFE_Cal_Scale(j);                             // Convert to engineering units
    }
    else                                        // If test flag is True, use ADC value
    {
      AFE_new_samples[j] = ADC_samples[j];
    }
    Meter_Filter_Sample(j);                     // Filter the sample for metered values
  }
  // ****************************** End of voltage channels, 5 - 7 *****************************************

  
  // ****************************** Now process the ground current channel *********************************
  // Check for a chip error.  If the error bit is set, blink the error LED and set the sample magnitude
  //   (held in utemp32) to the maximum value.
  if ( (AFE_single_capture[8] & 0x8000) == 0x8000)  // *** DAH  1) WE MAY WANT TO ENTER AN EVENT HERE
  {                                                 //          2) IS THIS HOW WE WANT TO HANDLE THE STATUS LED?
    TP_LEDCode = 2;                                 //          3) I THINK WE SHOULD HANDLE THIS AS A BROKEN CHIP AND WE SHOULD RESET THE AFE
    utemp32 = 0x7FFFFE;                             //          4) WE NEED TO TALK TO ADI TO SEE WHETHER THERE IS A LIKELIHOOD OF AN ERROR
    AFE_Error = TRUE;                               //          5) SHOULD WE HAVE AN ERROR COUNTER?  WE NEED A WAY TO ENSURE WE
  }                                                 //             KEEP USING THE ADC AND DON'T SWITCH BACK TO THE AFE
  // If no chip error, assemble the value as an unsigned integer
  else
  {
    utemp32 = (((uint32_t)(AFE_single_capture[8] & 0x00FF)) << 16) + AFE_single_capture[9];
  }
  // Store the sample as a floating point.  Store the data value in stemp32 (not the header info) as a
  //   signed integer.  Set utemp32 to the magnitude of the value
  if (utemp32 < 0x00800000)                 // Value is positive...
  {
    stemp32 = utemp32;
  }
  else                                      // Value is negative...
  {
    utemp32 |= 0xFF000000;                      // Sign extend the value
    stemp32 = (int32_t)utemp32;                 // Store the signed number in stemp
    utemp32 = (~(int32_t)utemp32) + 1;          // Set utemp to the magnitude
  }
  AFE_new_samples[4] = (float)(stemp32);        // Store it as a floating point number

  if (GF_USING_ROGOWSKI)                    // If Igsrc is being measured with a Rogowski coil, process it
  {                                         //   like the other currents
    if (TP_AFEIntOff == FALSE)                         
    {
      AFE_Integrate_Sample(4);                    // Integrate the sample to convert the di/dt
      ftemp = AFE_PrevSample[4];                  //   value to current.  Note, the output of the
      AFE_PrevSample[4] = AFE_new_samples[4];     //   integrator leads the voltage by one
      AFE_new_samples[4] = ftemp;                 //   sample, so use the previous sample
    }
    // Convert to engineering units - use Rogowski cal constants (AFEcal index = 8)
    AFE_new_samples[4] = (AFE_new_samples[4] - AFEcal.offset[8]) * AFEcal.gain[8];
    Meter_Filter_Sample(4);                     // Filter the sample for metered values
  }
  else                                      // Otherwise Igsrc is being measured with a CT, so process it
  {                                         //   like the voltages
    // Convert to engineering units - use CT cal constants (AFEcal index = 4)
    AFE_Cal_Scale(4);
    Meter_Filter_Sample(4);                     // Filter the sample for metered values
  }
  // ******************************* End of ground current channel *****************************************


  // ************************* Finally, process the neutral current, channel 3 *****************************
  // Initialize temporary  pointer to the half-cyc sum of squares current value.  This is used to determine
  //   whether it is ok to go back to using the AFE samples, if we have switched to the ADC samples.
  HCSOS_ptr = &CurHalfCycSOS_SumF.In;
  // Check for a chip error.  If the error bit is set, blink the error LED and set the sample magnitude
  //   (held in utemp32) to an overrange value.  This will force the state machine to switch to the ADC.
  //   We will reset the AFE anyway, and sampling will stop, but at least this sample will be handled ok
  if ( (AFE_single_capture[6] & 0x8000) == 0x8000)  // *** DAH  1) WE MAY WANT TO ENTER AN EVENT HERE
  {                                                 //          2) IS THIS HOW WE WANT TO HANDLE THE STATUS LED?
    TP_LEDCode = 2;                                 //          3) I THINK WE SHOULD HANDLE THIS AS A BROKEN CHIP AND WE SHOULD RESET THE AFE
    utemp32 = 0x7FFFFE;                             //          4) WE NEED TO TALK TO ADI TO SEE WHETHER THERE IS A LIKELIHOOD OF AN ERROR
    AFE_Error = TRUE;                               //          5) SHOULD WE HAVE AN ERROR COUNTER?  WE NEED A WAY TO ENSURE WE
  }                                                 //             KEEP USING THE ADC AND DON'T SWITCH BACK TO THE AFE
  // If no chip error, store the sample as a floating point.  Store the data value in stemp32 (not the
  //   header info) as a signed integer.  Set utemp32 to the magnitude of the value
  else
  {
    utemp32 = (((uint32_t)(AFE_single_capture[6] & 0x00FF)) << 16) + AFE_single_capture[7];
    if (utemp32 < 0x00800000)                 // Value is positive...
    {
      stemp32 = utemp32;
    }
    else                                      // Value is negative...
    {
      utemp32 |= 0xFF000000;                      // Sign extend the value
      stemp32 = (int32_t)utemp32;                 // Store the signed number in stemp
      utemp32 = (~(int32_t)utemp32) + 1;          // Set utemp to the magnitude
    }
  }
  // Enter the sample state machine.  Note, each channel has its own state.
  //   AFE State 0: AFE Operation
  //      - Check the AFE value for errors and range.
  //      - If there are no errors and the sample value is in range, use the AFE value.  The integration
  //        subroutine is called.  Note, seeding may occur in this state after a reset.
  //      - If the AFE value is invalid, fall into AFE State 1.
  //   AFE State 1: ADC Operation
  //      - Check the RMS current value and the number of consecutive valid AFE samples to determine whether
  //        we can switch back to AFE operation.
  //      - If we can switch back to AFE operation, set Seed_State[] to 1 and call AFE_Integrate_Sample() to
  //        reseed the integrator with the low-gain seed value.  Seed_State[] will then be set back to 0.
  //        The (low-gain) ADC value is still used for this sample.  Set AFE_State[] to 0 for when the next
  //        sample is processed.
  //      - If we cannot switch back to AFE operation, just use the (low-gain) ADC value
  //      - Note, because the multiplexer high-gain or low-gain setting applies to all of the inputs, it is
  //        simplest to just use the low-gain setting for reseeding, as opposed to also getting high-gain
  //        readings.  If we switched to the high-gain setting, we run the risk of the samples being clamped
  //        on both this channel and the other channels, and this sample is still used for metering and
  //        protection.  Using the low-gain samples for reseeding should be ok.  Since the transition is
  //        from the low-gain ADC value to the AFE, presumably we are going from a higher current to a lower
  //        (safer) current, and so the accuracy loss of using the lower-precision low-gain sample for
  //        seeding will not matter.
  switch (AFE_State[3])
  {
    case 0:                                       // State 0: AFE Operation
    default:
      if (utemp32 < 0x7FFF80)                         // If value is in range, use the AFE value and stay in
      {                                               //   this state
        if (!UseADCvals)                                // If test flag is False use AFE value and stay in
        {                                               //   this state *** DAH PUT TIMER ON THIS FLAG SO IT IS CANNOT BE ON INDEFINITELY
          AFE_new_samples[3] = (float)(stemp32);           // Store it as a floating point number
          if (IN_USING_ROGOWSKI)                           // If In is being measured with a Rogowski
          {                                                //   coil, process it like the other currents
            if (TP_AFEIntOff == FALSE)              // *** DAH TEST SHOULD THE "IF CLAUSE" USED TO BYPASS INTEGRATION IF DESIRED           
            {                                       //              BE ELIMINATED EVENTUALLY?
              AFE_Integrate_Sample(3);                        // Integrate the sample to convert the di/dt
              ftemp = AFE_PrevSample[3];                      //   value to current.  Note, delay the
              AFE_PrevSample[3] = AFE_new_samples[3];          //   samples so they are aligned with the      
              AFE_new_samples[3] = ftemp;                      //   other current samples
            }
            // *** DAH TEST END OF IF CLAUSE
            // Convert to engineering units - use Rogowski cal constants (AFEcal index = 3)
            AFE_Cal_Scale(3);
          }
          else                                             // Otherwise In is being measured with a CT, so
          {                                                //   process it like the voltages
            // Convert to engineering units - use CT cal constants (AFEcal index = 9)
            AFE_new_samples[3] = (AFE_new_samples[3] - AFEcal.offset[9]) * AFEcal.gain[9];
          }
        }
        else                                            // If test flag is True, use ADC value and stay in
        {                                               //   this state
          AFE_new_samples[3] = ADC_samples[3];
        }
        break;
      }
      else                                            // Value is out of range
      {                                                 // Set state to 1, initialize the number of
        AFE_State[3] = 1;                               //   consecutive good samples, and fall into State 1
        SampleCntOK[3] = 0;
//        break;
      }

    case 1:                                       // State 1: ADC Operation
      if (utemp32 < 0x7FFF80)                         // If value is in range, increment the number of
      {                                               //   conecutive good samples
        if (SampleCntOK[j] < 255)
        {
          SampleCntOK[j]++;
        }
      }
      else                                            // Otherwise reset the number of consecutive good
      {                                               //   samples
        SampleCntOK[j] = 0;
      }
      if ( (SampleCntOK[j] > 89)                      // If we are ok to switch back to the AFE, reseed the
        && (CurHalfCycSOS_SumF.In < 35000) )          //   integrator using the existing (low-gain) ADC      *** DAH  THIS CURRENT SHOULD PROBABLY BE LOWER AND MAYBE SHOULD BE BASED ON THE FRAME SIZE?
      {                                               //   sample.  This sample will also be used in
        Seed_State[3] = 1;                            //   protection and metering processing
        AFE_Integrate_Sample(3);
        AFE_State[3] = 0;                             //   Set state back to 0
      }
      AFE_new_samples[3] = ADC_samples[3];            // Use the ADC samples for metering and protection
      break;
  }

  Meter_Filter_Sample(3);                         // Filter the sample for metered values

  // **************************** End of neutral current channel, 3 ****************************************


//  if (SD_ProtOn)                                    // *** DAH TEST STORE THE INTEGRATED AND SCALED SAMPLE
//  {
//    TestSamples[1][TestWFind] = AFE_new_samples[0];       // *** DAH 190925 replaced with scaled before integration (see above)
//    TestSamples[2][TestWFind] = AFE_new_samples[2];     // *** DAH 190925 added    scaled after integration
//    TestSamples[2][TestWFind] = ADC_samples[0];           // *** DAH 190925 replaced with directly above
//    TestSamples[3][TestWFind++] = MTR_new_samples[0];     // *** DAH 190925 replaced with directly below
//    TestSamples[3][TestWFind++] = MTR_new_samples[2];   // *** DAH 190925 aded     filtered
//    TestSamples[3][TestWFind] = AFE_new_samples[6];
//    TestSamples[2][TestWFind++] = AFE_new_samples[5];
//    if (TestWFind >= 800)
//    {
 //     TestWFind = 0;
//      SD_ProtOn = FALSE;                // *** DAH ADDED 160603      
//      SD_ProtOn = FALSE;                // *** DAH 190925 added
//    }
//  }                                                 // *** DAH TEST END OF TEST CODE

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         AFE_Process_Samples()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       AFE_Integrate_Sample()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Integrate AFE Sample
//
//  MECHANICS:          This subroutine performs the digital integration of the AFE sample to convert it
//                      from a voltage (the output of the Rogowski coil is a voltage proportional to di/dt)
//                      to a current.  It is implemented as a low-pass filter, so it will only climb to the
//                      input value.
//                      George Gao designed the filter algorithm.  It is the transposed direct form II
//                      algorithm.
//                      The gain of the filter is 1/b0 at 60Hz.  This is why the default current gain is
//                      scaled by SCALE_FACTOR_b0.
//
//                      Measuring the ground current with a Rogowski coil was a later addition.  It is
//                      somewhat of a special case.  In this case, index is four and all of the values used
//                      in the integration computation are indexed at 4, EXCEPT for the calibration
//                      constants, which are at index = 8: AFEcal.gain[8] and AFEcal.offset[8] apply to the
//                      Rogowski coil.  (AFEcal.gain[4] and AFEcal.offset[4] apply to the CT ground
//                      measurement)
//
//  CAVEATS:            None
//
//  INPUTS:             index - index of the sample to be integrated
//                      AFE_new_samples[] - array of AFE sample values in floating point format
//                      ADC_samples[] - array of ADC sample values in floating point format to be used as
//                          integrator seed
//                      Seed_State[] - array of values used to determine when to perform the integrator
//                          seeding.  If the initial value is 2, the low-gain ADC value must be used as the
//                          seed.  That value is on the next sample.  If the initial value is 1, the
//                          high-gain ADC value must be used, and so seeding is done.  For any other value,
//                          no seeding is performed.
//                      Note, the high-gain ADC value is read first, and Seed-State is set to 1.  One sample
//                      later, the low-gain value is read.  If it is used (because the high-gain value is
//                      invalid), Seed_State is set to 2.  In this subroutine, if Seed_State is 1, seeding
//                      is performed.  If it was 1 to begin with, the the high-gain seed is used.  When
//                      Seed_State is 2, no seeding is done, and Seed_State is set to 1.  The next time the
//                      subroutine is entered, it will seed with the next ADC sample, which is the low-gain
//                      seed.  After seeding is done, Seed_State is set to 0, and no further seeding is
//                      done.
// 
//  OUTPUTS:            AFE_new_samples[] - the integrated sample values in floating point format
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void AFE_Integrate_Sample(uint8_t index)
{
  float y;

  // Filter coefficients
  //  const float b0 = 1;
  //  const float a1 = 9.9691733373313E-1;

  // If not the ground current, sample processing must include checking for seeding the integrator
  if (index != 4)
  {
    if (Seed_State[index] == 2)                       // If Seed_State == 2, seed is from low-gain ADC, wait
    {                                                 //   until the next sample to do the seeding so AFE and
      Seed_State[index] = 1;                          //   ADC samples are aligned.  Set state to 1 so seeding
    }                                                 //   is done on next sample
    else if (Seed_State[index] == 1)                  // If Seed_State == 1, seed the integrator, then
    {                                                 //   set the state to 0
      // Remove offset and gain calibration before placing in the integrator
      ss[index] = ADC_samples[index]/AFEcal.gain[index] + AFEcal.offset[index];  // *** DAH MAYBE SHOULD SAVE 1/GAIN AS SEPARATE CONSTANT SO NO DIVIDE OPERATION
      Seed_State[index] = 0;    
      if (index == 0)                          // *** DAH TEST
      {
        seed_val = ADC_samples[0];
      }
    }
    else                                              // Otherwise do normal integration
    {
  //  y = b0 * AFE_new_samples[index] + ss[index];    (b0 = 1)
      y = AFE_new_samples[index] + ss[index];                                 // *** DAH ADD RANGE CHECK BECAUSE IF GET NaN, WILL STAY FOREVER!
      ss[index] = INT_a1[index] * y;
      AFE_new_samples[index] = y;
    }
  }
  // If this is the ground current (index = 4), always do normal integration.  Note, the cal constants are
  //   at index = 8
  else
  {
//  y = b0 * AFE_new_samples[index] + ss[index];    (b0 = 1)
    y = AFE_new_samples[index] + ss[index];                                 // *** DAH ADD RANGE CHECK BECAUSE IF GET NaN, WILL STAY FOREVER!
    ss[index] = INT_a1[index] * y;
    AFE_new_samples[index] = y;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         AFE_Integrate_Sample()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Meter_Filter_Sample()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Filter the Sample
//
//  MECHANICS:          This subroutine performs the filtering (DC removal) of the sample.
//                      George Gao designed the filter algorithm.  It is the transposed direct form II
//                      algorithm.
//                      The gain of the filter is 1 at 60Hz.
//
//  CAVEATS:            None
//
//  INPUTS:             index - index in AFE_new_samples[] of the sample to be integrated
//                      AFE_new_samples[] - array of sample values in floating point format
// 
//  OUTPUTS:            MTR_new_samples[] - the filtered sample values in floating point format
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void Meter_Filter_Sample(uint8_t index)
{
  float y;

  // Filter coefficients
  //  const float b0 = 9.9928408556729E-1;
  //  const float a1 = 9.9609375000000E-1;

  y = MTR_b0 * AFE_new_samples[index] + MTR_ss[index];                      // *** DAH ADD RANGE CHECK BECAUSE IF GET NaN, WILL STAY FOREVER!
  MTR_ss[index] = (MTR_a1 * y) - (MTR_b0 * AFE_new_samples[index]);
  MTR_new_samples[index] = y;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Meter_Filter_Sample()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       AFE_Cal_Scale()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Scale AFE Sample
//
//  MECHANICS:          This subroutine performs the scaling of the AFE sample to convert it from an
//                      integrated floating-point number to a current in amps or volts.  For now, the gain
//                      and offset scaling coefficients are based on calculated (ideal) values.  Eventually,
//                      they will be comprised of the Frame's calibration constants and the board's
//                      calibration constants.
//
//  CAVEATS:            None
//
//  INPUTS:             index - index in AFE_new_samples[] of the sample to be scaled (channel number)
//                      AFEcal.gain[] - the gain scaling coefficient for the channel
//                      AFEcal.offset[] - the offset scaling coefficient for the channel
// 
//  OUTPUTS:            AFE_new_samples[] - the scaled sample values in floating point format (engineering
//                                          units)
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void AFE_Cal_Scale(uint8_t index)
{
  AFE_new_samples[index] = (AFE_new_samples[index] - AFEcal.offset[index]) * AFEcal.gain[index];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         AFE_Cal_Scale()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Meter_Current()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Current for Metering
//
//  MECHANICS:          This subroutine computes the 200msec filtered RMS currents.  It is called every
//                      200msec (12 cycles at 60Hz).  Each current value is computed as a floating-point
//                      number.  The values are generated over a 200msec window, using the metered
//                      (filtered) sample values.  The following currents are computed:
//                          Per-phase 200msec filtered RMS current: Cur200msFltr.Ix
//                      These currents are used for metering.  The (unfiltered one-cycle) current values 
//                      used for protection are computed in Calc_Prot_Current().
//
//  CAVEATS:            None
//
//  INPUTS:             FreezeMinMax, Cur200msFltrSOS_Sav.Ix
// 
//  OUTPUTS:            Cur200msFltr.Ix, Cur200msFltrIg, Cur200msIavg
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Meter_Current(void)
{
  Cur200msFltr.Ia = sqrtf(Cur200msFltrSOS_Sav.Ia/960);              // *** DAH  This will need to be modified based on 50/60Hz operation
  Cur200msFltr.Ib = sqrtf(Cur200msFltrSOS_Sav.Ib/960);
  Cur200msFltr.Ic = sqrtf(Cur200msFltrSOS_Sav.Ic/960);
  Cur200msFltr.In = sqrtf(Cur200msFltrSOS_Sav.In/960);
  Cur200msFltr.Igsrc = sqrtf(Cur200msFltrSOS_Sav.Igsrc/960);
  Cur200msFltr.Igres = sqrtf(Cur200msFltrSOS_Sav.Igres/960);
  if(Setpoints1.stp.Gnd_Type)//***ALG if Gnd_Type is 1-Sensing (read-only) [0=residual, 1=source/zero sequence]
  {
    Cur200msFltrIg = Cur200msFltr.Igsrc;
  }
  else
  {
    Cur200msFltrIg = Cur200msFltr.Igres; 
  }                              // *** DAH  NEED TO SET BASED ON SETPOINT; SET TO IGRES FOR NOW
  Cur200msIavg = (Cur200msFltr.Ia + Cur200msFltr.Ib + Cur200msFltr.Ic)/3;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Meter_Current()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Prot_Current()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Current for Protection
//
//  MECHANICS:          This subroutine computes the one-cycle unfiltered RMS currents.  It is called every
//                      cycle.  Each current value is computed as a floating-point number.  The values are
//                      generated over a one-cycle window, using the unfiltered sample values.  The
//                      following currents are computed:
//                          Per-phase one-cycle unfiltered RMS current: CurOneCyc.Ix
//                          Per-phase maximum one-cycle unfiltered RMS current: CurOneCyc_max.Ix
//                          Per-phase minimum one-cycle unfiltered RMS current: CurOneCyc_min.Ix
//                      These currents are used for protection and the following metering computations:
//                          crest factor
//                          min/max
//                          unbalance
//                      The (filtered 200msec) current values used for metering are computed in
//                      Calc_Meter_Current().
//
//  CAVEATS:            None
//
//  INPUTS:             CurOneCycSOS_Sav.Ix
// 
//  OUTPUTS:            CurOneCyc.Ix, CurOneCyc_max.Ix, CurOneCyc_max.IxTS, CurOneCyc_min.Ix,
//                      CurOneCyc_min.IxTS, CurUnbalTot, CurUnbal.Ix, CurUnbalMax, CurUnbalPh_max.Ix,
//                      CurUnbalPh_max.IxTS, CurUnbalAllMax, CurUnbalAllMaxTS
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//  EXECUTION TIME:     Measured on 190813 (rev 0.33 code): 2.3usec (with interrupts disabled)
//
//------------------------------------------------------------------------------------------------------------

void Calc_Prot_Current(void)
{
  float imax, imin;
  struct INTERNAL_TIME presenttime;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  CurOneCyc.Ia = sqrtf(CurOneCycSOS_Sav.Ia/80);
  CurOneCyc.Ib = sqrtf(CurOneCycSOS_Sav.Ib/80);
  CurOneCyc.Ic = sqrtf(CurOneCycSOS_Sav.Ic/80);
  CurOneCyc.In = sqrtf(CurOneCycSOS_Sav.In/80);
  CurOneCyc.Igsrc = sqrtf(CurOneCycSOS_Sav.Igsrc/80);
  CurOneCyc.Igres = sqrtf(CurOneCycSOS_Sav.Igres/80);
  if(Setpoints1.stp.Gnd_Type)// *** DAH  NEED TO SET BASED ON SETPOINT; SET TO IGRES FOR NOW
  {//***ALG if Gnd_Type is 1-Sensing (read-only) [0=residual, 1=source/zero sequence]
  CurOneCycIg = CurOneCyc.Igsrc;
  }
  else
  {
    CurOneCycIg = CurOneCyc.Igres;
  }
  CurOneCycIavg = (CurOneCyc.Ia + CurOneCyc.Ib + CurOneCyc.Ic)/3;

  // *** DAH  IMPORTANT NOTE: THIS IS PRESENTLY UNDER FREEZE CONTROL.  IF WE KEEP FREEZE CONTROL, THESE
  //          MIN/MAX VALUES CAN ONLY BE USED FOR METERING.  WE WILL NEED SEPARATE VARIABLES FOR PROTECTION
  //          AND ALARMS.  THE NEW VARIABLES WILL ALWAYS BE UPDATED AND WILL NOT BE UNDER FREEZE CONTROL.
  //          ALSO, BE CAREFUL THAT THE PROTECTION VALUES ARE NOT USED UNTIL THEY ARE VALID (INITIALIZE TO
  //          NAN, CHECK FOR NAN IN THE PROTECTION SUBROUTINE)
  if (!FreezeMinMax)
  {
    // Check for minimum and maximum.  Use value as new min or max if the existing min or max value is
    //   invalid
    // Note, we don't use an "else if" on the max calculation because after a reset, the first demand will
    //   be both a min and a max.
    if ( (CurOneCyc.Ia < CurOneCyc_min.Ia) || (isnan(CurOneCyc_min.Ia)) )      // Check for min
    {
      CurOneCyc_min.Ia = CurOneCyc.Ia;
      CurOneCyc_min.IaTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (CurOneCyc.Ia > CurOneCyc_max.Ia) || (isnan(CurOneCyc_max.Ia)) )      // Check for max
    {
      CurOneCyc_max.Ia = CurOneCyc.Ia;
      CurOneCyc_max.IaTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MAX
    }
    if ( (CurOneCyc.Ib < CurOneCyc_min.Ib) || (isnan(CurOneCyc_min.Ib)) )      // Check for min
    {
      CurOneCyc_min.Ib = CurOneCyc.Ib;
      CurOneCyc_min.IbTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (CurOneCyc.Ib > CurOneCyc_max.Ib) || (isnan(CurOneCyc_max.Ib)) )      // Check for max
    {
      CurOneCyc_max.Ib = CurOneCyc.Ib;
      CurOneCyc_max.IbTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MAX
    }
    if ( (CurOneCyc.Ic < CurOneCyc_min.Ic) || (isnan(CurOneCyc_min.Ic)) )      // Check for min
    {
      CurOneCyc_min.Ic = CurOneCyc.Ic;
      CurOneCyc_min.IcTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (CurOneCyc.Ic > CurOneCyc_max.Ic) || (isnan(CurOneCyc_max.Ic)) )      // Check for max
    {
      CurOneCyc_max.Ic = CurOneCyc.Ic;
      CurOneCyc_max.IcTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MAX
    }
    if ( (CurOneCyc.In < CurOneCyc_min.In) || (isnan(CurOneCyc_min.In)) )      // Check for min
    {
      CurOneCyc_min.In = CurOneCyc.In;
      CurOneCyc_min.InTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (CurOneCyc.In > CurOneCyc_max.In) || (isnan(CurOneCyc_max.In)) )      // Check for max
    {
      CurOneCyc_max.In = CurOneCyc.In;
      CurOneCyc_max.InTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MAX
    }
    if ( (CurOneCycIg < CurOneCyc_min.Ig) || (isnan(CurOneCyc_min.Ig)) )       // Check for min
    {
      CurOneCyc_min.Ig = CurOneCycIg;
      CurOneCyc_min.IgTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (CurOneCycIg > CurOneCyc_max.Ig) || (isnan(CurOneCyc_max.Ig)) )       // Check for max
    {
      CurOneCyc_max.Ig = CurOneCycIg;
      CurOneCyc_max.IgTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MAX
    }
  }

  // Compute total current unbalance
  imax = ((CurOneCyc.Ia > CurOneCyc.Ib) ? CurOneCyc.Ia : CurOneCyc.Ib);    // Compute Imax
  imax = ((CurOneCyc.Ic > imax) ? CurOneCyc.Ic : imax);
  imin = ((CurOneCyc.Ia < CurOneCyc.Ib) ? CurOneCyc.Ia : CurOneCyc.Ib);    // Compute Imin
  imin = ((CurOneCyc.Ic < imin) ? CurOneCyc.Ic : imin);
  if (imax > 0)                             // Compute current unbalance if max current > *** DAH ONLY COMPUTE IF GREATER THAN ?
  {
    CurUnbalTot = ((imax - imin)/imax) * 100.0;
  }

  CurOneCycMax = imax;                                      // Used in Current Unbalance protection routines

  // Compute per-phase current unbalances.  These are equal to (Ix - Iavg)/Iavg * 100.  Use imin as a
  //   holder for Iavg
  imin = (CurOneCyc.Ia + CurOneCyc.Ib + CurOneCyc.Ia)/3;        // Average current does not include neutral
  if (imin > 0)
  {
    CurUnbal.Ia = ((CurOneCyc.Ia - imin)/imin) * 100;
    CurUnbal.Ia = ((CurUnbal.Ia < 0) ? (-1.0 * CurUnbal.Ia) : CurUnbal.Ia);
    CurUnbal.Ib = ((CurOneCyc.Ib - imin)/imin) * 100;
    CurUnbal.Ib = ((CurUnbal.Ib < 0) ? (-1.0 * CurUnbal.Ib) : CurUnbal.Ib);
    CurUnbal.Ic = ((CurOneCyc.Ic - imin)/imin) * 100;
    CurUnbal.Ic = ((CurUnbal.Ic < 0) ? (-1.0 * CurUnbal.Ic) : CurUnbal.Ic);
    CurUnbal.In = ((CurOneCyc.In - imin)/imin) * 100;
    CurUnbal.In = ((CurUnbal.In < 0) ? (-1.0 * CurUnbal.In) : CurUnbal.In);
  }

  // Update the present max unbalance
  CurUnbalMax = ((CurUnbal.Ia > CurUnbal.Ib) ? CurUnbal.Ia : CurUnbal.Ib);
  CurUnbalMax = ((CurUnbal.Ic > CurUnbalMax) ? CurUnbal.Ic : CurUnbalMax);
  CurUnbalMax = ((CurUnbal.In > CurUnbalMax) ? CurUnbal.In : CurUnbalMax);

  // Update max total and per-phase current unbalances
  if (!FreezeMinMax)
  {
    if ( (CurUnbalTot > CurUnbalTotMax) || (isnan(CurUnbalTotMax)) )
    {
      CurUnbalTotMax = CurUnbalTot;
      CurUnbalTotMaxTS = presenttime;
    }
    if ( (CurUnbal.Ia > CurUnbalPh_max.Ia) || (isnan(CurUnbalPh_max.Ia)) )
    {
      CurUnbalPh_max.Ia = CurUnbal.Ia;
      CurUnbalPh_max.IaTS = presenttime;
    }
    if ( (CurUnbal.Ib > CurUnbalPh_max.Ib) || (isnan(CurUnbalPh_max.Ib)) )      // Check for max
    {
      CurUnbalPh_max.Ib = CurUnbal.Ib;
      CurUnbalPh_max.IbTS = presenttime;
    }
    if ( (CurUnbal.Ic > CurUnbalPh_max.Ic) || (isnan(CurUnbalPh_max.Ic)) )      // Check for max
    {
      CurUnbalPh_max.Ic = CurUnbal.Ic;
      CurUnbalPh_max.IcTS = presenttime;
    }
    if ( (CurUnbal.In > CurUnbalPh_max.In) || (isnan(CurUnbalPh_max.In)) )      // Check for max
    {
      CurUnbalPh_max.In = CurUnbal.In;
      CurUnbalPh_max.InTS = presenttime;
    }

    // Update overall max unbalance.  First find the max of the four max unbalances
    imax = ((CurUnbalPh_max.Ia > CurUnbalPh_max.Ib) ? CurUnbalPh_max.Ia : CurUnbalPh_max.Ib);
    imax = ((CurUnbalPh_max.Ic > imax) ? CurUnbalPh_max.Ic : imax);
    imax = ((CurUnbalPh_max.In > imax) ? CurUnbalPh_max.In : imax);
    // If this max value is greater than the overall max, set the overall max to this value and the
    //   timestamp to the present time
    if (imax > CurUnbalAllMax)
    {
      CurUnbalAllMax = imax;
      CurUnbalAllMaxTS = presenttime;
    }
  }

  // Coil Detection Measurement
  if (CoilDetect.State == 1)                                     // Coil Detection Phase A testing is active
  {
    if (CoilDetect.SampleCount < 240)                            // Collect 4 seconds of one-cycle currents
    {  
      CoilDetect.Avg_Ia += CurOneCyc.Ia;                         // Sum the 240 values
      CoilDetect.SampleCount++;
    }       
  }  
  else if (CoilDetect.State == 2)                                // Coil Detection Phase B testing is active
  {
    if (CoilDetect.SampleCount < 240)                            // Collect 4 seconds of one-cycle currents
    {  
      CoilDetect.Avg_Ib += CurOneCyc.Ib;                         // Sum the 240 values
      CoilDetect.SampleCount++;
    }       
  }   
  else if (CoilDetect.State == 3)                                // Coil Detection Phase C testing is active
  {
    if (CoilDetect.SampleCount < 240)                            // Collect 4 seconds of one-cycle currents
    {  
      CoilDetect.Avg_Ic += CurOneCyc.Ic;                         // Sum the 240 values
      CoilDetect.SampleCount++;
    }       
  }     
  else if (CoilDetect.State == 4)                                // Coil Detection Phase N testing is active
  {
    if (CoilDetect.SampleCount < 240)                            // Collect 4 seconds of one-cycle currents
    {  
      CoilDetect.Avg_In += CurOneCyc.In;                         // Sum the 240 values
      CoilDetect.SampleCount++;
    }       
  }               
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Prot_Current()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Meter_AFE_Voltage()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate AFE Voltage for Metering
//
//  MECHANICS:          This subroutine computes the 200msec filtered RMS AFE voltages.  It is called every
//                      200msec (12 cycles at 60Hz).  Each voltage value is computed as a floating-point
//                      number.  The values are generated over a 200msec window, using the metered
//                      (filtered) AFE sample values.  The following voltages are computed:
//                          Per-phase 200msec filtered RMS voltage: VolAFE200msFltr.Vxx
//                          Per-phase maximum 200msec filtered RMS voltage: VolAFE200msFltr_max.Vxx
//                          Per-phase minimum 200msec filtered RMS voltage: VolAFE200msFltr_min.Vxx
//                      These voltages are used for metering.  The (unfiltered one-cycle) voltage values 
//                      used for protection are computed in Calc_Prot_AFE_Voltage().
//
//  CAVEATS:            None
//
//  INPUTS:             VolAFE200msFltrSOS_Sav.Vxx
// 
//  OUTPUTS:            VolAFE200msFltr.Vxx, VolAFE200msFltrVllavg, VolAFE200msFltrVlnavg
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Meter_AFE_Voltage(void)
{
  // Compute 200msec voltages
  VolAFE200msFltr.Van = sqrtf(VolAFE200msFltrSOS_Sav.Van/960);  // *** DAH  This will need to be modified based on 50/60Hz operation
  VolAFE200msFltr.Vbn = sqrtf(VolAFE200msFltrSOS_Sav.Vbn/960);
  VolAFE200msFltr.Vcn = sqrtf(VolAFE200msFltrSOS_Sav.Vcn/960);
  VolAFE200msFltr.Vab = sqrtf(VolAFE200msFltrSOS_Sav.Vab/960);
  VolAFE200msFltr.Vbc = sqrtf(VolAFE200msFltrSOS_Sav.Vbc/960);
  VolAFE200msFltr.Vca = sqrtf(VolAFE200msFltrSOS_Sav.Vca/960);
  VolAFE200msFltrVllavg = (VolAFE200msFltr.Vab + VolAFE200msFltr.Vbc + VolAFE200msFltr.Vca)/3;
  VolAFE200msFltrVlnavg = (VolAFE200msFltr.Van + VolAFE200msFltr.Vbn + VolAFE200msFltr.Vcn)/3;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Meter_AFE_Voltage()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Prot_AFE_Voltage()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate AFE Voltage for Protection
//
//  MECHANICS:          This subroutine computes the one-cycle unfiltered RMS AFE voltages.  It is called
//                      cycle.  Each voltage value is computed as a floating-point number.  The values are
//                      generated over a one-cycle window, using the unfiltered AFE sample values.  The
//                      following voltages are computed:
//                          Per-phase one-cycle unfiltered RMS voltage: VolAFEOneCyc.Vxx
//                          Per-phase maximum one-cycle unfiltered RMS voltage: VolAFEOneCyc_max.Vxx
//                          Per-phase minimum one-cycle unfiltered RMS voltage: VolAFEOneCyc_min.Vxx
//                      These voltages are used for protection.  The min/max values are also used for
//                      metering.  The (filtered 200msec) voltage values used for metering are computed in
//                      Calc_Meter_AFE_Voltage().
//
//  CAVEATS:            None
//
//  INPUTS:             FreezeMinMax, VolAFEOneCycSOS_Sav.Vxx, VolAFEOneCyc_min.Vxx, VolAFEOneCyc_max.Vxx
// 
//  OUTPUTS:            VolAFEOneCyc.Vxx, VolAFEOneCyc_min.Vxx, VolAFEOneCyc_max.Vxx,
//                      VolAFEOneCyc_min.VxxTS, VolAFEOneCyc_max.VxxTS
//
//  ALTERS:             None
// 
//  CALLS:              sqrtf()
// 
//  EXECUTION TIME:     Measured on 190813 (rev 0.33 code): 3.5usec (with interrupts disabled)
//
//------------------------------------------------------------------------------------------------------------

void Calc_Prot_AFE_Voltage(void)
{
  float vmax, vmin;
  struct INTERNAL_TIME presenttime;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  // Compute one-cycle voltages
  VolAFEOneCyc.Van = sqrtf(VolAFEOneCycSOS_Sav.Van/80);
  VolAFEOneCyc.Vbn = sqrtf(VolAFEOneCycSOS_Sav.Vbn/80);
  VolAFEOneCyc.Vcn = sqrtf(VolAFEOneCycSOS_Sav.Vcn/80);
  VolAFEOneCyc.Vab = sqrtf(VolAFEOneCycSOS_Sav.Vab/80);
  VolAFEOneCyc.Vbc = sqrtf(VolAFEOneCycSOS_Sav.Vbc/80);
  VolAFEOneCyc.Vca = sqrtf(VolAFEOneCycSOS_Sav.Vca/80);

  // *** DAH  IMPORTANT NOTE: THIS IS PRESENTLY UNDER FREEZE CONTROL.  IF WE KEEP FREEZE CONTROL, THESE
  //          MIN/MAX VALUES CAN ONLY BE USED FOR METERING.  WE WILL NEED SEPARATE VARIABLES FOR PROTECTION
  //          AND ALARMS.  THE NEW VARIABLES WILL ALWAYS BE UPDATED AND WILL NOT BE UNDER FREEZE CONTROL.
  //          ALSO, BE CAREFUL THAT THE PROTECTIO VALUES ARE NOT USED UNTIL THEY ARE VALID (INITIALIZE TO
  //          NAN, CHECK FOR NAN IN THE PROTECTION SUBROUTINE)
  if (!FreezeMinMax)
  {
    // Check for minimum and maximum.  Use value as new min or max if the existing min or max value is
    //   invalid
    // Note, we don't use an "else if" on the max calculation because after a reset, the first demand will
    //   be both a min and a max.
    // Check for min
    if ( (VolAFEOneCyc.Van < VolAFEOneCyc_min.Van) || (isnan(VolAFEOneCyc_min.Van)) )
    {
      VolAFEOneCyc_min.Van = VolAFEOneCyc.Van;
      VolAFEOneCyc_min.VanTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for max
    if ( (VolAFEOneCyc.Van > VolAFEOneCyc_max.Van) || (isnan(VolAFEOneCyc_max.Van)) )
    {
      VolAFEOneCyc_max.Van = VolAFEOneCyc.Van;
      VolAFEOneCyc_max.VanTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolAFEOneCyc.Vbn < VolAFEOneCyc_min.Vbn) || (isnan(VolAFEOneCyc_min.Vbn)) )
    {
      VolAFEOneCyc_min.Vbn = VolAFEOneCyc.Vbn;
      VolAFEOneCyc_min.VbnTS = presenttime;
    }
    // Check for max
    if ( (VolAFEOneCyc.Vbn > VolAFEOneCyc_max.Vbn) || (isnan(VolAFEOneCyc_max.Vbn)) )
    {
      VolAFEOneCyc_max.Vbn = VolAFEOneCyc.Vbn;
      VolAFEOneCyc_max.VbnTS = presenttime;
    }
    if ( (VolAFEOneCyc.Vcn < VolAFEOneCyc_min.Vcn) || (isnan(VolAFEOneCyc_min.Vcn)) )
    {
      VolAFEOneCyc_min.Vcn = VolAFEOneCyc.Vcn;
      VolAFEOneCyc_min.VcnTS = presenttime;
    }
    // Check for max
    if ( (VolAFEOneCyc.Vcn > VolAFEOneCyc_max.Vcn) || (isnan(VolAFEOneCyc_max.Vcn)) )
    {
      VolAFEOneCyc_max.Vcn = VolAFEOneCyc.Vcn;
      VolAFEOneCyc_max.VcnTS = presenttime;
    }
    if ( (VolAFEOneCyc.Vab < VolAFEOneCyc_min.Vab) || (isnan(VolAFEOneCyc_min.Vab)) )
    {
      VolAFEOneCyc_min.Vab = VolAFEOneCyc.Vab;
      VolAFEOneCyc_min.VabTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for max
    if ( (VolAFEOneCyc.Vab > VolAFEOneCyc_max.Vab) || (isnan(VolAFEOneCyc_max.Vab)) )
    {
      VolAFEOneCyc_max.Vab = VolAFEOneCyc.Vab;
      VolAFEOneCyc_max.VabTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolAFEOneCyc.Vbc < VolAFEOneCyc_min.Vbc) || (isnan(VolAFEOneCyc_min.Vbc)) )
    {
      VolAFEOneCyc_min.Vbc = VolAFEOneCyc.Vbc;
      VolAFEOneCyc_min.VbcTS = presenttime;
    }
    // Check for max
    if ( (VolAFEOneCyc.Vbc > VolAFEOneCyc_max.Vbc) || (isnan(VolAFEOneCyc_max.Vbc)) )
    {
      VolAFEOneCyc_max.Vbc = VolAFEOneCyc.Vbc;
      VolAFEOneCyc_max.VbcTS = presenttime;
    }
    if ( (VolAFEOneCyc.Vca < VolAFEOneCyc_min.Vca) || (isnan(VolAFEOneCyc_min.Vca)) )
    {
      VolAFEOneCyc_min.Vca = VolAFEOneCyc.Vca;
      VolAFEOneCyc_min.VcaTS = presenttime;
    }
    // Check for max
    if ( (VolAFEOneCyc.Vca > VolAFEOneCyc_max.Vca) || (isnan(VolAFEOneCyc_max.Vca)) )
    {
      VolAFEOneCyc_max.Vca = VolAFEOneCyc.Vca;
      VolAFEOneCyc_max.VcaTS = presenttime;
    }
  }

  // Compute voltage unbalance
  vmax = ((VolAFEOneCyc.Vab > VolAFEOneCyc.Vbc) ? VolAFEOneCyc.Vab : VolAFEOneCyc.Vbc);    // Compute Vmax
  vmax = ((vmax > VolAFEOneCyc.Vca) ? vmax : VolAFEOneCyc.Vca);
  vmin = ((VolAFEOneCyc.Vab < VolAFEOneCyc.Vbc) ? VolAFEOneCyc.Vab : VolAFEOneCyc.Vbc);    // Compute Vmin
  vmin = ((vmin < VolAFEOneCyc.Vca) ? vmin : VolAFEOneCyc.Vca);
  if (vmax > 0)                             // Compute voltage unbalance if max voltage > *** DAH ONLY COMPUTE IF GREATER THAN ?
  {
    VolUnbalTot = ((vmax - vmin)/vmax) * 100.0;
  }

  // Save vmax and vmin values for Over/Under Voltage protection
  Vll_max = vmax;
  Vll_min = vmin;

  // Compute per-phase voltage unbalances.  These are equal to (Vx - Vavg)/Vavg * 100.  Use vmin as a holder
  //   for Vavg
  vmin = (VolAFEOneCyc.Van + VolAFEOneCyc.Vbn + VolAFEOneCyc.Vcn)/3;            // Average voltage
  if (vmin > 0)
  {
    VolUnbal.Van = ((VolAFEOneCyc.Van - vmin)/vmin) * 100;
    VolUnbal.Van = ((VolUnbal.Van < 0) ? (-1.0 * VolUnbal.Van) : VolUnbal.Van);
    VolUnbal.Vbn = ((VolAFEOneCyc.Vbn - vmin)/vmin) * 100;
    VolUnbal.Vbn = ((VolUnbal.Vbn < 0) ? (-1.0 * VolUnbal.Vbn) : VolUnbal.Vbn);
    VolUnbal.Vcn = ((VolAFEOneCyc.Vcn - vmin)/vmin) * 100;
    VolUnbal.Vcn = ((VolUnbal.Vcn < 0) ? (-1.0 * VolUnbal.Vcn) : VolUnbal.Vcn);
  }
  vmin = (VolAFEOneCyc.Vab + VolAFEOneCyc.Vbc + VolAFEOneCyc.Vca)/3;            // Average voltage
  if (vmin > 0)
  {
    VolUnbal.Vab = ((VolAFEOneCyc.Vab - vmin)/vmin) * 100;
    VolUnbal.Vab = ((VolUnbal.Vab < 0) ? (-1.0 * VolUnbal.Vab) : VolUnbal.Vab);
    VolUnbal.Vbc = ((VolAFEOneCyc.Vbc - vmin)/vmin) * 100;
    VolUnbal.Vbc = ((VolUnbal.Vbc < 0) ? (-1.0 * VolUnbal.Vbc) : VolUnbal.Vbc);
    VolUnbal.Vca = ((VolAFEOneCyc.Vca - vmin)/vmin) * 100;
    VolUnbal.Vca = ((VolUnbal.Vca < 0) ? (-1.0 * VolUnbal.Vca) : VolUnbal.Vca);
  }

  // Update the present max unbalances
  VolUnbalMaxLN = ((VolUnbal.Van > VolUnbal.Vbn) ? VolUnbal.Van : VolUnbal.Vbn);
  VolUnbalMaxLN = ((VolUnbal.Vcn > VolUnbalMaxLN) ? VolUnbal.Vcn : VolUnbalMaxLN);
  VolUnbalMaxLL = ((VolUnbal.Vab > VolUnbal.Vbc) ? VolUnbal.Vab : VolUnbal.Vbc);
  VolUnbalMaxLL = ((VolUnbal.Vca > VolUnbalMaxLL) ? VolUnbal.Vca : VolUnbalMaxLL);

  // Update max total and per-phase voltage unbalances
  if (!FreezeMinMax)
  {
    if ( (VolUnbalTot > VolUnbalTotMax) || (isnan(VolUnbalTotMax)) )
    {
      VolUnbalTotMax = VolUnbalTot;
      VolUnbalTotMaxTS = presenttime;
    }
    if ( (VolUnbal.Van > VolUnbalPh_max.Van) || (isnan(VolUnbalPh_max.Van)) )
    {
      VolUnbalPh_max.Van = VolUnbal.Van;
      VolUnbalPh_max.VanTS = presenttime;
    }
    if ( (VolUnbal.Vbn > VolUnbalPh_max.Vbn) || (isnan(VolUnbalPh_max.Vbn)) )
    {
      VolUnbalPh_max.Vbn = VolUnbal.Vbn;
      VolUnbalPh_max.VbnTS = presenttime;
    }
    if ( (VolUnbal.Vcn > VolUnbalPh_max.Vcn) || (isnan(VolUnbalPh_max.Vcn)) )
    {
      VolUnbalPh_max.Vcn = VolUnbal.Vcn;
      VolUnbalPh_max.VcnTS = presenttime;
    }
    if ( (VolUnbal.Vab > VolUnbalPh_max.Vab) || (isnan(VolUnbalPh_max.Vab)) )
    {
      VolUnbalPh_max.Vab = VolUnbal.Vab;
      VolUnbalPh_max.VabTS = presenttime;
    }
    if ( (VolUnbal.Vbc > VolUnbalPh_max.Vbc) || (isnan(VolUnbalPh_max.Vbc)) )
    {
      VolUnbalPh_max.Vbc = VolUnbal.Vbc;
      VolUnbalPh_max.VbcTS = presenttime;
    }
    if ( (VolUnbal.Vca > VolUnbalPh_max.Vca) || (isnan(VolUnbalPh_max.Vca)) )
    {
      VolUnbalPh_max.Vca = VolUnbal.Vca;
      VolUnbalPh_max.VcaTS = presenttime;
    }

    // Update overall max unbalance.  First find the max of the four max unbalances
    vmax = ((VolUnbalPh_max.Van > VolUnbalPh_max.Vbn) ? VolUnbalPh_max.Van : VolUnbalPh_max.Vbn);
    vmax = ((VolUnbalPh_max.Vcn > vmax) ? VolUnbalPh_max.Vcn : vmax);
    // If this max value is greater than the overall max, set the overall max to this value and the
    //   timestamp to the present time
    if (vmax > VolUnbalAllMaxLN)
    {
      VolUnbalAllMaxLN = vmax;
      VolUnbalAllMaxLNTS = presenttime;
    }
    vmax = ((VolUnbalPh_max.Vab > VolUnbalPh_max.Vbc) ? VolUnbalPh_max.Vab : VolUnbalPh_max.Vbc);
    vmax = ((VolUnbalPh_max.Vca > vmax) ? VolUnbalPh_max.Vca : vmax);
    // If this max value is greater than the overall max, set the overall max to this value and the
    //   timestamp to the present time
    if (vmax > VolUnbalAllMaxLL)
    {
      VolUnbalAllMaxLL = vmax;
      VolUnbalAllMaxLLTS = presenttime;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Prot_AFE_Voltage()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_ADC_200ms_Voltage()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate 200msec ADC Voltages for Metering
//
//  MECHANICS:          This subroutine computes the 200msec unfiltered RMS ADC voltages.  It is called
//                      every 200msec (12 cycles at 60Hz).  Each voltage value is computed as a
//                      floating-point number.  The values are generated over a 200msec window, using the
//                      (unfiltered) ADC sample values.  The following voltages are computed:
//                          Per-phase 200msec unfiltered RMS voltage: VolADC200ms.Vxx
//                      These voltages are used for metering.  The (one-cycle) voltage values 
//                      used for protection are computed in Calc_Prot_AFE_Voltage().
//
//  CAVEATS:            None
//
//  INPUTS:             FreezeMinMax, VolADC200msSOS_Sav.Vxx, VolADC200ms_min.Vxx, VolADC200ms_max.Vxx
// 
//  OUTPUTS:            VolADC200ms.Vxx, VolADC200ms_min.Vxx, VolADC200ms_max.Vxx, VolADC200ms_min.VxxTS,
//                      VolADC200ms_max.VxxTS, VolADC200msVllavg, VolADC200msVlnavg
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_ADC_200ms_Voltage(void)
{
  VolADC200ms.Van = sqrtf(VolADC200msSOS_Sav.Van/960);  // *** DAH  This will need to be modified based on 50/60Hz operation
  VolADC200ms.Vbn = sqrtf(VolADC200msSOS_Sav.Vbn/960);
  VolADC200ms.Vcn = sqrtf(VolADC200msSOS_Sav.Vcn/960);
  VolADC200ms.Vab = sqrtf(VolADC200msSOS_Sav.Vab/960);
  VolADC200ms.Vbc = sqrtf(VolADC200msSOS_Sav.Vbc/960);
  VolADC200ms.Vca = sqrtf(VolADC200msSOS_Sav.Vca/960);
  VolADC200msVllavg = (VolADC200ms.Vab + VolADC200ms.Vbc + VolADC200ms.Vca)/3;
  VolADC200msVlnavg = (VolADC200ms.Van + VolADC200ms.Vbn + VolADC200ms.Vcn)/3;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_ADC_200ms_Voltage()
//------------------------------------------------------------------------------------------------------------








//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_ADC_OneCyc_Voltage()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate ADC Voltage for Metering
//
//  MECHANICS:          This subroutine computes the one-cycle unfiltered RMS ADC voltages.  It is called
//                      every one-cycle anniversary.  Each voltage value is computed as a floating-point
//                      number.  The values are generated over a one-cycle window, using the (unfiltered)
//                      ADC sample values.  The following voltages are computed:
//                          Per-phase one-cycle unfiltered RMS voltage: VolADCOneCyc.Vxx
//                          Per-phase maximum one-cycle unfiltered RMS voltage: VolADCOneCyc_max.Vxx
//                          Per-phase minimum one-cycle unfiltered RMS voltage: VolADCOneCyc_min.Vxx
//                      These voltages are used for metering.  The (one-cycle) voltage values 
//                      used for protection are computed in Calc_Prot_AFE_Voltage().
//
//  CAVEATS:            None
//
//  INPUTS:             FreezeMinMax, VolADCOneCycSOS_Sav.Vxx, VolADCOneCyc_min.Vxx, VolADCOneCyc_max.Vxx
// 
//  OUTPUTS:            VolADCOneCyc.Vxx, VolADCOneCyc_min.Vxx, VolADCOneCyc_max.Vxx, VolADCOneCyc_min.VxxTS,
//                      VolADCOneCyc_max.VxxTS, VolADC200msVllavg, VolADC200msVlnavg
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_ADC_OneCyc_Voltage(void)
{
  struct INTERNAL_TIME presenttime;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  VolADCOneCyc.Van = sqrtf(VolADCOneCycSOS_Sav.Van/80);           // *** DAH  ARE THESE USED FOR ANYTHING OTHER THAN MIN/MAX?
  VolADCOneCyc.Vbn = sqrtf(VolADCOneCycSOS_Sav.Vbn/80);
  VolADCOneCyc.Vcn = sqrtf(VolADCOneCycSOS_Sav.Vcn/80);
  VolADCOneCyc.Vab = sqrtf(VolADCOneCycSOS_Sav.Vab/80);
  VolADCOneCyc.Vbc = sqrtf(VolADCOneCycSOS_Sav.Vbc/80);
  VolADCOneCyc.Vca = sqrtf(VolADCOneCycSOS_Sav.Vca/80);

  if (!FreezeMinMax)
  {
    // Check for minimum and maximum.  Use value as new min or max if the existing min or max value is
    //   invalid
    // Note, we don't use an "else if" on the max calculation because after a reset, the first demand will
    //   be both a min and a max.
    if ( (VolADCOneCyc.Van < VolADCOneCyc_min.Van) || (isnan(VolADCOneCyc_min.Van)) )       // Check for min
    {
      VolADCOneCyc_min.Van = VolADCOneCyc.Van;
      VolADCOneCyc_min.VanTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolADCOneCyc.Van > VolADCOneCyc_max.Van) || (isnan(VolADCOneCyc_max.Van)) )       // Check for max
    {
      VolADCOneCyc_max.Van = VolADCOneCyc.Van;
      VolADCOneCyc_max.VanTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolADCOneCyc.Vbn < VolADCOneCyc_min.Vbn) || (isnan(VolADCOneCyc_min.Vbn)) )
    {
      VolADCOneCyc_min.Vbn = VolADCOneCyc.Vbn;
      VolADCOneCyc_min.VbnTS = presenttime;
    }
    if ( (VolADCOneCyc.Vbn > VolADCOneCyc_max.Vbn) || (isnan(VolADCOneCyc_max.Vbn)) )
    {
      VolADCOneCyc_max.Vbn = VolADCOneCyc.Vbn;
      VolADCOneCyc_max.VbnTS = presenttime;
    }
    if ( (VolADCOneCyc.Vcn < VolADCOneCyc_min.Vcn) || (isnan(VolADCOneCyc_min.Vcn)) )
    {
      VolADCOneCyc_min.Vcn = VolADCOneCyc.Vcn;
      VolADCOneCyc_min.VcnTS = presenttime;
    }
    if ( (VolADCOneCyc.Vcn > VolADCOneCyc_max.Vcn) || (isnan(VolADCOneCyc_max.Vcn)) )
    {
      VolADCOneCyc_max.Vcn = VolADCOneCyc.Vcn;
      VolADCOneCyc_max.VcnTS = presenttime;
    }
    if ( (VolADCOneCyc.Vab < VolADCOneCyc_min.Vab) || (isnan(VolADCOneCyc_min.Vab)) )       // Check for min
    {
      VolADCOneCyc_min.Vab = VolADCOneCyc.Vab;
      VolADCOneCyc_min.VabTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolADCOneCyc.Vab > VolADCOneCyc_max.Vab) || (isnan(VolADCOneCyc_max.Vab)) )       // Check for max
    {
      VolADCOneCyc_max.Vab = VolADCOneCyc.Vab;
      VolADCOneCyc_max.VabTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (VolADCOneCyc.Vbc < VolADCOneCyc_min.Vbc) || (isnan(VolADCOneCyc_min.Vbc)) )
    {
      VolADCOneCyc_min.Vbc = VolADCOneCyc.Vbc;
      VolADCOneCyc_min.VbcTS = presenttime;
    }
    if ( (VolADCOneCyc.Vbc > VolADCOneCyc_max.Vbc) || (isnan(VolADCOneCyc_max.Vbc)) )
    {
      VolADCOneCyc_max.Vbc = VolADCOneCyc.Vbc;
      VolADCOneCyc_max.VbcTS = presenttime;
    }
    if ( (VolADCOneCyc.Vca < VolADCOneCyc_min.Vca) || (isnan(VolADCOneCyc_min.Vca)) )
    {
      VolADCOneCyc_min.Vca = VolADCOneCyc.Vca;
      VolADCOneCyc_min.VcaTS = presenttime;
    }
    if ( (VolADCOneCyc.Vca > VolADCOneCyc_max.Vca) || (isnan(VolADCOneCyc_max.Vca)) )
    {
      VolADCOneCyc_max.Vca = VolADCOneCyc.Vca;
      VolADCOneCyc_max.VcaTS = presenttime;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_ADC_OneCyc_Voltage()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Meter_Power()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Power for Metering
//
//  MECHANICS:          This subroutine computes the real, reactive, and apparent power values.  It is
//                      called every 200msec, after the voltage and current values have been computed, and
//                      before the energy values are computed.
//                      Each power value is computed as a floating-point number.  The values are generated
//                      over a 200msec window, using the metered (filtered) sample values.  The following
//                      powers are computed:
//                          Per-phase real power: Pwr200msec.Px
//                          Total real power: Pwr200msec.Ptot
//                          Per-phase reactive power: Pwr200msec.RPx
//                          Total reactive power: Pwr200msec.Rtot
//                          Per-phase apparent power: Pwr200msecApp.AppPx
//                          Total apparent power: Pwr200msecApp.Apptot
//                          Per-phase maximum real power: Pwr200msecMax.Px
//                          Per-phase minimum real power: Pwr200msecMin.Px
//                          Per-phase maximum reactive power: Pwr200msecMax.RPx
//                          Per-phase minimum reactive power: Pwr200msecMin.RPx
//                          Per-phase maximum apparent power: Pwr200msecAppMax.AppPx
//                          Per-phase minimum apparent power: Pwr200msecAppMin.AppPx
//                      These powers are used for metering and energy computations.  The power values used 
//                      for protection are computed in Calc_Prot_Power().
//
//  CAVEATS:            None
//
//  INPUTS:             Pwr200msecSOS_Sav.Px, Cur200msFltr.Ix, VolAFE200msFltr.Vxn, Pwr200msecMax.Px,
//                      Pwr200msecMax.RPx, Pwr200msecAppMax.AppPx, Pwr200msecSOS_Sav.RPx
// 
//  OUTPUTS:            Pwr200msec.Px, Pwr200msecApp.AppPx,Pwr200msec.RPx, Pwr200msecMax.Px,
//                      Pwr200msecMax.RPx, Pwr200msecAppMax.AppPx, Pwr200msecMax.PxTS, Pwr200msecMin.PxTS,
//                      Pwr200msecMax.RPxTS, Pwr200msecMin.RPxTS
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Meter_Power(void)
{
  float temp, temp1;
  struct INTERNAL_TIME presenttime;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();


  // Compute 200msec per-phase real powers
  //  Real power is the average of the sum of the sample-by-sample product of current and voltage
  Pwr200msec.Pa = Pwr200msecSOS_Sav.Pa/960;
  Pwr200msec.Pb = Pwr200msecSOS_Sav.Pb/960;
  Pwr200msec.Pc = Pwr200msecSOS_Sav.Pc/960;
  Pwr200msec.Ptot = Pwr200msec.Pa + Pwr200msec.Pb + Pwr200msec.Pc;

  // Compute 200msec per-phase apparent powers
  //  Apparent power is the product of the rms current and rms voltage.  (the rms current and voltage are
  //    computed by taking the root-mean-square of the samples over 12 cycles)
  Pwr200msecApp.AppPa = Cur200msFltr.Ia * VolAFE200msFltr.Van;
  Pwr200msecApp.AppPb = Cur200msFltr.Ib * VolAFE200msFltr.Vbn;
  Pwr200msecApp.AppPc = Cur200msFltr.Ic * VolAFE200msFltr.Vcn;
  Pwr200msecApp.Apptot = Pwr200msecApp.AppPa + Pwr200msecApp.AppPb + Pwr200msecApp.AppPc;

  // Compute 200msec per-phase reactive powers
  //  Reactive power is the vector difference between the apparent and real power: RP = SQRT[APP^2 - REAL^2]
  //    Reactive power is NOT computed by shifting the voltage 90 degrees and computing similarly to the
  //    real power.  That approach does not include harmonics.  The vector difference includes the effects
  //    of harmonics.
  // However, the sign of the reactive power is obtained by checking the sign of the product of the voltage
  // (shifted by 90 degrees) and the current.  This is stored in Pwr200msecSOS_Sav.RPx
  temp = (Pwr200msecApp.AppPa * Pwr200msecApp.AppPa) - (Pwr200msec.Pa * Pwr200msec.Pa);
  if (temp > 0)
  {
    Pwr200msec.RPa = sqrtf(temp);
    if ( (Pwr200msecSOS_Sav.RPa < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      Pwr200msec.RPa = -1.0 * Pwr200msec.RPa;
    }
  }
  else
  {
    Pwr200msec.RPa = 0;
  }
  temp = (Pwr200msecApp.AppPb * Pwr200msecApp.AppPb) - (Pwr200msec.Pb * Pwr200msec.Pb);
  if (temp > 0)
  {
    Pwr200msec.RPb = sqrtf(temp);
    if ( (Pwr200msecSOS_Sav.RPb < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      Pwr200msec.RPb = -1.0 * Pwr200msec.RPb;
    }
  }
  else
  {
    Pwr200msec.RPb = 0;
  }
  temp = (Pwr200msecApp.AppPc * Pwr200msecApp.AppPc) - (Pwr200msec.Pc * Pwr200msec.Pc);
  if (temp > 0)
  {
    Pwr200msec.RPc = sqrtf(temp);
    if ( (Pwr200msecSOS_Sav.RPc < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      Pwr200msec.RPc = -1.0 * Pwr200msec.RPc;
    }
  }
  else
  {
    Pwr200msec.RPc = 0;
  }
  Pwr200msec.Rtot = Pwr200msec.RPa + Pwr200msec.RPb + Pwr200msec.RPc;

  // Compute mins and maxes if not frozen
  // Note, the sign of the power is not considered when mins and maxs are computed
  if (!FreezeMinMax)                                                            // *** DAH  DO WE NEED TO SUPPORT FREEZE/THAW?
  {
    temp = ((Pwr200msec.Pa < 0) ? (-Pwr200msec.Pa) : Pwr200msec.Pa);            // Get absolute values
    temp1 = ((Pwr200msecMax.Pa < 0) ? (-Pwr200msecMax.Pa) : Pwr200msecMax.Pa);
    if (temp > temp1)                                                           // Set max to input if abs
    {                                                                           //   value of input is
      Pwr200msecMax.Pa = Pwr200msec.Pa;                                         //   greater than existing
      Pwr200msecMax.PaTS = presenttime;
    }                                                                           //   max value
    temp1 = ((Pwr200msecMin.Pa < 0) ? (-Pwr200msecMin.Pa) : Pwr200msecMin.Pa);
    if (temp < temp1)                                                           // Set min to input if abs
    {                                                                           //   value of input is less
      Pwr200msecMin.Pa = Pwr200msec.Pa;                                         //   than existing min value
      Pwr200msecMin.PaTS = presenttime;
    }
    temp = ((Pwr200msec.Pb < 0) ? (-Pwr200msec.Pb) : Pwr200msec.Pb);
    temp1 = ((Pwr200msecMax.Pb < 0) ? (-Pwr200msecMax.Pb) : Pwr200msecMax.Pb);
    if (temp > temp1)
    {
      Pwr200msecMax.Pb = Pwr200msec.Pb;
      Pwr200msecMax.PbTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.Pb < 0) ? (-Pwr200msecMin.Pb) : Pwr200msecMin.Pb);
    if (temp < temp1)
    {
      Pwr200msecMin.Pb = Pwr200msec.Pb;
      Pwr200msecMin.PbTS = presenttime;
    }
    temp = ((Pwr200msec.Pc < 0) ? (-Pwr200msec.Pc) : Pwr200msec.Pc);
    temp1 = ((Pwr200msecMax.Pc < 0) ? (-Pwr200msecMax.Pc) : Pwr200msecMax.Pc);
    if (temp > temp1)
    {
      Pwr200msecMax.Pc = Pwr200msec.Pc;
      Pwr200msecMax.PcTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.Pc < 0) ? (-Pwr200msecMin.Pc) : Pwr200msecMin.Pc);
    if (temp < temp1) 
    {
      Pwr200msecMin.Pc = Pwr200msec.Pc;
      Pwr200msecMin.PcTS = presenttime;
    }
    temp = ((Pwr200msec.Ptot < 0) ? (-Pwr200msec.Ptot) : Pwr200msec.Ptot);
    temp1 = ((Pwr200msecMax.Ptot < 0) ? (-Pwr200msecMax.Ptot) : Pwr200msecMax.Ptot);
    if (temp > temp1)
    {
      Pwr200msecMax.Ptot = Pwr200msec.Ptot;
      Pwr200msecMax.PtotTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.Ptot < 0) ? (-Pwr200msecMin.Ptot) : Pwr200msecMin.Ptot);
    if (temp < temp1) 
    {
      Pwr200msecMin.Ptot = Pwr200msec.Ptot;
      Pwr200msecMin.PtotTS = presenttime;
    }

    temp = ((Pwr200msec.RPa < 0) ? (-Pwr200msec.RPa) : Pwr200msec.RPa);         // Get absolute values
    temp1 = ((Pwr200msecMax.RPa < 0) ? (-Pwr200msecMax.RPa) : Pwr200msecMax.RPa);
    if (temp > temp1)                                                           // Set max to input if abs
    {                                                                           //   value of input is
      Pwr200msecMax.RPa = Pwr200msec.RPa;                                       //   greater than existing
      Pwr200msecMax.RPaTS = presenttime;
    }                                                                           //   max value
    temp1 = ((Pwr200msecMin.RPa < 0) ? (-Pwr200msecMin.RPa) : Pwr200msecMin.RPa);
    if (temp < temp1)                                                           // Set min to input if abs
    {                                                                           //   value of input is less
      Pwr200msecMin.RPa = Pwr200msec.RPa;                                       //   than existing min value
      Pwr200msecMin.RPaTS = presenttime;
    }
    temp = ((Pwr200msec.RPb < 0) ? (-Pwr200msec.RPb) : Pwr200msec.RPb);
    temp1 = ((Pwr200msecMax.RPb < 0) ? (-Pwr200msecMax.RPb) : Pwr200msecMax.RPb);
    if (temp > temp1)
    {
      Pwr200msecMax.RPb = Pwr200msec.RPb;
      Pwr200msecMax.RPbTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.RPb < 0) ? (-Pwr200msecMin.RPb) : Pwr200msecMin.RPb);
    if (temp < temp1)
    {
      Pwr200msecMin.RPb = Pwr200msec.RPb;
      Pwr200msecMin.RPbTS = presenttime;
    }
    temp = ((Pwr200msec.RPc < 0) ? (-Pwr200msec.RPc) : Pwr200msec.RPc);
    temp1 = ((Pwr200msecMax.RPc < 0) ? (-Pwr200msecMax.RPc) : Pwr200msecMax.RPc);
    if (temp > temp1)
    {
      Pwr200msecMax.RPc = Pwr200msec.RPc;
      Pwr200msecMax.RPcTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.RPc < 0) ? (-Pwr200msecMin.RPc) : Pwr200msecMin.RPc);
    if (temp < temp1) 
    {
      Pwr200msecMin.RPc = Pwr200msec.RPc;
      Pwr200msecMin.RPcTS = presenttime;
    }
    temp = ((Pwr200msec.Rtot < 0) ? (-Pwr200msec.Rtot) : Pwr200msec.Rtot);
    temp1 = ((Pwr200msecMax.Rtot < 0) ? (-Pwr200msecMax.Rtot) : Pwr200msecMax.Rtot);
    if (temp > temp1)
    {
      Pwr200msecMax.Rtot = Pwr200msec.Rtot;
      Pwr200msecMax.RtotTS = presenttime;
    }
    temp1 = ((Pwr200msecMin.Rtot < 0) ? (-Pwr200msecMin.Rtot) : Pwr200msecMin.Rtot);
    if (temp < temp1) 
    {
      Pwr200msecMin.Rtot = Pwr200msec.Rtot;
      Pwr200msecMin.RtotTS = presenttime;
    }
    // Apparent powers are always positive
    // Check for max
    if ( (Pwr200msecApp.AppPa > Pwr200msecAppMax.AppPa) || (isnan(Pwr200msecAppMax.AppPa)) )
    {
      Pwr200msecAppMax.AppPa = Pwr200msecApp.AppPa;
      Pwr200msecAppMax.AppPaTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for min
    if ( (Pwr200msecApp.AppPa < Pwr200msecAppMin.AppPa) || (isnan(Pwr200msecAppMin.AppPa)) )
    {
      Pwr200msecAppMin.AppPa = Pwr200msecApp.AppPa;
      Pwr200msecAppMin.AppPaTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for max
    if ( (Pwr200msecApp.AppPb > Pwr200msecAppMax.AppPb) || (isnan(Pwr200msecAppMax.AppPb)) )
    {
      Pwr200msecAppMax.AppPb = Pwr200msecApp.AppPb;
      Pwr200msecAppMax.AppPbTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for min
    if ( (Pwr200msecApp.AppPb < Pwr200msecAppMin.AppPb) || (isnan(Pwr200msecAppMin.AppPb)) )
    {
      Pwr200msecAppMin.AppPb = Pwr200msecApp.AppPb;
      Pwr200msecAppMin.AppPbTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for max
    if ( (Pwr200msecApp.AppPc > Pwr200msecAppMax.AppPc) || (isnan(Pwr200msecAppMax.AppPc)) )
    {
      Pwr200msecAppMax.AppPc = Pwr200msecApp.AppPc;
      Pwr200msecAppMax.AppPcTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for min
    if ( (Pwr200msecApp.AppPc < Pwr200msecAppMin.AppPc) || (isnan(Pwr200msecAppMin.AppPc)) )
    {
      Pwr200msecAppMin.AppPc = Pwr200msecApp.AppPc;
      Pwr200msecAppMin.AppPcTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for max
    if ( (Pwr200msecApp.Apptot > Pwr200msecAppMax.Apptot) || (isnan(Pwr200msecAppMax.Apptot)) )
    {
      Pwr200msecAppMax.Apptot = Pwr200msecApp.Apptot;
      Pwr200msecAppMax.ApptotTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    // Check for min
    if ( (Pwr200msecApp.Apptot < Pwr200msecAppMin.Apptot) || (isnan(Pwr200msecAppMin.Apptot)) )
    {
      Pwr200msecAppMin.Apptot = Pwr200msecApp.Apptot;
      Pwr200msecAppMin.ApptotTS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
  }

/*  Pwr200msec.Pa = -9e4;   // *** ADDED FOR TEST 160510
  Pwr200msec.Pb = -9e4;
  Pwr200msec.Pc = -9e4;
  Pwr200msec.RPa = -9e4;
  Pwr200msec.RPb = -9e4;
  Pwr200msec.RPc = -9e4;
  Pwr200msecApp.AppPa = 9E4;
  Pwr200msecApp.AppPb = 9E4;
  Pwr200msecApp.AppPc = 9E4;   */

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Meter_Power()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Prot_Power()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Power for Protection
//
//  MECHANICS:          This subroutine computes the real, reactive, and apparent power values.  It is
//                      called every cycle, after the voltage and current values have been computed.
//                      Each power value is computed as a floating-point number.  The values are generated
//                      over a one-cycle window, using the non-filtered sample values.  The following powers
//                      are computed:
//                          Per-phase real power: PwrOneCyc.Px
//                          Per-phase reactive power: PwrOneCyc.RPx
//                          Per-phase apparent power: PwrOneCycApp.AppPx
//                          Per-phase maximum real power: PwrOneCycMax.Px
//                          Per-phase minimum real power: PwrOneCycMin.Px
//                          Per-phase maximum reactive power: PwrOneCycMax.RPx
//                          Per-phase minimum reactive power: PwrOneCycMin.RPx
//                          Per-phase maximum apparent power: PwrOneCycAppMax.AppPx
//                          Per-phase minimum apparent power: PwrOneCycAppMin.AppPx
//                      These powers are used for protection computations.  The power values used for
//                      metering and energy computations are computed in Calc_Meter_Power().
//
//  CAVEATS:            None
//
//  INPUTS:             Pwr200msecSOS_Sav.Px, Cur200msec.Ix, Pwr200msecMax.Px,            *** DAH  FIX THIS
//                      Pwr200msecMax.RPx, Pwr200msecAppMax.AppPx, Pwr200msecSOS_Sav.RPx
// 
//  OUTPUTS:            Pwr200msec.Px, Pwr200msecApp.AppPx,Pwr200msec.RPx, Pwr200msecMax.Px,
//                      Pwr200msecMax.RPx, Pwr200msecAppMax.AppPx
//
//  ALTERS:             None
// 
//  CALLS:              sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Prot_Power(void)
{
  float temp, temp1;

  // Compute one-cycle per-phase real powers
  //  Real power is the average of the sum of the sample-by-sample product of current and voltage
  PwrOneCyc.Pa = PwrOneCycSOS_Sav.Pa/80;
  PwrOneCyc.Pb = PwrOneCycSOS_Sav.Pb/80;
  PwrOneCyc.Pc = PwrOneCycSOS_Sav.Pc/80;

  // Compute one-cycle per-phase apparent powers
  //  Apparent power is the product of the rms current and rms voltage.  (the rms current and voltage are
  //    computed by taking the root-mean-square of the samples over 12 cycles)
  PwrOneCycApp.AppPa = CurOneCyc.Ia * VolAFEOneCyc.Van;
  PwrOneCycApp.AppPb = CurOneCyc.Ib * VolAFEOneCyc.Vbn;
  PwrOneCycApp.AppPc = CurOneCyc.Ic * VolAFEOneCyc.Vcn;

  // Compute one-cycle per-phase reactive powers
  //  Reactive power is the vector difference between the apparent and real power: RP = SQRT[APP^2 - REAL^2]
  //    Reactive power is NOT computed by shifting the voltage 90 degrees and computing similarly to the
  //    real power.  That approach does not include harmonics.  The vector difference includes the effects
  //    of harmonics.
  // However, the sign of the reactive power is obtained by checking the sign of the product of the voltage
  // (shifted by 90 degrees) and the current.  This is stored in Pwr200msecSOS_Sav.RPx
  temp = (PwrOneCycApp.AppPa * PwrOneCycApp.AppPa) - (PwrOneCyc.Pa * PwrOneCyc.Pa);
  if (temp > 0)
  {
    PwrOneCyc.RPa = sqrtf(temp);
    if ( (PwrOneCycSOS_Sav.RPa < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      PwrOneCyc.RPa = -1.0 * PwrOneCyc.RPa;
    }
  }
  else
  {
    PwrOneCyc.RPa = 0;
  }
  temp = (PwrOneCycApp.AppPb * PwrOneCycApp.AppPb) - (PwrOneCyc.Pb * PwrOneCyc.Pb);
  if (temp > 0)
  {
    PwrOneCyc.RPb = sqrtf(temp);
    if ( (PwrOneCycSOS_Sav.RPb < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      PwrOneCyc.RPb = -1.0 * PwrOneCyc.RPb;
    }
  }
  else
  {
    PwrOneCyc.RPb = 0;
  }
  temp = (PwrOneCycApp.AppPc * PwrOneCycApp.AppPc) - (PwrOneCyc.Pc * PwrOneCyc.Pc);
  if (temp > 0)
  {
    PwrOneCyc.RPc = sqrtf(temp);
    if ( (PwrOneCycSOS_Sav.RPc < 0) /* || (Setpoint == ??) */ )   // *** DAH  ADD CODE TO CHECK SIGN CONVENTION SETPOINT
    {
      PwrOneCyc.RPc = -1.0 * PwrOneCyc.RPc;
    }
  }
  else
  {
    PwrOneCyc.RPc = 0;
  }

  // Compute mins and maxes if not frozen
  // Note, the sign of the power is not considered when mins and maxs are computed
   if (!FreezeMinMax)                                                           // *** DAH  DO WE NEED TO SUPPORT FREEZE/THAW?
  {
    temp = ((PwrOneCyc.Pa < 0) ? (-PwrOneCyc.Pa) : PwrOneCyc.Pa);               // Get absolute values
    temp1 = ((PwrOneCycMax.Pa < 0) ? (-PwrOneCycMax.Pa) : PwrOneCycMax.Pa);
    if (temp > temp1)                                                           // Set max to input if abs
    {                                                                           //   value of input is
      PwrOneCycMax.Pa = PwrOneCyc.Pa;                                           //   greater than existing
    }                                                                           //   max value
    temp = ((PwrOneCyc.Pa < 0) ? (-PwrOneCyc.Pa) : PwrOneCyc.Pa);               // Get absolute values
    temp1 = ((PwrOneCycMin.Pa < 0) ? (-PwrOneCycMin.Pa) : PwrOneCycMin.Pa);
    if (temp < temp1)                                                           // Set min to input if abs
    {                                                                           //   value of input is less
      PwrOneCycMin.Pa = PwrOneCyc.Pa;                                           //   than existing min value
    }
    temp = ((PwrOneCyc.Pb < 0) ? (-PwrOneCyc.Pb) : PwrOneCyc.Pb);
    temp1 = ((PwrOneCycMax.Pb < 0) ? (-PwrOneCycMax.Pb) : PwrOneCycMax.Pb);
    if (temp > temp1)
    {
      PwrOneCycMax.Pb = PwrOneCyc.Pb;
    }
    temp = ((PwrOneCyc.Pb < 0) ? (-PwrOneCyc.Pb) : PwrOneCyc.Pb);
    temp1 = ((PwrOneCycMin.Pb < 0) ? (-PwrOneCycMin.Pb) : PwrOneCycMin.Pb);
    if (temp < temp1)
    {
      PwrOneCycMin.Pb = PwrOneCyc.Pb;
    }
    temp = ((PwrOneCyc.Pc < 0) ? (-PwrOneCyc.Pc) : PwrOneCyc.Pc);
    temp1 = ((PwrOneCycMax.Pc < 0) ? (-PwrOneCycMax.Pc) : PwrOneCycMax.Pc);
    if (temp > temp1)
    {
      PwrOneCycMax.Pc = PwrOneCyc.Pc;
    }
    temp = ((PwrOneCyc.Pc < 0) ? (-PwrOneCyc.Pc) : PwrOneCyc.Pc);
    temp1 = ((PwrOneCycMin.Pc < 0) ? (-PwrOneCycMin.Pc) : PwrOneCycMin.Pc);
    if (temp < temp1) 
    {
      PwrOneCycMin.Pc = PwrOneCyc.Pc;
    }

    temp = ((PwrOneCyc.RPa < 0) ? (-PwrOneCyc.RPa) : PwrOneCyc.RPa);            // Get absolute values
    temp1 = ((PwrOneCycMax.RPa < 0) ? (-PwrOneCycMax.RPa) : PwrOneCycMax.RPa);
    if (temp > temp1)                                                           // Set max to input if abs
    {                                                                           //   value of input is
      PwrOneCycMax.RPa = PwrOneCyc.RPa;                                         //   greater than existing
    }                                                                           //   max value
    temp = ((PwrOneCyc.RPa < 0) ? (-PwrOneCyc.RPa) : PwrOneCyc.RPa);            // Get absolute values
    temp1 = ((PwrOneCycMin.RPa < 0) ? (-PwrOneCycMin.RPa) : PwrOneCycMin.RPa);
    if (temp < temp1)                                                           // Set min to input if abs
    {                                                                           //   value of input is less
      PwrOneCycMin.RPa = PwrOneCyc.RPa;                                         //   than existing min value
    }
    temp = ((PwrOneCyc.RPb < 0) ? (-PwrOneCyc.RPb) : PwrOneCyc.RPb);
    temp1 = ((PwrOneCycMax.RPb < 0) ? (-PwrOneCycMax.RPb) : PwrOneCycMax.RPb);
    if (temp > temp1)
    {
      PwrOneCycMax.RPb = PwrOneCyc.RPb;
    }
    temp = ((PwrOneCyc.RPb < 0) ? (-PwrOneCyc.RPb) : PwrOneCyc.RPb);
    temp1 = ((PwrOneCycMin.RPb < 0) ? (-PwrOneCycMin.RPb) : PwrOneCycMin.RPb);
    if (temp < temp1)
    {
      PwrOneCycMin.RPb = PwrOneCyc.RPb;
    }
    temp = ((PwrOneCyc.RPc < 0) ? (-PwrOneCyc.RPc) : PwrOneCyc.RPc);
    temp1 = ((PwrOneCycMax.RPc < 0) ? (-PwrOneCycMax.RPc) : PwrOneCycMax.RPc);
    if (temp > temp1)
    {
      PwrOneCycMax.RPc = PwrOneCyc.RPc;
    }
    temp = ((PwrOneCyc.RPc < 0) ? (-PwrOneCyc.RPc) : PwrOneCyc.RPc);
    temp1 = ((PwrOneCycMin.RPc < 0) ? (-PwrOneCycMin.RPc) : PwrOneCycMin.RPc);
    if (temp < temp1) 
    {
      PwrOneCycMin.RPc = PwrOneCyc.RPc;
    }
    // Apparent powers are always positive
    PwrOneCycAppMax.AppPa = ((PwrOneCycApp.AppPa > PwrOneCycAppMax.AppPa) ? PwrOneCycApp.AppPa : PwrOneCycAppMax.AppPa);
    PwrOneCycAppMin.AppPa = ((PwrOneCycApp.AppPa < PwrOneCycAppMin.AppPa) ? PwrOneCycApp.AppPa : PwrOneCycAppMin.AppPa);
    PwrOneCycAppMax.AppPb = ((PwrOneCycApp.AppPb > PwrOneCycAppMax.AppPb) ? PwrOneCycApp.AppPb : PwrOneCycAppMax.AppPb);
    PwrOneCycAppMin.AppPb = ((PwrOneCycApp.AppPb < PwrOneCycAppMin.AppPb) ? PwrOneCycApp.AppPb : PwrOneCycAppMin.AppPb);
    PwrOneCycAppMax.AppPc = ((PwrOneCycApp.AppPc > PwrOneCycAppMax.AppPc) ? PwrOneCycApp.AppPc : PwrOneCycAppMax.AppPc);
    PwrOneCycAppMin.AppPc = ((PwrOneCycApp.AppPc < PwrOneCycAppMin.AppPc) ? PwrOneCycApp.AppPc : PwrOneCycAppMin.AppPc);
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Prot_Power()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Energy()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Energy
//
//  MECHANICS:          This subroutine computes the energy for each phase and the sum of all of the phases.
//                      Energy is computed over 200msec (12 cycles at 60Hz).  It is called each 200msec
//                      anniversary.  The updated energy values are then written to FRAM
//
//  CAVEATS:            This MUST be called after Calc_Power() because it uses the 200msec power values to
//                      compute the energy
//
//  INPUTS:             Pwr200msec.Pa, Pwr200msec.Pb, Pwr200msec.Pc
//                      AFEcal.gain[] - the gain scaling coefficient for the channel
//                      AFEcal.offset[] - the offset scaling coefficient for the channel
// 
//  OUTPUTS:            ResidualPha.xxx, ResidualPhb.xxx, ResidualPhc.xxx, ResidualAll.xxx, EnergyPha.xxx,
//                      EnergyPhb.xxx, EnergyPhc.xxx, EngyDmnd[1].TotFwdWHr, EngyDmnd[1].TotRevWHr,
//                      EngyDmnd[1].TotLagVarHr, EngyDmnd[1].TotLeadVarHr, EngyDmnd[1].TotVAHr, TotWHr,
//                      NetWHr, TotVarHr, NetVarHr
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_WriteEnergy
//
//  EXECUTION TIME:     Measured execution time on 160510 (Rev 00.13 code).  Hard-coded Pwr200msec.Pa, Pb,
//                      Pc, RPa, RPb, RPc to -9E4, and Pwr200msecApp.Pa, Pb, Pc to 9E4 to generate the
//                      worst-case condition.  Execution time = 6.35usec without being interrupted  *** DAH RECALCULATE SINCE FRAM_WriteEnergy() ADDED  220831 SHOULD BE ~271USEC
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Energy(void)
{
  float Energy200msec;
  uint32_t temp;

  // Update the forward and reverse active energy registers with the energy accumulated over the past
  //   200msec from the three phases.  Only update if the current is less than 125% of the frame rating and
  //   greater than 0.15% of the minimum rated current        *** DAH CHECK THESE LIMITS
  // The range is 3A * 100 * .2 = 60Wsec min to about 7000A * (277 + 25%) * .2 = 485KWsec x 3 phases = 1.455MWsec
  // Max/Min is about 24000 or 2^15.  This is still significantly less than the mantissa of a floating point
  //   number, which is 22 bits, so we should be ok using a floating point for the residual energy
  if (Pwr200msec.Pa >= 0)               // Positive power - forward energy register
  {
    Energy200msec = Pwr200msec.Pa * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPha.FwdWHr += Energy200msec;
      ResidualAll.FwdWHr += Energy200msec;
    }
  }
  else                                  // Negative power - reverse energy register
  {
    Energy200msec = -(Pwr200msec.Pa * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPha.RevWHr += Energy200msec;
      ResidualAll.RevWHr += Energy200msec;
    }
  }
  if (Pwr200msec.Pb >= 0)               // Positive power - forward energy register
  {
    Energy200msec = Pwr200msec.Pb * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhb.FwdWHr += Energy200msec;
      ResidualAll.FwdWHr += Energy200msec;
    }
  }
  else                                  // Negative power - reverse energy register
  {
    Energy200msec = -(Pwr200msec.Pb * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhb.RevWHr += Energy200msec;
      ResidualAll.RevWHr += Energy200msec;
    }
  }
  if (Pwr200msec.Pc >= 0)               // Positive power - forward energy register
  {
    Energy200msec = Pwr200msec.Pc * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhc.FwdWHr += Energy200msec;
      ResidualAll.FwdWHr += Energy200msec;
    }
  }
  else                                  // Negative power - reverse energy register
  {
    Energy200msec = -(Pwr200msec.Pc * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhc.RevWHr += Energy200msec;
      ResidualAll.RevWHr += Energy200msec;
    }
  }
  // Max residual per-phase is 1.455MWsec/3600sec/hr = 404WHr x 3 phases = 1212WHr
  // Increment energy register with any energy greater than 1 WHr, and remove this energy from the residual
  // Note, ok to use uint32 since number is always positive and significantly less than 2^32
  temp = (uint32_t)(ResidualAll.FwdWHr);    // Casting drops the fractional portion - it does not round
  EngyDmnd[1].TotFwdWHr += temp;
  ResidualAll.FwdWHr -= temp;
  if (ResidualAll.FwdWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualAll.FwdWHr = 0;
  }
  temp = (uint32_t)(ResidualAll.RevWHr);    // Casting drops the fractional portion - it does not round
  EngyDmnd[1].TotRevWHr += temp;
  ResidualAll.RevWHr -= temp;
  if (ResidualAll.RevWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualAll.RevWHr = 0;
  }
  TotWHr = EngyDmnd[1].TotFwdWHr + EngyDmnd[1].TotRevWHr;
  NetWHr = EngyDmnd[1].TotFwdWHr - EngyDmnd[1].TotRevWHr;

  temp = (uint32_t)(ResidualPha.FwdWHr);    // Casting drops the fractional portion - it does not round
  EnergyPha.FwdWHr += temp;
  ResidualPha.FwdWHr -= temp;
  if (ResidualPha.FwdWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPha.FwdWHr = 0;
  }
  temp = (uint32_t)(ResidualPha.RevWHr);    // Casting drops the fractional portion - it does not round
  EnergyPha.RevWHr += temp;
  ResidualPha.RevWHr -= temp;
  if (ResidualPha.RevWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPha.RevWHr = 0;
  }
  temp = (uint32_t)(ResidualPhb.FwdWHr);    // Casting drops the fractional portion - it does not round
  EnergyPhb.FwdWHr += temp;
  ResidualPhb.FwdWHr -= temp;
  if (ResidualPhb.FwdWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhb.FwdWHr = 0;
  }
  temp = (uint32_t)(ResidualPhb.RevWHr);    // Casting drops the fractional portion - it does not round
  EnergyPhb.RevWHr += temp;
  ResidualPhb.RevWHr -= temp;
  if (ResidualPhb.RevWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhb.RevWHr = 0;
  }
  temp = (uint32_t)(ResidualPhc.FwdWHr);    // Casting drops the fractional portion - it does not round
  EnergyPhc.FwdWHr += temp;
  ResidualPhc.FwdWHr -= temp;
  if (ResidualPhc.FwdWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhc.FwdWHr = 0;
  }
  temp = (uint32_t)(ResidualPhc.RevWHr);    // Casting drops the fractional portion - it does not round
  EnergyPhc.RevWHr += temp;
  ResidualPhc.RevWHr -= temp;
  if (ResidualPhc.RevWHr < 0)               // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhc.RevWHr = 0;
  }

  // Repeat this process for the lagging and leading reactive energy registers
  if (Pwr200msec.RPa >= 0)              // Positive power - lagging energy register
  {
    Energy200msec = Pwr200msec.RPa * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPha.LagVarHr += Energy200msec;
      ResidualAll.LagVarHr += Energy200msec;
    }
  }
  else                                  // Negative power - leading energy register
  {
    Energy200msec = -(Pwr200msec.RPa * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPha.LeadVarHr += Energy200msec;
      ResidualAll.LeadVarHr += Energy200msec;
    }
  }
  if (Pwr200msec.RPb >= 0)              // Positive power - lagging energy register
  {
    Energy200msec = Pwr200msec.RPb * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhb.LagVarHr += Energy200msec;
      ResidualAll.LagVarHr += Energy200msec;
    }
  }
  else                                  // Negative power - leading energy register
  {
    Energy200msec = -(Pwr200msec.RPb * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhb.LeadVarHr += Energy200msec;
      ResidualAll.LeadVarHr += Energy200msec;
    }
  }
  if (Pwr200msec.RPc >= 0)              // Positive power - lagging energy register
  {
    Energy200msec = Pwr200msec.RPc * MS200_TO_HRS;   // Compute energy in watt-hours for the 200msec period
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhc.LagVarHr += Energy200msec;
      ResidualAll.LagVarHr += Energy200msec;
    }
  }
  else                                  // Negative power - leading energy register
  {
    Energy200msec = -(Pwr200msec.RPc * MS200_TO_HRS);
    if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
    {
      ResidualPhc.LeadVarHr += Energy200msec;
      ResidualAll.LeadVarHr += Energy200msec;
    }
  }
  temp = (uint32_t)(ResidualAll.LagVarHr);  // Casting drops the fractional portion - it does not round
  EngyDmnd[1].TotLagVarHr += temp;
  ResidualAll.LagVarHr -= temp;
  if (ResidualAll.LagVarHr < 0)             // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualAll.LagVarHr = 0;
  }
  temp = (uint32_t)(ResidualAll.LeadVarHr); // Casting drops the fractional portion - it does not round
  EngyDmnd[1].TotLeadVarHr += temp;
  ResidualAll.LeadVarHr -= temp;
  if (ResidualAll.LeadVarHr < 0)            // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualAll.LeadVarHr = 0;
  }
  TotVarHr = EngyDmnd[1].TotLeadVarHr + EngyDmnd[1].TotLagVarHr;
  NetVarHr = EngyDmnd[1].TotLeadVarHr - EngyDmnd[1].TotLagVarHr;

  temp = (uint32_t)(ResidualPha.LagVarHr);  // Casting drops the fractional portion - it does not round
  EnergyPha.LagVarHr += temp;
  ResidualPha.LagVarHr -= temp;
  if (ResidualPha.LagVarHr < 0)             // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPha.LagVarHr = 0;
  }
  temp = (uint32_t)(ResidualPha.LeadVarHr); // Casting drops the fractional portion - it does not round
  EnergyPha.LeadVarHr += temp;
  ResidualPha.LeadVarHr -= temp;
  if (ResidualPha.LeadVarHr < 0)            // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPha.LeadVarHr = 0;
  }
  temp = (uint32_t)(ResidualPhb.LagVarHr);  // Casting drops the fractional portion - it does not round
  EnergyPhb.LagVarHr += temp;
  ResidualPhb.LagVarHr -= temp;
  if (ResidualPhb.LagVarHr < 0)             // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhb.LagVarHr = 0;
  }
  temp = (uint32_t)(ResidualPhb.LeadVarHr); // Casting drops the fractional portion - it does not round
  EnergyPhb.LeadVarHr += temp;
  ResidualPhb.LeadVarHr -= temp;
  if (ResidualPhb.LeadVarHr < 0)            // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhb.LeadVarHr = 0;
  }
  temp = (uint32_t)(ResidualPhc.LagVarHr);  // Casting drops the fractional portion - it does not round
  EnergyPhc.LagVarHr += temp;
  ResidualPhc.LagVarHr -= temp;
  if (ResidualPhc.LagVarHr < 0)             // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhc.LagVarHr = 0;
  }
  temp = (uint32_t)(ResidualPhc.LeadVarHr); // Casting drops the fractional portion - it does not round
  EnergyPhc.LeadVarHr += temp;
  ResidualPhc.LeadVarHr -= temp;
  if (ResidualPhc.LeadVarHr < 0)            // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhc.LeadVarHr = 0;
  }

  // Repeat this process for the apparent energy register.  Apparent power is always positive
  Energy200msec = Pwr200msecApp.AppPa * MS200_TO_HRS;
  if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
  {
    ResidualPha.VAHr += Energy200msec;
    ResidualAll.VAHr += Energy200msec;
  }
  Energy200msec = Pwr200msecApp.AppPb * MS200_TO_HRS;
  if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
  {
    ResidualPhb.VAHr += Energy200msec;
    ResidualAll.VAHr += Energy200msec;
  }
  Energy200msec = Pwr200msecApp.AppPc * MS200_TO_HRS;
  if ((Energy200msec < Max_Metering_Energy) && (Energy200msec > Min_Metering_Energy))
  {
    ResidualPhc.VAHr += Energy200msec;
    ResidualAll.VAHr += Energy200msec;
  }
  temp = (uint32_t)(ResidualAll.VAHr);      // Casting drops the fractional portion - it does not round
  EngyDmnd[1].TotVAHr += temp;
  ResidualAll.VAHr -= temp;
  if (ResidualAll.VAHr < 0)                 // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualAll.VAHr = 0;
  }
  temp = (uint32_t)(ResidualPha.VAHr);      // Casting drops the fractional portion - it does not round
  EnergyPha.VAHr += temp;
  ResidualPha.VAHr -= temp;
  if (ResidualPha.VAHr < 0)                 // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPha.VAHr = 0;
  }
  temp = (uint32_t)(ResidualPhb.VAHr);      // Casting drops the fractional portion - it does not round
  EnergyPhb.VAHr += temp;
  ResidualPhb.VAHr -= temp;
  if (ResidualPhb.VAHr < 0)                 // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhb.VAHr = 0;
  }
  temp = (uint32_t)(ResidualPhc.VAHr);      // Casting drops the fractional portion - it does not round
  EnergyPhc.VAHr += temp;
  ResidualPhc.VAHr -= temp;
  if (ResidualPhc.VAHr < 0)                 // This shouldn't be necessary, but clamp at zero in case this
  {                                         //   is negative due to conversion errors
    ResidualPhc.VAHr = 0;
  }
  FRAM_WriteEnergy(DEV_FRAM2);              // Write the energy values into FRAM   264usec *** DAH  MUST BE ADDED TO THE EXECUTION TIME FOR THIS SUBROUTINE
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Energy()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ManageSPI1Flags()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Manage SPI1 Request Flags
//
//  MECHANICS:          This subroutine handles the SPI1 request flags that are not reset by the
//                      subroutine that set them:
//                          S1F_DMND_ERASE          demand sector erase request
//                          S1F_TRIP_WF_WR          trip waveform write request
//                          S1F_TRIP_WF_ERASE       trip waveform block 1 erase request
//                          S1F_TRIP_WF_ERASE1      trip waveform block 2 erase request
//                          S1F_ALARM_WF_WR         alarm waveform write request
//                          S1F_ALARM_WF_ERASE      alarm waveform block 1 erase request
//                          S1F_ALARM_WF_ERASE1     alarm waveform block 2 erase request
//                          S1F_EXT_WF_WR           extended-capture waveform write request
//                          S1F_EXT_WF_ERASE        extended-capture waveform block 1 erase request
//                          S1F_EXT_WF_ERASE1       extended-capture waveform block 2 erase request
//                          S1F_CHK_CAL             check cal constants request
//                          S1F_RD_WF               proc-proc comms waveform read
//                      For the demand sector erase request, the request is set in initialization (under
//                      certain conditions), and so we need a place to check for its completion.
//
//  CAVEATS:            None
//
//  INPUTS:             SPI1Flash.Ack
// 
//  OUTPUTS:            SPI1Flash.Req
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 230320: 855nsec worst-case if all flags set (no interrupts during the
//                      subroutine's execution)
//
//------------------------------------------------------------------------------------------------------------

void ManageSPI1Flags(void)
{
  if (SPI1Flash.Ack & S1F_DMND_ERASE)                   // If the demand sector erase has been completed
  {                                                     //   (request has been handled), clear the request
    SPI1Flash.Req &= (uint32_t)(~S1F_DMND_ERASE);       //   flag
  }
  // If trip waveform write has been completed (request has been handled), set the first block erase request
  //   flag.  Also make sure alarm and extended capture are cleared.  It is too late to do a capture
  // Note, don't clear the trip waveform write request flag.  Clearing this flag causes the Ack flag to be
  //   cleared, and it is checked in EventManager().  The request is cleared in EventManager()
  if (SPI1Flash.Ack & S1F_TRIP_WF_WR)                   
  {
    SPI1Flash.Req |= S1F_TRIP_WF_ERASE;
    SPI1Flash.Req &= (uint32_t)(~S1F_ALARM_WF_WR);      
    SPI1Flash.Req &= (uint32_t)(~S1F_EXT_WF_WR);   
  }
  if (SPI1Flash.Ack & S1F_TRIP_WF_ERASE)                // If trip waveform first block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_TRIP_WF_ERASE);    //   request flag and set the second block erase
    SPI1Flash.Req |= S1F_TRIP_WF_ERASE1;                //   request flag
  }
  if (SPI1Flash.Ack & S1F_TRIP_WF_ERASE1)               // If trip waveform second block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_TRIP_WF_ERASE1);   //   request flag
  }
  // If alarm waveform write has been completed (request has been handled), set the first block erase
  //   request flag.  Also make sure extended capture is cleared.  It is too late to do a capture
  // Note, don't clear the alarm waveform write request flag.  Clearing this flag causes the Ack flag to be
  //   cleared, and it is checked in EventManager().  The request is cleared in EventManager()
  if (SPI1Flash.Ack & S1F_ALARM_WF_WR)
  {
    SPI1Flash.Req |= S1F_ALARM_WF_ERASE;
    SPI1Flash.Req &= (uint32_t)(~S1F_EXT_WF_WR);
  }
  if (SPI1Flash.Ack & S1F_ALARM_WF_ERASE)               // If alarm waveform first block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_ALARM_WF_ERASE);   //   request flag and set the second block erase
    SPI1Flash.Req |= S1F_ALARM_WF_ERASE1;               //   request flag
  }
  if (SPI1Flash.Ack & S1F_ALARM_WF_ERASE1)              // If alarm waveform second block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_ALARM_WF_ERASE1);  //   request flag
  }
  if (SPI1Flash.Ack & S1F_CHK_CAL)                      // If Flash cal constants check has been completed
  {                                                     //   (request has been handled), clear the request
    SPI1Flash.Req &= (uint32_t)(~S1F_CHK_CAL);          //   flag
  }
  // If extended waveform write has been completed (request has been handled), set the first block erase
  //   request flag
  // Note, don't clear the extended waveform write request flag.  Clearing this flag causes the Ack flag to
  //   be cleared, and it is checked in EventManager().  The request is cleared in EventManager()
  if (SPI1Flash.Ack & S1F_EXT_WF_WR)
  {
    SPI1Flash.Req |= S1F_EXT_WF_ERASE;
  }
  if (SPI1Flash.Ack & S1F_EXT_WF_ERASE)                 // If trip waveform first block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_EXT_WF_ERASE);     //   request flag and set the second block erase
    SPI1Flash.Req |= S1F_EXT_WF_ERASE1;                 //   request flag
  }
  if (SPI1Flash.Ack & S1F_EXT_WF_ERASE1)                // If trip waveform second block erase has been
  {                                                     //   completed (request has been handled), clear the
    SPI1Flash.Req &= (uint32_t)(~S1F_EXT_WF_ERASE1);    //   request flag
  }

  // If a capture is in progress, set the corresponding request to write the waveform to Flash
  if (Trip_WF_Capture.InProg)
  {
    SPI1Flash.Req |= S1F_TRIP_WF_WR;
  }
  else if (Alarm_WF_Capture.InProg)
  {
    SPI1Flash.Req |= S1F_ALARM_WF_WR;
  }
  else if (Ext_WF_Capture.InProg)
  {
    SPI1Flash.Req |= S1F_EXT_WF_WR;
  }

  // If a waveform read has been completed for a proc-proc comms delayed read, clear the request and set
  //   the flag to generate the response
  if (SPI1Flash.Ack & S1F_RD_WF)                        // If a waveform read has been completed for a
  {                                                     //   proc-proc comms delayed read, clear the request
    SPI1Flash.Req &= (uint32_t)(~S1F_RD_WF);            //   flag and set the flag to generate a response
    DPComm.Flags |= GEN_WRITERESP;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ManageSPI1Flags()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ResetEnergy()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Reset Energy
//
//  MECHANICS:          This subroutine resets the energy registers.
//
//  CAVEATS:            None
//
//  INPUTS:             None
// 
//  OUTPUTS:            ResidualPha.xxx, ResidualPhb.xxx, ResidualPhc.xxx, ResidualAll.xxx, EnergyPha.xxx,
//                      EnergyPhb.xxx, EnergyPhc.xxx, EngyDmnd[1].TotFwdWHr, EngyDmnd[1].TotRevWHr,
//                      EngyDmnd[1].TotLagVarHr, EngyDmnd[1].TotLeadVarHr, EngyDmnd[1].TotVAHr
//
//  ALTERS:             None
// 
//  CALLS:              None
//
//  EXECUTION TIME:     
// 
//------------------------------------------------------------------------------------------------------------

void ResetEnergy(void)
{
  EnergyPha.FwdWHr = 0;
  EnergyPha.RevWHr = 0;
  EnergyPha.LagVarHr = 0;
  EnergyPha.LeadVarHr = 0;
  EnergyPha.VAHr = 0;
  ResidualPha.FwdWHr = 0;
  ResidualPha.RevWHr = 0;
  ResidualPha.LagVarHr = 0;
  ResidualPha.LeadVarHr = 0;
  ResidualPha.VAHr = 0;
  EnergyPhb.FwdWHr = 0;
  EnergyPhb.RevWHr = 0;
  EnergyPhb.LagVarHr = 0;
  EnergyPhb.LeadVarHr = 0;
  EnergyPhb.VAHr = 0;
  ResidualPhb.FwdWHr = 0;
  ResidualPhb.RevWHr = 0;
  ResidualPhb.LagVarHr = 0;
  ResidualPhb.LeadVarHr = 0;
  ResidualPhb.VAHr = 0;
  EnergyPhc.FwdWHr = 0;
  EnergyPhc.RevWHr = 0;
  EnergyPhc.LagVarHr = 0;
  EnergyPhc.LeadVarHr = 0;
  EnergyPhc.VAHr = 0;
  ResidualPhc.FwdWHr = 0;
  ResidualPhc.RevWHr = 0;
  ResidualPhc.LagVarHr = 0;
  ResidualPhc.LeadVarHr = 0;
  ResidualPhc.VAHr = 0;
  EngyDmnd[1].TotFwdWHr = 0;
  EngyDmnd[1].TotRevWHr = 0;
  EngyDmnd[1].TotLagVarHr = 0;
  EngyDmnd[1].TotLeadVarHr = 0;
  EngyDmnd[1].TotVAHr = 0;
  ResidualAll.FwdWHr = 0;
  ResidualAll.RevWHr = 0;
  ResidualAll.LagVarHr = 0;
  ResidualAll.LeadVarHr = 0;
  ResidualAll.VAHr = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ResetEnergy()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ADC_Process_Samples()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process ADC Samples
//
//  MECHANICS:          This subroutine processes the samples from the DMA operation used to read the .
//                      simultaneous ADC1/ADC2 conversions from the STM32 internal ADC.
//                      The ADC readings (ten total) are received as five 32-bit words (ADC2:ADC1).
//                      The 24-bit ADC sample data is extracted from the reading and stored in
//                      floating point format in ADC_samples[].  The 1.25VDC offset is subtracted from
//                      each sample.
//                                                                  ADC2                  ADC1
//                             i    ADC_single_capture[i]          ms word              ls word        
//                             0                             Va_Line - ADC2_IN14     Ia - ADC1_IN10    
//                             1                             Vb_Line - ADC2_IN15     Ib - ADC1_IN11    
//                             2                             Vc_Line - ADC2_IN8      Ic - ADC1_IN12    
//                             3                             1.25Vref - ADC2_IN9     In - ADC1_IN13    
//                             4                             Therm Mem - ADC2_IN3    Vn_Line - ADC1_IN1
//
//                             i     ADC_samples[i] (float value)
//                             0               Ia   
//                             1               Ib   
//                             2               Ic   
//                             3               In   
//                             4               Vn_Line 
//                             5               Va_Line
//                             6               Vb_Line
//                             7               Vc_Line
//                             8               1.25Vref
//                             9               Therm Mem
//                      The AFE samples lag the ADC samples by 2.5 sample times.  In order to compensate,
//                      two steps are taken:  1) The first ADC interrupt occurs one half-sample time after
//                      the AFE interrupt.  Subsequent interrupts are one sample time later.  This delays
//                      the ADC samples by one half-sample time.  2) This subroutine delays the samples by
//                      another 2 sample times.
//
//  CAVEATS:            None
//
//  INPUTS:             ADC_single_capture[] - the present set of values from the ADC (32-bit  ADC2:ADC1)
//
//  OUTPUTS:            ADC_samples[] - the new sample values in floating point format
//
//  ALTERS:             None
//
//  CALLS:              ADC_Cal_Scale()
//
//  EXECUTION TIME:     6.5usec max (measured on 190131 - rev 00.29 code)
//
//------------------------------------------------------------------------------------------------------------

void ADC_Process_Samples(void)
{
  uint32_t utemp32;
  uint8_t i;
  float DC_Offset_125, ftemp;

  for (i=0; i<5; i++)
  {
    ADC_test_samples[i] = ADC_single_capture[i];         // for DEBUG
  }


  for (i=0; i<5; i++)                                                        // convert to floats
  {
    utemp32 = ADC_single_capture[i];
    ADC_samples[i] = (float)(utemp32 & 0x0000FFFF);                          // ADC1 is lower 16-bit word
    ADC_samples[i+5] = (float)((ADC_single_capture[i] >> 16) & 0x0000FFFF);  // ADC2 is upper 16-bit word
  }

  DC_Offset_125 = ADC_samples[8];

  for (i=0; i<8; i++)
  {                                                     // Subtract 1.25VDC offset for currents and voltages
    ADC_samples[i] = ADC_samples[i] - DC_Offset_125;    //   (not for Therm Mem)
    if (i > 4)                                          // For Va, Vb, and Vc, compute Vln by subtracting Vn
    {                                                   //   Do this before scaling the values
      ADC_samples[i] = ADC_samples[i] - ADC_samples[4];
    }
    // Perform offset and gain cal to convert to engineering units.  Note, this must be executed before the
    //   samples are shifted so that the appropriate scaling constants (high-gain or low-gain) are applied.
    //   Also note that scaling is applied to Vn using the scaling constant for In sensed with a CT  - this
    //   doesn't matter, because Vn is subtracted from the voltages (already done immediately above) before
    //   scaling is applied.  Vn is not used otherwise.
    ADC_Cal_Scale(i);

    // Result of the following delay shifts:
    //         New sample --> ADC_PrevSample[][0] --> ADC_PrevSample[][1] --> ADC_samples[]
    // time     ADC_PrevSample[][0]       ADC_PrevSample[][1]    ADC_samples[]     
    //  n-1        Sample[n-1]               Sample[n-2]          Sample[n-3]
    //  n          Sample[n]                 Sample[n-1]          Sample[n-2]
    //  n+1        Sample[n+1]               Sample[n]            Sample[n-1]
    ftemp = ADC_PrevSample[i][1];                       // The ADC samples lead the AFE samples by 2.5
    ADC_PrevSample[i][1] = ADC_PrevSample[i][0];        //   sample periods.  A 1/2 sample-period delay is
    ADC_PrevSample[i][0] = ADC_samples[i];              //   done when the ADC is started.  The remaining 2
    ADC_samples[i] = ftemp;                             //   sample period delay is done here
  }

  if (Getting_AFE_Seed)                           // If getting seed values, first read the ADC high-gain
  {                                               //   inputs, then read the low-gain inputs
    if (USING_HIGH_GAIN)                          //   Read each high-gain input, and use it for the seed
    {                                             //   value if it is in range.  If the value is to be used,
      for (i=0; i<4; i++)                         //   set the state to 1
      {                                           //   Must use ADC_PrevSample because this now holds the
                                                  //   sample that was just read.
        if ((ADC_PrevSample[i][0] < ADC_MAX_SEEDVAL_NFRAME) && (ADC_PrevSample[i][0] > -ADC_MAX_SEEDVAL_NFRAME))    // *** DAH LIMITS ARE FRAME DEPENDENT
//        if ((ADC_PrevSample[i][0] < ADC_MAX_SEEDVAL_RFRAME) && (ADC_PrevSample[i][0] > -ADC_MAX_SEEDVAL_RFRAME))    // *** DAH LIMITS ARE FRAME DEPENDENT
//        if ((ADC_PrevSample[i][0] < 100) && (ADC_PrevSample[i][0] > -100))    // *** DAH LIMITS ARE FRAME DEPENDENT  FORCE LOW-GAIN SEED TO BE USED
        {
          Seed_State[i] = 1;
        }
      }
      LOW_GAIN_INPUTS;                            // Switch to low gain inputs
    }
    else                                          // If reading the low gain inputs, use the value if the
    {                                             //   existing value (that was read with the high gain
      for (i=0; i<4; i++)                         //   inputs) is out of range
      {
        if (Seed_State[i] != 1)
        {
          Seed_State[i] = 2;
        }
      }
      Getting_AFE_Seed = FALSE;                   // In any event, we are done getting the seed values
//      HIGH_GAIN_INPUTS;                 // *** DAH TEST ADDED 160424
    }
  }

  utemp32 = 0;                                            // for DEBUG - to set a breakpoint

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ADC_Process_Samples()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ADC_Cal_Scale()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Scale ADC Samples
//
//  MECHANICS:          This subroutine performs the scaling of the ADC sample to convert it from an
//                      integrated floating-point number to a current in amps or volts.  For now, the gain
//                      and offset scaling coefficients are based on calculated (ideal) values.  Eventually,
//                      they will be comprised of the Frame's calibration constants and the board's
//                      calibration constants.
//
//  CAVEATS:            None
//
//  INPUTS:             ADC_samples[] - the present set of values from the ADC (32-bit  ADC2:ADC1)
//
//  OUTPUTS:            ADC_samples[] - the new sample values in floating point format
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void ADC_Cal_Scale(uint8_t index)
{
  if (USING_HIGH_GAIN)
  {
    // For In, use the CT cal constants if not using a Rogowski
    if ((index == 3) && (!IN_USING_ROGOWSKI))
    {
      ADC_samples[index] = (ADC_samples[index] - ADCcalHigh.offset[4]) * ADCcalHigh.gain[4];
    }
    else
    {
      ADC_samples[index] = (ADC_samples[index] - ADCcalHigh.offset[index]) * ADCcalHigh.gain[index];
    }
  }
  else
  {
    // For In, use the CT cal constants if not using a Rogowski
    if ((index == 3) && (!IN_USING_ROGOWSKI))
    {
      ADC_samples[index] = (ADC_samples[index] - ADCcalLow.offset[4]) * ADCcalLow.gain[4];
    }
    else
    {
      ADC_samples[index] = (ADC_samples[index] - ADCcalLow.offset[index]) * ADCcalLow.gain[index];
    }
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ADC_Cal_Scale()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------
//  START OF FUNCTION           TA_Volt_Monitoring()
//------------------------------------------------------------------------------
//
// FUNCTION:            TA Voltage Monitoring function
//
// MECHANICS:           This function processes the TA voltage sample from the STM32 internal ADC3.
//                      ADC3 is 12-bit ADC with a 2.5V reference.
//
//                      PF3    TA_Sense               ADC3_IN9
//
//                      TA voltage sensing circuit consists of voltage divider resistors (10k and 215K).
//
//                       TA_Volt * (10k/225k)
//                       --------------------  * 4095 = TAVoltADC
//                              2.5V
//
//                      TA_Volt = TAVoltADC * (225/10) * (2.5/4095)
//
//                      TA_LOWVOLT_THRESHOLD = 38V.  lower than this threshold, TA can't trip
//                      TA_OPENVOLT_THRESHOLD = 18V. lower than this threshold, TA open/disconnect, insert alarm event
//                      TA_LOWVOLT_BOOST_THRESHOLD =    q1DCX34V.  TA threshold for USB boost. USB boost is not capable of providing 38V
//
// CAVEATS:             Called from Half Cycle Anniversary
//
// INPUTS:              TA Sense voltage (ADC3_IN9)
//
// OUTPUTS:             LowTA_Voltage, LowTA_Boost_Voltage, OpenTA_Voltage
//
// ALTERS:
//
// CALLS:
//
//------------------------------------------------------------------------------------------------------------
void TA_Volt_Monitoring(void)
{
  uint16_t TAVoltADC;

  // Set the ADC3 channel for the TA Sense voltage to get ready to convert
  ADC3->SQR3 = ADC3_TASENSE_CH;

  ADC3->CR2 |= ADC_CR2_SWSTART;                   // Start the conversion
  while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
  {
  }
  TAVoltADC = ((uint16_t)ADC3->DR & 0x0FFF);


  if (TAVoltValid == FALSE)                // If this is the first time reading TA Sense voltage,
  {                                               // seed sum with initial reading
    sum_TAVolt = TAVoltADC << 2;
    TAVoltValid = TRUE;
  }

  sum_TAVolt = sum_TAVolt - (sum_TAVolt >> 2) + TAVoltADC;
  avg_TAVolt = sum_TAVolt >> 2;                   // average 4 readings

  TA_Volt = avg_TAVolt * TA_VOLT_SCALING;         // scale based on voltage divider and 12-bit ADC with 2.5V ref

  if (TA_Volt <= TA_LOWVOLT_THRESHOLD)
  {
    LowTA_Voltage = 1;                            // TA voltage is too low to trip
  }
  else
  {
    LowTA_Voltage = 0;
  }

  if (TA_Volt <= TA_LOWVOLT_BOOST_THRESHOLD)
  {
     LowTA_Boost_Voltage = 1;                    // below TA threshold for USB boost
  }
  else
  {
     LowTA_Boost_Voltage = 0;
  }


  if (TA_Volt <= TA_OPENVOLT_THRESHOLD)
  {
                           // if USB is not connected, the power supply must be provided from CT or from aux-power.
                           // if only from CT, the fact that 3.3VDC to power up must based on the foundation that enough
                           // current goes through CT and TA voltage higher than 20V, which is definitely higher than the
                           // TA open voltage threshold.
                           // so once code arrives there, TA must be on open status.
     if (USBStatus == 0x00)
     {
        OpenTA_Voltage = 1;
     }
     else                           // if with USB cable connects there, actually we can't know the low voltage on TA is caused by
                                    // TA open or just a normal situation.
     {
        if (AuxPower_LowVolt == 1)  // if no aux-power, take the situation as a normal situation
        {                           // maybe TA is open, but we can't identify the case
           OpenTA_Voltage = 0;
        }
        else                        // if with aux-power, the TA must on open status
        {
           OpenTA_Voltage = 1;
        }
     }
  }
  else
  {
     OpenTA_Voltage = 0;
  }
}
//------------------------------------------------------------------------------
//  END OF FUNCTION             TA_Volt_Monitoring()
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//  START OF FUNCTION           AuxPower_Monitoring()
//------------------------------------------------------------------------------------------------------------
//
// FUNCTION:            Aux Power Voltage Monitoring function
//
// MECHANICS:           This function processes the Aux Power voltage sample from the STM32 internal ADC3.
//                      ADC3 is 12-bit ADC with a 2.5V reference.
//
//                      PF5    AuxPower_Sense         ADC3_IN15
//
//                      Aux Power voltage sensing circuit consists of voltage divider resistors (10k and 200K).
//
//                       Vaux1 * (10k/210k)
//                       --------------------  * 4095 = AuxVoltADC
//                              2.5V
//
//                      AUX_Volt = AuxVoltADC * (210/10) * (2.5/4095)     
//
//                      AUXPWR_LOWVOLT_THRESHOLD = 19.5V.  lower than this threshold, set flag
//
//                      Added code when Aux Power (or USB Power) is first applied to hold off protection for
//                      200ms to prevent GF and SD nuisance tripping at the lowest settings.  This was done for
//                      Power Defense.  The USB Boost signal (36V) gets diode "OR'ed" with the Aux Power signal and 
//                      goes to Vaux1 where we measure the voltage after the voltage divider.
//
// CAVEATS:             Called from main loop
//
// INPUTS:              AuxPower_Sense (ADC3_IN15)
//
// OUTPUTS:             AuxPower_Volt, AuxPower_LowVolt
//
// ALTERS:
//
// CALLS:
//
//------------------------------------------------------------------------------------------------------------
void AuxPower_Monitoring(void)
{
  uint16_t AuxVoltADC;


  // Set the ADC3 channel for the Aux Power Sense voltage to get ready to convert
  ADC3->SQR3 = ADC3_AUXPWR_CH;

  ADC3->CR2 |= ADC_CR2_SWSTART;                        // Start the conversion
  while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
  {
  }
  AuxVoltADC = ((uint16_t)ADC3->DR & 0x0FFF);


  if (AuxVoltValid == FALSE)                            // If this is the first time reading TA Sense voltage,
  {                                                     // seed sum with initial reading
    sum_AuxVolt = AuxVoltADC << 2;
    AuxVoltValid = TRUE; 
  }

  sum_AuxVolt = sum_AuxVolt - (sum_AuxVolt >> 2) + AuxVoltADC;
  avg_AuxVolt = sum_AuxVolt >> 2;                       // average 4 readings

  AuxPower_Volt = avg_AuxVolt * AUXPWR_VOLT_SCALING;    // scale based on voltage divider and 12-bit ADC with 2.5V ref

  if (AuxPower_Volt <= AUXPWR_LOWVOLT_THRESHOLD)
  {
    AuxPower_LowVolt = 1;                               // low Aux Power
  }
  else
  {
    AuxPower_LowVolt = 0;
    
    if (passed_1st_detection == FALSE)                  // Aux Power is good for the first time
    {  
      Prot_Enabled = FALSE;                             // Turn off protection to avoid nuisance trips when Auz Power is applied
      Prot_Timer = 20;                                  // Hold off protection for 200ms
      passed_1st_detection = TRUE;                      // This also works for USB power because the USB boost shows up as 36V and
    }                                                   // and is diode "OR'ed" with Aux Power
  }



}
//------------------------------------------------------------------------------------------------------------
//  END OF FUNCTION             AuxPower_Monitoring()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_BatteryVolt()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read and calculate the battery voltage
//
//  MECHANICS:          This subroutine processes the battery voltage sample from the STM32 internal ADC3.
//                      ADC3 is 12-bit ADC with a 2.5V reference.
//
//                      Battery voltage sensing circuit consists of a series Schottky diode and voltage
//                      divider resistors (1M and 499k)
//
//                       Vbattery * (1/1.499)
//                       --------------------- * 4095 =  BatteryVoltADC
//                              2.5V
//
//                      Vbattery = BatteryVoltADC * 1.499 * (2.5/4095) 
//
//                      Actually, the battery scaling will be done on an average of 4 readings.
//
//  CAVEATS:            Battery sensing must be enabled via the BAT_SENSE_EN signal from the STM32.  It is presently
//                      enabled here before the ADC sampling.  It is disabled before we exit the function.
//
//  INPUTS:             Battery Sense (ADC3_IN14)
//
//  OUTPUTS:            Vbattery
//
//  ALTERS:             None
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void Calc_BatteryVolt(void)                    // *** BP - Battery voltage is 2.14V with voltage divider
{                                              
  uint16_t BatteryVoltADC, j;            

  BATTERY_SENSE_EN;                            // Enable Battery Sensing (turn on FET)
                                               
  j = 2900;                                    // Delay 60usec before sampling                     
  while (j > 0)
  {
    --j;
  }
  
  // Set the ADC3 channel for the battery sense voltage to get ready to convert
  ADC3->SQR3 = ADC3_BATSENSE_CH;

  ADC3->CR2 |= ADC_CR2_SWSTART;         // Start the conversion
  while ( (ADC3->SR & ADC_SR_EOC) != ADC_SR_EOC )
  {
  }
  BatteryVoltADC = ((uint16_t)ADC3->DR & 0x0FFF);

  if (BatteryValid == FALSE)                    // If this is the first time reading battery voltage,
  {                                             // seed sum with initial reading
    sum_Vbat = BatteryVoltADC << 2;
  }

  sum_Vbat = sum_Vbat - (sum_Vbat >> 2) + BatteryVoltADC;
  Vbattery = sum_Vbat >> 2;                     // average 4 readings

  Vbattery = Vbattery * BATT_VOLT_SCALING;      // scale and add Schottky diode drop
  
  BatteryValid = TRUE;                          // 1st battery voltage reading done, value valid now
  BATTERY_SENSE_DIS;                            // Disable Battery Sensing (turn off FET)
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_BatteryVolt()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//
//                                  PXR35 Harmonics Computations
//
// The PXR35 performs harmonic analysis of the waveform data located in the waveform sample buffer,
// SampleBuf[].  A Discrete Fourier Transform (DFT) is performed on the four currents: Ia, Ib, Ic, In and
// six load-side (AFE) voltages: Van, Vbn, Vcn, Vab, Vbc, Vcb.
//
// Sampling at 80 times a line cycle permits harmonics up to the 39th to be extracted.  Samples are taken
// over 12 line cycles, allowing inter-harmonics to 5Hz (60Hz/12) are be computed.  The harmonic calculation
// is the "amplitude" form.  This is the rms value of the harmonic divided by the rms value of the
// fundamental.  Note, the sum of the harmonics in this format will not sum to 1.00.
//
// Harmonics computations are based largely on IEC 61000-4-30 and IEC 61000-4-7.  These specifications
// require that:
//   - the base sampling window be 200msec (12 line cycles at 60Hz)
//   - interharmonics be computed and grouped into the primary harmonics values
//   - the harmonics pass through a low-pass filter
//   - harmonics are computed on a periodic basis, at least 3 per every 3-second period
//   - harmonics are "aggregated" every 3 seconds
//
// The PXR35 provides two sets of harmonics.  One set, the "Aggregated" Harmonics, meets the above IEC
// requirements.  The second set, the "Captured" Harmonics, are the Aggregated Harmonics that can be frozen
// (capturing and computations stopped and last values are held) and unfrozen upon command.
//
// Harmonics are computed every 600msec, and are treated as real-time data (not as event data).  The
// harmonics process runs independently of the user waveform capture process and sampling process.  Twelve
// cycles of each waveform (except Igsrc, which is not included for harmonics) are used for harmonics
// computations.  To save memory space, a separate buffer is NOT used to hold the samples for harmonics.
// The Trip and Alarms sample buffer, SampleBuf[], is used instead.  This buffer holds 39 cycles.  When a
// harmonics computation is requested, the present index of SampleBuf[] is captured and used to determine
// the index of the 12th cycle prior to the present one.  This means we have 27 (39 - 12) cycles' worth of
// time, or 450msec, to complete the harmonics computations before these 12 cycles are overwritten.
// Assuming a 12msec main loop time, 3 passes through the Calc_Harmonics() subroutine for each waveform, and
// 10 waveforms, it takes 360msec to complete the harmonics computations.  This is less than the 450msec
// window, so the samples should not be overwritten before the computations are completed.  This also leaves
// 240msec (600msec - 360msec) to transmit the harmonics to the display processor, CAM port, and/or Modbus
// port.
//
// The harmonics are computed using the ARM4 DSP Library.  The algorithm was developed by George Gao and is
// described further in the Calc_Harmonics() subroutine.
// Note, harmonics are computed from 0Hz to 2395Hz, in increments of 5Hz.  The interharmonics (harmonics
// between mulitples of the fundamental frequency) are added to the primary harmonics per IEC 61000-4-7.
//
// The following is a flowchart of the harmonics process
//
//  600msec
//     |
//     |
// Set HarmReq
//     |
//     |
// (Calc_Harmonics)
//     |
// Set pointers for IA
//     |
//     |
// Increment aggregation counter
//     |
//     |
//     +<---------------------------------------------------------------------------------+
//     |                                                                                  |
//     |                                                                                  |
// Compute harmonics, including interharmonics                                            |
//     |                                                                                  |
//     |                                                                                  |
// Compute main harmonics (add adjacent interharmonics)                                   |
//     |                                                                                  |
//     |                                                                                  |
// Save as Instantaneous Harmonics                                                        |
//     |                                                                                  |
//     |                                                                                  |
// Pass harmonics through low-pass filter                                                 |
//     |                                                                                  |
//     |                                                                                  |
// Square and add to aggregation sum                                                      |
//     |                                                                                  |
//     |                         Y                                                        |
// If aggregation counter == 5 ----> Take square root of aggregation sum                  |
//     |                                           |                                      |
//     | N                                         |                                      |
//     |                             Save as Temporary Aggregated Harmonics               |
//     |                                           |                                      |
//     |                                           |                                      |
//     |                             Clear aggregation counter                            |
//     |                                           |                                      |
//     |                                           |                                      |
//     +<------------------------------------------+                                      |
//     |                                                                                  |
//     |                                                                                  |
// Increment waveform counter                                                             |
//     |                                                                                  |
//     |                   N                                                              |
// If last waveform done ----------> Set pointers for next waveform ----------------------+
//     |
//     | Y
//     |
// If not Frozen, save Instantaneous Harmonics as Captured Instantaneous Harmonics
//     |
//     |
// Clear HarmReq
//     |
//     |
// Increment aggregation sequence number
//     |
//     |
// Save Temporary Aggregated Harmonics as Aggregated Harmonics
//     |
//     |
// Initiate Display Processor transmissions for Instantaneous and Aggregated Harmonics
//     |
//     |
// Set state to 0
//
//
// The output of Calc_Harmonics() is a set of ten arrays of 39 elements, each representing a single harmonic
// ratio, the Aggregated Harmonics: 
//     HarmonicsAgg.Ia[0..38]
//     HarmonicsAgg.Ib[0..38]
//     HarmonicsAgg.Ic[0..38]
//     HarmonicsAgg.In[0..38]
//     HarmonicsAgg.Van[0..38]
//     HarmonicsAgg.Vbn[0..38]
//     HarmonicsAgg.Vcn[0..38]
//     HarmonicsAgg.Vab[0..38]
//     HarmonicsAgg.Vbc[0..38]
//     HarmonicsAgg.Vca[0..38]
//
// Each array element (0..38) is a harmonic value in multiples of 60Hz,  For example:
//     HarmonicsAgg.Ia[0] -- 1st (fundamental) harmonic (60Hz)
//     HarmonicsAgg.Ia[1] -- 2nd harmonic (120Hz)
//     HarmonicsAgg.Ia[2] -- 3rd harmonic (180Hz)
//           ......
//     HarmonicsAgg.Ia[38] - 39th harmonic (2340Hz)
//     HarmonicsAgg.Ia[39] - 40th harmonic (2400Hz)
// Note, the 40th harmonic is estimated and is taken directly from the harmonic at 2395Hz.
//
// The harmonic values are stored as unsigned integers (Uint16) that is the ratio of the harmonic rms
// amplitude and the fundamental rms amplitude in tenths of a per cent.
// Therefore, the values are from 0 .. 1000.
// 

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Harmonics()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Harmonics Analysis
//
//                      This subroutine performs harmonic analysis of the waveform data located in the
//                      waveform sample buffer, SampleBuf[].  A Discrete Fourier Transform (DFT) is
//                      performed on the four currents: Ia, Ib, Ic, In and 6 load-side (AFE) voltages: Van,
//                      Vbn, Vcn, Vab, Vbc, Vcb.  Harmonics to the 40th are computed in intervals of 5Hz.
//
//                      The harmonic calculation is the "amplitude" form.  This is the rms value of the
//                      harmonic divided by the rms value of the fundamental.  Note, the sum of the
//                      harmonics in this format will not sum to 1.00.
//
//  MECHANICS:          Sampling at 80 times a line cycle permits harmonics up to the 40th to be extracted.
//                      By sampling 12 line cycles, inter-harmonics to 5Hz (60Hz/12) may be computed.  The
//                      output of this routine is a set of ten arrays of 39 elements, each representing a
//                      single harmonic ratio.  This set of arrays is called the Aggregated Harmonics.
//                      There is an array for each current & voltage: 
//                              HarmonicsAgg.Ia[]
//                              HarmonicsAgg.Ib[]
//                              HarmonicsAgg.Ic[]
//                              HarmonicsAgg.In[]
//                              HarmonicsAgg.Van[]
//                              HarmonicsAgg.Vbn[]
//                              HarmonicsAgg.Vcn[]
//                              HarmonicsAgg.Vab[]
//                              HarmonicsAgg.Vbc[]
//                              HarmonicsAgg.Vca[]
//                      Each harmonic value is an unsigned integer (Uint16) that is the ratio of the
//                      harmonic rms amplitude and the fundamental rms amplitude in tenths of a per cent.
//                      Therefore, the values are from 0 .. 1000.
//
//                      The harmonics are computed using the ARM4 DSP Library.  The algorithm was developed
//                      by George Gao:
//
//      Abstract
//        This document describes a step-by-step approach to efficiently compute harmonics for an
//        aribtary-length real input.  The approach is based on chirp transform algorithm, or chirp-z
//        transform as it is commonly known.  The real input is first multiplied with a sequence of complex
//        numbers.  The result is then zero-padded, and then fed into an FFT function.  The FFT function's
//        output is further multiplied with a second sequence of complex numbers.  The result is then
//        multipled with a third sequence of complex numbers to yield the final results.
//
//      Signal Flow Diagram
//        The overall signal flow diagram is shown below.
//
//          w_n = W_N^(n^2/2)              x_n
//                  |                       |
//                  |                       V
//                  |              +------------------+
//                  +------------->|     Multiply     |
//                  |              +------------------+
//                  |                       |
//                  |                      g_n                        h_n = W_N^(-n^2/2)
//                  |                       |                                |
//                  |                       V                                V
//                  |              +------------------+             +------------------+
//                  |              | Zero-pad to NFFT |             | Zero-pad to NFFT |
//                  |              +------------------+             +------------------+
//                  |                       |                                |
//                  |                 zero-padded g_n                  zero-padded h_n
//                  |                       |                                |
//                  |                       V                                V
//                  |              +------------------+             +------------------+
//                  |              |  NFFT-point FFT  |             |  NFFT-point FFT  |
//                  |              +------------------+             +------------------+
//                  |                       |                                |
//                  |                      G_w                              H_w
//                  |                       |                                |
//                  |                       V                                |
//                  |              +------------------+                      |
//                  |              |     Multiply     |<---------------------+
//                  |              +------------------+
//                  |                       |
//                  |                      R_w
//                  |                       |
//                  |                       V
//                  |              +------------------+
//                  |              |  NFFT-point iFFT |    iFFT = Inverse FFT
//                  |              +------------------+
//                  |                       |
//                  |                      r_n
//                  |                       |
//                  |                       V
//                  |              +------------------+
//                  +------------->|     Multiply     |
//                                 +------------------+
//                                          |
//                                         y_n
//                                          |
//                                          V
//                                  +----------------+
//                                  |  Amplitude |.| |
//                                  +----------------+
//                                          |
//                                          V
//                                         z_n
//
//      Nomenclature
//
//        +---------------+--------------------------------------------------------------------------------+
//        | Symbol        | Meaning                                                                        |
//        |---------------|--------------------------------------------------------------------------------|
//        | x_n           | Aribtary-length real input signal, e.g., phase-A current Ia.                   |
//        | N             | Length of x_n, e.g., N=80 for a single cycle Ia in PXR 35.                     |
//        | M             | Total number of harmonics. In this document, M=N.                              |
//        | n             | Time index, 0 <= n < N.                                                        |
//        | W_N           | A complex constant. W_N = e^(-2*pi*i/N) = cos(-2*pi/80) + i*sin(-2*pi/80)      |
//        |               |    = 0.99692 - i * 0.078459                                                    |
//        | w_n           | N-element complex array. w_n = W_N^[(n^2)/2].                                  |
//        | NFFT          | Next power of 2 for FFT. NFFT = int32{2^[ceil(log_2(N+M-1))]}.                 |
//        | g_n           | N-element complex array. Element-wise product of x_n and w_n.                  |
//        |zero-padded g_n| NFFT-element complex array. First N elements are g_n. Remaining are zeros.     |
//        | G_w           | NFFT-element complex array. G_w = FFT(zero-padded g_n).                        |
//        | h_n           | (N+M-1)-element complex array. h_n = W_N^[-(n^2)/2] with 1 <= n < (N+M).       |
//        |zero-padded h_n| NFFT-element complex array. First (N+M-1) elements are h_n. Remaining are zero |
//        | H_w           | NFFT-element complex array. H_w=FFT(zero-padded h_n).                          |
//        | R_w           | NFFT-element complex array. Element-wise product of G_w and H_w.               |
//        | r_n           | NFFT-element complex array. r_n=iFFT(R_w). iFFT = Inverse FFT                  |
//        | y_n           | N-element complex array. Element-wise product of the Nth to the (N+M-1)th      |
//        |               |    elements of r_n with w_n.                                                   |
//        | z_n           | N-element complex array. Amplitude of y_n.                                     |
//        +---------------+--------------------------------------------------------------------------------+
//
//                      A single harmonic is computed in three passes through the subroutine.  Thus, all of
//                      the harmonics are computed after 30 passes through the subroutine.  Assuming a main
//                      loop time of 8msec, it will take 240 msec to update the harmonics.
//
//                      Note, the harmonics process runs independently of the user waveform capture process.
//                      Twelve cycles of each waveform (except Igsrc, which is not included for harmonics)
//                      are used for harmonics computations.  To save memory space, a separate buffer is NOT
//                      used to hold the samples for harmonics.  The global sample buffer, SampleBuf[], is
//                      used instead.  This buffer holds 39 cycles.  When a harmonics computation is
//                      requested, we only need to capture the present index of SampleBuf[], SampleIndex. We
//                      then have 27 (39 - 12) cycles' worth of time to complete the harmonics computations
//                      before these 12 cycles are overwritten.  This means we have 27/60Hz, or 450msec to
//                      complete the computations.  Assuming an 8msec main loop time, this gives us 56
//                      passes through the Calc_Harmonics() subroutine.  As there are 10 waveforms to
//                      compute, we must use 5 or less passes per waveform.
//
//                      Harmonic computations are controlled by a single flag, HarmReq
//                      The process is summarized below:
//                          Initialization:
//                              HarmReq = FALSE
//
//                          Timer Interrupt:                                         
//                              Set harmonics request flag (HarmReq = TRUE)
//
//                          Foreground, harmonics:
//                              if harmonics computation is requested (HarmReq == True)
//                                  compute harmonics
//                              clear request flag when done (HarmReq = False)
//
//  CAVEATS:            As mentioned above, the harmonics are computed using the samples in SampleBuf[].
//                      In order for all ten harmonics to be computed over the same "snapshot" of samples,
//                      they must be completed before new samples overwrite the existing samples in the
//                      12-cycle window.  Since SampleBuf[] is 39 cycles long, we have 27 cycles worth of
//                      time to complete the computations.  27cyc = 450msec.  450msec/10 = 45msec per value
//                      Assuming an 8msec loop, we can use up to 5 passes per value.  We are presently using
//                      3 passes, so this should be ok.
// 
//  INPUTS:             w_n[], H_w[], x_n[], HarmReq
// 
//  OUTPUTS:            HarmonicsAgg.xxx[], HarmonicsCap.xxx[]
//
//  ALTERS:             g_n[]
// 
//  CALLS:              arm_cmplx_mult_real_f32(), arm_fill_f32(), arm_cmplx_mult_cmplx_f32(),
//                      arm_cfft_f32(), arm_cmplx_mult_cmplx_f32(), arm_cmplx_mag_f32(), arm_scale_f32()
//
//  EXECUTION TIME:     Measured execution time on 220323 (Rev 00.53 code, with captured inputs on Van - not
//                      the captured waveform):
//                          4.89msec max (test pin toggled around the subroutine call in main)
//                          Note, this time includes sample interrupt times!
//                          3.42msec max with interrupts disabled around the subroutine call
// 
//------------------------------------------------------------------------------------------------------------

// *** DAH NEED TO ADD CODE TO CHECK THE RMS LEVEL OF THE CURRENT.  WE ONLY COMPUTE HARMONICS IF THE CURRENT
//         IS ABOVE A CERTAIN LEVEL.  MAY ALSO WANT TO ONLY COMPUTE HARMONICS IF THE CURRENT IS BELOW A
//         CERTAIN LEVEL TO ENSURE WE ARE ONLY USING THE AFE SAMPLES - REMEMBER, THE AFE FILTER IS
//         COMPENSATED FOR AND SO IF THE ADC SAMPLES ARE USED, THERE WILL BE AN INACCURACY INTRODUCED.  WE
//         ALSO NEED TO DECIDE WHAT TO DO IF WE AREN'T COMPUTING HARMONICS - DO WE KEEP THE LAST VALUE, MARK
//         THE VALUES INVALID, ETC.?  DO WE RESET THE AGGREGATION WHEN WE START UP AGAIN?  ALSO, WHAT DO WE
//         DO WITH THE SEQUENCE NUMBER?

void Calc_Harmonics(void)               // Amplitude analysis...
{
  static void *fptr;                        // These are void pointers so can be used for both the current
  static void *fptr1;                       //   and the voltage samples
  static void *fptr2;
  static void *fptr3;
  static uint16_t *instptr, *aggptr;
  static float *filptr, *sumptr;
  static uint8_t wf_count;
  uint16_t i, k, sample_indx_end, CH_exit;

  CH_exit = FALSE;

  while (!CH_exit)
  {
    switch (CH_State)                   // Enter state machine
    {
      case 0:                           // Case 0: Idle
        if (HarmReq)                        // If request for harmonics, begin
        {
          // This state is only executed the first time through (for Ia).
          // Set the starting index for harmonics computations to 12 cycles (960 samples) prior to the
          //   present sample index
            HarmSampleStartNdx = SampleIndex;         // Capture the present sample index
            HarmSampleStartNdx = ( (HarmSampleStartNdx >= N_SAMPLES) ?
                 (HarmSampleStartNdx - N_SAMPLES) : (TOTAL_SAMPLE_SETS + HarmSampleStartNdx - N_SAMPLES) );
          // Initialize fptr and fptr1 for Ia.  fptr points to the first sample in the 12-cycle window.
          //   fptr1 points to the first sample in SampleBuf[].  fptr1 is used if the 12-cycle window wraps
          //   around the end of SampleBuf[].
          // Initialize the Ia pointers to the harmonics calculation variables.
          //   instptr points to where the instantaneous harmonics result is stored (used in computing the
          //        aggregated harmonics)
          //   filptr points to where the filtered harmonics result is stored
          //   sumptr points to where the aggregated sum of squares is stored
          //   aggptr points to where the temporary aggregated harmonics result is stored
          // Initialize wf_count.  This is used to keep track of which of the ten waveforms is being
          //   processed
          // Increment HarmSumCount.  This holds the number of squared filtered results that have summed.
          //   When the count reaches 5, 3 seconds has elapsed and it is time to compute the new aggregated
          //   harmonics
          fptr = &SampleBuf[HarmSampleStartNdx].Ia;
          fptr1 = &SampleBuf[0].Ia;
          instptr = &HarmInst[0];
          filptr = &HarmonicsFil.Ia[0];
          sumptr = &HarmonicsSum.Ia[0];
          aggptr = &HarmonicsTemp.Ia[0];
          HarmSumCount++;
          wf_count = 0;
          CH_State = 1;
        }
        else                                // Otherwise, just exit
        {
          CH_exit = TRUE;
          break;
        }
//        break;                              Fall into next state

      case 1:                             // Case 1: Input setup for Ia thru In
        // Move the input waveform samples into x_n[].  Must check for whether the 12-cycle window wraps
        //   around the end of SampleBuf[].  There is wrap-around if the starting index, HarmSampleStartNdx,
        //   is greater than the length of the buffer, TOTAL_SAMPLE_SETS, minus 960 (the number of samples,
        //   held in constant N).
        //   If there is going to be wrap-around, divide it up into two moves, so we don't have to check
        //   each time in the for-loop.
//  TESTPIN_D1_LOW;
        // If the 12-cy SampleBuf[] will wrap around..
        if (HarmSampleStartNdx > (TOTAL_SAMPLE_SETS - N_SAMPLES))
        {                                                       // First move the samples from the starting
          k = 0;                                                //   index to the end of SampleBuf[]
          for (i=HarmSampleStartNdx; i<TOTAL_SAMPLE_SETS; ++i)
          {
            x_n[k++] = *((float *)fptr);
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);   // Increment pointer by number of bytes in
                                                                //   sample structure
          }
          sample_indx_end = (HarmSampleStartNdx + N_SAMPLES) - TOTAL_SAMPLE_SETS;
          for (i=0; i<sample_indx_end; ++i)                     // Then move the samples from the start of
          {                                                     //   SampleBuf[] to the last sample in the
            x_n[k++] = *((float *)fptr1);                       //   12-cycle window
            fptr1 = (char *)fptr1 + sizeof(struct RAM_SAMPLES); // Increment pointer by number of bytes in
          }                                                     //   sample structure                     
        }
        else                                                  // If no wrap-around, just move the samples
        {                                                     //   from the starting index to the last
          for (i=0; i<N_SAMPLES; ++i)                         //   sample in the 12-cycle window
          {
            x_n[i] = *((float *)fptr);
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);   // Increment pointer by number of bytes in
          }                                                     //   sample structure                     
        }
        CH_State = 4;
        break;

      case 2:                           // Case 2: Input setup Van thru Vcn
        // Move the input waveform samples into x_n[].  Must check for whether the 12-cycle window wraps
        //   around the end of SampleBuf[].  There is wrap-around if the starting index, HarmSampleStartNdx,
        //   is greater than the length of the buffer, TOTAL_SAMPLE_SETS, minus 960 (the number of samples,
        //   held in constant N).
        //   If there is going to be wrap-around, divide it up into two moves, so we don't have to check
        //   each time in the for-loop.
        // If the 12-cy SampleBuf[] will wrap around..
        if (HarmSampleStartNdx > (TOTAL_SAMPLE_SETS - N_SAMPLES))
        {                                                       // See comments above in case 1
          k = 0;
          for (i=HarmSampleStartNdx; i<TOTAL_SAMPLE_SETS; ++i)
          {
            x_n[k++] = (float32_t)(*((int16_t *)fptr));         // Voltage values are int16's - must cast
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);   //   to floats
          }
          sample_indx_end = (HarmSampleStartNdx + N_SAMPLES) - TOTAL_SAMPLE_SETS;
          for (i=0; i<sample_indx_end; ++i)
          {
            x_n[k++] = (float32_t)(*((int16_t *)fptr1));        // Voltage values are int16's - must cast
            fptr1 = (char *)fptr1 + sizeof(struct RAM_SAMPLES); //   to floats                           
          }
        }
        else                                                  // If no wrap-around, just do the move
        {                                                       // See comments above in case 1
          for (i=0; i<N_SAMPLES; ++i)
          {
            x_n[i] = (float32_t)(*((int16_t *)fptr));           // Voltage values are int16's - must cast
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);   //   to floats                           
          }
        }
        CH_State = 4;
        break;

      case 3:                           // Case 3: Input setup Vab thru Vca
        // Move the input waveform samples into x_n[].  Must check for whether the 12-cycle window wraps
        //   around the end of SampleBuf[].  There is wrap-around if the starting index, HarmSampleStartNdx,
        //   is greater than the length of the buffer, TOTAL_SAMPLE_SETS, minus 960 (the number of samples,
        //   held in constant N).
        //   If there is going to be wrap-around, divide it up into two moves, so we don't have to check
        //   each time in the for-loop.
        // Note, the line-line (LL) voltages are computed from the line-to-neutral (LN) voltages, e.g.
        //   Vab = Van - Vbn.  Four pointers are used, two for each voltage.
        // If the 12-cy SampleBuf[] will wrap around..
        if (HarmSampleStartNdx > (TOTAL_SAMPLE_SETS - N_SAMPLES))
        {                                                       // See comments above in case 1 and case 2
          k = 0;
          for (i=HarmSampleStartNdx; i<TOTAL_SAMPLE_SETS; ++i)
          {                                                     // LL voltages computed from LN voltages
            x_n[k++] = (float32_t)(*((int16_t *)fptr)) - (float32_t)(*((int16_t *)fptr2));
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);
            fptr2 = (char *)fptr2 + sizeof(struct RAM_SAMPLES);
          }
          sample_indx_end = (HarmSampleStartNdx + N_SAMPLES) - TOTAL_SAMPLE_SETS;
          for (i=0; i<sample_indx_end; ++i)
          {                                                     // LL voltages computed from LN voltages
            x_n[k++] = (float32_t)(*((int16_t *)fptr1)) - (float32_t)(*((int16_t *)fptr3));
            fptr1 = (char *)fptr1 + sizeof(struct RAM_SAMPLES);
            fptr3 = (char *)fptr3 + sizeof(struct RAM_SAMPLES);
          }
        }
        else                                                  // If no wrap-around, just do the move
        {                                                       // See comments above in case 1 and case 2
          for (i=0; i<N_SAMPLES; ++i)
          {                                                     // LL voltages computed from LN voltages
            x_n[i] = (float32_t)(*((int16_t *)fptr)) - (float32_t)(*((int16_t *)fptr2));
            fptr = (char *)fptr + sizeof(struct RAM_SAMPLES);
            fptr2 = (char *)fptr2 + sizeof(struct RAM_SAMPLES);
          }
        }
        CH_State = 4;
//        break;                                Fall into next state

      case 4:                           // Case 4: First half of calculations
        // Floating-point complex-by-real multiplication
        //   g_n = g_n .* w_n     '.*' denotes element-by-element multiplication
                                        // 131usec execution time typical (no sampling interrupt)
        arm_cmplx_mult_real_f32( (float32_t *)w_n, x_n, g_n, N_SAMPLES );

        // zero-pad both real and imag of g[n] for subsequent NFFT-point FFT
                                        // 69usec execution time typical (no sampling interrupt)
        arm_fill_f32(0.0f, g_n + 2 * N_SAMPLES, 2 * (NFFT - N_SAMPLES));

        // In-place FFT on zero-padded g[n].  Results Gw = gn
        //   ifftFlag = 0           Forward FFT
        //   bitReverseFlag = 1     Bit reversal for radix-2 Cooley-Tukey FFT
                                        // 3.0msec execution time typical (includes sampling interrupts)
        arm_cfft_f32(&arm_cfft_sR_f32_len2048, g_n, 0, 1);

        // Gw .* H_w = g_n .* H_w
                                        // 452usec execution time typical (includes sampling interrupt)
        arm_cmplx_mult_cmplx_f32(g_n, (float32_t *)H_w, g_n, NFFT);
        CH_State++;
        CH_exit = TRUE;                 // Exit the subroutine now
        break;

      case 5:                           // Case 5: Second half of calculations
        // Inverse FFT on Gw * H_w
        //   ifftFlag = 1           Inverse FFT
        //   bitReverseFlag = 1     Bit reversal for radix-2 Cooley-Tukey FFT
                                        // 3.5msec execution time typical (includes sampling interrupt)
        arm_cfft_f32( &arm_cfft_sR_f32_len2048, g_n, 1, 1 );

        // Use g_n[ 2 * ( N - 1 ) <= array index < 2 * ( N + N - 1 ) ] as input
        //   Harmonics output g_n in { real, imag, ... , ... , real, imag } format 
                                        // 216usec execution time typical (includes sampling interrupt)
        arm_cmplx_mult_cmplx_f32( g_n + 2 * (N_SAMPLES - 1), (float32_t *)w_n, g_n, N_SAMPLES );

        // Harmonics amplitude output g_n
                                        // 305usec execution time typical (includes sampling interrupt)
        arm_cmplx_mag_f32(g_n, g_n, N_SAMPLES);

        // Special scaling treatment of dc component in g_n[0]
        g_n[0] /= 2.0f;
  
        // Vector scaling w.r.t fundamental.  
        //   In one cycle data, g_n[ 1 ] is the fundamental.  
        //   And only need to consider the first ( N / 2 ) elements 
                                        // 24usec execution time typical (no sampling interrupt)
        arm_scale_f32(g_n, 1000 / g_n[ INDEX_FUNDAMENTAL ], g_n, N_SAMPLES/2);

        // Compensate for the AFE filter
        // Note, arm_mult_f32() does a vector by vector multiplication element by element.  See
        //   arm_mult_f32.c in
    // C:\Program Files (x86)\IAR Systems\Embedded Workbench 7.5\arm\CMSIS\DSP_Lib\Source\BasicMathFunctions
        arm_mult_f32(&g_n[1], (float32_t *)COMPENSATION_FACTOR_60HZ, &g_n[1], 479);

        // Final harmonics in % stored in g_n[1..479]
        //     g_n[0] = DC component, g_n[1] = 5Hz, g_n[2] = 10Hz, ..., g_n[479] = 2395Hz
        // So primary harmonics (multiples of the fundamental) are at g_n[12], g_n[24], etc.
        k = 0;
        // We are only storing the 39 primary harmonics (i.e., the multiples of 60Hz).  The sub-harmonics
        //   are added to the primary harmonics per IEC61000-4-7:
        //       For the currents:
        //         for the first two harmonics (60Hz and 120Hz), it is just the corresponding result
        //         for the remaining higher harmonics, it is the square root of the sum of squares of the
        //           primary harmonic plus the adjacent 4 lower and higher harmonics plus 1/2 of the
        //           adjacent 5th higher and lower harmonics
        //           For example, for the 5th harmonic (at 300Hz), the result is
        //              SQRT(H[300]*H[300] + H[295]*H[295] + H[290]*H[290] + H[285]*H[285] + H[280]*H[280]
        //                                 + H[305]*H[305] + H[310]*H[310] + H[315]*H[315] + H[320]*H[320]
        //                                 + 1/2 * H[275]*H[275] + 1/2 * H[325]*H[325])
        //       For the voltages:
        //         for the first two harmonics (60Hz and 120Hz), it is just the corresponding result
        //         for the remaining higher harmonics, it is the square root of the sum of squares of the
        //           primary harmonic plus the adjacent lower and higher harmonics
        //           For example, for the 5th harmonic (at 300Hz), the result is
        //              SQRT(H[300]*H[300] + H[295]*H[295] + H[305]*H[305])
        if (wf_count < 4)
        {
          instptr[k++] = (uint16_t)((float)g_n[12] + 0.5f);     // index 12: 60Hz
          instptr[k++] = (uint16_t)((float)g_n[24] + 0.5f);     // index 24: 120Hz
          // 40th harmonic.  We only have up to 2395, so use 2395, 2390, 2385, 2380, 2375:
          instptr[39] =  (uint16_t)( sqrtf(
                               ((float)g_n[479] * (float)g_n[479])          // 1st lower adj harmonic (2395)
                             + ((float)g_n[478] * (float)g_n[479])          // 2nd lower adj harmonic (2390)
                             + ((float)g_n[477] * (float)g_n[477])          // 3rd lower adj harmonic (2385)
                             + ((float)g_n[476] * (float)g_n[476])          // 4th lower adj harmonic (2380)
                             + ((float)g_n[475] * (float)g_n[475] * 0.5f)   // 5th lower adj harmonic (2375)
                                   ) + 0.5f);                               // Add .5 for roundoff
          // Remaining harmonics (3 thru 39): add the adjacent harmonics per IEC61000-4-7
          for (i=36; i<=468; i+=12)
          {
            instptr[k++] = (uint16_t)( sqrtf(
                               ((float)g_n[i] * (float)g_n[i])              // Primary
                             + ((float)g_n[i-1] * (float)g_n[i-1])          // 1st lower adjacent harmonic
                             + ((float)g_n[i-2] * (float)g_n[i-2])          // 2nd lower adjacent harmonic
                             + ((float)g_n[i-3] * (float)g_n[i-3])          // 3rd lower adjacent harmonic
                             + ((float)g_n[i-4] * (float)g_n[i-4])          // 4th lower adjacent harmonic
                             + ((float)g_n[i+1] * (float)g_n[i+1])          // 1st upper adjacent harmonic
                             + ((float)g_n[i+2] * (float)g_n[i+2])          // 2nd upper adjacent harmonic
                             + ((float)g_n[i+3] * (float)g_n[i+3])          // 3rd upper adjacent harmonic
                             + ((float)g_n[i+4] * (float)g_n[i+4])          // 4th upper adjacent harmonic
                             + ((float)g_n[i-5] * (float)g_n[i-5] * 0.5f)   // 5th lower adjacent harmonic
                             + ((float)g_n[i+5] * (float)g_n[i+5] * 0.5f)   // 5th upper adjacent harmonic
                                     ) + 0.5f);                             // Add .5 for roundoff
          }
        }
        else
        {
          instptr[k++] = (uint16_t)((float)g_n[12] + 0.5f);     // index 12: 60Hz
          instptr[k++] = (uint16_t)((float)g_n[24] + 0.5f);     // index 24: 120Hz
          // 40th harmonic.  We only have up to 2395, so use 2395, 2390, 2385, 2380, 2375:
          instptr[39] =  (uint16_t)( sqrtf(
                               ((float)g_n[479] * (float)g_n[479])          // 1st lower adj harmonic (2395)
                             + ((float)g_n[478] * (float)g_n[479])          // 2nd lower adj harmonic (2390)
                             + ((float)g_n[477] * (float)g_n[477])          // 3rd lower adj harmonic (2385)
                             + ((float)g_n[476] * (float)g_n[476])          // 4th lower adj harmonic (2380)
                             + ((float)g_n[475] * (float)g_n[475] * 0.5f)   // 5th lower adj harmonic (2375)
                                   ) + 0.5f);                               // Add .5 for roundoff
          for (i=36; i<=468; i+=12)
          {
            instptr[k++] = (uint16_t)( sqrtf(
                               ((float)g_n[i] * (float)g_n[i])              // Primary
                             + ((float)g_n[i-1] * (float)g_n[i-1])          // 1st lower adjacent harmonic
                             + ((float)g_n[i+1] * (float)g_n[i+1])          // 1st upper adjacent harmonic
                                     ) + 0.5f);                             // Add .5 for roundoff
          }
        }
        // Filtered harmonics for aggregated harmonics.  Square the filtered harmonics and add to aggregated
        //   sum
        for (k=0; k<40; ++k)
        {
          filptr[k] = (filptr[k] * HARM_LPF_BETA + (float)instptr[k])/HARM_LPF_ALPHA;
          sumptr[k] += filptr[k] * filptr[k];
        }

        // When the count reaches 5, 3 seconds has elapsed (5 x 600msec).  Compute the RMS and store
        // Note: testing conducted on 190501 with simulated inputs showed it takes about 9 passes (around
        //   27 seconds) for the Aggregated Harmonics to be within 0.5% of the final value
        if (HarmSumCount >= 5)
        {
          for (k=0; k<40; ++k)
          {
            aggptr[k] = (uint16_t)(sqrtf(sumptr[k]/5) + 0.5f);
            sumptr[k] = 0;
          }
        }

        switch (++wf_count)                             // Increment the waveform count for the next
        {                                               //   waveform, then jump to initialize the
          case 1:                                       //   pointers and the state
            fptr = &SampleBuf[HarmSampleStartNdx].Ib;   // We are done when wf_count reaches 10
            fptr1 = &SampleBuf[0].Ib;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Ib[0];
            sumptr = &HarmonicsSum.Ib[0];
            aggptr = &HarmonicsTemp.Ib[0];
            CH_State = 1;
            break;
          case 2:
            fptr = &SampleBuf[HarmSampleStartNdx].Ic;
            fptr1 = &SampleBuf[0].Ic;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Ic[0];
            sumptr = &HarmonicsSum.Ic[0];
            aggptr = &HarmonicsTemp.Ic[0];
            CH_State = 1;
            break;
          case 3:
            fptr = &SampleBuf[HarmSampleStartNdx].In;
            fptr1 = &SampleBuf[0].In;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.In[0];
            sumptr = &HarmonicsSum.In[0];
            aggptr = &HarmonicsTemp.In[0];
            CH_State = 1;
            break;
          case 4:
            fptr = &SampleBuf[HarmSampleStartNdx].VanAFE;
            fptr1 = &SampleBuf[0].VanAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Van[0];
            sumptr = &HarmonicsSum.Van[0];
            aggptr = &HarmonicsTemp.Van[0];
            CH_State = 2;
            break;
          case 5:
            fptr = &SampleBuf[HarmSampleStartNdx].VbnAFE;
            fptr1 = &SampleBuf[0].VbnAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Vbn[0];
            sumptr = &HarmonicsSum.Vbn[0];
            aggptr = &HarmonicsTemp.Vbn[0];
            CH_State = 2;
            break;
          case 6:
            fptr = &SampleBuf[HarmSampleStartNdx].VcnAFE;
            fptr1 = &SampleBuf[0].VcnAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Vcn[0];
            sumptr = &HarmonicsSum.Vcn[0];
            aggptr = &HarmonicsTemp.Vcn[0];
            CH_State = 2;
            break;
          case 7:
            fptr = &SampleBuf[HarmSampleStartNdx].VanAFE;
            fptr1 = &SampleBuf[0].VanAFE;
            fptr2 = &SampleBuf[HarmSampleStartNdx].VbnAFE;
            fptr3 = &SampleBuf[0].VbnAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Vab[0];
            sumptr = &HarmonicsSum.Vab[0];
            aggptr = &HarmonicsTemp.Vab[0];
            CH_State = 3;
            break;
          case 8:
            fptr = &SampleBuf[HarmSampleStartNdx].VbnAFE;
            fptr1 = &SampleBuf[0].VbnAFE;
            fptr2 = &SampleBuf[HarmSampleStartNdx].VcnAFE;
            fptr3 = &SampleBuf[0].VcnAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Vbc[0];
            sumptr = &HarmonicsSum.Vbc[0];
            aggptr = &HarmonicsTemp.Vbc[0];
            CH_State = 3;
            break;
          case 9:
            fptr = &SampleBuf[HarmSampleStartNdx].VcnAFE;
            fptr1 = &SampleBuf[0].VcnAFE;
            fptr2 = &SampleBuf[HarmSampleStartNdx].VanAFE;
            fptr3 = &SampleBuf[0].VanAFE;
            instptr = &HarmInst[0];
            filptr = &HarmonicsFil.Vca[0];
            sumptr = &HarmonicsSum.Vca[0];
            aggptr = &HarmonicsTemp.Vca[0];
            CH_State = 3;
            break;
          default:                                      // Done so reset state and clear the request flag
            CH_State = 0;
            if (HarmSumCount >= 5)                      // If aggregation count has reached 5, update the
            {                                           //   harmonics and reset the count
              if (!HarmFrozen)                            // If harmonics are not frozen, update the Captured
              {                                           //   Instantaneous Harmonics with the new
                k = 500;                                  // *** DAH TEST CODE
                for (i=0; i<40; ++i)                      //   Captured Harmonics
                {
/*                  HarmonicsCap.Ia[i] = HarmonicsTemp.Ia[i];
                  HarmonicsCap.Ib[i] = HarmonicsTemp.Ib[i];
                  HarmonicsCap.Ic[i] = HarmonicsTemp.Ic[i];
                  HarmonicsCap.In[i] = HarmonicsTemp.In[i];
                  HarmonicsCap.Van[i] = HarmonicsTemp.Van[i];
                  HarmonicsCap.Vbn[i] = HarmonicsTemp.Vbn[i];
                  HarmonicsCap.Vcn[i] = HarmonicsTemp.Vcn[i];
                  HarmonicsCap.Vab[i] = HarmonicsTemp.Vab[i];
                  HarmonicsCap.Vbc[i] = HarmonicsTemp.Vbc[i];
                  HarmonicsCap.Vca[i] = HarmonicsTemp.Vca[i]; */
                  HarmonicsCap.Ia[i] = k++;               // *** DAH TEST CODE
                  HarmonicsCap.Ib[i] = k++;
                  HarmonicsCap.Ic[i] = k++;
                  HarmonicsCap.In[i] = k++;
                  HarmonicsCap.Van[i] = k++;
                  HarmonicsCap.Vbn[i] = k++;
                  HarmonicsCap.Vcn[i] = k++;
                  HarmonicsCap.Vab[i] = k++;
                  HarmonicsCap.Vbc[i] = k++;
                  HarmonicsCap.Vca[i] = k++;
                }
              }
              k = 0;                                         // *** DAH TEST CODE
              for (i=0; i<40; ++i)                        //   Aggregated Harmonics
              {
/*                HarmonicsAgg.Ia[i] = HarmonicsTemp.Ia[i];     *** DAH TEST CODE COMMENTED OUT
                HarmonicsAgg.Ib[i] = HarmonicsTemp.Ib[i];
                HarmonicsAgg.Ic[i] = HarmonicsTemp.Ic[i];
                HarmonicsAgg.In[i] = HarmonicsTemp.In[i];
                HarmonicsAgg.Van[i] = HarmonicsTemp.Van[i];
                HarmonicsAgg.Vbn[i] = HarmonicsTemp.Vbn[i];
                HarmonicsAgg.Vcn[i] = HarmonicsTemp.Vcn[i];
                HarmonicsAgg.Vab[i] = HarmonicsTemp.Vab[i];
                HarmonicsAgg.Vbc[i] = HarmonicsTemp.Vbc[i];
                HarmonicsAgg.Vca[i] = HarmonicsTemp.Vca[i];   */
                HarmonicsAgg.Ia[i] = k++;               // *** DAH TEST CODE
                HarmonicsAgg.Ib[i] = k++;
                HarmonicsAgg.Ic[i] = k++;
                HarmonicsAgg.In[i] = k++;
                HarmonicsAgg.Van[i] = k++;
                HarmonicsAgg.Vbn[i] = k++;
                HarmonicsAgg.Vcn[i] = k++;
                HarmonicsAgg.Vab[i] = k++;
                HarmonicsAgg.Vbc[i] = k++;
                HarmonicsAgg.Vca[i] = k++;
              }

              HarmSumCount = 0;
              K_FactorReq = TRUE;         // Set flag to compute the K-Factors
            }
            HarmReq = FALSE;   // *** DAH ADD CODE TO REQUEST TO SEND OUT CAM OR DISPLAY COMMS
         // break;
        }
        CH_exit = TRUE;
        break;

      default:                          // This state should never be entered
        CH_State = 0;
        CH_exit = TRUE;
        break;

    }
  }

//TESTPIN_D1_HIGH;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Harmonics()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       TP_CoilTempRMSavg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate coil temperature RMS average
//
//  MECHANICS:          This subroutine peforms the following operations:
//                        1) Calculates C*sin(800Hz), C*cos(800Hz)
//                           - sin(800Hz) and cos(800Hz) are defined by a table
//                           - C is the coil temp waveform
//                        2) Sums the values in C*sin(800Hz) and takes the average CS
//                        3) Sums the values in C*cos(800Hz) and takes the average CC
//                        4) Calculates RMS avg = sqrt(CS^2+CC^2)
//                      
//                      Coil Temperature Measurement Overview
//                        - A 4800Hz square wave (generated by Timer 8) is injected into the Rogowski coil.
//                        - The resultant voltage across the coil is sampled at a 4000Hz rate (Timer 5 is
//                          used to trigger ADC3 conversions every 250usec)
//                        - Samples are read in the sampling interrupt (DMA1_Stream0_IRQHandler()) and
//                          stored in coil_temp_samples[].  The circular buffer stores 200 samples - this is
//                          an integral number of cycles at both 4000Hz (200) and 4800Hz (240)
//                        - The RMS value of the difference waveform, 800Hz, is computed by multiplying the
//                          captured samples by a unit sin(800Hz) and cos(800Hz) waveform
//                      
//  CAVEATS:            Assumes a coil temperature waveform capture has occurred
//
//  INPUTS:             coil_temp_samples[ ]
// 
//  OUTPUTS:            Returns the coil temp RMS avg
//
//  ALTERS:             None
// 
//  CALLS:              sqrtf()
// 
//------------------------------------------------------------------------------------------------------------

float TP_CoilTempRMSavg(void)
{
  uint8_t i, j, k;
  float tmp1, tmp2, RMSavg;

  // Constants used only in this subroutine
  //   2*pi*800*t, t = 0, 250usec, 500usec, 750usec, 1msec, then repeats
  float sin800[5] = {0, 0.951057, 0.587785, -0.58779, -0.95106};  // sin wave at 800Hz 
  float cos800[5] = {1, 0.309017, -0.80902, -0.80902, 0.309017};  // cosine wave at 800Hz
  
  tmp1 = 0;
  tmp2 = 0;
  k = 0;
                                                                    // t for above starts a 0s, and increments
                                                                    // every 0.00025s
                                                                    // Values repeat every 5 samples
  
  // Coil Temp samples are multiplied by a 800Hz sine wave and 800Hz cosine wave and them summed
  //   respectively
  for(i=0; i<40; ++i)
//  for(i=0; i<120; ++i)                  // *** DAH  FOR LEO  190605
  {
    for(j=0; j<5; ++j)
    {
      tmp1 += coil_temp_samples[k]*sin800[j];                    
      tmp2 += coil_temp_samples[k]*cos800[j];
      ++k;
    }
  }
  tmp1 = tmp1/200;                                             // tmp1 average
  tmp2 = tmp2/200;                                             // tmp2 average
//  tmp1 = tmp1/200;                                             // tmp1 average  *** DAH  FOR LEO  190605
//  tmp2 = tmp2/200;                                             // tmp2 average  *** DAH  FOR LEO  190605
  RMSavg = sqrtf(tmp1*tmp1 +tmp2*tmp2);
  return (RMSavg);
}
  
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         TP_CoilTempRMSavg()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_AppPF()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Apparent Power Factor Calculation
//
//                      This subroutine performs apparent power factor calculations using the following
//                      definition:
//                      AC power flow has the three components: real power (P), measured in watts; apparent
//                      power (S), measured in volt-amperes; and reactive power (Q), measured in reactive 
//                      volt-amperes (var).
//
//                      Apparent Power Factor is defined as:
//                        PF = P/S
//                      Apparent power factor includes the effects of harmonics in the currents (and
//                      voltages), along with the phase difference between the voltage and current.
//
//                      In the case of a perfectly sinusoidal waveform, P, Q and S can be expressed as
//                      vectors that form a vector triangle such that:
//                            S^2 = P^2 + Q^2
//                      If "a" is the phase angle between the current and voltage, then the power factor is
//                      equal to cos(a), and P = S * abs(cos(a)).  Since the units are consistent, the power 
//                      factor is by definition a dimensionless number between 0 and 1.  When power factor
//                      is equal to 0, the energy flow is entirely reactive, and stored energy in the load
//                      returns to the source on each cycle.  When the power factor is 1, all the energy
//                      supplied by the source is consumed by the load. Power factors are usually stated as
//                      "leading" or "lagging" to show the sign of the phase angle.  A "lagging" PF
//                      describes the condition where the current "lags" the voltage.
//
//                      Non-linear loads change the shape of the current waveform from a sine wave to some
//                      other form.  Non-linear loads create harmonic currents in addition to the original
//                      (fundamental frequency) AC current.
//
//  MECHANICS:          Apparent PF is just P/S:
//                          Pwr200msec.Px/Pwr200msecApp.AppPx
//                      t200msecW points to Pwr200msec.Px
//                      t200msecVA points to Pwr200msecApp.AppPx
//                      
//                      If the apparent power is zero, the power factor is set to NAN to prevent divide by
//                      zero
//
//                      If the calculated power factor exceeds 1.00, it is clamped to 1.00 (or -1.00).
//
//  CAVEATS:            None
// 
//  INPUTS:             t200msecW[] - pointer to Pwr200msec.Px
//                      t200msecVA[] - pointer to Pwr200msecApp.AppPx
//                      PF_MinApp[], PF.MaxApp[]
// 
//  OUTPUTS:            PF.App[], PF.MinApp[], PF.MinApp_TS[], PF.MaxApp[], PF.MaxApp_TS[]
//                          Index   Description
//                            0     Phase A
//                            1     Phase B
//                            2     Phase C
//                            3     Total
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime()
// 
//  EXECUTION TIME:     Measured on 190813 (rev 0.33 code): 5.3usec (with interrupts disabled)
//
//------------------------------------------------------------------------------------------------------------

void Calc_AppPF(void)
{
  struct INTERNAL_TIME presenttime;
  float temp1, temp2;
  uint8_t i;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  // Apparent PFs: Four total sets: Phase A, B, C, and Total
  // Compute PF, then update min and max PF for each set
  // Pointer tables are used to point to the values.  The tables are defined at the top of this file
  for (i=0; i<4; i++)
  {
    temp1 = *t200msecW[i];              // Phase real power
    temp2 = *t200msecVA[i];             // Phase apparent power
    if (temp2 > 0.0F)                   // Make sure no divide by 0
    {
      // Compute unsigned PF
      temp1 = ((temp1 < 0.0F)? (-1.0F * temp1/temp2) : (temp1/temp2));
      if (temp1 > 1.0F)               // Clamp at 1
      {
        temp1 = 1.0F;
      }
      // *** DAH  ADD CODE TO SET THE SIGN BASED ON THE SETPOINTS  (IEC, IEEE, ALTERNATE IEEE)
      //  *** FOR NOW, JUST SAVE AS IS
      PF.App[i] = temp1;
    
      // Check for minimum and maximum - use absolute value for comparison.  Also use value as new min or max
      //   if the existing min or max value is invalid
      temp2 = ((PF.MinApp[i] < 0.0F) ? (-1.0F * PF.MinApp[i]) : PF.MinApp[i]);
      if ( (temp1 < temp2) || (isnan(temp2)) )      // Check for min
      {
        PF.MinApp[i] = PF.App[i];
        PF.MinApp_TS[i] = presenttime;
        // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
      }
      temp2 = ((PF.MaxApp[i] < 0.0F) ? (-1.0F * PF.MaxApp[i]) : PF.MaxApp[i]);
      if ( (temp1 > temp2) || (isnan(temp2)) )        // Check for max
      {
        PF.MaxApp[i] = PF.App[i];
        PF.MaxApp_TS[i] = presenttime;
        // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
      }
    }
    else
    {                                   // Set PF invalid if power is zero 
      PF.App[i] = NAN;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_AppPF()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_DispPF_THD()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Displacement Power Factor and THD Calculation
//
//                      This subroutine performs the displacement power factor and THD calculations using
//                      the following definitions:
//                      Displacement Power Factor is defined as:
//                        cos(I1/V1)  (I1 and V1 are the current and voltage at the fundamental frequency)
//                      Displacement power factor is the difference in phase between the current and voltage
//                      waveforms at the fundamental frequency
//                      THD is defined as SQRT[(RMS^2 - Fund^2)/Fund^2] * 100%
//                      Basically, it is the percentage of a waveform that is not comprised of the fundamental
//
//                      In the case of a perfectly sinusoidal waveform, P, Q and S can be expressed as
//                      vectors that form a vector triangle such that:
//                            S^2 = P^2 + Q^2
//                      If "a" is the phase angle between the current and voltage, then the power factor is
//                      equal to cos(a), and P = S * abs(cos(a)).  Since the units are consistent, the power 
//                      factor is by definition a dimensionless number between 0 and 1.  When power factor
//                      is equal to 0, the energy flow is entirely reactive, and stored energy in the load
//                      returns to the source on each cycle.  When the power factor is 1, all the energy
//                      supplied by the source is consumed by the load. Power factors are usually stated as
//                      "leading" or "lagging" to show the sign of the phase angle.  A "lagging" PF
//                      describes the condition where the current "lags" the voltage.
//
//                      Non-linear loads change the shape of the current waveform from a sine wave to some
//                      other form.  Non-linear loads create harmonic currents in addition to the original
//                      (fundamental frequency) AC current.
//
//                      Note, the Displacement Power factor and THD both use the unfiltered samples. THD
//                      must use the unfiltered currents and voltages because we don't want the filter to
//                      filter out some of the harmonics.  Displacement power factor also uses the
//                      unfiltered currents and voltages because it is related to THD (it can be derived
//                      from THD, and it is best to use the same values when computing the two values.
//
//                      THD is stored in array THD[0..9] as follows:
//                          THD[0] - Ia THD         THD[4] - Van THD        THD[7] - Vab THD
//                          THD[1] - Ib THD         THD[5] - Vbn THD        THD[8] - Vbc THD
//                          THD[2] - Ic THD         THD[6] - Vcn THD        THD[9] - Vca THD
//                          THD[3] - In THD
//
//  MECHANICS:          Displacement PF is cos(phi), where phi is the angle between the current and voltage
//                      waveforms at the fundamental frequency.  This is a more complicated computation:
//                          1) The real and imaginary components of the fundamentals of the voltage and
//                             currents are computed
//                          2) The phase angle of the current and voltage waveforms at the fundamental
//                             frequency are computed 
//                          3) The cosine of the angle between the voltage and current waveforms is computed
//                             using the trig identity cos(a - b) = cos(a)cos(b) + sin(a)sin(b)
//                      The real (cosine) and imaginary (sin) components of a waveform at the fundamental
//                      frequency are obtained by multiplying the input waveform by a unit cosine or sine
//                      waveform, then integrating over the period.  The trig identity:
//                          sin^2(x) = 1/2 - 1/2 cos(2x) is used.  Therefore, if:
//                          Input = Asin(x), then the fundamental sine is derived as
//                              1/T*integral[Asin^2(x)] = A/2, and A = 2/T * integral[Asin^2(x)]
//                      In the digital realm, the integral is the sum of the input samples multiplied by the
//                      the unit sine and cosine waveform, and so the magnitude of the fundamental sine and
//                      cosine components is:
//                          A1sin = 2/960 * CurVol200msNoFltrSinSOS_Sav[i]      (current)
//                          A1cos = 2/960 * CurVol200msNoFltrCosSOS_Sav[i]      (current)
//                          A2sin = 2/960 * CurVol200msNoFltrSinSOS_Sav[i+4]    (voltage)
//                          A2cos = 2/960 * CurVol200msNoFltrCosSOS_Sav[i+4]    (voltage)
//                      As mentioned before:
//                      cos(a-b) = cos(a)cos(b)+sin(a)sin(b) = adja/hypa * adjb/hypb + oppa/hypa * oppb/hypb
//                               = (adja * adjb + oppa * oppb)/(hypa * hypb)
//                      adja = A1cos = K * Icos, where K = 2/960 and Icos = CurVol200msNoFltrCosSOS_Sav[i]
//                      oppa = A1sin = K * Isin, where K = 2/960 and Isin = CurVol200msNoFltrSinSOS_Sav[i]
//                      adjb = A2cos = K * Vcos, where K = 2/960 and Vcos = CurVol200msNoFltrCosSOS_Sav[i+4]
//                      oppb = A2sin = K * Vsin, where K = 2/960 and Vcos = CurVol200msNoFltrCosSOS_Sav[i+4]
//                      hypa = SQRT[A1sin^2 + A1cos^2] = K * SQRT(Isin^2 + Icos^2)
//                      hypb = SQRT[A2sin^2 + A2cos^2] = K * SQRT(Vsin^2 + Vcos^2)
//                      Simpifying:
//                  cos(a-b) = [(K^2)*(Icos*Vcos + Isin*Vsin)]/K^2*SQRT[(Isin^2 + Icos^2)*(Vsin^2 + Vcos^2)]
//                           = [(Icos*Vcos + Isin*Vsin)]/SQRT[(Isin^2 + Icos^2)*(Vsin^2 + Vcos^2)]
//
//                      THD is computed using the formula SQRT[(RMS^2 - Fund^2)/Fund^2] * 100% and the real
//                      and imaginary components of the fundamental (that were obtained when computing
//                      displacement power factor)
//                      
//                      If the power is zero, the power factor is set to NAN to prevent divide by zero
//                      If the calculated power factor exceeds 1.00, it is clamped to 1.00 (or -1.00).
//
//  CAVEATS:            None
// 
//  INPUTS:             t200msecAV[] - pointer to input currents and voltages sums of squares
//                      CurVol200msNoFltrSinSOS_Sav[], CurVol200msNoFltrCosSOS_Sav[]
// 
//  OUTPUTS:            THD[], PF.Disp[], PF.MinDisp[], PF.MinDisp_TS[], PF.MaxDisp[], PF.MaxDisp_TS[]
//
//  ALTERS:             None
// 
//  CALLS:              Get_InternalTime(), sqrtf(), __disable_irq(), __enable_irq()
// 
//  EXECUTION TIME:     Measured on 190813 (rev 0.33 code): 13.6usec (with interrupts disabled)
//
//------------------------------------------------------------------------------------------------------------

void Calc_DispPF_THD(void)
{
  struct INTERNAL_TIME presenttime;
  float temp1, temp2, temp3, temp4, temp5, temp6;
  float t200msecSin[3], t200msecCos[3];
  uint8_t i;

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
 __enable_irq();

  // Begin the computations for the displacement PFs for Phase A, B, and C
  for (i=0; i<3; i++)
  {
    temp1 = (CurVol200msNoFltrSinSOS_Sav[i]*CurVol200msNoFltrSinSOS_Sav[i]                    // hypa^2
                    + CurVol200msNoFltrCosSOS_Sav[i]*CurVol200msNoFltrCosSOS_Sav[i]);
    temp2 = (CurVol200msNoFltrSinSOS_Sav[i+4]*CurVol200msNoFltrSinSOS_Sav[i+4]                // hypb^2
                    + CurVol200msNoFltrCosSOS_Sav[i+4]*CurVol200msNoFltrCosSOS_Sav[i+4]);
    
    // The values in temp1 and temp2 are used to compute THD, so compute THD now while they're handy
    // Compute the THD using the magnitudes in temp1 and temp2
    // Compute the RMS value of the fundamental frequency to get the THD.  This is done as follows:
    //   I(t) = input current = A1sin(wt) + A2(sin2wt) + A3sin(3wt) + ...
    //   I1(t) = the input current fundamental frequency
    //   A1 = the amplitude of I1(t)
    //   then 1/T*(integral over T)[I(t)*sin(t)]dt = 1/T*(integral over T)[A1/2dt] = A1/2
    // so A1sin = (2/960)*CurVol200msNoFltrSin[] and A1cos = (2/960)*CurVol200msNoFltrCos[]
    // the RMS value is A1/[SQRT(2)] = SQRT[( (A1sin)^2 + (A1cos)^2)/2]
    //   = SQRT[(2/960)^2 * (CurVol200msNoFltr[]^2 + CurVol200msNoFltr[]^2)/2]
    //   = SQRT[1/4.608E5 * temp1]

    // THD is SQRT[(IRMS^2 - I1^2)/I1^2] * 100
    //   = SQRT[(IRMS^2 - temp1/4.608E5)/(temp1/4.608E5)] * 100
    //   IRMS^2 = *t200msecAV[i]/960
    // so THD = SQRT[(*t200msecAV[i]/960 - temp1/4.608E5)/(temp1/4.608E5)] * 100
    //        = SQRT[(*t200msecAV[i] - temp1/480)/(temp1/480)] * 100
    temp3 = *t200msecAV[i];                 // THD for Ia, Ib, Ic
    temp4 = temp1/480.0F;

    if (temp3 >= temp4)
    {
      THD[i] = sqrtf((temp3 - temp4)/temp4) * 100.0F;
      if (THD[i] > THDminmax[i].THDmax)
      {
        THDminmax[i].THDmax = THD[i];
        THDminmax[i].THDmaxTS = presenttime;
      }
      if (THD[i] < THDminmax[i].THDmin)
      {
        THDminmax[i].THDmin = THD[i];
        THDminmax[i].THDminTS = presenttime;
      }
    }
    else
    {
      THD[i] = NAN;
    }
    temp3 = *t200msecAV[i+4];               // THD for Van, Vb, Vcn
    temp4 = temp2/480.0F;
    if (temp3 >= temp4)
    {
      THD[i+4] = sqrtf((temp3 - temp4)/temp4) * 100.0F;
      if (THD[i+4] > THDminmax[i+4].THDmax)
      {
        THDminmax[i+4].THDmax = THD[i+4];
        THDminmax[i+4].THDmaxTS = presenttime;
      }
      if (THD[i] < THDminmax[i].THDmin)
      {
        THDminmax[i+4].THDmin = THD[i+4];
        THDminmax[i+4].THDminTS = presenttime;
      }
    }
    else
    {
      THD[i+4] = NAN;
    }

    // Back to the displacement power factor...
    temp1 = (temp1 * temp2);            // temp1 = (hypa * hypb)^2
    if (temp1 != 0.0)
    {
      temp1 = sqrtf(temp1);
      temp1 = (CurVol200msNoFltrSinSOS_Sav[i] * CurVol200msNoFltrSinSOS_Sav[i+4]
                    + CurVol200msNoFltrCosSOS_Sav[i] * CurVol200msNoFltrCosSOS_Sav[i+4])/temp1;
      if ( temp1 > 1.0F)                    // Clamp at 1
      {
         temp1 = 1.0F;
      }
      // *** DAH  ADD CODE TO SET THE SIGN BASED ON THE SETPOINTS  (IEC, IEE, ALTERNATE IEEE)
      //  *** FOR NOW, JUST SAVE AS IS
      PF.Disp[i] = temp1;
    
      // Check for minimum and maximum - use absolute value for comparison.  Also use value as new min or max
      //   if the existing min or max value is invalid
      temp2 = ((PF.MinDisp[i] < 0.0F) ? (-1.0F * PF.MinDisp[i]) : PF.MinDisp[i]);
      if ( (temp1 < temp2) || (isnan(temp2)) )      // Check for min
      {
        PF.MinDisp[i] = PF.Disp[i];
        PF.MinDisp_TS[i] = presenttime;
        // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
      }
      temp2 = ((PF.MaxDisp[i] < 0) ? (-1.0F * PF.MaxDisp[i]) : PF.MaxDisp[i]);
      if ( (temp1 > temp2) || (isnan(temp2)) )        // Check for max
      {
        PF.MaxDisp[i] = PF.Disp[i];
        PF.MaxDisp_TS[i] = presenttime;
        // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
      }
    }
    else
    {
      PF.Disp[i] = NAN;
    }
  }
  // For the total displacement PF, we need to add the vectors for the 3 currents and 3 voltages, then
  //   compute the PF as was done above
  temp3 = CurVol200msNoFltrSinSOS_Sav[0] + CurVol200msNoFltrSinSOS_Sav[1]           // temp3 = totIsin
                + CurVol200msNoFltrSinSOS_Sav[2];
  temp4 = CurVol200msNoFltrCosSOS_Sav[0] + CurVol200msNoFltrCosSOS_Sav[2]           // temp4 = totIcos
                + CurVol200msNoFltrCosSOS_Sav[2];
  temp5 = CurVol200msNoFltrSinSOS_Sav[4] + CurVol200msNoFltrSinSOS_Sav[5]           // temp5 = totVsin
                + CurVol200msNoFltrSinSOS_Sav[6];
  temp6 = CurVol200msNoFltrCosSOS_Sav[4] + CurVol200msNoFltrCosSOS_Sav[5]           // temp6 = totVcos
                + CurVol200msNoFltrCosSOS_Sav[6];
  temp1 = sqrtf(temp3*temp3 + temp4*temp4);                 // hypa
  temp2 = sqrtf(temp5*temp5 + temp6*temp6);                 // hypb
  if ( (temp1 != 0.0) && (temp2 != 0.0) )
  {
    temp1 = (temp3 * temp5 + temp4 * temp6)/(temp1 *temp2);
    if ( temp1 > 1.0F)                                      // Clamp at 1
    {
       temp1 = 1.0F;
    }
    // *** DAH  ADD CODE TO SET THE SIGN BASED ON THE SETPOINTS  (IEC, IEE, ALTERNATE IEEE)
    //  *** FOR NOW, JUST SAVE AS IS
    PF.Disp[3] = temp1;
    
    // Check for minimum and maximum - use absolute value for comparison.  Also use value as new min or max
    //   if the existing min or max value is invalid
    temp2 = ((PF.MinDisp[3] < 0.0F) ? (-1.0F * PF.MinDisp[3]) : PF.MinDisp[3]);
    if ( (temp1 < temp2) || (isnan(temp2)) )      // Check for min
    {
      PF.MinDisp[3] = PF.Disp[3];
      PF.MinDisp_TS[3] = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    temp2 = ((PF.MaxDisp[3] < 0.0F) ? (-1.0F * PF.MaxDisp[3]) : PF.MaxDisp[3]);
    if ( (temp1 > temp2) || (isnan(temp2)) )        // Check for max
    {
      PF.MaxDisp[3] = PF.Disp[3];
      PF.MaxDisp_TS[3] = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
  }
  else
  {
    PF.Disp[3] = NAN;
  }

  // Do the THD for In
  temp1 = (CurVol200msNoFltrSinSOS_Sav[3]*CurVol200msNoFltrSinSOS_Sav[3]
                    + CurVol200msNoFltrCosSOS_Sav[3]*CurVol200msNoFltrCosSOS_Sav[3]);
  temp3 = *t200msecAV[3];
  temp4 = temp1/480.0F;
  if (temp3 >= temp4)
  {
    THD[3] = sqrtf((temp3 - temp4)/temp4) * 100.0F;
    if (THD[3] > THDminmax[3].THDmax)
    {
      THDminmax[3].THDmax = THD[3];
      THDminmax[3].THDmaxTS = presenttime;
    }
    if (THD[3] < THDminmax[3].THDmin)
    {
      THDminmax[3].THDmin = THD[3];
      THDminmax[3].THDminTS = presenttime;
    }
  }
  else
  {
    THD[3] = NAN;
  }

  // Do the THD for Vab, Vbc, and Vca
    // The sine and cosine components for the line-to-line voltages are just the differences between the
  //   line-to-neutral voltages
  t200msecSin[0] = CurVol200msNoFltrSinSOS_Sav[4] - CurVol200msNoFltrSinSOS_Sav[5];
  t200msecSin[1] = CurVol200msNoFltrSinSOS_Sav[5] - CurVol200msNoFltrSinSOS_Sav[6];
  t200msecSin[2] = CurVol200msNoFltrSinSOS_Sav[6] - CurVol200msNoFltrSinSOS_Sav[4];
  t200msecCos[0] = CurVol200msNoFltrCosSOS_Sav[4] - CurVol200msNoFltrCosSOS_Sav[5];
  t200msecCos[1] = CurVol200msNoFltrCosSOS_Sav[5] - CurVol200msNoFltrCosSOS_Sav[6];
  t200msecCos[2] = CurVol200msNoFltrCosSOS_Sav[6] - CurVol200msNoFltrCosSOS_Sav[4];
for (i=0; i<3; ++i)
  {
    temp1 = sqrtf(t200msecSin[i]*t200msecSin[i] + t200msecCos[i]*t200msecCos[i]);
    temp3 = *t200msecAV[i];
    temp4 = temp1/480.0F;
    if (temp3 >= temp4)
    {
      THD[i+7] = sqrtf((temp3 - temp4)/temp4) * 100.0F;
      if (THD[i+7] > THDminmax[i+7].THDmax)
      {
        THDminmax[i+7].THDmax = THD[i+7];
        THDminmax[i+7].THDmaxTS = presenttime;
      }
      if (THD[i] < THDminmax[i].THDmin)
      {
        THDminmax[i+7].THDmin = THD[i+7];
        THDminmax[i+7].THDminTS = presenttime;
      }
    }
    else
    {
      THD[i+7] = NAN;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_DispPF_THD()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_CF()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Crest Factor Calculations
//
//                      This subroutine performs the crest factor calculations
//
//  MECHANICS:          This subroutine performs the crest factor calculations as follows:
//                          - In the sampling interrupt, DMA1_Stream0_IRQHandler(), peak currents are stored
//                          - In the sampling interrupt, each one-cycle anniversary the peak currents are
//                            saved and the running peak currents are reset
//                          - Each one-cycle anniversary, this subroutine computes the one-cycle crest
//                            factor by dividing the peak current by the one-cycle RMS current.  The
//                            one-cycle crest factor is added to a running crest-factor sum
//                          - Each 200msec anniversary, this subroutine computes the crest factor by taking
//                            the average of the 12 one-cycle crest factors (dividing the crest-factor sum
//                            by 12).  The crest-factor sum is then reset.
//
//                      Note, unfiltered samples and currents are used to compute the crest factor
//
//  CAVEATS:            This subroutine must be called after Calc_Prot_Current() so that one-cycle RMS
//                      currents are computed before being used by this subroutine!!!
//                      
//  INPUTS:             CF_PeakSav.Ix - One-cycle peak currents
//                      CurOneCyc.Ix - One-cycle (unfiltered) RMS current
// 
//  OUTPUTS:            CF.Ix
//
//  ALTERS:             CF_Sum.Ix
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 180813 (rev 0.33 code):
//                          1.80usec without interrupts (interrupts were disabled around the subroutine)
//
//------------------------------------------------------------------------------------------------------------

void Calc_CF(void)
{

  // Add one-cycle crest factors to the running sums
  CF_Sum.Ia += (CF_PeakSav.Ia/CurOneCyc.Ia);
  CF_Sum.Ib += (CF_PeakSav.Ib/CurOneCyc.Ib);
  CF_Sum.Ic += (CF_PeakSav.Ic/CurOneCyc.Ic);
  CF_Sum.In += (CF_PeakSav.In/CurOneCyc.In);

  // If 200msec anniverary, compute the crest factors by taking the average of the one-cycle crest factors
  //   and reset the sums
  if (msec200Anniv)
  {
    CF.Ia = (CF_Sum.Ia/12.0F);
    CF.Ib = (CF_Sum.Ib/12.0F);
    CF.Ic = (CF_Sum.Ic/12.0F);
    CF.In = (CF_Sum.In/12.0F);
    CF_Sum.Ia = 0;
    CF_Sum.Ib = 0;
    CF_Sum.Ic = 0;
    CF_Sum.In = 0;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_CF()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_KFactor()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           K-Factor Calculations
//
//                      This subroutine performs the K-Factor calculations
//
//  MECHANICS:          This subroutine performs the K-Factor calculations whenever the K_FactorReq flag is
//                      True.  This flag is set in Calc_Harmonics() when harmonics calculations have been
//                      completed.  The K-Factor is computed for Ia, Ib, and Ic using the following formula:
//                          KF = (h1^2 + 4*h2^2 + 9*h3^2 + n^2*hn^2)/(h1^2 + h2^2 + h3^2 + ... + hn^2)
//                              where hk is the kth harmonic
//                                    n is the number of harmonics
//                      The aggregated harmonics are used for this computation.  Since the harmonics are
//                      computed every 600msec, the K-Factor is updated every 600msec.  Additionally, the
//                      harmonics take approximately 240msec to compute (assuming an 8msec main loop).  This
//                      leaves 360msec to compute the K-Factors before the harmonics are overwritten with
//                      new values.  Again, assuming an 8msec main loop time, this leaves us 45 total
//                      passes, or 15 passes per value, to compute the K-Factors.
//                      This subroutine is called each time through the main loop, but is "sliced" up so
//                      that each call takes a small amount of time.
//
//                      Note, as with the harmonics, unfiltered samples are used to compute the K-Factor
//
//  CAVEATS:            The computations begin after Calc_Harmonics() has completed updating the harmonics.
//                      This subroutine must be finished before new harmonics computations are started.
//                      
//  INPUTS:             K_FactorReq - Flag to begin computations
//                      HarmonicsAgg.Ix - Aggregated harmonics
// 
//  OUTPUTS:            KF.IxVal
//
//  ALTERS:             KF.State, KF.HarmPtr, KF.DivisorSum, KF.DividendSum
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 180813 (rev 0.33 code):
//                        3.30usec without interrupts (interrupts were disabled around the subroutine)
//
//------------------------------------------------------------------------------------------------------------

void Calc_KFactor(void)
{
  uint16_t i, j;
  float temp;

  switch (KF.State)
  {
    case 0:                             // Idle State
      if (K_FactorReq)                      // If time to update values, increment state and fall into the
      {                                     //   next state
        KF.State++;
        KF.HarmPtr = &HarmonicsAgg.Ia[0];
        KF.DivisorSum = 0;
        KF.DividendSum = 0;
      }
      else                                  // Otherwise exit
      {
        break;
      }

    case 1:                             // Computations
    case 5:
    case 9:
      for (i=0; i<10; ++i)
      {
        // Note, the max harmonic value is 1000 (100% in tenths of a per cent), so the max squared value is
        //   1E6 - need to convert to floats
        j = i + 1;
        temp = (float)(KF.HarmPtr[i]);
        temp = temp * temp;
        KF.DivisorSum += temp;
        KF.DividendSum += ((j * j) * temp);
      }
      KF.State++;
      break;

    case 2:                             // Computations
    case 6:
    case 10:
      for (i=10; i<20; ++i)
      {
        j = i + 1;
        temp = (float)(KF.HarmPtr[i]);
        temp = temp * temp;
        KF.DivisorSum += temp;
        KF.DividendSum += ((j * j) * temp);
      }
      KF.State++;
      break;

    case 3:                             // Computations
    case 7:
    case 11:
      for (i=20; i<30; ++i)
      {
        j = i + 1;
        temp = (float)(KF.HarmPtr[i]);
        temp = temp * temp;
        KF.DivisorSum += temp;
        KF.DividendSum += ((j * j) * temp);
      }
      KF.State++;
      break;

    case 4:                             // Computations
    case 8:
    case 12:
      for (i=30; i<39; ++i)
      {
        j = i + 1;
        temp = (float)(KF.HarmPtr[i]);
        temp = temp * temp;
        KF.DivisorSum += temp;
        KF.DividendSum += ((j * j) * temp);
      }
      if (KF.State == 4)
      {
        KF_Val.Ia = KF.DividendSum/KF.DivisorSum;
        KF.HarmPtr = &HarmonicsAgg.Ib[0];
        KF.DivisorSum = 0;
        KF.DividendSum = 0;
        KF.State++;
      }
      else if (KF.State == 8)
      {
        KF_Val.Ib = KF.DividendSum/KF.DivisorSum;
        KF.HarmPtr = &HarmonicsAgg.Ic[0];
        KF.DivisorSum = 0;
        KF.DividendSum = 0;
        KF.State++;
      }
      else
      {
        KF_Val.Ic = KF.DividendSum/KF.DivisorSum;
        K_FactorReq = FALSE;
        KF.State = 0;
      }
      break;

    default:
      K_FactorReq = FALSE;
      KF.State = 0;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_KFactor()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_SeqComp_PhAng()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Sequence Components Calculation
//
//                      This subroutine computes the sequence components for Ia thru Ic, and VanAFE thru
//                      VcnAFE over the most recent waveform in the Sample Buffer (SampleBuf[ ]).  The
//                      unfiltered samples are used - these are the only samples that are available and
//                      are preferred anyway, since we don't want any transients filtered out (the sequence
//                      components are desired during a fault or motor start-up condition).
//                      In addition, this subroutine computes the phase angles for currents Ia thru Ic,
//                      VanAFE thru VcnAFE, and VanADC thru VcnADC.  The phase angles are computed by taking
//                      the arctangent of the imaginary component divided by the real component of the
//                      waveform.  These are already computed for the currents and AFE voltages as part of
//                      the sequence components computations, so it makes sense to compute the phase angles
//                      here.  The ADC voltages are computed separately.
//
//  MECHANICS:          Sequence components are defined by the following theorem:
//                      Current inputs Ia, Ib, and Ic can be resolved into three different vectors,
//                      I0 (zero sequence component), I1 (positive sequence component), and I2 (negative
//                      sequence component).  These sequence components are derived from the three inputs by
//                      the following equations:
//                          I0 = (Ia + Ib + Ic)/3
//                          I1 = (Ia + a*Ib + (a^2)*Ic)/3
//                          I2 = (Ia + (a^2)*Ib + a*Ic)/3
//                              where "a" is a unit vector with a 120 degree phase shift (magnitude = 1,
//                                    angle = 120 degrees), and "a^2" is a unit vector with a 240 degree
//                                    phase shift
//                      Note, these are vector additions and multiplications
//                      The following steps are taken:
//                        1) Extract the real and imaginary components for the waveform by multiplying the
//                           waveform by a unit sin and cosine.  Take the average and double it to get the
//                           amplitude (if RMS, would multiply by sqrt(2)).  I'm not exactly sure why, but
//                           the sin multiplication extracts the real component, and the cosine
//                           multiplication extracts the imaginary component
//                        2) Compute a*Ix and (a^2)*Ix terms
//                           a = cos(120) + jsin(120) = -0.5 + j0.866
//                           a^2 = -0.5 - j0.866
//                           Re{a*Ix} = [Re{Ix}*(-0.5) - Im{Ix}*(0.866)]  (the Im{Ix} has a j-term)
//                           Im{a*Ix} = [Re{Ix}*(0.866) + Im{Ix}*(-0.5)]
//                           Re{(a^2)*Ix} = [Re{Ix}*(-0.5) + Im{Ix}*(0.866)]  (the Im{Ix} has a j-term)
//                           Im{(a^2)*Ix} = [Re{Ix}*(-0.866) + Im{Ix}*(-0.5)]
//                        3) Perform the vector additions for the sequence components
//                             Re{Io} = (Re{Ia}+Re{Ib}+Re{Ic})
//                             Im{Io} = (Im{Ia}+Im{Ib}+Im{Ic})
//                             Re{I1} = (Re{Ia}+Re{a*Ib}+Re{(a^2)*Ic})
//                               = [Re{Ia} + [Re{Ib}*(-0.5)-Im{Ib}*(0.866)] + [Re{Ic}*(-0.5)+Im{Ic}*(0.866)]
//                             Im{I1} = (Im{Ia}+Im{a*Ib}+Im{(a^2)*Ic})
//                               = [Im{Ia} + [Re{Ib}*(0.866)+Im{Ib}*(-0.5)] + [Re{Ic}*(-0.866)+Im{Ic}*(-0.5)]
//                             Re{I2} = (Re{Ia}+Re{(a^2)*Ib}+Re{a*Ic})
//                               = [Re{Ia} + [Re{Ib}*(-0.5)+Im{Ib}*(0.866)] + [Re{Ic}*(-0.5)-Im{Ic}*(0.866)]
//                             Im{I2} = (Im{Ia}+Im{(a^2)*Ib}+Im{a*Ic})
//                               = [Im{Ia} + [Re{Ib}*(-0.866)+Im{Ib}*(-0.5)] + [Re{Ic}*(0.866)+Im{Ic}*(-0.5)]
//                        4) Compute the sequence components
//                             I0 magnitude = sqrt[(Re{Ia}+Re{Ib}+Re{Ic})^2 + (Im{Ia}+Im{Ib}+Im{Ic})^2]/3
//                             I0 phase = arctan[(Im{Ia}+Im{Ib}+Im{Ic})/(Re{Ia}+Re{Ib}+Re{Ic})]
//                             I1 magnitude = sqrt[(Re{I1})^2 + (Im{I1})^2]/3
//                             I1 phase = arctan[(Im{I1})/(Re{I1})]
//                             I2 magnitude = sqrt[(Re{I2})^2 + (Im{I2})^2]/3
//                             I2 phase = arctan[(Im{I2})/(Re{I2})]
//
//  CAVEATS:            None
// 
//  INPUTS:             SampleBuf[].Ia, .Ib, .Ic, .VanAFE, .VbnAFE, .VcnAFE - input samples
//                      SampleIndex - next open index in SampleBuf[]
// 
//  OUTPUTS:            SeqComp.I_ZeroMag, .I_ZeroPh, .I_PosMag, .I_PosPh, .I_NegMag, .I_NegPh,
//                      SeqComp.V_ZeroMag, .V_ZeroPh, .V_PosMag, .V_PosPh, .V_NegMag, .V_NegPh,
//                      PhAngles1[]
//
//  ALTERS:             None
// 
//  CALLS:              sqrtf(), atanf()
// 
//  EXECUTION TIME:     Measured on 190820 (rev 0.33 code): 226usec (with interrupts disabled)
//
//------------------------------------------------------------------------------------------------------------

void Calc_SeqComp_PhAng(void)
{
  float temp, re_zero_seq, im_zero_seq, re_pos_seq, im_pos_seq, re_neg_seq, im_neg_seq;
  float re_temp[6], im_temp[6];//_ia, re_temp_ib, re_temp_ic, im_temp_ia, im_temp_ib, im_temp_ic;
//  float re_temp_van, re_temp_vbn, re_temp_vcn, im_temp_van, im_temp_vbn, im_temp_vcn;
  uint16_t ndx, ndx1;
  uint8_t i, j;

  // *** DAH  SHOULD PROBABLY ADD CODE TO ONLY CALCULATE IF THE RMS CURRENT OR VOLTAGE VALUE IS ABOVE A CERTAIN LIMIT - OTHERWISE SET TO NAN

  // Compute the starting sample index in SampleBuf[].  This is the present index - 80, accounting for
  // rollover
  ndx = SampleIndex;                    // Capture the present sample index
  ndx1 = ndx;                           // Save the captured sample index
  ndx = ( (ndx >= 80) ? (ndx - 80) : (TOTAL_SAMPLE_SETS + ndx - 80) );
  // Extract the real and imaginary components of each waveform by multiplying them by a unit sin and cosine
  for (i=0; i<6; i++)
  {
    re_temp[i] = 0.0f;
    im_temp[i] = 0.0f;
  }
  for (i=0; i<80; i++)
  {
    temp = SIN_COEFF[i];
    re_temp[0] += (SampleBuf[ndx].Ia * temp);
    re_temp[1] += (SampleBuf[ndx].Ib * temp);
    re_temp[2] += (SampleBuf[ndx].Ic * temp);
    re_temp[3] += (SampleBuf[ndx].VanAFE * temp);
    re_temp[4] += (SampleBuf[ndx].VbnAFE * temp);
    re_temp[5] += (SampleBuf[ndx].VcnAFE * temp);

    j = ((i <= 59) ? (i + 20) : (i - 60));           // Cos index = sin index + 90deg (20*4.5)
    temp = SIN_COEFF[j];                             //   with rollover at 80
    im_temp[0] += (SampleBuf[ndx].Ia * temp);
    im_temp[1] += (SampleBuf[ndx].Ib * temp);
    im_temp[2] += (SampleBuf[ndx].Ic * temp);
    im_temp[3] += (SampleBuf[ndx].VanAFE * temp);
    im_temp[4] += (SampleBuf[ndx].VbnAFE * temp);
    im_temp[5] += (SampleBuf[ndx].VcnAFE * temp);
    if (++ndx >= TOTAL_SAMPLE_SETS)                  // Increment ndx with rollover check
    {
      ndx = 0;
    }
  }
  for (i=0; i<6; i++)
  {
    re_temp[i] = re_temp[i]/40;
    im_temp[i] = im_temp[i]/40;
  }
  // First do the currents
  re_zero_seq = re_temp[0] + re_temp[1] + re_temp[2];
  im_zero_seq = im_temp[0] + im_temp[1] + im_temp[2];
  re_pos_seq = re_temp[0] + (-0.5f * re_temp[1]) - (0.866025404f * im_temp[1])
                    + (-0.5f * re_temp[2]) + (0.866025404f * im_temp[2]);
  im_pos_seq = im_temp[0] + (0.866025404f * re_temp[1]) - (0.5f * im_temp[1])
                    + (-0.866025404f * re_temp[2]) - (0.5f * im_temp[2]);
  re_neg_seq = re_temp[0] + (-0.5f * re_temp[1]) + (0.866025404f * im_temp[1])
                    + (-0.5f * re_temp[2]) - (0.866025404f * im_temp[2]);
  im_neg_seq = im_temp[0] + (-0.866025404f * re_temp[1]) - (0.5f * im_temp[1])
                    + (0.866025404f * re_temp[2]) - (0.5f * im_temp[2]);

  SeqComp.I_ZeroMag = (sqrtf(re_zero_seq * re_zero_seq + im_zero_seq * im_zero_seq))/3;
  if (im_zero_seq < 0.0001f)
  {
    SeqComp.I_ZeroPh = 0.0f;
  }
  else if (re_zero_seq < 0.0001f)
  {
    SeqComp.I_ZeroPh = 90.0f;
  }
  else
  {
    SeqComp.I_ZeroPh = atanf(im_zero_seq/re_zero_seq);
  }
  SeqComp.I_PosMag = (sqrtf(re_pos_seq * re_pos_seq + im_pos_seq * im_pos_seq))/3;
  if (im_pos_seq < 0.0001f)
  {
    SeqComp.I_PosPh = 0.0f;
  }
  else if (re_pos_seq < 0.0001f)
  {
    SeqComp.I_PosPh = 90.0f;
  }
  else
  {
    SeqComp.I_PosPh = atanf(im_pos_seq/re_pos_seq);
  }
  SeqComp.I_NegMag = (sqrtf(re_neg_seq * re_neg_seq + im_neg_seq * im_neg_seq))/3;
  if (im_neg_seq < 0.0001f)
  {
    SeqComp.I_NegPh = 0.0f;
  }
  else if (re_neg_seq < 0.0001f)
  {
    SeqComp.I_NegPh = 90.0f;
  }
  else
  {
    SeqComp.I_NegPh = atanf(im_neg_seq/re_neg_seq);
  }
  // Now do the voltages
  re_zero_seq = re_temp[3] + re_temp[4] + re_temp[5];
  im_zero_seq = im_temp[3] + im_temp[4] + im_temp[5];
  re_pos_seq = re_temp[3] + (-0.5f * re_temp[4]) - (0.866025404f * im_temp[4])
                    + (-0.5f * re_temp[5]) + (0.866025404f * im_temp[5]);
  im_pos_seq = im_temp[3] + (0.866025404f * re_temp[4]) - (0.5f * im_temp[4])
                    + (-0.866025404f * re_temp[5]) - (0.5f * im_temp[5]);
  re_neg_seq = re_temp[3] + (-0.5f * re_temp[4]) + (0.866025404f * im_temp[4])
                    + (-0.5f * re_temp[5]) - (0.866025404f * im_temp[5]);
  im_neg_seq = im_temp[3] + (-0.866025404f * re_temp[4]) - (0.5f * im_temp[4])
                    + (0.866025404f * re_temp[5]) - (0.5f * im_temp[5]);

  SeqComp.V_ZeroMag = (sqrtf(re_zero_seq * re_zero_seq + im_zero_seq * im_zero_seq))/3;
  if (im_zero_seq < 0.0001f)
  {
    SeqComp.V_ZeroPh = 0.0f;
  }
  else if (re_zero_seq < 0.0001f)
  {
    SeqComp.V_ZeroPh = 90.0f;
  }
  else
  {
    SeqComp.V_ZeroPh = atanf(im_zero_seq/re_zero_seq);
  }
  SeqComp.V_PosMag = (sqrtf(re_pos_seq * re_pos_seq + im_pos_seq * im_pos_seq))/3;
  if (im_pos_seq < 0.0001f)
  {
    SeqComp.V_PosPh = 0.0f;
  }
  else if (re_pos_seq < 0.0001f)
  {
    SeqComp.V_PosPh = 90.0f;
  }
  else
  {
    SeqComp.V_PosPh = atanf(im_pos_seq/re_pos_seq);
  }
  SeqComp.V_NegMag = (sqrtf(re_neg_seq * re_neg_seq + im_neg_seq * im_neg_seq))/3;
  if (im_neg_seq < 0.0001f)
  {
    SeqComp.V_NegPh = 0.0f;
  }
  else if (re_neg_seq < 0.0001f)
  {
    SeqComp.V_NegPh = 90.0f;
  }
  else
  {
    SeqComp.V_NegPh = atanf(im_neg_seq/re_neg_seq);
  }

  // Compute the phase angles for the currents and line-to-neutral AFE voltages.  This is just the
  //   arctangent of the angle formed by the real and imaginary component vectors, adjusted for the quadrant
  for (i=0; i<6; ++i)
  {
    temp = ((im_temp[i] < 0.0f) ? (im_temp[i] * -1.0f) : (im_temp[i]));
    if (temp < 0.0001f)
    {
      PhAngles1[i] = 0.0f;
    }
    else
    {
      temp = ((re_temp[i] < 0.0f) ? (re_temp[i] * -1.0f) : (re_temp[i]));
      if (temp < 0.0001f)
      {
        PhAngles1[i] = 90.0f;
      }
      else
      {
        PhAngles1[i] = 180.0 * atanf(im_temp[i]/re_temp[i])/3.141592659f;
      }
      // Adjust for the quadrant.  If real and imaginary are negative, move from 1st to 3rd quadrant by
      //   subtracting 180.  If real is negative and imaginary is positive, move from 4st to 2nd quadrant by
      //   adding 180.  Otherwise, value is ok as is.
      if (re_temp[i] < 0)
      {
        PhAngles1[i] = ((im_temp[i] < 0) ? (PhAngles1[i] - 180.0f) : (PhAngles1[i] +180.0f));
      }
    }
  }
  //Compute the line-to-line phase angles for the AFE voltages
  PhAngles1[6] = PhAngles1[4] - PhAngles1[3];       // Pha-b = Phb - Pha
  PhAngles1[7] = PhAngles1[5] - PhAngles1[4];       // Phb-c = Phc - Phb
  PhAngles1[8] = PhAngles1[3] - PhAngles1[5];       // Phc-a = Pha - Phc

  // Compute the phase angles for the ADC voltages.  First need to extract the real and imaginary
  //   components.
  // Extract the real and imaginary components of each waveform by multiplying them by a unit sin and cosine
  for (i=0; i<3; i++)
  {
    re_temp[i] = 0.0f;
    im_temp[i] = 0.0f;
  }
  for (i=0; i<80; i++)
  {
    temp = SIN_COEFF[i];
    re_temp[0] += (SampleBuf[ndx1].VanADC * temp);
    re_temp[1] += (SampleBuf[ndx1].VbnADC * temp);
    re_temp[2] += (SampleBuf[ndx1].VcnADC * temp);

    j = ((i <= 59) ? (i + 20) : (i - 60));           // Cos index = sin index + 90deg (20*4.5)
    temp = SIN_COEFF[j];                             //   with rollover at 80
    im_temp[0] += (SampleBuf[ndx1].VanADC * temp);
    im_temp[1] += (SampleBuf[ndx1].VbnADC * temp);
    im_temp[2] += (SampleBuf[ndx1].VcnADC * temp);
    if (++ndx1 >= TOTAL_SAMPLE_SETS)                 // Increment ndx with rollover check
    {
      ndx1 = 0;
    }
  }
  for (i=0; i<3; i++)
  {
    re_temp[i] = re_temp[i]/40;
    im_temp[i] = im_temp[i]/40;
  }
  // Compute the phase angles for the ADC voltages
  for (i=0; i<3; ++i)
  {
    temp = ((im_temp[i] < 0.0f) ? (im_temp[i] * -1.0f) : (im_temp[i]));
    if (temp < 0.0001f)
    {
      PhAngles2[i] = 0.0f;
    }
    else
    {
      temp = ((re_temp[i] < 0.0f) ? (re_temp[i] * -1.0f) : (re_temp[i]));
      if (temp < 0.0001f)
      {
        PhAngles2[i] = 90.0f;
      }
      else
      {
        PhAngles2[i] = 180.0 * atanf(im_temp[i]/re_temp[i])/3.141592659f;
      }
      // Adjust for the quadrant.  If real and imaginary are negative, move from 1st to 3rd quadrant by
      //   subtracting 180.  If real is negative and imaginary is positive, move from 4st to 2nd quadrant by
      //   adding 180.  Otherwise, value is ok as is.
      if (re_temp[i] < 0)
      {
        PhAngles2[i] = ((im_temp[i] < 0) ? (PhAngles2[i] - 180.0f) : (PhAngles2[i] +180.0f));
      }
    }
  }
  //Compute the line-to-line phase angles for the ADC voltages
  PhAngles2[3] = PhAngles2[0] - PhAngles2[1];       // Pha-b = Pha - Phb
  PhAngles2[4] = PhAngles2[1] - PhAngles2[2];       // Phb-c = Phb - Phc
  PhAngles2[5] = PhAngles2[2] - PhAngles2[0];       // Phc-a = Phc - Pha

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_SeqComp_PhAng()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Freq()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Frequency Calculations
//
//                      This subroutine performs the frequency calculations for both the line-side and
//                      load-side.
//
//  MECHANICS:          Every zero-crossing of the load Van, Timer 10 ISR TIM1_UP_TIM10_IRQHandler() updates
//                      the delta value in FreqLoad.DeltaTim.  Similarly, every zero-crossing of the line
//                      Van, Timer 11 ISR TIM1_TRG_COM_TIM11_IRQHandler() updates the delta value in
//                      FreqLine.DeltaTim.  Freqxx.DeltaTim contains the number of 1.5MHz timer ticks
//                      between the rising edges of Van.  To compute frequency: (1,500,000/delta).
//
//                      The Timer 10 and Timer 11 interrupt routines are capable of measuring frequencies in
//                      the range 33Hz ... 2.0kHz.  Frequencies below 33Hz will provide a DeltaTim of
//                      0xFFFF.  Frequencies above 2.0kHz will be ignored and the measured period extended
//                      by the "runt" cycle.  Ultimately, this will accumulate to a period equivalent to
//                      2.0kHz.  The output behavior for increasing frequency is thus:
//                               "invalid" -> 33Hz...2.0kHz -> "2.0kHz saturation"
//
//                      The routine stores the metered frequency as a floating point number in Hz, limiting
//                      the displayable frequency range to 33Hz ... 2.0KHz.  Frequency is computed as
//                      follows:
//                        - The frequency is set to "invalid" (NAN) if the voltage is below 25
//                        - The frequency is set to "invalid" (NAN) if the delta value measured by the
//                          interrupt is set to "invalid" (0xFFFF).
//                        - The frequency is computed by dividing 1,500,000 by the measured delta time 
//                          (DeltaTim).  If the value is greater than 2.0KHz, it is clamped to 2.0KHz.
//
//                      In order to prevent a single transient cycle from setting a new max/min (as done
//                      when changing the frequency on a tester like the AVO), compare the new frequency 
//                      with the previously set value.
//                         - If the difference is more than 0.10Hz, do not use the reading unless two
//                           consecutive measurements have been tossed.
//                         - If the difference is less than 0.10Hz, use the new reading & reset the dump
//                           counter.
//                      If a legitimate frequency was calculated, the metered value is compared against
//                      historical max & min values and updated if a new max or min has been set.  Any new
//                      max/min values are time-stamped.
//                          
//  CAVEATS:            Call after Calc_Volts()
//                      
//  INPUTS:             *f_ptr - pointer to the line or load frequency structure
//                      v_xn - l-n voltage of the channel used to measure the frequency
//                      f_ptr->DeltaTim
// 
//  OUTPUTS:            f_ptr->FreqVal
//
//  ALTERS:             f_ptr->Dump_Count
// 
//  CALLS:              Get_InternalTime(), __disable_irq(), __enable_irq()
// 
//  EXECUTION TIME:     Measured on 180826 (rev 0.34 code):
//                        2.25usec without interrupts (interrupts were disabled around the subroutine)
//
//------------------------------------------------------------------------------------------------------------

void Calc_Freq(struct FREQ_MEASURE_VARS *f_ptr, float v_xn)
{
  struct INTERNAL_TIME presenttime;
  float new_freq, f_diff;
  volatile float i = v_xn;        // just to get rid of warning in compiler
  
  __disable_irq();                              // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  // If voltage is less than 25, assume cannot measure accurately enough so set frequency to invalid
  // Also set frequency invalid if there is an invalid delta time
  if ( /*(v_xn < 25.0f) ||*/ (f_ptr->DeltaTim == 0xFFFF) )            // *** DAH COMMENT OUT VOLTAGE CHECK FOR NOW
  {
    f_ptr->FreqVal = NAN;
  }
  else                                          // Otherwise compute the frequency
  {
    new_freq = 1.50E6f/(float)f_ptr->DeltaTim;
    if (new_freq > 2.0E3f)                      // Clamp at 2.0KHz
    {
      new_freq = 2.0E3f;
    }
    f_diff = (new_freq - f_ptr->FreqVal);                       // Compute absolute value of the difference
    f_diff = ( (f_diff < 0) ? (-1.0f * f_diff) : (f_diff) );    //   between new and old frequencies

    if (f_diff > 0.10f)                         // If difference is greater than 0.10Hz, check how many
    {                                           //   consecutive readings have been ignored
      if (f_ptr->Dump_Count++ > 2)              // If greater than 2, use new value anyway and reset the
      {                                         //   dump counter
        f_ptr->FreqVal = new_freq;
        f_ptr->Dump_Count = 0;
      }
    }
    else
    {                                           // If difference is less than 0.10Hz, use new frequency
      f_ptr->FreqVal = new_freq;                //   value and reset the dump counter
      f_ptr->Dump_Count = 0;
    }

    // Update min and max.  Set if new min or max or if the old min or max was invalid
    if ( (f_ptr->FreqVal < f_ptr->MinFreqVal) || (isnan(f_ptr->MinFreqVal)) )
    {
      f_ptr->MinFreqVal = f_ptr->FreqVal;
      f_ptr->MinFreqVal_TS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
    if ( (f_ptr->FreqVal > f_ptr->MaxFreqVal) || (isnan(f_ptr->MaxFreqVal)) )
    {
      f_ptr->MaxFreqVal = f_ptr->FreqVal;
      f_ptr->MaxFreqVal_TS = presenttime;
      // *** DAH ALSO ADD CODE TO TRANSMIT OUT CAM, ETC. SINCE NEW MIN
    }
  }
    
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Freq()
//------------------------------------------------------------------------------------------------------------


void SnapshotMeter(uint16_t SummaryCode)
{

  SnapshotMeteredValues.CauseCode = StatusCode;
  
  SnapshotMeteredValues.SummaryCode = SummaryCode;

  SnapshotMeteredValues.BinaryStatus = 0;


  SnapshotMeteredValues.Ia_Rms = CurOneCyc.Ia;
  SnapshotMeteredValues.Ib_Rms = CurOneCyc.Ib;
  SnapshotMeteredValues.Ic_Rms = CurOneCyc.Ic;
  SnapshotMeteredValues.In_Rms = CurOneCyc.In;
  SnapshotMeteredValues.Ig_Rms = CurOneCycIg;

  SnapshotMeteredValues.Van1_Rms = VolAFEOneCyc.Van;
  SnapshotMeteredValues.Vbn1_Rms = VolAFEOneCyc.Vbn;
  SnapshotMeteredValues.Vcn1_Rms = VolAFEOneCyc.Vcn;

  SnapshotMeteredValues.Vab1_Rms = VolAFEOneCyc.Vab;
  SnapshotMeteredValues.Vbc1_Rms = VolAFEOneCyc.Vbc;
  SnapshotMeteredValues.Vca1_Rms = VolAFEOneCyc.Vca;

  SnapshotMeteredValues.Vab2_Rms = VolADCOneCyc.Vab;
  SnapshotMeteredValues.Vbc2_Rms = VolADCOneCyc.Vbc;
  SnapshotMeteredValues.Vca2_Rms = VolADCOneCyc.Vca;

  SnapshotMeteredValues.Van2_Rms = VolADCOneCyc.Van;
  SnapshotMeteredValues.Vbn2_Rms = VolADCOneCyc.Vbn;
  SnapshotMeteredValues.Vcn2_Rms = VolADCOneCyc.Vcn;

  SnapshotMeteredValues.P_real = PwrOneCyc.Pa + PwrOneCyc.Pb + PwrOneCyc.Pc;
  SnapshotMeteredValues.P_react = PwrOneCyc.RPa + PwrOneCyc.RPb + PwrOneCyc.RPc;
  SnapshotMeteredValues.P_apparent = PwrOneCycApp.AppPa + PwrOneCycApp.AppPb + PwrOneCycApp.AppPc;

  SnapshotMeteredValues.Ia_Dmnd = EngyDmnd[1].DmndIa;
  SnapshotMeteredValues.Ib_Dmnd = EngyDmnd[1].DmndIb;
  SnapshotMeteredValues.Ic_Dmnd = EngyDmnd[1].DmndIc;
  SnapshotMeteredValues.In_Dmnd = EngyDmnd[1].DmndIn;

  SnapshotMeteredValues.P_real_Dmnd = EngyDmnd[1].DmndTotW;
  SnapshotMeteredValues.Pr_Dmnd = EngyDmnd[1].DmndTotVar;
  SnapshotMeteredValues.Pa_Dmnd = EngyDmnd[1].DmndTotVA;

  SnapshotMeteredValues.Temperature = THSensor.Temperature;

  SnapshotMeteredValues.f1  = FreqLoad.FreqVal;
  SnapshotMeteredValues.f2  = FreqLine.FreqVal;

  SnapshotMeteredValues.PFa = PF.App[0];
  SnapshotMeteredValues.PFb = PF.App[1];
  SnapshotMeteredValues.PFc = PF.App[2];
  SnapshotMeteredValues.PFtot = PF.App[3];

  SnapshotMeteredValues.THD_Ia = THD[0];
  SnapshotMeteredValues.THD_Ib = THD[1];
  SnapshotMeteredValues.THD_Ic = THD[2];
  SnapshotMeteredValues.THD_In = THD[3];

  SnapshotMeteredValues.THD_Van1 = THD[4];
  SnapshotMeteredValues.THD_Vbn1 = THD[5];
  SnapshotMeteredValues.THD_Vcn1 = THD[6];
  SnapshotMeteredValues.THD_Vab1 = THD[7];
  SnapshotMeteredValues.THD_Vbc1 = THD[8];
  SnapshotMeteredValues.THD_Vca1 = THD[9];

  SnapshotMeteredValues.CFa = CF.Ia;
  SnapshotMeteredValues.CFb = CF.Ib;
  SnapshotMeteredValues.CFc = CF.Ic;
  SnapshotMeteredValues.CFn = CF.In;

  SnapshotMeteredValues.OperationsCount = 0;

}

void ExtCapSnapshotMeterOneCyc()
{

// temporary test code to fill the vairables with the values
//  temp_i++;

//  ExtCapSnapshotMeteredValues.Ia_Rms = CurOneCyc.Ia = 10*temp_i;
//  ExtCapSnapshotMeteredValues.Ib_Rms = CurOneCyc.Ib = 20*temp_i;
//  ExtCapSnapshotMeteredValues.Ic_Rms = CurOneCyc.Ic = 30*temp_i;
//  ExtCapSnapshotMeteredValues.In_Rms = CurOneCyc.In = 40*temp_i;
//  ExtCapSnapshotMeteredValues.Ig_Rms = CurOneCycIg = 50*temp_i;
//
//  ExtCapSnapshotMeteredValues.Van1_Rms = VolAFEOneCyc.Van = 60*temp_i;
//  ExtCapSnapshotMeteredValues.Vbn1_Rms = VolAFEOneCyc.Vbn = 70*temp_i;
//  ExtCapSnapshotMeteredValues.Vcn1_Rms = VolAFEOneCyc.Vcn = 80*temp_i;
//
//  ExtCapSnapshotMeteredValues.Van2_Rms = VolADCOneCyc.Van = 90*temp_i;
//  ExtCapSnapshotMeteredValues.Vbn2_Rms = VolADCOneCyc.Vbn = 100*temp_i;
//  ExtCapSnapshotMeteredValues.Vcn2_Rms = VolADCOneCyc.Vcn = 110*temp_i;

  ExtCapSnapshotMeteredValues.Ia_Rms = CurOneCyc.Ia;
  ExtCapSnapshotMeteredValues.Ib_Rms = CurOneCyc.Ib;
  ExtCapSnapshotMeteredValues.Ic_Rms = CurOneCyc.Ic;
  ExtCapSnapshotMeteredValues.In_Rms = CurOneCyc.In;
  ExtCapSnapshotMeteredValues.Ig_Rms = CurOneCycIg;

  ExtCapSnapshotMeteredValues.Van1_Rms = VolAFEOneCyc.Van;
  ExtCapSnapshotMeteredValues.Vbn1_Rms = VolAFEOneCyc.Vbn;
  ExtCapSnapshotMeteredValues.Vcn1_Rms = VolAFEOneCyc.Vcn;

  ExtCapSnapshotMeteredValues.Van2_Rms = VolADCOneCyc.Van;
  ExtCapSnapshotMeteredValues.Vbn2_Rms = VolADCOneCyc.Vbn;
  ExtCapSnapshotMeteredValues.Vcn2_Rms = VolADCOneCyc.Vcn;
}

void ExtCapSnapshotMeterTwoHundred()
{

// temporary test code to fill the vairables with the values
//  temp_i++;

//  ExtCapSnapshotMeteredValues.Ia_Rms = CurOneCyc.Ia = 10*temp_i;
//  ExtCapSnapshotMeteredValues.Ib_Rms = CurOneCyc.Ib = 20*temp_i;
//  ExtCapSnapshotMeteredValues.Ic_Rms = CurOneCyc.Ic = 30*temp_i;
//  ExtCapSnapshotMeteredValues.In_Rms = CurOneCyc.In = 40*temp_i;
//  ExtCapSnapshotMeteredValues.Ig_Rms = CurOneCycIg = 50*temp_i;
//
//  ExtCapSnapshotMeteredValues.Van1_Rms = VolAFEOneCyc.Van = 60*temp_i;
//  ExtCapSnapshotMeteredValues.Vbn1_Rms = VolAFEOneCyc.Vbn = 70*temp_i;
//  ExtCapSnapshotMeteredValues.Vcn1_Rms = VolAFEOneCyc.Vcn = 80*temp_i;
//
//  ExtCapSnapshotMeteredValues.Van2_Rms = VolADCOneCyc.Van = 90*temp_i;
//  ExtCapSnapshotMeteredValues.Vbn2_Rms = VolADCOneCyc.Vbn = 100*temp_i;
//  ExtCapSnapshotMeteredValues.Vcn2_Rms = VolADCOneCyc.Vcn = 110*temp_i;

  ExtCapSnapshotMeteredValues.Ia_Rms = Cur200msFltr.Ia;
  ExtCapSnapshotMeteredValues.Ib_Rms = Cur200msFltr.Ib;
  ExtCapSnapshotMeteredValues.Ic_Rms = Cur200msFltr.Ic;
  ExtCapSnapshotMeteredValues.In_Rms = Cur200msFltr.In;
  ExtCapSnapshotMeteredValues.Ig_Rms = Cur200msFltrIg;

  ExtCapSnapshotMeteredValues.Van1_Rms = VolAFE200msFltr.Van;
  ExtCapSnapshotMeteredValues.Vbn1_Rms = VolAFE200msFltr.Vbn;
  ExtCapSnapshotMeteredValues.Vcn1_Rms = VolAFE200msFltr.Vcn;

  ExtCapSnapshotMeteredValues.Van2_Rms = VolADC200ms.Van;
  ExtCapSnapshotMeteredValues.Vbn2_Rms = VolADC200ms.Vbn;
  ExtCapSnapshotMeteredValues.Vcn2_Rms = VolADC200ms.Vcn;
}


//[]
//------------------------------------------------------------------------------
//  START OF FUNCTION       Update_Std_Status()
//------------------------------------------------------------------------------
//  FUNCTION:   Creates the standard Trip-Unit Class binary status.
//
//  MECHANICS:  The standard trip-unit class status binaries are updated from various status flags
//              and GPIO states as follows:
//                  - Metered_Data.Std_TU_Status.Closed 
//                  - Metered_Data.Std_TU_Status.Trip
//                  - Metered_Data.Std_TU_Status.Alarm 
//                  - Metered_Data.Std_TU_Status.M_Mode 
//                  - Metered_Data.Std_TU_Status.T_Mode  
//                  - Metered_Data.Std_TU_Status.T_Forbid_Coil
//                  - Metered_Data.Std_TU_Status.Rotation_ABC
//                  - Metered_Data.Std_TU_Status.PickedUp            
//                  - Metered_Data.Std_TU_Status.Zin              
//                  - Metered_Data.Std_TU_Status.SRC_Gnd
//                  - Metered_Data.Std_TU_Status.T_Forbid_SecInj
//                  - Metered_Data.Std_TU_Status.Connected
//                  - Metered_Data.Std_TU_Status.Charged
//
uint16_t Update_Std_Status(union WORD_BITS *pStd_TU_Status)
{
    union WORD_BITS new_std, old_std;                   //temps

    old_std.all = pStd_TU_Status->all;

    //construct Std_TU_Status...
    new_std.all = 0;                                    //clear all status

                                                        
    //Closed -- b0 -- set to indicate breaker in close position
    new_std.bit.b0 = (OpenFlg == 0)? 1:0;
                                                       
    //Trip -- b1 -- set to indicate an un-acknowledge trip condition
    new_std.bit.b1 = ((Trip_Flags0.all & TRIP_MASK_FLG0) || (Trip_Flags1.all & TRIP_MASK_FLG1))? 1 : 0;
                                                        //alarm bit
    //Alarm -- b2 -- set to indicate an active or un-acknowledge alarm
    new_std.bit.b2 = ((Alarm_Flags0.all & ALARM_MASK_FLG0) || (Alarm_Flags1.all & ALARM_MASK_FLG1) || (Alarm_Flags2.all & ALARM_MASK_FLG2) )? 1 : 0;
                                                        

                                                        //Maintenance mode bit
    //M_Mode -- b4 -- set to indicate MM mode is active.
    //Setpoints0.MM_Enable is defined as:
    //      b08: MM state
    //      b07: MM local ON
    //      b06: MM local setting invalid
    //      b01: MM signal from secondary pin (not used in PXR35, combined with local switch)
    //      b00: MM remote com channel
    //bit08 == 1, MM mode enabled there; while bit08 == 0, MM mode disabled there.
    new_std.bit.b4 = ((Setpoints0.stp.MM_Enable & BIT8) == BIT8);

    //T_Mode -- b5 -- test mode -- set to indicate test mode is active
    new_std.bit.b5 = ((TU_State_TestMode == 1) || (TU_State_TestUSBMode == 1));

    //T_Forbid_Coil -- b6 -- set so trip unit can't execute Coil Detection testing if current levels are too high
    if ((CurOneCyc.Ia > CoilDetect.TestAllowedThreshold) || (CurOneCyc.Ib > CoilDetect.TestAllowedThreshold)
       || (CurOneCyc.Ic > CoilDetect.TestAllowedThreshold) ||(CurOneCyc.In > CoilDetect.TestAllowedThreshold))
    {        
      T_Forbid_Coil = 1;                          // current is too high to allow running a test
    }  
    else
    {
      T_Forbid_Coil = 0;
    }     
    new_std.bit.b6 = T_Forbid_Coil;
    
      
    //Rotation bit -- not implemented yet
                                                        
    //PickedUp -- b9 -- indicating pickup is active
    new_std.bit.b9 = ((TripPuFlags.all & TRIP_PU_MASK) || LdPuAlmFlg) ? 1 : 0;
                                                        
    //Zin -- b10 -- detection -- indicating ZIN input is active
    new_std.bit.b10 = Zin_Latched;
                                                        
    //AuxPower -- b11 -- detection -- indicating aux-power connection
    new_std.bit.b11 = (AuxPower_LowVolt == 1)? 0 : 1;

                                                        //source ground bit
    //SRC_Gnd -- b12 -- configuration -- indicating GF is source mode
    new_std.bit.b12 = (Setpoints1.stp.Gnd_Type == 0)? 0 : 1;
 
    
    //T_Forbid_SecInj -- b13 -- set so trip unit can't execute HW or FW Sec Inj testing if current levels are too high
    //                       -- HW and FW Test Thresholds are the same
    if ((CurOneCyc.Ia > FW_SimulatedTest.TestAllowedThreshold) || (CurOneCyc.Ib > FW_SimulatedTest.TestAllowedThreshold)
        || (CurOneCyc.Ic > FW_SimulatedTest.TestAllowedThreshold) ||(CurOneCyc.In > FW_SimulatedTest.TestAllowedThreshold))
    {    
      T_Forbid_SecInj = 1;           // current is too high to allow running a test
    }  
    else
    {
      T_Forbid_SecInj = 0;
    }   
    new_std.bit.b13 = T_Forbid_SecInj;


    //spring charged bit -- not implement yet

    //connected bit -- not implement yet

    pStd_TU_Status->all = new_std.all;

    //return TRUE for change, FALSE for no change
    return ((old_std.all != new_std.all)? TRUE : FALSE);
}
//------------------------------------------------------------------------------
//  END OF FUNCTION         Update_Std_Status()
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Update_PSC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Primary/Secondary/Cause of Status
//
//  MECHANICS:          This subroutine updates the PSC value.
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             *pPriSecCause - present PSC
//                      COT_AUX.AUX_OpenFlg - open/closed state of breaker (True=open), based on aux switch
// 
//  OUTPUTS:            *pPriSecCause - updated PSC
//                      The subroutine returns True if the PSC has changed; False otherwise
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

uint8_t Update_PSC(uint32_t *pPriSecCause)
{
  uint16_t pri, sec, cause;           //temps
  uint32_t old_PSC;

  old_PSC = *pPriSecCause;    //save old status

  // Primary Status//

  // check whether trip unit is OPENED  
  pri = (COT_AUX.AUX_OpenFlg == 1)? PSTATUS_OPEN : PSTATUS_CLOSED;
  // check whether trip unit is TRIPPED  
  if ((Trip_Flags0.all & TRIP_MASK_FLG0) || (Trip_Flags1.all & TRIP_MASK_FLG1) || (TestLCD == 1))
  {
    pri = PSTATUS_TRIPPED;
  }
  // check whether trip unit is ALARMED
  else if (((Alarm_Flags0.all & ALARM_MASK_FLG0)  != 0) || ((Alarm_Flags1.all & ALARM_MASK_FLG1) != 0) || ((Alarm_Flags2.all & ALARM_MASK_FLG2) != 0))
  {
    pri = PSTATUS_ALARMED;
  }
  // check wheather trip unit is PICKUP                            
  else if ( (TripPuFlags.all & TRIP_PU_MASK) || LdPuAlmFlg )         
  {
    pri = PSTATUS_PU;
  }
  
  //Secondary Status//

  if (pri == PSTATUS_TRIPPED)
  {
    //if aux trip flag "TestTrip & TestNoTrip (test from USB port)" or "TestLCD (test from LCD)" is set
    if (TestTrip  || TestNoTrip  || TestLCD )
    {
      sec = SSTATUS_TEST;           
    }
    else if (TU_State_PowerUp )   //trip caused by over current, and if PoweredUp is set...
    {
      sec = SSTATUS_POWERUP;          
    }
    else                          //trip caused by over current, and if PoweredUp is reset...
    {
      sec = SSTATUS_NA;             
    }
  }
  
  else if (pri == PSTATUS_ALARMED)
  {
                                       //for testing initiated alarm...
    if (TestTrip || TestNoTrip )
    {
      sec = SSTATUS_TEST;            
    }
    else                               //for actual alarm situation
    {
      sec = SSTATUS_ALARM;           
    }
  }                              
  else if (pri == PSTATUS_PU)
  {
    if (TU_State_PowerUp )       
    {
      sec = SSTATUS_POWERUP;         
    }
    else                               
    {
      sec = SSTATUS_NA;              
    }
  }  
  else  //if primary is "open" or "close"
  {
    if (TU_State_PowerUp )       //under breaker status, if PoweredUp is set...
    {
      sec = SSTATUS_POWERUP;         
    }
    else                         //under pick up status, if PoweredUp is reset...
    {
      sec = SSTATUS_NA;            
    }
  }

  //Cause of Status//
  
  //construct cause status
  cause = CAUSE_NORM;         //set default cause to "normal"

  //cause - primary status TRIP
  if (pri == PSTATUS_TRIPPED)
  {
    if (McrTripFlg )          
    {
      cause = CAUSE_MCR;
    }
    else if (HWInstTripFlg )     
    {
      cause = CAUSE_INSTOR;
    }
    else if (MM_TripFlg )      
    {
      cause = CAUSE_MM;
    }
    else if (DigBypassTripFlg)
    {
      cause = CAUSE_DBPASS;
    }
    else if (GndTripFlg )       
    {
      cause = (IecSel )? CAUSE_EF : CAUSE_GF;
    }                                 
    else if (NeuTripFlg && LdTripFlg ) 
    {
      cause = CAUSE_LDNEUT;
    }                               
    else if (NeuTripFlg  && SdTripFlg ) 
    {
      cause = CAUSE_SDNEUT;
    }                               
    else if (NeuTripFlg  && InstTripFlg ) 
    {
      cause = CAUSE_INSTNEUT;
    }
    else if (InstTripFlg )     
    {
      cause = CAUSE_INST;
    }
    else if (SdTripFlg )        
    {
      cause = CAUSE_SHORT;
    }
    else if (LdTripFlg )       
    {
      cause = CAUSE_LONG;
    }
    else if (TempTripFlg )      
    {
      cause = CAUSE_OT;
    }
    else if (UfTripFlg )      
    {
      cause = CAUSE_UF;
    }
    else if (OfTripFlg )      
    {
      cause = CAUSE_OF;
    }
    else if (SneakersTripFlg )  
    {
      cause = CAUSE_MECH;
    }
    else if (TestLCD )          
    {
      cause = CAUSE_LCDOPEN;      
    }
    else if(ComOpenFlg )        
    {
      cause = CAUSE_COMCONT;
    }
    else if(OvTripFlg )         
    {
      cause = CAUSE_OV;
    }
    else if(UvTripFlg )       
    {
      cause = CAUSE_UV;
    }
    else if(VoltUnbalTripFlg )   
    {
      cause = CAUSE_VUNB;
    }
    else if(CurrUnbTripFlg)    
    {
      cause = CAUSE_CUNB;
    }
    else if(RealPwrTripFlg )    
    {
      cause = CAUSE_REALP;           
    }
    else if(ReacPwrTripFlg )   
    {
      cause = CAUSE_REACTP;           
    }
    else if(AppPwrTripFlg )     
    {
      cause = CAUSE_APP;           
    }
    else if(PFTripFlg )       
    {
      cause = CAUSE_UPF;           
    }
    else if(RevPwrTripFlg )     
    {
      cause = CAUSE_REVP;
    }
    else if(RevReacPwrTripFlg ) 
    {
      cause = CAUSE_REVVARP;           
    }
    else if(RevSeqTripFlg)     
    {
      cause = CAUSE_REVS;           
    }
    else if(PhaseLossTripFlg)  
    {
      cause = CAUSE_PCL;
    }

  }
  //cause - primary status ALARM
  else if(pri == PSTATUS_ALARMED) 
  {
    if (BadTaFlg )              
    {
      cause = CAUSE_COIL;
    }
    else if (GndAlmFlg )        
    {                               
      cause = (IecSel )? CAUSE_EF : CAUSE_GF;
    }
    else if(TimeToTripFlg)
    {
      cause = CAUSE_TTT;
    }
    else if (GfPreAlarmFlg )      
    {
      cause = CAUSE_GF_PRE;
    }
    else if (HlAlm2Flg )          
    {
      cause = CAUSE_HLOAD_2;
    }
    else if (HlAlm1Flg )          
    {
      cause = CAUSE_HLOAD_1;
    }    
    else if (TherMemAlmFlg)      
    {
      cause = CAUSE_T_MEM;
    }
    else if (TempAlmFlg )         
    {
      cause = CAUSE_HIGH_TEMP;
    }
    else if (SneakersAlmFlg )     
    {
      cause = CAUSE_MECH;
    }
    else if (BrkHealthAlmFlg )    
    {                                 
      cause = CAUSE_BRK_HEALTH;
    }
    else if (RtcFault )           
    {
      cause = CAUSE_RTC;
    }
    else if (LowBatAlmFlg )       
    {
      cause = CAUSE_BATT;
    }
    else if (LowControlVoltage)  
    {
      cause = CAUSE_AUXUV;
    }
    else if(FrameVerFault )       
    {
      cause = CAUSE_FRAME;
    }                                     
    else if (ETUCalFault || RogoCalFault)           
    {
      cause = CAUSE_CAL;
    }
    else if (BrkrConfigFault)
    {
      cause = CAUSE_CONFIG;
    }  
    else if (StpFault )           
    {
      cause = CAUSE_STPE;
    }
    else if (WdgFault )           
    {
      cause = CAUSE_WTDG;
    }
    else if(OvAlmFlg )            
    {
      cause = CAUSE_OV;
    }
    else if(UvAlmFlg )            
    {
      cause = CAUSE_UV;
    }
    else if(VoltUnbalAlmFlg )     
    {
      cause = CAUSE_VUNB;
    }
    else if(CurrUnbAlmFlg )       
    {
      cause = CAUSE_CUNB;
    }
    else if(RevPwrAlmFlg )        
    {
      cause = CAUSE_REVP;
    }
    else if(PhaseLossAlmFlg)     
    {
      cause = CAUSE_PCL;
    }
    else if(WrongSensorAlmFlg )   
    {
      cause = CAUSE_WRONGSENSOR;
    }
    else if(PFAlmFlg )            
    {
      cause = CAUSE_UPF;
    }
    else if(RevSeqAlmFlg )            
    {
      cause = CAUSE_REVS;
    }
    else if(UfAlmFlg )            
    {
      cause = CAUSE_UF;
    }
    else if(OfAlmFlg )            
    {
      cause = CAUSE_OF;
    }
    else if(KWDmdAlmFlg )            
    {
      cause = CAUSE_REALPD; 
    }
    else if(KVADmdAlmFlg )            
    {
      cause = CAUSE_APPD; 
    }
    else if(NeutAlmFlg )            
    {
      cause = CAUSE_NEU; 
    }
    else if(THDAlmFlg )            
    {
      cause = CAUSE_THD; 
    }
    else if(MechAlmFlg_Mask & Flags0.all )            
    {
      cause = CAUSE_MECH; 
    }
    //else if(ElectAlmFlg_Mask & Flags0.all )   <--(individual causes are already handled here)         
    //{
    //  cause = CAUSE_ELF; 
    //}
    else if (NvmFault_Mask & Flags0.all )    // <-- these are non-protection like Min/Max, Energy, etc
    {
      cause = CAUSE_NVME;
    } 
    else if(RealPwrAlmFlg )            
    {
      cause = CAUSE_REALP; 
    }
    else if(ReacPwrAlmFlg )            
    {
      cause = CAUSE_REACTP; 
    }
    else if(AppPwrAlmFlg )            
    {
      cause = CAUSE_APP; 
    }
    else if(THDCurrAlmFlg )            
    {
      cause = CAUSE_CTHD; 
    }
    else if(THDVoltAlmFlg )            
    {
      cause = CAUSE_VTHD; 
    }
    else if(DispTempAlmFlg )            
    {
      cause = CAUSE_DTA; 
    }
    else if(RevReacPwrAlmFlg )            
    {
      cause = CAUSE_REVVARP; 
    }    
    else if (DigiBypassAlmFlg)
    {
      cause = CAUSE_DBPASS;
    }
  }
  //cause - primary status PICKED UP
  else if (pri == PSTATUS_PU)
  {
    if(LdPuAlmFlg)
    {
      cause = CAUSE_LONG;             
    }
    else if(UvTripPuFlg)
    {
      cause = CAUSE_UV;               
    }
    else if(OvTripPuFlg)
    {
      cause = CAUSE_OV;              
    }
    else if(VuTripPuFlg)
    {
      cause = CAUSE_VUNB;             
    }
    else if(CuTripPuFlg)
    {
      cause = CAUSE_CUNB;             
    }
    else if(RevWTripPuFlg)
    {
      cause = CAUSE_REVP;             
    }
    else if(RevVarTripPuFlg)
    {
      cause = CAUSE_REVVARP;          
    }
    else if(PlTripPuFlg)
    {
      cause = CAUSE_PCL;             
    }
    else if(OfTripPuFlg)
    {
      cause = CAUSE_OF;               
    }
    else if(UfTripPuFlg)
    {
      cause = CAUSE_UF;               
    }
    else if(OkWTripPuFlg)
    {
      cause = CAUSE_REALP;              
    }
    else if(OkVarTripPuFlg)
    {
      cause = CAUSE_REACTP;            
    }
    else if(OkVATripPuFlg)
    {
      cause = CAUSE_APP;            
    }
    else if(UPFTripPuFlg)
    {
      cause = CAUSE_UPF;              
    }
      
  }
  //cause - primary status OPEN CLOSED
  else if ((pri == PSTATUS_OPEN) || (pri == PSTATUS_CLOSED))
  {
      if ((TU_State_OpenedByComm == 1) || (TU_State_ClosedByComm == 1))
      {
        cause = CAUSE_COMCONT;
      }
      else if(ThermCapacityFlg)
      {
        cause = CAUSE_TCAP;
      }
  }

                                //update primary, secondary, & cause status
  *pPriSecCause = ( (((uint32_t)cause) << 16) | (uint32_t)(((sec << 8) & 0xFF00) | (pri & 0x00FF)) );

                               //return true for change, false for no change in status triple
  return ((old_PSC != *pPriSecCause)? TRUE : FALSE);
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Update_PSC()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ResetMinMax()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Function reseting Min/Max and ETU features
//
//  MECHANICS:          
//
//  CAVEATS:            None
//
//  INPUTS:             bufID
// 
//  OUTPUTS:            none
//
//  ALTERS:             None
// 
//  CALLS:              main()
// 
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ResetMinMax(void)
{
uint8_t i;

  for (i=0;i<RESET_MAX_VAL;i++)
  {
    if (ResetMinMaxFlags & (1<<i))
    {
      ResetMinMaxBufID(i);
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ResetMinMax()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ResetMinMaxBufID()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Function reseting Min/Max and ETU features
//
//  MECHANICS:          
//
//  CAVEATS:            None
//
//  INPUTS:             bufID
// 
//  OUTPUTS:            none
//
//  ALTERS:             None
// 
//  CALLS:              ResetMinMax()
// 
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void ResetMinMaxBufID(uint32_t bufID)
{
  struct INTERNAL_TIME presenttime;
  Get_InternalTime(&presenttime);
  uint8_t i;
  
  switch (bufID)
  {
    case RESET_MINMAX_ALL:           //Reset all min/max values
      ResetMinMaxFlags |= 0xFFFF;     // set the flag for all min/max flags
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_ALL);
      break;
  
    case RESET_MINMAX_CURR:           //Reset min/max currents
      CurOneCyc_max.Ia = RESETMAX;
      CurOneCyc_max.Ib = RESETMAX;
      CurOneCyc_max.Ic = RESETMAX;
      CurOneCyc_max.Ig = RESETMAX;
      CurOneCyc_max.In = RESETMAX;
      
      CurOneCyc_min.Ia = RESETMIN;
      CurOneCyc_min.Ib = RESETMIN;
      CurOneCyc_min.Ic = RESETMIN;
      CurOneCyc_min.Ig = RESETMIN;
      CurOneCyc_min.In = RESETMIN;

      CurOneCyc_max.IaTS = presenttime;
      CurOneCyc_max.IbTS = presenttime;
      CurOneCyc_max.IcTS = presenttime;
      CurOneCyc_max.InTS = presenttime;
      CurOneCyc_max.IgTS = presenttime;

      CurOneCyc_min.IaTS = presenttime;
      CurOneCyc_min.IbTS = presenttime;
      CurOneCyc_min.IcTS = presenttime;
      CurOneCyc_min.InTS = presenttime;
      CurOneCyc_min.IgTS = presenttime;
      
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_CURR);

      break;
      
    case RESET_MINMAX_LL_VOLT:           //Reset min/max LL voltages
      VolAFEOneCyc_max.Vab = RESETMAX;
      VolAFEOneCyc_max.Vbc = RESETMAX;
      VolAFEOneCyc_max.Vca = RESETMAX;
      
      VolAFEOneCyc_min.Vab = RESETMIN;
      VolAFEOneCyc_min.Vbc = RESETMIN;
      VolAFEOneCyc_min.Vca = RESETMIN;

      VolAFEOneCyc_max.VabTS = presenttime;
      VolAFEOneCyc_max.VbcTS = presenttime;
      VolAFEOneCyc_max.VcaTS = presenttime;
      
      VolAFEOneCyc_min.VabTS = presenttime;
      VolAFEOneCyc_min.VbcTS = presenttime;
      VolAFEOneCyc_min.VcaTS = presenttime;

      VolADCOneCyc_max.Vab = RESETMAX;
      VolADCOneCyc_max.Vbc = RESETMAX;
      VolADCOneCyc_max.Vca = RESETMAX;
      
      VolADCOneCyc_min.Vab = RESETMIN;
      VolADCOneCyc_min.Vbc = RESETMIN;
      VolADCOneCyc_min.Vca = RESETMIN;

      VolADCOneCyc_max.VabTS = presenttime;
      VolADCOneCyc_max.VbcTS = presenttime;
      VolADCOneCyc_max.VcaTS = presenttime;
      
      VolADCOneCyc_min.VabTS = presenttime;
      VolADCOneCyc_min.VbcTS = presenttime;
      VolADCOneCyc_min.VcaTS = presenttime;
      
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_LL_VOLT);

      break;
  
    case RESET_MINMAX_LN_VOLT:           //Reset min/max LN voltages
      VolAFEOneCyc_max.Van = RESETMAX;
      VolAFEOneCyc_max.Vbn = RESETMAX;
      VolAFEOneCyc_max.Vcn = RESETMAX;
      
      VolAFEOneCyc_min.Van = RESETMIN;
      VolAFEOneCyc_min.Vbn = RESETMIN;
      VolAFEOneCyc_min.Vcn = RESETMIN;

      VolAFEOneCyc_max.VanTS = presenttime;
      VolAFEOneCyc_max.VbnTS = presenttime;
      VolAFEOneCyc_max.VcnTS = presenttime;
      
      VolAFEOneCyc_min.VanTS = presenttime;
      VolAFEOneCyc_min.VbnTS = presenttime;
      VolAFEOneCyc_min.VcnTS = presenttime;

      VolADCOneCyc_max.Van = RESETMAX;
      VolADCOneCyc_max.Vbn = RESETMAX;
      VolADCOneCyc_max.Vcn = RESETMAX;
      
      VolADCOneCyc_min.Van = RESETMIN;
      VolADCOneCyc_min.Vbn = RESETMIN;
      VolADCOneCyc_min.Vcn = RESETMIN;

      VolADCOneCyc_max.VanTS = presenttime;
      VolADCOneCyc_max.VbnTS = presenttime;
      VolADCOneCyc_max.VcnTS = presenttime;
      
      VolADCOneCyc_min.VanTS = presenttime;
      VolADCOneCyc_min.VbnTS = presenttime;
      VolADCOneCyc_min.VcnTS = presenttime;

      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_LN_VOLT);
      break;

    case RESET_MINMAX_PF:           //Reset min/max PF
      for (i=0; i<4; ++i)
      {
        PF.MinApp[i] = RESETMIN;
        PF.MaxApp[i] = RESETMAX;
        PF.MinDisp[i] = RESETMIN;
        PF.MaxDisp[i] = RESETMAX;
        PF.MinApp_TS[i] = presenttime;
        PF.MaxApp_TS[i] = presenttime;
        PF.MinDisp_TS[i] = presenttime;
        PF.MaxDisp_TS[i] = presenttime;
      }

      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_PF);
      break;
  
    case RESET_MINMAX_FREQ:           //Reset min/max frequency
      FreqLoad.MaxFreqVal = RESETMAX;
      FreqLoad.MinFreqVal = RESETMIN;
      FreqLoad.MaxFreqVal_TS = presenttime;
      FreqLoad.MinFreqVal_TS = presenttime;
    
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_FREQ);
      break;

    case RESET_MINMAX_PWR:           //Reset min/max powers
      Pwr200msecMax.Pa = RESETMAX;
      Pwr200msecMax.Pb = RESETMAX;
      Pwr200msecMax.Pc = RESETMAX;
      Pwr200msecMax.Ptot = RESETMAX;
      Pwr200msecMin.Pa = RESETMIN;
      Pwr200msecMin.Pb = RESETMIN;
      Pwr200msecMin.Pc = RESETMIN;
      Pwr200msecMin.Ptot = RESETMIN;

      Pwr200msecMax.Pa = RESETMAX;
      Pwr200msecMax.Pb = RESETMAX;
      Pwr200msecMax.Pc = RESETMAX;
      Pwr200msecMax.Ptot = RESETMAX;
      Pwr200msecMin.Pa = RESETMIN;
      Pwr200msecMin.Pb = RESETMIN;
      Pwr200msecMin.Pc = RESETMIN;
      Pwr200msecMin.Ptot = RESETMIN;

      Pwr200msecMax.PaTS = presenttime;
      Pwr200msecMax.PbTS = presenttime;
      Pwr200msecMax.PcTS = presenttime;
      Pwr200msecMax.PtotTS = presenttime;
      Pwr200msecMin.PaTS = presenttime;
      Pwr200msecMin.PbTS = presenttime;
      Pwr200msecMin.PcTS = presenttime;
      Pwr200msecMin.PtotTS = presenttime;

      Pwr200msecAppMax.AppPa = RESETMAX;
      Pwr200msecAppMax.AppPb = RESETMAX;
      Pwr200msecAppMax.AppPc = RESETMAX;
      Pwr200msecAppMax.Apptot = RESETMAX;
      Pwr200msecAppMin.AppPa = RESETMIN;
      Pwr200msecAppMin.AppPb = RESETMIN;
      Pwr200msecAppMin.AppPc = RESETMIN;
      Pwr200msecAppMin.Apptot = RESETMIN;

      Pwr200msecAppMax.AppPaTS = presenttime;
      Pwr200msecAppMax.AppPbTS = presenttime;
      Pwr200msecAppMax.AppPcTS = presenttime;
      Pwr200msecAppMax.ApptotTS = presenttime;
      Pwr200msecAppMin.AppPaTS = presenttime;
      Pwr200msecAppMin.AppPbTS = presenttime;
      Pwr200msecAppMin.AppPcTS = presenttime;
      Pwr200msecAppMin.ApptotTS = presenttime;
      
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_MINMAX_PWR);

      break;
      
    case RESET_PEAK_DMNCURR:           //Reset peak demand current
    
      IDmnd.IaMin = RESETMIN;
      IDmnd.IaMax = RESETMAX;
      IDmnd.IbMin = RESETMIN;
      IDmnd.IbMax = RESETMAX;
      IDmnd.IcMin = RESETMIN;
      IDmnd.IcMax = RESETMAX;
      IDmnd.InMin = RESETMIN;
      IDmnd.InMax = RESETMAX;

      IDmnd.IaMaxTS = presenttime;
      IDmnd.IbMaxTS = presenttime;
      IDmnd.IcMaxTS = presenttime;
      IDmnd.InMaxTS = presenttime;
      IDmnd.IaMinTS = presenttime;
      IDmnd.IbMinTS = presenttime;
      IDmnd.IcMinTS = presenttime;
      IDmnd.InMinTS = presenttime;

      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_PEAK_DMNCURR);
      break;

    case RESET_PEAK_DMNPWR:           //Reset peak power demand

      PDmnd.TotWMax = RESETMAX;
      PDmnd.TotVarMax = RESETMAX;
      PDmnd.TotVAMax = RESETMAX;
      PDmnd.TotWMaxTS = presenttime;
      PDmnd.TotVarMaxTS = presenttime;
      PDmnd.TotVAMaxTS = presenttime;

      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_PEAK_DMNPWR);
      break;
  
    case RESET_ALLDMNDW:           //Reset all demand windows
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ALLDMNDW);
      break;

    case RESET_ALLSTATE_FLGS:          //Reset all change of state flags
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ALLSTATE_FLGS);
      break;
  
    case RESET_PWRUP_FLGS:          //Reset powered-up flag
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_PWRUP_FLGS);
      break;

    case RESET_ACC_ENG:          //Reset accumulated energy
      ResetEnergy();
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ACC_ENG);
      break;
  
    case RESET_HEALTHSUMM:          //Reset health summary
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_HEALTHSUMM);
      break;

    case RESET_TRIP_CNTR:          //Reset Trip counters
        ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_TRIP_CNTR);
      break;
  
    case RESET_OOPTS:          //Reset Oopts
        ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_OOPTS);
      break;

    case RESET_TEMP:          //Reset Temp
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_TEMP);
      break;
  
    case RESET_RUNTIME:          //Reset Runtime
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_RUNTIME);
      break;

    case RESET_ALL_EXT_DIAGS:          //Reset all external diagnostics
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ALL_EXT_DIAGS);
      break;
  
    case RESET_ALL_INT_DIAGS:          //Reset all internal diagnostics
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ALL_INT_DIAGS);
      break;

    case RESET_ETU:          //Reset Trip Unit
      ResetMinMaxFlags &= ~(((uint32_t)1) << RESET_ETU);
      break;

  };

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ResetMinMaxBufID()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CaptureUserWaveform()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Perform User Waveform Capture
//
//  MECHANICS:          This function performs a user waveform capture - either
//                          1 cycle of Ia - Igsrc, VanAFE - VcnAFE, VanADC - VcnADC   or
//                          12 cycles of one of Ia - Igsrc, VanAFE - VcnAFE, VanADC - VcnADC
//                      Basically, a User-Initiated waveform capture copies samples from the primary
//                      (39-cycle) sample buffer, SampleBuf[], into a dedicated buffer,
//                      struct UserSamples[].  The most recent cycle or cycles (at the time of the request)
//                      are used for the waveform capture.  That is, the present sample is the LAST sample
//                      captured.
//                      A waveform capture may be requested by a number of sources (test port, modbus,
//                      USB, display, etc.).  Once a waveform capture has been made, it is frozen until it
//                      is freed by the requestor (or a timeout occurs).  This prevents UserSamples.xx from
//                      being overwritten while in use.
//
//                      Presently, the timestamp is the time the subroutine begins - it is not tied exactly
//                      to the time of the last sample, and therefore, is not extremely accurate
//                      *** DAH - this must be checked to see whether it is ok
//                          
//                      User-Initiated waveform capture freeze/lock control is performed by a set of flags
//                      and variables:
//                      Name            Description
//                      UserWF.Locked   Flag indicating the waveform is locked and no further capture may be
//                                      Set after the waveform capture has been completed.  Cleared either
//                                      by the calling function, an execute action command, or by a timeout
//                      UserWF.Type     Type of capture that was done.  Set when the waveform capture has
//                                      been completed.  Set to invalid value when waveform is unlocked.
//                      UserWF.CapTime  Time of the capture.  Set when the capture is made
//
//                      The process is summarized below:
//
//                        Initialization:
//                           All flags and codes set to inactive values
//                         
//                          Capture subroutine called, capture performed
//                              UserWF.Locked = TRUE
//                              UserWF.Type = capture type
//                              UserWF.CapTime = present time
//                         
//                          If another capture is attempted, it will be rejected and the comms port NAK'ed, until the timeout
//                            time is exceeded
//                         
//                          Comms port that initiated the capture reads the waveform
//                         
//                          Comms port that initiated the capture unlocks the waveform
//
//  CAVEATS:            None
//                      
//  INPUTS:             type - type of waveform capture required
//                      source - the port that requested the wavweform capture
//                      UserWF.xx, SampleBuf[].xx
// 
//  OUTPUTS:            The function returns True if the capture was done, False otherwise
//                      UserSamples.xx[]
//
//  ALTERS:             UserWF.Locked, UserWF.LockTime
// 
//  CALLS:              Get_InternalTime(), __disable_irq(), __enable_irq()
// 
//  EXECUTION TIME:                           
//
//------------------------------------------------------------------------------------------------------------

uint8_t CaptureUserWaveform(uint16_t type)
{
  uint16_t BufferIndex, LastSample, PosBuf, NegBuf;
  uint16_t UsrWF_SampleCtr, UserWFind;
  struct INTERNAL_TIME temptime1;

  // Read the present time. We will need this later anyway
  __disable_irq();
  Get_InternalTime(&temptime1);     // need to figure out what to do with this Timestamp if ever needed
  __enable_irq();

  // If user waveform is locked, some other process is still working with a captured waveform (perhaps
  //   transmitting it out).  We cannot do a capture, because it will corrupt the existing capture.  Check
  //   for a timeout.  If timeout time has elapsed, do the capture anyway
  if (UserWF.Locked)
  {
    // If timeout time has not elapsed, return False since unable to do the capture
    if ( ((temptime1.Time_secs - UserWF.CapTime.Time_secs) < USERWF_TIMEOUT)
    ||   ( ((temptime1.Time_secs - UserWF.CapTime.Time_secs) == USERWF_TIMEOUT)
        && (temptime1.Time_nsec  < UserWF.CapTime.Time_nsec) )  )
    {
      return (FALSE);
    }
  }

  // Ok to do a capture

  UserWF.CapTime.Time_secs = temptime1.Time_secs;
  UserWF.CapTime.Time_nsec = temptime1.Time_nsec;
  LastSample = SampleIndex;                 // Freeze index of last sample in SampleBuf[]
                  
  UsrWF_SampleCtr = ((type == USR_TYPE_ALL) ? 80 : 960);

  UserWFind = 0;                            // Initialize the user waveform index

  do
  {
    // setting proper BufferIndex in circular buffer 
    if (LastSample > UsrWF_SampleCtr)          // buffer contains proper amount of samples in a row
    {
      BufferIndex = LastSample - UsrWF_SampleCtr + UserWFind;
    }
    else                                      // samples crossing 0 index
    {
      PosBuf = UsrWF_SampleCtr - LastSample;
      NegBuf = TOTAL_SAMPLE_SETS - PosBuf;
      BufferIndex = (UserWFind<PosBuf?NegBuf+UserWFind:UserWFind-PosBuf);       //zero index need to be checked
    }

    // read out from SampleBuff based on User Request
    switch (type)
    {
      case USR_TYPE_ALL:
        UserSamples.OneCyc[0][UserWFind] = SampleBuf[BufferIndex].Ia;
        UserSamples.OneCyc[0][UserWFind] = SampleBuf[BufferIndex].Ia;
        UserSamples.OneCyc[1][UserWFind] = SampleBuf[BufferIndex].Ib;
        UserSamples.OneCyc[2][UserWFind] = SampleBuf[BufferIndex].Ic;
        UserSamples.OneCyc[3][UserWFind] = SampleBuf[BufferIndex].In;
        UserSamples.OneCyc[4][UserWFind] = SampleBuf[BufferIndex].Igsrc;
        UserSamples.OneCyc[5][UserWFind] = SampleBuf[BufferIndex].VanAFE;
        UserSamples.OneCyc[6][UserWFind] = SampleBuf[BufferIndex].VbnAFE;
        UserSamples.OneCyc[7][UserWFind] = SampleBuf[BufferIndex].VcnAFE;
        UserSamples.OneCyc[8][UserWFind] = SampleBuf[BufferIndex].VanADC;
        UserSamples.OneCyc[9][UserWFind] = SampleBuf[BufferIndex].VbnADC;
        UserSamples.OneCyc[10][UserWFind] = SampleBuf[BufferIndex].VcnADC;
        break;
      case USR_TYPE_IA:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].Ia;
        break;
      case USR_TYPE_IB:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].Ib;
        break;
      case USR_TYPE_IC:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].Ic;
        break;
      case USR_TYPE_IN:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].In;
        break;
      case USR_TYPE_IGS:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].Igsrc;
        break;
      case USR_TYPE_VANAFE:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VanAFE;
        break;
      case USR_TYPE_VBNAFE:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VbnAFE;
        break;
      case USR_TYPE_VCNAFE:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VcnAFE;
        break;
      case USR_TYPE_VANADC:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VanADC;
        break;
      case USR_TYPE_VBNADC:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VbnADC;
        break;
      case USR_TYPE_VCNADC:
        UserSamples.TwelveCyc[UserWFind] = SampleBuf[BufferIndex].VcnADC;
        break;
    }
    ++UserWFind;
  } while(UserWFind < UsrWF_SampleCtr);

  UserWF.Locked = TRUE;
  UserWF.Type = type;

  return (TRUE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         CaptureUserWaveform()
//------------------------------------------------------------------------------------------------------------
