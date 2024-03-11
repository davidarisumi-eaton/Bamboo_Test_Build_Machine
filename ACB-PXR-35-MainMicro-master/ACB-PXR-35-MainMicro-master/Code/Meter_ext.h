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
//  MODULE NAME:        Meter_ext.h
//
//  MECHANICS:          This is the declarations file for the Meter.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150617  DAH File Creation
//   0.01   150818  DAH Code development
//   0.02   150919  DAH Code development
//   0.03   150918  DAH Code development
//   0.04   150928  DAH - Continued code development.  Added sample and capture variables, sums of squares
//                        (SOS) structures, currents metering structures, sine and cosine coefficient
//                        tables, Meter_VarInit(), AFE_Process_Samples(), Calc_Current(), and
//                        Calc_Current1()
//   0.05   151021  DAH - Added struct AFEcal and StoreAFEcal()
//   0.06   151110  DAH - Deleted StoreAFEcal() (moved to Iod.c)
//                  BP  - Added code for ADC and temperature calculation
//   0.07   151216  DAH - Renamed .sram1 to .sram2 for readability
//   0.08   160122  DAH - Structure CURRENTS_WITH_G renamed to CUR_WITH_G_F
//                      - Structure CURRENTS_WITHOUT_G renamed to CUR_WITHOUT_G_F
//   0.09   160224  DAH - Added UseADCvals
//   0.10   160310  DAH - Added TestWFind to support waveform captures when in SD pickup (for test purposes)
//                      - Added MTR_new_samples[].  These are the filtered samples for metered values
//                      - Structure CurOneSecSOS_Save renamed to Cur200msecSOS_Sav
//                      - Structure VolAFEOneSecLLSOS_Sav renamed to VolAFE200msecLLSOS_Sav
//                      - Structure VolAFEOneSecLNSOS_Sav renamed to VolAFE200msecLNSOS_Sav
//                      - Structure PwrOneSecSOS_Sav renamed to Pwr200msecSOS_Sav
//                      - Structure CurOneSec renamed to Cur200msec
//                      - Added Cur200msec_min and Cur200msec_max
//                      - Added struct ADCcalHigh and ADCcalLow
//   0.12   160502  DAH - Added structures VolAFEOneCycLN, VolAFEOneCycLL, VolAFE200msecLN, and
//                        VolAFE200msecLLCalc_Voltage() to support metered AFE voltages
//                      - Added Calc_Voltage() and Calc_Voltage1() to support metered AFE voltages
//                      - Added structures VolAFEOneCycLNmax, VolAFEOneCycLNmin, VolAFEOneCycLLmax,
//                        VolAFEOneCycLLmin, VolAFE200msecLNmax, VolAFE200msecLNmin, VolAFE200msecLLmax,
//                        VolAFE200msecLLmin to support metered AFE min and max voltages
//                      - Added Calc_Power() and Calc_Power1()
//                      - Added structures PwrOneCyc, Pwr200msec, PwrOneCycApp and Pwr200msecApp
//          160502   BP - Removed Calc_Temperature().
//                      - Added Read_ThermMem().
//   0.13   160512  DAH - Added structures ResidualXxx and EnergyXxx
//                      - Added Calc_Energy() and ResetEnergy()
//                      - Added structures PwrOneCycMin, PwrOneCycMax, Pwr200msecMin, Pwr200msecMax,
//                        PwrOneCycAppMin, PwrOneCycAppMax, Pwr200msecAppMin, Pwr200msecAppMax
//   0.14   160620  DAH - Added support for Test Injection
//                          - INT_a1[] added
//   0.21   180301  DAH - Added support for harmonics computations
//                          - Added Calc_Harmonics() subroutine
//                      - Added support for ADC (line-side) voltages
//                          - Added numerous "VolADCxxxx" variables
//                          - Added Calc_Voltage2() and Calc_Voltage3()
//                      - Deleted Sin1_CoefQ15[] and Cos1_CoefQ15[] as they are no longer used
//   0.22   180420  DAH - Added support for demand computations
//                          - Added Calc_I_Demand() and Dmnd_I_VarInit()
//                          - struct EnergyAll was deleted and struct EngyDmnd was added
//                          - Added struct Dmnd_SaveEvent 
//   0.23   180504  DAH - Deleted Dmnd_I_VarInit(), Calc_I_Demand(), Dmnd_SaveEvent, and struct EngyDmnd[]
//                        (moved to Demand_ext.h)
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Added ManageSPI2Flags()
//   0.27   181001  DAH - Added HarmReq and HarmAck
//   0.30   190314  DAH - Added  TP_CoilTempRMSavg() for coil temperature measurement
//   0.31   190506  DAH - Deleted HarmAck as it is no longer used
//   0.32   190726  DAH - Renamed Calc_Power() to Calc_Meter_Power()
//                      - Renamed Calc_Power1() to Calc_Prot_Power()
//                      - struct Cur200msecSOS_Sav renamed to Cur200msFltrSOS_Sav
//                      - Added struct Cur200msNoFltrSOS_Sav 
//                      - struct CurOneCycSinSOS_Sav replaced with CurVol200msNoFltrSinSOS_Sav and
//                        struct CurOneCycCosSOS_Sav replaced with CurVol200msNoFltrCosSOS_Sav
//                      - struct VolAFEOneCycLNSOS_Sav and VolAFEOneCycLLSOS_Sav combined into
//                        VolAFEOneCycSOS_Sav
//                      - struct VolAFE200msecLNSOS_Sav and VolAFE200msecLLSOS_Sav combined into
//                        VolAFE200msFltrSOS_Sav
//                      - struct VolADCOneCycLNSOS_Sav and VolADCOneCycLLSOS_Sav combined into
//                        VolADCOneCycSOS_Sav
//                      - struct VolADC200msecLNSOS_Sav and VolADC200msecLLSOS_Sav combined into
//                        VolADC200msSOS_Sav
//                      - Added struct VolAFE200msNoFltrSOS_Sav
//                      - struct VolAFEOneCycLN and VolAFEOneCycLL combined into VolAFEOneCyc
//                      - struct VolAFE200msecLN and VolAFE200msecLL combined into VolAFE200msFltr
//                      - struct VolADCOneCycLN and VolADCOneCycLL combined into VolADCOneCyc
//                      - struct VolADC200msecLN and VolADC200msecLL combined into VolADC200ms
//                      - Cur200msec renamed to Cur200msFltr, Cur200msec_max to Cur200msFltr_max, and
//                        Cur200msec_min to Cur200msFltr_min
//                      - struct VolAFEOneCycLNmax and VolAFEOneCycLLmax combined into VolAFEOneCyc_max
//                      - struct VolAFEOneCycLNmin and VolAFEOneCycLLmin combined into VolAFEOneCyc_min
//                      - struct VolAFE200msecLNmax and VolAFE200msecLLmax combined into VolAFE200msFltr_max
//                      - struct VolAFE200msecLNmin and VolAFE200msecLLmin combined into VolAFE200msFltr_min
//                      - struct VolADCOneCycLNmax and VolADCOneCycLLmax combined into VolADCOneCyc_max
//                      - struct VolADCOneCycLNmin and VolADCOneCycLLmin combined into VolADCOneCyc_min
//                      - struct VolADC200msecLNmax and VolADC200msecLLmax combined into VolADC200ms_max
//                      - struct VolADC200msecLNmin and VolADC200msecLLmin combined into VolADC200ms_min
//   0.33   190823  DAH - Renamed Calc_Current() to Calc_Meter_Current()
//                      - Renamed Calc_Current1() to Calc_Prot_Current()
//                      - Renamed Calc_Voltage() to Calc_Meter_AFE_Voltage()
//                      - Renamed Calc_Voltage2() to Calc_Meter_ADC_Voltage()
//                      - Renamed Calc_Voltage1() to Calc_Prot_AFE_Voltage()
//                      - Cur200msFltr_max and Cur200msFltr_min definitions changed from
//                        struct CUR_WITH_G_F to struct MIN_MAX_CUR_WITH_G_F
//                      - VolAFE200msFltr_max and VolAFE200msFltr_min definitions changed from
//                        struct VOLTAGES to struct MIN_MAX_VOLTAGES
//                      - VolADC200ms_max and VolADC200ms_min definitions changed from struct VOLTAGES to
//                        struct MIN_MAX_VOLTAGES
//                      - Pwr200msecMin and Pwr200msecMax changed from struct POWERS to
//                        struct MIN_MAX_POWERS
//                      - Deleted struct CurOneCycPeak (moved to Intr.c)
//                      - Added Crest Factor support
//                          - Added struct CF_PeakSav
//                          - Added Calc_CF()
//                      - Added K-Factor support
//                          - Added Calc_KFactor()
//                      - Added Sequence Components and Phase Angles support
//                          - Added Calc_SeqComp_PhAng()
//                      - Added Frequency measurement support
//                          - Added Calc_Freq() and struct FreqLoad, FreqLine
//   0.36   200210  DAH - Added Cur200msFltrUnbal, VolAFE200msFltrUnbal to support display processor RTD
//                        buffer communications
//   0.50   220203  DAH - Added Cur200msIavg, VolAFE200msFltrVllavg, VolAFE200msFltrVlnavg,
//                        VolADC200msVllavg, VolADC200msVlnavg, struct PF_VALUES PF, Cur200msFltrIgmin,
//                        Cur200msFltrIgmax, THD[], KF_Val, CF
//   0.51   220304  DAH - Added PhAngles1[], PhAngles2[]
//                      - Corrected Pwr200msecAppMin, Pwr200msecAppMax declarations
//                      - Min/max current values changed from 200msec values to one-cycle values
//                          - Replaced Cur200msFltr_min.Ix and Cur200msFltr_max.Ix with CurOneCyc_min.Ix and
//                            CurOneCyc_max.Ix.  ResetMinMaxValues() revised 
//                          - Replaced Cur200msFltrIgmin and Cur200msFltrIgmax with CurOneCycIgmin and
//                            CurOneCycIgmax
//                          - Replaced Cur200msFltrIgminTS and Cur200msFltrIgmaxTS with CurOneCycIgminTS and
//                            CurOneCycIgmaxTS
//                          - Replaced CurOneCyc_min.Igsrc and Igres with CurOneCyc_min.Ig
//                          - Replaced CurOneCyc_max.Igsrc and Igres with CurOneCyc_max.Ig
//                      - Current unbalance value changed from 200msec value to one-cycle value
//                          - Replaced Cur200msFltrUnbal with CurOneCycUnbal
//                      - Min/max voltage values changed from 200msec values to one-cycle values
//                          - Deleted struct MIN_MAX_VOLTAGES VolAFE200msFltr_min and VolAFE200msFltr_max
//                          - VolAFEOneCyc_min and VolAFEOneCyc_max changed from struct VOLTAGES to
//                            struct MIN_MAX_VOLTAGES
//                          - Deleted struct MIN_MAX_VOLTAGES VolADC200ms_max and VolADC200ms_min
//                          - VolADCOneCyc_min and VolADCOneCyc_max changed from struct VOLTAGES to
//                            struct MIN_MAX_VOLTAGES
//                      - Voltage unbalance value changed from 200msec value to one-cycle value
//                          - Replaced VolAFE200msFltrUnbal with VolAFEOneCycUnbal
//                      - Calc_Meter_ADC_Voltage renamed Calc_ADC_200ms_Voltage()
//                      - Calc_Voltage3() renamed to Calc_ADC_OneCyc_Voltage()
//                      - Added per-phase current and voltage values
//                          - Added CurUnbal, CurUnbalPh_max, CurUnbalMax, CurUnbalAllMax, CurUnbalAllMaxTS
//                          - Added VolUnbal, VolUnbalMaxLN, VolUnbalMaxLL, VolUnbalPh_max,
//                            VolUnbalAllMaxLN, VolUnbalAllMaxLL, VolUnbalAllMaxLNTS, VolUnbalAllMaxLLTS
//   0.52   220309  DAH - Added TotWHr, NetWHr, TotVarHr, and NetVarHr
//   0.53   220325  DAH - Added THDminmax, HarmonicsAgg, HarmonicsCap
//   0.54   220330  DAH - CurOneCycUnbal renamed to CurUnbalTot
//                      - VolAFEOneCycUnbal renamed to VolUnbalTot
//                      - Added CurUnbalTotMax, CurUnbalTotMaxTS, VolUnbalTotMax, and VolUnbalTotMaxTS
//   0.55   220420  DAH - Added CurOneCycIavg to support processor-processor communications
//                      - Added StatusCode for Modbus and proc-proc comms testing
//   0.56   220526  DAH - Added HarmFrozen
//   0.57   220628  DAH - Added CurOneCycIg to support poc-proc communications
//   0.61   221001  DB  - Added SnapshotMeteredValues and SnapshotMeter()
//   0.67   221209  DAH - ManageSPI2Flags() renamed to ManageSPI1Flags()
//   0.69   220214  DB  - Added ExtCapSnapshotMeteredValues and ExtCapSnapshotMeter() declarations
//   0.70   230224  BP  - Added global values, Vll_max and Vll_min, to be used in Over/Under Voltage
//                        protection
//                      - Added TA_Volt_Monitoring() and Calc_BatteryVolt()
//   0.72   230320  DAH - Deleted ExtCapSnapshotMeteredValues declaration (moved to Events_ext.h)
//    25    230403  DAH - Added preliminary support for PriSecCause status
//                          - Update_PSC() added
//   30     230412  DAH - Replaced ExtCapSnapshotMeter() with ExtCapSnapshotMeterOneCyc() and
//                        ExtCapSnapshotMeterTwoHundred(), as one-cycle values are captured for the 6sec
//                        readings and 200msec values are captured for the 60sec readings
//   40     230531  DAH - Revised SnapshotMeter() declaration (added SummaryCode)
//   94     231010  DAH - Added struct USER_WF_CAPTURE UserWF, UserSamples, and CaptureUserWaveform()
//   117    231129  DAH - Deleted Read_ThermMem()
//   148    240131  BP  - Added AuxPower_Monitoring()
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
//                    Global Variable Declarations
//------------------------------------------------------------------------------------------------------------
//
extern struct CUR_WITH_G_F              CurOneCycSOS_Sav;
extern struct CUR_WITH_G_F              Cur200msFltrSOS_Sav;
extern struct CUR_WITHOUT_G_F           Cur200msNoFltrSOS_Sav;
extern float                            CurVol200msNoFltrSinSOS_Sav[7];
extern float                            CurVol200msNoFltrCosSOS_Sav[7];
extern struct CUR_WITH_G_F              CurOneCyc;
extern struct CUR_WITH_G_F              Cur200msFltr;
extern struct MIN_MAX_CUR_WITH_G_F      CurOneCyc_max;
extern struct MIN_MAX_CUR_WITH_G_F      CurOneCyc_min;
extern struct CUR_WITHOUT_G_F           CF_PeakSav;
extern struct CUR_WITHOUT_G_F           CurUnbal;
extern struct MIN_MAX_CUR_WITHOUT_G_F   CurUnbalPh_max;
extern float                            CurUnbalMax;
extern float                            CurUnbalAllMax;
extern struct INTERNAL_TIME             CurUnbalAllMaxTS;
extern float                            CurUnbalTot;
extern float                            CurUnbalTotMax;
extern struct INTERNAL_TIME             CurUnbalTotMaxTS;

extern struct VOLTAGES              VolADCOneCycSOS_Sav;
extern struct VOLTAGES              VolADC200msSOS_Sav;
extern struct VOLTAGES              VolAFEOneCycSOS_Sav;
extern struct VOLTAGES              VolAFE200msFltrSOS_Sav;
extern struct VOLTAGES              VolAFE200msNoFltrSOS_Sav;
extern struct VOLTAGES              VolADCOneCyc;
extern struct MIN_MAX_VOLTAGES      VolADCOneCyc_max;
extern struct MIN_MAX_VOLTAGES      VolADCOneCyc_min;
extern struct VOLTAGES              VolADC200ms;
extern struct VOLTAGES              VolAFEOneCyc;
extern struct MIN_MAX_VOLTAGES      VolAFEOneCyc_max;
extern struct MIN_MAX_VOLTAGES      VolAFEOneCyc_min;
extern struct VOLTAGES              VolAFE200msFltr;
extern struct VOLTAGES              VolUnbal;
extern float                        VolUnbalMaxLN, VolUnbalMaxLL;
extern struct MIN_MAX_VOLTAGES      VolUnbalPh_max;
extern float                        VolUnbalAllMaxLN, VolUnbalAllMaxLL;
extern struct INTERNAL_TIME         VolUnbalAllMaxLNTS, VolUnbalAllMaxLLTS;
extern float                        VolUnbalTot;
extern float                        VolUnbalTotMax;
extern struct INTERNAL_TIME         VolUnbalTotMaxTS;

extern struct POWERS                PwrOneCycSOS_Sav;
extern struct POWERS                Pwr200msecSOS_Sav;
extern struct POWERS                PwrOneCyc;
extern struct POWERS                PwrOneCycMin;
extern struct POWERS                PwrOneCycMax;
extern struct POWERS                Pwr200msec;
extern struct MIN_MAX_POWERS        Pwr200msecMin;
extern struct MIN_MAX_POWERS        Pwr200msecMax;
extern struct APP_POWERS            PwrOneCycApp;
extern struct APP_POWERS            PwrOneCycAppMin;
extern struct APP_POWERS            PwrOneCycAppMax;
extern struct APP_POWERS            Pwr200msecApp;
extern struct MIN_MAX_APP_POWERS    Pwr200msecAppMin;
extern struct MIN_MAX_APP_POWERS    Pwr200msecAppMax;

extern struct ENERGY_FLOATS         ResidualPha;
extern struct ENERGY_UINT64         EnergyPha;
extern struct ENERGY_FLOATS         ResidualPhb;
extern struct ENERGY_UINT64         EnergyPhb;
extern struct ENERGY_FLOATS         ResidualPhc;
extern struct ENERGY_UINT64         EnergyPhc;
extern struct ENERGY_FLOATS         ResidualAll;
extern signed long long             TotWHr;
extern signed long long             NetWHr;
extern signed long long             TotVarHr;
extern signed long long             NetVarHr;

extern uint16_t AFE_single_capture[16];
extern float AFE_new_samples[8];
extern float MTR_new_samples[8];
extern uint32_t ADC_single_capture[5];
extern uint32_t ADC_test_samples[5];            // DEBUG - BP
extern float ADC_samples[10];

extern uint8_t HarmReq, HarmFrozen;
extern struct HARMONICS_I_STRUCT HarmonicsAgg, HarmonicsCap;

extern struct AFE_CAL AFEcal @".sram2";
extern struct ADC_CAL ADCcalHigh;
extern struct ADC_CAL ADCcalLow;

extern uint8_t UseADCvals;
extern float INT_a1[4];

extern float Cur200msFltrIg;
extern float Cur200msIavg;
extern float CurOneCycIg;
extern float CurOneCycIavg;
extern float VolAFE200msFltrVllavg;
extern float VolAFE200msFltrVlnavg;
extern float VolADC200msVllavg;
extern float VolADC200msVlnavg;

extern struct PF_VALUES PF;
extern float THD[10];
extern struct THD_MIN_MAX THDminmax[10];
extern struct K_FACTORS KF_Val;
extern struct CUR_WITHOUT_G_F CF;
extern float PhAngles1[9];
extern float PhAngles2[6];
extern struct SEQ_COMP SeqComp;

extern struct FREQ_MEASURE_VARS FreqLoad, FreqLine;

extern uint32_t StatusCode;
extern union WORD_BITS TU_BinStatus;


extern uint16_t TestWFind;             // *** DAH Added for test waveform capture

extern struct SNAPSHOT_METER SnapshotMeteredValues; 

extern float Vll_max;
extern float Vll_min;
extern float CurOneCycMax;
extern float Vbattery;


extern struct USER_WF_CAPTURE UserWF;
extern struct USER_SAMPLES UserSamples;


extern float ThermMemReading;         // *** BP Added for test

//------------------------------------------------------------------------------------------------------------
//                    Global Function Declarations
//------------------------------------------------------------------------------------------------------------
//
extern void Meter_VarInit(void);
extern void AFE_Process_Samples(void);
extern void Calc_Meter_Current(void);
extern void Calc_Prot_Current(void);
extern void Calc_Meter_AFE_Voltage(void);
extern void Calc_Prot_AFE_Voltage(void);
extern void Calc_Meter_Power(void);
extern void Calc_Prot_Power(void);
extern void Calc_Energy(void);
extern void Calc_AppPF(void);
extern void Calc_DispPF_THD(void);
extern void Calc_CF(void);
extern void Calc_KFactor(void);
extern void ADC_Process_Samples(void);
extern void TA_Volt_Monitoring(void);
extern void AuxPower_Monitoring(void);
extern void Calc_BatteryVolt(void);
extern void ResetEnergy(void);
extern void Calc_ADC_200ms_Voltage(void);

extern void Calc_ADC_OneCyc_Voltage(void);

extern void Calc_Harmonics(void);
extern void ManageSPI1Flags(void);
extern float TP_CoilTempRMSavg(void);
extern void Calc_SeqComp_PhAng(void);
extern void Calc_Freq(struct FREQ_MEASURE_VARS *f_ptr, float v_xn);
extern void SnapshotMeter(uint16_t SummaryCode);
extern void ExtCapSnapshotMeterOneCyc(void);
extern void ExtCapSnapshotMeterTwoHundred(void);
extern uint8_t Update_PSC(uint32_t *pPriSecCause);
extern uint16_t Update_Std_Status(union WORD_BITS *pStd_TU_Status);

extern void ResetMinMax(void);

extern uint8_t CaptureUserWaveform(uint16_t type);

