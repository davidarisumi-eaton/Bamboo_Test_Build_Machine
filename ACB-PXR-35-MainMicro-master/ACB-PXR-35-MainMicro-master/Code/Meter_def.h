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
//  MECHANICS:          This is the definitions file for the Meter.c module
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
//   0.04   150928  DAH Added structures CURRENTS_WITH_G, CURRENTS_WITHOUT_G, VOLTAGES_LN, VOLTAGES_LL,
//                      and POWERS
//   0.06   151110  DAH Added structure AFE_CAL and revised it to have a single checksum and complement
//   0.08   160122  DAH - Structure CURRENTS_WITH_G renamed to CUR_WITH_G_F
//                      - Structure CURRENTS_WITHOUT_G renamed to CUR_WITHOUT_G_F
//                      - Added structure CUR_WITH_G_I
//   0.09   160224  DAH - Added AFE_CAL_DEFAULT_GAIN and AFE_CAL_DEFAULT_OFFSET
//   0.10   160310  DAH - Added structure ADC_CAL
//                      - Added default cal constants for the ADC (moved from Meter.c)
//                      - Revised AFE_CAL_DEFAULT_GAIN per input from George Gao
//   0.12   160502  DAH - Renamed AFE_CAL_DEFAULT_GAIN to AFE_CAL_DEFAULT_IGAIN
//                      - Renamed AFE_CAL_DEFAULT_OFFSET to AFE_CAL_DEFAULT_IOFFSET
//                      - Added AFE_CAL_DEFAULT_VGAIN and AFE_CAL_DEFAULT_VOFFSET
//                      - Added APP_POWERS
//                      - Added AFE_CAL_DEFAULT_PHASE
//   0.13   160512  DAH - Added ENERGY_FLOATS and ENERGY_UINT64
//   0.14   160620  DAH - Added AFE and ADC default cal constants for N-frame and commented out R-frame
//                        cal constants
//                      - Added ADC_MAX_SEEDVAL_RFRAME and ADC_MAX_SEEDVAL_NFRAME
//                      - Added support for Test Injection
//                          - NORMAL_a1 and TESTINJ_a1 added
//   0.22   180420  DAH - Added "f" to constants definitions to ensure they are interpreted as floats
//                      - Added ENERGY_DEMAND_STRUCT and ENERGY_DEMAND_SAVE_EVENT definitions to support
//                        Demand and Energy logging
//   0.23   180504  DAH - Deleted ENERGY_DEMAND_STRUCT and ENERGY_DEMAND_SAVE_EVENT definitions  (moved to
//                        Demand_def.h)
//   0.32   190726  DAH - Deleted struct VOLTAGES_LL as it is no longer used (placed in struct VOLTAGES)
//                      - Added struct VOLTAGES
//                      - Added total real, reactive, and apparent powers to structs POWERS and APP_POWERS
//                      - Added struct PF_VALUES
//   0.33   190823  DAH - Added struct MIN_MAX_CUR_WITH_G_F, struct MIN_MAX_VOLTAGES, struct MIN_MAX_POWERS,
//                        and struct MIN_MAX_APP_POWERS
//                      - Added struct FREQ_MEASURE_VARS
//   0.35   191118  DAH - In struct ADC_CAL, reduced size of gain[] and offset[] from 10 to 8.  The last two
//                        cal constants were not used.  Also repurposed the cal constant at index = 4 for
//                        In sensed with a CT.  It had been unused.
//                      - In struct AFE_CAL, increased size of gain[], offset[], and phase[] from 8 to 10.
//                        The added cal constants are for Igsrc sensed via a Rogowski coil and In sensed via
//                        a CT.  There already are cal constants at index = 4 for Igsrc using a CT, and at
//                        index = 3 for In using a Rogowski coil.  Also added 2 spare bytes to keep the
//                        alignment even (so no padding would be added).
//                      - Changed AFE_CAL_DEFAULT_VGAIN from 1.59E-4V/bit to 1.797E-4V/bit per Rev 4
//                        schematic
//                      - Added ADC_CAL_DEFAULT_VGAIN and ADC_CAL_DEFAULT_VOFFSET
//                      - Added AFE_CAL_CT_IGAIN
//                      - Added ADC_CAL_CT_IGAIN_HIGH, ADC_CAL_CT_IGAIN_LOW, and ADC_CAL_CT_IOFFSET
//   0.50   220203  DAH - Added struct K_FACTORS
//   0.51   220304  DAH - Added struct SEQ_COMP
//                      - Revised struct MIN_MAX_CUR_WITH_G_F to replace Igsrc and Igres with a single Ig
//                      - Added struct MIN_MAX_CUR_WITHOUT_G_F
//   0.53   220325  DAH - Added struct THD_MIN_MAX
//                      - Added struct HARMONICS_I_STRUCT
//   0.61   221001  DB  - Added struct SNAPSHOT_METER definition
//   0.69   220214  DB  - Added struct EXTCAP_SNAPSHOT_METER definition
//   0.70   230224  BP  - Added battery scaling and TA scaling/thresholds
//   0.72   230320  DAH - Changed default current scaling constants from N-Frame to Magnum Standard Frame
//                  DB  - Deleted EXTCAP_SNAPSHOT_METER definition (moved to Events_def.h)
//    24    230323  DAH - Added preliminary support for PriSecCause status
//                          - Added primary status code definitions (PSC_PRI_xxxx)
//    40    230531  DAH - Revised struct SNAPSHOT_METER
//    94    231010  DAH - Added user waveform capture type definitions (USR_TYPE_xx, moved from Inter_def.h)
//                      - Added struct USER_WF_CAPTURE definition
//                      - Added struct USER_SAMPLES definition (moved from Intr_def.h)
//   141    240115  BP  - Added Config cause and removed RAM and ROM cause
//   142    240119 DAH - Added default frame and VDB cal definitions
//                      - Revised struct AFE_CAL to support both 50Hz and 60Hz phase cal constants.
//                        Structure size did not change.  Two spare bytes were repurposed for additional
//                        phase cal constants
//   148    240131  BP  - Added Aux Power scaliing and threshold      
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
//
//        The Rogowski scaling in mV/A is:
//           NF                  0.345
//           RF                  0.0862
//           Magnum Standard     0.166
//           Magnum Standard DW  0.166
//           Magnum Narrow       0.208
//           Magnum Narrow DW    0.208

#define NORMAL_a1               9.9691733373313E-1f             // Integrator constant for normal operation
#define TESTINJ_a1              9.90742468983431E-1f            // Integrator constant for test injection

#define SCALE_FACTOR_b0         7.845909572784E-2f              // 1/b0 is the 60Hz gain of the dig filter
//#define SCALE_FACTOR_b0         7.87017068246188E-2               // 1/b0 is the 60Hz gain of the dig filter  For test injection
// Default calibration constants for AFE samples
//#define AFE_CAL_DEFAULT_IGAIN  2.8786248E-7                   // For on-board DC voltages
//#define AFE_CAL_DEFAULT_IOFFSET 89.71234E-6
//#define AFE_CAL_DEFAULT_IGAIN  (6.81E-3 * SCALE_FACTOR_b0)    // For 60Hz input R-Frame
//#define AFE_CAL_DEFAULT_IGAIN  (1.73E-3 * SCALE_FACTOR_b0)      // For 60Hz input N-Frame
#define AFE_CAL_DEFAULT_IGAIN  (3.57E-3 * SCALE_FACTOR_b0)      // For 60Hz input Magnum Standard Frame

#define AFE_CAL_CT_IGAIN    1.915E-4                            // Default current gain with CT
                                                                // For DC input R-Frame (secondary injection
                                                                //   with DC signal)
//#define AFE_CAL_DEFAULT_IGAIN   (6.81E-3/(1.274549484E1 * 2.545169957937348E+01))

#define AFE_CAL_DEFAULT_IOFFSET     0.0f

#define AFE_CAL_DEFAULT_VGAIN       1.797E-4f
#define AFE_CAL_DEFAULT_VOFFSET     0.0f

#define AFE_CAL_DEFAULT_PHASE       0

// Default calibration constants for ADC samples
//#define ADC_CAL_DEFAULT_IGAIN_LOW    2.66E+2                  // For 60Hz input R-Frame
//#define ADC_CAL_DEFAULT_IGAIN_HIGH   5.22                     // For 60Hz input R-Frame
//#define ADC_CAL_DEFAULT_IGAIN_LOW    5.69E+1f                 // For 60Hz input N-Frame
//#define ADC_CAL_DEFAULT_IGAIN_HIGH   1.12f                    // For 60Hz input N-Frame
#define ADC_CAL_DEFAULT_IGAIN_LOW    1.34E+2f                   // For 60Hz Magnum Standard Frame
#define ADC_CAL_DEFAULT_IGAIN_HIGH   2.64f                      // For 60Hz Magnum Standard Frame
#define ADC_CAL_DEFAULT_IOFFSET      0.0f

#define ADC_CAL_CT_IGAIN_HIGH        1.0f                       // *** DAH NEED TO DETERMINE THESE
#define ADC_CAL_CT_IGAIN_LOW         1.0f                       // *** DAH NEED TO DETERMINE THESE
#define ADC_CAL_CT_IOFFSET           0.0f

#define ADC_CAL_DEFAULT_VGAIN        1.127E-1f
#define ADC_CAL_DEFAULT_VOFFSET      0.0f

// Maximum valid ADC sample for seeding for the high-gain circuit
#define ADC_MAX_SEEDVAL_RFRAME      1.0E4f                      // 2^11 x 5.22A/bit ~ 10,000A
#define ADC_MAX_SEEDVAL_NFRAME      2.0E3f                      // 2^11 x 1.12A/bit ~ 2,000A


// Defaults for VDB board
#define ROGO_DEFAULT_IGAIN      1.0                     // Default Rogowski coil gain
#define VDB_DEFAULT_VGAIN       0x4000                  // Default VDB voltage gain (Q14 format = 1.00)
#define VDB_DEFAULT_VOFFSET     0x0000                  // Default VDB voltage offset (Q14 format = 0.00)
#define VDB_DEFAULT_PHASE       0x0000                  // Default VDB voltage phase shift



#define BATT_VOLT_SCALING           9.151E-4f                   // 1.499 x (2.5/4095)
#define TA_VOLT_SCALING             1.374E-2f                   // (225/10) x (2.5/4095)
#define AUXPWR_VOLT_SCALING         1.282E-2f                   // (210/10) x (2.5/4095)

// TA voltage monitoring thresholds
#define TA_LOWVOLT_THRESHOLD        38.0f                       // 38.0V - lower than this threshold, TA can't trip
#define TA_OPENVOLT_THRESHOLD       18.0f                       // 18.0V - lower than this threshold, TA open/disconnect, insert alarm event
#define TA_LOWVOLT_BOOST_THRESHOLD  34.0f                       // 34.0V - TA threshold for USB boost. USB boost is not capable of provideing 38V

// Aux Power voltage monitoring thresholds
#define AUXPWR_LOWVOLT_THRESHOLD    19.5f                       // 19.5V - from Tokyo (MCU2 Powersys_def.h)

// Primary Status Codes
#define PSTATUS_OPEN                0x01    // Open
#define PSTATUS_CLOSED              0x02    // Closed
#define PSTATUS_TRIPPED             0x03    // Tripped
#define PSTATUS_ALARMED             0x04    // Alarmed
#define PSTATUS_PU                  0x0D    // Pickup

// Second Status Codes
#define SSTATUS_NA      0x0001      //not applicable
#define SSTATUS_TEST    0x0003      //test mode
#define SSTATUS_POWERUP 0x0007      //powered-up since last trip/alarm reset
#define SSTATUS_ALARM   0x0008      //alarm

// Cause of Status Codes
#define CAUSE_UNKNOWN   0x0000      //unknown
#define CAUSE_NORM      0x0001      //normal
#define CAUSE_INST      0x0003      //instantaneous
#define CAUSE_OV        0x000B      //over voltage
#define CAUSE_UV        0x000C      //under voltage
#define CAUSE_AUXUV     0x000E      //aux-power under power
#define CAUSE_OF        0x000F      //over frequency
#define CAUSE_UF        0x0010      //under frequency
#define CAUSE_CUNB      0x0011      //current un-balance
#define CAUSE_VUNB      0x0012      //voltage un-balance
#define CAUSE_VAPF      0x0013      //apparent power factor
#define CAUSE_PWDE      0x001A      //power demand
#define CAUSE_VADE      0x001B      //VA demand
#define CAUSE_THD       0x001E      //THD
#define CAUSE_BRK_HEALTH     0x001F      //operations count
#define CAUSE_COMCONT   0x0021      //control via communications
#define CAUSE_COIL      0x0025      //coil supervision
#define CAUSE_WRONGSENSOR      0x0026      //coil supervision
#define CAUSE_DW1       0x0027      //diagnostic warning #1 (A/D calibration)
#define CAUSE_BATT      0x0029      //Battery low 
#define CAUSE_LONG      0x003D      //long delay
#define CAUSE_SHORT     0x003E      //short delay
#define CAUSE_PLUG      0x0040      //bad/missing rating plug
#define CAUSE_REVP      0x0041      //reverse power
#define CAUSE_REVS      0x0044      //reverse sequence
#define CAUSE_PCL       0x0045      //phase current loss
#define CAUSE_HLOAD_1   0x0049      //High Load Alarm 1 
#define CAUSE_HLOAD_2   0x00A1      //High Load Alarm 2 
#define CAUSE_T_MEM     0x009F      //Thermal Memory Alarm 
#define CAUSE_GF_PRE    0x00A2      //Ground fault pre-alarm 
#define CAUSE_HIGH_TEMP 0x00A3      //High temperature Alarm 
#define CAUSE_MCR       0x004B      //making current release
#define CAUSE_INSTOR    0x004C      //fixed hardware Inst.
#define CAUSE_STPE      0x004D      //setpoints error
#define CAUSE_OT        0x004E      //over temperature
#define CAUSE_LDNEUT    0x0050      //long delay neutral over current
#define CAUSE_SDNEUT    0x009E      //short delay neutral over current
#define CAUSE_INSTNEUT  0x0005      //inst neutral over current
#define CAUSE_GF        0x0054      //ground fault
#define CAUSE_EF        0x0055      //earth fault   
#define CAUSE_CAL       0x0071      //calibration
#define CAUSE_CONFIG    0x0072      //breaker config error
#define CAUSE_RTC       0x0088      //real time clock
#define CAUSE_MM        0x0099      //MM mode
#define CAUSE_MECH      0x009A      //breaker mechanism fault
#define CAUSE_FRAME     0x009B      //Frame board version fault
#define CAUSE_REALP     0x0002      //Real Power    
#define CAUSE_REACTP    0x0004      //Reactive Power
#define CAUSE_APP       0x0006      //Apparent Power         
#define CAUSE_UPF       0x0007      //Under Power Factor
#define CAUSE_REVVARP   0x0008      //Reverse Reactive Power
#define CAUSE_REALPD    0x0009      //Real Power Demand
#define CAUSE_APPD      0x000A      //Apparent Power Demand
#define CAUSE_NEU       0x0014      //Neutral
#define CAUSE_ELF       0x0015      //Electrical Alarm
#define CAUSE_CTHD      0x0016      //Current THD
#define CAUSE_VTHD      0x0017      //Voltage THD
#define CAUSE_DTA       0x0018      //Display Over Temperature
#define CAUSE_TTT       0x0019      //Time To Trip
#define CAUSE_TCAP      0x001C      //Thermal Capacity

// non-standard Cause of Status Codes
//#define CAUSE_RAM       0x07FA      //RAM error  <--(not used)
//#define CAUSE_ROM       0x07FB      //ROM error  <--(not used)
#define CAUSE_DBPASS    0X07FC      //Digital Bypass
#define CAUSE_NVME      0x07FD      //product-defined code 2045: Non-volatile memory error
#define CAUSE_WTDG      0x07FE      //product-defined code 2046: watchdog fault
#define CAUSE_MOTOR     0x07FF      //Motor alarm or trip
#define CAUSE_LCDOPEN   0x00A0      //product-defined code 2044: Trip breaker from LCD side



#define DS_LD_PICKUP        0x00                                // Long Delay Pickup
#define DS_LD               (((uint32_t)1) << DS_LD_PICKUP)


// RESET flags definitions

#define RESET_MINMAX_ALL              0     //Reset all min/max values
#define RESET_MINMAX_ALL_FLAG               (((uint32_t)1) << RESET_MINMAX_ALL)

#define RESET_MINMAX_CURR             1     //Reset min/max currents
#define RESET_MINMAX_CURR_FLAG              (((uint32_t)1) << RESET_MINMAX_CURR)

#define RESET_MINMAX_LL_VOLT          2     //Reset min/max LL voltages
#define RESET_MINMAX_LL_VOLT_FLAG           (((uint32_t)1) << RESET_MINMAX_LL_VOLT)

#define RESET_MINMAX_LN_VOLT          3     //Reset min/max LN voltages
#define RESET_MINMAX_LN_VOLT_FLAG           (((uint32_t)1) << RESET_MINMAX_LN_VOLT)

#define RESET_MINMAX_PF               4     //Reset min/max PF
#define RESET_MINMAX_PF_FLAG                (((uint32_t)1) << RESET_MINMAX_PF)

#define RESET_MINMAX_FREQ             5     //Reset min/max frequency
#define RESET_MINMAX_FREQ_FLAG              (((uint32_t)1) << RESET_MINMAX_FREQ)

#define RESET_MINMAX_PWR              6     //Reset min/max powers
#define RESET_MINMAX_PWR_FLAG               (((uint32_t)1) << RESET_MINMAX_PWR)

#define RESET_PEAK_DMNCURR            7     //Reset peak demand current
#define RESET_PEAK_DMNCURR_FLAG             (((uint32_t)1) << RESET_PEAK_DMNCURR)

#define RESET_PEAK_DMNPWR             8     //Reset peak power demand
#define RESET_PEAK_DMNPWR_FLAG              (((uint32_t)1) << RESET_PEAK_DMNPWR)

#define RESET_ALLDMNDW                9     //Reset all demand windows
#define RESET_ALLDMNDW_FLAG                 (((uint32_t)1) << RESET_ALLDMNDW)

#define RESET_ALLSTATE_FLGS           10     //Reset all change of state flags
#define RESET_ALLSTATE_FLGS_FLAG             (((uint32_t)1) << RESET_ALLSTATE_FLGS)

#define RESET_PWRUP_FLGS              11     //Reset powered-up flag
#define RESET_PWRUP_FLGS_FLAG                (((uint32_t)1) << RESET_PWRUP_FLGS)

#define RESET_ACC_ENG                 12     //Reset accumulated energy
#define RESET_ACC_ENG_FLAG                   (((uint32_t)1) << RESET_ACC_ENG)

#define RESET_HEALTHSUMM              13     //Reset health summary
#define RESET_HEALTHSUMM_FLAG                (((uint32_t)1) << RESET_HEALTHSUMM)

#define RESET_TRIP_CNTR               14     //Reset Trip counters
#define RESET_TRIP_CNTR_FLAG                 (((uint32_t)1) << RESET_TRIP_CNTR)

#define RESET_OOPTS                   15     //Reset Opts
#define RESET_OOPTS_FLAG                     (((uint32_t)1) << RESET_OOPTS)

#define RESET_TEMP                    16     //Reset Temp
#define RESET_TEMP_FLAG                      (((uint32_t)1) << RESET_TEMP)

#define RESET_RUNTIME                 17     //Reset Runtime
#define RESET_RUNTIME_FLAG                   (((uint32_t)1) << RESET_RUNTIME)

#define RESET_ALL_EXT_DIAGS           18     //Reset all external diagnostics
#define RESET_ALL_EXT_DIAGS_FLAG             (((uint32_t)1) << RESET_ALL_EXT_DIAGS)

#define RESET_ALL_INT_DIAGS           19     //Reset all internal diagnostics
#define RESET_ALL_INT_DIAGS_FLAG             (((uint32_t)1) << RESET_ALL_INT_DIAGS)

#define RESET_ETU                     20     //Reset Trip Unit
#define RESET_ETU_FLAG                       (((uint32_t)1) << RESET_ETU)

#define RESET_MAX_VAL                 RESET_ETU

#define RESETMIN          0xFFFFFFFF
#define RESETMAX          0


// User waveform capture type definitions
#define USR_TYPE_ALL            0 
#define USR_TYPE_IA             1 
#define USR_TYPE_IB             2 
#define USR_TYPE_IC             3 
#define USR_TYPE_IN             4 
#define USR_TYPE_IGS            5 
#define USR_TYPE_VANAFE         6 
#define USR_TYPE_VBNAFE         7 
#define USR_TYPE_VCNAFE         8 
#define USR_TYPE_VANADC         9 
#define USR_TYPE_VBNADC         10
#define USR_TYPE_VCNADC         11
#define USR_TYPE_LAST           USR_TYPE_VCNADC
#define USR_TYPE_NOREQ          (USR_TYPE_LAST + 1)

#define USERWF_TIMEOUT          8       // This is in seconds
 
//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct CUR_WITH_G_F                     // Structure for currents, including ground currents
{
  float Ia;                         
  float Ib;
  float Ic;
  float In;
  float Igsrc;
  float Igres;
};

struct MIN_MAX_CUR_WITH_G_F             // Structure for min and max currents
{
  float Ia;                         
  float Ib;
  float Ic;
  float In;
  float Ig;
  struct INTERNAL_TIME IaTS;
  struct INTERNAL_TIME IbTS;
  struct INTERNAL_TIME IcTS;
  struct INTERNAL_TIME InTS;
  struct INTERNAL_TIME IgTS;
};

struct CUR_WITH_G_I                     // Structure for currents, including ground currents
{
  unsigned long long Ia;                         
  unsigned long long Ib;
  unsigned long long Ic;
  unsigned long long In;
  unsigned long long Igsrc;
  unsigned long long Igres;
};

struct CUR_WITHOUT_G_F                  // Structure for currents, not including ground currents
{
  float Ia;                         
  float Ib;
  float Ic;
  float In;
};

struct MIN_MAX_CUR_WITHOUT_G_F          // Structure for min and max currents without ground
{
  float Ia;                         
  float Ib;
  float Ic;
  float In;
  struct INTERNAL_TIME IaTS;
  struct INTERNAL_TIME IbTS;
  struct INTERNAL_TIME IcTS;
  struct INTERNAL_TIME InTS;
};

struct K_FACTORS
{
    float Ia;
    float Ib;
    float Ic;
};


struct HARMONICS_I_STRUCT               // Structure for Harmonics Final Values
{                                           // Harmonic analysis percent * 100 (100% = 10000, 0xFFFF=undef)
   uint16_t Ia[40];                         // Phase A harmonics [0]=fundamental, [1]=2nd harmonic, etc.
   uint16_t Ib[40];                         // Phase B harmonics
   uint16_t Ic[40];                         // Phase C harmonics
   uint16_t In[40];                         // Neutral harmonics
   uint16_t Vab[40];                        // Vab harmonics
   uint16_t Vbc[40];                        // Vbc harmonics
   uint16_t Vca[40];                        // Vca harmonics
   uint16_t Van[40];                        // Van harmonics
   uint16_t Vbn[40];                        // Vbn harmonics
   uint16_t Vcn[40];                        // Vcn harmonics
};


struct SEQ_COMP
{
    float I_ZeroMag;
    float I_ZeroPh;
    float I_PosMag;
    float I_PosPh;
    float I_NegMag;
    float I_NegPh;
    float V_ZeroMag;
    float V_ZeroPh;
    float V_PosMag;
    float V_PosPh;
    float V_NegMag;
    float V_NegPh;
};


struct VOLTAGES_LN                      // Structure for line-to-neutral voltages
{
  float Van;                         
  float Vbn;
  float Vcn;
};

struct VOLTAGES                         // Structure for line-to-neutral and line-to-line voltages
{
  float Van;                         
  float Vbn;
  float Vcn;
  float Vab;                         
  float Vbc;
  float Vca;
};

struct MIN_MAX_VOLTAGES                 // Structure for min and max voltages
{
  float Van;                         
  float Vbn;
  float Vcn;
  float Vab;                         
  float Vbc;
  float Vca;
  struct INTERNAL_TIME VanTS;
  struct INTERNAL_TIME VbnTS;
  struct INTERNAL_TIME VcnTS;
  struct INTERNAL_TIME VabTS;
  struct INTERNAL_TIME VbcTS;
  struct INTERNAL_TIME VcaTS;
};

struct POWERS
{
  float Pa;
  float Pb;
  float Pc;
  float Ptot;
  float RPa;
  float RPb;
  float RPc;
  float Rtot;
};

struct MIN_MAX_POWERS
{
  float Pa;
  float Pb;
  float Pc;
  float Ptot;
  float RPa;
  float RPb;
  float RPc;
  float Rtot;
  struct INTERNAL_TIME PaTS;
  struct INTERNAL_TIME PbTS;
  struct INTERNAL_TIME PcTS;
  struct INTERNAL_TIME PtotTS;
  struct INTERNAL_TIME RPaTS;
  struct INTERNAL_TIME RPbTS;
  struct INTERNAL_TIME RPcTS;
  struct INTERNAL_TIME RtotTS;
};

struct APP_POWERS
{
  float AppPa;
  float AppPb;
  float AppPc;
  float Apptot;
};

struct MIN_MAX_APP_POWERS
{
  float AppPa;
  float AppPb;
  float AppPc;
  float Apptot;
  struct INTERNAL_TIME AppPaTS;
  struct INTERNAL_TIME AppPbTS;
  struct INTERNAL_TIME AppPcTS;
  struct INTERNAL_TIME ApptotTS;
};

struct AFE_CAL
{
  // Gain and offset cal constants for the AFE:
  //   0 - Ia     1 - Ib     2 - Ic     3 - In for Rogowski       4 - Igsrc for CT
  //   5 - Van    6 - Vbn    7 - Vcn    8 - Igsrc for Rogowski    9 - In for CT
  float gain[10];
  float offset[10];
  // Phase cal constants for the AFE:
  //   60Hz: 0 - Ia     1 - Ib     2 - Ic     3 - Van    4 - Vbn    5 - Vcn
  //   50Hz: 6 - Ia     7 - Ib     8 - Ic     9 - Van    10 - Vbn   11 - Vcn
  uint8_t phase[12];
  uint32_t chk;
  uint32_t cmp;
};

struct ADC_CAL
{
  // Gain and offset cal constants for the ADC:
  //   0 - Ia     1 - Ib     2 - Ic     3 - In for Rogowski
  //   4 - In for CT (applied to Vn also, but doesn't matter)    5 - Van    6 - Vbn    7 - Vcn
  float gain[8];
  float offset[8];
  uint32_t chk;
  uint32_t cmp;
};

struct ENERGY_FLOATS
{
  float FwdWHr;
  float RevWHr;
  float LagVarHr;
  float LeadVarHr;
  float VAHr;
};

struct ENERGY_UINT64
{
  unsigned long long FwdWHr;
  unsigned long long RevWHr;
  unsigned long long LagVarHr;
  unsigned long long LeadVarHr;
  unsigned long long VAHr;
};

struct PF_VALUES
{
//  float AppPha, AppPhb, AppPhc, AppTot;
  float App[4];
  float MinApp[4];
  float MaxApp[4];
  float Disp[4];
  float MinDisp[4];
  float MaxDisp[4];
  struct INTERNAL_TIME MinApp_TS[4];
  struct INTERNAL_TIME MaxApp_TS[4];
  struct INTERNAL_TIME  MinDisp_TS[4];
  struct INTERNAL_TIME  MaxDisp_TS[4];
};

struct FREQ_MEASURE_VARS                            // Frequency measurement variables
{
  float                 FreqVal;
  float                 MinFreqVal;
  float                 MaxFreqVal;
  struct INTERNAL_TIME  MinFreqVal_TS;
  struct INTERNAL_TIME  MaxFreqVal_TS;
  uint16_t              CapTim;
  uint16_t              DeltaTim;
  uint8_t               Start;
  uint8_t               OvrTimer;
  uint8_t               Dump_Count;
};

struct THD_MIN_MAX
{
   float THDmin;
   float THDmax;
   struct INTERNAL_TIME THDminTS;
   struct INTERNAL_TIME THDmaxTS;
 };

struct SNAPSHOT_METER
{
  uint32_t CauseCode;       // Trip cause;

  // To ensure no padding bytes are added, the two uint16 values must be together!
  uint16_t SummaryCode;     // Summary Event Code

  uint16_t BinaryStatus;    // Binary Status

  // Currents
  float Ia_Rms;             // phase A current
  float Ib_Rms;             // phase B current
  float Ic_Rms;             // phase C current
  float In_Rms;             // phase N current
  float Ig_Rms;             // ground fault current

  //Line-to-Neutral Voltages
  float Van1_Rms;           // phase A to Neutral
  float Vbn1_Rms;           // phase A to Neutral
  float Vcn1_Rms;           // phase A to Neutral

  //Line-to-Line Voltages
  float Vab1_Rms;           // phase A to phase B
  float Vbc1_Rms;           // phase B to phase C
  float Vca1_Rms;           // phase C to phase A

  //Line-to-Neutral Voltages
  float Van2_Rms;           // phase A to Neutral
  float Vbn2_Rms;           // phase A to Neutral
  float Vcn2_Rms;           // phase A to Neutral

  //Line-to-Line Voltages
  float Vab2_Rms;           // phase A to phase B
  float Vbc2_Rms;           // phase B to phase C
  float Vca2_Rms;           // phase C to phase A

  //Power
  float P_real;             // real power
  float P_react;            // reactive power
  float P_apparent;         // apparent power

  // Demands
  float Ia_Dmnd;            // phase A current
  float Ib_Dmnd;            // phase B current
  float Ic_Dmnd;            // phase C current
  float In_Dmnd;            // phase N current

  //Demand powers
  float P_real_Dmnd;        // net(real) demand power, Watts
  float Pr_Dmnd;            // reactive demand power, VAR
  float Pa_Dmnd;            // apparent demand power, VA
  
  //Temperature
  float Temperature;

  // Frequency
  float f1;                 // Frequency load
  float f2;                 // Frequency line

  // Power Factor
  float PFa;
  float PFb;
  float PFc;
  float PFtot;

  // Current THD
  float THD_Ia;            
  float THD_Ib;           
  float THD_Ic;            
  float THD_In;

  // Voltage THD
  float THD_Van1;
  float THD_Vbn1;
  float THD_Vcn1;
  float THD_Vab1;
  float THD_Vbc1;
  float THD_Vca1;

  // Crest Factor
  float CFa;
  float CFb;
  float CFc;
  float CFn;

  //Power Factor
//  float PoweFact;

  uint16_t OperationsCount;    //not defined yet

//  float BinaryStatus;     // Binary Status
};

// User waveform capture samples
struct USER_SAMPLES                                 // User waveform capture sample buffer.  Either one
{                                                   //   cycle each of Ia - Igsrc, VanAFE - VcnAFE,
  float TwelveCyc[12 * 80];                         //   VanADC - VcnADC, or 12 cycles of a single waveform
  float OneCyc[11][80];
};

struct USER_WF_CAPTURE
{
  uint8_t Locked;
  uint8_t Type;
  struct INTERNAL_TIME CapTime;
};
