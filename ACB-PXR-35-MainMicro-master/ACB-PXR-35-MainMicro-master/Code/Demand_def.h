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
//  MODULE NAME:        Demand_def.h
//
//  MECHANICS:          This is the definitions file for the Demand.c module
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.23   180504  DAH File Creation
//   0.32   190726  DAH - Deleted voltages from struct ENERGY_DEMAND_STRUCT.  They are no longer required.
//                      - Added struct DEMAND_I_MIN_MAX_STRUCT and struct DEMAND_P_MAX_STRUCT to support
//                        min and max current and power demands
//   0.37   200604  DAH - Added Spare to struct ENERGY_DEMAND_STRUCT to eliminate padding.  The .map file
//                        was checked to verify this
//   0.50   220203  DAH - Added TotSumWHr and TotSumVarHr to struct ENERGY_DEMAND_STRUCT to make Modbus
//                        messaging simpler
//   0.51   220304  DAH - Added struct FIVEMIN_AVGVALS_STRUCT, struct FIVEMIN_MINMAXVALS_STRUCT, and
//                        NUM5MINVALS definition to support 5-minute values
//   0.52   220309  DAH - Deleted In, Ig, and voltages from struct FIVEMIN_AVGVALS_STRUC and struct
//                        FIVEMIN_MINMAXVALS_STRUCT based on design review
//                      - Added powers to struct FIVEMIN_AVGVALS_STRUCT and struct FIVEMIN_MINMAXVALS_STRUCT
//                        based on design review
//                      - Deleted TotSumWHr and TotSumVarHr to struct ENERGY_DEMAND_STRUCT.  These would be
//                        logged, and they shouldn't be.  They are replaced by separate variables
//   0.72   230320  DAH - In struct DEMAND_I_MIN_MAX_STRUCT, renamed CurLastResetTS to ResetTS
//                      - In struct DEMAND_P_MAX_STRUCT, renamed PwrLastResetTS to ResetTS
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
//    Constants
//------------------------------------------------------------------------------------------------------------



//
//------------------------------------------------------------------------------------------------------------
//    Structure & Unions
//------------------------------------------------------------------------------------------------------------

struct ENERGY_DEMAND_STRUCT                 // Energy and Demand Structure
{                                               // Note, the structure is intentionally arranged in this
  uint32_t EID;                                 //   order to minimize padding bytes
  struct INTERNAL_TIME TS;                      // There are 12 32-bit values, followed by 5 64-bit values
  uint32_t Status;                              // Defined as a uint32 so there is no padding
  float DmndIa;
  float DmndIb;
  float DmndIc;
  float DmndIn;
  float DmndTotW;
  float DmndTotVar;
  float DmndTotVA;
  float Spare;                                  // Defined so no padding
  unsigned long long TotFwdWHr;
  unsigned long long TotRevWHr;
  unsigned long long TotLagVarHr;
  unsigned long long TotLeadVarHr;
  unsigned long long TotVAHr;
};

struct ENERGY_DEMAND_SAVE_EVENT                 // Energy and Demand Save Event Structure
{
  uint32_t EID;
  struct INTERNAL_TIME TS;
};

struct DEMAND_I_MIN_MAX_STRUCT              // Min and Max Current Demand Structure
{
  float IaMin;                                  // All values are 32 bits, so don't need to worry about
  struct INTERNAL_TIME IaMinTS;                 //   padding
  float IbMin;
  struct INTERNAL_TIME IbMinTS;
  float IcMin;
  struct INTERNAL_TIME IcMinTS;
  float InMin;
  struct INTERNAL_TIME InMinTS;
  float IaMax;
  struct INTERNAL_TIME IaMaxTS;
  float IbMax;
  struct INTERNAL_TIME IbMaxTS;
  float IcMax;
  struct INTERNAL_TIME IcMaxTS;
  float InMax;
  struct INTERNAL_TIME InMaxTS;
  struct INTERNAL_TIME ResetTS;
};

struct DEMAND_P_MAX_STRUCT                  // Max Power Demand Structure
{                                               // All values are 32 bits, so don't need to worry about
  float TotWMax;                                //   padding
  struct INTERNAL_TIME TotWMaxTS;
  float TotVarMax;
  struct INTERNAL_TIME TotVarMaxTS;
  float TotVAMax;
  struct INTERNAL_TIME TotVAMaxTS;
  struct INTERNAL_TIME ResetTS;
};

struct FIVEMIN_AVGVALS_STRUCT               // Five-minute average values structure
{                                           //   Note: These must align with const FIVEMIN_INPUT_VALS[]!
  float Ia;
  float Ib;
  float Ic;
  float Iavg;
  float Pa;
  float Pb;
  float Pc;
  float Ptot;
  float AppPa;
  float AppPb;
  float AppPc;
  float AppPtot;
};

struct FIVEMIN_MINMAXVALS_STRUCT            // Five-minute min/max values structure
{                                           //   Note: These must align with const FIVEMIN_INPUT_VALS[]!
  float Iamax;
  float Iamin;
  float Ibmax;
  float Ibmin;
  float Icmax;
  float Icmin;
  float Iavgmax;
  float Iavgmin;
  float Pamax;
  float Pamin;
  float Pbmax;
  float Pbmin;
  float Pcmax;
  float Pcmin;
  float Ptotmax;
  float Ptotmin;
  float AppPamax;
  float AppPamin;
  float AppPbmax;
  float AppPbmin;
  float AppPcmax;
  float AppPcmin;
  float AppPtotmax;
  float AppPtotmin;
};

#define NUM5MINVALS     (sizeof(struct FIVEMIN_AVGVALS_STRUCT)/4)




