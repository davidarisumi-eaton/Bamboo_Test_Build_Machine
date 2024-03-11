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
//  MODULE NAME:        Demand.c
//
//  MECHANICS:          Program module containing the demand metering subroutines
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.23   180504  DAH - File Creation
//                          - Moved demand code from Meter.c and Iod.c into here
//                          - Revised demand anniversary time measurements from using 200msec counts as a
//                            time base to using the actual internal time as a time base (still sychronized
//                            to 200msec anniversaries)
//                          - Added 1 second delay before starting the demand accumulations because it takes
//                            roughly that long for the metered currents to stabilize through the digital
//                            filter
//                              - Dmnd_Delay added
//                          - In Calc_Demand(), corrected bug in resetting the sub-totals
//                          - Added voltage and power demands
//                              - Renamed DEMAND__I_HISTORY_STRUCT to DEMAND_HISTORY_STRUCT and added
//                                200msec line-to-neutral voltages and total powers
//                              - Renamed  to Dmnd_I_SubTots to Dmnd_SubTots (now also contains the LN
//                                voltages)
//                              - Revised Dmnd_I_VarInit() to initialize Dmnd_SubTots.Vxn[] and
//                                Dmnd_SubTots.Totxx[]
//                              - Renamed Dmnd_I_VarInit() to Dmnd_VarInit()
//                              - Renamed struct Dmnd_I_WinSum to Dmnd_WinSum and added LN voltages and
//                                total powers to DEMAND_STRUCT.
//                              - Revised Dmnd_VarInit() to initialize Dmnd_WinSum.Vxn and Dmnd_WinSum.Totxx
//                              - Added line-to-neutral voltage demands and total power demands to
//                                Calc_I_Demand()
//                              - Renamed Calc_I_Demand() to Calc_Demand()
//                          - Moved the definition of struct Dmnd_WinSum from the global to the local
//                            section as it is only used in this file
//   0.24   180517  DAH - Combined Flash_def.h and FRAM_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Modified Calc_Demand() to use the request flag to write the EID to FRAM
//                            instead of calling FRAM_Write() directly
//   0.32   190726  DAH - Deleted voltages from the demand calculations.  They are no longer required.  They
//                        have been removed from the structure holding the values, EngyDmnd[1].
//                          - Removed Van, Vbn, Vcn from Dmnd_WinSum
//                          - Removed Van[], Vbn[], Vcn[] from Dmnd_SubTots
//                          - Modified Dmnd_VarInit() and Calc_Demand()
//                      - In Dmnd_VarInit(), added code to initialize demand values to NaN
//                      - Added min and max (peak) demands
//                          - Code added to Dmnd_VarInit() to initialize min and max demand values to NaN
//                          - Code added to Calc_Demand() to compute min and max demand values and to
//                            initiate write to FRAM
//                      - Renamed Cur200msec to Cur200msFltr to distinguish it from unfiltered currents
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added includes of RealTime_def.h and RealTime_ext.h
//   0.51   220304  DAH - 5-minute average and min/max values added
//                          - Added Calc_5minAverages() and CalcNext5minAnniversary() to compute the new
//                            5-minute anniversary average and min/max values
//                          - Res5min_Avg, Res5min_MinMax, Sum5min_Avg, Sum5min_MinMax, Next_5minTime,
//                            Delay5, SumCnt added
//                      - Moved SECS_IN_FOUR_YRS, SECS_IN_ONE_YR, SECS_IN_ONE_DAY, SECS_IN_ONE_HR, and
//                        SECS_IN_MONTH[] to top of module from inside Calc_5minAverages() as these
//                        definitions and constants are now also used in CalcNext5minAnniversary()
//   0.52   220309  DAH - Deleted In, Ig, and voltages from struct FIVEMIN_AVGVALS_STRUC and struct
//                        FIVEMIN_MINMAXVALS_STRUCT based on design review
//                      - Added powers to struct FIVEMIN_AVGVALS_STRUCT and struct FIVEMIN_MINMAXVALS_STRUCT
//                        based on design review
//                      - Completed 5-minute min/max values computations
//                          - Sum5min_MinMax moved from global to local section
//                          - Calc_5minAverages() revised
//                      - Revised Calc_Demand() to make logging anniversaries completely independent of
//                        demand window anniveraries
//   0.59   220831  DAH - Revised Calc_Demand() to replace code that sets S2NF_WR_EID, S2NF_IDMND_WR, and
//                        S2NF_PDMND_WR with calls to FRAM_Write() and FRAM_WriteMinMax()
//   0.69   230220  DAH - In Calc_Demand(), NewEventFIFO[].Type renamed to NewEventFIFO[].Code
//   0.72   230320  DAH - Added Dmnd_Type_Window for proc-proc comms
//    129   231213  MAG - Changed Setpoints0.stp.CurrentWindow to Setpoints0.stp.DemandWindow (in comment)
//    149   240131  DAH - Revised Dmnd_VarInit() to compute Dmnd_NumSubInt_Per_DmndWin and
//                        Dmnd_Num200ms_Per_SubInt based on the setpoints (these values were fixed in
//                        initial development)
//                      - Revised Calc_Demand() to use the setpoint to determine whether a fixed or sliding
//                        window is used (values was fixed in initial development)
//                      - Moved CalcNextAnniversaries() from global to local section
//                      - Revised CalcNextAnniversaries() to use setpoints to determine next anniversaries
//                      - Fixed bug in Calc_Demand() where EID was being incremented with each entry
//    150   240202  DAH - Revised Calc_Demand() to handle cases where demand calculations and/or demand
//                        logging may be disabled.  Switched subroutine to a state machine format
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
#include "Meter_def.h"
#include "Events_def.h"
#include "Demand_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!



//
//      Local Definitions used in this module...
//
#define HISTORY_BUF_SIZE        60

#define SECS_IN_FOUR_YRS        (1461L * 24L * 60L * 60L)
#define SECS_IN_ONE_YR          (365L * 24L * 60L * 60L)
#define SECS_IN_ONE_DAY         (24L * 60L * 60L)
#define SECS_IN_ONE_HR          (60L * 60L)


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
#include "Events_ext.h"
#include "Meter_ext.h"
#include "Setpnt_ext.h"



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Dmnd_VarInit(void);
void Calc_Demand(void);
void CalcNext5minAnniversary(uint32_t time_secs, uint32_t *next_5min);


//      Local Function Prototypes (These functions are called only within this module)
//
void CalcNextAnniversaries(uint32_t time_secs, uint32_t *next_si, uint32_t *next_dw, uint32_t *next_la);


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct ENERGY_DEMAND_STRUCT EngyDmnd[2];
uint16_t Dmnd_Num200ms_Per_SubInt;                      // Number of 200msec readings per sub-interval
uint16_t Dmnd_Num200ms_Per_DmndWin;                     // Number of 200msec readings per demand window
uint8_t Dmnd_NumSubInt_Per_DmndWin;                     // Number of sub-intervals per demand window
uint8_t Dmnd_ForceLog;                                  // Flag to force a demand and energy logging event
uint8_t Dmnd_Setp_DemandWindow;                         // Copy of corresponding setpoint
uint8_t Dmnd_Setp_DemandInterval;                       // Copy of corresponding setpoint
uint8_t Dmnd_Setp_DemandLogInterval;                    // Copy of corresponding setpoint
uint32_t Dmnd_Type_Window;                              // Window type and interval for proc-proc comms

struct ENERGY_DEMAND_SAVE_EVENT Dmnd_SaveEvent;

uint32_t Next_SI_Time, Next_DW_Time, Next_LA_Time;      // Next anniversary times in seconds

struct DEMAND_I_MIN_MAX_STRUCT IDmnd;
struct DEMAND_P_MAX_STRUCT PDmnd;

struct FIVEMIN_AVGVALS_STRUCT Res5min_Avg;
struct FIVEMIN_MINMAXVALS_STRUCT Res5min_MinMax;


//------------------------------------------------------------------------------------------------------------
//                   Global Constants used in this module and other modules
//------------------------------------------------------------------------------------------------------------
//
const uint32_t SECS_IN_MONTH[12] = 
{
  (31L * 24L * 60L * 60L),              // Jan
  (28L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L),
  (30L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L),
  (30L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L),
  (30L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L),
  (30L * 24L * 60L * 60L),
  (31L * 24L * 60L * 60L)               // Dec
};

// 5-minute input values
//   Note: These must align with struct FIVEMIN_AVGVALS_STRUCT and struct FIVEMIN_MINMAXVALS_STRUCT!
float * const FIVEMIN_INPUT_VALS[] =
{
  &Cur200msFltr.Ia,     &Cur200msFltr.Ib,     &Cur200msFltr.Ic,     &Cur200msIavg,
  &Pwr200msec.Pa,       &Pwr200msec.Pb,       &Pwr200msec.Pc,       &Pwr200msec.Ptot,
  &Pwr200msecApp.AppPa, &Pwr200msecApp.AppPb, &Pwr200msecApp.AppPc, &Pwr200msecApp.Apptot
};



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
struct DEMAND_HISTORY_STRUCT                            // Demand Window History
{
  float Ia[HISTORY_BUF_SIZE];                           // Sub-interval current sums
  float Ib[HISTORY_BUF_SIZE];
  float Ic[HISTORY_BUF_SIZE];
  float In[HISTORY_BUF_SIZE];
  float TotW[HISTORY_BUF_SIZE];                         // Sub-interval power sums
  float TotVar[HISTORY_BUF_SIZE];
  float TotVA[HISTORY_BUF_SIZE];
  uint16_t NumVals[HISTORY_BUF_SIZE];                   // Number of readings in the sub-interval
} Dmnd_SubTots;
struct DEMAND_STRUCT                                    // Demand window sums
{
  float Ia;
  float Ib;
  float Ic;
  float In;
  float TotW;
  float TotVar;
  float TotVA;
} Dmnd_WinSum;
uint16_t Dmnd_200msecCnt;                               // Demand 200msec-period counter
uint8_t Dmnd_SubIntCnt;                                 // Sub-interval counter and index
uint8_t Dmnd_Delay;                                     // Reset delay counter before starting demand calculations

struct FIVEMIN_AVGVALS_STRUCT Sum5min_Avg;
struct FIVEMIN_MINMAXVALS_STRUCT Sum5min_MinMax;
uint32_t Next_5minTime;
uint8_t Delay5;
uint16_t SumCnt;




//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Dmnd_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Current Demand Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in Current Demand computations.  A
//                      separate subroutine is required because these variables are initialized not only
//                      when the unit is reset, but also when the Current Demand process is reset (when it
//                      is resynchronized).
//                      The following variables do not need initialization:
//                          EngyDmnd[].xx
//                          Dmnd_SaveEvent
//                          Next_SI_Time (initialized in Calc_Demand())
//                          Next_DW_Time (initialized in Calc_Demand())
//                          Next_LA_Time (initialized in Calc_Demand())
//                          Res5min_Avg.xxx (initialized in Calc_5minAverages())
//                          Res5min_MinMax.xxx (initialized in Calc_5minAverages())
//                          Sum5min_Avg.xxx (initialized in Calc_5minAverages())
//                          Sum5min_MinMax.xxx (initialized in Calc_5minAverages())
//                          Next_5minTime (initialized in Calc_5minAverages())
//                          SumCnt (initialized in Calc_5minAverages())
//                          Dmnd_Type_Window (initialized in Gen_Values after setpoints are retrieved)
//
//  CAVEATS:            None
//
//  INPUTS:             HISTORY_BUF_SIZE
//
//  OUTPUTS:            Dmnd_200msecCnt, Dmnd_SubIntCnt, Dmnd_WinSum.xx, Dmnd_NumSubInt_Per_DmndWin,
//                      Dmnd_Num200ms_Per_SubInt, Dmnd_Num200ms_Per_DmndWin, Dmnd_SubTots.xx[0],
//                      Dmnd_ForceLog, EngyDmnd[1].Dmndxxx, IDmnd.xxx, PDmnd.xxx, Delay5
//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code, no FRAQM reads): 108usec
//
//------------------------------------------------------------------------------------------------------------

void Dmnd_VarInit(void)
{
  uint8_t i;
  float *resptr;

  Dmnd_200msecCnt = 0;
  Dmnd_SubIntCnt = 0;

  Dmnd_WinSum.Ia = 0;
  Dmnd_WinSum.Ib = 0;
  Dmnd_WinSum.Ic = 0;
  Dmnd_WinSum.In = 0;
  Dmnd_WinSum.TotW = 0;
  Dmnd_WinSum.TotVar = 0;
  Dmnd_WinSum.TotVA = 0;

  // Save copies of the setpoints so we can tell whether they changed.  All values fit in U8's
  Dmnd_Setp_DemandWindow = (uint8_t)Setpoints0.stp.DemandWindow;
  Dmnd_Setp_DemandInterval = (uint8_t)Setpoints0.stp.DemandInterval;
  Dmnd_Setp_DemandLogInterval = (uint8_t)Setpoints0.stp.DemandLogInterval;

  // Calculate the number of sub-intervals in the demand window
  // If the demand window is 15 minutes or less, the number of sub-intervals per demand window is
  // 4 * demand window length (15sec sub-interval time).  Otherwise, it is the demand window length
  // (60sec sub-interval time)
  Dmnd_NumSubInt_Per_DmndWin = ( (Dmnd_Setp_DemandInterval <= 15) ?
                                    (4 * Dmnd_Setp_DemandInterval) : Dmnd_Setp_DemandInterval );
//  Dmnd_NumSubInt_Per_DmndWin = 20;         // *** DAH  FOR NOW, FIXED AT 5 MINUTE INTERVAL. MAYBE MOVE TO GEN_VALS()
//  Dmnd_NumSubInt_Per_DmndWin = 40;         // *** DAH  FOR NOW, FIXED AT 10 MINUTE INTERVAL.

  // Calculate the number of 200msec periods in the sub-interval
  // If the demand window is 15 minutes or less, the sub-interval time is 15 seconds, so the number of
  // 200msec periods in a sub-interval is 15sec/200msec = 75.  If the window is greater than 15 minutes, the
  // sub-interval time is 60sec, so the number of periods in a sub-interval is 60sec/200msec = 300.
  Dmnd_Num200ms_Per_SubInt = ((Dmnd_Setp_DemandInterval <= 15) ? 75 : 300);
//  Dmnd_Num200ms_Per_SubInt = 75;        // *** DAH  FOR NOW, FIXED AT 5 MINUTE SLIDING INTERVAL

  // Compute the number of readings per demand window.  This is computed here (as opposed to locally in the
  // Calc_Demand() subroutine) because Dmnd_NumSubInt_Per_DmndWin and Dmnd_Num200ms_Per_SubInt may be
  // changed during a time change, and this number should remain constant
  Dmnd_Num200ms_Per_DmndWin = Dmnd_NumSubInt_Per_DmndWin * Dmnd_Num200ms_Per_SubInt;

  // Initialize the sub-interval sums.  The entire array must be initialized to zero, because if a sliding
  // window is used, the sum is taken over the entire demand window (not the number of entries).
  for (i=0; i<HISTORY_BUF_SIZE; ++i)
  {
    Dmnd_SubTots.Ia[i] = 0;
    Dmnd_SubTots.Ib[i] = 0;
    Dmnd_SubTots.Ic[i] = 0;
    Dmnd_SubTots.In[i] = 0;
    Dmnd_SubTots.TotW[i] = 0;
    Dmnd_SubTots.TotVar[i] = 0;
    Dmnd_SubTots.TotVA[i] = 0;
    Dmnd_SubTots.NumVals[i] = 0;
  }

  Dmnd_ForceLog = FALSE;                // Clear flag to force demand log

  Dmnd_Delay = 5;

  EngyDmnd[1].DmndIa  = NAN;
  IDmnd.IaMin = NAN;
  IDmnd.IaMax = NAN;
  EngyDmnd[1].DmndIb  = NAN;
  IDmnd.IbMin = NAN;
  IDmnd.IbMax = NAN;
  EngyDmnd[1].DmndIc  = NAN;
  IDmnd.IcMin = NAN;
  IDmnd.IcMax = NAN;
  EngyDmnd[1].DmndIn  = NAN;
  IDmnd.InMin = NAN;
  IDmnd.InMax = NAN;
  EngyDmnd[1].DmndTotW  = NAN;
  PDmnd.TotWMax = NAN;
  EngyDmnd[1].DmndTotVar  = NAN;
  PDmnd.TotVarMax = NAN;
  EngyDmnd[1].DmndTotVA  = NAN;
  PDmnd.TotVAMax = NAN;

  resptr = &Res5min_Avg.Ia;
  for (i=0; i<NUM5MINVALS; ++i)
  {
    *resptr++ = 0;
  }
  resptr = &Res5min_MinMax.Iamax;
  for (i=0; i<NUM5MINVALS; ++i)
  {
    *resptr++ = 0;
    *resptr++ = 1E36;
  }

  Delay5 = 5;

  IDmnd.ResetTS.Time_secs = 0x234;              // *** DAH  TEST VALUES FOR NOW - THIS HAS TO BE STORED AND RETRIEVED IN FRAM!!
  IDmnd.ResetTS.Time_nsec = 0x567;
  PDmnd.ResetTS.Time_secs = 0x89A;
  PDmnd.ResetTS.Time_nsec = 0xBCD;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Dmnd_VarInit()
//------------------------------------------------------------------------------------------------------------





//----------------------------------------------------------------------------------------------------------
//                      Demand Calculation Architecture
//----------------------------------------------------------------------------------------------------------
//
// Demands are calculated for current and power.  The demand and logging windows are common to all demand
// value sets.  Each demand value has an associated min and max (peak) value function associated with the
// demand calculation.  Execute action commands are provided to 1) reset the min and Peak Demand, and
// 2) force a demand computation.  A simple diagram of the basic architecture for the demand structure is as
// follows:
//
//  200msec anniversary ---+
//                         |
//  +----------------------+-------------------------------------------------------------------------------+
//  |                                                                                                      |
//  | Major Time Change                                                                                    |
//  | Sync Command ------> Force Log --+--->                                                               |
//  | Dmnd Setpnts Change              |                                                                   |
//  |                                  |                                                                   |
//  |                          +-------+------+    Next Logging Anniversary Time -----------+              |
//  |    Internal time ------->| Synchronizer |--> Next Demand Window Time       --+        |              |
//  |                          +-------+------+    Next Sub-Interval Time        --+        |              |
//  |                                  |                                           |        |              |
//  | Normal Log ----------------------+                                           |        |              |
//  |                                                                              |        |              |
//  |             Force Log ------+      +-----------------------------------------+        |              |
//  |                             |      |                                                  |              |
//  |                             |      |                                    +-------------+              |
//  |                          +--+------+--+                                 |                            |
//  |    Internal Time         |            |                             +---+----+                       |
//  |      ------------------->|            |                             |        |                       |
//  |                          |            |          Energies---------->|        |----> Normal Log       |
//  |                          |            |                             | Logger |                       |
//  |    Real-Time Data        |            |          Internal time ---->|        |                       |
//  |      ------------------->|            |                             |        |                       |
//  |        200msec Currents  |            |  Demands                    |        |                       |
//  |        200msec Voltages  |            |---------------+------------>|        |                       |
//  |        200msec Powers    |            |               |             |        |                       |
//  |                          |   Demand   |               |             +----+---+                       |
//  |                          | Calculator |               |                  |                           |
//  |                          |            |               |     Force Log ---+                           |
//  |                          |            |               |                                              |
//  |    Setpoints             |            |               |                                              |
//  |      ------------------->|            |               |             +------------+                   |
//  |        Mode              |            |               |             |            |                   |
//  |        Window Type       |            |               +------------>|    Peak    |                   |
//  |        Demand Interval   |            |                             |   Demand   |---> Peak Demands  |
//  |        Logging Interval  |            |          Internal time ---->| Calculator |                   |
//  |                          |            |                             |            |                   |
//  |                          +------------+                             +-----+------+                   |
//  |                                                                           |                          |
//  |                                                  Reset Peak Demand -------+                          |
//  |                                                                                                      |
//  +------------------------------------------------------------------------------------------------------+



//----------------------------------------------------------------------------------------------------------
//                      Demand Calculation
//----------------------------------------------------------------------------------------------------------
//          
// The Demand Calculation function takes real-time rms data generated every 200msec and produces demand
// averages.  The demand window characteristics (mode, type, demand interval, logging interval) are
// determined by setpoints.
// Note, energy values are logged at the same time as demand values.  However, the logged energy values are
// not "computed" or averaged.  They are merely a snapshot of the present energy register value at the time
// of the logging anniversary.
//
//              Demand Modes
// There are two modes of operation: internal time-based and external time-based.
//
// In external time-based mode, demand logging in the PXR35 is under control of one or more master devices.
// Demand values are computed when a "synchronize" command is received over one of the communications ports
// (CAM1, CAM2, Modbus, USB,, etc.).  Demands are computed, and demand values and energies are then logged
// at that time.  The Demand Window Type is always fixed - sliding windows are not allowed.  Synchronize
// commands must be received at least once every 60 minutes.  If a synchronize command is not received in
// this time period, the values are logged automatically.  Time changes do not affect the logging operation
// in this mode.  Note, the demand and logging intervals are determined by the external master device, and
// may change dynamically.  The PXR35 merely computes and logs the demand values under command of the
// master.  In the diagram above, a synchronize command just sets the Force Log flag.
//
// In internal time-based mode, the PXR35 uses its internal timer to determine when to compute demand
// values, and when to log the values.  The Demand Window Type (fixed or sliding), Demand Interval, and
// Logging Time Interval are all configurable via setpoints.  Demands are synchronized from a Master Device
// whenever the Master Device sets the time.  "Synchronize Demand" commands are ignored.
//
//              Demand Window Time and Logging Interval Time
// The Demand Window Time applies to Demand Logs only.  It does not apply to Energy Logs.  The Demand Window
// defines the interval over which demands are computed, and when the demand history buffer rolls over.  In
// instances where the data is accumulated over an interval that is less than the Demand Window, the data is
// averaged over the defined Demand Window (NOT the actual interval time).  In instances where the data is
// accumulated over an interval that is slightly longer than the Demand Window, the data is averaged over
// the actual interval time (NOT the defined Demand Window).  If the actual demand interval time is found to
// be significantly greater than the defined Demand Window (an error condition), the reading is dropped.
// The Logging Interval Time defines when Log Entries are made.  This time applies to both Demand and Energy
// Logs.  All Log entries are made at the same time.  In other words, the Time Stamp will be the same for
// all 12 logged values (3 powers, 4 currents, 5 energies).  The Logging Interval Time is independent of the
// Demand Window Time.
// Valid demand window times are 5-60 minutes in 1-minute increments.  Valid logging window times are 5, 10,
// 15, 20, 30, and 60 minutes.  Logging anniversaries are always synchronized to the top of the hour.
// Demand readings are always synchronized to the clock minute (that is, they always occur at seconds = 0).
// If the demand window is 5, 10, 15, 20, 30, or 60 minutes, it is also synchronized with the top of the
// hour.  It is NOT (and cannot be) synchronized to the top of the hour for any other demand window time.
//
// +-------------------------------------------------------------------------------------------------------+
// |                                          Setpoints                                                    |
// +------------------+----------------+-------------------------------------------------------------------+
// | Mode             |    External    |                            Internal                               |
// | Type             | Fixed Only     |         Fixed                              Sliding                |
// | Demand Interval  | Not applicable | 5 - 60 min, 1 min increments       5 - 60 min, 1 min increments   |
// | Logging Interval | Not applicable | 5, 10, 15, 20, 30, 60 min          5, 10, 15, 20, 30, 60 min      |
// +------------------+----------------+-------------------------------------------------------------------+
//
//              Demand Calculations
// A sub-interval is used in the demand calculation.  The sub-interval length depends on the length of the
// demand window (Internal Mode).  It is 15 seconds for demand windows of 15 minutes or less; 60 seconds
// otherwise.  The sub-interval is always 60 seconds when External Mode is used.
// The sub-interval is used as a multi-cycle current sum.  200msec currents are summed over the period of
// the sub-interval.  Each sub-interval anniversary, these "sub-interval" current sums are placed into an
// array, along with the number of values read in the sub-interval.  The arrays are the history of the
// sub-interval currents over the demand window.
// As mentioned previously, sub-interval anniversaries occur every 15 seconds or every 60 seconds.
// Sub-interval anniversaries are synchronized to the top of the minute.  In other words, for 15-second
// sub-interval times, they occur at the 0, 15, 30, and 45 second points.  For 60-second sub-interval times,
// they occur at the 0-second point.
//
// Regardless of Mode and Type, the demand is computed by taking the average of the sub-interval currents
// over the length of the demand window (the sum of the sub-interval sums divided by the sum of the number
// of values in each sub-interval).
//
//This computation is performed whenever a demand anniversary is reached:
//   - In External Mode, whenever the Sync command is received (the Force Log flag is set)
//   - In Internal Mode, Fixed Window, the computation is performed every Demand Anniversary, or every 5,
//     10, 15, 20, 30, or 60 minutes, synchronized to the top of the hour:
//          Demand Window        Number of sub-intervals (typ)      Length of sub-interval
//            5 minutes                    20                             15 seconds
//           10 minutes                    40                             15 seconds
//           15 minutes                    60                             15 seconds
//           20 minutes                    20                             60 seconds
//           30 minutes                    30                             60 seconds
//           60 minutes                    60                             60 seconds
//   - In Internal Mode, Sliding Window, the computation is performed each sub-interval anniversary
// If the demand interval reaches 60 minutes, a demand computation (and log) is forced, because the buffer
// is filled at that point.
//
//              Demand Window Anniversaries and Logging Anniversaries
// The demand window anniversary is the time over which the demand is computed.  For fixed demand windows,
// this time also defines the rate at which new demand values are generated.  For sliding demand windows,
// new demand values are generated every sub-interval time (15 seconds or 60 seconds).  The demand window
// time defines the "history" of the rolling buffer.  It is the duration for which a sub-interval demand
// reading remains in the buffer.
// The logging anniversary time is merely the time when the present demand and energy values are placed into
// the demand logs (the ~45-day logs).
// Consider the following examples:
//   1) Demand window anniversary = 15 minutes, logging anniversary = 15 minutes
//      For fixed demand windows, new demand values will be computed and stored in the demand logs every 15
//      minutes.
//      For sliding windows, new demand values will be computed every 15 seconds.  However, the demand
//      values will be logged every 15 minutes.  The logged values are the same regardless of whether a
//      fixed or sliding window is used, because at the 15-minute interval, the two buffers are identical.
//   2) Demand window anniversary = 5 minutes, logging anniversary = 15 minutes
//      For fixed demand windows, new demand values will be computed every 5 minutes.  Demand values will be
//      logged every 15 minutes.  The demand values that are logged are taken over a 5-minute window.
//      Essentially, two demand values are skipped for logging purposes.
//      For sliding windows, new demand values will be computed every 15 seconds, and the demand values are
//      logged every 15 minutes.  The history is 5 minutes.  Similar to example 1), the logged values will
//      be identical to the logged values for the fixed demand window.  This is true whenever the logging
//      anniversary is greater than or equal to the demand window anniversary.
//
//              Time Intervals Versus Real Time (Synchronizer Function)
// This section applies to Internal Mode only.  The following are requirements for the demand function:
//   - Demand computations are synchronized to the top of the hour, and then are performed every demand
//     interval.  In other words, the minutes of the Time Stamp should be a multiple of the demand interval,
//     and the seconds of the time stamp should be approximately zero.
//   - For demand anniversaries equal to 5, 10, 15, 20, 30, or 60 minutes, logging anniversaries should be
//     synchronized with demand anniversaries.  As shown in the previous examples, the two intervals may not
//     be identical.  However, in all cases, both logging anniversaries and demand anniversaries should be
//     synchronized to the sub-interval anniversary.
//   - Measurement Canada Bulletin E-34: "Policy on demand measurement established using demand interval
//     length other than the programmed demand interval length" is used to establish the rules for computing
//     demands over demand intervals that are not the equal to the programmed demand window length.  This
//     could occur after a reset, after a change to the demand setpoints, or after a time change.  The
//     following describes the two acceptable behaviors:
//         1) the meter discards any demand measurement that is taken using a demand window that is shorter
//            or longer than the programmed interval
//         2) if the demand measurement is taken over an interval that is less than the programmed demand
//            window, the calculation is done using the programmed demand interval (NOT the actual demand
//            window)
//     The bulletin does not provide any tolerances for the intervals, so a tolerance of +/-1sec (five
//     200 msec reading times) was chosen.  This results in the following algorithm:
//         (Actual number of readings) < (ideal number of readings - 5):
//             Demand = (sum of readings)/(ideal number of readings), mark partial
//         (ideal number of readings - 5) <= (Actual number of readings) <= (ideal number of readings):
//             Demand = (sum of readings)/(ideal number of readings), mark normal          *** DAH  DO WE WANT TO DIVIDE BY ACTUAL NUMBER OF READINGS SINCE MORE ACCURATE AND WITHIN THE TOLERANCE BAND?
//         (ideal number of readings) < (Actual number of readings) <= (ideal number of readings + 5):
//             Demand = (sum of readings)/(actual number of readings), mark normal
//         (ideal number of readings + 5) < (Actual number of readings):
//             Discard
//     Basically, this means that when calculating the demand, the divisor is either the ideal number of
//     readings for the demand window, or the actual number of readings, whichever is greater.  If the
//     window is more than one second too short, the value is flagged as partial.  If the demand window is
//     greater than one second too long, it is discarded (should not happen in normal operation).
// Time changes are handled in ProcessTimeAdjustment().  This subroutine examines the magnitude of the
// change, and forces a demand window and logging anniversary (the Dmnd_ForceLog flag is set) if the
// magnitude of the change is more than one second.  The EID and time stamp of the forced logging event are
// taken care of in this subroutine.  However, any time a demand calculation is performed, the number of
// readings is compared to the ideal number.  This is used to determine the handling of the result based on
// the algorithm described directly above.  This guarantees the integrity of the demand computations.
// 
// The sub-interval, demand window, and logging anniversary times are all synchronized to the internal
// timer.  The demand subroutine is invoked every 200msec, when new readings are available.
//
// 


 

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_Demand()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Demand
//
//  MECHANICS:          Demand computations enabled and logging enabled:
//                        This subroutine computes the demand currents and powers, and enters an event into
//                        the FIFO if a demand logging anniversary has been reached.
//                        Demand currents sre computed for fixed or sliding windows as described above.
//                        Note that there are basically four anniversaries that apply to the demand
//                        algorithm:
//                          1) 200msec anniversary: This is when the demand subroutine is invoked, and it
//                             aligns with new 200msec metered values.  Every 200msec anniversary, the new
//                             values are added to the present sub-interval sum, and the 200msec sample
//                             counter is incremented.
//                          2) sub-interval anniversary: The subroutine checks for a sub-interval
//                             anniversary each time it is entered (every 200msec).  A sub-interval
//                             anniversary occurs every 15sec for demand windows that are 15 minutes or
//                             less, and every 60sec for demand windows that are longer than 15 minutes.
//                             When a sub-interval anniversary is reached, the demand currents over the
//                             sub-interval are computed and stored in a rolling buffer (Dmnd_SubTots).  A
//                             counter is incremented and checked to determine whether the logging
//                             anniversary has been reached.
//                          3) demand window anniversary: The subroutine checks for a demand window
//                             anniversary every sub-interval anniversary.  For fixed demand windows, a
//                             demand window anniversary occurs whenever the demand time is reached (every
//                             5..60 minutes.  It is NOT necessarily the same as the logging time).  For
//                             sliding windows, a demand window anniversary occurs every sub-interval
//                             anniversary.  When a demand window anniversary is reached, the demand
//                             currents (over the demand interval time) are computed and stored, along with
//                             a time-stamp.  The demand currents are merely the average of the sub-interval
//                             currents over the demand window.
//                          4) logging anniversary: The subroutine checks for a logging anniversary every
//                             demand window annversary.  For fixed windows, this occurs at the end of the
//                             demand window.  For sliding windows, this occurs every sub-interval
//                             anniversary.  A logging anniversary occurs whenever the logging time is
//                             reached (every 5, 10, 15, 20, 30, or 60 minutes).  When a logging anniversary
//                             is reached, a LOG_ENERGY event is placed into the New Events FIFO.  This will
//                             result in a "snapshot" of the present energy registers and demand values (the
//                             values in struct EngyDmnd) being placed into the energy and demand log in
//                             Flash memory.
//                        Note, the 200msec anniversaries are NOT aligned with the top of the hour, so
//                        actual demand computation and logging entries may be delayed by as much as
//                        200msec.
//                        The subroutine checks for demand window and logging anniversaries only at the end
//                        of the sub-interval anniversary.  This ensures that the anniversaries remain
//                        aligned.
//                        The target times for the three anniversaries are computed in the same subroutine,
//                        using the same real-time snapshot.
//
//                      Demand computations enabled and logging disabled:
//                        Same as above except we do not check for a logging anniversary
//
//                      Demand computations disabled and logging enabled:
//                        We check for a logging anniversary each time (not every demand window
//                        anniversary).  When a logging anniversary is reached, a LOG_ENERGY event is placed
//                        into the New Events FIFO.  This will result in a "snapshot" of the present energy
//                        registers and demand values (the values in struct EngyDmnd) being placed into the
//                        energy and demand log in Flash memory.
//
//                      Demand computations disabled and logging disabled:
//                        No action
//
//  CAVEATS:            This subroutine must be called each 200msec anniversary AFTER the 200msec currents
//                      (Cur200msFltr.Ix) have been updated
//
//  INPUTS:             Cur200msFltr.Ix, Pwr200msec.xx, Pwr200msecApp.xx, Dmnd_ForceLog, EventMasterEID,
//                      presenttime.Time_secs, Dmnd_NumSubInt_Per_DmndWin, Dmnd_Num200ms_Per_DmndWin,
//                      Dmnd_Setp_DemandInterval, Dmnd_Setp_DemandLogInterval
// 
//  OUTPUTS:            NewEventFIFO[].x, NewEventInNdx, EngyDmnd[1].Dmndxxx, IDmnd.xxx, PDmnd.xxx
//
//  ALTERS:             Dmnd_SubTots.Ix[], Dmnd_SubTots.NumVals[], Dmnd_SubIntCnt, Dmnd_200msecCnt,
//                      Dmnd_WinSum.Ix, Dmnd_ForceLog, Next_SI_Time, Next_DW_Time, Next_LA_Time,
//                      Dmnd_SaveEvent.xx, EventMasterEID
// 
//  CALLS:              __disable_irq(), __enable_irq(), isnan(), Get_InternalTime(), FRAM_Write(),
//                      CalcNextAnniversaries(), FRAM_WriteMinMax()
//
//  EXECUTION TIME:     About 40usec max when there is a demand window anniversary.  Note, this is with
//                      interrupts enabled (need the sampling interrupt enabled so we get 200msec
//                      anniversaries), so actual subroutine time may be less.      *** DAH NEED TO REMEASURE SINCE MIN AND MAX DEMANDS ADDED
// 
//------------------------------------------------------------------------------------------------------------

void Calc_Demand(void)
{
  enum DEMAND_COMPUTATION_STATUS
  {
    DMND_NORMAL, DMND_PARTIAL, DMND_ERROR
  };

  struct INTERNAL_TIME presenttime;
  uint32_t next_dw_temp, next_la_temp;
  uint32_t temp1[2];
  float ftemp, ftemp1;
  uint16_t total_readings_in_dmndwin, diff;
  int16_t temp;
  uint8_t eow, i, dmndwin_status, state, cd_exit;

  // After a reset (Dmnd_Delay initialized to 5), delay about 1 second before starting the demand
  // computations to compensate for the delay through the digital filter on the metered currents
  // On the transition to 0 (one time only), calculate the sub-interval, demand window, and logging
  // anniversaries
  if (Dmnd_Delay > 0)
  {
    --Dmnd_Delay;
    if (Dmnd_Delay == 0)
    {
      CalcNextAnniversaries(SysTickTime.cnt_sec, &Next_SI_Time, &Next_DW_Time, &Next_LA_Time);
    }
    return;
  }

  // This subroutine uses a state machine to enable different portions of the code to be executed based on
  //   the setpoints.
  //
  //   Setpoints0.stp.   Setpoints0.stp.    Demand     Demand    Energy     (Energy is always computed)
  //   DemandInterval   DemandLogInterval  Computed?  Logging?  Logging?
  //        >0                >0              Yes       Yes       Yes
  //         0                >0              No        Yes*      Yes       * - values logged but are NaNs
  //        >0                 0              Yes       No        No
  //         0                 0              No        No        No
  //
  // The state machine always starts from 0 and completes before the the subroutine is exited.  As mentioned
  //   previously, the state machine just provides a mechanism for jumping to different portions of code.

  state = 0;
  cd_exit = FALSE;

  while (!cd_exit)
  {
    switch (state)
    {
      case 0:                           // State 0: Check whether we are computing demands
        if (Dmnd_Setp_DemandInterval == 0)      // If not computing demands...
        {
          if (Dmnd_Setp_DemandLogInterval > 0)      // If logging, jump to state 2
          {
            state = 2;
          }
          else                                      // If not logging, we're not doing anything, so exit
          {
            cd_exit = TRUE;
          }
          break;
        }
        else                                    // If computing demands, fall into state 1
        {
          state = 1;
        }
//        break;                                Don't need break here

      case 1:                           // State 1: Compute demands
        //-------------------- 200msec Reading Processing --------------------------------------------------
        //
        Dmnd_SubTots.Ia[Dmnd_SubIntCnt] += Cur200msFltr.Ia;      // Update the sub-interval sums and
        Dmnd_SubTots.Ib[Dmnd_SubIntCnt] += Cur200msFltr.Ib;      //   increment the 200msec sample counter
        Dmnd_SubTots.Ic[Dmnd_SubIntCnt] += Cur200msFltr.Ic;
        Dmnd_SubTots.In[Dmnd_SubIntCnt] += Cur200msFltr.In;
        Dmnd_SubTots.TotW[Dmnd_SubIntCnt] += Pwr200msec.Ptot;
        Dmnd_SubTots.TotVar[Dmnd_SubIntCnt] += Pwr200msec.Rtot;
        Dmnd_SubTots.TotVA[Dmnd_SubIntCnt] += Pwr200msecApp.Apptot;
        ++Dmnd_200msecCnt;

        // Call InsertNewEvent() with no event to check for events that occurred during interrupts.  We need
        //   to do this here to ensure these events are in the queue ahead of the Demand/Energy event (if
        //   one is entered).
        InsertNewEvent(NO_EVENT);

        __disable_irq();                                // Get the present time in case we need to enter an
        Get_InternalTime(&presenttime);                 //   event and to check for anniversaries
         __enable_irq();

        // Perform sub-interval anniversary processing if it is a sub-interval or a forced logging
        //   anniversary
        if (Dmnd_ForceLog || (presenttime.Time_secs >= Next_SI_Time))
        {                                               // Intentionally fall into the next section
        }
        // Perform an error check on the number of sub-intervals in  case time somehow got corrupted
        // Dmnd_200msecCnt should never significantly exceed 300 (there are 300 readings in a 1-minute
        //   sub-interval).  If it does, set the force log flag so that the anniversaries will resync with
        //   internal time
        else if (Dmnd_200msecCnt > 600)
        {
          Dmnd_ForceLog = TRUE;
          Dmnd_SaveEvent.TS.Time_secs = presenttime.Time_secs;    // Save the present time for the force log
          Dmnd_SaveEvent.TS.Time_nsec = presenttime.Time_nsec;    //   event time stamp
          Dmnd_SaveEvent.EID = EventMasterEID++;                  // Generate a new EID
          temp1[0] = EventMasterEID;
          temp1[1] = ~EventMasterEID;
          FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&temp1[0]));
          FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&temp1[0]));
        }
        else                                            // Othewise no sub-interval anniversary or forced
        {                                               //   log, so exit
          cd_exit = TRUE;
          break;
        }

        //-------------------- Sub-Interval Anniversary Processing -----------------------------------------
        //
        // If we are using a sliding window, it is a demand window anniversary at every sub-interval
        //   anniversary
        if (Dmnd_Setp_DemandWindow != 0)
        {
          eow = TRUE;
        }
        else                                            // Clear "end-of-window" flag if it is not a sliding
        {                                               //   window
          eow = FALSE;
        }

        // Save the number of 200msec readings taken in the sub-interval (should normally be 75 or 300) and
        // increment the sub-interval counter, Dmnd_SubIntCnt.  This variable also serves as the index into
        // the demand history arrays.  Note, the sub-interval sums are already in the buffer
        Dmnd_SubTots.NumVals[Dmnd_SubIntCnt++] = Dmnd_200msecCnt;

        Dmnd_200msecCnt = 0;                            // Reset the 200msec sample counter

        // Call the subroutine to compute the next sub-interval, demand, and logging anniversaries, based on
        // the present time.  We can store the next sub-interval time in its normal variable, since this
        // value needs to be updated.  The other two will be saved in temporaries.  They are not be updated
        // until their anniversaries have passed.
        CalcNextAnniversaries(presenttime.Time_secs, &Next_SI_Time, &next_dw_temp, &next_la_temp);

        // Check that there will not be an overflow of the history buffer when the next 200msec reading
        //   occurs (this should never happen)
        // An overflow will occur if Dmnd_SubIntCnt has reached HISTORY_BUF_SIZE AND we're not going to do a
        //   demand log.  In normal operation Dmnd_SubIntCnt may equal HISTORY_BUF_SIZE when we're ready to
        //   enter a log.
        if ( (Dmnd_SubIntCnt >= HISTORY_BUF_SIZE)               // If buffer count is full,
          && (!eow) && (presenttime.Time_secs < Next_DW_Time)   //   and we're not logging,
          && (!Dmnd_ForceLog) )                                 //   and there's no forced-log condition,
        {                                                       //   force a demand log
          Dmnd_ForceLog = TRUE;                                 //     Set the flag
          Dmnd_SaveEvent.TS.Time_secs = presenttime.Time_secs;  //     Save the present time for the
          Dmnd_SaveEvent.TS.Time_nsec = presenttime.Time_nsec;  //       force-log event time stamp
          Dmnd_SaveEvent.EID = EventMasterEID++;                //     Generate a new EID
          temp1[0] = EventMasterEID;
          temp1[1] = ~EventMasterEID;
          FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&temp1[0]));
          FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&temp1[0]));
        }

        // Note: This section is only reached if there has been a sub-interval anniversary
        // Perform demand window anniversary processing if it is a demand window or a forced logging
        //   anniversary
        // A demand window anniversary occurs if:
        //   for a sliding window, it is a sub-interval anniversary (eow = TRUE)
        //   for a fixed window, the present time has reached the next demand window anniversary time
        //     (presenttime.Time_secs >= Next_DW_Time)
        if (Dmnd_ForceLog || eow || (presenttime.Time_secs >= Next_DW_Time))
        {

          //-------------------- Demand Window Anniversary Processing --------------------------------------
          //
          dmndwin_status = DMND_NORMAL;                 // Initialize demand computation status to normal

          // Compare the total number of 200msec readings taken in this demand window versus the ideal
          //   number.  The total number is the sum of the number of readings in each sub-interval.  The
          //   ideal number is determined from the setpoints, and is:
          //     the number of 200msec readings/sub-interval x the number of sub-intervals/demand window
          // Canadian bulletin E-34: "Policy on demand measurement established using demand interval length
          //   other than the programmed demand interval length" was used to determine how to handle
          //   computing the demand when time adjustments are made.  The bulletin states that demand
          //   registration is acceptable provided that either:
          //   a) the meter discards any demand measurement for an interval that is shorter or longer than
          //      the programmed interval, or
          //   b) if energy is accumulated over an interval that is less than the programmed interval, the
          //      demand is calculated using the programmed interval
          // The bulletin does not provide any tolerances for the intervals, so I opted to use +/-1sec (five
          // 200msec reading times).  This results in the following algorithm for handling demands:
          //   (Actual number of readings) < (ideal number of readings - 4):
          //        Demand = (sum of readings)/(ideal number of readings), mark partial
          //   (ideal number of readings - 5) <= (Actual number of readings) <= (ideal number of readings):
          //        Demand = (sum of readings)/(ideal number of readings), mark normal
          //   (ideal number of readings) < (Actual number of readings) <= (ideal number of readings + 5):
          //        Demand = (sum of readings)/(actual number of readings), mark normal
          //   (ideal number of readings + 5) < (Actual number of readings):
          //        Discard

          // Compute the total number of readings (of 200msec values) taken during the demand window
          total_readings_in_dmndwin = 0;
          if (Dmnd_Setp_DemandWindow != 0)              // For a sliding window, the number of readings is
          {                                             //   over the demand window
            for (i=0; i<Dmnd_NumSubInt_Per_DmndWin; ++i)        
            {
              total_readings_in_dmndwin += Dmnd_SubTots.NumVals[i];
            }
          }
          else
          {
            for (i=0; i<Dmnd_SubIntCnt; ++i)        
            {
              total_readings_in_dmndwin += Dmnd_SubTots.NumVals[i];
            }
          }

          // If there are more readings than expected ...
          if (total_readings_in_dmndwin > Dmnd_Num200ms_Per_DmndWin)
          {
            diff = total_readings_in_dmndwin - Dmnd_Num200ms_Per_DmndWin;    // Compute the difference
            // If more than one second's worth of readings, error - this should never happen.  Discard the
            //   demand reading and don't log anything.
            // If this is the first time the error was detected (EngyDmnd[1].Status != DMND_ERROR),
            //   increment the Demand EID since there will be a break in the log on the next entry, and set
            //   EngyDmnd[1].Status equal to DMND_ERROR to indicate that an error has been detected.  This
            //   way, if it is a sliding window, the EID won't keep getting incremented for the number of
            //   sub-intervals in the sliding window.  When the number of readings is good (diff <= 5, and
            //   dmndwin_status != DMND_ERROR), EngyDmnd[1].Status will be overwritten with either
            //   DMND_NORMAL or DMND_PARTIAL
            if (diff > 5)                                         
            {
              dmndwin_status = DMND_ERROR;
              if (EngyDmnd[1].Status != DMND_ERROR)
              {
                EngyDmnd[1].Status = DMND_ERROR;
                EngyDmnd[1].EID = EventMasterEID++;
                temp1[0] = EventMasterEID;
                temp1[1] = ~EventMasterEID;
                FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&temp1[0]));
                FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&temp1[0]));
              }
            }
          }
          // Otherwise there are less than or the same number of readings as expected...
          else                                                          
          {
            // Set diff to 0.  This is used as a flag to indicate that the ideal number of readings must be
            //   used to compute the demand
            diff = 0;
            // If less than one second's worth of readings, set status to Partial
            if ((Dmnd_Num200ms_Per_DmndWin - total_readings_in_dmndwin) > 5)
            {
              dmndwin_status = DMND_PARTIAL;
            }
          }

          if (dmndwin_status != DMND_ERROR)             // If the demand window time is valid (normal or
          {                                             //   partial, just not too long), compute the demand
            // First compute the sum of all of the sub-interval currents and powers.  Note, this is done by
            //   summing the values in the array, rather than subtracting the oldest sub-interval demand
            //   from a running total and then adding in the newest value.  Testing showed that error can
            //   creep in due to the inherent imprecision of floating point numbers.
            // Also note that the computations are done in blocks of four.  This is done because the ARM
            //   processor is optimized to do floating point operations in groups of four (I think the
            //   operations can be pipelined).  This saves significant time at the expense of a little bit
            //   of code space.  With Dmnd_SubIntCnt = 60, the time was 22.18usec to complete the for-loop
            //   with a "normal" implementation, with just the currents (powers weren't added yet), and
            //   17.74usec with the "4x" implementation.  The difference would likely be greater with the
            //   other values added to the loop.
            // The number of sub-intervals, Dmnd_SubIntCnt, is usually a multiple of four, except for when
            //   Dmnd_NumSubInt_Per_DmndWin = 30, Dmnd_NumSubInt_Per_DmndWin = 60, and when there is a major
            //   time adjustment.
//TESTPIN_D1_HIGH;
            Dmnd_WinSum.Ia = 0;                         // Initialize the demand window sums
            Dmnd_WinSum.Ib = 0;
            Dmnd_WinSum.Ic = 0;
            Dmnd_WinSum.In = 0;
            Dmnd_WinSum.TotW = 0;
            Dmnd_WinSum.TotVar = 0;
            Dmnd_WinSum.TotVA = 0;
            // For fixed windows, the demand window is taken from Dmnd_SubTots.Ix[0] to
            //   Dmnd_SubTots.Ix[Dmnd_SubIntCnt - 1].  It is not necessarily taken over 0 to
            //   Dmnd_NumSubInt_Per_DmndWin, because if there is a forced logging event, the number is
            //   smaller.
            // For sliding windows, the demand is taken from Dmnd_SubTots.Ix[0] to
            //   Dmnd_SubTots.Ix[Dmnd_NumSubInt_Per_DmndWin - 1].  Here, the entire demand window is always
            //   used.
            // The array SubTots[] array is initialized to zero on power up to ensure the readings are valid
            //   while the initial demand window fills up.
            // temp = greatest multiple of 4 that is less than or equal to the number of sub-intervals
            temp = ( (Dmnd_Setp_DemandWindow == 0) ?
                        (Dmnd_SubIntCnt & 0xFC) : (Dmnd_NumSubInt_Per_DmndWin & 0xFC) );
            // The code is written this way to maximize speed
            for (i=0; i<temp; i+=4)                     // Compute the sum up to this number
            {
              Dmnd_WinSum.Ia += Dmnd_SubTots.Ia[i];
              Dmnd_WinSum.Ia += Dmnd_SubTots.Ia[i+1];
              Dmnd_WinSum.Ia += Dmnd_SubTots.Ia[i+2];
              Dmnd_WinSum.Ia += Dmnd_SubTots.Ia[i+3];
              Dmnd_WinSum.Ib += Dmnd_SubTots.Ib[i];
              Dmnd_WinSum.Ib += Dmnd_SubTots.Ib[i+1];
              Dmnd_WinSum.Ib += Dmnd_SubTots.Ib[i+2];
              Dmnd_WinSum.Ib += Dmnd_SubTots.Ib[i+3];
              Dmnd_WinSum.Ic += Dmnd_SubTots.Ic[i];
              Dmnd_WinSum.Ic += Dmnd_SubTots.Ic[i+1];
              Dmnd_WinSum.Ic += Dmnd_SubTots.Ic[i+2];
              Dmnd_WinSum.Ic += Dmnd_SubTots.Ic[i+3];
              Dmnd_WinSum.In += Dmnd_SubTots.In[i];
              Dmnd_WinSum.In += Dmnd_SubTots.In[i+1];
              Dmnd_WinSum.In += Dmnd_SubTots.In[i+2];
              Dmnd_WinSum.In += Dmnd_SubTots.In[i+3];
              Dmnd_WinSum.TotW += Dmnd_SubTots.TotW[i];
              Dmnd_WinSum.TotW += Dmnd_SubTots.TotW[i+1];
              Dmnd_WinSum.TotW += Dmnd_SubTots.TotW[i+2];
              Dmnd_WinSum.TotW += Dmnd_SubTots.TotW[i+3];
              Dmnd_WinSum.TotVar += Dmnd_SubTots.TotVar[i];
              Dmnd_WinSum.TotVar += Dmnd_SubTots.TotVar[i+1];
              Dmnd_WinSum.TotVar += Dmnd_SubTots.TotVar[i+2];
              Dmnd_WinSum.TotVar += Dmnd_SubTots.TotVar[i+3];
              Dmnd_WinSum.TotVA += Dmnd_SubTots.TotVA[i];
              Dmnd_WinSum.TotVA += Dmnd_SubTots.TotVA[i+1];
              Dmnd_WinSum.TotVA += Dmnd_SubTots.TotVA[i+2];
              Dmnd_WinSum.TotVA += Dmnd_SubTots.TotVA[i+3];
            }
            while (i<Dmnd_NumSubInt_Per_DmndWin)        // Pick up the remaining sub-intervals (up to 3)
            {
              Dmnd_WinSum.Ia += Dmnd_SubTots.Ia[i];
              Dmnd_WinSum.Ib += Dmnd_SubTots.Ib[i];
              Dmnd_WinSum.Ic += Dmnd_SubTots.Ic[i];
              Dmnd_WinSum.In += Dmnd_SubTots.In[i];
              Dmnd_WinSum.TotW += Dmnd_SubTots.TotW[i];
              Dmnd_WinSum.TotVar += Dmnd_SubTots.TotVar[i];
              Dmnd_WinSum.TotVA += Dmnd_SubTots.TotVA[i];
              ++i;
            }

            // If the difference is 0, the actual window is less than or equal to the expected window, so
            //   use the expected (ideal) window length
            if (diff == 0)
            {
              EngyDmnd[1].DmndIa = Dmnd_WinSum.Ia/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndIb = Dmnd_WinSum.Ib/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndIc = Dmnd_WinSum.Ic/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndIn = Dmnd_WinSum.In/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndTotW = Dmnd_WinSum.TotW/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndTotVar = Dmnd_WinSum.TotVar/(float)Dmnd_Num200ms_Per_DmndWin;
              EngyDmnd[1].DmndTotVA = Dmnd_WinSum.TotVA/(float)Dmnd_Num200ms_Per_DmndWin;
            }
            // Otherwise the actual window is greater than the expected window, so use the actual window
            //   length
            else
            {
              EngyDmnd[1].DmndIa = Dmnd_WinSum.Ia/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndIb = Dmnd_WinSum.Ib/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndIc = Dmnd_WinSum.Ic/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndIn = Dmnd_WinSum.In/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndTotW = Dmnd_WinSum.TotW/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndTotVar = Dmnd_WinSum.TotVar/(float)total_readings_in_dmndwin;
              EngyDmnd[1].DmndTotVA = Dmnd_WinSum.TotVA/(float)total_readings_in_dmndwin;
            }

            EngyDmnd[1].Status = dmndwin_status;        // Save dmnd computation status (normal or partial)

            // Update min and max demands.  Note, we don't use an "else if" on the max calculation because
            //   after a reset, the first demand will be both a min and a max.  Also note that for the power
            //   demands, we need to take the absolute value of both the new demand and the present min and
            //   max demands, because these may be positive or negative.
            i = FALSE;                  // i is used as a temporary flag to indicate whether there is a new
                                        //   min or max
            if ( (isnan(IDmnd.IaMin)) || (EngyDmnd[1].DmndIa < IDmnd.IaMin) )
            {
              IDmnd.IaMin = EngyDmnd[1].DmndIa;
              IDmnd.IaMinTS.Time_secs = presenttime.Time_secs;
              IDmnd.IaMinTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.IaMax)) || (EngyDmnd[1].DmndIa > IDmnd.IaMax) )
            {
              IDmnd.IaMax = EngyDmnd[1].DmndIa;
              IDmnd.IaMaxTS.Time_secs = presenttime.Time_secs;
              IDmnd.IaMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.IbMin)) || (EngyDmnd[1].DmndIb < IDmnd.IbMin) )
            {
              IDmnd.IbMin = EngyDmnd[1].DmndIb;
              IDmnd.IbMinTS.Time_secs = presenttime.Time_secs;
              IDmnd.IbMinTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.IbMax)) || (EngyDmnd[1].DmndIb > IDmnd.IbMax) )
            {
              IDmnd.IbMax = EngyDmnd[1].DmndIb;
              IDmnd.IbMaxTS.Time_secs = presenttime.Time_secs;
              IDmnd.IbMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.IcMin)) || (EngyDmnd[1].DmndIc < IDmnd.IcMin) )
            {
              IDmnd.IcMin = EngyDmnd[1].DmndIc;
              IDmnd.IcMinTS.Time_secs = presenttime.Time_secs;
              IDmnd.IcMinTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.IcMax)) || (EngyDmnd[1].DmndIc > IDmnd.IcMax) )
            {
              IDmnd.IcMax = EngyDmnd[1].DmndIc;
              IDmnd.IcMaxTS.Time_secs = presenttime.Time_secs;
              IDmnd.IcMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.InMin)) || (EngyDmnd[1].DmndIn < IDmnd.InMin) )
            {
              IDmnd.InMin = EngyDmnd[1].DmndIn;
              IDmnd.InMinTS.Time_secs = presenttime.Time_secs;
              IDmnd.InMinTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if ( (isnan(IDmnd.InMax)) || (EngyDmnd[1].DmndIn > IDmnd.InMax) )
            {
              IDmnd.InMax = EngyDmnd[1].DmndIn;
              IDmnd.InMaxTS.Time_secs = presenttime.Time_secs;
              IDmnd.InMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if (i)                              // If there is a new min or max demand current, save it in
            {                                   //   FRAM
              FRAM_WriteMinMax(MNMX_DMNDI_ADD, ((sizeof(struct DEMAND_I_MIN_MAX_STRUCT))/2),
                                                    (uint16_t *)(&IDmnd.IaMin));
            }
            i = FALSE;                  // i is used as a temp flag to indicate whether there is a new max
            ftemp1 = ( (EngyDmnd[1].DmndTotW < 0) ?
                            (-1.0 * EngyDmnd[1].DmndTotW) : (EngyDmnd[1].DmndTotW) );
            ftemp = ( (PDmnd.TotWMax < 0) ? (-1.0 * PDmnd.TotWMax) : (PDmnd.TotWMax) );
            if ( (isnan(PDmnd.TotWMax)) || (ftemp1 > ftemp) )
            {
              PDmnd.TotWMax = EngyDmnd[1].DmndTotW;
              PDmnd.TotWMaxTS.Time_secs = presenttime.Time_secs;
              PDmnd.TotWMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            ftemp1 = ((EngyDmnd[1].DmndTotVar < 0) ?
                            (-1.0 * EngyDmnd[1].DmndTotVar) : (EngyDmnd[1].DmndTotVar));
            ftemp = ( (PDmnd.TotVarMax < 0) ? (-1.0 * PDmnd.TotVarMax) : (PDmnd.TotVarMax) );
            if ( (isnan(PDmnd.TotVarMax)) || (ftemp1 > ftemp) )
            {
              PDmnd.TotVarMax = EngyDmnd[1].DmndTotVar;
              PDmnd.TotVarMaxTS.Time_secs = presenttime.Time_secs;
              PDmnd.TotVarMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            ftemp1 = ( (EngyDmnd[1].DmndTotVA < 0) ?
                        (-1.0 * EngyDmnd[1].DmndTotVA) : (EngyDmnd[1].DmndTotVA) );
            ftemp = ( (PDmnd.TotVAMax < 0) ? (-1.0 * PDmnd.TotVAMax) : (PDmnd.TotVAMax) );
            if ( (isnan(PDmnd.TotVAMax)) || (ftemp1 > ftemp) )
            {
              PDmnd.TotVAMax = EngyDmnd[1].DmndTotVA;
              PDmnd.TotVAMaxTS.Time_secs = presenttime.Time_secs;
              PDmnd.TotVAMaxTS.Time_nsec = presenttime.Time_nsec;
              i = TRUE;
            }
            if (i)                              // If there is a new max demand power, save it in FRAM
            {
              FRAM_WriteMinMax(MAX_DMNDP_ADD, ((sizeof(struct DEMAND_P_MAX_STRUCT))/2),
                                    (uint16_t *)(&PDmnd.TotWMax));
            }
          }
    
          // Clear the sub-interval counter if:
          //   1) we are using a fixed demand window (eow = FALSE), because with a fixed window, we always
          //      restart the new demand window from 0
          //   2) the sub-interval counter has reached the length of the demand window
          //      (Dmnd_SubIntCnt >= Dmnd_NumSubInt_Per_DmndWin).  This applies to sliding windows only (the
          //      counter is reset for fixed windows regardless).  This means the index in the history
          //      buffer has rolled over.  Note that the index (Dmnd_SubIntCnt) is compared to the number of
          //      sub-intervals in the demand window, NOT the length of the buffer (HISTORY_BUF_SIZE).  The
          //      sliding window history must only extend to the demand window length, NOT the maximum
          //      length.
          if (!eow || (Dmnd_SubIntCnt >= Dmnd_NumSubInt_Per_DmndWin))
          {
            Dmnd_SubIntCnt = 0;
          }
          
          Next_DW_Time = next_dw_temp;                    // Update the next demand window anniversary time

          //
          //-------------------- End Demand Window Anniversary Processing ----------------------------------
        }
  
        // Regardless of whether there was a demand window anniversary, initialize the next set of
        //   sub-interval sums (there was a sub-interval anniversary)
        Dmnd_SubTots.Ia[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.Ib[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.Ic[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.In[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.TotW[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.TotVar[Dmnd_SubIntCnt] = 0;
        Dmnd_SubTots.TotVA[Dmnd_SubIntCnt] = 0;

        // If we are logging, we will jump to state 3.  We have already called CalcNextAnniversaries() to
        //   compute the next sub-interval, demand, and logging anniversaries.  We don't need to call it
        //   again.
        // If we are not logging, we are done
        if (Dmnd_Setp_DemandLogInterval > 0)
        {
          state = 3;
        }
        else
        {
          cd_exit = TRUE;
        }
        break;

      case 2:                           // State 2: Update next logging anniversary
        __disable_irq();                                // Get the present time in case we need to enter an
        Get_InternalTime(&presenttime);                 //   event and to check for anniversaries
         __enable_irq();
        // Call the subroutine to compute the next sub-interval, demand, and logging anniversaries, based on
        // the present time.  We only care about the next logging anniversary, since this state is only
        // reached when demands are not being computed
        CalcNextAnniversaries(presenttime.Time_secs, &Next_SI_Time, &next_dw_temp, &next_la_temp);
        state = 3;                                      // Fall into the next state to do logging
//      break;                                          Don't need break here

      case 3:                           // State 3: Logging
        //-------------------- Logging Anniversary Processing ----------------------------------------------------
        //
        // If demand computations are enabled, this section is only entered if there has been a sub-interval
        //   anniversary.  If demand computations are disabled, it is entered each time the subroutine is
        //   invoked.
        //
        // Perform logging anniversary processing if it is a normal or forced logging anniversary
        //   A normal logging anniversary occurs if the time has reached the next logging anniversary time
        //   (Next_LA_Time).  A forced anniversary occurs when the associated flag (Dmnd_ForceLog) is set.
        //   This is usually due to a major time change or a change in the demand setpoints.  The processing
        //   is different for each occurrence.
        // First check for a forced log event - this is a higher priority
        if (Dmnd_ForceLog)                              // If there is a forced logging anniversary..
        {
          EngyDmnd[1].TS.Time_secs = Dmnd_SaveEvent.TS.Time_secs;     // Insert an event into the queue with
          EngyDmnd[1].TS.Time_nsec = Dmnd_SaveEvent.TS.Time_nsec;     //   the saved EID and time stamp
          EngyDmnd[1].EID = Dmnd_SaveEvent.EID;
          NewEventFIFO[NewEventInNdx++].Code = DEMAND_EVENT;
          NewEventInNdx &= 0x0F;                                // *** DAH This relies on NEWEVENTFIFO[] SIZE = 16!

          Next_LA_Time = next_la_temp;                                // Update next logging anniversary time
          Dmnd_ForceLog = FALSE;                                      // Clear the force log flag
        }
        // Otherwise if there is a "normal" logging anniversary...
        else if (presenttime.Time_secs >= Next_LA_Time)
        {
          // If the latest demand calculation is valid, insert a logging event into the new event FIFO
          // Note, we only need to place the event into the FIFO.  We don't need the EID.  It is already in
          //   the buffer.
          if (dmndwin_status != DMND_ERROR)
          {
            NewEventFIFO[NewEventInNdx++].Code = DEMAND_EVENT;
            NewEventInNdx &= 0x0F;                              // This relies on NEWEVENTFIFO[] SIZE = 16!
            EngyDmnd[1].TS.Time_secs = presenttime.Time_secs;
            EngyDmnd[1].TS.Time_nsec = presenttime.Time_nsec;
          }
          Next_LA_Time = next_la_temp;                          // Update next logging anniversary time
        }
        // Otherwise there is no logging anniversary, so we're done

        cd_exit = TRUE;
        break;

        //
        //-------------------- End Logging Anniversary Processing ------------------------------------------------

    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_Demand()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CalcNextAnniversaries()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Next Demand Anniversary Times
//
//  MECHANICS:          This subroutine computes the next sub-interval, demand window, and logging
//                      anniversary times based on the (presumably) present time in seconds (parameter
//                      time_secs).
//                      Using a single subroutine to compute these values ensures that they are aligned with
//                      each other.
//                      The subroutine works by assuming time_secs is the number of seconds past midnight of
//                      January 1, 2000.  All of the seconds are stripped off this count until the count is
//                      the number of seconds in the minutes of the hour of the present time.  This is used
//                      to compute the next demand iwndow and logging anniversary times.  The seconds
//                      comprising the minutes are then stripped off so that the count is then the number of
//                      seconds in the minute of the present time.  This is used to compute the next
//                      sub-interval time.
//
//  CAVEATS:            None
//
//  INPUTS:             time_secs - the present time in seconds past January 1, 2000
//                      Dmnd_Setp_DemandInterval - the demand window in minutes
//                      Dmnd_Setp_DemandLogInterval - the demand logging window in minutes
// 
//  OUTPUTS:            *next_si - next sub-interval anniversary time
//                      *next_dw - next demand window anniversary time
//                      *next_la - next logging anniversary time
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void CalcNextAnniversaries(uint32_t time_secs, uint32_t *next_si, uint32_t *next_dw, uint32_t *next_la)
{
  uint32_t secs;
  uint16_t interval;
  uint8_t i, secs_in_mins, interval1;

  // Eliminate all seconds down to the remaining minutes of the present time
  secs = time_secs % SECS_IN_FOUR_YRS;      // Get rid of seconds comprising the year portion of the time
  secs = (secs % SECS_IN_ONE_YR);
  for (i=0; i<12; ++i)                      // Get rid of seconds comprising the month portion of the time
  {
    if (secs > SECS_IN_MONTH[i])
    {
      secs -= SECS_IN_MONTH[i];
    }
    else
    {
      break;
    }
  }
  secs = (secs % SECS_IN_ONE_DAY);          // Get rid of seconds comprising the date portion of the time
  secs = (secs % SECS_IN_ONE_HR);           // Get rid of seconds comprising the hour portion of the time

  // At this point, secs has the number of seconds in the present minutes and seconds portion of the time.
  // Keep this to get the demand window and logging anniversary times.  Use a temporary to compute the
  // sub-interval anniversary time.

  secs_in_mins = (uint8_t)(secs % 60);      // Get rid of seconds comprising the minutes portion of the time
  // The next sub-interval time is then:
  //     time_secs                                   (present time)
  //   + (interval1 - (secs_in_mins % interval1))    (time to the next interval time)
  // Compute the sub-interval time.  If the demand window is 15 minutes or less, the sub-interval time is
  // 15 seconds.  If the window is greater than 15 minutes, the sub-interval time is 60sec.
  interval1 = ((Dmnd_Setp_DemandInterval <= 15) ? 15 : 60);
  *next_si = time_secs + (interval1 - (secs_in_mins % interval1));

  // Similar computation for the next demand window anniversary time using the minutes and seconds time
  interval = Dmnd_Setp_DemandInterval * 60;                    // Demand window in seconds
  *next_dw = time_secs + (interval - (uint16_t)(secs % interval));

  // Similar computation for the next logging anniversary time using the minutes and seconds time
  interval = Dmnd_Setp_DemandLogInterval * 60;                 // Demand log interval in seconds
  *next_la = time_secs + (interval - (uint16_t)(secs % interval));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         CalcNextAnniversaries()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Calc_5minAverages()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate 5-Minute Average Values
//
//  MECHANICS:          This subroutine computes the 5-minute average values.  The subroutine operates as
//                      follows:
//                      There are basically two anniversaries that apply to the algorithm:
//                          1) 200msec anniversary: This is when the demand subroutine is invoked, and it
//                             aligns with new 200msec metered values.  Every 200msec anniversary, the new
//                             values are added to the present 5-minute sum, and the 200msec counter is
//                             incremented.
//                          2) 5-minute anniversary: The subroutine checks for a 5-minute anniversary every
//                             200msec anniversary.  When a 5-minute anniversary is reached, the 5-minute
//                             average values are computed, and the min, max, and average values are stored.
//                      The min and max values are based on one-cycle values and so are captured in the
//                      corresponding one-cycle metering subroutine.  After they are captured, the "running"
//                      variables are reset to NAN so new min/max values are captured in the next reading.
//                      Note, the 200msec anniversaries are NOT aligned with the top of the hour, so actual
//                      demand computation and logging entries may be delayed by as much as 200msec.
//
//  CAVEATS:            This subroutine must be called each 200msec anniversary AFTER the 200msec values
//                      have been updated
//
//  INPUTS:             Delay5, SysTickTime.cnt_sec, Next_5minTime, Sum5min_Avg.xx, Sum5min_MinMax.xx,
//                      Cur200msFltr.Ix, VolAFE200msFltr.Vxx, VolAFE200msFltrVlnavg, VolAFE200msFltrVllavg,
//                      presenttime.Time_secs, SumCnt
// 
//  OUTPUTS:            Res5min_Avg.xxx, Res5min_MinMax.xxx
//
//  ALTERS:             Next_5minTime, Sum5min_Avg.xx, Sum5min_MinMax.xx, SumCnt, Delay5
// 
//  CALLS:              __disable_irq(), __enable_irq(), Get_InternalTime(), CalcNext5minAnniversary()
//
//  EXECUTION TIME:     Measured on 220304 (rev 0.51 code): 327usec with 14 values supported.  Note, this is
//                      with interrupts disabled around the subroutine call.
// 
//------------------------------------------------------------------------------------------------------------

void Calc_5minAverages(void)
{
  struct INTERNAL_TIME presenttime;
  float ftemp, ftemp1;
  float *resptr, *sumptr;
  float * const *input_val_ptr;
  uint8_t i;

  // After a reset (Dmnd_Delay initialized to 5), delay about 1 second before starting the demand
  // computations to compensate for the delay through the digital filter on the metered currents
  // On the transition to 0 (one time only), calculate the 5-minute anniversaries, and initialize the sums
  // and min/max values
  if (Delay5 > 0)
  {
    --Delay5;
    if (Delay5 == 0)
    {
      CalcNext5minAnniversary(SysTickTime.cnt_sec, &Next_5minTime);
      sumptr = &Sum5min_Avg.Ia;
      for (i=0; i<NUM5MINVALS; ++i)
      {
        *sumptr++ = 0;
      }
      sumptr = &Sum5min_MinMax.Iamax;
      for (i=0; i<NUM5MINVALS; ++i)
      {
        *sumptr++ = 0;
        *sumptr++ = 1E36;
      }
      SumCnt = 0;
    }
    return;
  }
  // *** DAH TEST CODE
/*  Cur200msFltr.Ia = 300.0/10;
  Cur200msFltr.Ib = 301.0/10;
  Cur200msFltr.Ic = 302.0/10;
  Cur200msIavg = 303.0/10;
  Pwr200msec.Pa = 400.0;
  Pwr200msec.Pb = -401.0;
  Pwr200msec.Pc = 402.0;
  Pwr200msec.Ptot = -403.0;
  Pwr200msecApp.AppPa = 500.0;
  Pwr200msecApp.AppPb = 501.0;
  Pwr200msecApp.AppPc = 502.0;
  Pwr200msecApp.Apptot = 503.0; */
  // *** DAH TEST CODE


  //------------------------------------ 200msec Anniversary Processing ------------------------------------
  //
  // Update the 5-minute sums and min/max valus with the 200msec readings
  sumptr = &Sum5min_Avg.Ia;
  input_val_ptr = &FIVEMIN_INPUT_VALS[0];
  for (i=0; i<NUM5MINVALS; ++i)
  {
    *sumptr = (*sumptr + **input_val_ptr);
    ++sumptr;
    ++input_val_ptr;
  }

  // Update the 5-minute min/max values with the 200msec readings
  sumptr = &Sum5min_MinMax.Iamax;
  input_val_ptr = &FIVEMIN_INPUT_VALS[0];
  for (i=0; i<NUM5MINVALS; ++i)
  {
    // Get absolute values (only necessary for real powers but easier to do for all values)
    ftemp = ((**input_val_ptr < 0) ? (-1.0 * (**input_val_ptr)) : **input_val_ptr);  // Input

    ftemp1 = ((*sumptr < 0) ? (-1.0 * (*sumptr)) : *sumptr);                        // Max
    if (ftemp > ftemp1)                                       // Set max to input if abs value of input is
    {                                                         //   greater than abs value of the existing
      *sumptr = **input_val_ptr;                              //   max value
    }

    ++sumptr;                                                                       // Min
    ftemp1 = ((*sumptr < 0) ? (-1.0 * (*sumptr)) : *sumptr);
    if (ftemp < ftemp1)                                       // Set min to input if abs value of input is
    {                                                         //   less than abs value of the existing min
      *sumptr = **input_val_ptr;                              //   value
    }

    ++sumptr;
    ++input_val_ptr;
  }
    
  ++SumCnt;                               // Increment the counter

  __disable_irq();                                      // Get the present time
  Get_InternalTime(&presenttime);
  __enable_irq();

  //----------------------------------- 5-minute Anniversary Processing ------------------------------------
  //
  // If we reached the 5-minute anniversary or ten minutes has elapsed (which means the clock is constantly
  //   being adjusted - this is considered an anomaly), generate new readings
  if ( (presenttime.Time_secs >= Next_5minTime) || (SumCnt > 3000) )
  {
    ftemp = (float)SumCnt;
    resptr = &Res5min_Avg.Ia;
    sumptr = &Sum5min_Avg.Ia;
    for (i=0; i<NUM5MINVALS; ++i)
    {
      *resptr = *sumptr/ftemp;
      *sumptr = 0;
      ++sumptr;
      ++resptr;
    }

    resptr = &Res5min_MinMax.Iamax;
    sumptr = &Sum5min_MinMax.Iamax;
    for (i=0; i<NUM5MINVALS; ++i)
    {
      *resptr = *sumptr;
      *sumptr = 0;
      ++sumptr;
      ++resptr;
      *resptr = *sumptr;
      *sumptr = 1E36;
      ++sumptr;
      ++resptr;
    }

    SumCnt = 0;
  }

  // Recompute the next anniversary time in case there was a time change.  This is not as precise as
  //   handling it in the time-change occurrence (as is done for demands), but is much simpler
  CalcNext5minAnniversary(presenttime.Time_secs, &Next_5minTime);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Calc_5minAverages()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       CalcNext5minAnniversary()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Calculate Next 5-Minute Anniversary Time
//
//  MECHANICS:          This subroutine computes the next 5-minute anniversary time (in seconds) based on
//                      the (presumably) present time in seconds (parameter time_secs).
//                      The subroutine works by assuming time_secs is the number of seconds past midnight of
//                      January 1, 2000.  All of the seconds are stripped off this count until the count is
//                      the number of seconds in the minutes of the hour of the present time.  This is used
//                      to compute the next 5-minute anniversay time.
//
//  CAVEATS:            None
//
//  INPUTS:             time_secs - the present time in seconds past January 1, 2000
// 
//  OUTPUTS:            *next_5min - next 5-minute anniversary time
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

void CalcNext5minAnniversary(uint32_t time_secs, uint32_t *next_5min)
{
  uint32_t secs;
  uint8_t i;

  // Eliminate all seconds down to the remaining minutes of the present time
  secs = time_secs % SECS_IN_FOUR_YRS;      // Get rid of seconds comprising the year portion of the time
  secs = (secs % SECS_IN_ONE_YR);
  for (i=0; i<12; ++i)                      // Get rid of seconds comprising the month portion of the time
  {
    if (secs > SECS_IN_MONTH[i])
    {
      secs -= SECS_IN_MONTH[i];
    }
    else
    {
      break;
    }
  }
  secs = (secs % SECS_IN_ONE_DAY);          // Get rid of seconds comprising the date portion of the time
  secs = (secs % SECS_IN_ONE_HR);           // Get rid of seconds comprising the hour portion of the time

  // At this point, secs has the number of seconds in the present minutes and seconds portion of the time.
  //   secs = present number of seconds past the top of the hour (0 - 3599)
  //   secs % 300 = number of seconds past the last 5-minute point (0 - 299)
  //   300 - (secs % 300) = number of seconds until the next 5-minute point
  //   time_secs + (300 - (secs % 300)) = the next 5-minute anniversary point
  *next_5min = time_secs + (300 - (uint16_t)(secs % 300));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         CalcNext5minAnniversary()
//------------------------------------------------------------------------------------------------------------

