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
//  MODULE NAME:        RealTime.c
//
//  MECHANICS:          Program module containing the RTC and Internal Time subroutines
//
//  TARGET HARDWARE:    PXR35 Rev 4 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.42   201202  DAH - File Creation.  Code moved from Iod.c
//                      - Added Check_IntRTC(), IntRTC_Write(), and IntRTC_Read()to support the internal RTC
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
//   0.51   220304  DAH - Added InternalTimeToMBTime(), SecondstoTime()
//                      - Replace code in InternalTimeRead() with call to SecondstoTime()
//   0.58   220829  DAH - Revisions to move RTC initialization from the initialization code to the main loop
//                          - Added RTC_InitState
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
#include "Init_def.h"



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
#include "Init_ext.h"



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void RT_VarInit(void);
void Get_InternalTime(struct INTERNAL_TIME *ts_ptr);
void RTC_to_SysTickTime(struct SYSTICK_TIME *systck_ptr);
void InternalTimeRead(void);
uint8_t Check_IntRTC(void);
uint8_t IntRTC_Read(void);
void IntRTC_Update(void);
uint64_t InternalTimeToMBTime(struct INTERNAL_TIME *inttime_ptr);



//      Local Function Prototypes (These functions are called only within this module)
//
void IntRTC_Write(void);
void SecondstoTime(uint32_t Time_secs, uint16_t *days, uint8_t *minu, uint8_t *hour, uint8_t *year,
                                            uint8_t *month, uint8_t *dayofweek);



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
uint8_t RTC_buf[12];
uint8_t IntTimeBuf[9];
uint8_t RTC_State;
uint8_t RTC_InitState;
struct SYSTICK_TIME SysTickTime;



//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t RTC_ShiftVal;



//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//
// Time computations
// Array of day offsets by month (Jan=0) for non-leap years
const uint16_t Day_Offset_Reg[12] =     // Accumulated days to month for regular year
{
      0,                                //    Jan (31)
     31,                                //    Feb (28)
     59,                                //    Mar (31)
     90,                                //    Apr (30)
    120,                                //    May (31)
    151,                                //    Jun (30)
    181,                                //    Jly (31)
    212,                                //    Aug (31)
    243,                                //    Sep (30)
    273,                                //    Oct (31)
    304,                                //    Nov (30)
    334                                 //    Dec (31)
};

// Array of day offsets by month (Jan=0) for leap years
const uint16_t Day_Offset_Leap[12] =    // Accumulated days to month for leap year
{
      0,                                //    Jan (31)
     31,                                //    Feb (29)
     60,                                //    Mar (31)
     91,                                //    Apr (30)
    121,                                //    May (31)
    152,                                //    Jun (30)
    182,                                //    Jly (31)
    213,                                //    Aug (31)
    244,                                //    Sep (30)
    274,                                //    Oct (31)
    305,                                //    Nov (30)
    335                                 //    Dec (31)
};

// Array of day offsets in remaining 4-year period (2000...2003,  2004...2007, etc)
const uint16_t Days_In_Period[4] =
{
     0,                                 //    No additional year in period
   366,                                 //    2000, 2004, 2008... is a leap year
   731,                                 //    366 + 365
  1096                                  //    366 + 365 + 365
};




//
//-------------------------------- Internal Real Time Description --------------------------------------------
//
// Overview
// UTC time (Coordinated Universal Time) and IEEE1588 are the standards used in the design of the algorithms
// used to handle real time.  IEEE 1588 defines two components:
//      Seconds past January 1, 1970
//      Nanoseconds past the last second
//
// SysTick Time
// The PXR35 maintains time using the SysTick timer.  The timer is driven by a 15MHz clock (SysClk is
// 120MHz, and it is divided by eight).  The timer is configured to interrupt every 10msec.  Each SysTick
// tick, and therefore, the internal time resolution, is 66.666667nsec.
// Internal SysTick time is kept in structure SysTickTime.  SysTickTime consists of three components:
//     uint32_t cnt_sec     Seconds past January 1, 2000
//     uint32_t cnt_10msec  10msec counts past the last second, resets at one second (100)
//     SysTick->VAL indicates the number of 66.666667nsec timer ticks past the last 10msec tick.  This may
//       be used to provide a timestamp with ~67nsec resolution.  Note, SysTick->VAL is a downcounter, so
//       the number of ticks past the last 10msec tick is actually the initial value (COUNT_120MHZ_10MSEC)
//       minus SysTick->VAL.
//
// The maximum value for cnt_sec is 2^32 = 4.29E9.  This means it will roll over in 2136.  We can change the
// start date to a later year if desired, but this should be ok.
// Basically, SysTick Time is a free-running counter that serves as the system timebase.  A SysTick
// interrupt occurs every 10msec, during which cnt_10msec and possibly cnt_sec are incremented.  Internal
// Time is derived by reading cnt_10msec, cnt_sec, and SysTick->VAL.
//
// Internal Time
// Internal time is obtained from SysTick Time.  Internal time consists of two components:
//     uint32_t Time_secs   Seconds past January 1, 2000
//     uint32_t Time_nsec   Nanoseconds past the last second
// The internal time structure is very similar to the SysTick Time structure.  SysTick Time uses a 10msec
// counter instead of a nsec counter to simplify the interrupt.
//
// SysTick Time to Internal Time
// There are four steps to reading the time (to obtain a timestamp):
//   1) SysTickTime.cnt_sec contains the seconds (this is straightforward)
//   2) SysTickTime.cnt_msec contains the 10msec counts past the last second
//   3) if there is a pending SysTick interrupt, increment cnt_msec (and possibly cnt_sec) because 10msec
//      has elapsed
//   4) SysTick->VAL indicates the number of 66.666667nsec timer ticks past the last 10msec interrupt
// Internal time is then:
//      InternalTime.Time_secs = SysTickTime.cnt_sec
//      InternalTime.Time_nsec = SysTickTime.cnt_msec * 1E7 + (COUNT_120MHZ_10MSEC - SysTick->VAL) * 66.66
//          If (InternalTime.Time_nsec > 1E9)
//              InternalTime.Time_nsec -= 1E9
//              ++InternalTime.Time_secs
// This is the basic algorithm.  In practice, interrupts are disabled when retrieving internal time.  This
// must be done because the PXR35 has numberous interrupts and we will not get a precise and accurate
// timestamp otherwise.  The code accounts for possible pending SysTick interrupts when the time is
// retrieved.
// The floating-point representation of a single timer tick is 66.66666412353515625.  Therefore, each tick
// a loss of (66.666666667 - 66.666664122) or 2.545fsec (2.545E-15 seconds) occurs.
// The maximum count can be 99, so the maximum error is (round to 100) 254.5fsec.  As a percentage, this is
// 254.5fsec/66.666666667nsec x 100 = .000382% so we should not need to compensate when adding the time from
// SysTick->VAL
//
// RTC  
// The PXR35 contains an internal battery-backed RTC.  This is only used to obtain time on power-up.  The
// RTC is read every 5 minutes.  The RTC time is comnpared to the internal time and the RTC is adjusted if
// the times are not within 500msec of each other.
// The RTC is not intended to be extremely accurate or precise.  It serves merely as a backup timer when
// powering up.  The internal time is intended to be continuously adjusted using PTP (Precision Time
// Protocol) to maintain 0.5msec accuracy.
//
// System
// If PTP is in use, the intent is for a new PTP message to be received every 5 seconds, and Internal Time
// will therefore be updated every 5 seconds.  The micro crystal has 55ppm total error over temperature, so
// if time is updated every 5 seconds, it will only lose 275usec.  That should keep us within the 1msec
// accuracy requirement.
// The present RTC crystal has about 170ppm total error (10ppm accuracy, but ~160ppm temperature drift).
// Worst-case, it will be off about 10msec for every minute that power is off.  Remember, when the unit is
// operating, the RTC will be continuously synced to Internal Time within 50msec.
//  
//------------------------------------------------------------------------------------------------------------



//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        RT_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Real Time Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Real Time Module.
//                      The following variables do not need initialization:
//                          RTC_ShiftVal
//
//  CAVEATS:            Call only during initialization
//
//  INPUTS:             None
//
//  OUTPUTS:            RTC_State, RTC_InitState
//
//  ALTERS:             None
//
//  CALLS:              ReadStartupScaleConstant()
//
//  EXECUTION TIME:     Measured on 160625 (rev 0.25 code): 56.1usec    *** DAH  CAN REMEASURE SINCE SOME CODE REMOVED (SHOULD BE FASTER)
//
//------------------------------------------------------------------------------------------------------------

void RT_VarInit(void)
{
  RTC_State = 0;
  RTC_InitState = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          RT_VarInit()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Get_InternalTime()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Get internal time
//
//  MECHANICS:          This subroutine writes the current time into the INTERNAL_TIME structure pointed to
//                      by ts_ptr:
//                          ts_ptr->Time_secs: seconds past January 1, 2000
//                          ts_ptr->Time_nsec: nanoseconds past the last second
//                      The current time is contained in SysTickTime.cnt_sec and SysTickTime.cnt_10msec,
//                      along with residual time in the SysTick timer (SysTick->VAL).
//
//                      SysTickTime.cnt_sec contains the seconds from midnight between 12/31/1999 and
//                      1/1/2000
//                      SysTickTime.cnt_10msec contains the 10msec counts after the last second (0 - 99)
//                      SysTick->VAL contains the 66.67nsec count after the last 10msec count (0 - 149999)
//
//                      The 10ms SysTick timer has a resolution of 8/120MHz = 66.66667nsec when operating
//                      under normal operating conditions (SYSCLK = 120MHz).  It has a resolution of 500nsec
//                      on power up (SYSCLK = HSI = 16MHz).
//                      We must recover the offset from the last 10msec clock tick by reading the SysTick
//                      value register.
//
//                      If the last SysTick interrupt has not yet been processed
//                      (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk == 1), 10msec must be added to the time stamp.
//
//  CAVEATS:            Interrupts must be locked-out prior to calling this routine.  If called from the 
//                      foreground, call as follows:
//                          __disable_irq();
//                          Get_InternalTime(&timestamp);
//                          __enable_irq();
//
//  INPUTS:             ts_ptr, SysTickTime, SysClk_120MHz
// 
//  OUTPUTS:            Captured time in structure pointed to by ts_ptr
//
//  ALTERS:             None 
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 230901: 775nsec
//
//------------------------------------------------------------------------------------------------------------

void Get_InternalTime(struct INTERNAL_TIME *ts_ptr)
{
  float ftemp;
  uint32_t temp;

  // Read the present SysTick value.  This value will be used if there is not a pending SysTick interrupt.
  // This way, if we are close to a SysTick interrupt, but haven't reached it yet, the large value in the
  // SysTick->VAL register will be used.
  temp = SysTick->VAL;
                                       
  // Compute the total nanoseconds that have elapsed.  First check whether there is a pending SysTick
  // interrupt.
  // If there is a pending interrupt, SysTick->VAL has rolled over at some point.  In this case, we will add
  // 10msec to the nanosecond count to compensate for the rollover.  However, we need to recapture the
  // SysTick->VAL register to be certain it is AFTER it has rolled over.  We don't want it to be the value
  // right before it rolled over because it will then be artifically high (close to 10msec too high).
  // If there is not a pending interrupt, SysTick->VAL has not rolled over.  In this case, we want to use
  // the SysTick->VAL register value captured BEFORE we checked for the rollover, in case it rolls over
  // after we checked.  This way, we are certain it is not artifically low.
  // Note, since SysTick is a downcounter, we must subtract it from its initial autoload value to get the
  // elapsed time.  Perform a validity check and convert it to nanoseconds in floating point format

  if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) == SCB_ICSR_PENDSTSET_Msk)    // VAL has rolled over...
  {
    temp = SysTick->VAL;
    // Check to ensure that the AutoReload value has been placed into the counter.  Testing showed that
    //   sometimes the value is still zero (counter runs at 1/8 the micro speed) and the autoreload hasn't
    //   occurred yet.  If this is the case, temp should be the autoreload value and ftemp, the elapsed nsec
    //   since the last 10msec tick, will be zero
    if (temp == 0)
    {
      ftemp = 0;
    }
    // Otherwise autoreload value is ok, so use it to compute the nsec since the last 10msec tick.
    else
    {
      // *** DAH  DO WE NEED TO SUPPORT 16MHZ OPERATION????
      if (SysClk_120MHz == TRUE)                                              // Operating at 120MHz...
      {
        ftemp = (temp < COUNT_120MHZ_10MSEC) ? ((float)(COUNT_120MHZ_10MSEC - temp) * 66.6666667f) : 0.0f;
      }
      else                                                                    // Operating at 16MHz...
      {
        ftemp = (temp < COUNT_16MHZ_10MSEC) ? ((float)(COUNT_16MHZ_10MSEC - temp) * 500.00f) : 0.0f;
      }
    }
    // Now add in the elapsed 10msec time since the last second (in nsec) plus one 10msec time since we
    //   rolled over
    ftemp += ((float)SysTickTime.cnt_10msec) * 1.0E7f + 1.0E7f;
  }
  else                                                                   // VAL has not rolled over...
  {
    if (SysClk_120MHz == TRUE)                                              // Operating at 120MHz...
    {
      ftemp = (temp < COUNT_120MHZ_10MSEC) ? ((float)(COUNT_120MHZ_10MSEC - temp) * 66.6666667f) : 0.0f;
    }
    else                                                                    // Operating at 16MHz...
    {
      ftemp = (temp < COUNT_16MHZ_10MSEC) ? ((float)(COUNT_16MHZ_10MSEC - temp) * 500.00f) : 0.0f;
    }
    ftemp += ((float)SysTickTime.cnt_10msec) * 1.0E7f;
  }
  if (ftemp >= 1.0E9f)
  {
    ftemp -= 1.0E9f;
    ts_ptr->Time_secs = SysTickTime.cnt_sec + 1;
  }
  else
  {
    ts_ptr->Time_secs = SysTickTime.cnt_sec;
  }
  ts_ptr->Time_nsec = (uint32_t)(ftemp + 0.5f);

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Get_InternalTime()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       RTC_to_SysTickTime()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Convert RTC time into the SysTick time format
//
//  MECHANICS:          This subroutine converts the RTC time into the SysTick time format and stores it in
//                      the location pointed to by *systck_ptr.  SysTick time consists of the following two
//                      parameters:
//                      SysTickTime.cnt_sec contains the seconds from midnight between 12/31/1999 and
//                      1/1/2000
//                      SysTickTime.cnt_10msec contains the 10msec counts after the last second (0 - 99)
//                      RTC time is stored as follows:
//                          RTC_buf[0]: hundredths of a second in hexadecimal (0 - 99)
//                          RTC_buf[1]: seconds in hexadecimal (0 - 59)
//                          RTC_buf[2]: minutes in hexadecimal (0 - 59)
//                          RTC_buf[3]: hours in hexadecimal (0 - 23)
//                          RTC_buf[5]: date in hexadecimal (1 - 31)
//                          RTC_buf[6]: month in hexadecimal (1 - 12)
//                          RTC_buf[7]: year in hexadecimal (0 - 99)
//
//  CAVEATS:            Note: The RTC is not expected to be extremely precise.  Therefore, we do not need
//                      to adjust for the time spent converting the RTC time to the internal time format
//                      and writing it into the buffers.
//
//  INPUTS:             RTC_buf[]
// 
//  OUTPUTS:            *systck_ptr
//
//  ALTERS:             None 
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 180626 (rev 0.25 code): 4.5usec
//
//------------------------------------------------------------------------------------------------------------

void RTC_to_SysTickTime(struct SYSTICK_TIME *systck_ptr)
{
  uint32_t tmp;

  // RTC time is valid
  // Time is as follows:
  //   RTC_buf[0]: hundredths of a second in hexadecimal (0 - 99)
  //   RTC_buf[1]: seconds in hexadecimal (0 - 59)
  //   RTC_buf[2]: minutes in hexadecimal (0 - 59)
  //   RTC_buf[3]: hours in hexadecimal (0 - 23)
  //   RTC_buf[4]: day in hexadecimal - not used here
  //   RTC_buf[5]: date in hexadecimal (1 - 31)
  //   RTC_buf[6]: month in hexadecimal (1 - 12)
  //   RTC_buf[7]: year in hexadecimal (0 - 99)

  systck_ptr->cnt_10msec = RTC_buf[0];      // Update 10msec counter with hundredths

  // Update seconds counter with the present seconds, minutes, and hours
  systck_ptr->cnt_sec = RTC_buf[1];
  systck_ptr->cnt_sec += (RTC_buf[2] * 60);
  systck_ptr->cnt_sec += (RTC_buf[3] * 3600);

                                       // Compute days from midnight between 12/31/1999 & 1/1/2000
  tmp = (RTC_buf[7] >> 2) * 1461;         // Load Date w/ days in 4-year intervals since 2000 (365 * 3 + 366)
  tmp += Days_In_Period[RTC_buf[7] % 4];  // Add days in remaining 4-year period
                                          //    Add days in months from January 1
  tmp += ((RTC_buf[7] % 4) == 0) ? Day_Offset_Leap[RTC_buf[6] - 1] : Day_Offset_Reg[RTC_buf[6] - 1];
                                          // Add days offset from the 1st of the month
  tmp += RTC_buf[5] - 1;                  // Note: the first adds no days!
    
  systck_ptr->cnt_sec += tmp * 86400;    // 3600sec/hr * 24 hr/day = 86400 sec/day
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         RTC_to_SysTickTime()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    SecondstoTime()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Convert Seconds to Time Parameters    
//                      
//  MECHANICS:          This subroutine converts the seconds past January 1, 2000, to the following time
//                      parameters:
//                          Year:        0 - 127 (2000 - 2127)
//                          Month:       1 - 12  (Jan - Dec)
//                          Date:        1 - 31
//                          Day of Week: 1 - 7  (1 = Sunday)
//                          Hour:        0 - 23
//                          Minutes:     0 - 59
//
//  CAVEATS:            None
//
//  INPUTS:             Time_secs
// 
//  OUTPUTS:            days, minu, hour, year, month, dayofweek
//                      
//  ALTERS:             None
//                      
//  CALLS:              None
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void SecondstoTime(uint32_t Time_secs, uint16_t *days, uint8_t *minu, uint8_t *hour, uint8_t *year,
                                            uint8_t *month, uint8_t *dayofweek)
{
  // Extract minutes and hours from the 1sec intervals after (midnight between 1999 and 2000)
  Time_secs = Time_secs/60L;                             // Time_secs is now minutes after midnight
  *minu = (uint8_t)(Time_secs % 60L);                    // Remaining minutes after midnight
  Time_secs = Time_secs/60L;                             // Time_secs is now hours after midnight
  *hour = (uint8_t)(Time_secs % 24L);                    // Remaining hours after midnight
  Time_secs = Time_secs/24L;                             // Time_secs is now days after midnight

  // Extract day of week from the days after (midnight between 1999 and 2000).
  //   Note:  1/1/2000 was a Saturday (day of week = 7).
  *dayofweek = (uint8_t)(Time_secs % 7);
  if (*dayofweek == 0)
  {
    *dayofweek = 7;
  }

  // Extract days, months, years from the days after (midnight between 1999 and 2000)
  *days = (uint16_t)(Time_secs % (365L * 3L + 366L));    // number of days in a four-year period
  *year = (uint8_t)(Time_secs/(365L * 3L + 366L)) * 4;   // num yrs after 2000 in groups of 4-yr periods

  // Pick up remaining years within the 4-year period, and set remaining days
  if (*days < 366)                          // First year is a leap year, so 366 days
  {
    *year += 0;                             // year now holds the years after 2000
  }                                         // days has remaining days within the year
  else if (*days < 731)
  {
    *year += 1;                             // year now holds the years after 2000
    *days = *days - 366;                    // days has remaining days within the year
  }
  else if (*days < 1096)
  {
    *year += 2;                             // year now holds the years after 2000
    *days = *days - 731;                    // days has remaining days within the year
  }
  else
  {
    *year += 3;                             // year now holds the years after 2000
    *days = *days - 1096;                   // days has remaining days within the year
  }

  // Determine the month and day of month from the remaining days by subtracting out the accumulated days
  //   for the year in a per-month basis.  month will then hold the month of the year from 0 (Jan) to 11
  //   (Dec).  days will hold the date within the month
  // Must use either regular year or leap year - use two lists so compare not done each time in loop.
  if ((*year % 4) == 0)                     // If leap year:
  {
    *month = 12;                                   // Start with accumulated days to December, and work
    do                                             //   backwards to January
    {
      (*month)--;
      if (*days >= Day_Offset_Leap[*month])        // If equal, it is the first of the month
      {
        *days -= (Day_Offset_Leap[*month] - 1);    // Add one to the days because date starts with 1
        break;                                     //   Day_Offset_Reg[month]is always > 0 because
      }                                            //   month stops at 1
    }
    while (*month > 0);
  }
  else                                      // Not leap year:
  {                                         
    *month = 12;                                   // Start with accumulated days to December, and work
    do                                             //   backwards to January
    {
      (*month)--;
      if (*days >= Day_Offset_Reg[*month])         // If equal, it is the first of the month
      {
        *days -= (Day_Offset_Reg[*month] - 1);     // Add one to the days because date starts with 1
        break;                                     //   Day_Offset_Reg[month]is always > 0 because
      }                                            //   month stops at 1                          
    }
    while (*month > 0);
  }
  (*month)++;                                 // month is from 1 - 12
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         SecondstoTime()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    InternalTimeRead()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read real time from the Internal Timer    
//                      
//  MECHANICS:          This subroutine reads the internal time, converts it to standard time (year, month,
//                      date, etc.), and stores it in IntTimeBuf[ ].
//                      Internal time is read using Get_InternalTime(&presenttime), where presenttime
//                      contains:
//                          presenttime.Time_secs: seconds past January 1, 2000
//                          presenttime.Time_nsec: nanoseconds past the last second
//                      This time is converted to:
//                          IntTimeBuf[0]: hundredths of a second in hex
//                          IntTimeBuf[1]: seconds in hex (0 - 59)
//                          IntTimeBuf[2]: minutes in hex (0 - 59)
//                          IntTimeBuf[3]: hours in hex (0 - 23)
//                          IntTimeBuf[4]: weekday in hex (1 - 7, 1 = Sunday)
//                          IntTimeBuf[5]: date in hex (1 - 31)
//                          IntTimeBuf[6]: month in hex (1 - 12)
//                          IntTimeBuf[7]: year in hex (0 - 99)
//                          IntTimeBuf[8]: 3.90625msec count in hex (0 - 255) (for adjusting the RTC only)
//                      In the Gregorian calendar, the current standard calendar in most of the world, most
//                      years that are evenly divisible by 4 are leap years. In each leap year, the month of 
//                      February has 29 days instead of 28. Adding an extra day to the calendar every four 
//                      years compensates for the fact that a period of 365 days is shorter than a solar year
//                      by almost 6 hours.
//                      However, some exceptions to this rule are required since the duration of a solar 
//                      year is slightly less than 365.25 days. Years that are evenly divisible by 100 are not 
//                      leap years, unless they are also evenly divisible by 400, in which case they are leap 
//                      years.  For example, 1600 and 2000 were leap years, but 1700, 1800 and 1900 were not. 
//                      Similarly, 2100, 2200, 2300, 2500, 2600, 2700, 2900 and 3000 will not be leap years, 
//                      but 2400 and 2800 will be. By this rule, the average number of days per year will be 
//                      365 + 1/4 - 1/100 + 1/400 = 365.2425, which is 365 days, 5 h, 49 m, and 12 s. The 
//                      Gregorian calendar was designed to keep the vernal equinox on or close to March 21, 
//                      so that the date of Easter (celebrated on the Sunday after the 14th day of the 
//                      Moon—i.e. a full moon—that falls on or after March 21) remains correct with respect 
//                      to the vernal equinox.  The vernal equinox year is about 365.242374 days long 
//                      (and increasing).
//                      An algorithm for determining if a year is a leap year follows:
//                         if (year modulo 4 is 0) and (year modulo 100 is not 0) or (year modulo 400 is 0)
//                         then 
//                               is_leap_year
//                         else
//                               not_leap_year
//
//  CAVEATS:            The SysTick interrupt must be disabled when calling Get_InternalTime()
//
//  INPUTS:             ts_ptr, SysTickTime, SysClk_120MHz
// 
//  OUTPUTS:            Captured time in structure pointed to by ts_ptr
//                      
//  ALTERS:             None
//                      
//  CALLS:              Get_InternalTime(), SecondstoTime()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void InternalTimeRead(void)
{
  struct INTERNAL_TIME presenttime;
  uint16_t days;
  uint8_t sec, minu, hour, year, month, dayofweek, hundredths;

    __disable_irq();                    // Disable interrupts when checking the system clock
  Get_InternalTime(&presenttime);       // Retrieve present time
    __enable_irq();


  // Save 1/256sec count for adjusting the RTC only
  IntTimeBuf[8] = (uint8_t)(presenttime.Time_nsec/3906250L);
  hundredths = (uint8_t)(presenttime.Time_nsec/10000000L);
  if (hundredths > 99)
  {
    hundredths -= 100;
    ++presenttime.Time_secs;
  }

  // Extract seconds from the 1sec intervals after (midnight between 1999 and 2000)
  sec = (uint8_t)(presenttime.Time_secs % 60L);             // Remaining seconds after midnight

  // Extract minutes, and hours from the 1sec intervals after (midnight between 1999 and 2000)
  SecondstoTime(presenttime.Time_secs, &days, &minu, &hour, &year, &month, &dayofweek);

/*  // Extract seconds, minutes, and hours from the 1sec intervals after (midnight between 1999 and 2000)
  sec = (uint8_t)(presenttime.Time_secs % 60L);             // Remaining seconds after midnight
  presenttime.Time_secs = presenttime.Time_secs/60L;        // Time is now minutes after midnight
  minu = (uint8_t)(presenttime.Time_secs % 60L);            // Remaining minutes after midnight
  presenttime.Time_secs = presenttime.Time_secs/60L;        // Time is now hours after midnight
  hour = (uint8_t)(presenttime.Time_secs % 24L);            // Remaining hours after midnight
  presenttime.Time_secs = presenttime.Time_secs/24L;        // Time is now days after midnight
  
  // Extract day of week from the days after (midnight between 1999 and 2000).
  //   Note:  1/1/2000 was a Saturday (day of week = 7).
  dayofweek = (uint8_t)(presenttime.Time_secs % 7);
  if (dayofweek == 0)
  {
    dayofweek = 7;
  }

  // Extract days, months, years from the days after (midnight between 1999 and 2000)
                            // days holds the number of days in a four-year period
  days = (uint16_t)(presenttime.Time_secs % (365L * 3L + 366L));
                            // year holds number of years after 2000 in groups of 4-year periods
  year = (uint8_t)(presenttime.Time_secs/(365L * 3L + 366L)) * 4;

  // Pick up remaining years within the 4-year period, and set remaining days
  if (days < 366)                           // First year is a leap year, so 366 days
  {
    year += 0;                              // year now holds the years after 2000
  }                                         // days has remaining days within the year
  else if (days < 731)
  {
    year += 1;                              // year now holds the years after 2000
    days = days - 366;                      // days has remaining days within the year
  }
  else if (days < 1096)
  {
    year += 2;                              // year now holds the years after 2000
    days = days - 731;                      // days has remaining days within the year
  }
  else
  {
    year += 3;                              // year now holds the years after 2000
    days = days - 1096;                     // days has remaining days within the year
  }

  // Determine the month and day of month from the remaining days by subtracting out the accumulated days
  //   for the year in a per-month basis.  month will then hold the month of the year from 0 (Jan) to 11
  //   (Dec).  days will hold the date within the month
  // Must use either regular year or leap year - use two lists so compare not done each time in loop.
  if ((year % 4) == 0)                      // If leap year:
  {
    month = 12;                                 // Start with accumulated days to December, and work
    do                                          //   backwards to January
    {
      month--;
      if (days >= Day_Offset_Leap[month])       // If equal, it is the first of the month
      {
        days -= (Day_Offset_Leap[month] - 1);   // Add one to the days because date starts with 1
        break;                                  //   Day_Offset_Reg[month]is always > 0 because
      }                                         //   month stops at 1
    }
    while (month > 0);
  }
  else                                      // Not leap year:
  {                                         
    month = 12;                                 // Start with accumulated days to December, and work
    do                                          //   backwards to January
    {
      if (days >= Day_Offset_Reg[month])        // If equal, it is the first of the month
      {
        days -= (Day_Offset_Reg[month] - 1);    // Add one to the days because date starts with 1
        break;                                  //   Day_Offset_Reg[month]is always > 0 because
      }                                         //   month stops at 1                          
    }
    while (month > 0);
  }
  month++;                                  // month is from 1 - 12    */

  // Store the time in IntTimeBuf[]
  IntTimeBuf[0] = hundredths;
  IntTimeBuf[1] = sec;
  IntTimeBuf[2] = minu;
  IntTimeBuf[3] = hour;
  IntTimeBuf[4] = dayofweek;
  IntTimeBuf[5] = (uint8_t)days;
  IntTimeBuf[6] = month;
  IntTimeBuf[7] = year;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      InternalTimeRead()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    Check_IntRTC()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Initialize the Internal RTC
//
//  MECHANICS:          The STM32F407 contains an internal, battery-backed RTC.  This subroutine reads the
//                      RTC to determine whether it has a valid configuration and time.  This is done by
//                      checking the configuration registers for validity, and checking two of the
//                      user-configurable backup registers for specific values
//                          - If the configuration is valid, return True
//                          - If the configuration is invalid, return False
//
//                      REFERENCE: STM32F407 Reference Manual RM0090 (DocID018909) Rev 7
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             RTC register contents
//                      
//  OUTPUTS:            The subroutine returns True if the RTC has correct register contents and valid time
//                      The subroutine returns False otherwise
//                      
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     ?
// 
//------------------------------------------------------------------------------------------------------------

uint8_t Check_IntRTC(void)
{
  uint8_t ok;
  uint32_t temp;

  // Check the BDCR register.  The following bits should be checked:
  //   b0 = 1: LSE clock is on
  //   b1 = 1: LSE clock is ready
  //   b2 = 0: LSE clock is not bypassed
  //   b9..8 = 01: LSE clock is used as the RTC clock
  //   b15 = 1: RTC clock enabled
  temp = RCC->BDCR;
  ok = ( ((temp & 0x00008307) == 0x00008103) ? TRUE : FALSE);

  // Check the CR register.  The following bit should be checked:
  //   b5 = 1: Time and date values are taken directly from the time registers
  // The rest of the bits should be 0
  temp = RTC->CR;
  if (temp != 0x00000020)
  {
    ok = FALSE;
  }

  // First backup register should be special code #1.  If not, battery must have failed or RTC was never
  //   configured
  temp = RTC->BKP0R;
  if (temp != RTC_BKUP_CODE1)
  {
    ok = FALSE;
  }

  // Second backup register should be special code #2.  If not, battery must have failed or RTC was never
  //   configured
  temp = RTC->BKP1R;
  if (temp != RTC_BKUP_CODE2)
  {
    ok = FALSE;
  }

  return (ok);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      Check_IntRTC()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    IntRTC_Read()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read real time from the Internal RTC    
//                      
//  MECHANICS:          This subroutine checks the STM32F407's RTC for valid configuration and reads the
//                      date and time.  The date and time are placed in RTC_buf[0..7].
//                      Note, the values are converted from BCD format to hexadecimal format.
//                      In addition, the low bytes of the CR register and two marker bytes are stored in
//                      RTC_buf[8..10].  Finally, the 1/256sec count is stored in RTC_buf[11].  This is used
//                      only to adjust the RTC to the internal time.

//                      
//  CAVEATS:            None
//                      
//  INPUTS:             RTC time and configuration
//                      
//  OUTPUTS:            RTC_buf[ ]
//                      Returns True if time is valid, False otherwise
//                      
//  ALTERS:             None
//                      
//  CALLS:              Check_IntRTC()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

uint8_t IntRTC_Read(void)
{
  uint8_t i, j;
  uint32_t dt1[3], dt2[3];

  if (!Check_IntRTC())                  // If RTC is not ok, return False
  {
    return (FALSE);
  }
  if ((RTC->ISR & RTC_ISR_SHPF) > 0)    // If shift operation is pending, don't read
  {
    return (FALSE);
  }

  // RTC is ok, so read it.  Note, read TR register first to lock the other two
  for (i=0; i<3; ++i)
  {
    dt1[0] = RTC->TR;
    dt1[2] = RTC->SSR;
    dt1[1] = RTC->DR;
    dt2[0] = RTC->TR;
    dt2[2] = RTC->SSR;
    dt2[1] = RTC->DR;
    if ( (dt1[0] == dt2[0]) && (dt1[1] == dt2[1]) && (dt1[2] == dt2[2]) )
    {
      break;
    }
  }
  if (i == 3)
  {
    return (FALSE);
  }

  // Check the values and store them in RTC_Buf[]
  j = TRUE;                             // Use j as temporary value to return (assume ok)
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
  // SSR register
  //   b31..16: Reserved
  //   b15..0: Fraction of second - (PREDIV_S - SSR)/(PREDIV_S + 1)
  //                              = 255 - SSR/256
  // RTC_buf[0] = hundredths of a second
  if (dt2[2] < 255)                   // Sanity check
  {
    // Save number of 3.9msec (1/256) counts for adjusting the RTC
    RTC_buf[11] = 255 - (uint8_t)dt2[2];

    // Per above formula for the SSR reg.  Note, 128 added to properly round off the number (adds 0.5)
    RTC_buf[0] = (uint8_t)( ( (((uint16_t)(255 - (uint8_t)dt2[2])) * 100) + 128)/256 );
  }
  else
  {
    // Save number of 3.9msec (1/256) counts for adjusting the RTC
    RTC_buf[11] = 0;
    RTC_buf[0] = 0;
  }
  // RTC_buf[1] = seconds in hex
  i= (uint8_t)(dt2[0] & 0x0000007F);
  RTC_buf[1] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[1] > 59)
  {
    RTC_buf[1] = 59;
    j = FALSE;
  }

  // RTC_buf[2] = minutes in hex
  i = (uint8_t)((dt2[0] >> 8) & 0x0000007F);
  RTC_buf[2] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[2] > 59)
  {
    RTC_buf[2] = 59;
    j = FALSE;
  }

  // RTC_buf[3] = hours in hex
  i = (uint8_t)((dt2[0] >> 16) & 0x0000003F);
  RTC_buf[3] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[3] > 23)
  {
    RTC_buf[3] = 23;
    j = FALSE;
  }

  // RTC_buf[4] = weekday in hex
  // The internal RTC stores the day as 1=Monday, 7=Sunday
  // Our format is 1=Sunday, 7=Saturday
  RTC_buf[4] = (uint8_t)((dt2[1] >> 13) & 0x00000007);
  if (RTC_buf[4] == 0)
  {
    RTC_buf[4] = 1;
    j = FALSE;
  }
  else
  {
    RTC_buf[4] = (RTC_buf[4] + 1) % 7;
    if (RTC_buf[4] == 0)
    {
      RTC_buf[4] = 7;
    }
  }

  // RTC_buf[5] = date in hex
  i = (uint8_t)(dt2[1] & 0x0000007F);
  RTC_buf[5] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[5] > 31)
  {
    RTC_buf[5] = 31;
    j = FALSE;
  }
  else if (RTC_buf[5] == 0)
  {
    RTC_buf[5] = 1;
    j = FALSE;
  }

  // RTC_buf[6] = month in hex
  i = (uint8_t)((dt2[1] >> 8) & 0x0000001F);
  RTC_buf[6] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[6] > 12)
  {
    RTC_buf[6] = 12;
    j = FALSE;
  }
  else if (RTC_buf[6] == 0)
  {
    RTC_buf[6] = 1;
    j = FALSE;
  }

  // RTC_buf[7] = year in BCD
  i = (uint8_t)((dt2[1] >> 16) & 0x000000FF);
  RTC_buf[7] = ( (i & 0x0F) + ((i >> 4) * 10) );
  if (RTC_buf[7] > 99)
  {
    RTC_buf[7] = 99;
    j = FALSE;
  }

  // Save low byte of CR register (b5 = Bypass Shadow Registers = 0)
  RTC_buf[8] = (uint8_t)RTC->CR;
  // Save low bytes of marker registers
  RTC_buf[9] = (uint8_t)RTC->BKP0R;
  RTC_buf[10] = (uint8_t)RTC->BKP1R;

  return (j);
  
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      IntRTC_Read()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    IntRTC_Write()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Write Time to the Internal RTC
//
//  MECHANICS:          This subroutine writes the time in RTC_buf[ ] into the RTC
//
//                      REFERENCE: STM32F407 Reference Manual RM0090 (DocID018909) Rev 7
//                      
//  CAVEATS:            None
//                      
//  INPUTS:             RTC_buf[]
//                      
//  OUTPUTS:            None
//                      
//  ALTERS:             RTC register contents
//
//  CALLS:              None
//
//  EXECUTION TIME:     ?
// 
//------------------------------------------------------------------------------------------------------------

void IntRTC_Write(void)
{
  uint8_t i, j, k, n;
  uint16_t cnt;


  //  RTC_buf[0] = hundredths of a second in BCD (0 - 99)
  //  RTC_buf[1] = seconds in BCD (0 - 59)
  //  RTC_buf[2] = minutes in BCD (0 - 59)
  //  RTC_buf[3] = hours in BCD (0 - 23)
  //  RTC_buf[4] = day in BCD (1 - 7, 1 = Sunday)
  //  RTC_buf[5] = date in BCD (1 - 31)
  //  RTC_buf[6] = month in BCD (1 - 12)
  //  RTC_buf[7] = year in BCD (0 - 99)
  
    PWR->CR |= 0x00000100;                  // Set DBP bit to enable write access to RTC registers
    for (i=0; i<5; ++i)                     // Add slight delay loop just to be safe
    {
    }
    RTC->WPR = 0xCA;                        // Key to unlock registers for access
    for (i=0; i<5; ++i)                     // Add slight delay loop just to be safe
    {
    }
    RTC->WPR = 0x53;
    for (i=0; i<5; ++i)                     // Add slight delay loop just to be safe
    {
    }

    RTC->ISR |= 0x00000080;                 // Enter initialization mode
    while ((RTC->ISR & 0x00000040) == 0)    // Wait for initialization mode to be entered
    {
    }

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
    i = RTC_buf[3];                             // Convert to BCD
    i = ( ((i/10) << 4) | (i % 10) );
    j = RTC_buf[2];                             // Convert to BCD
    j = ( ((j/10) << 4) | (j % 10) );
    k = RTC_buf[1];                             // Convert to BCD
    k = ( ((k/10) << 4) | (k % 10) );
    RTC->TR = ( (((uint32_t)i) << 16) + (((uint32_t)j) << 8) + k );
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
    i = RTC_buf[7];                             // Convert to BCD
    i = ( ((i/10) << 4) | (i % 10) );
    j = RTC_buf[6];                             // Convert to BCD
    j = ( ((j/10) << 4) | (j % 10) );
    k = RTC_buf[5];                             // Convert to BCD
    k = ( ((k/10) << 4) | (k % 10) );
    // RTC_buf[4] = weekday in hex
    // Our format is 1=Sunday, 7=Saturday
    // The internal RTC stores the day as 1=Monday, 7=Sunday
    n = RTC_buf[4] - 1;
    if (n == 0)
    {
      n = 7;
    }
    RTC->DR = ( (((uint32_t)i) << 16) + (((uint32_t)n) << 13) + (((uint32_t)j) << 8) + k );
    RTC->ISR &= 0xFFFFFF7F;                          // Exit initialization mode to load values in
    while ((RTC->ISR & 0x00000040) == 0x00000040)    // Wait for initialization mode to be exited
    {
    }
    for (cnt=0; cnt<1500; ++cnt)                   // *** DAH TEST  get no errors (apparently) with this in
    {
      if ((RTC->ISR & 0x00000020) == 0x00000020)
      {
        break;
      }
    }
    RTC->ISR &= 0xFFFFFFDF;                       // *** END DAH TEST

    // Write special codes into two of the backup register as a marker to show the RTC was configured
    RTC->BKP0R = RTC_BKUP_CODE1;
    RTC->BKP1R = RTC_BKUP_CODE2;

    RTC->WPR = 0;                   // Lock them up again
    PWR->CR &= 0xFFFFFEFF;          // Clear DBP bit to safeguard registers

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      IntRTC_Write()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    IntRTC_Update()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update the Internal RTC
//
//  MECHANICS:          This subroutine compares the RTC time to the internal time and adjusts the RTC time
//                      if necessary.  The following steps are taken:
//                          1) Read the internal time
//                          2) Read the RTC time
//                          3) Compare the two times
//                              a) |RTC time - internal time| > 1 second:
//                                        write the internal time into the RTC time
//                              b) 30msec < |RTC time - internal time| < 1sec:
//                                  b)i)  use RTC shift register to adjust time
//                                  b)ii) add 1 second and use RTC shift register to adjust time
//                              c) |RTC time - internal time| < 30msec:
//                                        do nothing
//
//                      REFERENCE: STM32F407 Reference Manual RM0090 (DocID018909) Rev 7
//                      
//  CAVEATS:            RTC_buf[] and IntTimeBuf[] are also used and set in other subroutines, such as
//                      TP_RTC().  These variables must be set and used in one pass through this subroutine.
//                      They may be changed the next time the subroutine is called.
//                      
//  INPUTS:             RTC_buf[], IntTimeBuf[]
//                      
//  OUTPUTS:            None
//                      
//  ALTERS:             RTC register contents, RTC_ShiftVal, RTC_State
//
//  CALLS:              Get_InternalTime(), InternalTimeRead(), IntRTC_Write()
//
//  EXECUTION TIME:     ?
// 
//------------------------------------------------------------------------------------------------------------

void IntRTC_Update(void)
{
  uint8_t i, RU_exit;
  struct SYSTICK_TIME RTC_time;
  struct INTERNAL_TIME inttime;
  uint32_t inttime10msec;

  RU_exit = (RTC_State == 0);

  while (!RU_exit)
  {
    switch (RTC_State)
    {
      case 0:                           // Not time to do adjustment (should never be entered)
      default:
        RU_exit = TRUE;
        break;

      case 1:                           // Read the RTC and internal times and see if should adjust
        if (IntRTC_Read())                  // Read RTC time.  If read is successful, read the internal time
        {                                   //   also
          __disable_irq();
          Get_InternalTime(&inttime);         // This is used to compare times quickly
          InternalTimeRead();                 // Read internal time - need this to compute ShiftVal
          __enable_irq();                     //   (computed using RTC_bufInt[11] and TimeBuf[8])
        }
        else                                // If read is unsuccessful, quit - will try next time
        {
          RU_exit = TRUE;
          break;
        }
        // Time has been retrieved from the RTC and the internal timer
            // RTC Time:
            //   RTC_buf[0] = hundredths of a second in hexadecimal (0 - 99)
            //   RTC_buf[1] = seconds in hexadecimal (0 - 59)
            //   RTC_buf[2] = minutes in hexadecimal (0 - 59)
            //   RTC_buf[3] = hours in hexadecimal (0 - 23)
            //   RTC_buf[4] = day in hexadecimal (1 - 7, 1 = Sunday)
            //   RTC_buf[5] = date in hexadecimal (1 - 31)
            //   RTC_buf[6] = month in hexadecimal (1 - 12)
            //   RTC_buf[7] = year in hexadecimal (0 - 99)
            //   RTC_buf[11] = 1/256sec in hexadecimal (0 - 255)
            // System Time:
            //   IntTimeBuf[0]: hundredths of a second in hex
            //   IntTimeBuf[1]: seconds in hex (0 - 59)
            //   IntTimeBuf[2]: minutes in hex (0 - 59)
            //   IntTimeBuf[3]: hours in hex (0 - 23)
            //   IntTimeBuf[4]: weekday in hex (1 - 7, 1 = Sunday)
            //   IntTimeBuf[5]: date in hex (1 - 31)
            //   IntTimeBuf[6]: month in hex (1 - 12)
            //   IntTimeBuf[7]: year in hex (0 - 99)
            //   IntTimeBuf[8]: 1/256sec in hexadecimal (0 - 255)
  
        RTC_to_SysTickTime(&RTC_time);                  // Convert RTC time to SysTickTime format
        inttime10msec = inttime.Time_nsec/10000000L;    // Convert internal time nsec to 10msec

        // Compare RTC time to Internal Time
        // If the time is off by more than 1 second, need to write the internal time into the RTC
        if ( (RTC_time.cnt_sec > (inttime.Time_secs+1)) || (inttime.Time_secs > (RTC_time.cnt_sec+1))
          || ((RTC_time.cnt_sec > inttime.Time_secs) & (RTC_time.cnt_10msec >= inttime10msec))
          || ((inttime.Time_secs > RTC_time.cnt_sec) & (inttime10msec >= RTC_time.cnt_10msec)) )
        {
          RTC_State = 2;
        }
        // If times are between ~500msec and 1 sec apart, need to adjust RTC using shift register
          // RTC time > internal time: delay RTC time if difference > ~500msec
        else if ( ((RTC_time.cnt_sec > inttime.Time_secs) && ((RTC_time.cnt_10msec + 50) >= inttime10msec))
               || ((RTC_time.cnt_sec == inttime.Time_secs) && (RTC_time.cnt_10msec >= (inttime10msec+50))) )
        {
          RTC_ShiftVal = RTC_buf[11] - IntTimeBuf[8];
          RTC_State = 3;
        }
          // RTC time < internal time: advance RTC time if difference > ~500msec
        else if ( ((inttime.Time_secs > RTC_time.cnt_sec) && ((inttime10msec + 50) >= RTC_time.cnt_10msec))
               || ((inttime.Time_secs == RTC_time.cnt_sec) && (inttime10msec >= (RTC_time.cnt_10msec+50))) )
        {
          RTC_ShiftVal = IntTimeBuf[8] - RTC_buf[11];
          RTC_ShiftVal = 256 - RTC_ShiftVal;
          RTC_State = 4;
        }
        else                                            // Times are within 500msec of each other, done (do
        {                                               //   nothing)
          RTC_State = 0;
          RU_exit = TRUE;
        }
        break;

      case 2:                           // Write internal time into RTC
        InternalTimeRead();                 // Read internal time to get the latest time before writing to
        for (i=0; i<8; ++i)                 //   the RTC
        {
          RTC_buf[i] = IntTimeBuf[i];
        }
        IntRTC_Write();
        RTC_State = 1;                      // Set state to 1 to recheck times and do finer adjustment
        RU_exit = TRUE;                     //   if necessary
        break;

      case 3:                           // Adjust RTC using shift register (less than 1 second adjustment)
      case 4:
        if ( ((uint32_t)RTC_ShiftVal + RTC->SSR) < 0x0000FFFE)      // Make sure adding shift value won't
        {                                                           //   cause overflow of the SSR register
          // The shift register is protected - need to unlock it first!
          PWR->CR |= 0x00000100;            // Set DBP bit to enable write access to RTC registers
          for (i=0; i<5; ++i)               // Add slight delay loop just to be safe
          {
          }
          RTC->WPR = 0xCA;                  // Key to unlock registers for access
          for (i=0; i<5; ++i)               // Add slight delay loop just to be safe
          {
          }
          RTC->WPR = 0x53;
          for (i=0; i<5; ++i)               // Add slight delay loop just to be safe
          {
          }
          if (RTC_State == 3)                                       // State 3: Delay the clock
          {
            RTC->SHIFTR = (uint32_t)RTC_ShiftVal;
          }
          else                                                      // State 4: Advance the clock
          {
            RTC->SHIFTR = ((uint32_t)RTC_ShiftVal | 0x80000000);
          }
          RTC->WPR = 0;                     // Lock them up again
          PWR->CR &= 0xFFFFFEFF;            // Clear DBP bit to safeguard registers
        }
        // If shift causes overflow (should never happen), then we won't do it, and eventually the RTC will
        //   be written with the internal time when they drift 1 second apart
        RTC_State = 0;
        RU_exit = TRUE;
        break;
    }
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      IntRTC_Update()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    InternalTimeToMBTime()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Convert Internal Time to Modbus Time    
//                      
//  MECHANICS:          This subroutine converts the internal time stamp pointed at by inttime_ptr to the
//                      Modbus time stamp format:
//                        Inputs:
//                          inttime_ptr-->Time_secs         Seconds past January 1, 2000
//                          inttime_ptr-->Time_nsec         Nanoseconds past the last second
//                        Outputs:
//                          outtime b63..55: reserved = 0
//                          outtime b54..48: Year 0 - 127 (2000 - 2127)
//                          outtime b47..44: reserved = 0
//                          outtime b43..40: Month 1 - 12 (Jan - Dec)
//                          outtime b39..37: reserved = 0
//                          outtime b36..32: Date 1 - 31
//                          outtime b31..29: reserved = 0
//                          outtime b28..24: Hour 0 - 23
//                          outtime b23..22: reserved = 0
//                          outtime b21..16: Minutes 0 - 59
//                          outtime b15..0:  Milliseconds 0 - 59,999 (msec in one minute)
//                      This time is based on IEC 60870-5
//                      In the Gregorian calendar, the current standard calendar in most of the world, most
//                      years that are evenly divisible by 4 are leap years. In each leap year, the month of 
//                      February has 29 days instead of 28. Adding an extra day to the calendar every four 
//                      years compensates for the fact that a period of 365 days is shorter than a solar year
//                      by almost 6 hours.
//                      However, some exceptions to this rule are required since the duration of a solar 
//                      year is slightly less than 365.25 days. Years that are evenly divisible by 100 are not 
//                      leap years, unless they are also evenly divisible by 400, in which case they are leap 
//                      years.  For example, 1600 and 2000 were leap years, but 1700, 1800 and 1900 were not. 
//                      Similarly, 2100, 2200, 2300, 2500, 2600, 2700, 2900 and 3000 will not be leap years, 
//                      but 2400 and 2800 will be. By this rule, the average number of days per year will be 
//                      365 + 1/4 - 1/100 + 1/400 = 365.2425, which is 365 days, 5 h, 49 m, and 12 s. The 
//                      Gregorian calendar was designed to keep the vernal equinox on or close to March 21, 
//                      so that the date of Easter (celebrated on the Sunday after the 14th day of the 
//                      Moon—i.e. a full moon—that falls on or after March 21) remains correct with respect 
//                      to the vernal equinox.  The vernal equinox year is about 365.242374 days long 
//                      (and increasing).
//                      An algorithm for determining if a year is a leap year follows:
//                         if (year modulo 4 is 0) and (year modulo 100 is not 0) or (year modulo 400 is 0)
//                         then 
//                               is_leap_year
//                         else
//                               not_leap_year
//
//  CAVEATS:            None
//
//  INPUTS:             inttime_ptr
// 
//  OUTPUTS:            The functions returns the time as described above as a U64
//                      
//  ALTERS:             None
//                      
//  CALLS:              SecondstoTime()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

uint64_t InternalTimeToMBTime(struct INTERNAL_TIME *inttime_ptr)
{
  uint32_t Time_secs;
  uint16_t msec, days;
  uint8_t sec, minu, hour, year, month, dayofweek;

  Time_secs = inttime_ptr->Time_secs;                    // Seconds past January 1, 2000

  // Extract seconds from the 1sec intervals after (midnight between 1999 and 2000)
  sec = (uint8_t)(Time_secs % 60L);                      // Remaining seconds after midnight

  // Compute msec past last minute.  This is:
  //     nsec past the last second/1000000 (convert to msec)
  //   + sec past last minute * 1000 (convert to msec)
  msec = ((uint16_t)(inttime_ptr->Time_nsec/1000000L)) + ((int16_t)sec * 1000);
  // If this somehow exceeds one minute, there's an error somewhere.  Adjust the times accordingly by
  //   decrementing 1 second from the msec count and incrementing the second count
  if (msec >= 60000)
  {
    msec -= 60000;
    Time_secs++;    
    sec = (uint8_t)(Time_secs % 60L);                      // Remaining seconds after midnight
  }

  // Extract minutes, and hours from the 1sec intervals after (midnight between 1999 and 2000)
  SecondstoTime(Time_secs, &days, &minu, &hour, &year, &month, &dayofweek);

/*  // Extract minutes and hours from the 1sec intervals after (midnight between 1999 and 2000)
  Time_secs = Time_secs/60L;                             // Time_secs is now minutes after midnight
  minu = (uint8_t)(Time_secs % 60L);                     // Remaining minutes after midnight
  Time_secs = Time_secs/60L;                             // Time_secs is now hours after midnight
  hour = (uint8_t)(Time_secs % 24L);                     // Remaining hours after midnight
  Time_secs = Time_secs/24L;                             // Time_secs is now days after midnight

  // Extract day of week from the days after (midnight between 1999 and 2000).
  //   Note:  1/1/2000 was a Saturday (day of week = 7).
  dayofweek = (uint8_t)(Time_secs % 7);
  if (dayofweek == 0)
  {
    dayofweek = 7;
  }

  // Extract days, months, years from the days after (midnight between 1999 and 2000)
  days = (uint16_t)(Time_secs % (365L * 3L + 366L));     // number of days in a four-year period
  year = (uint8_t)(Time_secs/(365L * 3L + 366L)) * 4;    // num yrs after 2000 in groups of 4-yr periods

  // Pick up remaining years within the 4-year period, and set remaining days
  if (days < 366)                           // First year is a leap year, so 366 days
  {
    year += 0;                              // year now holds the years after 2000
  }                                         // days has remaining days within the year
  else if (days < 731)
  {
    year += 1;                              // year now holds the years after 2000
    days = days - 366;                      // days has remaining days within the year
  }
  else if (days < 1096)
  {
    year += 2;                              // year now holds the years after 2000
    days = days - 731;                      // days has remaining days within the year
  }
  else
  {
    year += 3;                              // year now holds the years after 2000
    days = days - 1096;                     // days has remaining days within the year
  }

  // Determine the month and day of month from the remaining days by subtracting out the accumulated days
  //   for the year in a per-month basis.  month will then hold the month of the year from 0 (Jan) to 11
  //   (Dec).  days will hold the date within the month
  // Must use either regular year or leap year - use two lists so compare not done each time in loop.
  if ((year % 4) == 0)                      // If leap year:
  {
    month = 12;                                 // Start with accumulated days to December, and work
    do                                          //   backwards to January
    {
      month--;
      if (days >= Day_Offset_Leap[month])       // If equal, it is the first of the month
      {
        days -= (Day_Offset_Leap[month] - 1);   // Add one to the days because date starts with 1
        break;                                  //   Day_Offset_Reg[month]is always > 0 because
      }                                         //   month stops at 1
    }
    while (month > 0);
  }
  else                                      // Not leap year:
  {                                         
    month = 12;                                 // Start with accumulated days to December, and work
    do                                          //   backwards to January
    {
      if (days >= Day_Offset_Reg[month])        // If equal, it is the first of the month
      {
        days -= (Day_Offset_Reg[month] - 1);    // Add one to the days because date starts with 1
        break;                                  //   Day_Offset_Reg[month]is always > 0 because
      }                                         //   month stops at 1                          
    }
    while (month > 0);
  }
  month++;                                  // month is from 1 - 12   */

  // Return the time in the format listed above
  //   b63..55: reserved = 0
  //   b54..48: Year 0 - 127 (2000 - 2127)
  //   b47..44: reserved = 0
  //   b43..40: Month 1 - 12 (Jan - Dec)
  //   b39..37: reserved = 0
  //   b36..32: Date 1 - 31
  //   b31..29: reserved = 0
  //   b28..24: Hour 0 - 23
  //   b23..22: reserved = 0
  //   b21..16: Minutes 0 - 59
  //   b15..0:  Milliseconds 0 - 59,999 (msec in one minute)
  return (  ( ((uint64_t)year) << 48)       // will use b55 in year 2128
          | ( ((uint64_t)month) << 40)
          | ( ((uint64_t)days) << 32)
          | ( ((uint64_t)hour) << 24)
          | ( ((uint64_t)minu) << 16)
          | (msec)  );
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      InternalTimeToMBTime()
//------------------------------------------------------------------------------------------------------------
