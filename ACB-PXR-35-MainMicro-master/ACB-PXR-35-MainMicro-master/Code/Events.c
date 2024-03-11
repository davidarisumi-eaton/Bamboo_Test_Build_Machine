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
//  MODULE NAME:        Events.c
//
//  MECHANICS:          Program module containing the subroutines to process events
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.22   180420  DAH - Added code for demand and energy logging
//                          - Event_VarInit(), EventManager(), and ClearEvents() added
//   0.23   180504  DAH - Added FLASH_IN_USE arbitration when writing events to Flash in EventManager()
//                      - In ClearEvents(), Dmnd_I_VarInit() renamed to Dmnd_VarInit()
//                      - Fixed bug in EventManager() that caused demand event processing to freeze whenever
//                        the next sector in Flash needed to be erased
//   0.24   180517  DAH - Combined Flash_def.h and FRAM_def.h
//                          - Deleted include of Flash_def.h
//                          - Replaced include of FRAM_def.h with FRAM_Flash_def.h
//                      - In EventManager(), added support for multiple Flash requestors
//                          - Replaced SystemFlags with FlashCtrl for FLASH_IN_USE arbitration
//                      - In EM_ENERGYLOG2 state of EventManager(), modified Flash address rollover check
//                        from ">= ENERGY_SECTOR_END" to "> ENERGY_SECTOR_END" because reduced
//                        ENERGY_SECTOR_END by one, so it is the last valid sector
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Modified EventManager() to use the request and acknowledge flags when writing
//                            the demand logs
//                          - Modified ClearEvents() to use the request flag when erasing the next sector
//                      - Added support for Alarm waveform capture events
//                          - Added InsertNewEvent()
//   0.27   181001  DAH - Revised ClearEvents() to clear out and initialize Trip, Alarm, and Strip-Chart
//                        waveforms
//   0.37   200604  DAH - Additional firmware development of the events code.  Memory allocation in FRAM and
//                        Flash was firmed up, and the basic method for retrieving events information has
//                        been implemented.  This has been done in support of the proc-proc communications,
//                        and was implemented for the USB PXPM events support.
//                      - Revised ClearEvents() to clear Num_Events and NextEvntIndex when initializing
//                        Trip, Alarm, and Strip-Chart waveforms
//                      - Changed the MasterEID storage in FRAM to include a complement and second copy
//                          - Revised Event_VarInit() and ClearEvents() accordingly
//                      - Added support for reading the earliest and latest event IDs for display processor
//                        communications
//                          - Added GetEVInfo[]
//                      - Added code for storing event logs in FRAM and Flash, and revised Flash waveform
//                        storage algorithm to use an index to compute the Flash and FRAM addresses instead
//                        of the actual Flash address.  This is easier to manipulate, and provides
//                        commonality between the Flash and FRAM storage.  All log entries now use
//                        index-based storage
//                          - Added struct EV_SUM, struct EV_DMND, and struct EV_ADD variables
//                          - Added constant arrays RAM_EV_ADD[ ], FRAM_EV_ADD[ ], MAX_NUM_EVENTS[ ],
//                            FRAM_EV_INFO[ ], and EV_SIZE[]
//                          - Event_VarInit() and ClearEvents() modified to initialize the new variables
//                          - Added ReadEVAddress()
//                      - Moved events-related initialization out of IO_VarInit() and into Event_VarInit()
//                          - Added calls to ReadEVAddress() to retrieve the index and number of events from
//                            FRAM
//                      - Revised Demand and Energy logging variables to be compatible with the other event
//                        variables
//                          - Replaced Energy_Log_Add with EV_Dmnd.NextDmndAdd
//                          - Added EV_Dmnd.Num_Entries to track how many demand and energy log entries were
//                            made and when the log has rolled over
//                          - Revised Event_VarInit() to use ReadEVAddress() to retrieve the values from
//                            FRAM
//   0.41   201028  DAH - Corrected error in GetEVInfo() when retrieving the earliest and latest Event IDs
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added includes of RealTime_def.h and RealTime_ext.h
//   0.46   210521  DAH - In ClearEvents(), moved call to Event_VarInit() after the initialization of
//                        xxx.NextEvntNdx so waveform buffers are cleared correctly
//   0.59   220831  DAH - In EventManager(), for Trip events, added code to capture and write the EID and
//                        waveform header to FRAM, and to write the index and number of samples to FRAM
//                      - In EventManager() demand and energy states (EM_ENERGYLOGxx), added code to do FRAM
//                        writes (this was in the SPI2_Flash_Manager() with the Flash handling)
//   0.60   220907  DAH - In EventManager(), added summary event log entry for trip events
//   0.61   220930  DB    EventSummaryWrite(), EventTimeAdjWrite(), EventTripWrite(), EventMinAlarmWrite(),
//                        EventMajAlarmWrite() added.
//   0.62   221027  DAH - Revised FRAM_Read() to eliminate device parameter as it is dedicated to the
//                        on-board FRAM
//   0.63   221111  DB  - EventLookUp, BinarySearch and ReadValuesFRAM added
//   0.64   221118  DAH - Revised EventManager() for demand log codes and to handle unrecognized codes
//   0.65   221201  DB  - EID parameter in BinarySearch() and EventLookUp() definitions changed from
//                        uint16_t to uint32_t
//   0.66   221206 DAH  - Fixed bug in EventManager().  NewEventOutNdx was being incremented in state
//                        EM_ENERGYLOG.  It shouldn't be, as it is incremented in state EM_FINISH
//                      - Renamed ReadValuesFRAM() to ReadEID() and eliminated length parameter since it is
//                        only used to read the EID.  Instead of reading "length" number of words, it only
//                        reads the first two words in the log entry, which is the EID.  This significantly
//                        reduces the execution time
//                      - Revised ClearEvents() to initialize the Master EID to 1 instead of 0
//   0.67   221209  DAH - S2F_xx flags renamed to S1F_xx flags
//                      - SPI2Flash.xx renamed to SPI1Flash.xx
//   0.68   230124  DB  - Extended Capture function added - it manages extended snapshot captured values into BB FRAM
//   0.69   230220  DAH - Revised GetEVInfo() to just return the requested header info.  It no longer
//                        assembles the response message.  Also corrected bug for when log wraps around and
//                        index for next log is at 0
//                      - Deleted Minor Alarm events.  These are handled by Basic events with the expanded
//                        Summary Log codes
//                          - EventMinAlarmWrite() deleted
//                          - Minor alarm states deleted from EventManager()
//                          - EventMajAlarmWrite() renamed to EventAlarmWrite()
//                          - MINALARM_EVENT_SIZE deleted from EV_SIZE[]
//                          - MINALARM_NUM_LOGS deleted from MAX_NUM_EVENTS[]
//                          - MINALARM_LOG_START deleted from FRAM_EV_INFO[]
//                          - MINORALARM_LOG_ADD deleted from FRAM_EV_ADD[]
//                          - &EV_MinAlarm deleted from RAM_EV_ADD[]
//                          - Event_VarInit() and ClearEvents() modified to delete Minor Alarm Log support
//                          - struct EV_MinAlarm deleted
//                          - struct EV_MajAlarm renamed to struct EV_Alarm
//                          - MAJORALARM_LOG_ADD renamed to ALARM_LOG_ADD
//                          - MAJALARM_EVENT_SIZE renamed to ALARM_EVENT_SIZE
//                          - MAJALARM_NUM_LOGS renamed to ALARM_NUM_LOGS
//                          - MAJALARM_LOG_START renamed to ALARM_LOG_START
//                      - Deleted Strip-Chart waveforms.  These have been replaced by the extended capture
//                        waveforms
//                          - Modified Event_VarInit() to replace strip-chart variable initialization with
//                            extended capture variable initialization
//                          - Deleted CHART_WF_ADD from FRAM_EV_ADD[]
//                          - Deleted Chart_WF_Capture from RAM_EV_ADD[]
//                          - NUM_CHART_WAVEFORMS deleted from MAX_NUM_EVENTS[]
//                          - CHART_WF_INFO deleted from FRAM_EV_INFO[]
//                          - Strip-chart placeholder deleted from EV_SIZE[]
//                          - Replaced strip-chart variable initialization with extended capture variable
//                            initialization in ClearEvents()
//                      - In EventManager(), revised Idle state to reflect rearranging and renumbering of
//                        event codes
//                      - In EventManager(), deleted code to handle trips without waveform captures.  All
//                        Trip events, except Test Trips, generate a waveform capture.  Test Trips are
//                        handled in separate states.
//                      - In EventManager(), deleted superfluous code from EM_EXT_CAPT state
//                      - NewEventFIFO[].Type renamed to NewEventFIFO.Code
//   0.70   230221  DB  - DisturbanceCapture function and structure DISTURBANCE_CAPTURE added - this
//                        function manages Disturbance capture with 3 stages inside
//                        1. Generate Entry Time
//                        2. Acumulate AVG values for One Minute Subinterval
//                        3. Acumulate Total AVG values and store complete structure DISTURBANCE_CAPTURE
//                        then it generates Event Summary with Exit Time Stamp
//   0.72   230320  DAH - Revised EventManager() to check waveform capture flags when advancing state
//                      - Revised  SPI1Flash.Ack and SPI1Flash.Req operation
//                          - EventManager() revised
//                      - Added EventExtCapWrite()
//   24     230323  DAH - Revised extended captures to lock out new events after a capture has been made
//                        until the extended capture has been completed (on an individual event basis)
//                          - ExtCap_Flag renamed to ExtCap_ReqFlag
//                          - Added ExtCap_AckFlag
//                      - Added saving EID for the extended capture log entries
//                          - In EventManager(), added code to save the EID for the event that initiated the
//                            extended capture (in state EM_EXT_CAPT)
//   25     230403  DAH - Added code to initialize Extended Capture snapshot addresses in Event_VarInit()
//                      - Added support for retrieving extended capture logs
//                          - Modified RAM_EV_ADD[], FRAM_EV_ADD[], MAX_NUM_EVENTS[], FRAM_EV_INFO[],
//                            EV_SIZE[]
//                      - Added support for retrieving summary logs
//                          - EventSummaryRead() added
//                      - Added support for retrieving waveform EIDs
//                          - EventGetWfEIDs() added
//                      - Corrected bug in ExtendedCapture() in inserting an event into the queue.  Replaced
//                        lines of code with call to InsertNewEvent()
//   26     230403  DAH - Added support for Extended Capture one-cycle and 200msec proc-proc Read Commands
//                          - Modified ExtendedCaptureValues() to save the timestamp of the first capture
//   27     230404  DAH - Added support for Disturbance Capture Read Commands
//                          - Added EventDisturbanceRead()
//                      - Revised Disturbance capture processing, primarily fixing average value computation
//                          - Added disturbance index as an input parameter to DisturbanceCaptureValues()
//                          - In DisturbanceCaptureValues(), modified average value computation so minute
//                            values are proportionally weighted versus the one-cycle values
//                          - Changed the disturbance capture cancel flag so that it is bit-mapped to handle
//                            each capture type on an individual basis
//                          - Added process code (const DIST_PROC_CODE) to DisturbanceCaptureValues() so
//                            that the processing will match the value (i.e., min value for UV, max value
//                            OV)
//   28     230406  DAH - Added support for GOOSE capture command
//                          - Added Dist_GOOSE flag to control disturbance captures initiated by GOOSE comms
//                          - Modified DisturbanceCapture() to check Dist_GOOSE flag
//                      - In GetEVInfo(), fixed bug decoding the event type
//   29     230406  DAH - Added clearing extended capture snapshot logs in ClearEvents()
//   30     230412  DAH - In ExtendedCaptureValues() replaced call to ExtCapSnapshotMeter() with calls to
//                        ExtCapSnapshotMeterOneCyc() and ExtCapSnapshotMeterTwoHundred(), as one-cycle
//                        values are captured for the 6sec readings and 200msec values are captured for the
//                        60sec readings
//   31     230414  DAH - Revised code to handle instances where multiple extended capture events of the
//                        type occur (like HL2 going in and out).  In this case, the initial extended
//                        capture event initiates Summary, Snapshot, Waveform, and RMS capture entries.
//                        Subsequent events only cause Summary events to be made until the Extended Capture
//                        is completed (after 60 seconds).
//                          - Added new extended capture event codes for just summary events
//                          - Revised EventManager() to make log entries based on the new codes
//                          - Revised ExtendedCapture() to process requests even if a previous request is in
//                            progress.  Also revised to clear the request once it is recognized.  This
//                            prevents the same event occurrence from causing multiple entries.
//   36     230428  DAH - Decoupled waveform captures from snapshot and summary captures, so that snapshot
//                        and summary entries will be logged immediately for new events even as a previous
//                        event's waveform capture is still being stored
//                          - Added EV_WfState
//                          - Revised EventManager() to move waveform captures into a separate state machine
//                          - Revised ExtendedCapture() to only set the waveform capture flag for new event
//                            occurrences
//   37     230516  DAH - Revised EC_START state in ExtendedCaptureValues() to only begin the RMS captures
//                        if the extended capture waveform capture is in progress.  The waveform and RMS
//                        values are considered a set.  If the waveform capture isn't made (because an alarm
//                        or trip capture is occurring), the RMS captures are not made
//                      - Revised ClearEvents() to set the extended capture EID to 0.  The EID for an
//                        extended capture can never be 0
//                      - In DisturbanceCaptureValues(), added DS_WAIT state between DS_END and DS_CANCEL.
//                        This state waits for the event values to be written to FRAM before they are
//                        cleared in DS_CANCEL.
//                        Also revised DisturbanceCapture() to accomodate this extra state
//                        The values to be written are no longer stored in sDisturbanceCaptureVal.  They are
//                        pulled directly from sListOfDist[].  This allows all of the disturbance captures
//                        to be processed independently and "simultaneously"
//                      - In ExtendedCapture(), added code to save the EID of the extended capture event in
//                        case it initiates a disturbance capture
//                      - Added Goose_Capture_Code to support GOOSE Global Capture commands
//   38     230518  DAH - Revised ExtendedCapture() to capture the EID of the event for the disturbance
//                        capture even when only a summary log entry (no extended log entries) is being made
//                      - Fixed minor bug in DisturbanceCaptureValues() clearing memory
//   40     230531  DAH - In EventManager(), added Summary Code input to call to SnapshotMeter()
//                      - Corrected address offset bug in EventAlarmWrite()
//                      - Fixed bug in EventManager() in saving the EID for extended capture waveforms
//                      - Replaced TRIP_EVENT_SIZE, TESTTRIP_EVENT_SIZE, ALARM_EVENT_SIZE, and
//                        EXTCAP_EVENT_SIZE with SNAPSHOTS_EVENT_SIZE since the log entries are the same for
//                        these four event logs
//                          - EventTripWrite(), EventTestTripWrite(), EventAlarmWrite(), EventExtCapWrite(),
//                            EV_SIZE[] revised
//                      - Added EventSnapshotSummaryRead() to support reading the snapshot summaries
//                      - Added EventSnapshotRead() to support reading the snapshots
//                      - Revised EventDisturbanceRead() to return total number of logs and eliminated EID
//                        info from response
//   44     230623  DAH - Revised EventGetWfEIDs() to also return whether a requested EID is present.  If
//                        the log is available, the index is returned.  Parameters srch_eid and *match_ndx
//                        were added to the subroutine
//                      - Fixed bugs in EventSnapshotRead() and EventDisturbanceRead()
//                      - Revised EventDisturbanceRead() to use the EID that INITIATED the capture (not the
//                        EID of the actual event entry), when searching for an event by EID.  This EID is
//                        stored as data in the log.  It is more useful to use this EID because it ties the
//                        disturbance capture to the other event log entries for the Event
//   46     230703  DAH - Moved definition of MAX_NUM_EVENTS[] from local area to global area as it is now
//                        also used in DispComm.c
//   54     230801  DAH - Added support for SDPU and GF disturbance captures
//                          - DIST_PROC_CODE[], pDistValues[], aDistCause[], and ECNDX_TO_DISTNDX[] modified
//                          - Dist_Flag_SD, Dist_Flag_GF, Dist_Flag_Cancel_SD, Dist_Flag_Cancel_GF added.
//                            These are disturbance capture flags used in interrupts
//                          - Revised DisturbanceCapture() to insert SD and GF flags into bit-flag variables
//                            using the corresponding interrupt-flag variables
//                          - Revised EventManager() to save the EID for SDPU pickup and for a GF Alarm
//                          - Revised DisturbanceCaptureValues() to save the initiating EID when a capture
//                            is canceled (some test instances occur where it is needed)
//                          - In DisturbanceCaptureValues(), added code to compute how close we came to
//                            tripping for all the events (before just used LD_Bucket)
//   98     231017  DAH - Revised EventManager() to update binary and PSC status before taking trip
//                        snapshots to ensure the status reflects any trip event that occurred in a
//                        protection subroutine that is run in the sampling interrupt
//  133     231219  DAH - In EM_TRIP of EventManager(), added code to compute one-cycle currents and save
//                        them as the snapshot values.  On a cold start, EventManager() could be called
//                        before the one-cycle currents have been computed.  This ensures the one-cycle
//                        currents are valid and are the most recent values.
//                        Also added code to check whether in one-cycle values have ever been computed.  If
//                        they have not, these values are set to 0.
//  149     240131  DAH - Removed code that initializes the demand logging EID from Event_VarInit().  The
//                        demand logging EID is initialized to the power-up EID in main
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


//------------------------------------------ Events Organization -------------------------------------------
//
// The occurrence of an event causes data to be captured, time-stamped, and stored.  When an event occurs,
// the occurrence is captured by placing the time stamp, event code, and EID in a FIFO (function
// InsertNewEvent()).  Additional flags may be set to initiate additional actions such as a waveform
// capture.
// Events are processed in the order they are received (function EventManager()). Processing an event
// basically means capturing data (snapshots of real-time values and sometimes waveforms), and writing this
// data into FRAM or Flash memory.
// Events data is organized by event logs.  An event log (or simply "log") is not the same as an event.  An
// event causes entries to be made in one or more event logs.  For example, a Trip event causes entries to
// be made in the Summary, Trip, and Trip Waveforms Logs.  There are 9 types of event logs.  Each Event Log
// is managed independently of the other logs.
//   Type               Max Number                      Stored in       Comment
//   Summary            SUMMARY_NUM_LOGS (500)          FRAM
//   Time Adjustment    TIMEADJ_NUM_LOGS (50)           FRAM
//   Trip               TRIP_NUM_LOGS (200)             FRAM            Snapshots of real-time values
//   Test Trip          TESTTRIP_NUM_LOGS (20)          FRAM            Snapshots of real-time values
//   Alarm              ALARM_NUM_LOGS (200)            FRAM            Snapshots of real-time values
//   Energy & Demand    12,960 packets                  Flash           45 days' worth at 5-minute intervals
//   Trip Waveform      NUM_TRIP_WAVEFORMS (21)         Flash           Waveforms
//   Alarm Waveforms    NUM_ALARM_WAVEFORMS (21)        Flash           Waveforms
//   Extended-Capture WFs   NUM_EXTCAP_WAVEFORMS (7)    Flash           Waveform
// The Event Types are defined in file Events_def.h (enum EVENT_TYPES).  The "Max Number" constants (listed
// below as MAX_NUM_EVENTS[]) are defined in file FRAM_Flash_def.h.  These are used to reserve space in FRAM
// and Flash for the log entries.
// Note, the event types numbered from 1 - 9, not 0 - 8, so that they match up with the processor-processor
// communications buffer id and buffer info numbers.
//
// Events Storage - Summary, Time Adjustment, Trip, Test Trip, Alarm:
//   Event logs are stored in FRAM:
//          Base Address - FRAM_EV_INFO[EVENT_TYPE]
//          Offset Address - struct EV_ADD.NextEvntNdx * EV_SIZE[EVENT_TYPE]
//          Number of logs and rollover indication - struct EV_ADD.Num_Events
//              (if Num_Events  > MAX_NUM_EVENTS[EVENT_TYPE], rollover has occurred)
//     Example - Summary Log (special case because index and number of logs are uint16's):
//        - the next open location to store a Summary Event is
//            FRAM_EV_INFO[EV_TYPE_SUMMARY] +  EV_Sum.NextEvntNdx * EV_SIZE[EV_TYPE_SUMMARY]
//        - the number of summary logs is
//            EV_Sum.Num_Events
//        - the earliest summary log is
//            if (EV_Sum.Num_Events < MAX_NUM_EVENTS[EV_TYPE_SUMMARY]),
//                then FRAM_EV_INFO[EV_TYPE_SUMMARY] + 0
//            else
//                     FRAM_EV_INFO[EV_TYPE_SUMMARY] + EV_Sum.NextEvntNdx * EV_SIZE[EV_TYPE_SUMMARY]
//        - the latest summary log is
//            FRAM_EV_INFO[EV_TYPE_SUMMARY] + (EV_Sum.NextEvntNdx-1) * EV_SIZE[EV_TYPE_SUMMARY]
//     Example - Other Logs (index and number of logs are uint8's), denoted by event_type:
//        - the next open location to store a non-Summary Event is
//            FRAM_EV_INFO[event_type] +  RAM_EV_ADD[event_type]->NextEvntNdx * EV_SIZE[event_type]
//        - the number of logs is
//            RAM_EV_ADD[event_type]->Num_Events
//        - the earliest log is
//            if (RAM_EV_ADD[event_type]->Num_Events < MAX_NUM_EVENTS[event_type]),
//                then FRAM_EV_INFO[event_type] + 0
//            else
//                     FRAM_EV_INFO[event_type] + RAM_EV_ADD[event_type]->NextEvntNdx * EV_SIZE[event_type]
//        - the latest summary log is
//            FRAM_EV_INFO[event_type] + (RAM_EV_ADD[event_type]->NextEvntNdx-1) * EV_SIZE[event_type]
//   The event storage address information (NextEvntNdx and Num_Events) are stored in duplicate copies in
//   FRAM.  They are read from FRAM and held in RAM after a reset.  The FRAM copies are updated whenever a
//   new log entry is made.  Two tables hold the address information:
//      RAM copy: RAM_EV_ADD[event_type]
//      FRAM copy: FRAM_EV_ADD[event_type]
//   The first 12 bytes of all of the events are the EID (4 bytes) followed by the Time Stamp (8 bytes).
//   When EID information is desired, it is merely read from the earliest and latest log entry.
//
// Events Storage - Energy and Demands:
//   Energy and Demand logs are stored in Flash:
//          Address - EV_Dmnd.NextDmndAdd
//             b15..5: Sector address in Flash (4Kbytes each - ENERGY_SECTOR_START to ENERGY_SECTOR_END)
//             b4..1: Page address in Flash (256 bytes each - 0x0 thru 0xF)
//             b0: 1/2 page address (128 bytes, 0 = bytes 0 thru 127, 1 = bytes 128 thru 255 in the page)
//          Offset Address - None
//          Number of logs and rollover indication - EV_Dmnd.Num_Entries
//   Each Energy and Demand log uses 1/2 page of Flash memory.  When an entry is made, if it is the for the
//   first half page (b0 = 0), the entry is stored in FRAM (location FRAM_DEMAND).  When the next entry is
//   made (b0 = 1), the first entry is retrieved from FRAM and the entire page is written to Flash.
//
// Events Storage - Trip, Alarm, and Extended-Capture Waveforms
//   Waveform log samples are stored in Flash:
//          Base Address - TRIP_WAVEFORMS_START, ALARM_WAVEFORMS_START, EXTCAP_WAVEFORM_START
//          Offset Address - struct FLASH_INT_REQ.NextEvntIndex * WF_SIZE_IN_SECTORS
//          Number of logs and rollover indication - struct FLASH_INT_REQ.Num_Events
//              (if Num_Events  > MAX_NUM_EVENTS[EVENT_TYPE], rollover has occurred)
//   Waveform header information is store in FRAM:
//          Base Address - FRAM_EV_INFO[EVENT_TYPE]
//          Offset Address - struct FLASH_INT_REQ.NextEvntIndex * WF_HEADER_SIZE
//          Number of logs and rollover indication - struct EV_ADD.Num_Events
//              (if Num_Events  > MAX_NUM_EVENTS[EVENT_TYPE], rollover has occurred)
//
//----------------------------------------End Events Organization ------------------------------------------


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
#include "string.h"
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Meter_def.h"
#include "Events_def.h"
#include "Demand_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "DispComm_def.h"
#include "Prot_def.h"



//
//      Local Definitions used in this module...
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
#include "Demand_ext.h"
#include "DispComm_ext.h"
#include "Meter_ext.h"
#include "Prot_ext.h"
#include "Events_ext.h"
#include "Intr_ext.h"



//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void Event_VarInit(void);
void EventManager(void);
void ClearEvents(void);
void ClearFRAM(void);
uint32_t InsertNewEvent(uint8_t EventType);
uint8_t GetEVInfo(uint8_t ev_type);
//void EventWrite(uint8_t EventType);
void EventSummaryWrite(void);
void EventTimeAdjWrite(void);
void EventTripWrite(void);
void EventTestTripWrite(void);
void EventExtCapWrite(void);
void EventDisturbanceWrite(void);
void EventSummaryRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num);
void EventDisturbanceRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs);
uint8_t EventGetWfEIDs(uint8_t evtype, uint32_t srch_eid, uint8_t *match_ndx);
void EventSnapshotSummaryRead(uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs, uint16_t bid);
void EventSnapshotRead(uint8_t *bufinfoptr, uint16_t bid);

int16_t BinarySearch(uint16_t FirstIndex, uint16_t LastIndex, uint32_t ValueSearched, uint8_t EventType);
int16_t EventLookUp(uint32_t EIDLookUp, uint8_t EventType);

//      Local Function Prototypes (These functions are called only within this module)
//
void EventAlarmWrite(void);
uint8_t ReadEVAddress(uint8_t ev_type, uint16_t *ev_index, uint16_t *ev_num_events);
uint32_t ReadEID(uint32_t fram_address);
void ClearLogFRAM(uint8_t EventType);
void ExtendedCapture(uint8_t TypeOfRecord);
void ExtendedCaptureValues(uint8_t TypeOfRecord);
void DisturbanceCapture(void);
void DisturbanceCaptureValues(struct DIST_VALUES *psDistValues, uint8_t proc_ndx);




//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
uint32_t EventMasterEID;
struct NEWEVENTFIFO NewEventFIFO[16];      // Note: If the size (16) is changed, the code that handles
                                           //   NewEventInNdx AND NewEventOutNdx must be changed accordingly
struct NEWEVENTFIFO IntEventBuf[8];        // This also applies to this array
uint8_t NewEventInNdx;
uint8_t IntEventInNdx, IntEventOutNdx, IntEventBufFull;
uint8_t Dist_GOOSE;

struct EV_SUM EV_Sum;
struct EV_ADD EV_TimeAdj;
struct EV_ADD EV_Trip;
struct EV_ADD EV_TestTrip;
struct EV_ADD EV_Alarm;
struct EV_ADD EV_Dist;
struct EV_ADD EV_ExtCap;
struct EV_DMND EV_Dmnd;
struct TIMEADJ TimeAdjust;
struct EXTCAP_SNAPSHOT_METER ExtCapSnapshotMeteredValues;
struct EV_ADD EV_ExtCap;
struct INTERNAL_TIME SnapshotTime, ExtCapSnapshotTime;
struct EXCTAP_STATE sExtCapState;

//struct DIST_CAPTURE sDisturbanceCaptureVal;
struct EV_ADD EV_Disturbance;

struct DIST_VALUES sListOfDist[DIST_NUM];

uint8_t ExtCap_ReqFlag;
uint8_t ExtCap_AckFlag;

uint8_t Goose_Capture_Code;

uint32_t Dist_Flag;
uint8_t Dist_Flag_SD;
uint8_t Dist_Flag_GF;
uint32_t Dist_Flag_Cancel;
uint8_t Dist_Flag_Cancel_SD;
uint8_t Dist_Flag_Cancel_GF;

float* const pDistValues[] =
{
  &CurOneCycMax,      &THSensor.Temperature, &Vll_max,            &Vll_min,        &VolUnbalTot,
  &CurUnbalTot,       &MaxPwrOneCycRevW,     &MaxPwrOneCycRevVar, &CurUnbalTot,    &FreqLine.FreqVal,
  &FreqLine.FreqVal,  &MaxPwrOneCycW,        &MaxPwrOneCycVar,    &MaxPwrOneCycVA, &MinProtPF,
  &CurOneCycMax,      &CurOneCycMax,         &CurOneCycMax,       &CurOneCycIg,    &CurOneCycMax
};

// Note, the causes shown below MUST align with the flags in Events_def.h!
const uint8_t aDistCause[] =
{
  LDPU_EXIT,           HIGH_TEMPERATURE_EXIT, OVERVOLTAGE_EXIT,   UNDERVOLTAGE_EXIT, VOLTAGE_UNBAL_EXIT,
  CURRENT_UNBAL_EXIT,  REVERSE_KW_EXIT,       REVERSE_KVAR_EXIT,  PHASE_LOSS_EXIT,   OVERFREQUENCY_EXIT,
  UNDERFREQUENCY_EXIT, OVERKW_EXIT,           OVERKVAR_EXIT,      OVERKVA_EXIT,      UNDERPF_EXIT,
  HIGHLOAD1_EXIT,      HIGHLOAD2_EXIT,        GOOSE_CAPTURE_EXIT, GND_FAULT_EXIT,    SDPU_EXIT
};

const uint8_t aExtCapCause[] =
{
  EXTCAP_GOOSE_CAPTURE_ENTRY, EXTCAP_OV_ENTRY, EXTCAP_UV_ENTRY, EXTCAP_HIGHLOAD1_ENTRY,
  EXTCAP_HIGHLOAD2_ENTRY
};

// Constant Array of RAM addresses that hold the event index and number of events
//   Note, these must align with the EVENT_TYPES definitions in Events_def.h!!
struct EV_ADD* const RAM_EV_ADD[] =
{
    0,                                  // Not used
    0,                                  // Summary handled separately since it has uint16_t values
    &EV_TimeAdj,
    &EV_Trip,
    &EV_TestTrip,
    &EV_Alarm,
    0,                                  // Demand/Energy handled separately since it has uint16_t values
    &Trip_WF_Capture.EV_Add,
    &Alarm_WF_Capture.EV_Add,
    &Ext_WF_Capture.EV_Add,
    &EV_Dist,
    &EV_ExtCap
};

// Constant array of FRAM addresses that hold the event index and number of events
//   Note, these must align with the EVENT_TYPES definitions in Events_def.h!!
const uint32_t FRAM_EV_ADD[] =
{
    0,                                  // Not used
    SUMMARY_LOG_ADD,
    TIMEADJ_LOG_ADD,
    TRIP_LOG_ADD,
    TESTTRIP_LOG_ADD,
    ALARM_LOG_ADD,
    DMND_LOG_ADDR,
    TRIP_WF_ADD,
    ALARM_WF_ADD,
    EXTCAP_WF_ADD,
    DIST_LOG_ADD,
    EXCAP_LOG_ADD
};

// Array of FRAM addresses that hold the event info (for waveforms, the event header info)
//   Note, these must align with the EVENT_TYPES definitions in Events_def.h!!
const uint32_t FRAM_EV_INFO[] =
{
    0,                                  // Not used
    SUMMARY_LOG_START,
    TIMEADJ_LOG_START,
    TRIP_LOG_START,
    TESTTRIP_LOG_START,
    ALARM_LOG_START,
    0,                                  // Demand logs are stored in Flash
    TRIP_WF_INFO,                       // This is the waveform header info
    ALARM_WF_INFO,
    EXTCAP_WF_INFO,
    DIST_LOG_START,
    EXTCAP_LOG_START
};

// Array of constants that are the event log entry sizes in bytes
//   Note, these must align with the EVENT_TYPES definitions in Events_def.h!!
const uint32_t EV_SIZE[] =
{
    0,                                  // Not used
    SUMMARY_EVENT_SIZE,
    TIMEADJ_EVENT_SIZE,
    SNAPSHOTS_EVENT_SIZE,               // Trip snapshot logs
    SNAPSHOTS_EVENT_SIZE,               // Test Trip snapshot logs
    SNAPSHOTS_EVENT_SIZE,               // Alarm snapshot logs
    0,                                  // Demand logs are stored in Flash
    0,                                  // Trip waveforms are stored in Flash
    0,                                  // Alarm waveforms are stored in Flash
    0,                                  // Extended capture waveforms are stored in Flash
    DIST_EVENT_SIZE,
    SNAPSHOTS_EVENT_SIZE                // Extended Capture snapshot logs
};

// Array of constants that are the maximum number of events that can be stored
//   Note, these must align with the EVENT_TYPES definitions in Events_def.h!!
const uint16_t MAX_NUM_EVENTS[] =
{
    0,                                  // Not used
    SUMMARY_NUM_LOGS,
    TIMEADJ_NUM_LOGS,
    TRIP_NUM_LOGS,
    TESTTRIP_NUM_LOGS,
    ALARM_NUM_LOGS,
    DMND_NUM_ENTRIES,
    NUM_TRIP_WAVEFORMS,
    NUM_ALARM_WAVEFORMS,
    NUM_EXTCAP_WAVEFORMS,
    DIST_NUM_LOGS,
    EXTCAP_NUM_LOGS
};


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t EventState, EV_WfState;
uint8_t NewEventOutNdx;

//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       Event_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Module Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used in the Event Module.
//                      The following variables do not need initialization:
//                          NewEventFIFO[]
//
//  CAVEATS:            Call only during initialization or through SPI1_Flash_Manager()
//                      This must be called after IO_VarInit() because IO_VarInit() clears SPI1Flash.Req,
//                      and this subroutine may set the S1F_DMND_ERASE bit.
//
//  INPUTS:             None
//
//  OUTPUTS:            NewEventInNdx, NewEventOutNdx, IntEventInNdx, IntEventOutNdx, IntEventBufFull,
//                      EventState, Alarm_WF_Capture.Req, Alarm_WF_Capture.InProg, Trip_WF_Capture.Req,
//                      Trip_WF_Capture.InProg, Ext_WF_Capture.Req, Ext_WF_Capture.InProg, EventMasterEID,
//                      EV_Dmnd.NextDmndAdd, EV_Dmnd.Num_Entries, SPI1Flash.Req (S1F_DMND_ERASE),
//                      SystemFlags, EV_Sum.NextEvntNdx, EV_Sum.Num_Events,
//                      Alarm_WF_Capture.EV_Add.NextEvntNdx, Alarm_WF_Capture.EV_Add.Num_Events,
//                      Trip_WF_Capture.EV_Add.NextEvntNdx, Trip_WF_Capture.EV_Add.Num_Events,
//                      Chart_WF_Capture.EV_Add.NextEvntNdx, Chart_WF_Capture.EV_Add.Num_Events,
//                      SPI1Flash.Req (S1F_ALARM_WF_ERASE + S1F_TRIP_WF_ERASE + S1F_EXT_WF_ERASE),
//                      EV_WfState, Goose_Capture_Code
//
//  ALTERS:             None
//
//  CALLS:              FRAM_Read(), ReadEVAddress()
//
//  EXECUTION TIME:     Measured on 230323 (rev 0.70 code): 183usec
//
//------------------------------------------------------------------------------------------------------------

void Event_VarInit(void)
{
  uint16_t t_nxt_index, t_num_events;
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;
  NewEventInNdx = 0;
  NewEventOutNdx = 0;
  IntEventInNdx = 0;
  IntEventOutNdx = 0;
  IntEventBufFull = 0;
  EventState = 0;
  EV_WfState = 0;

  ExtCap_ReqFlag = 0;
  ExtCap_AckFlag = 0;

  Goose_Capture_Code = DS_LAST + 1;

//  memset(&sDisturbanceCaptureVal, 0, sizeof(sDisturbanceCaptureVal));
  memset(&sListOfDist, 0, sizeof(sListOfDist));
  memset(&sExtCapState, 0, sizeof(sExtCapState));
  Dist_Flag = 0;
  Dist_Flag_SD = FALSE;
  Dist_Flag_GF = FALSE;
  Dist_Flag_Cancel = 0;
  Dist_Flag_Cancel_SD = FALSE;
  Dist_Flag_Cancel_GF = FALSE;
  
  Alarm_WF_Capture.Req = FALSE;
  Alarm_WF_Capture.InProg = FALSE;
  Trip_WF_Capture.Req = FALSE;
  Trip_WF_Capture.InProg = FALSE;
  Ext_WF_Capture.Req = FALSE;
  Ext_WF_Capture.InProg = FALSE;

  // Read the Master EID and complement from FRAM and store in a temporary
  FRAM_Read(MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  // If the complement isn't correct, read the second copy
  if (uval.u32[0] != (~uval.u32[1]))
  {
    FRAM_Read((MASTER_EID+SECONDBLK_OFFSET), 4, (uint16_t *)(&uval.u32[0]));
    // If the second copy is also bad, set the temporary EID to 0
    // Note, don't need to log the error - it will be apparent from the EIDs that something got corrupted
    if (uval.u32[0] != (~uval.u32[1]))
    {
      uval.u32[0] = 0;
    }
  }
  // Set the Master EID to the value read from FRAM
  EventMasterEID = uval.u32[0];
/*  // Set the Demand Logging EID
  EngyDmnd[1].EID = EventMasterEID++;         // *** DAH  MAYBE DO THIS AFTER INITIALIZATION SO POWER-UP LOGS ARE FIRST
  // Save the  Master EID                     //          SHOULD THIS BE THE SAME AS THE POWER UP EID??
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = (~EventMasterEID);
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, (MASTER_EID+SECONDBLK_OFFSET), 4, (uint16_t *)(&uval.u32[0]));    */


  // Retrieve summary log indices from FRAM.  Checksum and complement should match
  FRAM_Read(SUMMARY_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_Sum.NextEvntNdx = uval.u16[0];
  }
  else                                     // *** DAH  ADD FRAM ERROR EVENT
  {
     EV_Sum.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_Sum.Num_Events = uval.u16[1];
  }
  else                                     // *** DAH  ADD FRAM ERROR EVENT
  {
     EV_Sum.Num_Events = 0;
  }

  FRAM_Read(TIMEADJ_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_TimeAdj.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_TimeAdj.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_TimeAdj.Num_Events = uval.u16[1];
  }
  else
  {
     EV_TimeAdj.Num_Events = 0;
  }

  FRAM_Read(TRIP_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_Trip.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_Trip.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_Trip.Num_Events = uval.u16[1];
  }
  else
  {
     EV_Trip.Num_Events = 0;
  }

  FRAM_Read(TESTTRIP_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_TestTrip.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_TestTrip.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_TestTrip.Num_Events = uval.u16[1];
  }
  else
  {
     EV_TestTrip.Num_Events = 0;
  }

  FRAM_Read(ALARM_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_Alarm.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_Alarm.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_Alarm.Num_Events = uval.u16[1];
  }
  else
  {
     EV_Alarm.Num_Events = 0;
  }

  FRAM_Read(EXCAP_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_ExtCap.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_ExtCap.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_ExtCap.Num_Events = uval.u16[1];
  }
  else
  {
     EV_ExtCap.Num_Events = 0;
  }

  FRAM_Read(DIST_LOG_ADD, 4, &uval.u16[0]);
  if (uval.u16[0] == (0xFFFF ^ uval.u16[2]))
  {
     EV_Dist.NextEvntNdx = uval.u16[0];
  }
  else
  {
     EV_Dist.NextEvntNdx = 0;
  }
  if (uval.u16[1] == (0xFFFF ^ uval.u16[3]))
  {
     EV_Dist.Num_Events = uval.u16[1];
  }
  else
  {
     EV_Dist.Num_Events = 0;
  }

  // Get the next open Demand Log entry address and the number of entries.  If there is an error, set a
  //   flag.  The subroutine already resets the values if there is an error.  Note, the values can go
  //   directly in the variables, since they are both uint16's.
  if (!ReadEVAddress(EV_TYPE_DEMAND, &EV_Dmnd.NextDmndAdd, &EV_Dmnd.Num_Entries))
  {
    EV_Dmnd.NextDmndAdd = (ENERGY_SECTOR_START << 5);
    EV_Dmnd.Num_Entries = 0;
    SystemFlags |= ALARM_WF_FRAM_ERR;
    // *** DAH  LOG THIS AS AN ERROR - MAYBE POWER UP - ERROR AND STORE STATUS FLAGS IN THE EVENT LOG
  }
  if ((EV_Dmnd.NextDmndAdd & 0x001F) == 0)          // If page and half-page address are zero, this is a new
  {                                                 //   sector.  Set request flag to erase this sector to
    SPI1Flash.Req |= S1F_DMND_ERASE;                //   ensure it can be written to correctly (cannot
  }                                                 //   guarantee it had been erased)
  // Get the next open Summary Log index and number of events.  If there is an error, set a flag.  The
  //   subroutine already resets the values if there is an error.  Note, the values can go directly in the
  //   variables, since they are both uint16's.
  if (!ReadEVAddress(EV_TYPE_SUMMARY, &EV_Sum.NextEvntNdx, &EV_Sum.Num_Events))
  {
    SystemFlags |= ALARM_WF_FRAM_ERR;
    // *** DAH  LOG THIS AS AN ERROR - MAYBE POWER UP - ERROR AND STORE STATUS FLAGS IN THE EVENT LOG
  }
  // Get the next open Alarm Waveform index and the number of Alarm Waveform captures.  Note, the values
  //   must go through uint16_t temporaries first before being saved as uint8_t's.
  if (!ReadEVAddress(EV_TYPE_ALARMWF, &t_nxt_index, &t_num_events))
  {
    SystemFlags |= ALARM_WF_FRAM_ERR;
    // *** DAH  LOG THIS AS AN ERROR - MAYBE POWER UP - ERROR AND STORE STATUS FLAGS IN THE EVENT LOG
  }
  Alarm_WF_Capture.EV_Add.NextEvntNdx = (uint8_t)(t_nxt_index);
  Alarm_WF_Capture.EV_Add.Num_Events = (uint8_t)t_num_events;
  // Get the next open Trip Waveform index and the number of Trip Waveform captures.  Note, the values must
  //   go through uint16_t temporaries first before being saved as uint8_t's.
  if (!ReadEVAddress(EV_TYPE_TRIPWF, &t_nxt_index, &t_num_events))
  {
    SystemFlags |= TRIP_WF_FRAM_ERR;
    // *** DAH  LOG THIS AS AN ERROR - MAYBE POWER UP - ERROR AND STORE STATUS FLAGS IN THE EVENT LOG
  }
  Trip_WF_Capture.EV_Add.NextEvntNdx = (uint8_t)(t_nxt_index);
  Trip_WF_Capture.EV_Add.Num_Events = (uint8_t)t_num_events;
  // Get the next open Extended Waveform index and the number of Extended Waveform captures.  Note, the
  //   values must go through uint16_t temporaries first before being saved as uint8_t's.
  if (!ReadEVAddress(EV_TYPE_EXTCAPWF, &t_nxt_index, &t_num_events))
  {
    SystemFlags |= EXTCAP_WF_FRAM_ERR;
    // *** DAH  LOG THIS AS AN ERROR - MAYBE POWER UP - ERROR AND STORE STATUS FLAGS IN THE EVENT LOG
  }
  Ext_WF_Capture.EV_Add.NextEvntNdx = (uint8_t)(t_nxt_index);
  Ext_WF_Capture.EV_Add.Num_Events = (uint8_t)t_num_events;
  // Set flags to erase the next open trip, alarm, and extended-capture waveform storage blocks to ensure
  //   they are ready for the next capture
  SPI1Flash.Req |= (S1F_ALARM_WF_ERASE + S1F_TRIP_WF_ERASE + S1F_EXT_WF_ERASE);

  Dist_GOOSE = 0;

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         Event_VarInit()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION    ReadEVAddress()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read Event FRAM Address    
//                      
//  MECHANICS:          This subroutine reads the event index (index of the location to store the next
//                      event) and the number of events from FRAM.
//                      The complements of the two values are read also and compared to the values.  In
//                      addition, the integrity of the values are also checked by performing a range check
//                      and also comparing the two values to each other.
//                      The subroutine returns True if the values are good; False otherwise.
//                      The function also returns the two values in ev_index and ev_num_events
//                          
//  CAVEATS:            This subroutine assumes SPI2 is free for use
//                      It is assumed this subroutine is called during initialization
//                      The subroutine sets the index and number of events to 0 if there is an error.  It is
//                      up to the calling subroutine to change the default values if necessary!!
//                      
//  INPUTS:             ev_type - Code indicating the event type (Summary, ..., Strip-Chart Waveform)
//                      
//  OUTPUTS:            ev_index - the index of the next open spot for a waveform (0 if invalid)
//                      ev_num_events - number of events (capped at MAX_NUM_EVENTS[ev_type]) (0 if invalid)
//                      Also, the subroutine returns True if the values read are valid.  The suroutine
//                      returns False otherwise
//                      
//  ALTERS:             None
//                      
//  CALLS:              FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t ReadEVAddress(uint8_t ev_type, uint16_t *ev_index, uint16_t *ev_num_events)
{
  uint8_t ok, i;
  uint32_t temp[2];
  uint32_t fram_add;

  fram_add = FRAM_EV_ADD[ev_type];
  *ev_index = 0;                        // Assume bad FRAM values: initialize index and num captures to 0
  *ev_num_events = 0;                   // Note, initialized values are for ev_type != EV_TYPE_DEMAND

  for (i=0; i<2; ++i)
  {
    // Get the next open event index and the number of events
    FRAM_Read(fram_add, 4, (uint16_t *)(&temp[0]));
    // If the complement is incorrect, set error flag
    if (temp[0] != (0xFFFFFFFF ^ temp[1]))
    {
      ok = FALSE;
    }
    else
    {
      temp[1] = (temp[0] >> 16);          // temp[1] = num events (demand num entries)
      temp[0] &= 0x0000FFFF;              // temp[0] = index (demand address)
      // Integrity check - NextEvntIndex and Num_Events must be in range
      if (ev_type == EV_TYPE_DEMAND)                            // For demand address and number of entries:
      // Number of entries must be less than or equal to the maximum number of entries
      // Address must be between ENERGY_SECTOR_START and ENERGY_SECTOR_END
      {
        if ( ((uint16_t)temp[1] > DMND_NUM_ENTRIES)
          || (((uint16_t)temp[0] >> 5) < ENERGY_SECTOR_START)
          || (((uint16_t)temp[0] >> 5) > ENERGY_SECTOR_END) )
        {
          ok = FALSE;
        }
        else                              // Everything is ok, so save values and break out in case it was
        {                                 //   the first pass
          *ev_index = (uint16_t)temp[0];
          *ev_num_events = (uint16_t)temp[1];
          ok = TRUE;
          break;
        }
      }
      else                                                      // For all other event types:
      {
        // Index must be less than the maximum number of events
        // Number of events must be less than or equal to the maximum number of events
        // If number of events is less than the maximum, index must equal the number of events
        // If bad, set error flag
        if ( ((uint16_t)temp[0] > (MAX_NUM_EVENTS[ev_type] - 1))
          || ((uint16_t)temp[1] > MAX_NUM_EVENTS[ev_type])
          || (((uint16_t)temp[1] < MAX_NUM_EVENTS[ev_type]) && ((uint16_t)temp[0] != (uint16_t)temp[1])) )
        {
          ok = FALSE;
        }
        else                              // Everything is ok, so save values and break out in case it was
        {                                 //   the first pass
          *ev_index = (uint16_t)temp[0];
          *ev_num_events = (uint16_t)temp[1];
          ok = TRUE;
          break;
        }
      }
    }
    fram_add += SECONDBLK_OFFSET;       // First copy is bad.  Set up to read second copy
  }
  return (ok);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION      ReadEVAddress()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventManager()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Top Level Event Manager Subroutine
//
//  MECHANICS:          This subroutine is called from the main loop.  It is the top-level events processor.
//                      It checks the New Event FIFO (Event.NewDesc[]) for new events.  If there is an event
//                      in the FIFO, it is processed as follows:
//                        - Assemble the corresponding event "package" and set the flag to store it (in FRAM
//                          or Flash
//                        - Remove the event from the New Event FIFO.
//                      Event packages consist of:
//                        - Time Stamp
//                        - Event ID
//                        - Event Data (if required at the time of insertion)
//                      If a waveform capture is part of the event data, the capture waveform flag is set.
//                      The event manager allocates the space in the buffer, and completes the rest of the
//                      event package.  It also initializes the pointers to insert the waveform data.  The
//                      Event Manager inserts the data into the event package when the capture has been
//                      completed.
//                      The New Event FIFO contains the Event Type, Time Stamp, and the new time in the case
//                      of Time Adjustment Events.  The New Event FIFO does not contain the Event ID.  It is
//                      not necessary, as the order of events is inherent to the FIFO.
//                      Event ID Handling:
//                      The Master Event ID is read from FRAM on power up, and a working copy is kept in
//                      RAM.  This working copy is incremented each time a new event is transferred from the
//                      New Event FIFO into the FRAM buffer.  The Master EID in FRAM is also updated at this
//                      time.
//                      The Pending Event List (Event.PendingList[]) is a list indicating the pending events
//                      that are in FRAM, and the order they are to be transmitted.  The list can hold up to
//                      10 entries - one for each event.  The list is a FIFO.  The entries are the Event Type
//                      (Event.NewDesc[].Type).  A value of NO_EVENT (0) indicates an open entry.
//                      The Pending Event List is examined before the next event in the New Event FIFO is
//                      transferred in, to ensure that there is not an entry of this type already in FRAM.
//                      If there is not, the entry is then made at the next open location.  When an event in
//                      the Pending Event List has been communicated to the CAM and Display Modules, it is
//                      removed from the list.
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
//------------------------------------------------------------------------------------------------------------

void EventManager(void)
{
  uint8_t em_exit;
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  em_exit = FALSE;

  // Call InsertNewEvent() with no event to check for events that occurred during the interrupts.
  InsertNewEvent(NO_EVENT);

  
  while (!em_exit)
  {
    switch (EventState)
    {
      case EM_IDLE:                     // Idle State
        // Check new events FIFO for new events
        if (NewEventInNdx != NewEventOutNdx)    // If input index not equal to output index, there is
        {                                           //   an event in the new events FIFO
          // Set the next state to process the event, according to the event type
          // The defined event types in NewEventFIFO[] are the actual "case" labels
          if(NewEventFIFO[NewEventOutNdx].Code <= SUMMARY_END)
          {
            EventState = EM_SUMMARY;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code <= TRIPS_END)
          {
            EventState = EM_TRIP;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code <= ALARMS_END)
          {
            EventState = EM_ALARM;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code <= EXTCAP_END)
          {
            EventState = EM_EXT_CAPT;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code <= DISTCAP_END)
          {
            EventState = EM_DISTURBANCE;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code == SUMMARY_DEMAND)     // Only one code for demand
          {
            EventState = EM_ENERGYLOG;
            break;
          }
          else if (NewEventFIFO[NewEventOutNdx].Code <= SUMMARY_RTD)
          {
            EventState = EM_RTD;
            break;
          }

          else                                                              // Otherwise if not found, jump
          {                                                                 //   to the Finish state so the
            EventState = EM_FINISH;                                         //   event is removed from the
            break;                                                          //   queue.
          }                                                                 // Note, this should never occur
        }
        else                                         // If input index equals output index, no new events,
        {                                            //   so remain in the idle state and exit
          em_exit = TRUE;
        }
        break;


      case EM_SUMMARY:
        EventSummaryWrite();
        // If the event code is SDPU entry, save the EID for the disturbance capture
        if (NewEventFIFO[NewEventOutNdx].Code == SDPU_ENTRY)
        {
          sListOfDist[DS_SD_PICKUP].EIDofStartEvnt = NewEventFIFO[NewEventOutNdx].EID;
        }
        EventState = EM_FINISH;
        break;


      case EM_RTD:
        EventSummaryWrite();
        EventState = EM_FINISH;
        break;


      case EM_ALARM:
        EventSummaryWrite();
        SnapshotMeter(NewEventFIFO[NewEventOutNdx].Code);
        EventAlarmWrite();
        // Save the EID of the event that was processed in case it is needed for waveform processing
        //   Note, it is assumed that this state will execute before the waveform capture is completed and
        //   the EID is written into the header file.  This should be the case.  Event if the waveform is
        //   aborted, the event has been inserted into the buffer and should be processed
        Alarm_WF_Capture.EID = NewEventFIFO[NewEventOutNdx].EID;
        // Also save it for the disturbance capture if it is a GF event
        if (NewEventFIFO[NewEventOutNdx].Code == ALARM_GROUND_FAULT)
        {
          sListOfDist[DS_GND_FAULT].EIDofStartEvnt = NewEventFIFO[NewEventOutNdx].EID;
        }
        EventState = EM_FINISH;
        break;


      case EM_ENERGYLOG:                // Energy and Demand Logging - request storage
        // EV_Dmnd.NextDmndAdd holds the next open address of the demand/energy log.  Each log entry takes
        //   1/2 of a page of Flash.  The mapping between EV_Dmnd.NextDmndAdd and the Flash is as follows
        //     b15..5: Sector address in Flash (4Kbytes each - ENERGY_SECTOR_START to ENERGY_SECTOR_END)
        //     b4..1: Page address in Flash (256 bytes each - 0x0 thru 0xF)
        //     b0: 1/2 page address (128 bytes, 0 = bytes 0 thru 127, 1 = bytes 128 thru 255 in the page)
        if ((EV_Dmnd.NextDmndAdd & 0x0001) == 0x0000)       // If writing the first 128 bytes, store in FRAM
        {                                                   //   and increment the address
          FRAM_Write(DEV_FRAM2, FRAM_DEMAND, (DEMAND_SIZE >> 1), (uint16_t *)(&EngyDmnd[1].EID));
          ++EV_Dmnd.NextDmndAdd;                            // This increment only sets the LSB
          ++EV_Dmnd.Num_Entries;                            // Increment the number of entries
          // Save the demand address and number of entries in FRAM
          //   Need to save values and complement.  In addition, use two separate writes to ensure at
          //   least one set is always valid.
          uval.u32[0] = EV_Dmnd.NextDmndAdd + (((uint32_t)EV_Dmnd.Num_Entries) << 16);
          uval.u32[1] = (0xFFFFFFFF ^ uval.u32[0]);
          FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR, 4, (uint16_t *)(&uval.u32[0]));
          FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));
          EventState = EM_FINISH;                           // Jump to finish
        }
        else                                                // If writing the second 128 bytes, first get
        {                                                   //   the first 128 bytes from FRAM
          FRAM_Read(FRAM_DEMAND, (DEMAND_SIZE >> 1), (uint16_t *)(&EngyDmnd[0].EID));
          SPI1Flash.Req |= S1F_DMND_WR;                     // Set flag to store the energy log
          EventState = EM_ENERGYLOG1;
          em_exit = TRUE;
        }
        break;  

      case EM_ENERGYLOG1:               // Energy and Demand Logging 1 - wait for storage completion
        if (SPI1Flash.Ack & S1F_DMND_WR)    // If storage has been completed, clear the request flag and
        {                                   //   update the logging address
          SPI1Flash.Req &= (uint32_t)(~S1F_DMND_WR);
          // Increment the logging address and the number of entries.  Check for rollover 
          ++EV_Dmnd.NextDmndAdd;
          ++EV_Dmnd.Num_Entries;
          if ((EV_Dmnd.NextDmndAdd >> 5) >= ENERGY_SECTOR_END)   // If we have rolled over, reset the
          {                                                      //   address
            EV_Dmnd.NextDmndAdd = (ENERGY_SECTOR_START << 5);
          }
          if ((EV_Dmnd.NextDmndAdd & 0x001F) == 0)               // If address has rolled into a new sector,
          {                                                      //   set the request flag to erase it, set
            SPI1Flash.Req |= S1F_DMND_ERASE;                     //   the state to wait for the erase to
            EventState = EM_ENERGYLOG2;                          //   complete, and exit
            em_exit = TRUE;
            if (EV_Dmnd.Num_Entries > (DMND_NUM_ENTRIES - 32))   // If the number of entries is greater than
            {                                                    //   25 sector's worth, reset it to 25
              EV_Dmnd.Num_Entries = (DMND_NUM_ENTRIES - 32);     //   sector's worth because the extra
            }                                                    //   sector is going to be erased
          }
          else                                                   // If we are still writing to the sector,
          {                                                      //   we are done so jump to finish
            EventState = EM_FINISH;
          }
          // Save the demand address and number of entries in FRAM
          //   Need to save values and complement.  In addition, use two separate writes to ensure at
          //   least one set is always valid.
          uval.u32[0] = EV_Dmnd.NextDmndAdd + (((uint32_t)EV_Dmnd.Num_Entries) << 16);
          uval.u32[1] = (0xFFFFFFFF ^ uval.u32[0]);
          FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR, 4, (uint16_t *)(&uval.u32[0]));
          FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));
        }
        else                                    // If storage is not complete remain in this state and exit
        {
          em_exit = TRUE;
        }
        break;

      case EM_ENERGYLOG2:               // Energy and Demand Logging 2 - wait for erase to complete
        // Req flag may also be cleared in ManageSPI1Flags() - this is necessary because the request flag
        //   is set in initialization and we will not wait to clear it, so it is cleared automatically.
        //   If it is cleared in ManageSPI1Flags(), the Ack flag will get cleared in SPI1_Flash_Manager().
        // So check two conditions for erase being completed: Ack set, or Ack and Req clear
        if ( (SPI1Flash.Ack & S1F_DMND_ERASE)       // If the sector erase has been completed, clear the
          || (!(SPI1Flash.Req & S1F_DMND_ERASE)) )  //   request flag and jump to finish
        {
          SPI1Flash.Req &= (uint32_t)(~S1F_DMND_ERASE);
          EventState = EM_FINISH;
        }
        else                                   // Otherwise remain in this state and exit
        {
          em_exit = TRUE;
        }
        break;

      case EM_TRIP:                     // Trip
        // Call subroutines to update the status before doing the snapshot.  This is necessary because the
        //   instantaneous and SD protection are run in the sampling interrupt and the status may not have
        //   been updated in the main loop.
        // Note, two separate "if" statements are used, as opposed to a single "if" statement with an "or"
        //   of the two functions, because we want both subroutines to be called.  If an "or" statement is
        //   used, the second subroutine will not be called if the first one returns True
        if (Update_Std_Status(&TU_BinStatus))
        {
          DPComm.RTD_XmitTimer[0] = 0;
        }
        if (Update_PSC(&StatusCode))
        {
          DPComm.RTD_XmitTimer[0] = 0;
        }
        SnapshotMeter(NewEventFIFO[NewEventOutNdx].Code);
        // Overwrite current values with most recent one-cycle currents if it is an instantaneous or short
        //   delay trip.  This is necessary because these trip events occur in the sampling interrupt, and
        //   so the event may be processed before Calc_Prot_Current() is called on the one-cycle
        //   anniversary, especially on a cold start.  The currents may not be up-to-date.
        if ( (NewEventFIFO[NewEventOutNdx].Code == TRIP_INSTANTANEOUS)
          || (NewEventFIFO[NewEventOutNdx].Code == TRIP_SHORT_DELAY)
          || (NewEventFIFO[NewEventOutNdx].Code == TRIP_GROUND_FAULT) )
        {
          SnapshotMeteredValues.Ia_Rms = EventCurOneCyc.Ia;
          SnapshotMeteredValues.Ib_Rms = EventCurOneCyc.Ib;
          SnapshotMeteredValues.Ic_Rms = EventCurOneCyc.Ic;
          SnapshotMeteredValues.In_Rms = EventCurOneCyc.In;
          SnapshotMeteredValues.Ig_Rms = EventCurOneCycIg;
          if ( (NewEventFIFO[NewEventOutNdx].Code == TRIP_GROUND_FAULT) && (SystemFlags & VALONECYC) )
          {
            SnapshotMeteredValues.Ig_Rms = CurOneCycIg;
          }
        }
        // If one-cycle values have not been computed yet, put zeros in any non-initialized values
        if (!(SystemFlags & VALONECYC))
        {
          SnapshotMeteredValues.Van1_Rms = 0;
          SnapshotMeteredValues.Vbn1_Rms = 0;
          SnapshotMeteredValues.Vcn1_Rms = 0;
          SnapshotMeteredValues.Vab1_Rms = 0;
          SnapshotMeteredValues.Vbc1_Rms = 0;
          SnapshotMeteredValues.Vca1_Rms = 0;
          SnapshotMeteredValues.Vab2_Rms = 0;
          SnapshotMeteredValues.Vbc2_Rms = 0;
          SnapshotMeteredValues.Vca2_Rms = 0;
          SnapshotMeteredValues.Van2_Rms = 0;
          SnapshotMeteredValues.Vbn2_Rms = 0;
          SnapshotMeteredValues.Vcn2_Rms = 0;
          SnapshotMeteredValues.P_real = 0;
          SnapshotMeteredValues.P_react = 0;
          SnapshotMeteredValues.P_apparent = 0;
        }

        EventSummaryWrite();
        EventTripWrite();
//        sExtCapState.State = EC_CANCEL;
        if (Dist_Flag)                                // *** DAH CANCEL FLAG NEEDS TO BE ON AN INDIVIDUAL
        {                                             //         FUNCTION BASIS, BECAUSE WE WILL CANCEL FOR
          Dist_Flag_Cancel = DS_ALL;                  //         OTHER REASONS ALSO
        }
        // Save the EID of the event that was processed in case it is needed for waveform processing
        //   Note, it is assumed that this state will execute before the waveform capture is completed and
        //   the EID is written into the header file
        Trip_WF_Capture.EID = NewEventFIFO[NewEventOutNdx].EID;    
        EventState = EM_FINISH;
        break;

      case EM_EXT_CAPT:                                      // Extended Capture
        if (NewEventFIFO[NewEventOutNdx].Code <= EXTCAP_HIGHLOAD2_ENTRY)
        {
          SnapshotMeter(NewEventFIFO[NewEventOutNdx].Code);
          EventExtCapWrite();
        }
        EventSummaryWrite();
        // If the event code matches the code for the extended captures, save the EID
        if (NewEventFIFO[NewEventOutNdx].Code == sExtCapState.Cause)
        {
          sExtCapState.EID = NewEventFIFO[NewEventOutNdx].EID;
        }
        // Save the EID of the event that was processed in case it is needed for waveform processing
        //   Note, it is assumed that this state will execute before the waveform capture is completed and
        //   the EID is written into the header file.  This should be the case.  Event if the waveform is
        //   aborted, the event has been inserted into the buffer and should be processed
        Ext_WF_Capture.EID = NewEventFIFO[NewEventOutNdx].EID;
        EventState = EM_FINISH;
        break;

      case EM_DISTURBANCE:
//        sDisturbanceCaptureVal.EID = NewEventFIFO[NewEventOutNdx].EID;
//        sDisturbanceCaptureVal.sExitTS = NewEventFIFO[NewEventOutNdx].TS;
        EventSummaryWrite();
//        EventState = EM_DISTURBANCE1;
//      break;
          
//      case EM_DISTURBANCE1:
        EventDisturbanceWrite();
        sListOfDist[NewEventFIFO[NewEventOutNdx].Code - DIST_EVCODE_START].Spare = 0;
        EventState = EM_FINISH;
      break;

      case EM_FINISH:                   // Common Code to Finish Processing an Event
        // Update the output index for the events FIFO
        ++NewEventOutNdx;
        NewEventOutNdx &= 0x0F;                 // *** DAH THIS RELIES ON NEWEVENTFIFO[] SIZE = 16  CHANGE WHEN FINALIZED
        EventState = EM_IDLE;
        em_exit = TRUE;
        break;
        
      default:
        EventState = EM_IDLE;
        em_exit = TRUE;
      break;
    
    }
  }

  // Event waveform processing
  // This is separate from the other event processing, so that while the waveform is being captured, other
  //   events may be processed
  switch (EV_WfState)
  {
    case 0:
    default:
      if ((Trip_WF_Capture.Req) && (Trip_WF_Capture.InProg))          // If this is a new trip waveform
      {                                                               //   capture, set the next state and
        EV_WfState = EM_TRIP1;                                        //   clear the request
        Trip_WF_Capture.Req = FALSE;
      }
      else if ((Alarm_WF_Capture.Req) && (Alarm_WF_Capture.InProg))   // If this is a new alarm waveform
      {                                                               //   capture, set the next state and
        EV_WfState = EM_ALARM1;                                       //   clear the request
        Alarm_WF_Capture.Req = FALSE;
      }
      else if ((Ext_WF_Capture.Req) && (Ext_WF_Capture.InProg))       // If this is a new ec waveform
      {                                                               //   capture, set the next state and
        EV_WfState = EM_EXT_CAPT1;                                    //   clear the request
        Ext_WF_Capture.Req = FALSE;                                            // *** DAH  WE NEED TO INITIALIZE A TIMER HERE SO WE DON'T HANG IF THE CAPTURE NEVER COMPLETES
      }                                                                        //          SINCE TRIP CAPTURE IS 36 CYCLES (.6 SECONDS), TIMER SHOULD BE ABOUT 700MSEC
      break;

    case EM_TRIP1:                    // Trip With Waveform Capture Event
      // Wait for the waveform capture to complete - trip captures do not get aborted
      if (SPI1Flash.Ack & S1F_TRIP_WF_WR)
      {
        SPI1Flash.Req &= (uint32_t)(~S1F_TRIP_WF_WR);     // Clear the waveform write request flag 
        EV_WfState = EM_TRIP2;
      }
      else                             // *** DAH  CHECK TIMER - IF EXPIRED, SET ABORT FLAGS AND QUIT
      {
        break;
      }

    case EM_TRIP2:                    // Trip With Waveform Capture Event
      // Compute the starting address in FRAM for the waveform capture header info and write the info
      uval.u32[0] = FRAM_EV_INFO[EV_TYPE_TRIPWF] +
                          ((uint32_t)Trip_WF_Capture.EV_Add.NextEvntNdx * WF_HEADER_SIZE);
      FRAM_Write(DEV_FRAM2, uval.u32[0], (WF_HEADER_SIZE/2), (uint16_t *)(&Trip_WF_Capture.NumSamples));
      // Increment the capture index
      if (++Trip_WF_Capture.EV_Add.NextEvntNdx >= MAX_NUM_EVENTS[EV_TYPE_TRIPWF])
      {
        Trip_WF_Capture.EV_Add.NextEvntNdx = 0;
      }
      // Increment the number of captures - this method also makes sure number doesn't exceed the maximum
      //   number of captures
      Trip_WF_Capture.EV_Add.Num_Events =
              ((Trip_WF_Capture.EV_Add.Num_Events < MAX_NUM_EVENTS[EV_TYPE_TRIPWF]) ?
                    (Trip_WF_Capture.EV_Add.Num_Events + 1) : (MAX_NUM_EVENTS[EV_TYPE_TRIPWF]));
      // Save the capture index and number of captures in FRAM
      //   Need to save values and complement.  In addition, use two separate writes to ensure at
      //   least one set is always valid.
      uval.u32[0] = Trip_WF_Capture.EV_Add.NextEvntNdx +
                          (((uint32_t)Trip_WF_Capture.EV_Add.Num_Events) << 16);
      uval.u32[1] = (0xFFFFFFFF ^ uval.u32[0]);
      FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_TRIPWF], 4, (uint16_t *)(&uval.u32[0]));
      FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_TRIPWF]+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));
      EV_WfState = 0;
      break;

    case EM_ALARM1:
      // We are finished if the Ack flag is set or if the capture is no longer in progress
      // If Ack flag not set, we may still be done - it may have been aborted and the Ack flag cleared in
      //   SPI1_Flash_Manager() before we got here.  In this case, the capture will no longer be in progress
      if ( (SPI1Flash.Ack & S1F_ALARM_WF_WR) || (!Alarm_WF_Capture.InProg) )
      {
        SPI1Flash.Req &= (uint32_t)(~S1F_ALARM_WF_WR);
        EV_WfState = EM_ALARM2;
      }
      else
      {
        break;
      }

    case EM_ALARM2:
      // Compute the starting address in FRAM for the waveform capture header info and write the info
      uval.u32[0] = FRAM_EV_INFO[EV_TYPE_ALARMWF] +
                          ((uint32_t)Alarm_WF_Capture.EV_Add.NextEvntNdx * WF_HEADER_SIZE);
      FRAM_Write(DEV_FRAM2, uval.u32[0], (WF_HEADER_SIZE/2), (uint16_t *)(&Alarm_WF_Capture.NumSamples));
      // Increment the capture index
      if (++Alarm_WF_Capture.EV_Add.NextEvntNdx >= MAX_NUM_EVENTS[EV_TYPE_ALARMWF])
      {
        Alarm_WF_Capture.EV_Add.NextEvntNdx = 0;
      }
      // Increment the number of captures - this method also makes sure number doesn't exceed the maximum
      //   number of captures
      Alarm_WF_Capture.EV_Add.Num_Events =
              ((Alarm_WF_Capture.EV_Add.Num_Events < MAX_NUM_EVENTS[EV_TYPE_ALARMWF]) ?
                    (Alarm_WF_Capture.EV_Add.Num_Events + 1) : (MAX_NUM_EVENTS[EV_TYPE_ALARMWF]));
      // Save the capture index and number of captures in FRAM
      //   Need to save values and complement.  In addition, use two separate writes to ensure at
      //   least one set is always valid.
      uval.u32[0] = Alarm_WF_Capture.EV_Add.NextEvntNdx +
                          (((uint32_t)Alarm_WF_Capture.EV_Add.Num_Events) << 16);
      uval.u32[1] = (0xFFFFFFFF ^ uval.u32[0]);
      FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_ALARMWF], 4, (uint16_t *)(&uval.u32[0]));
      FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_ALARMWF]+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));
      EV_WfState = 0;
      break;

      case EM_EXT_CAPT1:
        // We are finished if the Ack flag is set or if the capture is no longer in progress
        // If Ack flag not set, we may still be done - it may have been aborted and the Ack flag cleared in
        //   SPI1_Flash_Manager() before we got here.  In this case, the capture will no longer be in
        //   progress
        if ( (SPI1Flash.Ack & S1F_EXT_WF_WR) || (!Ext_WF_Capture.InProg) )
        {
          SPI1Flash.Req &= (uint32_t)(~S1F_EXT_WF_WR);
          EV_WfState = EM_EXT_CAPT2;
        }
        else
        {
          break;
        }

      case EM_EXT_CAPT2:
        // Compute the starting address in FRAM for the waveform capture header info and write the info
        uval.u32[0] = FRAM_EV_INFO[EV_TYPE_EXTCAPWF] +
                            ((uint32_t)Ext_WF_Capture.EV_Add.NextEvntNdx * WF_HEADER_SIZE);
        FRAM_Write(DEV_FRAM2, uval.u32[0], (WF_HEADER_SIZE/2), (uint16_t *)(&Ext_WF_Capture.NumSamples));
        // Increment the capture index
        if (++Ext_WF_Capture.EV_Add.NextEvntNdx >= MAX_NUM_EVENTS[EV_TYPE_EXTCAPWF])
        {
          Ext_WF_Capture.EV_Add.NextEvntNdx = 0;
        }
        // Increment the number of captures - this method also makes sure number doesn't exceed the maximum
        //   number of captures
        Ext_WF_Capture.EV_Add.Num_Events =
                ( (Ext_WF_Capture.EV_Add.Num_Events < MAX_NUM_EVENTS[EV_TYPE_EXTCAPWF]) ?
                        (Ext_WF_Capture.EV_Add.Num_Events + 1) : (MAX_NUM_EVENTS[EV_TYPE_EXTCAPWF]) );
        // Save the capture index and number of captures in FRAM
        //   Need to save values and complement.  In addition, use two separate writes to ensure at
        //   least one set is always valid.
        uval.u32[0] = Ext_WF_Capture.EV_Add.NextEvntNdx + (((uint32_t)Ext_WF_Capture.EV_Add.Num_Events) << 16);
        uval.u32[1] = (0xFFFFFFFF ^ uval.u32[0]);
        FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_EXTCAPWF], 4, (uint16_t *)(&uval.u32[0]));
        FRAM_Write(DEV_FRAM2, FRAM_EV_ADD[EV_TYPE_EXTCAPWF]+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));
        EV_WfState = 0;
        break;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventManager()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ClearEvents()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Clear Events
//
//  MECHANICS:          Clears all events in RAM, FRAM, and Flash
//
//  CAVEATS:            Called only through the Test Port in the factory
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID, EV_Dmnd.NextDmndAdd, SPI1Flash.Req, SPI1Flash.Ack, S2NF.Req,
//                      S2NF.Ack
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write(), Event_VarInit(), Dmnd_VarInit(), IO_VarInit()
//
//  *** DAH - CHECK THE TIMING ON THIS - IT APPEARS TO TAKE A VERY LONG TIME TO EXECUTE!!!!
// 
//------------------------------------------------------------------------------------------------------------

void ClearEvents(void)
{
  uint32_t uval[2];

//  EventMasterEID = 0;
  // Initialize master EID to 1.  The EIDs in the logs are all set to 0.  This is used to indicate that
  //   the log is empty
  EventMasterEID = 1;

  // Save the  Master EID
  uval[0] = EventMasterEID;
  uval[1] = (~EventMasterEID);
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, (MASTER_EID+SECONDBLK_OFFSET), 4, (uint16_t *)(&uval[0]));

  // Clear out energy and demand
  EV_Dmnd.NextDmndAdd = (ENERGY_SECTOR_START << 5);
  EV_Dmnd.Num_Entries = 0;
  uval[0] = EV_Dmnd.NextDmndAdd + (((uint32_t)EV_Dmnd.Num_Entries) << 16);
  uval[1] = (0xFFFFFFFF ^ uval[0]);
  FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, DMND_LOG_ADDR+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval[0]));
  Dmnd_VarInit();
  // For demands, still need to set the request flag to erase the next sector and to clear any pending
  //   request to write a new demand.  This must be done after IO_VarInit(), so it is done at the end of
  //   this subroutine.  IO_VarInit() clears all of the request flags

  // Clear out trip, alarm, and strip-chart waveforms by setting Num_Events and NextEvntIndex to 0
  Trip_WF_Capture.EV_Add.Num_Events = 0;
  Trip_WF_Capture.EV_Add.NextEvntNdx = 0;
  Alarm_WF_Capture.EV_Add.Num_Events = 0;
  Alarm_WF_Capture.EV_Add.NextEvntNdx = 0;
  Ext_WF_Capture.EV_Add.Num_Events = 0;
  Ext_WF_Capture.EV_Add.NextEvntNdx = 0;

  ExtCap_ReqFlag = 0;
  ExtCap_AckFlag = 0;
  sExtCapState.State = EC_IDLE;


//  memset(&sDisturbanceCaptureVal, 0, sizeof(sDisturbanceCaptureVal));
  memset(&sListOfDist, 0, sizeof(sListOfDist));
  memset(&sExtCapState, 0, sizeof(sExtCapState));
  Dist_Flag = 0;
  Dist_Flag_SD = FALSE;
  Dist_Flag_GF = FALSE;
  Dist_Flag_Cancel = DS_ALL;
  Dist_Flag_Cancel_SD = FALSE;
  Dist_Flag_Cancel_GF = FALSE;

  EV_Sum.NextEvntNdx = 0;
  EV_Sum.Num_Events = 0;
  EV_TimeAdj.NextEvntNdx = 0;
  EV_TimeAdj.Num_Events = 0;
  EV_Trip.NextEvntNdx = 0;
  EV_Trip.Num_Events = 0;
  EV_TestTrip.NextEvntNdx = 0;
  EV_TestTrip.Num_Events = 0;
  EV_Alarm.NextEvntNdx = 0;
  EV_Alarm.Num_Events = 0;
  EV_Dist.NextEvntNdx = 0;
  EV_Dist.Num_Events = 0;
  EV_ExtCap.NextEvntNdx = 0;
  EV_ExtCap.Num_Events = 0;

  uval[0] = 0x00000000;
  uval[1] = 0xFFFFFFFF;
  FRAM_Write(DEV_FRAM2, TRIP_WF_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, ALARM_WF_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, EXTCAP_WF_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, SUMMARY_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, TIMEADJ_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, TRIP_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, TESTTRIP_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, ALARM_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, DIST_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  FRAM_Write(DEV_FRAM2, EXCAP_LOG_ADD, 4, (uint16_t *)(&uval[0]));
  ExtCapt_FRAM_Write(EXTCAP_EID_START, (sizeof(sExtCapState.EID) >> 1), (uint16_t *)(&uval[0]));

  IO_VarInit();                                 // Clear SPI1Flash request and ack flags

  Event_VarInit();                              // Set the flags to erase the waveform blocks
  SPI1Flash.Req |= S1F_DMND_ERASE;              // Set request flag to erase the next demand sector

}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ClearEvents()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       InsertNewEvent()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Insert New Event into the Event FIFO
//
//  MECHANICS:          This subroutine updates the New Event FIFO with pending events.
//                      First, the subroutine places any events that are in the interrupt event buffer.
//                      This buffer contains events that were detected in the interrupt service routines.
//                      These events occurred between calls to this subroutine.
//                      After the interrupt events have been stored, the subroutine stores the event that is
//                      passed in through the parameters.
//                      Note, events are stored only if there is room in the New Event FIFO.
//
//  CAVEATS:            1) It is assumed the New Event FIFO is large enough that it won't be full, although
//                         it is always checked as a precaution.
//                      2) This subroutine should be called at least once each time through the main loop,
//                         even if there are no pending foreground events, to ensure that interrupt events
//                         are processed.  If there are no pending events, NO_EVENT is passed as the event
//                         type.
//
//  INPUTS:             EventType - the event type
//                      SystemFlags (EVENT_FF_FULL), IntEventInNdx, IntEventOutNdx, IntEventBufFull,
//                      NewEventOutNdx, IntEventBuf[].xxx
// 
//  OUTPUTS:            Returns the EID of the inserted event
//                      NewEventFIFO[].xxx
//
//  ALTERS:             SystemFlags (EVENT_FF_FULL), NewEventInNdx, IntEventOutNdx
// 
//  CALLS:              Get_DateTime()
// 
//------------------------------------------------------------------------------------------------------------

uint32_t InsertNewEvent(uint8_t EventType)
{
//  struct INTERNAL_TIME presenttime;
  uint32_t ret_val;

  // Check the interrupt event buffer for events that occurred in the interrupts.  If there are any, insert
  //   them into the new event FIFO                                    *** DAH  MAY NEED DEDICATED BUFFERS FOR INTERRUPT EVENTS.  IF USE FIFO, MAY NEED TO DISABLE INTERRUPTS WHEN CHANGING THE POINTERS OR THE FULL FLAG
  //                                                                            HAVE TO LOOK INTO THIS FURTHER (MAYBE INPTR AND FULL FLAG ONLY CHANGED BY INTERRUPTS, OUTPTR ONLY CHANGED BY FOREGROUND)
  while ( (!(SystemFlags & EVENT_FF_FULL)) && ((IntEventInNdx != IntEventOutNdx) || IntEventBufFull) )
  {
    NewEventFIFO[NewEventInNdx].Code = IntEventBuf[IntEventOutNdx].Code;
    NewEventFIFO[NewEventInNdx].TS.Time_secs = IntEventBuf[IntEventOutNdx].TS.Time_secs;
    NewEventFIFO[NewEventInNdx].TS.Time_nsec = IntEventBuf[IntEventOutNdx++].TS.Time_nsec;
    NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
    NewEventInNdx &= 0x0F;
    if (NewEventInNdx == NewEventOutNdx)
    {
      SystemFlags |= EVENT_FF_FULL;
    }
    IntEventOutNdx &= 0x07;
  }

  // Now insert the event that caused this routine to be called - must recheck the event FIFO
  //   Only load the new event if the event FIFO is not full and the event is valid
  if ( (!(SystemFlags & EVENT_FF_FULL)) && (EventType > NO_EVENT) && (EventType <= MAXEVENTCODE) )
  {
    NewEventFIFO[NewEventInNdx].Code = EventType;
    __disable_irq();                                            // Get the present time for the time stamp
//    Get_InternalTime(&presenttime);
    Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
    __enable_irq();
//    NewEventFIFO[NewEventInNdx].TS.Time_secs = presenttime.Time_secs;
//    NewEventFIFO[NewEventInNdx].TS.Time_nsec = presenttime.Time_nsec;
    ret_val = EventMasterEID++;
    NewEventFIFO[NewEventInNdx++].EID = ret_val;
    NewEventInNdx &= 0x0F;
    if (NewEventInNdx == NewEventOutNdx)
    {
      SystemFlags |= EVENT_FF_FULL;
    }
  }
  return (ret_val);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         InsertNewEvent()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       GetEVInfo()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Event Info
//
//  MECHANICS:          This subroutine retrieves the earliest event ID, latest EID, and number of events
//                      for a specific log type.  The subroutine then assembles a Write Buffer Response
//                      or NAK message based on the info.
//
//  CAVEATS:            This subroutine assumes the request is for display processor communications
//                      It is assumed that the command, source/destination address, buffer type, and buffer
//                      ID have all been checked and decoded
//
//  INPUTS:             ev_type: Event Type
//                          1 - Summary
//                          2 - Time Adjustment
//                          3 - Trip
//                          4 - Test Trip
//                          5 - Alarm
//                          6 - Demand
//                          7 - Trip Waveform
//                          8 - Alarm Waveform
//                          9 - Extended Capture Waveform
//                         10 - Disturbance
//                         11 - Extended Capture
// 
//  OUTPUTS:            Returns True if the received message is valid; False otherwise
//                      If valid message received, returns the event header info as follows:
//                        SPI2_buf[0]     - Event Type
//                        SPI2_buf[1]     - Number of Events ls byte
//                        SPI2_buf[2]     - Number of Events ms byte
//                        SPI2_buf[3..6]  - Earliest Event ID (ls byte .. ms byte)
//                        SPI2_buf[7..10] - Latest Event ID (ls byte .. ms byte)
//
//  ALTERS:             SystemFlags (EVENT_FF_FULL), NewEventInNdx, IntEventOutNdx
// 
//  CALLS:              FRAM_Read(), AssembleAck1()
// 
//------------------------------------------------------------------------------------------------------------

uint8_t GetEVInfo(uint8_t ev_type)
{
  uint8_t i;

  // Check buffer info
  if ( (ev_type >= EV_TYPE_SUMMARY) && (ev_type <= EV_TYPE_LAST) )
  {
    // Retrieve the information and store it starting at SPI2_buf[0].  Information is:
    //   Event Type
    //   Number of events
    //   Earliest Event ID
    //   Latest Event ID
    // Note, the Summary and Demand Logs must be handled separately because their number of events are words
    //   The number of events for the remaining logs are bytes
    SPI2_buf[0] = ev_type;                             // Event Type
    if (ev_type == EV_TYPE_SUMMARY)
    {
      SPI2_buf[1] = EV_Sum.Num_Events;                            // Number of events low byte
      SPI2_buf[2] = (EV_Sum.Num_Events >> 8);                     // Number of events high byte
    }
    else if (ev_type == EV_TYPE_DEMAND)
    {
      SPI2_buf[1] = EV_Dmnd.Num_Entries;                          // Number of events low byte
      SPI2_buf[2] = (EV_Dmnd.Num_Entries >> 8);                   // Number of events high byte
    }
    // Waveform captures are a special case.  The number of waveforms available to the user is one less than
    //   the internal capacity, because the next location must be erased so it is ready to store the next
    //   capture.  Therefore, if the buffer is full, we must subtract one from the number of waveforms
    else if (EV_TYPE_TRIPWF <= ev_type <= EV_TYPE_EXTCAPWF)
    {
      SPI2_buf[1] = ( (RAM_EV_ADD[ev_type]->Num_Events < MAX_NUM_EVENTS[ev_type]) ?
                            (RAM_EV_ADD[ev_type]->Num_Events) : (MAX_NUM_EVENTS[ev_type] - 1) );
      SPI2_buf[2] = 0;
    }
    else
    {
      SPI2_buf[1] = RAM_EV_ADD[ev_type]->Num_Events;              // Number of events low byte
      SPI2_buf[2] = 0;                                            // Number of events high byte
    }
    // If no events, then set EIDs to 0
    if ( (SPI2_buf[1] == 0) && (SPI2_buf[2] == 0))
    {
      for (i=0; i<8; ++i)
      {
        SPI2_buf[i+13] = 0;
      }
    }
    // Otherwise retrieve the earliest and latest EIDs
    else
    {
      if (ev_type == EV_TYPE_SUMMARY)                     // If event type is Summary..
      {
        // If the number of events is less than maximum number of events, the earliest EID is at index=0 and
        //   the latest EID is at index = NextEvntNdx - 1.  Note, in this case, the number of events is
        //   greater than 0 and less than the max number of events for the summary log.  NextEvntNdx cannot
        //   be zero, because there is at least one log entry and it hasn't rolled over yet.
        if (EV_Sum.Num_Events < SUMMARY_NUM_LOGS)
        {
          FRAM_Read( (SUMMARY_LOG_START + 0), 2, (uint16_t *)(&SPI2_buf[3]) );
          FRAM_Read( (SUMMARY_LOG_START + ((EV_Sum.NextEvntNdx - 1) * SUMMARY_EVENT_SIZE) + 0),
                        2, (uint16_t *)(&SPI2_buf[7]) );
        }
        // Otherwise the number of events is equal to the maximum number of events, so the earliest EID is at
        //   index=NextEvntNdx, and the latest is at index=NextEvntNdx-1.  In this case, the log has rolled
        //   over, so we EV_Sum.NextEvntNdx can be 0.
        else
        {
          FRAM_Read( (SUMMARY_LOG_START + (EV_Sum.NextEvntNdx * SUMMARY_EVENT_SIZE) + 0), 2,
                            (uint16_t *)(&SPI2_buf[3]) );
          // If NextEvntNdx is > 0, most recent log is at NextEvntNdx - 1.
          if (EV_Sum.NextEvntNdx > 0)
          {
            FRAM_Read( (SUMMARY_LOG_START + ((EV_Sum.NextEvntNdx - 1) * SUMMARY_EVENT_SIZE) + 0), 2,
                            (uint16_t *)(&SPI2_buf[7]) );
          }
          // If NextEvntNdx = 0, most recent log is at (SUMMARY_NUM_LOGS - 1)
          else
          {
            FRAM_Read( (SUMMARY_LOG_START + ((SUMMARY_NUM_LOGS - 1) * SUMMARY_EVENT_SIZE) + 0), 2,
                            (uint16_t *)(&SPI2_buf[7]) );
          }
        }
      }
      else if (DPComm.RxMsgSav[8] == EV_TYPE_DEMAND)                 // If event type is Demand..
      {
        // Set the EID fields to 0.  EIDs don't have much meaning for Demand and Energy Logs and so no point
        //   in returning this (would have to retrieve from Flash) - just set to 0
        for (i=0; i<8; ++i)
        {
          SPI2_buf[i+13] = 0;
        }
      }                                                              // If event type is waveform capture...
      else if ( (ev_type >= EV_TYPE_TRIPWF) && (ev_type <= EV_TYPE_EXTCAPWF) )
      {
        // If waveform buffer not rolled over, can handle like other event buffers (earliest at 0, latest at
        //   NextEvntNdx - 1, no rollover concerns)
        if (RAM_EV_ADD[ev_type]->Num_Events < MAX_NUM_EVENTS[ev_type])
        {
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 2), 2, (uint16_t *)(&SPI2_buf[3]) );
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 2 +
                ((RAM_EV_ADD[ev_type]->NextEvntNdx - 1) * EV_SIZE[ev_type])),
                2, (uint16_t *)(&SPI2_buf[7]) );
        }
        // If buffer rolled over or full, NextEvntNdx is the next OPEN spot, NOT the earliest or latest
        //   spot.  Earliest is at NextEvntNdx + 1.  Latest is at NextEvntNdx - 1.  Must consider rollover
        //   when adding or adding or subtracting
        else
        {
          i = ( (RAM_EV_ADD[ev_type]->NextEvntNdx < (MAX_NUM_EVENTS[ev_type] - 1)) ?
                    (RAM_EV_ADD[ev_type]->NextEvntNdx + 1) : (0) );
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 2 + (i * EV_SIZE[ev_type])), 2, (uint16_t *)(&SPI2_buf[3]) );
          i = ( (RAM_EV_ADD[ev_type]->NextEvntNdx > 0) ?
                    (RAM_EV_ADD[ev_type]->NextEvntNdx - 1) : (MAX_NUM_EVENTS[ev_type] - 1) );
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 2 + (i * EV_SIZE[ev_type])), 2, (uint16_t *)(&SPI2_buf[7]) );
        }
      }
      // See comments above for summary log
      else                                                          // All other event types
      {
        if (RAM_EV_ADD[ev_type]->Num_Events < MAX_NUM_EVENTS[ev_type])
        {
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 0), 2, (uint16_t *)(&SPI2_buf[3]) );
          FRAM_Read( (FRAM_EV_INFO[ev_type] + 
                (((RAM_EV_ADD[ev_type]->NextEvntNdx - 1) * EV_SIZE[ev_type]) + 0)),
                2, (uint16_t *)(&SPI2_buf[7]) );
        }
        else
        {
          FRAM_Read( (FRAM_EV_INFO[ev_type] + ((RAM_EV_ADD[ev_type]->NextEvntNdx * EV_SIZE[ev_type]) + 0)),
                2, (uint16_t *)(&SPI2_buf[3]) );
          if (EV_Sum.NextEvntNdx > 0)
          {
            FRAM_Read( (FRAM_EV_INFO[ev_type] +
                  (((RAM_EV_ADD[ev_type]->NextEvntNdx - 1) * EV_SIZE[ev_type]) + 0)), 2,
                  (uint16_t *)(&SPI2_buf[7]) );
          }
          else
          {
            FRAM_Read( (FRAM_EV_INFO[ev_type] +
                  (((MAX_NUM_EVENTS[ev_type] - 1) * EV_SIZE[ev_type]) + 0)), 2,
                  (uint16_t *)(&SPI2_buf[7]) );
          }
        }
      }
    }
    return (TRUE);
  }
  // If the buffer info is invalid, return False
  else
  {
    return (FALSE);
  }
} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         GetEVInfo()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventSummaryWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Summary Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//  EXECUTION TIME:     Measured on 230519 (rev 40 code): 72usec (with interrupts disabled)
// 
//------------------------------------------------------------------------------------------------------------

void EventSummaryWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  uint16_t offset = 0;
  
  FRAM_Write(DEV_FRAM2, (SUMMARY_LOG_START + (EV_Sum.NextEvntNdx * SUMMARY_EVENT_SIZE)),
        (sizeof(NewEventFIFO[NewEventOutNdx].EID) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].EID);
  FRAM_Write(DEV_FRAM2, (SUMMARY_LOG_START + (EV_Sum.NextEvntNdx * SUMMARY_EVENT_SIZE) + offset),
        (sizeof(NewEventFIFO[NewEventOutNdx].TS) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].TS);
  FRAM_Write(DEV_FRAM2, (SUMMARY_LOG_START + (EV_Sum.NextEvntNdx * SUMMARY_EVENT_SIZE) + offset),
        (sizeof(NewEventFIFO[NewEventOutNdx].Code) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].Code));

  if (++EV_Sum.NextEvntNdx >= SUMMARY_NUM_LOGS)
  {
    EV_Sum.NextEvntNdx = 0;
  }
  EV_Sum.Num_Events = ((EV_Sum.Num_Events < SUMMARY_NUM_LOGS) ? (EV_Sum.Num_Events + 1) : SUMMARY_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_Sum.NextEvntNdx;
  uval.u16[1] = EV_Sum.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, SUMMARY_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventSummaryWrite()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventSummaryRead()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Summary Events
//
//  MECHANICS:          This subroutine retrieves up to 20 Summary Event logs, along with associated EID
//                      information
//
//  CAVEATS:            This subroutine assumes the request is from proc-proc comms.  As such, it examines
//                      the message length (msglen) to determine whether a starting index or starting EID
//                      was provided.  Any other use of this subroutine must provide the same input
//                      parameters in the same format
//
//  INPUTS:             msglen - the length of the proc-proc comms input message
//                          3 - search by index
//                          5 - search by EID
//                      bufinfoptr[] - the starting index or EID and number of events requested
//                          msglen = 3: bufinfoptr[1..0] = starting index
//                                      bufinfoptr[2] = number of events requested (20 max)
//                          msglen = 5: bufinfoptr[3..0] = starting EID
//                                      bufinfoptr[4] = number of events requested (20 max)
//                      *num - the number of requested event logs
// 
//  OUTPUTS:            *num - the number of returned event logs
//                      SPI2_buf[3..0]: EID of event log previous to the earliest returned event log (same
//                                      as EID of earliest returned event log if no previous log)
//                      SPI2_buf[7..4]: EID of event log after the most recent returned event log (same as
//                                      EID of most recent returned event log if no later log)
//                      SPI2_buf[((N-1)* 14 + 21).. 8]: Summary event logs (where N is the number of logs),
//                          if N > 0
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void EventSummaryRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num)
{
  uint32_t eid;
  uint16_t i, start_ndx, prev_ndx, next_ndx;

  if (EV_Sum.Num_Events == 0)           // If no events, done
  {
    *num = 0;
    return;
  }

  // Assemble the starting index for the events to retrieve, and the number of events to retrieve
  if (msglen == 3)                      // If message length is 3, retrieving events by index
  {
    start_ndx = (((uint16_t)bufinfoptr[1]) << 8) + bufinfoptr[0];   // Index of the most recent event
    *num = ( (bufinfoptr[2] <= 20) ? bufinfoptr[2] : 20);           // Number of events requested (max = 20)
  }
  else if (msglen == 5)                 // If message length is 5, retrieving events by EID
  {
    eid = (((uint32_t )bufinfoptr[3]) << 24) + (((uint32_t)bufinfoptr[2]) << 16)
                + (((uint32_t)bufinfoptr[1]) << 8) + (uint32_t)bufinfoptr[0];
    // Note: EventLookUp() returns -1 if event is not found.  This will be converted to 0xFFFF in start_ndx,
    //   which is greater than 500 (max value for Num_Events), and so routine will return with *num = 0
    start_ndx = (uint16_t)(EventLookUp(eid, EV_TYPE_SUMMARY));      // Index of the most recent event
    *num = ( (bufinfoptr[4] <= 20) ? bufinfoptr[5] : 20);           // Number of events requested (max = 20)
  }
  else                                  // Invalid message length - should never happen, return 0 logs
  {
    *num = 0;
    return;
  }

  if (start_ndx > (EV_Sum.Num_Events - 1))                          // If starting index > number of events,
  {                                                                 //   no events to return, so done
    *num = 0;
    return;
  }

  // Events are requested from the most recent event to the earliest event.  However, when reading them from
  //   FRAM, they are read from earliest event to most recent event.  Therefore, if the request is by index,
  //   we need to offset the requested starting index by the index of the last entry.  For example:
  //   Requested index    Internal log index (not rolled over)     Internal log index (rolled over)
  //        0                      NextEvntNdx - 1                                          
  //       20                      NextEvntNdx - 21
  //   If the log has not rolled over, we will clamp at index = 0, and reduce the number sent back
  //   accordingly.  If the log has rolled over, we need to consider this when doing the math

  // Check the parameters, and compute the previous and next indices
  //   prev_ndx: index of event before the earliest one we are sending back
  //   next_ndx: index of event after the latest one we are sending back
  //   There are two cases to handle, log not rolled over, and log rolled over

  // If the log has not rolled over,
  //     0 = index of the earliest entry
  //     NextEvntNdx - 1 = index of the latest entry
  //     Num_Events = NextEvntNdx = number of logs
  if (EV_Sum.Num_Events < SUMMARY_NUM_LOGS)   // First case: log not rolled over
  {
    // internal requested starting index is at index of most recent entry (NextEvntNdx - 1) minus the
    //   requested starting index.  Don't need to worry about rollover.  We already checked and ensured that
    //   start_ndx <= (Num_Events - 1), Num_Events = NextEvntNdx if not rolled over
    start_ndx = ((EV_Sum.NextEvntNdx - 1) - start_ndx);

    // First compute the index of the next log, i.e., the log after the requested log
    if (start_ndx == (EV_Sum.NextEvntNdx - 1))  // If requested log is the most recent log, the next index
    {                                           //   is index is the same index as the most recent log
      next_ndx = EV_Sum.NextEvntNdx - 1;        // Note, we don't need to worry about rollover, we already
    }                                           //   checked that Num_Events > 0
    else                                        // Otherwise add one.  We don't need to worry about rollover
    {                                           //   because NextEvntNdx < SUMMARY_NUM_LOGS - 1 
      next_ndx = start_ndx + 1;
    }
    // Now check whether the number of entries requested and starting index is in the range of available
    //   events
    if (*num > (start_ndx + 1))                 // If number requested greater than starting index + 1, we
    {                                           //   are requesting logs before the earliest entry, so clamp
      *num = start_ndx + 1;                     //   the number of logs requested
    }
    // Set start index to the first log to retrieve - we have to move start_ndx back by the number of logs
    //   to retrieve
    start_ndx = (start_ndx + 1) - *num;       // Set start index to the first log to retrieve
    // Finally, compute the index of the previous log, i.e., the log before the earliest one we are
    //   returning
    if (start_ndx == 0)                       // If the first log is the earliest entry in the log, the
    {                                         //   previous index is the same
      prev_ndx = 0;
    }
    else                                      // Otherwise subtract one.  We don't need to worry about
    {                                         //   rollover
      prev_ndx = start_ndx - 1;
    }
  }
  // If the log has rolled over,
  //     NextEvntNdx = index of the earliest entry
  //     NextEvntNdx - 1 = index of the latest entry
  //     Num_Events = SUMMARY_NUM_LOGS = log size
  else                                        // Second case: log rolled over
  {
    // internal requested starting index is at index of most recent entry (NxtEvntNdx - 1) minus the
    //   requested starting index.  We need to account for rollover
    //   First set i to the index of the most recent log, which is at NextEvntNdx - 1, accounting for
    //     rollover
    i = ( (EV_Sum.NextEvntNdx > 0) ? (EV_Sum.NextEvntNdx - 1) : (SUMMARY_NUM_LOGS - 1) );
    // Now subtract the requested starting index from i, accounting for rollover
    start_ndx = ( (i >= start_ndx) ? (i - start_ndx) : ((i + SUMMARY_NUM_LOGS) - start_ndx) );

    // First compute the index of the next log, i.e., the log after the requested log
    if (start_ndx == i)                      // If the requested log is the most recent log, the next index
    {                                        //   is the same
      next_ndx = start_ndx;
    }
    else                                     // Otherwise add one, accounting for rollover
    {
      next_ndx = ( (start_ndx == (SUMMARY_NUM_LOGS - 1)) ? (0) : (start_ndx + 1) );
    }
    // Now check whether the number of entries requested and starting index is in the range of available
    //   events
    // Compute number of logs from the earliest entry to the starting index
    i = ( (start_ndx >= EV_Sum.NextEvntNdx) ?
                ((start_ndx + 1) - EV_Sum.NextEvntNdx)
                                : ((start_ndx + 1 + SUMMARY_NUM_LOGS) - EV_Sum.NextEvntNdx) );
    if (*num > i)                             // If number requested exceeds the number of available
    {                                         //   previous entries, clamp the number of requested logs
      *num = i;
    }
    // Set start index to the first log to retrieve - we have to move start_ndx back by the number of logs
    //   to retrieve
    start_ndx = ( (*num <= (start_ndx + 1)) ?
                  ((start_ndx + 1) - *num) : ((start_ndx + 1 + SUMMARY_NUM_LOGS) - *num) );
    // Finally, compute the index of the previous log, i.e., the log before the earliest one we are
    //   returning
    if (start_ndx == EV_Sum.NextEvntNdx)      // If the first log is the earliest entry in the log, the
    {                                         //   previous index is the same
      prev_ndx = start_ndx;
    }
    else                                      // Otherwise subtract one, accounting for rollover
    {
      prev_ndx = ( (start_ndx == 0) ? (SUMMARY_NUM_LOGS - 1) : (start_ndx - 1) );
    }
  }

  // Get the previous and next EIDs 
  FRAM_Read( (SUMMARY_LOG_START + (prev_ndx * SUMMARY_EVENT_SIZE)), 2, (uint16_t *)(&SPI2_buf[0]) );
  FRAM_Read( (SUMMARY_LOG_START + (next_ndx * SUMMARY_EVENT_SIZE)), 2, (uint16_t *)(&SPI2_buf[4]) );

  // Retrieve the events
  FRAM_Read((SUMMARY_LOG_START + (start_ndx * SUMMARY_EVENT_SIZE)),
                    ((SUMMARY_EVENT_SIZE >> 1) * *num), (uint16_t *)(&SPI2_buf[8]));

} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventSummaryRead()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventSnapshotSummaryRead()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Snapshot Event Summaries
//
//  MECHANICS:          This subroutine retrieves up to 12 Snapshot Summaries
//                      Summary information is retrieved from either the Trip, Test Trip, Alarm, or Extended
//                      Capture Snapshot logs and consists of the
//                      following information:
//                          EID
//                          Timestamp
//                          Pri/Sec/Cause data object
//                          Associated summary event log code
//                      This summary information is retrieved by index, beginning with the most recent
//                      event, and then working backwards
//
//  CAVEATS:            This subroutine assumes the request is from proc-proc comms.  Any other use of this
//                      subroutine must provide the same input parameters in the same format
//
//  INPUTS:             bufinfoptr[] - the starting index and number of events requested
//                          bufinfoptr[1..0] = starting index
//                          bufinfoptr[2] = number of events requested (12 max)
//                      *num - the number of requested event logs
//                      bid - buffer id (DP_EVENT_TRIP_SUM <= bid <= DP_EVENT_EXTCAP_SUM)
// 
//  OUTPUTS:            *total_logs - total number of snapshot logs available
//                      *num - the number of returned event logs
//                      SPI2_buf[((N * 18) - 1).. 0]: Summary event logs (where N is the number of logs),
//                          if N > 0
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void EventSnapshotSummaryRead(uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs, uint16_t bid)
{
  uint8_t i, start_ndx, maxlogs;
  uint16_t j;
  uint32_t basefaddr;
  struct EV_ADD* snpsht_ptr;

  // Initialize address/number of events pointer.  Note, we already confirmed that
  //   DP_EVENT_TRIP_SUM <= bid <= DP_EVENT_EXTCAP_SUM
  if (bid == DP_EVENT_TRIP_SUM)
  {
    snpsht_ptr = &EV_Trip;
    basefaddr = TRIP_LOG_START;
    maxlogs = TRIP_NUM_LOGS;
  }
  else if (bid == DP_EVENT_TESTTRIP_SUM)
  {
    snpsht_ptr = &EV_TestTrip;
    basefaddr = TESTTRIP_LOG_START;
    maxlogs = TESTTRIP_NUM_LOGS;
  }
  else if (bid == DP_EVENT_ALARM_SUM)
  {
    snpsht_ptr = &EV_Alarm;
    basefaddr = ALARM_LOG_START;
    maxlogs = ALARM_NUM_LOGS;
  }
  else
  {
    snpsht_ptr = &EV_ExtCap;
    basefaddr = EXTCAP_LOG_START;
    maxlogs = EXTCAP_NUM_LOGS;
  }

  *total_logs = snpsht_ptr->Num_Events;    // Save the total number of logs
  if (*total_logs == 0)                    // If no events, done
  {
    *num = 0;
    return;
  }

  // Assemble the starting index for the events to retrieve, and the number of events to retrieve
  start_ndx = bufinfoptr[0];                              // 8 bit number, bufinfoptr[1] = 0
  *num = ( (bufinfoptr[2] <= 12) ? bufinfoptr[2] : 12);   // Number of events requested (max = 12)
  // Number of events requested clamped to the number of events in the log minus the starting index
  *num = ( (*num <= (*total_logs - start_ndx)) ? *num : (*total_logs - start_ndx) );

  if (start_ndx > (*total_logs - 1))                      // If starting index > number of events, no events
  {                                                       //   to return, so done
    *num = 0;
    return;
  }

  // Retrieve the Snapshot Summary info.  This Event info is requested from the most recent event to the
  //   earliest event.  However, it is stored in FRAM from the earliest event to the most recent event.
  //   We will start with the most recent event and work backwards.  We are retrieving the EID (4 bytes),
  //   Timestamp (8 bytes), Pri/Sec/Cause (4 bytes), and Summary Event Code (2 bytes), or 18 bytes of data
    
  // internal requested starting index is at the index of the most recent entry (NxtEvntNdx - 1) minus the
  //   requested starting index, accounting for rollover
  //   First set i to the index of the most recent log, which is at NextEvntNdx - 1, accounting for
  //     rollover
  i = ( (snpsht_ptr->NextEvntNdx > 0) ? (snpsht_ptr->NextEvntNdx - 1) : (maxlogs - 1) );
  // Now subtract the requested starting index from i, accounting for rollover
  start_ndx = ( (i >= start_ndx) ? (i - start_ndx) : (i + (maxlogs - start_ndx)) );

  j = 0;
  for (i = 0; i < *num; ++i)
  {
    // Retrieve the snapshot event summary info
    FRAM_Read( (basefaddr + (start_ndx * SNAPSHOTS_EVENT_SIZE)), 9, (uint16_t *)(&SPI2_buf[j]) );
    j += 18;
    start_ndx = ((start_ndx > 0) ? (start_ndx - 1) : (maxlogs - 1));
  }
} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventSnapshotSummaryRead()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventSnapshotRead()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Snapshot Event Summaries
//
//  MECHANICS:          This subroutine retrieve the Snapshot values for either a Trip, Test Trip, Alarm, or
//                      Extended Capture Event.  The Event Type is gleaned from the buffer ID (input bid),
//                      the requested event is noted from the Event ID, which is pointed to by *bufinfoptr
//
//  CAVEATS:            This snapshot information can only be retrieved by EID (not by index)
//                      This subroutine assumes the request is from proc-proc comms.  Any other use of this
//                      subroutine must provide the same input parameters in the same format
//
//  INPUTS:             bufinfoptr[] - the EID of the requested event
//                          bufinfoptr[3..0] = EID
//                          bufintoptr[4] = number requested.  This is ignored (1 is assumed)
//                      bid - buffer id (DP_EVENT_TRIP_SNAP <= bid <= DP_EVENT_EXTCAP_SNAP)
// 
//  OUTPUTS:            SPI2_buf[0] - the number of returned event logs (1 if event log is found, 0 if not)
//                      SPI2_buf[214 .. 1]: Snapshot event logs (valid data if SPI2_buf[0] = 1)
//
//  ALTERS:             None
// 
//  CALLS:              EventLookUp(), FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void EventSnapshotRead(uint8_t *bufinfoptr, uint16_t bid)
{
  uint8_t i;
  uint16_t ndx;
  uint32_t basefaddr, eid;
  struct EV_ADD* snpsht_ptr;

  // Initialize address/number of events pointer.  Note, we already confirmed that
  //   DP_EVENT_TRIP_SNAP <= bid <= DP_EVENT_EXTCAP_SNAP
  if (bid == DP_EVENT_TRIP_SNAP)
  {
    snpsht_ptr = &EV_Trip;
    basefaddr = TRIP_LOG_START;
    i = EV_TYPE_TRIP;
  }
  else if (bid == DP_EVENT_TESTTRIP_SNAP)
  {
    snpsht_ptr = &EV_TestTrip;
    basefaddr = TESTTRIP_LOG_START;
    i = EV_TYPE_TESTTRIP;
  }
  else if (bid == DP_EVENT_ALARM_SNAP)
  {
    snpsht_ptr = &EV_Alarm;
    basefaddr = ALARM_LOG_START;
    i = EV_TYPE_ALARM;
  }
  else
  {
    snpsht_ptr = &EV_ExtCap;
    basefaddr = EXTCAP_LOG_START;
    i = EV_TYPE_EXTCAP;
  }

  if (snpsht_ptr->Num_Events == 0)                  // If no events, done
  {
    SPI2_buf[0] = 0;
    return;
  }

  // There is at least one snapshot event stored.  Compute the EID and find the location index
  eid = (((uint32_t )bufinfoptr[3]) << 24) + (((uint32_t)bufinfoptr[2]) << 16)
                + (((uint32_t)bufinfoptr[1]) << 8) + (uint32_t)bufinfoptr[0];
  // Note: EventLookUp() returns -1 if event is not found.  This will be converted to 0xFFFF in start_ndx,
  //   which is greater than 500 (max value for Num_Events), and so routine will return with
  //   SPI2_buf[0] = 0 (the number of events returned)
  ndx = (uint16_t)(EventLookUp(eid, i));            // Index of the most recent event

  if (ndx == 0xFFFF)                                // If EID not found, no event to return, so done
  {
    SPI2_buf[0] = 0;
    return;
  }
  else                                              // Otherwise read the event
  {
    SPI2_buf[0] = 1;                                // Returning one event
    FRAM_Read( (basefaddr + (ndx * SNAPSHOTS_EVENT_SIZE)), (SNAPSHOTS_EVENT_SIZE >> 1),
                    (uint16_t *)(&SPI2_buf[1]) );
  }
} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventSnapshotRead()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventGetWfEIDs()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Waveform Event IDs
//
//  MECHANICS:          This subroutine retrieves the Event IDs for Trip, Alarm, or Extended Capture
//                      Waveform Capture events.  The EIDs are returned in SPI2_buf[] in chronological
//                      order:
//                          SPI2_buf[3..0] = Earliest EID
//                          SPI2_buf[n+3..n] = Most recent EID
//                      In addition, the subroutine checks whether any of the EIDs match a requested EID,
//                      entered as srch_eid.  If a match is found, the subroutine returns the index of the
//                      requested EID in *match_ndx, and the subroutine returns True.  If the EID is not
//                      found, the subroutine returns False and *match_ndx is undefined
//
//  CAVEATS:            evtype MUST be either EV_TYPE_TRIPWF, EV_TYPE_ALARMWF, or EV_TYPE_EXTCAPWF
//
//  INPUTS:             evtype - the Event Waveform Type (Trip, Alarm, or Extended Capture
//                      srch_eid - the EID of the requested waveform log entry
//                      RAM_EV_ADD[], MAX_NUM_EVENTS[]
// 
//  OUTPUTS:            The subroutine returns True if the requested EID is found, False otherwise
//                      *match_ndx - the index of the requested EID
//                      SPI2_buf[] - the EIDs contained in the log
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Read()
//
//  EXECUTION TIME:     Measured execution time on 230616: 101usec worst-case (includes interrupts) for
//                      extended-capture waveforms (7 waveforms)
// 
//------------------------------------------------------------------------------------------------------------

uint8_t EventGetWfEIDs(uint8_t evtype, uint32_t srch_eid, uint8_t *match_ndx)
{
  uint8_t num_events, i, j, ndx, ret_val;

  // Retrieve EIDs in chronological order
  // If waveform buffer not rolled over, earliest EID is at 0, latest EID is at NextEvntNdx - 1, no rollover
  //   concerns
  num_events = RAM_EV_ADD[evtype]->Num_Events;
  ndx = 0;
  ret_val = FALSE;
  if (num_events < MAX_NUM_EVENTS[evtype])
  {
    for (i = 0; i < num_events; ++i)
    {
      FRAM_Read( (FRAM_EV_INFO[evtype] + 2 + (i * WF_HEADER_SIZE)), 2, (uint16_t *)(&SPI2_buf[ndx]) );
      if ( (SPI2_buf[ndx] == (uint8_t)(srch_eid))
        && (SPI2_buf[ndx+1] == (uint8_t)(srch_eid >> 8))
        && (SPI2_buf[ndx+2] == (uint8_t)(srch_eid >> 16))
        && (SPI2_buf[ndx+3] == (uint8_t)(srch_eid >> 24)) )
      {
        *match_ndx = i;
        ret_val = TRUE;
      }
      ndx += 4;
    }
  }
  // If buffer rolled over or full, NextEvntNdx is the next OPEN spot, NOT the earliest or latest spot.
  //   Earliest is at NextEvntNdx + 1.  Must consider rollover when adding
  else
  {
    j = ( (RAM_EV_ADD[evtype]->NextEvntNdx < (MAX_NUM_EVENTS[evtype] - 1)) ?
                    (RAM_EV_ADD[evtype]->NextEvntNdx + 1) : (0) );
    for (i = 0; i < (num_events - 1); ++i)                          // num_events = MAX_NUM_EVENTS[evtype]
    {
      FRAM_Read( (FRAM_EV_INFO[evtype] + 2 + (j * WF_HEADER_SIZE)), 2, (uint16_t *)(&SPI2_buf[ndx]) );
      if ( (SPI2_buf[ndx] == (uint8_t)(srch_eid))
        && (SPI2_buf[ndx+1] == (uint8_t)(srch_eid >> 8))
        && (SPI2_buf[ndx+2] == (uint8_t)(srch_eid >> 16))
        && (SPI2_buf[ndx+3] == (uint8_t)(srch_eid >> 24)) )
      {
        *match_ndx = j;
        ret_val = TRUE;
      }
      ndx += 4;
      j = ( (j < (MAX_NUM_EVENTS[evtype] - 1)) ? (j + 1) : (0) );
    }
  }
  return (ret_val);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventGetWfEIDs()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventTimeAdjWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event time Adjust Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventTimeAdjWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  uint16_t offset = 0;
  
  FRAM_Write(DEV_FRAM2, (TIMEADJ_LOG_START + (EV_TimeAdj.NextEvntNdx * TIMEADJ_EVENT_SIZE)),
        (sizeof(NewEventFIFO[NewEventOutNdx].EID) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].EID);
  FRAM_Write(DEV_FRAM2, (TIMEADJ_LOG_START + (EV_TimeAdj.NextEvntNdx * TIMEADJ_EVENT_SIZE) + offset),
        (sizeof(NewEventFIFO[NewEventOutNdx].TS) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].TS);
  FRAM_Write(DEV_FRAM2, (TIMEADJ_LOG_START + (EV_TimeAdj.NextEvntNdx * TIMEADJ_EVENT_SIZE) + offset),
        (sizeof(TimeAdjust) >> 1), (uint16_t *)&TimeAdjust);

  if (++EV_TimeAdj.NextEvntNdx >= TIMEADJ_NUM_LOGS)
  {
    EV_TimeAdj.NextEvntNdx = 0;
  }
  EV_TimeAdj.Num_Events = ((EV_TimeAdj.Num_Events < TIMEADJ_NUM_LOGS) ? (EV_TimeAdj.Num_Events + 1) : TIMEADJ_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_TimeAdj.NextEvntNdx;
  uval.u16[1] = EV_TimeAdj.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, TIMEADJ_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventTimeAdjWrite()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventTripWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Trip Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventTripWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  uint16_t offset = 0;

  FRAM_Write(DEV_FRAM2, (TRIP_LOG_START + (EV_Trip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE)),
        (sizeof(NewEventFIFO[NewEventOutNdx].EID) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].EID);
  FRAM_Write(DEV_FRAM2, (TRIP_LOG_START + (EV_Trip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + offset),
        (sizeof(NewEventFIFO[NewEventOutNdx].TS) >> 1), (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  offset += sizeof(NewEventFIFO[NewEventOutNdx].TS);
  FRAM_Write(DEV_FRAM2, (TRIP_LOG_START + (EV_Trip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + offset),
        (SNAPSHOT_SIZE >> 1), (uint16_t *)(&SnapshotMeteredValues));

  if (++EV_Trip.NextEvntNdx >= TRIP_NUM_LOGS)
  {
    EV_Trip.NextEvntNdx = 0;
  }
  EV_Trip.Num_Events = ((EV_Trip.Num_Events < TRIP_NUM_LOGS) ? (EV_Trip.Num_Events + 1) : TRIP_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_Trip.NextEvntNdx;
  uval.u16[1] = EV_Trip.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, TRIP_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventTripWrite()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventTestTripWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Test Trip Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventTestTripWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  FRAM_Write(DEV_FRAM2, (TESTTRIP_LOG_START + (EV_TestTrip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE)), 2,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  FRAM_Write(DEV_FRAM2, (TESTTRIP_LOG_START + (EV_TestTrip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 4), 4,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  FRAM_Write(DEV_FRAM2, (TESTTRIP_LOG_START + (EV_TestTrip.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + (SNAPSHOT_SIZE  >> 1)),
                (SNAPSHOT_SIZE  >> 1), (uint16_t *)&SnapshotMeteredValues);

  if (++EV_TestTrip.NextEvntNdx >= TESTTRIP_NUM_LOGS)
  {
    EV_TestTrip.NextEvntNdx = 0;
  }
  EV_TestTrip.Num_Events = ((EV_TestTrip.Num_Events < TESTTRIP_NUM_LOGS) ? (EV_TestTrip.Num_Events + 1) : TESTTRIP_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_TestTrip.NextEvntNdx;
  uval.u16[1] = EV_TestTrip.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, TESTTRIP_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventTestTripWrite()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventAlarmWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Alarm Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventAlarmWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  FRAM_Write(DEV_FRAM2, (ALARM_LOG_START + (EV_Alarm.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 0), 2,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  FRAM_Write(DEV_FRAM2, (ALARM_LOG_START + (EV_Alarm.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 4), 4,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  FRAM_Write(DEV_FRAM2, (ALARM_LOG_START + (EV_Alarm.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 12),
                (SNAPSHOT_SIZE  >> 1), (uint16_t *)&SnapshotMeteredValues);

  if (++EV_Alarm.NextEvntNdx >= ALARM_NUM_LOGS)
  {
    EV_Alarm.NextEvntNdx = 0;
  }
  EV_Alarm.Num_Events = ((EV_Alarm.Num_Events < ALARM_NUM_LOGS) ? (EV_Alarm.Num_Events + 1) : ALARM_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_Alarm.NextEvntNdx;
  uval.u16[1] = EV_Alarm.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, ALARM_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventAlarmWrite()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventExtCapWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Extended Capture Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventExtCapWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  FRAM_Write(DEV_FRAM2, (EXTCAP_LOG_START + (EV_ExtCap.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 0), 2,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  FRAM_Write(DEV_FRAM2, (EXTCAP_LOG_START + (EV_ExtCap.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 4), 4,
                (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  FRAM_Write(DEV_FRAM2, (EXTCAP_LOG_START + (EV_ExtCap.NextEvntNdx * SNAPSHOTS_EVENT_SIZE) + 12),
                (SNAPSHOT_SIZE  >> 1), (uint16_t *)&SnapshotMeteredValues);

  if (++EV_ExtCap.NextEvntNdx >= EXTCAP_NUM_LOGS)
  {
    EV_ExtCap.NextEvntNdx = 0;
  }
  EV_ExtCap.Num_Events = ((EV_ExtCap.Num_Events < EXTCAP_NUM_LOGS) ? (EV_ExtCap.Num_Events + 1) : EXTCAP_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_ExtCap.NextEvntNdx;
  uval.u16[1] = EV_ExtCap.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, EXCAP_LOG_ADD, 4, &uval.u16[0]);

  // Update the master EID in FRAM
  uval.u32[0] = EventMasterEID;
  uval.u32[1] = ~EventMasterEID;
  FRAM_Write(DEV_FRAM2, MASTER_EID, 4, (uint16_t *)(&uval.u32[0]));
  FRAM_Write(DEV_FRAM2, MASTER_EID+SECONDBLK_OFFSET, 4, (uint16_t *)(&uval.u32[0]));

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventExtCapWrite()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventDisturbanceWrite()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Event Alarm Write
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             None
// 
//  OUTPUTS:            EventMasterEID
//
//  ALTERS:             None
// 
//  CALLS:              Fram_Write()
// 
//------------------------------------------------------------------------------------------------------------

void EventDisturbanceWrite()
{
  union dword_word
  {
    uint32_t u32[2];
    uint16_t u16[4];
  } uval;

  FRAM_Write(DEV_FRAM2, (DIST_LOG_START + (EV_Dist.NextEvntNdx * DIST_EVENT_SIZE) + 0),
        2, (uint16_t *)(&NewEventFIFO[NewEventOutNdx].EID));
  FRAM_Write(DEV_FRAM2, (DIST_LOG_START + (EV_Dist.NextEvntNdx * DIST_EVENT_SIZE) + 4),
        4, (uint16_t *)(&NewEventFIFO[NewEventOutNdx].TS));
  FRAM_Write(DEV_FRAM2, (DIST_LOG_START + (EV_Dist.NextEvntNdx * DIST_EVENT_SIZE) + 12),
        16, (uint16_t *)(&sListOfDist[NewEventFIFO[NewEventOutNdx].Code - DIST_EVCODE_START]));

  if (++EV_Dist.NextEvntNdx >= DIST_NUM_LOGS)
  {
    EV_Dist.NextEvntNdx = 0;
  }
  EV_Dist.Num_Events = ((EV_Dist.Num_Events < DIST_NUM_LOGS) ? (EV_Dist.Num_Events + 1) : DIST_NUM_LOGS);
  
  // Store index and number of events in FRAM
  uval.u16[0] = EV_Dist.NextEvntNdx;
  uval.u16[1] = EV_Dist.Num_Events;
  uval.u16[2] = (uval.u16[0] ^ 0xFFFF);
  uval.u16[3] = (uval.u16[1] ^ 0xFFFF);
  FRAM_Write(DEV_FRAM2, DIST_LOG_ADD, 4, &uval.u16[0]);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventDisturbanceWrite()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventDisturbanceRead()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Retrieve Disturbance Capture Events
//
//  MECHANICS:          This subroutine retrieves up to 5 Disturbance Capture Event logs, along with
//                      associated EID information
//
//  CAVEATS:            This subroutine assumes the request is from proc-proc comms.  As such, it examines
//                      the message length (msglen) to determine whether a starting index or starting EID
//                      was provided.  Any other use of this subroutine must provide the same input
//                      parameters in the same format
//
//  INPUTS:             msglen - the length of the proc-proc comms input message
//                          3 - search by index
//                          5 - search by EID
//                      bufinfoptr[] - the starting index or EID and number of events requested
//                          msglen = 3: bufinfoptr[1..0] = starting index
//                                      bufinfoptr[2] = number of events requested (5 max)
//                          msglen = 5: bufinfoptr[3..0] = starting EID
//                                      bufinfoptr[4] = number of events requested (5 max)
//                      *num - the number of requested event logs
// 
//  OUTPUTS:            *total_logs - the total number of disturbance logs
//                      *num - the number of returned event logs
//                      SPI2_buf[3..0]: EID of event log previous to the earliest returned event log (same
//                                      as EID of earliest returned event log if no previous log)
//                      SPI2_buf[7..4]: EID of event log after the most recent returned event log (same as
//                                      EID of most recent returned event log if no later log)
//                      SPI2_buf[((N-1)* 14 + 21).. 8]: Disturbance event logs (where N is the number of
//                          logs), if N > 0
//
//  ALTERS:             None
// 
//  CALLS:              EventLookUp(), FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

void EventDisturbanceRead(uint16_t msglen, uint8_t *bufinfoptr, uint8_t *num, uint8_t *total_logs)
{
  uint32_t eid;
  uint16_t i, start_ndx;

  *total_logs = EV_Dist.Num_Events;     // Save the total number of logs
  if (EV_Dist.Num_Events == 0)          // If no events, done
  {
    *num = 0;
    return;
  }

  // If message length is 3, retrieving events by index
  if (msglen == 3)
  {
    // Assemble the starting index for the events to retrieve, and the number of events to retrieve
    start_ndx = (((uint16_t)bufinfoptr[1]) << 8) + bufinfoptr[0];   // Index of the most recent event
    *num = ( (bufinfoptr[2] <= 5) ? bufinfoptr[2] : 5);             // Number of events requested (max = 5)

    // If starting index >= number of events, no events to return, so done
    if (start_ndx > (EV_Dist.Num_Events - 1))
    {
      *num = 0;
      return;
    }
    // Events are requested from the most recent event to the earliest event.  However, when reading them
    //   from FRAM, they are read from earliest event to most recent event.  Therefore, we need to offset
    //   the requested starting index by the index of the last entry.  For example:
    //   Requested index    Internal log index (not rolled over)
    //        0                      NextEvntNdx - 1                                          
    //        5                      NextEvntNdx - 6
    //   If the log has not rolled over, we will clamp at index = 0, and reduce the number sent back
    //   accordingly.  If the log has rolled over, we need to consider this when doing the math

    // Check the parameters, and compute the previous and next indices
    //   prev_ndx: index of event before the earliest one we are sending back
    //   next_ndx: index of event after the latest one we are sending back
    //   There are two cases to handle, log not rolled over, and log rolled over

    // If the log has not rolled over,
    //     0 = index of the earliest entry
    //     NextEvntNdx - 1 = index of the latest entry
    //     Num_Events = NextEvntNdx = number of logs
    if (EV_Dist.Num_Events < DIST_NUM_LOGS)       // First case: log not rolled over
    {
      // Internal requested starting index is at index of most recent entry (NextEvntNdx - 1) minus the
      //   requested starting index.  Don't need to worry about rollover.  We already checked and ensured
      //   that start_ndx <= (Num_Events - 1), Num_Events = NextEvntNdx if not rolled over
      start_ndx = ((EV_Dist.NextEvntNdx - 1) - start_ndx);

      // Check whether number of entries requested and starting index is in the range of available events
      if (*num > (start_ndx + 1))                 // If number requested greater than starting index + 1, we
      {                                           //   are requesting logs before the earliest entry, so
        *num = start_ndx + 1;                     //   clamp the number of logs requested
      }
      // Set start index to the first log to retrieve - we have to move start_ndx back by the number of logs
      //   to retrieve
      start_ndx = (start_ndx + 1) - *num;       // Set start index to the first log to retrieve
    }
    // If the log has rolled over,
    //     NextEvntNdx = index of the earliest entry
    //     NextEvntNdx - 1 = index of the latest entry
    //     Num_Events = SUMMARY_NUM_LOGS = log size
    else                                        // Second case: log rolled over
    {
      // Internal requested starting index is at index of most recent entry (NxtEvntNdx - 1) minus the
      //   requested starting index.  We need to account for rollover
      //   First set i to the index of the most recent log, which is at NextEvntNdx - 1, accounting for
      //     rollover
      i = ( (EV_Dist.NextEvntNdx > 0) ? (EV_Dist.NextEvntNdx - 1) : (DIST_NUM_LOGS - 1) );
      // Now subtract the requested starting index from i, accounting for rollover
      start_ndx = ( (i >= start_ndx) ? (i - start_ndx) : ((i + DIST_NUM_LOGS) - start_ndx) );

      // Check whether number of entries requested and starting index is in the range of available events
      // Compute number of logs from the earliest entry to the starting index
      i = ( (start_ndx >= EV_Dist.NextEvntNdx) ?
                  ((start_ndx + 1) - EV_Dist.NextEvntNdx)
                                  : ((start_ndx + 1 + DIST_NUM_LOGS) - EV_Dist.NextEvntNdx) );
      if (*num > i)                             // If number requested exceeds the number of available
      {                                         //   previous entries, clamp the number of requested logs
        *num = i;
      }
      // Set start index to the first log to retrieve - we have to move start_ndx back by the number of logs
      //   to retrieve
      start_ndx = ( (*num <= (start_ndx + 1)) ?
                    ((start_ndx + 1) - *num) : ((start_ndx + 1 + DIST_NUM_LOGS) - *num) );
    }
  }
  // If message length is 5, retrieving events by EID
  else if (msglen == 5)
  {
    // When retrieving by EID, for disturbance captures, we are looking for the EID that INITIATED the
    //   capture, not the EID of the actual disturbance event.  This is stored as data in the disturbance
    //   capture log.  In addition, we cannot do a binary search, as is done with the regular EIDs, because
    //   the EIDs that initiated he capture may not be in ascending order - we have to check each (up to 75)
    //   EID in order, one at a time
    // Also, we will only retrieve one event - just the requested event
    // Assemble the EID we are looking for
    eid = (((uint32_t )bufinfoptr[3]) << 24) + (((uint32_t)bufinfoptr[2]) << 16)
                + (((uint32_t)bufinfoptr[1]) << 8) + (uint32_t)bufinfoptr[0];
    for (i = 0; i < EV_Dist.Num_Events; ++i)
    {
      // Retrieve the EID for the event that initiated the capture and compare it to the EID we are looking
      //   for.  If it matches, break
      if (ReadEID(DIST_LOG_START + (DIST_EVENT_SIZE - 4) + (i * DIST_EVENT_SIZE)) == eid)
      {
        break;
      }
    }
    // Save search result
    start_ndx = i;                          // Save search result
    *num = 1;                               // Number of events requested is 1 if retrieving events by EID
    // If the EID was not found, start_ndx = EV_Dist.Num_Events, which indicates there are no events to
    //   return
    if (start_ndx > (EV_Dist.Num_Events - 1))
    {
      *num = 0;
      return;
    }
  }
  else                                  // Invalid message length - should never happen, return 0 logs
  {
    *num = 0;
    return;
  }

  // Retrieve the events
  FRAM_Read((DIST_LOG_START + (start_ndx * DIST_EVENT_SIZE)),
                    ((DIST_EVENT_SIZE >> 1) * *num), (uint16_t *)(&SPI2_buf[8]));

} 

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventDisturbanceRead()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ReadEID()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Read EID from an event log
//
//  MECHANICS:          Calls FRAM_Read to read the first 4 bytes of an event log, assembles the EID, then
//                      returns the EID
//
//  CAVEATS:            This subroutine assumes the fram_address is a valid Event log address
//
//  INPUTS:             fram_address - address in FRAM
// 
//  OUTPUTS:            value in specific FRAM
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Read()
// 
//------------------------------------------------------------------------------------------------------------

uint32_t ReadEID(uint32_t fram_address)
{
  uint32_t values = 0;
  FRAM_Read(fram_address, 2, (uint16_t *)(&SPI2_buf[0]));
  values = (SPI2_buf[0] + (((uint32_t)SPI2_buf[1]) << 8) + (((uint32_t)SPI2_buf[2]) << 16) + (((uint32_t)SPI2_buf[3]) << 24) );

  return values;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ReadEID()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       BinarySearch()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Binary Search on FRAM Read Outs
//
//  MECHANICS:          Performs a binary search for the EID in ValueSearched over the logs ranging from
//                      FirstIndex to LastIndex
//
//                      Note: Although the code appears to be a recursive subroutine, the actual
//                            (disassembled) code is NOT recursive.  It operates more like a while-loop,
//                            where it merely jumps back to the beginning of the subroutine if another
//                            pass is required.  It does not do another subroutine call.
//
//  INPUTS:             FirstIndex - starting point of the field
//                      LastIndex - last index of the field - actually the size of Logs
//                      ValueSearched - value bieng looked ip
// 
//  OUTPUTS:            Index of EID in FRAM. When it's -1 the value that we search is not in FRAM
//
//  ALTERS:             None
// 
//  CALLS:              ReadEID() 
// 
//------------------------------------------------------------------------------------------------------------

int16_t BinarySearch(uint16_t FirstIndex, uint16_t LastIndex, uint32_t ValueSearched, uint8_t EventType)
{
  uint16_t MidIndex;
  uint32_t Values;
  int16_t RetVal = -1;

//TESTPIN_D1_TOGGLE;
  
  if (LastIndex >= FirstIndex) 
  {
    MidIndex = FirstIndex + (LastIndex - FirstIndex) / 2;

    Values =  ReadEID(FRAM_EV_INFO[EventType] + (MidIndex * EV_SIZE[EventType]));
  
    if(Values == ValueSearched) return MidIndex;
    if(Values > ValueSearched)  return BinarySearch(FirstIndex, MidIndex - 1, ValueSearched, EventType);

    return BinarySearch(MidIndex + 1, LastIndex, ValueSearched, EventType);
  }

  return RetVal;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         BinarySearch()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       EventLookUp()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           EventLookUp in event logs
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             EIDLookUp - EID what look for
//                      EventType - enum of EVENT_TYPES
// 
//  OUTPUTS:            return the Index of EID in array of FRAM reading out. When it's -1 the value that we
//                      search is not in FRAM
//
//  ALTERS:             None
// 
//  CALLS:              ReadEID(), BinarySearch()
//
//  EXECUTION TIME:     Measured on 221205 (rev 0.66 code): 136usec  This includes one sample interrupt and
//                      a worst-case condition where 10 iterations are made in BinarySearch()
// 
//------------------------------------------------------------------------------------------------------------

int16_t EventLookUp(uint32_t EIDLookUp, uint8_t EventType)
{
  int16_t result;
  uint32_t EIDAtZero;
  uint16_t NxtEvntIndex;

  // Initialize the index of the next event to be entered.
  //   Note, if the log is rolled over,
  //                NxtEvntIndex = index of the earliest entry
  //                NxtEvntIndex - 1 = index of the latest entry
  //                number of logs = log size
  //         if the log has not rolled over
  //                0 = index of the earliest entry
  //                NxtEvntIndex - 1 = index of the latest entry
  //                number of logs = NxtEvntIndex
  switch (EventType)
  {
    case EV_TYPE_SUMMARY:
      NxtEvntIndex = EV_Sum.NextEvntNdx;
      break;
    
    case EV_TYPE_DEMAND:
      NxtEvntIndex = EV_Dmnd.NextDmndAdd;        // not sure whether I can use this NextDmndAdd
      break;
      
    default :
      NxtEvntIndex = RAM_EV_ADD[EventType]->NextEvntNdx;
  }

  // There are no log entries with EID = 0.  The Master EID is initialized to 1, and it is assumed to never
  //   roll over (it is a 32-bit number).  If the EID at index = 0 is 0, there are no entries in the log, so
  //   return - 1.  If the EID that is being looked up is 0, it is invalid, so return -1
  EIDAtZero = ReadEID(FRAM_EV_INFO[EventType]);
  if ((EIDAtZero == 0) || EIDLookUp == 0) return -1;
  
  // If the buffer is rolled over, it can be considered split into two parts.  One part is from the earliest
  //   entry to the entry at index = 0, and the second part is from the entry at index = 0 to the latest
  //   entry.  In this case, we prescreen the search by comparing the search EID to the EID at 0.
  //   If the search EID is less than the EID at 0, we search from earliest entry (index = NxtEvntIndex) to
  //   entry immediately prior to the EID at 0 (index = MAX_NUM_EVENTS[EventType]-1).
  //   If the search EID is greater than the EID at 0, we search from entry at 0 (index = 0) to most recent
  //   entry (index = NxtEvntIndex - 1, accounting for rollover)
  // If the buffer is not rolled over and the search EID is valid, we will search from entry at 0
  //   (index = 0) to most recent entry (index = NxtEvntIndex - 1)
  // If the buffer is not rolled over and the search EID is invalid (< EIDAtZero), we will search from entry
  //   (index = NxtEvntIndex) to entry immediately prior to the EID at 0
  //   (index = MAX_NUM_EVENTS[EventType]-1).  Although this contains unwritten log entries, the FRAM has
  //   been zeroed out at the factory, and so any EID that is read will be zero and will not match the
  //   requested EID.
  if (EIDLookUp < EIDAtZero)
  {
    result = BinarySearch(NxtEvntIndex, (MAX_NUM_EVENTS[EventType]-1), EIDLookUp, EventType);
  }
  else
  {
    result = BinarySearch(0,
                NxtEvntIndex==0?(MAX_NUM_EVENTS[EventType]-1):NxtEvntIndex - 1, EIDLookUp, EventType);
  }

  return result;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         EventLookUp()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ClearLogFRAM()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Clear and make 0 log in FRAM in specific logs
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             EventType - enum of EVENT_TYPES that determines what log being zero
//                                    when the parameter is 0 it formats entire FRAM for logs
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              FRAM_Write(), ClearEvents()
//
//  EXECUTION TIME:     Measured on 221205 (rev 0.66 code): 215msec (this includes sample interrupts)
//                      This is under the worst-case condition where all logs are cleared
// 
//------------------------------------------------------------------------------------------------------------

void ClearLogFRAM(uint8_t EventType)
{

uint16_t r;

if (EventType == 0)
{
  for (EventType = 1; EventType <= (sizeof(FRAM_EV_INFO)/sizeof(FRAM_EV_INFO[0])); EventType++)
  {
    // *** DAH   MAY NEED TO ADD CODE TO REFRESH THE WATCHDOG TIMER - TOTAL EXECUTION TIME IS ABOUT 215MSEC
    for (r = 0; r <= MAX_NUM_EVENTS[EventType]; r++)
    {
      FRAM_Clean(DEV_FRAM2, (FRAM_EV_INFO[EventType] + (r * EV_SIZE[EventType])), EV_SIZE[EventType] >> 1);
    }
  }
}
else 
{
  for (r = 0; r < MAX_NUM_EVENTS[EventType]; r++)
  {
    FRAM_Clean(DEV_FRAM2, (FRAM_EV_INFO[EventType] + (r * EV_SIZE[EventType])), EV_SIZE[EventType] >> 1);
  }
}

ClearEvents();

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ClearLogFRAM()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ExtendedCapture()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Wrapping functions that stored values into BB FRAM
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             ExtCapVal - what type will be stored in FRAM - OneCycle or TwoHundred ms
//                      Counter - Counter of values determines position in BB FRAM
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              InsertNewEvent(), ExtendedCaptureValues()
//
//  EXECUTION TIME:     Measured on 230518 (Rev 38 code): 245usec max with interrupts enabled and a capture
//                      occurring
//
//------------------------------------------------------------------------------------------------------------

// Constants table that maps the index of the extended capture event to the index of the associated
//   disturbance capture event
//      GOOSE = 17, OV = 2, UV = 3, HL1 = 15, HL2 = 16
const uint8_t ECNDX_TO_DISTNDX[EXTCAP_NUM] = {17, 2, 3, 15, 16};

void ExtendedCapture(uint8_t TypeOfRecord)
{
  uint8_t i;
  uint8_t bmsk;

    for (i = 0; i< EXTCAP_NUM; i++)
    {
      bmsk = (1 << i);
      // Step through the flags to process any new extended capture requests. Since the request flag is
      //   cleared as soon as processing begins, if it is set again, we assume it is a new event occurrence.
      // The subroutines that set these flags, must ensure that the flag is set only once for each event
      //   occurrence!  Reference Highload_Alarm() in Prot.c
      if ((ExtCap_ReqFlag & bmsk) == bmsk)
      {
        // Put the new event in the queue: this will cause summary and perhaps snapshot entries to be made
        if ((ExtCap_AckFlag & bmsk) == 0)       // If this is a new occurrence for this event type, insert
        {                                       //   the code to do snapshots and summary
          // Save the EID of the event that started the associated disturbance capture.  It is the returned
          //   value
          sListOfDist[ECNDX_TO_DISTNDX[i]].EIDofStartEvnt = InsertNewEvent(aExtCapCause[i]);

          if (Ext_WF_Capture.InProg == FALSE)   // Initiate a waveform capture for this event also, unless
          {                                     //   one is already in progress
            Ext_WF_Capture.Req = TRUE;
          }
        }
        else                                    // If we are already doing an extended capture for the same
        {                                       //   event type, insert code to just do summary
          // Save the EID of the event that started the disturbance capture.  It is the returned value
          sListOfDist[ECNDX_TO_DISTNDX[i]].EIDofStartEvnt = InsertNewEvent(aExtCapCause[i]+ EXTCAP_SNAP_NOSNAP_DIFF);
        }

        // Set the corresponding Ack flag.  This will lock out any new events of this type from generating
        //   extended captures or snapshots (only summary events allowed) until the extended capture is
        //   completed (60 seconds later).  The Ack flag is actually used more like an "in progress" flag
        ExtCap_AckFlag |= bmsk;
        ExtCap_ReqFlag &= (0xFF ^ bmsk);        // Clear the request flag

        // If we are in the Idle state, no extended capture is in progress, so set the state to initiate one
        if (sExtCapState.State == EC_IDLE)
        {
          sExtCapState.Cause = aExtCapCause[i];
          sExtCapState.State = EC_START;
        }
      }
    }
    ExtendedCaptureValues(TypeOfRecord);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ExtendedCapture()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       ExtendedCaptureValues()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Wrapping functions that stored values into BB FRAM
//
//  MECHANICS:          Event Manager
//
//  INPUTS:             ExtCapVal - what type will be stored in FRAM - OneCycle or TwoHundred ms
//                      Counter - Counter of values determines position in BB FRAM
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              main
//
//------------------------------------------------------------------------------------------------------------

void ExtendedCaptureValues(uint8_t TypeOfRecord)
{
  sExtCapState.TypeOfRecord = TypeOfRecord;
  
  switch (sExtCapState.State)
   {    
    case EC_IDLE:
    default:
      break;

    case EC_START:
      // We want to do an extended capture only if the waveform capture is made (or at least started so that
      //   a partial capture is made).  Although the request for the waveform capture has been generated
      //   (when Ext_WF_Capture.Req = TRUE), the request will be cleared and the capture not made if a trip
      //   or alarm capture is in progess, so we won't start unless there is no alarm or trip waveform
      //   capture in progress and the extended waveform capture is in progress.  The underlying philosphy
      //   is that the waveform and RMS captures are considered a single set.
      //   Note, we must check the alarm and trip in-progress flags first, because the extended capture
      //   in-progress flag may be set even if a trip or alarm capture is in progress.  It is then cleared
      //   when the alarm or trip capture is completed
      if ( (Trip_WF_Capture.InProg == TRUE)      // If alarm or trip capture is in progress, cancel the
        || (Alarm_WF_Capture.InProg == TRUE) )   //   extended capture
      {
        sExtCapState.State = EC_IDLE;
        ExtCap_ReqFlag = 0;
        ExtCap_AckFlag = 0;
        break;
      }
      else if (Ext_WF_Capture.InProg == TRUE)    // If extended wf capture in progress, jump to start
      {                                          //   the RMS captures
        sExtCapState.State = EC_IN_PROGRESS;
      }
      // Otherwise just remain in this state.  Either the extended capture flag will be set the next time we
      //   enter or the extended capture was aborted and the trip and alarm in-progress flags were cleared
      //   before we checked them (above).  In this case, we will just remain in this state until another
      //   extended capture event occurs.  This is ok, since this is equivalent to the IDLE state
      else
      {
        break;
      }

    case EC_IN_PROGRESS:

      switch (TypeOfRecord)
      {
        case OneCycle:
          if (sExtCapState.OneCycleCounter != EXTCAP_ONECYCLE_NUM_LOGS)
          {
            // Insert timestamp if this is the first value
            if (sExtCapState.OneCycleCounter == 0)
            {
              __disable_irq();
              Get_InternalTime((struct INTERNAL_TIME *)(&SPI2_buf[0]));
              __enable_irq();              
              ExtCapt_FRAM_Write(EXTCAP_ONECYC_TS_START, 4, (uint16_t *)(&SPI2_buf[0]));
            }
            ExtCapSnapshotMeterOneCyc();
            ExtCapt_FRAM_Write((EXTCAP_ONECYCLE_LOG_START + EXTCAP_ONECYCLE_SIZE*sExtCapState.OneCycleCounter),
                    (sizeof(ExtCapSnapshotMeteredValues) >> 1), (uint16_t *)(&ExtCapSnapshotMeteredValues));
            sExtCapState.OneCycleNumVals++;
            sExtCapState.OneCycleCounter++;
          }
         break;

        case TwoHundred:
          if (sExtCapState.TwoHundredCounter != EXTCAP_TWOHUNDRCYCLE_NUM_LOGS)
          {
            // Insert timestamp if this is the first value
            if (sExtCapState.TwoHundredCounter == 0)
            {
              __disable_irq();
              Get_InternalTime((struct INTERNAL_TIME *)(&SPI2_buf[0]));
              __enable_irq();              
              ExtCapt_FRAM_Write(EXTCAP_TWOHUN_TS_START, 4, (uint16_t *)(&SPI2_buf[0]));
            }
            ExtCapSnapshotMeterTwoHundred();
            ExtCapt_FRAM_Write((EXTCAP_TWOHUNDRCYCLE_LOG_START + EXTCAP_TWOHUNDRCYCLE_SIZE*sExtCapState.TwoHundredCounter),
                    (sizeof(ExtCapSnapshotMeteredValues) >> 1), (uint16_t *)(&ExtCapSnapshotMeteredValues));
            sExtCapState.TwoHundredNumVals++;
            sExtCapState.TwoHundredCounter++;
          }
          else
          {
            sExtCapState.State = EC_END;
          }

        break;
      }
      break;

    case EC_CANCEL:
      memset(&sExtCapState, 0, sizeof(sExtCapState));

    case EC_END:
      ExtCapt_FRAM_Write(EXTCAP_EID_START, (sizeof(sExtCapState.EID) >> 1), (uint16_t *)(&sExtCapState.EID));
      sExtCapState.OneCycleCounter = 0;
      sExtCapState.TwoHundredCounter = 0;
      sExtCapState.State = EC_IDLE;
      ExtCap_ReqFlag = 0;                           // Clear all extended capture requests since we only
      ExtCap_AckFlag = 0;                           //   do one for all requests
    break;
        
   }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         ExtendedCaptureValues()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       DisturbanceCapture()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DisturbanceCapture function to select what params and how will be proccessed according to Event (flag)
//
//  MECHANICS:          
//
//  INPUTS:             State
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              main
//
//  EXECUTION TIME:     Measured on 230518 (Rev 38 code): 202usec max; tested with all request flags set,
//                      then all flags cleared (causes captures to be made for all paramaeters, then memory
//                      cleared).  Test conducted with interrupts enabled
//
//------------------------------------------------------------------------------------------------------------

void DisturbanceCapture(void)
{
  uint8_t i;
  uint32_t bmsk;

  // Insert GOOSE capture command into Dist_Flag
  if (Dist_GOOSE == 1)
  {
    Dist_Flag |= DS_GSE;
  }
  else if (Dist_GOOSE == 0)
  {
    Dist_Flag &= (DS_GSE ^ 0xFFFFFFFF);
  }

  // Insert SD capture command into Dist_Flag
  if (Dist_Flag_SD == 1)
  {
    Dist_Flag |= DS_SD;
  }
  else
  {
    Dist_Flag &= (DS_SD ^ 0xFFFFFFFF);
  }
  if (Dist_Flag_Cancel_SD == TRUE)
  {
    Dist_Flag_Cancel |= DS_SD;
    Dist_Flag_Cancel_SD = FALSE;
  }

  // Insert GF capture command into Dist_Flag
  if (Dist_Flag_GF == 1)
  {
    Dist_Flag |= DS_GF;
  }
  else
  {
    Dist_Flag &= (DS_GF ^ 0xFFFFFFFF);
  }
  if (Dist_Flag_Cancel_GF == TRUE)
  {
    Dist_Flag_Cancel |= DS_GF;
    Dist_Flag_Cancel_GF = FALSE;
  }


  for (i = 0; i< DIST_NUM; i++)
  {
    bmsk = (1 << i);
    // Step through the flags to process the disturbance captures
    if (Dist_Flag & bmsk)                           // If flag set, either it is a new request or a capture
    {                                               //   is in progress
      if (!sListOfDist[i].State)                    // If State = IDLE, new event 
      {
        sListOfDist[i].State = DS_START;
        sListOfDist[i].Cause = aDistCause[i];
      }
    }
    else                                            // If flag clear, either a capture has ended or there
    {                                               //   was no capture to begin with
      if (sListOfDist[i].State == DS_IN_PROGRESS)   // If State = IN_PROGRESS, a capture has ended
      {
        sListOfDist[i].State = DS_END;
      }                                             // Otherwise captue is ending or there was no capture
    }                                               //   to begin with.  Either way, don't change the state

    if (Dist_Flag_Cancel & bmsk)                    // Cancel flag overrides everything
    {
      sListOfDist[i].State = DS_CANCEL;
      Dist_Flag_Cancel &= (bmsk ^ 0xFFFFFFFF);
      Dist_Flag &= (bmsk ^ 0xFFFFFFFF);                 // Make sure request flag is reset also
    }
    
    // go for the values!
    sListOfDist[i].pDistValue = pDistValues[i];
    DisturbanceCaptureValues(&sListOfDist[i], i);
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         DisturbanceCapture()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION       DisturbanceCaptureValues()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           DisturbanceCapture function stores values into structure
//
//  MECHANICS:          A disturbance capture consists of:
//                          - duration of the event
//                          - the average value of the critical parameter
//                          - the min or max value of the critical parameter
//                          - the maximum thermal capacity (i.e., how close we came to tripping)
//                      Captures are performed for:
//                        event                   parameter             min/max  thermal capacity value
//                        LDPU                    CurOneCycMax          max (0)  LD_Bucket
//                        Overtemperature         THSensor.Temperature  max (0)  Tmax/threshold
//                        Overvoltage             Vll_max               max (0)  duration/time setting
//                        Undervoltage            Vll_min               min (1)  duration/time setting
//                        Voltage unbalance       VolUnbalTot           max (0)  duration/time setting
//                        Current unbalance       CurUnbalTot           max (0)  duration/time setting
//                        Reverse active power    Pwr200msec.Ptot       max (0)  duration/time setting
//                        Reverse reactive power  Pwr200msec.Rtot       max (0)  duration/time setting
//                        Phase loss              CurUnbalTot           max (0)  duration/time setting
//                        Overfrequency           FreqLoad.FreqVal      max (0)  duration/time setting
//                        Underfrequency          FreqLoad.FreqVal      min (1)  duration/time setting
//                        Overpower, real         Pwr200msec.Ptot       max (0)  duration/time setting
//                        Overpower, reactive     Pwr200msec.Rtot       max (0)  duration/time setting
//                        Overpower, apparent     Pwr200msecApp.Apptot  max (0)  duration/time setting
//                        UnderPF                 PF.App[3]             min (1)  duration/time setting
//                        High Load1              CurOneCycMax          max (0)  LD_Bucket
//                        High Load 2             CurOneCycMax          max (0)  LD_Bucket
//                        GOOSE command           CurOneCycMax          max (0)  0 (non-tripping event)
//                        Ground Fault            CurOneCycIg           max (0)  duration/time setting
//                        SDPU                    CurOneCycMax          max (0)  SD bucket
//
//  INPUTS:             sDistValues - pointer to DS structure of particular parameter being captured
// 
//  OUTPUTS:            None
//
//  ALTERS:             None
// 
//  CALLS:              DisturbanceCaptureSelect
//
//------------------------------------------------------------------------------------------------------------

const uint8_t DIST_PROC_CODE[] =
// 0 - Max, 1 = Min, 2 = Custom TBD 
{
// LDPU   Temp    OV    UV    VUnbal    CUnbal    RevW    RevVar    PhLoss    Of 
    0,     0,     0,    1,      0,        0,       2,       2,        0,      0,

//  Uf    OW    OVar    OVa    UPF       HL1       HL2    GOOSE      GF     SDPU
    1,     2,     2,    0,      3,        0,       0,       0,        0,      0
};

void DisturbanceCaptureValues(struct DIST_VALUES *psDistValues, uint8_t proc_ndx)
{
  uint32_t temp;

  switch (psDistValues->State)
  {
    case DS_IDLE:
      break;

    case DS_START:
      // Get Entry Time of Disturbance Capture log
      __disable_irq();
      Get_InternalTime(&psDistValues->sEntryTS);
      __enable_irq();
      if ( (DIST_PROC_CODE[proc_ndx] == 0) || (DIST_PROC_CODE[proc_ndx] == 4) )
      {
        psDistValues->MaxMinVal = 0;
      }
      else if (DIST_PROC_CODE[proc_ndx] == 1)
      {
        psDistValues->MaxMinVal = 1.0E9;
      }
      psDistValues->SubCounter = 0;
      psDistValues->MinsCounter = 0;
      psDistValues->ThermalCap = 0;
      psDistValues->AvgVal = 0;
      psDistValues->SubAvg = 0;;
      psDistValues->Duration = 0;
      
      psDistValues->State++;

    case DS_IN_PROGRESS:
      // Unsigned max value
      if (DIST_PROC_CODE[proc_ndx] == 0)
      {
        psDistValues->MaxMinVal = ( (psDistValues->MaxMinVal > *psDistValues->pDistValue)
                    ? (psDistValues->MaxMinVal) : (*psDistValues->pDistValue) );
      }
      // Unsigned min value
      else if (DIST_PROC_CODE[proc_ndx] == 1)
      {
        psDistValues->MaxMinVal = ( (psDistValues->MaxMinVal < *psDistValues->pDistValue)
                    ? (psDistValues->MaxMinVal) : (*psDistValues->pDistValue) );
      }
      // Signed max value.  In this case, the max is taken over the absolute value.  It is safe to assume
      //   all incoming values will be either positive or negative.  That is, we won't receive a mix of
      //   positive and negative values in one set of values.
      else if (DIST_PROC_CODE[proc_ndx] == 2)
      {
        if (*psDistValues->pDistValue < 0)          // Negative input
        {
          psDistValues->MaxMinVal = ( (*psDistValues->pDistValue < psDistValues->MaxMinVal)
                      ? (*psDistValues->pDistValue) : (psDistValues->MaxMinVal) );
        }
        else                                        // Positive input
        {
          psDistValues->MaxMinVal = ( (*psDistValues->pDistValue > psDistValues->MaxMinVal)
                      ? (*psDistValues->pDistValue) : (psDistValues->MaxMinVal) );
        }
      }
      // Signed min value.  In this case, the min is taken over the absolute value.  It is safe to assume
      //   all incoming values will be either positive or negative.  That is, we won't receive a mix of
      //   positive and negative values in one set of values.
      else                                          // if (DIST_PROC_CODE[proc_ndx] == 3)
      {
        if (*psDistValues->pDistValue < 0)          // Negative input
        {
          psDistValues->MaxMinVal = ( (*psDistValues->pDistValue > psDistValues->MaxMinVal)
                      ? (*psDistValues->pDistValue) : (psDistValues->MaxMinVal) );
        }
        else                                        // Positive input
        {
          psDistValues->MaxMinVal = ( (*psDistValues->pDistValue < psDistValues->MaxMinVal)
                      ? (*psDistValues->pDistValue) : (psDistValues->MaxMinVal) );
        }
      }

      psDistValues->SubAvg += *psDistValues->pDistValue;
      psDistValues->SubCounter++;

      if (psDistValues->SubCounter > DISTURBANCEONEMIN)
      {
        psDistValues->AvgVal += psDistValues->SubAvg/psDistValues->SubCounter;
        psDistValues->MinsCounter++;

        // resetting OneMin subinterval accumulators
        psDistValues->SubCounter = 0;
        psDistValues->SubAvg = 0;
      }

      break;

    case DS_END:
      if (psDistValues->SubCounter > 0)             // avoiding dividing by zero, OneMin subinterval AVG
      {                                             //   calculation and storing in structure
        psDistValues->SubAvg = psDistValues->SubAvg/psDistValues->SubCounter;
      }

      if (psDistValues->MinsCounter > 0)            // avoiding dividing by zero, OneMin interval AVG
      {                                             //   calculation and storing in structure
        psDistValues->AvgVal = psDistValues->AvgVal/psDistValues->MinsCounter;
      }

      // calculation of average number of two sets of numbers                  *** DAH  CHECK DIVIDE BY 0 IF COUNTERS = 0
      psDistValues->AvgVal = ( (psDistValues->AvgVal*psDistValues->MinsCounter*(DISTURBANCEONEMIN+1))
                                + psDistValues->SubAvg*psDistValues->SubCounter ) /
                         ( psDistValues->SubCounter + (psDistValues->MinsCounter * (DISTURBANCEONEMIN+1)) );

      // calculation of duration in ms
      psDistValues->Duration = ( ((float)psDistValues->MinsCounter)*60.0
                               + ((float)psDistValues->SubCounter)/60.0 ) * 1000.0;

      InsertNewEvent(psDistValues->Cause);                                 // Insert event into queue

      psDistValues->Spare = 30;                     // Test value since unused anyway - useful for debugging

      psDistValues->ValueCode = proc_ndx;
      // Store how close we came to tripping as a percentage (100% = trip)
      switch (proc_ndx)
      {
        case DS_LD_PICKUP:                          // LDPU: thermal bucket
          psDistValues->ThermalCap = LD_Bucket;
          break;

        case DS_HIGH_TEMP:                          // Overtemperature: max temp/threshold * 100%
          psDistValues->ThermalCap = (psDistValues->MaxMinVal * 100)/TEMP_TRIP_THRESHOLD;
          break;

        case DS_OVER_VOLT:                          // Overvoltage: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)OvTripBucket)/10.0;
           break;

        case DS_UNDER_VOLT:                         // Undervoltage: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)UvTripBucket)/10.0;
           break;

        case DS_VOLT_UNBAL:                         // Voltage unbalance: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)VuTripBucket)/10.0;
           break;

        case DS_CUR_UNBAL:                          // Current unbalance: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)CuTripBucket)/10.0;
           break;

        case DS_REV_W:                              // Reverse Active Power: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)RevWTripBucket)/10.0;
           break;

        case DS_REV_VAR:                            // Reverse Reactive Power: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)RevVarTripBucket)/10.0;
           break;

        case DS_PHASE_LOSS:                         // Phase Loss: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)PlTripBucket)/10.0;
           break;

        case DS_OVER_FREQ:                          // Overfrequency: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)OfTripBucket)/10.0;
           break;

        case DS_UNDER_FREQ:                         // Underfrequency: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)UfTripBucket)/10.0;
           break;

        case DS_OVER_W:                             // Over Real Power: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)OvrWTripBucket)/10.0;
           break;

        case DS_OVER_VAR:                           // Over Reactive Power: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)OvrVarTripBucket)/10.0;
           break;

        case DS_OVER_VA:                            // Over Apparent Power: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)OvrVATripBucket)/10.0;
           break;

        case DS_UNDER_PF:                           // Low PF: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)PFTripBucket)/10.0;
           break;

        case DS_HIGHLOAD_1:                         // HL1: thermal bucket
          psDistValues->ThermalCap = LD_Bucket;
          break;

        case DS_HIGHLOAD_2:                         // HL2: thermal bucket
          psDistValues->ThermalCap = LD_Bucket;
          break;

        case DS_GOOSE:                              // GOOSE: 0 (non-tripping event)
          psDistValues->ThermalCap = 0;
          break;

        case DS_GND_FAULT:                          // GF: duration/time setting * 100%
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)GFTripBucket)/10.0;

        case DS_SD_PICKUP:                          // SD: Trip Bucket
           // Value is U16 in tenths of a percent
           psDistValues->ThermalCap = ((float)SDTripBucket)/10.0;
          break;
      }
      psDistValues->State++;
      break;

    case DS_WAIT:
      if (psDistValues->Spare == 0)         // Wait for event to be stored in EventManager()
      {                                     //   before clearing the values
        psDistValues->State++;              // This ensures a new event won't overwrite the values before
      }                                     //   they are saved
      else
      {
        break;
      }      

    case DS_CANCEL:
      // Reset all values associated with the event, except the initiating EID.  We will keep this because
      //   in some cases where a trip occurs but current is not interrupted, the original event gets cleared
      //   and then a new disturbance capture is started without an originating event.  This should never
      //   happen in the field, but I can't see a problem in retaining the EID.  If it's updated, it will
      //   overwritten anyway, and the worst case is that it's the wrong number
      temp = psDistValues->EIDofStartEvnt;
      memset(psDistValues, 0, (sizeof(sListOfDist[0])) );
      psDistValues->EIDofStartEvnt = temp;
      psDistValues->State = DS_IDLE;
      break;

    default:
      break;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION         DisturbanceCaptureValues()
//------------------------------------------------------------------------------------------------------------

