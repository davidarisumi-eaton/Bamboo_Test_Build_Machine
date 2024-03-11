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
//  MODULE NAME:        FRAM_Flash_def.h
//
//  MECHANICS:          This is the definitions file for the FRAM and Flash memory chips
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.10   160310  DAH File Creation
//   0.12   160502  DAH Changed comments because AFE_CAL_SIZE changed
//   0.13   160512  DAH Added ENERGY_SIZE and FRAM_ENERGY_x to support storing the energy in FRAM
//   0.14   160620  DAH - Changed comments for 32K x 8 size instead of 64K x 8 size
//                      - Added support for Test Injection
//                          - TESTINJ_CAL_SIZE and FRAM_TESTINJ added
//   0.15   160718  DAH - Added Black Box (SPI1) FRAM definitions
//   0.22   180420  DAH - Revised SPI2 FRAM definitions because the FRAM was changed from a 25V02 (256kbit)
//                        to a 25V20 (2Mbit) for the rev 3 hardware
//                      - Added DEMAND_SIZE,FRAM_DEMAND, MASTER_EID to support storing demand logs in FRAM
//   0.24   180517  DAH - Combined Flash_def.h and FRAM_def.h
//                          - Renamed file from FRAM_def.h FRAM_Flash_def.h
//                          - Added Flash definitions
//                          - Changed ENERGY_SECTOR_END from 422 to 421
//                      - Added struct FLASH_INT_REQ to support alarm and trip events
//   0.25   180621  DAH - Added support for SPI2 requests to be handled in the 1.5msec ISR (for waveform
//                        writes to Flash)
//                          - Deleted Flash in-use and foreground request flags (FlashCtrl is no longer
//                            used)
//                      - Added STARTUP_SCALE for start time scaling constant
//                      - Added support for Alarm waveform capture events
//                          - Added Flash definitions for Alarm and Trip waveforms
//                          - Added EID, InProg, and NumSamples to struct FLASH_INT_REQ definition
//                          - Added ALARM_WF_INFO and ALARM_WF_ADD to FRAM definitions
//   0.26   180829  DAH - Added support for Strip-Chart waveforms in Flash memory
//                          - Reduced number of Trip waveforms and remapped the Flash memory storage
//                          - Replaced NUM_WAVEFORMS with NUM_TRIP_WAVEFORMS and NUM_ALARM_WAVEFORMS
//   0.27   181001  DAH - Added support for handling Trip waveforms
//                          - Added FlashAdd to struct FLASH_INT_REQ definition
//                          - Added TRIP_WF_ADD and TRIP_WF_INFO definitions
//                      - Corrected error in FRAM address definitions (comments only)
//                      - Added support for handling Strip-Chart waveforms
//                          - Added NUM_CHART_WAVEFORMS, CHART_WAVEFORMS_START,CHART_WAVEFORMS_END,
//                            CHART_WF_INFO, and CHART_WF_ADD definitions
//   0.32   190726  DAH - Deleted demand voltages.  They are no longer required.  Modified comments
//                        accordingly.
//                      - Added MNMX_DMNDI_ADD and MAX_DMNDP_ADD for min and max demand current and power
//                        storage
//   0.35   191118  DAH - struct ADC_CAL size reduced from 88 to 72 bytes
//                      - struct AFE_CAL size increased from 80 to 100 bytes
//   0.37   200604  DAH - Added struct EV_ADD EV_Add to FLASH_INT_REQ to hold the number of events and index
//                        of the next event entry.
//                      - In struct FLASH_INT_REQ, renamed StartIndex to SampleStartIndex so it is not
//                        confused with the Event storage index
//                      - In struct FLASH_INT_REQ, added uint8_t WF_Type and uint8_t spare
//                      - Reduced FRAM memory allocation for Alarm (ALARM_WF_INFO), Trip )TRIP_WF_INFO), and
//                        strip-chart (CHART_WF_INFO) from 22 bytes to 14 bytes.  Since padding bytes were
//                        removed from struct FLASH_INT_REQ, we only need to save NumSamples, EID, and TS
//                      - Added WF_HEADER_SIZE definition
//                      - Changed ENERGY_SECTOR_END from x1A5 to x1A6 because we need the extra sector to
//                        ensure we still have at least 45 days' worth of entries even when we erase a
//                        sector to prepare it for the next entry.  Note, Sector x1A6 is not used - it is
//                        the end point.  Only sectors x10 thru x1A5 hold data
//                      - Added FRAM allocation for redundant storage of the event addressing info (index
//                        and number of events), along with the Master EID and Startup Cal Constant
//                      - Added FRAM allocation for events storage
//  0.38    200708  DAH - Added SETP_START definition to support saving setpoints (as a test)
//  0.59    220831  DAH - Deleted Black Box FRAM 10-cycle waveform definitions
//  0.60    220928  DAH - Added Group 2 and Group 3 setpoint definitions
//  0.61    221001  DB  - Added SNAPSHOT_SIZE and TIMEADJ_SIZE definitions
//  0.69    230220  DAH - Deleted Minor Alarm events.  These are handled by Basic events with the expanded
//                        Summary Log codes
//                          - Minor Alarm Event Storage deleted from Storage FRAM map
//                          - Renamed MAJORALARM_LOG_ADD to ALARM_LOG_ADD
//                          - Renamed MAJALARM_EVENT_SIZE to ALARM_EVENT_SIZE
//                          - Renamed MAJALARM_NUM_LOGS to ALARM_NUM_LOGS
//                          - Renamed MAJALARM_LOG_START to ALARM_LOG_START
//                          - Renamed MAJALARM_LOG_END to ALARM_LOG_END
//                      - Strip chart waveform captures has been deleted and replaced with the extended
//                        capture waveforms
//                          - CHART_WF_ADD renamed to EXTCAP_WF_ADD
//                      - Reduced number of Trip waveforms from 25 to 21 and increased number of Extended
//                        Capture waveforms from 2 to 7
//                      - Deleted setpoints definitions from the Flash as they are not used
//  0.71    230228  DAH - Added Group 11 setpoints to FRAM
//  0.72    230320  DAH - Added thermal memory bucket value storage to FRAM map (TM_CAPACITY)
//                      - Added snapshots for extended captures (similar to alarm snapshots)
//   25     230403  DAH - Corrected FRAM memory location comments
//                      - Added EID allocation to the extended capture FRAM (EXTCAP_EID_xxx)
//   26     230403  DAH - Added Time Stamp allocation to the extended capture FRAM (EXTCAP_ONECYC_TS_xxx and
//                        EXTCAP_TWOHUN_TS_xxx)
//   36     230428  DAH - In struct FLASH_INT_REQ, replaced spare with savindx to support waveform capture
//                        changes
//   37     230516  DAH - Added the EID of the event that initiated the capture to the disturbance capture
//                        event package
//                      - Reduced the number of disturbance captures from 100 to 75 so that the all of the
//                        EIDs can be transmitted to the display processor in one message
//   40     230531  DAH - Snapshot size changed from 152 to 202 for trip, test trip, alarm, and extended
//                        capture snapshots
//                      - Replaced TRIP_EVENT_SIZE, TESTTRIP_EVENT_SIZE, ALARM_EVENT_SIZE, and
//                        EXTCAP_EVENT_SIZE with SNAPSHOTS_EVENT_SIZE since the log entries are the same for
//                        these four event logs
//                      - In struct FLASH_INT_REQ, replaced savindx with spare as part of a bug fix (savindx
//                        is no longer used)
//   66     230825  DAH - Modified FRAM_UNPROTEND definition
//   82     230928  DAH - Deleted FIRMWARE_VR from factory buffer FRAM definitions.  It is not stored in
//                        FRAM
//   142    240119  DAH - Revised factory calibration values in the SPI2 FRAM map
//   143    240122  DAH - Revised factory buffer variable definitions
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
//          Common Definitions
//------------------------------------------------------------------------------------------------------------

// Note, struct AFE_CAL, ADC_CAL, and TESTINJ_CAL include the checksum and complement
#define AFE_CAL_SIZE        (sizeof(struct AFE_CAL))     
#define ADC_CAL_SIZE        (sizeof(struct ADC_CAL))
#define TESTINJ_CAL_SIZE    (sizeof(struct TESTINJ_CAL))




//
//------------------------------------------------------------------------------------------------------------
//          Flash Memory Overview
//------------------------------------------------------------------------------------------------------------
//
// REFERENCE: MicroChip SST26VF064B, 64Mb Serial 3V Flash datasheet DS20005119G 2015
//
// The serial Flash memory is organized in uniform, 4KByte erasable sectors with the following erasable
// blocks: eight 8KByte parameter, two 32 KByte overlay, and one-hundred twenty-six 64KByte overlay blocks:
//   Note, the data sheet defines a block as the largest erasable segment (other than a chip erase).
//
//                            Block Size
//     -------------------  +------------+
//     Sectors 2046 - 2047  |   8 KByte  |
//                          +------------+
//     Sectors 2044 - 2045  |   8 KByte  |          2 sectors for 8KByte blocks
//                          +------------+
//     Sectors 2042 - 2043  |   8 KByte  |
//                          +------------+
//     Sectors 2040 - 2041  |   8 KByte  |
//                          +------------+
//     Sectors 2032 - 2039  |  32 KByte  |          8 sectors for 32KByte blocks
//     -------------------  +------------+
//     Sectors 2016 - 2031  |  64 KByte  |
//     -------------------  +------------+          16 sectors for 64KByte blocks
//              .           |      .     |      +-----> +-----------+
//              .           |      .     |      |       |  4 KByte  |
//              .           |      .     |      |       +-----------+
//     -------------------  +------------+      |       |  4 KByte  |
//       Sectors 32 - 47    |  64 KByte  |      |       +-----------+
//     -------------------  +------------+      |       |     .     |
//       Sectors 16 - 31    |  64 KByte  | -----+       |     .     |
//     -------------------  +------------+      |       |     .     |
//       Sectors 8 - 15     |  32 KByte  |      |       +-----------+
//                          +------------+      |       |  4 KByte  |
//        Sectors 6 - 7     |   8 KByte  |      |       +-----------+
//                          +------------+      |       |  4 KByte  |
//        Sectors 4 - 5     |   8 KByte  |      +-----> +-----------+
//                          +------------+
//        Sectors 2 - 3     |   8 KByte
//                          +------------+
//        Sectors 0 - 1     |   8 KByte  |
//     -------------------  +------------+
//       
// NOTE: Defined block sizes are in bytes
//       Flash starting addresses are in bytes.
//
// Locations are written a page (256 bytes at a time).  The page must be erased before it is written.  The
// entire chip, a block, or a sector may be erased.
//
// Flash memory is used to store the following:
//      Factory calibration constants
//      45 days at 5 minute intervals of demand data
//      Trip log waveforms
//      Alarm log waveforms
//      Extended capture waveforms
//
// Factory calibration constants are stored in Sectors 0 thru 15.
//
// Demand data is stored in Sectors 16 thru 421.
//
// Sectors 422 - 447 are reserved to maintain block boundaries.
//
// Extended Capture waveforms are stored in Sectors 448 thru 671 (32 sectors per waveform - 7 waveforms)
//
// Trip log waveforms are stored in Sectors 672 thru 1343 (32 sectors per waveform - 21 trip waveforms)
//
// Alarm waveforms are stored in Sectors 1344 thru 2015 (32 sectors per alarm event - 21 alarm waveforms)
//  
// Sectors 2016 - 2047 are reserved.
//
//------------------------------------------------------------------------------------------------------------

#define NUM_SECTORS         2048            // 2048 sectors in the Flash chip (4K bytes each): 0x000 - 0x7FF
#define PAGES_PER_SECTOR    16              // 16 pages in each sector (256 bytes each) - this is the write
                                            //   size

// Block protection definitions
// Write protection block data is transmitted bit 143 first .. bit 0 last
//  All data is 0 except block 0 (sectors 0 & 1) is write protected
//  Block 0 protection is bits 129 (read) and 128 (write).  This is bits 1..0 of byte 16
// Reference Table 5-6 of the data sheet

#define BLK_PROT_BYTENUM    1               // Bytes are transmitted 17, 16, ..., 0
                                            //   so 1 --> 16
#define BLK_PROT_VAL        0x01            // b0 ---> write protection


#define FLASH_SECTOR_0      0x0000          // Sector 0: Test Injection calibration constants
#define FLASH_TSTINJ_SECTOR FLASH_SECTOR_0
    #define FLASH_TESTINJ   ((((uint32_t)FLASH_TSTINJ_SECTOR) << 12) + 0)

#define FLASH_SECTOR_1      0x0001          // Sector 1: AFE and ADC calibration constants
#define FLASH_AFE_SECTOR    FLASH_SECTOR_1
    // Note, the following definitions assume all of the data is on Page 0!!!
    #define FLASH_AFECAL    ((((uint32_t)FLASH_AFE_SECTOR) << 12) + 0)
    #define FLASH_ADCCALH   (FLASH_AFECAL + AFE_CAL_SIZE)
    #define FLASH_ADCCALL   (FLASH_ADCCALH + ADC_CAL_SIZE)


// Demand Data Definitions
// Demand data consists of 88 bytes:
//   EID - 4 bytes
//   Time Tag - 8 bytes
//   Energy - 40 bytes  (5 values at 8 bytes each)
//   Current Demand - 16 bytes (4 values at 4 bytes each)
//   Power Demand - 12 bytes (3 values at 4 bytes each)
//   Status - 4 bytes
//   Spare - 4 bytes
// For simplicity and to allow for future expansion, 40 bytes are reserved, so that each energy and demand
// packet consists of a 128-byte buffer, or 1/2-page of Flash.  The early demand packet (the first half of
// the page) is saved in FRAM.  After the later demand packet is assembled, it is appended to the early
// demand packet and written into Flash.
// We must store 45 days' worth of demand data at 5-minute intervals, or 45 x 24 x 12 = 12,960 packets.
// This translates to 12,960/2 = 6480 pages, or 6480/16 = 405 sectors.  We must reserve an additional sector
// to allow the latest demand data to be entered before erasing the old data.  This way, the next sector to
// be written may be erased and ready long before it needs to be written, and we still have 45 days of data.
// Therefore, 406 sectors are reserved for energy and demand logging.
// Note, Sector x1A6 is not used - it is the end point.  Only sectors x10 thru x1A5 hold data
//
#define ENERGY_SECTOR_START     0x0010            // Energy and Demand begins at Sector 16
#define ENERGY_SECTOR_END       0x01A6            // Energy and Demand ends at Sector 422
#define DMND_NUM_ENTRIES        12992             // 406 sectors * 16 pages/sector x 2 entries/page = 12992



// Extended-capture log waveforms - similar to Trip log waveforms (see below)
//   7 waveforms x 2 blocks/waveform = 14 blocks   14 blocks x 16 sectors/block = 224 sectors
#define NUM_EXTCAP_WAVEFORMS    7
#define EXTCAP_WAVEFORM_START   0x1C0                           // x1C0 (448) - Even block boundary (x1C)
                                                                // x29F (671) - Ext capture waveforms end
#define EXTCAP_WAVEFORMS_END    (EXTCAP_WAVEFORM_START + (NUM_EXTCAP_WAVEFORMS * WF_SIZE_IN_SECTORS) - 1)


// Trip log waveforms
// Trip log waveforms consist of the samples in struc RAMSAMPLES (intr_def.h):
//   Ia, Ib, Ic, In, Igsrc, Igres (float)
//   VanAFE, VbnAFE, VcnAFE, VanADC, VbnADC, VcnADC (int16)
// Each sample packet is then 6 x 4 + 6 x 2 = 36 bytes
// A Trip log waveform capture is 36 cycles, with 80 samples per cycle, so the total storage is
//   36 cyc x 80 sa/cyc x 36 bytes/sa = 103680 bytes
// A single cycle requires 80 sa/cyc x 36 bytes/sa = 2880 bytes
// Flash is written one page at a time.  Each page is 256 bytes.  For simplicity in storage, we store
//   7 sample packets (252 bytes) each page.
// Therefore, a waveform capture takes (36 cyc x 80sa/cyc)/7 sa/page = 412 pages, or 412/16 = 26 sectors.
// The Flash may be erased in sectors, blocks, or the entire chip.  The sectors must be erased before they
// can be programmed.  Therefore, we need to erase the 26 sectors to prepare for the next waveform capture.
// Both block and sector erase operations take 25msec.  To speed things up, waveforms are stored on block
// boundaries and two blocks are used to store them.
// We plan to store 24 trip waveforms, plus an extra so that it is always ready (erased), so we need 50
// blocks, or 50 x 16  = 800 sectors, even though only 650 (25 x 26) sectors are used
//
// NOTE: ALL WAVEFORM BLOCKS MUST BE ON EVEN BLOCK BOUNDARIES, BECAUSE WHEN WE ABORT A CAPTURE AND NEED TO
//       ERASE THE BLOCKS THAT HAD BEEN PARTIALLY WRITTEN, WE SET THE ADDRESS TO THE BLOCK ADDRESS & 0xFE00
//       TO ENSURE BOTH BLOCKS ARE ERASED.  THIS ASSUMES THE BLOCKS ARE ON EVEN BOUNDARIES
#define NUM_TRIP_WAVEFORMS      21
#define WF_SIZE_IN_SECTORS      (2 * 16)
#define TRIP_WAVEFORMS_START    (EXTCAP_WAVEFORMS_END + 1)      // x2A0 (572) - Even block boundary (x2A)
                                                                // x53F (1343) - Trip waveforms end
#define TRIP_WAVEFORMS_END      (TRIP_WAVEFORMS_START + (NUM_TRIP_WAVEFORMS * WF_SIZE_IN_SECTORS) - 1)

// Alarm log waveforms - similar to Trip log waveforms
#define NUM_ALARM_WAVEFORMS     21
#define ALARM_WAVEFORMS_START   (TRIP_WAVEFORMS_END + 1)        // x540 (1344) - Even block boundary (x54)
                                                                // x7DF (2015) - Alarm waveforms end
#define ALARM_WAVEFORMS_END     (ALARM_WAVEFORMS_START + (NUM_ALARM_WAVEFORMS * WF_SIZE_IN_SECTORS) - 1)




//------------------------------------------------------------------------------------------------------------
//          Accessing Flash via SPI2
//------------------------------------------------------------------------------------------------------------
//
// Flash is read from and written to via SPI2.  SPI2 is also used to access the FRAM, EEPOT, RTC, and test
// DAC (not used).  Every access may be accomplished in a single step, except for writing and erasing the
// Flash.  Writing to the Flash involves multiple SPI2 transactions, along with erase and programming
// delays, during which the Flash may not be accessed other than to read the status.  However, SPI2 may
// continue to be used to access the other devices while the Flash is being written to.  For this reason,
// an arbitration scheme is used to access the Flash.
// When storing a waveform, the Flash must be written as quickly as possible, in order to prevent the input
// samples (in the sample buffer) from overwriting the captured samples before they are written to Flash.
// For this reason, Flash transactions were moved to a 1.5msec (the page write time) interrupt.  Since the
// Flash is handled by SPI2, all SPI2 transactions were moved to this interrupt.
// When a Flash access is desired, the thread sets the corresponding request bit.  The ISR checks these bits
// when it is in its idle state, and the highest-priority request is serviced.  When the transaction is
// completed, the ISR sets the corresponding complete flag and then returns to the idle state.
//
//------------------------------------------------------------------------------------------------------------


// This is used for waveform captures.  It is important that the structure contains no padding bytes (it is
//   a multiple of 4 in length)!!!  This ensures it is ok to insert a portion of the structure into FRAM
//   and assume the number of bytes to write is just the length of the objects to be inserted, for example:
//      FRAM_Write(DEV_FRAM2, ALARM_WF_INFO, 14, (uint16_t *)(&WF_Struct->NumSamples));
//   will definitely write NumSamples, EID, and TS into the FRAM - we are certain there are no padding bytes
//   inserted anywhere as the structure length (determined by checking the .map file) is 24, so every byte
//   in the structure is accounted for.  If spare is not defined, the structure length is still 24, so there
//   are padding bytes somewhere.  Since we are not certain where, you cannot be 100% certain that
//   performing the above write operation will write all of the desired info - the length may include 2
//   padding bytes
struct FLASH_INT_REQ                    // *** DAH  MAY WANT TO ADD FLAG TO SHOW WHETHER NEXT BLOCKS HAVE BEEN ERASED
{                                       //   IF ANYTHING IS ADDED, REDUCE THE SIZE OF SPARE SO THE LENGTH IS STILL 24!!
  volatile uint8_t InProg;
  uint8_t  Req;
  struct EV_ADD EV_Add;
  uint8_t  WF_Type;
  uint8_t  spare;
  uint16_t SampleStartIndex;
  uint16_t FlashAdd;
  uint16_t NumSamples;
  uint32_t EID;
  struct INTERNAL_TIME TS;
};

#define WF_HEADER_SIZE  14              // sizeof(NumSamples + EID + TS)




//
//------------------------------------------------------------------------------------------------------------
//          Generic (SPI2) FRAM Definitions
//------------------------------------------------------------------------------------------------------------
//
//          REFERENCE: Cypress FM25V20A, 2Mb Serial 3V FRAM datasheet Doc 001-90261 Rev G June, 2017
//
//          FRAM is organized as a 256k x 8 array.
//          Write protection is provided by two bits in the status register as follows:
//             Protect Range     BP1   BP0
//                  None          0     0
//            0x30000 - 0x3FFFF   0     1     Setpoints & calibration
//            0x20000 - 0x3FFFF   1     0
//            0x00000 - 0x3FFFF   1     1
//       
// NOTE: Defined block sizes are in bytes
//       FRAM starting addresses are in bytes.
//
//
//------------------------------------------------------------------------------------------------------------
//          FRAM Factory Data Storage Structure (All Factory Settings are written with BP[1:0] = 00)
//------------------------------------------------------------------------------------------------------------

// IMPORTANT NOTE: FRAM READ AND WRITE OPERATIONS ARE DONE ON A WORD (2-BYTE) BASIS.  ALL MEMORY ALLOCATION
//                 SIZES MUST BE EVEN!!!

// #define AFE_CAL_SIZE        (sizeof(struct AFE_CAL))     
// #define ADC_CAL_SIZE        (sizeof(struct ADC_CAL))
// #define TESTINJ_CAL_SIZE    (sizeof(struct TESTINJ_CAL))

#define FRAM_CAL        0x30000             // Start of calibration constants

                                            // AFE calibration - 100 bytes                  x30000...x30063
#define FRAM_AFECAL     FRAM_CAL

                                            // ADC high-gain calibration - 72 bytes         x30064...x300AB
#define FRAM_ADCCALH    (FRAM_AFECAL + AFE_CAL_SIZE)

                                            // ADC low-gain calibration - 72 bytes          x300AC...x300F3
#define FRAM_ADCCALL    (FRAM_ADCCALH + ADC_CAL_SIZE)

                                            // Test Injection calibration - 96 bytes        x300F4...x30153
#define FRAM_TESTINJ    (FRAM_ADCCALL + ADC_CAL_SIZE)

                                            // Protected Section of FRAM End                x30153
#define FRAM_PROTEND    (FRAM_TESTINJ + TESTINJ_CAL_SIZE)

                                             // Unused FRAM locations:                      x30154...x3FFFF


//------------------------------------- First Block Of Redundant Values ------------------------------------
//  200 bytes total
//  0x00000 - 0x000C7
#define FIRSTBLK_START  0x00000

// This block contains values that are duplicated elsewhere in FRAM.  The values are typically updated in
//   normal operation (as opposed to only written in the factory, such as calibration constants).  Two
//   copies are maintained so that if one copy is corrupted (such as due to a power loss in the middle of a
//   write operation), the second copy will still be valid, although it may be "old".
// The FRAM is organized as 16K rows x 8 bytes per row, so as long as the duplicate copy is more than 8
//   bytes away, it should be ok.  For this reason, each value has at least 8 bytes reserved for it!
//
//----------------------------------------- Startup Scaling Constant ---------------------------------------
//  8 bytes
//  0x00000 - 0x00007
//
// This includes the following:
//   Scaling constant (4 bytes)
//   Scaling constant complement (4 bytes)
                                            // Startup Time Scaling Constant - 8 bytes      x00000...x00007
#define STARTUP_SCALE   FIRSTBLK_START

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------------- Master Event ID --------------------------------------------
//  8 bytes
//  0x00008 - 0x0000F
//
// This includes the following:
//   EID (4 bytes)
//   EID complement (4 bytes)
                                            // Master Event EID - 8 bytes                   x00008...x0000F
#define MASTER_EID      (STARTUP_SCALE + 8)

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------- Waveform Capture Address -----------------------------------------
//  24 bytes
//  0x00010 - 0x00027
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following for Alarm, Trip, and Strip-Chart Waveforms:
//   Index (2 bytes - index of the next open location in Flash, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
//
                                            // Alarm Waveform Log Address - 8 bytes         x00010...x00017
#define ALARM_WF_ADD    (MASTER_EID + 8)
                                            // Trip Waveform Log Address - 8 bytes          x00018...x0001F
#define TRIP_WF_ADD     (ALARM_WF_ADD + 8)
                                            // Ext-Capture Waveform Log Address - 8 bytes   x00020...x00027
#define EXTCAP_WF_ADD   (TRIP_WF_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//----------------------------------------- Summary Log Address --------------------------------------------
//  8 bytes
//  0x00028 - 0x0002F
//
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define SUMMARY_LOG_ADD     (EXTCAP_WF_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//------------------------------------- Time Adjustment Log Address ----------------------------------------
//  8 bytes
//  0x00030 - 0x00037
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define TIMEADJ_LOG_ADD     (SUMMARY_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//------------------------------------------ Trip Log Address ----------------------------------------------
//  8 bytes
//  0x00038 - 0x0003F
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define TRIP_LOG_ADD        (TIMEADJ_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------- Test Trip Log Address --------------------------------------------
//  8 bytes
//  0x00040 - 0x00047
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define TESTTRIP_LOG_ADD    (TRIP_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//----------------------------------------- Alarm Log Address ----------------------------------------------
//  8 bytes
//  0x00048 - 0x0004F
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define ALARM_LOG_ADD  (TESTTRIP_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//---------------------------------------- Demand Log Address ----------------------------------------------
//  8 bytes
//  0x00050 - 0x00057
//
// This includes the following:
//   Address (2 bytes - sector, page, and 1/2 page addr of next open location in Flash
        //     b15..5: Sector address in Flash (4Kbytes each - ENERGY_SECTOR_START to ENERGY_SECTOR_END)
        //     b4..1: Page address in Flash (256 bytes each - 0x0 thru 0xF)
        //     b0: 1/2 page address (128 bytes, 0 = bytes 0 thru 127, 1 = bytes 128 thru 255 in the page)
//   Number of entries (2 bytes)
//   Address complement (2 bytes)
//   Number of entries complement (2 bytes)
#define DMND_LOG_ADDR       (ALARM_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//-------------------------------------- Disturbance Log Address -------------------------------------------
//  8 bytes
//  0x00058 - 0x0005F
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define DIST_LOG_ADD        (DMND_LOG_ADDR + 8)

//---------------------------------------- Disturbance Log Address ----------------------------------------------
//  8 bytes
//  0x00060 - 0x00068
//
// This includes the following:
//   Address (2 bytes - sector, page, and 1/2 page addr of next open location in Flash
        //     b15..5: Sector address in Flash (4Kbytes each - ENERGY_SECTOR_START to ENERGY_SECTOR_END)
        //     b4..1: Page address in Flash (256 bytes each - 0x0 thru 0xF)
        //     b0: 1/2 page address (128 bytes, 0 = bytes 0 thru 127, 1 = bytes 128 thru 255 in the page)
//   Number of entries (2 bytes)
//   Address complement (2 bytes)
//   Number of entries complement (2 bytes)
//#define DISTURBANCE_LOG_ADDR       (DMND_LOG_ADDR + 8)            *** DAH 230314


//
//----------------------------------------------------------------------------------------------------------


//----------------------------------- Extended Capture Log Address -----------------------------------------
//  8 bytes
//  0x00060 - 0x00067
//
// In order to maintain consistency with all of the event address storage, the index and number of events
// are stored as words, not bytes, because the Summary Logs (500 maximum events) requires words
// This includes the following:
//   Index (2 bytes - index of the next open location in FRAM, high byte is 0)
//   Number of events (2 bytes, high byte is 0)
//   Index complement (2 bytes)
//   Number of events complement (2 bytes)
#define EXCAP_LOG_ADD       (DIST_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//---------------------------------------- Group 2 Setpoints -----------------------------------------------
//  32 bytes
//  0x00068 - 0x00087
//
// Reference file Setpnt_def.h for further information
#define SETP_SETA_GR2_COPY1_ADDR       (EXCAP_LOG_ADD + 8)

//
//----------------------------------------------------------------------------------------------------------


//---------------------------------------- Group 3 Setpoints -----------------------------------------------
//  36 bytes
//  0x00088 - 0x000AB
//
// Reference file Setpnt_def.h for further information
#define SETP_SETA_GR3_COPY1_ADDR       (SETP_SETA_GR2_COPY1_ADDR + STP_GRP2_SIZE)

//
//----------------------------------------------------------------------------------------------------------


//---------------------------------------- Group 11 Setpoints ----------------------------------------------
//  36 bytes
//  0x000AC - 0x000CF
//
// Reference file Setpnt_def.h for further information
#define SETP_SETA_GR11_COPY1_ADDR      (SETP_SETA_GR3_COPY1_ADDR + STP_GRP3_SIZE)

//
//----------------------------------------------------------------------------------------------------------


#define FIRSTBLK_END    (SETP_SETA_GR11_COPY1_ADDR + STP_GRP11_SIZE - 1)
//
//----------------------------------- End First Block Of Redundant Values ----------------------------------


//--------------------------------------------- Min/Max Values ---------------------------------------------
//  164 bytes
//  0x000D0 - 0x00173
//
                                            // Min and Max Demand Currents                  x000D0...x00137
#define MNMX_DMNDI_ADD  (FIRSTBLK_END + 1)                                  // chk/comp     x00137...x0013F
                                            // Max Demand Powers                            x00140...x0016B
                                                                        // "+ 8" is for the chksum and comp
#define MAX_DMNDP_ADD   (MNMX_DMNDI_ADD + sizeof(struct DEMAND_I_MIN_MAX_STRUCT) + 8) //    x0016C...x00173

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------------- Energy Storage ---------------------------------------------
//  248 bytes
//  0x00174 - 0x0026B
//
// Note, this is the (real-time) energy storage.  It is updated and written to FRAM every 200msec
// Note structs ENERGY_FLOATS and ENERGY_UINT64 do not include the checksum and complement
#define RES_SIZE       (sizeof(struct ENERGY_FLOATS))
#define NRG_SIZE       (sizeof(struct ENERGY_UINT64))
#define ENERGY_SIZE    (4 * (RES_SIZE + NRG_SIZE) + 8)      // "+ 8" is for the checksum and complement

  // Start of energy storage - 248 bytes
#define FRAM_ENERGY     (MAX_DMNDP_ADD + (sizeof(struct DEMAND_P_MAX_STRUCT) + 8))       // x00174...x0026B

                                            // Phase A residual - 20 bytes                  x00174...x00187
#define FRAM_ENGY_RESA  FRAM_ENERGY
                                            // Phase A energy - 40 bytes                    x00188...x001AF
#define FRAM_ENGY_NRGA  (FRAM_ENGY_RESA + RES_SIZE)
                                            // Phase B residual - 20 bytes                  x001B0...x001C3
#define FRAM_ENGY_RESB  (FRAM_ENGY_NRGA + NRG_SIZE)
                                            // Phase B energy - 40 bytes                    x001C4...x001EB
#define FRAM_ENGY_NRGB  (FRAM_ENGY_RESB + RES_SIZE)
                                            // Phase C residual - 20 bytes                  x001EC...x001FF
#define FRAM_ENGY_RESC  (FRAM_ENGY_NRGB + NRG_SIZE)
                                            // Phase C energy - 40 bytes                    x00200...x00227
#define FRAM_ENGY_NRGC  (FRAM_ENGY_RESC + RES_SIZE)
                                            // All Phases residual - 20 bytes               x00228...x0023B
#define FRAM_ENGY_ALLR  (FRAM_ENGY_NRGC + NRG_SIZE)
                                            // All Phases energy - 40 bytes                 x0023C...x00263
#define FRAM_ENGY_ALLE  (FRAM_ENGY_ALLR + RES_SIZE)
                                            // Checksum and complement - 8 bytes            x00264...x0026B
#define FRAM_ENGY_CHK   (FRAM_ENGY_ALLE + NRG_SIZE)

//
//------------------------------------------- End Energy Storage -------------------------------------------


//----------------------------------- Demand and Energy Logging Storage ------------------------------------
//  88 bytes
//  0x0026C - 0x002C3
//
// Note, this is for demand and energy logging storage.  It is used to temporarily hold the energy and
//   demand logging values (1/2 of a Flash page) until the values are written into Flash memory.  It is
//   written once every demand logging window.
                                            // Start of demand storage - 88 bytes           x00264...x002C3
#define DEMAND_SIZE    (sizeof(struct ENERGY_DEMAND_STRUCT))
#define FRAM_DEMAND     (FRAM_ENERGY + ENERGY_SIZE)

//
//----------------------------------------------------------------------------------------------------------


//------------------------------------- Waveform Capture Header Info ---------------------------------------
//  672 bytes
//  0x002C4 - 0x00563
//
// A waveform header consists of the following:
//  Number of samples (2 bytes) -   uint16_t NumSamples in struct FLASH_INT_REQ
//  EID (4 bytes)               -   uint32_t EID in struct FLASH_INT_REQ 
//  Time stamp (8 bytes)        -   struct INTERNAL_TIME TS in struct FLASH_INT_REQ

                                            // Alarm WF Capture Info - 14 x 21 = 294 bytes  x002C4...x003E9
#define ALARM_WF_INFO   (FRAM_DEMAND + DEMAND_SIZE)
                                            // Trip WF Capture Info - 14 x 21 = 294 bytes   x003EA...x0050F
#define TRIP_WF_INFO    (ALARM_WF_INFO + (WF_HEADER_SIZE * NUM_ALARM_WAVEFORMS))
                                            // Ext-Capture WF Cap Info - 14 x 7 = 98 bytes  x00510...x00571
#define EXTCAP_WF_INFO  (TRIP_WF_INFO + (WF_HEADER_SIZE * NUM_TRIP_WAVEFORMS))

//
//----------------------------------------------------------------------------------------------------------


//---------------------------------------- Summary Event Storage -------------------------------------------
//  7000 bytes
//  0x00572 - 0x020C9
//
// Summary Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Event Code              2 bytes
//
#define SUMMARY_EVENT_SIZE  14
#define SUMMARY_NUM_LOGS    500

#define SUMMARY_LOG_START   (EXTCAP_WF_INFO + (WF_HEADER_SIZE * NUM_EXTCAP_WAVEFORMS))
#define SUMMARY_LOG_END     (SUMMARY_LOG_START + (SUMMARY_NUM_LOGS * SUMMARY_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


//------------------------------------ Time Adjustment Event Storage ---------------------------------------
//  1100 bytes
//  0x020CA - 0x02515
//
// Time Adjustment Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Source of Time Change   1 byte
// Spare                   1 byte
// New Time Stamp          8 bytes
//
#define TIMEADJ_SIZE        10
#define TIMEADJ_EVENT_SIZE  22
#define TIMEADJ_NUM_LOGS    50

#define TIMEADJ_LOG_START   (SUMMARY_LOG_END + 1)
#define TIMEADJ_LOG_END     (TIMEADJ_LOG_START + (TIMEADJ_NUM_LOGS * TIMEADJ_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


// Trip, Test Trip, Alarm, and extended Capture Snapshots storage - Reference struct SNAPSHOT_METER in
//   Meter_def.h
//
// Snapshots Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Snapshots             202 bytes

#define SNAPSHOT_SIZE           202
#define SNAPSHOTS_EVENT_SIZE    214



//----------------------------------------- Trip Event Storage ---------------------------------------------
//  42800 bytes
//  0x02516 - 0x0CC45
//
// Trip Snapshot Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Snapshots             202 bytes
//
#define TRIP_NUM_LOGS      200

#define TRIP_LOG_START     (TIMEADJ_LOG_END + 1)
#define TRIP_LOG_END       (TRIP_LOG_START + (TRIP_NUM_LOGS * SNAPSHOTS_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------  Test Trip Event Storage ------------------------------------------
//  4280 bytes
//  0x0CC46 - 0x0DCFD
//
// Test Trip Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Snapshots             202 bytes
//
#define TESTTRIP_NUM_LOGS       20        // *** DAH  REDUCE THIS TO 15 (MAXIMUM NUMBER THAT ALLOWS TEST TRIP SUMMARY INFO TO BE RETURNED IN A SINGLE COMMAND)

#define TESTTRIP_LOG_START      (TRIP_LOG_END + 1)
#define TESTTRIP_LOG_END        (TESTTRIP_LOG_START + (TESTTRIP_NUM_LOGS * SNAPSHOTS_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


//-----------------------------------------  Alarm Event Storage -------------------------------------------
//  42800 bytes
//  0x0DCFE - 0x1842D
//
// Alarm Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Snapshots             202 bytes
//
#define ALARM_NUM_LOGS          200

#define ALARM_LOG_START         (TESTTRIP_LOG_END + 1)
#define ALARM_LOG_END           (ALARM_LOG_START + (ALARM_NUM_LOGS * SNAPSHOTS_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------  Disturbance Capture Storage --------------------------------------
//  3300 bytes
//  0x1842E - 0x19111
//
// Disturbance Capture Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes      // Disturbance exit time
// Entry Time              8 bytes      // Disturbance start time (prior to the delay time)
// Duration                4 bytes      // Disturbance Duration in milliseconds ** DAH COULD BE FLOAT OR INTEGER
// Value Code              2 byte       // Value of interest *** DAH  must have separate codes for OV and UV
// Spare                   2 byte
// Max (or min) Value      4 bytes
// Average value           4 bytes
// Thermal capacity        4 bytes      // If applicable, how close we came to tripping (maybe make a per cent?)
// EID of initiation event 4 bytes
//
#define DIST_EVENT_SIZE          44
#define DIST_NUM_LOGS            75

#define DIST_LOG_START          (ALARM_LOG_END + 1)
#define DIST_LOG_END            (DIST_LOG_START + (DIST_NUM_LOGS * DIST_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------


//--------------------------------------  Extended Capture Snapshots ---------------------------------------
//  4100 bytes
//  0x19112 - 0x1A5F7
//
// Extended Capture Event Package
// Event ID                4 bytes
// Time Stamp              8 bytes
// Snapshots             202 bytes
//
#define EXTCAP_NUM_LOGS         25

#define EXTCAP_LOG_START       (DIST_LOG_END + 1)
#define EXTCAP_LOG_END         (EXTCAP_LOG_START + (EXTCAP_NUM_LOGS * SNAPSHOTS_EVENT_SIZE) - 1)

//
//----------------------------------------------------------------------------------------------------------



//----------------------------------- Second Block Of Redundant Values -------------------------------------
//  200 bytes total
//  0x1A5F8 - 0x1A6BF
#define SECONDBLK_OFFSET    ((EXTCAP_LOG_END + 1) - FIRSTBLK_START)                         // x1A5F8

#define SETP_SETA_GR2_COPY2_ADDR       (SETP_SETA_GR2_COPY1_ADDR + SECONDBLK_OFFSET)        // x1A660

#define SETP_SETA_GR3_COPY2_ADDR       (SETP_SETA_GR3_COPY1_ADDR + SECONDBLK_OFFSET)        // x1A680

#define SETP_SETA_GR11_COPY2_ADDR      (SETP_SETA_GR11_COPY1_ADDR + SECONDBLK_OFFSET)       // x1A6A4

#define SECONDBLK_END       (FIRSTBLK_END + SECONDBLK_OFFSET)                               // x1A6C7

//
//----------------------------------------------------------------------------------------------------------



//----------------------------------- Thermal Memory "Bucket" Storage --------------------------------------
//  4 bytes total
//  x1A6C8 - 1A6CB
#define TM_CAPACITY                     (SECONDBLK_END + 1)                               // x1A6C8 - x1A6C9
#define TM_CAPACITY_COMP                (TM_CAPACITY + 2)                                 // x1A6CA - x1A6CB



//----------------------------------- Factory buffer variables ---------------------------------------------
#define FAC_BUF_START    (TM_CAPACITY_COMP + 2)              // x1A6CC
#define ETU_SERIAL_NUM   FAC_BUF_START                       // factory buf 23  x1A6CC-x1A6CF
#define ETU_HW_VERSION   (ETU_SERIAL_NUM + 4)                // factory buf 24  x1A6D0-x1A6D1
#define MFG_LOC_DATE     (ETU_HW_VERSION + 2)                // factory buf 30  x1A6D2-x1A6DD

#define FACTOR_CHKSUM    (MFG_LOC_DATE + 12)                 // checksum/comp   x1A6DE-x1A6E1
#define FAC_BUF_END      (FACTOR_CHKSUM + 4)                 // x1A6E2

// length of factory buf = 22
#define START_COPY2_FAC1 FAC_BUF_END                         // Second copy: x1A6D8 - x1A6ED
#define END_COPY2_FAC1   ((FAC_BUF_END-FAC_BUF_START)+ START_COPY2_FAC1)  // x1A6EE
#define START_COPY3_FAC1 END_COPY2_FAC1                      // Third copy:  x1A6EE - x1A703
#define END_COPY3_FAC1   ((FAC_BUF_END-FAC_BUF_START)+ START_COPY3_FAC1)  // x1A704

#define PSWD_ADDRESS      END_COPY3_FAC1                     // x1A704-x1A711
#define FRAM_TEST_VAL    (PSWD_ADDRESS + PASSWORD_SIZE)      // x1A712-x1A713

#define FRAM_UNPROTEND   (FRAM_TEST_VAL + 2)                 // x1A714

                                            // FRAM End                                    x1A714


                                            // Unused FRAM locations:                      x1A714...x2FFFF




//----------------------------------------------------------------------------------------------------------
//          Extended Capture (SPI1) FRAM Definitions
//----------------------------------------------------------------------------------------------------------
//
//          REFERENCE: Cypress FM25V02, 256Kb Serial 3V FRAM datasheet Doc 001-84497 Rev C February, 2015
//
//          FRAM is organized as a 32k x 8 array.
//          Write protection is provided by two bits in the status register as follows:
//             Protect Range     BP1   BP0
//                  none          0     0     Waveform samples
//             0x6000 - 0x7FFF    0     1     
//             0x4000 - 0x7FFF    1     0
//             0x0000 - 0x7FFF    1     1
//       
// NOTES: Defined block sizes are in bytes
//        FRAM starting addresses are in bytes
//        Write protection is not used for this FRAM
//

#define EXTCAP_FRAM_START           0x00000
//------------------------------- RMS Snapshots Extended Capture FRAM Storage ------------------------------
//
//
// One Cycle area
#define EXTCAP_ONECYCLE_SIZE           44              // 11 float values (11 by 4 bytes)
#define EXTCAP_ONECYCLE_NUM_LOGS       360              //6s duration for one cycle measurement

#define EXTCAP_ONECYCLE_LOG_START     (EXTCAP_FRAM_START)
#define EXTCAP_ONECYCLE_LOG_END       (EXTCAP_ONECYCLE_LOG_START + (EXTCAP_ONECYCLE_NUM_LOGS * EXTCAP_ONECYCLE_SIZE) - 1)

// 200msAniversary Area
#define EXTCAP_TWOHUNDRCYCLE_SIZE           EXTCAP_ONECYCLE_SIZE
#define EXTCAP_TWOHUNDRCYCLE_NUM_LOGS       300         //60s duration for 200ms for 60s measurement

#define EXTCAP_TWOHUNDRCYCLE_LOG_START     (EXTCAP_ONECYCLE_LOG_END + 1)
#define EXTCAP_TWOHUNDRCYCLE_LOG_END       (EXTCAP_TWOHUNDRCYCLE_LOG_START + (EXTCAP_TWOHUNDRCYCLE_NUM_LOGS * EXTCAP_TWOHUNDRCYCLE_SIZE) - 1)

// EID
#define EXTCAP_EID_START                (EXTCAP_TWOHUNDRCYCLE_LOG_END + 1)
#define EXTCAP_EID_END                  (EXTCAP_EID_START + 3)

// Time Stamp for one-cyc values
#define EXTCAP_ONECYC_TS_START          (EXTCAP_EID_END + 1)
#define EXTCAP_ONECYC_TS_END            (EXTCAP_ONECYC_TS_START + 7)

// Time Stamp for 200msec values
#define EXTCAP_TWOHUN_TS_START          (EXTCAP_ONECYC_TS_END + 1)
#define EXTCAP_TWOHUN_TS_END            (EXTCAP_TWOHUN_TS_START + 7)
//

// Address of test value for manufacturing purposes
#define EXTCAP_TEST_START               (EXTCAP_TWOHUN_TS_END + 1)
#define EXTCAP_TEST_END                 (EXTCAP_TEST_START + 2)

//----------------------------------------------------------------------------------------------------------

