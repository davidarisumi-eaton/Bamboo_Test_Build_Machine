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
//  FIRMWARE DRAWING:   ????????    This drawing combines the unprogrammed STM32F407 with the code's
//                                  flash programming file to produce an "assembly group" that is the
//                                  programmed device.
//
//  PROCESSOR:          ST Micro STM32F407
//
//  COMPILER:           IAR C/C++ Compiler for ARM - v8.40.1.21539
//                      IAR Embedded Workbench from IAR Systems
//
//  MODULE NAME:        DispComm.c
//
//  MECHANICS:          Program module containing the subroutines to handle display communications
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.00   150616  DAH File Creation
//   0.16   160818  DAH - Added Display communications
//                          - Added struct Disp
//                          - Added Disp_VarInit() and Disp_Tx()
//   0.19   170223  DAH - Revised display communications protocol.  Added CRC, message length, escape chars
//                          - Disp_Tx() revised
//                          - CRC_TABLE[] added
//                      - Changed message body to send 200msec and one-cycle currents instead of a test
//                        string
//                          - Disp_Tx() revised
//                      - Revised the communications to use DMA instead of interrupts
//                          - Disp_VarInit() revised (several variables eliminated from struct DISPLAYVARS)
//                          - Disp_Tx() revised
//                          - Moved location of structure Disp to SRAM2 to support DMA (default is CCM,
//                            which does not work with DMA)
//                      - Added display receptions using the comms protocol that transmissions use.  Added
//                        support for two test commands: start test injection and stop test injection
//                          - Disp_Rx() added
//   0.22   180420  DAH - Moved "include "Iod_def.h" " statement before "#include "Meter_def.h" " because
//                        Meter_def.h uses INTERNAL_TIME, which is defined in Iod_def.h
//   0.28   181030  DAH - Modified the communications protocol for the planned new high-speed CAM protocol
//                          - Added Assemble_TxPkt(), AssembleRxPkt(), and CalcCRC().  Presently,
//                            AssembleRxPkt() has test code to transfer the transmitted message to the rx
//                            buffer
//                          - Replaced CRC_TABLE[] with CRC_TABLE1[]
//                          - Revised Disp struct to accomodate the new protocol
//                          - Revised Disp_Tx() to assemble messages per the new protocol.  Presently, this
//                            subroutine only transmits test messages on command via the Test Port
//                          - Revised Disp_Rx() to decode messages per the new protocol.  Presently, this
//                            subroutine only supports receiving the last currents that were transmitted and
//                            comparing the two currents to ensure communications are working properly
//   0.30   190314  DAH - Added support for an Initiate Test Injection Execution Action command
//                          - case 8 added to Disp_Rx()
//   0.32   190726  DAH - Simplified CalcCRC() and moved it to the global section as it will be used by the
//                        Modbus communications handler also
//                      - Renamed Cur200msec to Cur200msFltr to distinguish it from unfiltered currents
//   0.36   200210  DAH - Renamed file from Display.c to DispComm.c
//                      - Display_def.h renamed to DispComm_def.h
//                      - Disp_VarInit() renamed to DispComm_VarInit()
//                      - Disp_Rx() renamed to DispComm_Rx()
//                      - Disp_Tx() renamed to DispComm_Tx()
//                      - const DISP_RXBUFSIZE renamed to const DISPCOMM_RXBUFSIZE
//                      - Renamed Assemble_TxPkt() to AssembleTxPkt()
//                      - Renamed struct DISPLAYVARS Disp to struct DISPCOMMVARS DPComm
//                      - Added RTD buffers to the display processor communications
//                          - Disp.RxState (used in AssembleRxPkt() and DispComm_VarInit()) renamed to
//                            DPComm.AssRxPktState
//                          - Revised DispComm_Rx() to handle different commands and RTD buffers
//                              - DPComm.RxState added
//                              - RTD Buffer 0 is presently supported as a Write Response (from a Read
//                                Request by the Display Processor)
//                          - Deleted SendCurrents as Real-Time Data Buffers will be transmitted to the
//                            Display Processor upon request, rather than pushed up periodically.
//                      - In DPComm_Rx(), eliminated call to CalcCRC(), as the CRC is computed in
//                        AssembleRxPkt() as the message is assembled
//                      - Deleted CalcCRC() as it is no longer used (moved to Modbus.c)
//                      - Moved CRC_TABLE1[] from local to global section, as it is now used in Modbus.c
//                      - Corrected bug in encoding the transmit packets.  When inserting the CRC bytes, the
//                        segment index was not checked to see if a segment code needed to be entered.  This
//                        resulted in occasional CRC errors.  In addition, when inserting the last segment
//                        code, the char index is checked for whether it has exceeded 126 chars.
//   0.37   200604  DAH - In state 3 of AssembleRxPkt(), added code to make sure the Rx buffer index is at
//                        the next open position in the buffer when the end of a message is reached, in the
//                        (unlikely) event there are additional characters in the buffer after the end of a
//                        message.  If this is not done, the code will process the additional characters
//                        instead of the new message.  This bug was found by halting the protection
//                        processor while the display processor was still transmitting.  The protection
//                        processor never "caught up" to the new message, and continuously processed and
//                        responded to old messages instead of the newest one.  The sequence numbers were
//                        off, and communications were stalled.
//                      - Renamed DPComm.XmitReqBitMsk to DPComm.XmitReqClrMsk and simplified its usage
//                      - Added code to process Read Delayed requests
//                          - In DispComm_Rx(), combined the code that processes Read Immediate and Read
//                            Delayed requests
//                          - Replaced ProcReadReq() with ProcReadReqImm() and ProcReadReqDel() in
//                            DispComm_Tx()
//                      - In DispCommRx(), deleted DPComm.RcvReqBitMsk as it is no longer used
//                      - Added code to process Write Response Requests from an outside process.  Usually
//                        the outside process is a Flash or FRAM data retrieval for setpoints or events in
//                        response to a Read Delayed Request.
//                          - Revised DispComm_Tx() accordingly
//                          - Added AssembleAck1() to handle Ack responses to delayed requests
//                      - Added include of Events_def.h for compilation purposes (for FRAM_Flash_def.h)
//   0.38   200708  DAH - The RTD buffers (stored in the Display Processor's DCI) are now "pushed up" by the
//                        Protection Processor rather than "pulled up" by the Display Processor.  Note, we
//                        still support (for now) Read Immediates of the Real-Time Data buffers
//                          - Added DPComm.RTD_XmitTimer[] to determine when to send the RTD buffers
//                          - Added constant RTDBUF_INTERVAL_TIME to set the period for transmitting the RTD
//                            buffers
//                          - Revised DispComm_Tx() to check the timers and generate Writes Without
//                            Acknowledge transmissions of the RTD buffers
//                          - Added code to DispComm_VarInit() to initialize the RTD transmission timers
//                          - Modified AssembleRTDBuffer() so it can be used for both responses and unsolicited
//                            writes
//                              - Added source/destination address as an input parameter
//                      - Revised DispComm_Tx() to wait for the Turnaround Time or an Ack after transmitting
//                        a message
//                      - Fixed bug in AssembleAck1().  The sequence number needs to be the seq number that
//                        was received in the original (delayed request) message, and is saved in
//                        DPComm.RxMsgSav[2]
//                      - Revised DispComm_Rx() to have the test injection command be an Execute Action With
//                        Acknowledge instead of an Execute Action Without Acknowledge
//                      - Added test code to to test receiving Write With Ack and Execute Action With Check
//                        commands
//                          - Added Check_Status to struct DPComm.  This is used to hold the status of the
//                            execute action with check command.  DispComm_VarInit() revised to initialize
//                            DPComm.Check_Status
//                          - Added buffer type DP_BUFTYPE_CHECK to ProcReadReqImm() to support reading the
//                            status check byte
//                          - Revised DispComm_Rx() to add test code in Write With Acknowledge command state
//                            to store 8 bytes of test data.  This data is eventually written to Flash
//                          - Added DP_CMND_EXWCHK state to DispComm_Rx() to support execute action with
//                            check command to save setpoints
//                      - Deleted support for Write Without Acknowledge commands
//                          - Revised DispComm_Rx() accordingly
//                          - Deleted test variables Ix_In and Ix_Iout as they are no longer used
//                      - In DispComm_Tx(), moved TX_ACK flag into DPComm.Flags and deleted
//                        DPComm.XmitReqClrMsk as it is no longer used
//                      - Corrected code in DispComm_Tx() for generating Write Response Messages (in
//                        response to Read Delayed receptions)
//                      - General code development and cleanup
//                          - Added constant TURNAROUND_TIME
//                          - In DispComm_Rx(), deleted code that computes the CRC.  This is not required,
//                            as the CRC is computed as the message bytes are received and should sum to 0
//                            over the final message
//   0.39   200729  DAH - Further enhancements to the protocol implementation.  Analysis showed that to
//                        ensure proper full-duplex operation, the wait time after transmitting must be
//                        initialized properly.  The Response Time should be greater than the sum of the
//                        maximum Turnaround Time for both devices plus the maximum transmission time for
//                        the longest message.
//                          - Renamed DPComm.TurnaroundTimer to DPComm.XmitWaitTimer
//                          - Added DPComm.WaitTime and RESPONSE_TIME
//                          - Revised DispComm_Tx() to intialize XmitWaitTimer to either the Turnaround Time
//                            (if no response is expected) or the Response Time (if a response is expected)
//                      - Fixed bug in DispComm_Tx().  GEN_WRITERESP was not cleared after assembling the
//                        Write Response message
//                      - Corrected bugs in DispComm_Tx() to ensure a new message does not overwrite an
//                        existing message until it has been processed
//                          - Added code to go to State 1 for commands that require an ACK response.  This
//                            ensures the proper ACK is assembled and transmitted before a new message is
//                            received
//                          - Deleted code to go to State 1 when an ACK is received.  This is unnecessary
//   0.41   201028  DAH - In DispComm_Tx(), made Write Responses the highest priority.  Testing showed that
//                        if ACKS or Read Requests are higher, they can starve the Read Delayed response if
//                        the Display Processor has continuous requests to transmit Read Immediates or
//                        Execute Actions.  The Display Processor transmits these while waiting for the
//                        Write Response to the Delayed Read.  In this scenario, the Protection Processor
//                        won't ever get to transmit the response - it will keep servicing the Read
//                        Immediate requests.  This change fixes that.
//                      - In ProcReadReqDel() added code to process a setpoints read request (for test
//                        purposes)
//   0.42   201202  DAH - Moved RTC and Internal Time code from Iod.c to a new module, RealTime.c
//                          - Added include of RealTime_def.h
//   0.43   210115  DAH - Added support for 61850 communications with the Display Processor
//                          - Added struct DISPCOMM61850VARS DPComm61850
//                          - Modified DispComm_VarInit() to initialize the appropriate 61850 variables
//                          - Added DispComm61850_Tx(), DispComm61850_Rx(), Assemble61850TripMsg(),
//                            Assemble61850ZSIMsg(), Assemble61850AckMsg(), Assemble61850XferMsg(),
//                            Assemble61850SetpMsg(), Assemble61850StateMsg(), Assemble61850ValsMsg()
//                          - Added AssembleTxPkt1() and AssembleRxPkt1().  Note, this are identical to
//                            AssembleTxPkt() and AssembleRxPkt() except for the input pointer definitions.
//                            These subroutines should eventually be deleted and incorporated into
//                            AssembleTxPkt() and AssembleRxPkt() when the 61850 comms code is finalized.
//                          - Added test code and vars ("ddxxx" vars) to monitor the 61850 communications
//                            between the two processors.  This code should eventually be deleted
//                      - Revised DispComm_Tx() to clear the corresponding event flags (per RM0090 Reference
//                        Manual) before enabling the stream
//   0.44   210324  DAH - Additional support for 61850 communications with the Display Processor
//                          - Revised AssembleRxPkt1() to handle multiple "stacked" messages.  Previous code
//                            discarded characters received after the end of packet character was received.
//                            - Added CharCount as a global variable to struct DPCOMMRXVARS to hold the
//                              number of chars left to process in the Rx FIFO after an end of packet char
//                              has been received.
//                            - Deleted code that moved the buffer index to the end of the Rx FIFO
//                              (discarding new chars) after an EOP was received
//                          - Changed the sequence numbers from variable numbers to fixed numbers
//                            corresponding to the GOOSE message type.  The reason is that each GOOSE
//                            message is fixed and is a broadcast message.  For each type, each message is
//                            the same and there is no need for a sequence number.
//                            - Revised DispComm61850_Rx(), Assemble61850ZSIMsg(), and
//                              Assemble61850TripMsg() to eliminate variable sequence numbers
//                          - Revised DispComm61850_Tx() to increase wait time from 2 to 8 for setpoint,
//                            transfer, health, and values messages
//                          - Added test messages for Assemble61850XferMsg(), Assemble61850SetpMsg(),
//                            Assemble61850StateMsg(), Assemble61850ValsMsg()
//                          - Revised DispComm61850_Rx() to handle the different message types that were
//                            added for transmission - transfer, setpoint, health (status), values
//                          - In DispComm61850_Rx(), fixed the message length check for a trip message
//                          - Added code to delay one sample interrupt after a high speed comms transmission
//                            instead of allowing a high-speed transmission every sample interrupt.  The
//                            GOOSE testing in which three messages are sent in succession every second
//                            uncovered an issue.  Sometimes the second (ZSI) GOOSE transmission was being
//                            missed by the display processor.  It was determined that it takes too long for
//                            the display processor to respond to the interrupt, so code was added to the
//                            protection processor to delay one sample interrupt after transmitting before
//                            checking whether there is another GOOSE message to transmit.  The extra
//                            207usec is ok, considering the total time is still under 4msec.  With the new
//                            code, a test was conducted in which three consecutive high-speed comms GOOSE
//                            messages were transmitted every second for a few hours (over 15000 total
//                            passes), and there were no crashes or missed publications or receptions.
//                              - xmitwait was added
//                              - DispComm61850_Tx() revised
//                      - Added special test code to DispComm_Tx() transmit a WRACK message to the display
//                        processor.  The command is used to test the DMA operation in the H7 processor.
//                        This code is presently commented out.  It is only for H7 porocessor testing, and
//                        once the DMA is running, it will be deleted.
//   0.51   220304  DAH - Current and voltage unbalance changed from 200msec values to one-cycle values
//                          - In AssembleRTDBuffer(), replaced Cur200msFltrUnbal with CurOneCycUnbal and
//                            replaced VolAFE200msFltrUnbal with VolAFEOneCycUnbal
//   0.54   220330  DAH - CurOneCycUnbal renamed to CurUnbalTot in AssembleRTDBuffer()
//                      - VolAFEOneCycUnbal renamed to VolUnbalTot in AssembleRTDBuffer()
//   0.55   220420  DAH - Finalized real-time data buffer definitions and added these to the communications
//                          - Added constant arrays DPCOMM_RTD_ADDR_BUFxx[ ]
//                          - Revised AssembleRTDBuffer()
//   0.56   220526  DAH - Added harmonics to the real-time data communications
//                          - DPCOMM_HARM[] added
//                          - ProcReadReqImm() revised slightly (2 unused info bytes in all buffers)
//                          - Support for harmonics added to AssembleRTDBuffer()
//   0.57   220628  DAH - Reversed the order of Ig and In in DPCOMM_RTD_ADDR_BUF0[] and
//                        DPCOMM_RTD_ADDR_BUF0_TESTINJ[] make it easier for the display processor when
//                        interfacing with the LCD
//                      - Replaced &CurOneCyc (error) with &CurOneCycIg in DPCOMM_RTD_ADDR_BUF0_TESTINJ[]
//   0.59   220831  DAH - Revised ProcReadReqDel() to replace code that sets S2NF_DP_REQ with a call to
//                        GetEVInfo() and code that assembles the test message directly
//                      - Revised ProcReadReqDel() to hold the Write Response message in a separate buffer,
//                        TxDelMsgBuf[], instead of SPI2_buf[].  SPI2_buf[] may be used elsewhere in the
//                        foreground before the Write Response message is transmitted.
//                        Revised DispComm_Tx() to use TxDelMsgBuf[] for the Write Response msg (response to
//                        a Read Delayed Request message) instead of SPI2_Buf[]
//   0.62   221027  DAH - Added support for setpoints buffer read commands
//                          - Added AssembleSetpBuffer()
//                          - Added case DP_BUFTYPE_SETP to ProcReadReqImm().  This state supports setpoints
//                            buffer types
//                      - Deleted code to initiate setpoints write to Flash from DP_CMND_EXWCHK state in
//                        DispComm_Rx().  This is no longer needed.  Setpoints are stored in Frame FRAM only
//                      - Added support for setpoints buffer write commands
//                          - Revised DP_CMND_WRWACK state in DispComm_Rx() to handle setpoints write cmnds
//   0.64   221118  DAH - Revised AssembleSetpBuffer() to overwrite the Group1 read-only setpoints with the
//                        corresponding Group0 setpoints when retrieving them from the Frame FRAM (i.e.,
//                        when they are not the active setpoints set)
//                      - In AssembleSetpBuffer(), calls to Get_Setpoints() now include the setpoints set
//                      - Corrected bug in DispComm_Rx().  When active setpoints group is written, setpoint
//                        status (SetpointsStat) must be updated
//   0.69   230220  DAH - Added AssembleEvntBuffer() to ProcReadReqImm() for reading events
//                      - Began removing events from ProcReadReqDel(), as these no longer have to be delayed
//                        reads
//   0.72   230320  DAH - Added support for factory test commands to read and write the frame rating
//                          - Added AssembleFactoryBuffer()
//                          - In ProcReadReqImm(), added DP_BUFTYPE_FACTORY state with call to
//                            AssembleFactoryBuffer()
//                          - In DispComm_Rx(), added code to support Factory Write of Frame Rating
//                      - Added demand window type and interval to DPCOMM_RTD_ADDR_BUF2[]
//                      - Added Time of Last Reset for voltage and current min/max to DPCOMM_RTD_ADDR_BUF2[]
//                      - Added support for Group 11 setpoints
//                          - DispComm_Rx() revised
//   24     230323  DAH - Added preliminary support for PriSecCause status
//                          - StatusCode added to DPCOMM_RTD_ADDR_BUF0[]
//                      - Added support for displaying long delay time to trip
//                          - Added LD_TimeToTrip to DPCOMM_RTD_ADDR_BUF0[] and
//                            DPCOMM_RTD_ADDR_BUF0_TESTINJ[]
//   25     230403  DAH - Added reading Summary Event Logs
//                          - AssembleEvntBuffer() revised
//                      - Revised ProcReadReqDel() to support read waveforms command
//                  KT  - Added 61850 GOOSE Capture command and IEC61850 GOOSE devlopment
//                          - Added Assemble61850CaptureCommandMsg()
//                          - Modified DispComm61850_Rx()
//   26     230403  DAH - Added Extended Capture one-cycle and 200msec proc-proc Read Commands
//                          - AssembleEvntBuffer() revised
//   27     230404  DAH - Added support for Disturbance Capture Read Commands
//                          - AssembleEvntBuffer() revised
//   28     230406  DAH - In ProcReadReqDel(), removed code that sets the buffer length for valid waveform
//                        reads.  It is set in SPI1_Flash_Manager() when the waveform has been retrieved
//                      - Added code to support GOOSE capture command
//                          - In DispComm61850_Rx() added flags to initiate extended and disturbance
//                            captures, and to end disturbance capture, when GOOSE capture command received
//   29     230406  DAH - Fixed bug in AssembleEvntBuffer() with retrieving Extended Capture timestamp
//                      - Corrected bugs in calling AssembleTxPkt() with message lengths greater than 126.
//                        The source data buffer cannot be TxBuf[] if the message length is greater than
//                        126.  This can cause the buffer to be corrupted.
//                        Also checked calls to AssembleTxPkt1().  It is ok (GOOSE messages are less than
//                        126 bytes long)
//                      - In ProcReadReqDel(), fixed bug reading waveforms (Tx buffer index is u16, not u8)
//   30     230412  DAH - Fixed bug encoding packets in AssembleTxPkt() and AssembleTxPkt1()
//                      - Fixed bug retrieving packets for 6sec and 60sec values in AssembleEvntBuffer()
//   38     230518  DAH - Fixed bug in AssembleEvntBuffer() in assembling read response for disturbance logs
//   40     230531  DAH - Improved readability of AssembleEvntBuffer() by moving code to assemble the
//                        summary events response into a new subroutine, AssembleSummaryEvntTxBuffer()
//                      - Improved readability of AssembleEvntBuffer() by moving the code to assemble the
//                        RMS profile response into a new subroutine, AssembleProfileEvntTxBuffer()
//                      - Improved readability of AssembleEvntBuffer() by moving the code to assemble the
//                        Disturbance Capture response into a new subroutine, AssembleDistCapTxBuffer()
//                      - Revised Disturbance Capture response (eliminated EID info, added search by EID)
//                      - Added support for reading snapshot summaries and snapshot logs
//                          - AssembleSnapshotSummaryTxBuffer() and AssembleSnapshotTxBuffer() added
//                          - AssembleEvntBuffer() revised
//   44     230623  DAH - Measured and added max execution time for ProcReadReqDel()
//                      - ProcReadReqDel() was altered to use the modified EventGetWfEIDs() subroutine to
//                        search for the requested EID
//                      - AssembleEvntBuffer() was altered because of the changes to EventGetWfEIDs()
//                      - AssembleSummaryEvntTxBuffer() and AssembleDistCapTxBuffer() modified to correct
//                        bug when number of events to return is zero (parameter in AssembleTxPkt() must be
//                        TRUE, not FALSE)
//   46     230703  DAH - Added AssembleWFCapEIDsTxBuffer() to support reading Waveform Capture EIDs
//                      - Added call to AssembleWFCapEIDsTxBuffer() in AssembleEvntBuffer() for Buffer IDs
//                        8, 9, and 10
//   58     230810  DAH - Added support for setpoint set changes
//                          - Added EA w/ Ack command to change the active setpoint set in DispComm_Rx()
//                      - Added Vbattery to RTD buffer 3 to send to display processor
//                      - Revised DispComm_Rx() to update protection parameters whenever setpoints are
//                        written
//   66     230825  DAH - Added support for internal and external diagnostic buffers
//                          - Added DPTxReqFlags to signal when to transmit 5-minute average values
//                            (RTD Buf 3) or internal diag buffer (RTD Buf 17)
//                          - Revised AssembleRTDBuffer() to send either RTD Buffer 3 or 17 whenever RTD
//                            Timer 3 is 0, depending on DPTxReqFlags
//                          - Revised AssembleRTDBuffer() to support RTD Buffer 18.  This is transmitted as
//                            a Write Response when the corresponding Read Immediate command is received
//                          - Revised DispComm_Tx() to only reset the Buffer 3 timer
//                            (DPComm.RTD_XmitTimer[3]) if neither RTD Buffer 3 nor 17 needs to be
//                            transmitted
//   69     230828  DAH - Added transmit time execute action command
//                          - Added struct INTERNAL_TIME DP_OutSyncTime, IntSyncTime
//                          - Added subroutine AssembleExActBuffer()
//                          - Revised DispComm_Tx() to process a request to transmit time, initiate the
//                            execute action transmission, and toggle the sync time port line
//                          - Revised DispComm_Rx() to process a received execute action command to set time
//   82     230928  DAH - Cleaned up DispComm_Rx() by moving setpoints and factory configuration
//                        write-handling code into separate subroutines, ProcWrSetpoints() and
//                        ProcWrFactoryConfig()
//                      - Revised ProcWrFactoryConfig() to handle display processor writes of the firmware
//                        version info.  Added DispProc_FW_Rev, DispProc_FW_Ver, DispProc_FW_Build to hold
//                        the display processor's firmware revision info.
//                      - Added Buffer 19 to AssembleRTDBuffer().  This buffer holds the protection
//                        processor and override micro firmware version info.
//                      - Deleted firmware version from FacBuf_Info[ ], as it is not stored in FRAM
//   94     231010  DAH - Corrected FacBuf_Info[][] per input from Ashley
//                      - Cleaned up DispComm_Rx() by moving execute actions with acknowledge code into a
//                        separate subroutine, ProcExActWAck()
//                      - Added code to ProcExActWAck() to handle test injection commands
//                      - Revised ProcReadReqImm() to handle health and diagnostics buffers for test
//                        injection results.  Added call to AssembleDiagBuffer()
//                      - Removed user waveform capture code from ProcExActWAck() and into a separate
//                        subroutine in Meter.c.  Also removed all related variable (UserWF_xx) definitions
//                      - Moved temperature from DPCOMM_RTD_ADDR_BUF0[] to DPCOMM_RTD_ADDR_BUF3[]
//                      - Added one cycle currents to DPCOMM_RTD_ADDR_BUF0[]
//   96     231012  DAH - Added one-cycle currents to DPCOMM_RTD_ADDR_BUF0_TESTINJ[]
//                      - Revised ProcExActWAck() to correct TestInjVars.Status handling
//   98     231017  DAH - Revised DispComm_VarInit() to initialize TestInjVars.Type and .Status
//   99     231023  BP  - Added some Secondary Injection code
//                      - Moved Secondary Injection defines to DispComm_def.h
//   108    231108  DAH - Added include of Flags_def.h for BITxx definitions
//                      - Deleted unused code from DispComm_Rx()
//   113    231128  BP  - Setpoints1 Style 2 is now writeable for Digitization
//   116    231129  MAG - Modified ProcExActWAckf() handling of enable/disable Mainenance Mode
//                      - Modified AssembleFactoryBuffer() as per Mila to fix USB lockup when reading FW ver
//   125    231212  BP  - Added Ashley's fix for Breaker Config write
//   129    231213  MAG - Various changes due to splitting Setpoints10 into Setpoints10 and Setpoints13
//                      - Added call to Verify_Setpoints() for setpoint changes from display processor
//                      - Added Modbus UART reset for communications setpoint changes
//                      - Changed SETP_ACTIVE_SET to SETP_ACTIVE_SET_ADDR to match naming convention
//                      - Tweaked Dispable Maintenance Mode handling
//                      - Consolidated handling of Group0/Group1 read-only setpoints into GetSetpoints()
//   133    231219  DAH - Revised ProcExActWAck().  For setpoint set change, added code to check the return
//                        status from the call to Load_SetpGr2_LastGr() and enter an event if necessary
//                      - In ProcExActWAck(), replaced calls to Write_DefaultBrkConfig() and
//                        Write_DefaultCriticalBrkConfig() with call to Brk_Config_DefaultInit()
//                      - In ProcWrFactoryConfig(), deleted code that saves non-critical configuration
//                        values in Break_Config.config.xxx.  Only the critical values are saved in RAM.
//                        Combined bufid states accordingly
//                        Also added code to write the critical config values to the critical section of
//                        Frame FRAM (new section for PXR35 added), not just the large breaker config
//                        section
//                      - In AssembleFactoryBuffer(), added code to overwrite values critical breaker config
//                        values with the values in RAM when the breaker config values are retrieved from
//                        Frame FRAM
//                      - In ProcExActWAck(), HEALTH_DAT_SIZE was changed to FRAME_FRAM_HEALTH_DAT_SIZE
//                      - Revised AssembleDiagBuffer() to add support for read of internal time parameters
//   141    240115  BP  - Fixed the Cancel Test bug for HW Sec Injection
//   142    240119  DAH - Revised ProcWrFactoryConfig() to update read-only breaker frame setpoints if new
//                        breaker frame is written in configuration
//                      - Revised ProcWrFactoryConfig() to NAK if invalid buffer number is received
//                      - Revised ProcWrFactoryConfig() to handle additional buffers.  Merged in Ashley's
//                        code to process calibration values
//                      - Major revisions to ProcWrFactoryConfig() and AssembleFactoryBuffer() to process
//                        calibration values correctly if stored in both FRAM and Flash
//                      - Revised ProcWrFactoryConfig() and AssembleFactoryBuffer() to add factory buffers
//                        43 and 44 to support reading and writing the Standard and Max Instantaneous Trip
//                        Setting configuration values to PXR25 and PXR35 critical breaker config sections
//                        in Frame FRAM 
//                      - Moved call of Flash_Read_ID() from AssembleFactoryBuffer() and into initialization
//                        code.  It cannot be called during normal operation unless it goes through
//                        SPI1_Flash_Manager(), because the Flash chip is accessed in the 1.5msec interrupt.
//                        The value is now read from Flash in initialization, stored in global
//                        union Flash_ID, and used in AssembleFactoryBuffer()
//   143    240122  DAH - In FacBuf_Info[][], revised parameters for buffers 23, 24, and 30 (FAC_BUF_START
//                        parameters)
//   144    240123  DAH - Revised ProcWrFactoryConfig() to update checksum when changing cal constants
//   149    240131  DAH - Renamed NUM_RTD_BUFFERS to NUM_RTD_TIMESLICES
//                      - Revised RTD buffer transmissions to free up time for other transmissions.
//                        Previously, each of the 11 buffers had its own time slice.  The new code reduces
//                        number of time slices from 11 to 7.  Up to 3 min/max buffers share time slices, so
//                        these are transmitted every 750msec instead of every 250msec.  Metered values are
//                        still transitted every 250msec.  This opens up a >100msec window for other comms
//                        (such as requests from the display processor) to be serviced.  A bug was seen
//                        (JIRA item 1899) where under certain conditions, RTD buffer comms was locking out
//                        all other comms
//                          - Replaced AssembleRTDBuffer() with subroutines BuildRTDBufByTSlice() and
//                            BuildRTDBufByBufnum().  BuildRTDBufByTSlice() handles periodic writes
//                            initiated in DispComm_Tx().  BuildRTDBufByBufnum() handles Read Requests
//                            initiated in ProcReadReqImm()
//                          - Revised DispComm_VarInit() to initialize DPTxReqFlags to 0 (all flags cleared)
//                          - Revised ProcWrFactoryConfig() to set both the firmware rev flag and the
//                            external diagnostic flag, and to clear RTD Timer 3 to initiate transmissions
//                            of the firmware version and the external diagnostics when the display
//                            processor's firmware version buffer is received
//                      - Modified ProcWrSetpoints() and ProcExActWAck() to add event insertion for
//                        setpoints download and set change
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
#include "string.h"
#include "DispComm_def.h"
#include "RealTime_def.h"
#include "Iod_def.h"
#include "Meter_def.h"
#include "Demand_def.h"
#include "Events_def.h"
#include "Setpnt_def.h"
#include "FRAM_Flash_def.h"             // Must be preceded by Events_def.h and Setpnt_def.h!
#include "Test_def.h"
#include "Prot_def.h"
#include "Diag_def.h"
#include "main_def.h"
#include "Intr_def.h"
#include "Flags_def.h"
#include "Modbus_def.h"


//
//      Local Definitions used in this module...
//
#define RTDBUF_INTERVAL_TIME    25      // Send real time data values every 250msec

// For now, set turnaround time to 40msec (30msec min).  Note, if we need to tighten the resolution of this
//   timer to speed communications, we can replace it with a timer that uses the real-time clock and use
//   Get_InternalTime() to compute the elapsed time.  This will take more computing time and memory
#define TURNAROUND_TIME         4       // *** DAH  THIS MAY BE TOO SHORT FOR THE DISPLAY PROCESSOR - NEED TO TEST THE MAX RESPONSE TIME FOR THE DISPLAY PROCESSOR!!
#define RESPONSE_TIME           6       // *** DAH  THIS MAY BE TOO SHORT FOR THE DISPLAY PROCESSOR - NEED TO TEST THE MAX RESPONSE TIME FOR THE DISPLAY PROCESSOR!!

//
//
//------------------------------------------------------------------------------------------------------------
//                   Declarations
//------------------------------------------------------------------------------------------------------------
//
//      Global Declarations from external files...
//
#include "Meter_ext.h"
#include "Test_ext.h"
#include "Iod_ext.h"
#include "Prot_ext.h"
#include "Demand_ext.h"
#include "Events_ext.h"
#include "Setpnt_ext.h"
#include "Intr_ext.h"
#include "Diag_ext.h"
#include "RealTime_ext.h"
#include "Ovrcom_ext.h"
#include "Modbus_ext.h"


//      Global (Visible) Function Prototypes (These functions are called by other modules)
//
void DispComm_VarInit(void);
void DispComm_Tx(void);
void DispComm_Rx(void);
void AssembleAck1(uint8_t ack_val);
void DispComm61850_Tx(void);
void DispComm61850_Rx(void);


//      Local Function Prototypes (These functions are called only within this module)
//
void AssembleTxPkt(uint8_t *SrcPtr, uint16_t SrcLen, struct DISPCOMMVARS *port, uint8_t LastSet);
uint8_t AssembleRxPkt(DMA_Stream_TypeDef *DMA_Stream, struct DISPCOMMVARS *port);
void ProcReadReqImm();
void ProcReadReqDel();
void AssembleAck(void);
void BuildRTDBufByBufnum(uint8_t bufnum, uint8_t cmnd, uint8_t addr);
void BuildRTDBufByTSlice(uint8_t timeslice, uint8_t cmnd, uint8_t addr);
void AssembleSetpBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr, uint16_t bufinfo);
void AssembleEvntBuffer(uint16_t msglen, uint16_t bufid, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr);
void AssembleFactoryBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr);
void ProcWrSetpoints(uint16_t bufid);
void ProcWrFactoryConfig(uint16_t bufid);
void ProcExActWAck(uint16_t actionid, uint8_t actiontype);

void Assemble61850XCBR_CB_Status_Ctrl_Msg();
void Assemble61850ValsMsg();
void Assemble61850AckMsg(uint8_t seqnum);

void AssembleTxPkt1(uint8_t *SrcPtr, uint16_t SrcLen, struct DPCOMMTXVARS *port, uint8_t LastSet);
uint8_t AssembleRxPkt1(DMA_Stream_TypeDef *DMA_Stream, struct DPCOMMRXVARS *port);

void AssembleSummaryEvntTxBuffer(uint16_t msglen, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr);
void AssembleSnapshotSummaryTxBuffer(uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr, uint16_t bid);
void AssembleSnapshotTxBuffer(uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr, uint16_t bid);
void AssembleProfileEvntTxBuffer(uint8_t bufid, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr);
void AssembleDistCapTxBuffer(uint8_t msglen, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr);
void AssembleWFCapEIDsTxBuffer(uint16_t bid, uint8_t cmnd, uint8_t addr);
void AssembleExActBuffer(uint8_t type, uint16_t id, uint8_t addr);
void AssembleDiagBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr, uint16_t bufinfo);


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Global (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used by other modules...
//
struct DISPCOMMVARS DPComm @".sram2";
struct DISPCOMM61850VARS DPComm61850 @".sram2";
struct INTERNAL_TIME IntSyncTime;

uint8_t DispProc_FW_Rev;
uint8_t DispProc_FW_Ver;
uint16_t DispProc_FW_Build;

#ifdef ENABLE_GOOSE_COMM_AUTOSEND
extern uint8_t gtest;
#endif

struct TESTINJ_VARS TestInjVars;
union PASSWORD_DEF Password;

uint8_t Reset_to_PLL_Count;
uint32_t maxlooptime;



//------------------------------------------------------------------------------------------------------------
//                   Global Constants
//------------------------------------------------------------------------------------------------------------
//
const uint16_t CRC_TABLE1[256] =
{  0x0000, 0xC1C0, 0x81C1, 0x4001, 0x01C3, 0xC003, 0x8002, 0x41C2,
   0x01C6, 0xC006, 0x8007, 0x41C7, 0x0005, 0xC1C5, 0x81C4, 0x4004,
   0x01CC, 0xC00C, 0x800D, 0x41CD, 0x000F, 0xC1CF, 0x81CE, 0x400E,
   0x000A, 0xC1CA, 0x81CB, 0x400B, 0x01C9, 0xC009, 0x8008, 0x41C8,
   0x01D8, 0xC018, 0x8019, 0x41D9, 0x001B, 0xC1DB, 0x81DA, 0x401A,
   0x001E, 0xC1DE, 0x81DF, 0x401F, 0x01DD, 0xC01D, 0x801C, 0x41DC,
   0x0014, 0xC1D4, 0x81D5, 0x4015, 0x01D7, 0xC017, 0x8016, 0x41D6,
   0x01D2, 0xC012, 0x8013, 0x41D3, 0x0011, 0xC1D1, 0x81D0, 0x4010,
   0x01F0, 0xC030, 0x8031, 0x41F1, 0x0033, 0xC1F3, 0x81F2, 0x4032,
   0x0036, 0xC1F6, 0x81F7, 0x4037, 0x01F5, 0xC035, 0x8034, 0x41F4,
   0x003C, 0xC1FC, 0x81FD, 0x403D, 0x01FF, 0xC03F, 0x803E, 0x41FE,
   0x01FA, 0xC03A, 0x803B, 0x41FB, 0x0039, 0xC1F9, 0x81F8, 0x4038,
   0x0028, 0xC1E8, 0x81E9, 0x4029, 0x01EB, 0xC02B, 0x802A, 0x41EA,
   0x01EE, 0xC02E, 0x802F, 0x41EF, 0x002D, 0xC1ED, 0x81EC, 0x402C,
   0x01E4, 0xC024, 0x8025, 0x41E5, 0x0027, 0xC1E7, 0x81E6, 0x4026,
   0x0022, 0xC1E2, 0x81E3, 0x4023, 0x01E1, 0xC021, 0x8020, 0x41E0,
   0x01A0, 0xC060, 0x8061, 0x41A1, 0x0063, 0xC1A3, 0x81A2, 0x4062,
   0x0066, 0xC1A6, 0x81A7, 0x4067, 0x01A5, 0xC065, 0x8064, 0x41A4,
   0x006C, 0xC1AC, 0x81AD, 0x406D, 0x01AF, 0xC06F, 0x806E, 0x41AE,
   0x01AA, 0xC06A, 0x806B, 0x41AB, 0x0069, 0xC1A9, 0x81A8, 0x4068,
   0x0078, 0xC1B8, 0x81B9, 0x4079, 0x01BB, 0xC07B, 0x807A, 0x41BA,
   0x01BE, 0xC07E, 0x807F, 0x41BF, 0x007D, 0xC1BD, 0x81BC, 0x407C,
   0x01B4, 0xC074, 0x8075, 0x41B5, 0x0077, 0xC1B7, 0x81B6, 0x4076,
   0x0072, 0xC1B2, 0x81B3, 0x4073, 0x01B1, 0xC071, 0x8070, 0x41B0,
   0x0050, 0xC190, 0x8191, 0x4051, 0x0193, 0xC053, 0x8052, 0x4192,
   0x0196, 0xC056, 0x8057, 0x4197, 0x0055, 0xC195, 0x8194, 0x4054,
   0x019C, 0xC05C, 0x805D, 0x419D, 0x005F, 0xC19F, 0x819E, 0x405E,
   0x005A, 0xC19A, 0x819B, 0x405B, 0x0199, 0xC059, 0x8058, 0x4198,
   0x0188, 0xC048, 0x8049, 0x4189, 0x004B, 0xC18B, 0x818A, 0x404A,
   0x004E, 0xC18E, 0x818F, 0x404F, 0x018D, 0xC04D, 0x804C, 0x418C,
   0x0044, 0xC184, 0x8185, 0x4045, 0x0187, 0xC047, 0x8046, 0x4186,
   0x0182, 0xC042, 0x8043, 0x4183, 0x0041, 0xC181, 0x8180, 0x4040
};


//
//------------------------------------------------------------------------------------------------------------
//                   Storage Allocation - Local (Static) Variables
//------------------------------------------------------------------------------------------------------------
//
//       These variables are used only in this module...
//
uint8_t xmitwait;
uint8_t DPTxReqFlags;
uint8_t DP_Tmr4BufSel, DP_Tmr5BufSel;
struct INTERNAL_TIME DP_OutSyncTime;

uint8_t                     StartUP_Status_Send;
struct SUB_CBSTATUSCTRL     Sub_GoCB_Status_Ctrl_Pkt[(MAX_GOOSE_SUB - 1)];
struct PUB_GOOSE_CBS        PXR35_CB_Publish_Data;

// *** DAH  TEST VARIABLES TO MONITOR THE GOOSE TRANSMISSIONS/RECEPTIONS FOR TEST PURPOSES ONLY
//          EVENTUALLY THESE SHOULD BE DELETED
//          NOTE - CAN ONLY READ THESE USING THE EMULATOR
//      ddcnttripack - number of acks to GOOSE Trip messages (should match number sent)
//      ddcntzsiack - number of acks to GOOSE ZI messages (should match number sent)
//      ddcnttriptx - number of GOOSE Trip messages transmitted
//      ddcntzsitx - number of GOOSE ZSI messages transmitted
//      ddintcnt - number of calls to the transmit and rx subroutines.  if no retries, this equals the
//                 number of messages requested
//      ddcntsoe[] - sequence of events
//                          0 - GOOSE Trip transmitted
//                          1 - Valid GOOSE Trip ACK received
//                          2 - GOOSE ZSI transmitted
//                          3 - Valid GOOSE ZSI ACK received
uint8_t ddcnttripack, ddcntzsiack, ddcnttriptx, ddcntzsitx;
uint16_t ddcntseqnum;
uint8_t ddcntsoe[512];
uint16_t ddintcnt;

extern uint8_t MB_Addr;                     // *** DAH  ADDED FOR TEST
extern uint8_t MB_Par;
extern uint8_t MB_Stop;
extern uint16_t MB_Baud;

uint32_t ResetMinMaxFlags;


extern uint8_t displayOFFTIM;
//
//------------------------------------------------------------------------------------------------------------
//                   Local Constants used in this module
//------------------------------------------------------------------------------------------------------------
//
//
// Real Time Data Buffer 0 Address Table
void * const DPCOMM_RTD_ADDR_BUF0[] =
{
  &StatusCode,
  &Cur200msFltr.Ia,      &Cur200msFltr.Ib,      &Cur200msFltr.Ic,       &Cur200msFltr.In,
  &Cur200msFltrIg,       &Cur200msIavg,
  &VolAFE200msFltr.Vab,  &VolAFE200msFltr.Vbc,  &VolAFE200msFltr.Vca,   &VolAFE200msFltrVllavg,
  &VolAFE200msFltr.Van,  &VolAFE200msFltr.Vbn,  &VolAFE200msFltr.Vcn,   &VolAFE200msFltrVlnavg,
  &VolADC200ms.Vab,      &VolADC200ms.Vbc,      &VolADC200ms.Vca,       &VolADC200msVllavg,
  &VolADC200ms.Van,      &VolADC200ms.Vbn,      &VolADC200ms.Vcn,       &VolADC200msVlnavg,
  &Pwr200msec.Ptot,      &Pwr200msec.Rtot,      &Pwr200msecApp.Apptot,
  &Pwr200msec.Pa,        &Pwr200msec.Pb,        &Pwr200msec.Pc,
  &Pwr200msec.RPa,       &Pwr200msec.RPb,       &Pwr200msec.RPc,
  &Pwr200msecApp.AppPa,  &Pwr200msecApp.AppPb,  &Pwr200msecApp.AppPc,
  &CurUnbalTot,          &VolUnbalTot,
  &CurUnbal.Ia,          &CurUnbal.Ib,          &CurUnbal.Ic,           &CurUnbal.In,
  &CurUnbalMax,
  &VolUnbal.Van,         &VolUnbal.Vbn,         &VolUnbal.Vcn,          &VolUnbalMaxLN,
  &VolUnbal.Vab,         &VolUnbal.Vbc,         &VolUnbal.Vca,          &VolUnbalMaxLL,
  &CF.Ia,                &CF.Ib,                &CF.Ic,                 &CF.In,
  &LD_Bucket,
  &FreqLoad.FreqVal,     &FreqLine.FreqVal,
  &LD_TimeToTrip,
  &TU_BinStatus,
  &CurOneCyc.Ia,         &CurOneCyc.Ib,         &CurOneCyc.Ic,          &CurOneCyc.In,
  &CurOneCycIg
};

void * const DPCOMM_RTD_ADDR_BUF0_TESTINJ[] =
{
  &StatusCode,
  &CurOneCyc.Ia,         &CurOneCyc.Ib,         &CurOneCyc.Ic,          &CurOneCyc.In,
  &CurOneCycIg,          &CurOneCycIavg,
  &VolAFE200msFltr.Vab,  &VolAFE200msFltr.Vbc,  &VolAFE200msFltr.Vca,   &VolAFE200msFltrVllavg,
  &VolAFE200msFltr.Van,  &VolAFE200msFltr.Vbn,  &VolAFE200msFltr.Vcn,   &VolAFE200msFltrVlnavg,
  &VolADC200ms.Vab,      &VolADC200ms.Vbc,      &VolADC200ms.Vca,       &VolADC200msVllavg,
  &VolADC200ms.Van,      &VolADC200ms.Vbn,      &VolADC200ms.Vcn,       &VolADC200msVlnavg,
  &Pwr200msec.Ptot,      &Pwr200msec.Rtot,      &Pwr200msecApp.Apptot,
  &Pwr200msec.Pa,        &Pwr200msec.Pb,        &Pwr200msec.Pc,
  &Pwr200msec.RPa,       &Pwr200msec.RPb,       &Pwr200msec.RPc,
  &Pwr200msecApp.AppPa,  &Pwr200msecApp.AppPb,  &Pwr200msecApp.AppPc,
  &CurUnbalTot,          &VolUnbalTot,
  &CurUnbal.Ia,          &CurUnbal.Ib,          &CurUnbal.Ic,           &CurUnbal.In,
  &CurUnbalMax,
  &VolUnbal.Van,         &VolUnbal.Vbn,         &VolUnbal.Vcn,          &VolUnbalMaxLN,
  &VolUnbal.Vab,         &VolUnbal.Vbc,         &VolUnbal.Vca,          &VolUnbalMaxLL,
  &CF.Ia,                &CF.Ib,                &CF.Ic,                 &CF.In,
  &LD_Bucket,
  &FreqLoad.FreqVal,     &FreqLine.FreqVal,
  &LD_TimeToTrip,
  &TU_BinStatus,
  &CurOneCyc.Ia,         &CurOneCyc.Ib,         &CurOneCyc.Ic,          &CurOneCyc.In,
  &CurOneCycIg
};


// Real Time Data Buffer 1 Address Table A - these are 8 bytes long
void * const DPCOMM_RTD_ADDR_BUF1_A[] =
{
  &EngyDmnd[1].TotFwdWHr,     &EngyDmnd[1].TotRevWHr,    &TotWHr,            &NetWHr,
  &EngyDmnd[1].TotLeadVarHr,  &EngyDmnd[1].TotLagVarHr,  &NetVarHr,          &TotVarHr,
  &EngyDmnd[1].TotVAHr
};

// Real Time Data Buffer 1 Address Table B
void * const DPCOMM_RTD_ADDR_BUF1_B[] =
{
  &PF.Disp[3],                &PF.App[3],
  &PF.Disp[0],                &PF.Disp[1],               &PF.Disp[2],
  &PF.App[0],                 &PF.App[1],                &PF.App[2],
  &THD[0],                    &THD[1],                   &THD[2],            &THD[3],
  &THD[7],                    &THD[8],                   &THD[9],
  &THD[4],                    &THD[5],                   &THD[6],
  &KF_Val.Ia,                 &KF_Val.Ib,                &KF_Val.Ic,
  &SeqComp.V_PosMag,          &SeqComp.V_PosPh,          &SeqComp.V_NegMag,  &SeqComp.V_NegPh,
  &SeqComp.V_ZeroMag,         &SeqComp.V_ZeroPh,
  &SeqComp.I_PosMag,          &SeqComp.I_NegMag,         &SeqComp.I_ZeroMag, &SeqComp.I_PosPh,
  &SeqComp.I_NegPh,           &SeqComp.I_ZeroPh,
  &PhAngles1[0],              &PhAngles1[1],             &PhAngles1[2],      &PhAngles1[3],
  &PhAngles1[4],              &PhAngles1[5],             &PhAngles1[6],      &PhAngles1[7],
  &PhAngles1[8],
  &PhAngles2[0],              &PhAngles2[1],             &PhAngles2[2],      &PhAngles2[3],
  &PhAngles2[4],              &PhAngles2[5]
};

// Real Time Data Buffer 2 Address Table
void * const DPCOMM_RTD_ADDR_BUF2[] =
{
  &EngyDmnd[1].DmndIa,       &EngyDmnd[1].DmndIb,           &EngyDmnd[1].DmndIc,        &EngyDmnd[1].DmndIn,
  &EngyDmnd[1].DmndTotW,     &EngyDmnd[1].DmndTotVar,       &EngyDmnd[1].DmndTotVA,
  &IDmnd.IaMin,              &IDmnd.IaMinTS.Time_secs,      &IDmnd.IaMinTS.Time_nsec,
  &IDmnd.IaMax,              &IDmnd.IaMaxTS.Time_secs,      &IDmnd.IaMaxTS.Time_nsec,
  &IDmnd.IbMin,              &IDmnd.IbMinTS.Time_secs,      &IDmnd.IbMinTS.Time_nsec,
  &IDmnd.IbMax,              &IDmnd.IbMaxTS.Time_secs,      &IDmnd.IbMaxTS.Time_nsec,
  &IDmnd.IcMin,              &IDmnd.IcMinTS.Time_secs,      &IDmnd.IcMinTS.Time_nsec,
  &IDmnd.IcMax,              &IDmnd.IcMaxTS.Time_secs,      &IDmnd.IcMaxTS.Time_nsec,
  &IDmnd.InMin,              &IDmnd.InMinTS.Time_secs,      &IDmnd.InMinTS.Time_nsec,
  &IDmnd.InMax,              &IDmnd.InMaxTS.Time_secs,      &IDmnd.InMaxTS.Time_nsec,
  &PDmnd.TotWMax,            &PDmnd.TotWMaxTS.Time_secs,    &PDmnd.TotWMaxTS.Time_nsec,
  &PDmnd.TotVarMax,          &PDmnd.TotVarMaxTS.Time_secs,  &PDmnd.TotVarMaxTS.Time_nsec,
  &PDmnd.TotVAMax,           &PDmnd.TotVAMaxTS.Time_secs,   &PDmnd.TotVAMaxTS.Time_nsec,
  &Dmnd_Type_Window,         &IDmnd.ResetTS.Time_secs,      &IDmnd.ResetTS.Time_nsec,
  &PDmnd.ResetTS.Time_secs,  &PDmnd.ResetTS.Time_nsec
};

// Real Time Data Buffer 3 Address Table
void * const DPCOMM_RTD_ADDR_BUF3[] =
{
  &Res5min_Avg.Ia,             &Res5min_Avg.Ib,            &Res5min_Avg.Ic,           &Res5min_Avg.Iavg,
  &Res5min_Avg.Pa,             &Res5min_Avg.Pb,            &Res5min_Avg.Pc,           &Res5min_Avg.Ptot,        
  &Res5min_Avg.AppPa,          &Res5min_Avg.AppPb,         &Res5min_Avg.AppPc,        &Res5min_Avg.AppPtot,
  &Res5min_MinMax.Iamax,       &Res5min_MinMax.Iamin,      &Res5min_MinMax.Ibmax,     &Res5min_MinMax.Ibmin,
  &Res5min_MinMax.Icmax,       &Res5min_MinMax.Icmin,      &Res5min_MinMax.Iavgmax,
  &Res5min_MinMax.Iavgmin,
  &Res5min_MinMax.Pamax,       &Res5min_MinMax.Pamin,      &Res5min_MinMax.Pbmax,     &Res5min_MinMax.Pbmin,
  &Res5min_MinMax.Pcmax,       &Res5min_MinMax.Pcmin,      &Res5min_MinMax.Ptotmax,
  &Res5min_MinMax.Ptotmin,
  &Res5min_MinMax.AppPamax,    &Res5min_MinMax.AppPamin,   &Res5min_MinMax.AppPbmax,
  &Res5min_MinMax.AppPbmin,    &Res5min_MinMax.AppPcmax,   &Res5min_MinMax.AppPcmin, 
  &Res5min_MinMax.AppPtotmax,  &Res5min_MinMax.AppPtotmin, &Vbattery,                 &THSensor.Temperature,
  &THSensor.Humidity
};

// Real Time Data Buffer 4 Address Table
void * const DPCOMM_RTD_ADDR_BUF4[] =
{
  &CurOneCyc_min.Ia,  &CurOneCyc_min.IaTS.Time_secs,  &CurOneCyc_min.IaTS.Time_nsec,
  &CurOneCyc_max.Ia,  &CurOneCyc_max.IaTS.Time_secs,  &CurOneCyc_max.IaTS.Time_nsec,
  &CurOneCyc_min.Ib,  &CurOneCyc_min.IbTS.Time_secs,  &CurOneCyc_min.IbTS.Time_nsec,
  &CurOneCyc_max.Ib,  &CurOneCyc_max.IbTS.Time_secs,  &CurOneCyc_max.IbTS.Time_nsec,
  &CurOneCyc_min.Ic,  &CurOneCyc_min.IcTS.Time_secs,  &CurOneCyc_min.IcTS.Time_nsec,
  &CurOneCyc_max.Ic,  &CurOneCyc_max.IcTS.Time_secs,  &CurOneCyc_max.IcTS.Time_nsec,
  &CurOneCyc_min.Ig,  &CurOneCyc_min.IgTS.Time_secs,  &CurOneCyc_min.IgTS.Time_nsec,
  &CurOneCyc_max.Ig,  &CurOneCyc_max.IgTS.Time_secs,  &CurOneCyc_max.IgTS.Time_nsec,
  &CurOneCyc_min.In,  &CurOneCyc_min.InTS.Time_secs,  &CurOneCyc_min.InTS.Time_nsec,
  &CurOneCyc_max.In,  &CurOneCyc_max.InTS.Time_secs,  &CurOneCyc_max.InTS.Time_nsec
};

// Real Time Data Buffer 5 Address Table
void * const DPCOMM_RTD_ADDR_BUF5[] =
{
  &VolAFEOneCyc_min.Vab,  &VolAFEOneCyc_min.VabTS.Time_secs,  &VolAFEOneCyc_min.VabTS.Time_nsec,
  &VolAFEOneCyc_max.Vab,  &VolAFEOneCyc_max.VabTS.Time_secs,  &VolAFEOneCyc_max.VabTS.Time_nsec,
  &VolAFEOneCyc_min.Vbc,  &VolAFEOneCyc_min.VbcTS.Time_secs,  &VolAFEOneCyc_min.VbcTS.Time_nsec,
  &VolAFEOneCyc_max.Vbc,  &VolAFEOneCyc_max.VbcTS.Time_secs,  &VolAFEOneCyc_max.VbcTS.Time_nsec,
  &VolAFEOneCyc_min.Vca,  &VolAFEOneCyc_min.VcaTS.Time_secs,  &VolAFEOneCyc_min.VcaTS.Time_nsec,
  &VolAFEOneCyc_max.Vca,  &VolAFEOneCyc_max.VcaTS.Time_secs,  &VolAFEOneCyc_max.VcaTS.Time_nsec,
  &VolAFEOneCyc_min.Van,  &VolAFEOneCyc_min.VanTS.Time_secs,  &VolAFEOneCyc_min.VanTS.Time_nsec,
  &VolAFEOneCyc_max.Van,  &VolAFEOneCyc_max.VanTS.Time_secs,  &VolAFEOneCyc_max.VanTS.Time_nsec,
  &VolAFEOneCyc_min.Vbn,  &VolAFEOneCyc_min.VbnTS.Time_secs,  &VolAFEOneCyc_min.VbnTS.Time_nsec,
  &VolAFEOneCyc_max.Vbn,  &VolAFEOneCyc_max.VbnTS.Time_secs,  &VolAFEOneCyc_max.VbnTS.Time_nsec,
  &VolAFEOneCyc_min.Vcn,  &VolAFEOneCyc_min.VcnTS.Time_secs,  &VolAFEOneCyc_min.VcnTS.Time_nsec,
  &VolAFEOneCyc_max.Vcn,  &VolAFEOneCyc_max.VcnTS.Time_secs,  &VolAFEOneCyc_max.VcnTS.Time_nsec,
  &VolADCOneCyc_min.Vab,  &VolADCOneCyc_min.VabTS.Time_secs,  &VolADCOneCyc_min.VabTS.Time_nsec,
  &VolADCOneCyc_max.Vab,  &VolADCOneCyc_max.VabTS.Time_secs,  &VolADCOneCyc_max.VabTS.Time_nsec,
  &VolADCOneCyc_min.Vbc,  &VolADCOneCyc_min.VbcTS.Time_secs,  &VolADCOneCyc_min.VbcTS.Time_nsec,
  &VolADCOneCyc_max.Vbc,  &VolADCOneCyc_max.VbcTS.Time_secs,  &VolADCOneCyc_max.VbcTS.Time_nsec,
  &VolADCOneCyc_min.Vca,  &VolADCOneCyc_min.VcaTS.Time_secs,  &VolADCOneCyc_min.VcaTS.Time_nsec,
  &VolADCOneCyc_max.Vca,  &VolADCOneCyc_max.VcaTS.Time_secs,  &VolADCOneCyc_max.VcaTS.Time_nsec,
  &VolADCOneCyc_min.Van,  &VolADCOneCyc_min.VanTS.Time_secs,  &VolADCOneCyc_min.VanTS.Time_nsec,
  &VolADCOneCyc_max.Van,  &VolADCOneCyc_max.VanTS.Time_secs,  &VolADCOneCyc_max.VanTS.Time_nsec,
  &VolADCOneCyc_min.Vbn,  &VolADCOneCyc_min.VbnTS.Time_secs,  &VolADCOneCyc_min.VbnTS.Time_nsec,
  &VolADCOneCyc_max.Vbn,  &VolADCOneCyc_max.VbnTS.Time_secs,  &VolADCOneCyc_max.VbnTS.Time_nsec,
  &VolADCOneCyc_min.Vcn,  &VolADCOneCyc_min.VcnTS.Time_secs,  &VolADCOneCyc_min.VcnTS.Time_nsec,
  &VolADCOneCyc_max.Vcn,  &VolADCOneCyc_max.VcnTS.Time_secs,  &VolADCOneCyc_max.VcnTS.Time_nsec 
};

// Real Time Data Buffer 6 Address Table
void * const DPCOMM_RTD_ADDR_BUF6[] =
{
  &Pwr200msecMin.Pa,          &Pwr200msecMin.PaTS.Time_secs,           &Pwr200msecMin.PaTS.Time_nsec,
  &Pwr200msecMax.Pa,          &Pwr200msecMax.PaTS.Time_secs,           &Pwr200msecMax.PaTS.Time_nsec,
  &Pwr200msecMin.Pb,          &Pwr200msecMin.PbTS.Time_secs,           &Pwr200msecMin.PbTS.Time_nsec,
  &Pwr200msecMax.Pb,          &Pwr200msecMax.PbTS.Time_secs,           &Pwr200msecMax.PbTS.Time_nsec,
  &Pwr200msecMin.Pc,          &Pwr200msecMin.PcTS.Time_secs,           &Pwr200msecMin.PcTS.Time_nsec,
  &Pwr200msecMax.Pc,          &Pwr200msecMax.PcTS.Time_secs,           &Pwr200msecMax.PcTS.Time_nsec,
  &Pwr200msecMin.Ptot,        &Pwr200msecMin.PtotTS.Time_secs,         &Pwr200msecMin.PtotTS.Time_nsec,
  &Pwr200msecMax.Ptot,        &Pwr200msecMax.PtotTS.Time_secs,         &Pwr200msecMax.PtotTS.Time_nsec,
  &Pwr200msecMin.RPa,         &Pwr200msecMin.RPaTS.Time_secs,          &Pwr200msecMin.RPaTS.Time_nsec,
  &Pwr200msecMax.RPa,         &Pwr200msecMax.RPaTS.Time_secs,          &Pwr200msecMax.RPaTS.Time_nsec,
  &Pwr200msecMin.RPb,         &Pwr200msecMin.RPbTS.Time_secs,          &Pwr200msecMin.RPbTS.Time_nsec,
  &Pwr200msecMax.RPb,         &Pwr200msecMax.RPbTS.Time_secs,          &Pwr200msecMax.RPbTS.Time_nsec,
  &Pwr200msecMin.RPc,         &Pwr200msecMin.RPcTS.Time_secs,          &Pwr200msecMin.RPcTS.Time_nsec,
  &Pwr200msecMax.RPc,         &Pwr200msecMax.RPcTS.Time_secs,          &Pwr200msecMax.RPcTS.Time_nsec,
  &Pwr200msecMin.Rtot,        &Pwr200msecMin.RtotTS.Time_secs,         &Pwr200msecMin.RtotTS.Time_nsec,
  &Pwr200msecMax.Rtot,        &Pwr200msecMax.RtotTS.Time_secs,         &Pwr200msecMax.RtotTS.Time_nsec,
  &Pwr200msecAppMin.AppPa,    &Pwr200msecAppMin.AppPaTS.Time_secs,     &Pwr200msecAppMin.AppPaTS.Time_nsec,
  &Pwr200msecAppMax.AppPa,    &Pwr200msecAppMax.AppPaTS.Time_secs,     &Pwr200msecAppMax.AppPaTS.Time_nsec,
  &Pwr200msecAppMin.AppPb,    &Pwr200msecAppMin.AppPbTS.Time_secs,     &Pwr200msecAppMin.AppPbTS.Time_nsec,
  &Pwr200msecAppMax.AppPb,    &Pwr200msecAppMax.AppPbTS.Time_secs,     &Pwr200msecAppMax.AppPbTS.Time_nsec,
  &Pwr200msecAppMin.AppPc,    &Pwr200msecAppMin.AppPcTS.Time_secs,     &Pwr200msecAppMin.AppPcTS.Time_nsec,
  &Pwr200msecAppMax.AppPc,    &Pwr200msecAppMax.AppPcTS.Time_secs,     &Pwr200msecAppMax.AppPcTS.Time_nsec,
  &Pwr200msecAppMin.Apptot,   &Pwr200msecAppMin.ApptotTS.Time_secs,    &Pwr200msecAppMin.ApptotTS.Time_nsec,
  &Pwr200msecAppMax.Apptot,   &Pwr200msecAppMax.ApptotTS.Time_secs,    &Pwr200msecAppMax.ApptotTS.Time_nsec
};

// Real Time Data Buffer 7 Address Table
void * const DPCOMM_RTD_ADDR_BUF7[] =
{
  &FreqLoad.MinFreqVal,   &FreqLoad.MinFreqVal_TS.Time_secs,   &FreqLoad.MinFreqVal_TS.Time_nsec,
  &FreqLoad.MaxFreqVal,   &FreqLoad.MaxFreqVal_TS.Time_secs,   &FreqLoad.MaxFreqVal_TS.Time_nsec,
  &FreqLine.MinFreqVal,   &FreqLine.MinFreqVal_TS.Time_secs,   &FreqLine.MinFreqVal_TS.Time_nsec,
  &FreqLine.MaxFreqVal,   &FreqLine.MaxFreqVal_TS.Time_secs,   &FreqLine.MaxFreqVal_TS.Time_nsec,
  &LD_BucketMax,          &LD_BucketMaxTS.Time_secs,           &LD_BucketMaxTS.Time_nsec,
  &SD_BucketMax,          &SD_BucketMaxTS.Time_secs,           &SD_BucketMaxTS.Time_nsec,
  &CurOneCyc.Ia,          &CurOneCyc.Ib,                       &CurOneCyc.Ic,                  &CurOneCyc.In
};

// Real Time Data Buffer 8 Address Table
void * const DPCOMM_RTD_ADDR_BUF8[] =
{
  &CurUnbalPh_max.Ia,      &CurUnbalPh_max.IaTS.Time_secs,      &CurUnbalPh_max.IaTS.Time_nsec, 
  &CurUnbalPh_max.Ib,      &CurUnbalPh_max.IbTS.Time_secs,      &CurUnbalPh_max.IbTS.Time_nsec, 
  &CurUnbalPh_max.Ic,      &CurUnbalPh_max.IcTS.Time_secs,      &CurUnbalPh_max.IcTS.Time_nsec, 
  &CurUnbalPh_max.In,      &CurUnbalPh_max.InTS.Time_secs,      &CurUnbalPh_max.InTS.Time_nsec, 
  &CurUnbalAllMax,         &CurUnbalAllMaxTS.Time_secs,         &CurUnbalAllMaxTS.Time_nsec,    
  &VolUnbalPh_max.Van,     &VolUnbalPh_max.VanTS.Time_secs,     &VolUnbalPh_max.VanTS.Time_nsec,
  &VolUnbalPh_max.Vbn,     &VolUnbalPh_max.VbnTS.Time_secs,     &VolUnbalPh_max.VbnTS.Time_nsec,
  &VolUnbalPh_max.Vcn,     &VolUnbalPh_max.VcnTS.Time_secs,     &VolUnbalPh_max.VcnTS.Time_nsec,
  &VolUnbalAllMaxLN,       &VolUnbalAllMaxLNTS.Time_secs,       &VolUnbalAllMaxLNTS.Time_nsec,  
  &VolUnbalPh_max.Vab,     &VolUnbalPh_max.VabTS.Time_secs,     &VolUnbalPh_max.VabTS.Time_nsec,
  &VolUnbalPh_max.Vbc,     &VolUnbalPh_max.VbcTS.Time_secs,     &VolUnbalPh_max.VbcTS.Time_nsec,
  &VolUnbalPh_max.Vca,     &VolUnbalPh_max.VcaTS.Time_secs,     &VolUnbalPh_max.VcaTS.Time_nsec,
  &VolUnbalAllMaxLL,       &VolUnbalAllMaxLLTS.Time_secs,       &VolUnbalAllMaxLLTS.Time_nsec,  
  &CurUnbalTotMax,         &CurUnbalTotMaxTS.Time_secs,         &CurUnbalTotMaxTS.Time_nsec,    
  &VolUnbalTotMax,         &VolUnbalTotMaxTS.Time_secs,         &VolUnbalTotMaxTS.Time_nsec     
};

// Real Time Data Buffer 9 Address Table
void * const DPCOMM_RTD_ADDR_BUF9[] =
{
  &THDminmax[0].THDmin,    &THDminmax[0].THDminTS.Time_secs,    &THDminmax[0].THDminTS.Time_nsec,
  &THDminmax[0].THDmax,    &THDminmax[0].THDmaxTS.Time_secs,    &THDminmax[0].THDmaxTS.Time_nsec,
  &THDminmax[1].THDmin,    &THDminmax[1].THDminTS.Time_secs,    &THDminmax[1].THDminTS.Time_nsec,
  &THDminmax[1].THDmax,    &THDminmax[1].THDmaxTS.Time_secs,    &THDminmax[1].THDmaxTS.Time_nsec,
  &THDminmax[2].THDmin,    &THDminmax[2].THDminTS.Time_secs,    &THDminmax[2].THDminTS.Time_nsec,
  &THDminmax[2].THDmax,    &THDminmax[2].THDmaxTS.Time_secs,    &THDminmax[2].THDmaxTS.Time_nsec,
  &THDminmax[3].THDmin,    &THDminmax[3].THDminTS.Time_secs,    &THDminmax[3].THDminTS.Time_nsec,
  &THDminmax[3].THDmax,    &THDminmax[3].THDmaxTS.Time_secs,    &THDminmax[3].THDmaxTS.Time_nsec,
  &THDminmax[7].THDmin,    &THDminmax[7].THDminTS.Time_secs,    &THDminmax[7].THDminTS.Time_nsec,
  &THDminmax[7].THDmax,    &THDminmax[7].THDmaxTS.Time_secs,    &THDminmax[7].THDmaxTS.Time_nsec,
  &THDminmax[8].THDmin,    &THDminmax[8].THDminTS.Time_secs,    &THDminmax[8].THDminTS.Time_nsec,
  &THDminmax[8].THDmax,    &THDminmax[8].THDmaxTS.Time_secs,    &THDminmax[8].THDmaxTS.Time_nsec,
  &THDminmax[9].THDmin,    &THDminmax[9].THDminTS.Time_secs,    &THDminmax[9].THDminTS.Time_nsec,
  &THDminmax[9].THDmax,    &THDminmax[9].THDmaxTS.Time_secs,    &THDminmax[9].THDmaxTS.Time_nsec,
  &THDminmax[4].THDmin,    &THDminmax[4].THDminTS.Time_secs,    &THDminmax[4].THDminTS.Time_nsec,
  &THDminmax[4].THDmax,    &THDminmax[4].THDmaxTS.Time_secs,    &THDminmax[4].THDmaxTS.Time_nsec,
  &THDminmax[5].THDmin,    &THDminmax[5].THDminTS.Time_secs,    &THDminmax[5].THDminTS.Time_nsec,
  &THDminmax[5].THDmax,    &THDminmax[5].THDmaxTS.Time_secs,    &THDminmax[5].THDmaxTS.Time_nsec,
  &THDminmax[6].THDmin,    &THDminmax[6].THDminTS.Time_secs,    &THDminmax[6].THDminTS.Time_nsec,
  &THDminmax[6].THDmax,    &THDminmax[6].THDmaxTS.Time_secs,    &THDminmax[6].THDmaxTS.Time_nsec 
};

// Real Time Data Buffer 10 Address Table
void * const DPCOMM_RTD_ADDR_BUF10[] =
{
  &PF.MinApp[3],       &PF.MinApp_TS[3].Time_secs,       &PF.MinApp_TS[3].Time_nsec, 
  &PF.MaxApp[3],       &PF.MaxApp_TS[3].Time_secs,       &PF.MaxApp_TS[3].Time_nsec, 
  &PF.MinDisp[3],      &PF.MinDisp_TS[3].Time_secs,      &PF.MinDisp_TS[3].Time_nsec,
  &PF.MaxDisp[3],      &PF.MaxDisp_TS[3].Time_secs,      &PF.MaxDisp_TS[3].Time_nsec,
  &PF.MinApp[0],       &PF.MinApp_TS[0].Time_secs,       &PF.MinApp_TS[0].Time_nsec, 
  &PF.MaxApp[0],       &PF.MaxApp_TS[0].Time_secs,       &PF.MaxApp_TS[0].Time_nsec, 
  &PF.MinApp[1],       &PF.MinApp_TS[1].Time_secs,       &PF.MinApp_TS[1].Time_nsec, 
  &PF.MaxApp[1],       &PF.MaxApp_TS[1].Time_secs,       &PF.MaxApp_TS[1].Time_nsec, 
  &PF.MinApp[2],       &PF.MinApp_TS[2].Time_secs,       &PF.MinApp_TS[2].Time_nsec, 
  &PF.MaxApp[2],       &PF.MaxApp_TS[2].Time_secs,       &PF.MaxApp_TS[2].Time_nsec, 
  &PF.MinDisp[0],      &PF.MinDisp_TS[0].Time_secs,      &PF.MinDisp_TS[0].Time_nsec,
  &PF.MaxDisp[0],      &PF.MaxDisp_TS[0].Time_secs,      &PF.MaxDisp_TS[0].Time_nsec,
  &PF.MinDisp[1],      &PF.MinDisp_TS[1].Time_secs,      &PF.MinDisp_TS[1].Time_nsec,
  &PF.MaxDisp[1],      &PF.MaxDisp_TS[1].Time_secs,      &PF.MaxDisp_TS[1].Time_nsec,
  &PF.MinDisp[2],      &PF.MinDisp_TS[2].Time_secs,      &PF.MinDisp_TS[2].Time_nsec,
  &PF.MaxDisp[2],      &PF.MaxDisp_TS[2].Time_secs,      &PF.MaxDisp_TS[2].Time_nsec 
};

void * const * const DPCOMM_RTD_ADDR[] =
{
  DPCOMM_RTD_ADDR_BUF0, DPCOMM_RTD_ADDR_BUF1_B, DPCOMM_RTD_ADDR_BUF2,  DPCOMM_RTD_ADDR_BUF3,
  DPCOMM_RTD_ADDR_BUF4, DPCOMM_RTD_ADDR_BUF5,   DPCOMM_RTD_ADDR_BUF6,  DPCOMM_RTD_ADDR_BUF7,
  DPCOMM_RTD_ADDR_BUF8, DPCOMM_RTD_ADDR_BUF9,   DPCOMM_RTD_ADDR_BUF10
};

const uint16_t DPCOMMM_RTD_BUFSIZE[] =
{
  sizeof(DPCOMM_RTD_ADDR_BUF0), sizeof(DPCOMM_RTD_ADDR_BUF1_B), sizeof(DPCOMM_RTD_ADDR_BUF2),
  sizeof(DPCOMM_RTD_ADDR_BUF3), sizeof(DPCOMM_RTD_ADDR_BUF4),   sizeof(DPCOMM_RTD_ADDR_BUF5),
  sizeof(DPCOMM_RTD_ADDR_BUF6), sizeof(DPCOMM_RTD_ADDR_BUF7),   sizeof(DPCOMM_RTD_ADDR_BUF8),
  sizeof(DPCOMM_RTD_ADDR_BUF9), sizeof(DPCOMM_RTD_ADDR_BUF10)
};



// Aggregated Harmonics Data Buffer Address Table
void * const DPCOMM_HARM[] =
{
  &HarmonicsAgg.Ia[0], &HarmonicsAgg.Van[0], &HarmonicsAgg.Vab[0],
  &HarmonicsCap.Ia[0], &HarmonicsCap.Van[0], &HarmonicsCap.Vab[0]
};

// Factory Buffer Info Table
//   bufID, Length, ProcCode, Address (Hex), place in category, length of category
// ProcCode generally controls how data is retrieved and processed.  Note, the buffer id is also used for
//   this purpose; some buffers require special processing in addition or instead of that dictated by
//   ProcCode.  For example Buffer 9 retrieves data from Frame FRAM, but then fills in the critical values
//   from RAM
//   ProcCode   Retrieve Data From   Write Data To        Number Copies   Default Value or 0        Comment
//      0         on-board FRAM      on-board FRAM              3                0
//      1         Frame FRAM         Frame FRAM                 3                0
//      2         RAM                Frame FRAM                 3         N/A - already in RAM
//      3         RAM                on-board FRAM              3         N/A - already in RAM
//      4       FRAM/Flash AFE Cal   FRAM/Flash AFE Cal        N/A        N/A - generated           Special
//      5       FRAM/Flash ADCH Cal  FRAM/Flash ADCH Cal       N/A        N/A - generated           Special
//      6       FRAM/Flash ADCL Cal  FRAM/Flash ADCL Cal       N/A        N/A - generated           Special
// Note, in the table below, the items with ProcCode = 4 (**) do not use the Address or place in category
//   values.  These are merely placeholders.  These items are retrieved and written with custom code
const uint16_t FacBuf_Info[46][6] =
{
  {0,    2, 2, 0x68A,  0,   12},                    // Frame Rating
  {1,    2, 2, 0x68A,  2,   12},                    // Frame
  {2,    2, 1,  0x30, 397, 403},                    // CT
  {3,   40, 1,  0x30,   4, 403},                    // Parent catalog number
  {4,   64, 1,  0x30,  44, 403},                    // Parent serial number
  {5,    5, 1,  0x30, 114, 403},                    // Parent manufacturing location
  {6,   40, 1,  0x30, 119, 403},                    // Configured catalog number
  {7,   64, 1,  0x30, 159, 403},                    // Configured serial number
  {8,    5, 1,  0x30, 229, 403},                    // Configured manufacturing location
  {9,   34, 1,  0x30, 234, 403},                    // Current configuration
  {10,  25, 1,  0x30, 268, 403},                    // Pole phase order
  {11,   6, 1,  0x30, 108, 403},                    // Parent manufacturing date
  {12,   6, 1,  0x30, 223, 403},                    // Breaker config manufacturing date
  {13,   4, 1,  0x30, 293, 403},                    // Number of customer in reprogramming
  {14, 100, 1,  0x30, 297, 403},                    // Security data (not used)
  {15,  16, 0,     0,   0, 100},                    // Rogowski offset (invalid)
  {16,   8, 1,     0,   0,  12},                    // Rogowski gain
  {17,  12, 4,     0,  60, 100},                    // Not used
  {18,  12, 4,     0,  20, 100},                    // ETU AFE voltage gain                 **
  {19,  12, 4,     0,  83, 100},                    // ETU AFE phase shift                  **
  {20,  24, 3,     0,  46, 100},                    // ETU ADC voltage cal constants        **
  {21,  16, 4,     0,   0, 100},                    // SG cal constants                     **
  {22,   2, 0,     0,   0,   0},                    // ETU style                            **
  {23,   4, 0, FAC_BUF_START - 0x10000,  0, 22},    // ETU serial number
  {24,   2, 0, FAC_BUF_START - 0x10000,  4, 22},    // ETU HW version
  {25,   4, 1, 0x660,   2,  10},                    // Frame module serial number
  {26,   2, 1, 0x660,   0,  10},                    // Frame module hardware version
  {27,  68, 1, 0x6BA,   0,  72},                    // Breaker health config   not in triplicate
  {28,   6, 1, 0x76A,   0,  10},                    // VDB1 configuration
  {29,  24, 1, 0x812,   0,  28},                    // VDB1 cal factors
  {30,  12, 0, FAC_BUF_START - 0x10000,  6, 22},    // Manufacuring Location and Date
  {31,  52, 1, 0x4F8,   0,  56},                    // external diagnostics
  {32,  52, 1, 0x5AC,   0,  56},                    // Internal diagnostics
  {33,  42, 1, 0x706,   0,  46},                    // Customer breaker health data  not in triplicate
  {34,  42, 1, 0x738,   0,  46},                    // Internal breaker health data  not in triplicate
  {35,  40, 4,     0,   0, 100},                    // AFE Cal Constants                    **
  {36,  40, 6,     0,   0,  72},                    // ADC Low-Gain Cal Constants           **
  {37,  40, 5,     0,   0,  72},                    // ADC High-Gain Cal Constants          **
  {38,   6, 1, 0x794,   0,  10},                    // VDB2 Config
  {39,  12, 1, 0x872,   0,  16},                    // VDB2 Cal Factors
  {40,  24, 4,     0,   0, 150},                    // Neutral CT AFE and ADC Cal Constants (invalid)
  {41,   2, 0,     0,  0,    0},                    // ETU style2                           **
  {42,   0, 0,     0,   0,   0},                    // Placeholder for NV buffer            **
  {43,   2, 1, 0x68A,   4, 403},                    // Standard
  {44,   2, 1, 0x68A,   6, 403},                    // Max Instantaneous Trip Setting
  {45,   2, 3,     0,   0,   0}                     // Buf I/O                              **
};
#define LAST_FAC_BUFID  45
#define MAX_FAC_BUFID   100                         // Buffers 99 and 100 are used for firmware version info

// 12345678; XX.XX.XXXX format to be read in
//   ver - uint8, rev - uint8, build uint16 (per PXR25 V3.0, see struct FIRM_VER_DEF in System_def.h
//   For above example ver = 12, rev = 34, build = 5678
uint32_t firmware_version;  

uint8_t RelayFlagStp;

//------------------------------------------------------------------------------------------------------------
//                   Message Protocol
//------------------------------------------------------------------------------------------------------------
//
// This protocol can be considered to handle the Transport (Layer 4)and Application (Layer 7) layers in the
// Open Systems Interconnection (OSI) Communications Model.
//
// The normal protection processor - display processor comms link (described here) uses UART2.
//
// The protocol is loosely based on the original CAM-Comm (9-bit) Protocol.  The major change is that this
// protocol is an 8-bit protocol that utilizes special Start-of-Message and End-of-Message characters and
// the COBS (Consistent Overhead Byte Stuffing) method for handling the special characters.
//
// The upper-level protocol (Application Layer) is described in the Advanced Communications Adapter
// Specification.  The lower-level portion of the protocol, including the COBS technique, is briefly
// described below:
//   - Characters consist of 10 bits: start (low), 8 data bits (LSB first), stop (high)
//   - Messages consist of the following characters:
//          - Start of packet (x00)
//          - One or more segment codes
//          - Command
//          - Source address
//          - Destination address
//          - Sequence number
//          - Buffer type
//          - Buffer ID
//          - Buffer length
//          - Buffer information
//          - CRC - Least significant byte first
//          - End of packet (x01)
//   - 16-bit and 32-bit values, including the 16-bit CRC, are transmitted with the least significant byte
//     first and the most significant byte last
//   - Messages employ a modified COBS technique for encoding the message contents (that is, the characters
//     from the command through the CRC).  The approach is similar to the COBs technique, except that two
//     framing characters are defined instead of one.  The technique operates as follows:
//          - Messages begin with a start-of-message character (0x00)
//          - The next character, provided it is not a packet delimiter, is the segment code.  Bits 7..1
//            contain the offset of the next data character that matches either packet delimiter (0x00 or
//            0x01), and bit 0 contains the data value.  Valid values for bits 7..1 are 1 - 127.  If b7..1
//            is 127, the next data character that matches the packet delimiter is past the 126th character.
//            Bit 0 is not applicable in this case.  The 127th character is then another segment code.
//     Therefore, there is a maximum of one additional character for every 126 characters in the message.
//
//  Communications between the Protection Processor (this firmware) and the Display Processor are
//  full-duplex.  Sequence numbers and addressing are used to ensure the message responses are aligned to
//  the proper command.  However, once a message is transmitted, the sending device must wait the Turnaround
//  Time for the device to respond, unless a response to the command is received from the receiving device.
//  The Display Processor sends the following commands to the Protection Processor:
//      Command = 2: Read Request Immediate
//      Command = 3: Read Request Delayed
//      Command = 6: Write Buffer With Acknowledge
//      Command = 9: Execute Action With Acknowledge
//      Command = 10: Execute Action With Check
//      Command = 14: Acknowledge
//
//  The Protection Processor sends the following commands to the Display Processor:
//      Command = 4: Write Buffer Response
//      Command = 6: Write Buffer With Acknowledge
//      Command = 9: Execute Action With Acknowledge
//      Command = 14: Acknowledge
//
//  Protection Processor Receptions
//  Message bytes are received into a circular buffer (RxBuf[ ]) via DMA.  The buffer is checked for new
//  characters in the foreground.  Received characters are processed and a completed message is stored in a
//  separate buffer, RxMsg[ ].
//  Because there is limited RAM space, once a completed message is stored,
//    - Any remaining characters in RxBuf[ ] are dropped
//    - No new characters are processed (if new ones arrive through DMA) until the RxMsg[ ] is freed
//  The Display processor must honor the Turnaround Time of the Protection Processor to ensure messages are
//  not dropped.  However, the message protocol is set up such that neither processor needs to wait a long
//  time for a response.  If data is read or written that cannot be done immediately (such as setpoints that
//  must be read or written to Flash), the Read Delayed or execute Action w/ Check commands are used.  This
//  allows the receiving processor to respond immediately to the command, perform the requested action in
//  the background, and then respond when ready or indicate the action has been completed when requested.
//
//  Messages received from the Display Processor may be from any of 9 sources.  These sources are in the
//  Source Address field (bits 4-7 of byte 3) of the received message.  Source addresses are listed below:
//      Source Address = 4: LCD Display
//      Source Address = 6: DCI (Timed RTD Buffers - refresh metered values in the DCI)
//      Source Address = 7: Bluetooth
//      Source Address = 8: USB
//      Source Address = 9: Ethernet #1
//      Source Address = 10: Ethernet #2
//      Source Address = 11: Ethernet #3
//      Source Address = 12: Ethernet #4
//      Source Address = 13: Ethernet #5
//  The Protection Processor does not "care" about the Source Address, other than to verify that it is
//  valid.  It merely places this address in the Destination Address field of the response.
//  The Protection Processor uses one Source Address - 5.

//  Protection Processor Transmissions
//  There are two types of Protection Processor transmissions:
//    1) Unsolicited writes.  These utilize the Write w/ Ack or Execute Action w/ Ack commands.  The write
//       is not to a particular port; it is to the "global" Display Processor, and so the Destination
//       Address is always the DCI.
//       The Write w/ Ack is used to refresh the metered values in the DCI of the Display Processor.
//       The Execute Action w/ Ack is used to update time in the Display Processor (not sure if needed)
//    2) Responses.  These utilize the Write Buffer Response or Acknowledge commands.  The address is one of
//       the ports; the DCI does not "request" data.  These are responses to Read Requests, Write w/ Ack, or
//       Execute Action w/ Ack commands.
//       If the received command is a Read request, the response consists of either real-time data that is
//       not stored in the DCI (like Harmonics), Setpoints, or Events (including waveforms).  If the
//       received command is a Write or Execute Action, the response is an Ack.
//
//
//  DispComm_Tx()
//  This is the transmission handler.  This handler does the processing for transmission requests.  There
//  are four types of requests:
//      - Ack responses.  These are generated when a Write Buffer With Acknowledge (command = 5), Execute
//        Time/Date With Acknowledge (command = 6), or Execute Action With Acknowledge (command = 7) command
//        is received.  The ACK is assembled based on the value in DPComm.AckNak, and transmissions are
//        initiated.  Note, although a NAK may be generated for a Read Request (command = 2) command, it is
//        not signaled by this flag - it is just placed in the response buffer in the same manner as a Write
//        Buffer Response
//      - Read Request Received.  These are generated when a Read Request (command = 2) is received.  The
//        command is parsed, the response is generated - either a Write Buffer Response or a NAK, and
//        transmissions are initiated.  Note, the Read Request command is parsed and a response is assembled
//        DispComm_Tx(), rather than DispComm_Rx().  This ensures that the transmit buffer cannot be
//        overwritten during a transmission.  The communications protocol is full-duplex.  It is possible in
//        the future for the Protection Processor to transmit an unsolicited Write (command = 4 or 5) while
//        it is receiving a message.  We cannot allow the response to the newly-received message to be
//        generated until the Tx buffer is free.
//      - Message Requests.  These are generated when a request to transmit a Write Response message
//        (GEN_WRITERESP is set in DPComm.Flags).  This occurs if a Read Delayed command was received from
//        the display processor, such as when events information is requested.  The command is processed in
//        the SPI2 manager subroutines.  When the data has been retrieved from FRAM or Flash, the flag is
//        set.  The message is then assembled and transmitted.
//      - A Real-Time Buffer transmission timer has expired.  This causes a Write Without Acknowledge
//        message to be transmitted, with the data being the corresponding buffer.
//  Message requests are the highest priority, then Ack responses, then Read Requests, and then Real-Time
//  Buffers.  Ack responses and Read Requests should not occur simultaneously, because the Display Processor
//  should wait long enough for Protection Processor to process the message and transmit the response,
//  before sending the next message.    *** DAH  THIS NEEDS TO BE REVISITED BECAUSE NEED TO GUARANTEE WE TRANSMIT (AT LEAST) THE VOLTAGES EVERY 200MSEC FOR THE TRANSFER FUNCTION!!!
//
//  DispComm_Rx()
//  This is the reception handler.  This handler does the processing for receptions from the Display
//  processor.  The receptions may be Read Requests, Writes, or Execute Actions.  Receptions are handled as
//  follows:
//      - parse the input packet and place the received message in RxMsg[]
//      - process the message based on the command and destination address
//          - Read Requests (command = 2, 3): the READ_REQ_RCVD flag is set, and the request is handled in
//            DispComm_Tx() as described above.
//          - Write Response (command = 4): not supported at present.  There does not appear to be any data
//              that the Protection Processor would request from the Display Processor
//          - Write Without Acknowledge (command = 5): for now, this is not supported.  For now, it is
//              believed that the only Writes from the Display Processor would be for setpoints.  These will
//              only be Writes With Acknowledge.
//          - Write With Acknowledge (command = 6): this is generated for a specific process, likely
//              setpoints.  The WRITE_MSG_RCVD flag is set and a timer is started.  The subroutine then
//              waits for the either the process to clear the flag, or for the timer to expire.  The process
//              is expected to generate clear the WRITE_MSG_RCVD flag, save the response status, and set the
//              ACK request flag.
//          - Execute Action With Acknowledge (command = 9): the action is handled immediately in
//              DispComm_Rx() if possible.  Otherwise, it is handled in the same manner as the Write With
//              Acknowledge command.
//          - Execute Action With Check (command = 10): this is used for actions that require accessing FRAM
//              or Flash.  Set flags to initiate the FRAM and Flash Writes, and initialize the Check byte to
//              IN_PROGRESS.  When the writes have been completed, the Check byte is set to COMPLETED.  The
//              check byte can be read at any time by the Display Processor.  Also set the TXACK flag to
//              initiate an Ack response to this message.  Once the ACK has been sent, new messages may be
//              processed, since the existing message has been saved, provided the new message is NOT
//              another save setpoints command (this should never happen because only one session can be
//              opened at a time).  If the Check byte is IN_PROGRESS when this message is received, the unit
//              will NAK the command.
//          - Acknowledge (command = 15): This is a response to a Write or Execute Action message that was
//              sent by the Protection Processor.  At present, this is not supported.
//
//------------------------------------------------------------------------------------------------------------




//
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DispComm_VarInit()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Processor Communications Variable Initialization
//
//  MECHANICS:          This subroutine initializes the variables used for Display Processor communications.
//                      The following variables do not need initialization:
//                        DPComm.RxBuf[]              DPComm.TxBuf[]              DPComm.RxMsg[]
//                        DPComm.RxMsgSav[]           DPComm.TxSegCharCnt         DPComm.XmitReqClrMsk
//                        DPComm.RxSegCode            DPComm.RxSegOffset          DPComm.AckNak
//                        DPComm.RxCharsLeft          DPComm.RxMsgNdx             DPComm.RxCRC
//                        DPComm.TxSegNdx             DPComm.TxNdx                DPComm.TxCRC
//                        DPComm.XmitWaitTimer        DPComm.WaitTime             DP_OutSyncTime.xx
//                        TestInjVars.Phase           TestInjVars.Current         TestInjVars.TestTime
//                        TestInjVars.PriSecCos
//                        DPComm61850.TxSeqNum[]             DPComm61850.RxVars.RxBuf[]
//                        DPComm61850.RxVars.RxMsg[]         DPComm61850.RxVars.RxSegCode
//                        DPComm61850.RxVars.RxSegOffset     DPComm61850.RxCharsLeft
//                        DPComm61850.RxVars.RxMsgNdx        DPComm61850.RxVars.RxCRC
//                        DPComm61850.XmitWaitTimer[]        DPComm61850.TxVars.TxNdx
//                        DPComm61850.TxVars.TxSegNdx        DPComm61850.TxVars.TxCRC
//                        DPComm61850.TxVars.TxBuf[]         DPComm61850.TxVars.TxSegCharCnt
//
//  CAVEATS:            Call only during initialization.
//
//  INPUTS:             None
//
//  OUTPUTS:            DPComm.RxBufNdx, DPComm.Flags, DPComm.TxState, DPComm.RxCharsLeft, DPComm.RxState,
//                      DPComm.AssRxPktState, DPComm.Addr, DPComm.SeqNum, DPComm.SeqNumSaved,
//                      DPComm.RTD_XmitTimer[], DPComm.Check_Status, DPComm61850.Req[], DPComm61850.TxState,
//                      DPComm61850.RxVars.RxBufNdx, DPComm61850.SeqNum, DPComm61850.RxVars.RxCharsLeft,
//                      DPComm61850.RxVars.AssRxPktState, DPComm61850.RxVars.CharCount, xmitwait,
//                      DPTxReqFlags, IntSyncTime.xx, DispProc_FW_Rev, DispProc_FW_Ver, DispProc_FW_Build,
//                      TestInjVars.Type, TestInjVars.Status

//
//  ALTERS:             None
//
//  CALLS:              None
//
//  EXECUTION TIME:     Measured on 180625 (rev 0.25 code): 1.1usec
//
//------------------------------------------------------------------------------------------------------------

void DispComm_VarInit(void)
{
  uint8_t i;

  DPComm.RxBufNdx = 0;
  DPComm.Flags = 0;
  DPComm.TxState = 0;
  DPComm.AssRxPktState = 0;
  DPComm.RxState = 0;
  DPComm.Addr = 0x56;
  DPComm.SeqNum = 2;
  DPComm.Check_Status = CHK_STAT_COMPLETED; // Initialize to completed so we are ok to do a transaction
  // Saved sequence number must be initialized to a different value than the sequence number so that a
  //   received ACK won't be mistakenly tied to a Write With Ack transmission
  DPComm.SeqNumSaved = DPComm.SeqNum + 1;
  for (i=0; i<NUM_RTD_TIMESLICES; ++i)                // Initialize real time data buffer timers to send up
  {                                                   //   the values after 250msec.  This should be enough
    DPComm.RTD_XmitTimer[i] = RTDBUF_INTERVAL_TIME;   //   time to compute the 200msec values, and to allow
  }                                                   //   the display processor to power up and initialize

  // Initialize the number of chars left to the buffer size so that it matches the number of chars left in
  //   the buffer (DMA1_Stream5->NDTR) after a reset
  DPComm.RxCharsLeft = DISPCOMM_RXBUFSIZE;

  DPComm61850.RxVars.RxCharsLeft = DISPCOMM_61850RXBUFSIZE;
  DPComm61850.RxVars.AssRxPktState = 0;
  DPComm61850.RxVars.RxBufNdx = 0;
  DPComm61850.RxVars.CharCount = 0;
  DPComm61850.SeqNum = 2;
  for (i = 0; i < NUM_61850_REQUESTS; ++i)
  {
    DPComm61850.Req[i] = 0;
  }
  DPComm61850.TxState = 0;
  
  Setpoints10.stp.GC_DeviceName = 1;
  Setpoints10.stp.GC_MeteredValEnable = 1;
  Setpoints10.stp.GC_TripFailProtEnable = 1;
  Setpoints13.stp.GC_GlobalCapEnable = 1;
  Setpoints13.stp.GC_XferType = 1;
  Setpoints13.stp.GC_XferApp = 4;
  
  // Publish Initialization
  memset(&PXR35_CB_Publish_Data, 0, sizeof(PXR35_CB_Publish_Data));
  PXR35_CB_Publish_Data.CB_Status_Ctrl.CB_Pos_Status = ON;
  PXR35_CB_Publish_Data.CB_Status_Ctrl.Device_ID = 1;
  PXR35_CB_Publish_Data.CB_Meter_values.Device_ID = 1;
  
  // Subscription Initialization
  for (char x = 0; x < (MAX_GOOSE_SUB - 1); x++)
  {    
    memset(&Sub_GoCB_Status_Ctrl_Pkt[x], 0, sizeof(Sub_GoCB_Status_Ctrl_Pkt[x]));
    Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CB_Pos_Status = OFF;
    Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.SPSet_Select = DEFAULT_SETPOINT_SET;
  }
  
  StartUP_Status_Send = TRUE;
  
  xmitwait = 0;
  DPTxReqFlags = 0;

  IntSyncTime.Time_secs = 0;
  IntSyncTime.Time_nsec = 0;

  DispProc_FW_Rev = 0;
  DispProc_FW_Ver = 0;
  DispProc_FW_Build = 0;

  TestInjVars.Type = TESTINJ_TYPE_NOTEST;
  TestInjVars.Status = TESTINJ_STAT_NONE;

  ddcnttripack = 0;
  ddcntzsiack = 0;
  ddcnttriptx = 0;
  ddcntzsitx = 0;
  ddcntseqnum = 0;
  ddintcnt = 0;

  RelayFlagStp = FALSE;

  DP_Tmr4BufSel = 0;
  DP_Tmr5BufSel = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          DispComm_VarInit()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleTxPkt()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Insert Data For Transmission Subroutine
//
//  MECHANICS:          This subroutine inserts data into the transmit buffer according to the Advanced
//                      Communications Adapter Interface protocol
//
//  CAVEATS:            The length of the data to be inserted must be less than the Transmit Buffer size - 1
//                      IMPORTANT NOTE: The source of data cannot be the same as the destination (usually
//                        Tx_Buf) if the message length is greater than 126 bytes.  If none of the data is 0
//                        or 1, a byte must be stuffed in at index = 126.  This will cause the input data to
//                        be overwritten!!
//
//  INPUTS:             SrcPtr - pointer to the data to be inserted
//                      SrcLen - length (number of bytes) of data to be inserted
//                      port->TxNdx - Next open index in the transmit buffer
//                      port->TxSegCharCnt - Number of characters in the present segment
//                      port->TxSegNdx - Location (index) of the present segment code
//                      LastSet - True if this is the last set of data to be inserted into the packet
//
//  OUTPUTS:            port->TxBuf[] - Transmit buffer
//
//  ALTERS:             port->TxNdx - Next open index in the transmit buffer
//                      port->TxCRC - CRC of the message
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void AssembleTxPkt(uint8_t *SrcPtr, uint16_t SrcLen, struct DISPCOMMVARS *port, uint8_t LastSet)
{
  uint16_t SrcNdx;
  uint8_t nxt_char, i;

  // Insert the data into the transmit buffer.  Check each character to see if it matches a packet
  //   delimiter.  If it does, this marks the end of a segement, so format it using the modified COBS
  //   technique described above.  The end of a segment is also reached after 126 characters
  for (SrcNdx=0; SrcNdx<SrcLen; ++SrcNdx)
  {
    if (++port->TxSegCharCnt > 126)         // Increment the segment character count.  If it is greater than
    {                                       //   126, there are no data chars in this segment that need to
      port->TxBuf[port->TxSegNdx] = 0xFF;   //   be encoded, so set the segment code to xFF, reset the
      port->TxSegCharCnt = 0;               //   segment character count for the next segment, set the
      port->TxSegNdx = port->TxNdx++;       //   segment index to the next spot in the Tx buffer, increment
      ++port->TxSegCharCnt;                 //   the Tx buffer index, and increment the segment character
    }                                       //   count (to account for the stuffed 0xFF byte)
    nxt_char = (port->TxCRC ^ SrcPtr[SrcNdx]) & 0x00FF;      // Update the CRC
    port->TxCRC = ((port->TxCRC ^ CRC_TABLE1[nxt_char]) >> 8) + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
    // If the data char does not match a delimiter, just store it in the transmit buffer and increment the
    //   index
    if (SrcPtr[SrcNdx] > 0x01)
    {
       port->TxBuf[port->TxNdx++] = SrcPtr[SrcNdx];
    }
    // Otherwise store the segment code at the present segment index, set the segment index to the next open
    //   spot in the Tx buffer, increment the Tx index (i.e., skip a spot to reserve for the next segment
    //   code), and reset the segment character count
    else
    {
      port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1)  // b7..b1 = segment length
                                        + SrcPtr[SrcNdx];                  // b0 = data val at next seg code
      port->TxSegNdx = port->TxNdx++;
      port->TxSegCharCnt = 0;
    }
  }
  if (LastSet)
  {
    // Insert the CRC bytes
    for (i=0; i<2; ++i)                     // Two CRC bytes - first do LS byte
    {
      if (++port->TxSegCharCnt > 126)         // Still need to check the segment character count in case we
      {                                       //   need to insert a segment code
        port->TxBuf[port->TxSegNdx] = 0xFF;
        port->TxSegCharCnt = 0;
        port->TxSegNdx = port->TxNdx++;
        ++port->TxSegCharCnt;
      }
      if (((uint8_t)(port->TxCRC)) > 0x01)  // If the CRC char does not match a delimiter, just store it in
      {                                     //   the transmit buffer, and increment the index
         port->TxBuf[port->TxNdx++] = (uint8_t)(port->TxCRC);
      }
      else                                  // Otherwise store the segment code, set the segment index to
      {                                     //   the next open spot in the Tx buffer, and inc the Tx index
        port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1) // b7..b1 = segment length
                                          + ((uint8_t)(port->TxCRC));       // b0 = CRC val at next seg code
        port->TxSegNdx = port->TxNdx++;
      }
      port->TxCRC = (port->TxCRC >> 8);     // Shift to do the CRC ms byte
    }
    // Insert the last segment code - makes sure it is xFF if the segment count is more than 126
    if (++port->TxSegCharCnt > 126)
    {
      port->TxBuf[port->TxSegNdx] = 0xFF;
    }
    else
    {
      port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1) + 1;
    }
    // Insert the end of packet char.  Note, TxNdx is incremented so it contains the number of bytes to be
    //   transmitted
    port->TxBuf[port->TxNdx++] = END_OF_PKT;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleTxPkt()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleTxPkt1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Insert Data For Transmission Subroutine
//
//  MECHANICS:          This subroutine inserts data into the transmit buffer according to the Advanced
//                      Communications Adapter Interface protocol
//
//  CAVEATS:            The length of the data to be inserted must be less than the Transmit Buffer size - 1
//                      IMPORTANT NOTE: The source of data cannot be the same as the destination (usually
//                        Tx_Buf) if the message length is greater than 126 bytes.  If none of the data is 0
//                        or 1, a byte must be stuffed in at index = 126.  This will cause the input data to
//                        be overwritten!!
//
//  INPUTS:             SrcPtr - pointer to the data to be inserted
//                      SrcLen - length (number of bytes) of data to be inserted
//                      port->TxNdx - Next open index in the transmit buffer
//                      port->TxSegCharCnt - Number of characters in the present segment
//                      port->TxSegNdx - Location (index) of the present segment code
//                      LastSet - True if this is the last set of data to be inserted into the packet
//
//  OUTPUTS:            port->TxBuf[] - Transmit buffer
//
//  ALTERS:             port->TxNdx - Next open index in the transmit buffer
//                      port->TxCRC - CRC of the message
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

void AssembleTxPkt1(uint8_t *SrcPtr, uint16_t SrcLen, struct DPCOMMTXVARS *port, uint8_t LastSet)
{
  uint16_t SrcNdx;
  uint8_t nxt_char, i;

  // Insert the data into the transmit buffer.  Check each character to see if it matches a packet
  //   delimiter.  If it does, this marks the end of a segement, so format it using the modified COBS
  //   technique described above.  The end of a segment is also reached after 126 characters
  for (SrcNdx=0; SrcNdx<SrcLen; ++SrcNdx)
  {
    if (++port->TxSegCharCnt > 126)         // Increment the segment character count.  If it is greater than
    {                                       //   126, there are no data chars in this segment that need to
      port->TxBuf[port->TxSegNdx] = 0xFF;   //   be encoded, so set the segment code to xFF, reset the
      port->TxSegCharCnt = 0;               //   segment character count for the next segment, set the
      port->TxSegNdx = port->TxNdx++;       //   segment index to the next spot in the Tx buffer, increment
      ++port->TxSegCharCnt;                 //   the Tx buffer index, and increment the segment character
    }                                       //   count (to account for the stuffed 0xFF byte)
    nxt_char = (port->TxCRC ^ SrcPtr[SrcNdx]) & 0x00FF;      // Update the CRC
    port->TxCRC = ((port->TxCRC ^ CRC_TABLE1[nxt_char]) >> 8) + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
    // If the data char does not match a delimiter, just store it in the transmit buffer and increment the
    //   index
    if (SrcPtr[SrcNdx] > 0x01)
    {
       port->TxBuf[port->TxNdx++] = SrcPtr[SrcNdx];
    }
    // Otherwise store the segment code at the present segment index, set the segment index to the next open
    //   spot in the Tx buffer, increment the Tx index (i.e., skip a spot to reserve for the next segment
    //   code), and reset the segment character count
    else
    {
      port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1)  // b7..b1 = segment length
                                        + SrcPtr[SrcNdx];                  // b0 = data val at next seg code
      port->TxSegNdx = port->TxNdx++;
      port->TxSegCharCnt = 0;
    }
  }
  if (LastSet)
  {
    // Insert the CRC bytes
    for (i=0; i<2; ++i)                     // Two CRC bytes - first do LS byte
    {
      if (++port->TxSegCharCnt > 126)         // Still need to check the segment character count in case we
      {                                       //   need to insert a segment code
        port->TxBuf[port->TxSegNdx] = 0xFF;
        port->TxSegCharCnt = 0;
        port->TxSegNdx = port->TxNdx++;
        ++port->TxSegCharCnt;
      }
      if (((uint8_t)(port->TxCRC)) > 0x01)  // If the CRC char does not match a delimiter, just store it in
      {                                     //   the transmit buffer, and increment the index
         port->TxBuf[port->TxNdx++] = (uint8_t)(port->TxCRC);
      }
      else                                  // Otherwise store the segment code, set the segment index to
      {                                     //   the next open spot in the Tx buffer, and inc the Tx index
        port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1) // b7..b1 = segment length
                                          + ((uint8_t)(port->TxCRC));       // b0 = CRC val at next seg code
        port->TxSegNdx = port->TxNdx++;
      }
      port->TxCRC = (port->TxCRC >> 8);     // Shift to do the CRC ms byte
    }
    // Insert the last segment code - makes sure it is xFF if the segment count is more than 126
    if (++port->TxSegCharCnt > 126)
    {
      port->TxBuf[port->TxSegNdx] = 0xFF;
    }
    else
    {
      port->TxBuf[port->TxSegNdx] = ((port->TxNdx - port->TxSegNdx) << 1) + 1;
    }
    // Insert the end of packet char.  Note, TxNdx is incremented so it contains the number of bytes to be
    //   transmitted
    port->TxBuf[port->TxNdx++] = END_OF_PKT;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleTxPkt1()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DispComm_Tx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Processor Communications Top-Level Transmit Subroutine
//
//  MECHANICS:          This subroutine handles requests to transmit a message to the Display Processor.
//                      There are four sources of requests:
//                        - A request to transmit an Acknowledge message (TX_ACK is set in DPComm.Flags).
//                          This occurs when a Write With Acknowledge, Execute Action, or Execute Date/Time
//                          command is received from the Display Processor.  The command is processed in
//                          Disp_Rx() and the TX_ACK flag is set after the processing is done.
//                        - A Read Request message from the Display Processor has been received
//                          (READ_REQ_RCVD is set in DPComm.Flags).  This will cause a Write Buffer Response
//                          message or NAK (if the Read Request is invalid) to be generated.  The
//                          READ_REQ_RCVD flag is set in Disp_Rx() when the Read Request message is
//                          received.  It is processed in this subroutine, and the response is also
//                          generated here.  This is done because the Read Request may be received as we are
//                          transmitting.  This ensures that the transmit buffer is not changed until any
//                          existing transmission is finished.
//                        - A request to transmit a Write Response message (GEN_WRITERESP is set in
//                          DPComm.Flags).  This occurs if a Read Delayed command was received from the
//                          display processor, such as when events information is requested.  The command
//                          is processed in the SPI2 manager subroutines.  When the data has been retrieved
//                          from FRAM or Flash, the flag is set.  The message is then assembled and
//                          transmitted.
//                        - A Real-Time Buffer transmission timer has expired.  This causes a Write Without
//                          Acknowledge message to be transmitted, with the data being the corresponding
//                          buffer.
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.Flags, DPComm.RxMsg[], SPI2_buf[]
//
//  OUTPUTS:            DPComm.TxBuf[], DMA1_Stream6 register
//
//  ALTERS:             DPComm.TxState, DPComm.XmitReqClrMsk, DPComm.Flags
//
//  CALLS:              AssembleAck(), ProcReadReqImm(), ProcReadReqDel(), AssembleTxPkt(),
//                      AssembleExActBuffer(), Get_InternalTime()
//
//------------------------------------------------------------------------------------------------------------

void DispComm_Tx(void)
{
  uint16_t i, msglen;

  switch (DPComm.TxState)
  {
    case 0:                             // Idle
    default:
      if (DPComm.Flags & GEN_WRITERESP)       // If write response request received...
      {
        // This needs to be the highest priority.  Testing on 201012 showed that if ACKS or Read Requests
        // are higher, they can starve the Read Delayed response if the Display Processor has continuous
        // requests to transmit Read Immediates or Execute Actions.  The Display Processor transmits these
        // while waiting for the Write Response to the Delayed Read.  In this scenario, the Protection
        // Processor won't ever get to transmit the response - it will keep servicing the Read Immediate
        // requests.  Making this the highest priority fixes that.
        // Assemble the message length. This is 10 + the data length, not including the checksum and end of
        //   message bytes
        msglen = (uint16_t)DPComm.TxDelMsgBuf[8] + (((uint16_t)DPComm.TxDelMsgBuf[9]) << 8) + 10;
        if (msglen < DISPCOMM_TXBUFSIZE)        // Sanity check - should be ok
        {
          DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
          DPComm.TxSegCharCnt = 0;
          DPComm.TxNdx = 2;
          DPComm.TxCRC = 0xFFFF;
          AssembleTxPkt(&DPComm.TxDelMsgBuf[2], (msglen-2), &DPComm, TRUE);
        }
        else
        {
          AssembleAck1(DP_NAK_GENERAL);         // If bad for some reason, send NAK
          for (i=0; i<11; ++i)                  // Transfer the NAK from SPI2_buf[] to the transmit buffer
          {
            DPComm.TxBuf[i] = SPI2_buf[i];
          }
        }
        DMA1_Stream6->NDTR &= 0xFFFF0000;       // Set up the DMA
        DMA1_Stream6->NDTR |= DPComm.TxNdx;
        // Must clear all event flags before initiating a DMA operation
        DMA1->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                                + DMA_HIFCR_CFEIF6);
        DMA1_Stream6->CR |= 0x00000001;         // Initiate the DMA to transmit the data
        // Clear the delayed request in progress flag - we are ready to receive another message
        DPComm.Flags &= (DEL_MSG_INPROG ^ 0xFF);
        DPComm.Flags &= (GEN_WRITERESP ^ 0xFF); // Clear the request
        DPComm.WaitTime = TURNAROUND_TIME;      // Set wait time to Turnaround Time since no resp expected
        DPComm.TxState = 1;                     // Go to State 1 to check for message completion
      }
      else if (DPComm.Flags & TX_ACK)         // If ACK to transmit...
      {
        AssembleAck();                          // Assemble ACK message
        DMA1_Stream6->NDTR &= 0xFFFF0000;       // Set up the DMA
        DMA1_Stream6->NDTR |= DPComm.TxNdx;
        // Must clear all event flags before initiating a DMA operation
        DMA1->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                                + DMA_HIFCR_CFEIF6);
        DMA1_Stream6->CR |= 0x00000001;         // Initiate the DMA to transmit the data
        // Clear the Ack request flag - we are ready to process another received message
        DPComm.Flags &= (TX_ACK ^ 0xFF);
        DPComm.WaitTime = TURNAROUND_TIME;      // Set wait time to Turnaround Time since no resp expected
        DPComm.TxState = 1;                     // Go to State 1 to check for message completion
      }
      else if (DPComm.Flags & READ_REQ_RCVD)  // If read request received...
      {
        if (DPComm.RxMsg[0] == DP_CMND_RDIMM)   // Call the appropriate subroutine to process the request
        {
          ProcReadReqImm();
        }
        else
        {
          ProcReadReqDel();
        }
        DMA1_Stream6->NDTR &= 0xFFFF0000;       // Set up the DMA
        DMA1_Stream6->NDTR |= DPComm.TxNdx;
        // Must clear all event flags before initiating a DMA operation
        DMA1->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                                + DMA_HIFCR_CFEIF6);
        DMA1_Stream6->CR |= 0x00000001;         // Initiate the DMA to transmit the data
        // Clear the read request received flag - we are ready to process another received message
        DPComm.Flags &= (READ_REQ_RCVD ^ 0xFF);
        DPComm.WaitTime = TURNAROUND_TIME;      // Set wait time to Turnaround Time since no resp expected
        DPComm.TxState = 1;                     // Go to State 1 to check for message completion
      }
      else if (DPComm.Flags & TX_TIME)        // If transmit time request...
      {
        // For now, the display processor time is updated as follows:
        //   1) Disable interrupts
        //   2) Capture the present time and lower the time sync pin - this will generate an interrupt in
        //      the display processor
        //   3) Enable interrupts
        //   4) Delay ~10usec
        //   5) Raise the time sync pin
        //   6) send the captured time to the display processor
        __disable_irq();
        // Disable the interrupt that is generated by toggling the sync line
        EXTI->IMR &= 0xFFFFFDFF;
        Get_InternalTime(&DP_OutSyncTime);
        TIME_SYNC_OUTLOW;
        EXTI->PR = 0x00000200;            // Clear the interrupt that is generated by toggling this bit
        EXTI->IMR |= 0x00000200;          // Reenable the sync line interrupt
        __enable_irq();
        i = 250;                                // Measured time on 230828: ~12usec
        while (i > 0)
        {
          --i;
        }
        TIME_SYNC_INHIGH;
        DPComm.SeqNum++;                            // Send the time via an Execute Action w/ Ack command
        DPComm.SeqNumSaved = DPComm.SeqNum;
        AssembleExActBuffer(DP_EATYPE_TIME, DP_EAID_WRITETIME, 0x56);
        DPComm.Flags &= (0xFF ^ TX_TIME);
        DMA1_Stream6->NDTR &= 0xFFFF0000;                       // Set up the DMA
        DMA1_Stream6->NDTR |= DPComm.TxNdx;
        // Must clear all event flags before initiating a DMA operation
        DMA1->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                              + DMA_HIFCR_CFEIF6);
        DMA1_Stream6->CR |= 0x00000001;                         // Initiate the DMA to transmit the data
        DPComm.WaitTime = RESPONSE_TIME;        // Set wait time to Response Time since response expected
        DPComm.TxState = 1;                     // Go to State 1 to check for message completion
      }
      else                                    // See if we need to transmit an RTD buffer
      {
        for (i=0; i<NUM_RTD_TIMESLICES; ++i)              // Check the timers in round-robin fashion
        {                                                 //   RTD_TmrNdx holds the index to check
          if (++DPComm.RTD_TmrNdx >= NUM_RTD_TIMESLICES)  // If timer has expired, time to issue a Write of
          {                                               //   the corresponding buffer
            DPComm.RTD_TmrNdx = 0;
          }
          if (DPComm.RTD_XmitTimer[DPComm.RTD_TmrNdx] == 0)
          {
            break;
          }
        }
        if (i < NUM_RTD_TIMESLICES)             // If i < NUM_RTD_TIMESLICES, we broke out of the loop early
        {                                       //   due to an expired timer, so it is time to transmit
          DPComm.SeqNum++;                                        // Increment the sequence number and save
          DPComm.SeqNumSaved = DPComm.SeqNum;                     //   it to compare when ACK is received *** DAH  DO THIS WHEN EX ACT W/ ACK IS ADDED
          // Call subroutine to assemble the message
          BuildRTDBufByTSlice(DPComm.RTD_TmrNdx, DP_CMND_WRWACK, 0x56);
          DMA1_Stream6->NDTR &= 0xFFFF0000;                       // Set up the DMA
          DMA1_Stream6->NDTR |= DPComm.TxNdx;
          // Must clear all event flags before initiating a DMA operation
          DMA1->HIFCR |= (DMA_HIFCR_CTCIF6 + DMA_HIFCR_CHTIF6 + DMA_HIFCR_CTEIF6 + DMA_HIFCR_CDMEIF6
                                + DMA_HIFCR_CFEIF6);
          DMA1_Stream6->CR |= 0x00000001;                         // Initiate the DMA to transmit the data
          // Reset all of the timers to the normal (250msec) interval time, unless it is timer 3
          if (DPComm.RTD_TmrNdx != 3)
          {
            DPComm.RTD_XmitTimer[DPComm.RTD_TmrNdx] = RTDBUF_INTERVAL_TIME;
          }
          // Timer 3 handles RTD buffers 3, 17, and 18.  Only reset this timer if there are no more requests
          //   for any of these buffers. Otherwise the timer remains 0, and the next requested buffer will
          //   be transmitted after we have checked all of the other timers (i.e., the next time around)
          // In addition, reset this timer to the max time, because we only transmit this buffer on request
          //   by the application (typically, every 5 minutes for the 5-minute average values)
          else if ((DPTxReqFlags & DP_TXREQFLAGS_ALL) == 0)
          {
            DPComm.RTD_XmitTimer[DPComm.RTD_TmrNdx] = 0xFFFF;
          }
          DPComm.WaitTime = RESPONSE_TIME;        // Set wait time to Response Time since response expected
          DPComm.TxState = 1;                     // Go to State 1 to check for message completion
        }
      }
      break;
        
    case 1:                             // Transmitting
      // Check whether done transmitting
      if (DMA1->HISR & DMA_HISR_TCIF6)          // If transmission has been completed...
      {
        DMA1->HIFCR |= DMA_HISR_TCIF6;              // Reset the transfer complete interrupt flag
        DPComm.XmitWaitTimer = DPComm.WaitTime;     // Initialize the transmit wait timer
        // Go to next state to see whether ACK received or turnaround time has expired
        DPComm.TxState = 2;
      }
      else
      {
        break;
      }
      // *** DAH ADD ELSE CHECK FOR ERROR - NEED TIMEOUT

    case 2:                             // Wait For Turnaround Time or ACK
/*      // If we have received an ACK for a Write With Acknowledge command (sequence number matches) or the
      //   wait time has elapsed, we are ok to transmit another message
      if ( ( (DPComm.Flags & ACK_RECEIVED) && (DPComm.SeqNumSaved == DPComm.SeqNum) )
        || (DPComm.XmitWaitTimer == 0) )
      {
        DPComm.Flags &= (0xFF ^ ACK_RECEIVED);
        DPComm.TxState = 0;
      }   */

      // If we have received an ACK for a Write With Acknowledge command (sequence number matches) or the
      //   wait time has elapsed, we are ok to transmit another message
      if ( (DPComm.Flags & ACK_RECEIVED) && (DPComm.SeqNumSaved == DPComm.SeqNum) )
      {
        DPComm.Flags &= (0xFF ^ ACK_RECEIVED);
        DPComm.TxState = 0;
      }
      else if (DPComm.XmitWaitTimer == 0)
      {
        DPComm.Flags &= (0xFF ^ ACK_RECEIVED);
        DPComm.TxState = 0;
      }
      break;
      
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          DispComm_Tx()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleRxPkt()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Receive Packet Subroutine
//
//  MECHANICS:          This subroutine inserts data into the receive buffer according to the Advanced
//                      Communications Adapter Interface protocol
//
//  CAVEATS:            None
//
//  INPUTS:             DMA_Stream - the active DMA stream that is receiving characters
//                      *port - pointer to the active port
//                      port->RxBuf[] - Receive buffer (filled by the DMA stream)
//                      CRC_TABLE1[] - Constants used to compute the CRC
//                      DISPCOMM_RXBUFSIZE - Receive buffer size  *** RENAME THIS WHEN USED USED FOR CAM_COMM
//                      RXNDX_INCMASK - Mask used when incrementing RxBufNdx
//
//  OUTPUTS:            port->RxMsg[] - Received message
//                      The subroutine returns True when a message has been received.  It returns False
//                      otherwise
//
//  ALTERS:             port->AssRxPktState - State machine value
//                      port->RxCharsLeft - Number of characters left in the Rx FIFO
//                      port->RxBufNdx - Receiver buffer index
//                      port->RxMsgNdx - Received message buffer index
//                      port->RxSegCode - Most recent segment code that has been received
//                      port->RxSegOffset - Offset value of the most recent segment code
//                      port->RxCRC - CRC of the received message
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

uint8_t AssembleRxPkt(DMA_Stream_TypeDef *DMA_Stream, struct DISPCOMMVARS *port)
{
  uint8_t rxchar, nxt_char;
  uint16_t charcount, chars_left;

  // Capture the number of chars left in the buffer - need to take a snapshot in case this changes due to
  //   more characters being received while we are in this subroutine
  chars_left = DMA_Stream->NDTR;

  // Compute the number of chars to process.  We must take into account when the buffer circles back around
  // Note, the DMA controller will reset the number of chars left (NDTR) to the initial value,
  //   DISPCOMM_RXBUFSIZE, when it reaches 0.  This is ok, because charcount will compute to the same value
  //   in either case, i.e., chars_left=0 is the same as chars_left=DISPCOMM_RXBUFSIZE - in both cases
  //   charcount = RxCharsLeft
  if (chars_left <= port->RxCharsLeft)      // No wraparound
  {
    charcount = port->RxCharsLeft - chars_left;
  }
  else                                      // Buffer has gone past the end
  {
    charcount = (DISPCOMM_RXBUFSIZE + port->RxCharsLeft) - chars_left;
  }

  if (charcount == 0)                       // If no chars are received since the last time we were here,
  {                                         //   exit
    return (FALSE);
  }

  port->RxCharsLeft = chars_left;           // Update the number of chars left in the buffer

  while (charcount-- > 0)                   // Process the received chars and decrement the char count
  {
    rxchar = port->RxBuf[port->RxBufNdx];       // Get next char and increment index
    port->RxBufNdx = (port->RxBufNdx + 1) & RXNDX_INCMASK;

    if (rxchar == START_OF_PKT)                 // Any time a Start of Packet (SOP) char is received,
    {                                           //   restart processing
      port->AssRxPktState = 1;                      // Set state to 1
      port->RxMsgNdx = 0;                           // Clear message buffer index (char count)
      port->RxCRC = 0xFFFF;                         // Initialize CRC
      port->RxSegCode = 0xFF;                       // Set segment code to xFF so that when the first
    }                                               //   segment code is received, no data is stored
    else                                        // If not an SOP char...
    {
      if ( (rxchar == END_OF_PKT)                   // Check for an End of Packet (EOP) char.  If an EOP is
        && (port->AssRxPktState != 0) )             //   received in any state except 0 (when we are waiting
      {                                             //   for a new message), set the state to 3 to wrap up
        port->AssRxPktState = 3;                    //   the message
      }
      switch (port->AssRxPktState)                  // Enter the state machine to process the character
      {
        case 0:                                         // Wait for Start of Package (SOP) Char
        default:
          break;                                            // If waiting for SOP, break to check next char

        case 1:                                         // Process Segment Code
          // Segment code is:
          //   b7..1: Segment offset, which is the offset location of the next segment code
          //   b0:    Data value at the next segment code
          port->RxSegOffset = (rxchar >> 1);                // Save segment offset
          if (port->RxSegOffset == 0)                       // If invalid segment offset, set the state to 0
          {                                                 //   to restart
            port->AssRxPktState = 0;
          }
          else                                              // Otherwise the new segment code is valid
          {
            // If the old segment code is less than 126 (xFE), the b0 value has the encoded data value for
            //   this location (where the new segment code is).  If the old segment code is xFE or xFF,
            //   there are more than 126 data chars in the segment, so don't store any encoded value
            if ((port->RxSegCode & 0xFE) != 0xFE)
            {
              port->RxMsg[port->RxMsgNdx] = (port->RxSegCode & 0x01);
              nxt_char = (port->RxCRC ^ port->RxMsg[port->RxMsgNdx++]);   // Update CRC and inc Rx msg index
              port->RxCRC = ((port->RxCRC ^ CRC_TABLE1[nxt_char]) >> 8)
                    + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
            }
            port->RxSegCode = rxchar;                           // Save the new segment code
            if (--port->RxSegOffset > 0)                        // Decrement the new segment offset.  If it
            {                                                   //   is greater than 0, the next char is
              port->AssRxPktState = 2;                          //   data, so go to state 2 to process it
            }                                                   // Otherwise next char is another seg code
          }                                                     //   so remain in state 1 to process it
          break;

        case 2:                                         // Process Data Char
          if (port->RxMsgNdx < DISPCOMM_RXBUFSIZE)          // If room in message buffer, store the data
          {                                                 //   char and increment the index / char count
            port->RxMsg[port->RxMsgNdx] = rxchar;
            nxt_char = (port->RxCRC ^ port->RxMsg[port->RxMsgNdx++]);     // Update CRC and inc Rx msg index
            port->RxCRC = ((port->RxCRC ^ CRC_TABLE1[nxt_char]) >> 8)
                    + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
          }
          if (--port->RxSegOffset == 0)                     // Decrement the new segment offset.  If it has
          {                                                 //   reached 0, the next char is a segment code,
            port->AssRxPktState = 1;                        //   so go to state 1 to process it
          }                                                 // Otherwise the next char is another data char,
          break;                                            //   so remain in state 2 to process it

        case 3:                                         // End of Message
          port->AssRxPktState = 0;                          // Reset state
          // Make sure buffer index is at the next open location in the buffer since we may have dropped
          //   some extra characters
          port->RxBufNdx = (port->RxBufNdx + charcount) & RXNDX_INCMASK;
          return (TRUE);                                    // Return True
          break;

      }
    }

  }
  return (FALSE);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleRxPkt()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleRxPkt1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Receive Packet Subroutine
//
//  MECHANICS:          This subroutine inserts data into the receive buffer according to the Advanced
//                      Communications Adapter Interface protocol
//
//  CAVEATS:            None
//
//  INPUTS:             DMA_Stream - the active DMA stream that is receiving characters
//                      *port - pointer to the active port
//                      port->RxBuf[] - Receive buffer (filled by the DMA stream)
//                      CRC_TABLE1[] - Constants used to compute the CRC
//                      DISPCOMM_RXBUFSIZE - Receive buffer size  *** RENAME THIS WHEN USED USED FOR CAM_COMM
//                      RXNDX_INCMASK - Mask used when incrementing RxBufNdx
//
//  OUTPUTS:            port->RxMsg[] - Received message
//                      The subroutine returns True when a message has been received.  It returns False
//                      otherwise
//
//  ALTERS:             port->AssRxPktState - State machine value
//                      port->RxCharsLeft - Number of characters left in the Rx FIFO
//                      port->RxBufNdx - Receiver buffer index
//                      port->RxMsgNdx - Received message buffer index
//                      port->RxSegCode - Most recent segment code that has been received
//                      port->RxSegOffset - Offset value of the most recent segment code
//                      port->RxCRC - CRC of the received message
//
//  CALLS:              None
//
//------------------------------------------------------------------------------------------------------------

uint8_t AssembleRxPkt1(DMA_Stream_TypeDef *DMA_Stream, struct DPCOMMRXVARS *port)
{
  uint8_t rxchar, nxt_char;
  uint16_t chars_left;

  // Capture the number of chars left in the buffer - need to take a snapshot in case this changes due to
  //   more characters being received while we are in this subroutine
  chars_left = DMA_Stream->NDTR;

  // Compute the number of chars to process.  We must take into account when the buffer circles back around
  // Note, the DMA controller will reset the number of chars left (NDTR) to the initial value,
  //   DISPCOMM_61850RXBUFSIZE, when it reaches 0.  This is ok, because charcount will compute to the same value
  //   in either case, i.e., chars_left=0 is the same as chars_left=DISPCOMM_61850RXBUFSIZE - in both cases
  //   charcount = RxCharsLeft
  if (chars_left <= port->RxCharsLeft)      // No wraparound
  {
    port->CharCount += (port->RxCharsLeft - chars_left);
  }
  else                                      // Buffer has gone past the end
  {
    port->CharCount += ((DISPCOMM_61850RXBUFSIZE + port->RxCharsLeft) - chars_left);
  }

  if (port->CharCount == 0)                 // If no chars to process, exit
  {
    return (FALSE);
  }

  port->RxCharsLeft = chars_left;           // Update the number of chars left in the buffer

  while (port->CharCount > 0)                 // Process the received chars and decrement the char count
  {
    port->CharCount--;
    rxchar = port->RxBuf[port->RxBufNdx];       // Get next char and increment index
    port->RxBufNdx = (port->RxBufNdx + 1) & RXNDX_61850INCMASK;

    if (rxchar == START_OF_PKT)                 // Any time a Start of Packet (SOP) char is received,
    {                                           //   restart processing
      port->AssRxPktState = 1;                      // Set state to 1
      port->RxMsgNdx = 0;                           // Clear message buffer index (char count)
      port->RxCRC = 0xFFFF;                         // Initialize CRC
      port->RxSegCode = 0xFF;                       // Set segment code to xFF so that when the first
    }                                               //   segment code is received, no data is stored
    else                                        // If not an SOP char...
    {
      if ( (rxchar == END_OF_PKT)                   // Check for an End of Packet (EOP) char.  If an EOP is
        && (port->AssRxPktState != 0) )             //   received in any state except 0 (when we are waiting
      {                                             //   for a new message), set the state to 3 to wrap up
        port->AssRxPktState = 3;                    //   the message
      }
      switch (port->AssRxPktState)                  // Enter the state machine to process the character
      {
        case 0:                                         // Wait for Start of Package (SOP) Char
        default:
          break;                                            // If waiting for SOP, break to check next char

        case 1:                                         // Process Segment Code
          // Segment code is:
          //   b7..1: Segment offset, which is the offset location of the next segment code
          //   b0:    Data value at the next segment code
          port->RxSegOffset = (rxchar >> 1);                // Save segment offset
          if (port->RxSegOffset == 0)                       // If invalid segment offset, set the state to 0
          {                                                 //   to restart
            port->AssRxPktState = 0;
          }
          else                                              // Otherwise the new segment code is valid
          {
            // If the old segment code is less than 126 (xFE), the b0 value has the encoded data value for
            //   this location (where the new segment code is).  If the old segment code is xFE or xFF,
            //   there are more than 126 data chars in the segment, so don't store any encoded value
            if ((port->RxSegCode & 0xFE) != 0xFE)
            {
              port->RxMsg[port->RxMsgNdx] = (port->RxSegCode & 0x01);
              nxt_char = (port->RxCRC ^ port->RxMsg[port->RxMsgNdx++]);   // Update CRC and inc Rx msg index
              port->RxCRC = ((port->RxCRC ^ CRC_TABLE1[nxt_char]) >> 8)
                    + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
            }
            port->RxSegCode = rxchar;                           // Save the new segment code
            if (--port->RxSegOffset > 0)                        // Decrement the new segment offset.  If it
            {                                                   //   is greater than 0, the next char is
              port->AssRxPktState = 2;                          //   data, so go to state 2 to process it
            }                                                   // Otherwise next char is another seg code
          }                                                     //   so remain in state 1 to process it
          break;

        case 2:                                         // Process Data Char
          if (port->RxMsgNdx < DISPCOMM_61850RXBUFSIZE)     // If room in message buffer, store the data
          {                                                 //   char and increment the index / char count
            port->RxMsg[port->RxMsgNdx] = rxchar;
            nxt_char = (port->RxCRC ^ port->RxMsg[port->RxMsgNdx++]);     // Update CRC and inc Rx msg index
            port->RxCRC = ((port->RxCRC ^ CRC_TABLE1[nxt_char]) >> 8)
                    + ((CRC_TABLE1[nxt_char] & 0x00FF) * 256);
          }
          if (--port->RxSegOffset == 0)                     // Decrement the new segment offset.  If it has
          {                                                 //   reached 0, the next char is a segment code,
            port->AssRxPktState = 1;                        //   so go to state 1 to process it
          }                                                 // Otherwise the next char is another data char,
          break;                                            //   so remain in state 2 to process it

        case 3:                                         // End of Message
          port->AssRxPktState = 0;                          // Reset state
          return (TRUE);                                    // Return True
          break;

      }
    }

  }
  return (FALSE);

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleRxPkt1()
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ContinousDPCommRxtoSPI2BITSHIFT()
//------------------------------------------------------------------------------------------------------------
// FUNCTION:            Adds the new data to write to the SPI2_buf from the DPComm.RxMsg Buf
// MECHANICS:           Does this by using the length to detemine how big the amount of data recieved from the
//                      Rx Buf is. Start is the index in the category that the piece of data is in,
//                      This is used to insert it into the SPI2_buf exactly where it is needed.
// INPUTS:              Length and start of data, DispComm_Rx()
// OUTPUTS:             SPI2_buf
// CALLS:               None
void ContinousDPCommRxtoSPI2BITSHIFT(uint32_t length, uint16_t start)//***ALG adding location in CAT
{
  int eod_Length = length +start;
  uint16_t j = 8;
  for(int i =start; i < eod_Length; i++)
  {
    SPI2_buf[i] = (uint8_t) (DPComm.RxMsg[j]); // Data Byte
    // data in DPComm.RxMsg is already in uint8_t so no need to shift
    j++;
  }
}
//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ContinousDPCommRxtoSPI2BITSHIFT()
//------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DispComm_Rx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Processor Communications Top-Level Receive Subroutine
//
//  MECHANICS:          This subroutine handles receptions from the Display Processor:
//                        - Parse the input packets in RxBuf[] and assemble the received message in RxMsg[]
//                            Call AssembleRxPkt().  True is returned when a message is received
//                        - Check the message for validity
//                        - If it is valid, decode the command, save the appropriate information, and set
//                            the appropriate flag to complete processing the command.  This involves
//                            responding to the command and/or storing the data or executing an action
//                          Note, the response is not assembled in this subroutine TxBuf[] may be in use,
//                          that is, we may be transmitting while this message is being received.  We
//                          therefore just set a flag in this subroutine to signal the DispComm_Tx() to
//                          assemble the response when the TxBuf[] is free and we are able to transmit
//                      The following commands are supported:
//                        - x02: Read Buffer Request Immediate
//                          This is a request for data that can be retrieved immediately.  Set READ_REQ_RCVD
//                          flag to cause a Write Buffer Response to be transmitted in Disp_Tx(), then wait
//                          for the response to complete before parsing any additional receptions.  This
//                          ensures the existing message isn't corrupted before it is decoded and responded
//                          to.
//                        - x03: Read Buffer Request Delayed
//                          This is a request for information that will take time to retrieve.  Save the
//                          request, and set READ_REQ_RCVD flag to cause the command to be processed further
//                          in Disp_Tx().  An acknowledge response will be generated there.  When the data
//                          has been retrieved (via a separate subroutine), a flag is set to cause a Write
//                          Buffer Response to be transmitted in Disp_Tx().***********************, then wait
//                          for the response to complete before parsing any additional receptions.  This
//                          ensures the existing message isn't corrupted before it is decoded and responded
//                          to. to initiate the Flash Read.  When the data has been
//                          retrieved from Flash, the WRRESP flag will be set.  This will initiate a Write
//                          Buffer Response to be transmitted in Disp_Tx().  Also set the TXACK flag
//                          to initiate an Ack response to this message, and the RDDELAY_IN_PROG flag to
//                          indicate a delayed read is in progress.  Once the ACK has been sent, new
//                          messages may be processed, since the existing message has been saved, provided
//                          the new message is NOT another delayed read (it is up to the Display Processor
//                          ensure this does not occur).  If another Delayed Read is requested (the
//                          RDDELAY_IN_PROG flag is set), the unit will NAK the command.
//                        - x06: Write With Acknowledge
//                          This writes new Setpoints data into the the Frame FRAM and perhaps also the
//                          active setpoints.  The message is parsed and executed, and the TXACK flag is set
//                          to generate an Ack response.
//                        - x09: Execute Action With Acknowledge
//                          This is typically used to set time or initiate a Secondary Test.  We need only
//                          process the message and set the TXACK flag to generate an Ack response.
//                        - x0A: Execute Action With Check
//                          Also set the TXACK flag to initiate an Ack response to this message.  Once the
//                          ACK has been sent, new messages may be processed, since the existing message has
//                          been saved, provided the new message is NOT another save setpoints command (this
//                          should never happen because only one session can be opened at a time).  If the
//                          Check_Status byte is IN_PROGRESS when this message is received, the unit will
//                          NAK the command.
//                        - x0E: Acknowledge
//                          This is a response to a message (Write or Execute Action, for example) by some
//                          process we are running.  Signal the process that we have received the ACK by
//                          setting the ACK_RECEIVED flag, then wait for the process that initiated the
//                          message to clear the flag.
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.RxMsg[], DPComm.Flags
//
//  OUTPUTS:            DPComm.Flags, DPComm.Addr, DPComm.SeqNum, DPComm.AckNak, Prot_Enabled, SetpChkGrp
//
//  ALTERS:             DPComm.RxState
//
//  CALLS:              AssembleRxPkt(), TestInjCur(), Frame_FRAM_Write(), Load_SetpGr0_Gr1(), Gen_Values()
//
//------------------------------------------------------------------------------------------------------------

void DispComm_Rx(void)
{
  uint16_t bufID;

  switch (DPComm.RxState)
  {
    case 0:                             // Wait for message and process command
      // Call subroutine to process incoming characters.  Subroutine returns True when a completed message
      //   has been received.  The completed message is stored in DPComm.RxMsg[].
      //        RxMsg[0] - Command
      //        RxMsg[1] - Source/Destination Address
      //        RxMsg[2] - Sequence Number
      //        RxMsg[3] - Buffer Type
      //        RxMsg[4] - Buffer ID least significant byte
      //        RxMsg[5] - Buffer ID most significant byte
      //        RxMsg[6] - Buffer Length N least significant byte
      //        RxMsg[7] - Buffer Length N most significant byte
      //        RxMsg[8] - Data byte 0 (least significant byte)
      //            ...             ...
      //        RxMsg[N+7] - Data byte N-1 (most significant byte)
      //        RxMsg[N+8] - CRC LSB
      //        RxMsg[N+9] - CRC MSB
      if (AssembleRxPkt(DMA1_Stream5, &DPComm))
      {
        // The CRC of the incoming message is computed as the message is received and assembled in
        //   AssembleRxPkt().  It is taken over the entire message, including the CRC bytes.  If the CRC is
        //   valid, DPComm.RxCRC will be zero.
        if (DPComm.RxCRC == 0)
        {
          if ( (DPComm.RxMsg[1] == 0x45)            // Valid destination address is 5 (trip unit)
            || ( ((DPComm.RxMsg[1] & 0x0F) == 0x05) // Valid source addresses are 4, 6 - 13
              && ((DPComm.RxMsg[1] >= 0x65) && (DPComm.RxMsg[1] <= 0xD5)) ) )
          {
            DPComm.Addr = DPComm.RxMsg[1];        // Save the received source/destination address and
            DPComm.SeqNum = DPComm.RxMsg[2];      //   sequence number

            switch (DPComm.RxMsg[0])          // Switch on command
            {
              case DP_CMND_RDIMM:                 // Read Buffer Request Immediate
              case DP_CMND_RDDEL:                 // Read Buffer Request Delayed
                // Set flag to process it.  Cannot process it here, because we may be transmitting and we
                //   cannot overwrite the Tx buffer.  We will process this message in the transmission
                //   subroutine when the transmitter is free.
                DPComm.Flags |= READ_REQ_RCVD;        // Set flag to process read request
                // Set state to wait for the message to be processed and responded to.  This ensures RxMsg[]
                //   is not changed until it has been processed.
                DPComm.RxState = 1;
                break;

/*              case DP_CMND_WRRESP:                // Write Buffer Response  IT IS ASSUMED THE PROTECTION PROCESSOR WILL NOT NEED TO READ DATA FROM THE DISPLAY PROCESSOR
                // If command is a Write Response, some other process requested data.  If the process is
                //   still waiting for the data, set a flag for it to process the message.  If it is no
                //   longer waiting for the data, just drop the message.
                if (DPComm.Flags & WAITING_FOR_WRITERESP)
                {
                  DPComm.Flags |= RESPONSE_RECEIVED;
                  // Set state to wait for the response to be processed.  This ensures RxMsg[] is not
                  //   changed until it has been processed.
                  DPComm.RxState = 1;
                }
                break; */

              case DP_CMND_WRWACK:                // Write With Acknowledge
                // Process the message, store the data, and set flag to send back an ACK or NAK depending on
                //   the message status
                bufID = DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8);     // Assemble the Buffer ID
                // Check for setpoint write command.  bufid is the setpoints group

                if ((DPComm.RxMsg[3] == DP_BUFTYPE_SETP) && (bufID < NUM_STP_GROUPS))
                {
                  ProcWrSetpoints(bufID);
                }
                // Check for factory write command
                else if ( (DPComm.RxMsg[3] == DP_BUFTYPE_FACTORY)
                       && ((bufID == DP_FAC_BID_DISPFWVER) || (bufID <= LAST_FAC_BUFID)) )
                {
                  ProcWrFactoryConfig(bufID);
                }
                else                                            // Invalid type and ID - NAK
                {
                  DPComm.AckNak = DP_NAK_BUFINVALID;
                }
                DPComm.Flags |= TX_ACK;
                // Set state to wait for the Ack to be processed.  This ensures RxMsg[] is not changed
                //   until it has been processed.
                DPComm.RxState = 1;
                break;

              case DP_CMND_EXWACK:                // Execute Action With Acknowledge
                // Assemble the action ID (in bufID) and call subroutine to process the message.  Note,
                //   DPComm.RxMsg[3] contains the action type
                bufID = DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8);    // Assemble the Action ID
                ProcExActWAck(bufID, DPComm.RxMsg[3]);
                DPComm.Flags |= TX_ACK;
                // Set state to wait for the Ack to be processed.  This ensures RxMsg[] is not changed
                //   until it has been processed.
                DPComm.RxState = 1;
                break;
                
              case DP_CMND_EXWCHK:                // Execute Action With Check
                if ((DPComm.RxMsg[3] == DP_BUFTYPE_FACTORY) && ExAct.State == IDLE)
                {
                  bufID = DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8);
                  //uint8_t length =  DPComm.RxMsg[6] + (((uint16_t)DPComm.RxMsg[7]) << 8);  
                  // first data in package are on possition DPComm.RxMsg[8]
                  if(Load_ExecuteAction_struct(bufID, &DPComm.RxMsg[0]))
                  {
                    DPComm.Check_Status = CHK_STAT_IN_PROGRESS;       // Status is now in  progress
                    DPComm.AckNak = DP_ACK; 
                  }
                  else
                  {
                    DPComm.AckNak = DP_NAK_BUFINVALID;
                  }             
                }
                else                                            // Invalid type and ID - NAK */
                {
                  DPComm.AckNak = DP_NAK_BUFINVALID;
                }
                DPComm.Flags |= TX_ACK;
                // Set state to wait for the Ack to be processed.  This ensures RxMsg[] is not changed
                //   until it has been processed.
                DPComm.RxState = 1;
                break;

              case DP_CMND_ACK:                   // Acknowledge
                // If command is an Acknowledge, some other process sent a message requiring it.  If the
                //   process is still waiting for the Ack, set a flag for it to process the Ack.  If it is
                //   no longer waiting for the Ack, just drop it.
                // Note, this flag will be checked before we enter this subroutine again - it is safe to
                //   assume that whatever subroutine sent the message is now sitting on this flag.
                //   Therefore, we don't have to worry about the sequence number or flag being overwritten
//                if (DPComm.Flags & WAITING_FOR_ACK)
//                {
//                  set flag to notify port that we got an ack
//                }
                // Set flag to notify transmit routine that we got a response
                DPComm.Flags |= ACK_RECEIVED;
                break;

              default:                            // Invalid command
                DPComm.AckNak = DP_NAK_CMDINVALID;      // NAK for now since not supported
                DPComm.Flags |= TX_ACK;
                break;                                  // Ignore the message    *** DAH MAYBE ADD ERROR COUNTER HERE TO RESTART COMMS IF TOO MANY ERRORS
            }

          }
        }
      }
      break;

    case 1:                             // Wait for flags to clear
    default:
      // Don't process another message until any previously-queued messages are transmitted.  If either
      //   READ_REQ_RCVD or TX_ACK in DPComm.Flags is set, there is still a message to transmit.  We
      //   can't assemble another received message yet because the previous message may not have been
      //   processed yet
      if (!(DPComm.Flags & (READ_REQ_RCVD + TX_ACK)))
      {
        DPComm.RxState = 0;
      }
      else
      {
        // *** DAH  ADD WATCHDOG TIMER TO ABORT ALSO IF NO ACK AFTER SOME TIME - RESET AND RESTART COMMS!!
      }

  }
            
}


//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          DispComm_Rx()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcWrSetpoints()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Write Setpoints Message Subroutine
//
//  MECHANICS:          This subroutine handles a command to write setpoints from the display processor.
//                      It
//                          - parses the write request message contained in DPComm.RxMsg[]
//                          - stores the setpoints in the appropriate FRAM, if the message is valid
//                          - assembles the appropriate response (ack or nak) into DPComm.TxBuf[]
//
//  CAVEATS:            Note: Protection is disabled while the new setpoints are being written.  This only
//                      impacts the protection that is being done in the sampling interrupt, because the
//                      write is completed before this subroutine is exited
//
//  INPUTS:             bufid
//                      DPComm.RxMsg[]
// 
//  OUTPUTS:            Prot_Enabled
//
//  ALTERS:             DPComm.AckNak
//
//  CALLS:              Checksum8_16(), FRAM_Write(), Frame_FRAM_Write(), Gen_Values()
//
//------------------------------------------------------------------------------------------------------------

void ProcWrSetpoints(uint16_t bufid)
{
  uint16_t sum, faddr, i, k, numsetp;
  uint16_t *sptr;
  uint32_t newEID;

  // The message has already been checked for address, CRC, command, buffer type, and buffer ID validity.
  //   The Write Command is stored as follows:
  //        DPComm.RxMsg[0]- Command = x02
  //        DPComm.RxMsg[1]- Source/Destination Address
  //        DPComm.RxMsg[2]- Sequence Number
  //        DPComm.RxMsg[3]- Buffer Type
  //        DPComm.RxMsg[4]- Buffer ID least significant byte
  //        DPComm.RxMsg[5]- Buffer ID most significant byte
  //        DPComm.RxMsg[6]- Buffer Length N least significant byte
  //        DPComm.RxMsg[7]- Buffer Length N most significant byte
  //        DPComm.RxMsg[8]- Write Data
  //        DPComm.RxMsg[9]- Write Data
  //          ..........................
  //        DPComm.RxMsg[N]- Write Data

  // Check the buffer length and setpoints set to be written to.
  //   Notes:
  //     - the buffer length is the size of the setpoints structure (in bytes) - 4
  //       (checksum & checksum complement), plus add two bytes for the destination
  //       setpoints set and the reserved byte
  //     - the setpoints set to be written to is the first data byte (RxMsg[8])
  if ( ((DPComm.RxMsg[6] + (((uint16_t)DPComm.RxMsg[7]) << 8)) == (SETP_GR_SIZE[bufid] - 2))
    && (DPComm.RxMsg[8] < 4) && (Verify_Setpoints(bufid, (uint16_t *)&DPComm.RxMsg[10])))
  {
    // Compute the checksum of the new setpoints.
    //   Note, the first two bytes of the data (RxMsg[8] and RxMsg[9]) are NOT setpoints.  These are the
    //   setpoints set to be written to and a reserved byte.  The first setpoint is at RxMsg[10].  The last
    //   setpoint byte is at RxMsg[10 + (SETP_GR_SIZE[bufID] - 4) - 1].  Store the checksum in RxMsg[].
    //   Note, RxMsg[] is large enough to handle the extra 4 bytes
    sum = Checksum8_16(&DPComm.RxMsg[10], (SETP_GR_SIZE[bufid] - 4));
    DPComm.RxMsg[10 + SETP_GR_SIZE[bufid] - 4] = (uint8_t)sum;          // LS byte of chksum
    DPComm.RxMsg[10 + SETP_GR_SIZE[bufid] - 3] = (uint8_t)(sum >> 8);   // MS byte of chksum
    sum = (sum ^ 0xFFFF);                                               // Checksum comp
    DPComm.RxMsg[10 + SETP_GR_SIZE[bufid] - 2] = (uint8_t)sum;          // LS byte of comp
    DPComm.RxMsg[10 + SETP_GR_SIZE[bufid] - 1] = (uint8_t)(sum >> 8);   // MS byte of comp

    // Write the copies to FRAM
    if ( (bufid == 2) || (bufid == 3) || (bufid == 11) )   // Groups 2, 3, and 11 are stored in on-board
    {                                                      //   FRAM
      FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(bufid << 1) + 0], (SETP_GR_SIZE[bufid] >> 1),
                (uint16_t *)(&DPComm.RxMsg[10]));
      FRAM_Write(DEV_FRAM2, SETP_GR_FRAM_ADDR[(bufid << 1) + 1], (SETP_GR_SIZE[bufid] >> 1),
                (uint16_t *)(&DPComm.RxMsg[10]));
      
      // If this is Group 2 (Modbus), check for changes that require resetting the UART
      if ((bufid == 2) && 
            ((DPComm.RxMsg[11] != Setpoints2.stp.Modbus_Port_Baudrate) ||
             (DPComm.RxMsg[12] != Setpoints2.stp.Modbus_Port_Parity) ||
             (DPComm.RxMsg[13] != Setpoints2.stp.Modbus_Port_Stopbit)))
      {
         ModB.Reset_Req = TRUE;
      }
    }
    else                                      // All other groups are in the Frame FRAM
    {
      faddr = ( (bufid < 10) ?
         (SETP_GR_FRAM_ADDR[(bufid << 1) + 0] + (SETP_SET_ADDRESS_OFFSET * DPComm.RxMsg[8]))
       : (SETP_GR_FRAM_ADDR[(bufid << 1) + 0]) );
      Frame_FRAM_Write(faddr, SETP_GR_SIZE[bufid], &DPComm.RxMsg[10]);
      faddr = ( (bufid < 10) ?
         (SETP_GR_FRAM_ADDR[(bufid << 1) + 1] + (SETP_SET_ADDRESS_OFFSET * DPComm.RxMsg[8]))
       : (SETP_GR_FRAM_ADDR[(bufid << 1) + 1]) );
      Frame_FRAM_Write(faddr, SETP_GR_SIZE[bufid], &DPComm.RxMsg[10]);
    }

    // If this is the active setpoints set or Group 2, 3 or 11, also write to the active setpoints, and
    //   update the setpoints status
    if ( (DPComm.RxMsg[8] == SetpActiveSet) || (bufid == 2) || (bufid == 3) || (bufid == 11) )
    {
      sptr = SETP_GR_DATA_ADDR[bufid];                  // Set pointer to the first setpoint
      // SETP_GR_SIZE[ ] is the structure size.  Divide by 2 because each setpoint is a
      //   word.  Subtract 2 because the structure includes the checksum and checksum
      //   complement
      numsetp = ((SETP_GR_SIZE[bufid] >> 1) - 2);
      k = 10;
      // Disable protection while setpoints are being changed.  This only impacts the
      //   protection that is being done in the sampling interrupt
      // This takes about 92usec max (including interrupts)
      Prot_Enabled = FALSE;
      for (i = 0; i < numsetp; ++i)
      {
        *sptr = ( (DPComm.RxMsg[k]) + (((uint16_t)DPComm.RxMsg[k+1]) << 8));
        sptr++;
        k += 2;
      }
      Gen_Values();                     // Generate new protection values
      Prot_Enabled = TRUE;
      // Enter a setpoints download event
      i = DPComm.RxMsg[1] >> 4;            // Compute the event code based on the source address
      if (i == DP_DISP_ADD)
      {
        i = STP_DWNLD_DISPLAY;
      }
      else if (i == DP_USB_ADD)
      {
        i = STP_DWNLD_USB;
      }
      else if (i == DP_BT_ADD)
      {
        i = STP_DWNLD_BLUETOOTH;
      }
      else                                 // Must be Modbus TCP
      {
        i = STP_DWNLD_MODBUS_TCP;
      }
      newEID = InsertNewEvent(i);
      // If the demand setpoints changed, we need to restart logging with the new variables
      if ( (Dmnd_Setp_DemandWindow != (uint8_t)Setpoints0.stp.DemandWindow)
        || (Dmnd_Setp_DemandInterval != (uint8_t)Setpoints0.stp.DemandInterval)
        || (Dmnd_Setp_DemandLogInterval = (uint8_t)Setpoints0.stp.DemandLogInterval) )
      {
        Dmnd_VarInit();
        EngyDmnd[1].EID = newEID;
      }
      // Update setpoints status byte to "ok" for the group that was written
      if ( (bufid == 2) || (bufid == 3) || (bufid == 11) )   // If group 2, 3, or 11, set
      {                                                      //   status to 3: retrieved
        SetpointsStat |= (3 << (bufid << 1));                //   successfully from on-board
      }                                                      // FRAM
      else                                                   // Any other group, set status
      {                                                      //   to 0: retrieved
        SetpointsStat &= ~((3 << (bufid << 1)));             //   successfully from Frame
      }                                                      //   FRAM as a PXR35
    }
    DPComm.AckNak = DP_ACK;                             // Ack since command is valid
    if (bufid == 12) RelayFlagStp = TRUE;
    Admin_Verified_Tmr = (Admin_Verified_Tmr) ? (COUNT_10_MIN) : (Admin_Verified_Tmr);
    User_Verified_Tmr = (User_Verified_Tmr) ? (COUNT_10_MIN) : (User_Verified_Tmr);
  }
  else                                                  // Length is wrong - Nak
  {
    DPComm.AckNak = DP_NAK_CMDFORMAT;
  }
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ProcWrSetpoints()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcWrFactoryConfig()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Write Factory Configuration Subroutine
//
//  MECHANICS:          This subroutine handles a command to write factory configuration info from the
//                      display processor.  Note, "factory configuration info" includes internal commands to
//                      change setpoints!!  This subroutine:
//                          - parses the write request message contained in DPComm.RxMsg[]
//                          - stores the factory config info in the appropriate FRAM (and Flash, for some
//                            cal constants), if the message is valid
//                          - assembles the appropriate response (ack or nak) into DPComm.TxBuf[]
//                      There are 3 main portions to the code:
//                          - Retrieve the entire section of memory
//                          - Overwrite the existing values with the received values
//                          - Compute a new checksum and write the section back into memory
//                      There are numerous parameters that depend on the value(s) being written:
//                          - Memory: on-board FRAM, Frame FRAM, on-board Flash
//                          - Number of copies: 3 or 1
//                          - Default values (if all copies have checksum error): 0 or defined default
//                      This is handled in two ways:
//                          - Values that are handled in unique ways (FW version, Style) have their own
//                            bufid state with their own code
//                          - Values that are treated similarly are grouped into additional bufid states,
//                            and the bufid along with array FacBuf_Info[][] is used to control the
//                            processing
//
//  CAVEATS:            None
//
//  INPUTS:             bufid - the buffer id of the received message
//                      DPComm.RxMsg[] - the received message
//                      FacBuf_Info[][] - constant array theat controls the buffer processing
// 
//  OUTPUTS:            DispProc_FW_xx, Setpoints0.xx, Setpoints1.xx, Break_Config.xx, AFEcal.xx,
//                      ADCcalLow.xx, ADCcalHigh.xx, other manufacturing info in Frame FRAM,
//                      SPI1Flash.Req, DPComm.AckNak, DPTxReqFlags 
//
//  ALTERS:             SetpScratchBuf[], SPI2_buf[]
//
//  CALLS:              Get_Setpoints(), Checksum8_16(), FRAM_Write(), ExtCapt_FRAM_Write(),
//                      Frame_FRAM_Write(), FRAM_Stat_Write(), FRAM_Read(), Frame_FRAM_Read(),
//                      Gen_Values(), Save_Critical_BrkConfig(), ContinousDPCommRxtoSPI2BITSHIFT()
//
//------------------------------------------------------------------------------------------------------------

void ProcWrFactoryConfig(uint16_t bufid)
{
  uint8_t set, ok;
  uint16_t i, length_cat, sum, sumrd, comprd, EndOfDataCAT, address;
  int first_addressofCAT;
  union float_u32
  {
    float fval;
    uint32_t uval;
  } flt_uval;

  // The message has already been checked for address, CRC, command, buffer type, and buffer ID validity
  //   The Write Command is stored as follows:
  //        DPComm.RxMsg[0]- Command = x02
  //        DPComm.RxMsg[1]- Source/Destination Address
  //        DPComm.RxMsg[2]- Sequence Number
  //        DPComm.RxMsg[3]- Buffer Type
  //        DPComm.RxMsg[4]- Buffer ID least significant byte
  //        DPComm.RxMsg[5]- Buffer ID most significant byte
  //        DPComm.RxMsg[6]- Buffer Length N least significant byte
  //        DPComm.RxMsg[7]- Buffer Length N most significant byte
  //        DPComm.RxMsg[8]- Write Data
  //        DPComm.RxMsg[9]- Write Data
  //          ..........................
  //        DPComm.RxMsg[N]- Write Data

  // If this buffer is the display processor's FW version, we know the display processor is up and running.
  //   Store the display processor's FW version in RAM, then set the flags to write our FW version and the
  //   override micro's FW version to the display processor, along with the external diagnostics buffer
  if (bufid == DP_FAC_BID_DISPFWVER)    // Display Micro FW version
  {
    if ( (DPComm.RxMsg[6] + (((uint16_t)DPComm.RxMsg[7]) << 8)) == 4)       // Check length
    {
      DispProc_FW_Rev = DPComm.RxMsg[8];
      DispProc_FW_Ver = DPComm.RxMsg[9];
      DispProc_FW_Build = (((uint16_t)DPComm.RxMsg[11]) << 8) + DPComm.RxMsg[10];
      DPComm.AckNak = DP_ACK;                                               // Ack since command is valid
      DPTxReqFlags |= DP_VERREV_BUF_REQ;
      DPTxReqFlags |= DP_EXTDIAG_BUF_REQ;
      DPComm.RTD_XmitTimer[3] = 0;                // Reset the associated timer to initiate a transmission
    }
    else                                                                    // If length bad, NAK
    {
      DPComm.AckNak = DP_NAK_CMDFORMAT;
    }                 
  } 
  else if (bufid == DP_FAC_BID_NV)      // NV
  {
    FRAM_Write(DEV_FRAM2, FRAM_TEST_VAL, 1, (uint16_t*)&DPComm.RxMsg[8]);
    ExtCapt_FRAM_Write(EXTCAP_TEST_START, 1, (uint16_t*)&DPComm.RxMsg[10]);
    DPComm.AckNak = DP_ACK;
  }
  else if (bufid == DP_FAC_BID_STYLE || bufid == DP_FAC_BID_STYLE_2)
  {                                     // Style and Style 2
    // Save new style in temporary - use "sum" as the temporary
    sum = (((uint16_t)DPComm.RxMsg[9]) << 8) + DPComm.RxMsg[8];
    
    // Move new style into the appropriate setpoints in the active setpoints set
    Setpoints0.stp.Style = ((bufid == DP_FAC_BID_STYLE) ? sum : Setpoints0.stp.Style);
    Setpoints1.stp.Style = ((bufid == DP_FAC_BID_STYLE) ? sum : Setpoints0.stp.Style);

    Setpoints0.stp.Style_2 = ((bufid == DP_FAC_BID_STYLE_2) ? sum : Setpoints0.stp.Style_2);

    // Save the new setpoints in FRAM
    for(set = 0; set < 4; set++)
    {
      // First get Group0 setpoints - setpoints returned in SetpScratchBuf[]
      Get_Setpoints(set, 0, (uint8_t *)(&i));
      // Now save the following Group0 setpoints that are read-only in Group1.  These will overwrite the
      //   Group1 values:
      //        Group 0 Rating - SetpScratchBuf[0]            Group 1 Rating - SetpScratchBuf[0]    
      //        Group 0 Breakframe - SetpScratchBuf[1]        Group 1 Breakframe - SetpScratchBuf[1]
      //        Group 0 Style - SetpScratchBuf[2]             Group 1 Style - SetpScratchBuf[2]     
      //        Group 0 Style_2 - SetpScratchBuf[3]           Group 1 Style_2 - SetpScratchBuf[3]   
      //        Group 0 SG_Sensor - SetpScratchBuf[22]        Group 1 SG_Sensor - SetpScratchBuf[32]
      //   We will save these at the end of SetpScratchBuf[], because we know that SetpScratchBuf[] is
      //     much bigger than the number of Group1 setpoints
      SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 5] = SetpScratchBuf[0];
      SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 4] = SetpScratchBuf[1];
      SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 3] = SetpScratchBuf[2];
      SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 2] = SetpScratchBuf[3];
      SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 1] = SetpScratchBuf[22];

      SetpScratchBuf[2] = Setpoints0.stp.Style;
      SetpScratchBuf[3] = Setpoints0.stp.Style_2;

      sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_SIZE[0] - 4));
      comprd = (sum ^ 0xFFFF);
      SetpScratchBuf[(SETP_GR_SIZE[0] >> 1) - 2] = sum;         // Size buffers are in bytes
      SetpScratchBuf[(SETP_GR_SIZE[0] >> 1) - 1] = comprd;

      Frame_FRAM_Write((SETP_GR_FRAM_ADDR[0] + (SETP_SET_ADDRESS_OFFSET * set)),
                              SETP_GR_SIZE[0], (uint8_t *)(&SetpScratchBuf[0]));
      Frame_FRAM_Write((SETP_GR_FRAM_ADDR[0 + 1] + (SETP_SET_ADDRESS_OFFSET * set)),
                              SETP_GR_SIZE[0], (uint8_t *)(&SetpScratchBuf[0]));  

      // Now get the Group1 setpoints - setpoints returned in SetpScratchBuf[]
      Get_Setpoints(set, 1, (uint8_t *)(&i));
      // Finally, overwrite the aforementioned read-only setpoints with the Group0 setpoints
      SetpScratchBuf[0] = SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 5];
      SetpScratchBuf[1] = SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 4];
      SetpScratchBuf[2] = Setpoints0.stp.Style;
      SetpScratchBuf[3] = Setpoints0.stp.Style_2;
      SetpScratchBuf[32] = SetpScratchBuf[(SETP_SCRATCHBUF_SIZE >> 1) - 1];

      sum = Checksum8_16((uint8_t *)(&SetpScratchBuf[0]), (SETP_GR_SIZE[1] - 4));
      comprd = (sum ^ 0xFFFF);
      SetpScratchBuf[(SETP_GR_SIZE[1] >> 1) - 2] = sum;         // Size buffers are in bytes
      SetpScratchBuf[(SETP_GR_SIZE[1] >> 1) - 1] = comprd;
      
      Frame_FRAM_Write((SETP_GR_FRAM_ADDR[1<<1] + (SETP_SET_ADDRESS_OFFSET * set)),
                              SETP_GR_SIZE[1], (uint8_t *)(&SetpScratchBuf[0]));
      Frame_FRAM_Write((SETP_GR_FRAM_ADDR[(1<<1) + 1] + (SETP_SET_ADDRESS_OFFSET * set)),
                              SETP_GR_SIZE[1], (uint8_t *)(&SetpScratchBuf[0]));  
    }
    
    DPComm.AckNak = DP_ACK;    // Ack since command is valid  
  }
  else if(bufid == DP_FAC_BID_BIO)
  {
    if(DPComm.RxMsg[8] & BIT3)
    {
      DIO3_OUT_HIGH;
    }
    else
    {
      DIO3_OUT_LOW;
    }
    if(DPComm.RxMsg[8] & BIT4)
    {
      DIO4_OUT_HIGH;
    }
    else
    {
      DIO4_OUT_LOW;
    }
    DPComm.AckNak = DP_ACK;
  }

  // All other buffers...
  // Measured the execution time to write a configuration value on 231214:
  //   Worst-case condition, if all three copies have to be read: 4.33msec
  // Although this is far too long for normal operation, this is for a factory command - the customer
  //   cannot execute this command, and so we should be ok with this time, even if it extends the main loop
  //   past 16msec
  // Check the buffer length
  else if ( (DPComm.RxMsg[6] + (((uint16_t)DPComm.RxMsg[7]) << 8)) == FacBuf_Info[bufid][1] )
  {
    //----------------- First retrieve the existing info.  Store in SPI2_buf[] -----------------------------
    length_cat = FacBuf_Info[bufid][5];
    first_addressofCAT = FacBuf_Info[bufid][3];

    ok = TRUE;                          // Assume data is ok
    // If retrieving existing info from FRAM...
    if ( (FacBuf_Info[bufid][2] == 0) || (FacBuf_Info[bufid][2] == 1) )
    {
      ok = FALSE;                           // Assume checksums are bad
      set = ( (bufid == 27) ? 1 : 3);       // BID = 27 has only one copy; the rest have 3 copies
      for (i = 0; i < set; ++i)             //   "set" is used as a temporary
      {
        // 0x10000 offsets are to make the FRAM protection addresses smaller in the table
        if(FacBuf_Info[bufid][2] == 0)
        {
          FRAM_Read(((first_addressofCAT + 0x10000) + (i * (length_cat+4))), length_cat, (uint16_t*)&SPI2_buf[0]);
        }
        else
        {
          Frame_FRAM_Read((first_addressofCAT + (i * (length_cat+4))), length_cat, &SPI2_buf[0]);
        }
        // If the checksum is good, done
        sum = Checksum8_16((uint8_t *)(&SPI2_buf[0]), (length_cat-4));
        sumrd = SPI2_buf[length_cat-4] + (((uint16_t)SPI2_buf[length_cat-3]) << 8);
        comprd = SPI2_buf[length_cat-2] + (((uint16_t)SPI2_buf[length_cat-1]) << 8);
        if ( (sum == sumrd) && (sum == (comprd ^ 0xFFFF)) )
        {
          ok = TRUE;
          break;
        }
      }
    }
    // If we are changing on-board AFE cal constants - need to set request to read
    else if (FacBuf_Info[bufid][2] == 4)
    {
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_AFECAL_RD;                               // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_AFECAL_RD) != S1F_AFECAL_RD)      // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD);                  // Clear the request flag
      // IMPORTANT NOTE: AT THIS POINT, THE CAL CONSTANTS HAVE CHANGED FROM THE COMBINED CAL CONSTANTS TO
      //                 THE BOARD CAL CONSTANTS.  THE ROGOWSKI CAL FACTORS ARE NOT BEING USED FOR
      //                 GENERATING THE THE METERED CURRENTS!!
      // AFEcal.xxx has the cal constants.  The subroutine takes care of whether they are defaults, etc.,
      //   so set ok to True
      ok = TRUE;
    }
    // If we are changing on-board ADCH cal constants - need to set request to read
    else if (FacBuf_Info[bufid][2] == 5)
    {
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_ADCHCAL_RD;                              // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_ADCHCAL_RD) != S1F_ADCHCAL_RD)    // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_ADCHCAL_RD);                 // Clear the request flag
      // IMPORTANT NOTE: AT THIS POINT, THE CAL CONSTANTS HAVE CHANGED FROM THE COMBINED CAL CONSTANTS TO
      //                 THE BOARD CAL CONSTANTS.  THE ROGOWSKI CAL FACTORS ARE NOT BEING USED FOR
      //                 GENERATING THE THE METERED CURRENTS!!
      // ADCcalHigh.xxx has the cal constants.  The subroutine takes care of whether they are defaults,
      //   etc., so set ok to True
      ok = TRUE;
    }
    // If we are changing on-board ADCL cal constants - need to set request to read
    else if (FacBuf_Info[bufid][2] == 6)
    {
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_ADCLCAL_RD;                              // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_ADCLCAL_RD) != S1F_ADCLCAL_RD)    // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_ADCLCAL_RD);                 // Clear the request flag
      // IMPORTANT NOTE: AT THIS POINT, THE CAL CONSTANTS HAVE CHANGED FROM THE COMBINED CAL CONSTANTS TO
      //                 THE BOARD CAL CONSTANTS.  THE ROGOWSKI CAL FACTORS ARE NOT BEING USED FOR
      //                 GENERATING THE THE METERED CURRENTS!!
      // ADCcalLow.xxx has the cal constants.  The subroutine takes care of whether they are defaults, etc.,
      //   so set ok to True
      ok = TRUE;
    }
    // If the data in FRAM is bad, either set the values to the defined default values or to 0, if there are
    //   no defined defaults.  The new configuration info should be written anyway
    if (!ok)
    {
      if (bufid  == 16)
      {
        for (i = 0; i < length_cat; ++i)
        {
          SPI2_buf[i] = ROGO_DEFAULT_IGAIN;
        }
      }
      else if (bufid == 29)
      {
        set = 0;                            // "set" is used as a temporary
        for (i = 0; i < 3; ++i)
        {
          SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_VGAIN;
          SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_VGAIN >> 8);
        }
        for (i = 0; i < 3; ++i)
        {
          SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_VOFFSET;
          SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_VOFFSET >> 8);
        }
        for (i = 0; i < 6; ++i)
        {
          SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_PHASE;
          SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_PHASE >> 8);
        }
      }
      else
      {
        for (i = 0; i < length_cat; ++i)
        {
          SPI2_buf[i] = 0;
        }
      }
    }
    //----------------- End retrieve the existing info.  Store in SPI2_buf[] -------------------------------

    // "ok" holds a code used to determine whether to Ack or Nak, and whether to perform a FRAM write
    //   ok = 2: Ack and perform a FRAM write
    //   ok = 1: Ack but do not perform a FRAM write
    //   ok = 0: Nak and do not perform a FRAM write
    ok = 2;                             // Initialize message validity to Ack/write

    //  Switch on bufID
    switch (bufid)
    {
      case 0:                           // Frame Rating
        // Save new frame rating
        Break_Config.config.Rating = (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        // Update associated read-only setpoints
        Setpoints0.stp.Rating = Break_Config.config.Rating;
        Setpoints1.stp.Rating = Break_Config.config.Rating;
        // Load SPI2_buf[] with the new frame rating, so it is ready to be written to the PXR25 section of
        //   Frame FRAM
        SPI2_buf[0] = (uint8_t)Break_Config.config.Rating;
        SPI2_buf[1] = (uint8_t)(Break_Config.config.Rating >> 8);
        break;
      case 1:                           // Frame
        // Save new frame type
        Break_Config.config.BreakerFrame = (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        // Update associated read-only setpoints
        Setpoints0.stp.Breakframe = Break_Config.config.BreakerFrame;
        Setpoints1.stp.Breakframe = Break_Config.config.BreakerFrame;
        // Load SPI2_buf[] with the new frame type, so it is ready to be written to the PXR25 section of
        //   Frame FRAM
        SPI2_buf[2] = (uint8_t)Break_Config.config.BreakerFrame;
        SPI2_buf[3] = (uint8_t)(Break_Config.config.BreakerFrame >> 8);
        break;
      case 9:                           // Current Configuration
        // Save new critical config info
        Break_Config.config.Poles = (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        Break_Config.config.Standard = (DPComm.RxMsg[10] + (((uint16_t)DPComm.RxMsg[11]) << 8));
        Break_Config.config.OvrWithstand = (DPComm.RxMsg[30] + (((uint16_t)DPComm.RxMsg[31]) << 8));
        Break_Config.config.MCR = (DPComm.RxMsg[32] + (((uint16_t)DPComm.RxMsg[33]) << 8));
        Break_Config.config.MaxInstTripSetting = (DPComm.RxMsg[40] + (((uint16_t)DPComm.RxMsg[41]) << 8));
        // Write the new config info into SPI2_buf[] so it is ready to write to Frame FRAM
        ContinousDPCommRxtoSPI2BITSHIFT(FacBuf_Info[bufid][1], (FacBuf_Info[bufid][4]));
        // Add in the present Frame Rating and Type
        SPI2_buf[0] = (uint8_t)Break_Config.config.Rating;
        SPI2_buf[1] = (uint8_t)(Break_Config.config.Rating >> 8);
        SPI2_buf[2] = (uint8_t)Break_Config.config.BreakerFrame;
        SPI2_buf[3] = (uint8_t)(Break_Config.config.BreakerFrame >> 8);
        break;
      case 2:                           // CT
      case 3:                           // Parent Catalog Number
      case 4:                           // Parent Serial Number
      case 5:                           // Parent Manufacturing Location
      case 6:                           // Configured Catalog Number
      case 7:                           // Configured Serial Number
      case 8:                           // Configured Manufacturing Location
      case 10:                          // Pole Phase Order
      case 11:                          // Parent Manufacturing Date
      case 12:                          // Configured Manufacturing Date
      case 13:                          // Number of Customer in Reprogramming
      case 14:                          // Security Data (not used)
      // case 15: Rogowoski offset, is invalid
      case 16:                          // Rogowski Gain
      // case 17: Not used, is invalid
      case 23:                          // ETU Serial Number
      case 24:                          // ETU Hardware Version
      case 25:                          // Frame Module Serial Number
      case 26:                          // Frame Module Hardware Version
      case 27:                          // Breaker Health Configuration
      case 28:                          // VDB1 Configuration
      case 29:                          // VDB1 CAL Factors
      case 30:                          // Manufacturing Location and Date
      case 38:                          // VDB2 Configuration
      case 39:                          // VDB2 CAL Factors
        // Note, buffers 31 and 32 (diagnostics) are not supported for write operations (by design)
        //       Not sure about buffers 33 and 34 (health data)  *** DAH CHECK THIS
      // case 31: External Diagnostics
      // case 32: Internal Diagnostics
      // case 33: Customer Breaker Health
      // case 34: Internal Breaker Health
        // Move received message into SPI2_buf[] in preparation for storing
        ContinousDPCommRxtoSPI2BITSHIFT(FacBuf_Info[bufid][1], (FacBuf_Info[bufid][4]));                      
        break;
      case 18:                          // ETU AFE Voltage Cal Constants
        // Save new calibration constants
        for(i = 0; i < 3; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[8+ (i*4)] + (((uint32_t)DPComm.RxMsg[9+ (i*4)]) << 8)
                + (((uint32_t)DPComm.RxMsg[10+(i*4)]) << 16)+ (((uint32_t)DPComm.RxMsg[11+(i*4)]) << 24) );
          // Don't accept invalid gain value
          AFEcal.gain[i+5] = ((flt_uval.fval > 0) ? flt_uval.fval : AFE_CAL_DEFAULT_IGAIN);
          flt_uval.uval = (DPComm.RxMsg[20+ (i*4)] + (((uint32_t)DPComm.RxMsg[21+ (i*4)]) << 8)
              + (((uint32_t)DPComm.RxMsg[22+(i*4)]) << 16)+ (((uint32_t)DPComm.RxMsg[23+(i*4)]) << 24) );
          AFEcal.offset[i+5] = flt_uval.fval;
        }
        // Update the checksum and complement
        AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
        AFEcal.cmp = ~AFEcal.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        // *** DAH   ADD CODE TO COMBINE THE NEW BOARD CAL CONSTANTS WITH THE FRAME CAL CONSTANTS - PROBABLY PUT IN SUBROUTINE AND CALL IT
        // MAYBE ADD CODE TO CLEAR MAXLOOPTIME SINCE THIS IS LIKELY TO TAKE A LONG TIME TO EXECUTE - PROBABLY NEED TO DO THROUGH A FLAG AND CLEAR IN MAIN LOOP
        break;
      case 19:                          // AFE Phase Shift uint16
        // Save new calibration constants
        //   Values are stored as uint8's in the structure (only the LS byte is used)
        for(i = 0; i < 3; i++)
        {
          AFEcal.phase[i+6] = DPComm.RxMsg[8 + (i*2)];      // Ia - Ic, 50Hz
          AFEcal.phase[i] = DPComm.RxMsg[14 + (i*2)];       // Ia - Ic, 60Hz
        }
        // Update the checksum and complement
        AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
        AFEcal.cmp = ~AFEcal.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
      case 20:                          // ADC Voltage Constants floats
        // Save new calibration constants
        for(i = 0; i < 3; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[8+ (i*4)] + (((uint32_t)DPComm.RxMsg[9+ (i*4)]) << 8)
                + (((uint32_t)DPComm.RxMsg[10+(i*4)]) << 16)+ (((uint32_t)DPComm.RxMsg[11+(i*4)]) << 24) );
          ADCcalHigh.gain[i+5] = ((flt_uval.fval > 0) ? flt_uval.fval : ADC_CAL_DEFAULT_VGAIN);
          ADCcalLow.gain[i+5] = ADCcalHigh.gain[i+5];
          flt_uval.uval = (DPComm.RxMsg[20+ (i*4)] + (((uint32_t)DPComm.RxMsg[21+ (i*4)]) << 8)
              + (((uint32_t)DPComm.RxMsg[22+(i*4)]) << 16)+ (((uint32_t)DPComm.RxMsg[23+(i*4)]) << 24) );
          ADCcalHigh.offset[i+5] = flt_uval.fval;
          ADCcalLow.offset[i+5] = ADCcalHigh.offset[i+5];
        }
        // Update the checksum and checksum complement
        ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        ADCcalHigh.cmp = ~ADCcalHigh.chk;
        ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        ADCcalLow.cmp = ~ADCcalLow.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));
        FRAM_Write(DEV_FRAM2, FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
      case 21:                          // SG Constants
        // Save new source ground calibration constants
        // CT Gain
        flt_uval.uval = ( DPComm.RxMsg[8] + (((uint32_t)DPComm.RxMsg[9]) << 8)
              + (((uint32_t)DPComm.RxMsg[10]) << 16) + (((uint32_t)DPComm.RxMsg[11]) << 24) ); 
        AFEcal.gain[4] = ((flt_uval.fval > 0) ? flt_uval.fval : AFE_CAL_CT_IGAIN);
        // CT Offset
        flt_uval.uval = ( DPComm.RxMsg[12] + (((uint32_t)DPComm.RxMsg[13]) << 8)
              + (((uint32_t)DPComm.RxMsg[14]) << 16) + (((uint32_t)DPComm.RxMsg[15]) << 24) ); 
        AFEcal.offset[4] = flt_uval.fval; 
        // Rogowski Gain
        flt_uval.uval = ( DPComm.RxMsg[16] + (((uint32_t)DPComm.RxMsg[17]) << 8)
              + (((uint32_t)DPComm.RxMsg[18]) << 16) + (((uint32_t)DPComm.RxMsg[19]) << 24) ); 
        AFEcal.gain[8] = ((flt_uval.fval > 0) ? flt_uval.fval : AFE_CAL_DEFAULT_IGAIN);
        // Rogowski Offset
        flt_uval.uval = ( DPComm.RxMsg[20] + (((uint32_t)DPComm.RxMsg[21]) << 8)
              + (((uint32_t)DPComm.RxMsg[22]) << 16) + (((uint32_t)DPComm.RxMsg[23]) << 24) ); 
        AFEcal.offset[8] = flt_uval.fval; 
        // Update the checksum and complement
        AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
        AFEcal.cmp = ~AFEcal.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
      case 35:                          // AFE Current Calibration Constants
        // Move received message into the calibration constants so we can write them to FRAM and Flash
        for(i =0; i < 4; i++)               // Ia - In gains
        {
          flt_uval.uval = (DPComm.RxMsg[8+(i*4)] + (((uint16_t)DPComm.RxMsg[9+(i*4)]) << 8)
                + ((uint32_t)(DPComm.RxMsg[10+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[11+(i*4)] << 24)) );
          AFEcal.gain[i] = ((flt_uval.fval > 0) ? flt_uval.fval : AFE_CAL_DEFAULT_IGAIN);
        }
                                            // In CT gain
        flt_uval.uval = (DPComm.RxMsg[24] + (((uint16_t)DPComm.RxMsg[25]) << 8)
                + ((uint32_t)(DPComm.RxMsg[26] << 16)) + ((uint32_t)(DPComm.RxMsg[27] << 24)) );
        AFEcal.gain[9] = ((flt_uval.fval > 0) ? flt_uval.fval : AFE_CAL_CT_IGAIN);
        for(i =0; i < 4; i++)               // Ia - In Rogo offsets
        {
          flt_uval.uval = (DPComm.RxMsg[28+(i*4)] + (((uint16_t)DPComm.RxMsg[29+(i*4)]) << 8)
                + ((uint32_t)(DPComm.RxMsg[30+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[31+(i*4)] << 24)) );
          AFEcal.offset[i] = flt_uval.fval;
        }                                   // In CT offset
        flt_uval.uval = (DPComm.RxMsg[44] + (((uint16_t)DPComm.RxMsg[45]) << 8)
                + ((uint32_t)(DPComm.RxMsg[46] << 16))+ ((int)(DPComm.RxMsg[47] << 24)) );
        AFEcal.offset[9] = flt_uval.fval;
        // IMPORTANT NOTE: AT THIS POINT, THE COMBINED CAL CONSTANTS (BOARD + ROGOWSKI) ARE STILL NOT BEING
        //                 USED.  ONLY THE NEWLY-WRITTEN CAL BOARD CAL CONSTANTS AND THE EXISTING BOARD
        //                 CONSTANTS (FOR THE ONES THAT WERE NOT WRITTEN) ARE BEING USED TO GENERATE THE
        //                 METERED CURRENTS!!  THE ROGOWSKI CAL FACTORS ARE NOT BEING USED FOR GENERATING
        //                 THE METERED CURRENTS!!
        // Compute and store the new checksums
        AFEcal.chk = ComputeChksum32((uint32_t *)(&AFEcal.gain[0]), ((AFE_CAL_SIZE >> 2)-2));
        AFEcal.cmp = ~AFEcal.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_AFECAL, (AFE_CAL_SIZE >> 1), (uint16_t *)(&AFEcal.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
      case 36:                          // Low Gain ADC Current Calibration Constants
        // Move received message into the calibration constants so we can write them to FRAM and Flash
        for(i =0; i < 4; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[8+(i*4)] + (((uint16_t)DPComm.RxMsg[9+(i*4)]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[10+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[11+(i*4)] << 24)) );
          ADCcalLow.gain[i] = ((flt_uval.fval > 0) ? flt_uval.fval : ADC_CAL_DEFAULT_IGAIN_LOW);
        }
        flt_uval.uval = (DPComm.RxMsg[24] + (((uint16_t)DPComm.RxMsg[25]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[26] << 16))+ ((uint32_t)(DPComm.RxMsg[27] << 24)) );
        ADCcalLow.gain[4] = ((flt_uval.fval > 0) ? flt_uval.fval : ADC_CAL_CT_IGAIN_LOW);
        for(i =0; i < 4; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[28+(i*4)] + (((uint16_t)DPComm.RxMsg[29+(i*4)]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[30+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[31+(i*4)] << 24)) );
          ADCcalLow.offset[i] = flt_uval.fval;
        }
        flt_uval.uval = (DPComm.RxMsg[44] + (((uint16_t)DPComm.RxMsg[45]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[46] << 16))+ ((uint32_t)(DPComm.RxMsg[47] << 24)) );
        ADCcalLow.offset[4] = flt_uval.fval;
        // IMPORTANT NOTE: AT THIS POINT, THE COMBINED CAL CONSTANTS (BOARD + ROGOWSKI) ARE STILL NOT BEING
        //                 USED.  ONLY THE NEWLY-WRITTEN CAL BOARD CAL CONSTANTS AND THE EXISTING BOARD
        //                 CONSTANTS (FOR THE ONES THAT WERE NOT WRITTEN) ARE BEING USED TO GENERATE THE
        //                 METERED CURRENTS!!  THE ROGOWSKI CAL FACTORS ARE NOT BEING USED FOR GENERATING
        //                 THE METERED CURRENTS!!
        // Compute and store the new checksums
        ADCcalLow.chk = ComputeChksum32((uint32_t *)(&ADCcalLow.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        ADCcalLow.cmp = ~ADCcalLow.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_ADCCALL, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalLow.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
      case 37:                          // High Gain ADC Current Calibration Constants
        // Move received message into the calibration constants so we can write them to FRAM and Flash
        for(i =0; i < 4; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[8+(i*4)] + (((uint16_t)DPComm.RxMsg[9+(i*4)]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[10+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[11+(i*4)] << 24)) );
          ADCcalHigh.gain[i] = ((flt_uval.fval > 0) ? flt_uval.fval : ADC_CAL_DEFAULT_IGAIN_HIGH);
        }
        flt_uval.uval = (DPComm.RxMsg[24] + (((uint16_t)DPComm.RxMsg[25]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[26] << 16))+ ((uint32_t)(DPComm.RxMsg[27] << 24)) );
        ADCcalHigh.gain[4] = ((flt_uval.fval > 0) ? flt_uval.fval : ADC_CAL_CT_IGAIN_HIGH);
        for(i =0; i < 4; i++)
        {
          flt_uval.uval = (DPComm.RxMsg[28+(i*4)] + (((uint16_t)DPComm.RxMsg[29+(i*4)]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[30+(i*4)] << 16))+ ((uint32_t)(DPComm.RxMsg[31+(i*4)] << 24)) );
          ADCcalHigh.offset[i] = flt_uval.fval;
        }
        flt_uval.uval = (DPComm.RxMsg[44] + (((uint16_t)DPComm.RxMsg[45]) << 8)
                    + ((uint32_t)(DPComm.RxMsg[46] << 16))+ ((int)(DPComm.RxMsg[47] << 24)) );
        ADCcalHigh.offset[4] = flt_uval.fval;
        // Compute and store the new checksums
        ADCcalHigh.chk = ComputeChksum32((uint32_t *)(&ADCcalHigh.gain[0]), ((ADC_CAL_SIZE >> 2)-2));
        ADCcalHigh.cmp = ~ADCcalHigh.chk;
        // Write the new calibration constants into FRAM and Flash
        // Set request to write the cal constants to Flash and wait for it to finish.  If we are messing
        //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
        //   demand logging, so we should be serviced almost immediately
        SPI1Flash.Req |= S1F_CAL_WR;          // Set flag to write the cal constants to Flash
        // Write the new calibration constants into FRAM
        FRAM_Stat_Write(DEV_FRAM2, 0x80);     // Remove upper block write protection
        FRAM_Write(DEV_FRAM2, FRAM_ADCCALH, (ADC_CAL_SIZE >> 1), (uint16_t *)(&ADCcalHigh.gain[0]));
        FRAM_Stat_Write(DEV_FRAM2, 0x84);     // Restore block write protection
        while ((SPI1Flash.Ack & S1F_CAL_WR) != S1F_CAL_WR)      // Wait for Flash write to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_CAL_WR);
        ok = 1;                               // Set code to Ack/No write
        break;
//      case 40:                          // Neutral CT Cal Constants - Not supported
      case 43:                          // Standard
        // Save new Standard, for use in other parts of code
        Break_Config.config.Standard = (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        SPI2_buf[0] = ((uint8_t)Break_Config.config.Standard);
        SPI2_buf[1] = ((uint8_t)(Break_Config.config.Standard >> 8));
        break;
      case 44:                          // Max Instantaneous Trip Setting
        // Save new MaxInstTripSetting, for use in other parts of code
        Break_Config.config.MaxInstTripSetting = (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        SPI2_buf[0] = ((uint8_t)Break_Config.config.MaxInstTripSetting);
        SPI2_buf[1] = ((uint8_t)(Break_Config.config.MaxInstTripSetting >> 8));
        break;
      default:                          // Invalid buffer number
         ok = 0;
         break;
    }

    // Now write if buffer is supported
    if (ok == 2)
    {
      // this calculates the Checksum of the whole category
      EndOfDataCAT = length_cat-4; //index of the last data value in category
      sum = Checksum8_16((uint8_t *)(&SPI2_buf[0]), EndOfDataCAT);
      SPI2_buf[EndOfDataCAT] = ((uint8_t)sum);
      SPI2_buf[EndOfDataCAT+1] = ((uint8_t)(sum >> 8)); //writes checksum
      sum = (sum ^ 0xFFFF);
      SPI2_buf[EndOfDataCAT+2] = ((uint8_t)sum);
      SPI2_buf[EndOfDataCAT+3] = ((uint8_t)(sum >> 8)); //writes checksum NOT
      set = ( (bufid == 27) ? 1 : 3);       // BID = 27 has only one copy; the rest have 3 copies
      for (i = 0; i < set; ++i)             //   "set" is used as a temporary
      {
        address = (first_addressofCAT + (i * (length_cat+4)));
        // 0x10000 offsets are to make the FRAM protection addresses smaller in the table
        // Note, for buffers 0, 1, and 9, Frame_FRAM_Write() places the data into the PXR25 critical
        //   breaker information section.  They have already been written to the PXR35 section
        if(FacBuf_Info[bufid][2] == 0)
        {
          FRAM_Write(DEV_FRAM2, address +0x10000, length_cat, (uint16_t*)&SPI2_buf[0]);
        }
        else
        {
          Frame_FRAM_Write(address, length_cat, &SPI2_buf[0]);
        }
      }
      Gen_Values();                       // Called in case the configuration changed
      // If buffer is 0, 1, 9, 43, or 44 we need to write the values into the PXR35 critical breaker config
      //   locations in Frame FRAM
      if ( (bufid == 0) || (bufid == 1) || (bufid == 9) || (bufid == 43) || (bufid == 44) )
      {
        Save_Critical_BrkConfig();
      }
      DPComm.AckNak = DP_ACK;                           // Ack since command is valid
    }
    else if (ok == 1)
    {
      DPComm.AckNak = DP_ACK;                           // Ack since command is valid
    }
    else                                                // Buffer not supported - Nak
    {
      DPComm.AckNak = DP_NAK_BUFINVALID;
    }                 
  }
  else                                                  // Length is wrong - Nak
  {
    DPComm.AckNak = DP_NAK_CMDFORMAT;
  }                 

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ProcWrFactoryConfig()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcExActWAck()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Execute Action With Acknowledge Commands Subroutine
//
//  MECHANICS:          This subroutine handles an Execute Action With Acknowledge command.
//                      It
//                          - checks the action type and ID to determine the specific command
//                          - performs the appropriate action
//                          - assembles the appropriate response (ack or nak) into DPComm.TxBuf[]
//
//  CAVEATS:            None
//
//  INPUTS:             actionid, actiontype
//                      DPComm.RxMsg[]
// 
//  OUTPUTS:            SetpActiveSet, Prot_Enabled, SetpChkGrp, ResetMinMaxFlags, IntSyncTime.xx,
//                      SysTickTime.xx, SysTick->LOAD, SysTick->VAL, Manufacture_Mode
//
//  ALTERS:             DPComm.AckNak
//
//  CALLS:              Frame_FRAM_Write(), Load_SetpGr0_Gr1(), Gen_Values(), Load_SetpGr2_LastGr(),
//                      Get_InternalTime(), ProcessTimeAdjustment(), CaptureUserWaveform(),
//                      Brk_Config_DefaultInit()
//
//------------------------------------------------------------------------------------------------------------

void ProcExActWAck(uint16_t actionid, uint8_t actiontype)
{
  uint32_t newEID;
  uint16_t i;
  uint16_t mm_setting_updated, sum;
  union FLOAT_UINT32
  {
    float fval;
    uint32_t uval;
  } temp;
  struct INTERNAL_TIME temptime1, oldtime;
  uint8_t tmp;

  // The message has already been checked for address, CRC, and command validity.
  //   The Execute Action Command is stored as follows:
  //        DPComm.RxMsg[0]- Command = x09
  //        DPComm.RxMsg[1]- Source/Destination Address
  //        DPComm.RxMsg[2]- Sequence Number
  //        DPComm.RxMsg[3]- Action Type
  //        DPComm.RxMsg[4]- Action ID least significant byte
  //        DPComm.RxMsg[5]- Action ID most significant byte
  //        DPComm.RxMsg[6]- Action Length N least significant byte
  //        DPComm.RxMsg[7]- Action Length N most significant byte
  //        DPComm.RxMsg[8]- Action Data
  //        DPComm.RxMsg[9]- Action Data
  //          ..........................
  //        DPComm.RxMsg[N]- Action Data

  // Process the command according to the action type and ID
  if ((actiontype == DP_EATYPE_SETP) && (actionid == 0x0002))       // Type = 0, ID = 2: Change setp set
  {
    // Check length (should be 2) and data format (set is 0..3), complement in high byte
    if ( (DPComm.RxMsg[7] == 0) && (DPComm.RxMsg[6] == 2)
      && (DPComm.RxMsg[8] < 4) && ((DPComm.RxMsg[8] + DPComm.RxMsg[9]) == 0xFF) )
    {
      SetpActiveSet = DPComm.RxMsg[8];                          // Change the active set
      // Write it to Frame FRAM (active set plus complement
      i = (((uint16_t)SetpActiveSet ^ 0x00FF) << 8) + SetpActiveSet;
      Frame_FRAM_Write(SETP_ACTIVE_SET_ADDR, 2, (uint8_t *)(&i));
      // Load up the new Group 0 and Group 1 setpoints.  These are used for current
      //   protection.  Disable protection while the new setpoints are retrieved and the new
      //   protection parameters are generated, because SD, GF, and firmware INST protection
      //   are done in the sampling interrupt.  We don't want to disable interrupts because we
      //   might miss a sample or two.
      Prot_Enabled = FALSE;
      Load_SetpGr0_Gr1();
      Gen_Values();
      // Reset the setpoints group to check because we have cleared the status
      //   (SetpointsStat) and the only two statuses that are valid are Gr0 and Gr1.  The
      //   subroutine is called every 5 minutes, so we will have the other groups loaded
      //   before they are checked.
      SetpChkGrp = 0;
      Prot_Enabled = TRUE;
      // Enter a setpoints download event
      tmp = DPComm.RxMsg[1] >> 4;            // Compute the event code based on the source address
      if (tmp == DP_DISP_ADD)
      {
        tmp = STP_SET_CHANGE_DISPLAY;
      }
      else if (tmp == DP_USB_ADD)
      {
        tmp = STP_SET_CHANGE_USB;
      }
      else if (tmp == DP_BT_ADD)
      {
        tmp = STP_SET_CHANGE_BLUETOOTH;
      }
      else                                 // Must be Modbus TCP
      {
        tmp = STP_SET_CHANGE_MODBUS_TCP;
      }
      newEID = InsertNewEvent(tmp);
      // If the demand setpoints changed, we need to restart logging with the new variables
      if ( (Dmnd_Setp_DemandWindow != (uint8_t)Setpoints0.stp.DemandWindow)
        || (Dmnd_Setp_DemandInterval != (uint8_t)Setpoints0.stp.DemandInterval)
        || (Dmnd_Setp_DemandLogInterval = (uint8_t)Setpoints0.stp.DemandLogInterval) )
      {
        Dmnd_VarInit();
        EngyDmnd[1].EID = newEID;
      }
      // Retrieve the new setpoints for the remaining groups.  Save the retrieval status (OK = 0)
      i = Load_SetpGr2_LastGr();
      // Enter an event if Frame mismatch or frame bad
      if (i == 1)                     // Status = 1: Frame Mismatch
      {
        NewEventFIFO[NewEventInNdx].Code = STP_FRAME_MISMATCH;
        Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
        NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
        NewEventInNdx &= 0x0F;              // Note, FIFO size must be 16!!
      }
      else if (i == 2)                // Status = 2: Frame bad
      {
        NewEventFIFO[NewEventInNdx].Code = STP_ERROR;
        Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
        NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
        NewEventInNdx &= 0x0F;              // Note, FIFO size must be 16!!
      }
      DPComm.AckNak = DP_ACK;         // Ack since command is valid
    }
    else                                                        // Length is wrong - Nak
    {
      DPComm.AckNak = DP_NAK_CMDFORMAT;
    }
  }

  else if (actiontype == DP_EATYPE_RESET)                           // Type = 1: Reset Values
  {
    ResetMinMaxFlags |=  (((uint32_t)1) << actionid);
    DPComm.AckNak = DP_ACK;                                     // Ack since command is valid
  }

  else if (actiontype == DP_EATYPE_TU)
  {
    switch (actionid)
    {
      case 0: //Open Breaker
      break;
      case 1: //Close Breaker
      break;
      case 2: //Enable MM
        // b08: MM state
        // b07: MM local ON
        // b06: MM local setting invalid
        // b01: MM signal from secondary pin (not used in PXR35, combined with b07)
        // b00: MM remote com channel
        if ((Setpoints0.stp.MM_Enable & BIT0) == 0)
        {
          // if MM remote com channel status is off, turn it on and ensure MM state is ON
          Setpoints0.stp.MM_Enable |= (BIT0 | BIT8);
          mm_setting_updated = TRUE;
        }
      break;
      case 3: //Disable MM (clear remote comm chan
        // if MM remote com channel status is on, turn it off
        if ((Setpoints0.stp.MM_Enable & BIT0) != 0)
        {
          // if MM local is also off, ensure all MM bits are cleared
          if ((Setpoints0.stp.MM_Enable & BIT7) == 0)
          {
            Setpoints0.stp.MM_Enable = 0;
          }
          else // otherwise just clear MM remote com channel
          {
            Setpoints0.stp.MM_Enable &= ~BIT0;
          }
          mm_setting_updated = TRUE;
        }
      break;
      default:
      break;
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
    DPComm.AckNak = DP_ACK;
  }

  else if ((actiontype == DP_EATYPE_CAP) && (actionid <= 12))       // Type = 3: User waveform capture
  {
    if (actionid == 12)                                 // ID = 12: free captured waveform
    {
      UserWF.Locked = FALSE;
    }
    else if (CaptureUserWaveform(actionid))             // Call routine to do the capture.  True is returned
    {                                                   //   if successful
       DPComm.AckNak = DP_ACK;
    }
    else                                                // If no capture, send NAK
    {
      DPComm.AckNak = DP_NAK_NOTSTATE;         // Set NAK status to Not in Correct State
    }
  }

  else if ((actiontype == DP_EATYPE_TIME) && (actionid == 0x0001))  // If Type = 5 and ID = 1, set real time
  {
    // Save the display processor's sync toggle time
    temptime1.Time_secs = (uint32_t)DPComm.RxMsg[8] + (((uint32_t)DPComm.RxMsg[9]) << 8)
           + (((uint32_t)DPComm.RxMsg[10]) << 16) + (((uint32_t)DPComm.RxMsg[11]) << 24);
    temptime1.Time_nsec = (uint32_t)DPComm.RxMsg[12] + (((uint32_t)DPComm.RxMsg[13]) << 8)
           + (((uint32_t)DPComm.RxMsg[14]) << 16) + (((uint32_t)DPComm.RxMsg[15]) << 24);
    // If we have captured a sync time, adjust our present time by the difference between the
    //   two times.  We have captured a sync time if it is not zero
    if ( (IntSyncTime.Time_secs != 0) || (IntSyncTime.Time_nsec != 0) )
    {
      // Compute the difference between the transmitted disp proc time and the captured sync time.
      //   Save this difference in IntSyncTime
      if ( (temptime1.Time_secs > IntSyncTime.Time_secs)         // Disp time > Int time
        || ((temptime1.Time_secs == IntSyncTime.Time_secs)
        && (temptime1.Time_nsec >= IntSyncTime.Time_nsec)) )
      {
        if (IntSyncTime.Time_nsec > temptime1.Time_nsec)         // If need to borrow
        {
          IntSyncTime.Time_nsec = temptime1.Time_nsec + (1000000000 - IntSyncTime.Time_nsec);
          IntSyncTime.Time_secs = (temptime1.Time_secs - 1)- IntSyncTime.Time_secs;
        }
        else
        {
          IntSyncTime.Time_nsec = temptime1.Time_nsec - IntSyncTime.Time_nsec;
          IntSyncTime.Time_secs = temptime1.Time_secs - IntSyncTime.Time_secs;
        }
        i = TRUE;             // i = temporary flag to denote positive adjustment
      }
      else                                                       // Int time > Disp time
      {
        if (temptime1.Time_nsec > IntSyncTime.Time_nsec)         // If need to borrow
        {
          IntSyncTime.Time_nsec = IntSyncTime.Time_nsec + (1000000000 - temptime1.Time_nsec);
          IntSyncTime.Time_secs = (IntSyncTime.Time_secs - 1)- temptime1.Time_secs;
        }
        else
        {
          IntSyncTime.Time_nsec = IntSyncTime.Time_nsec - temptime1.Time_nsec;
          IntSyncTime.Time_secs = IntSyncTime.Time_secs - temptime1.Time_secs;
        }
        i = FALSE;            // i = temporary flag to denote negative adjustment
      }
      // Adjust internal time by this difference.  The new internal time will be temporarily
      //   held in temptime1
      __disable_irq();
        // Capture the present time - save it in oldtime.  This subroutine takes about 775nsec
        Get_InternalTime(&oldtime);
      // This section of code takes about 215nsec, so we should add 990nsec to the adjustment
      //   amount
      if (i)                              // Positive adjustment
      {
        temptime1.Time_nsec = oldtime.Time_nsec + IntSyncTime.Time_nsec + 990;
        temptime1.Time_secs = oldtime.Time_secs + IntSyncTime.Time_secs;
        if (temptime1.Time_nsec >= 1000000000)
        {
          ++temptime1.Time_secs;
          temptime1.Time_nsec -= 1000000000;
        }
      }
      else                                    // Negative adjustment
      {
        if ((oldtime.Time_nsec + 290) >= IntSyncTime.Time_nsec)
        {
          temptime1.Time_nsec = (oldtime.Time_nsec + 290) - IntSyncTime.Time_nsec;
          temptime1.Time_secs = oldtime.Time_secs - IntSyncTime.Time_secs;
        }
        else
        {
          temptime1.Time_nsec = ((1000000000 - IntSyncTime.Time_nsec)
                                          + oldtime.Time_nsec + 990);
          temptime1.Time_secs = oldtime.Time_secs - (IntSyncTime.Time_secs + 1);
        }
      }
      // We now have the new time in seconds, nsec.  We have to convert it to SysTick time
      // Internal time is kept in structure SysTickTime.  SysTickTime consists of three
      // components:
      //   cnt_sec     Seconds past January 1, 2000
      //   cnt_10msec  10msec counts past the last second, resets at one second (100)
      //   SysTick->VAL indicates the number of 66.666667nsec timer ticks past the last 10msec
      //     tick.  This may be used to provide a timestamp with ~67nsec resolution.
      //     Note, SysTick->VAL is a downcounter, so the number of ticks past the last 10msec
      //     tick is actually the initial value (COUNT_120MHZ_10MSEC) minus SysTick->VAL.
      SysTickTime.cnt_sec = temptime1.Time_secs;
      SysTickTime.cnt_10msec = temptime1.Time_nsec/10000000;
      // We need to load the SysTick timer value with the count corresponding to the remaining
      //   nanoseconds.  Each SysTick count is equal to 66.67nsec = (200/3)nsec
      temp.uval = temptime1.Time_nsec % 10000000;   // Remaining nsec (< 10msec) = 9999999 max
      // The SysTick timer is a down-counter, so the count to load into the timer is the number
      //   of counts for 10msec (COUNT_120MHZ_10MSEC) minus the number of counts for the
      //   remaining nsec (temp.uval/66.67)
      temp.uval = COUNT_120MHZ_10MSEC - ((temp.uval * 3)/200);
      SysTick->LOAD  = temp.uval;              // Initialize reload value
      SysTick->VAL   = temp.uval;              // Initialize present value
      i = 10;                                  // Wait for new count to load
      while (i > 0)                            // Loading the new value does not cause an
      {                                        //   an interrupt
        --i;
      }
      SysTick->LOAD  = COUNT_120MHZ_10MSEC;    // Reinitialize reload value
      __enable_irq();
      // Call subroutine to process the time adjustment
      ProcessTimeAdjustment(&oldtime, &temptime1);
      IntSyncTime.Time_secs = 0;               // Reset the captured sync time 
      IntSyncTime.Time_nsec = 0;
    }
    else                                       // If no sync time captured, NAK
    {
      DPComm.AckNak = DP_NAK_NOTSTATE;         // Set NAK status to Not in Correct State
    }
  }

  // Others
  else if (actiontype == DP_EATYPE_OTHERS)
  {
    DPComm.AckNak = DP_ACK;
    switch(actionid)
    {
      case 0:
        // Coil open detection start
        CoilDetect.Enable = TRUE;
        CoilDetect.TestInProgress = 1;
        break;        
      case 1:
        switch (DPComm.RxMsg[9])
        {
          case 1:
            if(DPComm.RxMsg[8]==1)
            {
              RemoteCtrlFlag1 = 1;
            }
            else 
            {
              RemoteCtrlFlag1 = 0;
            }
          break;
          
          case 2:
            if(DPComm.RxMsg[8]==1)
            {
              RemoteCtrlFlag2 = 1;
            }
            else 
            {
              RemoteCtrlFlag2 = 0;
            }
          break;

          case 3:
            if(DPComm.RxMsg[8]==1)
            {
              RemoteCtrlFlag3 = 1;
            }
            else 
            {
              RemoteCtrlFlag3 = 0;
            }
          break;

        }
        break;
  
      case 2: // Thermal memory reset
        Reset_ThermalMemory(DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));    
        break;
      default:
        DPComm.AckNak = DP_NAK_BUFINVALID;
        break;
        
      
    }
  }
  
  // Password
  else if (actiontype == DP_EATYPE_PWD)
  {
    FRAM_Read(PSWD_ADDRESS, PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]));
    sum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
    if ( (sum != Password.data.Cksum) || (sum != (Password.data.Cksum_not ^ 0xFFFF)) )
    {
      for(i = 0; i < PASSWORD_SIZE-5; i++)
      {
        Password.buf[i] = 0;
      }
    }
    uint16_t rcvd_pswd = DPComm.RxMsg[8] * 1000 + DPComm.RxMsg[9] * 100 +
                         DPComm.RxMsg[10] * 10 + DPComm.RxMsg[11];
    uint16_t admin_pswd = Password.data.admin_num[0] * 1000 + Password.data.admin_num[1] * 100 +
                          Password.data.admin_num[2] * 10 + Password.data.admin_num[3];
    uint16_t user_pswd = Password.data.user_num[0] * 1000 + Password.data.user_num[1] * 100 +
                         Password.data.user_num[2] * 10 + Password.data.user_num[3];
    uint8_t user_exists = Password.data.user_info & BIT0;
    
    DPComm.AckNak = DP_ACK;
      
    switch (actionid)
    {      
      case 0x00:  //Enter Password - special command, new Admin password entered via LCD without previous password verificaion
      case 0x02:  //Set Password - special command, new Admin password entered via USB withou previous password verification
      {
        for(i = 0; i<4; i++)
        {
          Password.data.admin_num[i] = DPComm.RxMsg[8 + i];
        }
        Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
        Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
        FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );  

      }
      break;
      case 0x01:  //Verify Admin Password 
      {
        if(Pswd_Rejection_Tmr)
        {
          NewEventFIFO[NewEventInNdx].Code = WRONG_PWD_ENTERED;
          DPComm.AckNak = DP_NAK_BUFINVALID; 
        }
        //Correct password received
        else if(rcvd_pswd == admin_pswd)
        {
          NewEventFIFO[NewEventInNdx].Code = ADMIN_PWD_ENTERED;
          Admin_Verified_Tmr = COUNT_10_MIN; 
          Pswd_attempt = 0;         
        }
        else
        {
          NewEventFIFO[NewEventInNdx].Code = WRONG_PWD_ENTERED;
          DPComm.AckNak = DP_NAK_BUFINVALID;           
          Admin_Verified_Tmr = 0;  
          Pswd_attempt++; 
          if(Pswd_attempt >= 3)
          {
            Pswd_Rejection_Tmr = COUNT_10_MIN; 
            User_Verified_Tmr = 0;
          }                                                        
        }
        Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
        NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
        NewEventInNdx &= 0x0F;                                      // Note, FIFO size must be 16!! 
      }
      break;
      case 0x03:  //Create User
        if(Admin_Verified_Tmr && user_exists == FALSE && rcvd_pswd != admin_pswd)
        {
          Password.data.user_info |= BIT0;
          Password.data.user_info &= ~BIT1;
          for(i = 0; i<4; i++)
          {
            Password.data.user_num[i] = DPComm.RxMsg[8 + i];
          }
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      case 0x04:  //Delete User
        if(Admin_Verified_Tmr && user_exists)
        {
          Password.data.user_info &= ~BIT0;
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      case 0x05:  //Change User Password
        if(Admin_Verified_Tmr && user_exists && (rcvd_pswd != admin_pswd))
        {
          for(i = 0; i<4; i++)
          {
            Password.data.user_num[i] = DPComm.RxMsg[8 + i];
          }
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );

          NewEventFIFO[NewEventInNdx].Code = USER_PWD_CHANGED;
          Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
          NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
          NewEventInNdx &= 0x0F;                                      // Note, FIFO size must be 16!! 
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      case 0x06:  //User Permission Extend
        if(Admin_Verified_Tmr && user_exists)
        {
          Password.data.user_info |= BIT1; 
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      case 0x07:  //User Permission Restrict
        if(Admin_Verified_Tmr && user_exists)
        {
          Password.data.user_info &= ~BIT1; 
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      case 0x08:  //Verify User Password
      {
        if(Pswd_Rejection_Tmr)
        {
          NewEventFIFO[NewEventInNdx].Code = WRONG_PWD_ENTERED;
          DPComm.AckNak = DP_NAK_BUFINVALID; 
        }
        //Correct password received and User Exists
        else if(rcvd_pswd == user_pswd && user_exists)
        {
          NewEventFIFO[NewEventInNdx].Code = USER_PWD_ENTERED;
          User_Verified_Tmr = COUNT_10_MIN; 
          Pswd_attempt = 0;         
        }
        else
        {
          NewEventFIFO[NewEventInNdx].Code = WRONG_PWD_ENTERED;
          DPComm.AckNak = DP_NAK_BUFINVALID;           
          User_Verified_Tmr = 0;  
          Pswd_attempt++; 
          if(Pswd_attempt >= 3)
          {
            Pswd_Rejection_Tmr = COUNT_10_MIN; 
            Admin_Verified_Tmr = 0;
          }                                                        
        }
        Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
        NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
        NewEventInNdx &= 0x0F;                                      // Note, FIFO size must be 16!! 
      }
      break;
      case 0x09:  //Change Admin Password
        if(Admin_Verified_Tmr && (((rcvd_pswd != user_pswd) && user_exists) || user_exists == FALSE))
        {
          for(i = 0; i<4; i++)
          {
            Password.data.admin_num[i] = DPComm.RxMsg[8 + i];
          }
          Password.data.Cksum = Checksum8_16((uint8_t *)(&Password.buf[0]), (PASSWORD_SIZE - 4));
          Password.data.Cksum_not = (Password.data.Cksum ^ 0xFFFF);
          FRAM_Write( DEV_FRAM2, PSWD_ADDRESS , PASSWORD_SIZE>>1, (uint16_t *)(&Password.buf[0]) );

          NewEventFIFO[NewEventInNdx].Code = ADMIN_PWD_CHANGED;
          Get_InternalTime(&NewEventFIFO[NewEventInNdx].TS);
          NewEventFIFO[NewEventInNdx++].EID = EventMasterEID++;
          NewEventInNdx &= 0x0F;                                      // Note, FIFO size must be 16!! 
        }
        else
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
        }
      break;
      default:
        DPComm.AckNak = DP_NAK_BUFINVALID;
      break;
        
    }
  }

  else if (actiontype == DP_EATYPE_FACTORY)                         // Type = 13: Factory
  {
    switch(actionid)
    {
      case 28: //AUX5VPOWER
        if(DPComm.RxMsg[6] == 1)//check length
        {
          if(DPComm.RxMsg[8] == 0)
          {
            DISPLAY_DISABLE;
            displayOFFTIM = 25;
            MODBUS_PWR_DISABLE;
          }
          else if(DPComm.RxMsg[8] == 1)
          {
            MODBUS_PWR_DISABLE;
            DISPLAY_ENABLE;
          }
          else if(DPComm.RxMsg[8] == 2)
          {
            MODBUS_PWR_ENABLE;
            DISPLAY_DISABLE;
            displayOFFTIM = 25;
          }
          else if(DPComm.RxMsg[8] == 3)
          {
            DISPLAY_ENABLE;
            MODBUS_PWR_ENABLE;                       
          }
          else
            DPComm.AckNak = DP_NAK_BUFINVALID;
        }
        else
          DPComm.AckNak = DP_NAK_BUFINVALID;
          break;
      case 29:  //ADC Selection
        if(DPComm.RxMsg[8] == 0)  //ADC Lo-Gain
        {
          UseADCvals = TRUE;
          LOW_GAIN_INPUTS;            
        }
        if(DPComm.RxMsg[8] == 1)  //ADC Hi-Gain
        {
          UseADCvals = TRUE;
          HIGH_GAIN_INPUTS;            
        }
        if(DPComm.RxMsg[8] == 2)  // Normal Operation - AFE
        {
          UseADCvals = FALSE;          
        }
        break;
      case 3:   //Recover Calibration
        Write_Default_Cal(DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8));
        break;
      case 23:  //Enter Manufacture Mode
        Manufacture_Mode = TRUE;
        break;
      case 24:  //Exit Manufacture Mode
        Manufacture_Mode = FALSE;
        break;
      case 25:  //Factory Default Reset
        ClearEvents();
        ResetEnergy();
        ResetMinMaxFlags |= RESET_MINMAX_ALL_FLAG;
        Clear_Diag();
        Write_FrameFRAM_Default_Section(HEALTHDATA_FADDR, FRAME_FRAM_HEALTH_DAT_SIZE, 1);
        Write_FrameFRAM_Default_Section(INTERNAL_HEALTHDATA_FADDR, FRAME_FRAM_HEALTH_DAT_SIZE, 1);
        Stp_to_Default();
        //TODO: too keep it consistant with PXR25 ADD Reset Power Demand,
        // Reset Current Demand
        break;
      case 26:  //Clear ETU
        Brk_Config_DefaultInit();
        Write_FrameFRAM_Default_Section(FRAMECONFIG_FADDR, FRAMEMODULECONFIG_BLOCK_SIZE, 3);
        Write_FrameFRAM_Default_Section(VDBCONFIG_LINE_FADDR, VDB_CONFIG_BLOCK_SIZE, 3);
        Write_FrameFRAM_Default_Section(VDBCONFIG_LOAD_FADDR, VDB_CONFIG_BLOCK_SIZE, 3);
        Write_DefaultBrkHealthConfig();
        Write_Default_Cal(0xFFFF);  // All calibrations

        Write_FrameFRAM_Default_Section(HEALTHDATA_FADDR, FRAME_FRAM_HEALTH_DAT_SIZE, 1);
        Write_FrameFRAM_Default_Section(INTERNAL_HEALTHDATA_FADDR, FRAME_FRAM_HEALTH_DAT_SIZE, 1);
        
        ClearEvents();
        ResetEnergy();
        ResetMinMaxFlags |= RESET_MINMAX_ALL_FLAG;
        Clear_Diag();
        Stp_to_Default();
        //TODO: too keep it consistant with PXR25 ADD Reset Power Demand,
        // Reset Current Demand
        break;
      case 30:  //LED Test
        ExAct.LED_image = DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8);  
        ExAct.State = LED_TEST;
        break;
      case 31:  //ZSI Output Test
        if(DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8))
        {
          SET_ZOUT;
        }
        else
        {
          CLEAR_ZOUT;
        }
        break;
      case 32:  //TA Test
        TA_TRIP_ACTIVE;
        break;                      
      case 33:  //Relay Test
        ExAct.Relay_image = DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8);
        ExAct.State = RELAY_TEST;
        break;
      default:
        break;
    }
    DPComm.AckNak = DP_ACK;
  }

  else if (actiontype == DP_EATYPE_DIAG)                            // Type = 4: Health and Diagnostics
  {
    switch(actionid)
    {
      case 0:                                   // SW No Trip Test (Only for LSIG trips)
        // DPComm.RxMsg[8]- Phase
        // DPComm.RxMsg[12..9]- Current (uint32)
        if (TestInjVars.Status != TESTINJ_STAT_INPROG)      // If not in progress, ok to start
        {
          TestInjVars.Type = TESTINJ_TYPE_SWNOTRIP;
          TestInjVars.Status = TESTINJ_STAT_INPROG;
          TestInjVars.Phase = DPComm.RxMsg[8];
          TestInjVars.Current = (DPComm.RxMsg[10] + (((uint32_t)DPComm.RxMsg[11]) << 8)
                    + (((uint32_t)DPComm.RxMsg[12]) << 16) + (((uint32_t)DPComm.RxMsg[13]) << 24));
          
          FW_SimulatedTest.Enable = TRUE;
          FW_SimulatedTest.Phase = TestInjVars.Phase;  
          FW_SimulatedTest.TestCurrent = TestInjVars.Current;
          FW_SimulatedTest.Trip_NoTrip = NO_TRIP;
          TestNoTrip = 1; 
          DPComm.AckNak = DP_ACK;
        }
        else
        {
          DPComm.AckNak = DP_NAK_NOTSTATE;
        }
        break;
      case 1:                                   // SW Trip Test (only for LSIG trips)
        // DPComm.RxMsg[8]- Phase
        // DPComm.RxMsg[12..9]- Current (uint32)
        if (TestInjVars.Status != TESTINJ_STAT_INPROG)      // If not in progress, ok to start
        {
          TestInjVars.Type = TESTINJ_TYPE_SWTRIP;
          TestInjVars.Status = TESTINJ_STAT_INPROG;
          TestInjVars.Phase = DPComm.RxMsg[8];
          TestInjVars.Current = (DPComm.RxMsg[10] + (((uint32_t)DPComm.RxMsg[11]) << 8)
                    + (((uint32_t)DPComm.RxMsg[12]) << 16) + (((uint32_t)DPComm.RxMsg[13]) << 24));
                  
          FW_SimulatedTest.Enable = TRUE;
          FW_SimulatedTest.Phase = TestInjVars.Phase;  
          FW_SimulatedTest.TestCurrent = TestInjVars.Current;
          FW_SimulatedTest.Trip_NoTrip = TRIP;
          TestTrip = 1;          
          DPComm.AckNak = DP_ACK;
        }
        else
        {
          DPComm.AckNak = DP_NAK_NOTSTATE;
        }
        break;
        
      case 2:                                   // HW No Trip Test (only for LSIG trips)
        // DPComm.RxMsg[8]- Phase
        // DPComm.RxMsg[12..9]- Current (uint32)       
        if ((TestInjVars.Status != TESTINJ_STAT_INPROG) || (Manufacture_Mode == FALSE))     // If not in progress, ok to start
        {                                           // There is not a No Trip test for Maintenance Moce, MCR, or Override (Mfg Mode tests)
          TestInjVars.Type = TESTINJ_TYPE_HWNOTRIP;
          TestInjVars.Status = TESTINJ_STAT_INPROG;
          TestInjVars.Phase = DPComm.RxMsg[8];
          TestInjVars.Current = (DPComm.RxMsg[10] + (((uint32_t)DPComm.RxMsg[11]) << 8)
                    + (((uint32_t)DPComm.RxMsg[12]) << 16) + (((uint32_t)DPComm.RxMsg[13]) << 24));
          
          // Call subroutine to perform the test injection.  Note, if the input is invalid, test injection will be
          //  turned off.  
          TestInjCur(TestInjVars.Phase, TestInjVars.Current);   // for a Protection processor trip (LSIG) 
          HW_SecInjTest.Enable = TRUE;
          HW_SecInjTest.Phase = TestInjVars.Phase;  
          HW_SecInjTest.Trip_NoTrip = NO_TRIP;
          TestNoTrip = 1;
          DPComm.AckNak = DP_ACK;
        }
        else
        {
          DPComm.AckNak = DP_NAK_NOTSTATE;
        }
        break;
      case 3:                                   // HW Trip Test
        // DPComm.RxMsg[8]- Phase
        // DPComm.RxMsg[12..9]- Current (uint32)
        if (TestInjVars.Status != TESTINJ_STAT_INPROG)      // If not in progress, ok to start
        {
          TestInjVars.Type = TESTINJ_TYPE_HWTRIP;
          TestInjVars.Status = TESTINJ_STAT_INPROG;
          TestInjVars.Phase = DPComm.RxMsg[8];
          TestInjVars.Current = (DPComm.RxMsg[10] + (((uint32_t)DPComm.RxMsg[11]) << 8)
                    + (((uint32_t)DPComm.RxMsg[12]) << 16) + (((uint32_t)DPComm.RxMsg[13]) << 24));
          HW_SecInjTest.Trip_NoTrip = TRIP;
          // Call subroutine to perform the test injection.  Note, if the input is invalid, test injection will be
          //  turned off
          if (Manufacture_Mode == FALSE)
          {
            TestInjCur(TestInjVars.Phase, TestInjVars.Current);   // for a Protection processor trip (LSIG)
          }
          else
          {
            TestInjCur_OvrMicro(TestInjVars.Phase, TestInjVars.Current);     // for an Override micro trip (MM, MCR, Override)
          }  

          HW_SecInjTest.Enable = TRUE;
          HW_SecInjTest.Phase = TestInjVars.Phase;  
          TestTrip = 1;        
          DPComm.AckNak = DP_ACK;
        }
        else
        {
          DPComm.AckNak = DP_NAK_NOTSTATE;
        }
        break;
      case 4:                                   // Cancel SW Test
        TestInjVars.Type = TESTINJ_TYPE_CANCELSW;
        TestInjVars.Status = TESTINJ_STAT_NONE;             // No results to return
        FW_SimulatedTest.Enable = FALSE;
        FW_SimulatedTest.State = 2;
        FW_SimulatedTest.TestCurrent = 0;  
        DPComm.AckNak = DP_ACK;
        break;
      case 5:                                   // Cancel HW Test
        TestInjVars.Type = TESTINJ_TYPE_CANCELHW;
        TestInjVars.Status = TESTINJ_STAT_NONE;             // No results to return
        if (TestInj.Flags & TEST_INJ_ON)                    // Only if test has been running
        {
          TestInj.Flags |= TEST_INJ_INIT_OFF;               // Turn off Sec Inj signal
        }          
        HW_SecInjTest.Enable = FALSE;
        HW_SecInjTest.State = 2; 
        DPComm.AckNak = DP_ACK;
        break;
      default:
        DPComm.AckNak = DP_NAK_BUFINVALID;
        break;
    }
  }

  else                                                              // Invalid type and ID - NAK
  {
    DPComm.AckNak = DP_NAK_BUFINVALID;
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ProcExActWAck()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcReadReqImm()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Read Immediate Request Message Subroutine
//
//  MECHANICS:          This subroutine handles immediate read request messages from the display processor.
//                      It
//                          - parses the read request message contained in DPComm.RxMsg[]
//                          - calls the appropriate subroutine to assemble the response into DPComm.TxBuf[]
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.RxMsg[] - the read request message prompting this response
// 
//  OUTPUTS:            None
//
//  ALTERS:             DPComm.AckNak
//
//  CALLS:              AssembleTxPkt(), BuildRTDBufByBufnum(), AssembleSetpBuffer(), AssembleEvntBuffer(),
//                      AssembleFactoryBuffer(), AssembleAck()
//
//------------------------------------------------------------------------------------------------------------

void ProcReadReqImm()
{
  uint16_t datalen;

  // The message has already been checked for address, CRC, and command.  The Read Request is stored as
  //   follows:
  //        DPComm.RxMsg[0]- Command = x02
  //        DPComm.RxMsg[1]- Source/Destination Address
  //        DPComm.RxMsg[2]- Sequence Number
  //        DPComm.RxMsg[3]- Buffer Type
  //        DPComm.RxMsg[4]- Buffer ID least significant byte
  //        DPComm.RxMsg[5]- Buffer ID most significant byte
  //        DPComm.RxMsg[6]- Buffer Length N least significant byte
  //        DPComm.RxMsg[7]- Buffer Length N most significant byte
  //        DPComm.RxMsg[8]- Buffer Information
  //        DPComm.RxMsg[9]- Buffer Information
  //          ..........................
  //        DPComm.RxMsg[N]- Buffer Information

  datalen = DPComm.RxMsg[6] + (((uint16_t)DPComm.RxMsg[7]) << 8);
  switch (DPComm.RxMsg[3])              // Switch on buffer type
  {
    case DP_BUFTYPE_RTDATA:             // Real-Time Data
      if (DPComm.RxMsg[5] > 0)              // Valid buffer numbers are 0 - 16
      {                                     // If MSB not zero, set LSB to invalid number to force NAK to be
        DPComm.RxMsg[4] = 255;              //   assembled
      }
      if (datalen == 2)                     // Two buf info bytes that are unused
      {
        BuildRTDBufByBufnum(DPComm.RxMsg[4],                // buffer number
                          DP_CMND_WRRESP,                   // command
                          (0x50 | (DPComm.Addr >> 4)));     // source and destination address
      }
      else
      {
        DPComm.AckNak = DP_NAK_BUFINVALID;
        AssembleAck();
      }
      break;

    case DP_BUFTYPE_DIAG:               // Health and Diagnostics
      if (datalen == 2)                     // Message should have two buf info bytes
      {
        AssembleDiagBuffer( (DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8)),     // buffer number
                            DP_CMND_WRRESP,                                             // command
                            (0x50 | (DPComm.Addr >> 4)),                                // src & dest addr
                            (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8)) );   // buffer info
      }
      else                                  // Otherwise it is invalid
      {
        DPComm.AckNak = DP_NAK_CMDINVALID;
        AssembleAck();
      }
      break;

    case DP_BUFTYPE_SETP:               // Setpoints
      if (datalen == 2)                     // Message should have two buf info bytes
      {
        AssembleSetpBuffer( (DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8)),     // buffer number
                            DP_CMND_WRRESP,                                             // command
                            (0x50 | (DPComm.Addr >> 4)),                                // src & dest addr
                            (DPComm.RxMsg[8] + (((uint16_t)DPComm.RxMsg[9]) << 8)) );   // buffer info
      }
      else                                  // Otherwise it is invalid
      {
        DPComm.AckNak = DP_NAK_CMDINVALID;
        AssembleAck();
      }
      break;

    case DP_BUFTYPE_EVENT:              // Events
      AssembleEvntBuffer( datalen,                                                    // message length
                          (DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8)),     // buffer number
                          DP_CMND_WRRESP,                                             // command
                          (0x50 | (DPComm.Addr >> 4)),                                // src & dest addr
                          &DPComm.RxMsg[8] );                                         // buffer info address
      break;


    case DP_BUFTYPE_FACTORY:            // Factory test command
      AssembleFactoryBuffer( (DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8)),   // buffer number
                             DP_CMND_WRRESP,                                           // command
                             (0x50 | (DPComm.Addr >> 4)) );                            // src & dest addr
      break;

    case DP_BUFTYPE_CHECK:              // Delayed Operation Status Check

      if(DPComm.Check_Status == CHK_STAT_IN_PROGRESS && ExAct.State == IDLE)
      {
        DPComm.Check_Status = CHK_STAT_COMPLETED;
      }
      
      DPComm.TxBuf[0] = START_OF_PKT;                 // Packet start
                                                      // Leave DPComm.TxBuf[1] open for first segment code
      DPComm.TxBuf[2] = DP_CMND_WRRESP;               // Command = Write Response
      DPComm.TxBuf[3] = (0x50 | (DPComm.Addr >> 4));  // Source/Destination Address
      DPComm.TxBuf[4] = DPComm.SeqNum;                // Sequence number
      DPComm.TxBuf[5] = DP_BUFTYPE_CHECK;             // Buffer type = 3: Real-Time Data
      DPComm.TxBuf[6] = 0xFF;                         // Buffer ID least significant byte
      DPComm.TxBuf[7] = 0xFF;                         // Buffer ID most significant byte
      DPComm.TxBuf[8] = 1;                            // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                            // Buffer length most significant byte
      DPComm.TxBuf[10] = DPComm.Check_Status;         // Data is the check status byte

      // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
      DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
      DPComm.TxSegCharCnt = 0;
      DPComm.TxNdx = 2;
      DPComm.TxCRC = 0xFFFF;
      // Note, it is ok to use TxBuf as both the source and destination because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 9, &DPComm, TRUE);
      break;

    default:                            // Invalid buffer type 
      DPComm.AckNak = DP_NAK_BUFTYPEINV;
      AssembleAck();
      break;
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ProcReadReqImm()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ProcReadReqDel()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Process Read Delayed Request Message Subroutine
//
//  MECHANICS:          This subroutine handles delayed read request messages from the display processor.
//                      It
//                          - parses the read request message contained in DPComm.RxMsg[]
//                          - if the message is valid:
//                              - saves the message in DPComm.RxMsgSav[]
//                              - sets DEL_MSG_INPROG flag to indicate delayed message is in progress
//                              - retrieves the appropriate data and generates a Write Buffer Response after
//                                the data has been retrieved
//                              - assembles an ACK response 
//                          - if the message is not valid:
//                              - assembles a NAK response
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.RxMsg[] - the read request message prompting this response
// 
//  OUTPUTS:            DPComm.RxMsgSav[], DPComm.Flags
//
//  ALTERS:             DPComm.AckNak
//
//  CALLS:              AssembleAck(), GetEVInfo()
//
//  EXECUTION TIME:     Measured on 230616 with interrupts and for reading a waveform: 125usec max 
//
//------------------------------------------------------------------------------------------------------------

void ProcReadReqDel()
{
  uint8_t i, k, evtype, wfvaloffset;
  uint16_t msgBID, temp1, temp2;
  uint32_t temp32, eid;

  // If a delayed message is already being processed, NAK this request
  if (DPComm.Flags & DEL_MSG_INPROG)
  {
    DPComm.AckNak = DP_NAK_NOTSTATE;
    AssembleAck();
    return;
  }

  // The message has already been checked for address, CRC, and command.  The Read Request is stored as
  //   follows:
  //        DPComm.RxMsg[0]- Command = x03
  //        DPComm.RxMsg[1]- Source/Destination Address
  //        DPComm.RxMsg[2]- Sequence Number
  //        DPComm.RxMsg[3]- Buffer Type
  //        DPComm.RxMsg[4]- Buffer ID least significant byte
  //        DPComm.RxMsg[5]- Buffer ID most significant byte
  //        DPComm.RxMsg[6]- Buffer Length N least significant byte
  //        DPComm.RxMsg[7]- Buffer Length N most significant byte
  //        DPComm.RxMsg[8-13]- Buffer Information
  //            For events: DPComm.RxMsg[11..8] - EID of desired waveform
  //                        DPComm.RxMsg[12] - cycle number (0 - 35)

  msgBID = DPComm.RxMsg[4] + (((uint16_t)DPComm.RxMsg[5]) << 8); // Assemble the Buffer ID
  // Check whether the request is valid
  switch (DPComm.RxMsg[3])              // Switch on buffer type
  {
    case DP_BUFTYPE_EVENT:                  // Event Type
      if (// (msgBID == 32)                         // BID =    32: Energy and Demand
//        || ((msgBID >= 40) && (msgBID <= 42))            40-42: Waveforms, all by page (7 sample sets)
         ((msgBID >= 64) && (msgBID <= 96)) )   //       64-96: Waveform, single value, by cycle
      {
        // The message is valid so far.  For now, we are only supporting reading a single waveform a cycle
        //   at a time.  We will check the EID and save the corresponding index.  In the extremely unlikely
        //   event that the waveform is erased before we retrieve it, we'll get all 0xFF values and the data
        //   will obviously be corrupted.
        if (msgBID < 75)                                     // Set evtype to the waveform type, temp32 to
        {                                                    //   the base address of the waveforms in
          evtype = EV_TYPE_TRIPWF;                           //   Flash, and initialize wfvaloffset to the
          temp32 = ((uint32_t)TRIP_WAVEFORMS_START) << 12;   //   index of the value within the sample set
          wfvaloffset = msgBID - 64;
        }
        else if (msgBID < 86)
        {
          evtype = EV_TYPE_ALARMWF;
          temp32 = ((uint32_t)ALARM_WAVEFORMS_START) << 12;
          wfvaloffset = msgBID - 75;
        }
        else
        {
          evtype = EV_TYPE_EXTCAPWF;
          temp32 = ((uint32_t)EXTCAP_WAVEFORM_START) << 12;
          wfvaloffset = msgBID - 86;
        }
        // Input index is 0=Ia, ..., 3=In, 4=Ig, 5=Van1, ..., 10=Vcn2
        // However, sample set is stored as 0=Ia, ..., 3=In, 4=Igsrc, 5=Igres, 6=Van1, ..., 11=Vcn2
        if (wfvaloffset <= 3)                 // If Ia - In, no index adjustment, 4 bytes per value
        {
          wfvaloffset = wfvaloffset * 4;
        }
        else if (wfvaloffset == 4)            // If Ig, just use as is for now  *** DAH  TALK TO BERT, COULD PROBABLY JUST ALWAYS USE 4 (SET TO EITHER SRC OR RES) AND SKIP 5
        {
          wfvaloffset = wfvaloffset * 4;
        }
        else                                  // If Van1 - Vcn2, adjust for six current values, 2 bytes
        {                                     //   per value
          wfvaloffset = 24 + (wfvaloffset -5) * 2;
        }
        // Search for the desired EID
        // First assemble the EID
        eid = DPComm.RxMsg[8] + (((uint32_t)DPComm.RxMsg[9]) << 8) + (((uint32_t)DPComm.RxMsg[10]) << 16)
                        + (((uint32_t)DPComm.RxMsg[11]) << 24);
        if (!EventGetWfEIDs(evtype, eid, &i))   // Subroutine returns True if EID found.  If not found, NAK
        {
          DPComm.AckNak = DP_NAK_BUFINVALID;
          AssembleAck();
        }
        else                                    // Otherwise i has the index of the requested waveform
        {
          // Retrieve the header info for the desired waveform (at index = i):
              // SPI2_buf[1..0] - Number of samples
              // SPI2_buf[5..2] - EID 
              // SPI2_buf[13..6] - Time stamp
          FRAM_Read( (FRAM_EV_INFO[evtype] + (i * WF_HEADER_SIZE)), 7, (uint16_t *)(&SPI2_buf[0]) );

          // Check whether the captured waveform has at least some of the samples in the requested waveform
          //   (the captured waveform may have been aborted), and compute the number of samples to capture
          // Set temp1 to the number of samples in the waveform capture (2880 max)
          temp1 = ((uint16_t)SPI2_buf[1] << 8) + SPI2_buf[0];
          // Set temp2 to the index of the first sample
          temp2 = DPComm.RxMsg[12] * 80;
          if (temp2 >= temp1)           // If requested sample does not exist (remember, the capture could
          {                             //   have been aborted), we are done
            DPComm.TxDelMsgBuf[0] = START_OF_PKT;       // Packet start
                                                        // Leave DPComm.TxBuf[1] open for first segment code
            DPComm.TxDelMsgBuf[2] = DP_CMND_WRRESP;     // Command = Write Response
            DPComm.TxDelMsgBuf[3] = (0x50 | (DPComm.RxMsg[1] >> 4));  // Source/Destination Address
            DPComm.TxDelMsgBuf[4] = DPComm.RxMsg[2];    // Sequence number
            DPComm.TxDelMsgBuf[5] = DP_BUFTYPE_EVENT;   // Buffer type = 7: Event
            DPComm.TxDelMsgBuf[6] = DPComm.RxMsg[4];    // Buffer ID least significant byte
            DPComm.TxDelMsgBuf[7] = DPComm.RxMsg[5];    // Buffer ID most significant byte
            DPComm.TxDelMsgBuf[8] = 14;                 // Buffer length least significant byte
            DPComm.TxDelMsgBuf[9] = 0;                  // Buffer length most significant byte
            DPComm.TxDelMsgBuf[10] = SPI2_buf[2];       // EID LS byte
            DPComm.TxDelMsgBuf[11] = SPI2_buf[3];
            DPComm.TxDelMsgBuf[12] = SPI2_buf[4];
            DPComm.TxDelMsgBuf[13] = SPI2_buf[5];       // EID MS byte
            DPComm.TxDelMsgBuf[14] = DPComm.RxMsg[12];  // Cycle number
            for (k = 15; k < 24; ++k)
            {
              DPComm.TxDelMsgBuf[k] = 0;                // Timestamp and number of cycles is 0
            }
            DPComm.Flags |= GEN_WRITERESP;              // Set flag to transmit the response
          }
          else                          // Otherwise we can return some samples (80 max)
          {
            DPComm.TxDelMsgBuf[0] = START_OF_PKT;       // Packet start
                                                        // Leave DPComm.TxBuf[1] open for first segment code
            DPComm.TxDelMsgBuf[2] = DP_CMND_WRRESP;     // Command = Write Response
            DPComm.TxDelMsgBuf[3] = (0x50 | (DPComm.RxMsg[1] >> 4));  // Source/Destination Address
            DPComm.TxDelMsgBuf[4] = DPComm.RxMsg[2];    // Sequence number
            DPComm.TxDelMsgBuf[5] = DP_BUFTYPE_EVENT;   // Buffer type = 7: Event
            DPComm.TxDelMsgBuf[6] = DPComm.RxMsg[4];    // Buffer ID least significant byte
            DPComm.TxDelMsgBuf[7] = DPComm.RxMsg[5];    // Buffer ID most significant byte
            // Buffer length is entered in SPI1_Flash_Manager() after the samples have been retrieved.  We
            //   are not sure how many samples are available
            DPComm.TxDelMsgBuf[10] = SPI2_buf[2];       // EID LS byte
            DPComm.TxDelMsgBuf[11] = SPI2_buf[3];
            DPComm.TxDelMsgBuf[12] = SPI2_buf[4];
            DPComm.TxDelMsgBuf[13] = SPI2_buf[5];       // EID MS byte
            DPComm.TxDelMsgBuf[14] = DPComm.RxMsg[12];  // Cycle number
            for (k = 6; k < 14; ++k)                    // Time stamp is in SPI2_buf[13..6].  Put it in
            {                                           //   DPComm.TxDelMsgBuf[22..15]
              DPComm.TxDelMsgBuf[k+9] = SPI2_buf[k];
            }
            // Compute the base address of the first sample - this doesn't change
            //   The page and byte offsets are handled in SPI1_Flash_Manager(), as these change as we read
            //   the samples
            // i still has the index of the requested waveform
            temp32 += ((i * WF_SIZE_IN_SECTORS) << 12)           // offset of the selected waveform log
                   + (WF_ADDRESS[DPComm.RxMsg[12]][0] << 8)      // starting page address of the cycle
                   + wfvaloffset;                                // offset of the selected value
            // Save this address
            DPComm.RxMsgSav[0] = (uint8_t)temp32;
            DPComm.RxMsgSav[1] = (uint8_t)(temp32 >> 8);
            DPComm.RxMsgSav[2] = (uint8_t)(temp32 >> 16);
            DPComm.RxMsgSav[3] = (uint8_t)(temp32 >> 24);

            // Compute the number of available samples (80 max).
            //   temp1 - number of samples in the waveform capture (2880 max)
            //   temp2 - index of the first sample (cycle number = DPComm.RxMsg[12] = 0-35) (0 - 2800)
            //   We already checked, and temp1 > temp2
            temp1 -= temp2;
            // Save this value and also update the transmit buffer
            DPComm.RxMsgSav[4] = (uint8_t)((temp1 > 80) ? (80) : (temp1));
            DPComm.TxDelMsgBuf[23] = DPComm.RxMsgSav[4];
            DPComm.RxMsgSav[5] = 0;                   // Initialize the page count and save it
            // Save the value offset, this will indicate whether we are reading currents or voltages
            DPComm.RxMsgSav[6] = wfvaloffset;
            // Initialize the index to store the samples and save it
            DPComm.RxMsgSav[7] = 24;
            DPComm.RxMsgSav[8] = 0;

            DPComm.AckNak = DP_ACK;                   // Ack the request
            AssembleAck();
            SPI1Flash.Req |= S1F_RD_WF;               // Set request flag to read a waveform
            DPComm.Flags |= DEL_MSG_INPROG;           // Set flag for delayed msg process is in progress
          }
        }
      }
      // If invalid buffer ID, NAK
      else
      {
        DPComm.AckNak = DP_NAK_BUFINVALID;
        AssembleAck();
      }
      break;

    default:
      DPComm.AckNak = DP_NAK_BUFTYPEINV;
      AssembleAck();
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ProcReadReqDel()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        BuildRTDBufByTSlice()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Real Time Data Buffer by Time Slice Subroutine
//
//  MECHANICS:          This subroutine assembles an RTD Buffer message for Write Without Acknowledge
//                      (Command = 5) and Write With Acknowledge (Command = 6) commands.
//                      This subroutine assumes these commands are initiated periodically in
//                      DispComm_Tx().  Input parameters timeslice, DPTxReqFlags, DP_Tmr4BufSel, and
//                      DP_Tmr5BufSel determine which RTD buffer to assemble.  If timeslice is invalid, a
//                      NAK message is assembled.
//                      The response is placed into DPComm.TxBuf[]
//                      The buffer assembled for transmission is as follows:
//                      timeslice    DPTxReqFlags      DP_Tmr4BufSel    DP_Tmr5BufSel    Buffer Xmitted
//                         0          don't care        don't care       don't care            0
//                         1          don't care        don't care       don't care            1
//                         2          don't care        don't care       don't care            2
//                         3             none           don't care       don't care            3
//                         3       DP_5MINAVG_BUF_REQ   don't care       don't care            3
//                         3       DP_EXTDIAG_BUF_REQ   don't care       don't care           17
//                         3       DP_VERREV_BUF_REQ    don't care       don't care           19
//                         4          don't care          0, >2          don't care            4
//                         4          don't care            1            don't care            5
//                         4          don't care            2            don't care            6
//                         5          don't care        don't care         0, >2               7
//                         5          don't care        don't care           1                 8
//                         5          don't care        don't care           2                 9
//                         6          don't care        don't care       don't care           10
//
//  CAVEATS:            As stated above, this subroutine assumes the commands are initiated periodically in
//                      DispComm_Tx().  It cannot be used to service Read Request commands (generating Write
//                      Responses), because the buffer number is not directly inputted.
//
//  INPUTS:             timeslice - the RTD time slice (0 - 6) for the bufferbuffer ID number
//                      DPTxReqFlags - flags indicating whether buffers 3, 17, or 19 should be sent
//                      DP_Tmr4BufSel - counter indicating whether buffers 4, 5, or 6 should be sent
//                      DP_Tmr5BufSel - counter indicating whether buffers 7, 8, or 9 should be sent
//                      DPComm.Addr - The Source and Destination Address byte received from the Display
//                              Processor
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             DPTxReqFlags, DP_Tmr4BufSel, DP_Tmr5BufSel
//
//  CALLS:              AssembleTxPkt(), AssembleAck()
//
//  EXECUTION TIME:     Measured on 220411 with Buffer 0: ~200usec   *** DAH MEASURED BEFORE HARMONICS WERE ADDED
//
//------------------------------------------------------------------------------------------------------------

void BuildRTDBufByTSlice(uint8_t timeslice, uint8_t cmnd, uint8_t addr)
{
  uint8_t bufnum;
  uint16_t temp, i;
  void * const *objaddr_ptr;

  // Common code for all of the real-time data buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = cmnd;                    // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_RTDATA;       // Buffer type = Real-Time Data
  DPComm.TxBuf[7] = 0x00;                    // Buffer ID most significant byte is always 0

  // Initialize buffer number to the time slice
  bufnum = timeslice;

  // If timeslice is 3, either the 5-minute values (buffer 3), firmware version info (buffer 19), or
  //   customer (external) diagnostic buffer (buffer 17) needs to be sent.  These buffers are not sent every
  //   250msec.  They are sent only when the data changes.
  //   The priority is firmware info, diagnostic buffer, 5-minute values.  bufnum is set according to
  //   the flags if necessary
  if (timeslice == 3)
  {
    if (DPTxReqFlags & DP_VERREV_BUF_REQ)
    {
      bufnum = 19;
      DPTxReqFlags &= (0xFF ^ DP_VERREV_BUF_REQ);           // Clear the request flag
    }
    else if (DPTxReqFlags & DP_EXTDIAG_BUF_REQ)
    {
      bufnum = 17;
      DPTxReqFlags &= (0xFF ^ DP_EXTDIAG_BUF_REQ);          // Clear the request flag
    }
    else
    {
      DPTxReqFlags &= (0xFF ^ DP_5MINAVG_BUF_REQ);          // Clear the request flag
    }
  }
  // If not a diagnostic buffer, either it is the 5-minute average buffer or nothing changed (the timer just
  //   expired naturally).  In either case, send the 5-minute average buffer (buffer 3)

  // If timeslice is 4, either the min/max currents (buffer 4), min/max voltages (buffer 5), or min/max
  //   powers (buffer 6) needs to be sent
  else if (timeslice == 4)
  {
    if (DP_Tmr4BufSel == 1)
    {
      bufnum = 5;
    }
    else if (DP_Tmr4BufSel == 2)
    {
      bufnum = 6;
    }
    DP_Tmr4BufSel = ((DP_Tmr4BufSel < 2) ? (DP_Tmr4BufSel + 1) : 0);
  }

  // If timeslice is 5, either the min/max frequencies (buffer 7), min/max unbalances (buffer 8), or min/max
  //   THD (buffer 9) needs to be sent
  else if (timeslice == 5)
  {
    if (DP_Tmr5BufSel == 0)
    {
      bufnum = 7;
    }
    else if (DP_Tmr5BufSel == 1)
    {
      bufnum = 8;
    }
    else if (DP_Tmr5BufSel == 2)
    {
      bufnum = 9;
    }
    DP_Tmr5BufSel = ((DP_Tmr5BufSel < 2) ? (DP_Tmr5BufSel + 1) : 0);
  }

  // If timeslice is 6, the min/max PF (buffer 10) needs to be sent
  else if (timeslice == 6)
  {
    bufnum = 10;
  }

  DPComm.TxBuf[6] = bufnum;                  // Buffer ID least significant byte

  // Get and store the buffer length.  There are three cases.  The first is for normal real-time data
  // buffers:
  //   Set temp to the buffer length, which is the number of bytes to be transmitted.  The buffers consist
  //     of addresses, which are 4 bytes long.  All of the buffer data objects are 4 bytes long also, except
  //     for buffer 1A.  This buffer contains energy buffers, which are 8 bytes long.  For this buffer, we
  //     need to multiply the size by two.
  if (bufnum < 11)
  {
    temp = ( (bufnum == 1) ?
      ((sizeof(DPCOMM_RTD_ADDR_BUF1_A) * 2) + DPCOMMM_RTD_BUFSIZE[bufnum]) : DPCOMMM_RTD_BUFSIZE[bufnum] );
  }
  // The second case is for the external diagnostics buffers 
  else if (bufnum == 17)
  {
    // DIAG_DATA_SIZE includes the checksum, and does not include the validity indicator (which shows
    //   whether the data was retrieved correctly from FRAM), so subtract 4 and add 1 to get the length
    temp = DIAG_DATA_SIZE - 4 + 1;
  }
  // The third case is for the firmware version info.  The length is 5
  else
  {
    temp = 5;
  }
  DPComm.TxBuf[8] = (uint8_t)temp;           // Buffer length least significant byte
  DPComm.TxBuf[9] = (uint8_t)(temp >> 8);    // Buffer length most significant byte
  // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
  DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
   // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 8, &DPComm, FALSE);

  switch (bufnum)
  {
    case 0:                             // Buffer 0:  Current, voltage, power, unbalance, CF, frequency
    case 1:                             // Buffer 1:  Energy, THD, PF, sequence components, and phase angles
    case 2:                             // Buffer 2:  Demand current and power
    case 3:                             // Buffer 3:  5-minute average current and power
    case 4:                             // Buffer 4:  Min/max current
    case 5:                             // Buffer 5:  Min/max voltage
    case 6:                             // Buffer 6:  Min/max power
    case 7:                             // Buffer 7:  Min/max frequency, overloads
    case 8:                             // Buffer 8:  Min/max current and voltage unbalance
    case 9:                             // Buffer 9:  Min/max current and voltage THD
    case 10:                            // Buffer 10: Min/max power factor
      // Buffer 1 is unique.  It is divided into two buffers.  The first nine objects are 8 bytes long.
      //   These are processed first.  The remaining objects are 4 bytes long and are processed next, in the
      //   same manner as the other buffers.
      if (bufnum == 1)
      {
        objaddr_ptr = DPCOMM_RTD_ADDR_BUF1_A;
        // Insert the objects into the transmit buffer
        for (i=0; i<9; ++i)
        {
          // objaddr_ptr[i] is the data object in the buffer.  It is cast as a byte pointer
          // 8 bytes are inserted since each object is 8 bytes in length
          AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 8, &DPComm, FALSE);
        }
        // Reinitialize temp to buffer 1B size.  It was buffer 1A + 1B.  Buffer 1A is finished, so temp
        //   must now be just for 1B
        temp = DPCOMMM_RTD_BUFSIZE[bufnum];
      }

      // Set pointer to the address of the first data object in the buffer
      objaddr_ptr = DPCOMM_RTD_ADDR[bufnum];

      // For buffer 0 only, send one-cycle currents if test injection is on because: 1) these are used for
      //   protection, and 2) 200msec currents (through the dc filter) are not available when dc is used for
      //   the higher injected currents
      if ( (bufnum == 0) && (TestInj.Flags & TEST_INJ_ON) )
      {
        objaddr_ptr = DPCOMM_RTD_ADDR_BUF0_TESTINJ;
      }

      // Set temp to the number of data objects in the buffer - 1.  Each object is 4 bytes long, so divide
      //   by 4.  The last object is processed differently than the other objects (parameter LastSet in
      //   AssembleTxPkt is True instead of False)
      temp = temp/4 - 1;
      // Insert the objects into the transmit buffer
      for (i=0; i<temp; ++i)
      {
        // objaddr_ptr[i] is the data object in the buffer.  It is cast as a byte pointer
        // 4 bytes are inserted since each object is a byte in length
        AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 4, &DPComm, FALSE);
      }
      // Last object - set parameter to True
      AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 4, &DPComm, TRUE);
      break;

    case 17:                            // Buffer 17: External Diagnostics
      // Retrieve diagnostics info from Frame FRAM
      i = Get_Diag(1);

      // Insert the validity after the last data object.  Since DIAG_DATA_SIZE includes the 4 checksum
      //   bytes, the validity data object is at DIAG_DATA_SIZE - 4.  The data amount is
      //   DIAG_DATA_SIZE - 4 + 1
      SPI2_buf[DIAG_DATA_SIZE - 4] = (uint8_t)i;
      // We will transmit this as a mirror image of the Frame FRAM storage, so insert them all with one
      //   subroutine call
      AssembleTxPkt(&SPI2_buf[0], (DIAG_DATA_SIZE - 4 + 1), &DPComm, TRUE);
      break;

    case 19:                            // Buffer 19: Firmware Version Info
      // Assemble the firmware version info
      SPI2_buf[0] = PROT_PROC_FW_VER;
      SPI2_buf[1] = PROT_PROC_FW_REV;
      SPI2_buf[2] = (uint8_t)PROT_PROC_FW_BUILD;
      SPI2_buf[3] = (uint8_t)(PROT_PROC_FW_BUILD >> 8);
      SPI2_buf[4] = Ovr_FW_Version;
      AssembleTxPkt(&SPI2_buf[0], 5, &DPComm, TRUE);
      break;

    default:                            // Invalid Buffer Number - NAK
      DPComm.AckNak = DP_NAK_BUFINVALID;
      AssembleAck();
      break;
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          BuildRTDBufByTSlice()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        BuildRTDBufByBufnum()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Real Time Data Buffer by Buffer Number Subroutine
//
//  MECHANICS:          This subroutine assembles an RTD Buffer message for Write Response (Command = 4),
//                      Write Without Acknowledge (Command = 5), and Write With Acknowledge (Command = 6)
//                      commands.
//                      Input parameter bufnum determines which RTD buffer to assemble.  If bufum is
//                      invalid, a NAK message is assembled.  The response is placed into DPComm.TxBuf[]
//
//  CAVEATS:            None
//
//  INPUTS:             bufnum - the buffer ID number
//                      DPComm.Addr - The Source and Destination Address byte received from the Display
//                              Processor
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             DPTxReqFlags
//
//  CALLS:              AssembleTxPkt(), AssembleAck()
//
//  EXECUTION TIME:     Measured on 220411 with Buffer 0: ~200usec   *** DAH MEASURED BEFORE HARMONICS WERE ADDED
//
//------------------------------------------------------------------------------------------------------------

void BuildRTDBufByBufnum(uint8_t bufnum, uint8_t cmnd, uint8_t addr)
{
  uint16_t temp, i;
  void * const *objaddr_ptr;

  // Common code for all of the real-time data buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = cmnd;                    // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_RTDATA;       // Buffer type = Real-Time Data
  DPComm.TxBuf[6] = bufnum;                  // Buffer ID least significant byte
  DPComm.TxBuf[7] = 0x00;                    // Buffer ID most significant byte is always 0

  // Get and store the buffer length.  There are four cases.  The first is for normal real-time data
  // buffers:
  //   Set temp to the buffer length, which is the number of bytes to be transmitted.  The buffers consist
  //     of addresses, which are 4 bytes long.  All of the buffer data objects are 4 bytes long also, except
  //     for buffer 1A.  This buffer contains energy buffers, which are 8 bytes long.  For this buffer, we
  //     need to multiply the size by two.
  if (bufnum < 11)
  {
    temp = ( (bufnum == 1) ?
      ((sizeof(DPCOMM_RTD_ADDR_BUF1_A) * 2) + DPCOMMM_RTD_BUFSIZE[bufnum]) : DPCOMMM_RTD_BUFSIZE[bufnum] );
  }
  // The second case is for the diagnostics buffers.  Both buffers are the same size 
  else if ( (bufnum == 17) || (bufnum == 18) )
  {
    // DIAG_DATA_SIZE includes the checksum, and does not include the validity indicator (which shows
    //   whether the data was retrieved correctly from FRAM), so subtract 4 and add 1 to get the length
    temp = DIAG_DATA_SIZE - 4 + 1;
  }
  // The third case is for the firmware version info.  The length is 5
  else if (bufnum == 19)
  {
    temp = 5;
  }
  // The fourth case is for harmonics.  The data length is either 320 (for the current harmonics) or 240
  //   (for the voltage harmonics)
  else
  {
    if ( (bufnum == 11) || (bufnum == 14) )
    {
      temp = 320;
    }
    else                                        // bufnum = 12, 13, 15, 16
    {
      temp = 240;
    }
  }
  DPComm.TxBuf[8] = (uint8_t)temp;           // Buffer length least significant byte
  DPComm.TxBuf[9] = (uint8_t)(temp >> 8);    // Buffer length most significant byte
  // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
  DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
   // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 8, &DPComm, FALSE);

  switch (bufnum)
  {
    case 0:                             // Buffer 0:  Current, voltage, power, unbalance, CF, frequency
    case 1:                             // Buffer 1:  Energy, THD, PF, sequence components, and phase angles
    case 2:                             // Buffer 2:  Demand current and power
    case 3:                             // Buffer 3:  5-minute average current and power
    case 4:                             // Buffer 4:  Min/max current
    case 5:                             // Buffer 5:  Min/max voltage
    case 6:                             // Buffer 6:  Min/max power
    case 7:                             // Buffer 7:  Min/max frequency, overloads
    case 8:                             // Buffer 8:  Min/max current and voltage unbalance
    case 9:                             // Buffer 9:  Min/max current and voltage THD
    case 10:                            // Buffer 10: Min/max power factor
      // Buffer 1 is unique.  It is divided into two buffers.  The first nine objects are 8 bytes long.
      //   These are processed first.  The remaining objects are 4 bytes long and are processed next, in the
      //   same manner as the other buffers.
      if (bufnum == 1)
      {
        objaddr_ptr = DPCOMM_RTD_ADDR_BUF1_A;
        // Insert the objects into the transmit buffer
        for (i=0; i<9; ++i)
        {
          // objaddr_ptr[i] is the data object in the buffer.  It is cast as a byte pointer
          // 8 bytes are inserted since each object is 8 bytes in length
          AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 8, &DPComm, FALSE);
        }
        // Reinitialize temp to buffer 1B size.  It was buffer 1A + 1B.  Buffer 1A is finished, so temp
        //   must now be just for 1B
        temp = DPCOMMM_RTD_BUFSIZE[bufnum];
      }

      // Set pointer to the address of the first data object in the buffer
      objaddr_ptr = DPCOMM_RTD_ADDR[bufnum];

      // For buffer 0 only, send one-cycle currents if test injection is on because: 1) these are used for
      //   protection, and 2) 200msec currents (through the dc filter) are not available when dc is used for
      //   the higher injected currents
      if ( (bufnum == 0) && (TestInj.Flags & TEST_INJ_ON) )
      {
        objaddr_ptr = DPCOMM_RTD_ADDR_BUF0_TESTINJ;
      }

      // Set temp to the number of data objects in the buffer - 1.  Each object is 4 bytes long, so divide
      //   by 4.  The last object is processed differently than the other objects (parameter LastSet in
      //   AssembleTxPkt is True instead of False)
      temp = temp/4 - 1;
      // Insert the objects into the transmit buffer
      for (i=0; i<temp; ++i)
      {
        // objaddr_ptr[i] is the data object in the buffer.  It is cast as a byte pointer
        // 4 bytes are inserted since each object is a byte in length
        AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 4, &DPComm, FALSE);
      }
      // Last object - set parameter to True
      AssembleTxPkt((uint8_t *)(objaddr_ptr[i]), 4, &DPComm, TRUE);
      break;

    case 11:                            // Buffer 11: Aggregated Harmonics Ia thru In
    case 12:                            // Buffer 12: Aggregated Harmonics Van thru Vcn
    case 13:                            // Buffer 13: Aggregated Harmonics Vab thru Vca
    case 14:                            // Buffer 14: Captured Harmonics Ia thru In
    case 15:                            // Buffer 15: Captured Harmonics Van thru Vcn
    case 16:                            // Buffer 16: Captured Harmonics Vab thru Vca
      objaddr_ptr = DPCOMM_HARM[bufnum - 11];       // Set pointer to first object to insert
      if ( (bufnum == 11) || (bufnum == 14) )
      {
        temp = 320;
      }
      else
      {
        temp = 240;
      }
      // All of the harmonics objects are two bytes each and are in order, so insert them all with one
      //   subroutine call
      AssembleTxPkt((uint8_t *)(objaddr_ptr), temp, &DPComm, TRUE);
      break;

    case 17:                            // Buffer 17: External Diagnostics
    case 18:                            // Buffer 18: Internal Diagnostics
      // Retrieve diagnostics info from Frame FRAM
      if (bufnum == 17)
      {
        i = Get_Diag(1);
      }
      else
      {
        i = Get_Diag(0);
      }
      // Insert the validity after the last data object.  Since DIAG_DATA_SIZE includes the 4 checksum
      //   bytes, the validity data object is at DIAG_DATA_SIZE - 4.  The data amount is
      //   DIAG_DATA_SIZE - 4 + 1
      SPI2_buf[DIAG_DATA_SIZE - 4] = (uint8_t)i;
      // We will transmit this as a mirror image of the Frame FRAM storage, so insert them all with one
      //   subroutine call
      AssembleTxPkt(&SPI2_buf[0], (DIAG_DATA_SIZE - 4 + 1), &DPComm, TRUE);
      break;

    case 19:                            // Buffer 19: Firmware Version Info
      // Assemble the firmware version info
      SPI2_buf[0] = PROT_PROC_FW_VER;
      SPI2_buf[1] = PROT_PROC_FW_REV;
      SPI2_buf[2] = (uint8_t)PROT_PROC_FW_BUILD;
      SPI2_buf[3] = (uint8_t)(PROT_PROC_FW_BUILD >> 8);
      SPI2_buf[4] = Ovr_FW_Version;
      AssembleTxPkt(&SPI2_buf[0], 5, &DPComm, TRUE);
      break;

    default:                            // Invalid Buffer Number - NAK
      DPComm.AckNak = DP_NAK_BUFINVALID;
      AssembleAck();
      break;
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          BuildRTDBufByBufnum()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleExActBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Execute Action Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles an Execute Action message.
//                      Input parameters type and id determine which EA buffer to assemble.  The message is
//                      placed into DPComm.TxBuf[]
//
//  CAVEATS:            None
//
//  INPUTS:             type - the execute action type
//                      id - the execute action ID number
//                      addr - The Source and Destination Address
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt(), AssembleAck()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void AssembleExActBuffer(uint8_t type, uint16_t id, uint8_t addr)
{

  // Common code for all of the execute action buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = DP_CMND_EXWACK;          // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = type;                    // Action type
  DPComm.TxBuf[6] = (uint8_t)id;             // Action ID ls byte
  DPComm.TxBuf[7] = (uint8_t)(id >> 8);      // Action ID ms byte

  if ( (type == DP_EATYPE_TIME) && (id == DP_EAID_WRITETIME) )      // If write time...
  {
    DPComm.TxBuf[8] = 8;                                                  // Length = 8
    DPComm.TxBuf[9] = 0;
    DPComm.TxBuf[10] = (uint8_t)DP_OutSyncTime.Time_secs;                 // Data = seconds
    DPComm.TxBuf[11] = (uint8_t)(DP_OutSyncTime.Time_secs >> 8);
    DPComm.TxBuf[12] = (uint8_t)(DP_OutSyncTime.Time_secs >> 16);
    DPComm.TxBuf[13] = (uint8_t)(DP_OutSyncTime.Time_secs >> 24);
    DPComm.TxBuf[14] = (uint8_t)DP_OutSyncTime.Time_nsec;                 // Data = nsec
    DPComm.TxBuf[15] = (uint8_t)(DP_OutSyncTime.Time_nsec >> 8);
    DPComm.TxBuf[16] = (uint8_t)(DP_OutSyncTime.Time_nsec >> 16);
    DPComm.TxBuf[17] = (uint8_t)(DP_OutSyncTime.Time_nsec >> 24);
  }

  // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
  DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
   // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 16, &DPComm, TRUE);

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleExActBuffer()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleSetpBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Setpoints Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a Setpoints Buffer message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             bufid - the buffer ID
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfo - the buffer information bytes (the desired setpoints set, 0..3)
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt()
//
//  EXECUTION TIME:     Measured on 221115 with retrieving Buffer 1 setpoints from FRAM: 810usec max
//                      This is the worst-case condition, because we have to retrieve both Group0 and Group1
//                      setpoints
//
//------------------------------------------------------------------------------------------------------------

void AssembleSetpBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr, uint16_t bufinfo)
{
  uint16_t *sptr;
  uint16_t i, numsetp;

  // bufinfo contains the request setpoints set (0..3).  If it is greater than 3 (invalid), set it to the
  //   active setpoints set
  bufinfo = ( (bufinfo < 4) ? (bufinfo) : SetpActiveSet);
  
  // Common to all of the setpoints buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = cmnd;                    // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_SETP;         // Buffer type = Setpoints
  DPComm.TxBuf[6] = (uint8_t)bufid;          // Buffer ID least significant byte
  DPComm.TxBuf[7] = (uint8_t)(bufid >> 8);   // Buffer ID most significant byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                       // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;

  // Setpoints groups are from 0..(NUM_STP_GROUPS - 1).  If Buf ID exceeds the max group number, just return
  //   the active setpoints set (0..3)
  if (bufid >= NUM_STP_GROUPS)
  {
    DPComm.TxBuf[8] = 1;                     // Buffer length least significant byte
    DPComm.TxBuf[9] = 0;                     // Buffer length most significant byte
    DPComm.TxBuf[10] = SetpActiveSet;        // Data is the active setpoints set

    // Call subroutine to assemble the packet per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 9, &DPComm, TRUE);
  }
  else
  {
    // If the requested setpoints are the active set, just use the ones in RAM
    //   We do not need to worry about the read-only Group1 setpoints.  These were already corrected
    //   (overwritten) to the corresponding Group0 setpoints when they were retrieved from Frame FRAM
    //   Set sptr to first setpoint in the group referenced by bufid
    if (bufinfo == SetpActiveSet)
    {
      sptr = SETP_GR_DATA_ADDR[bufid];
    }
    // If the requested setpoints are not the active set, retrieve them from FRAM
    else
    {
      // Read-only setpoints are handled in Get_Setpoints()
      Get_Setpoints(bufinfo, (uint8_t)bufid, (uint8_t *)(&i)); // Setpoints returned in SetpScratchBuf[]
      sptr = &SetpScratchBuf[0];                        // Set ptr to the first setpoint
    }
    // SETP_GR_SIZE[ ] is the structure size.  Divide by 2 because each setpoint is a word.  Subtract 2
    //   because the structure includes the checksum and checksum complement 
    numsetp = ((SETP_GR_SIZE[bufid] >> 1) - 2);
    // Buffer length is size of the setpoints structure (in bytes) - 4 (checksum & checksum complement),
    //   plus add two bytes because we will also return the requested setpoints set and the active set
    DPComm.TxBuf[8] = (uint8_t)(SETP_GR_SIZE[bufid] - 2);          // Buf len least significant byte
    DPComm.TxBuf[9] = (uint8_t)((SETP_GR_SIZE[bufid] - 2) >> 8);   // Buf len most significant byte
    DPComm.TxBuf[10] = (uint8_t)bufinfo;                           // Requested setpoints set
    DPComm.TxBuf[11] = (uint8_t)SetpActiveSet;                     // Active setpoints set

    // Call subroutine to assemble this portion of the packet per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, FALSE);

    // Put the data objects in
    AssembleTxPkt((uint8_t *)(&sptr[0]), numsetp*2, &DPComm, TRUE);

  }
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleSetpBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleDiagBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Health and Diagnostics Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a Health and Diagnostics Buffer message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             bufid - the buffer ID
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfo - the buffer information bytes (not used)
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt(), AssembleAck()
//
//  EXECUTION TIME:     
//
//------------------------------------------------------------------------------------------------------------

void AssembleDiagBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr, uint16_t bufinfo)
{
  union FLOAT_UINT32
  {
    float fval;
    uint32_t uval;
  } temp;

  // Common to all of the health and diagnostics response buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = cmnd;                    // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_DIAG;         // Buffer type = Health and Diagnostics
  DPComm.TxBuf[6] = (uint8_t)bufid;          // Buffer ID least significant byte
  DPComm.TxBuf[7] = (uint8_t)(bufid >> 8);   // Buffer ID most significant byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                       // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;

  switch (bufid)
  {
    case 0: //Test Injection Status and Result
      DPComm.TxBuf[8] = 13;                                           // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                                            // Buffer length most significant byte
      DPComm.TxBuf[10] = TestInjVars.Status;                          // Test status
      DPComm.TxBuf[11] = (uint8_t)TestInjVars.TestTime;               // Test time
      DPComm.TxBuf[12] = (uint8_t)(TestInjVars.TestTime >> 8);
      DPComm.TxBuf[13] = (uint8_t)(TestInjVars.TestTime >> 16);
      DPComm.TxBuf[14] = (uint8_t)(TestInjVars.TestTime >> 24);
      DPComm.TxBuf[15] = (uint8_t)TestInjVars.PriSecCos;              // Pri/Sec/Cause status
      DPComm.TxBuf[16] = (uint8_t)(TestInjVars.PriSecCos >> 8);
      DPComm.TxBuf[17] = (uint8_t)(TestInjVars.PriSecCos >> 16);
      DPComm.TxBuf[18] = (uint8_t)(TestInjVars.PriSecCos >> 24);
      DPComm.TxBuf[19] = (uint8_t)TestInjVars.Current;                // Trip current
      DPComm.TxBuf[20] = (uint8_t)(TestInjVars.Current >> 8);
      DPComm.TxBuf[21] = (uint8_t)(TestInjVars.Current >> 16);
      DPComm.TxBuf[22] = (uint8_t)(TestInjVars.Current >> 24);

      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 21, &DPComm, TRUE);
   break;
   case 1:  //Passsword Status

      //Read pswd from FRAM and send
      DPComm.TxBuf[8] = 9;                                           // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                                            // Buffer length most significant byte

      //Read PSWD status from FRAM and fill DPComm.TxBuff
      //include other PSWD infor (User Verified? Admin Verified? Restriction timer on?)
      FRAM_Read( PSWD_ADDRESS , 5, (uint16_t *)(&DPComm.TxBuf[10]) );              
     
      DPComm.TxBuf[18] |= (User_Verified_Tmr) ? (BIT2) : (0); 
      DPComm.TxBuf[18] |= (Admin_Verified_Tmr) ? (BIT3) : (0); 
      DPComm.TxBuf[18] |= (Pswd_Rejection_Tmr) ? (BIT4) : (0); 

      AssembleTxPkt(&DPComm.TxBuf[2], 17, &DPComm, TRUE);
   break;

   case 2: //Coil Open Detection Status and Result
      DPComm.TxBuf[8] = 2;                                           // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                                           // Buffer length most significant byte
      DPComm.TxBuf[10] = CoilDetect.TestInProgress;                  // Test status
      DPComm.TxBuf[11] = CoilDetect.Result;                          // Test result

      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
    break;

   case 3: //Read Flags
      DPComm.TxBuf[8] = 18;                                          // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                                           // Buffer length most significant byte
      DPComm.TxBuf[10] = (uint8_t)Trip_Flags0.all;
      DPComm.TxBuf[11] = (uint8_t)(Trip_Flags0.all >> 8);
      DPComm.TxBuf[12] = (uint8_t)Trip_Flags1.all;
      DPComm.TxBuf[13] = (uint8_t)(Trip_Flags1.all >> 8);
      DPComm.TxBuf[14] = (uint8_t)Alarm_Flags0.all;
      DPComm.TxBuf[15] = (uint8_t)(Alarm_Flags0.all >> 8);
      DPComm.TxBuf[16] = (uint8_t)Alarm_Flags1.all;
      DPComm.TxBuf[17] = (uint8_t)(Alarm_Flags1.all >> 8);
      DPComm.TxBuf[18] = (uint8_t)Alarm_Flags2.all;
      DPComm.TxBuf[19] = (uint8_t)(Alarm_Flags2.all >> 8);
      DPComm.TxBuf[20] = (uint8_t)Flags0.all;
      DPComm.TxBuf[21] = (uint8_t)(Flags0.all >> 8);
      DPComm.TxBuf[22] = (uint8_t)Flags1.all;
      DPComm.TxBuf[23] = (uint8_t)(Flags1.all >> 8);
      DPComm.TxBuf[24] = (uint8_t)Flags2.all;
      DPComm.TxBuf[25] = (uint8_t)(Flags2.all >> 8);
      DPComm.TxBuf[26] = (uint8_t)TripPuFlags.all;
      DPComm.TxBuf[27] = (uint8_t)(TripPuFlags.all >> 8);

      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 26, &DPComm, TRUE);
   break;

   case 4: //Read Internal Time Buffers
      DPComm.TxBuf[8] = 9;                                           // Buffer length least significant byte
      DPComm.TxBuf[9] = 0;                                           // Buffer length most significant byte
      DPComm.TxBuf[10] = Reset_to_PLL_Count;
      temp.fval = StartupTime.Time;
      DPComm.TxBuf[11] = (uint8_t)(temp.uval);
      DPComm.TxBuf[12] = (uint8_t)(temp.uval >> 8);
      DPComm.TxBuf[13] = (uint8_t)(temp.uval >> 16);
      DPComm.TxBuf[14] = (uint8_t)(temp.uval >> 24);
      DPComm.TxBuf[15] = (uint8_t)(maxlooptime);
      DPComm.TxBuf[16] = (uint8_t)(maxlooptime >> 8);
      DPComm.TxBuf[17] = (uint8_t)(maxlooptime >> 16);
      DPComm.TxBuf[18] = (uint8_t)(maxlooptime >> 24);

      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 17, &DPComm, TRUE);
   break;

   default:
      DPComm.AckNak = DP_NAK_BUFINVALID;
      AssembleAck();
   break;
      
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleDiagBuffer()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        ContinousDPcommBITSHIFT()
//------------------------------------------------------------------------------------------------------------
//  FUNCTION:           Used to Perform a Bitshift for individual data, no longer needed.
//                      Now is used to move data from SPI2_buf, that was filled during checksum read
//  MECHANICS:          Uses the SPI2_buf data from a read function to fill in the DP.CommTxBuf
//                      Starts at DPComm.Tx index 10 because 0-9 are not data, and are prefilled
//  INPUTS:             length- length of the data being read
//  OUTPUTS:            DPComm.TxBuf
//  CALLS:              AssembleFactoryBuffer()

void ContinousDPcommBITSHIFT(uint16_t start, uint32_t length)
{
  int eod_Length, i;

  DPComm.TxBuf[8] = length; // Max 256, Buffer Length Least Significant Byte
  DPComm.TxBuf[9] = 0;      // Buffer Length Most Significant Byte
  eod_Length = length +10;
  //int shift = 0;
  for(i = 10; i < eod_Length; i++)
  {
    DPComm.TxBuf[i] = SPI2_buf[start++]; // Data Byte
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          ContinousDPcommBITSHIFT()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleFactoryBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Factory Command Response Subroutine
//
//  MECHANICS:          This subroutine assembles a Factory Command Response message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             bufid - the buffer ID
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt(), AssembleAck()
//
//  EXECUTION TIME:     ????
//
//------------------------------------------------------------------------------------------------------------

void AssembleFactoryBuffer(uint16_t bufid, uint8_t cmnd, uint8_t addr)
{
  uint8_t i, ok, set;
  uint16_t length_cat, sum, sumrd, k;
  int first_addressofCAT;

  // Common to all of the factory buffers
  DPComm.TxBuf[0] = START_OF_PKT;            // Packet start
                                             // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = cmnd;                    // Command
  DPComm.TxBuf[3] = addr;                    // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;           // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_FACTORY;      // Buffer type = Factory
  DPComm.TxBuf[6] = (uint8_t)bufid;          // Buffer ID least significant byte
  DPComm.TxBuf[7] = (uint8_t)(bufid >> 8);   // Buffer ID most significant byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                       // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;

  // The data retrieval depends primarily on the buffer id and the contents of FacBuf_Info[][].  The buffers
  //   listed individually do not use FacBuf_Info[][]
  switch (bufid)
  {
    case 0:                             // Frame Rating
      SPI2_buf[0] = (uint8_t)Break_Config.config.Rating;
      SPI2_buf[1] = (uint8_t)(Break_Config.config.Rating >> 8);
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 1:                             // Frame
      SPI2_buf[0] = (uint8_t)Break_Config.config.BreakerFrame;
      SPI2_buf[1] = (uint8_t)(Break_Config.config.BreakerFrame >> 8);
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 15:                            // Rogowski offset - invalid
    case 17:                            // Not used - invalid
    case 40:                            // Neutral CT cal constants - invalid
      DPComm.AckNak = DP_NAK_BUFINVALID;
      AssembleAck();
      break;
    case 19:                            // ETU AFE phase shift
      // Set request to read cal constants into SPI2_buf[] and wait for it to finish.  If we are messing
      //   with the cal constants nothing else should be going on with the Flash chip, except for maybe
      //   demand logging, so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_AFECAL_RD1;                               // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_AFECAL_RD1) != S1F_AFECAL_RD1)      // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD1);                  // Clear the request flag
      DPComm.TxBuf[8] = 12;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      DPComm.TxBuf[10] = SPI2_buf[86];              // Phase shift Pha, 50Hz (current)
      DPComm.TxBuf[11] = 0;                         // High byte = 0
      DPComm.TxBuf[12] = SPI2_buf[87];              // Phase shift Phb, 50Hz (current)
      DPComm.TxBuf[13] = 0;                         // High byte = 0
      DPComm.TxBuf[14] = SPI2_buf[88];              // Phase shift Phc, 50Hz (current)
      DPComm.TxBuf[15] = 0;                         // High byte = 0
      DPComm.TxBuf[16] = SPI2_buf[80];              // Phase shift Pha, 60Hz (current)
      DPComm.TxBuf[17] = 0;                         // High byte = 0
      DPComm.TxBuf[18] = SPI2_buf[81];              // Phase shift Phb, 60Hz (current)
      DPComm.TxBuf[19] = 0;                         // High byte = 0
      DPComm.TxBuf[20] = SPI2_buf[82];              // Phase shift Phc, 60Hz (current)
      DPComm.TxBuf[21] = 0;                         // High byte = 0
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 20, &DPComm, TRUE);
      break;
    case 18:                            // ETU AFE voltage cal constants
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_AFECAL_RD1;                              // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_AFECAL_RD1) != S1F_AFECAL_RD1)    // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD1);                 // Clear the request flag
      DPComm.TxBuf[8] = 24;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      for(i=10; i<22; i++)                      // Gains
      {
        DPComm.TxBuf[i] = SPI2_buf[i+10];           // Gains start at index=20 in SPI2_buf[]
      }
      for(i=22; i<34; i++)                      // Offsets
      {
        DPComm.TxBuf[i] = SPI2_buf[i+38];           // Offsets start at index=60 in SPI2_buf[]
      }
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 32, &DPComm, TRUE);
      break;
    case 20:                            // ETU ADC voltage cal constants
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_ADCLCAL_RD1;                               // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_ADCLCAL_RD1) != S1F_ADCLCAL_RD1)    // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_ADCLCAL_RD1);                  // Clear the request flag
      DPComm.TxBuf[8] = 24;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      for(i=10; i<22; i++)                      // Gains
      {
        DPComm.TxBuf[i] = SPI2_buf[i+10];           // Gains start at index=20 in SPI2_buf[]
      }
      for(i=22; i<34; i++)                      // Offsets
      {
        DPComm.TxBuf[i] = SPI2_buf[i+30];           // Offsets start at index=52 in SPI2_buf[]
      }
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 32, &DPComm, TRUE);
      break;
    case 21:                            // SG cal constants
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_AFECAL_RD1;                               // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_AFECAL_RD1) != S1F_AFECAL_RD1)     // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD1);                  // Clear the request flag
      DPComm.TxBuf[8] = 16;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      for(i=0; i<4; i++)
      {
        DPComm.TxBuf[i+10] = SPI2_buf[i+16];        // Gain for CT
        DPComm.TxBuf[i+14] = SPI2_buf[i+56];        // Offset for CT
        DPComm.TxBuf[i+18] = SPI2_buf[i+32];        // Gain for Rogowski
        DPComm.TxBuf[i+22] = SPI2_buf[i+72];        // Offset for rogowski
      }
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 24, &DPComm, TRUE);
      break;
    case 22:                            // Style
    case 41:                            // Style 2
      k = (bufid == DP_FAC_BID_STYLE) ? (Setpoints0.stp.Style) : (Setpoints0.stp.Style_2);
      SPI2_buf[0] = (uint8_t)k;
      SPI2_buf[1] = (uint8_t)(k >> 8);
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 35:                            // ETU AFE current cal constants
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      SPI1Flash.Req |= S1F_AFECAL_RD1;                               // Set request to read cal constants
      while ((SPI1Flash.Ack & S1F_AFECAL_RD1) != S1F_AFECAL_RD1)     // Wait for it to finish
      {
      }
      SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD1);                  // Clear the request flag
      DPComm.TxBuf[8] = 40;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      for(i=0; i<16; i++)                      // Gains
      {
        DPComm.TxBuf[i+10] = SPI2_buf[i];           // Data starts at index = 10 in TxBuf[]
      }                                             // Ia - In Rogo gains start at index=0 in SPI2_buf[]
      for(i=26; i<30; i++)
      {
        DPComm.TxBuf[i] = SPI2_buf[i+10];           // Data starts at index = 26 in TxBuf[]
      }                                             // In CT gain starts at index=36 in SPI2_buf[]
      for(i=30; i<46; i++)                     // Offsets
      {
        DPComm.TxBuf[i] = SPI2_buf[i+10];           // Data starts at index = 30 in TxBuf[]
      }                                             // Ia - In Rogo offsets start at index=40 in SPI2_buf[]
      for(i=46; i<50; i++)
      {
        DPComm.TxBuf[i] = SPI2_buf[i+30];           // Data starts at index = 76 in TxBuf[]
      }                                             // In CT offset starts at index=46 in SPI2_buf[]
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 48, &DPComm, TRUE);
      break;
    case 36:                            // ETU Low-Gain ADC current cal constants
    case 37:                            // ETU High-Gain ADC current cal constants
      // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
      //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
      //   so we should be serviced almost immediately
      if (bufid == 36)
      {
        SPI1Flash.Req |= S1F_ADCLCAL_RD1;                               // Set request to read cal constants
        while ((SPI1Flash.Ack & S1F_ADCLCAL_RD1) != S1F_ADCLCAL_RD1)    // Wait for it to finish
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_ADCLCAL_RD1);                  // Clear the request flag
      }
      else
      {
        SPI1Flash.Req |= S1F_ADCHCAL_RD1;
        while ((SPI1Flash.Ack & S1F_ADCHCAL_RD1) != S1F_ADCHCAL_RD1)
        {
        }
        SPI1Flash.Req &= (uint32_t)(~S1F_ADCHCAL_RD1);
      }
      DPComm.TxBuf[8] = 40;                         // Length
      DPComm.TxBuf[9] = 0;                          // Reserved
      for(i=0; i<20; i++)                      // Gains
      {
        DPComm.TxBuf[i+10] = SPI2_buf[i];           // Data starts at index = 10 in TxBuf[]
      }                                             // Ia - In CT gains start at index=0 in SPI2_buf[]
      for(i=30; i<50; i++)                     // Offsets
      {
        DPComm.TxBuf[i] = SPI2_buf[i+2];            // Data starts at index = 30 in TxBuf[]
      }                                             // Ia - In Rogo offsets start at index=32 in SPI2_buf[]
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 48, &DPComm, TRUE);
      break;
    case 42:                            // Nonvolatile Memory
      SPI2_buf[0] =  Flash_ID.by[0];
      SPI2_buf[1] =  Flash_ID.by[1];
      SPI2_buf[2] =  Flash_ID.by[2];
      SPI2_buf[3] =  Flash_ID.by[3];
      FRAM_Read(FRAM_TEST_VAL, 1, (uint16_t *)(&SPI2_buf[4]));
      ExtCapt_FRAM_Read(EXTCAP_TEST_START, 1, (uint16_t *)(&SPI2_buf[6]));
      ContinousDPcommBITSHIFT(0, 8);        // starting index = 0, length = 8
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 16, &DPComm, TRUE);
      break;
    case 43:                            // Standard
      SPI2_buf[0] = (uint8_t)Break_Config.config.Standard;
      SPI2_buf[1] = (uint8_t)(Break_Config.config.Standard >> 8);
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 44:                            // Max Inst Trip Setting
      SPI2_buf[0] = (uint8_t)Break_Config.config.MaxInstTripSetting;
      SPI2_buf[1] = (uint8_t)(Break_Config.config.MaxInstTripSetting >> 8);
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 45:                            // Buf I/O (bufid == DP_FAC_BID_BIO)
      SPI2_buf[0] = 0;
      SPI2_buf[0] |= (DIO1_IN_HIGH) ? (BIT1) : (0);
      SPI2_buf[0] |= (DIO2_IN_HIGH) ? (BIT2) : (0);
      SPI2_buf[1] = 0;                      // reserved
      ContinousDPcommBITSHIFT(0, 2);        // starting index = 0, length = 2
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 10, &DPComm, TRUE);
      break;
    case 99:                            // FW Version
      SPI2_buf[0] = PROT_PROC_FW_VER;
      SPI2_buf[1] = PROT_PROC_FW_REV;
      SPI2_buf[2] = (uint8_t) PROT_PROC_FW_BUILD;
      SPI2_buf[3] = (uint8_t)(PROT_PROC_FW_BUILD >> 8);
      SPI2_buf[4] = Ovr_FW_Version;
      ContinousDPcommBITSHIFT(0, 5);        // starting index = 0, length = 5
      // Call subroutine to assemble the packet per the modified COBS algorithm
      // Note, it is ok to use TxBuf[] as both the source and dest because the length is less than 126
      AssembleTxPkt(&DPComm.TxBuf[2], 13, &DPComm, TRUE);
      break;
    default:                            // All others - these use FacBuf_Info[][]
      if (bufid <= LAST_FAC_BUFID)      //   buffers 2 - 14, 16, 23 - 34, 38 - 39
      {
        //----------------- First retrieve the existing info.  Store in SPI2_buf[] -----------------------------
        length_cat = FacBuf_Info[bufid][5];
        first_addressofCAT = FacBuf_Info[bufid][3];

        ok = TRUE;                          // Assume data is ok
        // If retrieving existing info from FRAM...
        if ( (FacBuf_Info[bufid][2] == 0) || (FacBuf_Info[bufid][2] == 1) )
        {
          ok = FALSE;                           // Assume checksums are bad
          // "set" is used as a temporary to hold the number of copies of the parameters.  Buffers 27, 33,
          //   and 34 only have one copy.  The rest have 3 copies
          if ( (bufid == 27) || (bufid == 33) || (bufid == 34) )
          {
            set = 1;
          }
          else
          {
            set = 3;
          }
          for (i = 0; i < set; ++i)             //   "set" is used as a temporary
          {
            // 0x10000 offsets are to make the FRAM protection addresses smaller in the table
            if(FacBuf_Info[bufid][2] == 0)
            {
              FRAM_Read(((first_addressofCAT + 0x10000) + (i * (length_cat+4))), length_cat, (uint16_t*)&SPI2_buf[0]);
            }
            else
            {
              Frame_FRAM_Read((first_addressofCAT + (i * (length_cat+4))), length_cat, &SPI2_buf[0]);
            }
            // If the checksum is good, done.  "k" is used as a temporary to hold the checksum complement
            sum = Checksum8_16((uint8_t *)(&SPI2_buf[0]), (length_cat-4));
            sumrd = SPI2_buf[length_cat-4] + (((uint16_t)SPI2_buf[length_cat-3]) << 8);
            k = SPI2_buf[length_cat-2] + (((uint16_t)SPI2_buf[length_cat-1]) << 8);
            if ( (sum == sumrd) && (sum == (k ^ 0xFFFF)) )
            {
              ok = TRUE;
              break;
            }
          }
        }
        // If we are reading on-board AFE cal constants - need to set request to read to get from either
        //   FRAM or Flash
        else if (FacBuf_Info[bufid][2] == 4)
        {
          // Set request to read cal constants and wait for it to finish.  If we are messing with the cal
          //   constants nothing else should be going on with the Flash chip, except for maybe demand logging,
          //   so we should be serviced almost immediately
          SPI1Flash.Req |= S1F_AFECAL_RD1;                               // Set request to read cal constants
          while ((SPI1Flash.Ack & S1F_AFECAL_RD1) != S1F_AFECAL_RD1)      // Wait for it to finish
          {
          }
          SPI1Flash.Req &= (uint32_t)(~S1F_AFECAL_RD1);                  // Clear the request flag
          // SPI2_buf[] has the cal constants.  The subroutine takes care of whether they are defaults, etc.,
          //   so set ok to True
          ok = TRUE;
        }
        // If the data in FRAM is bad, either set the values to the defined default values or to 0, if there are
        //   no defined defaults.  The new configuration info should be written anyway
        if (!ok)
        {
          if (bufid  == 16)
          {
            for (i = 0; i < length_cat; ++i)
            {
              SPI2_buf[i] = ROGO_DEFAULT_IGAIN;
            }
          }
          else if (bufid == 29)
          {
            set = 0;                            // "set" is used as a temporary
            for (i = 0; i < 3; ++i)
            {
              SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_VGAIN;
              SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_VGAIN >> 8);
            }
            for (i = 0; i < 3; ++i)
            {
              SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_VOFFSET;
              SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_VOFFSET >> 8);
            }
            for (i = 0; i < 6; ++i)
            {
              SPI2_buf[set++] = (uint8_t)VDB_DEFAULT_PHASE;
              SPI2_buf[set++] = (uint8_t)(VDB_DEFAULT_PHASE >> 8);
            }
          }
          else
          {
            for (k = 0; k < length_cat; ++k)
            {
              SPI2_buf[k] = 0;
            }
          }
        }
        // If the buffer = 9, the values retrieved are also in the critical breaker config section, so
        //   overwrite the values with these
        if (bufid == 9)
        {
          SPI2_buf[0] = (uint8_t)Break_Config.config.Poles;
          SPI2_buf[1] = (uint8_t)(Break_Config.config.Poles >> 8);
          SPI2_buf[2] = (uint8_t)Break_Config.config.Standard;
          SPI2_buf[3] = (uint8_t)(Break_Config.config.Standard >> 8);
          SPI2_buf[22] = (uint8_t)Break_Config.config.OvrWithstand;
          SPI2_buf[23] = (uint8_t)(Break_Config.config.OvrWithstand >> 8);
          SPI2_buf[24] = (uint8_t)Break_Config.config.MCR;
          SPI2_buf[25] = (uint8_t)(Break_Config.config.MCR >> 8);
          SPI2_buf[32] = (uint8_t)Break_Config.config.MaxInstTripSetting;
          SPI2_buf[33] = (uint8_t)(Break_Config.config.MaxInstTripSetting >> 8);
        }
        //----------------- End retrieve the existing info.  Store in SPI2_buf[] -------------------------------

        ContinousDPcommBITSHIFT(FacBuf_Info[bufid][4], FacBuf_Info[bufid][1]);
        // Call subroutine to assemble the packet per the modified COBS algorithm
        // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
        //   length = 8 + FacBuf_Info[bufid][1]: data length + 8 header bytes
        AssembleTxPkt(&DPComm.TxBuf[2], (FacBuf_Info[bufid][1]+8), &DPComm, TRUE);
      }
      else
      {
          DPComm.AckNak = DP_NAK_BUFINVALID;
          AssembleAck();
      }
      break;
  }
    
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleFactoryBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleEvntBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Events Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles an Events Buffer message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             msglen - the length of the input message
//                      bufid - the buffer ID
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC, 
//                      DPComm.AckNak
//
//  ALTERS:             None
//
//  CALLS:              GetEVInfo(), AssembleTxPkt(), AssembleAck(), AssembleSummaryEvntTxBuffer(),
//                      AssembleProfileEvntTxBuffer()
//
//  EXECUTION TIME:     ????
//
//------------------------------------------------------------------------------------------------------------

void AssembleEvntBuffer(uint16_t msglen, uint16_t bufid, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr)
{
  uint8_t i;

  if (bufid == 0)                       // Buffer ID = 0: Event header info
  {                                     //   buffer length is 1, bufinfo contains the event type
    if (msglen == 1)
    {
      // Call subroutine to retrieve event header info
      //   Subroutine returns True if valid request; False otherwise
      //   If valid request: header info is returned as follows:
      //     SPI2_buf[0]     - Event Type
      //     SPI2_buf[1]     - Number of Events ls byte
      //     SPI2_buf[2]     - Number of Events ms byte
      //     SPI2_buf[3..6]  - Earliest Event ID (ls byte .. ms byte)
      //     SPI2_buf[7..10] - Latest Event ID (ls byte .. ms byte)
      //   If valid, assemble response
      if (GetEVInfo(bufinfoptr[0]))
      {
        DPComm.TxBuf[0] = START_OF_PKT;             // Start of message
        DPComm.TxBuf[1] = 0;                        // Segment code (filled in later)
        DPComm.TxBuf[2] = cmnd;                     // Command
        DPComm.TxBuf[3] = addr;                     // Source and destination address
        DPComm.TxBuf[4] = DPComm.SeqNum;            // Sequence number
        DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;         // Type
        DPComm.TxBuf[6] = 0;                        // Buffer ID LS byte
        DPComm.TxBuf[7] = 0;                        // Buffer ID MS byte
        DPComm.TxBuf[8] = 11;                       // Length LS byte
        DPComm.TxBuf[9] = 0;                        // Length MS byte
        for (i=0; i<11; ++i)                           // Transfer the data from SPI2_buf[] to the transmit
        {                                              //   buffer
          DPComm.TxBuf[i+10] = SPI2_buf[i];
        }
        
        // Initialize variables used to assemble the packet per the modified COBS algorithm
        DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
        DPComm.TxSegCharCnt = 0;
        DPComm.TxNdx = 2;
        DPComm.TxCRC = 0xFFFF;

        // Call subroutine to assemble the message per the modified COBS algorithm
        // Note, it is ok to use TxBuf[] as both the source and destination because the length is less
        //   than 126
        AssembleTxPkt(&DPComm.TxBuf[2], 19, &DPComm, TRUE);
      }
      else                                        // If invalid request, NAK
      {
        DPComm.AckNak = DP_NAK_CMDINVALID;
        AssembleAck();
      }
    }
    else                                    // If invalid message length, NAK
    {
      DPComm.AckNak = DP_NAK_CMDINVALID;
      AssembleAck();
    }
  }
  else if ((8 <= bufid) && (bufid <= 10))   // Buffer ID = 8, 9, 10: Waveform EIDs
  {
    AssembleWFCapEIDsTxBuffer(bufid, cmnd, addr);
  }
  else if (bufid == DP_EVENT_SUMMARY)   // Buffer ID = DP_EVENT_SUMMARY: Summary events
  {
    AssembleSummaryEvntTxBuffer(msglen, cmnd, addr, bufinfoptr);
  }
  // DP_EVENT_TRIP_SNAP <= Buf ID <= DP_EVENT_EXTCAP_SNAP: Trip, Test Trip, Alarm, or Ext Cap snapshot data
  //   len must be 5: retrieval by EID only
  else if ( (bufid >= DP_EVENT_TRIP_SNAP) && (bufid <= DP_EVENT_EXTCAP_SNAP) && (msglen == 5) )
  {
    AssembleSnapshotTxBuffer(cmnd, addr, bufinfoptr, bufid);
  }
  // DP_EVENT_TRIP_SUM <= Buffer ID <= DP_EVENT_EXTCAP_SUM: Trip, Test Trip, Alarm, or Ext Cap summary data
  //   len must be 3: retrieval by index only
  else if ( (bufid >= DP_EVENT_TRIP_SUM) && (bufid <= DP_EVENT_EXTCAP_SUM) && (msglen == 3) )
  {
    AssembleSnapshotSummaryTxBuffer(cmnd, addr, bufinfoptr, bufid);
  }
  else if ( (bufid >= 97) && (bufid <= 118) )     // 97 <= Buffer ID <= 118: 6sec/60sec RMS captures
  {
    AssembleProfileEvntTxBuffer(bufid, cmnd, addr, bufinfoptr);
  }
  else if (bufid == 128)                // Buffer ID = 128: Disturbance capture events
  {
    AssembleDistCapTxBuffer(msglen, cmnd, addr, bufinfoptr);
  }
  else                                  // Buffer ID != 0, invalid request
  {
    DPComm.AckNak = DP_NAK_CMDINVALID;
    AssembleAck();
  }
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleEvntBuffer()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleSummaryEvntTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Summary Events Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a Summary Events Buffer message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             msglen - the length of the input message
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              GetEVInfo(), EventSummaryRead(), AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleSummaryEvntTxBuffer(uint16_t msglen, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr)
{
  uint8_t i, j, numlogs;
  uint16_t dest_ndx, src_ndx;

  // Command info
  DPComm.TxBuf[0] = START_OF_PKT;           // Start of message
  DPComm.TxBuf[1] = 0;                      // Segment code (filled in later)
  DPComm.TxBuf[2] = cmnd;                   // Command
  DPComm.TxBuf[3] = addr;                   // Source and destination address
  DPComm.TxBuf[4] = DPComm.SeqNum;          // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;       // Type
  DPComm.TxBuf[6] = DP_EVENT_SUMMARY;       // Buffer ID LS byte
  DPComm.TxBuf[7] = 0;                      // Buffer ID MS byte

  // Buffer length is entered after the logs have been retrieved.  We do not know how many Summary logs are
  //   available
        
  // Put the earliest overall and latest overall EIDs into DPComm.TxBuf[17..10]
  //   SPI2_buf[3..6]  - Earliest Event ID (ls byte .. ms byte)
  //   SPI2_buf[7..10] - Latest Event ID (ls byte .. ms byte)
  GetEVInfo(EV_TYPE_SUMMARY);
  j = 3;
  for (i = 10; i < 18; i++)
  {
    DPComm.TxBuf[i] = SPI2_buf[j++];
  }

  // Retrieve the logs and associated log info.  We will only store the log info now
  //   numlogs: number of logs returned
  //   SPI2_buf[3..0]: previous EID (before the earliest one we are sending)
  //   SPI2_buf[7..4]: next EID (after the one we are sending)
    //       SPI2_buf[21..8]: event 0 (earliest event)
    //       SPI2_buf[35..22]: event 1
    //       ...
    //       SPI2_buf[(14*numlogs + 8)-1..(14*numlogs+8)-14]: event n (most recent event)
  // They must be transmitted latest to earliest, beginning at DPComm.TxBuf[29]
  EventSummaryRead(msglen, bufinfoptr, &numlogs);

  // Previous and next EIDs, at DPComm.TxBuf[25..18]
  j = 18;
  for (i = 0; i < 8; ++i)
  {
    DPComm.TxBuf[j++] = SPI2_buf[i];
  }
        
  DPComm.TxBuf[26] = numlogs;               // Number returned
  if (msglen == 3)                          // Starting index (if requested by index)
  {
    DPComm.TxBuf[27] = bufinfoptr[0];
    DPComm.TxBuf[28] = bufinfoptr[1];
  }
  else                                      // Mark starting index invalid if requested by EID
  {
    DPComm.TxBuf[27] = 0xFF;
    DPComm.TxBuf[28] = 0xFF;
  }

  // Compute and store the response length.  Use dest_ndx as a temporary
  dest_ndx = 19 + (14 * numlogs);
  DPComm.TxBuf[8] = (uint8_t)dest_ndx;          // Length LS byte
  DPComm.TxBuf[9] = (uint8_t)(dest_ndx >> 8);   // Length MS byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                      // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  
  //   They must be transmitted latest to earliest, beginning at DPComm.TxBuf[29]
  if  (numlogs > 0)
  {
    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 27, &DPComm, FALSE);

    // We need to reverse the events chronologically.  The events were retrieved earliest to latest,
    //   beginning at SPI2_buf[8].  Assuming 20 (the max) logs and SUMMARY_EVENT_SIZE = 14:
    //       SPI2_buf[21..8]: event 0 (earliest event)
    //       SPI2_buf[35..22]: event 1
    //       ...
    //       SPI2_buf[(14*numlogs + 8)-1..(14*numlogs+8)-14]: event n (most recent event)
    //   They must be transmitted latest to earliest
    src_ndx = (SUMMARY_EVENT_SIZE*numlogs+8)-SUMMARY_EVENT_SIZE;  // starting index of most recent log
    for (j = 0; j < numlogs; ++j)
    {
      // Call subroutine to assemble the message per the modified COBS algorithm
      if (j < (numlogs - 1))                // Still have more to insert
      {
        AssembleTxPkt(&SPI2_buf[src_ndx], SUMMARY_EVENT_SIZE, &DPComm, FALSE);
      }
      else                                  // Last one
      {
        AssembleTxPkt(&SPI2_buf[src_ndx], SUMMARY_EVENT_SIZE, &DPComm, TRUE);
      }
      src_ndx -= SUMMARY_EVENT_SIZE;
    }
  }
  else
  {
    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 27, &DPComm, TRUE);
  }
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleSummaryEvntTxBuffer()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleProfileEvntTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble RMS Profile Events Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a 6sec or 60sec RMS Profile Events Buffer message for
//                      Write Response (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             bufid - the requested buffer id
//                      msglen - the length of the input message
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              ExtCapt_FRAM_Read(), AssembleTxPkt(), AssembleAck()
//
//------------------------------------------------------------------------------------------------------------

void AssembleProfileEvntTxBuffer(uint8_t bufid, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr)
{
  uint8_t i, j;
  uint16_t dest_ndx, src_ndx;

  // Check the EID - there is only one capture at present
  ExtCapt_FRAM_Read(EXTCAP_EID_START, (sizeof(sExtCapState.EID) >> 1), (uint16_t *)(&SPI2_buf[0]));
  // May as well compare on a byte-by-byte basis
  if ( (SPI2_buf[0] == bufinfoptr[0]) && (SPI2_buf[1] == bufinfoptr[1])
    && (SPI2_buf[2] == bufinfoptr[2]) && (SPI2_buf[3] == bufinfoptr[3]) )
  {
    // EID matches - assemble the response
    // These values are common to both one-cyc and 200msec values
    DPComm.TxBuf[0] = START_OF_PKT;                 // Start of message
    DPComm.TxBuf[1] = 0;                            // Segment code (filled in later)
    DPComm.TxBuf[2] = cmnd;                         // Command
    DPComm.TxBuf[3] = addr;                         // Source and destination address
    DPComm.TxBuf[4] = DPComm.SeqNum;                // Sequence number
    DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;             // Type
    DPComm.TxBuf[6] = (uint8_t)bufid;               // Buffer ID LS byte
    DPComm.TxBuf[7] = (uint8_t)(bufid >> 8);        // Buffer ID MS byte
    DPComm.TxBuf[10] = bufinfoptr[0];               // EID LS byte
    DPComm.TxBuf[11] = bufinfoptr[1];
    DPComm.TxBuf[12] = bufinfoptr[2];
    DPComm.TxBuf[13] = bufinfoptr[3];               // EID MS byte
    DPComm.TxBuf[14] = bufinfoptr[4];               // Second or 10-sec number

    // These values depend on whether one-cyc or 200msec
    if (bufid <= 107)
    {
      // Retrieve the Timestamp and insert it into TxBuf[22..15]
      ExtCapt_FRAM_Read(EXTCAP_ONECYC_TS_START, 4, (uint16_t *)(&DPComm.TxBuf[15]));
      DPComm.TxBuf[23] = 60;                        // 60 one-cyc values in 1 sec
      j = 60;                                       // Save number of vals in temporary
      DPComm.TxBuf[8] = 254;                        // Length LS byte (14 + 60 * 4)
      DPComm.TxBuf[9] = 0;                          // Length MS byte
      // Assemble the starting address of the 6-sec values to read use src_ndx as address holder
      // Base address + 1-sec period offset + value offset
      // "* 60": 50 readings per 10sec period
      src_ndx = EXTCAP_ONECYCLE_LOG_START + (DPComm.TxBuf[14] * EXTCAP_ONECYCLE_SIZE * 60) 
                          + ((bufid - 97) << 2);
    }
    else
    {
      // Retrieve the Timestamp and insert it into TxBuf[22..15]
      ExtCapt_FRAM_Read(EXTCAP_TWOHUN_TS_START, 4, (uint16_t *)(&DPComm.TxBuf[15]));
      DPComm.TxBuf[23] = 50;                        // 50 200msec values in 10 sec
      j = 50;                                       // Save number of vals in temporary
      DPComm.TxBuf[8] = 214;                        // Length LS byte (14 + 50 * 4)
      DPComm.TxBuf[9] = 0;                          // Length MS byte
      // Assemble the starting address of the 60-sec values to read use src_ndx as address holder
      // Base address + 1-sec period offset + value offset
      // "* 50": 50 readings per 10sec period
      src_ndx = EXTCAP_TWOHUNDRCYCLE_LOG_START + (DPComm.TxBuf[14] * EXTCAP_ONECYCLE_SIZE *  50) 
                          + ((bufid - 108) << 2);
    }

    // Call subroutine to assemble this portion of the message per the modified COBS algorithm
    //   Note, we are ok using TxBuf[] as both the source and destination because we are not indexing up to
    //   126.  If indexing past 126, depending on the data content, we could overwrite the next value
    DPComm.TxSegNdx = 1;                            // Initialize vars used in the insertion
    DPComm.TxSegCharCnt = 0;
    DPComm.TxNdx = 2;
    DPComm.TxCRC = 0xFFFF;
    AssembleTxPkt(&DPComm.TxBuf[2], 22, &DPComm, FALSE);

    // Retrieve the values from the EC FRAM and store starting at SPI_Buf[24]
    dest_ndx = 24;                                  // Initialize destination index
    for (i = 0; i < j; ++i)
    {
      ExtCapt_FRAM_Read(src_ndx, 2, (uint16_t *)(&SPI2_buf[dest_ndx]));
      dest_ndx += 4;
      src_ndx += EXTCAP_ONECYCLE_SIZE;              // Inc address by the size of the value set
    }

    // Call subroutine to assemble the message per the modified COBS algorithm
    AssembleTxPkt(&SPI2_buf[24], (dest_ndx - 24), &DPComm, TRUE);
  }
  else                                              // If invalid message length, NAK
  {
    DPComm.AckNak = DP_NAK_CMDINVALID;
    AssembleAck();
  }

}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleProfileEvntTxBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleSnapshotSummaryTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Snapshot Summary Events Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a Snapshot Summary Events Buffer message for Write
//                      Response (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
//                      bid - buffer id (DP_EVENT_TRIP_SUM <= bid <= DP_EVENT_EXTCAP_SUM)
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              GetEVInfo(), EventSummaryRead(), AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleSnapshotSummaryTxBuffer(uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr, uint16_t bid)
{
  uint8_t numlogs, totlogs;
  uint16_t dlen_tot, dlen_logs;

  // Command info
  DPComm.TxBuf[0] = START_OF_PKT;           // Start of message
  DPComm.TxBuf[1] = 0;                      // Segment code (filled in later)
  DPComm.TxBuf[2] = cmnd;                   // Command
  DPComm.TxBuf[3] = addr;                   // Source and destination address
  DPComm.TxBuf[4] = DPComm.SeqNum;          // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;       // Type
  DPComm.TxBuf[6] = (uint8_t)bid;           // Buffer ID LS byte
  DPComm.TxBuf[7] = (uint8_t)(bid >> 8);    // Buffer ID MS byte

  // Buffer length is entered after the logs have been retrieved.  We do not know how many Alarm logs there
  //   are
        
  // Retrieve the snapshot summary info and store it
  //   totlogs: total number of snapshot events
  //   numlogs: number of events returned
  //   bufinfoptr[1..0]: starting index
    //       SPI2_buf[3..0]: EID0 (most recent event)
    //       SPI2_buf[11..4]: Timestamp0
    //       SPI2_buf[15..12]: Pri/Sec/Cause0
    //       SPI2_buf[17..16]: Summary Code0
    //       SPI2_buf[35..18]: event 1
    //       ...
    //       SPI2_buf[((18*numlogs)-1)..(18*(numlogs-1))]: event n
  // The total number of events, number of returned events, and starting index are returned in
  //   DPComm.TxBuf[13..10]
  // The snapshot summaries are returned latest to earliest, beginning at DPComm.TxBuf[14]
  EventSnapshotSummaryRead(bufinfoptr, &numlogs, &totlogs, bid);

  // Compute and store the response length.  Use dlen_tot as a temporary
  dlen_logs = (18 * numlogs);                   // Save the data length of the logs
  dlen_tot = 4 + dlen_logs;
  DPComm.TxBuf[8] = (uint8_t)dlen_tot;          // Length LS byte
  DPComm.TxBuf[9] = (uint8_t)(dlen_tot >> 8);   // Length MS byte

  DPComm.TxBuf[10] = totlogs;                   // Total number of snapshot events
  DPComm.TxBuf[11] = numlogs;                   // Number returned
  DPComm.TxBuf[12] = bufinfoptr[0];             // Starting index LS byte
  DPComm.TxBuf[13] = bufinfoptr[1];             // Starting index MS byte
        
  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                      // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  // Call subroutine to assemble the message per the modified COBS algorithm
  // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 12, &DPComm, FALSE);

  // Now insert the data.  Note, length may be greater than 126, but we are ok because the source is
  //   SPI2_buf[], not DPComm.TxBuf[]
  AssembleTxPkt(&SPI2_buf[0], dlen_logs, &DPComm, TRUE);
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleSnapshotSummaryTxBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleSnapshotTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Snapshot Events Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles Snapshot Events Buffer message for Write Response
//                      (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
//                      bid - buffer id (DP_EVENT_TRIP_SUM <= bid <= DP_EVENT_EXTCAP_SUM)
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              GetEVInfo(), EventSummaryRead(), AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleSnapshotTxBuffer(uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr, uint16_t bid)
{
  // Command info
  DPComm.TxBuf[0] = START_OF_PKT;           // Start of message
  DPComm.TxBuf[1] = 0;                      // Segment code (filled in later)
  DPComm.TxBuf[2] = cmnd;                   // Command
  DPComm.TxBuf[3] = addr;                   // Source and destination address
  DPComm.TxBuf[4] = DPComm.SeqNum;          // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;       // Type
  DPComm.TxBuf[6] = (uint8_t)bid;           // Buffer ID LS byte
  DPComm.TxBuf[7] = (uint8_t)(bid >> 8);    // Buffer ID MS byte

  // Buffer length is entered after the logs have been retrieved.  We do not know how many Alarm logs there
  //   are
        
  // Retrieve the snapshot info and store it
  //   SPI2_buf[0]: number of events returned (= 1 if EID found, = 0 if not found)
  //   SPI2_buf[4..1]: EID
  //   SPI2_buf[12..5]: Timestamp
  //   SPI2_buf[16..13]: Pri/Sec/Cause
  //   SPI2_buf[18..17]: Summary Code
  //   SPI2_buf[20..19]: TU Status (Binary Status)
  //   SPI2_buf[40..21]: Ia .. Ig
  //   SPI2_buf[64..41]: Van1 - Vcn1, Vab1 - Vca1
  //   SPI2_buf[88..65]: Van2 - Vcn2, Vab2 - Vca2
  //   SPI2_buf[100.89]: Ptot, RPtot, AppPtot
  //   SPI2_buf[116..101]: DmndIa - DmndIn
  //   SPI2_buf[128..117]: DmndPtot, DmndRPtot, DmndAppPtot
  //   SPI2_buf[132..149]: Temperature
  //   SPI2_buf[140..133]: f1, f2
  //   SPI2_buf[156..141]: PFa - PFc, PFtot
  //   SPI2_buf[172..157]: THDIa - THDIn
  //   SPI2_buf[196..173]: THDVan1 - THDVcn1, THDVab1 - THDVca1
  //   SPI2_buf[212..197]: CFIa - CFIn
  //   SPI2_buf[214..213]: Operations Count
  EventSnapshotRead(bufinfoptr, bid);

  // Store the response length
  DPComm.TxBuf[8] = 0xD7;               // Length (215) LS byte = 0xD7
  DPComm.TxBuf[9] = 0x00;               // Length (215) MS byte = 0

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                      // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  // Call subroutine to assemble the message per the modified COBS algorithm
  // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 8, &DPComm, FALSE);

  // Now insert the data.  Note, length may be greater than 126, but we are ok because the source is
  //   SPI2_buf[], not DPComm.TxBuf[]
  AssembleTxPkt(&SPI2_buf[0], 215, &DPComm, TRUE);
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleSnapshotTxBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleDistCapTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Disturbance Events Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles a Disturbance Capture Event Buffer message for Write
//                      Response (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             msglen - the length of the input message (determines whether to retrieve by index or
//                          by EID)
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      bufinfoptr - pointer to the buffer info
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              EventDisturbanceRead(), AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleDistCapTxBuffer(uint8_t msglen, uint8_t cmnd, uint8_t addr, uint8_t *bufinfoptr)
{
  uint8_t j, numlogs, totlogs;
  uint16_t src_ndx;

  // Command info
  DPComm.TxBuf[0] = START_OF_PKT;           // Start of message
  DPComm.TxBuf[1] = 0;                      // Segment code (filled in later)
  DPComm.TxBuf[2] = cmnd;                   // Command
  DPComm.TxBuf[3] = addr;                   // Source and destination address
  DPComm.TxBuf[4] = DPComm.SeqNum;          // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;       // Type
  DPComm.TxBuf[6] = 128;                    // Buffer ID LS byte
  DPComm.TxBuf[7] = 0;                      // Buffer ID MS byte

  // Buffer length is entered after the logs have been retrieved.  We do not know how many Disturbance Logs
  //   are available
        
  // Retrieve the logs and associated log info.  We will only store the log info now
  //   numlogs: number of logs returned.  log size is defined in DIST_EVENT_SIZE (44)
  //   SPI2_buf[8..0]: not used
  //   SPI2_buf[51..8]: earliest event
  //   SPI2_buf[95..52]: event 1
  //   ...
  //   SPI2_buf[(44*numlogs + 8)-1..(44*numlogs+8)-44]: event n (most recent event)
  // They must be transmitted latest to earliest, beginning at DPComm.TxBuf[29]
  EventDisturbanceRead(msglen, bufinfoptr, &numlogs, &totlogs);

  // Compute and store the response length.  Use src_ndx as a temporary
  src_ndx = 4 + (DIST_EVENT_SIZE * numlogs);
  DPComm.TxBuf[8] = (uint8_t)src_ndx;           // Length LS byte
  DPComm.TxBuf[9] = (uint8_t)(src_ndx >> 8);    // Length MS byte

  DPComm.TxBuf[10] = totlogs;                   // Total number of snapshot events
  DPComm.TxBuf[11] = numlogs;                   // Number returned
  DPComm.TxBuf[12] = bufinfoptr[0];             // Starting index LS byte
  DPComm.TxBuf[13] = bufinfoptr[1];             // Starting index MS byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  
  //   They must be transmitted latest to earliest, beginning at DPComm.TxBuf[29]
  if  (numlogs > 0)
  {
    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 12, &DPComm, FALSE);

    // We need to reverse the events chronologically.  The events were retrieved earliest to latest,
    //   beginning at SPI2_buf[8].  Assuming 5 (the max) logs and DIST_EVENT_SIZE = 44:
    //   SPI2_buf[51..8]: event 0 (the earliest event)
    //   SPI2_buf[95..52]: event 1
    //       ...
    //   SPI2_buf[(44*numlogs + 8)-1..(44*numlogs+8)-44]: event n (most recent event)
    //   They must be transmitted latest to earliest
    src_ndx = (DIST_EVENT_SIZE*numlogs+8)- DIST_EVENT_SIZE;     // Starting index of most recent log
    for (j = 0; j < numlogs; ++j)
    {
      // Call subroutine to assemble the message per the modified COBS algorithm
      if (j < (numlogs - 1))            // Still have more to insert
      {
        AssembleTxPkt(&SPI2_buf[src_ndx], (DIST_EVENT_SIZE), &DPComm, FALSE);
      }
      else                              // Last one
      {
        AssembleTxPkt(&SPI2_buf[src_ndx], (DIST_EVENT_SIZE), &DPComm, TRUE);
      }
      src_ndx -= DIST_EVENT_SIZE;
    }
  }
  else
  {
    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 12, &DPComm, TRUE);
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleDistCapTxBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleWFCapEIDsTxBuffer()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Waveform Capture Event IDs Transmit Buffer Subroutine
//
//  MECHANICS:          This subroutine assembles the list of EIDs for a requested Waveform Capture type
//                      (Trip, Alarm, Extended Capture) for Write Response (Command = 4) commands.
//
//  CAVEATS:            None
//
//  INPUTS:             bid - buffer id (DP_EVENT_TRIP_SUM <= bid <= DP_EVENT_EXTCAP_SUM)
//                      cmnd - the output command (for now, always a Write Response)
//                      addr - the Source and Destination Address byte
//                      DPComm.SeqNum - The Sequence Number received from the Display Processor
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC 
//
//  ALTERS:             SPI2_buf[]
//
//  CALLS:              EventSummaryRead(), AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleWFCapEIDsTxBuffer(uint16_t bid, uint8_t cmnd, uint8_t addr)
{
  uint8_t j, evtype, numlogs;

  // Command info
  DPComm.TxBuf[0] = START_OF_PKT;           // Start of message
  DPComm.TxBuf[1] = 0;                      // Segment code (filled in later)
  DPComm.TxBuf[2] = cmnd;                   // Command
  DPComm.TxBuf[3] = addr;                   // Source and destination address
  DPComm.TxBuf[4] = DPComm.SeqNum;          // Sequence number
  DPComm.TxBuf[5] = DP_BUFTYPE_EVENT;       // Type
  DPComm.TxBuf[6] = (uint8_t)bid;           // Buffer ID LS byte
  DPComm.TxBuf[7] = (uint8_t)(bid >> 8);    // Buffer ID MS byte

  // Number of waveform logs.  Event type is based on the buffer ID
  //   Note, clamp number of logs to maximum number - 1.  Don't count the "extra" log entry (the earliest
  //   entry), because it may be overwritten at any point - it is an open spot and has been erased
  evtype = (bid - 8) + EV_TYPE_TRIPWF;
  numlogs = ( (RAM_EV_ADD[evtype]->Num_Events >= MAX_NUM_EVENTS[evtype]) ?
                    (MAX_NUM_EVENTS[evtype] - 1) : (RAM_EV_ADD[evtype]->Num_Events) );
  DPComm.TxBuf[10] = numlogs;

  // Compute and store the response length.  Max value is 81.
  DPComm.TxBuf[8] = 1 + (4 * numlogs);      // Length LS byte
  DPComm.TxBuf[9] = 0;                      // Length MS byte

  // Initialize variables used to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  
  // Assemble the transmit buffer
  if  (numlogs > 0)
  {
    // Retrieve the EIDs for the requested waveform type
    //   SPI2_buf[3..0]: most recent event
    //   SPI2_buf[7..4]: event 1
    //   ...
    //   SPI2_buf[(4*numlogs - 1)..(4*numlogs - 4)]: event n (earliest event)
    EventGetWfEIDs(evtype, 0, &j);

    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 9, &DPComm, FALSE);

    AssembleTxPkt(&SPI2_buf[0], (numlogs * 4), &DPComm, TRUE);
  }
  else
  {
    // Call subroutine to assemble the message per the modified COBS algorithm
    // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
    AssembleTxPkt(&DPComm.TxBuf[2], 9, &DPComm, TRUE);
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleWFCapEIDsTxBuffer()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleAck()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Ack Message For Transmission
//
//  MECHANICS:          This subroutine assembles an ACK transmit message
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.Addr - The Source and Destination Address byte received from the Display
//                              Processor
//                      DPComm.SeqNum - Sequence Number
//                      DPComm.AckNak - Acknowledge Status
// 
//  OUTPUTS:            DPComm.TxBuf[], DPComm.TxSegNdx, DPComm.TxSegCharCnt, DPComm.TxNdx, DPComm.TxCRC
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleAck(void)
{
  // Assemble the ACK message.  Message consists of:
  //   Start Packet - 0x00
  //   Segment Code
  //   Message body:
  //      - Command = 0x0E: Acknowledge
  //      - Source/Destination Address = 0x23: Src Addr = 2, Dest addr = 3
  //      - Sequence Number = DPComm.SeqNum (must match incoming message)
  //      - Buffer Type = 0x00: Real-Time Data
  //      - Buffer ID = 0x0000
  //      - Buffer Length = 0x0001
  //      - Data bytes 0: Ack status
  //      - CRC LSB
  //      - CRC MSB
  //   Segment Code
  //   End Packet - 0x01
  DPComm.TxBuf[0] = START_OF_PKT;               // Packet start
                                                // Leave DPComm.TxBuf[1] open for first segment code
  DPComm.TxBuf[2] = 0x0E;                       // Command
  DPComm.TxBuf[3] = 0x50 | (DPComm.Addr >> 4);  // Source/Destination Address
  DPComm.TxBuf[4] = DPComm.SeqNum;              // Sequence number
  DPComm.TxBuf[5] = 0x00;                       // Buffer type
  DPComm.TxBuf[6] = 0x00;                       // Buffer ID least significant byte
  DPComm.TxBuf[7] = 0x00;                       // Buffer ID most significant byte
  DPComm.TxBuf[8] = 0x01;                       // Buffer length least significant byte
  DPComm.TxBuf[9] = 0x00;                       // Buffer length most significant byte
  DPComm.TxBuf[10] = DPComm.AckNak;             // Acknowledge status

  // Call subroutine to assemble the packet per the modified COBS algorithm
  DPComm.TxSegNdx = 1;                          // Initialize vars used in the insertion
  DPComm.TxSegCharCnt = 0;
  DPComm.TxNdx = 2;
  DPComm.TxCRC = 0xFFFF;
  // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt(&DPComm.TxBuf[2], 9, &DPComm, TRUE);
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleAck()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        AssembleAck1()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble Ack Message For Delayed Response Transmission
//
//  MECHANICS:          This subroutine assembles an ACK transmit message into SPI2_buf[]
//
//  CAVEATS:            None
//
//  INPUTS:             ack_val - the acknowledge response value
//                      DPComm.RxMsgSav[1]- Source/Destination Address
//                      DPComm.RxMsgSav[2]- Sequence Number
// 
//  OUTPUTS:            SPI2_buf[]
//
//  ALTERS:             None
//
//  CALLS:              AssembleTxPkt()
//
//------------------------------------------------------------------------------------------------------------

void AssembleAck1(uint8_t ack_val)
{
  // Assemble the ACK message.  Message consists of:
  //   Start Packet - 0x00
  //   Segment Code
  //   Message body:
  //      - Command = 0x0E: Acknowledge
  //      - Source/Destination Address = 0x23: Src Addr = 2, Dest addr = 3
  //      - Sequence Number = DPComm.RxMsgSav[2] (must match incoming message)
  //      - Buffer Type = 0x00: Real-Time Data
  //      - Buffer ID = 0x0000
  //      - Buffer Length = 0x0001
  //      - Data bytes 0: Ack status
  //      - CRC LSB
  //      - CRC MSB
  //   Segment Code
  //   End Packet - 0x01
  SPI2_buf[0] = START_OF_PKT;                       // Packet start
                                                    // Leave DPComm.TxBuf[1] open for first segment code
  SPI2_buf[2] = 0x0E;                               // Command
  SPI2_buf[3] = 0x50 | (DPComm.RxMsgSav[1] >> 4);   // Source/Destination Address
  SPI2_buf[4] = DPComm.RxMsgSav[2];                 // Sequence number
  SPI2_buf[5] = 0x00;                               // Buffer type
  SPI2_buf[6] = 0x00;                               // Buffer ID least significant byte
  SPI2_buf[7] = 0x00;                               // Buffer ID most significant byte
  SPI2_buf[8] = 0x01;                               // Buffer length least significant byte
  SPI2_buf[9] = 0x00;                               // Buffer length most significant byte
  SPI2_buf[10] = ack_val;                           // Acknowledge status
}            

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          AssembleAck1()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//                   High-Speed (GOOSE) Message Protocol
//------------------------------------------------------------------------------------------------------------
//
// The protocol is identical to the protocol for the normal communications described above at the beginning
// of this file, with one exception.  The sequence numbers are fixed and are identical to the type of GOOSE
// message:
//   0 = Trip, 1 = ZSI, 2 = ACK, 3 = Transfer, 4 = Settings Change, 5 = Health & State, 6 = Analog values
// The reason is that GOOSE messages are broadcast messages; there is no Ethernet port associated with them.
// If a GOOSE message is not ACK'ed in time, it will be repeated.  However, we want it to go out on the
// Ethernet only once, so an ACK of the message type is sufficient.
// The high-Speed comms link uses UART3.  It is dedicated to GOOSE communications.
//
//  Communications between the Protection Processor (this firmware) and the Display Processor are
//  full-duplex.  Sequence numbers and addressing are used to ensure the message responses are aligned to
//  the proper command.
//  The Display Processor sends the following commands to the Protection Processor:
//      Command = 6: Write Buffer With Acknowledge
//      Command = 14: Acknowledge
//
//  The Protection Processor sends the following commands to the Display Processor:
//      Command = 6: Write Buffer With Acknowledge
//      Command = 14: Acknowledge
//
//  Protection Processor Receptions
//  Message bytes are received into a circular buffer (RxBuf[ ]) via DMA.  The buffer is checked for new
//  characters in the DispComm61850 Rx subroutine, which is called in the sampling ISR
//  (DMA1_Stream0_IRQHandler()).  Received characters are processed and a completed message is stored in
//  separate buffers, RxMsg0[ ], RxMsg1[ ], ???. 
//
//  Messages received from the Display Processor are only received via GOOSE - it doesn't matter which
//  Ethernet port it came from, because we will only respond with an ACK.  The port originating the GOOSE
//  message does not get a response - GOOSE is a Publisher-Subscriber protocol.  Therefore, the Source
//  Address is always 9 (Ethernet #1).
//  The Protection Processor does not "care" about the Source Address.  It merely places this address in the
//  Destination Address field of the ACK response.
//  The Protection Processor uses one Source Address - 5.
//
//  Protection Processor Transmissions
//  Protection Processor transmissions are due to an event or other occurrence.  These will initiate a
//  GOOSE message transmission.  Therefore, they are always unsolicited writes (Write w/ Ack commands).  The
//  write is not to a specific Ethernet port, and so the Destination Address is always Ethernet #1.  The
//  Display Processor has a separate buffer for messages received over this UART (UART3).  The messages are
//  transmitted via UDP over Ethernet.
//  Transmissions are initiated in the DispComm61850 Tx subroutine, which is called sampling ISR
//  (DMA1_Stream0_IRQHandler()). 
//
//  DispComm61850_Tx()
//  This is the transmission handler.  This handler does the processing for transmission requests.  Requests
//  are generated by various subroutines in both the foreground and the background.  Requests are handled in
//  a similar fashion as events.  A GOOSE transmission request is described by a type and any associated
//  data that may be specific to the request and must be captured at the time of the request (such as the
//  protection type that caused pickup for a ZSI request).  Other data included in the message, such as the
//  timestamp, are added when the message is assembled.  There are 7 different request types:
//      1) Trip - this occurs when the PXR35 has tripped, and so is generated when the TA is fired
//      2) ZSI - this occurs when the PXR35 enters pickup, and so can be generated by any of the protection
//           subroutines (in both the foreground and the background).
//      3) Ack Response - this is in response to a received GOOSE message
//      4) Transfer
//      5) Setpoints
//      6) Health and State
//      7) Timed Analog Values
//  Trip requests are the highest priority, then ZSI requests, then ACKs, etc.
//
//  *** DAH  ADD A TEST WRITE TO ENSURE COMMUNICATIONS ARE WORKING.  OCCASIONALLY WRITE AND IF DON'T GET ACK,
//              RESET THE INTERFACE.  SIMILARLY, IF DISPLAY DOESN'T RECEIVE THIS COMMAND PERIODICALLY, IT
//              WILL RESET IT'S INTERFACE
//
//  DispComm61850_Rx()
//  This is the reception handler.  This handler does the processing for GOOSE receptions from the Display
//  processor.  The receptions are either Writes or ACKs.  Receptions are handled as follows:
//      - parse the input packet and place the received message in RxMsg0[], RxMsg1[], etc.
//      - process the message based on the command
//          - Write With Acknowledge (command = 6): this is generated for a GOOSE reception.  The message is
//            saved and acted upon in the affected subroutine (*** DAH TBD LEFT OFF HERE).
//          - Acknowledge (command = 15): This is a response to a GOOSE Write message that was sent by the
//            Protection Processor.  This basically closes out the message that was sent
//
//  As mentioned previously, GOOSE message receptions and transmissions are handled handled in subroutines
//  called from the sampling ISR.  This ISR occurs every 200usec.  This delay is considered acceptable, even
//  for a ZSI message. 
//
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DispComm61850_Tx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Processor Communications Top-Level Transmit Subroutine
//
//  MECHANICS:          This subroutine handles 61850 requests to transmit a GOOSE message to the Display
//                      Processor.  There are six sources of requests:
//                          0    Trip (Highest priority) - transmit a Trip message
//                          1    ZSI - transmit a ZSI message
//                          2    ACK - transmit an Ack of a GOOSE message received from the Display Proc
//                          3    Transfer - transmit a Transfer ZSI message
//                          4    Settings Change - transmit a Settings ZSI message
//                          5    Health & State - transmit a Breaker State ZSI message
//                          6    Analog values (Lowest priority) - transmit an Analog Values ZSI message
//                      Each request is either a Write With Acknowledge (x6) command or an ACK (xE) command
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.Flags, DPComm.RxMsg[], SPI2_buf[]
//
//  OUTPUTS:            DPComm.TxBuf[], DMA1_Stream6 register
//
//  ALTERS:             DPComm.TxState, DPComm.XmitReqClrMsk, DPComm.Flags
//
//  CALLS:              Assemble61850TripMsg(), Assemble61850ZSIMsg(), Assemble61850AckMsg(),
//                      Assemble61850XferMsg(), Assemble61850SetpMsg(), Assemble61850StateMsg(),
//                      Assemble61850ValsMsg()
//
//  EXECUTION TIME:     Measured execution time on ??? (Rev 00.?? code).
//                                ????usec
//
//------------------------------------------------------------------------------------------------------------

void DispComm61850_Tx(void)
{
  uint8_t i, ack_requestor;
  uint16_t bitmask;

  // Decrement timers waiting for an ACK from the display processor
  for (i=0; i<NUM_61850_REQUESTS; ++i)
  {
    if (DPComm61850.XmitWaitTimer[i] > 0)
    {
      --DPComm61850.XmitWaitTimer[i];
    }
  }

  // Only can transmit if xmitwait is zero.  Presently this introduces a 1-sample (207usec) delay between
  //   transmissions to give the display processor enough time to process the previous transmission
  if (xmitwait > 0)
  {
    --xmitwait;
    return;
  }


  // If transmitting, check whether we are finished.  We should be, as all messages should be completed in
  //   less than 200usec (the system is designed that way).  If we are not finished, abort the DMA operation
  //   and start over.
  if (DPComm61850.TxState == 1)
  {
    if (DMA1->LISR & DMA_LISR_TCIF3)          // If transmission has been completed, reset the transfer
    {                                         //   complete interrupt flag
      DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
    }
    else                                      // Otherwise error - abort the and reset the DMA channel.
    {                                         //   This should never happen
      DPComm61850.TxVars.TxSegNdx = 1;        // *** DAH  THIS STATEMENT IS JUST FOR A BREAKPOINT
    }                                         //          NEED TO ADD IN CODE TO ABORT DMA OPERATION AND RESET THE DMA
    DPComm61850.TxState = 0;                  // Either way, set state to 0
  }

  // At this point, we are not transmitting.  Step through the request buffers to see if there are any
  //   requests to transmit.  Note, the requests are prioritized:
  //      index  Message
  //        0    Trip                Highest priority
  //        1    ZSI
  //        2    ACK
  //        3    Transfer
  //        4    Settings Change
  //        5    Health & State
  //        6    Analog values
  //        7    Capture Enable
  //        8    Capture Disable     Lowest priority
  i = TRUE;                             // Use i as temporary flag - assume there is a message to transmit
  if ( (DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] > 0) && (DPComm61850.XmitWaitTimer[DP61850_TYPE_GOCB_STATUS_CTRL] == 0) )
  {
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
    TESTPIN_A3_LOW;
#endif
    Assemble61850XCBR_CB_Status_Ctrl_Msg();
    ddcntzsitx++;
    ddcntsoe[ddcntseqnum++] = 2;
    ddcntseqnum &= 0x7FF;
    DPComm61850.XmitWaitTimer[DP61850_TYPE_GOCB_STATUS_CTRL] = 10;
    DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = FALSE;             // *** DAH TEST  SET TO FALSE FOR NOW SO IF EMULATOR STOPS WILL NOT KEEP TRANSMITTING
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
    TESTPIN_A3_HIGH;
#endif
  }
  else if ((DPComm61850.Req[DP61850_TYPE_VALS] > 0) && (DPComm61850.XmitWaitTimer[DP61850_TYPE_VALS] == 0))
  {
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
    TESTPIN_A3_LOW;
#endif
    Assemble61850ValsMsg();
    DPComm61850.XmitWaitTimer[DP61850_TYPE_VALS] = 10;
    DPComm61850.Req[DP61850_TYPE_VALS] = FALSE;             // *** DAH TEST  SET TO FALSE FOR NOW SO IF EMULATOR STOPS WILL NOT KEEP TRANSMITTING
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
    TESTPIN_A3_HIGH;
#endif
  }
  else if ( (DPComm61850.Req[DP61850_TYPE_ACK] > 0) && (DPComm61850.XmitWaitTimer[DP61850_TYPE_ACK] == 0) )
  {
    // Get code requesting the ACK
    bitmask = 1;
    for (ack_requestor=0; ack_requestor<NUM_61850_REQUESTS; ++ack_requestor)
    {
      if ( (DPComm61850.Req[DP61850_TYPE_ACK] & bitmask) == bitmask)
      {
        break;
      }
      bitmask = (bitmask << 1);
    }
    // If code is ok, assemble ACK message and clear request flag
    if ( (ack_requestor < NUM_61850_REQUESTS) && (ack_requestor != DP61850_TYPE_ACK) )
    {
      Assemble61850AckMsg(ack_requestor);
      DPComm61850.Req[DP61850_TYPE_ACK] &= (bitmask ^ 0xFFFF);
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
      TESTPIN_D1_LOW;
#endif
    }
    // If code is not ok, this is an error that should never happen.  Correct the request byte and set flag
    //   to initiate transmission to False
    else
    {
      DPComm61850.Req[DP61850_TYPE_ACK] &= NUM_61850_REQMASK;
      i = FALSE;
    }
  }
  else                                  // If no message to send, done
  {
    i = FALSE;
  }
  if (i)
  {
//    TESTPIN_D1_TOGGLE;    // *** DAH TEST 230321
    // Message to send - set up the DMA and initiate transmissions
    DMA1_Stream3->NDTR &= 0xFFFF0000;          // Set up the DMA
    DMA1_Stream3->NDTR |= DPComm61850.TxVars.TxNdx;
    // Must clear all event flags before initiating a DMA operation
    DMA1->LIFCR |= (DMA_LIFCR_CTCIF3 + DMA_LIFCR_CHTIF3 + DMA_LIFCR_CTEIF3 + DMA_LIFCR_CDMEIF3
                        + DMA_LIFCR_CFEIF3);
    DMA1_Stream3->CR |= 0x00000001;            // Initiate the DMA to transmit the data
    DPComm61850.TxState = 1;                   // Set TxState to 1
    xmitwait = 1;                              // Set transmit delay to 1 sample
  }
  if (ddcntseqnum < 1)
  {
    ddintcnt = 0;
  }
  else if (ddcntseqnum < 250)
  {
    ++ddintcnt;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          DispComm61850_Tx()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Assemble61850XCBR_CB_Status_Ctrl_Msg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble 61850 GOOSE XCBR & control output Message
//
//  MECHANICS:          
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm61850.SeqNum
//
//  OUTPUTS:            
//
//  ALTERS:             
//
//  CALLS:              AssembleTxPkt()
//
//  EXECUTION TIME:     Measured execution time on ??? (Rev 00.?? code).
//                                ????usec
//
//------------------------------------------------------------------------------------------------------------

void Assemble61850XCBR_CB_Status_Ctrl_Msg(void)
{
  // Assemble a test message
  DPComm61850.TxVars.TxBuf[0] = START_OF_PKT;       // Packet start
                                                    // Leave DPComm.TxBuf[1] open for first segment code
  DPComm61850.TxVars.TxBuf[2] = DP_CMND_WRWACK;     // Command
  DPComm61850.TxVars.TxBuf[3] = 0x59;               // Src/Dest Address  (5 = Trip Unit, 9 = Ethernet #1)
  DPComm61850.TxVars.TxBuf[4] = DP61850_TYPE_GOCB_STATUS_CTRL;   // Sequence number
  DPComm61850.TxVars.TxBuf[5] = DP_BUFTYPE_GOOSE;   // Buffer type: GOOSE
  DPComm61850.TxVars.TxBuf[6] = DP61850_TYPE_GOCB_STATUS_CTRL;   // Buffer ID least significant byte
  DPComm61850.TxVars.TxBuf[7] = 0x00;               // Buffer ID most significant byte
  DPComm61850.TxVars.TxBuf[8] = sizeof(PXR35_CB_Publish_Data.CB_Status_Ctrl) - 2;                  // Buffer length least significant byte
  DPComm61850.TxVars.TxBuf[9] = 0;                  // Buffer length most significant byte

  // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
  DPComm61850.TxVars.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm61850.TxVars.TxSegCharCnt = 0;
  DPComm61850.TxVars.TxNdx = 2;
  DPComm61850.TxVars.TxCRC = 0xFFFF;
  // Note, it is ok to use TxBuf[] as both the source and destination because the total length (8 + 5*4) is
  //   less than 126
  AssembleTxPkt1(&DPComm61850.TxVars.TxBuf[2], 8, &DPComm61850.TxVars, FALSE);

  // Assemble test data portion of message
  AssembleTxPkt1((uint8_t *)(&PXR35_CB_Publish_Data.CB_Status_Ctrl.CB_Pos_Status), sizeof(PXR35_CB_Publish_Data.CB_Status_Ctrl) - 2, &DPComm61850.TxVars, TRUE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Assemble61850XCBR_CB_Status_Msg()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Assemble61850ValsMsg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble 61850 GOOSE Analog Values Message
//
//  MECHANICS:          
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm61850.SeqNum
//
//  OUTPUTS:            
//
//  ALTERS:             
//
//  CALLS:              AssembleTxPkt()
//
//  EXECUTION TIME:     Measured execution time on ??? (Rev 00.?? code).
//                                ????usec
//
//------------------------------------------------------------------------------------------------------------

void Assemble61850ValsMsg(void)
{
  // Assemble a test message
  DPComm61850.TxVars.TxBuf[0] = START_OF_PKT;       // Packet start
                                                    // Leave DPComm.TxBuf[1] open for first segment code
  DPComm61850.TxVars.TxBuf[2] = DP_CMND_WRWACK;     // Command
  DPComm61850.TxVars.TxBuf[3] = 0x59;               // Src/Dest Address  (5 = Trip Unit, 9 = Ethernet #1)
  DPComm61850.TxVars.TxBuf[4] = DP61850_TYPE_VALS;  // Sequence number
  DPComm61850.TxVars.TxBuf[5] = DP_BUFTYPE_GOOSE;   // Buffer type: GOOSE
  DPComm61850.TxVars.TxBuf[6] = DP61850_TYPE_VALS;  // Buffer ID least significant byte
  DPComm61850.TxVars.TxBuf[7] = 0x00;               // Buffer ID most significant byte
  DPComm61850.TxVars.TxBuf[8] = sizeof(PXR35_CB_Publish_Data.CB_Meter_values) - 2;                 // Buffer length least significant byte
  DPComm61850.TxVars.TxBuf[9] = 0;                  // Buffer length most significant byte

  // Call subroutine to assemble the packet per the modified COBS algorithm - header portion of msg
  DPComm61850.TxVars.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm61850.TxVars.TxSegCharCnt = 0;
  DPComm61850.TxVars.TxNdx = 2;
  DPComm61850.TxVars.TxCRC = 0xFFFF;
  // Note, it is ok to use TxBuf[] as both the source and destination because the total length (8 + 5*4) is
  //   less than 126
  AssembleTxPkt1(&DPComm61850.TxVars.TxBuf[2], 8, &DPComm61850.TxVars, FALSE);

  // Assemble test data portion of message
  AssembleTxPkt1((uint8_t *)(&PXR35_CB_Publish_Data.CB_Meter_values.Meter_Readings[0]), sizeof(PXR35_CB_Publish_Data.CB_Meter_values) - 2, &DPComm61850.TxVars, TRUE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Assemble61850ValsMsg()
//------------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        Assemble61850AckMsg()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Assemble 61850 GOOSE Ack Message
//
//  MECHANICS:          
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm61850.SeqNum
//
//  OUTPUTS:            
//
//  ALTERS:             
//
//  CALLS:              AssembleTxPkt()
//
//  EXECUTION TIME:     Measured execution time on ??? (Rev 00.?? code).
//                                ????usec
//
//------------------------------------------------------------------------------------------------------------

void Assemble61850AckMsg(uint8_t seqnum)
{
  // Assemble the ACK message.  Message consists of:
  //   Start Packet - 0x00
  //   Segment Code
  //   Message body:
  //      - Command = 0x0E: Acknowledge
  //      - Source/Destination Address = 0x59: Src Addr = 5, Dest addr = 9
  //      - Sequence Number = seqnum
  //      - Buffer Type = DP_BUFTYPE_GOOSE
  //      - Buffer ID = 0x0000
  //      - Buffer Length = 0x0001
  //      - Data bytes 0: Ack status
  //      - CRC LSB
  //      - CRC MSB
  //   Segment Code
  //   End Packet - 0x01
  DPComm61850.TxVars.TxBuf[0] = START_OF_PKT;       // Packet start
                                                    // Leave DPComm.TxBuf[1] open for first seg code
  DPComm61850.TxVars.TxBuf[2] = 0x0E;               // Command
  DPComm61850.TxVars.TxBuf[3] = 0x59;               // Src/Dest Address  (5 = Trip Unit, 9 = Ethernet #1)
  DPComm61850.TxVars.TxBuf[4] = seqnum;             // Sequence number
  DPComm61850.TxVars.TxBuf[5] = DP_BUFTYPE_GOOSE;   // Buffer type
  DPComm61850.TxVars.TxBuf[6] = 0x00;               // Buffer ID least significant byte
  DPComm61850.TxVars.TxBuf[7] = 0x00;               // Buffer ID most significant byte
  DPComm61850.TxVars.TxBuf[8] = 0x01;               // Buffer length least significant byte
  DPComm61850.TxVars.TxBuf[9] = 0x00;               // Buffer length most significant byte
  DPComm61850.TxVars.TxBuf[10] = DP_ACK;            // Acknowledge status

  // Call subroutine to assemble the packet per the modified COBS algorithm
  DPComm61850.TxVars.TxSegNdx = 1;                  // Initialize vars used in the insertion
  DPComm61850.TxVars.TxSegCharCnt = 0;
  DPComm61850.TxVars.TxNdx = 2;
  DPComm61850.TxVars.TxCRC = 0xFFFF;
  // Note, it is ok to use TxBuf[] as both the source and destination because the length is less than 126
  AssembleTxPkt1(&DPComm61850.TxVars.TxBuf[2], 9, &DPComm61850.TxVars, TRUE);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          Assemble61850AckMsg()
//------------------------------------------------------------------------------------------------------------






//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        DispComm61850_Rx()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Display Processor Communications Top-Level Receive Subroutine
//
//  MECHANICS:          This subroutine handles 61850 receptions from the Display Processor:
//                        - Parse the input packets in RxBuf[] and assemble the received message in RxMsg[]
//                            Call AssembleRxPkt().  True is returned when a message is received
//                        - Check the message for validity
//                        - If it is valid, decode the command, save the appropriate information, and set
//                            the appropriate flag to complete processing the command.  This involves
//                            responding to the command and/or storing the data or executing an action
//                          Note, the response is not assembled in this subroutine TxBuf[] may be in use,
//                          that is, we may be transmitting while this message is being received.  We
//                          therefore just set a flag in this subroutine to signal the DispComm_Tx() to
//                          assemble the response when the TxBuf[] is free and we are able to transmit
//                      The following commands are supported:
//                        - x06: Write With Acknowledge
//                          This writes new Setpoints data into the temporary copy.  We need only process
//                          the message and set the TXACK flag to generate an Ack response.
//                        - x0E: Acknowledge
//                          This is a response to a message (Write or Execute Action, for example) by some
//                          process we are running.  Signal the process that we have received the ACK by
//                          setting the ACK_RECEIVED flag, then wait for the process that initiated the
//                          message to clear the flag.
//
//  CAVEATS:            None
//
//  INPUTS:             DPComm.RxMsg[], DPComm.Flags
//
//  OUTPUTS:            DPComm.Flags, DPComm.Addr, DPComm.SeqNum, DPComm.AckNak
//
//  ALTERS:             DPComm.RxState
//
//  CALLS:              AssembleRxPkt(), TestInjCur()
//
//  EXECUTION TIME:     Measured execution time on ??? (Rev 00.?? code).
//                                ????usec
//
//------------------------------------------------------------------------------------------------------------

void DispComm61850_Rx(void)
{
  uint16_t bufID, msgLength, device_ID;
  uint8_t x = 0;

  // Wait for message and process command
  // Call AssembleRxPkt1() to process incoming characters.  Subroutine returns True when a completed message
  //   has been received.  The completed message is stored in DPComm.RxMsg[].
  //        RxMsg[0] - Command
  //        RxMsg[1] - Source/Destination Address
  //        RxMsg[2] - Sequence Number
  //        RxMsg[3] - Buffer Type
  //        RxMsg[4] - Buffer ID least significant byte
  //        RxMsg[5] - Buffer ID most significant byte
  //        RxMsg[6] - Buffer Length N least significant byte
  //        RxMsg[7] - Buffer Length N most significant byte
  //        RxMsg[8] - Data byte 0 (least significant byte)
  //            ...             ...
  //        RxMsg[N+7] - Data byte N-1 (most significant byte)
  //        RxMsg[N+8] - CRC LSB
  //        RxMsg[N+9] - CRC MSB
  if (AssembleRxPkt1(DMA1_Stream1, &DPComm61850.RxVars))
  {
    // The CRC of the incoming message is computed as the message is received and assembled in
    //   AssembleRxPkt1().  It is taken over the entire message, including the CRC bytes.  If the CRC is
    //   valid, DPComm61850.RxVars.RxCRC will be zero.
    // Valid source address is 9.  Valid destination address is 5.
    if ( (DPComm61850.RxVars.RxCRC == 0) && (DPComm61850.RxVars.RxMsg[1] == 0x95) )
    {
      if ( (DPComm61850.RxVars.RxMsg[0] == DP_CMND_WRWACK)       // If command is Write With Acknowledge
        && (DPComm61850.RxVars.RxMsg[3] == DP_BUFTYPE_GOOSE) )   //   and buffer type is GOOSE
      {
        // Process the message, store the data, and set flag to send back an ACK if everything is ok
        // Note: if the message is bad, it is just ignored.  This should never happen, as the GOOSE messages
        //       are fixed and limited.  Since the timeout time is so short, we may as well not send a
        //       response rather than NAK.
        // Assemble the Buffer ID
        bufID = DPComm61850.RxVars.RxMsg[4] + (((uint16_t)DPComm61850.RxVars.RxMsg[5]) << 8);
        msgLength = DPComm61850.RxVars.RxMsg[6] + ((uint16_t)DPComm61850.RxVars.RxMsg[7] << 8);
        device_ID = DPComm61850.RxVars.RxMsg[8] | ((uint16_t)DPComm61850.RxVars.RxMsg[9] << 8);
        // Check message IDs - Trip message received
        if (bufID == DP61850_TYPE_GO_CON)
        {
          // Check length
          if (msgLength == 4)
          {
            for(x = 0; x < (MAX_GOOSE_SUB - 1); x++)
            {
              if (Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID == 0)
              {
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID = device_ID;
                Sub_GoCB_Status_Ctrl_Pkt[x].Goose_Connection = DPComm61850.RxVars.RxMsg[10];
                DPComm61850.Req[DP61850_TYPE_ACK] |= ((uint16_t)(1 << DP61850_TYPE_GO_CON));  // Set flag to ACK
                break;
              }
              else if (Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID == device_ID)
              {
                Sub_GoCB_Status_Ctrl_Pkt[x].Goose_Connection = DPComm61850.RxVars.RxMsg[10];
                DPComm61850.Req[DP61850_TYPE_ACK] |= ((uint16_t)(1 << DP61850_TYPE_GO_CON));  // Set flag to ACK
                break;
              }
            }
          }
        }
        // Check message IDs - XCBR message received
        else if (bufID == DP61850_TYPE_GOCB_STATUS_CTRL)
        {
          // Check length
          if (msgLength == (sizeof(Sub_GoCB_Status_Ctrl_Pkt[0].CB_Status_Ctrl) + 1))
          {
            for(x = 0; x < (MAX_GOOSE_SUB - 1); x++)
            {
              if (Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID == 0)
              {
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID = device_ID;
              }
              if(Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Device_ID == device_ID)
              {
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CB_Pos_Status = DPComm61850.RxVars.RxMsg[10];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Trip_Failure = DPComm61850.RxVars.RxMsg[11];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.ZSI_Status = DPComm61850.RxVars.RxMsg[12];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Capture_State = DPComm61850.RxVars.RxMsg[13];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.SPSet_Select = DPComm61850.RxVars.RxMsg[14];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.Source_Status = DPComm61850.RxVars.RxMsg[15];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB1_Open_Op = DPComm61850.RxVars.RxMsg[16];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB1_Close_Op = DPComm61850.RxVars.RxMsg[17];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB2_Open_Op = DPComm61850.RxVars.RxMsg[18];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB2_Close_Op = DPComm61850.RxVars.RxMsg[19];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB3_Open_Op = DPComm61850.RxVars.RxMsg[20];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB3_Close_Op = DPComm61850.RxVars.RxMsg[21];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB4_Open_Op = DPComm61850.RxVars.RxMsg[22];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB4_Close_Op = DPComm61850.RxVars.RxMsg[23];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB5_Open_Op = DPComm61850.RxVars.RxMsg[24];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB5_Close_Op = DPComm61850.RxVars.RxMsg[25];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB6_Open_Op = DPComm61850.RxVars.RxMsg[26];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB6_Close_Op = DPComm61850.RxVars.RxMsg[27];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB7_Open_Op = DPComm61850.RxVars.RxMsg[28];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB7_Close_Op = DPComm61850.RxVars.RxMsg[29];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB8_Open_Op = DPComm61850.RxVars.RxMsg[30];
                Sub_GoCB_Status_Ctrl_Pkt[x].CB_Status_Ctrl.CTRL_CB8_Close_Op = DPComm61850.RxVars.RxMsg[31];
                Sub_GoCB_Status_Ctrl_Pkt[x].Subscriptions = DPComm61850.RxVars.RxMsg[32];
                
                DPComm61850.Req[DP61850_TYPE_ACK] |= ((uint16_t)(1 << DP61850_TYPE_GOCB_STATUS_CTRL));   // Set flag to ACK
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
                TESTPIN_D1_HIGH; //Turn OFF Goose Message Received signal
#endif
                break;
              }
            }
          }
        }
        
        if (StartUP_Status_Send)
        {
            StartUP_Status_Send = FALSE;
            DPComm61850.Req[DP61850_TYPE_GOCB_STATUS_CTRL] = 1;
            DPComm61850.Req[DP61850_TYPE_VALS] = 1;
        }
      }
      else if (DPComm61850.RxVars.RxMsg[0] == DP_CMND_ACK)      // If command is Acknowledge
      {
        msgLength = DPComm61850.RxVars.RxMsg[6] + ((uint16_t)DPComm61850.RxVars.RxMsg[7] << 8);
        // This is in response to a GOOSE message that was previously transmitted.  Check the sequence
        //   numbers to match the ack to the message, then clear the request.
        // Check length and whether ACK was received
        if ( (msgLength == 1) && (DPComm61850.RxVars.RxMsg[8] == DP_ACK) )
        {
            // Clear the request associated with the ACK
            if (DPComm61850.RxVars.RxMsg[2] < NUM_61850_REQUESTS)             // Make sure in range
            {
#ifdef ENABLE_GOOSE_COMM_SPEED_TEST
              if(DPComm61850.RxVars.RxMsg[2] == DP61850_TYPE_GOCB_STATUS_CTRL)
              {
                TESTPIN_A3_LOW;
              }
              else if(DPComm61850.RxVars.RxMsg[2] == DP61850_TYPE_VALS)
              {
                TESTPIN_A3_LOW;
              }
#endif
              DPComm61850.Req[DPComm61850.RxVars.RxMsg[2]] = FALSE;
            }
        }
      }

    }

  }
            
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION          DispComm61850_Rx()
//------------------------------------------------------------------------------------------------------------
