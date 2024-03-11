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
//  MODULE NAME:        IntrInline_def.h
//
//  MECHANICS:          This file contains in-line subroutine definitions file for the Intr.c module.  It is
//                      only used in the Intr.c module and was created for readability purposes only
//
//  TARGET HARDWARE:    PXR35 Rev 1 and later boards
//
//------------------------------------------------------------------------------------------------------------
//
//  Development Revision History:
//   0.32   190726  DAH File Creation
//                      - struct Cur200msecSOS_Sav renamed to Cur200msFltrSOS_Sav
//                      - struct CurOneCycSinSOS_Sav replaced  with CurVol200msNoFltrSinSOS_Sav and
//                        struct CurOneCycCosSOS_Sav replaced with CurVol200msNoFltrCosSOS_Sav
//                      - struct VolAFEOneCycLNSOS_Sav and VolAFEOneCycLLSOS_Sav combined into one
//                        structure, VolAFEOneCycSOS_Sav
//                      - struct VolAFE200msecLNSOS_Sav and VolAFE200msecLLSOS_Sav combined into one
//                        structure, VolAFE200msFltrSOS_Sav
//                      - struct VolADCOneCycLNSOS_Sav and VolADCOneCycLLSOS_Sav combined into one
//                        structure, VolADCOneCycSOS_Sav
//                      - struct VolADC200msecLNSOS_Sav and VolADC200msecLLSOS_Sav combined into one
//                        structure, VolADC200msSOS_Sav
//   0.33   190823  DAH - In save_OC_SOS(), rearranged instructions (grouped resets together) to save code
//                        space and especially execution time
//                      - Added 200msec counter (msec200Ctr) to inc_buf_indices() for readablility
//                      - Added save_OC_peaks() to support crest factor calculations
//   0.59   220831  DAH - Deleted Black Box FRAM function (10-cycle waveform capture without aux power)
//                          - Modified buffer_samples()
//                          - Deleted assemble_BB_packet()
//   0.70   230224   BP - Updated the Neutral scaling code
//                      - Added One sec counter (OneSecCtr)
//   0.93   231010   BP - Added code for Firmware Simulated test
//   104    231102   BP - Fixed the 60% neutral ratio and added 200%
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
//    In-line subroutines for the sampling interrupt
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void buffer_samples(void);
__attribute__(( always_inline )) inline void update_peaks(void);
__attribute__(( always_inline )) inline void save_OC_peaks(void);
__attribute__(( always_inline )) inline void update_Ia_SOS(void);
__attribute__(( always_inline )) inline void update_Ib_SOS(void);
__attribute__(( always_inline )) inline void update_Ic_SOS(void);
__attribute__(( always_inline )) inline void update_In_SOS(void);
__attribute__(( always_inline )) inline void update_Igsrc_SOS(void);
__attribute__(( always_inline )) inline void update_Igres_SOS(void);
__attribute__(( always_inline )) inline void update_VlnAFE_SOS(void);
__attribute__(( always_inline )) inline void update_VllAFE_SOS(void);
__attribute__(( always_inline )) inline void update_VlnADC_SOS(void);
__attribute__(( always_inline )) inline void update_VllADC_SOS(void);
__attribute__(( always_inline )) inline void update_Power_SOS(void);
__attribute__(( always_inline )) inline void update_THD_Sin_Cos_SOS(void);
__attribute__(( always_inline )) inline void save_OC_SOS(void);
__attribute__(( always_inline )) inline void save_200msec_SOS(void);
__attribute__(( always_inline )) inline void calc_min_max_I_HC_SOS(void);
__attribute__(( always_inline )) inline void calc_min_max_I_OC_SOS(void);
__attribute__(( always_inline )) inline void calc_max_I_HC_SOS(void);
__attribute__(( always_inline )) inline void calc_max_I_OC_SOS(void);
__attribute__(( always_inline )) inline void update_I_HC_SOS(void);
__attribute__(( always_inline )) inline void update_I_OC_SOS(void);
__attribute__(( always_inline )) inline void update_delayed_vbuf(void);
__attribute__(( always_inline )) inline void inc_buf_indices(void);




extern const float x_n_test[ ];                 // *** DAH  UNCOMMENT FOR TEST
extern const float x_n_test1[ ];                 // *** DAH  UNCOMMENT FOR TEST


//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        buffer_samples()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Store Samples in RAM In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine moves the input samples into the RAM buffer.
//                      Note, input samples are assigned as follows:
//                          Ia - AFE_new_samples[0]    Ib - AFE_new_samples[1]    Ic - AFE_new_samples[2]
//                          In - AFE_new_samples[3]    Igsrc - AFE_new_samples[4]
//                          VanAFE - AFE_new_samples[5]  VbnAFE - AFE_new_samples[6]
//                          VcnAFE - AFE_new_samples[7]
//                          VanADC - ADC_samples[5]     VbnADC - ADC_samples[6]     VcnADC - ADC_samples[7]
//                          Igres is computed earlier
//                      The samples are stored in:
//                          SampleBuf[SampleIndex].xxx, xxx = Ia .. Igres, Van1 - Vcn1, Van2 - Vcn2
//                      SampleBuf[] is used for Trip and Alarm waveform captures
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[], ADC_samples[]
// 
//  OUTPUTS:            SampleBuf[]
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void buffer_samples(void)
{
  // Store samples in the RAM buffer
//      SampleBuf[SampleIndex].Ia = (int16_t)(x_n_test[freddah] * 10.0f);           // *** DAH  UNCOMMENT FOR HARMONICS TEST
//      SampleBuf[SampleIndex].VanAFE = (int16_t)(x_n_test1[freddah++] * 10.0f);    // *** DAH  UNCOMMENT FOR HARMONICS TEST
//      SampleBuf[SampleIndex].Ia = x_n_test[freddah];                              // *** DAH  FOR PF AND THD TEST
//      AFE_new_samples[0] = x_n_test[freddah++];                                   // *** DAH  FOR PF AND THD TEST  ***********************
//      SampleBuf[SampleIndex].VanAFE = (uint16_t)(x_n_test1[freddah1]*10.0F);      // *** DAH  FOR PF AND THD TEST
//      AFE_new_samples[5] = x_n_test1[freddah1++];                                 // *** DAH  FOR PF AND THD TEST  ***********************
//      if (freddah > 959)
//      freddah = 0;
//      if (freddah1 > 959)
//      freddah1 = 0;
//  SampleBuf[SampleIndex].Ia = SampleIndex;                    //     *** DAH TEST FOR ALARM AND TRIP LOG TESTING
  SampleBuf[SampleIndex].Ia = AFE_new_samples[0];                        
  SampleBuf[SampleIndex].Ib = AFE_new_samples[1];
  SampleBuf[SampleIndex].Ic = AFE_new_samples[2];
  SampleBuf[SampleIndex].In = AFE_new_samples[3];
  SampleBuf[SampleIndex].Igsrc = AFE_new_samples[4];
  SampleBuf[SampleIndex].Igres = Igres;
  SampleBuf[SampleIndex].VanAFE = (int16_t)(AFE_new_samples[5] * 10);               // *** DAH TEST  COMMENT THIS OUT FOR HARMONICS TEST
  SampleBuf[SampleIndex].VbnAFE = (int16_t)(AFE_new_samples[6] * 10);
  SampleBuf[SampleIndex].VcnAFE = (int16_t)(AFE_new_samples[7] * 10);
  SampleBuf[SampleIndex].VanADC = (int16_t)(ADC_samples[5] * 10);
  SampleBuf[SampleIndex].VbnADC = (int16_t)(ADC_samples[6] * 10);
  SampleBuf[SampleIndex].VcnADC = (int16_t)(ADC_samples[7] * 10);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        buffer_samples()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_peaks()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update One-Cycle Peak Currents In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the one-cycle peak currents based on the new current samples
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[]
// 
//  OUTPUTS:            CurOneCycPeak.Ix, x = a, b, c, n
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_peaks(void)
{
  float ftemp1;

  ftemp1 = (AFE_new_samples[0] > 0) ? (AFE_new_samples[0]) : (-AFE_new_samples[0]);
  if (ftemp1 > CurOneCycPeak.Ia)
  {
    CurOneCycPeak.Ia = ftemp1;
  }
  ftemp1 = (AFE_new_samples[1] > 0) ? (AFE_new_samples[1]) : (-AFE_new_samples[1]);
  if (ftemp1 > CurOneCycPeak.Ib)
  {
    CurOneCycPeak.Ib = ftemp1;
  }
  ftemp1 = (AFE_new_samples[2] > 0) ? (AFE_new_samples[2]) : (-AFE_new_samples[2]);
  if (ftemp1 > CurOneCycPeak.Ic)
  {
    CurOneCycPeak.Ic = ftemp1;
  }
  ftemp1 = (AFE_new_samples[3] > 0) ? (AFE_new_samples[3]) : (-AFE_new_samples[3]);
  if (ftemp1 > CurOneCycPeak.In)
  {
    CurOneCycPeak.In = ftemp1;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_peaks()
//------------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        save_OC_peaks()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Save One-Cycle Peak Currents In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine saves the one-cycle peak currents for crest factor calculations and
//                      then resets the one-cycle peak currents to zero
//
//  CAVEATS:            None
// 
//  INPUTS:             CurOneCycPeak.Ix, x = a, b, c, n
// 
//  OUTPUTS:            CF_PeakSav.Ix, x = a, b, c, n
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void save_OC_peaks(void)
{
  CF_PeakSav.Ia = CurOneCycPeak.Ia;
  CF_PeakSav.Ib = CurOneCycPeak.Ib;
  CF_PeakSav.Ic = CurOneCycPeak.Ic;
  CF_PeakSav.In = CurOneCycPeak.In;
  CurOneCycPeak.Ia = 0;
  CurOneCycPeak.Ib = 0;
  CurOneCycPeak.Ic = 0;
  CurOneCycPeak.In = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        save_OC_peaks()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Ia_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Ia Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called,
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec filtered, 200msec unfiltered, one-cycle, and
//                      half-cycle sums of squares for Ia with the new sample value.
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.Ia, is used for metering
//                      and is updated with the square of MTR_new_samples[0].  This value is more heavily
//                      filtered.  The half-cycle (CurHalfCycSOS_SumI.Ia) and one-cycle
//                      (CurOneCycSOS_SumI.Ia) values are used for protection and are updated with
//                      AFE_new_samples[0].  The 200msec sum-of-squares value, Cur200msNoFltrSOS_Sum.Ia, is
//                      used for THD and Displacement Power Factor computations and is also updated with
//                      AFE_new_samples[0].
//
//  CAVEATS:            None
// 
//  INPUTS:             MTR_new_samples[0], AFE_new_samples[0], FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.Ia, CurHalfCycSOS_SumI.Ia, CurOneCycSOS_SumI.Ia,
//                      Cur200msNoFltrSOS_Sum.Ia
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Ia_SOS(void)
{
  float ftemp1;
  unsigned long ultemp;
  
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IA))   // FW Simulated Test - Phase A
  {
    AFE_new_samples[0] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[0] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  }  

  ftemp1 = MTR_new_samples[0] * MTR_new_samples[0];             // Add square of new filtered current
  Cur200msFltrSOS_Sum.Ia += ftemp1;                             //   sample to 200msec tally
  ftemp1 = AFE_new_samples[0];                                  // Add square of new unfiltered current
  Cur200msNoFltrSOS_Sum.Ia += ftemp1 * ftemp1;                  //   sample to 200msec tally
  // Convert the unfiltered sample from float to integer * 10.  Save the magnitude since will be squared
  ultemp = ( (ftemp1 < 0) ? ((unsigned long)(ftemp1 * -10)) : ((unsigned long)(ftemp1 * 10)) );
  ulltemp[0] = (unsigned long long)(ultemp) * ultemp;           // Square the sample and save it
  CurHalfCycSOS_SumI.Ia += ulltemp[0];                          // Add the square to the 1/2- and 1-cyc
  CurOneCycSOS_SumI.Ia += ulltemp[0];                           //   buffers
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Ia_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Ib_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Ib Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec filtered, 200msec unfiltered, one-cycle, and
//                      half-cycle sums of squares for Ib with the new sample value.
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.Ib, is used for metering
//                      and is updated with the square of MTR_new_samples[1].  This value is more heavily
//                      filtered.  The half-cycle (CurHalfCycSOS_SumI.Ib) and one-cycle
//                      (CurOneCycSOS_SumI.Ib) values are used for protection and are updated with
//                      AFE_new_samples[1].  The 200msec sum-of-squares value, Cur200msNoFltrSOS_Sum.Ib, is
//                      used for THD and Displacement Power Factor computations and is also updated with
//                      AFE_new_samples[1].
//
//  CAVEATS:            None
// 
//  INPUTS:             MTR_new_samples[1], AFE_new_samples[1], FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.Ib, CurHalfCycSOS_SumI.Ib, CurOneCycSOS_SumI.Ib,
//                      Cur200msNoFltrSOS_Sum.Ib
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Ib_SOS(void)
{
  float ftemp1;
  unsigned long ultemp;

    
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IB))   // FW Simulated Test - Phase B
  {
    AFE_new_samples[1] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[1] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  }  

  ftemp1 = MTR_new_samples[1] * MTR_new_samples[1];
  Cur200msFltrSOS_Sum.Ib += ftemp1;
  ftemp1 = AFE_new_samples[1];
  Cur200msNoFltrSOS_Sum.Ib += ftemp1 * ftemp1;
  // Convert the unfiltered sample from float to integer * 10.  Save the magnitude since will be squared
  ultemp = ( (ftemp1 < 0) ? ((unsigned long)(ftemp1 * -10)) : ((unsigned long)(ftemp1 * 10)) );
  ulltemp[1] = (unsigned long long)(ultemp) * ultemp;
  CurHalfCycSOS_SumI.Ib += ulltemp[1];
  CurOneCycSOS_SumI.Ib += ulltemp[1];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Ib_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Ic_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Ic Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec filtered, 200msec unfiltered, one-cycle, and
//                      half-cycle sums of squares for Ic with the new sample value.
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.Ic, is used for metering
//                      and is updated with the square of MTR_new_samples[2].  This value is more heavily
//                      filtered.  The half-cycle (CurHalfCycSOS_SumI.Ic) and one-cycle
//                      (CurOneCycSOS_SumI.Ic) values are used for protection and are updated with
//                      AFE_new_samples[2].  The 200msec sum-of-squares value, Cur200msNoFltrSOS_Sum.Ic, is
//                      used for THD and Displacement Power Factor computations and is also updated with
//                      AFE_new_samples[2].
//
//  CAVEATS:            None
// 
//  INPUTS:             MTR_new_samples[2], AFE_new_samples[2], FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.Ic, CurHalfCycSOS_SumI.Ic, CurOneCycSOS_SumI.Ic
//                      Cur200msNoFltrSOS_Sum.Ic
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Ic_SOS(void)
{
  float ftemp1;
  unsigned long ultemp;
      
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IC))   // FW Simulated Test - Phase C
  {
    AFE_new_samples[2] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[2] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  }  

  ftemp1 = MTR_new_samples[2] * MTR_new_samples[2];
  Cur200msFltrSOS_Sum.Ic += ftemp1;
  ftemp1 = AFE_new_samples[2];
  Cur200msNoFltrSOS_Sum.Ic += ftemp1 * ftemp1;
  // Convert the unfiltered sample from float to integer * 10.  Save the magnitude since will be squared
  ultemp = ( (ftemp1 < 0) ? ((unsigned long)(ftemp1 * -10)) : ((unsigned long)(ftemp1 * 10)) );
  ulltemp[2] = (unsigned long long)(ultemp) * ultemp;
  CurHalfCycSOS_SumI.Ic += ulltemp[2];
  CurOneCycSOS_SumI.Ic += ulltemp[2];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Ic_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_In_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update In Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec filtered, 200msec unfiltered, one-cycle, and
//                      half-cycle sums of squares for In with the new sample value.
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.In, is used for metering
//                      and is updated with the square of MTR_new_samples[3].  This value is more heavily
//                      filtered.  The half-cycle (CurHalfCycSOS_SumI.In) and one-cycle
//                      (CurOneCycSOS_SumI.In) values are used for protection and are updated with
//                      AFE_new_samples[3].  The 200msec sum-of-squares value, Cur200msNoFltrSOS_Sum.In, is
//                      used for THD and Displacement Power Factor computations and is also updated with
//                      AFE_new_samples[3].
//
//  CAVEATS:            None
// 
//  INPUTS:             MTR_new_samples[3], AFE_new_samples[3], FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.In, CurHalfCycSOS_SumI.In, CurOneCycSOS_SumI.In
//                      Cur200msNoFltrSOS_Sum.In
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_In_SOS(void)
{
  float ftemp1;
  unsigned long ultemp;

      
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IN))   // FW Simulated Test - Phase N
  {
    AFE_new_samples[3] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[3] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  }  

  ftemp1 = MTR_new_samples[3] * MTR_new_samples[3];
  Cur200msFltrSOS_Sum.In += ftemp1;
  ftemp1 = AFE_new_samples[3];
  Cur200msNoFltrSOS_Sum.In += ftemp1 * ftemp1;
  // Convert the unfiltered sample from float to integer * 10.  Save the magnitude since will be squared
  ultemp = ( (ftemp1 < 0) ? ((unsigned long)(ftemp1 * -10)) : ((unsigned long)(ftemp1 * 10)) );
  ulltemp[3] = (unsigned long long)(ultemp) * ultemp;
  CurHalfCycSOS_SumI.In += ulltemp[3];
  CurOneCycSOS_SumI.In += ulltemp[3];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_In_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Igsrc_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Igsrc Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec, one-cycle, and half-cycle sums of squares for
//                      Igsrc with the new sample value.
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.Igsrc, is used for
//                      metering and is updated with the square of MTR_new_samples[4].  This value is more
//                      heavily filtered.  The half-cycle (CurHalfCycSOS_SumI.Igsrc) and one-cycle
//                      (CurOneCycSOS_SumI.Igsrc) values are used for protection and are updated with
//                      AFE_new_samples[4].
//
//  CAVEATS:            None
// 
//  INPUTS:             MTR_new_samples[4], AFE_new_samples[4], FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.Igsrc, CurHalfCycSOS_SumI.Igsrc, CurOneCycSOS_SumI.Igsrc
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Igsrc_SOS(void)
{
  float ftemp1;
  unsigned long ultemp;

      
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IG))   // FW Simulated Test - Phase G
  {
    AFE_new_samples[4] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[4] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  }  

  ftemp1 = MTR_new_samples[4] * MTR_new_samples[4];
  Cur200msFltrSOS_Sum.Igsrc += ftemp1;
  ultemp = ( (AFE_new_samples[4] < 0) ?
    ((unsigned long)(AFE_new_samples[4] * -10)) : ((unsigned long)(AFE_new_samples[4] * 10)) );
  ulltemp[4] = (unsigned long long)(ultemp) * ultemp;
  CurHalfCycSOS_SumI.Igsrc += ulltemp[4];
  CurOneCycSOS_SumI.Igsrc += ulltemp[4];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Igsrc_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Igres_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Igres Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec, one-cycle, and half-cycle sums of squares for
//                      Igres with the new sample value (the sum of Ia, Ib, Ic, and In, which is stored in
//                      Igres and Igres_mtr).
//                      Note, the 200msec sum-of-squares value, Cur200msFltrSOS_Sum.Igres, is used for
//                      metering and is updated with the square of Igres_mtr.  This value is more heavily
//                      filtered.  The half-cycle (CurHalfCycSOS_SumI.Igres) and one-cycle
//                      (CurOneCycSOS_SumI.Igres) values are used for protection and are updated with Igres.
//
//  CAVEATS:            None
// 
//  INPUTS:             Igres, Igres_mtr, FW_SimulatedTest.TestEnable,
//                      FW_SimulatedTest.TestPhase, FW_SimulatedTest.TestCurrent
// 
//  OUTPUTS:            Cur200msFltrSOS_Sum.Igres, CurHalfCycSOS_SumI.Igres, CurOneCycSOS_SumI.Igres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Igres_SOS(void)
{
  float ftemp1;                              // *** BP - Not sure if we do Sec Inj on Residual Ground
  unsigned long ultemp; 
        
  if ((FW_SimulatedTest.Enable == TRUE) && (FW_SimulatedTest.Phase == TEST_IG))   // FW Simulated Test - Phase G
  {
    AFE_new_samples[5] = FW_SimulatedTest.TestCurrent;         // for one cycle value
    MTR_new_samples[5] = FW_SimulatedTest.TestCurrent;         // for 200msec value
  } 
                                               
  ftemp1 = Igres_mtr * Igres_mtr;
  Cur200msFltrSOS_Sum.Igres += ftemp1;
  ultemp = ( (Igres < 0) ? ((unsigned long)(Igres * -10)) : ((unsigned long)(Igres * 10)) );
  ulltemp[5] = (unsigned long long)(ultemp) * ultemp;
  CurHalfCycSOS_SumI.Igres += ulltemp[5];
  CurOneCycSOS_SumI.Igres += ulltemp[5];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Igres_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_VlnAFE_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Load-Side Line-to-Neutral Voltages (AFE Vxn) Sums of Squares In-Line Routine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec and one-cycle sums of squares for the AFE Vxn
//                      (x = a, b, c) with the new sample values.
//                      Note, the 200msec sum-of-squares value, VolAFE200msFltrSOS_Sum.Vxn, is used for
//                      metering and is updated with the square of MTR_new_samples[].  This value is more
//                      heavily filtered.  The one-cycle (VolAFEOneCycSOS_Sum.Vxn) values are used for
//                      protection and are updated with AFE_new_samples[].
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[5..7], MTR_new_samples[5..7]
// 
//  OUTPUTS:            VolAFEOneCycSOS_Sum.Vxn, VolAFE200msFltrSOS_Sum.Vxn, x = a, b, c
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_VlnAFE_SOS(void)
{
  float ftemp1;

  ftemp1 = AFE_new_samples[5] * AFE_new_samples[5];             // Add square of new voltage sample
  VolAFEOneCycSOS_Sum.Van += ftemp1;
  ftemp1 = MTR_new_samples[5] * MTR_new_samples[5];             // Add square of new voltage sample
  VolAFE200msFltrSOS_Sum.Van += ftemp1;
  ftemp1 = AFE_new_samples[6] * AFE_new_samples[6];             // Add square of new voltage sample
  VolAFEOneCycSOS_Sum.Vbn += ftemp1;
  ftemp1 = MTR_new_samples[6] * MTR_new_samples[6];             // Add square of new voltage sample
  VolAFE200msFltrSOS_Sum.Vbn += ftemp1;
  ftemp1 = AFE_new_samples[7] * AFE_new_samples[7];             // Add square of new voltage sample
  VolAFEOneCycSOS_Sum.Vcn += ftemp1;
  ftemp1 = MTR_new_samples[7] * MTR_new_samples[7];             // Add square of new voltage sample
  VolAFE200msFltrSOS_Sum.Vcn += ftemp1;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_VlnAFE_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_VllAFE_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Load-Side Line-to-Line Voltages (AFE Vxn) Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec and one-cycle sums of squares for the AFE Vxx
//                      (xx = ab, bc, ca) with the new sample values.
//                      Note, the 200msec sum-of-squares value, VolAFE200msFltrSOS_Sum.Vxx, is used for
//                      metering and is updated with the square of MTR_new_samples[].  This value is more
//                      heavily filtered.  The one-cycle (VolAFEOneCycSOS_Sum.Vxx) values are used for
//                      protection and are updated with AFE_new_samples[].
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[5..7], MTR_new_samples[5..7]
// 
//  OUTPUTS:            VolAFEOneCycSOS_Sum.Vxx, VolAFE200msFltrSOS_Sum.Vxx, x = ab, bc, ca
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_VllAFE_SOS(void)
{
  float ftemp1;

  ftemp1 = AFE_new_samples[5] - AFE_new_samples[6];             // Compute Vab
  ftemp1 = ftemp1 * ftemp1;                                     // Square the voltage
  VolAFEOneCycSOS_Sum.Vab += ftemp1;                            // Add to the sum of squares
  ftemp1 = MTR_new_samples[5] - MTR_new_samples[6];
  ftemp1 = ftemp1 * ftemp1;
  VolAFE200msFltrSOS_Sum.Vab += ftemp1;
  ftemp1 = AFE_new_samples[6] - AFE_new_samples[7];             // Vbc
  ftemp1 = ftemp1 * ftemp1;
  VolAFEOneCycSOS_Sum.Vbc += ftemp1;
  ftemp1 = MTR_new_samples[6] - MTR_new_samples[7];
  ftemp1 = ftemp1 * ftemp1;
  VolAFE200msFltrSOS_Sum.Vbc += ftemp1;
  ftemp1 = AFE_new_samples[7] - AFE_new_samples[5];             // Vca
  ftemp1 = ftemp1 * ftemp1;
  VolAFEOneCycSOS_Sum.Vca += ftemp1;
  ftemp1 = MTR_new_samples[7] - MTR_new_samples[5];
  ftemp1 = ftemp1 * ftemp1;
  VolAFE200msFltrSOS_Sum.Vca += ftemp1;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_VllAFE_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_VlnADC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Line-Side Line-to-Neutral Voltages (ADC Vxn) Sums of Squares In-Line Routine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec and one-cycle sums of squares for the ADC Vxn
//                      (x = a, b, c) with the new sample values.
//
//  CAVEATS:            None
// 
//  INPUTS:             ADC_samples[5..7]
// 
//  OUTPUTS:            VolADCOneCycSOS_Sum.Vxn, VolADC200msSOS_Sum.Vxn, x = a, b, c
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_VlnADC_SOS(void)
{
  float ftemp1;

  ftemp1 = ADC_samples[5] * ADC_samples[5];                     // Add square of new voltage sample
  VolADCOneCycSOS_Sum.Van += ftemp1;
  VolADC200msSOS_Sum.Van += ftemp1;
  ftemp1 = ADC_samples[6] * ADC_samples[6];                     // Add square of new voltage sample
  VolADCOneCycSOS_Sum.Vbn += ftemp1;
  VolADC200msSOS_Sum.Vbn += ftemp1;
  ftemp1 = ADC_samples[7] * ADC_samples[7];                     // Add square of new voltage sample
  VolADCOneCycSOS_Sum.Vcn += ftemp1;
  VolADC200msSOS_Sum.Vcn += ftemp1;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_VlnADC_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_VllADC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Line-Side Line-to-Line Voltages (ADC Vxn) Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec and one-cycle sums of squares for the ADC Vxx
//                      (xx = ab, bc, ca) with the new sample values.
//
//  CAVEATS:            None
// 
//  INPUTS:             ADC_samples[5..7]
// 
//  OUTPUTS:            VolADCOneCycSOS_Sum.Vxx, VolADC200msSOS_Sum.Vxx, x = ab, bc, ca
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_VllADC_SOS(void)
{
  float ftemp1;

  ftemp1 = ADC_samples[5] - ADC_samples[6];                     // Compute Vab
  ftemp1 = ftemp1 * ftemp1;                                     // Square the voltage
  VolADCOneCycSOS_Sum.Vab += ftemp1;                            // Add to the sum of squares
  VolADC200msSOS_Sum.Vab += ftemp1;
  ftemp1 = ADC_samples[6] - ADC_samples[7];                     // Vbc
  ftemp1 = ftemp1 * ftemp1;
  VolADCOneCycSOS_Sum.Vbc += ftemp1;
  VolADC200msSOS_Sum.Vbc += ftemp1;
  ftemp1 = ADC_samples[7] - ADC_samples[5];                     // Vca
  ftemp1 = ftemp1 * ftemp1;
  VolADCOneCycSOS_Sum.Vca += ftemp1;
  VolADC200msSOS_Sum.Vca += ftemp1;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_VllADC_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_Power_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Load-Side Power Sums of Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the 200msec and one-cycle sums of squares for the powers
//                      using the AFE voltage samples.
//                      Note, the 200msec sum-of-squares values (for example, PwrOneCycSOS_Sum.Pa) use
//                      MTR_new_samples[].  This value is more heavily filtered.  The one-cycle
//                      sum-of-squares values (for example, PwrOneCycSOS_Sum.Pa) use AFE_new_samples[].
//                      These are not filtered as much and so are appropriate for the faster response.
//
//  CAVEATS:            None
// 
//  INPUTS:             AFE_new_samples[], MTR_new_samples[]
// 
//  OUTPUTS:            PwrOneCycSOS_Sum.Px, PwrOneCycSOS_Sum.RPx, x = a, b, c
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_Power_SOS(void)
{
  float ftemp1;

  // Update the power sums of squares.  Note, these use the AFE voltages only.  There is no provision to use
  //   the ADC voltages.  The AFE voltages (generated by the VDB board) are the only voltages that can be
  //   used to get revenue-grade accuracy.
  ftemp1 = AFE_new_samples[0] * AFE_new_samples[5];
  PwrOneCycSOS_Sum.Pa += ftemp1;
  ftemp1 = MTR_new_samples[0] * MTR_new_samples[5];
  Pwr200msecSOS_Sum.Pa += ftemp1;
  ftemp1 = AFE_new_samples[1] * AFE_new_samples[6];
  PwrOneCycSOS_Sum.Pb += ftemp1;
  ftemp1 = MTR_new_samples[1] * MTR_new_samples[6];
  Pwr200msecSOS_Sum.Pb += ftemp1;
  ftemp1 = AFE_new_samples[2] * AFE_new_samples[7];
  PwrOneCycSOS_Sum.Pc += ftemp1;
  ftemp1 = MTR_new_samples[2] * MTR_new_samples[7];
  Pwr200msecSOS_Sum.Pc += ftemp1;
  ftemp1 = AFE_new_samples[0] * DelayedVolts_OC.Van[DelayedVoltsNdx];
  PwrOneCycSOS_Sum.RPa += ftemp1;
  ftemp1 = MTR_new_samples[0] * DelayedVolts_200msec.Van[DelayedVoltsNdx];
  Pwr200msecSOS_Sum.RPa += ftemp1;
  ftemp1 = AFE_new_samples[1] * DelayedVolts_OC.Vbn[DelayedVoltsNdx];
  PwrOneCycSOS_Sum.RPb += ftemp1;
  ftemp1 = MTR_new_samples[1] * DelayedVolts_200msec.Vbn[DelayedVoltsNdx];
  Pwr200msecSOS_Sum.RPb += ftemp1;
  ftemp1 = AFE_new_samples[2] * DelayedVolts_OC.Vcn[DelayedVoltsNdx];
  PwrOneCycSOS_Sum.RPc += ftemp1;
  ftemp1 = MTR_new_samples[2] * DelayedVolts_200msec.Vcn[DelayedVoltsNdx];
  Pwr200msecSOS_Sum.RPc += ftemp1;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_Power_SOS()
//------------------------------------------------------------------------------------------------------------





//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_THD_Sin_Cos_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Sin and Cos Components of the Fundamental For THD In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called,
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the sum of squares for the sine and cosine components of the
//                      fundamental for Ia, Ib, Ic, In, VanAFE, VbnAFE, and VcnAFE for total harmonic
//                      distortion (THD) and displacement power factor computations.
//                      Note, The line-to-line voltages can be computed from the line-to-neutral voltages
//                      and so do not need to be computed here.
//                      Note, AFE_new_samples[] is used for these computations.  These are not digitally
//                      filtered like the MTR_new_samples[] are, and so are more accurate when computing
//                      THD.
//
//  CAVEATS:            None
// 
//  INPUTS:             OneCycInd, AFE_new_samples[], SIN_COEFF[]
// 
//  OUTPUTS:            Cur200msNoFltrSinSOS_Sum.Ix, Cur200msNoFltrCosSOS_Sum.Ix, x = a, b, c
//                      VolAFE200msNoFltrSinSOS_Sum.Vxn, VolAFE200msNoFltrCosSOS_Sum.Vxn, x = a, b, c
//
//  ALTERS:             None
// 
//  CALLS:              None
//
//  EXECUTION TIME:     Measured on 190724 (rev 0.32 code): 1.4usec
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_THD_Sin_Cos_SOS(void)
{
  float ftemp1;
  uint8_t i;

  // Update the sine and cosine components of the fundamental for THD
  ftemp1 = SIN_COEFF[OneCycInd];            // Sin index is OneCycInd (OneCycInd is 0 .. 79)
  Cur200msNoFltrSinSOS_Sum.Ia += (AFE_new_samples[0] * ftemp1);
  Cur200msNoFltrSinSOS_Sum.Ib += (AFE_new_samples[1] * ftemp1);
  Cur200msNoFltrSinSOS_Sum.Ic += (AFE_new_samples[2] * ftemp1);
  Cur200msNoFltrSinSOS_Sum.In += (AFE_new_samples[3] * ftemp1);
  VolAFE200msNoFltrSinSOS_Sum.Van += (AFE_new_samples[5] * ftemp1);
  VolAFE200msNoFltrSinSOS_Sum.Vbn += (AFE_new_samples[6] * ftemp1);
  VolAFE200msNoFltrSinSOS_Sum.Vcn += (AFE_new_samples[7] * ftemp1);
                                            // Cos index = sin index + 90deg (20*4.5) with rollover at 80
  i = (OneCycInd <= 59) ? (OneCycInd + 20) : (OneCycInd - 60);
  ftemp1 = SIN_COEFF[i];
  Cur200msNoFltrCosSOS_Sum.Ia += (AFE_new_samples[0] * ftemp1);
  Cur200msNoFltrCosSOS_Sum.Ib += (AFE_new_samples[1] * ftemp1);
  Cur200msNoFltrCosSOS_Sum.Ic += (AFE_new_samples[2] * ftemp1);
  Cur200msNoFltrCosSOS_Sum.In += (AFE_new_samples[3] * ftemp1);
  VolAFE200msNoFltrCosSOS_Sum.Van += (AFE_new_samples[5] * ftemp1);
  VolAFE200msNoFltrCosSOS_Sum.Vbn += (AFE_new_samples[6] * ftemp1);
  VolAFE200msNoFltrCosSOS_Sum.Vcn += (AFE_new_samples[7] * ftemp1);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_THD_Sin_Cos_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        save_OC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Save One Cycle Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine saves the one cycle sum-of-squares values by transferring them from
//                      the .sum buffers to the .sav buffers.  The .sum buffers are then cleared.  This is
//                      done for the current, voltage, and power buffers.
//                      Note, the one-cycle current buffers are not cleared, because they are continuously
//                      updated on a per-sample basis.
//
//  CAVEATS:            None
// 
//  INPUTS:             CurOneCycSOS_SumF.Ix, VolAFEOneCycSOS_Sum.Vxn, VolADCOneCycSOS_Sum.Vxx,
//                      PwrOneCycSOS_Sum.Px, PwrOneCycSOS_Sum.RPx
// 
//  OUTPUTS:            CurOneCycSOS_Sav.Ix, VolAFEOneCycSOS_Sav.Vxx, VolADCOneCycSOS_Sav.Vxn,
//                      PwrOneCycSOS_Sav.Px, PwrOneCycSOS_Sav.RPx, VolAFE200msNoFltrSOS_Sum.Vxx
//
//  ALTERS:             VolAFEOneCycSOS_Sum.Vxn, VolADCOneCycSOS_Sum.Vxn, PwrOneCycSOS_Sum.Px,
//                      PwrOneCycSOS_Sum.RPx
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 190807 (rev 0.33 code): 1.71usec
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void save_OC_SOS(void)
{
  CurOneCycSOS_Sav.Ia = CurOneCycSOS_SumF.Ia;           // Save the sums of squares
  CurOneCycSOS_Sav.Ib = CurOneCycSOS_SumF.Ib;           // Note, current sums are not reset, because
  CurOneCycSOS_Sav.Ic = CurOneCycSOS_SumF.Ic;           //   they are continuously updated each sample
  CurOneCycSOS_Sav.In = CurOneCycSOS_SumF.In;
  CurOneCycSOS_Sav.Igsrc = CurOneCycSOS_SumF.Igsrc;
  CurOneCycSOS_Sav.Igres = CurOneCycSOS_SumF.Igres;                //                  LEFT OFF CHECKING HERE - STILL NEED TO FIX  update_THD_Sin_Cos_SOS()
  VolAFEOneCycSOS_Sav.Van = VolAFEOneCycSOS_Sum.Van;
  VolAFE200msNoFltrSOS_Sum.Van += VolAFEOneCycSOS_Sum.Van;
  VolAFEOneCycSOS_Sav.Vbn = VolAFEOneCycSOS_Sum.Vbn;
  VolAFE200msNoFltrSOS_Sum.Vbn += VolAFEOneCycSOS_Sum.Vbn;
  VolAFEOneCycSOS_Sav.Vcn = VolAFEOneCycSOS_Sum.Vcn;
  VolAFE200msNoFltrSOS_Sum.Vcn += VolAFEOneCycSOS_Sum.Vcn;
  VolAFEOneCycSOS_Sav.Vab = VolAFEOneCycSOS_Sum.Vab;
  VolAFE200msNoFltrSOS_Sum.Vab += VolAFEOneCycSOS_Sum.Vab;
  VolAFEOneCycSOS_Sav.Vbc = VolAFEOneCycSOS_Sum.Vbc;
  VolAFE200msNoFltrSOS_Sum.Vbc += VolAFEOneCycSOS_Sum.Vbc;
  VolAFEOneCycSOS_Sav.Vca = VolAFEOneCycSOS_Sum.Vca;
  VolAFE200msNoFltrSOS_Sum.Vca += VolAFEOneCycSOS_Sum.Vca;
  VolADCOneCycSOS_Sav.Van = VolADCOneCycSOS_Sum.Van;
  VolADCOneCycSOS_Sav.Vbn = VolADCOneCycSOS_Sum.Vbn;
  VolADCOneCycSOS_Sav.Vcn = VolADCOneCycSOS_Sum.Vcn;
  VolADCOneCycSOS_Sav.Vab = VolADCOneCycSOS_Sum.Vab;
  VolADCOneCycSOS_Sum.Vab = 0;
  VolADCOneCycSOS_Sav.Vbc = VolADCOneCycSOS_Sum.Vbc;
  VolADCOneCycSOS_Sav.Vca = VolADCOneCycSOS_Sum.Vca;
  PwrOneCycSOS_Sav.Pa = PwrOneCycSOS_Sum.Pa;
  PwrOneCycSOS_Sav.Pb = PwrOneCycSOS_Sum.Pb;
  PwrOneCycSOS_Sav.Pc = PwrOneCycSOS_Sum.Pc;
  PwrOneCycSOS_Sav.RPa = PwrOneCycSOS_Sum.RPa;
  PwrOneCycSOS_Sav.RPb = PwrOneCycSOS_Sum.RPb;
  PwrOneCycSOS_Sav.RPc = PwrOneCycSOS_Sum.RPc;

  VolAFEOneCycSOS_Sum.Van = 0;                                      // Reset the sums.  Note, the resets     
  VolAFEOneCycSOS_Sum.Vbn = 0;                                      //   are grouped together so that the    
  VolAFEOneCycSOS_Sum.Vcn = 0;                                      //   compiler sets a register to 0 once, 
  VolAFEOneCycSOS_Sum.Vab = 0;                                      //   then writes to each variable.  This 
  VolAFEOneCycSOS_Sum.Vbc = 0;                                      //   saves code space and execution time
  VolAFEOneCycSOS_Sum.Vca = 0;
  VolADCOneCycSOS_Sum.Van = 0;
  VolADCOneCycSOS_Sum.Vbn = 0;
  VolADCOneCycSOS_Sum.Vcn = 0;
  VolADCOneCycSOS_Sum.Vbc = 0;
  VolADCOneCycSOS_Sum.Vca = 0;
  PwrOneCycSOS_Sum.Pa = 0;
  PwrOneCycSOS_Sum.Pb = 0;
  PwrOneCycSOS_Sum.Pc = 0;
  PwrOneCycSOS_Sum.RPa = 0;
  PwrOneCycSOS_Sum.RPb = 0;
  PwrOneCycSOS_Sum.RPc = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        save_OC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        save_200msec_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Save 200msec Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine saves the 200msec sum-of-squares values by transferring them from the
//                      .sum buffers to the .sav buffers.  The .sum buffers are then cleared.  This is done
//                      for the current, voltage, and power buffers.
//
//  CAVEATS:            None
// 
//  INPUTS:             Cur200msFltrSOS_Sum.Ix, Cur200msNoFltrSOS_Sum.Ix, VolAFE200msFltrSOS_Sum.Vxx,
//                      VolAFE200msNoFltrSOS_Sum.Vxx, VolADC200msSOS_Sum.Vxn, Pwr200msecSOS_Sum.Px,
//                      Pwr200msecSOS_Sum.RPx, Cur200msNoFltrSinSOS_Sum.Ix, Cur200msNoFltrCosSOS_Sum.Ix
// 
//  OUTPUTS:            Cur200msFltrSOS_Sav.Ix, Cur200msNoFltrSOS_Sav.Ix, VolAFE200msFltrSOS_Sav.Vxn,
//                      VolAFE200msNoFltrSOS_Sav.Vxx, VolADC200msSOS_Sav.Vxx, Pwr200msecSOS_Sav.Px,
//                      Pwr200msecSOS_Sav.RPx
//                      
//
//  ALTERS:             Cur200msFltrSOS_Sum.Ix, VolAFE200msFltrSOS_Sum.Vxn, VolAFE200msNoFltrSOS_Sum.Vxx,
//                      VolAFE200msNoFltrSOS_Sav.Vxx, VolADC200msSOS_Sum.Vxn, Pwr200msecSOS_Sum.Px,
//                      Pwr200msecSOS_Sum.RPx, Cur200msNoFltrSinSOS_Sum.Ix, Cur200msNoFltrCosSOS_Sum.Ix
// 
//  CALLS:              None
// 
//  EXECUTION TIME:     Measured on 190724 (rev 0.32 code): 2.7usec
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void save_200msec_SOS(void)
{
  Cur200msFltrSOS_Sav.Ia = Cur200msFltrSOS_Sum.Ia;             // Save the sums of squares
  Cur200msFltrSOS_Sav.Ib = Cur200msFltrSOS_Sum.Ib;
  Cur200msFltrSOS_Sav.Ic = Cur200msFltrSOS_Sum.Ic;
  Cur200msFltrSOS_Sav.In = Cur200msFltrSOS_Sum.In;
  Cur200msFltrSOS_Sav.Igsrc = Cur200msFltrSOS_Sum.Igsrc;
  Cur200msFltrSOS_Sav.Igres = Cur200msFltrSOS_Sum.Igres;
  Cur200msNoFltrSOS_Sav.Ia = Cur200msNoFltrSOS_Sum.Ia;
  Cur200msNoFltrSOS_Sav.Ib = Cur200msNoFltrSOS_Sum.Ib;
  Cur200msNoFltrSOS_Sav.Ic = Cur200msNoFltrSOS_Sum.Ic;
  Cur200msNoFltrSOS_Sav.In = Cur200msNoFltrSOS_Sum.In;
  VolAFE200msFltrSOS_Sav.Van = VolAFE200msFltrSOS_Sum.Van;
  VolAFE200msFltrSOS_Sav.Vbn = VolAFE200msFltrSOS_Sum.Vbn;
  VolAFE200msFltrSOS_Sav.Vcn = VolAFE200msFltrSOS_Sum.Vcn;
  VolAFE200msFltrSOS_Sav.Vab = VolAFE200msFltrSOS_Sum.Vab;
  VolAFE200msFltrSOS_Sav.Vbc = VolAFE200msFltrSOS_Sum.Vbc;
  VolAFE200msFltrSOS_Sav.Vca = VolAFE200msFltrSOS_Sum.Vca;
  VolAFE200msNoFltrSOS_Sav.Van = VolAFE200msNoFltrSOS_Sum.Van;
  VolAFE200msNoFltrSOS_Sav.Vbn = VolAFE200msNoFltrSOS_Sum.Vbn;
  VolAFE200msNoFltrSOS_Sav.Vcn = VolAFE200msNoFltrSOS_Sum.Vcn;
  VolAFE200msNoFltrSOS_Sav.Vab = VolAFE200msNoFltrSOS_Sum.Vab;
  VolAFE200msNoFltrSOS_Sav.Vbc = VolAFE200msNoFltrSOS_Sum.Vbc;
  VolAFE200msNoFltrSOS_Sav.Vca = VolAFE200msNoFltrSOS_Sum.Vca;
  VolADC200msSOS_Sav.Van = VolADC200msSOS_Sum.Van;
  VolADC200msSOS_Sav.Vbn = VolADC200msSOS_Sum.Vbn;
  VolADC200msSOS_Sav.Vcn = VolADC200msSOS_Sum.Vcn;
  VolADC200msSOS_Sav.Vab = VolADC200msSOS_Sum.Vab;
  VolADC200msSOS_Sav.Vbc = VolADC200msSOS_Sum.Vbc;
  VolADC200msSOS_Sav.Vca = VolADC200msSOS_Sum.Vca;
  Pwr200msecSOS_Sav.Pa = Pwr200msecSOS_Sum.Pa;
  Pwr200msecSOS_Sav.Pb = Pwr200msecSOS_Sum.Pb;
  Pwr200msecSOS_Sav.Pc = Pwr200msecSOS_Sum.Pc;
  Pwr200msecSOS_Sav.RPa = Pwr200msecSOS_Sum.RPa;
  Pwr200msecSOS_Sav.RPb = Pwr200msecSOS_Sum.RPb;
  Pwr200msecSOS_Sav.RPc = Pwr200msecSOS_Sum.RPc;

  CurVol200msNoFltrSinSOS_Sav[0] = Cur200msNoFltrSinSOS_Sum.Ia;     // Save sin component of sums of squares
  CurVol200msNoFltrSinSOS_Sav[1] = Cur200msNoFltrSinSOS_Sum.Ib;
  CurVol200msNoFltrSinSOS_Sav[2] = Cur200msNoFltrSinSOS_Sum.Ic;
  CurVol200msNoFltrSinSOS_Sav[3] = Cur200msNoFltrSinSOS_Sum.In;
  CurVol200msNoFltrSinSOS_Sav[4] = VolAFE200msNoFltrSinSOS_Sum.Van;
  CurVol200msNoFltrSinSOS_Sav[5] = VolAFE200msNoFltrSinSOS_Sum.Vbn;
  CurVol200msNoFltrSinSOS_Sav[6] = VolAFE200msNoFltrSinSOS_Sum.Vcn;

  CurVol200msNoFltrCosSOS_Sav[0] = Cur200msNoFltrCosSOS_Sum.Ia;     // Save cos component of sums of squares
  CurVol200msNoFltrCosSOS_Sav[1] = Cur200msNoFltrCosSOS_Sum.Ib;
  CurVol200msNoFltrCosSOS_Sav[2] = Cur200msNoFltrCosSOS_Sum.Ic;
  CurVol200msNoFltrCosSOS_Sav[3] = Cur200msNoFltrCosSOS_Sum.In;
  CurVol200msNoFltrCosSOS_Sav[4] = VolAFE200msNoFltrCosSOS_Sum.Van;
  CurVol200msNoFltrCosSOS_Sav[5] = VolAFE200msNoFltrCosSOS_Sum.Vbn;
  CurVol200msNoFltrCosSOS_Sav[6] = VolAFE200msNoFltrCosSOS_Sum.Vcn;

  Cur200msFltrSOS_Sum.Ia = 0;                                       // Reset the sums.  Note, the resets
  Cur200msFltrSOS_Sum.Ib = 0;                                       //   are grouped together so that the
  Cur200msFltrSOS_Sum.Ic = 0;                                       //   compiler sets a register to 0 once,
  Cur200msFltrSOS_Sum.In = 0;                                       //   then writes to each variable.  This
  Cur200msFltrSOS_Sum.Igsrc = 0;                                    //   saves code space and execution time
  Cur200msFltrSOS_Sum.Igres = 0;
  Cur200msNoFltrSOS_Sum.Ia = 0;
  Cur200msNoFltrSOS_Sum.Ib = 0;
  Cur200msNoFltrSOS_Sum.Ic = 0;
  Cur200msNoFltrSOS_Sum.In = 0;
  VolAFE200msFltrSOS_Sum.Van = 0;
  VolAFE200msFltrSOS_Sum.Vbn = 0;
  VolAFE200msFltrSOS_Sum.Vcn = 0;
  VolAFE200msFltrSOS_Sum.Vab = 0;
  VolAFE200msFltrSOS_Sum.Vbc = 0;
  VolAFE200msFltrSOS_Sum.Vca = 0;
  VolAFE200msNoFltrSOS_Sum.Van = 0;
  VolAFE200msNoFltrSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrSOS_Sum.Vcn = 0;
  VolAFE200msNoFltrSOS_Sum.Vab = 0;
  VolAFE200msNoFltrSOS_Sum.Vbc = 0;
  VolAFE200msNoFltrSOS_Sum.Vca = 0;
  VolADC200msSOS_Sum.Van = 0;
  VolADC200msSOS_Sum.Vbn = 0;
  VolADC200msSOS_Sum.Vcn = 0;
  VolADC200msSOS_Sum.Vab = 0;
  VolADC200msSOS_Sum.Vbc = 0;
  VolADC200msSOS_Sum.Vca = 0;
  Pwr200msecSOS_Sum.Pa = 0;
  Pwr200msecSOS_Sum.Pb = 0;
  Pwr200msecSOS_Sum.Pc = 0;
  Pwr200msecSOS_Sum.RPa = 0;
  Pwr200msecSOS_Sum.RPb = 0;
  Pwr200msecSOS_Sum.RPc = 0;

  Cur200msNoFltrSinSOS_Sum.Ia = 0;                              // Reset the sin sums of squares
  Cur200msNoFltrSinSOS_Sum.Ib = 0;
  Cur200msNoFltrSinSOS_Sum.Ic = 0;
  Cur200msNoFltrSinSOS_Sum.In = 0;
  VolAFE200msNoFltrSinSOS_Sum.Van = 0;
  VolAFE200msNoFltrSinSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrSinSOS_Sum.Vcn = 0;

  Cur200msNoFltrCosSOS_Sum.Ia = 0;                              // Reset the cos sums of squares
  Cur200msNoFltrCosSOS_Sum.Ib = 0;
  Cur200msNoFltrCosSOS_Sum.Ic = 0;
  Cur200msNoFltrCosSOS_Sum.In = 0;
  VolAFE200msNoFltrCosSOS_Sum.Van = 0;
  VolAFE200msNoFltrCosSOS_Sum.Vbn = 0;
  VolAFE200msNoFltrCosSOS_Sum.Vcn = 0;
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        save_200msec_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        calc_min_max_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute Half-Cycle Current Min and Max Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine computes the minimum and maximum half-cycle sum-of-squares currents.
//
//  CAVEATS:            None
// 
//  INPUTS:             CurHalfCycSOS_SumF.Ix, CurHCSOS_Fmax.Ix, CurHCSOS_Fmin.Ix,
//                        x = a, b, c, n, gsrc, gres
// 
//  OUTPUTS:            CurHCSOS_Fmax.Ix, CurHCSOS_Fmin.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void calc_min_max_I_HC_SOS(void)
{
  CurHCSOS_Fmax.Ia = ((CurHalfCycSOS_SumF.Ia > CurHCSOS_Fmax.Ia) ? CurHalfCycSOS_SumF.Ia : CurHCSOS_Fmax.Ia);
  CurHCSOS_Fmin.Ia = ((CurHalfCycSOS_SumF.Ia < CurHCSOS_Fmin.Ia) ? CurHalfCycSOS_SumF.Ia : CurHCSOS_Fmin.Ia);
  CurHCSOS_Fmax.Ib = ((CurHalfCycSOS_SumF.Ib > CurHCSOS_Fmax.Ib) ? CurHalfCycSOS_SumF.Ib : CurHCSOS_Fmax.Ib);
  CurHCSOS_Fmin.Ib = ((CurHalfCycSOS_SumF.Ib < CurHCSOS_Fmin.Ib) ? CurHalfCycSOS_SumF.Ib : CurHCSOS_Fmin.Ib);
  CurHCSOS_Fmax.Ic = ((CurHalfCycSOS_SumF.Ic > CurHCSOS_Fmax.Ic) ? CurHalfCycSOS_SumF.Ic : CurHCSOS_Fmax.Ic);
  CurHCSOS_Fmin.Ic = ((CurHalfCycSOS_SumF.Ic < CurHCSOS_Fmin.Ic) ? CurHalfCycSOS_SumF.Ic : CurHCSOS_Fmin.Ic);
  CurHCSOS_Fmax.In = ((CurHalfCycSOS_SumF.In > CurHCSOS_Fmax.In) ? CurHalfCycSOS_SumF.In : CurHCSOS_Fmax.In);
  CurHCSOS_Fmin.In = ((CurHalfCycSOS_SumF.In < CurHCSOS_Fmin.In) ? CurHalfCycSOS_SumF.In : CurHCSOS_Fmin.In);
  CurHCSOS_Fmax.Igsrc = ((CurHalfCycSOS_SumF.Igsrc > CurHCSOS_Fmax.Igsrc) ? CurHalfCycSOS_SumF.Igsrc : CurHCSOS_Fmax.Igsrc);
  CurHCSOS_Fmin.Igsrc = ((CurHalfCycSOS_SumF.Igsrc < CurHCSOS_Fmin.Igsrc) ? CurHalfCycSOS_SumF.Igsrc : CurHCSOS_Fmin.Igsrc);
  CurHCSOS_Fmax.Igres = ((CurHalfCycSOS_SumF.Igres > CurHCSOS_Fmax.Igres) ? CurHalfCycSOS_SumF.Igres : CurHCSOS_Fmax.Igres);
  CurHCSOS_Fmin.Igres = ((CurHalfCycSOS_SumF.Igres < CurHCSOS_Fmin.Igres) ? CurHalfCycSOS_SumF.Igres : CurHCSOS_Fmin.Igres);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        calc_min_max_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        calc_min_max_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute One-Cycle Current Min and Max Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine computes the minimum and maximum one-cycle sum-of-squares currents.
//
//  CAVEATS:            None
// 
//  INPUTS:             CurOneCycSOS_SumF.Ix, CurOCSOS_Fmax.Ix, CurOCSOS_Fmin.Ix, x = a, b, c, n, gsrc, gres
// 
//  OUTPUTS:            CurOCSOS_Fmax.Ix, CurOCSOS_Fmin.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void calc_min_max_I_OC_SOS(void)
{
  CurOCSOS_Fmax.Ia = ((CurOneCycSOS_SumF.Ia > CurOCSOS_Fmax.Ia) ? CurOneCycSOS_SumF.Ia : CurOCSOS_Fmax.Ia);
  CurOCSOS_Fmin.Ia = ((CurOneCycSOS_SumF.Ia < CurOCSOS_Fmin.Ia) ? CurOneCycSOS_SumF.Ia : CurOCSOS_Fmin.Ia);
  CurOCSOS_Fmax.Ib = ((CurOneCycSOS_SumF.Ib > CurOCSOS_Fmax.Ib) ? CurOneCycSOS_SumF.Ib : CurOCSOS_Fmax.Ib);
  CurOCSOS_Fmin.Ib = ((CurOneCycSOS_SumF.Ib < CurOCSOS_Fmin.Ib) ? CurOneCycSOS_SumF.Ib : CurOCSOS_Fmin.Ib);
  CurOCSOS_Fmax.Ic = ((CurOneCycSOS_SumF.Ic > CurOCSOS_Fmax.Ic) ? CurOneCycSOS_SumF.Ic : CurOCSOS_Fmax.Ic);
  CurOCSOS_Fmin.Ic = ((CurOneCycSOS_SumF.Ic < CurOCSOS_Fmin.Ic) ? CurOneCycSOS_SumF.Ic : CurOCSOS_Fmin.Ic);
  CurOCSOS_Fmax.In = ((CurOneCycSOS_SumF.In > CurOCSOS_Fmax.In) ? CurOneCycSOS_SumF.In : CurOCSOS_Fmax.In);
  CurOCSOS_Fmin.In = ((CurOneCycSOS_SumF.In < CurOCSOS_Fmin.In) ? CurOneCycSOS_SumF.In : CurOCSOS_Fmin.In);
  CurOCSOS_Fmax.Igsrc = ((CurOneCycSOS_SumF.Igsrc > CurOCSOS_Fmax.Igsrc) ? CurOneCycSOS_SumF.Igsrc : CurOCSOS_Fmax.Igsrc);
  CurOCSOS_Fmin.Igsrc = ((CurOneCycSOS_SumF.Igsrc < CurOCSOS_Fmin.Igsrc) ? CurOneCycSOS_SumF.Igsrc : CurOCSOS_Fmin.Igsrc);
  CurOCSOS_Fmax.Igres = ((CurOneCycSOS_SumF.Igres > CurOCSOS_Fmax.Igres) ? CurOneCycSOS_SumF.Igres : CurOCSOS_Fmax.Igres);
  CurOCSOS_Fmin.Igres = ((CurOneCycSOS_SumF.Igres < CurOCSOS_Fmin.Igres) ? CurOneCycSOS_SumF.Igres : CurOCSOS_Fmin.Igres);
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        calc_min_max_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        calc_max_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute Maximum Half-Cycle Current Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine:
//                        1) converts the integer half-cycle sum-of-squares to floating point
//                        2) computes the maximum half-cycle sum-of-squares currents
//                      Note, the maximum current is used only for short delay and instantaneous protection,
//                        so it does not include the ground currents
//
//                      Scale the Neutral Half Cycle SOS and new sample values based on the Neutral Ratio setpoint.
//
//  CAVEATS:            None
// 
//  INPUTS:             CurHalfCycSOS_SumI.Ix  x = a, b, c, n, gsrc, gres
//                      Setpoints1.stp.NeutRatio [0, 60, 100] %
//
//  OUTPUTS:            CurHalfCycSOSmax, CurHalfCycSOS_SumF.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void calc_max_I_HC_SOS(void)
{
  float ftemp1, ftemp2;

  CurHalfCycSOS_SumF.Ia = ((float)CurHalfCycSOS_SumI.Ia)/100;
  CurHalfCycSOS_SumF.Ib = ((float)CurHalfCycSOS_SumI.Ib)/100;
  CurHalfCycSOS_SumF.Ic = ((float)CurHalfCycSOS_SumI.Ic)/100;
  CurHalfCycSOS_SumF.In = (float)CurHalfCycSOS_SumI.In/100;
  CurHalfCycSOS_SumF.Igsrc = ((float)CurHalfCycSOS_SumI.Igsrc)/100;
  CurHalfCycSOS_SumF.Igres = ((float)CurHalfCycSOS_SumI.Igres)/100;
  CurHalfCycSOSmax = CurHalfCycSOS_SumF.Ia;
  if (CurHalfCycSOS_SumF.Ib > CurHalfCycSOSmax)
  {
    CurHalfCycSOSmax = CurHalfCycSOS_SumF.Ib;
  }
  if (CurHalfCycSOS_SumF.Ic > CurHalfCycSOSmax)
  {
    CurHalfCycSOSmax = CurHalfCycSOS_SumF.Ic;
  }

  switch (Setpoints1.stp.NeutRatio)    // Setpoints1.stp.NeutRatio can be 0, 60%, or 100% so scale accordingly
  {
    case 0:                            // multiply by 0
      ftemp1 = 0;
      ftemp2 = 0;
     break;

    case 60:                           // multiply by 1/.36 for SOS and 1/.6 for new sample
      ftemp1 = CurHalfCycSOS_SumF.In * 2.778;
      ftemp2 = AFE_new_samples[3] * 1.667;
     break;

    case 100:                          // use as is.
      ftemp1 = CurHalfCycSOS_SumF.In;
      ftemp2 = AFE_new_samples[3];
     break;

    case 200:                           // multiply by 1/4 for SOS and 1/2 for new sample
      ftemp1 = CurHalfCycSOS_SumF.In * 0.25;
      ftemp2 = AFE_new_samples[3] * 0.5;
     break;     
     
     default:                          // use as is.
      ftemp1 = CurHalfCycSOS_SumF.In;
      ftemp2 = AFE_new_samples[3];
     break;
  }
  
  if (ftemp1 > CurHalfCycSOSmax)
  {
    CurHalfCycSOSmax = ftemp1;
    NewSample = ftemp2;
    NmaxFlg = 1;                      // Neutral is highest of the 4 phases
  }
  else
  {
    NmaxFlg = 0;
  }

}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        calc_max_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        calc_max_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Compute Maximum One-Cycle Current Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine:
//                        1) converts the integer one-cycle sum-of-squares to floating point
//                        2) computes the maximum one-cycle sum-of-squares currents
//                        2) saves the latest sample of the maximum current for use in the short delay
//                           protection subroutine
//
//                      Scale the Neutral SOS and new sample values based on the Neutral Ratio setpoint.
//
//  CAVEATS:            None
// 
//  INPUTS:             CurOneCycSOS_SumI.Ix  x = a, b, c, n, gsrc, gres
//                      Setpoints1.stp.NeutRatio [0, 60, 100] %
//
//  OUTPUTS:            CurHalfCycSOSmax, NewSample, CurOneCycSOS_SumF.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void calc_max_I_OC_SOS(void)
{
  float ftemp1, ftemp2;

  CurOneCycSOS_SumF.Ia = ((float)CurOneCycSOS_SumI.Ia)/100;
  CurOneCycSOS_SumF.Ib = ((float)CurOneCycSOS_SumI.Ib)/100;
  CurOneCycSOS_SumF.Ic = ((float)CurOneCycSOS_SumI.Ic)/100;
  CurOneCycSOS_SumF.In = ((float)CurOneCycSOS_SumI.In)/100;
  CurOneCycSOS_SumF.Igsrc = ((float)CurOneCycSOS_SumI.Igsrc)/100;
  CurOneCycSOS_SumF.Igres = ((float)CurOneCycSOS_SumI.Igres)/100;
  CurOneCycSOSmax = CurOneCycSOS_SumF.Ia;
  NewSample = AFE_new_samples[0];
  if (CurOneCycSOS_SumF.Ib > CurOneCycSOSmax)
  {
    CurOneCycSOSmax = CurOneCycSOS_SumF.Ib;
    NewSample = AFE_new_samples[1];
  }
  if (CurOneCycSOS_SumF.Ic > CurOneCycSOSmax)
  {
    CurOneCycSOSmax = CurOneCycSOS_SumF.Ic;
    NewSample = AFE_new_samples[2];
  }
  switch (Setpoints1.stp.NeutRatio)    // Setpoints1.stp.NeutRatio can be 0, 60%, or 100% so scale accordingly
  {
    case 0:                            // multiply by 0
      ftemp1 = 0;
      ftemp2 = 0;
     break;

    case 60:                           // multiply by 1/.36 for SOS and 1/.6 for new sample
      ftemp1 = CurOneCycSOS_SumF.In * 2.778;
      ftemp2 = AFE_new_samples[3] * 1.667;
     break;

    case 100:                          // use as is.
      ftemp1 = CurOneCycSOS_SumF.In;
      ftemp2 = AFE_new_samples[3];
     break;
     
    case 200:                           // multiply by 1/4 for SOS and 1/2 for new sample
      ftemp1 = CurOneCycSOS_SumF.In * 0.25;
      ftemp2 = AFE_new_samples[3] * 0.5;
     break;

     default:                          // use as is.
      ftemp1 = CurOneCycSOS_SumF.In;
      ftemp2 = AFE_new_samples[3];
     break;
  }
  ScaledCurOneCycSOS_SumF_In = ftemp1;    // Used in Neutral_Alarm code in Prot.c

  if (ftemp1 > CurOneCycSOSmax)
  {
    CurOneCycSOSmax = ftemp1;
    NewSample = ftemp2;
    NmaxFlg = 1;                       // Neutral is highest of the 4 phases
  }
  else
  {
    NmaxFlg = 0;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        calc_max_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Half-Cycle Current Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine subtracts the square of the oldest sample from half-cycle
//                      sum-of-squares for the currents
//
//  CAVEATS:            None
// 
//  INPUTS:             HalfCycInd, HalfCycSamplesSq[x][y], x = 0..6 for Ia..Igres, y=index of oldest sample
// 
//  OUTPUTS:            CurHalfCycSOS_SumI.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_I_HC_SOS(void)
{
  // Subtract the square of the oldest 1/2-cycle sample from the sums of squares for the 1/2-cycle currents.
  //   The oldest sample is indexed by HalfCycInd
  CurHalfCycSOS_SumI.Ia -= HalfCycSamplesSq[0][HalfCycInd];
  CurHalfCycSOS_SumI.Ib -= HalfCycSamplesSq[1][HalfCycInd];
  CurHalfCycSOS_SumI.Ic -= HalfCycSamplesSq[2][HalfCycInd];
  CurHalfCycSOS_SumI.In -= HalfCycSamplesSq[3][HalfCycInd];
  CurHalfCycSOS_SumI.Igsrc -= HalfCycSamplesSq[4][HalfCycInd];
  CurHalfCycSOS_SumI.Igres -= HalfCycSamplesSq[5][HalfCycInd];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_I_HC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update One-Cycle Current Sum-of-Squares In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine subtracts the square of the oldest sample from one-cycle
//                      sum-of-squares for the currents
//
//  CAVEATS:            None
// 
//  INPUTS:             OneCycInd, OneCycSamplesSq[x][y], x = 0..6 for Ia..Igres, y=index of oldest sample
// 
//  OUTPUTS:            CurOneCycSOS_SumI.Ix,  x = a, b, c, n, gsrc, gres
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_I_OC_SOS(void)
{
  // Subtract the square of the oldest 1-cycle sample from the sums of squares for the 1-cycle currents.
  //   The oldest sample is indexed by OneCycInd.
  CurOneCycSOS_SumI.Ia -= OneCycSamplesSq[0][OneCycInd];
  CurOneCycSOS_SumI.Ib -= OneCycSamplesSq[1][OneCycInd];
  CurOneCycSOS_SumI.Ic -= OneCycSamplesSq[2][OneCycInd];
  CurOneCycSOS_SumI.In -= OneCycSamplesSq[3][OneCycInd];
  CurOneCycSOS_SumI.Igsrc -= OneCycSamplesSq[4][OneCycInd];
  CurOneCycSOS_SumI.Igres -= OneCycSamplesSq[5][OneCycInd];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_I_OC_SOS()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        update_delayed_vbuf()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Update Delayed Voltage Sample Buffers In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine updates the delayed voltage sample buffers by storing the latest
//                      voltage samples at index DelayedVoltsNdx.  The delayed voltage sample buffers are
//                      used to delay the voltage samples by 90 degrees (20 samples), for use in reactive
//                      power calculations.
//
//  CAVEATS:            None
// 
//  INPUTS:             DelayedVoltsNdx, AFE_new_samples[5..7], MTR_new_samples[5..7]
// 
//  OUTPUTS:            DelayedVolts_OC.Vxx[DelayedVoltsNdx],  xx = an, bn, cn
//                      DelayedVolts_200msec.Vxx[DelayedVoltsNdx],  xx = an, bn, cn
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void update_delayed_vbuf(void)
{
  // Update the delayed voltage sample buffers.  These buffers are used to delay the voltage samples by
  //   90 degrees (20 samples), for use in reactive power calculations
  DelayedVolts_OC.Van[DelayedVoltsNdx] = AFE_new_samples[5];
  DelayedVolts_OC.Vbn[DelayedVoltsNdx] = AFE_new_samples[6];
  DelayedVolts_OC.Vcn[DelayedVoltsNdx] = AFE_new_samples[7];
  DelayedVolts_200msec.Van[DelayedVoltsNdx] = MTR_new_samples[5];
  DelayedVolts_200msec.Vbn[DelayedVoltsNdx] = MTR_new_samples[6];
  DelayedVolts_200msec.Vcn[DelayedVoltsNdx] = MTR_new_samples[7];
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        update_delayed_vbuf()
//------------------------------------------------------------------------------------------------------------



//------------------------------------------------------------------------------------------------------------
//             START OF FUNCTION        inc_buf_indices()
//------------------------------------------------------------------------------------------------------------
//
//  FUNCTION:           Increment Buffer Indices In-Line Subroutine
// 
//  MECHANICS:          This subroutine is jumped to from the DMA1_Stream0_IRQ (sampling) interrupt service
//                      routine.  It is defined as an in-line subroutine, and so is jumped to, not called
//                      to save overhead time.  This subroutine is used to improve the readability of the
//                      sampling interrupt service routine.
//                      The subroutine increments the One-Cycle, Half-Cycle, Delayed-Voltage buffer, and
//                      200msec indices.
//
//  CAVEATS:            None
// 
//  INPUTS:             None
// 
//  OUTPUTS:            OneCycInd, HalfCycInd, DelayedVoltsNdx, msec200Ctr
//
//  ALTERS:             None
// 
//  CALLS:              None
// 
//------------------------------------------------------------------------------------------------------------

__attribute__(( always_inline )) inline void inc_buf_indices(void)
{
  // Increment the buffer indices
  if (OneCycInd >= 79)
  {
    OneCycInd = 0;
  }
  else
  {
    OneCycInd++;
  }
  if (HalfCycInd >= 39)
  {
    HalfCycInd = 0;
  }
  else
  {
    HalfCycInd++;
  }
  if (DelayedVoltsNdx >= 19)
  {
    DelayedVoltsNdx = 0;
  }
  else
  {
    DelayedVoltsNdx++;
  }
  if (msec200Ctr >= 959)
  {
    msec200Ctr = 0;
  }
  else
  {
    msec200Ctr++;
  }
  if (OneSecCtr >= 4799)
  {
    OneSecCtr = 0;
  }
  else
  {
    OneSecCtr++;
  }
}

//------------------------------------------------------------------------------------------------------------
//             END OF FUNCTION        inc_buf_indices()
//------------------------------------------------------------------------------------------------------------

