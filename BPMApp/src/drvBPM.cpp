/*
 * drvBPM.cpp
 *
 * Authors: Juliano Murari
 *          Lucas Russo
 *
 * Created Jul. 13, 2015
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>

#include "drvBPM.h"
#include "position.h"
#include <epicsExport.h>

/** The polling interval when checking to see if acquisition is complete */
#define BPM_POLL_TIME                   .1
#define BPM_SP_POLL_TIME                .01
#define BPM_PM_POLL_TIME                1

#define PI                              3.14159265
#define FREQ_SAMPLE                     100.00              /* Hz */
#define FREQ                            1.00                /* Hz */
#define TEST_LENGTH                     4092

/* FIXME: This should be read from hardware */
#define HARMONIC_NUMBER                 148
#define ADC_CLK_FREQ_UVX_DFLT           113040445       /* Hz */
#define ADC_SI57X_FSTARTUP_DFLT         155490000       /* Hz */
#define ADC_RATE_FACTOR                 1
#define TBT_RATE_FACTOR                 35
#define FOFB_RATE_FACTOR                980
#define MONIT_RATE_FACTOR               9800000
#define MONIT1_RATE_FACTOR              98000

#define ADC_DFLT_SW                     0x1             /* No switching. Direct state */
#define ADC_DFLT_DIV_CLK                980             /* in ADC counts */
#define FMC_REF_CLK_SEL_1               0
#define AD9510_ADC_DFLT_B_DIV           35
#define AD9510_ADC_DFLT_MUX_STATUS      1
#define AD9510_ADC_DFLT_CP_CURRENT      600
#define AD9510_ADC_DFLT_OUTPUTS         31

#define AFC_SI57X_FREQ_DFLT             100000000       /* Hz */
#define AFC_SI57X_FSTARTUP_DFLT         100000000       /* Hz */

#define FMCPICO_1MA_SCALE               1

#define TIMRCV_DFLT_PHASE_MEAS_NAVG     10
#define TIMRCV_DFLT_DMTD_A_DEGLITCH_THRES 10
#define TIMRCV_DFLT_DMTD_B_DEGLITCH_THRES 10

#define CH_DFLT_TRIGGER_CHAN            0
#define ADC_RST_NORMAL_OP               1
#define ADC_NUM_CHANNELS                4
#define CH_DFLT_TRIGGER_SW_CHAN         18

#define CH_DFLT_TRIGGER_SEL_PM_CHAN     1
/* Number fo triggers that are relative to waveform acquisition */
#define MAX_WAVEFORM_TRIGGERS           17

#define CH_DEFAULT_PM                   CH_TBT
#define SAMPLES_PRE_DEFAULT_PM(maxPoints) \
                                        (maxPoints/2)
#define SAMPLES_POST_DEFAULT_PM(maxPoints) \
                                        (maxPoints/2)
#define NUM_SHOTS_DEFAULT_PM            1
#define TRIG_DEFAULT_PM                 ACQ_CLIENT_TRIG_EXTERNAL
#define DFLT_SAMPLE_SIZE                8 /* in bytes */
#define DFLT_NUM_ATOMS                  4
#define DFLT_ATOM_WIDTH                 2 /* in bytes */

#define SERVICE_NAME_SIZE               50

typedef struct {
    drvBPM *drvBPMp;
    bpm_coreID_types coreID;
    double pollTime;
    bool autoStart;
} taskParams_t;

static const boardMap_t boardMap[MAX_BPMS+1] = {
         /* board, bpm*/
    /* 0 (INVALID)  */ {-1, -1,  -1},
    /* 1            */ {1,   0,   2},
    /* 2            */ {1,   1,   3},
    /* 3            */ {2,   0,   2},
    /* 4            */ {2,   1,   3},
    /* 5            */ {3,   0,   2},
    /* 6            */ {3,   1,   3},
    /* 7            */ {4,   0,   2},
    /* 8            */ {4,   1,   3},
    /* 9            */ {5,   0,   2},
    /* 10           */ {5,   1,   3},
    /* 11           */ {6,   0,   2},
    /* 12           */ {6,   1,   3},
    /* 13           */ {7,   0,   2},
    /* 14           */ {7,   1,   3},
    /* 15           */ {8,   0,   2},
    /* 16           */ {8,   1,   3},
    /* 17           */ {9,   0,   2},
    /* 18           */ {9,   1,   3},
    /* 19           */ {10,  0,   2},
    /* 20           */ {10,  1,   3},
    /* 21           */ {11,  0,   2},
    /* 22           */ {11,  1,   3},
    /* 23           */ {12,  0,   2},
    /* 24           */ {12,  1,   3}
};

static const channelMap_t channelMap[CH_END] = {
                        /* Amp, Phase, Pos, AmpA, AmpB, AmpC, AmpD, AmpALL */
    /* [CH_ADC] =     */ {CH_HW_ADC,                           // HwAmpChannel
                          -1,                                  // HwPhaseChannel
                          -1,                                  // HwPosChannel
                          0,                                   // CalcPos
                          {{WVF_GENAMP_A,                      // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                           {WVF_AMP_PM_A,
                            WVF_AMP_PM_B,
                            WVF_AMP_PM_C,
                            WVF_AMP_PM_D,
                            WVF_AMP_PM_ALL},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{-1,                                 // NDArrayPos
                            -1,
                            -1,
                            -1,
                            -1},
                           {-1,
                            -1,
                            -1,
                            -1,
                            -1},
                          },
                          },
    /* [CH_ADCSWAP] = */ {CH_HW_ADCSWAP,                        // HwAmpChannel
                          -1,                                   // HwPhaseChannel
                          -1,                                   // HwPosChannel
                          0,                                    // CalcPos
                          {{WVF_GENAMP_A,                       // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                           {WVF_AMP_PM_A,
                            WVF_AMP_PM_B,
                            WVF_AMP_PM_C,
                            WVF_AMP_PM_D,
                            WVF_AMP_PM_ALL},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{-1,                                 // NDArrayPos
                            -1,
                            -1,
                            -1,
                            -1},
                           {-1,
                            -1,
                            -1,
                            -1,
                            -1},
                          },
                          },
    /* [CH_TBT] =     */ {CH_HW_TBT,                             // HwAmpChannel
                          -1,                                    // HwPhaseChannel
                          -1,                                    // HwPosChannel
                          1,                                     // CalcPos
                          {{WVF_GENAMP_A,                        // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                            {WVF_AMP_PM_A,
                             WVF_AMP_PM_B,
                             WVF_AMP_PM_C,
                             WVF_AMP_PM_D,
                             WVF_AMP_PM_ALL},
                          },
                          {{-1,                                  // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{WVF_GENPOS_A,                        // NDArrayPos
                            WVF_GENPOS_B,
                            WVF_GENPOS_C,
                            WVF_GENPOS_D,
                            WVF_GENPOS_ALL},
                            {WVF_POS_PM_A,
                             WVF_POS_PM_B,
                             WVF_POS_PM_C,
                             WVF_POS_PM_D,
                             WVF_POS_PM_ALL},
                          },
                          },
    /* [CH_FOFB] =    */ {CH_HW_FOFB,                            // HwAmpChannel
                          -1,                                    // HwPhaseChannel
                          -1,                                    // HwPosChannel
                          1,                                     // CalcPos
                          {{WVF_GENAMP_A,                        // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                           {WVF_AMP_PM_A,
                            WVF_AMP_PM_B,
                            WVF_AMP_PM_C,
                            WVF_AMP_PM_D,
                            WVF_AMP_PM_ALL},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{WVF_GENPOS_A,                        // NDArrayPos
                            WVF_GENPOS_B,
                            WVF_GENPOS_C,
                            WVF_GENPOS_D,
                            WVF_GENPOS_ALL},
                           {WVF_POS_PM_A,
                            WVF_POS_PM_B,
                            WVF_POS_PM_C,
                            WVF_POS_PM_D,
                            WVF_POS_PM_ALL},
                          },
                          },
    /* [CH_TBTPHA] =     */ {CH_HW_TBTPHA,                       // HwAmpChannel
                          -1,                                    // HwPhaseChannel
                          -1,                                    // HwPosChannel
                          1,                                     // CalcPos
                          {{WVF_GENAMP_A,                        // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                            {WVF_AMP_PM_A,
                             WVF_AMP_PM_B,
                             WVF_AMP_PM_C,
                             WVF_AMP_PM_D,
                             WVF_AMP_PM_ALL},
                          },
                          {{-1,                                  // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{WVF_GENPOS_A,                        // NDArrayPos
                            WVF_GENPOS_B,
                            WVF_GENPOS_C,
                            WVF_GENPOS_D,
                            WVF_GENPOS_ALL},
                            {WVF_POS_PM_A,
                             WVF_POS_PM_B,
                             WVF_POS_PM_C,
                             WVF_POS_PM_D,
                             WVF_POS_PM_ALL},
                          },
                          },
    /* [CH_FOFBPHA] =    */ {CH_HW_FOFBPHA,                      // HwAmpChannel
                          -1,                                    // HwPhaseChannel
                          -1,                                    // HwPosChannel
                          1,                                     // CalcPos
                          {{WVF_GENAMP_A,                        // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                           {WVF_AMP_PM_A,
                            WVF_AMP_PM_B,
                            WVF_AMP_PM_C,
                            WVF_AMP_PM_D,
                            WVF_AMP_PM_ALL},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{WVF_GENPOS_A,                        // NDArrayPos
                            WVF_GENPOS_B,
                            WVF_GENPOS_C,
                            WVF_GENPOS_D,
                            WVF_GENPOS_ALL},
                           {WVF_POS_PM_A,
                            WVF_POS_PM_B,
                            WVF_POS_PM_C,
                            WVF_POS_PM_D,
                            WVF_POS_PM_ALL},
                          },
                          },
    /* [CH_MONIT1] =  */ {CH_HW_MONIT1,                          // HwAmpChannel
                          -1,                                    // HwPhaseChannel
                          -1,                                    // HwPosChannel
                          1,                                     // CalcPos
                          {{WVF_GENAMP_A,                        // NDArrayAmp
                            WVF_GENAMP_B,
                            WVF_GENAMP_C,
                            WVF_GENAMP_D,
                            WVF_GENAMP_ALL},
                           {-1,
                            -1,
                            -1,
                            -1,
                            -1},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{WVF_GENPOS_A,                        // NDArrayPos
                            WVF_GENPOS_B,
                            WVF_GENPOS_C,
                            WVF_GENPOS_D,
                            WVF_GENPOS_ALL},
                           {-1,
                            -1,
                            -1,
                            -1,
                            -1},
                          },
                          },
    /* [CH_SP] =      */ {CH_HW_ADC,                           // HwAmpChannel
                          -1,                                  // HwPhaseChannel
                          -1,                                  // HwPosChannel
                          0,                                   // CalcPos
                          {{WVF_AMP_SP_A,                      // NDArrayAmp
                            WVF_AMP_SP_B,
                            WVF_AMP_SP_C,
                            WVF_AMP_SP_D,
                            WVF_AMP_SP_ALL},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{-1,                                 // NDArrayPhase
                            -1,
                            -1,
                            -1,
                            -1},
                            {-1,
                             -1,
                             -1,
                             -1,
                             -1},
                          },
                          {{-1,                                 // NDArrayPos
                            -1,
                            -1,
                            -1,
                            -1},
                           {-1,
                            -1,
                            -1,
                            -1,
                            -1},
                          },
                          },
};

/* FIXME: This reverse mapping must match the maximum HwAmpChannel for ChannelMap */
static const channelRevMap_t channelRevMap[CH_HW_END] = {
                        /* EPICS channel */
     /* [CH_HW_ADC] =       */  {CH_ADC},
     /* [CH_HW_ADCSWAP] =   */  {CH_ADCSWAP},
     /* 2 = Unavailable     */  {-1},
     /* 3 = Unavailable     */  {-1},
     /* 4 = Unavailable     */  {-1},
     /* 5 = Unavailable     */  {-1},
     /* [CH_HW_TBT] =       */  {CH_TBT},
     /* [CH_HW_TBTPHA] =    */  {CH_TBTPHA},
     /* 8 = Unavailable     */  {-1},
     /* 9 = Unavailable     */  {-1},
     /* 10 = Unavailable    */  {-1},
     /* [CH_HW_FOFB] =      */  {CH_FOFB},
     /* [CH_HW_FOFBPHA] =   */  {CH_FOFBPHA},
     /* 13 = Unavailable    */  {-1},
     /* [CH_HW_MONIT1] =    */  {CH_MONIT1},
};

/* This function should not be called, as there is no client function to replace it and
 * the EPICS Db should not export PVs that maps here.
 * FIXME: not sure why, but some unavailable functions are called even with no
 * "apperently" Db record mapped to it. When this happens, segfault occurs. So,
 * until we figure out what s happening we keep "NULL" function mapped to this dummy
 * fcuntions */
static halcs_client_err_e halcs_dummy_read_32 (halcs_client_t *self, char *service, uint32_t *param)
{
    (void) self;
    (void) service;
    (void) param;
    return HALCS_CLIENT_ERR_INV_FUNCTION;
}

static halcs_client_err_e halcs_dummy_read_chan_32 (halcs_client_t *self, char *service,
        uint32_t chan, uint32_t *param)
{
    (void) self;
    (void) service;
    (void) chan;
    (void) param;
    return HALCS_CLIENT_ERR_INV_FUNCTION;
}

/* Int32 functions mapping */
static const functionsAny_t bpmSetGetKxFunc =                    {functionsUInt32_t{"DSP", halcs_set_kx, halcs_get_kx}};
static const functionsAny_t bpmSetGetKyFunc =                    {functionsUInt32_t{"DSP", halcs_set_ky, halcs_get_ky}};
static const functionsAny_t bpmSetGetKsumFunc =                  {functionsUInt32_t{"DSP", halcs_set_ksum, halcs_get_ksum}};
static const functionsAny_t bpmSetGetSwTagEnFunc =               {functionsUInt32_t{"DSP", halcs_set_sw_tag_en, halcs_get_sw_tag_en}};
static const functionsAny_t bpmSetGetSwDataMaskEnFunc =          {functionsUInt32_t{"DSP", halcs_set_sw_data_mask_en,
                                                                                            halcs_get_sw_data_mask_en}};
static const functionsAny_t bpmSetGetSwDataMaskSamplesFunc =     {functionsUInt32_t{"DSP", halcs_set_sw_data_mask_samples,
                                                                                            halcs_get_sw_data_mask_samples}};
static const functionsAny_t bpmSetGetTbtTagEnFunc =              {functionsUInt32_t{"DSP", halcs_set_tbt_tag_en, halcs_get_tbt_tag_en}};
static const functionsAny_t bpmSetGetTbtTagDlyFunc =             {functionsUInt32_t{"DSP", halcs_set_tbt_tag_dly, halcs_get_tbt_tag_dly}};
static const functionsAny_t bpmSetGetTbtDataMaskEnFunc =         {functionsUInt32_t{"DSP", halcs_set_tbt_data_mask_en,
                                                                                            halcs_get_tbt_data_mask_en}};
static const functionsAny_t bpmSetGetTbtDataMaskSamplesBegFunc = {functionsUInt32_t{"DSP", halcs_set_tbt_data_mask_samples_beg,
                                                                                         halcs_get_tbt_data_mask_samples_beg}};
static const functionsAny_t bpmSetGetTbtDataMaskSamplesEndFunc = {functionsUInt32_t{"DSP", halcs_set_tbt_data_mask_samples_end,
                                                                                            halcs_get_tbt_data_mask_samples_end}};
static const functionsAny_t bpmSetGetSwTagDesyncCntRstFunc =     {functionsUInt32_t{"DSP", halcs_set_sw_tag_desync_cnt_rst,
                                                                                            halcs_get_sw_tag_desync_cnt_rst}};
static const functionsAny_t bpmSetGetSwTagDesyncCntFunc =        {functionsUInt32_t{"DSP", NULL,
                                                                                            halcs_get_sw_tag_desync_cnt}};
static const functionsAny_t bpmSetGetTbtTagDesyncCntRstFunc =    {functionsUInt32_t{"DSP", halcs_set_tbt_tag_desync_cnt_rst,
                                                                                           halcs_get_tbt_tag_desync_cnt_rst}};
static const functionsAny_t bpmSetGetTbtTagDesyncCntFunc =       {functionsUInt32_t{"DSP", NULL,
                                                                                            halcs_get_tbt_tag_desync_cnt}};
static const functionsAny_t bpmSetGetMonit1TagEnFunc =              {functionsUInt32_t{"DSP", halcs_set_monit1_tag_en, halcs_get_monit1_tag_en}};
static const functionsAny_t bpmSetGetMonit1TagDlyFunc =             {functionsUInt32_t{"DSP", halcs_set_monit1_tag_dly, halcs_get_monit1_tag_dly}};
static const functionsAny_t bpmSetGetMonit1DataMaskEnFunc =         {functionsUInt32_t{"DSP", halcs_set_monit1_data_mask_en,
                                                                                            halcs_get_monit1_data_mask_en}};
static const functionsAny_t bpmSetGetMonit1DataMaskSamplesBegFunc = {functionsUInt32_t{"DSP", halcs_set_monit1_data_mask_samples_beg,
                                                                                         halcs_get_monit1_data_mask_samples_beg}};
static const functionsAny_t bpmSetGetMonit1DataMaskSamplesEndFunc = {functionsUInt32_t{"DSP", halcs_set_monit1_data_mask_samples_end,
                                                                                            halcs_get_monit1_data_mask_samples_end}};
static const functionsAny_t bpmSetGetMonit1TagDesyncCntRstFunc =    {functionsUInt32_t{"DSP", halcs_set_monit1_tag_desync_cnt_rst,
                                                                                           halcs_get_monit1_tag_desync_cnt_rst}};
static const functionsAny_t bpmSetGetMonit1TagDesyncCntFunc =       {functionsUInt32_t{"DSP", NULL,
                                                                                            halcs_get_monit1_tag_desync_cnt}};
static const functionsAny_t bpmSetGetMonitTagEnFunc =              {functionsUInt32_t{"DSP", halcs_set_monit_tag_en, halcs_get_monit_tag_en}};
static const functionsAny_t bpmSetGetMonitTagDlyFunc =             {functionsUInt32_t{"DSP", halcs_set_monit_tag_dly, halcs_get_monit_tag_dly}};
static const functionsAny_t bpmSetGetMonitDataMaskEnFunc =         {functionsUInt32_t{"DSP", halcs_set_monit_data_mask_en,
                                                                                            halcs_get_monit_data_mask_en}};
static const functionsAny_t bpmSetGetMonitDataMaskSamplesBegFunc = {functionsUInt32_t{"DSP", halcs_set_monit_data_mask_samples_beg,
                                                                                         halcs_get_monit_data_mask_samples_beg}};
static const functionsAny_t bpmSetGetMonitDataMaskSamplesEndFunc = {functionsUInt32_t{"DSP", halcs_set_monit_data_mask_samples_end,
                                                                                            halcs_get_monit_data_mask_samples_end}};
static const functionsAny_t bpmSetGetMonitTagDesyncCntRstFunc =    {functionsUInt32_t{"DSP", halcs_set_monit_tag_desync_cnt_rst,
                                                                                           halcs_get_monit_tag_desync_cnt_rst}};
static const functionsAny_t bpmSetGetMonitTagDesyncCntFunc =       {functionsUInt32_t{"DSP", NULL,
                                                                                            halcs_get_monit_tag_desync_cnt}};
static const functionsAny_t bpmSetGetMonitPollTimeFunc =         {functionsUInt32_t{"DSP", halcs_set_monit_poll_time, halcs_get_monit_poll_time}};
static const functionsAny_t bpmSetGetXOffsetFunc =               {functionsInt32_t{"DSP", halcs_set_offset_x, halcs_get_offset_x}};
static const functionsAny_t bpmSetGetYOffsetFunc =               {functionsInt32_t{"DSP", halcs_set_offset_y, halcs_get_offset_y}};

static const functionsAny_t bpmSetGetAdcGainFixedPointPos =            {functionsUInt32_t{"DSP", NULL, halcs_get_adc_gain_fixed_point_pos}};

static const functionsAny_t bpmSetGetAmpGainCh0SwInv =            {functionsUInt32_t{"DSP", halcs_set_adc_ch0_swclk_0_gain, halcs_get_adc_ch0_swclk_0_gain}};
static const functionsAny_t bpmSetGetAmpGainCh1SwInv =            {functionsUInt32_t{"DSP", halcs_set_adc_ch1_swclk_0_gain, halcs_get_adc_ch1_swclk_0_gain}};
static const functionsAny_t bpmSetGetAmpGainCh2SwInv =            {functionsUInt32_t{"DSP", halcs_set_adc_ch2_swclk_0_gain, halcs_get_adc_ch2_swclk_0_gain}};
static const functionsAny_t bpmSetGetAmpGainCh3SwInv =            {functionsUInt32_t{"DSP", halcs_set_adc_ch3_swclk_0_gain, halcs_get_adc_ch3_swclk_0_gain}};
static const functionsAny_t bpmSetGetAmpGainCh0SwDir =            {functionsUInt32_t{"DSP", halcs_set_adc_ch0_swclk_1_gain, halcs_get_adc_ch0_swclk_1_gain}};
static const functionsAny_t bpmSetGetAmpGainCh1SwDir =            {functionsUInt32_t{"DSP", halcs_set_adc_ch1_swclk_1_gain, halcs_get_adc_ch1_swclk_1_gain}};
static const functionsAny_t bpmSetGetAmpGainCh2SwDir =            {functionsUInt32_t{"DSP", halcs_set_adc_ch2_swclk_1_gain, halcs_get_adc_ch2_swclk_1_gain}};
static const functionsAny_t bpmSetGetAmpGainCh3SwDir =            {functionsUInt32_t{"DSP", halcs_set_adc_ch3_swclk_1_gain, halcs_get_adc_ch3_swclk_1_gain}};

static const functionsAny_t bpmSetGetAdcSwFunc =                 {functionsUInt32_t{"SWAP", halcs_set_sw, halcs_get_sw}};
static const functionsAny_t bpmSetGetAdcSwDlyFunc =              {functionsUInt32_t{"SWAP", halcs_set_sw_dly, halcs_get_sw_dly}};
static const functionsAny_t bpmSetGetAdcSwDivClkFunc =           {functionsUInt32_t{"SWAP", halcs_set_div_clk, halcs_get_div_clk}};
static const functionsAny_t bpmSetGetAdcTrigDirFunc =            {functionsUInt32_t{"FMC_ADC_COMMON", halcs_set_trig_dir, halcs_get_trig_dir}};
static const functionsAny_t bpmSetGetAdcTrigTermFunc =           {functionsUInt32_t{"FMC_ADC_COMMON", halcs_set_trig_term, halcs_get_trig_term}};
static const functionsAny_t bpmSetGetAdcRandFunc =               {functionsUInt32_t{"FMC130M_4CH", halcs_set_adc_rand, halcs_get_adc_rand}};
static const functionsAny_t bpmSetGetAdcDithFunc =               {functionsUInt32_t{"FMC130M_4CH", halcs_set_adc_dith, halcs_get_adc_dith}};
static const functionsAny_t bpmSetGetAdcShdnFunc =               {functionsUInt32_t{"FMC130M_4CH", halcs_set_adc_shdn, halcs_get_adc_shdn}};
static const functionsAny_t bpmSetGetAdcPgaFunc =                {functionsUInt32_t{"FMC130M_4CH", halcs_set_adc_pga, halcs_get_adc_pga}};
static const functionsAny_t bpmSetGetAdcTestModeFunc =           {functionsUInt32Chan_t{"FMC250M_4CH", halcs_set_test_mode_adc, halcs_dummy_read_chan_32}};
static const functionsAny_t bpmSetGetAdcRstModesFunc =           {functionsUInt32Chan_t{"FMC250M_4CH", halcs_set_rst_modes_adc, halcs_dummy_read_chan_32}};
static const functionsAny_t bpmSetGetAdcTempFunc =               {functionsUInt32Chan_t{"FMC250M_4CH", NULL, halcs_get_temp_adc}};
static const functionsAny_t bpmSetGetAdcCalStatusFunc =          {functionsUInt32Chan_t{"FMC250M_4CH", NULL, halcs_get_cal_status_adc}};
static const functionsAny_t bpmSetGetAdcTestDataFunc =           {functionsUInt32_t{"FMC_ADC_COMMON", halcs_set_adc_test_data_en, halcs_get_adc_test_data_en}};
static const functionsAny_t bpmSetGetAdcClkSelFunc =             {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_fmc_clk_sel, halcs_get_fmc_clk_sel}};
static const functionsAny_t bpmSetGetAdcAD9510DefaultsFunc =     {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_defaults, halcs_dummy_read_32}};
static const functionsAny_t bpmSetGetAdcAD9510PllFunctionFunc =  {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_fmc_pll_function, halcs_get_fmc_pll_function}};
static const functionsAny_t bpmSetGetAdcAD9510PllStatusFunc =    {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_fmc_pll_status, halcs_get_fmc_pll_status}};
static const functionsAny_t bpmSetGetAdcAD9510ClkSelFunc =       {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_pll_clk_sel, halcs_get_ad9510_pll_clk_sel}};
static const functionsAny_t bpmSetGetAdcAD9510ADivFunc =         {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_pll_a_div, halcs_get_ad9510_pll_a_div}};
static const functionsAny_t bpmSetGetAdcAD9510BDivFunc =         {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_pll_b_div, halcs_get_ad9510_pll_b_div}};
static const functionsAny_t bpmSetGetAdcAD9510PrescalerFunc =    {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_pll_prescaler, halcs_get_ad9510_pll_prescaler}};
static const functionsAny_t bpmSetGetAdcAD9510RDivFunc =         {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_r_div, halcs_get_ad9510_r_div}};
static const functionsAny_t bpmSetGetAdcAD9510PllPDownFunc =     {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_pll_pdown, halcs_get_ad9510_pll_pdown}};
static const functionsAny_t bpmSetGetAdcAD9510MuxStatusFunc =    {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_mux_status, halcs_get_ad9510_mux_status}};
static const functionsAny_t bpmSetGetAdcAD9510CPCurrentFunc =    {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_cp_current, halcs_get_ad9510_cp_current}};
static const functionsAny_t bpmSetGetAdcAD9510OutputsFunc =      {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_ad9510_outputs, halcs_get_ad9510_outputs}};
static const functionsAny_t bpmSetGetActiveClkRstADCsFunc =      {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_rst_isla216p, halcs_dummy_read_32}};
static const functionsAny_t bpmSetGetActiveClkSi571OeFunc =      {functionsUInt32_t{"FMC_ACTIVE_CLK", halcs_set_si571_oe, halcs_get_si571_oe}};
static const functionsAny_t bpmSetGetAfcSi57xOeFunc =            {functionsUInt32_t{"AFC_MGMT", halcs_set_si571_oe, halcs_get_si571_oe}};

static const functionsAny_t bpmSetGetFmcPicoRngR0Func =          {functionsUInt32_t{"FMCPICO1M_4CH", halcs_set_fmcpico_rng_r0, halcs_get_fmcpico_rng_r0}};
static const functionsAny_t bpmSetGetFmcPicoRngR1Func =          {functionsUInt32_t{"FMCPICO1M_4CH", halcs_set_fmcpico_rng_r1, halcs_get_fmcpico_rng_r1}};
static const functionsAny_t bpmSetGetFmcPicoRngR2Func =          {functionsUInt32_t{"FMCPICO1M_4CH", halcs_set_fmcpico_rng_r2, halcs_get_fmcpico_rng_r2}};
static const functionsAny_t bpmSetGetFmcPicoRngR3Func =          {functionsUInt32_t{"FMCPICO1M_4CH", halcs_set_fmcpico_rng_r3, halcs_get_fmcpico_rng_r3}};

static const functionsAny_t bpmSetGetAcqDataTrigThresFunc =      {functionsUInt32Acq_t{"ACQ", acq_set_data_trig_thres, acq_get_data_trig_thres}};
static const functionsAny_t bpmSetGetAcqDataTrigPolFunc =        {functionsUInt32Acq_t{"ACQ", acq_set_data_trig_pol, acq_get_data_trig_pol}};
static const functionsAny_t bpmSetGetAcqDataTrigSelFunc =        {functionsUInt32Acq_t{"ACQ", acq_set_data_trig_sel, acq_get_data_trig_sel}};
static const functionsAny_t bpmSetGetAcqDataTrigFiltFunc =       {functionsUInt32Acq_t{"ACQ", acq_set_data_trig_filt, acq_get_data_trig_filt}};
static const functionsAny_t bpmSetGetAcqHwDlyFunc =              {functionsUInt32Acq_t{"ACQ", acq_set_hw_trig_dly, acq_get_hw_trig_dly}};
static const functionsAny_t bpmSetGetAcqDataTrigChanFunc =       {functionsUInt32Acq_t{"ACQ", acq_set_data_trig_chan, acq_get_data_trig_chan}};

static const functionsAny_t timRcvSetGetPhaseMeasNavgFunc =      {functionsUInt32_t{"TIM_RCV", halcs_set_phase_meas_navg, halcs_get_phase_meas_navg}};
static const functionsAny_t timRcvSetGetDMTDADeglitchThresFunc = {functionsUInt32_t{"TIM_RCV", halcs_set_dmtd_a_deglitcher_thres, halcs_get_dmtd_a_deglitcher_thres}};
static const functionsAny_t timRcvSetGetDMTDBDeglitchThresFunc = {functionsUInt32_t{"TIM_RCV", halcs_set_dmtd_b_deglitcher_thres, halcs_get_dmtd_a_deglitcher_thres}};
static const functionsAny_t timRcvSetGetPhaseMeasFunc =          {functionsUInt32_t{"TIM_RCV", halcs_set_phase_meas, halcs_get_phase_meas}};
static const functionsAny_t timRcvSetGetDMTDAFreqFunc =          {functionsUInt32_t{"TIM_RCV", NULL, halcs_get_dmtd_a_freq}};
static const functionsAny_t timRcvSetGetDMTDBFreqFunc =          {functionsUInt32_t{"TIM_RCV", NULL, halcs_get_dmtd_b_freq}};

static const functionsAny_t bpmSetGetIntlkEnFunc =                {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_en, halcs_get_orbit_intlk_en}};
static const functionsAny_t bpmSetGetIntlkClrFunc =               {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_clr, halcs_get_orbit_intlk_clr }};
static const functionsAny_t bpmSetGetIntlkMinSumEnFunc =          {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_min_sum_en, halcs_get_orbit_intlk_min_sum_en}};
static const functionsAny_t bpmSetGetIntlkTransEnFunc =           {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_en, halcs_get_orbit_intlk_trans_en}};
static const functionsAny_t bpmSetGetIntlkTransClrFunc =          {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_clr, halcs_get_orbit_intlk_trans_clr}};
static const functionsAny_t bpmSetGetIntlkAngEnFunc =             {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_en, halcs_get_orbit_intlk_ang_en}};
static const functionsAny_t bpmSetGetIntlkAngClrFunc =            {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_clr, halcs_get_orbit_intlk_ang_clr}};
static const functionsAny_t bpmSetGetIntlkTransBiggerXFunc =      {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_x}};
static const functionsAny_t bpmSetGetIntlkTransBiggerYFunc =      {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_y}};
static const functionsAny_t bpmSetGetIntlkTransBiggerLtcXFunc =   {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_ltc_x}};
static const functionsAny_t bpmSetGetIntlkTransBiggerLtcYFunc =   {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_ltc_y}};
static const functionsAny_t bpmSetGetIntlkTransBiggerAnyFunc =    {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_any}};
static const functionsAny_t bpmSetGetIntlkTransBiggerFunc =       {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger}};
static const functionsAny_t bpmSetGetIntlkTransBiggerLtcFunc =    {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_bigger_ltc}};
static const functionsAny_t bpmSetGetIntlkAngBiggerXFunc =        {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_x}};
static const functionsAny_t bpmSetGetIntlkAngBiggerYFunc =        {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_y}};
static const functionsAny_t bpmSetGetIntlkAngBiggerLtcXFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_ltc_x}};
static const functionsAny_t bpmSetGetIntlkAngBiggerLtcYFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_ltc_y}};
static const functionsAny_t bpmSetGetIntlkAngBiggerAnyFunc =      {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_any}};
static const functionsAny_t bpmSetGetIntlkAngBiggerFunc =         {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger}};
static const functionsAny_t bpmSetGetIntlkAngBiggerLtcFunc =      {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_bigger_ltc}};
static const functionsAny_t bpmSetGetIntlkFunc =                  {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk}};
static const functionsAny_t bpmSetGetIntlkLtcFunc =               {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ltc}};
static const functionsAny_t bpmSetGetIntlkTransSmallerXFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_x}};
static const functionsAny_t bpmSetGetIntlkTransSmallerYFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_y}};
static const functionsAny_t bpmSetGetIntlkTransSmallerLtcXFunc =  {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_ltc_x}};
static const functionsAny_t bpmSetGetIntlkTransSmallerLtcYFunc =  {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_ltc_y}};
static const functionsAny_t bpmSetGetIntlkTransSmallerAnyFunc =   {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_any}};
static const functionsAny_t bpmSetGetIntlkTransSmallerFunc =      {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller}};
static const functionsAny_t bpmSetGetIntlkTransSmallerLtcFunc =   {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_smaller_ltc}};
static const functionsAny_t bpmSetGetIntlkAngSmallerXFunc =       {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_x}};
static const functionsAny_t bpmSetGetIntlkAngSmallerYFunc =       {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_y}};
static const functionsAny_t bpmSetGetIntlkAngSmallerLtcXFunc =    {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_ltc_x}};
static const functionsAny_t bpmSetGetIntlkAngSmallerLtcYFunc =    {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_ltc_y}};
static const functionsAny_t bpmSetGetIntlkAngSmallerAnyFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_any}};
static const functionsAny_t bpmSetGetIntlkAngSmallerFunc =        {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller}};
static const functionsAny_t bpmSetGetIntlkAngSmallerLtcFunc =     {functionsUInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_smaller_ltc}};
static const functionsAny_t bpmSetGetIntlkMinSumFunc =            {functionsUInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_min_sum, halcs_get_orbit_intlk_min_sum}};
static const functionsAny_t bpmSetGetIntlkTransMaxXFunc =         {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_max_x, halcs_get_orbit_intlk_trans_max_x}};
static const functionsAny_t bpmSetGetIntlkTransMaxYFunc =         {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_max_y, halcs_get_orbit_intlk_trans_max_y}};
static const functionsAny_t bpmSetGetIntlkAngMaxXFunc =           {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_max_x, halcs_get_orbit_intlk_ang_max_x}};
static const functionsAny_t bpmSetGetIntlkAngMaxYFunc =           {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_max_y, halcs_get_orbit_intlk_ang_max_y}};
static const functionsAny_t bpmSetGetIntlkTransMinXFunc =         {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_min_x, halcs_get_orbit_intlk_trans_min_x}};
static const functionsAny_t bpmSetGetIntlkTransMinYFunc =         {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_trans_min_y, halcs_get_orbit_intlk_trans_min_y}};
static const functionsAny_t bpmSetGetIntlkAngMinXFunc =           {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_min_x, halcs_get_orbit_intlk_ang_min_x}};
static const functionsAny_t bpmSetGetIntlkAngMinYFunc =           {functionsInt32_t{"ORBIT_INTLK", halcs_set_orbit_intlk_ang_min_y, halcs_get_orbit_intlk_ang_min_y}};
static const functionsAny_t bpmSetGetIntlkTransDiffXFunc = {functionsInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_x_diff}};
static const functionsAny_t bpmSetGetIntlkTransDiffYFunc = {functionsInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_trans_y_diff}};
static const functionsAny_t bpmSetGetIntlkAngDiffXFunc = {functionsInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_x_diff}};
static const functionsAny_t bpmSetGetIntlkAngDiffYFunc = {functionsInt32_t{"ORBIT_INTLK", NULL, halcs_get_orbit_intlk_ang_y_diff}};

/* Double funfunctionsAny_t ctions mapping */
static const functionsAny_t bpmSetGetAdcSi57xFreqFunc =          {functionsFloat64_t{"FMC_ACTIVE_CLK", halcs_set_si571_freq, halcs_get_si571_freq}};
static const functionsAny_t bpmSetGetAdcSi57xFStartupFunc =      {functionsFloat64_t{"FMC_ACTIVE_CLK", halcs_set_si571_fstartup, halcs_get_si571_fstartup}};
static const functionsAny_t bpmSetGetAfcSi57xFreqFunc =          {functionsFloat64_t{"AFC_MGMT", halcs_set_si571_freq, halcs_get_si571_freq}};
static const functionsAny_t bpmSetGetAfcSi57xFStartupFunc =      {functionsFloat64_t{"AFC_MGMT", halcs_set_si571_fstartup, halcs_get_si571_fstartup}};

/* Int32 withfunctionsAny_t  channel selection functions mapping */
static const functionsAny_t bpmSetGetTrigDirFunc =               {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_dir, halcs_get_trigger_dir}};
static const functionsAny_t bpmSetGetTrigDirPolFunc =            {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_dir_pol, halcs_get_trigger_dir_pol}};
static const functionsAny_t bpmSetGetTrigRcvCntRstFunc =         {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_rcv_count_rst, halcs_get_trigger_rcv_count_rst}};
static const functionsAny_t bpmSetGetTrigTrnCntRstFunc =         {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_transm_count_rst, halcs_get_trigger_transm_count_rst}};
static const functionsAny_t bpmSetGetTrigRcvLenFunc =            {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_rcv_len, halcs_get_trigger_rcv_len}};
static const functionsAny_t bpmSetGetTrigTrnLenFunc =            {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_transm_len, halcs_get_trigger_transm_len}};
static const functionsAny_t bpmSetGetTrigCntRcvFunc =            {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_count_rcv, halcs_get_trigger_count_rcv}};
static const functionsAny_t bpmSetGetTrigCntTrnFunc =            {functionsUInt32Chan_t{"TRIGGER_IFACE", halcs_set_trigger_count_transm, halcs_get_trigger_count_transm}};

static const functionsAny_t bpmSetGetTrigRcvSrcFunc =            {functionsUInt32Chan_t{"TRIGGER_MUX", halcs_set_trigger_rcv_src, halcs_get_trigger_rcv_src}};
static const functionsAny_t bpmSetGetTrigTrnSrcFunc =            {functionsUInt32Chan_t{"TRIGGER_MUX", halcs_set_trigger_transm_src, halcs_get_trigger_transm_src}};
static const functionsAny_t bpmSetGetTrigRcvSelFunc =            {functionsUInt32Chan_t{"TRIGGER_MUX", halcs_set_trigger_rcv_in_sel, halcs_get_trigger_rcv_in_sel}};
static const functionsAny_t bpmSetGetTrigTrnSelFunc =            {functionsUInt32Chan_t{"TRIGGER_MUX", halcs_set_trigger_transm_out_sel, halcs_get_trigger_transm_out_sel}};

static const functionsAny_t fofbCtrlSetGeErrClrFunc                   = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_err_clr,
                                                                          halcs_get_fofb_ctrl_err_clr}};
static const functionsAny_t fofbCtrlSetGetCcEnableFunc                = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_cc_enable,
                                                                          halcs_get_fofb_ctrl_cc_enable}};
static const functionsAny_t fofbCtrlSetGetTfsOverrideFunc             = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_tfs_override,
                                                                          halcs_get_fofb_ctrl_tfs_override}};
static const functionsAny_t fofbCtrlSetGetBpmIdFunc                   = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_bpm_id,
                                                                          halcs_get_fofb_ctrl_bpm_id}};
static const functionsAny_t fofbCtrlSetGetTimeFrameLenFunc            = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_time_frame_len,
                                                                          halcs_get_fofb_ctrl_time_frame_len}};
static const functionsAny_t fofbCtrlSetGetMgtPowerdownFunc            = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_mgt_powerdown,
                                                                          halcs_get_fofb_ctrl_mgt_powerdown}};
static const functionsAny_t fofbCtrlSetGetMgtLoopbackFunc             = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_mgt_loopback,
                                                                          halcs_get_fofb_ctrl_mgt_loopback}};
static const functionsAny_t fofbCtrlSetGetTimeFrameDlyFunc            = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_time_frame_dly,
                                                                          halcs_get_fofb_ctrl_time_frame_dly}};
static const functionsAny_t fofbCtrlSetGetGoldenOrbXFunc              = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_golden_orb_x,
                                                                          halcs_get_fofb_ctrl_golden_orb_x}};
static const functionsAny_t fofbCtrlSetGetGoldenOrbYFunc              = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_golden_orb_y,
                                                                          halcs_get_fofb_ctrl_golden_orb_y}};
static const functionsAny_t fofbCtrlSetGetCustFeatureFunc             = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_cust_feature,
                                                                          halcs_get_fofb_ctrl_cust_feature}};
static const functionsAny_t fofbCtrlSetGetRxPolarityFunc              = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_rx_polarity,
                                                                          halcs_get_fofb_ctrl_rx_polarity}};
static const functionsAny_t fofbCtrlSetGetPayloadselFunc              = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_payloadsel,
                                                                          halcs_get_fofb_ctrl_payloadsel}};
static const functionsAny_t fofbCtrlSetGetFofbdataselFunc             = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_fofbdatasel,
                                                                          halcs_get_fofb_ctrl_fofbdatasel}};
static const functionsAny_t fofbCtrlSetGetFirmwareVerFunc             = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_firmware_ver}};
static const functionsAny_t fofbCtrlSetGetSysStatusFunc               = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_sys_status}};
static const functionsAny_t fofbCtrlSetGetLinkPartnerFunc             = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_link_partner}};
static const functionsAny_t fofbCtrlSetGetLinkUpFunc                  = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_link_up}};
static const functionsAny_t fofbCtrlSetGetTimeFrameCountFunc          = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_time_frame_count}};
static const functionsAny_t fofbCtrlSetGetHardErrCntFunc              = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_hard_err_cnt}};
static const functionsAny_t fofbCtrlSetGetSoftErrCntFunc              = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_soft_err_cnt}};
static const functionsAny_t fofbCtrlSetGetFrameErrCntFunc             = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_frame_err_cnt}};
static const functionsAny_t fofbCtrlSetGetRxPckCntFunc                = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_rx_pck_cnt}};
static const functionsAny_t fofbCtrlSetGetTxPckCntFunc                = {functionsUInt32Chan_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_tx_pck_cnt}};
static const functionsAny_t fofbCtrlSetGetFodProcessTimeFunc          = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_fod_process_time}};
static const functionsAny_t fofbCtrlSetGetBpmCntFunc                  = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_bpm_count}};
static const functionsAny_t fofbCtrlSetGetToaRdEnFunc                 = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_toa_rd_en,
                                                                          halcs_get_fofb_ctrl_toa_rd_en}};
static const functionsAny_t fofbCtrlSetGetToaRdStrFunc                = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_toa_rd_str,
                                                                          halcs_get_fofb_ctrl_toa_rd_str}};
static const functionsAny_t fofbCtrlSetGetToaDataFunc                 = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_toa_data}};
static const functionsAny_t fofbCtrlSetGetRcbRdEnFunc                 = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_rcb_rd_en,
                                                                          halcs_get_fofb_ctrl_rcb_rd_en}};
static const functionsAny_t fofbCtrlSetGetRcbRdStrFunc                = {functionsUInt32_t{"FOFB_CTRL", halcs_set_fofb_ctrl_rcb_rd_str,
                                                                          halcs_get_fofb_ctrl_rcb_rd_str}};
static const functionsAny_t fofbCtrlSetGetRcbDataFunc                 = {functionsUInt32_t{"FOFB_CTRL", NULL,
                                                                          halcs_get_fofb_ctrl_rcb_data}};

static const char *driverName="drvBPM";
static taskParams_t taskParams[NUM_ACQ_CORES_PER_BPM] = {
    /* Regular Core */
    {
        NULL,                          // drvBPMp
        BPMIDReg,                      // coreID
        BPM_POLL_TIME,                 // pollTime
        false                          // autoStart
    },
    /* Post-Mortem Core */
    {
        NULL,                          // drvBPMp
        BPMIDPM,                       // coreID
        BPM_PM_POLL_TIME,              // pollTime
        true                           // autoStart
    },
};
static taskParams_t taskSPParams[NUM_ACQ_CORES_PER_BPM] = {
    /* Regular Core */
    {
        NULL,                          // drvBPMp
        BPMIDReg,                      // coreID
        BPM_SP_POLL_TIME,              // pollTime
        false                          // autoStart
    },
#if 0
    /* Post-Mortem Core */
    {
        NULL,                          // drvBPMp
        BPMIDPM,                       // coreID
        BPM_PM_POLL_TIME,              // pollTime
        false                          // autoStart
    },
#endif
};
static taskParams_t taskMonitParams = {
    NULL,                              // drvBPMp
    BPMIDReg,                          // coreID
    BPM_POLL_TIME,                     // pollTime
    false                              // autoStart
};
void acqTask(void *drvPvt);
void acqSPTask(void *drvPvt);
void acqMonitTask(void *drvPvt);

static uint32_t float2fixed(double v, unsigned point_pos)
{
    return v * ((uint64_t)1 << point_pos);
}
static double fixed2float(uint32_t v, unsigned point_pos)
{
    return (double)(int32_t)v / ((uint64_t)1 << point_pos);
}

static void exitHandlerC(void *pPvt)
{
    drvBPM *pdrvBPM = (drvBPM *)pPvt;
    pdrvBPM->~drvBPM();
}

asynStatus drvBPM::getServiceChan (int bpmNumber, int addr, const char *serviceName,
        epicsUInt32 *chanArg) const
{
    static const char *functionName = "getServiceChan";
    asynStatus status = asynSuccess;
    epicsUInt32 chan = 0;

    /* Static mapping. FIXME? */
    if (streq(serviceName, "TRIGGER_MUX") || streq(serviceName, "TRIGGER_IFACE")) {
        chan = addr % MAX_TRIGGERS;
    }
    else if (streq(serviceName, "FOFB_CTRL")) {
        chan = addr % NUM_FOFB_CC_CHANNELS_PER_FOFB_CC;
    }
    else {
        chan = addr;
    }

    *chanArg = chan;
    return status;
}

asynStatus drvBPM::getServiceID (int bpmNumber, int addr, const char *serviceName,
        int *serviceIDArg) const
{
    static const char *functionName = "getServiceID";
    asynStatus status = asynSuccess;
    int serviceID = 0;
    int addrMod = 0;

    /* Static mapping. FIXME? */
    /* For these services there's only a single core per FPGA, so serviceID is always 0.
     * INIT service is always 0 per HALCS instance */
    if (streq(serviceName, "TRIGGER_IFACE") || streq(serviceName, "ORBIT_INTLK") ||
            streq(serviceName, "INIT")) {
        *serviceIDArg = 0;
        return status;
    }
    if (streq(serviceName, "ACQ")) {
        addrMod = addr;
    }
    else if (streq(serviceName, "TRIGGER_MUX")) {
        addrMod = addr/MAX_TRIGGERS;
    }
    else if (streq(serviceName, "FOFB_CTRL")) {
        *serviceIDArg = addr / NUM_FOFB_CC_CHANNELS_PER_FOFB_CC;
        return asynSuccess;
    }
    else {
        addrMod = 0;
    }

    switch (addrMod) {
        case BPMIDReg:
            serviceID = boardMap[bpmNumber].bpm;
            break;

        case BPMIDPM:
            serviceID = boardMap[bpmNumber].core_id;
            break;

        default:
            status = asynError;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error getting serviceID for addr = %d for service = %s\n",
                driverName, functionName, addr, serviceName);
            goto err_exit;
    }
    *serviceIDArg = serviceID;

err_exit:
    return status;
}

asynStatus drvBPM::getFullServiceName (int bpmNumber, int addr, const char *serviceName,
        char *fullServiceName, int fullServiceNameSize) const
{
    static const char *functionName = "getFullServiceName";
    int coreID = 0;
    int errs = 0;
    asynStatus status = asynSuccess;

    status = getServiceID (bpmNumber, addr, serviceName, &coreID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getServiceID, status=%d\n",
            driverName, functionName, status);
        goto get_service_id_err;
    }

    errs = snprintf(fullServiceName, fullServiceNameSize, "HALCS%d:DEVIO:%s%d",
            boardMap[bpmNumber].board, serviceName, coreID);
    if (errs < 0 || errs >= fullServiceNameSize){
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error generating fullServiceName, errs=%d\n",
            driverName, functionName, errs);
        status = asynError;
        goto gen_full_service_name;
    }

gen_full_service_name:
get_service_id_err:
    return status;
}

/** Constructor for the drvBPM class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] endpoint The device address string ]
 * */
drvBPM::drvBPM(const char *portName, const char *endpoint, int bpmNumber,
        const char *type, int verbose, int timeout, int maxPoints,
        int maxBuffers, size_t maxMemory)
   : asynNDArrayDriver(portName,
                    MAX_ADDR, /* maxAddr */
                    maxBuffers, maxMemory, /* maxBuffers, maxMemory */
                    asynUInt32DigitalMask | asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynGenericPointerMask | asynDrvUserMask,    /* Interface mask     */
                    asynUInt32DigitalMask | asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynGenericPointerMask,                      /* Interrupt mask     */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver blocks it is multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    asynStatus status;
    const char *functionName = "drvBPM";

    srand(time(NULL));

    /* Create portName so we can create a new AsynUser later */
    bpmPortName = epicsStrDup(portName);
    bpmType = epicsStrDup(type);
    bpmMaxPoints = maxPoints;

    this->endpoint = strdup(endpoint);
    if (this->endpoint == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvBPM failure to copy endpoint\n",
                driverName, functionName);
        status = asynError;
        goto endpoint_dup_err;
    }

    if (bpmNumber < BPM_NUMBER_MIN || bpmNumber > BPM_NUMBER_MAX) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvBPM invalid bpmNumber\n",
                driverName, functionName);
        status = asynError;
        goto invalid_bpm_number_err;
    }

    this->bpmNumber = bpmNumber;
    this->verbose = verbose;
    this->timeout = timeout;
    for (int i = 0; i < NUM_BPM_MODES; ++i) {
        for (int j = 0; j < NUM_ACQ_CORES_PER_BPM; ++j) {
            this->readingActive[i][j] = 0;
            this->repetitiveTrigger[i][j] = 0;
        }
    }

    for (int i = 0; i < NUM_BPM_MODES; ++i) {

        this->reconfSPassAcqEventId[i] = epicsEventCreate(epicsEventEmpty);
        if (!this->reconfSPassAcqEventId[i]) {
            printf("%s:%s: epicsEventCreate[%d] failure for SP reconfig. event\n",
                    driverName, functionName, i);
            return;
        }

        for (int j = 0; j < NUM_ACQ_CORES_PER_BPM; ++j) {
            /* Create events for signalling acquisition thread */
            this->startAcqEventId[i][j] = epicsEventCreate(epicsEventEmpty);
            if (!this->startAcqEventId[i][j]) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s epicsEventCreate[%d] failure for start event\n",
                        driverName, functionName, i);
                return;
            }

            this->stopAcqEventId[i][j] = epicsEventCreate(epicsEventEmpty);
            if (!this->stopAcqEventId[i][j]) {
                printf("%s:%s: epicsEventCreate[%d] failure for stop event\n",
                        driverName, functionName, i);
                return;
            }

            this->abortAcqEventId[i][j] = epicsEventCreate(epicsEventEmpty);
            if (!this->abortAcqEventId[i][j]) {
                printf("%s:%s: epicsEventCreate[%d] failure for abort event\n",
                        driverName, functionName, i);
                return;
            }

            this->activeAcqEventId[i][j] = epicsEventCreate(epicsEventEmpty);
            if (!this->activeAcqEventId[i][j]) {
                printf("%s:%s: epicsEventCreate[%d] failure for active event\n",
                        driverName, functionName, i);
                return;
            }
        }
    }

    this->activeMonitEnableEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->activeMonitEnableEventId) {
        printf("%s:%s: epicsEventCreate failure for activeMonitEnableEventId\n",
                driverName, functionName);
        return;
    }

    /* Create parameters for all addresses without specifying the ones that don't
     * make sense to be on a specified list. Without this we woudl have to create
     * different parameterIndex structures to store each index, as they could be
     * differently if created in just a few lists */

    /* CAUTION. The order of craetion must be the same as defined in .h file.
     * Otherwise, checking for FIRST_PARAM/LAST_PARAM won't work */

    /* Create BPM Status parameters */
    createParam(P_BPMModeString,    asynParamInt32,                 &P_BPMMode);
    createParam(P_BPMStatusString,  asynParamInt32,                 &P_BPMStatus);

    /* Create general parameters */
    createParam(P_HarmonicNumberString,
                                    asynParamUInt32Digital,         &P_HarmonicNumber);
    createParam(P_AdcRateString,    asynParamUInt32Digital,         &P_AdcRate);
    createParam(P_TbtRateString,    asynParamUInt32Digital,         &P_TbtRate);
    createParam(P_FofbRateString,   asynParamUInt32Digital,         &P_FofbRate);
    createParam(P_MonitRateString,  asynParamUInt32Digital,         &P_MonitRate);
    createParam(P_Monit1RateString, asynParamUInt32Digital,         &P_Monit1Rate);

    createParam(P_SwModeString,     asynParamUInt32Digital,         &P_SwMode);
    createParam(P_SwDlyString,      asynParamUInt32Digital,         &P_SwDly);
    createParam(P_SwDivClkString,   asynParamUInt32Digital,         &P_SwDivClk);

    createParam(P_ClkFreqString,    asynParamFloat64,               &P_ClkFreq);

    /* Create ADC/AD9510/FMCPICO/DSP parameters */
    createParam(P_AdcTrigDirString, asynParamUInt32Digital,         &P_AdcTrigDir);
    createParam(P_AdcTrigTermString,
                                    asynParamUInt32Digital,         &P_AdcTrigTerm);
    createParam(P_AdcRandString,    asynParamUInt32Digital,         &P_AdcRand);
    createParam(P_AdcDithString,    asynParamUInt32Digital,         &P_AdcDith);
    createParam(P_AdcShdnString,    asynParamUInt32Digital,         &P_AdcShdn);
    createParam(P_AdcPgaString,     asynParamUInt32Digital,         &P_AdcPga);
    createParam(P_AdcTestDataString,
                                    asynParamUInt32Digital,         &P_AdcTestData);
    createParam(P_AdcClkSelString,  asynParamUInt32Digital,         &P_AdcClkSel);
    createParam(P_AdcSi57xFreqString,
                                    asynParamFloat64,               &P_AdcSi57xFreq);
    createParam(P_AdcSi57xFStartupString,
                                    asynParamFloat64,               &P_AdcSi57xFStartup);
    createParam(P_AdcAD9510DfltString,
                                    asynParamUInt32Digital,         &P_AdcAD9510Dflt);
    createParam(P_AdcAD9510PllFuncString,
                                    asynParamUInt32Digital,         &P_AdcAD9510PllFunc);
    createParam(P_AdcAD9510PllStatusString,
                                    asynParamUInt32Digital,         &P_AdcAD9510PllStatus);
    createParam(P_AdcAD9510ClkSelString,
                                    asynParamUInt32Digital,         &P_AdcAD9510ClkSel);
    createParam(P_AdcAD9510ADivString,
                                    asynParamUInt32Digital,         &P_AdcAD9510ADiv);
    createParam(P_AdcAD9510BDivString,
                                    asynParamUInt32Digital,         &P_AdcAD9510BDiv);
    createParam(P_AdcAD9510PrescalerString,
                                    asynParamUInt32Digital,         &P_AdcAD9510Prescaler);
    createParam(P_AdcAD9510RDivString,
                                    asynParamUInt32Digital,         &P_AdcAD9510RDiv);
    createParam(P_AdcAD9510PllPDownString,
                                    asynParamUInt32Digital,         &P_AdcAD9510PllPDown);
    createParam(P_AdcAD9510MuxStatusString,
                                    asynParamUInt32Digital,         &P_AdcAD9510MuxStatus);
    createParam(P_AdcAD9510CpCurrentString,
                                    asynParamUInt32Digital,         &P_AdcAD9510CpCurrent);
    createParam(P_AdcAD9510OutputsString,
                                    asynParamUInt32Digital,         &P_AdcAD9510Outputs);
    createParam(P_ActiveClkRstADCsString,
                                    asynParamUInt32Digital,         &P_ActiveClkRstADCs);
    createParam(P_ActiveClkSi571OeString,
                                    asynParamUInt32Digital,         &P_ActiveClkSi571Oe);
    createParam(P_AfcSi57xOeString,
                                    asynParamUInt32Digital,         &P_AfcSi57xOe);
    createParam(P_AfcSi57xFreqString,
                                    asynParamFloat64,               &P_AfcSi57xFreq);
    createParam(P_AfcSi57xFStartupString,
                                    asynParamFloat64,               &P_AfcSi57xFStartup);
    createParam(P_FmcPicoRngR0String,
                                    asynParamUInt32Digital,         &P_FmcPicoRngR0);
    createParam(P_FmcPicoRngR1String,
                                    asynParamUInt32Digital,         &P_FmcPicoRngR1);
    createParam(P_FmcPicoRngR2String,
                                    asynParamUInt32Digital,         &P_FmcPicoRngR2);
    createParam(P_FmcPicoRngR3String,
                                    asynParamUInt32Digital,         &P_FmcPicoRngR3);
    createParam(P_KxString,         asynParamUInt32Digital,         &P_Kx);
    createParam(P_KyString,         asynParamUInt32Digital,         &P_Ky);
    createParam(P_KsumString,       asynParamUInt32Digital,         &P_Ksum);
    createParam(P_SwTagEnString,
                                    asynParamUInt32Digital,         &P_SwTagEn);
    createParam(P_SwDataMaskEnString,
                                    asynParamUInt32Digital,         &P_SwDataMaskEn);
    createParam(P_SwDataMaskSamplesString,
                                    asynParamUInt32Digital,         &P_SwDataMaskSamples);
    createParam(P_TbtTagEnString,
                                    asynParamUInt32Digital,         &P_TbtTagEn);
    createParam(P_TbtTagDlyString,
                                    asynParamUInt32Digital,         &P_TbtTagDly);
    createParam(P_TbtDataMaskEnString,
                                    asynParamUInt32Digital,         &P_TbtDataMaskEn);
    createParam(P_TbtDataMaskSamplesBegString,
                                    asynParamUInt32Digital,         &P_TbtDataMaskSamplesBeg);
    createParam(P_TbtDataMaskSamplesEndString,
                                    asynParamUInt32Digital,         &P_TbtDataMaskSamplesEnd);
    createParam(P_SwTagDesyncCntRstString,
                                    asynParamUInt32Digital,         &P_SwTagDesyncCntRst);
    createParam(P_SwTagDesyncCntString,
                                    asynParamUInt32Digital,         &P_SwTagDesyncCnt);
    createParam(P_TbtTagDesyncCntRstString,
                                    asynParamUInt32Digital,         &P_TbtTagDesyncCntRst);
    createParam(P_TbtTagDesyncCntString,
                                    asynParamUInt32Digital,         &P_TbtTagDesyncCnt);
    createParam(P_Monit1TagEnString,
                                    asynParamUInt32Digital,         &P_Monit1TagEn);
    createParam(P_Monit1TagDlyString,
                                    asynParamUInt32Digital,         &P_Monit1TagDly);
    createParam(P_Monit1DataMaskEnString,
                                    asynParamUInt32Digital,         &P_Monit1DataMaskEn);
    createParam(P_Monit1DataMaskSamplesBegString,
                                    asynParamUInt32Digital,         &P_Monit1DataMaskSamplesBeg);
    createParam(P_Monit1DataMaskSamplesEndString,
                                    asynParamUInt32Digital,         &P_Monit1DataMaskSamplesEnd);
    createParam(P_Monit1TagDesyncCntRstString,
                                    asynParamUInt32Digital,         &P_Monit1TagDesyncCntRst);
    createParam(P_Monit1TagDesyncCntString,
                                    asynParamUInt32Digital,         &P_Monit1TagDesyncCnt);
    createParam(P_MonitTagEnString,
                                    asynParamUInt32Digital,         &P_MonitTagEn);
    createParam(P_MonitTagDlyString,
                                    asynParamUInt32Digital,         &P_MonitTagDly);
    createParam(P_MonitDataMaskEnString,
                                    asynParamUInt32Digital,         &P_MonitDataMaskEn);
    createParam(P_MonitDataMaskSamplesBegString,
                                    asynParamUInt32Digital,         &P_MonitDataMaskSamplesBeg);
    createParam(P_MonitDataMaskSamplesEndString,
                                    asynParamUInt32Digital,         &P_MonitDataMaskSamplesEnd);
    createParam(P_MonitTagDesyncCntRstString,
                                    asynParamUInt32Digital,         &P_MonitTagDesyncCntRst);
    createParam(P_MonitTagDesyncCntString,
                                    asynParamUInt32Digital,         &P_MonitTagDesyncCnt);
    createParam(P_KqString,         asynParamUInt32Digital,         &P_Kq);
    createParam(P_XOffsetString,    asynParamInt32,                 &P_XOffset);
    createParam(P_YOffsetString,    asynParamInt32,                 &P_YOffset);
    createParam(P_QOffsetString,    asynParamInt32,                 &P_QOffset);
    
    createParam("DSP_ADC_GAIN_FP_POS", asynParamInt32,             &P_AdcGainFixedPointPos);
    createParam("DSP_ADC_CH0_SWDIR", asynParamFloat64,        &P_AdcCh0SwDir);
    createParam("DSP_ADC_CH1_SWDIR", asynParamFloat64,        &P_AdcCh1SwDir);
    createParam("DSP_ADC_CH2_SWDIR", asynParamFloat64,        &P_AdcCh2SwDir);
    createParam("DSP_ADC_CH3_SWDIR", asynParamFloat64,        &P_AdcCh3SwDir);
    createParam("DSP_ADC_CH0_SWINV", asynParamFloat64,        &P_AdcCh0SwInv);
    createParam("DSP_ADC_CH1_SWINV", asynParamFloat64,        &P_AdcCh1SwInv);
    createParam("DSP_ADC_CH2_SWINV", asynParamFloat64,        &P_AdcCh2SwInv);
    createParam("DSP_ADC_CH3_SWINV", asynParamFloat64,        &P_AdcCh3SwInv);
    
    /* Timing parameters */
    createParam(P_TimRcvPhaseMeasNavgString,
                                    asynParamUInt32Digital,         &P_TimRcvPhaseMeasNavg);
    createParam(P_TimRcvDMTDADeglitchThresString,
                                    asynParamUInt32Digital,         &P_TimRcvDMTDADeglitchThres);
    createParam(P_TimRcvDMTDBDeglitchThresString,
                                    asynParamUInt32Digital,         &P_TimRcvDMTDBDeglitchThres);
    createParam(P_TimRcvPhaseMeasString,
                                    asynParamUInt32Digital,         &P_TimRcvPhaseMeas);
    createParam(P_TimRcvDMTDAFreqString,
                                    asynParamUInt32Digital,         &P_TimRcvDMTDAFreq);
    createParam(P_TimRcvDMTDBFreqString,
                                    asynParamUInt32Digital,         &P_TimRcvDMTDBFreq);

    /* Create acquistion parameters */
    createParam(P_SamplesPreString, asynParamUInt32Digital,        &P_SamplesPre);
    createParam(P_SamplesPostString,
                                    asynParamUInt32Digital,        &P_SamplesPost);
    createParam(P_NumShotsString,   asynParamUInt32Digital,        &P_NumShots);
    createParam(P_ChannelString,    asynParamInt32,                &P_Channel);
    createParam(P_UpdateTimeString, asynParamFloat64,              &P_UpdateTime);
    createParam(P_TriggerString,    asynParamUInt32Digital,        &P_Trigger);
    createParam(P_TriggerEventString,
                                    asynParamUInt32Digital,        &P_TriggerEvent);
    createParam(P_TriggerRepString,
                                    asynParamUInt32Digital,        &P_TriggerRep);
    createParam(P_TriggerDataThresString,
                                    asynParamUInt32Digital,        &P_TriggerDataThres);
    createParam(P_TriggerDataPolString,
                                    asynParamUInt32Digital,        &P_TriggerDataPol);
    createParam(P_TriggerDataSelString,
                                    asynParamUInt32Digital,        &P_TriggerDataSel);
    createParam(P_TriggerDataFiltString,
                                    asynParamUInt32Digital,        &P_TriggerDataFilt);
    createParam(P_TriggerHwDlyString,
                                    asynParamUInt32Digital,        &P_TriggerHwDly);
    createParam(P_DataTrigChanString,
                                    asynParamUInt32Digital,        &P_DataTrigChan);
    createParam(P_ChannelSampleSizeString,
                                    asynParamUInt32Digital,        &P_ChannelSampleSize);
    createParam(P_ChannelNumAtomsString,
                                    asynParamUInt32Digital,        &P_ChannelNumAtoms);
    createParam(P_ChannelAtomWidthString,
                                    asynParamUInt32Digital,        &P_ChannelAtomWidth);

    /* Create MONIT/SP parameters */
    createParam(P_MonitPollTimeString,
                                    asynParamUInt32Digital,        &P_MonitPollTime);
    createParam(P_MonitEnableString,
                                    asynParamInt32,                 &P_MonitEnable);

    createParam(P_SPAmpAString,     asynParamFloat64,               &P_SPAmpA);
    createParam(P_SPAmpBString,     asynParamFloat64,               &P_SPAmpB);
    createParam(P_SPAmpCString,     asynParamFloat64,               &P_SPAmpC);
    createParam(P_SPAmpDString,     asynParamFloat64,               &P_SPAmpD);
    createParam(P_SPPosXString,     asynParamFloat64,               &P_SPPosX);
    createParam(P_SPPosYString,     asynParamFloat64,               &P_SPPosY);
    createParam(P_SPPosQString,     asynParamFloat64,               &P_SPPosQ);
    createParam(P_SPPosSumString,   asynParamFloat64,               &P_SPPosSum);

    /* Create ADC parameters  */
    createParam(P_AdcTestModeString,
                                    asynParamUInt32Digital,         &P_AdcTestMode);
    createParam(P_AdcRstModesString,
                                    asynParamUInt32Digital,         &P_AdcRstModes);
    createParam(P_AdcCalStatusString,
                                    asynParamUInt32Digital,         &P_AdcCalStatus);

    /* FIXME. This must come after P_AdcCalStatus, as we don't want to update this value
     * everytime a new write on ADC parameter occurs. ADC temperature is already
     * SCAN = 1 second  on Database level */
    createParam(P_AdcTempString,    asynParamUInt32Digital,         &P_AdcTemp);
    createParam(P_AdcRegReadString, asynParamUInt32Digital,         &P_AdcRegRead);
    createParam(P_AdcRegReadDataString,
                                    asynParamUInt32Digital,         &P_AdcRegReadData);
    createParam(P_AdcRegReadAddrString,
                                    asynParamUInt32Digital,         &P_AdcRegReadAddr);
    createParam(P_AdcRegWriteString,
                                    asynParamUInt32Digital,         &P_AdcRegWrite);
    createParam(P_AdcRegWriteDataString,
                                    asynParamUInt32Digital,         &P_AdcRegWriteData);
    createParam(P_AdcRegWriteAddrString,
                                    asynParamUInt32Digital,         &P_AdcRegWriteAddr);

    /* Create Trigger parameters */
    createParam(P_TriggerChanString,      asynParamInt32,           &P_TriggerChan);
    createParam(P_TriggerDirString,       asynParamUInt32Digital,   &P_TriggerDir);
    createParam(P_TriggerDirPolString,    asynParamUInt32Digital,   &P_TriggerDirPol);
    createParam(P_TriggerRcvCntRstString, asynParamUInt32Digital,   &P_TriggerRcvCntRst);
    createParam(P_TriggerTrnCntRstString, asynParamUInt32Digital,   &P_TriggerTrnCntRst);
    createParam(P_TriggerRcvLenString,    asynParamUInt32Digital,   &P_TriggerRcvLen);
    createParam(P_TriggerTrnLenString,    asynParamUInt32Digital,   &P_TriggerTrnLen);
    createParam(P_TriggerCntRcvString,    asynParamUInt32Digital,   &P_TriggerCntRcv);
    createParam(P_TriggerCntTrnString,    asynParamUInt32Digital,   &P_TriggerCntTrn);
    createParam(P_TriggerRcvSrcString,    asynParamUInt32Digital,   &P_TriggerRcvSrc);
    createParam(P_TriggerTrnSrcString,    asynParamUInt32Digital,   &P_TriggerTrnSrc);
    createParam(P_TriggerRcvInSelString,  asynParamUInt32Digital,   &P_TriggerRcvInSel);
    createParam(P_TriggerTrnOutSelString, asynParamUInt32Digital,   &P_TriggerTrnOutSel);

    /* Create Interlock parameters */
    createParam(P_IntlkEnString,               asynParamUInt32Digital, &P_IntlkEn);
    createParam(P_IntlkClrString,              asynParamUInt32Digital, &P_IntlkClr);
    createParam(P_IntlkMinSumEnString,         asynParamUInt32Digital, &P_IntlkMinSumEn);
    createParam(P_IntlkTransEnString,          asynParamUInt32Digital, &P_IntlkTransEn);
    createParam(P_IntlkTransClrString,         asynParamUInt32Digital, &P_IntlkTransClr);
    createParam(P_IntlkAngEnString,            asynParamUInt32Digital, &P_IntlkAngEn);
    createParam(P_IntlkAngClrString,           asynParamUInt32Digital, &P_IntlkAngClr);
    createParam(P_IntlkTransBiggerXString,     asynParamUInt32Digital, &P_IntlkTransBiggerX);
    createParam(P_IntlkTransBiggerYString,     asynParamUInt32Digital, &P_IntlkTransBiggerY);
    createParam(P_IntlkTransBiggerLtcXString,  asynParamUInt32Digital, &P_IntlkTransBiggerLtcX);
    createParam(P_IntlkTransBiggerLtcYString,  asynParamUInt32Digital, &P_IntlkTransBiggerLtcY);
    createParam(P_IntlkTransBiggerAnyString,   asynParamUInt32Digital, &P_IntlkTransBiggerAny);
    createParam(P_IntlkTransBiggerString,      asynParamUInt32Digital, &P_IntlkTransBigger);
    createParam(P_IntlkTransBiggerLtcString,   asynParamUInt32Digital, &P_IntlkTransBiggerLtc);
    createParam(P_IntlkAngBiggerXString,       asynParamUInt32Digital, &P_IntlkAngBiggerX);
    createParam(P_IntlkAngBiggerYString,       asynParamUInt32Digital, &P_IntlkAngBiggerY);
    createParam(P_IntlkAngBiggerLtcXString,    asynParamUInt32Digital, &P_IntlkAngBiggerLtcX);
    createParam(P_IntlkAngBiggerLtcYString,    asynParamUInt32Digital, &P_IntlkAngBiggerLtcY);
    createParam(P_IntlkAngBiggerAnyString,     asynParamUInt32Digital, &P_IntlkAngBiggerAny);
    createParam(P_IntlkAngBiggerString,        asynParamUInt32Digital, &P_IntlkAngBigger);
    createParam(P_IntlkAngBiggerLtcString,     asynParamUInt32Digital, &P_IntlkAngBiggerLtc);
    createParam(P_IntlkString,                 asynParamUInt32Digital, &P_Intlk);
    createParam(P_IntlkLtcString,              asynParamUInt32Digital, &P_IntlkLtc);
    createParam(P_IntlkTransSmallerXString,    asynParamUInt32Digital, &P_IntlkTransSmallerX);
    createParam(P_IntlkTransSmallerYString,    asynParamUInt32Digital, &P_IntlkTransSmallerY);
    createParam(P_IntlkTransSmallerLtcXString, asynParamUInt32Digital, &P_IntlkTransSmallerLtcX);
    createParam(P_IntlkTransSmallerLtcYString, asynParamUInt32Digital, &P_IntlkTransSmallerLtcY);
    createParam(P_IntlkTransSmallerAnyString,  asynParamUInt32Digital, &P_IntlkTransSmallerAny);
    createParam(P_IntlkTransSmallerString,     asynParamUInt32Digital, &P_IntlkTransSmaller);
    createParam(P_IntlkTransSmallerLtcString,  asynParamUInt32Digital, &P_IntlkTransSmallerLtc);
    createParam(P_IntlkAngSmallerXString,      asynParamUInt32Digital, &P_IntlkAngSmallerX);
    createParam(P_IntlkAngSmallerYString,      asynParamUInt32Digital, &P_IntlkAngSmallerY);
    createParam(P_IntlkAngSmallerLtcXString,   asynParamUInt32Digital, &P_IntlkAngSmallerLtcX);
    createParam(P_IntlkAngSmallerLtcYString,   asynParamUInt32Digital, &P_IntlkAngSmallerLtcY);
    createParam(P_IntlkAngSmallerAnyString,    asynParamUInt32Digital, &P_IntlkAngSmallerAny);
    createParam(P_IntlkAngSmallerString,       asynParamUInt32Digital, &P_IntlkAngSmaller);
    createParam(P_IntlkAngSmallerLtcString,    asynParamUInt32Digital, &P_IntlkAngSmallerLtc);
    createParam(P_IntlkMinSumString,           asynParamInt32,         &P_IntlkMinSum);
    createParam(P_IntlkTransMaxXString,        asynParamInt32,         &P_IntlkTransMaxX);
    createParam(P_IntlkTransMaxYString,        asynParamInt32,         &P_IntlkTransMaxY);
    createParam(P_IntlkAngMaxXString,          asynParamInt32,         &P_IntlkAngMaxX);
    createParam(P_IntlkAngMaxYString,          asynParamInt32,         &P_IntlkAngMaxY);
    createParam(P_IntlkTransMinXString,        asynParamInt32,         &P_IntlkTransMinX);
    createParam(P_IntlkTransMinYString,        asynParamInt32,         &P_IntlkTransMinY);
    createParam(P_IntlkAngMinXString,          asynParamInt32,         &P_IntlkAngMinX);
    createParam(P_IntlkAngMinYString,          asynParamInt32,         &P_IntlkAngMinY);
    createParam("TRANS_X_DIFF", asynParamInt32, &P_IntlkTransDiffX);
    createParam("TRANS_Y_DIFF", asynParamInt32, &P_IntlkTransDiffY);
    createParam("ANG_X_DIFF", asynParamInt32, &P_IntlkAngDiffX);
    createParam("ANG_Y_DIFF", asynParamInt32, &P_IntlkAngDiffY);

    /* Create fofb_ctrl parameters */
    createParam(P_FofbCtrlErrClrString,              asynParamUInt32Digital,        &P_FofbCtrlErrClr);
    createParam(P_FofbCtrlCcEnableString,            asynParamUInt32Digital,        &P_FofbCtrlCcEnable);
    createParam(P_FofbCtrlTfsOverrideString,         asynParamUInt32Digital,        &P_FofbCtrlTfsOverride);
    createParam(P_FofbCtrlBpmIdString,               asynParamUInt32Digital,        &P_FofbCtrlBpmId);
    createParam(P_FofbCtrlTimeFrameLenString,        asynParamUInt32Digital,        &P_FofbCtrlTimeFrameLen);
    createParam(P_FofbCtrlMgtPowerdownString,        asynParamUInt32Digital,        &P_FofbCtrlMgtPowerdown);
    createParam(P_FofbCtrlMgtLoopbackString,         asynParamUInt32Digital,        &P_FofbCtrlMgtLoopback);
    createParam(P_FofbCtrlTimeFrameDlyString,        asynParamUInt32Digital,        &P_FofbCtrlTimeFrameDly);
    createParam(P_FofbCtrlGoldenOrbXString,          asynParamUInt32Digital,        &P_FofbCtrlGoldenOrbX);
    createParam(P_FofbCtrlGoldenOrbYString,          asynParamUInt32Digital,        &P_FofbCtrlGoldenOrbY);
    createParam(P_FofbCtrlCustFeatureString,         asynParamUInt32Digital,        &P_FofbCtrlCustFeature);
    createParam(P_FofbCtrlRxPolarityString,          asynParamUInt32Digital,        &P_FofbCtrlRxPolarity);
    createParam(P_FofbCtrlPayloadselString,          asynParamUInt32Digital,        &P_FofbCtrlPayloadsel);
    createParam(P_FofbCtrlFofbdataselString,         asynParamUInt32Digital,        &P_FofbCtrlFofbdatasel);
    createParam(P_FofbCtrlFirmwareVerString,         asynParamUInt32Digital,        &P_FofbCtrlFirmwareVer);
    createParam(P_FofbCtrlSysStatusString,           asynParamUInt32Digital,        &P_FofbCtrlSysStatus);
    createParam(P_FofbCtrlLinkPartnerString,         asynParamUInt32Digital,        &P_FofbCtrlLinkPartner);
    createParam(P_FofbCtrlLinkUpString,              asynParamUInt32Digital,        &P_FofbCtrlLinkUp);
    createParam(P_FofbCtrlTimeFrameCountString,      asynParamUInt32Digital,        &P_FofbCtrlTimeFrameCount);
    createParam(P_FofbCtrlHardErrCntString,          asynParamUInt32Digital,        &P_FofbCtrlHardErrCnt);
    createParam(P_FofbCtrlSoftErrCntString,          asynParamUInt32Digital,        &P_FofbCtrlSoftErrCnt);
    createParam(P_FofbCtrlFrameErrCntString,         asynParamUInt32Digital,        &P_FofbCtrlFrameErrCnt);
    createParam(P_FofbCtrlRxPckCntString,            asynParamUInt32Digital,        &P_FofbCtrlRxPckCnt);
    createParam(P_FofbCtrlTxPckCntString,            asynParamUInt32Digital,        &P_FofbCtrlTxPckCnt);
    createParam(P_FofbCtrlFodProcessTimeString,      asynParamUInt32Digital,        &P_FofbCtrlFodProcessTime);
    createParam(P_FofbCtrlBpmCntString,              asynParamUInt32Digital,        &P_FofbCtrlBpmCnt);
    createParam(P_FofbCtrlToaRdEnString,             asynParamUInt32Digital,        &P_FofbCtrlToaRdEn);
    createParam(P_FofbCtrlToaRdStrString,            asynParamUInt32Digital,        &P_FofbCtrlToaRdStr);
    createParam(P_FofbCtrlToaDataString,             asynParamUInt32Digital,        &P_FofbCtrlToaData);
    createParam(P_FofbCtrlRcbRdEnString,             asynParamUInt32Digital,        &P_FofbCtrlRcbRdEn);
    createParam(P_FofbCtrlRcbRdStrString,            asynParamUInt32Digital,        &P_FofbCtrlRcbRdStr);
    createParam(P_FofbCtrlRcbDataString,             asynParamUInt32Digital,        &P_FofbCtrlRcbData);

    /* BPM HW Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHwFunc.emplace(P_Kx, bpmSetGetKxFunc);
    bpmHwFunc.emplace(P_Ky, bpmSetGetKyFunc);
    bpmHwFunc.emplace(P_Ksum, bpmSetGetKsumFunc);
    bpmHwFunc.emplace(P_SwTagEn, bpmSetGetSwTagEnFunc);
    bpmHwFunc.emplace(P_SwDataMaskEn, bpmSetGetSwDataMaskEnFunc);
    bpmHwFunc.emplace(P_SwDataMaskSamples, bpmSetGetSwDataMaskSamplesFunc);
    bpmHwFunc.emplace(P_TbtTagEn, bpmSetGetTbtTagEnFunc);
    bpmHwFunc.emplace(P_TbtTagDly, bpmSetGetTbtTagDlyFunc);
    bpmHwFunc.emplace(P_TbtDataMaskEn, bpmSetGetTbtDataMaskEnFunc);
    bpmHwFunc.emplace(P_TbtDataMaskSamplesBeg, bpmSetGetTbtDataMaskSamplesBegFunc);
    bpmHwFunc.emplace(P_TbtDataMaskSamplesEnd, bpmSetGetTbtDataMaskSamplesEndFunc);
    bpmHwFunc.emplace(P_SwTagDesyncCntRst, bpmSetGetSwTagDesyncCntRstFunc);
    bpmHwFunc.emplace(P_SwTagDesyncCnt, bpmSetGetSwTagDesyncCntFunc);
    bpmHwFunc.emplace(P_TbtTagDesyncCntRst, bpmSetGetTbtTagDesyncCntRstFunc);
    bpmHwFunc.emplace(P_TbtTagDesyncCnt, bpmSetGetTbtTagDesyncCntFunc);
    bpmHwFunc.emplace(P_XOffset, bpmSetGetXOffsetFunc);
    bpmHwFunc.emplace(P_YOffset, bpmSetGetYOffsetFunc);

    bpmHwFunc.emplace(P_AdcGainFixedPointPos, bpmSetGetAdcGainFixedPointPos);

    bpmHwFunc.emplace(P_AdcCh0SwDir, bpmSetGetAmpGainCh0SwDir);
    bpmHwFunc.emplace(P_AdcCh1SwDir, bpmSetGetAmpGainCh1SwDir);
    bpmHwFunc.emplace(P_AdcCh2SwDir, bpmSetGetAmpGainCh2SwDir);
    bpmHwFunc.emplace(P_AdcCh3SwDir, bpmSetGetAmpGainCh3SwDir);
    bpmHwFunc.emplace(P_AdcCh0SwInv, bpmSetGetAmpGainCh0SwInv);
    bpmHwFunc.emplace(P_AdcCh1SwInv, bpmSetGetAmpGainCh1SwInv);
    bpmHwFunc.emplace(P_AdcCh2SwInv, bpmSetGetAmpGainCh2SwInv);
    bpmHwFunc.emplace(P_AdcCh3SwInv, bpmSetGetAmpGainCh3SwInv);

    bpmHwFunc.emplace(P_Monit1TagEn, bpmSetGetMonit1TagEnFunc);
    bpmHwFunc.emplace(P_Monit1TagDly, bpmSetGetMonit1TagDlyFunc);
    bpmHwFunc.emplace(P_Monit1DataMaskEn, bpmSetGetMonit1DataMaskEnFunc);
    bpmHwFunc.emplace(P_Monit1DataMaskSamplesBeg, bpmSetGetMonit1DataMaskSamplesBegFunc);
    bpmHwFunc.emplace(P_Monit1DataMaskSamplesEnd, bpmSetGetMonit1DataMaskSamplesEndFunc);
    bpmHwFunc.emplace(P_Monit1TagDesyncCntRst, bpmSetGetMonit1TagDesyncCntRstFunc);
    bpmHwFunc.emplace(P_Monit1TagDesyncCnt, bpmSetGetMonit1TagDesyncCntFunc);
    bpmHwFunc.emplace(P_MonitTagEn, bpmSetGetMonitTagEnFunc);
    bpmHwFunc.emplace(P_MonitTagDly, bpmSetGetMonitTagDlyFunc);
    bpmHwFunc.emplace(P_MonitDataMaskEn, bpmSetGetMonitDataMaskEnFunc);
    bpmHwFunc.emplace(P_MonitDataMaskSamplesBeg, bpmSetGetMonitDataMaskSamplesBegFunc);
    bpmHwFunc.emplace(P_MonitDataMaskSamplesEnd, bpmSetGetMonitDataMaskSamplesEndFunc);
    bpmHwFunc.emplace(P_MonitTagDesyncCntRst, bpmSetGetMonitTagDesyncCntRstFunc);
    bpmHwFunc.emplace(P_MonitTagDesyncCnt, bpmSetGetMonitTagDesyncCntFunc);
    bpmHwFunc.emplace(P_MonitPollTime, bpmSetGetMonitPollTimeFunc);
    /* FIXME: There is no BPM function to do that. Add funcionality to
     * FPGA firmware */
#if 0
    bpmHwFunc.emplace(P_Kq, bpmSetGetKqFunc);
#endif
    bpmHwFunc.emplace(P_SwMode, bpmSetGetAdcSwFunc);
    bpmHwFunc.emplace(P_SwDly, bpmSetGetAdcSwDlyFunc);
    bpmHwFunc.emplace(P_SwDivClk, bpmSetGetAdcSwDivClkFunc);
    bpmHwFunc.emplace(P_AdcTrigDir, bpmSetGetAdcTrigDirFunc);
    bpmHwFunc.emplace(P_AdcTrigTerm, bpmSetGetAdcTrigTermFunc);
    bpmHwFunc.emplace(P_AdcRand, bpmSetGetAdcRandFunc);
    bpmHwFunc.emplace(P_AdcDith, bpmSetGetAdcDithFunc);
    bpmHwFunc.emplace(P_AdcShdn, bpmSetGetAdcShdnFunc);
    bpmHwFunc.emplace(P_AdcPga, bpmSetGetAdcPgaFunc);
    bpmHwFunc.emplace(P_AdcTestData, bpmSetGetAdcTestDataFunc);
    bpmHwFunc.emplace(P_AdcClkSel, bpmSetGetAdcClkSelFunc);
    bpmHwFunc.emplace(P_AdcAD9510Dflt, bpmSetGetAdcAD9510DefaultsFunc);
    bpmHwFunc.emplace(P_AdcAD9510PllFunc, bpmSetGetAdcAD9510PllFunctionFunc);
    bpmHwFunc.emplace(P_AdcAD9510PllStatus, bpmSetGetAdcAD9510PllStatusFunc);
    bpmHwFunc.emplace(P_AdcAD9510ClkSel, bpmSetGetAdcAD9510ClkSelFunc);
    bpmHwFunc.emplace(P_AdcAD9510ADiv, bpmSetGetAdcAD9510ADivFunc);
    bpmHwFunc.emplace(P_AdcAD9510BDiv, bpmSetGetAdcAD9510BDivFunc);
    bpmHwFunc.emplace(P_AdcAD9510Prescaler, bpmSetGetAdcAD9510PrescalerFunc);
    bpmHwFunc.emplace(P_AdcAD9510RDiv, bpmSetGetAdcAD9510RDivFunc);
    bpmHwFunc.emplace(P_AdcAD9510PllPDown, bpmSetGetAdcAD9510PllPDownFunc);
    bpmHwFunc.emplace(P_AdcAD9510MuxStatus, bpmSetGetAdcAD9510MuxStatusFunc);
    bpmHwFunc.emplace(P_AdcAD9510CpCurrent, bpmSetGetAdcAD9510CPCurrentFunc);
    bpmHwFunc.emplace(P_AdcAD9510Outputs, bpmSetGetAdcAD9510OutputsFunc);
    bpmHwFunc.emplace(P_ActiveClkRstADCs, bpmSetGetActiveClkRstADCsFunc);
    bpmHwFunc.emplace(P_ActiveClkSi571Oe, bpmSetGetActiveClkSi571OeFunc);
    bpmHwFunc.emplace(P_AfcSi57xOe, bpmSetGetAfcSi57xOeFunc);
    bpmHwFunc.emplace(P_FmcPicoRngR0, bpmSetGetFmcPicoRngR0Func);
    bpmHwFunc.emplace(P_FmcPicoRngR1, bpmSetGetFmcPicoRngR1Func);
    bpmHwFunc.emplace(P_FmcPicoRngR2, bpmSetGetFmcPicoRngR2Func);
    bpmHwFunc.emplace(P_FmcPicoRngR3, bpmSetGetFmcPicoRngR3Func);

    bpmHwFunc.emplace(P_TimRcvPhaseMeasNavg, timRcvSetGetPhaseMeasNavgFunc);
    bpmHwFunc.emplace(P_TimRcvDMTDADeglitchThres, timRcvSetGetDMTDADeglitchThresFunc);
    bpmHwFunc.emplace(P_TimRcvDMTDBDeglitchThres, timRcvSetGetDMTDBDeglitchThresFunc);
    bpmHwFunc.emplace(P_TimRcvPhaseMeas, timRcvSetGetPhaseMeasFunc);
    bpmHwFunc.emplace(P_TimRcvDMTDAFreq, timRcvSetGetDMTDAFreqFunc);
    bpmHwFunc.emplace(P_TimRcvDMTDBFreq, timRcvSetGetDMTDBFreqFunc);

    bpmHwFunc.emplace(P_DataTrigChan, bpmSetGetAcqDataTrigChanFunc);

    bpmHwFunc.emplace(P_TriggerDataThres, bpmSetGetAcqDataTrigThresFunc);
    bpmHwFunc.emplace(P_TriggerDataPol, bpmSetGetAcqDataTrigPolFunc);
    bpmHwFunc.emplace(P_TriggerDataSel, bpmSetGetAcqDataTrigSelFunc);
    bpmHwFunc.emplace(P_TriggerDataFilt, bpmSetGetAcqDataTrigFiltFunc);
    bpmHwFunc.emplace(P_TriggerHwDly, bpmSetGetAcqHwDlyFunc);

    /* BPM HW Double Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHwFunc.emplace(P_AdcSi57xFreq, bpmSetGetAdcSi57xFreqFunc);
    bpmHwFunc.emplace(P_AdcSi57xFStartup, bpmSetGetAdcSi57xFStartupFunc);
    bpmHwFunc.emplace(P_AfcSi57xFreq, bpmSetGetAfcSi57xFreqFunc);
    bpmHwFunc.emplace(P_AfcSi57xFStartup, bpmSetGetAfcSi57xFStartupFunc);

    /* BPM HW Int32 with channel selection. Functions not mapped here are just written
     * to the parameter library */
    bpmHwFunc.emplace(P_AdcTestMode, bpmSetGetAdcTestModeFunc);
    bpmHwFunc.emplace(P_AdcRstModes,  bpmSetGetAdcRstModesFunc);
    bpmHwFunc.emplace(P_AdcCalStatus, bpmSetGetAdcCalStatusFunc);
    bpmHwFunc.emplace(P_AdcTemp, bpmSetGetAdcTempFunc);

    bpmHwFunc.emplace(P_TriggerDir, bpmSetGetTrigDirFunc);
    bpmHwFunc.emplace(P_TriggerDirPol, bpmSetGetTrigDirPolFunc);
    bpmHwFunc.emplace(P_TriggerRcvCntRst, bpmSetGetTrigRcvCntRstFunc);
    bpmHwFunc.emplace(P_TriggerTrnCntRst, bpmSetGetTrigTrnCntRstFunc);
    bpmHwFunc.emplace(P_TriggerCntRcv, bpmSetGetTrigCntRcvFunc);
    bpmHwFunc.emplace(P_TriggerCntTrn, bpmSetGetTrigCntTrnFunc);
    bpmHwFunc.emplace(P_TriggerRcvLen, bpmSetGetTrigRcvLenFunc);
    bpmHwFunc.emplace(P_TriggerTrnLen, bpmSetGetTrigTrnLenFunc);
    bpmHwFunc.emplace(P_TriggerRcvSrc, bpmSetGetTrigRcvSrcFunc);
    bpmHwFunc.emplace(P_TriggerTrnSrc, bpmSetGetTrigTrnSrcFunc);
    bpmHwFunc.emplace(P_TriggerRcvInSel, bpmSetGetTrigRcvSelFunc);
    bpmHwFunc.emplace(P_TriggerTrnOutSel, bpmSetGetTrigTrnSelFunc);

    bpmHwFunc.emplace(P_IntlkEn, bpmSetGetIntlkEnFunc);
    bpmHwFunc.emplace(P_IntlkClr, bpmSetGetIntlkClrFunc);
    bpmHwFunc.emplace(P_IntlkMinSumEn, bpmSetGetIntlkMinSumEnFunc);
    bpmHwFunc.emplace(P_IntlkTransEn, bpmSetGetIntlkTransEnFunc);
    bpmHwFunc.emplace(P_IntlkTransClr, bpmSetGetIntlkTransClrFunc);
    bpmHwFunc.emplace(P_IntlkAngEn, bpmSetGetIntlkAngEnFunc);
    bpmHwFunc.emplace(P_IntlkAngClr, bpmSetGetIntlkAngClrFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerX, bpmSetGetIntlkTransBiggerXFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerY, bpmSetGetIntlkTransBiggerYFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerLtcX, bpmSetGetIntlkTransBiggerLtcXFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerLtcY, bpmSetGetIntlkTransBiggerLtcYFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerAny, bpmSetGetIntlkTransBiggerAnyFunc);
    bpmHwFunc.emplace(P_IntlkTransBigger, bpmSetGetIntlkTransBiggerFunc);
    bpmHwFunc.emplace(P_IntlkTransBiggerLtc, bpmSetGetIntlkTransBiggerLtcFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerX, bpmSetGetIntlkAngBiggerXFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerY, bpmSetGetIntlkAngBiggerYFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerLtcX, bpmSetGetIntlkAngBiggerLtcXFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerLtcY, bpmSetGetIntlkAngBiggerLtcYFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerAny, bpmSetGetIntlkAngBiggerAnyFunc);
    bpmHwFunc.emplace(P_IntlkAngBigger, bpmSetGetIntlkAngBiggerFunc);
    bpmHwFunc.emplace(P_IntlkAngBiggerLtc, bpmSetGetIntlkAngBiggerLtcFunc);
    bpmHwFunc.emplace(P_Intlk, bpmSetGetIntlkFunc);
    bpmHwFunc.emplace(P_IntlkLtc, bpmSetGetIntlkLtcFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerX, bpmSetGetIntlkTransSmallerXFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerY, bpmSetGetIntlkTransSmallerYFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerLtcX, bpmSetGetIntlkTransSmallerLtcXFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerLtcY, bpmSetGetIntlkTransSmallerLtcYFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerAny, bpmSetGetIntlkTransSmallerAnyFunc);
    bpmHwFunc.emplace(P_IntlkTransSmaller, bpmSetGetIntlkTransSmallerFunc);
    bpmHwFunc.emplace(P_IntlkTransSmallerLtc, bpmSetGetIntlkTransSmallerLtcFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerX, bpmSetGetIntlkAngSmallerXFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerY, bpmSetGetIntlkAngSmallerYFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerLtcX, bpmSetGetIntlkAngSmallerLtcXFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerLtcY, bpmSetGetIntlkAngSmallerLtcYFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerAny, bpmSetGetIntlkAngSmallerAnyFunc);
    bpmHwFunc.emplace(P_IntlkAngSmaller, bpmSetGetIntlkAngSmallerFunc);
    bpmHwFunc.emplace(P_IntlkAngSmallerLtc, bpmSetGetIntlkAngSmallerLtcFunc);
    bpmHwFunc.emplace(P_IntlkMinSum, bpmSetGetIntlkMinSumFunc);
    bpmHwFunc.emplace(P_IntlkTransMaxX, bpmSetGetIntlkTransMaxXFunc);
    bpmHwFunc.emplace(P_IntlkTransMaxY, bpmSetGetIntlkTransMaxYFunc);
    bpmHwFunc.emplace(P_IntlkAngMaxX, bpmSetGetIntlkAngMaxXFunc);
    bpmHwFunc.emplace(P_IntlkAngMaxY, bpmSetGetIntlkAngMaxYFunc);
    bpmHwFunc.emplace(P_IntlkTransMinX, bpmSetGetIntlkTransMinXFunc);
    bpmHwFunc.emplace(P_IntlkTransMinY, bpmSetGetIntlkTransMinYFunc);
    bpmHwFunc.emplace(P_IntlkAngMinX, bpmSetGetIntlkAngMinXFunc);
    bpmHwFunc.emplace(P_IntlkAngMinY, bpmSetGetIntlkAngMinYFunc);
    bpmHwFunc.emplace(P_IntlkTransDiffX, bpmSetGetIntlkTransDiffXFunc);
    bpmHwFunc.emplace(P_IntlkTransDiffY, bpmSetGetIntlkTransDiffYFunc);
    bpmHwFunc.emplace(P_IntlkAngDiffX, bpmSetGetIntlkAngDiffXFunc);
    bpmHwFunc.emplace(P_IntlkAngDiffY, bpmSetGetIntlkAngDiffYFunc);

    bpmHwFunc.emplace(P_FofbCtrlErrClr,              fofbCtrlSetGeErrClrFunc);
    bpmHwFunc.emplace(P_FofbCtrlCcEnable,            fofbCtrlSetGetCcEnableFunc);
    bpmHwFunc.emplace(P_FofbCtrlTfsOverride,         fofbCtrlSetGetTfsOverrideFunc);
    bpmHwFunc.emplace(P_FofbCtrlBpmId,               fofbCtrlSetGetBpmIdFunc);
    bpmHwFunc.emplace(P_FofbCtrlTimeFrameLen,        fofbCtrlSetGetTimeFrameLenFunc);
    bpmHwFunc.emplace(P_FofbCtrlMgtPowerdown,        fofbCtrlSetGetMgtPowerdownFunc);
    bpmHwFunc.emplace(P_FofbCtrlMgtLoopback,         fofbCtrlSetGetMgtLoopbackFunc);
    bpmHwFunc.emplace(P_FofbCtrlTimeFrameDly,        fofbCtrlSetGetTimeFrameDlyFunc);
    bpmHwFunc.emplace(P_FofbCtrlGoldenOrbX,          fofbCtrlSetGetGoldenOrbXFunc);
    bpmHwFunc.emplace(P_FofbCtrlGoldenOrbY,          fofbCtrlSetGetGoldenOrbYFunc);
    bpmHwFunc.emplace(P_FofbCtrlCustFeature,         fofbCtrlSetGetCustFeatureFunc);
    bpmHwFunc.emplace(P_FofbCtrlRxPolarity,          fofbCtrlSetGetRxPolarityFunc);
    bpmHwFunc.emplace(P_FofbCtrlPayloadsel,          fofbCtrlSetGetPayloadselFunc);
    bpmHwFunc.emplace(P_FofbCtrlFofbdatasel,         fofbCtrlSetGetFofbdataselFunc);
    bpmHwFunc.emplace(P_FofbCtrlFirmwareVer,         fofbCtrlSetGetFirmwareVerFunc);
    bpmHwFunc.emplace(P_FofbCtrlSysStatus,           fofbCtrlSetGetSysStatusFunc);
    bpmHwFunc.emplace(P_FofbCtrlLinkPartner,         fofbCtrlSetGetLinkPartnerFunc);
    bpmHwFunc.emplace(P_FofbCtrlLinkUp,              fofbCtrlSetGetLinkUpFunc);
    bpmHwFunc.emplace(P_FofbCtrlTimeFrameCount,      fofbCtrlSetGetTimeFrameCountFunc);
    bpmHwFunc.emplace(P_FofbCtrlHardErrCnt,          fofbCtrlSetGetHardErrCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlSoftErrCnt,          fofbCtrlSetGetSoftErrCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlFrameErrCnt,         fofbCtrlSetGetFrameErrCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlRxPckCnt,            fofbCtrlSetGetRxPckCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlTxPckCnt,            fofbCtrlSetGetTxPckCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlFodProcessTime,      fofbCtrlSetGetFodProcessTimeFunc);
    bpmHwFunc.emplace(P_FofbCtrlBpmCnt,              fofbCtrlSetGetBpmCntFunc);
    bpmHwFunc.emplace(P_FofbCtrlToaRdEn,             fofbCtrlSetGetToaRdEnFunc);
    bpmHwFunc.emplace(P_FofbCtrlToaRdStr,            fofbCtrlSetGetToaRdStrFunc);
    bpmHwFunc.emplace(P_FofbCtrlToaData,             fofbCtrlSetGetToaDataFunc);
    bpmHwFunc.emplace(P_FofbCtrlRcbRdEn,             fofbCtrlSetGetRcbRdEnFunc);
    bpmHwFunc.emplace(P_FofbCtrlRcbRdStr,            fofbCtrlSetGetRcbRdStrFunc);
    bpmHwFunc.emplace(P_FofbCtrlRcbData,             fofbCtrlSetGetRcbDataFunc);

    lock();
    status = bpmClientConnect(this->pasynUserSelf);
    unlock();

    /* If we correct connect for this first time, libbpmclient
     * will ensure the reconnection to server if necessary, but we
     * must succeed here or we must abort completely */
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling bpmClientConnect, status=%d\n",
            driverName, functionName, status);
        exit(1);
    }

    /* Set the initial values of some parameters */

    for (int addr = 0; addr < NUM_ACQ_CORES_PER_BPM; ++addr) {
        setIntegerParam(addr, P_BPMMode,                   BPMModeMultiBunch);
        setIntegerParam(addr, P_BPMStatus,                 BPMStatusIdle);
    }

    setUIntDigitalParam(P_HarmonicNumber,
                                        HARMONIC_NUMBER,    0xFFFFFFFF);
    setUIntDigitalParam(P_AdcRate,      ADC_RATE_FACTOR,    0xFFFFFFFF);
    setUIntDigitalParam(P_TbtRate,      TBT_RATE_FACTOR,    0xFFFFFFFF);
    setUIntDigitalParam(P_FofbRate,     FOFB_RATE_FACTOR,   0xFFFFFFFF);
    setUIntDigitalParam(P_MonitRate,    MONIT_RATE_FACTOR,  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1Rate,   MONIT1_RATE_FACTOR, 0xFFFFFFFF);
    setUIntDigitalParam(P_SwMode,       ADC_DFLT_SW,        0xFFFFFFFF);
    setUIntDigitalParam(P_SwDly,        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwDivClk,     ADC_DFLT_DIV_CLK,   0xFFFFFFFF);

    setDoubleParam(P_ClkFreq,                               ADC_CLK_FREQ_UVX_DFLT);

    setUIntDigitalParam(P_AdcTrigDir,   0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcTrigTerm,  0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcRand,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcDith,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcShdn,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcPga,       0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcTestData,  0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcClkSel,    FMC_REF_CLK_SEL_1,  0xFFFFFFFF);
    setDoubleParam(P_AdcSi57xFreq,                          ADC_CLK_FREQ_UVX_DFLT);
    setDoubleParam(P_AdcSi57xFStartup,                      ADC_SI57X_FSTARTUP_DFLT);
    setDoubleParam(P_AfcSi57xFreq,                          AFC_SI57X_FREQ_DFLT);
    setDoubleParam(P_AfcSi57xFStartup,                      AFC_SI57X_FSTARTUP_DFLT);

    setUIntDigitalParam(P_AdcAD9510Dflt,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510PllFunc,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510PllStatus,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510ClkSel,
                                        AD9510_ADC_CLK_SEL_2,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510ADiv,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510BDiv,
                                        AD9510_ADC_DFLT_B_DIV,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510Prescaler,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510RDiv,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510PllPDown,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510MuxStatus,
                                        AD9510_ADC_DFLT_MUX_STATUS,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510CpCurrent,
                                        AD9510_ADC_DFLT_CP_CURRENT,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_AdcAD9510Outputs,
                                        AD9510_ADC_DFLT_OUTPUTS,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_ActiveClkRstADCs,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_ActiveClkSi571Oe,
                                        SI57X_ENABLE,       0xFFFFFFFF);

    setUIntDigitalParam(P_AfcSi57xOe,
                                        SI57X_ENABLE,       0xFFFFFFFF);

    setUIntDigitalParam(P_FmcPicoRngR0, FMCPICO_1MA_SCALE,  0xFFFFFFFF);
    setUIntDigitalParam(P_FmcPicoRngR1, FMCPICO_1MA_SCALE,  0xFFFFFFFF);
    setUIntDigitalParam(P_FmcPicoRngR2, FMCPICO_1MA_SCALE,  0xFFFFFFFF);
    setUIntDigitalParam(P_FmcPicoRngR3, FMCPICO_1MA_SCALE,  0xFFFFFFFF);

    setUIntDigitalParam(P_Kx,           10000000,           0xFFFFFFFF);
    setUIntDigitalParam(P_Ky,           10000000,           0xFFFFFFFF);
    setUIntDigitalParam(P_Ksum,         1,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwTagEn,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwDataMaskEn, 0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwDataMaskSamples,
                                        382,                0xFFFFFFFF);
    setUIntDigitalParam(P_TbtTagEn,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtTagDly,     0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtDataMaskEn, 0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtDataMaskSamplesBeg,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtDataMaskSamplesEnd,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwTagDesyncCntRst,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwTagDesyncCnt,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtTagDesyncCntRst,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TbtTagDesyncCnt,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1TagEn,  0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1TagDly, 0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1DataMaskEn,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1DataMaskSamplesBeg,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1DataMaskSamplesEnd,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1TagDesyncCntRst,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Monit1TagDesyncCnt,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitTagEn,   0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitTagDly,  0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitDataMaskEn,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitDataMaskSamplesBeg,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitDataMaskSamplesEnd,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitTagDesyncCntRst,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitTagDesyncCnt,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_Kq,           10000000,           0xFFFFFFFF);
    setIntegerParam(P_XOffset,                              0);
    setIntegerParam(P_YOffset,                              0);
    setIntegerParam(P_QOffset,                              0);

    setIntegerParam(P_AdcGainFixedPointPos, 0);
    getParamInteger(P_AdcGainFixedPointPos, &adc_gain_fixed_point_pos, 0);

    setDoubleParam(P_AdcCh0SwDir, 0);
    setDoubleParam(P_AdcCh1SwDir, 0);
    setDoubleParam(P_AdcCh2SwDir, 0);
    setDoubleParam(P_AdcCh3SwDir, 0);
    setDoubleParam(P_AdcCh0SwInv, 0);
    setDoubleParam(P_AdcCh1SwInv, 0);
    setDoubleParam(P_AdcCh2SwInv, 0);
    setDoubleParam(P_AdcCh3SwInv, 0);

    setUIntDigitalParam(P_TimRcvPhaseMeasNavg,
                                        TIMRCV_DFLT_PHASE_MEAS_NAVG,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_TimRcvDMTDADeglitchThres,
                                        TIMRCV_DFLT_DMTD_A_DEGLITCH_THRES,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_TimRcvDMTDBDeglitchThres,
                                        TIMRCV_DFLT_DMTD_B_DEGLITCH_THRES,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_TimRcvPhaseMeas,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRcvDMTDAFreq,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TimRcvDMTDBFreq,
                                        0,                  0xFFFFFFFF);

    /* Set acquisition parameters */
    for (int addr = 0; addr < NUM_ACQ_CORES_PER_BPM; ++addr) {
        setUIntDigitalParam(addr, P_SamplesPre,    1000,               0xFFFFFFFF);
        setUIntDigitalParam(addr, P_SamplesPost,   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_NumShots,      1,                  0xFFFFFFFF);
        setIntegerParam(    addr, P_Channel,                               CH_ADC);
        setDoubleParam(     addr, P_UpdateTime,                             1.0);
        setUIntDigitalParam(addr, P_Trigger,       ACQ_CLIENT_TRIG_SKIP,      0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerEvent,  TRIG_ACQ_STOP,      0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerRep,    0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerDataThres,
                                                   100,                0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerDataPol,
                                                   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerDataSel,
                                                   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerDataFilt,
                                                   1,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_TriggerHwDly,
                                                   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_DataTrigChan,
                                                   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_ChannelSampleSize,
                                                   DFLT_SAMPLE_SIZE,   0xFFFFFFFF);
        setUIntDigitalParam(addr, P_ChannelNumAtoms,
                                                   DFLT_NUM_ATOMS,     0xFFFFFFFF);
        setUIntDigitalParam(addr, P_ChannelAtomWidth,
                                                   DFLT_ATOM_WIDTH,    0xFFFFFFFF);
    }

    /* Acquisition PM parameters */
    setUIntDigitalParam(BPMIDPM, P_SamplesPre, SAMPLES_PRE_DEFAULT_PM(bpmMaxPoints),
                                                                0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_SamplesPost,SAMPLES_POST_DEFAULT_PM(bpmMaxPoints),
                                                                0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_NumShots,   NUM_SHOTS_DEFAULT_PM,
                                                                0xFFFFFFFF);
    setIntegerParam(    BPMIDPM, P_Channel,                        CH_DEFAULT_PM);
    setDoubleParam(     BPMIDPM, P_UpdateTime,                          1.0);
    setUIntDigitalParam(BPMIDPM, P_Trigger,    ACQ_CLIENT_TRIG_EXTERNAL,  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerEvent,
                                               TRIG_ACQ_START,     0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerRep, 1,                  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerDataThres,
                                               100,                0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerDataPol,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerDataSel,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerDataFilt,
                                               1,                  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_TriggerHwDly,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(BPMIDPM, P_DataTrigChan,
                                               0,                  0xFFFFFFFF);

    /* Write to HW */
    for (int i = P_SamplesPre; i < P_DataTrigChan+1; ++i) {
        setParamGeneric(i, BPMIDPM);
    }

    /* Set MONIT/SP parameters */
    setUIntDigitalParam(P_MonitPollTime, 4, 0xFFFFFFFF); // 4ms = 250 Hz
    setIntegerParam(P_MonitEnable,      0);    // Disable by default

    setDoubleParam(P_SPAmpA,            0.0);
    setDoubleParam(P_SPAmpB,            0.0);
    setDoubleParam(P_SPAmpC,            0.0);
    setDoubleParam(P_SPAmpD,            0.0);
    setDoubleParam(P_SPPosX,            0.0);
    setDoubleParam(P_SPPosY,            0.0);
    setDoubleParam(P_SPPosQ,            0.0);
    setDoubleParam(P_SPPosSum,          0.0);

    /* Set ADC parameters */
    for (int addr = 0; addr < ADC_NUM_CHANNELS; ++addr) {
        setUIntDigitalParam(addr, P_AdcTestMode,  0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRstModes,  ADC_RST_NORMAL_OP,  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcCalStatus, 0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcTemp,      0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegRead,   0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegReadData,
                                                  0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegReadAddr,
                                                  0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegWrite,  0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegWriteData,
                                                  0,                  0xFFFFFFFF);
        setUIntDigitalParam(addr, P_AdcRegWriteAddr,
                                                  0,                  0xFFFFFFFF);
    }

    /* Set parameters for all triggers, all Acquisition cores. FIXME (Caution):
     * We must deal with it at the Database level and be sure we are
     * accessing the right parameter! */
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        for (int addr = 0; addr < MAX_TRIGGERS; ++addr) {
            setIntegerParam(    i*MAX_TRIGGERS + addr, P_TriggerChan,                      CH_DFLT_TRIGGER_CHAN);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerDir,       1,              0xFFFFFFFF); /* FPGA Input */
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerDirPol,    1,              0xFFFFFFFF); /* Reverse Direction Polarity */
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerRcvCntRst, 0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerTrnCntRst, 0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerCntRcv,    0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerCntTrn,    0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerRcvLen,    1,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerTrnLen,    1,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerRcvSrc,    0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerTrnSrc,    0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerRcvInSel,  0,              0xFFFFFFFF);
            setUIntDigitalParam(i*MAX_TRIGGERS + addr, P_TriggerTrnOutSel, 0,              0xFFFFFFFF);
        }
    }

    /* Set trigger parameters for PM */
    for (int addr = 0; addr < MAX_WAVEFORM_TRIGGERS; ++addr) {
        setIntegerParam(    BPMIDPM*MAX_TRIGGERS + addr, P_TriggerChan,                      CH_DFLT_TRIGGER_CHAN);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerDir,       1,              0xFFFFFFFF); /* FPGA Input */
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerDirPol,    1,              0xFFFFFFFF); /* Reverse Direction Polarity */
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerRcvCntRst, 0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerTrnCntRst, 0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerCntRcv,    0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerCntTrn,    0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerRcvLen,    1,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerTrnLen,    1,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerRcvSrc,    0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerTrnSrc,    0,              0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerRcvInSel,  CH_DFLT_TRIGGER_SEL_PM_CHAN, 0xFFFFFFFF);
        setUIntDigitalParam(BPMIDPM*MAX_TRIGGERS + addr, P_TriggerTrnOutSel, 0,              0xFFFFFFFF);
    }

    /* Set Switching Trigger values */
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        setIntegerParam(    i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerChan,                      CH_DFLT_TRIGGER_CHAN);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerDir,       1,              0xFFFFFFFF); /* FPGA Input */
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerDirPol,    1,              0xFFFFFFFF); /* Reverse Direction Polarity */
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerRcvCntRst, 0,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerTrnCntRst, 0,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerCntRcv,    0,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerCntTrn,    0,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerRcvLen,    1,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerTrnLen,    1,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerRcvSrc,    1,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerTrnSrc,    0,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerRcvInSel,  1,              0xFFFFFFFF);
        setUIntDigitalParam(i*MAX_TRIGGERS + CH_DFLT_TRIGGER_SW_CHAN, P_TriggerTrnOutSel, 0,              0xFFFFFFFF);
    }

    /* Write to HW */
    for (int trig_core = 0; trig_core < NUM_TRIG_CORES_PER_BPM; ++trig_core) {
        for (int addr = 0; addr < MAX_TRIGGERS; ++addr) {
            for (int i = P_TriggerChan; i < P_TriggerTrnOutSel+1; ++i) {
                setParamGeneric(i, trig_core*MAX_TRIGGERS + addr);
            }
        }
    }

    /* Initialize interlock parameters */
    setUIntDigitalParam(P_IntlkEn,                   0,               0x1);
    setUIntDigitalParam(P_IntlkClr,                  0,               0x1);
    setUIntDigitalParam(P_IntlkMinSumEn,             0,               0x1);
    setUIntDigitalParam(P_IntlkTransEn,              0,               0x1);
    setUIntDigitalParam(P_IntlkTransClr,             0,               0x1);
    setUIntDigitalParam(P_IntlkAngEn,                0,               0x1);
    setUIntDigitalParam(P_IntlkAngClr,               0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerX,         0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerY,         0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerLtcX,      0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerLtcY,      0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerAny,       0,               0x1);
    setUIntDigitalParam(P_IntlkTransBigger,          0,               0x1);
    setUIntDigitalParam(P_IntlkTransBiggerLtc,       0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerX,           0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerY,           0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerLtcX,        0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerLtcY,        0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerAny,         0,               0x1);
    setUIntDigitalParam(P_IntlkAngBigger,            0,               0x1);
    setUIntDigitalParam(P_IntlkAngBiggerLtc,         0,               0x1);
    setUIntDigitalParam(P_Intlk,                     0,               0x1);
    setUIntDigitalParam(P_IntlkLtc,                  0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerX,        0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerY,        0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerLtcX,     0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerLtcY,     0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerAny,      0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmaller,         0,               0x1);
    setUIntDigitalParam(P_IntlkTransSmallerLtc,      0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerX,          0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerY,          0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerLtcX,       0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerLtcY,       0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerAny,        0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmaller,           0,               0x1);
    setUIntDigitalParam(P_IntlkAngSmallerLtc,        0,               0x1);
    setIntegerParam(P_IntlkMinSum,                                      0);
    setIntegerParam(P_IntlkTransMaxX,                                   0);
    setIntegerParam(P_IntlkTransMaxY,                                   0);
    setIntegerParam(P_IntlkAngMaxX,                                     0);
    setIntegerParam(P_IntlkAngMaxY,                                     0);
    setIntegerParam(P_IntlkTransMinX,                                   0);
    setIntegerParam(P_IntlkTransMinY,                                   0);
    setIntegerParam(P_IntlkAngMinX,                                     0);
    setIntegerParam(P_IntlkAngMinY,                                     0);
    setIntegerParam(P_IntlkTransDiffX, 0);
    setIntegerParam(P_IntlkTransDiffY, 0);
    setIntegerParam(P_IntlkAngDiffX, 0);
    setIntegerParam(P_IntlkAngDiffY, 0);

    for (int addr: {0, 8}) {
        setUIntDigitalParam(addr, P_FofbCtrlErrClr,                     0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlCcEnable,                   0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlTfsOverride,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlBpmId,                      0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlTimeFrameLen,               0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlMgtPowerdown,               0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlMgtLoopback,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlTimeFrameDly,               0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlGoldenOrbX,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlGoldenOrbY,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlCustFeature,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlRxPolarity,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlPayloadsel,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlFofbdatasel,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlFirmwareVer,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlSysStatus,                  0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlLinkUp,                     0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlTimeFrameCount,             0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlFodProcessTime,             0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlBpmCnt,                     0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlToaRdEn,                    0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlToaRdStr,                   0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlToaData,                    0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlRcbRdEn,                    0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlRcbRdStr,                   0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlRcbData,                    0,              0xFFFFFFFF);
    }
    for (int addr = 0; addr < NUM_FOFB_CC_CORES_PER_FOFB * NUM_FOFB_CC_CHANNELS_PER_FOFB_CC; addr++) {
        setUIntDigitalParam(addr, P_FofbCtrlLinkPartner,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlHardErrCnt,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlSoftErrCnt,                 0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlFrameErrCnt,                0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlRxPckCnt,                   0,              0xFFFFFFFF);
        setUIntDigitalParam(addr, P_FofbCtrlTxPckCnt,                   0,              0xFFFFFFFF);
    }

#if 0
    /* Read values from HW */
    readAD9510Params (0xFFFFFFFF, 0);
    readSi57xParams (0);

    for (int addr = 0; addr < ADC_NUM_CHANNELS; ++addr) {
        readADCsParams (0xFFFFFFFF, addr);
    }

    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        for (int addr = 0; addr < MAX_TRIGGERS; ++addr) {
            readTriggerParams(0xFFFFFFFF, i*MAX_TRIGGERS + addr);
        }
    }

#if 0
    readFMCPicoParams(0xFFFFFFFF, 0);
#endif

#endif

    /* Do callbacks so higher layers see any changes. Call callbacks for every addr */
    for (int i = 0; i < MAX_ADDR; ++i) {
        callParamCallbacks(i);
    }

    /* Create the thread that computes the waveforms in the background */
    for (int i = 0; i < NUM_ACQ_CORES_PER_BPM; ++i) {
        /* Assign task parameters passing the ACQ/Trigger instance ID as parameter.
         * The other parameters are already set-up*/
        taskParams[i].drvBPMp = this;
        status = (asynStatus)(epicsThreadCreate("drvBPMTask",
                    epicsThreadPriorityMedium,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)::acqTask,
                    &taskParams[i]) == NULL);
        if (status) {
            printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
            return;
        }
    }

    /* Create the thread that computes the SP waveforms in the background */
    /* Assign task parameters passing the ACQ/Trigger instance ID as parameter.
     * The other parameters are already set-up*/
    taskSPParams[BPMIDReg].drvBPMp = this;
    status = (asynStatus)(epicsThreadCreate("drvBPMSPTask",
                epicsThreadPriorityMedium,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)::acqSPTask,
                &taskSPParams[BPMIDReg]) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    /* Create monitoring thread */
    taskMonitParams.drvBPMp = this;
    status = (asynStatus)(epicsThreadCreate("drvBPMMonitTask",
                epicsThreadPriorityHigh,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)::acqMonitTask,
                &taskMonitParams) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

#if 0
    /* This driver supports MAX_ADDR with autoConnect=1.  But there are only records
    * connected to addresses 0-3, so addresses 4-11 never show as "connected"
    * since nothing ever calls pasynManager->queueRequest.  So we do an
    * exceptionConnect to each address so asynManager will show them as connected.
    * Note that this is NOT necessary for the driver to function correctly, the
    * NDPlugins will still get called even for addresses that are not "connected".
    * It is just to avoid confusion.
    * */
    for (i=0; i<MAX_ADDR; ++i) {
        pasynUser = pasynManager->createAsynUser(0,0);
        pasynManager->connectDevice(pasynUser, portName, i);
        pasynManager->exceptionConnect(pasynUser);
    }
#endif

    epicsAtExit(exitHandlerC, this);
    return;

invalid_bpm_number_err:
    free (this->endpoint);
endpoint_dup_err:
    return;
}

/** Destructor for the drvBPM class.
 */
drvBPM::~drvBPM()
{
    asynStatus status = asynSuccess;
    const char *functionName = "~drvBPM";

    lock();
    status = bpmClientDisconnect(this->pasynUserSelf);
    unlock();
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling bpmClientDisconnect, status=%d\n",
            driverName, functionName, status);
    }

    free (this->endpoint);
    this->endpoint = NULL;
    free (this->bpmType);
    this->bpmType = NULL;
    free (this->bpmPortName);
    this->bpmPortName = NULL;
}

asynStatus drvBPM::connect(asynUser* pasynUser)
{
    return bpmClientConnect(pasynUser);
}

asynStatus drvBPM::bpmClientConnect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;
    const char *bpmLogFile = "stdout";
    const char *functionName = "bpmClientConnect";

    /* Connect BPM */
    if (bpmClient == NULL) {
        bpmClient = halcs_client_new_time (endpoint, verbose, bpmLogFile, timeout);
        if (bpmClient == NULL) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s bpmClientConnect failure to create bpmClient instance\n",
                    driverName, functionName);
            status = asynError;
            goto create_halcs_client_err;
        }
    }

    /* Connect BPM Monit */
    if (bpmClientMonit == NULL) {
        bpmClientMonit = halcs_client_new_time (endpoint, verbose, bpmLogFile, timeout);
        if (bpmClientMonit == NULL) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s bpmClientConnect failure to create bpmClientMonit instance\n",
                    driverName, functionName);
            status = asynError;
            goto create_halcs_client_monit_err;
        }
    }

    /* Connect ACQ BPM parameter clients*/
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcqParam[i] == NULL) {
            bpmClientAcqParam[i] = acq_client_new_time (endpoint, verbose, bpmLogFile, timeout);
            if (bpmClientAcqParam[i] == NULL) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                        "%s:%s bpmClientConnect failure to create bpmClientAcqParam[%d] instance\n",
                        driverName, functionName, i);
                status = asynError;
                goto create_halcs_client_acq_param_err;
            }
        }
    }

    /* Connect ACQ BPM clients */
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcq[i] == NULL) {
            bpmClientAcq[i] = acq_client_new_time (endpoint, verbose, bpmLogFile, timeout);
            if (bpmClientAcq[i] == NULL) {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                        "%s:%s bpmClientConnect failure to create bpmClientAcq[%d] instance\n",
                        driverName, functionName, i);
                status = asynError;
                goto create_halcs_client_acq_err;
            }
        }
    }

    asynPrint(pasynUser, ASYN_TRACE_FLOW,
        "%s:%s: BPM client connected\n",
        driverName, functionName);

    pasynManager->exceptionConnect(pasynUser);

    return status;

create_halcs_client_acq_err:
    /* Destroy possible uninitialized bpmClientAcq instances */
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcq[i] != NULL) {
            acq_client_destroy (&bpmClientAcq[i]);
        }
    }
create_halcs_client_acq_param_err:
    /* Destroy possible uninitialized bpmClientAcqParam instances */
    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcqParam[i] != NULL) {
            acq_client_destroy (&bpmClientAcqParam[i]);
        }
    }
    halcs_client_destroy (&bpmClientMonit);
create_halcs_client_monit_err:
    /* Destroy regular bpmClient instance */
    halcs_client_destroy (&bpmClient);
create_halcs_client_err:
    return status;
}

asynStatus drvBPM::disconnect(asynUser* pasynUser)
{
    return bpmClientDisconnect(pasynUser);
}

asynStatus drvBPM::bpmClientDisconnect(asynUser *pasynUser)
{
    asynPrint(pasynUser, ASYN_TRACE_FLOW,
            "%s: calling bpmClientDisconnect\n",
            driverName);
    asynStatus status = asynSuccess;

    if (bpmClient != NULL) {
        halcs_client_destroy (&bpmClient);
    }

    if (bpmClientMonit != NULL) {
        halcs_client_destroy (&bpmClientMonit);
    }

    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcqParam[i] != NULL) {
            acq_client_destroy (&bpmClientAcqParam[i]);
        }
    }

    for (int i = 0; i < NUM_TRIG_CORES_PER_BPM; ++i) {
        if (bpmClientAcq[i] != NULL) {
            acq_client_destroy (&bpmClientAcq[i]);
        }
    }

    pasynManager->exceptionDisconnect(pasynUser);
    return status;
}

void acqTask(void *drvPvt)
{
   taskParams_t *pPvt = (taskParams_t *)drvPvt;
   pPvt->drvBPMp->acqTask(pPvt->coreID, pPvt->pollTime, pPvt->autoStart);
}

void acqSPTask(void *drvPvt)
{
   taskParams_t *pPvt = (taskParams_t *)drvPvt;
   pPvt->drvBPMp->acqSPTask(pPvt->coreID, pPvt->pollTime, pPvt->autoStart);
}

void acqMonitTask(void *drvPvt)
{
   taskParams_t *pPvt = (taskParams_t *)drvPvt;
   pPvt->drvBPMp->acqMonitTask();
}

/********************************************************************/
/******************* BPM Acquisition functions **********************/
/********************************************************************/

asynStatus drvBPM::initAcqPM(int coreID)
{
    static const char *functionName = "initAcqPM";
    asynStatus status = asynSuccess;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: setting Post-Mortem parameters for coreID = %d\n",
        driverName, functionName, coreID);

    /* Set paramters for Post-Mortem */
    setUIntDigitalParam(coreID, P_SamplesPre,  SAMPLES_PRE_DEFAULT_PM(bpmMaxPoints),
                                                                 0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_SamplesPost, SAMPLES_POST_DEFAULT_PM(bpmMaxPoints),
                                                                 0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_NumShots,    NUM_SHOTS_DEFAULT_PM,
                                                                 0xFFFFFFFF);
    setIntegerParam(    coreID, P_Channel,                         CH_DEFAULT_PM);
    setDoubleParam(     coreID, P_UpdateTime,                           1.0);
    setUIntDigitalParam(coreID, P_Trigger,     ACQ_CLIENT_TRIG_EXTERNAL,  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerEvent,
                                               TRIG_ACQ_START,     0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerRep,  1,                  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerDataThres,
                                               100,                0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerDataPol,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerDataSel,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerDataFilt,
                                               1,                  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_TriggerHwDly,
                                               0,                  0xFFFFFFFF);
    setUIntDigitalParam(coreID, P_DataTrigChan,
                                               0,                  0xFFFFFFFF);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(coreID);

    /* Start triggered acquisition */
    status = setAcqEvent(0xFFFFFFFF, coreID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling setAcqEvent, status=%d\n",
            driverName, functionName, status);
        goto set_acq_trig;
    }
    return status;

set_acq_trig:
    return status;
}

/* This should only be called by asyn thread, not Acquisition ones */
asynStatus drvBPM::setAcqTrig(int coreID, acq_client_trig_e trig)
{
    static const char *functionName = "setAcqTrig";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    asynStatus status = asynSuccess;
    char service[SERVICE_NAME_SIZE];

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    err = acq_set_trig (bpmClientAcqParam[coreID], service, trig);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling halcs_set_acq_trig for service = %s, trigger = %d\n",
                driverName, functionName, service, trig);
        status = asynError;
        goto halcs_acq_trig_err;
    }

halcs_acq_trig_err:
get_service_err:
    return status;
}

/* This can only return if the ACQ engine is IDLE or waiting
 * for some trigger (External, Data or Software) */
/* This should only be called by asyn thread, not Acquisition ones */
bpm_status_types drvBPM::getBPMInitAcqStatus(int coreID)
{
    bpm_status_types bpmStatus = BPMStatusErrAcq;
    asynStatus status = asynSuccess;
    halcs_client_err_e herr = HALCS_CLIENT_SUCCESS;
    uint32_t trig = 0;
    const char* functionName = "getBPMAcqStatus";
    char service[SERVICE_NAME_SIZE];

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    /* Have ACQ engine completed some work or is it still busy? */
    herr = acq_check (bpmClientAcqParam[coreID], service);
    if (herr == HALCS_CLIENT_SUCCESS) {
        return BPMStatusIdle;
    }

    /* If the ACQ is doing something we need to figure it out what is it */
    herr = acq_get_trig (bpmClientAcqParam[coreID], service, &trig);
    if (herr != HALCS_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling halcs_get_acq_trig, status=%d\n",
            driverName, functionName, herr);
        goto get_service_err;
    }

    switch (trig) {
        case ACQ_CLIENT_TRIG_SKIP:
            /* If we are doing something and the trigger is set to SKIP,
             * then we are acquiring */
            bpmStatus = BPMStatusAcquire;
            break;

        case ACQ_CLIENT_TRIG_EXTERNAL:
            bpmStatus = BPMStatusTriggerHwExtWaiting;
            break;

        case ACQ_CLIENT_TRIG_DATA_DRIVEN:
            bpmStatus = BPMStatusTriggerHwDataWaiting;
            break;

        case ACQ_CLIENT_TRIG_SOFTWARE:
            bpmStatus = BPMStatusTriggerSwWaiting;
            break;

        default:
            bpmStatus = BPMStatusErrAcq;
    }

get_service_err:
    return bpmStatus;
}

static bool acqIsBPMStatusWaitSomeTrigger(int bpmStatus)
{
    if (bpmStatus == BPMStatusTriggerHwExtWaiting ||
        bpmStatus == BPMStatusTriggerHwDataWaiting ||
        bpmStatus == BPMStatusTriggerSwWaiting) {
        return true;
    }

    return false;
}

static bool acqIsBPMStatusErr(int bpmStatus)
{
    if (bpmStatus == BPMStatusErrAcq ||
        bpmStatus == BPMStatusAborted ||
        bpmStatus == BPMStatusErrTooManyPoints ||
        bpmStatus == BPMStatusErrTooFewPoints ||
        bpmStatus == BPMStatusErrNoMem ||
        bpmStatus == BPMStatusErrAcqOFlow) {
        return true;
    }

    return false;
}

/*
 * BPM acquisition functions
 */

/** Acquisition task that runs as a separate thread.
 *  CAUTION. FIXME? Only one acquisition task is working at any given time: MultiMode or SinglePass
*/
void drvBPM::acqTask(int coreID, double pollTime, bool autoStart)
{
    int status = asynSuccess;
    asynUser *pasynUser = NULL;
    epicsUInt32 num_samples_pre;
    epicsUInt32 num_samples_post;
    epicsUInt32 num_shots;
    epicsUInt32 sampleSize = 16; /* bytes */
    epicsUInt32 numAtoms = 4;
    epicsUInt32 atomWidth = 32; /* bits */
    int channel;
    int bpmMode;
    epicsUInt32 trigger;
    double updateTime;
    double delay;
    int hwAmpChannel = 0;
    int acqCompleted = 0;
    int bpmStatus = 0;
    int newAcq = 1;
    bool autoStartFirst = autoStart;
    epicsTimeStamp now;
    epicsFloat64 timeStamp;
    NDArray *pArrayAllChannels = NULL;
    NDDataType_t NDType = NDInt32;
    epicsTimeStamp startTime;
    epicsTimeStamp endTime;
    double elapsedTime;
    int arrayCounter;
    size_t dims[MAX_WVF_DIMS];
    static const char *functionName = "acqTask";

    /* Create an asynUser. FIXME: we should probably create a callback
     * for the processCallback, which would be called on a queuePortLock ()
     * so as to not block all addresses, just the ones related to that
     * specific BOARD */
    pasynUser = pasynManager->createAsynUser(0, 0);
    pasynUser->timeout = BPM_TIMEOUT;
    status = pasynManager->connectDevice(pasynUser, bpmPortName, 0);
    if(status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: connectDevice failed, status=%d\n",
            driverName, functionName, status);
        return;
    }

    /* Loop forever */
    lock ();
    while (1) {
        /* Free buffers if needed*/
        if (pArrayAllChannels) {
            pArrayAllChannels->release ();
            pArrayAllChannels = NULL;
        }

        getIntegerParam(coreID, P_BPMStatus, &bpmStatus);

        /* Check if we received a stop event */
        status = epicsEventWaitWithTimeout(this->stopAcqEventId[BPMModeMultiBunch][coreID], pollTime);
        if (status == epicsEventWaitOK || !repetitiveTrigger[BPMModeMultiBunch][coreID] || acqIsBPMStatusErr(bpmStatus)) {
            /* We got a stop event, stop repetitive acquisition */
            readingActive[BPMModeMultiBunch][coreID] = 0;

            /* Wait until we are in MultiBunch mode */
            getIntegerParam(coreID, P_BPMMode, &bpmMode);
            if (bpmMode != BPMModeMultiBunch) {
                unlock ();
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: waiting for BPMMode = MultiBunch\n", driverName, functionName);
                epicsEventWait(activeAcqEventId[BPMModeMultiBunch][coreID]);
                lock ();
            }

            /* Default to new acquisition. If we are waiting for a trigger
             * we will change this */
            newAcq = 1;

            /* Now, we can either be finished with the previous acquisition
             * (repetitive or not) or we could be waiting for a trigger armed
             * outside this thread (for now, the only option is the case when
             * you set a trigger and then exit the IOC for some reason) */
            if (!acqCompleted && acqIsBPMStatusWaitSomeTrigger(bpmStatus)) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: waiting for trigger\n", driverName, functionName);
                newAcq = 0;
            }
            /* Only change state to IDLE if we are not in a error state and we have just acquired some data */
            else if (!acqIsBPMStatusErr(bpmStatus)) {
                setIntegerParam(coreID, P_BPMStatus, BPMStatusIdle);
                callParamCallbacks(coreID);
            }

            /* We have consumed our data. This is important if we abort the next
             * acquisition, as we can detect that the current acquisition is completed,
             * which would be wrong */
            acqCompleted = 0;

            /* Only wait for the startEvent if we are waiting for a
             * new acquisition */
            if (newAcq && !autoStartFirst) {
                unlock();
                /* Release the lock while we wait for an event that says acquire has started, then lock again */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: waiting for acquire to start\n", driverName, functionName);
                epicsEventWait(startAcqEventId[BPMModeMultiBunch][coreID]);
                lock();
            }
            readingActive[BPMModeMultiBunch][coreID] = 1;
            autoStartFirst = 0;
        }

        /* We are acquiring. Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* Set the parameter in the parameter library. */
        getUIntDigitalParam(coreID , P_Trigger      , &trigger          , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_SamplesPre   , &num_samples_pre  , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_SamplesPost  , &num_samples_post , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_NumShots     , &num_shots        , 0xFFFFFFFF);
        getIntegerParam(    coreID , P_Channel      , &channel);
        getDoubleParam(     coreID , P_UpdateTime   , &updateTime);
        getUIntDigitalParam(coreID,  P_ChannelSampleSize,
                                                      &sampleSize,        0xFFFFFFFF);
        getUIntDigitalParam(coreID,  P_ChannelNumAtoms,
                                                      &numAtoms,          0xFFFFFFFF);
        getUIntDigitalParam(coreID,  P_ChannelAtomWidth,
                                                      &atomWidth,         0xFFFFFFFF);

        /* Convert bit to byte */
        atomWidth = atomWidth/8;

        if(numAtoms > MAX_WVF_AMP_TYPES) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unsupported numAtoms > %d. Reduce this value in the gateware\n",
                    driverName, functionName, MAX_WVF_AMP_TYPES);
            continue;
        }

        /* Convert user channel into hw channel */
        hwAmpChannel = channelMap[channel].HwAmpChannel;
        if(hwAmpChannel < 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid HwAmpChannel channelMap for channel %d\n",
                    driverName, functionName, hwAmpChannel);
            continue;
        }

        /* Our waveform will have "num_samples_pres + num_samples_post"
         * samples in each dimension */
        dims[0] = numAtoms;
        dims[1] = (num_samples_pre + num_samples_post)*num_shots;

        /* We can't acquire something with 0 points */
        if (dims[1] == 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid number of points for acquisition (= 0)\n",
                    driverName, functionName);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrTooFewPoints);
            callParamCallbacks(coreID);
            continue;
        }

        /* dims[1] must not exceed bpmMaxPoints, as we use this to alloc
         * points for the Waveform Plugins */
        if (dims[1] > bpmMaxPoints) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid number of points for acquisition (> %d)\n",
                    driverName, functionName, bpmMaxPoints);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrTooManyPoints);
            callParamCallbacks(coreID);
            continue;
        }

        /* Waveform statistics */
        epicsTimeGetCurrent(&now);
        getIntegerParam(NDArrayCounter, &arrayCounter);
        arrayCounter++;
        setIntegerParam(NDArrayCounter, arrayCounter);

        status = getAcqNDArrayType(coreID, hwAmpChannel, atomWidth, &NDType);
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to determine NDArray type for acquisition, coreID = %d\n",
                    driverName, functionName, coreID);
            continue;
        }

        pArrayAllChannels = pNDArrayPool->alloc(MAX_WVF_DIMS, dims, NDType, 0, NULL);
        if (pArrayAllChannels == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayAllChannels\n",
                driverName, functionName);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrNoMem);
            callParamCallbacks(coreID);
            continue;
        }
        pArrayAllChannels->uniqueId = arrayCounter;
        timeStamp = now.secPastEpoch + now.nsec / 1.e9;
        pArrayAllChannels->timeStamp = timeStamp;
        pArrayAllChannels->epicsTS.secPastEpoch = now.secPastEpoch;
        pArrayAllChannels->epicsTS.nsec = now.nsec;
        getAttributes(pArrayAllChannels->pAttributeList);

        /* Just start the acquisition if we are not already acquiring */
        if (newAcq) {
            /* Tell we are acquiring just before we actually start it */
            setIntegerParam(coreID, P_BPMStatus, BPMStatusAcquire);
            callParamCallbacks(coreID);

            /* Do acquisition */
            unlock();
            pasynManager->lockPort(pasynUser);
            status = startAcq(coreID, hwAmpChannel, num_samples_pre, num_samples_post,
                    num_shots);
            pasynManager->unlockPort(pasynUser);
            lock();

            if (status == asynSuccess) {
                /* FIXME: Improve BPMStatus trigger waiting. The information
                 * about waiting for trigger is not totally accurate here.
                 * Although, we will for SW or HW trigger in a short time,
                 * we are not actually there yet ...
                 */
                if (trigger == ACQ_CLIENT_TRIG_EXTERNAL) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerHwExtWaiting);
                }
                else if (trigger == ACQ_CLIENT_TRIG_DATA_DRIVEN) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerHwDataWaiting);
                }
                else if (trigger == ACQ_CLIENT_TRIG_SOFTWARE) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerSwWaiting);
                }

                callParamCallbacks(coreID);
            }
            else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unable to acquire waveform\n",
                        driverName, functionName);
                /* Could not start acquisition. Invalid parameters */
                setIntegerParam(coreID, P_BPMStatus, BPMStatusErrAcq);
                callParamCallbacks(coreID);
                continue;
            }
        }

        /* Wait for acquisition to complete, but allow acquire stop events to be handled */
        while (1) {
            unlock();
            status = epicsEventWaitWithTimeout(this->abortAcqEventId[BPMModeMultiBunch][coreID], pollTime);
            lock();
            if (status == epicsEventWaitOK) {
                /* We got a stop event, abort acquisition */
                abortAcq(coreID);
                setIntegerParam(coreID, P_BPMStatus, BPMStatusAborted);
                callParamCallbacks(coreID);
                break;
            }
            else {
                acqCompleted = checkAcqCompletion(coreID);
            }

            /* Acquisition overflow */
            if (acqCompleted == -1) {
                setIntegerParam(coreID, P_BPMStatus, BPMStatusErrAcqOFlow);
                callParamCallbacks(coreID);
                break;
            }

            if (acqCompleted == 1) {
                /* Get curve */
                unlock();
                pasynManager->lockPort(pasynUser);
                getAcqCurve(coreID, pArrayAllChannels, hwAmpChannel, num_samples_pre,
                        num_samples_post, num_shots);
                pasynManager->unlockPort(pasynUser);
                lock();
                break;
            }
        }

        /* Only do callbacks and calculate position if we could acquire some
         * data */
        if (acqCompleted == 1) {
            /* Do callbacks on the full waveform (all channels interleaved) */
            doCallbacksGenericPointer(pArrayAllChannels, NDArrayData,
                    channelMap[channel].NDArrayAmp[coreID][WVF_AMP_ALL]);

            /* Copy AMP data to arrays for each type of data, do callbacks on that */
            status = deinterleaveNDArray(pArrayAllChannels, channelMap[channel].NDArrayAmp[coreID],
                    dims[0], arrayCounter, &now);
            if (status != asynSuccess) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unable to deinterleave NDArray\n",
                        driverName, functionName);
                continue;
            }

            /* Calculate positions and call callbacks */
            status = computePositions(coreID, pArrayAllChannels, channel, &now);
            if (status != asynSuccess) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unable to compute positions\n",
                        driverName, functionName);
                continue;
            }
        }

        /* Release buffers */
        pArrayAllChannels->release();
        pArrayAllChannels = NULL;
        callParamCallbacks(coreID);

        /* If we are in repetitive mode then sleep for the acquire period minus elapsed time. */
        if (repetitiveTrigger[BPMModeMultiBunch][coreID]) {
            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            delay = updateTime - elapsedTime;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                     "%s:%s: delay=%f\n",
                      driverName, functionName, delay);
            if (delay >= 0.0) {
                /* We set the status to indicate we are in the period delay */
                setIntegerParam(coreID, P_BPMStatus, BPMStatusWaiting);
                callParamCallbacks(coreID);
                unlock();
                epicsEventWaitWithTimeout(this->stopAcqEventId[BPMModeMultiBunch][coreID], delay);
                lock();
            }
        }
    }
}

/** Acquisition task for Single Pass BPM mode that runs as a separate thread.
 *  CAUTION. FIXME? Only one acquisition task is working at any given time: MultiMode or SinglePass
*/
void drvBPM::acqSPTask(int coreID, double pollTime, bool autoStart)
{
    int status = asynSuccess;
    asynUser *pasynUser = NULL;
    epicsUInt32 num_samples_pre;
    epicsUInt32 num_samples_post;
    epicsUInt32 num_shots;
    epicsUInt32 sampleSize = 16; /* bytes */
    epicsUInt32 numAtoms = 4;
    epicsUInt32 atomWidth = 32; /* bits */
    int channel;
    int bpmMode;
    epicsUInt32 trigger;
    epicsUInt32 TriggerDataThres;
    epicsUInt32 TriggerDataPol;
    epicsUInt32 TriggerDataSel;
    epicsUInt32 TriggerDataFilt;
    epicsUInt32 TriggerHwDly;
    epicsUInt32 DataTrigChan;
    bpm_sample_t bpm_sample = {0};
    char service[SERVICE_NAME_SIZE];
    char service_board[SERVICE_NAME_SIZE];
    int hwAmpChannel = 0;
    int acqCompleted = 0;
    int bpmStatus = 0;
    int interrupted = 0;
    bool autoStartFirst = autoStart;
    epicsTimeStamp now;
    epicsFloat64 timeStamp;
    NDArray *pArrayAllChannels = NULL;
    NDDataType_t NDType = NDInt32;
    epicsTimeStamp startTime;
    int arrayCounter;
    size_t dims[MAX_WVF_DIMS];
    bpm_single_pass_t *bpm_single_pass = NULL;
    static const char *functionName = "acqSPTask";

    /* Create an asynUser. FIXME: we should probably create a callback
     * for the processCallback, which would be called on a queuePortLock ()
     * so as to not block all addresses, just the ones related to that
     * specific BOARD */
    pasynUser = pasynManager->createAsynUser(0, 0);
    pasynUser->timeout = BPM_TIMEOUT;
    status = pasynManager->connectDevice(pasynUser, bpmPortName, 0);
    if(status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: connectDevice failed, status=%d\n",
            driverName, functionName, status);
        return;
    }

    /* Loop forever */
    lock ();
    while (1) {
        /* Free buffers if needed*/
        if (pArrayAllChannels) {
            pArrayAllChannels->release ();
            pArrayAllChannels = NULL;
        }

        /* Clear out any flags*/
        interrupted = 0;

        /* We got a stop event, stop acquisition */
        readingActive[BPMModeSinglePass][coreID] = 0;

        /* Wait until we are in SinglePass mode */
        getIntegerParam(coreID, P_BPMMode, &bpmMode);
        if (bpmMode != BPMModeSinglePass) {
            unlock ();
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for BPMMode = SinglePass\n", driverName, functionName);
            epicsEventWait(activeAcqEventId[BPMModeSinglePass][coreID]);
            lock ();
        }

        getIntegerParam(coreID, P_BPMStatus, &bpmStatus);

        /* Only change state to IDLE if we are not in a error state and we have just acquired some data */
        if (!acqIsBPMStatusErr(bpmStatus)) {
            setIntegerParam(coreID, P_BPMStatus, BPMStatusIdle);
            callParamCallbacks(coreID);
        }

        /* We have consumed our data. This is important if we abort the next
         * acquisition, as we can detect that the current acquisition is completed,
         * which would be wrong */
        acqCompleted = 0;

        /* Only wait for the startEvent if we are waiting for a
         * new acquisition */
        if (!autoStartFirst) {
            unlock();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquire to start\n", driverName, functionName);
            epicsEventWait(startAcqEventId[BPMModeSinglePass][coreID]);
            lock();
        }
        readingActive[BPMModeSinglePass][coreID] = 1;
        autoStartFirst = 0;

        /* We are acquiring. Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* Set the parameter in the parameter library. */
        getUIntDigitalParam(coreID , P_Trigger      , &trigger          , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_SamplesPre   , &num_samples_pre  , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_SamplesPost  , &num_samples_post , 0xFFFFFFFF);
        getUIntDigitalParam(coreID , P_NumShots     , &num_shots        , 0xFFFFFFFF);
        getUIntDigitalParam(coreID,  P_ChannelSampleSize,
                                                      &sampleSize,        0xFFFFFFFF);
        getUIntDigitalParam(coreID,  P_ChannelNumAtoms,
                                                      &numAtoms,          0xFFFFFFFF);
        getUIntDigitalParam(coreID,  P_ChannelAtomWidth,
                                                      &atomWidth,          0xFFFFFFFF);

        /* Convert bit to byte */
        atomWidth = atomWidth/8;

        if(numAtoms > MAX_WVF_AMP_TYPES) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unsupported numAtoms > %d. Reduce this value in the gateware\n",
                    driverName, functionName, MAX_WVF_AMP_TYPES);
            continue;
        }

        /* Select our "fake" channel if we are in single pass mode.
         * This is done so we can the same flow as BPMModeMultiBunch mode,
         * without having to separate the implementations */
        channel = CH_SP;

        /* Convert user channel into hw channel */
        hwAmpChannel = channelMap[channel].HwAmpChannel;
        if(hwAmpChannel < 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid HwAmpChannel channelMap for channel %d\n",
                    driverName, functionName, hwAmpChannel);
            continue;
        }

        /* Our waveform will have "num_samples_pres + num_samples_post"
         * samples in each dimension */
        dims[0] = numAtoms;
        dims[1] = (num_samples_pre + num_samples_post)*num_shots;

        /* We can't acquire something with 0 points */
        if (dims[1] == 0) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid number of points for acquisition (= 0)\n",
                    driverName, functionName);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrTooFewPoints);
            callParamCallbacks(coreID);
            continue;
        }

        /* dims[1] must not exceed bpmMaxPoints, as we use this to alloc
         * points for the Waveform Plugins */
        if (dims[1] > bpmMaxPoints) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: invalid number of points for acquisition (> %d)\n",
                    driverName, functionName, bpmMaxPoints);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrTooManyPoints);
            callParamCallbacks(coreID);
            continue;
        }

        status = getAcqNDArrayType(coreID, hwAmpChannel, atomWidth, &NDType);
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to determine NDArray type for acquisition, coreID = %d\n",
                    driverName, functionName, coreID);
            continue;
        }

        pArrayAllChannels = pNDArrayPool->alloc(MAX_WVF_DIMS, dims, NDType, 0, NULL);
        if (pArrayAllChannels == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayAllChannels\n",
                driverName, functionName);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrNoMem);
            callParamCallbacks(coreID);
            continue;
        }

        /* Tell we are acquiring just before we actually start it */
        setIntegerParam(coreID, P_BPMStatus, BPMStatusAcquire);
        callParamCallbacks(coreID);

        getUIntDigitalParam(coreID, P_TriggerDataThres, &TriggerDataThres,    0xFFFFFFFF);
        getUIntDigitalParam(coreID, P_TriggerDataPol,   &TriggerDataPol,      0xFFFFFFFF);
        getUIntDigitalParam(coreID, P_TriggerDataSel,   &TriggerDataSel,      0xFFFFFFFF);
        getUIntDigitalParam(coreID, P_TriggerDataFilt,  &TriggerDataFilt,     0xFFFFFFFF);
        getUIntDigitalParam(coreID, P_TriggerHwDly,     &TriggerHwDly,        0xFFFFFFFF);
        getUIntDigitalParam(coreID, P_DataTrigChan,     &DataTrigChan,        0xFFFFFFFF);

        /* Setup single-pass parameters */
        /* Kx, Ky, etc are epicsUInt32 type which are perfectly representable
         * on a double (52-bit mantissa). So, we makle an explicitly cast to
         * avoid warning of the type: warning: narrowing conversion of ...
         * This just comes from the fact that in a particular architecture,
         * epicsUInt32 is unsigned int, but epics types are portable */
        /* We use K = 1 and Offsets = 0, as we apply those number at the record
         * level to give more flexibility. Later we can convert the SP calculation
         * to the IOC level, as well */
        bpm_parameters_t bpm_parameters = {.kx       = 1.0,
                                           .ky       = 1.0,
                                           .kq       = 1.0,
                                           .ksum     = 1.0,
                                           .offset_x = 0.0,
                                           .offset_y = 0.0,
                                           .offset_q = 0.0
                                           };

        /* Get correct service name*/
        status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
        if (status) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
            continue;
        }

        status = getFullServiceName (this->bpmNumber, coreID, "INIT", service_board, 
            sizeof(service_board));
        if (status) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName for INIT service, status=%d\n",
                driverName, functionName, status);
            continue;
        }

        /* Reconfigure acquisition */
        status = epicsEventTryWait(this->reconfSPassAcqEventId[coreID]);
        if (status == epicsEventWaitOK) {
            if (bpm_single_pass != NULL) {
                bpm_single_pass_destroy (&bpm_single_pass);
            }
        }

        if (bpm_single_pass == NULL) {
            bpm_single_pass = bpm_single_pass_new (this->endpoint,
                    this->verbose, NULL, service, service_board,
                    &bpm_parameters, num_samples_pre, num_samples_post,
                    num_shots);
            if (bpm_single_pass == NULL) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: bpm_single_pass could not be created, status=%d\n",
                        driverName, functionName, status);
                continue;
            }
        }

        bpm_single_pass_configure_trigger (bpm_single_pass, TriggerDataFilt,
            TriggerDataPol, TriggerHwDly);
        if (trigger == ACQ_CLIENT_TRIG_DATA_DRIVEN) {
            bpm_single_pass_configure_data_trigger (bpm_single_pass,
                TriggerDataThres, TriggerDataSel);
        }
        else if (trigger == ACQ_CLIENT_TRIG_EXTERNAL) {
            bpm_single_pass_configure_external_trigger (bpm_single_pass);
        }
        else {
        /* Invalid trigger for SP*/
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Invalid trigger for Single Pass\n",
                driverName, functionName);
            setIntegerParam(coreID, P_BPMStatus, BPMStatusErrAcq);
            callParamCallbacks(coreID);
            continue;
        }

        /* Do acquisition until a stop event arrives */
        while (1) {
            /* If we were interrupted, jut go back to waiting the start event */
            if (interrupted) {
                break;
            }

            /* Waveform statistics */
            epicsTimeGetCurrent(&now);
            getIntegerParam(NDArrayCounter, &arrayCounter);
            arrayCounter++;
            setIntegerParam(NDArrayCounter, arrayCounter);
            pArrayAllChannels->uniqueId = arrayCounter;
            timeStamp = now.secPastEpoch + now.nsec / 1.e9;
            pArrayAllChannels->timeStamp = timeStamp;
            pArrayAllChannels->epicsTS.secPastEpoch = now.secPastEpoch;
            pArrayAllChannels->epicsTS.nsec = now.nsec;
            getAttributes(pArrayAllChannels->pAttributeList);

            unlock();
            pasynManager->lockPort(pasynUser);
            status = startSPAcq(bpm_single_pass);
            pasynManager->unlockPort(pasynUser);
            lock();

            if (status == asynSuccess) {
                /* FIXME: Improve BPMStatus trigger waiting. The information
                 * about waiting for trigger is not totally accurate here.
                 * Although, we will for SW or HW trigger in a short time,
                 * we are not actually there yet ...
                 */
                if (trigger == ACQ_CLIENT_TRIG_EXTERNAL) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerHwExtWaiting);
                }
                else if (trigger == ACQ_CLIENT_TRIG_DATA_DRIVEN) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerHwDataWaiting);
                }
                else if (trigger == ACQ_CLIENT_TRIG_SOFTWARE) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusTriggerSwWaiting);
                }

                callParamCallbacks(coreID);
            }
            else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: unable to acquire waveform\n",
                        driverName, functionName);
                /* Could not start acquisition. Invalid parameters */
                setIntegerParam(coreID, P_BPMStatus, BPMStatusErrAcq);
                callParamCallbacks(coreID);
                break;
            }

            /* Wait for acquisition to complete, but allow acquire stop events to be handled */
            while (1) {
                unlock();
                status = epicsEventWaitWithTimeout(this->abortAcqEventId[BPMModeSinglePass][coreID], pollTime);
                lock();
                if (status == epicsEventWaitOK) {
                    /* We got a stop event, abort acquisition */
                    abortAcq(coreID);
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusAborted);
                    callParamCallbacks(coreID);
                    interrupted = 1;
                    break;
                }
                else {
                    acqCompleted = checkSPAcqCompletion (bpm_single_pass);
                }

                if (acqCompleted == -1) {
                    setIntegerParam(coreID, P_BPMStatus, BPMStatusErrAcqOFlow);
                    callParamCallbacks(coreID);
                    break;
                }

                if (acqCompleted == 1) {
                    /* Get curve */
                    getAcqSPSamples(bpm_single_pass, &bpm_sample);
                    break;
                }
            }

            /* Only do callbacks and calculate position if we could acquire some
             * data */
            if (acqCompleted == 1) {
                /* Get SinglePass Raw Data for the user */
                getAcqSPCurve(bpm_single_pass, pArrayAllChannels);
                /* Do callbacks on the full waveform (all channels interleaved) */
                doCallbacksGenericPointer(pArrayAllChannels, NDArrayData,
                        channelMap[channel].NDArrayAmp[coreID][WVF_AMP_ALL]);

                /* Copy AMP data to arrays for each type of data, do callbacks on that */
                status = deinterleaveNDArray(pArrayAllChannels, channelMap[channel].NDArrayAmp[coreID],
                        dims[0], arrayCounter, &now);
                if (status != asynSuccess) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                            "%s:%s: unable to deinterleave NDArray\n",
                            driverName, functionName);
                    continue;
                }

                /* Set SinglePass AMP/POS Calculate positions and call callbacks */
                setDoubleParam(P_SPAmpA,       bpm_sample.a);
                setDoubleParam(P_SPAmpB,       bpm_sample.b);
                setDoubleParam(P_SPAmpC,       bpm_sample.c);
                setDoubleParam(P_SPAmpD,       bpm_sample.d);
                setDoubleParam(P_SPPosX,       bpm_sample.x);
                setDoubleParam(P_SPPosY,       bpm_sample.y);
                setDoubleParam(P_SPPosQ,       bpm_sample.q);
                setDoubleParam(P_SPPosSum,     bpm_sample.sum);
            }

            callParamCallbacks(coreID);
        }

        /* Release buffers */
        pArrayAllChannels->release();
        pArrayAllChannels = NULL;
    }

    if (bpm_single_pass != NULL) {
        bpm_single_pass_destroy (&bpm_single_pass);
    }
}

void drvBPM::acqMonitTask()
{
    asynStatus status = asynSuccess;
    int err = HALCS_CLIENT_SUCCESS;
    size_t dims[MAX_WVF_DIMS];
    epicsUInt32 mask = 0xFFFFFFFF;
    ABCD_ROW abcdRow;
    XYQS_ROW xyqsRow;
    XYQS_ROW xyqsFakeRow;
    NDArray *pArrayMonitData[MAX_MONIT_DATA];
    double monitData[MAX_MONIT_DATA];
    NDDataType_t NDType = NDFloat64;
    int NDArrayAddrInit = WVF_MONIT_AMP_A;
    epicsTimeStamp now;
    int monitEnable = 0;
    static const char *functionName = "acqMonitTask";
    char service[SERVICE_NAME_SIZE];

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, 0, "DSP", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    /* Format channel name */
    strncat (service, ":DATA_PRODUCER", sizeof(service));

    dims[0] = 1;
    for (int i = 0; i < MAX_MONIT_DATA; ++i) {
        pArrayMonitData[i] = pNDArrayPool->alloc(1, dims, NDType, 0, 0);
        if (pArrayMonitData == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayMonitData\n",
                driverName, functionName);
            status = asynError;
            goto alloc_ndarray_err;
        }
    }

    err = halcs_set_monit_subscription (bpmClientMonit, service, "MONIT_AMP");
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto set_monit_subscription_err;
    }

    smio_dsp_monit_data_t monit_data;
    while (1) {

        getIntegerParam(P_MonitEnable, &monitEnable);
        if (!monitEnable) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for monitEnable =true\n", driverName, functionName);
            /* remove subscription to avoid burst of old data when enabling */
            halcs_remove_monit_subscription (bpmClientMonit, service);
            epicsEventWait(activeMonitEnableEventId);
            halcs_set_monit_subscription (bpmClientMonit, service, "MONIT_AMP");
        }

        err = halcs_get_monit_stream (bpmClientMonit, "MONIT_AMP", &monit_data);
        if(err == HALCS_CLIENT_SUCCESS) {
            /* Prepare for position calculation */
            abcdRow.A = monit_data.amp_ch0;
            abcdRow.B = monit_data.amp_ch1;
            abcdRow.C = monit_data.amp_ch2;
            abcdRow.D = monit_data.amp_ch3;

            ABCDtoXYQS(&abcdRow, &xyqsRow, 1, true);
            ABCDtoXYQS(&abcdRow, &xyqsFakeRow, 1, false);

            monitData[0] = monit_data.amp_ch0;
            monitData[1] = monit_data.amp_ch1;
            monitData[2] = monit_data.amp_ch2;
            monitData[3] = monit_data.amp_ch3;
            monitData[4] = xyqsRow.X;
            monitData[5] = xyqsRow.Y;
            monitData[6] = xyqsRow.Q;
            monitData[7] = xyqsRow.S;
            monitData[8] = xyqsFakeRow.X;
            monitData[9] = xyqsFakeRow.Y;

            epicsTimeGetCurrent(&now);

            for (int i = 0; i < MAX_MONIT_DATA; ++i) {
                /* NDArray atributtes */
                pArrayMonitData[i]->timeStamp = now.secPastEpoch + now.nsec / 1.e9;
                pArrayMonitData[i]->epicsTS.secPastEpoch = now.secPastEpoch;
                pArrayMonitData[i]->epicsTS.nsec = now.nsec;
                getAttributes(pArrayMonitData[i]->pAttributeList);
                /* NDArray data */
                *((epicsFloat64 *)pArrayMonitData[i]->pData) = monitData[i];
                doCallbacksGenericPointer(pArrayMonitData[i], NDArrayData, NDArrayAddrInit+i);
            }
        }
        else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Could not get Monit. AMP/POS data. Status = %d\n",
                    driverName, functionName, err);
        }
    }

set_monit_subscription_err:

alloc_ndarray_err:
    for (int i = 0; i < MAX_MONIT_DATA; ++i) {
        pArrayMonitData[i]->release();
    }
get_service_err:
    return;
}

asynStatus drvBPM::deinterleaveNDArray (NDArray *pArrayAllChannels, const int *pNDArrayAddr,
        int pNDArrayAddrSize, int arrayCounter, epicsTimeStamp *timeStamp)
{
    int status = asynSuccess;
    size_t dims[MAX_WVF_DIMS];
    NDArrayInfo_t arrayInfo;
    NDDataType_t NDType;
    NDDataType_t NDTypeSingle;
    size_t arrayYStride = 0;
    NDArray *pArraySingleChannel;
    epicsFloat64 *pInFloat64;
    epicsFloat64 *pOutFloat64;
    epicsInt32 *pIn32;
    epicsInt32 *pOut32;
    epicsInt16 *pIn16;
    epicsInt8 *pIn8;
    int channelAddr;
    static const char *functionName = "deinterleaveNDArray";

    status = pArrayAllChannels->getInfo(&arrayInfo);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to get information about pArrayAllChannels\n",
                driverName, functionName);
        status = asynError;
        goto get_info_array_err;
    }

    arrayYStride = arrayInfo.yStride;
    dims[0] = arrayInfo.ySize;
    NDType = pArrayAllChannels->dataType;
    NDTypeSingle = NDType;
    /* Convert all integer types to 32-bit */
    if (NDTypeSingle == NDInt8 || NDTypeSingle == NDInt16) {
        NDTypeSingle = NDInt32;
    }

    for (int i = 0; i < pNDArrayAddrSize; ++i) {
        channelAddr = pNDArrayAddr[i];
        pArraySingleChannel = pNDArrayPool->alloc(1, dims, NDTypeSingle, 0, 0);
        if (pArraySingleChannel == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArraySingleChannel\n",
                driverName, functionName);
            status = asynError;
            goto alloc_ndarray_err;
        }

        pArraySingleChannel->uniqueId = arrayCounter;
        pArraySingleChannel->timeStamp = timeStamp->secPastEpoch + timeStamp->nsec / 1.e9;
        pArraySingleChannel->epicsTS.secPastEpoch = timeStamp->secPastEpoch;
        pArraySingleChannel->epicsTS.nsec = timeStamp->nsec;
        getAttributes(pArraySingleChannel->pAttributeList);

        pIn8 = (epicsInt8 *)pArrayAllChannels->pData;
        pIn16 = (epicsInt16 *)pArrayAllChannels->pData;
        pIn32 = (epicsInt32 *)pArrayAllChannels->pData;
        pOut32 = (epicsInt32 *)pArraySingleChannel->pData;
        pInFloat64 = (epicsFloat64 *)pArrayAllChannels->pData;
        pOutFloat64 = (epicsFloat64 *)pArraySingleChannel->pData;

        /* Get only a single channel samples from a multi-channel
         * array */
        switch (NDType) {
            case NDInt8:
                for (size_t j = 0; j < dims[0]; ++j) {
                    pOut32[j] = pIn8[i];
                    pIn8 += arrayYStride;
                }
                break;

            case NDInt16:
                for (size_t j = 0; j < dims[0]; ++j) {
                    pOut32[j] = pIn16[i];
                    pIn16 += arrayYStride;
                }
                break;

            case NDInt32:
                for (size_t j = 0; j < dims[0]; ++j) {
                    pOut32[j] = pIn32[i];
                    pIn32 += arrayYStride;
                }
                break;

            case NDFloat64:
                for (size_t j = 0; j < dims[0]; ++j) {
                    pOutFloat64[j] = pInFloat64[i];
                    pInFloat64 += arrayYStride;
                }
                break;

            default:
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unsupported NDType of type: %d\n",
                    driverName, functionName, NDType);
                status = asynError;
                goto unsup_ndtype_err;
        }

        doCallbacksGenericPointer(pArraySingleChannel, NDArrayData,
                channelAddr);
        pArraySingleChannel->release();
    }

    return (asynStatus)status;

unsup_ndtype_err:
alloc_ndarray_err:
get_info_array_err:
    return (asynStatus)status;
}

/** This function computes the sums, diffs and positions
  * \param[in] NDArray of amplitudes interleaved (A1, B1, C1, D1,
  * A2, B2, C2, D2, ...)
  */
asynStatus drvBPM::computePositions(int coreID, NDArray *pArrayAllChannels, int channel,
        epicsTimeStamp *timeStamp)
{
    int status = asynSuccess;
    epicsUInt32 mask = 0xFFFFFFFF;
    NDArrayInfo_t arrayInfo;
    size_t arrayElements = 0;
    size_t arrayYStride = 0;
    size_t arraySingleElements = 0;
    NDDataType_t NDType;
    ABCD_ROW *abcdRow;
    XYQS_ROW *xyqsRow;
    NDArray *pArrayPosAllChannels = NULL;
    int arrayCounter;
    size_t dims[MAX_WVF_DIMS];
    static const char *functionName = "computePositions";

    /* Check if we need to compute position for this channel */
    if (channelMap[channel].CalcPos != 1) {
        status = asynSuccess;
        goto no_calc_pos;
    }

    /* Get NDArray info */
    status = pArrayAllChannels->getInfo(&arrayInfo);
    if (status != 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to get information about pArrayAllChannels\n",
                driverName, functionName);
        status = asynError;
        goto get_arrayinfo_err;
    }

    arrayElements = arrayInfo.nElements;
    arrayYStride = arrayInfo.yStride;
    arraySingleElements = arrayElements/arrayYStride;

    /* Waveform statistics */
    getIntegerParam(NDArrayCounter, &arrayCounter);
    arrayCounter++;
    setIntegerParam(NDArrayCounter, arrayCounter);

    /* Alloc destination array */
    dims[0] = arrayInfo.xSize;
    dims[1] = arrayInfo.ySize;
    NDType = NDFloat64;
    pArrayPosAllChannels = pNDArrayPool->alloc(MAX_WVF_DIMS, dims, NDType, 0, NULL);
    if (pArrayPosAllChannels == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayPosAllChannels\n",
                driverName, functionName);
        status = asynError;
        goto array_pool_copy_err;
    }
    pArrayPosAllChannels->uniqueId = arrayCounter;
    pArrayPosAllChannels->timeStamp = timeStamp->secPastEpoch + timeStamp->nsec / 1.e9;
    pArrayPosAllChannels->epicsTS.secPastEpoch = timeStamp->secPastEpoch;
    pArrayPosAllChannels->epicsTS.nsec = timeStamp->nsec;
    getAttributes(pArrayPosAllChannels->pAttributeList);

    /* FIXME: we must be sure that we are dealing with 32-bit data here and
     * sometimes we are not (ADC channels), organized in interleaved from
     * like: A0, B0, C0, D0, A1, B1, C1, D1 */
    NDType = pArrayAllChannels->dataType;
    if (NDType != NDInt32) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to calculate positions for data different than NDInt32\n",
                driverName, functionName);
        status = asynError;
        goto inv_ndtype_err;
    }

    abcdRow = (ABCD_ROW *) pArrayAllChannels->pData;
    xyqsRow = (XYQS_ROW *) pArrayPosAllChannels->pData;

    /* Compute position for all array positions */
    ABCDtoXYQS(abcdRow, xyqsRow, arraySingleElements, true);

    /* Do callbacks on the full waveform (all channels interleaved) */
    doCallbacksGenericPointer(pArrayPosAllChannels, NDArrayData,
            channelMap[channel].NDArrayPos[coreID][WVF_POS_ALL]);

    /* Copy data to arrays for each type of data, do callbacks on that */
    status = deinterleaveNDArray(pArrayPosAllChannels, channelMap[channel].NDArrayPos[coreID],
            MAX_WVF_POS_SINGLE, arrayCounter, timeStamp);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to deinterleaveNDArray\n",
                driverName, functionName);
        goto deinterleave_ndarray_err;
    }

    pArrayPosAllChannels->release();
    return (asynStatus)status;

deinterleave_ndarray_err:
inv_ndtype_err:
    /* Release array */
    pArrayPosAllChannels->release();
array_pool_copy_err:
get_arrayinfo_err:
no_calc_pos:
    return (asynStatus)status;
}

asynStatus drvBPM::setAcqEvent(epicsUInt32 mask, int addr)
{
    asynStatus status = asynSuccess;
    const char* functionName = "setAcqEvent";
    epicsUInt32 triggerEvent = 0;
    epicsUInt32 triggerType = 0;
    epicsUInt32 triggerRep = 0;
    epicsUInt32 hwAmpChannel = 0;
    int channel = 0;
    int bpmMode = 0;
    int bpmModeOther = 0;
    channelProp_t channelProp;

    /* Get the parameter in the parameter library. */
    getUIntDigitalParam(addr, P_TriggerEvent, &triggerEvent, 0xFFFFFFFF);
    getUIntDigitalParam(addr, P_Trigger, &triggerType, 0xFFFFFFFF);
    getUIntDigitalParam(addr, P_TriggerRep, &triggerRep, 0xFFFFFFFF);
    getIntegerParam(addr, P_BPMMode, &bpmMode);
    getIntegerParam(addr, P_Channel, &channel);

    /* Convert user channel into hw channel */
    hwAmpChannel = channelMap[channel].HwAmpChannel;
    if(hwAmpChannel < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: invalid HwAmpChannel channelMap for channel %d\n",
                driverName, functionName, hwAmpChannel);
        status = asynError;
        goto halcs_inv_channel;
    }

    /* Get channel properties */
    status = getChannelProperties(addr, hwAmpChannel, &channelProp);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getChannelProperties, status=%d\n",
            driverName, functionName, status);
        goto get_chan_prop_err;
    }

    setUIntDigitalParam(addr, P_ChannelSampleSize, channelProp.sampleSize, 0xFFFFFFFF);
    setUIntDigitalParam(addr, P_ChannelNumAtoms, channelProp.numAtoms, 0xFFFFFFFF);
    setUIntDigitalParam(addr, P_ChannelAtomWidth, channelProp.atomWidth, 0xFFFFFFFF);

    /* Get the other acquisition task mode */
    if (bpmMode == BPMModeSinglePass) {
        bpmModeOther = BPMModeMultiBunch;
    }
    else {
        bpmModeOther = BPMModeSinglePass;
    }

    switch (triggerEvent) {
        case TRIG_ACQ_START:
            /* Abort the other acquisition task if needed */
            stopAcqTask(addr, bpmModeOther);
            abortAcqTask(addr, bpmModeOther, false);

            /* Don't try to change anything is we are still acquiring.
             * We must stop r abort the acquisition first */
            if (readingActive[bpmMode][addr]) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Not starting acquistion as acqTask is still active\n",
                    driverName, functionName);
                break;
            }

            if (triggerRep) {
                repetitiveTrigger[bpmMode][addr] = 1;
            }
            else {
                repetitiveTrigger[bpmMode][addr] = 0;
            }

            status = setAcqTrig(addr, (acq_client_trig_e) triggerType);
            if (status) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error calling setAcqTrig, status=%d\n",
                    driverName, functionName, status);
                goto get_set_acq_trig_err;
            }

            /* Send event telling the current task to proceed */
            epicsEventSignal(activeAcqEventId[bpmMode][addr]);
            /* Signal acq thread to start acquisition with the current parameters */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: trigger TRIG_ACQ_START called\n",
                    driverName, functionName);
            epicsEventSignal(startAcqEventId[bpmMode][addr]);
            break;

        /* Stop acquisition if we are in repetitive mode and if we are currently
         * acquiring. Otherwise, we don't need to do anything, as the acquisition
         * task will stop after the current acquisition */
        case TRIG_ACQ_STOP: /* Trigger == Stop */
            stopAcqTask(addr, bpmMode);
            break;

        /* Send the abort event if we are reading (repetitive or regular).
         *  If we want to stop a repetitive trigger, we must send a stop
         *  event */
        case TRIG_ACQ_ABORT: /* Trigger == Abort */
            /* abort the other acquisition task if needed */
            abortAcqTask(addr, bpmMode, true);
            break;

        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: invalid trigger event\n",
                    driverName, functionName);
            status = asynError;
            goto trig_unimplemented_err;
    }

trig_unimplemented_err:
get_set_acq_trig_err:
get_chan_prop_err:
halcs_inv_channel:
    return status;
}

asynStatus drvBPM::abortAcqTask(int addr, int bpmMode, bool abortAcqHw)
{
    asynStatus status = asynSuccess;
    const char* functionName = "abortAcqTask";

    /* we are waiting for a trigger */
    if (readingActive[bpmMode][addr]) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: trigger ACQ_ABORT called for acqTask = %d, coreID = %d\n",
                driverName, functionName, bpmMode, addr);
        epicsEventSignal(this->abortAcqEventId[bpmMode][addr]);
    }
    else {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: trigger ACQ_ABORT but with acquisition in progress, "
                "called for acqTask = %d, coreID = %d\n",
                driverName, functionName, bpmMode, addr);
        /* If we are not actively waiting for an event on acqTask,
         * abort the acquisition anyway, as we might have something
         * going on inside the FPGA from a previous acquisition */
        if (abortAcqHw) {
            abortAcqFromPortThread(addr);
        }
    }

    return status;
}

asynStatus drvBPM::stopAcqTask(int addr, int bpmMode)
{
    asynStatus status = asynSuccess;
    const char* functionName = "stopAcqTask";

    /* We are in repetitive mode */
    if (readingActive[bpmMode][addr]) {
        repetitiveTrigger[bpmMode][addr] = 0;
        /* Send the stop event */
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: trigger ACQ_STOP called for acqTask = %d, coreID = %d\n",
                driverName, functionName, bpmMode, addr);
        epicsEventSignal(this->stopAcqEventId[bpmMode][addr]);
    }

    return status;
}

asynStatus drvBPM::startAcq(int coreID, int hwChannel, epicsUInt32 num_samples_pre,
        epicsUInt32 num_samples_post, epicsUInt32 num_shots)
{
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "startAcq";
    char service[SERVICE_NAME_SIZE];
    acq_trans_t acq_trans;
    acq_req_t req;
    acq_block_t block;

    if (num_samples_pre + num_samples_post > MAX_ARRAY_POINTS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to select %u pre-trigger samples and\n"
                "%u post-trigger samples for acquisition\n",
                driverName, functionName, num_samples_pre, num_samples_post);
        status = asynError;
        goto halcs_samples_sel_err;
    }

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    req.num_samples_pre  = num_samples_pre;
    req.num_samples_post = num_samples_post;
    req.num_shots = num_shots;
    req.chan = (uint32_t) hwChannel;

    /* Fill BPM acquisition transaction structure */
    acq_trans = {req, block};

#ifdef TEST_SYNTH_DATA
    double t[TEST_LENGTH];
    for (int i = 0; i < TEST_LENGTH*4; ++i) {
        t[i] = (1/FREQ_SAMPLE)*i;
        ((int16_t *)pArrayAllChannels->pData)[i] = sin(2*PI*FREQ*t[i])*(1<<15);
    }
#else
    err = acq_start (bpmClientAcq[coreID], service, &req);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to acquire waveform on hwChannel %d, with %u\n"
                "\tpre-trigger samples and %u post-trigger samples\n",
                driverName, functionName, hwChannel, num_samples_pre,
                num_samples_post);
        status = asynError;
        goto halcs_acq_err;
    }
#endif

halcs_acq_err:
get_service_err:
halcs_samples_sel_err:
    return status;
}

asynStatus drvBPM::startSPAcq(bpm_single_pass_t *bpm_single_pass)
{
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "startSPAcq";

    err = bpm_single_pass_start (bpm_single_pass);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
    }
    else {
        status = asynSuccess;
    }

    return status;
}

asynStatus drvBPM::abortAcqRaw(int coreID, acq_client_t *acq_client)
{
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "abortAcq";
    char service[SERVICE_NAME_SIZE];
    uint32_t fsm_stop = 1;

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    err = acq_set_fsm_stop (acq_client, service, fsm_stop);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto halcs_acq_stop_err;
    }

halcs_acq_stop_err:
get_service_err:
    return status;
}

/* This must be called only acquisition threads */
asynStatus drvBPM::abortAcq(int coreID)
{
    return abortAcqRaw(coreID, bpmClientAcq[coreID]);
}

/* This must be called only from asyn PortThread*/
asynStatus drvBPM::abortAcqFromPortThread(int coreID)
{
    return abortAcqRaw(coreID, bpmClientAcqParam[coreID]);
}

int drvBPM::checkAcqCompletion(int coreID)
{
    int complete = 0;
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "checkAcqCompletion";
    char service[SERVICE_NAME_SIZE];

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    err = acq_check (bpmClientAcq[coreID], service);
    /* FIFO Full error */
    if (err == HALCS_CLIENT_ERR_MODULE_ERR8) {
        complete = -1;
        goto halcs_acq_not_finished;
    }

    if (err != HALCS_CLIENT_SUCCESS) {
        complete = 0;
        goto halcs_acq_not_finished;
    }

    complete = 1;

halcs_acq_not_finished:
get_service_err:
    return complete;
}

int drvBPM::checkSPAcqCompletion(bpm_single_pass_t *bpm_single_pass)
{
    int complete = 0;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "checkSPAcqCompletion";

    err = bpm_single_pass_check (bpm_single_pass);
    /* FIFO Full error */
    if (err == HALCS_CLIENT_ERR_MODULE_ERR8) {
        complete = -1;
        goto halcs_acq_not_finished;
    }

    if (err != HALCS_CLIENT_SUCCESS) {
        complete = 0;
        goto halcs_acq_not_finished;
    }

    complete = 1;

halcs_acq_not_finished:
    return complete;
}

asynStatus drvBPM::getAcqCurve(int coreID, NDArray *pArrayAllChannels, int hwChannel,
        epicsUInt32 num_samples_pre, epicsUInt32 num_samples_post,
        epicsUInt32 num_shots)
{
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "getAcqCurve";
    char service[SERVICE_NAME_SIZE];
    acq_trans_t acq_trans;
    acq_req_t req;
    acq_block_t block;

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    req.num_samples_pre  = num_samples_pre;
    req.num_samples_post = num_samples_post;
    req.num_shots = num_shots;
    req.chan = (uint32_t) hwChannel;
    block.idx = 0;
    block.data = (uint32_t *)pArrayAllChannels->pData;
    block.data_size = (uint32_t)pArrayAllChannels->dataSize;
    block.bytes_read = 0;

    /* Fill BPM acquisition transaction structure */
    acq_trans = {req, block};

    /* This just reads the data from memory */
    err = acq_get_curve (bpmClientAcq[coreID], service, &acq_trans);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to read waveform on hwChannel %d, with %u\n"
                "\tpre-trigger samples and %u post-trigger samples\n",
                driverName, functionName, hwChannel, num_samples_pre,
                num_samples_post);
        status = asynError;
        goto halcs_acq_err;
    }

halcs_acq_err:
get_service_err:
    return status;
}

asynStatus drvBPM::getAcqSPCurve(bpm_single_pass_t *bpm_single_pass, NDArray *pArrayAllChannels)
{
    asynStatus status = asynSuccess;
    const char* functionName = "getAcqSPCurve";

    /* Copy data to NDArray */
    const acq_trans_t * acq_trans = bpm_single_pass_get_acq_transaction (bpm_single_pass);
    memcpy (pArrayAllChannels->pData, acq_trans->block.data, acq_trans->block.data_size);
    pArrayAllChannels->dataSize = acq_trans->block.data_size;

    return status;
}

asynStatus drvBPM::getAcqSPSamples(bpm_single_pass_t *bpm_single_pass, bpm_sample_t *bpm_sample)
{
    asynStatus status = asynSuccess;
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "getAcqSPSamples";

    err = bpm_single_pass_sample (bpm_single_pass, bpm_sample);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to read Single Pass samples\n",
                driverName, functionName);
        status = asynError;
        goto halcs_acq_err;
    }

halcs_acq_err:
    return status;
}

asynStatus drvBPM::getAcqNDArrayType(int coreID, int hwChannel, epicsUInt32 atomWidth, NDDataType_t *NDType)
{
    asynStatus status = asynSuccess;
    static const char *functionName = "getAcqNDArrayType";

    /* Determine minimum data size */
    switch (atomWidth) {
        case 2: /* bytes */
            *NDType = NDInt16;
            break;
        case 4: /* bytes */
            *NDType = NDInt32;
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to determine NDType for acquisition with atomWidth = %u\n",
                    driverName, functionName, atomWidth);
            status = asynError;
            goto get_ndarray_type_err;
    }

get_ndarray_type_err:
    return status;
}

asynStatus drvBPM::getChannelProperties(int coreID, int channel, channelProp_t *channelProp)
{
    asynStatus status = asynSuccess;
    int err = HALCS_CLIENT_SUCCESS;
    const char* functionName = "getChannelProperties";
    char service[SERVICE_NAME_SIZE];

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, coreID, "ACQ", service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling getFullServiceName, status=%d\n",
            driverName, functionName, status);
        goto get_service_err;
    }

    err = halcs_get_acq_ch_sample_size (bpmClientAcqParam[coreID], service,
        channel, &channelProp->sampleSize);
    err |= halcs_get_acq_ch_num_atoms (bpmClientAcqParam[coreID], service,
        channel, &channelProp->numAtoms);
    err |= halcs_get_acq_ch_atom_width (bpmClientAcqParam[coreID], service,
        channel, &channelProp->atomWidth);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto halcs_get_sample_size_err;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: channel properties for coreID = %u, channel = %d:\n"
        "\tsampleSize = %u\n"
        "\tnumAtoms = %u\n"
        "\tatomWidth = %u\n",
        driverName, functionName, coreID, channel,
        channelProp->sampleSize, channelProp->numAtoms, channelProp->atomWidth);

halcs_get_sample_size_err:
get_service_err:
    return status;
}

/********************************************************************/
/********************* Asyn overrided methods  **********************/
/********************************************************************/

/*
 * Asyn overrided methods that are called by higher layers
 */

/** Called when asyn clients call pasynUInt32Digital->write().
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus drvBPM::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName = NULL;
    const char* functionName = "writeUInt32Digital";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
        return status;
    }

    if (function >= FIRST_COMMAND) {
        /* Set the parameter in the parameter library. */
        setUIntDigitalParam(addr, function, value, mask);

        /* Some operations need some special handling*/
        if (function == P_TriggerEvent) {
            /* If run was set then wake up the simulation task */
            status = setAcqEvent(mask, addr);
        }
        else if (function == P_AdcClkSel) {
            status = setAdcClkSel(mask, addr);
        }
        else if (function == P_AdcAD9510ClkSel) {
            status = setAdcAD9510ClkSel(mask, addr);
        }
        else if (function == P_AdcAD9510Dflt) {
            status = resetAD9510(mask, addr);
        }
        else if (function == P_DataTrigChan) {
            /* Ah... FIXME: ugly static mapping! */
            status = setDataTrigChan(mask, addr);
        }
        else if (function == P_AdcRegWrite) {
            /* Ah... FIXME: ugly static mapping! */
            status = setAdcReg(mask, addr);
        }
        else if (function == P_AdcRegRead) {
            /* Ah... FIXME: ugly static mapping! */
            status = getAdcReg(NULL, mask, addr);
        }
        /* This bit is self-clearing. So, we use some special treatment */
        else if (function == P_ActiveClkRstADCs) {
            status = resetADCs(mask, addr);
        }
        else {
            int bpmMode = 0;
            getIntegerParam(addr, P_BPMMode, &bpmMode);

            // Theese parameters will trigger a change in Single Pass configuration
            if (bpmMode == BPMModeSinglePass &&
                    (function == P_SamplesPre || function == P_SamplesPost ||
                     function == P_NumShots)) {

                /* Send the reconfig. event */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: trigger SP_ACQ_RECONF called for coreID = %d\n",
                        driverName, functionName, addr);
                epicsEventSignal(this->reconfSPassAcqEventId[addr]);
            }

            /* Do operation on HW. Some functions do not set anything on hardware */
            status = setParam32(function, mask, addr);
            /* Readback all parameters from Hw */
            readUInt32Params(mask, addr);
        }
    }
    else {
        /* Call base class */
#if 0
        status = asynNDArrayDriver::writeUInt32DigitalInt32(pasynUser, value, mask);
#endif
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s, value=%d\n",
                driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynUInt32Digital->read().
 * For all parameters it gets the value in the parameter library..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[out] value Value to read. */
asynStatus drvBPM::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *functionName = "readUInt32Digital";
    const char *paramName = NULL;

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }

    if (function >= FIRST_COMMAND) {
        if (function == P_DataTrigChan) {
            status = getDataTrigChan(value, mask, addr);
        }
        else {
            /* Get parameter, possibly from HW */
            status = getParam32(function, value, mask, addr);
        }
    }
    else {
#if 0
        /* Call base class */
        status = asynNDArrayDriver::readUIn32Digital(pasynUser, value, mask);
#endif
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);

    return status;
}

/** Called when asyn clients call pasynInt32->write().
  * For all parameters it sets the value in the parameter library and calls any
  * registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus drvBPM::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName = NULL;
    const char* functionName = "writeInt32";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
        return status;
    }

    if (function >= FIRST_COMMAND) {
        /* Set the parameter in the parameter library. */
        status = setIntegerParam(addr, function, value);

        if (function == P_MonitEnable) {
            /* Send the start event if the value is 1 */
            if (value) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: Monit enable task event to be send\n",
                        driverName, functionName);
                epicsEventSignal(this->activeMonitEnableEventId);
            }
        }
        else if (function == P_BPMMode) {
            status = setBPMMode(addr, function);
        }
        else {
            /* Do operation on HW. Some functions do not set anything on hardware */
            status = setParamInteger(function, addr);
            /* Readback all parameters from Hw */
            readIntegerParams(addr);
        }
    }
    else {
        /* Call base class */
        status = asynNDArrayDriver::writeInt32(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s, value=%d\n",
                driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynInt32->read().
 * This does nothing for now and just call the base implementation. If needed,
 * add processing before calling the base class implementation
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvBPM::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName = NULL;
    const char* functionName = "readInt32";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }

    if (function >= FIRST_COMMAND) {
        /* Get parameter in library, as some parameters are not written in HW */
        status = getParamInteger(function, value, addr);
    }
    else {
        /* Call base class */
        status = asynNDArrayDriver::readInt32(pasynUser, value);
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvBPM::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName = NULL;
    const char* functionName = "writeFloat64";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%f",
                driverName, functionName, status, function, paramName, value);
        return status;
    }

    if (function >= FIRST_COMMAND) {
        /* Set the parameter in the parameter library. */
        setDoubleParam(addr, function, value);

        /* Some operations need some special handling*/
        if (function == P_AdcSi57xFreq) {
            status = setSi57xFreq(addr);
        }
        else {
            /* Do operation on HW. Some functions do not set anything on hardware */
            status = setParamDouble(function, addr);
            /* Readback all parameters from Hw */
            readFloat64Params(addr);
        }
    }
    else {
        /* Call base class */
        status = asynNDArrayDriver::writeFloat64(pasynUser, value);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks(addr);

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%f",
                driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s, value=%f\n",
                driverName, functionName, function, paramName, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvBPM::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName = NULL;
    const char* functionName = "readFloat64";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);
    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }

    /* Get double param, possibly from HW */
    if (function >= FIRST_COMMAND) {
        status = getParamDouble(function, value, addr);
    }
    else {
        /* Call base class */
        status = asynNDArrayDriver::readFloat64(pasynUser, value);
    }

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                "%s:%s: function=%d, name=%s\n",
                driverName, functionName, function, paramName);
    return status;
}

/********************************************************************/
/************ Function Mapping Overloaded Write functions ***********/
/********************************************************************/

asynStatus drvBPM::doExecuteHwWriteFunction(functionsUInt32Acq_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsUInt32Acq_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    int serviceID = 0;

    /* Get service ID for correct use with acquisition instance */
    status = getServiceID (this->bpmNumber, addr, func.serviceName,
            &serviceID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getServiceID, status=%d\n",
                driverName, functionName, status);
        goto get_service_id_err;
    }

    /* Execute registered function */
    err = func.write(bpmClientAcqParam[serviceID], service, functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, service, functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
get_service_id_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwWriteFunction(functions2UInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functions2UInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    epicsUInt32 param1 = 0;
    epicsUInt32 param2 = 0;

    /* Silently exit if no function is registered */
    if(!func.read) {
        status = asynSuccess;
        goto no_registered_read_func_err;
    }

    /* Read the HW values first as we need to update
       only one of the parameters */
    err = func.read(bpmClient, service, &param1, &param2);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

    /* Determine if we want to change the first or second parameter in HW */
    if (func.parameterPos == 1) {
        err = func.write(bpmClient, service, functionParam.argUInt32, param2);
    }
    else if (func.parameterPos == 2) {
        err = func.write(bpmClient, service, param1, functionParam.argUInt32);
    }

    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, service, functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
halcs_get_func_param_err:
no_registered_read_func_err:
        return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(bpmClient, service, functionParam.argFloat64);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %f\n",
                driverName, functionName, service, functionParam.argFloat64);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwWriteFunction(functionsUInt32Chan_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsUInt32Chan_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    char serviceChanStr[SERVICE_NAME_SIZE];
    int serviceID = 0;
    epicsUInt32 serviceChan = 0;

    /* Create full service name*/
    status = getFullServiceName (this->bpmNumber, addr, func.serviceName,
            serviceChanStr, sizeof(serviceChanStr));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Get correct service channel */
    getServiceChan (this->bpmNumber, addr, func.serviceName, &serviceChan);

    /* Execute registered function */
    err = func.write(bpmClient, serviceChanStr, serviceChan, functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, serviceChanStr,
                functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
get_service_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwWriteFunction(functionsUInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsUInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(bpmClient, service, functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, service, functionParam.argUInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(bpmClient, service, functionParam.argInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing write function for service %s,"
                "param = %u\n",
                driverName, functionName, service, functionParam.argInt32);
        status = asynError;
        goto halcs_set_func_param_err;
    }

halcs_set_func_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::executeHwWriteFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwWriteFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    const char *paramName = NULL;
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = bpmHwFunc.find (functionId);
    if (func == bpmHwFunc.end()) {
        getParamName(functionId, &paramName);
        /* This is not an error. Exit silently */
        status = asynSuccess;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d, name %s\n",
                driverName, functionName, functionId, paramName);
        goto get_reg_func_err;
    }

    /* Get service name from structure */
    funcService = func->second.getServiceName(*this);
    /* Create full service name*/
    status = getFullServiceName (this->bpmNumber, addr, funcService,
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Execute overloaded function for each function type we know of */
    status = func->second.executeHwWrite(*this, service, addr, functionParam);

    if (status == asynSuccess && streq(funcService, "FOFB_CTRL")) {
        /* Force hardware to read and apply parameter change */
        halcs_set_fofb_ctrl_act_part(bpmClient, service, 1);
    }

get_reg_func_err:
get_service_err:
        return (asynStatus)status;
}

/********************************************************************/
/************ Function Mapping Overloaded Read functions ************/
/********************************************************************/

asynStatus drvBPM::doExecuteHwReadFunction(functionsUInt32Acq_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsUInt32Acq_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    int serviceID = 0;

    /* Get service ID for correct use with acquisition instance */
    status = getServiceID (this->bpmNumber, addr, func.serviceName,
            &serviceID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getServiceID, status=%d\n",
                driverName, functionName, status);
        goto get_service_id_err;
    }

    /* Execute registered function */
    err = func.read(bpmClientAcqParam[serviceID], service, &functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
get_service_id_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwReadFunction(functions2UInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functions2UInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;
    epicsUInt32 param1 = 0;
    epicsUInt32 param2 = 0;

    /* Read the HW values first as we need to update
       only one of the parameters */
    err = func.read(bpmClient, service, &param1, &param2);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

    /* Determine if we want to read the first or second parameter */
    if (func.parameterPos == 1) {
        functionParam.argUInt32 = param1;
    }
    else if (func.parameterPos == 2) {
        functionParam.argUInt32 = param2;
    }

halcs_get_func_param_err:
        return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(bpmClient, service, &functionParam.argFloat64);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwReadFunction(functionsUInt32Chan_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsUInt32Chan_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    char serviceChanStr[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    int serviceID = 0;
    epicsUInt32 serviceChan = 0;

    /* Create full service name*/
    status = getFullServiceName (this->bpmNumber, addr, func.serviceName,
            serviceChanStr, sizeof(serviceChanStr));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Get correct service channel */
    getServiceChan (this->bpmNumber, addr, func.serviceName, &serviceChan);

    /* Execute registered function */
    err = func.read(bpmClient, serviceChanStr, serviceChan, &functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, serviceChanStr);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
get_service_err:
    return (asynStatus) status;
}

asynStatus drvBPM::doExecuteHwReadFunction(functionsUInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsUInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(bpmClient, service, &functionParam.argUInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::doExecuteHwReadFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(bpmClient, service, &functionParam.argInt32);
    if (err != HALCS_CLIENT_SUCCESS) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failure executing read function for service %s\n",
                driverName, functionName, service);
        status = asynError;
        goto halcs_get_func_param_err;
    }

halcs_get_func_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::executeHwReadFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwReadFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    const char *paramName = NULL;
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = bpmHwFunc.find (functionId);
    if (func == bpmHwFunc.end()) {
        getParamName(functionId, &paramName);
        /* We use disabled to indicate the function was not found on Hw mapping */
        status = asynDisabled;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d, name %s\n",
                driverName, functionName, functionId, paramName);
        goto get_reg_func_err;
    }

    /* Get service name from structure */
    funcService = func->second.getServiceName(*this);
    /* Create full service name*/
    status = getFullServiceName (this->bpmNumber, addr, funcService,
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Execute overloaded function for each function type we know of */
    status = func->second.executeHwRead(*this, service, addr, functionParam);

get_reg_func_err:
get_service_err:
        return (asynStatus)status;
}

/********************************************************************/
/*************** Generic 32-bit/Double BPM Operations ***************/
/********************************************************************/

/*
 * 32-bit/Double generic BPM operations. These will map to real
 * functions defined in the structures. e.g., functionsUInt32_t
 * and functionsFloat64_t
 */

asynStatus drvBPM::setParamGeneric(int functionId, int addr)
{
    int status = asynSuccess;
    const char *functionName = "setParamGeneric";
    const char *paramName = NULL;
    asynParamType asynType = asynParamNotDefined;

    getParamName(functionId, &paramName);
    status = getParamType(addr, functionId, &asynType);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getParamType failure retrieving asynParamType, "
                "functionId = %d, paramName = %s\n",
                driverName, functionName, functionId, paramName);
        goto get_type_err;
    }

    switch (asynType) {
        case asynParamInt32:
            status = setParamInteger(functionId, addr);
        break;

        case asynParamUInt32Digital:
            status = setParam32(functionId, 0xFFFFFFFF, addr);
        break;

        case asynParamFloat64:
            status = setParamDouble(functionId, addr);
        break;

        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unsupported type for asynParamType: %d, "
                    "functionId = %d, paramName = %s\n",
                    driverName, functionName, asynType,
                    functionId, paramName);
            goto unsup_asyn_type;
    }

    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: setParam32/setParamDouble failure setting value %d, "
                "for functionId = %d, paramName = %s\n",
                driverName, functionName, status, functionId, paramName);
        goto set_type_err;
    }

set_type_err:
unsup_asyn_type:
get_type_err:
    return (asynStatus)status;
}

asynStatus drvBPM::setParam32(int functionId, epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParam32";

    status = getUIntDigitalParam(addr, functionId, &functionArgs.argUInt32, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

    if (functionId >= P_FofbCtrlErrClr && functionId <= P_FofbCtrlRcbData)
        updateUInt32Params(0xFFFFFFFF, addr, functionId, functionId, false);

get_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::getParam32(int functionId, epicsUInt32 *param,
        epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParam32";
    const char *paramName;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(addr, functionId, param, mask);
    if (status != asynSuccess) {
        if (status != asynParamUndefined) {
            getParamName(functionId, &paramName);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: getUIntDigitalParam failure for retrieving parameter %s, status = %d\n",
                    driverName, functionName, paramName, status);
        }
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);
    if (status == asynSuccess) {
        /* Mask parameter according to the received mask */
        functionArgs.argUInt32 &= mask;
        *param = functionArgs.argUInt32;
    }
    /* We recover from asynDisabled just by retrieving
     * the parameter from the list */
    else if (status == asynDisabled){
        status = asynSuccess;
    }

get_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::setParamInteger(int functionId, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParamInteger";

    status = getIntegerParam(addr, functionId, &functionArgs.argInt32);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getIntegerParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

get_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::getParamInteger(int functionId, epicsInt32 *param,
        int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParamInteger";
    const char *paramName;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getIntegerParam(addr, functionId, param);
    if (status != asynSuccess) {
        if (status != asynParamUndefined) {
            getParamName(functionId, &paramName);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: getIntegerParam failure for retrieving parameter %s, status = %d\n",
                    driverName, functionName, paramName, status);
        }
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);
    if (status == asynSuccess) {
        *param = functionArgs.argInt32;
    }
    /* We recover from asynDisabled just by retrieving
     * the parameter from the list */
    else if (status == asynDisabled){
        status = asynSuccess;
    }

get_param_err:
    return (asynStatus)status;
}

asynStatus drvBPM::setParamDouble(int functionId, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParamDouble";

    status = getDoubleParam(addr, functionId, &functionArgs.argFloat64);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Convert floating point value to fixed point for the hw write function */
    if (functionId >= P_AdcCh0SwDir && functionId <= P_AdcCh3SwInv) {
        functionArgs.argUInt32 = float2fixed(functionArgs.argFloat64, adc_gain_fixed_point_pos);
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

    if (functionId >= P_AdcCh0SwDir && functionId <= P_AdcCh3SwInv) {
        updateDoubleParams(addr, functionId, functionId, true);
    }

get_param_err:
    return status;
}

asynStatus drvBPM::getParamDouble(int functionId, epicsFloat64 *param, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParamDouble";
    const char *paramName;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(addr, functionId, param);
    if (status != asynSuccess) {
        if (status != asynParamUndefined) {
            getParamName(functionId, &paramName);
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: getDoubleParam failure for retrieving parameter %s\n",
                    driverName, functionName, paramName);
        }
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);

    /* Convert fixed point from hw to float */
    if (functionId >= P_AdcCh0SwDir && functionId <= P_AdcCh3SwInv) {
        functionArgs.argFloat64 = fixed2float(functionArgs.argUInt32, adc_gain_fixed_point_pos);
    }

    if (status == asynSuccess) {
        *param = functionArgs.argFloat64;
    }
    /* We recover from asynDisabled just by retrieving
     * the parameter from the list */
    else if (status == asynDisabled){
        status = asynSuccess;
    }

get_param_err:
    return status;
}

/********************************************************************/
/*********************** Misc BPM Operations ************************/
/********************************************************************/

/*
 * Miscellaneous functions that don't map easily
 * to our generic handlers get/setParam[32/Double]
 */

asynStatus drvBPM::setBPMMode(int addr, int function)
{
    int status = asynSuccess;
    int bpmMode = 0;
    int bpmModeOther = 0;

    /* Get BPMMode previously set */
    getIntegerParam(addr, P_BPMMode, &bpmMode);

    /* Throw an error if we are acquiring in the other mode
     * and we tried to change BPM mode */
    if (bpmMode == BPMModeSinglePass) {
        bpmModeOther = BPMModeMultiBunch;
    }
    else {
        bpmModeOther = BPMModeSinglePass;
    }

    /* Check if an acquisition is running while trying to change mode */
    if (readingActive[bpmModeOther][addr]) {
        setIntegerParam(addr, function, bpmModeOther);
        status = asynError;
        goto other_acq_acquiring_err;
    }

other_acq_acquiring_err:
    return (asynStatus) status;
}

asynStatus drvBPM::setDataTrigChan(epicsUInt32 mask, int addr)
{
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "setDataTrigChan";
    epicsUInt32 dataTrigChan = 0;
    int hwAmpChannel = 0;
    int serviceID = 0;

    /* Set the parameter in the parameter library. */
    getUIntDigitalParam(addr, P_DataTrigChan, &dataTrigChan, mask);

    /* Convert user channel into hw channel */
    hwAmpChannel = channelMap[dataTrigChan].HwAmpChannel;
    if(hwAmpChannel < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: invalid HwAmpChannel channelMap for channel %d\n",
                driverName, functionName, hwAmpChannel);
        status = asynError;
        goto halcs_inv_channel;
    }

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, addr, "ACQ",
            service, sizeof(service));
    status |= getServiceID (this->bpmNumber, addr, "ACQ", &serviceID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    err = acq_set_data_trig_chan (bpmClientAcqParam[serviceID], service, hwAmpChannel);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto halcs_set_data_trig_chan_err;
    }

halcs_set_data_trig_chan_err:
get_service_err:
halcs_inv_channel:
    return (asynStatus)status;
}

asynStatus drvBPM::setAdcClkSel(epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    const char* functionName = "setAdcClkSel";

    /* Call ClkSel function */
    status = setParam32 (P_AdcClkSel, mask, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error setting AdcClkSel, status=%d\n",
            driverName, functionName, status);
        goto set_adc_clk_sel_err;
    }

    status = resetAdcMMCM(mask, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling resetAdcMMCM, status=%d\n",
            driverName, functionName, status);
        goto reset_adc_mmcm_err;
    }

    /* Read AD9510 and ADCs */
    status = readAD9510AndADCsParams(mask, addr);
    status |= readGenDSPParams(0xFFFFFFFF, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling readAD9510AndADCsParams/readGenDSPParams, status=%d\n",
            driverName, functionName, status);
        goto read_adcs_ad9510_err;
    }

    return (asynStatus)status;

read_adcs_ad9510_err:
reset_adc_mmcm_err:
set_adc_clk_sel_err:
    return (asynStatus)status;
}

asynStatus drvBPM::setAdcAD9510ClkSel(epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    epicsUInt32 ad9510ClkSel = 0;
    const char* functionName = "setAdcAD9510ClkSel";

    /* Call ClkSel function */
    status = setParam32 (P_AdcAD9510ClkSel, mask, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error setting AdcAD9510ClkSel, status=%d\n",
            driverName, functionName, status);
        goto set_adcAD9510_clk_sel_err;
    }

    status = resetAdcMMCM(0xFFFFFFFF, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling resetAdcMMCM, status=%d\n",
            driverName, functionName, status);
        goto reset_adc_mmcm_err;
    }

    /* If we select CLK1, disable Si57x outputs */
    getUIntDigitalParam(addr, P_AdcAD9510ClkSel, &ad9510ClkSel, mask);
    if (ad9510ClkSel == AD9510_ADC_CLK_SEL_1) {
        setUIntDigitalParam(addr, P_ActiveClkSi571Oe, SI57X_DISABLE);
    }
    else if (ad9510ClkSel == AD9510_ADC_CLK_SEL_2) {
        setUIntDigitalParam(addr, P_ActiveClkSi571Oe, SI57X_ENABLE);
    }

    /* Restart AD9510 and ADCs */
    status = readAD9510AndADCsParams(mask, addr);
    status |= readGenDSPParams(0xFFFFFFFF, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling readAD9510AndADCsParams/readGenDSPParams, status=%d\n",
            driverName, functionName, status);
        goto read_adcs_ad9510_err;
    }

    return (asynStatus)status;

read_adcs_ad9510_err:
reset_adc_mmcm_err:
set_adcAD9510_clk_sel_err:
    return (asynStatus)status;
}

asynStatus drvBPM::getDataTrigChan(epicsUInt32 *channel, epicsUInt32 mask, int addr)
{
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    int status = asynSuccess;
    const char* functionName = "getDataTrigChan";
    epicsUInt32 dataTrigChan = 0;
    epicsUInt32 hwAmpChannel = 0;
    int serviceID = 0;

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, addr, "ACQ",
            service, sizeof(service));
    status |= getServiceID (this->bpmNumber, addr, "ACQ", &serviceID);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Clear parameter in case of an error occurs */
    *channel = 0;

    err = acq_get_data_trig_chan (bpmClientAcqParam[serviceID], service, &hwAmpChannel);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto halcs_get_data_trig_chan_err;
    }

    if (hwAmpChannel > CH_HW_END-1) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: invalid HwAmpChannel channelRevMap for channel %d\n",
                driverName, functionName, hwAmpChannel);
        status = asynError;
        goto halcs_inv_hw_channel;
    }

    /* Convert user channel into hw channel */
    dataTrigChan = channelRevMap[hwAmpChannel].epicsChannel;
    if(dataTrigChan < 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: invalid channel channelRevMap for channel %d\n",
                driverName, functionName, dataTrigChan);
        status = asynError;
        goto halcs_inv_channel;
    }

    /* Mask parameter according to the received mask */
    dataTrigChan &= mask;
    *channel = dataTrigChan;

halcs_inv_channel:
halcs_inv_hw_channel:
halcs_get_data_trig_chan_err:
get_service_err:
    return (asynStatus)status;
}

asynStatus drvBPM::setAdcReg(epicsUInt32 mask, int addr)
{
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    asynStatus status = asynSuccess;
    const char* functionName = "setAdcReg";
    epicsUInt32 adcRegData = 0;
    epicsUInt32 adcRegAddr = 0;
    epicsUInt32 adcRegWrite = 0;

    /* Get parameters in the parameter library. */
    getUIntDigitalParam(addr, P_AdcRegWrite, &adcRegWrite, mask);
    getUIntDigitalParam(addr, P_AdcRegWriteData, &adcRegData, mask);
    getUIntDigitalParam(addr, P_AdcRegWriteAddr, &adcRegAddr, mask);

    if (adcRegWrite) {
        /* Get correct service name*/
        status = getFullServiceName (this->bpmNumber, addr, "FMC250M_4CH",
                service, sizeof(service));
        if (status) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error calling getFullServiceName, status=%d\n",
                    driverName, functionName, status);
            goto get_service_err;
        }

        err = halcs_set_reg_adc (bpmClient, service, addr, adcRegAddr, adcRegData);
        if (err != HALCS_CLIENT_SUCCESS) {
            status = asynError;
            goto halcs_set_err;
        }
    }

halcs_set_err:
get_service_err:
    return status;
}

asynStatus drvBPM::getAdcReg(epicsUInt32 *data, epicsUInt32 mask, int addr)
{
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    asynStatus status = asynSuccess;
    const char* functionName = "getAdcReg";
    epicsUInt32 adcRegData = 0;
    epicsUInt32 adcRegAddr = 0;
    epicsUInt32 adcRegRead = 0;

    /* Get parameters */
    getUIntDigitalParam(addr, P_AdcRegRead, &adcRegRead, mask);
    getUIntDigitalParam(addr, P_AdcRegReadAddr, &adcRegAddr, mask);

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, addr, "FMC250M_4CH",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    /* Clear parameter in case of an error occurs */
    if (data) {
        *data = 0;
    }

    if (adcRegRead) {
        err = halcs_get_reg_adc (bpmClient, service, addr, adcRegAddr, &adcRegData);
        if (err != HALCS_CLIENT_SUCCESS) {
            status = asynError;
            goto halcs_get_err;
        }

        /* Mask parameter according to the received mask */
        adcRegData &= mask;
        if (data) {
            *data = adcRegData;
        }

        /* Set parameters in the parameter library. */
        setUIntDigitalParam(addr, P_AdcRegReadData, adcRegData, mask);
    }

halcs_get_err:
get_service_err:
    return status;
}

asynStatus drvBPM::resetAdcMMCM(epicsUInt32 mask, int addr)
{
    int err = HALCS_CLIENT_SUCCESS;
    char service[SERVICE_NAME_SIZE];
    asynStatus status = asynSuccess;
    const char* functionName = "resetAdcMMCM";

    /* Get correct service name*/
    status = getFullServiceName (this->bpmNumber, addr, "FMC_ADC_COMMON",
            service, sizeof(service));
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: error calling getFullServiceName, status=%d\n",
                driverName, functionName, status);
        goto get_service_err;
    }

    err = halcs_set_adc_mmcm_rst (bpmClient, service, 1);
    err |= halcs_set_adc_mmcm_rst (bpmClient, service, 0);
    if (err != HALCS_CLIENT_SUCCESS) {
        status = asynError;
        goto halcs_set_err;
    }

halcs_set_err:
get_service_err:
    return status;
}

asynStatus drvBPM::setSi57xFreq(int addr)
{
    int status = asynSuccess;
    const char* functionName = "setSi57xFreq";

    /* On any frequency change, abort the current acquisitions
     * and restart Post-Mortem */
    for(int i = 0; i < NUM_ACQ_CORES_PER_BPM; ++i) {
       setUIntDigitalParam(addr, P_TriggerEvent, TRIG_ACQ_ABORT, 0xFFFFFFFF);
       status = setAcqEvent(0xFFFFFFFF, addr);
       if (status) {
           asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
               "%s:%s: error calling setAcqEvent, status=%d\n",
               driverName, functionName, status);
           goto abort_acq_err;
       }
    }

    /* Set the Si57x frequency on HW */
    status = setParamDouble (P_AdcSi57xFreq, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling setParamDouble, status=%d\n",
            driverName, functionName, status);
        goto set_si57x_freq_err;
    }

    status = resetAdcMMCM(0xFFFFFFFF, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling resetAdcMMCM, status=%d\n",
            driverName, functionName, status);
        goto reset_adc_mmcm_err;
    }

    /* Read AD9510 and ADCs */
    status = readAD9510AndADCsParams(0xFFFFFFFF, addr);
    status |= readGenDSPParams(0xFFFFFFFF, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling readAD9510AndADCsParams/readDSPParams, status=%d\n",
            driverName, functionName, status);
        goto read_adcs_ad9510_err;
    }

    /* Read Si57x */
    status = readSi57xParams(addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling readSi57xParams, status=%d\n",
            driverName, functionName, status);
        goto read_si57x_err;
    }

    /* Restart Acq cores again */
    for(int i = 0; i < NUM_ACQ_CORES_PER_BPM; ++i) {
       setUIntDigitalParam(addr, P_TriggerEvent, TRIG_ACQ_ABORT, 0xFFFFFFFF);
       status = setAcqEvent(0xFFFFFFFF, addr);
       if (status) {
           asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
               "%s:%s: error calling setAcqEvent, status=%d\n",
               driverName, functionName, status);
           goto abort2_acq_err;
       }
    }

    /* Restart Post-Mortem */
    status = initAcqPM (BPMIDPM);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error initAcqPM, status=%d\n",
            driverName, functionName, status);
        goto init_acq_pm_err;
    }

init_acq_pm_err:
abort2_acq_err:
read_si57x_err:
read_adcs_ad9510_err:
reset_adc_mmcm_err:
set_si57x_freq_err:
abort_acq_err:
    return (asynStatus) status;
}

asynStatus drvBPM::resetAD9510AndADCs(epicsUInt32 mask, int addr)
{
    int status = resetAD9510(mask, addr);
    status |= resetADCs(mask, addr);

    return (asynStatus)status;
}

asynStatus drvBPM::resetAD9510(epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    const char* functionName = "resetAD9510";

    epicsUInt32 adcAD9510Dflt = 0;
    getUIntDigitalParam(addr, P_AdcAD9510Dflt, &adcAD9510Dflt, 0xFFFFFFFF);

    /* Only reset on rising edge of signal */
    if (adcAD9510Dflt != 1) {
        goto reset_ad9510_not_rising_edge;
    }

    /* Restart AD9510 */
    status = setParam32 (P_AdcAD9510Dflt, mask, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error restarting AD9510, status=%d\n",
            driverName, functionName, status);
        goto reset_ad9510_err;
    }
    setUIntDigitalParam(addr, P_AdcAD9510Dflt, 0x0, mask);

    /* In order to update all of the readback values from AD9510,
     * force a change in all of its parameters and then call
     * callbacks */
    readAD9510Params(mask, addr);

    return (asynStatus)status;

reset_ad9510_not_rising_edge:
reset_ad9510_err:
    return (asynStatus)status;
}

asynStatus drvBPM::resetADCs(epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    const char* functionName = "resetADCs";

    epicsUInt32 activeClkRstADCs = 0;
    getUIntDigitalParam(addr, P_ActiveClkRstADCs, &activeClkRstADCs, 0xFFFFFFFF);

    /* Only reset on rising edge of signal */
    if (activeClkRstADCs != 1) {
        goto reset_adcs_not_rising_edge;
    }

    /* Restart and ADCs */
    status |= setParam32 (P_ActiveClkRstADCs, mask, addr);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error restarting ADCs, status=%d\n",
            driverName, functionName, status);
        goto reset_adcs_err;
    }
    /* Set bit to 0 in library as it is a self-clearing bit */
    setUIntDigitalParam(addr, P_ActiveClkRstADCs, 0x0, mask);

    /* In order to update all of the readback values from AD9510,
     * force a change in all of its parameters and then call
     * callbacks */
    readADCsParams (mask, addr);

    return (asynStatus)status;

reset_adcs_not_rising_edge:
reset_adcs_err:
    return (asynStatus)status;
}

asynStatus drvBPM::readAD9510AndADCsParams(epicsUInt32 mask, int addr)
{
    int status = readAD9510Params(mask, addr);
    status |= readADCsParams(mask, addr);

    return (asynStatus)status;
}

asynStatus drvBPM::updateUInt32Params(epicsUInt32 mask, int addr, int firstParam,
        int lastParam, bool acceptErrors)
{
    int status = asynSuccess;
    int errs = 0;
    const char* functionName = "updateUInt32Params";
    epicsUInt32 param = 0;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: updating UInt32 parameters with firstParam = %d, lastParam = %d, "
        "addr = %d\n",
        driverName, functionName, firstParam, lastParam, addr);

    for (int i = firstParam; i < lastParam+1; ++i) {
        status = getParam32(i, &param, mask, addr);
        /* Only write values if there is no error */
        if (status) {
            if (status != asynParamUndefined) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting UInt32 parameter for function = %d, "
                        "addr = %d status = %d\n",
                        driverName, functionName, i, addr, status);
            }
            ++errs;
        }
        else {
            setUIntDigitalParam(addr, i, param, mask);
        }
    }

    if (acceptErrors) {
        return asynSuccess;
    }

    return (errs == 0)? asynSuccess : asynError;
}

asynStatus drvBPM::updateIntegerParams(int addr, int firstParam,
        int lastParam, bool acceptErrors)
{
    int status = asynSuccess;
    int errs = 0;
    const char* functionName = "updateIntegerParams";
    epicsInt32 param = 0;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: updating Int32 parameters with firstParam = %d, lastParam = %d, "
        "addr = %d\n",
        driverName, functionName, firstParam, lastParam, addr);

    for (int i = firstParam; i < lastParam+1; ++i) {
        status = getParamInteger(i, &param, addr);
        /* Only write values if there is no error */
        if (status) {
            if (status != asynParamUndefined) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting Int32 parameter for function = %d, "
                        "addr = %d status = %d\n",
                        driverName, functionName, i, addr, status);
            }
            ++errs;
        }
        else {
            setIntegerParam(addr, i, param);
        }
    }

    if (acceptErrors) {
        return asynSuccess;
    }

    return (errs == 0)? asynSuccess : asynError;
}

asynStatus drvBPM::updateDoubleParams(int addr, int firstParam, int lastParam,
        bool acceptErrors)
{
    int status = asynSuccess;
    int errs = 0;
    const char* functionName = "updateDoubleParams";
    epicsFloat64 param = 0.0;

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: updating Double parameters with firstParam = %d, lastParam = %d, "
        "addr = %d\n",
        driverName, functionName, firstParam, lastParam, addr);

    for (int i = firstParam; i < lastParam+1; ++i) {
        status = getParamDouble(i, &param, addr);
        /* Only write values is there is no error */
        if (status) {
            if (status != asynParamUndefined) {
                asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: error getting Double parameter for function = %d, "
                        "addr = %d status = %d\n",
                        driverName, functionName, i, addr, status);
            }
            ++errs;
        }
        else {
            setDoubleParam(addr, i, param);
        }
    }

    if (acceptErrors) {
        return asynSuccess;
    }

    return (errs == 0)? asynSuccess : asynError;
}


asynStatus drvBPM::readUInt32Params(epicsUInt32 mask, int addr)
{
    int status = 0;

    /* Only FMC130M_4CH and FMC250M_4CH have these */
    if (streq(this->bpmType, "FMC130M_4CH") ||
            streq(this->bpmType, "FMC250M_4CH")) {
        status |= readAD9510Params(mask, addr);
        status |= readADCsParams(mask, addr);
    }
    /* All types, but FMCPOF_5CH have these */
    if (!streq(this->bpmType, "FMCPOF_5CH")) {
        status |= readGenParams(mask, addr);
        status |= readDSPParams(mask, addr);
    }

    return (asynStatus) status;
}

asynStatus drvBPM::readFloat64Params(int addr)
{
    int status = 0;

    /* Only FMC130M_4CH and FMC250M_4CH have these */
    if (streq(this->bpmType, "FMC130M_4CH") ||
            streq(this->bpmType, "FMC250M_4CH")) {
        status |= readSi57xParams(addr);
    }

    return (asynStatus) status;
}

asynStatus drvBPM::readIntegerParams(int addr)
{
    int status = 0;

    status = readOffsetParams(addr);

    return (asynStatus) status;
}

asynStatus drvBPM::readAD9510Params(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_AdcAD9510PllFunc, P_AdcAD9510Outputs, true);
}

asynStatus drvBPM::readADCsParams(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_AdcCalStatus, P_AdcCalStatus, true);
}

asynStatus drvBPM::readOffsetParams(int addr)
{
    return updateIntegerParams(addr, P_XOffset, P_YOffset, true);
}

asynStatus drvBPM::readSi57xParams(int addr)
{
    int status = asynSuccess;
    status = updateDoubleParams(addr, P_AdcSi57xFreq, P_AdcSi57xFreq, false);
    status |= updateUInt32Params(0xFFFFFFFF, addr, P_ActiveClkSi571Oe, P_ActiveClkSi571Oe, false);
    return (asynStatus)status;
}

asynStatus drvBPM::readTriggerParams(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_TriggerDir, P_TriggerTrnOutSel, false);
}

asynStatus drvBPM::readFMCPicoParams(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_FmcPicoRngR0, P_FmcPicoRngR3, true);
}

asynStatus drvBPM::readGenParams(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_SwMode, P_SwDivClk, true);
}

/* Some DSP parameters are only available in software. Som, only update
 * the ones in Hw here, otherwise we will have 0 for those */
asynStatus drvBPM::readDSPParams(epicsUInt32 mask, int addr)
{
    return updateUInt32Params(mask, addr, P_Kx, P_TbtDataMaskSamplesEnd, true);
}

asynStatus drvBPM::readGenDSPParams(epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    status = readGenParams(mask, addr);
    status |= readDSPParams(mask, addr);
    return (asynStatus)status;
}

/* Configuration routine.  Called directly, or from the iocsh function below */
extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvBPM class.
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] endpoint The address device string */
    int drvBPMConfigure(const char *portName, const char *endpoint,
            int bpmNumber, const char *type, int verbose, int timeout,
            int maxPoints, int maxBuffers, size_t maxMemory)
    {
        new drvBPM(portName, endpoint, bpmNumber, type, verbose,
                timeout, maxPoints, maxBuffers, maxMemory);
        return(asynSuccess);
    }

    /* EPICS iocsh shell commands */
    static const iocshArg initArg0 = { "portName", iocshArgString};
    static const iocshArg initArg1 = { "endpoint", iocshArgString};
    static const iocshArg initArg2 = { "bpmNumber", iocshArgInt};
    static const iocshArg initArg3 = { "type", iocshArgString};
    static const iocshArg initArg4 = { "verbose", iocshArgInt};
    static const iocshArg initArg5 = { "timeout", iocshArgInt};
    static const iocshArg initArg6 = { "maxPoints", iocshArgInt};
    static const iocshArg initArg7 = { "maxBuffers", iocshArgInt};
    static const iocshArg initArg8 = { "maxMemory", iocshArgInt};
    static const iocshArg * const initArgs[] = {&initArg0,
        &initArg1,
        &initArg2,
        &initArg3,
        &initArg4,
        &initArg5,
        &initArg6,
        &initArg7,
        &initArg8};
    static const iocshFuncDef initFuncDef = {"drvBPMConfigure",9,initArgs};
    static void initCallFunc(const iocshArgBuf *args)
    {
        drvBPMConfigure(args[0].sval, args[1].sval, args[2].ival,
                args[3].sval, args[4].ival, args[5].ival, args[6].ival,
                args[7].ival, args[8].ival);
    }

    void drvBPMRegister(void)
    {
        iocshRegister(&initFuncDef,initCallFunc);
    }

    epicsExportRegistrar(drvBPMRegister);
}
