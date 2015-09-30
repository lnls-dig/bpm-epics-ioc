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
#include "convert.h"
#include <epicsExport.h>

/** The polling interval when checking to see if acquisition is complete */
#define BPM_POLL_TIME                   .1

#define PI                              3.14159265
#define FREQ_SAMPLE                     100.00              /* Hz */
#define FREQ                            1.00                /* Hz */
#define TEST_LENGTH                     4092

/* FIXME: This should be read from hardware */
#define HARMONIC_NUMBER                 864
#define ADC_CLK_FREQ_UVX_DFLT           113040445           /* Hz */
#define ADC_RATE_FACTOR                 1
#define TBT_RATE_FACTOR                 35
#define FOFB_RATE_FACTOR                980
#define MONIT_RATE_FACTOR               9800000

#define ADC_DFLT_MAX_GAIN               ((1 << 16)-1)   /* 16-bit gain */
#define ADC_DFLT_MIN_GAIN               0
/* Default gain is just the mean of the gain range */
#define ADC_DFLT_GAIN                   ( ((ADC_DFLT_MAX_GAIN + \
                                            ADC_DFLT_MIN_GAIN) + 1) / 2)
#define ADC_DFLT_DIV_CLK                980             /* in ADC counts */

static const boardMap_t boardMap[MAX_BPMS] = {
         /* board, bpm*/
    /* 0  */ {1,  0},
    /* 1  */ {1,  1},
    /* 2  */ {2,  0},
    /* 3  */ {2,  1},
    /* 4  */ {3,  0},
    /* 5  */ {3,  1},
    /* 6  */ {4,  0},
    /* 7  */ {4,  1},
    /* 8  */ {5,  0},
    /* 9  */ {5,  1},
    /* 10 */ {6,  0},
    /* 11 */ {6,  1},
    /* 12 */ {7,  0},
    /* 13 */ {7,  1},
    /* 14 */ {8,  0},
    /* 15 */ {8,  1},
    /* 16 */ {9,  0},
    /* 17 */ {9,  1},
    /* 18 */ {10, 0},
    /* 19 */ {10, 1},
    /* 20 */ {11, 0},
    /* 21 */ {11, 1},
    /* 22 */ {12, 0},
    /* 23 */ {12, 1}
};

static const channelMap_t channelMap[CH_END] = {
         /* Amp, Phase, Pos, AmpA, AmpB, AmpC, AmpD, AmpALL */
    /* 0 = ADC      */  {0, -1, -1, 0,
                                        {WVF_ADC_A, WVF_ADC_B, WVF_ADC_C, WVF_ADC_D, WVF_ADC_ALL},
                                        {-1, -1, -1, -1, -1},
                                        {-1, -1, -1, -1, -1}},
    /* 1 = ADCSWAP  */  {1, -1, -1, 0,
                                        {WVF_ADCSWAP_A, WVF_ADCSWAP_B, WVF_ADCSWAP_C, WVF_ADCSWAP_D, WVF_ADCSWAP_ALL},
                                        {-1, -1, -1, -1, -1},
                                        {-1, -1, -1, -1, -1}},
    /* 2 = TBT      */  {6, -1, -1, 1,
                                        {WVF_TBTAMP_A, WVF_TBTAMP_B, WVF_TBTAMP_C, WVF_TBTAMP_D, WVF_TBTAMP_ALL},
                                        {WVF_TBTPHASE_A, WVF_TBTPHASE_B, WVF_TBTPHASE_C, WVF_TBTPHASE_D, WVF_TBTPHASE_ALL},
                                        {WVF_TBTPOS_A, WVF_TBTPOS_B, WVF_TBTPOS_C, WVF_TBTPOS_D, WVF_TBTPOS_ALL}},
    /* 3 = FOFB     */  {11, -1, -1, 1,
                                        {WVF_FOFBAMP_A, WVF_FOFBAMP_B, WVF_FOFBAMP_C, WVF_FOFBAMP_D, WVF_FOFBAMP_ALL},
                                        {WVF_FOFBPHASE_A, WVF_FOFBPHASE_B, WVF_FOFBPHASE_C, WVF_FOFBPHASE_D, WVF_FOFBPHASE_ALL},
                                        {WVF_FOFBPOS_A, WVF_FOFBPOS_B, WVF_FOFBPOS_C, WVF_FOFBPOS_D, WVF_FOFBPOS_ALL}},
};

/* Int32 functions mapping */
static const functionsInt32_t bpmSetGetKxFunc = {"DSP", bpm_set_kx, bpm_get_kx};
static const functionsInt32_t bpmSetGetKyFunc = {"DSP", bpm_set_ky, bpm_get_ky};
static const functionsInt32_t bpmSetGetKsumFunc = {"DSP", bpm_set_ksum, bpm_get_ksum};
static const functionsInt32_t bpmSetGetMonitAmpAFunc = {"DSP", bpm_set_monit_amp_ch0, bpm_get_monit_amp_ch0};
static const functionsInt32_t bpmSetGetMonitAmpBFunc = {"DSP", bpm_set_monit_amp_ch1, bpm_get_monit_amp_ch1};
static const functionsInt32_t bpmSetGetMonitAmpCFunc = {"DSP", bpm_set_monit_amp_ch2, bpm_get_monit_amp_ch2};
static const functionsInt32_t bpmSetGetMonitAmpDFunc = {"DSP", bpm_set_monit_amp_ch3, bpm_get_monit_amp_ch3};
static const functionsInt32_t bpmSetGetMonitUpdtFunc = {"DSP", bpm_set_monit_updt, bpm_get_monit_updt};
static const functionsInt32_t bpmSetGetRffeSwFunc = {"RFFE", bpm_set_rffe_sw, bpm_get_rffe_sw};
static const functionsInt32_t bpmSetGetAdcSwFunc = {"SWAP", bpm_set_sw, bpm_get_sw};
static const functionsInt32_t bpmSetGetAdcSwDlyFunc = {"SWAP", bpm_set_sw_dly, bpm_get_sw_dly};
static const functionsInt32_t bpmSetGetAdcSwEnFunc = {"SWAP", bpm_set_sw_en, bpm_get_sw_en};
static const functionsInt32_t bpmSetGetAdcSwDivClkFunc = {"SWAP", bpm_set_div_clk, bpm_get_div_clk};
static const functionsInt32_t bpmSetGetAdcWdwFunc = {"SWAP", bpm_set_wdw, bpm_get_wdw};
static const functionsInt32_t bpmSetGetAdcWdwDlyFunc = {"SWAP", bpm_set_wdw_dly, bpm_get_wdw_dly};
static const functionsInt32_t bpmSetGetAcqTriggerFunc = {"ACQ", bpm_set_acq_trig, bpm_get_acq_trig};
static const functionsInt32_t bpmSetGetAcqDataTrigThresFunc = {"ACQ", bpm_set_acq_data_trig_thres, bpm_get_acq_data_trig_thres};
static const functionsInt32_t bpmSetGetAcqDataTrigPolFunc = {"ACQ", bpm_set_acq_data_trig_pol, bpm_get_acq_data_trig_pol};
static const functionsInt32_t bpmSetGetAcqDataTrigSelFunc = {"ACQ", bpm_set_acq_data_trig_sel, bpm_get_acq_data_trig_sel};
static const functionsInt32_t bpmSetGetAcqDataTrigFiltFunc = {"ACQ", bpm_set_acq_data_trig_filt, bpm_get_acq_data_trig_filt};
static const functionsInt32_t bpmSetGetAcqHwDlyFunc = {"ACQ", bpm_set_acq_hw_trig_dly, bpm_get_acq_hw_trig_dly};

/* 2 Int32 functions mapping */
static const functions2Int32_t bpmSetGetAdcGainAAFunc = {"SWAP", bpm_set_gain_a, bpm_get_gain_a, 1};
static const functions2Int32_t bpmSetGetAdcGainBBFunc = {"SWAP", bpm_set_gain_b, bpm_get_gain_b, 1};
static const functions2Int32_t bpmSetGetAdcGainCCFunc = {"SWAP", bpm_set_gain_c, bpm_get_gain_c, 1};
static const functions2Int32_t bpmSetGetAdcGainDDFunc = {"SWAP", bpm_set_gain_d, bpm_get_gain_d, 1};
static const functions2Int32_t bpmSetGetAdcGainACFunc = {"SWAP", bpm_set_gain_a, bpm_get_gain_a, 2};
static const functions2Int32_t bpmSetGetAdcGainCAFunc = {"SWAP", bpm_set_gain_c, bpm_get_gain_c, 2};
static const functions2Int32_t bpmSetGetAdcGainBDFunc = {"SWAP", bpm_set_gain_b, bpm_get_gain_b, 2};
static const functions2Int32_t bpmSetGetAdcGainDBFunc = {"SWAP", bpm_set_gain_d, bpm_get_gain_d, 2};

/* Double functions mapping */
static const functionsFloat64_t bpmSetGetRffeAtt1Func = {"RFFE", bpm_set_rffe_att1, bpm_get_rffe_att1};
static const functionsFloat64_t bpmSetGetRffeAtt2Func = {"RFFE", bpm_set_rffe_att2, bpm_get_rffe_att2};
static const functionsFloat64_t bpmSetGetRffeTemp1Func = {"RFFE", NULL, bpm_get_rffe_temp1};
static const functionsFloat64_t bpmSetGetRffeTemp2Func = {"RFFE", NULL, bpm_get_rffe_temp2};
static const functionsFloat64_t bpmSetGetRffeTemp3Func = {"RFFE", NULL, bpm_get_rffe_temp3};
static const functionsFloat64_t bpmSetGetRffeTemp4Func = {"RFFE", NULL, bpm_get_rffe_temp4};

static const char *driverName="drvBPM";
void acqTask(void *drvPvt);

static void exitHandlerC(void *pPvt)
{
    drvBPM *pdrvBPM = (drvBPM *)pPvt;
    pdrvBPM->~drvBPM();
}

/** Constructor for the drvBPM class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] endpoint The device address string ]
 * */
drvBPM::drvBPM(const char *portName, const char *endpoint, int bpmNumber,
        int verbose, uint32_t timeout)
   : asynNDArrayDriver(portName,
                    MAX_ADDR, /* maxAddr */
                    (int)NUM_PARAMS,
                    0, 0,        /* maxBuffers, maxMemory, no limits */
                    asynUInt32DigitalMask | asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynGenericPointerMask | asynDrvUserMask,    /* Interface mask     */
                    asynUInt32DigitalMask | asynInt32Mask | asynInt16ArrayMask | asynFloat64Mask | asynGenericPointerMask,                      /* Interrupt mask     */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver blocks it is multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    asynStatus status;
    const char *functionName = "drvBPM";

    /* Create portName so we can create a new AsynUser later */
    bpmPortName = epicsStrDup(portName);

    this->endpoint = strdup(endpoint);
    if (this->endpoint == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvBPM failure to copy endpoint\n",
                driverName, functionName);
        status = asynError;
        goto endpoint_dup_err;
    }

    this->bpmNumber = bpmNumber;
    this->verbose = verbose;
    this->timeout = timeout;
    this->readingActive = 0;
    this->repetitiveTrigger = 0;

    /* Create events for signalling acquisition thread */
    this->startAcqEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->startAcqEventId) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s epicsEventCreate failure for start event\n",
                driverName, functionName);
        return;
    }

    this->stopAcqEventId = epicsEventCreate(epicsEventEmpty);
    if (!this->stopAcqEventId) {
        printf("%s:%s: epicsEventCreate failure for stop event\n",
            driverName, functionName);
        return;
    }

    /* Create parameters */
    createParam(P_HarmonicNumberString,
                                    asynParamUInt32Digital,         &P_HarmonicNumber);
    createParam(P_AdcClkFreqString, asynParamUInt32Digital,         &P_AdcClkFreq);
    createParam(P_TbtRateString,    asynParamUInt32Digital,         &P_TbtRate);
    createParam(P_FofbRateString,   asynParamUInt32Digital,         &P_FofbRate);
    createParam(P_MonitRateString,  asynParamUInt32Digital,         &P_MonitRate);
    createParam(P_CompMethodString, asynParamInt32,                 &P_CompMethod);
    createParam(P_BPMStatusString,  asynParamInt32,                 &P_BPMStatus);
    createParam(P_RffeAtt1String,   asynParamFloat64,               &P_RffeAtt1);
    createParam(P_RffeAtt2String,   asynParamFloat64,               &P_RffeAtt2);
    createParam(P_RffeTemp1String,  asynParamFloat64,               &P_RffeTemp1);
    createParam(P_RffeTemp2String,  asynParamFloat64,               &P_RffeTemp2);
    createParam(P_RffeTemp3String,  asynParamFloat64,               &P_RffeTemp3);
    createParam(P_RffeTemp4String,  asynParamFloat64,               &P_RffeTemp4);
    createParam(P_RffeSwString,     asynParamUInt32Digital,         &P_RffeSw);
    createParam(P_SwString,         asynParamUInt32Digital,         &P_Sw);
    createParam(P_SwDlyString,      asynParamUInt32Digital,         &P_SwDly);
    createParam(P_SwEnString,       asynParamUInt32Digital,         &P_SwEn);
    createParam(P_SwDivClkString,   asynParamUInt32Digital,         &P_SwDivClk);
    createParam(P_WdwString,        asynParamUInt32Digital,         &P_Wdw);
    createParam(P_WdwDlyString,     asynParamUInt32Digital,         &P_WdwDly);
    createParam(P_GainAAString,     asynParamUInt32Digital,         &P_GainAA);
    createParam(P_GainBBString,     asynParamUInt32Digital,         &P_GainBB);
    createParam(P_GainCCString,     asynParamUInt32Digital,         &P_GainCC);
    createParam(P_GainDDString,     asynParamUInt32Digital,         &P_GainDD);
    createParam(P_GainACString,     asynParamUInt32Digital,         &P_GainAC);
    createParam(P_GainCAString,     asynParamUInt32Digital,         &P_GainCA);
    createParam(P_GainBDString,     asynParamUInt32Digital,         &P_GainBD);
    createParam(P_GainDBString,     asynParamUInt32Digital,         &P_GainDB);
    createParam(P_KxString,         asynParamUInt32Digital,         &P_Kx);
    createParam(P_KyString,         asynParamUInt32Digital,         &P_Ky);
    createParam(P_KqString,         asynParamUInt32Digital,         &P_Kq);
    createParam(P_KsumString,       asynParamUInt32Digital,         &P_Ksum);
    createParam(P_XOffsetString,    asynParamUInt32Digital,         &P_XOffset);
    createParam(P_YOffsetString,    asynParamUInt32Digital,         &P_YOffset);
    createParam(P_QOffsetString,    asynParamUInt32Digital,         &P_QOffset);
    createParam(P_SamplesPreString, asynParamUInt32Digital,         &P_SamplesPre);
    createParam(P_SamplesPostString,
                                    asynParamUInt32Digital,         &P_SamplesPost);
    createParam(P_NumShotsString,   asynParamUInt32Digital,         &P_NumShots);
    createParam(P_ChannelString,    asynParamInt32,                 &P_Channel);
    createParam(P_UpdateTimeString, asynParamFloat64,               &P_UpdateTime);
    createParam(P_TriggerString,    asynParamUInt32Digital,         &P_Trigger);
    createParam(P_TriggerDataThresString,
                                    asynParamUInt32Digital,         &P_TriggerDataThres);
    createParam(P_TriggerDataPolString,
                                    asynParamUInt32Digital,         &P_TriggerDataPol);
    createParam(P_TriggerDataSelString,
                                    asynParamUInt32Digital,         &P_TriggerDataSel);
    createParam(P_TriggerDataFiltString,
                                    asynParamUInt32Digital,         &P_TriggerDataFilt);
    createParam(P_TriggerHwDlyString,
                                    asynParamUInt32Digital,         &P_TriggerHwDly);
    createParam(P_MonitAmpAString,  asynParamUInt32Digital,         &P_MonitAmpA);
    createParam(P_MonitAmpBString,  asynParamUInt32Digital,         &P_MonitAmpB);
    createParam(P_MonitAmpCString,  asynParamUInt32Digital,         &P_MonitAmpC);
    createParam(P_MonitAmpDString,  asynParamUInt32Digital,         &P_MonitAmpD);
    createParam(P_MonitUpdtString,  asynParamUInt32Digital,         &P_MonitUpdt);

    /* Set the initial values of some parameters */
    setUIntDigitalParam(P_HarmonicNumber,
                                        HARMONIC_NUMBER,    0xFFFFFFFF);
    setUIntDigitalParam(P_AdcClkFreq, ADC_CLK_FREQ_UVX_DFLT,
                                                            0xFFFFFFFF);
    setUIntDigitalParam(P_TbtRate,      TBT_RATE_FACTOR,    0xFFFFFFFF);
    setUIntDigitalParam(P_FofbRate,     FOFB_RATE_FACTOR,   0xFFFFFFFF);
    setUIntDigitalParam(P_MonitRate,    MONIT_RATE_FACTOR,  0xFFFFFFFF);
    setIntegerParam(P_CompMethod,                           0);
    setUIntDigitalParam(P_RffeSw,       0x1,                0xFFFFFFFF);
    setDoubleParam(P_RffeAtt1,                              31.5);
    setDoubleParam(P_RffeAtt2,                              31.5);
    setDoubleParam(P_RffeTemp1,                             0.0);
    setDoubleParam(P_RffeTemp2,                             0.0);
    setDoubleParam(P_RffeTemp3,                             0.0);
    setDoubleParam(P_RffeTemp4,                             0.0);
    setUIntDigitalParam(P_Sw,           0x1,                0xFFFFFFFF);
    setUIntDigitalParam(P_SwDly,        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SwEn,         0x0,                0xFFFFFFFF);
    setUIntDigitalParam(P_SwDivClk,     ADC_DFLT_DIV_CLK,   0xFFFFFFFF);
    setUIntDigitalParam(P_Wdw,          0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_WdwDly,       0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_GainAA,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainBB,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainCC,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainDD,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainAC,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainCA,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainBD,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_GainDB,       ADC_DFLT_GAIN,      0xFFFFFFFF);
    setUIntDigitalParam(P_Kx,           10000000,           0xFFFFFFFF);
    setUIntDigitalParam(P_Ky,           10000000,           0xFFFFFFFF);
    setUIntDigitalParam(P_Kq,           10000000,           0xFFFFFFFF);
    setUIntDigitalParam(P_Ksum,         1,                  0xFFFFFFFF);
    setUIntDigitalParam(P_XOffset,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_YOffset,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_QOffset,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_SamplesPre,   1000,               0xFFFFFFFF);
    setUIntDigitalParam(P_SamplesPost,  0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_NumShots,     1,                  0xFFFFFFFF);
    setIntegerParam(P_Channel,                              CH_ADC);
    setDoubleParam(P_UpdateTime,                            1.0);
    setUIntDigitalParam(P_Trigger,      0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TriggerDataThres,
                                        100,                0xFFFFFFFF);
    setUIntDigitalParam(P_TriggerDataPol,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TriggerDataSel,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TriggerDataFilt,
                                        1,                  0xFFFFFFFF);
    setUIntDigitalParam(P_TriggerHwDly,
                                        0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitAmpA,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitAmpB,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitAmpC,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitAmpD,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitPosA,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitPosB,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitPosC,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitPosD,    0,                  0xFFFFFFFF);
    setUIntDigitalParam(P_MonitUpdt,    0,                  0xFFFFFFFF);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    /* BPM HW Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHwInt32Func[P_Kx] = bpmSetGetKxFunc;
    bpmHwInt32Func[P_Ky] = bpmSetGetKyFunc;
    /* FIXME: There is no BPM function to do that. Add funcionality to
     * FPGA firmware */
#if 0
    bpmHwInt32Func[P_Kq] = bpmSetGetKqFunc;
#endif
    bpmHwInt32Func[P_Ksum] = bpmSetGetKsumFunc;
    bpmHwInt32Func[P_Sw] = bpmSetGetAdcSwFunc;
    bpmHwInt32Func[P_SwDly] = bpmSetGetAdcSwDlyFunc;
    bpmHwInt32Func[P_SwEn] = bpmSetGetAdcSwEnFunc;
    bpmHwInt32Func[P_SwDivClk] = bpmSetGetAdcSwDivClkFunc;
    bpmHwInt32Func[P_Wdw] = bpmSetGetAdcWdwFunc;
    bpmHwInt32Func[P_WdwDly] = bpmSetGetAdcWdwDlyFunc;
    bpmHwInt32Func[P_MonitAmpA] = bpmSetGetMonitAmpAFunc;
    bpmHwInt32Func[P_MonitAmpB] = bpmSetGetMonitAmpBFunc;
    bpmHwInt32Func[P_MonitAmpC] = bpmSetGetMonitAmpCFunc;
    bpmHwInt32Func[P_MonitAmpD] = bpmSetGetMonitAmpDFunc;
    bpmHwInt32Func[P_MonitUpdt] = bpmSetGetMonitUpdtFunc;
    bpmHwInt32Func[P_RffeSw] = bpmSetGetRffeSwFunc;
    bpmHwInt32Func[P_Trigger] = bpmSetGetAcqTriggerFunc;
    bpmHwInt32Func[P_TriggerDataThres] = bpmSetGetAcqDataTrigThresFunc;
    bpmHwInt32Func[P_TriggerDataPol] = bpmSetGetAcqDataTrigPolFunc;
    bpmHwInt32Func[P_TriggerDataSel] = bpmSetGetAcqDataTrigSelFunc;
    bpmHwInt32Func[P_TriggerDataFilt] = bpmSetGetAcqDataTrigFiltFunc;
    bpmHwInt32Func[P_TriggerHwDly] = bpmSetGetAcqHwDlyFunc;

    /* BPM HW 2 Int32 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHw2Int32Func[P_GainAA] = bpmSetGetAdcGainAAFunc;
    bpmHw2Int32Func[P_GainBB] = bpmSetGetAdcGainBBFunc;
    bpmHw2Int32Func[P_GainCC] = bpmSetGetAdcGainCCFunc;
    bpmHw2Int32Func[P_GainDD] = bpmSetGetAdcGainDDFunc;
    bpmHw2Int32Func[P_GainAC] = bpmSetGetAdcGainACFunc;
    bpmHw2Int32Func[P_GainCA] = bpmSetGetAdcGainCAFunc;
    bpmHw2Int32Func[P_GainBD] = bpmSetGetAdcGainBDFunc;
    bpmHw2Int32Func[P_GainDB] = bpmSetGetAdcGainDBFunc;

    /* BPM Float64 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHwFloat64Func[P_RffeAtt1] = bpmSetGetRffeAtt1Func;
    bpmHwFloat64Func[P_RffeAtt2] = bpmSetGetRffeAtt2Func;
    bpmHwFloat64Func[P_RffeTemp1] = bpmSetGetRffeTemp1Func;
    bpmHwFloat64Func[P_RffeTemp2] = bpmSetGetRffeTemp2Func;
    bpmHwFloat64Func[P_RffeTemp3] = bpmSetGetRffeTemp3Func;
    bpmHwFloat64Func[P_RffeTemp4] = bpmSetGetRffeTemp4Func;

    lock();
    status = bpmClientConnect();
    unlock();

    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling bpmClientConnect, status=%d\n",
            driverName, functionName, status);
    }

    /* Create the thread that computes the waveforms in the background */
    status = (asynStatus)(epicsThreadCreate("drvBPMTask",
                epicsThreadPriorityMedium,
                epicsThreadGetStackSize(epicsThreadStackMedium),
                (EPICSTHREADFUNC)::acqTask,
                this) == NULL);
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
    status = bpmClientDisconnect();
    unlock();
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling bpmClientDisconnect, status=%d\n",
            driverName, functionName, status);
    }

    free (this->endpoint);
    this->endpoint = NULL;
    free (this->bpmPortName);
    this->bpmPortName = NULL;
}

#if 0
asynStatus drvBPM::connect(asynUser* pasynUser)
{
    return bpmClientConnect();
}
#endif

asynStatus drvBPM::bpmClientConnect(void)
{
    asynStatus status = asynSuccess;
    const char *bpmLogFile = "stdout";
    const char *functionName = "bpmClientConnect";

    bpmClient = bpm_client_new_time (endpoint, verbose, bpmLogFile, timeout);
    if (bpmClient == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s bpmClientConnect failure to create bpm_client instance\n",
                driverName, functionName);
        status = asynError;
        goto create_bpm_client_err;
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: BPM client connected\n",
        driverName, functionName);

    pasynManager->exceptionConnect(this->pasynUserSelf);

create_bpm_client_err:
    return status;
}

#if 0
asynStatus drvBPM::disconnect(asynUser* pasynUser)
{
    return bpmClientDisconnect();
}
#endif

asynStatus drvBPM::bpmClientDisconnect(void)
{
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
            "%s: calling bpmClientDisconnect\n",
            driverName);
    asynStatus status = asynSuccess;
    bpm_client_destroy (&bpmClient);
    pasynManager->exceptionDisconnect(this->pasynUserSelf);
    return status;
}

void acqTask(void *drvPvt)
{
    drvBPM *pPvt = (drvBPM *)drvPvt;
    pPvt->acqTask();
}

/** Acquisition task that runs as a separate thread.
*/
void drvBPM::acqTask(void)
{
    int status = asynSuccess;
    asynUser *pasynUser = NULL;
    epicsUInt32 num_samples_pre;
    epicsUInt32 num_samples_post;
    epicsUInt32 num_shots;
    int channel;
    epicsUInt32 trigger;
    double updateTime;
    double delay;
    int hwAmpChannel = 0;
    int acqCompleted = 0;
    int bpmStatus = 0;
    epicsTimeStamp now;
    epicsFloat64 timeStamp;
    NDArray *pArrayAllChannels;
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
        /* Check if we received a stop event */
        status = epicsEventWaitWithTimeout(this->stopAcqEventId, BPM_POLL_TIME);
        if (status == epicsEventWaitOK || !repetitiveTrigger) {
            /* We got a stop event, abort acquisition */
            readingActive = 0;
            /* Only change state to IDLE if we are not in a error state */
            getIntegerParam(P_BPMStatus, &bpmStatus);
            if (bpmStatus != BPMStatusErrAcq) {
                setIntegerParam(P_BPMStatus, BPMStatusIdle);
                callParamCallbacks();
            }
            unlock();
            /* Release the lock while we wait for an event that says acquire has started, then lock again */
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquire to start\n", driverName, functionName);
            epicsEventWait(startAcqEventId);
            lock();
            readingActive = 1;
        }

        /* We are acquiring. Get the current time */
        epicsTimeGetCurrent(&startTime);

        /* Set the parameter in the parameter library. */
        getUIntDigitalParam(P_Trigger, &trigger, 0xFFFFFFFF);
        getUIntDigitalParam(P_SamplesPre, &num_samples_pre, 0xFFFFFFFF);
        getUIntDigitalParam(P_SamplesPost, &num_samples_post, 0xFFFFFFFF);
        getUIntDigitalParam(P_NumShots, &num_shots, 0xFFFFFFFF);
        getIntegerParam(P_Channel, &channel);
        getDoubleParam(P_UpdateTime, &updateTime);

        setIntegerParam(P_BPMStatus, BPMStatusAcquire);
        callParamCallbacks();

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
        dims[0] = POINTS_PER_SAMPLE;
        dims[1] = num_samples_pre + num_samples_post;

        /* Waveform statistics */
        epicsTimeGetCurrent(&now);
        getIntegerParam(NDArrayCounter, &arrayCounter);
        arrayCounter++;
        setIntegerParam(NDArrayCounter, arrayCounter);

        status = getAcqNDArrayType(hwAmpChannel, &NDType);
        if (status != asynSuccess) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to determine NDArray type for acquisition\n",
                    driverName, functionName);
            continue;
        }

        pArrayAllChannels = pNDArrayPool->alloc(MAX_WVF_DIMS, dims, NDType, 0, NULL);
        if (pArrayAllChannels == NULL) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayAllChannels\n",
                driverName, functionName);
        }
        pArrayAllChannels->uniqueId = arrayCounter;
        timeStamp = now.secPastEpoch + now.nsec / 1.e9;
        pArrayAllChannels->timeStamp = timeStamp;
        getAttributes(pArrayAllChannels->pAttributeList);

        /* Do acquisition */
        unlock();
        pasynManager->lockPort(pasynUser);
        status = startAcq(hwAmpChannel, num_samples_pre, num_samples_post,
                num_shots);
        pasynManager->unlockPort(pasynUser);
        lock();

        if (status == asynSuccess) {
            /* FIXME: Improve BPMStatus trigger waiting. The information
             * about waiting for trigger is not totally accurate here.
             * Although, we will for SW or HW trigger in a short time,
             * we are not actually there yet ...
             */
            if (trigger == TRIG_ACQ_EXT_HW) {
                setIntegerParam(P_BPMStatus, BPMStatusTriggerHwExtWaiting);
            }
            else if (trigger == TRIG_ACQ_EXT_DATA) {
                setIntegerParam(P_BPMStatus, BPMStatusTriggerHwDataWaiting);
            }
            else if (trigger == TRIG_ACQ_SW) {
                setIntegerParam(P_BPMStatus, BPMStatusTriggerSwWaiting);
            }

            callParamCallbacks();

            /* Wait for acquisition to complete, but allow acquire stop events to be handled */
            while (1) {
                unlock();
                status = epicsEventWaitWithTimeout(this->stopAcqEventId, BPM_POLL_TIME);
                lock();
                if (status == epicsEventWaitOK) {
                    /* TODO. We got a stop event, abort acquisition */
                    //this->pExpSetup->Stop();
                    readingActive = 0;
                    acqCompleted = 1;
                } else {
                    acqCompleted = checkAcqCompletion();
                }

                if (acqCompleted == 1) {
                    /* Get curve */
                    getAcqCurve(pArrayAllChannels, hwAmpChannel, num_samples_pre,
                            num_samples_post, num_shots);
                    break;
                }
            }
            /* Do callbacks on the full waveform (all channels interleaved) */
            unlock();
            /* We must do the callbacks with mutex unlocked ad the plugin
             * can call us and a deadlock would occur */
            doCallbacksGenericPointer(pArrayAllChannels, NDArrayData,
                    channelMap[channel].NDArrayAmp[WVF_AMP_ALL]);
            lock();

            /* Copy AMP data to arrays for each type of data, do callbacks on that */
            deinterleaveNDArray(pArrayAllChannels, channelMap[channel].NDArrayAmp,
                MAX_WVF_AMP_SINGLE, arrayCounter, timeStamp);

            /* Calculate positions and call callbacks */
            computePositions(pArrayAllChannels, channel);

            /* Release buffer */
            pArrayAllChannels->release();
            callParamCallbacks();
        }
        else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to acquire waveform\n",
                    driverName, functionName);
            /* Could not start acquisition. Invalid parameters */
            setIntegerParam(P_BPMStatus, BPMStatusErrAcq);
            callParamCallbacks();
            continue;
        }

        /* If we are in repetitive mode then sleep for the acquire period minus elapsed time. */
        if (repetitiveTrigger) {
            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            delay = updateTime - elapsedTime;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                     "%s:%s: delay=%f\n",
                      driverName, functionName, delay);
            if (delay >= 0.0) {
                /* We set the status to indicate we are in the period delay */
                setIntegerParam(P_BPMStatus, BPMStatusWaiting);
                callParamCallbacks();
                unlock();
                epicsEventWaitWithTimeout(this->stopAcqEventId, delay);
                lock();
            }
        }
    }
}

void drvBPM::deinterleaveNDArray (NDArray *pArrayAllChannels, const int *pNDArrayAddr,
        int pNDArrayAddrSize, int arrayCounter, epicsFloat64 timeStamp)
{
    int status = 0;
    size_t dims[MAX_WVF_DIMS];
    NDArrayInfo_t arrayInfo;
    NDDataType_t NDType;
    size_t arrayYStride = 0;
    NDArray *pArraySingleChannel;
    epicsUInt32 *pIn32;
    epicsUInt32 *pOut32;
    epicsUInt16 *pIn16;
    epicsUInt16 *pOut16;
    int channelAddr;
    static const char *functionName = "deinterleaveNDArray";

    status = pArrayAllChannels->getInfo(&arrayInfo);
    if (status != 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to get information about pArrayAllChannels\n",
                driverName, functionName);
        return;
    }

    arrayYStride = arrayInfo.yStride;
    dims[0] = arrayInfo.ySize;
    NDType = pArrayAllChannels->dataType;
    for (int i = 0; i < pNDArrayAddrSize; ++i) {
        //channelAddr = channelMap[channel].NDArray_Amp[i];
        channelAddr = pNDArrayAddr[i];
        pArraySingleChannel = pNDArrayPool->alloc(1, dims, NDType, 0, 0);
        pArraySingleChannel->uniqueId = arrayCounter;
        pArraySingleChannel->timeStamp = timeStamp;
        getAttributes(pArraySingleChannel->pAttributeList);

        pIn16 = (epicsUInt16 *)pArrayAllChannels->pData;
        pOut16 = (epicsUInt16 *)pArraySingleChannel->pData;
        pIn32 = (epicsUInt32 *)pArrayAllChannels->pData;
        pOut32 = (epicsUInt32 *)pArraySingleChannel->pData;

        /* Get only a single channel samples from a multi-channel
         * array */
        /* FIXME: ugly if */
        if (NDType == NDInt16) {
            for (size_t j = 0; j < dims[0]; ++j) {
                pOut16[j] = pIn16[i];
                pIn16 += arrayYStride;
            }
        }
        else if (NDType == NDInt32){
            for (size_t j = 0; j < dims[0]; ++j) {
                pOut32[j] = pIn32[i];
                pIn32 += arrayYStride;
            }
        }

        unlock();
        /* We must do the callbacks with mutex unlocked ad the plugin
         * can call us and a deadlock would occur */
        doCallbacksGenericPointer(pArraySingleChannel, NDArrayData,
                channelAddr);
        pArraySingleChannel->release();
        lock();
    }
}

/** This function computes the sums, diffs and positions
  * \param[in] NDArray of amplitudes interleaved (A1, B1, C1, D1,
  * A2, B2, C2, D2, ...)
  */
void drvBPM::computePositions(NDArray *pArrayAllChannels, int channel)
{
    int status = 0;
    epicsUInt32 XOffset;
    epicsUInt32 YOffset;
    epicsUInt32 QOffset;
    epicsUInt32 Kx;
    epicsUInt32 Ky;
    epicsUInt32 Kq;
    epicsUInt32 Ksum;
    epicsUInt32 mask = 0xFFFFFFFF;
    NDArrayInfo_t arrayInfo;
    size_t arrayElements = 0;
    size_t arrayYStride = 0;
    size_t arraySingleElements = 0;
    NDDataType_t NDType;
    ABCD_ROW *abcdRow;
    XYQS_ROW *xyqsRow;
    POS_OFFSETS posOffsets;
    K_FACTORS kFactors;
    NDArray *pArrayPosAllChannels = NULL;
    epicsTimeStamp now;
    epicsFloat64 timeStamp;
    int arrayCounter;
    static const char *functionName = "computePositions";

    /* Check if we need to compute position for this channel */
    if (channelMap[channel].CalcPos != 1) {
        goto no_calc_pos;
    }

    /* Get offsets and scaling factors */
    getUIntDigitalParam(P_XOffset, &XOffset, mask);
    getUIntDigitalParam(P_YOffset, &YOffset, mask);
    getUIntDigitalParam(P_QOffset, &QOffset, mask);
    getUIntDigitalParam(P_Kx, &Kx, mask);
    getUIntDigitalParam(P_Ky, &Ky, mask);
    getUIntDigitalParam(P_Kq, &Kq, mask);
    getUIntDigitalParam(P_Ksum, &Ksum, mask);

    /* FIXME: Interpret bits as signed */
    posOffsets.XOFFSET = (epicsInt32)XOffset;
    posOffsets.YOFFSET = (epicsInt32)YOffset;
    posOffsets.QOFFSET = (epicsInt32)QOffset;
    kFactors.KX = Kx;
    kFactors.KY = Ky;
    kFactors.KQ = Kq;
    kFactors.KSUM = Ksum;

    /* Get NDArray info */
    status = pArrayAllChannels->getInfo(&arrayInfo);
    if (status != 0) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to get information about pArrayAllChannels\n",
                driverName, functionName);
        goto get_arrayinfo_err;
    }

    arrayElements = arrayInfo.nElements;
    arrayYStride = arrayInfo.yStride;
    arraySingleElements = arrayElements/arrayYStride;

    /* Waveform statistics */
    epicsTimeGetCurrent(&now);
    getIntegerParam(NDArrayCounter, &arrayCounter);
    arrayCounter++;
    setIntegerParam(NDArrayCounter, arrayCounter);

    /* Alloc destination array */
    pArrayPosAllChannels = pNDArrayPool->copy(pArrayAllChannels, NULL, 0);
    if (pArrayPosAllChannels == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to alloc pArrayPosAllChannels\n",
                driverName, functionName);
        goto array_pool_copy_err;
    }
    pArrayPosAllChannels->uniqueId = arrayCounter;
    timeStamp = now.secPastEpoch + now.nsec / 1.e9;
    pArrayPosAllChannels->timeStamp = timeStamp;
    getAttributes(pArrayPosAllChannels->pAttributeList);

    /* FIXME: we must be sure that we are dealing with 32-bit data here and
     * sometimes we are not (ADC channels), organized in interleaved from
     * like: A0, B0, C0, D0, A1, B1, C1, D1 */
    NDType = pArrayAllChannels->dataType;
    if (NDType != NDInt32) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to calculate positions for data different than NDInt32\n",
                driverName, functionName);
        goto inv_ndtype_err;
    }

    abcdRow = (ABCD_ROW *) pArrayAllChannels->pData;
    xyqsRow = (XYQS_ROW *) pArrayPosAllChannels->pData;

    /* Compute position for all array positions */
    ABCDtoXYQS(abcdRow, xyqsRow, &kFactors, &posOffsets,
            arraySingleElements);

    /* Do callbacks on the full waveform (all channels interleaved) */
    unlock();
    /* We must do the callbacks with mutex unlocked ad the plugin
     * can call us and a deadlock would occur */
    doCallbacksGenericPointer(pArrayPosAllChannels, NDArrayData,
            channelMap[channel].NDArrayPos[WVF_POS_ALL]);
    lock();

    /* Copy data to arrays for each type of data, do callbacks on that */
    deinterleaveNDArray(pArrayPosAllChannels, channelMap[channel].NDArrayPos,
            MAX_WVF_POS_SINGLE, arrayCounter, timeStamp);

inv_ndtype_err:
    /* Release array */
    pArrayPosAllChannels->release();
array_pool_copy_err:
get_arrayinfo_err:
no_calc_pos:
    return;
}

asynStatus drvBPM::setAcquire()
{
    asynStatus status = asynSuccess;
    const char* functionName = "setAcquire";
    epicsUInt32 trigger_type = 0;

    /* Set the parameter in the parameter library. */
    getUIntDigitalParam(P_Trigger, &trigger_type, 0xFFFFFFFF);

    /* Set the trigger if it matches the HW */
    if (trigger_type < TRIG_ACQ_STOP) {
        setParam32_r (P_Trigger, 0xFFFFFFFF);
    }

    switch (trigger_type) {
        case TRIG_ACQ_NOW:
        case TRIG_ACQ_EXT_HW:
        case TRIG_ACQ_EXT_DATA:
        case TRIG_ACQ_SW:
            if (!readingActive && !repetitiveTrigger) {
                repetitiveTrigger = 0;
                /* Signal acq thread to start acquisition with the current parameters */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: trigger ACQ_NOW or HW/SW called\n",
                        driverName, functionName);
                epicsEventSignal(startAcqEventId);
            }
            break;

        /* Stop acquisition if we are in repetitive mode and if we are currently
         * acquiring. Otherwise, we don't need to do anything, as the acquisition
         * task will stop after the current acquisition */
        case TRIG_ACQ_STOP: /* Trigger == Stop */
            if (readingActive && repetitiveTrigger) {
                repetitiveTrigger = 0;
                /* Send the stop event */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: trigger ACQ_STOP called\n",
                        driverName, functionName);
                epicsEventSignal(this->stopAcqEventId);
            }
            break;

        case TRIG_ACQ_REPETITIVE:
            if (!repetitiveTrigger) {
                repetitiveTrigger = 1;
                /* Signal acq thread to start acquisition with the current parameters */
                asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s:%s: trigger ACQ_REPETITIVE called\n",
                        driverName, functionName);
                epicsEventSignal(startAcqEventId);
            }
            break;

        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: trigger type not implemented yet\n",
                    driverName, functionName);
            status = asynError;
            goto trig_unimplemented_err;
    }

trig_unimplemented_err:
    return status;
}

asynStatus drvBPM::startAcq(int hwChannel, epicsUInt32 num_samples_pre,
        epicsUInt32 num_samples_post, epicsUInt32 num_shots)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    const char* functionName = "startAcq";
    char service[50];
    acq_trans_t acq_trans;
    acq_req_t req;
    acq_block_t block;

    if (num_samples_pre + num_samples_post > MAX_ARRAY_POINTS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to select %u pre-trigger samples and\n"
                "%u post-trigger samples for acquisition\n",
                driverName, functionName, num_samples_pre, num_samples_post);
        status = asynError;
        goto bpm_samples_sel_err;
    }

    /* Get correct service name*/
    snprintf(service, sizeof(service), "BPM%d:DEVIO:ACQ%d",
        boardMap[this->bpmNumber].board, boardMap[this->bpmNumber].bpm);

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
    err = bpm_acq_start (bpmClient, service, &req);
    if (err != BPM_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to acquire waveform on hwChannel %d, with %u\n"
                "\tpre-trigger samples and %u post-trigger samples\n",
                driverName, functionName, hwChannel, num_samples_pre,
                num_samples_post);
        status = asynError;
        goto bpm_acq_err;
    }
#endif

bpm_acq_err:
bpm_samples_sel_err:
    return status;
}

int drvBPM::checkAcqCompletion()
{
    int status = 0;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    const char* functionName = "checkAcqCompletion";
    char service[50];

    /* Get correct service name*/
    snprintf(service, sizeof(service), "BPM%d:DEVIO:ACQ%d",
        boardMap[this->bpmNumber].board, boardMap[this->bpmNumber].bpm);

    err = bpm_acq_check (bpmClient, service);
    if (err != BPM_CLIENT_SUCCESS) {
        status = 0;
        goto bpm_acq_not_finished;
    }

    status = 1;

bpm_acq_not_finished:
    return status;
}

asynStatus drvBPM::getAcqCurve(NDArray *pArrayAllChannels, int hwChannel,
        epicsUInt32 num_samples_pre, epicsUInt32 num_samples_post,
        epicsUInt32 num_shots)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    const char* functionName = "getAcqCurve";
    char service[50];
    acq_trans_t acq_trans;
    acq_req_t req;
    acq_block_t block;

    /* Get correct service name*/
    snprintf(service, sizeof(service), "BPM%d:DEVIO:ACQ%d",
        boardMap[this->bpmNumber].board, boardMap[this->bpmNumber].bpm);

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
    err = bpm_acq_get_curve (bpmClient, service, &acq_trans);
    if (err != BPM_CLIENT_SUCCESS) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to read waveform on hwChannel %d, with %u\n"
                "\tpre-trigger samples and %u post-trigger samples\n",
                driverName, functionName, hwChannel, num_samples_pre,
                num_samples_post);
        status = asynError;
        goto bpm_acq_err;
    }

bpm_acq_err:
    return status;
}

asynStatus drvBPM::getAcqNDArrayType(int hwChannel, NDDataType_t *NDType)
{
    asynStatus status = asynSuccess;
    static const char *functionName = "getAcqNDArrayType";

    /* Determine minimum data size. FIXME: This should be in libbpmclient */
    switch (acq_chan[hwChannel].sample_size/POINTS_PER_SAMPLE) {
        case 2: /* bytes */
            *NDType = NDInt16;
            break;
        case 4: /* bytes */
            *NDType = NDInt32;
            break;
        default:
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to determine NDType for acquisition\n",
                    driverName, functionName);
            status = asynError;
            goto get_ndarray_type_err;
    }

get_ndarray_type_err:
    return status;
}

/** Called when asyn clients call pasynUInt32Digital->write().
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus drvBPM::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeUInt32Digital";

    /* Set the parameter in the parameter library. */
    setUIntDigitalParam(function, value, mask);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Some operations need some special handling*/
    if (function == P_Trigger) {
        /* If run was set then wake up the simulation task */
        setAcquire();
    }
    else {
        /* Do operation on HW. Some functions do not set anything on hardware */
        status = setParam32_r(function, mask);
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

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

asynStatus drvBPM::setParam32(int functionId, epicsUInt32 mask)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsUInt32 paramLib = 0;
    epicsUInt32 param1 = 0;
    epicsUInt32 param2 = 0;
    const char *functionName = "setParam32";
    char service[50];
    std::unordered_map<int,functionsInt32_t>::const_iterator func;
    std::unordered_map<int,functions2Int32_t>::const_iterator func2;

    status = getUIntDigitalParam(functionId, &paramLib, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function on 32-bit map */
    func = bpmHwInt32Func.find (functionId);
    if (func != bpmHwInt32Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.write) {
            goto no_registered_write_func;
        }

        /* Function found. Execute it */
        err = func->second.write(bpmClient, service, paramLib);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func->second.write() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_set_func1_param_err;
        }
        /* We've done our job here. No need to check other maps */
        return status;
    }

    /* Lookup function on 2 32-bit map */
    func2 = bpmHw2Int32Func.find (functionId);
    if (func2 != bpmHw2Int32Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func2->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func2->second.read) {
            goto no_registered_read_func2;
        }

        /* Function found. Read the HW values first as we need to update
           only one of the parameters */
        err = func2->second.read(bpmClient, service, &param1, &param2);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func2->second.read() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_get_func2_param_err;
        }

        /* Silently exit if no function is registered */
        if(!func2->second.write) {
            goto no_registered_write_func2;
        }

        /* Determine if we want to change the first or second parameter in HW */
        if (func2->second.parameterPos == 1) {
            err = func2->second.write(bpmClient, service, paramLib, param2);
        }
        else if (func2->second.parameterPos == 2) {
            err = func2->second.write(bpmClient, service, param1, paramLib);
        }

        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func2->second.write() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_set_func2_param_err;
        }
        /* We've done our job here. No need to check other maps */
        return status;
    }

bpm_set_func2_param_err:
no_registered_write_func2:
bpm_get_func2_param_err:
no_registered_read_func2:
bpm_set_func1_param_err:
no_registered_write_func:
get_param_err:
    return status;
}

asynStatus drvBPM::setParam32_r(int functionId, epicsUInt32 mask)
{
    lock();
    asynStatus status = setParam32(functionId, mask);
    unlock();
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
    const char *functionName = "readUInt32Digital";
    const char *paramName;

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get parameter, possibly from HW */
    status = getParam32_r(function, value, mask);

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

asynStatus drvBPM::getParam32(int functionId, epicsUInt32 *param,
        epicsUInt32 mask)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsUInt32 paramHw = 0;
    epicsUInt32 param1 = 0;
    epicsUInt32 param2 = 0;
    const char *functionName = "getParam32";
    char service[50];
    std::unordered_map<int,functionsInt32_t>::const_iterator func;
    std::unordered_map<int,functions2Int32_t>::const_iterator func2;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(functionId, param, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function */
    func = bpmHwInt32Func.find (functionId);
    if (func != bpmHwInt32Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.read) {
            goto no_registered_read_func;
        }

        /* Function found. Execute it */
        err = func->second.read(bpmClient, service, &paramHw);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func->second.read() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_get_func1_param_err;
        }

        /* Mask parameter according to the received mask */
        paramHw &= mask;
        *param = paramHw;
        /* We've done our job here. No need to check other maps */
        return status;
    }

    /* Lookup function */
    func2 = bpmHw2Int32Func.find (functionId);
    if (func2 != bpmHw2Int32Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func2->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func2->second.read) {
            goto no_registered_read_func2;
        }

        /* Function found. Execute it */
        err = func2->second.read(bpmClient, service, &param1, &param2);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func2->second.read() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_get_func2_param_err;
        }

        /* Determine if we want to read the first or second parameter in HW */
        if (func2->second.parameterPos == 1) {
            paramHw = param1;
        }
        else if (func2->second.parameterPos == 2) {
            paramHw = param2;
        }

        /* Mask parameter according to the received mask */
        paramHw &= mask;
        *param = paramHw;
        /* We've done our job here. No need to check other maps */
        return status;
    }

bpm_get_func2_param_err:
no_registered_read_func2:
bpm_get_func1_param_err:
no_registered_read_func:
get_param_err:
    return status;
}

asynStatus drvBPM::getParam32_r(int functionId, epicsUInt32 *param, epicsUInt32 mask)
{
    lock();
    asynStatus status = getParam32(functionId, param, mask);
    unlock();
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
    const char *paramName;
    const char* functionName = "writeInt32";

    /* Set the parameter in the parameter library. */
    setIntegerParam(function, value);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Call base class */
    status = asynNDArrayDriver::writeInt32(pasynUser, value);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

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
    asynStatus status = asynSuccess;

    /* Call base class */
    status = asynNDArrayDriver::readInt32(pasynUser, value);
    return status;
}

/** Called when asyn clients call pasynFloat64->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvBPM::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeFloat64";

    /* Set the parameter in the parameter library. */
    setDoubleParam(function, value);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParamDouble_r(function);

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

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

asynStatus drvBPM::setParamDouble(int functionId)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsFloat64 paramLib = 0;
    const char *functionName = "setParamDouble";
    char service[50];
    std::unordered_map<int,functionsFloat64_t>::const_iterator func;

    status = getDoubleParam(functionId, &paramLib);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving Parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function on float64 map */
    func = bpmHwFloat64Func.find (functionId);
    if (func != bpmHwFloat64Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.write) {
            goto no_registered_write_func;
        }

        /* Function found. Execute it */
        err = func->second.write(bpmClient, service, paramLib);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func->second.write() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_set_func_param_err;
        }
        /* We've done our job here. No need to check other maps */
        return status;
    }

bpm_set_func_param_err:
no_registered_write_func:
get_param_err:
    return status;
}

asynStatus drvBPM::setParamDouble_r(int functionId)
{
    lock();
    asynStatus status = setParamDouble(functionId);
    unlock();
    return status;
}

/** Called when asyn clients call pasynFloat64->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Value to read */
asynStatus drvBPM::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "readFloat64";

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get double param, possibly from HW */
    status = getParamDouble_r(function, value);

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

asynStatus drvBPM::getParamDouble(int functionId, epicsFloat64 *param)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    const char *functionName = "getParamDouble";
    char service[50];
    std::unordered_map<int,functionsFloat64_t>::const_iterator func;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(functionId, param);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function */
    func = bpmHwFloat64Func.find (functionId);
    if (func != bpmHwFloat64Func.end()) {
        /* Get correct service name*/
        snprintf(service, sizeof(service), "BPM%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.read) {
            goto no_registered_read_func;
        }

        /* Function found. Execute it */
        err = func->second.read(bpmClient, service, param);
        if (err != BPM_CLIENT_SUCCESS) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: func->second.read() failure\n",
                    driverName, functionName);
            status = asynError;
            goto bpm_get_func_param_err;
        }

        /* We've done our job here. No need to check other maps */
        return status;
    }

bpm_get_func_param_err:
no_registered_read_func:
get_param_err:
    return status;
}

asynStatus drvBPM::getParamDouble_r(int functionId, epicsFloat64 *param)
{
    lock();
    asynStatus status = getParamDouble(functionId, param);
    unlock();
    return status;
}

template <typename epicsType>
asynStatus drvBPM::doReadArray(asynUser *pasynUser, epicsType *value,
                               size_t nElements, size_t *nIn, epicsType *pValue)
{
    int function = pasynUser->reason;
    size_t ncopy = MAX_ARRAY_POINTS;
    int status = asynSuccess;
    const char *functionName = "doReadArray";

    if (ncopy > nElements) {
        ncopy = nElements;
    }

    memcpy(value, pValue, ncopy*sizeof(epicsType));
    *nIn = ncopy;

    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d",
                  driverName, functionName, status, function);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d\n",
              driverName, functionName, function);
    return (asynStatus)status;
}

/* Configuration routine.  Called directly, or from the iocsh function below */
extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvBPM class.
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] endpoint The address device string */
    int drvBPMConfigure(const char *portName, const char *endpoint,
            int bpmNumber, int verbose, uint32_t timeout)
    {
        new drvBPM(portName, endpoint, bpmNumber, verbose, timeout);
        return(asynSuccess);
    }

    /* EPICS iocsh shell commands */
    static const iocshArg initArg0 = { "portName", iocshArgString};
    static const iocshArg initArg1 = { "endpoint", iocshArgString};
    static const iocshArg initArg2 = { "bpmNumber", iocshArgInt};
    static const iocshArg initArg3 = { "verbose", iocshArgInt};
    static const iocshArg initArg4 = { "timeout", iocshArgInt};
    static const iocshArg * const initArgs[] = {&initArg0,
        &initArg1,
        &initArg2,
        &initArg3,
        &initArg4};
    static const iocshFuncDef initFuncDef = {"drvBPMConfigure",5,initArgs};
    static void initCallFunc(const iocshArgBuf *args)
    {
        drvBPMConfigure(args[0].sval, args[1].sval, args[2].ival,
                args[3].ival, args[4].ival);
    }

    void drvBPMRegister(void)
    {
        iocshRegister(&initFuncDef,initCallFunc);
    }

    epicsExportRegistrar(drvBPMRegister);
}
