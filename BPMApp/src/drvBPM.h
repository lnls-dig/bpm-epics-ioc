/*
 * drvBPM.cpp
 *
 * Authors: Juliano Murari
 *          Lucas Russo
 *
 * Created Jul. 13, 2015
 */

#include "asynNDArrayDriver.h"
#include <epicsExit.h>
#include <NDArray.h>
#include <epicsMutex.h>
#include <epicsRingBytes.h>
/* Third-party libraries */
#include <unordered_map>
#include <halcs_client.h>
#include <acq_client.h>
#include <bpm_client.h>

// any implementation for non c++-17 compilers
#include "any.hpp"

using linb::any;
using linb::any_cast;
using linb::bad_any_cast;

#define ARRAY_SIZE(ARRAY)           (sizeof(ARRAY)/sizeof((ARRAY)[0]))
/* Waveforms: RAW data, ADC SWAP data, TBT Amp, TBT Phase, FOFB Amp, FOFB Phase */
#define MAX_ARRAY_POINTS            200000
#define BPM_TIMEOUT                 1.0

/* We have two DCC cores, and therefore two FOFB_CC cores to control them:
 * - FMC core (0)
 * - P2P core (1) */
#define NUM_FOFB_CC_CORES_PER_FOFB 2
/* We have 8 channels on each FOFB_CC register set */
#define NUM_FOFB_CC_CHANNELS_PER_FOFB_CC 8

typedef enum {
    BPMIDReg = 0,
    BPMIDPM = 1,
    BPMIDEnd,
} bpm_coreID_types;

#define NUM_ACQ_CORES_PER_BPM       BPMIDEnd /* Regular Acquisition core and Post-Mortem */
#define NUM_TRIG_CORES_PER_BPM      NUM_ACQ_CORES_PER_BPM /* Trigger core for regular Acquisition and Post-Mortem */

/* BPM modes */
typedef enum {
    BPMModeMultiBunch = 0,
    BPMModeSinglePass = 1,
    BPMModeEnd,
} bpm_mode_types;

#define NUM_BPM_MODES               BPMModeEnd

/* BPM acquisition status */
typedef enum {
    BPMStatusIdle = 0,
    BPMStatusWaiting,
    BPMStatusTriggerHwExtWaiting,
    BPMStatusTriggerHwDataWaiting,
    BPMStatusTriggerSwWaiting,
    BPMStatusAcquire,
    BPMStatusErrAcq,
    BPMStatusAborted,
    BPMStatusErrTooManyPoints,
    BPMStatusErrTooFewPoints,
    BPMStatusErrNoMem,
    BPMStatusErrAcqOFlow,
} bpm_status_types;

/* Waveform IDs */
typedef enum {
    WVF_GENAMP_A = 0,
    WVF_GENAMP_B,
    WVF_GENAMP_C,
    WVF_GENAMP_D,
    WVF_GENAMP_ALL,
    WVF_UNUSED_1,
    WVF_GENPOS_A,
    WVF_GENPOS_B,
    WVF_GENPOS_C,
    WVF_GENPOS_D,
    WVF_GENPOS_ALL,
    WVF_UNUSED_2,
    WVF_AMP_PM_A,
    WVF_AMP_PM_B,
    WVF_AMP_PM_C,
    WVF_AMP_PM_D,
    WVF_AMP_PM_ALL,
    WVF_UNUSED_3,
    WVF_POS_PM_A,
    WVF_POS_PM_B,
    WVF_POS_PM_C,
    WVF_POS_PM_D,
    WVF_POS_PM_ALL,
    WVF_UNUSED_4,
    WVF_AMP_SP_A,
    WVF_AMP_SP_B,
    WVF_AMP_SP_C,
    WVF_AMP_SP_D,
    WVF_AMP_SP_ALL,
    WVF_UNUSED_5,
    WVF_MONIT_AMP_A,
    WVF_MONIT_AMP_B,
    WVF_MONIT_AMP_C,
    WVF_MONIT_AMP_D,
    WVF_MONIT_POS_X,
    WVF_MONIT_POS_Y,
    WVF_MONIT_POS_Q,
    WVF_MONIT_POS_SUM,
    WVF_MONIT_POSFAKE_X,
    WVF_MONIT_POSFAKE_Y,
    WVF_END
} wvf_types;

/* FIXME: This number must be at least the number of triggers
 * available on the FPGA. Although this is used to alloc the number
 * of waveforms, it's not used by getAddress () by the NDArray plugins,
 * as this function returns the address that is declared on plugin startup
 * (NDStdArraysConfigure function, NDArrayAddr). So, we are free to use all
 * of the addresses that are set by the database.
 * In summary, we use the different addresses to call different trigger channel
 * functions */
#define MAX_WAVEFORMS               WVF_END
/* FIXME FIXME: This should be read from HW. Also, this is actually less than 24,
 * but we let space for extra room */
#define MAX_TRIGGERS                24
/* This is needed so we have EPICS Asyn addresses sufficient for all of the
 * Triggers, from either ACQ core */
#define MAX_TRIGGERS_ALL_ACQ        (NUM_ACQ_CORES_PER_BPM*MAX_TRIGGERS)
/* Get the greater between them */
#define MAX_ADDR                    MAX(MAX_WAVEFORMS,MAX_TRIGGERS_ALL_ACQ)
/* Number of Monitoring waveforms */
#define MAX_MONIT_DATA              10

/* Channel IDs */
typedef enum {
    CH_ADC = 0,
    CH_ADCSWAP = 1,
    CH_TBT = 2,
    CH_FOFB = 3,
    CH_TBTPHA = 4,
    CH_FOFBPHA = 5,
    CH_MONIT1 = 6,
    CH_SP = 7,
    CH_END
} ch_types;

#define MAX_CHANNELS                CH_END

typedef enum {
    CH_HW_ADC = 0,
    CH_HW_ADCSWAP = 1,
    CH_HW_TBT = 6,
    CH_HW_TBTPHA = 7,
    CH_HW_FOFB = 11,
    CH_HW_FOFBPHA = 12,
    CH_HW_MONIT1 = 14,
    CH_HW_END
} ch_hw_types;

#define MAX_HW_CHANNELS             CH_HW_END

/* Waveform AMP types IDs */
typedef enum {
    WVF_AMP_A = 0,
    WVF_AMP_B,
    WVF_AMP_C,
    WVF_AMP_D,
    WVF_AMP_ALL,
    WVF_AMP_END
} wvf_amp_types;

#define MAX_WVF_AMP_SINGLE          (WVF_AMP_D+1)
#define MAX_WVF_AMP_TYPES           WVF_AMP_END

/* Waveform Phase types IDs */
typedef enum {
    WVF_PHASE_A = 0,
    WVF_PHASE_B,
    WVF_PHASE_C,
    WVF_PHASE_D,
    WVF_PHASE_ALL,
    WVF_PHASE_END
} wvf_pha_types;

#define MAX_WVF_PHA_SINGLE          (WVF_PHASE_D+1)
#define MAX_WVF_PHA_TYPES           WVF_PHASE_END

/* Waveform Position types IDs */
typedef enum {
    WVF_POS_X = 0,
    WVF_POS_Y,
    WVF_POS_Q,
    WVF_POS_SUM,
    WVF_POS_ALL,
    WVF_POS_END
} wvf_pos_types;

#define MAX_WVF_POS_SINGLE          (WVF_POS_SUM+1)
#define MAX_WVF_POS_TYPES           WVF_POS_END

/* One dimension for each point */
#define MAX_WVF_DIMS                2

#define MAX_SLOTS                   12
#define MAX_BPM_PER_SLOT            2
#define MAX_BPMS                    (MAX_SLOTS*MAX_BPM_PER_SLOT)

#define BPM_NUMBER_MIN              1
#define BPM_NUMBER_MAX              MAX_BPMS

/* AD9510 clock sel */
typedef enum {
    AD9510_ADC_CLK_SEL_1 = 1,
    AD9510_ADC_CLK_SEL_2 = 2,
    AD9510_ADC_CLK_SEL_END,
} ad9510_clk_sel_types;

#define MAX_AD9510_CLK_SEL_TYPES    AD9510_ADC_CLK_SEL_END

/* Si57x clock sel */
typedef enum {
    SI57X_DISABLE = 0,
    SI57X_ENABLE = 1,
    SI57X_OE_END,
} si57x_oe_types;

#define MAX_SI57X_OE_TYPES          SI57X_OE_END

/* BPM Mappping structure */
typedef struct {
    int board;
    int bpm;
    int core_id; /* Acquisition and Trigger core IDs */
} boardMap_t;

/* BPM Channel structure */
typedef struct {
    /* HW channel mapping. -1 means not available */
    int HwAmpChannel;
    int HwPhaseChannel;
    int HwPosChannel;
    /* 1 if we want to have position calculated from its amplitudes,
     * 0 otherwise */
    int CalcPos;
    /* NDArray addresses mapping */
    int NDArrayAmp[NUM_ACQ_CORES_PER_BPM][MAX_WVF_AMP_TYPES];
    int NDArrayPhase[NUM_ACQ_CORES_PER_BPM][MAX_WVF_PHA_TYPES];
    int NDArrayPos[NUM_ACQ_CORES_PER_BPM][MAX_WVF_POS_TYPES];
} channelMap_t;

/* BPM Reverse channel mapping structure */
typedef struct {
    /* EPICS channel. -1 means not available */
    int epicsChannel;
} channelRevMap_t;

/* BPM Acq Channel properties structure */
typedef struct {
    epicsUInt32 sampleSize;
    epicsUInt32 numAtoms;
    epicsUInt32 atomWidth;
} channelProp_t;

/* Write 32-bit function pointer */
typedef halcs_client_err_e (*writeUInt32Fp)(halcs_client_t *self, char *service,
    uint32_t param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readUInt32Fp)(halcs_client_t *self, char *service,
    uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeUInt32Fp write;
    readUInt32Fp read;
} functionsUInt32_t;

/* Write 32-bit function pointer */
typedef halcs_client_err_e (*writeInt32Fp)(halcs_client_t *self, char *service,
    int32_t param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readInt32Fp)(halcs_client_t *self, char *service,
    int32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32Fp write;
    readInt32Fp read;
} functionsInt32_t;

/* Write 32-bit function pointer with acq_client structure */
typedef halcs_client_err_e (*writeUInt32AcqFp)(acq_client_t *self, char *service,
    uint32_t param);
/* Read 32-bit function pointer with acq_client structure */
typedef halcs_client_err_e (*readUInt32AcqFp)(acq_client_t *self, char *service,
    uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeUInt32AcqFp write;
    readUInt32AcqFp read;
} functionsUInt32Acq_t;

/* Write 2 32-bit function pointer */
typedef halcs_client_err_e (*write2UInt32Fp)(halcs_client_t *self, char *service,
    uint32_t param1, uint32_t param2);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*read2UInt32Fp)(halcs_client_t *self, char *service,
    uint32_t *param1, uint32_t *param2);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    write2UInt32Fp write;
    read2UInt32Fp read;
    /* Which parameter (first or second) would trigger this function to be
     * executed on hardware (the other one won't be changed) */
    int parameterPos;
} functions2UInt32_t;

/* Write 64-bit float function pointer */
typedef halcs_client_err_e (*writeFloat64Fp)(halcs_client_t *self, char *service,
    double param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readFloat64Fp)(halcs_client_t *self, char *service,
    double *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeFloat64Fp write;
    readFloat64Fp read;
} functionsFloat64_t;

/* Write 32-bit function pointer with channel selection */
typedef halcs_client_err_e (*writeUInt32ChanFp)(halcs_client_t *self, char *service,
    uint32_t chan, uint32_t param);
/* Read 32-bit function pointer with channel selection */
typedef halcs_client_err_e (*readUInt32ChanFp)(halcs_client_t *self, char *service,
    uint32_t chan, uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeUInt32ChanFp write;
    readUInt32ChanFp read;
} functionsUInt32Chan_t;

typedef struct {
    union {
        epicsInt32 argInt32;
        epicsUInt32 argUInt32;
        epicsFloat64 argFloat64;
    };
} functionsArgs_t;

/* Forward declaration as struct functionsAny_t needs it */
class drvBPM;

/* Idea based on https://stackoverflow.com/questions/15102139/boostany-and-templates*/

/* Generic Function Structure for "any" function pointer */
struct functionsAny_t {
    template<typename T>
        functionsAny_t(T const& functionFp) :
            _functionFp(functionFp),
            _executeHwReadFunction(&functionsAny_t::executeHwReadFunction<T>),
            _executeHwWriteFunction(&functionsAny_t::executeHwWriteFunction<T>),
            _getServiceNameFromFunc(&functionsAny_t::getServiceNameFromFunc<T>) {}

    asynStatus executeHwRead(const drvBPM& drvBPM, char *service,
        int addr, functionsArgs_t &functionParam)
    {
        return (this->*_executeHwReadFunction)(drvBPM, _functionFp,
                service, addr, functionParam);
    }

    asynStatus executeHwWrite(const drvBPM& drvBPM, char *service,
        int addr, functionsArgs_t &functionParam)
    {
        return (this->*_executeHwWriteFunction)(drvBPM, _functionFp,
                service, addr, functionParam);
    }

    const char *getServiceName(const drvBPM& drvBPM)
    {
        return (this->*_getServiceNameFromFunc)(drvBPM, _functionFp);
    }

private:
    any _functionFp;
    /* Read template function for Hw execution */
    typedef asynStatus (functionsAny_t::*executeHwReadFunctionFp)
        (const drvBPM& drvBPM, const any& functionFp, char *service,
            int addr, functionsArgs_t &functionParam);
    executeHwReadFunctionFp _executeHwReadFunction;
    /* Write template function for Hw execution */
    typedef asynStatus (functionsAny_t::*executeHwWriteFunctionFp)
        (const drvBPM& drvBPM, const any& functionFp, char *service,
            int addr, functionsArgs_t &functionParam);
    executeHwWriteFunctionFp _executeHwWriteFunction;
    /* Service name utilities */
    typedef const char * (functionsAny_t::*getServiceNameFromFuncFp)
        (const drvBPM& drvBPM, const any& functionFp) const;
    getServiceNameFromFuncFp _getServiceNameFromFunc;

    /* Read function for Hw execution */
    template<typename T>
    asynStatus executeHwReadFunction(const drvBPM& drvBPM,
            const any& functionFp, char *service, int addr,
            functionsArgs_t &functionParam);

    /* Write function for Hw execution */
    template<typename T>
    asynStatus executeHwWriteFunction(const drvBPM& drvBPM,
            const any& functionFp, char *service, int addr,
            functionsArgs_t &functionParam);

    /* Service name utilities */
    template<typename T>
    const char *getServiceNameFromFunc(const drvBPM& drvBPM,
            const any& functionFp) const;
};

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_BPMModeString             "BPM_MODE"              /* asynInt32,              r/w */
#define P_BPMStatusString           "ACQ_STATUS"            /* asynInt32,              r/o */
#define P_HarmonicNumberString      "INFO_HARMNUMB"         /* asynUInt32Digital,      r/o */
#define P_ClkFreqString             "INFO_CLKFREQ"          /* asynFloat64,            r/w */
#define P_AdcRateString             "INFO_ADCRATE"          /* asynUInt32Digital,      r/o */
#define P_TbtRateString             "INFO_TBTRATE"          /* asynUInt32Digital,      r/o */
#define P_FofbRateString            "INFO_FOFBRATE"         /* asynUInt32Digital,      r/o */
#define P_MonitRateString           "INFO_MONITRATE"        /* asynUInt32Digital,      r/o */
#define P_Monit1RateString          "INFO_MONIT1RATE"       /* asynUInt32Digital,      r/o */
#define P_SwModeString              "SW_MODE"               /* asynUInt32Digital,      r/w */
#define P_SwDlyString               "SW_DLY"                /* asynUInt32Digital,      r/w */
#define P_SwDivClkString            "SW_DIVCLK"             /* asynUInt32Digital,      r/w */
#define P_SwDivFCntEnString         "SW_DIV_F_CNT_EN"       /* asynInt32,              r/w */
#define P_WdwString                 "ADC_WDW"               /* asynUInt32Digital,      r/w */
#define P_WdwDlyString              "ADC_WDWDLY"            /* asynfloat64             r/w */
#define P_AdcTrigDirString          "ADC_TRIGDIR"           /* asynUInt32Digital,      r/w */
#define P_AdcTrigTermString         "ADC_TRIGTERM"          /* asynUInt32Digital,      r/w */
#define P_AdcRandString             "ADC_RAND"              /* asynUInt32Digital,      r/w */
#define P_AdcDithString             "ADC_DITH"              /* asynUInt32Digital,      r/w */
#define P_AdcShdnString             "ADC_SHDN"              /* asynUInt32Digital,      r/w */
#define P_AdcPgaString              "ADC_PGA"               /* asynUInt32Digital,      r/w */
#define P_AdcTestDataString         "ADC_TESTDATA"          /* asynUInt32Digital,      r/w */
#define P_AdcClkSelString           "ADC_CLKSEL"            /* asynUInt32Digital,      r/w */
#define P_AdcSi57xFreqString        "ADC_SI57XFREQ"         /* asynFloat64,            r/w */
#define P_AdcSi57xFStartupString    "ADC_SI57XFSTARTUP"     /* asynFloat64,            r/w */
#define P_AdcAD9510DfltString       "ADC_AD9510DFLT"        /* asynUInt32Digital,      r/w */
#define P_AdcAD9510PllFuncString    "ADC_AD9510PLLFUNC"     /* asynUInt32Digital,      r/w */
#define P_AdcAD9510PllStatusString  "ADC_AD9510PLLSTATUS"   /* asynUInt32Digital,      r/w */
#define P_AdcAD9510ClkSelString     "ADC_AD9510CLKSEL"      /* asynUInt32Digital,      r/w */
#define P_AdcAD9510ADivString       "ADC_AD9510ADIV"        /* asynUInt32Digital,      r/w */
#define P_AdcAD9510BDivString       "ADC_AD9510BDIV"        /* asynUInt32Digital,      r/w */
#define P_AdcAD9510PrescalerString  "ADC_AD9510PRESCALER"   /* asynUInt32Digital,      r/w */
#define P_AdcAD9510RDivString       "ADC_AD9510RDIV"        /* asynUInt32Digital,      r/w */
#define P_AdcAD9510PllPDownString   "ADC_AD9510PLLPDOWN"    /* asynUInt32Digital,      r/w */
#define P_AdcAD9510MuxStatusString  "ADC_AD9510MUXSTATUS"   /* asynUInt32Digital,      r/w */
#define P_AdcAD9510CpCurrentString  "ADC_AD9510CPCURRENT"   /* asynUInt32Digital,      r/w */
#define P_AdcAD9510OutputsString    "ADC_AD9510OUTPUTS"     /* asynUInt32Digital,      r/w */
#define P_ActiveClkRstADCsString    "ACTIVE_CLK_RST_ADCS"   /* asynUInt32Digital,      r/w */
#define P_ActiveClkSi571OeString    "ACTIVE_CLK_SI57X_OE"   /* asynUInt32Digital,      r/w */
#define P_AfcSi57xFreqString        "AFC_SI57XFREQ"         /* asynFloat64,            r/w */
#define P_AfcSi57xFStartupString    "AFC_SI57XFSTARTUP"     /* asynFloat64,            r/w */
#define P_AfcSi57xOeString          "AFC_SI57XOE"           /* asynUInt32Digital,      r/w */
#define P_FmcPicoRngR0String        "FMCPICO_RNG_R0"        /* asynUInt32Digital,      r/w */
#define P_FmcPicoRngR1String        "FMCPICO_RNG_R1"        /* asynUInt32Digital,      r/w */
#define P_FmcPicoRngR2String        "FMCPICO_RNG_R2"        /* asynUInt32Digital,      r/w */
#define P_FmcPicoRngR3String        "FMCPICO_RNG_R3"        /* asynUInt32Digital,      r/w */
#define P_KxString                  "DSP_KX"                /* asynUInt32Digital,      r/w */
#define P_KyString                  "DSP_KY"                /* asynUInt32Digital,      r/w */
#define P_KqString                  "DSP_KQ"                /* asynUInt32Digital,      r/w */
#define P_KsumString                "DSP_KSUM"              /* asynUInt32Digital,      r/w */
#define P_XOffsetString             "DSP_XOFFSET"           /* asynUInt32Digital,      r/w */
#define P_YOffsetString             "DSP_YOFFSET"           /* asynUInt32Digital,      r/w */
#define P_QOffsetString             "DSP_QOFFSET"           /* asynUInt32Digital,      r/w */
#define P_SwTagEnString             "DSP_SW_TAG_EN"         /* asynUInt32Digital,      r/w */
#define P_SwDataMaskEnString        "DSP_SW_DATA_MASK_EN"   /* asynUInt32Digital,      r/w */
#define P_SwDataMaskSamplesString   "DSP_SW_DATA_MASK_SAMPLES"          /* asynUInt32Digital,      r/w */
#define P_TbtTagEnString            "DSP_TBT_TAG_EN"                    /* asynUInt32Digital,      r/w */
#define P_TbtTagDlyString           "DSP_TBT_TAG_DLY"                   /* asynUInt32Digital,      r/w */
#define P_TbtDataMaskEnString       "DSP_TBT_DATA_MASK_EN"              /* asynUInt32Digital,      r/w */
#define P_TbtDataMaskSamplesBegString  "DSP_TBT_DATA_MASK_SAMPLES_BEG"  /* asynUInt32Digital,      r/w */
#define P_TbtDataMaskSamplesEndString  "DSP_TBT_DATA_MASK_SAMPLES_END"  /* asynUInt32Digital,      r/w */
#define P_SwTagDesyncCntRstString   "DSP_SW_TAG_DESYNC_CNT_RST"         /* asynUInt32Digital,      r/w */
#define P_SwTagDesyncCntString      "DSP_SW_TAG_DESYNC_CNT"             /* asynUInt32Digital,      r/w */
#define P_TbtTagDesyncCntRstString   "DSP_TBT_TAG_DESYNC_CNT_RST"  /* asynUInt32Digital,      r/w */
#define P_TbtTagDesyncCntString      "DSP_TBT_TAG_DESYNC_CNT"      /* asynUInt32Digital,      r/w */
#define P_Monit1TagEnString            "DSP_MONIT1_TAG_EN"            /* asynUInt32Digital,      r/w */
#define P_Monit1TagDlyString           "DSP_MONIT1_TAG_DLY"           /* asynUInt32Digital,      r/w */
#define P_Monit1DataMaskEnString       "DSP_MONIT1_DATA_MASK_EN"      /* asynUInt32Digital,      r/w */
#define P_Monit1DataMaskSamplesBegString  "DSP_MONIT1_DATA_MASK_SAMPLES_BEG"  /* asynUInt32Digital,      r/w */
#define P_Monit1DataMaskSamplesEndString  "DSP_MONIT1_DATA_MASK_SAMPLES_END"  /* asynUInt32Digital,      r/w */
#define P_Monit1TagDesyncCntRstString   "DSP_MONIT1_TAG_DESYNC_CNT_RST" /* asynUInt32Digital,      r/w */
#define P_Monit1TagDesyncCntString      "DSP_MONIT1_TAG_DESYNC_CNT"     /* asynUInt32Digital,      r/w */
#define P_MonitTagEnString            "DSP_MONIT_TAG_EN"            /* asynUInt32Digital,      r/w */
#define P_MonitTagDlyString           "DSP_MONIT_TAG_DLY"           /* asynUInt32Digital,      r/w */
#define P_MonitDataMaskEnString       "DSP_MONIT_DATA_MASK_EN"      /* asynUInt32Digital,      r/w */
#define P_MonitDataMaskSamplesBegString  "DSP_MONIT_DATA_MASK_SAMPLES_BEG"  /* asynUInt32Digital,      r/w */
#define P_MonitDataMaskSamplesEndString  "DSP_MONIT_DATA_MASK_SAMPLES_END"  /* asynUInt32Digital,      r/w */
#define P_MonitTagDesyncCntRstString   "DSP_MONIT_TAG_DESYNC_CNT_RST" /* asynUInt32Digital,      r/w */
#define P_MonitTagDesyncCntString      "DSP_MONIT_TAG_DESYNC_CNT"     /* asynUInt32Digital,      r/w */
#define P_TimRcvPhaseMeasNavgString "TIM_RCV_PHASE_MEAS_NAVG"        /* asynUInt32Digital,      r/w */
#define P_TimRcvDMTDADeglitchThresString "TIM_RCV_DMTD_A_DEGLITCH"   /* asynUInt32Digital,      r/w */
#define P_TimRcvDMTDBDeglitchThresString "TIM_RCV_DMTD_B_DEGLITCH"   /* asynUInt32Digital,      r/w */
#define P_TimRcvPhaseMeasString     "TIM_RCV_PHASE_MEAS"    /* asynUInt32Digital,      r/w */
#define P_TimRcvDMTDAFreqString     "TIM_RCV_DMTD_A_FREQ"   /* asynUInt32Digital,      r/w */
#define P_TimRcvDMTDBFreqString     "TIM_RCV_DMTD_B_FREQ"   /* asynUInt32Digital,      r/w */
#define P_SamplesPreString          "ACQ_SAMPLES_PRE"       /* asynUInt32Digital,      r/w */
#define P_SamplesPostString         "ACQ_SAMPLES_POST"      /* asynUInt32Digital,      r/w */
#define P_NumShotsString            "ACQ_NUM_SHOTS"         /* asynUInt32Digital,      r/w */
#define P_ChannelString             "ACQ_CHANNEL"           /* asynInt32,              r/w */
#define P_TriggerString             "ACQ_TRIGGER"           /* asynUInt32Digital,      r/w */
#define P_TriggerEventString        "ACQ_TRIGGER_EVENT"     /* asynUInt32Digital,      r/w */
#define P_TriggerRepString          "ACQ_TRIGGER_REP"       /* asynUInt32Digital,      r/w */
#define P_UpdateTimeString          "ACQ_UPDATE_TIME"       /* asynFloat64,            r/w */
#define P_TriggerDataThresString    "ACQ_TRIGGER_THRES"     /* asynInt32,              r/w */
#define P_TriggerDataPolString      "ACQ_TRIGGER_POL"       /* asynInt32,              r/w */
#define P_TriggerDataSelString      "ACQ_TRIGGER_SEL"       /* asynInt32,              r/w */
#define P_TriggerDataFiltString     "ACQ_TRIGGER_FILT"      /* asynInt32,              r/w */
#define P_TriggerHwDlyString        "ACQ_TRIGGER_HWDLY"     /* asynInt32,              r/w */
#define P_DataTrigChanString        "ACQ_DATA_TRIG_CHAN"    /* asynuint32digital,      r/w */
#define P_ChannelSampleSizeString   "ACQ_CH_SAMPLE_SIZE"    /* asynUInt32Digital,      r/o */
#define P_ChannelNumAtomsString     "ACQ_CH_NUM_ATOMS"      /* asynUInt32Digital,      r/o */
#define P_ChannelAtomWidthString    "ACQ_CH_ATOM_WIDTH"     /* asynUInt32Digital,      r/o */
#define P_MonitAmpAString           "MONITAMP_A"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpBString           "MONITAMP_B"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpCString           "MONITAMP_C"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpDString           "MONITAMP_D"            /* asynUInt32Digital,      r/o */
#define P_MonitPosXString           "MONITPOS_X"            /* asynFloat64,            r/o */
#define P_MonitPosXFakeString       "MONITPOS_XFAKE"        /* asynFloat64,            r/o */
#define P_MonitPosYString           "MONITPOS_Y"            /* asynFloat64,            r/o */
#define P_MonitPosYFakeString       "MONITPOS_YFAKE"        /* asynFloat64,            r/o */
#define P_MonitPosQString           "MONITPOS_Q"            /* asynFloat64,            r/o */
#define P_MonitPosSumString         "MONITPOS_SUM"          /* asynFloat64,            r/o */
#define P_MonitUpdtString           "MONIT_UPDT"            /* asynUInt32Digital,      r/w */
#define P_MonitPollTimeString       "MONIT_POLL_TIME"       /* asynUInt32Digital,      r/w */
#define P_MonitEnableString         "MONIT_ENABLE"          /* asynInt32,              r/w */
#define P_SPAmpAString              "SP_AMP_A"              /* asynFloat64,            r/o */
#define P_SPAmpBString              "SP_AMP_B"              /* asynFloat64,            r/o */
#define P_SPAmpCString              "SP_AMP_C"              /* asynFloat64,            r/o */
#define P_SPAmpDString              "SP_AMP_D"              /* asynFloat64,            r/o */
#define P_SPPosXString              "SP_POS_X"              /* asynFloat64,            r/o */
#define P_SPPosYString              "SP_POS_Y"              /* asynFloat64,            r/o */
#define P_SPPosQString              "SP_POS_Q"              /* asynFloat64,            r/o */
#define P_SPPosSumString            "SP_POS_SUM"            /* asynFloat64,            r/o */
#define P_AdcTestModeString         "ADC_TEST_MODE"         /* asynUInt32Digital,      r/w */
#define P_AdcRstModesString         "ADC_RST_MODES"         /* asynUInt32Digital,      r/w */
#define P_AdcTempString             "ADC_TEMP"              /* asynUInt32Digital,      r/w */
#define P_AdcCalStatusString        "ADC_CAL_STATUS"        /* asynUInt32Digital,      r/w */
#define P_AdcRegReadString          "ADC_REG_READ"          /* asynUInt32Digital,      r/w */
#define P_AdcRegReadDataString      "ADC_REG_READ_DATA"     /* asynUInt32Digital,      r/o */
#define P_AdcRegReadAddrString      "ADC_REG_READ_ADDR"     /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteString         "ADC_REG_WRITE"         /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteDataString     "ADC_REG_WRITE_DATA"    /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteAddrString     "ADC_REG_WRITE_ADDR"    /* asynUInt32Digital,      r/w */
#define P_TriggerChanString         "TRIGGER_CHAN"          /* asynUInt32Digital,      r/w */
#define P_TriggerDirString          "TRIGGER_DIR"           /* asynUInt32Digital,      r/w */
#define P_TriggerDirPolString       "TRIGGER_DIR_POL"       /* asynUInt32Digital,      r/w */
#define P_TriggerRcvCntRstString    "TRIGGER_RCV_CNT_RST"   /* asynUInt32Digital,      r/w */
#define P_TriggerTrnCntRstString    "TRIGGER_TRN_CNT_RST"   /* asynUInt32Digital,      r/w */
#define P_TriggerRcvLenString       "TRIGGER_RCV_LEN"       /* asynUInt32Digital,      r/w */
#define P_TriggerTrnLenString       "TRIGGER_TRN_LEN"       /* asynUInt32Digital,      r/w */
#define P_TriggerCntRcvString       "TRIGGER_CNT_RCV"       /* asynUInt32Digital,      r/w */
#define P_TriggerCntTrnString       "TRIGGER_CNT_TRN"       /* asynUInt32Digital,      r/w */
#define P_TriggerRcvSrcString       "TRIGGER_RCV_SRC"       /* asynUInt32Digital,      r/w */
#define P_TriggerTrnSrcString       "TRIGGER_TRN_SRC"       /* asynUInt32Digital,      r/w */
#define P_TriggerRcvInSelString     "TRIGGER_RCV_IN_SEL"    /* asynUInt32Digital,      r/w */
#define P_TriggerTrnOutSelString    "TRIGGER_TRN_OUT_SEL"   /* asynUInt32Digital,      r/w */
#define P_IntlkEnString             "INTLK_EN"              /* asynUInt32Digital,      r/w */
#define P_IntlkClrString            "INTLK_CLR"             /* asynUInt32Digital,      r/w */
#define P_IntlkMinSumEnString         "INTLK_MIN_SUM_EN"          /* asynUInt32Digital,      r/w */
#define P_IntlkTransEnString          "INTLK_TRANS_EN"            /* asynUInt32Digital,      r/w */
#define P_IntlkTransClrString         "INTLK_TRANS_CLR"           /* asynUInt32Digital,      r/w */
#define P_IntlkAngEnString            "INTLK_ANG_EN"              /* asynUInt32Digital,      r/w */
#define P_IntlkAngClrString           "INTLK_ANG_CLR"             /* asynUInt32Digital,      r/w */
#define P_IntlkTransBiggerXString     "INTLK_TRANS_BIGGER_X"      /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerYString     "INTLK_TRANS_BIGGER_Y"      /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerLtcXString  "INTLK_TRANS_BIGGER_LTC_X"  /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerLtcYString  "INTLK_TRANS_BIGGER_LTC_Y"  /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerAnyString   "INTLK_TRANS_BIGGER_ANY"    /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerString      "INTLK_TRANS_BIGGER"        /* asynUInt32Digital,      ro */
#define P_IntlkTransBiggerLtcString   "INTLK_TRANS_BIGGER_LTC"    /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerXString       "INTLK_ANG_BIGGER_X"        /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerYString       "INTLK_ANG_BIGGER_Y"        /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerLtcXString    "INTLK_ANG_BIGGER_LTC_X"    /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerLtcYString    "INTLK_ANG_BIGGER_LTC_Y"    /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerAnyString     "INTLK_ANG_BIGGER_ANY"      /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerString        "INTLK_ANG_BIGGER"          /* asynUInt32Digital,      ro */
#define P_IntlkAngBiggerLtcString     "INTLK_ANG_BIGGER_LTC"      /* asynUInt32Digital,      ro */
#define P_IntlkString                 "INTLK"                     /* asynUInt32Digital,      ro */
#define P_IntlkLtcString              "INTLK_LTC"                 /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerXString    "INTLK_TRANS_SMALLER_X"      /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerYString    "INTLK_TRANS_SMALLER_Y"      /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerLtcXString "INTLK_TRANS_SMALLER_LTC_X"  /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerLtcYString "INTLK_TRANS_SMALLER_LTC_Y"  /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerAnyString  "INTLK_TRANS_SMALLER_ANY"    /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerString     "INTLK_TRANS_SMALLER"        /* asynUInt32Digital,      ro */
#define P_IntlkTransSmallerLtcString  "INTLK_TRANS_SMALLER_LTC"    /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerXString      "INTLK_ANG_SMALLER_X"        /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerYString      "INTLK_ANG_SMALLER_Y"        /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerLtcXString   "INTLK_ANG_SMALLER_LTC_X"    /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerLtcYString   "INTLK_ANG_SMALLER_LTC_Y"    /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerAnyString    "INTLK_ANG_SMALLER_ANY"      /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerString       "INTLK_ANG_SMALLER"          /* asynUInt32Digital,      ro */
#define P_IntlkAngSmallerLtcString    "INTLK_ANG_SMALLER_LTC"      /* asynUInt32Digital,      ro */
#define P_IntlkMinSumString           "INTLK_MIN_SUM"             /* asynUInt32Digital,      r/w */
#define P_IntlkTransMaxXString        "INTLK_TRANS_MAX_X"         /* asynInt32,      r/w */
#define P_IntlkTransMaxYString        "INTLK_TRANS_MAX_Y"         /* asynInt32,      r/w */
#define P_IntlkAngMaxXString          "INTLK_ANG_MAX_X"           /* asynInt32,      r/w */
#define P_IntlkAngMaxYString          "INTLK_ANG_MAX_Y"           /* asynInt32,      r/w */
#define P_IntlkTransMinXString        "INTLK_TRANS_MIN_X"         /* asynInt32,      r/w */
#define P_IntlkTransMinYString        "INTLK_TRANS_MIN_Y"         /* asynInt32,      r/w */
#define P_IntlkAngMinXString          "INTLK_ANG_MIN_X"           /* asynInt32,      r/w */
#define P_IntlkAngMinYString          "INTLK_ANG_MIN_Y"           /* asynInt32,      r/w */

#define P_FofbCtrlErrClrString                  "FOFB_CC_CFG_VAL_ERR_CLR"                   /* asynUInt32Digital,      r/w */
#define P_FofbCtrlCcEnableString                "FOFB_CC_CFG_VAL_CC_ENABLE"                 /* asynUInt32Digital,      r/w */
#define P_FofbCtrlTfsOverrideString             "FOFB_CC_CFG_VAL_TFS_OVERRIDE"              /* asynUInt32Digital,      r/w */
#define P_FofbCtrlBpmIdString                   "FOFB_CC_BPM_ID"                            /* asynUInt32Digital,      r/w */
#define P_FofbCtrlTimeFrameLenString            "FOFB_CC_TIME_FRAME_LEN"                    /* asynUInt32Digital,      r/w */
#define P_FofbCtrlMgtPowerdownString            "FOFB_CC_MGT_POWERDOWN"                     /* asynUInt32Digital,      r/w */
#define P_FofbCtrlMgtLoopbackString             "FOFB_CC_MGT_LOOPBACK"                      /* asynUInt32Digital,      r/w */
#define P_FofbCtrlTimeFrameDlyString            "FOFB_CC_TIME_FRAME_DLY"                    /* asynUInt32Digital,      r/w */
#define P_FofbCtrlGoldenOrbXString              "FOFB_CC_GOLDEN_ORB_X"                      /* asynUInt32Digital,      r/w */
#define P_FofbCtrlGoldenOrbYString              "FOFB_CC_GOLDEN_ORB_Y"                      /* asynUInt32Digital,      r/w */
#define P_FofbCtrlCustFeatureString             "FOFB_CC_CUST_FEATURE"                      /* asynUInt32Digital,      r/w */
#define P_FofbCtrlRxPolarityString              "FOFB_CC_RXPOLARITY"                        /* asynUInt32Digital,      r/w */
#define P_FofbCtrlPayloadselString              "FOFB_CC_PAYLOADSEL"                        /* asynUInt32Digital,      r/w */
#define P_FofbCtrlFofbdataselString             "FOFB_CC_FOFBDATASEL"                       /* asynUInt32Digital,      r/w */
#define P_FofbCtrlFirmwareVerString             "FOFB_CC_FIRMWARE_VER"                      /* asynUInt32Digital,      r/o */
#define P_FofbCtrlSysStatusString               "FOFB_CC_SYS_STATUS"                        /* asynUInt32Digital,      r/o */
#define P_FofbCtrlLinkPartnerString             "FOFB_CC_LINK_PARTNER_1"                    /* asynUInt32Digital,      r/o */
#define P_FofbCtrlLinkUpString                  "FOFB_CC_LINK_UP"                           /* asynUInt32Digital,      r/o */
#define P_FofbCtrlTimeFrameCountString          "FOFB_CC_TIME_FRAME_COUNT"                  /* asynUInt32Digital,      r/o */
#define P_FofbCtrlHardErrCntString              "FOFB_CC_HARD_ERR_CNT_1"                    /* asynUInt32Digital,      r/o */
#define P_FofbCtrlSoftErrCntString              "FOFB_CC_SOFT_ERR_CNT_1"                    /* asynUInt32Digital,      r/o */
#define P_FofbCtrlFrameErrCntString             "FOFB_CC_FRAME_ERR_CNT_1"                   /* asynUInt32Digital,      r/o */
#define P_FofbCtrlRxPckCntString                "FOFB_CC_RX_PCK_CNT_1"                      /* asynUInt32Digital,      r/o */
#define P_FofbCtrlTxPckCntString                "FOFB_CC_TX_PCK_CNT_1"                      /* asynUInt32Digital,      r/o */
#define P_FofbCtrlFodProcessTimeString          "FOFB_CC_FOD_PROCESS_TIME"                  /* asynUInt32Digital,      r/o */
#define P_FofbCtrlBpmCntString                  "FOFB_CC_BPM_COUNT"                         /* asynUInt32Digital,      r/o */
#define P_FofbCtrlToaRdEnString                 "FOFB_CC_TOA_CTL_RD_EN"                     /* asynUInt32Digital,      r/w */
#define P_FofbCtrlToaRdStrString                "FOFB_CC_TOA_CTL_RD_STR"                    /* asynUInt32Digital,      r/w */
#define P_FofbCtrlToaDataString                 "FOFB_CC_TOA_DATA_VAL"                      /* asynUInt32Digital,      r/o */
#define P_FofbCtrlRcbRdEnString                 "FOFB_CC_RCB_CTL_RD_EN"                     /* asynUInt32Digital,      r/w */
#define P_FofbCtrlRcbRdStrString                "FOFB_CC_RCB_CTL_RD_STR"                    /* asynUInt32Digital,      r/w */
#define P_FofbCtrlRcbDataString                 "FOFB_CC_RCB_DATA_VAL"                      /* asynUInt32Digital,      r/o */

typedef enum {
    TRIG_ACQ_START,
    TRIG_ACQ_STOP,
    TRIG_ACQ_ABORT,
} trigEvent_e;

class drvBPM : public asynNDArrayDriver {
    public:
        drvBPM(const char *portName, const char *endpoint, int bpmNumber,
                const char *type, int verbose, int timeout,
                int maxPoints, int maxBuffers, size_t maxMemory);
        ~drvBPM();

        /* These are the methods that we override from asynPortDriver */
        virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
                epicsUInt32 mask);
        virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
                epicsUInt32 mask);
        virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

        /* These methods are overwritten from asynPortDriver */
        virtual asynStatus connect(asynUser* pasynUser);
        virtual asynStatus disconnect(asynUser* pasynUser);

        /* These are the methods that are new to this class */
        void acqTask(int coreID, double pollTime, bool autoStart);
        void acqSPTask(int coreID, double pollTime, bool autoStart);
        void acqMonitTask();

        /* Overloaded functions for extracting service name*/
        const char *doGetServiceNameFromFunc (functionsUInt32_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functionsInt32_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functionsUInt32Acq_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functions2UInt32_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functionsFloat64_t &func) const
        {
            return func.serviceName;
        }

        const char *doGetServiceNameFromFunc (functionsUInt32Chan_t &func) const
        {
            return func.serviceName;
        }

        /* Overloaded function mappings called by functionsAny_t */
        asynStatus doExecuteHwWriteFunction(functionsUInt32Acq_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functions2UInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsUInt32Chan_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsUInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwWriteFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        asynStatus doExecuteHwReadFunction(functionsUInt32Acq_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functions2UInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsUInt32Chan_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsUInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus doExecuteHwReadFunction(functionsInt32_t &func, char *service,
                int addr, functionsArgs_t &functionParam) const;
        asynStatus executeHwReadFunction(int functionId, int addr,
                functionsArgs_t &functionParam);

        /* General service name handling utilities */
        asynStatus getServiceChan (int bpmNumber, int addr, const char *serviceName,
                epicsUInt32 *chanArg) const;
        asynStatus getServiceID (int bpmNumber, int addr, const char *serviceName,
                int *serviceIDArg) const;
        asynStatus getFullServiceName (int bpmNumber, int addr, const char *serviceName,
                char *fullServiceName, int fullServiceNameSize) const;

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_BPMMode;
#define FIRST_COMMAND P_BPMMode
        int P_BPMStatus;
        int P_BPMCount;
        int P_HarmonicNumber;
        int P_ClkFreq;
        int P_AdcRate;
        int P_TbtRate;
        int P_FofbRate;
        int P_MonitRate;
        int P_Monit1Rate;
        int P_SwMode;
        int P_SwDly;
        int P_SwDivClk;
        int P_SwDivFCntEn;
        int P_AdcTrigDir;
        int P_AdcTrigTerm;
        int P_AdcRand;
        int P_AdcDith;
        int P_AdcShdn;
        int P_AdcPga;
        int P_AdcTestData;
        int P_AdcClkSel;
        int P_AdcSi57xFreq;
        int P_AdcSi57xFStartup;
        int P_AdcAD9510Dflt;
        int P_AdcAD9510PllFunc;
        int P_AdcAD9510PllStatus;
        int P_AdcAD9510ClkSel;
        int P_AdcAD9510ADiv;
        int P_AdcAD9510BDiv;
        int P_AdcAD9510Prescaler;
        int P_AdcAD9510RDiv;
        int P_AdcAD9510PllPDown;
        int P_AdcAD9510MuxStatus;
        int P_AdcAD9510CpCurrent;
        int P_AdcAD9510Outputs;
        int P_ActiveClkRstADCs;
        int P_ActiveClkSi571Oe;
        int P_AfcSi57xFreq;
        int P_AfcSi57xFStartup;
        int P_AfcSi57xOe;
        int P_FmcPicoRngR0;
        int P_FmcPicoRngR1;
        int P_FmcPicoRngR2;
        int P_FmcPicoRngR3;
        int P_Kx;
        int P_Ky;
        int P_Kq;
        int P_Ksum;
        int P_XOffset;
        int P_YOffset;
        int P_QOffset;
        int P_SwTagEn;
        int P_SwDataMaskEn;
        int P_SwDataMaskSamples;
        int P_TbtTagEn;
        int P_TbtTagDly;
        int P_TbtDataMaskEn;
        int P_TbtDataMaskSamplesBeg;
        int P_TbtDataMaskSamplesEnd;
        int P_SwTagDesyncCntRst;
        int P_SwTagDesyncCnt;
        int P_TbtTagDesyncCntRst;
        int P_TbtTagDesyncCnt;
        int P_Monit1TagEn;
        int P_Monit1TagDly;
        int P_Monit1DataMaskEn;
        int P_Monit1DataMaskSamplesBeg;
        int P_Monit1DataMaskSamplesEnd;
        int P_Monit1TagDesyncCntRst;
        int P_Monit1TagDesyncCnt;
        int P_MonitTagEn;
        int P_MonitTagDly;
        int P_MonitDataMaskEn;
        int P_MonitDataMaskSamplesBeg;
        int P_MonitDataMaskSamplesEnd;
        int P_MonitTagDesyncCntRst;
        int P_MonitTagDesyncCnt;
        int P_TimRcvPhaseMeasNavg;
        int P_TimRcvDMTDADeglitchThres;
        int P_TimRcvDMTDBDeglitchThres;
        int P_TimRcvPhaseMeas;
        int P_TimRcvDMTDAFreq;
        int P_TimRcvDMTDBFreq;
        int P_SamplesPre;
        int P_SamplesPost;
        int P_NumShots;
        int P_Channel;
        int P_UpdateTime;
        int P_Trigger;
        int P_TriggerEvent;
        int P_TriggerRep;
        int P_TriggerDataThres;
        int P_TriggerDataPol;
        int P_TriggerDataSel;
        int P_TriggerDataFilt;
        int P_TriggerHwDly;
        int P_DataTrigChan;
        int P_ChannelSampleSize;
        int P_ChannelNumAtoms;
        int P_ChannelAtomWidth;
        int P_MonitAmpA;
        int P_MonitAmpB;
        int P_MonitAmpC;
        int P_MonitAmpD;
        int P_MonitPosX;
        int P_MonitPosXFake;
        int P_MonitPosY;
        int P_MonitPosYFake;
        int P_MonitPosQ;
        int P_MonitPosSum;
        int P_MonitUpdt;
        int P_MonitPollTime;
        int P_MonitEnable;
        int P_SPAmpA;
        int P_SPAmpB;
        int P_SPAmpC;
        int P_SPAmpD;
        int P_SPPosX;
        int P_SPPosY;
        int P_SPPosQ;
        int P_SPPosSum;
        int P_AdcTestMode;
        int P_AdcRstModes;
        int P_AdcTemp;
        int P_AdcCalStatus;
        int P_AdcRegRead;
        int P_AdcRegReadData;
        int P_AdcRegReadAddr;
        int P_AdcRegWrite;
        int P_AdcRegWriteData;
        int P_AdcRegWriteAddr;
        int P_TriggerChan;
        int P_TriggerDir;
        int P_TriggerDirPol;
        int P_TriggerRcvCntRst;
        int P_TriggerTrnCntRst;
        int P_TriggerRcvLen;
        int P_TriggerTrnLen;
        int P_TriggerCntRcv;
        int P_TriggerCntTrn;
        int P_TriggerRcvSrc;
        int P_TriggerTrnSrc;
        int P_TriggerRcvInSel;
        int P_TriggerTrnOutSel;
        int P_IntlkEn;
        int P_IntlkClr;
        int P_IntlkMinSumEn;
        int P_IntlkTransEn;
        int P_IntlkTransClr;
        int P_IntlkAngEn;
        int P_IntlkAngClr;
        int P_IntlkTransBiggerX;
        int P_IntlkTransBiggerY;
        int P_IntlkTransBiggerLtcX;
        int P_IntlkTransBiggerLtcY;
        int P_IntlkTransBiggerAny;
        int P_IntlkTransBigger;
        int P_IntlkTransBiggerLtc;
        int P_IntlkAngBiggerX;
        int P_IntlkAngBiggerY;
        int P_IntlkAngBiggerLtcX;
        int P_IntlkAngBiggerLtcY;
        int P_IntlkAngBiggerAny;
        int P_IntlkAngBigger;
        int P_IntlkAngBiggerLtc;
        int P_Intlk;
        int P_IntlkLtc;
        int P_IntlkTransSmallerX;
        int P_IntlkTransSmallerY;
        int P_IntlkTransSmallerLtcX;
        int P_IntlkTransSmallerLtcY;
        int P_IntlkTransSmallerAny;
        int P_IntlkTransSmaller;
        int P_IntlkTransSmallerLtc;
        int P_IntlkAngSmallerX;
        int P_IntlkAngSmallerY;
        int P_IntlkAngSmallerLtcX;
        int P_IntlkAngSmallerLtcY;
        int P_IntlkAngSmallerAny;
        int P_IntlkAngSmaller;
        int P_IntlkAngSmallerLtc;
        int P_IntlkMinSum;
        int P_IntlkTransMaxX;
        int P_IntlkTransMaxY;
        int P_IntlkAngMaxX;
        int P_IntlkAngMaxY;
        int P_IntlkTransMinX;
        int P_IntlkTransMinY;
        int P_IntlkAngMinX;
        int P_IntlkAngMinY;
        int P_IntlkAngDiffX;
        int P_IntlkAngDiffY;
        int P_IntlkTransDiffX;
        int P_IntlkTransDiffY;
        int P_AdcGainFixedPointPos;
        int P_AdcCh0SwDir;
        int P_AdcCh1SwDir;
        int P_AdcCh2SwDir;
        int P_AdcCh3SwDir;
        int P_AdcCh0SwInv;
        int P_AdcCh1SwInv;
        int P_AdcCh2SwInv;
        int P_AdcCh3SwInv;
        int P_FofbCtrlErrClr;
        int P_FofbCtrlCcEnable;
        int P_FofbCtrlTfsOverride;
        int P_FofbCtrlBpmId;
        int P_FofbCtrlTimeFrameLen;
        int P_FofbCtrlMgtPowerdown;
        int P_FofbCtrlMgtLoopback;
        int P_FofbCtrlTimeFrameDly;
        int P_FofbCtrlGoldenOrbX;
        int P_FofbCtrlGoldenOrbY;
        int P_FofbCtrlCustFeature;
        int P_FofbCtrlRxPolarity;
        int P_FofbCtrlPayloadsel;
        int P_FofbCtrlFofbdatasel;
        int P_FofbCtrlFirmwareVer;
        int P_FofbCtrlSysStatus;
        int P_FofbCtrlLinkPartner;
        int P_FofbCtrlLinkUp;
        int P_FofbCtrlTimeFrameCount;
        int P_FofbCtrlHardErrCnt;
        int P_FofbCtrlSoftErrCnt;
        int P_FofbCtrlFrameErrCnt;
        int P_FofbCtrlRxPckCnt;
        int P_FofbCtrlTxPckCnt;
        int P_FofbCtrlFodProcessTime;
        int P_FofbCtrlBpmCnt;
        int P_FofbCtrlToaRdEn;
        int P_FofbCtrlToaRdStr;
        int P_FofbCtrlToaData;
        int P_FofbCtrlRcbRdEn;
        int P_FofbCtrlRcbRdStr;
        int P_FofbCtrlRcbData;
#define LAST_COMMAND P_FofbCtrlRcbData

    private:
        /* Our data */
        halcs_client_t *bpmClient;
        halcs_client_t *bpmClientMonit;
        acq_client_t *bpmClientAcqParam[NUM_ACQ_CORES_PER_BPM];
        acq_client_t *bpmClientAcq[NUM_ACQ_CORES_PER_BPM];
        bpm_single_pass_t *bpmSinglePass[NUM_ACQ_CORES_PER_BPM];
        char *endpoint;
        int bpmNumber;
        int bpmMaxPoints;
        int verbose;
        int timeout;
        char *bpmPortName;
        char *bpmType;
        int readingActive[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        int repetitiveTrigger[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId startAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId stopAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId abortAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId reconfSPassAcqEventId[NUM_ACQ_CORES_PER_BPM];
        epicsEventId activeAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId activeMonitEnableEventId;
        std::unordered_map<int, functionsAny_t> bpmHwFunc;

        epicsInt32 adc_gain_fixed_point_pos;

        /* Our private methods */

        /* Client connection management */
        asynStatus bpmClientConnect(asynUser* pasynUser);
        asynStatus bpmClientDisconnect(asynUser* pasynUser);

        /* Acquisition functions */
        asynStatus setAcqEvent(epicsUInt32 mask, int addr);
        asynStatus getAcqNDArrayType(int coreID, int channel, epicsUInt32 atomWidth, NDDataType_t *NDType);
        asynStatus getChannelProperties(int coreID, int channel, channelProp_t *channelProp);
        bpm_status_types getBPMInitAcqStatus(int coreID);
        asynStatus startAcq(int coreID, int hwChannel, epicsUInt32 num_samples_pre,
                epicsUInt32 num_samples_post, epicsUInt32 num_shots);
        asynStatus startSPAcq(bpm_single_pass_t *bpm_single_pass);
        asynStatus setAcqTrig(int coreID, acq_client_trig_e trig);
        asynStatus initAcqPM(int coreID);
        asynStatus abortAcqRaw(int coreID, acq_client_t *acq_client);
        asynStatus abortAcq(int coreID);
        asynStatus abortAcqFromPortThread(int coreID);
        asynStatus abortAcqTask(int addr, int bpmMode, bool abortAcqHw = false);
        asynStatus stopAcqTask(int addr, int bpmMode);
        int checkAcqCompletion(int coreID);
        int checkSPAcqCompletion(bpm_single_pass_t *bpm_single_pass);
        asynStatus getAcqCurve(int coreID, NDArray *pArrayAllChannels, int hwChannel,
                epicsUInt32 num_samples_pre, epicsUInt32 num_samples_post,
                epicsUInt32 num_shots);
        asynStatus getAcqSPCurve(bpm_single_pass_t *bpm_single_pass, NDArray *pArrayAllChannels);
        asynStatus getAcqSPSamples(bpm_single_pass_t *bpm_single_pass, bpm_sample_t *bpm_sample);
        asynStatus deinterleaveNDArray (NDArray *pArrayAllChannels, const int *pNDArrayAddr,
                int pNDArrayAddrSize, int arrayCounter, epicsTimeStamp *timeStamp);

        /* General set/get hardware functions */
        asynStatus computePositions(int coreID, NDArray *pArrayAllChannels, int channel,
                epicsTimeStamp *timeStamp);
        asynStatus setParamGeneric(int funcionId, int addr);
        asynStatus setParam32(int functionId, epicsUInt32 mask, int addr);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask, int addr);
        asynStatus setParamInteger(int functionId, int addr);
        asynStatus getParamInteger(int functionId, epicsInt32 *param, int addr);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);

        /* Specific hardware functions that need extra processing and don't
         * fit into the general set/get template */
        asynStatus setBPMMode(int addr, int function);
        asynStatus setDataTrigChan(epicsUInt32 mask, int addr);
        asynStatus setAdcClkSel(epicsUInt32 mask, int addr);
        asynStatus setAdcAD9510ClkSel(epicsUInt32 mask, int addr);
        asynStatus getDataTrigChan(epicsUInt32 *channel, epicsUInt32 mask, int addr);
        asynStatus setAdcReg(epicsUInt32 mask, int addr);
        asynStatus getAdcReg(epicsUInt32 *data, epicsUInt32 mask, int addr);
        asynStatus resetAdcMMCM(epicsUInt32 mask, int addr);
        asynStatus setSi57xFreq(int addr);
        asynStatus resetAD9510AndADCs(epicsUInt32 mask, int addr);
        asynStatus resetAD9510(epicsUInt32 mask, int addr);
        asynStatus resetADCs(epicsUInt32 mask, int addr);
        asynStatus updateUInt32Params(epicsUInt32 mask, int addr, int firstParam,
                int lastParam, bool acceptErrors);
        asynStatus updateIntegerParams(int addr, int firstParam,
                int lastParam, bool acceptErrors);
        asynStatus updateDoubleParams(int addr, int firstParam, int lastParam,
                bool acceptErrors);
        asynStatus readUInt32Params(epicsUInt32 mask, int addr);
        asynStatus readIntegerParams(int addr);
        asynStatus readFloat64Params(int addr);
        asynStatus readAD9510AndADCsParams(epicsUInt32 mask, int addr);
        asynStatus readAD9510Params(epicsUInt32 mask, int addr);
        asynStatus readADCsParams(epicsUInt32 mask, int addr);
        asynStatus readOffsetParams(int addr);
        asynStatus readSi57xParams(int addr);
        asynStatus readTriggerParams(epicsUInt32 mask, int addr);
        asynStatus readFMCPicoParams(epicsUInt32 mask, int addr);
        asynStatus readGenParams(epicsUInt32 mask, int addr);
        asynStatus readDSPParams(epicsUInt32 mask, int addr);
        asynStatus readGenDSPParams(epicsUInt32 mask, int addr);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)

/********************************************************************/
/*************** fucntionsAny_t template functions ******************/
/********************************************************************/

/* Read function for Hw execution */
template<typename T>
asynStatus functionsAny_t::executeHwReadFunction(const drvBPM& drvBPM,
        const any& functionFp, char *service,
    int addr, functionsArgs_t &functionParam)
{
    if(!any_cast<T>(functionFp).read) {
        return asynSuccess;
    }
    auto functionFpCast = any_cast<T>(functionFp);
    return drvBPM.doExecuteHwReadFunction(functionFpCast, service, addr, functionParam);
}

/* Write function for Hw execution */
template<typename T>
asynStatus functionsAny_t::executeHwWriteFunction(const drvBPM& drvBPM,
        const any& functionFp, char *service,
    int addr, functionsArgs_t &functionParam)
{
    if(!any_cast<T>(functionFp).write) {
        return asynSuccess;
    }
    auto functionFpCast = any_cast<T>(functionFp);
    return drvBPM.doExecuteHwWriteFunction(functionFpCast, service, addr, functionParam);
}

/* Service name utilities */
template<typename T>
const char *functionsAny_t::getServiceNameFromFunc(const drvBPM& drvBPM,
        const any& functionFp) const
{
    auto functionFpCast = any_cast<T>(functionFp);
    return drvBPM.doGetServiceNameFromFunc(functionFpCast);
}
