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
/* Variable macros */
#include "varg_macros.h"

#define ARRAY_SIZE(ARRAY)           (sizeof(ARRAY)/sizeof((ARRAY)[0]))
/* Waveforms: RAW data, ADC SWAP data, TBT Amp, TBT Phase, FOFB Amp, FOFB Phase */
#define MAX_ARRAY_POINTS            200000
#define BPM_TIMEOUT                 1.0

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
} bpm_status_types;

/* Waveform IDs */
typedef enum {
    WVF_ADC_A = 0,
    WVF_ADC_B,
    WVF_ADC_C,
    WVF_ADC_D,
    WVF_ADC_ALL,
    WVF_ADC_FREQ,
    WVF_ADCSWAP_A,
    WVF_ADCSWAP_B,
    WVF_ADCSWAP_C,
    WVF_ADCSWAP_D,
    WVF_ADCSWAP_ALL,
    WVF_ADCSWAP_FREQ,
    WVF_TBTAMP_A,
    WVF_TBTAMP_B,
    WVF_TBTAMP_C,
    WVF_TBTAMP_D,
    WVF_TBTAMP_ALL,
    WVF_TBTAMP_FREQ,
    WVF_TBTPOS_A,
    WVF_TBTPOS_B,
    WVF_TBTPOS_C,
    WVF_TBTPOS_D,
    WVF_TBTPOS_ALL,
    WVF_TBTPOS_FREQ,
    WVF_TBTPHASE_A,
    WVF_TBTPHASE_B,
    WVF_TBTPHASE_C,
    WVF_TBTPHASE_D,
    WVF_TBTPHASE_ALL,
    WVF_TBTPHASE_FREQ,
    WVF_FOFBAMP_A,
    WVF_FOFBAMP_B,
    WVF_FOFBAMP_C,
    WVF_FOFBAMP_D,
    WVF_FOFBAMP_ALL,
    WVF_FOFBAMP_FREQ,
    WVF_FOFBPOS_A,
    WVF_FOFBPOS_B,
    WVF_FOFBPOS_C,
    WVF_FOFBPOS_D,
    WVF_FOFBPOS_ALL,
    WVF_FOFBPOS_FREQ,
    WVF_FOFBPHASE_A,
    WVF_FOFBPHASE_B,
    WVF_FOFBPHASE_C,
    WVF_FOFBPHASE_D,
    WVF_FOFBPHASE_ALL,
    WVF_FOFBPHASE_FREQ,
    WVF_AMP_PM_A,
    WVF_AMP_PM_B,
    WVF_AMP_PM_C,
    WVF_AMP_PM_D,
    WVF_AMP_PM_ALL,
    WVF_AMP_PM_FREQ,
    WVF_POS_PM_A,
    WVF_POS_PM_B,
    WVF_POS_PM_C,
    WVF_POS_PM_D,
    WVF_POS_PM_ALL,
    WVF_POS_PM_FREQ,
    WVF_AMP_SP_A,
    WVF_AMP_SP_B,
    WVF_AMP_SP_C,
    WVF_AMP_SP_D,
    WVF_AMP_SP_ALL,
    WVF_AMP_SP_FREQ,
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
#define MAX_ADDR                    IF(MAX_WAVEFORMS > MAX_TRIGGERS_ALL_ACQ)(MAX_WAVEFORMS, MAX_TRIGGERS_ALL_ACQ)

/* Channel IDs */
typedef enum {
    CH_ADC = 0,
    CH_ADCSWAP = 1,
    CH_TBT = 2,
    CH_FOFB = 3,
    CH_SP = 4,
    CH_END
} ch_types;

#define MAX_CHANNELS                CH_END

typedef enum {
    CH_HW_ADC = 0,
    CH_HW_ADCSWAP = 1,
    CH_HW_TBT = 6,
    CH_HW_FOFB = 11,
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
#define POINTS_PER_SAMPLE           4

#define MAX_SLOTS                   12
#define MAX_BPM_PER_SLOT            2
#define MAX_BPMS                    (MAX_SLOTS*MAX_BPM_PER_SLOT)

#define BPM_NUMBER_MIN              1
#define BPM_NUMBER_MAX              MAX_BPMS

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
    int NDArrayAmpFreq[NUM_ACQ_CORES_PER_BPM];
    int NDArrayPhase[NUM_ACQ_CORES_PER_BPM][MAX_WVF_PHA_TYPES];
    int NDArrayPhaseFreq[NUM_ACQ_CORES_PER_BPM];
    int NDArrayPos[NUM_ACQ_CORES_PER_BPM][MAX_WVF_POS_TYPES];
    int NDArrayPosFreq[NUM_ACQ_CORES_PER_BPM];
} channelMap_t;

/* BPM Reverse channel mapping structure */
typedef struct {
    /* EPICS channel. -1 means not available */
    int epicsChannel;
} channelRevMap_t;

/* Write 32-bit function pointer */
typedef halcs_client_err_e (*writeInt32Fp)(halcs_client_t *self, char *service,
	uint32_t param);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*readInt32Fp)(halcs_client_t *self, char *service,
	uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32Fp write;
    readInt32Fp read;
} functionsInt32_t;

/* Write 32-bit function pointer with acq_client structure */
typedef halcs_client_err_e (*writeInt32AcqFp)(acq_client_t *self, char *service,
	uint32_t param);
/* Read 32-bit function pointer with acq_client structure */
typedef halcs_client_err_e (*readInt32AcqFp)(acq_client_t *self, char *service,
	uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32AcqFp write;
    readInt32AcqFp read;
} functionsInt32Acq_t;

/* Write 2 32-bit function pointer */
typedef halcs_client_err_e (*write2Int32Fp)(halcs_client_t *self, char *service,
	uint32_t param1, uint32_t param2);
/* Read 32-bit function pointer */
typedef halcs_client_err_e (*read2Int32Fp)(halcs_client_t *self, char *service,
	uint32_t *param1, uint32_t *param2);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    write2Int32Fp write;
    read2Int32Fp read;
    /* Which parameter (first or second) would trigger this function to be
     * executed on hardware (the other one won't be changed) */
    int parameterPos;
} functions2Int32_t;

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
typedef halcs_client_err_e (*writeInt32ChanFp)(halcs_client_t *self, char *service,
	uint32_t chan, uint32_t param);
/* Read 32-bit function pointer with channel selection */
typedef halcs_client_err_e (*readInt32ChanFp)(halcs_client_t *self, char *service,
	uint32_t chan, uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32ChanFp write;
    readInt32ChanFp read;
} functionsInt32Chan_t;

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_BPMModeString             "BPM_MODE"              /* asynInt32,              r/w */
#define P_HarmonicNumberString      "INFO_HARMNUMB"         /* asynUInt32Digital,      r/o */
#define P_AdcClkFreqString          "INFO_ADCCLKFREQ"          /* asynUInt32Digital,      r/o */
#define P_TbtRateString             "INFO_TBTRATE"          /* asynUInt32Digital,      r/o */
#define P_FofbRateString            "INFO_FOFBRATE"         /* asynUInt32Digital,      r/o */
#define P_MonitRateString           "INFO_MONITRATE"        /* asynUInt32Digital,      r/o */
#define P_SwString                  "ADC_SW"                /* asynUInt32Digital,      r/w */
#define P_SwDlyString               "ADC_SWDLY"             /* asynUInt32Digital,      r/w */
#define P_SwDivClkString            "ADC_SW_DIVCLK"         /* asynUInt32Digital,      r/w */
#define P_SwEnString                "ADC_SWEN"              /* asynUInt32Digital,      r/w */
#define P_WdwString                 "ADC_WDW"               /* asynUInt32Digital,      r/w */
#define P_WdwDlyString              "ADC_WDWDLY"            /* asynfloat64             r/w */
#define P_CompMethodString          "ADC_COMPMETHOD"        /* asynInt32,              r/w */
#define P_GainAAString              "ADC_GAINAA"            /* asynUInt32Digital,      r/w */
#define P_GainBBString              "ADC_GAINBB"            /* asynUInt32Digital,      r/w */
#define P_GainCCString              "ADC_GAINCC"            /* asynUInt32Digital,      r/w */
#define P_GainDDString              "ADC_GAINDD"            /* asynUInt32Digital,      r/w */
#define P_GainACString              "ADC_GAINAC"            /* asynUInt32Digital,      r/w */
#define P_GainCAString              "ADC_GAINCA"            /* asynUInt32Digital,      r/w */
#define P_GainBDString              "ADC_GAINBD"            /* asynUInt32Digital,      r/w */
#define P_GainDBString              "ADC_GAINDB"            /* asynUInt32Digital,      r/w */
#define P_AdcTrigDirString          "ADC_TRIGDIR"           /* asynUInt32Digital,      r/w */
#define P_AdcTrigTermString         "ADC_TRIGTERM"          /* asynUInt32Digital,      r/w */
#define P_AdcRandString             "ADC_RAND"              /* asynUInt32Digital,      r/w */
#define P_AdcDithString             "ADC_DITH"              /* asynUInt32Digital,      r/w */
#define P_AdcShdnString             "ADC_SHDN"              /* asynUInt32Digital,      r/w */
#define P_AdcPgaString              "ADC_PGA"               /* asynUInt32Digital,      r/w */
#define P_AdcTestDataString         "ADC_TESTDATA"          /* asynUInt32Digital,      r/w */
#define P_AdcClkSelString           "ADC_CLKSEL"            /* asynUInt32Digital,      r/w */
#define P_AdcSi57xFreqString        "ADC_SI57XFREQ"         /* asynUInt32Digital,      w/o */
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
#define P_KxString                  "DSP_KX"                /* asynUInt32Digital,      r/w */
#define P_KyString                  "DSP_KY"                /* asynUInt32Digital,      r/w */
#define P_KqString                  "DSP_KQ"                /* asynUInt32Digital,      r/w */
#define P_KsumString                "DSP_KSUM"              /* asynUInt32Digital,      r/w */
#define P_XOffsetString             "DSP_XOFFSET"           /* asynUInt32Digital,      r/w */
#define P_YOffsetString             "DSP_YOFFSET"           /* asynUInt32Digital,      r/w */
#define P_QOffsetString             "DSP_QOFFSET"           /* asynUInt32Digital,      r/w */
#define P_SamplesPreString          "ACQ_SAMPLES_PRE"       /* asynUInt32Digital,      r/w */
#define P_SamplesPostString         "ACQ_SAMPLES_POST"      /* asynUInt32Digital,      r/w */
#define P_NumShotsString            "ACQ_NUM_SHOTS"         /* asynUInt32Digital,      r/w */
#define P_ChannelString             "ACQ_CHANNEL"           /* asynInt32,              r/w */
#define P_TriggerString             "ACQ_TRIGGER"           /* asynInt32,              r/w */
#define P_BPMStatusString           "ACQ_STATUS"            /* asynInt32,              r/o */
#define P_AcqControlString          "ACQ_CONTROL"           /* asynInt32,              r/w */
#define P_UpdateTimeString          "ACQ_UPDATE_TIME"       /* asynFloat64,            r/w */
#define P_TriggerDataThresString    "ACQ_TRIGGER_THRES"     /* asynInt32,              r/w */
#define P_TriggerDataPolString      "ACQ_TRIGGER_POL"       /* asynInt32,              r/w */
#define P_TriggerDataSelString      "ACQ_TRIGGER_SEL"       /* asynInt32,              r/w */
#define P_TriggerDataFiltString     "ACQ_TRIGGER_FILT"      /* asynInt32,              r/w */
#define P_TriggerHwDlyString        "ACQ_TRIGGER_HWDLY"     /* asynInt32,              r/w */
#define P_DataTrigChanString        "ACQ_DATA_TRIG_CHAN"    /* asynuint32digital,      r/w */
#define P_MonitAmpAString           "MONITAMP_A"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpBString           "MONITAMP_B"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpCString           "MONITAMP_C"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpDString           "MONITAMP_D"            /* asynUInt32Digital,      r/o */
#define P_MonitUpdtString           "MONIT_UPDT"            /* asynUInt32Digital,      r/w */
#define P_AdcTestModeString         "ADC_TEST_MODE"         /* asynUInt32Digital,      r/w */
#define P_AdcRstModesString         "ADC_RST_MODES"         /* asynUInt32Digital,      r/w */
#define P_AdcRegReadString          "ADC_REG_READ"          /* asynUInt32Digital,      r/w */
#define P_AdcRegReadDataString      "ADC_REG_READ_DATA"     /* asynUInt32Digital,      r/o */
#define P_AdcRegReadAddrString      "ADC_REG_READ_ADDR"     /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteString         "ADC_REG_WRITE"         /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteDataString     "ADC_REG_WRITE_DATA"    /* asynUInt32Digital,      r/w */
#define P_AdcRegWriteAddrString     "ADC_REG_WRITE_ADDR"    /* asynUInt32Digital,      r/w */
#define P_AdcTempString             "ADC_TEMP"              /* asynUInt32Digital,      r/w */
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

/* Single Pass parameters */
#define P_SPAmpAString              "SP_AMP_A"              /* asynFloat64,            r/o */
#define P_SPAmpBString              "SP_AMP_B"              /* asynFloat64,            r/o */
#define P_SPAmpCString              "SP_AMP_C"              /* asynFloat64,            r/o */
#define P_SPAmpDString              "SP_AMP_D"              /* asynFloat64,            r/o */
#define P_SPPosXString              "SP_POS_X"              /* asynFloat64,            r/o */
#define P_SPPosYString              "SP_POS_Y"              /* asynFloat64,            r/o */
#define P_SPPosQString              "SP_POS_Q"              /* asynFloat64,            r/o */
#define P_SPPosSumString            "SP_POS_SUM"            /* asynFloat64,            r/o */

typedef enum {
    /* These trigger types matches the HW */
    TRIG_ACQ_NOW = 0,
    TRIG_ACQ_EXT_HW,
    TRIG_ACQ_EXT_DATA,
    TRIG_ACQ_SW,
    /* These trigger types do not exist in HW */
    /* FIXME: TRIG_ACQ_STOP must be after the valid HW
     * triggers */
    TRIG_ACQ_STOP,
    TRIG_ACQ_ABORT,
    TRIG_ACQ_REPETITIVE
} trigEnum_t;

class drvBPM : public asynNDArrayDriver {
    public:
        drvBPM(const char *portName, const char *endpoint,
                int bpmNumber, int verbose, int timeout);
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

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_BPMMode;
#define FIRST_COMMAND P_BPMMode
        int P_HarmonicNumber;
        int P_AdcClkFreq;
        int P_TbtRate;
        int P_FofbRate;
        int P_MonitRate;
        int P_BPMStatus;
        int P_CompMethod;
        int P_Sw;
        int P_SwDly;
        int P_SwEn;
        int P_SwDivClk;
        int P_Wdw;
        int P_WdwDly;
        int P_GainAA;
        int P_GainBB;
        int P_GainCC;
        int P_GainDD;
        int P_GainAC;
        int P_GainCA;
        int P_GainBD;
        int P_GainDB;
        int P_AdcTrigDir;
        int P_AdcTrigTerm;
        int P_AdcRand;
        int P_AdcDith;
        int P_AdcShdn;
        int P_AdcPga;
        int P_AdcTestData;
        int P_AdcClkSel;
        int P_AdcSi57xFreq;
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
        int P_Kx;
        int P_Ky;
        int P_Kq;
        int P_Ksum;
        int P_XOffset;
        int P_YOffset;
        int P_QOffset;
        int P_SamplesPre;
        int P_SamplesPost;
        int P_NumShots;
        int P_Channel;
        int P_AcqControl;
        int P_UpdateTime;
        int P_Trigger;
        int P_TriggerDataThres;
        int P_TriggerDataPol;
        int P_TriggerDataSel;
        int P_TriggerDataFilt;
        int P_TriggerHwDly;
        int P_DataTrigChan;
        int P_MonitAmpA;
        int P_MonitAmpB;
        int P_MonitAmpC;
        int P_MonitAmpD;
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
        int P_AdcRegRead;
        int P_AdcRegReadData;
        int P_AdcRegReadAddr;
        int P_AdcRegWrite;
        int P_AdcRegWriteData;
        int P_AdcRegWriteAddr;
        int P_AdcTemp;
        int P_MonitUpdt;
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
#define LAST_COMMAND P_TriggerTrnOutSel

    private:
        /* Our data */
        halcs_client_t *bpmClient;
        acq_client_t *bpmClientAcqParam[NUM_ACQ_CORES_PER_BPM];
        acq_client_t *bpmClientAcq[NUM_ACQ_CORES_PER_BPM];
        bpm_single_pass_t *bpmSinglePass[NUM_ACQ_CORES_PER_BPM];
        char *endpoint;
        int bpmNumber;
        int verbose;
        int timeout;
        char *bpmPortName;
        int readingActive[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        int repetitiveTrigger[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId startAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId stopAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId abortAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        epicsEventId activeAcqEventId[NUM_BPM_MODES][NUM_ACQ_CORES_PER_BPM];
        std::unordered_map<int, functionsInt32_t> bpmHwInt32Func;
        std::unordered_map<int, functionsInt32Acq_t> bpmHwInt32AcqFunc;
        std::unordered_map<int, functions2Int32_t> bpmHw2Int32Func;
        std::unordered_map<int, functionsFloat64_t> bpmHwFloat64Func;
        std::unordered_map<int, functionsInt32Chan_t> bpmHwInt32ChanFunc;

        /* Our private methods */
        asynStatus bpmClientConnect(void);
        asynStatus bpmClientDisconnect(void);
        asynStatus getServiceChan (int bpmNumber, int addr, const char *serviceName,
                epicsUInt32 *chanArg);
        asynStatus getServiceID (int bpmNumber, int addr, const char *serviceName,
                int *serviceIDArg);
        asynStatus getFullServiceName (int bpmNumber, int addr, const char *serviceName,
                char *fullServiceName, int fullServiceNameSize);
        asynStatus setAcquire(int addr);
        asynStatus getAcqNDArrayType(int coreID, int channel, NDDataType_t *NDType);
        bpm_status_types getBPMInitAcqStatus(int coreID);
        asynStatus startAcq(int coreID, int hwChannel, epicsUInt32 num_samples_pre,
                epicsUInt32 num_samples_post, epicsUInt32 num_shots);
        asynStatus startSPAcq(bpm_single_pass_t *bpm_single_pass);
        asynStatus setAcqTrig(int coreID, acq_client_trig_e trig);
        asynStatus initAcqPM(int coreID);
        asynStatus abortAcq(int coreID);
        asynStatus abortAcqTask(int addr, int bpmMode);
        int checkAcqCompletion(int coreID);
        int checkSPAcqCompletion(bpm_single_pass_t *bpm_single_pass);
        asynStatus getAcqCurve(int coreID, NDArray *pArrayAllChannels, int hwChannel,
                epicsUInt32 num_samples_pre, epicsUInt32 num_samples_post,
                epicsUInt32 num_shots);
        asynStatus getAcqSPCurve(bpm_single_pass_t *bpm_single_pass, NDArray *pArrayAllChannels);
        asynStatus getAcqSPSamples(bpm_single_pass_t *bpm_single_pass, bpm_sample_t *bpm_sample);
        void computeFreqArray(int coreID, NDArray *pArrayChannelFreq, int channel,
                epicsFloat64 adcFreq, epicsUInt32 num_samples_pre,
                epicsUInt32 num_samples_post, epicsUInt32 num_shots);
        void deinterleaveNDArray (NDArray *pArrayAllChannels, const int *pNDArrayAddr,
                int pNDArrayAddrSize, int arrayCounter, epicsFloat64 timeStamp);
        void computePositions(int coreID, NDArray *pArrayAllChannels, int channel);
        asynStatus setParam32(int functionId, epicsUInt32 mask, int addr);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask, int addr);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);
        asynStatus setDataTrigChan(epicsUInt32 mask, int addr);
        asynStatus getDataTrigChan(epicsUInt32 *channel, epicsUInt32 mask, int addr);
        asynStatus setAdcReg(epicsUInt32 mask, int addr);
        asynStatus getAdcReg(epicsUInt32 *data, epicsUInt32 mask, int addr);
        asynStatus setSi57xFreq(int addr);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)
