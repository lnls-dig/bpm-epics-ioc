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
#include <bpm_client.h>

#define ARRAY_SIZE(ARRAY)           (sizeof(ARRAY)/sizeof((ARRAY)[0]))
/* Waveforms: RAW data, ADC SWAP data, TBT Amp, TBT Phase, FOFB Amp, FOFB Phase */
#define MAX_ARRAY_POINTS            200000
#define BPM_TIMEOUT                 1.0

/* BPM acquisition status */
typedef enum {
    BPMStatusIdle = 0,
    BPMStatusWaiting,
    BPMStatusTriggerHwExtWaiting,
    BPMStatusTriggerHwDataWaiting,
    BPMStatusTriggerSwWaiting,
    BPMStatusAcquire,
} bpm_status_types;

/* Waveform IDs */
typedef enum {
    WVF_ADC_A = 0,
    WVF_ADC_B,
    WVF_ADC_C,
    WVF_ADC_D,
    WVF_ADC_ALL,
    WVF_ADCSWAP_A,
    WVF_ADCSWAP_B,
    WVF_ADCSWAP_C,
    WVF_ADCSWAP_D,
    WVF_ADCSWAP_ALL,
    WVF_TBTAMP_A,
    WVF_TBTAMP_B,
    WVF_TBTAMP_C,
    WVF_TBTAMP_D,
    WVF_TBTAMP_ALL,
    WVF_TBTPOS_A,
    WVF_TBTPOS_B,
    WVF_TBTPOS_C,
    WVF_TBTPOS_D,
    WVF_TBTPOS_ALL,
    WVF_TBTPHASE_A,
    WVF_TBTPHASE_B,
    WVF_TBTPHASE_C,
    WVF_TBTPHASE_D,
    WVF_TBTPHASE_ALL,
    WVF_FOFBAMP_A,
    WVF_FOFBAMP_B,
    WVF_FOFBAMP_C,
    WVF_FOFBAMP_D,
    WVF_FOFBAMP_ALL,
    WVF_FOFBPOS_A,
    WVF_FOFBPOS_B,
    WVF_FOFBPOS_C,
    WVF_FOFBPOS_D,
    WVF_FOFBPOS_ALL,
    WVF_FOFBPHASE_A,
    WVF_FOFBPHASE_B,
    WVF_FOFBPHASE_C,
    WVF_FOFBPHASE_D,
    WVF_FOFBPHASE_ALL,
    WVF_END
} wvf_types;

#define MAX_WAVEFORMS               WVF_END
#define MAX_ADDR                    MAX_WAVEFORMS

/* Channel IDs */
typedef enum {
    CH_ADC = 0,
    CH_ADCSWAP,
    CH_TBT,
    CH_FOFB,
    CH_END
} ch_types;

#define MAX_CHANNELS                CH_END

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

/* BPM Mappping structure */
typedef struct {
    int board;
    int bpm;
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
    int NDArrayAmp[MAX_WVF_AMP_TYPES];
    int NDArrayPhase[MAX_WVF_PHA_TYPES];
    int NDArrayPos[MAX_WVF_POS_TYPES];
} channelMap_t;

/* Write 32-bit function pointer */
typedef bpm_client_err_e (*writeInt32Fp)(bpm_client_t *self, char *service,
	uint32_t param);
/* Read 32-bit function pointer */
typedef bpm_client_err_e (*readInt32Fp)(bpm_client_t *self, char *service,
	uint32_t *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeInt32Fp write;
    readInt32Fp read;
} functionsInt32_t;

/* Write 2 32-bit function pointer */
typedef bpm_client_err_e (*write2Int32Fp)(bpm_client_t *self, char *service,
	uint32_t param1, uint32_t param2);
/* Read 32-bit function pointer */
typedef bpm_client_err_e (*read2Int32Fp)(bpm_client_t *self, char *service,
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
typedef bpm_client_err_e (*writeFloat64Fp)(bpm_client_t *self, char *service,
	double param);
/* Read 32-bit function pointer */
typedef bpm_client_err_e (*readFloat64Fp)(bpm_client_t *self, char *service,
	double *param);

/* BPM command dispatch table */
typedef struct {
    const char *serviceName;
    writeFloat64Fp write;
    readFloat64Fp read;
} functionsFloat64_t;

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_HarmonicNumberString      "INFO_HARMNUMB"         /* asynUInt32Digital,      r/o */
#define P_TbtRateString             "INFO_TBTRATE"          /* asynUInt32Digital,      r/o */
#define P_FofbRateString            "INFO_FOFBRATE"         /* asynUInt32Digital,      r/o */
#define P_MonitRateString           "INFO_MONITRATE"        /* asynUInt32Digital,      r/o */
#define P_RffeSwString              "RFFE_SW"               /* asynUInt32Digital,      r/w */
#define P_RffeAtt1String            "RFFE_ATT1"             /* asynFloat64,            r/w */
#define P_RffeAtt2String            "RFFE_ATT2"             /* asynFloat64,            r/w */
#define P_RffeTemp1String           "RFFE_TEMP1"            /* asynfloat64             r/w */
#define P_RffeTemp2String           "RFFE_TEMP2"            /* asynfloat64             r/w */
#define P_RffeTemp3String           "RFFE_TEMP3"            /* asynfloat64             r/w */
#define P_RffeTemp4String           "RFFE_TEMP4"            /* asynfloat64             r/w */
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
#define P_UpdateTimeString          "ACQ_UPDATE_TIME"       /* asynFloat64,            r/w */
#define P_TriggerDataThresString    "ACQ_TRIGGER_THRES"     /* asynInt32,              r/w */
#define P_TriggerDataPolString      "ACQ_TRIGGER_POL"       /* asynInt32,              r/w */
#define P_TriggerDataSelString      "ACQ_TRIGGER_SEL"       /* asynInt32,              r/w */
#define P_TriggerDataFiltString     "ACQ_TRIGGER_FILT"      /* asynInt32,              r/w */
#define P_TriggerHwDlyString        "ACQ_TRIGGER_HWDLY"     /* asynInt32,              r/w */
#define P_MonitAmpAString           "MONITAMP_A"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpBString           "MONITAMP_B"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpCString           "MONITAMP_C"            /* asynUInt32Digital,      r/o */
#define P_MonitAmpDString           "MONITAMP_D"            /* asynUInt32Digital,      r/o */
#define P_MonitPosAString           "MONITPOS_A"            /* asynUInt32Digital,      r/o */
#define P_MonitPosBString           "MONITPOS_B"            /* asynUInt32Digital,      r/o */
#define P_MonitPosCString           "MONITPOS_C"            /* asynUInt32Digital,      r/o */
#define P_MonitPosDString           "MONITPOS_D"            /* asynUInt32Digital,      r/o */
#define P_MonitUpdtString           "MONIT_UPDT"            /* asynUInt32Digital,      r/w */

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
    TRIG_ACQ_REPETITIVE
} trigEnum_t;

class drvBPM : public asynNDArrayDriver {
    public:
        drvBPM(const char *portName, const char *endpoint,
                int bpmNumber, int verbose, uint32_t timeout);
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
#if 0
        virtual asynStatus readInt16Array(asynUser *pasynUser, epicsInt16 *value,
                size_t nElements, size_t *nIn);

        /* These methods are overwritten from asynPortDriver */
        virtual asynStatus connect(asynUser* pasynUser);
        virtual asynStatus disconnect(asynUser* pasynUser);
#endif

        /* These are the methods that are new to this class */
        void acqTask(void);

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_HarmonicNumber;
#define FIRST_COMMAND P_HarmonicNumber
        int P_TbtRate;
        int P_FofbRate;
        int P_MonitRate;
        int P_BPMStatus;
        int P_CompMethod;
        int P_RffeSw;
        int P_RffeAtt1;
        int P_RffeAtt2;
        int P_RffeTemp1;
        int P_RffeTemp2;
        int P_RffeTemp3;
        int P_RffeTemp4;
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
        int P_UpdateTime;
        int P_Trigger;
        int P_TriggerDataThres;
        int P_TriggerDataPol;
        int P_TriggerDataSel;
        int P_TriggerDataFilt;
        int P_TriggerHwDly;
        int P_MonitAmpA;
        int P_MonitAmpB;
        int P_MonitAmpC;
        int P_MonitAmpD;
        int P_MonitPosA;
        int P_MonitPosB;
        int P_MonitPosC;
        int P_MonitPosD;
        int P_MonitUpdt;
#define LAST_COMMAND P_MonitUpdt

    private:
        /* Our data */
        bpm_client_t *bpmClient;
        char *endpoint;
        int bpmNumber;
        int verbose;
        uint32_t timeout;
        char *bpmPortName;
        int readingActive;
        int repetitiveTrigger;
        epicsEventId startAcqEventId;
        epicsEventId stopAcqEventId;
        std::unordered_map<int, functionsInt32_t> bpmHwInt32Func;
        std::unordered_map<int, functions2Int32_t> bpmHw2Int32Func;
        std::unordered_map<int, functionsFloat64_t> bpmHwFloat64Func;

        /* Our private methods */
        asynStatus bpmClientConnect(void);
        asynStatus bpmClientDisconnect(void);
        asynStatus setParam32(int functionId, epicsUInt32 mask);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask);
        asynStatus setParamDouble(int functionId);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param);
        asynStatus setAcquire();
        asynStatus getAcqNDArrayType(int channel, NDDataType_t *NDType);
        asynStatus startAcq(int hwChannel, epicsUInt32 num_samples_pre,
                epicsUInt32 num_samples_post, epicsUInt32 num_shots);
        int checkAcqCompletion();
        asynStatus getAcqCurve(NDArray *pArrayAllChannels, int hwChannel,
                epicsUInt32 num_samples_pre, epicsUInt32 num_samples_post,
                epicsUInt32 num_shots);
        void deinterleaveNDArray (NDArray *pArrayAllChannels, const int *pNDArrayAddr,
                int pNDArrayAddrSize, int arrayCounter, epicsFloat64 timeStamp);
        void computePositions(NDArray *pArrayAllChannels, int channel);
        template <typename epicsType>
            asynStatus doReadArray(asynUser *pasynUser, epicsType *value,
                    size_t nElements, size_t *nIn, epicsType *pValue);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)
