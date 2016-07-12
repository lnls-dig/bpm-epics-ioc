/*
 * drvBPMRFFE.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Mar. 02, 2016
 */

#include "asynPortDriver.h"
#include <epicsExit.h>
#include <epicsMutex.h>
/* Third-party libraries */
#include <unordered_map>
#include <bpm_client.h>

#define ARRAY_SIZE(ARRAY)           (sizeof(ARRAY)/sizeof((ARRAY)[0]))

#define MAX_SLOTS                   12
#define MAX_BPM_PER_SLOT            2
#define MAX_BPMS                    (MAX_SLOTS*MAX_BPM_PER_SLOT)

#define BPM_NUMBER_MIN              1
#define BPM_NUMBER_MAX              MAX_BPMS

#define MAX_ADDR                    1

/* BPM Mappping structure */
typedef struct {
    int board;
    int bpm;
} boardMap_t;

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

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_RffeAttString              "RFFE_ATT"              /* asynFloat64,        r/w */
#define P_RffeTempACString           "RFFE_TEMP_AC"          /* asynfloat64         r/w */
#define P_RffeTempBDString           "RFFE_TEMP_BD"          /* asynfloat64         r/w */
#define P_RffeSetPointACString       "RFFE_SET_POINT_AC"     /* asynfloat64         r/w */
#define P_RffeSetPointBDString       "RFFE_SET_POINT_BD"     /* asynfloat64         r/w */
#define P_RffeTempCtlString          "RFFE_TEMP_CTL"         /* asynInt32           r/w */
#define P_RffeHeaterACString         "RFFE_HEATER_AC"        /* asynfloat64         r/w */
#define P_RffeHeaterBDString         "RFFE_HEATER_BD"        /* asynfloat64         r/w */
#define P_RffeRstString              "RFFE_RST"              /* asynInt32           r/w */
#if 0
#define P_RffeReprogString           "RFFE_REPROG"           /* asynInt32           r/w */
#define P_RffeDataString             "RFFE_DATA"             /* asynInt32           r/w */
#define P_RffeVersionString          "RFFE_VERSION"          /* waveform            r/w */
#endif
#define P_RffePidACKpString          "RFFE_PID_AC_KP"        /* asynfloat64         r/w */
#define P_RffePidACTiString          "RFFE_PID_AC_TI"        /* asynFloat64         r/w */
#define P_RffePidACTdString          "RFFE_PID_AC_TD"        /* asynFloat64         r/w */
#define P_RffePidBDKpString          "RFFE_PID_BD_KP"        /* asynfloat64         r/w */
#define P_RffePidBDTiString          "RFFE_PID_BD_TI"        /* asynFloat64         r/w */
#define P_RffePidBDTdString          "RFFE_PID_BD_TD"        /* asynFloat64         r/w */


class drvBPMRFFE : public asynPortDriver {
    public:
        drvBPMRFFE(const char *portName, const char *endpoint,
                int bpmNumber, int verbose, int timeout);
        ~drvBPMRFFE();

        /* These are the methods that we override from asynPortDriver */
        virtual asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
                epicsUInt32 mask);
        virtual asynStatus readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
                epicsUInt32 mask);
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

        /* These methods are overwritten from asynPortDriver */
        virtual asynStatus connect(asynUser* pasynUser);
        virtual asynStatus disconnect(asynUser* pasynUser);

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_RffeAtt;
#define FIRST_COMMAND P_RffeAtt
        int P_RffeTempAC;
        int P_RffeTempBD;
        int P_RffeSetPointAC;
        int P_RffeSetPointBD;
        int P_RffeTempCtl;
        int P_RffeHeaterAC;
        int P_RffeHeaterBD;
        int P_RffeRst;
#if 0
        int P_RffeReprog;
        int P_RffeData;
        int P_RffeVersion;
#endif
        int P_RffePidACKp;
        int P_RffePidACTi;
        int P_RffePidACTd;
        int P_RffePidBDKp;
        int P_RffePidBDTi;
        int P_RffePidBDTd;
#define LAST_COMMAND P_RffePidBDTd

    private:
        /* Our data */
        bpm_client_t *bpmClientRFFE;
        char *endpoint;
        int bpmNumber;
        int verbose;
        int timeout;
        char *bpmPortName;
        std::unordered_map<int, functionsFloat64_t> bpmHwFloat64Func;
        std::unordered_map<int, functionsInt32_t> bpmHwInt32Func;

        /* Our private methods */
        asynStatus bpmClientConnect(void);
        asynStatus bpmClientDisconnect(void);
        asynStatus setParam32(int functionId, epicsUInt32 mask, int addr);
        asynStatus getParam32(int functionId, epicsUInt32 *param,
                epicsUInt32 mask, int addr);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)
