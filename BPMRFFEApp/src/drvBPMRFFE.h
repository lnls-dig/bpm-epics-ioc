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

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_RffeAtt1String            "RFFE_ATT1"             /* asynFloat64,            r/w */
#define P_RffeAtt2String            "RFFE_ATT2"             /* asynFloat64,            r/w */
#define P_RffeTemp1String           "RFFE_TEMP1"            /* asynfloat64             r/w */
#define P_RffeTemp2String           "RFFE_TEMP2"            /* asynfloat64             r/w */
#define P_RffeTemp3String           "RFFE_TEMP3"            /* asynfloat64             r/w */
#define P_RffeTemp4String           "RFFE_TEMP4"            /* asynfloat64             r/w */

class drvBPMRFFE : public asynPortDriver {
    public:
        drvBPMRFFE(const char *portName, const char *endpoint,
                int bpmNumber, int verbose, int timeout);
        ~drvBPMRFFE();

        /* These are the methods that we override from asynPortDriver */
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);

        /* These methods are overwritten from asynPortDriver */
        virtual asynStatus connect(asynUser* pasynUser);
        virtual asynStatus disconnect(asynUser* pasynUser);

    protected:
        /** Values used for pasynUser->reason, and indexes into the parameter library. */
        int P_RffeAtt1;
#define FIRST_COMMAND P_RffeAtt1
        int P_RffeAtt2;
        int P_RffeTemp1;
        int P_RffeTemp2;
        int P_RffeTemp3;
        int P_RffeTemp4;
#define LAST_COMMAND P_RffeTemp4

    private:
        /* Our data */
        bpm_client_t *bpmClientRFFE;
        char *endpoint;
        int bpmNumber;
        int verbose;
        int timeout;
        char *bpmPortName;
        std::unordered_map<int, functionsFloat64_t> bpmHwFloat64Func;

        /* Our private methods */
        asynStatus bpmClientConnect(void);
        asynStatus bpmClientDisconnect(void);
        asynStatus setParamDouble(int functionId, int addr);
        asynStatus getParamDouble(int functionId, epicsFloat64 *param, int addr);
};

#define NUM_PARAMS (&LAST_COMMAND - &FIRST_COMMAND + 1)
