/*
 * drvBPMRFFE.cpp
 *
 * Authors: Lucas Russo
 *
 * Created Mar. 02, 2016
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

#include "drvBPMRFFE.h"
#include <epicsExport.h>

static const boardMap_t boardMap[MAX_BPMS+1] = {
         /* board, bpm*/
    /* 0 (INVALID)  */ {-1, -1},
    /* 1            */ {1,   0},
    /* 2            */ {1,   1},
    /* 3            */ {2,   0},
    /* 4            */ {2,   1},
    /* 5            */ {3,   0},
    /* 6            */ {3,   1},
    /* 7            */ {4,   0},
    /* 8            */ {4,   1},
    /* 9            */ {5,   0},
    /* 10           */ {5,   1},
    /* 11           */ {6,   0},
    /* 12           */ {6,   1},
    /* 13           */ {7,   0},
    /* 14           */ {7,   1},
    /* 15           */ {8,   0},
    /* 16           */ {8,   1},
    /* 17           */ {9,   0},
    /* 18           */ {9,   1},
    /* 19           */ {10,  0},
    /* 20           */ {10,  1},
    /* 21           */ {11,  0},
    /* 22           */ {11,  1},
    /* 23           */ {12,  0},
    /* 24           */ {12,  1}
};

/* Double functions mapping */
static const functionsFloat64_t bpmSetGetRffeAttFunc = {"RFFE", bpm_set_rffe_att, bpm_get_rffe_att};
static const functionsFloat64_t bpmSetGetRffeTempACFunc = {"RFFE", NULL, bpm_get_rffe_temp_ac};
static const functionsFloat64_t bpmSetGetRffeTempBDFunc = {"RFFE", NULL, bpm_get_rffe_temp_bd};
static const functionsFloat64_t bpmSetGetRffeSetPointACFunc = {"RFFE", bpm_set_rffe_set_point_ac, bpm_get_rffe_set_point_ac};
static const functionsFloat64_t bpmSetGetRffeSetPointBDFunc = {"RFFE", bpm_set_rffe_set_point_bd, bpm_get_rffe_set_point_bd};
static const functionsFloat64_t bpmSetGetRffeHeaterACFunc = {"RFFE", bpm_set_rffe_heater_ac, bpm_get_rffe_heater_ac};
static const functionsFloat64_t bpmSetGetRffeHeaterBDFunc = {"RFFE", bpm_set_rffe_heater_bd, bpm_get_rffe_heater_bd};
static const functionsFloat64_t bpmSetGetRffePidACKpFunc = {"RFFE", bpm_set_rffe_pid_ac_kp, bpm_get_rffe_pid_ac_kp};
static const functionsFloat64_t bpmSetGetRffePidACTiFunc = {"RFFE", bpm_set_rffe_pid_ac_ti, bpm_get_rffe_pid_ac_ti};
static const functionsFloat64_t bpmSetGetRffePidACTdFunc = {"RFFE", bpm_set_rffe_pid_ac_td, bpm_get_rffe_pid_ac_td};
static const functionsFloat64_t bpmSetGetRffePidBDKpFunc = {"RFFE", bpm_set_rffe_pid_bd_kp, bpm_get_rffe_pid_bd_kp};
static const functionsFloat64_t bpmSetGetRffePidBDTiFunc = {"RFFE", bpm_set_rffe_pid_bd_ti, bpm_get_rffe_pid_bd_ti};
static const functionsFloat64_t bpmSetGetRffePidBDTdFunc = {"RFFE", bpm_set_rffe_pid_bd_td, bpm_get_rffe_pid_bd_td};


/* Compatibility version for uint8_t functions types */
bpm_client_err_e bpm_set_rffe_temp_control_compat (bpm_client_t *self, char *service,
        uint32_t rffe_temp_control)
{
    return bpm_set_rffe_temp_control (self, service, (uint8_t) rffe_temp_control);
}

bpm_client_err_e bpm_get_rffe_temp_control_compat (bpm_client_t *self, char *service,
        uint32_t *rffe_temp_control)
{
    return bpm_get_rffe_temp_control (self, service, (uint8_t *) rffe_temp_control);
}

bpm_client_err_e bpm_set_rffe_reset_compat (bpm_client_t *self, char *service,
        uint32_t rffe_reset)
{
    return bpm_set_rffe_reset (self, service, (uint8_t) rffe_reset);
}

bpm_client_err_e bpm_get_rffe_reset_compat (bpm_client_t *self, char *service,
        uint32_t *rffe_reset)
{
    return bpm_get_rffe_reset (self, service, (uint8_t *) rffe_reset);
}

/* Int32 functions mapping */
static const functionsInt32_t bpmSetGetRffeTempCtlFunc = {"RFFE", bpm_set_rffe_temp_control_compat, bpm_get_rffe_temp_control_compat};
static const functionsInt32_t bpmSetGetRffeRstFunc = {"RFFE", bpm_set_rffe_reset_compat, bpm_get_rffe_reset_compat};

static const char *driverName="drvBPMRFFE";
void acqTask(void *drvPvt);

static void exitHandlerC(void *pPvt)
{
    drvBPMRFFE *pdrvBPMRFFE = (drvBPMRFFE *)pPvt;
    pdrvBPMRFFE->~drvBPMRFFE();
}

/** Constructor for the drvBPMRFFE class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] endpoint The device address string ]
 * */
drvBPMRFFE::drvBPMRFFE(const char *portName, const char *endpoint, int bpmNumber,
        int verbose, int timeout)
   : asynPortDriver(portName,
                    MAX_ADDR, /* maxAddr */
                    (int)NUM_PARAMS,
                    asynUInt32DigitalMask | asynFloat64Mask  | asynDrvUserMask,    /* Interface mask     */
                    asynUInt32DigitalMask | asynFloat64Mask ,                      /* Interrupt mask     */
                    ASYN_CANBLOCK | ASYN_MULTIDEVICE, /* asynFlags.  This driver blocks it is multi-device */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size*/
{
    asynStatus status;
    const char *functionName = "drvBPMRFFE";

    /* Create portName so we can create a new AsynUser later */
    bpmPortName = epicsStrDup(portName);

    this->endpoint = strdup(endpoint);
    if (this->endpoint == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s drvBPMRFFE failure to copy endpoint\n",
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

    /* Create parameters */
    createParam(P_RffeAttString,         asynParamFloat64,               &P_RffeAtt);
    createParam(P_RffeTempACString,      asynParamFloat64,               &P_RffeTempAC);
    createParam(P_RffeTempBDString,      asynParamFloat64,               &P_RffeTempBD);
    createParam(P_RffeSetPointACString,  asynParamFloat64,               &P_RffeSetPointAC);
    createParam(P_RffeSetPointBDString,  asynParamFloat64,               &P_RffeSetPointBD);
    createParam(P_RffeHeaterACString,    asynParamFloat64,               &P_RffeHeaterAC);
    createParam(P_RffeHeaterBDString,    asynParamFloat64,               &P_RffeHeaterBD);
    createParam(P_RffePidACKpString,     asynParamFloat64,               &P_RffePidACKp);
    createParam(P_RffePidACTiString,     asynParamFloat64,               &P_RffePidACTi);
    createParam(P_RffePidACTdString,     asynParamFloat64,               &P_RffePidACTd);
    createParam(P_RffePidBDKpString,     asynParamFloat64,               &P_RffePidBDKp);
    createParam(P_RffePidBDTiString,     asynParamFloat64,               &P_RffePidBDTi);
    createParam(P_RffePidBDTdString,     asynParamFloat64,               &P_RffePidBDTd);

    createParam(P_RffeTempCtlString,     asynParamUInt32Digital,         &P_RffeTempCtl);
    createParam(P_RffeRstString,         asynParamUInt32Digital,         &P_RffeRst);

    /* Set the initial values of some parameters */
    setDoubleParam(P_RffeAtt,                               31.5);
    setDoubleParam(P_RffeTempAC,                             0.0);
    setDoubleParam(P_RffeTempBD,                             0.0);
    setDoubleParam(P_RffeSetPointAC,                         0.0);
    setDoubleParam(P_RffeSetPointBD,                         0.0);
    setDoubleParam(P_RffeHeaterAC,                           0.0);
    setDoubleParam(P_RffeHeaterBD,                           0.0);
    setDoubleParam(P_RffePidACKp,                            0.0);
    setDoubleParam(P_RffePidACTi,                            0.0);
    setDoubleParam(P_RffePidACTd,                            0.0);
    setDoubleParam(P_RffePidBDKp,                            0.0);
    setDoubleParam(P_RffePidBDTi,                            0.0);
    setDoubleParam(P_RffePidBDTd,                            0.0);

    setUIntDigitalParam(P_RffeTempCtl,                       0, 0xFFFFFFFF);
    setUIntDigitalParam(P_RffeRst,                           0, 0xFFFFFFFF);

    /* Do callbacks so higher layers see any changes. Call callbacks for every addr */
    for (int i = 0; i < MAX_ADDR; ++i) {
        callParamCallbacks(i);
    }

    /* BPM Float64 Functions mapping. Functions not mapped here are just written
     * to the parameter library */
    bpmHwFloat64Func[P_RffeAtt] = bpmSetGetRffeAttFunc;
    bpmHwFloat64Func[P_RffeTempAC] = bpmSetGetRffeTempACFunc;
    bpmHwFloat64Func[P_RffeTempBD] = bpmSetGetRffeTempBDFunc;
    bpmHwFloat64Func[P_RffeSetPointAC] = bpmSetGetRffeSetPointACFunc;
    bpmHwFloat64Func[P_RffeSetPointBD] = bpmSetGetRffeSetPointBDFunc;
    bpmHwFloat64Func[P_RffeHeaterAC] = bpmSetGetRffeHeaterACFunc;
    bpmHwFloat64Func[P_RffeHeaterBD] = bpmSetGetRffeHeaterBDFunc;
    bpmHwFloat64Func[P_RffePidACKp] = bpmSetGetRffePidACKpFunc;
    bpmHwFloat64Func[P_RffePidACTi] = bpmSetGetRffePidACTiFunc;
    bpmHwFloat64Func[P_RffePidACTd] = bpmSetGetRffePidACTdFunc;
    bpmHwFloat64Func[P_RffePidBDKp] = bpmSetGetRffePidBDKpFunc;
    bpmHwFloat64Func[P_RffePidBDTi] = bpmSetGetRffePidBDTiFunc;
    bpmHwFloat64Func[P_RffePidBDTd] = bpmSetGetRffePidBDTdFunc;

    bpmHwInt32Func[P_RffeTempCtl] = bpmSetGetRffeTempCtlFunc;
    bpmHwInt32Func[P_RffeRst] = bpmSetGetRffeRstFunc;

    lock();
    status = bpmClientConnect();
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

    epicsAtExit(exitHandlerC, this);

invalid_bpm_number_err:
    free (this->endpoint);
endpoint_dup_err:
    return;
}

/** Destructor for the drvBPMRFFE class.
 */
drvBPMRFFE::~drvBPMRFFE()
{
    asynStatus status = asynSuccess;
    const char *functionName = "~drvBPMRFFE";

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

asynStatus drvBPMRFFE::connect(asynUser* pasynUser)
{
    return bpmClientConnect();
}

asynStatus drvBPMRFFE::bpmClientConnect(void)
{
    asynStatus status = asynSuccess;
    const char *bpmLogFile = "stdout";
    const char *functionName = "bpmClientConnect";

    /* Connect BPM */
    if (bpmClientRFFE == NULL) {
        bpmClientRFFE = bpm_client_new_time (endpoint, verbose, bpmLogFile, timeout);
        if (bpmClientRFFE == NULL) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s bpmClientConnect failure to create bpmClientRFFE instance\n",
                    driverName, functionName);
            status = asynError;
            goto create_bpm_client_err;
        }
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: BPM client connected\n",
        driverName, functionName);

    pasynManager->exceptionConnect(this->pasynUserSelf);

    return status;

create_bpm_client_err:
    return status;
}

asynStatus drvBPMRFFE::disconnect(asynUser* pasynUser)
{
    return bpmClientDisconnect();
}

asynStatus drvBPMRFFE::bpmClientDisconnect(void)
{
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s: calling bpmClientDisconnect\n",
            driverName);
    asynStatus status = asynSuccess;

    if (bpmClientRFFE != NULL) {
        bpm_client_destroy (&bpmClientRFFE);
    }

    pasynManager->exceptionDisconnect(this->pasynUserSelf);
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
asynStatus drvBPMRFFE::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "writeUInt32Digital";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%d",
                driverName, functionName, status, function, paramName, value);
        return status;
    }
    /* Set the parameter in the parameter library. */
    setUIntDigitalParam(function, value, mask);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParam32(function, mask, addr);

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
asynStatus drvBPMRFFE::readUInt32Digital(asynUser *pasynUser, epicsUInt32 *value,
        epicsUInt32 mask)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *functionName = "readUInt32Digital";
    const char *paramName;

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get parameter, possibly from HW */
    status = getParam32(function, value, mask, addr);

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
asynStatus drvBPMRFFE::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "writeFloat64";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s, value=%f",
                driverName, functionName, status, function, paramName, value);
        return status;
    }
    /* Set the parameter in the parameter library. */
    setDoubleParam(addr, function, value);
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParamDouble(function, addr);

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
asynStatus drvBPMRFFE::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int addr = 0;
    const char *paramName;
    const char* functionName = "readFloat64";

    /* Get channel for possible use */
    status = getAddress(pasynUser, &addr);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                "%s:%s: status=%d, function=%d, name=%s",
                driverName, functionName, status, function, paramName);
        return status;
    }
    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    /* Get double param, possibly from HW */
    status = getParamDouble(function, value, addr);

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
/*************** Generic 32-bit/Double BPM Operations ***************/
/********************************************************************/

/*
* 32-bit/Double generic BPM operations. These will map to real
* functions defined in the structures. e.g., functionsInt32_t
* and functionsFloat64_t
*/
asynStatus drvBPMRFFE::setParam32(int functionId, epicsUInt32 mask, int addr)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsUInt32 paramLib = 0;
    const char *functionName = "setParam32";
    char service[50];
    std::unordered_map<int,functionsInt32_t>::const_iterator func;

    status = getUIntDigitalParam(addr, functionId, &paramLib, mask);
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
        snprintf(service, sizeof(service), "HALCS%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.write) {
            goto no_registered_write_func;
        }

        /* Function found. Execute it */
        err = func->second.write(bpmClientRFFE, service, paramLib);
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

bpm_set_func1_param_err:
no_registered_write_func:
get_param_err:
    return status;
}

asynStatus drvBPMRFFE::getParam32(int functionId, epicsUInt32 *param,
        epicsUInt32 mask, int addr)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsUInt32 paramHw = 0;
    const char *functionName = "getParam32";
    char service[50];
    std::unordered_map<int,functionsInt32_t>::const_iterator func;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(addr, functionId, param, mask);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function */
    func = bpmHwInt32Func.find (functionId);
    if (func != bpmHwInt32Func.end()) {
        *param = 0;
        /* Get correct service name*/
        snprintf(service, sizeof(service), "HALCS%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.read) {
            goto no_registered_read_func;
        }

        /* Function found. Execute it */
        err = func->second.read(bpmClientRFFE, service, &paramHw);
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

bpm_get_func1_param_err:
no_registered_read_func:
get_param_err:
    return status;
}

asynStatus drvBPMRFFE::setParamDouble(int functionId, int addr)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    epicsFloat64 paramLib = 0;
    const char *functionName = "setParamDouble";
    char service[50];
    std::unordered_map<int,functionsFloat64_t>::const_iterator func;

    status = getDoubleParam(addr, functionId, &paramLib);
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
        snprintf(service, sizeof(service), "HALCS%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.write) {
            goto no_registered_write_func;
        }

        /* Function found. Execute it */
        err = func->second.write(bpmClientRFFE, service, paramLib);
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

asynStatus drvBPMRFFE::getParamDouble(int functionId, epicsFloat64 *param, int addr)
{
    asynStatus status = asynSuccess;
    bpm_client_err_e err = BPM_CLIENT_SUCCESS;
    const char *functionName = "getParamDouble";
    char service[50];
    std::unordered_map<int,functionsFloat64_t>::const_iterator func;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(addr, functionId, param);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter\n",
                driverName, functionName);
        goto get_param_err;
    }

    /* Lookup function */
    func = bpmHwFloat64Func.find (functionId);
    if (func != bpmHwFloat64Func.end()) {
        *param = 0;
        /* Get correct service name*/
        snprintf(service, sizeof(service), "HALCS%d:DEVIO:%s%d",
                boardMap[this->bpmNumber].board, func->second.serviceName,
                boardMap[this->bpmNumber].bpm);

        /* Silently exit if no function is registered */
        if(!func->second.read) {
            goto no_registered_read_func;
        }

        /* Function found. Execute it */
        err = func->second.read(bpmClientRFFE, service, param);
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

/* Configuration routine.  Called directly, or from the iocsh function below */
extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvBPMRFFE class.
     * \param[in] portName The name of the asyn port driver to be created.
     * \param[in] endpoint The address device string */
    int drvBPMRFFEConfigure(const char *portName, const char *endpoint,
            int bpmNumber, int verbose, int timeout)
    {
        new drvBPMRFFE(portName, endpoint, bpmNumber, verbose, timeout);
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
    static const iocshFuncDef initFuncDef = {"drvBPMRFFEConfigure",5,initArgs};
    static void initCallFunc(const iocshArgBuf *args)
    {
        drvBPMRFFEConfigure(args[0].sval, args[1].sval, args[2].ival,
                args[3].ival, args[4].ival);
    }

    void drvBPMRFFERegister(void)
    {
        iocshRegister(&initFuncDef,initCallFunc);
    }

    epicsExportRegistrar(drvBPMRFFERegister);
}
