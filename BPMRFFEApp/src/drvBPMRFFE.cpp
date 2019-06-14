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

#define SERVICE_NAME_SIZE               50

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
static const functionsAny_t bpmSetGetRffeAttFunc =        {functionsFloat64_t{"RFFE", halcs_set_rffe_att, halcs_get_rffe_att}};
static const functionsAny_t bpmSetGetRffeTempACFunc =     {functionsFloat64_t{"RFFE", NULL, halcs_get_rffe_temp_ac}};
static const functionsAny_t bpmSetGetRffeTempBDFunc =     {functionsFloat64_t{"RFFE", NULL, halcs_get_rffe_temp_bd}};
static const functionsAny_t bpmSetGetRffeSetPointACFunc = {functionsFloat64_t{"RFFE", halcs_set_rffe_set_point_ac, halcs_get_rffe_set_point_ac}};
static const functionsAny_t bpmSetGetRffeSetPointBDFunc = {functionsFloat64_t{"RFFE", halcs_set_rffe_set_point_bd, halcs_get_rffe_set_point_bd}};
static const functionsAny_t bpmSetGetRffeHeaterACFunc =   {functionsFloat64_t{"RFFE", halcs_set_rffe_heater_ac, halcs_get_rffe_heater_ac}};
static const functionsAny_t bpmSetGetRffeHeaterBDFunc =   {functionsFloat64_t{"RFFE", halcs_set_rffe_heater_bd, halcs_get_rffe_heater_bd}};
static const functionsAny_t bpmSetGetRffePidACKpFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_ac_kp, halcs_get_rffe_pid_ac_kp}};
static const functionsAny_t bpmSetGetRffePidACTiFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_ac_ti, halcs_get_rffe_pid_ac_ti}};
static const functionsAny_t bpmSetGetRffePidACTdFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_ac_td, halcs_get_rffe_pid_ac_td}};
static const functionsAny_t bpmSetGetRffePidBDKpFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_bd_kp, halcs_get_rffe_pid_bd_kp}};
static const functionsAny_t bpmSetGetRffePidBDTiFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_bd_ti, halcs_get_rffe_pid_bd_ti}};
static const functionsAny_t bpmSetGetRffePidBDTdFunc =    {functionsFloat64_t{"RFFE", halcs_set_rffe_pid_bd_td, halcs_get_rffe_pid_bd_td}};

/* Compatibility version for uint8_t functions types */
halcs_client_err_e halcs_set_rffe_temp_control_compat (halcs_client_t *self, char *service,
        uint32_t rffe_temp_control)
{
    return halcs_set_rffe_temp_control (self, service, (uint8_t) rffe_temp_control);
}

halcs_client_err_e halcs_get_rffe_temp_control_compat (halcs_client_t *self, char *service,
        uint32_t *rffe_temp_control)
{
    return halcs_get_rffe_temp_control (self, service, (uint8_t *) rffe_temp_control);
}

halcs_client_err_e halcs_set_rffe_reset_compat (halcs_client_t *self, char *service,
        uint32_t rffe_reset)
{
    return halcs_set_rffe_reset (self, service, (uint8_t) rffe_reset);
}

halcs_client_err_e halcs_get_rffe_reset_compat (halcs_client_t *self, char *service,
        uint32_t *rffe_reset)
{
    return halcs_get_rffe_reset (self, service, (uint8_t *) rffe_reset);
}

/* Int32 functions mapping */
static const functionsAny_t bpmSetGetRffeTempCtlFunc = {functionsInt32_t{"RFFE", halcs_set_rffe_temp_control_compat, halcs_get_rffe_temp_control_compat}};
static const functionsAny_t bpmSetGetRffeRstFunc =     {functionsInt32_t{"RFFE", halcs_set_rffe_reset_compat, halcs_get_rffe_reset_compat}};

static const char *driverName="drvBPMRFFE";
void acqTask(void *drvPvt);

static void exitHandlerC(void *pPvt)
{
    drvBPMRFFE *pdrvBPMRFFE = (drvBPMRFFE *)pPvt;
    pdrvBPMRFFE->~drvBPMRFFE();
}

asynStatus drvBPMRFFE::getServiceID (int bpmNumber, int addr, const char *serviceName,
        int *serviceIDArg) const
{
    static const char *functionName = "getServiceID";
    asynStatus status = asynSuccess;

    *serviceIDArg = boardMap[bpmNumber].bpm;

    return status;
}

asynStatus drvBPMRFFE::getFullServiceName (int bpmNumber, int addr, const char *serviceName,
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

/** Constructor for the drvBPMRFFE class.
 * Calls constructor for the asynPortDriver base class.
 * \param[in] portName The name of the asyn port driver to be created.
 * \param[in] endpoint The device address string ]
 * */
drvBPMRFFE::drvBPMRFFE(const char *portName, const char *endpoint, int bpmNumber,
        int verbose, int timeout)
   : asynPortDriver(portName,
                    MAX_ADDR, /* maxAddr */
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
    createParam(P_RffeTempACString,      asynParamFloat64,               &P_RffeTempAC);
    createParam(P_RffeTempBDString,      asynParamFloat64,               &P_RffeTempBD);
    createParam(P_RffeAttString,         asynParamFloat64,               &P_RffeAtt);
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
    setDoubleParam(P_RffeTempAC,                             0.0);
    setDoubleParam(P_RffeTempBD,                             0.0);
    setDoubleParam(P_RffeAtt,                               31.5);
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
    bpmRFFEHwFunc.emplace(P_RffeTempAC, bpmSetGetRffeTempACFunc);
    bpmRFFEHwFunc.emplace(P_RffeTempBD, bpmSetGetRffeTempBDFunc);
    bpmRFFEHwFunc.emplace(P_RffeAtt, bpmSetGetRffeAttFunc);
    bpmRFFEHwFunc.emplace(P_RffeSetPointAC, bpmSetGetRffeSetPointACFunc);
    bpmRFFEHwFunc.emplace(P_RffeSetPointBD, bpmSetGetRffeSetPointBDFunc);
    bpmRFFEHwFunc.emplace(P_RffeHeaterAC, bpmSetGetRffeHeaterACFunc);
    bpmRFFEHwFunc.emplace(P_RffeHeaterBD, bpmSetGetRffeHeaterBDFunc);
    bpmRFFEHwFunc.emplace(P_RffePidACKp, bpmSetGetRffePidACKpFunc);
    bpmRFFEHwFunc.emplace(P_RffePidACTi, bpmSetGetRffePidACTiFunc);
    bpmRFFEHwFunc.emplace(P_RffePidACTd, bpmSetGetRffePidACTdFunc);
    bpmRFFEHwFunc.emplace(P_RffePidBDKp, bpmSetGetRffePidBDKpFunc);
    bpmRFFEHwFunc.emplace(P_RffePidBDTi, bpmSetGetRffePidBDTiFunc);
    bpmRFFEHwFunc.emplace(P_RffePidBDTd, bpmSetGetRffePidBDTdFunc);

    bpmRFFEHwFunc.emplace(P_RffeTempCtl, bpmSetGetRffeTempCtlFunc);
    bpmRFFEHwFunc.emplace(P_RffeRst, bpmSetGetRffeRstFunc);

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
        bpmClientRFFE = halcs_client_new_time (endpoint, verbose, bpmLogFile, timeout);
        if (bpmClientRFFE == NULL) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s bpmClientConnect failure to create bpmClientRFFE instance\n",
                    driverName, functionName);
            status = asynError;
            goto create_halcs_client_err;
        }
    }

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: BPM client connected\n",
        driverName, functionName);

    pasynManager->exceptionConnect(this->pasynUserSelf);

    return status;

create_halcs_client_err:
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
        halcs_client_destroy (&bpmClientRFFE);
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
    /* Readback all parameters from Hw */
    readUInt32Params(mask, addr);

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
    /* Set the parameter in the parameter library. */
    setDoubleParam(addr, function, value);

    /* Do operation on HW. Some functions do not set anything on hardware */
    status = setParamDouble(function, addr);
    /* Readback all parameters from Hw */
    readFloat64Params(addr);

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
/************ Function Mapping Overloaded Write functions ***********/
/********************************************************************/

asynStatus drvBPMRFFE::doExecuteHwWriteFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(bpmClientRFFE, service, functionParam.argFloat64);
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

asynStatus drvBPMRFFE::doExecuteHwWriteFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwWriteFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.write(bpmClientRFFE, service, functionParam.argUInt32);
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

asynStatus drvBPMRFFE::executeHwWriteFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwWriteFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    const char *paramName = NULL;
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = bpmRFFEHwFunc.find (functionId);
    if (func == bpmRFFEHwFunc.end()) {
        getParamName(functionId, &paramName);
        /* This is not an error. Exit silently */
        status = asynSuccess;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d, name = %s\n",
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

get_reg_func_err:
get_service_err:
        return (asynStatus)status;
}

/********************************************************************/
/************ Function Mapping Overloaded Read functions ************/
/********************************************************************/

asynStatus drvBPMRFFE::doExecuteHwReadFunction(functionsFloat64_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsFloat64_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(bpmClientRFFE, service, &functionParam.argFloat64);
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

asynStatus drvBPMRFFE::doExecuteHwReadFunction(functionsInt32_t &func, char *service,
        int addr, functionsArgs_t &functionParam) const
{
    const char *functionName = "doExecuteHwReadFunction<functionsInt32_t>";
    halcs_client_err_e err = HALCS_CLIENT_SUCCESS;
    int status = asynSuccess;

    /* Execute registered function */
    err = func.read(bpmClientRFFE, service, &functionParam.argUInt32);
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

asynStatus drvBPMRFFE::executeHwReadFunction(int functionId, int addr,
        functionsArgs_t &functionParam)
{
    int status = asynSuccess;
    const char *functionName = "executeHwReadFunction";
    const char *funcService = NULL;
    char service[SERVICE_NAME_SIZE];
    const char *paramName = NULL;
    std::unordered_map<int,functionsAny_t>::iterator func;

    /* Lookup function on map */
    func = bpmRFFEHwFunc.find (functionId);
    if (func == bpmRFFEHwFunc.end()) {
        getParamName(functionId, &paramName);
        /* We use disabled to indicate the function was not found on Hw mapping */
        status = asynDisabled;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: no registered function for functionID = %d, name = %s\n",
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
* functions defined in the structures. e.g., functionsInt32_t
* and functionsFloat64_t
*/
asynStatus drvBPMRFFE::setParamGeneric(int functionId, int addr)
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

asynStatus drvBPMRFFE::setParam32(int functionId, epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParam32";
    const char *paramName = NULL;

    status = getUIntDigitalParam(addr, functionId, &functionArgs.argUInt32, mask);
    if (status != asynSuccess) {
        getParamName(functionId, &paramName);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

get_param_err:
    return (asynStatus)status;
}

asynStatus drvBPMRFFE::getParam32(int functionId, epicsUInt32 *param,
        epicsUInt32 mask, int addr)
{
    int status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParam32";
    const char *paramName = NULL;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getUIntDigitalParam(addr, functionId, param, mask);
    if (status != asynSuccess) {
        getParamName(functionId, &paramName);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getUIntDigitalParam failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
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

asynStatus drvBPMRFFE::setParamDouble(int functionId, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "setParamDouble";
    const char *paramName = NULL;

    status = getDoubleParam(addr, functionId, &functionArgs.argFloat64);
    if (status != asynSuccess) {
        getParamName(functionId, &paramName);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getDoubleParam failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
        goto get_param_err;
    }

    status = executeHwWriteFunction(functionId, addr, functionArgs);

get_param_err:
    return status;
}

asynStatus drvBPMRFFE::getParamDouble(int functionId, epicsFloat64 *param, int addr)
{
    asynStatus status = asynSuccess;
    functionsArgs_t functionArgs = {0};
    const char *functionName = "getParamDouble";
    const char *paramName = NULL;

    /* Get parameter in library, as some parameters are not written in HW */
    status = getDoubleParam(addr, functionId, param);
    if (status != asynSuccess) {
        getParamName(functionId, &paramName);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: getDoubleParam failure for retrieving parameter %s\n",
                driverName, functionName, paramName);
        goto get_param_err;
    }

    status = executeHwReadFunction(functionId, addr, functionArgs);
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

asynStatus drvBPMRFFE::updateUInt32Params(epicsUInt32 mask, int addr, int firstParam,
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
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting UInt32 parameter for function = %d, "
                    "addr = %d status = %d\n",
                    driverName, functionName, i, addr, status);
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

asynStatus drvBPMRFFE::updateDoubleParams(int addr, int firstParam, int lastParam,
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
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: error getting Double parameter for function = %d, "
                    "addr = %d status = %d\n",
                    driverName, functionName, i, addr, status);
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

asynStatus drvBPMRFFE::readUInt32Params(int mask, int addr)
{
    return updateUInt32Params(mask, addr, P_RffeTempCtl, P_RffeRst, false);
}

asynStatus drvBPMRFFE::readFloat64Params(int addr)
{
    return updateDoubleParams(addr, P_RffeAtt, P_RffePidBDTd, false);
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
