< envPaths
epicsEnvSet(STARTUP,$(TOP)/iocBoot/$(IOC))

< BPM.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/BPM.dbd")
BPM_registerRecordDeviceDriver (pdbbase)

drvBPMConfigure("$(BPM_NAME)", "$(BPM_ENDPOINT)", "$(BPM_NUMBER)", "$(BPM_VERBOSE)", "$(BPM_TIMEOUT)")

## Load record instances
dbLoadRecords("${TOP}/BPMApp/Db/BPMAcq.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADDR=0, BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMAdc.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADDR=0, BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMDsp.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADDR=0, BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMInfo.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADDR=0, BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMRffe.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADDR=0, BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${EPICS_HOSTNAME}-$(BPM_NUMBER):,R=asyn,PORT=$(BPM_NAME),ADDR=0,OMAX=80,IMAX=80")

< waveformPlugins.cmd
< waveformFilePlugins.cmd
< waveformFFTRecords.cmd
< statsPlugins.cmd
< save_restore.cmd

# initFastSweep(portName, inputName, maxSignals, maxPoints)
#  portName = asyn port name for this new port (string)
#  inputName = name of asynPort providing data
#  maxSignals  = maximum number of signals (spectra)
#  maxPoints  = maximum number of channels per spectrum
#  dataString  = drvInfo string for position data
#  intervalString  = drvInfo string for time interval per point
#initFastSweep("$(PORT)TBT_TS", "$(PORT)", 4, 1000000, "POS_TBT_ARRAY", "POS_TBT_SAMPLE_TIME")
#dbLoadRecords("${TOP}/BPMApp/Db/BPM_TimeSeries.template", "P=${EPICS_HOSTNAME}-$(BPM_NUMBER):,R=TBT:,NUM_TS=1000000,NUM_FREQ=1000000,PORT=$(PORT)TBT_TS")
#initFastSweep("$(PORT)FOFB_TS", "$(PORT)", 4, 1000000, "POS_FOFB_ARRAY", "POS_FOFB_SAMPLE_TIME")
#dbLoadRecords("${TOP}/BPMApp/Db/BPM_TimeSeries.template", "P=${EPICS_HOSTNAME}-$(BPM_NUMBER):,R=FOFB:,NUM_TS=1000000,NUM_FREQ=1000000,PORT=$(PORT)FOFB_TS")

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(BPM_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(BPM_NAME)",0,0xff)

iocInit()

< initCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P={EPICS_HOSTNAME}-$(BPM_NUMBER):")
