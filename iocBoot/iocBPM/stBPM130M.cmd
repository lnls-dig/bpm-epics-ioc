< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

< BPM.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/BPM.dbd")
BPM_registerRecordDeviceDriver (pdbbase)

drvBPMConfigure("$(BPM_NAME)", "$(BPM_ENDPOINT)", "$(BPM_NUMBER)", "$(BPM_VERBOSE)", "$(BPM_TIMEOUT)", "$(WAVEFORM_MAX_POINTS)", "$(MAXBUFFERS)", "$(MAXMEMORY)")

## Load record instances
dbLoadRecords("${TOP}/BPMApp/Db/BPMAcq.template", "P=${P}, R=${R}, ACQ_NAME=ACQ, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMAcq.template", "P=${P}, R=${R}, ACQ_NAME=ACQ_PM, PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMSwitching.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMActiveClk.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMAdcCommon.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFmc130m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMDsp.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMInfo.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMSP.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}asyn,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")

< trigger.cmd
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
#dbLoadRecords("${TOP}/BPMApp/Db/BPM_TimeSeries.template", "P=${P}, R=${R},R=TBT,NUM_TS=1000000,NUM_FREQ=1000000,PORT=$(PORT)TBT_TS")
#initFastSweep("$(PORT)FOFB_TS", "$(PORT)", 4, 1000000, "POS_FOFB_ARRAY", "POS_FOFB_SAMPLE_TIME")
#dbLoadRecords("${TOP}/BPMApp/Db/BPM_TimeSeries.template", "P=${P}, R=${R},R=FOFB,NUM_TS=1000000,NUM_FREQ=1000000,PORT=$(PORT)FOFB_TS")

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(BPM_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(BPM_NAME)",0,0xff)

iocInit()

< initBPMCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
