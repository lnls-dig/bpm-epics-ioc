< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")

< BPMRFFE.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/BPMRFFE.dbd")
BPMRFFE_registerRecordDeviceDriver (pdbbase)

drvBPMRFFEConfigure("$(BPM_NAME)", "$(BPM_ENDPOINT)", "$(BPM_NUMBER)", "$(BPM_VERBOSE)", "$(BPM_TIMEOUT)")

## Load record instances
dbLoadRecords("${TOP}/db/BPMRffe.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}RFFEasyn,PORT=$(BPM_NAME),ADDR=0,OMAX=80,IMAX=80")

< save_restore.cmd

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(BPM_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(BPM_NAME)",0,0xff)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
