< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
epicsEnvSet("BPM_TYPE","FMC130M_4CH")

< BPM.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/BPM.dbd")
BPM_registerRecordDeviceDriver (pdbbase)

drvBPMConfigure("$(BPM_NAME)", "$(BPM_ENDPOINT)", "$(BPM_NUMBER)", "$(BPM_TYPE)", "$(BPM_VERBOSE)", "$(BPM_TIMEOUT)", "$(WAVEFORM_MAX_POINTS)", "$(MAXBUFFERS)", "$(MAXMEMORY)")

## Load record instances
dbLoadRecords("${TOP}/db/BPMAcq.template", "P=${P}, R=${R}, ACQ_NAME=ACQ, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMAcq.template", "P=${P}, R=${R}, ACQ_NAME=ACQ_PM, PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMSwitching.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMActiveClk.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMAdcCommon.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMFmc130m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMDsp.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/MonitDspCtl.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMInfo.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMSP.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}asyn,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")

< triggerBPM.cmd
< waveformPlugins.cmd
< waveformFilePlugins.cmd
< waveformFFTRecords.cmd
< statsPlugins.cmd
< save_restore.cmd

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(BPM_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(BPM_NAME)",0,0xff)

iocInit()

< initBPMCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
