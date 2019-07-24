< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
epicsEnvSet("BPM_TYPE","FMCPOF_5CH")

< BPM.config

## Register all support components
dbLoadDatabase("${TOP}/dbd/BPM.dbd")
BPM_registerRecordDeviceDriver (pdbbase)

drvBPMConfigure("$(BPM_NAME)", "$(BPM_ENDPOINT)", "$(BPM_NUMBER)", "$(BPM_TYPE)", "$(BPM_VERBOSE)", "$(BPM_TIMEOUT)", "$(WAVEFORM_MAX_POINTS)", "$(MAXBUFFERS)", "$(MAXMEMORY)")

## Load record instances
dbLoadRecords("${TOP}/db/BPMAcq.template", "P=${P}, R=${R}, ACQ_NAME=ACQ, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/AFCMgmt.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMInfo.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/MonitDspCtl.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/TIMRcv.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}asyn,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")

< triggerTIM.cmd
< waveformPlugins.cmd
< waveformFilePlugins.cmd
< waveformFFTRecords.cmd
< statsPlugins.cmd
< save_restore.cmd

# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
asynSetTraceIOMask("$(BPM_NAME)",0,0x2)
#asynSetTraceMask("", 0, 17)
#asynSetTraceMask("$(BPM_NAME)",0,0xff)

# Disable locking virtual memory as it can increase memory usage
# unnecessarily in systems that run the IOC with SCHED_FIFO and
# supports running threads with different priorities. See:
# https://epics.anl.gov/base/R3-15/6-docs/RELEASE_NOTES.html and
# https://github.com/epics-base/epics-base/commit/e721be4ff528bc1fff35b9e0cffd2a194f3e3675
var dbThreadRealtimeLock 0

iocInit()

< initTIMCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
