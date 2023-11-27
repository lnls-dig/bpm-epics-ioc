< envPaths

# Override default TOP variable
epicsEnvSet("TOP","../..")
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")
epicsEnvSet("BPM_TYPE","FMC250M_4CH")

# devIOCStats vars
epicsEnvSet("ENGINEER","$(ENGINEER=Lucas Russo)")
epicsEnvSet("LOCATION","$(LOCATION=DIGS)")
epicsEnvSet("STARTUP","$(TOP)")
epicsEnvSet("ST_CMD","stBPM250.cmd")

< BPM.config

asSetFilename("$(TOP)/BPMApp/Db/accessSecurityFile.acf")
asSetSubstitutions("P=$(P),R=$(R)")

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
dbLoadRecords("${TOP}/db/MonitDspCtl.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMInfo.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMSP.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMIntlk.template", "P=${P}, R=${R}, PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=${P}, R=${R}asyn,PORT=$(PORT),ADDR=0,OMAX=80,IMAX=80")
# devIOCStats records
dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminSoft.db","IOC=${P}${R}Stats")
dbLoadRecords("$(DEVIOCSTATS)/db/iocAdminScanMon.db","IOC=${P}${R}Stats")

< fofb_cc.cmd

< triggerBPM.cmd
< fmc250m_4ch.cmd
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

< initBPMCommands

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=${P}, R=${R}")
set_savefile_name("auto_settings.req", "auto_settings_${P}${R}.sav")
