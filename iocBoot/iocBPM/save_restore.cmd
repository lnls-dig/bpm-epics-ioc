### save_restore setup
# status-PV prefix
save_restoreSet_status_prefix("BPM:")
# Debug-output level
save_restoreSet_Debug(0)

# Ok to save/restore save sets with missing values (no CA connection to PV)?
save_restoreSet_IncompleteSetsOk(1)
# Save dated backup files?
save_restoreSet_DatedBackupFiles(1)

# Number of sequenced backup files to write
save_restoreSet_NumSeqFiles(3)
# Time interval between sequenced backups
save_restoreSet_SeqPeriodInSeconds(300)

# If you want save_restore to manage its own NFS mount, specify the name and
# IP address of the file server to which save files should be written.
# This currently is supported only on vxWorks.
#save_restoreSet_NFSHost("oxygen", "164.54.52.4")

# specify where save files should be
set_savefile_path("$(TOP)/iocBoot/$(IOC)", "autosave")

# specify what save files should be restored.  Note these files must be
# in the directory specified in set_savefile_path(), or, if that function
# has not been called, from the directory current when iocInit is invoked

# Save files associated with the request files 'auto_positions.req' and
# 'auto_settings.req'.  These files are the standard way to use autosave in
# synApps.
set_pass0_restoreFile("auto_settings_${EPICS_HOSTNAME}-$(BPM_NUMBER).sav")
set_pass1_restoreFile("auto_settings_${EPICS_HOSTNAME}-$(BPM_NUMBER).sav")

# specify directories in which to to search for included request files
set_requestfile_path("$(TOP)/iocBoot/$(IOC)", "")
set_requestfile_path("$(TOP)/iocBoot/$(IOC)", "autosave")
set_requestfile_path("$(TOP)", "BPMApp/Db")
set_requestfile_path("$(AUTOSAVE)", "asApp/Db")

dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=${EPICS_HOSTNAME}-$(BPM_NUMBER):")
