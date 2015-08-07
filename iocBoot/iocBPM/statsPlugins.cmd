# Create statistics plugins

############################## ADC Statistics ##########################
#
#NDStatsConfigure("STATS1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_A_STATS:, PORT=STATS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_A_STATS:, PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATS2", $(QSIZE), 0, "$(PORT)", 1, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_B_STATS:, PORT=STATS2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_B_STATS:, PORT=STATS2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATS3", $(QSIZE), 0, "$(PORT)", 2, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_C_STATS:, PORT=STATS3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_C_STATS:, PORT=STATS3,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATS4", $(QSIZE), 0, "$(PORT)", 3, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_D_STATS:, PORT=STATS4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_D_STATS:, PORT=STATS4,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
###################### ADC SWAP Statistics ##########################
#
#NDStatsConfigure("STATSSWAP1", $(QSIZE), 0, "$(PORT)", 5, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_A_STATS:, PORT=STATSSWAP1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=5")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_A_STATS:, PORT=STATSSWAP1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSSWAP2", $(QSIZE), 0, "$(PORT)", 6, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_B_STATS:, PORT=STATSSWAP2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_B_STATS:, PORT=STATSSWAP2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSSWAP3", $(QSIZE), 0, "$(PORT)", 7, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_C_STATS:, PORT=STATSSWAP3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_C_STATS:, PORT=STATSSWAP3,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSSWAP4", $(QSIZE), 0, "$(PORT)", 8, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_D_STATS:, PORT=STATSSWAP4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADCSWAP_D_STATS:, PORT=STATSSWAP4,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
###################### TBT AMP Statistics ##########################
#
#NDStatsConfigure("STATSTBT_A", $(QSIZE), 0, "$(PORT)", 10, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_A_STATS:, PORT=STATSTBT_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=10")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_A_STATS:, PORT=STATSTBT_A,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSTBT_B", $(QSIZE), 0, "$(PORT)", 11, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_B_STATS:, PORT=STATSTBT_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=11")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_B_STATS:, PORT=STATSTBT_B,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSTBT_C", $(QSIZE), 0, "$(PORT)", 12, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_C_STATS:, PORT=STATSTBT_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_C_STATS:, PORT=STATSTBT_C,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSTBT_D", $(QSIZE), 0, "$(PORT)", 13, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_D_STATS:, PORT=STATSTBT_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):TBT_D_STATS:, PORT=STATSTBT_D,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
###################### FOFB AMP Statistics ##########################
#
#NDStatsConfigure("STATSFOFB_A", $(QSIZE), 0, "$(PORT)", 21, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_A_STATS:, PORT=STATSFOFB_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_A_STATS:, PORT=STATSFOFB_A,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSFOFB_B", $(QSIZE), 0, "$(PORT)", 22, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_B_STATS:, PORT=STATSFOFB_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=22")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_B_STATS:, PORT=STATSFOFB_B,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSFOFB_C", $(QSIZE), 0, "$(PORT)", 23, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_C_STATS:, PORT=STATSFOFB_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=23")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_C_STATS:, PORT=STATSFOFB_C,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
#
#NDStatsConfigure("STATSFOFB_D", $(QSIZE), 0, "$(PORT)", 24, 0, 0)
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_D_STATS:, PORT=STATSFOFB_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24")
#dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):FOFB_D_STATS:, PORT=STATSFOFB_D,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### File Saving Waveforms ##########################

### Create a netCDF file saving plugin.
##NDFileNetCDFConfigure("ADC_A_FileNetCDF1", $(QSIZE), 0, "$(PORT)", 0)
##dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_A_netCDF1:,PORT=ADC_A_FileNetCDF1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
##dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFile.template",      "P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_A_netCDF1:,PORT=ADC_A_FileNetCDF1,ADDR=0,TIMEOUT=1")
##dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDFileNetCDF.template","P=${EPICS_HOSTNAME}:,R=$(PORT)$(BPM_NUMBER):ADC_A_netCDF1:,PORT=ADC_A_FileNetCDF1,ADDR=0,TIMEOUT=1")
