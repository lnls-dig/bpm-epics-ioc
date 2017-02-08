# Create statistics plugins

############################## ADC Statistics ##########################

NDStatsConfigure("STATS1", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_A_STATS, PORT=STATS1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_A_STATS, PORT=STATS1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS2", $(QSIZE), 0, "$(PORT)", 1, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_B_STATS, PORT=STATS2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_B_STATS, PORT=STATS2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS3", $(QSIZE), 0, "$(PORT)", 2, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_C_STATS, PORT=STATS3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_C_STATS, PORT=STATS3,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS4", $(QSIZE), 0, "$(PORT)", 3, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_D_STATS, PORT=STATS4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_D_STATS, PORT=STATS4,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### ADC SWAP Statistics ##########################

NDStatsConfigure("STATSSWAP1", $(QSIZE), 0, "$(PORT)", 6, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_A_STATS, PORT=STATSSWAP1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADCSWAP_A_STATS, PORT=STATSSWAP1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSSWAP2", $(QSIZE), 0, "$(PORT)", 7, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_B_STATS, PORT=STATSSWAP2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADCSWAP_B_STATS, PORT=STATSSWAP2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSSWAP3", $(QSIZE), 0, "$(PORT)", 8, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_C_STATS, PORT=STATSSWAP3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADCSWAP_C_STATS, PORT=STATSSWAP3,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSSWAP4", $(QSIZE), 0, "$(PORT)", 9, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_D_STATS, PORT=STATSSWAP4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADCSWAP_D_STATS, PORT=STATSSWAP4,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### TBT AMP Statistics ##########################

NDStatsConfigure("STATSTBT_A", $(QSIZE), 0, "$(PORT)", 12, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_A_STATS, PORT=STATSTBT_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_A_STATS, PORT=STATSTBT_A,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_B", $(QSIZE), 0, "$(PORT)", 13, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_B_STATS, PORT=STATSTBT_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_B_STATS, PORT=STATSTBT_B,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_C", $(QSIZE), 0, "$(PORT)", 14, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_C_STATS, PORT=STATSTBT_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_C_STATS, PORT=STATSTBT_C,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_D", $(QSIZE), 0, "$(PORT)", 15, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_D_STATS, PORT=STATSTBT_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_D_STATS, PORT=STATSTBT_D,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### TBT POS Statistics ##########################

NDStatsConfigure("STATSTBT_X", $(QSIZE), 0, "$(PORT)", 18, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_X_STATS, PORT=STATSTBT_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_X_STATS, PORT=STATSTBT_X,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_Y", $(QSIZE), 0, "$(PORT)", 19, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_Y_STATS, PORT=STATSTBT_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_Y_STATS, PORT=STATSTBT_Y,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_Q", $(QSIZE), 0, "$(PORT)", 20, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_Q_STATS, PORT=STATSTBT_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_Q_STATS, PORT=STATSTBT_Q,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSTBT_SUM", $(QSIZE), 0, "$(PORT)", 21, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_SUM_STATS, PORT=STATSTBT_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}TBT_SUM_STATS, PORT=STATSTBT_SUM,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### FOFB AMP Statistics ##########################

NDStatsConfigure("STATSFOFB_A", $(QSIZE), 0, "$(PORT)", 30, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_A_STATS, PORT=STATSFOFB_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=30")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_A_STATS, PORT=STATSFOFB_A,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_B", $(QSIZE), 0, "$(PORT)", 31, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_B_STATS, PORT=STATSFOFB_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=31")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_B_STATS, PORT=STATSFOFB_B,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_C", $(QSIZE), 0, "$(PORT)", 32, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_C_STATS, PORT=STATSFOFB_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=32")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_C_STATS, PORT=STATSFOFB_C,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_D", $(QSIZE), 0, "$(PORT)", 33, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_D_STATS, PORT=STATSFOFB_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=33")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_D_STATS, PORT=STATSFOFB_D,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

##################### FOFB POS Statistics ##########################

NDStatsConfigure("STATSFOFB_X", $(QSIZE), 0, "$(PORT)", 36, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_X_STATS, PORT=STATSFOFB_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=36")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_X_STATS, PORT=STATSFOFB_X,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_Y", $(QSIZE), 0, "$(PORT)", 37, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_Y_STATS, PORT=STATSFOFB_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=37")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_Y_STATS, PORT=STATSFOFB_Y,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_Q", $(QSIZE), 0, "$(PORT)", 38, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_Q_STATS, PORT=STATSFOFB_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=38")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_Q_STATS, PORT=STATSFOFB_Q,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATSFOFB_SUM", $(QSIZE), 0, "$(PORT)", 39, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_SUM_STATS, PORT=STATSFOFB_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=39")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}FOFB_SUM_STATS, PORT=STATSFOFB_SUM,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

############################## ADC PM Statistics ##########################

NDStatsConfigure("STATS_PM1", $(QSIZE), 0, "$(PORT)", 48, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_PM_A_STATS, PORT=STATS_PM1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=48")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_PM_A_STATS, PORT=STATS_PM1,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS_PM2", $(QSIZE), 0, "$(PORT)", 49, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_PM_B_STATS, PORT=STATS_PM2,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=49")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_PM_B_STATS, PORT=STATS_PM2,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS_PM3", $(QSIZE), 0, "$(PORT)", 50, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_PM_C_STATS, PORT=STATS_PM3,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=50")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_PM_C_STATS, PORT=STATS_PM3,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")

NDStatsConfigure("STATS_PM4", $(QSIZE), 0, "$(PORT)", 51, 0, 0)
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_PM_D_STATS, PORT=STATS_PM4,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=51")
dbLoadRecords("$(AREA_DETECTOR)/ADApp/Db/NDStats.template",     "P=${P},R=${R}ADC_PM_D_STATS, PORT=STATS_PM4,ADDR=0,TIMEOUT=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(TSPOINTS)")
