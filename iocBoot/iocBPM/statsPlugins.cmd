# Create statistics plugins

##################### GEN AMP Statistics ##########################

NDStatsConfigure("STATSGEN_A", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_A_STATS, PORT=STATSGEN_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSGEN_B", $(QSIZE), 0, "$(PORT)", 1, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_B_STATS, PORT=STATSGEN_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSGEN_C", $(QSIZE), 0, "$(PORT)", 2, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_C_STATS, PORT=STATSGEN_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSGEN_D", $(QSIZE), 0, "$(PORT)", 3, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_D_STATS, PORT=STATSGEN_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

##################### GEN POS Statistics ##########################

NDStatsConfigure("STATSGEN_X", $(QSIZE), 0, "$(PORT)", 6, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_X_STATS, PORT=STATSGEN_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_X_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosX-Mon")

NDStatsConfigure("STATSGEN_Y", $(QSIZE), 0, "$(PORT)", 7, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_Y_STATS, PORT=STATSGEN_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_Y_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosY-Mon")

NDStatsConfigure("STATSGEN_Q", $(QSIZE), 0, "$(PORT)", 8, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_Q_STATS, PORT=STATSGEN_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_Q_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosQ-Mon")

NDStatsConfigure("STATSGEN_SUM", $(QSIZE), 0, "$(PORT)", 9, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_SUM_STATS, PORT=STATSGEN_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_SUM_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQSum-Mon")

##################### PM AMP Statistics ##########################

NDStatsConfigure("STATSPM_A", $(QSIZE), 0, "$(PORT)", 12, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_A_STATS, PORT=STATSPM_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_B", $(QSIZE), 0, "$(PORT)", 13, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_B_STATS, PORT=STATSPM_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_C", $(QSIZE), 0, "$(PORT)", 14, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_C_STATS, PORT=STATSPM_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_D", $(QSIZE), 0, "$(PORT)", 15, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_D_STATS, PORT=STATSPM_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

##################### PM POS Statistics ##########################

NDStatsConfigure("STATSPM_X", $(QSIZE), 0, "$(PORT)", 18, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_X_STATS, PORT=STATSPM_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_Y", $(QSIZE), 0, "$(PORT)", 19, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_Y_STATS, PORT=STATSPM_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_Q", $(QSIZE), 0, "$(PORT)", 20, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_Q_STATS, PORT=STATSPM_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSPM_SUM", $(QSIZE), 0, "$(PORT)", 21, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_SUM_STATS, PORT=STATSPM_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

##################### SP AMP Statistics ##########################

NDStatsConfigure("STATSSP_A", $(QSIZE), 0, "$(PORT)", 24, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_A_STATS, PORT=STATSSP_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSSP_B", $(QSIZE), 0, "$(PORT)", 25, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_B_STATS, PORT=STATSSP_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=25,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSSP_C", $(QSIZE), 0, "$(PORT)", 26, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_C_STATS, PORT=STATSSP_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=26,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")

NDStatsConfigure("STATSSP_D", $(QSIZE), 0, "$(PORT)", 27, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_D_STATS, PORT=STATSSP_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=27,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
