# Create statistics plugins

##################### GEN AMP Statistics ##########################

NDStatsConfigure("STATSGEN_A", $(QSIZE), 0, "$(PORT)", 0, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_A_STATS, PORT=STATSGEN_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_A_TS", $(QSIZE), 0, "STATSGEN_A", 0, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_A_TSSTATS, PORT=STATSGEN_A_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_A,NDARRAY_ADDR=0,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSGEN_B", $(QSIZE), 0, "$(PORT)", 1, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_B_STATS, PORT=STATSGEN_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_B_TS", $(QSIZE), 0, "STATSGEN_B", 1, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_B_TSSTATS, PORT=STATSGEN_B_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_B,NDARRAY_ADDR=1,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSGEN_C", $(QSIZE), 0, "$(PORT)", 2, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_C_STATS, PORT=STATSGEN_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_C_TS", $(QSIZE), 0, "STATSGEN_C", 2, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_C_TSSTATS, PORT=STATSGEN_C_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_C,NDARRAY_ADDR=2,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSGEN_D", $(QSIZE), 0, "$(PORT)", 3, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_D_STATS, PORT=STATSGEN_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_D_TS", $(QSIZE), 0, "STATSGEN_D", 3, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_D_TSSTATS, PORT=STATSGEN_D_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_D,NDARRAY_ADDR=3,NCHANS=$(NCHANS),ENABLED=0")

##################### GEN POS Statistics ##########################

NDStatsConfigure("STATSGEN_X", $(QSIZE), 0, "$(PORT)", 6, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_X_STATS, PORT=STATSGEN_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_X_TS", $(QSIZE), 0, "STATSGEN_X", 6, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_X_TSSTATS, PORT=STATSGEN_X_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_X,NDARRAY_ADDR=6,NCHANS=$(NCHANS),ENABLED=0")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_X_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosX-Mon")

NDStatsConfigure("STATSGEN_Y", $(QSIZE), 0, "$(PORT)", 7, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_Y_STATS, PORT=STATSGEN_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_Y_TS", $(QSIZE), 0, "STATSGEN_Y", 7, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_Y_TSSTATS, PORT=STATSGEN_Y_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_Y,NDARRAY_ADDR=7,NCHANS=$(NCHANS),ENABLED=0")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_Y_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosY-Mon")

NDStatsConfigure("STATSGEN_Q", $(QSIZE), 0, "$(PORT)", 8, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_Q_STATS, PORT=STATSGEN_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_Q_TS", $(QSIZE), 0, "STATSGEN_Q", 8, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_Q_TSSTATS, PORT=STATSGEN_Q_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_Q,NDARRAY_ADDR=8,NCHANS=$(NCHANS),ENABLED=0")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_Q_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQPosQ-Mon")

NDStatsConfigure("STATSGEN_SUM", $(QSIZE), 0, "$(PORT)", 9, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}GEN_SUM_STATS, PORT=STATSGEN_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSGEN_SUM_TS", $(QSIZE), 0, "STATSGEN_SUM", 9, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}GEN_SUM_TSSTATS, PORT=STATSGEN_SUM_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSGEN_SUM,NDARRAY_ADDR=9,NCHANS=$(NCHANS),ENABLED=0")
# Alias
dbLoadRecords("${TOP}/db/BPMStatsPluginAliases.template", "P=${P}, R=${R}, STATS_P=GEN_SUM_STATS, NEW_P=${P}, NEW_R=${R}, NEW_FIELD=ACQSum-Mon")

##################### PM AMP Statistics ##########################

NDStatsConfigure("STATSPM_A", $(QSIZE), 0, "$(PORT)", 12, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_A_STATS, PORT=STATSPM_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_A_TS", $(QSIZE), 0, "STATSPM_A", 12, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_A_TSSTATS, PORT=STATSPM_A_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_A,NDARRAY_ADDR=12,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_B", $(QSIZE), 0, "$(PORT)", 13, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_B_STATS, PORT=STATSPM_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_B_TS", $(QSIZE), 0, "STATSPM_B", 13, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_B_TSSTATS, PORT=STATSPM_B_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_B,NDARRAY_ADDR=13,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_C", $(QSIZE), 0, "$(PORT)", 14, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_C_STATS, PORT=STATSPM_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_C_TS", $(QSIZE), 0, "STATSPM_C", 14, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_C_TSSTATS, PORT=STATSPM_C_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_C,NDARRAY_ADDR=14,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_D", $(QSIZE), 0, "$(PORT)", 15, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_D_STATS, PORT=STATSPM_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_D_TS", $(QSIZE), 0, "STATSPM_D", 15, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_D_TSSTATS, PORT=STATSPM_D_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_D,NDARRAY_ADDR=15,NCHANS=$(NCHANS),ENABLED=0")

##################### PM POS Statistics ##########################

NDStatsConfigure("STATSPM_X", $(QSIZE), 0, "$(PORT)", 18, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_X_STATS, PORT=STATSPM_X,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_X_TS", $(QSIZE), 0, "STATSPM_X", 18, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_X_TSSTATS, PORT=STATSPM_X_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_X,NDARRAY_ADDR=18,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_Y", $(QSIZE), 0, "$(PORT)", 19, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_Y_STATS, PORT=STATSPM_Y,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_Y_TS", $(QSIZE), 0, "STATSPM_Y", 19, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_Y_TSSTATS, PORT=STATSPM_Y_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_Y,NDARRAY_ADDR=19,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_Q", $(QSIZE), 0, "$(PORT)", 20, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_Q_STATS, PORT=STATSPM_Q,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_Q_TS", $(QSIZE), 0, "STATSPM_Q", 20, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_Q_TSSTATS, PORT=STATSPM_Q_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_Q,NDARRAY_ADDR=20,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSPM_SUM", $(QSIZE), 0, "$(PORT)", 21, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}PM_SUM_STATS, PORT=STATSPM_SUM,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSPM_SUM_TS", $(QSIZE), 0, "STATSPM_SUM", 21, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}PM_SUM_TSSTATS, PORT=STATSPM_SUM_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSPM_SUM,NDARRAY_ADDR=21,NCHANS=$(NCHANS),ENABLED=0")

##################### SP AMP Statistics ##########################

NDStatsConfigure("STATSSP_A", $(QSIZE), 0, "$(PORT)", 24, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_A_STATS, PORT=STATSSP_A,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSSP_A_TS", $(QSIZE), 0, "STATSSP_A", 24, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}SP_A_TSSTATS, PORT=STATSSP_A_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSSP_A,NDARRAY_ADDR=24,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSSP_B", $(QSIZE), 0, "$(PORT)", 25, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_B_STATS, PORT=STATSSP_B,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=25,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSSP_B_TS", $(QSIZE), 0, "STATSSP_B", 25, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}SP_B_TSSTATS, PORT=STATSSP_B_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSSP_B,NDARRAY_ADDR=25,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSSP_C", $(QSIZE), 0, "$(PORT)", 26, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_C_STATS, PORT=STATSSP_C,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=26,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSSP_C_TS", $(QSIZE), 0, "STATSSP_C", 26, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}SP_C_TSSTATS, PORT=STATSSP_C_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSSP_C,NDARRAY_ADDR=26,NCHANS=$(NCHANS),ENABLED=0")

NDStatsConfigure("STATSSP_D", $(QSIZE), 0, "$(PORT)", 27, 0, 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStats.template",     "P=${P},R=${R}SP_D_STATS, PORT=STATSSP_D,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=27,HIST_SIZE=256,XSIZE=$(RING_SIZE),YSIZE=0,NCHANS=$(NCHANS)")
NDTimeSeriesConfigure("STATSSP_D_TS", $(QSIZE), 0, "STATSSP_D", 27, 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDTimeSeries.template",  "P=${P},R=${R}SP_D_TSSTATS, PORT=STATSSP_D_TS,ADDR=0,TIMEOUT=1,NDARRAY_PORT=STATSSP_D,NDARRAY_ADDR=27,NCHANS=$(NCHANS),ENABLED=0")
