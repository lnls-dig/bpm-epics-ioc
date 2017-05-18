##################### GEN AMP Waveforms ##########################

NDStdArraysConfigure("GEN_A_Array", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_A,PORT=GEN_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_A,PORT=GEN_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_B_Array", $(QSIZE), 0, "$(PORT)", 1)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_B,PORT=GEN_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_B,PORT=GEN_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_C_Array", $(QSIZE), 0, "$(PORT)", 2)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_C,PORT=GEN_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_C,PORT=GEN_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_D_Array", $(QSIZE), 0, "$(PORT)", 3)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_D,PORT=GEN_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_D,PORT=GEN_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### GEN POS Waveforms ##########################

NDStdArraysConfigure("GEN_X_Array", $(QSIZE), 0, "$(PORT)", 6)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_X,PORT=GEN_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_X,PORT=GEN_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_Y_Array", $(QSIZE), 0, "$(PORT)", 7)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_Y,PORT=GEN_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_Y,PORT=GEN_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_Q_Array", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_Q,PORT=GEN_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_Q,PORT=GEN_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_SUM_Array", $(QSIZE), 0, "$(PORT)", 9)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_SUM,PORT=GEN_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_SUM,PORT=GEN_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### AMP PM Waveforms ##########################

NDStdArraysConfigure("PM_A_Array", $(QSIZE), 0, "$(PORT)", 12)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_B_Array", $(QSIZE), 0, "$(PORT)", 13)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_C_Array", $(QSIZE), 0, "$(PORT)", 14)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_D_Array", $(QSIZE), 0, "$(PORT)", 15)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### POS PM Waveforms ##########################

NDStdArraysConfigure("PM_X_Array", $(QSIZE), 0, "$(PORT)", 18)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_X,PORT=PM_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_X,PORT=PM_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_Y_Array", $(QSIZE), 0, "$(PORT)", 19)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_Y,PORT=PM_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_Y,PORT=PM_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_Q_Array", $(QSIZE), 0, "$(PORT)", 20)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_Q,PORT=PM_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_Q,PORT=PM_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_SUM_Array", $(QSIZE), 0, "$(PORT)", 21)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_SUM,PORT=PM_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_SUM,PORT=PM_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### Raw Data Single Pass Waveforms ##########################

NDStdArraysConfigure("SP_A_Array", $(QSIZE), 0, "$(PORT)", 24)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}SP_A,PORT=SP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}SP_A,PORT=SP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_B_Array", $(QSIZE), 0, "$(PORT)", 25)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}SP_B,PORT=SP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=25")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}SP_B,PORT=SP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_C_Array", $(QSIZE), 0, "$(PORT)", 26)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}SP_C,PORT=SP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=26")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}SP_C,PORT=SP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_D_Array", $(QSIZE), 0, "$(PORT)", 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}SP_D,PORT=SP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=27")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}SP_D,PORT=SP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
