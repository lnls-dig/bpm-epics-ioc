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

NDStdArraysConfigure("GEN_UncalX_Array", $(QSIZE), 0, "$(PORT)", 6)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_UncalX,PORT=GEN_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalX,PORT=GEN_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=GEN,ARRAY_NAME=_X,ARRAY_NAME_UNCAL=_UncalX,ARRAY_NAME_CAL=_CalX,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

NDStdArraysConfigure("GEN_UncalY_Array", $(QSIZE), 0, "$(PORT)", 7)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_UncalY,PORT=GEN_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalY,PORT=GEN_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=GEN,ARRAY_NAME=_Y,ARRAY_NAME_UNCAL=_UncalY,ARRAY_NAME_CAL=_CalY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

dbLoadRecords("${TOP}/BPMApp/Db/BPMPosCalcXY.template", "P=${P},R=${R},R=${R},ACQ=GEN,ARRAY_NAME=_XY,ARRAY_NAME_UNCAL=_UncalXY,ARRAY_NAME_CAL=_CalXY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("GEN_UncalQ_Array", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_UncalQ,PORT=GEN_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalQ,PORT=GEN_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=GEN,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

NDStdArraysConfigure("GEN_UncalSUM_Array", $(QSIZE), 0, "$(PORT)", 9)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}GEN_UncalSUM,PORT=GEN_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalSUM,PORT=GEN_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=GEN,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

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

NDStdArraysConfigure("PM_UncalX_Array", $(QSIZE), 0, "$(PORT)", 18)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_UncalX,PORT=PM_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_UncalX,PORT=PM_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=PM,ARRAY_NAME=_X,ARRAY_NAME_UNCAL=_UncalX,ARRAY_NAME_CAL=_CalX,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

NDStdArraysConfigure("PM_UncalY_Array", $(QSIZE), 0, "$(PORT)", 19)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_UncalY,PORT=PM_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_UncalY,PORT=PM_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=PM,ARRAY_NAME=_Y,ARRAY_NAME_UNCAL=_UncalY,ARRAY_NAME_CAL=_CalY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

dbLoadRecords("${TOP}/BPMApp/Db/BPMPosCalcXY.template", "P=${P},R=${R},R=${R},ACQ=PM,ARRAY_NAME=_XY,ARRAY_NAME_UNCAL=_UncalXY,ARRAY_NAME_CAL=_CalXY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("PM_UncalQ_Array", $(QSIZE), 0, "$(PORT)", 20)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_UncalQ,PORT=PM_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_UncalQ,PORT=PM_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=PM,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

NDStdArraysConfigure("PM_UncalSUM_Array", $(QSIZE), 0, "$(PORT)", 21)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_UncalSUM,PORT=PM_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_UncalSUM,PORT=PM_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/BPMApp/Db/BPMPos.template", "P=${P},R=${R},R=${R},ACQ=PM,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE)")

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
