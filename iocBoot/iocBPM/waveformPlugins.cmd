##################### GEN AMP Waveforms ##########################

NDStdArraysConfigure("GEN_A_Array", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_A,PORT=GEN_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_A,PORT=GEN_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_B_Array", $(QSIZE), 0, "$(PORT)", 1)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_B,PORT=GEN_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_B,PORT=GEN_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_C_Array", $(QSIZE), 0, "$(PORT)", 2)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_C,PORT=GEN_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_C,PORT=GEN_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_D_Array", $(QSIZE), 0, "$(PORT)", 3)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_D,PORT=GEN_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_D,PORT=GEN_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### GEN POS Waveforms ##########################

NDStdArraysConfigure("GEN_UncalX_Array", $(QSIZE), 0, "$(PORT)", 6)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_UncalX,PORT=GEN_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalX,PORT=GEN_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_X,ARRAY_NAME_UNCAL=_UncalX,ARRAY_NAME_CAL=_CalX,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=X")

NDStdArraysConfigure("GEN_UncalY_Array", $(QSIZE), 0, "$(PORT)", 7)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_UncalY,PORT=GEN_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalY,PORT=GEN_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Y,ARRAY_NAME_UNCAL=_UncalY,ARRAY_NAME_CAL=_CalY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Y")

dbLoadRecords("${TOP}/db/BPMPosCalcXY.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_XY,ARRAY_NAME_UNCAL=_UncalXY,ARRAY_NAME_CAL=_CalXY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_XOFFSET=PosX,POS_YOFFSET=PosY,GLOBAL_ARRAY_NAME_X=X,GLOBAL_ARRAY_NAME_Y=Y,ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("GEN_UncalQ_Array", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_UncalQ,PORT=GEN_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalQ,PORT=GEN_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Q")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosQ,GLOBAL_ARRAY_NAME=Q,ASUB_ROUTINE=bpmPolyCalQProcessAsub")

NDStdArraysConfigure("GEN_UncalSUM_Array", $(QSIZE), 0, "$(PORT)", 9)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}GEN_UncalSUM,PORT=GEN_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_UncalSUM,PORT=GEN_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=SUM")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosSUM,GLOBAL_ARRAY_NAME=SUM,ASUB_ROUTINE=bpmPolyCalSUMProcessAsub")

##################### AMP PM Waveforms ##########################

NDStdArraysConfigure("PM_A_Array", $(QSIZE), 0, "$(PORT)", 12)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_B_Array", $(QSIZE), 0, "$(PORT)", 13)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_C_Array", $(QSIZE), 0, "$(PORT)", 14)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_D_Array", $(QSIZE), 0, "$(PORT)", 15)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### POS PM Waveforms ##########################

NDStdArraysConfigure("PM_UncalX_Array", $(QSIZE), 0, "$(PORT)", 18)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_UncalX,PORT=PM_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_UncalX,PORT=PM_UncalX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_X,ARRAY_NAME_UNCAL=_UncalX,ARRAY_NAME_CAL=_CalX,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=X")

NDStdArraysConfigure("PM_UncalY_Array", $(QSIZE), 0, "$(PORT)", 19)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_UncalY,PORT=PM_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_UncalY,PORT=PM_UncalY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Y,ARRAY_NAME_UNCAL=_UncalY,ARRAY_NAME_CAL=_CalY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Y")

dbLoadRecords("${TOP}/db/BPMPosCalcXY.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_XY,ARRAY_NAME_UNCAL=_UncalXY,ARRAY_NAME_CAL=_CalXY,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_XOFFSET=PosX,POS_YOFFSET=PosY,GLOBAL_ARRAY_NAME_X=X,GLOBAL_ARRAY_NAME_Y=Y,ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("PM_UncalQ_Array", $(QSIZE), 0, "$(PORT)", 20)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_UncalQ,PORT=PM_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_UncalQ,PORT=PM_UncalQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Q")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Q,ARRAY_NAME_UNCAL=_UncalQ,ARRAY_NAME_CAL=_CalQ,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosQ,GLOBAL_ARRAY_NAME=Q,ASUB_ROUTINE=bpmPolyCalQProcessAsub")

NDStdArraysConfigure("PM_UncalSUM_Array", $(QSIZE), 0, "$(PORT)", 21)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}PM_UncalSUM,PORT=PM_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_UncalSUM,PORT=PM_UncalSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=SUM")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_SUM,ARRAY_NAME_UNCAL=_UncalSUM,ARRAY_NAME_CAL=_CalSUM,ARRAY_NAME_XUNCAL=_UncalX,ARRAY_NAME_YUNCAL=_UncalY,ARRAY_NAME_QUNCAL=_UncalQ,ARRAY_NAME_SUMUNCAL=_UncalSUM,ARRAY_NAME_XCAL=_CalX,ARRAY_NAME_YCAL=_CalY,ARRAY_NAME_QCAL=_CalQ,ARRAY_NAME_SUMCAL=_CalSUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosSUM,GLOBAL_ARRAY_NAME=SUM,ASUB_ROUTINE=bpmPolyCalSUMProcessAsub")

##################### Raw Data Single Pass Waveforms ##########################

NDStdArraysConfigure("SP_A_Array", $(QSIZE), 0, "$(PORT)", 24)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}SP_A,PORT=SP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_A,PORT=SP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_B_Array", $(QSIZE), 0, "$(PORT)", 25)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}SP_B,PORT=SP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=25")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_B,PORT=SP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_C_Array", $(QSIZE), 0, "$(PORT)", 26)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}SP_C,PORT=SP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=26")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_C,PORT=SP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_D_Array", $(QSIZE), 0, "$(PORT)", 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}SP_D,PORT=SP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=27")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_D,PORT=SP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### Monit AMP/POS/POSFake Waveforms ##########################

NDStdArraysConfigure("MONIT_A_Array", $(QSIZE), 0, "$(PORT)", 30)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_A,PORT=MONIT_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=30")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_A,PORT=MONIT_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_AArrayData", "$(P)$(R)AmplA-Mon")

NDStdArraysConfigure("MONIT_B_Array", $(QSIZE), 0, "$(PORT)", 31)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_B,PORT=MONIT_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=31")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_B,PORT=MONIT_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_BArrayData", "$(P)$(R)AmplB-Mon")

NDStdArraysConfigure("MONIT_C_Array", $(QSIZE), 0, "$(PORT)", 32)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_C,PORT=MONIT_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=32")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_C,PORT=MONIT_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_CArrayData", "$(P)$(R)AmplC-Mon")

NDStdArraysConfigure("MONIT_D_Array", $(QSIZE), 0, "$(PORT)", 33)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_D,PORT=MONIT_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=33")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_D,PORT=MONIT_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_DArrayData", "$(P)$(R)AmplD-Mon")

NDStdArraysConfigure("MONIT_X_Array", $(QSIZE), 0, "$(PORT)", 34)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_X,PORT=MONIT_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=34")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_X,PORT=MONIT_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_XArrayData", "$(P)$(R)PosX-Mon")

NDStdArraysConfigure("MONIT_Y_Array", $(QSIZE), 0, "$(PORT)", 35)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_Y,PORT=MONIT_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=35")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_Y,PORT=MONIT_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_YArrayData", "$(P)$(R)PosY-Mon")

NDStdArraysConfigure("MONIT_Q_Array", $(QSIZE), 0, "$(PORT)", 36)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_Q,PORT=MONIT_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=36")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_Q,PORT=MONIT_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_QArrayData", "$(P)$(R)PosQ-Mon")

NDStdArraysConfigure("MONIT_SUM_Array", $(QSIZE), 0, "$(PORT)", 37)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_SUM,PORT=MONIT_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=37")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_SUM,PORT=MONIT_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_SUMArrayData", "$(P)$(R)Sum-Mon")

NDStdArraysConfigure("MONIT_FAKEX_Array", $(QSIZE), 0, "$(PORT)", 38)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_FAKEX,PORT=MONIT_FAKEX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=38")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_FAKEX,PORT=MONIT_FAKEX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_FAKEXArrayData", "$(P)$(R)PosXFake-Mon")

NDStdArraysConfigure("MONIT_FAKEY_Array", $(QSIZE), 0, "$(PORT)", 39)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDPluginBase.template","P=${P},R=${R}MONIT_FAKEY,PORT=MONIT_FAKEY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=39")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_FAKEY,PORT=MONIT_FAKEY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_FAKEYArrayData", "$(P)$(R)PosYFake-Mon")

##################### DSP configuration ##########################

dbLoadRecords("${TOP}/db/BPMDsp.template", "P=${P}, R=${R}, GLOBAL_ARRAY_NAME_X=X, GLOBAL_ARRAY_NAME_Y=Y, GLOBAL_ARRAY_NAME_Q=Q, GLOBAL_ARRAY_NAME_SUM=SUM, PORT=$(PORT), ADDR=0, TIMEOUT=1")
