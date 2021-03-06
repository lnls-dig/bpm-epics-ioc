##################### GEN AMP Waveforms ##########################

NDStdArraysConfigure("GEN_A_Array", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_A,PORT=GEN_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_B_Array", $(QSIZE), 0, "$(PORT)", 1)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_B,PORT=GEN_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_C_Array", $(QSIZE), 0, "$(PORT)", 2)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_C,PORT=GEN_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("GEN_D_Array", $(QSIZE), 0, "$(PORT)", 3)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_D,PORT=GEN_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### GEN POS Waveforms ##########################

NDStdArraysConfigure("GEN_RawX_Array", $(QSIZE), 0, "$(PORT)", 6)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_RawX,PORT=GEN_RawX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_X,ARRAY_NAME_RAW=_RawX,ARRAY_NAME_POLY=_PolyX,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffX,POS_GAIN=PosKx-RB,POS_OFFSET=PosXOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=XY")

NDStdArraysConfigure("GEN_RawY_Array", $(QSIZE), 0, "$(PORT)", 7)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_RawY,PORT=GEN_RawY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Y,ARRAY_NAME_RAW=_RawY,ARRAY_NAME_POLY=_PolyY,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffY,POS_GAIN=PosKy-RB,POS_OFFSET=PosYOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=XY")

dbLoadRecords("${TOP}/db/BPMPosCalcXY.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_XY,ARRAY_NAME_RAW=_RawXY,ARRAY_NAME_POLY=_PolyXY,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_XOFFSET=PosXOffset-RB,POS_YOFFSET=PosYOffset-RB,GLOBAL_ARRAY_NAME_X=X,GLOBAL_ARRAY_NAME_Y=Y,GLOBAL_ARRAY_NAME_XY=XY,ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("GEN_RawQ_Array", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_RawQ,PORT=GEN_RawQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Q,ARRAY_NAME_RAW=_RawQ,ARRAY_NAME_POLY=_PolyQ,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffQ,POS_GAIN=PosKq-RB,POS_OFFSET=PosQOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Q")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_Q,ARRAY_NAME_RAW=_RawQ,ARRAY_NAME_POLY=_PolyQ,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosQOffset-RB,GLOBAL_ARRAY_NAME=Q,ASUB_ROUTINE=bpmPolyCalQProcessAsub")

NDStdArraysConfigure("GEN_RawSUM_Array", $(QSIZE), 0, "$(PORT)", 9)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}GEN_RawSUM,PORT=GEN_RawSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_SUM,ARRAY_NAME_RAW=_RawSUM,ARRAY_NAME_POLY=_PolySUM,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffSUM,POS_GAIN=PosKsum-RB,POS_OFFSET=PosSumOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=SUM")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=GEN,ARRAY_NAME=_SUM,ARRAY_NAME_RAW=_RawSUM,ARRAY_NAME_POLY=_PolySUM,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosSumOffset-RB,GLOBAL_ARRAY_NAME=SUM,ASUB_ROUTINE=bpmPolyCalSUMProcessAsub")

##################### AMP PM Waveforms ##########################

NDStdArraysConfigure("PM_A_Array", $(QSIZE), 0, "$(PORT)", 12)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_B_Array", $(QSIZE), 0, "$(PORT)", 13)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_C_Array", $(QSIZE), 0, "$(PORT)", 14)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_D_Array", $(QSIZE), 0, "$(PORT)", 15)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### POS PM Waveforms ##########################

NDStdArraysConfigure("PM_RawX_Array", $(QSIZE), 0, "$(PORT)", 18)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_RawX,PORT=PM_RawX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_X,ARRAY_NAME_RAW=_RawX,ARRAY_NAME_POLY=_PolyX,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffX,POS_GAIN=PosKx-RB,POS_OFFSET=PosXOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=XY")

NDStdArraysConfigure("PM_RawY_Array", $(QSIZE), 0, "$(PORT)", 19)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_RawY,PORT=PM_RawY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Y,ARRAY_NAME_RAW=_RawY,ARRAY_NAME_POLY=_PolyY,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffY,POS_GAIN=PosKy-RB,POS_OFFSET=PosYOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=XY")

dbLoadRecords("${TOP}/db/BPMPosCalcXY.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_XY,ARRAY_NAME_RAW=_RawXY,ARRAY_NAME_POLY=_PolyXY,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_XOFFSET=PosXOffset-RB,POS_YOFFSET=PosYOffset-RB,GLOBAL_ARRAY_NAME_X=X,GLOBAL_ARRAY_NAME_Y=Y,GLOBAL_ARRAY_NAME_XY=XY,ASUB_ROUTINE=bpmPolyCalXYProcessAsub")

NDStdArraysConfigure("PM_RawQ_Array", $(QSIZE), 0, "$(PORT)", 20)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_RawQ,PORT=PM_RawQ_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Q,ARRAY_NAME_RAW=_RawQ,ARRAY_NAME_POLY=_PolyQ,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffQ,POS_GAIN=PosKq-RB,POS_OFFSET=PosQOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=Q")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_Q,ARRAY_NAME_RAW=_RawQ,ARRAY_NAME_POLY=_PolyQ,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosQOffset-RB,GLOBAL_ARRAY_NAME=Q,ASUB_ROUTINE=bpmPolyCalQProcessAsub")

NDStdArraysConfigure("PM_RawSUM_Array", $(QSIZE), 0, "$(PORT)", 21)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}PM_RawSUM,PORT=PM_RawSUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
dbLoadRecords("${TOP}/db/BPMPos.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_SUM,ARRAY_NAME_RAW=_RawSUM,ARRAY_NAME_POLY=_PolySUM,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_GOFF=_GOffSUM,POS_GAIN=PosKsum-RB,POS_OFFSET=PosSumOffset-RB,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),GLOBAL_ARRAY_NAME=SUM")

dbLoadRecords("${TOP}/db/BPMPosCalcSingle.template", "P=${P},R=${R},ACQ=PM,ARRAY_NAME=_SUM,ARRAY_NAME_RAW=_RawSUM,ARRAY_NAME_POLY=_PolySUM,ARRAY_NAME_XRAW=_RawX,ARRAY_NAME_YRAW=_RawY,ARRAY_NAME_QRAW=_RawQ,ARRAY_NAME_SUMRAW=_RawSUM,ARRAY_NAME_XPOLY=_PolyX,ARRAY_NAME_YPOLY=_PolyY,ARRAY_NAME_QPOLY=_PolyQ,ARRAY_NAME_SUMPOLY=_PolySUM,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS),COEFF_ARRAY_SIZE=$(COEFF_ARRAY_SIZE),POS_OFFSET=PosSumOffset-RB,GLOBAL_ARRAY_NAME=SUM,ASUB_ROUTINE=bpmPolyCalSUMProcessAsub")

##################### Raw Data Single Pass Waveforms ##########################

NDStdArraysConfigure("SP_A_Array", $(QSIZE), 0, "$(PORT)", 24)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_A,PORT=SP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=24,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_B_Array", $(QSIZE), 0, "$(PORT)", 25)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_B,PORT=SP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=25,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_C_Array", $(QSIZE), 0, "$(PORT)", 26)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_C,PORT=SP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=26,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("SP_D_Array", $(QSIZE), 0, "$(PORT)", 27)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}SP_D,PORT=SP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=27,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### Monit AMP/POS/POSFake Waveforms ##########################

NDStdArraysConfigure("MONIT_A_Array", $(QSIZE), 0, "$(PORT)", 30)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_A,PORT=MONIT_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=30,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_AArrayData", "$(P)$(R)AmplA-Mon")

NDStdArraysConfigure("MONIT_B_Array", $(QSIZE), 0, "$(PORT)", 31)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_B,PORT=MONIT_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=31,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_BArrayData", "$(P)$(R)AmplB-Mon")

NDStdArraysConfigure("MONIT_C_Array", $(QSIZE), 0, "$(PORT)", 32)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_C,PORT=MONIT_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=32,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
#alias( "${P}${R}MONIT_CArrayData", "$(P)$(R)AmplC-Mon")

NDStdArraysConfigure("MONIT_D_Array", $(QSIZE), 0, "$(PORT)", 33)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_D,PORT=MONIT_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=33,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_DArrayData", "$(P)$(R)AmplD-Mon")

NDStdArraysConfigure("MONIT_X_Array", $(QSIZE), 0, "$(PORT)", 34)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_X,PORT=MONIT_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=34,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_XArrayData", "$(P)$(R)PosX-Mon")

NDStdArraysConfigure("MONIT_Y_Array", $(QSIZE), 0, "$(PORT)", 35)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_Y,PORT=MONIT_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=35,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_YArrayData", "$(P)$(R)PosY-Mon")

NDStdArraysConfigure("MONIT_Q_Array", $(QSIZE), 0, "$(PORT)", 36)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_Q,PORT=MONIT_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=36,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_QArrayData", "$(P)$(R)PosQ-Mon")

NDStdArraysConfigure("MONIT_SUM_Array", $(QSIZE), 0, "$(PORT)", 37)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_SUM,PORT=MONIT_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=37,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_SUMArrayData", "$(P)$(R)Sum-Mon")

NDStdArraysConfigure("MONIT_FAKEX_Array", $(QSIZE), 0, "$(PORT)", 38)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_FAKEX,PORT=MONIT_FAKEX_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=38,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_FAKEXArrayData", "$(P)$(R)PosXFake-Mon")

NDStdArraysConfigure("MONIT_FAKEY_Array", $(QSIZE), 0, "$(PORT)", 39)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/db/NDStdArrays.template", "P=${P},R=${R}MONIT_FAKEY,PORT=MONIT_FAKEY_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=39,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=1")
# alias( "${P}${R}MONIT_FAKEYArrayData", "$(P)$(R)PosYFake-Mon")

##################### DSP configuration ##########################

dbLoadRecords("${TOP}/db/BPMDsp.template", "P=${P}, R=${R}, GLOBAL_ARRAY_NAME_X=X, GLOBAL_ARRAY_NAME_Y=Y, GLOBAL_ARRAY_NAME_XY=XY, GLOBAL_ARRAY_NAME_Q=Q, GLOBAL_ARRAY_NAME_SUM=SUM, PORT=$(PORT), ADDR=0, TIMEOUT=1")
