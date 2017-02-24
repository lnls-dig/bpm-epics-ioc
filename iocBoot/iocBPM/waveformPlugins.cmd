##################### ADC Waveforms ##########################

NDStdArraysConfigure("ADC_A_Array", $(QSIZE), 0, "$(PORT)", 0)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_A,PORT=ADC_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=0")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADC_A,PORT=ADC_A_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADC_B_Array", $(QSIZE), 0, "$(PORT)", 1)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_B,PORT=ADC_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=1")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADC_B,PORT=ADC_B_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADC_C_Array", $(QSIZE), 0, "$(PORT)", 2)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_C,PORT=ADC_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=2")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADC_C,PORT=ADC_C_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADC_D_Array", $(QSIZE), 0, "$(PORT)", 3)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADC_D,PORT=ADC_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=3")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADC_D,PORT=ADC_D_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADCAMP_Freq_Array", $(QSIZE), 0, "$(PORT)", 5)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCAMP_Freq,PORT=ADCAMP_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=5")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCAMP_Freq,PORT=ADCAMP_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### ADC SWAP Waveforms ##########################

NDStdArraysConfigure("ADCSWAP_A_Array", $(QSIZE), 0, "$(PORT)", 6)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_A,PORT=ADCSWAP_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=6")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCSWAP_A,PORT=ADCSWAP_A_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADCSWAP_B_Array", $(QSIZE), 0, "$(PORT)", 7)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_B,PORT=ADCSWAP_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=7")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCSWAP_B,PORT=ADCSWAP_B_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADCSWAP_C_Array", $(QSIZE), 0, "$(PORT)", 8)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_C,PORT=ADCSWAP_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=8")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCSWAP_C,PORT=ADCSWAP_C_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADCSWAP_D_Array", $(QSIZE), 0, "$(PORT)", 9)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAP_D,PORT=ADCSWAP_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=9")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCSWAP_D,PORT=ADCSWAP_D_Array,ADDR=0,TIMEOUT=1,TYPE=Int16,FTVL=SHORT,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("ADCSWAPAMP_Freq_Array", $(QSIZE), 0, "$(PORT)", 11)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}ADCSWAPAMP_Freq,PORT=ADCSWAPAMP_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=11")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}ADCSWAPAMP_Freq,PORT=ADCSWAPAMP_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### TBT AMP Waveforms ##########################

NDStdArraysConfigure("TBT_A_Array", $(QSIZE), 0, "$(PORT)", 12)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_A,PORT=TBT_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=12")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_A,PORT=TBT_A_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_B_Array", $(QSIZE), 0, "$(PORT)", 13)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_B,PORT=TBT_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=13")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_B,PORT=TBT_B_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_C_Array", $(QSIZE), 0, "$(PORT)", 14)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_C,PORT=TBT_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=14")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_C,PORT=TBT_C_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_D_Array", $(QSIZE), 0, "$(PORT)", 15)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_D,PORT=TBT_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=15")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_D,PORT=TBT_D_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBTAMP_Freq_Array", $(QSIZE), 0, "$(PORT)", 17)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBTAMP_Freq,PORT=TBTAMP_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=17")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBTAMP_Freq,PORT=TBTAMP_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### TBT POS Waveforms ##########################

NDStdArraysConfigure("TBT_X_Array", $(QSIZE), 0, "$(PORT)", 18)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_X,PORT=TBT_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=18")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_X,PORT=TBT_X_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_Y_Array", $(QSIZE), 0, "$(PORT)", 19)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_Y,PORT=TBT_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=19")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_Y,PORT=TBT_Y_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_Q_Array", $(QSIZE), 0, "$(PORT)", 20)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_Q,PORT=TBT_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=20")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_Q,PORT=TBT_Q_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBT_SUM_Array", $(QSIZE), 0, "$(PORT)", 21)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBT_SUM,PORT=TBT_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=21")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBT_SUM,PORT=TBT_SUM_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("TBTPOS_Freq_Array", $(QSIZE), 0, "$(PORT)", 23)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}TBTPOS_Freq,PORT=TBTPOS_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=23")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}TBTPOS_Freq,PORT=TBTPOS_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### FOFB AMP Waveforms ##########################

NDStdArraysConfigure("FOFB_A_Array", $(QSIZE), 0, "$(PORT)", 30)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_A,PORT=FOFB_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=30")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_A,PORT=FOFB_A_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_B_Array", $(QSIZE), 0, "$(PORT)", 31)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_B,PORT=FOFB_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=31")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_B,PORT=FOFB_B_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_C_Array", $(QSIZE), 0, "$(PORT)", 32)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_C,PORT=FOFB_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=32")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_C,PORT=FOFB_C_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_D_Array", $(QSIZE), 0, "$(PORT)", 33)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_D,PORT=FOFB_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=33")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_D,PORT=FOFB_D_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFBAMP_Freq_Array", $(QSIZE), 0, "$(PORT)", 35)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFBAMP_Freq,PORT=FOFBAMP_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=35")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFBAMP_Freq,PORT=FOFBAMP_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### FOFB POS Waveforms ##########################

NDStdArraysConfigure("FOFB_X_Array", $(QSIZE), 0, "$(PORT)", 36)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_X,PORT=FOFB_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=36")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_X,PORT=FOFB_X_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_Y_Array", $(QSIZE), 0, "$(PORT)", 37)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_Y,PORT=FOFB_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=37")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_Y,PORT=FOFB_Y_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_Q_Array", $(QSIZE), 0, "$(PORT)", 38)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_Q,PORT=FOFB_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=38")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_Q,PORT=FOFB_Q_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFB_SUM_Array", $(QSIZE), 0, "$(PORT)", 39)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFB_SUM,PORT=FOFB_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=39")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFB_SUM,PORT=FOFB_SUM_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("FOFBPOS_Freq_Array", $(QSIZE), 0, "$(PORT)", 41)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}FOFBPOS_Freq,PORT=FOFBPOS_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=41")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}FOFBPOS_Freq,PORT=FOFBPOS_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### AMP PM Waveforms ##########################

NDStdArraysConfigure("PM_A_Array", $(QSIZE), 0, "$(PORT)", 48)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=48")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_A,PORT=PM_A_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_B_Array", $(QSIZE), 0, "$(PORT)", 49)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=49")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_B,PORT=PM_B_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_C_Array", $(QSIZE), 0, "$(PORT)", 50)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=50")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_C,PORT=PM_C_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_D_Array", $(QSIZE), 0, "$(PORT)", 51)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=51")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_D,PORT=PM_D_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PMAMP_Freq_Array", $(QSIZE), 0, "$(PORT)", 35)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PMAMP_Freq,PORT=PMAMP_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=35")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PMAMP_Freq,PORT=PMAMP_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

##################### POS PM Waveforms ##########################

NDStdArraysConfigure("PM_X_Array", $(QSIZE), 0, "$(PORT)", 54)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_X,PORT=PM_X_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=54")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_X,PORT=PM_X_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_Y_Array", $(QSIZE), 0, "$(PORT)", 55)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_Y,PORT=PM_Y_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=55")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_Y,PORT=PM_Y_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_Q_Array", $(QSIZE), 0, "$(PORT)", 56)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_Q,PORT=PM_Q_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=56")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_Q,PORT=PM_Q_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PM_SUM_Array", $(QSIZE), 0, "$(PORT)", 57)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PM_SUM,PORT=PM_SUM_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=57")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PM_SUM,PORT=PM_SUM_Array,ADDR=0,TIMEOUT=1,TYPE=Int32,FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS)")

NDStdArraysConfigure("PMPOS_Freq_Array", $(QSIZE), 0, "$(PORT)", 41)
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDPluginBase.template","P=${P},R=${R}PMPOS_Freq,PORT=PMPOS_Freq_Array,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),NDARRAY_ADDR=41")
dbLoadRecords("$(AREA_DETECTOR_COMPAT)/ADApp/Db/NDStdArrays.template", "P=${P},R=${R}PMPOS_Freq,PORT=PMPOS_Freq_Array,ADDR=0,TIMEOUT=1,TYPE=Float64,FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS)")
