##################### GEN AMP FFT ##########################

dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_A, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_B, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_C, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_D, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")

##################### GEN POS FFT ##########################

dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_X, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_Y, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_Q, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=GEN_SUM, ACQ_NAME=ACQ, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")

# These FFTs are not exactly useful so they are commented here
# to save memory.
###################### AMP PM FFT ##########################
#
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_A, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_B, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_C, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_D, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#
###################### POS PM FFT ##########################
#
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_X, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_Y, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_Q, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=PM_SUM, ACQ_NAME=ACQ_PM, FTVL=LONG,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#
###################### AMP Single Pass FFT ##########################
#
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=SP_A, ACQ_NAME=ACQ, FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=SP_B, ACQ_NAME=ACQ, FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=SP_C, ACQ_NAME=ACQ, FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
#dbLoadRecords("${TOP}/BPMApp/Db/BPMFFT.template", "P=${P}, R=${R}, PORT=$(BPM_NAME), ADDR=0, ARRAY_NAME=SP_D, ACQ_NAME=ACQ, FTVL=DOUBLE,NELEMENTS=$(WAVEFORM_MAX_POINTS), TIMEOUT=1")
