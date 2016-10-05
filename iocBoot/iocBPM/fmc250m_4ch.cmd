##################### FMC250M_4CH Channels ##########################

dbLoadRecords("${TOP}/BPMApp/Db/BPMFmc250m_4ch.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADC_CHAN=0, ADDR=0,  BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFmc250m_4ch.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADC_CHAN=1, ADDR=1,  BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFmc250m_4ch.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADC_CHAN=2, ADDR=2,  BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/BPMFmc250m_4ch.template", "P=${EPICS_HOSTNAME}, PORT=$(BPM_NAME), ADC_CHAN=3, ADDR=3,  BPM_NUMBER=$(BPM_NUMBER), TIMEOUT=1")
