##################### FMC250M_4CH Channels ##########################

dbLoadRecords("${TOP}/db/BPMFmc250m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADC_CHAN=0, ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMFmc250m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADC_CHAN=1, ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMFmc250m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADC_CHAN=2, ADDR=2, TIMEOUT=1")
dbLoadRecords("${TOP}/db/BPMFmc250m_4ch.template", "P=${P}, R=${R}, PORT=$(PORT), ADC_CHAN=3, ADDR=3, TIMEOUT=1")
