dbLoadRecords("${TOP}/BPMApp/Db/FOFBCtrl.template", "P=${P}, R=${R}, FOFB_CC=P2P:, PORT=$(PORT), ADDR=0, TIMEOUT=1")

dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH0,  PORT=$(PORT), ADDR=0, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH1,  PORT=$(PORT), ADDR=1, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH2,  PORT=$(PORT), ADDR=2, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH3,  PORT=$(PORT), ADDR=3, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH4,  PORT=$(PORT), ADDR=4, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH5,  PORT=$(PORT), ADDR=5, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH6,  PORT=$(PORT), ADDR=6, TIMEOUT=1")
dbLoadRecords("${TOP}/BPMApp/Db/Ctrl_channel.template", "P=${P}, R=${R}, FOFB_CC=P2P:, FOFB_CHAN=CH7,  PORT=$(PORT), ADDR=7, TIMEOUT=1")
