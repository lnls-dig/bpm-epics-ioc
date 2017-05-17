#!/bin/bash

BPM_PREFIX=$1

caput ${BPM_PREFIX}:DI-BPM:MONITUpdtAll.SCAN Passive
caput ${BPM_PREFIX}:DI-BPM:ADCAD9510MuxStatus-RB.SCAN Passive
caput ${BPM_PREFIX}:DI-BPM:ADCAD9510PllStatus-RB.SCAN Passive

for i in $(seq 0 23); do 
	caput ${BPM_PREFIX}:DI-BPM:TRIGGER${i}TrnCnt-RB.SCAN Passive 
	caput ${BPM_PREFIX}:DI-BPM:TRIGGER${i}RcvCnt-RB.SCAN Passive 

	caput ${BPM_PREFIX}:DI-BPM:TRIGGER_PM${i}TrnCnt-RB.SCAN Passive 
	caput ${BPM_PREFIX}:DI-BPM:TRIGGER_PM${i}RcvCnt-RB.SCAN Passive 
done

for i in $(seq 0 3); do 
	caput ${BPM_PREFIX}:DI-BPM:ADC${i}Temp-RB.SCAN Passive 
	caput ${BPM_PREFIX}:DI-BPM:ADC${i}CalStatus-RB.SCAN Passive 
done
