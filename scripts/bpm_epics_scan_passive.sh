#!/bin/bash

BPM_PREFIX=$1

caput ${BPM_PREFIX}:MonitEnable-Sel No
caput ${BPM_PREFIX}:ADCAD9510MuxStatus-RB.SCAN Passive
caput ${BPM_PREFIX}:ADCAD9510PllStatus-Mon.SCAN Passive
caput ${BPM_PREFIX}:TbtTagDesyncCnt-Mon.SCAN Passive

for i in $(seq 0 23); do 
	caput ${BPM_PREFIX}:TRIGGER${i}TrnCnt-Mon.SCAN Passive 
	caput ${BPM_PREFIX}:TRIGGER${i}RcvCnt-Mon.SCAN Passive 

	caput ${BPM_PREFIX}:TRIGGER_PM${i}TrnCnt-Mon.SCAN Passive 
	caput ${BPM_PREFIX}:TRIGGER_PM${i}RcvCnt-Mon.SCAN Passive 
done

for i in $(seq 0 3); do 
	caput ${BPM_PREFIX}:ADC${i}Temp-Mon.SCAN Passive 
	caput ${BPM_PREFIX}:ADC${i}CalStatus-Mon.SCAN Passive 
done
