#!/bin/bash

BPM_PREFIX=$1

caput ${BPM_PREFIX}:MonitEnable-Sel No
caput ${BPM_PREFIX}:ADCAD9510MuxStatus-RB.SCAN Passive
caput ${BPM_PREFIX}:ADCAD9510PllStatus-Mon.SCAN Passive
caput ${BPM_PREFIX}:SwTagDesyncCnt-Mon.SCAN Passive
caput ${BPM_PREFIX}:TbtTagDesyncCnt-Mon.SCAN Passive
caput ${BPM_PREFIX}:Monit1TagDesyncCnt-Mon.SCAN Passive
caput ${BPM_PREFIX}:MonitTagDesyncCnt-Mon.SCAN Passive

caput ${BPM_PREFIX}:IntlkBigger-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkBiggerLtc-Mon.SCAN Passive

caput ${BPM_PREFIX}:IntlkTransBigger-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerLtc-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerX-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerLtcX-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerY-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerLtcY-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkTransBiggerAny-Mon.SCAN Passive

caput ${BPM_PREFIX}:IntlkAngBigger-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerLtc-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerX-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerLtcX-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerY-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerLtcY-Mon.SCAN Passive
caput ${BPM_PREFIX}:IntlkAngBiggerAny-Mon.SCAN Passive

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
