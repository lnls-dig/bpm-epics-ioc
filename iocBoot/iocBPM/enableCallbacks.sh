#!/bin/bash

channel_names="A B C D"

for i in $channel_names; do
    caput ${EPICS_HOSTNAME}:BPM17:ADC_${i}:EnableCallbacks 1
    caput ${EPICS_HOSTNAME}:BPM17:ADC_${i}_STATS:EnableCallbacks 1
done
