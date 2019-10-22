#!/usr/bin/env bash

: ${EPICS_HOST_ARCH:?"Environment variable needs to be set"}

VALID_BPM_NUMBER_STR="Valid values are between 0 and 23."

# Select endpoint.
BPM_ENDPOINT=$1

if [ -z "$BPM_ENDPOINT" ]; then
    echo "\"BPM_ENDPOINT\" variable unset."
    exit 1
fi

# Select board in which we will work.
BPM_NUMBER=$2

if [ -z "$BPM_NUMBER" ]; then
    echo "\"BPM_NUMBER\" variable unset. "$VALID_BPM_NUMBERS_STR
    exit 1
fi

if [ "$BPM_NUMBER" -lt 1 ] || [ "$BPM_NUMBER" -gt 24 ]; then
    echo "Unsupported BPM number. "$VALID_BPM_NUMBERS_STR
    exit 1
fi

export BPM_CURRENT_PV_AREA_PREFIX=${EPICS_PV_CRATE_PREFIX}_BPM_${BPM_NUMBER}_PV_AREA_PREFIX
export BPM_CURRENT_PV_DEVICE_PREFIX=${EPICS_PV_CRATE_PREFIX}_BPM_${BPM_NUMBER}_PV_DEVICE_PREFIX
export EPICS_PV_AREA_PREFIX=${!BPM_CURRENT_PV_AREA_PREFIX}
export EPICS_PV_DEVICE_PREFIX=${!BPM_CURRENT_PV_DEVICE_PREFIX}

BPM_ENDPOINT=${BPM_ENDPOINT} BPM_NUMBER=${BPM_NUMBER} ../../bin/${EPICS_HOST_ARCH}/BPM stBPM250M.cmd
