#!/bin/sh

BPM_ENDPOINT=$1
BOARD_NUMBER=$2
HALCS_NUMBER=$3
TRIES=$4

if [ -z "$BPM_ENDPOINT" ]; then
    echo "\"BPM_ENDPOINT\" variable unset."
    exit 1
fi

if [ -z "$BOARD_NUMBER" ]; then
    echo "\"BOARD_NUMBER\" variable unset"
    exit 1
fi

if [ "$BOARD_NUMBER" -lt 1 ] || [ "$BOARD_NUMBER" -gt 12 ]; then
    echo "Unsupported BOARD number"
    exit 1
fi

if [ -z "$HALCS_NUMBER" ]; then
    echo "\"HALCS_NUMBER\" variable unset"
    exit 1
fi

if [ "$HALCS_NUMBER" -lt 0 ] || [ "$HALCS_NUMBER" -gt 1 ]; then
    echo "Unsupported HALCS number"
    exit 1
fi

if [ -z "$TRIES" ]; then
    echo "\"TRIES\" variable unset."
    exit 1
fi

TRIES_SUCCESSFULLY=0
for i in `seq 1 ${TRIES}`;
do
    ../../bin/${EPICS_HOST_ARCH}/BPMReadParam -b ipc:///tmp/malamute -board ${BOARD_NUMBER} -halcs ${HALCS_NUMBER}
    if [ $? -eq 0 ]; then
        ((TRIES_SUCCESSFULLY++))
    fi
done

# FIXME please...
if [ "$TRIES_SUCCESSFULLY" -ge "2" ]; then
    exit 0
fi

exit 1
