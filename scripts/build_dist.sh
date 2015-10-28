#!/usr/bin/env bash

USER=$(whoami)
IP="localhost"
PREFIX=$(pwd)
DIR="bpm-epics-ioc-deploy"
DIST_NAME="bpm-epics-ioc"

./deploy.sh ${USER} ${IP} ${PREFIX}/${DIR}
makeself --bzip2  --notemp ${PREFIX}/${DIR} ${DIST_NAME}.bz2.run "LNLS BPM EPICS IOC Package" \
    make
