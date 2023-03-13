#!/usr/bin/sh

source /etc/sysconfig/bpm-epics-ioc-slot-mapping

BOARDS="$1"
CRATE_NUM=$2

OLDIFS=$IFS; IFS=',';
# Separate "tuple" arguments with positional notation
set -- ${BOARDS};
for board in ${BOARDS[*]}; do
  IFS=$OLDIFS;

  # Each board has 2 BPMs
  for i in 0 1; do
    bpm=$((2*${board} - 1 + $i))

    pv_area_prefix=CRATE_${CRATE_NUM}_BPM_${bpm}_PV_AREA_PREFIX
    pv_device_prefix=CRATE_${CRATE_NUM}_BPM_${bpm}_PV_DEVICE_PREFIX

    caput ${!pv_area_prefix}${!pv_device_prefix}SwTagEn-Sel disabled      # FOFB sync
    caput ${!pv_area_prefix}${!pv_device_prefix}TbtTagEn-Sel disabled     # TbT sync
    caput ${!pv_area_prefix}${!pv_device_prefix}MonitTagEn-Sel disabled   # Monit sync
    caput ${!pv_area_prefix}${!pv_device_prefix}Monit1TagEn-Sel disabled  # Monit1 sync

    caput ${!pv_area_prefix}${!pv_device_prefix}SwTagEn-Sel enabled       # FOFB sync
    caput ${!pv_area_prefix}${!pv_device_prefix}TbtTagEn-Sel enabled      # TbT sync
    caput ${!pv_area_prefix}${!pv_device_prefix}MonitTagEn-Sel enabled    # Monit sync
    caput ${!pv_area_prefix}${!pv_device_prefix}Monit1TagEn-Sel enabled   # Monit1 sync

    sleep 1

    caput ${!pv_area_prefix}${!pv_device_prefix}Monit1TagEn-Sel disabled  # Monit1 sync
    caput ${!pv_area_prefix}${!pv_device_prefix}MonitTagEn-Sel disabled   # Monit sync
    caput ${!pv_area_prefix}${!pv_device_prefix}TbtTagEn-Sel disabled     # TbT sync
    caput ${!pv_area_prefix}${!pv_device_prefix}SwTagEn-Sel disabled      # FOFB sync

  done;

done;
