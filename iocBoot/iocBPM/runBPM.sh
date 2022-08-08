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

SDB_FILENAME_PATH=/var/log/halcs
SDB_FILENAME_PREFIX=halcsd
SDB_FILENAME_TYPE=be
SDB_FILENAME_SUFFIX=info
SDB_FILENAME_HALCS_IDX=0

INSTANCE_IDX=$(echo ${BPM_NUMBER} | sed 's|.*-||g')
BOARD_IDX=$(expr ${INSTANCE_IDX} / 2 + ${INSTANCE_IDX} % 2)
HALCS_IDX=$(expr 1 - ${INSTANCE_IDX} % 2)
# Compose filename as example: halcsd8_be0_info.log
SDB_FILENAME=${SDB_FILENAME_PATH}/${SDB_FILENAME_PREFIX}${BOARD_IDX}_${SDB_FILENAME_TYPE}${SDB_FILENAME_HALCS_IDX}_${SDB_FILENAME_SUFFIX}.log

if [ -f "${SDB_FILENAME}" ]; then
    echo "${SDB_FILENAME} found. Parsing it to find which FMC board we have."
else
    echo "${SDB_FILENAME} not found. Exiting."
    exit 1
fi

FPGA_SYNTHESIS_NAME=$(cat ${SDB_FILENAME} | grep "synthesis-name:" | head -n 1 | awk '{print $2}')
FPGA_FMC0_NAME=$(cat ${SDB_FILENAME} | grep "LNLS_FMC" | head -n 1 | awk '{print $4}')
FPGA_FMC1_NAME=$(cat ${SDB_FILENAME} | grep "LNLS_FMC" | tail -n 1 | awk '{print $4}')
FPGA_FMC_NAME_IND=FPGA_FMC${HALCS_IDX}_NAME
FPGA_FMC_NAME=${!FPGA_FMC_NAME_IND}
ST_CMD_FILE=

echo "Synthesis name: "${FPGA_SYNTHESIS_NAME}
echo "FMC 0 name: "${FPGA_FMC0_NAME}
echo "FMC 1 name: "${FPGA_FMC1_NAME}

case ${FPGA_SYNTHESIS_NAME} in
    bpm-gw-bo*)
        case ${FPGA_FMC_NAME} in
            LNLS_FMC250M*)
                ST_CMD_FILE=stBPM250M_bo.cmd
                ;;
            LNLS_FMC130M*)
                ST_CMD_FILE=stBPM130M.cmd
                ;;
            *)
                echo "Unsupported Gateware Module: "${FPGA_FMC_NAME} >&2
                exit 1
                ;;
        esac
        ;;

    bpm-gw-sr*)
        case ${FPGA_FMC_NAME} in
            LNLS_FMC250M*)
                ST_CMD_FILE=stBPM250M_sr.cmd
                ;;
            LNLS_FMC130M*)
                ST_CMD_FILE=stBPM130M.cmd
                ;;
            *)
                echo "Unsupported Gateware Module: "${FPGA_FMC_NAME} >&2
                exit 1
                ;;
        esac
        ;;

    tim-receiver*)
        ST_CMD_FILE=stTIM.cmd
        ;;

    pbpm-gw*)
        ST_CMD_FILE=stPBPMPICO.cmd
        ;;
    *)
        echo "Invalid Gateware: "${FPGA_SYNTHESIS_NAME} >&2
        exit 1
        ;;
esac

echo "Using st.cmd file: "${ST_CMD_FILE}

BPM_ENDPOINT=${BPM_ENDPOINT} BPM_NUMBER=${BPM_NUMBER} ../../bin/${EPICS_HOST_ARCH}/BPM ${ST_CMD_FILE}
