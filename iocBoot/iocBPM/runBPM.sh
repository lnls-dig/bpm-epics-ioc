#!/bin/sh

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

BOARD_IDX=$(/usr/local/share/halcs/scripts/generate-board-halcs-idx.sh ${BPM_NUMBER} | awk '{print $2}')
HALCS_IDX=$(/usr/local/share/halcs/scripts/generate-board-halcs-idx.sh ${BPM_NUMBER} | awk '{print $3}')
FPGA_SYNTHESIS_NAME=$(sdb-read-lnls -l -e 0x0 /dev/fpga-${BOARD_IDX} 2>/dev/null | grep "synthesis-name:" | head -n 1 | awk '{print $2}')
FPGA_FMC0_NAME=$(sdb-read-lnls -l -e 0x0 /dev/fpga-${BOARD_IDX} 2>/dev/null | grep "LNLS_FMC" | head -n 1 | awk '{print $4}')
FPGA_FMC1_NAME=$(sdb-read-lnls -l -e 0x0 /dev/fpga-${BOARD_IDX} 2>/dev/null | grep "LNLS_FMC" | tail -n 1 | awk '{print $4}')
FPGA_FMC_NAME_IND=FPGA_FMC${HALCS_IDX}_NAME
FPGA_FMC_NAME=${!FPGA_FMC_NAME_IND}
ST_CMD_FILE=

case ${FPGA_SYNTHESIS_NAME} in
    bpm-gw*)
        case ${FPGA_FMC_NAME} in
            LNLS_FMC250M*)
                ST_CMD_FILE=stBPM250M.cmd
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

    pbpm-gw*)
        ST_CMD_FILE=stPBPMPICO.cmd
        ;;
    *)
        echo "Invalid Gateware: "${FPGA_SYNTHESIS_NAME} >&2
        exit 1
        ;;
esac


BPM_ENDPOINT=${BPM_ENDPOINT} BPM_NUMBER=${BPM_NUMBER} ../../bin/${EPICS_HOST_ARCH}/BPM ${ST_CMD_FILE}
