#!/usr/bin/env python3

import argparse
import numpy as np
from time import sleep
from bpm.bpm import BPM, BPMEnums

parser = argparse.ArgumentParser(description='BPM Acquistion utility')
parser.add_argument('prefix', type=str, help='EPICS PV prefix (e.g., SI-09SAFE:DI-PBPM-1:)')
parser.add_argument('nr_samples', type=int, help='Number of acquisition samples')
parser.add_argument('acq_channel', type=str, help='Acquisition channel',
                    choices=BPMEnums.ACQCHAN.keys())
parser.add_argument('--acq_trigger_type', type=str, help='Acquisition trigger type',
                    default='Now', choices=BPMEnums.ACQTRIGTYP.keys())
parser.add_argument('--nr_post_samples', type=int, help='Number of acquisition post-trigger samples',
                    default=0)
parser.add_argument('--nr_shots', type=int, help='Number of acquisition shots',
                    default=1)
parser.add_argument('--repetitive', type=str, help='Repetitive acquisition',
                    default='Normal', choices=BPMEnums.ACQREPEAT.keys())
parser.add_argument('--fmcpico_range', type=str, help='FMC PICO range selection (only available for XBPMs)',
                    nargs='?', const='1 mA', default=None, choices=BPMEnums.FMCPICORANGE.keys())
parser.add_argument('--fmcpico_offset', type=int, help='FMC PICO digital electronics offset (only available for XBPMs)',
                    default=1000)
parser.add_argument('--fmcpico_conv', action='store_true', help='FMC PICO conversion to engineering units',
                    default=False)

args = parser.parse_args()

# Setup acquistiion parameters
bpm = BPM(args.prefix, wait_for_connection=True)

bpm.nr_samples_pre = args.nr_samples
bpm.nr_samples_post = args.nr_post_samples
bpm.nr_shots = args.nr_shots
bpm.acq_repeat = BPMEnums.ACQREPEAT[args.repetitive]
bpm.acq_channel = BPMEnums.ACQCHAN[args.acq_channel]

# Set range if present on command-line
if args.fmcpico_range is not None:
    bpm.fmc_pico_range_ch0 = BPMEnums.FMCPICORANGE[args.fmcpico_range]
    bpm.fmc_pico_range_ch1 = BPMEnums.FMCPICORANGE[args.fmcpico_range]
    bpm.fmc_pico_range_ch2 = BPMEnums.FMCPICORANGE[args.fmcpico_range]
    bpm.fmc_pico_range_ch3 = BPMEnums.FMCPICORANGE[args.fmcpico_range]

# Acquistion type
bpm.acq_trigger = BPMEnums.ACQTRIGTYP[args.acq_trigger_type]
# Acquistion type
bpm.acq_event = BPMEnums.ACQEVENTS['Start']

# Wait for acquisition to complete
while not bpm.is_acq_completed:
    sleep(0.5)

# Convert to engineering units if required for FMC PICO
vals = {
    'A': {
        'data'  : np.array(bpm.array_a),
    },
    'C': {
        'data'  : np.array(bpm.array_b),
    },
    'B': {
        'data'  : np.array(bpm.array_c),
    },
    'D': {
        'data'  : np.array(bpm.array_d),
    }
}

if args.fmcpico_conv:
    # Convert range (0, 1) to uA (100, 1000)
    vals['A']['range'] = 100 if bpm.fmc_pico_range_ch0 == 0 else 1000
    vals['B']['range'] = 100 if bpm.fmc_pico_range_ch1 == 0 else 1000
    vals['C']['range'] = 100 if bpm.fmc_pico_range_ch2 == 0 else 1000
    vals['D']['range'] = 100 if bpm.fmc_pico_range_ch3 == 0 else 1000

    # Offset
    offset = args.fmcpico_offset

    # Conversion factor
    conv_factors = {
        BPMEnums.ACQCHAN['ADC']     : 524288,
        BPMEnums.ACQCHAN['ADCSwap'] : 524288,
        BPMEnums.ACQCHAN['TbT']     : 1048576,
        BPMEnums.ACQCHAN['FOFB']    : 5242880,
        BPMEnums.ACQCHAN['Monit1']  : 4096000
    }
    conv_factor = conv_factors.get(BPMEnums.ACQCHAN[bpm.acq_channel], 524288)

    for _, v in vals.items():
        range = v['range']
        data = v['data']

        data = (range/conv_factor) * (data - offset)

# Print values
print(vals['A']['data'])
print(vals['B']['data'])
print(vals['C']['data'])
print(vals['D']['data'])

if BPMEnums.ACQCHAN[args.acq_channel] not in {
    BPMEnums.ACQCHAN['ADC'], BPMEnums.ACQCHAN['ADCSwap']
}:
    print(bpm.pos_x)
    print(bpm.pos_y)
    print(bpm.pos_sum)
