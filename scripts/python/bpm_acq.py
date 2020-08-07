#!/usr/bin/env python3

import argparse
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

print(bpm.array_a)
print(bpm.array_b)
print(bpm.array_c)
print(bpm.array_d)

if BPMEnums.ACQCHAN[args.acq_channel] not in {
    BPMEnums.ACQCHAN['ADC'], BPMEnums.ACQCHAN['ADCSwap']
}:
    print(bpm.pos_x)
    print(bpm.pos_y)
    print(bpm.pos_sum)
