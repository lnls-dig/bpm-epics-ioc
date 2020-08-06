#!/usr/bin/env python3

import argparse
from time import sleep
from bpm.bpm import BPM, BPMEnums

parser = argparse.ArgumentParser(description='BPM Acquistion utility')
parser.add_argument('prefix', type=str, help='EPICS PV prefix')
parser.add_argument('nr_samples', type=int, help='Number of acquisition samples')
parser.add_argument('acq_channel', type=str, help='Acquisition channel')
parser.add_argument('--acq_trigger_type', type=str, help='Acquisition channel', default='Now')
parser.add_argument('--nr_post_samples', type=int, help='Number of acquisition post-trigger acquisition samples', default=0)
parser.add_argument('--nr_shots', type=int, help='Number of acquisition shots', default=1)
parser.add_argument('--repetitive', type=str, help='Repetitive acquisition', default='Normal')

args = parser.parse_args()

# Setup acquistiion parameters
bpm = BPM(args.prefix, wait_for_connection=True)

bpm.nr_samples_pre = args.nr_samples
bpm.nr_samples_post = args.nr_post_samples
bpm.nr_shots = args.nr_shots
bpm.acq_repeat = BPMEnums.ACQREPEAT[args.repetitive]
bpm.acq_channel = BPMEnums.ACQCHAN[args.acq_channel]

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