#!/usr/bin/env python3

from time import sleep
from bpm.bpm_intlk import BPMIntlk
from bpm.timing_evr import TimingEVR
from bpm.timing_evg import TimingEVG
from textwrap import indent

# test parameters

# bpm
reset_intlk_lmt_trans_max_x = 0
reset_intlk_trans_en = 1
reset_intlk_en = 0
reset_trigger4_trn_src = 1       # internal
reset_trigger4_trn_out_sel = 2   # interlock generator
reset_trigger4_dir = 0           # transmitter
reset_trigger4_trn_len = 20      # transmitter length

intlk_lmt_trans_max_x = 0
intlk_trans_en = 1
intlk_en = 1
trigger4_trn_src = 1             # internal
trigger4_trn_out_sel = 2         # interlock generator
trigger4_dir = 0                 # transmitter
trigger4_trn_len = 20            # transmitter length

# timing
reset_amc4_state = 0
reset_amc4_evt = 117              
reset_amc4_nr_pulses = 1
reset_amc4_src = 0               # trigger          
reset_amc4_dir = 1               # receiver

amc4_state = 1
amc4_evt = 117              
amc4_nr_pulses = 1
amc4_src = 0                     # trigger          
amc4_dir = 1                     # receiver

# timing evg
reset_intlk_ctrl_enbl = 0
reset_intlk_ctrl_rst = 1
reset_intlk_evt_in0 = 117
reset_intlk_evt_in1 = 118 
reset_intlk_evt_in2 = 119

intlk_ctrl_enbl = 1
intlk_ctrl_rst = 1
intlk_evt_in0 = 117
intlk_evt_in1 = 118 
intlk_evt_in2 = 119

ss_names_by_sector = [
    ("SA", 1), 
    ("SB", 2),
    ("SP", 3),
    ("SB", 4),
    ("SA", 5),
    ("SB", 6),
    ("SP", 7),
    ("SB", 8),
    ("SA", 9),
    ("SB", 10),
    ("SP", 11),
    ("SB", 12),
    ("SA", 13),
    ("SB", 14),
    ("SP", 15),
    ("SB", 16),
    ("SA", 17),
    ("SB", 18),
    ("SP", 19),
    ("SB", 20),
]

# Prefix generation

bpm_si_prefixes = []
for ss in ss_names_by_sector:

    bpm_prefix_item = [
# Straight sections BPMs are not installed in most sections
#        "SI-{:02d}{}:DI-BPM-1:".format(idx, ss),
#        "SI-{:02d}{}:DI-BPM-2:".format(idx, ss),
        "SI-{:02d}M1:DI-BPM:".format(ss[1]),
        "SI-{:02d}M2:DI-BPM:".format(ss[1]),
        "SI-{:02d}C1:DI-BPM-1:".format(ss[1]),
        "SI-{:02d}C1:DI-BPM-2:".format(ss[1]),
        "SI-{:02d}C2:DI-BPM:".format(ss[1]),
        "SI-{:02d}C3:DI-BPM-1:".format(ss[1]),
        "SI-{:02d}C3:DI-BPM-2:".format(ss[1]),
        "SI-{:02d}C4:DI-BPM:".format(ss[1]),
    ]
    bpm_si_prefixes.append(bpm_prefix_item)

timing_prefixes = []
for ss in ss_names_by_sector:
    timing_prefix_item = "IA-{:02d}RaBPM:TI-AMCFPGAEVR:".format(ss[1])
    timing_prefixes.append(timing_prefix_item)

timing_evg_prefix = 'AS-RaMO:TI-EVG:'

# PVs generation

bpms = []
for i, bpm_sector_prefixes in enumerate(bpm_si_prefixes):
    bpms.append([])
    for j, bpm_prefix in enumerate(bpm_sector_prefixes):
        bpms[i].append(BPMIntlk(bpm_prefix, wait_for_connection=True))

timings = []
for i, timing_prefix in enumerate(timing_prefixes):
    timings.append(TimingEVR(timing_prefix, wait_for_connection=True))

timing_evg = TimingEVG(timing_evg_prefix, wait_for_connection=True)

print("#####################")
print("BPM status")
print("#####################")

for i, bpm_sector in enumerate(bpms):
    print("BPM sector {}:".format(i+1))
    for j, bpm in enumerate(bpm_sector):
        print("    BPM {:20}".format(bpms[i][j].prefix))
        print("{}".format(indent(str(bpms[i][j]), '        ')), end='')

print("#####################")
print("Timing status")
print("#####################")

for i, _ in enumerate(timings):
    print("Timing sector {}:".format(i+1))
    print("    Timing {:20}".format(timings[i].prefix))
    print("{}".format(indent(str(timings[i]), '        ')), end='')

print("#####################")
print("Timing EVG status")
print("#####################")

print("Timing EVG")
print("    Timing EVG {:20}".format(timing_evg.prefix))
print("{}".format(indent(str(timing_evg), '        ')), end='')

print("Resetting BPM test parameters... ", end='')
for i, bpm_sector in enumerate(bpms):
    for j, bpm in enumerate(bpm_sector):
        bpms[i][j].intlk_lmt_trans_max_x = reset_intlk_lmt_trans_max_x
        bpms[i][j].intlk_trans_en = reset_intlk_trans_en
        bpms[i][j].intlk_en = reset_intlk_en
        bpms[i][j].trigger4_trn_src = reset_trigger4_trn_src
        bpms[i][j].trigger4_trn_out_sel = reset_trigger4_trn_out_sel
        bpms[i][j].trigger4_dir = reset_trigger4_dir
        bpms[i][j].trigger4_trn_len = reset_trigger4_trn_len
print("Done")

print("Resetting Timing test parameters... ", end='')
for i, _ in enumerate(timings):
    timings[i].amc4_state = reset_amc4_state
    timings[i].amc4_evt = reset_amc4_evt
    timings[i].amc4_nr_pulses = reset_amc4_nr_pulses
    timings[i].amc4_src = reset_amc4_src
    timings[i].amc4_dir = reset_amc4_dir
print("Done")

print("Resetting Timing EVG test parameters... ", end='')
timing_evg.intlk_ctrl_enbl = reset_intlk_ctrl_enbl
timing_evg.intlk_ctrl_rst = reset_intlk_ctrl_rst
timing_evg.intlk_evt_in0 = reset_intlk_evt_in0 
timing_evg.intlk_evt_in1 = reset_intlk_evt_in1
timing_evg.intlk_evt_in2 = reset_intlk_evt_in2 
print("Done")

print("Waiting some time to settle... ", end='')
sleep(1)
print("Done")

#print("Setting BPM test parameters... ", end='')
#for i, bpm_sector in enumerate(bpms):
#    for j, bpm in enumerate(bpm_sector):
#        bpms[i][j].intlk_lmt_trans_max_x = intlk_lmt_trans_max_x
#        bpms[i][j].intlk_trans_en = intlk_trans_en
#        bpms[i][j].intlk_en = intlk_en
#        bpms[i][j].trigger4_trn_src = trigger4_trn_src
#        bpms[i][j].trigger4_trn_out_sel = trigger4_trn_out_sel
#        bpms[i][j].trigger4_dir = trigger4_dir
#        bpms[i][j].trigger4_trn_len = trigger4_trn_len
#print("Done")

print("Setting Timing test parameters... ", end='')
for i, _ in enumerate(timings):
    timings[i].amc4_state = amc4_state
    timings[i].amc4_evt = amc4_evt
    timings[i].amc4_nr_pulses = amc4_nr_pulses
    timings[i].amc4_src = amc4_src
    timings[i].amc4_dir = amc4_dir
print("Done")

print("Setting Timing EVG test parameters... ", end='')
timing_evg.intlk_ctrl_enbl = intlk_ctrl_enbl
timing_evg.intlk_ctrl_rst = intlk_ctrl_rst
timing_evg.intlk_evt_in0 = intlk_evt_in0 
timing_evg.intlk_evt_in1 = intlk_evt_in1
timing_evg.intlk_evt_in2 = intlk_evt_in2 
print("Done")

print("Waiting some time to settle... ", end='')
sleep(1)
print("Done")

errs = 0
print("Testing BPM interlock generation... ")
for i, bpm_sector in enumerate(bpms):
    for j, bpm in enumerate(bpm_sector):
        print("    {}".format(bpms[i][j].prefix))

        print("        Disabling BPM interlock generation...", end='')
        # reset BPM to send the trigger
        bpms[i][j].intlk_en = reset_intlk_en
        sleep(1)
        print(" Ok")

        print("        Resetting Timing EVG interlock status...", end='')
        timing_evg.intlk_ctrl_rst = reset_intlk_ctrl_rst
        sleep(1)
        print(" Ok")

        print("        Checking if EVG Intlk Status is clear...", end='')
        # check if EVG received the event
        if (timing_evg.intlk_evt_status == 0):
            print(" Ok")
        else:
            print(" Error")
            errs = errs + 1

        print("        Enabling BPM interlock generation...", end='')
        # enable BPM to send the trigger
        bpms[i][j].intlk_en = intlk_en
        # wait IOC to capture the interlock status
        sleep(1)
        print(" Ok")

        print("        Timing EVG Intlk Status: {}".format(timing_evg.intlk_evt_status))
        # check if EVG received the event
        if (timing_evg.intlk_evt_status == 1):
            print("        Passed")
        else:
            print("        Failed")
            errs = errs + 1

if (errs > 0):
    print("Test Failed")
else:
    print("Test Succeeded")
