#!/usr/bin/env python3

from time import sleep
from bpm.bpm_intlk import BPMIntlk
from bpm.timing_evr import TimingEVR
from bpm.timing_evg import TimingEVG
from bpm.timing_fout import TimingFOUT
from textwrap import indent
from enum import IntEnum
import datetime
import re
import sys

# from: https://stackoverflow.com/questions/107705/disable-output-buffering
class Unbuffered(object):
    def __init__(self, stream):
        self.stream = stream

    def write(self, data):
        self.stream.write(data)
        self.stream.flush()

    def writelines(self, datas):
        self.stream.writelines(datas)
        self.stream.flush()

    def __getattr__(self, attr):
        return getattr(self.stream, attr)

# unbuffer stdout for responsive printing
sys.stdout = Unbuffered(sys.stdout)

# test parameters

# bpm
class BPMResetParams(IntEnum):
    INTLK_LMT_TRANS_MAX_X = 0
    INTLK_LMT_TRANS_MIN_X = 0
    INTLK_TRANS_EN = 1
    INTLK_EN = 0
    TRIGGER4_TRN_SRC = 1       # internal
    TRIGGER4_TRN_OUT_SEL = 2   # interlock generator
    TRIGGER4_DIR = 0           # transmitter
    TRIGGER4_TRN_LEN = 20      # transmitter length

class BPMParams(IntEnum):
    INTLK_LMT_TRANS_MAX_X = 0
    INTLK_LMT_TRANS_MIN_X = 0
    INTLK_TRANS_EN = 1
    INTLK_EN = 1
    TRIGGER4_TRN_SRC = 1             # internal
    TRIGGER4_TRN_OUT_SEL = 2         # interlock generator
    TRIGGER4_DIR = 0                 # transmitter
    TRIGGER4_TRN_LEN = 20            # transmitter length

# timing
class TimingEVRResetParams(IntEnum):
    AMC4_STATE = 0
    AMC4_EVT = 117
    AMC4_NR_PULSES = 1
    AMC4_SRC = 0               # trigger
    AMC4_DIR = 1               # receiver

class TimingEVRParams(IntEnum):
    AMC4_STATE = 1
    AMC4_EVT = 117
    AMC4_NR_PULSES = 1
    AMC4_SRC = 0                     # trigger
    AMC4_DIR = 1                     # receiver

# fout timing
class TimingFOUTResetParams(IntEnum):
    RX_ENBL = 0

class TimingFOUTParams(IntEnum):
    RX_ENBL = 0

# timing evg
class TimingEVGResetParams(IntEnum):
    RX_ENBL = 0
    INTLK_CTRL_ENBL = 0
    INTLK_CTRL_RST = 1
    INTLK_EVT_IN0 = 117
    INTLK_EVT_IN1 = 118
    INTLK_EVT_IN2 = 119

class TimingEVGParams(IntEnum):
    INTLK_CTRL_ENBL = 1
    INTLK_CTRL_RST = 1
    INTLK_EVT_IN0 = 117
    INTLK_EVT_IN1 = 118
    INTLK_EVT_IN2 = 119

timing_evg_evt_status_mapping = {
    TimingEVGParams.INTLK_EVT_IN0: 1,
    TimingEVGParams.INTLK_EVT_IN1: 2,
    TimingEVGParams.INTLK_EVT_IN2: 4,
}

def bpm_cleanup_test_parameters(bpms):
    # reset BPM to send the trigger
    for bpm in bpms:
        bpm.intlk_en = BPMResetParams.INTLK_EN

def timing_fout_clenup_test_parameters(fout):
    fout.rx_enbl = TimingFOUTResetParams.RX_ENBL

def timing_evg_clenup_intlk_status(evg):
    evg.intlk_ctrl_rst = TimingEVGResetParams.INTLK_CTRL_RST

def timing_evg_clenup_rx_enbl(evg):
    evg.rx_enbl = TimingEVGResetParams.RX_ENBL

def timing_evg_clenup_test_parameters(evg):
    timing_evg_clenup_rx_enbl(evg)
    timing_evg_clenup_intlk_status(evg)

def timing_fout_test_comma_position(fout, channel, tryouts=2, seconds_after_tryout=1):
    fout_rx_frame_pos = fout.rx_frame_pos[channel]
    # print("            Frame pos baseline: {}".format(fout_rx_frame_pos))
    for frame_pos_iter in range(tryouts):
        sleep(seconds_after_tryout)
        fout_rx_frame_pos_iter = fout.rx_frame_pos[channel]
        # print("            Frame pos iteration #{}: {}".format(frame_pos_iter, fout_rx_frame_pos_iter))
        if fout_rx_frame_pos_iter != fout_rx_frame_pos:
            print(" Frame pos iteration #{}: {}, differs from baseline {}".format(
                frame_pos_iter, fout_rx_frame_pos_iter,fout_rx_frame_pos))
            sys.exit(1)

ss_names_by_sector = {
    "1":  {"ss_name": "SA"},
    "2":  {"ss_name": "SB"},
    "3":  {"ss_name": "SP"},
    "4":  {"ss_name": "SB"},
    "5":  {"ss_name": "SA"},
    "6":  {"ss_name": "SB"},
    "7":  {"ss_name": "SP"},
    "8":  {"ss_name": "SB"},
    "9":  {"ss_name": "SA"},
    "10": {"ss_name": "SB"},
    "11": {"ss_name": "SP"},
    "12": {"ss_name": "SB"},
    "13": {"ss_name": "SA"},
    "14": {"ss_name": "SB"},
    "15": {"ss_name": "SP"},
    "16": {"ss_name": "SB"},
    "17": {"ss_name": "SA"},
    "18": {"ss_name": "SB"},
    "19": {"ss_name": "SP"},
    "20": {"ss_name": "SB"},
}

timing_evr_fout_mapping = {
    # AFC Timing, Fanout, RX input port
    "1":  {"fanout": 3, "channel": 0},
    "2":  {"fanout": 3, "channel": 1},
    "3":  {"fanout": 3, "channel": 2},
    "4":  {"fanout": 3, "channel": 3},
    "5":  {"fanout": 3, "channel": 4},
    "6":  {"fanout": 3, "channel": 5},
    "7":  {"fanout": 3, "channel": 6},
    "8":  {"fanout": 4, "channel": 0},
    "9":  {"fanout": 4, "channel": 1},
    "10": {"fanout": 4, "channel": 2},
    "11": {"fanout": 4, "channel": 3},
    "12": {"fanout": 4, "channel": 4},
    "13": {"fanout": 4, "channel": 5},
    "14": {"fanout": 4, "channel": 6},
    "15": {"fanout": 5, "channel": 0},
    "16": {"fanout": 5, "channel": 1},
    "17": {"fanout": 5, "channel": 2},
    "18": {"fanout": 5, "channel": 3},
    "19": {"fanout": 5, "channel": 4},
    "20": {"fanout": 5, "channel": 5},
}

timing_fout_evg_mapping = {
    # Fanout Timing, EVG, RX input port
    "1": {"evg": 1, "channel": 0},
    "2": {"evg": 1, "channel": 1},
    "3": {"evg": 1, "channel": 2},
    "4": {"evg": 1, "channel": 3},
    "5": {"evg": 1, "channel": 4},
}

# Prefix generation

bpm_si_prefixes = []
for k, v in ss_names_by_sector.items():
    bpm_prefix_item = [
# Straight sections BPMs are not installed in most sections
#        "SI-{:02d}{}:DI-BPM-1:".format(int(k), v["ss_name"]),
#        "SI-{:02d}{}:DI-BPM-2:".format(int(k), v["ss_name"]),
        "SI-{:02d}M1:DI-BPM:".format(int(k)),
        "SI-{:02d}M2:DI-BPM:".format(int(k)),
        "SI-{:02d}C1:DI-BPM-1:".format(int(k)),
        "SI-{:02d}C1:DI-BPM-2:".format(int(k)),
        "SI-{:02d}C2:DI-BPM:".format(int(k)),
        "SI-{:02d}C3:DI-BPM-1:".format(int(k)),
        "SI-{:02d}C3:DI-BPM-2:".format(int(k)),
        "SI-{:02d}C4:DI-BPM:".format(int(k)),
    ]
    bpm_si_prefixes.append(bpm_prefix_item)

timing_prefixes = []
for k, v in ss_names_by_sector.items():
    timing_prefix_item = "IA-{:02d}RaBPM:TI-AMCFPGAEVR:".format(int(k))
    timing_prefixes.append(timing_prefix_item)

timing_fout_prefixes = []
timing_fout_prefixes.append("CA-RaTim:TI-Fout-3:")
timing_fout_prefixes.append("CA-RaTim:TI-Fout-4:")
timing_fout_prefixes.append("CA-RaTim:TI-Fout-5:")

timing_evg_prefix = 'AS-RaMO:TI-EVG:'

# PVs generation

bpms = []
for i, bpm_sector_prefixes in enumerate(bpm_si_prefixes):
    bpms.append([])
    for _, bpm_prefix in enumerate(bpm_sector_prefixes):
        bpms[i].append(BPMIntlk(bpm_prefix, wait_for_connection=True))

timings = []
for _, timing_prefix in enumerate(timing_prefixes):
    timings.append(TimingEVR(timing_prefix, wait_for_connection=True))

timings_fout = []
for _, timing_fout_prefix in enumerate(timing_fout_prefixes):
    timings_fout.append(TimingFOUT(timing_fout_prefix, wait_for_connection=True))

timing_evg = TimingEVG(timing_evg_prefix, wait_for_connection=True)

# Print date
now = datetime.datetime.now()
print("Test date and time: ")
print(now.strftime('%Y-%m-%d %H:%M:%S'))

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
print("Timing Fanout status")
print("#####################")

for i, _ in enumerate(timings_fout):
    print("Timing Fout Index {}:".format(i+1))
    print("    Timing Fout {:20}".format(timings_fout[i].prefix))
    print("{}".format(indent(str(timings_fout[i]), '        ')), end='')

print("#####################")
print("Timing EVG status")
print("#####################")

print("Timing EVG")
print("    Timing EVG {:20}".format(timing_evg.prefix))
print("{}".format(indent(str(timing_evg), '        ')), end='')

print("Resetting BPM test parameters... ", end='')
for i, bpm_sector in enumerate(bpms):
    for j, bpm in enumerate(bpm_sector):
        bpms[i][j].intlk_lmt_trans_max_x = BPMResetParams.INTLK_LMT_TRANS_MAX_X
        bpms[i][j].intlk_lmt_trans_min_x = BPMResetParams.INTLK_LMT_TRANS_MIN_X
        bpms[i][j].intlk_trans_en = BPMResetParams.INTLK_TRANS_EN
        bpms[i][j].intlk_en = BPMResetParams.INTLK_EN
        bpms[i][j].trigger4_trn_src = BPMResetParams.TRIGGER4_TRN_SRC
        bpms[i][j].trigger4_trn_out_sel = BPMResetParams.TRIGGER4_TRN_OUT_SEL
        bpms[i][j].trigger4_dir = BPMResetParams.TRIGGER4_DIR
        bpms[i][j].trigger4_trn_len = BPMResetParams.TRIGGER4_TRN_LEN
print("Done")

print("Resetting Timing test parameters... ", end='')
for i, _ in enumerate(timings):
    timings[i].amc4_state = TimingEVRResetParams.AMC4_STATE
    timings[i].amc4_evt = TimingEVRResetParams.AMC4_EVT
    timings[i].amc4_nr_pulses = TimingEVRResetParams.AMC4_NR_PULSES
    timings[i].amc4_src = TimingEVRResetParams.AMC4_SRC
    timings[i].amc4_dir = TimingEVRResetParams.AMC4_DIR
print("Done")

print("Resetting Timing Fout test parameters... ", end='')
for i, _ in enumerate(timings_fout):
    timings_fout[i].rx_enbl = TimingFOUTResetParams.RX_ENBL
print("Done")

print("Resetting Timing EVG test parameters... ", end='')
timing_evg.intlk_ctrl_enbl = TimingEVGResetParams.INTLK_CTRL_ENBL
timing_evg.intlk_ctrl_rst = TimingEVGResetParams.INTLK_CTRL_RST
timing_evg.intlk_evt_in0 = TimingEVGResetParams.INTLK_EVT_IN0
timing_evg.intlk_evt_in1 = TimingEVGResetParams.INTLK_EVT_IN1
timing_evg.intlk_evt_in2 = TimingEVGResetParams.INTLK_EVT_IN2
print("Done")

print("Waiting some time to allow parameters to settle... ", end='')
sleep(1)
print("Done")

print("Setting Timing test parameters... ", end='')
for i, _ in enumerate(timings):
    timings[i].amc4_state = TimingEVRParams.AMC4_STATE
    timings[i].amc4_evt = TimingEVRParams.AMC4_EVT
    timings[i].amc4_nr_pulses = TimingEVRParams.AMC4_NR_PULSES
    timings[i].amc4_src = TimingEVRParams.AMC4_SRC
    timings[i].amc4_dir = TimingEVRParams.AMC4_DIR
print("Done")

print("Setting Timing EVG test parameters... ", end='')
timing_evg.intlk_ctrl_enbl = TimingEVGParams.INTLK_CTRL_ENBL
timing_evg.intlk_ctrl_rst = TimingEVGParams.INTLK_CTRL_RST
timing_evg.intlk_evt_in0 = TimingEVGParams.INTLK_EVT_IN0
timing_evg.intlk_evt_in1 = TimingEVGParams.INTLK_EVT_IN1
timing_evg.intlk_evt_in2 = TimingEVGParams.INTLK_EVT_IN2
print("Done")

print("Waiting some time to allow parameters to settle... ", end='')
sleep(1)
print("Done")

errs_bpm = 0
errs_bpm_names = []
errs_bpm_evg = 0
errs_bpm_evg_names = []
print("Testing BPM interlock generation... ")
for i, bpm_sector in enumerate(bpms):

    sector = re.search("SI-([0-9][0-9]).+:.+", bpms[i][0].prefix).group(1).lstrip("0")
    print("        Sector: {}".format(sector))

    # search for correct Fanout module
    timing_fout_num = timing_evr_fout_mapping[sector]["fanout"]
    for fout in timings_fout:
        if f'TI-Fout-{timing_fout_num}:' in fout.prefix:
            timing_fout = fout
            break

    print("        Resetting test parameters for Fout ({})...".format(timing_fout.prefix), end='')
    timing_fout_clenup_test_parameters(timing_fout)
    print(" Ok")

    print("        Resetting test parameters for EVG ({})...".format(timing_evg.prefix), end='')
    timing_evg_clenup_test_parameters(timing_evg)
    print(" Ok")

    rx_evg_enable_param = (0x1 << timing_fout_evg_mapping[str(timing_fout_num)]["channel"])
    print("        Enabling Timing EVG RX ({}) Enable to {}...".format(timing_evg.prefix, rx_evg_enable_param), end='')
    timing_evg.rx_enbl = rx_evg_enable_param
    print(" Ok")

    rx_fout_enable_param = (0x1 << timing_evr_fout_mapping[sector]["channel"])
    print("        Enabling Timing Fout RX ({}) Enable to {}...".format(timing_fout.prefix, rx_fout_enable_param), end='')
    timing_fout.rx_enbl = rx_fout_enable_param
    print(" Ok")

    print("        Waiting some time to allow parameters to settle...", end='')
    sleep(2)
    print(" Done")

    # check if fanout channel for sector is chaning, This is a bug
    timing_fout_channel = timing_evr_fout_mapping[sector]["channel"]
    print("        Checking if Fout ({}) RX enable channel {} is not changing \"comma position\"...".format(
        timing_fout.prefix, timing_fout_channel), end='')
    timing_fout_test_comma_position(timing_fout, timing_fout_channel)
    print(" Ok")

    for j, bpm in enumerate(bpm_sector):
        print("    {}".format(bpms[i][j].prefix))

        print("        Resetting test parameters for all sector BPMs...", end='')
        # reset BPM to send the trigger
        bpm_cleanup_test_parameters(bpm_sector)
        print(" Ok")

        print("        Resetting interlock status for EVG ({})...".format(timing_evg.prefix), end='')
        timing_evg_clenup_intlk_status(timing_evg)
        print(" Ok")

        # Wait some time to check and set things
        print("        Waiting some time to allow parameters to settle...", end='')
        sleep(1)
        print(" Done")

        print("        Checking if EVG Intlk Status is clear...", end='')
        # check if EVG received the event
        event_detected = timing_evg.intlk_evt_status
        if (event_detected == 0):
            print(" Ok")
        else:
            print(" Error")
            errs_bpm_evg = errs_bpm_evg + 1
            errs_bpm_evg_names.append({"bpm": bpms[i][j], "event_expected": 0, "event_detected": event_detected})

        print("        Enabling BPM interlock generation...", end='')
        # enable BPM to send the trigger
        bpms[i][j].intlk_en = BPMParams.INTLK_EN
        # wait IOC to capture the interlock status
        print(" Ok")

        print("        Waiting some time to allow EVG event to propagate...", end='')
        sleep(2)
        print(" Done")

        print("        Timing EVG Intlk Status: {}".format(timing_evg.intlk_evt_status))
        # check if EVG received the event
        event_detected = timing_evg.intlk_evt_status
        event_expected = timing_evg_evt_status_mapping[TimingEVRParams.AMC4_EVT]
        if (event_detected == event_expected):
            print("        PASSED")
        else:
            print("        FAILED")
            errs_bpm = errs_bpm + 1
            errs_bpm_names.append({"bpm": bpms[i][j], "event_expected": event_expected, "event_detected": event_detected})

    else:
        print("    Cleaning up test parameters... ")
        print("    Cleaning up sector BPM test parameters... ", end='')
        bpm_cleanup_test_parameters(bpm_sector)
        print(" Ok")
        print("    Cleaning up EVG test parameters... ", end='')
        timing_evg_clenup_test_parameters(timing_evg)
        print(" Ok")

        print("    Waiting some time to allow parameters to settle...", end='')
        sleep(1)
        print(" Done")

if (errs_bpm > 0):
    print("Test FAILED (with {} errors) for BPMs that the EVG could not detect the correct event:".format(errs_bpm))
    for err_bpm in errs_bpm_names:
        print("    BPM {:20}".format(err_bpm["bpm"].prefix))
        print("{}".format(indent(str(err_bpm["bpm"]), '        ')), end='')
        print("    Expected event: {}, detected: {}\n".format(err_bpm["event_expected"], err_bpm["event_detected"]))

if (errs_bpm_evg > 0):
    print("Test FAILED (with {} errors) for BPMs that the EVG could not reset events:".format(errs_bpm_evg))
    for err_bpm_evg in errs_bpm_evg_names:
        print("    BPM {:20}".format(err_bpm_evg["bpm"].prefix))
        print("{}".format(indent(str(err_bpm_evg["bpm"]), '        ')), end='')
        print("    Expected event: {}, detected: {}\n".format(err_bpm_evg["event_expected"], err_bpm_evg["event_detected"]))

if (errs_bpm == 0 and errs_bpm_evg == 0):
    print("Test PASSED")
    sys.exit(1)
