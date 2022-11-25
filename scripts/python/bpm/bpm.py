from epics import PV
from time import sleep

class BPMEnums:
    """BPM enumerated types"""

    TRIGDIR = {
        'Transmitter' : 'trn',
        'Receiver'    : 'rcv'
    }
    TRIGDIRPOL = {
        'Same'     : 'same',
        'Reversed' : 'rev'
    }
    TRIGSRC = {
        'External' : 'ext',
        'Internal' : 'int'
    }
    TRIGEXTERN = {
        'Machine1'          : 0,
        'Machine2'          : 1,
        'BeamLossIn'        : 2,
        'BeamLossOut'       : 3,
        'OrbitInterlock'    : 4,
        'StorageRingTbTClk' : 5,
        'BoosterTbTClk'     : 6,
        'SOFBClk'           : 7
    }
    TRIGINTERN = {
        'FMCTrigIn' : 0,
        'DSPSWClk'  : 1
    }
    LOGTRIGINTERN = {
        'ADC'          : 0,
        'ADCSwap'      : 1,
        'MixerIQ'      : 2,
        'Unconnected'  : 3,
        'TbTIQ'        : 4,
        'Unconnected2' : 5,
        'TbTAmp'       : 6,
        'TbTPha'       : 7,
        'TbTPos'       : 8,
        'FOFBIQ'       : 9,
        'Unconnected3' : 10,
        'FOFBAmp'      : 11,
        'FOFBPha'      : 12,
        'FOFBPos'      : 13,
        'MonitAmp'     : 14,
        'MonitPos'     : 15,
        'Monit1Pos'    : 16,
        'FMCTrigOut'   : 17,
        'Unconnected4' : 18,
        'Unconnected5' : 19,
        'Unconnected6' : 20,
        'Unconnected7' : 21,
        'Unconnected8' : 22,
        'Unconnected9' : 23
    }
    SWMODES = {
        'Rffe_switching' : 'rffe_switching',
        'Direct'         : 'direct',
        'Inverted'       : 'inverted',
        'Switching'      : 'switching'
    }
    SWTAGENBL = {
        'Disabled' : 'disabled',
        'Enabled'  : 'enabled'
    }
    SWDATAMASKENBL = {
        'Disabled' : 'disabled',
        'Enabled'  : 'enabled'
    }
    MONITENBL = {
        'No'  : 'No',
        'Yes' : 'Yes'
    }
    OPMODES = {
        'MultiBunch': 'MultiBunch',
        'SinglePass': 'SinglePass'
    }
    POLARITY = {
        'Positive' : 'positive',
        'Negative' : 'negative'
    }
    ENBLTYP = {
        'Disabled' : 'disabled',
        'Enabled'  : 'enabled'
    }
    ENBLDDSBLD = {
        'Disabled' : 'disabled',
        'Enabled'  : 'enabled'
    }
    ACQREPEAT = {
        'Normal'     : 'normal',
        'Repetitive' : 'repetitive'
    }
    ACQEVENTS = {
        'Start' : 'start',
        'Stop'  : 'stop',
        'Abort' : 'abort'
    }
    ACQDATATYP = {
        'A': 'A',
        'B': 'B',
        'C': 'C',
        'D': 'D'
    }
    ACQCHAN = {
        'ADC'     : 'adc',
        'ADCSwap' : 'adcswap',
        'TbT'     : 'tbt',
        'FOFB'    : 'fofb',
        'TbTPha'  : 'tbtpha',
        'FOFBPha' : 'fofbpha',
        'Monit1'  : 'monit1'
    }
    ACQSTATES = {
        'Idle'             : 'Idle',
        'Waiting'          : 'Waiting',
        'External Trig'    : 'External Trig',
        'Data Trig'        : 'Data Trig',
        'Software Trig'    : 'Software Trig',
        'Acquiring'        : 'Acquiring',
        'Error'            : 'Error',
        'Aborted'          : 'Aborted',
        'Too Many Samples' : 'Too Many Samples',
        'Too Few Samples'  : 'Too Few Samples',
        'No Memory'        : 'No Memory',
        'Acq Overflow'     : 'Acq. Overflow'
    }
    ACQTRIGTYP = {
        'Now'      : 'now',
        'External' : 'external',
        'Data'     : 'data',
        'Software' : 'software'
    }
    FMCPICORANGE = {
        '100 uA' : 0,
        '1 mA'   : 1,
    }

bpmEnums = BPMEnums

class BPM:
    """BPM Acquistiion class"""

    def __init__(self, prefix, wait_for_connection=False, connection_timeout=5, fmc_pico=False):
        self._prefix = prefix

        # Options for epics PV
        opt = {}
        opt['connection_timeout'] = connection_timeout
        self._pos_kx = PV(self._prefix + 'PosKx-RB', **opt)
        self._pos_ky = PV(self._prefix + 'PosKy-RB', **opt)
        self._pos_ksum = PV(self._prefix + 'PosKsum-RB', **opt)
        self._poly_x = PV(self._prefix + 'GEN_PolyXArrayCoeff-RB', **opt)
        self._poly_y = PV(self._prefix + 'GEN_PolyYArrayCoeff-RB', **opt)
        opt['auto_monitor'] = False
        self._array_a = PV(self._prefix + 'GEN_AArrayData', **opt)
        self._array_b = PV(self._prefix + 'GEN_BArrayData', **opt)
        self._array_c = PV(self._prefix + 'GEN_CArrayData', **opt)
        self._array_d = PV(self._prefix + 'GEN_DArrayData', **opt)
        self._array_x = PV(self._prefix + 'GEN_XArrayData', **opt)
        self._array_y = PV(self._prefix + 'GEN_YArrayData', **opt)
        self._array_s = PV(self._prefix + 'GEN_SUMArrayData', **opt)
        opt.pop('auto_monitor')
        self._offset_x = PV(self._prefix + 'PosXOffset-RB', **opt)
        self._offset_y = PV(self._prefix + 'PosYOffset-RB', **opt)

        # Acquisition PVs
        pvs = {
            'asyn.ENBL': 'asyn.ENBL',
            'ACQBPMMode': 'ACQBPMMode-Sel',
            'ACQChannel': 'ACQChannel-Sel',
            'ACQShots': 'ACQShots-SP',
            'ACQUpdateTime': 'ACQUpdateTime-SP',
            'ACQSamplesPre': 'ACQSamplesPre-SP',
            'ACQSamplesPost': 'ACQSamplesPost-SP',
            'ACQTriggerEvent': 'ACQTriggerEvent-Sel',
            'ACQTrigger': 'ACQTrigger-Sel',
            'ACQTriggerRep': 'ACQTriggerRep-Sel',
            'ACQDataTrigChan': 'ACQDataTrigChan-Sel',
            'ACQTriggerDataSel': 'ACQTriggerDataSel-SP',
            'ACQTriggerDataThres': 'ACQTriggerDataThres-SP',
            'ACQTriggerDataPol': 'ACQTriggerDataPol-Sel',
            'ACQTriggerDataHyst': 'ACQTriggerDataHyst-SP',
            'TbtTagEn': 'TbtTagEn-Sel',  # Enable TbT sync with timing
            'Monit1TagEn': 'Monit1TagEn-Sel',  # Enable Monit1 sync with timing
            'MonitTagEn': 'MonitTagEn-Sel',  # Enable Monit sync with timing
            'TbtDataMaskEn': 'TbtDataMaskEn-Sel',  # Enable use of mask
            'TbtDataMaskSamplesBeg': 'TbtDataMaskSamplesBeg-SP',
            'TbtDataMaskSamplesEnd': 'TbtDataMaskSamplesEnd-SP',
            'XYPosCal': 'XYPosCal-Sel',
            'SUMPosCal': 'SUMPosCal-Sel',
        }
        if fmc_pico:
            pvs['FMCPICORngR0'] = 'FMCPICORngR0-Sel'
            pvs['FMCPICORngR1'] = 'FMCPICORngR1-Sel'
            pvs['FMCPICORngR2'] = 'FMCPICORngR2-Sel'
            pvs['FMCPICORngR3'] = 'FMCPICORngR3-Sel'

        self._config_pvs_sp = {
            k: PV(self._prefix + v, **opt) for k, v in pvs.items()
        }
        # Ensure all PVs are connected
        self._config_pvs_sp_connected = {
            k: (v.wait_for_connection(timeout=connection_timeout) if wait_for_connection else v.connected)
                for k, v in self._config_pvs_sp.items()
        }

        pvs = {
            'asyn.ENBL': 'asyn.ENBL',
            'asyn.CNCT': 'asyn.CNCT',
            'INFOClkFreq': 'INFOClkFreq-RB',
            'INFOHarmonicNumber': 'INFOHarmonicNumber-RB',
            'INFOTBTRate': 'INFOTBTRate-RB',
            'INFOFOFBRate': 'INFOFOFBRate-RB',
            'INFOMONITRate': 'INFOMONITRate-RB',
            'INFOMONIT1Rate': 'INFOMONIT1Rate-RB',
            'ACQBPMMode': 'ACQBPMMode-Sts',
            'ACQChannel': 'ACQChannel-Sts',
            'ACQShots': 'ACQShots-RB',
            'ACQUpdateTime': 'ACQUpdateTime-RB',
            'ACQSamplesPre': 'ACQSamplesPre-RB',
            'ACQSamplesPost': 'ACQSamplesPost-RB',
            'ACQTriggerEvent': 'ACQTriggerEvent-Sts',
            'ACQStatus': 'ACQStatus-Sts',
            'ACQTrigger': 'ACQTrigger-Sts',
            'ACQTriggerRep': 'ACQTriggerRep-Sts',
            'ACQDataTrigChan': 'ACQDataTrigChan-Sts',
            'ACQTriggerDataSel': 'ACQTriggerDataSel-RB',
            'ACQTriggerDataThres': 'ACQTriggerDataThres-RB',
            'ACQTriggerDataPol': 'ACQTriggerDataPol-Sts',
            'ACQTriggerDataHyst': 'ACQTriggerDataHyst-RB',
            'TbtTagEn': 'TbtTagEn-Sts',
            'Monit1TagEn': 'Monit1TagEn-Sts',
            'MonitTagEn': 'MonitTagEn-Sts',
            'TbtDataMaskEn': 'TbtDataMaskEn-Sts',
            'TbtDataMaskSamplesBeg': 'TbtDataMaskSamplesBeg-RB',
            'TbtDataMaskSamplesEnd': 'TbtDataMaskSamplesEnd-RB',
            'XYPosCal': 'XYPosCal-Sts',
            'SUMPosCal': 'SUMPosCal-Sts',
        }
        if fmc_pico:
            pvs['FMCPICORngR0'] = 'FMCPICORngR0-Sts'
            pvs['FMCPICORngR1'] = 'FMCPICORngR1-Sts'
            pvs['FMCPICORngR2'] = 'FMCPICORngR2-Sts'
            pvs['FMCPICORngR3'] = 'FMCPICORngR3-Sts'

        self._config_pvs_rb = {
            k: PV(self._prefix + v, **opt) for k, v in pvs.items()
        }
        # Ensure all PVs are connected
        self._config_pvs_rb_connected = {
            k: (v.wait_for_connection(timeout=connection_timeout) if wait_for_connection else v.connected)
                for k, v in self._config_pvs_rb.items()
        }

    @property
    def prefix(self):
        """ Get BPM PV prefix"""
        return self._prefix

    @property
    def connected(self):
        """ Get if PVs are connected """
        conn = True
        pvs = (
            self._array_x, self._array_y, self._array_s,
            self._offset_x, self._offset_y,
            self._array_a, self._array_b, self._array_c, self._array_d,
            self._poly_x, self._poly_x,
            self._pos_kx, self._pos_ky, self._pos_ksum)
        for pvobj in pvs:
            conn &= pvobj.connected
        return conn

    @property
    def is_acq_completed(self):
        """ Get acquisition status """
        pvobj = self._config_pvs_rb['ACQStatus']
        stts = bpmEnums.ACQSTATES
        acq_stt = pvobj.get(as_string=True)
        okay = acq_stt not in {
            stts['Error'], stts['No Memory'], stts['Too Few Samples'],
            stts['Too Many Samples'], stts['Acq Overflow']}

        if self.acq_event == bpmEnums.ACQEVENTS['Start']:
            okay &= acq_stt not in {
                stts['Waiting'], stts['External Trig'], stts['Data Trig'],
                stts['Software Trig'], stts['Acquiring']}
        else:
            okay &= acq_stt in {stts['Idle'], stts['Aborted']}
        return okay

    @property
    def pos_kx(self):
        """ Get BPM Kx coefficient """
        defv = 1
        pvobj = self._pos_kx
        val = pvobj.value if pvobj.connected else defv
        return val if val else defv

    @property
    def pos_ky(self):
        """."""
        """ Get BPM Ky coefficient """
        defv = 1
        pvobj = self._pos_ky
        val = pvobj.value if pvobj.connected else defv
        return val if val else defv

    @property
    def pos_ksum(self):
        """ Get BPM KSum coefficient """
        defv = 1
        pvobj = self._pos_ksum
        val = pvobj.value if pvobj.connected else defv
        return val if val else defv

    @property
    def mode(self):
        """ Get BPM mode """
        pvobj = self._config_pvs_rb['ACQBPMMode']
        return pvobj.value if pvobj.connected else bpmEnums.OPMODES['MultiBunch']

    @mode.setter
    def mode(self, mode):
        """ Set BPM mode """
        pvobj = self._config_pvs_sp['ACQBPMMode']
        if pvobj.connected:
            pvobj.value = mode

    @property
    def array_a(self):
        """ Get BPM Array A """
        pvobj = self._array_a
        return pvobj.get() if pvobj.connected else None

    @property
    def array_b(self):
        """ Get BPM Array B """
        pvobj = self._array_b
        return pvobj.get() if pvobj.connected else None

    @property
    def array_c(self):
        """ Get BPM Array C """
        pvobj = self._array_c
        return pvobj.get() if pvobj.connected else None

    @property
    def array_d(self):
        """ Get BPM Array D """
        pvobj = self._array_d
        return pvobj.get() if pvobj.connected else None

    @property
    def pos_x(self):
        """ Get BPM Position X """
        pvobj = self._array_x
        return pvobj.get() if pvobj.connected else None

    @property
    def pos_y(self):
        """ Get BPM Position Y """
        pvobj = self._array_y
        return pvobj.get() if pvobj.connected else None

    @property
    def pos_sum(self):
        """ Get BPM Position Sum """
        pvobj = self._array_s
        return pvobj.get() if pvobj.connected else None

    @property
    def offset_x(self):
        """ Get BPM Offset X """
        pvobj = self._offset_x
        return  pvobj.value if pvobj.connected else None

    @property
    def offset_y(self):
        """ Get BPM Offset Y """
        pvobj = self._offset_y
        return  pvobj.value if pvobj.connected else None

    @property
    def acq_event(self):
        """ Get BPM trigger event """
        pvobj = self._config_pvs_rb['ACQTriggerEvent']
        return pvobj.value if pvobj.connected else None

    @acq_event.setter
    def acq_event(self, val):
        """ Set BPM trigger event """
        pvobj = self._config_pvs_sp['ACQTriggerEvent']
        if pvobj.connected:
            pvobj.put(val, wait=False)

            # Wait until acquistion is actually started
            stts = bpmEnums.ACQSTATES
            pvobj_sts = self._config_pvs_rb['ACQStatus']
            while pvobj_sts.get(as_string=True, use_monitor=False) not in {stts['Acquiring']}:
                sleep(0.1)

    @property
    def acq_channel(self):
        """ Get BPM acquistion channel """
        pvobj = self._config_pvs_rb['ACQChannel']

        if not pvobj.connected:
            return None

        # return user-facing string
        for k, v in bpmEnums.ACQCHAN.items():
            if v == pvobj.get(as_string=True):
                return k

        return 'Invalid'

    @acq_channel.setter
    def acq_channel(self, val):
        """ Set BPM acquistion channel """
        pvobj = self._config_pvs_sp['ACQChannel']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trigger(self):
        """ Get BPM acquistion trigger """
        # pvobj = self._config_pvs_rb['ACQTriggerType']
        pvobj = self._config_pvs_rb['ACQTrigger']
        return pvobj.value if pvobj.connected else None

    @acq_trigger.setter
    def acq_trigger(self, val):
        """ Get BPM acquistion trigger """
        pvobj = self._config_pvs_sp['ACQTrigger']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_repeat(self):
        """ Get BPM acquisition repeatitive trigger """
        pvobj = self._config_pvs_rb['ACQTriggerRep']
        return pvobj.value if pvobj.connected else None

    @acq_repeat.setter
    def acq_repeat(self, val):
        """ Set BPM acquisition repeatitive trigger """
        pvobj = self._config_pvs_sp['ACQTriggerRep']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trig_datatype(self):
        """ Get BPM acquisition data trigger channel """
        pvobj = self._config_pvs_rb['ACQDataTrigChan']
        return pvobj.value if pvobj.connected else None

    @acq_trig_datatype.setter
    def acq_trig_datatype(self, val):
        """ Set BPM acquisition data trigger channel """
        pvobj = self._config_pvs_sp['ACQDataTrigChan']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trig_datasel(self):
        """ Get BPM acquisition data trigger selection """
        pvobj = self._config_pvs_rb['ACQTriggerDataSel']
        return pvobj.value if pvobj.connected else None

    @acq_trig_datasel.setter
    def acq_trig_datasel(self, val):
        """ Set BPM acquisition data trigger selection """
        pvobj = self._config_pvs_sp['ACQTriggerDataSel']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trig_datathres(self):
        """ Get BPM acquisition data trigger threshold """
        pvobj = self._config_pvs_rb['ACQTriggerDataThres']
        return pvobj.value if pvobj.connected else None

    @acq_trig_datathres.setter
    def acq_trig_datathres(self, val):
        """ Set BPM acquisition data trigger threshold """
        pvobj = self._config_pvs_sp['ACQTriggerDataThres']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trig_datahyst(self):
        """ Get BPM acquisition data trigger hysteresis """
        pvobj = self._config_pvs_rb['ACQTriggerDataHyst']
        return pvobj.value if pvobj.connected else None

    @acq_trig_datahyst.setter
    def acq_trig_datahyst(self, val):
        """ Set BPM acquisition data trigger hysteresis """
        pvobj = self._config_pvs_sp['ACQTriggerDataHyst']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def acq_trig_datapol(self):
        """ Get BPM acquisition data trigger polarity """
        pvobj = self._config_pvs_rb['ACQTriggerDataPol']
        return pvobj.value if pvobj.connected else None

    @acq_trig_datapol.setter
    def acq_trig_datapol(self, val):
        """ Set BPM acquisition data trigger polarity """
        pvobj = self._config_pvs_sp['ACQTriggerDataPol']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def tbt_sync_enbl(self):
        """ Get BPM TbT synchronization """
        pvobj = self._config_pvs_rb['TbtTagEn']
        return pvobj.value if pvobj.connected else None

    @tbt_sync_enbl.setter
    def tbt_sync_enbl(self, val):
        """ Set BPM TbT synchronization """
        pvobj = self._config_pvs_sp['TbtTagEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def monit1_sync_enbl(self):
        """ Get BPM Monit1 synchronization """
        pvobj = self._config_pvs_rb['Monit1TagEn']
        return pvobj.value if pvobj.connected else None

    @monit1_sync_enbl.setter
    def monit1_sync_enbl(self, val):
        """ Set BPM Monit1 synchronization """
        pvobj = self._config_pvs_sp['Monit1TagEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def monit_sync_enbl(self):
        """ Get BPM Monit synchronization """
        pvobj = self._config_pvs_rb['MonitTagEn']
        return pvobj.value if pvobj.connected else None

    @monit_sync_enbl.setter
    def monit_sync_enbl(self, val):
        """ Set BPM Monit synchronization """
        pvobj = self._config_pvs_sp['MonitTagEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def tbt_mask_enbl(self):
        """ Get BPM TbT masking """
        pvobj = self._config_pvs_rb['TbtDataMaskEn']
        return pvobj.value if pvobj.connected else None

    @tbt_mask_enbl.setter
    def tbt_mask_enbl(self, val):
        """ Set BPM TbT masking """
        pvobj = self._config_pvs_sp['TbtDataMaskEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def tbt_mask_begin(self):
        """ Get BPM TbT masking beginning"""
        pvobj = self._config_pvs_rb['TbtDataMaskSamplesBeg']
        return pvobj.value if pvobj.connected else None

    @tbt_mask_begin.setter
    def tbt_mask_begin(self, val):
        """ Set BPM TbT masking beginning """
        pvobj = self._config_pvs_sp['TbtDataMaskSamplesBeg']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def tbt_mask_end(self):
        """ Get BPM TbT masking ending """
        pvobj = self._config_pvs_rb['TbtDataMaskSamplesEnd']
        return pvobj.value if pvobj.connected else None

    @tbt_mask_end.setter
    def tbt_mask_end(self, val):
        """ Set BPM TbT masking ending """
        pvobj = self._config_pvs_sp['TbtDataMaskSamplesEnd']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def poly_x(self):
        """ Get BPM polynomial coefficientes for X """
        pvobj = self._poly_x
        return pvobj.value if pvobj.connected else None

    @property
    def poly_y(self):
        """ Get BPM polynomial coefficientes for Y """
        pvobj = self._poly_y
        return pvobj.value if pvobj.connected else None

    @property
    def poly_cal(self):
        """ Get BPM X/Y correction polynom """
        pvobj = self._config_pvs_rb['XYPosCal']
        return pvobj.value if pvobj.connected else None

    @poly_cal.setter
    def poly_cal(self, val):
        """ Set BPM X/Y correction polynom """
        val = bpmEnums.ENBLDDSBLD['Enabled'] if val else bpmEnums.ENBLDDSBLD['Disabled']
        pv1 = self._config_pvs_sp['XYPosCal']
        pv2 = self._config_pvs_sp['SUMPosCal']
        if pv1.connected:
            pv1.put(val, wait=False)
        if pv2.connected:
            pv2.put(val, wait=False)

    @property
    def nr_samples_post(self):
        """ Get BPM number of post-trigger samples """
        pvobj = self._config_pvs_rb['ACQSamplesPost']
        return pvobj.value if pvobj.connected else None

    @nr_samples_post.setter
    def nr_samples_post(self, val):
        """ Set BPM number of post-trigger samples """
        # pvobj = self._config_pvs_sp['ACQnr_samples_post']
        pvobj = self._config_pvs_sp['ACQSamplesPost']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def nr_samples_pre(self):
        """ Get BPM number of pre-trigger samples """
        pvobj = self._config_pvs_rb['ACQSamplesPre']
        return pvobj.value if pvobj.connected else None

    @nr_samples_pre.setter
    def nr_samples_pre(self, val):
        """ Set BPM number of pre-trigger samples """
        pvobj = self._config_pvs_sp['ACQSamplesPre']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def nr_shots(self):
        """ Get BPM number of shots """
        pvobj = self._config_pvs_rb['ACQShots']
        return pvobj.value if pvobj.connected else None

    @nr_shots.setter
    def nr_shots(self, val):
        """ Set BPM number of shots """
        pvobj = self._config_pvs_sp['ACQShots']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def fmc_pico_range_ch0(self):
        """ Get BPM FMCPICO range channel 0 """
        pvobj = self._config_pvs_rb['FMCPICORngR0']
        return pvobj.value if pvobj.connected else None

    @fmc_pico_range_ch0.setter
    def fmc_pico_range_ch0(self, val):
        """ Set BPM FMC PICO range for channel 0 """
        pvobj = self._config_pvs_sp['FMCPICORngR0']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def fmc_pico_range_ch1(self):
        """ Get BPM FMCPICO range channel 1 """
        pvobj = self._config_pvs_rb['FMCPICORngR1']
        return pvobj.value if pvobj.connected else None

    @fmc_pico_range_ch1.setter
    def fmc_pico_range_ch1(self, val):
        """ Set BPM FMC PICO range for channel 1 """
        pvobj = self._config_pvs_sp['FMCPICORngR1']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def fmc_pico_range_ch2(self):
        """ Get BPM FMCPICO range channel 2 """
        pvobj = self._config_pvs_rb['FMCPICORngR2']
        return pvobj.value if pvobj.connected else None

    @fmc_pico_range_ch2.setter
    def fmc_pico_range_ch2(self, val):
        """ Set BPM FMC PICO range for channel 2 """
        pvobj = self._config_pvs_sp['FMCPICORngR2']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def fmc_pico_range_ch3(self):
        """ Get BPM FMCPICO range channel 3 """
        pvobj = self._config_pvs_rb['FMCPICORngR3']
        return pvobj.value if pvobj.connected else None

    @fmc_pico_range_ch3.setter
    def fmc_pico_range_ch3(self, val):
        """ Set BPM FMC PICO range for channel 3 """
        pvobj = self._config_pvs_sp['FMCPICORngR3']
        if pvobj.connected:
            pvobj.put(val, wait=False)
