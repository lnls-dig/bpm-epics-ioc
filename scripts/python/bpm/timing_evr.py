from epics import PV

class TimingEVR:
    """Timing EVR class"""

    def __init__(self, prefix, wait_for_connection=False, connection_timeout=5, auto_monitor=True):
        self._prefix = prefix

        # Options for epics PV
        opt = {}
        opt['connection_timeout'] = connection_timeout
        opt['auto_monitor'] = auto_monitor

        # Acquisition PVs
        pvs = {
            'asyn.ENBL': 'asyn.ENBL',
            'AMC4State': 'AMC4State-Sel',
            'AMC4Evt': 'AMC4Evt-SP',
            'AMC4NrPulses': 'AMC4NrPulses-SP',
            'AMC4Src': 'AMC4Src-Sel',
            'AMC4Dir': 'AMC4Dir-Sel',
        }
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
            'AMC4State': 'AMC4State-Sts',
            'AMC4Evt': 'AMC4Evt-RB',
            'AMC4NrPulses': 'AMC4NrPulses-RB',
            'AMC4Src': 'AMC4Src-Sts',
            'AMC4Dir': 'AMC4Dir-Sts',
        }
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
        """ Get Timing EVR PV prefix"""
        return self._prefix

    @property
    def connected(self):
        pvobj = self._config_pvs_rb['asyn.CNCT']
        return pvobj.value if pvobj.connected else None

    @property
    def enable(self):
        pvobj = self._config_pvs_rb['asyn.ENBL']
        return pvobj.value if pvobj.connected else None

    @enable.setter
    def enable(self, val):
        pvobj = self._config_pvs_sp['asyn.ENBL']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def amc4_state(self):
        pvobj = self._config_pvs_rb['AMC4State']
        return pvobj.value if pvobj.connected else None

    @amc4_state.setter
    def amc4_state(self, val):
        pvobj = self._config_pvs_sp['AMC4State']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def amc4_evt(self):
        pvobj = self._config_pvs_rb['AMC4Evt']
        return pvobj.value if pvobj.connected else None

    @amc4_evt.setter
    def amc4_evt(self, val):
        pvobj = self._config_pvs_sp['AMC4Evt']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def amc4_nr_pulses(self):
        pvobj = self._config_pvs_rb['AMC4NrPulses']
        return pvobj.value if pvobj.connected else None

    @amc4_nr_pulses.setter
    def amc4_nr_pulses(self, val):
        pvobj = self._config_pvs_sp['AMC4NrPulses']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def amc4_src(self):
        pvobj = self._config_pvs_rb['AMC4Src']
        return pvobj.value if pvobj.connected else None

    @amc4_src.setter
    def amc4_src(self, val):
        pvobj = self._config_pvs_sp['AMC4Src']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def amc4_dir(self):
        pvobj = self._config_pvs_rb['AMC4Dir']
        return pvobj.value if pvobj.connected else None

    @amc4_dir.setter
    def amc4_dir(self, val):
        pvobj = self._config_pvs_sp['AMC4Dir']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    def __str__(self):
        return (
            f'connected            : {self.connected}\n'
            f'enabled              : {self.enable}\n'
            f'AMC4 State           : {self.amc4_state}\n'
            f'AMC4 Evt             : {self.amc4_evt}\n'
            f'AMC4 Nr Pulses       : {self.amc4_nr_pulses}\n'
            f'AMC4 Src             : {self.amc4_src}\n'
            f'ACM4 Dir             : {self.amc4_dir}\n'
        )
