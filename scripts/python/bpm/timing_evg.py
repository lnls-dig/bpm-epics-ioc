from epics import PV

class TimingEVG:
    """Timing EVG class"""

    def __init__(self, prefix, wait_for_connection=False, connection_timeout=5, auto_monitor=True):
        self._prefix = prefix

        # Options for epics PV
        opt = {}
        opt['connection_timeout'] = connection_timeout
        opt['auto_monitor'] = auto_monitor

        # Acquisition PVs
        pvs = {
            'IntlkCtrlEnbl': 'IntlkCtrlEnbl-Sel',
            'IntlkCtrlRst': 'IntlkCtrlRst-Sel',
            'IntlkEvtIn0': 'IntlkEvtIn0-SP',
            'IntlkEvtIn1': 'IntlkEvtIn1-SP',
            'IntlkEvtIn2': 'IntlkEvtIn2-SP',
            'RxEnblB0': 'RxEnbl-SP.B0',
            'RxEnblB1': 'RxEnbl-SP.B1',
            'RxEnblB2': 'RxEnbl-SP.B2',
            'RxEnblB3': 'RxEnbl-SP.B3',
            'RxEnblB4': 'RxEnbl-SP.B4',
            'RxEnblB5': 'RxEnbl-SP.B5',
            'RxEnblB6': 'RxEnbl-SP.B6',
            'RxEnblB7': 'RxEnbl-SP.B7',
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
            'IntlkCtrlEnbl': 'IntlkCtrlEnbl-Sts',
            'IntlkCtrlRst': 'IntlkCtrlRst-Sts',
            'IntlkEvtStatus': 'IntlkEvtStatus-Mon',
            'IntlkEvtIn0': 'IntlkEvtIn0-RB',
            'IntlkEvtIn1': 'IntlkEvtIn1-RB',
            'IntlkEvtIn2': 'IntlkEvtIn2-RB',
            'RxEnbl': 'RxEnbl-RB',
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
        """ Get Timing EVG PV prefix"""
        return self._prefix

    @property
    def intlk_ctrl_enbl(self):
        pvobj = self._config_pvs_rb['IntlkCtrlEnbl']
        return pvobj.value if pvobj.connected else None

    @intlk_ctrl_enbl.setter
    def intlk_ctrl_enbl(self, val):
        pvobj = self._config_pvs_sp['IntlkCtrlEnbl']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_ctrl_rst(self):
        pvobj = self._config_pvs_rb['IntlkCtrlRst']
        return pvobj.value if pvobj.connected else None

    @intlk_ctrl_rst.setter
    def intlk_ctrl_rst(self, val):
        pvobj = self._config_pvs_sp['IntlkCtrlRst']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_evt_in0(self):
        pvobj = self._config_pvs_rb['IntlkEvtIn0']
        return pvobj.value if pvobj.connected else None

    @intlk_evt_in0.setter
    def intlk_evt_in0(self, val):
        pvobj = self._config_pvs_sp['IntlkEvtIn0']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_evt_in1(self):
        pvobj = self._config_pvs_rb['IntlkEvtIn1']
        return pvobj.value if pvobj.connected else None

    @intlk_evt_in1.setter
    def intlk_evt_in1(self, val):
        pvobj = self._config_pvs_sp['IntlkEvtIn1']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_evt_in2(self):
        pvobj = self._config_pvs_rb['IntlkEvtIn2']
        return pvobj.value if pvobj.connected else None

    @intlk_evt_in2.setter
    def intlk_evt_in2(self, val):
        pvobj = self._config_pvs_sp['IntlkEvtIn2']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_evt_status(self):
        pvobj = self._config_pvs_rb['IntlkEvtStatus']
        return pvobj.value if pvobj.connected else None

    @property
    def rx_enbl(self):
        pvobj = self._config_pvs_rb['RxEnbl']
        return pvobj.value if pvobj.connected else None

    @rx_enbl.setter
    def rx_enbl(self, val):
        for i in range(7):
            pvobj = self._config_pvs_sp[f'RxEnblB{i}']
            if pvobj.connected:
                if (val & (0x1 << i)):
                    pvobj.put(1, wait=False)
                else:
                    pvobj.put(0, wait=False)

    def __str__(self):
        return (
            f'Intlk Ctrl Enbl      : {self.intlk_ctrl_enbl}\n'
            f'Intlk Evt In 0       : {self.intlk_evt_in0}\n'
            f'Intlk Evt In 1       : {self.intlk_evt_in1}\n'
            f'Intlk Evt In 2       : {self.intlk_evt_in2}\n'
            f'Intlk Evt Status     : {self.intlk_evt_status}\n'
            f'RX Enbl              : {self.rx_enbl}\n'
        )
