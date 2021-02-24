from epics import PV

class BPMIntlk:
    """BPM Interlock class"""

    def __init__(self, prefix, wait_for_connection=False, connection_timeout=5, auto_monitor=True):
        self._prefix = prefix

        # Options for epics PV
        opt = {}
        opt['connection_timeout'] = connection_timeout
        opt['auto_monitor'] = auto_monitor

        pvs = {
            'asyn.ENBL': 'asyn.ENBL',
            'IntlkLmtTransMaxX': 'IntlkLmtTransMaxX-SP',
            'IntlkLmtTransMinX': 'IntlkLmtTransMinX-SP',
            'IntlkTransEn': 'IntlkTransEn-Sel',
            'IntlkEn': 'IntlkEn-Sel',
            'TRIGGER4TrnSrc': 'TRIGGER4TrnSrc-Sel',
            'TRIGGER4TrnOutSel': 'TRIGGER4TrnOutSel-SP',
            'TRIGGER4Dir': 'TRIGGER4Dir-Sel',
            'TRIGGER4TrnLen': 'TRIGGER4TrnLen-SP',
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
            'IntlkLmtTransMaxX': 'IntlkLmtTransMaxX-RB',
            'IntlkLmtTransMinX': 'IntlkLmtTransMinX-RB',
            'IntlkTransEn': 'IntlkTransEn-Sts',
            'IntlkEn': 'IntlkEn-Sts',
            'TRIGGER4TrnSrc': 'TRIGGER4TrnSrc-Sts',
            'TRIGGER4TrnOutSel': 'TRIGGER4TrnOutSel-RB',
            'TRIGGER4Dir': 'TRIGGER4Dir-Sts',
            'TRIGGER4TrnLen': 'TRIGGER4TrnLen-RB',
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
        """ Get BPM PV prefix"""
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
    def intlk_lmt_trans_max_x(self):
        pvobj = self._config_pvs_rb['IntlkLmtTransMaxX']
        return pvobj.value if pvobj.connected else None

    @intlk_lmt_trans_max_x.setter
    def intlk_lmt_trans_max_x(self, val):
        pvobj = self._config_pvs_sp['IntlkLmtTransMaxX']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_lmt_trans_min_x(self):
        pvobj = self._config_pvs_rb['IntlkLmtTransMinX']
        return pvobj.value if pvobj.connected else None

    @intlk_lmt_trans_min_x.setter
    def intlk_lmt_trans_min_x(self, val):
        pvobj = self._config_pvs_sp['IntlkLmtTransMinX']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_trans_en(self):
        pvobj = self._config_pvs_rb['IntlkTransEn']
        return pvobj.value if pvobj.connected else None

    @intlk_trans_en.setter
    def intlk_trans_en(self, val):
        pvobj = self._config_pvs_sp['IntlkTransEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def intlk_en(self):
        pvobj = self._config_pvs_rb['IntlkEn']
        return pvobj.value if pvobj.connected else None

    @intlk_en.setter
    def intlk_en(self, val):
        pvobj = self._config_pvs_sp['IntlkEn']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def trigger4_trn_src(self):
        pvobj = self._config_pvs_rb['TRIGGER4TrnSrc']
        return pvobj.value if pvobj.connected else None

    @trigger4_trn_src.setter
    def trigger4_trn_src(self, val):
        pvobj = self._config_pvs_sp['TRIGGER4TrnSrc']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def trigger4_trn_out_sel(self):
        pvobj = self._config_pvs_rb['TRIGGER4TrnOutSel']
        return pvobj.value if pvobj.connected else None

    @trigger4_trn_out_sel.setter
    def trigger4_trn_out_sel(self, val):
        pvobj = self._config_pvs_sp['TRIGGER4TrnOutSel']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def trigger4_dir(self):
        pvobj = self._config_pvs_rb['TRIGGER4Dir']
        return pvobj.value if pvobj.connected else None

    @trigger4_dir.setter
    def trigger4_dir(self, val):
        pvobj = self._config_pvs_sp['TRIGGER4Dir']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    @property
    def trigger4_trn_len(self):
        pvobj = self._config_pvs_rb['TRIGGER4TrnLen']
        return pvobj.value if pvobj.connected else None

    @trigger4_trn_len.setter
    def trigger4_trn_len(self, val):
        pvobj = self._config_pvs_sp['TRIGGER4TrnLen']
        if pvobj.connected:
            pvobj.put(val, wait=False)

    def __str__(self):
        return (
            f'connected            : {self.connected}\n'
            f'enabled              : {self.enable}\n'
            f'Intlk Trans Max X    : {self.intlk_lmt_trans_max_x}\n'
            f'Intlk Trans En       : {self.intlk_trans_en}\n'
            f'Intlk En             : {self.intlk_en}\n'
            f'Trigger4 Trn Src     : {self.trigger4_trn_src}\n'
            f'Trigger4 Trn Out Sel : {self.trigger4_trn_out_sel}\n'
            f'Trigger4 Dir         : {self.trigger4_dir}\n'
            f'Trigger4 Trn Len     : {self.trigger4_trn_len}\n'
        )
