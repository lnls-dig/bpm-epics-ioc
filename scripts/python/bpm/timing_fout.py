from epics import PV

class TimingFOUT:
    """Timing Fanout class"""

    def __init__(self, prefix, wait_for_connection=False, connection_timeout=5, auto_monitor=True):
        self._prefix = prefix

        # Options for epics PV
        opt = {}
        opt['connection_timeout'] = connection_timeout
        opt['auto_monitor'] = auto_monitor

        #  PVs
        pvs = {
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
        """ Get Timing Fanout PV prefix"""
        return self._prefix

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
            f'RX enable         : {self.rx_enbl}\n'
        )
