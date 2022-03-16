import radar.redpitaya_scpi as scpi
import numpy as np
import struct


class RedPitayaSampler:
    """
    Control sampling
    """
    def __init__(self, ip='192.168.0.153', fs=122.88e6, n=1024):
        self._ip = ip
        self._rp_s = None
        self._fs = fs
        self._dt = 1 / fs
        # need to set this ones along with time series.
        self._n = n
        self._t = np.linspace(0, self._n * self._dt, self._n)
        self._f = np.linspace(0, self._fs / 2, self._n // 2)

    def connect(self):
        self._rp_s = scpi.scpi(self._ip)
        return self._rp_s

    def configure_sampler(self):
        self._rp_s.tx_txt('ACQ:DATA:FORMAT BIN')
        self._rp_s.tx_txt('ACQ:DATA:UNITS VOLTS')
        self._rp_s.tx_txt('ACQ:DEC 1')

    def start_sampler(self):
        self._rp_s.tx_txt('ACQ:START')

    def stop_sampler(self):
        self._rp_s.tx_txt('ACQ:STOP')

    def reset_sampler(self):
        self._rp_s.tx_txt('ACQ:RST')

    def trigger_sampler(self):
        self._rp_s.tx_txt('ACQ:START')
        self._rp_s.tx_txt('ACQ:TRIG NOW')
        while 1:
            self._rp_s.tx_txt('ACQ:TRIG:STAT?')
            if self._rp_s.rx_txt() == 'TD':
                break
        self._rp_s.tx_txt('ACQ:STOP')

    def get_data(self, channel=1):
        self._rp_s.tx_txt(f'ACQ:SOUR{channel}:DATA?')
        buff_byte = self._rp_s.rx_arb()
        buff = [struct.unpack('!f', bytearray(buff_byte[i:i + 4]))[0] for i in range(0, len(buff_byte), 4)]
        x = np.array(buff)
        return x[:self._n].copy()


    def get_number_samples(self):
        return self._n

    def get_t(self):
        return self._t

    def get_f(self):
        return self._f
