import numpy as np
from windfreak import SynthHD
import time

class RFsource:
    def __init__(self,
                 start_frequency=2000,
                 step_frequency=20,
                 number_of_frequencies=101,
                 intermediate_frequency=32,
                 transmit_power=0,
                 lo_power=15,
                 port='/dev/cu.usbmodem206834A04E561'):
        self._port = port
        self._synth = None
        self._fs = start_frequency
        self._df = step_frequency
        self._if = intermediate_frequency
        self._p_tx = transmit_power
        self._p_lo = lo_power
        self._N = number_of_frequencies
        self._n = 0
        self._sweep_done = False

        self._f = np.linspace(self._fs, self._fs + self._N * self._df,self._N)

    def set_frequency(self):
        self._synth.write('channel', 0)
        self._synth.write('frequency', self._f[self._n])
        self._synth.write('channel', 1)
        self._synth.write('frequency', self._f[self._n] + self._if)

    def set_power(self):
        self._synth.write('channel', 0)
        self._synth.write('power', self._p_tx)
        self._synth.write('channel', 1)
        self._synth.write('power', self._p_lo)

    def connect(self):
        self._synth = SynthHD('/dev/cu.usbmodem206834A04E561')
        self._synth.init()
        return self._synth

    def enable(self):
        self._synth[0].enable = True
        self._synth[1].enable = True
        # need to wait on lock detect

    def off(self):
        self._synth[0].enable = False
        self._synth[1].enable = False

    def frequency_step(self):
        if not self._sweep_done:
            self.set_frequency()
            self._n += 1
            if self._n == self._N:
                self._sweep_done = True
                self._n = 0
            else:
                self._sweep_done = False

    def sweep_done(self):
        return self._sweep_done

    def sweep_reset(self):
        self._n = 0
        self._sweep_done = False

    def get_frequency(self):
        return self._f

    def get_number_frequency_steps(self):
        return self._N

    def get_frequency_step(self):
        return self._df

    def get_frequency_range(self):
        return self._f

    def close(self):
        self._synth.close()
