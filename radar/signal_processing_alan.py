import numpy as np
from scipy import signal


class LowPassFilter:
    def __init__(self, fc=4e6, ft=0.5e6, number_of_taps=128, fs=122.88e6, ts=1e-6, n=2048, filter_type="low"):
        self._fc = fc
        self._ft = ft
        self._number_of_taps = number_of_taps
        self._fs = fs
        self._dt = 1 / fs
        self._ts = ts
        self._L = int(np.round(ts * fs))
        self._N = n
        self._T = n * self._dt
        self._y_ts = 0
        if filter_type == "low":
            self._edge = [0, fc, fc + ft, 0.5 * fs]
            self._b = signal.remez(number_of_taps, self._edge, [1, 0], Hz=fs)
        else:
            self._edge = [0, fc - ft, fc, 0.5 * fs]
            self._b = signal.remez(number_of_taps, self._edge, [0,1], Hz=fs)
        self._a = [1]
        self._x = np.array([])

        self._SN = int(np.round(self._ts/self._dt))

    def __call__(self, x, padlen=150):
        self._x = x
        self._y = signal.filtfilt(self._b, self._a, x,padlen=padlen)
        return self._y[self._SN:], (self._N - self._SN)

    def get_settling_time(self):
        return self._SN

    def sample(self):
        self._y_ts = 2 * np.mean(self._y[self._L:self._N // 2])
        return self._y_ts


class BandPassFilter:
    def __init__(self, fc=32e6, bw=1e6, ft=1e6, number_of_taps=256, fs=122.88e6, ts=1e-6, N=16384):
        self._fc = fc
        self._bw = bw
        self._ft = ft
        self._number_of_taps = number_of_taps
        self._fs = fs
        self._dt = 1 / fs
        self._ts = ts
        self._L = int(np.round(ts * fs))
        self._N = N
        self._T = N * self._dt
        self._y_ts = 0
        self._edge = [0, fc - bw / 2 - ft, fc - bw / 2, fc + bw / 2, fc + bw / 2 + ft, 0.5 * fs]
        self._b = signal.remez(number_of_taps, self._edge, [0, 1, 0], Hz=fs)
        self._a = [1]
        self._x = np.array([])

        self._SN = int(np.round(self._ts / self._dt))

    def __call__(self, x, padlen=150):
        self._x = x
        self._y = signal.filtfilt(self._b, self._a, x)
        return self._y[self._SN:], (self._N-self._SN)

    def get_settling_time(self):
        return self._SN

    def sample(self):
        self._y_ts = 2 * np.mean(self._y[self._L:self._N // 2])
        return self._y_ts


class IQDemodulator:
    def __init__(self, f_lo=31e6, fc=1e6, ft=0.1e6, number_of_taps=128, fs=122.88e6, ts=1e-6, n=16384):
        self._f_lo=f_lo
        self._fc = fc
        self._ft= ft
        self._number_of_taps=number_of_taps
        self._fs=fs
        self._dt=1/fs
        self._ts = ts
        self._n=n

        self._t = np.linspace(0,self._n*self._dt,self._n)

        #Create lo frequency
        self._lo = np.exp(-1j*2*np.pi*self._f_lo*self._t)

        #Build filter
        self._edge = [0, fc, fc + ft, 0.5 * fs]
        self._b = signal.remez(number_of_taps, self._edge, [1, 0], Hz=fs)
        self._a = [1]
        self._x = np.array([],dtype=complex)

        self._y = np.array([],dtype=complex)

        self._SN = int(np.round(self._ts / self._dt))

    def __call__(self, x):
        x_mixer_o = np.multiply(x, self._lo)
        self._y = signal.filtfilt(self._b,self._a,x_mixer_o)
        return self._y[self._SN:],self._lo[self._SN:],x_mixer_o[self._SN:]

    def get_settling_time(self):
        return self._SN
