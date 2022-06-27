from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import numpy as np
from scipy import signal

import h5py

def process_sweep(filename, dataset_key):
    print("processing job", filename, dataset_key)

    scanFile = h5py.File(filename, "r")

    sweepDataSet = scanFile[dataset_key]

    num_samples = sweepDataSet.attrs['sampleCount']
    number_of_frequencies = sweepDataSet.attrs['frequencyCount']
    number_of_channels = sweepDataSet.attrs['channelCount']
    start_frequency = sweepDataSet.attrs['startFrequency']
    step_frequency = sweepDataSet.attrs['stepFrequency']
    intermediate_frequency = sweepDataSet.attrs['intermediateFreq']
    transmit_power = sweepDataSet.attrs['transmitPower']
    lo_power = sweepDataSet.attrs['loPower']

    RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)
    profile = RadarProfile()

    profile.setArrayFromDataset(sweepDataSet)

    scanFile.close()

    if_filter = IQDemodulator(f_lo=36e6, fc=4e6, ft=1e6, number_of_taps=256, fs=122.88e6, t_sample=1e-6, n=num_samples)
    bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=256, fs=122.88e6, ts=1e-6, N=num_samples)

    N = number_of_frequencies
    x = np.zeros(N, dtype=complex)
    f = np.linspace(start_frequency, start_frequency + ((number_of_frequencies - 1) * step_frequency), number_of_frequencies)

    for i in range(N):
      dut = profile.getSamplesAtIndex(1, i)
      ref = profile.getSamplesAtIndex(0, i)

      ref_n = ref/np.max(np.abs(ref))
      ref_iq,lo,a=  if_filter (ref_n)
      dut_iq,lo,a =  if_filter (dut)
      bb_mixer = np.multiply(ref_iq/np.abs(ref_iq),np.conj(dut_iq))
      bb = bb_filter(bb_mixer)
      bb = bb_filter(bb)
      x[i] = np.mean(bb[200:len(bb)//2])

    f = f[1:]
    x = x[1:]
    N=N-1

    M=number_of_frequencies*10;
    r_max = 3e8/2/(step_frequency*1e6)
    r = np.linspace(0,r_max,M)

    window = np.kaiser(N,9);
    #window = signal.windows.dpss(N, 3)
    #window = np.hamming(N)
    x_w = x*window
    x_w = x_w/np.max(np.abs(x_w))
    z = np.fft.ifft(x_w,M)
    z = z/np.max(np.abs(z))

    return 1
