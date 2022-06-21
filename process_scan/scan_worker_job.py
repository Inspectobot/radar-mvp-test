import sys
sys.path.append("..")
import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"


from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import numpy as np
from scipy import signal

import h5py

import asyncio



def process_sweep(ctx, filename, dataset_key,
    num_samples,
    number_of_frequencies,
    number_of_channels,
    start_frequency,
    step_frequency,
    intermediate_frequency,
    transmit_power,
    lo_power,
    posePos,
    poseRot, ProfileArray):

    print("processing job", filename, dataset_key)

    RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)
    profile = RadarProfile()

    profile._array = ProfileArray # this is ratchet but whatever

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

    scanFile = h5py.File(filename + '_' + dataset_key + "-range.hdf5", "w")

    rangeProfileDataSet = scanFile.create_dataset('range-' + dataset_key, (z.size), dtype=np.clongdouble)

    rangeProfileDataSet.attrs['sampleCount'] = num_samples
    rangeProfileDataSet.attrs['frequencyCount'] = number_of_frequencies
    rangeProfileDataSet.attrs['channelCount'] = number_of_channels
    rangeProfileDataSet.attrs['start_frequency'] = start_frequency
    rangeProfileDataSet.attrs['step_frequency'] = step_frequency
    rangeProfileDataSet.attrs['intermediate_frequency'] = intermediate_frequency
    rangeProfileDataSet.attrs['transmitPower'] = transmit_power
    rangeProfileDataSet.attrs['loPower'] = lo_power
    rangeProfileDataSet.attrs['pose.pos'] = posePos
    rangeProfileDataSet.attrs['pose.rot'] = poseRot

    rangeProfileDataSet.write_direct(z)

    scanFile.close()
    print("Job Done {}".format(dataset_key))
