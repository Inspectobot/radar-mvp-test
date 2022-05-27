import time
import sys
import ctypes as c

from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import matplotlib.pyplot as plot
import numpy as np
from scipy import signal

import socket

import csv
if __name__ == '__main__':
    num_samples = 2048
    number_of_frequencies = 101
    number_of_channels = 2
    start_frequency = 1000
    step_frequency = 10
    intermediate_frequency = 32
    transmit_power = 0
    lo_power = 15

    RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)

    if_filter = IQDemodulator(f_lo=36e6, fc=4e6, ft=1e6, number_of_taps=256, fs=122.88e6, t_sample=1e-6, n=num_samples) 
    bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=256, fs=122.88e6, ts=1e-6, N=num_samples)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('eli-rover.local', 1001)

    sock.connect(server_address)
    profile = RadarProfile()
    data = sock.recv(c.sizeof(RadarProfile()), socket.MSG_WAITALL)
    c.memmove(c.addressof(profile), data, c.sizeof(profile))

    N = number_of_frequencies
    x = np.zeros(N, dtype=complex)
    f = np.linspace(start_frequency, start_frequency + ((number_of_frequencies - 1) * step_frequency), number_of_frequencies)

    print(f)

    for i in range(N):
      dut = profile.getSamplesAtIndex(1, i)
      ref = profile.getSamplesAtIndex(0, i)

      np.savetxt('dut_' + str(i) + '.csv', dut, delimiter=",", newline="\n")
      np.savetxt('ref_' + str(i) + '.csv', ref, delimiter=",", newline="\n")

      print(dut)
      print(ref)


      print(i)
      print('dut')
      print(len(dut))
      print('ref')
      print(len(ref))
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
    plot.figure(1)
    plot.plot(f, np.real(x),alpha=0.5)
    plot.plot(f, np.imag(x),alpha=0.5)
    plot.plot(f,np.abs(x),'k',alpha=0.3)
    plot.plot(f,-np.abs(x),'k',alpha=0.3)
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Amplitude in volts')
    plot.title('Plot of raw I,Q data')

    plot.figure(2)
    plot.plot(f,np.angle(x))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Phase in radians')
    plot.title('Plot of phase as a function of frequency')

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

    plot.figure(3)
    plot.plot(f, np.unwrap(np.angle(x)))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Phase in radians')
    plot.title('Plot of unwrapped phase as a function of frequency')

    plot.figure(4)
    plot.plot(r, np.abs(z))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Amplitude in volts')
    plot.title('Plot of radar range profile (Linear)')

    plot.figure(5)
    plot.plot(r, 20*np.log10(np.abs(z)))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Plot of power in dBW')
    plot.title('Plot of radar range profile (dB scale)')

    plot.figure(6)
    plot.plot(r,np.real(z))
    plot.plot(r,np.imag(z))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Plot of amplitude')
    plot.title('Plot of radar range profile ')
    plot.xlim([0.5,1.5])
    plot.show()
