import time
import sys
import ctypes as c

from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import ProfileType, CreateRadarProfile 

import matplotlib.pyplot as plot
import numpy as np
from scipy import signal

import socket

if __name__ == '__main__':
    num_samples = 101
    number_of_frequencies = 101
    start_frequency = 1000
    step_frequency = 10
    number_of_frequencies = 201
    intermediate_frequency = 20
    transmit_power = 0
    lo_power = 15

    RadarProfile = CreateRadarProfile(num_samples, number_of_frequencies)

    if_filter = IQDemodulator(f_lo=24e6, fc=4e6, ft=1e6, number_of_taps=128, fs=122.88e6, t_sample=1e-6, n=num_samples)
    bb_filter = LowPassFilter(fc=1e6, ft=1e6, number_of_taps=128, fs=122.88e6, ts=1e-6, N=num_samples)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('radar', 1001)

    dutProfile = None
    refProfile = None

    sock.connect(server_address)
    
    print("Scanning ", end='')
    while (dutProfile is None or refProfile is None) or (dutProfile.timestamp != refProfile.timestamp):
      profile = RadarProfile()
      data = sock.recv(c.sizeof(RadarProfile()), socket.MSG_WAITALL)
      c.memmove(c.addressof(profile), data, c.sizeof(profile))

      if(profile.type == ProfileType.DUT):
        dutProfile = profile
      
      if(profile.type == ProfileType.REF):
        refProfile = profile

    N = 101
    x = np.zeros(N, dtype=complex)
    f = np.linspace(start_frequency, start_frequency + number_of_frequencies * step_frequency, number_of_frequencies)

    for i in range(N):
      dut = dutProfile.getSamplesAtIndex(i)
      ref = refProfile.getSamplesAtIndex(i)
      ref_n = ref/np.max(np.abs(ref))
      ref_iq,lo,a=  if_filter (ref_n)
      dut_iq,lo,a =  if_filter (dut)
      bb_mixer = np.multiply(ref_iq/np.abs(ref_iq),np.conj(dut_iq))
      bb = bb_filter(bb_mixer)
      bb = bb_filter(bb)
      x[i] = np.mean(bb[200:len(bb)//2])
      print(".", end='')
    
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

    M=synth.get_number_frequency_steps()*10;
    r_max = 3e8/2/(synth.get_frequency_step()*1e6)
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
