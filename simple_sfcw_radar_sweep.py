import time
import sys
from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.digital_backend import RedPitayaSampler
from radar.rf_source import  RFsource
import matplotlib.pyplot as plot
import struct
import numpy as np
from scipy import signal


if __name__ == '__main__':
    num_samples = 2048
    if_filter = IQDemodulator(f_lo=36e6, fc=4e6, ft=2e6, number_of_taps=128, fs=122.88e6, t_sample=1e-6, n=num_samples)
    bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=128, fs=122.88e6, ts=1e-6, N=num_samples)

    synth = RFsource(start_frequency=1000,
                     step_frequency=30,
                     number_of_frequencies=101,
                     intermediate_frequency=32,
                     transmit_power=0,
                     lo_power=15,
                     port='/dev/cu.usbmodem206834A04E561')
    synth.connect()
    synth.set_frequency()
    synth.set_power()
    synth.enable()
    time.sleep(0.5)

    N = synth.get_number_frequency_steps()
    x = np.zeros(N, dtype=complex)

    rpi = RedPitayaSampler(n=num_samples)
    rpi.connect()
    rpi.configure_sampler()

    print("Scanning ", end='')
    for i in range(N):
        rpi.trigger_sampler()
        time.sleep(0.05)
        dut = rpi.get_data(channel=1)
        ref = rpi.get_data(channel=2)
        ref_n = ref/np.max(np.abs(ref))
        ref_iq,lo,a=  if_filter (ref_n)
        dut_iq,lo,a =  if_filter (dut)
        bb_mixer = np.multiply(ref_iq,np.conj(dut_iq))
        bb = bb_filter(bb_mixer)
        bb= bb_filter(bb)
        x[i] = np.mean(bb[200:len(bb)//2])
        print(".", end='')
        synth.frequency_step()
        time.sleep(0.05)
    print ("done")
    f = synth.get_frequency_range()
    plot.figure(1)
    plot.plot(f, np.real(x))
    plot.plot(f, np.imag(x))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Amplitude in volts')

    plot.figure(2)
    plot.plot(f,np.angle(x))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Phase in radians')

    M=synth.get_number_frequency_steps()*32;
    r_max = 3e8/2/(synth.get_frequency_step()*1e6)
    r = np.linspace(0,r_max,M)

    window = np.kaiser(N,9);
    # window = signal.windows.dpss(N, 3)

    z = np.fft.ifft(x*window,M)

    plot.figure(3)
    plot.plot(f,np.angle(x))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Phase in radians')

    plot.figure(4)
    plot.plot(f, np.unwrap(np.angle(x)))
    plot.grid()
    plot.xlabel('Frequency in MHz')
    plot.ylabel('Phase in radians')

    plot.figure(5)
    plot.plot(r, np.abs(z))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Amplitude in volts')
    plot.title('lot of radar range profile (dB scale)')

    plot.figure(6)
    plot.plot(r, 20*np.log10(np.abs(z)))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Plot of power in dBW')
    plot.title('Plot of radar range profile (dB scale)')

    plot.figure(7)
    plot.plot(r,np.real(z))
    plot.plot(r,np.imag(z))
    plot.grid()
    plot.xlabel('Range in meters')
    plot.ylabel('Plot of power in dBW')
    plot.title('Plot of radar range profile (dB scale)')
    plot.xlim([5,7])
    plot.show()


    rpi.stop_sampler()
    synth.close()
