import time

import matplotlib.pyplot as plot
import numpy as np

from radar.signal_processing import LowPassFilter, IQDemodulator
from radar.digital_backend import RedPitayaSampler
from radar.rf_source import RFsource

if __name__ == '__main__':
    num_samples = 2048
    if_filter = IQDemodulator(f_lo=36e6, fc=4e6, ft=2e6, number_of_taps=128, fs=122.88e6, t_sample=1e-6, n=num_samples)
    bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=128, fs=122.88e6, ts=1e-6, N=num_samples)

    synth = RFsource(start_frequency=2000,
                     step_frequency=10,
                     number_of_frequencies=101,
                     intermediate_frequency=32,
                     transmit_power=0,
                     lo_power=15,
                     port='/dev/cu.usbmodem206834A0z4E561')
    synth.connect()
    synth.set_frequency()
    synth.set_power()
    synth.enable()

    rpi = RedPitayaSampler(n=num_samples)
    rpi.connect()
    rpi.configure_sampler()
    rpi.start_sampler()
    time.sleep(1)
    rpi.trigger_sampler()

    time.sleep(1)

    dut = rpi.get_data(channel=1)
    ref = rpi.get_data(channel=2)

    #ref_n = ref / np.max(np.abs(ref))
    ref_n = ref
    dut_iq, lo, x_mixer_o = if_filter(dut)
    ref_iq, lo, x_mixer_o = if_filter(ref_n)

    bb_mixer = np.multiply(ref_iq, np.conj(dut_iq))
    bb = bb_filter(bb_mixer)
    bb = bb_filter(bb)

    x = np.mean(bb[200:num_samples//2])
    print(f"f={synth.get_frequency()[0]}, Mag(x) = {np.abs(x)}, I={np.real(x)},Q={np.imag(x)}")

    t = rpi.get_t()
    f = rpi.get_f()

    plot.figure(1)
    plot.plot(t[:100] / 1e-6, dut[:100])
    plot.plot(t[:100] / 1e-6, ref[:100])
    plot.xlabel('Time (us)')
    plot.ylabel('Voltage')
    plot.title("Raw dut/ref channels")

    plot.figure(2)
    plot.plot(t[200:num_samples//2] / 1e-6, np.real(dut_iq[200:num_samples//2]))
    plot.plot(t[200:num_samples//2] / 1e-6, np.real(ref_iq[200:num_samples//2]))
    plot.xlabel('Time (us)')
    plot.ylabel('Voltage')
    plot.title("Raw dut/ref IQ")

    plot.figure(3)
    z = np.fft.ifft(dut * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])))
    plot.grid()
    plot.title('Spectrum raw dut')

    plot.figure(4)
    z = np.fft.ifft(dut_iq * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])), alpha=0.5)
    z = np.fft.ifft(ref_iq * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])), alpha=0.5)

    plot.grid()
    plot.title('Spectrum post IQ demod')

    plot.figure(5)
    z = np.fft.fft(np.conj(x_mixer_o) * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])))
    plot.grid()
    plot.title('Spectrum Pre filtering of IQ')

    plot.figure(6)
    z = np.fft.fft(np.conj(lo) * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])))
    plot.grid()
    plot.title('Spectrum LO')

    plot.figure(7)
    plot.plot(t[:500] / 1e-6, np.real(lo[:500]))
    plot.xlabel('Time (us)')
    plot.ylabel('Voltage')
    plot.title("Local Oscillator")

    plot.figure(8)
    plot.plot(t[200:num_samples//2] / 1e-6, np.real(bb[200:num_samples//2]))
    plot.plot(t[200:num_samples//2] / 1e-6, np.imag(bb[200:num_samples//2]))
    plot.xlabel('Time (us)')
    plot.ylabel('Voltage')
    plot.title("Raw dut IQ")

    plot.figure(9)
    z = np.fft.ifft(bb * np.kaiser(num_samples, 9))
    plot.plot(f[:num_samples // 2] / 1e6, 20 * np.log10(abs(z[:num_samples // 2])))
    plot.grid()
    plot.xlim([0, 10])
    plot.title('Spectrum of base band')

    plot.show()

    rpi.stop_sampler()
    synth.close()
