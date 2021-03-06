import time
import sys
import argparse
import ctypes as c

from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import matplotlib.pyplot as plot
import numpy as np
from scipy import signal
import requests

import redis
import msgpack
from redis import asyncio as aioredis

import asyncio
import uvloop

import socket
import h5py

import csv

def main():
  parser = argparse.ArgumentParser(description='Run a sweep or process and view saved sweep data.')
  parser.add_argument('-d', '--data-file-path', type=str, help="Read a saved sweep hdf5 file")
  parser.add_argument('-o', '--data-file-output-path', type=str, help="Data file output filename")

  parser.add_argument('-r', '--read-from-redis', action='store_true', default=True, help="Read radar parameters from redis")
  
  parser.add_argument('-H', '--hostname', type=str, default='inspectobot-rover.local', help="Hostname or IP address of radar data server")
  parser.add_argument('-p', '--port', type=int, default=1001, help="Listening TCP port of radar data server")
  
  parser.add_argument('-s', '--num_samples', type=int, default=2048, help="Number of samples")
  parser.add_argument('-n', '--number_of_frequencies', type=int, default=151, help="Number of samples")
  parser.add_argument('-c', '--number_of_channels', type=int, default=2, help="Number of channels")
  parser.add_argument('-f', '--start_frequency', type=float, default=1500.0, help="Start frequency")
  parser.add_argument('-t', '--step_frequency', type=float, default=20.0, help="Step frequency")
  parser.add_argument('-if', '--intermediate_frequency', type=float, default=32.0, help="Intermediate (IF) frequency")
  parser.add_argument('-tx', '--transmit_power', type=float, default=0.0, help="Transmit power in dB")
  parser.add_argument('-lo', '--lo_power', type=float, default=15.0, help="Local oscillator power in dB")

  args = parser.parse_args()
        
  uvloop.install()
  loop = asyncio.get_event_loop()

  def exception_handler(loop, context):
      logger.error("unhandled exception in %s: %s",
          context['future'] if 'future' in context else '<none>',
          context['message'],
          exc_info=context['exception']
              if 'exception' in context else False)
      loop.stop()
  
  loop.set_exception_handler(exception_handler)
  loop.run_until_complete(run(args))

async def run(args):
  
  redis = aioredis.from_url(f"redis://{args.hostname}")

  if args.data_file_path is None:

    if args.read_from_redis is True:
      params = msgpack.unpackb(await redis.get('radar_parameters'))
      num_samples = params['sampleCount']
      number_of_frequencies = params['frequencyCount']
      number_of_channels = params['channelCount']
      start_frequency = params['startFrequency']
      step_frequency = params['stepFrequency']
      intermediate_frequency = params['intermediateFreq']
      transmit_power = params['transmitPower']
      lo_power = params['loPower']

    else:
      num_samples = args.num_samples
      number_of_frequencies = args.number_of_frequencies
      number_of_channels = args.number_of_channels
      start_frequency = args.start_frequency
      step_frequency = args.step_frequency
      intermediate_frequency = args.intermediate_frequency
      transmit_power = args.transmit_power
      lo_power = args.lo_power


    print(number_of_channels, num_samples, number_of_frequencies)
    output_path = args.data_file_output_path

    RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)
    if output_path:
        print("path " + output_path)
        filename = output_path + ".hdf5"
    else:
        filename = "sweep-" + str(int(time.time())) + ".hdf5"
    
    print("Output data will be saved to: " + filename)

    sweepFile = h5py.File(filename, "w")
    sweepDataSet = sweepFile.create_dataset('sweep_data_raw', (number_of_channels, number_of_frequencies, num_samples), dtype='f')

    sweepDataSet.attrs['num_samples'] = num_samples
    sweepDataSet.attrs['number_of_frequencies'] = number_of_frequencies
    sweepDataSet.attrs['number_of_channels'] = number_of_channels
    sweepDataSet.attrs['start_frequency'] = start_frequency
    sweepDataSet.attrs['step_frequency'] = step_frequency
    sweepDataSet.attrs['intermediate_frequency'] = intermediate_frequency
    sweepDataSet.attrs['transmit_power'] = transmit_power
    sweepDataSet.attrs['lo_power'] = lo_power
    
    print(args)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #sock.settimeout(10)
    server_address = (args.hostname, args.port)
    try:
        sock.connect(server_address)
    except Exception as e:
        print("socket error restarting radar" + str(e))
        exit()

    profile = RadarProfile()
    sock.send("\r\n".encode())
    data = sock.recv(c.sizeof(RadarProfile()), socket.MSG_WAITALL)
    c.memmove(c.addressof(profile), data, c.sizeof(profile))

    sweepDataSet.write_direct(profile.asArray())
    sweepFile.close()
    if output_path:
        print("not rendering plot")
        return

  else:
    sweepFile = h5py.File(args.data_file_path, "r")

    sweepDataSet = sweepFile['sweep_data_raw']

    num_samples = sweepDataSet.attrs['num_samples']
    number_of_frequencies = sweepDataSet.attrs['number_of_frequencies']
    number_of_channels = sweepDataSet.attrs['number_of_channels']
    start_frequency = sweepDataSet.attrs['start_frequency']
    step_frequency = sweepDataSet.attrs['step_frequency']
    intermediate_frequency = sweepDataSet.attrs['intermediate_frequency']
    transmit_power = sweepDataSet.attrs['transmit_power']
    lo_power = sweepDataSet.attrs['lo_power']

    RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)
    profile = RadarProfile()

    profile.setArrayFromDataset(sweepDataSet)
    sweepFile.close()

  if_filter = IQDemodulator(f_lo=36e6, fc=4e6, ft=1e6, number_of_taps=256, fs=122.88e6, t_sample=1e-6, n=num_samples)
  bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=256, fs=122.88e6, ts=1e-6, N=num_samples)

  N = number_of_frequencies
  x = np.zeros(N, dtype=complex)
  f = np.linspace(start_frequency, start_frequency + ((number_of_frequencies - 1) * step_frequency), number_of_frequencies)

  for i in range(N):
    dut = profile.getSamplesAtIndex(1, i)
    ref = profile.getSamplesAtIndex(0, i)

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
  plot.xlim([0,2])

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

  plot.clf()


if __name__ == '__main__':
    main()
