import sys
import os
import inspect
import argparse
import glob
import datetime


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
import logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

import h5py as h5
from radar.signal_processing_alan import LowPassFilter,IQDemodulator,BandPassFilter
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal,constants

MHz=1e6
GHz=1e9
us = 1e-6


class RadarProcess(object):

    def __init__(self, sampleCount, frequencyCount, channelCount,
                startFrequency, stepFrequency, intermediateFreq,
                transmitPower, loPower,  maxNumSweeps=100, line_number=1, params=None, **kwargs ):
        self.num_samples = np.int64(sampleCount)
        self.number_of_frequencies = np.int64(frequencyCount)
        self.number_of_channels = np.int64(channelCount)
        self.start_frequency = np.int64(startFrequency)*MHz
        self.step_frequency = np.int64(stepFrequency)*MHz
        self.intermediate_frequency = np.int64(intermediateFreq)*MHz
        self.transmit_power = np.int64(transmitPower)
        self.lo_power = np.int64(loPower)

        self.max_num_sweeps=np.int64(maxNumSweeps) # used pre-allocate allocate arrays.
        # If more samples come in, arrays will need to be extended
        self.actual_num_sweeps=np.int64(0)

        self.line_number=line_number

        self.params = params


        Fs = 122.88e6
        Ts = 1/Fs
        #N = number of frequencies
        #num_channels,N,num_samples = data_i.shape

        # pre-calculate all the params that don't need # of sweeps

        self.raw_filter = BandPassFilter(fc=32e6, bw=2e6, ft=4e6, number_of_taps=32, fs=122.88e6, ts=0e-6,N=self.num_samples)
        #offset = raw_filter.get_settling_time()
        self.if_filter = IQDemodulator(f_lo=self.intermediate_frequency+4e6, fc=4e6, ft=1e6, number_of_taps=64,fs=122.88e6, ts=0e-6, n=self.num_samples)
        self.bb_filter = LowPassFilter(fc=0.5e6, ft=1e6, number_of_taps=64, fs=122.88e6, ts=4e-6, n=self.num_samples)
        t = np.linspace(0,(self.num_samples-1)*Ts, self.num_samples);
        self.bb_filter.get_settling_time();


        offset = 60;
        self.f = np.linspace(self.start_frequency,
            self.step_frequency*(self.number_of_frequencies-1)+self.start_frequency,
            self.number_of_frequencies)
        self.dr = constants.c/(self.start_frequency*(self.num_samples-1))
        self.r_max = constants.c/2/self.step_frequency

        self.M = 1024
        self.r = np.linspace(0,self.r_max,self.M)


        self.window = np.kaiser(self.number_of_frequencies, 6)

        self.allocate_dataset(maxNumSweeps)



    def allocate_dataset(self, num_sweeps):
        self.radar_data_plane = np.zeros(num_sweeps, self.number_of_frequencies)
        self.raw_data = np.zeros((num_sweeps,self.number_of_frequencies), dtype=complex)
        self.proc_data = np.zeros((num_sweeps,self.M), dtype=complex)



    def process_sample(self, radar_file, sweep_num=None, radar_hdf5_file=None):
        """ Todo process from in-memory array instead of hdf5 """
        start = datetime.datetime.now()
        if sweep_num==0:
            sweep=0
        else:
            sweep = sweep_num or self.actual_num_sweeps
        if radar_hdf5_file:
            data_set = radar_hdf5_file
        else:
            data_set = h5.File(radar_file ,'r+')
        data_i = data_set['sweep_data_raw']
        print(f"sweep={sweep} file={radar_file}")
        for i in range(int(self.number_of_frequencies)):

                dut,_ = self.raw_filter(data_i[0,i,:])
                ref,_ = self.raw_filter(data_i[1,i,:])
                ref_iq,lo,a=  self.if_filter (ref)
                dut_iq,lo,a = self.if_filter (dut)
                ref_iq_n = np.exp(-1j*np.angle(ref_iq));
                bb_mixer = np.multiply(dut_iq,np.conj(ref_iq))
                bb,_= self.bb_filter(bb_mixer)
                self.raw_data[sweep,i]=np.mean(bb[100:self.num_samples//2])
        #Range compression
        proc_data = self.proc_data[sweep,:] = np.fft.ifft(self.raw_data[sweep,:]*self.window,self.M)/self.M

        print("proc data single shape {}".format(proc_data.shape))

        print('shape!!')
        print(proc_data.shape)
        
        print('dtype')
        print(proc_data.dtype)

        proc_data_hdf5 = data_set.create_dataset('sweep_data_proc', (self.M, ), dtype=np.complex128)
        proc_data_hdf5.write_direct(self.proc_data[sweep])

        self.actual_num_sweeps+=1
        data_set.close()
        seconds = (datetime.datetime.now() - start).total_seconds()
        logger.warn(f"done processing: {radar_file} sweep_num: {sweep} taken {seconds} seconds")



    def save_plots(self):
        logger.warn("Saving plots")
        for dir in ['iq', 'amp']:
            try:
                os.makedirs(f'output/radar/{dir}')
            except:
                pass

        for sweep in range(self.actual_num_sweeps):
            z = self.proc_data[sweep,:];
            print(f"sweep={sweep}")
            plt.figure(1)
            plt.plot(self.r,abs(z),'b')
            plt.plot(self.r,-abs(z),'b')
            plt.plot(self.r,z.real,self.r,z.imag);
            plt.grid()
            plt.xlabel('Range (m)')
            plt.ylabel('Relative amplitude (v)')
            plt.xlim([0,2])
            plt.ylim([-2.5e-8,2.5e-8])
            plt.title(f'Range profile IQ (envelope for sweep={sweep}')
            plt.savefig(f'output/radar/iq/iq-sweep-{sweep}.png')
            plt.close()

            plt.figure(2)
            plt.plot(self.r,np.abs(z))
            plt.grid()
            plt.xlabel('Range (m)')
            plt.ylabel('Relative power (dB)')
            plt.xlim([0,2])
            plt.ylim([0,2.5e-8])
            plt.title(f'Range profile amplitude for sweep={sweep}')
            plt.savefig(f'output/radar/amp/amp-sweep-{sweep}.png')
            plt.close()

    def save_image(self):
        logger.warn("Saving plots")
        try:
            os.makedirs('img')
        except:
            pass
        num_sweeps = self.actual_num_sweeps

        rdr_real = np.real(self.proc_data[:num_sweeps, :200])
        rdr_real_n = rdr_real/np.max(np.max(np.abs(self.proc_data[:num_sweeps]))) # does this get effected by empty elements???



        #without bg subtraction
        plt.figure(figsize=(16,8))
        plt.imshow(rdr_real_n.transpose(),interpolation ='spline16',cmap='Greys',extent=[0,int(num_sweeps/2),self.r_max-2,-2])
        plt.ylabel("Depth in meters")
        plt.xlabel("Travel distance in meters")
        plt.savefig(f'img/raw_real-line-{self.line_number}.png')

        #bg subtraction

        num_sweeps = self.actual_num_sweeps
        P = int(num_sweeps/2)
        print(num_sweeps)
        rdr_bg_removed = np.zeros((num_sweeps,200))
        for sweep in range(num_sweeps):
                background = np.mean(rdr_real_n,axis=0) # should we exclude empty elements?
                rdr_bg_removed[sweep] = rdr_real_n[sweep] - background;


        print("radar bg shape {}".format(rdr_bg_removed.shape))


        print("radar non-bg  shape {}".format(rdr_real_n.shape))
        filename = f"img/{self.line_num}-bg.hdf5"

        bscan_file= h5py.File(filename, "w")
        bscan_raw = bscan_file.create_dataset('raw_data',(self.M,), dtype='f' )
        for key in self.params:
          bscan_raw.attrs[key] = self.params[key]

        bscan_raw.write_direct(self.proc_data[:num_sweeps, :])

        bscan_bg = bscan_file.create_dataset('bg_subtract_data',(self.M,), dtype='f' )
        for key in self.params:
          bscan_bg.attrs[key] = self.params[key]

        bscan_bg.write_direct(rdr_real_n)

        bscan_file.close()



        plt.figure(figsize=(16,8))
        plt.imshow(rdr_bg_removed.transpose(),cmap='Greys',extent=[0,int(num_sweeps/2),self.r_max-2,-2])
        plt.ylabel("Depth in meters")
        plt.xlabel("Travel distance in meters")
        plt.savefig(f'img/raw_real_bg_removed-line-{self.line_number}.png')

def main():
    parser = argparse.ArgumentParser(description='Run a sweep or process and view saved sweep data.')
    parser.add_argument('-d', '--data-file-path', type=str, default='/Users/ian/projects/radar-mvp-test/process_scan/output/2022-06-29T21:52:24.415070/raw/*.hdf5',
                            help="path to data files to read")
    args = parser.parse_args()
    try:
        import redis
        import msgpack
        r = redis.Redis(host='inspectobot-rover.local', socket_timeout=0.01)
        params = msgpack.unpackb(r.get('radar_parameters'))
        print(params)
    except Exception as e:
        logger.exception("failed to get params from redis, using first file")
        params = {"timestamp":0,"startFrequency":1500,"stepFrequency":20,"frequencyCount":151,"intermediateFreq":32,"transmitPower":-10,"loPower":15,"sampleCount":2048,"channelCount":2,"stepTriggerTimeInMicro":50,"synthWarmupTimeInMicro":5000000,"settlingTimeInMicro":500,"bufferSampleDelay":0}

    def get_sweep(f):
        try:
            filename = f.split('/')[-1].replace('.hdf5', '')
            line, sweep = filename.split('-')
            return int(sweep)
        except:
            print(f"failed to get sortkey {f}")
            return f

    file_list = sorted(glob.glob(args.data_file_path), key=get_sweep )

    radar = RadarProcess(**params, line_number=1)

    for file_name in file_list:
        radar.process_sample(file_name)
    radar.save_image()

if __name__ == '__main__':
    main()
