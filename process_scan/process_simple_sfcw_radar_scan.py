import sys
sys.path.append("..")
import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"

import time
import sys
import argparse
import ctypes as c

from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import matplotlib.pyplot as plot
import numpy as np
from scipy import signal

import socket
import h5py


import multiprocessing


import asyncio


from scan_worker_job import process_sweep
from concurrent.futures import ProcessPoolExecutor


async def main(args):
    executor = ProcessPoolExecutor(max_workers=4)
    scanFile = h5py.File(args.data_file_path, "r")

    sweepDataSets = scanFile.keys()

    futures = []

    loop = asyncio.get_event_loop()

    for key in sweepDataSets:
        print("Queuing key: " + key)
        sweepDataSet = scanFile[key]

        num_samples = sweepDataSet.attrs['sampleCount']
        number_of_frequencies = sweepDataSet.attrs['frequencyCount']
        number_of_channels = sweepDataSet.attrs['channelCount']
        start_frequency = sweepDataSet.attrs['startFrequency']
        step_frequency = sweepDataSet.attrs['stepFrequency']
        intermediate_frequency = sweepDataSet.attrs['intermediateFreq']
        transmit_power = sweepDataSet.attrs['transmitPower']
        lo_power = sweepDataSet.attrs['loPower']
        posePos = sweepDataSet.attrs['pose.pos']
        poseRot = sweepDataSet.attrs['pose.rot']

        RadarProfile = CreateRadarProfile(number_of_channels, num_samples, number_of_frequencies)
        profile = RadarProfile()

        profile.setArrayFromDataset(sweepDataSet)
        #profile.saveCacheKey(r, args.data_file_path, key)

        future = loop.run_in_executor(executor, process_sweep, None,
            args.data_file_path, key, num_samples, number_of_frequencies,
            number_of_channels, start_frequency, step_frequency,
            intermediate_frequency, transmit_power, lo_power, posePos, poseRot,
            profile.asArray())

        futures.append(future)

    scanFile.close()

    print(await asyncio.gather(*futures))
    print("done!!!!!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process saved raw scan.')
    parser.add_argument('-d', '--data-file-path', type=str, help="Path to saved raw data scan hdf5 file")
    parser.add_argument('-j', '--number-of-processes', type=int, default=multiprocessing.cpu_count(), help="Number of worker processes to spawn")

    args = parser.parse_args()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main(args))
