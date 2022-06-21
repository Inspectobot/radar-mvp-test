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

import redis
import msgpack

import multiprocessing

from redis import Redis

import asyncio

from arq import create_pool
from arq.connections import RedisSettings

from scan_worker_job import process_sweep

async def main(args):
    scanFile = h5py.File(args.data_file_path, "r")

    rq = await create_pool(RedisSettings())

    r = redis.StrictRedis(host='localhost', port=6379, db=0)

    sweepDataSets = scanFile.keys()

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
        profile.saveCacheKey(r, args.data_file_path, key)

        await rq.enqueue_job('process_sweep', args.data_file_path, key,
            num_samples,
            number_of_frequencies,
            number_of_channels,
            start_frequency,
            step_frequency,
            intermediate_frequency,
            transmit_power,
            lo_power,
            posePos,
            poseRot
        )

    scanFile.close()

class WorkerSettings:
    functions = [process_sweep]

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process saved raw scan.')
    parser.add_argument('-d', '--data-file-path', type=str, help="Path to saved raw data scan hdf5 file")
    parser.add_argument('-j', '--number-of-processes', type=int, default=multiprocessing.cpu_count(), help="Number of worker processes to spawn")

    args = parser.parse_args()
    asyncio.run(main(args))
