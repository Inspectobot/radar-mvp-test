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
from rq import Queue

from scan_worker_job import process_sweep

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process saved raw scan.')
    parser.add_argument('-d', '--data-file-path', type=str, help="Path to saved raw data scan hdf5 file")
    parser.add_argument('-j', '--number-of-processes', type=int, default=multiprocessing.cpu_count(), help="Number of worker processes to spawn")

    args = parser.parse_args()

    scanFile = h5py.File(args.data_file_path, "r")

    q = Queue(connection=Redis())

    sweepDataSets = scanFile.keys()

    for key in sweepDataSets:
        job = q.enqueue(process_sweep, args.data_file_path, key)
