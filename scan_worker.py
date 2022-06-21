#!/usr/bin/env python
import sys
from rq import Connection, Worker

from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile

import numpy as np
from scipy import signal

import h5py

with Connection():
    qs = sys.argv[1:] or ['default']

    w = Worker(qs)
    w.work()
