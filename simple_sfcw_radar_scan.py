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

if __name__ == '__main__':
    r = redis.Redis(host="inspectobot-rover.local")
    p = r.pubsub()

    params = msgpack.unpackb(r.get('radar_parameters'))
    RadarProfile = CreateRadarProfile(params['channelCount'], params['sampleCount'], params['frequencyCount'])

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('eli-rover.local', 1001)

    sock.connect(server_address)

    trajectoryRunning = False
    scanFileName = None
    scanFile = None
    def roverControllerEvent(message):
      if(message == "runActiveTrajectory"):
          trajectoryRunning = True

          scanFileName = "sweep-" + str(int(time.time())) + ".hdf5"
          scanFile = h5py.File(scanFileName, "w")

          print("Started scan: " + scanFileName)
      elif(message == "trajectoryComplete"):
          trajectoryRunning = False
          scanFile.close()
          scanFile = None

          print("Scan complete: " + scanFileName)
      else:
          trajectoryRunning = False

      while trajectoryRunning:
        profile = RadarProfile()

        data = sock.recv(c.sizeof(RadarProfile()), socket.MSG_WAITALL)
        c.memmove(c.addressof(profile), data, c.sizeof(profile))

        sweepDataSet = scanFile.create_dataset('scan_data_raw', (params['channelCount'], params['frequencyCount'], params['sampleCount']), dtype='f')
        sweepDataSet.attrs = params

        pose = msgpack.unpackb(r.get("rover_pose"))

        sweepDataSet.attrs['pose.pos'] = np.array(pose['pos'])
        sweepDataSet.attrs['pose.rot'] = np.array(pose['rot'])

        sweepDataSet.write_direct(profile.asArray())

    p.subscribe(**{'rover_controller'}: roverControllerEvent})
