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
    r = redis.Redis(host='localhost')
    p = r.pubsub()

    params = msgpack.unpackb(r.get('radar_parameters'))
    RadarProfile = CreateRadarProfile(params['channelCount'], params['sampleCount'], params['frequencyCount'])

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('radar', 1001)

    sock.connect(server_address)

    trajectoryRunning = False
    sweepCount = 0
    scanFileName = None
    scanFile = None

    print("Waiting for trajectory to start..")

    def roverControllerEvent(message):
      global trajectoryRunning, sweepCount, scanFileName, scanFile

      msg = message['data'].decode('utf-8')
      print("Received event: ", msg)

      if(msg == "runActiveTrajectory"):
          trajectoryRunning = True
          sweepCount = 0

          scanFileName = "sweep-" + str(int(time.time())) + ".hdf5"
          scanFile = h5py.File(scanFileName, "w")

          print("Started scan: " + scanFileName)
      elif(msg == "trajectoryComplete"):
          trajectoryRunning = False
          scanFile.close()
          scanFile = None

          print("Scan complete: " + scanFileName)
          print("Total sweeps: " + str(sweepCount))
      else:
          trajectoryRunning = False

      while trajectoryRunning:
        profile = RadarProfile()

        data = sock.recv(c.sizeof(RadarProfile()), socket.MSG_WAITALL)
        c.memmove(c.addressof(profile), data, c.sizeof(profile))

        sweepDataSet = scanFile.create_dataset('sweep-' + str(sweepCount), (params['channelCount'], params['frequencyCount'], params['sampleCount']), dtype='f')
        for key in params:
          sweepDataSet.attrs[key] = params[key]

        sweepDataSet.attrs['timestamp'] = profile.timestamp
        pose = msgpack.unpackb(r.get('rover_pose'))

        sweepDataSet.attrs['pose.pos'] = np.array(pose['pos'])
        sweepDataSet.attrs['pose.rot'] = np.array(pose['rot'])

        r.publish('radar_sample_point', msgpack.packb({
          'timestamp': profile.timestamp,
          'pose': pose
        }));

        sweepDataSet.write_direct(profile.asArray())
        sweepCount += 1

        message = p.get_message()

    p.subscribe(**{'rover_controller': roverControllerEvent})
    thread = p.run_in_thread(sleep_time=0.001)
