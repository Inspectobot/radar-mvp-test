import ctypes as c
import numpy as np

class ProfileType():
  DUT = 0
  REF = 1

def CreateRadarProfile(num_samples = 101, number_of_frequencies = 101):
  class RadarProfile(c.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
      ('timestamp', c.c_uint32),
      ('type', c.c_int, 4),
      ('data', c.c_float * (num_samples * number_of_frequencies))
    ]

    def getSamplesAtIndex(self, index = 0):
      d = np.array(self.data)
      d = d.reshape(number_of_frequencies, num_samples)
      return d[index]

  return RadarProfile

if __name__ == '__main__':
  import socket
  
  RadarProfile = CreateRadarProfile()
  packet_size = c.sizeof(RadarProfile())

  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server_address = ('radar', 1001)

  sock.connect(server_address)

  while True:
    data = sock.recv(packet_size, socket.MSG_WAITALL)

    profile = RadarProfile()
    c.memmove(c.addressof(profile), data, c.sizeof(profile))
   
    print(profile.timestamp)
    print(profile.type)
    print(len(profile.data))
