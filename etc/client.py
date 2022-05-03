import socket
import ctypes as c
from enum import Enum

num_samples = 101
number_of_frequencies = 101

packet_size = 8 + (number_of_frequencies * num_samples * 2);

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('radar', 1001)

sock.connect(server_address)

class ProfileType(Enum):
  DUT = 0
  REF = 0

class RadarProfile(c.LittleEndianStructure):
  _pack_ = 1
  _fields_ = [
    ('timestamp', c.c_uint32),
    ('type', c.c_byte, 4),
    ('data', c.c_uint16 * (num_samples * number_of_frequencies))
  ]

while True:
  data = sock.recv(packet_size, socket.MSG_WAITALL)

  profile = RadarProfile()
  c.memmove(c.addressof(profile), data, c.sizeof(profile))
 
  print(profile.timestamp)
  print(profile.type)
  print(len(profile.data))
