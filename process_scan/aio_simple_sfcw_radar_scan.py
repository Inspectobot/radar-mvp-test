import time
import sys
import argparse
import ctypes as c
import sys
sys.path.append("..")
import os

import datetime
from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile
from concurrent.futures import ProcessPoolExecutor

import matplotlib.pyplot as plot
import numpy as np
from scipy import signal
from redis import asyncio as aioredis
import asyncio
import asyncio.streams
import socket
import h5py
import aiohttp.web
import redis
import msgpack
import aiohttp
import logging
logger = logging.getLogger(__name__)
import uvloop

class RadarService(object):
    trajectoryRunning = False
    sweepCount = 0
    scanFileName = None
    scanFile = None

    rover_name ='inspectobot-rover.local'

    http_port = 9005
    http_address='0.0.0.0'

    executor = ProcessPoolExecutor(max_workers=2)

    run_data_coro = None

    tcp_server_coro = None


    async def async_init(self, restart_radar=False):
        self.rover_address = socket.gethostbyname(self.rover_name)

        self.redis = aioredis.from_url(f"redis://{self.rover_address}")
        self.pubsub = self.redis.pubsub()
        self.session = aiohttp.ClientSession()
        await self.refresh_params(restart_radar=restart_radar)
        self.tcp_server_coro = asyncio.start_server(self.tcp_rover_server, '0.0.0.0', 8888)

    async def refresh_params(self, restart_radar = False):

        params = self.params = msgpack.unpackb(await self.redis.get('radar_parameters'))
        self.RadarProfile = CreateRadarProfile(params['channelCount'], params['sampleCount'], params['frequencyCount'])
        if restart_radar:
            async with self.session.get(f"http://{self.rover_address}:8081/restart"):
                pass
            await asyncio.sleep(15)

        while True:
            try:
                self.radar_reader, self.radar_writer = await asyncio.open_connection(self.rover_address, 1001)
                break

            except Exception as e:
                if "Connection" in str(e):
                    logger.exception("connection error, restarting")
                    try:
                        async with self.session.get(f"http://{self.rover_address}:8081/restart"):
                            pass
                    except:
                        logger.exception("Unable to restart radar server, sleeping")
                    await asyncio.sleep(15)
                else:
                    raise e

        self.start_time = datetime.datetime.now().isoformat()

        for dir in ['out','img', 'raw']:
            p = self.start_time + '/' + dir
            os.makedirs(p)
            setattr(self, dir, p)

        #hacky thing because it doesn't respond on first profile
        self.radar_writer.write(b"send")
        await self.radar_writer.drain()




    async def get_data_single(self, point_id=None, line_id=1, timeout=5, write_file=True):
        profile = self.RadarProfile()
        while True:
            i = 0
            try:
                if i > 4:
                    logger.error("too many attempts to retry radar")
                    return
                self.radar_writer.write(b"send")
                await self.radar_writer.drain()
                data = await asyncio.wait_for(self.radar_reader.readexactly(c.sizeof(self.RadarProfile())), timeout=timeout)
                break

            except asyncio.TimeoutError:
                logger.exception(f"took more than {timeout} seconds to get data")
                return

            except (EOFError, asyncio.streams.IncompleteReadError) as e:
                i+=1
                logger.exception("failed to get radar data, broken socket, resetting")
                await asyncio.sleep(0.1)
                logger.exception("Failed to write to radar socket, trying to reconnect")
                await self.refresh_params()


        self.sweepCount+=1
        c.memmove(c.addressof(profile), data, c.sizeof(profile))
        print("sweep complete", profile.getSamplesAtIndex())
        pose = msgpack.unpackb(await self.redis.get('rover_pose'))
        if write_file:
            self.write_profile(profile, pose, f"{line_id}-{point_id or self.sweepCount}")
        return profile

    def write_profile(self, profile, pose, file_suffix):
        sweepFile = h5py.File(self.raw + f"/{file_suffix}.hdf5", "w")
        sweepDataSet = sweepFile.create_dataset('sweep_data_raw',
                                                (self.params['channelCount'],
                                                self.params['frequencyCount'],
                                                self.params['sampleCount']), dtype='f')

        for key in self.params:
          sweepDataSet.attrs[key] = self.params[key]

        sweepDataSet.attrs['pose.pos'] = np.array(pose['pos'])
        sweepDataSet.attrs['pose.rot'] = np.array(pose['rot'])
        sweepDataSet.write_direct(profile.asArray())



    async def get_data(self):
        while self.trajectoryRunning:
            try:
                profile = await self.get_data_single()
            except  asyncio.TimeoutError:
                print("more than 5 secs for data")
                continue
            pose = msgpack.unpackb(await self.redis.get('rover_pose'))
            print("sweep complete", profile.getSamplesAtIndex())



    async def rest_start(self, request):
        if not self.trajectoryRunning:
            try:
                await self.run_data_coro
            except:
                pass
            self.trajectoryRunning = True
            self.run_data_coro = asyncio.ensure_future(self.get_data())
            await asyncio.sleep(1)
            return aiohttp.web.json_response(self.to_dict())

        else:
            return aiohttp.web.json_response({"error": "Currently running, please cancel"})

    async def rest_cancel(self, request):

        self.trajectoryRunning = False
        try:
            await self.run_data_coro
        except:
            pass
        return aiohttp.web.json_response(self.to_dict())


    async def rest_trigger_scan(self, request):

        await self.get_data_single()
        return aiohttp.web.json_response(self.to_dict())


    def to_dict(self):
        return {"running": self.trajectoryRunning,
        "scanFile": self.scanFile, "sweepCount": self.sweepCount}


    async def rest_status(self, request):
        data = self.to_dict()
        return aiohttp.web.json_response(data)

    async def tcp_rover_server(self, reader, writer):
        unpacker = msgpack.Unpacker(use_list=False, raw=False)

        while True:
            data = await reader.read(1024)
            if not data:
                break
            unpacker.feed(data)
            for unpacked in unpacker:
                print(unpacked)
                if unpacked.get('msg_type') == "scan":
                  try:
                    params = {'point_id': unpacked.get('point_id'), 'line_id': unpacked.get('line_id') }
                    await self.get_data_single(timeout=5, write_file=True, **params)
                    writer.write(msgpack.packb(dict(success=True, msg_type="scan_done", **params)))
                  except Exception as e:
                    logger.exception("failed to get point")
                    writer.write(msgpack.packb(dict(success=True, msg_type="scan_failed", **params)))
                  await writer.drain()


    async def http_server(self):
        while True:
            try:
                logger.info("running setup")
                await self.async_init()
                break
            except Exception as e:
                logger.exception("unable to run setup")
                await asyncio.sleep(1)


        self.http_app = aiohttp.web.Application()
        self.http_app.add_routes([
            aiohttp.web.get('/status', self.rest_status),
            aiohttp.web.get('/start', self.rest_start),
            aiohttp.web.get('/cancel', self.rest_cancel),
            aiohttp.web.get('/scan', self.rest_trigger_scan)
            ])

        self.http_runner = aiohttp.web.AppRunner(self.http_app)

        await self.http_runner.setup()
        print("ran http setup")
        try:
            print("starting site...")
            self.http_site = aiohttp.web.TCPSite(self.http_runner,
                    self.http_address, self.http_port, reuse_address=True)
            await self.http_site.start()

            while True:
                await asyncio.sleep(10)
        finally:
            await self.http_runner.cleanup()

    def main(self):
        uvloop.install()
        loop = asyncio.get_event_loop()
        # Setup exception handler

        def exception_handler(loop, context):
            logger.error("unhandled exception in %s: %s",
                context['future'] if 'future' in context else '<none>',
                context['message'],
                exc_info=context['exception']
                    if 'exception' in context else False)
            loop.stop()
        loop.set_exception_handler(exception_handler)
        loop.run_until_complete(self.http_server())


async def main_aio():
    radar = RadarService()
    await radar.async_init()
    radar.trajectoryRunning = True
    await radar.get_data()





if __name__ == '__main__':

    radar = RadarService()
    radar.main()

    # loop = asyncio.get_event_loop()
    # # Setup exception handler
    #
    # def exception_handler(loop, context):
    #     logger.error("unhandled exception in %s: %s",
    #         context['future'] if 'future' in context else '<none>',
    #         context['message'],
    #         exc_info=context['exception']
    #             if 'exception' in context else False)
    #     loop.stop()
    # loop.set_exception_handler(exception_handler)
    # loop.run_until_complete(main_aio())

async def main():
    r = aioredis.from_url("redis://inspectobot-rover.local")
    p = r.pubsub()

    params = msgpack.unpackb(await r.get('radar_parameters'))
    RadarProfile = CreateRadarProfile(params['channelCount'], params['sampleCount'], params['frequencyCount'])

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('inspectobot-rover.local', 1001)

    sock.connect(server_address)

    trajectoryRunning = False
    sweepCount = 0
    scanFileName = None
    scanFile = None
    def roverControllerEvent(message):
      global trajectoryRunning, sweepCount, scanFileName, scanFile

      msg = message['data'].decode('utf-8')
      print("Received event: ", msg)

      if(msg == "runActiveTrajectory"):
          trajectoryRunning = True
          sweepCount = 0

          scanFileName = "scan-" + str(int(time.time())) + ".hdf5"
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

        pose = msgpack.unpackb(r.get('rover_pose'))

        sweepDataSet.attrs['pose.pos'] = np.array(pose['pos'])
        sweepDataSet.attrs['pose.rot'] = np.array(pose['rot'])

        sweepDataSet.write_direct(profile.asArray())
        sweepCount += 1

        message = p.get_message()

    p.subscribe(**{'rover_controller': roverControllerEvent})
    thread = p.run_in_thread(sleep_time=0.001)
