# http://localhost:9005/scan?patternIndex=0&lineIndex=0&sampleIndex=0
# http://localhost:9005/scan?patternIndex=0&lineIndex=0&sampleIndex=1
# http://localhost:9005/scan?patternIndex=0&lineIndex=0&sampleIndex=2
# http://localhost:9005/proc?patternIndex=0

import time
import sys
import argparse
import ctypes as c
import json
import sys
sys.path.append("..")
import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"

DATA_DIRECTORY = os.path.join(os.path.dirname(__file__), 'output')

import matplotlib
matplotlib.use("Agg")

import datetime
from radar.signal_processing import BandPassFilter, LowPassFilter, IQDemodulator
from radar.client import CreateRadarProfile
from radar.makeimage import RadarProcess
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
logging.basicConfig(level=logging.DEBUG)
import uvloop

from collections import defaultdict,OrderedDict

from concurrent.futures import ThreadPoolExecutor

import s3sync

class RadarService(object):
    trajectoryRunning = False
    sweepCount = 0
    scanFileName = None
    scanFile = None

    radar_address = 'radar'
    rover_address ='localhost'

    http_port = 9005
    http_address='0.0.0.0'

    executor = ThreadPoolExecutor(max_workers=4)

    run_data_coro = None

    tcp_server_coro = None

    radar_process = None
    scan_lock = asyncio.Lock()



    def __init__(self):
        self.reset_data()

    def reset_data(self):
        self.data = defaultdict(list)
        self.cached_data = defaultdict(list)
        self.futures = []

        self.line_index = 0

    def _setup_dirs(self):
        self.start_time = datetime.datetime.now().isoformat()
        for dir in ['out','img', 'raw']:
            p = 'output/' + self.start_time + '/' + dir
            try:
                os.makedirs(p)
            except:
                pass
            abs_path=os.path.abspath(p)
            setattr(self, dir, abs_path)

    async def async_init(self, restart_radar=False):
        self.rover_address = socket.gethostbyname(self.rover_address)

        self.redis = aioredis.from_url(f"redis://{self.rover_address}")
        self.pubsub = self.redis.pubsub()
        self.session = aiohttp.ClientSession()
        await self.refresh_params(restart_radar=restart_radar)
        #self.tcp_server_coro = asyncio.start_server(self.tcp_rover_server, '0.0.0.0', 8888)

        await self.start_line_scan(self.line_index) # setup default line scan

    async def refresh_params(self, restart_radar = False, reset_radar_connection=True):

        params = self.params = msgpack.unpackb(await self.redis.get('radar_parameters'))
        self.RadarProfile = CreateRadarProfile(params['channelCount'], params['sampleCount'], params['frequencyCount'])
        if restart_radar:
            try:
                async with self.session.get(f"http://{self.radar_address}:8081/restart"):
                    pass
                await asyncio.sleep(15)
            except Exception as e:
                logger.exception("Unable to restart radar")

        if reset_radar_connection:
            while True:
                try:
                    self.radar_reader, self.radar_writer = await asyncio.open_connection(self.radar_address, 1001)
                    break

                except Exception as e:
                    if "Connection" in str(e):
                        logger.exception("connection error, restarting")
                        try:
                            async with self.session.get(f"http://{self.radar_address}:8081/restart"):
                                pass
                        except:
                            logger.exception("Unable to restart radar server, sleeping")
                        await asyncio.sleep(15)
                    else:
                        raise e

        self.start_time = datetime.datetime.now().isoformat()

        self._setup_dirs()

        #hacky thing because it doesn't respond on first profile
        # async with self.scan_lock:
        #     self.radar_writer.write(b"\r\n")
        #     await self.radar_writer.drain()



    async def get_data_single(self, point_id=None, line_id=1, timeout=5, write_file=True):
        start_time = datetime.datetime.utcnow()
        profile = self.RadarProfile()
        while True:
            i = 0
            try:
                if i > 3:
                    logger.error("too many attempts to retry radar")
                    raise aiohttp.web.HTTPFailedDependency(text="too many attempts to retry radar")
                async with self.scan_lock:
                    self.radar_writer.write(b"\r\n")
                    await self.radar_writer.drain()
                    data = await asyncio.wait_for(self.radar_reader.readexactly(c.sizeof(self.RadarProfile())), timeout=timeout)
                    break

            except asyncio.TimeoutError:
                logger.exception(f"took more than {timeout} seconds to get data")

            except (EOFError, asyncio.streams.IncompleteReadError, RuntimeError) as e:
                i+=1
                logger.exception("failed to get radar data, broken socket, resetting")
                await asyncio.sleep(0.1)
                logger.exception("Failed to write to radar socket, trying to reconnect")
                await self.refresh_params()

        logger.info("Read data from radar {} seconds".format((datetime.datetime.utcnow() - start_time).total_seconds()))


        self.sweepCount+=1
        c.memmove(c.addressof(profile), data, c.sizeof(profile))
        print("sweep complete", profile.getSamplesAtIndex())
        pose = msgpack.unpackb(await self.redis.get('rover_pose'))
        if point_id or point_id==0:
            pass
        else:
            point_id = self.sweepCount
        filename = f"{line_id}-{point_id}"
        logger.info("total with memory copy {} seconds".format((datetime.datetime.utcnow() - start_time).total_seconds()))


        if write_file:
            loop = asyncio.get_event_loop()
            logger.info(f"Queuing {filename} for processing / writing")

            _run = lambda: self.write_profile(profile, pose, filename, line_id, sweep_num=point_id)
            self.futures.append(loop.run_in_executor(self.executor, _run))

        return profile

    def write_profile(self, profile, pose, file_suffix, line_id, sweep_num=None, proc=True, ):
        filename = self.raw + f"/{file_suffix}.hdf5"

        sweepFile = h5py.File(filename, "w")
        sweepDataSet = sweepFile.create_dataset('sweep_data_raw',
                                                (self.params['channelCount'],
                                                self.params['frequencyCount'],
                                                self.params['sampleCount']), dtype='f')

        for key in self.params:
          sweepDataSet.attrs[key] = self.params[key]

        sweepDataSet.attrs['pose.pos'] = np.array(pose['pos'])
        sweepDataSet.attrs['pose.rot'] = np.array(pose['rot'])
        sweepDataSet.write_direct(profile.asArray())
        sweepFile.close()

        # todo what kind of indexing do we want to do  ? do we need this ?
        self.data[line_id].append(dict(name=file_suffix, **pose))
        self.cached_data[file_suffix].append(dict(profile=profile, pose=pose, file_suffix=file_suffix))

        if proc:
            self.radar_process.process_sample(filename, sweep_num)

        return filename





    # todo refactor radar controller secion to use multiple procs instead of threads

    async def start_line_scan(self, line_number, max_sweeps=100):
        """ Setup line scan radar controller, todo do this in a separate subproc"""
        self.line_index = line_number

        radar_proc_old = self.radar_process
        self.radar_process = RadarProcess(line_number=line_number, maxNumSweeps=max_sweeps, params=self.params, img_path = self.img, **self.params)
        logger.info(f"Created radar service line: {line_number} maxsweeps: {max_sweeps} {self.params}")
        if radar_proc_old is not None:
            # do this to avoid race conditions without using locks (need to do before any awaits)
            logger.info(f"existing radar service line {radar_proc_old.line_number} exists, saving")
            await asyncio.gather(*self.futures)
            if radar_proc_old.actual_num_sweeps > 0:
                logger.info("done waiting, saving image")
                radar_proc_old.save_image()
        #todo might still be a race condition here?
        self.proccess_futures = []


    async def process_scan(self, plots=True):

        print(f"run plots {plots}")
        # Is this a race condition?  Maybe we need to asyncio lock this operation ?
        await asyncio.gather(*self.futures)
        self.futures = []
        loop = asyncio.get_event_loop()
        def _run():
            self.radar_process.save_image()
            if plots:
                self.radar_process.save_plots()
        #todo can't run matplotlib in thread
        #return await loop.run_in_executor(self.executor, _run)
        return _run()

    async def upload_scan_data(self, start_time, sweep_count = 0, line_count = 0):
        logger.info(f"Starting S3 upload for scan {start_time}...");

        s3sync.pathSync(os.path.join(DATA_DIRECTORY, start_time), start_time, sweep_count, line_count)

        syncResult = s3sync.getResult()

        logger.info(f"S3 upload completed for scan {start_time}: {syncResult}")
        s3sync.fileWrite(self.start_time, syncResult) #save the sync progress output to the json folder

    async def rest_process_scan(self, request):
        logger.info(dict(request.rel_url.query))

        patternIndex = request.rel_url.query.get('patternIndex')
        run_plots = patternIndex is not None
        await self.process_scan(plots=run_plots)

        ## INSERT
        loop = asyncio.get_event_loop()
        loop.create_task(self.upload_scan_data(start_time=self.start_time, sweep_count = self.sweepCount, line_count = self.line_index))

        scanResult = dict(success=True, **self.to_dict())

        self.sweepCount = 0
        return aiohttp.web.json_response(scanResult)

    #not used with triggering mode
    # async def get_data(self):
    #     while self.trajectoryRunning:
    #         try:
    #             profile = await self.get_data_single()
    #         except  asyncio.TimeoutError:
    #             print("more than 5 secs for data")
    #             continue
    #         pose = msgpack.unpackb(await self.redis.get('rover_pose'))
    #         print("sweep complete", profile.getSamplesAtIndex())


    ## Not used with triggering mode
    # async def rest_start(self, request):
    #     if not self.trajectoryRunning:
    #         try:
    #             await self.run_data_coro
    #         except:
    #             pass
    #         self.trajectoryRunning = True
    #         self.run_data_coro = asyncio.ensure_future(self.get_data())
    #         await asyncio.sleep(1)
    #         return aiohttp.web.json_response(self.to_dict())
    #
    #     else:
    #         return aiohttp.web.json_response({"error": "Currently running, please cancel"})

    ## Not used with triggering mode
    # async def rest_cancel(self, request):
    #
    #     self.trajectoryRunning = False
    #     try:
    #         await self.run_data_coro
    #     except:
    #         pass
    #     return aiohttp.web.json_response(self.to_dict())


    async def rest_trigger_scan(self, request):
        logger.error(dict(request.rel_url.query))
        params = dict(request.rel_url.query)
        line_number = int(params.get('lineIndex', 0))
        #line_number = int(params.get('patternIndex', 0))
        sample_index = int(params.get('sampleIndex', 0))
        if line_number and int(line_number) != self.line_index:
            await self.start_line_scan(line_number)
        await self.get_data_single(point_id=sample_index, line_id=line_number)
        return aiohttp.web.json_response(self.to_dict())


    async def reset_process(self, request):

        """ Params:   reset_radar: Bollean (if sent as a parameter, will recconect to radar default False)
                      reset_data: Boolean """
        logger.error(dict(request.rel_url.query))
        params = dict(request.rel_url.query)

        reset_radar = False
        if params.get('reset_radar'):
            reset_radar = True
        self.reset_data()
        await self.refresh_params(reset_radar_connection=reset_radar)
        pass


    async def rest_reset(self, request):
        await self.reset_process()
        return aiohttp.web.json_response(self.to_dict())


    def to_dict(self):
        return {"running": self.trajectoryRunning,
        "scanFile": self.scanFile, "sweepCount": self.sweepCount,"output_dir":os.path.abspath(self.raw),
        "start_time": self.start_time,'image_dir':self.img, 'current_data_path':f'/outputjson/{self.start_time}/',
         "data": self.data}


    async def rest_status(self, request):
        data = self.to_dict()
        return aiohttp.web.json_response(data)

    async def rest_start_scan_post(self, request):
        self._setup_dirs()
        await self.start_line_scan(0)

        data = await request.json()
        with open(f'{self.img}/../data.json', 'w') as f:
            f.write(json.dumps(data))
            #sync data here
        return aiohttp.web.json_response(self.to_dict())

    # async def tcp_rover_server(self, reader, writer):
    #     unpacker = msgpack.Unpacker(use_list=False, raw=False)
    #
    #     while True:
    #         data = await reader.read(1024)
    #         if not data:
    #             break
    #         unpacker.feed(data)
    #         for unpacked in unpacker:
    #             print(unpacked)
    #             if unpacked.get('msg_type') == "scan":
    #               try:
    #                 params = {'point_id': unpacked.get('point_id'), 'line_id': unpacked.get('line_id') }
    #                 await self.get_data_single(timeout=5, write_file=True, **params)
    #                 writer.write(msgpack.packb(dict(success=True, msg_type="scan_done", **params)))
    #               except Exception as e:
    #                 logger.exception("failed to get point")
    #                 writer.write(msgpack.packb(dict(success=True, msg_type="scan_failed", **params)))
    #               await writer.drain()



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
            #aiohttp.web.get('/start', self.rest_start),
            #aiohttp.web.get('/cancel', self.rest_cancel),
            aiohttp.web.get('/scan', self.rest_trigger_scan),
            aiohttp.web.get('/proc', self.rest_process_scan),
            aiohttp.web.post('/start_scan', self.rest_start_scan_post)
            ])

        self.http_runner = aiohttp.web.AppRunner(self.http_app,access_log_format='%a %t "%r" %s %b "%{Referer}i" "%{User-Agent}i" time: %Tf')

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
