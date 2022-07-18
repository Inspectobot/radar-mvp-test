var net     = require('net'),
    express = require('express'),
    redis   = require('redis'),
    msgpack = require('msgpack');


async function main() {
	const redisClient = redis.createClient({
		socket: { host: 'localhost' }
	});

	await redisClient.connect();

	var radarParams = msgpack.unpack(await redisClient.get(redis.commandOptions({ returnBuffers: true }), 'radar_parameters'));
  	var packetSize = Uint32Array.BYTES_PER_ELEMENT + (radarParams.frequencyCount * radarParams.sampleCount * Float32Array.BYTES_PER_ELEMENT * 2);

	var app = express();

	const radarDataPort = 1001;
	const radarProcessManagerPort = 8081;

	app.get('/restart', (req, res) => {
		console.log('got restart request');
		res.status(200).end();
	});

	app.listen(radarProcessManagerPort, () => {
		console.log(`Radar process manager listening on port ${radarProcessManagerPort}`)
	});

	const server = new net.Server();

	server.listen(radarDataPort, function() {
		console.log(`Radar data server listening for connection requests on socket localhost:${radarDataPort}`);
	});

	server.on('connection', function(socket) {
		console.log('got connection', socket);
		
		var id;
		socket.on('close', () => {
			clearInterval(id);
		});

		socket.on('error', () => {
			clearInterval(id);
		});

		socket.on('timeout', () => {
			clearInterval(id);
		});

		socket.setNoDelay(true);
		id = setInterval(() => {
			if(socket && socket.readyState === 'open') {
				console.log('write dummy radar data');
				var data = new Buffer(packetSize).fill(1);
				try {
					socket.write(data);
				} catch(e) {
					console.log('Error writing data', e.toString());
					clearInterval(id);
				}
			} else {
				clearInterval(id);
			}
		}, 100);

	});

	
}

main();
