var s = require('net').Socket();
s.connect({
  port: 1001,
  host: 'eli-rover.local',
});

// ssh -L 1001:radar:1001 johnathan@eli-rover.local
var frequencyCount = 201;
var sampleCount = 16384;

var packetSize = Uint32Array.BYTES_PER_ELEMENT + (frequencyCount * sampleCount * Float32Array.BYTES_PER_ELEMENT * 2);

function getChunk() {
  var data = s.read(packetSize);
  if(data != null) {
    decodeProfile(data);
    setImmediate(getChunk);
  }
}

console.log('packet size', packetSize);
function decodeProfile(data, channels = 2) {
  var timestamp = data.readUInt32LE();

  console.log(timestamp, data.length);

  //return;

  var unpackedData = new Array(channels);

  for(var i = 0; i < channels; i++) {
    unpackedData[i] = [];

    for(var j = 0; j < frequencyCount; j++) {
      unpackedData[i][j] = [];

      for(var k = 0; k < sampleCount; k++) {
        unpackedData[i][j].push(data.readFloatLE((j * k * Float32Array.BYTES_PER_ELEMENT) + Uint32Array.BYTES_PER_ELEMENT));
      }
    }
  }

  console.log(unpackedData[0][0].length);

  if(unpackedData.length === channels &&
    unpackedData.every(channel => channel.length === frequencyCount && channel.every(sample => sample.length === sampleCount))
  ) {
    console.log('valid profile');
  } else {
    console.log('invalid profile');
  }
}

s.on('readable', getChunk);

s.on('error', err => {
  console.log('error', err.toString());
});

s.end();
