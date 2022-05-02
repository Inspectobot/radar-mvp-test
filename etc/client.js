var s = require('net').Socket();
s.connect({
  port: 1001, 
  host: 'radar',
});

var frequencyCount = 101;
var sampleCount = 101;

var packetSize = 8 + (frequencyCount * sampleCount * 2);

function getChunk() {
  var data = s.read(packetSize);
  if(data != null) {
    decodeProfile(data);
    setImmediate(getChunk);
  }
}

console.log('packet size', packetSize);
function decodeProfile(data) {
  var timestamp = data.readUInt32LE();
  var type      = data.readUInt8(4);

  console.log(type);

  var profileData = new Array(frequencyCount);
  for(var i = 0; i < frequencyCount; i++) {
    profileData[i] = [];
    
    for(var j = 0; j < sampleCount; j++) {
      profileData[i].push(data.readUint16LE((i * j * 2) + 8));
    }
  }

  if(profileData.length === frequencyCount && profileData.every(samples => samples.length === sampleCount)) {
    console.log(timestamp, 'valid profile');
  } else {
    console.log(timestamp, 'invalid profile');
  }
 
}

s.on('readable', getChunk);

s.on('error', err => {
  console.log('error', err.toString());
});

s.end();
