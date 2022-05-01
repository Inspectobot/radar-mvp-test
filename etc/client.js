var s = require('net').Socket();
s.connect({
  port: 1001, 
  host: 'radar',
});

s.on('data', function(d){
  console.log(d);
});

s.on('error', err => {
  console.log('error', err.toString());
});

s.end();
