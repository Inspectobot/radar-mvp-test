const pm2 = require('pm2');
const express = require('express');
const app = express();


pm2.connect(function(err) {
  if (err) {
    console.error(err)
    process.exit(2)
  }

  app.get('/', (req, res) => {
    pm2.list((err, list) => {
      if(err) return res.status(500).end(err.toString());

      res.status(200).json(list).end();
    });
  });

  app.get('/restart', (req, res) => {
    pm2.restart('radar_redis_driver', (err, proc) => {
      if(err) return res.status(500).end(err.toString());

      res.status(200).json(proc).end();
    });
  });

  app.listen(8081);
});
