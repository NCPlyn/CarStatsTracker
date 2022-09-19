const express = require('express');
const socketio = require('socket.io');
const http = require('http');
const path = require('path');
const fs = require("fs");

const app = express();
const server = http.createServer(app);
const io = socketio(server);

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());

app.post('/uplink', function (req, res) {
	var jsonData = JSON.stringify(req.body);
	var jsonParsed = JSON.parse(jsonData);
	var buf = Buffer.from(jsonParsed.uplink_message.frm_payload, 'base64');
	var str = buf.toString('utf-8');
	console.log(str);
  io.emit("loraDecoded", str); //tohle má v sobě routeid, nutno opravit před posláním nebo nějak
  res.status(200).end()
});

io.on('connection', (socket) => {
  socket.on('getRoutes', function() { //signal to send routes from json file
    let data = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    data.forEach(function(obj) {
      socket.emit('loadRoutes', obj.idk);
    });
  });
  socket.on('getPoints', routeId => { //signal to send post from specific board, load from json
    let routePath = "routes/" + routeId + ".json";
    if (fs.existsSync(routePath)) {
      let data = JSON.parse(fs.readFileSync(routePath, 'utf8'));
      data.forEach(function(obj) { //moving;lat;lng;speed;heading;epoch
        socket.emit('loadPoints', obj.idk);
      });
    } else {
      //error: not found
    }
  });
});

const PORT = process.env.PORT || 80;
server.listen(PORT, function() {
  console.log(`Server listening on port ${PORT}`);
});
