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
	const dataArray = str.split(";");
	var out = {rideId:dataArray[0],moving:dataArray[1],lat:dataArray[2],lng:dataArray[3],speed:dataArray[4],epoch:dataArray[5]};
  io.emit("loraDecoded", out.moving, out.lat, out.lng, out.speed, out.epoch);
	saveUplink(out);
  res.status(200).end()
});

function search(array, insert, cb) {
    var left = -1,
        right = array.length,
        actual;

    while (left !== right && left + 1 !== right) {
        actual = Math.floor((left + right) / 2);
        if (cb(array[actual]) < cb(insert)) {
            left = actual;
            continue;
        }
        if (cb(array[actual]) > cb(insert)) {
            right = actual;
        }
    }
    return left;
}

function saveUplink(data) {
	var currentFileContent;
	if (fs.existsSync("routes/"+String(data.rideId)+".json")) { //also check routes.json???
		currentFileContent = JSON.parse(fs.readFileSync("routes/"+String(data.rideId)+".json", 'utf8'));
		let insert = {"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":data.epoch};
		var index = search(currentFileContent, insert, function (a) { return a.epoch; });
		currentFileContent.splice(index + 1, 0, insert);
  } else { //create entry in db
		let dbData = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    dbData.push({"id": data.rideId,"startEpoch": 0,"max": 0,"avg": 0});
		//!!!dodělat startEpoch,max,avg - calc function při každém přidání
    jsonStr = JSON.stringify(dbData, null, 2);
    fs.writeFile("routes.json", jsonStr, err => {
      if (err) {
        console.log(`Data couldn't be saved! Error: ${err}`);
      }
    });
		let newjson = [];
	  newjson.push({"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":data.epoch});
	}
	fs.writeFile("routes/"+String(data.rideId)+".json", String(JSON.stringify(currentFileContent, null, 2)), err => {
    if (err) {
      console.log(`Data couldn't be saved! Error: ${err}`);
    }
  });
}

io.on('connection', (socket) => {
  socket.on('getRoutes', function() { //signal to send routes from json file
    let data = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    data.forEach(function(obj) {
      socket.emit('loadRoutes', obj.id, obj.startEpoch, obj.max, obj.avg);
    });
  });
  socket.on('getPoints', routeId => { //signal to send points from specific ride
    let routePath = "routes/" + routeId + ".json";
    if (fs.existsSync(routePath)) {
      let data = JSON.parse(fs.readFileSync(routePath, 'utf8'));
			socket.emit('loadPoints', "new",0,0,0,0);
      data.forEach(function(obj) { //rideid;moving;lat;lng;kmph;epoch
        socket.emit('loadPoints', obj.moving,obj.lat,obj.lng,obj.speed,obj.epoch);
      });
    } else {
      socket.emit('error', "No ride file with ID:"+routeId+" found on server!");
    }
  });
});

process.on('uncaughtException', function(err) {
  console.log('Caught exception: ' + err);
	socket.emit('error', 'Caught exception: ' + err);
});

const PORT = process.env.PORT || 80;
server.listen(PORT, function() {
  console.log(`Server listening on port ${PORT}`);
});