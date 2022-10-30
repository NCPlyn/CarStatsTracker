const express = require('express');
const socketio = require('socket.io');
const http = require('http');
const path = require('path');
const fs = require("fs");

const app = express();
const server = http.createServer(app);
const io = socketio(server);

let pendingCalc = false;

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());

app.post('/uplink', function (req, res) {
  let jsonParsed = JSON.parse(JSON.stringify(req.body));
  let buf = Buffer.from(jsonParsed.uplink_message.frm_payload, 'base64');
  let str = buf.toString('utf-8');
  console.log(str);
  parseData(str);
  res.status(200).end()
});

app.get('/directuplink', (req, res) => {
  if(!req.params.data || req.params.data == "")
    return res.send("no param or empty");
  parseData(req.query.data);
  res.status(200).end()
});

function parseData(input) {
  const dataArray = input.split(";");
  let out = {rideId:dataArray[0],moving:dataArray[1],lat:parseFloat(dataArray[2]),lng:parseFloat(dataArray[3]),speed:parseFloat(dataArray[4]),epoch:dataArray[5]};
  io.emit("loraDecoded", out.moving, out.lat, out.lng, out.speed, out.epoch);
  saveUplink(out);
}

function search(array, insert, cb) {
  let left = -1, right = array.length, actual;
  while (left !== right && left + 1 !== right) {
    actual = Math.floor((left + right) / 2);
    if (cb(array[actual]) < cb(insert)) {
      left = actual;
      continue;
    }
    if (cb(array[actual]) > cb(insert)) {
      right = actual;
    }
    if (cb(array[actual]) == cb(insert)) {
      return -1;
    }
  }
  return left;
}

function saveUplink(data) {
  let currentFileContent;
  if (fs.existsSync("routes/"+String(data.rideId)+".json")) { //also check routes.json???
  	currentFileContent = JSON.parse(fs.readFileSync("routes/"+String(data.rideId)+".json", 'utf8'));
  	let insert = {"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":data.epoch};
  	let index = search(currentFileContent, insert, function (a) { return a.epoch; });
  	if (index != -1) {
  		currentFileContent.splice(index + 1, 0, insert);
			if(!pendingCalc){
				pendingCalc = true;
				setTimeout(function(){calcStuff(data.rideId);}, 5000);
			}
  	}
  } else { //create entry in db
  	let dbData = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    dbData.push({"id": data.rideId,"startEpoch": 0,"max": 0,"avg": 0,"total": 0});
    fs.writeFile("routes.json", JSON.stringify(dbData, null, 2), err => {
      if (err)
        console.log(`Data couldn't be saved! Error: ${err}`);
    });
  	let newjson = [];
    newjson.push({"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":data.epoch});
  }
  fs.writeFile("routes/"+String(data.rideId)+".json", String(JSON.stringify(currentFileContent, null, 2)), err => {
    if (err)
      console.log(`Data couldn't be saved! Error: ${err}`);
  });
}

function calcStuff(rideId){
  let rideData = JSON.parse(fs.readFileSync("routes/"+String(rideId)+".json", 'utf8'));
  let maxSpeed = 0, avgSpeed = 0, avgCount = 0, totalKM = 0;
  rideData.forEach(function(obj,index) {
  	if(obj.speed > maxSpeed) {
  		maxSpeed = obj.speed;
  	}
  	avgSpeed += obj.speed;
  	avgCount++;
  	if(index > 0) {
  		totalKM += getDistanceFromCoords(obj.lat,obj.lng,rideData[index-1].lat,rideData[index-1].lng);
  	}
  });
  avgSpeed = avgSpeed/avgCount;
  let dbData = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
  dbData.forEach(function(obj,index) {
  	if(obj.id == rideId) {
  		dbData[index].total = parseFloat(totalKM.toFixed(2));
  		dbData[index].max = parseFloat(maxSpeed.toFixed(2));
  		dbData[index].avg = parseFloat(avgSpeed.toFixed(2));
  		dbData[index].startEpoch = rideData[0].epoch;
  	}
  });
  fs.writeFile("routes.json", JSON.stringify(dbData, null, 2), err => {
  	if (err)
  		console.log(`Data couldn't be saved! Error: ${err}`);
  });
	pendingCalc = false;
}

function getDistanceFromCoords(lat1, lon1, lat2, lon2) {
  let dLat = deg2rad(lat2-lat1);
  let dLon = deg2rad(lon2-lon1);
  let a = Math.sin(dLat/2) * Math.sin(dLat/2) +
  				Math.cos(deg2rad(lat1)) * Math.cos(deg2rad(lat2)) *
  		    Math.sin(dLon/2) * Math.sin(dLon/2);
  let c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  let d = 6371 * c; // Distance in km
  return d;
}

function deg2rad(deg) {
  return deg * (Math.PI/180);
}

io.on('connection', (socket) => {
  socket.on('reCalc', rideId => { //signal to recalculate rides
    calcStuff(rideId);
  });
	socket.on('upload', data => {
		if(data == "") {
			io.emit('error', 'Empty upload');
		} else {
			parseData(data);
		}
  });
  socket.on('getRoutes', function() { //signal to send routes from json file
    let data = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    data.forEach(function(obj) {
      socket.emit('loadRoutes', obj.id, obj.startEpoch, obj.max, obj.avg, obj.total);
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
  io.emit('error', 'Caught exception: ' + err.stack);
  console.error(err.stack || err);
});

const PORT = process.env.PORT || 80;
server.listen(PORT, function() {
  console.log(`Server listening on port ${PORT}`);
});
