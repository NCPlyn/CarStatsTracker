const express = require('express');
const basicAuth = require('express-basic-auth')
const socketio = require('socket.io');
const https = require('https');
const http = require('http');
const path = require('path');
const fs = require("fs");
const mqtt = require('mqtt');
const dotenv = require('dotenv').config();

let httpsoptions = {
  key: fs.readFileSync('privatekey.pem'),
  cert: fs.readFileSync('certificate.pem')
};

const app = express();
const server = http.createServer(app);
const io = socketio(server);
const serverhttps = https.createServer(httpsoptions, app);
const sio = socketio(serverhttps);

let pendingCalc = [];
let buffArray = [];
let savingBusy = false;

//----------MQTT----------
let mqttoptions = {
    port: 1883,
    clientId: 'mqttjs_' + Math.random().toString(16).substr(2, 8),
    username: process.env.MQTT_USERNAME,
    password: process.env.MQTT_PASSWORD,
    keepalive: 60,
    reconnectPeriod: 1000,
    protocolId: 'MQIsdp',
    protocolVersion: 3,
    clean: true,
    encoding: 'utf8'
};
let mqttclient = mqtt.connect('https://eu1.cloud.thethings.network',mqttoptions);

mqttclient.on('connect', function() {
    console.log('Connected to TTN MQTT')
    mqttclient.subscribe('#')
});

mqttclient.on('error', function(err) {
    console.log(err);
});

mqttclient.on('message', function(topic, message) {
    let parsedData = JSON.parse(message);
    if(parsedData.uplink_message) {
      let decodedData = Buffer.from(parsedData.uplink_message.frm_payload, 'base64').toString();
      console.log("Data from TTN: ", decodedData);
      if(!parseData(decodedData))
        console.log("Something is wrong m8 with the data trying do be parsed!");
    } else {
      console.log("No data available");
    }
});

//----------Express----------
app.use(basicAuth({
    users: { 'admin': process.env.ADMINPASS },
    challenge: true
}));

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.json());
app.use(express.text());

app.post('/uplink', function (req, res) {
  console.log(req.body);
  if(req.body.charAt(0) == "{") {
    if(parseDrag(req.body)){
      res.status(200).end()
    } else {
      res.status(422).end()
    }
  } else {
    if(parseData(req.body)){
      res.status(200).end()
    } else {
      res.status(422).end()
    }
  }
});

//----------Funtions----------
function parseData(input) {
  if((input.match(/;/g) || []).length != 5)
    return false;
  let dataArray = input.split(";"); //rideid;moving;lat;lng;kmph;epoch
  if(isNaN(dataArray[0]) || dataArray[5] < 1582230000 || isNaN(dataArray[4]) || isNaN(dataArray[3]) || dataArray[3] < 1 || isNaN(dataArray[2]) || dataArray[2] < 1 || (dataArray[1].charAt(0) != "y" && dataArray[1].charAt(0) != "n") || dataArray[1].length != 1)
    return false;
  let out = {rideId:dataArray[0],moving:dataArray[1],lat:parseFloat(dataArray[2]),lng:parseFloat(dataArray[3]),speed:parseFloat(dataArray[4]),epoch:dataArray[5]};
  io.emit("loraDecoded", out.moving, out.lat, out.lng, out.speed, out.epoch);
  buffArray.push(out);
  return true;
}

function parseDrag(input) {
  if (fs.existsSync("routes/drags.json")) {
    let currentFileContent = JSON.parse(fs.readFileSync("routes/drags.json", 'utf8'));
    currentFileContent.push(JSON.parse(input));
    fs.writeFile("routes/drags.json", JSON.stringify(currentFileContent, null, 2), err => {
      if (err) {
        console.log(`Drag data couldn't be saved! Error: ${err}`);
        return false;
      }
    });
    return true;
  } else {
    console.log("File routes/drags.json does not exist!");
    return false;
  }
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
      return -2;
    }
  }
  return left;
}

function saveUplink(data) {
  let currentFileContent = [];
  if (fs.existsSync("routes/"+String(data.rideId)+".json")) {
    currentFileContent = JSON.parse(fs.readFileSync("routes/"+String(data.rideId)+".json", 'utf8'));
    let insert = {"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":Number(data.epoch)};
    let index = search(currentFileContent, insert, function (a) { return a.epoch; });
    if (index != -2) {
      currentFileContent.splice(index + 1, 0, insert);
      if(!pendingCalc.includes(Number(data.rideId))) {
        pendingCalc.push(Number(data.rideId));
        setTimeout(function(){calcStuff(Number(data.rideId));}, 30000);
      }
    }
  } else { //create entry in db
    let alreadyFound = false;
    let dbData = JSON.parse(fs.readFileSync("routes.json", 'utf8'));
    for(let index = 0; index < dbData.length; index++) {
      if(dbData[index].id == data.rideId) {
        dbData[index].startEpoch = 0;
        dbData[index].max = 0;
        dbData[index].avg = 0;
        dbData[index].total = 0;
        alreadyFound = true;
        break;
      }
    }
    if(!alreadyFound) {
      dbData.push({"id": data.rideId,"startEpoch": 0,"max": 0,"avg": 0,"total": 0});
      fs.writeFileSync("routes.json", JSON.stringify(dbData, null, 2));
	}
    currentFileContent.push({"moving":data.moving,"lat":data.lat,"lng":data.lng,"speed":data.speed,"epoch":data.epoch});
  }
  fs.writeFileSync("routes/"+String(data.rideId)+".json", String(JSON.stringify(currentFileContent, null, 2)));
}

setInterval(function () {
  if(!savingBusy && buffArray.length > 0) {
    savingBusy = true;
    saveUplink(buffArray.shift());
    savingBusy = false;
  }
}, 150);

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
  fs.writeFileSync("routes.json", JSON.stringify(dbData, null, 2));
  pendingCalc = pendingCalc.filter(item => item !== rideId);
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

//----------Socket.io----------
io.on("connection", (socket) => {
    ServersReceived(socket);
});
sio.on("connection", (socket) => {
    ServersReceived(socket);
});

function ServersReceived(socket) {
  socket.on('reCalc', rideId => { //signal to recalculate rides
    calcStuff(rideId);
  });
  socket.on('upload', data => {
    if(data == "") {
      io.emit('error', 'Empty upload');
    } else {
      if(data.charAt(0) == "{") {
        if(!parseDrag(data)) {
          console.log("Something is wrong m8 with the data trying do be parsed!");
          io.emit('error', "Something is wrong m8 with the data trying do be parsed!");
        }
      } else {
        if(!parseData(data)) {
          console.log("Something is wrong m8 with the data trying do be parsed!");
          io.emit('error', "Something is wrong m8 with the data trying do be parsed!");
        }
      }
    }
  });
  socket.on('getDrags', function() { //signal to send drags from json file
    let data = JSON.parse(fs.readFileSync("routes/drags.json", 'utf8'));
    socket.emit('loadDrags', "new");
    if(data.length != 0) {
      data.forEach(function(obj) {
        socket.emit('loadDrags', obj);
      });
    } else {
      socket.emit('error', "Drag file empty!");
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
};

//----------Server stuff----------
process.on('uncaughtException', function(err) {
  io.emit('error', 'Caught exception: ' + err.stack);
  console.error(err.stack || err);
});

server.listen(80, function() {
  console.log(`HTTP server listening on port 80`);
});
serverhttps.listen(443, function() {
  console.log(`HTTPS server listening on port 433`);
});
