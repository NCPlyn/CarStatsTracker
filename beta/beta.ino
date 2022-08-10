#include <Arduino.h>
#include <WiFi.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <TinyGPS++.h>

//AsyncWebServer server(80);

const char* ssid = "ESP";
const char* password = "1234567890";

/*void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}*/

TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGGA", 5);
 
LSM6DS3 myIMU;

const int numOfSpeeds = 2;
int targetSpeeds[30] = {30,50};
float targetSpeedsTime[30];

unsigned long gpsStatMillis = millis();
float deadzoneX=0,deadzoneY=0,deadzoneZ=0;
float detectVal=0.2;
bool doDrag = false, gpsReady = false;
int opMode = 1; //0 - normal, 1 - drag
 
void setup() {
  for(int x = 0;x<numOfSpeeds;x++) {
    targetSpeedsTime[x] = 0;
  }
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("Processor came out of reset.\n");
   
  myIMU.begin();

  Serial2.begin(9600, SERIAL_8N1, 16, 17); //RX 16, TX 17

  pinMode(0, INPUT);

  /*WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String sout = "opMode:"+String(opMode)+", gpsReady:"+String(gpsReady);
    request->send(200, "text/plain", sout);
  });
  server.on("/drag", HTTP_GET, [](AsyncWebServerRequest *request){
    modeSwitch(1);
    request->send(200, "text/plain", "ok");
  });
  server.on("/normal", HTTP_GET, [](AsyncWebServerRequest *request){
    modeSwitch(0);
    request->send(200, "text/plain", "ok");
  });

  server.onNotFound(notFound);
  server.begin();*/

  calibrateAcc();
}

unsigned long detectMoveMillis = millis();
unsigned long lastLora = millis();
bool moveDetected = false, routeActive = false, detectMove = true, dragging = false;
int retry = 0,didntMove = 0;
double lastLat = 48.85826 ,lastLng = 2.294516;

void loop()
{
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if(millis()>(gpsStatMillis+5000)) {
        if(gps.location.age() > 2000) {// || gpsFix.value() == 0 || gps.satellites.value() < 3
          gpsReady = false;
        } else {
          gpsReady = true;
        }
        gpsStatMillis = millis();
      }
    }
  }
  
  if(opMode == 0) {
    if(digitalRead(0) == LOW) {
      calibrateAcc();
    }
    //vymenit za gyro?
    if(detectMove && millis()>(detectMoveMillis+5000) && ((myIMU.readFloatAccelX()-deadzoneX)>detectVal || (myIMU.readFloatAccelY()-deadzoneY)>detectVal || (myIMU.readFloatAccelZ()-deadzoneZ)>detectVal)) {
      Serial.println("Movement detected on accel");
      moveDetected = true;
      detectMove = false;
      detectMoveMillis = millis();
    }
    if(millis()>(detectMoveMillis+10000) && moveDetected) { //has to be better
      Serial.println(gps.speed.kmph());
      if(gps.location.isValid() && gps.speed.kmph() > 5) {
        Serial.println("We movin");
        routeActive = true;
        moveDetected = false;
      } else {
        if(retry < 3) {
          retry++;
          detectMoveMillis = millis();
        } else if (retry > 2) {
          moveDetected = false;
          detectMove = true;
          retry = 0;
          Serial.println("Failed to move");
        }
      }
    }
    if(millis()>(lastLora+60000) && routeActive) {
      if(TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),lastLat,lastLng) < 10) {
        if(didntMove < 3) {
          Serial.println("Didnt move gps");
          didntMove++;
        } else if (didntMove > 2) {
          routeActive = false;
          detectMove = true;
          didntMove = 0;
          //save route
        }
      }
      lastLat = gps.location.lat();
      lastLng = gps.location.lng();
      //send lora
      Serial.print(gps.location.lat());
      Serial.print(",");
      Serial.print(gps.location.lng());
      Serial.print(",");
      Serial.print(gps.speed.kmph());
      Serial.print(",");
      Serial.println(gps.time.value());
      
      lastLora = millis();
    }
    //start sending lora packets
  } else if(opMode == 1) {
    if(digitalRead(0) == LOW && gpsReady && !doDrag) {
      calibrateAcc();
      doDrag = true;
    } else if (digitalRead(0) == LOW && !gpsReady) {//better
      Serial.print("GPS is not ready, current info (age,fix,sats): ");
      Serial.print(gps.location.age());
      Serial.print(",");
      Serial.print(gpsFix.value());
      Serial.print(",");
      Serial.println(gps.satellites.value());
      delay(100);
    }
  
    if(doDrag == true) {
       if((myIMU.readFloatAccelX()-deadzoneX)>detectVal) {
        Serial.println("Movement detected on X");
        doDrag = false;
        dragTimer();
       } else if ((myIMU.readFloatAccelY()-deadzoneY)>detectVal) {
        Serial.println("Movement detected on Y");
        doDrag = false;
        dragTimer();
      } else if ((myIMU.readFloatAccelZ()-deadzoneZ)>detectVal) {
        Serial.println("Movement detected on Z");
        doDrag = false;
        dragTimer();
      }
    }
  }
}

void modeSwitch(int opm) {
  if(opMode == 0 && opm == 1) {
    routeActive = false;
    detectMove = true;
    didntMove = 0;
    //save route
    opMode = 1;
  } else if (opMode == 1 && opm == 0) {
    doDrag = false;
    dragging = false;
  }
}

void calibrateAcc() {
  Serial.println("Stay still, calibrating...");
  for(int x = 0; x<5; x++) {
    deadzoneX+=myIMU.readFloatAccelX();
    deadzoneY+=myIMU.readFloatAccelY();
    deadzoneZ+=myIMU.readFloatAccelZ();
    delay(10);
  }
  deadzoneX=deadzoneX/5;
  deadzoneY=deadzoneY/5;
  deadzoneZ=deadzoneZ/5;
  Serial.println("Calibration done");
}

int dragTimer() { //this needs to run on another core
  unsigned long startTime = millis();
  float currentSpeed = 0;
  Serial.println("Drag started");
  dragging = true;
  
  while(digitalRead(0) == HIGH && targetSpeedsTime[numOfSpeeds-1] == 0 && dragging) {
    
    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }
    
    currentSpeed = gps.speed.kmph();

    for(int x = 0;x<numOfSpeeds;x++){
      if(targetSpeedsTime[x] == 0) {
        if(currentSpeed > targetSpeeds[x]) {
          targetSpeedsTime[x] = float(millis()-startTime-gps.location.age())/1000;
        }
      }
    }
    
  }
  
  Serial.println("Drag ended");
  dragging = false;

  for(int x = 0;x<numOfSpeeds;x++){
    Serial.printf("%d kmh/h in %f sec\n", targetSpeeds[x], targetSpeedsTime[x]);
    targetSpeedsTime[x] = 0;
  }
}
