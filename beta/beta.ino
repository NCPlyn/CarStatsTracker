#include <Arduino.h>
#include <sstream>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <LoRaWan-Arduino.h>
#define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
#define CONFIG_LITTLEFS_CACHE_SIZE 256
#define SPIFFS LITTLEFS
#include <LITTLEFS.h>
#include "loraCreds.h"
#include "FS.h"
#include "SD.h"

#define LORALED 26

//------------------SD CARD------------------//
#define SD_CS 15
#define SPI_SCK 14
#define SPI_MISO 27
#define SPI_MOSI 13
SPIClass SPISD(HSPI);
//Also to properly work, add "VSPI" to the () in 'SX126x-Arduino\src\boards\mcu\spi_board.cpp' on line 5

//------------------LORA------------------//
#define LORAWAN_APP_DATA_BUFF_SIZE 50 // Size of the data to be transmitted
#define JOINREQ_NBTRIALS 3  // Number of trials for the join request
hw_config hwConfig;
// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(void);
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE]; // Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; // Lora user application data structure.
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, DR_3, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF}; //Structure containing LoRaWan parameters, needed for lmh_init()
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  									lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler};//Structure containing LoRaWan callback functions, needed for lmh_init()

uint8_t nodeDeviceEUI[8], nodeAppEUI[8];
uint8_t nodeAppKey[16], nodeNwsKey[16], nodeAppsKey[16];
uint32_t nodeDevAddr;

void copyArrays() {
  memcpy(nodeDeviceEUI,STOREDnodeDeviceEUI,sizeof(STOREDnodeDeviceEUI));
  memcpy(nodeAppEUI,STOREDnodeAppEUI,sizeof(STOREDnodeAppEUI));
  memcpy(nodeAppKey,STOREDnodeAppKey,sizeof(STOREDnodeAppKey));
  memcpy(nodeNwsKey,STOREDnodeNwsKey,sizeof(STOREDnodeNwsKey));
  memcpy(nodeAppsKey,STOREDnodeAppsKey,sizeof(STOREDnodeAppsKey));
}

//------------------WiFi & Web------------------//
const char* ssid = "CST";
const char* password = "1234567890";

AsyncWebServer server(80);

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

//------------------GPS & IMU ------------------//
TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGGA", 6);

LSM6DS3 myIMU;

//------------------Variable declaration------------------//
float deadzoneX=0,deadzoneY=0,deadzoneZ=0;

int opMode = 0; //0 - normal, 1 - drag
int currentRideID = 0;
bool enaLora = true; //false - GSM
float detectVal=0.4;

//------Drag------
const int numOfSpeeds = 2;
int targetSpeeds[30] = {30,50};
float targetSpeedsTime[30];

const int numOfDistances = 4;
int targetDistances[30] = {100,201,305,402};
float targetDistancesTime[30];

bool dragging = false, doDrag = false;

//------Normal------
unsigned long detectMoveMillis = millis(), lastLora = millis();
bool detectMove = true, moveDetected = false, routeActive = false, gpsReady = false;
int retryMoveDetect = 0, didntMove = 0;
double lastLat = 48.85826, lastLng = 2.294516;


bool loadSPIFFS() {
  DynamicJsonDocument doc(512);

  File file = SPIFFS.open("/config.json", "r");
  if (!file) {
    Serial.println("There was an error opening the config file!");
    file.close();
    return false;
  }
  Serial.println("Config file opened!");

  DeserializationError error = deserializeJson(doc, file);
  if (error){
    Serial.println("Failed to deserialize the config file!");
    file.close();
    return false;
  }
  file.close();

  currentRideID = doc["currentRideID"].as<int>();
  enaLora = doc["enaLora"].as<bool>();
  detectVal = doc["detectVal"].as<float>();
  return true;
}

bool saveSPIFFS() {
  DynamicJsonDocument doc(512);

  File file = SPIFFS.open("/config.json", "w");
  if (!file) {
    Serial.println("There was an error opening the config file!");
    file.close();
    return false;
  }
  Serial.println("Config file opened!");

  doc["currentRideID"] = currentRideID;
  doc["enaLora"] = enaLora;
  doc["detectVal"] = detectVal;

  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to deserialize the config file");
    file.close();
    return false;
  }
  file.close();

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Processor came out of reset.\n");

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
  }

  for(int x = 0;x<numOfSpeeds;x++) {
    targetSpeedsTime[x] = 0;
  }
  for(int x = 0;x<numOfDistances;x++) {
    targetDistancesTime[x] = 0;
  }

  pinMode(SD_CS, OUTPUT);
  SPISD.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if(!SD.begin(SD_CS,SPISD)){
    Serial.println("Card Mount Failed");
  } else {
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  }

  myIMU.begin();

  Serial2.begin(115200, SERIAL_8N1, 16, 17); //RX 16, TX 17

  pinMode(0, INPUT);
  pinMode(LORALED, OUTPUT);
  digitalWrite(LORALED, LOW);

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LITTLEFS, "/").setDefaultFile("index.html");

  server.on("/opmode", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("val")) {
      modeSwitch(request->getParam("rideid")->value().toInt());
      if(request->getParam("rideid")->value() == 0) {
        request->redirect("/");
      } else {
        request->redirect("/drag.html");
      }
    }
    request->send(200, "text/plain", "No param 'val'");
  });
  server.on("/doDrag", HTTP_GET, [](AsyncWebServerRequest *request){
    if(opMode == 1 && gpsReady && !doDrag) {
      calibrateAcc();
      doDrag = true;
      request->redirect("/drag.html");
    } else if (opMode == 0) {
      request->send(200, "text/plain", "not in drag mode");
    } else if (!gpsReady) {
      request->send(200, "text/plain", "GPS not ready: age("+String(gps.location.age())+"), fix("+String(gpsFix.value())+"), sats("+String(gps.satellites.value())+")");
    } else {
      request->send(200, "text/plain", "Drag already in progress or other.");
    }
  });
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request){
    calibrateAcc();
    request->send(200, "text/plain", "ok");
  });
  server.on("/getfile", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("rideid")) {
      String path = "/"+request->getParam("rideid")->value()+".data";
      Serial.printf("Reading file: %s\n", path);
      File file = SD.open(path);
      if(!file){
          Serial.println("Failed to open file for reading");
          return;
      }
      String dataFileContents;
      while(file.available()){
          dataFileContents += (char)file.read();
      }
      //Serial.println(dataFileContents);
      file.close();
      request->send(200, "text/plain", dataFileContents);
    } else {
      request->send(200, "text/plain", "Parameter 'rideid' not present!");
    }
  });
  server.on("/getfiles", HTTP_GET, [](AsyncWebServerRequest *request){
    String out;
    File root = SD.open("/");
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
          out += String(file.name())+";";
        }
        file = root.openNextFile();
    }
    request->send(200, "text/plain", out);
  });
  server.on("/saveconfig", HTTP_GET, [](AsyncWebServerRequest *request){ //saves config
    if(request->hasParam("enaLora"))
      std::istringstream(request->getParam("enaLora")->value().c_str()) >> std::boolalpha >> enaLora;
    if(request->hasParam("currentRideID"))
      currentRideID = request->getParam("currentRideID")->value().toInt();
    if(request->hasParam("detectVal"))
      detectVal = String(request->getParam("detectVal")->value()).substring(0, 4).toFloat();
    if(saveSPIFFS()) {
      ESP.restart();
      request->redirect("/");
    } else {
      request->send(200, "text/plain", "Saving config failed!");
    }
  });

  server.onNotFound(notFound);
  server.begin();

  calibrateAcc();
  loadSPIFFS();

  if(enaLora) {
    copyArrays();
    LoraSetup();
  } else {
    //GSM init
  }
}

void loop()
{
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if(!gps.location.isValid() || gps.location.age() > 2000 || gpsFix.value() == 0 || gps.satellites.value() < 2) {
        gpsReady = false;
      } else {
        gpsReady = true;
      }
    }
  }

  if(gpsReady==true && routeActive==false && millis()>lastLora+60000){ //předělat na delší čas,
    Serial.println("sending data");
    sendPos("n");
    lastLora = millis();
  }

  if(opMode == 0) {
    if(detectMove && millis()>(detectMoveMillis+5000) && ((myIMU.readFloatAccelX()-deadzoneX)>detectVal || (myIMU.readFloatAccelY()-deadzoneY)>detectVal || (myIMU.readFloatAccelZ()-deadzoneZ)>detectVal)) {
      Serial.println("Movement detected on accel");
      moveDetected = true;
      detectMove = false;
      retryMoveDetect = 0, didntMove = 0;
      detectMoveMillis = millis();
    }
    if(millis()>(detectMoveMillis+10000) && moveDetected) { //has to be better
      Serial.println(gps.speed.kmph());
      if(gpsReady && gps.speed.kmph() > 5) {
        Serial.println("We movin");
        routeActive = true;
        moveDetected = false;
        currentRideID++;
        saveSPIFFS();
      } else {
        if(retryMoveDetect < 3) {
          retryMoveDetect++;
          detectMoveMillis = millis();
        } else if (retryMoveDetect > 2) {
          moveDetected = false;
          detectMove = true;
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
        }
      } else {
        didntMove = 0;
      }
      lastLat = gps.location.lat();
      lastLng = gps.location.lng();
      sendPos("y");
      lastLora = millis();
    }
  } else if(opMode == 1) {
    if(doDrag == true) {
      if((myIMU.readFloatAccelX()-deadzoneX)>detectVal || (myIMU.readFloatAccelY()-deadzoneY)>detectVal || (myIMU.readFloatAccelZ()-deadzoneZ)>detectVal) {
        Serial.println("Movement detected");
        doDrag = false;
        dragTimer();
      }
    }
  }
}

uint32_t getUnix(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second) {
  int8_t my = (month >= 3) ? 1 : 0;
  uint16_t y = year + my - 1970;
  uint16_t dm = 0;
  for (int i = 0; i < month - 1; i++) dm += (i<7)?((i==1)?28:((i&1)?30:31)):((i&1)?31:30);
  return (((day-1+dm+((y+1)>>2)-((y+69)/100)+((y+369)/100/4)+365*(y-my))*24ul+hour)*60ul+minute)*60ul+second;
}

bool sendPos(String moving) { //rideid;moving;lat;lng;kmph;epoch - opravdu celý float?
  String out = String(currentRideID)+";"+moving+";"+String(gps.location.lat(),5)+";"+String(gps.location.lng(),5)+";"+String(gps.speed.kmph())+";"+String(getUnix(gps.date.year(),gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),gps.time.second()));
  Serial.println(out);
  fileOper(SD,"/"+String(currentRideID)+".data",out);
  if(enaLora) { //&& loraReady
    send_lora_frame(out);
  } else { //gsm ready
    //GSM send
  }
}

void fileOper(fs::FS &fs, String path, String message) {
  String msg;
  File file;
  if(fs.exists(path)) {
    file = fs.open(path, FILE_APPEND);
    msg = "append";
  } else {
    file = fs.open(path, FILE_WRITE);
    msg = "write";
  }
  if(!file){
    Serial.println("Failed to open file for "+msg);
    return;
  }
  if(file.println(message)){
    Serial.println("Data "+msg);
  } else {
    Serial.println("Failed data "+msg);
  }
  file.close();
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
    opMode = 0;
  }
  calibrateAcc();
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

int dragTimer() {
  double startLat = gps.location.lat(), startLng = gps.location.lng();
  unsigned long startTime = millis();
  float currentSpeed = 0, currentDistance = 0;
  Serial.println("Drag started");
  dragging = true;

  while(digitalRead(0) == HIGH && targetSpeedsTime[numOfSpeeds-1] == 0 && targetDistancesTime[numOfDistances-1] == 0 && dragging) {

    while (Serial2.available() > 0) {
      gps.encode(Serial2.read());
    }

    currentSpeed = gps.speed.kmph();
    currentDistance = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),startLat,startLng);

    for(int x = 0;x<numOfSpeeds;x++){
      if(targetSpeedsTime[x] == 0) {
        if(currentSpeed > targetSpeeds[x]) {
          targetSpeedsTime[x] = float(millis()-startTime-gps.location.age())/1000;
        }
      }
    }
    for(int x = 0;x<numOfDistances;x++){
      if(targetDistancesTime[x] == 0) {
        if(currentDistance > targetDistances[x]) {
          targetDistancesTime[x] = float(millis()-startTime-gps.location.age())/1000;
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
  for(int x = 0;x<numOfDistances;x++){
    Serial.printf("%d m in %f sec\n", targetDistances[x], targetDistancesTime[x]);
    targetDistancesTime[x] = 0;
  }
}

void LoraSetup() {
  // Define the HW configuration between MCU and SX126x
  hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
  hwConfig.PIN_LORA_RESET = 4;    // LORA RESET
  hwConfig.PIN_LORA_NSS = 5;	    // LORA SPI CS
  hwConfig.PIN_LORA_SCLK = 18;	  // LORA SPI CLK
  hwConfig.PIN_LORA_MISO = 19;	  // LORA SPI MISO
  hwConfig.PIN_LORA_DIO_1 = 2;    // LORA DIO_1
  hwConfig.PIN_LORA_BUSY = 34;	  // LORA SPI BUSY
  hwConfig.PIN_LORA_MOSI = 23;	  // LORA SPI MOSI
  hwConfig.RADIO_TXEN = -1;		  // LORA ANTENNA TX ENABLE
  hwConfig.RADIO_RXEN = -1;		  // LORA ANTENNA RX ENABLE
  hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
  hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
  hwConfig.USE_DIO3_ANT_SWITCH = false;	  // Only Insight ISP4520 module uses DIO3 as antenna control
  hwConfig.USE_RXEN_ANT_PWR = false;

  // Initialize Serial for debug output
  Serial.println("SX126x LoRaWan");

  // Initialize LoRa chip.
  uint32_t err_code = lora_hardware_init(hwConfig);
  if (err_code != 0)
  {
  	Serial.printf("lora_hardware_init failed - %d\n", err_code);
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);
  lmh_setNwkSKey(nodeNwsKey);
  lmh_setAppSKey(nodeAppsKey);
  lmh_setDevAddr(nodeDevAddr);

  // Initialize LoRaWan
  err_code = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAMAC_REGION_EU868);
  if (err_code != 0)
  {
  	Serial.printf("lmh_init failed - %d\n", err_code);
  }

  // Setup connection to a single channel gateway
  //lmh_setSingleChannelGateway(2, DR_3);

  // For some regions we might need to define the sub band the gateway is listening to
  // This must be called AFTER lmh_init()
  /// \todo This is for Dragino LPS8 gateway. How about other gateways???
  if (!lmh_setSubBandChannels(1))
  {
  	Serial.println("lmh_setSubBandChannels failed. Wrong sub band requested?");
  }

  // Start Join procedure
  lmh_join();
}

//LoRa function for handling OTAA join failed
static void lorawan_join_failed_handler(void) {
  Serial.println("OVER_THE_AIR_ACTIVATION failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
  digitalWrite(LORALED, LOW);
}

//LoRa function for handling HasJoined event.
static void lorawan_has_joined_handler(void) {
  #if (OVER_THE_AIR_ACTIVATION != 0)
  	Serial.println("Network Joined");
  #else
  	Serial.println("OVER_THE_AIR_ACTIVATION != 0");

  #endif
  lmh_class_request(CLASS_A);

  //lora ready (what about not ready? disconnected..)
  digitalWrite(LORALED, HIGH);
}

//Function for handling LoRaWan received data from Gateway
static void lorawan_rx_handler(lmh_app_data_t *app_data) { //app_data - Pointer to rx data
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
  			  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

  switch (app_data->port)
  {
  case 3:// Port 3 switches the class
  	if (app_data->buffsize == 1)
  	{
  		switch (app_data->buffer[0])
  		{
  		case 0:
  			lmh_class_request(CLASS_A);
  			break;

  		case 1:
  			lmh_class_request(CLASS_B);
  			break;

  		case 2:
  			lmh_class_request(CLASS_C);
  			break;

  		default:
  			break;
  		}
  	}
  	break;

  case LORAWAN_APP_PORT:
  	// YOUR_JOB: Take action on received data
  	break;

  default:
  	break;
  }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("switch to class %c done\n", "ABC"[Class]);

  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

static void send_lora_frame(String toSend) {
  if (lmh_join_status_get() != LMH_SET)
  {
  	//Not joined, try again later
  	Serial.println("Did not join network, skip sending frame");
  	return;
  }

  uint32_t i = 0;
  m_lora_app_data.port = LORAWAN_APP_PORT;
  toSend.toCharArray(reinterpret_cast<char *>(m_lora_app_data.buffer), toSend.length()+1);
  m_lora_app_data.buffsize = toSend.length();

  lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
  Serial.printf("lmh_send result %d\n", error);
}
