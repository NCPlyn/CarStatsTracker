#include <Arduino.h>
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

#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */
#define JOINREQ_NBTRIALS 3			   /**< Number of trials for the join request. */
hw_config hwConfig;
// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(void);
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];			  ///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.
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

AsyncWebServer server(80);

const char* ssid = "ESP";
const char* password = "1234567890";

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGGA", 5);

LSM6DS3 myIMU;

const int numOfSpeeds = 2;
int targetSpeeds[30] = {30,50};
float targetSpeedsTime[30];

const int numOfDistances = 4;
int targetDistances[30] = {100,201,305,402};
float targetDistancesTime[30];

unsigned long gpsStatMillis = millis();
float deadzoneX=0,deadzoneY=0,deadzoneZ=0;
float detectVal=0.2;
bool doDrag = false, gpsReady = false;
int opMode = 1; //0 - normal, 1 - drag
int currentRideID = 0;

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
    return false;
  }
  file.close();

  currentRideID = doc["currentRideID"].as<int>();
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

  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to deserialize the config file");
    return false;
  }
  file.close();

  return true;
}

void setup() {
  for(int x = 0;x<numOfSpeeds;x++) {
    targetSpeedsTime[x] = 0;
  }
  for(int x = 0;x<numOfDistances;x++) {
    targetDistancesTime[x] = 0;
  }

  Serial.begin(115200);
  delay(1000);
  Serial.println("Processor came out of reset.\n");

  myIMU.begin();

  Serial2.begin(9600, SERIAL_8N1, 16, 17); //RX 16, TX 17

  pinMode(0, INPUT);

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String sout = "<a href='/drag'>DRAG</a><br><a href='/normal'>NORMAL</a>";
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
  server.begin();

  calibrateAcc();
  loadSPIFFS();

  copyArrays();
  LoraSetup();
}

unsigned long detectMoveMillis = millis(), lastLora = millis(), lastNotMovedLora = millis();
bool moveDetected = false, routeActive = false, detectMove = true, dragging = false;
int retry = 0,didntMove = 0;
double lastLat = 48.85826 ,lastLng = 2.294516, startLat = 0.0, startLng = 0.0;

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

  if(gpsReady==true && routeActive==false && millis()>lastNotMovedLora+60000 /*&& loraReady*/){ //předělat na delší čas,
    sendPos("n");
    lastNotMovedLora = millis();
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
        currentRideID++;
        saveSPIFFS();
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
      //send rideId,moving(y/n),lat,lng,kmph,epochTime
      sendPos("y");

      lastLora = millis();
    }
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
       if((myIMU.readFloatAccelX()-deadzoneX)>detectVal || (myIMU.readFloatAccelY()-deadzoneY)>detectVal || (myIMU.readFloatAccelZ()-deadzoneZ)>detectVal) {
        Serial.println("Movement detected on X");
        doDrag = false;
        startLat = gps.location.lat();
        startLng = gps.location.lng();
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
  String out = String(currentRideID)+","+moving+String(gps.location.lat())+","+String(gps.location.lng())+","+String(gps.speed.kmph())+","+String(getUnix(gps.date.year(),gps.date.month(),gps.date.day(),gps.time.hour(),gps.time.minute(),gps.time.second()));
  Serial.println(out);
  //write to file?
  send_lora_frame(out);
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
//this commit not tested
//after lora will be added, it will be tested

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
	Serial.begin(115200);
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
}

//Function for handling LoRaWan received data from Gateway
static void lorawan_rx_handler(lmh_app_data_t *app_data) { //app_data - Pointer to rx data
	Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d\n",
				  app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);

	switch (app_data->port)
	{
	case 3:
		// Port 3 switches the class
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
  toSend.toCharArray(reinterpret_cast<char *>(m_lora_app_data.buffer), toSend.length());
  Serial.println(m_lora_app_data.buffer);
	m_lora_app_data.buffsize = toSend.length();

	lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
	Serial.printf("lmh_send result %d\n", error);
}

//stále nevyzkoušené, ale zkompiluje se a mělo by to fungovat
