#include <Arduino.h>
#include <LoRaWan-Arduino.h>
#include <SPI.h>

#include <TinyGPS++.h>

TinyGPSPlus gps;

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE //< Maximum size of scheduler events.????
#define SCHED_QUEUE_SIZE 60										  //< Maximum number of events in the scheduler queue.????

#define LORAWAN_APP_DATA_BUFF_SIZE 64  /**< Size of the data to be transmitted. */
#define JOINREQ_NBTRIALS 3			   /**< Number of trials for the join request. */

hw_config hwConfig;

// ESP32 - SX126x pin configuration
int PIN_LORA_RESET = 4;	 // LORA RESET
int PIN_LORA_NSS = 5;	 // LORA SPI CS
int PIN_LORA_SCLK = 18;	 // LORA SPI CLK
int PIN_LORA_MISO = 19;	 // LORA SPI MISO
int PIN_LORA_DIO_1 = 2; // LORA DIO_1 //w:21
int PIN_LORA_BUSY = 34;	 // LORA SPI BUSY //w:22
int PIN_LORA_MOSI = 23;	 // LORA SPI MOSI
int RADIO_TXEN = -1;	 // LORA ANTENNA TX ENABLE
int RADIO_RXEN = -1;	 // LORA ANTENNA RX ENABLE

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_join_failed_handler(void);
static void send_lora_frame(String msg);

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];		///< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; ///< Lora user application data structure.

//Structure containing LoRaWan parameters, needed for lmh_init()
static lmh_param_t lora_param_init = {LORAWAN_ADR_ON, DR_3, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

//Structure containing LoRaWan callback functions, needed for lmh_init()
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
										lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler};

uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x04, 0xE4, 0x13};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16] = {0xD9, 0x59, 0x79, 0x92, 0xA7, 0x66, 0x23, 0xC7, 0x83, 0x39, 0xFA, 0xF1, 0x45, 0x5C, 0x7D, 0x2D};

//uint32_t nodeDevAddr = 0x260B96C1;
//uint8_t nodeNwsKey[16] = {0x01, 0x22, 0x7E, 0x9D, 0xA8, 0x6F, 0x4A, 0x44, 0x9C, 0x21, 0x1C, 0xCC, 0xE2, 0xE8, 0x6D, 0xE1};
//uint8_t nodeAppsKey[16] = {0x8F, 0xD7, 0x3F, 0xE4, 0x51, 0x67, 0x9C, 0x2A, 0x1B, 0xB0, 0x1E, 0xC2, 0xB0, 0x01, 0x29, 0x35};

void setup()
{
  pinMode(0, INPUT);
  Serial.begin(115200);

  //GPS
  Serial2.begin(9600, SERIAL_8N1, 16, 17); //RX 16, TX 17
  
	//LoRa
	hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	  // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;	  // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;	  // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;	  // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;	  // LORA SPI MOSI
	hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	  // Only Insight ISP4520 module uses DIO3 as antenna control
  hwConfig.USE_RXEN_ANT_PWR = false;

	Serial.println("=====================================");
	Serial.println("SX126x LoRaWan test");
	Serial.println("=====================================");

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
	//lmh_setNwkSKey(nodeNwsKey);
	//lmh_setAppSKey(nodeAppsKey);
	//lmh_setDevAddr(nodeDevAddr);

	// Initialize LoRaWan
	err_code = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, LORAMAC_REGION_EU868);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %d\n", err_code);
	}
 
	// Setup connection to a single channel gateway
  //lmh_setSingleChannelGateway(2, DR_3);
	if (!lmh_setSubBandChannels(1))
	{
		Serial.println("lmh_setSubBandChannels failed. Wrong sub band requested?");
	}

	// Start Join procedure
	lmh_join();
}

bool pressed = false;
void loop() {
	// We are on FreeRTOS, give other tasks a chance to run
	gps.encode(Serial2.read());
  if (digitalRead(0) == LOW && !pressed)
  { 
    pressed = true;
    String outformatted = "0;1;";

    if (gps.location.isValid()) {
      outformatted += String(gps.location.lat()) + ";" + gps.location.lng() + ";" + gps.speed.kmph() + ";";
    } else {
      outformatted += ";;;";
    }
    if (gps.date.isValid()) {
      outformatted += String(gps.date.day()) + ";" + gps.date.month() + ";" + gps.date.year() + ";";
    } else {
      outformatted += ";;;";
    }
    if (gps.time.isValid()) {
      outformatted += String(gps.time.hour()) + ";" + gps.time.minute() + ";" + gps.time.second() + ";" + gps.time.centisecond()  + ";";
    } else {
      outformatted += ";;;;";
    }
    
    send_lora_frame(outformatted);
    //rideId;pointId;lat;lng;kmph;day;month;year;hour;minute;second;centisecond
    //0;1;14.56584;50.68877;50;15;5;2022;14;28;3;86;
  }
  if(digitalRead(0) == HIGH && pressed) {
    pressed = false;
  }
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

static void send_lora_frame(String msg) {
	if (lmh_join_status_get() != LMH_SET)
	{
		//Not joined, try again later
		Serial.println("Did not join network, skip sending frame");
		return;
	}
  Serial.println("Sending frame");
 
  int n = msg.length();
  m_lora_app_data.port = LORAWAN_APP_PORT;
  for(int y = 0;y<n;y++) {
    m_lora_app_data.buffer[y] = msg[y];
  }
  m_lora_app_data.buffsize = n;

	lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
	Serial.printf("lmh_send result %d\n", error);
}
