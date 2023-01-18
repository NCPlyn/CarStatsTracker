# CarStatsTracker
Závěrečná práce\
Pro více informací použijte [dokumentaci](Dokumentace.pdf)

## Cíl projektu

Cílem projektu je vyrobit zařízení které:
- Sleduje polohu zařízení/auta pomocí GPS a posílá tyto údaje přes LoRaWAN nebo GSM na server.
- Je možno na mapě zobrazit projeté trasy s naměřenou rychlostí v bodech, ať již na serveru či lokálně na ESP.
- Možnost zjištění rychlosti akcelerace s přesností až na 200ms pomocí GPS (0-100kmph,0-60mph,1/4mile.... nastavitelné)
- Nahrání neodeslaných dat po připojení k internetu přes WiFi
- Popř. vyžádání aktuální lokace pomocí SMS zprávy a multithreading

## Použitý hardware

- ESP32 Development Board
- RA-01SH SX1276 LoRa module
- NEO-6M GPS module
- LSM6DS3 Akcelerometr module
- SD Card module, GSM module a napájecí moduly

## Použitý software a dokumentace
- LoRaWAN pro SX1276: https://github.com/beegee-tokyo/SX126x-Arduino
- TinyGPS++ pro NEO-6M: http://arduiniana.org/libraries/tinygpsplus/
- ESPAsyncWebServer pro lokání web: https://github.com/me-no-dev/ESPAsyncWebServer
- Hlavní server (Debian VPS) funguje na základě https://nodejs.org/
- spolu s základními express a socket.io knihoven, pro mapy je využit https://leafletjs.com/
- a další knihovny pro ESP\

- Byly použity návody z https://randomnerdtutorials.com/
- Dále příklady využití jednotlivých knihoven
- ArduinoJson dokumentace
- Kód z jiného projektu (https://github.com/NCPlyn/ProtogenHelmet-ESP32 & https://github.com/NCPlyn/Lidl4chan)
- Kód pro konvertování unix času (https://github.com/GyverLibs/UnixTime/blob/main/src/UnixTime.h)
- Kód pro zasunutí objektu do array (https://stackoverflow.com/questions/38418921/insert-json-object-to-ordered-array-in-javascript)
- HTTPS Wifi POST (https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFiClientSecure)
- OpenSSL (https://serverfault.com/questions/845766/generating-a-self-signed-cert-with-openssl-that-works-in-chrome-58)
- MQTT pro TTN (https://youtu.be/hxIKyFAgZCE)
- EasyEDA pro vytvoření schématu a PCB
