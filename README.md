# CarStatsTracker
Závěrečná práce

## Cíl projektu

Cílem projektu je vyrobit zařízení které:
- Sleduje polohu zařízení/auta pomocí GPS a posílá tyto údaje přes LoRaWAN na server.
- Je možno na mapě zobrazit projeté trasy s naměřenou rychlostí v bodech, ať již na serveru či lokálně na ESP
- Možnost zjištění rychlosti akcelerace s přesností až na 200ms pomocí GPS (0-100kmph,0-60mph,1/4mile.... nastavitelné)
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
- a další menší knihovny pro ESP
- Někdy byly použity návody z https://randomnerdtutorials.com/
- Hlavní server funguje na základě https://nodejs.org/ a základních podpůrných express a socket.io knihoven na Debian VPS
