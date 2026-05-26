# Documentation of the os4os ESP32 V5 board

## Overview
This repository contains the firmware, payload decoder and minimal documentation for the ParKli os4os ESP32 V5 board. This board is a flexible IoT platform designed for various sensor applications, including environmental monitoring in buildings, agriculture, or infrastructure. An onboard BME280 measures air temperature, pressure, humidity inside the enclosure. Data is sent via LoRa (TTN/OTAA). In addition up to five analoge sensors and one didital sensor can be connected to the board.

## Further information
https://datahub.openscience.eu/dataset/parkli-boje <br>
https://parkli.de/

## Repository layout
- [src_code/main.ino](src_code/main.ino) — Main ESP32 Arduino sketch with LMIC LoRaWAN integration; key functions: [`do_send`](src_code/main.ino), [`refreshSensorData`](src_code/main.ino), [`setSleepTime`](src_code/main.ino), [`getCellLvlPercent`](src_code/main.ino), [`getBootCycle`](src_code/main.ino), [`onEvent`](src_code/main.ino), [`checkTxIntervalWatchdog`](src_code/main.ino), [`enforceBackoffLimit`](src_code/main.ino).
- [TTN_Decoder/decoder.txt](TTN_Decoder/decoder.txt) — TTN payload formatter / decoder used on The Things Network; exposes [`decodeUplink`](TTN_Decoder/decoder.txt), [`decodeSigned16Bit`](TTN_Decoder/decoder.txt), [`moveComma`](TTN_Decoder/decoder.txt).
- [Documentation/](Documentation/) — Additional documentation and notes.
- [Images/](Images/) — Project images and GIFs used in this README.

## Firmware summary
- Power management: measures battery via ADC, computes percent in [`getCellLvlPercent`](src_code/main.ino), and gates deep-sleep duration via [`setSleepTime`](src_code/main.ino).
- Sensors:
  - BME280 via I2C (address probing 0x76 / 0x77).
  - Additional digital senor via I2C (address probing 0x76 / 0x77).
  - Analog sensors to connect for example TDS (conductivity), pH, ulrtasonics distance sensors, DS18B20 DallasTemperature, etc.
- Transmission:
  - OTAA join with LMIC; join and TX event handling in [`onEvent`](src_code/main.ino).
  - Data packed into a payload inside `buffer` and sent from [`do_send`](src_code/main.ino).
  - Watchdog/backoff safety: [`checkTxIntervalWatchdog`](src_code/main.ino) and [`enforceBackoffLimit`](src_code/main.ino).

## TTN Payload decoder
Use the TTN JavaScript decoder in [TTN_Decoder/decoder.txt](TTN_Decoder/decoder.txt). It maps raw bytes to:
- sensor0 to sensor7 (each with pin2 and pin3 values for analog sensors)
- sensTemperature (Air_Temperature, in °C with two decimal places)
- sensPressure (Pressure in hPa)
- sensHumidity (Humidity, scaled by 1/100 in %)
- bootCycle (boot counter)
- cellLvlPercent (battery percent)
- batteryRAW (battery raw value)

## Build / flash
1. Open [src_code/main.ino](src_code/main.ino) in the Arduino IDE or PlatformIO (ESP32 board).
2. Install required libraries: LMIC, DallasTemperature, OneWire, Adafruit_BME280, Adafruit_Sensor (and ESP32 core).
3. Update OTAA credentials in the sketch by replacing the `FILLMEIN` placeholders with real values from the TTN control panel (APPEUI / DEVEUI / APPKEY).
4. Compile and flash to the ESP32.

## Notes & troubleshooting
- If battery percent is low, the device enforces long deep-sleep.
- Large LoRa backoff forces a longer sleep via [`enforceBackoffLimit`](src_code/main.ino).
- Serial prints are available at 115200 baud for debugging.

## License and credits
Copyright 2025 os4os
This project is licensed under the MIT License — see the included LICENSE file for details.

## Contact
Use repository issues for questions or debugging.

![Sensor Image](Images/SensorOS4OS.gif)
