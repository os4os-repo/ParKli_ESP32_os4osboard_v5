# Documentation of the os4os ESP32 V5 board

## Overview
This repository contains the firmware, payload decoder and minimal documentation for the ParKli os4os ESP32 V5 board. This board is a flexible IoT platform designed for various sensor applications, including environmental monitoring in buildings, agriculture, or infrastructure. An onboard BME280 measures air temperature, pressure, humidity inside the enclosure. Data is sent via LoRa (TTN/OTAA). In addition up to five analoge sensors and one didital sensor can be connected to the board.

## Further information
https://datahub.openscience.eu/dataset/parkli-boje <br>
https://parkli.de/

## Repository layout
- [src_code/main.ino](src_code/main.ino) — Main ESP32 Arduino sketch with LMIC LoRaWAN integration; key functions: [`do_send`](src_code/main.ino), [`refreshSensorData`](src_code/main.ino), [`setSleepTime`](src_code/main.ino), [`getCellLvlPercent`](src_code/main.ino), [`getBootCycle`](src_code/main.ino), [`onEvent`](src_code/main.ino), [`checkTxIntervalWatchdog`](src_code/main.ino), [`enforceBackoffLimit`](src_code/main.ino).
- [src_code/archive/](src_code/archive/) — Superseded versions of the sketch, kept for reference (see "Archived versions" below).
- [TTN_Decoder/decoder.txt](TTN_Decoder/decoder.txt) — TTN payload formatter / decoder used on The Things Network; exposes [`decodeUplink`](TTN_Decoder/decoder.txt), [`decodeSigned16Bit`](TTN_Decoder/decoder.txt), [`moveComma`](TTN_Decoder/decoder.txt).
- [Documentation/](Documentation/) — Additional documentation and notes.
- [Images/](Images/) — Project images and GIFs used in this README.

## Archived versions
`src_code/` only ever contains one buildable sketch: `main.ino`, always the current version. Superseded versions are not left in `src_code/` under ambiguous names (e.g. `main_old.ino`) because that makes it unclear which file is meant to be compiled/flashed, and the "old" one silently goes stale as `main.ino` moves on. Instead, retired versions are moved into [src_code/archive/](src_code/archive/) and renamed to `main_<YYYY-MM-DD>_<short-description>.ino`, where the date and description identify what the sketch looked like and why it was superseded. Archived files are kept for reference only and are not meant to be compiled as-is.

- [src_code/archive/main_2026-03-05_otaa-join.ino](src_code/archive/main_2026-03-05_otaa-join.ino) — version prior to LMIC session persistence: performs a fresh OTAA join on every wake from deep sleep and still uses `FILLMEIN` placeholders for OTAA credentials. See "Changes vs. archived versions" below.

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

## Changes vs. archived versions
[src_code/archive/main_2026-03-05_otaa-join.ino](src_code/archive/main_2026-03-05_otaa-join.ino) is the previous version of the sketch (archived, see "Archived versions" above). It performs a fresh OTAA join on every wake from deep sleep. `main.ino` adds LMIC session persistence across deep sleep so the device does not need to rejoin (and burn airtime) on every wake cycle:

- **Persisted session in RTC memory**: a new `RTC_DATA_ATTR` struct `lmicSession` (magic marker, `netid`, `devaddr`, `nwkKey`, `artKey`, `seqnoUp`, `seqnoDn`, `uplinkCount`) survives deep sleep (but not power loss/reset) and stores the OTAA session state.
- **`saveLmicSession()`** (new function): snapshots the current LMIC session keys and frame counters into `lmicSession` and prints them to serial. Called after a successful `EV_JOINED` and after every `EV_TXCOMPLETE`.
- **`restoreLmicSession()`** (new function): called once from `setup()` after `LMIC_reset()`. If a valid saved session exists, it restores it via `LMIC_setSession()`, re-applies the EU868 channel plan (channels 0–7, since `LMIC_setSession()` clears the join state) and the saved frame counters, and returns `true` so `setup()` skips the OTAA join. Returns `false` (forcing a fresh join) when there is no valid saved session yet.
- **Periodic forced rejoin**: `REJOIN_AFTER_UPLINKS` (500 uplinks) caps how long a session is reused before `restoreLmicSession()` forces a fresh OTAA join, to heal frame-counter drift, refresh session keys, and let ADR re-tune from scratch. The counter (`lmicSession.uplinkCount`) is reset to 0 on `EV_JOINED` and incremented on every `EV_TXCOMPLETE`.
- **Hardcoded OTAA keys**: `main.ino` ships with example `APPEUI`/`DEVEUI`/`APPKEY` values already filled in, whereas the archived version still uses the `FILLMEIN` placeholders described in "Build / flash" step 3.

All sensor reading, payload packing, sleep-time, and watchdog/backoff logic is unchanged between the current and archived version.

## License and credits
Copyright 2025 os4os
This project is licensed under the MIT License — see the included LICENSE file for details.

## Contact
Use repository issues for questions or debugging.

![Sensor Image](Images/SensorOS4OS.gif)
