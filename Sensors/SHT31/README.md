# LoRaWAN Environmental Sensor — BME280 + SHT31

ESP32 firmware for a battery-powered LoRaWAN sensor node that periodically reads a BME280 (temperature, pressure, humidity) and an SHT31 (temperature, humidity), packs the readings into a binary payload, and transmits them to The Things Network on EU868. The device spends most of its life in deep sleep to maximise battery life, caches the LoRaWAN session in RTC memory so it does not have to rejoin on every wake-up, and forces a fresh OTAA join periodically to keep frame counters, keys, and ADR settings healthy.

---

## Hardware

- **MCU:** ESP32-WROOM-DA (dual-antenna variant; standard ESP32 dual-core LX6)
- **LoRa radio:** SX127x via SPI — NSS=GPIO5, RST=GPIO26, DIO0=GPIO4, DIO1=GPIO17
- **Temperature / pressure / humidity:** Bosch BME280 on I²C
- **Temperature / humidity:** Sensirion SHT31 on I²C
- **I²C bus:** SDA=GPIO21, SCL=GPIO22, 100 kHz
- **Power rails:**
  - GPIO27 (`loraRoutedSensorEnPin`) enables the shared 3 V3 rail for I²C devices.
  - GPIO16 (`BME280EnPin`) enables power for the BME280 and SHT31 (they share the same enable line on this PCB).
- **Battery monitoring:** ADC1 on GPIO34 with 11 dB attenuation

## Dependencies

- [MCCI LoRaWAN LMIC](https://github.com/mcci-catena/arduino-lmic)
- [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
- [Adafruit SHT31 Library](https://github.com/adafruit/Adafruit_SHT31)
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)
- ESP32 Arduino core (uses `EEPROM`, `esp_task_wdt`, `esp_sleep`)

## TTN configuration

- **Region:** EU868
- **Activation:** OTAA
- **DevEUI / AppEUI / AppKey:** edit the `DEVEUI`, `APPEUI`, and `APPKEY` arrays near the top of the sketch. Note the byte order in the comments — `DEVEUI` and `APPEUI` are **LSB**, `APPKEY` is **MSB**.
- **Frame-counter checks:** leave **enabled**. The firmware handles counter persistence and rejoin on its own.

---

## What the firmware does

### Boot-to-sleep flow

1. **Boot.** Configure GPIOs, start the task watchdog (120 s), measure battery voltage on GPIO34.
2. **Probe sensors.** Power up the I²C rail and check whether the BME280 (0x76/0x77) and SHT31 (0x44/0x45) respond. Set availability flags.
3. **Battery safety check.** If the battery is below 20% (`batterylvl < 2000`), skip everything and deep-sleep for 24 hours.
4. **Restore LoRa session from RTC memory.** If a previous OTAA session is cached and the rejoin threshold has not been reached, reuse it. Otherwise fall through to OTAA join.
5. **Read sensors and build the payload.** BME280 → SHT31 → boot counter → battery percent → raw battery ADC. See [Payload format](#payload-format).
6. **Transmit via LMIC.** Either OTAA-join (first boot, or after a forced rejoin) or send straight away with the cached session keys.
7. **On `EV_TXCOMPLETE`:** increment the uplink counter, persist the updated frame counter to RTC memory, decide the next sleep interval based on battery level, and call `esp_deep_sleep_start()`.
8. **Wake-up resets the chip.** Execution starts again at step 1, with RTC memory intact.

The boot counter lives in flash (via the Arduino `EEPROM` library, which is backed by ESP32 NVS). Everything else session-related lives in RTC memory and is wiped on hard reset, brown-out, or power loss.

### Session persistence

A struct in RTC memory (`RTC_DATA_ATTR`) survives `esp_deep_sleep_start()` and stores:

- `magic` — sentinel used to detect uninitialised memory
- `netid`, `devaddr`
- `nwkKey` (NwkSKey) and `artKey` (AppSKey)
- `seqnoUp`, `seqnoDn` — LoRaWAN frame counters
- `uplinkCount` — how many uplinks since the last successful OTAA join

`saveLmicSession()` is called on `EV_JOINED` and again on every `EV_TXCOMPLETE` so the frame counter is persisted before the chip sleeps. `restoreLmicSession()` is called once per boot, after `LMIC_reset()` and before the first `do_send()`. It re-applies the EU868 channel plan (channels 0–7, RX2 on 869.525 MHz / SF9) and reinstates the cached counters.

### Periodic forced rejoin

After `REJOIN_AFTER_UPLINKS` successful uplinks (default 150), the cached session is invalidated and the next boot performs a fresh OTAA join. This protects against:

- 16-bit FCntUp wrap on LoRaWAN 1.0.x
- Silent session loss on the network side (gateway outage, backend reset)
- Stale ADR settings that no longer match the link

A JoinRequest is sent at SF12 and costs roughly 1.5 s of airtime, so the threshold has to be balanced against TTN's 30 s/device/day fair-use budget. 150 is conservative at the 5-minute cadence; 1000–2000 is more typical for production.

### BME280 forced mode

The BME280's humidity reading is temperature-compensated against the chip's own die temperature. In NORMAL (continuous) mode the die self-heats, which makes the reported humidity drift low. `readAndPackBME280()`:

1. Calls `bme.setSampling()` to switch the sensor into **MODE_FORCED** with x1 oversampling on all channels and the IIR filter off. In this mode the BME280 performs one measurement on demand and then sleeps.
2. Waits **2 seconds** after powering up the rail so the polymer humidity element can equilibrate with ambient air.
3. Triggers a forced measurement, discards the result, triggers a second forced measurement, and reads the values out.

The cycle takes about 2 s longer than a plain continuous-mode read, which is irrelevant compared to a multi-minute sleep interval.

### Adaptive sleep schedule

The sleep interval is chosen from the measured battery level at the end of every cycle:

| Battery percent | Sleep interval |
|-----------------|----------------|
| > 90%           | 10 minutes     |
| > 70%           | 20 minutes     |
| > 40%           | 1 hour         |
| > 20%           | 2 hours        |
| ≤ 20%           | 24 hours (low-power guard) |

`esp_deep_sleep_start()` is always used between transmissions, so RTC memory and the cached session are preserved across wake-ups.

---

## Payload format

18 bytes, big endian:

| Offset | Bytes | Field                | Encoding        | Notes                       |
|-------:|------:|----------------------|-----------------|-----------------------------|
| 0      | 2     | BME280 temperature   | int16, ×100     | °C                          |
| 2      | 2     | BME280 pressure      | uint16, Pa / 10 | divide by 100 to get hPa    |
| 4      | 2     | BME280 humidity      | uint16, ×100    | %RH                         |
| 6      | 2     | SHT31 temperature    | int16, ×100     | °C                          |
| 8      | 2     | SHT31 humidity       | uint16, ×100    | %RH                         |
| 10     | 4     | Boot counter         | uint32          | persisted in EEPROM         |
| 14     | 2     | Battery percent      | uint16          | 0–100                       |
| 16     | 2     | Battery raw ADC      | uint16          | raw value from GPIO34       |

If a sensor is not detected at boot, its slots are filled with zeros so the payload length stays constant.

---

## Configuration

The most useful knobs are near the top of the sketch:

- `debug` — when `true`, sleep is overridden to 6 s and the device transmits continuously with no deep sleep. Useful for bench testing; **leaves the radio active and drains the battery quickly**.
- `WDT_TIMEOUT` — task watchdog timeout in seconds (default 120).
- `REJOIN_AFTER_UPLINKS` — how often to force a fresh OTAA join (default 150).
- `DEVEUI`, `APPEUI`, `APPKEY` — TTN credentials.

---

## Operational notes

- **First wake after flashing always joins.** Both the OTAA session and the boot counter start fresh on a clean power-up because RTC memory is wiped by hardware reset.
- **Boot counter wear is not a concern.** The ESP32's SPI flash is rated for ~100,000 erase cycles per sector, and NVS wear-levels writes across the partition. At one 4-byte write per 10 minutes, expect roughly 380 years of headroom before NVS wears out — battery, sensor drift, and solder joints will all fail long before that.
- **`debug = true` keeps the radio on continuously** and disables deep sleep. Use only on the bench.
- **`Wire.setClock(100000)` is intentional.** Higher I²C clocks have been observed to cause occasional NACKs on long sensor cables.
- **BME280 vs SHT31 humidity gap.** Even with forced mode and a 2 s warm-up, a residual offset of a few %RH is normal — the BME280 is spec'd at ±3 %RH and the SHT31 at ~±2 %RH. If a larger gap persists, a per-device humidity offset against the SHT31 (the more accurate of the two) is the practical fix.
