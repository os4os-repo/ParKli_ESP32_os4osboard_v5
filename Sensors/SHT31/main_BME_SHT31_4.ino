#include <Arduino.h>
#include <EEPROM.h>  //https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SHT31.h>
#include <esp_task_wdt.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
Adafruit_SHT31 sht31 = Adafruit_SHT31();

// --- LoRa / SX127x pin map ---
#define loraNSS 5
#define loraRST 26
#define loraDOI0 4
#define loraDOI1 17

// --- I2C bus ---
#define SCL_PIN 22
#define SDA_PIN 21

// --- Misc ---
#define ledPin 13
#define vBatPin 34

// Power-rail enables.
// loraRoutedSensorEnPin (GPIO27) still powers the I2C rail used by BME280/SHT31,
// so it must stay even though the routed analog sensors are gone.
#define loraRoutedSensorEnPin 27
#define BME280EnPin 16

static uint32_t cycleStartMs = 0;

// --- Sleep / interval handling ---
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
uint64_t TIME_TO_SLEEP = 10ULL;   /* Overwritten by setSleepTime() */
int16_t CellLvlPercent;
uint8_t buffer[64];
size_t payloadLength = 0;
#define EEPROM_SIZE 24
uint32_t bootCount;
int16_t batterylvl;

// If true, sensor values are sent every few seconds (no deep sleep)
bool debug = false;
#define WDT_TIMEOUT 120  // Watchdog timeout in seconds
esp_err_t ESP32_ERROR;
bool bmeAvailable = false;
bool sht31Available = false;

// Forward declarations
void do_send(osjob_t *j);
void setSleepTime();
void getCellLvlPercent();
void getBootCycle();
static inline void checkTxIntervalWatchdog(const char *where);
static inline void enforceBackoffLimit(uint32_t msUntil, const char *where);
static inline void rebootNow(const char *reason);
void saveLmicSession();
bool restoreLmicSession();

// --- Persisted LMIC session in RTC memory (survives deep sleep, not power loss/reset) ---
#define LMIC_SESSION_MAGIC 0xCAFEBABE
// Force a fresh OTAA join after this many uplinks (heals frame-counter drift,
// refreshes session keys, lets ADR re-tune from scratch).
// Note: TTN community network fair-use is 30s of airtime/device/day, and a
// JoinRequest is sent at SF12 (~1.5s airtime). Keep this comfortably high.
// 150 = roughly twice per day at the 5-min cadence; 1000–2000 is more typical.
#define REJOIN_AFTER_UPLINKS 150UL
RTC_DATA_ATTR struct {
  uint32_t  magic;
  u4_t      netid;
  devaddr_t devaddr;
  u1_t      nwkKey[16];
  u1_t      artKey[16];
  u4_t      seqnoUp;
  u4_t      seqnoDn;
  uint32_t  uplinkCount;   // cycles since last successful OTAA join
} lmicSession;

#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LSB
static const u1_t PROGMEM APPEUI[8] = { FILLMEIN };
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// LSB
static const u1_t PROGMEM DEVEUI[8] = { FILLMEIN };
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// MSB
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL = 1;

const lmic_pinmap lmic_pins = {
  .nss = 5,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 26,
  .dio = { 4, 17, LMIC_UNUSED_PIN },
};

void printHex(unsigned v) {
  v &= 0xff;
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:    Serial.println(F("EV_SCAN_TIMEOUT")); break;
    case EV_BEACON_FOUND:    Serial.println(F("EV_BEACON_FOUND")); break;
    case EV_BEACON_MISSED:   Serial.println(F("EV_BEACON_MISSED")); break;
    case EV_BEACON_TRACKED:  Serial.println(F("EV_BEACON_TRACKED")); break;
    case EV_JOINING:         Serial.println(F("EV_JOINING")); break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: "); Serial.println(netid, DEC);
        Serial.print("devaddr: "); Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0) Serial.print("-");
          printHex(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0) Serial.print("-");
          printHex(nwkKey[i]);
        }
        Serial.println();
      }
      LMIC_setLinkCheckMode(0);
      lmicSession.uplinkCount = 0;   // reset on fresh join
      saveLmicSession();
      Serial.println(F("LMIC session saved to RTC memory"));
      break;
    case EV_JOIN_FAILED:   Serial.println(F("EV_JOIN_FAILED")); break;
    case EV_REJOIN_FAILED: Serial.println(F("EV_REJOIN_FAILED")); break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK) Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Count this successful uplink and persist updated frame counters
      // before sleeping/scheduling next TX
      lmicSession.uplinkCount++;
      Serial.print(F("Uplink count since last join: "));
      Serial.print(lmicSession.uplinkCount);
      Serial.print(F(" / "));
      Serial.println(REJOIN_AFTER_UPLINKS);
      saveLmicSession();
      // Schedule next transmission
      setSleepTime();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");

      // Only enter Deep Sleep mode if esp goes to sleep more than 60sec
      if (TX_INTERVAL == 1) {
        Serial.println("Goes into Deep Sleep mode");
        Serial.println("----------------------");
        delay(100);
        esp_deep_sleep_start();
        Serial.println("This will never be displayed");
      }
      // If sleep time is below 60 sec, schedule TX_INTERVAL without resetting LORA session to minimize data usage.
      esp_task_wdt_reset();
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:   Serial.println(F("EV_LOST_TSYNC")); break;
    case EV_RESET:        Serial.println(F("EV_RESET")); break;
    case EV_RXCOMPLETE:   Serial.println(F("EV_RXCOMPLETE")); break;
    case EV_LINK_DEAD:    Serial.println(F("EV_LINK_DEAD")); break;
    case EV_LINK_ALIVE:   Serial.println(F("EV_LINK_ALIVE")); break;
    case EV_TXSTART:      Serial.println(F("EV_TXSTART")); break;
    case EV_TXCANCELED:   Serial.println(F("EV_TXCANCELED")); break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      setSleepTime();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");
      Serial.println("Goes into Deep Sleep mode");
      Serial.println("----------------------");
      delay(100);
      esp_deep_sleep_start();
      Serial.println("This will never be displayed");
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

/**
 * Writes a 16-bit unsigned value into the payload buffer (big endian).
 */
static inline bool put_u16(uint8_t *buf, size_t *payloadLength, size_t max, uint16_t value) {
  if (*payloadLength + 2 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;
  }
  buf[(*payloadLength)++] = (uint8_t)(value >> 8);
  buf[(*payloadLength)++] = (uint8_t)(value);
  return true;
}

/**
 * Writes a 16-bit signed value into the payload buffer (big endian).
 */
static inline bool put_i16(uint8_t *buf, size_t *payloadLength, size_t max, int16_t value) {
  if (*payloadLength + 2 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;
  }
  buf[(*payloadLength)++] = (uint8_t)(value >> 8);
  buf[(*payloadLength)++] = (uint8_t)value;
  return true;
}

/**
 * Writes a 32-bit unsigned value into the payload buffer (big endian).
 */
static inline bool put_u32(uint8_t *buf, size_t *payloadLength, size_t max, uint32_t value) {
  if (*payloadLength + 4 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;
  }
  buf[(*payloadLength)++] = (uint8_t)(value >> 24);
  buf[(*payloadLength)++] = (uint8_t)(value >> 16);
  buf[(*payloadLength)++] = (uint8_t)(value >> 8);
  buf[(*payloadLength)++] = (uint8_t)(value);
  return true;
}

/**
 * Reads BME280 sensor values and packs them into the payload buffer.
 * Temperature (°C × 100) is signed, Pressure (Pa / 10) and Humidity (% × 100) are unsigned.
 */
void readAndPackBME280(Adafruit_BME280 &bme, uint8_t *buffer, size_t *payloadLength, size_t max) {
  Serial.println("==============================================");

  digitalWrite(loraRoutedSensorEnPin, HIGH);  // enable I2C rail
  digitalWrite(BME280EnPin, HIGH);            // power BME
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Re-init after power-up
  bool ok = bme.begin(0x76);
  if (!ok) ok = bme.begin(0x77);
  if (!ok) {
    Serial.println("BME280 init failed after power-up!");
    // Pack zeros to keep payload length fixed
    put_i16(buffer, payloadLength, max, 0);
    put_u16(buffer, payloadLength, max, 0);
    put_u16(buffer, payloadLength, max, 0);
    digitalWrite(BME280EnPin, LOW);
    digitalWrite(loraRoutedSensorEnPin, LOW);
    Serial.println("==============================================");
    return;
  }

  // Throw away first measurement after power-up	 
  (void)bme.readTemperature();				  						  
  delay(10);

  int16_t sensTemperature = int16_t(bme.readTemperature() * 100.0F);
  uint32_t rawPressure = bme.readPressure();           // Pa
  uint16_t sensPressure = uint16_t(rawPressure / 10);  // Pa/10
  uint16_t sensHumidity = uint16_t(bme.readHumidity() * 100.0F);

  put_i16(buffer, payloadLength, max, sensTemperature);
  put_u16(buffer, payloadLength, max, sensPressure);
  put_u16(buffer, payloadLength, max, sensHumidity);

  Serial.print("BME T="); Serial.print(sensTemperature / 100.0F);
  Serial.print("C P="); Serial.print(sensPressure / 100.0F);
  Serial.print("hPa H="); Serial.print(sensHumidity / 100.0F);
  Serial.println("%");

  digitalWrite(BME280EnPin, LOW);
  Serial.println("==============================================");
}

/**
 * Reads SHT31 sensor values and packs them into the payload buffer.
 * Temperature (°C × 100) is signed, Humidity (% × 100) is unsigned.
 */
void readAndPackSHT31(Adafruit_SHT31 &sht, uint8_t *buffer, size_t *payloadLength, size_t max) {
  Serial.println("==============================================");

  digitalWrite(loraRoutedSensorEnPin, HIGH);  // enable I2C rail
  digitalWrite(BME280EnPin, HIGH);            // power SHT31 (shares enable line)
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bool ok = sht.begin(0x44);
  if (!ok) ok = sht.begin(0x45);
  if (!ok) {
    Serial.println("SHT31 init failed after power-up!");
    put_i16(buffer, payloadLength, max, 0);
    put_u16(buffer, payloadLength, max, 0);
    digitalWrite(BME280EnPin, LOW);
    digitalWrite(loraRoutedSensorEnPin, LOW);
    Serial.println("==============================================");
    return;
  }

  // Throw away first measurement after power-up
  (void)sht.readTemperature();
  delay(10);

  int16_t sensTemperature = int16_t(sht.readTemperature() * 100.0F);
  uint16_t sensHumidity = uint16_t(sht.readHumidity() * 100.0F);

  put_i16(buffer, payloadLength, max, sensTemperature);
  put_u16(buffer, payloadLength, max, sensHumidity);

  Serial.print("SHT31 T="); Serial.print(sensTemperature / 100.0F);
  Serial.print("C H="); Serial.print(sensHumidity / 100.0F);
  Serial.println("%");

  digitalWrite(BME280EnPin, LOW);
  Serial.println("==============================================");
}

void refreshSensorData() {
  payloadLength = 0;

  if (bmeAvailable) {
    readAndPackBME280(bme, buffer, &payloadLength, sizeof(buffer));
  } else {
    // Keep payload length fixed even if sensor is missing at boot
    put_i16(buffer, &payloadLength, sizeof(buffer), 0);
    put_u16(buffer, &payloadLength, sizeof(buffer), 0);
    put_u16(buffer, &payloadLength, sizeof(buffer), 0);
  }

  if (sht31Available) {
    readAndPackSHT31(sht31, buffer, &payloadLength, sizeof(buffer));
  } else {
    put_i16(buffer, &payloadLength, sizeof(buffer), 0);
    put_u16(buffer, &payloadLength, sizeof(buffer), 0);
  }

  getBootCycle();
  put_u32(buffer, &payloadLength, sizeof(buffer), bootCount);

  getCellLvlPercent();
  put_u16(buffer, &payloadLength, sizeof(buffer), CellLvlPercent);
  put_u16(buffer, &payloadLength, sizeof(buffer), batterylvl);
}

void do_send(osjob_t *j) {
  cycleStartMs = millis();
  refreshSensorData();
  Serial.println(F("refresh sensor data completed..."));
  digitalWrite(loraRoutedSensorEnPin, HIGH);

  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    Serial.print("payloadLength=");
    Serial.println(payloadLength);
    for (size_t i = 0; i < payloadLength; i++) {
      if (i % 16 == 0) Serial.println();
      if (buffer[i] < 16) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    LMIC_setTxData2(1, buffer, payloadLength, 0);

    // Show when we are actually allowed to transmit again
    ostime_t now = os_getTime();
    ostime_t next = LMIC.txend;
    ostime_t dt_ticks = next - now;
    if (dt_ticks < 0) dt_ticks = 0;

    uint32_t msUntil = osticks2ms(dt_ticks);
    Serial.print("Next possible TX in [ms]: ");
    Serial.println(msUntil);
    enforceBackoffLimit(msUntil, "do_send()");

    checkTxIntervalWatchdog("after LMIC_setTxData2");

    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Serial.println("Configuring WDT...");
  Serial.print("Watchdog Timeout (in seconds) set to : ");
  Serial.println(WDT_TIMEOUT);
  esp_task_wdt_deinit();
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true
  };
  ESP32_ERROR = esp_task_wdt_init(&wdt_config);
  Serial.println("Last Reset : " + String(esp_err_to_name(ESP32_ERROR)));
  esp_task_wdt_add(NULL);

  // --- GPIO configuration ---
  pinMode(BME280EnPin, OUTPUT);
  digitalWrite(BME280EnPin, LOW);
  pinMode(loraRoutedSensorEnPin, OUTPUT);
  digitalWrite(loraRoutedSensorEnPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(vBatPin, INPUT);
  analogSetPinAttenuation(vBatPin, ADC_11db);

  delay(100);
  batterylvl = analogRead(vBatPin);
  delay(100);
  Serial.print("batterylvl: ");
  Serial.println(batterylvl);

  // Probe I2C sensors once at boot to set availability flags
  digitalWrite(loraRoutedSensorEnPin, HIGH);
  digitalWrite(BME280EnPin, HIGH);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bmeAvailable = bme.begin(0x76);
  if (!bmeAvailable) bmeAvailable = bme.begin(0x77);
  Serial.println(bmeAvailable ? "BME280 found" : "BME280 NOT found!");

  sht31Available = sht31.begin(0x44);
  if (!sht31Available) sht31Available = sht31.begin(0x45);
  Serial.println(sht31Available ? "SHT31 found" : "SHT31 NOT found!");

  digitalWrite(BME280EnPin, LOW);
  digitalWrite(loraRoutedSensorEnPin, LOW);

  if (!debug) {
    // Avoid deadlock by going into long deep sleep if battery is critically low
    getCellLvlPercent();
    if (CellLvlPercent < 20) {
      TIME_TO_SLEEP = 24 * 3600ULL;  // 24h
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");
      Serial.println("Goes into Deep Sleep mode");
      Serial.println("----------------------");
      delay(100);
      esp_deep_sleep_start();
      Serial.println("This will never be displayed");
    }
  }

  // LMIC init
  os_init();
  LMIC_reset();
  // LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);  // 1% — required on ESP32
  if (restoreLmicSession()) {
    Serial.println(F("LMIC session restored from RTC memory — skipping OTAA join"));
  } else {
    Serial.println(F("No valid saved session — will perform OTAA join"));
  }
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void setSleepTime() {
  getCellLvlPercent();
  // Default Sleep Time (24 hours) — safety fallback
  TIME_TO_SLEEP = 24 * 3600ULL;

  if (CellLvlPercent > 90) {
    TIME_TO_SLEEP = 600ULL;       // 10 minutes
  } else if (CellLvlPercent > 70) {
    TIME_TO_SLEEP = 1200ULL;      // 20 minutes
  } else if (CellLvlPercent > 40) {
    TIME_TO_SLEEP = 3600ULL;      // 1 hour
  } else if (CellLvlPercent > 20) {
    TIME_TO_SLEEP = 2 * 3600ULL;  // 2 hours
  }
  // Always enter deep sleep after EV_TXCOMPLETE so RTC memory (and the
  // saved LMIC session) is preserved. The os_setTimedCallback path
  // (TX_INTERVAL > 1) would keep the CPU awake and trip the task WDT.
  TX_INTERVAL = 1ULL;

  Serial.print("TIME_TO_SLEEP: ");
  Serial.println(TIME_TO_SLEEP);
  if (debug) {
    TIME_TO_SLEEP = 6ULL;
    Serial.print("Debug=true; overwrite TIME_TO_SLEEP: ");
    Serial.println(TIME_TO_SLEEP);
  }
}

void getCellLvlPercent() {
  const int maxOperatingVoltage = 2400;
  const int minOperatingVoltage = 2000;  // High threshold because DeepSleep current rises a lot below 2000
  const float operationalRange = maxOperatingVoltage - minOperatingVoltage;

  if (batterylvl <= minOperatingVoltage) {
    CellLvlPercent = 0;
  } else if (batterylvl > maxOperatingVoltage) {
    CellLvlPercent = 100;
  } else {
    CellLvlPercent = ((batterylvl - minOperatingVoltage) / operationalRange) * 100;
  }

  Serial.print("CellLvlPercent: ");
  Serial.println(CellLvlPercent);
  Serial.print("batterylvl: ");
  Serial.println(batterylvl);
}

void getBootCycle() {
  EEPROM.begin(EEPROM_SIZE);

  const int addressBootCycle = 0;
  const int addressInitCode = 10;
  const uint32_t expectedCode = 123456789;

  uint32_t storedInitCode;
  EEPROM.get(addressInitCode, storedInitCode);

  Serial.print("Stored Init-Code: ");
  Serial.println(storedInitCode);

  if (storedInitCode != expectedCode) {
    Serial.println("Init-Code not found, BootCycle reset to 1.");
    EEPROM.writeUInt(addressInitCode, expectedCode);
    EEPROM.writeUInt(addressBootCycle, 1);
    EEPROM.commit();
    bootCount = 1;
  } else {
    Serial.println("Incrementing BootCycle.");
    uint32_t currentBootCycle;
    EEPROM.get(addressBootCycle, currentBootCycle);
    Serial.print("Current Boot-Cycle: ");
    Serial.println(currentBootCycle);
    uint32_t newBootCycle = currentBootCycle + 1;
    EEPROM.writeUInt(addressBootCycle, newBootCycle);
    EEPROM.commit();
    bootCount = newBootCycle;
  }

  EEPROM.end();
}

static inline void checkTxIntervalWatchdog(const char *where) {
  const uint32_t now = millis();
  // Compare with subtraction to survive millis() wrap
  if (cycleStartMs && (now - cycleStartMs) > ((TIME_TO_SLEEP * TX_INTERVAL) * 1000UL)) {
    Serial.print("SW-WDT ");
    Serial.print(where);
    Serial.print(" elapsed(ms)=");
    Serial.print(now - cycleStartMs);
    Serial.print(" > TX_INTERVAL(ms)=");
    Serial.println(TIME_TO_SLEEP * TX_INTERVAL * uS_TO_S_FACTOR);
    rebootNow("Rebooting: TX cycle exceeded TIME_TO_SLEEP * TX_INTERVAL");
  }
}

static inline void enforceBackoffLimit(uint32_t msUntil, const char *where) {
  const uint32_t limitMs = 30 * 1000;  // TODO check if correct
  if (msUntil > limitMs) {
    Serial.print("Backoff too large at ");
    Serial.print(where);
    Serial.print(": msUntil=");
    Serial.print(msUntil);
    Serial.print(" > limit=");
    Serial.println(limitMs);
    Serial.println("start deep sleep and try again afterwards");
    delay(100);
    TIME_TO_SLEEP = 2 * 3600ULL;  // 2 hours
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");
    Serial.println("Goes into Deep Sleep mode");
    Serial.println("----------------------");
    delay(100);
    esp_deep_sleep_start();
    Serial.println("This will never be displayed");
  }
}

static inline void rebootNow(const char *reason) {
  Serial.println(reason);
  delay(100);
  esp_restart();
}

// --- LMIC session persistence (RTC memory, survives deep sleep) ---

void saveLmicSession() {
  lmicSession.magic = LMIC_SESSION_MAGIC;
  LMIC_getSessionKeys(&lmicSession.netid, &lmicSession.devaddr,
                      lmicSession.nwkKey, lmicSession.artKey);
  lmicSession.seqnoUp = LMIC.seqnoUp;
  lmicSession.seqnoDn = LMIC.seqnoDn;
  Serial.print(F("Session saved. devaddr="));
  Serial.print(lmicSession.devaddr, HEX);
  Serial.print(F(" seqnoUp="));
  Serial.print(lmicSession.seqnoUp);
  Serial.print(F(" seqnoDn="));
  Serial.println(lmicSession.seqnoDn);
}

bool restoreLmicSession() {
  if (lmicSession.magic != LMIC_SESSION_MAGIC) {
    return false;
  }

  // Force a periodic OTAA rejoin so frame counters, keys, and ADR get refreshed.
  if (lmicSession.uplinkCount >= REJOIN_AFTER_UPLINKS) {
    Serial.print(F("Rejoin threshold reached ("));
    Serial.print(lmicSession.uplinkCount);
    Serial.print(F(" >= "));
    Serial.print(REJOIN_AFTER_UPLINKS);
    Serial.println(F(") — forcing OTAA rejoin"));
    lmicSession.magic = 0;  // invalidate; will be repopulated on EV_JOINED
    return false;
  }

  // Restore keys + devaddr (this clears the join state internally, so we must
  // re-apply channels and DR/TXpow afterwards on EU868).
  LMIC_setSession(lmicSession.netid, lmicSession.devaddr,
                  lmicSession.nwkKey, lmicSession.artKey);

  // --- EU868 channel plan (3 mandatory + 5 optional CFList channels) ---
  // Channels 0..2 are mandatory on EU868. Channels 3..7 are typically
  // assigned by TTN via the JoinAccept CFList — re-add them so behaviour
  // matches the original session.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // RX2 on EU868 is 869.525 MHz / SF9 — LMIC's default is fine, but be explicit:
  LMIC.dn2Dr = DR_SF9;

  LMIC.seqnoUp = lmicSession.seqnoUp;
  LMIC.seqnoDn = lmicSession.seqnoDn;

  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7, 14);

  Serial.print(F("Session restored. devaddr="));
  Serial.print(lmicSession.devaddr, HEX);
  Serial.print(F(" seqnoUp="));
  Serial.print(lmicSession.seqnoUp);
  Serial.print(F(" seqnoDn="));
  Serial.println(lmicSession.seqnoDn);
  return true;
}
