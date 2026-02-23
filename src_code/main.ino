#include <Arduino.h>
#include <EEPROM.h>  //https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_task_wdt.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
/*
userButtonPin --> Button was removed
#define humTempOptEnPin 32
#define tempPin 14
#define optSensorPin 33

#define phEnPin 25
#define phPin 36

#define loraTdsEnPin 27
#define tdsPin 39

*/
#define loraNSS 5
#define loraRST 26
#define loraDOI0 4
#define loraDOI1 17

#define SCL_PIN 22
#define SDA_PIN 21

#define ledPin 13
#define vBatPin 34

#define routedSensorPin3 32       //Screw terminal 1-8 is wired like: 3v3|33|32|GND
#define routedSensorPin2 33       //Screw terminal 1-8 is wired like: 3v3|33|32|GND
#define loraRoutedSensorEnPin 27  //The Sensor Routing, Lora Module and I2C Sensor is only active if set to HIGH
#define BME280EnPin 16

#define routedSensorS0 25  //Based on the S0, S1, S2 -> select Screw Terminal 0-7
#define routedSensorS1 14
#define routedSensorS2 15


static uint32_t cycleStartMs = 0;

//Define interval for measurements, set by function setSleepTime() or absolut
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
uint64_t TIME_TO_SLEEP = 10ULL;   /* Time ESP32 will go to sleep (in seconds) Will be overwritten by setSleepTime() funktion*/
int16_t CellLvlPercent;
uint8_t buffer[128];
size_t payloadLength = 0;
#define EEPROM_SIZE 24
uint32_t bootCount;
int16_t batterylvl;
//if true sensor values are send every 6 sec
bool debug = false;
#define WDT_TIMEOUT 120  //Watchdog timeout in seconds
esp_err_t ESP32_ERROR;
bool bmeAvailable = false;
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
#define FILLMEIN 0
#else
#warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
#define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

void do_send(osjob_t *j);

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format lsb, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0xF7, 0x54, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };  //(lsb)

void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xEC, 0x6D, 0x81, 0x17, 0xD0, 0xAF, 0xCB, 0xF4, 0xFA, 0x08, 0x41, 0xB1, 0x15, 0x2C, 0xC5, 0x48 };  //(msb)

void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned TX_INTERVAL = 1;

const lmic_pinmap lmic_pins = {
  .nss = 5,  // chip select on (rf95module) CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 26,                          // reset pin
  .dio = { 4, 17, LMIC_UNUSED_PIN },  //G0, G1
};

void printHex(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      setSleepTime();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");

      //Only enter Deep Sleep mode if esp goes to sleep more than 60sec
      if (TX_INTERVAL == 1) {
        // Go in Deep Sleep mode
        Serial.println("Goes into Deep Sleep mode");
        Serial.println("----------------------");
        delay(100);

        esp_deep_sleep_start();
        Serial.println("This will never be displayed");
      }
      //if sleep time is below 60 sec schedule TX_Intervall without resetting LORA session to minimize data usage.
      esp_task_wdt_reset();  // Reset the watchdog timer to prevent it from triggering (wdt must be > TX_INTERVAL)
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      //if data can not be send the esp32 goes back to sleep
      setSleepTime();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");

      // Go in Deep Sleep mode
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

void do_send(osjob_t *j) {
  // Payload to send (uplink)
  // Start/renew the TX cycle timer
  cycleStartMs = millis();
  refreshSensorData();
  Serial.println(F("refresh sensor data completed..."));
  digitalWrite(loraRoutedSensorEnPin, HIGH);
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    
    Serial.print("payloadLength=");
    Serial.println(payloadLength);
    for (size_t i=0; i<payloadLength; i++){
      if (i % 16 == 0) Serial.println();
      if (buffer[i] < 16) Serial.print("0");
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, buffer, payloadLength, 0);

    // Show when we are actually allowed to transmit again
    ostime_t now = os_getTime();
    ostime_t next = LMIC.txend;

    // compute signed delta in ticks (wrap-safe with signed math)
    ostime_t dt_ticks = next - now;
    if (dt_ticks < 0) dt_ticks = 0;

    uint32_t msUntil = osticks2ms(dt_ticks);
    Serial.print("Next possible TX in [ms]: ");
    Serial.println(msUntil);
    // NEW: reboot right away if the backoff exceeds your limit
    enforceBackoffLimit(msUntil, "do_send()");

    // --------- WDT TX_INTERAVL Check start ----------
    checkTxIntervalWatchdog("after LMIC_setTxData2");

    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}


/**
 * Writes a 16 or 32-bit value into the payload buffer (big endian)
 * Prints an error if buffer overflow occurs.
 *
 * @param buf           Pointer to the payload buffer
 * @param payloadLength Pointer to the current write index in the buffer
 *                      (will be automatically incremented after writing)
 * @param max           Maximum buffer size (to prevent overflow)
 * @param value         16-bit value to write into the buffer
 * 
 * @return true if the write was successful
 *         false if there was not enough space in the buffer
 *
 * typical call: put_u16(buffer, &payloadLength, sizeof(buffer), sensors[j]);
 */
static inline bool put_u16(uint8_t *buf, size_t *payloadLength, size_t max, uint16_t value) {
  if (*payloadLength + 2 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;  // still return false for programmatic handling
  }

  buf[(*payloadLength)++] = (uint8_t)(value >> 8);  // High byte
  buf[(*payloadLength)++] = (uint8_t)(value);       // Low byte
  return true;
}
// put_i16 for signed values
static inline bool put_i16(uint8_t *buf, size_t *payloadLength, size_t max, int16_t value) {
  if (*payloadLength + 2 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;
  }

  buf[(*payloadLength)++] = (uint8_t)(value >> 8);  // High byte
  buf[(*payloadLength)++] = (uint8_t)value;         // Low byte
  return true;
}
// put_u32 for long values
static inline bool put_u32(uint8_t *buf, size_t *payloadLength, size_t max, uint32_t value) {
  if (*payloadLength + 4 > max) {
    Serial.print("Buffer overflow at index ");
    Serial.println(*payloadLength);
    return false;  // still return false for programmatic handling
  }

  buf[(*payloadLength)++] = (uint8_t)(value >> 24);  // Most significant byte
  buf[(*payloadLength)++] = (uint8_t)(value >> 16);
  buf[(*payloadLength)++] = (uint8_t)(value >> 8);
  buf[(*payloadLength)++] = (uint8_t)(value);  // Least significant byte
  return true;
}

/**
 * Selects a sensor (0-7) via routedSensorS0..S2 pins using binary addressing
 */
void selectSensor(uint8_t sensor) {
  // sensor = 0..7, drei Steuerpins
  digitalWrite(routedSensorS0, sensor & 0x01);  // LSB
  digitalWrite(routedSensorS1, (sensor >> 1) & 0x01);
  digitalWrite(routedSensorS2, (sensor >> 2) & 0x01);  // MSB
  delay(10);                                           // kurze Stabilisierung
}

/**
 * Reads analog values from routed sensors and writes them into the payload buffer
 * Each sensor has 2 pins: Pin2 and Pin3
 */
void readAndPackSensors(uint8_t *buffer, size_t *payloadLength, size_t max) {
  Serial.println("==============================================");
  digitalWrite(loraRoutedSensorEnPin, HIGH);
  size_t stabilizingTime = 100;
  // Sensor 0
  selectSensor(0);
  delay(stabilizingTime);
  uint16_t val0 = analogRead(routedSensorPin2);
  uint16_t val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 0 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 1
  selectSensor(1);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 1 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 2
  selectSensor(2);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 2 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 3
  selectSensor(3);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 3 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 4
  selectSensor(4);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 4 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 5
  selectSensor(5);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 5 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 6
  selectSensor(6);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 6 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Sensor 7
  selectSensor(7);
  delay(stabilizingTime);
  val0 = analogRead(routedSensorPin2);
  val1 = analogRead(routedSensorPin3);
  put_u16(buffer, payloadLength, max, val0);
  put_u16(buffer, payloadLength, max, val1);
  Serial.print("Sensor 7 - Pin2: ");
  Serial.print(val0);
  Serial.print(", Pin3: ");
  Serial.println(val1);

  // Return all select pins to 0
  selectSensor(0);
  digitalWrite(loraRoutedSensorEnPin, LOW);
  Serial.println("==============================================");
}

/**
 * Reads BME280 sensor values and packs them into the payload buffer.
 * Temperature (°C) is signed, Pressure (hPa*100) and Humidity (%) are unsigned.
 * Ensures values fit into 16 bits and prints debug output.
 *
 * @param bme           Reference to Adafruit_BME280 object
 * @param buffer        Pointer to payload buffer
 * @param payloadLength Pointer to current payload index
 * @param max           Maximum buffer size
 */
void readAndPackBME280(Adafruit_BME280 &bme, uint8_t *buffer, size_t *payloadLength, size_t max) {
  Serial.println("==============================================");

  digitalWrite(loraRoutedSensorEnPin, HIGH);  // enable I2C rail
  digitalWrite(BME280EnPin, HIGH);            // power BME
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);               // safe to call; ensures bus is up
  Wire.setClock(100000);

  // IMPORTANT: re-init after power-up
  bool ok = bme.begin(0x76);
  if (!ok) ok = bme.begin(0x77);
  if (!ok) {
    Serial.println("BME280 init failed after power-up!");
    // Option A: pack zeros/sentinels to keep payload length fixed
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

  uint32_t rawPressure = bme.readPressure();   // Pa
  uint16_t sensPressure = uint16_t(rawPressure / 10); // Pa/10 => hPa*100-ish

  uint16_t sensHumidity = uint16_t(bme.readHumidity() * 100.0F);

  put_i16(buffer, payloadLength, max, sensTemperature);
  put_u16(buffer, payloadLength, max, sensPressure);
  put_u16(buffer, payloadLength, max, sensHumidity);

  Serial.print("T="); Serial.print(sensTemperature/100.0F);
  Serial.print("C P="); Serial.print(sensPressure/100.0F);
  Serial.print("hPa H="); Serial.print(sensHumidity/100.0F);
  Serial.println("%");

  digitalWrite(BME280EnPin, LOW);
  Serial.println("==============================================");
}

void refreshSensorData() {
  payloadLength = 0;
  readAndPackSensors(buffer, &payloadLength, sizeof(buffer));

  if (bmeAvailable) {
    readAndPackBME280(bme, buffer, &payloadLength, sizeof(buffer));
  }

  getBootCycle();
  put_u32(buffer, &payloadLength, sizeof(buffer), bootCount);

  getCellLvlPercent();
  put_u16(buffer, &payloadLength, sizeof(buffer), CellLvlPercent);
  put_u16(buffer, &payloadLength, sizeof(buffer), batterylvl);
}

void setup() {
  // Serielle Verbindung initialisieren
  Serial.begin(115200);
  Serial.println(F("Starting"));

  Serial.println("Configuring WDT...");
  Serial.print("Watchdog Timeout (in seconds) set to : ");
  Serial.println(WDT_TIMEOUT);
  esp_task_wdt_deinit();
  // Task Watchdog configuration
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,                 // Convertin ms
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // Bitmask of all cores, https://github.com/espressif/esp-idf/blob/v5.2.2/examples/system/task_watchdog/main/task_watchdog_example_main.c
    .trigger_panic = true                             // Enable panic to restart ESP32
  };
  // WDT Init
  ESP32_ERROR = esp_task_wdt_init(&wdt_config);
  Serial.println("Last Reset : " + String(esp_err_to_name(ESP32_ERROR)));
  esp_task_wdt_add(NULL);  //add current thread to WDT watch

  // --- GPIO configuration ---
  pinMode(BME280EnPin, OUTPUT);
  digitalWrite(BME280EnPin, LOW);
  pinMode(routedSensorS0, OUTPUT);
  digitalWrite(routedSensorS0, LOW);
  pinMode(routedSensorS1, OUTPUT);
  digitalWrite(routedSensorS1, LOW);
  pinMode(routedSensorS2, OUTPUT);
  digitalWrite(routedSensorS2, LOW);
  pinMode(loraRoutedSensorEnPin, OUTPUT);
  digitalWrite(loraRoutedSensorEnPin, LOW);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(routedSensorPin3, INPUT);
  analogSetPinAttenuation(routedSensorPin3, ADC_11db);
  pinMode(routedSensorPin2, INPUT);
  analogSetPinAttenuation(routedSensorPin2, ADC_11db);
  pinMode(vBatPin, INPUT);
  analogSetPinAttenuation(vBatPin, ADC_11db);
  
  
  
  delay(100);
  batterylvl = analogRead(vBatPin);
  delay(100);
  Serial.println("batterylvl: ");
  Serial.println(batterylvl);
  
  digitalWrite(loraRoutedSensorEnPin, HIGH);  // enable I2C rail / routing
  digitalWrite(BME280EnPin, HIGH);            // power BME
  delay(300);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  bmeAvailable = bme.begin(0x76);
  if (!bmeAvailable) bmeAvailable = bme.begin(0x77);

  Serial.println(bmeAvailable ? "BME280 gefunden" : "BME280 NICHT gefunden!");

  digitalWrite(BME280EnPin, LOW);
  digitalWrite(loraRoutedSensorEnPin, LOW);

  if (!debug) {
    //Verhindere einen deadlock des ESP32 durch zu geringen Akkustand
    // Batterielevel auslesen

    getCellLvlPercent();
    if (CellLvlPercent < 20) {
      TIME_TO_SLEEP = 24 * 3600ULL;  //(24H)
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");

      // Go in Deep Sleep mode
      Serial.println("Goes into Deep Sleep mode");
      Serial.println("----------------------");
      delay(100);
      esp_deep_sleep_start();
      Serial.println("This will never be displayed");
    }
  }

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}

void setSleepTime() {
  getCellLvlPercent();
  // Standard Sleep Time (24 hours)

  TIME_TO_SLEEP = 24 * 3600ULL;

  if (CellLvlPercent > 90) {
    TIME_TO_SLEEP = 1ULL;  // Disabled ESP32 deep sleep
    TX_INTERVAL = 300ULL;   // Use os_setTimedCallback
  } else if (CellLvlPercent > 70) {
    TIME_TO_SLEEP = 1200ULL;  // 10 minutes   // Use ESP32 deep sleep
    TX_INTERVAL = 1ULL;      // Disabled os_setTimedCallback
  } else if (CellLvlPercent > 40) {
    TIME_TO_SLEEP = 3600ULL;  // 1 hour
    TX_INTERVAL = 1ULL;
  } else if (CellLvlPercent > 20) {
    TIME_TO_SLEEP = 2 * 3600ULL;  // 2 hours
    TX_INTERVAL = 1ULL;
  }
  // Ausgabe der Sleep-Time
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
  const int minOperatingVoltage = 2000;  //Vergleichsweise hoch da DeepSleep Current bei werten unter 2000 deutlich über normal 200uA
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
  // Initialisierung des EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Adressen und Code-Initialisierung
  const int addressBootCycle = 0;
  const int addressInitCode = 10;
  const uint32_t expectedCode = 123456789;

  // Initialisierungscode lesen
  uint32_t storedInitCode;
  EEPROM.get(addressInitCode, storedInitCode);

  Serial.print("Gelesener Init-Code: ");
  Serial.println(storedInitCode);

  // Überprüfen, ob der gespeicherte Init-Code übereinstimmt
  if (storedInitCode != expectedCode) {
    Serial.println("Init-Code nicht gefunden, BootCycle wird auf 1 gesetzt.");

    // Init-Code und Boot-Zähler zurücksetzen
    EEPROM.writeUInt(addressInitCode, expectedCode);
    EEPROM.writeUInt(addressBootCycle, 1);
    EEPROM.commit();  // Commit, da Änderungen vorgenommen wurden

    bootCount = 1;
  } else {
    // Boot-Zyklus erhöhen
    Serial.println("BootCycle wird erhöht.");

    uint32_t currentBootCycle;
    EEPROM.get(addressBootCycle, currentBootCycle);

    Serial.print("Aktueller Boot-Zyklus: ");
    Serial.println(currentBootCycle);

    uint32_t newBootCycle = currentBootCycle + 1;
    EEPROM.writeUInt(addressBootCycle, newBootCycle);
    EEPROM.commit();  // Commit, da Änderungen vorgenommen wurden

    bootCount = newBootCycle;
  }

  // EEPROM-Prozess beenden
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
  const uint32_t limitMs = 30 * 1000; //Todo check if correct
  if (msUntil > limitMs) {
    Serial.print("Backoff too large at ");
    Serial.print(where);
    Serial.print(": msUntil=");
    Serial.print(msUntil);
    Serial.print(" > limit=");
    Serial.println(limitMs);
    Serial.print("start deep sleep and try again afterwards");
    Serial.println(limitMs);
    delay(100);
    TIME_TO_SLEEP = 2 * 3600ULL;  // 2 Stunden
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("ESP32 wake-up in " + String(TIME_TO_SLEEP) + " seconds");
    // Go in Deep Sleep mode
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




/*######################################
//TTN Payload formatter
//######################################

function decodeUplink(input) {
    var bytes = input.bytes;
    
    // Hilfsfunktion für 16-Bit signed
    function decodeSigned16Bit(byte1, byte2) {
        var value = (byte1 << 8) | byte2;
        if (value >= 0x8000) value -= 0x10000;
        return value;
    }
    
    // Hilfsfunktion für 16-Bit unsigned
    function decodeUnsigned16Bit(byte1, byte2) {
        return (byte1 << 8) | byte2;
    }
    
    
    // Helper: decode unsigned 32-bit integer
    function decodeUnsigned32(bytes, index) {
        return (bytes[index] << 24) | (bytes[index + 1] << 16) | (bytes[index + 2] << 8) | bytes[index + 3];
    }
    
    // Hilfsfunktion für Werte mit Komma (z.B. Temperature)
    function moveComma(input) {
        return input / 100;
    }

    // ------------- Analoge Sensorwerte ------------- //
    var sensorValues = [];
    for (var i = 0; i < 8; i++) {
        // Je Sensor zwei Pins → 2 Bytes pro Pin → 4 Bytes pro Sensor
        var offset = i * 4;
        var pin2 = decodeUnsigned16Bit(bytes[offset], bytes[offset + 1]);
        var pin3 = decodeUnsigned16Bit(bytes[offset + 2], bytes[offset + 3]);
        sensorValues.push({pin2: pin2, pin3: pin3});
    }

    // ------------- BME280 ------------- //
    var sensTemperature = moveComma(decodeSigned16Bit(bytes[32], bytes[33]));       // °C
    var sensPressure = decodeUnsigned16Bit(bytes[34], bytes[35]);                   // hPa*100
    var sensHumidity = moveComma(decodeUnsigned16Bit(bytes[36], bytes[37]));        // %*100

    // ------------- BootCycle und Batterie ------------- //
    var bootCycle = decodeUnsigned32(bytes, 38);                                    // 32-bit unsigned
    var cellLvlPercent = decodeUnsigned16Bit(bytes[42], bytes[43]);
    var batteryRAW = decodeUnsigned16Bit(bytes[44], bytes[45]);

    // Ergebnisobjekt
    var decoded = {
        sensor0: sensorValues[0],
        sensor1: sensorValues[1],
        sensor2: sensorValues[2],
        sensor3: sensorValues[3],
        sensor4: sensorValues[4],
        sensor5: sensorValues[5],
        sensor6: sensorValues[6],
        sensor7: sensorValues[7],
        sensTemperature: sensTemperature,
        sensPressure: sensPressure / 100.0, // optional in hPa
        sensHumidity: sensHumidity,
        bootCycle: bootCycle,
        cellLvlPercent: cellLvlPercent,
        batteryRAW: batteryRAW
    };
    
    return {
        data: decoded,
        warnings: [],
        errors: []
    };
}
*/