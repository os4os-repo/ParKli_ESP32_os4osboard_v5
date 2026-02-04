#include <OneWire.h>
#include <DallasTemperature.h>

//#define loraNSS 5
//#define loraRST 26
//#define loraDOI0 4
//#define loraDOI1 17

//#define SCL_PIN 22
//#define SDA_PIN 21

#define ledPin 13

//#define sparePin35 35
//#define sparePin39 36
//#define sparePin39 39

#define vBatPin 34

#define routedSensorPin3 32       //humTempOptEnPin
#define routedSensorPin2 33       //optSensorPin
#define loraRoutedSensorEnPin 27  //loraTdsEnPin

#define routedSensorS0 25
#define routedSensorS1 14
#define routedSensorS2 16

OneWire oneWire(routedSensorPin2);
DallasTemperature sensors(&oneWire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Starting"));

  //selecting 010=2
  pinMode(routedSensorS0, OUTPUT);
  digitalWrite(routedSensorS0, LOW);
  pinMode(routedSensorS1, OUTPUT);
  digitalWrite(routedSensorS1, HIGH);
  pinMode(routedSensorS2, OUTPUT);
  digitalWrite(routedSensorS2, LOW);

  //activating the senosrs
  pinMode(loraRoutedSensorEnPin, OUTPUT);
  digitalWrite(loraRoutedSensorEnPin, HIGH);
  delay(1000);
  sensors.begin();
}

void loop() {

  sensors.requestTemperatures();
  Serial.println(sensors.getTempCByIndex(0));
  delay(500);
}
