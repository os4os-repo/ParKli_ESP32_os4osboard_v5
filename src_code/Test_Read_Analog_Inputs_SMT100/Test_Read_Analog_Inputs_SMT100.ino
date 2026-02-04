/*
// Use Screw Terminal 2 for this example
// Use SMT50/100 analog sensor
// Connect Brown to the first port (3.3V)
// Connect Green Wire to the second port (Pin 33 - temperature)
// Connect Yellow Wire to the third port (Pin 32 - moisture)
// Connect White Wire to the fourth port (GND)
*/

//#include <OneWire.h>
//#include <DallasTemperature.h>

//#define loraNSS 5
//#define loraRST 26
//#define loraDOI0 4
//#define loraDOI1 17

//#define SCL_PIN 22
//#define SDA_PIN 21

//#define ledPin 13

//#define sparePin35 35
//#define sparePin39 36
//#define sparePin39 39

//#define vBatPin 34

#define routedSensorPin3 32       //humTempOptEnPin
#define routedSensorPin2 33       //optSensorPin
#define loraRoutedSensorEnPin 27  //loraTdsEnPin

#define routedSensorS0 25
#define routedSensorS1 14
#define routedSensorS2 16

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Starting"));

  //selecting Screw Terminal 2 (010)
  pinMode(routedSensorS0, OUTPUT);
  digitalWrite(routedSensorS0, LOW);
  pinMode(routedSensorS1, OUTPUT);
  digitalWrite(routedSensorS1, HIGH);
  pinMode(routedSensorS2, OUTPUT);
  digitalWrite(routedSensorS2, LOW);

  //defining the 2 and 3 screw terminal as inputs
  pinMode(routedSensorPin2, INPUT);
  pinMode(routedSensorPin3, INPUT);
  
  //activating the senosrs
  pinMode(loraRoutedSensorEnPin, OUTPUT);
  digitalWrite(loraRoutedSensorEnPin, HIGH);
}

void loop() {
  Serial.println("analog value of terminal 2 - temperature");  
  Serial.println(analogRead(routedSensorPin2));
  Serial.println("");
  Serial.println("analog value of terminal 3 - moisture");
  Serial.println(analogRead(routedSensorPin3));
  Serial.println("============================");
  delay(500);
}
