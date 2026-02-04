/*
// Use Screw Terminal 2 for this example
// Connect Red Wire to the first port (3.3V)
// Connect Yellow Wire to the second port (Pin 33)
// Connect White Wire to the third port (Pin 32)
// Connect Black Wire to the fourth port (GND)
*/



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

HardwareSerial mySerial(2);  // RX, TX
unsigned char data[4] = {};
float distance;

#define routedSensorPin3 32       // Sensor TX → ESP RX (white wire)
#define routedSensorPin2 33       // Sensor RX ← ESP TX (yellow wire)
#define loraRoutedSensorEnPin 27  //loraTdsEnPin

#define routedSensorS0 25
#define routedSensorS1 14
#define routedSensorS2 16

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

  mySerial.begin(9600, SERIAL_8N1, routedSensorPin3, routedSensorPin2);
}

void loop() {

  // Warten bis mindestens 4 Bytes da sind
  if (mySerial.available() >= 4) {

    // Synchronisation auf Frame-Start 0xFF
    if (mySerial.read() == 0xFF) {

      data[0] = 0xFF;
      data[1] = mySerial.read();
      data[2] = mySerial.read();
      data[3] = mySerial.read();

      int sum = (data[0] + data[1] + data[2]) & 0xFF;

      if (sum == data[3]) {
        distance = (data[1] << 8) + data[2];

        if (distance > 280) {
          Serial.print("distance=");
          Serial.print(distance / 10);
          Serial.println("cm");
        } else {
          Serial.println("Below the lower limit");
        }
      } 
      else {
        Serial.println("Checksum ERROR");
      }

    } else {
      // Wenn erstes Byte kein 0xFF war → verwerfen und neu versuchen
      Serial.println("Frame sync error");
    }
  }

  delay(50);
}
