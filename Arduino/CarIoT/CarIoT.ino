#include <SoftwareSerial.h>

int txPin = 10;
int rxPin = 11;
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  Serial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(9600);
}

void loop() {
  if(mySerial.available() > 0) {
    //Serial.println("Message receive :");
    unsigned char data = mySerial.read();
    Serial.println(data);
  }
  // Communicate (transmission) with the STM32
  /*mySerial.print("1");
  delay(500);
  mySerial.print("0");
  delay(500);*/
}