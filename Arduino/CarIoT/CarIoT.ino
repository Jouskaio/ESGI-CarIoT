#include <SoftwareSerial.h>

int txPin = 10;
int rxPin = 11;

int soundPort = A0;

SoftwareSerial STM32 = SoftwareSerial(rxPin, txPin);

void setup() {
  Serial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  // echanger button R avec celui moteur
  attachInterrupt(0, sendDataInterruptionL,FALLING); // L
  //attachInterrupt(1, sendDataInterruptionR, FALLING); // 
  //pinMode(buttonLEDLeft, INPUT);
  //pinMode(buttonLEDRight,INPUT);
  //pinMode(soundPort, OUTPUT)
  STM32.begin(9600);
  //while (!Serial) {} // This function is used to wait until the connexion is set up, then the code access to the loop() function 
}

void loop() {
  // Send and read message from Serial
  if(STM32.available() > 0) {
    //Serial.println("Message receive :");
    unsigned char data = STM32.read();
    Serial.println(data, DEC);
    //Serial.println(detectInterruption(buttonLEDLeft));
  }
  //musicSound();



  // Communicate (transmission) with the STM32
  //mySerial.print("1");
  //delay(500);
  //mySerial.print("0");
  //delay(500);*/
}


void sendDataInterruptionL() {
  //STM32.print("L");
  Serial.println(0);
  STM32.print(0);
}

void sendDataInterruptionR() {
  //STM32.print("L");
  Serial.println(1);
  STM32.print(1);
}

void musicSound() {
  tone(soundPort, 2637, 200);
  delay(400);
  tone(soundPort, 1975, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 2349, 200);
  delay(400);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 1975, 200);
  delay(200);
  tone(soundPort, 1760, 200);
  delay(400);
  tone(soundPort, 1760, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 2637, 200);
  delay(400);
  tone(soundPort, 2349, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 1975, 200);
  delay(400);
  tone(soundPort, 1975, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 2349, 200);
  delay(400);
  tone(soundPort, 2637, 200);
  delay(400);
  tone(soundPort, 2093, 200);
  delay(400);
  tone(soundPort, 1760, 200);
  delay(400);
  tone(soundPort, 1760, 200);
  delay(800);
  tone(soundPort, 1760, 200);
  delay(400);
  tone(soundPort, 2349, 200);
  delay(200);
  tone(soundPort, 2794, 200);
  delay(200);
  tone(soundPort, 3520, 200);
  delay(400);
  tone(soundPort, 3136, 200);
  delay(200);
  tone(soundPort, 2794, 200);
  delay(200);
  tone(soundPort, 2637, 200);
  delay(600);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 2637, 200);
  delay(400);
  tone(soundPort, 2349, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 1975, 200);
  delay(400);
  tone(soundPort, 1975, 200);
  delay(200);
  tone(soundPort, 2093, 200);
  delay(200);
  tone(soundPort, 2349, 200);
  delay(400);
  tone(soundPort, 2637, 200);
  delay(400);
  tone(soundPort, 2093, 200);
  delay(400);
  tone(soundPort, 1760, 200);
  delay(400);
  tone(soundPort, 1760, 200);
  delay(800);
}
