#include <SoftwareSerial.h>

int txPin = 10;
int rxPin = 11;
int buttonLEDLeft = 4;
int buttonLEDRight = 7;
SoftwareSerial STM32 = SoftwareSerial(rxPin, txPin);

void setup() {
  Serial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(txPin, OUTPUT);
  pinMode(buttonLEDLeft, INPUT);
  pinMode(buttonLEDRight,INPUT);
  STM32.begin(9600);
  //while (!Serial) {} // This function is used to wait until the connexion is set up, then the code access to the loop() function 
}

void loop() {
  // Read interruption
  delay(500);
  bool detectLeft = detectInterruption(buttonLEDLeft);
  bool detectRight = detectInterruption(buttonLEDRight);
  // Send and read message from Serial
  if(STM32.available() > 0) {
    //Serial.println("Message receive :");
    sendDataInterruption(detectLeft, 4);
    sendDataInterruption(detectRight, 7);
    unsigned char data = STM32.read();
    //Serial.println(data);
    //Serial.println(detectInterruption(buttonLEDLeft));
  }

  // Communicate (transmission) with the STM32
  //mySerial.print("1");
  //delay(500);
  //mySerial.print("0");
  //delay(500);*/
}

bool detectInterruption(int numberPort) {
  int state=digitalRead(numberPort);
  if (state == 0) {
    return true;
  } else  {
    return false;
  }
}


void sendDataInterruption(bool event, int portLED) {
  //STM32.print("L");
  if(event == 1) {
    if (portLED == 4) {
      Serial.println("L");
      STM32.print("L");
    } else if (portLED == 7) {
      Serial.println("R");
      STM32.print("R");
    }
  } Serial.println(".");
}
