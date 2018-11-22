#include <SoftwareSerial.h>
#define ESP8266_RX 2 //digital pin 2
#define ESP8266_TX 3 //digital pin 3
//set up serial object to communicate with NodeMCU
SoftwareSerial ESPserial(ESP8266_RX, ESP8266_TX); 
//connect digital pin 2 to D1 on NodeMCU
//connect digital pin 3 to D2 on NodeMCU

void setup() {
  Serial.begin(19200);  //for debugging
  ESPserial.begin(9600);  //for communication with NodeMCU
}

void loop() {
  if (ESPserial.available()){
     //set hello to the original single-digit integer that was sent by the NodeMCU
     int hello = ESPserial.read()-48; 
     Serial.print(hello); //for debugging
  }
}
