#include <SoftwareSerial.h>
#define ESP8266_RX   2
#define ESP8266_TX   3
SoftwareSerial ESPserial(ESP8266_RX, ESP8266_TX);
char buffer[] = {' ',' ',' '};

void setup() {
  Serial.begin(19200);     // communication with the host computer
  ESPserial.begin(9600);  
}

void loop() {

  if (ESPserial.available()){
   // ESPserial.readBytes(buffer, 3);
   // int hello = (buffer[0]-48)*100+(buffer[1]-48)*10+(buffer[2]-48);
    int hello = ESPserial.read();
    Serial.println(hello-48);
  }
}
