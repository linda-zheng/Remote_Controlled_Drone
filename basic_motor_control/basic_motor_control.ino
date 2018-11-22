#include <SoftwareSerial.h>
#define ESP8266_RX   8
#define ESP8266_TX   3
SoftwareSerial ESPserial(ESP8266_RX, ESP8266_TX);

#include <Servo.h> 
Servo R_T;  
Servo R_B;
Servo L_T;
Servo L_B;

#define HIGH_THRUST 1150
#define MIN_THRUST 1000

void setup() {
  Serial.begin(19200);
  ESPserial.begin(9600);
  
  L_T.attach(7);
  R_T.attach(6);
  R_B.attach(5);
  L_B.attach(4);

  Serial.println("waiting...");

  Serial.println("go!");
  
  L_T.writeMicroseconds(1000);
  R_T.writeMicroseconds(1000);
  L_B.writeMicroseconds(1000);
  R_B.writeMicroseconds(1000);
  delay(5000);
  Serial.println("ready"); // so I can keep track of what is loaded
}

void loop() {
    
    if (ESPserial.available()){
      int hello = ESPserial.read()-48;
      Serial.print("Receiving: ");
      Serial.println(hello);
      int n;
      
      if (hello == 1) {
        n = HIGH_THRUST;
       } else {
        n = MIN_THRUST;
      }

      Serial.print("Writing: ");
      Serial.println(n);
      
      L_T.writeMicroseconds(n);
      L_B.writeMicroseconds(n);
      R_T.writeMicroseconds(n);
      R_B.writeMicroseconds(n);
    }
    
}
