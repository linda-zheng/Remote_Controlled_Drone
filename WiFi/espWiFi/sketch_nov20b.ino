#include <ESP8266WiFi.h>
const char* ssid = "LindaIsCool";
const char* password = "1234home";
WiFiServer server(80);

#include <SoftwareSerial.h>
#define Arduino_RX 4 // D2
#define Arduino_TX 5 // D1
SoftwareSerial ArduinoSerial(Arduino_RX, Arduino_TX);

void droneUp();
void droneDown();

void setup() {
  Serial.begin(115200);
  ArduinoSerial.begin(9600);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  server.begin();
  delay(5000);
  Serial.println(WiFi.softAPIP());
}

void loop() 
{
   WiFiClient client = server.available();
      if (!client) { 
      return; 
    } 

    String request = client.readString();

    Serial.println("Somebody has connected :)");

    if (request.indexOf("/Down") != -1){ 
            droneDown(); 
            Serial.println("droneDown command received via wifi");
    }
    else if (request.indexOf("/Up") != -1){ 
            droneUp(); 
            Serial.println("droneUp command received via wifi");
    }

  // Prepare the HTML document to respond and add buttons:
  String s = "HTTP/1.1 200 OK\r\n";
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<br><input type=\"button\" name=\"b1\" value=\"Drone Up\" onclick=\"location.href='/Up'\">";
  s += "<br><br><br>";
  s += "<input type=\"button\" name=\"bi\" value=\"Drone Down\" onclick=\"location.href='/Down'\">";
  s += "</html>\n";
  //Serve the HTML document to the browser.
  client.flush ();
  //clear previous info in the stream
  client.print (s); // Send the response to the client
  delay(1);
  Serial.println("Client disonnected" );
  delay(500);
} 

void droneUp() {
  ArduinoSerial.print(1); 
}

void droneDown() {
  ArduinoSerial.print(0); 
}
