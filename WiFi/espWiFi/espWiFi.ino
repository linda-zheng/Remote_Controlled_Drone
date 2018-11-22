#include <ESP8266WiFi.h>
//set up ssid and password for access point
const char* ssid = "LindaIsCool"; 
const char* password = "1234home";
WiFiServer server(80);

#include <SoftwareSerial.h>
#define Arduino_RX 4 //pin D2 (GPIO4) on NodeMCU
#define Arduino_TX 5 //pin D1 (GPIO5) on NodeMCU
//set up serial object for communication with arduino
SoftwareSerial ArduinoSerial(Arduino_RX, Arduino_TX);

void droneOff();
void droneUp();
void droneDown();
void droneLeft();
void droneRight();

void setup() {
  Serial.begin(115200);
  ArduinoSerial.begin(9600);

  WiFi.mode(WIFI_AP); //set NodeMCU to access point mode
  WiFi.softAP(ssid, password); //set credentials
  server.begin(); //begin server
  delay(5000);
  Serial.println(WiFi.softAPIP()); //for debugging
}

void loop() 
{
  WiFiClient client = server.available();
  if (!client) { //if there is no client making a request, then keep looping until there is
    return; 
  } 

  String request = client.readString(); //store the request made by the client
  
  //check if request matches a button
  //if it does, run the desired function
  if (request.indexOf("/DRONEOFF") != -1) {
    Serial.println("drone off");
    droneOff();
  } else if (request.indexOf("/DRONEDOWN") != -1){ 
    Serial.println("drone down");
    droneDown(); 
  } else if (request.indexOf("/DRONEUP") != -1){ 
    Serial.println("drone up");
    droneUp(); 
  } else if (request.indexOf("/DRONELEFT") != -1){ 
    Serial.println("drone left");
    droneLeft(); 
  } else if (request.indexOf("/DRONERIGHT") != -1){ 
    Serial.println("drone right");
    droneRight(); 
  } 

  //prepare the html document
  String s = "HTTP/1.1 200 OK\r\n"; //respond using HTTP Protocol 1.1 200
  s += "Content-Type: text/html\r\n\r\n";
  s += "<!DOCTYPE HTML>\r\n<html>\r\n";
  s += "<br><br><br><p align=\"middle\"><input type=\"button\" name=\"bUp\" value=\"UP\" style=\"font-size:50px; height:125px; width:300px\" align=\"middle\" onclick=\"location.href='/DRONEUP'\"></p>";
  s += "<br><br><br><p align=\"middle\"><input type=\"button\" name=\"bLeft\" value=\"LEFT\" style=\"font-size:50px; height:125px; width:300px\" onclick=\"location.href='/DRONELEFT'\">";
  s += "&nbsp;&nbsp;&nbsp;<input type=\"button\" name=\"bOff\" value=\"OFF\" style=\"font-size:50px; height:125px; width:300px\" onclick=\"location.href='/DRONEOFF'\">";
  s += "&nbsp;&nbsp;&nbsp;<input type=\"button\" name=\"bRight\" value=\"RIGHT\" style=\"font-size:50px; height:125px; width:300px\" onclick=\"location.href='/DRONERIGHT'\"></p>";
  s += "<br><br><br><p align=\"middle\"><input type=\"button\" name=\"bDown\" value=\"DOWN\" style=\"font-size:50px; height:125px; width:300px\" onclick=\"location.href='/DRONEDOWN'\"></p>";
  s += "<br></html>\n";
    
  client.flush (); //clear previous info in the stream
  client.print (s); // Send the response to the client
  Serial.println("Client disonnected" ); //for debugging
  delay(500);
} 


//the functions below send a character to the arduino corresponding to the button that was pressed
void droneOff() {
  ArduinoSerial.print(0);  
}

void droneUp() {
  ArduinoSerial.print(1); 
}

void droneDown() {
  ArduinoSerial.print(2); 
}

void droneLeft() {
  ArduinoSerial.print(3);  
}

void droneRight() {
  ArduinoSerial.print(4);  
}
