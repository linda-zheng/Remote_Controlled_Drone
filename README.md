# Linda and Grace's SE101 Project

The goal was to build a drone which can be controlled through WiFi. The arduino uno R3 is our flight controller and the NodeMCU is our WiFi module.

## Installing

On the master branch, the WiFi folder contains a file for the NodeMCU (espWiFi.ino) and a file for the arduino (arduinoWiFi.ino). 

There is a folder called basic_motor_control which contains the basic arduino code to turn on and off the motors. You can run basic_motor_control.ino on the arduino and espWiFi.ino on the NodeMCU to get the motors to turn on and off via commands sent from your phone.

There is also a folder called wifi_control_drone which contains a more complex arduino code for the drone which includes PID. You can run wifi_control_drone.ino on the arduino and espWiFi.ino on the NodeMCU to get the drone to receive more complex commands.