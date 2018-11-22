# Linda and Grace's SE101 Project

The goal was to build a drone which can be controlled through WiFi. The arduino uno R3 is our flight controller and the NodeMCU is our WiFi module. To construct the drone, propellers, brushless motors, ESCs, 4S LiPo battery, power distribution board and MPU6050 were needed.

## Master Branch

On the master branch, the WiFi folder contains a file for the NodeMCU (espWiFi.ino) and a file for the arduino (arduinoWiFi.ino). 

There is a folder called basic_motor_control which contains the basic arduino code to turn on and off the motors. You can run basic_motor_control.ino on the arduino and espWiFi.ino on the NodeMCU to get the motors to turn on and off via commands sent from your phone. A video (basic_control.mp4) of this has been added to the main project folder.

There is also a folder called wifi_control_drone which contains a more complex arduino code for the drone which includes PID. You can run wifi_control_drone.ino on the arduino and espWiFi.ino on the NodeMCU to get the drone to receive more complex commands.

## Branches

Other than the master branch, this project has two other branches (flight_controller, test_app) where previous versions of code can be found.