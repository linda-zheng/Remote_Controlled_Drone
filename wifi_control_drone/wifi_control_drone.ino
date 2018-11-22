
//---------------------------constants to control drone
//these will be set to the respective angles required to roll left, right and pitch up, down
#define ROLL_ANGLE 7
#define PITCH_ANGLE 7

//define different thrust to write to motors
#define MAX_T 1200 //thrust to accelerate up, right, left
#define HOVER_T 1100 //thrust to hover in place
#define FALL_T 1050 //thrust to fall slowly
#define MIN_T 1000 //min value that can be written to motoe (no thrust)

//define absolute bounds for thrust
#define UPPER_T 1250
#define LOWER_T 1000

//define how long thrust is applied for each signal (in milliseconds)
#define THRUST_DURATION 2000

//set the integer signals sent through wifi that will control the drone
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define OFF 0

//variables to hold speed of motors, initialize all to min
int L_F_thrust = MIN_T;
int R_F_thrust = MIN_T;
int R_B_thrust = MIN_T;
int L_B_thrust = MIN_T;

//---------------------------stuff for wifi
#include <SoftwareSerial.h>
#define ESP8266_RX 8
#define ESP8266_TX 3
SoftwareSerial ESPserial(ESP8266_RX, ESP8266_TX);

int first_connect = 0;

//---------------------------stuff for motors
#include <Servo.h> 
Servo L_F; //left front motor
Servo R_F; //right front motor
Servo R_B; //right back motor
Servo L_B; //left back motor
String readString;
//---------------------------stuff for accel/gyro
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//-----------------------------stuff for PID (stabilization)
//we will need to use elapsed time for D (Derivative) of PID
float elapsedTime, time, timePrev;

//ROLL
//variables to calculate PID
float roll_PID, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
//adjusting constants for roll PID
double roll_kp=0.7;//3.55
double roll_ki=0.006;//0.003
double roll_kd=1.2;//2.05
//target roll angle
float roll_desired_angle = 0;    

//PITCH
//variables to calculate PID
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
//adjusting constants for pitch PID
double pitch_kp=0.72;//3.55
double pitch_ki=0.006;//0.003
double pitch_kd=1.22;//2.05
//target pitch angle
float pitch_desired_angle = 0;     

//YAW
//variables to calculate PID
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;
//adjusting constants for roll PID
double yaw_kp=0.7;//3.55
double yaw_ki=0.006;//0.003
double yaw_kd=1.2;//2.05
//target yaw angle
double yaw_desired_angle = 0; //we want to keep yaw constant

void setup() {
  //serial setup
  Serial.begin(115200);
  ESPserial.begin(9600);

  //motor setup
  L_F.attach(7);
  R_F.attach(6);
  R_B.attach(5);
  L_B.attach(4);
  
  //write the minimum value immediately so that motors don't go into calibration mode
  L_F.writeMicroseconds(MIN_T);
  R_F.writeMicroseconds(MIN_T);
  R_B.writeMicroseconds(MIN_T);
  L_B.writeMicroseconds(MIN_T);
  
  Serial.print("Writing: ");
  Serial.println(MIN_T);

  //accel/gyro setup
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial) // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop(){
  int hello = -1;
  
  //get the input from wifi
    if (ESPserial.available()){
      Serial.println("Wifi Connected");
      first_connect = 1;
      hello = ESPserial.read()-48;
      Serial.print("Receiving: ");
      Serial.println(hello);
      
      int startTime = millis();
      while(millis() - startTime <= THRUST_DURATION){ // apply thrust for 3 seconds
        thrust(hello, 0);//0 --> not hovering
      }
  
    }

  if(first_connect){
  //apply hovering thrust
  thrust(hello, 1);//1 --> is hovering, we want to use it here to stabilize and set hovering thrust
  }
  
  
}

void thrust(int hello, int is_hovering) {
    //calculate elapsed time
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;
   
    //mutate array of yaw pitch roll angles
    //test if the accelerometer is working, if not, let the drone fall slowly
    if(!getYPR()){
      L_F.writeMicroseconds(FALL_T);
      R_F.writeMicroseconds(FALL_T);
      R_B.writeMicroseconds(FALL_T);
      L_B.writeMicroseconds(FALL_T);
      return;
    }

    if(!is_hovering){
    //determine base throttle to write to motor based on hello signal
      if (hello == OFF) {
        //set all motors to minimum (no thrust)
        L_F_thrust = MIN_T;
        R_F_thrust = MIN_T;
        R_B_thrust = MIN_T;
        L_B_thrust = MIN_T;
        
      } else if(hello == DOWN){
        L_F_thrust = FALL_T;
        R_F_thrust = FALL_T;
        R_B_thrust = FALL_T;
        L_B_thrust = FALL_T;
        
      } else {//hello == UP, RIGHT, LEFT
        L_F_thrust = MAX_T;
        R_F_thrust = MAX_T;
        R_B_thrust = MAX_T;
        L_B_thrust = MAX_T;
      }
      
    } else {
      L_F_thrust = HOVER_T;
      R_F_thrust = HOVER_T;
      R_B_thrust = HOVER_T;
      L_B_thrust = HOVER_T;
    }
    
    //store angles
    float yaw = ypr[0] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float roll = ypr[2] * 180/M_PI;

    Serial.print("ypr\t");
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.println(roll);

    if(is_hovering){
      //calculate PID values based on current yaw pitch and roll, keep drone level
      PID(5, roll, pitch, yaw);
    } else {
      //calculate PID values based on current yaw pitch and roll and signal from wifi
      PID(hello, roll, pitch, yaw);
    }
    
    //adjust thrust using PID values
    L_F_thrust  = L_F_thrust - roll_PID - pitch_PID - yaw_PID;
    R_F_thrust = R_F_thrust + roll_PID - pitch_PID + yaw_PID;
    R_B_thrust  = R_B_thrust + roll_PID + pitch_PID - yaw_PID;
    L_B_thrust  = L_B_thrust - roll_PID + pitch_PID + yaw_PID;
    
    //bound thrust values to range of thrust that we want
    L_F_thrust  = bound(L_F_thrust, UPPER_T, LOWER_T);
    R_F_thrust  = bound(R_F_thrust, UPPER_T, LOWER_T);
    R_B_thrust  = bound(R_B_thrust, UPPER_T, LOWER_T);
    L_B_thrust  = bound(L_B_thrust, UPPER_T, LOWER_T);

    L_F.writeMicroseconds(L_F_thrust);
    R_F.writeMicroseconds(R_F_thrust);
    R_B.writeMicroseconds(R_B_thrust);
    L_B.writeMicroseconds(L_B_thrust);
    Serial.print("Left Front: ");
    Serial.println(L_F_thrust);
    Serial.print("Right Front: ");
    Serial.println(R_F_thrust);
    Serial.print("Right Back: ");
    Serial.println(R_B_thrust);
    Serial.print("Left Back: ");
    Serial.println(L_B_thrust);
    
}


//mutates ypr array with the current yaw, pitch and roll angles
//also acts as a boolean for if the accelerometer is working or not
int getYPR(void){
  //get the values from accelerometer
    // if programming failed, don't try to do anything
    if (!dmpReady) return 0;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } /*else*/ if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        
        //above two functions get values for this function
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
    
    return 1;
}

//calculate the PID values for yaw pitch roll based on "hello" signal from wifi
void PID(int hello, float roll, float pitch, float yaw){
   if (hello == OFF) {
      return;
   } else if(hello == UP){
      roll_desired_angle = 0;
      pitch_desired_angle = PITCH_ANGLE;
      
   } else if(hello == DOWN){
      roll_desired_angle = 0;
      pitch_desired_angle = -PITCH_ANGLE;
        
   } else if(hello == LEFT){
      roll_desired_angle = -ROLL_ANGLE;
      pitch_desired_angle = 0;  
         
   } else if(hello == RIGHT){
      roll_desired_angle = ROLL_ANGLE;
      pitch_desired_angle = 0;
      
   } else if(hello == 5){//when we want the drone to hover
      roll_desired_angle = 0;
      pitch_desired_angle = 0;
   }
  

  /*First calculate the error between the desired angle and 
*the real measured angle*/
roll_error = roll - roll_desired_angle;
pitch_error = pitch - pitch_desired_angle; 
yaw_error = yaw - yaw_desired_angle;
   
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/
roll_pid_p = roll_kp*roll_error;
pitch_pid_p = pitch_kp*pitch_error;
yaw_pid_p = yaw_kp*yaw_error;

/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 < roll_error <3)
{
  roll_pid_i = roll_pid_i + (roll_ki*roll_error);  
}

if(-3 < pitch_error <3)
{
  pitch_pid_i = pitch_pid_i + (pitch_ki*pitch_error);  
}

if(-3 < yaw_error <3)
{
  yaw_pid_i = yaw_pid_i + (yaw_ki*yaw_error);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/
roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
yaw_pid_d = yaw_kd*((yaw_error - yaw_previous_error)/elapsedTime);

//store error for next iteration
roll_previous_error = roll_error; 
pitch_previous_error = pitch_error;
yaw_previous_error = yaw_error;

/*The final PID values is the sum of each of this 3 parts*/
roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;
  
}

//return value bounded by upper and lower bounds
int bound(int value, int upper, int lower){
  if(value > upper){
    return upper;
  } else if (value < lower){
    return lower;
  } else {
    return value;
  }
}
