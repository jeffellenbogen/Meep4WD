//XBee setup
#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

#define DEBUG_SERIAL

#define headlight_pin_left 12
#define headlight_pin_right 13
#define heading_baseline 132
#define small_turn_amt 6
#define large_turn_amt 13

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);


Servo servo1;
int currentSpeed = 100;
bool drivingForward;

// Simple version of debug print only takes strings.  Use sprintf if you
// want to embed paramenters.  Currently sending newline to determine when the string
// is done....no newline needed in the string itself.
void debug_print(char *string)
{

  #ifdef DEBUG_SERIAL
  Serial.println(string);
  #endif
  
  XBee.println(string);
}

void setup() {
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  XBee.begin(9600); 
  AFMS.begin();  // create with the default frequency 1.6KHz
  servo1.attach(10);
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  frontMotor->setSpeed(currentSpeed);
  backLeftMotor->setSpeed(currentSpeed);
  backRightMotor->setSpeed(currentSpeed);
  pinMode(headlight_pin_left,OUTPUT);
  pinMode(headlight_pin_right,OUTPUT);  
  stopDriving();

  debug_print("Setup finished");
}

void loop() {
 char debug_string[40];
 
 if (XBee.available())
     {
     char c = XBee.read();

     sprintf(debug_string, "RX char %c", c);
     debug_print(debug_string);
     
        if (c == '1')
          driveForwardSlightLeft();
         else if (c == '2'){
            if (drivingForward)
              driveForward();
            else {
              stopDriving();
              delay(250);
              driveForward();
            }
           }
         else if (c == '3')
            driveForwardSlightRight();           
         else if (c == '4')
            if (drivingForward)
              driveLeft();
            else
              driveLeftBack();            
         else if (c == '5')
            stopDriving();        
         else if (c == '6')
            if (drivingForward)
              driveRight();
            else
              driveRightBack();          
         else if (c == '7')
            driveBackSlightLeft();
         else if (c == '8'){
            if (drivingForward==false)
              driveBack();
            else {
              stopDriving();
              delay(250);
              driveBack();
             }
            }
         else if (c == '9')
            driveBackSlightRight();       
         else if (c == 'R')
            regSpeed();
         else if (c == 'T')
            turboSpeed();
         else if (c == 'S')
            slowSpeed();                    
     }
}

void stopDriving(){
  headlightsOff();
  servo1.write(heading_baseline);
  frontMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  backRightMotor->run(RELEASE);

  debug_print("Stop Driving");
}

void driveForward(){
  drivingForward = true;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  debug_print("Drive Forward");
}

void driveBack(){
  drivingForward = false;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  debug_print("Drive Back");
}

void driveLeft(){
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-large_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
  delay(50);
  headlightsOff();
  delay(50);

  debug_print("Drive Left");
}

void driveRight(){
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+large_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
  delay(50);
  headlightsOff();
  delay(50);

  debug_print("Drive Right");
  
}

void driveLeftBack(){
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-large_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
  delay(50);
  headlightsOff();
  delay(50);

  debug_print("Drive Left Back");
}

void driveRightBack(){
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+large_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
  delay(50);
  headlightsOff();
  delay(50);

  debug_print("Drive Right Back");
}

void driveForwardSlightLeft(){
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  debug_print("Drive Forward Slight Left");
}

void driveForwardSlightRight(){
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  debug_print("Drive Forward Slight Right");
}

void driveBackSlightRight(){
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  debug_print("Drive Back Slight Right");
  
}


void driveBackSlightLeft(){
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  debug_print("Drive Back Slight Left");
}


void regSpeed(){
  currentSpeed = 100;
  setCurrentSpeed();

  debug_print("Reg speed");
}

void turboSpeed(){
  currentSpeed = 255;
  setCurrentSpeed();

  debug_print("Turbo Speed");
  
}

void slowSpeed(){
  currentSpeed = 50;
  setCurrentSpeed();

  debug_print("Slow Speed");
  
}


void setCurrentSpeed(){
  frontMotor->setSpeed(currentSpeed);
  backLeftMotor->setSpeed(currentSpeed);
  backRightMotor->setSpeed(currentSpeed);

}

void headlightsOn(){
  digitalWrite(headlight_pin_left,HIGH);
  digitalWrite(headlight_pin_right,HIGH);
}

void headlightsOff(){
  digitalWrite(headlight_pin_left,LOW);
  digitalWrite(headlight_pin_right,LOW); 
}

void headlightsLeftBlinker(){
   digitalWrite(headlight_pin_left,HIGH);
   digitalWrite(headlight_pin_right,LOW); 
}

void headlightsRightBlinker(){
   digitalWrite(headlight_pin_left,LOW);
   digitalWrite(headlight_pin_right,HIGH); 
}
