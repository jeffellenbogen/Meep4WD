//XBee setup
#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 


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


}

void loop() {
 if (XBee.available())
     {
     char c = XBee.read();
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

  // send ack.  Magic number for now.
  XBee.print("A5");
}

void driveForward(){
  drivingForward = true;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  // send ack.  Magic number for now.
  XBee.print("A2");

}

void driveBack(){
  drivingForward = false;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  // send ack.  Magic number for now.
  XBee.print("A8");

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

  // send ack.  Magic number for now.
  XBee.print("A4");

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

  // send ack.  Magic number for now.
  XBee.print("A6");

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

  // send ack.  Magic number for now.
  XBee.print("A4");

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

  // send ack.  Magic number for now.
  XBee.print("A6");

}

void driveForwardSlightLeft(){
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  // send ack.  Magic number for now.
  XBee.print("A1");

}

void driveForwardSlightRight(){
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);

  // send ack.  Magic number for now.
  XBee.print("A3");

}

void driveBackSlightRight(){
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  // send ack.  Magic number for now.
  XBee.print("A9");

}


void driveBackSlightLeft(){
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);

  // send ack.  Magic number for now.
  XBee.print("A7");

}


void regSpeed(){
  currentSpeed = 100;
  setCurrentSpeed();
}

void turboSpeed(){
  currentSpeed = 255;
  setCurrentSpeed();
}

void slowSpeed(){
  currentSpeed = 50;
  setCurrentSpeed();
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
