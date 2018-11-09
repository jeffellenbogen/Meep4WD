//XBee setup
//Adding feedback to the Joystick LCD

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
#define POP_DELAY 150

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);


Servo servo1;
int currentSpeed = 100;
bool drivingForward;

// if the Meep hasn't received anything from the joystick 
// for a while, then stop driving.
// Units:  ms
#define KEEP_ALIVE_TIME 1000

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

void loop() 
{
  static unsigned long last_rx_char_time=0;
  unsigned long        current_time;
  static bool          stopped=true;
  
  if (XBee.available())
  {
        char c = XBee.read();

        last_rx_char_time = millis();
        
        if (c == '1')
        {
          stopped=false;
          driveForwardSlightLeft();
          XBee.print('1');
        }                
        else if (c == '2')
        {
          stopped = false;
          
          if (drivingForward)
          {
             driveForward();
             XBee.print('2');
          }
          else 
          {
             stopDriving();
             delay(POP_DELAY);
             driveForward();
             XBee.print('2');
           }
         }
         else if (c == '3')
         {
            stopped = false;
            driveForwardSlightRight(); 
            XBee.print('3');  
         }        
         else if (c == '4')
         {
            stopped = false;
            if (drivingForward)
            {
              driveLeft();
              XBee.print('4');
            }
            else
            {
              driveLeftBack(); 
              XBee.print('4');  // This might need to be fixed to deal with forward vs back left commands
            }     
         }      
         else if (c == '5')
         {
            stopped = true;
            stopDriving(); 
            XBee.print('5');    
         }   
         else if (c == '6')
         {
            stopped = false;
            if (drivingForward)
            {
              driveRight();
              XBee.print('6');
            }
            else
            {
              driveRightBack(); 
              XBee.print('6');  // This might need to be fixed to deal with forward vs back right commands
            } 
         }        
         else if (c == '7')
         {
            stopped = false;
            driveBackSlightLeft();
            XBee.print('7');
         }
         else if (c == '8')
         {
            stopped = false;
            if (drivingForward==false)
            {
              driveBack();
              XBee.print('8');
            }
            else 
            {
              stopDriving();
              delay(POP_DELAY);
              driveBack();
              XBee.print('8');
             }
          }
          else if (c == '9')
          {
            stopped = false;
            driveBackSlightRight();
            XBee.print('9');  
          }     
         else if (c == 'R')
         {
            regSpeed();
            XBee.print('R');
         }
         else if (c == 'T')
         {
            turboSpeed();
            XBee.print('T');
         }
         else if (c == 'S')
         {
            slowSpeed();
            XBee.print('S');                    
         }
   }  // end if character available

   // if we haven't received anything from the Joystick in a while, 
   // it's likely we've lost contact.  We should stop.
   current_time = millis();
   if ((stopped == false) && (current_time > last_rx_char_time + KEEP_ALIVE_TIME))
   {
     stopped = true;
     stopDriving();
     Serial.println("Lost contact w/Joystick.  Stopping");
   }
}

void stopDriving(){
  headlightsOff();
  servo1.write(heading_baseline);
  frontMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
}

void driveForward(){
  drivingForward = true;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveBack(){
  drivingForward = false;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
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
}
void driveForwardSlightLeft(){
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveForwardSlightRight(){
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}

void driveBackSlightRight(){
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}


void driveBackSlightLeft(){
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
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
