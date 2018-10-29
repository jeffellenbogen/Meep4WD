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

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);


Servo servo1;
int currentSpeed = 100;
bool drivingForward;

/**************************************************************************************
 * Function:  setup
 */
void setup() 
{
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
}  //  end of setup

/**************************************************************************************
 * Function:  processForward
 */
void processForward()
{
     if (drivingForward)
     {
        driveForward();
        XBee.print('2');
     }
     else 
     {
        stopDriving();
        delay(250);
        driveForward();
        XBee.print('2');
     }
  
}


/**************************************************************************************
 * Function:  processLeft
 */
void processLeft()
{
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


/**************************************************************************************
 * Function:  processRight
 */
void processRight()
{
           
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


/**************************************************************************************
 * Function:  processBack
 */
void processBack()
{
      if (drivingForward==false)
      {
         driveBack();
         XBee.print('8');
      }
      else 
      {
         stopDriving();
         delay(250);
         driveBack();
         XBee.print('8');
      }
}

/**************************************************************************************
 * Function:  processCommand
 */
void processCommand(char c)
{
  if (c == '1')
  {
     driveForwardSlightLeft();
     XBee.print('1');
  }                
  else if (c == '2')
  {
     processForward();
  }
  else if (c == '3')
  {
     driveForwardSlightRight(); 
     XBee.print('3');  
  }        
  else if (c == '4')
  {
     processLeft();
  }      
  else if (c == '5')
  {
     stopDriving(); 
     XBee.print('5');    
  }   
  else if (c == '6')
  {
     processRight();
  }        
  else if (c == '7')
  {
     driveBackSlightLeft();
     XBee.print('7');
  }
  else if (c == '8')
  {
     processBack();
  }
  else if (c == '9')
  {
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
}

/**************************************************************************************
 * Function:  loop
 */
void loop() 
{
  char command;
  
  if (XBee.available())
  {
    command = XBee.read();

    processCommand(command);
  }

}  // end of loop

/**************************************************************************************
 * Function:  stopDriving
 */
void stopDriving()
{
  headlightsOff();
  servo1.write(heading_baseline);
  frontMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
}  //  end of stopDriving

/**************************************************************************************
 * Function:  driveForward
 */
void driveForward()
{
  drivingForward = true;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
} // end of driveForward

/**************************************************************************************
 * Function:  driveBack
 */
void driveBack()
{
  drivingForward = false;
  headlightsOn();
  servo1.write(heading_baseline);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}  // end of driveBack

/**************************************************************************************
 * Function:  driveLeft
 */
void driveLeft()
{
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-large_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
  delay(50);
  headlightsOff();
  delay(50);
}  // end of driveLeft

/**************************************************************************************
 * Function:  driveRight
 */
void driveRight()
{
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+large_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
  delay(50);
  headlightsOff();
  delay(50);
}  // end of driveRight

/**************************************************************************************
 * Function:  driveLeftBack
 */
void driveLeftBack()
{
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-large_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
  delay(50);
  headlightsOff();
  delay(50);
}  // end of driveLeftBack

/**************************************************************************************
 * Function:  driveRightBack
 */
void driveRightBack()
{
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+large_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
  delay(50);
  headlightsOff();
  delay(50);
}  // end of driveRightBack

/**************************************************************************************
 * Function:  driveForwardSlightLeft
 */
void driveForwardSlightLeft()
{
  drivingForward = true;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}  // end of driveForwardSlightLeft

/**************************************************************************************
 * Function:  driveForwardSlightRight
 */
void driveForwardSlightRight()
{
  drivingForward = true;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
}  // end of driveForwardSlightRight

/**************************************************************************************
 * Function:  driveBackSlightRight
 */
void driveBackSlightRight()
{
  drivingForward = false;
  headlightsRightBlinker();
  servo1.write(heading_baseline+small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}  // end of driveBackSlightRight

/**************************************************************************************
 * Function:  driveBackSlightLeft
 */
void driveBackSlightLeft()
{
  drivingForward = false;
  headlightsLeftBlinker();
  servo1.write(heading_baseline-small_turn_amt);
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}  // end of driveBackSlightLeft

/**************************************************************************************
 * Function:  regSpeed
 */
void regSpeed()
{
  currentSpeed = 100;
  setCurrentSpeed();
}

/**************************************************************************************
 * Function:  turboSpeed
 */
void turboSpeed()
{
  currentSpeed = 255;
  setCurrentSpeed();
}

/**************************************************************************************
 * Function:  slowSpeed
 */
void slowSpeed()
{
  currentSpeed = 50;
  setCurrentSpeed();
}

/**************************************************************************************
 * Function:  setCurrentSpeed
 */
void setCurrentSpeed()
{
  frontMotor->setSpeed(currentSpeed);
  backLeftMotor->setSpeed(currentSpeed);
  backRightMotor->setSpeed(currentSpeed);

}

/**************************************************************************************
 * Function:  headlightsOn
 */
void headlightsOn()
{
  digitalWrite(headlight_pin_left,HIGH);
  digitalWrite(headlight_pin_right,HIGH);
}

/**************************************************************************************
 * Function:  headlightsOff
 */
void headlightsOff()
{
  digitalWrite(headlight_pin_left,LOW);
  digitalWrite(headlight_pin_right,LOW); 
}

/**************************************************************************************
 * Function:  headlightsLeftBlinker
 */
void headlightsLeftBlinker()
{
   digitalWrite(headlight_pin_left,HIGH);
   digitalWrite(headlight_pin_right,LOW); 
}

/**************************************************************************************
 * Function:  headlightsRightBlinker
 */
void headlightsRightBlinker()
{
   digitalWrite(headlight_pin_left,LOW);
   digitalWrite(headlight_pin_right,HIGH); 
}
