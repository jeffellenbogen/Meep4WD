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

// State machine inputs.
typedef enum
{
  INPUT_INVALID,        
  INPUT_FWD_LEFT,       // '1'
  INPUT_FWD,            // '2'
  INPUT_FWD_RIGHT,      // '3'
  INPUT_LEFT,           // '4'
  INPUT_STOP,           // '5'
  INPUT_RIGHT,          // '6'
  INPUT_BACK_LEFT,      // '7'
  INPUT_BACK,           // '8'
  INPUT_BACK_RIGHT,     // '9'
  INPUT_SLOWMO_SPEED,   // 'S'
  INPUT_REGULAR_SPEED,  // 'R'
  INPUT_TURBO_SPEED     // 'T'
} inputType;

// Forward function declarations needed for the function table below.
void invalidCommand();
void driveForwardSlightLeft();
void processForward();
void driveForwardSlightRight();
void processLeft();
void stopDriving();
void processRight();
void driveBackSlightLeft();
void processBack();
void driveBackSlightRight();
void slowSpeed();
void regSpeed();
void turboSpeed();

// The state machine table itself.  Functions map, in order, to the 
// input definitions above.
typedef void (*functionType)();
functionType functionTable[]  =
{
  invalidCommand,             // INPUT_INVALID
  driveForwardSlightLeft,     // INPUT_FWD_LEFT
  processForward,             // INPUT_FWD
  driveForwardSlightRight,    // INPUT_FWD_RIGHT
  processLeft,                // INPUT_LEFT
  stopDriving,                // INPUT_STOP
  processRight,               // INPUT_RIGHT
  driveBackSlightLeft,        // INPUT_BACK_LEFT
  processBack,                // INPUT_BACK
  driveBackSlightRight,       // INPUT_BACK_RIGHT
  slowSpeed,                  // INPUT_SLOWMO_SPEED
  regSpeed,                   // INPUT_REGULAR_SPEED
  turboSpeed                  // INPUT_TURBO_SPEED
};

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
     }
     else 
     {
        stopDriving();
        delay(250);
        driveForward();
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
      }
      else
      {
         driveLeftBack(); 
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
      }
      else
      {
         driveRightBack(); 
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
      }
      else 
      {
         stopDriving();
         delay(250);
         driveBack();
      }
}

void invalidCommand()
{
  /* do nothing */
}

/**************************************************************************************
 * Function:  mapCommandToInput
 */
inputType mapCommandToInput( char command )
{
  inputType retVal;
  
  // number characters are easy to map...do a char to int converstion.
  if ((command >= '1') && (command <= '9'))
  {
    retVal = (inputType) (command - '0');
  }
  else
  {
    // Three other inputs we need to process are the speed inputs
    switch (command)
    {
      case 'S':
        retVal = INPUT_SLOWMO_SPEED;
      break;

      case 'R':
        retVal = INPUT_REGULAR_SPEED;
      break;

      case 'T':
        retVal = INPUT_TURBO_SPEED;
      break;

      default:
        Serial.print("Invalid command received: ");
        Serial.println(command);
        retVal = INPUT_INVALID;
    }
  }

  return retVal;
}

/**************************************************************************************
 * Function:  processCommand
 */
void processCommand(char c)
{
  inputType input;

  input = mapCommandToInput(c);
  
  functionTable[input]();
  
  // Ack the command
  XBee.print(c);
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
