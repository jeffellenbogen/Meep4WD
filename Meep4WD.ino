//XBee setup
//Adding feedback to the Joystick LCD

#include <SoftwareSerial.h>
SoftwareSerial XBee(2, 3); // Arduino RX, TX (XBee Dout, Din)

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

#include "intQueue.h"  // input queue used for state machine operations.

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
  INPUT_GO_FORWARD,
  INPUT_GO_BACK,
  INPUT_STOP,           
  INPUT_GO_HARD_LEFT,
  INPUT_GO_HARD_RIGHT,
  INPUT_TURN_SLIGHT_LEFT,
  INPUT_TURN_SLIGHT_RIGHT,
  INPUT_TURN_STRAIGHT,
  INPUT_SLOWMO_SPEED,   
  INPUT_REGULAR_SPEED,  
  INPUT_TURBO_SPEED,    
  NUM_INPUTS
} inputType;

intQueue inputQ;

// State machine states
typedef enum
{
  STATE_DRIVING_FORWARD,
  STATE_DRIVING_BACK,
  NUM_STATES
} stateType;

stateType currentState;

// Forward function declarations needed for the function table below.
void invalidCommand();
void driveForward();
void driveForwardWithPause();
void driveBack();
void driveBackWithPause();
void stopDriving();
void driveHardLeftForward();
void driveHardLeftBack();
void driveHardRightForward();
void driveHardRightBack();
void turnSlightLeft();
void turnSlightRight();
void turnStraight();
void slowSpeed();
void regSpeed();
void turboSpeed();

// The state machine table itself.  Functions map, in order, to the 
// input definitions above.
typedef void (*functionType)();
functionType stateMachineTable[NUM_INPUTS][NUM_STATES]  =
{ 
  // STATE_DRIVING_FORWARD    STATE_DRIVING_BACK 
  {invalidCommand,            invalidCommand},          // INPUT_INVALID
  {driveForward,              driveForwardWithPause},   // INPUT_GO_FORWARD
  {driveBackWithPause,        driveBack},               // INPUT_GO_BACK
  {stopDriving,               stopDriving},             // INPUT_STOP
  {driveHardLeftForward,      driveHardLeftBack},       // INPUT_GO_HARD_LEFT
  {driveHardRightForward,     driveHardRightBack},      // INPUT_GO_HARD_RIGHT
  {turnSlightLeft,            turnSlightLeft},          // INPUT_TURN_SLIGHT_LEFT
  {turnSlightRight,           turnSlightRight},         // INPUT_TURN_SLIGHT_RIGHT
  {turnStraight,              turnStraight},            // INPUT_TURN_STRAIGHT
  {slowSpeed,                 slowSpeed},               // INPUT_SLOWMO_SPEED
  {regSpeed,                  regSpeed},                // INPUT_REGULAR_SPEED
  {turboSpeed,                turboSpeed}               // INPUT_TURBO_SPEED
};

Servo servo1;
int currentSpeed = 100;

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

  // if the first thing the meep receives is a left or right, we
  // want to go forward.
  currentState = STATE_DRIVING_FORWARD;
  
}  //  end of setup

/**************************************************************************************
 * Function:  driveForwardWithPause
 */
void driveForwardWithPause()
{
  stopDriving();
  delay(250);
  driveForward();
}

/**************************************************************************************
 * Function:  driveBackWithPause
 */
void driveBackWithPause()
{
  stopDriving();
  delay(250);
  driveBack();
}

/**************************************************************************************
 * Function:  driveHardLeftForward
 */
void driveHardLeftForward()
{
  turnHardLeft();
  driveForward();
}

/**************************************************************************************
 * Function:  driveHardLeftBack
 */
void driveHardLeftBack()
{
  turnHardLeft();
  driveBack();
}

/**************************************************************************************
 * Function:  driveHardRightForward
 */
void driveHardRightForward()
{
  turnHardRight();
  driveForward();
}

/**************************************************************************************
 * Function:  driveHardLeftBack
 */
void driveHardRightBack()
{
  turnHardRight();
  driveBack();
}

/**************************************************************************************
 * Function:  invalidCommand
 */
void invalidCommand()
{
  /* do nothing.  Error message printed by processCommand */
}

/**************************************************************************************
 * Function:  processCommand
 */
void processCommand(char c)
{
  int input;

  switch (c)
  {
    case '1':
      inputQ.put(INPUT_TURN_SLIGHT_LEFT);
      inputQ.put(INPUT_GO_FORWARD);
    break;

    case '2':
      inputQ.put(INPUT_TURN_STRAIGHT);
      inputQ.put(INPUT_GO_FORWARD);
    break;

    case '3':
      inputQ.put(INPUT_TURN_SLIGHT_LEFT);
      inputQ.put(INPUT_GO_FORWARD);
    break;

    case '4':
      inputQ.put(INPUT_GO_HARD_LEFT);
    break;

    case '5':
      inputQ.put(INPUT_STOP);
    break;

    case '6':
      inputQ.put(INPUT_GO_HARD_RIGHT);
    break;

    case '7':
      inputQ.put(INPUT_TURN_SLIGHT_LEFT);
      inputQ.put(INPUT_GO_BACK);
    break;

    case '8':
      inputQ.put(INPUT_TURN_STRAIGHT);
      inputQ.put(INPUT_GO_BACK);
    break;

    case '9':
      inputQ.put(INPUT_TURN_SLIGHT_RIGHT);
      inputQ.put(INPUT_GO_BACK);
    break;

    case 'S':
      inputQ.put(INPUT_SLOWMO_SPEED);
    break;

    case 'R':
      inputQ.put(INPUT_REGULAR_SPEED);
    break;

    case 'T':
      inputQ.put(INPUT_TURBO_SPEED);
    break;

    default:
      Serial.print("Received unknown command from Joystick: ");
      Serial.println(c);
  }
  
  // Ack the command
  // Note:  yes, we're now acking this before we actually act on the command.
  // We *could* move this back into the appropriate processing functions, but
  // the purpose of the Ack is really to show that the command wasn't dropped.
  // Can revisit later if need be.
  XBee.print(c);
}

/**************************************************************************************
 * Function:  loop
 */
void loop() 
{
  char command;
  int  input;
  
  if (XBee.available())
  {
    command = XBee.read();

    processCommand(command);
  }

  // process any inputs for our state machine
  while (inputQ.numElements())
  {
    inputQ.get(&input);
    stateMachineTable[input][currentState]();
  }

}  // end of loop

/**************************************************************************************
 * Function:  stopDriving
 */
void stopDriving()
{
  frontMotor->run(RELEASE);
  backLeftMotor->run(RELEASE);
  backRightMotor->run(RELEASE);
}  //  end of stopDriving

/**************************************************************************************
 * Function:  driveForward
 */
void driveForward()
{
  currentState=STATE_DRIVING_FORWARD;
  frontMotor->run(FORWARD);
  backLeftMotor->run(FORWARD);
  backRightMotor->run(FORWARD);
} // end of driveForward

/**************************************************************************************
 * Function:  driveBack
 */
void driveBack()
{
  currentState = STATE_DRIVING_BACK;
  frontMotor->run(BACKWARD);
  backLeftMotor->run(BACKWARD);
  backRightMotor->run(BACKWARD);
}  // end of driveBack

/**************************************************************************************
 * Function:  turnHardLeft
 */
void turnHardLeft()
{
  servo1.write(heading_baseline-large_turn_amt);
}  // end of turnHardLeft

/**************************************************************************************
 * Function:  turnHardRight
 */
void turnHardRight()
{
  servo1.write(heading_baseline+large_turn_amt);
}  // end of turnHardRight

/**************************************************************************************
 * Function:  turnSlightLeft
 */
void turnSlightLeft()
{
  servo1.write(heading_baseline-small_turn_amt);
}  // end of turnSlightLeft

/**************************************************************************************
 * Function:  turnSlightRight
 */
void turnSlightRight()
{
  servo1.write(heading_baseline+small_turn_amt);
}  // end of turnSlightRight

/**************************************************************************************
 * Function:  turnStraight
 */
void turnStraight()
{
  servo1.write(heading_baseline);
}  // end of turnStraight

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
