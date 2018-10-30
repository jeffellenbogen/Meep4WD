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
#define TURN_SLIGHT_LEFT  (heading_baseline-small_turn_amt)
#define TURN_HARD_LEFT    (heading_baseline-large_turn_amt)
#define TURN_SLIGHT_RIGHT (heading_baseline+small_turn_amt)
#define TURN_HARD_RIGHT   (heading_baseline+large_turn_amt)
#define TURN_STRAIGHT      heading_baseline

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontMotor = AFMS.getMotor(1);
Adafruit_DCMotor *backLeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *backRightMotor = AFMS.getMotor(4);

//  Delay, in ms, for forward->back and back->forward transitions.
#define PAUSE_TIME 250 

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
stateType invalidCommand();
stateType driveForward();
stateType driveForwardWithPause();
stateType driveBack();
stateType driveBackWithPause();
stateType stopDriving();
stateType driveHardLeftForward();
stateType driveHardLeftBack();
stateType driveHardRightForward();
stateType driveHardRightBack();
stateType turnSlightLeft();
stateType turnSlightRight();
stateType turnStraight();
stateType slowSpeed();
stateType regSpeed();
stateType turboSpeed();

// The state machine table itself.  Functions map, in order, to the 
// input definitions above.
typedef stateType (*functionType)();
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
 * STATE MACHINE Function:  invalidCommand
 */
stateType invalidCommand()
{
  /* do nothing.  Error message printed by processCommand */
}


/**************************************************************************************
 * STATE MACHINE Function:  driveForward
 */
stateType driveForward()
{
  runMotors(FORWARD);
  return STATE_DRIVING_FORWARD;
  
} // end of driveForward

/**************************************************************************************
 * STATE MACHINE Function:  driveBack
 */
stateType driveBack()
{
  runMotors(BACKWARD);
  return STATE_DRIVING_BACK;
  
} // end of driveForward
/**************************************************************************************
 * STATE MACHINE Function:  stopDriving
 */
stateType stopDriving()
{
  runMotors(RELEASE);
  return currentState;
      
}  //  end of stopDriving

/**************************************************************************************
 * STATE MACHINE Function:  driveForwardWithPause
 */
stateType driveForwardWithPause()
{
  runMotors(RELEASE);
  delay(PAUSE_TIME);
  runMotors(FORWARD);

  return STATE_DRIVING_FORWARD;
}

/**************************************************************************************
 * STATE MACHINE Function:  driveBackWithPause
 */
stateType driveBackWithPause()
{
  runMotors(RELEASE);
  delay(PAUSE_TIME);
  runMotors(BACKWARD);

  return STATE_DRIVING_BACK;
}

/**************************************************************************************
 * STATE MACHINE Function:  driveHardLeftForward
 */
stateType driveHardLeftForward()
{
  servo1.write(TURN_HARD_LEFT);
  runMotors(FORWARD);

  return STATE_DRIVING_FORWARD;
}

/**************************************************************************************
 * STATE MACHINE Function:  driveHardLeftBack
 */
stateType driveHardLeftBack()
{
  servo1.write(TURN_HARD_LEFT);
  runMotors(BACKWARD);

  return STATE_DRIVING_BACK;
}

/**************************************************************************************
 * STATE MACHINE Function:  driveHardRightForward
 */
stateType driveHardRightForward()
{
  servo1.write(TURN_HARD_RIGHT);
  runMotors(FORWARD);

  return STATE_DRIVING_FORWARD;
}

/**************************************************************************************
 * STATE MACHINE Function:  driveHardLeftBack
 */
stateType driveHardRightBack()
{
  servo1.write(TURN_HARD_RIGHT);
  runMotors(BACKWARD);

  return STATE_DRIVING_BACK;
}

/**************************************************************************************
 * STATE MACHINE Function:  turnSlightLeft()
 */
stateType turnSlightLeft()
{
  servo1.write(TURN_SLIGHT_LEFT);
  return currentState;
}

/**************************************************************************************
 * STATE MACHINE Function:  turnSlightRight()
 */
stateType turnSlightRight()
{
  servo1.write(TURN_SLIGHT_RIGHT);
  return currentState;
}


/**************************************************************************************
 * STATE MACHINE Function:  turnStraight()
 */
stateType turnStraight()
{
  servo1.write(TURN_STRAIGHT);
  return currentState;
}

/**************************************************************************************
 * STATE MACHINE Function:  regSpeed
 */
stateType regSpeed()
{
  currentSpeed = 100;
  setCurrentSpeed();

  return currentState;
}

/**************************************************************************************
 * STATE MACHINE Function:  turboSpeed
 */
stateType turboSpeed()
{
  currentSpeed = 255;
  setCurrentSpeed();

  return currentState;
}

/**************************************************************************************
 * STATE MACHINE Function:  slowSpeed
 */
stateType slowSpeed()
{
  currentSpeed = 50;
  setCurrentSpeed();

  return currentState;
}

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
    currentState = stateMachineTable[input][currentState]();
  }

}  // end of loop

/**************************************************************************************
 * Function:  runMotors
 */
void runMotors(int dir)
{
  frontMotor->run(dir);
  backLeftMotor->run(dir);
  backRightMotor->run(dir);
} // end of driveForward

/**************************************************************************************
 * Function:  setCurrentSpeed
 */
void setCurrentSpeed()
{
  frontMotor->setSpeed(currentSpeed);
  backLeftMotor->setSpeed(currentSpeed);
  backRightMotor->setSpeed(currentSpeed);

}
