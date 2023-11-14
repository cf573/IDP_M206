#include <Adafruit_MotorShield.h>

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Ports

int frontLineSensorPin = 5;
int leftLineSensorPin = 4;
int backLineSensorPin = 3;
int rightLineSensorPin = 2;

int valFront = 0, valLeft = 0, valBack = 0, valRight = 0;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Motor constants

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

int MAX_SPEED = 255;
int LINE_FOLLWOING_INTERVAL = 200; // let the motor run 50ms before each sensing & action to provide fierce change of commands
int TURNING_TIME_INTERVAL = 1250; // the time motor takes to turn to 90 degrees in full speed

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// sensor reading mode constants
// L means that the left sensor reads white, etc.
// the sensor readings are binary encoded into one integer; sensorReadingNum = valLeft * 8 + valFront * 4 + valBack * 2 + valRight

#define L 8
#define W 4
#define B 2
#define R 1
#define LW 12
#define LB 10
#define LR 9
#define WB 6
#define WR 5
#define BR 3
#define LWB 14
#define LWR 13
#define LBR 11
#define WBR 7
#define LWBR 15

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// basic functions, including sensor reading and interval motor control

int readLineSensor(){
  valFront = digitalRead(frontLineSensorPin);
  valLeft = digitalRead(leftLineSensorPin);
  valBack = digitalRead(backLineSensorPin);
  valRight = digitalRead(rightLineSensorPin);
  int sensorReadingNum = valLeft * 8 + valFront * 4 + valBack * 2 + valRight;
  return sensorReadingNum;
}

void goFront(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED);
}

void goLeft(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(MAX_SPEED);
}

void goRight(){
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED);
}

void goBack(){
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(MAX_SPEED);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of different working mode
//stop_condition: class int, a sensor reading pattern that educates the robot to change into another mode of motion

// Go straight, whatever
void goStraight(int stop_condition){
  boolean quitted = false;
  int sensorReadingInt = 0;
  while(sensorReadingInt != stop_condition || !quitted){
    sensorReadingInt = readLineSensor();
    if (sensorReadingInt != stop_condition){
      quitted = true;
    }
    goFront();
  }
}

// Turn left, whatever
void turnLeft(int stop_condition){
  boolean quitted = false;
  int sensorReadingInt = 0;
  while(sensorReadingInt != stop_condition || !quitted){
    sensorReadingInt = readLineSensor();
    if (sensorReadingInt != stop_condition){
      quitted = true;
    }
    goLeft();
  }
}

//Turn right, whatever
void turnRight(int stop_condition){
  boolean quitted = false;
  int sensorReadingInt = 0;
  while(sensorReadingInt != stop_condition || !quitted){
    sensorReadingInt = readLineSensor();
    if (sensorReadingInt != stop_condition){
      quitted = true;
    }
    goRight();
  }
}

void turnLeft90Deg(){
  goLeft();
  delay(TURNING_TIME_INTERVAL);
  Serial.print('?');
}

void turnRight90Deg(){
  goRight();
  delay(TURNING_TIME_INTERVAL);
}


// Follow the white line
void followLine(int stop_condition){
  boolean quitted = false;
  int sensorReadingInt = 0;
  while(sensorReadingInt != stop_condition || !quitted){
    sensorReadingInt = readLineSensor();
    if (sensorReadingInt != stop_condition){
      quitted = true;
    }
    switch(sensorReadingInt){
      case 0:
        goFront();
        break;
      case L:
        goLeft();
        delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case W:
        goFront();
        break;
      case B:
        goFront();
        break;
      case R:
        goRight();
        delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LW:
        // goLeft();
        // delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LB:
        goLeft();
        delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LR:
        goFront();
        break;
      case WB:
        goFront();
        break;
      case WR:
        // goRight();
        // delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case BR:
        // goRight();
        // delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LWB:
        // goLeft();
        // delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LWR:
        goFront();
        break;
      case LBR:
        goFront();
        break;
      case WBR:
        // goRight();
        // delay(LINE_FOLLWOING_INTERVAL);
        goFront();
        break;
      case LWBR:
        goFront();
        break;
    }
    delay(0.5 * LINE_FOLLWOING_INTERVAL);
  }
}




// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of two tasks

// Task 1: Go around the grids & collect 2 blocks on them
void task1(){
  goStraight(LWR);
  followLine(LWBR); // reached the cross
  turnLeft90Deg();
  followLine(WBR);
  followLine(LBR); // reached first corner
  turnRight90Deg();
  followLine(WBR); // reached second corner
  turnRight90Deg();
  followLine(WBR);
  followLine(WBR);
  followLine(WBR);
  followLine(LBR); // reached third corner
  turnRight90Deg();
  followLine(WBR); // reached fourth corner
  turnRight90Deg();
  followLine(WBR);
  followLine(LWBR); // back to the cross
  turnRight90Deg();// end
  return;
}

void task2(){
  return;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// main functions

void setup() {
  // Initialize serial port and input pins
  Serial.begin(9600);
  pinMode(leftLineSensorPin, INPUT);
  pinMode(frontLineSensorPin, INPUT);
  pinMode(rightLineSensorPin, INPUT);
  pinMode(backLineSensorPin, INPUT);

  // turn on motor
  // leftMotor->run(FORWARD);
  // leftMotor->setSpeed(MAX_SPEED);
  // leftMotor->run(RELEASE);
  // rightMotor->run(FORWARD);
  // rightMotor->setSpeed(MAX_SPEED);
  // rightMotor->run(RELEASE); 
}

void loop(){
    if (!AFMS.begin()){         // create with the default frequency 1.6KHz
      Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }
  Serial.println("Motor Shield found.");
  task1();
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  while(1);
}
