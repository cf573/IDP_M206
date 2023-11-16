#include <Adafruit_MotorShield.h>

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Ports

int frontLineSensorPin = 3;
int leftLineSensorPin = 2;
int backLineSensorPin = 4;
int rightLineSensorPin = 5;

int valFront = 0, valLeft = 0, valBack = 0, valRight = 0;

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Motor constants

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

int MAX_SPEED = 200;
int LINE_FOLLOWING_INTERVAL = 10; // let the motor run 50ms before each sensing & action to provide fierce change of commands
int TURNING_TIME_INTERVAL = 1020; // the time motor takes to turn to 90 degrees in full speed

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// sensor reading mode constants

#define L 1 // (left & front) | (left & rear)
#define R 2 // (right & front) | (right & rear)
#define LR 3 // left & right

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// basic functions, including sensor reading and interval motor control

// the line sensor readings are binary encoded into one integer; sensorReadingNum = valLeft * 8 + valFront * 4 + valBack * 2 + valRight
int readLineSensor(){
  valFront = digitalRead(frontLineSensorPin);
  valLeft = digitalRead(leftLineSensorPin);
  valBack = digitalRead(backLineSensorPin);
  valRight = digitalRead(rightLineSensorPin);
  int sensorReadingNum = valLeft * 8 + valFront * 4 + valBack * 2 + valRight;
  return sensorReadingNum;
}


float readUltrasound(){
  // to be developed by 2nd coder
  return 10000.0;
}


float readDistance(){
  // to be developed by 2nd coder
  return 10000.0;
}


int readMagnetic(){
  return 0;
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


void slightLeft(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(0.6 * MAX_SPEED);
}


void slightRight(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(0.6 * MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED);
}


// open the claw
void clawOpen(){
  // to be developed by 2nd coder
  return;
}


// close the claw to catch the block
void clawClose(){
  // to be developed by 2nd coder
  return;
}


// change LED according to the rules. Please specify ports of the LED in the comments.
// condition parameter can be encoded however for your convenience.
void changeLED(int condition){
  // to be developed by 2nd coder
  return;
}


// start the program only after the button is pressed; return true when it is pressed.
void bottonControl(){
  // to be developed by 2nd coder
  return false;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of different working mode
//stop_condition: class int, a sensor reading pattern that educates the robot to change into another mode of motion


// bool foundBlock(){
//   return false;
// }

bool isMagnetic(){
  return readMagnetic == 1;
}

// Go straight, whatever
void goStraight(int stop_condition){
  while(1){
    int sensorReadingInt = readLineSensor();
    int left = sensorReadingInt / 8;
    int front = (sensorReadingInt % 8) / 4;
    int back = (sensorReadingInt % 4) / 2;
    int right = sensorReadingInt % 2;

    if(stop_condition == LR && left == 1 && right == 1){return;}
    if(stop_condition == L && ((left == 1 && front == 1) || (left == 1 && back == 1))){return;}
    if(stop_condition == R && ((right == 1 && front == 1) || (right == 1 && back == 1))){return;}

    goFront();
    delay(LINE_FOLLOWING_INTERVAL);
  }
}


// // Turn left, whatever
// void turnLeft(int stop_condition){
//   boolean quitted = false;
//   int sensorReadingInt = 0;
//   while(sensorReadingInt != stop_condition || !quitted){
//     sensorReadingInt = readLineSensor();
//     if (sensorReadingInt != stop_condition){
//       quitted = true;
//     }
//     goLeft();
//   }
// }


// //Turn right, whatever
// void turnRight(int stop_condition){
//   boolean quitted = false;
//   int sensorReadingInt = 0;
//   while(sensorReadingInt != stop_condition || !quitted){
//     sensorReadingInt = readLineSensor();
//     if (sensorReadingInt != stop_condition){
//       quitted = true;
//     }
//     goRight();
//   }
// }


void turnLeft90Deg(){
  goLeft();
  delay(TURNING_TIME_INTERVAL);
  Serial.print('?');
}


void turnRight90Deg(){
  goRight();
  delay(TURNING_TIME_INTERVAL);
}

void turn180Deg(){
  goLeft();
  delay(2 * TURNING_TIME_INTERVAL);
}

void stepForward(){
  goFront();
  delay(0.5 * TURNING_TIME_INTERVAL);
}


// Follow the white line
bool followLine(int stop_condition){
  while(1){
    int sensorReadingInt = readLineSensor();
    int distance = readDistance();
    if(distance < 100.0){return true;}
    int left = sensorReadingInt / 8;
    int front = (sensorReadingInt % 8) / 4;
    int back = (sensorReadingInt % 4) / 2;
    int right = sensorReadingInt % 2;

    if(stop_condition == LR && left == 1 && right == 1){return false;}
    if(stop_condition == L && ((left == 1 && front == 1) || (left == 1 && back == 1))){return false;}
    if(stop_condition == R && ((right == 1 && front == 1) || (right == 1 && back == 1))){return false;}
    if(left == 1){slightLeft();}
    else if(right == 1){slightRight();}
    else{goFront();}

    delay(LINE_FOLLOWING_INTERVAL);
  }
}

void gotoStartingPoint(){
  followLine(LR);
  stepForward();
  goStraight(LR);
  return;
}

void releaseGreen(){
  return;
}

void releaseRed(){
  return;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of two tasks

// Do the movement according to the junction it currently at
// true return indicates that a block is found; false return indicates that stop condition is met
bool search1(int blockProcessed, int order){
  if(blockProcessed == 0){
    switch(order){
      case 0:
        goStraight(LR);
        stepForward();
        return followLine(LR); // reached the cross
      case 1:
        turnLeft90Deg();
        stepForward();
        return followLine(LR); // reached first corner
      case 2:
        turnRight90Deg();
        stepForward();
        return followLine(R); // reached second corner
      case 3:
        turnRight90Deg();
        stepForward();
        return followLine(LR); // reached third corner
      case 4:
        turnRight90Deg();
        stepForward();
        return followLine(R); // reached fourth corner
      case 5:
        turnRight90Deg();
        stepForward();
        return followLine(LR); // back to the cross
    }
  }
  return;
}

void return1(int blockProcessed, int order){
  // to be developed by the 2nd coder
  // path of going a way back according to the frontward path; inverse of searchTask1
  return;
}


void task1(){
  int blockProcessed = 0;
  while(blockProcessed <= 1){
    clawOpen();
    int order = 0;
    while(!search1(blockProcessed, order)){order++;}
    clawClose();
    turn180Deg();
    while(order != 0){
      return1(blockProcessed, order);
      order--;
    }
    gotoStartingPoint();
    if(isMagnetic()){
      releaseGreen();
    }
    else{
      releaseRed();
    }
    blockProcessed++;
  }
}

// Task 1: Go around the grids & collect 2 blocks on them
void testLoopFollowing(){
  goStraight(LR);
  stepForward();
  followLine(LR); // reached the cross
  turnLeft90Deg();
  stepForward();
  followLine(LR); // reached first corner
  turnRight90Deg();
  stepForward();
  followLine(R); // reached second corner
  turnRight90Deg();
  stepForward();
  followLine(LR); // reached third corner
  turnRight90Deg();
  stepForward();
  followLine(R); // reached fourth corner
  turnRight90Deg();
  stepForward();
  followLine(LR); // back to the cross
  turnRight90Deg();// end
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
  testLoopFollowing();
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  while(1);
}
