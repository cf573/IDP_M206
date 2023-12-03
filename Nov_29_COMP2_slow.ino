#include <Adafruit_MotorShield.h> 
#include <Wire.h>
#include <DFRobot_VL53L0X.h>
#include <Servo.h>

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Process constants

int blockProcessed = 0; // total number of blocks processed
int magneticBlockProcessed = 0;  // total number of magnetic blocks processed
int nonMagBlockProcessed = 0;  // total number of non-magnetic blocks processed


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Sensor constants

// pins
const int magneticPin = 2;
const int ultrasoundPin = A3;
const int frontRightSensorPin = 3;
const int frontLeftSensorPin = 4;
const int rearRightSensorPin = 5;
const int rearLeftSensorPin = 6;
const int blueLEDPin = 7;
const int redLEDPin = 8;
const int greenLEDPin = 9;
// pin 10 and 11 for servos
const int buttonPin = 12;


// Time of flight
DFRobot_VL53L0X tof; // define time of flight sensor
int avgDistance = 220;  // initial guess for tof distance
const int foundBlockDistance = 70.0;  // boundary for which the bot determines discovery of a block

// Ultrasound
#define MAX_RANGE 520  // the max measurement value of the module is 520cm(a little bit longer than  effective max range) 
#define ADC_SOLUTION 1023.0 // ADC accuracy of Arduino UNO is 10bit 
const int scanBlockDistance = 60.0;  // ultrasound value of maximum allowed block scan distance

// LED for stop & stay
bool holdBlueLED = false;

// Line sensors
int fr = 0, fl = 0, rr = 0, rl = 0; // store digital readings from the line sensor

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Motor constants

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

const int MAX_SPEED = 200;  // current selected speed for forward motion of the bot
const int LEFT_TURNING_INTERVAL = 1450;  // the time motor takes to turn to 90 degrees in max speed
const int RIGHT_TURNING_INTERVAL = 1275;
const int STEP_FORWARD_INTERVAL = 350;  // the time motor takes to make a small step forward to leave a junction
const int STOP_INTERVAL = 300;  // the desired time of stopping before & after each turning
const int START_TO_DROP_INTERVAL = 5000;
const int SCAN_REVERSE_INTERVAL = 600;
const int START_REVERSE_INTERVAL = 1300;

const int LROffset = 3; // set right motor speed to be MAX_SPEED - LROffset to counter the inherent speed difference of two motors


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Junction constants

#define L 1 // rl
#define R 2 // rr
#define LR 3 // rl && rr

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// LED mode constants

#define OFF 0
#define RED 1
#define GREEN 2

#define IN 0
#define OUT 1

// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// time control for blue LED blinking
unsigned long previousTime = millis(); // the time stamp of last blue LED state change
const long timeInterval = 250; // supposed time interval of state change
int ledState = LOW;


// turn on & off red & green LED as the rule requires.
void changeLED(int condition){
  if(condition == OFF){
    digitalWrite(redLEDPin, LOW);
    digitalWrite(greenLEDPin, LOW);
  }
  else if(condition == RED){digitalWrite(redLEDPin, HIGH);}
  else if(condition == GREEN){digitalWrite(greenLEDPin, HIGH);}
  return;
}


// check state of blue LED & flash according to time when called upon
void blink(){
  unsigned long currentTime = millis();
  if(currentTime - previousTime > timeInterval && !holdBlueLED){
    previousTime = currentTime;
    if(ledState == HIGH){ledState = LOW;}
    else{ledState = HIGH;}
    digitalWrite(blueLEDPin, ledState);
  }
}


void stopAndStay(){
  holdBlueLED = true;
  digitalWrite(blueLEDPin, HIGH);
  delay(5500);
  holdBlueLED = false;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Sensor control

// the line sensor readings are binary encoded into one integer
// digitalRead always return 0 or 1
// Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
int readLineSensor(){
  fr = digitalRead(frontRightSensorPin);
  fl = digitalRead(frontLeftSensorPin);
  rr = digitalRead(rearRightSensorPin);
  rl = digitalRead(rearLeftSensorPin);
  int sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl;
  return sensorReadingNum;
}


// when time of flight sensor detects the block, return true; otherwise return false.
// basic smoothing is deployed to offset jumps and instabilities in the original circuit.
bool catchedBlock(){
  avgDistance = 14.0/15.0 * avgDistance + 1.0/15.0 * tof.getDistance();
  return avgDistance < foundBlockDistance || digitalRead(magneticPin) == 1;
}


// return true if detects magnetic block; return false if not.
bool isMagnetic(){
  for(int i = 1; i <= 75; i++){
    if(digitalRead(magneticPin) == 1){return true;}
    delay(20);
  }
  return false;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Motor control

// set the motor to go front. delay-oriented control.
void goFront(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED  - LROffset);
  blink();
}


// set the motor to turn left (self-centered). delay-oriented control.
void goLeft(){
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED);
  blink();
}


// set the motor to turn right (self-centered). delay-oriented control.
void goRight(){
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(MAX_SPEED);
  blink();
}


// set the motor to go reverse. delay-oriented control.
void goReverse(){
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(MAX_SPEED);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(MAX_SPEED);
  blink();
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of different working mode
// stop_condition: class int, a sensor reading pattern that tells the robot to change into another mode of motion
// correspondence of stop_condition and stopping pattern is stated in the constant definition part above
// #define L 1 // (left & front & rear)
// #define R 2 // (right & front & rear)


// go front for a required time. Timestamp-oriented control.
// interval: the required going-front time
void goFrontTimed(int interval){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= interval){
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(MAX_SPEED);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(MAX_SPEED - LROffset);
    blink();
    currentTime = millis();
  }
  return;
}


// go reversed for a required time. Timestamp-oriented control.
// interval: the required going-reverse time
void goReverseTimed(int interval){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= interval){
    leftMotor->run(BACKWARD);
    leftMotor->setSpeed(MAX_SPEED);
    rightMotor->run(BACKWARD);
    rightMotor->setSpeed(MAX_SPEED);
    blink();
    currentTime = millis();
  }
  return;
}


// turn left (self-centered) for 90 degrees.
void turnLeft90Deg(){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= LEFT_TURNING_INTERVAL){
    goLeft();
    blink();
    currentTime = millis();
  }
}


// turn right (self-centered) for 90 degrees.
void turnRight90Deg(){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= RIGHT_TURNING_INTERVAL){
    goRight();
    blink();
    currentTime = millis();
  }
}


// turn (self-centered) 180 degrees.
void turn180Deg(){
  // goLeft();
  // delay(TURNING_180_TIME_INTERVAL);
  // rightTurns += 2;
  turnRight90Deg();
  stop(STOP_INTERVAL);
  turnRight90Deg();
}


// go an increment forward step.
// designed to let robot go off the pattern when turning at a junction.
void stepForward(){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= STEP_FORWARD_INTERVAL){
    goFront();
    blink();
    currentTime = millis();
  }
}


// stop (stay motionless) for required time. Timestamp-oriented control.
// interval: the required going-front time
void stop(int interval){
  int startTime = millis();
  int currentTime = startTime;
  while(currentTime - startTime <= interval){
    leftMotor->run(FORWARD);
    leftMotor->setSpeed(0);
    rightMotor->run(FORWARD);
    rightMotor->setSpeed(0);
    blink();
    currentTime = millis();
  }
  return;
}


// Go straight until line sensor detects a pattern (junction) that indicates it to stop.
// stop_condition: integer that indicates stop pattern as defined at the top.
bool goStraight(int stop_condition){
  while(1){
    int sensorReadingInt = readLineSensor(); // Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
    int fr = sensorReadingInt / 8;
    int fl = (sensorReadingInt % 8) / 4;
    int rr = (sensorReadingInt % 4) / 2;
    int rl = sensorReadingInt % 2;

    if(stop_condition == LR && rr == 1 && rl == 1){return false;}
    if(stop_condition == L && rl == 1){return false;}
    if(stop_condition == R && rr == 1){return false;}

    goFront();
  }
}


void reverseStraight(int stop_condition){
  while(1){
    int sensorReadingInt = readLineSensor(); // Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
    int fr = sensorReadingInt / 8;
    int fl = (sensorReadingInt % 8) / 4;
    int rr = (sensorReadingInt % 4) / 2;
    int rl = sensorReadingInt % 2;

    if(stop_condition == LR && rr == 1 && rl == 1){return false;}
    if(stop_condition == L && rl == 1){return false;}
    if(stop_condition == R && rr == 1){return false;}

    goReverse();
  }
}


// Follow the white line until line sensor detects a pattern (junction) that indicates it to stop.
// stop_condition: integer that indicates stop pattern as defined at the top.
// searching: true indicates that it is actively finding blocks; false indicates that it is not finding blocks and may only return false.
// true return indicates that a block is found; false return indicates that stop condition is met
bool followLine(int stop_condition, bool searching){
  bool left_junction = false;
  while(1){
    int sensorReadingInt = readLineSensor(); // Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
    if(catchedBlock() && searching){return true;}
    int fr = sensorReadingInt / 8;
    int fl = (sensorReadingInt % 8) / 4;
    int rr = (sensorReadingInt % 4) / 2;
    int rl = sensorReadingInt % 2;

    if(rl == 0 && rr == 0){left_junction = true;}

    if(stop_condition == LR && rr == 1 && rl == 1 && left_junction){return false;}
    if(stop_condition == L && rl == 1 && left_junction){return false;}
    if(stop_condition == R && rr == 1 && left_junction){return false;}

    if(fr == 1 && fl == 1){goFront();}
    else if(fr == 1){goRight();}
    else if(fl == 1){goLeft();}
    else{goFront();}

    blink();
    // delay(LINE_FOLLOWING_INTERVAL);
  }
}


bool scanLine(int stop_condition, bool searching){
  bool left_junction = false;
  while(1){
    int sensorReadingInt = readLineSensor(); // Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
    if(analogRead(ultrasoundPin) < 70.0 && searching){return true;}
    int fr = sensorReadingInt / 8;
    int fl = (sensorReadingInt % 8) / 4;
    int rr = (sensorReadingInt % 4) / 2;
    int rl = sensorReadingInt % 2;

    if(rl == 0 && rr == 0){left_junction = true;}

    if(stop_condition == LR && rr == 1 && rl == 1 && left_junction){return false;}
    if(stop_condition == L && rl == 1 && left_junction){return false;}
    if(stop_condition == R && rr == 1 && left_junction){return false;}

    if(fr == 1 && fl == 1){goFront();}
    else if(fr == 1){goRight();}
    else if(fl == 1){goLeft();}
    else{goFront();}

    blink();
    // delay(LINE_FOLLOWING_INTERVAL);
  }
}


void followLineTimed(int interval){
  int currentTime = millis();
  int prevTime = currentTime;
  while(currentTime - prevTime <= interval){
    int sensorReadingInt = readLineSensor(); // Boolean encoding: sensorReadingNum = fr * 8 + fl * 4 + rr * 2 + rl
    int fr = sensorReadingInt / 8;
    int fl = (sensorReadingInt % 8) / 4;
    int rr = (sensorReadingInt % 4) / 2;
    int rl = sensorReadingInt % 2;

    if(fr == 1 && fl == 1){goFront();}
    else if(fr == 1){goRight();}
    else if(fl == 1){goLeft();}
    else{goFront();}

    blink();

    currentTime = millis();
  }
  return;
}


// release a block to the green square by pre-coded path.
// starting position being the starting point on the board.
void gotoGreen(int blockProcessed){
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  followLine(R, false);
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  followLineTimed(500);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  goFrontTimed(START_TO_DROP_INTERVAL);
  stop(STOP_INTERVAL);
  // goReverseTimed(START_TO_DROP_INTERVAL);
  reverseStraight(LR);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  followLine(L, false);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  followLine(R, false);
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  goReverseTimed(START_REVERSE_INTERVAL);
  stop(STOP_INTERVAL);
  return;
}


// release a block to the red square by pre-coded path.
// starting position being the starting point on the board.
void gotoRed(int blockProcessed){
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  followLine(L, false);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  followLineTimed(500);
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  goFrontTimed(START_TO_DROP_INTERVAL);
  stop(STOP_INTERVAL);
  // goReverseTimed(START_TO_DROP_INTERVAL);
  reverseStraight(LR);
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  followLine(R, false);
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  followLine(L, false);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  goReverseTimed(START_REVERSE_INTERVAL);
  stop(STOP_INTERVAL);
  return;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of pre-coded movement

// Do the movement according to the junction it currently at
// true return indicates that a block is found; false return indicates that stop condition is met
bool search1(int blockProcessed, int numJunction){
  if(blockProcessed == 0){
    switch(numJunction){
      case 0:
        goStraight(LR);
        stepForward();
        return followLine(LR, true); // reached the cross
      case 1:
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // reached first corner
      case 2:
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(R, true); // reached second corner
      case 3:
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // reached third corner
      case 4:
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(R, true); // reached fourth corner
      case 5:
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // back to the cross
    }
  }
  else if(blockProcessed == 1){
    switch(numJunction){
      case 0:
        goStraight(LR);
        stepForward();
        stepForward();
        return followLine(LR, true); // reached the cross
      case 1:
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // reached first corner
      case 2:
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(L, true); // reached second corner
      case 3:
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // reached third corner
      case 4:
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(L, true); // reached fourth corner
      case 5:
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return followLine(LR, true); // back to the cross
    }
  }
  return;
}


// path of going a way back according to the frontward path; inverse of searchTask1
bool return1(int blockProcessed, int numJunction){
  if(blockProcessed == 0){
    switch(numJunction){
      case 0:
        followLine(LR, false);
        return true;
      case 1:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 2:
        followLine(L, false); 
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 3:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 4:
        followLine(L, false);
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 5:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
    }
    return true;
  }
  else if(blockProcessed == 1){
    switch(numJunction){
      case 0:
        followLine(LR, false);
        return true;
      case 1:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 2:
        followLine(R, false);
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 3:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 4:
        followLine(R, false);
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
      case 5:
        followLine(LR, false);
        stop(STOP_INTERVAL);
        turnRight90Deg();
        stop(STOP_INTERVAL);
        stepForward();
        return false;
    }
    return true;
  }
}

bool search2(int numJunction, int fetchPoint){
  bool finding = false;
  if (fetchPoint == -1){finding = true;}
  switch(numJunction){
    case 0:
      stepForward();
      return scanLine(R, finding);
    case 1:
      stepForward();
      return scanLine(LR, finding);
    case 2:
      if(fetchPoint != 2){
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
      }
      stepForward();
      return scanLine(L, finding);
    case 3:
      if(fetchPoint != 3){
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
      }
      stepForward();
      return scanLine(L, finding);
    case 4:
      if(fetchPoint != 4){
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
      }
      stepForward();
      return scanLine(L, finding);
    case 5:
      if(fetchPoint != 5){
        stop(STOP_INTERVAL);
        turnLeft90Deg();
        stop(STOP_INTERVAL);
      }
      stepForward();
      return scanLine(R, finding);
    case 6:
      stepForward();
      return scanLine(R, finding);
  }
}

bool fetchBlock(){
  bool magnetic = false;
  stop(STOP_INTERVAL);
  goReverseTimed(SCAN_REVERSE_INTERVAL);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  while(!catchedBlock()){goFrontTimed(20);}
  stop(20);
  if(isMagnetic()){
    magnetic = true;
    changeLED(RED);
    delay(6000);
    changeLED(OFF);
  }
  else{
    magnetic = false;
    changeLED(GREEN);
    delay(6000);
    changeLED(OFF);
  }
  stop(STOP_INTERVAL);
  turn180Deg();
  stop(STOP_INTERVAL);
  goStraight(LR);
  stop(STOP_INTERVAL);
  turnLeft90Deg();
  stop(STOP_INTERVAL);
  return magnetic;
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Functions of two tasks

// Task 1: Go around the grids & collect 2 blocks on them
// numJunction 
// release_method: 
//    0 indicates going the same way back to the starting point and turn 90 deg
//    1 indicates direct locating from Pythagoras theorem
void task1(){
  bool magnetic = false;
  changeLED(OFF);
  while(blockProcessed <= 1){
    // clawOpen();
    int numJunction = 0;
    while(!search1(blockProcessed, numJunction)){
      if(numJunction < 5){numJunction++;}
      else{break;}
    } 
    stop(50);
    if(isMagnetic()){
      magnetic = true;
      changeLED(RED);
      delay(6000);
      changeLED(OFF);
    }
    else{
      magnetic = false;
      changeLED(GREEN);
      delay(6000);
      changeLED(OFF);
    }
    // clawClose(); 
    // clawUp();
    stop(STOP_INTERVAL);
    turn180Deg();
    stop(STOP_INTERVAL);
    while(!return1(blockProcessed, numJunction)){
      Serial.println(numJunction);
      if(numJunction > 0){numJunction--;}
      else{break;}
    }
    if(magnetic){gotoRed(magneticBlockProcessed);}
    else{gotoGreen(nonMagBlockProcessed);}
    blockProcessed++;
    if(blockProcessed == 1){stopAndStay();}
  }
}


void task2(){
  int numJunction = 0;
  int fetchPoint = -1;
  bool magnetic = false;
  goStraight(LR);
  stepForward();
  followLine(LR, false); // reached the cross
  stepForward();
  followLine(LR, false); // reached the T junction
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  while(numJunction < 7){
    if(search2(numJunction, fetchPoint)){
      magnetic = fetchBlock();
      fetchPoint = numJunction;
      numJunction --;
    }
    numJunction ++;
    if(numJunction == 7 && fetchPoint == -1){numJunction = 0;}
  }
  stop(STOP_INTERVAL);
  turnRight90Deg();
  stop(STOP_INTERVAL);
  stepForward();
  followLine(LR, false);
  followLine(LR, false);
  if(magnetic){gotoRed(magneticBlockProcessed);}
  else{gotoGreen(nonMagBlockProcessed);}
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// main functions

void setup() {
  // Initialize serial port and input pins
  Serial.begin(115200);

  pinMode(frontRightSensorPin, INPUT);
  pinMode(frontLeftSensorPin, INPUT);
  pinMode(rearRightSensorPin, INPUT);
  pinMode(rearLeftSensorPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(blueLEDPin, INPUT);  
  pinMode(redLEDPin, INPUT); 
  pinMode(greenLEDPin, INPUT); 
  pinMode(magneticPin, INPUT);
  pinMode(ultrasoundPin, INPUT);
  
  Wire.begin();  //join i2c bus (address optional for master)   
  tof.begin(0x50);  //Set I2C sub-device address   
  tof.setMode(tof.eContinuous,tof.eHigh);  //Set to Back-to-back mode and high precision mode   
  tof.start();  //Laser rangefinder begins to work   

  // clawServo.attach(11);
  // clawServo.write(clawClosePos);
  // pullServo.attach(10);
  // pullServo.write(pullDownPos);
}


void loop(){
  if (!AFMS.begin()){         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }
  Serial.println("Motor Shield found.");

  while(digitalRead(buttonPin) == LOW){delay(10);}  // the button aims to stop the program from proceeding without button press

  // main function starts ----------------------------------------------

  task1();
  while(1){task2();}

  //main function ends ----------------------------------------------
  
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  Serial.println("loop ended");
  while(1){delay(100);}
}
