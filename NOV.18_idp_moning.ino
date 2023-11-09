#include<Arduino_LSM6DS3.h>
#include <Adafruit_MotorShield.h>

int leftlinesensorPin = 3;
int rightlinesensorPin = 2;

int valLeft, valRight;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);

void goStraight(){
  valLeft = digitalRead(leftlinesensorPin);
  valRight = digitalRead(rightlinesensorPin);

  
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(150);
  // turn on motor
  leftMotor->run(RELEASE);

  
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  // turn on motor
  rightMotor->run(RELEASE);

  while(1){
    if(valLeft == 0 && valRight == 0){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(150);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(150);
      delay(50);
    }
    else if(valLeft==1){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(150);
      rightMotor->run(BACKWARD);
      rightMotor->setSpeed(150);
      delay(50);
    }
    else{
      leftMotor->run(BACKWARD);
      leftMotor->setSpeed(150);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(150);
      delay(50);
    }
    valLeft = digitalRead(leftlinesensorPin);
    valRight = digitalRead(rightlinesensorPin);
  }
}

float x,y,z;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(leftlinesensorPin, INPUT);
  pinMode(rightlinesensorPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while(1);
  }
  Serial.println("Motor Shield found.");
  goStraight();
}
