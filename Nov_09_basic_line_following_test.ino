#include <Adafruit_MotorShield.h>

int leftlinesensorPin = 2;
int rightlinesensorPin = 4;

int valLeft, valRight;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);


int MAX_SPEED = 255;

void goStraight(){
  valLeft = digitalRead(leftlinesensorPin);
  valRight = digitalRead(rightlinesensorPin);

  
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(MAX_SPEED);
  // turn on motor
  leftMotor->run(RELEASE);

  
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(MAX_SPEED);
  // turn on motor
  rightMotor->run(RELEASE); 

  while(1){
    if(!(valLeft) && !(valRight)){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(MAX_SPEED);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(MAX_SPEED);
    }
    else if(valLeft && !(valRight)){
      leftMotor->run(BACKWARD);
      leftMotor->setSpeed(MAX_SPEED);
      rightMotor->run(FORWARD);
      rightMotor->setSpeed(MAX_SPEED);
      Serial.println("going left")
    }
    else if(!(valLeft) && valRight){
      leftMotor->run(FORWARD);
      leftMotor->setSpeed(MAX_SPEED);
      rightMotor->run(BACKWARD);
      rightMotor->setSpeed(MAX_SPEED);
      Serial.println("going right")
    }
    delay(50)
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
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
