/*
 IDP
 David Paterson
 Push Button Module Example V1
 When you push the digital button the Led 2 will turn off otherwise the LED turns on.
*/
int ledPin = 7; // choose the pin for the LED
int inputPin = 8; // Connect button to input pin 8

int ledstate = LOW; // set initial state of vehivle to be low, which means vehicle shoud stay still
int current;
int last;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT); // declare LED as output
  pinMode(inputPin, INPUT); // declare pushbutton as input
  current = digitalRead(inputPin); // read input value
}
unsigned long previousMillis = 0;  // will store last time LED was updated
int ledBlinkState = LOW;
void blink(){
  

  // constants won't change:
  const long interval = 250;  // interval at which to blink (milliseconds)
  unsigned long currentMillis = millis();

  
  //Serial.println('cur');
  //Serial.println(currentMillis);
  //Serial.println('pre');
  //Serial.println(previousMillis);

  if ((currentMillis - previousMillis) >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    //Serial.println('blinklllllllllllllllllll');
    // if the LED is off turn it on and vice-versa:
    if (ledBlinkState == LOW) {
      ledBlinkState = HIGH;
    } 
    else {
      ledBlinkState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledBlinkState);
  }
}

void loop(){
  last = current; //assign last loop button reading(int: current) to int: last
  current = digitalRead(inputPin);
  if(last==HIGH && current == LOW){  //pressed the button to toggle the state
    ledstate = HIGH;
  }
  
  while(ledstate){
    last = current;
    current = digitalRead(inputPin);
    if(last==HIGH && current == LOW){
      ledstate = LOW;
    }
    blink();
    
    Serial.println("inthe while loop");
    Serial.println(ledstate);
    
  }
  Serial.println("outside the while loop");
  Serial.println(ledstate);
  digitalWrite(ledPin, LOW);
}


