/*
Arduino Hall Effect Sensor Project
by Arvind Sanjeev
Please check out  http://diyhacking.com for the tutorial of this project.
DIY Hacking
*/
/*Need to solder the hall effect sensor and the wires on the board*/

// constants won't change. They're used here to set pin numbers:
const int hallPin1 = 53;     // the number of the hall effect sensor pin
const int hallPin2 = 51;
const int hallPin3 = 49;
const int hallPin4 = 47;
const int hallPin5 = 45;

const int ledPin =  13;     // the number of the LED pin
// variables will change:
int hallState1 = 0;          // variable for reading the hall sensor status
int hallState2 = 0;
int hallState3 = 0;
int hallState4 = 0;
int hallState5 = 0;

int oldState1 = 0;
int oldState2 = 0;
int oldState3 = 0;
int oldState4 = 0;
int oldState5 = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the hall effect sensor pin as an input:
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2,INPUT);  
  pinMode(hallPin3,INPUT);
  pinMode(hallPin4, INPUT);
  pinMode(hallPin5,INPUT); 
  Serial.begin(9600);   

  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
  hallState3 = digitalRead(hallPin3);
  hallState4 = digitalRead(hallPin4);
  hallState5 = digitalRead(hallPin5);
  oldState1 = hallState1;
  oldState2 = hallState2;
  oldState3 = hallState3;
  oldState2 = hallState4;
  oldState3 = hallState5;

  Serial.println("Move");
}

void loop(){
  delay(5000);
  
  // read the state of the hall effect sensor:
  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
  hallState3 = digitalRead(hallPin3);
  hallState4 = digitalRead(hallPin4);
  hallState5 = digitalRead(hallPin5);
  
  if (hallState1 != oldState1){
    Serial.println("1 Detects");
  }
  if (hallState2 != oldState2){
    Serial.println("2 Detects");
  }
  if (hallState3 != oldState3){
    Serial.println("3 Detects");
  }
  if (hallState4 != oldState4){
    Serial.println("4 Detects");
  }
  if (hallState5 != oldState5){
    Serial.println("5 Detects");
  } 
  
  oldState1 = hallState1;
  oldState2 = hallState2;
  oldState3 = hallState3;
  oldState4 = hallState4;
  oldState5 = hallState5;
  while(1) {}
}
