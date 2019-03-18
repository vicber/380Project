/*
Arduino Hall Effect Sensor Project
by Arvind Sanjeev
Please check out  http://diyhacking.com for the tutorial of this project.
DIY Hacking
*/
/*Need to solder the hall effect sensor and the wires on the board*/

// constants won't change. They're used here to set pin numbers:
const int hallPin1 = 9;     // the number of the hall effect sensor pin
const int hallPin2 = 10;
const int ledPin =  13;     // the number of the LED pin
// variables will change:
int hallState1 = 0;          // variable for reading the hall sensor status
int hallState2 = 0;

int oldState1 = 0;
int oldState2 = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the hall effect sensor pin as an input:
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2,INPUT);  
  Serial.begin(9600);   

  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
   oldState1 = hallState1;
      oldState2 = hallState2;;
}

void loop(){
  // read the state of the hall effect sensor:
  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
  
  if (hallState1 != oldState1 || hallState2 != oldState2){
    Serial.println("Detects");
  }
  
      oldState1 = hallState1;
      oldState2 = hallState2;
  
 
}
