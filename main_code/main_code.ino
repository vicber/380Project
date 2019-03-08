#include "SR04.h"
#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

#define TRIG_PIN 7
#define ECHO_PIN 6

#define ROWS 6
#define COLS 6

//Colour Sensor Stuff
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

//Flame & Thermal Sensor
#define FLAME 8

//Hall Effect Sensor
const int hallPin = 12;     // the number of the hall effect sensor pin
const int ledPin =  13;     // the number of the LED pin
int hallState = 0;          // variable for reading the hall sensor status

// Vin 3.3V 
int red = 0;
int blue = 0;
int green = 0;
int totalRGB = 0;

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;
char terrain_map[6][6];

void setup() {
  Serial.begin(9600);

  //Flame and Thermal
  pinMode(FLAME, INPUT);
  
  //Hall effect
  pinMode(hallPin, INPUT);
  
  pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  //Colour Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);
}

bool ReachWall(){
  return true;
}

bool DetectMagnet() {
  hallState = digitalRead(hallPin);
  if (hallState == LOW) {     
    return true;
  } 
  else {
    return false;
  }
}

void ReadColour() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = pulseIn(sensorOut, LOW);
  red = map(red, 60, 146, 255, 0);
  delay(100);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  green = pulseIn(sensorOut, LOW);
  green = map(green, 121, 277, 255, 0);
  delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blue = pulseIn(sensorOut, LOW);
  blue = map(blue, 59, 378, 255, 0);

  totalRGB = red + blue + green;
}

void HandleObject() {
  ReadColour();

  if(totalRGB > 150 && double(red+green)/totalRGB >= 0.70 && double(green) / totalRGB > 0.38) {
    Serial.println("Detect Yellow House");
  }
  else if(totalRGB > 150 && double(red+blue)/totalRGB >= 0.75 && double(blue) / totalRGB > 0.40){
    Serial.println("Detect Red House");
  }
  else if(digitalRead(FLAME)==HIGH) {
    Serial.println("Detect Lit Candle");
  }
  else if(DetectMagnet()) {
    Serial.println("Detect Food");
  }
  else {
    Serial.println("Nothing read");
  }
}

void GoToWallEdge() {
  while(1) {
    //move forward
    digitalWrite(DIR_A_M1, HIGH);
    digitalWrite(DIR_A_M2, HIGH);
    digitalWrite(DIR_B_M1, LOW);
    digitalWrite(DIR_B_M2, LOW);
  
    a=sr04.Distance();
    while(a > 5) {
      a=sr04.Distance();
      delay(100);
    }
    
    if(ReachWall()) {
      //turn CCW
      digitalWrite(DIR_A_M1, LOW);
      delay(2000);
      return;
    }
    else {
      HandleObject();
      if(!DetectMagnet()) {
        //back up to last tile
        digitalWrite(DIR_A_M1, LOW);
        digitalWrite(DIR_A_M2, LOW);
        digitalWrite(DIR_B_M1, HIGH);
        digitalWrite(DIR_B_M2, HIGH);

        delay(1000);
        
        //CCW
        digitalWrite(DIR_A_M1, LOW);
        digitalWrite(DIR_A_M2, HIGH);
        digitalWrite(DIR_B_M1, LOW);
        digitalWrite(DIR_B_M2, LOW);
        
        delay(2000);
      }
    }
  }
}

void ExploreTerrain() {
  //Explore terrain and update the terrain map
  terrain_map[0][0] = '1';
}

void loop() {
  digitalWrite(ENABLE_M1, HIGH);
  digitalWrite(ENABLE_M2, HIGH);
  GoToWallEdge();
  ExploreTerrain();  
}
