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
char terrain_map[6][6];

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;

void setup() {
  Serial.begin(9600);
  
  pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);
}

void GoToWallEdge() {
  //Go to a wall edge code
}

void ExploreTerrain() {
  //Explore terrain and update the terrain map
  terrain_map[0][0] = '1';
}

void loop() {
  GoToWallEdge();
  ExploreTerrain();  
}
