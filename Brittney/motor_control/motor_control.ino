#include "SR04.h"

#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

#define TRIG_PIN 7
#define ECHO_PIN 6
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
//long a;

const int min_fwd_speed = 170; //220
const int min_turn_speed = 210;
int speed;

//int US_count = 0;

//const int min_fwd_dist = 22;
long distance_travelled; // CM
// int direction = 1; // 1: FORWARDS, -1: BACKWARDS
long last_dist, curr_dist; // CM

#define MAX_DIFF_DIST 50

void setup() {
  Serial.begin(9600);
  
  // pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  // pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_fwd_speed;
  distance_travelled = 0;

  // SUBJECT TO CHANGE...
  // Min speed for moving forwards: 100
  // Min speed going backwards: 90
  // Min speed for turning: 150
  // Approximately, speed for turning should be speed for moving forwards + 50

  // TESTING RESULTS:
  // Min speed fwd on gravel: 110
  // Min speed fwd sand: 140 - 150
  // Min speed fwd low to normal level: 180
  // Min speed fwd sand to low to low to normal level: 180
  // Min speed fwd normal to low to normal level: 180
  // Min speed fwd low to normal, 2 in gap: 200
}

void loop() {
  analogWrite(ENABLE_M1, speed); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?

  // FORWARDS
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);

  last_dist = sr04.Distance();
  while(distance_travelled < 182) {
    curr_dist=sr04.Distance();
    if (abs(curr_dist - last_dist) < MAX_DIFF_DIST) {
      distance_travelled = distance_travelled + (last_dist - curr_dist);
      last_dist = curr_dist;
    }
    Serial.println(distance_travelled);
    //Serial.print(a);
    //Serial.println("cm");
    delay(50);
  }

  delay(1000);

  // BACKWARDS
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  last_dist = sr04.Distance();
  while(distance_travelled > 0) {
    curr_dist=sr04.Distance();
    if (abs(curr_dist - last_dist) < MAX_DIFF_DIST) {
      distance_travelled = distance_travelled + (last_dist - curr_dist);
      last_dist = curr_dist;
    }
    Serial.println(distance_travelled);
    //Serial.print(a);
    //Serial.println("cm");
    delay(50);
  }

  delay(1000);
}
