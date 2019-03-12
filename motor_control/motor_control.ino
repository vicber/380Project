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
long a;

const int min_fwd_speed = 220;
const int min_turn_speed = 210;
int speed;

const int min_fwd_dist = 15;

void setup() {
  Serial.begin(9600);
  
  // pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  // pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_fwd_speed;

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
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  a=sr04.Distance();
  while(a > min_fwd_dist) {
    a=sr04.Distance();
    //Serial.print(a);
    //Serial.println("cm");
    delay(100);
  }

  speed = min_turn_speed;

  analogWrite(ENABLE_M1, speed); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?

  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, HIGH);

  delay(1000);

  speed = min_fwd_speed;

  analogWrite(ENABLE_M1, speed); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  delay(1500);
}
