#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

const int min_fwd_speed = 140;

int speed;

void setup() {
  Serial.begin(9600);
  
  // pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  // pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_fwd_speed; // TESTING

  // SUBJECT TO CHANGE...
  // Min speed for moving forwards: 100
  // Min speed going backwards: 90
  // Min speed for turning: 150
  // Approximately, speed for turning should be speed for moving forwards + 50
}

void loop() {
  // digitalWrite(ENABLE_M1, HIGH);
  // digitalWrite(ENABLE_M2, HIGH);

  analogWrite(ENABLE_M1, speed); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  delay(5000);

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);

  delay(5000);

//  analogWrite(ENABLE_M1, speed+50); // From 0 - 255?
//  analogWrite(ENABLE_M2, speed+50); // From 0 - 255?
//  
//  digitalWrite(DIR_A_M1, LOW);
//  
//  delay(5000);
//
//  digitalWrite(DIR_A_M1, HIGH);
//  digitalWrite(DIR_A_M2, LOW);
//
//  delay(5000);
//
//  analogWrite(ENABLE_M1, speed); // From 0 - 255?
//  analogWrite(ENABLE_M2, speed); // From 0 - 255?
//
//  digitalWrite(DIR_A_M1, LOW);
//  digitalWrite(DIR_A_M2, LOW);
//  digitalWrite(DIR_B_M1, HIGH);
//  digitalWrite(DIR_B_M2, HIGH);
//
//  delay(1500);

  speed += 10;
  if (speed > 255) {
    speed = min_fwd_speed;
  }
}
