#define MOTOR_ENC_PIN_A   22 // DIGITAL
#define MOTOR_ENC_PIN_B   24 // DIGITAL

// POWER CONNECTED TO 5V
// GND JUST CONNECTED TO GROUND

#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12

#define DIR_B_M2 13

int last_enc_val_A, last_enc_val_B, enc_val_A, enc_val_B;
int curr_count;

// CCW:  0, 1, 3, 2 
//      00, 01, 11, 10, ...
// CW: 0, 2, 3, 1
//      00, 10, 11, 01, ...

// 45 counts is approximately equal to 360 degrees

#define ONE_TURN_COUNT_VAL  45
#define MAX_MIN_COUNT       20

const int min_fwd_speed = 100; //220
const int min_turn_speed = 210;
int speed;
int curr_dir = 1;
// 1: Forward, 0: Backward

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_ENC_PIN_A, INPUT);
  pinMode(MOTOR_ENC_PIN_B, INPUT);
  last_enc_val_A = digitalRead(MOTOR_ENC_PIN_A);
  last_enc_val_B = digitalRead(MOTOR_ENC_PIN_B);
  curr_count = 0;

  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_fwd_speed;
}

void loop() {
  analogWrite(ENABLE_M1, 70); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?
  
  if (curr_dir == 1) {
    if (curr_count < MAX_MIN_COUNT) {
      // Keep going forwards
      // Serial.println("KEEP GOING FORWARDS");
      digitalWrite(DIR_A_M1, LOW);
      digitalWrite(DIR_A_M2, LOW);
      digitalWrite(DIR_B_M1, HIGH);
      digitalWrite(DIR_B_M2, HIGH);
    } else {
      // Go backwards now
      // Serial.println("GO BACKWARDS NOW");
      digitalWrite(DIR_A_M1, HIGH);
      digitalWrite(DIR_A_M2, HIGH);
      digitalWrite(DIR_B_M1, LOW);
      digitalWrite(DIR_B_M2, LOW);
      curr_dir = 0;
    }
  } else {
    if (curr_count > -MAX_MIN_COUNT) {
      // Keep going backwards
      // Serial.println("KEEP GOING BACKWARDS");
      digitalWrite(DIR_A_M1, HIGH);
      digitalWrite(DIR_A_M2, HIGH);
      digitalWrite(DIR_B_M1, LOW);
      digitalWrite(DIR_B_M2, LOW);
    } else {
      // Go forwards now
      // Serial.println("GO FORWARDS NOW");
      digitalWrite(DIR_A_M1, LOW);
      digitalWrite(DIR_A_M2, LOW);
      digitalWrite(DIR_B_M1, HIGH);
      digitalWrite(DIR_B_M2, HIGH);
      curr_dir = 1;
    }
  }
  
  int diff_enc_val_A, diff_enc_val_B;
  
  enc_val_A = digitalRead(MOTOR_ENC_PIN_A);
  enc_val_B = digitalRead(MOTOR_ENC_PIN_B);
  
  diff_enc_val_A = enc_val_A - last_enc_val_A;
  diff_enc_val_B = enc_val_B - last_enc_val_B;

  // CCW:  0, 1, 3, 2 
  //      00, 01, 11, 10, ...
  // CW: 0, 2, 3, 1
  //      00, 10, 11, 01, ...

  if (abs(diff_enc_val_A) == 1 || abs(diff_enc_val_B) == 1) {
    // If one of the pin values changed, need to evaluate change...
    if (diff_enc_val_A == 1) {
      // Pin A value switched from low to high
      if (enc_val_B == 1) {
        // Going CCW, increment the count
        Serial.print("CCW");
        curr_count++;
      } else {
        // Going CW, decrement the count
        Serial.print("CW");
        curr_count--;
      }
    } else if (diff_enc_val_A == -1) {
      // Pin A value switched from high to low
      if (enc_val_B == 0) {
        // Going CCW, increment the count
        Serial.print("CCW");
        curr_count++;
      } else {
        // Going CW, decrement the count
        Serial.print("CW");
        curr_count--;
      }
    } else if (diff_enc_val_B == 1) {
      // Pin B value switched from low to high
      if (enc_val_A == 0) {
        // Going CCW, increment the count
        Serial.print("CCW");
        curr_count++;
      } else {
        // Going CW, decrement the count
        Serial.print("CW");
        curr_count--;
      }
    } else {
      // Pin B value switched from high to low
      if (enc_val_A == 1) {
        // Going CCW, increment the count
        Serial.print("CCW");
        curr_count++;
      } else {
        // Going CW, decrement the count
        Serial.print("CW");
        curr_count--;
      }
    }
  }
  
  Serial.print("\t");
  Serial.print(enc_val_A);
  Serial.print(enc_val_B);
  Serial.print("\t");
  Serial.print(curr_count);
  Serial.print("\t");
  Serial.println(curr_count*360/ONE_TURN_COUNT_VAL);

  last_enc_val_A = enc_val_A;
  last_enc_val_B = enc_val_B;
}
