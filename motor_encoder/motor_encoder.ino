#define MOTOR_ENC_PIN_A   22
#define MOTOR_ENC_PIN_B   23

int last_enc_val_A, last_enc_val_B, enc_val_A, enc_val_B;
int curr_count;

// CW:  0, 1, 3, 2 
//      00, 01, 11, 10, ...
// CCW: 0, 2, 3, 1
//      00, 10, 11, 01, ...

// 45 counts is approximately equal to 360 degrees

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_ENC_PIN_A, INPUT);
  pinMode(MOTOR_ENC_PIN_B, INPUT);
  last_enc_val_A = digitalRead(MOTOR_ENC_PIN_A);
  last_enc_val_B = digitalRead(MOTOR_ENC_PIN_B);
  curr_count = 0;
}

void loop() {
  int diff_enc_val_A, diff_enc_val_B;
  
  enc_val_A = digitalRead(MOTOR_ENC_PIN_A);
  enc_val_B = digitalRead(MOTOR_ENC_PIN_B);
  
  diff_enc_val_A = enc_val_A - last_enc_val_A;
  diff_enc_val_B = enc_val_B - last_enc_val_B;

  if (abs(diff_enc_val_A) == 1 || abs(diff_enc_val_B) == 1) {
    // If one of the pin values changed, need to evaluate change...
    if (diff_enc_val_A == 1) {
      // Pin A value switched from low to high
      if (enc_val_B == 1) {
        // Going CW, increment the count
        curr_count++;
      } else {
        // Going CCW, decrement the count
        curr_count--;
      }
    } else if (diff_enc_val_A == -1) {
      // Pin A value switched from high to low
      if (enc_val_B == 0) {
        // Going CW, increment the count
        curr_count++;
      } else {
        // Going CCW, decrement the count
        curr_count--;
      }
    } else if (diff_enc_val_B == 1) {
      // Pin B value switched from low to high
      if (enc_val_A == 0) {
        // Going CW, increment the count
        curr_count++;
      } else {
        // Going CCW, decrement the count
        curr_count--;
      }
    } else {
      // Pin B value switched from high to low
      if (enc_val_A == 1) {
        // Going CW, increment the count
        curr_count++;
      } else {
        // Going CCW, decrement the count
        curr_count--;
      }
    }
  }
  
  Serial.print(enc_val_A);
  Serial.print(enc_val_B);
  Serial.print("\t");
  Serial.print(curr_count);
  Serial.print("\t");
  Serial.println(curr_count*360/45);

  last_enc_val_A = enc_val_A;
  last_enc_val_B = enc_val_B;
}
