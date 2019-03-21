/*
 * Library: https://github.com/bolderflight/MPU9250
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
 * Updated by Ahmad Shamshiri on July 09, 2018 for Robojax.com
 * in Ajax, Ontario, Canada
 * watch instrucion video for this code: 
For this sketch you need to connect:
VCC to 5V and GND to GND of Arduino
SDA to A4 and SCL to A5

S20A is 3.3V voltage regulator MIC5205-3.3BM5
*/

#include "MPU9250.h"
#include <PID_v1.h>

// MOTOR PARAMETERS
#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

// FASTER MOTOR
#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

#define SPEED_DIFF 35

const int min_fwd_speed = 220;
const int min_turn_speed = 210;
int speed;
double calculated_speed;
int set_PID_max = 1;
double PID_max;

// PID Gain Values
double Kp=0.05, Ki=5, Kd=0.1;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

double yaw, desired_yaw;

// PID Control
PID turnPID(&yaw, &calculated_speed, &desired_yaw, Kp, Ki, Kd, DIRECT);

// Gyroscope Data
double omega_z; // RAD/SEC
double theta_z; // RAD

#define GZ_OFFSET   -0.000434851
#define THETA_SCALING  1.003786916

#define GZ_DEAD_PASS_CUTOFF 0.25 // RAD/SEC

// Filter Parameters
// LOW PASS FILTERS NOISE FROM RAW DATA
double curr_LP_filtered_data, prev_LP_filtered_data; // RAD/SEC

#define LP_CUTOFF_FREQ_HZ  5

double RC_LP = 1.0/(2*M_PI*LP_CUTOFF_FREQ_HZ);
double alpha_LP;

// Time Data
int t1, t2;
double dt;

double apply_dead_pass_gz(double gz) {
  if (abs(gz) < GZ_DEAD_PASS_CUTOFF) {
    return 0.0;
  } else {
    return gz;
  }
}

void reset_yaw() {
  omega_z = 0.0;
  theta_z = 0.0;
  yaw = 0.0;

  IMU.readSensor();
  t1 = millis();
  
  curr_LP_filtered_data = apply_dead_pass_gz(IMU.getGyroZ_rads() + GZ_OFFSET);
  prev_LP_filtered_data = curr_LP_filtered_data;
}

void update_yaw() {
  // Read the sensor
  IMU.readSensor();
  t2 = millis();

  // Apply LP Filter
  dt = (t2 - t1)/1000.0; // Convert to seconds
  alpha_LP = dt/(RC_LP+dt);

  curr_LP_filtered_data = prev_LP_filtered_data + (alpha_LP * (apply_dead_pass_gz(IMU.getGyroZ_rads() + GZ_OFFSET) - prev_LP_filtered_data));
  
  // Update integrations
  omega_z = (curr_LP_filtered_data + prev_LP_filtered_data)/2.0;
  theta_z += (omega_z * dt) * THETA_SCALING;
  
  // Setup parameters for next run
  t1 = t2;
  prev_LP_filtered_data = curr_LP_filtered_data;

  // Set yaw value
  yaw = abs(theta_z);
  Serial.print(yaw/M_PI*180);

  // Adjust the output according to current yaw value
  turnPID.Compute();
  Serial.print("  ");
  Serial.print(calculated_speed);
  if (set_PID_max) {
    PID_max = calculated_speed;
    set_PID_max = 0;
  }
  speed = map(calculated_speed, 0, PID_max+10, 155 + SPEED_DIFF, 255);
  Serial.print("  ");
  Serial.println(speed);
}

void turn_ccw() {
  Serial.println("Turning CCW...");

  // Set initial parameters
  reset_yaw();
  speed = min_turn_speed;
  set_PID_max = 1;
  
  // Initialize PID Parameters
//  desired_yaw = M_PI/2 - 8/180*M_PI;
//  desired_yaw = 1.43117; // 82 degrees
  desired_yaw = 1.41372; // 81 degrees
  turnPID.SetMode(AUTOMATIC); // Turn the PID on
  turnPID.SetTunings(Kp, Ki, Kd);

  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed - SPEED_DIFF);

  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, HIGH);
  
  while (abs(yaw) < desired_yaw) {
    update_yaw();
    analogWrite(ENABLE_M1, speed);
    analogWrite(ENABLE_M2, speed - SPEED_DIFF);
  }

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  delay(500);
}

void move_forwards() {
  Serial.println("Moving forwards...");

  speed = min_fwd_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed  - SPEED_DIFF);
  
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);

  delay(6000);

  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  delay(500);
}

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  // pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  // pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_fwd_speed;

// WITHOUT MOTORS
//  omega_z = 0.0;
//  theta_z = 0.0;
//  
//  IMU.readSensor();
//  t1 = millis();
//  
//  curr_LP_filtered_data = apply_dead_pass_gz(IMU.getGyroZ_rads() + GZ_OFFSET);
//  prev_LP_filtered_data = curr_LP_filtered_data;
}

void loop() {
  // WITH MOTORS
  move_forwards();
  turn_ccw();
  
  
  // WITHOUT MOTORS
//  // read the sensor
//  IMU.readSensor();
//
//  // Apply LP Filter
//  t2 = millis();
//  dt = (t2 - t1)/1000.0; // Convert to seconds
//  alpha_LP = dt/(RC_LP+dt);
//
//  curr_LP_filtered_data = prev_LP_filtered_data + (alpha_LP * (apply_dead_pass_gz(IMU.getGyroZ_rads() + GZ_OFFSET) - prev_LP_filtered_data));
//  
//  // Update integrations
//  omega_z = (curr_LP_filtered_data + prev_LP_filtered_data)/2.0;
//  theta_z += (omega_z * dt) * GZ_SCALING;
//  
//  // Setup parameters for next integration
//  t1 = t2;
//
//  // Set yaw value
//  yaw = abs(theta_z);
//  
//  // display the data (1)
//  Serial.print("GyroZ: ");  
//  Serial.print(curr_LP_filtered_data,6);
//  Serial.print("  "); 
//  Serial.print("Time in [sec]: ");
//  Serial.print(t2/1000.0);
//  Serial.print("  ");
//  Serial.print("Gyro Yaw in [deg]: ");
//  Serial.println(yaw/M_PI*180);

//  prev_LP_filtered_data = curr_LP_filtered_data;
}
