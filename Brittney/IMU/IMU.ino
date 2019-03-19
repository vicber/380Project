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

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

double roll, pitch, yaw;

// Accelerometer Data
double a_x, a_y; // M/(SEC^2)
double v_x, v_y; // M/SEC
double d_x, d_y; // M

// Gyroscope Data
double omega_z; // RAD/SEC
double theta_z; // RAD

//#define GZ_OFFSET   -0.000434851
//#define GZ_SCALING  1.003786916
#define GZ_OFFSET   0.0
#define GZ_SCALING  1.0

// RC Filter Parameters
double curr_filtered_data[6], prev_filtered_data[6];

#define CUTOFF_FREQ_HZ  0.5

double RC = 1.0/(2*M_PI*CUTOFF_FREQ_HZ);
double alpha;

// Time Data
int t1, t2;
double dt;

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
  
  a_y = 0.0;
  a_x = 0.0;
  v_y = 0.0;
  v_x = 0.0;
  d_y = 0.0;
  d_x = 0.0;
  
  omega_z = 0.0;
  theta_z = 0.0;

  IMU.readSensor();
  t1 = millis();

  // Initialize filtered set of data
  prev_filtered_data[0] = IMU.getAccelX_mss();
  prev_filtered_data[1] = IMU.getAccelY_mss();
  prev_filtered_data[2] = IMU.getAccelZ_mss();
  prev_filtered_data[3] = IMU.getGyroX_rads();
  prev_filtered_data[4] = IMU.getGyroY_rads();
  prev_filtered_data[5] = IMU.getGyroZ_rads();
}

void loop() {
  // read the sensor
  IMU.readSensor();

  // Filter data
  t2 = millis();
  dt = (t2 - t1)/1000.0; // Convert to seconds
  alpha = dt/(RC+dt);
  
  curr_filtered_data[0] = prev_filtered_data[0] + (alpha * (IMU.getAccelX_mss() - prev_filtered_data[0]));
  curr_filtered_data[1] = prev_filtered_data[1] + (alpha * (IMU.getAccelY_mss() - prev_filtered_data[1]));
  curr_filtered_data[2] = prev_filtered_data[2] + (alpha * (IMU.getAccelZ_mss() - prev_filtered_data[2]));
  curr_filtered_data[3] = prev_filtered_data[3] + (alpha * (IMU.getGyroX_rads() - prev_filtered_data[3]));
  curr_filtered_data[4] = prev_filtered_data[4] + (alpha * (IMU.getGyroY_rads() - prev_filtered_data[4]));
  curr_filtered_data[5] = prev_filtered_data[5] + (alpha * (IMU.getGyroZ_rads() - prev_filtered_data[5]));
  
  // Apply Calibration Offset to Gz Value
  curr_filtered_data[5] += GZ_OFFSET;
  
  // Update integrations
  a_x = curr_filtered_data[0];
  a_y = curr_filtered_data[1];
  v_x += a_x * dt;
  v_y += a_y * dt;
  d_x += v_x * dt;
  d_y += v_y * dt;
  
  omega_z = curr_filtered_data[5];
  theta_z += omega_z * dt;
  
  // Setup parameters for next integration
  t1 = t2;

  // Using Gyroscope and Calibration Scaling
  yaw = theta_z*GZ_SCALING;
  
  // display the data LONG
  Serial.print("AccelX: ");
  Serial.print(curr_filtered_data[0],6);
  Serial.print("  ");
  Serial.print("AccelY: ");  
  Serial.print(curr_filtered_data[1],6);
  Serial.print("  ");
  Serial.print("AccelZ: ");  
  Serial.println(curr_filtered_data[2],6);
  
  Serial.print("GyroX: ");
  Serial.print(curr_filtered_data[3],6);
  Serial.print("  ");
  Serial.print("GyroY: ");  
  Serial.print(curr_filtered_data[4],6);
  Serial.print("  ");
  Serial.print("GyroZ: ");  
  Serial.println(curr_filtered_data[5],6);

  Serial.print("MagX: ");  
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("  ");  
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("  ");
  Serial.print("MagZ: ");  
  Serial.println(IMU.getMagZ_uT(),6);
  
  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(),6);
  Serial.println();
  
  Serial.print("Time in [sec]: ");
  Serial.print(t2/1000.0);
  Serial.print("  ");
  Serial.print("Gyro Yaw in [deg]: ");
  Serial.print(yaw/M_PI*180);
  Serial.print("  ");
  Serial.print("Accel Position in [cm]: (");
  Serial.print(d_x*100);
  Serial.print(", ");
  Serial.print(d_y*100);
  Serial.println(")");
  Serial.println();

//  // display the data SHORT
//  Serial.print(curr_filtered_data[0],6);
//  Serial.print("\t");
//  Serial.print(curr_filtered_data[1],6);
//  Serial.print("\t");
//  Serial.print(curr_filtered_data[2],6);
//  Serial.print("\t");
//  Serial.print(curr_filtered_data[3],6);
//  Serial.print("\t");
//  Serial.print(curr_filtered_data[4],6);
//  Serial.print("\t");
//  Serial.print(curr_filtered_data[5],6);
//  Serial.print("\t");
//  Serial.print(IMU.getMagX_uT(),6);
//  Serial.print("\t");
//  Serial.print(IMU.getMagY_uT(),6);
//  Serial.print("\t");
//  Serial.print(IMU.getMagZ_uT(),6);
//  Serial.print("\t");
//  Serial.print(IMU.getTemperature_C(),6);
//  Serial.print("\t");
//  Serial.println(t2/1000.0);

  // Set up previous filtered data for next run
  prev_filtered_data[0] = curr_filtered_data[0];
  prev_filtered_data[1] = curr_filtered_data[1];
  prev_filtered_data[2] = curr_filtered_data[2];
  prev_filtered_data[3] = curr_filtered_data[3];
  prev_filtered_data[4] = curr_filtered_data[4];
  prev_filtered_data[5] = curr_filtered_data[5];
  
  delay(20);
}
