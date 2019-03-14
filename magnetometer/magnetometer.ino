#include <Wire.h>

#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

#define MAG_ADDR 0x0E

int mag_x, mag_y, mag_z;
double adjusted_mag_x, adjusted_mag_y;
double yaw_read;
double prev_yaw, curr_yaw; // To determine if we have a yaw jump

const int min_fwd_speed = 220;
const int min_turn_speed = 200;
int speed;

//#define MAG_X_OFFSET  1929.64
//#define MAG_Y_OFFSET  -1821.27
#define MAG_X_OFFSET  1112.73
#define MAG_Y_OFFSET  -1782.96

void config_magnetometer(void)
{
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x11);             // cntrl register2
  Wire.write(0x80);             // write 0x80, enable auto resets
  Wire.endTransmission();       // stop transmitting
  delay(15); 
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x10);             // cntrl register1
  Wire.write(1);                // write 0x01, active mode
  Wire.endTransmission();       // stop transmitting
}

void print_values(void)
{
  calculate_yaw();
  Serial.print("x=");
  Serial.print(adjusted_mag_x);
  Serial.print(",");  
  Serial.print("y=");    
  Serial.print(adjusted_mag_y);
//  Serial.print(",");       
//  Serial.print("z=");    
//  Serial.print(mag_z);
  Serial.print(", angle=");
  Serial.println(yaw_read);
}

void calculate_yaw(){
  mag_x = read_x();
  mag_y = read_y();
  mag_z = read_z();
  adjusted_mag_x = (double)mag_x + MAG_X_OFFSET;
  adjusted_mag_y = (double)mag_y + MAG_Y_OFFSET;
  yaw_read = 180 + 180*atan2(adjusted_mag_y, adjusted_mag_x)/M_PI;
}
 
int mag_read_register(int reg)
{
  int reg_val;
 
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(reg);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
 
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may write less than requested
  { 
    reg_val = Wire.read(); // read the byte
  }
 
  return reg_val;
}
 
int mag_read_value(int msb_reg, int lsb_reg)
{
  int val_low, val_high;  //define the MSB and LSB
  val_high = mag_read_register(msb_reg);
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  val_low = mag_read_register(lsb_reg);
  int out = (val_low|(val_high << 8)); //concatenate the MSB and LSB
  return out;
}
 
int read_x(void)
{
  return mag_read_value(0x01, 0x02);
}
 
int read_y(void)
{
  return mag_read_value(0x03, 0x04);
}
 
int read_z(void)
{
  return mag_read_value(0x05, 0x06);
}

void setup() {
  Wire.begin();                     // join i2c bus (address optional for master)
  Serial.begin(9600);               // start serial for output
  config_magnetometer();            // turn the MAG3110 on

  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  speed = min_turn_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed);
}

int detectYawJump() {
  
  if (prev_yaw >= 0 && prev_yaw < 90 && curr_yaw > 270 && curr_yaw <= 360) {
    // Went from 0 to 360
    return 1;
    
  } else if (prev_yaw > 270 && prev_yaw <= 360 && curr_yaw >= 0 && curr_yaw < 90) {
    // Went from 360 to 0
    return 2;
  }
  return 0;
}

void loop() {
//  print_values();
//  delay(500);
  
  double init_turn_angle = 0.0;
  double current_turn_angle = 0.0;
  double yaw_offset = 0.0;

  // Turn CCW
  
  calculate_yaw();
  init_turn_angle = yaw_read;
  current_turn_angle = yaw_read;
  prev_yaw = yaw_read;
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, HIGH);
  
  Serial.println(current_turn_angle);
  
  while(abs(current_turn_angle - init_turn_angle) < 90) {
    calculate_yaw();
    curr_yaw = yaw_read + yaw_offset;
    int yaw_jumped = detectYawJump();
    if (yaw_jumped) {
      if (yaw_jumped == 1) {
        // Went from 0 to 360
        yaw_offset = -360;
      } else if (yaw_jumped == 2) {
        // Went from 360 to 0
        yaw_offset = 360;
      }
    }
    current_turn_angle = yaw_read + yaw_offset;
    prev_yaw = curr_yaw;
    Serial.println(current_turn_angle);
    delay(50);
  };

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
  
  Serial.println("TURNED 90 DEGREES CCW");
  delay(2000);

  yaw_offset = 0.0;

   // Turn CW
  
  calculate_yaw();
  init_turn_angle = yaw_read;
  current_turn_angle = yaw_read;
  prev_yaw = yaw_read;
  
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, LOW);
  
  Serial.println(current_turn_angle);
  
  while(abs(current_turn_angle - init_turn_angle) < 90) {
    calculate_yaw();
    curr_yaw = yaw_read + yaw_offset;
    int yaw_jumped = detectYawJump();
    if (yaw_jumped) {
      if (yaw_jumped == 1) {
        // Went from 0 to 360
        yaw_offset = -360;
      } else if (yaw_jumped == 2) {
        // Went from 360 to 0
        yaw_offset = 360;
      }
    }
    current_turn_angle = yaw_read + yaw_offset;
    prev_yaw = curr_yaw;
    Serial.println(current_turn_angle);
    delay(50);
  };

  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  
  Serial.println("TURNED 90 DEGREES CW");
  delay(2000);

  yaw_offset = 0.0;
}
