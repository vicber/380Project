#include "SR04.h"
#include "MPU9250.h"
#include <PID_v1.h>

bool wall_reached = false;

/*
 * IMU STUFF
 */
#define SPEED_DIFF 35
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
/*
 * IMU STUFF
 */

//Test LED
int redPin= 8;
int greenPin = 7;
int bluePin = 6;

//Motors
#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 13
#define DIR_B_M2 12
const int min_fwd_speed = 220;
const int min_turn_speed = 210;
int speed;

#define TRIG_PIN 15
#define ECHO_PIN 14
#define TRIG_PIN_BACK 24
#define ECHO_PIN_BACK 26

//Colour Sensor Stuff
#define S0 30
#define S1 32
#define S2 34
#define S3 36
#define sensorOut 28
const int minRGB = 150; //used to ensure that there is enough RGB value to sensibly use percentage method

//Flame
#define FLAME A7

#define ROWS 6
#define COLS 6

//Hall Effect Sensor
const int hallPin1 = 53;     // the number of the hall effect sensor pin
const int hallPin2 = 51;     // the number of the hall effect sensor pin
const int hallPin3 = 49;     // the number of the hall effect sensor pin
const int hallPin4 = 47;     // the number of the hall effect sensor pin
const int hallPin5 = 45;     // the number of the hall effect sensor pin
int hallState1 = 0;          // variable for reading the hall sensor status
int hallState2 = 0;          // variable for reading the hall sensor status
int hallState3 = 0;          // variable for reading the hall sensor status
int hallState4 = 0;          // variable for reading the hall sensor status
int hallState5 = 0;          // variable for reading the hall sensor status
int oldState1 = 0;
int oldState2 = 0;
int oldState3 = 0;
int oldState4 = 0;
int oldState5 = 0;

// Vin 3.3V 
int red = 0;
int blue = 0;
int green = 0;
int totalRGB = 0;

//Ultrasonic Sensor
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long ultrasonic_dist;
SR04 sr04_back = SR04(ECHO_PIN_BACK,TRIG_PIN_BACK);
long ultrasonic_dist_back;
const long tile_dist = 30;

char terrain_map[6][6];
char directions[] = {'N', 'W', 'S', 'E'};
int curr_direction_index = 3; //start facing north
int curr_row = 5;
int curr_col = 3;

//Flags whether or not an objective has been located or not, used in the Explore_Terrain algorithm
bool foundFood = false;
bool foundCandle = false;
bool foundPerson = false;
bool foundGroup = false;

// Task numbers...
#define FIRE_OFF          0
#define COLLECT_FOOD      1
#define FEED_SURVIVORS    2
#define FIND_LOST_PERSON  3

#define TOTAL_NUM_TASKS   4

int task_status[4] = {0}; //All tasks are incomplete at start

int task_location[4][2] = {0};
// This is used to find exact location of each task quickly without having to iterate over the whole terrain map

/* IMU
 *  PID
 */
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

void xTurn_CCW() {
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
  
  Serial.println("Done turing CCW");
  curr_direction_index = (curr_direction_index + 1) % 4; //Set direction index to CCW
  Serial.print("Current Direction:");
  Serial.println(directions[curr_direction_index]);
  delay(2000);
}

void xTurn_CW() {
  Serial.println("Turning CW...");

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

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, LOW);
  
  while (abs(yaw) < desired_yaw) {
    update_yaw();
    analogWrite(ENABLE_M1, speed);
    analogWrite(ENABLE_M2, speed - SPEED_DIFF);
  }

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  Serial.println("Done turing CW");
  curr_direction_index = (curr_direction_index + 3) % 4; //Set direction index to CW
  Serial.print("Current Direction:");
  Serial.println(directions[curr_direction_index]);
  delay(2000);
}

void Move_Forward() {
  
  Serial.println("Moving Forward");
  /*speed = min_fwd_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed  - SPEED_DIFF);
  
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);*/
}

void setup() {
  Serial.begin(9600);
  /*
  //IMU/PID STUFF
  // serial to display data
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
  */

  //LED
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  
  //Flame
  pinMode(FLAME, INPUT);
  
  //Hall effect
  pinMode(hallPin1, INPUT);
  pinMode(hallPin2, INPUT);
  pinMode(hallPin3, INPUT);
  pinMode(hallPin4, INPUT);
  pinMode(hallPin5, INPUT);
  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
  hallState3 = digitalRead(hallPin3);
  hallState4 = digitalRead(hallPin4);
  hallState5 = digitalRead(hallPin5);
  oldState1 = hallState1;
  oldState2 = hallState2;
  oldState3 = hallState3;
  oldState4 = hallState4;
  oldState5 = hallState5;

  //Motors
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);
  speed = min_fwd_speed;

  //Colour Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);

  // initialize map to be unknown/unvisited
  for(int i = 0; i < 6; ++i) {
    for(int j = 0; j < 6; ++j) {
      terrain_map[i][j] = '0';
    }
  }
  //Record starting position
  terrain_map[curr_row][curr_col] = '1';
}

void Print_Map() {
  Serial.println();
  
  for(int i = 0; i < 6; ++i) {
    for(int j =0; j < 6; ++j) {
      if(i == curr_row && j == curr_col) {
        Serial.print("*");
      }
      else {
        Serial.print(terrain_map[i][j]);
      }
      Serial.print(" ");
    }
    Serial.println("");
  }
  Serial.println();
}

void setLEDColor(int redValue, int greenValue, int blueValue) {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

void Move_Backward() {
  speed = min_fwd_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed  - SPEED_DIFF);
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
}

void Stop_Motors() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);  
}

void Backup(long dist) {
  Serial.print(" > Backing up by ");
  Serial.println(dist);
  ultrasonic_dist_back = sr04_back.Distance();
  
  Move_Backward();  

  //Deal with ultrasonic noise
  while(ultrasonic_dist_back < 9) {
    ultrasonic_dist_back = sr04_back.Distance();
    delay(100);
  }

  //keep moving forward until hit objective or go to new tile
  int jump_count_back = 0;
  long initial_ultrasonic = ultrasonic_dist_back;
  while(abs(ultrasonic_dist_back - initial_ultrasonic) < dist || ultrasonic_dist_back == 0) {
    long old_val_back = ultrasonic_dist_back;
    ultrasonic_dist_back = sr04_back.Distance();
    
    if(abs(old_val_back - ultrasonic_dist_back) > 5){
      jump_count_back++;
      if(jump_count_back < 5) {
        Serial.print("too big of jump:   ");
        Serial.println(ultrasonic_dist_back);
        ultrasonic_dist_back = old_val_back;                       
      }
      else {
        jump_count_back = 0;
      }
    }
    Serial.print("  ->");
    Serial.println(ultrasonic_dist_back);                
    delay(100);
  }
  
  Stop_Motors();
  Serial.println(" > Done backing up.");
}

bool FoundEverything() {
  /*
   * Function to check and see if all the objectives in the grid have been located, not necessarily meaning that they have been dealt with
   */
  if(foundFood && foundCandle && foundPerson && foundGroup) {
    return true;
  }
  else {
    return false;
  }
}

bool DetectMagnet() {
  int numDetects = 0;
  hallState1 = digitalRead(hallPin1);
  hallState2 = digitalRead(hallPin2);
  hallState3 = digitalRead(hallPin3);
  hallState4 = digitalRead(hallPin4);
  hallState5 = digitalRead(hallPin5);
  
  if(hallState1 != oldState1) {
    numDetects++;
    Serial.println("Detects");
  }
  if(hallState2 != oldState2) {
    numDetects++;
    Serial.println("Detects");
  }
  if(hallState3 != oldState3) {
    numDetects++;
    Serial.println("Detects");
  }
  if(hallState4 != oldState4) {
    numDetects++;
    Serial.println("Detects");
  }
  if(hallState5 != oldState5) {
    numDetects++;
    Serial.println("Detects");
  }

  oldState1 = hallState1;
  oldState2 = hallState2;
  oldState3 = hallState3;
  oldState4 = hallState4;
  oldState5 = hallState5;

  if(numDetects >= 2) {
    Serial.println("Magnet Detected");
    return true;
  }
  else {
    //Serial.println("No Magnet Detected");
    return false;
  }
}

void ReadColour() {
  // Setting red filtered photodiodes to be read
  //Serial.print("  > Read_Colour: ");
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = pulseIn(sensorOut, LOW);
  //red = map(red, loRed, hiRed, 0, 255);
  delay(100);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  green = pulseIn(sensorOut, LOW);
  //green = map(green, loGreen, hiGreen, 0, 255);
  delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blue = pulseIn(sensorOut, LOW);
  //blue = map(blue, loBlue, hiBlue, 0, 255);
  /*Serial.print("R: ");
  Serial.print(red);
  Serial.print(" G: ");
  Serial.print(green);
  Serial.print(" B: ");
  Serial.println(blue);
  */
  totalRGB = red + blue + green;
}

bool Detect_Yellow_House() {
  ReadColour();
  if(totalRGB < 3500 && red < blue && red < green && green < blue && double(green)/totalRGB > 0.24 && double(green)/totalRGB < 0.44) {
    return true;
  }
  else {
    return false;
  }
}

bool Detect_Red_House() {
  ReadColour();
  if(totalRGB < 4500 && green > blue && green > red && double(green)/double(green+red) > 0.45) {
    return true;
  }
  else {
    return false;
  }
}

void Put_Out_Fire() {
  Move_Forward();
  ultrasonic_dist = sr04.Distance();
  while(analogRead(FLAME)!=0) {
    delay(50);
  }
  delay(200); //go a bit more in
  Stop_Motors();
  delay(1500);
  
  //Backup the distance we travelled
  Backup(abs(ultrasonic_dist - sr04.Distance()));
  Serial.println("  > Candle extinguished");
  
}

void Handle_Object() {
  Serial.println(" > Handle_Object");
  //ReadColour();
  if(Detect_Yellow_House()) {
    Serial.println("  > Detect Yellow House: Found lost person");
    setLEDColor(0, 0, 255); // Yellow Color
    delay(2000);
    setLEDColor(255, 255, 255); // Off
    foundPerson = true;
    task_status[FIND_LOST_PERSON] = 1;
    terrain_map[curr_row][curr_col] = 'P';
    task_location[FIND_LOST_PERSON][0] = curr_row;
    task_location[FIND_LOST_PERSON][1] = curr_col;
    
  }
  else if(Detect_Red_House()){
    Serial.println("  > Detect Red House: Group of Survivors");
    setLEDColor(0, 255, 0); // Purple Color
    delay(2000);
    setLEDColor(255, 255, 255); // Off
    foundGroup = true;
    terrain_map[curr_row][curr_col] = 'G';
    task_location[FEED_SURVIVORS][0] = curr_row;
    task_location[FEED_SURVIVORS][1] = curr_col;
    if(task_status[COLLECT_FOOD]) {
      task_status[FEED_SURVIVORS] = 1;
    }
    else {
      Serial.println("  > Need to collect food before feeding survivors.");
    }
  }
  else if(analogRead(FLAME) != 0) {
    Serial.println("  > Detect Lit Candle");
    setLEDColor(0, 255, 255); // Red Color
    foundCandle = true;
    terrain_map[curr_row][curr_col] = 'C';
    task_location[FIRE_OFF][0] = curr_row;
    task_location[FIRE_OFF][1] = curr_col;
    
    Put_Out_Fire();
    
    setLEDColor(0, 255, 255); // Green Color
    delay(700);
    setLEDColor(0, 0, 0); // Turn LED off
    delay(700);
    setLEDColor(0, 255, 255); // Green Color
    delay(700);
    setLEDColor(0, 255, 255); // Green Color
    delay(700);
    setLEDColor(0, 0, 0); // Turn LED off
    delay(700);
    
    task_status[FIRE_OFF] = 1;
  }
  else if(DetectMagnet()) {
    Serial.println("  > Detect Food");
    setLEDColor(0, 255, 255); // cyan colour
    delay(2000);
    setLEDColor(255, 255, 255); // Turn LED off
    terrain_map[curr_row][curr_col] = 'F';
    task_location[COLLECT_FOOD][0] = curr_row;
    task_location[COLLECT_FOOD][1] = curr_col;
    foundFood = true;
    task_status[COLLECT_FOOD] = 1;
  }
  else {
    Serial.println("  > Nothing read");
    terrain_map[curr_row][curr_col] = '1';
  }
}

void Update_Position(bool forward) {
  char dir = directions[curr_direction_index];
  Serial.print(" > UPDATE_POSITION: (");
  Serial.print(curr_row);
  Serial.print(",");
  Serial.print(curr_col);
  Serial.print(") --> (");
  
  if(!forward) {
    //toggle directions if going backwards
    if(dir == 'N') dir = 'S';
    else if(dir == 'S') dir == 'N';
    else if(dir == 'W') dir == 'E';
    else if(dir == 'E') dir == 'W';
  }
  if(dir == 'N') {
    if(curr_row >= 0) {
      curr_row--;
    }
    else {
      //error, or ran into a wall
    }
  }
  else if(dir == 'W') {
    if(curr_col >= 0) {
      curr_col--;
    }
    else {
      //error, or ran into a wall
    }
  }
  else if(dir == 'S') {
    if(dir == 'S') {
      if(curr_row < 5) {
        curr_row++;
      }
      else {
        //error, or ran into a wall
      }
    }  
  }
  else if(dir == 'E') {
    if(curr_col < 5) {
      curr_col++;
    }
    else {
      //error, or ran into a wall
    }      
  }

  Serial.print(curr_row);
  Serial.print(",");
  Serial.print(curr_col);
  Serial.println(")");
}

bool Layer_Searched(int n){
  /*
   * Check if layer is completely visited
   * There are 3 layers in total
   * n=0, outter layer
   * n=2, inner layer
   */
   
   for(int i = n; i < 6 - n; ++i) {
    if(terrain_map[n][i] == '0') return false; //check top edge
    if(terrain_map[i][n] == '0') return false; //check left edge
    if(terrain_map[i][5 - n] == '0') return false; //check right edge
    if(terrain_map[5 - n][i] == '0') return false; //check bottom edge
   }
   return true;
}

bool Reach_Wall() {
  long dist = sr04.Distance();
  if(dist < 9 && analogRead(FLAME)==0 && !Detect_Yellow_House() && !Detect_Yellow_House()) {
    Serial.println("Reach Wall!");
    wall_reached = true;
    return true;
  }
  else {
    return false;
  }
}

bool Need_To_Explore_Right() {

char dir = directions[curr_direction_index];
if(dir == 'N') {
  if(curr_col < 5 && terrain_map[curr_row][curr_col + 1] == '0') {
    return true;
  }
}
else if(dir == 'S') {
  if(curr_col > 0 && terrain_map[curr_row][curr_col - 1] == '0') {
    return true;
  }
}
else if(dir == 'W') {
  if(curr_row > 0 && terrain_map[curr_row - 1][curr_col] == '0') {
    return true;
  }
}
else if(dir == 'E') {
  if(curr_row < 5 && terrain_map[curr_row + 1][curr_col] == '0') {
    return true;
  }
}
return false;
}

void ExploreTerrain() {
  /*
   * Explore terrain and update the terrain map
   * This code assumes you are already at an edge
   * '1' is a normal tile
  */
  
  Serial.println("Enter Explore Terrain Algorithm");

  int curr_search_layer  = 0; //0 is the outer layer, 2 is the most inner layer
  while(!FoundEverything()) {
    wall_reached = false;
    Serial.println("Explore Terrain Loop");
    Serial.println("Moving Forward..");
    Serial.print("  ->");
    
    Move_Forward();
    ultrasonic_dist = sr04.Distance();
    ultrasonic_dist_back = sr04_back.Distance();
    
    Serial.println(ultrasonic_dist);
    //Deal with ultrasonic noise
    while(ultrasonic_dist < 2 || ultrasonic_dist_back < 2) {
      ultrasonic_dist = sr04.Distance();
      ultrasonic_dist_back = sr04_back.Distance();
      delay(100);
    }

    //keep moving forward until hit objective or go to new tile
    //front ultrasonic
    int jump_count = 0;
    int jumped = 0;
    
    //back ultrasonic
    int jump_count_back = 0;
    int jumped_back = 0;
    
    long initial_ultrasonic = ultrasonic_dist_back;
    long initial_ultrasonic_front = ultrasonic_dist;
    
    //selects one of the ultrasonic that doesn't have a pit infront of it to use as the distance measurement, to decide which one isn't noise the jump vars are used
    while(  
      (((ultrasonic_dist > 8 && !Reach_Wall()) &&
    !(((abs(initial_ultrasonic - ultrasonic_dist_back) >= tile_dist) && jumped_back <= jumped) ||
       (abs(initial_ultrasonic_front - ultrasonic_dist) >= tile_dist) && jumped <= jumped_back)))
    || ultrasonic_dist == 0 || ultrasonic_dist_back == 0) {
      
      long old_val = ultrasonic_dist;
      long old_val_back = ultrasonic_dist_back;
      ultrasonic_dist = sr04.Distance();
      ultrasonic_dist_back = sr04_back.Distance();
      
      if(abs(old_val - ultrasonic_dist) > 5){
        jump_count++;
        if(jump_count < 5) {
          Serial.print("too big of jump (front):   ");
          Serial.println(ultrasonic_dist);
          ultrasonic_dist = old_val;                       
        }
        else {
          jump_count = 0;
          jumped++;
        }
      }
      else {
        jump_count = 0;
      }

      if(abs(old_val_back - ultrasonic_dist_back) > 5){
        jump_count_back++;
        if(jump_count_back < 5) {
          Serial.print("too big of jump (back):   ");
          Serial.println(ultrasonic_dist_back);
          ultrasonic_dist_back = old_val_back;                       
        }
        else {
          jump_count_back = 0;
          jumped_back++;
        }
      }
      
      else {
        jump_count_back = 0;
      }
      Serial.print("  -> FRONT: ");
      Serial.print(ultrasonic_dist);                
      Serial.print("  -> BACK: ");
      Serial.println(ultrasonic_dist_back);                
      delay(100);
    }       

    //Move a tile fully to ensure 30cm dist was travelled
    if(wall_reached) {
      if(abs(initial_ultrasonic_front - ultrasonic_dist) >= tile_dist && jumped <= jumped_back){
        while(abs(initial_ultrasonic_front - ultrasonic_dist) < 30 || ultrasonic_dist == 0) {
          ultrasonic_dist = sr04.Distance();
          delay(50);  
        }
      }
      else {
        while(abs(initial_ultrasonic - ultrasonic_dist_back) < 30 || ultrasonic_dist_back == 0) {
          ultrasonic_dist_back = sr04.Distance();
          delay(50);  
        }        
      }
    }
    
    //Select the correct ultrasonic to use for measurement distance travelled
    if(abs(initial_ultrasonic_front - ultrasonic_dist) >= tile_dist && jumped <= jumped_back){
      ultrasonic_dist_back = ultrasonic_dist;
      initial_ultrasonic = initial_ultrasonic_front;
    }
    Stop_Motors();
    
    //DEBUG
    long dist_travelled = abs(initial_ultrasonic - ultrasonic_dist_back);
    Serial.print("Dist Travelled: ");
    Serial.print(dist_travelled);
    Serial.print("  Front Ultrasonic: ");
    Serial.println(ultrasonic_dist);
    delay(5000);
  
    //Case if we just moved a tile
    if(dist_travelled >= tile_dist || wall_reached) {
      Serial.println("Moved a tile:");
      Update_Position(true);
  
      //Detect food
      Serial.println("Checking for magnet:");
      if(DetectMagnet()) {
        //TODO: Handle Food Object code, indicate that food was detected
        terrain_map[curr_row][curr_col] = 'F';
        task_location[COLLECT_FOOD][0] = curr_row;
        task_location[COLLECT_FOOD][1] = curr_col;
        foundFood = true;
        task_status[COLLECT_FOOD] = 1;
      }
      else {
        terrain_map[curr_row][curr_col] = '1';  
      }
  
      //Do we need to handle object to the right?
      if(Need_To_Explore_Right()){
        Serial.println("Need to explore tile to the right:");
        Turn_CW();
        Stop_Motors();        
      }
      else{
        Serial.println("Don't need to explore tile to the right");  
      }

      //TURN CCW IF WALL REACHED
      if(wall_reached) {
        Turn_CCW();
      }
      
      //Change search direction if needed, i.e. if the outer layer has been searched now search the inner layer
      if(Layer_Searched(curr_search_layer)) {
        Serial.println("Changing search layer");
        curr_search_layer++;
        Turn_CCW();
      }
    }
  
    //Case if we ran into something
    else if(ultrasonic_dist <= 8) {
      Serial.println("Ran into something:");
      
      if(Reach_Wall()) {
        Serial.println("Detected a wall");
        //if a wall
        //Back up the amount we went forward
        Backup(dist_travelled);
        Turn_CCW();
      }
      else {
        //if an object
        Serial.println("Detected an object");
        Update_Position(true);
        Handle_Object();     
        Backup(dist_travelled);
        Update_Position(false); //revert the prior update position
        Turn_CCW();
      }

      //Change search direction if needed, i.e. if the outer layer has been searched now search the inner layer
      if(Layer_Searched(curr_search_layer)) {
        Serial.println("Layer completely searched, changing search layer");
        curr_search_layer++;
        Turn_CCW();
      }
    }
  
    Serial.println("");
    Serial.println("Print current map:");
    Print_Map();
    Serial.println("");
    Serial.println("");
    Serial.println("");
    delay(10000);
  }
}

void Run_Into_Object_Test() {
  Move_Forward();
  ultrasonic_dist = sr04.Distance();
  
  Serial.println(ultrasonic_dist);
  //Deal with ultrasonic noise
  while(ultrasonic_dist < 9) {
    ultrasonic_dist = sr04.Distance();
    delay(100);
  }
  int jump_count = 0;
  while(ultrasonic_dist > 8 || ultrasonic_dist == 0) {
    long old_val = ultrasonic_dist;
    ultrasonic_dist = sr04.Distance();
    
    if(abs(old_val - ultrasonic_dist) > 5){
      jump_count++;
      if(jump_count < 5) {
        Serial.print("too big of jump:   ");
        Serial.println(ultrasonic_dist);
        ultrasonic_dist = old_val;                       
      }
      else {
        jump_count = 0;
      }
    }
    Serial.print("  ->");
    Serial.println(ultrasonic_dist);                
    delay(100);
  }

  Stop_Motors();
  Serial.println("Detected Object");
  Handle_Object();
  Serial.println("Done Function");
  delay(20000);
}

void loop() {
  
  //Locate all of the objectives within the grid
  /*
  Move_Forward();
  if(DetectMagnet()) {
    Serial.println("DETECTED!");
    Stop_Motors();
    setLEDColor(0, 255, 255); // Green Color
    delay(20000);
  }
  delay(50);
  */
  
  ExploreTerrain();
  //CompleteRemainingTasks();
  Serial.println("Done Main Loop");
  delay(40000); //delay 20 sec
}



//#ifndef TILE_STRUCT
//#define TILE_STRUCT
//
//struct Tile {
//  double row_pos;
//  double col_pos;
//  double f;
//  double g;
//  double h;
//  Tile* neighbours;
//  int num_neighbours;
//  Tile* previous;
//};
//
//#endif

// Variables needed for search algorithm -------------------------------
/*
Tile** grid;

Tile* openSet;
Tile* closedSet;
int size_openSet;
int size_closedSet;
#define OPEN_SET_ID     0
#define CLOSED_SET_ID   1

Tile* start_tile;
Tile* end_tile;

Tile* shortest_path;
int size_shortest_path;

// ---------------------------------------------------------------------

void addNeighbours(Tile* tile) {
  int row = tile->row_pos;
  int col = tile->col_pos;
  int num = 0;
  if (row < ROWS-1){ // Not on the last row
    tile->neighbours[num] = grid[row+1][col];
    num++;
  }
  if (row > 0){ // Not on the first row
    tile->neighbours[num] = grid[row-1][col];
    num++;
  }
  if (col < COLS-1){ // Not on the last column
    tile->neighbours[num] = grid[row][col+1];
    num++;
  }
  if (col > 0){ // Not on the first column
    tile->neighbours[num] = grid[row][col-1];
    num++;
  }
  tile->num_neighbours = num;
}

void addToSet(int set_id, Tile* tile_to_add) {
  if(set_id == OPEN_SET_ID){
    openSet[size_openSet] = tile_to_add;
    size_openSet++;
  } else {
    closedSet[size_closedSet] = tile_to_add;
    size_closedSet++;
  }
}

void removeFromSet(int set_id, int tile_to_remove_index) {
  int i;
  if(set_id == OPEN_SET_ID){
    for(i = tile_to_remove_index; i < size_openSet - 1; i++){
      openSet[i] = openSet[i+1];
    }
    size_openSet--;
  } else {
    for(i = tile_to_remove_index; i < size_closedSet - 1; i++){
      closedSet[i] = closedSet[i+1];
    }
    size_closedSet--;
  }
}
int tileInSet(int set_id, Tile* tile) {
  int i;
  if(set_id == OPEN_SET_ID){
    for (i = 0; i < size_openSet; i++) {
      if(tile == openSet[i]) {
        return 1;
      }
    }
    return 0;
  } else {
    for (i = 0; i < size_closedSet; i++) {
      if(tile == closedSet[i]) {
        return 1;
      }
    }
    return 0;
  }
}

double calcHeuristic(Tile* tile) {
  double dist = abs(tile->row_pos-end_tile->row_pos) + abs(tile->col_pos-end_tile->col_pos);
  return dist;
}

void FindShortestPath(int start_row, int start_col, int end_row, int end_col) {
  // Initialization ----------------------------------------------------
  int i, j;

  // Initialize grid
  grid = malloc(sizeof(Tile*) * ROWS);
  for(i = 0; i < ROWS; i++){
    grid[i] = malloc(sizeof(Tile) * COLS);
    for(j = 0; j < COLS; j++) {
      grid[i][j]->row_pos = i;
      grid[i][j]->col_pos = j;
      grid[i][j]->f = 0;
      grid[i][j]->g = 0;
      grid[i][j]->h = 0;
      addNeighbours(grid[i][j]);
      grid[i][j]->previous = NULL;
    }
  }

  // Initialize open and closed set sizes
  size_openSet = 0;
  size_closedSet = 0;
  
  // Initialize start and end tiles
  start_tile = grid[start_row][start_col];
  end_tile = grid[end_row][end_col];
  
  addToSet(OPEN_SET_ID, start_tile);

  // -------------------------------------------------------------------
  
  // Search algorithm --------------------------------------------------
  while (size_openSet > 0) {
    // Keep searching...
    int curr_best_index = 0;
    for (i = 0; i < size_openSet; i++){
      if(openSet[i]->f < openSet[curr_best_index]->f) {
        curr_best_index = i;
      }
    }
    Tile* curr_best = openSet[curr_best_index];
    if(curr_best == end_tile) {
      // We reached the end!!!
      size_shortest_path = 0;
      Tile* temp = current;
      while (temp->previous) {
        shortest_path[size_shortest_path]=temp;
        size_shortest_path++;
        temp = temp->previous;
      }
      return;
    }
    removeFromSet(OPEN_SET_ID, curr_best_index);
    addToSet(CLOSED_SET_ID, curr_best);
    
    for(i = 0; i < current->num_neighbours; i++) {
      Tile* neighbour = current->neighbours[i];
      if(!tileInSet(CLOSED_SET_ID, neighbour)) {
        double temp_g = current->g + 1;
        if(tileInSet(OPEN_SET_ID, neighbour)) {
          if(temp_g < neighbour->g) {
            neighbour->g = temp_g;
          }
        } else {
          neighbour->g = temp_g;
          addToSet(OPEN_SET_ID, neighbour);
        }
        neighbour->h = calcHeuristic(neighbour);
        neighbour->f = neighbour->g + neighbour->h;
        neighbour->previous = current;
        
      }
    }
  }
    // No solution, blocked!
  // -------------------------------------------------------------------
}

void CompleteRemainingTasks() {
  // Determine closest on-queue task that can actually be completed
  int upcoming_task = 0;
  int approx_distance = 26; // Max distance would be 5+5, so add 1 to make it larger than any possible distance value
  int i;
  
  for (i = 0; i < TOTAL_NUM_TASKS; i++) {
    if (task_status[i] == 0) {
      // Task not complete yet, check if valid task...
      // ASSUMPTION: Remember if the whole terrain has already been explored by this point, fire must already be out, skip this check
      if (i != FEED_SURVIVORS || task_status[COLLECT_FOOD] == 1) {
        // Valid task, check distance to task...
        approx_distance_i = abs(task_location[i][0]-curr_row) + abs(task_location[i][1]-curr_col);
        if (approx_distance_i < approx_distance) {
          // Task at i is closer, update values...
          upcoming_task = i;
          approx_distance = approx_distance_i;
        }
      }
    }
  }
  
  // Use A* to go to the location
  // Handle object
  // If all tasks are not complete, repeat process
}
*/

void Turn_CW() {
  /*
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, LOW);
  //TODO: adjust this to use magnetometer
  delay(2000); 
  Stop_Motors();
  */
  Serial.println("> Turn_CW_Test");
  curr_direction_index = (curr_direction_index + 3) % 4; //Set direction index to CW
  Serial.print("Current Direction:");
  Serial.println(directions[curr_direction_index]);
  delay(5000); 
  Stop_Motors();  
}

void Turn_CCW() {
  /*
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, HIGH);
  //TODO: adjust this to use magnetometer
  delay(2000); 
  Stop_Motors();
  */
  Serial.println("> Turn_CCW_Test");
  curr_direction_index = (curr_direction_index + 1) % 4; //Set direction index to CCW
  Serial.print("Current Direction:");
  Serial.println(directions[curr_direction_index]);
  delay(5000); 
  Stop_Motors();
}
