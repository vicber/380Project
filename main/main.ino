/* BB's Main Code for the Search and Resuce Mission */

#include <stdlib.h>
#include <SR04.h>
#include <MPU9250.h>
// #include <PID_v1.h>

// Some definitions ------------------------------------------
enum TASK_ID {
  PUT_OUT_FIRE = 0,
  FIND_FOOD = 1,
  FEED_SURVIVORS = 2,
  FIND_LOST_PERSON = 3
};

typedef struct TASK{
  TASK_ID task_id;
  int task_status; // 0: Not complete, 1: Complete
  int obj_location[2]; // -1 means unknown
  int obj_location_known;
} Task;

enum DIRECTION{
  NORTH = 0,
  WEST = 1,
  SOUTH = 2,
  EAST = 3
};

enum TURN_DIRECTION{
  CCW = 0,
  CW = 1
};

// Position and direction parameters -------------------------
#define ROW_INDEX   0
#define COL_INDEX   1

#define NUM_ROWS    6
#define NUM_COLS    6

// Start position: (5, 3), last row, fourth tile from the left
#define START_ROW   5
#define START_COL   3
int curr_pos[2] = {START_ROW, START_COL};

DIRECTION curr_dir = EAST; // Assuming we start facing to the right

#define DIST_ONE_TILE   23
// 1ft is slightly over 30cm

#define DIST_TO_OBSTACLE  5
// Stop when obstacle is 8cm ahead

// Tile traversal reference array
int arr_n_r[3][2] = {{5, 2}, {3, 1}, {1, 0}};

// Tracks which tiles have been already visited
// 0 --> unvisited, 1 --> visited
int terrain_map[NUM_ROWS][NUM_COLS] = {0};

#define NUM_GRID_LAYERS   3
#define NUM_SIDES_MINUS_ONE  3

int curr_layer;

// A* search parameters --------------------------------------
#define MAX_NEIGHBOURS  4
#define NUM_OBSTACLES   2

typedef struct TILE{
  int row_pos;
  int col_pos;
  double f;
  double g;
  double h;
  struct TILE* neighbours[MAX_NEIGHBOURS];
  int num_neighbours;
  struct TILE* previous;
  int is_obstacle;
}Tile;

Tile* grid[NUM_ROWS][NUM_COLS];

Tile* openSet[NUM_ROWS*NUM_COLS];
Tile* closedSet[NUM_ROWS*NUM_COLS];
int size_openSet;
int size_closedSet;
#define OPEN_SET_ID     0
#define CLOSED_SET_ID   1

Tile* start_tile;
Tile* end_tile;

Tile* shortest_path[NUM_ROWS*NUM_COLS];
int size_shortest_path;

// For each obstacle, track row and column
int obstacles[NUM_OBSTACLES][2];

#define MAX_NUM_TASKS_REMAINING 3

// Task parameters -------------------------------------------
#define NUM_TASKS   4

Task tasks[NUM_TASKS];

// Motor parameters ------------------------------------------
#define ENABLE_M1 3
#define DIR_A_M1  4 
#define DIR_B_M1  5 

#define ENABLE_M2 11
#define DIR_A_M2  12
#define DIR_B_M2  13

#define MAX_FWD_SPEED   255
#define MIN_FWD_SPEED   220

#define MAX_TURN_SPEED  255
#define MIN_TURN_SPEED  190

int motor_speed;

#define SPEED_DIFF  28
#define SPEED_REDUCTION_FOR_HANDLING_FLAME  30

// IMU parameters --------------------------------------------
// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int imu_status;
double yaw, desired_yaw; // DEGREES

#define NUM_DEGREES_TURN    85.0

// Gyroscope parameters --------------------------------------
double omega_z; // RAD/SEC
double theta_z; // RAD

#define GZ_OFFSET   -0.000434851
#define THETA_SCALING  1.003786916

// Dead pass filter parameters ----------
#define GZ_DP_CUTOFF 0.25 // RAD/SEC

// Low pass filter parameters -----------
double curr_LP_filtered_data, prev_LP_filtered_data; // RAD/SEC

#define LP_CUTOFF_FREQ_HZ  5

double RC_LP = 1.0/(2*M_PI*LP_CUTOFF_FREQ_HZ);
double alpha_LP;

// Time parameters ----------------------
int t1, t2;
double dt;

// PID controller parameters ---------------------------------
double calculated_motor_speed;
int set_pid_max = 1;
double pid_max;
double Kp=0.05, Ki=5, Kd=0.1;
// PID turnPID(&yaw, &calculated_motor_speed, &desired_yaw, Kp, Ki, Kd, DIRECT);

// Ultrasonic sensor parameters ------------------------------
#define TRIG_PIN 15
#define ECHO_PIN 14

SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long dist_travelled; // CM
long prev_dist, curr_dist; // CM

#define MAX_DIST_DIFF 30

// Hall effect sensor parameters -----------------------------
#define HALL_EFFECT_PIN1  53
#define HALL_EFFECT_PIN2  51
#define HALL_EFFECT_PIN3  49
#define HALL_EFFECT_PIN4  47
#define HALL_EFFECT_PIN5  45

#define NUM_HALL_EFFECT_SENSORS 5

int hall_effect_states[NUM_HALL_EFFECT_SENSORS];
int old_hall_effect_states[NUM_HALL_EFFECT_SENSORS];

// Flame sensor parameters -----------------------------------
#define FLAME_SENSOR_PIN  A7

#define FLAME_SENSOR_THRESHOLD  5

// Colour sensor parameters ----------------------------------
#define COLOUR_SENSOR_PIN_S0   30
#define COLOUR_SENSOR_PIN_S1   32
#define COLOUR_SENSOR_PIN_S2   34
#define COLOUR_SENSOR_PIN_S3   36
#define COLOUR_SENSOR_PIN_OUT  28

//#define RED_LOW_VAL     146
//#define RED_HIGH_VAL    60
//#define BLUE_LOW_VAL    59
//#define BLUE_HIGH_VAL   378
//#define GREEN_LOW_VAL   277
//#define GREEN_HIGH_VAL  121

//#define MAX_TOTAL_RGB_VAL_YELLOW_HOUSE   3500
//#define MAX_TOTAL_RGB_VAL_RED_HOUSE      4500
//
//#define YELLOW_HOUSE_G_MIN_PER 0.24
//#define YELLOW_HOUSE_G_MAX_PER 0.44
//#define RED_HOUSE_G_DIV_GR_PER 0.45

#define YELLOW_HOUSE_R_MIN  0
#define YELLOW_HOUSE_R_MAX  1000
#define YELLOW_HOUSE_G_MIN  500
#define YELLOW_HOUSE_G_MAX  1500
#define YELLOW_HOUSE_B_MIN  1000
#define YELLOW_HOUSE_B_MAX  2000

#define RED_HOUSE_R_MIN  500
#define RED_HOUSE_R_MAX  1500
#define RED_HOUSE_G_MIN  2500
#define RED_HOUSE_G_MAX  3500
#define RED_HOUSE_B_MIN  1500
#define RED_HOUSE_B_MAX  2500

#define YELLOW_HOUSE_ID   0
#define RED_HOUSE_ID      1

// LED parameters --------------------------------------------
#define LED_PIN 8

// -----------------------------------------------------------

// Function definitions --------------------------------------

void blink_led(Task task){
  // Blink LED to indicate task complete
  int i;
  for(i=0; i < task.task_id+1; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    delay(1000);
  }
}

double apply_dead_pass_gz(double gz) {
  if (abs(gz) < GZ_DP_CUTOFF) {
    return 0.0;
  } else {
    return gz;
  }
}

void reset_gyro_params() {
  omega_z = 0.0;
  theta_z = 0.0;
  yaw = 0.0;

  IMU.readSensor();
  t1 = millis();
  
  curr_LP_filtered_data = apply_dead_pass_gz(IMU.getGyroZ_rads() + GZ_OFFSET);
  prev_LP_filtered_data = curr_LP_filtered_data;

  set_pid_max = 1;
}

void update_yaw_calc_motor_speed() {
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
  yaw = abs(180*theta_z/M_PI);
//  Serial.print("Yaw [deg]: ");
//  Serial.println(yaw);

  // Adjust the output according to current yaw value
//  turnPID.Compute();
//  Serial.print("\tCalculated speed: ");
//  Serial.print(calculated_motor_speed);
//  if (set_pid_max) {
//    pid_max = calculated_motor_speed;
//    set_pid_max = 0;
//  }
//  motor_speed = calculated_motor_speed;
////  motor_speed = map(calculated_motor_speed, 0, pid_max+10, MIN_TURN_SPEED, MAX_TURN_SPEED);
//  Serial.print("\tSet speed: ");
//  Serial.println(motor_speed);
}

void reset_dist_params() {
  dist_travelled = 0;
  curr_dist = sr04.Distance();
  prev_dist = curr_dist;
}

long update_dist_travelled() {
  curr_dist = sr04.Distance();
  Serial.print("Current distance: ");
  Serial.println(curr_dist);
  if (abs(curr_dist - prev_dist) < MAX_DIST_DIFF) {
    Serial.println(dist_travelled);
    Serial.println(prev_dist-curr_dist);
    dist_travelled = dist_travelled + (prev_dist - curr_dist);
    prev_dist = curr_dist;
  }
  Serial.print("Distance travelled: ");
  Serial.println(dist_travelled);
  delay(50);
}

int detect_house(){
  int red, green, blue, total_rgb;
  
  digitalWrite(COLOUR_SENSOR_PIN_S2, LOW);
  digitalWrite(COLOUR_SENSOR_PIN_S3, LOW);
  red = pulseIn(COLOUR_SENSOR_PIN_OUT, LOW);
//  red = map(red, RED_LOW_VAL, RED_HIGH_VAL, 0, 255);
  delay(100);

  digitalWrite(COLOUR_SENSOR_PIN_S2, HIGH);
  digitalWrite(COLOUR_SENSOR_PIN_S3, HIGH);
  green = pulseIn(COLOUR_SENSOR_PIN_OUT, LOW);
//  green = map(green, GREEN_LOW_VAL, GREEN_HIGH_VAL, 0, 255);
  delay(100);

  digitalWrite(COLOUR_SENSOR_PIN_S2, LOW);
  digitalWrite(COLOUR_SENSOR_PIN_S3, HIGH);
  blue = pulseIn(COLOUR_SENSOR_PIN_OUT, LOW);
//  blue = map(blue, BLUE_LOW_VAL, BLUE_HIGH_VAL, 0, 255);
  delay(100);

  if(red > YELLOW_HOUSE_R_MIN && red < YELLOW_HOUSE_R_MAX &&
     green > YELLOW_HOUSE_G_MIN && green < YELLOW_HOUSE_G_MAX &&
     blue > YELLOW_HOUSE_B_MIN && blue < YELLOW_HOUSE_B_MAX){
    
    Serial.println("Detecting yellow house!!!");
    tasks[FIND_LOST_PERSON].obj_location[ROW_INDEX]=curr_pos[ROW_INDEX];
    tasks[FIND_LOST_PERSON].obj_location[COL_INDEX]=curr_pos[COL_INDEX];
    tasks[FIND_LOST_PERSON].obj_location_known = 1;
    Serial.println("Recorded lost person location: (");
    Serial.print(tasks[FIND_LOST_PERSON].obj_location[ROW_INDEX]);
    Serial.print(", ");
    Serial.print(tasks[FIND_LOST_PERSON].obj_location[COL_INDEX]);
    Serial.println(")");
    return YELLOW_HOUSE_ID;
    
  } else if(red > RED_HOUSE_R_MIN && red < RED_HOUSE_R_MAX &&
            green > RED_HOUSE_G_MIN && green < RED_HOUSE_G_MAX &&
            blue > RED_HOUSE_B_MIN && blue < RED_HOUSE_B_MAX){
    
    Serial.println("Detecting red house!!!");
    tasks[FEED_SURVIVORS].obj_location[ROW_INDEX]=curr_pos[ROW_INDEX];
    tasks[FEED_SURVIVORS].obj_location[COL_INDEX]=curr_pos[COL_INDEX];
    tasks[FEED_SURVIVORS].obj_location_known = 1;
    Serial.print("Recorded survivors location: (");
    Serial.print(tasks[FEED_SURVIVORS].obj_location[ROW_INDEX]);
    Serial.print(", ");
    Serial.print(tasks[FEED_SURVIVORS].obj_location[COL_INDEX]);
    Serial.println(")");
    return RED_HOUSE_ID;
    
  } else {
    
    Serial.println("House not detected.");
    return -1;
    
  }
}

void handle_house(int house_id){
  if(house_id == YELLOW_HOUSE_ID){
    // Lost person
    if (tasks[PUT_OUT_FIRE].task_status == 1){
      Serial.println("Saving lost person. Find lost person task complete.");
      tasks[FIND_LOST_PERSON].task_status = 1;
    } else {
      Serial.println("Not saving lost person yet. Fire has not been put out.");
    }
  } else{
    // Survivors
    if (tasks[PUT_OUT_FIRE].task_status == 1 && tasks[FIND_FOOD].task_status == 1){
      Serial.println("Feeding survivors. Feed survivors task complete.");
      tasks[FIND_LOST_PERSON].task_status = 1;
    } else {
      Serial.println("Not feeding survivors yet. Either food has not yet been found or fire has not been put out.");
    }
  }
}

int detect_flame(){
  int detect_flame;
  int flame_sensor_val = analogRead(FLAME_SENSOR_PIN);
  if (flame_sensor_val > FLAME_SENSOR_THRESHOLD) {
    Serial.println("Detecting flame!!!");
    detect_flame = 1;
    tasks[PUT_OUT_FIRE].obj_location[ROW_INDEX] = curr_pos[ROW_INDEX];
    tasks[PUT_OUT_FIRE].obj_location[COL_INDEX] = curr_pos[COL_INDEX];
    tasks[PUT_OUT_FIRE].obj_location_known = 1;
    Serial.print("Recorded flame location: (");
    Serial.print(tasks[PUT_OUT_FIRE].obj_location[ROW_INDEX]);
    Serial.print(", ");
    Serial.print(tasks[PUT_OUT_FIRE].obj_location[COL_INDEX]);
    Serial.println(")");
  } else {
    Serial.println("Flame not detected.");
    detect_flame = 0;
  }
  return detect_flame;
}

long handle_flame(){
  long extra_dist_travelled;
  motor_speed = MIN_FWD_SPEED - SPEED_REDUCTION_FOR_HANDLING_FLAME;
  analogWrite(ENABLE_M1, motor_speed);
  analogWrite(ENABLE_M2, motor_speed - SPEED_DIFF);

  reset_dist_params();
  
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);

  while (detect_flame() == 1){
    update_dist_travelled();
  }

  delay(300);

  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
  
  update_dist_travelled();
  extra_dist_travelled = dist_travelled;

  delay(1000);

  // Assuming flame is already out
  Serial.println("Flame is extinguished. Put out fire task is complete.");
  tasks[PUT_OUT_FIRE].task_status = 1;

  return extra_dist_travelled;
}

void detect_and_handle_food(){
  int i;
  
  hall_effect_states[1] = digitalRead(HALL_EFFECT_PIN1);
  hall_effect_states[2] = digitalRead(HALL_EFFECT_PIN2);
  hall_effect_states[3] = digitalRead(HALL_EFFECT_PIN3);
  hall_effect_states[4] = digitalRead(HALL_EFFECT_PIN4);
  hall_effect_states[5] = digitalRead(HALL_EFFECT_PIN5);
  
  if(hall_effect_states[1] != old_hall_effect_states[1] ||
     hall_effect_states[2] != old_hall_effect_states[2] ||
     hall_effect_states[3] != old_hall_effect_states[3] ||
     hall_effect_states[4] != old_hall_effect_states[4] ||
     hall_effect_states[5] != old_hall_effect_states[5]) {
    Serial.println("Detecting food!!!");
    tasks[FIND_FOOD].obj_location[ROW_INDEX] = curr_pos[ROW_INDEX];
    tasks[FIND_FOOD].obj_location[COL_INDEX] = curr_pos[COL_INDEX];
    tasks[FIND_FOOD].obj_location_known = 1;
    Serial.print("Recorded food location: (");
    Serial.print(tasks[FIND_FOOD].obj_location[ROW_INDEX]);
    Serial.print(", ");
    Serial.print(tasks[FIND_FOOD].obj_location[COL_INDEX]);
    Serial.println(")");
    if (tasks[PUT_OUT_FIRE].task_status == 1){
      Serial.println("Picking up food. Find food task complete.");
      tasks[FIND_FOOD].task_status = 1;
    } else {
      Serial.println("Not picking up food yet. Fire has not been put out.");
    }
  } else {
    Serial.println("No food detected.");
  }
  for(i = 0; i < NUM_HALL_EFFECT_SENSORS; i++){
    old_hall_effect_states[i] = hall_effect_states[i];
  }
}

void move_bwd(int dist){
  Serial.println("Moving backward...");
  Serial.print("Distance: ");
  Serial.println(dist);
  
  motor_speed = MIN_FWD_SPEED;
  analogWrite(ENABLE_M1, motor_speed);
  analogWrite(ENABLE_M2, motor_speed - SPEED_DIFF);

  reset_dist_params();
  
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);

  while (dist_travelled > dist){
    update_dist_travelled();
  }

  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);

  // Update current position
  if (curr_dir == NORTH){
    // Went down
    // Increment row
    curr_pos[ROW_INDEX]++;
  } else if (curr_dir == WEST){
    // Went right
    // Increment column
    curr_pos[COL_INDEX]++;
  } else if (curr_dir == SOUTH){
    // Went up
    // Decrement row
    curr_pos[ROW_INDEX]--;
  } else{
    // Went left
    // Decrement column
    curr_pos[COL_INDEX]--;
  }
};

int move_fwd(int dist){
  long orig_dist_travelled, extra_dist_travelled;
  int house_detected_id, obstacle_ahead = 0;

  Serial.println("Moving forward...");
  
  motor_speed = MIN_FWD_SPEED;
  analogWrite(ENABLE_M1, motor_speed);
  analogWrite(ENABLE_M2, motor_speed - SPEED_DIFF);

  reset_dist_params();
  
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
  
  while (dist_travelled < dist && curr_dist > DIST_TO_OBSTACLE){
    update_dist_travelled();
  }

  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);

  orig_dist_travelled = dist_travelled;

  // Update current position and terrain map
  if (curr_dir == NORTH){
    // Went up
    // Decrement row
    curr_pos[ROW_INDEX]--;
  } else if (curr_dir == WEST){
    // Went left
    // Decrement column
    curr_pos[COL_INDEX]--;
  } else if (curr_dir == SOUTH){
    // Went down
    // Increment row
    curr_pos[ROW_INDEX]++;
  } else{
    // Went right
    // Increment column
    curr_pos[COL_INDEX]++;
  }
  terrain_map[curr_pos[ROW_INDEX]][curr_pos[COL_INDEX]] = 1;

  if (curr_dist <= DIST_TO_OBSTACLE){
    Serial.println("Object in front. Evaluating...");
    if (detect_flame() == 1){
      Serial.println("Detected flame (move_fwd())...");
      extra_dist_travelled = handle_flame();
      move_bwd(orig_dist_travelled+extra_dist_travelled);
    } else {
        house_detected_id = detect_house();
        Serial.print("House detected id return value: ");
        Serial.println(house_detected_id);
        if (house_detected_id != -1){
          Serial.println("Detected house (move_fwd())...");
          handle_house(house_detected_id);
          move_bwd(orig_dist_travelled);
        } else {
          Serial.println("Detecting wall, continue...");
          move_bwd(orig_dist_travelled);
        }
    }
    obstacle_ahead = 1;
  }

  if (tasks[FIND_FOOD].task_status == 0){
    detect_and_handle_food();
  }

  Serial.print("Current position: (");
  Serial.print(curr_pos[ROW_INDEX]);
  Serial.print(", ");
  Serial.print(curr_pos[COL_INDEX]);
  Serial.println(")");
  delay(500);
  return obstacle_ahead;
};

void turn(TURN_DIRECTION turn_dir){
  Serial.println("Turning...");
  
  // Set initial parameters
  reset_gyro_params();
  motor_speed = MIN_TURN_SPEED;
  
  // Initialize PID Parameters
  desired_yaw = NUM_DEGREES_TURN; // 81 degrees
//  turnPID.SetMode(AUTOMATIC); // Turn the PID on
//  turnPID.SetTunings(Kp, Ki, Kd);

  analogWrite(ENABLE_M1, motor_speed);
  analogWrite(ENABLE_M2, motor_speed - SPEED_DIFF);

  if (turn_dir == CCW) {
    digitalWrite(DIR_A_M1, HIGH);
    digitalWrite(DIR_A_M2, LOW);
    digitalWrite(DIR_B_M1, LOW);
    digitalWrite(DIR_B_M2, HIGH);
  } else {
    digitalWrite(DIR_A_M1, LOW);
    digitalWrite(DIR_A_M2, HIGH);
    digitalWrite(DIR_B_M1, HIGH);
    digitalWrite(DIR_B_M2, LOW);
  }
  
  while (yaw < desired_yaw) {
    update_yaw_calc_motor_speed();
//    analogWrite(ENABLE_M1, motor_speed);
//    analogWrite(ENABLE_M2, motor_speed - SPEED_DIFF);
  }

  if (turn_dir == CCW) {
    digitalWrite(DIR_A_M1, LOW);
    digitalWrite(DIR_B_M2, LOW);
    // Update direction
    curr_dir = (curr_dir+1)%4;
  } else {
    digitalWrite(DIR_A_M2, LOW);
    digitalWrite(DIR_B_M1, LOW);
    // Update direction
    curr_dir = curr_dir-1;
    if (curr_dir == -1){
      curr_dir = 3;
    }
  }
}

int all_tiles_in_layer_explored(){
  int i;
  for(i = 0+curr_layer; i < NUM_COLS-curr_layer; i++){
      if(terrain_map[0+curr_layer][i] == 0 || terrain_map[(NUM_ROWS-1)-curr_layer][i] == 0){
        // Not all tiles are explored
        return 0;
      }
    }
    for(i = 1+curr_layer; i < (NUM_ROWS-1)-curr_layer; i++){
      if(terrain_map[i][0+curr_layer] == 0 || terrain_map[i][(NUM_COLS-1)-curr_layer] == 0){
        // Not all tiles are explored
        return 0;
      }
    }
    // All tiles in layer have been explored
    // Will be moving to next layer so increment curr_layer
    curr_layer++;
    return 1;
}

int located_all_objectives(){
  // Check if all objectives have been located
  if (tasks[FIND_FOOD].obj_location_known && tasks[FIND_LOST_PERSON].obj_location_known &&
      tasks[FEED_SURVIVORS].obj_location_known && tasks[PUT_OUT_FIRE].obj_location_known){
    // Know where everything is
    return 1;
  } else {
    // Don't know where everything is
    return 0;
  }
}

int tasks_all_done(){
  return (tasks[FIND_FOOD].task_status == 1) && (tasks[FIND_LOST_PERSON].task_status == 1) &&
         (tasks[FEED_SURVIVORS].task_status == 1) && (tasks[PUT_OUT_FIRE].task_status == 1);
}

void add_neighbours(Tile* tile) {
  int row = tile->row_pos;
  int col = tile->col_pos;
  int num = 0;
  if (row < NUM_ROWS-1){ // Not on the last row
    tile->neighbours[num] = grid[row+1][col];
    num++;
  }
  if (row > 0){ // Not on the first row   
    tile->neighbours[num] = grid[row-1][col];
    num++;
  }
  if (col < NUM_COLS-1){ // Not on the last column
    tile->neighbours[num] = grid[row][col+1];
    num++;
  }
  if (col > 0){ // Not on the first column
    tile->neighbours[num] = grid[row][col-1];
    num++;
  }
  tile->num_neighbours = num;
}

void add_to_set(int set_id, Tile* tile_to_add) {
  if(set_id == OPEN_SET_ID){
    openSet[size_openSet] = tile_to_add;
    size_openSet++;
  } else {
    closedSet[size_closedSet] = tile_to_add;
    size_closedSet++;
  }
}

void remove_from_set(int set_id, int tile_to_remove_index) {
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

int tile_in_set(int set_id, Tile* tile) {
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

double calc_heuristic(Tile* tile) {
  double dist = abs(tile->row_pos-end_tile->row_pos) + abs(tile->col_pos-end_tile->col_pos);
  return dist;
}

void dealloc_grid(){
  int i, j;
  for(i = 0; i < NUM_ROWS; i++) {
    for(j = 0; j < NUM_COLS; j++) {
      free(grid[i][j]);
    }
  }
}

int in_shortest_path(Tile* tile){
  int i;
  for (i = 0; i < size_shortest_path; i++) {
    if(tile == shortest_path[i]) {
      return 1;
    }
  }
  return 0;
}

// CONSIDER: start and end should not be obstacles in
//           order for the path to make sense and have
//           a solution
void init_obstacles(){
  int i, row, col;
  for(i = 0; i < NUM_OBSTACLES; i++) {
    row = obstacles[i][0];
    col = obstacles[i][1]; 
    grid[row][col]->is_obstacle = 1;
  }
}

void determine_shortest_path(Tile* curr_best) {
  int i, j;
  size_shortest_path = 0;
  Tile* temp = curr_best;
  shortest_path[size_shortest_path]=temp;
  size_shortest_path++;
  while (temp->previous) {
    shortest_path[size_shortest_path]=temp->previous;
    size_shortest_path++;
    temp = temp->previous;
  }
  Serial.print("PATH: ");
  for (i = size_shortest_path-1; i >= 0; i--) {
    Serial.print(shortest_path[i]->row_pos);
    Serial.print(", ");
    Serial.print(shortest_path[i]->col_pos);
    Serial.print("; ");
  }
  Serial.println();

  // Output to screen
  for (i = 0; i < NUM_ROWS; i++){
    for (j = 0; j < NUM_COLS; j++){
      if(grid[i][j]->is_obstacle) {
        Serial.print("X ");
      } else if(in_shortest_path(grid[i][j])) {
        Serial.print("* ");
      } else if(tile_in_set(OPEN_SET_ID, grid[i][j])) {
        Serial.print("O ");
      } else if(tile_in_set(CLOSED_SET_ID, grid[i][j])) {
        Serial.print("C ");
      } else {
        Serial.print("N ");
      }
    }
    Serial.println();
  }
  Serial.println();
  Serial.println();
  Serial.println();
}

void find_shortest_path(int start_row, int start_col, int end_row, int end_col) {
  // Initialization ----------------------------------------------------
  int i, j;

  // Initialize grid
  for(i = 0; i < NUM_ROWS; i++){
    for(j = 0; j < NUM_COLS; j++) {
      grid[i][j]=(Tile*)malloc(sizeof(Tile));
    }
  }
  for(i = 0; i < NUM_ROWS; i++){
    for(j = 0; j < NUM_COLS; j++) {
      grid[i][j]->row_pos = i;
      grid[i][j]->col_pos = j;
      grid[i][j]->f = 0;
      grid[i][j]->g = 0;
      grid[i][j]->h = 0;
      add_neighbours(grid[i][j]);
      grid[i][j]->previous = NULL;
      grid[i][j]->is_obstacle = 0;
    }
  }
  
  init_obstacles();

  // Initialize open and closed set sizes
  size_openSet = 0;
  size_closedSet = 0;
  
  // Initialize start and end tiles
  start_tile = grid[start_row][start_col];
  end_tile = grid[end_row][end_col];
  
  add_to_set(OPEN_SET_ID, start_tile);

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
      determine_shortest_path(curr_best);
      dealloc_grid();
      Serial.println("DONE!!!");
      return;
    }
    remove_from_set(OPEN_SET_ID, curr_best_index);
    add_to_set(CLOSED_SET_ID, curr_best);
    
    for(i = 0; i < curr_best->num_neighbours; i++) {
      Tile* neighbour = curr_best->neighbours[i];
      if(!tile_in_set(CLOSED_SET_ID, neighbour) && !neighbour->is_obstacle) {
        double temp_g = curr_best->g + 1;
        if(tile_in_set(OPEN_SET_ID, neighbour)) {
          if(temp_g < neighbour->g) {
            neighbour->g = temp_g;
          }
        } else {
          neighbour->g = temp_g;
          add_to_set(OPEN_SET_ID, neighbour);
        }
        neighbour->h = calc_heuristic(neighbour);
        neighbour->f = neighbour->g + neighbour->h;
        neighbour->previous = curr_best;
      }
    }
  }
  // No solution, blocked!
  dealloc_grid();
  Serial.println("No solution!!!");
  return;
  // -------------------------------------------------------------------
}

void fix_orientation(Tile* current_tile, Tile* next_tile) {
  int i;
  int num_90_deg_turns = 0;
  int curr_row = current_tile->row_pos;
  int curr_col = current_tile->col_pos;
  int next_row = next_tile->row_pos;
  int next_col = next_tile->col_pos;
  int new_dir = 0; // 0: N, 1: W, 2: S, 3: E
  
  if (curr_row-next_row == 1) {
    // Need to go N
    new_dir = 0;
  } else if (curr_col-next_row == 1) {
    // Need to go W
    new_dir = 1;
  } else if (curr_row-next_row == -1) {
    //Need to go S
    new_dir = 2;
  } else {
    // Need to go E
    new_dir = 3;
  }
  
  if (new_dir - curr_dir == 0) {
    // No need to fix orientation, already in correct orientation!
    return;
  } else if (new_dir - curr_dir > 0) {
      num_90_deg_turns = new_dir - curr_dir;
  } else {
      num_90_deg_turns = (4 - curr_dir - new_dir);
  }
  for (i = 0; i < num_90_deg_turns; i++){
    turn(CCW);
  }
  curr_dir = new_dir;
  
  return;
}

void go_through_shortest_path() {
  Tile* curr_tile_in_path = shortest_path[size_shortest_path-1];
  while(curr_tile_in_path != shortest_path[0]) { // May need to change end condition
    fix_orientation(curr_tile_in_path, curr_tile_in_path->previous);
    move_fwd(DIST_ONE_TILE);
    curr_tile_in_path = curr_tile_in_path->previous;
  }
}

int calc_dist_from_curr_pos(Task task){
  int dist_from_curr_pos;
  dist_from_curr_pos = abs(curr_pos[ROW_INDEX]-task.obj_location[ROW_INDEX]);
  dist_from_curr_pos += abs(curr_pos[COL_INDEX]-task.obj_location[COL_INDEX]);
  return dist_from_curr_pos;
}

// Setup code ------------------------------------------------

void setup() {
  int i;

  Serial.begin(9600);
  while(!Serial){}

  // Setup motors
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);
  motor_speed = MIN_FWD_SPEED;

  // Setup IMU
  imu_status = IMU.begin();
  if (imu_status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imu_status);
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

  // Setup Hall effect sensor
  pinMode(HALL_EFFECT_PIN1, INPUT);
  pinMode(HALL_EFFECT_PIN2, INPUT);
  pinMode(HALL_EFFECT_PIN3, INPUT);
  pinMode(HALL_EFFECT_PIN4, INPUT);
  pinMode(HALL_EFFECT_PIN5, INPUT);

  hall_effect_states[1] = digitalRead(HALL_EFFECT_PIN1);
  hall_effect_states[2] = digitalRead(HALL_EFFECT_PIN2);
  hall_effect_states[3] = digitalRead(HALL_EFFECT_PIN3);
  hall_effect_states[4] = digitalRead(HALL_EFFECT_PIN4);
  hall_effect_states[5] = digitalRead(HALL_EFFECT_PIN5);

  for(i = 0; i < NUM_HALL_EFFECT_SENSORS; i++){
    old_hall_effect_states[i] = hall_effect_states[i];
  }

  // Setup colour sensor
  pinMode(COLOUR_SENSOR_PIN_S0, OUTPUT);
  pinMode(COLOUR_SENSOR_PIN_S1, OUTPUT);
  pinMode(COLOUR_SENSOR_PIN_S2, OUTPUT);
  pinMode(COLOUR_SENSOR_PIN_S3, OUTPUT);
  pinMode(COLOUR_SENSOR_PIN_OUT, OUTPUT);
  // Set frequency scaling to 20%
  digitalWrite(COLOUR_SENSOR_PIN_S0, HIGH);
  digitalWrite(COLOUR_SENSOR_PIN_S1, HIGH);

  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize tasks array
  for (i=0; i < NUM_TASKS; i++){
    tasks[i].task_id = i;
    tasks[i].task_status = 0;
    tasks[i].obj_location[ROW_INDEX] = -1; // -1 means unknown
    tasks[i].obj_location[COL_INDEX] = -1; // -1 means unknown
    tasks[i].obj_location_known = 0;
  }

  // Initialize terrain map
  terrain_map[START_ROW][START_COL] = 1;

  curr_layer = 0;
}

// Main loop -------------------------------------------------

void loop() {
  int i, j, k, r, n;
  int obstacle_ahead, at_edge;
  int tile_to_right[2];
  int move_to_next_layer;
  Task tasks_remaining[MAX_NUM_TASKS_REMAINING]; 
  Task closest_task;
  int closest_task_dist, curr_task_dist;
  
  Serial.println("Beginning search and rescue...");

//  // Code to test navigation without obstacles
//  for (i = 0; i < NUM_GRID_LAYERS; i++) {
//    n = arr_n_r[i][0];
//    r = arr_n_r[i][1];
//    for (j = 0; j < r; j++) {
//      // Going right
//      move_fwd(DIST_ONE_TILE);
//      delay(5000);
//    }
//    turn(CCW);
//    delay(5000);
//    for (j = 0; j < NUM_SIDES_MINUS_ONE; j++) {
//      for (k = 0; k < n; k++) {
//        move_fwd(DIST_ONE_TILE);
//        delay(5000);
//      }
//      turn(CCW);
//      delay(5000);
//    }
//    for (j = 0; j < n-r; j++) {
//      // Going right
//      move_fwd(DIST_ONE_TILE);
//      delay(5000);
//    }
//    turn(CCW);
//    delay(5000);
//    move_fwd(DIST_ONE_TILE);
//    delay(5000);
//    turn(CW);
//    delay(5000);
//  }

  // Actual main code
  // Explore terrain
  while(located_all_objectives() == 0){
    // First check if we are facing a wall. If yes then turn.
    if (curr_dir == NORTH && curr_pos[ROW_INDEX] == 0 ||
        curr_dir == WEST && curr_pos[COL_INDEX] == 0 ||
        curr_dir == SOUTH && curr_pos[ROW_INDEX] == 5 ||
        curr_dir == EAST && curr_pos[COL_INDEX] == 5){
      turn(CCW); 
    }
    obstacle_ahead = move_fwd(DIST_ONE_TILE);
    move_to_next_layer = all_tiles_in_layer_explored();
    if (obstacle_ahead == 1){
      turn(CCW);
      obstacle_ahead = 0;
    } else{
      // Check if we are at the edge
      at_edge = (curr_pos[ROW_INDEX] <= 0) ||
                (curr_pos[ROW_INDEX] >= 5) ||
                (curr_pos[COL_INDEX] <= 0) ||
                (curr_pos[COL_INDEX] >= 5);
      if (!at_edge){
        // Check if right tile is not explored
        Serial.print("Not at edge. Check if right tile is unexplored.");
        if(curr_dir = NORTH){
          tile_to_right[ROW_INDEX] = curr_pos[ROW_INDEX];
          tile_to_right[COL_INDEX] = curr_pos[COL_INDEX]+1;
        } else if(curr_dir == WEST){
          tile_to_right[ROW_INDEX] = curr_pos[ROW_INDEX]-1;
          tile_to_right[COL_INDEX] = curr_pos[COL_INDEX];
        } else if(curr_dir == SOUTH){
          tile_to_right[ROW_INDEX] = curr_pos[ROW_INDEX];
          tile_to_right[COL_INDEX] = curr_pos[COL_INDEX]-1;
        } else{
          tile_to_right[ROW_INDEX] = curr_pos[ROW_INDEX]+1;
          tile_to_right[COL_INDEX] = curr_pos[COL_INDEX];
        }
        if(terrain_map[tile_to_right[ROW_INDEX]][tile_to_right[COL_INDEX]] == 0){
          // Right tile exists and is not explored, turn CW
          Serial.print("Right tile exists and is unexplored.");
          turn(CW);
        }
      }
      // Not sure about this statement, test this...
      if (move_to_next_layer){
        turn(CCW);
      }
    }
  }

  // Complete remaining tasks
  while(tasks_all_done() == 0){
    // Assume fire has already been extinguished
    i = 0;
    if(tasks[FIND_FOOD].task_status == 0){
      // Have yet to find food
      tasks_remaining[i] = tasks[FIND_FOOD];
      i++;
    }
    if(tasks[FIND_LOST_PERSON].task_status == 0){
      // Have yet to save lost person
      tasks_remaining[i] = tasks[FIND_LOST_PERSON];
      i++;
    }
    if(tasks[FIND_FOOD].task_status == 1 && tasks[FEED_SURVIVORS].task_status == 0){
      // Already found food and have yet to feed survivors
      tasks_remaining[i] = tasks[FEED_SURVIVORS];
      i++;
    }

    // Determine which of the valid tasks is closest
    for(i = 0; i < MAX_NUM_TASKS_REMAINING; i++){
      curr_task_dist = calc_dist_from_curr_pos(tasks_remaining[i]);
      if (curr_task_dist < closest_task_dist){
        closest_task = tasks_remaining[i];
        closest_task_dist = curr_task_dist;
      }
    }

    // Set obstacles array accordingly
    // Don't set destination as obstacle, or else A* search will fail
    i = 0;
    for(j = 0; j < MAX_NUM_TASKS_REMAINING; j++){
      if (tasks_remaining[j].task_id != closest_task.task_id){
        obstacles[i][0] = tasks_remaining[j].obj_location[ROW_INDEX];
        obstacles[i][1] = tasks_remaining[j].obj_location[COL_INDEX];
        i++;
      }
    }

    // Determine shortest path to the object location
    find_shortest_path(curr_pos[ROW_INDEX], curr_pos[COL_INDEX], closest_task.obj_location[ROW_INDEX], closest_task.obj_location[COL_INDEX]);

    // Go to the object location
    go_through_shortest_path();
  }

  // Go back to start
  find_shortest_path(curr_pos[ROW_INDEX], curr_pos[COL_INDEX], START_ROW, START_COL);
  go_through_shortest_path();

  Serial.println("Finished search and rescue. PEACE OUT.");

  while(1){}
}
