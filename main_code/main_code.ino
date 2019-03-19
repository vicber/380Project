#include "SR04.h"

//Motors
#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13
const int min_fwd_speed = 220;
const int min_turn_speed = 210;
int speed;

#define TRIG_PIN 7
#define ECHO_PIN 6

//Colour Sensor Stuff
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8
const int loRed = 146;
const int hiRed = 60;
const int loBlue = 59; 
const int hiBlue = 378;
const int loGreen = 277;
const int hiGreen = 121;
const int minRGB = 150; //used to ensure that there is enough RGB value to sensibly use percentage method
const double yellowHouse_RG = 0.70; //percent of RG over RGB
const double yellowHouse_G = 0.38;  //percent of G over RGB
const double redHouse_RB = 0.75; //percent of RB over RGB
const double redHouse_B = 0.40;  //percent of B over RGB

//Flame
#define FLAME 8

//Encoder
#define MOTOR_ENC_PIN_A   22 // DIGITAL
#define MOTOR_ENC_PIN_B   24 // DIGITAL
int last_enc_val_A, last_enc_val_B, enc_val_A, enc_val_B;
int encoder_count;

#define ROWS 6
#define COLS 6

//Hall Effect Sensor
const int hallPin = 12;     // the number of the hall effect sensor pin
const int ledPin =  13;     // the number of the LED pin
int hallState = 0;          // variable for reading the hall sensor status

// Vin 3.3V 
int red = 0;
int blue = 0;
int green = 0;
int totalRGB = 0;

//Ultrasonic Sensor
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long ultrasonicDist;

char terrain_map[6][6];
char directions[] = {'N', 'W', 'S', 'E'};
int curr_direction_index = 0; //start facing north
int curr_row = 5;
int curr_col = 2;

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

void setup() {
  Serial.begin(9600);

  //Flame
  pinMode(FLAME, INPUT);
  
  //Hall effect
  pinMode(hallPin, INPUT);
  
  //Encoders
  pinMode(MOTOR_ENC_PIN_A, INPUT);
  pinMode(MOTOR_ENC_PIN_B, INPUT);
  last_enc_val_A = digitalRead(MOTOR_ENC_PIN_A);
  last_enc_val_B = digitalRead(MOTOR_ENC_PIN_B);
  encoder_count = 0;

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
}

bool ReachWall(){
  return true;
}

void Move_Forward() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);
}

void Move_Backward() {
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

void EncoderLoop(){
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
        encoder_count++;
      } else {
        // Going CCW, decrement the count
        encoder_count--;
      }
    } else if (diff_enc_val_A == -1) {
      // Pin A value switched from high to low
      if (enc_val_B == 0) {
        // Going CW, increment the count
        encoder_count++;
      } else {
        // Going CCW, decrement the count
        encoder_count--;
      }
    } else if (diff_enc_val_B == 1) {
      // Pin B value switched from low to high
      if (enc_val_A == 0) {
        // Going CW, increment the count
        encoder_count++;
      } else {
        // Going CCW, decrement the count
        encoder_count--;
      }
    } else {
      // Pin B value switched from high to low
      if (enc_val_A == 1) {
        // Going CW, increment the count
        encoder_count++;
      } else {
        // Going CCW, decrement the count
        encoder_count--;
      }
    }
  }
  Serial.print("Encoder: ");
  Serial.println(encoder_count);

  last_enc_val_A = enc_val_A;
  last_enc_val_B = enc_val_B;
}

void BackupOneTile() {
  Serial.println("Backing up one tile");
  encoder_count = 0;
  Move_Backward();
  while(encoder_count < numTicksBtwnTiles) {
      EncoderLoop();
  }
  Stop_Motors();
}

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
  Serial.println("Turn_CW");
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
  Serial.println("Turn_CCW");
  delay(5000); 
  Stop_Motors();
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
  //TODO: Hall Effect code is a toggle, so check if the value toggled from before
  hallState = digitalRead(hallPin);
  if (hallState == LOW) {     
    return true;
  } 
  else {
    return false;
  }
}

void ReadColour() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = pulseIn(sensorOut, LOW);
  red = map(red, hiRed, loRed, 255, 0);
  delay(100);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  green = pulseIn(sensorOut, LOW);
  green = map(green, hiGreen, loGreen, 255, 0);
  delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blue = pulseIn(sensorOut, LOW);
  blue = map(blue, hiBlue, loBlue, 255, 0);

  totalRGB = red + blue + green;
}

bool Detect_Yellow_House() {
  ReadColour();
  if(totalRGB > minRGB && double(red+green)/totalRGB >= yellowHouse_RG && double(green) / totalRGB > yellowHouse_G) {
    return true;
  }
  else {
    return false;
  }
}

bool Detect_Red_House() {
  ReadColour();
  if(totalRGB > minRGB && double(red+blue)/totalRGB >= redHouse_RB && double(blue) / totalRGB > redHouse_B) {
    return true;
  }
  else {
    return false;
  }  
}

void Put_Out_Fire() {
  Move_Forward();
  while(digitalRead(FLAME)==HIGH) {}
  delay(200); //go a bit more in
  Stop_Motors();
}

void Handle_Object() {
  ReadColour();
  Serial.println("Handle_Object");
  if(Detect_Yellow_House()) {
    Serial.println("Detect Yellow House: Found lost person");
    foundPerson = true;
    task_status[FIND_LOST_PERSON] = 1;
    terrain_map[curr_row][curr_col] = 'P';
    task_location[FIND_LOST_PERSON][0] = curr_row;
    task_location[FIND_LOST_PERSON][1] = curr_col;
  }
  else if(Detect_Red_House()){
    Serial.println("Detect Red House: Group of Survivors");
    foundGroup = true;
    terrain_map[curr_row][curr_col] = 'G';
    task_location[FEED_SURVIVORS][0] = curr_row;
    task_location[FEED_SURVIVORS][1] = curr_col;
    if(task_status[COLLECT_FOOD]) {
      task_status[FEED_SURVIVORS] = 1;
    }
    else {
      Serial.println("Need to collect food before feeding survivors.");
    }
  }
  else if(digitalRead(FLAME)==HIGH) {
    Serial.println("Detect Lit Candle");
    foundCandle = true;
    terrain_map[curr_row][curr_col] = 'C';
    task_location[FIRE_OFF][0] = curr_row;
    task_location[FIRE_OFF][1] = curr_col;
    Put_Out_Fire();
    task_status[FIRE_OFF] = 1;
  }
  else if(DetectMagnet()) {
    Serial.println("Detect Food");
    terrain_map[curr_row][curr_col] = 'F';
    task_location[COLLECT_FOOD][0] = curr_row;
    task_location[COLLECT_FOOD][1] = curr_col;
    foundFood = true;
    task_status[COLLECT_FOOD] = 1;
  }
  else {
    Serial.println("Nothing read");
    terrain_map[curr_row][curr_col] = '1';
  }
}

void Update_Position(bool forward) {
  char dir = directions[curr_direction_index];
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

void ExploreTerrain() {
  /*
   * Explore terrain and update the terrain map
   * This code assumes you are already at an edge
   * '1' is a normal tile
  */
  
  Serial.println("Explore Terrain Start Loop");
  int curr_search_layer  = 0; //0 is the outer layer, 2 is the most inner layer
  while(!FoundEverything()) {
    Move_Forward();
    encoder_count = 0;
    ultrasonicDist = sr04.Distance();
  
    //keep moving forward until hit objective or go to new tile
    while(ultrasonicDist > 5 && encoder_count < numTicksBtwnTiles) {
      EncoderLoop();
    }
    
    Stop_Motors();
  
    //Case if we just moved a tile
    if(encoder_count < numTicksBtwnTiles) {
      Update_Position(true);
  
      //Detect food?
      if(DetectMagnet()) {
        //TODO: Handle Food Object code, indicate that food was detected
        terrain_map[curr_row][curr_col] = 'F';
        foundFood = true;
      }
      else {
        terrain_map[curr_row][curr_col] = '1';  
      }
  
      //Do we need to handle object to the right?
      if(curr_col < 5 && terrain_map[curr_row][curr_col + 1] == '0') {
        Turn_CW();
        curr_direction_index += 3; //Set direction index to CW
        Stop_Motors();
      }
  
      //Change search direction if needed, i.e. if the outer layer has been searched now search the inner layer
      if(Layer_Searched(curr_search_layer)) {
        curr_search_layer++;
        Turn_CCW();
      }
    }
  
    //Case if we ran into something
    else if(ultrasonicDist < 5) {
      Serial.println("Ran into something");
      
      if(digitalRead(FLAME)==LOW && !(totalRGB > 150 && double(red+blue)/totalRGB >= 0.75 && double(blue) / totalRGB > 0.40)
      && !(totalRGB > 150 && double(red+green)/totalRGB >= 0.70 && double(green) / totalRGB > 0.38)) {
        //if a wall
        //TODO: Instead of backing up one tile, back up the amount we went forward, for better accuracy
        BackupOneTile();
        
        Turn_CCW();
      }
      else {
        //if an object
        Update_Position(true);
        Handle_Object();        
        BackupOneTile();
        Update_Position(false); //revert the prior update position
        Turn_CCW();
      }

      //Change search direction if needed, i.e. if the outer layer has been searched now search the inner layer
      if(Layer_Searched(curr_search_layer)) {
        curr_search_layer++;
        Turn_CCW();
      }
      
    }
  }
}

void loop() {
  // enable the motors
  analogWrite(ENABLE_M1, speed); // From 0 - 255?
  analogWrite(ENABLE_M2, speed); // From 0 - 255?

  // initialize map to be unknown/unvisited
  for(int i = 0; i < 6; ++i) {
    for(int j = 0; j < 6; ++j) {
      terrain_map[i][j] = '0';
    }
  }

  //Record starting position
  terrain_map[curr_row][curr_col] = '1';
  
  //Locate all of the objectives within the grid
  ExploreTerrain();

  //CompleteRemainingTasks();
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
