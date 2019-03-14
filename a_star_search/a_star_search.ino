// A * SEARCH ALGORITHM
/*
 * 
 */

#include <stdlib.h>
#include <Wire.h>

// ALREADY DEFINED IN MAIN ---------------------------------------------

#define ROWS  6
#define COLS  6

char directions[] = {'N', 'W', 'S', 'E'};
int curr_direction_index = 2; //start facing south

#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

// ---------------------------------------------------------------------

#define MAG_ADDR 0x0E
#define X 3
#define Y 7
#define Z 5

#define MAX_NEIGHBOURS  4
#define NUM_OBSTACLES   3

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

// Variables needed for search algorithm -------------------------------

Tile* grid[ROWS][COLS];

Tile* openSet[ROWS*COLS];
Tile* closedSet[ROWS*COLS];
int size_openSet;
int size_closedSet;
#define OPEN_SET_ID     0
#define CLOSED_SET_ID   1

Tile* start_tile;
Tile* end_tile;

Tile* shortest_path[ROWS*COLS];
int size_shortest_path;

// For each obstacle, track row and column
int obstacles[NUM_OBSTACLES][2];

// ---------------------------------------------------------------------

const int min_fwd_speed = 220;
const int min_turn_speed = 210;
int speed;

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

void deallocGrid(){
  int i, j;
  for(i = 0; i < ROWS; i++) {
    for(j = 0; j < COLS; j++) {
      free(grid[i][j]);
    }
  }
}

int inShortestPath(Tile* tile){
  int i;
  for (i = 0; i < size_shortest_path; i++) {
    if(tile == shortest_path[i]) {
      return 1;
    }
  }
  return 0;
}

void initObstacles(){
  int i, row, col;
  for(i = 0; i < NUM_OBSTACLES; i++) {
    row = obstacles[i][0];
    col = obstacles[i][1]; 
    grid[row][col]->is_obstacle = 1;
  }
  // CONSIDER: start and end should not be obstacles in order for the path to make sense and have a solution
}

void determineShortestPath(Tile* curr_best) {
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
  for (i = 0; i < ROWS; i++){
    for (j = 0; j < COLS; j++){
      if(grid[i][j]->is_obstacle) {
        Serial.print("X ");
      } else if(inShortestPath(grid[i][j])) {
        Serial.print("* ");
      } else if(tileInSet(OPEN_SET_ID, grid[i][j])) {
        Serial.print("O ");
      } else if(tileInSet(CLOSED_SET_ID, grid[i][j])) {
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

void FindShortestPath(int start_row, int start_col, int end_row, int end_col) {
  // Initialization ----------------------------------------------------
  int i, j;

  // Initialize grid
  for(i = 0; i < ROWS; i++){
    for(j = 0; j < COLS; j++) {
      grid[i][j]=(Tile*)malloc(sizeof(Tile));
    }
  }
  for(i = 0; i < ROWS; i++){
    for(j = 0; j < COLS; j++) {
      grid[i][j]->row_pos = i;
      grid[i][j]->col_pos = j;
      grid[i][j]->f = 0;
      grid[i][j]->g = 0;
      grid[i][j]->h = 0;
      addNeighbours(grid[i][j]);
      grid[i][j]->previous = NULL;
      grid[i][j]->is_obstacle = 0;
    }
  }
  
  initObstacles();

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
      determineShortestPath(curr_best);
      deallocGrid();
      Serial.println("DONE!!!");
      return;
    }
    removeFromSet(OPEN_SET_ID, curr_best_index);
    addToSet(CLOSED_SET_ID, curr_best);
    
    for(i = 0; i < curr_best->num_neighbours; i++) {
      Tile* neighbour = curr_best->neighbours[i];
      if(!tileInSet(CLOSED_SET_ID, neighbour) && !neighbour->is_obstacle) {
        double temp_g = curr_best->g + 1;
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
        neighbour->previous = curr_best;
      }
    }
  }
  // No solution, blocked!
  deallocGrid();
  Serial.println("No solution!!!");
  return;
  // -------------------------------------------------------------------
}

void StopMotors() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
}

void Move_Forward(int num_tiles) {
  speed = min_fwd_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed);

  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
  
  delay(1000 * num_tiles); // Temporary, might need to change time value

  StopMotors();
}

void Turn_CCW(int num_turns) {
  speed = min_turn_speed;
  analogWrite(ENABLE_M1, speed);
  analogWrite(ENABLE_M2, speed);

  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, HIGH);
  
  delay(1350 * num_turns); // Temporary, might need to change time value
  
  StopMotors();
}

void fixOrientation(Tile* current_tile, Tile* next_tile) {
  int i;
  int num_90_deg_turns = 0;
  int curr_row = current_tile->row_pos;
  int curr_col = current_tile->col_pos;
  int next_row = next_tile->row_pos;
  int next_col = next_tile->col_pos;
  int new_dir_index = 0; // 0: N, 1: W, 2: S, 3: E
  
  if (curr_row-next_row == 1) {
    // Need to go N
    new_dir_index = 0;
  } else if (curr_col-next_row == 1) {
    // Need to go W
    new_dir_index = 1;
  } else if (curr_row-next_row == -1) {
    //Need to go S
    new_dir_index = 2;
  } else {
    // Need to go E
    new_dir_index = 3;
  }
  
  if (new_dir_index - curr_direction_index == 0) {
    // No need to fix orientation, already in correct orientation!
    return;
  } else if (new_dir_index - curr_direction_index > 0) {
      num_90_deg_turns = new_dir_index - curr_direction_index;
  } else {
      num_90_deg_turns = (4 - curr_direction_index - new_dir_index);
  }

  Turn_CCW(num_90_deg_turns);
  curr_direction_index = new_dir_index;
  
  return;
}

void GoThroughShortestPath() {
  Tile* curr_tile_in_path = shortest_path[size_shortest_path-1];
  while(curr_tile_in_path != shortest_path[0]) { // May need to change end condition
    fixOrientation(curr_tile_in_path, curr_tile_in_path->previous);
    Move_Forward(1);
    curr_tile_in_path = curr_tile_in_path->previous;
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);
}

void loop() {
  // Just for testing...
  // Should be defined more efficiently before A* search algorithm runs
  obstacles[0][0] = 0;
  obstacles[0][1] = 2;
  obstacles[1][0] = 1;
  obstacles[1][1] = 2;
  obstacles[2][0] = 2;
  obstacles[2][1] = 4;
  
  FindShortestPath(0,0,5,5);
  // GoThroughShortestPath();
  while(1){}
}
