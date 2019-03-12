// A * SEARCH ALGORITHM
/*
 * 
 */

// ALREADY DEFINED IN MAIN ---------------------------------------------

#define ROWS  6
#define COLS  6

// ---------------------------------------------------------------------

struct Tile {
  double row_pos;
  double col_pos;
  double f;
  double g;
  double h;
  Tile* neighbours;
  int num_neighbours;
  Tile* previous;
};

// Variables needed for search algorithm -------------------------------

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

void deallocGrid(){
  int i;
  for(i = 0; i < ROWS; i++) {
    free(grid[i]);
  }
  free(grid);
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

void FindShortestPath(int start_row, int start_col, int end_row, int end_col) {
  // Initialization ----------------------------------------------------
  int i, j;

  // Initialize grid
  grid = (Tile**)malloc(sizeof(Tile*) * ROWS);
  for(i = 0; i < ROWS; i++){
    grid[i] = (Tile*)malloc(sizeof(Tile) * COLS);
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
      deallocGrid();
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
    size_shortest_path = 0;
    Tile* temp = current;
    while (temp->previous) {
      shortest_path[size_shortest_path]=temp;
      size_shortest_path++;
      temp = temp->previous;
    }

    // Output to screen
    Serial.print("[2J"); // Clear screen command
    for (i = 0; i < ROWS; i++){
      for (j = 0; j < COLS; j++){
        if(inShortestPath(grid[i][j])) {
          Serial.print("P ");
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
    delay(5000);
  }
  // No solution, blocked!
  deallocGrid();
  return;
  // -------------------------------------------------------------------
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  FindShortestPath(0,0,5,5);
}
