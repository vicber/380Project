#include "SR04.h"

//Motors
#define ENABLE_M1 3
#define DIR_A_M1 4
#define DIR_B_M1 5

#define ENABLE_M2 11
#define DIR_A_M2 12
#define DIR_B_M2 13

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

//Flame
#define FLAME 8

//Encoder
#define outputA 6
#define outputB 7
int encoderCount = 0; 
const int numTicksBtwnTiles = 50; //number of encoder ticks from center of one tile to another
int aState;
int aLastState; 

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
  
  //Motors
  pinMode(ENABLE_M1, OUTPUT);
  pinMode(DIR_A_M1, OUTPUT);
  pinMode(DIR_B_M1, OUTPUT);

  pinMode(ENABLE_M2, OUTPUT);
  pinMode(DIR_A_M2, OUTPUT);
  pinMode(DIR_B_M2, OUTPUT);

  //Colour Sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,HIGH);

  //Encoder
  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  aLastState = digitalRead(outputA); // Reads the initial state of the outputA
}

bool ReachWall(){
  return true;
}

void Move_Backward() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, HIGH);  
}

void Move_Forward() {
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

void BackupOneTile() {
  Serial.println("Backing up one tile");
  encoderCount = 0;
  Move_Backward();
  while(encoderCount < numTicksBtwnTiles) {
    aState = digitalRead(outputA); // Reads the "current" state of the outputA
    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (aState != aLastState){     
      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
      if (digitalRead(outputB) != aState) { 
        encoderCount --;
      } else {
        encoderCount ++;
      }
      Serial.print("Position: ");
      Serial.println(encoderCount);
    } 
    aLastState = aState; // Updates the previous state of the outputA with the current state
  }
  Stop_Motors();
}

void Turn_CW() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, LOW);
  //TODO: adjust this to use magnetometer
  delay(2000); 
  Stop_Motors();
}

void Turn_CCW() {
  digitalWrite(DIR_A_M1, HIGH);
  digitalWrite(DIR_A_M2, LOW);
  digitalWrite(DIR_B_M1, LOW);
  digitalWrite(DIR_B_M2, LOW);
  //TODO: adjust this to use magnetometer
  delay(2000); 
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

void Handle_Object() {
  ReadColour();

  // BRITT: Maybe add all these number values as global constants instead of hardcoded values?
  // BRITT: Once task is complete, set corresponding flag in task_status[] (i.e. task_status[COLLECT_FOOD] = 1)
  // BRITT: Also, these should be updating terrain_map[][] and task_location[][] accordingly.
  if(totalRGB > 150 && double(red+green)/totalRGB >= 0.70 && double(green) / totalRGB > 0.38) {
    Serial.println("Detect Yellow House");
  }
  else if(totalRGB > 150 && double(red+blue)/totalRGB >= 0.75 && double(blue) / totalRGB > 0.40){
    Serial.println("Detect Red House");
  }
  else if(digitalRead(FLAME)==HIGH) {
    Serial.println("Detect Lit Candle");
  }
  else if(DetectMagnet()) {
    Serial.println("Detect Food");
  }
  else {
    Serial.println("Nothing read");
  }
}

void ExploreTerrain() {
  /*
   * Explore terrain and update the terrain map
   * 1 is a normal tile
  */
  Serial.println("Explore Terrain Start Loop");

  while(!FoundEverything()) {
    Move_Forward();
    encoderCount = 0;
    ultrasonicDist = sr04.Distance();
  
    //keep moving forward until hit objective or go to new tile
    while(ultrasonicDist > 5 && encoderCount < numTicksBtwnTiles) {
      aState = digitalRead(outputA); // Reads the "current" state of the outputA
      // If the previous and the current state of the outputA are different, that means a Pulse has occured
      if (aState != aLastState){     
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (digitalRead(outputB) != aState) { 
          encoderCount ++;
        } else {
          encoderCount --;
        }
        Serial.print("Position: ");
        Serial.println(encoderCount);
      } 
      aLastState = aState; // Updates the previous state of the outputA with the current state
    }
    
    Stop_Motors();
  
    //Case if we just moved a tile
    if(encoderCount < numTicksBtwnTiles) {
      char dir = directions[curr_direction_index];
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
  
      //TODO: Change search direction if needed, i.e. if the outter layer has been searched now search the inner layer
    }
  
    //Case if we ran into something
    else if(ultrasonicDist < 5) {
      Serial.println("Ran into something");
      
      if(digitalRead(FLAME)==LOW && !(totalRGB > 150 && double(red+blue)/totalRGB >= 0.75 && double(blue) / totalRGB > 0.40)
      && !(totalRGB > 150 && double(red+green)/totalRGB >= 0.70 && double(green) / totalRGB > 0.38)) {
        //if a wall
        BackupOneTile();
        Turn_CCW();
      }
      else {
        //if an object
        Handle_Object();
        BackupOneTile();
        Turn_CCW();
      }
    }
  }
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

void loop() {
  // enable the motors
  digitalWrite(ENABLE_M1, HIGH);
  digitalWrite(ENABLE_M2, HIGH);

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

  CompleteRemainingTasks();
}
