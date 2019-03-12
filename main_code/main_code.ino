#include "SR04.h"
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

//Flame & Thermal Sensor
#define FLAME 8

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

long encoderDist = 0;

//Ultrasonic Sensor
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;

char terrain_map[6][6];
char directions[] = {'N', 'W', 'S', 'E'};
int curr_direction_index = 0; //start facing north
int curr_row = 5;
int curr_col = 2;

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

  //Flame and Thermal
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
}

bool ReachWall(){
  return true;
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

void Turn_CW() {
  digitalWrite(DIR_A_M1, LOW);
  digitalWrite(DIR_A_M2, HIGH);
  digitalWrite(DIR_B_M1, HIGH);
  digitalWrite(DIR_B_M2, LOW);
  //TODO: adjust this to use magnetometer
  delay(2000); 
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
  // BRITT: Maybe add all these number values as global constants instead of hardcoded values?
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = pulseIn(sensorOut, LOW);
  red = map(red, 60, 146, 255, 0);
  delay(100);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  green = pulseIn(sensorOut, LOW);
  green = map(green, 121, 277, 255, 0);
  delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  blue = pulseIn(sensorOut, LOW);
  blue = map(blue, 59, 378, 255, 0);

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

void GetEncoderLength() {
  encoderDist = 5;
}

void ExploreTerrain() {
  /*
   * Explore terrain and update the terrain map
   * 1 is a normal tile
  */
  Move_Forward();
  GetEncoderLength();
  a=sr04.Distance();
  if(encoderDist >= 5) {
    Stop_Motors();
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

    if(DetectMagnet()) {
      terrain_map[curr_row][curr_col] = 'F';
      //Handle_Object
    }
    else {
      terrain_map[curr_row][curr_col] = '1';  
    }

    //Do we need to handle object to the right?
    if(curr_col < 5 && terrain_map[curr_row][curr_col + 1] == '0') {
      Turn_CW();
      curr_direction_index += 3;
      Stop_Motors();
    }

    //TODO: Change search direction if needed, i.e. if the outer layer has been searched now search the inner layer
  }
  else if(a < 5) {
    //if a wall
    if(digitalRead(FLAME)==LOW && !(totalRGB > 150 && double(red+blue)/totalRGB >= 0.75 && double(blue) / totalRGB > 0.40)
    && !(totalRGB > 150 && double(red+green)/totalRGB >= 0.70 && double(green) / totalRGB > 0.38)) {
      //wall
      //back up
      //turn CCW
    }
    else {
      Handle_Object();
      //back up
      //turn CCW
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
  ExploreTerrain();
  CompleteRemainingTasks();
}
