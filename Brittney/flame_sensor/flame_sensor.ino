/*The flame sensor has to use analog instead of digital*/
#define FLAME_1 A7
#define TESTLED 13

#define TESTLED 13

int DISPLAY_TIME = 10;  // In milliseconds

/*
 * Setup Info
 * positive side of the sensor goes to the resistor
 * negative side of the sensor goes to the 5V
 * positive side of the sensor goes to the pinA7
 * free side of the resistor goes to ground
 */

int flameReading_1 = 0;
void setup() {
  Serial.begin(9600);
  
  pinMode(FLAME_1, INPUT);
  pinMode(TESTLED,OUTPUT);
}

void loop() {
  flameReading_1 = analogRead(FLAME_1);
//  Serial.println(flameReading_1);
  /*918~919 is the baseline when the flame sensor starts to detetect the flame
  * Threshold is 918
  */

  if(flameReading_1==0){
    digitalWrite(TESTLED,LOW);
    Serial.println("No flame...");
  }
  else {
    digitalWrite(TESTLED,HIGH);
    Serial.println("FLAME");
  }
}
