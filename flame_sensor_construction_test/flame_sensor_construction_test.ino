/*The flame sensor has to use analog instead of digital*/
#define FLAME A7
#define LED 10

/*
 * Setup Info
 * positive side of the sensor goes to the resistor
 * negative side of the sensor goes to the 5V
 * positive side of the sensor goes to the pin8
 * free side of the resistor goes to ground
 */

int flameReading = 0;
void setup() {
  Serial.begin(9600);
  
  pinMode(FLAME, INPUT);
  pinMode(LED, OUTPUT);
}

void loop() {
  flameReading = analogRead(FLAME);
/*
  if(digitalRead(FLAME)==LOW){
    digitalWrite(LED, LOW); 
  }
  else if(digitalRead(FLAME)==HIGH){
    digitalWrite(LED, HIGH); 
  }*/

  Serial.println(flameReading);
  delay(20);
}
