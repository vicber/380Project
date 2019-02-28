#define FLAME 8
#define LED 10

/*
 * Setup Info
 * positive side of the sensor goes to the resistor
 * negative side of the sensor goes to the 5V
 * positive side of the sensor goes to the pin8
 * free side of the resistor goes to ground
 */
 
void setup() {
  Serial.begin(9600);
  
  pinMode(FLAME, INPUT);
  pinMode(LED, OUTPUT);
}

void loop() {

  if(digitalRead(FLAME)==LOW){
    digitalWrite(LED, LOW); 
  }
  else if(digitalRead(FLAME)==HIGH){
    digitalWrite(LED, HIGH); 
  }
}
