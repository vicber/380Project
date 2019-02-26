#define FLAME 8
#define LED 10

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
