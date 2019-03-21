/*
// TCS230 color recognition sensor 
// Sensor connection pins to Arduino are shown in comments

Color Sensor      Arduino
-----------      --------
 VCC               5V
 GND               GND
 s0                4
 s1                5
 s2                6
 s3                7
 OUT               8
 OE                GND
*/

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int red = 0;
int blue = 0;
int green = 0;
int total = 0;

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);

  Serial.begin(9600);
}
void loop() {
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  red = pulseIn(sensorOut, LOW);
  // red = map(red, 230, 640, 255, 0);
  // Printing the value on the serial monitor
//  Serial.print("R= ");//printing name
  Serial.print(red);//printing RED color frequency
  Serial.print("\t");
  delay(100);
  
  // Setting green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  green = pulseIn(sensorOut, LOW);
  // green = map(green, 430, 1090, 255, 0);
  // Printing the value on the serial monitor
//  Serial.print("G= ");//printing name
  Serial.print(green);//printing RED color frequency
  Serial.print("\t");
  delay(100);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  blue = pulseIn(sensorOut, LOW);
  // blue = map(blue, 200, 1132, 255, 0);
  // Printing the value on the serial monitor
//  Serial.print("B= ");//printing name
  Serial.print(blue);//printing RED color frequency
  Serial.print("\t");

  Serial.println(millis()/1000.0);

  /*
  total = red + blue + green;
  if(total > 150 && double(red+green)/total >= 0.70 && double(green) / total > 0.38) {
    Serial.print("Detect Yellow House");
  }
  else if(total > 150 && double(red+blue)/total >= 0.75 && double(blue) / total > 0.40){
    Serial.print("Detect Red House");
  }
  else {
    Serial.print("Nothing read");
  }
  Serial.println("  ");*/
  
  delay(100);
}
