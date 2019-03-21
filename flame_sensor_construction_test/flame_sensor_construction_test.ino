/*The flame sensor has to use analog instead of digital*/
#define FLAME_1 A7
// #define FLAME_2 A8
#define TESTLED 13

#define RED_PIN 7
#define GREEN_PIN 8
#define BLUE_PIN 9
#define TESTLED 13

int DISPLAY_TIME = 10;  // In milliseconds

/*
 * Setup Info
 * positive side of the sensor goes to the resistor
 * negative side of the sensor goes to the 5V
 * positive side of the sensor goes to the pinA7
 * free side of the resistor goes to ground
 */

int flameReading_1 = 0, flameReading_2 = 0;
void setup() {
  Serial.begin(9600);
  
  pinMode(FLAME_1, INPUT);
  pinMode(FLAME_2, INPUT);
  pinMode(TESTLED,OUTPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  //mainColors();

  //showSpectrum();

  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(TESTLED,LOW);
}

void loop() {
  flameReading_1 = analogRead(FLAME_1);
  flameReading_2 = analogRead(FLAME_2);
  Serial.print("sensor1 ");
  Serial.print(flameReading_1);
  Serial.print("\tsensor2 ");
  Serial.println(flameReading_2);
  /*918~919 is the baseline when the flame sensor starts to detetect the flame
  * Threshold is 918
  */

  if(flameReading_1==0 && flameReading_2 == 0){
    digitalWrite(TESTLED,LOW);
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
  }
  else {
    //mainColors();
    //showSpectrum();
    digitalWrite(TESTLED,HIGH);
  }
}

void mainColors()
{
// Off (all LEDs off):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Red (turn just the red LED on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

/*
// Green (turn just the green LED on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Blue (turn just the blue LED on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// Yellow (turn red and green on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, LOW);

delay(1000);

// Cyan (turn green and blue on):

digitalWrite(RED_PIN, LOW);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// Purple (turn red and blue on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, LOW);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);

// White (turn all the LEDs on):

digitalWrite(RED_PIN, HIGH);
digitalWrite(GREEN_PIN, HIGH);
digitalWrite(BLUE_PIN, HIGH);

delay(1000);*/
}

void showSpectrum()
{
int x;  // define an integer variable called "x"

// Now we'll use a for() loop to make x count from 0 to 767
// (Note that there's no semicolon after this line!
// That's because the for() loop will repeat the next
// "statement", which in this case is everything within
// the following brackets {} )

for (x = 0; x < 768; x++)

// Each time we loop (with a new value of x), do the following:

{
showRGB(x);  // Call RGBspectrum() with our new x
delay(DISPLAY_TIME);   // Delay for 10 ms (1/100th of a second)
}
}


// showRGB()

// This function translates a number between 0 and 767 into a
// specific color on the RGB LED. If you have this number count
// through the whole range (0 to 767), the LED will smoothly
// change color through the entire spectrum.

// The "base" numbers are:
// 0   = pure red
// 255 = pure green
// 511 = pure blue
// 767 = pure red (again)

// Numbers between the above colors will create blends. For
// example, 640 is midway between 512 (pure blue) and 767
// (pure red). It will give you a 50/50 mix of blue and red,
// resulting in purple.

// If you count up from 0 to 767 and pass that number to this
// function, the LED will smoothly fade between all the colors.
// (Because it starts and ends on pure red, you can start over
// at 0 without any break in the spectrum).


void showRGB(int color)
{
int redIntensity;
int greenIntensity;
int blueIntensity;

// Here we'll use an "if / else" statement to determine which
// of the three (R,G,B) zones x falls into. Each of these zones
// spans 255 because analogWrite() wants a number from 0 to 255.

// In each of these zones, we'll calculate the brightness
// for each of the red, green, and blue LEDs within the RGB LED.

if (color <= 255)          // zone 1
{
redIntensity = 255 - color;    // red goes from on to off
greenIntensity = color;        // green goes from off to on
blueIntensity = 0;             // blue is always off
}
else if (color <= 511)     // zone 2
{
redIntensity = 0;                     // red is always off
greenIntensity = 255 - (color - 256); // green on to off
blueIntensity = (color - 256);        // blue off to on
}
else // color >= 512       // zone 3
{
redIntensity = (color - 512);         // red off to on
greenIntensity = 0;                   // green is always off
blueIntensity = 255 - (color - 512);  // blue on to off
}

// Now that the brightness values have been set, command the LED
// to those values

analogWrite(RED_PIN, redIntensity);
analogWrite(BLUE_PIN, blueIntensity);
analogWrite(GREEN_PIN, greenIntensity);
}
