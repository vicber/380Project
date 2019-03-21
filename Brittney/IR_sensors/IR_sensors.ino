#define IR_PIN  A2

#define MIN_DIFF_FILTERED_DATA 3.0

#define CUTOFF_FREQ_HZ  2.5
double curr_filtered_data, prev_filtered_data;
double RC = 1.0/(2*M_PI*CUTOFF_FREQ_HZ);
double alpha;

// Time data
int t1, t2;
double dt;

int sensorValue = 0;   // The sensor value
int sensorMin = 1023;  // Minimum sensor value
int sensorMax = 0;     // Maximum sensor value
int scaledMax = 500;

void setup() {
  Serial.begin(9600);
  // Serial.println("Calibration begin...");

  // Calibrate during the first five seconds 
  while (millis() < 10000) {
    sensorValue = analogRead(IR_PIN);

    // Record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // Record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

  // Initialize filter data
  sensorValue = analogRead(IR_PIN);
  t1 = millis();
  curr_filtered_data = sensorValue;
  prev_filtered_data = curr_filtered_data;
  
  // Serial.println("Calibration end...");
}

void loop() {
  double temp;
  // Read the sensor
  sensorValue = analogRead(IR_PIN);
  t2 = millis();

  // Apply the calibration to the sensor reading
  sensorValue = map(sensorValue, sensorMin, sensorMax, 0, scaledMax);

  // In case the sensor value is outside the range seen during calibration
  sensorValue = constrain(sensorValue, 0, scaledMax);

  // Apply LP filter
  dt = (t2-t1)/1000.0; // Convert to seconds
  alpha = dt/(RC+dt);
  temp = prev_filtered_data + (alpha * (sensorValue - prev_filtered_data));
  if (abs(temp-prev_filtered_data) > MIN_DIFF_FILTERED_DATA) {
    curr_filtered_data = temp;
  } else {
    curr_filtered_data = prev_filtered_data;
  }
  t1 = t2;
  prev_filtered_data = curr_filtered_data;

  // Output value to monitor
  Serial.println(curr_filtered_data);
}
