const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  calibrateSensors();

  Serial.println("Simple Line Following Robot Started");
}

void calibrateSensors() {
  Serial.println("Calibrating sensors... Place robot on the line and wait.");
  delay(2000); // Wait for placement
  for (int i = 0; i < 5; i++) {
    int minVal = 4095, maxVal = 0;
    for (int j = 0; j < 100; j++) {
      int val = analogRead(sensorPins[i]);
      if (val < minVal) minVal = val;
      if (val > maxVal) maxVal = val;
      delay(10);
    }
    threshold[i] = 1.5 * ((minVal + maxVal) * 0.5); // Set threshold as midpoint
    Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(threshold[i]);
  }
  Serial.println("Calibration complete.");
  Serial.println("Threshold: " + String(threshold[0]) + " " + String(threshold[1]) + " " + String(threshold[2]) + " " + String(threshold[3]) + " " + String(threshold[4]));

}

void loop() {
  // put your main code here, to run repeatedly:
  int position = calculatePosition();
  Serial.println(position);
  delay(500);
}

int calculatePosition() {
  long weightedSum = 0;
  long sum = 0;
  int sensorsOn = 0;

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Check if sensor is above threshold (on the line)
    if (sensorValues[i] < threshold[i]) {
      sensorsOn++;
      Serial.println("Sensor " + String(i) + " has detected black.");
    }
    // Use analog values for smoother position calculation
    int weight = map(sensorValues[i], threshold[i], 4095, 0, 1000); // Scale to 0-1000
    
    if (weight > 100) { // Only count significant detections
      weightedSum += (long)i * weight;
      sum += weight;
    }
  }

  if (sum == 0) {
    return 2000;  
  }

  return (weightedSum * 1000) / sum; // Scale to 0-4000 range
}
