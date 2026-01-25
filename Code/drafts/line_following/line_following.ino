const int ain1Pin = 13;    // Left motor direction 1
const int ain2Pin = 12;     // Left motor direction 2
const int pwmA = 25;       // Left motor PWM
const int LeftC1 = 16;
const int LeftC2 = 17;
const int RightC1 = 23;
const int RightC2 = 22;
const int standByPin = 19; // Motor driver standby
const int bin1Pin = 14;    // Right motor direction 1
const int bin2Pin = 27;    // Right motor direction 2
const int pwmB = 26;       // Right motor PWM

// --- IR Sensor Pins (TCRT5000 5-channel) ---
const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];

// --- PWM Configuration for ESP32-S3 ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 8;        // 8-bit (0-255)

// PID constants (tuned for faster response)
float Kp = 0.2;  // Proportional gain
float Ki = 0.0; // Integral gain
float Kd = 0.23;  // Derivative gain

long P, I, D, previousError = 0;
int error = 0;
int speed = 150; // Base speed (increased for faster response)
const float motorBalance = 1.0; // Adjust if one motor is stronger (e.g., 0.95 for weaker right motor)

// --- Setup ---
void setup() {
  Serial.begin(115200);
  
  // Motor pins
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(standByPin, OUTPUT);
  pinMode(LeftC1, INPUT);
  pinMode(LeftC2, INPUT);
  pinMode(RightC1, INPUT);
  pinMode(RightC2, INPUT);
  digitalWrite(standByPin, HIGH); // Enable driver

  // PWM setup
  

  // Sensor pins
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Calibrate sensor thresholds
  calibrateSensors();

  Serial.println("Simple Line Following Robot Started");
  
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);
}

// --- Calibrate sensor thresholds ---
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
    threshold[i] = (minVal + maxVal) * 0.5; // Set threshold as midpoint
    Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.println(threshold[i]);
  }
  Serial.println("Calibration complete.");
}
// --- Main loop ---
void loop() {
  int position = calculatePosition();
  // Only calculate error and follow line if not in stop condition
  Serial.print("Left motor1: ");
  Serial.print(digitalRead(LeftC1));
  Serial.print("  Left motor2: ");
  Serial.print(digitalRead(LeftC2));

  if (position != -1) { // -1 indicates all sensors are on, stop condition
    error = 2000 - position;  // 2000 is center
    PID_Linefollow(error);

    // Debugging output
    Serial.print("Pos: "); Serial.print(position);
    Serial.print(" Err: "); Serial.print(error);
    Serial.print(" L: "); Serial.print(speed - (Kp * P + Ki * I + Kd * D));
    Serial.print(" R: "); Serial.print(speed + (Kp * P + Ki * I + Kd * D));
    Serial.println();
    
  } else {
    Serial.println("All sensors on line: Stopping motors");
  }
}

// --- Calculate line position ---
int calculatePosition() {
  long weightedSum = 0;
  long sum = 0;
  int sensorsOn = 0;

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Check if sensor is above threshold (on the line)
    if (sensorValues[i] < threshold[i]) {
      sensorsOn++;
    }
    // Use analog values for smoother position calculation
    int weight = map(sensorValues[i], threshold[i], 4095, 0, 1000); // Scale to 0-1000
    if (weight > 100) { // Only count significant detections
      weightedSum += (long)i * weight;
      sum += weight;
    }
  }

  // Check if all sensors are on the line (stop condition)
  if (sensorsOn == 5) {
    motor_drive(0, 0); // Stop motors
    return -1; // Indicate stop condition
  }

  // Original condition for no significant detection
  if (sum == 0) { // Changed from sum == 5000 to handle no detection case
    motor_drive(0, 0); // Stop motors
    return 2000; // Assume center
  }
  
  return (weightedSum * 1000) / sum; // Scale to 0-4000 range
}
// --- PID Control ---
void PID_Linefollow(int error) {
  P = error;
  I += error;
  I = constrain(I, -1000, 1000); // Prevent integral windup
  D = error - previousError;

  float PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;

  int lsp = speed - PIDvalue;
  int rsp = (speed + PIDvalue) * motorBalance; // Apply motor balance

  lsp = constrain(lsp, -180, 180); // Allow reverse for sharper corrections
  rsp = constrain(rsp, -180, 180);

  motor_drive(lsp, rsp);
}

// --- Motor drive ---
void motor_drive(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed <= 0) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
  } else {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
  }
  ledcWrite(pwmA, abs(leftSpeed));

  // Right motor
  if (rightSpeed <= 0) {
    digitalWrite(bin1Pin, HIGH);
    digitalWrite(bin2Pin, LOW);
  } else {
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, HIGH);
  }
  ledcWrite(pwmB, abs(rightSpeed));
}