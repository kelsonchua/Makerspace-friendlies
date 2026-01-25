const int ain1Pin = 13;     // Left motor cw direction
const int ain2Pin = 12;     // Left motor acw direction
const int pwmA = 25;        // Left motor PWM
const int LeftC1 = 16;      // Left motor encoder output A
const int LeftC2 = 17;      // Left motor encoder output B
const int RightC1 = 23;     // Right motor encoder output A
const int RightC2 = 22;     // Right motor encoder output B
const int standByPin = 19;  // Motor driver standby
const int bin1Pin = 14;     // Right motor cw direction
const int bin2Pin = 27;     // Right motor acw direction
const int pwmB = 26;        // Right motor PWM

// --- IR Sensor Pins (TCRT5000 5-channel) ---
const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];

// --- PWM Configuration for ESP32-S3 ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 8;        // 8-bit (0-255)

// PID constants (To be tuned)
float Kp = 0.2;    // Proportional gain
float Ki = 0.0;    // Integral gain
float Kd = 0.23;   // Derivative gain

long P, I, D, previousError = 0;
int error = 0;
int lfspeed = 140;  // Base speed (increased for faster response)
const float motorBalance = 1.00; // Adjust if one motor is stronger (e.g., 0.95 for weaker right motor)

// For time calculation
unsigned long timer = 0;

// To recognise correct junction
const long REQUIRED_JUNCTION_DURATION = 30; // To be tuned

// For time calculation
const long REQUIRED_END_GOAL_DURATION = 100; // To be tuned

// --- State Machine Definition ---
enum RobotState {
    LINE_FOLLOW,
    MAZE_ALIGN_TURN,  // State to execute the 90-degree turn
    MAZE_STRAIGHT,    // State to go to goal
    MAZE_GOAL 
};
RobotState currentState = LINE_FOLLOW;

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
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  // Sensor pins
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Calibrate sensor thresholds
  calibrateSensors();

  Serial.println("Simple Line Following Robot Started");
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
  // The switch statement handles the robot's current mode of operation.
  switch (currentState) {
    case LINE_FOLLOW:
    {
      int position = calculatePosition();

      // When out of track, stop  
      if (position == -1) {
        motor_drive(0, 0);
        Serial.println("Out of track");
      }
      
      // Check for junction
      if (position == -2) { 
        // motor_drive(0, 0) is already called in calculatePosition, but we ensure it here
        if (timer == 0) {
          timer = millis();
          // Optional: Print this only once
          // Serial.println("Potential junction detected. Starting timer...");
        }

        // Check if the required duration has passed (CONFIRMATION)
        if (millis() - timer >= REQUIRED_JUNCTION_DURATION) {
          motor_drive(0, 0); 
          currentState = MAZE_ALIGN_TURN;
          timer = 0; // Reset timer
          Serial.println("CONFIRMED: Wide junction. Entering MAZE_ALIGN_TURN.");
          break; 
        }
      
        // If not confirmed yet, keep motors stopped to prevent moving past the turn point
        motor_drive(0, 0); 
      }

      else{
        // Line Following
        error = 2000 - position; 
        PID_Linefollow(error);
        // Optional Debugging (uncomment if needed)
        /*
        Serial.print("Pos: "); Serial.print(position);
        Serial.print(" Err: "); Serial.print(error);
        Serial.print(" L_Spd: "); Serial.print(lfspeed - (Kp * P + Ki * I + Kd * D));
        Serial.print(" R_Spd: "); Serial.print(lfspeed + (Kp * P + Ki * I + Kd * D));
        Serial.println();
        */
      }
    }
    break;

    // Turn right 90 degree
    case MAZE_ALIGN_TURN:
    {
      int turnSpeed = 100;
      
      // Move forward a few more before turn
      motor_drive(turnSpeed, turnSpeed); 
      delay(100); // Move forward for this long
      
      // Turn right
      motor_drive(turnSpeed, -turnSpeed); 
      delay(300); // Turn right for this long
      
      // Enter next state
      motor_drive(0, 0); 
      currentState = MAZE_STRAIGHT; 
    }
    break;

    // Continue Line Following until goal
    case MAZE_STRAIGHT:
    {
      int position = calculatePosition();
      
      // Check if already reach goal 
      if (position == -1) {            
        if (timer == 0) {   // Start timer
          timer = millis();
          // Optional: Print this only once
          // Serial.println("Potential junction detected. Starting timer...");
        }
        // Check if the required duration has passed (CONFIRMATION)
        if (millis() - timer >= REQUIRED_END_GOAL_DURATION) {
          currentState = MAZE_GOAL;
          timer = 0; // Reset timer
          break; 
        }
        Serial.println("Out of track");
      }

      // Junction detection
      else if (position == -2) {
        if (timer == 0) {   // Start timer
          timer = millis();
          // Optional: Print this only once
          // Serial.println("Potential junction detected. Starting timer...");
        }
        // Check if the required duration has passed (CONFIRMATION)
        if (millis() - timer >= REQUIRED_JUNCTION_DURATION) {
          motor_drive(100, 100); // Drive forward
          delay(100); // Drive for this long
          timer = 0; // Reset timer
          break; 
        }
      }

      else{
        // Line Following
        timer = 0; // Reset timer after wrong detection
        error = 2000 - position; 
        PID_Linefollow(error);
        // Optional Debugging (uncomment if needed)
        /*
        Serial.print("Pos: "); Serial.print(position);
        Serial.print(" Err: "); Serial.print(error);
        Serial.print(" L_Spd: "); Serial.print(lfspeed - (Kp * P + Ki * I + Kd * D));
        Serial.print(" R_Spd: "); Serial.print(lfspeed + (Kp * P + Ki * I + Kd * D));
        Serial.println();
        */
      }
    }
    break;

    // Goal reached
    case MAZE_GOAL:
    {
      motor_drive(100, 100); // Drive forward to enter the goal box
      delay(300); // Drive for this long
      while (true) {
        motor_drive(0,0); // Stop 
      }
    }
    
    // Stop if error occur
    default:
      motor_drive(0, 0);
      Serial.println("ERROR: Unknown state! Halting.");
      break;
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

  Serial.println(sum);

  // Check if car is out of track
  if (sensorsOn == 5) {
    motor_drive(0, 0); // Stop motors
    return -1; // Indicate stop condition
  }

  // Original condition for no significant detection
  if (sum == 0) { // Changed from sum == 5000 to handle no detection case
    motor_drive(0, 0); // Stop motors
    return 2000; // Assume center
  }

  // Junction detected (Might be false positive)
  if (sum == 4000){
    Serial.println("Turn right state");
    return -2;
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

  int lsp = lfspeed - PIDvalue;
  int rsp = (lfspeed + PIDvalue) * motorBalance; // Apply motor balance

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