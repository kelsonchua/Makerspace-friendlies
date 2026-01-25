#include <ESP32Encoder.h>

const int ain1Pin = 13;     // Left motor direction 1
const int ain2Pin = 18;     // Left motor direction 2
const int pwmA = 25;        // Left motor PWM
const int LeftC1 = 4;      // Left motor C1
const int LeftC2 = 5;      // Left motor C2
const int RightC1 = 23;     // Right motor C1
const int RightC2 = 22;     // Right motor C2
const int standByPin = 19;  // Motor driver standby
const int bin1Pin = 14;     // Right motor direction 1
const int bin2Pin = 27;     // Right motor direction 2
const int pwmB = 26;        // Right motor PWM
const int tx = 17;          // TX pin
const int rx = 16;          // RX pin

// --- IR Sensor Pins (TCRT5000 5-channel) ---
const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];
uint8_t sensorState = 0;

// --- PWM Configuration for ESP32-S3 ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 8;        // 8-bit (0-255)

// Motor
const float GEAR_RATIO = 100.0;           
const int ENCODER_PPR = 7;                // Pulses per rev
const float COUNTS_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO; // 2800
const float WHEEL_DIAMETER = 43.68;        // Actual wheel diameter in mm
const float CIRCUMFERENCE = PI * WHEEL_DIAMETER;

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// Distance from sensor to centre
const int CENTRE_DISTANCE = 50;   // in mm

// Maze
const int GRID_SIZE = 244;  // Grid size in mm

// PID constants (tuned for faster response)
float Kp = 0.2;   // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.23;  // Derivative gain

long P, I, D, previousError = 0;
int error = 0;
int ExpSpeed = 150; // Exploration speed (increased for faster response)
const float motorBalance = 1.0; // Adjust if one motor is stronger (e.g., 0.95 for weaker right motor)

// State
enum RobotState {
  WAITING_FOR_START,  // Initial wait for Pi
  START,              // Go through starting line
  EXPLORATION,       // Mapping the maze
  WAITING_FOR_RUN,  // Finished maze, waiting for shortest path from Pi
  SPEED_RUN          // Following the optimized path
};
RobotState currentState = WAITING_FOR_START;  // Initial state

// Communication
String resp = ""; // Data received from Pi

// Junction
struct Junction {
  bool left;
  bool front;
  bool right;
};

void setup() {
  // Serial Monitor
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

  // Initialize Encoders
  pinMode(LeftC1, INPUT_PULLUP);
  pinMode(LeftC2, INPUT_PULLUP);
  pinMode(RightC1, INPUT_PULLUP);
  pinMode(RightC2, INPUT_PULLUP);

  // Pull signal wires up to 3.3V to prevent floating signal
  leftEncoder.attachFullQuad(LeftC2, LeftC1);     // Use 4x Encoding (Every rise and fall of both Phase A & B)
  rightEncoder.attachFullQuad(RightC1, RightC2);
  
  leftEncoder.clearCount();
  rightEncoder.clearCount();

  // Sensor pins
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // PWM setup
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  // Calibrate sensor thresholds
  calibrateSensors();

  // UART Communication
  Serial2.begin(115200, SERIAL_8N1, rx, tx);
}

void loop() {
  switch (currentState) {
    case WAITING_FOR_START:
     if (Serial2.available()) {
       resp = read();
       if (resp == "START") {
        currentState = START;
       }
     }
     break;
  
    case START:
      motor_drive(ExpSpeed, ExpSpeed);
      sensorState = updateSensorState();
      
      while (sensorState == 0) {
        motor_drive(ExpSpeed, ExpSpeed);
        sensorState = updateSensorState();
        delay(1);
      }

      while ((sensorState & 0b10001) != 0 || (sensorState & 0b01110) == 0) { // Wait until any middle 3 is on and outer 2 is off
        sensorState = updateSensorState();
        motor_drive(ExpSpeed, ExpSpeed);
        delay(1);
        // Can add a timeout here
      }
      
      leftEncoder.clearCount();
      rightEncoder.clearCount();
      currentState = EXPLORATION;
      break;

    case EXPLORATION:
      long leftTicks = leftEncoder.getCount();
      long rightTicks = rightEncoder.getCount();
      float leftDist = (leftTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float rightDist = (rightTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float averageDistance = (leftDist + rightDist) / 2.0;

      int position = calculatePosition();
      sensorState = updateSensorState();

      // Junction encountered
      if ((sensorState & 0b10001) == 1) {
        Junction paths = checkJunction();
        send("JUNCTION:" + paths.left + "," + paths.front + "," + paths.right);

        while (!Serial2.available()) { // Wait for command from Pi
          delay(1);
        }

        resp = read();

        if (resp.length() > 1) { // Backtrack mode
          for (int i = 0; i < resp.length(); i++) {
            char command = resp.charAt(i);
            switch (cmd) {
              case 'F': // Forward
                int numF = 0;
                while (numF+i < resp.length() & resp.charAt(i+numF == 'F')) {
                  numF++;
                }
                error = 2000 - position; 
                currentDistance = averageDistance;
                while (currentDistance <= averageDistance + (numF - 1) * GRID_SIZE) { // Move until last grid
                  PID_Linefollow(error, ExpSpeed);
                }

                i += numF - 1;
                break;
              case 'B': // Backward
                moveBackward();
                break;
              case 'R': // Right Turn
                sensorState = updateSensorState();
                while ((sensorState & 0b00001) != 1) {
                  sensorState = updateSensorState();
                  motor_drive(ExpSpeed);
                }
                turnRight();
                moveForward(GRID_SIZE / 2 - 10 - CENTRE_DISTANCE, ExpSpeed);
                leftEncoder.clearCount();
                rightEncoder.clearCount();
                break;
              case 'L': // Left Turn
                turn90Left();
                break;
              default:
                Serial.println("Unknown command!");
                break;
            }
          }
        }

        else if (resp = "L") {
          turnLeft();
          moveForward(GRID_SIZE / 2 - 10 - CENTRE_DISTANCE, ExpSpeed);
          leftEncoder.clearCount();
          rightEncoder.clearCount();
        }

        else if (resp = "R") {
          turnRight();
          moveForward(GRID_SIZE / 2 - 10 - CENTRE_DISTANCE, ExpSpeed);
          leftEncoder.clearCount();
          rightEncoder.clearCount();
        }

        else if (resp = "F") {
          moveForward(GRID_SIZE / 2 - 10 - CENTRE_DISTANCE, speed);
          leftEncoder.clearCount();
          rightEncoder.clearCount();
        }

        else if (resp = "B") {
          Uturn();
          moveForward(GRID_SIZE / 2 - 10 - CENTRE_DISTANCE, speed);
          leftEncoder.clearCount();
          rightEncoder.clearCount();
        }

      // Update
      if (averageDistance >= GRID_SIZE) {
        send("UPDATE");
        leftEncoder.clearCount();
        rightEncoder.clearCount();
      }

      error = 2000 - position; 
      PID_Linefollow(error, ExpSpeed);
  }
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

// Send message to Raspi
void send(String message) {
  Serial2.println(message);
} 

// Check if receive message from Raspi
String read() {
  String data = Serial2.readStringUntil('\n');
  data.trim();
  Serial.print("Pi says: ");
  Serial.println(data);
  return data;
}

// --- Calculate line position --- 
int calculatePosition() { 
  long weightedSum = 0;
  long sum = 0;
  uint8_t state = 0;  // To check the sensors state, value vary from 00000 to 11111

  for (int i = 0; i < 5; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Check if sensor is above threshold (on the line)
    if (sensorValues[i] < threshold[i]) {
      state |= (1 << i);  // Sense black
    }
    // Use analog values for smoother position calculation
    int weight = map(sensorValues[i], threshold[i], 4095, 0, 1000); // Scale to 0-1000
    if (weight > 100) { // Only count significant detections
      weightedSum += (long)i * weight;
      sum += weight;
    }
  }


  // Original condition for no significant detection
  if (sum == 0) { // Changed from sum == 5000 to handle no detection case
    motor_drive(0, 0); // Stop motors
    return 2000; // Assume center
  }

  // Out of track
  if (state == 0b00000) {
    motor_drive(0, 0); // Stop motors
    return -1; // Indicate stop condition
  }

  // Turn left 'L' junction
  if (state == 0b11100) {

  }


  
  return (weightedSum * 1000) / sum; // Scale to 0-4000 range
}

// --- PID Control ---
void PID_Linefollow(int error, int speed) {
  P = error;
  I += error;
  I = constrain(I, -1000, 1000); // Prevent integral windup
  D = error - previousError;

  float PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;

  int lsp = speed - PIDvalue;
  int rsp = (speed + PIDvalue);

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
  ledcWrite(pwmA, abs(leftSpeed * motorBalance));

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

uint8_t updateSensorState() {
  uint8_t state = 0;
  for (int i = 0; i < 5; i++) {
    // If sensor is on the line (less than threshold)
    if (analogRead(sensorPins[i]) < threshold[i]) {
      // Set the i-th bit to 1
      state |= (1 << i); 
    }
  }
  return state; // Returns a value from 0 (00000) to 31 (11111)
}

Junction checkJunction() {
  Junction j;
  sensorState = updateSensorState();
  
  if ((sensorState & 0b11000) != 0) j.left = true;
  if ((sensorState & 0b00011) != 0) j.right = true;
  
  while ((sensorState & 0b10001) != 0) {
    sensorState = updateSensorState();
    if ((sensorState & 0b11000) != 0) j.left = true;
    if ((sensorState & 0b00011) != 0) j.right = true;

    motor_drive(ExpSpeed, ExpSpeed); // Keep moving forward
    delay(1);
  }
  
  moveForward(CENTRE_DISTANCE, ExpSpeed);

  sensorState = updateSensorState();
  if ((sensorState & 0b01110) != 0) {
    j.front = true;
  }  

  return j;
}

void moveForward(int distance, int speed) { // distance in mm
  long targetTicks = (1.0 * distance / CIRCUMFERENCE) * COUNTS_PER_REV;

  long IniLeft = leftEncoder.getCount();

  motor_drive(speed, speed);

  while (abs(leftEncoder.getCount() - IniLeft) < targetTicks) {
    // Optional: Add a small PID adjustment here if the robot veers
    motor_drive(speed, speed);
    delay(1); 
  }

  motor_drive(-150, -150); 

  long lastTicks = leftEncoder.getCount();
  delay(5); // Small window to check movement
  while (leftEncoder.getCount() > lastTicks) {
      lastTicks = leftEncoder.getCount();
      delay(2); 
  }

  motor_drive(0, 0);
}

void turnLeft() {
  motor_drive(-100, 100);
  delay(300);
}

void turnRight(int speed) {
  motor_drive(100, -100);
  delay(300);
}

void Uturn(int speed) {
  motor_drive(100, -100);
  delay(600);
}
