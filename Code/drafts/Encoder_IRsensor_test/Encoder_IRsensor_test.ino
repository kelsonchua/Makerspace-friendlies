#include <ESP32Encoder.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const char* laptop_ip = "172.20.10.3"; // Replace with your laptop's hotspot IP
const int udp_port = 4210;

// WiFi Credentials
const char* ssid = "Kelson iPhone";     // Must match exactly
const char* password = "kelson123";

// Pins
const int ain1Pin = 13;    // Left motor direction 1
const int ain2Pin = 18;     // Left motor direction 2
const int pwmA = 25;       // Left motor PWM
const int LeftC1 = 4;
const int LeftC2 = 5;
const int RightC1 = 23;
const int RightC2 = 22;
const int standByPin = 19; // Motor driver standby
const int bin1Pin = 14;    // Right motor direction 1
const int bin2Pin = 27;    // Right motor direction 2
const int pwmB = 26;       // Right motor PWM

const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];
uint8_t prevsensorState = 0;
uint8_t sensorState = 0;

// --- PWM Configuration for ESP32-S3 ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 8;        // 8-bit (0-255)

// Control
int speed = 50; // Base speed (increased for faster response)
const float motorBalance = 0.975; // Adjust if one motor is stronger (e.g., 0.95 for weaker right motor)

// Motor
const float GEAR_RATIO = 51.102;           
const int ENCODER_PPR = 7;                // Pulses per rev
const float COUNTS_PER_REV = ENCODER_PPR * 4 * GEAR_RATIO; // 2800
const float WHEEL_DIAMETER = 43.0;        // Actual wheel diameter in mm
const float CIRCUMFERENCE = PI * WHEEL_DIAMETER;

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// PID constants (tuned for faster response)
float Kp = 0.2;   // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.23;  // Derivative gain

long P, I, D, previousError = 0;
int error = 0;

// State
enum RobotState {
  WAITING_FOR_START,  // Initial wait for Pi
  START,              // Go through starting line
  FIRST_TURN,       // Mapping the maze
  SECOND_TURN,  // Finished maze, waiting for shortest path from Pi
  LAST_RUN          // Following the optimized path
};
RobotState currentState = WAITING_FOR_START;  // Initial state

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");

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

  // PWM setup
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);
  
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  calibrateSensors();

  send("Simple Line Following Robot Started");
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
  switch (currentState) {
    case WAITING_FOR_START:
      Serial.println("Waiting...");
      send("Waiting...");
      delay(1000);
      currentState = START;
      send("Entering START state");
      break;

    case START:
      motor_drive(speed, speed);
      sensorState = updateSensorState();
      if (prevsensorState != sensorState) {
        sendState(sensorState);
        prevsensorState = sensorState;
      }

      if ((sensorState & 0b00100)) {
        send("Entering FIRST_TURN state");
        currentState = FIRST_TURN;
      }
      break;

    case FIRST_TURN: { // <--- Added brace
      long leftTicks = leftEncoder.getCount();
      long rightTicks = rightEncoder.getCount();
      float leftDist = (leftTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float rightDist = (rightTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float averageDistance = (leftDist + rightDist) / 2.0;

      int position = calculatePosition();
      sensorState = updateSensorState();
      if (prevsensorState != sensorState) {
        sendState(sensorState);
        prevsensorState = sensorState;
      }

      if (sensorState == 0b11000) {
        send("Turn left junction detected, turning left...");
        moveForward(110, speed);
        turnLeft();
        send("Entering SECOND_TURN state");
        currentState = SECOND_TURN;
      }

      error = 2000 - position; 
      PID_Linefollow(error, speed);

      break;
    } // <--- Added brace

    case SECOND_TURN: { // <--- Added brace
      long leftTicks = leftEncoder.getCount();
      long rightTicks = rightEncoder.getCount();
      float leftDist = (leftTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float rightDist = (rightTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float averageDistance = (leftDist + rightDist) / 2.0;

      int position = calculatePosition();
      sensorState = updateSensorState();
      if (prevsensorState != sensorState) {
        sendState(sensorState);
        prevsensorState = sensorState;
      }
      
      // Changed "else if" to "if" because it's the start of a check in this state
      if (sensorState == 0b11111) { 
        send("Cross junction detected, turning right...");
        moveForward(110, speed);
        turnRight();
        motor_drive(0, 0);
        delay(1000);
        send("Entering LAST_RUN state");
        currentState = LAST_RUN;
      }
      
      error = 2000 - position; 
      PID_Linefollow(error, speed);
      break;
    }

    case LAST_RUN: { // <--- Added brace
      long leftTicks = leftEncoder.getCount();
      long rightTicks = rightEncoder.getCount();
      float leftDist = (leftTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float rightDist = (rightTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
      float averageDistance = (leftDist + rightDist) / 2.0;

      int position = calculatePosition();
      sensorState = updateSensorState();
      if (prevsensorState != sensorState) {
        sendState(sensorState);
        prevsensorState = sensorState;
      }

      // Changed "else if" to "if"
      if ((sensorState & 0b10001) != 0) {
        send("Detected disruption, moving forward...");
        moveForward(20, speed);
        send("Continue line following");
      }

      error = 2000 - position; 
      PID_Linefollow(error, speed);

      break;
    }
  }
  // long leftTicks = leftEncoder.getCount();
  // long rightTicks = rightEncoder.getCount();

  // float leftDist = (1.0 * leftTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
  // float rightDist = (1.0 * rightTicks / COUNTS_PER_REV) * CIRCUMFERENCE;
  // float averageDistance = (leftDist + rightDist) / 2.0;

  // int position = calculatePosition();
  // Serial.println(position);
  // delay(500);
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
    
    if (weight > 0) { // Only count significant detections
      weightedSum += (long)i * weight;
      sum += weight;
    }
  }

  if (sum == 0) {
    return 2000;  
  }

  return (weightedSum * 1000) / sum; // Scale to 0-4000 range
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
  delay(325);
  motor_drive(0, 0);
}

void turnRight() {
  motor_drive(100, -100);
  delay(325);
  motor_drive(0, 0);
}

void Uturn() {
  motor_drive(100, -100);
  delay(650);
  motor_drive(0, 0);
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

uint8_t updateSensorState() {
  uint8_t state = 0;
  for (int i = 0; i < 5; i++) {
    // If sensor is on the line (less than threshold)
    if (analogRead(sensorPins[i]) > threshold[i]) {
      // Set the i-th bit to 1
      state |= (1 << i); 
    }
  }
  return state; // Returns a value from 0 (00000) to 31 (11111)
}

void send(String msg) {
  udp.beginPacket(laptop_ip, udp_port);
  udp.print(msg);
  udp.endPacket();
}

void sendState(uint8_t msg) {
  String binaryMessage = "";
  
  // Loop through 5 bits (from 4 down to 0)
  for (int i = 4; i >= 0; i--) {
    // Check if the i-th bit is set
    if (msg & (1 << i)) {
      binaryMessage += "1";
    } else {
      binaryMessage += "0";
    }
  }

  send("sensorState: " + String(binaryMessage));
}