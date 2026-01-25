#include <vector>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const char* laptop_ip = "172.20.10.3"; // Replace with your laptop's hotspot IP
const int udp_port = 4210;

// WiFi Credentials
const char* ssid = "Kelson iPhone";     // Must match exactly
const char* password = "kelson123";

const int ain1Pin = 13;     // Left motor direction 1
const int ain2Pin = 18;     // Left motor direction 2
const int pwmA = 25;        // Left motor PWM
const int LeftC1 = 4;      // Left motor C1
const int LeftC2 = 5;      // Left motor C2
const int RightC1 = 23;     // Right motor C1
const int RightC2 = 22;     // Right motor C2
const int bin1Pin = 14;     // Right motor direction 1
const int bin2Pin = 27;     // Right motor direction 2
const int pwmB = 26;        // Right motor PWM
const int button = 19;       // Button
const int launcher = 21;     // Launcher servo motor
const int collector = 17;     // Collecter servo motor

// --- IR Sensor Pins (TCRT5000 5-channel) ---
const int sensorPins[5] = {39, 34, 35, 32, 33};  // Out1..Out5
int sensorValues[5];
int threshold[5];
uint8_t sensorState = 0;
uint8_t prevsensorState = 0;

// --- PWM Configuration for ESP32-S3 ---
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES = 8;        // 8-bit (0-255)

// PID constants (tuned for faster response)
float Kp = 0.1;   // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 4.0;  // Derivative gain

long P, I, D, previousError = 0;
int error = 0;
int ExpSpeed = 80; // Exploration speed (increased for faster response)
int speed = 80;    // Speed run speed
const float motorBalance = 1.0; // Adjust if one motor is stronger (e.g., 0.95 for weaker right motor)

// Time
const int GOAL_TIME = 500;
const int FREE_TIME = 120;
const int TIME_OUT = 3000;

// Communication
String resp = ""; // Data received from Pi

// State
enum RobotState {
  WAITING,            // Initial wait for Pi
  START,              // Go through starting line
  EXPLORATION_MAZE,        // Mapping the maze
  EXPLORATION_FREE,
  START_SPEED_RUN,    // Go through starting line
  SPEED_RUN           // Following the optimized path
};
RobotState currentState = WAITING;    // Initial state

// Junction 
std::vector<char> path;  // Stores 'L', 'R', 'F'

// Speed run
int pathIndex = 0;
int position = 0;

void setup() {
  // Serial Monitor
  Serial.begin(115200);

  // WiFi connect
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  send("Connected");
  
  // Motor pins
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(LeftC1, INPUT);
  pinMode(LeftC2, INPUT);
  pinMode(RightC1, INPUT);
  pinMode(RightC2, INPUT);
  pinMode(launcher, OUTPUT);
  pinMode(collector, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  // Sensor pins
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // PWM setup
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  // Calibrate sensor thresholds
  calibrateSensors();
  Serial.println("Mode: Waiting");
  send("Mode: Waiting");
}

void loop() {
  switch (currentState) {
    case WAITING: {
      calculatePosition();
      motor_drive(0, 0);
      if (digitalRead(button) == LOW) {  
        unsigned long duration = buttonCheck();

        if (duration <= 500) {    // Button pressed
          currentState = EXPLORATION_MAZE;
          Serial.println("Mode: START");
          send("Mode: EXPLORATION MAZE");
        }

        else if (duration >= 1000) {  // Button hold for more than 1 second
          currentState = START_SPEED_RUN;
          Serial.println("Mode: Start speed run");
          send("Mode: Start speed run");
        }
      }

      break;
    }

    case START: {
      position = calculatePosition();
      int error = 2000 - position;
      if (position >= 0) {         // Following line
        PID_Linefollow(error, ExpSpeed);
      }

      else if (position == -2) {   // Found junction/dead end
        Serial.println("Junction detected");
        send("Junction detected");
        char junction = handleJunction();
        currentState = EXPLORATION_MAZE;
      }
      
      break;
    }

    case EXPLORATION_MAZE: {
      position = calculatePosition();
      int error = 2000 - position;
      if (position >= 0) {         // Following line
        PID_Linefollow(error, ExpSpeed);
      }

      else if (position == -2) {   // Found junction/dead end
        Serial.println("Junction detected");
        send("Junction detected");
        char junction = handleJunction();
        if (junction != 'N') {
          path.push_back(junction);
          simplifyPath();
          turn(junction, ExpSpeed);
        }
      }

      break;
    }

    case EXPLORATION_FREE: {
      position = calculatePosition();
      int error = 2000 - position;
      if (position >= 0) {         // Following line
        PID_Linefollow(error, ExpSpeed);
      }

      else if (position == -2) {   // Found junction/dead end
        Serial.println("Junction detected");
        send("Junction detected");
        char junction = handleJunction();
        if (junction != 'G') {
          path.push_back(junction);
          simplifyPath();
          turn(junction, ExpSpeed);
        }
      }

      break;
    }

    case START_SPEED_RUN: {
      position = calculatePosition();
      int error = 2000 - position;
      if (position >= 0) {         // Following line
        PID_Linefollow(error, ExpSpeed);
      }

      else if (position == -2) {   // Found junction/dead end
        Serial.println("Junction detected");
        send("Junction detected");
        char junction = handleJunction();
        currentState = EXPLORATION_MAZE;
      }
      
      break;
    }

    case SPEED_RUN: {
      position = calculatePosition();
      int error = 2000 - position;
      if (position >= 0) {         // Following line
        PID_Linefollow(error, ExpSpeed);
      }

      else if (position == -2) {   // Found junction
        Serial.println("Junction detected");
        send("Junction detected");
        handleJunction();

        if (pathIndex >= path.size()) {   // Finish path
          unsigned long startTime = millis();
          unsigned long duration = 0;
          while (position == -2) {        // Check whether reach goal
            duration = millis() - startTime;
            position = calculatePosition();
            if (duration >= FREE_TIME) {        // Reach goal
              motor_drive(0, 0);
              currentState = WAITING;
              Serial.println("Goal reached");
              send("Goal reached");
              Serial.println("Mode: Waiting");
              send("Mode: Waiting");
              break;
            }
            delay(1);
          }

          if (duration < FREE_TIME) { // If unexpected error happen
            Serial.println("Error: Goal not found");
            send("Error: Goal not found");
            Serial.println("Mode: Waiting");
            send("Mode: Waiting");
            currentState = WAITING;
            break;
          }
        }

        else {
          while (position == -2) {
            position = calculatePosition();
            delay(1);
          }
          turn(path[pathIndex], speed);
          pathIndex++;
        }    
      }

      break;
    }
  }
}

// --- Calibrate sensor thresholds ---
void calibrateSensors() {
  Serial.println("Calibrating sensors... Place robot on the line and wait.");
  send("Calibrating sensors... Place robot on the line and wait.");
  delay(2000); // Wait for placement
  for (int i = 0; i < 5; i++) {
    threshold[i] = 0.5 * 4095;
  }
  Serial.println("Calibration complete.");
  send("Calibration complete.");
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

// --- Calculate line position + Update sensor state --- 
int calculatePosition() { 
  long weightedSum = 0;
  long sum = 0;
  static uint8_t lastRawState = 0;
  static int consistencyCount = 0;
  uint8_t currentRawState = 0; 

  for (int i = 0; i < 5; i++) {
    int val = analogRead(sensorPins[i]);
    sensorValues[i] = val;

    // 1. Binary Mapping (White Line = 1)
    if (val > threshold[i]) {
      currentRawState |= (1 << (4 - i));
    }

    // 2. Analog Weighting for PID (Invert so White Line has higher weight)
    // map(value, fromLow, fromHigh, toLow, toHigh)
    int weight = map(val, threshold[i], 4095, 0, 1000); 
    weight = constrain(weight, 0, 1000);

    if (weight > 0) { // Noise filter
      weightedSum += (long)i * weight;
      sum += weight;
    }
  }

  if (currentRawState == lastRawState) {
    // If the new reading matches the previous one, increase confidence
    consistencyCount++;
  } else {
    // If it changed, reset the counter and update the "last" value
    consistencyCount = 0;
    lastRawState = currentRawState;
  }

  // Only update the global 'sensorState' if we have seen the same pattern
  // 3 times in a row. (Adjust '3' to '5' if you need more stability)
  if (consistencyCount >= 3) {
    sensorState = currentRawState;
  }

  if (prevsensorState != sensorState) {
    sendState(sensorState);
    prevsensorState = sensorState;
  }

  // Junction / dead end
  if (!(sensorState == 0b01000 || sensorState == 0b00100 || sensorState == 0b00010 || sensorState == 0b01100 || sensorState == 0b00110 || sensorState == 0b10000 || sensorState == 0b11000 || sensorState == 0b00001 || sensorState == 0b00011)) {
    return -2; 
  }

  // Normal Line Following
  if (sum == 0) {
    return 2000; // Default to center if unsure
  }

  return ((weightedSum * 1000) / sum); // Result is 0 to 4000
}

void send(String msg) {
  udp.beginPacket(laptop_ip, udp_port);
  udp.print(msg);
  udp.endPacket();
}

unsigned long buttonCheck() {
  unsigned long startTime = millis();
  unsigned long duration = 0;
  
  Serial.println("Button is pressed");
  send("Button is pressed");
  
  while (digitalRead(button) == LOW) {
    duration = millis() - startTime;
  }

  Serial.print("Button is pressed for ");
  Serial.println(duration);
  send("Button is pressed for " + String(duration) + " ms");
  return duration;
}

char handleJunction() {
  unsigned long startTime = millis();
  unsigned long goalTime = 0;
  bool canGoLeft = false;
  bool canGoRight = false; 
  
  send("Start checking");
  while (position == -2 && sensorState != 0b00000) {
    motor_drive(ExpSpeed, ExpSpeed); // Keep moving forward
    position = calculatePosition();
     
    // Accumulate findings (Latch logic: Once true, stays true)
    if ((sensorState & 0b10000)) canGoLeft = true;
    if ((sensorState & 0b00001)) canGoRight = true;

    switch (currentState) {
      case EXPLORATION_MAZE: {
        if (sensorState == 0b11111) {
          if (goalTime == 0) goalTime = millis(); // Start timer
          if (millis() - goalTime > FREE_TIME) {         // Check timer
            motor_drive(0, 0);
            send("Goal");
            send("Mode: WAITING");
            while (sensorState == 0b11111) {
              calculatePosition();
              delay(1);
            }
            currentState = WAITING;
            return 'N';
          }
        } else {
            goalTime = 0; // Reset timer if signal breaks
        }
        break;
      }

      case EXPLORATION_FREE: {
        if (sensorState == 0b11111) {
          if (goalTime == 0) goalTime = millis(); // Start timer
          if (millis() - goalTime > GOAL_TIME) {         // Check timer
            motor_drive(0, 0);
            send("Goal!!!!");
            send(String(path));
            currentState = WAITING;
            return 'G';
          }
        } else {
            goalTime = 0; // Reset timer if signal breaks
        }
        break;
      }
    }  
    delay(1);
  }
  send("Finish checking");

  bool canGoForward = !(sensorState == 0b00000); 

  send("Junction: " + String(canGoLeft) + " " + String(canGoForward) + " " + String(canGoRight));
  if (canGoLeft) {
    Serial.println("Turn Left");
    send("Turn Left");
    return 'L';
  } 
  else if (canGoForward) {
    Serial.println("Move Forward");
    send("Move Forward");
    return 'F';
  } 
  else if (canGoRight) {
    Serial.println("Turn Right");
    send("Turn Right");
    return 'R';
  } 
  else {
    Serial.println("U Turn");
    send("U Turn");
    return 'B';
  }
}

void simplifyPath() {
  int n = path.size();
  if (n < 3 || path[n-2] != 'B') return;

  char dir1 = path[n-3]; // Move before the dead end
  char dir2 = path[n-1]; // Move after the dead end
  char newDir = ' ';

  // Simplified Logic Table:
  if (dir1 == 'L' && dir2 == 'R') newDir = 'B';
  else if (dir1 == 'L' && dir2 == 'F') newDir = 'R';
  else if (dir1 == 'L' && dir2 == 'L') newDir = 'F';
  else if (dir1 == 'F' && dir2 == 'R') newDir = 'L';
  else if (dir1 == 'F' && dir2 == 'F') newDir = 'B';
  else if (dir1 == 'F' && dir2 == 'L') newDir = 'R';
  else if (dir1 == 'R' && dir2 == 'R') newDir = 'F';
  else if (dir1 == 'R' && dir2 == 'F') newDir = 'L';
  else if (dir1 == 'R' && dir2 == 'L') newDir = 'B';

  // Remove the 3 bad moves and push the 1 correct move
  if (newDir != ' ') {
      path.pop_back(); // Remove dir2
      path.pop_back(); // Remove 'B'
      path.pop_back(); // Remove dir1
      path.push_back(newDir);
  }
}

void turn(char move, int speed) {
  if (move == 'L') {
    Serial.println("Turning Left");
    send("Turning Left");
    
    unsigned long startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(100);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }

      calculatePosition();
      motor_drive(-0.75*speed, 0.75*speed);
      if ((sensorState & 0b10000) != 0) {
        break;
      }
    }

    startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(100);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }
      
      calculatePosition();
      motor_drive(-0.75*speed, 0.75*speed);
      if (sensorState == 0b00100) {
        break;
      }
    }

    Serial.println("Completed");
    send("Completed");
    motor_drive(0, 0);
  }

  else if (move == 'R') {
    Serial.println("Turning Right");
    send("Turning Right");
    unsigned long startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(100);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }
      
      calculatePosition();
      motor_drive(0.75*speed, -0.75*speed);
      if ((sensorState & 0b00001) != 0) {
        break;
      }
    }

    startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(100);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }
      
      calculatePosition();
      motor_drive(0.75*speed, -0.75*speed);
      if (sensorState == 0b00100) {
        break;
      }
    }

    Serial.println("Completed");
    send("Completed");
    motor_drive(0, 0);
  }

  else if (move == 'F') {
    Serial.println("Move Forward (Do nothing)");
    send("Move Forward (Do nothing)");
    return;
  }

  else if (move == 'B') {
    Serial.println("U turning");
    send("U turning");
    motor_drive(speed*1.5, speed*1.5);
    delay(1500);

    while (true) {
      calculatePosition();
      motor_drive(-0.75*speed, -0.75*speed);
      if ((sensorState & 0b01110) != 0) {
        break;
      }
    }

    unsigned long startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(100);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }

      calculatePosition();
      motor_drive(0.75*speed, -0.75*speed);
      if ((sensorState & 0b00001) != 0) {
        break;
      }
    }

    startTime = millis();

    while (true) {
      if (millis() - startTime > TIME_OUT) {
        motor_drive(-0.75*speed, 0.75*speed);
        delay(50);
        motor_drive(-0.75*speed, -0.75*speed);
        delay(100);
        startTime = millis();
      }

      calculatePosition();
      motor_drive(0.5*speed, -0.5*speed);
      if (sensorState == 0b00100) {
        break;
      }
    }

    Serial.println("Completed");
    send("Completed");
    motor_drive(0, 0);
  }
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