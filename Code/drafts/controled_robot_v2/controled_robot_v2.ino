#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESP32Servo.h>

Servo myServo;

// --- CONFIG ---
#define WIFI_CHANNEL 6 

// --- NEW CONFIG ---
const int MAX_SPEED = 200;       // 0 to 255 (Lower this to make it slower/safer)
const float TURN_SENSITIVITY = 0.6; // 0.1 to 1.0 (Lower this if it spins too fast)
const int DEADZONE = 40;         // Ignores small joystick drift

// Motor Pins
const int ain1Pin = 13;
const int ain2Pin = 18;
const int pwmA = 25;    // Left Motor
const int bin1Pin = 14;
const int bin2Pin = 27;
const int pwmB = 26;    // Right Motor
const int launcher = 4; // Servo Pin

const int PWM_FREQ = 1000;
const int PWM_RES = 8;

// State variable to prevent "machine gun" firing
int prevButtonState = 0; 

// ESP-NOW Structure (Must match Transmitter)
typedef struct struct_message {
  int x;
  int y;
  int button;
} struct_message;

struct_message incomingData;
volatile unsigned long lastRecvTime = 0;

void drive(int leftSpeed, int rightSpeed);
void launch();

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  lastRecvTime = millis();
}

void setup() {
  Serial.begin(115200);

  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  // Note: ledcAttach is for ESP32 Arduino Core v3.0+. 
  // If using v2.x, use ledcSetup and ledcAttachPin.
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  myServo.setPeriodHertz(50); 
  myServo.attach(launcher, 500, 2400); 
  myServo.write(100); // Set initial 'closed' position

  WiFi.mode(WIFI_STA);
  // Trick to set channel without connecting to AP
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  Serial.println("Robot Ready.");
}

void loop() {
  // Failsafe: Stop if signal lost
  if (millis() - lastRecvTime > 500) {
    drive(0, 0);
    return;
  }

  // --- 1. READ & CLEAN INPUTS ---
  // Map 0-4095 to -255 to +255
  // NOTE: If robot moves backward when pushing Stick UP, swap 255 and -255 in 'rawY'
  int rawY = map(incomingData.y, 0, 4095, 255, -255); 
  int rawX = map(incomingData.x, 0, 4095, -255, 255);

  // Apply Deadzone (If stick is close to center, set to 0)
  if (abs(rawY) < DEADZONE) rawY = 0;
  if (abs(rawX) < DEADZONE) rawX = 0;

  // --- 2. LAUNCHER LOGIC ---
  if (incomingData.button == 0 && prevButtonState == 1) {
    launch();
  }
  prevButtonState = incomingData.button;

  // --- 3. ARCADE DRIVE MIXING (The Smooth Logic) ---
  // Throttle is direct
  float throttle = rawY;
  
  // Turn is reduced by sensitivity so it doesn't spin out of control
  float turn = rawX * TURN_SENSITIVITY;

  // Calculate Motor Speeds
  // Left = Forward + Turn
  // Right = Forward - Turn
  int leftMotor = throttle + turn;
  int rightMotor = throttle - turn;

  // --- 4. SCALE SPEED ---
  // This maps the full joystick range to your custom MAX_SPEED limit
  leftMotor = map(leftMotor, -255, 255, -MAX_SPEED, MAX_SPEED);
  rightMotor = map(rightMotor, -255, 255, -MAX_SPEED, MAX_SPEED);

  // --- 5. CONSTRAIN ---
  // Ensure we never send more than 255 to the motors
  leftMotor = constrain(leftMotor, -255, 255);
  rightMotor = constrain(rightMotor, -255, 255);

  drive(leftMotor, rightMotor);
  
  delay(10); 
}

void drive(int leftSpeed, int rightSpeed) {
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

void launch() {
  Serial.println("FIRE!");
  drive(0,0); // Stop robot for stability
  myServo.write(180);  // Open / Launch
  delay(439);          // Wait for servo to move
  myServo.write(100);  // Close / Reset
  // Note: delay() pauses the code. Robot will be unresponsive for 0.45s.
}