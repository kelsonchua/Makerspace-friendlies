#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESP32Servo.h>

Servo myServo;

// --- CONFIG ---
#define WIFI_CHANNEL 6 
const float MANUAL_BALANCE = 1.0; 

// --- SPEED SETTINGS ---
const int SPEED_CRUISE = 40;  // Speed 1: "Walking" speed (0-255)
const int SPEED_TURBO  = 100;  // Speed 2: "Running" speed (0-255)
const float TURBO_THRESHOLD = 0.95; // Stick must be pushed 95% to trigger Turbo

// Motor Pins
const int ain1Pin = 13;
const int ain2Pin = 18;
const int pwmA = 25;    
const int bin1Pin = 14;
const int bin2Pin = 27;
const int pwmB = 26;    
const int launcher = 21;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;

// Launcher Variables
bool isLaunching = false;
unsigned long launchTimer = 0;

// ESP-NOW Structure
typedef struct struct_message {
  int x;
  int y;
  int button;
} struct_message;

struct_message incomingData;
volatile unsigned long lastRecvTime = 0;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  lastRecvTime = millis();
}

void setup() {
  Serial.begin(115200);

  // Pin Setup
  pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(launcher, OUTPUT);
  
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  myServo.setPeriodHertz(50); 
  myServo.attach(launcher, 500, 2400);
  myServo.write(100); 

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
}

void drive(int left, int right) {
  left = left * MANUAL_BALANCE; 

  if (left > 0) {
    digitalWrite(ain1Pin, HIGH); digitalWrite(ain2Pin, LOW);
  } else if (left < 0) {
    digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, HIGH);
  } else {
    digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, LOW);
  }
  ledcWrite(pwmA, abs(left));

  if (right > 0) {
    digitalWrite(bin1Pin, HIGH); digitalWrite(bin2Pin, LOW);
  } else if (right < 0) {
    digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, HIGH);
  } else {
    digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, LOW);
  }
  ledcWrite(pwmB, abs(right));
}

void handleLauncher() {
  if (incomingData.button == 0 && !isLaunching) {
    myServo.write(180);  // Move to 0
    delay(439);
    myServo.write(100);
}

void loop() {
  if (millis() - lastRecvTime > 500) {
    drive(0, 0);
    return;
  }

  handleLauncher();

  // 1. Normalize Input (-1.0 to 1.0)
  float rawY = (incomingData.y - 2048) / 2048.0; 
  float rawX = (incomingData.x - 2048) / 2048.0;

  // 2. Deadzone
  if (abs(rawX) < 0.1) rawX = 0;
  if (abs(rawY) < 0.1) rawY = 0;

  // 3. TWO-SPEED LOGIC
  // Calculate how far the stick is pushed from center (0.0 to 1.41)
  float stickMagnitude = sqrt((rawX * rawX) + (rawY * rawY));
  
  int currentMaxSpeed = SPEED_CRUISE; // Default to Slow

  // If stick is pushed harder than the threshold, switch to Fast
  if (stickMagnitude > TURBO_THRESHOLD) {
     currentMaxSpeed = SPEED_TURBO;
  }

  // 4. Arcade Mixing
  // We apply the speed limit to our mixing
  float leftOut  = (rawY + rawX);
  float rightOut = (rawY - rawX);

  // Constrain to -1.0 to 1.0 so we don't exceed math limits
  leftOut = constrain(leftOut, -1.0, 1.0);
  rightOut = constrain(rightOut, -1.0, 1.0);

  // 5. Apply the Variable Speed Limit
  // Instead of multiplying by 255, we multiply by currentMaxSpeed
  int finalLeft = leftOut * currentMaxSpeed;
  int finalRight = rightOut * currentMaxSpeed;

  drive(finalLeft, finalRight);
  delay(10); 
}