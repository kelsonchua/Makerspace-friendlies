#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> 
#include <ESP32Servo.h>

Servo myServo;

// --- CONFIG ---
#define WIFI_CHANNEL 6 

const float MANUAL_BALANCE = 1.0; 

// Motor Pins
const int ain1Pin = 13;
const int ain2Pin = 18;
const int pwmA = 25;    // Left Motor
const int bin1Pin = 14;
const int bin2Pin = 27;
const int pwmB = 26;    // Right Motor
const int standByPin = 0; 
const int launcher = 21;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;

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

  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(launcher, OUTPUT);
  
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  myServo.setPeriodHertz(50); 
  myServo.attach(launcher, 500, 2400); // Typical pulse width for SG90/MG90

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  
  Serial.println("Pivot Racing Mode: READY");
}

void loop() {
  if (millis() - lastRecvTime > 500) {
    drive(0, 0);
    return;
  }

  // 1. Map Joystick
  int speed = map(incomingData.y, 0, 4095, 255, -255); 
  int turn  = map(incomingData.x, 0, 4095, -255, 255);

  // 2. Deadzone
  if (abs(speed) < 30) speed = 0;
  if (abs(turn) < 30)  turn = 0;

  int leftSpeed = 0;
  int rightSpeed = 0;

  // 3. NEW Mixing Logic: Pivot Turn
  if (turn > 125) {
    // Turning RIGHT: Only Left motor moves
    leftSpeed = speed;
    rightSpeed = 0;
  } 
  else if (turn < -125) {
    // Turning LEFT: Only Right motor moves
    leftSpeed = 0;
    rightSpeed = speed;
  } 
  else {
    // Going STRAIGHT: Both motors move
    leftSpeed = speed;
    rightSpeed = speed;
  }

  // 4. Constrain (Safety check)
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  drive(leftSpeed, rightSpeed);
  
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
  myServo.write(180);  // Move to 0
  delay(439);
  myServo.write(100);
}