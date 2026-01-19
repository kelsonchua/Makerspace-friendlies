#include <Arduino.h>
#include <WiFi.h>

// ESP32-S3 joystick pins
#define JOY_X 1
#define JOY_Y 2
#define JOY_SW 10

const char* ssid = "KS.H";       // replace with your real SSID
const char* password = "jokx2670"; // replace with your real password

bool wifiStarted = false;
bool wifiConnected = false;
unsigned long wifiStartTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(JOY_SW, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  Serial.println("ESP32-S3 joystick test started");
}

void loop() {
  int xValue = analogRead(JOY_X);
  int yValue = analogRead(JOY_Y);
  int buttonState = digitalRead(JOY_SW);

  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(" | Y: ");
  Serial.print(yValue);
  Serial.print(" | Button: ");
  Serial.println(buttonState == LOW ? "PRESSED" : "RELEASED");

  delay(200);

  // Always print a heartbeat
  Serial.println("Loop is running");
  delay(1000);

  // Start Wi-Fi after 2 seconds to make sure Serial is ready
  if (!wifiStarted && millis() > 2000) {
    Serial.println("Starting Wi-Fi...");
    WiFi.disconnect(true); // erase old credentials
    delay(100);
    WiFi.begin(ssid, password);
    wifiStarted = true;
    wifiStartTime = millis();
  }

  // Check Wi-Fi status every second
  if (wifiStarted && !wifiConnected) {
    wl_status_t status = WiFi.status();
    Serial.print("Wi-Fi status: "); Serial.println(status);

    if (status == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println("Wi-Fi connected!");
      Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
    } else if (millis() - wifiStartTime > 20000) { // 20s timeout
      Serial.println("Failed to connect to Wi-Fi. Check SSID/password.");
    }
  }
}
