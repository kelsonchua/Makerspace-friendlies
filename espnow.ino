#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h> // <--- THIS WAS MISSING

// --- CONFIGURATION ---
// MATCH THIS TO YOUR ROBOT'S WIFI CHANNEL (Check Robot Serial Monitor)
#define WIFI_CHANNEL 6  

// PIN DEFINITIONS
#define JOY_X 1
#define JOY_Y 2
#define JOY_SW 10

// DATA STRUCTURE (Must match Robot)
typedef struct struct_message {
  int x;
  int y;
  int button;
} struct_message;

struct_message joystickData;

// Broadcast MAC (Sends to everyone)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: Blink LED on failure if needed
}

void setup() {
  Serial.begin(115200);

  // Init Joystick Pins
  pinMode(JOY_SW, INPUT_PULLUP);
  analogReadResolution(12);       
  analogSetAttenuation(ADC_11db);

  // Init WiFi
  WiFi.mode(WIFI_STA);
  
  // FORCE WIFI CHANNEL
  // This is crucial because the Robot is locked to the Hotspot's channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // This forces the compiler to accept your function regardless of the argument type
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

  // Register Peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.print("Joystick Ready on Channel: ");
  Serial.println(WIFI_CHANNEL);
}

void loop() {
  // Read Data
  joystickData.x = analogRead(JOY_X);
  joystickData.y = analogRead(JOY_Y);
  joystickData.button = digitalRead(JOY_SW);

  // Send Data
  esp_now_send(broadcastAddress, (uint8_t *) &joystickData, sizeof(joystickData));

  // Small delay for stability
  delay(50); 
}