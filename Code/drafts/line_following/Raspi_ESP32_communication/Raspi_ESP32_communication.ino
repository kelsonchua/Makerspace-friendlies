#include <Arduino.h>

// Shared variable for communication between cores
// 'S' = solve (follow line), 'P' = pause (at intersection), 'T' = turning
volatile char robotState = 'S'; 
volatile char nextMove = ' ';

// Task Handles
TaskHandle_t MotorTask;
TaskHandle_t CommTask;

void setup() {
  Serial.begin(115200);      // USB Debugging
  Serial2.begin(115200);     // Connection to Raspberry Pi (TX2/RX2 pins)

  // Create Motor Task on Core 1 (High Priority)
  xTaskCreatePinnedToCore(
    motorLoop,    /* Function to implement the task */
    "MotorTask",  /* Name of the task */
    10000,        /* Stack size in words */
    NULL,         /* Task input parameter */
    3,            /* Priority of the task (High) */
    &MotorTask,   /* Task handle */
    1);           /* Core 1 */

  // Create Communication Task on Core 0
  xTaskCreatePinnedToCore(
    commLoop,
    "CommTask",
    10000,
    NULL,
    1,            /* Priority (Lower) */
    &CommTask,
    0);           /* Core 0 */
}

// --- CORE 1: PID & SENSORS ---
void motorLoop(void * pvParameters) {
  for(;;) {
    if (robotState == 'S') {
      // 1. RUN YOUR PID CODE HERE
      // 2. CHECK FOR INTERSECTION
      if (atIntersection()) {
         stopMotors();
         robotState = 'P'; // Pause and wait for Pi
      }
    }
    vTaskDelay(1); // Allow FreeRTOS to breathe
  }
}

// --- CORE 0: PI COMMUNICATION ---
void commLoop(void * pvParameters) {
  for(;;) {
    if (robotState == 'P') {
      // Tell Pi we are at an intersection
      Serial2.println("INTERSECTION");
      
      // Wait for Pi to send 'L', 'R', or 'S'
      while (!Serial2.available()) { vTaskDelay(1); }
      
      nextMove = Serial2.read();
      executeTurn(nextMove); // Execute the turn (L/R/S)
      
      robotState = 'S'; // Resume Line Following
    }
    vTaskDelay(10);
  }
}

void loop() {
  // Leave empty! Everything happens in the tasks above.
}

// --- HELPER FUNCTIONS ---
bool atIntersection() {
  // Your logic: if(sensor[0] == W && sensor[5] == W) return true;
  return false; 
}

void executeTurn(char move) {
  // Your logic to turn 90 degrees based on 'L', 'R', or 'S'
}