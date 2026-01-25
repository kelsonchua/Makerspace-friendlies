void setup() {
  Serial.begin(115200);   // To PC (USB)
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // To Pi (RX=16, TX=17)
  Serial.println("ESP32 UART Test Started...");
}

void loop() {
  // Send to Pi
  Serial2.println("PING_FROM_ESP");
  
  // Check if Pi sent something back
  if (Serial2.available()) {
    String response = Serial2.readStringUntil('\n');
    Serial.print("Pi says: ");
    Serial.println(response);
  }
  delay(1000);
}