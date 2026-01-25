#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ================= CONFIGURATION =================
const char* ssid = "Redmi Note 11 Pro 5G";      // <--- YOUR HOTSPOT NAME
const char* password = "zhehong0212";      // <--- YOUR HOTSPOT PASSWORD

// ================= PINS =================
// Motor A (Left)
const int ain1Pin = 13;
const int ain2Pin = 18;
const int pwmA = 25;

// Motor B (Right)
const int bin1Pin = 14;
const int bin2Pin = 27;
const int pwmB = 26;

// Accessories
const int launcherPin = 4; // Servo

// ================= GLOBALS =================
WebServer server(80);
Servo myServo;

// PWM Properties
const int PWM_FREQ = 20000;
const int PWM_RES = 8;

// Robot State
int currentSpeedPercent = 50; // Default 50%
int pwmSpeed = 128;           // Calculated PWM (0-255)
String currentDir = "S";      // Track current direction to update speed live

// ================= HTML INTERFACE =================
// This is the webpage your phone will load
const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <style>
    body { font-family: sans-serif; background-color: #121212; color: white; text-align: center; overflow: hidden; }
    h2 { margin: 10px 0; }
    
    /* SLIDER STYLING */
    .slidecontainer { width: 90%; margin: 20px auto; }
    .slider { -webkit-appearance: none; width: 100%; height: 15px; border-radius: 5px; background: #333; outline: none; }
    .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 30px; height: 30px; border-radius: 50%; background: #04AA6D; cursor: pointer; }
    
    /* BUTTON STYLING */
    .btn-grid { display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 10px; }
    .row { display: flex; gap: 20px; }
    
    button {
      width: 80px; height: 80px;
      border: none; border-radius: 15px;
      font-size: 30px; color: white;
      touch-action: manipulation; /* Prevents double-tap zoom */
      user-select: none;
    }

    .fwd, .back { background-color: #2196F3; }
    .left, .right { background-color: #FF9800; }
    .shoot { background-color: #f44336; width: 100px; height: 100px; border-radius: 50%; font-weight: bold; border: 4px solid #fff; }
    
    /* FEEDBACK TEXT */
    #valBox { font-size: 20px; font-weight: bold; color: #04AA6D; }
  </style>
</head>
<body>

  <h2>Speed: <span id="valBox">50</span>%</h2>
  
  <div class="slidecontainer">
    <input type="range" min="0" max="100" value="50" class="slider" id="speedRange" oninput="updateSpeed(this.value)">
  </div>

  <div class="btn-grid">
    <button class="fwd" ontouchstart="send('F')" ontouchend="send('S')">▲</button>
    
    <div class="row">
      <button class="left" ontouchstart="send('L')" ontouchend="send('S')">◄</button>
      <button class="shoot" onclick="shoot()">FIRE</button>
      <button class="right" ontouchstart="send('R')" ontouchend="send('S')">►</button>
    </div>

    <button class="back" ontouchstart="send('B')" ontouchend="send('S')">▼</button>
  </div>

<script>
  // Send Speed Update (Throttled slightly to prevent flooding)
  function updateSpeed(val) {
    document.getElementById("valBox").innerText = val;
    fetch("/speed?val=" + val);
  }

  // Send Direction Command
  function send(dir) {
    fetch("/" + dir);
  }

  // Send Shoot Command
  function shoot() {
    fetch("/SHOOT");
    // Optional: Visual feedback
    let btn = document.querySelector('.shoot');
    btn.style.backgroundColor = "#fff";
    setTimeout(() => { btn.style.backgroundColor = "#f44336"; }, 200);
  }
</script>

</body>
</html>
)=====";

// ================= HELPERS =================

void drive(int left, int right) {
  // Left Motor
  if (left > 0) { digitalWrite(ain1Pin, HIGH); digitalWrite(ain2Pin, LOW); }
  else if (left < 0) { digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, HIGH); }
  else { digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, LOW); }
  ledcWrite(pwmA, abs(left));

  // Right Motor
  if (right > 0) { digitalWrite(bin1Pin, HIGH); digitalWrite(bin2Pin, LOW); }
  else if (right < 0) { digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, HIGH); }
  else { digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, LOW); }
  ledcWrite(pwmB, abs(right));
}

// Re-apply current direction with NEW speed
void updateMotorState() {
  if (currentDir == "F") drive(pwmSpeed, pwmSpeed);
  else if (currentDir == "B") drive(-pwmSpeed, -pwmSpeed);
  else if (currentDir == "L") drive(-pwmSpeed, pwmSpeed);
  else if (currentDir == "R") drive(pwmSpeed, -pwmSpeed);
  else drive(0, 0);
}

// ================= HANDLERS =================

void handleRoot() {
  server.send(200, "text/html", HTML_PAGE);
}

void handleSpeed() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    currentSpeedPercent = constrain(val, 0, 100);
    
    // Map 0-100% to 0-255 PWM
    pwmSpeed = map(currentSpeedPercent, 0, 100, 0, 255);
    
    // Apply new speed IMMEDIATELY if moving
    updateMotorState();
    
    server.send(200, "text/plain", "OK");
    Serial.printf("Speed set to: %d%%\n", currentSpeedPercent);
  }
}

void handleF() { currentDir = "F"; updateMotorState(); server.send(200, "text/plain", "OK"); }
void handleB() { currentDir = "B"; updateMotorState(); server.send(200, "text/plain", "OK"); }
void handleL() { currentDir = "L"; updateMotorState(); server.send(200, "text/plain", "OK"); }
void handleR() { currentDir = "R"; updateMotorState(); server.send(200, "text/plain", "OK"); }
void handleS() { currentDir = "S"; updateMotorState(); server.send(200, "text/plain", "OK"); }

void handleShoot() {
  Serial.println("FIRE!");
  myServo.setPeriodHertz(50);
  myServo.attach(launcherPin, 500, 2400);

  myServo.write(180);  // Move to 0
  Serial.println("Shooting");
  delay(439);
  myServo.write(100);

  myServo.detach();

  Serial.println("Completed");
  server.send(200, "text/plain", "BANG");
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  // Pin Config
  pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  
  // PWM Config
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);

  // WiFi Config
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("CONTROLLER IP: http://");
  Serial.println(WiFi.localIP());

  // Server Routes
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed); // Handle Slider
  server.on("/F", handleF);
  server.on("/B", handleB);
  server.on("/L", handleL);
  server.on("/R", handleR);
  server.on("/S", handleS);
  server.on("/SHOOT", handleShoot);

  server.begin();
}

void loop() {
  server.handleClient();
}