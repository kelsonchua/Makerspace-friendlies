#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const char* laptop_ip = "172.20.10.3"; // Replace with your laptop's hotspot IP
const int udp_port = 4210;

// ================= CONFIGURATION =================
const char* ssid = "Coconut";
const char* password = "zYALP0702";

// ================= PINS =================
const int ain1Pin = 13;
const int ain2Pin = 18;
const int pwmA = 25;

const int bin1Pin = 14;
const int bin2Pin = 27;
const int pwmB = 26;

const int launcherPin = 4; 

// ================= GLOBALS =================
WebServer server(80);
Servo launcherServo;

// --- FIX 1: LOWER FREQUENCY ---
// 20000Hz is too fast and fights the Servo. 
// 1000Hz is standard for motors and easier for the ESP32 to manage alongside a servo.
const int PWM_FREQ = 1000; 
const int PWM_RES = 8;

// Robot State
int currentSpeedPercent = 50; 
int pwmSpeed = 128;           
String currentDir = "S";      

// --- FIX 2: NON-BLOCKING SHOOTING VARIABLES ---
bool isShooting = false;
unsigned long shootStartTime = 0;
const int SHOOT_DURATION = 439; // How long the servo spins (ms)

// ================= HTML INTERFACE (YOUR DESIGN) =================
const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <style>
    /* --- RESET & LAYOUT --- */
    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      background: linear-gradient(to bottom right, #0f172a, #1e293b, #0f172a); /* Slate 900 */
      color: white;
      margin: 0; padding: 20px;
      height: 100vh;
      display: flex; flex-direction: column;
      overflow: hidden;
      user-select: none; -webkit-user-select: none;
      box-sizing: border-box;
    }

    /* --- SLIDER SECTION (TOP) --- */
    .slider-container {
      width: 100%; margin-bottom: 20px;
      display: flex; flex-direction: column; gap: 10px;
    }
    
    .slider-header {
      display: flex; justify-content: space-between; align-items: center;
      font-weight: 600; font-size: 1.1rem;
    }

    .speed-badge {
      background-color: rgba(51, 65, 85, 0.5); /* Slate 700/50 */
      color: #22d3ee; /* Cyan 400 */
      padding: 4px 12px; border-radius: 8px; font-weight: bold; font-size: 1.25rem;
    }

    /* CUSTOM RANGE SLIDER */
    input[type=range] {
      -webkit-appearance: none; width: 100%; height: 12px;
      background: #334155; border-radius: 8px; outline: none;
    }
    input[type=range]::-webkit-slider-thumb {
      -webkit-appearance: none; appearance: none;
      width: 28px; height: 28px;
      border-radius: 50%;
      background: #06b6d4; /* Cyan 500 */
      cursor: pointer;
      box-shadow: 0 4px 6px -1px rgba(6, 182, 212, 0.5);
      border: 2px solid #fff;
    }

    /* --- MAIN CONTROLLER AREA --- */
    .controls-area {
      flex: 1;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 20px;
    }

    /* --- LEFT SIDE: D-PAD GRID --- */
    .dpad-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-template-rows: repeat(3, 1fr);
      gap: 15px;
      width: 280px; height: 280px; /* Fixed size for consistency */
    }

    /* BUTTON BASE STYLES */
    button {
      border: none; border-radius: 12px;
      color: white; font-weight: bold; font-size: 1.2rem;
      cursor: pointer; position: relative;
      touch-action: manipulation;
      box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.5);
      transition: transform 0.1s, filter 0.1s;
      display: flex; align-items: center; justify-content: center;
    }

    button:active { transform: scale(0.95); filter: brightness(1.1); }

    /* D-PAD COLORS (Matching your React code) */
    .btn-front { background: linear-gradient(135deg, #3b82f6, #2563eb); } /* Blue */
    .btn-back  { background: linear-gradient(135deg, #3b82f6, #2563eb); } /* Blue */
    .btn-left  { background: linear-gradient(135deg, #a855f7, #9333ea); } /* Purple */
    .btn-right { background: linear-gradient(135deg, #a855f7, #9333ea); } /* Purple */
    .btn-stop  { background: linear-gradient(135deg, #dc2626, #b91c1c); font-size: 1rem; } /* Red */

    /* Positioning in Grid */
    .pos-front { grid-column: 2; grid-row: 1; }
    .pos-left  { grid-column: 1; grid-row: 2; }
    .pos-stop  { grid-column: 2; grid-row: 2; }
    .pos-right { grid-column: 3; grid-row: 2; }
    .pos-back  { grid-column: 2; grid-row: 3; }

    /* --- RIGHT SIDE: FIRE BUTTON --- */
    .fire-container {
      display: flex; align-items: center; justify-content: center;
      padding-right: 20px;
    }

    .btn-fire {
      width: 180px; height: 180px; /* Big Circle */
      border-radius: 50%;
      background: linear-gradient(135deg, #ef4444, #f97316, #dc2626); /* Red/Orange */
      border: 6px solid rgba(254, 202, 202, 0.3); /* Red-300 transparent border */
      font-size: 2.5rem;
      letter-spacing: 2px;
      text-shadow: 0 2px 4px rgba(0,0,0,0.3);
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.4); }
      70% { box-shadow: 0 0 0 20px rgba(239, 68, 68, 0); }
      100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
    }

  </style>
</head>
<body>

  <div class="slider-container">
    <div class="slider-header">
      <label>Speed Control</label>
      <span class="speed-badge" id="valBox">50</span>
    </div>
    <input type="range" min="0" max="100" value="50" oninput="updateSpeed(this.value)">handle
  </div>

  <div class="controls-area">
    
    <div class="dpad-grid">
      <button class="btn-front pos-front" ontouchstart="send('F')" ontouchend="send('S')" onmousedown="send('F')" onmouseup="send('S')">FRONT</button>
      
      <button class="btn-left pos-left"   ontouchstart="send('L')" ontouchend="send('S')" onmousedown="send('L')" onmouseup="send('S')">LEFT</button>
      <button class="btn-stop pos-stop"   ontouchstart="send('S')" onmousedown="send('S')">STOP</button>
      <button class="btn-right pos-right" ontouchstart="send('R')" ontouchend="send('S')" onmousedown="send('R')" onmouseup="send('S')">RIGHT</button>
      
      <button class="btn-back pos-back"   ontouchstart="send('B')" ontouchend="send('S')" onmousedown="send('B')" onmouseup="send('S')">BACK</button>
    </div>

    <div class="fire-container">
      <button class="btn-fire" onclick="shoot()">FIRE</button>
    </div>

  </div>

<script>
  // Prevent context menu (right click)
  document.addEventListener('contextmenu', event => event.preventDefault());

  function updateSpeed(val) {
    document.getElementById("valBox").innerText = val;
    fetch("/speed?val=" + val);
  }

  function send(dir) {
    fetch("/" + dir);
  }

  function shoot() {
    fetch("/SHOOT");
    // Visual Flash effect
    const btn = document.querySelector('.btn-fire');
    btn.style.filter = "brightness(1.5)";
    setTimeout(() => { btn.style.filter = "brightness(1.0)"; }, 150);
  }
</script>

</body>
</html>
)=====";

// ================= HELPERS =================

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

void updateMotorState() {
  if (currentDir == "F") drive(pwmSpeed, pwmSpeed);
  else if (currentDir == "B") drive(-pwmSpeed, -pwmSpeed);
  else if (currentDir == "L") drive(-pwmSpeed, pwmSpeed);
  else if (currentDir == "R") drive(pwmSpeed, -pwmSpeed);
  else drive(0, 0);
}

// ================= HANDLERS =================

void handleRoot() { server.send(200, "text/html", HTML_PAGE); }

void handleSpeed() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    currentSpeedPercent = constrain(val, 0, 100);
    pwmSpeed = map(currentSpeedPercent, 0, 100, 0, 255);
    updateMotorState();
    server.send(200, "text/plain", "OK");
  }
}

void handleF() { currentDir = "F"; updateMotorState();  server.send(200, "text/plain", "OK"); }
void handleB() { currentDir = "B"; updateMotorState();  server.send(200, "text/plain", "OK"); }
void handleL() { currentDir = "L"; updateMotorState();  server.send(200, "text/plain", "OK"); }
void handleR() { currentDir = "R"; updateMotorState();  server.send(200, "text/plain", "OK"); }
void handleS() { currentDir = "S"; updateMotorState();  server.send(200, "text/plain", "OK"); }

// --- FIX 3: INSTANT HANDLER ---
// This function now only "Start" the process. It does not wait.
void handleShoot() {
  Serial.println("FIRE START");
  if (!isShooting) {
    isShooting = true;
    shootStartTime = millis(); // Record the exact time we started
    launcherServo.write(180);  // Start spinning
  }
  server.send(200, "text/plain", "BANG");
}

void stop() {
  send("Stopping");
  drive(0, 0);
}

// ================= SETUP =================

void setup() {
  Serial.begin(115200);

  pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  
  // --- ORDER MATTERS ---
  // 1. Attach Servo FIRST (Secures a Timer)
  launcherServo.setPeriodHertz(50);
  launcherServo.attach(launcherPin);
  launcherServo.write(90); // Stop

  // 2. Attach Motors SECOND (Uses remaining timers)
  // We use 1000Hz instead of 20000Hz to avoid conflict
  ledcAttach(pwmA, PWM_FREQ, PWM_RES);
  ledcAttach(pwmB, PWM_FREQ, PWM_RES);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.print("CONTROLLER IP: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/F", handleF); server.on("/B", handleB);
  server.on("/L", handleL); server.on("/R", handleR);
  server.on("/S", handleS); server.on("/SHOOT", handleShoot);

  server.begin();
}

// ================= LOOP =================

void loop() {
  server.handleClient();

  // --- FIX 4: CHECK SERVO TIME EVERY LOOP ---
  // This allows the robot to keep driving while checking the time
  if (isShooting) {
    // If enough time has passed...
    if (millis() - shootStartTime > SHOOT_DURATION) {
      launcherServo.write(90); // STOP spinning
      isShooting = false;
      Serial.println("FIRE END");
    }
  }
}

void send(String msg) {
  udp.beginPacket(laptop_ip, udp_port);
  udp.print(msg);
  udp.endPacket();
}
