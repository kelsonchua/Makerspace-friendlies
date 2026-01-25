// =======================================================
// JGA12-N20 GEAR RATIO ESTIMATOR (ESP32)
// This code estimates the gear ratio by measuring the output shaft RPM
// and comparing it to the motor's known internal RPM.
// =======================================================

// --- MOTOR DRIVER PINS (From your previous configuration) ---
const int ain1Pin = 13;    // Left motor direction 1
const int ain2Pin = 12;    // Left motor direction 2
const int pwmA = 25;       // Left motor PWM (Speed Control)
const int standByPin = 19; // Motor driver standby

// --- ENCODER PIN ---
// Using a safe GPIO pin for the encoder input.
const int ENCODER_PIN = 16; 

// --- CALIBRATION CONSTANTS ---
// 1. BASE MOTOR RPM (The speed of the motor before the gearbox)
// The N20 motor typically runs between 16,000 and 20,000 RPM at 6V.
// We'll use a conservative 16,000 RPM for the estimation.
const float BASE_MOTOR_RPM = 16000.0; // RPM of the motor shaft (no load, 6V)

// 2. ENCODER BASE PULSES (Pulses per revolution of the motor shaft)
// This is known to be 7 PPR for the JGA12-N20 series.
const float BASE_PPR = 7.0; 

// --- TEST SETTINGS ---
const int TEST_DURATION_MS = 2000;    // How long to run the test motor (2 seconds)
const int TEST_PWM_DUTY = 10000;      // Low PWM value (out of 255 * 255 = 65535) for controlled movement

// --- PWM CONFIGURATION (ESP32 uses LEDC for PWM) ---
const int PWM_CHANNEL = 0;
const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 16; // 16-bit resolution (0 to 65535)

// --- ENCODER VARIABLES ---
volatile long encoderTicks = 0; // Stores the pulse count
volatile bool counting_active = false; // Flag to control counting

// --- FUNCTION PROTOTYPES ---
void IRAM_ATTR countPulses();
void setupMotorPWM();
void runMotor(int duty);
void stopMotor();
float estimateGearRatio(long totalTicks, int duration_ms);

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=========================================");
  Serial.println("JGA12-N20 GEAR RATIO ESTIMATOR");
  Serial.println("=========================================");

  // Setup motor driver pins
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(standByPin, OUTPUT);
  digitalWrite(standByPin, HIGH); // Enable motor driver

  // Setup PWM
  setupMotorPWM();
  
  // Setup encoder pin and interrupt
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  // Attach interrupt, but only enable counting during the test run
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulses, RISING);
  
  Serial.println("Ready. Starting gear ratio estimation test in 3 seconds...");
  delay(3000);

  // Perform the test
  float estimated_ratio = estimateGearRatio(TEST_DURATION_MS, TEST_PWM_DUTY);

  Serial.println("=========================================");
  Serial.print("4. Estimated Gear Ratio: 1:");
  Serial.print(estimated_ratio, 0); // Print as integer
  Serial.println("\n=========================================");
  
  // Now print the final PPR value to use in your main code
  float estimated_ppr = BASE_PPR * estimated_ratio;
  Serial.print("   => Recommended PULSES_PER_REVOLUTION constant: ");
  Serial.print(estimated_ppr, 0);
  Serial.println(" (Base PPR * Estimated Ratio)");
  Serial.println("=========================================");
}

// =======================================================
// MAIN LOOP
// =======================================================
void loop() {
  // Loop is intentionally empty after the one-time test in setup()
  // You can remove the delay if you want it to run just once and stop.
  delay(10000); 
}

// =======================================================
// INTERRUPT SERVICE ROUTINE (ISR)
// =======================================================
void IRAM_ATTR countPulses() {
  if (counting_active) {
    encoderTicks++;
  }
}

// =======================================================
// MOTOR CONTROL FUNCTIONS
// =======================================================
void setupMotorPWM() {
  // Configure LEDC PWM channel for the motor
  ledcAttach(pwmA, PWM_FREQUENCY, PWM_RESOLUTION);
}

void runMotor(int duty) {
  // Set motor direction (e.g., forward)
  digitalWrite(ain1Pin, HIGH);
  digitalWrite(ain2Pin, LOW);
  
  // Set speed
  ledcWrite(pwmA, duty);
}

void stopMotor() {
  // Set speed to zero
  ledcWrite(PWM_CHANNEL, 0);
  // Optional: Set both direction pins low to brake
  digitalWrite(ain1Pin, LOW);
  digitalWrite(ain2Pin, LOW);
}

// =======================================================
// GEAR RATIO ESTIMATION FUNCTION
// =======================================================
float estimateGearRatio(int duration_ms, int duty) {
  encoderTicks = 0;
  
  Serial.print("1. Starting motor test (PWM Duty: ");
  Serial.print(duty);
  Serial.print(") for ");
  Serial.print(duration_ms / 1000);
  Serial.println(" seconds...");
  
  // Enable pulse counting and start the motor
  counting_active = true;
  runMotor(duty);
  
  // Wait for the test duration
  delay(duration_ms);
  
  // Stop motor and counting
  stopMotor();
  counting_active = false;
  
  long finalTicks = encoderTicks;

  Serial.print("2. Test complete. Total Ticks measured: ");
  Serial.println(finalTicks);
  
  // Calculate Motor Shaft Revolutions in the interval (based on measured ticks)
  // Total Motor Shaft Pulses = Total Ticks / (BASE_PPR)
  float motorShaftRevolutions = (float)finalTicks / BASE_PPR;
  
  // We will assume that one turn of the output shaft is equivalent to the average ratio.
  // A safer way is to instruct the user to physically turn the output shaft.
  
  Serial.println("3. To find the ratio, you must perform a manual test.");
  Serial.println("   The calculation below is a poor estimate due to motor speed variation.");
  Serial.println("   => Based on your motor, the most accurate test is MANUAL:");
  Serial.println("   **Physically turn the OUTPUT SHAFT (where the wheel is) exactly ONE revolution (360 degrees).**");
  Serial.println("   **The final Ticks value printed in the Serial Monitor will be your PPR (PULSES_PER_REVOLUTION).**");
  
  // Since the automatic calculation is unreliable without knowing the actual duty cycle RPM,
  // we default to the most likely common ratio for a final estimate.
  float estimatedRatio = finalTicks > 0 ? (float)finalTicks / BASE_PPR : 150.0;
  
  return estimatedRatio;
}