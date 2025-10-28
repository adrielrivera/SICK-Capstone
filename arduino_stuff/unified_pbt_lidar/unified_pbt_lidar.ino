// Unified SICK7 System - PBT Sensor + LiDAR Detection + GPIO Control
// Combines working LiDAR detection with PBT sensor processing
// Single Arduino handles both systems for Pi

const int PBT_PIN = A0;           // PBT sensor input
const int TIM100_PIN = 8;         // TiM100 (Left) detection input
const int TIM150_PIN = 9;         // TiM150 (Right) detection input
const int LED_PIN = 13;           // Status LED
const int GPIO_PIN6 = 6;          // Arcade motherboard Pin 6 (Press START)
const int GPIO_PIN5 = 5;          // Arcade motherboard Pin 5 (Press ACTIVE)

// PBT Sensor Configuration
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int AVG_SAMPLES = 4;
const int CALIBRATION_SAMPLES = 1000;

// PBT Sensor Variables
int baselineOffset = 512;
bool calibrated = false;
int calCount = 0;
long calSum = 0;
unsigned long nextSample = 0;

// LiDAR Status
bool tim100_detected = false;
bool tim150_detected = false;
int lastTim100State = LOW;
int lastTim150State = LOW;

// Status reporting timing
unsigned long lastStatusReport = 0;
const unsigned long STATUS_INTERVAL = 500;  // Report every 500ms

// Diagnostics
unsigned long lastStatsMs = 0;
unsigned long sampleCount = 0;
int minValue = 1023;
int maxValue = 0;
int activityThreshold = 30;
unsigned long lastActivityMs = 0;
const unsigned long LED_ON_MS = 100;

// GPIO command buffer
String gpioCommand = "";

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(PBT_PIN, INPUT);
  pinMode(TIM100_PIN, INPUT);  // TiM100 (Left) - external pull-down resistor to GND
  pinMode(TIM150_PIN, INPUT);  // TiM150 (Right) - external pull-down resistor to GND
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN6, OUTPUT);
  pinMode(GPIO_PIN5, OUTPUT);
  
  // Set initial states
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GPIO_PIN6, LOW);   // Pin 6 normally LOW
  digitalWrite(GPIO_PIN5, HIGH);  // Pin 5 normally HIGH
  
  analogReference(DEFAULT);
  nextSample = micros();
  lastStatsMs = millis();
  lastStatusReport = millis();
  delay(100);
  
  Serial.println("========================================");
  Serial.println("SICK7 Unified System - PBT + LiDAR");
  Serial.println("========================================");
  Serial.println("PBT Sensor: A0");
  Serial.println("TiM100 Pin 8, TiM150 Pin 9");
  Serial.println("GPIO: Pin 5 (ACTIVE), Pin 6 (START)");
  Serial.println("HIGH = Person detected, LOW = Area clear");
  Serial.println("# CALIBRATING PBT SENSOR...");
  Serial.println("========================================");
}

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = millis();
  
  // Handle GPIO commands from Pi
  handleGPIOCommands();
  
  // LED activity indicator
  if (nowMs - lastActivityMs > LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // PBT sensor reading
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Read with averaging
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
      delayMicroseconds(10);
    }
    int rawValue = sum / AVG_SAMPLES;
    
    // Calibration phase
    if (!calibrated) {
      calSum += rawValue;
      calCount++;
      
      if (calCount >= CALIBRATION_SAMPLES) {
        baselineOffset = calSum / calCount;
        calibrated = true;
        
        Serial.print("# BASELINE=");
        Serial.println(baselineOffset);
        Serial.println("# READY");
        
        // Blink LED to indicate ready
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(100);
          digitalWrite(LED_PIN, LOW);
          delay(100);
        }
      }
      return;
    }
    
    // Calibrate and center value
    int value = rawValue - baselineOffset + 512;
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    
    // Activity detection for LED
    int deviation = abs(value - 512);
    if (deviation > activityThreshold) {
      digitalWrite(LED_PIN, HIGH);
      lastActivityMs = nowMs;
    }
    
    // Track statistics
    sampleCount++;
    if (rawValue < minValue) minValue = rawValue;
    if (rawValue > maxValue) maxValue = rawValue;
    
    // Send PBT data (just the number)
    Serial.println(value);
  }
  
  // LiDAR detection (like the working tim1xx_test.ino)
  int tim100_state = digitalRead(TIM100_PIN);
  int tim150_state = digitalRead(TIM150_PIN);
  
  // Update detection status based on current state
  tim100_detected = (tim100_state == HIGH);
  tim150_detected = (tim150_state == HIGH);
  
  // Report LiDAR status changes
  if (tim100_state != lastTim100State) {
    if (tim100_detected) {
      Serial.println("🚨 ===== PERSON DETECTED - LEFT SIDE (TiM100) =====");
      Serial.println("# TiM100 DETECTED - Person on LEFT side");
    } else {
      Serial.println("✅ ===== AREA CLEAR - LEFT SIDE (TiM100) =====");
      Serial.println("# TiM100 CLEAR - LEFT side clear");
    }
    lastTim100State = tim100_state;
  }
  
  if (tim150_state != lastTim150State) {
    if (tim150_detected) {
      Serial.println("🚨 ===== PERSON DETECTED - RIGHT SIDE (TiM150) =====");
      Serial.println("# TiM150 DETECTED - Person on RIGHT side");
    } else {
      Serial.println("✅ ===== AREA CLEAR - RIGHT SIDE (TiM150) =====");
      Serial.println("# TiM150 CLEAR - RIGHT side clear");
    }
    lastTim150State = tim150_state;
  }
  
  // Report combined status every 0.5 seconds
  if (nowMs - lastStatusReport >= STATUS_INTERVAL) {
    reportCombinedStatus();
    lastStatusReport = nowMs;
  }
  
  // Print diagnostics every 5 seconds
  if (calibrated && nowMs - lastStatsMs >= 5000) {
    printDiagnostics();
    lastStatsMs = nowMs;
    minValue = 1023;
    maxValue = 0;
  }
}

void handleGPIOCommands() {
  // Read serial commands from Pi
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (gpioCommand.length() > 0) {
        processGPIOCommand(gpioCommand);
        gpioCommand = "";
      }
    } else {
      gpioCommand += c;
    }
  }
}

void processGPIOCommand(String command) {
  command.trim();
  
  if (command == "PBT_HIT") {
    Serial.println("# PBT HIT: Received from Pi");
  }
  else if (command == "PIN6_HIGH") {
    digitalWrite(GPIO_PIN6, HIGH);
    Serial.println("# GPIO: Pin 6 → HIGH (5V)");
  } 
  else if (command == "PIN6_LOW") {
    digitalWrite(GPIO_PIN6, LOW);
    Serial.println("# GPIO: Pin 6 → LOW (0V)");
  } 
  else if (command == "PIN5_HIGH") {
    digitalWrite(GPIO_PIN5, HIGH);
    Serial.println("# GPIO: Pin 5 → HIGH (5V)");
  } 
  else if (command == "PIN5_LOW") {
    digitalWrite(GPIO_PIN5, LOW);
    Serial.println("# GPIO: Pin 5 → LOW (0V)");
  }
  else if (command == "RESET_GPIO") {
    digitalWrite(GPIO_PIN6, LOW);
    digitalWrite(GPIO_PIN5, HIGH);
    Serial.println("# GPIO: Reset to idle state");
  }
  else if (command == "STATUS") {
    Serial.print("# STATUS: TiM100=");
    Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
    Serial.print(" TiM150=");
    Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

void reportCombinedStatus() {
  // Report in the format expected by app_combined.py
  Serial.print("# LIDAR_STATUS: TIM100=");
  Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
  Serial.print(" TIM150=");
  Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
}

void printDiagnostics() {
  int range = maxValue - minValue;
  
  Serial.print("# STATS: samples=");
  Serial.print(sampleCount);
  Serial.print(" raw_min=");
  Serial.print(minValue);
  Serial.print(" raw_max=");
  Serial.print(maxValue);
  Serial.print(" range=");
  Serial.print(range);
  Serial.print(" (");
  Serial.print((range * 5.0) / 1023.0, 2);
  Serial.println("V)");
  
  Serial.print("# LIDARS: TiM100=");
  Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
  Serial.print(" TiM150=");
  Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
  
  if (range < 20) {
    Serial.println("# No PBT activity detected");
  } else if (range > 200) {
    Serial.println("# High PBT activity detected!");
  }
}

// Hardware Connections:
// PBT Sensor: A0
// TiM100 (Left): Pin 8 (with pull-down resistor to GND)
// TiM150 (Right): Pin 9 (with pull-down resistor to GND)
// LED: Pin 13
// Arcade GPIO: Pin 5 (ACTIVE), Pin 6 (START)
//
// Output Format:
// - PBT data: Just numbers (512, 513, 514...)
// - LiDAR status: # LIDAR_STATUS: TIM100=CLEAR TIM150=DETECTED
// - Detection events: # TiM100 DETECTED - Person on LEFT side
// - GPIO commands: Responds to PIN5_HIGH, PIN6_LOW, etc.
