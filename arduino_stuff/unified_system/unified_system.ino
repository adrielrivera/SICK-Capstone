// Unified SICK7 System - Credit Tracking + PBT Processing + Alarm Control
// Integrates: Credit detection, PBT sensor processing, GPIO control, and safety alarm
// Only activates alarm system when credits are available

const int PBT_PIN = A0;           // PBT sensor input
const int CREDIT_PIN = 2;         // Credit falling edge input (with pull-up)
const int TIM100_PIN = 8;         // TiM100 (Left) detection input
const int TIM150_PIN = 9;         // TiM150 (Right) detection input
const int BUZZER_PIN = 10;        // Piezo buzzer output (moved to avoid conflict)
const int LED_PIN = 13;           // Status LED
const int GPIO_PIN6 = 6;          // Arcade motherboard Pin 6 (Press START)
const int GPIO_PIN5 = 5;           // Arcade motherboard Pin 5 (Press ACTIVE)

// PBT Sensor Configuration
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int AVG_SAMPLES = 4;
const int CALIBRATION_SAMPLES = 1000;
const int EXPECTED_BASELINE_MIN = 350;
const int EXPECTED_BASELINE_MAX = 450;

// Credit System
int creditCount = 0;
int pbtHitCount = 0;
int remainingStrikes = 0;
bool safetySystemActive = false;
int lastCreditState = HIGH;

// Individual LiDAR Status
bool tim100_detected = false;
bool tim150_detected = false;
int lastTim100State = LOW;
int lastTim150State = LOW;

// Credit debouncing
unsigned long lastCreditTime = 0;
const unsigned long CREDIT_DEBOUNCE_MS = 300; // 300ms debounce time
bool creditDebouncing = false;
unsigned long creditDebounceStart = 0;

// PBT Sensor Variables
int baselineOffset = 512;
bool calibrated = false;
int calCount = 0;
long calSum = 0;
unsigned long nextSample = 0;

// Alarm System (from original buzzer code)
unsigned long startTime = 0;
bool sirenActive = false;
const unsigned long sirenDuration = 5000; // 5 seconds

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
bool gpioCommandReady = false;

// Function declarations
void handleGPIOCommands();
void processGPIOCommand(String command);
void checkCreditInsertion();
void checkPBTMessage();
void updateSafetySystem();
void checkLidarTrigger();
void checkTim100Status();
void checkTim150Status();
void sendLidarStatus();
void runSiren();
void printDiagnostics();

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(PBT_PIN, INPUT);
  pinMode(CREDIT_PIN, INPUT_PULLUP);
  pinMode(TIM100_PIN, INPUT);  // TiM100 (Left) - no pull-up needed
  pinMode(TIM150_PIN, INPUT);  // TiM150 (Right) - no pull-up needed
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN6, OUTPUT);
  pinMode(GPIO_PIN5, OUTPUT);
  
  // Set initial states
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GPIO_PIN6, LOW);   // Pin 6 normally LOW
  digitalWrite(GPIO_PIN5, HIGH);  // Pin 5 normally HIGH
  
  analogReference(DEFAULT);
  nextSample = micros();
  lastStatsMs = millis();
  delay(100);
  
  Serial.println("# SICK7 Unified System - Credit Tracking + PBT + Alarm");
  Serial.println("# Credit System: 1 credit = 2 strikes");
  Serial.println("# Safety System: Only active when credits available");
  Serial.println("# CALIBRATING PBT SENSOR...");
}

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = millis();
  
  // Handle GPIO commands from Pi
  handleGPIOCommands();
  
  // Check for credit insertion
  checkCreditInsertion();
  
  // Check individual LiDAR status
  checkTim100Status();
  checkTim150Status();
  
  // Send LiDAR status to Pi every 100ms
  if (nowMs - lastStatsMs >= 100) {
    sendLidarStatus();
    lastStatsMs = nowMs;
  }
  
  // Check for PBT hits from Pi
  checkPBTMessage();
  
  // Update safety system status
  updateSafetySystem();
  
  // Check LiDAR trigger (only if safety system active)
  if (safetySystemActive) {
    checkLidarTrigger();
  }
  
  // Run siren if active
  runSiren();
  
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
        int measuredBaseline = calSum / calCount;
        baselineOffset = measuredBaseline;
        calibrated = true;
        
        Serial.print("# BASELINE MEASURED=");
        Serial.print(measuredBaseline);
        Serial.print(" ADC counts (");
        Serial.print((measuredBaseline * 5.0) / 1023.0, 2);
        Serial.println("V)");
        
        // Sanity check
        if (measuredBaseline < EXPECTED_BASELINE_MIN || 
            measuredBaseline > EXPECTED_BASELINE_MAX) {
          Serial.print("# WARNING: Baseline outside expected range ");
          Serial.print(EXPECTED_BASELINE_MIN);
          Serial.print("-");
          Serial.println(EXPECTED_BASELINE_MAX);
        } else {
          Serial.println("# Baseline looks good!");
        }
        
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
    
    // Normal operation - re-center around 512
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
    
    // Send data to Pi
    Serial.println(value);
  }
  
  // Print diagnostics every 5 seconds
  if (calibrated && nowMs - lastStatsMs >= 5000) {
    printDiagnostics();
    lastStatsMs = nowMs;
    minValue = 1023;
    maxValue = 0;
  }
}

void checkCreditInsertion() {
  static unsigned long lastDebounceTime = 0;
  static int lastStableState = HIGH;
  int currentState = digitalRead(CREDIT_PIN);
  
  // If the state has changed, reset the debounce timer
  if (currentState != lastStableState) {
    lastDebounceTime = millis();
    // Debug: Show state change
    Serial.print("# Credit pin state changed to: ");
    Serial.println(currentState == HIGH ? "HIGH" : "LOW");
  }
  
  // If enough time has passed since the last state change
  if ((millis() - lastDebounceTime) > CREDIT_DEBOUNCE_MS) {
    // If the current state is different from the last recorded state
    if (currentState != lastCreditState) {
      lastCreditState = currentState;
      
      // Only register credit on falling edge (HIGH to LOW)
      if (currentState == LOW) {
        // Valid credit insertion (debounced)
        creditCount++;
        remainingStrikes = (creditCount * 2) - pbtHitCount;
        safetySystemActive = (remainingStrikes > 0);
        
        Serial.print("# CREDIT INSERTED: Total=");
        Serial.print(creditCount);
        Serial.print(" Strikes=");
        Serial.print(remainingStrikes);
        Serial.print(" Safety=");
        Serial.println(safetySystemActive ? "ACTIVE" : "INACTIVE");
      }
    }
  }
  
  lastStableState = currentState;
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

void checkPBTMessage() {
  // This function is now handled by handleGPIOCommands()
  // Keeping for compatibility but redirecting to handleGPIOCommands
  handleGPIOCommands();
}

void processGPIOCommand(String command) {
  command.trim();
  
  if (command == "PBT_HIT") {
    pbtHitCount++;
    remainingStrikes = (creditCount * 2) - pbtHitCount;
    safetySystemActive = (remainingStrikes > 0);
    
    Serial.print("# PBT HIT: Total=");
    Serial.print(pbtHitCount);
    Serial.print(" Strikes=");
    Serial.print(remainingStrikes);
    Serial.print(" Safety=");
    Serial.println(safetySystemActive ? "ACTIVE" : "INACTIVE");
    
    if (remainingStrikes <= 0) {
      Serial.println("# GAME OVER - No strikes remaining");
    }
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
  else if (command == "LIDAR_TRIGGER") {
    // Simulate LiDAR trigger for safety system
    if (safetySystemActive) {
      sirenActive = true;
      startTime = millis();
      Serial.println("# LiDAR TRIGGER - Siren ON (Safety System Active)");
    } else {
      Serial.println("# LiDAR TRIGGER IGNORED - No credits remaining");
    }
  }
  else if (command == "RESET_CREDITS") {
    creditCount = 0;
    pbtHitCount = 0;
    remainingStrikes = 0;
    safetySystemActive = false;
    Serial.println("# CREDITS RESET - All counters cleared");
  }
  else if (command == "STATUS") {
    Serial.print("# STATUS: Credits=");
    Serial.print(creditCount);
    Serial.print(" PBT=");
    Serial.print(pbtHitCount);
    Serial.print(" Strikes=");
    Serial.print(remainingStrikes);
    Serial.print(" Safety=");
    Serial.println(safetySystemActive ? "ACTIVE" : "INACTIVE");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

void updateSafetySystem() {
  safetySystemActive = (remainingStrikes > 0);
}

void checkLidarTrigger() {
  // This function is now handled by individual TiM1xx checks
  // Keeping for compatibility but redirecting to new functions
  checkTim100Status();
  checkTim150Status();
}

void checkTim100Status() {
  int currentState = digitalRead(TIM100_PIN);
  
  // Detect rising edge (LOW to HIGH) - person detected
  if (currentState == HIGH && lastTim100State == LOW) {
    tim100_detected = true;
    Serial.println("🚨 ===== PERSON DETECTED - LEFT SIDE (TiM100) =====");
    Serial.println("# TiM100 DETECTED - Person on LEFT side");
  }
  // Detect falling edge (HIGH to LOW) - person cleared
  else if (currentState == LOW && lastTim100State == HIGH) {
    tim100_detected = false;
    Serial.println("✅ ===== AREA CLEAR - LEFT SIDE (TiM100) =====");
    Serial.println("# TiM100 CLEAR - LEFT side clear");
  }
  
  lastTim100State = currentState;
}

void checkTim150Status() {
  int currentState = digitalRead(TIM150_PIN);
  
  // Detect rising edge (LOW to HIGH) - person detected
  if (currentState == HIGH && lastTim150State == LOW) {
    tim150_detected = true;
    Serial.println("🚨 ===== PERSON DETECTED - RIGHT SIDE (TiM150) =====");
    Serial.println("# TiM150 DETECTED - Person on RIGHT side");
  }
  // Detect falling edge (HIGH to LOW) - person cleared
  else if (currentState == LOW && lastTim150State == HIGH) {
    tim150_detected = false;
    Serial.println("✅ ===== AREA CLEAR - RIGHT SIDE (TiM150) =====");
    Serial.println("# TiM150 CLEAR - RIGHT side clear");
  }
  
  lastTim150State = currentState;
}

void sendLidarStatus() {
  Serial.print("# LIDAR_STATUS: TIM100=");
  Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
  Serial.print(" TIM150=");
  Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
}

void runSiren() {
  // Run siren for 5 seconds (only if safety system is active)
  if (sirenActive && safetySystemActive) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed < sirenDuration) {
      int freq = map((elapsed / 10) % 100, 0, 99, 500, 1000); // sweeping tone
      tone(BUZZER_PIN, freq);
      digitalWrite(LED_PIN, (elapsed / 200) % 2); // blink LED
    } else {
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
      sirenActive = false;
      Serial.println("# Siren OFF");
    }
  } else if (sirenActive && !safetySystemActive) {
    // Safety system deactivated - stop siren immediately
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    sirenActive = false;
    Serial.println("# Siren STOPPED - No credits remaining");
  }
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
  
  Serial.print("# CREDITS: Total=");
  Serial.print(creditCount);
  Serial.print(" PBT=");
  Serial.print(pbtHitCount);
  Serial.print(" Strikes=");
  Serial.print(remainingStrikes);
  Serial.print(" Safety=");
  Serial.println(safetySystemActive ? "ACTIVE" : "INACTIVE");
  
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
// Credit Signal: Pin 2 (with pull-up resistor)
// LiDAR Trigger: Pin 8
// Buzzer: Pin 9
// LED: Pin 13
// Arcade GPIO: Pins 5, 6
//
// Credit System Logic:
// - 1 credit = 2 strikes (PBT hits)
// - Safety system only active when strikes > 0
// - Alarm only sounds when safety system is active
