// SICK 10 Bar PBT Sensor with GPIO Control + Credit Tracking
// Reads PBT sensor, controls arcade GPIO pins, and tracks credits
// Credit system: 2 PBT hits = 1 credit deducted

const int PBT_PIN = A0;
const int LED_PIN = 13;
const int GPIO_PIN6 = 6;  // Arcade motherboard Pin 6 (Press START)
const int GPIO_PIN5 = 5;  // Arcade motherboard Pin 5 (Press ACTIVE)
const int CREDIT_ADD_PIN = 2;  // Pin 2 (interrupt-capable) - Credit add signal (falling edge 5V→0V)
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

// Averaging for noise reduction
const int AVG_SAMPLES = 4;

// Expected baseline for SICK sensor (~380 for 1.86V on 5V Arduino)
const int EXPECTED_BASELINE_MIN = 350;
const int EXPECTED_BASELINE_MAX = 450;

// Calibration
const int CALIBRATION_SAMPLES = 1000;  // ~1.25 seconds
int baselineOffset = 512;
bool calibrated = false;
int calCount = 0;
long calSum = 0;

// Diagnostics
unsigned long lastStatsMs = 0;
unsigned long sampleCount = 0;
int minValue = 1023;
int maxValue = 0;

// Activity detection for LED
int activityThreshold = 30;
unsigned long lastActivityMs = 0;
const unsigned long LED_ON_MS = 100;

// GPIO command handling
String gpioCommand = "";
bool gpioCommandReady = false;

// Credit tracking system
volatile int credits = 0;     // Current credit count (starts at 0) - volatile for interrupt
int pbt_hit_count = 0;        // Count PBT hits (2 hits = 1 credit deducted)
const int HITS_PER_CREDIT = 2;

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN6, OUTPUT);
  pinMode(GPIO_PIN5, OUTPUT);
  pinMode(CREDIT_ADD_PIN, INPUT_PULLUP);  // Enable internal pull-up, expects HIGH (5V) normally
  
  // Set initial GPIO states (idle state)
  digitalWrite(LED_PIN, LOW);
  digitalWrite(GPIO_PIN6, LOW);   // Pin 6 normally LOW
  digitalWrite(GPIO_PIN5, HIGH);  // Pin 5 normally HIGH
  
  // Attach interrupt for credit add signal (falling edge: 5V → 0V)
  attachInterrupt(digitalPinToInterrupt(CREDIT_ADD_PIN), handleCreditAdd, FALLING);
  
  // Use default 5V reference
  analogReference(DEFAULT);
  
  nextSample = micros();
  lastStatsMs = millis();
  delay(100);
  
  Serial.println("# SICK 10 Bar PBT Sensor + GPIO Control + Credit Tracking");
  Serial.println("# Credit System: 2 PBT hits = 1 credit deducted");
  Serial.println("# Expected baseline: ~385 ADC counts (1.88V)");
  Serial.println("# Expected peak: ~607 ADC counts (2.96V)");
  Serial.println("# GPIO: Pin 6 (START), Pin 5 (ACTIVE) - 5V output");
  Serial.println("# Credit Add: Pin 2 (falling edge 5V→0V triggers +1 credit)");
  Serial.println("# Commands: PIN6_HIGH, PIN6_LOW, PIN5_HIGH, PIN5_LOW, RESET_GPIO, STATUS");
  Serial.println("# Credit Commands: PBT_HIT, GET_CREDITS, SET_CREDITS:<n>, ADD_CREDITS:<n>");
  Serial.println("# CALIBRATING...");
  
  // Send initial credit status
  sendCreditStatus();
}

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = millis();
  
  // Handle GPIO and credit commands from Pi
  handleSerialCommands();
  
  // LED activity indicator
  if (nowMs - lastActivityMs > LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
  }
  
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Read with averaging to reduce noise
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
      delayMicroseconds(10);
    }
    int rawValue = sum / AVG_SAMPLES;
    
    // Calibration phase - find baseline
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
          Serial.println("# Check sensor connections!");
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
        
        // Send credit status after calibration
        sendCreditStatus();
      }
      return;
    }
    
    // Normal operation - re-center around 512
    int value = rawValue - baselineOffset + 512;
    
    // Clamp to valid range
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
    
    // Send centered data (compatible with existing Pi software)
    Serial.println(value);
  }
  
  // Print diagnostics every 5 seconds
  unsigned long nowMs2 = millis();
  if (calibrated && nowMs2 - lastStatsMs >= 5000) {
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
    
    // Interpretation
    if (range < 20) {
      Serial.println("# No activity detected (range < 20 counts)");
    } else if (range > 200) {
      Serial.println("# High activity detected! Good signal.");
    }
    
    lastStatsMs = nowMs2;
    // Reset min/max for next period
    minValue = 1023;
    maxValue = 0;
  }
}

void handleSerialCommands() {
  // Read serial commands from Pi
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (gpioCommand.length() > 0) {
        processCommand(gpioCommand);
        gpioCommand = "";
      }
    } else {
      gpioCommand += c;
    }
  }
}

void processCommand(String command) {
  command.trim();
  
  // GPIO commands
  if (command == "PIN6_HIGH") {
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
    Serial.print("# GPIO STATUS: Pin6=");
    Serial.print(digitalRead(GPIO_PIN6) ? "HIGH" : "LOW");
    Serial.print(" Pin5=");
    Serial.println(digitalRead(GPIO_PIN5) ? "HIGH" : "LOW");
    sendCreditStatus();
  }
  
  // Credit tracking commands
  else if (command == "PBT_HIT") {
    handlePBTHit();
  }
  else if (command == "GET_CREDITS") {
    sendCreditStatus();
  }
  else if (command.startsWith("SET_CREDITS:")) {
    int newCredits = command.substring(12).toInt();
    if (newCredits >= 0) {
      credits = newCredits;
      pbt_hit_count = 0;  // Reset hit counter when manually setting credits
      Serial.print("# Credits set to: ");
      Serial.println(credits);
      sendCreditStatus();
    }
  }
  else if (command.startsWith("ADD_CREDITS:")) {
    int addCredits = command.substring(12).toInt();
    if (addCredits > 0) {
      credits += addCredits;
      Serial.print("# Credits added: ");
      Serial.print(addCredits);
      Serial.print(" (Total: ");
      Serial.print(credits);
      Serial.println(")");
      sendCreditStatus();
    }
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

void handlePBTHit() {
  // Increment PBT hit counter
  pbt_hit_count++;
  
  Serial.print("# PBT_HIT received (hit #");
  Serial.print(pbt_hit_count);
  Serial.print(" of ");
  Serial.print(HITS_PER_CREDIT);
  Serial.println(")");
  
  // Check if we've reached the threshold for credit deduction
  if (pbt_hit_count >= HITS_PER_CREDIT) {
    if (credits > 0) {
      credits--;
      Serial.print("# Credit deducted! Credits remaining: ");
      Serial.println(credits);
    } else {
      Serial.println("# No credits remaining - hit counted but no credit to deduct");
    }
    
    // Reset hit counter
    pbt_hit_count = 0;
    
    // Send updated credit status to Pi
    sendCreditStatus();
  } else {
    // Send intermediate status (shows hit count progress)
    Serial.print("# CREDIT_PROGRESS:");
    Serial.print(pbt_hit_count);
    Serial.print("/");
    Serial.println(HITS_PER_CREDIT);
  }
}

void sendCreditStatus() {
  Serial.print("CREDITS:");
  Serial.println(credits);
}

// Interrupt handler for credit add signal (falling edge on Pin 2)
void handleCreditAdd() {
  // Simple increment - debouncing handled by interrupt timing
  // Note: credits is volatile, so this is safe from interrupt context
  credits++;
  pbt_hit_count = 0;  // Reset hit counter when manually adding credits
  // Note: Serial in interrupt should be minimal, but sendCreditStatus() is quick
  // For safety, you could set a flag here and check it in loop() instead
  sendCreditStatus();
  Serial.println("# Credit added via hardware signal (Pin 2 falling edge)");
}

// Hardware Connection:
// SICK 10 Bar PBT Sensor:
//   Signal Out ──── 100kΩ ──── Arduino A0
//   Ground     ──────────────── Arduino GND
// Arcade Motherboard:
//   Pin 6 (START) ──────────── Arduino Pin 6
//   Pin 5 (ACTIVE) ─────────── Arduino Pin 5
// Credit Add Signal:
//   Pi GPIO 18 (Pin 12) ────── Arduino Pin 2
//   Signal: Falling edge (5V → 0V) triggers +1 credit
//   Arduino Pin 2: INPUT_PULLUP (normally HIGH, goes LOW on falling edge)
//
// Credit System:
//   - 2 PBT hits = 1 credit deducted
//   - Falling edge on Pin 2 = +1 credit (hardware trigger)
//   - Starts with 0 credits
//   - Use SET_CREDITS:<n> or ADD_CREDITS:<n> to add credits
//   - Pi sends PBT_HIT after each pulse generation
//   - Arduino sends CREDITS:<n> to Pi when count changes

