// LiDAR Detection System - Second Arduino (Analog Input with Dual-Path Detection)
// Monitors OR gate input from TiM100, TiM150, and TiM240 detection circuit
// Uses DUAL-PATH DETECTION: Fast path for strong signals (5V), careful path for weak signals (0.8V)
// Includes MOVING AVERAGE FILTERING to smooth noise
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int INPUT_PIN = A0;           // Analog input (TiM100 OR TiM150 OR TiM240)
const int BUZZER_PIN = 9;           // Piezo buzzer
const int LED_PIN = 13;              // Status LED

// Dual-path detection thresholds
// Arduino ADC: 0-1023 for 0-5V
// Strong signal path: Side LiDARs (TiM100/TiM150) output 5V
// Weak signal path: Top LiDAR (TiM240) outputs 0.8-1V
const int STRONG_THRESHOLD = 400;    // ~2V - clearly above noise, fast detection for 5V signals
const unsigned long STRONG_CONFIRMATION_MS = 10;  // Fast confirmation for strong signals

// Weak signal path (TiM240) - hysteresis to handle 0.8-1V signals
const int WEAK_THRESHOLD_HIGH = 165; // ~0.8V - activate detection
const int WEAK_THRESHOLD_LOW = 143;  // ~0.7V - deactivate detection
const unsigned long WEAK_CONFIRMATION_MS = 100;  // Longer confirmation for weak signals

// Moving average filter
const int SAMPLE_COUNT = 5;          // Number of samples for averaging
int voltageBuffer[SAMPLE_COUNT] = {0};
int bufferIndex = 0;
bool bufferInitialized = false;

// Detection state
bool person_detected = false;

// Alarm system (based on your existing code)
unsigned long alarmStartTime = 0;
bool alarmActive = false;
const unsigned long alarmDuration = 5000; // 5 seconds

// Time-based confirmation for detection
unsigned long detectionStartTime = 0;  // When signal first exceeded threshold

// Input debounce/hysteresis (software only)
const unsigned long debounceMs = 50;   // Require stable level for 50 ms
int stableLevel = LOW;                 // Idle is LOW (0.4 V or less)
int candidateLevel = LOW;              // Last observed level
unsigned long lastToggleMs = 0;        // Time when candidate changed
unsigned long lastHighMs = 0;          // Time when pin was last HIGH (for force clear)
unsigned long continuousLowStartMs = 0; // When pin started being continuously LOW
bool wasLowLastCycle = false;          // Track if pin was LOW last loop

// Serial communication with Pi
String serialCommand = "";
bool commandReady = false;

// Status reporting
unsigned long lastStatusReport = 0;
const unsigned long statusInterval = 1000; // Report every 1 second

void setup() {
  Serial.begin(9600);  // Match your working code baud rate
  
  // Pin setup
  pinMode(INPUT_PIN, INPUT);    // Analog pin (A0-A5 can be used as INPUT)
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize states
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  lastHighMs = millis();  // Initialize to current time (pin starts LOW)
  
  // Initialize moving average buffer
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    voltageBuffer[i] = analogRead(INPUT_PIN);
    delay(5);  // Small delay between samples
  }
  bufferInitialized = true;
  bufferIndex = 0;
  
  // Initialize pin state tracking
  int initialVoltage = calculateMovingAverage();
  wasLowLastCycle = (initialVoltage < STRONG_THRESHOLD && initialVoltage < WEAK_THRESHOLD_HIGH);
  if (wasLowLastCycle) {
    continuousLowStartMs = millis();  // Pin starts LOW
  }
  detectionStartTime = 0;  // Initialize detection timer
  
  Serial.println("# LiDAR Detection System - Arduino Ready (Dual-Path Detection)");
  Serial.println("# OR Gate Input: Analog Pin A0 (TiM100 OR TiM150 OR TiM240)");
  Serial.println("# Moving Average Filter: 5 samples");
  Serial.println("# --- Strong Signal Path (Side LiDARs: 5V) ---");
  Serial.print("#   Threshold: ");
  Serial.print(STRONG_THRESHOLD);
  Serial.print(" ADC (");
  Serial.print((STRONG_THRESHOLD * 5.0 / 1023.0), 2);
  Serial.print("V), Confirmation: ");
  Serial.print(STRONG_CONFIRMATION_MS);
  Serial.println(" ms");
  Serial.println("# --- Weak Signal Path (Top LiDAR: 0.8-1V) ---");
  Serial.print("#   Activation: ");
  Serial.print(WEAK_THRESHOLD_HIGH);
  Serial.print(" ADC (");
  Serial.print((WEAK_THRESHOLD_HIGH * 5.0 / 1023.0), 2);
  Serial.print("V), Deactivation: ");
  Serial.print(WEAK_THRESHOLD_LOW);
  Serial.print(" ADC (");
  Serial.print((WEAK_THRESHOLD_LOW * 5.0 / 1023.0), 2);
  Serial.print("V), Confirmation: ");
  Serial.print(WEAK_CONFIRMATION_MS);
  Serial.println(" ms");
  Serial.println("# Alarm: Buzzer Pin 9, LED Pin 13");
  Serial.println("# Commands: STATUS, RESET_ALARM, SIMULATE_DETECTION, SIMULATE_CLEAR");
  Serial.println("# READY");
  
  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// Calculate moving average of voltage readings
int calculateMovingAverage() {
  int sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += voltageBuffer[i];
  }
  return sum / SAMPLE_COUNT;
}

void loop() {
  // Read analog voltage and add to moving average buffer
  voltageBuffer[bufferIndex] = analogRead(INPUT_PIN);
  bufferIndex = (bufferIndex + 1) % SAMPLE_COUNT;
  
  // Calculate filtered voltage (moving average smooths noise)
  int filteredVoltage = calculateMovingAverage();
  
  unsigned long now = millis();
  
  // Handle serial commands from Pi
  handleSerialCommands();
  
  // DUAL-PATH DETECTION WITH TIME-BASED CONFIRMATION
  // Path 1: Strong signals (5V from side LiDARs) - fast detection
  // Path 2: Weak signals (0.8-1V from TiM240) - careful hysteresis
  
  if (!person_detected) {
    // NOT DETECTING: Check for strong signal first (fast path)
    if (filteredVoltage > STRONG_THRESHOLD) {
      // Strong signal detected (5V from side LiDARs)
      if (detectionStartTime == 0) {
        detectionStartTime = now;
      } else if ((now - detectionStartTime) >= STRONG_CONFIRMATION_MS) {
        // Confirmed: strong signal stable for short confirmation time
        person_detected = true;
        detectionStartTime = 0;
        triggerAlarm();
        sendStatusToPi();
        Serial.println("✅ Person detected - Strong signal (side LiDAR)");
      }
    }
    // Then check for weak signal (hysteresis path)
    else if (filteredVoltage > WEAK_THRESHOLD_HIGH) {
      // Weak signal above activation threshold (TiM240)
      if (detectionStartTime == 0) {
        detectionStartTime = now;
      } else if ((now - detectionStartTime) >= WEAK_CONFIRMATION_MS) {
        // Confirmed: weak signal stable for longer confirmation time
        person_detected = true;
        detectionStartTime = 0;
        triggerAlarm();
        sendStatusToPi();
        Serial.println("✅ Person detected - Weak signal (top LiDAR)");
      }
    } else {
      // Signal below both thresholds - reset timer
      detectionStartTime = 0;
    }
  } else {
    // DETECTING: Check which path to use for deactivation
    if (filteredVoltage > STRONG_THRESHOLD) {
      // Still strong signal - keep detecting, reset timer
      detectionStartTime = 0;
    } else if (filteredVoltage < WEAK_THRESHOLD_LOW) {
      // Signal dropped below weak deactivation threshold
      if (detectionStartTime == 0) {
        detectionStartTime = now;
      } else if ((now - detectionStartTime) >= WEAK_CONFIRMATION_MS) {
        // Confirmed: stable low signal for confirmation time
        person_detected = false;
        detectionStartTime = 0;
        // Ensure alarm is stopped (clean state)
        if (alarmActive) {
          alarmActive = false;
          noTone(BUZZER_PIN);
          digitalWrite(LED_PIN, LOW);
        }
        Serial.println("✅ Area clear - Person left (confirmed after stable low signal)");
        sendStatusToPi();
      }
    } else {
      // Signal between thresholds - reset timer (might be transitioning)
      detectionStartTime = 0;
    }
  }
  
  // Convert to digital level for compatibility with existing tracking code
  // Use strong threshold for general tracking
  int rawLevel = (filteredVoltage > STRONG_THRESHOLD || filteredVoltage > WEAK_THRESHOLD_HIGH) ? HIGH : LOW;
  
  // Track when pin was last HIGH (for robust force clear)
  if (rawLevel == HIGH) {
    lastHighMs = now;
    continuousLowStartMs = 0;  // Reset continuous LOW timer
    wasLowLastCycle = false;
  }
  
  // Track continuous LOW state (for force clear)
  if (rawLevel == LOW) {
    if (!wasLowLastCycle) {
      // Pin just went LOW - start timer
      continuousLowStartMs = now;
      wasLowLastCycle = true;
    }
  } else {
    // Pin is HIGH - reset tracking
    wasLowLastCycle = false;
    continuousLowStartMs = 0;
  }
  
  // Additional safety: If pin has been continuously LOW for extended period, force clear
  // This ensures clean state even if dual-path logic missed the transition
  if (filteredVoltage < WEAK_THRESHOLD_LOW && person_detected && continuousLowStartMs > 0 && (now - continuousLowStartMs) > 200) {
    // Force clear regardless of confirmation state - pin has been continuously LOW for 200ms
    person_detected = false;
    detectionStartTime = 0;
    continuousLowStartMs = 0;  // Reset timer
    wasLowLastCycle = false;
    if (alarmActive) {
      alarmActive = false;
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
    }
    Serial.println("✅ Area clear - Forced by continuous LOW");
    sendStatusToPi();
  }
  
  // Extra safety: If alarm finished and pin is LOW, immediately clear (no delay)
  // This handles the case where alarm completes but person_detected wasn't cleared
  if (!alarmActive && filteredVoltage < WEAK_THRESHOLD_LOW && person_detected) {
    person_detected = false;
    detectionStartTime = 0;
    continuousLowStartMs = 0;
    wasLowLastCycle = false;
    Serial.println("✅ Area clear - Alarm finished, pin LOW");
    sendStatusToPi();
  }
  
  // Run alarm system (exactly like your working code)
  runAlarmSystem();
  
  // Send status to Pi every second
  if (now - lastStatusReport >= statusInterval) {
    sendStatusToPi();
    lastStatusReport = now;
  }
  
  // Optional: Debug output (comment out for production)
  // Uncomment to see raw voltage readings
  /*
  static unsigned long lastDebug = 0;
  if (now - lastDebug >= 500) {
    Serial.print("# DEBUG: Voltage=");
    Serial.print(rawVoltage);
    Serial.print(" ADC (");
    Serial.print((rawVoltage * 5.0 / 1023.0), 2);
    Serial.print("V), Level=");
    Serial.println(rawLevel == HIGH ? "HIGH" : "LOW");
    lastDebug = now;
  }
  */
}


void triggerAlarm() {
  alarmActive = true;
  alarmStartTime = millis();
  
  Serial.println("Trigger detected - Siren ON");
  
  // Send detection signal to Pi
  Serial.println("PERSON_DETECTED");
}

void runAlarmSystem() {
  // Run siren for 5 seconds (exactly like your working code)
  if (alarmActive) {
    unsigned long elapsed = millis() - alarmStartTime; // to find the amount of time that has passed since siren is activated
    if (elapsed < alarmDuration) { // if time that has passed since siren started < 5s
      int freq = map((elapsed / 10) % 100, 0, 99, 500, 1000); // sweeping tone
      tone(BUZZER_PIN, freq);
      digitalWrite(LED_PIN, (elapsed / 200) % 2); // blink LED
    } else {
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
      alarmActive = false;
      Serial.println("Siren OFF");
    }
  }
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (serialCommand.length() > 0) {
        processCommand(serialCommand);
        serialCommand = "";
      }
    } else {
      serialCommand += c;
    }
  }
}

void processCommand(String command) {
  command.trim();
  
  if (command == "STATUS") {
    sendStatusToPi();
    // Also print current voltage for debugging (both raw and filtered)
    int rawVoltage = analogRead(INPUT_PIN);
    int filteredVoltage = calculateMovingAverage();
    Serial.print("# Raw voltage: ");
    Serial.print(rawVoltage);
    Serial.print(" ADC (");
    Serial.print((rawVoltage * 5.0 / 1023.0), 2);
    Serial.print("V), Filtered: ");
    Serial.print(filteredVoltage);
    Serial.print(" ADC (");
    Serial.print((filteredVoltage * 5.0 / 1023.0), 2);
    Serial.println("V)");
  }
  else if (command == "RESET_ALARM") {
    alarmActive = false;
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    Serial.println("# Alarm reset manually");
  }
  else if (command == "SIMULATE_DETECTION") {
    // Simulate detection for testing
    person_detected = true;
    triggerAlarm();
    Serial.println("# Simulated detection triggered");
  }
  else if (command == "SIMULATE_CLEAR") {
    // Simulate clear for testing
    person_detected = false;
    Serial.println("# Simulated clear triggered");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

void sendStatusToPi() {
  Serial.print("LIDAR_STATUS:");
  Serial.print(person_detected ? "1" : "0");  // Any LiDAR detecting
  Serial.print(",");
  Serial.print(alarmActive ? "1" : "0");      // Alarm active
  Serial.println();
}

// Hardware Connections:
// OR Gate Output → Analog Pin A0 (with pull-down resistor to GND)
//   - TiM100 Signal → OR Gate Input 1
//   - TiM150 Signal → OR Gate Input 2  
//   - TiM240 Signal → OR Gate Input 3 (from Pi via GPIO, ~0.8V when HIGH, but OR gate may amplify)
// Piezo Buzzer → Pin 9
// Status LED → Pin 13 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors OR gate input via analog pin A0
// - Uses MOVING AVERAGE FILTER (5 samples) to smooth noise
// - Uses DUAL-PATH DETECTION:
//   * Strong Signal Path (Side LiDARs: 5V):
//     - Threshold: >2V (400 ADC) for fast detection
//     - Confirmation: 10ms (fast response)
//   * Weak Signal Path (Top LiDAR: 0.8-1V):
//     - Activation: >0.8V (165 ADC) with hysteresis
//     - Deactivation: <0.7V (143 ADC)
//     - Confirmation: 100ms (careful filtering)
// - Triggers 5-second alarm when ANY LiDAR detects person
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing
//
// Dual-Path Benefits:
// - Fast response for strong signals (5V side LiDARs)
// - Robust detection for weak signals (0.8V top LiDAR)
// - Moving average smooths noise spikes
// - Hysteresis prevents oscillation
// - Different confirmation times optimized for each signal type
