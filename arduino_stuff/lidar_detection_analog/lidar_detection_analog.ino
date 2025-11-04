// LiDAR Detection System - Second Arduino (Analog Input Version)
// Monitors OR gate input from TiM100, TiM150, and TiM240 detection circuit
// Uses ANALOG INPUT to detect 0.8V signals (TiM240 GPIO output)
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int INPUT_PIN = A0;           // Analog input (TiM100 OR TiM150 OR TiM240)
const int BUZZER_PIN = 9;           // Piezo buzzer
const int LED_PIN = 13;              // Status LED

// Analog voltage threshold (0.8V detection)
// Arduino ADC: 0-1023 for 0-5V
// 0.8V = (0.8/5.0) * 1023 ≈ 164 ADC counts
// Using 165 for reliable detection with some margin
const int VOLTAGE_THRESHOLD = 165;  // ~0.8V threshold

// Detection state
bool person_detected = false;

// Alarm system (based on your existing code)
unsigned long alarmStartTime = 0;
bool alarmActive = false;
const unsigned long alarmDuration = 5000; // 5 seconds

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
  
  // Initialize pin state tracking based on actual analog reading
  int initialVoltage = analogRead(INPUT_PIN);
  int initialPinState = (initialVoltage > VOLTAGE_THRESHOLD) ? HIGH : LOW;
  wasLowLastCycle = (initialPinState == LOW);
  if (wasLowLastCycle) {
    continuousLowStartMs = millis();  // Pin starts LOW
  }
  
  Serial.println("# LiDAR Detection System - Arduino Ready (Analog Input)");
  Serial.println("# OR Gate Input: Analog Pin A0 (TiM100 OR TiM150 OR TiM240)");
  Serial.print("# Voltage Threshold: ");
  Serial.print(VOLTAGE_THRESHOLD);
  Serial.print(" ADC (~");
  Serial.print((VOLTAGE_THRESHOLD * 5.0 / 1023.0), 2);
  Serial.println("V)");
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

void loop() {
  // Read analog voltage (0-1023 for 0-5V)
  int rawVoltage = analogRead(INPUT_PIN);
  
  // Convert analog reading to digital level based on threshold
  // Voltage > 0.8V (threshold) = HIGH = person detected
  int rawLevel = (rawVoltage > VOLTAGE_THRESHOLD) ? HIGH : LOW;
  
  unsigned long now = millis();
  
  // Handle serial commands from Pi
  handleSerialCommands();
  
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
  
  // Debounce: track candidate level and accept only if stable for debounceMs
  if (rawLevel != candidateLevel) {
    candidateLevel = rawLevel;
    lastToggleMs = now;
  }
  
  if ((now - lastToggleMs) >= debounceMs && stableLevel != candidateLevel) {
    stableLevel = candidateLevel;
    bool currentDetection = (stableLevel == HIGH);  // HIGH = person detected
    
    // Edge actions on debounced signal
    if (currentDetection && !person_detected) {
      person_detected = true;
      triggerAlarm();
      // Immediately send status on detection
      sendStatusToPi();
    } else if (!currentDetection && person_detected) {
      // Person left - force clear state
      person_detected = false;
      // Ensure alarm is stopped (clean state)
      if (alarmActive) {
        alarmActive = false;
        noTone(BUZZER_PIN);
        digitalWrite(LED_PIN, LOW);
      }
      Serial.println("✅ Area clear - Person left");
      // Immediately send status on clear
      sendStatusToPi();
    }
  }
  
  // Additional safety: If pin has been continuously LOW for extended period, force clear
  // Check rawLevel (actual pin state) not stableLevel (debounced state)
  // This ensures clean state even if debounce logic missed the transition
  // Use continuousLowStartMs to track when pin started being continuously LOW
  if (rawLevel == LOW && person_detected && continuousLowStartMs > 0 && (now - continuousLowStartMs) > 200) {
    // Force clear regardless of debounce state - pin has been continuously LOW for 200ms
    person_detected = false;
    stableLevel = LOW;      // Sync debounce state to match reality
    candidateLevel = LOW;   // Sync candidate to match reality
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
  if (!alarmActive && rawLevel == LOW && person_detected) {
    person_detected = false;
    stableLevel = LOW;
    candidateLevel = LOW;
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
    // Also print current voltage for debugging
    int voltage = analogRead(INPUT_PIN);
    Serial.print("# Current voltage: ");
    Serial.print(voltage);
    Serial.print(" ADC (");
    Serial.print((voltage * 5.0 / 1023.0), 2);
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
//   - TiM240 Signal → OR Gate Input 3 (from Pi via GPIO, ~0.8V when HIGH)
// Piezo Buzzer → Pin 9
// Status LED → Pin 13 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors OR gate input via analog pin A0
// - Detects voltage > 0.8V (165 ADC counts) as person detection
// - Triggers 5-second alarm when ANY LiDAR detects person
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing
//
// Voltage Threshold:
// - 0.8V = 164 ADC counts (theoretical)
// - Using 165 ADC counts for reliable detection with margin
// - Adjust VOLTAGE_THRESHOLD if needed (higher = less sensitive, lower = more sensitive)
