// LiDAR Detection System - Second Arduino (Analog Input Version with Hysteresis)
// Monitors OR gate input from TiM100, TiM150, and TiM240 detection circuit
// Uses ANALOG INPUT with HYSTERESIS to reliably detect 0.8V signals while filtering noise
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int INPUT_PIN = A0;           // Analog input (TiM100 OR TiM150 OR TiM240)
const int BUZZER_PIN = 9;           // Piezo buzzer
const int LED_PIN = 13;              // Status LED

// Analog voltage thresholds with hysteresis
// Arduino ADC: 0-1023 for 0-5V
// Hysteresis prevents false triggers from noise while allowing TiM240's 0.8V to trigger
const int THRESHOLD_HIGH = 184;  // ~0.9V - activate detection (prevents noise false triggers)
const int THRESHOLD_LOW = 143;   // ~0.7V - deactivate detection (allows TiM240 0.8V to trigger)
const unsigned long CONFIRMATION_MS = 50;  // Require stable signal for 50ms before triggering

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
  
  // Initialize pin state tracking based on actual analog reading
  int initialVoltage = analogRead(INPUT_PIN);
  int initialPinState = (initialVoltage > THRESHOLD_HIGH) ? HIGH : LOW;
  wasLowLastCycle = (initialPinState == LOW);
  if (wasLowLastCycle) {
    continuousLowStartMs = millis();  // Pin starts LOW
  }
  detectionStartTime = 0;  // Initialize detection timer
  
  Serial.println("# LiDAR Detection System - Arduino Ready (Analog Input with Hysteresis)");
  Serial.println("# OR Gate Input: Analog Pin A0 (TiM100 OR TiM150 OR TiM240)");
  Serial.print("# Activation Threshold: ");
  Serial.print(THRESHOLD_HIGH);
  Serial.print(" ADC (");
  Serial.print((THRESHOLD_HIGH * 5.0 / 1023.0), 2);
  Serial.println("V) - prevents noise false triggers");
  Serial.print("# Deactivation Threshold: ");
  Serial.print(THRESHOLD_LOW);
  Serial.print(" ADC (");
  Serial.print((THRESHOLD_LOW * 5.0 / 1023.0), 2);
  Serial.println("V) - allows TiM240 0.8V to trigger");
  Serial.print("# Confirmation Time: ");
  Serial.print(CONFIRMATION_MS);
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

void loop() {
  // Read analog voltage (0-1023 for 0-5V)
  int rawVoltage = analogRead(INPUT_PIN);
  
  unsigned long now = millis();
  
  // Handle serial commands from Pi
  handleSerialCommands();
  
  // HYSTERESIS LOGIC WITH TIME-BASED CONFIRMATION
  // Use different thresholds for activation vs deactivation to prevent false triggers
  
  if (!person_detected) {
    // NOT DETECTING: Require HIGH threshold to activate (prevents noise false triggers)
    if (rawVoltage > THRESHOLD_HIGH) {
      // Signal above activation threshold
      if (detectionStartTime == 0) {
        // Start timing - signal just crossed threshold
        detectionStartTime = now;
      } else if ((now - detectionStartTime) >= CONFIRMATION_MS) {
        // Confirmed: stable signal above threshold for confirmation time
        person_detected = true;
        detectionStartTime = 0;  // Reset timer
        triggerAlarm();
        sendStatusToPi();
        Serial.println("✅ Person detected - Confirmed after stable signal");
      }
      // If not confirmed yet, keep waiting (don't reset timer)
    } else {
      // Signal dropped below threshold before confirmation
      detectionStartTime = 0;  // Reset timer
    }
  } else {
    // DETECTING: Use LOW threshold to deactivate (easier to clear, allows TiM240 0.8V to work)
    if (rawVoltage < THRESHOLD_LOW) {
      // Signal below deactivation threshold
      if (detectionStartTime == 0) {
        // Start timing - signal just dropped below threshold
        detectionStartTime = now;
      } else if ((now - detectionStartTime) >= CONFIRMATION_MS) {
        // Confirmed: stable low signal for confirmation time
        person_detected = false;
        detectionStartTime = 0;  // Reset timer
        // Ensure alarm is stopped (clean state)
        if (alarmActive) {
          alarmActive = false;
          noTone(BUZZER_PIN);
          digitalWrite(LED_PIN, LOW);
        }
        Serial.println("✅ Area clear - Person left (confirmed after stable low signal)");
        sendStatusToPi();
      }
      // If not confirmed yet, keep waiting (don't reset timer)
    } else {
      // Signal still above deactivation threshold
      detectionStartTime = 0;  // Reset timer
    }
  }
  
  // Convert to digital level for compatibility with existing tracking code
  // Use HIGH threshold for general tracking purposes
  int rawLevel = (rawVoltage > THRESHOLD_HIGH) ? HIGH : LOW;
  
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
  // This ensures clean state even if hysteresis logic missed the transition
  if (rawVoltage < THRESHOLD_LOW && person_detected && continuousLowStartMs > 0 && (now - continuousLowStartMs) > 200) {
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
  if (!alarmActive && rawVoltage < THRESHOLD_LOW && person_detected) {
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
//   - TiM240 Signal → OR Gate Input 3 (from Pi via GPIO, ~0.8V when HIGH, but OR gate may amplify)
// Piezo Buzzer → Pin 9
// Status LED → Pin 13 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors OR gate input via analog pin A0
// - Uses HYSTERESIS: Different thresholds for activation vs deactivation
//   * Activation: Requires >0.9V (184 ADC) for 50ms (prevents noise false triggers)
//   * Deactivation: Requires <0.7V (143 ADC) for 50ms (allows TiM240 0.8V to trigger)
// - Triggers 5-second alarm when ANY LiDAR detects person
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing
//
// Hysteresis Benefits:
// - Prevents false triggers from noise (higher activation threshold)
// - Allows TiM240's 0.8V to trigger (lower deactivation threshold)
// - Time-based confirmation (50ms) filters out brief spikes
// - Adjust thresholds if needed:
//   * Higher THRESHOLD_HIGH = less sensitive, fewer false triggers
//   * Lower THRESHOLD_LOW = easier to clear, more responsive
