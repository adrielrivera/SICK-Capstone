// LiDAR Detection System - Dual Input Arduino
// Input 1: OR gate from side LiDARs (TiM100/TiM150) - Analog Pin A0 (5V signal)
// Input 2: TiM240 from Pi GPIO - Digital Pin 3 (3.3V signal)
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int OR_GATE_PIN = A0;        // Analog input - OR gate (TiM100 OR TiM150, outputs 5V)
const int TIM240_PIN = 3;          // Digital input - TiM240 from Pi GPIO (3.3V HIGH when detected)
const int BUZZER_PIN = 9;           // Piezo buzzer
const int LED_PIN = 13;             // Status LED

// OR gate detection threshold (analog)
const int OR_GATE_THRESHOLD = 400;  // ~2V - 5V signals are strong, easy to detect
const unsigned long OR_GATE_CONFIRMATION_MS = 10;  // Fast confirmation for strong signals

// TiM240 detection (digital from Pi GPIO)
// Pi GPIO outputs 3.3V HIGH when TiM240 detects person
const unsigned long TIM240_CONFIRMATION_MS = 50;  // Confirmation time for digital signal

// Detection state
bool person_detected = false;

// Alarm system
unsigned long alarmStartTime = 0;
bool alarmActive = false;
const unsigned long alarmDuration = 5000; // 5 seconds

// Time-based confirmation for each input
unsigned long or_gate_detectionStartTime = 0;
unsigned long tim240_detectionStartTime = 0;

// Input state tracking
bool or_gate_detected = false;
bool tim240_detected = false;

// Serial communication with Pi
String serialCommand = "";
bool commandReady = false;

// Status reporting
unsigned long lastStatusReport = 0;
const unsigned long statusInterval = 1000; // Report every 1 second

void setup() {
  Serial.begin(9600);
  
  // Pin setup
  pinMode(OR_GATE_PIN, INPUT);      // Analog input for OR gate
  pinMode(TIM240_PIN, INPUT);       // Digital input from Pi GPIO (no pull-up, Pi controls it)
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize states
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize detection timers
  or_gate_detectionStartTime = 0;
  tim240_detectionStartTime = 0;
  
  Serial.println("# LiDAR Detection System - Arduino Ready (Dual Input)");
  Serial.println("# Input 1: Analog Pin A0 - OR Gate (TiM100 OR TiM150, 5V signal)");
  Serial.print("#   Threshold: ");
  Serial.print(OR_GATE_THRESHOLD);
  Serial.print(" ADC (");
  Serial.print((OR_GATE_THRESHOLD * 5.0 / 1023.0), 2);
  Serial.print("V), Confirmation: ");
  Serial.print(OR_GATE_CONFIRMATION_MS);
  Serial.println(" ms");
  Serial.println("# Input 2: Digital Pin 3 - TiM240 from Pi GPIO (3.3V HIGH when detected)");
  Serial.print("#   Confirmation: ");
  Serial.print(TIM240_CONFIRMATION_MS);
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
  unsigned long now = millis();
  
  // Handle serial commands from Pi
  handleSerialCommands();
  
  // ============================================================
  // READ INPUT 1: OR Gate (Side LiDARs - TiM100/TiM150)
  // ============================================================
  int or_gate_voltage = analogRead(OR_GATE_PIN);
  bool or_gate_high = (or_gate_voltage > OR_GATE_THRESHOLD);
  
  if (or_gate_high) {
    // OR gate signal detected
    if (or_gate_detectionStartTime == 0) {
      or_gate_detectionStartTime = now;
    } else if ((now - or_gate_detectionStartTime) >= OR_GATE_CONFIRMATION_MS) {
      // Confirmed: stable signal above threshold
      or_gate_detected = true;
    }
  } else {
    // OR gate signal low - reset timer
    or_gate_detectionStartTime = 0;
    or_gate_detected = false;
  }
  
  // ============================================================
  // READ INPUT 2: TiM240 from Pi GPIO (Digital)
  // ============================================================
  int tim240_level = digitalRead(TIM240_PIN);
  bool tim240_high = (tim240_level == HIGH);
  
  if (tim240_high) {
    // TiM240 signal detected
    if (tim240_detectionStartTime == 0) {
      tim240_detectionStartTime = now;
    } else if ((now - tim240_detectionStartTime) >= TIM240_CONFIRMATION_MS) {
      // Confirmed: stable HIGH signal
      tim240_detected = true;
    }
  } else {
    // TiM240 signal low - reset timer
    tim240_detectionStartTime = 0;
    tim240_detected = false;
  }
  
  // ============================================================
  // COMBINED DETECTION LOGIC (OR both inputs)
  // ============================================================
  bool new_person_detected = (or_gate_detected || tim240_detected);
  
  // Edge detection: trigger alarm on rising edge
  if (new_person_detected && !person_detected) {
    person_detected = true;
    triggerAlarm();
    sendStatusToPi();
    Serial.println("✅ Person detected - Triggering alarm");
  }
  
  // Clear detection on falling edge
  if (!new_person_detected && person_detected) {
    person_detected = false;
    // Ensure alarm is stopped (clean state)
    if (alarmActive) {
      alarmActive = false;
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
    }
    Serial.println("✅ Area clear - Person left");
    sendStatusToPi();
  }
  
  // Run alarm system
  runAlarmSystem();
  
  // Send status to Pi every second
  if (now - lastStatusReport >= statusInterval) {
    sendStatusToPi();
    lastStatusReport = now;
  }
}

void triggerAlarm() {
  alarmActive = true;
  alarmStartTime = millis();
  
  Serial.println("Trigger detected - Siren ON");
  
  // Send detection signal to Pi
  Serial.println("PERSON_DETECTED");
}

void runAlarmSystem() {
  // Run siren for 5 seconds
  if (alarmActive) {
    unsigned long elapsed = millis() - alarmStartTime;
    if (elapsed < alarmDuration) {
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
    // Also print current input states for debugging
    int or_voltage = analogRead(OR_GATE_PIN);
    int tim240_level = digitalRead(TIM240_PIN);
    Serial.print("# OR Gate voltage: ");
    Serial.print(or_voltage);
    Serial.print(" ADC (");
    Serial.print((or_voltage * 5.0 / 1023.0), 2);
    Serial.print("V), TiM240: ");
    Serial.println(tim240_level == HIGH ? "HIGH" : "LOW");
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
// OR Gate Output → Analog Pin A0
//   - TiM100 Signal → OR Gate Input 1
//   - TiM150 Signal → OR Gate Input 2
//   - OR Gate outputs 5V when either side LiDAR detects
//
// TiM240 Signal:
//   - TiM240 → Pi GPIO 18 (Pin 12) → Arduino Digital Pin 3
//   - Pi GPIO outputs 3.3V HIGH when TiM240 detects person
//   - Arduino Pin 3: INPUT (no pull-up, Pi controls it)
//
// Piezo Buzzer → Pin 9
// Status LED → Pin 13 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors two separate inputs independently
// - OR gate (A0): 5V signal from side LiDARs - fast detection (10ms)
// - TiM240 (Pin 3): 3.3V signal from Pi - digital detection (50ms)
// - Triggers alarm if EITHER input detects person
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing

