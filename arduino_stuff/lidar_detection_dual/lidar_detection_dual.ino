// LiDAR Detection System - Dual Input Arduino + Credit Tracking
// Input 1: OR gate from side LiDARs (TiM100/TiM150) - Analog Pin A0 (5V signal)
// Input 2: TiM240 from Pi GPIO - Digital Pin 3 (3.3V signal)
// Credit Add: Pi GPIO 18 → Digital Pin 2 (falling edge 3.3V→0V)
// Activates alarm circuit when ANY LiDAR detects a person
// Tracks credits: 2 PBT hits = 1 credit deducted
// Sends status back to Pi for webapp display

const int OR_GATE_PIN = A0;        // Analog input - OR gate (TiM100 OR TiM150, outputs 5V)
const int TIM240_PIN = 3;           // Digital input - TiM240 from Pi GPIO (3.3V HIGH when detected)
const int CREDIT_ADD_PIN = 2;       // Digital input - Credit add signal from Pi GPIO (falling edge 3.3V→0V)
const int BUZZER_PIN = 9;           // Piezo buzzer
const int LED_PIN = 13;             // Status LED

// OR gate detection threshold (analog)
// Changed to 3V threshold to ignore 2.5V idle level
// 3.0V * 1023 / 5.0V = 614 ADC counts
const int OR_GATE_THRESHOLD = 610;  // ~3V - Only accept 3V+ as HIGH (ignores 2.5V idle)
const unsigned long OR_GATE_CONFIRMATION_MS = 10;  // Fast confirmation for strong signals

// TiM240 detection (digital from Pi GPIO)
// Pi GPIO outputs 3.3V HIGH when TiM240 detects person
// Uses confirmation time for both HIGH and LOW states to prevent false triggers/clears
const unsigned long TIM240_CONFIRMATION_MS = 50;  // Confirmation time for both HIGH and LOW states

// Detection state
bool person_detected = false;

// Alarm system
unsigned long alarmStartTime = 0;
bool alarmActive = false;
const unsigned long alarmDuration = 5000; // 5 seconds

// Time-based confirmation for each input
unsigned long or_gate_detectionStartTime = 0;
unsigned long tim240_detectionStartTime = 0;
unsigned long tim240_clearStartTime = 0;  // Timer for confirming LOW state

// Input state tracking
bool or_gate_detected = false;
bool tim240_detected = false;

// Credit tracking system
volatile int credits = 0;              // Current credit count (starts at 0) - volatile for interrupt
// Note: Hit counting is done on Pi - Arduino just manages credit balance

// Credit add interrupt debouncing (flag-based approach)
volatile bool creditAddPending = false;  // Flag set by interrupt, processed in main loop
volatile unsigned long creditAddInterruptTime = 0;  // Timestamp when interrupt fired
unsigned long lastCreditAddProcessedMs = 0;  // Last time credit was actually added
const unsigned long CREDIT_ADD_DEBOUNCE_MS = 1000;  // Debounce time (1000ms = max 1 credit/second)
const unsigned long CREDIT_ADD_MIN_LOW_MS = 20;  // Minimum time pin must stay LOW to be valid (filters noise)

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
  pinMode(CREDIT_ADD_PIN, INPUT);   // Digital input from Pi GPIO for credit add (no pull-up, Pi controls it)
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize states
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize detection timers
  or_gate_detectionStartTime = 0;
  tim240_detectionStartTime = 0;
  tim240_clearStartTime = 0;
  
  // Initialize credit system
  creditAddPending = false;
  creditAddInterruptTime = 0;
  lastCreditAddProcessedMs = 0;
  
  // Attach interrupt for credit add signal (falling edge: 3.3V → 0V)
  attachInterrupt(digitalPinToInterrupt(CREDIT_ADD_PIN), handleCreditAdd, FALLING);
  
  Serial.println("# LiDAR Detection System - Arduino Ready (Dual Input + Credit Tracking)");
  Serial.println("# Input 1: Analog Pin A0 - OR Gate (TiM100 OR TiM150, 5V signal)");
  Serial.print("#   Threshold: ");
  Serial.print(OR_GATE_THRESHOLD);
  Serial.print(" ADC (");
  Serial.print((OR_GATE_THRESHOLD * 5.0 / 1023.0), 2);
  Serial.print("V) - Only accepts 3V+ as HIGH (ignores 2.5V idle)");
  Serial.print(", Confirmation: ");
  Serial.print(OR_GATE_CONFIRMATION_MS);
  Serial.println(" ms");
  Serial.println("# Input 2: Digital Pin 3 - TiM240 from Pi GPIO (3.3V HIGH when detected)");
  Serial.print("#   Confirmation: ");
  Serial.print(TIM240_CONFIRMATION_MS);
  Serial.println(" ms");
  Serial.println("# Credit Add: Digital Pin 2 - Pi GPIO 18 (falling edge 3.3V→0V triggers +1 credit)");
  Serial.println("# Credit System: Pi tracks hits (2 hits = 1 credit), sends DEDUCT_CREDIT when needed");
  Serial.println("# LiDAR Safety: DISABLED when credits == 0 (no detection, no alarm)");
  Serial.println("# Alarm: Buzzer Pin 9, LED Pin 13");
  Serial.println("# Commands: STATUS, RESET_ALARM, SIMULATE_DETECTION, SIMULATE_CLEAR");
  Serial.println("# Credit Commands: DEDUCT_CREDIT, GET_CREDITS, SET_CREDITS:<n>, ADD_CREDITS:<n>");
  
  // Send initial credit status
  sendCreditStatus();
  
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
  
  // Process pending credit add from interrupt (with debouncing and noise filtering)
  if (creditAddPending) {
    // CRITICAL: Verify the pin is actually LOW and stays LOW for minimum duration
    // This filters out brief noise spikes that cause false falling edges
    unsigned long interruptTime = creditAddInterruptTime;
    unsigned long timeSinceInterrupt = now - interruptTime;
    
    // Check current pin state
    int currentPinState = digitalRead(CREDIT_ADD_PIN);
    
    if (currentPinState == HIGH) {
      // Pin is HIGH - was just a brief noise spike, ignore
      // This filters out very brief LOW pulses that don't last long enough
      creditAddPending = false;
    } else if (currentPinState == LOW) {
      // Pin is LOW - check if it's been LOW long enough to be valid
      if (timeSinceInterrupt >= CREDIT_ADD_MIN_LOW_MS) {
        // Pin has been LOW for minimum duration - this is a valid signal
        // Now check debounce (time since last credit add)
        if (now - lastCreditAddProcessedMs >= CREDIT_ADD_DEBOUNCE_MS) {
          // Double-check pin is still LOW (one more verification)
          if (digitalRead(CREDIT_ADD_PIN) == LOW) {
            // Process the credit add - this is a confirmed valid signal
            credits++;
            lastCreditAddProcessedMs = now;
            
            sendCreditStatus();
            Serial.println("# Credit added via hardware signal (Pin 2 falling edge)");
          }
          // Clear flag after processing (whether successful or not)
          creditAddPending = false;
        } else {
          // Debounce active - ignore this trigger
          // Clear flag to prevent spamming
          creditAddPending = false;
        }
      }
      // If not enough time has passed, keep flag set and check again next iteration
      // This allows the pin to stabilize before we decide if it's valid
    }
  }
  
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
    // TiM240 signal detected - start confirmation timer
    if (tim240_detectionStartTime == 0) {
      tim240_detectionStartTime = now;
    } else if ((now - tim240_detectionStartTime) >= TIM240_CONFIRMATION_MS) {
      // Confirmed: stable HIGH signal
      tim240_detected = true;
      // Reset clear timer since we're detecting
      tim240_clearStartTime = 0;
    }
  } else {
    // TiM240 signal low - start clear confirmation timer
    if (tim240_clearStartTime == 0) {
      tim240_clearStartTime = now;
    } else if ((now - tim240_clearStartTime) >= TIM240_CONFIRMATION_MS) {
      // Confirmed: stable LOW signal - clear detection
      tim240_detected = false;
      // Reset detection timer since we're clear
      tim240_detectionStartTime = 0;
    }
    // If we were detecting but signal went LOW, reset detection timer
    // (but don't clear detection until confirmed LOW for 50ms)
    if (tim240_detectionStartTime > 0) {
      tim240_detectionStartTime = 0;
    }
  }
  
  // ============================================================
  // COMBINED DETECTION LOGIC (OR both inputs)
  // ============================================================
  // CRITICAL: Only detect and trigger alarm if credits > 0
  // When credits == 0, LiDAR safety is disabled
  bool new_person_detected = false;
  
  if (credits > 0) {
    // Credits available - process detection normally
    new_person_detected = (or_gate_detected || tim240_detected);
    
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
  } else {
    // Credits == 0 - LiDAR safety disabled
    // Force clear any existing detection and stop alarm
    if (person_detected || alarmActive) {
      person_detected = false;
      alarmActive = false;
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
      sendStatusToPi();
      Serial.println("# LiDAR safety disabled (credits == 0) - Detection cleared");
    }
  }
  
  // Send status to Pi every second
  if (now - lastStatusReport >= statusInterval) {
    sendStatusToPi();
    lastStatusReport = now;
  }
}

void triggerAlarm() {
  // CRITICAL: Only trigger alarm if credits > 0
  if (credits == 0) {
    Serial.println("# Alarm trigger blocked (credits == 0)");
    return;  // Don't trigger alarm
  }
  
  alarmActive = true;
  alarmStartTime = millis();
  
  Serial.println("Trigger detected - Siren ON");
  
  // Send detection signal to Pi
  Serial.println("PERSON_DETECTED");
}

void runAlarmSystem() {
  // CRITICAL: Only run alarm if credits > 0
  // If credits == 0, immediately stop any active alarm
  if (credits == 0) {
    if (alarmActive) {
      alarmActive = false;
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
      Serial.println("# Alarm stopped (credits == 0)");
    }
    return;  // Exit early - don't run alarm system
  }
  
  // Run siren for 5 seconds (only if credits > 0)
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
    sendCreditStatus();
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
  // Credit tracking commands
  else if (command == "DEDUCT_CREDIT") {
    // Deduct 1 credit (Pi handles hit counting, just deducts when 2 hits reached)
    if (credits > 0) {
      credits--;
      Serial.print("# Credit deducted! Credits remaining: ");
      Serial.println(credits);
    } else {
      Serial.println("# No credits remaining - cannot deduct");
    }
    sendCreditStatus();
  }
  else if (command == "GET_CREDITS") {
    sendCreditStatus();
  }
  else if (command.startsWith("SET_CREDITS:")) {
    int newCredits = command.substring(12).toInt();
    if (newCredits >= 0) {
      credits = newCredits;
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

void sendStatusToPi() {
  Serial.print("LIDAR_STATUS:");
  Serial.print(person_detected ? "1" : "0");  // Any LiDAR detecting
  Serial.print(",");
  Serial.print(alarmActive ? "1" : "0");      // Alarm active
  Serial.println();
}

void sendCreditStatus() {
  Serial.print("CREDITS:");
  Serial.println(credits);
}

// Interrupt handler for credit add signal (falling edge on Pin 2)
// Uses flag-based approach: set flag in interrupt, process in main loop
void handleCreditAdd() {
  // Simply set the flag - actual processing happens in main loop with proper debouncing
  // This prevents issues with millis() not updating in interrupt context
  creditAddPending = true;
  creditAddInterruptTime = millis();  // Note: millis() may not update in interrupt, but we use it for reference
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
// Credit Add Signal:
//   - Pi GPIO 18 (Pin 12) → Arduino Digital Pin 2
//   - Signal: Falling edge (3.3V HIGH → 0V LOW) triggers +1 credit
//   - Arduino Pin 2: INPUT (no pull-up, Pi controls it)
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
// - Credit System: Pi tracks hits (2 hits = 1 credit), sends DEDUCT_CREDIT command
// - Credit Add: Falling edge on Pin 2 = +1 credit (hardware trigger)
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing (LiDAR + Credit commands)

