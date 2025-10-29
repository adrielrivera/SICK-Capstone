// LiDAR Detection System - Second Arduino
// Monitors OR gate input from TiM100, TiM150, and TiM240 detection circuit
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int INPUT_PIN = 8;      // OR gate input (TiM100 OR TiM150 OR TiM240)
const int BUZZER_PIN = 9;     // Piezo buzzer
const int LED_PIN = 13;       // Status LED

// Detection state
bool person_detected = false;

// Alarm system (based on your existing code)
unsigned long alarmStartTime = 0;
bool alarmActive = false;
const unsigned long alarmDuration = 5000; // 5 seconds

// Serial communication with Pi
String serialCommand = "";
bool commandReady = false;

// Status reporting
unsigned long lastStatusReport = 0;
const unsigned long statusInterval = 1000; // Report every 1 second

void setup() {
  Serial.begin(9600);  // Match your working code baud rate
  
  // Pin setup
  pinMode(INPUT_PIN, INPUT);    // External pull-down resistor to GND
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize states
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("# LiDAR Detection System - Arduino Ready");
  Serial.println("# OR Gate Input: Pin 8 (TiM100 OR TiM150 OR TiM240)");
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
  int state = digitalRead(INPUT_PIN);
  
  // Handle serial commands from Pi
  handleSerialCommands();
  
  // Check for person detection (HIGH = person detected)
  bool currentDetection = (state == HIGH);
  
  // Trigger alarm if detection starts
  if (currentDetection && !person_detected) {
    person_detected = true;
    triggerAlarm();
  } else if (!currentDetection && person_detected) {
    person_detected = false;
    Serial.println("✅ Area clear - Person left");
  }
  
  // Run alarm system (exactly like your working code)
  runAlarmSystem();
  
  // Send status to Pi every second
  unsigned long now = millis();
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
// OR Gate Output → Pin 8 (with pull-down resistor)
//   - TiM100 Signal → OR Gate Input 1
//   - TiM150 Signal → OR Gate Input 2  
//   - TiM240 Signal → OR Gate Input 3 (from Pi via GPIO)
// Piezo Buzzer → Pin 9
// Status LED → Pin 13 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors OR gate input (Pin 8)
// - Triggers 5-second alarm when ANY LiDAR detects person
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing
