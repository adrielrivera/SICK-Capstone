// LiDAR Detection System - Second Arduino
// Monitors TiM100, TiM150, and TiM240 detection signals
// Activates alarm circuit when ANY LiDAR detects a person
// Sends status back to Pi for webapp display

const int TIM100_PIN = 8;     // TiM100 (Left) detection input
const int TIM150_PIN = 9;     // TiM150 (Right) detection input  
const int TIM240_PIN = 10;    // TiM240 (Rear) detection input from Pi
const int BUZZER_PIN = 11;    // Piezo buzzer
const int LED_PIN = 13;       // Status LED
const int ALARM_LED_PIN = 12; // Alarm LED (separate from status)

// Detection state
bool tim100_detected = false;
bool tim150_detected = false;
bool tim240_detected = false;
bool any_detection = false;

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
  Serial.begin(115200);
  
  // Pin setup
  pinMode(TIM100_PIN, INPUT);
  pinMode(TIM150_PIN, INPUT);
  pinMode(TIM240_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);
  
  // Initialize states
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(ALARM_LED_PIN, LOW);
  
  Serial.println("# LiDAR Detection System - Arduino Ready");
  Serial.println("# TiM100: Pin 8, TiM150: Pin 9, TiM240: Pin 10");
  Serial.println("# Alarm: Buzzer Pin 11, LED Pin 12");
  Serial.println("# Commands: STATUS, RESET_ALARM");
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
  
  // Read LiDAR detection inputs
  checkTim100Status();
  checkTim150Status();
  checkTim240Status();
  
  // Update overall detection state
  bool previousDetection = any_detection;
  any_detection = tim100_detected || tim150_detected || tim240_detected;
  
  // Trigger alarm if detection starts
  if (any_detection && !previousDetection) {
    triggerAlarm();
  }
  
  // Run alarm system
  runAlarmSystem();
  
  // Send status to Pi every second
  if (now - lastStatusReport >= statusInterval) {
    sendStatusToPi();
    lastStatusReport = now;
  }
  
  // Blink status LED to show system is running
  digitalWrite(LED_PIN, (now / 500) % 2);
}

void checkTim100Status() {
  static int lastState = LOW;
  int currentState = digitalRead(TIM100_PIN);
  tim100_detected = (currentState == HIGH);
  
  if (currentState != lastState) {
    if (tim100_detected) {
      Serial.println("🚨 TiM100 DETECTED - Person on LEFT side");
    } else {
      Serial.println("✅ TiM100 CLEAR - LEFT side clear");
    }
    lastState = currentState;
  }
}

void checkTim150Status() {
  static int lastState = LOW;
  int currentState = digitalRead(TIM150_PIN);
  tim150_detected = (currentState == HIGH);
  
  if (currentState != lastState) {
    if (tim150_detected) {
      Serial.println("🚨 TiM150 DETECTED - Person on RIGHT side");
    } else {
      Serial.println("✅ TiM150 CLEAR - RIGHT side clear");
    }
    lastState = currentState;
  }
}

void checkTim240Status() {
  static int lastState = LOW;
  int currentState = digitalRead(TIM240_PIN);
  tim240_detected = (currentState == HIGH);
  
  if (currentState != lastState) {
    if (tim240_detected) {
      Serial.println("🚨 TiM240 DETECTED - Person on REAR side");
    } else {
      Serial.println("✅ TiM240 CLEAR - REAR side clear");
    }
    lastState = currentState;
  }
}

void triggerAlarm() {
  alarmActive = true;
  alarmStartTime = millis();
  
  Serial.println("🚨 ===== ALARM TRIGGERED =====");
  Serial.println("# Person detected by LiDAR system!");
  Serial.println("# Alarm will run for 5 seconds");
  
  // Send detection signal to Pi
  Serial.println("PERSON_DETECTED");
}

void runAlarmSystem() {
  if (alarmActive) {
    unsigned long elapsed = millis() - alarmStartTime;
    
    if (elapsed < alarmDuration) {
      // Run alarm for 5 seconds (based on your existing code)
      int freq = map((elapsed / 10) % 100, 0, 99, 500, 1000); // Sweeping tone
      tone(BUZZER_PIN, freq);
      digitalWrite(ALARM_LED_PIN, (elapsed / 200) % 2); // Blink alarm LED
    } else {
      // Stop alarm after 5 seconds
      noTone(BUZZER_PIN);
      digitalWrite(ALARM_LED_PIN, LOW);
      alarmActive = false;
      Serial.println("🔇 Alarm OFF - 5 seconds elapsed");
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
    digitalWrite(ALARM_LED_PIN, LOW);
    Serial.println("# Alarm reset manually");
  }
  else if (command == "TIM240_HIGH") {
    digitalWrite(TIM240_PIN, HIGH);
    Serial.println("# TiM240 signal set HIGH (simulated detection)");
  }
  else if (command == "TIM240_LOW") {
    digitalWrite(TIM240_PIN, LOW);
    Serial.println("# TiM240 signal set LOW (simulated clear)");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

void sendStatusToPi() {
  Serial.print("LIDAR_STATUS:");
  Serial.print(tim100_detected ? "1" : "0");
  Serial.print(",");
  Serial.print(tim150_detected ? "1" : "0");
  Serial.print(",");
  Serial.print(tim240_detected ? "1" : "0");
  Serial.print(",");
  Serial.print(any_detection ? "1" : "0");
  Serial.print(",");
  Serial.print(alarmActive ? "1" : "0");
  Serial.println();
}

// Hardware Connections:
// TiM100 Signal → Pin 8 (with pull-down resistor)
// TiM150 Signal → Pin 9 (with pull-down resistor)  
// TiM240 Signal → Pin 10 (from Pi via GPIO)
// Piezo Buzzer → Pin 11
// Status LED → Pin 13 (blinks when running)
// Alarm LED → Pin 12 (blinks during alarm)
// GND → Common ground
//
// Expected Behavior:
// - Monitors all 3 LiDAR inputs
// - Triggers 5-second alarm when ANY detection occurs
// - Sends status updates to Pi every second
// - Responds to Pi commands for testing
