// SICK7 PBT-Only System - Arduino Code
// Reads PBT sensor data and controls arcade machine GPIO
// No credit tracking, no LiDAR - just PBT sensor + GPIO control

const int PBT_PIN = A0;           // PBT sensor input
const int BUZZER_PIN = 10;        // Piezo buzzer output
const int LED_PIN = 13;           // Status LED
const int GPIO_PIN6 = 6;          // Arcade motherboard Pin 6 (Press START)
const int GPIO_PIN5 = 5;          // Arcade motherboard Pin 5 (Press ACTIVE)

// PBT sensor configuration
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int BASELINE_ADC = 512;     // 2.5V baseline
const int NOISE_LEVEL = 5;        // Random noise amplitude

// System state
bool system_running = true;
unsigned long next_sample = 0;
int sample_count = 0;

// GPIO command handling
String gpioCommand = "";
bool gpioCommandReady = false;

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(PBT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN6, OUTPUT);
  pinMode(GPIO_PIN5, OUTPUT);
  
  // Set initial states
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GPIO_PIN6, LOW);
  digitalWrite(GPIO_PIN5, HIGH);
  
  next_sample = micros();
  
  Serial.println("# SICK7 PBT-Only System - Arduino Ready");
  Serial.println("# PBT Sensor: Pin A0");
  Serial.println("# Arcade Control: Pin 6 (START), Pin 5 (ACTIVE)");
  Serial.println("# Commands: PIN6_HIGH, PIN6_LOW, PIN5_HIGH, PIN5_LOW, RESET_GPIO, STATUS");
  Serial.println("# READY");
  
  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = millis();
  
  // Handle GPIO commands from Pi
  handleGPIOCommands();
  
  // Read PBT sensor data
  if ((long)(now - next_sample) >= 0) {
    next_sample += SAMPLE_US;
    readPBTSensor();
  }
  
  // LED activity indicator
  if (nowMs % 1000 < 50) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void readPBTSensor() {
  // Read PBT sensor value
  int sensorValue = analogRead(PBT_PIN);
  
  // Add some noise for realism (optional)
  sensorValue += random(-NOISE_LEVEL, NOISE_LEVEL + 1);
  sensorValue = constrain(sensorValue, 0, 1023);
  
  // Send to Pi
  Serial.println(sensorValue);
  
  sample_count++;
  
  // Status update every 1000 samples
  if (sample_count % 1000 == 0) {
    Serial.print("# Status: Samples=");
    Serial.print(sample_count);
    Serial.print(" Sensor=");
    Serial.print(sensorValue);
    Serial.print(" ADC (");
    Serial.print((sensorValue * 5.0) / 1023.0, 2);
    Serial.println("V)");
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
  
  if (command == "PIN6_HIGH") {
    digitalWrite(GPIO_PIN6, HIGH);
    Serial.println("# GPIO: Pin 6 → HIGH (5V) - START signal");
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
    Serial.println("# GPIO: Pin 5 → LOW (0V) - ACTIVE signal");
  }
  else if (command == "RESET_GPIO") {
    digitalWrite(GPIO_PIN6, LOW);
    digitalWrite(GPIO_PIN5, HIGH);
    Serial.println("# GPIO: Reset to idle state");
  }
  else if (command == "STATUS") {
    Serial.print("# STATUS: System=RUNNING Samples=");
    Serial.print(sample_count);
    Serial.print(" Pin6=");
    Serial.print(digitalRead(GPIO_PIN6) ? "HIGH" : "LOW");
    Serial.print(" Pin5=");
    Serial.println(digitalRead(GPIO_PIN5) ? "HIGH" : "LOW");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

// Hardware Connections:
// PBT Sensor → Pin A0 (Analog input)
// Arcade Pin 6 → Pin 6 (Digital output) - START signal
// Arcade Pin 5 → Pin 5 (Digital output) - ACTIVE signal
// Piezo Buzzer → Pin 10 (Digital output) - Audio feedback
// Status LED → Pin 13 (Digital output) - System status
// GND → Common ground
//
// Expected Behavior:
// - Continuous PBT sensor reading at 800Hz
// - GPIO control via serial commands from Pi
// - Real-time status reporting
// - No credit tracking or LiDAR functionality
