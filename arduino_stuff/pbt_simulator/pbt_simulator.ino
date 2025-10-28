// PBT Simulator - Generates realistic PBT waveforms for testing
// Outputs waveform on Pin 3 (PWM) for oscilloscope monitoring
// Simulates real PBT sensor behavior with peaks, noise, and baseline

const int PBT_PIN = A0;           // PBT sensor input (not used in simulation)
const int TIM100_PIN = 8;         // TiM100 (Left) detection input
const int TIM150_PIN = 9;         // TiM150 (Right) detection input
const int BUZZER_PIN = 10;        // Piezo buzzer output
const int LED_PIN = 13;           // Status LED
const int GPIO_PIN6 = 6;          // Arcade motherboard Pin 6 (Press START)
const int GPIO_PIN5 = 5;          // Arcade motherboard Pin 5 (Press ACTIVE)
const int WAVEFORM_PIN = 3;       // PWM output for oscilloscope (DAC simulation)

// PBT Simulation Configuration
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int BASELINE_ADC = 512;     // 2.5V baseline
const int NOISE_LEVEL = 5;        // Random noise amplitude
const int PEAK_AMPLITUDE = 100;   // Maximum peak amplitude
const int PEAK_DURATION = 50;     // Peak duration in samples

// Simulation state
bool simulation_running = true;
bool generate_peak = false;
int peak_counter = 0;
int peak_amplitude = 0;
int current_value = BASELINE_ADC;
unsigned long next_sample = 0;

// LiDAR status
bool tim100_detected = false;
bool tim150_detected = false;
int lastTim100State = LOW;
int lastTim150State = LOW;

// GPIO command handling
String gpioCommand = "";
bool gpioCommandReady = false;

// Custom peak input
int custom_peak_amplitude = 0;
bool custom_peak_requested = false;

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(PBT_PIN, INPUT);
  pinMode(TIM100_PIN, INPUT);
  pinMode(TIM150_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GPIO_PIN6, OUTPUT);
  pinMode(GPIO_PIN5, OUTPUT);
  pinMode(WAVEFORM_PIN, OUTPUT);
  
  // Set initial states
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(GPIO_PIN6, LOW);
  digitalWrite(GPIO_PIN5, HIGH);
  
  // Initialize PWM for waveform output
  analogWrite(WAVEFORM_PIN, BASELINE_ADC / 4); // Convert 0-1023 to 0-255
  
  next_sample = micros();
  
  Serial.println("# PBT Simulator - Custom Peak Waveform Generation");
  Serial.println("# Waveform output: Pin 3 (PWM)");
  Serial.println("# Oscilloscope: Connect Pin 3 to Channel 1");
  Serial.println("# Arcade signals: Pin 6 (Channel 2), Pin 5 (Channel 3)");
  Serial.println("# Commands: CUSTOM_PEAK:amplitude, START_SIMULATION, STOP_SIMULATION");
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
  
  // Check LiDAR status
  checkTim100Status();
  checkTim150Status();
  
  // Generate PBT waveform
  if ((long)(now - next_sample) >= 0) {
    next_sample += SAMPLE_US;
    generatePBTWaveform();
  }
  
  // Check for custom peak request
  if (custom_peak_requested) {
    peak_amplitude = custom_peak_amplitude;
    generate_peak = true;
    peak_counter = PEAK_DURATION;
    custom_peak_requested = false;
    
    Serial.print("# CUSTOM PEAK GENERATED: Amplitude=");
    Serial.print(peak_amplitude);
    Serial.print(" ADC (");
    Serial.print((peak_amplitude * 5.0) / 1023.0, 2);
    Serial.println("V)");
  }
  
  // LED activity indicator
  if (nowMs % 1000 < 50) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

void generatePBTWaveform() {
  // Generate realistic PBT waveform
  int value = BASELINE_ADC;
  
  // Add random noise
  value += random(-NOISE_LEVEL, NOISE_LEVEL + 1);
  
  // Add peak if active
  if (generate_peak && peak_counter > 0) {
    // Generate peak shape (bell curve)
    float peak_factor = sin(PI * (PEAK_DURATION - peak_counter) / PEAK_DURATION);
    int peak_value = peak_amplitude * peak_factor;
    value += peak_value;
    peak_counter--;
    
    if (peak_counter <= 0) {
      generate_peak = false;
    }
  }
  
  // Clamp value to valid range
  value = constrain(value, 0, 1023);
  current_value = value;
  
  // Output to PWM for oscilloscope (convert 0-1023 to 0-255)
  analogWrite(WAVEFORM_PIN, value / 4);
  
  // Send to Pi via serial
  Serial.println(value);
}

void generateCustomPeak(int amplitude) {
  // Generate custom peak with specified amplitude
  custom_peak_amplitude = constrain(amplitude, 0, 200); // Limit to reasonable range
  custom_peak_requested = true;
  
  Serial.print("# CUSTOM PEAK REQUESTED: Amplitude=");
  Serial.print(custom_peak_amplitude);
  Serial.print(" ADC (");
  Serial.print((custom_peak_amplitude * 5.0) / 1023.0, 2);
  Serial.println("V)");
}

void checkTim100Status() {
  int currentState = digitalRead(TIM100_PIN);
  tim100_detected = (currentState == HIGH);
  
  if (currentState != lastTim100State) {
    if (tim100_detected) {
      Serial.println("🚨 ===== PERSON DETECTED - LEFT SIDE (TiM100) =====");
      Serial.println("# TiM100 DETECTED - Person on LEFT side");
    } else {
      Serial.println("✅ ===== AREA CLEAR - LEFT SIDE (TiM100) =====");
      Serial.println("# TiM100 CLEAR - LEFT side clear");
    }
    lastTim100State = currentState;
  }
}

void checkTim150Status() {
  int currentState = digitalRead(TIM150_PIN);
  tim150_detected = (currentState == HIGH);
  
  if (currentState != lastTim150State) {
    if (tim150_detected) {
      Serial.println("🚨 ===== PERSON DETECTED - RIGHT SIDE (TiM150) =====");
      Serial.println("# TiM150 DETECTED - Person on RIGHT side");
    } else {
      Serial.println("✅ ===== AREA CLEAR - RIGHT SIDE (TiM150) =====");
      Serial.println("# TiM150 CLEAR - RIGHT side clear");
    }
    lastTim150State = currentState;
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
  
  if (command == "PBT_HIT") {
    Serial.println("# PBT HIT: Received from Pi");
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
  else if (command.startsWith("CUSTOM_PEAK:")) {
    // Parse custom peak command: "CUSTOM_PEAK:45"
    int colonIndex = command.indexOf(':');
    if (colonIndex > 0) {
      int amplitude = command.substring(colonIndex + 1).toInt();
      generateCustomPeak(amplitude);
    }
  }
  else if (command == "START_SIMULATION") {
    simulation_running = true;
    Serial.println("# SIMULATION: Started");
  }
  else if (command == "STOP_SIMULATION") {
    simulation_running = false;
    Serial.println("# SIMULATION: Stopped");
  }
  else if (command == "STATUS") {
    Serial.print("# STATUS: Simulation=");
    Serial.print(simulation_running ? "RUNNING" : "STOPPED");
    Serial.print(" TiM100=");
    Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
    Serial.print(" TiM150=");
    Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
  }
  else {
    Serial.print("# Unknown command: ");
    Serial.println(command);
  }
}

// Hardware Connections for Oscilloscope:
// Pin 3 (PWM) → Oscilloscope Channel 1 (PBT Waveform)
// Pin 6 (Digital) → Oscilloscope Channel 2 (Arcade Start Signal)
// Pin 5 (Digital) → Oscilloscope Channel 3 (Arcade Active Signal)
// GND → Oscilloscope Ground
//
// Expected Waveform:
// - Baseline: ~2.5V (512 ADC = 128 PWM)
// - Noise: ±0.1V random variation
// - Peaks: 0.5V to 2.5V spikes (30-100 ADC = 8-25 PWM)
// - Frequency: 800Hz sampling rate
