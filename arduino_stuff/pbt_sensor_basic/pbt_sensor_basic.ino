// Real PBT Sensor - Basic Version
// Direct analog read from piezoelectric sensor
// Replace signal_simulator.ino with this for real sensor input

const int PBT_PIN = A0;           // Analog pin connected to PBT sensor
const int SAMPLES_PER_SEC = 800;  // Sample rate (must match Pi config)
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  analogReference(DEFAULT);  // Use default voltage reference (5V or 3.3V)
  nextSample = micros();
  
  // Wait for serial to stabilize
  delay(100);
}

void loop() {
  unsigned long now = micros();
  
  // Sample at precise intervals
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Read analog value from PBT sensor (0-1023)
    int value = analogRead(PBT_PIN);
    
    // Send to serial
    Serial.println(value);
  }
}

// Hardware Setup:
// PBT Sensor (+) ──┤├── 1MΩ ──┬── Arduino A0
//                  10µF       │
// PBT Sensor (-) ────────────┴── Arduino GND
//
// This is the simplest possible circuit.
// May need amplification for better sensitivity.

