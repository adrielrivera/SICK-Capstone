// Real PBT Sensor - Auto-Calibrated Version
// Automatically finds baseline and centers signal around 512

const int PBT_PIN = A0;
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

// Calibration settings
const int CALIBRATION_SAMPLES = 1000;  // ~1.25 seconds at 800 Hz
int baselineOffset = 512;  // Will be updated during calibration
bool calibrated = false;
int calCount = 0;
long calSum = 0;

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  analogReference(DEFAULT);
  nextSample = micros();
  delay(100);
  
  // Notify that calibration is starting
  // (Pi will ignore data during this period)
  Serial.println("# CALIBRATING...");
}

void loop() {
  unsigned long now = micros();
  
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    int rawValue = analogRead(PBT_PIN);
    
    // Calibration phase
    if (!calibrated) {
      calSum += rawValue;
      calCount++;
      
      if (calCount >= CALIBRATION_SAMPLES) {
        // Calculate baseline offset
        baselineOffset = calSum / calCount;
        calibrated = true;
        
        // Notify calibration complete
        Serial.print("# BASELINE=");
        Serial.println(baselineOffset);
        Serial.println("# READY");
      }
      return;  // Don't send data during calibration
    }
    
    // Normal operation - send calibrated value
    // Re-center signal around 512 (middle of 0-1023 range)
    int value = rawValue - baselineOffset + 512;
    
    // Clamp to valid ADC range
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    
    Serial.println(value);
  }
}

// This version auto-calibrates the baseline on startup.
// Make sure sensor is not being impacted during first 1.25 seconds!
//
// Pi will see:
// # CALIBRATING...
// # BASELINE=523
// # READY
// 512
// 513
// 511
// ... (centered data)

