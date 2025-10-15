// Real PBT Sensor - Full Featured Version
// Includes: auto-calibration, averaging, diagnostics, LED indicator

const int PBT_PIN = A0;
const int LED_PIN = 13;  // Built-in LED for visual feedback
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

// Averaging
const int AVG_SAMPLES = 4;

// Calibration
const int CALIBRATION_SAMPLES = 1000;
int baselineOffset = 512;
bool calibrated = false;
int calCount = 0;
long calSum = 0;

// Diagnostics
unsigned long lastStatsMs = 0;
unsigned long sampleCount = 0;
int minValue = 1023;
int maxValue = 0;

// Activity detection for LED
int activityThreshold = 50;  // Deviation from baseline to trigger LED
unsigned long lastActivityMs = 0;
const unsigned long LED_ON_MS = 100;  // LED stays on for 100ms after activity

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  analogReference(DEFAULT);
  nextSample = micros();
  lastStatsMs = millis();
  delay(100);
  
  Serial.println("# PBT Sensor Full Version");
  Serial.println("# CALIBRATING...");
}

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = millis();
  
  // LED activity indicator
  if (nowMs - lastActivityMs > LED_ON_MS) {
    digitalWrite(LED_PIN, LOW);
  }
  
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Read with averaging
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
      delayMicroseconds(10);
    }
    int rawValue = sum / AVG_SAMPLES;
    
    // Calibration phase
    if (!calibrated) {
      calSum += rawValue;
      calCount++;
      
      if (calCount >= CALIBRATION_SAMPLES) {
        baselineOffset = calSum / calCount;
        calibrated = true;
        
        Serial.print("# BASELINE=");
        Serial.println(baselineOffset);
        Serial.println("# READY");
        
        // Blink LED to indicate ready
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(100);
          digitalWrite(LED_PIN, LOW);
          delay(100);
        }
      }
      return;
    }
    
    // Calibrate and center value
    int value = rawValue - baselineOffset + 512;
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    
    // Activity detection for LED
    int deviation = abs(value - 512);
    if (deviation > activityThreshold) {
      digitalWrite(LED_PIN, HIGH);
      lastActivityMs = nowMs;
    }
    
    // Track statistics
    sampleCount++;
    if (value < minValue) minValue = value;
    if (value > maxValue) maxValue = value;
    
    // Send data
    Serial.println(value);
  }
  
  // Print diagnostics every 5 seconds
  unsigned long nowMs2 = millis();
  if (calibrated && nowMs2 - lastStatsMs >= 5000) {
    Serial.print("# STATS: samples=");
    Serial.print(sampleCount);
    Serial.print(" min=");
    Serial.print(minValue);
    Serial.print(" max=");
    Serial.print(maxValue);
    Serial.print(" range=");
    Serial.println(maxValue - minValue);
    
    lastStatsMs = nowMs2;
    // Reset min/max for next period
    minValue = 1023;
    maxValue = 0;
  }
}

// Features:
// - Auto-calibration on startup
// - Noise reduction via averaging
// - LED blinks on activity (visual feedback)
// - Diagnostic output every 5 seconds (commented lines starting with #)
// - Min/max tracking to verify signal range
//
// Diagnostic output example:
// # STATS: samples=4000 min=490 max=687 range=197
//
// This tells you if your signal is in a good range or needs amplifier adjustment.

