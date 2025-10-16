// SICK 10 Bar PBT Sensor - Optimized for Real Hardware
// Based on oscilloscope measurements:
// - Baseline: ~1.88V = ~385 ADC counts
// - Peak: ~2.96V = ~607 ADC counts
// - Pulse duration: ~700ms

const int PBT_PIN = A0;
const int LED_PIN = 13;
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

// Averaging for noise reduction
const int AVG_SAMPLES = 4;

// Expected baseline for SICK sensor (~380 for 1.86V on 5V Arduino)
// Will auto-calibrate, but this is the expected range
const int EXPECTED_BASELINE_MIN = 350;
const int EXPECTED_BASELINE_MAX = 450;

// Calibration
const int CALIBRATION_SAMPLES = 1000;  // ~1.25 seconds
int baselineOffset = 512;  // Will be updated during calibration
bool calibrated = false;
int calCount = 0;
long calSum = 0;

// Diagnostics
unsigned long lastStatsMs = 0;
unsigned long sampleCount = 0;
int minValue = 1023;
int maxValue = 0;

// Activity detection for LED (adjusted for SICK sensor range)
int activityThreshold = 30;  // ~30 counts above baseline
unsigned long lastActivityMs = 0;
const unsigned long LED_ON_MS = 100;

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Use default 5V reference
  analogReference(DEFAULT);
  
  nextSample = micros();
  lastStatsMs = millis();
  delay(100);
  
  Serial.println("# SICK 10 Bar PBT Sensor");
  Serial.println("# Expected baseline: ~385 ADC counts (1.88V)");
  Serial.println("# Expected peak: ~607 ADC counts (2.96V)");
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
    
    // Read with averaging to reduce noise
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
      delayMicroseconds(10);
    }
    int rawValue = sum / AVG_SAMPLES;
    
    // Calibration phase - find baseline
    if (!calibrated) {
      calSum += rawValue;
      calCount++;
      
      if (calCount >= CALIBRATION_SAMPLES) {
        int measuredBaseline = calSum / calCount;
        baselineOffset = measuredBaseline;
        calibrated = true;
        
        Serial.print("# BASELINE MEASURED=");
        Serial.print(measuredBaseline);
        Serial.print(" ADC counts (");
        Serial.print((measuredBaseline * 5.0) / 1023.0, 2);
        Serial.println("V)");
        
        // Sanity check
        if (measuredBaseline < EXPECTED_BASELINE_MIN || 
            measuredBaseline > EXPECTED_BASELINE_MAX) {
          Serial.print("# WARNING: Baseline outside expected range ");
          Serial.print(EXPECTED_BASELINE_MIN);
          Serial.print("-");
          Serial.println(EXPECTED_BASELINE_MAX);
          Serial.println("# Check sensor connections!");
        } else {
          Serial.println("# Baseline looks good!");
        }
        
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
    
    // Normal operation - re-center around 512
    // This makes the signal compatible with Pi's existing baseline tracking
    int value = rawValue - baselineOffset + 512;
    
    // Clamp to valid range
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
    if (rawValue < minValue) minValue = rawValue;
    if (rawValue > maxValue) maxValue = rawValue;
    
    // Send centered data (compatible with existing Pi software)
    Serial.println(value);
  }
  
  // Print diagnostics every 5 seconds
  unsigned long nowMs2 = millis();
  if (calibrated && nowMs2 - lastStatsMs >= 5000) {
    int range = maxValue - minValue;
    
    Serial.print("# STATS: samples=");
    Serial.print(sampleCount);
    Serial.print(" raw_min=");
    Serial.print(minValue);
    Serial.print(" raw_max=");
    Serial.print(maxValue);
    Serial.print(" range=");
    Serial.print(range);
    Serial.print(" (");
    Serial.print((range * 5.0) / 1023.0, 2);
    Serial.println("V)");
    
    // Interpretation
    if (range < 20) {
      Serial.println("# No activity detected (range < 20 counts)");
    } else if (range > 200) {
      Serial.println("# High activity detected! Good signal.");
    }
    
    lastStatsMs = nowMs2;
    // Reset min/max for next period
    minValue = 1023;
    maxValue = 0;
  }
}

// Hardware Connection:
// SICK 10 Bar PBT Sensor:
//   Signal Out ──── 100kΩ ──── Arduino A0
//   Ground     ──────────────── Arduino GND
//
// The 100kΩ resistor provides input impedance.
// SICK sensor output is already in good voltage range (1.88V - 2.96V)
//
// Expected output from this code:
// - During calibration: Finds baseline (~385 ADC)
// - During operation: Re-centers to 512 for Pi compatibility
// - Diagnostics: Shows raw min/max values every 5s
//
// This code is 100% compatible with your existing app_combined.py!

