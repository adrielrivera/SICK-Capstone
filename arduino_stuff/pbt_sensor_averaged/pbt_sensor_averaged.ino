// Real PBT Sensor - Averaged Version
// Reduces noise by averaging multiple ADC readings per sample

const int PBT_PIN = A0;
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int AVG_SAMPLES = 4;  // Average 4 readings per output (reduces noise)

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  analogReference(DEFAULT);
  nextSample = micros();
  delay(100);
}

void loop() {
  unsigned long now = micros();
  
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Average multiple readings to reduce noise
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
      delayMicroseconds(10);  // Small delay between reads
    }
    int value = sum / AVG_SAMPLES;
    
    Serial.println(value);
  }
}

// Pros: Cleaner signal, reduced noise
// Cons: Slightly more CPU time per sample
// Good for: Noisy environments or low-quality sensors

