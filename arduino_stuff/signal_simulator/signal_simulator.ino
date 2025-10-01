// arduino "PBT simulator" -> prints 0..1023 lines at ~800 Hz
// quiet baseline with noise + occasional hits (half-sine envelope)

const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

unsigned long nextSample = 0;
bool inHit = false;
float phase = 0.0;          // 0..PI for half-sine
float phaseStep = 0.0;      // controls hit duration
int baseline = 40;          // around the Pi TRIGGER_THRESHOLD (60) minus margin
int noiseAmp = 6;           // baseline noise amplitude
int peak = 500;             // will be randomised per hit
unsigned long lastHitMs = 0;
unsigned long hitGapMs = 3000; // ms between hits (randomized a bit)

void startNewHit() {
  // randomise peak and duration
  peak = random(200, 900);           // 200..900 counts ~ strength
  float hitMs = random(120, 320);    // 120..320 ms capture-like
  phase = 0.0;
  phaseStep = (3.14159265f) / (hitMs * (SAMPLES_PER_SEC / 1000.0f));
  inHit = true;
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));  // if floating just give some entropy
  nextSample = micros();
  lastHitMs = millis();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;

    // randomly start a hit every few seconds
    unsigned long ms = millis();
    if (!inHit && ms - lastHitMs > hitGapMs) {
      startNewHit();
      lastHitMs = ms;
      hitGapMs = 2500 + random(-800, 800); // vary the gap
    }

    int value = baseline + (random(-noiseAmp, noiseAmp + 1)); // noise around baseline

    if (inHit) {
      // half-sine envelope on top of baseline
      float env = sinf(phase); // 0..1
      int hitAdd = (int)(env * (peak - baseline));
      value = baseline + hitAdd;
      phase += phaseStep;
      if (phase >= 3.14159265f) {
        inHit = false; // hit ended
      }
    }

    // clamp 0..1023
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;

    Serial.println(value);
  }
}

//67th line
