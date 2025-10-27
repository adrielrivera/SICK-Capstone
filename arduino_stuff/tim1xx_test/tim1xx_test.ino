// TiM1xx Test - Simple LiDAR Detection
// Only handles TiM100 and TiM150 detection for testing

const int TIM100_PIN = 8;  // TiM100 (Left) detection input
const int TIM150_PIN = 9;  // TiM150 (Right) detection input

// Individual LiDAR Status
bool tim100_detected = false;
bool tim150_detected = false;

// Status reporting timing
unsigned long lastStatusReport = 0;
const unsigned long STATUS_INTERVAL = 500;  // Report every 500ms (0.5 seconds)

void setup() {
  Serial.begin(115200);
  
  // Pin setup - using simple INPUT like the working code
  pinMode(TIM100_PIN, INPUT);  // TiM100 (Left) - external pull-down resistor to GND
  pinMode(TIM150_PIN, INPUT);  // TiM150 (Right) - external pull-down resistor to GND
  
  Serial.println("========================================");
  Serial.println("TiM1xx Test - Simple LiDAR Detection");
  Serial.println("========================================");
  Serial.println("TiM100 Pin 8, TiM150 Pin 9");
  Serial.println("HIGH = Person detected, LOW = Area clear");
  Serial.println("========================================");
}

void loop() {
  // Read current pin states (like the working code)
  int tim100_state = digitalRead(TIM100_PIN);
  int tim150_state = digitalRead(TIM150_PIN);
  
  // Update detection status based on current state
  tim100_detected = (tim100_state == HIGH);
  tim150_detected = (tim150_state == HIGH);
  
  // Report status every 0.5 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastStatusReport >= STATUS_INTERVAL) {
    reportStatus();
    lastStatusReport = currentTime;
  }
  
  // Small delay to prevent overwhelming serial output
  delay(10);
}

// Removed edge detection functions - now using simple state reading like the working code

void reportStatus() {
  Serial.print("TiM100: ");
  Serial.print(tim100_detected ? "DETECTED" : "CLEAR");
  Serial.print(" | TiM150: ");
  Serial.println(tim150_detected ? "DETECTED" : "CLEAR");
}
