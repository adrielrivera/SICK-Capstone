// TiM1xx Test - Simple LiDAR Detection
// Only handles TiM100 and TiM150 detection for testing

const int TIM100_PIN = 8;  // TiM100 (Left) detection input
const int TIM150_PIN = 9;  // TiM150 (Right) detection input

// Individual LiDAR Status
bool tim100_detected = false;
bool tim150_detected = false;
int lastTim100State = LOW;
int lastTim150State = LOW;

void setup() {
  Serial.begin(115200);
  
  // Pin setup
  pinMode(TIM100_PIN, INPUT);  // TiM100 (Left) - no pull-up needed
  pinMode(TIM150_PIN, INPUT);  // TiM150 (Right) - no pull-up needed
  
  Serial.println("========================================");
  Serial.println("TiM1xx Test - Simple LiDAR Detection");
  Serial.println("========================================");
  Serial.println("TiM100 Pin 8, TiM150 Pin 9");
  Serial.println("Rising edge = Person detected");
  Serial.println("Falling edge = Area clear");
  Serial.println("========================================");
}

void loop() {
  // Check TiM100 status
  checkTim100Status();
  
  // Check TiM150 status
  checkTim150Status();
  
  // Small delay to prevent overwhelming serial output
  delay(10);
}

void checkTim100Status() {
  int currentState = digitalRead(TIM100_PIN);
  
  // Detect rising edge (LOW to HIGH) - person detected
  if (currentState == HIGH && lastTim100State == LOW) {
    tim100_detected = true;
    Serial.println("🚨 TiM100 DETECTED - Person on LEFT side");
  }
  // Detect falling edge (HIGH to LOW) - person cleared
  else if (currentState == LOW && lastTim100State == HIGH) {
    tim100_detected = false;
    Serial.println("✅ TiM100 CLEAR - LEFT side clear");
  }
  
  lastTim100State = currentState;
}

void checkTim150Status() {
  int currentState = digitalRead(TIM150_PIN);
  
  // Detect rising edge (LOW to HIGH) - person detected
  if (currentState == HIGH && lastTim150State == LOW) {
    tim150_detected = true;
    Serial.println("🚨 TiM150 DETECTED - Person on RIGHT side");
  }
  // Detect falling edge (HIGH to LOW) - person cleared
  else if (currentState == LOW && lastTim150State == HIGH) {
    tim150_detected = false;
    Serial.println("✅ TiM150 CLEAR - RIGHT side clear");
  }
  
  lastTim150State = currentState;
}
