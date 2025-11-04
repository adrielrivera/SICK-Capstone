// SEC110 Trigger Test - Clean 0→5V Rising Edge Output
// Simple test to verify Arduino can trigger SEC110
// Outputs a clean rising edge on pin 7

const int OUTPUT_PIN = 7;  // Pin to output clean signal (change as needed)

void setup() {
  Serial.begin(9600);
  pinMode(OUTPUT_PIN, OUTPUT);
  
  // Start with output LOW (0V)
  digitalWrite(OUTPUT_PIN, LOW);
  
  Serial.println("# SEC110 Trigger Test");
  Serial.println("# Output pin: " + String(OUTPUT_PIN));
  Serial.println("# Will output 0→5V rising edge every 2 seconds");
  Serial.println("# Ready...");
  delay(1000);
}

void loop() {
  // Output LOW (0V)
  digitalWrite(OUTPUT_PIN, LOW);
  Serial.println("LOW (0V)");
  delay(1000);  // Hold LOW for 1 second
  
  // Output HIGH (5V) - Rising edge
  digitalWrite(OUTPUT_PIN, HIGH);
  Serial.println("HIGH (5V) - RISING EDGE");
  delay(1000);  // Hold HIGH for 1 second
  
  // Repeat every 2 seconds
}

