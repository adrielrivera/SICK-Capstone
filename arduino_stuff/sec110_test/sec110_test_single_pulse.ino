// SEC110 Trigger Test - Single Pulse on Startup
// Outputs one clean rising edge pulse when Arduino starts
// Useful for testing if SEC110 responds to a single trigger

const int OUTPUT_PIN = 7;  // Pin to output clean signal (change as needed)
const unsigned long PULSE_WIDTH_US = 1000;  // Pulse width in microseconds (adjust as needed)

void setup() {
  Serial.begin(9600);
  pinMode(OUTPUT_PIN, OUTPUT);
  
  // Start with output LOW (0V)
  digitalWrite(OUTPUT_PIN, LOW);
  
  Serial.println("# SEC110 Trigger Test - Single Pulse");
  Serial.println("# Output pin: " + String(OUTPUT_PIN));
  Serial.println("# Will output ONE 0→5V pulse on startup");
  Serial.println("# Pulse width: " + String(PULSE_WIDTH_US) + " microseconds");
  
  delay(500);  // Wait 500ms after startup
  
  // Generate clean rising edge pulse
  Serial.println("# Generating pulse...");
  digitalWrite(OUTPUT_PIN, HIGH);  // Rising edge (0V → 5V)
  delayMicroseconds(PULSE_WIDTH_US);  // Hold HIGH
  digitalWrite(OUTPUT_PIN, LOW);   // Falling edge (5V → 0V)
  
  Serial.println("# Pulse complete!");
  Serial.println("# To retrigger, press RESET button on Arduino");
}

void loop() {
  // Do nothing - just wait
  // To trigger again, reset Arduino
  delay(1000);
}

