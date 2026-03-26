// Pin Definitions
const int PWM_PIN = 18;  // Connect to Cytron PWM
const int DIR_PIN = 19;  // Connect to Cytron DIR

// PWM Settings for New ESP32 Core (v3.0+)
const int pwmFreq = 5000;    // 5 kHz frequency
const int pwmResolution = 8; // 8-bit resolution (0-255)

void setup() {
  Serial.begin(115200);
  
  // New ESP32 Core syntax: 
  // ledcAttach(pin, frequency, resolution)
  ledcAttach(PWM_PIN, pwmFreq, pwmResolution);
  
  // Configure Direction Pin
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.println("ESP32 Actuator Emulator Ready (Core v3.0).");
  Serial.println("Commands: 'e' (Extend), 'r' (Retract), 's' (Stop)");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    // Clean up newline characters
    if (cmd == '\n' || cmd == '\r') return;

    if (cmd == 'e') {
      moveActuator(1.0); 
      Serial.println(">> Action: EXTENDING");
    } 
    else if (cmd == 'r') {
      moveActuator(-1.0);
      Serial.println(">> Action: RETRACTING");
    } 
    else if (cmd == 's') {
      moveActuator(0.0);
      Serial.println(">> Action: STOPPED");
    }
  }
}

void moveActuator(float command) {
  // 1. Set Direction
  if (command > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }

  // 2. Calculate PWM (0 to 255)
  int dutyCycle = abs(command) * 255;
  dutyCycle = constrain(dutyCycle, 0, 255);
  
  // 3. New ESP32 Core syntax:
  // ledcWrite(pin, dutyCycle)
  ledcWrite(PWM_PIN, dutyCycle);
}