// Pin Definitions for Teensy 4.1
const int PWM_PIN = 19;  // PWM pin
const int DIR_PIN = 17;  // Direction pin

// PWM Settings
const int pwmFreq = 5000;    // 5 kHz frequency
const int pwmResolution = 8; // 8-bit resolution (0-255)

void setup() {
  Serial.begin(115200);
  
  // Configure Pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Set PWM Frequency and Resolution for Teensy
  analogWriteFrequency(PWM_PIN, pwmFreq);
  analogWriteResolution(pwmResolution);
  
  Serial.println("Teensy 4.1 Actuator Emulator Ready.");
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

  // 2. Calculate PWM (0 to 255 for 8-bit)
  int dutyCycle = abs(command) * 255;
  dutyCycle = constrain(dutyCycle, 0, 255);
  
  // 3. Standard Arduino/Teensy PWM command
  analogWrite(PWM_PIN, dutyCycle);
}
