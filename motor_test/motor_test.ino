// Sabertooth PWM for Teensy without external libraries
const int motor1Pin = 0; 
const int motor2Pin = 1;

float angle = 0.0;
const float increment = 0.01; 

// For 12-bit resolution (0 to 4095)
// 1500us (Neutral) is approx 307
// 1000us (Reverse) is approx 205
// 2000us (Forward) is approx 410
const int neutral = 307;
const int amplitude = 100; 

void setup() {
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);

  // Set PWM frequency to 50Hz (Standard for RC/Sabertooth)
  analogWriteFrequency(motor1Pin, 50);
  analogWriteFrequency(motor2Pin, 50);

  // Set resolution to 12-bit for smoother motion
  analogWriteResolution(12);

  // Start at Neutral (Zero)
  analogWrite(motor1Pin, neutral);
  analogWrite(motor2Pin, neutral);
  delay(2000); 
}

void loop() {
  float sineValue = sin(angle);
  
  // Calculate output
  int motorOutput = neutral + (sineValue * amplitude);
  
  analogWrite(motor1Pin, motorOutput);
  analogWrite(motor2Pin, motorOutput);
  
  angle += increment;
  if (angle > 2 * PI) angle -= 2 * PI;
  
  delay(20); 
}