const int inputPin = 3;   
const int outputPin = 9;  

void setup() {
  pinMode(inputPin, INPUT); 
  pinMode(outputPin, OUTPUT); 
}

void loop() {
  unsigned long pulseWidth = pulseIn(inputPin, HIGH, 500); // Timeout after 500 microseconds
  Serial.println(pulseWidth);
  if (pulseWidth >= 124 && pulseWidth <= 251) {
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(outputPin, LOW);
  }
}
