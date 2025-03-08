#define INPUT_PIN A1
#define OUTPUT_PIN A6
#define ONESHOT_ZERO 125 
#define ONESHOT_MIN 127    
#define ONESHOT_MAX 250    

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);

  Serial.println("ESC Calibration...");

  // Send idle (zero throttle)
  sendOneShotPulse(ONESHOT_ZERO);
  delay(4000);

  // Ramp-up
  for (int i = ONESHOT_MIN; i <= ONESHOT_MAX; i++) {
    sendOneShotPulse(i);
    delay(10);
  }

  // Ramp-down
  for (int i = ONESHOT_MAX; i >= ONESHOT_MIN; i--) {
    sendOneShotPulse(i);
    delay(10);
  }

  // Hold min
  sendOneShotPulse(ONESHOT_MIN);
  delay(10000);
  Serial.println("ESC calibration complete.");
}

// Function to generate precise OneShot125 pulses
void sendOneShotPulse(int pulseWidth) {
  digitalWrite(OUTPUT_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(OUTPUT_PIN, LOW);
}


void loop() {
  int throttle = pulseIn(INPUT_PIN, HIGH, 50000); // Read pulse width in µs

  Serial.print("INPUT: ");
  Serial.println(throttle);

  // Ensure throttle is in range and send correct pulse
  if (throttle < ONESHOT_MIN) {
    sendOneShotPulse(ONESHOT_ZERO);
  } else {
    sendOneShotPulse(throttle);
  }
}

