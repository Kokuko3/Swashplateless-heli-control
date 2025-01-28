#define INPUT_PIN 3
#define OUTPUT_PIN 7 
#define ONESHOT_ZERO 0  
#define ONESHOT_MIN 125
#define ONESHOT_MAX 250

void setup() {
  
  // Set up the serial output
  Serial.begin(115200); 
  Serial.println("Initializing");
  Serial.println("------------");

  // Set up ONESHOT pins for ESCs
  pinMode(OUTPUT_PIN, OUTPUT);

  // Set up input PIN from FC
  pinMode(INPUT_PIN, INPUT);
  
  // ESC Calibration Sequence
  Serial.println("Calibrating ESC...");

  // ONESHOT_ZERO = "no signal", 4 seconds
  Serial.println("Calibrating: zero signal...");
  pulseOut(OUTPUT_PIN, ONESHOT_ZERO);
  delay(4000);

  // Raise the "stick" from zero to ONESHOT_MAX
  Serial.println("Calibrating: raising stick...");
  for (byte i = ONESHOT_MIN; i <= ONESHOT_MAX; i++){
    pulseOut(OUTPUT_PIN, i);
    delay(10);
  }

  // Lower the "stick" back to min
  Serial.println("Calibrating: lowering stick...");
  for (byte i = ONESHOT_MAX; i >= ONESHOT_MIN; i--){
    pulseOut(OUTPUT_PIN, i);
    delay(10);
  }

  // Hold "min" for 10 seconds
  Serial.println("Calibrating: minimum value...");
  pulseOut(OUTPUT_PIN, ONESHOT_MIN);
  delay(10000);
  Serial.println("ESC calibration complete.\n");
    
}  // END setup()

void loop() {
  // Read the digital signal from the input pin
  int throttle = pulseIn(INPUT_PIN, HIGH);
  int throttle_percentage = map(throttle, 124, 250, 0, 100);
  Serial.print("INPUT:");
  Serial.println(throttle);

  // Write the same signal to the output pin
  pulseOut(OUTPUT_PIN, throttle);
  Serial.print("OUTPUT:");
  Serial.println(OUTPUT_PIN);
}

void pulseOut(int pin, int us)
{
    digitalWrite(pin, HIGH);
    us = max(us - 5, 1);
    delayMicroseconds(us);
    digitalWrite(pin, LOW);
}
