#define INPUT_PIN 3
#define OUTPUT_PIN 7 
#define ONESHOT_ZERO 125  
#define ONESHOT_MIN 127
#define ONESHOT_MAX 250
#define ONESHOT_FREQ 4000

void setup() {
  
  // Set up the serial output
  Serial.begin(115200); 
  Serial.println("Initializing");
  Serial.println("------------");

  // Set up ONESHOT pins for ESCs
  analogWriteFrequency(OUTPUT_PIN, ONESHOT_FREQ);

  // Set up input PIN from FC
  pinMode(INPUT_PIN, INPUT);
  
  // ESC Calibration Sequence
  Serial.println("Calibrating ESC...");

  // ONESHOT_ZERO = "no signal", 4 seconds
  Serial.println("Calibrating: zero signal...");
  analogWrite(OUTPUT_PIN, map(ONESHOT_ZERO, 125, 250, 0, 255));
  delay(4000);

  // Raise the "stick" from zero to ONESHOT_MAX
  Serial.println("Calibrating: raising stick...");
  for (byte i = ONESHOT_MIN; i <= ONESHOT_MAX; i++){
    analogWrite(OUTPUT_PIN, map(i, 125, 250, 0, 255));
    delay(10);
  }

  // Lower the "stick" back to min
  Serial.println("Calibrating: lowering stick...");
  for (byte i = ONESHOT_MAX; i >= ONESHOT_MIN; i--){
    analogWrite(OUTPUT_PIN, map(i, 125, 250, 0, 255));
    delay(10);
  }

  // Hold "min" for 10 seconds
  Serial.println("Calibrating: minimum value...");
  analogWrite(OUTPUT_PIN, map(ONESHOT_MIN, 125, 250, 0, 255));
  delay(10000);
  Serial.println("ESC calibration complete.\n");
    
}  // END setup()

void loop() {
  // Read the digital signal from the input pin
  int throttle = pulseIn(INPUT_PIN, HIGH);
  int dutyCycle = map(throttle, 125, 250, 0, 255);
  Serial.print("INPUT:");
  Serial.println(throttle);

  // Write the same signal to the output pin
  analogWriteFrequency(OUTPUT_PIN, ONESHOT_FREQ);
  analogWrite(OUTPUT_PIN, dutyCycle);
}
