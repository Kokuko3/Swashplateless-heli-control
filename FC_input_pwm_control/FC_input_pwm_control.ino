/***
 * Rotor control logic.
 * @author Michael Raymer, Wright State University
 * @author Kyle Raymer, Yellow Springs High School
 * @author Mason McDaniel, Wright State University
 * @version 1.2
 * @since 4/25/2025
 * @see - too many to mention.  Almost all of this code is based on
 *        Arduino library documention, examples and other sources.
 **/

#include <Arduino.h>
#include <Wire.h>                   // I2C bus
#include <AS5600.h>                 // Magnetic position sensor

// LOCAL PIN DEFINITIONS
#define MAIN_PWM_PIN 9   // ESC for main motor
#define THROTTLE_IN_PIN 18   // Read throttle from FC
#define PITCH_IN_PIN 2   // Read pitch from FC
#define ROLL_IN_PIN 3   // Read roll from FC
#define PWM_ZERO 64   // Minimum power during calibration: 0 to 255 for analog_write
#define PWM_MIN 128   // Minimum power during flight: 0 to 255 for analog_write
#define PWM_MAX 252   // Maximum power during flight: 0 to 255 for analog_write
#define MIN_BASE_POWER 0
#define MAX_BASE_POWER 100

#define PHASE_INC 0.196349540849f

// Variables for magnet read and motor control
AMS_5600 ams5600;       // ams5600 is the magnetic angle sensor

float phase;            // 0.0 to TWO_PI
double angle;           // Current rotor angle
double adjustedAngle;   // Rotor angle + phase correction
byte mainPower;         // Power for the main ESC: 0 to 255

float pitch;
float roll;
int Duty;
volatile int throttle;
volatile int pitchPulse;
volatile int rollPulse;

volatile unsigned long throttleRiseTime = 0;
volatile bool newThrottle = false;
volatile unsigned long pitchRiseTime = 0;
volatile bool newPitch = false;
volatile unsigned long rollRiseTime = 0;
volatile bool newRoll = false;

void setup() {
  // Set up the serial output
  Serial.begin(115200);
  Serial.println("Initializing");
  Serial.println("------------");

  // Interrupts for inputs
  pinMode(THROTTLE_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), throttleISR, CHANGE);
  pinMode(PITCH_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PITCH_IN_PIN), pitchISR, CHANGE);
  pinMode(ROLL_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ROLL_IN_PIN), rollISR, CHANGE);

  // Set up PWM pins for ESCs
  pinMode(MAIN_PWM_PIN, OUTPUT);
  
  // Set up I2C bus
  Wire.begin();  // Initialize the I2C bus
  
  // ESC Calibration Sequence
  calibrateESC();

  // Initial settings
  throttle = MIN_BASE_POWER;
  pitch = 1.0f;
  phase = 2.75f;    // Based on empirical testing, 3/22/24
    
}



void calibrateESC() {
  // ESC Calibration Sequence
  Serial.println("Calibrating ESC...");

  // PWM_ZERO = "no signal", 4 seconds
  Serial.println("Calibrating: zero signal...");
  analogWrite(MAIN_PWM_PIN, PWM_ZERO);
  delay(4000);

  // Raise the "stick" from zero to PWM_MAX
  Serial.println("Calibrating: raising stick...");
  for (byte i = PWM_MIN; i <= PWM_MAX; i++){
    analogWrite(MAIN_PWM_PIN, i);
    delay(10);
  }

  // Lower the "stick" back to min
  Serial.println("Calibrating: lowering stick...");
  for (byte i = PWM_MAX; i >= PWM_MIN; i--){
    analogWrite(MAIN_PWM_PIN, i);
    delay(10);
  }

  // Hold "min" for 10 seconds
  Serial.println("Calibrating: minimum value...");
  analogWrite(MAIN_PWM_PIN, PWM_MIN);
  delay(10000);
  Serial.println("ESC calibration complete.\n");

}



float convertRawAngleToRadians(word newAngle){
  // Raw angle is 0-4095 = 0.001533981 of a radian
  float retVal = newAngle * 0.001533981;
  return retVal;
}



void throttleISR() {
  if (digitalRead(THROTTLE_IN_PIN)) {
    throttleRiseTime = micros();
  } else {
    unsigned long fallTime = micros();
    unsigned int pulse = fallTime - throttleRiseTime;
    if (pulse >= 1000 && pulse <= 2000) {
      throttle = pulse;
      newThrottle = true;
    }
  }
}



void pitchISR() {
  if (digitalRead(PITCH_IN_PIN)) {
    pitchRiseTime = micros();
  } else {
    unsigned long fallTime = micros();
    unsigned int pulse = fallTime - pitchRiseTime;
    if (pulse >= 1000 && pulse <= 2000) {
      pitchPulse = pulse;
      newPitch = true;
    }
  }
}



void rollISR() {
  if (digitalRead(ROLL_IN_PIN)) {
    rollRiseTime = micros();
  } else {
    unsigned long fallTime = micros();
    unsigned int pulse = fallTime - rollRiseTime;
    if (pulse >= 1000 && pulse <= 2000) {
      rollPulse = pulse;
      newRoll = true;
    }
  }
}



void loop() {

  noInterrupts();
  long localThrottle = throttle;
  newThrottle = false;
  int localPitch = pitchPulse;
  newPitch = false;
  int localRoll = rollPulse;
  newRoll = false;
  interrupts();
  
  pitch = constrain((localPitch - 1000.0) / 1000.0, 0.0, 1.0);
  roll = constrain((localRoll - 1000.0) / 1000.0, -1.0, 1.0);
  long newThrottle = map(localThrottle, 1000, 2000, 0, 100);

  Serial.print("Throttle: ");
  Serial.print(newThrottle);
  Serial.print("  |  Pitch: ");
  Serial.print(pitch);
  Serial.print("  |  Roll: ");
  Serial.println(roll);

  angle = convertRawAngleToRadians(ams5600.getRawAngle());
  adjustedAngle = angle + phase;
  if (adjustedAngle > TWO_PI){
    adjustedAngle -= TWO_PI;
  }

  // Control law for main motor
  mainPower = newThrottle + (pitch * sin(adjustedAngle) * newThrottle);
  Duty = map(mainPower, 0, 100, PWM_MIN, PWM_MAX);
    
  // Send PWM signal to ESC
  analogWrite(MAIN_PWM_PIN, Duty);
  
}
