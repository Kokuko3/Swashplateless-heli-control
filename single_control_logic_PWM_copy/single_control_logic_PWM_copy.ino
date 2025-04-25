/***
 * Rotor control logic.
 * @author Michael Raymer, Wright State University
 * @author Kyle Raymer, Yellow Springs High School
 * @version 1.1
 * @since 3/17/2024
 * @see - too many to mention.  Almost all of this code is based on
 *        Arduino library documention, examples and other sources.
 **/

#include <Arduino.h>
#include <Wire.h>                   // I2C bus
#include <AS5600.h>                 // Magnetic position sensor

// LOCAL PIN DEFINITIONS
#define THROTTLE_IN_PIN D5
#define PITCH_IN_PIN D6
#define ROLL_IN_PIN D7
#define MAIN_PWM_PIN A7      // ESC for main motor

// PWM Constants for motor control @ 490Hz
// After empirical testing, it seems the motor is responsive to power levels from 128 to 252
// after our calibration sequence.  Some online resources say that this ESC will reset when
// the duty cycle is less than 50%
#define PWM_ZERO 64   // Minimum power during calibration: 0 to 255 for analog_write
#define PWM_MIN 128   // Minimum power during flight: 0 to 255 for analog_write
#define PWM_MAX 252   // Maximum power during flight: 0 to 255 for analog_write

// All the power calculations are based on a 0 to 100 scale, which is then
// remapped for the ESC (usually 0 to 255 for analogWrite)
#define MIN_BASE_POWER 0
#define MAX_BASE_POWER 100

// Increment phase adjustment by pi/16
#define TWO_PI 6.2831853071f
#define PHASE_INC 0.196349540849f

// Variables for magnet read and motor control
AMS_5600 ams5600;       // ams5600 is the magnetic angle sensor
float pitch;            // 0.0 to 1.0

// Increment phase adjustment by pi/16
#define TWO_PI 6.2831853071f
#define PHASE_INC 0.196349540849f

float phase;            // 0.0 to TWO_PI
double angle;           // Current rotor angle
double adjustedAngle;   // Rotor angle + phase correction
byte mainPower;         // Power for the main ESC: 0 to 255
volatile unsigned long throttleRiseTime = 0;
volatile int throttle = 1000;  // µs
volatile bool newThrottle = false;
volatile unsigned long pitchRiseTime = 0;
volatile int pitchPulse = 1500;
volatile bool newPitch = false;
int Duty;

void setup() {
  
  // Set up the serial output
  Serial.begin(115200);
  Serial.println("Initializing");
  Serial.println("------------");

  // Set up PWM pins for ESCs
  pinMode(MAIN_PWM_PIN, OUTPUT);
  
  //Set up pins to take in throttle and pitch values
  pinMode(THROTTLE_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), throttleISR, CHANGE);
  pinMode(PITCH_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PITCH_IN_PIN), pitchISR, CHANGE);
  //pinMode(AUX_PWM_PIN, OUTPUT);
  
  // Set up I2C bus
  Wire.begin();  // Initialize the I2C bus
  
  // ESC Calibration Sequence
  calibrateESC();

  // Initial settings
  throttle = MIN_BASE_POWER;
  pitch = 0.0f;
  phase = 2.75f;    // Based on empirical testing, 3/22/24
    
}



void calibrateESC() {
  Serial.println("ESC Calibration Start");

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

  // Hold "min" for 4 seconds
  Serial.println("Calibrating: minimum value...");
  analogWrite(MAIN_PWM_PIN, PWM_MIN);
  delay(4000);
  Serial.println("ESC calibration complete.\n");

}



void loop() {

  static unsigned long lastPulseTime = 0;
  unsigned long now = micros();

  if (now - lastPulseTime >= 2500) { // 400 Hz
    lastPulseTime = now;

    noInterrupts();
    int localThrottle = throttle;
    int localPitchPulse = pitchPulse;
    newThrottle = false;
    newPitch = false;
    interrupts();

    angle = convertRawAngleToRadians(ams5600.getRawAngle());
    adjustedAngle = angle + phase;
    if (adjustedAngle > TWO_PI){
      adjustedAngle -= TWO_PI;
    }

    // Control law for main motor
    pitch = constrain((localPitchPulse - 1000.0) / 1000.0, 0.0, 1.0);
    int baseThrottle = map(localThrottle, 1000, 2000, 1, 100);
    mainPower = baseThrottle + (pitch * sin(adjustedAngle) * baseThrottle);
    Duty = map(mainPower, 0, 100, PWM_MIN, PWM_MAX);
    
    // Send PWM signal to ESC
    analogWrite(MAIN_PWM_PIN, Duty);
    Serial.println(Duty);
  }
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


