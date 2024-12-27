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
#include "HX711.h"                  // Load cell amplifier
#include <IRremote.hpp>             // Ifrared remote
#include "PinDefinitionsAndMore.h"  // For IR remote

#define DECODE_NEC                   // Protocol for IR remote

// LOCAL PIN DEFINITIONS
#define MAIN_PWM_PIN 9      // ESC for main motor
//#define AUX_PWM_PIN 10      // ESC for auxilliary motor
#define LIFT_DATA_PIN 7     // Data pin for lift load cell
#define LIFT_CLOCK_PIN 8    // Clock pin for lift load cell
#define PITCH_DATA_PIN 12   // Data pin for pitch load cell
#define PITCH_CLOCK_PIN 4   // Clock pin for pitch load cell

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
#define MIN_AUX_POWER 4

// Increment phase adjustment by pi/16
#define TWO_PI 6.2831853071f
#define PHASE_INC 0.196349540849f

// Report interval in ms
#define REPORT_INTERVAL 1000

// Report RPMs every RPM_INTERVAL reports
#define RPM_INTERVAL 10

// Variables for magnet read and motor control
AMS_5600 ams5600;       // ams5600 is the magnetic angle sensor
float pitch;            // 0.0 to 1.0
float phase;            // 0.0 to TWO_PI
double angle;           // Current rotor angle
double adjustedAngle;   // Rotor angle + phase correction
double lastAngle;       // for RPM tracking - see below
int revolutions;        // for RPM tracking
double rpms;            // calculated rpms
byte basePower;         // Current base power from 1 to 100
//byte auxPower;          // Power for the aux ESC: 0 to 255
byte mainPower;         // Power for the main ESC: 0 to 255
//byte rawAuxPower;       // AUX power from 0 to 100
byte rawMainPower;      // Main power from 0 to 100

// Variables for load cell read
HX711 pitchScale;       // Bottom load cell measures pitch force
HX711 liftScale;        // Top load cell measures lift

// We'll capture load cell readings intermittently
double pitchScaleReading;     // Pitch scale reading
double liftScaleReading;      // Lift scale reading
double pitchGrams;            // Pitch force in grams
double liftGrams;             // Lift force in grams
unsigned long lastReport = 0; // Time (in ms) of last report
unsigned long currentTime;    // Current time in ms
unsigned currentReport;       // Number of current report

void setup() {
  
  // Set up the serial output
  Serial.begin(115200);
  Serial.println("Initializing");
  Serial.println("------------");

  // Set up PWM pins for ESCs
  pinMode(MAIN_PWM_PIN, OUTPUT);
  //pinMode(AUX_PWM_PIN, OUTPUT);
  
  // Set up I2C bus
  Wire.begin();  // Initialize the I2C bus

  // Set up load cell reading
  pitchScale.begin(PITCH_DATA_PIN, PITCH_CLOCK_PIN);
  liftScale.begin(LIFT_DATA_PIN, LIFT_CLOCK_PIN);
  pitchScale.set_scale(2000.0);       // TODO you need to calibrate this yourself.
  liftScale.set_scale(2000.0);        // TODO you need to calibrate this yourself.
  pitchScale.tare(0);                 // Tare (set 0 level) for pitch load cell
  liftScale.tare(0);                  // Tare the lift load cell

  // Start the IR remote receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
// ESC Calibration Sequence
  Serial.println("Calibrating ESC...");

  // PWM_ZERO = "no signal", 4 seconds
  Serial.println("Calibrating: zero signal...");
  analogWrite(MAIN_PWM_PIN, PWM_ZERO);
  //analogWrite(AUX_PWM_PIN, PWM_ZERO);
  delay(4000);

  // Raise the "stick" from zero to PWM_MAX
  Serial.println("Calibrating: raising stick...");
  for (byte i = PWM_MIN; i <= PWM_MAX; i++){
    analogWrite(MAIN_PWM_PIN, i);
    //analogWrite(AUX_PWM_PIN, i);
    delay(10);
  }

  // Lower the "stick" back to min
  Serial.println("Calibrating: lowering stick...");
  for (byte i = PWM_MAX; i >= PWM_MIN; i--){
    analogWrite(MAIN_PWM_PIN, i);
    //analogWrite(AUX_PWM_PIN, i);
    delay(10);
  }

  // Hold "min" for 10 seconds
  Serial.println("Calibrating: minimum value...");
  analogWrite(MAIN_PWM_PIN, PWM_MIN);
  //analogWrite(AUX_PWM_PIN, PWM_MIN);
  delay(10000);
  Serial.println("ESC calibration complete.\n");

// Initial settings
  basePower = MIN_BASE_POWER;
  pitch = 0.0f;
  phase = 2.75f;    // Based on empirical testing, 3/22/24
  revolutions = 0;
  currentReport = 0;
    
}  // END setup()

void loop() {
  
  // Check for IR input
  if (IrReceiver.decode()) {
    IrReceiver.resume(); // Enable receiving of the next value

    // If there is input, decode and change settings as needed
    switch (IrReceiver.decodedIRData.command) {

      // Up arrow
      case 0x9:
          if (basePower < MAX_BASE_POWER)
            basePower++;
          break;

      // Down arrow
      case 0x7:
          if (basePower > MIN_BASE_POWER)
            basePower--;
          break;

      // Right arrow (skip forward)
      case 0x44:
          if (phase >= PHASE_INC){
            phase -= PHASE_INC;
          }
          break;

      // Left arrow (skip back)
      case 0x43:
          if (phase <= TWO_PI - PHASE_INC){
            phase += PHASE_INC;
          }
          break;

      // Number 0
      case 0x16:
          pitch = 0.0f;
          break;

      // Number 1
      case 0xC:
          pitch = 0.1f;
          break;

      // Number 2
      case 0x18:
          pitch = 0.2f;
          break;

      // Number 3
      case 0x5E:
          pitch = 0.3f;
          break;

      // Number 4
      case 0x8:
          pitch = 0.4f;
          break;

      // Number 5
      case 0x1C:
          pitch = 0.5f;
          break;

      // Number 6
      case 0x5A:
          pitch = 0.6f;
          break;

      // Number 7
      case 0x42:
          pitch = 0.7f;
          break;

      // Number 8
      case 0x52:
          pitch = 0.8f;
          break;

      // Number 9
      case 0x4A:
          pitch = 0.9f;
          break;

      // Power button
      case 0x45:
          Serial.println("STOP");
          break;
    }

    Serial.println("Power: " + String(basePower, DEC));
    Serial.println("Pitch: " + String(pitch, 2));
    Serial.println("Phase correction: " + String(phase, 2));
  }
  
  // Get current rotor angle & elapsed time
  currentTime = millis();
  lastAngle = angle;
  angle = convertRawAngleToRadians(ams5600.getRawAngle());

  // Track revolutions. The rotor is running "backwards" relative to the magnet,
  // so the angle grows smaller every reading.  If it gets bigger, we passed 2*PI
  // so increment the revolution count  (unless the change is very small, which
  // generally means we are sitting around 0/2*PI and oscillating):
  if ((lastAngle > angle) && ((angle - lastAngle) < -0.01)) revolutions++;

  // Phase adjustment so we can pitch in any direction
  adjustedAngle = angle + phase;
  if (adjustedAngle > TWO_PI){
    adjustedAngle -= TWO_PI;
  }
  
  // Control law for aux motor
  // rawAuxPower = (basePower * pitch) + (sin(adjustedAngle) * pitch * basePower);
  // rawAuxPower *= 20.0f/9.0f;
  // if (basePower >= MIN_AUX_POWER)
  //  rawAuxPower += MIN_AUX_POWER; // To prevent stopping the motor at low speeds
  // auxPower = map(rawAuxPower, 0, 100, PWM_MIN, PWM_MAX);


  // Control law for main motor
  rawMainPower = basePower + (pitch * sin(adjustedAngle) * basePower);
  mainPower = map(rawMainPower, 0, 100, PWM_MIN, PWM_MAX);
  
  // Send PWM signal to ESC
  analogWrite(MAIN_PWM_PIN, mainPower);
  //analogWrite(AUX_PWM_PIN, auxPower);

  // Read load cells if they are ready.
  // Grams calculations are based on empirical calibration experiments
  // done on each load cell.
  if (pitchScale.is_ready()) {
    pitchScaleReading = pitchScale.get_units(1);
    pitchGrams = (-2.14277 * pitchScaleReading - 0.25167) * -1.0;   // Conversion to grams for pitch cell
  }
  if (liftScale.is_ready()) {
    liftScaleReading = liftScale.get_units(1);
    liftGrams = (1.880629 * liftScaleReading - 0.265621);  // Conversion to grams for lift cell
  }

  // Report load cell readings every REPORT_INTERVAL ms
  // Report RPMs every RPM_INTERVAL reports, or (RPM_INTERVAL * REPORT_INTERVAL) ms
  if (currentTime - lastReport >= REPORT_INTERVAL){
    Serial.print(String(pitchGrams, 2) + "\t" + String(liftGrams, 2));
    if (currentReport % RPM_INTERVAL == 0) {
      rpms = (double) revolutions * (1000.0 / ((double) REPORT_INTERVAL * (double) RPM_INTERVAL)) * 60;
      Serial.print("\t" + String(rpms, 2));
      revolutions = 0;
    }
    Serial.println();
    lastReport = currentTime;
    currentReport++;
  }
}

/*******************************************************
/* Function: convertRawAngleToRadians
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable radians as float
/* Description: takes the raw angle and calculates
/* float value in radians.
/*******************************************************/
float convertRawAngleToRadians(word newAngle){
  // Raw angle is 0-4095 = 0.001533981 of a radian
  float retVal = newAngle * 0.001533981;
  return retVal;
}

/*******************************************************
/* Function: convertRawAngleToDegrees
/* In: angle data from AMS_5600::getRawAngle
/* Out: human readable degrees as float
/* Description: takes the raw angle and calculates
/* float value in degrees.
/*******************************************************/
float convertRawAngleToDegrees(word newAngle)
{
  // Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree
  float retVal = newAngle * 0.087890625;
  return retVal;
}
