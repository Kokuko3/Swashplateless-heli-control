#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>                   

#define THROTTLE_IN_PIN A0
#define PITCH_IN_PIN A1
#define ROLL_IN_PIN A2
#define OUTPUT_PIN D6
#define ONESHOT_MIN 125    
#define ONESHOT_MAX 250  
AMS_5600 ams5600;  

void setup() {
  delay(10000);
  Serial.begin(115200);
  Serial.println("Initializing...");

  // set up Hall effect Sensor
  Wire.begin();
  ams5600setup();

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(THROTTLE_IN_PIN, INPUT);
  pinMode(PITCH_IN_PIN, INPUT);
  pinMode(ROLL_IN_PIN, INPUT);

  Serial.println("ESC Calibration...");

  Serial.println("Sending MAX throttle...");
  sendOneShotPulse(ONESHOT_MAX);
  delay(4000);  // Wait for ESC to detect max throttle

  Serial.println("Sending MIN throttle...");
  sendOneShotPulse(ONESHOT_MIN);
  delay(4000);  // Wait for ESC to confirm calibration

  Serial.println("ESC calibration complete.");
}

/*******************************************************
/* Function: sendOneShotPulse
/* In: The pulse width of the one shot signal to send 
/* Out: Writes a digital signal 
/* Description: Sends out a PWM signal within the
/* pulse width of OneShot125
/*******************************************************/
void sendOneShotPulse(int pulseWidth) {
  digitalWrite(OUTPUT_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(OUTPUT_PIN, LOW);
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

/*******************************************************
/* Function: ams5600setup
/* Description: Sets up the AMS5600 sensor 
/*******************************************************/
void ams5600setup(){
  if(ams5600.detectMagnet() == 0 ){
    while(1){
        if(ams5600.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(ams5600.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
}


void loop() {
  int throttle = pulseIn(THROTTLE_IN_PIN, HIGH, 50000); // Read pulse width in µs
  float pitch = pulseIn(PITCH_IN_PIN, HIGH, 50000); // Read pulse width in µs
  float roll = pulseIn(ROLL_IN_PIN, HIGH, 50000); // Read pulse width in µs
  throttle = map(throttle, 125, 250, 0, 100);
  pitch = 2*(pitch/2000)-1;
  roll = 2*(roll/2000)-1;
  float motorRads = convertRawAngleToRadians(ams5600.getRawAngle());
  int Duty = throttle + (pitch * cos(motorRads) * throttle) + (roll * sin(motorRads) * throttle);
  Duty = map(Duty, 0, 100, 125, 250);


  // Serial.println("----THROTTLE----PITCH----ROLL----Motor Radians-----");
  // Serial.print("----   ");
  // Serial.print(throttle);
  // Serial.print("    ----  ");
  // Serial.print(pitch);
  // Serial.print("  ---- ");
  // Serial.print(roll);
  // Serial.print("   ---- ");
  // Serial.print(String(convertRawAngleToRadians(ams5600.getRawAngle()),DEC));
  // Serial.println(" -----");

  Serial.println(Duty);

  if (throttle < 127) {
    sendOneShotPulse(0);
  } else {
    sendOneShotPulse(Duty);
  }
}

