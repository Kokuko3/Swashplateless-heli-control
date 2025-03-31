#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>

#define THROTTLE_IN_PIN A0
#define PITCH_IN_PIN A1
#define ROLL_IN_PIN A2
#define OUTPUT_PIN D6
AMS_5600 ams5600; 

void setup() {
  delay(10000);
  Serial.begin(115200);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(THROTTLE_IN_PIN, INPUT);
  pinMode(PITCH_IN_PIN, INPUT);
  pinMode(ROLL_IN_PIN, INPUT);
  Wire.begin();
  ams5600setup();
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

void loop() {
  int throttle = pulseIn(THROTTLE_IN_PIN, HIGH, 50000); // Read pulse width in µs
  float pitch = pulseIn(PITCH_IN_PIN, HIGH, 50000); // Read pulse width in µs
  float roll = pulseIn(ROLL_IN_PIN, HIGH, 50000); // Read pulse width in µs
  throttle = map(throttle, 125, 250, 0, 100);
  pitch = 2*(pitch/2000)-1;
  roll = 2*(roll/2000)-1;
  float motorRads = convertRawAngleToRadians(ams5600.getRawAngle());
  int Duty = throttle + (pitch * cos(motorRads) * throttle) + (roll * sin(motorRads) * throttle);
  Serial.println(Duty);
  // Serial.println(roll * sin(motorRads);
  // Serial.println(Duty);
}
