#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>                   

#define THROTTLE_IN_PIN D5
#define PITCH_IN_PIN D6
#define ROLL_IN_PIN D7
#define OUTPUT_PIN D8

#define PWM_MIN 1000    
#define PWM_MAX 2000 
#define LOOP_INTERVAL 500 // 2 KHz

AMS_5600 ams5600;

float pitch;       
float roll;       
float phase = 2.75f; // Static for now
double angle;
double adjustedAngle;

volatile unsigned long throttleRiseTime = 0;
volatile int throttle = 1500;  // µs
volatile bool newThrottle = false;
volatile unsigned long pitchRiseTime = 0;
volatile int pitchPulse = 1500;
volatile bool newPitch = false;
volatile unsigned long rollRiseTime = 0;
volatile int rollPulse = 1500;
volatile bool newRoll = false;

int Duty;

void setup() {
  delay(10000);
  Serial.begin(115200);
  Serial.println("Initializing...");

  Wire.begin();
  ams5600setup();

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(THROTTLE_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), throttleISR, CHANGE);
  pinMode(PITCH_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PITCH_IN_PIN), pitchISR, CHANGE);
  pinMode(ROLL_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ROLL_IN_PIN), rollISR, CHANGE);

  Serial.println(">> Sending MAX throttle (2000 µs)");
  sendPWMPulse(2000);
  delay(4000); // Hold MAX for 4 seconds

  Serial.println(">> Sending MIN throttle (1000 µs)");
  sendPWMPulse(1000);
  delay(4000); // Hold MIN for 4 seconds

  Serial.println("Calibration Complete.");
}

void loop() {
  static unsigned long lastPulseTime = 0;
  unsigned long now = micros();

  if (now - lastPulseTime >= LOOP_INTERVAL) {
    lastPulseTime = now;

    // Same as before: update pitch/throttle
    noInterrupts();
    int localThrottle = throttle;
    int localPitchPulse = pitchPulse;
    int localRollPulse = rollPulse;
    newThrottle = false;
    newPitch = false;
    newRoll = false;
    interrupts();

    angle = convertRawAngleToRadians(ams5600.getRawAngle());
    adjustedAngle = angle + phase;
    if (adjustedAngle > TWO_PI){
      adjustedAngle -= TWO_PI;
    }

    pitch = constrain((localPitchPulse - 1000.0) / 1000.0, 0.0, 1.0);
    roll = constrain((localRollPulse - 1000.0) / 1000.0, 0.0, 1.0);
    float throttle_scale = map(localThrottle, PWM_MIN, PWM_MAX, 0, 100);  // More symmetric around 50
    float modulation = pitch * sin(adjustedAngle);  // -1 to 1 range
    float modulatedPower = throttle_scale * (1.0 + 0.8 * modulation); // ±50% range
    Duty = constrain(map(modulatedPower, 0, 100, PWM_MIN, PWM_MAX), PWM_MIN, PWM_MAX);

    sendPWMPulse(Duty);
    Serial.print("Power: ");
    Serial.print(modulatedPower);
    Serial.print("  |  Pitch: ");
    Serial.println(pitch);
  }
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

void sendPWMPulse(int pulseWidth) {
  digitalWrite(OUTPUT_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(OUTPUT_PIN, LOW);
}

float convertRawAngleToRadians(word newAngle) {
  return newAngle * 0.001533981;
}

void ams5600setup() {
  if (ams5600.detectMagnet() == 0) {
    while (1) {
      if (ams5600.detectMagnet() == 1) {
        Serial.print("Current Magnitude: ");
        Serial.println(ams5600.getMagnitude());
        break;
      } else {
        Serial.println("Cannot detect magnet");
        delay(1000);
      }
    }
  }
}

