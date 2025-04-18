#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>                   

#define THROTTLE_IN_PIN D5
#define PITCH_IN_PIN A1
#define ROLL_IN_PIN A2
#define OUTPUT_PIN D4

#define ONESHOT_MIN 125    
#define ONESHOT_MAX 250  

AMS_5600 ams5600;

float pitch = 1.0;         // Static for now
float phase = 2.75f;
double angle;
double adjustedAngle;

volatile unsigned long throttleRiseTime = 0;
volatile int throttle = 150;  // µs
volatile bool newThrottle = false;

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

  Serial.println("Calibrating: zero signal...");
  sendOneShotPulse(ONESHOT_MIN);
  delay(8000);

  // Raise the "stick" from zero to PWM_MAX
  Serial.println("Calibrating: raising stick...");
  for (byte i = ONESHOT_MIN; i <= ONESHOT_MAX; i++){
    sendOneShotPulse(i);
    delay(100);
  }

  Serial.println("Calibrating: max signal...");
  sendOneShotPulse(ONESHOT_MAX);
  delay(8000);

  // Lower the "stick" back to min
  Serial.println("Calibrating: lowering stick...");
  for (byte i = ONESHOT_MAX; i >= ONESHOT_MIN; i--){
    sendOneShotPulse(i);
    delay(100);
  }

  // Hold "min" for 10 seconds
  Serial.println("Calibrating: minimum value...");
  sendOneShotPulse(ONESHOT_MIN);
  delay(10000);
  Serial.println("ESC calibration complete.\n");
}

void loop() {
  if (newThrottle) {
    noInterrupts();       
    int localThrottle = throttle;
    newThrottle = false;
    interrupts();

    // Calculate motor angle
    angle = convertRawAngleToRadians(ams5600.getRawAngle());
    adjustedAngle = angle + phase;
    if (adjustedAngle > TWO_PI) adjustedAngle -= TWO_PI;

    // Map and modulate
    int safeThrottle = constrain(localThrottle, ONESHOT_MIN, ONESHOT_MAX);
    float throttle_scale = map(safeThrottle, ONESHOT_MIN, ONESHOT_MAX, 1, 100);
    float sineMod = pitch * 0.5f;  // 50% modulation range
    float mod = 1.0f + sineMod * sin(adjustedAngle);
    float power = throttle_scale * mod;
    Duty = constrain(map(power, 0, 200, ONESHOT_MIN, ONESHOT_MAX), ONESHOT_MIN, ONESHOT_MAX);

    // Output pulse
    sendOneShotPulse(Duty);
    Serial.println(Duty);
  }
}

void throttleISR() {
  if (digitalRead(THROTTLE_IN_PIN)) {
    throttleRiseTime = micros();
  } else {
    unsigned long fallTime = micros();
    unsigned int pulse = fallTime - throttleRiseTime;
    if (pulse >= 100 && pulse <= 300) {
      throttle = pulse;
      newThrottle = true;
    }
  }
}

void sendOneShotPulse(int pulseWidth) {
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

