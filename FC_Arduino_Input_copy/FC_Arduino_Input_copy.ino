#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>                   

#define THROTTLE_IN_PIN D5
#define PITCH_IN_PIN A1
#define ROLL_IN_PIN A2
#define OUTPUT_PIN D4

#define ONESHOT_MIN 125    
#define ONESHOT_MAX 250 

#define LEDC_CHANNEL 0
#define LEDC_TIMER_BIT 16  // resolution (2^16 = 65536 steps)
#define LEDC_BASE_FREQ 4000 // Frequency (can be very low — we use pulse width)

AMS_5600 ams5600;

float pitch = 0.8;         // Static for now
float phase = 2.75f;
double angle;
double adjustedAngle;

volatile unsigned long throttleRiseTime = 0;
volatile int throttle = 150;  // µs
volatile bool newThrottle = false;
volatile unsigned long pitchRiseTime = 0;
volatile int pitchPulse = 1500;
volatile bool newPitch = false;

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
  ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(OUTPUT_PIN, LEDC_CHANNEL);

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
  static unsigned long lastPulseTime = 0;
  unsigned long now = micros();

  if (now - lastPulseTime >= 2500) { // 400 Hz
    lastPulseTime = now;

    // Same as before: update pitch/throttle
    noInterrupts();
    int localThrottle = throttle;
    int localPitchPulse = pitchPulse;
    newThrottle = false;
    newPitch = false;
    interrupts();

    angle = convertRawAngleToRadians(ams5600.getRawAngle());
    adjustedAngle = angle + phase;
    if (adjustedAngle > TWO_PI) adjustedAngle -= TWO_PI;

    pitch = constrain((localPitchPulse - 1000.0) / 1000.0, 0.0, 1.0);
    float throttle_scale = map(localThrottle, ONESHOT_MIN, ONESHOT_MAX, 1, 100);
    float power = throttle_scale + (pitch * sin(adjustedAngle) * throttle_scale);
    Duty = constrain(map(power, 0, 200, ONESHOT_MIN, ONESHOT_MAX), ONESHOT_MIN, ONESHOT_MAX);

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

void sendOneShotPulse(int pulseWidthMicros) {
  int duty = map(pulseWidthMicros, 0, 1000, 0, 65535); // For 1ms max range
  ledcWrite(LEDC_CHANNEL, duty);
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

