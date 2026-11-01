/*
  Arduino-Based Ultrasonic Radar System
  EEE 3100 (Electronics Shop Practice), Department of EEE, RUET

  A servo sweeps an HC-SR04 sensor across an arc. The angle and the distance of
  each reading are sent to a PC over serial for a radar display. A second,
  fixed HC-SR04 estimates the approach velocity of an object, and an RCWL-0516
  microwave sensor reports motion.

  The pins, the sweep range, the filter constants, the classification
  thresholds and the serial format follow the project report.

  Serial output (115200 baud), one line per servo step:
      smoothed_angle,distance_cm,velocity_m_s,rcwl_state,label

    smoothed_angle  servo angle after the Kalman filter, in degrees
    distance_cm     distance seen by the sweep sensor, 0 if no object in range
    velocity_m_s    approach speed from the fixed sensor, positive = approaching
    rcwl_state      1 while the RCWL-0516 output is HIGH, otherwise 0
    label           STATIC, SLOW, BIRD or VEHICLE (from the speed)
*/

#include <Servo.h>
#include <math.h>

// ---------------- Pins ----------------
const uint8_t TRIG1 = 10;      // HC-SR04 #1, on the servo (sweep)
const uint8_t ECHO1 = 11;
const uint8_t TRIG2 = 6;       // HC-SR04 #2, fixed (velocity)
const uint8_t ECHO2 = 7;
const uint8_t SERVO_PIN = 12;  // SG90 signal
const uint8_t RCWL_PIN = 2;    // RCWL-0516 OUT (external interrupt INT0)

// ---------------- Sweep ----------------
const int ANGLE_MIN = 15;                  // degrees
const int ANGLE_MAX = 165;                 // degrees
const int ANGLE_STEP = 1;                  // degrees per step
const unsigned int SETTLE_MS = 25;         // wait for the servo to settle

// ---------------- Ranging ----------------
const float SWEEP_MIN_CM = 2.0;            // objects closer than this are ignored
const float SWEEP_MAX_CM = 40.0;           // objects farther than this are ignored
const unsigned long SWEEP_TIMEOUT_US = 3500UL;    // about 60 cm of echo time
const float VEL_MAX_CM = 200.0;            // range of the fixed velocity sensor
const unsigned long VEL_TIMEOUT_US = 12000UL;     // about 200 cm of echo time
const float CM_PER_US = 0.0343;            // speed of sound, 343 m/s

// ---------------- Kalman filter (angle) ----------------
const float KALMAN_Q = 0.01;               // process noise
const float KALMAN_R = 0.1;                // measurement noise

// ---------------- Classification (m/s) ----------------
const float V_STATIC = 0.3;                // below this: STATIC
const float V_SLOW = 0.8;                  // below this: SLOW
const float V_BIRD = 2.5;                  // below this: BIRD, above: VEHICLE

Servo sweepServo;

int angle = ANGLE_MIN;
int dir = ANGLE_STEP;

float kalmanX = ANGLE_MIN;                 // filtered angle
float kalmanP = 1.0;                       // estimate error

float prevDist = -1.0;                     // previous reading of the fixed sensor
unsigned long prevTime = 0;

volatile bool motion = false;

// Called on every change of the RCWL-0516 output.
void onMotion() {
  motion = digitalRead(RCWL_PIN);
}

// Distance in cm from one HC-SR04, or -1.0 if there is no valid echo.
float readDistanceCm(uint8_t trig, uint8_t echo, unsigned long timeoutUs, float maxCm) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  unsigned long us = pulseIn(echo, HIGH, timeoutUs);
  if (us == 0) {
    return -1.0;
  }
  float cm = us * CM_PER_US / 2.0;     // the pulse travels to the object and back
  if (cm < SWEEP_MIN_CM || cm > maxCm) {
    return -1.0;
  }
  return cm;
}

// Approach velocity in m/s from two consecutive readings: v = (d_prev - d_curr) / dt
float readVelocity() {
  float d = readDistanceCm(TRIG2, ECHO2, VEL_TIMEOUT_US, VEL_MAX_CM);
  unsigned long now = micros();
  float v = 0.0;
  if (d > 0 && prevDist > 0) {
    float dt = (now - prevTime) / 1000000.0;
    if (dt > 0) {
      v = ((prevDist - d) / 100.0) / dt;
    }
  }
  prevDist = d;
  prevTime = now;
  return v;
}

// One step of a scalar Kalman filter on the servo angle.
float kalmanFilter(float measured) {
  kalmanP += KALMAN_Q;
  float gain = kalmanP / (kalmanP + KALMAN_R);
  kalmanX += gain * (measured - kalmanX);
  kalmanP *= (1.0 - gain);
  return kalmanX;
}

// Class of the object from the size of its speed.
const char* classify(float v) {
  float s = fabs(v);
  if (s < V_STATIC) return "STATIC";
  if (s < V_SLOW) return "SLOW";
  if (s < V_BIRD) return "BIRD";
  return "VEHICLE";
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  digitalWrite(TRIG1, LOW);
  digitalWrite(TRIG2, LOW);

  pinMode(RCWL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RCWL_PIN), onMotion, CHANGE);

  sweepServo.attach(SERVO_PIN);
  sweepServo.write(ANGLE_MIN);
  delay(500);                          // let the servo reach the start position
}

void loop() {
  sweepServo.write(angle);
  delay(SETTLE_MS);

  float d1 = readDistanceCm(TRIG1, ECHO1, SWEEP_TIMEOUT_US, SWEEP_MAX_CM);
  delay(10);                           // keep the two sensors from hearing each other
  float v = readVelocity();
  float smoothed = kalmanFilter(angle);

  Serial.print(smoothed, 1);
  Serial.print(',');
  Serial.print(d1 > 0 ? d1 : 0.0, 1);
  Serial.print(',');
  Serial.print(v, 2);
  Serial.print(',');
  Serial.print(motion ? 1 : 0);
  Serial.print(',');
  Serial.println(classify(v));

  // Sweep back and forth between ANGLE_MIN and ANGLE_MAX.
  angle += dir;
  if (angle >= ANGLE_MAX) {
    angle = ANGLE_MAX;
    dir = -ANGLE_STEP;
  } else if (angle <= ANGLE_MIN) {
    angle = ANGLE_MIN;
    dir = ANGLE_STEP;
  }
}
