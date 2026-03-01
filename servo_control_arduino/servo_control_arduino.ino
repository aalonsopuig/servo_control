/*
===============================================================================
Title:        servo_control
Date:         2026-03-01
Author:       Alejandro Alonso Puig + ChatGPT
License:      Apache 2.0 License
-------------------------------------------------------------------------------
Description:
This sketch is a standalone servo control experiment driven by 3 potentiometers:

  - A0: Target position (0..180 deg)
  - A1: Max speed v_max (deg/s)
  - A2: Acceleration a (deg/s^2)

It implements a simple trapezoidal motion profile (ramp up / cruise / ramp down),

Key idea:
- The servo is commanded in small position steps, updated periodically.
- The step size is not fixed: it depends on a "virtual velocity" v (deg/s).
- v ramps up using acceleration a, but is automatically limited so the motion
  can still stop on time (using the classic stopping-distance rule).

Printed to PC (USB Serial):
  TargetDeg | CurrentCmdDeg | V_deg_s | A_deg_s2 | DistDeg | DeltaDeg | PWM_us

Hardware notes:
- Servo signal: SERVO_PIN (digital)
- Potentiometers: standard 0..5V dividers, wiper to A0/A1/A2, ends to 5V and GND
- Servo GND and Arduino GND MUST be common
===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// --------------------------- USER CONFIGURATION -----------------------------

#define SERVO_PIN        9     // Servo PWM output pin
#define POT_TARGET_PIN   A0    // Target position pot
#define POT_VMAX_PIN     A1    // Max speed pot
#define POT_ACCEL_PIN    A2    // Acceleration pot

#define ADC_SCALE        1023.0f
#define VREF             5.0f

#define LOOP_INTERVAL_MS 40    // Control update period (ms) ~25 Hz

// Servo PWM calibration (your HS-805BB real endpoints found experimentally)
#define SERVO_CENTER_US  1500
#define SERVO_SPAN_US     860   // +/- span around center for 0..180 mapping

#define PWM_MIN_US       (SERVO_CENTER_US - SERVO_SPAN_US)  // 0 deg
#define PWM_MAX_US       (SERVO_CENTER_US + SERVO_SPAN_US)  // 180 deg

// Ranges for the pots -> motion parameters (tune as you like)
#define VMAX_MIN_DPS     5.0f     // deg/s (slowest)
#define VMAX_MAX_DPS     250.0f   // deg/s (fastest)

#define ACCEL_MIN_DPS2   20.0f    // deg/s^2 (gentle)
#define ACCEL_MAX_DPS2   2000.0f  // deg/s^2 (aggressive)

// Small threshold to consider we reached the target (deg)
#define EPS_DEG          0.25f

// --------------------------- GLOBALS ----------------------------------------

Servo s;

static float currentCmdDeg = 0.0f;   // "virtual" commanded position we send to servo
static float v_dps         = 0.0f;   // "virtual" velocity (deg/s) used to step the command

// --------------------------- HELPERS ----------------------------------------

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float absf(float x) { return (x >= 0.0f) ? x : -x; }

static inline float map01(int adc) {
  return (float)adc / ADC_SCALE;  // 0..1
}

// Convert 0..180 degrees into PWM microseconds using your calibrated endpoints
static inline int pwmUsFromDeg(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  float us = PWM_MIN_US + (deg / 180.0f) * (PWM_MAX_US - PWM_MIN_US);
  return (int)(us + 0.5f);
}

// Read one pot and map to a float range
float readPotMapped(uint8_t pin, float outMin, float outMax) {
  int adc = analogRead(pin);                      // 0..1023
  float x = map01(adc);                           // 0..1
  return outMin + x * (outMax - outMin);          // linear map
}

// Read target pot and map to 0..180 degrees
float readTargetDeg() {
  int adc = analogRead(POT_TARGET_PIN);
  float deg = map01(adc) * 180.0f;
  return clampf(deg, 0.0f, 180.0f);
}

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(700);

  s.attach(SERVO_PIN);  // We will always drive the servo with microseconds

  // IMPORTANT: do NOT force 90 deg at startup.
  // We start by reading the target pot and immediately commanding that position.
  float targetDeg = readTargetDeg();
  currentCmdDeg = targetDeg;
  v_dps = 0.0f;

  s.writeMicroseconds(pwmUsFromDeg(currentCmdDeg));

  Serial.println("servo_control (target + v_max + accel)");
  Serial.println("Target | CurrCmd | V(d/s) | A(d/s^2) | Dist | dDeg | PWM_us");
  Serial.println("--------------------------------------------------------------");
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------

void loop() {

  // 1) Timing
  const float dt = (float)LOOP_INTERVAL_MS / 1000.0f;   // seconds per tick

  // 2) Read the three user knobs
  float targetDeg = readTargetDeg();                                     // 0..180
  float vMax_dps  = readPotMapped(POT_VMAX_PIN,  VMAX_MIN_DPS,  VMAX_MAX_DPS);
  float a_dps2    = readPotMapped(POT_ACCEL_PIN, ACCEL_MIN_DPS2, ACCEL_MAX_DPS2);

  // 3) Compute remaining distance to target
  float dist = targetDeg - currentCmdDeg;  // signed distance (deg)
  float dabs = absf(dist);                 // absolute distance (deg)

  // 4) If very close, snap and stop (prevents endless tiny dithering)
  if (dabs <= EPS_DEG) {
    currentCmdDeg = targetDeg;
    v_dps = 0.0f;
  } else {

    // 5) Determine motion direction (+1 or -1)
    float dir = (dist > 0.0f) ? 1.0f : -1.0f;

    // 6) Ramp up velocity by acceleration
    //    v = v + a * dt
    v_dps += a_dps2 * dt;

    // 7) Apply max speed limit from pot
    if (v_dps > vMax_dps) v_dps = vMax_dps;

    // 8) Deceleration constraint using stopping-distance rule
    //
    //    To be able to stop within remaining distance dabs:
    //       d_stop = v^2 / (2a)  <= dabs
    //    Rearranged to max allowed velocity for that distance:
    //       v_stop = sqrt(2*a*dabs)
    //
    //    If our current v exceeds v_stop, clamp it down (meaning: start braking).
    //
    float v_stop = sqrtf(2.0f * a_dps2 * dabs);
    if (v_dps > v_stop) v_dps = v_stop;

    // 9) Convert velocity into a position step for this tick
    float delta = v_dps * dt;         // deg to advance this tick (always positive)

    // 10) Never step past the target (cap delta to remaining distance)
    if (delta > dabs) delta = dabs;

    // 11) Apply the step in the correct direction
    currentCmdDeg += dir * delta;
  }

  // 12) Send command to servo
  int pwm_us = pwmUsFromDeg(currentCmdDeg);
  s.writeMicroseconds(pwm_us);

  // 13) Print for observation (USB Serial)
  Serial.print(targetDeg, 1); Serial.print(" | ");
  Serial.print(currentCmdDeg, 1); Serial.print(" | ");
  Serial.print(v_dps, 1); Serial.print(" | ");
  Serial.print(a_dps2, 0); Serial.print(" | ");
  Serial.print(dabs, 1); Serial.print(" | ");

  // DeltaDeg printed as the last applied step size (approx):
  // We can recompute it cheaply for display by comparing to target:
  // (Not exact when snapped, but good enough for debugging)
  // Instead, let's print the theoretical step v*dt (bounded by distance):
  float deltaPrint = clampf(v_dps * dt, 0.0f, dabs);
  Serial.print(deltaPrint, 3); Serial.print(" | ");

  Serial.println(pwm_us);

  delay(LOOP_INTERVAL_MS);
}
