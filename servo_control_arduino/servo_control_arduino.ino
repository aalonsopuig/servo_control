/*
===============================================================================
Title:        servo_control
Date:         2026-03-01
Author:       Alejandro Alonso Puig + ChatGPT
License:      Apache 2.0 License
-------------------------------------------------------------------------------
Description:

Generic standalone servo controller driven by 3 potentiometers:

  - A0: Target position (0..180 deg)
  - A1: Vmax%  (1..100 %)  -> speed limit as % of SERVO_MAX_SPEED_DEGPS
  - A2: Amax%  (1..100 %)  -> acceleration limit as % of ACCEL_MAX_DEGPS2

It implements a trapezoidal motion profile (ramp up / cruise / ramp down)
by updating a "commanded reference" in small steps at a fixed rate.

Key ideas:

- The servo receives a PWM command corresponding to the commanded reference angle.
- The commanded reference does NOT jump instantly to the final target:
  it evolves according to a speed limit and an acceleration limit.
- The speed ramps up with acceleration, but is also constrained so the reference
  can still stop in time (classic stopping-distance rule).

-------------------------------------------------------------------------------
SERVO SPEED PARAMETER (IMPORTANT)

This sketch uses one physical characterization parameter:

    SERVO_MAX_SPEED_DEGPS   [deg/s]

It represents the *maximum real angular speed* the servo can achieve under the
current supply voltage and load conditions.

Example conversions from datasheet-style specs:

  HS-805BB @6V
  0.14 s / 60°  ->  60 / 0.14  = 428.6 deg/s

  Futaba S3003 @4.8V
  0.23 s / 60°  ->  60 / 0.23  = 261.0 deg/s  (DEFAULT USED HERE)

If you change the servo model, the supply voltage, or mechanical conditions
that affect speed, you MUST update SERVO_MAX_SPEED_DEGPS accordingly.

-------------------------------------------------------------------------------
USB SERIAL OUTPUT (printed each loop)

Target | Cmd | V | V% | A% | Dist | dDeg | PWM

- Target : target angle from the target potentiometer (deg)
- Cmd    : commanded reference angle (deg) after motion profile update
- V      : instantaneous profile velocity magnitude used for stepping (deg/s)
- V%     : user knob for vmax limit (1..100%) of SERVO_MAX_SPEED_DEGPS
- A%     : user knob for accel limit (1..100%) of ACCEL_MAX_DEGPS2
- Dist   : remaining distance to target (deg), absolute value
- dDeg   : step applied this tick (deg)
- PWM    : microseconds sent to the servo (us)

===============================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// ============================ USER CONFIGURATION ============================

// ---- Pins ----
// SERVO_PIN      : PWM output to servo signal wire
// POT_TARGET_PIN : user knob for target angle (0..180°)
// POT_VMAX_PIN   : user knob for vmax% (1..100%)
// POT_ACCEL_PIN  : user knob for accel% (1..100%)
#define SERVO_PIN        9
#define POT_TARGET_PIN   A0
#define POT_VMAX_PIN     A1
#define POT_ACCEL_PIN    A2

#define BAUDRATE         115200

// ---- Control loop timing ----
// 20ms is a typical servo refresh frame; 40ms is fine too, but slower.
// Keep it stable: the motion profile assumes a constant dt.
#define LOOP_INTERVAL_MS 40

// ---- Servo PWM calibration ----
// These define how the 0..180° command maps to microseconds.
// Adjust to match YOUR servo endpoints safely.
#define SERVO_CENTER_US  1150
#define SERVO_HALFSPAN_US  650

#define PWM_MIN_US       (SERVO_CENTER_US - SERVO_HALFSPAN_US)  // 0 deg
#define PWM_MAX_US       (SERVO_CENTER_US + SERVO_HALFSPAN_US)  // 180 deg

// ---- Physical servo characterization ----
// DEFAULT: Futaba S3003 @4.8V
// 0.23 s / 60° -> 60 / 0.23 = 261 deg/s
//
// If you use another servo or change supply voltage, update this value.
#define SERVO_MAX_SPEED_DEGPS 261.0f

// ---- Acceleration reference (100% = this value) ----
// This is a chosen design limit for acceleration used by the software profile.
#define ACCEL_MAX_DEGPS2  800.0f

// ---- Percent mapping ----
#define PCT_MIN  1
#define PCT_MAX  100

// ---- ADC ----
#define ADC_SCALE  1023.0f

// ---- Small threshold to consider we reached the target (deg) ----
#define EPS_DEG    0.25f

// ============================ GLOBAL STATE ==================================

Servo s;

// "Commanded reference" angle that we evolve smoothly (deg).
// This is what we convert to PWM and send to the servo.
static float cmdDeg = 0.0f;

// Profile velocity magnitude (deg/s) used to step cmdDeg.
// We keep it as a magnitude (>=0) and apply direction separately.
static float vDegps = 0.0f;

// ============================ HELPERS =======================================

// Clamp float to [lo..hi].
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Absolute value for float.
static inline float absf(float x) { return (x >= 0.0f) ? x : -x; }

// Map ADC 0..1023 to 0..1.
static inline float map01(int adc) {
  return (float)adc / ADC_SCALE;
}

// Convert 0..180 degrees into PWM microseconds using calibrated endpoints.
static inline int pwmUsFromDeg(float deg) {
  deg = clampf(deg, 0.0f, 180.0f);
  float us = PWM_MIN_US + (deg / 180.0f) * (PWM_MAX_US - PWM_MIN_US);
  return (int)(us + 0.5f);
}

// Read target pot and map to 0..180 degrees.
float readTargetDeg() {
  int adc = analogRead(POT_TARGET_PIN);
  float deg = map01(adc) * 180.0f;
  return clampf(deg, 0.0f, 180.0f);
}

// Map ADC (0..1023) to integer percent (1..100), using full knob travel.
int percentFromAdc(int adc) {
  float t = clampf(map01(adc), 0.0f, 1.0f);                         // 0..1
  int pct = (int)(PCT_MIN + t * (PCT_MAX - PCT_MIN) + 0.5f);        // rounded
  if (pct < PCT_MIN) pct = PCT_MIN;
  if (pct > PCT_MAX) pct = PCT_MAX;
  return pct;
}

// Convert Vmax% to physical speed limit (deg/s) relative to REAL servo max.
// 100% -> SERVO_MAX_SPEED_DEGPS
//  1%  -> 0.01 * SERVO_MAX_SPEED_DEGPS
float vmaxDegpsFromPercent(int vmaxPct) {
  float t = (float)vmaxPct / 100.0f;
  float v = SERVO_MAX_SPEED_DEGPS * t;
  return clampf(v, SERVO_MAX_SPEED_DEGPS * 0.01f, SERVO_MAX_SPEED_DEGPS);
}

// Convert Amax% to acceleration limit (deg/s²) relative to ACCEL_MAX_DEGPS2.
float accelDegps2FromPercent(int accelPct) {
  float t = (float)accelPct / 100.0f;
  float a = ACCEL_MAX_DEGPS2 * t;
  return max(a, ACCEL_MAX_DEGPS2 * 0.01f);
}

// Fixed-width row printing so columns do not “dance” in Serial Monitor.
void printRowFixed(float targetDeg,
                   float cmdDeg,
                   float vDegps,
                   int vmaxPct,
                   int accelPct,
                   float distDeg,
                   float deltaDeg,
                   int pwmUs)
{
  char a[12], b[12], c[12], d[12], e[12];

  // Target, Cmd, V, Dist -> width 6, 1 decimal
  dtostrf(targetDeg, 6, 1, a);
  dtostrf(cmdDeg,    6, 1, b);
  dtostrf(vDegps,    6, 1, c);
  dtostrf(distDeg,   6, 1, d);

  // dDeg -> width 7, 3 decimals (a bit more precision for small steps)
  dtostrf(deltaDeg,  7, 3, e);

  // Target | Cmd | V | V% | A% | Dist | dDeg | PWM
  Serial.print(a); Serial.print(" | ");
  Serial.print(b); Serial.print(" | ");
  Serial.print(c); Serial.print(" | ");

  if (vmaxPct < 100) Serial.print(' ');
  if (vmaxPct < 10)  Serial.print(' ');
  Serial.print(vmaxPct); Serial.print("% | ");

  if (accelPct < 100) Serial.print(' ');
  if (accelPct < 10)  Serial.print(' ');
  Serial.print(accelPct); Serial.print("% | ");

  Serial.print(d); Serial.print(" | ");
  Serial.print(e); Serial.print(" | ");

  if (pwmUs < 1000) Serial.print(' ');
  Serial.println(pwmUs);
}

// ============================ SETUP =========================================

void setup() {

  Serial.begin(BAUDRATE);
  delay(700);

  // Attach servo with calibrated min/max pulse limits.
  // This makes writeMicroseconds safer by constraining output range.
  s.attach(SERVO_PIN, PWM_MIN_US, PWM_MAX_US);

  // Startup behaviour:
  // Read the target pot and immediately command that position,
  // so the servo does not jump to some arbitrary default (like 90°).
  float targetDeg = readTargetDeg();
  cmdDeg = targetDeg;
  vDegps = 0.0f;

  s.writeMicroseconds(pwmUsFromDeg(cmdDeg));

  Serial.println("servo_control (Target + V% + A%)");
  Serial.println("Target |   Cmd |     V |  V% |  A% |  Dist |   dDeg |  PWM");
  Serial.println("----------------------------------------------------------------");
}

// ============================ LOOP ==========================================

void loop() {

  // dt is constant because we run with a fixed delay at the end of loop.
  const float dt = (float)LOOP_INTERVAL_MS / 1000.0f;

  // ---- Read user knobs ----
  float targetDeg = readTargetDeg();                 // 0..180 deg

  int adcVmax   = analogRead(POT_VMAX_PIN);          // 0..1023
  int adcAccel  = analogRead(POT_ACCEL_PIN);         // 0..1023

  int vmaxPct   = percentFromAdc(adcVmax);           // 1..100 %
  int accelPct  = percentFromAdc(adcAccel);          // 1..100 %

  float vmaxDegps   = vmaxDegpsFromPercent(vmaxPct); // deg/s
  float accelDegps2 = accelDegps2FromPercent(accelPct); // deg/s²

  // ---- Distance to target ----
  float dist = targetDeg - cmdDeg;     // signed distance (deg)
  float dabs = absf(dist);             // absolute distance (deg)

  float deltaApplied = 0.0f;           // will print actual step used this tick

  // ---- If close enough, snap to target and stop ----
  if (dabs <= EPS_DEG) {
    cmdDeg = targetDeg;
    vDegps = 0.0f;
    deltaApplied = 0.0f;
  } else {

    // Direction of motion (+1 or -1) based on sign of distance.
    float dir = (dist > 0.0f) ? 1.0f : -1.0f;

    // Compute the maximum velocity that still allows stopping within dabs.
    // v_stop = sqrt(2 * a * d)
    float vStop = sqrtf(2.0f * accelDegps2 * dabs);

    // Allowed speed is limited by:
    // - user-selected vmaxDegps
    // - stopping constraint vStop (shrinks as we approach the target)
    float vAllowed = min(vmaxDegps, vStop);

    // Ramp velocity magnitude upward by accel*dt (simple acceleration limiter).
    // Note: deceleration is automatically produced because vAllowed decreases
    // near target, forcing vDegps down to satisfy the stop constraint.
    vDegps += accelDegps2 * dt;

    // Cap to allowed speed.
    if (vDegps > vAllowed) vDegps = vAllowed;

    // Convert velocity to step for this tick.
    float delta = vDegps * dt;         // always positive

    // Never overshoot target.
    if (delta > dabs) delta = dabs;

    // Apply step.
    cmdDeg += dir * delta;

    deltaApplied = delta;
  }

  // ---- Send command to servo ----
  int pwmUs = pwmUsFromDeg(cmdDeg);
  s.writeMicroseconds(pwmUs);

  // ---- Telemetry (aligned columns) ----
  printRowFixed(targetDeg, cmdDeg, vDegps, vmaxPct, accelPct, dabs, deltaApplied, pwmUs);

  // Fixed loop period.
  delay(LOOP_INTERVAL_MS);
}
