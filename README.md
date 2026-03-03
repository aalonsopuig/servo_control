# servo_control

This repository contains `servo_control`, an Arduino sketch for generic open-loop servo control using a trapezoidal motion profile and user-adjustable speed and acceleration limits expressed in percent.

The program does **not** read the servo’s internal feedback potentiometer. It operates purely in open loop, generating PWM position commands while logging internal motion variables to the PC over USB.

The sketch is intended as a generic and reusable motion-shaping controller for standard hobby servos.

---

## Objective

Instead of sending a single large step directly to the final target angle, the sketch progressively moves an internal commanded reference toward the target.

The motion is governed by a trapezoidal profile defined by:

- Target angle (degrees)
- Maximum velocity limit **V% (1–100%)**
- Maximum acceleration limit **A% (1–100%)**

Where:

- `V%` is a percentage of the physical maximum servo speed (`SERVO_MAX_SPEED_DEGPS`)
- `A%` is a percentage of a configurable acceleration reference (`ACCEL_MAX_DEGPS2`)

The commanded reference is incrementally updated at a fixed rate. This produces smooth acceleration and deceleration of the commanded signal sent to the servo.

---

## Servo speed parameter (SERVO_MAX_SPEED_DEGPS)

The sketch uses one physical characterization parameter: SERVO_MAX_SPEED_DEGPS (deg/s)


It represents the maximum real angular speed of the servo under the current supply voltage and operating conditions.

Example conversions from datasheet-style specifications:

- HS-805BB @6V  
  `0.14 s / 60°  →  60 / 0.14  = 428.6 deg/s`

- Futaba S3003 @4.8V  
  `0.23 s / 60°  →  60 / 0.23  = 261.0 deg/s`  
  (Default value used in the sketch)

If the servo model, supply voltage, or mechanical conditions change, this parameter must be updated.

The Vmax potentiometer always spans 1–100% of this value, ensuring the full knob travel remains meaningful for any servo.

---

## Hardware setup

An Arduino Uno (or compatible 5 V board), one standard RC servo, and three potentiometers configured as voltage dividers are required.

Connections:

- Servo signal → `D9` (configurable as `SERVO_PIN`)
- Servo power → external supply recommended
- Servo ground → supply ground
- Arduino ground → must be connected to the same ground as the servo supply

Potentiometers:

- Target position → wiper to `A0`
- Vmax% → wiper to `A1`
- Amax% → wiper to `A2`

All potentiometers should be wired as standard 0–5 V dividers (ends to 5 V and GND).

High-current or high-torque servos should not be powered from the Arduino 5 V rail.

---

## Control principle

At a fixed interval (`LOOP_INTERVAL_MS`), the sketch:

1. Reads the target angle from `A0` and maps it to 0–180°.
2. Reads Vmax% (`A1`) and Amax% (`A2`) and maps them to integer percentages (1–100%).
3. Converts percentages to physical limits:
   - `vmaxDegps = SERVO_MAX_SPEED_DEGPS × (V% / 100)`
   - `accelDegps2 = ACCEL_MAX_DEGPS2 × (A% / 100)`
4. Computes the remaining distance to the target.
5. Updates the commanded reference (`cmdDeg`) using a trapezoidal motion profile.
6. Converts the commanded angle to calibrated PWM microseconds.
7. Sends the PWM signal to the servo.

The trapezoidal profile uses:

- Acceleration ramp: `v = v + a·dt`
- Velocity limiting by `vmaxDegps`
- Stopping constraint: `v_stop = sqrt(2·a·distance)`
- Position update: `delta = v·dt`

The stopping constraint guarantees that the commanded reference can always decelerate and stop exactly at the target.

---

## Startup behavior

At power-up, the sketch reads the target potentiometer and immediately sets the commanded reference to that angle.

This prevents an initial jump to an arbitrary default position.

---

## Servo calibration

PWM calibration is defined by:

- `SERVO_CENTER_US`
- `SERVO_HALFSPAN_US`

These define:

- `PWM_MIN_US`
- `PWM_MAX_US`

They must be adjusted for the specific servo used to avoid mechanical overtravel or hard-stop impacts.

---

## Serial output

At each control update (115200 baud over USB), the sketch prints aligned telemetry columns: Target | Cmd | V | V% | A% | Dist | dDeg | PWM


Meaning:

- `Target` : target angle from potentiometer (deg)
- `Cmd`    : commanded reference angle after profile update (deg)
- `V`      : instantaneous profile velocity magnitude (deg/s)
- `V%`     : speed limit knob (1–100%) of `SERVO_MAX_SPEED_DEGPS`
- `A%`     : acceleration limit knob (1–100%) of `ACCEL_MAX_DEGPS2`
- `Dist`   : remaining distance to target (deg)
- `dDeg`   : step applied during this update (deg)
- `PWM`    : PWM pulse width sent to the servo (µs)

The fixed-width formatting keeps columns aligned for easier logging and analysis.

---

## Motion shaping and perceived speed

The position increment per update is: delta = v · dt


If the update interval decreases while velocity in deg/s remains constant, each step becomes smaller but occurs more frequently. The average commanded angular velocity remains approximately constant in an ideal actuator.

In real RC servos, the internal controller responds to position error between the commanded pulse width and the internal potentiometer. Smaller incremental updates tend to reduce instantaneous error, which typically results in smoother motion compared to a single large command step.

By shaping the commanded reference with acceleration and velocity limits, the sketch provides practical speed control behavior without modifying the servo electronics.


