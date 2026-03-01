# servo_control

This repository contains `servo_control`, an Arduino sketch designed to experimentally study servo motion shaping using incremental position commands and a configurable acceleration profile.

The program does not read the servo’s internal feedback potentiometer. It operates in open loop and generates PWM position commands while logging internal motion variables to the PC over USB. The objective is to observe how gradual reference updates influence perceived speed, smoothness and mechanical stress.

## Objective

Instead of sending a single large step to the final target angle, the sketch progressively moves an internal command variable toward the target. The motion is governed by a trapezoidal profile defined by a target `angle (degrees)`, a maximum velocity `v_max (deg/s)`, and an acceleration `a (deg/s²)`.

This approach allows empirical exploration of how velocity limiting and acceleration shaping affect servo behavior.

## Hardware setup

An Arduino Uno (or compatible 5 V board), one standard RC servo, and three potentiometers configured as voltage dividers are required.

Connections:

- Servo signal → D9 (configurable in the sketch as `SERVO_PIN`)  
- Servo power → external 6 V supply recommended for high-torque servos  
- Servo ground → supply ground  
- Arduino ground → must be connected to the same ground as the servo supply  

Potentiometers:

- Target position pot: ends to 5 V and GND, wiper to `A0`  
- Maximum velocity pot: ends to 5 V and GND, wiper to `A1`  
- Acceleration pot: ends to 5 V and GND, wiper to `A2`  

High-torque servos should not be powered from the Arduino 5 V rail.

## Control principle

At a fixed interval (`LOOP_INTERVAL_MS`), the sketch reads the target angle from `A0` and maps it to 0–180°. It reads `v_max` and acceleration from `A1` and `A2`. It then updates an internal commanded position (`currentCmdDeg`) using a trapezoidal motion profile and converts the commanded angle to PWM microseconds before sending it to the servo.

The trapezoidal profile works as follows. Velocity ramps up using `v = v + a·dt`. Velocity is limited by the maximum velocity set by the user. Velocity is further limited using the stopping-distance rule `v_stop = sqrt(2·a·distance)`, ensuring the motion can decelerate and stop exactly at the target. The commanded position is advanced by `delta = v·dt`.

This produces smooth acceleration and deceleration in the commanded reference.

## Servo calibration

For the HS-805BB servo used in testing, the real mechanical endpoints were measured experimentally as approximately:

- 0° → 1500 − 860 µs  
- 180° → 1500 + 860 µs  

These values are implemented in the sketch and should be adjusted if a different servo is used.

## Serial output

At each control update, the sketch prints one line to the USB serial port (115200 baud) including:

- Target angle (deg)  
- Current commanded angle (deg)  
- Current virtual velocity (deg/s)  
- Acceleration (deg/s²)  
- Remaining distance to target (deg)  
- Applied delta step (deg)  
- PWM command (µs)  

This allows correlation between motion parameters and observed mechanical behavior.

## Why reducing the update interval affects perceived speed

Two mechanisms are involved.

From the command generator perspective, the position increment per update is `delta = v·dt`. If the update interval is reduced while keeping velocity in deg/s constant, each individual step becomes smaller but occurs more frequently. In an ideal actuator, the average commanded angular velocity remains approximately constant.

However, real RC servos contain an internal closed-loop controller that behaves approximately like a proportional or PD system. The motor drive effort is proportional to the position error between the commanded pulse width and the internal feedback potentiometer.

When the commanded position is advanced in small increments, the instantaneous position error remains small. The internal controller therefore generates lower drive effort, the motor does not saturate at maximum speed, and the motion becomes smoother and slower on average.

In contrast, a single large jump in commanded angle creates a large internal error, driving the motor near maximum effort until the error decreases.

Incremental command updates therefore exploit the intrinsic closed-loop behavior of the servo. By limiting the error presented to the internal controller, the sketch indirectly limits motor speed without modifying the servo electronics. This is the fundamental reason why discrete interpolation and ramp shaping provide usable speed control in standard hobby servos.
