# 3wheel_KiwiDrive

## 3-Wheel Omni / Kiwi-Drive Indoor Robot Controller

A real-time embedded control system for a 3-wheel omniwheel robot built around an **STM32F103C8T6** and an **ESP8266** wireless bridge.

The project replaces the original 4-wheel skid-steer architecture with a holonomic 3-wheel Kiwi drive. The STM32 is responsible for real-time motor control, encoder processing, IMU heading estimation, odometry, waypoint navigation, calibration, safety supervision, telemetry, and survey/waypoint state. The ESP8266 provides the Wi-Fi access point, browser-based control UI, WebSocket transport, and serial bridge between the operator UI and the STM32.

The current firmware is designed around a **10 ms control cycle (100 Hz)**.

> **Project status:** active development / real-robot firmware. Several geometry and calibration constants are explicitly provisional and must be verified against the physical chassis before the system is treated as measurement-grade.

---

## Table of Contents

- [1. Project Overview](#1-project-overview)
- [2. System Architecture](#2-system-architecture)
- [3. Hardware](#3-hardware)
- [4. STM32 Pin Mapping](#4-stm32-pin-mapping)
- [5. Software Stack](#5-software-stack)
- [6. Repository Structure](#6-repository-structure)
- [7. Runtime Execution Model](#7-runtime-execution-model)
- [8. Motion Command Pipeline](#8-motion-command-pipeline)
- [9. Motor Control](#9-motor-control)
- [10. Encoder Processing](#10-encoder-processing)
- [11. Kiwi Kinematics](#11-kiwi-kinematics)
- [12. Odometry](#12-odometry)
- [13. IMU / Heading Estimation](#13-imu--heading-estimation)
- [14. Free Drive](#14-free-drive)
- [15. Survey Mode](#15-survey-mode)
- [16. Waypoint Patrol](#16-waypoint-patrol)
- [17. Field Calibration](#17-field-calibration)
- [18. Communication Protocol](#18-communication-protocol)
- [19. Telemetry and Diagnostics](#19-telemetry-and-diagnostics)
- [20. Safety and Fault Handling](#20-safety-and-fault-handling)
- [21. Non-Volatile Calibration Storage](#21-non-volatile-calibration-storage)
- [22. Timing and Real-Time Considerations](#22-timing-and-real-time-considerations)
- [23. Configuration and Important Constants](#23-configuration-and-important-constants)
- [24. Build and Development](#24-build-and-development)
- [25. Bring-Up / Commissioning Procedure](#25-bring-up--commissioning-procedure)
- [26. Known Limitations and Important Implementation Notes](#26-known-limitations-and-important-implementation-notes)
- [27. Recommended Development Order](#27-recommended-development-order)
- [28. Future Direction](#28-future-direction)
- [29. Development Principles](#29-development-principles)
- [30. Author / Repository](#30-author--repository)

---

# 1. Project Overview

The robot is a **three-wheel holonomic platform**. The three omniwheels are intended to be distributed around the chassis at approximately 120° intervals.

The current system provides five operator-facing modes:

1. **Free Drive**
2. **Survey**
3. **Waypoint Patrol**
4. **Calibration**
5. **Log History**

The important architectural decision is that the STM32 no longer thinks in terms of left and right drivetrain sides. A motion command is expressed as a **body-frame velocity vector**:

- `Vx` — forward velocity
- `Vy` — leftward velocity
- `ω` — counter-clockwise angular velocity

The Kiwi kinematic model converts that single chassis command into three wheel RPM targets.

Likewise, wheel encoder displacement is converted back to body-frame motion through the same kinematic model.

This gives the high-level control software a clean separation:

```text
Operator / Navigation
        |
        | body velocity
        v
Kiwi Inverse Kinematics
        |
        | wheel RPM targets
        v
3 × MotorController
        |
        v
Motor Driver / Motors
        |
        | encoder feedback
        v
3 × Encoders
        |
        v
Odometry / Diagnostics
```

The current pose estimator is **not SLAM**. It is primarily wheel-encoder dead reckoning for translation plus a gyro-primary heading estimate.

---

# 2. System Architecture

## 2.1 High-level architecture

```text
                         ┌───────────────────────────────┐
                         │          Browser UI            │
                         │ Free / Survey / Patrol / Calib│
                         │           / Logs              │
                         └───────────────┬───────────────┘
                                         │
                                  WebSocket binary
                                         │
                                         v
                         ┌───────────────────────────────┐
                         │            ESP8266             │
                         │   Wi-Fi AP + WebSocket bridge │
                         │   HTTP server + serial bridge │
                         └───────────────┬───────────────┘
                                         │
                                  UART 115200
                                         │
                                         v
┌─────────────────────────────────────────────────────────────────────┐
│                           STM32F103C8T6                              │
│                                                                     │
│  comm.c  → command parser/callback                                  │
│                    │                                                │
│                    v                                                │
│                 main.c                                              │
│                    │                                                │
│        ┌───────────┼───────────┬─────────────┐                      │
│        │           │           │             │                      │
│        v           v           v             v                      │
│   Navigation   Calibration  Manual      Safety/Faults              │
│        │           │           │                                      │
│        └───────────┴─────┬─────┘                                      │
│                          v                                            │
│                KiwiKinematics                                         │
│                          │                                            │
│                    3 wheel RPM                                       │
│                          │                                            │
│              ┌───────────┼───────────┐                               │
│              v           v           v                               │
│           PID #1      PID #2      PID #3                             │
│              │           │           │                               │
│           Motor #1    Motor #2    Motor #3                            │
│              │           │           │                               │
│           Encoder #1  Encoder #2  Encoder #3                          │
│              └───────────┼───────────┘                               │
│                          v                                            │
│                       Odometry                                        │
│                          ^                                            │
│                          │                                            │
│                       MPU6050                                         │
│                                                                     │
│                    Survey / Telemetry                                 │
└─────────────────────────────────────────────────────────────────────┘
```

## 2.2 Responsibilities

### STM32

The STM32 owns all robot state that matters for safe and deterministic motion:

- motor actuation
- wheel-speed PID
- encoder position
- encoder speed
- IMU gyro reading
- gyro calibration
- chassis kinematics
- odometry pose
- survey vertices
- waypoint list
- waypoint patrol state machine
- field calibration
- communication framing and CRC
- connection timeout / communication health
- IMU fault handling
- encoder fault supervision
- calibration persistence in internal flash
- periodic telemetry and diagnostics

### ESP8266

The ESP8266 is intentionally kept out of the control loop.

It provides:

- Wi-Fi SoftAP
- browser HTTP server
- WebSocket server
- browser-to-STM32 command translation
- STM32-to-browser telemetry forwarding
- browser-side map rendering
- browser-side survey area calculation
- browser-side log history
- operator controls

The ESP8266 does **not** own the authoritative robot pose or autonomous navigation state.

---

# 3. Hardware

## 3.1 Main controller

- MCU: **STM32F103C8T6**
- Architecture: ARM Cortex-M3
- System clock: **72 MHz**
- Development environment: STM32CubeIDE
- HAL: STM32Cube FW_F1 V1.8.7
- Debug/programming: SWD

## 3.2 Wireless / operator bridge

- MCU: **ESP8266**
- Firmware: Arduino-style `.ino`
- Wi-Fi mode: SoftAP
- SSID currently: `RobotCar`
- WebSocket server port: `81`
- HTTP server port: `80`
- STM32 serial baud: **115200**

## 3.3 Drive system

- 3 × geared DC motors with Hall encoders
- 3 × omniwheels
- 2 × L298N motor-driver modules
- 3 controlled motor channels in the firmware
- one motor controller object per physical wheel

The exact motor/encoder CPR/PPR and mechanical dimensions must be verified against the actual hardware before precision operation. The firmware currently contains an explicit `ENCODER_PPR` calibration constant.

## 3.4 IMU

- MPU6050
- Gyro Z axis is used for heading integration
- I2C2
- configured at 400 kHz
- gyro full-scale configuration currently corresponds to ±1000 dps
- gyro sample rate configuration targets 100 Hz

## 3.5 Power domains

The project is currently designed around:

- motor supply: nominal 12 V
- encoder supply: 5 V
- STM32 / ESP8266 / MPU6050 logic: 3.3 V

The repository does not contain a battery-management implementation. Power protection and battery integration remain hardware-level design tasks.

---

# 4. STM32 Pin Mapping

The current CubeMX configuration (`rc_kiwi_drive.ioc`) maps the hardware as follows.

| Function | STM32 pin | Peripheral |
|---|---|---|
| Encoder 1 A | PA0 | TIM2 CH1 |
| Encoder 1 B | PA1 | TIM2 CH2 |
| Encoder 2 A | PA6 | TIM3 CH1 |
| Encoder 2 B | PA7 | TIM3 CH2 |
| Encoder 3 A | PB7 | TIM4 CH2 |
| Encoder 3 B | PB6 | TIM4 CH1 |
| Motor 1 PWM | PA8 | TIM1 CH1 |
| Motor 2 PWM | PA9 | TIM1 CH2 |
| Motor 3 PWM | PA10 | TIM1 CH3 |
| Motor 1 IN1 | PB12 | GPIO |
| Motor 1 IN2 | PB13 | GPIO |
| Motor 2 IN3 | PB14 | GPIO |
| Motor 2 IN4 | PB15 | GPIO |
| Motor 3 IN1 | PA11 | GPIO |
| Motor 3 IN2 | PA12 | GPIO |
| MPU6050 SCL | PB10 | I2C2 SCL |
| MPU6050 SDA | PB11 | I2C2 SDA |
| ESP UART TX | PA2 | USART2 TX |
| ESP UART RX | PA3 | USART2 RX |
| SWDIO | PA13 | SWD |
| SWCLK | PA14 | SWD |
| Internal LED | PC13 | GPIO |
| HSE input | PD0 | HSE |
| HSE output | PD1 | HSE |

### Important timer usage

- **TIM1** = 3-channel PWM
- **TIM2** = encoder 1
- **TIM3** = encoder 2
- **TIM4** = encoder 3

USART1 is not used for debugging because the previous four-wheel architecture had a pin conflict involving PA9. The current design uses USART2 for ESP8266 communication.

### Current TIM1 configuration

The current CubeMX configuration uses:

```text
Prescaler = 7
Period    = 899
Mode      = Center-aligned 1
```

The current PWM helper uses an effective maximum compare value corresponding to the configured timer period.

---

# 5. Software Stack

## STM32 firmware

- C
- STM32 HAL
- STM32CubeIDE
- STM32CubeMX
- GCC ARM toolchain
- STM32Cube FW_F1 V1.8.7

## ESP8266 firmware

- C/C++
- Arduino ESP8266 framework
- `ESP8266WiFi`
- `ESP8266WebServer`
- `WebSocketsServer`

## Browser UI

- HTML
- CSS
- vanilla JavaScript
- WebSocket binary messages
- HTML5 Canvas for maps
- `DataView` / typed binary parsing

No frontend framework is currently used.

---

# 6. Repository Structure

Current project organization is approximately:

```text
3wheel_KiwiDrive/
├── Code/
│   └── rc_kiwi_drive/
│       ├── Core/
│       │   ├── Inc/
│       │   │   ├── bsp_pwm.h
│       │   │   ├── calib.h
│       │   │   ├── comm.h
│       │   │   ├── encoder.h
│       │   │   ├── imu.h
│       │   │   ├── kiwi_kinematics.h
│       │   │   ├── main.h
│       │   │   ├── motor.h
│       │   │   ├── motor_controller.h
│       │   │   ├── nvs.h
│       │   │   ├── odometry.h
│       │   │   ├── pid.h
│       │   │   ├── stm32f1xx_hal_conf.h
│       │   │   ├── stm32f1xx_it.h
│       │   │   ├── survey.h
│       │   │   └── waypoint_nav.h
│       │   │
│       │   └── Src/
│       │       ├── bsp_pwm.c
│       │       ├── calib.c
│       │       ├── comm.c
│       │       ├── encoder.c
│       │       ├── imu.c
│       │       ├── kiwi_kinematics.c
│       │       ├── main.c
│       │       ├── motor.c
│       │       ├── motor_controller.c
│       │       ├── nvs.c
│       │       ├── odometry.c
│       │       ├── pid.c
│       │       ├── stm32f1xx_hal_msp.c
│       │       ├── stm32f1xx_it.c
│       │       ├── stm32f1xx_syscalls.c
│       │       ├── stm32f1xx_sysmem.c
│       │       ├── system_stm32f1xx.c
│       │       ├── survey.c
│       │       └── waypoint_nav.c
│       │
│       ├── STM32F103C8TX_FLASH.ld
│       ├── rc_kiwi_drive Debug.launch
│       ├── rc_kiwi_drive.ioc
│       ├── .cproject
│       ├── .mxproject
│       └── .project
│
└── Esp_Code/
    └── esp_v2/
        └── esp_v2.ino
```

The exact repository may also contain generated IDE metadata and other project files. Generated build outputs should not be treated as firmware source.

---

# 7. Runtime Execution Model

## 7.1 Startup sequence

The STM32 startup sequence in `main()` is:

```text
HAL_Init()
    ↓
SystemClock_Config()
    ↓
MX_GPIO_Init()
MX_TIM2_Init()
MX_TIM3_Init()
MX_TIM4_Init()
MX_TIM1_Init()
MX_USART2_UART_Init()
MX_I2C2_Init()
    ↓
Encoder_Init(enc1, TIM2)
Encoder_Init(enc2, TIM3)
Encoder_Init(enc3, TIM4)
    ↓
set encoder direction signs
    ↓
MotorController_Init(ctrl1...)
MotorController_Init(ctrl2...)
MotorController_Init(ctrl3...)
    ↓
IMU_Init()
    ↓
Comm_Init()
Comm_RegisterCommandCallback()
    ↓
initialize g_kin
initialize g_odom
Nav_Init()
Cal_Init()
    ↓
load NVS calibration if valid
    ↓
main loop
```

The kinematic geometry starts from defaults and may then be replaced by values loaded from internal flash.

---

## 7.2 Main loop

The main loop is intentionally cooperative rather than RTOS-based.

Conceptually:

```text
while (1)
{
    Comm_Process();
    Survey_ResyncTick();
    Nav_DebugTick();

    every 10 ms:
        calculate dt
        IMU_UpdateGyroZ()
        IMU_CalibrationTick()

        safety checks

        if Nav active:
            Nav_Update()
        else if Calibration active:
            Cal_Update()
        else:
            manual body velocity
            -> Kiwi IK
            -> 3 MotorController_SetSpeed()
            -> 3 MotorController_Update()

        Odometry_Update()

        encoder supervision
        telemetry log

    every 100 ms:
        status telemetry
}
```

The exact order matters. Odometry runs after the motion-control branch and therefore consumes the current encoder positions collected during the preceding motor-control work.

---

# 8. Motion Command Pipeline

The current system uses **body-frame velocity**, not left/right wheel speed.

## 8.1 Command representation

```text
Vx  = forward / backward
Vy  = left / right
ω   = counter-clockwise / clockwise
```

Units:

- `Vx`, `Vy`: m/s
- `ω`: rad/s

The convention is:

```text
+X = forward
+Y = left
+ω = CCW
```

## 8.2 Browser → ESP8266

The browser creates a binary WebSocket message:

```text
[0x01][f32 vx][f32 vy][f32 omega]
```

where the floating-point values are transmitted little-endian.

## 8.3 ESP8266 → STM32

The ESP8266 wraps the body command in the STM32 serial protocol:

```text
[0xAA][length][command][payload][CRC8]
```

For `CMD_SET_SPEED`:

```text
command = 0x01
payload = 12 bytes
payload = float vx + float vy + float omega
length  = 13
```

## 8.4 STM32 command path

```text
USART2 RX interrupt
        ↓
Comm_UART_IRQHandler()
        ↓
64-byte RX ring buffer
        ↓
Comm_Process()
        ↓
CRC validation
        ↓
OnCommandReceived()
        ↓
g_cmd_vx
g_cmd_vy
g_cmd_omega
```

The ISR only places bytes into the ring buffer. Command callbacks are processed later from the main loop.

---

# 9. Motor Control

## 9.1 Architecture

There is one motor controller per wheel:

```text
MotorController_t
├── Motor_t
├── Encoder_t
├── PID_t
├── target_rpm
├── current_rpm
├── output_duty
└── enabled
```

There is no longer a master/slave motor pair or left/right synchronization controller.

The three wheel commands are synchronized because all three targets originate from the same chassis-level kinematic command.

---

## 9.2 Control path

```text
target wheel RPM
       ↓
PID setpoint
       ↓
Encoder_Update()
       ↓
current wheel RPM
       ↓
PID_Compute()
       ↓
signed duty command
       ↓
direction + duty
       ↓
Motor_SetDutyDirect()
       ↓
PWM + H-bridge inputs
```

---

## 9.3 Motor direction

`motor.c` implements:

```text
MOTOR_FORWARD
MOTOR_BACKWARD
MOTOR_STOP
MOTOR_BRAKE
```

`MOTOR_STOP` means:

```text
IN1 = 0
IN2 = 0
PWM = 0
```

`MOTOR_BRAKE` means:

```text
IN1 = 1
IN2 = 1
PWM = 100%
```

However, `Nav_Update()` and calibration use `MotorController_Stop()` as their coast/stop mechanism. This is intentionally different from active motor braking.

---

# 10. Encoder Processing

Each encoder uses a hardware timer configured in quadrature encoder mode.

## 10.1 Position

The encoder module maintains:

```text
prevCount
totalCount
```

Each update reads the current timer count and calculates:

```text
delta = currentCount - prevCount
```

The signed delta is accumulated into `totalCount`.

The cast to `int16_t` provides wrap-aware handling for the 16-bit timer counter.

---

## 10.2 Speed

Raw RPM is calculated from:

```text
RPM = (delta / ENCODER_PPR) × (60 / dt)
```

and then filtered with:

```text
speed = alpha × raw + (1-alpha) × previous
```

Current firmware value:

```text
ENCODER_SPEED_ALPHA = 0.2
```

The raw encoder count is still the authoritative source for odometry position. Odometry does not integrate the filtered speed.

---

## 10.3 Direction sign

The three encoder objects contain a configurable:

```text
dir_sign
```

Current startup configuration sets all three to:

```text
-1
```

This must be verified on the physical robot.

The correct sign is:

> encoder count should increase in the direction the firmware defines as positive wheel rotation for that physical wheel.

---

## 10.4 PPR

The firmware currently defines:

```text
ENCODER_PPR = 1768.0
```

This number is critical.

It must match the actual number of counted encoder transitions corresponding to the convention used by the timer configuration and encoder module.

Do not change it casually. If the real encoder/gearbox/resolution differs, all downstream position and speed calculations become wrong.

---

# 11. Kiwi Kinematics

## 11.1 Purpose

`kiwi_kinematics.c` is the central mathematical layer connecting robot-level motion to individual wheels.

It exposes:

```c
KiwiKinematics_Init()
KiwiKinematics_InverseKinematics()
KiwiKinematics_ForwardVelocity()
KiwiKinematics_WheelDeltaToBodyDelta()
```

---

## 11.2 Geometry

The model stores:

```text
wheel_radius_m
robot_radius_m
mount_angle_rad[3]
```

`mount_angle_rad[i]` describes the angular position of wheel `i` around the body.

The intended physical arrangement is three wheels approximately 120° apart.

The angles are intentionally passed into the kinematics module instead of hardcoded to exactly 120°.

---

## 11.3 Inverse kinematics

The firmware uses the general form:

```text
w_i = (1/r) × M_i × [Vx, Vy, ω]
```

with the implementation currently using:

```text
w_i = (
    -sin(beta_i) * Vx
    -cos(beta_i) * Vy
    + R * ω
) / r
```

and then converting rad/s to RPM.

All three wheel targets are therefore computed from the same body command.

---

## 11.4 Forward kinematics

The inverse of the 3×3 kinematic matrix is precomputed during initialization.

This matrix is stored in:

```text
kin->fwd[3][3]
```

and is then used for:

```text
wheel velocities
        ↓
body velocity
```

and:

```text
wheel angular displacements
        ↓
body displacement
```

The same linear transform is intentionally reused for both cases.

---

## 11.5 Degenerate geometry

`KiwiKinematics_Init()` calculates a 3×3 determinant.

If the matrix is singular or nearly singular:

```text
abs(det) < 1e-9
```

initialization fails and `kin->valid` remains false.

In that state the kinematics functions return zero outputs instead of dividing by a near-zero determinant.

---

## 11.6 Critical geometry/sign note

Explain that as of commit 5b5b6e5, the software is mathematically self-consistent. Both the initialization matrix M and the inverse kinematics formula now use -cos(beta_i) * Vy. The overall handedness of the robot was empirically flipped in this commit to match physical tests.

The README therefore documents the **actual implementation**, but this is a known point that must be experimentally verified against the real wheel orientations and encoder signs.

This matters especially for:

- lateral motion
- diagonal motion
- waypoint navigation
- any trajectory containing a non-zero body-Y component

A sign mistake here can produce a perfectly self-consistent software system that nevertheless drives the real robot in a mirrored lateral direction.

---

# 12. Odometry

## 12.1 Design

Current odometry is:

```text
translation = wheel encoders + Kiwi forward kinematics
heading     = MPU6050 gyro
```

This is not SLAM and does not use an external absolute position sensor.

---

## 12.2 Per-tick processing

At each control tick:

```text
current encoder counts
        ↓
count delta for each wheel
        ↓
wheel angular displacement
        ↓
per-wheel calibration scale
        ↓
KiwiKinematics_WheelDeltaToBodyDelta()
        ↓
dx_body
dy_body
dtheta_encoder
```

The encoder-derived angular increment is used for diagnostics.

The gyro-derived angular increment remains primary for pose heading.

---

## 12.3 World-frame translation

If the current heading before the tick is:

```text
theta_before
```

the implementation computes:

```text
theta_mid = theta_before + dtheta_gyro / 2
```

and rotates the body displacement into the world frame:

```text
dx_world = dx_body*cos(theta_mid) - dy_body*sin(theta_mid)
dy_world = dx_body*sin(theta_mid) + dy_body*cos(theta_mid)
```

Then:

```text
x += dx_world
y += dy_world
theta = normalize(theta_before + dtheta_gyro)
```

This midpoint integration is important because the robot is holonomic and `dy_body` is a real physical degree of freedom.

---

## 12.4 Motion mode behavior

`ODOM_IDLE` is functionally different from active motion.

When mode is:

```text
ODOM_IDLE
```

the current encoder deltas are still tracked and diagnostics are still updated, but pose integration is skipped.

`ODOM_STRAIGHT` and `ODOM_ROTATING` are currently retained mainly for interface compatibility. The old skid-steer-specific difference between them has been removed.

---

## 12.5 Heading disagreement

Odometry computes:

```text
dtheta_encoder - dtheta_gyro
```

per tick.

It maintains two diagnostics:

```text
heading_disagreement_signed_deg
heading_disagreement_abs_integral_deg
```

These values are not fused into the pose.

They are intended for:

- slip detection
- gyro-vs-encoder diagnosis
- calibration validation
- patrol debugging
- field testing

---

# 13. IMU / Heading Estimation

## 13.1 MPU6050 configuration

The current firmware:

- accesses address `0x68`
- checks `WHO_AM_I == 0x68`
- wakes the sensor
- enables DLPF configuration
- configures gyro full-scale for ±1000 dps
- targets a 100 Hz sample rate
- reads `GYRO_ZOUT_H/L`

Only gyro-Z is used by the current heading estimator.

---

## 13.2 Raw gyro conversion

The configured sensitivity is:

```text
32.8 LSB / dps
```

so:

```text
raw_dps = raw_counts / 32.8
```

Then:

```text
corrected_dps =
    (raw_dps - gyro_bias) * scale_correction
```

Finally:

```text
dtheta = corrected_dps × deg_to_rad × dt
```

---

## 13.3 Bias calibration

Automatic stationary bias calibration requires:

- three encoders to be quiet
- all motor controller targets to be exactly zero
- gyro samples to remain within the configured stationary range

The current thresholds are:

```text
IMU_EPS_SPEED_RPM             = 2.0 RPM
IMU_GYRO_STATIONARY_RANGE_DPS = 1.5 dps
IMU_BIAS_CALIBRATION_SAMPLES  = 100
```

A completed stationary window averages 100 raw gyro samples and updates the bias.

A previously calibrated bias is not replaced if the new estimated bias differs by more than:

```text
IMU_MAX_BIAS_JUMP_DPS = 2.0 dps
```

---

## 13.4 I2C fault recovery

The IMU driver includes an I2C bus recovery mechanism.

On repeated read failures:

1. the IMU is marked faulted
2. PB10/PB11 are temporarily used as open-drain GPIO
3. SCL is toggled up to nine times
4. a STOP condition is generated
5. `HAL_I2C_Init()` is called again
6. `WHO_AM_I` is probed
7. the IMU is returned to normal operation if recovery succeeds

The current consecutive-failure tolerance is:

```text
5 failed reads
```

Recovery attempts while faulted are rate-limited to:

```text
1 second
```

---

# 14. Free Drive

Free Drive is the direct operator-control mode.

## 14.1 Joystick

The browser joystick produces two translational axes:

```text
joystick vertical -> forward/backward
joystick horizontal -> lateral
```

The horizontal UI direction is converted to the robot body convention.

Rotation is controlled independently by the CCW/CW buttons.

The resulting command is:

```text
Vx = joystick forward component
Vy = joystick lateral component
ω  = rotation command
```

---

## 14.2 Current UI scaling

The current browser firmware defines:

```text
maxSpeedMps   = 0.4 m/s
maxOmegaRadps = 3.0 rad/s
```

These are operator-feel parameters, not precision physical calibration values.

---

## 14.3 Odometer display

The browser periodically requests odometry:

```text
CMD_GET_ODOM
```

and receives:

```text
CMD_ODOMETRY_DATA
```

with:

```text
float x
float y
float theta
```

The browser also computes a local traveled-distance accumulator by summing:

```text
sqrt(dx² + dy²)
```

between successive received poses.

That displayed distance is therefore a browser-side derived value, not a separately accumulated STM32 state.

---

# 15. Survey Mode

Survey mode records the robot's estimated position at manually selected vertices.

It is designed for polygon-based area measurement.

## 15.1 Vertex structure

Each vertex contains:

```text
x
y
theta
timestamp_ms
enc1
enc2
enc3
heading_disagreement_signed_deg
heading_disagreement_abs_integral_deg
```

The structure is intentionally fixed at:

```text
36 bytes
```

---

## 15.2 Marking a vertex

When the operator presses **Mark Vertex**:

```text
Browser
  ↓
ESP
  ↓
CMD_MARK_VERTEX
  ↓
Survey_MarkVertex()
```

The STM32 captures:

- current odometry pose
- current encoder counts
- current time
- heading disagreement diagnostics

The browser does not provide the authoritative pose.

This makes vertex capture atomic with respect to the STM32 state.

---

## 15.3 Vertex limit

Current maximum:

```text
64 vertices
```

The system explicitly reports:

- buffer full
- IMU fault
- normal success

---

## 15.4 Closing a survey

On close:

```text
Browser calculates:
    raw polygon area
    closure error
    precision ratio
    Bowditch-adjusted area
```

### Raw area

Uses the Shoelace formula.

### Closure error

The final point is compared to the first point:

```text
closure_error =
sqrt(
    (x_last - x_first)^2 +
    (y_last - y_first)^2
)
```

### Precision ratio

Conceptually:

```text
perimeter / closure_error
```

### Bowditch adjustment

The closure error vector is distributed proportionally along the traverse length and the resulting adjusted polygon area is calculated.

The UI deliberately reports both:

- RAW area
- Bowditch-adjusted area

because the two numbers communicate different information.

---

# 16. Waypoint Patrol

Patrol mode is the current autonomous navigation feature.

The robot receives an ordered list of `(x, y)` waypoints in the current odometry frame.

---

## 16.1 Key design decision

The robot is holonomic.

Therefore it does **not** need to rotate first so that its nose points toward the waypoint.

Instead:

```text
world target vector
        ↓
bearing
        ↓
bearing - current heading
        ↓
body-frame Vx/Vy
        ↓
Kiwi inverse kinematics
```

This means the robot can translate toward a waypoint while maintaining whatever body heading it already has.

---

## 16.2 Patrol state machine

Current states:

```text
NAV_STATE_IDLE
NAV_STATE_DRIVING
NAV_STATE_SETTLING
```

### DRIVING

The controller:

1. reads current pose
2. calculates target vector
3. calculates distance
4. calculates world bearing
5. rotates that desired direction into the current body frame
6. applies an approach-speed ramp
7. generates three wheel RPM targets
8. updates all three wheel-speed controllers

The navigation command always uses:

```text
omega = 0
```

The current patrol controller does not actively command heading rotation.

---

## 16.3 Approach behavior

Current parameters:

```text
NAV_ARRIVE_DIST_M       = 0.15 m
NAV_APPROACH_RAMP_M     = 0.8 m
NAV_APPROACH_FLOOR_PCT  = 30%
```

Far from the waypoint, cruise speed is used.

Within the approach ramp distance, the speed is reduced linearly until the floor percentage is reached.

---

## 16.4 Settling

When:

```text
distance <= 0.15 m
```

the controller cuts motor commands and enters SETTLING.

The purpose is to allow the robot's mechanical inertia to coast into a stable stop while the odometry continues integrating wheel motion.

Five consecutive control ticks with:

```text
d1 == 0
d2 == 0
d3 == 0
```

are treated as confirmed zero movement.

There is also a 2-second settling timeout.

---

## 16.5 Looping behavior

After settling:

```text
target = (target + 1) % waypoint_count
```

Therefore patrol loops:

```text
WP1 → WP2 → WP3 → ... → WP1 → ...
```

until explicitly stopped or a failsafe/fault aborts it.

---

## 16.6 Waypoint capture

The browser can request:

```text
CMD_NAV_MARK_WP
```

The STM32 then captures its live odometry position and appends it to the waypoint list.

The resulting authoritative position is returned through:

```text
CMD_NAV_WP_DATA
```

This prevents the browser's last polled position from being used as the ground truth for the captured waypoint.

---

## 16.7 Encoder disconnect supervision

While a wheel is commanded above:

```text
NAV_ENC_CHECK_MIN_RPM = 30 RPM
```

the firmware watches for zero encoder deltas.

After:

```text
NAV_ENC_DISCONNECT_TICKS = 100
```

consecutive zero-delta ticks, patrol aborts with:

```text
NAV_FAULT_ENCODER_LOST
```

At a nominal 10 ms cycle this corresponds to approximately one second.

Because there is no motor-current sensing, the firmware cannot distinguish:

- disconnected encoder
- hard mechanical stall
- another condition producing the same zero-count signature

from encoder data alone.

---

# 17. Field Calibration

Calibration is designed to measure and correct physical geometry instead of relying entirely on nominal dimensions.

There are two procedures:

```text
ROLL
SPIN
```

The recommended order is:

```text
ROLL first
SPIN second
```

because the spin-based robot-radius calculation depends on a reasonably accurate wheel radius.

---

# 17.1 ROLL test

Purpose:

```text
calibrate wheel radius
```

Procedure:

1. Mark a straight physical line.
2. Place the robot at the start.
3. Start the ROLL test.
4. The firmware commands pure `+X`.
5. The robot stops automatically after the estimated target distance.
6. Measure the true distance traveled on the physical surface.
7. Enter the true distance in the UI.
8. Apply calibration.

The firmware computes the corrected wheel radius from the ratio between:

```text
true physical distance
estimated odometry distance
```

---

# 17.2 SPIN test

Purpose:

```text
calibrate gyro scale
calibrate robot radius
```

The test consists of:

```text
CCW spin
    ↓
coast / settle
    ↓
CW spin
```

For each leg, the firmware records two independent angle estimates:

```text
gyro-integrated angle
encoder/kinematics-implied angle
```

Both are integrated through the mechanical coast phase so they cover the same physical motion.

The two directional legs are retained separately and later combined.

---

## 17.3 Applying spin calibration

The operator supplies the true physical angle reached on:

```text
CCW leg
CW leg
```

The STM32 then combines:

```text
gyro total
kinematics total
physical measured total
```

to update:

```text
gyro_scale
robot_radius
```

The resulting geometry is re-applied to:

```text
g_kin
g_odom
```

and persisted to flash.

---

# 17.4 Calibration result validity

A completed calibration test is cached internally.

Starting a new test invalidates the previous result.

An aborted test does not destroy a previously completed result.

After a successful Apply operation, the cached result is cleared to prevent accidentally applying the same calibration twice.

---

# 18. Communication Protocol

The STM32 and ESP8266 use a custom binary serial protocol.

## 18.1 Frame format

```text
+---------+--------+---------+----------------+------+
| 0xAA    | length | command | payload        | CRC8 |
+---------+--------+---------+----------------+------+
   1 byte   1 byte   1 byte    0..64 bytes      1
```

`length` is defined as:

```text
1 + payload_length
```

because it includes the command byte.

CRC8 is calculated over:

```text
[length][command][payload...]
```

using polynomial:

```text
0x07
```

with initial CRC:

```text
0x00
```

---

# 18.2 STM32 commands

Current command IDs:

| Command | ID | Direction |
|---|---:|---|
| CMD_SET_SPEED | `0x01` | ESP → STM32 |
| CMD_REQUEST_TELEMETRY | `0x02` | ESP → STM32 |
| CMD_SET_PID | `0x03` | ESP → STM32 |
| CMD_STATUS | `0x04` | STM32 → ESP |
| CMD_HEARTBEAT | `0x05` | ESP → STM32 |
| CMD_SET_PWM_PSC | `0x06` | reserved/legacy |
| CMD_RESET_ODOM | `0x07` | ESP → STM32 |
| CMD_GET_ODOM | `0x08` | ESP → STM32 |
| CMD_ODOMETRY_DATA | `0x09` | STM32 → ESP |
| CMD_MARK_VERTEX | `0x0A` | ESP → STM32 |
| CMD_VERTEX_DATA | `0x0B` | STM32 → ESP |
| CMD_CLEAR_VERTICES | `0x0C` | ESP → STM32 |
| CMD_SET_MOTION_MODE | `0x0D` | ESP → STM32 |
| CMD_TELEMETRY_LOG | `0x0E` | STM32 → ESP |
| CMD_GET_ALL_VERTICES | `0x0F` | ESP → STM32 |
| CMD_CLOSE_SURVEY | `0x10` | ESP → STM32 |
| CMD_NAV_CLEAR | `0x11` | ESP → STM32 |
| CMD_NAV_ADD_WP | `0x12` | ESP → STM32 |
| CMD_NAV_START | `0x13` | ESP → STM32 |
| CMD_NAV_STOP | `0x14` | ESP → STM32 |
| CMD_NAV_STATUS | `0x15` | STM32 → ESP |
| CMD_NAV_DEBUG | `0x16` | STM32 → ESP |
| CMD_CAL_SPIN | `0x19` | ESP → STM32 |
| CMD_CAL_ROLL | `0x1A` | ESP → STM32 |
| CMD_CAL_STATUS | `0x1B` | STM32 → ESP |
| CMD_CAL_APPLY_SPIN | `0x1C` | ESP → STM32 |
| CMD_CAL_APPLY_ROLL | `0x1D` | ESP → STM32 |
| CMD_SET_HEADING_HOLD | `0x1E` | defined/reserved |
| CMD_CAL_STOP | `0x1F` | ESP → STM32 |
| CMD_NAV_MARK_WP | `0x20` | ESP → STM32 |
| CMD_NAV_WP_DATA | `0x21` | STM32 → ESP |

The protocol should be treated as a versioned interface. Changing field order or packet length requires updating both STM32 and ESP/browser parsers.

---

# 18.3 Important packet layouts

## CMD_SET_SPEED

Payload:

```text
offset  size  type
0       4     float vx_mps
4       4     float vy_mps
8       4     float omega_radps
```

Total payload:

```text
12 bytes
```

## CMD_STATUS

Payload:

```text
offset  size  type
0       4     float rpm1
4       4     float rpm2
8       4     float rpm3
12      4     int32 encoder1
16      4     int32 encoder2
20      4     int32 encoder3
24      1     uint8 duty1
25      1     uint8 duty2
26      1     uint8 duty3
```

Total:

```text
27 bytes
```

## CMD_ODOMETRY_DATA

```text
float x
float y
float theta
```

Total:

```text
12 bytes
```

## CMD_VERTEX_DATA

Successful response:

```text
status
index
Vertex_t (36 bytes)
```

Total payload:

```text
38 bytes
```

Error response:

```text
status only
```

---

# 19. Telemetry and Diagnostics

The firmware produces two principal telemetry layers.

## 19.1 Standard status

Every approximately 100 ms:

```text
CMD_STATUS
```

contains:

- three measured wheel RPMs
- three raw cumulative encoder positions
- three output duties

This is used by the ESP/browser for the live robot status display.

---

## 19.2 Detailed telemetry log

When enabled:

```text
TELEMETRY_LOG_ENABLED = 1
```

a detailed 61-byte diagnostic payload is sent approximately every 50 ms.

Current fields include:

```text
timestamp
encoder absolute positions
encoder deltas
gyro raw dps
gyro bias
gyro dtheta in degrees
theta in degrees
x
y
heading disagreement signed
heading disagreement absolute integral
motion mode
gyro calibrated flag
gyro scale correction
worst observed tick elapsed time
```

The browser exposes these values through the live log UI.

---

# 19.3 Navigation debug

`CMD_NAV_DEBUG` provides a 44-byte diagnostic packet containing:

```text
state
target index
waypoint count
flags
fault
settle ticks
encoder deltas
distance
heading rate
theta
bearing
bearing error
wheel command RPMs
```

This is intended primarily for diagnosing real-robot patrol behavior.

---

# 20. Safety and Fault Handling

## 20.1 Communication health

The STM32 tracks the time of the last valid received packet.

The connection is considered active only if the latest valid packet arrived within:

```text
200 ms
```

The browser has a separate stale threshold based on the 100 ms status stream.

These are two different layers:

```text
STM32 UART/application health
browser telemetry health
```

Do not confuse them.

---

## 20.2 Failsafe order

During the 10 ms control cycle:

```text
if communication is not connected
OR IMU is faulted
OR encoder fault exists:
```

the firmware:

```text
aborts active navigation
aborts active calibration
zeros manual body command
stops all three motor controllers
```

This is intentionally centralized in `main.c`.

---

## 20.3 Autonomous mode exclusivity

While navigation is active:

```text
CMD_SET_SPEED
CMD_SET_MOTION_MODE
```

are ignored.

While calibration is active, the calibration module owns the three motor controllers.

This prevents manual commands from competing with autonomous control.

---

## 20.4 Encoder supervision

There are currently two different encoder-related ideas in the code:

### Patrol-level encoder disconnect detection

Only watches commanded wheels during active navigation.

### Main-loop encoder fault detection

After a configurable grace period, it compares:

```text
commanded RPM
actual encoder speed
```

and checks whether another wheel is actually moving.

This is intended to identify an individual wheel whose encoder reports almost no motion while another wheel is moving.

There is no current-sensing feedback, so mechanical and electrical failure modes cannot all be distinguished perfectly.

---

# 21. Non-Volatile Calibration Storage

Calibration parameters are stored in the final 1 KB flash page.

Current address:

```text
0x0800FC00
```

The linker script reserves the final 1 KB by defining application FLASH as approximately 63 KB.

---

## 21.1 Stored parameters

```text
gyro_scale
wheel_radius
robot_radius
wheel_scale1
wheel_scale2
wheel_scale3
```

---

## 21.2 Record format

The stored block contains:

```text
magic
version
CRC16
six float parameters
```

Current version:

```text
NVS_VERSION = 3
```

This versioning is important because the meaning of several fields changed during the migration from the previous skid-steer architecture.

---

## 21.3 Validation

NVS data is considered valid only if:

```text
magic matches
version matches
CRC16 matches
```

Otherwise the firmware falls back to defaults.

---

# 22. Timing and Real-Time Considerations

This project is a real-time motion-control system, not a generic desktop application.

The intended control frequency is:

```text
100 Hz
```

with a nominal:

```text
dt = 10 ms
```

---

## 22.1 Measured elapsed time

The control loop measures:

```text
elapsed_ms = now - lastControlTime
```

and derives `dt` from that actual elapsed time.

A maximum numerical cap is applied:

```text
DT_MAX_S = 0.050 s
```

The code deliberately does not silently replace a real delay with the nominal 10 ms value, because doing so would under-integrate motion that actually occurred during a stall.

---

## 22.2 Blocking operations

Several operations in the control path are potentially blocking:

- I2C operations
- UART transmit
- some diagnostic/status sends
- flash operations during calibration apply

The project therefore records:

```text
max_dt_ms
```

over each telemetry interval.

This metric should be watched during real-robot debugging.

A sustained control-loop interval much larger than 10 ms can directly affect:

- gyro integration accuracy
- PID timing
- navigation behavior
- coasting/settling detection
- trajectory consistency

---

# 23. Configuration and Important Constants

The following values have disproportionate impact on physical behavior.

## Kinematics

```text
KIWI_MOUNT_ANGLE1_DEG = 0°
KIWI_MOUNT_ANGLE2_DEG = 120°
KIWI_MOUNT_ANGLE3_DEG = 240°
```

These are currently the default symmetric geometry.

They must match the real chassis.

---

## Odometry/NVS defaults

Current defaults:

```text
wheel_radius = 0.05 m
robot_radius = 0.15 m

wheel_scale1 = 1.0
wheel_scale2 = 1.0
wheel_scale3 = 1.0

gyro_scale = 1.0
```

The wheel radius and robot radius are explicitly documented in the firmware as placeholders.

---

## Encoder

```text
ENCODER_PPR = 1768
ENCODER_SPEED_ALPHA = 0.2
```

---

## IMU

```text
IMU_EPS_SPEED_RPM = 2.0
IMU_GYRO_STATIONARY_RANGE_DPS = 1.5
IMU_BIAS_CALIBRATION_SAMPLES = 100
IMU_MAX_BIAS_JUMP_DPS = 2.0
IMU_I2C_FAIL_TOLERANCE = 5
IMU_RECOVERY_RETRY_MS = 1000
```

---

## PID

Default wheel-speed gains:

```text
Kp = 0.6
Ki = 1.5
Kd = 0.0
```

The output is limited to:

```text
-100 ... +100
```

because the result represents duty-cycle magnitude.

---

## Patrol

```text
NAV_MAX_WAYPOINTS          = 16
NAV_ARRIVE_DIST_M          = 0.15
NAV_APPROACH_RAMP_M        = 0.8
NAV_APPROACH_FLOOR_PCT     = 30
NAV_SETTLE_TICKS           = 5
NAV_SETTLE_TIMEOUT_TICKS   = 200
NAV_BRAKE_ENGAGE_RPM       = 25
NAV_ENC_CHECK_MIN_RPM      = 30
NAV_ENC_DISCONNECT_TICKS   = 100
NAV_STATUS_PERIOD_MS       = 200
NAV_RPM_AT_100PCT          = 236
```

---

## Survey

```text
SURVEY_MAX_VERTICES = 64
```

---

# 24. Build and Development

## 24.1 STM32 project

Open:

```text
Code/rc_kiwi_drive/
```

in STM32CubeIDE.

The main CubeMX project file is:

```text
rc_kiwi_drive.ioc
```

The project targets:

```text
STM32F103C8Tx
```

using:

```text
STM32CubeIDE
GCC
STM32Cube FW_F1 V1.8.7
```

---

## 24.2 CubeMX

The `.ioc` file is part of the source-of-truth configuration.

If CubeMX regenerates code:

1. preserve USER CODE sections
2. verify the pin mapping
3. verify timer allocation
4. verify UART and I2C configuration
5. verify NVIC USART2 interrupt configuration
6. rebuild
7. inspect the generated MSP functions

Do not assume generated peripheral configuration remains correct after changing the `.ioc`.

---

## 24.3 Flash layout

The project uses:

```text
application FLASH ≈ 63 KB
NVS page = final 1 KB
```

Do not allocate application data into the reserved final page.

---

## 24.4 ESP8266 project

The ESP source is:

```text
Esp_Code/esp_v2/esp_v2.ino
```

Required libraries include:

```text
ESP8266WiFi
ESP8266WebServer
WebSocketsServer
```

The ESP builds a single embedded HTML/CSS/JavaScript control application into flash.

---

# 25. Bring-Up / Commissioning Procedure

A professional bring-up should proceed in this order.

## Step 1 — Hardware verification

With motors disconnected:

- verify MCU power
- verify 3.3 V rail
- verify 5 V encoder rail
- verify I2C wiring
- verify UART wiring
- verify SWD access
- verify each GPIO assignment

---

## Step 2 — Single motor verification

Test each wheel independently.

Verify:

```text
positive wheel RPM command
        ↓
correct physical direction
        ↓
encoder count sign
```

The encoder must report the expected sign for the corresponding positive motor direction.

---

## Step 3 — Encoder verification

For each wheel:

- rotate manually
- confirm count direction
- measure count change over a known number of physical revolutions
- verify `ENCODER_PPR`
- verify speed calculation
- verify no excessive count wrap anomalies

---

## Step 4 — PWM / PID tuning

Before autonomous navigation:

- verify duty output
- verify motor deadband
- tune wheel-speed control
- verify positive and negative wheel commands
- verify all three motors reach comparable RPM

---

## Step 5 — Kinematic verification

Test separately:

```text
+X translation
-X translation
+Y translation
-Y translation
+ω rotation
-ω rotation
```

Record:

```text
commanded body velocity
wheel RPM targets
actual wheel RPM
encoder deltas
odometry dx/dy/dtheta
physical movement
```

Do not proceed to patrol until the signs and axis meanings match physical motion.

---

## Step 6 — ROLL calibration

Measure wheel radius on the actual operating surface.

Apply:

```text
wheel_radius
```

---

## Step 7 — SPIN calibration

Run both directions:

```text
CCW
CW
```

Measure the true physical rotation and apply:

```text
gyro_scale
robot_radius
```

---

## Step 8 — Odometry validation

Test:

- straight forward
- straight backward
- pure lateral movement
- diagonal movement
- pure rotation
- combined translation + rotation

Compare:

```text
physical displacement
encoder-derived displacement
gyro heading
encoder-implied heading
heading disagreement
```

---

## Step 9 — Patrol validation

Start with:

```text
2 waypoints
low speed
short distance
```

Then verify:

- correct target direction
- correct lateral behavior
- arrival radius
- settling
- waypoint transition
- full loop
- fault abort

Only increase speed/distance after the low-speed behavior is correct.

---

# 26. Known Limitations and Important Implementation Notes

This section is intentionally explicit. These are important facts about the current repository and should not be mistaken for future design goals.

---

## 26.1 This is not SLAM

The current system does not provide:

- LiDAR
- depth camera
- visual odometry
- UWB localization
- external absolute position
- loop-closure optimization
- occupancy-grid mapping
- graph SLAM

The current map/trajectory position is dead-reckoned from wheel encoders, with gyro-primary heading.

---

## 26.2 No absolute position correction

The robot can accumulate position drift.

The current system has no external mechanism to periodically reset the true `(x,y)` pose except operator-controlled origin reset or future integration of additional sensors.

---

## 26.3 Heading and wheel-kinematic disagreement is diagnostic only

The system calculates encoder-implied heading change, but does not fuse it into the heading state.

Therefore:

```text
dtheta_encoder
```

can disagree with:

```text
dtheta_gyro
```

without automatically changing `theta`.

This is intentional in the current architecture.

---

## 26.4 Kinematic sign convention must be physically verified

State that the internal software inconsistency is resolved. Note that while the software is self-consistent, the physical mapping of +X / +Y relies on the empirical "Left/Right inverse move fix" applied in the latest commits.

---

## 26.5 Patrol arrival is not a point guarantee

Patrol considers a waypoint reached when the estimated distance is inside:

```text
15 cm
```

It then allows the robot to coast.

Therefore the final physical location can differ from the mathematical waypoint by:

- odometry error
- calibration error
- wheel slip
- coast distance
- encoder quantization
- floor interaction

This threshold is a navigation design parameter, not a statement of absolute positioning accuracy.

---

## 26.6 Patrol has no navigation timeout

The navigation module intentionally has no per-waypoint timeout.

As long as safety conditions remain valid, it continues trying to reach the waypoint.

This is different from calibration, which has explicit safety timeouts.

---

## 26.7 Heading is used continuously during patrol

Patrol does not store a fixed startup heading and use it forever.

The actual implementation reads the current odometry heading every navigation tick and rotates the target bearing into the current body frame.

Therefore gyro drift directly affects the world-to-body conversion used by patrol.

---

## 26.8 `Odometry_GetLastDeltas()` timing

Within `Nav_Update()`, diagnostics and settling supervision read:

```text
Odometry_GetLastDeltas()
```

before the current control tick reaches the later `Odometry_Update()` call in `main.c`.

Consequently, some navigation supervision decisions refer to the most recently completed odometry delta rather than the delta that will be generated later in the current tick.

This is important when reasoning about exact one-tick timing.

---

## 26.9 Speed filtering introduces dynamic lag

Encoder speed is filtered:

```text
alpha = 0.2
```

The PID controller therefore does not respond to a perfectly instantaneous wheel-speed estimate.

This may be completely acceptable for the physical system, but it should be considered during control tuning.

---

## 26.10 PID reverse behavior

The motor controller only applies a PID output if the sign agrees with the target direction.

For example, for a positive target:

```text
PID output >= 0 → forward
PID output < 0  → stop
```

It does not use a negative PID output to reverse the motor under that positive setpoint.

The inverse applies for a negative target.

This behavior should be kept in mind when analyzing overshoot, disturbance recovery, and transient behavior.

---

## 26.11 Blocking communication

`Comm_SendPacket()` currently uses blocking:

```text
HAL_UART_Transmit()
```

This can consume part of the 10 ms control budget, especially when several diagnostic packets are sent.

The telemetry `max_dt_ms` field exists partly to expose this behavior.

---

## 26.12 Blocking I2C

The MPU6050 read is also blocking and has an I2C timeout.

Repeated I2C recovery therefore has the potential to affect control-loop timing.

This is a known real-time architecture consideration.

---

## 26.13 No battery telemetry

The current STM32 status packet contains no battery voltage/current field.

The browser UI therefore cannot report actual battery state from the robot firmware.

---

## 26.14 Current UI and firmware are tightly coupled

The ESP8266 firmware embeds the entire browser application.

Therefore protocol changes often require coordinated updates to:

```text
STM32 comm.h/comm.c
STM32 main.c / subsystem modules
ESP command mapping
ESP response parsing
browser JavaScript
```

Treat packet layouts as a shared interface contract.

---

# 27. Recommended Development Order

The safest order for continuing development is:

```text
1. Hardware / pin validation
        ↓
2. Single motor + encoder validation
        ↓
3. PID wheel-speed control
        ↓
4. Kiwi kinematics
        ↓
5. Odometry
        ↓
6. IMU heading validation
        ↓
7. Heading/encoder diagnostics
        ↓
8. Field calibration
        ↓
9. Free-drive validation
        ↓
10. Survey validation
        ↓
11. Waypoint patrol
        ↓
12. Communication hardening
        ↓
13. Higher-level mapping/localization
        ↓
14. ROS 2 integration
```

A higher-level subsystem should not be used to hide a lower-level physical or mathematical error.

For example:

```text
do not tune patrol
until kinematics and odometry are trustworthy

do not tune area measurement
until wheel scale and odometry are trustworthy
```

---

# 28. Future Direction

The long-term goal is a more capable indoor autonomous robot with:

- precise room navigation
- repeatable patrol loops
- real-time map representation
- improved localization
- obstacle detection
- sensor fusion
- higher-quality autonomous behavior
- eventual ROS 2 integration

A future ROS 2 architecture could separate responsibilities into:

```text
STM32
    ↓
low-level motor / encoder / IMU interface
    ↓
ROS 2 hardware interface
    ↓
odometry
    ↓
localization
    ↓
mapping / SLAM
    ↓
planner
    ↓
navigation controller
```

The current firmware is therefore best viewed as the low-level embedded foundation for the future robot rather than the final autonomy stack.

---

# 29. Development Principles

This repository should be developed with the following principles.

## 29.1 Keep hardware truth in the firmware

The STM32 should remain authoritative for:

- encoder state
- pose
- motion state
- safety state
- autonomous state
- calibration state

The UI should display and request state, not invent it.

---

## 29.2 Keep units explicit

Use:

```text
meters
meters/second
radians
radians/second
RPM
encoder counts
milliseconds
```

and convert only at well-defined interfaces.

---

## 29.3 Do not mix mechanical calibration with software tuning

Examples:

```text
wheel radius
robot radius
encoder PPR
gyro scale
wheel scale
```

are physical/calibration parameters.

Examples:

```text
PID gains
approach speed
arrival radius
joystick max speed
settle ticks
```

are control-behavior parameters.

They should not be casually interchanged.

---

## 29.4 Preserve protocol compatibility

Any change to:

- packet ID
- payload size
- field order
- byte interpretation
- units
- endianness

must be reflected across all protocol participants.

---

## 29.5 Prefer measured diagnostics over assumptions

When behavior is wrong, inspect:

```text
command
wheel target RPM
wheel actual RPM
encoder delta
gyro raw
gyro corrected
dtheta encoder
dtheta gyro
heading disagreement
x
y
theta
control-loop dt
```

before changing the controller.

---

## 29.6 Separate symptoms from root causes

For a real robot, the correct debugging question is:

> At which stage does the physical reality diverge from the mathematical/software expectation?

A useful diagnostic chain is:

```text
UI command
   ↓
ESP command
   ↓
UART packet
   ↓
STM32 command state
   ↓
Kiwi wheel target
   ↓
PID output
   ↓
physical wheel motion
   ↓
encoder counts
   ↓
kinematic reconstruction
   ↓
odometry pose
   ↓
navigation decision
```

This chain should be used when diagnosing future motion and navigation problems.

---

# 30. Author / Repository

Repository:

**https://github.com/AND2237/3wheel_KiwiDrive**

Main STM32 firmware:

```text
Code/rc_kiwi_drive
```

ESP8266 bridge + browser application:

```text
Esp_Code/esp_v2/esp_v2.ino
```

---

## Final Note

This README describes the **current implementation**, not an idealized future architecture.

Where the source code contains provisional constants, compatibility interfaces, temporary diagnostics, or known implementation/documentation inconsistencies, those are intentionally documented here rather than silently presented as finished design decisions.

Before deploying the robot for precision navigation or measurement, validate the complete physical chain:

```text
mechanical geometry
→ motor direction
→ encoder sign
→ encoder PPR
→ kinematic wheel angles
→ wheel radius
→ robot radius
→ gyro bias
→ gyro scale
→ wheel scale
→ odometry
→ navigation
```

A mathematically elegant navigation layer cannot compensate for an incorrect physical model underneath it.
