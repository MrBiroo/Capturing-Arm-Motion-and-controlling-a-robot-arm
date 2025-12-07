# Wearable Motion-Capture to Robotic Arm — 7-DOF Arm & 3-DOF Neck

Project abstract
----------------
A wearable motion-capture suit using BNO055, MPU-6500, and potentiometers to capture human hand/arm/neck movement and control a 7-DOF robotic arm and 3-DOF robotic neck. The system integrates sensor fusion using Complementary and Kalman filters, Field-Oriented Control (FOC) for BLDC motors, custom-designed split-ring epicyclic gearboxes, and SolidWorks-based mechanical structures fabricated with resin 3D printing. The aim is to replicate human motion with precision.

Summary
-------
This repository contains firmware, software, CAD placeholders, documentation, and filtering/kinematics code used for the mechatronics graduation project that maps human motion to a 7-DOF robotic arm and 3-DOF robotic neck. It is organized so a developer, mechanical engineer, or recruiter can quickly understand design decisions and reproduce or extend the system.

Features
--------
- Wearable IMU + potentiometer sensor suit for arm/neck capture
- Sensor fusion (Complementary + Kalman filters) for robust orientation estimates
- ESP32-based motion-capture node(s) with low-latency transmission to controller
- Arduino/STM32-based motor control running FOC for BLDC motors
- 7-DOF serial robotic arm kinematics solver (DH-based) and 3-DOF neck
- Custom gearbox design and SolidWorks CAD (placeholders)
- Python tools for filter tuning, visualization, and inverse kinematics prototyping

System architecture
-------------------
High-level block diagram (ASCII)

Human -> Wearable Sensors (IMUs + pots)
          |
          v
   ESP32 Motion Capture Node(s)
   - Read: BNO055, MPU-6500, potentiometers
   - Fuse: Complementary/Kalman (optional)
   - Output: serialized packet (UART/USB/WiFi/UDP)
          |
          v
   Bridge / Router Node (optional)
   - ESP32 bridge to Arduino/MCU
   - Protocol: binary, checksum, timestamp
          |
          v
   Motor Controllers (Arduino / BLDC drivers)
   - Run FOC for each motor
   - Receive target joint angles/vels
          |
          v
   Robotic Arm (7-DOF) + Neck (3-DOF)
   - Custom gearboxes and linkages
   - Feedback: encoders/pots

Data flow (ASCII)

[Human Motion]
   |
  IMUs/Pots
   |
[ESP32 MC Node] --(UDP/Serial)-> [Control PC / Raspberry Pi] --(serial)-> [Motor Microcontrollers] -> [Motors]

Photos / 3D renders (placeholders)
---------------------------------
- docs/figures/pose_capture.jpg (placeholder)
- docs/figures/arm_render.png (placeholder)
- docs/cad/assembly.SLDPRT (placeholder)
Replace placeholders by exporting renders / photos and committing them into docs/figures.

Hardware list
-------------
- ESP32 development board(s) (esp32-wroom or esp32-s3 recommended)
- Arduino-compatible MCU or STM32 for motor control (e.g., Arduino Due / STM32F4)
- BLDC motors (quantity: 7 for arm + 3 for neck, motor spec depends on torque/size)
- Motor drivers supporting FOC (or discrete MOSFET + gate drivers)
- BNO055 IMUs (absolute orientation sensor) and MPU-6500 (raw accel/gyro)
- Potentiometers (rotary) for joint angle redundancy & calibration
- Rotary encoders (optional / recommended)
- Power supplies / battery packs sized for motor stall currents
- Custom split-ring epicyclic gearboxes (designed in SolidWorks)
- Resin 3D printed structural parts
- Misc: wiring harnesses, connectors, PCB for sensor breakout

Software stack
--------------
- Embedded:
  - Espressif ESP-IDF or Arduino for ESP32
  - Arduino/PlatformIO for motor controllers
  - SimpleFOC or custom FOC implementation (C/C++)
- PC / prototyping:
  - Python 3.8+ (numpy, scipy, pyserial, matplotlib)
- CAD: SolidWorks (source CAD not included)
- Version control: git (GitHub)

Block diagrams (ASCII)
----------------------
Motion-capture node:

[MPU-6500]   [BNO055]   [Potentiometers]
    \           |           /
     \          |          /
      -> ESP32 Motion Capture (sensor read) -> Sensor fusion -> Packetize -> TX

Motor control node:

[Packet RX] -> [Joint mapping & IK] -> [FOC controllers per motor] -> [PWM drivers] -> [BLDC + gearbox] -> Feedback encoders -> [FOC]

Installation
------------
Prerequisites
- Git
- Python 3.8+
- pip packages: numpy, scipy, pyserial, matplotlib
- ESP32 toolchain (ESP-IDF or Arduino ESP32)
- Arduino IDE / PlatformIO for motor firmware

Clone
- git clone <your-repo-url>
- cd <repo>

Python environment (example)
- python -m venv venv
- source venv/bin/activate
- pip install -r requirements.txt  # (create this file when you add real extras)

How to run the motion-capture firmware (ESP32)
----------------------------------------------
1. Open firmware/esp32_motion_capture/main.cpp
2. Configure I2C pins, BNO055/MPU-6500 addresses, and serial/WiFi settings.
3. Build & flash with:
   - Arduino: select board -> Upload
   - ESP-IDF: idf.py build && idf.py -p /dev/ttyUSB0 flash
4. Use Serial Monitor or UDP to read the output packets. Example packet format (recommended):
   - Header (2B) | timestamp (4B) | sensor_id (1B) | q_w,q_x,q_y,q_z (4x4B floats) | pot1..N (floats) | CRC32

How to run the robotic arm control firmware (Arduino/MCU)
---------------------------------------------------------
1. Open firmware/arduino_motor_control/foc_example.cpp
2. Replace example motor parameters and pin mappings with your hardware values.
3. Build & upload via Arduino/PlatformIO.
4. The controller listens on Serial for joint targets. Example simple packet:
   - Header | joint_count | angle1 .. angleN | timestamp | CRC

Field-Oriented Control (FOC) — explanation
-----------------------------------------
Overview
- FOC controls BLDC motors by aligning stator currents to the rotor magnetic field using rotating reference frames. It enables smooth torque control, high efficiency, and regenerative capabilities.

Main steps
1. Clarke transform: Convert 3-phase currents i_a,i_b,i_c -> α/β stationary 2-axis frame.
2. Park transform: Rotate α/β into d/q frame aligned with rotor (using rotor angle θ).
3. PI controllers: Regulate id (flux) and iq (torque) currents to setpoints.
4. Inverse Park + Inverse Clarke: Convert d/q voltages back to 3-phase voltages.
5. PWM generation (e.g., SVPWM) to drive MOSFETs/driver.

Implementation notes
- Rotor angle can be obtained from encoder, sensorless estimator, or motor-specific observer.
- Tune PI gains on a test bench with torque/voltage/current limits.
- Useful libraries: SimpleFOC (Arduino), custom DSP code for STM32.

Kinematics explanation (DH table, equations)
-------------------------------------------
Notation (Denavit-Hartenberg):
For joint i:
- a_i: link length
- alpha_i: link twist
- d_i: link offset
- theta_i: joint angle

Homogeneous transform for joint i (A_i):

A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)

Matrix form:
A_i = [[cosθ_i, -sinθ_i*cosα_i,  sinθ_i*sinα_i, a_i*cosθ_i],
       [sinθ_i,  cosθ_i*cosα_i, -cosθ_i*sinα_i, a_i*sinθ_i],
       [0,       sinα_i,         cosα_i,        d_i],
       [0,       0,              0,             1]]

Overall transform:
T_0_n = A_1 * A_2 * ... * A_n

DH table (symbolic) — 7-DOF arm (example)
- i | a_i | alpha_i | d_i | theta_i
- 1 | a1  | alpha1  | d1  | theta1
- 2 | a2  | alpha2  | d2  | theta2
- 3 | a3  | alpha3  | d3  | theta3
- 4 | a4  | alpha4  | d4  | theta4
- 5 | a5  | alpha5  | d5  | theta5
- 6 | a6  | alpha6  | d6  | theta6
- 7 | a7  | alpha7  | d7  | theta7

(Replace ai/alpha/d with real link dimensions and joint offsets used in your CAD.)

Neck (3-DOF) DH table (symbolic)
- 1 | a1n | alpha1n | d1n | theta1n
- 2 | a2n | alpha2n | d2n | theta2n
- 3 | a3n | alpha3n | d3n | theta3n

Kinematics notes
- Use numeric forward kinematics to simulate workspace.
- Inverse kinematics: 7-DOF has redundancy — use null-space projection to track human pose while avoiding joint limits and collisions.
- Jacobian J relates joint velocities to end-effector twist: v = J(q) * q_dot
- For control, use resolved-rate or resolved-acceleration methods and add damping / null-space controllers.

Results (placeholder)
---------------------
- Capture latency: target < 20 ms sensor-to-actuator roundtrip (depends on comms and motor controller loop).
- Orientation RMSE (IMU fusion) targeted: < 3 deg on forearm/upperarm after calibration.
- Repeatability: < 0.5 deg after gearbox backlash minimization and calibration (target).
When adding your experimental data, include:
- Sensor fusion comparisons (raw IMU vs complementary vs Kalman)
- Trajectories (human vs robot) overlay plots
- Motor control traces (iq,id,phase currents) and torque step responses
- CAD photos and mechanical stress/FEA snapshots

Future improvements
-------------------
- Add absolute encoders on motors for closed-loop position redundancy
- Improve communications (use RTPS / ROS 2 over WiFi for distributed systems)
- Real-time safety layer: torque limits, joint soft-limits, collision detection
- Motor thermal management & current sensing improvements
- Train a neural IK/mapper for more natural motion mapping
- Integrate haptic feedback for bidirectional teleoperation


Recruiter-friendly highlights
-----------------------------
This project demonstrates:
- Hardware prototyping: sensor breakouts, motor drivers, custom gearbox & resin 3D printing.
- Sensor fusion: implementation and comparison of complementary and Kalman filters for IMUs.
- Robotics & control: DH kinematics, Jacobians, inverse kinematics, redundant DOF handling.
- Embedded systems: ESP32 firmware, low-latency telemetry, Arduino/STM32 motor loops.
- Mechatronics systems engineering: mechanical-electrical-software integration, testing, and iteration.
- Languages & tools: C/C++ (embedded/FOC), Python (prototyping and filters), SolidWorks (CAD).

Contributing
------------
- Add your CAD files to /docs/cad and renders to /docs/figures.
- Implement motor parameter files and motor calibration routines in firmware.
- Add recorded experiment data to /experiments/* and create analysis notebooks.

Repository structure (recommended)
---------------------------------
/docs
   /figures
   /cad
   /diagrams
/firmware
   /esp32_motion_capture
   /esp32_to_arduino_bridge
   /arduino_motor_control
   /foc
/python
   /kalman_filter
   /complementary_filter
   /kinematics_solver
   /quaternion_tools
/hardware
   /pcb
   /sensor_mounts
   /gearbox_design
   /3d_print_files
/schematics
/experiments
   /stress_analysis
   /filtering_results
   /sensor_raw_data

#Authers
Baraa Akbik
Ahmad Malas
