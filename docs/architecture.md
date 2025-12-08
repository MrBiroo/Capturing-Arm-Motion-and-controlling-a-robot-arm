# System Architecture — Detailed Notes

This document expands the high-level architecture from the README and provides more detail on communication, timing and node responsibilities.

Nodes
-----
1. Wearable Node (ESP32)
   - Responsibilities: Read IMU (MPU-6500 via SPI/I2C) and BNO055 where used, read analog potentiometers, run preliminary filtering (optional), timestamp, and transmit sensor frames.
   - Interfaces: I2C/SPI, ADC, UART, WiFi/UDP.

2. Bridge Node (optional)
   - Responsibilities: Aggregate multiple wearable nodes, provide buffering, translate packet formats, forward to control station or motor controllers.
   - Useful when many sensors are distributed.

3. Control Station (PC/RPi)
   - Responsibilities: High-level mapping, kinematics, visualization, logging.
   - Runs Python tools: filters tuning, inverse kinematics solver, safety checks.

4. Motor Controller Node(s)
   - Responsibilities: Receive joint target angles or currents, run high-frequency FOC loops (≥1 kHz controller update), close current loops, limit torques, and provide encoder feedback.

Latency & Timing
---------------
- IMU sample rate: typically 500–1000 Hz (MPU-6500 supports up to 1000 Hz).
- FOC control loop: recommended ≥1 kHz for current control, position loop slower (100–500 Hz).
- Target end-to-end latency: <20–50 ms for responsive mapping; reduce by using wired serial for critical control, otherwise ensure WiFi QoS.

Packet design (recommended)
--------------------------
- Binary, compact for low-latency.
- Include timestamp, sensor_id, quaternion (floats), analog pot floats or raw ADCs, sequence number, CRC32.
- Keep total packet size small (<128 bytes) for reliability.

Safety & Redundancy
-------------------
- Include watchdog timers in motor controllers to stop motors on communication loss.
- Hard limits & software limits to avoid joint overrange or collisions.
- Redundant angle source (pot + encoder + IMU) for calibration and safety.

Logging & Debugging
-------------------
- Log raw sensor streams in experiments/sensor_raw_data with timestamps in CSV or binary.
- Save filter outputs and joint targets for replay and analysis.
