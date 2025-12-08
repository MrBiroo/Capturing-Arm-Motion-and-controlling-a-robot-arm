# Hardware — Design Notes & Placeholders

This file lists the hardware design considerations, placeholders for PCB and mechanical files, and key mechanical design constraints.

Mechanics
---------
- Gearboxes: split-ring epicyclic design to get high reduction in compact volume. Factor backlash improvements by preloading and resin printing high-accuracy parts.
- Links: design for print orientation that minimizes support contact in resin prints and improves surface finish for joints.
- Fasteners: use metal inserts where repeated load/unload is expected.

Electronics
-----------
- Sensor PCBs: small breakout boards for BNO055 and MPU-6500 with level shifting and a common I2C bus topology.
- Motor driver boards: current sense, MOSFET stage, gate drivers, bootstrap circuits, and decoupling.
- Power: separate power domains for logic and motor to reduce noise coupling.

Placeholders 
---------------------------------
- /hardware/pcb/  — PCB schematics and gerbers or KiCad project
- /hardware/sensor_mounts/  — STL or STEP for sensor housings
- /hardware/gearbox_design/ — gearbox CAD assemblies
- /hardware/3d_print_files/ — sliced files and print setup notes

Testing & calibration
---------------------
- IMU orientation calibration: record static bias and scale from multiple poses.
- Motor calibration: measure phase resistance and inductance, encoder offsets, and torque constants.
- Gearbox backlash measurement: measure free-play and document preloads or shimming required.

