# AeroSim: Autonomous & Manual Drone Flight Simulator

A physics-driven 3D quadcopter simulation platform built on Python and the Ursina Engine (Panda3D). Designed for hardware-in-the-loop (HIL) prototyping, autonomous navigation benchmarking, and manual aerobatics testing.

---

## Key Systems & Architecture

- **Aero & Vector Dynamics:** Real-time multi-axis attitude control (pitch, roll, yaw), non-linear air drag models, and altitude-correlated wind shear vectors.
- **Power & Telemetry Management:** Dynamic battery discharge model tied directly to throttle load, with emergency auto-descent fail-safes during brownouts.
- **Sensor Fusion & Avionics HUD:** Glass cockpit instrumentation displaying real-time ground clearance, vertical climb rates (VSI), heading, gate tracking bearings, and ground-proximity warnings.
- **Multi-Perspective Optical Array:** Modular Chase cam, low-latency FPV cockpit, and 360° tactical orbital observation views.
- **Gate-Crossing Detection:** Plane-crossing vector dot-product calculations for competitive autonomous waypoints and precision landing scoring.

---

## Control Mapping

| Command | Key | Description |
| :--- | :--- | :--- |
| **Throttle Up / Down** | `Space` / `L-Shift` | Proportional motor thrust (Hover ~50%) |
| **Pitch Axis** | `W` / `S` | Forward / Backward tilt |
| **Roll Axis** | `A` / `D` | Port / Starboard lateral tilt |
| **Yaw Axis** | `Q` / `E` | Rotational heading control |
| **Camera View** | `C` | Toggle between Chase, Cockpit FPV, and Orbit |
| **Diagnostic Reset** | `R` | Reset run state and telemetry cache |
| **Emergency Abort** | `Esc` | Kill simulation process |

---

## Quickstart

```bash
# Clone the repository
git clone [https://github.com/noelvmathews/autonomous-drone-navigation.git](https://github.com/noelvmathews/autonomous-drone-navigation.git)
cd autonomous-drone-navigation

# Install dependencies
pip install -r requirements.txt

# Run the simulator
python main.py
