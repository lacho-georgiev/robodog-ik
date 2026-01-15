# RoboDog Leg IK (Python + Arduino)

This repo contains the code I used to control a 2-actuator RoboDog leg with inverse kinematics (IK).

- **Python (desktop):** solves the leg geometry in real time, shows a clickable 2D simulator, and (optionally) sends servo commands over Serial.
- **Arduino (PCA9685):** receives target servo pulses and moves the two servos smoothly using a PCA9685 driver.

The idea is simple:
1) You click a target point for the foot (point **D**) in the plot  
2) Python solves a physically valid leg configuration (angles + linkage points)  
3) Python converts angles → PWM microseconds  
4) Arduino interpolates to the new PWM targets (smooth motion)

---

## What’s inside

### `python` (IK + visualizer + optional hardware)
- Models a planar linkage with fixed anchors **O1** and **O2**
- Uses:
  - a 2-link solver for the **upper servo** horn + rod (reach point **B**)
  - a constraint/root search to satisfy the **lower actuator** rigid length (reach point **C**)
- Picks the “best” solution branch using continuity (minimizes sudden angle jumps)
- Displays the leg in matplotlib and animates motion to the clicked target

### `arduino` (motion receiver for PCA9685)
- Drives two channels on PCA9685 (CH0 = upper, CH1 = lower)
- Accepts simple Serial commands and clamps to safe pulse ranges
- Smoothly interpolates motion over a chosen time (ms)

---

## How it works (high level)

**Python → Arduino protocol (Serial):**
- `home`  
  Moves both servos to the calibrated home position.
- `g <us0> <us1> <ms>`  
  Smooth move to target pulses (microseconds) over `ms` milliseconds.
- `p <us0> <us1>`  
  Immediate set (no smoothing).

Python mainly sends the `g` command.

---

## Running it (viewer-friendly)

### 1) Simulator only (no hardware)
In the Python file set:
```py
HARDWARE_ENABLED = False
