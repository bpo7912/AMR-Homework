# AMR Homework Submission (HW1–HW7)

This repository contains my AMR homework deliverables:
- **HW1&HW2:**They are attached in the file AMR-HW-Bhavya.pptx
- **HW3–HW5:** 3-wheel omni robot kinematics + dynamic wheel model + Sliding Mode Control (SMC) error analysis
- **HW6:** Control Barrier Function (CBF) navigation around a pedestrian
- **HW7:** Potential Fields (one successful navigation + one local-minimum failure)

---

## Folder Structure

- `HW3-5/`
  - `model/` → Simulink model(s) (`.slx`)
  - `scripts/` → MATLAB scripts (`PARAMETERS.m`, `Run_SMC.m`)
  - `outputs/` → plots/screenshots if needed
- `HW6/`
  - `code/` → MATLAB CBF code (`.m`)
  - `outputs/` → animation video (`.mp4`)
- `HW7/`
  - `code/` → Python potential fields (`.py`)
  - `outputs/` → animation video (`.mp4`)

---

## HW3–HW5 (Simulink: Dynamics + SMC)

### What’s implemented
- **Kinematics:** wheel speeds → body velocities → world frame → integrated trajectory
- **Wheel dynamics:** each wheel modeled as a discrete first-order plant
- **SMC error logging:** wheel speed tracking errors `e_i` and error rates `ė_i`
- **Plots produced:** phase portraits (e vs ė) and sliding surfaces

### How to run
1. Open MATLAB, set current folder to the repo root.
2. Open the Simulink model in `HW3-5/model/` (e.g., `HW3.slx`).
3. Run:
   - `HW3-5/scripts/PARAMETERS.m`
   - `HW3-5/scripts/Run_SMC.m`

Outputs:
- Phase portraits for each wheel: `(e_i vs ė_i)`
- Sliding surfaces: `s_i = c e_i + ė_i`
- XY path from the robot integration

---

## HW6 (CBF Navigation)
Implements a control barrier function to maintain a minimum safe distance from a pedestrian while the robot moves to a goal.

Run:
- Open and run the MATLAB file in `HW6/code/`
Output:
- Saved animation in `HW6/outputs/`

---

## HW7 (Potential Fields)
Implements potential field navigation with:
1. A case that successfully reaches the goal
2. A case that demonstrates failure due to a local minimum

Run:
- `python HW7/code/HW7.py`
Output:
- animation/video in `HW7/outputs/`

---

## Notes
- Simulink cache and build artifacts are ignored via `.gitignore` (e.g., `slprj/`, `*.slxc`).
