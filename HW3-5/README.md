# HW3-5: Kinematic Control, Dynamic Control of 3-Wheel Omni Robot & SMC 

## Description
This homework extends the kinematic model of a 3-wheel omni robot by
introducing wheel dynamics and control.

## Features
- Body-to-wheel and wheel-to-body kinematics
- Discrete-time wheel dynamics
- PI controller
- Sliding Mode Controller (SMC)
- Phase portrait plots (e vs e_dot)
- Sliding surface convergence

## Files
- HW3-5.slx : Simulink model
- PARAMETERS.m : robot and controller parameters
- Run_SMC.m : simulation + plotting script

## Running the code
- In PARAMETERS.m there are 3 types of body velocities
- Straight line, Circle, Pure Spin
- You can only run one at a time by commenting out other 2. 
- To get the results of each case we need to do the same one after another
