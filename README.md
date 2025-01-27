# Nonlinear Quadrotor UAV Controller Implementation

## Project Overview
This project implements a nonlinear control system for a quadrotor UAV using nested saturation techniques. The controller is designed to stabilize the quadrotor's position and attitude while providing robust performance for trajectory tracking.

## System Architecture
The control system consists of four main components:
- Altitude Control (z-axis)
- Yaw Control (ψ)
- Pitch Control (θ)
- Roll Control (φ)

Each controller utilizes nested saturation techniques to ensure bounded control inputs and stable system response.

## Technical Specifications

### UAV Parameters
- Mass: 1 kg
- Arm Length: 0.22 m
- Moments of Inertia:
  - I_x = 5e-3 kg⋅m²
  - I_y = 5e-3 kg⋅m²
  - I_z = 9e-3 kg⋅m²
- Rotor Inertia: 4e-5 kg⋅m²
- Thrust Coefficient: 3e-6
- Drag Coefficient: 1.5e-7

### Controller Parameters
- Altitude Control Saturation Limits: r1 = 3, r2 = 1.4
- Yaw Control Saturation Limits: ψ1 = 1.5, ψ2 = 0.7
- Pitch/Roll Control Parameters:
  - Theta: a = 1.7, b = 0.82, c = 0.36, d = 0.16
  - Phi: a = 1.7, b = 0.82, c = 0.4, d = 0.18

## Simulation Features
- Time step: 0.01 seconds
- Total simulation time: 100 seconds
- Real-time visualization of:
  - Position (x, y, z)
  - Orientation (φ, θ, ψ)
  - Control inputs (thrust and torques)

## Results Visualization
The simulation produces three main plots:
1. Position tracking in x, y, and z directions
2. Orientation angles (roll, pitch, yaw)
3. Control inputs (thrust and torques)

## Usage
1. Ensure MATLAB is installed
2. Run the main script `code.m`
3. The simulation will execute and display results automatically

## Dependencies
- MATLAB (core functionality only, no additional toolboxes required)

## Notes
- This implementation is based on nonlinear control theory using nested saturation techniques
- The system includes complete 6-DOF quadrotor dynamics
- All control inputs are bounded to ensure physical feasibility

## Future Improvements
- Hardware implementation capabilities
- Additional trajectory types
- Disturbance rejection analysis
- Parameter tuning interface

## Mathematical Model
The implementation uses a complete dynamic model including:
- Rigid body dynamics
- Aerodynamic effects
- Gravitational forces
- Control moment generation

For detailed equations and derivations, refer to the original research paper.
