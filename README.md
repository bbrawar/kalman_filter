# Kalman Filter

## Overview
The **Kalman Filter** is an algorithm that uses a series of measurements over time to estimate unknown variables, even when the measurements are noisy or incomplete. It is widely used in control systems, signal processing, and other areas requiring dynamic system estimation.

### Key Concepts:
- **State Variables**: The variables that describe the system at a given time (e.g., position and velocity).
- **Measurement Variables**: Observations that provide partial information about the state variables.
- **Prediction-Correction Process**: The Kalman filter operates in two stages:
  1. **Prediction**: Estimates the current state based on the previous state.
  2. **Correction**: Updates the prediction using the new measurement.

## Kalman Filter Assumptions
1. **Linear Dynamics**: The system is assumed to be linear, which means the next state is a linear function of the current state.
2. **Gaussian Noise**: The noise in the process and measurement is assumed to be Gaussian (i.e., normally distributed).
3. **Markov Process**: The current state depends only on the previous state (and not on states before that).

## Kalman Filter Algorithm

### 1. Predict
- **State Prediction**: 
  $
  \hat{x}_{k|k-1} = F_k \hat{x}_{k-1} + B_k u_k
  $
  Where:
  - $\hat{x}_{k|k-1}$ is the predicted state at time step $k$,
  - $F_k$ is the state transition model,
  - $B_k$ is the control-input model,
  - $u_k$ is the control input.

- **Covariance Prediction**: 
  $
  P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k
  $
  Where:
  - $P_{k|k-1}$ is the predicted estimate covariance,
  - $Q_k$ is the process noise covariance.

### 2. Update (Correction)
- **Kalman Gain**:
  $
  K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
  $
  Where:
  - $K_k$ is the Kalman gain,
  - $H_k$ is the observation model,
  - $R_k$ is the measurement noise covariance.

- **State Update**:
  $
  \hat{x}_k = \hat{x}_{k|k-1} + K_k (z_k - H_k \hat{x}_{k|k-1})
  $
  Where:
  - $z_k$ is the actual measurement.

- **Covariance Update**:
  $
  P_k = (I - K_k H_k) P_{k|k-1}
  $

## Applications of Kalman Filter
1. **Navigation and Control**: Used in systems like GPS and autopilot to estimate position, velocity, and other state variables.
2. **Signal Processing**: Helps filter out noise from time-series data.
3. **Robotics**: Used in robot localization and motion tracking.

## Kalman Filter Variants
1. **Extended Kalman Filter (EKF)**: Used for non-linear systems by linearizing around the current estimate.
2. **Unscented Kalman Filter (UKF)**: Provides a more accurate way of handling non-linear systems by using unscented transforms.

## Strengths and Limitations
### Strengths:
- Optimal for linear systems with Gaussian noise.
- Efficient, as it runs in real-time and requires limited computational resources.

### Limitations:
- Assumes linearity; performance degrades for highly non-linear systems.
- Requires accurate models of system dynamics and noise.
