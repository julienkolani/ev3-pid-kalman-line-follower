# EV3 PID Kalman Line Follower

Line-following robot controller for LEGO Mindstorms EV3, implemented in Pybricks MicroPython. Three control strategies of increasing sophistication: Bang-Bang, PID, and Kalman filter-based state estimation.

## Overview

Implements reactive robot control with sensor fusion (color sensor + gyroscope). Tracks position via three parallel methods for comparison and validation. Logs 15+ real-time metrics per cycle.

## Features

- Bang-Bang threshold control
- PID controller (Kp=1.2, Ki=0.1, Kd=0.001) with adaptive speed for curves
- Kalman filter for position/angle estimation fusing odometry and gyroscope
- Three position tracking methods: PID integration, robot state, gyroscope
- DataLog system recording x, y, theta, velocity, error, command
- Obstacle detection with ultrasonic sensor

## Tech Stack

- Pybricks MicroPython (EV3)
- LEGO Mindstorms EV3 brick
- Color sensor, gyroscope, ultrasonic sensor
- Motors (Port B/C)

## Structure

- `tp/TP1-TP2/` - Bang-Bang and PID implementations
- `tp/tp3_kalman/` - Kalman filter and advanced position tracking
- `td/` - Calibration and image view exercises

## Setup

Deploy Python files to EV3 brick using Pybricks IDE or VS Code extension. Requires Pybricks firmware on EV3.
