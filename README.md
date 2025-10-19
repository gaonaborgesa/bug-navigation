# Bug Navigation Algorithm

> **Note:** This project was originally completed as part of CSC 386: Autonomous Systems at Berea College. This is a personal copy to showcase my work.

## Overview
Implemented a bug navigation algorithm in C++ for autonomous robot navigation. The robot can drive to specific goal positions and navigate around obstacles using sensor feedback and odometry.

## Technical Details
- **Language:** C++
- **Hardware:** MBot-Omni robot platform
- **Key Concepts:** Frame transformations, odometry, autonomous navigation, obstacle avoidance
- **Precision:** Achieves goal positioning within 3cm accuracy

## Implementation

### Part 1: Robot Hits the Spot
Developed a precise point-to-point navigation system:
- Implemented frame transformation calculations to convert between global and robot coordinate frames
- Created `driveToPose()` function that uses real-time odometry to drive the robot to specific (x, y) coordinates
- Command-line interface for entering goal positions
- Continuous odometry tracking with 3cm accuracy threshold

**Key Achievement:** The robot autonomously drives to user-specified coordinates and stops within 3cm of the target.

### Part 2: Bug Navigation
Extended the navigation system with obstacle avoidance capabilities:
- Implemented bug navigation algorithm for navigating around obstacles
- Integrated sensor feedback for obstacle detection
- Combined goal-seeking behavior with reactive obstacle avoidance

## Technical Challenges Solved
- Converting between coordinate frames (global to robot frame)
- Real-time odometry processing for accurate positioning
- Balancing goal-directed movement with obstacle avoidance
- Achieving precise positioning within tolerance thresholds

## What I Learned
- Coordinate frame transformations in robotics
- Odometry-based navigation and position tracking
- Bug navigation algorithms for autonomous systems
- Real-time sensor integration and decision-making
- Precision control for accurate positioning

## Usage
Run the program and enter target x, y coordinates via command line. The robot will autonomously navigate to the goal position.

## Project Instructions
Full project specifications available [here](https://robotics102.github.io/projects/a2.html).

## Acknowledgments
Project completed as part of CSC 386 coursework at Berea College.
