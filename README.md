# Bicycle Controller

A C++17 implementation of a bicycle kinematic model with position controller algorithms for autonomous navigation.

## Overview

This project implements a simplified bicycle kinematic model commonly used in robotics and autonomous vehicle simulations. It includes two control modes to navigate the bicycle model from a starting position to a target position:
- **PID Controller**: Adjusts steering angle based on heading error
- **Pure Pursuit Controller**: Calculates steering angle to follow a circular arc toward the target

## Features

- **Bicycle Kinematic Model**: Simplified two-wheeled vehicle dynamics
- **Dual Control Modes**:
  - PID control with configurable gains (Kp, Ki, Kd)
  - Pure Pursuit path tracking algorithm
- **Visualization Support**: Optional Raylib-based visualization of the bicycle motion
- **Steering Constraints**: Realistic limits on steering angle (±45°)
- **Angle Wrapping**: Automatic normalization of angles to [-π, π]

## Getting Started

### Prerequisites

- C++17 compatible compiler (e.g., g++, clang++)
- (Optional) Raylib library for visualization support

### Building

#### Basic build (without visualization):
```bash
g++ Bicycle_Model.cpp -o Bicycle_Model.exe
```

#### With Raylib visualization:
1. Install Raylib from https://www.raylib.com/
2. Compile with:
```bash
g++ Bicycle_Model.cpp -o Bicycle_Model.exe -I"path_to_raylib" -L"path_to_raylib_folder" -lraylib -lopengl32 -lgdi32 -lwinmm
```
3. Uncomment `#define ENABLE_VISUALIZATION` in the code

### Running

```bash
./Bicycle_Model.exe
```

## Implementation Details

### BicycleModel Class
Implements the kinematic equations of motion:
- `x_dot = v * cos(θ)`
- `y_dot = v * sin(θ)`
- `θ_dot = (v / L) * tan(δ)`

Where:
- `x, y`: Position coordinates
- `θ`: Vehicle heading angle
- `v`: Velocity
- `δ`: Steering angle
- `L`: Distance between front and rear axles

### PositionController Class
Provides two control strategies:

#### PID Mode
Calculates steering angle to minimize heading error to the target direction.

#### Pure Pursuit Mode
Computes steering angle using the pure pursuit algorithm to follow a circular arc toward the target point.

## Configuration

In `main()`, you can configure:
- Starting position: `x, y, theta`
- Target position: `x_d, y_d`
- Velocity: `v`
- Control mode and gains
- Time step: `dt_global`

## Example

```cpp
BicycleModel bicycle(1.0);  // 1.0m wheelbase
PositionController controller(ControlMode::PURE_PURSUIT, 1.0, 0.5, 0.0);

double x = 12.0, y = 2.0, theta = 30.0;  // Start position
double x_d = 1.0, y_d = 6.0;              // Target position
double v = 1.0;                           // Velocity
```

## Dependencies

- STL libraries: `<tuple>`, `<iostream>`, `<cmath>`, `<chrono>`, `<thread>`, `<vector>`
- (Optional) Raylib for visualization

## Output

The program outputs:
- Current position `(x, y, θ)` at each iteration
- Number of epochs until target is reached
- Visualized path (if ENABLE_VISUALIZATION is enabled)

## Notes

- The bicycle model simplifies vehicle dynamics and does not include side-slip angle (β)
- The integral term in the PID controller is currently set to zero for simplicity
- Steering angle is constrained to ±π/4 radians (±45°) to mimic mechanical limitations
- For detailed problem definition and documentation, refer to the linked Google Drive document in the source code

## License

This project is provided as-is for educational and research purposes.
