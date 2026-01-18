### Overview:
This project is a lightweight C++ implementation of a Monocular Visual Odometry system. It takes a raw video feed from a single camera and reconstructs the 3D trajectory of the camera's movement in real-time. Unlike basic demos that rely solely on OpenCV's high-level APIs, this project implements the core mathematical optimization pipeline using Eigen and Ceres Solver.

### Key Technical Features
1. Front-End (Tracking): Uses the Lucas-Kanade algorithm (Optical Flow) to track robust "Good Features to Track" (Shi-Tomasi corners) across frames. This avoids the computational overhead of re-detecting features every frame.
2. Initialization: Handles monocular initialization by decomposing the Essential Matrix ($E = t^\wedge R$) to recover the initial rotation and translation.
3. Back-End (Optimization): Implements a custom Cost Functor in Ceres Solver. The system performs local bundle adjustment, refining the estimated pose ($R, t$) by minimizing the geometric reprojection error of triangulated 3D points.
4. Memory Management: rigorous management of feature lifecycles; automatically pruning outliers and replenishing features when the track count drops below a threshold to maintain system stability.

### Build System
1. Managed via CMake for cross-platform compatibility.
2. Dependencies (OpenCV, Eigen, Ceres) handled strictly via vcpkg to ensure reproducible builds.