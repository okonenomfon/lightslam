# LightSLAM: Monocular Visual Odometry System

**LightSLAM** is a lightweight, real-time Visual SLAM (Simultaneous Localization and Mapping) system written in modern C++ (C++17). It estimates the 6-DoF trajectory of a monocular camera by tracking features and optimizing geometry without reliance on external sensors like IMUs or GPS.

## Key Features
* **Sparse Optical Flow Tracking:** Utilizes Lucas-Kanade (LK) flow to track Shi-Tomasi features across temporal frames, reducing computational load compared to frame-by-frame matching.
* **Geometric Initialization:** Decomposes the Essential Matrix to recover initial camera pose (Rotation/Translation) using Nistér’s 5-point algorithm.
* **Motion-Only Bundle Adjustment:** Integrates **Ceres Solver** to minimize reprojection error, refining the estimated trajectory in real-time.
* **High Performance:** Optimized matrix operations using **Eigen3** and a custom memory-efficient pipeline.

## Tech Stack
* **Language:** C++
* **Computer Vision:** OpenCV
* **Linear Algebra:** Eigen3
* **Optimization:** Ceres Solver
* **Build System:** CMake & Vcpkg

## How to Build (Windows/Linux)
This project uses **vcpkg** for dependency management to ensure a smooth build process.

### Prerequisites
* CMake
* Visual Studio 2019/2022 (Windows) or GCC (Linux)
* [vcpkg](https://github.com/microsoft/vcpkg)

### Build Instructions
# 1. Clone the repository
git clone [https://github.com/okonenomfon/LightSLAM.git](https://github.com/okonenomfon/LightSLAM.git)
cd LightSLAM

# 2. Create build directory
mkdir build && cd build

# 3. Configure (Point to your vcpkg toolchain)
cmake .. -DCMAKE_TOOLCHAIN_FILE=[path_to_vcpkg]/scripts/buildsystems/vcpkg.cmake

# 4. Build
cmake --build . --config Release

### Usage
Run the executable from the build directory. Ensure a webcam is connected.

./Debug/LightSLAM.exe
# Window 1: 
Shows the live camera feed with tracked feature points (Green).

# Window 2: 
Displays the real-time estimated trajectory (Red).