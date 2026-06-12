<a id="readme-top"></a>

<!--
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Unlicense License][license-shield]][license-url]
-->

<br />
<div align="center">
  <a href="https://github.com/github_username/dual-arm-control">
    <img src="images/logo.png" alt="Logo" width="120" height="120" onerror="this.src='https://img.shields.io/badge/Robot-mc__rtc-blue?style=for-the-badge'">
  </a>

  <h3 align="center">Dual-Arm Impedance Control</h3>

  <p align="center">
    A real-time Finite State Machine (FSM) dual-arm manipulation controller built on the mc_rtc framework for twin xArm7 robotic manipulators.
    <br />
    <a href="https://github.com/github_username/dual-arm-control/issues/new?labels=bug">Report Bug</a>
    &middot;
    <a href="https://github.com/github_username/dual-arm-control/issues/new?labels=enhancement">Request Feature</a>
  </p>
</div>

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-project">About The Project</a></li>
    <li><a href="#built-with">Built With</a></li>
    <li><a href="#finite-state-machine-fsm">Finite State Machine (FSM)</a></li>
    <li><a href="#control-architecture">Control Architecture</a></li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>

---

## About The Project

This project implements a multi-robot controller leveraging the mc_rtc framework to coordinate two xArm7 robotic arms. It utilizes real-time Cartesian task-space control, seamless asynchronous FSM state transitions via raw function pointers, and critically damped end-effector impedance tasks for stable collaborative physical interaction.

### Key Capabilities:
* **Zero Memory Jitter:** Pre-allocates space for all tasks inside the real-time loop to avoid heap allocations (std::make_shared).
* **Rigid Object Kinematics:** Computes virtual object frames and rigid transformation offsets dynamically.
* **Critically Damped Impedance:** Automatically derives damper gains based on virtual stiffness and mass matrices to prevent system oscillations.

### Dependencies

Before compiling, ensure you have the following system packages and libraries installed:

* **Core Framework:** `mc_rtc`
* **Other Packages:** `mc_xarm`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![mc_rtc][mc_rtc-shield]][mc_rtc-url]
* [![Eigen][Eigen-shield]][Eigen-url]
* [![C++][Cplus-shield]][Cplus-url]
* [![ROS][ROS-shield]][ROS-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Finite State Machine (FSM)

The controller executes an asynchronous state machine using a memory-safe double-pointer transition pattern (StateMethod):

| Phase | State Function | Duration / Trigger | Core Objective |
| :--- | :--- | :--- | :--- |
| **1. IDLE** | stateIdle | t >= 3.0s | Absolute holding state to settle kinematics/gravity initialization. |
| **2. INDEPENDENT** | stateIndependent | Cartesian error < 3 cm & speed < 0.5 cm/s | Both end-effectors independently drive to distinct spatial coordinates. |
| **3. COLLABORATIVE** | stateCollaborative | Linear interpolation down path | Arms lock onto an interpolated virtual object frame via rigid offsets. |

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Control Architecture

### 1. Kinematic Layout & Collision Bounds
Both robotic arms are managed within the same Cartesian workspace. The primary instance (xarm7) sits at the origin, while the secondary instance (xarm7_2) is loaded with a 0.5-meter offset along the Y-axis. Global self-collision constraints prevent geometric impacts between all links of the two robots.

### 2. Impedance Task Configurations
Coupled force control is sustained through 6D task spaces ordered as [Angular_X, Angular_Y, Angular_Z, Linear_X, Linear_Y, Linear_Z]:

* **Virtual Mass Matrix (M):** Configured as a constant 1.0 across all dimensions.
* **Virtual Stiffness (K):** Set to a constant 200.0 N/m for the elastic response profile.
* **Analytic Damping Calculation (D):** Derived analytically to ensure critical damping and avoid destructive system oscillations: D = 2 * sqrt(K * M).
* **Feedforward Target Wrench:** Preloads a constant 25.0 N linear downforce directed along the Z-axis.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Getting Started

### Prerequisites
* Fully installed and configured mc_rtc framework workspace.
* Kinematic descriptions for the xArm7 robot registered inside the RobotLoader path.

### Installation & Compilation

1. Clone the repository into your local mc_rtc source directory:
   git clone https://github.com/github_username/dual-arm-control.git
   cd dual-arm-control

2. Compile the C++ source files using CMake:
   mkdir build && cd build
   cmake ..
   make

3. Enable the plugin module inside your global system configuration (~/.config/mc_rtc/mc_rtc.yaml):
   MainRobot: xArm7
   Enabled: DualArmControl

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Roadmap

- [x] Pre-allocate tasks to eliminate real-time memory jitter
- [x] Analytic calculation for critical damping matrices
- [x] Trajectory interpolation of virtual frames for multiple waypoints
- [ ] Collect data of force and position errors during manipulation
- [ ] Implement and test on real Hardware

See the [open issues](https://github.com/github_username/dual-arm-control/issues) for a full list of proposed extensions.

---

## License

Distributed under the Unlicense License. See LICENSE for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



[mc_rtc-shield]: https://img.shields.io/badge/Framework-mc__rtc-blue?style=for-the-badge&logo=robotframework
[mc_rtc-url]: https://jrl-umi-aist.github.io/mc_rtc/
[Eigen-shield]: https://img.shields.io/badge/Linear_Algebra-Eigen3-orange?style=for-the-badge
[Eigen-url]: https://eigen.tuxfamily.org/
[Cplus-shield]: https://img.shields.io/badge/Language-C%2B%2B17-00599C?style=for-the-badge&logo=c%2B%2B
[Cplus-url]: https://en.cppreference.com/w/cpp/17
[ROS-shield]: https://img.shields.io/badge/Middleware-ROS_/_ROS2-22314E?style=for-the-badge&logo=ros
[ROS-url]: https://www.ros.org/
