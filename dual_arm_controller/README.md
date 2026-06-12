<a id="readme-top"></a>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Unlicense License][license-shield]][license-url]

<br />
<div align="center">
  <a href="https://github.com/github_username/dual-arm-control">
    <img src="https://raw.githubusercontent.com/xArm-Developer/xarm_ros/master/res/xarm7_architecture.png" alt="Logo" width="120" height="120" onerror="this.src='https://img.shields.io/badge/Robot-mc__rtc-blue?style=for-the-badge'">
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
Le due braccia robotiche sono gestite nello stesso spazio cartesiano. L'istanza principale (xarm7) si trova nell'origine, mentre la seconda (xarm7_2) viene caricata con un offset di 0.5 metri sull'asse Y. I vincoli di auto-collisione globale impediscono impatti geometrici tra tutti i link dei due robot.

### 2. Impedance Task Configurations
Il controllo accoppiato della forza si basa su vettori a 6 dimensioni ordinati come [Angular_X, Angular_Y, Angular_Z, Linear_X, Linear_Y, Linear_Z]:

* **Virtual Mass Matrix (M):** Impostata a 1.0 costante su tutti gli assi.
* **Virtual Stiffness (K):** Impostata a 200.0 N/m costante per la risposta elastica.
* **Analytic Damping Calculation (D):** Calcolato analiticamente per garantire lo smorzamento critico ed evitare oscillazioni distruttive: D = 2 * sqrt(K * M).
* **Feedforward Target Wrench:** Applica una forza costante di 25.0 N diretta verso il basso lungo l'asse lineare Z.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Getting Started

### Prerequisites
* Framework mc_rtc installato e configurato correttamente.
* Descrizioni cinematiche del robot xArm7 registrate all'interno del RobotLoader.

### Installation & Compilation

1. Clona il progetto nella cartella dei tuoi controller mc_rtc:
   git clone https://github.com/github_username/dual-arm-control.git
   cd dual-arm-control

2. Compila i sorgenti C++ tramite CMake:
   mkdir build && cd build
   cmake ..
   make

3. Abilita il plugin aggiungendo il modulo nel tuo file di configurazione globale (~/.config/mc_rtc/mc_rtc.yaml):
   Enabled: DualArmControl

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Roadmap

- [x] Pre-allocazione dei Task per azzerare il memory jitter in tempo reale
- [x] Calcolo analitico dinamico della matrice di smorzamento critico
- [x] Interpolazione lineare dei waypoint dell'oggetto virtuale
- [ ] Integrazione diretta dei flussi dei sensori di forza/coppia reali
- [ ] Implementazione dell'interpolazione sferica (Slerp) sull'orientamento del frame

See the [open issues](https://github.com/github_username/dual-arm-control/issues) for a full list of proposed extensions.

---

## License

Distributed under the Unlicense License. See LICENSE for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



[contributors-shield]: https://img.shields.io/github/contributors/github_username/dual-arm-control.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/dual-arm-control/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/dual-arm-control.svg?style=for-the-badge
[forks-url]: https://github.com/github_username/dual-arm-control/network/members
[stars-shield]: https://img.shields.io/github/stars/github_username/dual-arm-control.svg?style=for-the-badge
[stars-url]: https://github.com/github_username/dual-arm-control/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/dual-arm-control.svg?style=for-the-badge
[issues-url]: https://github.com/github_username/dual-arm-control/issues
[license-shield]: https://img.shields.io/github/license/github_username/dual-arm-control.svg?style=for-the-badge
[license-url]: https://github.com/github_username/dual-arm-control/blob/master/LICENSE

[mc_rtc-shield]: https://img.shields.io/badge/Framework-mc__rtc-blue?style=for-the-badge&logo=robotframework
[mc_rtc-url]: https://jrl-umi-aist.github.io/mc_rtc/
[Eigen-shield]: https://img.shields.io/badge/Linear_Algebra-Eigen3-orange?style=for-the-badge
[Eigen-url]: https://eigen.tuxfamily.org/
[Cplus-shield]: https://img.shields.io/badge/Language-C%2B%2B17-00599C?style=for-the-badge&logo=c%2B%2B
[Cplus-url]: https://en.cppreference.com/w/cpp/17
[ROS-shield]: https://img.shields.io/badge/Middleware-ROS_/_ROS2-22314E?style=for-the-badge&logo=ros
[ROS-url]: https://www.ros.org/
