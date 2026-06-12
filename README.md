<a id="readme-top"></a>

<div align="center">
  <h3 align="center">mc_rtc Controllers Collection</h3>
  <p align="center">
    A centralized monorepo housing modular, real-time robot controllers built on the mc_rtc framework.
  </p>
</div>

---

## Overview

This repository serves as a unified workspace for developing and managing multiple robotic control applications. Instead of maintaining separate repositories for different research experiments, kinematic tasks, or hardware setups, this monorepo structures each controller as an isolated sub-folder under a single master CMake build system.

Detailed installation instructions, state machine architectures, and hardware-specific configurations are located within the local `README.md` file of each respective controller directory.

### Available Controllers

```text
mc_rtc-controllers-collection/
├── dualarm/            # Real-time dual-arm coordinated impedance control
└── firstController/    # Basic position tracking and joint space trajectories
