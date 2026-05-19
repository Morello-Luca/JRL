# UR5 Manipulation Project

A simple UR5 robot manipulation controller using mc_rtc.

---

# Project Goal

This project controls a UR5 robot arm using:

- mc_rtc
- C++
- Cartesian control
- a small state machine

The robot:

1. Saves its initial pose
2. Moves to a waypoint
3. Returns to the initial pose
4. Stops in IDLE

---

# Main Idea

The controller moves the robot end-effector (`tool0`) in 3D space.

The motion is controlled through:

- a Posture Task
- an End Effector Task
- a simple State Machine

---

# Project Structure

```text
Controller
│
├── PostureTask
│
├── EndEffectorTask
│
└── State Machine
     │
     ├── WAYPOINT
     ├── GO_HOME
     └── IDLE
```

---

# End Effector

The end effector is:

```cpp
tool0
```

This is the robot tool/wrist frame.

The controller moves this frame in space.

---

# Coordinate System

The robot moves in:

| Axis | Meaning |
|------|----------|
| X | forward / backward |
| Y | left / right |
| Z | up / down |

Example:

```cpp
Eigen::Vector3d translation(-0.07, 0.05, 0.15);
```

Means:

- X = move backward 7 cm
- Y = move sideways 5 cm
- Z = move upward 15 cm

---

# Controller Functions

The controller has 3 important functions:

| Function | Purpose |
|----------|----------|
| Constructor | Creates tasks |
| reset() | Initializes motion |
| run() | Executes control loop |

---

# Constructor

```cpp
Ur5Manipulation::Ur5Manipulation(...)
```

Runs once when the controller starts.

It:

- adds constraints
- creates tasks
- adds tasks to solver
- configures stiffness and weight

---

# Tasks

## PostureTask

```cpp
postureTask
```

Purpose:

- keeps robot stable
- avoids strange joint configurations

---

## EndEffectorTask

```cpp
eeTask_
```

Purpose:

- moves `tool0` to a target pose

---

# Stiffness and Weight

Example:

```cpp
eeTask_->stiffness(5.0);
eeTask_->weight(100.0);
```

---

## Stiffness

Controls how aggressively the robot moves.

Low stiffness:
- smooth
- slow

High stiffness:
- faster
- more aggressive

---

## Weight

Controls task priority inside the solver.

High weight:
- task becomes more important

Low weight:
- other tasks dominate

---

# reset()

Called when the controller starts or resets.

---

## Step 1 — Read Current Pose

```cpp
homePose_ = realRobot().bodyPosW("tool0");
```

The robot saves the current end-effector pose.

This becomes the HOME pose.

---

## Step 2 — Create Waypoint

```cpp
Eigen::Vector3d translation(-0.07, 0.05, 0.15);
```

Defines an offset from HOME.

Waypoint is computed as:

```text
waypoint = home + offset
```

---

## Step 3 — Send Target

```cpp
eeTask_->set_ef_pose(waypointPose_);
```

The robot starts moving toward the waypoint.

---

# run()

Runs continuously during controller execution.

Typically:
- 200 Hz
- 500 Hz
- 1000 Hz

Inside:

```cpp
manageStateMachine();
```

The controller checks:
- current state
- task error
- transitions

---

# State Machine

The controller has 3 states:

| State | Description |
|------|--------------|
| WAYPOINT | Move to waypoint |
| GO_HOME | Return to home pose |
| IDLE | Stop moving |

---

# State Flow

```text
RESET
  ↓
WAYPOINT
  ↓
GO_HOME
  ↓
IDLE
```

---

# Error Computation

```cpp
double error = eeTask_->eval().norm();
```

Measures the distance between:
- current pose
- target pose

---

# Transition Condition

```cpp
if(error < 0.01)
```

If the error becomes small enough:
- target is considered reached
- state changes

---

# WAYPOINT State

Robot moves toward waypoint.

When reached:

```cpp
eeTask_->set_ef_pose(homePose_);
```

The controller sends the robot back HOME.

---

# GO_HOME State

Robot returns to initial pose.

When HOME is reached:
- controller enters IDLE

---

# IDLE State

Robot keeps current pose stabilized.

No new target is sent.

---



# Example Motion

```text
Robot Start Pose
        ↓
Move to Offset Waypoint
        ↓
Return Home
        ↓
IDLE
```


