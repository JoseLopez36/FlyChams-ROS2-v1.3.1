# FlyingChameleons-ROS2

The **FlyingChameleons-ROS2** project is a modular, reusable, and scalable ROS2-based software architecture designed for multi-UAV systems. The system is composed of several ROS2 packages, each with a specific role in ensuring efficient task planning, perception, control, simulation, and deployment.

This README provides an overview of the system architecture, package descriptions, and node responsibilities.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Packages](#packages)
   - [flychams_interfaces](#flychams_interfaces)
   - [flychams_core](#flychams_core)
   - [flychams_control](#flychams_control)
   - [flychams_perception](#flychams_perception)
   - [flychams_coordination](#flychams_coordination)
   - [flychams_dashboard](#flychams_dashboard)
   - [flychams_bringup](#flychams_bringup)
4. [External Packages](#external-packages)
   - [airsim_interfaces](#airsim_interfaces)
   - [airsim_wrapper](#airsim_wrapper)
5. [Installation and Setup](#installation-and-setup)
6. [Usage](#usage)
7. [License](#license)

---

## Overview

The **FlyingChameleons** system is designed to handle complex multi-UAV missions involving tasks such as object tracking, navigation, and payload control. It leverages ROS2's modularity and scalability to ensure seamless integration between perception, planning, control, and simulation components.

Key features:
- **Modularity**: Each package is (almost) self-contained and reusable.
- **Scalability**: Designed to support multiple UAVs and dynamic mission requirements.
- **Simulation Support**: Includes integration with **Cosys-AirSim** for testing and development.

---

## System Architecture

The system consists of seven core ROS2 packages:

1. **flychams_interfaces**: Shared message, service, and action interfaces.
2. **flychams_core**: Common utilities and base classes.
3. **flychams_control**: Navigation and payload control for UAVs.
4. **flychams_perception**: Perception algorithms for sensor data processing.
5. **flychams_coordination**: Task planning and coordination logic.
6. **flychams_dashboard**: Visualization and user interface components.
7. **flychams_bringup**: Launch files and configuration scripts for system deployment.

---

## Packages

### flychams_interfaces

**Purpose**: Defines shared message, service, and action interfaces.

**Functionality**:
- Provides custom message types for perception, control, and coordination.
- Ensures consistency and compatibility across the system.

---

### flychams_core

**Purpose**: Provides common utilities and base classes.

**Functionality**:
- Contains reusable code snippets and helper functions.
- Standardizes operations like parameter handling and ROS components creation.
- Serves as a dependency for other packages.

---

### flychams_control

**Purpose**: Manages UAV navigation and payload control.

**Functionality**:
- Executes navigation commands received from higher-level packages (`agent_control_node`).
- Interfaces with simulation environments.

---

### flychams_perception

**Purpose**: Implements perception algorithms and nodes for analyzing targets.

**Functionality**:
- Groups detected points into meaningful clusters (`clustering_node`).
- Computes geometric properties of clusters, such as minimal enclosing circle.

---

### flychams_coordination

**Purpose**: Implements task planning logic for UAVs in the FlyingChameleons system.

**Functionality**:
- Plans navigation commands for individual agents (`agent_positioning_node`).
- Assigns clusters to agents based on mission state (`cluster_assignment_node`).
- Plans commands for target (or cluster) tracking using agent payload (`agent_tracking_node`).

---

### flychams_dashboard

**Purpose**: Provides visualization and user interface components.

**Functionality**:
- Create RViz markers and system metrics (for plotting purposes).
- Offers user controls for simulation management.

---

### flychams_bringup

**Purpose**: Simplifies launching and configuration of the entire system.

**Functionality**:
- Contains multi-node launch files for complete system deployment.
- Includes parameter configurations for tuning components.
- Provides nodes for registering agents and targets.

---

## External Packages

### airsim_interfaces

**Purpose**: Provides message and service definitions for AirSim integration with ROS2.

**Functionality**:
- Defines custom message and service types for AirSim communication.
- Enables standardized communication between ROS2 nodes and the AirSim simulator.

---

### airsim_wrapper

**Purpose**: Wraps the AirSim API for ROS2 compatibility.

**Functionality**:
- Provides a ROS2 node for interfacing with AirSim.
- Handles simulation state, vehicle control, and sensor data.
- Provides a simulation clock with adjustable speed for time synchronization.

---

## Installation and Setup

### Prerequisites

- Ubuntu 22.04
- ROS2 Iron
- AirSim (for simulation enviroment)

### Building from Source

1. Clone the repository:
   ```bash
   git clone https://github.com/JoseLopez36/FlyingChameleons
   ```

   ---TODO---

---

## Usage

### Running the System

The system can be launched using the provided launch files in the `flychams_bringup` package:
