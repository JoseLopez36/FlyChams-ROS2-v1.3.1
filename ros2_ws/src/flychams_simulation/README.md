# Flychams Simulation: Simulation package for the Flying Chameleons Project

Simulation package for the Flying Chameleons (FlyChams) project that contains the logic for managing the simulation environment.

## Overview

The `flychams_simulation` package provides user interface and visualization tools for the FlyChams system. It handles:

1. **GUI manager** - Manages the graphical user interface for monitoring the system
2. **Metrics factory** - Creates metrics for plotting purposes
3. **Marker factory** - Creates markers for visualization purposes
4. **Target state** - Get and publishes the target states
5. **Target control** - Spawn and control simulated tracking targets

## Nodes

### GUI Manager

- Graphical user interface manager for system monitoring:
  - Manages windowing and display of system information
  - Handles user interactions and GUI control

### Metrics Factory

- Handles creation of metrics for plotting purposes:
  - Creates metrics for agents, targets, clusters, etc.
  - Publishes metrics to specific topics
  - Allows plotting of metrics in PlotJuggler or other tools

### Marker Factory

- Handles creation of markers for visualization purposes:
  - Creates markers for agents, targets, clusters, etc.
  - Publishes markers to specific topics
  - Allows visualization of markers in RViz2

### Target State

- Get and publishes the target states:
  - Gets the target states from pre-generated trajectories
  - Publishes the target states to the system

### Target Control

- Spawn and control simulated tracking targets:
  - Spawns simulated tracking targets and corresponding clusters
  - Sends commands to the targets to move along a pre-generated trajectory

## Configuration

Configuration parameters for the simulation nodes can be found in `flychams_bringup` package.