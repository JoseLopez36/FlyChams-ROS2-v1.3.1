# Flychams Control: Control nodes for the Flying Chameleons Project

Control package for the Flying Chameleons (FlyChams) project that manages the movement and behavior of aerial agents.

## Overview

The `flychams_control` package provides control algorithms for aerial agents in the FlyChams system. It handles:

1. **Agent movement control** - Manages the flight control of aerial platforms (e.g. takeoff, landing, moving to a position, etc.)
2. **Head control** - Manages the control of the heads of the agents (e.g. zoom, pan, tilt)

## Nodes

### Drone Control

- Node that handles UAV platform movement:
  - Manages drone state and transitions between flight modes (e.g. arming, disarming, takingoff, tracking, landing, etc.)
  - Receives position commands from coordination nodes
  - Implements speed scheduling and other control algorithms

### Head Control

- Node that handles head control:
  - Receives head commands from coordination nodes
  - Sends commands to the heads of the agents

## Configuration

Configuration parameters for the control nodes can be found in `flychams_bringup` package.