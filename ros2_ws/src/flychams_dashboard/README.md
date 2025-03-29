# Flychams Dashboard: Visualization and monitoring package for the Flying Chameleons Project

Visualization and monitoring package for the Flying Chameleons (FlyChams) project that provides user interfaces and system visualization.

## Overview

The `flychams_dashboard` package provides user interface and visualization tools for the FlyChams system. It handles:

1. **GUI interface** - Provides a graphical user interface for monitoring the system
2. **Metrics creation** - Creates metrics for plotting purposes
3. **Marker creation** - Creates markers for visualization purposes

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

## Configuration

Configuration parameters for the dashboard nodes can be found in `flychams_bringup` package.