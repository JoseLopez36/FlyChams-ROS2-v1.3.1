# Flying Chameleons: Multi-Camera UAV Simulation in Unreal Engine with AirSim

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Directory Structure](#directory-structure)
- [Configuration](#configuration)
- [Known Limitations and Issues](#known-limitations-and-issues)

## Introduction

This repository contains a simulation framework designed to model multiple Unmanned Aerial Vehicles (UAVs), each equipped with independently controllable multi-camera systems using Unreal Engine and AirSim. The UAVs are capable of providing enhanced versatility in monitoring and tracking multiple mobile targets through independently orientable cameras, equipped with zoom functionality. This allows for real-time adjustment of the focal length for optimal visual coverage.

The primary objective of the project is to simulate these UAVs collaborating in a shared mission scenario, enabling simultaneous tracking of various distant targets. In addition, the project leverages Unreal Engine 5's capabilities as a graphical simulation tool, offering photorealistic environments to validate the performance of multi-camera systems in real-time operations. The implementation supports the exploration of multiple UAV agents working in coordination to monitor large groups of mobile targets, presenting potential applications in fields such as search and rescue, traffic inspection, and wildlife monitoring​.

## Prerequisites

To run this project, you will need the following software and tools:

1. **Windows 11**  
   The simulation has been tested and is known to work on Windows 11. It has not been verified on Windows 10 or other versions.

2. **Unreal Engine 5.2.1**  
   Download and install Unreal Engine 5.2.1. This version is required for compatibility with the AirSim plugin and the project's specific configurations. For more information on installing Unreal Engine, visit the official [Unreal Engine documentation](https://docs.unrealengine.com/5.2/en-US/).

3. **Visual Studio 2022 LTSC 17.8**  
   You will need Visual Studio 2022 LTSC 17.8 (other versions have not been tested). Ensure that the **C++ development tools** are installed, as they are required to build and run both the client and the Unreal Engine project.

Make sure all prerequisites are installed and properly configured before proceeding with the usage of the project.

## Installation

### General Installation
1. **Clone the repository:**
   
   ```bash
   git clone https://github.com/JoseLopez36/FlyingChameleons.git

3. **Set up the environment variable for the project directory:**
   - Open the Start Search, type "env", and select **Edit the system environment variables**
   - In the System Properties window, click on the **Environment Variables...** button
   - In the User variables section, click **New...**
   - Set **Variable name** to `FLYCHAMS_PATH` and **Variable value** to the path of your project directory (e.g., `C:\path\to\your\project\directory`)

### Unreal Project Installation
1. **Generate the Visual Studio project for Unreal Engine:**  In the root directory of the project, locate the `.uproject` file. Right-click on this file and select **Generate Visual Studio project files**. This will create the necessary Visual Studio solution for the Unreal Engine project.

2. **Build the Unreal Engine project:**  Open the newly generated Visual Studio solution for Unreal Engine. Set the build configuration to **Debug Editor** and build the project. 

### Client Installation
1. **Build the client solution:**   Navigate to the `FlyingChameleonsClient` folder and open the Visual Studio solution. Set the build configuration to **Debug** mode and build the solution. This will compile the client application, which communicates with the AirSim server.
   
### PX4 Installation

To set up PX4 for use with AirSim, follow these steps:

   #### PX4 Cygwin Toolchain Installation Steps:
   - Download and run the installer with default settings: [PX4 Windows Cygwin Toolchain](https://s3-us-west-2.amazonaws.com/px4-tools/PX4+Windows+Cygwin+Toolchain/PX4+Windows+Cygwin+Toolchain+0.9.msi).
   - After installation, run `run-console.bat` found in the toolchain installation directory.
   - Execute the following command to install necessary Python packages:
     
     ```bash
     pip3 install --user kconfiglib jsonschema future
     ```
   - Clone the PX4-Autopilot repository:
     
     ```bash
     git clone --recursive -j8 https://github.com/PX4/PX4-Autopilot.git
     ```
   - Run the setup script:
   
     ```bash
     bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
     ```
   - Navigate to the new repository directory and checkout this specific version:
     
     ```bash
     cd PX4-Autopilot
     git checkout v1.11.3
     ```
   - Build the PX4 SITL with this configuration:
     
     ```bash
     make px4_sitl_default none_iris
     ```
     (Close the process with **CTRL+C** when done.)

More information in [Cygwin Toolchain setup guide](https://docs.px4.io/main/en/dev_setup/dev_env_windows_cygwin.html#getting-started) and [PX4 SITL in AirSim setup guide](https://microsoft.github.io/AirSim/px4_sitl/).
   
### QGroundControl Installation (Optional)

You can use QGroundControl to monitor the UAVs. Download and install Windows version in [QGroundControlDailyBuilds](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html).
   
## Usage

Follow these steps to configure and launch the simulation:

1. **Configure the simulation settings**  
   Open the `.xlsx` configuration file located in the `FlyingChameleonsConfiguration` folder. This file allows you to adjust parameters such as the number of UAV agents, the number of cameras per agent, and specific camera properties. Make sure to save your changes once you’ve completed your configuration.

2. **Run the PX4 SITL Controller**  
   - Open a terminal and run `run-console.bat` in the PX4 toolchain installation directory.
   - Navigate to the PX4-Autopilot directory:
     
     ```bash
     cd PX4-Autopilot
     ```
   - Launch PX4 SITL for multiple agents (replace `nVehicles` with the appropriate number):
     
     ```bash
     ./Tools/sitl_multiple_run.sh nVehicles
     ```

3. **Run the simulation in Unreal Engine**  
   In the Unreal Engine project solution, launch the program without debugging by pressing **CTRL+F5**. Once the program is running, press the **Play** button to start the simulation.

4. **Run the client process**  
   Launch the client executable located at: `FlyingChameleonsClient\build\FlyingChameleonsClient.exe`

You can now interact with the simulation and control the UAV agents through the AirSim client and the Unreal Engine environment.

## Directory Structure

### 1. `FlyingChameleonsConfiguration`
This folder contains the main configuration files, including settings for the number of agents, cameras per agent, and the specific characteristics of each agent and their cameras. The configuration is stored in `.xlsx` (Excel) format to provide a clear and user-friendly interface for adjusting project parameters.

### 2. `FlyingChameleonsClient`
This folder houses the Visual Studio solution for the simulation client. The client communicates with the server (the AirSim plugin in Unreal Engine) through the AirSim APIs (AirLib in C++), allowing the client-side control and visualization of the simulation.

### 3. `Plugin/AirSim`
This directory stores the AirSim plugin for Unreal Engine, specifically the customized fork of AirSim Colosseum used in this project.

### 4. `Plugin/AirSim/Source`
This folder contains the source code for the AirSim plugin. It also includes custom project files that begin with `FlyingChameleons_` (e.g., `FlyingChameleons_HUD_Manager`), which are specific to this simulation.

### 5. `Content`
This folder holds all project-related content for Unreal Engine, such as assets, blueprints, and other necessary resources.

## Configuration

The simulation configuration is managed through an `.xlsx` file located in the `FlyingChameleonsConfiguration` folder. This file consists of several sheets, each serving a specific purpose in defining the simulation parameters. Below is a brief overview of each sheet and its functionality:

1. **Missions**  
   This sheet outlines the general characteristics of the missions to be simulated. You can define multiple missions, but only the one marked with an **X** in the `missionSelected` column will be chosen for execution.

2. **Scenarios**  
   This sheet contains details about the simulated environment, such as the time of day and weather conditions.

3. **TargetGroups**  
   This sheet defines the targets to be followed, including their target type ID (e.g., HUMAN, with more types planned for future updates) and the trajectory ID. The trajectory ID corresponds to a CSV file that should be placed in the same configuration folder.

4. **ChamSquad**  
   Here, you specify each agent involved in the mission, detailing their central camera characteristics (such as gimbal and camera models) and other agent attributes (like initial position and drone model).

5. **TrackingHeads**  
   This sheet defines the tracking cameras for each agent, including specifications such as gimbal model and camera model.

6. **Clustering**  
   This sheet outlines the characteristics of the clustering mechanism used in the simulation.

7. **DroneModels**  
   In this sheet, you specify the characteristics of the chosen drone model, such as maximum speed.

8. **GimbalModels**  
   This sheet contains specifications for the gimbal associated with the head, including its maximum speed.

9. **CameraModels**  
   Here, you detail the characteristics of the camera associated with a head, including default focal length.

### Important Notes
When configuring any of the above sheets, it is crucial to maintain the order of the indices used to ensure the simulation functions correctly. Make sure to review each entry carefully to align with your simulation goals.


## Known Limitations and Issues

1. **Hardware Requirements**  
   The simulation requires a mid-to-high-end PC to run with photorealistic quality. Below are the recommended minimum specifications:
   - **CPU**: Quad-core Intel or AMD, 2.5 GHz or faster
   - **GPU**: DirectX 11 or 12 compatible graphics card with the latest drivers
   - **RAM**: 8 GB

   Lower-end systems may still run the simulation, but with significantly reduced performance and visual quality.

2. **Limit on Number of Agents and Cameras**  
   The project currently supports a maximum of 4 UAV agents, each equipped with up to 4 cameras. 

3. **Performance Degradation with Multiple Cameras**  
   When the total number of cameras across all agents exceeds 6, the simulation may experience a significant slowdown. This is due to the increased computational load required in the optimization of the agent positions.
