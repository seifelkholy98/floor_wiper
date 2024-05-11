# Robot Cleaning Simulation Project

## Overview
This project utilizes the Robot Operating System (ROS) to create a simulation environment where an autonomous cleaning robot navigates, detects dirt spots, and cleans them. The project integrates sensor inputs with predictive modeling to optimize the cleaning process.

## Project Structure
- `robot_mover.py`: Controls the robot's movement and decision-making processes based on real-time sensor data and predictive inputs.
- `dirt_sensor.py`: Simulates the detection of dirt within the robot's operational environment and publishes these locations.
- `predictive_cleaner.py`: Analyzes past cleaning data to predict future dirt locations, enhancing the robot's cleaning efficiency.
- `robot_simulation.launch`: Contains the configuration to launch the ROS simulation environment, setting up the necessary nodes and topics.
- `floor_wiper.urdf`: Describes the physical attributes of the robot, including dimensions and visual representation for simulation purposes.

## Installation
To get started with this simulation, ensure you have ROS installed on your system. Follow these steps to set up the project:

1. **Clone the repository** into your ROS workspace's `src` directory:
    ```bash
    cd ~/catkin_ws/src
    git clone repository-URL
    ```
2. **Build the project** to compile all necessary components:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

## Usage
After installation, you can start the robot simulation with the following command:
```bash
roslaunch floor_wiper robot_simulation.launch
