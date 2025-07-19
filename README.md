# Robot Status Estimation

This package, **robot\_status\_estimation**, simulates a multi-agent environment using the **turtlesim** package. It estimates and fuses the belief states of multiple agents in a robotic system, using Bayesian updates based on noisy observations. The system is designed to work in a ROS (Robot Operating System) setup.

---

## Table of Contents

* [Overview](#overview)
* [Installation](#installation)
* [Usage](#usage)

  * [Running the Launch File](#running-the-launch-file)
* [Package Structure](#package-structure)
* [Scripts](#scripts)

  * [belief\_main\_robot.py](#belief_main_robotpy)
  * [belief\_fusion\_node.py](#belief_fusion_nodepy)
* [Dependencies](#dependencies)
* [License](#license)

---

Overview
The robot_status_estimation package enables multi-agent belief estimation using Bayesian updates and belief fusion in a turtlesim simulation environment.

Core Features:
Agent Simulation: Simulates multiple agents (turtles) that have initial beliefs about the status of other agents (alive or dead).

Belief Update: Each agent updates its belief about the status of other agents using a Bayesian update formula based on noisy observations.

Belief Fusion: Aggregates the beliefs from all agents using a simple averaging technique to produce a fused belief per agent.

Logging: Records the beliefs and state updates in CSV files for later analysis.

---

## Installation

To use the package, follow these steps to install it in your ROS workspace:

1. Clone the repository into your `catkin_ws/src` directory:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/sankalp22445/robot_status_estimation.git
   ```

2. Install the required dependencies:

   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   catkin_make
   ```

4. Source the workspace:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

---

## Usage

Once the package is installed, you can launch the system and begin running the belief estimation and fusion.

### Running the Launch File

To start the simulation and begin belief estimation, use the following command:

```bash
roslaunch robot_status_estimation robot_belief.launch
```

This launch file will:

1. Start the **turtlesim** simulation.
2. Run the **belief\_main\_robot** node, which simulates agents and their belief updates.
3. Run the **belief\_fusion\_node**, which collects beliefs from agents and fuses them to produce an average belief.

---

## Package Structure

```
robot_status_estimation/
├── launch/
│   └── robot_belief.launch          # Launch file to start simulation and nodes
├── scripts/
│   ├── belief_fusion_node.py        # Node for belief fusion and logging
│   └── belief_main_robot.py        # Node for agent belief updates and movement
├── CMakeLists.txt                  # Build configuration file
└── package.xml                     # Package metadata
```

---

## Scripts

### belief\_main\_robot.py

This script simulates a robot (or multiple agents) and performs the following:

* **Spawns agents** at random positions within the simulation environment.
* **Simulates belief updates** using a Bayesian filter. Each agent's belief about the state of other agents is updated based on noisy observations.
* **Publishes beliefs** to the `/agent/belief` topic for other nodes (like the belief fusion node) to listen to.
* **Logs the results** in CSV format, including true states, observations, and updated beliefs.

Key components:

* **Bayesian Update**: Uses Bayesian reasoning to update agent beliefs based on observations.
* **Agent Movement**: Moves to the location of other agents in the simulation before updating their beliefs.

### belief\_fusion\_node.py

This node listens for belief messages from agents and fuses them using an averaging strategy. It performs the following:

* **Collects beliefs** from the `/agent/belief` topic.
* **Fuses beliefs** by averaging the received beliefs for each agent.
* **Logs fused beliefs** into a CSV file, recording the belief updates over time.

Key features:

* **Belief Averaging**: A simple averaging strategy is used to fuse beliefs over time.
* **Logging**: Logs each fused belief for all agents into a CSV file, with timestamps.

---

## Dependencies

This package depends on the following ROS packages:

* `rospy`: Python client library for ROS.
* `std_msgs`: Standard message types for ROS.
* `geometry_msgs`: Messages for geometry-related data.
* `turtlesim`: ROS package for simple simulation of a turtle robot.

Make sure all dependencies are installed by running:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---
