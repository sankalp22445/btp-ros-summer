
# Robot Status Estimation

The **robot\_status\_estimation** package is designed to simulate and estimate the belief states of a multi-agent system in a robotics context. The system runs in a **ROS** (Robot Operating System) environment and involves two main parts:

1. **Agent belief simulation** where agents (robots) update their beliefs based on observations of other agents.
2. **Belief fusion** where the beliefs of all agents are fused into a single estimation per agent.

This is a practical application of **Bayesian filtering** techniques used in multi-robot systems for tasks like belief propagation, status monitoring, and data fusion.

---

## Table of Contents

* [Overview](#overview)
* [Detailed Explanation](#detailed-explanation)

  * [Bayesian Belief Update](#bayesian-belief-update)
  * [Belief Fusion](#belief-fusion)
* [Installation](#installation)
* [Usage](#usage)

  * [Running the Launch File](#running-the-launch-file)
  * [Logging Behavior](#logging-behavior)
* [Package Structure](#package-structure)
* [Scripts](#scripts)

  * [belief\_main\_robot.py](#belief_main_robotpy)
  * [belief\_fusion\_node.py](#belief_fusion_nodepy)
* [Dependencies](#dependencies)
* [License](#license)

---

## Overview

The **robot\_status\_estimation** package enables multi-agent belief estimation using Bayesian updates and belief fusion in a **turtlesim** simulation environment.

### Core Features:

1. **Agent Simulation**: Simulates multiple agents (turtles) that have initial beliefs about the status of other agents (alive or dead).
2. **Belief Update**: Each agent updates its belief about the status of other agents using a Bayesian update formula based on noisy observations.
3. **Belief Fusion**: Aggregates the beliefs from all agents using a simple averaging technique to produce a fused belief per agent.
4. **Logging**: Records the beliefs and state updates in CSV files for later analysis.

---

## Detailed Explanation

### Bayesian Belief Update

Each agent in the simulation has an initial belief about the **true state** of other agents. The true state is a binary value (`1` for alive, `0` for dead). However, due to sensor noise, the agent's observation may not always match the true state. The **Bayesian update** method adjusts the belief using the following formula:

* **Prior** (`P(belief)`): The agent's belief before the observation is made.
* **Observation**: The agent's observation of another agent’s state (either `1` or `0`).
* **Likelihood**: The probability that an agent observes a certain state given the actual state (sensor noise).

Each agent adjusts its belief using Bayesian reasoning based on these noisy observations. The update allows the agent to refine its belief about the status of other agents over time, improving the accuracy of its belief despite noisy or uncertain data.

### Belief Fusion

The **belief\_fusion\_node** listens for belief updates from agents (through the `/agent/belief` topic). It collects all received beliefs and **fuses them** using an averaging strategy. This ensures that each agent has an updated belief that represents a more robust estimate of its state.

The fusion process involves averaging the belief values that are received over time. This helps mitigate the impact of noisy data and ensures that the final belief estimate is a more stable and accurate reflection of the agent’s true state.

---

## Installation

To install the `robot_status_estimation` package, follow these steps:

1. **Clone the repository** into your `catkin_ws/src` directory:

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/sankalp22445/robot_status_estimation.git
   ```

2. **Install dependencies** using `rosdep`:

   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace**:

   ```bash
   catkin_make
   ```

4. **Source the workspace**:

   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

---

## Usage

Once the package is installed, you can launch the entire system.

### Running the Launch File

Run the following command to start the simulation:

```bash
roslaunch robot_status_estimation robot_belief.launch
```

This will trigger the following:

* **Turtlesim Simulation**: A simple 2D environment where each agent (turtle) is represented.
* **belief\_main\_robot.py**: A Python script that simulates multiple agents, performs belief updates, and publishes belief messages to the `/agent/belief` topic.
* **belief\_fusion\_node.py**: A Python script that listens for belief updates and averages them to compute fused beliefs.

---

### Logging Behavior

Both nodes generate log files to track the states and beliefs of the agents:

1. **belief\_main\_robot.py** logs:

   * **True State**: The actual status of an agent (`alive` or `dead`).
   * **Observed State**: The agent’s observation of another agent’s status.
   * **Prior Belief**: The agent’s belief about the state of another agent before the update.
   * **Updated Belief**: The belief after performing the Bayesian update.

2. **belief\_fusion\_node.py** logs:

   * **Time**: The timestamp of when the belief was fused.
   * **Agent**: The name of the agent whose belief was fused.
   * **Fused Belief**: The averaged belief for the agent based on all received messages.

---

## Package Structure

The `robot_status_estimation` package is structured as follows:

```
robot_status_estimation/
├── launch/
│   └── robot_belief.launch           # Launch file to start simulation and nodes
├── scripts/
│   ├── belief_fusion_node.py         # Node for belief fusion and logging
│   └── belief_main_robot.py         # Node for agent belief updates and movement
├── CMakeLists.txt                   # Build configuration file
└── package.xml                      # Package metadata
```

---

## Scripts

### belief\_main\_robot.py

This script simulates the behavior of agents (turtles) and performs belief updates based on observations. Key tasks include:

* **Spawning Agents**: The agents (turtles) are spawned at random positions in the simulation environment.
* **Movement**: The robot moves to the position of other agents and performs observations.
* **Belief Update**: Bayesian belief updates are applied based on the observed state of the agents.

### belief\_fusion\_node.py

This node listens for belief updates from other agents and performs belief fusion. Key tasks include:

* **Fusion Strategy**: Beliefs from all agents are averaged to compute a fused belief for each agent.
* **Logging**: Logs the fused belief for each agent in a CSV file for future analysis.

---

## Dependencies

This package relies on the following ROS packages:

* `rospy`: ROS Python client library.
* `std_msgs`: Standard ROS message types.
* `geometry_msgs`: ROS message types for geometry-related data.
* `turtlesim`: Simple 2D simulation environment for turtles.

Ensure these dependencies are installed by running:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

