# 🤖 Robot Status Estimation (Bayesian Belief Update in ROS)

This package, **robot_status_estimation**, simulates a multi-agent environment using the **Turtlesim** simulator in ROS. It estimates and updates the belief states (alive/dead) of multiple agents using **Bayesian filtering** based on noisy observations. It includes belief sharing and logging mechanisms and is ideal for learning and demonstrating probabilistic robotics in a ROS environment.

---

## 📚 Table of Contents

- [Overview](#-overview)
- [Detailed Explanation](#-detailed-explanation)
  - [Bayesian Belief Update](#bayesian-belief-update)
  - [Belief Update Equation](#belief-update-equation)
- [Installation](#-installation)
- [Usage](#-usage)
  - [Running the Launch File](#running-the-launch-file)
  - [Logging Behavior](#logging-behavior)
- [Package Structure](#-package-structure)
- [Scripts](#-scripts)
- [Dependencies](#-dependencies)
- [License](#-license)
- [Author](#-author)
- [Future Improvements](#-future-improvements)

---

## 🧠 Overview

The **robot_status_estimation** package enables simulation and belief estimation for multiple agents in a ROS-based 2D world. The goal is to simulate uncertainty and sensor noise, then use **Bayesian reasoning** to update the robot’s belief in the true state of each agent.

### Core Features

- **Multi-Agent Simulation** using `turtlesim`
- **Belief Update** via Bayesian filtering
- **Randomized agent states** (alive/dead)
- **Sensor noise simulation**
- **Real-time CSV logging**
- **Belief sharing via ROS topics**

---

## 🔍 Detailed Explanation

### Bayesian Belief Update

Each agent is assigned a **hidden binary state**: `1` for alive, `0` for dead. The main robot (`turtle1`) visits each agent and makes a **noisy observation** of its state.

The system uses **Bayes’ Theorem** to update its belief:

### 🔣 Belief Update Equation

The Bayesian update rule is used:
P(H|E) = P(E|H)P(H)P(E∣H)P(H) + P(E∣¬H)(1-P(H))
Where:
P(H): Prior belief (initially 0.5)


P(E|H): Likelihood of observation given hypothesis (sensor accuracy)


P(E|¬H): Likelihood of observation given the hypothesis is false


P(H|E): Posterior belief after observation


Example Calculation:
Prior belief P(H)=0.5
Sensor accuracy:


P(E=1|H=1)=0.8 (observation alive given alive)


P(E=1|H=0)=0.2 (false positive rate)


Observation = alive (1)


Then,
P(Alive∣1) = 0.80.50.80.5 + 0.2(1-0.5) = 0.40.4 + 0.1 = 0.40.5 = 0.8

---

## 🔧 Installation

### 1. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/sankalp22445/robot_status_estimation.git
```

### 2. Install Dependencies

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace

```bash
catkin_make
source devel/setup.bash
```

---

## 🚀 Usage

### 🔁 Running the Launch File

```bash
roslaunch robot_status_estimation robot_belief.launch
```

This will:

- Launch the `turtlesim` simulation
- Spawn the main robot (`turtle1`) and 4 agents (`turtle2–turtle5`)
- Start the Bayesian belief update node
- Publish belief updates to `/agent/belief`

---

### 📝 Logging Behavior

Each belief update is recorded in a CSV file (auto-created in the home directory):

**Sample CSV Header:**

```csv
Time,Agent,TrueState,Observed,Prior,UpdatedBelief
```

**Example:**

```csv
37.50,turtle2,1,1,0.50,0.80
```

The log includes:

- Actual state of the agent (ground truth)
- Noisy observation result
- Prior belief before update
- Updated belief after Bayesian filtering

---

## 🗂 Package Structure

```
robot_status_estimation/
├── launch/
│   └── robot_belief.launch            # Launches simulation and main node
├── scripts/
│   └── belief_main_robot.py          # Main robot node for movement + belief update
├── log/
│   └── belief_log_TIMESTAMP.csv      # Auto-generated log files
├── CMakeLists.txt
└── package.xml
```

---

## 🧾 Scripts

### 🔹 belief_main_robot.py

- Spawns 5 agents at random positions
- Assigns each a true state (alive or dead)
- Main robot (`turtle1`) moves to each agent’s location
- Makes a noisy observation (e.g., 60% accuracy)
- Applies Bayesian update
- Publishes belief to `/agent/belief`
- Logs all interactions in a `.csv` file

---

## 📦 Dependencies

Make sure the following ROS packages are installed:

- `rospy`
- `std_msgs`
- `geometry_msgs`
- `turtlesim`

To ensure all dependencies are met:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🪪 License

This project is licensed under the MIT License.

---

## 👨‍💻 Author

**Sankalp Raj**  

---

## 🧭 Future Improvements

- Add multiple observation rounds to refine beliefs
- Add visualization for real-time belief changes
- Introduce agent movement and dynamic behavior
- Enable belief sharing between agents

---
