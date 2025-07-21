You're welcome! Here's a clean, informative, and ROS-friendly `README.md` for your GitHub repository:

---

```markdown
# 🐢 Bayesian Belief Update with Turtlesim (ROS)

This project demonstrates a simple **Bayesian filter** for probabilistic state estimation using ROS and Turtlesim. A main robot navigates through a 2D grid and attempts to identify whether randomly placed agents (other turtles) are **alive** or **dead** based on **noisy observations**. The robot updates its belief using **Bayes' Rule** and logs its reasoning.

---

## 📌 Features

- 🧭 **Main robot** (`turtle1`) navigates in a 2D grid (9×9).
- 🧍‍♂️ **4 agents** (`turtle2` to `turtle5`) are spawned at random positions.
- 🎯 Each agent has a hidden binary state: `alive` (1) or `dead` (0).
- 🔍 Robot senses each agent with noisy observations.
- 🧠 **Bayesian belief update** after each observation.
- 📢 Beliefs are published on `/agent/belief`.
- 📁 Belief history logged in a `.csv` file.

---

## 📁 Project Structure

```

catkin\_ws/
└── src/
└── door\_state\_estimation/
├── scripts/
│   ├── belief\_main\_robot.py
│   └── spawn\_agents.py
├── launch/
│   └── bayes\_filter\_demo.launch
├── log/
│   └── belief\_log.csv
├── CMakeLists.txt
└── package.xml

````

---

## 🚀 How to Run

### 1. Setup

Make sure you have a working ROS installation (`melodic`, `noetic`, etc.).

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
````

### 2. Launch the Simulation

```bash
roslaunch door_state_estimation bayes_filter_demo.launch
```

This will:

* Launch `turtlesim_node`
* Spawn `turtle1` and 4 agents
* Start the main robot
* Begin the belief update process

---

## 📊 Belief Update Equation

We use **Bayes' Rule** for each update:

$$
P(H|E) = \frac{P(E|H) \cdot P(H)}{P(E|H) \cdot P(H) + P(E|\neg H) \cdot (1 - P(H))}
$$

Where:

* $P(H)$: prior belief (default 0.5)
* $P(E|H)$: sensor accuracy (e.g. 0.8)
* $P(H|E)$: updated belief

---

## 📈 Example Log (CSV)

Each row in `log/belief_log.csv` records:

| Time  | Agent   | TrueState | Observed | Prior | UpdatedBelief |
| ----- | ------- | --------- | -------- | ----- | ------------- |
| 23.51 | turtle2 | 1         | 1        | 0.5   | 0.80          |
| 31.27 | turtle3 | 0         | 1        | 0.5   | 0.28          |

---

## 📡 ROS Topics

* `/turtle1/cmd_vel` – Movement of main robot
* `/agent/belief` – Belief updates about each agent (custom message: string + float)

---

## 📚 Dependencies

* ROS (`melodic`, `noetic`, etc.)
* `turtlesim`
* Python 3 (`rospy`)

---

## 🛠️ To Do / Future Work

* [ ] Multi-step trajectory belief refinement
* [ ] Add visualization for belief values
* [ ] Implement belief sharing among multiple robots
* [ ] Add agent movement and dynamic updates
* [ ] Add service to reset and re-randomize agent states

---

## 👨‍💻 Author

**Sankalp Raj**
Feel free to contribute, raise issues, or suggest features!

---

## 📜 License

This project is released under the MIT License.

---

