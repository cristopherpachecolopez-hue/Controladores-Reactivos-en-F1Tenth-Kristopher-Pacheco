# 🏎️ Reactive Follow the Gap (FTG) Controller with Disparity Extender — Zandvoort Circuit (F1Tenth)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Simulator](https://img.shields.io/badge/F1Tenth-Gym_ROS-E10600?style=for-the-badge)
![Status](https://img.shields.io/badge/Validation-10_Collision--Free_Laps-2EA44F?style=for-the-badge)

* **Author:** Kristopher Andrés Pacheco López
* **Course:** Autonomous Vehicles — II PAO 2026
* **Assessment:** First Midterm Project (Part 1) — Reactive Controllers Competition
* **Assigned Track:** Zandvoort (Group 4)

---

## 📑 Table of Contents
1. [Project Overview and Objectives](#-1-project-overview-and-objectives)
2. [Geometric Analysis of the Zandvoort Circuit](#-2-geometric-analysis-of-the-zandvoort-circuit)
3. [ROS 2 System Architecture](#-3-ros-2-system-architecture)
4. [Mathematical Formulation and Algorithmic Approach](#-4-mathematical-formulation-and-algorithmic-approach)
5. [Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory](#-5-telemetry-system-lap-counter-lap-timer-distance-tracker-and-3d-rviz-trajectory)
6. [Detailed Code Structure](#-6-detailed-code-structure)
7. [Hyperparameter Table and Tuning Strategy](#-7-hyperparameter-table-and-tuning-strategy)
8. [Installation, Configuration, and Execution Instructions](#-8-installation-configuration-and-execution-instructions)
9. [Experimental Results, Lap Distance, and Video Evidence](#-9-experimental-results-lap-distance-and-video-evidence)

---

## 🎯 1. Project Overview and Objectives

This repository presents the design, implementation, and experimental validation of an **autonomous reactive navigation controller** based on an enhanced **Follow the Gap (FTG)** algorithm featuring a **Disparity Extender**, an **adaptive Proportional-Derivative (PD) lateral control law**, and an **exponential longitudinal velocity profile**, developed in Python on **ROS 2 Humble** for the official **F1Tenth Gym ROS** simulator.

The controller was engineered to fulfill both on-track performance benchmarks and real-time telemetry and visualization requirements:
1. **Endurance and Reliability Test:** Complete **10 consecutive collision-free laps** around the assigned circuit.
2. **Fastest Lap Test (*Time Attack*):** Achieve the **minimum single-lap time** within the 10-lap collision-free run.
3. **Integrated Telemetry and Trajectory Visualization:** Automatic lap counting, real-time lap timer, Euclidean integration of **distance traveled per lap (m)**, 3D floating HUD telemetry (`visualization_msgs/Marker`), and continuous **vehicle trajectory rendering (`LINE_STRIP` / `nav_msgs/Path`)** in **RViz**.

---

## 🗺️ 2. Geometric Analysis of the Zandvoort Circuit

The assigned track is the **Zandvoort** circuit (`Zandvoort_map.png` / `Zandvoort_map.yaml`), which combines a high-speed main straight with tight $180^\circ$ hairpins and fast technical chicanes that require balancing early braking with clean apex tracking:

| Map Parameter | Value / Specification | Impact on Controller Design |
| :--- | :--- | :--- |
| **Spatial Resolution** | `0.05109 m/pixel` | Enables sub-decimeter wall boundary and corner edge detection. |
| **Coordinate Origin** | `[-19.7047, -57.9025, 0.0]` | Places the initial pose `(0.0, 0.0)` directly on the left main straight. |
| **Initial Heading (`stheta`)** | `1.22 rad` ($\approx 69.9^\circ$) | Aligns the vehicle longitudinal axis with the main straight (clockwise direction). |
| **Sector 1 (Main Straight)** | High-speed long straight | Allows reaching peak speed (`10.2 m/s`) using adaptive steering gain attenuation. |
| **Sector 2 ($180^\circ$ Hairpins)** | Tight minimum-radius turns | Requires a wide FOV ($\pm 88^\circ$) and predictive braking at `3.6 m` from front walls. |
| **Sector 3 (Fast Chicanes)** | Linked left-right transitions | The disparity extender prevents the chassis from clipping inner wall apexes. |

---

## 🏗️ 3. ROS 2 System Architecture

The controller operates inside a decoupled publish/subscribe **ROS 2** architecture where the node `ftg_zandvoort_kristopher` processes 2D LiDAR scans at high frequency ($\approx 40\text{ Hz}$) while concurrently updating odometry metrics and 3D RViz markers:

```text
+-----------------------------------------+
|         F1Tenth Gym ROS Bridge          |
|     (Physics Simulator + 2D Map)        |
+-----------+-------------------+---------+
            |                   |
            | /scan             | /ego_racecar/odom
            | (LaserScan)       | (Odometry)
            v                   v
+-----------------------------------------+
|     Node: ftg_zandvoort_kristopher      |
|     (ftg_zandvoort/ftg_node.py)         |
|                                         |
|  1. LiDAR Filtering + Disparity Ext.    |
|  2. Dynamic Safety Bubble               |
|  3. Max Gap & Hybrid Target Selection   |
|  4. Adaptive PD Steering + Speed Law    |
|  5. Lap Counter, Timer, Dist & Path     |
+-----+----------+-----------+----------+--+
      |          |           |          |
      | /drive   | /ftg_hud  | /ftg_... | /ftg_path &
      | (Ack.)   | _marker   | target   | /ftg_trajectory_marker
      v          v           v          v
+----------+  +----------------------------+
| Ego      |  |           RViz 2           |
| Racecar  |  | (3D HUD + Trajectory Line) |
+----------+  +----------------------------+
```

### ROS 2 Communication Interfaces (Topics)

| Topic | Message Type | Direction | System Role |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribed | Receives the 2D LiDAR range array ($1080$ beams). |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscribed | Provides $(x, y)$ pose for lap counting, lap timing, distance integration, and path drawing. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Published | Sends steering angle (`steering_angle`) and longitudinal velocity (`speed`) commands. |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D floating HUD above the vehicle displaying lap, time, lap distance, and speed. |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D green sphere at the selected target point inside the navigable gap. |
| `/ftg_trajectory_marker` | `visualization_msgs/msg/Marker` | Published | Draws a continuous 3D orange line (`LINE_STRIP`) showing the trajectory driven on the circuit. |
| `/ftg_path` | `nav_msgs/msg/Path` | Published | Publishes the historical pose sequence compatible with the native RViz `Path` display. |

---

## 🧮 4. Mathematical Formulation and Algorithmic Approach

Unlike a baseline *Follow the Gap* implementation—which frequently clips inner corners on hairpins or oscillates on high-speed straights—this controller executes a **7-stage pipeline** on every LiDAR scan:

### Stage 1: Field-of-View (FOV) Cropping and Spatial Smoothing
The raw LiDAR scan is cropped to retain only the frontal sector $[-\theta_{\text{fov}}, +\theta_{\text{fov}}]$ with $\theta_{\text{fov}} = 88^\circ$, discarding rear beams. Non-finite values (`NaN`, $\pm\infty$) are clamped to the maximum horizon $R_{\max} = 13.0\text{ m}$, and a 3-point moving-average convolution filter is applied:

$$r_{\text{smooth}}[i] = \frac{1}{3} \sum_{m=-1}^{1} r[i + m]$$

### Stage 2: Disparity Extender
When approaching corners or chicanes in Zandvoort, sharp depth discontinuities occur between a beam hitting an inner wall edge ($d_{\text{near}}$) and the adjacent beam looking into the open turn. Whenever $\vert{}r[i+1] - r[i]\vert{} > \delta_{\text{disp}}$ ($\delta_{\text{disp}} = 0.30\text{ m}$), the vehicle's virtual half-width ($w_{\text{half}} = 0.24\text{ m}$) is projected onto the closer reading to mask unsafe rays:

$$\Delta\theta = \arctan\left(\frac{w_{\text{half}}}{d_{\text{near}}}\right), \qquad N_{\text{samples}} = \left\lceil \frac{\Delta\theta}{\Delta\alpha} \right\rceil$$

where $\Delta\alpha$ is the LiDAR angular resolution (`angle_increment`).

### Stage 3: Dynamic Safety Bubble
If the closest obstacle lies within a critical proximity threshold ($d_{\min} \le 2.8\text{ m}$), its index $i_{\min}$ is identified and all rays within a safety radius $R_{\text{bubble}} = 0.34\text{ m}$ are zeroed out ($0.0\text{ m}$):

$$N_{\text{bubble}} = \left\lceil \frac{\arctan(R_{\text{bubble}} / \max(d_{\min}, 0.1))}{\Delta\alpha} \right\rceil$$

### Stage 4: Adaptive Threshold Max-Gap Detection
To avoid selecting narrow false gaps, a dynamic free-space threshold is computed relative to the current maximum range:

$$T_{\text{gap}} = \max\left(1.6,\; 0.38 \cdot \max_i(r_{\text{proc}}[i])\right)$$

A boolean mask $M[i] = (r_{\text{proc}}[i] > T_{\text{gap}})$ is generated, and the longest contiguous interval $[i_{\text{start}}, i_{\text{end}}]$ is extracted.

### Stage 5: Hybrid Weighted Target Point Selection (*Best Point*)
Aiming purely at the deepest ray in the gap causes early turn-in and inner wall collisions in $180^\circ$ hairpins, whereas aiming strictly at the gap midpoint sacrifices cornering speed. A weighted hybrid index combines both criteria:

$$i_{\text{best}} = \left\lfloor 0.68 \cdot i_{\text{deep}} + 0.32 \cdot i_{\text{center}} \right\rfloor$$

where $i_{\text{deep}}$ is the mean index of rays reaching at least $94\%$ of the maximum depth inside the gap, and $i_{\text{center}} = \lfloor (i_{\text{start}} + i_{\text{end}})/2 \rfloor$. The target steering direction is $\theta_{\text{target}} = \text{angles}[i_{\text{best}}]$.

### Stage 6: Adaptive PD Lateral Control Law
The Ackermann steering angle $\delta$ is computed via a Proportional-Derivative controller that automatically reduces its proportional gain on long straights ($d_{\text{front}} > 6.5\text{ m}$) to eliminate high-speed wobble above $10\text{ m/s}$:

$$K_{p,\text{dyn}} = \begin{cases} 0.72 \cdot K_{p,\text{base}} & \text{if } d_{\text{front}} > 6.5\text{ m} \\ K_{p,\text{base}} & \text{otherwise} \end{cases}$$

$$\delta = \text{clip}\Big(K_{p,\text{dyn}} \cdot \theta_{\text{target}} + K_d \cdot (\theta_{\text{target}} - \theta_{\text{prev}}),\; -0.4189,\; +0.4189\Big)$$

### Stage 7: Exponential Velocity Profile and Predictive Corner Braking
The longitudinal velocity $v$ scales exponentially with steering angle magnitude $\vert{}\delta\vert{}$ and lookahead clearance, supplemented by predictive braking when approaching a hairpin wall ($d_{\text{front}} < 3.6\text{ m}$):

$$v = v_{\min} + (v_{\max} - v_{\min}) \cdot e^{-3.2 \vert{}\delta\vert{}} \cdot \min\left(1.0,\; \frac{\max(d_{\text{front}}, 0.85\,d_{\text{target}})}{7.5}\right)$$

$$v \leftarrow \min\left(v,\; v_{\min} + 1.1 \cdot \frac{d_{\text{front}}}{3.6}\right) \quad \text{if } d_{\text{front}} < 3.6\text{ m}$$

---

## ⏱️ 5. Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory

Synchronized with `/ego_racecar/odom`, the telemetry subsystem provides full observability during execution:

* **Automatic Finish-Line Detection with Spatial Hysteresis:** Upon receiving the first odometry message, the initial coordinates $(x_0, y_0)$ are stored as the finish line. A new lap is registered only after the vehicle leaves the start zone ($d > 4.4\text{ m}$), completes the circuit ($t \ge 5.5\text{ s}$), and re-enters the finish radius ($d \le 2.0\text{ m}$).
* **Euclidean Lap Distance Integration:** On every odometry update $(x_k, y_k)$, the incremental displacement $\Delta s_k = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$ is accumulated to compute the exact distance traveled per lap:

$$D_{\text{lap}} = \sum_{k=1}^{K} \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$

* **Real-Time Trajectory Rendering in RViz:** Every $0.25\text{ m}$ of travel, the vehicle's $(x, y)$ coordinates in the `map` frame are appended to `/ftg_trajectory_marker` (`Marker.LINE_STRIP`) and `/ftg_path` (`nav_msgs/Path`), drawing the complete racing line followed by the vehicle around Zandvoort.

---

## 📂 6. Detailed Code Structure

The ROS 2 package `ftg_zandvoort` is organized inside the `FollowTheGapZandvoort(Node)` class in `ftg_zandvoort/ftg_node.py`:

| Method / Function | Technical Responsibility | Inputs / Outputs |
| :--- | :--- | :--- |
| `__init__(self)` | Initializes navigation hyperparameters, lap/timer/distance state variables, subscribers (`/scan`, `/ego_racecar/odom`), and publishers (`/drive`, `/ftg_hud_marker`, `/ftg_target_marker`, `/ftg_trajectory_marker`, `/ftg_path`). | Node initialization. |
| `odom_callback(self, msg)` | Integrates traveled distance per lap, runs the lap-counting finite-state machine, updates lap times, and triggers 3D HUD and trajectory updates. | Input: `Odometry` $\rightarrow$ Updates race metrics. |
| `append_trajectory_point(...)` | Appends the current vehicle pose to both `nav_msgs/Path` and the 3D `LINE_STRIP` marker to visualize the driven trajectory in RViz. | Input: `x, y, odom_msg` $\rightarrow$ Publishes path & line strip. |
| `publish_hud_marker(self, ...)` | Publishes a `TEXT_VIEW_FACING` 3D marker above the car showing lap count, live timer, distance traveled, best lap, and speed. | Input: `current_lap_time` $\rightarrow$ Publishes `/ftg_hud_marker`. |
| `publish_target_marker(self, ...)` | Converts polar target coordinates $(d_{\text{target}}, \theta_{\text{target}})$ to Cartesian $(x, y)$ to render the green target sphere. | Input: `target_dist, target_angle` $\rightarrow$ Publishes `/ftg_target_marker`. |
| `print_dashboard(self, ...)` | Prints live telemetry in the terminal and outputs a summary banner with lap time and lap distance upon each finish-line crossing. | Standard console output (`stdout`). |
| `preprocess_lidar(self, ...)` | Cleans invalid LiDAR readings, applies moving-average smoothing, and executes the disparity extender. | Input: `ranges, angle_inc` $\rightarrow$ Output: filtered `np.ndarray`. |
| `apply_safety_bubble(self, ...)` | Locates the nearest obstacle and zeroes out beams within the safety radius. | Input: `proc_ranges, angle_inc` $\rightarrow$ Output: masked `np.ndarray`. |
| `find_max_gap(self, ...)` | Applies dynamic thresholding and returns the `[start_i, end_i]` indices of the widest contiguous free gap. | Input: `free_space_ranges` $\rightarrow$ Output: `(start_i, end_i)`. |
| `find_best_point(self, ...)` | Computes the optimal target index by weighting deepest rays ($68\%$) and gap center ($32\%$). | Input: `start_i, end_i, ranges` $\rightarrow$ Output: `best_i`. |
| `scan_callback(self, data)` | Main reactive control loop: orchestrates LiDAR perception, adaptive PD steering, velocity profiling, and Ackermann publishing. | Input: `LaserScan` $\rightarrow$ Publishes `/drive`. |
| `main(args=None)` | Initializes `rclpy`, spins the node, and guarantees a zero-velocity stop command (`0.0 m/s`) upon `Ctrl+C` shutdown. | `console_scripts` entry point. |

---

## ⚙️ 7. Hyperparameter Table and Tuning Strategy

| Parameter | Tuned Value | Unit | Description and Effect on Zandvoort |
| :--- | :---: | :---: | :--- |
| `fov_deg` | `88.0` | degrees ($\pm^\circ$) | Frontal field of view; wide enough to see $180^\circ$ hairpin exits without losing straight-line focus. |
| `max_lidar_range` | `13.0` | $\text{m}$ | Maximum LiDAR lookahead horizon used on long straights. |
| `bubble_radius` | `0.34` | $\text{m}$ | Safety exclusion radius around the closest obstacle point. |
| `disparity_threshold` | `0.30` | $\text{m}$ | Minimum range jump between adjacent rays to trigger the disparity extender. |
| `car_half_width` | `0.24` | $\text{m}$ | Virtual half-width projected laterally across wall corners. |
| `v_max` | `10.2` | $\text{m/s}$ | Peak longitudinal speed on the main straight. |
| `v_min` | `2.65` | $\text{m/s}$ | Minimum cornering speed in tight hairpins to prevent understeer. |
| `kp_base` | `0.72` | — | Base proportional gain of the steering PD controller. |
| `kd_steer` | `0.16` | — | Derivative damping gain to suppress rapid steering oscillations. |

---

## 🚀 8. Installation, Configuration, and Execution Instructions

### Prerequisites
* **OS & Middleware:** Ubuntu 22.04 LTS with **ROS 2 Humble**.
* **Simulator:** Official **F1Tenth Gym ROS** installed inside `~/ros2_ws`.

### Step 1: Copy the Zandvoort Map and Configure `sim.yaml`
1. Copy `maps/Zandvoort_map.png` and `maps/Zandvoort_map.yaml` into the simulator's `maps/` directory:
   ```bash
   cp maps/Zandvoort_map.* ~/ros2_ws/src/f1tenth_gym_ros/maps/
   ```
2. Update `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml` with the Zandvoort map path and initial heading:
   ```yaml
   map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/maps/Zandvoort_map'
   map_img_ext: '.png'
   num_agent: 1
   sx: 0.0
   sy: 0.0
   stheta: 1.22
   ```

### Step 2: Clone and Build the Package
```bash
cd ~/ros2_ws/src
git clone [https://github.com/](https://github.com/)<YOUR_USERNAME>/ftg_zandvoort.git
cd ~/ros2_ws
colcon build --packages-select f1tenth_gym_ros ftg_zandvoort --symlink-install
source install/setup.bash
```

### Step 3: Launch the Simulator, RViz Trajectory, and Controller
1. **Terminal 1 — Launch the F1Tenth simulator and RViz:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. **Enable Camera Follow, 3D HUD, and Trajectory Visualization in RViz:**
   * In the right **Views** panel, set `Target Frame` to `ego_racecar/base_link` so the camera follows the vehicle.
   * Click **Add -> By topic** and add:
     * `/ftg_hud_marker` (`Marker`): 3D floating telemetry (laps, lap timer, lap distance, speed).
     * `/ftg_target_marker` (`Marker`): 3D green target sphere.
     * `/ftg_trajectory_marker` (`Marker`): Continuous 3D orange line showing the trajectory traveled around the circuit (or enable the `Path` display on `/ftg_path`).
3. **Terminal 2 — Run the Follow the Gap controller:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run ftg_zandvoort ftg_node
   ```

---

## 📊 9. Experimental Results, Lap Distance, and Video Evidence

### Performance Summary on Zandvoort (10-Lap Run)

| Evaluation Metric | Recorded Value | Status |
| :--- | :---: | :---: |
| **Test 1 — Consecutive Collision-Free Laps** | `10 / 10 laps` | ✅ Completed |
| **Test 2 — Fastest Lap Time** | `41.229 s` | ✅ Completed |
| **Distance Traveled per Lap (Racing Line)** | `~376.81 m / lap` *(measured via `/ego_racecar/odom`)* | ✅ Verified |
| **Peak Velocity on Main Straight** | `10.2 m/s` | ✅ Verified |
| **RViz Trajectory Visualization** | Active via `/ftg_trajectory_marker` (`LINE_STRIP`) & `/ftg_path` | ✅ Verified |

### 🎥 Video Evidence
* **Demonstration Video (Vehicle navigating Zandvoort + Visible Lap Counter + Live Lap Timer + Distance & RViz Trajectory):** `[https://youtu.be/idIuEAnc6hQ]`
