# Reactive Follow the Gap Controller (Part 2) - Zandvoort Circuit

**Author:** Kristopher Andrés Pacheco López  
**Course:** Autonomous Vehicles (II PAO 2026)  
**Environment:** F1Tenth Gym ROS (ROS 2 Humble)  
**Assigned Track:** Zandvoort (Group 4)

---

## 1. Project Description

Implementation of a reactive **Follow the Gap (FTG)** controller with a disparity extender in Python on **ROS 2 Humble**, adapted to complete both tests of Part 2 of the project on the Zandvoort circuit:

1. **Test 1 (Static Obstacles):** Navigate the modified map containing **5 static obstacles** distributed along the track and complete **10 consecutive collision-free laps**.
2. **Test 2 (Dynamic Obstacle and Overtaking):** Add a **second vehicle starting 5 meters ahead** of the main robot using the same base controller, **overtake** it during the race, and complete **10 consecutive collision-free laps**.

Additionally, the node integrates an automatic lap counter, real-time lap timer, lap distance calculation, and **RViz** visualization for both the 3D HUD and the vehicle trajectory reset on every lap.

---

## 2. Technical Approach

The controller (`ftg_node.py`) is parameterized to run on both the main vehicle (`ego_racecar`) and the opponent vehicle (`opp_racecar`), executing the following pipeline on every LiDAR scan:

* **Field-of-View Cropping and Disparity Extender:** The frontal LiDAR scan is cropped to a $\pm 74^\circ$ field of view (wide enough to spot corner exits without looking backward, preventing U-turns). After applying a 3-point moving average filter, any depth jump greater than `0.24 m` projects a `0.21 m` lateral margin onto the closer reading to clear inner wall apexes, static blocks, and the opponent's chassis.
* **Compact Safety Bubble and Lane Selection:** When an obstacle is detected within `1.9 m`, a `0.26 m` safety bubble zeroes out rays around the closest point. Among the remaining free-space segments, the controller selects the gap maximizing the depth-to-width score ($\text{depth} \cdot \sqrt{\text{width}}$) to pick the most viable overtaking lane.
* **Smoothed PD Steering and Side-Cut Guard:** Steering commands are computed via an adaptive PD controller with a per-cycle rate limiter (`±0.095 rad`) to avoid sudden swerves. If side rays ($32^\circ$ to $72^\circ$) detect a vehicle alongside within `0.48 m`, steering toward that side is restricted until the overtake is complete.
* **Velocity Profile with Straight-Line Overtaking Mode:** A longitudinal frontal corridor (`±0.26 m`) handles braking before tight hairpins or close obstacles. When the opponent blocks the front on a straight while an adjacent lane is clear (`> 5.5 m`), overtaking mode lowers the steering speed penalty so the ego vehicle keeps momentum and passes cleanly.
* **Telemetry and Per-Lap Trajectory in RViz:** Using `/ego_racecar/odom`, the node detects finish-line crossings with spatial hysteresis (`2.0 m`) and a minimum debounce lap time (`6.5 s`), integrates Euclidean distance traveled per lap, and publishes the racing line to `/ftg_trajectory_marker` and `/ftg_path`. At the start of each new lap, the trajectory resets and switches color in sync with the 3D HUD text (`/ftg_hud_marker`).

---

## 3. Project Structure

```text
ftg_zandvoort/
├── ftg_zandvoort/
│   ├── __init__.py
│   └── ftg_node.py           # Multi-agent FTG controller, telemetry, and RViz markers
├── maps/
│   ├── Zandvoort_map.png     # Original Zandvoort map (Test 2)
│   ├── Zandvoort_map.yaml    # Original map metadata (0.05109 m/px)
│   ├── Zandvoort_obs.png     # Modified map with 5 static obstacles (Test 1)
│   └── Zandvoort_obs.yaml    # Obstacles map metadata
├── resource/
│   └── ftg_zandvoort
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

### Main Components in `ftg_node.py`
* **Perception and Control (`apply_disparity_extender`, `apply_safety_bubble`, `find_max_gap`, `find_best_point`, `scan_callback`):** Process `/scan` (or `/opp_scan`) and publish Ackermann commands to `/drive` (or `/opp_drive`).
* **ROS 2 Parameters (`is_opponent`, `v_max`, `v_min`, `center_bias`, `lane_bias`):** Allow reusing the exact same node for both vehicles from the command line, tuning speed and lane preference without duplicating scripts.
* **Telemetry and Visualization (`odom_callback`, `reset_lap_trajectory`, `append_trajectory_point`, `publish_hud_marker`, `publish_target_marker`, `print_dashboard`):** Handle lap counting, timing, per-lap distance tracking, and RViz markers.

---

## 4. Static Obstacles Map and 2-Agent Configuration

* **5 Static Obstacles (`Zandvoort_obs.png`):** Created from a copy of the official Zandvoort map (`resolution: 0.05109 m/px`, `origin: [-19.7047, -57.9025, 0.0]`) by placing 5 solid black (`#000000`) `10x10` pixel blocks (`~0.51 m x 0.51 m`) across different track sectors while leaving passable room on one side.
* **Second Vehicle Pose (5 m Ahead):** With the ego vehicle starting at `(sx: 0.0, sy: 0.0, stheta: 1.22 rad)`, placing the opponent 5 meters ahead along the main straight gives:
  * `sx1 = 5 * cos(1.22) = 1.72`
  * `sy1 = 5 * sin(1.22) = 4.70`
  * `stheta1 = 1.22`

---

## 5. Installation and Execution Instructions

### Step 1: Copy Maps and Build the Package
```bash
cp maps/Zandvoort_* ~/ros2_ws/src/f1tenth_gym_ros/maps/
cd ~/ros2_ws
colcon build --packages-select f1tenth_gym_ros ftg_zandvoort --symlink-install
source install/setup.bash
```

### Step 2: Run Test 1 (5 Static Obstacles — 10 Laps)
Update `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml`:
```yaml
map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/maps/Zandvoort_obs'
map_img_ext: '.png'
num_agent: 1
sx: 0.0
sy: 0.0
stheta: 1.22
```

* **Terminal 1 (Simulator and RViz):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ```
* **Terminal 2 (Ego Controller):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run ftg_zandvoort ftg_node
  ```

### Step 3: Run Test 2 (2 Robots at 5 m and Overtaking — 10 Laps)
Update `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml`:
```yaml
map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/


## VIDEO

https://youtu.be/WKZRyJAe5v0

## RECORDED DATA
## Rosbag Recording and Unified Telemetry CSV

During the combined scenario (5 static obstacles + 2 vehicles overtaking), the `/scan`, `/ego_racecar/odom`, and `/drive` topics were recorded using `ros2 bag record` and synchronized into a single `.csv` file with `scripts/bag_to_csv.py`. Each row combines timestamp, vehicle position (`pos_x`, `pos_y`, `pos_z`, `yaw_rad`), linear and angular velocities (`vel_linear_x`, `vel_linear_y`, `vel_angular_z`), Ackermann drive commands (`drive_speed`, `drive_steering_angle`), and LiDAR scan ranges.

* **Rosbag + Unified CSV (Google Drive):** `[https://drive.google.com/file/d/1AdhiACzEYLgwOrMGtZ2QGZklbBXNh3zn/view?usp=sharing]`
