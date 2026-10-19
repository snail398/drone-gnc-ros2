# drone-gnc-ros2

Guidance, navigation and control for a quadrotor, written in C++ on top of ROS 2 and PX4.
The vehicle flies in Gazebo through PX4 SITL; the goal is to replace PX4's own position,
attitude and rate controllers and its EKF with my own, one layer at a time, and measure
how they compare.

Work in progress.

## Layout

| Package | What it contains |
|---|---|
| `gnc_core` | Algorithms in plain C++ (no ROS): controllers, mission logic, later estimators and planners. Unit-tested with gtest. |
| `gnc_ros` | Thin ROS 2 nodes that feed PX4 data into `gnc_core` and publish setpoints back. |

Keeping the algorithms out of ROS means they can be tested in milliseconds, without a simulator.

## Roadmap

- [x] Simulation environment: PX4 v1.17 SITL, Gazebo Harmonic, ROS 2 Jazzy, uXRCE-DDS
- [x] Repository skeleton, CI, PID controller with anti-windup
- [x] Offboard mission state machine: arm, take off, fly waypoints, land (PX4 controllers still in the loop)
- [ ] Own position/velocity controller (attitude + thrust setpoints)
- [ ] Own attitude controller (body rate setpoints)
- [ ] Own EKF: IMU prediction, GPS and barometer updates, compared against PX4 EKF2
- [ ] Flight log analysis tooling (ULog / rosbag2)
- [ ] Path planning around obstacles
- [ ] Sanitizers, benchmarks and profiling in CI

## Build

Requires Ubuntu 24.04 with ROS 2 Jazzy. In a colcon workspace:

```bash
cd ~/drone_ws/src
git clone --branch release/1.17 https://github.com/PX4/px4_msgs.git
git clone https://github.com/snail398/drone-gnc-ros2.git drone_gnc
cd ~/drone_ws
colcon build --packages-up-to gnc_ros
colcon test --packages-select gnc_core && colcon test-result --verbose
```

## Run

With PX4 SITL and `MicroXRCEAgent udp4 -p 8888` running, fly a 10 x 10 m square at 5 m:

```bash
ros2 launch gnc_ros mission.launch.py
```

Another mission file can be passed with `mission:=/path/to/mission.yaml`; see
`gnc_ros/config/square_mission.yaml` for the format.

The mission logic is a plain state machine in `gnc_core` (wait for pre-flight checks,
stream setpoints, switch to offboard and arm, take off, visit waypoints, land). It aborts
and leaves the vehicle to PX4 if offboard mode or arming is lost in flight. The ROS node
only converts PX4 messages to and from it, so the whole mission is covered by unit tests
against a simulated vehicle.

To just print the vehicle position: `ros2 run gnc_ros position_monitor`.
