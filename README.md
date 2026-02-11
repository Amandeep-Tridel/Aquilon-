# Aquilon - MOOS-ROS 2 Bridge & USV Simulation

Bidirectional bridge between MOOS-IvP and ROS 2 Jazzy, with a fake GPS obstacle avoidance simulation for USV (Unmanned Surface Vehicle) demos.

## Architecture

```
pMarineViewer <-- MOOSDB <-- moos_ros2_bridge --> ROS 2 Topics
                                  ^
                                  |
                        fake_gps_with_obstacles.py
                        (60 Hz simulation node)
```

- **moos_ros2_bridge** - C++ ROS 2 node embedding a `CMOOSCommClient` for bidirectional variable/topic forwarding
- **fake_gps_with_obstacles.py** - Python ROS 2 node simulating USV navigation with 4 moving obstacles and advanced avoidance
- **pMarineViewer** - MOOS-IvP GUI displaying the USV trail and obstacle animation

## Prerequisites

- Ubuntu 24.04 (WSL2 supported with WSLg)
- ROS 2 Jazzy Jalisco
- MOOS-IvP (built from source at `~/moos-ivp`)

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/moos_ros2_bridge_ws
colcon build --packages-select moos_ros2_bridge
source install/setup.bash
```

## Quick Start

### 1. Start MOOS mission (MOOSDB + pMarineViewer + pNodeReporter)

```bash
cd ~/moos-ivp/ivp/missions/gps_bridge_demo
pAntler gps_demo.moos
```

### 2. Launch the bridge

```bash
ros2 run moos_ros2_bridge moos_ros2_bridge_node \
  --ros-args --params-file src/fake_gps/bridge_gps_demo.yaml
```

### 3. Run the obstacle avoidance simulation

```bash
python3 src/fake_gps/fake_gps_with_obstacles.py
```

The USV will navigate a lawnmower survey pattern while avoiding 4 moving obstacles visible in pMarineViewer.

## Components

### MOOS-ROS 2 Bridge (`src/moos_ros2_bridge/`)

Configurable bidirectional bridge supporting `double` (Float64) and `string` (String) types. Variable mappings defined in YAML:

```yaml
moos_to_ros:
  - "NAV_X:/moos/nav_x:double"
ros_to_moos:
  - "/gps/x:NAV_X:double"
  - "/view_marker:VIEW_POLYGON:string"
```

Key design: Uses `CMOOSCommClient` (non-blocking) instead of `CMOOSApp` to avoid event loop conflicts with ROS 2. A wall timer polls MOOS mail, subscriber callbacks call `Notify()`. Thread-safe via `std::mutex`.

### Fake GPS + Obstacle Avoidance (`src/fake_gps/`)

| File | Description |
|------|-------------|
| `fake_gps_with_obstacles.py` | 60 Hz simulation with PID control and moving obstacles |
| `bridge_gps_demo.yaml` | Bridge config mapping GPS/obstacle topics to MOOS variables |
| `gps_demo.moos` | MOOS mission file for pMarineViewer visualization |

**Avoidance features:**
- PID heading controller (proportional + integral + derivative with anti-windup)
- PID speed controller with rate-limited acceleration/deceleration
- 6 speed states with active collision avoidance (never stops — always flees):

| State | Speed | Trigger | Behavior |
|-------|-------|---------|----------|
| FLEE | 4.5 m/s | Obstacle < 6m | Burst acceleration (6x), double turn rate, max speed escape |
| EVADE | up to 3.5 m/s | Obstacle < 12m & closing | Speed up proportional to urgency |
| CREEP | 0.6 m/s | Obstacle < 12m | Minimal forward, steering around |
| SLOW | 1.5 m/s | Obstacle < 20m | Cautious approach, extra caution if closing |
| CRUISE | 2.5 m/s | Obstacle < 30m | Normal navigation with avoidance steering |
| FAST | 3.5 m/s | All clear > 5s | Speed boost on open water |

- Closest Point of Approach (CPA) prediction using relative velocity
- Velocity obstacle model: predicts future collision risk, not just current distance
- Speed boost when all-clear for 5+ seconds

**4 obstacle patterns:**
- Horizontal patrol (red, r=4m)
- Circular orbit (orange, r=5m)
- Vertical patrol (red, r=3m)
- Diagonal wander (orange, r=6m)

### Lawnmower Mission (`src/missions/s1_lawnmower/`)

Standalone MOOS-IvP mission with a 6-lane lawnmower survey pattern (200m x 150m, 30m lane spacing) using the Helm behavior system.

## ROS 2 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/gps/x`, `/gps/y` | Float64 | ROS->MOOS | USV position (meters, local frame) |
| `/gps/heading` | Float64 | ROS->MOOS | Heading (0-360 degrees) |
| `/gps/speed` | Float64 | ROS->MOOS | Actual speed (m/s) |
| `/gps/lat`, `/gps/lon` | Float64 | ROS->MOOS | GPS coordinates |
| `/obstacle/alert` | String | ROS->MOOS | Detection/avoidance alerts |
| `/obstacle/range` | Float64 | ROS->MOOS | Nearest obstacle distance (m) |
| `/obstacle/count` | Float64 | ROS->MOOS | Number of obstacles in detection range |
| `/view_marker` | String | ROS->MOOS | VIEW_POLYGON markers for pMarineViewer |

## License

MIT
