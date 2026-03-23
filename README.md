# rosclaw-ur-rtde-mcp

ROSClaw MCP Server for Universal Robots — using **ur_rtde** (RTDE protocol, direct TCP, no ROS2 required). **With Robotiq Gripper support** (port 63352 via UR Cap).

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Features

| Category | Tools |
|----------|-------|
| Connection | `connect_robot`, `disconnect_robot` |
| Robot Lifecycle | `get_robot_info`, `robot_power_control`, `unlock_protective_stop`, `restart_safety` |
| Motion | `move_joint`, `move_linear`, `move_joint_ik`, `stop_motion`, `servo_joint`, `speed_joint` |
| Force Control | `force_mode`, `force_mode_stop`, `zero_ft_sensor` |
| Teaching | `teach_mode`, `jog` |
| Kinematics | `get_inverse_kinematics` |
| Payload | `set_payload` |
| I/O | `set_digital_output`, `set_speed_slider`, `set_analog_output`, `get_digital_io_state` |
| State | `get_robot_state`, `get_tcp_pose`, `get_joint_positions`, `get_tcp_force` |
| **Robotiq Gripper** | `connect_gripper`, `disconnect_gripper`, `gripper_activate`, `gripper_open`, `gripper_close`, `gripper_move`, `gripper_get_status` |

**MCP Resources**: `robot://status`, `robot://connection`

## Hardware

Universal Robots UR3/UR5/UR10/UR16/UR20 (CB3 and e-Series)
Protocol: RTDE over TCP port 30004
Robotiq Gripper: TCP port 63352 (via Robotiq_grippers UR Cap)

## Quick Start

```bash
# Install dependencies
pip install ur-rtde mcp

# Run MCP server
python src/ur_rtde_mcp_server.py

# Or with uv
uv venv --python 3.11
source .venv/bin/activate
uv pip install ur-rtde mcp
python src/ur_rtde_mcp_server.py
```

## Claude Desktop Configuration

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "rosclaw-ur-rtde": {
      "command": "python",
      "args": ["/path/to/rosclaw-ur-rtde-mcp/src/ur_rtde_mcp_server.py"],
      "transportType": "stdio"
    }
  }
}
```

## LLM Usage Example — Robot Arm

```
User: Connect to the robot at 192.168.1.100, move to home position, then check TCP force.

LLM calls:
1. connect_robot("192.168.1.100")
2. robot_power_control("brake_release")   # if needed
3. move_joint([0, -1.57, 0, -1.57, 0, 0])  # home position
4. get_tcp_force()
```

## LLM Usage Example — Robotiq Gripper

```
User: Connect to the robot and gripper, then pick up an object.

LLM calls:
1. connect_robot("192.168.1.100")
2. connect_gripper()                     # Connects to same IP, port 63352
3. gripper_activate()                    # Activates + auto-calibrates
4. gripper_open()                        # Open before picking
5. move_joint([...])                     # Move above object
6. gripper_close(speed=128, force=100)   # Close with controlled force
7. move_joint([...])                     # Move to drop position
8. gripper_open()                        # Release object
```

### Gripper Position Values

| Position | Description |
|----------|-------------|
| 0 | Fully open |
| 255 | Fully closed |
| 128 | Halfway |

Use `gripper_move(position=64, speed=255, force=100)` for precise positioning.

## Safety

- All motion commands validate against UR hardware limits before sending:
  - Joint velocity: max 3.14 rad/s
  - Tool velocity: max 3.0 m/s
  - Joint acceleration: max 40 rad/s²
- `check_safe_to_move()` blocks commands during protective/emergency stop
- Protective stop can be unlocked via `unlock_protective_stop()` (after 5 s)
- Gripper auto-calibration opens/closes to detect limits (call after `activate()`)

## Architecture

```
LLM (Claude)
    │ MCP tools (semantic level)
    ▼
ur_rtde_mcp_server.py   (FastMCP, stdio)
    │
ur_rtde_bridge.py       (thread-safe wrapper, threading.Lock)
    ├───────────────────────────────────────┐
    │                                       │
ur_rtde Python bindings              robotiq_gripper.py
    │ TCP port 30004                       │ TCP port 63352
    ▼                                       ▼
UR Robot Controller (RTDE)         Robotiq Gripper (UR Cap)
```

## File Structure

```
rosclaw-ur-rtde-mcp/
├── src/
│   ├── ur_rtde_mcp_server.py   # MCP server with FastMCP (569 lines)
│   ├── ur_rtde_bridge.py       # Thread-safe bridge for RTDE/IO/Dashboard (238 lines)
│   └── robotiq_gripper.py      # Robotiq gripper direct TCP control (297 lines)
├── tests/
│   ├── test_ur_rtde_bridge.py  # Bridge tests (15 passed)
│   └── test_robotiq_gripper.py # Gripper tests (15 passed)
├── config/
│   └── mcp_config.json         # MCP client configuration
├── pyproject.toml
├── README.md
└── LICENSE
```

## Testing

```bash
# Run all tests (30 total, no hardware required)
pytest tests/ -v

# Run specific test file
pytest tests/test_robotiq_gripper.py -v
```

## Related ROSClaw Servers

- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 via ROS2/MoveIt
- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 humanoid
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU gimbal
- [rosclaw-vision-mcp](https://github.com/ros-claw/rosclaw-vision-mcp) — RealSense RGB-D
