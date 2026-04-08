# rosclaw-ur-rtde-mcp

[English](./README.md) | **中文**

ROSClaw MCP Server for Universal Robots — 使用 **ur_rtde** (RTDE 协议, 直接 TCP, 无需 ROS2)。**支持 Robotiq 夹爪** (通过 UR Cap 端口 63352)。

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System。

## SDK 信息

| 属性 | 值 |
|----------|-------|
| **SDK 名称** | ur_rtde |
| **SDK 版本** | 1.0.0+ |
| **协议** | RTDE (Real-Time Data Exchange) |
| **源码仓库** | [gitlab.com/sdurobotics/ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) |
| **文档** | [ur_rtde Docs](https://sdurobotics.gitlab.io/ur_rtde/) |
| **许可证** | MIT |
| **生成日期** | 2026-04-07 |

## 硬件规格

| 规格 | 值 |
|--------------|-------|
| **支持机器人** | UR3, UR5, UR10, UR16, UR20 |
| **控制器类型** | CB3 系列, e-Series |
| **协议** | RTDE over TCP |
| **机器人端口** | 30004 (RTDE) |
| **仪表板端口** | 29999 |
| **夹爪端口** | 63352 (通过 UR Cap 的 Robotiq) |
| **最大关节速度** | 3.14 rad/s |
| **最大工具速度** | 3.0 m/s |
| **最大关节加速度** | 40 rad/s² |

### PolyScope 兼容性

| PolyScope 版本 | 兼容性 | 说明 |
|-------------------|---------------|-------|
| 5.6.0+ | ✅ 完全支持 | 所有功能 |
| 3.x - 5.5.x | ✅ 兼容 | 部分仪表板功能返回 "N/A" |
| CB3 系列 | ✅ 兼容 | 在 PolyScope 3.15.8 上测试 |
| e-Series | ✅ 兼容 | 推荐以获得最佳性能 |

## 功能特性

| 类别 | 工具 |
|----------|-------|
| 连接 | `connect_robot`, `disconnect_robot` |
| 机器人生命周期 | `get_robot_info`, `robot_power_control`, `unlock_protective_stop`, `restart_safety` |
| 运动 | `move_joint`, `move_linear`, **`move_joint_ik`** (新), `stop_motion`, `servo_joint`, `speed_joint` |
| 力控制 | `force_mode`, `force_mode_stop`, `zero_ft_sensor` |
| 示教 | `teach_mode`, `jog` |
| 运动学 | `get_inverse_kinematics` |
| 负载 | `set_payload` |
| I/O | `set_digital_output`, `set_speed_slider`, `set_analog_output`, `get_digital_io_state` |
| 状态 | `get_robot_state`, `get_tcp_pose`, `get_joint_positions`, `get_tcp_force` |
| **Robotiq 夹爪** | `connect_gripper`, `disconnect_gripper`, `gripper_activate`, `gripper_open`, `gripper_close`, `gripper_move`, `gripper_get_status` |
| **SDK 信息** | `get_sdk_info` |

**MCP 资源**: `robot://status`, `robot://connection`, `robot://sdk_info`

## 快速开始

```bash
# 安装依赖
pip install ur-rtde mcp

# 运行 MCP server
python src/ur_rtde_mcp_server.py

# 或使用 uv
uv venv --python 3.11
source .venv/bin/activate
uv pip install ur-rtde mcp
python src/ur_rtde_mcp_server.py
```

## Claude Desktop 配置

添加到 `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "rosclaw-ur-rtde": {
      "command": "python",
      "args": ["/path/to/rosclaw-ur-rtde-mcp/src/ur_rtde_mcp_server.py"],
      "transportType": "stdio",
      "description": "UR via RTDE with Gripper Support",
      "sdk_version": "1.0.0+",
      "sdk_source": "https://gitlab.com/sdurobotics/ur_rtde"
    }
  }
}
```

## LLM 使用示例 — 机械臂

```
用户: 连接到 192.168.1.100 的机器人，移动到原位，然后检查 TCP 力。

LLM 调用:
1. connect_robot("192.168.1.100")
2. robot_power_control("brake_release")   # 如果需要
3. move_joint([0, -1.57, 0, -1.57, 0, 0])  # 原位
4. get_tcp_force()
```

## LLM 使用示例 — Robotiq 夹爪

```
用户: 连接机器人和夹爪，然后拾取一个物体。

LLM 调用:
1. connect_robot("192.168.1.100")
2. connect_gripper()                     # 连接到相同 IP, 端口 63352
3. gripper_activate()                    # 激活 + 自动校准
4. gripper_open()                        # 拾取前打开
5. move_joint([...])                     # 移动到物体上方
6. gripper_close(speed=128, force=100)   # 带力控制的关闭
7. move_joint([...])                     # 移动到放置位置
8. gripper_open()                        # 释放物体
```

### moveJ_IK — 笛卡尔姿态控制 (新)

`move_joint_ik` 工具使用逆运动学将机器人移动到笛卡尔姿态 (比 moveL 更快，比关节角度更直观)。

```python
# 移动到特定笛卡尔姿态 [x, y, z, rx, ry, rz]
move_joint_ik(
    pose=[-0.212, 0.319, 0.41, -3.1416, 0, 0],  # 米 + 旋转向量
    speed=0.5,                                    # rad/s (保守)
    acceleration=0.8                              # rad/s²
)
```

**使用场景**: 当您知道笛卡尔空间中的期望 TCP 位置但不想手动计算关节角度时。

## 安全信息

**警告:** This MCP server controls an industrial robot arm. Improper use can cause:
- 严重伤害或死亡
- 设备损坏
- 财产损失

### 安全特性

| 特性 | 描述 |
|---------|-------------|
| **关节速度** | 强制执行最大 3.14 rad/s |
| **工具速度** | 强制执行最大 3.0 m/s |
| **关节加速度** | 强制执行最大 40 rad/s² |
| **保护性停止检查** | `check_safe_to_move()` 在停止期间阻止命令 |
| **紧急停止** | 示教器上的物理紧急停止 |

### 安全级别

| 级别 | 颜色 | 描述 | 示例 |
|-------|-------|-------------|---------|
| **CRITICAL** | 🔴 | 立即危险 | 碰撞、保护性停止 |
| **HIGH** | 🟠 | 潜在损坏 | 接近关节限制 |
| **MEDIUM** | 🟡 | 需要注意 | 力模式激活 |
| **LOW** | 🟢 | 信息提示 | 状态检查 |

### 紧急程序

1. **立即停止**: 按下物理紧急停止或调用 `stop_motion()`
2. **保护性停止**: 等待 5s, 然后调用 `unlock_protective_stop()`
3. **断电**: 如果需要，使用 `robot_power_control("power_off")`

## 错误处理

### 错误代码

| 代码 | 名称 | 严重级别 | 描述 |
|------|------|----------|-------------|
| -1 | CONNECTION_FAILED | 🟠 error | RTDE 连接失败 |
| -2 | TIMEOUT | 🟠 error | 命令响应超时 |
| -3 | INVALID_PARAMETER | 🟠 error | 无效的关节角度或姿态 |
| -4 | SAFETY_VIOLATION | 🔴 critical | 超出速度/加速度限制 |
| -5 | PROTECTIVE_STOP | 🔴 critical | 机器人处于保护性停止 |
| -6 | NOT_INITIALIZED | 🟠 error | 未连接到机器人 |

### 故障排除

| 问题 | 可能原因 | 解决方案 |
|-------|---------------|----------|
| 连接失败 | 错误的 IP | 验证机器人 IP 地址 |
| 连接失败 | 网络问题 | 检查以太网线缆 |
| 命令被拒绝 | 保护性停止 | 解锁保护性停止 |
| 命令被拒绝 | 不在远程模式 | 启用远程控制模式 |
| 夹爪无响应 | UR Cap 未安装 | 安装 Robotiq_grippers cap |
| 运动缓慢 | 速度滑块 | 检查速度滑块设置 |

## 硬件要求

- **Universal Robots**: UR3, UR5, UR10, UR16, UR20 (CB3 或 e-Series)
- **通信**: RTDE over TCP 端口 30004
- **可选 - Robotiq 夹爪**: TCP 端口 63352 (通过 Robotiq_grippers UR Cap)
- **可选 - F/T 传感器**: `force_mode` 和 `zero_ft_sensor` 需要

### 夹爪位置

| 位置 | 值 | 描述 |
|----------|-------|-------------|
| 0 | 完全打开 | 手指最大张开 |
| 255 | 完全关闭 | 手指完全闭合 |
| 128 | 一半 | 50% 闭合 |

使用 `gripper_move(position=64, speed=255, force=100)` 进行精确定位。

## 架构

```
LLM (Claude)
    │ MCP tools (semantic level)
    ▼
ur_rtde_mcp_server.py   (FastMCP, stdio)
    │
ur_rtde_bridge.py       (线程安全包装器, threading.Lock)
    ├───────────────────────────────────────┐
    │                                       │
ur_rtde Python bindings              robotiq_gripper.py
    │ TCP port 30004                       │ TCP port 63352
    ▼                                       ▼
UR Robot Controller (RTDE)         Robotiq Gripper (UR Cap)
```

## 文件结构

```
rosclaw-ur-rtde-mcp/
├── src/
│   ├── ur_rtde_mcp_server.py   # 带 FastMCP 的 MCP server (~600 行)
│   ├── ur_rtde_bridge.py       # 用于 RTDE/IO/仪表板的线程安全桥接 (~270 行)
│   └── robotiq_gripper.py      # Robotiq 夹爪直接 TCP 控制 (297 行)
├── tests/
│   ├── test_ur_rtde_bridge.py  # 桥接单测 (15 通过)
│   ├── test_robotiq_gripper.py # 夹爪单测 (15 通过)
│   ├── system_test_moveJ_IK.py # moveJ_IK 功能系统测试 (需要硬件)
│   ├── full_function_test.py   # 所有 MCP 工具的完整系统测试 (需要硬件)
│   ├── diagnose_failures.py    # 故障排除诊断工具
│   └── failure_analysis_report.md # 测试失败分析
├── config/
│   └── mcp_config.json         # MCP 客户端配置
├── pyproject.toml
├── README.md
└── LICENSE
```

## 测试

### 单元测试 (无需硬件)

```bash
# 运行所有单元测试 (30 个总计, 无需硬件)
pytest tests/test_ur_rtde_bridge.py tests/test_robotiq_gripper.py -v

# 运行特定测试文件
pytest tests/test_ur_rtde_bridge.py -v
pytest tests/test_robotiq_gripper.py -v
```

### 系统测试 (需要硬件)

⚠️ **警告**: 这些测试需要真实的 UR 机器人。运行前确保安全。

```bash
# 测试 moveJ_IK 功能 (需要脚本中的 ROBOT_IP)
python tests/system_test_moveJ_IK.py

# 所有 MCP 工具的完整功能测试
python tests/full_function_test.py

# 故障排除诊断工具
python tests/diagnose_failures.py
```

### 测试报告

系统测试生成带时间戳的报告:
- `ur5_system_test_report_YYYYMMDD_HHMMSS.txt`
- `ur5_full_test_report_YYYYMMDD_HHMMSS.txt`

## 参考

- [ur_rtde GitLab](https://gitlab.com/sdurobotics/ur_rtde)
- [ur_rtde 文档](https://sdurobotics.gitlab.io/ur_rtde/)
- [Universal Robots RTDE 指南](https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/)
- [Robotiq 夹爪手册](https://robotiq.com/support/2f-85-2f-140)
- [MCP 协议](https://modelcontextprotocol.io/)

## 更新日志

### v0.2.0 (2025-03-25)
- ✨ **新**: 添加 `moveJ_IK` 方法到 `URRTDEBridge` 用于通过 IK 进行笛卡尔姿态控制
- ✨ **新**: 添加全面的硬件验证系统测试
- ✨ **新**: 添加故障排除诊断工具
- ✨ **新**: 添加 SDK 元数据和 `get_sdk_info()` 工具
- 🐛 **修复**: `get_robot_info` 现在优雅地处理 PolyScope < 5.6.0
- 📚 **文档**: 使用兼容性矩阵和新功能更新 README

### v0.1.0 (初始发布)
- 初始 MCP server 实现
- 通过 RTDE 协议支持 UR 机器人
- Robotiq 夹爪支持
- 线程安全桥接实现

## 许可证

MIT License — See [LICENSE](LICENSE)

底层 ur_rtde 库也是 MIT 许可。

## Part of ROSClaw

- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 人形机器人
- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 通过 ROS2/MoveIt
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU 云台
- [rosclaw-ur-rtde-mcp](https://github.com/ros-claw/rosclaw-ur-rtde-mcp) — UR 通过 RTDE (本仓库)

---

**Generated by ROSClaw SDK-to-MCP Transformer**

*SDK Version: ur_rtde 1.0.0+ | Protocol: RTDE (TCP) | With Robotiq Gripper Support*
