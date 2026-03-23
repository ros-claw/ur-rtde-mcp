"""
ROSClaw UR-RTDE MCP Server

Universal Robots arm MCP Server using ur_rtde (RTDE protocol, direct TCP).
Part of the ROSClaw Embodied Intelligence Operating System.

Hardware: Universal Robots UR3/UR5/UR10/UR16/UR20 (CB3 and e-Series)
Protocol: RTDE (Real-Time Data Exchange) over TCP port 30004
"""

from __future__ import annotations

import json
import sys
import os
from typing import Optional

sys.path.insert(0, os.path.dirname(__file__))

from mcp.server.fastmcp import FastMCP
from ur_rtde_bridge import (
    URRTDEBridge, _HAS_UR_RTDE, ROBOT_MODE_NAMES,
    JOINT_VEL_MAX, JOINT_ACC_MAX, TOOL_VEL_MAX, TOOL_ACC_MAX,
    SERVO_LOOKAHEAD_MIN, SERVO_LOOKAHEAD_MAX, SERVO_GAIN_MIN, SERVO_GAIN_MAX,
)

mcp = FastMCP("rosclaw-ur-rtde")
_bridge: Optional[URRTDEBridge] = None


def _get_bridge() -> URRTDEBridge:
    global _bridge
    if _bridge is None:
        _bridge = URRTDEBridge()
    return _bridge


# --- Connection ---

@mcp.tool()
def connect_robot(hostname: str, frequency: float = -1.0) -> str:
    """Connect to a Universal Robot via RTDE (port 30004).
    hostname: robot IP. frequency: Hz (-1=default 500Hz/e-Series, 125Hz/CB)."""
    b = _get_bridge()
    if b.is_connected():
        return f"Already connected to {b.hostname}. Call disconnect_robot first."
    try:
        b.connect(hostname, frequency)
        s = b.get_state()
        return (f"Connected to {hostname}. Mode: {ROBOT_MODE_NAMES.get(s.robot_mode)}. "
                f"Safety OK: {not s.is_protective_stopped and not s.is_emergency_stopped}")
    except Exception as e:
        return f"Connection failed: {e}"


@mcp.tool()
def disconnect_robot() -> str:
    """Disconnect from the robot, stopping any running scripts."""
    b = _get_bridge()
    if not b.is_connected():
        return "Not connected."
    b.disconnect()
    return "Disconnected."


# --- Dashboard ---

@mcp.tool()
def get_robot_info() -> str:
    """Get robot model, serial, polyscope version, mode, safety status, loaded program."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        d = b._dash
        return json.dumps({
            "model": d.getRobotModel(), "serial_number": d.getSerialNumber(),
            "polyscope_version": d.polyscopeVersion(), "robot_mode": d.robotmode(),
            "safety_status": d.safetystatus(), "program_state": d.programState(),
            "loaded_program": d.getLoadedProgram(), "remote_control": d.isInRemoteControl(),
        }, indent=2)


@mcp.tool()
def robot_power_control(action: str) -> str:
    """Control robot power state. action: 'on', 'off', or 'brake_release'."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        d = b._dash
        if action == "on":
            d.powerOn()
            return "Power on sent. Wait ~30 s, then call robot_power_control('brake_release')."
        elif action == "off":
            d.powerOff()
            return "Power off sent."
        elif action == "brake_release":
            d.brakeRelease()
            return "Brake release sent. Robot should reach IDLE mode shortly."
        else:
            return f"Unknown action '{action}'. Use 'on', 'off', or 'brake_release'."


@mcp.tool()
def unlock_protective_stop() -> str:
    """Unlock protective stop. Must wait ≥ 5 s after the stop occurred."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        b._dash.unlockProtectiveStop()
    return "Protective stop unlocked."


@mcp.tool()
def restart_safety() -> str:
    """Restart safety after a safety fault. Robot will be in Power Off. Then call power_control('on')."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        b._dash.restartSafety()
    return "Safety restart initiated. Robot is in Power Off."


# --- Motion ---

@mcp.tool()
def move_joint(q: list[float], speed: float = 1.05, acceleration: float = 1.4,
               asynchronous: bool = False) -> str:
    """Move to joint positions moveJ (linear in joint-space).
    q: 6 joint angles [rad]. speed: max 3.14 rad/s. acceleration: max 40 rad/s²."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if len(q) != 6:
        return f"Error: q needs 6 values, got {len(q)}."
    try:
        b.clamp(speed, 0.001, JOINT_VEL_MAX, "speed")
        b.clamp(acceleration, 0.001, JOINT_ACC_MAX, "acceleration")
    except ValueError as e:
        return f"Safety limit: {e}"
    with b._lock:
        ok = b._rtde_c.moveJ(q, speed, acceleration, asynchronous)
    return f"moveJ {'started' if asynchronous else 'completed'} ok={ok}"


@mcp.tool()
def move_linear(pose: list[float], speed: float = 0.25, acceleration: float = 1.2,
                asynchronous: bool = False) -> str:
    """Move TCP linearly in Cartesian space (moveL).
    pose: [x,y,z,rx,ry,rz] m + rotation vector. speed: max 3.0 m/s."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if len(pose) != 6:
        return f"Error: pose needs 6 values, got {len(pose)}."
    try:
        b.clamp(speed, 0.001, TOOL_VEL_MAX, "speed")
        b.clamp(acceleration, 0.001, TOOL_ACC_MAX, "acceleration")
    except ValueError as e:
        return f"Safety limit: {e}"
    with b._lock:
        ok = b._rtde_c.moveL(pose, speed, acceleration, asynchronous)
    return f"moveL {'started' if asynchronous else 'completed'} ok={ok}"


@mcp.tool()
def move_joint_ik(pose: list[float], speed: float = 1.05, acceleration: float = 1.4,
                  asynchronous: bool = False) -> str:
    """Move to TCP pose using IK in joint-space (moveJ_IK).
    pose: [x,y,z,rx,ry,rz]."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if len(pose) != 6:
        return f"Error: pose needs 6 values, got {len(pose)}."
    with b._lock:
        ok = b._rtde_c.moveJ_IK(pose, speed, acceleration, asynchronous)
    return f"moveJ_IK {'started' if asynchronous else 'completed'} ok={ok}"


@mcp.tool()
def stop_motion(mode: str = "joint", deceleration: float = 2.0,
                asynchronous: bool = False) -> str:
    """Decelerate robot to stop. mode: 'joint' (stopJ, rad/s²) or 'linear' (stopL, m/s²)."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        if mode == "joint":
            b._rtde_c.stopJ(deceleration, asynchronous)
        elif mode == "linear":
            b._rtde_c.stopL(deceleration, asynchronous)
        else:
            return f"Unknown mode '{mode}'. Use 'joint' or 'linear'."
    return f"stop{mode.capitalize()} executed."


@mcp.tool()
def servo_joint(q: list[float], time: float, lookahead_time: float = 0.1,
                gain: int = 300) -> str:
    """Single servoJ step for real-time 500 Hz control.
    q: 6 joint positions [rad]. time: step duration [s]. lookahead_time: [0.03-0.2]. gain: [100-2000]."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if len(q) != 6:
        return f"Error: q needs 6 values, got {len(q)}."
    try:
        b.clamp(lookahead_time, SERVO_LOOKAHEAD_MIN, SERVO_LOOKAHEAD_MAX, "lookahead_time")
        b.clamp(float(gain), SERVO_GAIN_MIN, SERVO_GAIN_MAX, "gain")
    except ValueError as e:
        return f"Parameter error: {e}"
    with b._lock:
        ok = b._rtde_c.servoJ(q, 0.0, 0.0, time, lookahead_time, float(gain))
    return f"servoJ ok={ok}"


@mcp.tool()
def speed_joint(qd: list[float], acceleration: float = 0.5, time: float = 0.0) -> str:
    """Accelerate to and hold joint velocities (speedJ).
    qd: 6 velocities [rad/s], max ±3.14. acceleration: [rad/s²]. time: duration [s]."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if len(qd) != 6:
        return f"Error: qd needs 6 values, got {len(qd)}."
    for i, v in enumerate(qd):
        try:
            b.clamp(abs(v), 0.0, JOINT_VEL_MAX, f"qd[{i}]")
        except ValueError as e:
            return f"Safety limit: {e}"
    with b._lock:
        ok = b._rtde_c.speedJ(qd, acceleration, time)
    return f"speedJ ok={ok}"


@mcp.tool()
def force_mode(task_frame: list[float], selection_vector: list[int],
               wrench: list[float], force_type: int, limits: list[float]) -> str:
    """Enter force/compliance mode.
    selection_vector: 0=stiff, 1=compliant. force_type: 1/2/3. All lists must be length 6."""
    b = _get_bridge()
    b.require_connected()
    b.check_safe_to_move()
    if any(len(v) != 6 for v in [task_frame, selection_vector, wrench, limits]):
        return "Error: all lists must have 6 elements."
    if force_type not in (1, 2, 3):
        return "Error: force_type must be 1, 2, or 3."
    with b._lock:
        ok = b._rtde_c.forceMode(task_frame, selection_vector, wrench, force_type, limits)
    return f"forceMode active ok={ok}"


@mcp.tool()
def force_mode_stop() -> str:
    """Exit force mode and return to normal position control."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        ok = b._rtde_c.forceModeStop()
    return f"forceModeStop ok={ok}"


@mcp.tool()
def teach_mode(enable: bool = True) -> str:
    """Enable (True) or disable (False) freedrive/teach mode."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        ok = b._rtde_c.teachMode() if enable else b._rtde_c.endTeachMode()
    return f"{'teachMode' if enable else 'endTeachMode'} ok={ok}"


@mcp.tool()
def jog(speeds: list[float], feature: int = 0, acceleration: float = 0.5,
        stop: bool = False) -> str:
    """Jog robot. stop=True calls jogStop. speeds: [tx,ty,tz mm/s, rx,ry,rz rad/s].
    feature: 0=BASE, 1=TOOL."""
    b = _get_bridge()
    b.require_connected()
    if stop:
        with b._lock:
            ok = b._rtde_c.jogStop()
        return f"jogStop ok={ok}"
    if len(speeds) != 6:
        return f"Error: speeds needs 6 values, got {len(speeds)}."
    b.check_safe_to_move()
    with b._lock:
        ok = b._rtde_c.jogStart(speeds, feature, acceleration)
    return f"jogStart ok={ok}"


@mcp.tool()
def zero_ft_sensor() -> str:
    """Zero the F/T sensor — subtract current reading from subsequent readings."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        ok = b._rtde_c.zeroFtSensor()
    return f"zeroFtSensor ok={ok}"


@mcp.tool()
def set_payload(mass: float, cog: Optional[list[float]] = None) -> str:
    """Set payload mass [kg] and optional CoG [x,y,z m from tool mount]."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        ok = b._rtde_c.setPayload(mass, cog if cog else [])
    return f"setPayload({mass} kg) ok={ok}"


@mcp.tool()
def get_inverse_kinematics(pose: list[float], qnear: Optional[list[float]] = None) -> str:
    """Calculate IK: TCP pose [x,y,z,rx,ry,rz] → joint positions [rad].
    qnear: optional nearby config for redundancy resolution."""
    b = _get_bridge()
    b.require_connected()
    if len(pose) != 6:
        return f"Error: pose needs 6 values, got {len(pose)}."
    with b._lock:
        q = b._rtde_c.getInverseKinematics(pose, qnear if qnear else [])
    return json.dumps({"joint_positions_rad": [round(v, 6) for v in q]})


# --- I/O ---

@mcp.tool()
def set_digital_output(output_id: int, signal_level: bool,
                       output_type: str = "standard") -> str:
    """Set digital output. output_type: 'standard' [0-7], 'configurable' [0-7], 'tool' [0-1]."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        if output_type == "standard":
            ok = b._rtde_io.setStandardDigitalOut(output_id, signal_level)
        elif output_type == "configurable":
            ok = b._rtde_io.setConfigurableDigitalOut(output_id, signal_level)
        elif output_type == "tool":
            ok = b._rtde_io.setToolDigitalOut(output_id, signal_level)
        else:
            return f"Unknown output_type '{output_type}'."
    return f"setDigitalOut({output_type}[{output_id}]={signal_level}) ok={ok}"


@mcp.tool()
def set_speed_slider(speed: float) -> str:
    """Set controller speed slider [0.0–1.0]. 1.0=100%."""
    b = _get_bridge()
    b.require_connected()
    try:
        b.clamp(speed, 0.0, 1.0, "speed")
    except ValueError as e:
        return f"Range error: {e}"
    with b._lock:
        ok = b._rtde_io.setSpeedSlider(speed)
    return f"setSpeedSlider({speed:.2f}) ok={ok}"


@mcp.tool()
def set_analog_output(output_id: int, ratio: float,
                      output_type: str = "voltage") -> str:
    """Set analog output as fraction of full span [0.0–1.0]. output_type: 'voltage' or 'current'."""
    b = _get_bridge()
    b.require_connected()
    try:
        b.clamp(ratio, 0.0, 1.0, "ratio")
    except ValueError as e:
        return f"Range error: {e}"
    with b._lock:
        if output_type == "voltage":
            ok = b._rtde_io.setAnalogOutputVoltage(output_id, ratio)
        elif output_type == "current":
            ok = b._rtde_io.setAnalogOutputCurrent(output_id, ratio)
        else:
            return f"Unknown output_type '{output_type}'."
    return f"setAnalogOutput({output_type}[{output_id}]={ratio:.3f}) ok={ok}"


# --- State Reading ---

@mcp.tool()
def get_robot_state() -> str:
    """Get full robot state: joints, TCP pose/speed/force, safety, temperatures, voltage."""
    b = _get_bridge()
    try:
        return b.format_state_json()
    except Exception as e:
        return f"Error: {e}"


@mcp.tool()
def get_tcp_pose() -> str:
    """Get current TCP pose [x,y,z,rx,ry,rz] (meters + rotation vector)."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        pose = b._rtde_r.getActualTCPPose()
    return json.dumps({"tcp_pose_m_rad": [round(v, 6) for v in pose]})


@mcp.tool()
def get_joint_positions() -> str:
    """Get current joint positions [j1..j6] in radians."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        q = b._rtde_r.getActualQ()
    return json.dumps({"joint_positions_rad": [round(v, 6) for v in q]})


@mcp.tool()
def get_tcp_force() -> str:
    """Get TCP wrench [Fx,Fy,Fz N, Tx,Ty,Tz N·m]."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        force = b._rtde_r.getActualTCPForce()
    return json.dumps({"tcp_force_N_Nm": [round(v, 4) for v in force]})


@mcp.tool()
def get_digital_io_state() -> str:
    """Get all digital I/O states (DI/DO 0-17: 0-7=standard, 8-15=configurable, 16-17=tool)."""
    b = _get_bridge()
    b.require_connected()
    with b._lock:
        in_bits = b._rtde_r.getActualDigitalInputBits()
        out_bits = b._rtde_r.getActualDigitalOutputBits()
    return json.dumps({
        "digital_inputs": {f"DI{i}": bool((in_bits >> i) & 1) for i in range(18)},
        "digital_outputs": {f"DO{i}": bool((out_bits >> i) & 1) for i in range(18)},
    })


# --- Resources ---

@mcp.resource("robot://status")
def resource_robot_status() -> str:
    """Full robot status snapshot (mode, joints, TCP, forces, safety)."""
    b = _get_bridge()
    if not b.is_connected():
        return json.dumps({"connected": False, "message": "Call connect_robot first."})
    try:
        return b.format_state_json()
    except Exception as e:
        return json.dumps({"connected": True, "error": str(e)})


@mcp.resource("robot://connection")
def resource_connection() -> str:
    """RTDE connection status and library availability."""
    b = _get_bridge()
    return json.dumps({
        "connected": b.is_connected(),
        "hostname": b.hostname or "not set",
        "ur_rtde_available": _HAS_UR_RTDE,
    })


def main() -> None:
    mcp.run()


if __name__ == "__main__":
    main()
