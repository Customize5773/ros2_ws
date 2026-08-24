#!/usr/bin/env python3
"""Inject synthetic QR data and run mission, then analyze _st_grab logs."""
import subprocess
import time
import os
import signal

REPO = "/home/rasya/ros2_ws"
LOG = os.path.join(REPO, "mission_grab_validate.log")

# Source ROS
env = os.environ.copy()
env["ROS_DOMAIN_ID"] = "0"

# Start mission
print("=== Starting mission (headless, seed=1001, QR=A) ===")
launch = subprocess.Popen(
    [
        "ros2", "launch", "hydroships_bringup", "hydroships_mission.launch.py",
        "headless:=true", "spawn_seed:=1001", "qr_letter:=A",
        "payload_x:=0.4", "payload_y:=0.04", "joy_trigger:=false"
    ],
    stdout=open(LOG, "w"),
    stderr=subprocess.STDOUT,
    env=env,
    cwd=REPO,
    shell=False,
)
print(f"launch pid={launch.pid}")

# Wait for APPROACH_QR
print("Waiting for APPROACH_QR...")
for i in range(90):
    time.sleep(1)
    try:
        with open(LOG) as f:
            content = f.read()
        if "DIVE -> APPROACH_QR" in content:
            print(f"APPROACH_QR reached after {i+1}s")
            break
    except Exception:
        pass
else:
    print("TIMEOUT waiting for APPROACH_QR")

# Inject QR result
time.sleep(5)
print("Injecting qr_result A...")
subprocess.run(
    ["ros2", "topic", "pub", "-1", "/hydroships/qr_result",
     "std_msgs/msg/String", "{data: 'A'}"],
    env=env, cwd=REPO, capture_output=True
)

# Inject synthetic qr_offset (centered)
# ey_target at scan_depth=0.30 is approx -0.61 based on logs
print("Injecting synthetic qr_offset (centered)...")
subprocess.run(
    ["ros2", "topic", "pub", "-1", "/hydroships/qr_offset",
     "geometry_msgs/msg/PointStamped",
     "{header: {frame_id: 'camera_bottom_link'}, point: {x: 0.0, y: -0.61, z: 0.2}}"],
    env=env, cwd=REPO, capture_output=True
)

# Wait for mission to reach GRAB or terminal state
print("Waiting for GRAB/terminal state...")
for i in range(180):
    time.sleep(1)
    try:
        with open(LOG) as f:
            content = f.read()
        if any(event in content for event in [
            "GRAB terverifikasi", "GRAB timeout", "ABORT", "DONE",
            "WAIT_TRIGGER", "SURFACE", "NAV_WALL", "HANG", "AUTO_RELEASE"
        ]):
            print(f"Mission event detected after {i+1}s")
            break
    except Exception:
        pass

time.sleep(10)

# Analysis
print("\n=== LOG ANALYSIS ===")
print("--- GRAB / gripper status events ---")
try:
    with open(LOG) as f:
        content = f.read()
    for line in content.splitlines():
        if any(k in line for k in ["GRAB", "gripper/status", "gripper/state", "attached", "rejected", "ABORT"]):
            print(line)
except Exception as e:
    print(f"Error reading log: {e}")

print("\n--- FSM state transitions ---")
try:
    with open(LOG) as f:
        content = f.read()
    for line in content.splitlines():
        if "[FSM]" in line:
            print(line)
except Exception as e:
    print(f"Error reading log: {e}")

print("\n--- Score ---")
try:
    with open(LOG) as f:
        content = f.read()
    for line in content.splitlines():
        if "SKOR" in line:
            print(line)
except Exception as e:
    print(f"Error reading log: {e}")

# Teardown
print("\nTearing down...")
launch.terminate()
try:
    launch.wait(timeout=5)
except subprocess.TimeoutExpired:
    launch.kill()

for p in subprocess.run(["pgrep", "-f", "kki_arena"], capture_output=True, text=True).stdout.strip().split():
    if p:
        os.kill(int(p), signal.SIGKILL)
for pattern in ["hydroships_control/lib", "hydroships_gazebo/lib", "parameter_bridge", "robot_state_publisher"]:
    for p in subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True).stdout.strip().split():
        if p:
            try:
                os.kill(int(p), signal.SIGKILL)
            except ProcessLookupError:
                pass

time.sleep(3)
print("Done")
