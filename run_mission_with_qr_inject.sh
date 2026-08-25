#!/bin/bash
REPO=/home/rasya/ros2_ws
SP=/home/rasya/ros2_ws

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

TAG="mission_grab_validate"
LOG="$SP/${TAG}.log"

echo "=== Starting mission (headless, seed=1001, QR=A) ==="
ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true \
    spawn_seed:=1001 qr_letter:=A payload_x:=0.4 payload_y:=0.04 \
    joy_trigger:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "launch pid=$LAUNCH_PID"

# Wait for mission to reach APPROACH_QR
echo "Waiting for APPROACH_QR..."
for i in $(seq 1 90); do
    if grep -q "DIVE -> APPROACH_QR" "$LOG" 2>/dev/null; then
        echo "APPROACH_QR reached after ${i}s"
        break
    fi
    sleep 1
done

# Wait a bit more for ROV to get close, then inject QR
sleep 15

echo "Injecting QR result A..."
ros2 topic pub -1 /hydroships/qr_result std_msgs/msg/String "{data: 'A'}" >/dev/null 2>&1 || true

# Wait for mission to reach GRAB or terminal state
echo "Waiting for GRAB/terminal state..."
for i in $(seq 1 180); do
    if grep -q "GRAB terverifikasi\|GRAB timeout\|ABORT\|DONE\|WAIT_TRIGGER\|SURFACE\|NAV_WALL\|HANG\|AUTO_RELEASE" "$LOG" 2>/dev/null; then
        echo "Mission event detected after ${i}s"
        break
    fi
    sleep 1
done

sleep 10
echo "=== LOG ANALYSIS ==="
echo "--- GRAB / gripper status events ---"
grep -E "GRAB|gripper/status|gripper/state|attached|rejected|ABORT" "$LOG" | head -40

echo ""
echo "--- FSM state transitions ---"
grep "\[FSM\]" "$LOG"

echo ""
echo "--- Score ---"
grep "SKOR" "$LOG"

kill -9 "$LAUNCH_PID" 2>/dev/null
for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
pkill -9 -f "hydroships_control/lib" 2>/dev/null || true
pkill -9 -f "hydroships_gazebo/lib" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
sleep 3
echo "torn down"
