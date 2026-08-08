#!/bin/bash
# P0-2.1 APPROACH_QR instrumentation smoke test: one full-stack mission run,
# recorded with recorder_qr.py, to verify the recorder actually captures FSM
# state, QR detection, QR offset/error, controller output, and odometry.
#
# This is observability-only: no behavior change to APPROACH_QR, no gain or
# parameter tuning. Forked from run_mission.sh (P0-1e), same launch/gate/
# teardown pattern, same kki_arena full stack.
#
# usage:  run_approach_qr_smoke.sh <tag> [extra launch args...]
#   P0_DATA_DIR=/somewhere  bash tools/p0-experiments/run_approach_qr_smoke.sh S1
# Outputs <tag>.csv / <tag>.log into $P0_DATA_DIR (default: current directory).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"   # output dir; override with P0_DATA_DIR
set +u
TAG=$1; shift
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true "$@" \
    > "$SP/$TAG.log" 2>&1 &
LAUNCH=$!

# recorder waits internally for /clock + /hydroships/odom, so start it now
sleep 5
python3 "$HERE/recorder_qr.py" "$SP/$TAG.csv" 60 > "$SP/$TAG.rec.log" 2>&1 &
REC=$!

# gate while recording (does not perturb: read-only introspection)
sleep 45
echo "=== GATE $TAG ==="
bash "$HERE/gate_mission.sh" || echo "  >>> GATE FAILED - run is CONTAMINATED"

wait $REC
echo "recorder done: $(tail -1 "$SP/$TAG.rec.log")"
echo "rows=$(wc -l < "$SP/$TAG.csv")"
grep -E "FSM\]|qr_detector|DIVE timeout|spawn \(random" "$SP/$TAG.log" | head -12

kill -9 $LAUNCH 2>/dev/null
for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
sleep 3
echo "torn down; gz=$(pgrep -cf '^ign gazebo')"
