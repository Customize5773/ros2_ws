#!/bin/bash
# R-10 trajectory capture: run one mission + recorder_qr pair.
#
# Usage: P0_DATA_DIR=/path bash run_r10_trajectory.sh <tag> <descend_depth_tol> <spawn_seed> [duration_sim_s]
#
# Example:
#   P0_DATA_DIR=/tmp/r10-trajectory bash run_r10_trajectory.sh R10-repro-3001-new 0.02 3001 150
#
# Outputs:
#   $P0_DATA_DIR/<tag>.log      — mission launch stdout/stderr
#   $P0_DATA_DIR/<tag>.csv      — recorder_qr output (DESCEND trajectory)
#
# After the run, optionally run the reducer:
#   python3 tools/p0-experiments/reduce_r10_trajectory.py "$P0_DATA_DIR/<tag>.csv"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"

TAG="${1:?tag required}"
TOL="${2:?descend_depth_tol required}"
SEED="${3:?spawn_seed required}"
DURATION="${4:-150}"   # sim seconds for recorder (default 150 = ample for full mission)

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

LOG="$SP/${TAG}.log"
CSV="$SP/${TAG}.csv"
META="$SP/${TAG}.meta"

echo "=== $TAG (descend_depth_tol=$TOL spawn_seed=$SEED duration=${DURATION}s) ==="
echo "    log=$LOG"
echo "    csv=$CSV"
echo "    meta=$META"

# Write metadata for reducer
cat > "$META" <<EOF
tag=$TAG
descend_depth_tol=$TOL
spawn_seed=$SEED
duration=$DURATION
EOF

# Launch mission
ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true \
    spawn_seed:="$SEED" descend_depth_tol:="$TOL" \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!
echo "    launch pid=$LAUNCH_PID"

# Wait for sim clock to be alive before starting recorder
sleep 5

# Start recorder (subscribe-only, no publish)
python3 "$HERE/recorder_qr.py" "$CSV" "$DURATION" > "$SP/${TAG}.recorder.log" 2>&1 &
RECORDER_PID=$!
echo "    recorder pid=$RECORDER_PID"

# Wait: recorder stops itself after DURATION sim seconds, but wall-clock
# timeout is a safety net (Gazebo at 0.5x RT factor = 150s sim ≈ 300s wall).
WALL_TIMEOUT=$((DURATION * 3 + 60))
echo "    wall-clock timeout=${WALL_TIMEOUT}s"
sleep "$WALL_TIMEOUT"

# Teardown
kill -9 "$LAUNCH_PID" 2>/dev/null
kill -9 "$RECORDER_PID" 2>/dev/null
for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
pkill -9 -f "hydroships_control/lib" 2>/dev/null || true
pkill -9 -f "hydroships_gazebo/lib" 2>/dev/null || true
pkill -9 -f "parameter_bridge" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
sleep 3

if [ -f "$CSV" ]; then
    LINES=$(wc -l < "$CSV")
    echo "    done. csv lines=$LINES"
else
    echo "    WARNING: csv not created"
fi
echo "    torn down; gz=$(pgrep -cf '^ign gazebo' || echo 0)"
