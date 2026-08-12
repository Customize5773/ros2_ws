#!/bin/bash
# P1 Fase 1 — full 4-hook mission cycle, end to end, recorded.
#
# Forked from run_approach_qr_smoke.sh (same launch/gate/teardown pattern, same
# kki_arena full stack). Two differences, both required to reach DONE:
#
#   1. WAIT_TRIGGER needs an external pilot trigger on
#      /hydroships/mission/start_autonomous (Empty). mission_fsm resets
#      _trigger_received in SURFACE just before entering WAIT_TRIGGER
#      (mission_fsm.py:817), so a periodic publisher re-arms every cycle.
#      DEVIATION FROM COMPETITION SEMANTICS, stated openly: this fires as soon
#      as the FSM reaches WAIT_TRIGGER, i.e. zero pilot dwell. It tests the
#      autonomy path, not the operator-in-the-loop timing.
#   2. Much longer recording window — one cycle is ~40-90 s and a full run is
#      4 cycles, vs the 60 s single-state window the QR smoke test uses.
#
# Behavior-only observability: publishes nothing except the pilot trigger, and
# changes no parameter.
#
# usage:  run_mission_cycle.sh <tag> [extra launch args...]
#   P0_DATA_DIR=/tmp/p1-fase1 bash tools/p0-experiments/run_mission_cycle.sh C1 spawn_seed:=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
DUR="${CYCLE_DURATION:-420}"     # sim seconds to record
set +u
TAG=$1; shift
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
mkdir -p "$SP"

# PRE-RUN GUARD (P0-1 rule 2, enforced BEFORE launch instead of only after).
# A previous run whose recording window had not expired yet leaves a full second
# stack alive; both then publish /hydroships/cmd_vel and both spawn a Gazebo
# server, and every number from the run is garbage. The post-launch gate does
# catch it (run E1, 2026-08-12: gz-servers=2, cmd_vel pub=2 -> CONTAMINATED),
# but only after wasting the whole run. Refuse to start instead.
STALE=$(pgrep -cf '^ign gazebo' || true)
STALE_NODES=$(pgrep -cf 'install/hydroships' || true)
if [ "${STALE:-0}" -gt 0 ] || [ "${STALE_NODES:-0}" -gt 0 ]; then
    echo "  >>> REFUSING TO START: $STALE gazebo server(s), $STALE_NODES hydroships node(s)"
    echo "  >>> still alive. Tear the previous run down first, then re-run."
    echo "PRECONDITION_FAIL" > "$SP/$TAG.gate.txt"
    exit 2
fi

ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true "$@" \
    > "$SP/$TAG.log" 2>&1 &
LAUNCH=$!

sleep 5
python3 "$HERE/recorder_qr.py" "$SP/$TAG.csv" "$DUR" > "$SP/$TAG.rec.log" 2>&1 &
REC=$!

# Pilot trigger, re-armed every cycle (see header note 1).
ros2 topic pub -r 0.5 /hydroships/mission/start_autonomous std_msgs/msg/Empty '{}' \
    > "$SP/$TAG.trigger.log" 2>&1 &
TRIG=$!

sleep 45
echo "=== GATE $TAG ==="
if bash "$HERE/gate_mission.sh"; then
    echo "PASS" > "$SP/$TAG.gate.txt"
else
    echo "  >>> GATE FAILED - run is CONTAMINATED"
    echo "FAIL" > "$SP/$TAG.gate.txt"
fi

rm -f "$SP"/mission_fsm.yaml "$SP"/_mission_fsm.yaml
ros2 param dump /mission_fsm --output-dir "$SP" > "$SP/$TAG.paramdump.log" 2>&1
mv -f "$SP"/mission_fsm.yaml "$SP/$TAG.params.yaml" 2>/dev/null \
    || mv -f "$SP"/_mission_fsm.yaml "$SP/$TAG.params.yaml" 2>/dev/null \
    || echo "  >>> param dump did not produce expected file, see $TAG.paramdump.log"

wait $REC
kill -9 $TRIG 2>/dev/null
echo "recorder done: $(tail -1 "$SP/$TAG.rec.log")"
echo "rows=$(wc -l < "$SP/$TAG.csv")"
echo "--- mission trace ---"
grep -E "FSM\]|GRAB:|gripper|\+15|\+40|MISI SELESAI|SKOR|timeout|ABORT" "$SP/$TAG.log" | head -60

kill -9 $LAUNCH 2>/dev/null
for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
pkill -9 -f "start_autonomous"
sleep 3
echo "torn down; gz=$(pgrep -cf '^ign gazebo')"
