#!/bin/bash
# R-11 runtime verification (P1-OWNER-DECISIONS-AND-ROADMAP.md): replay
# deterministik `R11-3002-pinned` (payload dipin, spawn_seed:=3002 -- yang
# dulu 2026-08-14 ABORT via CONVERGEDBG: centered=False dist=0.058
# wall_scored=False) + seed baru (random spawn) untuk verifikasi Opsi 3
# (centered independen dari wall_scored + re-centering gate di DESCEND) di
# lingkungan berbeda dari kasus yang memicu diagnosis.
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_r11_replay_battery.sh
# Window per run 100s (t_dive=20 + t_scan=45 + t_descend=15 + t_grab=10 = 90s
# worst-case sampai keputusan attach pertama, +10s buffer).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

run_one () {
    local tag=$1; shift
    echo "=== $tag: $* ==="
    ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true "$@" \
        > "$SP/$tag.log" 2>&1 &
    local launch=$!
    sleep 100
    kill -9 "$launch" 2>/dev/null
    for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
    pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
    sleep 3
    echo "  torn down; gz=$(pgrep -cf '^ign gazebo')"
}

# 1. Replay deterministik R11-3002-pinned (payload dipin, letter C).
run_one "R11-3002-pinned-replay" qr_letter:=C payload_x:=0.34 payload_y:=-0.35 spawn_seed:=3002

# 2. Seed baru (random spawn ROV + payload) -- generalisasi di luar kasus
#    yang memicu diagnosis awal.
run_one "R11-4001" rov_random_spawn:=true spawn_seed:=4001
run_one "R11-4002" rov_random_spawn:=true spawn_seed:=4002
run_one "R11-4003" rov_random_spawn:=true spawn_seed:=4003

echo
echo "=== hasil: CONVERGEDBG (APPROACH_QR exit) + keputusan GRAB pertama + ABORT? ==="
for tag in R11-3002-pinned-replay R11-4001 R11-4002 R11-4003; do
    echo "--- $tag ---"
    grep -m1 "CONVERGEDBG:" "$SP/$tag.log" 2>/dev/null || echo "  (tak ada CONVERGEDBG -- APPROACH_QR tak pernah konvergen dlm window)"
    grep -m1 "GATEDBG close:" "$SP/$tag.log" 2>/dev/null || echo "  (tak ada GATEDBG close -- GRAB tak pernah diminta dlm window)"
    grep -m1 "DESCEND timeout\|APPROACH_QR timeout\|GRAB timeout\|-> ABORT\|ABORT\]" "$SP/$tag.log" 2>/dev/null \
        || echo "  (tak ada ABORT dlm window)"
done
