#!/bin/bash
# R-10 runtime verification (P1-OWNER-DECISIONS-AND-ROADMAP.md): battery
# pembanding descend_depth_tol SEBELUM (0.06, replikasi depth_tol lama yg
# dulu dipakai exit DESCEND) vs SESUDAH (0.02, default baru sejak 2026-08-14
# -- lihat hydroships_mission.launch.py). Hipotesis: toleransi lebih ketat
# menurunkan ROV lebih dekat ke grab_depth sebenarnya sebelum GRAB memicu
# "close", jadi alt_gap MENGECIL & margin ke max_alt_gap=0.08 MEMBESAR dari
# 5-7mm (M5-D diagnosis, 3/3 run) mendekati celah rancangan 0.034m.
#
# Same protocol run twice per seed (before/after), spawn_seed sama supaya
# pose spawn ROV & payload identik -- satu-satunya variabel yg beda adalah
# descend_depth_tol. Reuse spawn_seed 3001/3002/3003 (kontinuitas dgn
# battery R-9/R-11 sebelumnya, bukan seed baru tanpa alasan).
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_r10_descend_tol_battery.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"
SEEDS=(3001 3002 3003)

source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"

run_one () {
    local tag=$1 tol=$2 seed=$3
    echo "=== $tag (descend_depth_tol=$tol spawn_seed=$seed) ==="
    ros2 launch hydroships_bringup hydroships_mission.launch.py headless:=true \
        spawn_seed:="$seed" descend_depth_tol:="$tol" \
        > "$SP/$tag.log" 2>&1 &
    local launch=$!
    sleep 75
    kill -9 "$launch" 2>/dev/null
    for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
    pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
    sleep 3
    echo "  torn down; gz=$(pgrep -cf '^ign gazebo')"
}

for seed in "${SEEDS[@]}"; do
    run_one "R10-before-$seed" 0.06 "$seed"
    run_one "R10-after-$seed" 0.02 "$seed"
done

echo
echo "=== hasil: alt_gap pada tick GRAB pertama (GATEDBG close) ==="
for seed in "${SEEDS[@]}"; do
    for cond in before after; do
        tag="R10-$cond-$seed"
        line=$(grep -m1 "GATEDBG close:" "$SP/$tag.log" 2>/dev/null)
        echo "[$tag] ${line:-'(tak ada GATEDBG close -- GRAB tak pernah diminta dalam window)'}"
    done
done
