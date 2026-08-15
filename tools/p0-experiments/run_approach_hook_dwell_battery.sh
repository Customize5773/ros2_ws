#!/bin/bash
# M6 dwell-debounce runtime verification (docs/STATUS.md 2026-08-16 update,
# docs/CHANGELOG.md ~L974): fix di hook_logic.update_dwell() (grace period
# 0.4s) seharusnya bikin APPROACH_HOOK keluar via konvergensi asli ("hook
# terpusat ... -> AUTO_RELEASE"), bukan fallback timeout t_approach=25s
# ("APPROACH_HOOK timeout -> lanjut"). Belum pernah diverifikasi runtime.
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_approach_hook_dwell_battery.sh
# Window per run 40s (t_approach=25s worst-case + 15s buffer utk backoff+AUTO_RELEASE settle).
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
    sleep 55   # ponytail: 40s awal terlalu ketat -- gz boot (~10-15s) + t_approach=25s
               # kepotong window; 4/8 run pertama belum sampai exit. Naikkan jika masih kurang.
    kill -9 "$launch" 2>/dev/null
    for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
    pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
    sleep 3
    echo "  torn down; gz=$(pgrep -cf '^ign gazebo')"
}

# start_state:=APPROACH_HOOK butuh start_wall manual (self.wall tak ter-set
# dari state sebelumnya -- lihat mission_fsm.py:357, hydroships_mission.launch.py:87-91).
tags=()
for wall in A B C D; do
    for seed in 5001 5002; do
        tag="AH-${wall}-${seed}"
        tags+=("$tag")
        run_one "$tag" start_state:=APPROACH_HOOK start_wall:="$wall" spawn_seed:="$seed"
    done
done

echo
echo "=== hasil: konvergensi ('hook terpusat') vs timeout fallback ==="
conv=0; timeout=0; neither=0
for tag in "${tags[@]}"; do
    if grep -q "hook terpusat" "$SP/$tag.log" 2>/dev/null; then
        echo "--- $tag: KONVERGENSI ---"
        grep -m1 "hook terpusat" "$SP/$tag.log"
        conv=$((conv+1))
    elif grep -q "APPROACH_HOOK timeout" "$SP/$tag.log" 2>/dev/null; then
        echo "--- $tag: TIMEOUT ---"
        timeout=$((timeout+1))
    else
        echo "--- $tag: TAK ADA EXIT dlm window (cek log) ---"
        neither=$((neither+1))
    fi
done
echo
echo "TOTAL: $conv konvergensi, $timeout timeout, $neither tak-selesai (dari ${#tags[@]} run)"
