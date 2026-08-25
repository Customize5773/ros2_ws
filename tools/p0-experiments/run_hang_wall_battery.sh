#!/bin/bash
# Battery HANG per-wall/seed — cek apakah "dist macet" (posisi tak konvergen
# ke hang_tol=25mm) polanya per-wall (spt bias ey APPROACH_HOOK di wall C/D)
# atau kebetulan/acak.
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_hang_wall_battery.sh
# Wall x seed: A=6001-6003, B=6004-6006, C=6007-6009, D=6010-6012
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
    sleep 60
    kill -9 "$launch" 2>/dev/null
    for p in $(pgrep -f "kki_arena"); do kill -9 "$p" 2>/dev/null; done
    pkill -9 -f "gz sim"
    pkill -9 -f "hydroships_control/lib"; pkill -9 -f "hydroships_gazebo/lib"
    pkill -9 -f "parameter_bridge"; pkill -9 -f "robot_state_publisher"
    sleep 10
    echo "  torn down; gz=$(pgrep -cf 'gz sim')"
}

tags=()
for wall in A B C D; do
    case "$wall" in
        A) seeds=(6001 6002 6003) ;;
        B) seeds=(6004 6005 6006) ;;
        C) seeds=(6007 6008 6009) ;;
        D) seeds=(6010 6011 6012) ;;
    esac
    for seed in "${seeds[@]}"; do
        tag="HANG-${wall}-${seed}"
        tags+=("$tag")
        run_one "$tag" start_state:=HANG start_wall:="$wall" spawn_seed:="$seed"
    done
done

echo
echo "=== hasil: seated vs pos-timeout vs descend-timeout vs tak-selesai ==="
declare -A per_wall_seated per_wall_pos per_wall_desc per_wall_neither
seated=0; pos_to=0; desc_to=0; neither=0
for tag in "${tags[@]}"; do
    wall="${tag#HANG-}"; wall="${wall%%-*}"
    if grep -q "Payload tergantung stabil di hook" "$SP/$tag.log" 2>/dev/null; then
        line=$(grep -m1 "Payload tergantung stabil di hook" "$SP/$tag.log")
        echo "--- $tag: SEATED --- $line"
        seated=$((seated+1)); per_wall_seated[$wall]=$(( ${per_wall_seated[$wall]:-0} + 1 ))
    elif grep -q "HANG timeout (posisi" "$SP/$tag.log" 2>/dev/null; then
        line=$(grep -m1 "HANG timeout (posisi" "$SP/$tag.log")
        echo "--- $tag: POS-TIMEOUT --- $line"
        pos_to=$((pos_to+1)); per_wall_pos[$wall]=$(( ${per_wall_pos[$wall]:-0} + 1 ))
    elif grep -q "HANG timeout (turun" "$SP/$tag.log" 2>/dev/null; then
        line=$(grep -m1 "HANG timeout (turun" "$SP/$tag.log")
        echo "--- $tag: DESCEND-TIMEOUT --- $line"
        desc_to=$((desc_to+1)); per_wall_desc[$wall]=$(( ${per_wall_desc[$wall]:-0} + 1 ))
    else
        echo "--- $tag: TAK SELESAI dlm window (cek log manual) ---"
        neither=$((neither+1)); per_wall_neither[$wall]=$(( ${per_wall_neither[$wall]:-0} + 1 ))
    fi
    # bukti teleport odom (lihat CHANGELOG/STATUS 2026-08-24): dua pose >1m
    # beda di tick berurutan -> flag biar kelihatan tanpa buka log manual.
    if grep -oP 'HANG dbg: dist=\K[0-9.]+' "$SP/$tag.log" 2>/dev/null \
        | awk 'NR>1 && ($1-prev>1.0 || prev-$1>1.0) {f=1} {prev=$1} END{exit !f}'; then
        echo "    [!] lonjakan dist >1.0 antar-tick terdeteksi (dugaan odom interleave, lihat STATUS.md)"
    fi
done

echo
echo "TOTAL: $seated seated, $pos_to pos-timeout, $desc_to descend-timeout, $neither tak-selesai (dari ${#tags[@]} run)"
echo
echo "Per wall (seated/pos-timeout/descend-timeout/tak-selesai):"
for wall in A B C D; do
    echo "  $wall: ${per_wall_seated[$wall]:-0}/${per_wall_pos[$wall]:-0}/${per_wall_desc[$wall]:-0}/${per_wall_neither[$wall]:-0}"
done
