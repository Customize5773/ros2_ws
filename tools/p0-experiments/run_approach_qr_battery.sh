#!/bin/bash
# P0-2.2b/P0-2.3 run battery: N=6 APPROACH_QR runs (5 random-spawn + 1
# deterministic) for reduce_approach_qr.py / reduce_qr_precision.py. Loops
# run_approach_qr_smoke.sh sequentially (one Gazebo server at a time, same
# constraint as P0-1e). No auto-retry on gate failure — a failed/contaminated
# run is left for the reducer to flag INCONCLUSIVE; re-running it is a manual
# decision after reviewing why.
#
# Same protocol every time this runs (rov_random_spawn:=true distribution for
# the first 5, one fixed deterministic pose for the 6th) -- "protocol-
# comparable" across batteries, not pose-matched (no RNG seed is exposed to
# force identical random draws). TAG_PREFIX lets a later P0-2.3 battery reuse
# this script into a separate data dir without colliding with the original
# P0-2.2b Q1-Q6 tags.
#
# usage:  P0_DATA_DIR=/somewhere TAG_PREFIX=R bash tools/p0-experiments/run_approach_qr_battery.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
T="${TAG_PREFIX:-Q}"
mkdir -p "$SP"

echo "=== battery start: 6 runs (tag prefix '$T') into $SP ==="

bash "$HERE/run_approach_qr_smoke.sh" "${T}1" rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" "${T}2" rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" "${T}3" rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" "${T}4" rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" "${T}5" rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" "${T}6" rov_random_spawn:=false rov_x:=0.0 rov_y:=0.0 rov_z:=-0.5

echo "=== battery done ==="
for t in "${T}1" "${T}2" "${T}3" "${T}4" "${T}5" "${T}6"; do
    gate=$(cat "$SP/$t.gate.txt" 2>/dev/null || echo "MISSING")
    rows=$(wc -l < "$SP/$t.csv" 2>/dev/null || echo 0)
    echo "  $t: gate=$gate rows=$rows"
done
