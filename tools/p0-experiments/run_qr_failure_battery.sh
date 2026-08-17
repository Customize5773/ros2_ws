#!/bin/bash
# P0-2.7 SPEC -- failure-prone APPROACH_QR battery (docs/P0-2-7-FAILURE-BATTERY-SPEC.md).
# NOT RUN as part of P0-2.7's design task -- written and left for a separately
# approved execution pass, same discipline P0-2.6 used for its formulated
# candidates. No candidate reordering, no detector/FSM/controller changes.
#
# Loops run_approach_qr_smoke.sh (unmodified) with rov_random_spawn:=true and
# spawn_seed:=<seed> drawn from select_failure_seeds.py's output, in batches
# of 6 (one Gazebo server at a time, same constraint as every prior battery).
# After each batch, counts V1-class-failure runs (gate_mission.sh PASS AND
# reduce_approach_qr.py's entered_band_with_dwell==False) and their pooled
# corner-bearing observation count via count_failure_progress.py (this
# script's own inline python, no new file needed for something this small),
# stopping as soon as BOTH minimums are met:
#   >= 8 qualifying V1-class-failure runs
#   >= 40 pooled independent corner-bearing observations across those runs
# Hard cap: 30 total runs (5 batches). If the cap is hit without meeting both
# minimums, this script stops and prints INSUFFICIENT DATA -- it does NOT
# lower the bar or keep going past the cap.
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_qr_failure_battery.sh <seed1> <seed2> ...
#   (seed list: output of `select_failure_seeds.py`, e.g. its printed
#    "seed pool for run_qr_failure_battery.sh: ..." line)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
MIN_FAILURES=8
MIN_OBS=40
MAX_RUNS=30
BATCH_SIZE=6
mkdir -p "$SP"

SEEDS=("$@")
if [ "${#SEEDS[@]}" -eq 0 ]; then
    echo "usage: P0_DATA_DIR=/somewhere bash $0 <seed1> <seed2> ..." >&2
    echo "  (seed list from: python3 $HERE/select_failure_seeds.py)" >&2
    exit 1
fi
if [ "${#SEEDS[@]}" -lt "$MAX_RUNS" ]; then
    echo "WARNING: only ${#SEEDS[@]} seeds supplied, less than the $MAX_RUNS-run cap." >&2
    echo "  Battery may exhaust its seed pool before meeting minimums or hitting the cap." >&2
fi

echo "=== P0-2.7 failure battery start: up to $MAX_RUNS runs in batches of $BATCH_SIZE ==="
echo "    stop when >=$MIN_FAILURES qualifying failures AND >=$MIN_OBS pooled observations"

run_idx=0
tags=()
while [ "$run_idx" -lt "${#SEEDS[@]}" ] && [ "$run_idx" -lt "$MAX_RUNS" ]; do
    batch_end=$((run_idx + BATCH_SIZE))
    [ "$batch_end" -gt "${#SEEDS[@]}" ] && batch_end="${#SEEDS[@]}"
    [ "$batch_end" -gt "$MAX_RUNS" ] && batch_end="$MAX_RUNS"

    echo "--- batch: runs $((run_idx + 1))..$batch_end ---"
    while [ "$run_idx" -lt "$batch_end" ]; do
        seed="${SEEDS[$run_idx]}"
        tag="F$((run_idx + 1))"
        tags+=("$tag")
        bash "$HERE/run_approach_qr_smoke.sh" "$tag" rov_random_spawn:=true spawn_seed:="$seed"
        run_idx=$((run_idx + 1))
    done

    # docs/P0-2-7-FAILURE-BATTERY-SPEC.md failure definition: gate_mission.sh
    # PASS (excludes infra contamination) AND entered_band_with_dwell==False
    # (reduce_approach_qr.py's own Gate 4 retest / quality_gate() / analyze_run()
    # -- reused directly by import, no new metric invented, no new file).
    progress=$(HERE="$HERE" SP="$SP" python3 - "${tags[@]}" <<'PYEOF'
import os, sys
sys.path.insert(0, os.environ['HERE'])
from reduce_approach_qr import quality_gate, load_params, analyze_run
from analyze_qr_candidates import extract_observations  # P0-2.6, reused not copied

data_dir = os.environ['SP']
n_fail = 0
n_obs = 0
for tag in sys.argv[1:]:
    rows, reasons = quality_gate(tag, data_dir)
    if reasons:
        continue  # contaminated / infra-failed run -- not a V1-class failure
    params, used_defaults = load_params('%s/%s.params.yaml' % (data_dir, tag))
    result = analyze_run(tag, data_dir, rows, params, used_defaults)
    if not result.get('reached_approach_qr'):
        continue
    g4 = result['p0_2_4_gate4_retest']
    if g4['entered_band_with_dwell']:
        continue  # converged -- not a failure
    n_fail += 1
    n_obs += len(extract_observations(tag, rows))

print('qualifying_failures=%d pooled_observations=%d' % (n_fail, n_obs))
PYEOF
)
    echo "$progress"
    n_fail=$(echo "$progress" | grep -oP 'qualifying_failures=\K\d+')
    n_obs=$(echo "$progress" | grep -oP 'pooled_observations=\K\d+')

    if [ "$n_fail" -ge "$MIN_FAILURES" ] && [ "$n_obs" -ge "$MIN_OBS" ]; then
        echo "=== stopping: minimums met (failures=$n_fail obs=$n_obs) ==="
        exit 0
    fi
done

echo "=== cap reached ($run_idx runs) without meeting both minimums ==="
echo "=== VERDICT: INSUFFICIENT DATA (per docs/P0-2-7-FAILURE-BATTERY-SPEC.md stopping rule) ==="
