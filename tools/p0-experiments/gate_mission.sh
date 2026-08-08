#!/bin/bash
# P0-1e gate — INVERTED vs P0-1d: the controllers must now be PRESENT.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SP="${P0_DATA_DIR:-$PWD}"   # output dir; override with P0_DATA_DIR
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "$REPO/install/setup.bash" >/dev/null 2>&1
rc=0

n_gz=$(pgrep -cf '^ign gazebo')
[ "$n_gz" = "1" ] && echo "  GATE gz-servers=1 ........ PASS" \
                  || { echo "  GATE gz-servers=1 ........ FAIL ($n_gz)"; rc=1; }

if timeout 5 ros2 topic echo /hydroships/odom --once >/dev/null 2>&1; then
    echo "  GATE odom-publishing ..... PASS"
else
    echo "  GATE odom-publishing ..... FAIL"; rc=1
fi

nodes=$(ros2 node list 2>/dev/null)
for n in stabilizer thruster_allocator mission_fsm; do
    if echo "$nodes" | grep -qE "/$n\$"; then
        echo "  GATE $n present $(printf '%.0s.' $(seq 1 $((18-${#n})))) PASS"
    else
        echo "  GATE $n present ... FAIL (missing)"; rc=1
    fi
done

cv=$(ros2 topic info /hydroships/cmd_vel 2>/dev/null | grep -oP 'Publisher count: \K\d+')
[ "$cv" = "1" ] && echo "  GATE cmd_vel pub=1 ....... PASS" \
                || { echo "  GATE cmd_vel pub=1 ....... FAIL ($cv)"; rc=1; }

th=$(ros2 topic info /hydroships/thruster_1/thrust 2>/dev/null | grep -oP 'Publisher count: \K\d+')
[ "$th" = "1" ] && echo "  GATE thrust pub=1 ........ PASS" \
                || { echo "  GATE thrust pub=1 ........ FAIL ($th)"; rc=1; }

exit $rc
