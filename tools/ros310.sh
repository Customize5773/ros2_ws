#!/usr/bin/env bash
# Build/run this workspace with Humble's Python ABI, even from a 3.12 venv.
set -e
ws_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ws_dir"
if [[ ! -x .venv-ros310/bin/python ]]; then
    echo 'Create .venv-ros310 first; see docs/HOW-TO-RUN.md (Python 3.12).' >&2
    exit 1
fi
.venv-ros310/bin/python -c 'import sys; assert sys.version_info[:2] == (3, 10), "Python 3.10 required"'

# Discard inherited overlays and Snap GTK modules; load this workspace only.
unset PYTHONHOME PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH GTK_PATH GTK_EXE_PREFIX GTK_IM_MODULE_FILE GIO_MODULE_DIR
source /opt/ros/humble/setup.bash
source .venv-ros310/bin/activate

if [[ "${1:-}" == build ]]; then
    shift
    export CCACHE_DIR="$ws_dir/build/ros310/.ccache"
    exec python /usr/bin/colcon --log-base log/ros310 build \
        --base-paths src --build-base build/ros310 --install-base install/ros310 \
        "$@" --cmake-args \
        -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
        -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
fi

if [[ ! -f install/ros310/local_setup.bash ]]; then
    echo 'Run: bash tools/ros310.sh build' >&2
    exit 1
fi
source install/ros310/local_setup.bash
exec python /opt/ros/humble/bin/ros2 "$@"
