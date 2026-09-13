#!/usr/bin/env bash
# ============================================================================
# run_isaac.sh -- launch a kotek_ws script with a working Isaac Sim 6.0.1
# ============================================================================
# This host was migrated from x86_64 to aarch64 (NVIDIA GB10 / DGX Spark).
# kotek_ws/env_isaaclab, which every command in README.md used to source, is
# now an empty shell -- only a stray lib/ directory survives, there is no
# bin/activate and no interpreter. The working Isaac Sim 6.0.1 install on
# this host is the one ~/Projects/hsr_sim uses, and this script is modelled
# directly on hsr_sim/run.sh.
#
# Isaac Sim ships as a normal pip package here (not a standalone bundle with
# its own python.sh), so "the Isaac Sim launcher" is: that venv's interpreter,
# plus one required env var.
#
# On aarch64/GB10 `import isaacsim` aborts immediately unless libgomp is
# preloaded (empirical, not an Isaac Sim requirement in general):
#     ImportError: ... libgomp-xxxxxxxx.so.1: cannot allocate memory in
#     static TLS block
#
# Usage:
#   ./run_isaac.sh src/kotek_isaac_stage/kotek_isaac_stage/import_robots.py
#   ./run_isaac.sh path/to/script.py --some-arg
#   ISAAC_PYTHON=/other/env/bin/python ./run_isaac.sh script.py
#
# ROS 2: Isaac's internal rclpy needs its bundled typesupport libs on
# LD_LIBRARY_PATH (the GUI app wires this automatically; a standalone script
# does not). Set KOTEK_WITH_ROS=1 to add them plus the RMW/distro env.
# rmw_cyclonedds_cpp is mandatory for anything that crosses to the docker
# container -- see README.md and docs/pick_and_delivery_report.md section 4.1.
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DEFAULT_PYTHON="/home/trs/env_isaaclab/bin/python"
ISAAC_PYTHON="${ISAAC_PYTHON:-$DEFAULT_PYTHON}"

if [ ! -x "$ISAAC_PYTHON" ]; then
  echo "Isaac Sim python not found at: $ISAAC_PYTHON" >&2
  echo "Set ISAAC_PYTHON=/path/to/env/bin/python and retry." >&2
  exit 1
fi

if [ $# -lt 1 ]; then
  echo "Usage: $0 <script.py> [args...]" >&2
  exit 1
fi

# NOTE: Isaac Sim's startup check does a literal string match on the
# LD_PRELOAD path, not a resolved-inode check -- "/lib/aarch64-linux-gnu/..."
# passes but the equivalent "/usr/lib/aarch64-linux-gnu/..." (the same file,
# via the /lib -> usr/lib symlink) does NOT. Keep the /lib/... form first.
LIBGOMP=""
for cand in \
  /lib/aarch64-linux-gnu/libgomp.so.1 \
  /lib/x86_64-linux-gnu/libgomp.so.1 \
  /usr/lib/aarch64-linux-gnu/libgomp.so.1 \
  /usr/lib/x86_64-linux-gnu/libgomp.so.1; do
  if [ -f "$cand" ]; then LIBGOMP="$cand"; break; fi
done

if [ "${KOTEK_WITH_ROS:-0}" = "1" ]; then
  export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  # Derived from the interpreter's own site-packages rather than by importing
  # isaacsim: that import would run in a subprocess WITHOUT the LD_PRELOAD set
  # below, and Isaac's startup check aborts the whole invocation when it is
  # missing.
  ISAAC_DIR="$(dirname "$(dirname "$ISAAC_PYTHON")")/lib/python3.12/site-packages/isaacsim"
  if [ ! -d "$ISAAC_DIR" ]; then
    ISAAC_DIR="$(ls -d "$(dirname "$(dirname "$ISAAC_PYTHON")")"/lib/python*/site-packages/isaacsim 2>/dev/null | head -1)"
  fi
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ISAAC_DIR}/exts/isaacsim.ros2.core/${ROS_DISTRO}/lib"
fi

cd "$SCRIPT_DIR"
if [ -n "$LIBGOMP" ]; then
  exec env LD_PRELOAD="${LIBGOMP}${LD_PRELOAD:+:$LD_PRELOAD}" "$ISAAC_PYTHON" "$@"
else
  echo "Warning: libgomp.so.1 not found; launching without LD_PRELOAD." >&2
  exec "$ISAAC_PYTHON" "$@"
fi
