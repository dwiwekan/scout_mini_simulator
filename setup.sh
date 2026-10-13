#!/usr/bin/env bash
# Fetch the upstream AgileX Gazebo packages and apply this repo's changes on top.
#
#   ./setup.sh               simulation only
#   ./setup.sh --real-robot  also clone ugv_sdk + scout_ros2 (drivers for the real Scout Mini)
#   ./setup.sh --link        symlink the overlay files instead of copying them
#                            (for working on this repo: edits land in overlay/)
set -euo pipefail

UGV_SIM_REPO=https://github.com/ROSETEA-lab/ugv_gazebo_sim.git
UGV_SIM_COMMIT=3b9de23ace6aaa5f05159d1bc48da7a0e89a0a77   # the commit the overlay was made against
UGV_SDK_REPO=https://github.com/agilexrobotics/ugv_sdk.git
SCOUT_ROS2_REPO=https://github.com/ROSETEA-lab/scout_ros2.git

# ROS 1 only packages in ugv_gazebo_sim, they break colcon build
IGNORE_DIRS="autokit hunter hunter_se limo limo_s2 ranger_mini_V2 tracer tracer_mini scout/scout_control"

REAL_ROBOT=false
LINK=false
for arg in "$@"; do
  case "$arg" in
    --real-robot) REAL_ROBOT=true ;;
    --link) LINK=true ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$ROOT/src/ugv_gazebo_sim"
OVERLAY="$ROOT/overlay/ugv_gazebo_sim"

# 1. ugv_gazebo_sim at the pinned commit (shallow, it is ~400 MB with full history)
if [ ! -d "$SIM/.git" ]; then
  echo "==> Cloning ugv_gazebo_sim @ ${UGV_SIM_COMMIT:0:7}"
  git init -q "$SIM"
  git -C "$SIM" remote add origin "$UGV_SIM_REPO"
  if ! git -C "$SIM" fetch -q --depth 1 origin "$UGV_SIM_COMMIT"; then
    echo "    shallow fetch failed, fetching full history"
    git -C "$SIM" fetch -q origin
  fi
  git -C "$SIM" -c advice.detachedHead=false checkout -q "$UGV_SIM_COMMIT"
else
  HEAD_COMMIT="$(git -C "$SIM" rev-parse HEAD)"
  if [ "$HEAD_COMMIT" != "$UGV_SIM_COMMIT" ]; then
    echo "WARNING: src/ugv_gazebo_sim is at ${HEAD_COMMIT:0:7}, the overlay was made for ${UGV_SIM_COMMIT:0:7}" >&2
  fi
fi

# 2. Apply the overlay
echo "==> Applying overlay ($([ "$LINK" = true ] && echo symlink || echo copy))"
cd "$OVERLAY"
find . -type f | while read -r f; do
  dst="$SIM/${f#./}"
  mkdir -p "$(dirname "$dst")"
  if [ "$LINK" = true ]; then
    ln -sfn "$OVERLAY/${f#./}" "$dst"
  else
    rm -f "$dst"   # in case it was a symlink from --link
    cp "$f" "$dst"
  fi
done
cd "$ROOT"

for d in $IGNORE_DIRS; do
  [ -d "$SIM/$d" ] && touch "$SIM/$d/COLCON_IGNORE"
done

# 3. Real robot drivers (optional)
if [ "$REAL_ROBOT" = true ]; then
  [ -d "$ROOT/src/ugv_sdk" ]    || git clone --depth 1 "$UGV_SDK_REPO" "$ROOT/src/ugv_sdk"
  [ -d "$ROOT/src/scout_ros2" ] || git clone --depth 1 -b humble "$SCOUT_ROS2_REPO" "$ROOT/src/scout_ros2"
  # scout_ros2 has its own scout_description; keep ours (it has the Livox + ZED frames)
  touch "$ROOT/src/scout_ros2/scout_description/COLCON_IGNORE"
fi

echo
echo "Done. Next:"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install && source install/setup.bash"
