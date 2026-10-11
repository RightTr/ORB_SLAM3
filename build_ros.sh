#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 || ( "$1" != "ROS1" && "$1" != "ROS2" ) ]]; then
  echo "Usage: $0 ROS1|ROS2" >&2
  exit 2
fi
edition="$1"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
wrapper_root="$repo_root/Examples_ROS"
core_root_input="${ORB_SLAM3_ROOT:-$repo_root}"
core_root="$(cd "$core_root_input" && pwd)"
# ORB_SLAM3 lives in <ros_workspace>/src/ORB_SLAM3; build from the ROS workspace root.
workspace_root="$(cd "$repo_root/../.." && pwd)"

if [[ ! -f "$core_root/lib/libORB_SLAM3.so" || \
      ( ! -f "$core_root/Thirdparty/DBoW2/lib/libDBoW2.so" && \
        ! -f "$core_root/lib/libDBoW2.so" ) || \
      ! -f "$core_root/Thirdparty/g2o/lib/libg2o.so" ]]; then
  "$core_root/build.sh"
fi

if [[ "$edition" == "ROS1" ]]; then
  ros_distro="${ROS1_DISTRO:-noetic}"
  manifest="$wrapper_root/package_ROS1.xml"
else
  ros_distro="${ROS2_DISTRO:-humble}"
  manifest="$wrapper_root/package_ROS2.xml"
fi
setup_file="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -f "$setup_file" ]]; then
  echo "ROS setup not found: $setup_file" >&2
  exit 1
fi
# ROS setup scripts may reference optional unset variables such as
# AMENT_TRACE_SETUP_FILES, so temporarily disable nounset while sourcing.
set +u
source "$setup_file"
set -u

had_manifest=0
backup=""
if [[ -f "$wrapper_root/package.xml" ]]; then
  had_manifest=1
  backup="$(mktemp)"
  cp "$wrapper_root/package.xml" "$backup"
fi
restore_manifest() {
  if [[ "$had_manifest" -eq 1 ]]; then
    cp "$backup" "$wrapper_root/package.xml"
    rm -f "$backup"
  else
    rm -f "$wrapper_root/package.xml"
  fi
}
trap restore_manifest EXIT
cp "$manifest" "$wrapper_root/package.xml"

cd "$workspace_root"
if [[ "$edition" == "ROS1" ]]; then
  catkin_make --cmake-args -DROS_EDITION=ROS1 -DORB_SLAM3_ROOT="$core_root"
else
  colcon build --base-paths "$workspace_root/src" --packages-select orb_slam3 --cmake-args \
    -DROS_EDITION=ROS2 -DORB_SLAM3_ROOT="$core_root"
fi
