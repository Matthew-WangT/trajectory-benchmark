#!/usr/bin/env bash
# Environment of the Astribot SDK in third_party/astribot_sdk (its env.sh, without writing into the submodule), then
# runs the command. SDK_ROOT can point elsewhere.
set -eo pipefail

SDK_ROOT=${SDK_ROOT:-/workspace/third_party/astribot_sdk}
if [[ ! -d "$SDK_ROOT/astribot_sdk" ]]; then
  echo "[s1_bridge] $SDK_ROOT is empty, run: git submodule update --init third_party/astribot_sdk" >&2
  exit 1
fi

# The setup scripts of the SDK return non-zero for the optional /opt/astribot_ros packages of the robot PC.
set +e
source /opt/ros/humble/setup.bash
for setup in "$SDK_ROOT/third_party/software/setup.bash" \
             "$SDK_ROOT/astribot_msgs/share/astribot_msgs/local_setup.bash" \
             "$SDK_ROOT/third_party/third_pkg/local_setup.bash"; do
  [[ -f "$setup" ]] && source "$setup" > /dev/null
done
set -e

# The compiled whole-body control imports meta.whole_body_control... by the vendor source tree name: a package "meta"
# for astribot_sdk/core/common, outside the submodule.
META_DIR=/tmp/s1_bridge_pkgs
mkdir -p "$META_DIR"
ln -sfn "$SDK_ROOT/astribot_sdk/core/common" "$META_DIR/meta"

COMMON="$SDK_ROOT/astribot_sdk/core/common"
export PYTHONPATH="$COMMON:$SDK_ROOT:$SDK_ROOT/third_party/astribot_ros_middleware_py:$META_DIR:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$COMMON/robotics_library_py:$COMMON/whole_body_control/third_party:$SDK_ROOT/third_party/drake/lib"
export ROBOT_TYPE=S1
export ASTRIBOT_SDK_ROOT="$SDK_ROOT"
# The compiled modules build the model paths of the yaml files (/astribot_config/...) on this.
export ASTRIBOT_COMMON_ROOT="$SDK_ROOT"
export ROS_HOME=/tmp/ros_home
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-25}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# DDS on the 192.168.0.x interface of the robot network only, as the SDK env.sh does.
LOCAL_IP=$(ip -4 addr show | awk '/inet 192\.168\.0\./ {print $2}' | head -n1 | cut -d/ -f1)
if [[ -n "$LOCAL_IP" && "$LOCAL_IP" != "192.168.0.10" ]]; then
  sed "s/__LOCAL_192_IP__/${LOCAL_IP}/g" "$SDK_ROOT/config/fastdds_whitelist_192.xml.template" > /tmp/fastdds_whitelist.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_whitelist.xml
fi

exec "$@"
