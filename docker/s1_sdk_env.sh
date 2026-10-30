#!/usr/bin/env bash
# Runs a command in the environment of the Astribot SDK (its env.sh, without writing into the SDK), inside the s1
# image: docker/run.sh mounts the SDK at /opt/astribot_sdk, python3 is the one of the SDK venv. Only this process
# and its children see the SDK libraries, the replay keeps its own.
#   docker/s1_sdk_env.sh python3 scripts/s1_bridge.py --config config/s1.yaml
set -eo pipefail

SDK_ROOT=${SDK_ROOT:-/opt/astribot_sdk}
if [[ ! -d /opt/ros/humble ]]; then
  echo "[s1_sdk_env] not the s1 image: run the benchmark with TARGET=s1, see docs/replay.md" >&2
  exit 1
fi
if [[ ! -d "$SDK_ROOT/astribot_sdk" ]]; then
  echo "[s1_sdk_env] no Astribot SDK at $SDK_ROOT: put it at third_party/astribot_sdk or set ASTRIBOT_SDK_DIR," \
       "see docs/replay.md" >&2
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
# for astribot_sdk/core/common, outside the SDK.
META_DIR=/tmp/s1_bridge_pkgs
mkdir -p "$META_DIR"
ln -sfn "$SDK_ROOT/astribot_sdk/core/common" "$META_DIR/meta"

COMMON="$SDK_ROOT/astribot_sdk/core/common"
export PATH="/opt/astribot_venv/bin:$PATH"
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
