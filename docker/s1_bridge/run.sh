#!/usr/bin/env bash
# Runs a command (default: the sidecar of the S1 replay) in the s1_bridge image, building it when missing. Host
# networking reaches the robot (DDS) and the replay (UDP on 127.0.0.1).
#   docker/s1_bridge/run.sh                                    # python3 scripts/s1_bridge.py --config config/s1.yaml
#   docker/s1_bridge/run.sh python3 -c "import astribot_sdk"
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
IMAGE=${IMAGE:-eef-trajectory-benchmark-s1-bridge:latest}

if [[ -z $(docker images -q "$IMAGE") ]]; then
  docker build -t "$IMAGE" "$REPO_ROOT/docker/s1_bridge"
fi

if [[ $# -eq 0 ]]; then
  set -- python3 scripts/s1_bridge.py --config config/s1.yaml
fi

TTY_FLAGS=()
if [[ -t 0 ]]; then
  TTY_FLAGS=(-it)
fi

# The SDK logs under the home directory of the passwd entry of the user, which the container lacks.
USER_HOME=$(getent passwd "$(id -u)" | cut -d: -f6)
mkdir -p "$REPO_ROOT/runs/.s1_bridge_home"

NAME_FLAGS=()
if [[ -n ${CONTAINER_NAME:-} ]]; then
  NAME_FLAGS=(--name "$CONTAINER_NAME")
fi

exec docker run --rm "${TTY_FLAGS[@]}" "${NAME_FLAGS[@]}" \
  --network host \
  --user "$(id -u):$(id -g)" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e HOME="$USER_HOME" \
  -v "$REPO_ROOT/runs/.s1_bridge_home:$USER_HOME" \
  -v "$REPO_ROOT:/workspace" \
  -w /workspace \
  "$IMAGE" "$@"
