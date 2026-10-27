#!/usr/bin/env bash
# Builds the image when missing and runs a command (default: an interactive shell) inside it with the repository
# mounted at /workspace. Host networking reaches the controller, SYS_NICE and rtprio allow the SCHED_FIFO loop. The
# host passwd / group files give the host uid its user name in the container.
#   docker/run.sh                       # shell
#   docker/run.sh docker/build.sh       # build the C++ code into build/
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=${IMAGE:-eef-trajectory-benchmark:latest}

if [[ -z $(docker images -q "$IMAGE") ]]; then
  docker build -t "$IMAGE" "$REPO_ROOT/docker"
fi

TTY_FLAGS=()
if [[ -t 0 ]]; then
  TTY_FLAGS=(-it)
fi

mkdir -p "$REPO_ROOT/runs/.cppad_cache"
exec docker run --rm "${TTY_FLAGS[@]}" \
  --network host \
  --cap-add SYS_NICE \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  --user "$(id -u):$(id -g)" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -e HOME=/tmp \
  -v "$REPO_ROOT:/workspace" \
  -w /workspace \
  "$IMAGE" "${@:-bash}"
