#!/usr/bin/env bash
# One entry point: builds the image (first time) and the code (when it changed), then runs the interactive
# replay -> evaluation -> summary in the container. Arguments go to scripts/benchmark.py, see --help.
#   ./benchmark.sh
#   ./benchmark.sh --mode dry --case case02_fold_clothes_spd50 --runs 3 -y
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$REPO_ROOT"

INFO() { echo -e "\033[1;32m[benchmark]\033[0m $*"; }

INFO "building the code (incremental), log in build/build.log"
mkdir -p build
if ! docker/run.sh bash -c 'docker/build.sh > build/build.log 2>&1'; then
  echo -e "\033[1;31m[benchmark] build failed:\033[0m"
  grep -E "error|Error" build/build.log | head -20
  exit 1
fi

# The S1 real robot is driven by the SDK sidecar in its own image (docker/s1_bridge), which the container cannot
# start: scripts/benchmark.py writes the config into runs/.s1_bridge_request when it needs the sidecar, and this
# watcher starts it on the host (log in runs/s1_bridge.log). The replay waits for it.
REQUEST=runs/.s1_bridge_request
mkdir -p runs
rm -f "$REQUEST"
(
  while kill -0 $$ 2> /dev/null; do
    if [[ -f $REQUEST ]]; then
      config=$(cat "$REQUEST")
      rm -f "$REQUEST"
      docker rm -f eef-benchmark-s1-bridge > /dev/null 2>&1 || true
      CONTAINER_NAME=eef-benchmark-s1-bridge docker/s1_bridge/run.sh \
        python3 scripts/s1_bridge.py --config "$config" > runs/s1_bridge.log 2>&1 < /dev/null &
    fi
    sleep 0.2
  done
) &
WATCHER=$!
trap 'kill $WATCHER 2> /dev/null; docker rm -f eef-benchmark-s1-bridge > /dev/null 2>&1 || true' EXIT

docker/run.sh python3 scripts/benchmark.py "$@"
