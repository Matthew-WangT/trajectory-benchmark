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

# The S1 real robot needs the s1 image (ROS 2 and the Astribot SDK runtime), pulled only then: scripts/benchmark.py
# in the base image stops with exit code 75 after the menus and leaves the picked options in runs/.s1_rerun, which
# then run again in the s1 image.
code=0
docker/run.sh python3 scripts/benchmark.py "$@" || code=$?
if [[ $code -eq 75 && -f runs/.s1_rerun ]]; then
  mapfile -t ARGS < runs/.s1_rerun
  rm -f runs/.s1_rerun
  INFO "S1 real robot: switching to the s1 image"
  TARGET=s1 exec docker/run.sh python3 scripts/benchmark.py "${ARGS[@]}"
fi
exit $code
