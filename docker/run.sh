#!/usr/bin/env bash
# Builds the image when missing and runs a command (default: an interactive shell) inside it with the repository
# mounted at /workspace. Host networking reaches the controller, SYS_NICE and rtprio allow the SCHED_FIFO loop. The
# host passwd / group files give the host uid its user name in the container, the host time zone its local time.
# TARGET picks the image of docker/image.env: base (default), or s1 for the S1 real robot, which also mounts the
# Astribot SDK (ASTRIBOT_SDK_DIR, default third_party/astribot_sdk) at /opt/astribot_sdk. A missing image is pulled
# from the registry, else loaded from docker/images/, else built (slow).
#   docker/run.sh                       # shell
#   docker/run.sh docker/build.sh       # build the C++ code into build/
#   TARGET=s1 docker/run.sh docker/s1_sdk_env.sh python3 -c "import astribot_sdk"
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$REPO_ROOT/docker/image.env"
TARGET=${TARGET:-base}
case $TARGET in
  base) IMAGE=${IMAGE:-$BASE_IMAGE} ;;
  s1) IMAGE=${IMAGE:-$S1_IMAGE} ;;
  *) echo "unknown TARGET $TARGET (base or s1)" >&2; exit 1 ;;
esac

if [[ -z $(docker images -q "$IMAGE") ]]; then
  if ! "$REPO_ROOT/docker/image.sh" pull "$TARGET" && ! "$REPO_ROOT/docker/image.sh" load "$TARGET" 2> /dev/null; then
    echo -e "\033[1;33m[image]\033[0m $IMAGE neither in the registry nor in docker/images/, building it (slow)" >&2
    if [[ $TARGET == s1 && -z $(docker images -q "$BASE_IMAGE") ]]; then
      "$REPO_ROOT/docker/image.sh" build base
    fi
    "$REPO_ROOT/docker/image.sh" build "$TARGET"
  fi
fi
BUILT_FROM=$(docker inspect -f '{{index .Config.Labels "eef.dockerfile_sha"}}' "$IMAGE" 2> /dev/null || true)
if [[ -n $BUILT_FROM && $BUILT_FROM != $(sha256sum "$REPO_ROOT/docker/Dockerfile" | cut -c1-16) ]]; then
  echo -e "\033[1;33m[image]\033[0m $IMAGE was built from another docker/Dockerfile: bump VERSION in" \
          "docker/image.env and rebuild (docker/image.sh build)" >&2
fi

TTY_FLAGS=()
if [[ -t 0 ]]; then
  TTY_FLAGS=(-it)
fi

HOME_FLAGS=(-e HOME=/tmp)
SDK_FLAGS=()
if [[ $TARGET == s1 ]]; then
  # The SDK logs under the home directory of the passwd entry of the user, which the container lacks.
  USER_HOME=$(getent passwd "$(id -u)" | cut -d: -f6)
  mkdir -p "$REPO_ROOT/runs/.s1_home"
  HOME_FLAGS=(-e HOME="$USER_HOME" -v "$REPO_ROOT/runs/.s1_home:$USER_HOME")
  SDK_DIR=${ASTRIBOT_SDK_DIR:-$REPO_ROOT/third_party/astribot_sdk}
  if [[ -d $SDK_DIR ]]; then
    SDK_FLAGS=(-v "$(cd "$SDK_DIR" && pwd):/opt/astribot_sdk")
  fi
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
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  "${HOME_FLAGS[@]}" \
  -v "$REPO_ROOT:/workspace" \
  "${SDK_FLAGS[@]}" \
  -w /workspace \
  "$IMAGE" "${@:-bash}"
