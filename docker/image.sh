#!/usr/bin/env bash
# Builds, publishes and fetches the images named in docker/image.env. TARGET is base, s1 or all (default all; s1
# builds on top of base). Push needs a docker login to the registry first.
#   docker/image.sh build [TARGET]
#   docker/image.sh push [TARGET]
#   docker/image.sh pull [TARGET]
#   docker/image.sh save [TARGET]   # docker/images/<name>_<version>.tar.gz, for a host without the registry
#   docker/image.sh load [TARGET]   # from docker/images/
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$REPO_ROOT/docker/image.env"

INFO() { echo -e "\033[1;32m[image]\033[0m $*" >&2; }

image_of() {
  case $1 in
    base) echo "$BASE_IMAGE" ;;
    s1) echo "$S1_IMAGE" ;;
    *) echo "unknown target $1 (base, s1 or all)" >&2; exit 1 ;;
  esac
}

# The Dockerfile digest the image was built from, kept as a label so that docker/run.sh can tell a stale image.
dockerfile_sha() {
  sha256sum "$REPO_ROOT/docker/Dockerfile" | cut -c1-16
}

tarball_of() {
  local image=$1
  local name=${image##*/}
  echo "$REPO_ROOT/docker/images/${name/:/_}.tar.gz"
}

run() {
  local action=$1 target=$2 image
  image=$(image_of "$target")
  case $action in
    build)
      INFO "building $image"
      docker build --target "$target" --label "eef.dockerfile_sha=$(dockerfile_sha)" -t "$image" "$REPO_ROOT/docker"
      ;;
    push)
      INFO "pushing $image"
      docker push "$image"
      ;;
    pull)
      INFO "pulling $image"
      docker pull "$image"
      ;;
    save)
      mkdir -p "$REPO_ROOT/docker/images"
      INFO "saving $image to $(tarball_of "$image")"
      docker save "$image" | gzip > "$(tarball_of "$image")"
      ;;
    load)
      INFO "loading $(tarball_of "$image")"
      gunzip -c "$(tarball_of "$image")" | docker load
      ;;
    *)
      echo "unknown action $action (build, push, pull, save or load)" >&2
      exit 1
      ;;
  esac
}

ACTION=${1:?usage: docker/image.sh build|push|pull|save|load [base|s1|all]}
TARGETS=${2:-all}
if [[ $TARGETS == all ]]; then
  TARGETS="base s1"
fi
for target in $TARGETS; do
  run "$ACTION" "$target"
done
