#!/usr/bin/env bash
# Configures and builds the C++ code into build/, run inside the container.
set -euo pipefail

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cmake -S "$REPO_ROOT" -B "$REPO_ROOT/build" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$REPO_ROOT/build" -j"$(nproc)"
