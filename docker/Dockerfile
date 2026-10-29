# Build and runtime environment of the benchmark. Pinocchio and hpp-fcl are built from pinned tags so that the IK
# controller sees the same model library as the reference pipeline (Pinocchio 3.7.0 with hpp-fcl 2.x).
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG HPP_FCL_VERSION=v2.4.5
ARG PINOCCHIO_VERSION=v3.7.0
ARG JOBS=8
# Pinocchio translation units need several GB each, a higher parallelism runs out of memory.
ARG PINOCCHIO_JOBS=4

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git ca-certificates pkg-config \
      libeigen3-dev libboost-all-dev liburdfdom-dev liboctomap-dev libassimp-dev libqhull-dev \
      libgoogle-glog-dev libyaml-cpp-dev libgtest-dev \
      python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${HPP_FCL_VERSION} --recursive https://github.com/humanoid-path-planner/hpp-fcl.git /tmp/hpp-fcl \
    && cmake -S /tmp/hpp-fcl -B /tmp/hpp-fcl/build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF \
         -DBUILD_TESTING=OFF -DHPP_FCL_HAS_QHULL=ON \
    && cmake --build /tmp/hpp-fcl/build -j${JOBS} && cmake --install /tmp/hpp-fcl/build \
    && rm -rf /tmp/hpp-fcl

RUN git clone --depth 1 --branch ${PINOCCHIO_VERSION} --recursive https://github.com/stack-of-tasks/pinocchio.git /tmp/pinocchio \
    && cmake -S /tmp/pinocchio -B /tmp/pinocchio/build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF \
         -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARK=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON \
         -DBUILD_WITH_URDF_SUPPORT=ON \
    && cmake --build /tmp/pinocchio/build -j${PINOCCHIO_JOBS} && cmake --install /tmp/pinocchio/build \
    && rm -rf /tmp/pinocchio && ldconfig

# Python side: delivery CSV synthesis (FK with the same Pinocchio version) and evaluation. The pin wheel does not pin
# its cmeel dependencies (urdfdom, tinyxml2, ...) to the versions it was built against, so they are resolved as of its
# release date; the other packages do not depend on them.
RUN pip3 install --no-cache-dir uv \
    && uv pip install --system --no-cache --exclude-newer 2025-06-15 pin==3.7.0 \
    && uv pip install --system --no-cache scipy matplotlib "rosbags>=0.11"

# 3D playback of the trajectory solve in the browser (pinocchio MeshcatVisualizer), a layer of its own so that adding
# it does not rebuild the ones above.
RUN uv pip install --system --no-cache --exclude-newer 2025-06-15 meshcat

WORKDIR /workspace
