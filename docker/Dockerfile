# Build and runtime environment of the benchmark, two images (docker/image.env names them, docker/image.sh builds and
# publishes them, docker/run.sh pulls them):
#   base  the benchmark itself (Marvin, AR5, the S1 solve and dry run)
#   s1    base plus ROS 2 Humble and the dependencies of the Astribot SDK, only for the S1 real robot
# Pinocchio and hpp-fcl are built from pinned tags so that the IK controller sees the same model library as the
# reference pipeline (Pinocchio 3.7.0 with hpp-fcl 2.x).
FROM ubuntu:22.04 AS base

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
RUN pip3 install --no-cache-dir uv==0.13.6 \
    && uv pip install --system --no-cache --exclude-newer 2025-06-15 pin==3.7.0 \
    && uv pip install --system --no-cache scipy==1.15.3 matplotlib==3.10.9 rosbags==0.11.9

# 3D playback of the trajectory solve in the browser (pinocchio MeshcatVisualizer), a layer of its own so that adding
# it does not rebuild the ones above.
RUN uv pip install --system --no-cache --exclude-newer 2025-06-15 meshcat==0.3.2

WORKDIR /workspace

# Runtime of the S1 sidecar scripts/s1_bridge.py: ROS 2 Humble and the dependencies of the Astribot SDK, which is not
# part of the image (docker/run.sh mounts it, docker/s1_sdk_env.sh sets up its environment). The SDK ships compiled
# Python 3.10 modules. Shared libraries: its install.sh plus what its compiled modules load (ldd over the SDK; not the
# cp38 module, the second drake copy and the Python bindings of its pinocchio, which the SDK does not import).
FROM base AS s1

# Build arguments do not carry over from the base stage; without this tzdata waits for a time zone answer.
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_SOURCE_VERSION=1.3.0

RUN apt-get update && apt-get install -y --no-install-recommends curl \
    && curl -fsSL -o /tmp/ros2-apt-source.deb \
         "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.jammy_all.deb" \
    && apt-get install -y /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb \
    && apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base ros-humble-tf-transformations ros-humble-rmw-fastrtps-cpp \
      python3-venv iproute2 \
      libgsl27 libgsl-dev libmumps-seq-5.4 libmumps-seq-dev \
      libopengl0 libglx0 libboost-filesystem1.74.0 libboost-python1.74.0 libboost-serialization1.74.0 \
      liboctomap1.9 libassimp5 \
    && rm -rf /var/lib/apt/lists/*

# The Python packages of the SDK in a venv of their own: tf_transformations still uses np.float, hence numpy 1.22,
# while the benchmark scripts keep the newer numpy of the base layers. The venv sees the system packages (ROS, yaml)
# behind its own.
RUN python3 -m venv --system-site-packages /opt/astribot_venv \
    && /opt/astribot_venv/bin/pip install --no-cache-dir \
      numpy==1.22.4 scipy==1.10.1 matplotlib==3.7.5 opencv-python-headless==4.10.0.82 tabulate==0.9.0 \
      h5py==3.11.0 filterpy==1.4.5 colorama==0.4.6 pyyaml==6.0.1

# Log folders the compiled SDK modules write into, for any uid.
RUN mkdir -p /opt/astribot_ros/log /opt/astribot_ros/robot_config/log && chmod -R 777 /opt/astribot_ros
