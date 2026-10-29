# Runtime of scripts/s1_bridge.py: ROS 2 Humble and the dependencies of the Astribot SDK (third_party/astribot_sdk,
# mounted from the repository, not copied). The SDK ships compiled Python 3.10 modules, hence Ubuntu 22.04.
# Dependencies: the SDK install.sh, plus the shared libraries its compiled modules load (ldd over the SDK; not the
# cp38 module, the second drake copy and the Python bindings of its pinocchio, which the SDK does not import) and
# tf_transformations, which still uses np.float, hence numpy 1.22.
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip iproute2 \
      libgsl27 libgsl-dev libmumps-seq-5.4 libmumps-seq-dev \
      ros-humble-tf-transformations ros-humble-rmw-fastrtps-cpp \
      libopengl0 libglx0 libboost-filesystem1.74.0 libboost-python1.74.0 libboost-serialization1.74.0 \
      liboctomap1.9 libassimp5 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
      numpy==1.22.4 scipy==1.10.1 matplotlib==3.7.5 opencv-python-headless==4.10.0.82 tabulate==0.9.0 \
      h5py==3.11.0 filterpy==1.4.5 colorama==0.4.6 pyyaml==6.0.1

# Log folders the compiled SDK modules write into, for any uid.
RUN mkdir -p /opt/astribot_ros/log /opt/astribot_ros/robot_config/log && chmod -R 777 /opt/astribot_ros

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
