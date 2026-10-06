# syntax=docker/dockerfile:1
#
# docker build -t hold_and_weld .
# xhost +local:docker
# docker run -it --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#   --device /dev/dri hold_and_weld ros2 launch hold_and_weld_bringup system_bringup.launch.py
#
# pythonocc-core's SWIG modules take several GB of RAM per compile job; lower JOBS
# (docker build --build-arg JOBS=4 ...) if the build is killed.

ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS occt

ARG JOBS=""
ARG OCCT_DEPS="build-essential cmake git swig python3-dev python3-numpy tcl-dev tk-dev \
  libfreetype-dev libgl1-mesa-dev libxi-dev libxmu-dev"

RUN apt-get update && apt-get install -y --no-install-recommends ${OCCT_DEPS} \
  && rm -rf /var/lib/apt/lists/*

# apt only ships OCCT 7.6, and pythonocc-core is not on pip; both are built from
# source, pythonocc-core against the same OCCT.
RUN git clone --depth 1 -b V7_9_3 https://github.com/Open-Cascade-SAS/OCCT.git /tmp/occt \
  && cmake -S /tmp/occt -B /tmp/occt/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/opt/occt \
  && cmake --build /tmp/occt/build -j${JOBS:-$(nproc)} \
  && cmake --install /tmp/occt/build \
  && rm -rf /tmp/occt

# The glTF wrappers include OCCT headers that include rapidjson, even though OCCT
# itself was built without it.
RUN apt-get update && apt-get install -y --no-install-recommends rapidjson-dev \
  && rm -rf /var/lib/apt/lists/*

# 7.9.0 asks for SWIG 4.2.1, Ubuntu 24.04 ships 4.2.0, and 4.2.0 builds it fine.
RUN git clone --depth 1 -b 7.9.0 https://github.com/tpaviot/pythonocc-core.git /tmp/pythonocc \
  && sed -i 's/find_package(SWIG 4.2.1/find_package(SWIG 4.2.0/' /tmp/pythonocc/CMakeLists.txt \
  && cmake -S /tmp/pythonocc -B /tmp/pythonocc/build -DCMAKE_BUILD_TYPE=Release \
    -DOCCT_INCLUDE_DIR=/opt/occt/include/opencascade \
    -DOCCT_LIBRARY_DIR=/opt/occt/lib \
    -DPYTHONOCC_INSTALL_DIRECTORY=/opt/pythonocc/OCC \
    -DPYTHONOCC_MESHDS_NUMPY=ON \
  && cmake --build /tmp/pythonocc/build -j${JOBS:-$(nproc)} \
  && cmake --install /tmp/pythonocc/build \
  && rm -rf /tmp/pythonocc


FROM ros:${ROS_DISTRO}-ros-base

ARG OCCT_DEPS="build-essential cmake git swig python3-dev python3-numpy tcl-dev tk-dev \
  libfreetype-dev libgl1-mesa-dev libxi-dev libxmu-dev"

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends ${OCCT_DEPS} \
    python3-pip libgl1-mesa-dri \
  && rm -rf /var/lib/apt/lists/*

COPY --from=occt /opt/occt /opt/occt
COPY --from=occt /opt/pythonocc /opt/pythonocc

ENV CASROOT=/opt/occt
ENV CMAKE_PREFIX_PATH=/opt/occt
ENV LD_LIBRARY_PATH=/opt/occt/lib
ENV PYTHONPATH=/opt/pythonocc

WORKDIR /ws

# Manifests first, so a source change does not redo the dependency install.
COPY requirements.txt src/hold_and_weld/
COPY hold_and_weld_application/package.xml src/hold_and_weld/hold_and_weld_application/
COPY hold_and_weld_bringup/package.xml src/hold_and_weld/hold_and_weld_bringup/
COPY hold_and_weld_description/package.xml src/hold_and_weld/hold_and_weld_description/
COPY hold_and_weld_gripper_sampler/package.xml src/hold_and_weld/hold_and_weld_gripper_sampler/
COPY hold_and_weld_planning/package.xml src/hold_and_weld/hold_and_weld_planning/

RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
  && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
  && pip install --break-system-packages -r src/hold_and_weld/requirements.txt \
  && rm -rf /var/lib/apt/lists/*

COPY . src/hold_and_weld/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY --chmod=755 <<"EOF" /ros_entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/$ROS_DISTRO/setup.bash
source /ws/install/setup.bash
exec "$@"
EOF

CMD ["bash"]
