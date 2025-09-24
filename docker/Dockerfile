ARG DISTRO=humble
FROM ros:${DISTRO}
ARG DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

USER root

# Install
RUN apt update \
  && apt upgrade -y \
  && apt install -y cmake curl git python3

# Install and configure ROS infrastructure tools
RUN apt update \
  && apt install -y python3-vcstool python3-colcon-common-extensions python3-rosdep \
  && rosdep update

# Install the dependency repositories
# Use a tmpfs mount for the workspace so as not to unnecessarily copy files into the final image
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/noether_ros
ARG INSTALL_DIR=/opt/noether_ros
RUN mkdir -p ${INSTALL_DIR}

RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/noether_ros \
  source /opt/ros/${DISTRO}/setup.bash \
  && cd ${WORKSPACE_DIR} \
  && vcs import src < src/noether_ros/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
    --skip-keys libvtk \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}

# Set the entrypoint to source the workspace
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
