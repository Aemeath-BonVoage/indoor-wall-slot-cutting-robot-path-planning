# Indoor wall slot-cutting robot: ROS 2 Humble + Gazebo Classic 11 + Nav2 + this workspace.
#
# Build from the repository root (the build context must contain src/ and docs/):
#   docker build -f docker/Dockerfile -t scr-sim:humble .
# or use docker/docker-compose.yml, which sets the same context and tag.

# Stage 1: keep only the package manifests. The dependency layer of the final stage
# is then rebuilt only when a package.xml changes, not on every source edit.
FROM ros:humble AS manifests
COPY src /tmp/src
RUN find /tmp/src -type f ! -name package.xml -delete \
 && find /tmp/src -type d -empty -delete

# Stage 2: dependencies, build, runtime.
FROM ros:humble

LABEL org.opencontainers.image.title="scr-sim" \
      org.opencontainers.image.description="Indoor wall slot-cutting robot: ROS 2 Humble simulation, navigation and task execution" \
      org.opencontainers.image.source="https://github.com/Aemeath-BonVoage/indoor-wall-slot-cutting-robot-path-planning" \
      org.opencontainers.image.licenses="MIT"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

# Dependencies come from the package.xml files through rosdep (build, exec and test
# dependencies), the same set the CI workflow installs. No "-r": an unresolved key
# fails the build instead of producing an incomplete image.
#
# xvfb is the one package installed outside rosdep. Gazebo Classic renders the camera
# sensor with OGRE, which needs an X display even when gzclient is not started; in a
# container without a host display the entrypoint starts Xvfb and Mesa renders in
# software. It is a property of running headless in a container, not a dependency of
# any package, so it is not declared in a package.xml.
COPY --from=manifests /tmp/src src
# hadolint ignore=DL3008
RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
 && apt-get install -y --no-install-recommends xvfb \
 && rm -rf /var/lib/apt/lists/*

# Build the workspace with the same command as the CI workflow. build/ is kept in
# the image so that `colcon test` can run inside the container.
COPY src src
# hadolint ignore=SC1091
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
 && colcon build --event-handlers console_cohesion+

# Experiment data and scripts. Some unit tests read the recorded runs under
# docs/experiments (time-model calibration data, the example slot plan), and the
# reports' reproduction scripts live there. Copied after the build so that editing
# the documentation does not invalidate the build layer.
COPY docs docs

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod 0755 /ros_entrypoint.sh \
 && echo "source /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
