#!/bin/bash
# Container entrypoint: source ROS 2 and the workspace, make sure Gazebo has a
# display to render the camera sensor on, then run the given command.
#
# It can be called again inside a running container to get the same environment
# for a one-off command:
#   docker compose exec sim /ros_entrypoint.sh ros2 topic list
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ws/install/setup.bash

XVFB_DISPLAY=99

# True when the Xvfb started by an earlier call of this script is still running.
# The lock file survives a container restart, so the PID is checked against the
# process name as well.
xvfb_running() {
    local lock="/tmp/.X${XVFB_DISPLAY}-lock" pid
    [ -f "${lock}" ] || return 1
    pid="$(tr -d '[:space:]' < "${lock}")"
    [ -n "${pid}" ] && [ "$(cat "/proc/${pid}/comm" 2>/dev/null)" = "Xvfb" ]
}

# Gazebo Classic renders camera images through OGRE, which needs an X display even
# when gzclient is not started. When the host does not pass one in (headless use),
# start a virtual framebuffer; Mesa then renders in software. Without it the front
# camera publishes nothing and fine alignment cannot see the fiducial marker.
# Set SCR_XVFB=0 to skip this.
if [ -z "${DISPLAY:-}" ] && [ "${SCR_XVFB:-1}" != "0" ]; then
    export DISPLAY=":${XVFB_DISPLAY}"
    if ! xvfb_running; then
        rm -f "/tmp/.X${XVFB_DISPLAY}-lock" "/tmp/.X11-unix/X${XVFB_DISPLAY}"
        Xvfb "${DISPLAY}" -screen 0 1280x1024x24 -nolisten tcp > /dev/null 2>&1 &
        for _ in $(seq 50); do
            [ -S "/tmp/.X11-unix/X${XVFB_DISPLAY}" ] && break
            sleep 0.1
        done
        if [ ! -S "/tmp/.X11-unix/X${XVFB_DISPLAY}" ]; then
            echo "entrypoint: Xvfb did not start; Gazebo camera images will be unavailable" >&2
        fi
    fi
fi

exec "$@"
