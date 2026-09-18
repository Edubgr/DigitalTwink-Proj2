#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u

export DISPLAY=:99
STARTUP_LAUNCH="${STARTUP_LAUNCH:-gazebo}"

Xvfb ${DISPLAY} -screen 0 1280x720x24 -ac +extension RANDR >/tmp/xvfb.log 2>&1 &
xvfb_pid=$!

openbox >/tmp/openbox.log 2>&1 &
openbox_pid=$!

x11vnc -display ${DISPLAY} -forever -shared -nopw -rfbport 5900 >/tmp/x11vnc.log 2>&1 &
x11vnc_pid=$!

websockify --web=/usr/share/novnc/ 6080 localhost:5900 >/tmp/websockify.log 2>&1 &
websockify_pid=$!

cleanup() {
    kill ${websockify_pid} ${x11vnc_pid} ${openbox_pid} ${xvfb_pid} 2>/dev/null || true
}

trap cleanup EXIT

for _ in $(seq 1 20); do
    if xdpyinfo -display ${DISPLAY} >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

case "${STARTUP_LAUNCH}" in
    gazebo)
        ros2 launch problemas2_urdf_description gazebo.launch.py &
        ;;
    display)
        ros2 launch problemas2_urdf_description display.launch.py rviz:=true gui:=true &
        ;;
    headless)
        ros2 launch problemas2_urdf_description display.launch.py rviz:=false gui:=false &
        ;;
    *)
        echo "Unknown STARTUP_LAUNCH=${STARTUP_LAUNCH}" >&2
        exit 1
        ;;
esac

ros_pid=$!

wait ${ros_pid}