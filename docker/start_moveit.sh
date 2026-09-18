#!/usr/bin/env bash
#
# Sobe o move_group em modo planejamento + a ponte HTTP.
#
# RVIZ=true habilita o RViz dentro de um Xvfb servido por noVNC em
# http://localhost:6080/vnc.html — útil para ver a cena de
# planejamento, mas não é necessário para a interface web funcionar.

set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

RVIZ="${RVIZ:-false}"
HTTP_PORT="${HTTP_PORT:-8081}"

if [ "$RVIZ" = "true" ]; then

    export DISPLAY=:1

    Xvfb :1 -screen 0 1600x900x24 &
    sleep 2

    openbox &
    x11vnc -display :1 -forever -shared -nopw -quiet &
    websockify --web=/usr/share/novnc 6080 localhost:5900 &

    echo "RViz via noVNC: http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
fi

echo "========================================"
echo " MoveIt planning bridge"
echo " HTTP: 0.0.0.0:${HTTP_PORT}"
echo " RViz: ${RVIZ}"
echo "========================================"

exec ros2 launch problemas2_moveit_config planning.launch.py \
    http_port:="${HTTP_PORT}" \
    rviz:="${RVIZ}"
