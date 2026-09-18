# Imagem base já traz MoveIt 2 completo (OMPL, KDL, move_group,
# setup assistant). Evita baixar ~3 GB de pacotes via apt.
FROM moveit/moveit2:humble-release

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws

# A imagem carrega uma chave GPG expirada do repositório ROS, que faz
# qualquer apt-get update falhar. Como tudo que falta aqui vem dos
# repositórios do Ubuntu (colcon e o ROS já estão na imagem), a lista
# do ROS é removida antes do update.
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update && apt-get install -y \
    xauth \
    x11-utils \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    openbox \
    && rm -rf /var/lib/apt/lists/*

COPY problemas2_urdf_description /ws/src/problemas2_urdf_description
COPY problemas2_moveit_config    /ws/src/problemas2_moveit_config
COPY moveit_bridge               /ws/src/moveit_bridge

COPY docker/start_viz.sh    /usr/local/bin/start_viz.sh
COPY docker/start_moveit.sh /usr/local/bin/start_moveit.sh

RUN chmod +x /usr/local/bin/start_viz.sh /usr/local/bin/start_moveit.sh

RUN bash -lc "source /opt/ros/humble/setup.bash && \
    cd /ws && \
    colcon build --symlink-install"

# 8081 = ponte HTTP do MoveIt   5900/6080 = VNC/noVNC do RViz
EXPOSE 5900 6080 8081

CMD ["/usr/local/bin/start_moveit.sh"]
