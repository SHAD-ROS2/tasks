FROM ghcr.io/shad-ros2/tasks@sha256:604b85d31cfa14366b7f4c81a6cd08110b37cfe4f3c9804f7dd14be3860b929d
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs ros-jazzy-tf-transformations \
    python3-pytest python3-pytest-timeout python3-pil \
    xvfb x11vnc novnc websockify openbox x11-utils ffmpeg \
    && mkdir -p /opt/course && dpkg-query -W > /opt/course/day03-packages.txt \
    && rm -rf /var/lib/apt/lists/*
COPY docker/desktop.sh /usr/local/bin/course-desktop
RUN chmod +x /usr/local/bin/course-desktop
USER 1000:1000
WORKDIR /workspace
CMD ["/usr/local/bin/course-desktop"]
