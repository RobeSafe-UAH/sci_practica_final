# syntax=docker/dockerfile:1
# This official image provides native amd64 and arm64 variants.
FROM ros:jazzy-ros-base AS dependencies

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-mvsim \
        ros-jazzy-rclpy \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-tf2-ros \
        ros-jazzy-launch-ros \
        ros-jazzy-rmw-fastrtps-cpp \
        python3-pil python3-yaml \
        xvfb xauth x11-utils fluxbox x11vnc novnc websockify \
        libgl1-mesa-dri libglx-mesa0 fonts-dejavu-core util-linux \
    && rm -rf /var/lib/apt/lists/*

# Nothing is written to the host checkout, so a fixed runtime user is sufficient.
RUN groupadd --gid 10001 sci \
    && useradd --uid 10001 --gid sci --create-home --shell /bin/bash sci

FROM dependencies AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions build-essential cmake librapidxml-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/sci_ws
COPY src/ ./src/
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --merge-install --packages-select sci_start_gate sci_mvsim \
        --event-handlers console_direct+

# Only the installed application crosses into the final image. Build sources,
# build/log directories remain outside it.
FROM dependencies AS runtime
COPY --from=builder /opt/sci_ws/install/ /opt/sci_ws/install/
COPY LICENSE /opt/sci/
COPY docker/config/ /opt/sci/config/
COPY docker/start.sh docker/healthcheck.py docker/fastdds.xml /opt/sci/docker/
RUN sed -i 's/\r$//' /opt/sci/docker/start.sh \
    && chmod 0755 /opt/sci/docker/start.sh \
    && printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        'source /opt/sci_ws/install/setup.bash' >> /etc/bash.bashrc

ENV ROS_DOMAIN_ID=42 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/opt/sci/docker/fastdds.xml \
    FASTDDS_DEFAULT_PROFILES_FILE=/opt/sci/docker/fastdds.xml \
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
    SCI_CONFIG_FILE=/opt/sci/config/scenario.yaml \
    SCI_HEADLESS=false \
    SCI_VNC_PORT=6080 \
    SCI_SCREEN_WIDTH=1280 \
    SCI_SCREEN_HEIGHT=800 \
    XAUTHORITY=/tmp/sci-runtime/Xauthority \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    PYTHONUNBUFFERED=1

USER sci
WORKDIR /home/sci
EXPOSE 6080
HEALTHCHECK --interval=30s --timeout=6s --start-period=60s --retries=3 \
    CMD ["python3", "/opt/sci/docker/healthcheck.py"]
ENTRYPOINT ["/opt/sci/docker/start.sh"]
CMD ["simulate"]
