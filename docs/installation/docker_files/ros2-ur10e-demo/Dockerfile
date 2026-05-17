FROM osrf/ros:jazzy-desktop

# MoveIt 2, Universal Robots driver + MoveIt config, rosbridge for compas_fab/roslibpy,
# and rmw_zenoh_cpp for Zenoh-based transport (replacing DDS).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit \
        ros-jazzy-ur \
        ros-jazzy-ur-moveit-config \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-rmw-zenoh-cpp \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY zenoh_session.json5 /etc/zenoh/session.json5
COPY update_rate.yaml /etc/compas_fab/update_rate.yaml

SHELL ["/bin/bash", "-c"]
