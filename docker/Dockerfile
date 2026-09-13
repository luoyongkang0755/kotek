FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
        python3-pip \
        git \
        ros-jazzy-moveit \
        ros-jazzy-moveit-ros-planning-interface \
        ros-jazzy-moveit-ros-move-group \
        ros-jazzy-moveit-simple-controller-manager \
        ros-jazzy-moveit-setup-assistant \
        ros-jazzy-moveit-visual-tools \
        ros-jazzy-nav2-msgs \
        ros-jazzy-tf2 \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-control-msgs \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rmw-fastrtps-cpp \
        ros-jazzy-rviz2 \
        libspnav0 \
        can-utils \
        ethtool \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# piper_sdk (real Piper arm over CAN, used as the teleop leader) + python-can.
# Installed here rather than into a venv: the container's system Python (3.12)
# is what every ROS 2 node runs under, and piper_sdk is pure Python so there is
# no ABI reason to isolate it. See kotek_teleop/scripts/piper/ for the
# standalone bench scripts this was first validated against.
RUN pip3 install --break-system-packages piper_sdk python-can

RUN rosdep init || true

WORKDIR /workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> /etc/bash.bashrc

CMD ["/bin/bash"]
