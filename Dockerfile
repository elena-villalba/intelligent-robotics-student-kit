FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rviz2 \
    ros-humble-controller-manager \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-trajectory-msgs \
    ros-humble-velocity-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-gazebo-ros2-control-demos \
    ros-humble-urdf-tutorial \
    ros-humble-nav2-msgs \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    mesa-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# Create student user
# --------------------------------------------------

ARG USERNAME=student
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} \
               --gid ${USER_GID} \
               --create-home \
               --shell /bin/bash \
               ${USERNAME}

# Source ROS automatically in every terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo '[ -f ~/ir_ws/install/setup.bash ] && source ~/ir_ws/install/setup.bash' \
       >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}

USER ${USERNAME}

CMD ["bash"]