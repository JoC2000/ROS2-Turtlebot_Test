# Use ROS Humble image
FROM ros:humble-ros-base:latest

# Avoid prompts
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y\
    python3-pip \
    python3-rosdep \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    ros-humble-turtlebot4-bringup \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-ignition-bringup \
    ros-humble-turtlebot4-ignition-gui-plugins \
    ros-humble-rplidar-ros \
    ros-dev-tools && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
    && apt-get install ignition-fortress && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

# Copy ROS2 packages into the container
COPY ./package.xml .
COPY ./CMakeLists.txt .

COPY ./turtlebot_test/ ./turtlebot_test/
COPY ./src ./src/

# Install dependencies
RUN rm -rf /etc/ros/rosdep/sources.lists.d/20-default.list && \
    rosdep init && rosdep update && \
    rosdep install --from-paths . --ignore-src -y

# Build the workspace
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash"]