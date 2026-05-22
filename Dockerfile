# Use the official ROS 2 Humble image
FROM ros:humble-ros-base

ARG GPU_TYPE=nvidia

# Set the working directory inside the container
WORKDIR /ros2_ws

# Install system tools, ROS 2 build tools, and Python PIP
RUN apt-get update && apt-get install -y \
    nano \
    python3-colcon-common-extensions \
    git \
    python3-pip \
    python3-venv \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-bridge \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*


RUN if [ "$GPU_TYPE" = "amd" ]; then \
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/rocm6.2; \
    else \
        pip3 install torch torchvision torchaudio; \
    fi

# Copy the requirements file into the container
COPY requirements.txt /ros2_ws/

# Install the Python dependencies
RUN pip3 install -r requirements.txt

# Automatically source the ROS 2 environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/setup.bash" >> ~/.bashrc

# Clone and build RF2O laser odometry outside /ros2_ws so the host volume mount
# does not overwrite it at runtime.
RUN git clone -b humble-devel https://github.com/Adlink-ROS/rf2o_laser_odometry.git /opt/rf2o_laser_odometry && \
    mkdir -p /opt/rf2o_ws/src && \
    ln -s /opt/rf2o_laser_odometry /opt/rf2o_ws/src/rf2o_laser_odometry && \
    bash -c "source /opt/ros/humble/setup.bash && \
             cd /opt/rf2o_ws && \
             colcon build --cmake-args -DBoost_NO_BOOST_CMAKE=ON" && \
    echo "source /opt/rf2o_ws/install/setup.bash" >> /root/.bashrc

# Set the default command
CMD ["bash"]
