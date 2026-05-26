# Use the official ROS 2 Jazzy image
FROM ros:jazzy-ros-base

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
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# Copy the requirements file into the container
COPY requirements.txt /ros2_ws/

# Install the Python dependencies with the break-system-packages flag
RUN pip3 install -r requirements.txt --break-system-packages --ignore-installed psutil

# Install PyTorch after the application dependencies so it is not replaced
# by transitive requirements such as ultralytics.
RUN if [ "$GPU_TYPE" = "amd" ]; then \
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/rocm6.2 --break-system-packages; \
    elif [ "$GPU_TYPE" = "nvidia" ]; then \
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu130 --break-system-packages; \
    else \
        pip3 install torch torchvision torchaudio --break-system-packages; \
    fi

# Automatically source the ROS 2 environment variables
# (Updated from humble to jazzy to match your base image)
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["bash"]