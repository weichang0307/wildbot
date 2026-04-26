# Use the official ROS 2 Humble image
FROM ros:humble-ros-base

# Set the working directory inside the container
WORKDIR /ros2_ws

# Install system tools, ROS 2 build tools, and Python PIP
RUN apt-get update && apt-get install -y \
    nano \
    python3-colcon-common-extensions \
    git \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Copy the requirements file into the container
COPY requirements.txt /ros2_ws/

# Install the Python dependencies
RUN pip3 install -r requirements.txt

# Automatically source the ROS 2 environment variables
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["bash"]