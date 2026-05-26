#!/bin/bash

# 1. Handle XAUTHORITY gracefully (Fixes the "empty section between colons" error)
if [ -z "$XAUTHORITY" ]; then
    XAUTH_PATH="$HOME/.Xauthority"
else
    XAUTH_PATH="$XAUTHORITY"
fi

# 2. Try xhost if available, but silence errors if running over SSH
if command -v xhost &> /dev/null; then
    xhost +local:root > /dev/null 2>&1
fi

# 3. Remove existing container if it exists
if sudo docker ps -a --format '{{.Names}}' | grep -qx wildbot; then
  echo "Removing existing wildbot container..."
  sudo docker rm -f wildbot
fi

# 4. Define common arguments for all architectures
COMMON_ARGS=(
  -it
  --name wildbot
  --user root
  --rm
  --network host
  --ipc=host
  --shm-size=8g
  -e DISPLAY=$DISPLAY
  -e ROS_DOMAIN_ID=0
  -e XAUTHORITY=/root/.Xauthority
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro
  -v "$XAUTH_PATH:/root/.Xauthority:ro"
  -v "$(pwd)":/ros2_ws
)

# 5. Launch with the appropriate hardware acceleration
# Check for NVIDIA
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected. Launching with NVIDIA Container Toolkit..."
    sudo docker run "${COMMON_ARGS[@]}" \
      --runtime=nvidia \
      --gpus all \
      my-car-env

# Check for AMD
elif [ -c /dev/kfd ]; then
    echo "AMD GPU detected. Launching with ROCm/KFD..."
    sudo docker run "${COMMON_ARGS[@]}" \
      --device=/dev/kfd \
      --device=/dev/dri \
      --group-add=video \
      --group-add=110 \
      --security-opt seccomp=unconfined \
      --env-file .env \
      -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
      my-car-env

# Fallback to CPU
else
    echo "No supported GPU found. Running in CPU mode."
    sudo docker run "${COMMON_ARGS[@]}" \
      my-car-env
fi