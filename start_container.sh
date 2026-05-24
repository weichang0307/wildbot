#!/bin/bash
xhost +local:root

if sudo docker ps -a --format '{{.Names}}' | grep -qx wildbot; then
  echo "Removing existing wildbot container..."
  sudo docker rm -f wildbot
fi

# Check for NVIDIA
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected. Launching with NVIDIA Container Toolkit..."
    sudo docker run -it \
      --name wildbot \
      --user root \
      --rm \
      --runtime=nvidia \
      --gpus all \
      --network host \
      --ipc=host \
      --shm-size=8g \
      -e DISPLAY=$DISPLAY \
      -e ROS_DOMAIN_ID=23 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v $(pwd):/ros2_ws \
      my-car-env

# Check for AMD
elif [ -c /dev/kfd ]; then
    echo "AMD GPU detected. Launching with ROCm/KFD..."
    sudo docker run -it \
      --name wildbot \
      --device=/dev/kfd \
      --device=/dev/dri \
      --group-add=video \
      --group-add=110 \
      --user root \
      --rm \
      --security-opt seccomp=unconfined \
      --shm-size=8g \
      --network host \
      --ipc=host \
      --env-file .env \
      -e DISPLAY=$DISPLAY \
      -e ROS_DOMAIN_ID=23 \
      -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v $(pwd):/ros2_ws \
      my-car-env

else
    echo "No supported GPU found. Running in CPU mode."
    sudo docker run -it --rm --network host --ipc=host -v $(pwd):/ros2_ws my-car-env
fi