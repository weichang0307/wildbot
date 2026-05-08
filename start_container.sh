#!/bin/bash

# Check for NVIDIA
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected. Launching with NVIDIA Container Toolkit..."
    sudo docker run -it \
      --rm \
      --runtime=nvidia \
      --gpus all \
      --network host \
      --shm-size=8g \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v $(pwd):/ros2_ws \
      my-car-env

# Check for AMD
elif [ -c /dev/kfd ]; then
    echo "AMD GPU detected. Launching with ROCm/KFD..."
    sudo docker run -it \
      --device=/dev/kfd \
      --device=/dev/dri \
      --group-add=video \
      --group-add=110 \
      --security-opt seccomp=unconfined \
      --shm-size=8g \
      --network host \
      --env-file .env \
      -e DISPLAY=$DISPLAY \
      -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v $(pwd):/ros2_ws \
      my-car-env

else
    echo "No supported GPU found. Running in CPU mode."
    sudo docker run -it --rm --network host -v $(pwd):/ros2_ws my-car-env
fi