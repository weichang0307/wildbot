#!/bin/bash

# Check for NVIDIA
if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected. Build with NVIDIA Container Toolkit..."
    docker build --build-arg GPU_TYPE=nvidia -t my-car-env .

# Check for AMD
elif [ -c /dev/kfd ]; then
    echo "AMD GPU detected. Build with ROCm/KFD..."
    docker build --build-arg GPU_TYPE=amd -t my-car-env .

else
    echo "No supported GPU found. Build in CPU mode."
    docker build --build-arg GPU_TYPE=cpu -t my-car-env .
fi