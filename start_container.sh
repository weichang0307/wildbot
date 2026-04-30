sudo docker run -it \
  --device=/dev/kfd \
  --device=/dev/dri \
  --group-add=video \
  --group-add=110 \
  --security-opt seccomp=unconfined \
  --shm-size=8g \
  --network compose_my_bridge_network \
  --env-file .env \
  -e DISPLAY=$DISPLAY \
  -e HSA_OVERRIDE_GFX_VERSION=11.0.0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v $(pwd):/ros2_ws \
  my-car-env