sudo docker run -it \
    --gpus all \
    --shm-size=8g \
    --network compose_my_bridge_network \
    --env-file .env \
    -v $(pwd):/ros2_ws \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    my-car-env