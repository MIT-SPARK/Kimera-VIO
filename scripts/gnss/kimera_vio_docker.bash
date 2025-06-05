#!/bin/bash

# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="/mnt/c/Users/Star/Desktop/diplomav2/Euroc:/Euroc" \
    --volume="/mnt/c/Users/Star/Desktop/diplomaMy/Kimera-VIO:/root/Kimera-VIO" \
    kimera_vio
# Disallow X server connection
xhost -local:root
