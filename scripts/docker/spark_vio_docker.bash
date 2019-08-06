#!/bin/bash

docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    spark_vio \
    bash -ic "./VIO/scripts/spark_vio.bash -p ./data/V1_01_easy -r -parallel"
