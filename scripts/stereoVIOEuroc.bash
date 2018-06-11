#!/bin/bash
PROJECT_PATH="/home/tonirv/code/spark_vio"
DATASET_PATH="/home/tonirv/datasets/EuRoC/V1_01_easy"

# 1 to use regular vio, 0 to use normal vio with default parameters.
USE_REGULAR_VIO=1

# Execute stereoVIOEuroc with given flags. The flag --help will provide you with information about what each flag
# does.
if [ $USE_REGULAR_VIO == 1 ]; then
  $PROJECT_PATH/build/stereoVIOEuroc \
    --logtostderr=1 \
    --colorlogtostderr=1 \
    --log_prefix=0 \
    --dataset_path="$DATASET_PATH" \
    --vio_params_path="${PROJECT_PATH}/params/vioParameters.yaml" \
    --tracker_params_path="${PROJECT_PATH}/params/trackerParameters.yaml" \
    --backend_type=1 \
    --visualize=1 \
    --viz_type=5 \
    --log_output=false \
    --v=0 \
    --vmodule=VioBackEnd=0,RegularVioBackEnd=0,Mesher=0 \
    --add_extra_lmks_from_stereo=false
else
  $PROJECT_PATH/build/stereoVIOEuroc \
    --logtostderr=1 \
    --colorlogtostderr=1 \
    --log_prefix=0 \
    --dataset_path="$DATASET_PATH" \
    --vio_params_path="" \
    --tracker_params_path="" \
    --backend_type=0 \
    --visualize=1 \
    --viz_type=0 \
    --log_output=false \
    --v=0 \
    --vmodule=stereoVIOEuroc=100,VioBackEnd=0,Mesher=0 \
    --add_extra_lmks_from_stereo=false
fi
