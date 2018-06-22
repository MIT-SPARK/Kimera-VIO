#!/bin/bash
# Fill the variables DATASET_PATH and USE_REGULAR_VIO.

# Specify path of the EuRoC dataset.
DATASET_PATH="/home/luca/data/euroc/V1_01_easy"

# 1 to use regular vio, 0 to use normal vio with default parameters.
USE_REGULAR_VIO=0

###################################################################
# Change directory to parent path, in order to make this script
# independent of where we call it from.
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"


# Execute stereoVIOEuroc with given flags.
# The flag --help will provide you with information about what each flag
# does.
if [ $USE_REGULAR_VIO == 1 ]; then
  ../build/stereoVIOEuroc \
    --logtostderr=1 \
    --colorlogtostderr=1 \
    --log_prefix=0 \
    --dataset_path="$DATASET_PATH" \
    --vio_params_path="../params/vioParameters.yaml" \
    --tracker_params_path="../params/trackerParameters.yaml" \
    --backend_type=1 \
    --visualize=1 \
    --visualize_plane_constraints=false \
    --viz_type=5 \
    --log_output=false \
    --v=0 \
    --vmodule=VioBackEnd=0,RegularVioBackEnd=0,Mesher=0 \
    --add_extra_lmks_from_stereo=false \
    --visualize_mesh_2d_filtered=false \
    --use_gouraud_shading=true \
    --max_triangle_side=0.5
else
  ../build/stereoVIOEuroc \
    --logtostderr=1 \
    --colorlogtostderr=1 \
    --log_prefix=0 \
    --dataset_path="$DATASET_PATH" \
    --vio_params_path="" \
    --tracker_params_path="" \
    --backend_type=0 \
    --visualize=1 \
    --viz_type=0 \
    --log_output=true \
    --v=0 \
    --vmodule=stereoVIOEuroc=100,VioBackEnd=0,Mesher=0 \
    --add_extra_lmks_from_stereo=false
fi
