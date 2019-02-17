#!/bin/bash
###################################################################
# Fill the variables DATASET_PATH and USE_REGULAR_VIO.

# Specify path of the KITTI dataset.
KITTI_DATASET_PATH="/home/yunchang/data/2011_09_26/2011_09_26_drive_0113_sync"
DATASET_PATH="/home/yunchang/data/MH_03"
# TODO remove so this is not hard coded 

# Specify: 1 to use Regular VIO, 0 to use Normal VIO with default parameters.
USE_REGULAR_VIO=0
###################################################################

# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Using dataset at path: $KITTI_DATASET_PATH"
  if [ $USE_REGULAR_VIO == 1 ]; then
    echo "Using REGULAR VIO."
  fi
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) KITTI_DATASET_PATH=$2
          echo "Using dataset at path: $KITTI_DATASET_PATH"
          shift ;;
        # Option -r, specifies that we want to use regular vio.
      -r) USE_REGULAR_VIO=1
          echo "Using Regular VIO!" ;;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

# No user input from this point on.
# Unless user specified to use Regular VIO, run pipeline with default parameters.
BACKEND_TYPE=0
VIO_PARAMS_PATH=""
TRACKER_PARAMS_PATH=""
if [ $USE_REGULAR_VIO == 1 ]; then
  BACKEND_TYPE=1
  VIO_PARAMS_PATH="../params/regularVioParameters.yaml"
  TRACKER_PARAMS_PATH="../params/trackerParameters.yaml"
fi

# Change directory to parent path, in order to make this script
# independent of where we call it from.
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

echo """ Launching:

         ███████╗██████╗  █████╗ ██████╗ ██╗  ██╗    ██╗   ██╗██╗ ██████╗
         ██╔════╝██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝    ██║   ██║██║██╔═══██╗
         ███████╗██████╔╝███████║██████╔╝█████╔╝     ██║   ██║██║██║   ██║
         ╚════██║██╔═══╝ ██╔══██║██╔══██╗██╔═██╗     ╚██╗ ██╔╝██║██║   ██║
         ███████║██║     ██║  ██║██║  ██║██║  ██╗     ╚████╔╝ ██║╚██████╔╝
         ╚══════╝╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝      ╚═══╝  ╚═╝ ╚═════╝

 """

# Execute stereoVIOKitti with given flags.
# The flag --help will provide you with information about what each flag
# does.
../build/stereoVIOKitti \
  --logtostderr=1 \
  --colorlogtostderr=1 \
  --log_prefix=1 \
  --dataset_path="$DATASET_PATH" \
  --kitti_dataset_path="$KITTI_DATASET_PATH" \
  --vio_params_path="$VIO_PARAMS_PATH" \
  --initial_k=10 \
  --final_k=3000 \
  --initial_frame=12 \
  --final_frame=77 \
  --tracker_params_path="$TRACKER_PARAMS_PATH" \
  --flagfile="../params/flags/stereoVIOEuroc.flags" \
  --flagfile="../params/flags/Mesher.flags" \
  --flagfile="../params/flags/VioBackEnd.flags" \
  --flagfile="../params/flags/RegularVioBackEnd.flags" \
  --flagfile="../params/flags/Visualizer3D.flags" \
  --v=10 \
  --backend_type="$BACKEND_TYPE"
