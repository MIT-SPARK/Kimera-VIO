#!/bin/bash
###################################################################
# Fill the variables DATASET_PATH, USE_REGULAR_VIO and PARALLEL_RUN.

# Specify path of the EuRoC dataset.
# The path can be absolute, or relative to this file location.
DATASET_PATH="E:/kimera-slam/SLAM-Data/Datasets/MH_05_difficult"

# Specify: 1 to use Regular VIO, 0 to use Normal VIO with default parameters.
USE_REGULAR_VIO=0

# Specify: 0 to run on EuRoC data, 1 to run on Kitti
DATASET_TYPE=0

# Specify: 1 to run pipeline in parallel mode, 0 to run sequentially.
PARALLEL_RUN=1

# Specify: 1 to enable the LoopClosureDetector, 0 to not.
USE_LCD=1

# Specify: 1 to enable logging of output files, 0 to not.
LOG_OUTPUT=0
###################################################################

###################################################################
# Other PATHS
# All paths can be absolute or relative to this file location.

# Build path: specify where the executable for Kimera is.
BUILD_PATH="../build"

# Params path: specify where the parameters for Kimera are.
PARAMS_PATH="../params"

# Vocabulary path: specify where the vocabulary for loop closure is.
VOCABULARY_PATH="../vocabulary"

# Output path: specify where the output logs will be written.
# (only used if LOG_OUTPUT is enabled)
OUTPUT_PATH="../output_logs"
###################################################################

# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Using dataset at path: $DATASET_PATH"
  if [ $USE_REGULAR_VIO == 1 ]; then
    echo "Using REGULAR VIO."
  fi
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) DATASET_PATH=$2
          echo "Using dataset at path: $DATASET_PATH"
          shift ;;
        # Option -d, set dataset type
      -d) DATASET_TYPE=$2
          echo "Using dataset type: $DATASET_TYPE"
          echo "0 is for euroc and 1 is for kitti"
          shift ;;
        # Option -r, specifies that we want to use regular vio.
      -r) USE_REGULAR_VIO=1
          echo "Using Regular VIO!" ;;
      -s) PARALLEL_RUN=0
          echo "Run VIO in SEQUENTIAL mode!" ;;
      -lcd) USE_LCD=1
           echo "Run VIO with LoopClosureDetector!" ;;
      -log) LOG_OUTPUT=1
           echo "Logging output!";;
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

if [ $USE_REGULAR_VIO == 1 ]; then
  BACKEND_TYPE=1
fi

# Change directory to parent path, in order to make this script
# independent of where we call it from.
parent_path=$( cd  $(dirname ${BASH_SOURCE[0]}) ; pwd -P )
cd $parent_path

echo """ Launching:

            ██╗  ██╗██╗███╗   ███╗███████╗██████╗  █████╗
            ██║ ██╔╝██║████╗ ████║██╔════╝██╔══██╗██╔══██╗
            █████╔╝ ██║██╔████╔██║█████╗  ██████╔╝███████║
            ██╔═██╗ ██║██║╚██╔╝██║██╔══╝  ██╔══██╗██╔══██║
            ██║  ██╗██║██║ ╚═╝ ██║███████╗██║  ██║██║  ██║
            ╚═╝  ╚═╝╚═╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝

 """

# Execute stereoVIOEuroc with given flags.
# The flag --help will provide you with information about what each flag
# does.
E:/kimera_new/Kimera-VIO/out/build/x64-Release/stereoVIOEuroc.exe \
  --dataset_type="$DATASET_TYPE" \
  --dataset_path="$DATASET_PATH" \
  --initial_k=50 \
  --final_k=20000 \
  --backend_type="$BACKEND_TYPE" \
  --left_cam_params_path="$PARAMS_PATH/LeftCameraParams.yaml" \
  --right_cam_params_path="$PARAMS_PATH/RightCameraParams.yaml" \
  --imu_params_path="$PARAMS_PATH/ImuParams.yaml" \
  --backend_params_path="$PARAMS_PATH/regularVioParameters.yaml" \
  --frontend_params_path="$PARAMS_PATH/trackerParameters.yaml" \
  --use_lcd="$USE_LCD" \
  --lcd_params_path="$PARAMS_PATH/LCDParameters.yaml" \
  --vocabulary_path="$VOCABULARY_PATH/ORBvoc.yml" \
  --flagfile="$PARAMS_PATH/flags/stereoVIOEuroc.flags" \
  --flagfile="$PARAMS_PATH/flags/Mesher.flags" \
  --flagfile="$PARAMS_PATH/flags/VioBackEnd.flags" \
  --flagfile="$PARAMS_PATH/flags/RegularVioBackEnd.flags" \
  --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" \
  --flagfile="$PARAMS_PATH/flags/EthParser.flags" \
  --parallel_run="$PARALLEL_RUN" \
  --logtostderr=1 \
  --colorlogtostderr=1 \
  --log_prefix=1 \
  --v=0 \
  --vmodule=Pipeline*=00 \
  --log_output="$LOG_OUTPUT" \
  --output_path="$OUTPUT_PATH"


# If in debug mode, you can run gdb to trace problems.
#export DATASET_PATH=/home/tonirv/datasets/euroc/EuRoC/V1_01_easy
#gdb --args ../build/stereoVIOEuroc --flagfile="../params/flags/stereoVIOEuroc.flags" --flagfile="../params/flags/Mesher.flags" --flagfile="../params/flags/VioBackEnd.flags" --flagfile="../params/flags/RegularVioBackEnd.flags" --flagfile="../params/flags/Visualizer3D.flags" --flagfile="../params/flags/EthParser.flags" --logtostderr=1 --colorlogtostderr=1 --log_prefix=0 --dataset_path="$DATASET_PATH" --left_cam_params_path="../params/LeftCameraParams.yaml" --right_cam_params_path="../params/RightCameraParams.yaml" --imu_params_path="../params/ImuParams.yaml" --backend_params_path="$BACKEND_PARAMS_PATH" --initial_k=50 --final_k=2000 --frontend_params_path="" --lcd_params_path="" --vocabulary_path="../vocabulary/ORBvoc.yml" --use_lcd="0" --v=0 --vmodule=VioBackEnd=0,RegularVioBackEnd=0,Mesher=0,StereoVisionFrontEnd=0 --backend_type="0" --parallel_run="1" --dataset_type="0" --log_output="0" --output_path="../output_logs/"

