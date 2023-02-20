#!/bin/bash
###################################################################
# Fill the variables below

# Specify: 1 to enable the LoopClosureDetector, 0 to not.
USE_LCD=0

# Specify: 1 to enable logging of output files, 0 to not.
LOG_OUTPUT=0
###################################################################

###################################################################
# Other PATHS
# All paths can be absolute or relative to this file location.

USE_ONDEV=0

# Build path: specify where the executable for Kimera is.
BUILD_PATH="../build"

# Params path: specify where the parameters for Kimera are.
PARAMS_PATH="../params/OAK-D-mod"
# PARAMS_PATH="../params/EurocMono"  # use this for monocular-mode (left cam only)

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
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in

      -lcd) USE_LCD=1
            echo "Run VIO with LoopClosureDetector!" ;;
      -log) LOG_OUTPUT=1
            echo "Logging output!";;
        -b) BUILD_PATH=$2 #{OPTARG}
            echo "Changing Build path to -> ${BUILD_PATH}!";;
        -eod) USE_ONDEV=1
            echo "Using on device feature tracker!" ;;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

# Change directory to parent path, in order to make this script
# independent of where we call it from.
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

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
# $BUILD_PATH/stereoVIO_OAKD \
# valgrind --tool=memcheck --vgdb=yes --vgdb-error=0 
$BUILD_PATH/stereoVIO_OAKD \
  --params_folder_path="$PARAMS_PATH" \
  --use_lcd="$USE_LCD" \
  --enable_ondevice_stereo_feature="$USE_ONDEV" \
  --vocabulary_path="$VOCABULARY_PATH/ORBvoc.yml" \
  --flagfile="$PARAMS_PATH/flags/stereoVIO_OAK.flags" \
  --flagfile="$PARAMS_PATH/flags/Mesher.flags" \
  --flagfile="$PARAMS_PATH/flags/VioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/RegularVioBackend.flags" \
  --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" \
  --logtostderr=1 \
  --colorlogtostderr=1 \
  --log_prefix=1 \
  --v=4 \
  --log_output="$LOG_OUTPUT" \
  --save_frontend_images=0 \
  --visualize_frontend_images=0 \
  --output_path="$OUTPUT_PATH" \
  --logbuflevel=1 \
  --log_stereo_matching_images \
  --visualize_feature_predictions

  # --vmodule=Pipeline*=00 \



# If in debug mode, you can run gdb to trace problems.
#export PARAMS_PATH=../params/Euroc
#export DATASET_PATH=/home/tonirv/datasets/EuRoC/V1_01_easy
#gdb --args ../build/stereoVIOEuroc --flagfile="$PARAMS_PATH/flags/stereoVIOEuroc.flags" --flagfile="$PARAMS_PATH/flags/Mesher.flags" --flagfile="$PARAMS_PATH/flags/VioBackend.flags" --flagfile="$PARAMS_PATH/flags/RegularVioBackend.flags" --flagfile="$PARAMS_PATH/flags/Visualizer3D.flags" --logtostderr=1 --colorlogtostderr=1 --log_prefix=0 --dataset_path="$DATASET_PATH" --params_folder_path="$PARAMS_PATH" --initial_k=50 --final_k=2000 --vocabulary_path="../vocabulary/ORBvoc.yml" --use_lcd="0" --v=0 --vmodule=VioBackend=0 --dataset_type="0" --log_output="0" --output_path="../output_logs/"

