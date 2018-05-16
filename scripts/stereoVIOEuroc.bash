 PROJECT_PATH="/home/tonirv/code/spark_vio"
 DATASET_PATH="/home/tonirv/datasets/EuRoC/V1_01_easy"

# Execute stereoVIOEuroc with given flags. The flag --help will provide you with information about what each flag
# does.
$PROJECT_PATH/build/stereoVIOEuroc \
--log_dir=logs \
--alsologtostderr=1 \
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
--vmodule=VioBackEnd=0,RegularVioBackEnd=0 \
