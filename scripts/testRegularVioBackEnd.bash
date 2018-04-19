 PROJECT_PATH="/home/tonirv/code/spark_vio"

# Execute stereoVIOEuroc with given flags. The flag --help will provide you with information about what each flag
# does.
$PROJECT_PATH/build/tests/testRegularVioBackEnd \
--logtostderr=1 \
--colorlogtostderr=1 \
--v=1000 # Verbosity level.

