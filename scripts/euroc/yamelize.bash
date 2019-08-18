#!/bin/bash
# Use this script to add %YAML:1.0 tag as header to all relevant
# yaml files in Euroc datastet. Mind that the filesystem structure
# inside the Euroc directory should be:
# ├── Euroc/
# │   ├── V1_01_easy
# │   ├── V1_02_medium
# │   └── etc...
EUROC_DATASET="/home/tonirv/Euroc"
DIRECTORIES="$EUROC_DATASET/*"
for i in $(ls -d $DIRECTORIES); do
  echo Yamelizing dataset: $i
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/pointcloud0/sensor.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/vicon0/sensor.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/body.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/state_groundtruth_estimate0/sensor.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/cam1/sensor.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/cam0/sensor.yaml
  sed -i'' -e '1 i\\%YAML\:1\.0' $i/mav0/imu0/sensor.yaml
done
