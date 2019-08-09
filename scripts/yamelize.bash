#!/bin/bash
# Use this script to add %YAML:1.0 tag as header to all relevant
# yaml files in Euroc datastet.
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/pointcloud0/sensor.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/vicon0/sensor.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/body.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/state_groundtruth_estimate0/sensor.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/cam1/sensor.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/cam0/sensor.yaml
sed -i'' -e '1 i\\%YAML\:1\.0' ./mav0/imu0/sensor.yaml

