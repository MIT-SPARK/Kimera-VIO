#!/bin/bash
# Use this script to add %YAML:1.0 tag as header to all relevant
# yaml files in Euroc datastet. Run inside mav0 folder.
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./pointcloud0/sensor.yaml
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./vicon0/sensor.yaml
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./body.yaml
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./state_groundtruth_estimate0/sensor.yaml
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./cam1/sensor.yaml
sed -i'' -e '1i\
  \%YAML\:1\.0\'$'\n' ./cam0/sensor.yaml

