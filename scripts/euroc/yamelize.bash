#!/bin/bash
# Use this script to add %YAML:1.0 tag as header to all relevant
# yaml files in Euroc datastet. Mind that the filesystem structure
# inside the Euroc directory should be:
# ├── Euroc/
# │   ├── V1_01_easy
# │   ├── V1_02_medium
# │   └── etc...

# Either modify this variable yourself or use this script as
# ```
# bash yamelize.bash -p PATH_TO_YOUR_EUROC_DATASET
# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Please provide a Dataset path using the -p flag:
        Usage: yamelize.bash -p PATH_TO_EUROC_DATASET"
  exit
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) EUROC_DATASET=$2
          echo "Using dataset at path: $EUROC_DATASET"
          shift ;;
      --) shift # The double dash which separates options from parameters
          break
          ;;
      *) echo -e "\e[31mOption $1 not recognized"
         exit
         ;;
      esac
      shift
  done
fi

# Check EUROC_DATASET is not empty:
if [ -z "$EUROC_DATASET" ]; then
  echo -e "\e[31mDataset path cannot be empty: use -p flag"
  exit
fi

LIST_INODES=$(ls -d $EUROC_DATASET/*)
if [ ${#LIST_INODES[@]} -eq 0 ]; then
  echo -e "\e[31mNo datasets found..."
  exit
fi
for euroc_dataset in $LIST_INODES; do
  echo -e "\e[32mYamelizing dataset: $euroc_dataset\033[0m"
  YAML_FILES=$(find $euroc_dataset -iname '*.yaml')
  if [ ${#YAML_FILES[@]} -eq 0 ]; then
    echo -e "\e[31mNo yaml files found..."
  fi
  for file in $YAML_FILES; do
    echo -e "Yamelizing file: $file"
    sed -i'' -e '1 i\\%YAML\:1\.0' $file
  done
done
