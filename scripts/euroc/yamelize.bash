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
EUROC_DATASET="/home/tonirv/Euroc"

# Parse Options.
if [ $# -eq 0 ]; then
  # If there is no options tell user what are the values we are using.
  echo "Yamelizing dataset at path: $EUROC_DATASET_PATH"
else
  # Parse all the options.
  while [ -n "$1" ]; do # while loop starts
      case "$1" in
        # Option -p, provides path to dataset.
      -p) EUROC_DATASET=$2
          echo "Using dataset at path: $EUROC_DATASET"
          shift ;;
      --)
          shift # The double dash which separates options from parameters
          break
          ;; # Exit the loop using break command
      *) echo "Option $1 not recognized" ;;
      esac
      shift
  done
fi

DIRECTORIES="$EUROC_DATASET/*"
for euroc_dataset in $(ls -d $DIRECTORIES); do
  echo Yamelizing dataset: $euroc_dataset
  for file in $(find $euroc_dataset -iname '*.yaml'); do
    echo Yamelizing file: $file
    sed -i'' -e '1 i\\%YAML\:1\.0' $i/$file
  done
done
