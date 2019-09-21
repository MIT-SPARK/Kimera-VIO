<div align="center">
    <img src="docs/media/sparkvio_logo.png" width="100">
</div>

# SparkVIO: Open-Source Visual Inertial Odometry

[![Build Status](http://ci-sparklab.mit.edu:8080/buildStatus/icon?job=VIO/master)](http://ci-sparklab.mit.edu:8080/job/VIO/job/master/) 
For evaluation plots, check our [jenkins server](http://ci-sparklab.mit.edu:8080/job/VIO/job/master/VIO_20Euroc_20Performance_20Report/plots.html#V1_01_easy).

**Authors:** [Antoni Rosinol](https://www.mit.edu/~arosinol/), Yun Chang, Marcus Abate, Sandro Berchier, [Luca Carlone](https://lucacarlone.mit.edu/)

## What is SparkVIO?

SparkVIO is a Visual Inertial Odometry pipeline for accurate State Estimation from **Stereo** + **IMU** data. (Mono-only capabilities soon to be released).

## Related Publications

We kindly ask to cite the papers below if you find this library useful:

 - **TODO:** add latest paper.

Backend optimization is based on:

 - C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza. **On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation**. IEEE Trans. Robotics, 33(1):1-21, 2016.

 - L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert. **Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors.** IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.

Alternatively, the `Regular VIO` backend, using structural regularities, is described in this paper:

- A. Rosinol, T. Sattler, M. Pollefeys, and L. Carlone. **Incremental Visual-Inertial 3D Mesh Generation with Structural Regularities**. IEEE Int. Conf. on Robotics and Automation (ICRA), 2019.

## Demo

<div align="center">
  <img src="docs/media/SparkVIO_ROS_mesh.gif"/>
</div>

# 1. Installation

Tested on Mac, Ubuntu 14.04 & 16.04 & 18.04.

## Prerequisites

- [GTSAM](https://github.com/borglab/gtsam) >= 4.0
- [OpenCV](https://github.com/opencv/opencv) >= 3.3.1
- [OpenGV](https://github.com/laurentkneip/opengv)
- [Glog](http://rpg.ifi.uzh.ch/docs/glog.html), [Gflags](https://gflags.github.io/gflags/), [Gtest](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) (installed automagically).

> Note: if you want to avoid building all dependencies yourself, we provide a docker image that will install them for you. Check installation instructions in [docs/sparkvio_installation.md](./docs/sparkvio_installation.md).

## Installation Instructions

Find how to install SparkVIO and its dependencies here: **[Installation instructions](./docs/sparkvio_installation.md)**.

# 2. Usage

## i. [Euroc](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) Dataset

#### Download Euroc's dataset

- Download one of Euroc's datasets, for example [V1_01_easy.zip](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip).

> Datasets MH_04 and V2_03 have different number of left/right frames. We suggest using instead our version of Euroc [here](https://drive.google.com/open?id=1_kwqHojvBusHxilcclqXh6haxelhJW0O).

- Unzip the dataset to your preferred directory, for example, in `~/Euroc/V1_01_easy`:
```bash
mkdir -p ~/Euroc/V1_01_easy
unzip -o ~/Downloads/V1_01_easy.zip -d ~/Euroc/V1_01_easy
```

#### Yamelize Euroc's dataset
Add `%YAML:1.0` at the top of each `.yaml` file inside Euroc.
You can do this manually or run the `yamelize.bash` script by indicating where the dataset is (it is assumed below to be in `~/path/to/euroc`):
```bash
cd SparkVIO
bash ./scripts/euroc/yamelize.bash -p ~/path/to/euroc
```

### Run SparkVIO in Euroc's dataset

Using a bash script bundling all command-line options and gflags:

```bash
cd SparkVIO
bash ./scripts/stereoVIOEuroc.bash -p "PATH_TO_DATASET/V1_01_easy"
```

> Alternatively, one may directly use the executable in the build folder:
`./build/stereoVIOEuroc`. Nevertheless, check the script `./scripts/stereoVIOEuroc.bash` to understand what parameters are expected, or check the [parameters](#Parameters) section below.

## ii. [Kitti](http://www.cvlibs.net/datasets/kitti/raw_data.php) Dataset

#### Download Kitti's dataset

- Download raw data from [Kitti ](http://www.cvlibs.net/datasets/kitti/raw_data.php?type=residential) (in order to have IMU messages). For example:
  - Download unsynced + unrectified raw dataset (e.g. [2011\_09\_26\_drive\_0005\_extract](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/2011_09_26_drive_0005_extract.zip)).
  - Download as well the calibration data (three files) from raw dataset (e.g. [2011\_09\_26\_calib](https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip)), and save them inside the folder `2011_09_26`.

### Run SparkVIO in Kitti's dataset

- Run: 
  ```bash
  cd SparkVIO
  bash ./scripts/stereoVIOEuroc.bash -p "PATH_TO_DATASET/2011_09_26_drive_0005_extract" -d 1
  ```
   where you specify the path to the dataset (e.g. path to `2011_09_26_drive_0005_extract` folder).

## iii. Using [ROS wrapper](https://github.com/MIT-SPARK/Kimera-VIO-ROS)

We provide a ROS wrapper of SparkVIO that you can find at: https://github.com/MIT-SPARK/Kimera-VIO-ROS.

This library can be cloned into a catkin workspace and built alongside the ROS wrapper.

# 3. Parameters
SparkVIO accepts two sources of parameters:
- YAML files: contains parameters for Backend and Frontend.
- [gflags](https://gflags.github.io/gflags/) contains parameters for all the rest.

To get help on what each gflag parameter does, just run the executable with the `--help` flag: `./build/stereoVIOEuroc --help`. You should get a list of gflags similar to the ones [here](./docs/gflags_parameters.md).

  - Optionally, you can try the VIO using structural regularities, as in [our ICRA 2019 paper](https://ieeexplore.ieee.org/abstract/document/8794456), by specifying the option ```-r```: ```./stereoVIOEuroc.bash -p "PATH_TO_DATASET/V1_01_easy" -r```

OpenCV's 3D visualization has also some shortcuts for interacting with it: check [tips for usage](./docs/tips_usage.md) 

# 4. Contribution guidelines

We follow the branch, open PR, review, and merge workflow.

To contribute to this repo, ensure your commits pass the linter pre-commit checks.
To enable these checks you will need to [install linter](./docs/linter_installation.md).
We also provide a `.clang-format` file with the style rules that the repo uses, so that you can use [`clang-format`](https://clang.llvm.org/docs/ClangFormat.html) to reformat your code.

Also, check [tips for development](./docs/tips_development.md) and our **[developer guide](./docs/developer_guide.md)**.

# 5. FAQ
  If you have problems building or running the pipeline and/or issues with dependencies, you might find useful information in our [FAQ](./docs/faq.md) or in the issue tracker.

# 6. Chart

![vio_chart](./docs/media/sparkvio_chart.png)

![overall_chart](./docs/media/kimera_chart_23.jpeg)

# 7. BSD License

SparkVIO is open source under the BSD license, see the [LICENSE.BSD](LICENSE.BSD) file.
