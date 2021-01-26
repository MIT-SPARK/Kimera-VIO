# Kimera-VIO Installation

Tested on Mac, Ubuntu 14.04, 16.04 & 18.04.

If you want to avoid building all these dependencies yourself, we provide two options:

1. **Docker image**: that will install them for you. See section [From Dockerfile](#From-Dockerfile) below.

2. **Catkin**: if you use ROS, then [Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) can install all dependencies and Kimera inside a catkin workspace. Follow the instructions in there.

We recommend using the docker image for a quick demo, and catkin for actual development.
Alternatively, you may install the dependencies and Kimera from \"source\" as detailed below:

## Prerequisites:

- Third-party dependencies:

  - [GTSAM](https://github.com/borglab/gtsam) >= 4.0
  - [OpenCV](https://github.com/opencv/opencv) >= 3.3.1
  - [OpenGV](https://github.com/laurentkneip/opengv)
  - [Glog](http://rpg.ifi.uzh.ch/docs/glog.html), [Gflags](https://gflags.github.io/gflags/), [Gtest](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) (installed automagically).
  - [DBoW2](https://github.com/dorian3d/DBoW2)
  - [Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO)

> Installation instructions below.

## Install general dependencies

First, update package list: `sudo apt-get update`

- Build dependencies:
```bash
sudo apt-get install -y --no-install-recommends apt-utils
sudo apt-get install -y cmake
```

- Gtsam dependencies:
```bash
sudo apt-get install -y libboost-all-dev
```

- OpenCV dependencies:
  - on Mac:
```bash
homebrew install vtk # (to check)
```
  - On Ubuntu 18.04
```bash
# (libvtk5-dev, libgtk2.0-dev in ubuntu 16.04)
sudo apt-get install -y \
      build-essential unzip pkg-config \
      libjpeg-dev libpng-dev libtiff-dev \
      libvtk6-dev \
      libgtk-3-dev \
      libparmetis-dev \
      libatlas-base-dev gfortran
```

## Install GTSAM

#### GTSAM's Optional dependencies (highly recommended for speed)

Install [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/): `sudo apt-get install libtbb-dev`

#### GTSAM Source Install

Clone GTSAM: `git clone git@github.com:borglab/gtsam.git`

> (last tested with commit `ee069286b447ff58b809423cc77c777a02abdfe5`)
> Previously tested commits: `0c3e05f375c03c5ff5218e708db416b38f4113c8`

Make build dir, and run `cmake`:

```bash
cd gtsam
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=OFF -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF ..
```

Ensure that:
- TBB is enabled: check for `--   Use Intel TBB                  : Yes` after running `cmake`.
- Compilation is in Release mode: check for `--   Build type                     : Release` after running `cmake`.
- Use GTSAM's Eigen, **not** the system-wide one (OpenGV and GTSAM must use same Eigen, see OpenGV install instructions below).
- `Rot3 retract is full ExpMap` is set to enabled, and `Pose3 retract is full ExpMap` is also set to enabled. Without these flags, Kimera-RPGO does not optimize the pose-graph well and may produce incorrect results.

Compile and install GTSAM:
```bash
make -j $(nproc) check # (optional, runs unit tests)
sudo make -j $(nproc) install
```

> Alternatively, replace `$(nproc)` by the number of available cores in your computer.

> Note 1a: if you use MKL in gtsam, you may need to add to `~/.bashrc` a line similar to:
>  ```source /opt/intel/parallel_studio_xe_2018/compilers_and_libraries_2018/linux/mkl/bin/mklvars.sh intel64```
> (Alternatively, type `locate compilervars.sh` and then `source $output file.sh that you got from locate$`, add to your .bashrc to automate).

> Note 1b: sometimes you may need to add `/usr/local/lib` to `LD_LIBRARY_PATH` in `~/.bashrc` (if you get lib not found errors at run or test time).

> Note: we are considering to enable EPI in GTSAM, which will require to set the GTSAM_THROW_CHEIRALITY_EXCEPTION to false (cmake flag).

> Note: for better performance when using the IMU factors, set GTSAM_TANGENT_PREINTEGRATION to 'false' (cmake flag)

> Note: also enable the `GTSAM_BUILD_WITH_MARCH_NATIVE` compile option for max performance (at the expense of the portability of your executable). Check [install gtsam](https://github.com/borglab/gtsam/blob/develop/INSTALL.md) for more details. Note that for some systems, `MARCH_NATIVE` might cause problems that culminates in the form of segfaults when you run the unittests. If you do wish to use this flag, simply add `-DGTSAM_BUILD_WITH_MARCH_NATIVE=On ` to the flags on the `cmake` command. Note that sometimes, the flag it is enabled by default.

## Install OpenCV

#### OpenCV Source Install

Download OpenCV and run cmake:
```bash
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout tags/3.3.1
mkdir build
cd build
cmake -DWITH_VTK=On .. # Use -DWITH_TBB=On if you have TBB
```

Finally, build and install OpenCV:
```bash
sudo make -j $(nproc) install
```

> Alternatively, replace `$(nproc)` by the number of available cores in your computer.

## Install OpenGV
Clone the repo:
```bash
git clone https://github.com/laurentkneip/opengv.git
```

Set opengv to use the same Eigen version than GTSAM (for example: `$HOME/gtsam/gtsam/3rdparty/Eigen`), by modifying the cmake flags `EIGEN_INCLUDE_DIR` and `EIGEN_INCLUDE_DIRS`. If you don't do so, errors may appear (maybe due to GTSAM and OpenGV using different versions of Eigen!) Note that if using `cmake-gui` to modify paths, make sure you tick the `Advanced` box so that both flags are visible:

```bash
cd opengv
mkdir build
cd build
# Replace path to your GTSAM's Eigen
cmake .. -DEIGEN_INCLUDE_DIR=/home/tonirv/Code/gtsam/gtsam/3rdparty/Eigen -DEIGEN_INCLUDE_DIRS=/home/tonirv/Code/gtsam/gtsam/3rdparty/Eigen
```

Finally, install opengv:
```bash
sudo make -j $(nproc) install
```

> Alternatively, replace `$(nproc)` by the number of available cores in your computer.

## Install DBoW2
Clone the repo and run cmake:
```bash
git clone https://github.com/dorian3d/DBoW2.git
cd DBoW2
mkdir build
cd build
cmake ..
sudo make -j $(nproc) install
```

## Install Kimera-RPGO
Clone the repo and run cmake:
```bash
git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
cd Kimera-RPGO
mkdir build
cd build
cmake ..
sudo make -j $(nproc)
```

## Glog/Gflags

Linux
```bash
sudo apt-get install libgflags-dev libgoogle-glog-dev
```

MacOS
```bash
brew install gflags glog
```

## Gtest
Gtest will be automatically downloaded using cmake.

## Bag-of-Words Vocabulary

For loop closure detection, we use a bag-of-words method based on [DBoW2](https://github.com/dorian3d/DBoW2). This requires a vocabulary of visual words. If you wish to use the loop closure detection module, you must download a vocabulary file.

We have packaged a large vocabulary made by the creators of [ORB_SLAM_2](https://github.com/raulmur/ORB_SLAM2) along with their license at [this location](https://www.dropbox.com/s/lyo0qgbdxn6eg6o/ORBvoc.zip?dl=0). Follow that link to download the files, and put them in the [vocabulary](/vocabulary/) directory.

Alternatively, cmake will automatically download these files for you when you `make` Kimera-VIO. Follow the instructions below:

## Install Kimera-VIO

Before proceeding, ensure all dependencies are installed or use the provided [dockerfile](#From-Dockerfile).

### From Source:

Clone this repository:
```bash
git clone git@github.com:MIT-SPARK/Kimera-VIO.git Kimera-VIO
```

Build Kimera-VIO
```bash
cd Kimera-VIO
mkdir build
cd build
cmake ..
make -j $(nproc)
```

> Alternatively, replace '$(nproc)' by the number of available cores in your computer.

### From Dockerfile:

If you want to avoid building all these dependencies yourself, we provide a docker image that will install all dependencies for you.
For that, you will need to install [Docker](https://docs.docker.com/install/).

Once installed, clone this repo, build the image and run it:

```bash
# Clone the repo
git clone git@github.com:MIT-SPARK/Kimera-VIO.git Kimera-VIO

# Build the image
cd Kimera-VIO
docker build --rm -t kimera_vio -f ./scripts/docker/Dockerfile . 
```

This may take a while (>15min), so you can go and grab a cup of coffee.
Once done, you can run the `kimera_vio_docker.bash`:

```bash
# Run an example dataset
./scripts/docker/kimera_vio_docker.bash
```
