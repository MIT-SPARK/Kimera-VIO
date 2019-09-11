# SparkVIO Installation

Tested on Mac, Ubuntu 14.04 & 16.04 & 18.04.

> Note: if you want to avoid building all these dependencies yourself, we provide a docker image that will install them for you. See section `From Dockerfile` below.

## Prerequisites:

- Third-party dependencies:

  - [GTSAM](https://github.com/borglab/gtsam) >= 4.0
  - [OpenCV](https://github.com/opencv/opencv) >= 3.3.1
  - [OpenGV](https://github.com/laurentkneip/opengv)
  - [Glog](http://rpg.ifi.uzh.ch/docs/glog.html), [Gflags](https://gflags.github.io/gflags/), [Gtest](https://github.com/google/googletest/blob/master/googletest/docs/primer.md) (installed automagically).

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
      libatlas-base-dev gfortran
```

## Install GTSAM

#### GTSAM's Optional dependencies (highly recommended for speed)

Install [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/): `sudo apt-get install libtbb-dev`

#### GTSAM Source Install

Clone GTSAM: `git clone git@github.com:borglab/gtsam.git`

> (last tested with commit `0c3e05f375c03c5ff5218e708db416b38f4113c8`)

Make build dir, and run `cmake`:

```bash
cd gtsam
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=OFF ..
```

Ensure that:
- TBB is enabled: check for `--   Use Intel TBB                  : Yes` after running `cmake`.
- Compilation is in Release mode: check for `--   Build type                     : Release` after running `cmake`.
- Use GTSAM's Eigen, **not** the system-wide one (OpenGV and GTSAM must use same Eigen, see OpenGV install instructions below).

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

> Note: also add `-march=native` to `GTSAM_CMAKE_CXX_FLAGS` for max performance (at the expense of the portability of your executable). Check [install gtsam](https://github.com/borglab/gtsam/blob/develop/INSTALL.md) for more details. Note that for some systems, `-march=native` might cause problems that culminates in the form of segfaults when you run the unittests.

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
git clone https://github.com/laurentkneip/opengv
```

- using cmake-gui, set: the eigen version to the GTSAM one (for me: /Users/Luca/borg/gtsam/gtsam/3rdparty/Eigen). if you don't do so, very weird error (TODO document) appear (may be due to GTSAM and OpenGV using different versions of eigen!)
- in the opengv folder do:

```bash
cd opengv
mkdir build
cd build
cmake ..
sudo make -j $(nproc) install
```

> Alternatively, replace `$(nproc)` by the number of available cores in your computer.

## Glog, Gflags & Gtest
Glog, Gflags, and Gtest will be automatically downloaded using cmake unless there is a system-wide installation found (gtest will always be downloaded).

## Install SparkVIO

Before proceeding, ensure all dependencies are installed or use the provided [dockerfile](#From-Dockerfile).

### From Source:

Clone this repository:
```bash
git clone git@github.mit.edu:SPARK/VIO.git SparkVIO
```

Build SparkVIO:
```bash
cd SparkVIO
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
> Note: while this repo remains private, you'll need to **specify your ssh keys**: replace `<username>` below, with an ssh key that has read access to the repo in github.mit.edu (check your profile settings for such ssh key). Also, since docker build doesn't handle user input, ensure your ssh key does **not** have a passphrase.

```bash
# Clone the repo
git clone git@github.mit.edu:SPARK/VIO.git SparkVIO

# Build the image
cd SparkVIO
docker build --rm -t spark_vio -f ./scripts/docker/Dockerfile . \
--build-arg SSH_PRIVATE_KEY="$(cat /home/<username>/.ssh/id_rsa)"
```

This may take a while (>15min), so you can go and grab a cup of coffee.
Once done, you can run the `spark_vio_docker.bash`:

```bash
# Run an example dataset
./scripts/docker/spark_vio_docker.bash
```
