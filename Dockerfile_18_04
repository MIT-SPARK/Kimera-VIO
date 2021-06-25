# Use an official Python runtime as a parent image
FROM ubuntu:18.04

MAINTAINER Antoni Rosinol "arosinol@mit.edu"

# To avoid tzdata asking for geographic location...
ENV DEBIAN_FRONTEND=noninteractive

# Set the working directory to /root
ENV DIRPATH /root/
WORKDIR $DIRPATH

#Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y git cmake build-essential pkg-config

# Install xvfb to provide a display to container for GUI realted testing.
RUN apt-get update && apt-get install -y xvfb

# Install OpenCV for Ubuntu 18.04
RUN apt-get update && apt-get install -y \
      unzip \
      libjpeg-dev libpng-dev libtiff-dev \
      libvtk6-dev \
      libgtk-3-dev \
      libatlas-base-dev gfortran

RUN git clone https://github.com/opencv/opencv.git
RUN cd opencv && \
      git checkout tags/3.3.1 && \
      mkdir build

RUN git clone https://github.com/opencv/opencv_contrib.git
RUN cd opencv_contrib && \
      git checkout tags/3.3.1

RUN cd opencv/build && \
      cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -D BUILD_opencv_python=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=OFF \
      -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
      make -j$(nproc) install

# Install GTSAM
RUN apt-get update && apt-get install -y libboost-all-dev libtbb-dev
ADD https://api.github.com/repos/borglab/gtsam/git/refs/heads/master version.json
RUN git clone https://github.com/borglab/gtsam.git
RUN cd gtsam && \
    git fetch && \
    git checkout develop && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF .. && \
    make -j$(nproc) install

# Install Open_GV
RUN git clone https://github.com/laurentkneip/opengv
RUN cd opengv && \
      mkdir build
RUN cd opengv/build && \
      cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DEIGEN_INCLUDE_DIRS=$DIRPATH/gtsam/gtsam/3rdparty/Eigen \
      -DEIGEN_INCLUDE_DIR=$DIRPATH/gtsam/gtsam/3rdparty/Eigen .. && \
      make -j$(nproc) install

# Install DBoW2
RUN git clone https://github.com/dorian3d/DBoW2.git
RUN cd DBoW2 && \
      mkdir build && \
      cd build && \
      cmake .. && \
      make -j$(nproc) install

# Install RobustPGO
ADD https://api.github.com/repos/MIT-SPARK/Kimera-RPGO/git/refs/heads/master version.json
RUN git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
RUN cd Kimera-RPGO && \
      mkdir build && \
      cd build && \
      cmake .. && \
      make -j$(nproc)

RUN apt-get update && \
    apt-get install software-properties-common -y
# Get python3
RUN apt-get update && \
    add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && \
    apt-get install -y python3.6 python3.6-dev python-pip python3-pip python-tk python3-tk
RUN python3.6 -m pip install PyQt5==5.14

# Install evo-1 for evaluation
# Hack to avoid Docker's cache when evo-1 master branch is updated.
ADD https://api.github.com/repos/ToniRV/evo-1/git/refs/heads/master version.json
RUN git clone https://github.com/ToniRV/evo-1.git
RUN cd evo-1 && python3.6 $(which pip3) install .

# Install Kimera VIO Evaluation
RUN python3.6 $(which pip3) install ipython prompt_toolkit
# Hack to avoid Docker's cache when Kimera VIO Evaluation master branch is updated.
ADD https://api.github.com/repos/MIT-SPARK/Kimera-VIO-Evaluation/git/refs/heads/master version.json
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO-Evaluation.git
# We use `pip3 install -e .` so that Jinja2 has access to the webiste template...
RUN cd Kimera-VIO-Evaluation && git fetch && git checkout master && python3.6 $(which pip3) install -e .

# Install glog, gflags
RUN apt-get update && apt-get install -y libgflags2.2 libgflags-dev libgoogle-glog0v5 libgoogle-glog-dev

# Install Pangolin
RUN apt-get update && apt-get install -y libgl1-mesa-dev libglew-dev
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN cd Pangolin && \
      mkdir build && \
      cd build && \
      cmake .. && \
      make -j$(nproc)
