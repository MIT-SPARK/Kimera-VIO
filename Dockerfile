# Use an official Python runtime as a parent image
FROM ubuntu:18.04

# To avoid tzdata asking for geographic location...
ENV DEBIAN_FRONTEND noninteractive

# Set the working directory to /app
ENV DIRPATH /root/
WORKDIR $DIRPATH

#Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get update && apt-get install -y \
      git \
      cmake

# Install GTSAM
RUN apt-get update && apt-get install -y libboost-all-dev
RUN git clone https://bitbucket.org/gtborg/gtsam.git
RUN cd gtsam && \
    git fetch && git checkout feature/improvementsIncrementalFilter && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DGTSAM_BUILD_STATIC_LIBRARY=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE=ON .. && \
    make -j$(nproc) install

# Install OpenCV for Ubuntu 18.04
RUN apt-get update && apt-get install -y \
      build-essential cmake unzip pkg-config \
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
      cmake -DCMAKE_BUILD_TYPE=Release 
      -DCMAKE_INSTALL_PREFIX=/usr/local 
      -DWITH_VTK=On 
      -D BUILD_opencv_java=OFF
      -D BUILD_opencv_python=OFF
      -D BUILD_opencv_python2=OFF
      -D BUILD_opencv_python3=OFF ..
      -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
      make -j$(nproc) install

# Install Open_GV
RUN git clone https://github.com/laurentkneip/opengv
RUN cd opengv && \
      mkdir build
RUN cd opengv/build && \
      cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DEIGEN_INCLUDE_DIR=/root/gtsam/gtsam/3rdparty/Eigen .. && \
      make -j$(nproc) install

# Install spark_vio_evaluation from PyPI
RUN apt-get update && apt-get install -y python-pip python-dev
RUN pip install spark_vio_evaluation
