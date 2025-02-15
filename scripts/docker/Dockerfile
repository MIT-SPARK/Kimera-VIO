FROM ubuntu:20.04
LABEL maintainer="mabate@mit.edu"

# To avoid tzdata asking for geographic location...

ENV DIRPATH /root/
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils

RUN apt-get update && apt-get install -y git cmake

# Install xvfb to provide a display to container for GUI related testing.
RUN apt-get update && apt-get install -y xvfb

# Install GTSAM
RUN apt-get update && apt-get install -y libboost-all-dev libmetis-dev
ADD https://api.github.com/repos/borglab/gtsam/git/refs/heads/master version.json
RUN git clone https://github.com/borglab/gtsam.git
RUN cd gtsam && \
    git fetch && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_UNSTABLE=ON -DGTSAM_TANGENT_PREINTEGRATION=OFF .. && \
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
      cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -D BUILD_opencv_python=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=OFF \
      -DOPENCV_EXTRA_MODULES_PATH=$DIRPATH/opencv_contrib/modules .. && \
      make -j$(nproc) install

# Install Open_GV
RUN git clone https://github.com/marcusabate/opengv && \
      cd opengv && \
      git apply march_native_disable.patch && \
      mkdir build && \
      cd build && \
      cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      .. && \
      make -j$(nproc) install

# Install DBoW2
COPY .patches/fix_vocab.patch dbow2.patch
RUN git clone https://github.com/dorian3d/DBoW2.git
RUN cd DBoW2 && \
      git apply ../dbow2.patch && \
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
      make -j$(nproc) install

# Install Kimera-VIO
RUN git clone https://github.com/MIT-SPARK/Kimera-VIO.git
RUN cd Kimera-VIO && mkdir build && cd build && cmake .. && make -j$(nproc)
