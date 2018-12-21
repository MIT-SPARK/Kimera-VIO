sudo apt-get update

# Install GTSAM
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
git clone git@bitbucket.org:gtborg/gtsam.git
git fetch && git checkout feature/improvementsIncrementalFilter
mkdir build
cd build
cmake ../
sudo make -j8 install

# Install OpenCV for Ubuntu 18.04
sudo apt-get install build-essential cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libvtk6-dev # Ubuntu 18.04
sudo apt-get install libgtk-3-dev # Ubuntu 18.04
sudo apt-get install libatlas-base-dev gfortran
cd ~/Code
git clone https://github.com/opencv/opencv.git opencv_4_0_0
cd ./opencv
git checkout tags/4.0.0
git clone git@github.com:opencv/opencv_contrib.git opencv_contrib_4_0_0
cd opencv_contrib
git checkout tags/4.0.0
cd ../opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -DWITH_VTK=On \
  -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib_4_0_0/modules ..
sudo make -j8 install

# Install OpenCV Contrib for Ubuntu 18.04
cd ~/Code
mkdir build

# Install open_gv
cd ~/Code
git clone https://github.com/laurentkneip/opengv
cd opengv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release .. # NEEDS TO USE GTSAM's EIGEN!!jhj
sudo make -j8 install

# Install spark_vio
cd ~/Code
git clone git@github.mit.edu:SPARK/VIO.git spark_vio
cd ./spark_vio
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release ..
make -j8

# Install spark_vio_evaluation, requires python
# Make virtualenv? No need really...

# Install spark_vio_evaluation from PyPI
pip install spark_vio_evalution

# Run performance tests
./test_spark_vio_performance.py
