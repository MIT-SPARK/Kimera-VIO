README - VIO: Visual-Inertial Odometry
======================================

What is VIO?
------------

VIO is a library of C++ classes that implement the visual-inertial odometry pipeline described in these papers:
 
 - C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza. On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 - C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza. On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 - L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert. Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.

Quickstart
----------

Clone this repository, including submodules: `git clone --recursive git@github.mit.edu:lcarlone/VIO.git`

In the root library folder execute (using cmake-gui: if you changed the GTSAM install folder, you may need to redrect VIO to your-gtsam-install-folder/lib/cmake/GTSAM. Similarly,you may need to change the folder for CGAL and OpenGV):

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
$ make
$ make check
```

Notea: if you use MKL in gtsam, you may need to add to .bashrc a line similar to: source /opt/intel/parallel_studio_xe_2018/compilers_and_libraries_2018/linux/mkl/bin/mklvars.sh intel64
Note1b: sometimes you may need to add /usr/local/lib to LD_LIBRARY_PATH in ~/.bashrc (if you get lib not found errors at run or test time)
Note2: you may have to add %YAML:1.0 as first line in all YAML files :-(
Note3: we are considering to enable EPI in GTSAM, which will require to set the GTSAM_THROW_CHEIRALITY_EXCEPTION to false (cmake flag).

Prerequisites:

- [GTSAM](https://bitbucket.org/gtborg/gtsam/overview/) >= 4.0 (Branch: `feature/ImprovementsIncrementalFilter`)
- [OpenCV](https://opencv.org/opencv-3-0.html) >= 3.0 (Installation instructions below)
- [OpenGV] Installation instructions below 
- [CGAL] Installation instructions below

Installation of OpenCV
----------------------
- on Mac:
```
#!bash
$ homebrew install vtk (to check)
download opencv3.3.1 from https://opencv.org/releases.html
unzip and go to opencv3.3.1
$ mkdir build
$ cd build
$ cmake ../
$ sudo make install
```
- on Linux:
```
#!bash
$ sudo apt-get install libvtk5-dev   (libvtk6-dev in ubuntu 17.10)
$ sudo apt-get install libgtk2.0-dev 
$ sudo apt-get install pkg-config
download opencv3.3.1 from https://opencv.org/releases.html
unzip and go to opencv3.3.1
$ mkdir build
$ cd build
$ cmake -DWITH_VTK=On ..
$ sudo make -j8 install
$ sudo make -j8 test (optional - quite slow)
```

Installation of OpenGV
----------------------
- git clone https://github.com/laurentkneip/opengv (I did this in my "home/code/" folder)
- (not needed in latest version) open CMakeLists.txt and set INSTALL_OPENGV to ON (this can be also done using cmake-gui)
- using cmake-gui, set: the eigen version to the GTSAM one (for me: /Users/Luca/borg/gtsam/gtsam/3rdparty/Eigen). if you don't do so, very weird error appear (may be due to GTSAM and OpenGV using different versions of eigen!)
- in the opengv folder do:

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
$ sudo make -j8 install
$ sudo make -j8 check
```

Installation of CGAL
----------------------
- download CGAL `https://www.cgal.org/download.html` (I tried CGAL-4.11 on Ubuntu 17.10)
- go to CGAL downloaded folder and execute the following:

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
```
- using cmake-gui enable WithEigen3, click configure, and set the eigen version to the GTSAM one (for me: /Users/Luca/borg/gtsam/gtsam/3rdparty/Eigen)
- using cmake-gui check that CMAKE_BUILD_TYPE is set to 'Release" 
- go back to the build folder and execute the following:

```
#!bash
$ make -j8 
$ sudo make install
```

Glog and Gflags
----------------------
Glog and gflags will be automatically downloaded using cmake unless there is a system-wide installation found.

Running examples
----------------------

stereoVIOEuroc:
- Download [EuRoC](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset. You can just download this [file](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip) to do a first test.
- Add the comment ```%YAML:1.0``` at the top of each .yaml file in the dataset (each folder has one sensor.yaml). You can do this manually or alternatively paste and run the following bash script from within the dataset folder:
```
#!bash
sed -i '1 i\%YAML:1.0' body.yaml
sed -i '1 i\%YAML:1.0' */sensor.yaml
```
- Execute ```stereoVIOEuroc {DATASET_PATH}``` located in your build folder, where ```{DATASET_PATH}``` is the path to a dataset.
