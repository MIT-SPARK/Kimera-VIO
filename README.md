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

Clone this repository: `git clone git@github.mit.edu:lcarlone/VIO.git`

In the root library folder execute (using cmake-gui: if you changed the GTSAM install folder, you may need to redirect VIO to your-gtsam-install-folder/lib/cmake/GTSAM. Similarly,you may need to change the folder for CGAL and OpenGV):

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
$ make
$ make check
```

Note 1a: if you use MKL in gtsam, you may need to add to .bashrc a line similar to: source /opt/intel/parallel_studio_xe_2018/compilers_and_libraries_2018/linux/mkl/bin/mklvars.sh intel64
Note 1b: sometimes you may need to add /usr/local/lib to LD_LIBRARY_PATH in ~/.bashrc (if you get lib not found errors at run or test time)
Note 2: you may have to add %YAML:1.0 as first line in all YAML files :-(
Note 3: we are considering to enable EPI in GTSAM, which will require to set the GTSAM_THROW_CHEIRALITY_EXCEPTION to false (cmake flag).
Note 4: for better performance when using the IMU factors, set GTSAM_TANGENT_PREINTEGRATION to false (cmake flag)

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
======================

stereoVIOEuroc:
- Download [EuRoC](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) dataset. You can just download this [file](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip) to do a first test, which corresponds to the ```MH_01_easy``` EuRoC dataset.
- Add the comment ```%YAML:1.0``` at the top of each .yaml file in the dataset (each folder has one sensor.yaml). You can do this manually or alternatively paste and run the following bash script from within the dataset folder:
```
#!bash
sed -i '1 i\%YAML:1.0' body.yaml
sed -i '1 i\%YAML:1.0' */sensor.yaml
```
You have two ways to start the example:
- Using the script ```stereoVIOEuroc.bash``` inside the ```scripts``` folder:
  - Run: ```./stereoVIOEuroc.bash -p "PATH_TO_DATASET/MH_01_easy"```
  - Optionally, you can try the VIO using structural regularities, as in [Toni's thesis](https://www.research-collection.ethz.ch/handle/20.500.11850/297645), by specifying the option ```-r```: ```./stereoVIOEuroc.bash -p "PATH_TO_DATASET/MH_01_easy" -r```
- Alternatively, have a look inside the script to see how to change extra parameters that control the pipeline itself.

Tips for development
----------------------
- To make the pipeline deterministic:
    - Disable TBB in GTSAM (go to build folder in gtsam, use cmake-gui to unset ```GTSAM_WITH_TBB```).
    - Change ```ransac_randomize``` flag in ```params/trackerParameters.yaml``` to 0, to disable ransac randomization.

> Note: these changes are not sufficient to make the output repeatable between different machines.

> Note to self: remember that we are using ```-march=native``` compiler flag, which will be a problem if we ever want to distribute binaries of this code.
