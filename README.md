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

In the root library folder execute:

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
$ make
$ make check (optional, runs unit tests)
```

Prerequisites:

- [GTSAM](https://bitbucket.org/gtborg/gtsam/overview/) >= 4.0 (Branch: `feature/ImprovementsIncrementalFilter`)
- [OpenCV](https://opencv.org/opencv-3-0.html) >= 3.0
- [OpenGV] Installation instructions below 

Installation of OpenGV
----------------------
- git clone https://github.com/laurentkneip/opengv (I did this in my "home/code/" folder)
- open CMakeLists.txt and set INSTALL_OPENGV to ON (this can be also done using cmake-gui)
- using cmake-gui, set: the eigen version to the GTSAM one (for me: /Users/Luca/borg/gtsam/gtsam/3rdparty/Eigen). if you don't do so, very weird error appear (may be due to GTSAM and OpenGV using different versions of eigen!)
- in the opengv folder do:

```
#!bash
$ mkdir build
$ cd build
$ cmake ../
$ sudo make -j8 install
```


