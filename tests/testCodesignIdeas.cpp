/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testCodesignIdeas.h
 * @brief  Not a real unit test..
 * @author Luca Carlone
 */

#include <gtsam/geometry/StereoCamera.h>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"

DECLARE_string(test_data_path);

using namespace std;
using namespace gtsam;
using namespace VIO;
using namespace cv;

TEST(testCodesignIdeas, pixelDisplacementLinearVelocity) {
  // create 3D point
  Point3 point3d = Point3(0, 0, 0);  // (0,0,0)

  // create calibration info:
  // DUO: 413.2022130303619178, 412.8734515665207141, 382.5510651676885345,
  // 248.0045196435552839
  Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(332.385,  // fx(),
                                                332.385,  // fy(),
                                                0.0,      // skew(),
                                                367.452,  // px(),
                                                252.201,  // py()
                                                0.1));
  // K->print("Calibration:\n");

  // create camera 1: 1 meter from ground, pointing downwards
  StereoCamera stereoCam1 =
      StereoCamera(Pose3(Rot3::Rx(M_PI), Point3(0, 0, 1)), K);
  StereoPoint2 sp1 = stereoCam1.project(point3d);

  double fps = 40;  // fps
  // cout << "\n LINEAR VELOCITY" << endl;
  // cout << "\n speed, distanceTraveled, pxDisplacement" << endl;
  for (size_t i = 0; i <= 20; i++) {
    // create camera 2: 1 meter from ground, moving at 10m/s at 40fps
    double speed = double(i);  // m/s (from 0 to 20m/s): DJI Inspire 1, the max
                               // speed is about 22 m/s (50 mph).
    double interFrameTime = 1 / fps;
    double distanceTraveled = speed * interFrameTime;
    StereoCamera stereoCam2 =
        StereoCamera(Pose3(Rot3::Rx(M_PI), Point3(distanceTraveled, 0, 1)), K);
    StereoPoint2 sp2 = stereoCam2.project(point3d);

    double pxDisplacement =
        sqrt(pow(sp1.uL() - sp2.uL(), 2) + pow(sp1.v() - sp2.v(), 2));
    // sp1.print("sp1: \n"); sp2.print("sp2: \n");
    // cout << "distanceTraveled: " << distanceTraveled << " pxDisplacement: "
    // << pxDisplacement << endl;
    // cout << speed << " " << distanceTraveled << " " << pxDisplacement <<
    // endl;
  }
  // cout << "\n ANGULAR VELOCITY" << endl;
  // cout << "\n speed, distanceTraveled, pxDisplacement" << endl;
  for (size_t i = 0; i <= 20; i++) {
    // create camera 2: 1 meter from ground, moving at 10m/s at 40fps
    double angularRate =
        double(i * 10 * M_PI / 180);  // m/s (from 0 to 200deg/s)
    double interFrameTime = 1 / fps;
    double angleTraveled = angularRate * interFrameTime;
    StereoCamera stereoCam2 = StereoCamera(
        Pose3(Rot3::Rx(M_PI) * Rot3::Ry(angleTraveled), Point3(0, 0, 1)), K);
    StereoPoint2 sp2 = stereoCam2.project(point3d);

    double pxDisplacement =
        sqrt(pow(sp1.uL() - sp2.uL(), 2) + pow(sp1.v() - sp2.v(), 2));
    cout << angularRate << " " << angleTraveled << " " << pxDisplacement
         << endl;
    // sp1.print("sp1: \n"); sp2.print("sp2: \n");
  }
}
