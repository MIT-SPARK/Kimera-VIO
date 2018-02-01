/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackEnd.h
 * @brief  Derived class from VioBackEnd which enforces regularity constraints
 * on the factor graph.
 * @author Toni Rosinol
 */

#ifndef RegularVioBackEnd_H_
#define RegularVioBackEnd_H_

#include <VioBackEnd.h>

namespace VIO {

class RegularVioBackEnd: public VioBackEnd
{
  using GenericProjectionFactor = gtsam::GenericStereoFactor<Pose3, Point3>;

  public:
  RegularVioBackEnd(const Pose3 leftCamPose,
                    const Cal3_S2 leftCameraCalRectified,
                    const double baseline,
                    const VioBackEndParams vioParams = VioBackEndParams());

  ~RegularVioBackEnd() = default;

  private:
//    for (i in sf->measured().size) {
//        PosePoint2 measurement_i = meas.at(i)
//
//        Use luca's code in the email.
//
//
//
//
//    }
//  }
// Create a noise model for the pixel error
//
//static SharedNoiseModel model(noiseModel::Unit::Create(2));
//
//
//
//typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
//
//
//
//pointKey = from some map (old_smart_factors_);  // TODO
//
//
//
//Std:vector<StereoPoint2> measurements = smart_factor->measured();
//
//std::vector<boost::shared_ptr<Cal3_S2Stereo> > Kall = sf->calibration();
//
//
//
//sanity check:
//
//keys().size() == measurements.size()
//
//
//
//For(i = 0 .. i< measurements.size();++){
//
//                StereoPoint2 measurement_i = measurements.at(i);
//
//poseKey = sf->keys().at(i);
//
//                boost::shared_ptr<Cal3_S2Stereo>  K = Kall.at(i);
//
//                New_projection_factors_.push_back(ProjectionFactor(measurement_i, model, poseKey, pointKey, K);
//
//}
//
//
//
//complexity:
//
//Does this play well with BackendVIO?
//What happens when we marginalize? (attach right timestamp to landmark.. )
//
//};

} // namespace VIO
#endif /* RegularVioBackEnd_H_ */

