/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/*
 * PointPlaneFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "kimera-vio/factors/PointPlaneFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void PointPlaneFactor::print(const string& s,
                             const KeyFormatter& keyFormatter) const {
  std::cout << s << " Factor on point " << keyFormatter(pointKey_)
            << ", and plane " << keyFormatter(planeKey_) << "\n";
  this->noiseModel_->print("  noise model: ");
}

gtsam::NonlinearFactor::shared_ptr PointPlaneFactor::clone() const {
  return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PointPlaneFactor(*this)));
}

}  // namespace gtsam
