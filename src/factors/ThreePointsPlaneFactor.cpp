/*
 * ThreePointsPlaneFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "ThreePointsPlaneFactor.h"

using namespace std;

namespace gtsam {

void ThreePointsPlaneFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "RegularPlane3Factor Factor on points " <<
       point1Key_ << ", " <<
       point2Key_ << ", " <<
       point3Key_ << ", and plane " <<
       planeKey_  << "\n";
  this->noiseModel_->print("  noise model: ");
}

}

