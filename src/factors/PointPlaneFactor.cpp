/*
 * PointPlaneFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "PointPlaneFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void PointPlaneFactor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "BasicRegularPlane3Factor Factor on point " <<
       pointKey_ << ", " <<
       planeKey_  << "\n";
  this->noiseModel_->print("  noise model: ");
}

}

