/*
 * RegularPlane3Factor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "RegularPlane3Factor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void BasicRegularPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "BasicRegularPlane3Factor Factor on point " <<
       pointKey_ << ", " <<
       planeKey_  << "\n";
  this->noiseModel_->print("  noise model: ");
}

//***************************************************************************
void RegularPlane3Factor::print(const string& s,
    const KeyFormatter& keyFormatter) const {
  cout << "RegularPlane3Factor Factor on points " <<
       point1Key_ << ", " <<
       point2Key_ << ", " <<
       point3Key_ << ", and plane " <<
       planeKey_  << "\n";
  this->noiseModel_->print("  noise model: ");
}

}

