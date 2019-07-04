/*
 * ParallelPlaneRegularFactor.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Antoni Rosinol
 */

#include "ParallelPlaneRegularFactor.h"

namespace gtsam {

//***************************************************************************
void ParallelPlaneRegularFactor::print(const std::string& s,
                                       const KeyFormatter& keyFormatter) const {
  std::cout << "ParallelPlaneRegularFactor of type " << this->factor_type_
            << " acting on plane " << plane1Key_ << ", and plane " << plane2Key_
            << "\n";
  this->noiseModel_->print("  noise model: ");
}

}  // namespace gtsam
