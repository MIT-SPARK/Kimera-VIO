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

#include "RegularVioBackEnd.h"

namespace VIO {
RegularVioBackEnd::RegularVioBackEnd(const Pose3 leftCamPose,
                    const Cal3_S2 leftCameraCalRectified,
                    const double baseline,
                    const VioBackEndParams vioParams) :
    VioBackEnd(leftCamPose, leftCameraCalRectified, baseline, vioParams) {
}

} // namespace VIO
