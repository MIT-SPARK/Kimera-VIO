/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd-definitions.h
 * @brief  Definitions for ImuFrontEnd.h
 * @author Antoni Rosinol
 */

#pragma once

#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>

namespace VIO {

// Inertial containers.
using ImuStamps = Eigen::Matrix<int64_t, Eigen::Dynamic, 1>;
using ImuAccGyr = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Vector6 = gtsam::Vector6;
using Vector3 = gtsam::Vector3;
using ImuBias = gtsam::imuBias::ConstantBias;

} // End of VIO namespace.
