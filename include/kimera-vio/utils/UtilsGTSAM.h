/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UtilsGeometry.h
 * @brief  Utilities to deal with GTSAM
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/Values.h>

#include <glog/logging.h>

namespace VIO {

/** \brief Get the estimate of the given gtsam key.
 * \param[in] state set of gtsam values where to find the given key.
 * \param[in] key key to find in the given gtsam state.
 * \param[out] estimate the value of the estimated variable with given key.
 * \return Whether the key could be found or not.
 */
template <class T>
static bool getEstimateOfKey(const gtsam::Values& state,
                             const gtsam::Key& key,
                             T* estimate) {
  if (state.exists(key)) {
    *CHECK_NOTNULL(estimate) = state.at<T>(key);
    return true;
  } else {
    return false;
  }
}

}  // namespace VIO
