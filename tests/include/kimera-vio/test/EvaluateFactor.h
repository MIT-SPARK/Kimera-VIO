/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testfixtureFactorJacobian.h
 * @brief  Helper function for evaluating jacobians against numericll derivative
 * @author Nathan Hughes
 */

#pragma once
#include <gtsam/base/numericalDerivative.h>

namespace VIO::test {

template <typename Factor, typename V1, typename V2>
void evaluateFactor(const Factor& factor,
                    const V1& v1,
                    const V2& v2,
                    double tol,
                    double delta = 1.0e-5) {
  gtsam::Matrix H1_actual, H2_actual;
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
  factor.evaluateError(v1, v2, H1_actual, H2_actual);
#else
  factor.evaluateError(v1, v2, &H1_actual, &H2_actual);
#endif

  const auto H1_expected = gtsam::numericalDerivative21<gtsam::Vector, V1, V2>(
      [&](const auto& v1, const auto& v2) {
        return factor.evaluateError(v1, v2);
      },
      v1,
      v2,
      delta);

  const auto H2_expected = gtsam::numericalDerivative22<gtsam::Vector, V1, V2>(
      [&](const auto& v1, const auto& v2) {
        return factor.evaluateError(v1, v2);
      },
      v1,
      v2,
      delta);

  // Verify the Jacobians are correct
  EXPECT_TRUE(gtsam::assert_equal(H1_expected, H1_actual, tol));
  EXPECT_TRUE(gtsam::assert_equal(H2_expected, H2_actual, tol));
}

}  // namespace VIO::test
