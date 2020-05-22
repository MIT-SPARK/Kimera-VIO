/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testMeshOptimization.cpp
 * @brief  test MeshOptimization implementation
 * @author Antoni Rosinol
 */

#include <gtest/gtest.h>
#include <gflags/gflags.h>

#include "kimera-vio/mesh/MeshOptimization.h"

DECLARE_string(test_data_path);

namespace VIO {

class MeshOptimizationFixture : public ::testing::Test {
 public:
  MeshOptimizationFixture() {}

 protected:
  virtual void SetUp() override {}
  virtual void TearDown() override {}

 protected:
  static constexpr double tol = 1e-8;

 private:

};

TEST_F(MeshOptimization, test) {
}

}  // namespace VIO
