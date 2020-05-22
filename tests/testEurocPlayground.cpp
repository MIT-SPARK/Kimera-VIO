/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testEurocPlayground.cpp
 * @brief  Loads all Euroc GT dataset, including pointclouds, to test/play with algorithms.
 * @author Antoni Rosinol
 */

#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/playground/EurocPlayground.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

TEST(TestEurocPlayground, DISABLED_basicEurocPlayground) {
  EurocPlayground euroc_playground (FLAGS_test_data_path + "/V1_01_easy/",
                                    FLAGS_test_data_path + "/EurocParams",
                                    50, 1000);
  if (FLAGS_display) {
    euroc_playground.visualizeGtData(true, true, true);
  }
}

}

