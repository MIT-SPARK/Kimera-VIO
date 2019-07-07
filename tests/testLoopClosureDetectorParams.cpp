/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testLoopClosureDetectorParams.cpp
 * @brief  test LoopClosureDetectorParams
 * @author Marcus Abate, Luca Carlone
 */

#include "LoopClosureDetectorParams.h"
#include <glog/logging.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);

static const double tol = 1e-7;

// Parameters
// TODO: add parameters

/* TEST ideas:
      1) test default constructor to make sure that basic sanity checks pass
      2) test parseYAML to make sure that the defaultLCDParams file does get
          parsed correctly. Specifically, all parameters are defined.
      3) test the virtual void print() method for correct output for defaults.
      4) test parseYAMLLCDParams same way as in test 2 (if test 2 is even
          necessary).
      5) test printLCDParams in the same way as in test 3 (if test 3 is even
          necessary).
*/
