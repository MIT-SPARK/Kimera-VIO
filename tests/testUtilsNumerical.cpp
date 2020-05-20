/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testUtilsNumerical.h
 * @brief  test UtilsNumerical
 * @author Antoni Rosinol
 */

#include <algorithm>
#include <string>
#include <vector>

#include <cmath>
#include <utility>

#include <gtest/gtest.h>

#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

static constexpr double x_ = 1.141516;
static constexpr double tol_ = 1e-7;

TEST(UtilsNumerical, HashPair) {
  int a = 1;
  int b = 2;
  size_t hash_1 = UtilsNumerical::hashPair(std::make_pair(a, b));
  size_t hash_2 = UtilsNumerical::hashPair(std::make_pair(a, b + 1));
  EXPECT_NE(hash_1, hash_2);
  size_t hash_3 = UtilsNumerical::hashPair(std::make_pair(a + 1, b));
  EXPECT_NE(hash_1, hash_3);
  EXPECT_NE(hash_2, hash_3);
  size_t hash_4 = UtilsNumerical::hashPair(std::make_pair(a + 1, b + 1));
  EXPECT_NE(hash_1, hash_4);
  EXPECT_NE(hash_2, hash_4);
  EXPECT_NE(hash_3, hash_4);
}

TEST(UtilsNumerical, HashPairNonCommutativity) {
  int a = 1;
  int b = 2;
  size_t hash_1 = UtilsNumerical::hashPair(std::make_pair(a, b));
  size_t hash_2 = UtilsNumerical::hashPair(std::make_pair(b, a));
  EXPECT_NE(hash_1, hash_2);
}

TEST(UtilsNumerical, HashPairSign) {
  int a = 1;
  int b = 1;
  size_t hash_1 = UtilsNumerical::hashPair(std::make_pair(a, b));
  size_t hash_2 = UtilsNumerical::hashPair(std::make_pair(-a, -b));
  EXPECT_NE(hash_1, hash_2);
  size_t hash_3 = UtilsNumerical::hashPair(std::make_pair(-a, b));
  EXPECT_NE(hash_1, hash_3);
  EXPECT_NE(hash_2, hash_3);
}

TEST(UtilsNumerical, HashPairFloats) {
  float a = 23903.1290f;
  float b = 23903.1290f;
  size_t hash_1 = UtilsNumerical::hashPair(std::make_pair(a, b));
  size_t hash_2 = UtilsNumerical::hashPair(std::make_pair(a, b));
  EXPECT_EQ(hash_1, hash_2);
}

TEST(UtilsNumerical, RoundToDigit2digits) {
  double x_expected = 1.14;  // rounded to the 2nd decimal digit
  double x_actual = UtilsNumerical::RoundToDigit(x_);
  EXPECT_NEAR(x_expected, x_actual, tol_);
}

TEST(UtilsNumerical, RoundToDigit3digits) {
  double x_expected = 1.142;  // rounded to the 3rd decimal digit
  double x_actual = UtilsNumerical::RoundToDigit(x_, 3);
  EXPECT_NEAR(x_expected, x_actual, tol_);
}

TEST(UtilsNumerical, RoundToDigitNeg2digits) {
  double x_expected = -1.14;  // rounded to the 2nd decimal digit
  double x_actual = UtilsNumerical::RoundToDigit(-x_, 2);
  EXPECT_NEAR(x_expected, x_actual, tol_);
}

TEST(UtilsNumerical, RoundToDigitNeg3digits) {
  double x_expected = -1.142;  // rounded to the 3rd decimal digit!
  double x_actual = UtilsNumerical::RoundToDigit(-x_, 3);
  EXPECT_NEAR(x_expected, x_actual, tol_);
}

TEST(UtilsNumerical, ToStringWithPrecisionPos4digits) {
  std::string str_expected("1.142");
  std::string str_actual = UtilsNumerical::To_string_with_precision(x_, 4);
  EXPECT_EQ(str_expected.compare(str_actual), 0);
}

TEST(UtilsNumerical, ToStringWithPrecisionNeg3digits) {
  std::string str_expected("-1.14");
  std::string str_actual = UtilsNumerical::To_string_with_precision(-x_, 3);
  EXPECT_EQ(str_expected.compare(str_actual), 0);
}

TEST(UtilsNumerical, NsecToSec) {
  int64_t timestamp = 12345678;
  double sec_expected = 0.012345678;
  double sec_actual = UtilsNumerical::NsecToSec(timestamp);
  EXPECT_NEAR(sec_expected, sec_actual, tol_);
}

TEST(testUtilsNumerical, VectorUnique) {
  std::vector<int> vactual{1, 2, 3, 1, 2, 3, 3, 4, 5, 4, 5, 6, 7};
  std::vector<int> vexpected{1, 2, 3, 4, 5, 6, 7};

  UtilsNumerical::VectorUnique<int>(vactual);
  UtilsNumerical::PrintVector<int>(vactual, "vactual");
  UtilsNumerical::PrintVector<int>(vexpected, "vexpected");

  EXPECT_EQ(vexpected.size(), vactual.size());
  for (size_t i = 0; i < vexpected.size(); i++) {
    EXPECT_EQ(vexpected[i], vactual[i]);
  }
}

}
