/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UtilsNumerical.h
 * @brief  Utilities for numerical operations
 * @author Antoni Rosinol
 */

#pragma once

#include <algorithm>
#include <iostream>

#include <boost/functional/hash.hpp>

#include <glog/logging.h>

namespace VIO {

namespace UtilsNumerical {

//! A hash function used to hash a pair of any kind: ORDER MATTERS,
//! so we don't ues XOR (aka ^), we take the boost::hash_combine instead.
//! Hash(a, b) != Hash(b, a)
//! Used for hashing pixel positions of vertices of the mesh 2d.
template <class T1, class T2>
size_t hashPair(const std::pair<T1, T2>& p) {
  size_t seed = 0u;
  boost::hash_combine(seed, p.first);
  boost::hash_combine(seed, p.second);
  return seed;
}

//! A hash function used to hash a pair of any kind: ORDER DOESN'T MATTERS,
//! If you want order invariance just sort your values before accessing/checking
//! Used for hashing faces in a mesh.
template <class T1, class T2, class T3>
size_t hashTriplet(const T1& p1, const T2& p2, const T3& p3) {
  // return ((std::hash<T1>()(p1) ^ (std::hash<T2>()(p2) << 1)) >> 1) ^
  //        (std::hash<T3>()(p3) << 1);
  static constexpr size_t sl = 17191;
  static constexpr size_t sl2 = sl * sl;
  return static_cast<unsigned int>(p1 + p2 * sl + p3 * sl2);
}

//! A hash function used to hash a pair of any kind: ORDER MATTERS,
//! If you want order invariance just sort your values before accessing/checking
//! Used for hashing faces in a mesh.
template <class T1, class T2, class T3>
size_t hashTripletOrderAgnostic(const T1& p1, const T2& p2, const T3& p3) {
  size_t seed = 0u;
  boost::hash_combine(seed, p1);
  boost::hash_combine(seed, p2);
  boost::hash_combine(seed, p3);
  return seed;
}

// Sort vector and remove duplicate elements
template <typename T>
void VectorUnique(std::vector<T>& v) {
  // e.g.: std::vector<int> v{1,2,3,1,2,3,3,4,5,4,5,6,7};
  std::sort(v.begin(), v.end());  // 1 1 2 2 3 3 3 4 4 5 5 6 7
  auto last = std::unique(v.begin(), v.end());
  // v now holds {1 2 3 4 5 6 7 x x x x x x}, where 'x' is indeterminate
  v.erase(last, v.end());
}

//!  Print standard vector with header
template <typename T>
static void PrintVector(const std::vector<T>& vect,
                        const std::string& vector_name) {
  std::cout << vector_name << std::endl;
  for (auto si : vect) std::cout << " " << si;
  std::cout << std::endl;
}

// rounds number to a specified number of decimal digits
// (digits specifies the number of digits to keep AFTER the decimal point)
double RoundToDigit(const double x, const int digits = 2);

// Generate random float using random number generator between -sigma and sigma
double RandomFloatGenerator(const double sigma);

// Converts doulbe to sting with desired number of digits (total number of
// digits)
std::string To_string_with_precision(const double a_value, const int n = 3);

// Converts time from nanoseconds to seconds
double NsecToSec(const std::int64_t& timestamp);

// (NOT TESTED): converts time from seconds to nanoseconds
int64_t SecToNsec(const double timeInSec);

// (NOT TESTED): get current time in seconds
double GetTimeInSeconds();

}  // namespace UtilsNumerical

}  // namespace VIO
