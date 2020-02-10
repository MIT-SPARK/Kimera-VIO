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

#include <glog/logging.h>

namespace VIO {

namespace UtilsNumerical {
// rounds number to a specified number of decimal digits
// (digits specifies the number of digits to keep AFTER the decimal point)
double RoundToDigit(const double x, const int digits = 2);

// Generate random float using random number generator between -sigma and sigma
double RandomFloatGenerator(const double sigma);

// converts doulbe to sting with desired number of digits (total number of
// digits)
std::string To_string_with_precision(const double a_value,
                                            const int n = 3);
// converts time from nanoseconds to seconds
double NsecToSec(const std::int64_t& timestamp);

// (NOT TESTED): converts time from seconds to nanoseconds
int64_t SecToNsec(const double timeInSec);

// (NOT TESTED): get current time in seconds
double GetTimeInSeconds();

}  // namespace UtilsNumerical

}  // namespace VIO
