/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testTimer.h
 * @brief  test Timer
 * @author Antoni Rosinol
 */

#include <iostream>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "kimera-vio/utils/Timer.h"

struct functor {
  int state;
  explicit functor(int state) : state(state) {}
  void operator()() const { VLOG(1) << "In functor run for "; }
};

void func() { VLOG(1) << "In function, run for "; }

/* ************************************************************************** */
TEST(testTimer, testManualTimerEmptyFunction) {
  // Expect an empty measurement to be less than 10 microseconds.
  auto tic = VIO::utils::Timer::tic();
  EXPECT_LE(VIO::utils::Timer::toc<std::chrono::nanoseconds>(tic).count(), 1e4);
}

/* ************************************************************************** */
TEST(testTimer, testManualTimerAlmostEmptyFunction) {
  // Expect an almost empty code to take less than 100 microseconds.
  auto tic = VIO::utils::Timer::tic();
  VLOG(1) << "Almost null computation. It works?";
  auto duration = VIO::utils::Timer::toc(tic).count();
  EXPECT_LE(duration, 1e5);
}

/* ************************************************************************** */
TEST(testTimer, testFunctionMeasurer) {
  // Expect that measuring lambda call take less than 1 milliseconds.
  int dummy = 3;
  auto delta_lambda =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution([&dummy]() {
        dummy *= 2;
        VLOG(1) << "In lambda";
      });
  EXPECT_LE(delta_lambda, 1e6);
  // Expect that measuring functor call would take less than 1 milliseconds.
  auto delta_functor =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution(functor(dummy));
  EXPECT_LE(delta_functor, 1e6);
  // Expect that measuring empty function would take less than 1 milliseconds.
  auto delta_func =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution(func);
  EXPECT_LE(delta_func, 1e6);
}
