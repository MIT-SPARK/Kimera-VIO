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
#include "utils/Timer.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

static const double tol = 1e-7;

struct functor {
  int state;
  functor(int state) : state(state) {}
  void operator()() const { std::cout << "In functor run for "; }
};

void func() { std::cout << "In function, run for "; }

/* ************************************************************************** */
TEST(testTimer, testManualTimerEmptyFunction) {
  auto tic = VIO::utils::Timer::tic();
  // Expect an empty measurement to be less than 10 microseconds.
  EXPECT(VIO::utils::Timer::toc<std::chrono::nanoseconds>(tic).count() < 1e4);
}

/* ************************************************************************** */
TEST(testTimer, testManualTimerAlmostEmptyFunction) {
  auto tic = VIO::utils::Timer::tic();
  std::cout << "It works?";
  auto duration = VIO::utils::Timer::toc(tic).count();
  // Expect an almost empty code to take less than 100 microseconds.
  EXPECT(duration < 1e5);
}

/* ************************************************************************** */
TEST(testTimer, testFunctionMeasurer) {
  int dummy = 3;
  auto delta_lambda =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution([&dummy]() {
        dummy *= 2;
        std::cout << "In lambda";
      });
  // Expect that any of these would take less than 1 milliseconds.
  EXPECT(delta_lambda < 1e6);
  auto delta_functor =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution(functor(dummy));
  // Expect that any of these would take less than 1 milliseconds.
  EXPECT(delta_functor < 1e6);
  auto delta_func =
      VIO::utils::Measure<std::chrono::nanoseconds>::execution(func);
  // Expect that any of these would take less than 1 milliseconds.
  EXPECT(delta_func < 1e6);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
