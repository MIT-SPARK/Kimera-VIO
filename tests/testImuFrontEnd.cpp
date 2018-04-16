/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testImuBuffer.cpp
 * @brief  test ImuBuffer
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include "Frame.h"
#include "StereoFrame.h"
#include "ImuFrontEnd.h"
#include "test_config.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;
using namespace VIO;
using namespace cv;


/* ************************************************************************* */
// Data
static vector<pair<int64_t, Vector6>> imu_data_test;
static ImuFrontEnd test_buffer;
static int64_t test_start_time;
static int64_t test_end_time;
static const int num_test = 5;
static const int64_t test_time_step = 1000;
static const double tol = 1e-7;

/* ************************************************************************* */
// Helper functions!
void InitializeData() {
  imu_data_test.clear();
  test_start_time = 1000000000;
  test_end_time = test_start_time + (num_test - 1) * test_time_step;
  int64_t test_time = test_start_time;

  // create test data (imu_data_test)
  for (int i = 0; i < num_test; i++) {
    double f = ((double) i) * 0.1234;
    Vector6 imu_data_i;
    for (int t = 0; t < 6; t++) imu_data_i[t] = f + t * 0.01;
    imu_data_test.push_back(make_pair(test_time, imu_data_i));
    test_time += test_time_step;
  }

  // insert imu_data_test to test_buffer!
  for (int i = 0; i < num_test; i++) {
    test_buffer.insert(imu_data_test[i].first, imu_data_test[i].second);
  }
}

/* ************************************************************************* */
TEST(testImuBuffer, insert_empty_clear_size) {
  // Initialize randomness
  srand(0);
  const int repeat_time = 100;

  // repeat many times inserting imu_data in random order.
  for (int t = 0; t < repeat_time; t++) {
    // generate the insertion order
    vector<int> rand_perm;
    for (int i = 0; i < num_test; i++) rand_perm.push_back(i);
    random_shuffle(rand_perm.begin(), rand_perm.end());
    // insert
    ImuFrontEnd buf;
    for (auto idx : rand_perm) {
      buf.insert(imu_data_test[idx].first, imu_data_test[idx].second);
    }
    // check size
    EXPECT(buf.size() == rand_perm.size());
    // check empty
    EXPECT(!buf.empty());
    // Check the data is ordered
    int idx = 0;
    for (auto iter : buf.data()) {
      EXPECT(iter.first == imu_data_test[idx].first);
      EXPECT((imu_data_test[idx].second - iter.second).norm() < tol);
      idx++;
    }
    // check clear
    buf.clear();
    EXPECT(buf.empty());
  }
}

/* ************************************************************************* */
TEST(testImuBuffer, getOldestAndNewestStamp) {
  // Empty Buffer
  {
    ImuFrontEnd empty_buffer;
    int64_t timestamp_oldest, timestamp_newest;
    bool status;
    tie(timestamp_oldest, timestamp_newest, status) =
        empty_buffer.getOldestAndNewestStamp();
    EXPECT(timestamp_oldest == -1);
    EXPECT(timestamp_newest == -1);
    EXPECT(!status);
  }

  // Non-empty buffer;
  {
    int64_t timestamp_oldest_actual, timestamp_newest_actual;
    bool status;
    tie(timestamp_oldest_actual, timestamp_newest_actual, status) =
        test_buffer.getOldestAndNewestStamp();
    int64_t timestamp_oldest_expected = test_start_time;
    int64_t timestamp_newest_expected = test_end_time;
    EXPECT(timestamp_oldest_expected == timestamp_oldest_actual);
    EXPECT(timestamp_newest_expected == timestamp_newest_actual);
    EXPECT(status);
  }
}

/* ************************************************************************* */
TEST(testImuBuffer, removeDataBeforeTimestamp) {
  // remove before intermediate timestamp
  {
    // create simple buffer
    Vector6 constantImuData = Vector6::Zero();
    ImuFrontEnd buf;
    for (size_t i=0; i < 100; i++) { buf.insert(i, constantImuData); }
    // remove
    int64_t time_remove = 2; // should remove 2 entries
    buf.removeDataBeforeTimestamp(time_remove);
    // get newest and oldest
    int64_t timestamp_oldest_actual, timestamp_newest_actual;
    bool status;
    tie(timestamp_oldest_actual, timestamp_newest_actual, status) =
        buf.getOldestAndNewestStamp();
    EXPECT(status == true);
    // oldest time stamp should change (we removed first 2 elements)
    EXPECT(timestamp_oldest_actual == 2);
    // newest time stamp should remain the same
    EXPECT(timestamp_newest_actual == 99);
    // size is reduced
    EXPECT(buf.size() == 98);
  }
  // remove before first timestamp (buf should remain unchanged)
  {
    // create simple buffer
    Vector6 constantImuData = Vector6::Zero();
    ImuFrontEnd buf;
    for (size_t i=0; i < 100; i++) { buf.insert(i, constantImuData); }
    // remove
    int64_t time_remove = 0; // should remove no entry
    buf.removeDataBeforeTimestamp(time_remove);
    // get newest and oldest
    int64_t timestamp_oldest_actual, timestamp_newest_actual;
    bool status;
    tie(timestamp_oldest_actual, timestamp_newest_actual, status) =
        buf.getOldestAndNewestStamp();
    EXPECT(status == true);
    // oldest time stamp should change (we removed first 2 elements)
    EXPECT(timestamp_oldest_actual == 0);
    // newest time stamp should remain the same
    EXPECT(timestamp_newest_actual == 99);
    // size is reduced
    EXPECT(buf.size() == 100);
  }
  // remove before last timestamp (all buffer is removed)
  {
    // create simple buffer
    Vector6 constantImuData = Vector6::Zero();
    ImuFrontEnd buf;
    for (size_t i=0; i < 100; i++) { buf.insert(i, constantImuData); }
    // remove
    int64_t time_remove = 100; // should remove all entries
    buf.removeDataBeforeTimestamp(time_remove);
    // get newest and oldest
    int64_t timestamp_oldest_actual, timestamp_newest_actual;
    bool status;
    tie(timestamp_oldest_actual, timestamp_newest_actual, status) =
        buf.getOldestAndNewestStamp();
    EXPECT(timestamp_oldest_actual == -1);
    EXPECT(timestamp_newest_actual == -1);
    EXPECT(!status);
  }
}

/* ************************************************************************* */
TEST(testImuBuffer, getBetweenValuesInterpolated) {
  // case 1: stamp_from < 0 (returns empty)
  {
    int64_t stamp_from = -1;
    int64_t stamp_to = test_end_time - 1;
    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);
  }
  // case 2: stamp_from > stamp_to (returns empty)
  {
    int64_t stamp_from = test_start_time + 1000;
    int64_t stamp_to = test_start_time + 500;
    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);
  }
  // case 3: buffer.size() < 2
  // I do not quite understand why this deserves a separate check!
  {
    int64_t stamp_from = test_start_time + 1000;
    int64_t stamp_to = test_start_time + 500;
    ImuStamps stamps;
    ImuAccGyr accgyr;

    // buffer.size() == 0
    ImuFrontEnd empty_buffer;
    tie(stamps, accgyr) = empty_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);

    // buffer.size() == 1
    ImuFrontEnd single_buffer;
    single_buffer.insert(imu_data_test[0].first, imu_data_test[0].second);
    tie(stamps, accgyr) = empty_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);
  }
  // case 4: stamp_from < oldest_stamp (returns empty)
  {
    int64_t stamp_from = test_start_time - 1;
    int64_t stamp_to = test_end_time - 1;
    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);
  }
  // case 5: stamp_to > newest_stamp (returns empty)
  {
    int64_t stamp_from = test_start_time + 1;
    int64_t stamp_to = test_end_time + 1;
    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == 0 && accgyr.cols() == 0);
  }
  // case 6: stamp_from == oldest_stamp && stamp_to == newest_stamp
  {
    int64_t stamp_from = test_start_time;
    int64_t stamp_to = test_end_time;
    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == num_test && accgyr.cols() == num_test);
    for (int i = 0; i < num_test; i++) {
      EXPECT(stamps[i] == imu_data_test[i].first);
      EXPECT((accgyr.col(i) - imu_data_test[i].second).norm() < tol);
    }
  }
  // case 7: Interpolation for stamp_from & stamp_to
  {
    const double w_start = 0.35;
    int64_t stamp_from = test_start_time + ((int) (w_start * ((double) test_time_step)));
    cout << "stamp_from = " << stamp_from << endl;
    const double w_end = 0.75;
    int64_t stamp_to = test_end_time - (1 - w_end) * test_time_step;

    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to);
    EXPECT(stamps.rows() == num_test && accgyr.cols() == num_test);

    // verify the correctness of the data excluding the first and the last.
    for (int i = 1; i < num_test - 1; i++) {
      EXPECT(stamps[i] == imu_data_test[i].first);
      EXPECT((accgyr.col(i) - imu_data_test[i].second).norm() < tol);
    }

    // verify the correctness of the interpolation for the first one
    double w_start_fromtime = ((double) (stamp_from - test_start_time)) /((double) test_time_step);
    double w_end_fromtime = 1 - ((double) (test_end_time - stamp_to)) /((double) test_time_step);
    ImuAccGyr accgyr_expected_first = imu_data_test[0].second * (1 - w_start_fromtime)
                + imu_data_test[1].second * w_start_fromtime;

    EXPECT(stamps[0] == stamp_from);
    EXPECT((accgyr_expected_first - accgyr.col(0)).norm() < tol);

    // verify the correctness of the interpolation for the last one
    ImuAccGyr accgyr_expected_last = imu_data_test[num_test - 2].second * (1 - w_end_fromtime)
                + imu_data_test[num_test - 1].second * w_end_fromtime;
    EXPECT(stamps[num_test - 1] == stamp_to);
    EXPECT((accgyr_expected_last - accgyr.col(num_test - 1)).norm() < tol);
  }
  // case 8: no interpolation for stamp_from & stamp_to
  {
    bool doInterpolate = false;
    const double w_start = 0.35;
    int64_t stamp_from = test_start_time + ((int) (w_start * ((double) test_time_step)));
    cout << "stamp_from = " << stamp_from << endl;
    const double w_end = 0.75;
    int64_t stamp_to = test_end_time - (1 - w_end) * test_time_step;

    ImuStamps stamps;
    ImuAccGyr accgyr;
    tie(stamps, accgyr) = test_buffer.getBetweenValuesInterpolated(stamp_from, stamp_to, doInterpolate);
    EXPECT(stamps.rows() == num_test && accgyr.cols() == num_test);

    // verify the correctness of the data excluding the first and the last.
    for (int i = 1; i < num_test - 1; i++) {
      EXPECT(stamps[i] == imu_data_test[i].first);
      EXPECT((accgyr.col(i) - imu_data_test[i].second).norm() < tol);
    }

    // verify the correctness of the interpolation for the first one
    ImuAccGyr accgyr_expected_first = imu_data_test[0].second;
    EXPECT(stamps[0] == stamp_from);
    EXPECT((accgyr_expected_first - accgyr.col(0)).norm() < tol);

    // verify the correctness of the interpolation for the last one
    ImuAccGyr accgyr_expected_last = imu_data_test[num_test - 2].second;
    EXPECT(stamps[num_test - 1] == stamp_to);
    EXPECT((accgyr_expected_last - accgyr.col(num_test - 1)).norm() < tol);
  }
}

/* ************************************************************************* */
int main() {
  // Initialize the data!
  InitializeData();
  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
