/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ImuFrontEnd.h
 * @brief  Class managing sequences of IMU measurements
 * @author Luca Carlone
 */

#ifndef ImuFrontEnd_H_
#define ImuFrontEnd_H_

#include <map>
#include <string>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>
#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>

#define IMU_BUFFER_DEBUG_COUT

namespace VIO {

// Inertial containers.
using ImuStamps = Eigen::Matrix<int64_t, Eigen::Dynamic, 1>;
using ImuAccGyr = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Vector6 = gtsam::Vector6;
using Vector3 = gtsam::Vector3;

class ImuFrontEnd
{
public:
  using ImuData = std::map<int64_t, Vector6, std::less<int64_t>, Eigen::aligned_allocator<std::pair<int64_t const, Vector6> > >;

  explicit ImuFrontEnd(int64_t buffer_size_microseconds = 60 * 120 * 1e9) // 120 minutes of history
  : buffer_size_microsec_(buffer_size_microseconds) {}

public:
  /* +++++++++++++++++++++++++++++++ NONCONST FUNCTIONS ++++++++++++++++++++++++++++++++++++ */
  // Insert data according to ordering of timestamp
  inline void insert(int64_t stamp, const Vector6& acc_gyr) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[stamp] = acc_gyr;
    if(buffer_size_microsec_ > 0)
    {
      removeDataBeforeTimestamp_impl(
          buffer_.rbegin()->first - buffer_size_microsec_);
    }
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  /*! @brief Get Values between timestamps.
   *
   * If timestamps are not matched, the values are interpolated. Returns a vector
   * of timestamps and a block matrix with values as columns. Returns empty matrices if not successful.
   */
  std::pair<ImuStamps, ImuAccGyr>
  getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to, bool doInterpolate = true);
  typename ImuData::iterator iterator_equal_or_before(int64_t stamp);
  typename ImuData::iterator iterator_equal_or_after(int64_t stamp);
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  inline void clear(){
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  inline void removeDataBeforeTimestamp(int64_t stamp){
    std::lock_guard<std::mutex> lock(mutex_);
    removeDataBeforeTimestamp_impl(stamp);
  }
  /* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
  inline void removeDataBeforeTimestamp_impl(int64_t stamp){
    auto it = buffer_.lower_bound(stamp);
    buffer_.erase(buffer_.begin(), it);
  }
  /* ---------------------------- CONST FUNCTIONS ------------------------------------------- */
  //! Get timestamps of newest and oldest entry.
  std::tuple<int64_t, int64_t, bool> getOldestAndNewestStamp() const;
  /* --------------------------------------------------------------------------------------- */
  inline size_t size() const{
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.size();
  }
  /* --------------------------------------------------------------------------------------- */
  inline bool empty() const{
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.empty();
  }
  /* --------------------------------------------------------------------------------------- */
  inline void lock() const{
    mutex_.lock();
  }
  /* --------------------------------------------------------------------------------------- */
  inline void unlock() const {
    mutex_.unlock();
  }
  /* --------------------------------------------------------------------------------------- */
  const ImuData& data() const{
    std::lock_guard<std::mutex> lock(mutex_);
    // if (!mutex_.try_lock()) { printf("Call lock() before accessing data\n");}
    /* CHECK(!mutex_.try_lock()) << "Call lock() before accessing data."; */  /* @TODO: shouldn't comment out */
    return buffer_;
  }
  /* --------------------------------------------------------------------------------------- */
protected:
  mutable std::mutex mutex_;
  ImuData buffer_;
  int64_t buffer_size_microsec_ = -1; // Negative means, no fixed size.
  // int64_t buffer_size_microsec_ = 2.0 * pow(10,9);
};

} // namespace VIO

#endif /* ImuFrontEnd_H_ */
