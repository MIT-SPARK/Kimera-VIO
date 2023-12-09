/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FrameCache.cpp
 * @brief  Disk-backed cache for frame features
 * @author Nathan Hughes
 */

#include "kimera-vio/loopclosure/FrameCache.h"

#include <filesystem>

namespace VIO {

template <typename T>
void write(std::ostream& buffer, const T& field) {
  buffer.write(reinterpret_cast<const char*>(&field), sizeof(T));
}

template <typename T>
void read(std::istream& buffer, T& field) {
  buffer.read(reinterpret_cast<char*>(&field), sizeof(T));
}

template <>
void write<cv::KeyPoint>(std::ostream& buffer, const cv::KeyPoint& pt) {
  write(buffer, pt.angle);
  write(buffer, pt.class_id);
  write(buffer, pt.octave);
  write(buffer, pt.pt.x);
  write(buffer, pt.pt.y);
  write(buffer, pt.response);
  write(buffer, pt.size);
}

template <>
void read<cv::KeyPoint>(std::istream& buffer, cv::KeyPoint& pt) {
  read(buffer, pt.angle);
  read(buffer, pt.class_id);
  read(buffer, pt.octave);
  read(buffer, pt.pt.x);
  read(buffer, pt.pt.y);
  read(buffer, pt.response);
  read(buffer, pt.size);
}

template <>
void write<gtsam::Point3>(std::ostream& buffer, const gtsam::Point3& pt) {
  write(buffer, pt.x());
  write(buffer, pt.y());
  write(buffer, pt.z());
}

template <>
void read<gtsam::Vector3>(std::istream& buffer, gtsam::Vector3& pt) {
  read(buffer, pt.x());
  read(buffer, pt.y());
  read(buffer, pt.z());
}

template <>
void write<StatusKeypointCV>(std::ostream& buffer, const StatusKeypointCV& pt) {
  write(buffer, pt.first);
  write(buffer, pt.second);
}

template <>
void read<StatusKeypointCV>(std::istream& buffer, StatusKeypointCV& pt) {
  read(buffer, pt.first);
  read(buffer, pt.second);
}

template <>
void write<cv::Mat>(std::ostream& buffer, const cv::Mat& mat) {
  const auto type = mat.type();
  write(buffer, type);
  write(buffer, mat.dims);
  for (int i = 0; i < mat.dims; ++i) {
    write(buffer, mat.size[i]);
  }

  const size_t bytes = mat.total() * mat.elemSize();
  write(buffer, bytes);
  buffer.write(reinterpret_cast<const char*>(mat.data), bytes);
}

template <>
void read<cv::Mat>(std::istream& buffer, cv::Mat& mat) {
  int type;
  read(buffer, type);
  int dims;
  read(buffer, dims);
  std::vector<int> sizes(dims);
  for (int i = 0; i < dims; ++i) {
    read(buffer, sizes[i]);
  }

  size_t bytes;
  read(buffer, bytes);

  if (sizes.empty()) {
    return;
  }

  mat = cv::Mat(sizes, type);
  CHECK_EQ(bytes, mat.total() * mat.elemSize());
  buffer.read(reinterpret_cast<char*>(mat.data), bytes);
}

template <typename T, typename Alloc>
void write_vec(std::ostream& buffer, const std::vector<T, Alloc>& vec) {
  const auto size = vec.size();
  write(buffer, size);
  for (const auto& v : vec) {
    write(buffer, v);
  }
}

template <typename T, typename Alloc>
void read_vec(std::istream& buffer, std::vector<T, Alloc>& vec) {
  size_t size;
  read(buffer, size);
  vec.resize(size);
  for (size_t i = 0; i < size; ++i) {
    read(buffer, vec[i]);
  }
}

void LCDFrame::saveBytes(std::ostream& buffer) const {
  write(buffer, timestamp_);
  write(buffer, id_);
  write(buffer, id_kf_);
  write_vec(buffer, keypoints_);
  write_vec(buffer, keypoints_3d_);
  write_vec(buffer, descriptors_vec_);
  write(buffer, descriptors_mat_);
  write_vec(buffer, bearing_vectors_);
}

void LCDFrame::loadBytes(std::istream& buffer) {
  read(buffer, timestamp_);
  read(buffer, id_);
  read(buffer, id_kf_);
  read_vec(buffer, keypoints_);
  read_vec(buffer, keypoints_3d_);
  read_vec(buffer, descriptors_vec_);
  read(buffer, descriptors_mat_);
  read_vec(buffer, bearing_vectors_);
}

void StereoLCDFrame::saveBytes(std::ostream& buffer) const {
  LCDFrame::saveBytes(buffer);
  write_vec(buffer, left_keypoints_rectified_);
  write_vec(buffer, right_keypoints_rectified_);
}

void StereoLCDFrame::loadBytes(std::istream& buffer) {
  LCDFrame::loadBytes(buffer);
  read_vec(buffer, left_keypoints_rectified_);
  read_vec(buffer, right_keypoints_rectified_);
}

void LCDFrame::save(std::ostream& buffer) const {
  buffer << "normal\n";
  saveBytes(buffer);
}

void StereoLCDFrame::save(std::ostream& buffer) const {
  buffer << "stereo\n";
  saveBytes(buffer);
}

LCDFrame::Ptr LCDFrame::load(std::istream& buffer) {
  std::string marker;
  std::getline(buffer, marker);
  LCDFrame::Ptr frame;
  if (marker == "stereo") {
    frame = std::make_shared<StereoLCDFrame>();
  } else if (marker == "normal") {
    frame = std::make_shared<LCDFrame>();
  } else {
    LOG(ERROR) << "Unknown frame type: '" << marker << "'";
    return frame;
  }

  frame->loadBytes(buffer);
  return frame;
}

FrameCache::FrameCache() { impl_.reset(new InMemoryCacheImpl()); }

FrameCache::FrameCache(const FrameCacheConfig& config) {
  if (config.max_frames <= 0) {
    impl_.reset(new InMemoryCacheImpl());
  } else {
    impl_.reset(new LRUCacheImpl(config));
  }
}

InMemoryCacheImpl::InMemoryCacheImpl() {}

size_t InMemoryCacheImpl::addFrame(const LCDFrame::Ptr& frame) {
  const auto new_id = frames_.size();
  frame->id_ = new_id;
  frames_.push_back(frame);
  return new_id;
}

LCDFrame::Ptr InMemoryCacheImpl::getFrame(size_t index) const {
  if (index >= frames_.size()) {
    return nullptr;
  }

  return frames_.at(index);
}

size_t InMemoryCacheImpl::size() const { return frames_.size(); }

LRUCacheImpl::LRUCacheImpl(const FrameCacheConfig& conf) : config(conf) {
  std::filesystem::path cache_root(conf.cache_path);
  const auto cache_path = cache_root / conf.cache_name;
  std::error_code err;
  std::filesystem::create_directories(cache_path, err);
  if (err) {
    LOG(FATAL) << "Failed to create cache directory at '" << cache_path.string()
               << "'";
  }
  LOG(WARNING) << "Using disk-backed cache at '" << cache_path.string() << "'";
}

LRUCacheImpl::~LRUCacheImpl() {
  if (!config.remove_cache_on_exit) {
    return;
  }

  std::filesystem::path cache_root(config.cache_path);
  const auto cache_path = cache_root / config.cache_name;
  std::error_code err;
  std::filesystem::remove_all(cache_path, err);
  if (err) {
    LOG(FATAL) << "Failed to clear '" << cache_path.string() << "'";
  }
}

std::string LRUCacheImpl::getCacheFilepath(size_t i) const {
  std::filesystem::path cache_root(config.cache_path);
  const auto cache_path = cache_root / config.cache_name;
  const auto batch_idx = i / config.num_frames_per_file;
  const std::string name = "frames_" + std::to_string(batch_idx) + ".bin";
  return (cache_path / name).string();
}

void LRUCacheImpl::cacheLastFrame() {
  if (!last_added_) {
    return;
  }

  to_archive_.push_back(last_added_);
  if (to_archive_.size() < config.num_frames_per_file) {
    return;
  }

  const auto id_to_archive = to_archive_.front()->id_;
  const auto filepath = getCacheFilepath(id_to_archive);
  std::ofstream fin(filepath, std::ios::binary);
  CHECK(fin.good()) << "invalid file: '" << filepath << "'";
  VLOG(5) << "Archiving " << to_archive_.size() << " frames (starting at "
          << id_to_archive << ") to '" << filepath << "'";
  for (const auto& prev : to_archive_) {
    prev->save(fin);
  }

  to_archive_.clear();
}

size_t LRUCacheImpl::addFrame(const LCDFrame::Ptr& frame) {
  const auto next_id = total_;
  ++total_;
  frame->id_ = next_id;
  VLOG(5) << "Adding frame to cache: " << next_id;
  cacheLastFrame();
  last_added_ = frame;
  return next_id;
}

bool operator<(const LRUCacheImpl::CacheEntry& lhs,
               const LRUCacheImpl::CacheEntry& rhs) {
  return lhs.last_used < rhs.last_used;
}

size_t LRUCacheImpl::getNextSlot() const {
  if (loaded_.size() < static_cast<size_t>(config.max_frames)) {
    const auto new_slot = loaded_.size();
    loaded_.resize(new_slot + 1);
    return new_slot;
  }

  auto iter = std::min_element(
      entries_.begin(), entries_.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.second < rhs.second;
      });
  const auto new_slot = iter->second.slot;
  VLOG(5) << "Evicted entry " << iter->first << " from cache";
  entries_.erase(iter);
  return new_slot;
}

LCDFrame::Ptr LRUCacheImpl::getFrame(size_t index) const {
  if (index >= total_) {
    VLOG(5) << "Attempting to retrieve invalid frame " << index
            << " from cache (total: " << total_ << ")";
    return nullptr;
  }

  if (last_added_ && last_added_->id_ == index) {
    VLOG(5) << "Retrieved frame " << index << " (hit: last)";
    return last_added_;
  }

  if (!to_archive_.empty() && to_archive_.front()->id_ <= index) {
    for (const auto& frame : to_archive_) {
      if (frame->id_ == index) {
        return frame;
      }
    }

    LOG(ERROR) << "Failed to find " << index << " in frames to be archived";
    return nullptr;
  }

  // grab indices relative to batched files
  const auto batch_idx = index / config.num_frames_per_file;
  const auto local_idx = index % config.num_frames_per_file;

  bool miss = false;
  auto iter = entries_.find(batch_idx);
  if (iter == entries_.end()) {
    size_t new_slot = getNextSlot();
    miss = true;

    auto& to_fill = loaded_.at(new_slot);
    to_fill.resize(config.num_frames_per_file);
    const auto filepath = getCacheFilepath(index);
    VLOG(5) << "Loading batch from '" << filepath << "' for frame " << index;
    std::ifstream fin(filepath, std::ios::binary);
    CHECK(fin.good()) << "invalid filepath: '" << filepath << "'";
    for (auto& frame : to_fill) {
      frame = LCDFrame::load(fin);
    }

    CacheEntry entry{new_slot, total_};
    iter = entries_.emplace(batch_idx, entry).first;
  }

  VLOG(5) << "Retrieved frame " << index << (miss ? " (miss)" : " (hit)");
  iter->second.last_used = total_;
  return loaded_.at(iter->second.slot).at(local_idx);
}

size_t LRUCacheImpl::size() const { return total_; }

}  // namespace VIO
