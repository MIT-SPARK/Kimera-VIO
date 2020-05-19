/**
 * @file   SerializationOpenCv.cpp
 * @brief  Serialization functionality for OpenCv types.
 * @author Antoni Rosinol
 */

#pragma once

#include <opencv2/core/core.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/serialization.hpp>

// Add serialization capabilities for cv::Mat
BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat);
namespace boost {
namespace serialization {

template <class Archive>
void save(Archive& ar, const ::cv::Mat& m, const unsigned int version) {
  size_t elem_size = m.elemSize();
  size_t elem_type = m.type();

  ar& m.cols;
  ar& m.rows;
  ar& elem_size;
  ar& elem_type;

  const size_t data_size = m.cols * m.rows * elem_size;
  ar& boost::serialization::make_array(m.ptr(), data_size);
}

template <class Archive>
void load(Archive& ar, ::cv::Mat& m, const unsigned int version) {
  int cols, rows;
  size_t elem_size, elem_type;

  ar& cols;
  ar& rows;
  ar& elem_size;
  ar& elem_type;

  m.create(rows, cols, elem_type);

  size_t data_size = m.cols * m.rows * elem_size;
  ar& boost::serialization::make_array(m.ptr(), data_size);
}

}  // namespace serialization
}  // namespace boost

// Serialization of extra OpenCV classes: cv::Point2f, cv::Point3f, cv::KeyPoint
namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, cv::Point2f& p, const unsigned int version) {
  ar& BOOST_SERIALIZATION_NVP(p.x);
  ar& BOOST_SERIALIZATION_NVP(p.y);
}

template <class Archive>
void serialize(Archive& ar, cv::Point3f& p, const unsigned int version) {
  ar& BOOST_SERIALIZATION_NVP(p.x);
  ar& BOOST_SERIALIZATION_NVP(p.y);
  ar& BOOST_SERIALIZATION_NVP(p.z);
}

template <class Archive>
void serialize(Archive& ar, cv::KeyPoint& k, const unsigned int version) {
  ar& BOOST_SERIALIZATION_NVP(k.pt);
  ar& BOOST_SERIALIZATION_NVP(k.size);
  ar& BOOST_SERIALIZATION_NVP(k.angle);
  ar& BOOST_SERIALIZATION_NVP(k.response);
  ar& BOOST_SERIALIZATION_NVP(k.octave);
  ar& BOOST_SERIALIZATION_NVP(k.class_id);
}

}  // namespace serialization
}  // namespace boost
