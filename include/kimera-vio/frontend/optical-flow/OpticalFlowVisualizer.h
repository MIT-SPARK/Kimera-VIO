/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OpticalFlowVisualizer.h
 * @brief  Class tho visualize optical flow.
 * @author Antoni Rosinol
 */

#pragma once

#include <math.h>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * @brief The OpticalFlowVisualizer class
 * Taken from
 * https://github.com/opencv/opencv/blob/3.1.0/samples/cpp/tvl1_optical_flow.cpp
 */
class OpticalFlowVisualizer {
 public:
  KIMERA_POINTER_TYPEDEFS(OpticalFlowVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(OpticalFlowVisualizer);

 public:
  OpticalFlowVisualizer() = default;
  virtual ~OpticalFlowVisualizer() = default;

 public:
  static void drawOpticalFlow(const cv::Mat_<cv::Point2f>& flow,
                              cv::Mat& dst,
                              float max_motion = -1) {
    dst.create(flow.size(), CV_8UC3);
    dst.setTo(cv::Scalar::all(0));
    // Determine motion range:
    float max_rad = max_motion;
    if (max_motion <= 0.0f) {
      max_rad = 1.0f;
      for (int y = 0; y < flow.rows; ++y) {
        for (int x = 0; x < flow.cols; ++x) {
          cv::Point2f u = flow(y, x);
          if (!isFlowCorrect(u)) {
            continue;
          }
          max_rad = std::max(max_rad, std::sqrt(u.x * u.x + u.y * u.y));
        }
      }
    }

    for (int y = 0; y < flow.rows; ++y) {
      for (int x = 0; x < flow.cols; ++x) {
        cv::Point2f u = flow(y, x);
        if (isFlowCorrect(u))
          dst.at<cv::Vec3b>(y, x) = computeColor(u.x / max_rad, u.y / max_rad);
      }
    }
  }

  static cv::Mat drawOpticalFlow(const cv::Mat_<cv::Point2f>& flow) {
    // Visualization part
    cv::Mat flow_parts[2];
    cv::split(flow, flow_parts);

    // Convert the algorithm's output into Polar coordinates
    cv::Mat magnitude, angle, magn_norm;
    cv::cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    cv::normalize(magnitude, magn_norm, 0.0f, 1.0f, cv::NORM_MINMAX);
    static constexpr float angle_factor = ((1.f / 360.f) * (180.f / 255.f));
    angle *= angle_factor;

    // Build hsv image
    cv::Mat _hsv[3], hsv, hsv8, bgr_flow;
    _hsv[0] = angle;
    _hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    cv::merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);

    // Display the results
    cv::cvtColor(hsv8, bgr_flow, cv::COLOR_HSV2BGR);
    static constexpr bool save = false;
    if (save) {
      std::string save_path = "./optical_flow_frames/frame_flow.jpg";
      cv::imwrite(save_path, bgr_flow);
    }
    return bgr_flow;
  }

  static cv::Mat drawOpticalFlowArrows(const cv::Mat_<cv::Point2f>& flow) {
    cv::Mat bgr = drawOpticalFlow(flow);
    cv::Mat arrowed_flow(flow.size(), CV_8UC3);
    arrowed_flow.setTo(0);
    for (int y = 0; y < flow.rows; y += 10) {
      for (int x = 0; x < flow.cols; x += 10) {
        cv::Point2f u = flow(y, x);
        if (isFlowCorrect(u)) {
          cv::arrowedLine(arrowed_flow,
                          cv::Point2f(x, y),
                          cv::Point2f(x + u.x, y + u.y),
                          bgr.at<cv::Vec3b>(y, x),
                          1);
        }
      }
    }
    return arrowed_flow;
  }

 protected:
  static inline bool isFlowCorrect(cv::Point2f u) {
    return !cvIsNaN(u.x) && !cvIsNaN(u.y) && fabs(u.x) < 1e9 && fabs(u.y) < 1e9;
  }

  static cv::Vec3b computeColor(float fx, float fy) {
    static bool first = true;
    // relative lengths of color transitions:
    // these are chosen based on perceptual similarity
    // (e.g. one can distinguish more shades between red and yellow
    //  than between yellow and green)
    const int RY = 15;
    const int YG = 6;
    const int GC = 4;
    const int CB = 11;
    const int BM = 13;
    const int MR = 6;
    const int NCOLS = RY + YG + GC + CB + BM + MR;
    static cv::Vec3i colorWheel[NCOLS];
    if (first) {
      int k = 0;
      for (int i = 0; i < RY; ++i, ++k)
        colorWheel[k] = cv::Vec3i(255, 255 * i / RY, 0);
      for (int i = 0; i < YG; ++i, ++k)
        colorWheel[k] = cv::Vec3i(255 - 255 * i / YG, 255, 0);
      for (int i = 0; i < GC; ++i, ++k)
        colorWheel[k] = cv::Vec3i(0, 255, 255 * i / GC);
      for (int i = 0; i < CB; ++i, ++k)
        colorWheel[k] = cv::Vec3i(0, 255 - 255 * i / CB, 255);
      for (int i = 0; i < BM; ++i, ++k)
        colorWheel[k] = cv::Vec3i(255 * i / BM, 0, 255);
      for (int i = 0; i < MR; ++i, ++k)
        colorWheel[k] = cv::Vec3i(255, 0, 255 - 255 * i / MR);
      first = false;
    }

    const float rad = std::sqrt(fx * fx + fy * fy);
    const float a = std::atan2(-fy, -fx) / (float)CV_PI;

    const float fk = (a + 1.0f) / 2.0f * (NCOLS - 1);
    const int k0 = static_cast<int>(fk);
    const int k1 = (k0 + 1) % NCOLS;
    const float f = fk - k0;

    cv::Vec3b pix;
    for (int b = 0; b < 3; b++) {
      const float col0 = colorWheel[k0][b] / 255.f;
      const float col1 = colorWheel[k1][b] / 255.f;
      float col = (1 - f) * col0 + f * col1;
      if (rad <= 1) {
        col = 1 - rad * (1 - col);  // increase saturation with radius
      } else {
        col *= .75;  // out of range
      }
      pix[2 - b] = static_cast<uchar>(255.f * col);
    }
    return pix;
  }
};

}  // namespace VIO
