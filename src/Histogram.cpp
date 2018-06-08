/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Histogram.cpp
 * @brief  Histogram class to compute multi-dimensional histogram.
 * @author Antoni Rosinol
 */

#include "Histogram.h"

#include <glog/logging.h>


namespace VIO {

/* -------------------------------------------------------------------------- */
Histogram::Histogram(const int& n_images,
                     const int* channels,
                     const cv::Mat& mask,
                     const int& dims,
                     const int* hist_size,
                     const float** ranges,
                     const bool& uniform,
                     const bool& accumulate)
  : n_images_(n_images),
    channels_(channels),
    mask_(mask),
    dims_(dims),
    hist_size_(hist_size),
    ranges_(ranges),
    uniform_(uniform),
    accumulate_(accumulate) {}

/* -------------------------------------------------------------------------- */
void Histogram::calculateHistogram(const cv::Mat& input) {
  cv::calcHist(&input, n_images_, channels_, mask_,
               histogram_, dims_, hist_size_, ranges_,
               uniform_, accumulate_);
}

/* -------------------------------------------------------------------------- */
//void Histogram::print1DHistogram() {
//  CHECK_EQ(dims_, 1);
//
//
//}

/* -------------------------------------------------------------------------- */
int Histogram::drawPeaks(cv::Mat &histImage,
                         std::vector<int>& peaks,
                         int hist_size,
                         cv::Scalar color,
                         bool display_image) {
  CHECK_NE(hist_size, 0);
  int bin_w = cvRound((double) histImage.cols / hist_size);
  for (size_t i = 0; i < peaks.size(); i++) {
    cv::line(histImage,
             cv::Point(bin_w * peaks[i],
                       histImage.rows),
             cv::Point(bin_w * peaks[i], 0), color);
  }

  if (display_image) {
    cv::imshow("Peaks", histImage);
  }

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
cv::Mat Histogram::drawHistogram(cv::Mat &hist,
                                 int hist_h,
                                 int hist_w,
                                 int hist_size,
                                 cv::Scalar color,
                                 int type,
                                 bool display_image) {
  CHECK_NE(hist_size, 0);
  int bin_w = cvRound((double) hist_w / hist_size);

  cv::Mat histImage (hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));

  /// Normalize the result to [ 0, histImage.rows ]
  VLOG(0) << "Normalize image hist";
  cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

  switch (type) {
    case 1:
      for(int i = 0; i < histImage.cols; i++) {
        const unsigned x = i;
        const unsigned y = hist_h;

        cv::line(histImage,
                 cv::Point(bin_w * x, y),
                 cv::Point(bin_w * x, y - cvRound(hist.at<float>(i))), color);
      }
      break;
    case 2:
      for(int i = 1; i < hist_size; ++i) {
        cv::Point pt1 = cv::Point(bin_w * (i-1),
                                  hist_h);
        cv::Point pt2 = cv::Point(bin_w * i,
                                  hist_h);
        cv::Point pt3 = cv::Point(bin_w * i,
                                  hist_h - cvRound(hist.at<float>(i)));
        cv::Point pt4 = cv::Point(bin_w * (i-1),
                                  hist_h - cvRound(hist.at<float>(i-1)));
        cv::Point pts[] = {pt1, pt2, pt3, pt4, pt1};

        cv::fillConvexPoly(histImage, pts, 5, color);
      }
      break;
    default:
      for( int i = 1; i < hist_size; ++i) {
        cv::line(histImage,
                 cv::Point(bin_w * (i-1),
                           hist_h - cvRound(hist.at<float>(i-1))),
                 cv::Point(bin_w * (i),
                           hist_h - cvRound(hist.at<float>(i))),
                 color, 1, 8, 0);
      }
      break;
  }

  if (display_image) {
    cv::imshow("Histogram", histImage);
  }

  return histImage;
}

/* -------------------------------------------------------------------------- */
Histogram::PeakInfo Histogram::peakInfo(int pos, int left_size,
                                        int right_size,
                                        float value) {
  PeakInfo output;
  output.pos = pos;
  output.left_size = left_size;
  output.right_size = right_size;
  output.value = value;
  return output;
}

/* -------------------------------------------------------------------------- */
std::vector<Histogram::PeakInfo> Histogram::findPeaks(cv::InputArray _src,
                                                      int window_size) {
  cv::Mat src = _src.getMat();

  cv::Mat slope_mat = src.clone();

  // Transform initial matrix into 1channel, and 1 row matrix
  cv::Mat src2 = src.reshape(1, 1);

  int size = window_size / 2;

  Length up_hill, down_hill;
  std::vector<PeakInfo> output;

  int pre_state = 0;
  int i = size;

  while(i < src2.cols - size) {
    float cur_state = src2.at<float>(i + size) - src2.at<float>(i - size);

    if(cur_state > 0)
      cur_state = 2;
    else if(cur_state < 0)
      cur_state = 1;
    else cur_state = 0;

    // In case you want to check how the slope looks like
    slope_mat.at<float>(i) = cur_state;

    if (pre_state == 0 && cur_state == 2) {
      up_hill.pos1 = i;
    } else if (pre_state == 2 && cur_state == 1) {
      up_hill.pos2 = i - 1;
      down_hill.pos1 = i;
    }

    if ((pre_state == 1 && cur_state == 2) ||
        (pre_state == 1 && cur_state == 0)) {
      down_hill.pos2 = i - 1;
      int max_pos = up_hill.pos2;
      if(src2.at<float>(up_hill.pos2) <
         src2.at<float>(down_hill.pos1)) {
        max_pos = down_hill.pos1;
      }

      PeakInfo peak_info = peakInfo(max_pos,
                                    up_hill.size(), down_hill.size(),
                                    src2.at<float>(max_pos));

      output.push_back(peak_info);
    }
    i++;
    pre_state = (int)cur_state;
  }
  return output;
}

/* -------------------------------------------------------------------------- */
// If you play with the peak_per attribute value, you can increase/decrease the
// number of peaks found.
// cv::Point smooth_size: x: smoothing size in x, y: smoothing size in y.
// Dilates, does a difference of images, takes contour, find center of mass
// and that is the max, definitely not the best way...
std::vector<int> Histogram::getLocalMaximum(cv::Size smooth_size,
                                            int neighbor_size,
                                            float peak_per,
                                            bool display_histogram) {
  VLOG(0) << "Cloning histogram.";
  cv::Mat src = histogram_.clone();

  VLOG(0) << "Gaussian blur.";
  std::vector<int> output;
  CHECK_GE(histogram_.rows, smooth_size.height);
  CHECK_GE(histogram_.cols, smooth_size.width);
  //cv::GaussianBlur(InputArray src, OutputArray dst, Size ksize,
  //                 double sigmaX, double sigmaY = 0,
  //                 int borderType = BORDER_DEFAULT );
  cv::GaussianBlur(src, src, smooth_size, 0);

  VLOG(0) << "Find peaks.";
  std::vector<PeakInfo> peaks = findPeaks(src, neighbor_size);

  VLOG(0) << "min Max loc.";
  double min_val, max_val;
  cv::minMaxLoc(src, &min_val, &max_val);

  for (size_t i = 0; i < peaks.size(); i++) {
    if (peaks[i].value > max_val * peak_per &&
        peaks[i].left_size >= 2 &&
        peaks[i].right_size >= 2) {
      output.push_back(peaks[i].pos);
    }
  }

  if (display_histogram) {
    VLOG(0) << "Drawing histogram.";
    cv::Mat histImg = drawHistogram(src, 400, 1024, 256,
                                    cv::Scalar(255,255,255), 2, false);

    VLOG(0) << "Drawing peaks.";
    static const cv::Scalar peak_color (0, 0, 255);

    int hist_size = histogram_.rows; // WARNING assumes a 1D Histogram.
    drawPeaks(histImg, output, hist_size, peak_color, display_histogram);
  }
  return output;
}

} // namespace VIO
