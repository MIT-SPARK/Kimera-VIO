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
int Histogram::drawPeaks1D(cv::Mat* hist_image,
                           const std::vector<PeakInfo>& peaks,
                           int hist_size,
                           const cv::Scalar& color,
                           bool display_image) const {
  CHECK_NOTNULL(hist_image);
  CHECK_EQ(dims_, 1) << "This function is meant for 1D histograms.";
  CHECK_NE(hist_size, 0);
  int bin_w = cvRound((double) hist_image->cols / hist_size);
  for (size_t i = 0; i < peaks.size(); i++) {
    cv::line(*hist_image,
             cv::Point(bin_w * peaks[i].pos_,
                       hist_image->rows),
             cv::Point(bin_w * peaks[i].pos_, 0), color);
  }

  if (display_image) {
    cv::imshow("Peaks", *hist_image);
  }

  return EXIT_SUCCESS;
}

/* -------------------------------------------------------------------------- */
cv::Mat Histogram::drawHistogram1D(cv::Mat* hist,
                                   int hist_h,
                                   int hist_w,
                                   int hist_size,
                                   const cv::Scalar& color,
                                   int type,
                                   bool display_image) const {
  CHECK_NOTNULL(hist);
  CHECK_EQ(dims_, 1) << "This function is meant for 1D histograms.";
  CHECK_NE(hist_size, 0);
  int bin_w = cvRound((double) hist_w / hist_size);

  cv::Mat hist_image (hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));

  /// Normalize the result to [ 0, histImage.rows ]
  VLOG(10) << "Normalize image hist.";
  cv::normalize(*hist, *hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());

  switch (type) {
    case 1:
      for(int i = 0; i < hist_image.cols; i++) {
        const unsigned x = i;
        const unsigned y = hist_h;

        cv::line(hist_image,
                 cv::Point(bin_w * x, y),
                 cv::Point(bin_w * x, y - cvRound(hist->at<float>(i))), color);
      }
      break;
    case 2:
      for(int i = 1; i < hist_size; ++i) {
        cv::Point pt1 = cv::Point(bin_w * (i - 1),
                                  hist_h);
        cv::Point pt2 = cv::Point(bin_w * i,
                                  hist_h);
        cv::Point pt3 = cv::Point(bin_w * i,
                                  hist_h - cvRound(hist->at<float>(i)));
        cv::Point pt4 = cv::Point(bin_w * (i - 1),
                                  hist_h - cvRound(hist->at<float>(i - 1)));
        cv::Point pts[] = {pt1, pt2, pt3, pt4, pt1};

        cv::fillConvexPoly(hist_image, pts, 5, color);
      }
      break;
    default:
      for( int i = 1; i < hist_size; ++i) {
        cv::line(hist_image,
                 cv::Point(bin_w * (i - 1),
                           hist_h - cvRound(hist->at<float>(i - 1))),
                 cv::Point(bin_w * (i),
                           hist_h - cvRound(hist->at<float>(i))),
                 color, 1, 8, 0);
      }
      break;
  }

  if (display_image) {
    cv::imshow("Histogram", hist_image);
  }

  return hist_image;
}

/* -------------------------------------------------------------------------- */
Histogram::PeakInfo Histogram::peakInfo(int pos,
                                        int left_size, int right_size,
                                        float value) const {
  CHECK_EQ(dims_, 1) << "This function is meant for 1D histograms.";
  PeakInfo output;
  output.pos_ = pos;
  output.left_size_ = left_size;
  output.right_size_ = right_size;
  output.value_ = value;
  return output;
}

/* -------------------------------------------------------------------------- */
std::vector<Histogram::PeakInfo> Histogram::findPeaks(cv::InputArray _src,
                                                      int window_size) const {
  CHECK_EQ(dims_, 1) << "This function is meant for 1D histograms.";
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
std::vector<Histogram::PeakInfo> Histogram::getLocalMaximum1D(
    const cv::Size& smooth_size,
    int neighbor_size,
    float peak_per,
    float min_support, // Minimal number of votes for a peak.
    bool display_histogram) const {
  CHECK_EQ(dims_, 1) << "This function is meant for 1D histograms.";
  VLOG(10) << "Cloning histogram.";
  cv::Mat src = histogram_.clone();

  VLOG(10) << "Adding gaussian blur to histogram.";
  std::vector<PeakInfo> output;
  CHECK_GE(histogram_.rows, smooth_size.height);
  CHECK_GE(histogram_.cols, smooth_size.width);
  //cv::GaussianBlur(InputArray src, OutputArray dst, Size ksize,
  //                 double sigmaX, double sigmaY = 0,
  //                 int borderType = BORDER_DEFAULT );
  cv::GaussianBlur(src, src, smooth_size, 0);

  VLOG(10) << "Starting to find peaks in histogram...";
  std::vector<PeakInfo> peaks = findPeaks(src, neighbor_size);
  VLOG(10) << "Finished to find peaks in histogram.";

  double min_val, max_val;
  cv::minMaxLoc(src, &min_val, &max_val);

  for (size_t i = 0; i < peaks.size(); i++) {
    if (peaks[i].value_ > max_val * peak_per &&
        peaks[i].value_ > min_support &&
        peaks[i].left_size_ >= 2 &&
        peaks[i].right_size_ >= 2) {
      output.push_back(peaks[i]);
    }
  }

  if (display_histogram) {
    VLOG(10) << "Drawing histogram.";
    int hist_size = histogram_.rows; // WARNING assumes a 1D Histogram.
    cv::Mat hist_img = drawHistogram1D(&src, 400, 1024, hist_size, // WARNING: this number has to coincide with nr of bins...
                                       cv::Scalar(255,255,255), 2, false);

    VLOG(10) << "Drawing peaks.";
    static const cv::Scalar peak_color (0, 0, 255);

    drawPeaks1D(&hist_img, output, hist_size, peak_color, display_histogram);
  }
  return output;
}

/* -------------------------------------------------------------------------- */
// Does not resize the peaks vector, so make sure it is empty before sending.
bool Histogram::getLocalMaximum2D(std::vector<Histogram::PeakInfo2D>* peaks,
                                  const cv::Size& smooth_size,
                                  int number_of_local_max,
                                  int min_support,
                                  int min_dist_btw_loc_max,
                                  bool visualize) {
  CHECK_NOTNULL(peaks);
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";
  if (histogram_.rows == 0 || histogram_.cols == 0) {
    LOG(WARNING) << "Histogram is empty, cannot compute local max 2d."
                 << " Did you calculateHistogram before?";
    return false;
  }

  VLOG(10) << "Histogram size is: " << histogram_.size;
  // TODO remove hack: we are passing recontructed image of histogram, instead
  // of the histogram itself...
  CHECK_GE(histogram_.rows, smooth_size.height);
  CHECK_GE(histogram_.cols, smooth_size.width);
  cv::GaussianBlur(histogram_, histogram_, smooth_size, 0);

  int nr_of_maximums = imgRegionalMax(&histogram_, number_of_local_max,
                                      min_support, min_dist_btw_loc_max,
                                      peaks);
  if (nr_of_maximums > 0) {
    VLOG(0) << "Found local maximum in 2D histogram.";
  }

  if (visualize) {
    VLOG(10) << "Starting visualizeHistogram2DWithPeaks.";
    visualizeHistogram2DWithPeaks(*peaks);
    VLOG(10) << "Finished visualizeHistogram2DWithPeaks.";
  }
  return true;
}

/* -------------------------------------------------------------------------- */
// Visualize 2D histogram.
void Histogram::visualizeHistogram2DWithPeaks(
    const std::vector<PeakInfo2D>& peaks) const {
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";

  cv::Mat hist_img;
  static constexpr int scale_theta = 10;
  static constexpr int scale_distance = 10;
  drawHistogram2D(&hist_img, scale_theta, scale_distance);

  cv::cvtColor(hist_img, hist_img, cv::COLOR_GRAY2RGB);
  cv::bitwise_not(hist_img, hist_img);
  for (const PeakInfo2D& peak: peaks) {
    cv::Point peak_scaled (peak.pos_.x * scale_theta,
                           peak.pos_.y * scale_distance);
    cv::circle(hist_img, peak_scaled, 2, cv::Scalar(255,0,0), -1, 8, 0);
  }

  cv::namedWindow("Theta/Distance Histogram", 1);
  cv::imshow("Theta/Distance Histogram", hist_img);
}


/* -------------------------------------------------------------------------- */
void Histogram::drawHistogram2D(cv::Mat* img_output,
                                int scale_theta,
                                int scale_distance) const {
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";
  CHECK_NOTNULL(img_output);
  double max_val = 0;
  static constexpr bool use_fixed_histogram_height = true;
  if (use_fixed_histogram_height) {
    //max_val = 175;
    max_val = 100;
  } else {
    cv::minMaxLoc(histogram_, 0, &max_val, 0, 0);
  }

  // TODO remove below, it was just to find the max_val ever, to scale the
  // histogram size accordingly.
  //static double very_max_val = 0;
  //if (max_val > very_max_val) {
  //  very_max_val = max_val;
  //}
  //LOG(INFO) <<  "Very MAX value: " << very_max_val;

  *img_output = cv::Mat::zeros(hist_size_[0] * scale_theta,
      hist_size_[1] * scale_distance, CV_8UC1);
  if (max_val == 0) {
    return;
  } else {
    for (int t = 0; t < hist_size_[0]; t++) {
      for (int d = 0; d < hist_size_[1]; d++) {
        float binVal = histogram_.at<float>(t, d);
        int intensity = cvRound(binVal * 255 / max_val);
        cv::rectangle(*img_output,
                      cv::Point(t * scale_theta,
                                d * scale_distance),
                      cv::Point((t + 1) * scale_theta - 1,
                                (d + 1) * scale_distance - 1),
                      cv::Scalar::all(intensity),
                      CV_FILLED);
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// Calculates local max in 2D image.
// Locations is a list of point locations.
int Histogram::imgRegionalMax(cv::Mat* input,
                              int number_of_local_max,
                              float min_support,
                              float min_dist_btw_loc_max,
                              std::vector<PeakInfo2D>* locations) {
  CHECK_NOTNULL(input);
  CHECK_NOTNULL(locations);
  int nFoundLocMax = 0;
  for (int i = 0; i < number_of_local_max; i++) {
    cv::Point location;
    double max_val;
    cv::minMaxLoc(*input, NULL, &max_val, NULL, &location);
    if (max_val > min_support) {
      nFoundLocMax += 1;
      int row = location.y;
      int col = location.x;
      locations->push_back(PeakInfo2D(cv::Point(location.y, location.x),
                                      max_val));
      int r0 = (row - min_dist_btw_loc_max > -1 ?
                  row - min_dist_btw_loc_max : 0);
      int r1 = (row + min_dist_btw_loc_max < input->rows ?
                  row + min_dist_btw_loc_max : input->rows - 1);
      int c0 = (col - min_dist_btw_loc_max > -1 ?
                  col - min_dist_btw_loc_max : 0);
      int c1 = (col + min_dist_btw_loc_max < input->cols ?
                  col + min_dist_btw_loc_max : input->cols - 1);
      for (int r = r0; r <= r1; r++) {
        for (int c = c0; c <= c1; c++) {
          if (std::sqrt(std::pow(r - row, 2) + std::pow(c - col, 2))
              <= min_dist_btw_loc_max) {
            input->at<float>(r, c) = 0.0;
          }
        }
      }
    } else {
      break;
    }
  }
  return nFoundLocMax;
}

} // namespace VIO
