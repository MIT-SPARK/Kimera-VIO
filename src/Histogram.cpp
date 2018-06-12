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
             cv::Point(bin_w * peaks[i].pos,
                       hist_image->rows),
             cv::Point(bin_w * peaks[i].pos, 0), color);
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
  output.pos = pos;
  output.left_size = left_size;
  output.right_size = right_size;
  output.value = value;
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
    if (peaks[i].value > max_val * peak_per &&
        peaks[i].value > min_support &&
        peaks[i].left_size >= 2 &&
        peaks[i].right_size >= 2) {
      output.push_back(peaks[i]);
    }
  }

  if (display_histogram) {
    VLOG(10) << "Drawing histogram.";
    cv::Mat hist_img = drawHistogram1D(&src, 400, 1024, 256, // WARNING: this number has to coincide with nr of bins...
                                       cv::Scalar(255,255,255), 2, false);

    VLOG(10) << "Drawing peaks.";
    static const cv::Scalar peak_color (0, 0, 255);

    int hist_size = histogram_.rows; // WARNING assumes a 1D Histogram.
    drawPeaks1D(&hist_img, output, hist_size, peak_color, display_histogram);
  }
  return output;
}

/* -------------------------------------------------------------------------- */
std::vector<cv::Point> Histogram::contoursCenter(
    const std::vector<std::vector<cv::Point>>& contours,
    bool centerOfMass,
    int contourIdx,
    bool visualize) const {
  CHECK_EQ(dims_, 2) << "This function only works with 2D histograms.";
  VLOG(10) << "Starting contoursCenter.";
  std::vector<cv::Point> result;
  if (contourIdx > -1) {
    if (centerOfMass) {
      cv::Moments m = cv::moments(contours.at(contourIdx), true);
      result.push_back(cv::Point(m.m10 / m.m00,
                                 m.m01 / m.m00));
    } else {
      cv::Rect rct = cv::boundingRect(contours[contourIdx]);
      result.push_back(cv::Point(rct.x + rct.width / 2 ,
                                 rct.y + rct.height / 2));
    }
  } else {
    if (centerOfMass) {
      for (size_t i = 0; i < contours.size(); i++) {
        cv::Moments m = cv::moments(contours[i], true);
        result.push_back(cv::Point(m.m10/m.m00,
                                   m.m01/m.m00));

      }
    } else {
      for (size_t i = 0; i < contours.size(); i++) {
        cv::Rect rct = cv::boundingRect(contours.at(i));
        result.push_back(cv::Point(rct.x + rct.width / 2 ,
                                   rct.y + rct.height / 2));
      }
    }
  }

  if (visualize) {
    VLOG(10) << "Starting visualizeHistogram2DWithPeaks.";
    visualizeHistogram2DWithPeaks(result);
    VLOG(10) << "Finished visualizeHistogram2DWithPeaks.";
  }

  VLOG(10) << "Finished contoursCenter.";
  return result;
}

/* -------------------------------------------------------------------------- */
std::vector<cv::Point> Histogram::getLocalMaximum2D(int neighbor,
                                                    bool visualize) const {
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";
  if (histogram_.rows == 0 || histogram_.cols == 0) {
    LOG(WARNING) << "Histogram is empty, cannot compute local max 2d."
                 << " Did you calculateHistogram before?";
    return std::vector<cv::Point>();
  }

  VLOG(10) << "Histogram size is: " << histogram_.size;
  // TODO remove hack: we are passing recontructed image of histogram, instead
  // of the histogram itself...
  cv::Mat src; // = histogram_; //Should be histogram, hacking system!
  drawHistogram2D(&src);
  cv::Mat peak_img = src.clone();

  VLOG(10) << "Starting dilate...";
  cv::dilate(peak_img, peak_img, cv::Mat(), cv::Point(-1,-1), neighbor);
  peak_img = peak_img - src;
  VLOG(10) << "Finished dilate.";

  cv::Mat flat_img ;
  VLOG(10) << "Starting erode...";
  cv::erode(src, flat_img, cv::Mat(), cv::Point(-1, -1), neighbor);
  flat_img = src - flat_img;
  VLOG(10) << "Finished erode.";

  cv::threshold(peak_img, peak_img, 0, 255, CV_THRESH_BINARY);
  cv::threshold(flat_img, flat_img, 0, 255, CV_THRESH_BINARY);
  VLOG(10) << "Starting bitwise_not for flat_img...";
  cv::bitwise_not(flat_img, flat_img);
  VLOG(10) << "Finished bitwise_not for flat_img.";

  VLOG(10) << "Starting setTo for peak_img...";
  peak_img.setTo(cv::Scalar::all(255), flat_img);
  VLOG(10) << "Finished setTo for peak_img.";

  VLOG(10) << "Starting bitwise_not for peak_img...";
  cv::bitwise_not(peak_img, peak_img);
  VLOG(10) << "Finished bitwise_not for peak_img.";

  std::vector<std::vector<cv::Point>> contours;
  VLOG(10) << "Starting contours for peak_img...";
  cv::findContours(peak_img, contours,
                   CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  VLOG(10) << "Finished contours for peak_img.";

  return contoursCenter(contours, true, -1, visualize);
}


/* -------------------------------------------------------------------------- */
// Visualize 2D histogram.
void Histogram::visualizeHistogram2DWithPeaks(
    const std::vector<cv::Point>& peaks) const {
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";
  cv::Mat hist_img;
  VLOG(10) << "Starting drawHistogram2D.";
  drawHistogram2D(&hist_img);
  VLOG(10) << "Finished drawHistogram2D.";

  cv::cvtColor(hist_img, hist_img, cv::COLOR_GRAY2RGB);
  for (const cv::Point& peak: peaks) {
    cv::circle(hist_img, peak, 2, cv::Scalar(255,0,0), 1, 8, 0);
  }

  cv::namedWindow("Theta/Distance Histogram", 1);
  cv::imshow("Theta/Distance Histogram", hist_img);
}


/* -------------------------------------------------------------------------- */
void Histogram::drawHistogram2D(cv::Mat* img_output) const {
  CHECK_EQ(dims_, 2) << "This function is meant for 2D histograms.";
  CHECK_NOTNULL(img_output);
  double maxVal = 0;
  cv::minMaxLoc(histogram_, 0, &maxVal, 0, 0);

  static constexpr int scale_theta = 10;
  static constexpr int scale_distance = 10;
  *img_output = cv::Mat::zeros(hist_size_[0] * scale_distance,
      hist_size_[1] * scale_theta, CV_8UC1);
  for (int h = 0; h < hist_size_[0]; h++) {
    for (int s = 0; s < hist_size_[1]; s++) {
      float binVal = histogram_.at<float>(h, s);
      int intensity = cvRound(binVal * 255 / maxVal);
      cv::rectangle(*img_output,
                    cv::Point(h * scale_distance,
                              s * scale_theta),
                    cv::Point((h + 1) * scale_distance - 1,
                              (s + 1) * scale_theta - 1),
                    cv::Scalar::all(intensity),
                    CV_FILLED);
    }
  }
}

} // namespace VIO
