/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Histogram.h
 * @brief  Histogram class to compute multi-dimensional histogram.
 * @author Antoni Rosinol
 */

#pragma once

#include <vector>
#include <opencv2/core.hpp>

// TODO move these to .cpp when removing static functions...
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace VIO {

class Histogram {
public:
  /* ------------------------------------------------------------------------ */
  /** @brief Calculates a histogram of a set of arrays.
   * @param nimages Number of source images. (no idea what it does)
   * @param channels List of the dims channels used to compute the histogram. The first array channels
   * are numerated from 0 to images[0].channels()-1 , the second array channels are counted from
   * images[0].channels() to images[0].channels() + images[1].channels()-1, and so on.
   * @param mask Optional mask. If the matrix is not empty, it must be an 8-bit array of the same size
   * as images[i] . The non-zero mask elements mark the array elements counted in the histogram.
   * @param dims Histogram dimensionality that must be positive and not greater than CV_MAX_DIMS
   * (equal to 32 in the current OpenCV version).
   * @param histSize Array of histogram sizes in each dimension.
   * @param ranges Array of the dims arrays of the histogram bin boundaries in each dimension. When the
   * histogram is uniform ( uniform =true), then for each dimension i it is enough to specify the lower
   * (inclusive) boundary \f$L_0\f$ of the 0-th histogram bin and the upper (exclusive) boundary
   * \f$U_{\texttt{histSize}[i]-1}\f$ for the last histogram bin histSize[i]-1 . That is, in case of a
   * uniform histogram each of ranges[i] is an array of 2 elements. When the histogram is not uniform (
   * uniform=false ), then each of ranges[i] contains histSize[i]+1 elements:
   * \f$L_0, U_0=L_1, U_1=L_2, ..., U_{\texttt{histSize[i]}-2}=L_{\texttt{histSize[i]}-1}, U_{\texttt{histSize[i]}-1}\f$
   * . The array elements, that are not between \f$L_0\f$ and \f$U_{\texttt{histSize[i]}-1}\f$ , are not
   * counted in the histogram.
   * @param uniform Flag indicating whether the histogram is uniform or not (see above).
   * @param accumulate Accumulation flag. If it is set, the histogram is not cleared in the beginning
   * when it is allocated. This feature enables you to compute a single histogram from several sets of
   * arrays, or to update the histogram in time.
   */

  Histogram(const int& n_images,
            const int* channels,
            const cv::Mat& mask,
            const int& dims,
            const int* hist_size,
            const float** ranges,
            const bool& uniform = true,
            const bool& accumulate = false);

  /* ------------------------------------------------------------------------ */
  struct PeakInfo {
    int pos_ = 0;
    int left_size_ = 0;
    int right_size_ = 0;
    // Number of points supporting this peak (it does not need to be an int
    // since we smooth the histogram...)
    float value_ = 0;
    // < Operator will compare the support (# of points) of one peak vs the other.
    // nothing else.
    // Used for std::max_element.
    bool operator<(const PeakInfo& rhd) {
      return (value_ < rhd.value_);
    }

    // Used to remove duplicates.
    bool operator==(const PeakInfo& rhd) {
      return (value_ == rhd.value_ && pos_ == rhd.pos_);
    }
  };

  /* ------------------------------------------------------------------------ */
  struct PeakInfo2D {
    PeakInfo2D (const cv::Point& pos,
                const double& value)
      : pos_(pos),
        value_(value) {}
    cv::Point pos_;
    double value_ = 0;

    // < Operator will compare the support (# of points) of one peak vs the other.
    // nothing else.
    // Used for std::max_element.
    bool operator<(const PeakInfo2D& rhd) {
      return (value_ < rhd.value_);
    }

    // Used to remove duplicates.
    bool operator==(const PeakInfo2D& rhd) {
      return (value_ == rhd.value_ && pos_ == rhd.pos_);
    }
  };

  /* ------------------------------------------------------------------------ */
  // Calculates histogram.
  void calculateHistogram(const cv::Mat& input);

  /* ------------------------------------------------------------------------ */
  // If you play with the peak_per attribute value, you can increase/decrease the
  // number of peaks found.
  // Min_support specifies the minimal number of votes for a bin to be considered
  // a peak.
  std::vector<PeakInfo> getLocalMaximum1D(
      const cv::Size& smooth_size = cv::Size(9, 9),
      int neighbor_size = 3,
      float peak_per = 0.5,
      float min_support = 10,
      bool display_histogram = false) const;

  /* ------------------------------------------------------------------------ */
  // findLocalMaximum for a 2D histogram, it dilates the image and erodes it,
  // then makes the difference and checks centers of countours to get maximums.
  bool getLocalMaximum2D(std::vector<PeakInfo2D>* peaks,
                         const cv::Size& smooth_size,
                         bool visualize = false) const;

private:
  // Should be all const.
  const int n_images_;
  const int* channels_;
  const cv::Mat mask_; // Should be InputArray
  const int dims_;
  const int* hist_size_;
  const float** ranges_;
  const bool uniform_;
  const bool accumulate_;

  // The actual histogram.
  cv::Mat histogram_;

  /* ------------------------------------------------------------------------ */
  struct Length {
    int pos1 = 0;
    int pos2 = 0;
    int size() const {
      return pos2 - pos1 + 1;
    }
  };


  /* ------------------------------------------------------------------------ */
  int drawPeaks1D(cv::Mat* histImage,
                  const std::vector<PeakInfo>& peaks,
                  int hist_size = 256,
                  const cv::Scalar& color = cv::Scalar(0, 0, 255),
                  bool display_image = false) const;

  /* ------------------------------------------------------------------------ */
  // It normalizes the histogram...
  cv::Mat drawHistogram1D(cv::Mat* hist,
                          int hist_h = 400,
                          int hist_w = 1024,
                          int hist_size = 256,
                          const cv::Scalar& color = cv::Scalar(255, 255, 255),
                          int type = 2,
                          bool display_image = false) const;

  /* ------------------------------------------------------------------------ */
  PeakInfo peakInfo(int pos, int left_size, int right_size,
                    float value) const;

  /* ------------------------------------------------------------------------ */
  std::vector<PeakInfo> findPeaks(cv::InputArray _src,
                                  int window_size) const;

//////////////////////////// FOR 2D HISTOGRAM //////////////////////////////////
  /* ------------------------------------------------------------------------ */
  // Visualize 2D histogram.
  void visualizeHistogram2DWithPeaks(const std::vector<PeakInfo2D>& peaks) const;

  /* ------------------------------------------------------------------------ */
  // Draw the histogram in 2D, returns the image of it.
  void drawHistogram2D(cv::Mat* img_output,
                       int scale_theta = 10,
                       int scale_distance = 10) const;

  /* ------------------------------------------------------------------------ */
  // Calculates local max in 2D image.
  int imgRegionalMax(cv::Mat input,
                     int number_of_local_max,
                     float min_support,
                     float min_dist_btw_loc_max,
                     std::vector<PeakInfo2D>* locations) const;
};

} // namespace VIO

