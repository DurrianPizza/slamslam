#include "slam/core/detector.h"

namespace slam {

// ==================== ORBDetector ====================
ORBDetector::ORBDetector(int n_features, float scale_factor, int n_levels, int edge_threshold)
    : FeatureDetector(std::string("ORB")),
      n_features_(n_features),
      scale_factor_(scale_factor),
      n_levels_(n_levels),
      edge_threshold_(edge_threshold) {
    orb_ = cv::ORB::create(n_features, scale_factor, n_levels, edge_threshold);
}

void ORBDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) {
    orb_->detect(image, kps);
}

void ORBDetector::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) {
    orb_->compute(image, kps, descriptors);
}

// ==================== FASTDetector ====================
FASTDetector::FASTDetector(int threshold, bool nonmax_suppression)
    : FeatureDetector(std::string("FAST")),
      threshold_(threshold),
      nonmax_suppression_(nonmax_suppression) {
    fast_ = cv::FastFeatureDetector::create(threshold, nonmax_suppression);
}

void FASTDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) {
    fast_->detect(image, kps);
}

void FASTDetector::compute(const cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat& descriptors) {
    descriptors.release();
}

// ==================== HarrisDetector ====================
HarrisDetector::HarrisDetector(int block_size, double k, double threshold)
    : FeatureDetector(std::string("Harris")),
      block_size_(block_size),
      k_(k),
      threshold_(threshold) {}

void HarrisDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) {
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    cv::cornerHarris(gray, response_, block_size_, 3, k_);

    cv::threshold(response_, response_, threshold_ * 255, 0, cv::THRESH_BINARY);
    cv::dilate(response_, response_, cv::Mat());

    for (int i = 0; i < response_.rows; i++) {
        for (int j = 0; j < response_.cols; j++) {
            if (response_.at<float>(i, j) > threshold_ * 0.01) {
                kps.emplace_back(cv::Point2f(j, i), 1.0f);
            }
        }
    }
}

void HarrisDetector::compute(const cv::Mat&, std::vector<cv::KeyPoint>&, cv::Mat& descriptors) {
    descriptors.release();
}

}  // namespace slam