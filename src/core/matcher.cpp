#include "slam/core/matcher.h"

namespace slam {

// ==================== BruteForceMatcher ====================
BruteForceMatcher::BruteForceMatcher(cv::NormTypes norm)
    : FeatureMatcher(std::string("BF")),
      norm_(norm) {
    matcher_ = cv::BFMatcher::create(norm);
}

void BruteForceMatcher::match(const cv::Mat& query_desc, const cv::Mat& train_desc,
                               std::vector<cv::DMatch>& matches) {
    matcher_->match(query_desc, train_desc, matches);
}

void BruteForceMatcher::knnMatch(const cv::Mat& query_desc, const cv::Mat& train_desc,
                                  std::vector<std::vector<cv::DMatch>>& matches, int k) {
    matcher_->knnMatch(query_desc, train_desc, matches, k);
}

// ==================== FlannMatcher ====================
FlannMatcher::FlannMatcher()
    : FeatureMatcher(std::string("FLANN")) {
    matcher_ = cv::FlannBasedMatcher::create();
}

void FlannMatcher::match(const cv::Mat& query_desc, const cv::Mat& train_desc,
                          std::vector<cv::DMatch>& matches) {
    matcher_->match(query_desc, train_desc, matches);
}

void FlannMatcher::knnMatch(const cv::Mat& query_desc, const cv::Mat& train_desc,
                             std::vector<std::vector<cv::DMatch>>& matches, int k) {
    matcher_->knnMatch(query_desc, train_desc, matches, k);
}

// ==================== 静态方法 ====================
void FeatureMatcher::ratioTest(std::vector<std::vector<cv::DMatch>>& knn_matches,
                                std::vector<cv::DMatch>& good_matches,
                                float ratio) {
    for (const auto& knn_match : knn_matches) {
        if (knn_match.size() >= 2) {
            if (knn_match[0].distance < ratio * knn_match[1].distance) {
                good_matches.push_back(knn_match[0]);
            }
        }
    }
}

}  // namespace slam