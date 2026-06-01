#pragma once

#include "slam/common.h"

namespace slam {

class Frame;

/**
 * FeatureMatcher — 特征点匹配器
 */
class FeatureMatcher {
public:
    virtual ~FeatureMatcher() = default;

    // 匹配两组描述子
    virtual void match(const cv::Mat& desc1, const cv::Mat& desc2, std::vector<cv::DMatch>& matches) = 0;

    // KNN匹配（每个查询取K个最近邻）
    virtual void knnMatch(const cv::Mat& query_desc, const cv::Mat& train_desc,
                          std::vector<std::vector<cv::DMatch>>& matches, int k = 2) = 0;

    // 比率检验筛选（Lowe's ratio test）
    static void ratioTest(std::vector<std::vector<cv::DMatch>>& knn_matches,
                          std::vector<cv::DMatch>& good_matches,
                          float ratio = 0.75f);

    std::string getType() const { return type_; }

protected:
    explicit FeatureMatcher(const std::string& type) : type_(type) {}
    std::string type_;
};

/**
 * BruteForceMatcher — 暴力匹配
 */
class BruteForceMatcher : public FeatureMatcher {
public:
    BruteForceMatcher(cv::NormTypes norm = cv::NORM_HAMMING);

    void match(const cv::Mat& query_desc, const cv::Mat& train_desc,
              std::vector<cv::DMatch>& matches) override;

    void knnMatch(const cv::Mat& query_desc, const cv::Mat& train_desc,
                  std::vector<std::vector<cv::DMatch>>& matches, int k = 2) override;

private:
    cv::NormTypes norm_;
    cv::Ptr<cv::BFMatcher> matcher_;
};

/**
 * FlannMatcher — 快速近似最近邻匹配
 */
class FlannMatcher : public FeatureMatcher {
public:
    FlannMatcher();

    void match(const cv::Mat& query_desc, const cv::Mat& train_desc,
               std::vector<cv::DMatch>& matches) override;

    void knnMatch(const cv::Mat& query_desc, const cv::Mat& train_desc,
                  std::vector<std::vector<cv::DMatch>>& matches, int k = 2) override;

private:
    cv::Ptr<cv::FlannBasedMatcher> matcher_;
};

}  // namespace slam