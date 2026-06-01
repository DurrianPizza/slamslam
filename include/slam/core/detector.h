#pragma once

#include "slam/common.h"

namespace slam {

class MapPoint;

/**
 * FeatureDetector — 特征点检测器基类
 *
 * 设计模式：策略模式
 * 不同的特征检测算法（ORB/FAST/Harris）继承自此类
 */
class FeatureDetector {
public:
    virtual ~FeatureDetector() = default;

    // 检测特征点
    virtual void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) = 0;

    // 计算描述子
    virtual void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) = 0;

    // 检测+计算一气呵成
    void detectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) {
        detect(image, kps);
        compute(image, kps, descriptors);
    }

    // 获取检测器类型名称
    std::string getType() const { return type_; }

protected:
    explicit FeatureDetector(const std::string& type) : type_(type) {}

    std::string type_;  // 检测器类型："ORB", "FAST", "Harris"
};

/**
 * ORB检测器
 */
class ORBDetector : public FeatureDetector {
public:
    ORBDetector(int n_features = 500, float scale_factor = 1.2f, int n_levels = 8, int edge_threshold = 31);

    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) override;
    void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) override;

private:
    int n_features_;
    float scale_factor_;
    int n_levels_;
    int edge_threshold_;

    cv::Ptr<cv::ORB> orb_;  // OpenCV ORB检测器
};

/**
 * FAST检测器
 */
class FASTDetector : public FeatureDetector {
public:
    FASTDetector(int threshold = 30, bool nonmax_suppression = true);

    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) override;
    void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) override;

private:
    int threshold_;
    bool nonmax_suppression_;

    cv::Ptr<cv::FastFeatureDetector> fast_;
};

/**
 * Harris检测器
 */
class HarrisDetector : public FeatureDetector {
public:
    HarrisDetector(int block_size = 2, double k = 0.04, double threshold = 0.01);

    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& kps) override;
    void compute(const cv::Mat& image, std::vector<cv::KeyPoint>& kps, cv::Mat& descriptors) override;

private:
    int block_size_;
    double k_;
    double threshold_;

    cv::Mat response_;  // 存储Harris响应值
};

}  // namespace slam