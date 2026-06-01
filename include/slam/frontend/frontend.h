#pragma once

#include "slam/common.h"
#include "slam/types/frame.h"
#include "slam/types/map_point.h"
#include "slam/types/key_frame.h"
#include "slam/core/detector.h"
#include "slam/core/matcher.h"

namespace slam {

/**
 * Frontend — 前端（视觉里程计）
 *
 * 处理每一帧：
 * 1. 特征检测与匹配
 * 2. PnP位姿估计
 * 3. 局部地图更新
 * 4. 关键帧判定
 */
class Frontend {
public:
    Frontend();
    ~Frontend() = default;

    // 设置相机内参
    void setCameraIntrinsics(double fx, double fy, double cx, double cy) {
        fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    }

    // 设置特征检测器类型
    void setDetectorType(const std::string& type) { detector_type_ = type; }

    // 处理新帧
    // 返回：true=成功，false=失败
    bool addFrame(const cv::Mat& image, double timestamp);

    // 获取当前位姿
    Mat4d getCurrentPose() const { return current_Twc_; }

    // 获取追踪状态
    enum class Status { Init, Tracking, Lost };
    Status getStatus() const { return status_; }

    // 获取追踪到的地图点数量
    unsigned long getTrackedMapPoints() const { return num_tracked_points_; }

private:
    // 初始化（第一帧）
    bool initialize(const cv::Mat& image);

    // 追踪（后续帧）
    bool track(const cv::Mat& image);

    // 特征检测与匹配
    int detectAndMatch(const cv::Mat& image, std::vector<cv::DMatch>& matches);

    // PnP位姿估计
    bool estimatePose(const std::vector<cv::DMatch>& matches);

    // 判断是否是关键帧
    bool isKeyFrame();

    // 重置
    void reset();

    // 数据
    std::shared_ptr<Frame> current_frame_;
    std::shared_ptr<Frame> last_frame_;

    std::vector<std::shared_ptr<MapPoint>> local_map_points_;

    Mat4d current_Twc_;  // 当前位姿（相机→世界）
    Mat3d current_Rwc_;
    Vec3d current_tcw_;

    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;

    std::string detector_type_ = "ORB";
    std::shared_ptr<FeatureDetector> detector_;

    Status status_ = Status::Init;
    unsigned long num_tracked_points_ = 0;
    int num_inliers_ = 0;
};

}  // namespace slam