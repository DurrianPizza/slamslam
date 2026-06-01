#pragma once

#include "slam/common.h"

namespace slam {

class MapPoint;

/**
 * Frame — SLAM系统中每一帧的数据容器
 */
class Frame {
public:
    static std::shared_ptr<Frame> createFrame();

    Frame();
    ~Frame() = default;

    // ID和时间戳
    unsigned long id() const { return id_; }
    double timestamp() const { return timestamp_; }
    void setTimestamp(double ts) { timestamp_ = ts; }

    // 图像
    cv::Mat getImage() const { return image_; }
    void setImage(const cv::Mat& img) { image_ = img.clone(); }

    // 特征点
    std::vector<cv::KeyPoint>& getKeypoints() { return keypoints_; }
    const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints_; }

    cv::Mat& getDescriptors() { return descriptors_; }
    const cv::Mat& getDescriptors() const { return descriptors_; }

    // 位姿
    Mat4d getPose() const { return Tcw_; }
    void setPose(const Mat4d& T);
    Mat3d getRotation() const { return Rwc_; }
    Vec3d getCameraCenter() const { return camera_center_; }

    // 内参
    void setCameraIntrinsics(double fx, double fy, double cx, double cy) {
        fx_ = fx; fy_ = fy; cx_ = cx; cy_ = cy;
    }
    double fx() const { return fx_; }
    double fy() const { return fy_; }
    double cx() const { return cx_; }
    double cy() const { return cy_; }

    // 关键帧
    bool isKeyframe() const { return is_keyframe_; }
    void setKeyframe(bool is_kf) { is_keyframe_ = is_kf; }

private:
    unsigned long id_;
    double timestamp_ = 0.0;

    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    Mat4d Tcw_;
    Mat3d Rwc_;
    Vec3d camera_center_;

    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    bool is_keyframe_ = false;

    static unsigned long next_id_;
};

}  // namespace slam