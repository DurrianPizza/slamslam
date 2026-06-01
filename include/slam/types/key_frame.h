#pragma once

#include "slam/common.h"

namespace slam {

class MapPoint;

/**
 * KeyFrame — 关键帧
 *
 * 关键帧是从普通帧中筛选出来的"优质帧"
 * 用于地图构建和优化
 */
class KeyFrame {
public:
    static std::shared_ptr<KeyFrame> createKeyFrame();

    KeyFrame();
    ~KeyFrame() = default;

    // ========== 基本属性 ==========
    unsigned long id() const { return id_; }
    double timestamp() const { return timestamp_; }
    void setTimestamp(double ts) { timestamp_ = ts; }

    // ========== 图像 ==========
    cv::Mat getImage() const { return image_; }
    void setImage(const cv::Mat& img) { image_ = img.clone(); }

    // ========== 位姿 (Tcw: 世界→相机) ==========
    Mat4d getPose() const { return Tcw_; }
    void setPose(const Mat4d& T) { Tcw_ = T; updatePoseMatrices(); }
    Vec3d getCameraCenter() const { return camera_center_; }
    Mat3d getRotation() const { return Rwc_; }

    // ========== 特征点 ==========
    std::vector<cv::KeyPoint>& getKeypoints() { return keypoints_; }
    const std::vector<cv::KeyPoint>& getKeypoints() const { return keypoints_; }
    cv::Mat getDescriptors() const { return descriptors_; }
    void setDescriptors(const cv::Mat& desc) { descriptors_ = desc.clone(); }

    // ========== 地图点关联 ==========
    std::vector<std::shared_ptr<MapPoint>>& getMapPoints() { return map_points_; }
    const std::vector<std::shared_ptr<MapPoint>>& getMapPoints() const { return map_points_; }
    void addMapPoint(unsigned long kp_idx, const std::shared_ptr<MapPoint>& mp);

    // ========== 共视图关系 ==========
    std::map<unsigned long, int>& getConnectedKeyFrames() { return connected_kfs_; }
    void addConnection(unsigned long kf_id, int weight);

private:
    void updatePoseMatrices();

    unsigned long id_;
    double timestamp_ = 0.0;

    cv::Mat image_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<std::shared_ptr<MapPoint>> map_points_;

    // 位姿
    Mat4d Tcw_;           // 世界→相机
    Mat3d Rwc_;           // 相机→世界 的旋转
    Vec3d camera_center_; // 相机光心（世界坐标）

    // 共视关系 (keyframe_id -> 共视特征点数量)
    std::map<unsigned long, int> connected_kfs_;

    static unsigned long next_id_;
};

}  // namespace slam