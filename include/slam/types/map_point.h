#pragma once

#include "slam/common.h"

namespace slam {

/**
 * MapPoint — 地图中的3D点
 */
class MapPoint {
public:
    static std::shared_ptr<MapPoint> createMapPoint();

    MapPoint();
    ~MapPoint() = default;

    // 位置（世界坐标系）
    Vec3d getPosition() const { return pos_; }
    void setPosition(const Vec3d& pos) { pos_ = pos; }

    // 观测到这个点的关键帧
    const std::vector<unsigned long>& getObservedFrames() const { return observed_frames_; }
    void addObservation(unsigned long frame_id);

    // 描述子（这个点的最佳描述子）
    cv::Mat getDescriptor() const { return descriptor_; }
    void setDescriptor(const cv::Mat& desc) { descriptor_ = desc.clone(); }

    // 是否是坏点
    bool isBad() const { return is_bad_; }
    void setBad(bool bad) { is_bad_ = bad; }

private:
    unsigned long id_;
    Vec3d pos_;
    cv::Mat descriptor_;
    std::vector<unsigned long> observed_frames_;  // 观测到该点的关键帧ID
    bool is_bad_ = false;

    static unsigned long next_id_;
};

}  // namespace slam