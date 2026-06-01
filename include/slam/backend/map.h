#pragma once

#include "slam/common.h"
#include "slam/types/map_point.h"
#include "slam/types/key_frame.h"

namespace slam {

/**
 * Map — 地图管理器
 *
 * 管理所有地图点和关键帧
 */
class Map {
public:
    Map() = default;
    ~Map() = default;

    // ========== 地图点管理 ==========
    std::shared_ptr<MapPoint> addMapPoint(const Vec3d& pos);
    void addMapPoint(const std::shared_ptr<MapPoint>& mp);

    std::vector<std::shared_ptr<MapPoint>>& getAllMapPoints() { return map_points_; }
    const std::vector<std::shared_ptr<MapPoint>>& getAllMapPoints() const { return map_points_; }

    unsigned long mapPointCount() const { return map_points_.size(); }

    // 删除坏点
    void removeBadMapPoints();

    // ========== 关键帧管理 ==========
    std::shared_ptr<KeyFrame> addKeyFrame(const std::shared_ptr<KeyFrame>& kf);
    void addKeyFrame(const std::shared_ptr<KeyFrame>& kf, double timestamp);

    std::vector<std::shared_ptr<KeyFrame>>& getAllKeyFrames() { return key_frames_; }
    const std::vector<std::shared_ptr<KeyFrame>>& getAllKeyFrames() const { return key_frames_; }

    unsigned long keyFrameCount() const { return key_frames_.size(); }

    // 获取最新关键帧
    std::shared_ptr<KeyFrame> getLatestKeyFrame() {
        return key_frames_.empty() ? nullptr : key_frames_.back();
    }

    // ========== 共视图构建 ==========
    void updateConnections(unsigned long kf_id);

    // ========== 状态 ==========
    void clear() {
        map_points_.clear();
        key_frames_.clear();
    }

private:
    std::vector<std::shared_ptr<MapPoint>> map_points_;
    std::vector<std::shared_ptr<KeyFrame>> key_frames_;
    unsigned long next_map_point_id_ = 0;
};

}  // namespace slam