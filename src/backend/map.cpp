#include "slam/backend/map.h"
#include "slam/types/key_frame.h"
#include "slam/types/map_point.h"

namespace slam {

// ==================== 地图点管理 ====================
std::shared_ptr<MapPoint> Map::addMapPoint(const Vec3d& pos) {
    auto mp = MapPoint::createMapPoint();
    mp->setPosition(pos);
    addMapPoint(mp);
    return mp;
}

void Map::addMapPoint(const std::shared_ptr<MapPoint>& mp) {
    map_points_.push_back(mp);
}

void Map::removeBadMapPoints() {
    map_points_.erase(
        std::remove_if(map_points_.begin(), map_points_.end(),
                       [](const std::shared_ptr<MapPoint>& mp) { return mp->isBad(); }),
        map_points_.end()
    );
}

// ==================== 关键帧管理 ====================
std::shared_ptr<KeyFrame> Map::addKeyFrame(const std::shared_ptr<KeyFrame>& kf) {
    key_frames_.push_back(kf);
    return kf;
}

void Map::addKeyFrame(const std::shared_ptr<KeyFrame>& kf, double timestamp) {
    kf->setTimestamp(timestamp);
    key_frames_.push_back(kf);
}

void Map::updateConnections(unsigned long kf_id) {
    // 共视关系更新逻辑
    // 遍历所有关键帧，计算共视地图点数量
    if (key_frames_.empty()) return;

    auto current_kf = key_frames_.back();
    for (const auto& kf : key_frames_) {
        if (kf->id() == kf_id) continue;

        int common_points = 0;
        const auto& mps1 = current_kf->getMapPoints();
        const auto& mps2 = kf->getMapPoints();

        for (const auto& mp1 : mps1) {
            for (const auto& mp2 : mps2) {
                if (mp1 == mp2 && mp1 != nullptr) {
                    common_points++;
                }
            }
        }

        if (common_points > 10) {
            current_kf->addConnection(kf->id(), common_points);
            kf->addConnection(kf_id, common_points);
        }
    }
}

}  // namespace slam