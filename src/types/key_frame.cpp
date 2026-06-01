#include "slam/types/key_frame.h"
#include "slam/types/map_point.h"

namespace slam {

unsigned long KeyFrame::next_id_ = 0;

KeyFrame::KeyFrame() : id_(next_id_++) {}

std::shared_ptr<KeyFrame> KeyFrame::createKeyFrame() {
    return std::make_shared<KeyFrame>();
}

void KeyFrame::updatePoseMatrices() {
    Rwc_ = Tcw_.block<3, 3>(0, 0).transpose();
    camera_center_ = -Rwc_ * Tcw_.block<3, 1>(0, 3);
}

void KeyFrame::addMapPoint(unsigned long kp_idx, const std::shared_ptr<MapPoint>& mp) {
    if (kp_idx < map_points_.size()) {
        map_points_[kp_idx] = mp;
    }
}

void KeyFrame::addConnection(unsigned long kf_id, int weight) {
    connected_kfs_[kf_id] = weight;
}

}  // namespace slam