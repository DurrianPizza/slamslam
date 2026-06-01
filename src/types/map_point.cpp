#include "slam/types/map_point.h"

namespace slam {

unsigned long MapPoint::next_id_ = 0;

MapPoint::MapPoint() : id_(next_id_++) {}

std::shared_ptr<MapPoint> MapPoint::createMapPoint() {
    return std::make_shared<MapPoint>();
}

void MapPoint::addObservation(unsigned long frame_id) {
    observed_frames_.push_back(frame_id);
}

}  // namespace slam