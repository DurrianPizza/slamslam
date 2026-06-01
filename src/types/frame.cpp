#include "slam/types/frame.h"

namespace slam {

unsigned long Frame::next_id_ = 0;

Frame::Frame()
    : id_(next_id_++),
      Tcw_(Mat4d::Identity()),
      Rwc_(Mat3d::Identity()) {}

std::shared_ptr<Frame> Frame::createFrame() {
    return std::make_shared<Frame>();
}

void Frame::setPose(const Mat4d& T) {
    Tcw_ = T;
    Rwc_ = T.block<3, 3>(0, 0).transpose();
    camera_center_ = -Rwc_ * T.block<3, 1>(0, 3);
}

}  // namespace slam