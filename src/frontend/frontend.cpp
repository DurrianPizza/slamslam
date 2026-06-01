#include "slam/frontend/frontend.h"
#include "slam/core/detector.h"
#include "slam/core/pose_estimator.h"
#include "slam/types/frame.h"
#include "slam/types/map_point.h"

namespace slam {

Frontend::Frontend() {
    detector_ = std::make_shared<ORBDetector>(500);
}

bool Frontend::addFrame(const cv::Mat& image, double timestamp) {
    cv::Mat gray = image;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    switch (status_) {
        case Status::Init:
            return initialize(gray);
        case Status::Tracking:
            return track(gray);
        case Status::Lost:
            reset();
            return initialize(gray);
    }
    return false;
}

bool Frontend::initialize(const cv::Mat& image) {
    current_frame_ = Frame::createFrame();
    current_frame_->setImage(image);
    current_frame_->setTimestamp(0.0);
    current_frame_->setCameraIntrinsics(fx_, fy_, cx_, cy_);

    detector_->detect(image, current_frame_->getKeypoints());

    if (current_frame_->getKeypoints().size() < 100) {
        return false;
    }

    detector_->compute(image, current_frame_->getKeypoints(), current_frame_->getDescriptors());

    current_Twc_.setIdentity();
    status_ = Status::Tracking;

    return true;
}

bool Frontend::track(const cv::Mat& image) {
    last_frame_ = current_frame_;

    current_frame_ = Frame::createFrame();
    current_frame_->setImage(image);
    current_frame_->setTimestamp(last_frame_->timestamp() + 0.1);
    current_frame_->setCameraIntrinsics(fx_, fy_, cx_, cy_);

    detector_->detect(image, current_frame_->getKeypoints());

    if (current_frame_->getKeypoints().size() < 30) {
        status_ = Status::Lost;
        return false;
    }

    detector_->compute(image, current_frame_->getKeypoints(), current_frame_->getDescriptors());

    std::vector<cv::DMatch> matches;
    detectAndMatch(image, matches);

    if (matches.size() < 20) {
        status_ = Status::Lost;
        return false;
    }

    if (estimatePose(matches)) {
        status_ = Status::Tracking;
        return true;
    } else {
        status_ = Status::Lost;
        return false;
    }
}

int Frontend::detectAndMatch(const cv::Mat& image, std::vector<cv::DMatch>& matches) {
    if (!last_frame_) return 0;

    BruteForceMatcher matcher(cv::NORM_HAMMING);
    matcher.match(last_frame_->getDescriptors(), current_frame_->getDescriptors(), matches);

    return matches.size();
}

bool Frontend::estimatePose(const std::vector<cv::DMatch>& matches) {
    PoseEstimator estimator;
    estimator.setCameraIntrinsics(fx_, fy_, cx_, cy_);

    std::vector<Vec3d> pts_3d;
    std::vector<cv::Point2f> pts_2d;

    for (const auto& match : matches) {
        if (match.queryIdx < last_frame_->getKeypoints().size() &&
            match.trainIdx < current_frame_->getKeypoints().size()) {
            const auto& kp_last = last_frame_->getKeypoints()[match.queryIdx];
            const auto& kp_curr = current_frame_->getKeypoints()[match.trainIdx];

            pts_2d.emplace_back(kp_curr.pt.x, kp_curr.pt.y);

            pts_3d.emplace_back((kp_last.pt.x - cx_) / fx_,
                               (kp_last.pt.y - cy_) / fy_,
                               1.0);
        }
    }

    Mat3d R;
    Vec3d t;
    bool success = estimator.solvePnPRansac(pts_3d, pts_2d, R, t);

    if (success) {
        current_Twc_.block<3, 3>(0, 0) = R.transpose();
        current_Twc_.block<3, 1>(0, 3) = -R.transpose() * t;
    }

    return success;
}

bool Frontend::isKeyFrame() {
    if (num_tracked_points_ < 80) return true;
    if (num_tracked_points_ < 100 && rand() % 100 < 30) return true;
    return false;
}

void Frontend::reset() {
    current_frame_.reset();
    last_frame_.reset();
    status_ = Status::Init;
}

}  // namespace slam