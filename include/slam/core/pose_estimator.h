#pragma once

#include "slam/common.h"

namespace slam {

class Frame;

/**
 * PoseEstimator — PnP/BA位姿估计
 */
class PoseEstimator {
public:
    PoseEstimator();
    ~PoseEstimator() = default;

    // 设置相机内参
    void setCameraIntrinsics(double fx, double fy, double cx, double cy);

    // PnP求解（3D-2D）
    bool solvePnP(const std::vector<Vec3d>& pts_3d,
                  const std::vector<cv::Point2f>& pts_2d,
                  Mat3d& R, Vec3d& t);

    // 使用OpenCV的solvePnP
    bool solvePnPRansac(const std::vector<Vec3d>& pts_3d,
                        const std::vector<cv::Point2f>& pts_2d,
                        Mat3d& R, Vec3d& t,
                        int iterations = 100,
                        double reprojection_error = 2.0,
                        int min_inliers = 20);

    // 三角化（给定两个相机位姿和对应特征点，求3D点）
    static Vec3d triangulate(const Mat4d& Tcw1, const Mat4d& Tcw2,
                             const cv::Point2f& pt1, const cv::Point2f& pt2);

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
};

}  // namespace slam