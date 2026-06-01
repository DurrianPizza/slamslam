#include "slam/core/pose_estimator.h"
#include <opencv2/core/eigen.hpp>

namespace slam {

PoseEstimator::PoseEstimator() {
    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
}

void PoseEstimator::setCameraIntrinsics(double fx, double fy, double cx, double cy) {
    camera_matrix_.at<double>(0, 0) = fx;
    camera_matrix_.at<double>(1, 1) = fy;
    camera_matrix_.at<double>(0, 2) = cx;
    camera_matrix_.at<double>(1, 2) = cy;
}

bool PoseEstimator::solvePnP(const std::vector<Vec3d>& pts_3d,
                              const std::vector<cv::Point2f>& pts_2d,
                              Mat3d& R, Vec3d& t) {
    std::vector<cv::Point3f> pts_3d_cv;
    for (const auto& p : pts_3d) {
        pts_3d_cv.emplace_back(p.x(), p.y(), p.z());
    }

    cv::Mat rvec, tvec;
    cv::solvePnP(pts_3d_cv, pts_2d, camera_matrix_, dist_coeffs_, rvec, tvec);

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    cv::cv2eigen(R_cv, R);
    cv::cv2eigen(tvec, t);

    return true;
}

bool PoseEstimator::solvePnPRansac(const std::vector<Vec3d>& pts_3d,
                                    const std::vector<cv::Point2f>& pts_2d,
                                    Mat3d& R, Vec3d& t,
                                    int iterations,
                                    double reprojection_error,
                                    int min_inliers) {
    std::vector<cv::Point3f> pts_3d_cv;
    for (const auto& p : pts_3d) {
        pts_3d_cv.emplace_back(p.x(), p.y(), p.z());
    }

    cv::Mat rvec, tvec;
    cv::solvePnPRansac(pts_3d_cv, pts_2d, camera_matrix_, dist_coeffs_,
                       rvec, tvec, false, iterations, reprojection_error, min_inliers);

    cv::Mat R_cv;
    cv::Rodrigues(rvec, R_cv);
    cv::cv2eigen(R_cv, R);
    cv::cv2eigen(tvec, t);

    return true;
}

Vec3d PoseEstimator::triangulate(const Mat4d& Tcw1, const Mat4d& Tcw2,
                                  const cv::Point2f& pt1, const cv::Point2f& pt2) {
    // 提取相机参数
    Mat3d R1 = Tcw1.block<3, 3>(0, 0);
    Mat3d R2 = Tcw2.block<3, 3>(0, 0);
    Vec3d t1 = Tcw1.block<3, 1>(0, 3);
    Vec3d t2 = Tcw2.block<3, 1>(0, 3);

    // 归一化坐标
    double x1 = (pt1.x - 320) / 500;  // 近似内参
    double y1 = (pt1.y - 240) / 500;
    double x2 = (pt2.x - 320) / 500;
    double y2 = (pt2.y - 240) / 500;

    // 简化的三角化（用于演示）
    Vec3d rays1(-R1.col(2));  // 近似射线方向
    Vec3d rays2(-R2.col(2));

    // 平面交点近似
    double depth = 1.0;
    Vec3d p1 = R1 * rays1 * depth + t1;
    Vec3d p2 = R2 * rays2 * depth + t2;

    return (p1 + p2) * 0.5;
}

}  // namespace slam