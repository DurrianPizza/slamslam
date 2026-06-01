#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

// SlamCommon - 公共头文件

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <map>
#include <string>

// 类型别名
namespace slam {
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
}  // namespace slam

#endif  // SLAM_COMMON_H