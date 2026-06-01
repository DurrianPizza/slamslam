#include <iostream>
#include <opencv2/opencv.hpp>
#include "slam/common.h"
#include "slam/frontend/frontend.h"
#include "slam/core/detector.h"
#include "slam/core/matcher.h"

int main() {
    // 模拟相机内参（Realsense D435i近似）
    double fx = 385.0, fy = 385.0, cx = 320.0, cy = 240.0;

    // 创建前端
    slam::Frontend frontend;
    frontend.setCameraIntrinsics(fx, fy, cx, cy);
    frontend.setDetectorType("ORB");

    // 读取测试图像（如果有的话）
    // cv::Mat img = cv::imread("test.png");
    // frontend.addFrame(img, 0.0);

    // 演示特征检测
    cv::Mat test_img = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::randu(test_img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

    // ORB检测
    slam::ORBDetector orb_detector(500);
    std::vector<cv::KeyPoint> kps;
    cv::Mat desc;
    orb_detector.detectAndCompute(test_img, kps, desc);

    std::cout << "检测到 " << kps.size() << " 个特征点" << std::endl;
    std::cout << "描述子尺寸: " << desc.size() << std::endl;

    std::cout << "SLAM框架初始化完成!" << std::endl;
    return 0;
}