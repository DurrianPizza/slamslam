# SLAM框架概述

## 模块架构

```
slam_v1/
├── include/slam/           # 头文件
│   ├── common.h            # 公共类型（Eigen/OpenCV别名）
│   ├── types/              # 数据类型
│   │   ├── frame.h         # 帧数据容器
│   │   ├── map_point.h     # 地图点
│   │   └── key_frame.h     # 关键帧
│   ├── core/               # 核心算法
│   │   ├── detector.h      # 特征检测器
│   │   ├── matcher.h       # 特征匹配器
│   │   └── pose_estimator.h # PnP位姿估计
│   ├── backend/            # 后端优化
│   │   └── map.h          # 地图管理
│   └── frontend/           # 前端
│       └── frontend.h      # 视觉里程计
└── src/                    # 实现文件
```

## 核心类说明

### Frame（帧）
存储一帧图像的所有信息：
- 图像数据
- 特征点位置
- 描述子
- 位姿估计结果

### Detector（检测器）
支持多种特征点算法：
- ORB（Oriented FAST and Rotated BRIEF）
- FAST
- Harris

### Matcher（匹配器）
特征匹配算法：
- Brute Force暴力匹配
- FLANN快速最近邻匹配

### PoseEstimator（位姿估计）
PnP求解器：
- EPnP
- UPnP
- P3P

## 待完成功能

- [ ] 三角化模块
- [ ] BA优化（g2o集成）
- [ ] 闭环检测
- [ ] IMU预积分
