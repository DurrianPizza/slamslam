# FAST角点检测器详解

## 1. 算法概述

FAST（Features from Accelerated Segment Test）是一种高效的角点检测算法，由Edward Rosten和Tom Drummond在2006年提出。该算法的核心设计目标是**速度**，相比Harris等角点检测器，FAST的速度可以快出数十倍，非常适合对实时性要求高的应用场景（如SLAM、VIO）。

---

## 2. FAST-9/FAST-12原理（分段测试加速）

### 2.1 基本思想

FAST角点检测基于一个简单的观察：**如果一个像素与其周围邻域的像素存在明显差异，那么它可能是角点**。

具体做法是：以候选像素 $p$ 为圆心，选取半径为3的圆环上的16个像素点（编号1-16），如图所示：

```
        1   2   3
    16                  4
15                      5
    14              6
        13  12  11
            p
```

### 2.2 角点判定条件

给定阈值 $t$：

1. 圆环上存在**连续 $n$ 个像素**与中心像素 $p$ 的亮度差超过阈值 $t$
2. FAST-9表示需要连续9个像素
3. FAST-12表示需要连续12个像素

### 2.3 分段测试加速（High-Speed Test）

**核心优化**：直接排除非角点，避免对16个像素全部进行比较。

测试步骤（以FAST-9为例）：

**步骤1**：首先检测1、5、9、13四个位置（每隔90度的点）：

- 如果这4个像素中**至少有3个**与中心 $p$ 的差值都小于阈值 $t$，则 $p$ **不可能是角点**，直接排除。

**原理**：如果角点需要9个连续像素，那么在4个对立像素点（相距90度）中，至少要有3个与中心差异足够大，才能形成连续的弧段。

**步骤2**：只有通过步骤1的候选点，才进行完整的16像素检测。

**效率提升**：根据论文数据，约**75%的非角点**可以在只检查4个像素后被排除。

### 2.4 FAST-9 vs FAST-12

| 版本 | 需要的连续像素数 | 判定更严格 | 假阳性率 |
|------|-----------------|-----------|---------|
| FAST-9 | 9个 | 较宽松 | 较高 |
| FAST-12 | 12个 | 较严格 | 较低 |

---

## 3. 机器学习角落检测（非极大值抑制）

### 3.1 问题

直接使用上述测试会产生大量**密集分布的角点**，并且存在一些**伪角点**（edge corners）。

### 3.2 ID3决策树学习

论文使用**ID3决策树**来学习什么样的像素模式真正构成角点：

**训练过程**：

1. 收集大量训练图像，手动标记角点和非角点
2. 对每个候选像素，提取圆环上16个像素与中心像素的比较结果（大于阈值、等于、小于）
3. 使用ID3算法训练决策树，根据信息增益选择最佳测试位置

**决策树结构**：
- 内部节点：测试某个位置像素与中心的亮度关系
- 叶节点：判断是否为角点

**好处**：将角点判断从"需要连续9/12个像素"转化为**查表操作**，速度极快。

### 3.3 非极大值抑制（Non-Maximum Suppression）

即使使用了决策树，直接检测仍会产生**过多紧密相邻的角点**。

**非极大值抑制步骤**：

1. 对每个检测到的角点，计算其**得分**（采用与HARRIS类似的公式）
2. 若某角点与更近的更高分角点距离小于设定阈值，**抑制该角点**
3. 只保留局部区域内的**响应极大值**

**得分公式**（用于比较）：
$$V = \max\left(\sum_{i \in S_{bright}} (I_p - I_i - t), \sum_{i \in S_{dark}} (I_i - I_p - t)\right)$$

其中 $S_{bright}$ 是亮于中心的像素集合，$S_{dark}$ 是暗于中心的像素集合。

---

## 4. 算法完整流程

```
输入：图像 I，阈值 t，邻域半径 r

输出：角点集合

流程：

1.【预筛选】对图像中每个像素 p：
   a. 检查 1, 5, 9, 13 四个像素
   b. 如果其中至少 3 个与 p 的差异 < t，跳过（不是角点）
   c. 否则，检查完整的 16 像素环

2.【角点判定】
   a. 统计圆环上连续 n 个像素与 p 的差值 > t
   b. FAST-9: n = 9；FAST-12: n = 12
   c. 满足条件则标记为候选角点

3.【ID3决策树分类】（可选）
   a. 使用预训练的决策树进一步过滤

4.【非极大值抑制】
   a. 计算每个候选角点的得分 V
   b. 抑制局部非极大值点

5. 返回最终角点集合
```

---

## 5. OpenCV代码实现

### 5.1 基础用法

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

int main() {
    cv::Mat img = cv::imread("image.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) return -1;

    // 检测FAST角点
    cv::Ptr<cv::FastFeatureDetector> fast =
        cv::FastFeatureDetector::create(
            /*threshold=*/ 20,           // 阈值t
            /*nonmaxSuppression=*/ true, // 非极大值抑制
            /*type=*/ cv::FastFeatureDetector::TYPE_9_16  // FAST-9变体
        );

    std::vector<cv::KeyPoint> keypoints;
    fast->detect(img, keypoints);

    // 绘制角点
    cv::Mat output;
    cv::drawKeypoints(img, keypoints, output, cv::Scalar(0, 255, 0));

    cv::imwrite("fast_corners.jpg", output);
    return 0;
}
```

### 5.2 OpenCV中FAST的TYPE参数

```cpp
// OpenCV 4.x 支持的FAST类型
cv::FastFeatureDetector::TYPE_5_8     // 5个像素中需要3个
cv::FastFeatureDetector::TYPE_7_12    // 7个像素中需要6个
cv::FastFeatureDetector::TYPE_9_16    // 9个像素中需要12个（最常用）
```

### 5.3 在SLAM中的实际使用（ORB-SLAM风格）

```cpp
#include <opencv2/opencv2.hpp>

class ORBExtractor {
    std::vector<cv::Mat> imagePyramid;
    int nFeatures;  // 每层提取的特征点数量

public:
    ORBExtractor(int nFeatures = 2000) : nFeatures(nFeatures) {}

    void extract(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors) {
        // 构建图像金字塔（可选）

        // 1. FAST角点检测（多尺度）
        cv::Ptr<cv::FastFeatureDetector> fast =
            cv::FastFeatureDetector::create(20, true, cv::FastFeatureDetector::TYPE_9_16);

        std::vector<cv::KeyPoint> rawKeypoints;
        fast->detect(image, rawKeypoints);

        // 2. 过滤：只保留响应足够强的角点
        std::vector<cv::KeyPoint> filtered;
        for (const auto& kp : rawKeypoints) {
            if (kp.response > 10.0) {  // 响应阈值
                filtered.push_back(kp);
            }
        }

        // 3. 根据需要截取特征点数量
        if (filtered.size() > nFeatures) {
            std::nth_element(filtered.begin(), filtered.begin() + nFeatures,
                           filtered.end(),
                           [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                               return a.response > b.response;
                           });
            filtered.resize(nFeatures);
        }

        keypoints = std::move(filtered);

        // 4. 计算ORB描述子
        cv::ORB::create()->compute(image, keypoints, descriptors);
    }
};
```

---

## 6. 与Harris角点检测器对比

### 6.1 核心原理对比

| 特性 | FAST | Harris |
|------|------|--------|
| **检测依据** | 像素与邻域环的亮度差异 | 灰度变化的梯度矩阵 |
| **数学基础** | 几何拓扑 | 矩阵特征值分析 |
| **计算复杂度** | O(1) 近似 | O(n²) 或 O(n log n) |
| **是否需要迭代** | 否 | 可能需要 |

### 6.2 性能对比

| 指标 | FAST | Harris |
|------|------|--------|
| **检测速度** | 极快（10-100x） | 较慢 |
| **角点质量** | 中等 | 较高 |
| **旋转不变性** | 部分（取决于圆环） | 较好 |
| **光照鲁棒性** | 依赖阈值设置 | 依赖阈值设置 |
| **边缘响应** | 可能检测到伪角点 | 较少 |

### 6.3 主观对比图示

```
Harris: 角点响应平滑，边缘清晰
        ___________
       |           |
       |     *     |  <- 角点
       |___________|

FAST: 直接比较像素亮度差异
       ○ ○ ○
      ○     ○
     ○   p   ○  <- 中心像素与圆环比较
      ○     ○
       ○ ○ ○
```

### 6.4 何时选择哪个

| 场景 | 推荐算法 |
|------|---------|
| 实时SLAM/VIO | FAST（速度优先） |
| 高精度定位 | Harris（质量优先） |
| 纹理丰富的场景 | 两者皆可 |
| 弱纹理场景 | Harris（更稳定） |
| 需要旋转不变性 | Harris + 旋转矩阵 |
| 大规模特征点提取 | FAST（可并行化） |

---

## 7. 优缺点总结

### 7.1 优点

1. **速度快**：无需计算梯度/协方差矩阵，单像素判断复杂度极低
2. **适合硬件加速**：判断逻辑简单，易于SIMD并行化
3. **旋转不变性**：基于圆环的检测天然具有旋转不变性
4. **广泛应用于SLAM**：ORB-SLAM等主流框架使用FAST作为特征点提取器
5. **可调节阈值**：通过阈值适应不同场景

### 7.2 缺点

1. **检测质量低于Harris**：在低纹理或重复纹理区域表现不佳
2. **容易检测到伪角点**：如直线交点、小角度拐角
3. **光照敏感**：阈值固定时，光照变化会影响检测数量
4. **不具有尺度不变性**：需配合图像金字塔使用
5. **高阈值下角点丢失**：阈值过高会漏掉真实角点
6. **密集分布**：不使用非极大值抑制时会产生大量邻近角点

### 7.3 实际工程建议

```
使用FAST时的最佳实践：

1. 阈值选择：根据场景亮度水平调试，通常20-40是一个不错的起点
2. 始终开启非极大值抑制：避免角点扎堆
3. 配合金字塔使用：解决尺度不变性问题
4. 后续验证：对检测到的角点计算描述子，滤除匹配质量低的点
5. 场景适配：光照变化大时考虑自适应阈值
```

---

## 8. 参考文献

1. Rosten, E., & Drummond, T. (2006). Machine learning for high-speed corner detection. *European Conference on Computer Vision (ECCV)*.
2. Rosten, E., Porter, R., & Drummond, T. (2010). Faster and better: A machine learning approach to corner detection. *IEEE Transactions on Pattern Analysis and Machine Intelligence (TPAMI)*.
3. Rublee, E., et al. (2011). ORB: An efficient alternative to SIFT or SURF. *International Conference on Computer Vision (ICCV)*.
