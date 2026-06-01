# AKAZE 特征检测算法详解

## 1. 原理

### 1.1 尺度空间与高斯模糊的局限

传统特征检测算法（SIFT、SURF）采用高斯金字塔构建尺度空间，通过高斯模糊消除噪声、提取多尺度特征。然而高斯模糊是一种线性、各向同性的扩散过程——它对图像中的边缘和平坦区域一视同仁地平滑，导致边缘被模糊化，定位精度下降。此外，高斯模糊没有考虑图像本身的结构特征，在纹理重复或边缘复杂的场景中表现欠佳。

### 1.2 KAZE：非线性扩散尺度空间

KAZE（ECCV 2012）提出的核心思想是用**非线性扩散方程**替代高斯模糊，构建保边（edge-preserving）的尺度空间。其理论基础是 **Perona-Malik 非线性扩散方程**：

```
∂L/∂t = div(c(x, y, t) · ∇L)
```

其中 `c(x, y, t)` 是扩散系数（conductivity），定义为：

```
c(x, y, t) = g(|∇Lσ(x, y, t)|)
```

扩散系数的常见形式：
- `g1 = exp(-|∇L|² / λ²)` — 边缘处梯度大，扩散小，边缘保留
- `g2 = 1 / (1 + |∇L|² / λ²)` — 同理，梯度大的地方少扩散

**本质**：梯度大的地方（边缘）少扩散，梯度小的地方（平坦区）多扩散。这使得尺度空间的构建第一次做到了"该模糊的地方模糊，该保留的边缘保留"，在纹理重复场景下精度显著高于 SIFT。

KAZE 原版使用 **AOS（Additive Operator Splitting，加性算子分裂）** 求解非线性扩散方程，可以采用任意步长，稳定性好，但每一步迭代开销较大，构造尺度空间耗时较长。

### 1.3 AKAZE：FED 显式扩散加速

AKAZE（ACCV 2013）在 KAZE 基础上做了两个关键改进：

**改进一：用 FED 替代 AOS 构建尺度空间**

FED（Fast Explicit Diffusion，快速显式扩散）不是一种新的偏微分方程求解理论，而是对**显式欧拉法**的加速策略。其核心思想：通过一组不等长的时间步长构成一个"超级步"（cycle），每一步的稳定性条件可以放宽，整体收敛速度比标准显式法快一个数量级，同时保持与 AOS 同等的精度。

具体而言，FED 通过预计算一组优化的时间步长 `τk`：

```
τk = (3N² + 1) / [k²(N² - k)] · T_cycle / (4N(N+1))
```

其中 `N` 为每轮循环的子步数，`T_cycle` 为总演化时间。将多个 FED cycle 串联起来逐步演化图像，比逐点迭代的显式欧拉法快很多。AKAZE 还结合了**图像金字塔**（每层独立做 FED 非线性扩散），进一步提升效率。

**改进二：引入 M-LDB 二值描述子**

KAZE 使用 MSURF（改良的 SURF）描述子，计算局部梯度，精度高但速度慢。AKAZE 改用 **M-LDB（Modified Local Difference Binary）** 描述子——一种改进的局部差分二值描述子，兼具旋转和尺度不变性，计算和匹配速度极快（Hamming 距离即可完成匹配）。

### 1.4 AKAZE 算法流程

```
输入图像
  │
  ▼
[构建非线性尺度空间] ── FED + 图像金字塔
  │  每层：非线性扩散滤波 → 得到不同演化时间的图像
  │
  ▼
[关键点检测] ── Hessian 矩阵行列式的局部最大值
  │  使用 Scharr 滤波器提高旋转不变性
  │
  ▼
[主方向计算] ── 一阶微分图像的梯度统计
  │
  ▼
[描述子生成] ── M-LDB 二值描述子
  │  利用非线性尺度空间的梯度信息，增强独特性
  │
  ▼
输出：关键点 + M-LDB 描述子
```

**Hessian 矩阵定义**（尺度 σ 下的归一化版本）：

```
H = | Lxx  Lxy |
    | Lxy  Lyy |
```

检测器响应为 `det(H) = Lxx · Lyy - Lxy²`，在空间和尺度维度取局部极大值即为特征点。

---

## 2. 与 ORB 的对比

### 2.1 算法架构对比

| 维度 | AKAZE | ORB |
|------|-------|-----|
| 检测器 | 非线性扩散尺度空间 + Hessian 行列式极大值 | FAST 角点检测（图像金字塔多尺度） |
| 尺度空间 | 非线性扩散（保边） | 高斯金字塔（线性，各向同性） |
| 主方向 | 梯度统计 | oFAST（灰度质心法） |
| 描述子 | M-LDB（二值，改进的 LDB） | Rotated BRIEF（二值） |
| 求解器 | FED 显式扩散 | 无（直接图像缩放） |
| 发表 | ACCV 2013 | CVPR 2012 |

### 2.2 核心差异分析

**1. 尺度空间构建方式**
- ORB 的多尺度靠图像金字塔实现，本质上是简单的下采样 + 高斯模糊，没有考虑图像内容。
- AKAZE 的尺度空间通过非线性扩散演化得到，每层图像都是前一层经扩散滤波得到的结果，尺度参数连续可控，边缘感知能力强。

**2. 描述子鲁棒性**
- M-LDB 利用非线性扩散空间的梯度信息，对重复纹理场景的区分能力更强。
- BRIEF 描述子对旋转变换敏感，ORB 通过增加方向补偿（rotated BRIEF）缓解了部分问题，但尺度不变性仍依赖金字塔。

**3. 匹配性能**
- 在纹理重复场景（如砖墙、建筑立面）中，AKAZE 的匹配正确率（inlier ratio）显著高于 ORB。
- 在通用场景中，ORB 速度更快，但精度略低。

**4. 时间复杂度**
- AKAZE：非线性扩散计算成本较高，单帧处理约比 ORB 慢 3-5 倍（取决于参数）。
- ORB：纯二值操作（FAST + Hamming 距离），在 CPU 上非常快，适合实时性要求高的场景。

**5. 专利与授权**
- ORB 使用 FAST 和 BRIEF，均为开源算法，无专利限制，商业可用。
- AKAZE 同样无专利限制。

---

## 3. OpenCV 代码实现

### 3.1 基本用法

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

// OpenCV 3/4 中 AKAZE 在 xfeatures2d 模块
cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

// 读取图像并转灰度
cv::Mat img1 = cv::imread("image1.jpg");
cv::Mat img2 = cv::imread("image2.jpg");
cv::Mat gray1, gray2;
cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
cv::cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

// 检测关键点并计算描述子
std::vector<cv::KeyPoint> kp1, kp2;
cv::Mat des1, des2;
akaze->detectAndCompute(gray1, cv::noArray(), kp1, des1);
akaze->detectAndCompute(gray2, cv::noArray(), kp2, des2);

// 使用 BFMatcher 进行匹配（Hamming 距离，用于二值描述子）
cv::BFMatcher matcher(cv::NORM_HAMMING);
std::vector<cv::DMatch> matches;
matcher.match(des1, des2, matches);

// 筛选好的匹配（可选）
std::sort(matches.begin(), matches.end(),
          [](const cv::DMatch& a, const cv::DMatch& b) {
              return a.distance < b.distance;
          });
std::vector<cv::DMatch> good_matches(matches.begin(),
                                      matches.begin() + std::min(100, (int)matches.size()));

// 绘制匹配结果
cv::Mat img_matches;
cv::drawMatches(img1, kp1, img2, kp2, good_matches, img_matches);
cv::imshow("AKAZE Matches", img_matches);
cv::waitKey(0);
```

### 3.2 带 RANSAC 的单应性估计

```cpp
#include <opencv2/calib3d.hpp>

// 使用 KNNMatch + ratio test 筛选匹配
std::vector<std::vector<cv::DMatch>> knn_matches;
matcher.knnMatch(des1, des2, knn_matches, 2);

float nn_match_ratio = 0.8f;
std::vector<cv::DMatch> good_matches;
for (auto& knn : knn_matches) {
    if (knn[0].distance < nn_match_ratio * knn[1].distance) {
        good_matches.push_back(knn[0]);
    }
}

// 提取匹配点坐标
std::vector<cv::Point2f> pts1, pts2;
for (auto& m : good_matches) {
    pts1.push_back(kp1[m.queryIdx].pt);
    pts2.push_back(kp2[m.trainIdx].pt);
}

// RANSAC 估计单应性矩阵
std::vector<uchar> inlier_mask;
cv::Mat H = cv::findHomography(pts1, pts2, cv::RANSAC, 2.5f, inlier_mask);

// 绘制内点匹配
std::vector<cv::DMatch> inliers;
for (size_t i = 0; i < good_matches.size(); ++i) {
    if (inlier_mask[i]) {
        inliers.push_back(good_matches[i]);
    }
}

cv::Mat img_inliers;
cv::drawMatches(img1, kp1, img2, kp2, inliers, img_inliers,
                cv::Scalar(0, 255, 0), cv::Scalar::all(-1),
                std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

float inlier_ratio = static_cast<float>(inliers.size()) / good_matches.size();
std::cout << "Inlier ratio: " << inlier_ratio << std::endl;
```

### 3.3 自定义参数创建

```cpp
cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(
    cv::AKAZE::DESCRIPTOR_MLDB,  // 描述子类型：MLDB（默认）
    // 还有 DESCRIPTOR_KAZE（KAZE 风格的浮点描述子，但 AKAZE 类中不常用）
    0,                            // 描述子字节数：0 表示使用完整 486 字节
    3,                            //  octave 层数（默认 4，金字塔总层数）
    0.001f,                       //  阈值： Hessian 响应的最小值（越小特征点越多）
    4,                            //  仿射不变性参数（默认 4）
    cv::KAZE::DIFF_PM_G2         //  扩散方程类型：PM_G2（默认）或 PM_G1
);
```

---

## 4. 参数解释

### 4.1 AKAZE::create() 完整参数列表

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `descriptor_type` | `int` | `DESCRIPTOR_MLDB` | 描述子类型。`DESCRIPTOR_MLDB`（常用，二值）或 `DESCRIPTOR_KAZE`（浮点，KAZE 风格） |
| `descriptor_size` | `int` | `0` | 描述子字节数。`0` = 完整描述子（MLDB 为 486 字节）；可设为 32/64/128 等进行压缩 |
| `descriptor_channels` | `int` | `3` | 描述子使用的通道数（仅 MLDB）。`1/2/3`，影响描述子维度 |
| `threshold` | `float` | `0.001f` | Hessian 行列式检测阈值。越小检测到的特征点越多，默认约 1000 个/图 |
| `octaves` | `int` | `4` | 金字塔组数（octave groups），即 4 表示 2⁴ = 16 个尺度级别 |
| `sublevels` | `int` | `4` | 每组内的子层数（较少用到，默认 4） |
| `diffusivity` | `int` | `KAZE::DIFF_PM_G2` | 非线性扩散方程类型：`DIFF_PM_G1`（`g1 = exp(-|∇L|²/λ²)`）或 `DIFF_PM_G2`（`g2 = 1/(1+|∇L|²/λ²)`），G2 更常用 |

### 4.2 各参数对算法的影响

**`threshold`（阈值）**
- 控制检测灵敏度：值越小特征点越多，反之越少
- 场景适应：纹理丰富的图像可适当增大阈值以避免过多特征点；纹理简单的图像降低阈值以保证足够特征点

**`octaves`（金字塔层数）**
- 控制尺度范围：层数越多，能检测的尺度范围越大
- 深度估计/多视图：建议 4-6；实时跟踪：可降到 2-3

**`descriptor_size`（描述子大小）**
- 设为 `0` 使用完整描述子，匹配精度最高
- 压缩（32/64 字节）可加速匹配，但精度下降
- 嵌入式/实时场景：64-128 字节是常见折中

**`diffusivity`（扩散类型）**
- `DIFF_PM_G2`：默认，对大多数场景效果良好，边缘保留能力强
- `DIFF_PM_G1`：边缘保留更强，但可能产生 staircase 效应（边缘被阶梯化）

### 4.3 常用配置建议

| 应用场景 | threshold | octaves | descriptor_size |
|----------|-----------|---------|-----------------|
| 通用图像匹配（精度优先） | 0.0001 ~ 0.001 | 4 | 0（完整） |
| 实时视频跟踪（速度优先） | 0.005 ~ 0.01 | 2 | 64 |
| 宽基线多视图 | 0.0003 | 6 | 0 |
| 嵌入式/移动端 | 0.003 | 3 | 64 |

---

## 5. 优缺点

### 5.1 优点

**1. 保边尺度的非线性扩散**
非线性扩散使得 AKAZE 在构建尺度空间时能够保留图像边缘，这对特征点的定位精度和独特性至关重要。相比高斯模糊，非线性扩散在平坦区域有效降噪，在边缘处保持清晰，显著提升了特征点的可重复性（repeatability）。

**2. 优秀的匹配性能**
在纹理重复场景（砖墙、地板、建筑物立面）下，AKAZE 的匹配正确率（inlier ratio）明显优于 ORB 和 SIFT。原因是非线性扩散保留了边缘，使得特征点在重复纹理中更具区分度。

**3. 旋转和尺度不变性**
通过 FED 构建连续非线性尺度空间（而非离散金字塔），AKAZE 的尺度采样更加精细。结合 Scharr 滤波和主方向计算，对旋转和尺度变化具有较强的鲁棒性。

**4. 二值描述子，匹配速度快**
M-LDB 是二值描述子，使用 Hamming 距离进行匹配。在现代 CPU（支持 POPCNT 指令）和 GPU 上，Hamming 距离计算极快，适合大规模特征匹配。

**5. 无专利限制**
SIFT 在很长时间内受专利保护（已于 2020 年到期），SURF 也有部分专利。AKAZE 和 ORB 一样，可以自由用于商业项目。

### 5.2 缺点

**1. 计算速度慢于 ORB**
非线性扩散（FED 求解）的计算开销远大于 FAST 角点检测。即使有金字塔加速，AKAZE 的检测和描述速度仍约为 ORB 的 3-5 倍。在极端实时场景（如高速 SLAM）下，ORB 往往是首选。

**2. 描述子维度较高**
MLDB 完整描述子为 486 字节（远大于 ORB 的 32 字节），存储和匹配开销更大。虽然可以通过 `descriptor_size` 参数压缩，但这会牺牲精度。

**3. 对噪声敏感**
非线性扩散在低对比度或高噪声图像中可能产生不稳定的尺度空间。若图像质量较差，建议先做去噪预处理。

**4. 旋转不变性弱于 SIFT**
虽然 AKAZE 通过主方向计算实现旋转补偿，但其旋转不变性仍不如 SIFT/SURF 的梯度直方图方法。在大旋转角场景下可能出现描述子失效。

**5. OpenCV 支持有限**
OpenCV 中 AKAZE 的实现不如 ORB 完善（ORB 在主模块 `opencv_objdetect`/`opencv_features2d` 中，AKAZE 在扩展模块 `xfeatures2d` 中）。此外，部分 AKAZE 参数（如 FED 时间步长）在 OpenCV 接口中不可调。

### 5.3 适用场景总结

| 推荐使用 AKAZE | 推荐使用 ORB |
|---------------|-------------|
| 纹理重复场景（建筑、墙面、纹理图案） | 通用实时跟踪（机器人、AR） |
| 对匹配精度要求高，可容忍稍高延迟 | 对速度要求极高，精度要求一般 |
| 多视图重建、SLAM 后端 | 移动端/嵌入式视觉 |
| 需要保边特征的场景 | 计算资源受限的实时系统 |
