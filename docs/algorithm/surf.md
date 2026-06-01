# SURF (Speeded-Up Robust Features) 详解

## 1. 原理

### 1.1 积分图像 (Integral Image)

积分图像是一种加速盒式滤波的计算结构，对于图像 $I$ 在点 $(x, y)$ 处的积分图像定义为：

$$I_{\Sigma}(x, y) = \sum_{i=0}^{x} \sum_{j=0}^{y} I(i, j)$$

通过积分图像，任意矩形区域的像素和可以在 $O(1)$ 时间内计算得到，而不需要遍历区域内的每个像素。这使得 SURF 在特征点检测阶段比 SIFT 快一个数量级。

### 1.2 Hessian 矩阵

SURF 特征点检测基于 Hessian 矩阵的行列式值。给定图像 $I$ 中点 $\mathbf{x} = (x, y)$，在尺度 $\sigma$ 下的 Hessian 矩阵定义为：

$$H(\mathbf{x}, \sigma) = \begin{bmatrix} L_{xx}(x, \sigma) & L_{xy}(x, \sigma) \\ L_{xy}(x, \sigma) & L_{yy}(x, \sigma) \end{bmatrix}$$

其中 $L_{xx}$、$L_{yy}$、$L_{xy}$ 是图像与二阶高斯导数的卷积：

$$L_{xx} = \frac{\partial^2}{\partial x^2} (G(\sigma) * I), \quad L_{yy} = \frac{\partial^2}{\partial y^2} (G(\sigma) * I), \quad L_{xy} = \frac{\partial^2}{\partial xy} (G(\sigma) * I)$$

SURF 的核心加速技巧是用**近似高斯二阶导数**替代精确高斯导数。对于 $9 \times 9$ 滤波器，近似核如下：

```
```
Lxx:
[-1  0  2  0 -1]
[ 0  0  0  0  0]
[-2  0  4  0 -2]
[ 0  0  0  0  0]
[-1  0  2  0 -1]

Lyy:
[-1  0 -2  0 -1]
[ 0  0  0  0  0]
[ 2  0  4  0  2]
[ 0  0  0  0  0]
[-1  0 -2  0 -1]

Lxy:
[ 1  0 -2  0  1]
[ 0  0  0  0  0]
[-2  0  4  0 -2]
[ 0  0  0  0  0]
[ 1  0 -2  0  1]
```
```

为了保持 Hessian 行列式符号一致，在实际实现中 $L_{xy}$ 的权重会适当调整（通常为 $0.9$ 左右），具体公式为：

$$\det(H_{approx}) = D_{xx} D_{yy} - (w D_{xy})^2$$

其中 $w$ 为权重系数，用于补偿近似带来的误差。

### 1.3 尺度空间构建

SURF 的尺度空间通过**逐步放大滤波器尺寸**而非逐步降采样图像来构建。这是与 SIFT 的关键区别之一。

- SIFT：图像降采样，滤波器尺寸固定
- SURF：图像尺寸固定，逐步增大滤波器尺寸

滤波器边长与尺度的对应关系为（对于近似二阶导数核）：

$$FilterSize = 9 + 6 \times (octave - 1) + 6 \times (layer - 1)$$

SURF 通常采用 4 个八度（octave），每个八度包含 4 个尺度层。第一个八度的滤波器尺寸依次为 $9, 15, 21, 27$。

### 1.4 特征点定位

1. 在 Hessian 响应图上使用 $3 \times 3 \times 3$ 的非极大值抑制（Non-Maximum Suppression）
2. 通过插值（泰勒展开）精确定位特征点亚像素位置
3. 设置阈值过滤低对比度点

### 1.5 特征描述符

SURF 采用 **U-SURF**（Unimproved SURF）或 **SURF-64**，基于局部梯度方向的 Haar 小波响应构建描述符。

**步骤：**
1. 在特征点周围 $20\sigma$ 半径内，计算 $x$ 和 $y$ 方向的 Haar 小波响应（$dx$ 和 $dy$）
2. 以特征点为中心，划分 $4 \times 4$ 个子区域
3. 每个子区域统计 $dx$、$dy$、$|dx|$、$|dy|$ 的和，共 4 个值
4. 最终得到 $4 \times 4 \times 4 = 64$ 维特征向量

对于有方向版本，还会额外计算主方向（取最大响应的扇区角度），并在描述时考虑方向。

---

## 2. 与 SIFT 的对比（简化版）

| 特性 | SURF | SIFT |
|------|------|------|
| **检测速度** | 快（积分图像加速） | 较慢 |
| **描述符维度** | 64 维 | 128 维 |
| **尺度空间** | 滤波器尺寸放大 | 图像降采样 |
| **旋转不变性** | 有方向版本 | 有 |
| **鲁棒性** | 对模糊和光照较好 | 更好（128维更丰富） |
| **专利** | 部分免费 | 曾有专利（已过期） |
| **内存占用** | 较低 | 较高 |
| **匹配速度** | 快 | 较慢 |
| **OpenCV 函数** | `xfeatures2d::SURF::create()` | `SIFT::create()` |

**总结**：SURF 是 SIFT 的加速版，在几乎不损失太多匹配性能的前提下大幅提升了速度。但在需要极高精度的场景下，SIFT 仍是首选。

---

## 3. OpenCV 代码实现

### 3.1 基本使用

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>

int main() {
    // 读取图像
    cv::Mat img1 = cv::imread("image1.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("image2.jpg", cv::IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Failed to load images" << std::endl;
        return -1;
    }

    // 创建 SURF 检测器（hessianThreshold 是关键参数）
    double hessianThreshold = 100.0;
    int nOctaves = 4;        // 八度数
    int nOctaveLayers = 3;   // 每个八度的层数
    bool extended = false;   // false: 64维, true: 128维
    bool upright = false;    // false: 计算方向, true: 不计算方向（更快）

    cv::Ptr<cv::xfeatures2d::SURF> surf =
        cv::xfeatures2d::SURF::create(
            hessianThreshold, nOctaves, nOctaveLayers, extended, upright);

    // 检测特征点和计算描述符
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    surf->detectAndCompute(img1, cv::noArray(), kp1, des1);
    surf->detectAndCompute(img2, cv::noArray(), kp2, des2);

    std::cout << "Keypoints: " << kp1.size() << ", " << kp2.size() << std::endl;

    // 特征匹配 - 使用 FLANN 或 BFMatcher
    cv::BFMatcher matcher(cv::NORM_L2);
    std::vector<cv::DMatch> matches;
    matcher.match(des1, des2, matches);

    // 筛选好的匹配（取前 50 个）
    std::sort(matches.begin(), matches.end(),
              [](const cv::DMatch& a, const cv::DMatch& b) {
                  return a.distance < b.distance;
              });

    std::vector<cv::DMatch> goodMatches(matches.begin(),
                                         std::min(50, (int)matches.size()));

    // 绘制匹配结果
    cv::Mat imgMatches;
    cv::drawMatches(img1, kp1, img2, kp2, goodMatches, imgMatches);
    cv::imshow("SURF Matches", imgMatches);
    cv::waitKey(0);

    return 0;
}
```
```

### 3.2 完整管道示例（带异常过滤）

```cpp
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

void surfPipeline(const cv::Mat& img query, const cv::Mat& img train) {
    // 初始化
    auto surf = cv::xfeatures2d::SURF::create(400, 4, 3, true, false);
    // extended=true 会得到 128 维描述符

    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat des1, des2;

    surf->detectAndCompute(img query, cv::noArray(), kp1, des1);
    surf->detectAndCompute(img train, cv::noArray(), kp2, des2);

    // FLANN 匹配器（适合高维浮点描述符）
    cv::FlannBasedMatcher flannMatcher;
    std::vector<std::vector<cv::DMatch>> knnMatches;
    flannMatcher.knnMatch(des1, des2, knnMatches, 2);

    // Lowe's ratio test 过滤
    const float ratio_thresh = 0.75f;
    std::vector<cv::DMatch> goodMatches;
    for (const auto& knnMatch : knnMatches) {
        if (knnMatch.size() >= 2 &&
            knnMatch[0].distance < ratio_thresh * knnMatch[1].distance) {
            goodMatches.push_back(knnMatch[0]);
        }
    }

    // 计算单应性矩阵（如果需要）
    if (goodMatches.size() > 4) {
        std::vector<cv::Point2f> src_pts, dst_pts;
        for (const auto& m : goodMatches) {
            src_pts.push_back(kp1[m.queryIdx].pt);
            dst_pts.push_back(kp2[m.trainIdx].pt);
        }
        cv::Mat homography = cv::findHomography(src_pts, dst_pts, cv::RANSAC);
    }
}
```
```

### 3.3 CMakeLists.txt 依赖

```cmake
```cmake
find_package(OpenCV REQUIRED)
# OpenCV 需要包含 xfeatures2d 模块（opencv_contrib）

add_executable(surf_demo main.cpp)
target_link_libraries(surf_demo ${OpenCV_LIBS})
```
```

---

## 4. 关键参数解释

### 4.1 hessianThreshold

- **含义**：Hessian 矩阵行列式的阈值
- **作用**：只有响应值大于该阈值的点才会被检测为特征点
- **取值范围**：通常 100 - 1500
- **影响**：
  - 值越小：检测到的特征点越多，召回率高，但噪声也会增加
  - 值越大：特征点越少，更稳定，但可能遗漏重要特征
- **调整建议**：
  - 纹理丰富的场景：可用较大值（500-1000）
  - 纹理稀少或关键物体较小的场景：减小到 100-300

### 4.2 nOctaves

- **含义**：八度（octave）的数量
- **作用**：每个八度代表一个尺度空间的数量级，多的八度能检测到更大尺度的特征
- **取值**：通常 4（默认），最大可设 5
- **影响**：
  - 八度越多，能检测的特征尺度范围越广（大物体）
  - 计算量也相应增加

### 4.3 nOctaveLayers

- **含义**：每个八度内的层数
- **作用**：决定在每个八度内有多少个中间尺度
- **取值**：通常 3 或 4（默认 3）
- **影响**：层数越多，尺度空间采样越精细，但计算量增加

### 4.4 extended

- **含义**：描述符是否为扩展版本
- **取值**：`false` = 64 维，`true` = 128 维
- **影响**：
  - 64 维：速度更快，内存更少，足够大多数场景
  - 128 维：描述更丰富，匹配精度可能更高，但匹配速度下降

### 4.5 upright

- **含义**：是否计算特征点方向
- **取值**：`false` = 计算方向（有旋转不变性），`true` = 不计算（更快）
- **影响**：
  - `upright=true` 速度约快 20-30%
  - 只适用于相机只有平移运动或接近水平的场景

---

## 5. 优缺点

### 5.1 优点

1. **速度快**：得益于积分图像，特征检测速度比 SIFT 快数倍到数十倍
2. **内存效率高**：64 维描述符比 SIFT 的 128 维更紧凑
3. **旋转不变性**：通过主方向计算实现（`upright=false` 时）
4. **尺度不变性**：多八度尺度空间确保了对尺度变化的鲁棒性
5. **光照和模糊鲁棒**：基于 Hessian 矩阵的检测方式对光照变化和图像模糊有较好容忍度
6. **与 OpenCV 集成良好**：可直接用 `xfeatures2d::SURF::create()` 调用

### 5.2 缺点

1. **专利限制**（历史）：早期 SURF 有专利，OpenCV 中放在 `xfeatures2d` 扩展模块而非主模块
2. **精度略低于 SIFT**：在需要高精度匹配的场景下，SIFT 的 128 维描述符提供更丰富的梯度信息
3. **非官方维护**：OpenCV 4.x 起 SURF 从主模块移到了 `opencv_contrib` 的 `xfeatures2d`，后续可能缺乏维护
4. **不支持 GPU 加速的官方接口**：OpenCV 中没有官方的 `cuda::SURF` 实现
5. **对强烈透视变形敏感**：SURF 最初设计假设的是近平面和轻微透视，在强烈透视场景下表现下降

### 5.3 替代方案

| 场景 | 推荐算法 |
|------|----------|
| 追求速度，精度要求不高 | ORB（最快，免费） |
| 需要高精度 | SIFT 或 AKAZE |
| 模糊图像 | KAZE 或 AKAZE |
| 实时 SLAM/VIO | ORB（VINS-Mono）、AKAZE |
| 通用场景，综合表现 | BRISK（介于 ORB 和 SURF 之间） |

---

## 参考资料

- Bay, H., Ess, A., Tuytelaars, T., & Van Gool, L. (2008). "SURF: Speeded-Up Robust Features". Computer Vision and Image Understanding, 110(3), 346-359.
- OpenCV Documentation: `cv::xfeatures2d::SURF`
