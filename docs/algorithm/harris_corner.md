# Harris角点检测器详解

## 1. 原理（数学推导，特征值分析）

### 1.1 基本思想

Harris角点检测器的核心思想来源于一个直观的观察：**角点是图像中在所有方向上灰度变化都很剧烈的点**。与之相对：
- **平坦区域**：所有方向上灰度变化都很小
- **边缘**：仅在垂直于边缘的方向上灰度变化剧烈

### 1.2 数学推导

#### 步骤1：灰度变化度量

对于图像中的任意一点$(x,y)$，考虑其一个邻域窗口$W$在偏移$(\Delta x, \Delta y)$后的灰度变化：

$$E(\Delta x, \Delta y) = \sum_{u,v \in W} w(u,v) [I(x+u+\Delta x, y+v+\Delta y) - I(x+u, y+v)]^2$$

其中：
- $w(u,v)$是窗口权重函数（通常为高斯函数）
- $I(x,y)$是图像灰度值

#### 步骤2：泰勒展开近似

对灰度进行一阶泰勒展开：

$$I(x+u+\Delta x, y+v+\Delta y) \approx I(x+u, y+v) + I_x(x+u, y+v)\Delta x + I_y(x+u, y+v)\Delta y$$

其中$I_x$和$I_y$是图像在$x$和$y$方向的偏导数（梯度）。

代入得：

$$E(\Delta x, \Delta y) \approx \sum_{u,v} w(u,v) [I_x(x+u, y+v)\Delta x + I_y(x+u, y+v)\Delta y]^2$$

#### 步骤3：矩阵形式

将上式写成矩阵形式：

$$E(\Delta x, \Delta y) \approx \begin{bmatrix} \Delta x & \Delta y \end{bmatrix} M \begin{bmatrix} \Delta x \\ \Delta y \end{bmatrix}$$

其中$M$是$2 \times 2$的对称矩阵（也称为**结构张量**或**第二矩矩阵**）：

$$M = \sum_{u,v} w(u,v) \begin{bmatrix} I_x^2 & I_x I_y \\ I_x I_y & I_y^2 \end{bmatrix} = \begin{bmatrix} A & C \\ C & B \end{bmatrix}$$

其中：
- $A = \sum w(u,v) I_x^2$（梯度$x$分量的加权和）
- $B = \sum w(u,v) I_y^2$（梯度$y$分量的加权和）
- $C = \sum w(u,v) I_x I_y$（梯度交叉项）

#### 步骤4：特征值分析

对矩阵$M$进行特征值分解：

$$M = R^{-1} \begin{bmatrix} \lambda_1 & 0 \\ 0 & \lambda_2 \end{bmatrix} R$$

两个特征值$\lambda_1$和$\lambda_2$具有深刻的几何意义：

| 特征值关系 | 图像特征 |
|-----------|---------|
| $\lambda_1 \approx \lambda_2 \approx 0$ | 平坦区域 |
| $\lambda_1 \gg \lambda_2$ 或 $\lambda_2 \gg \lambda_1$ | 边缘 |
| $\lambda_1 > 0$，$\lambda_2 > 0$ 且两者都较大 | 角点 |

#### 步骤5：角点响应函数

直接使用特征值需要计算，成本较高。Harris提出了一个无需显式计算特征值的角点响应函数：

$$R = \det(M) - k \cdot \text{trace}(M)^2 = (AB - C^2) - k(A + B)^2$$

其中$k$是经验常数（通常取$0.04 \sim 0.06$）。

**响应值$R$的物理意义**：
- $R$很小（$\lambda_1$和$\lambda_2$都很小）：平坦区域
- $R < 0$（一个特征值远大于另一个）：边缘
- $R$很大（两个特征值都很大）：角点

### 1.3 几何解释

从椭圆几何的角度看，矩阵$M$定义了每个点的一个椭圆：

$$[\Delta x, \Delta y] M [\Delta x, \Delta y]^T = 1$$

椭圆的长短轴分别与$\lambda_1$和$\lambda_2$成反比：
- 椭圆扁平时，表示接近边缘
- 椭圆接近圆时，表示接近角点或各向同性区域

---

## 2. 算法流程

### 2.1 Harris角点检测完整流程

```
输入：灰度图像 I，参数 k，阈值 T，邻域大小
输出：角点坐标列表

1. 预处理
   ├── 将图像转换为灰度（如果不是）
   └── 可选：应用高斯模糊降噪

2. 计算梯度
   ├── 使用Sobel算子计算 Ix = dI/dx
   └── 使用Sobel算子计算 Iy = dI/dy

3. 计算梯度乘积
   ├── 计算 A = Ix²（加权）
   ├── 计算 B = Iy²（加权）
   └── 计算 C = Ix * Iy（加权）

4. 对每个像素计算响应值
   ├── 对A、B、C分别应用高斯平滑
   └── R = AB - C² - k(A+B)²

5. 非极大值抑制
   ├── 在3×3邻域内保留局部最大值
   └── 删除低于阈值的响应

6. 输出角点坐标
```

### 2.2 关键参数选择

| 参数 | 典型值 | 说明 |
|-----|-------|------|
| $k$ | 0.04 ~ 0.06 | 经验常数，控制对角点的敏感性 |
| 窗口大小 | 3 × 3 ~ 7 × 7 | 影响检测的稳定性和精度 |
| 高斯$\sigma$ | 1.0 ~ 2.0 | 梯度平滑的标准差 |
| 阈值$T$ | 0.01 ~ 0.1 × max(R) | 响应值阈值 |

---

## 3. OpenCV代码实现

### 3.1 使用OpenCV内置函数

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 读取图像并转换为灰度
    cv::Mat image = cv::imread("image.jpg");
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // 计算Harris角点
    // cornerHarris(src, dst, blockSize, ksize, k)
    // - blockSize: 邻域大小
    // - ksize: Sobel算子的孔径大小
    // - k: Harris响应函数中的常数
    cv::Mat dst;
    int blockSize = 2;      // 邻域大小
    int ksize = 3;           // Sobel核大小
    double k = 0.04;         // Harris参数

    cv::cornerHarris(gray, dst, blockSize, ksize, k);

    // 归一化以便显示
    cv::Mat dst_norm, dst_norm_scaled;
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // 标记角点
    double threshold = 0.01 * 255;  // 阈值为最大值的1%
    std::vector<cv::Point2f> corners;

    for (int i = 0; i < dst_norm.rows; i++) {
        for (int j = 0; j < dst_norm.cols; j++) {
            if ((int)dst_norm.at<float>(i, j) > threshold) {
                corners.push_back(cv::Point2f(j, i));
                cv::circle(dst_norm_scaled, cv::Point(j, i), 5,
                          cv::Scalar(0), 2);
            }
        }
    }

    // 显示结果
    cv::imshow("Harris Corners", dst_norm_scaled);
    cv::waitKey(0);

    std::cout << "检测到 " << corners.size() << " 个角点" << std::endl;

    return 0;
}
```

### 3.2 使用goodFeaturesToTrack（Shi-Tomasi变种）

```cpp
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::Mat image = cv::imread("image.jpg");
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Shi-Tomasi角点检测
    // goodFeaturesToTrack(image, corners, maxCorners, qualityLevel, minDistance)
    std::vector<cv::Point2f> corners;
    int maxCorners = 100;           // 最大角点数量
    double qualityLevel = 0.01;     // 质量等级（最小特征值的分数）
    double minDistance = 10;        // 角点之间的最小距离

    cv::goodFeaturesToTrack(gray, corners, maxCorners,
                            qualityLevel, minDistance);

    // 绘制角点
    for (size_t i = 0; i < corners.size(); i++) {
        cv::circle(image, corners[i], 5, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("Shi-Tomasi Corners", image);
    cv::waitKey(0);

    return 0;
}
```

---

## 4. 自己实现的代码

### 4.1 完整C++实现

```cpp
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

/**
 * 自定义Harris角点检测器
 */
class HarrisCornerDetector {
public:
    struct Parameters {
        int blockSize = 2;      // 邻域大小
        int ksize = 3;         // Sobel核大小
        double k = 0.04;        // Harris常数
        double threshold = 0.01; // 阈值（相对于最大值）
        bool useNMS = true;     // 是否使用非极大值抑制
    };

    HarrisCornerDetector(const Parameters& params = Parameters())
        : params_(params) {}

    /**
     * 检测角点
     * @param gray 输入灰度图像
     * @return 角点坐标列表
     */
    std::vector<cv::Point2f> detect(const cv::Mat& gray) {
        cv::Mat Ix, Iy;
        computeGradients(gray, Ix, Iy);

        // 计算梯度的乘积
        cv::Mat Ix2, Iy2, Ixy;
        cv::multiply(Ix, Ix, Ix2);
        cv::multiply(Iy, Iy, Iy2);
        cv::multiply(Ix, Iy, Ixy);

        // 高斯平滑
        cv::Mat A, B, C;
        cv::GaussianBlur(Ix2, A, cv::Size(3, 3), 1.0);
        cv::GaussianBlur(Iy2, B, cv::Size(3, 3), 1.0);
        cv::GaussianBlur(Ixy, C, cv::Size(3, 3), 1.0);

        // 计算响应值R
        cv::Mat R(gray.size(), CV_64F);
        for (int y = 0; y < gray.rows; y++) {
            for (int x = 0; x < gray.cols; x++) {
                double a = A.at<double>(y, x);
                double b = B.at<double>(y, x);
                double c = C.at<double>(y, x);

                double det = a * b - c * c;
                double trace = a + b;
                R.at<double>(y, x) = det - params_.k * trace * trace;
            }
        }

        // 归一化并找角点
        double maxVal;
        cv::minMaxLoc(R, nullptr, &maxVal);

        std::vector<cv::Point2f> corners;

        for (int y = params_.blockSize; y < gray.rows - params_.blockSize; y++) {
            for (int x = params_.blockSize; x < gray.cols - params_.blockSize; x++) {
                double val = R.at<double>(y, x);

                if (val > params_.threshold * maxVal) {
                    if (params_.useNMS) {
                        // 非极大值抑制
                        if (isLocalMaximum(R, x, y)) {
                            corners.emplace_back(x, y);
                        }
                    } else {
                        corners.emplace_back(x, y);
                    }
                }
            }
        }

        return corners;
    }

private:
    Parameters params_;

    void computeGradients(const cv::Mat& gray, cv::Mat& Ix, cv::Mat& Iy) {
        // 使用Sobel算子计算梯度
        cv::Sobel(gray, Ix, CV_64F, 1, 0, params_.ksize);
        cv::Sobel(gray, Iy, CV_64F, 0, 1, params_.ksize);
    }

    bool isLocalMaximum(const cv::Mat& R, int x, int y) {
        double centerVal = R.at<double>(y, x);

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = x + dx;
                int ny = y + dy;

                if (nx >= 0 && nx < R.cols && ny >= 0 && ny < R.rows) {
                    if (R.at<double>(ny, nx) > centerVal) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
};

// 使用示例
int main() {
    cv::Mat image = cv::imread("image.jpg");
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    gray.convertTo(gray, CV_64F);

    HarrisCornerDetector::Parameters params;
    params.blockSize = 2;
    params.k = 0.04;
    params.threshold = 0.01;

    HarrisCornerDetector detector(params);
    std::vector<cv::Point2f> corners = detector.detect(gray);

    // 绘制角点
    for (const auto& pt : corners) {
        cv::circle(image, pt, 3, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("Custom Harris Corners", image);
    cv::waitKey(0);

    return 0;
}
```

### 4.2 Python实现（NumPy版本）

```python
import numpy as np
import cv2

def harris_corner_detector(image, block_size=2, k=0.04, threshold=0.01):
    """
    Harris角点检测器

    参数:
        image: 输入灰度图像
        block_size: 邻域大小
        k: Harris常数
        threshold: 阈值（相对于最大值的比例）

    返回:
        corners: 角点坐标列表
    """
    # 计算梯度
    Ix = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
    Iy = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)

    # 计算梯度的乘积
    Ix2 = Ix * Ix
    Iy2 = Iy * Iy
    Ixy = Ix * Iy

    # 高斯平滑
    Ix2 = cv2.GaussianBlur(Ix2, (3, 3), 1.0)
    Iy2 = cv2.GaussianBlur(Iy2, (3, 3), 1.0)
    Ixy = cv2.GaussianBlur(Ixy, (3, 3), 1.0)

    # 计算响应值R
    A, B, C = Ix2, Iy2, Ixy
    R = A * B - C**2 - k * (A + B)**2

    # 归一化
    R = R / R.max() * 255

    # 非极大值抑制并阈值化
    corners = []
    for i in range(block_size, image.shape[0] - block_size):
        for j in range(block_size, image.shape[1] - block_size):
            if R[i, j] > threshold * 255:
                # 检查是否是局部最大值
                if R[i, j] == R[i-1:i+2, j-1:j+2].max():
                    corners.append((j, i))

    return corners

# 使用示例
if __name__ == "__main__":
    image = cv2.imread("image.jpg")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = np.float64(gray)

    corners = harris_corner_detector(gray)

    # 绘制角点
    for x, y in corners:
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

    cv2.imshow("Harris Corners", image)
    cv2.waitKey(0)
```

---

## 5. 参数解释

### 5.1 Harris算法参数

| 参数 | 说明 | 影响 | 典型值 |
|-----|------|------|-------|
| **blockSize** | 计算M矩阵的邻域大小 | 影响检测的稳定性；太大会丢失细节，太小会检测到噪声 | 2 ~ 5 |
| **ksize** | Sobel算子的核大小 | 影响梯度计算的精度；必须为奇数 | 3 |
| **k** | Harris响应函数的权重系数 | 控制角点检测的敏感性；增大k会使检测更严格（更少的角点） | 0.04 ~ 0.06 |
| **threshold** | 响应值阈值 | 决定哪些响应被认为是角点 | max(R) × 0.01 ~ 0.1 |

### 5.2 参数对检测结果的影响

```
k值影响:
┌─────────────────────────────────────────────────┐
│ k太小 (≈0.02)                                    │
│   → 响应值增大                                   │
│   → 可能检测到更多"假角点"                       │
│   → 边缘也可能被误判为角点                        │
├─────────────────────────────────────────────────┤
│ k太大 (≈0.10)                                    │
│   → 响应值减小                                   │
│   → 只有最显著的角点被检测到                      │
│   → 可能遗漏许多有效角点                          │
└─────────────────────────────────────────────────┘

阈值影响:
┌─────────────────────────────────────────────────┐
│ 阈值太低                                        │
│   → 检测到过多角点                               │
│   → 包含噪声点                                   │
├─────────────────────────────────────────────────┤
│ 阈值太高                                        │
│   → 仅检测到最显著的角点                         │
│   → 可能遗漏有效角点                             │
└─────────────────────────────────────────────────┘
```

### 5.3 窗口大小的影响

| 窗口大小 | 优点 | 缺点 |
|---------|------|------|
| 小 (3×3) | 精度高，能检测到细角点 | 对噪声敏感，角点位置可能不精确 |
| 中 (5×5) | 平衡稳定性和精度 | 可能平滑掉一些细节 |
| 大 (7×7+) | 抗噪声能力强 | 可能合并相邻角点，降低定位精度 |

---

## 6. 优缺点

### 6.1 优点

1. **旋转不变性**：Harris检测基于梯度分布，不依赖图像的旋转角度

2. **对图像强度变化部分不变**：由于使用梯度的乘积，对均匀的亮度变化（平移）不敏感

3. **计算效率较高**：相比基于模板匹配的角点检测，计算量较小

4. **角点响应函数设计合理**：通过特征值组合，既能检测角点又能区分边缘

5. **易于理解和实现**：数学原理清晰，代码实现相对简单

### 6.2 缺点

1. **对尺度敏感**：不同尺度的图像可能检测出不同数量和位置的角点
   - 解决：使用尺度不变特征变换（SIFT）

2. **阈值依赖性强**：需要人工设置阈值，不适合自动化场景

3. **非极大值抑制是局部的**：可能产生局部响应相近的多个响应点

4. **检测的角点位置精度有限**：亚像素级精度需要额外插值计算

5. **对模糊图像效果下降**：模糊会平滑梯度，降低响应值

6. **边缘响应问题**：在边缘处可能产生伪角点（非角点但响应值高）

### 6.3 与其他角点检测方法的对比

| 方法 | 优点 | 缺点 | 适用场景 |
|-----|------|------|---------|
| Harris | 旋转不变，计算快 | 尺度敏感 | 短基线匹配 |
| Shi-Tomasi | 最优特征值跟踪 | 同样尺度敏感 | 特征跟踪 |
| SIFT | 尺度不变，描述性强 | 计算量大 | 目标识别，大尺度匹配 |
| FAST | 极快 | 无尺度/旋转不变 | 实时性要求高的场景 |

---

## 7. Shi-Tomasi变种

### 7.1 原理

Shi-Tomasi（1994）在Harris的基础上进行了改进。观察到Harris响应函数：

$$R = \det(M) - k \cdot \text{trace}(M)^2 = \lambda_1 \lambda_2 - k(\lambda_1 + \lambda_2)^2$$

在某些情况下可能不是最优的。Shi-Tomasi直接使用最小特征值作为响应：

$$R = \min(\lambda_1, \lambda_2)$$

当$\min(\lambda_1, \lambda_2) > \lambda_{threshold}$时，判定为角点。

### 7.2 几何解释

```
Harris: R = λ₁λ₂ - k(λ₁ + λ₂)²
        └── 抛物线轮廓，在λ₁=λ₂时达到最大值

Shi-Tomasi: R = min(λ₁, λ₂)
            └── 只需两个特征值都大于阈值即可

                    λ₂
                    ↑
                    │     *
                    │   * *
                    │  * * *
        ───────────┼──*──────→ λ₁
                    │  * * *
                    │   * *
                    │     *
```

### 7.3 Shi-Tomasi的优势

1. **更好的稳定性**：直接基于特征值，响应函数更直观
2. **更适合跟踪**：在特征点跟踪场景中表现更好
3. **参数更直观**：只需设置最小特征值阈值

### 7.4 代码实现

```cpp
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * Shi-Tomasi角点检测器实现
 */
std::vector<cv::Point2f> shiTomasiDetector(
    const cv::Mat& gray,
    int maxCorners = 100,
    double qualityLevel = 0.01,
    double minDistance = 10)
{
    // 计算Harris用到的一些量
    cv::Mat Ix, Iy;
    cv::Sobel(gray, Ix, CV_64F, 1, 0, 3);
    cv::Sobel(gray, Iy, CV_64F, 0, 1, 3);

    // 计算结构张量的元素
    cv::Mat Ix2, Iy2, Ixy;
    cv::multiply(Ix, Ix, Ix2);
    cv::multiply(Iy, Iy, Iy2);
    cv::multiply(Ix, Iy, Ixy);

    // 高斯平滑
    cv::GaussianBlur(Ix2, Ix2, cv::Size(3, 3), 1.0);
    cv::GaussianBlur(Iy2, Iy2, cv::Size(3, 3), 1.0);
    cv::GaussianBlur(Ixy, Ixy, cv::Size(3, 3), 1.0);

    // 计算每个像素的最小特征值
    cv::Mat minEigenValue(gray.size(), CV_64F);

    for (int y = 0; y < gray.rows; y++) {
        for (int x = 0; x < gray.cols; x++) {
            double a = Ix2.at<double>(y, x);
            double b = Iy2.at<double>(y, x);
            double c = Ixy.at<double>(y, x);

            // 特征值: λ = (a+b ± sqrt((a-b)²+4c²)) / 2
            double discr = sqrt((a - b) * (a - b) + 4 * c * c);
            double lambda1 = (a + b + discr) / 2;
            double lambda2 = (a + b - discr) / 2;

            minEigenValue.at<double>(y, x) = std::min(lambda1, lambda2);
        }
    }

    // 阈值化
    double maxVal;
    cv::minMaxLoc(minEigenValue, nullptr, &maxVal);

    double threshold = qualityLevel * maxVal;

    std::vector<cv::Point2f> corners;
    for (int y = 0; y < gray.rows; y++) {
        for (int x = 0; x < gray.cols; x++) {
            if (minEigenValue.at<double>(y, x) > threshold) {
                corners.emplace_back(x, y);
            }
        }
    }

    // 非极大值抑制（保留质量最好的点）
    if (minDistance > 1) {
        std::sort(corners.begin(), corners.end(),
            [&minEigenValue](const cv::Point2f& a, const cv::Point2f& b) {
                return minEigenValue.at<double>((int)a.y, (int)a.x) >
                       minEigenValue.at<double>((int)b.y, (int)b.x);
            });

        std::vector<cv::Point2f> filtered;
        std::vector<bool> suppressed(corners.size(), false);

        for (size_t i = 0; i < corners.size(); i++) {
            if (suppressed[i]) continue;

            filtered.push_back(corners[i]);

            // 抑制邻近点
            for (size_t j = i + 1; j < corners.size(); j++) {
                double dist = norm(corners[i] - corners[j]);
                if (dist < minDistance) {
                    suppressed[j] = true;
                }
            }

            if (filtered.size() >= (size_t)maxCorners) break;
        }

        return filtered;
    }

    // 限制最大数量
    if ((int)corners.size() > maxCorners) {
        std::partial_sort(corners.begin(), corners.begin() + maxCorners,
                         corners.end(),
            [&minEigenValue](const cv::Point2f& a, const cv::Point2f& b) {
                return minEigenValue.at<double>((int)a.y, (int)a.x) >
                       minEigenValue.at<double>((int)b.y, (int)b.x);
            });
        corners.resize(maxCorners);
    }

    return corners;
}
```

### 7.5 OpenCV中的goodFeaturesToTrack

OpenCV的`goodFeaturesToTrack`函数实现了Shi-Tomasi算法，并提供了额外的改进：

```cpp
void cv::goodFeaturesToTrack(
    InputArray image,              // 输入灰度图像
    OutputArray corners,            // 输出角点
    int maxCorners,                 // 最大角点数量
    double qualityLevel,            // 质量等级（最小特征值的分数）
    double minDistance,             // 角点之间的最小欧氏距离
    InputArray mask = noArray(),    // 感兴趣区域掩码
    int blockSize = 3,              // 计算协方差矩阵的邻域大小
    bool useHarrisDetector = false,// 是否使用Harris检测器
    double k = 0.04                 // Harris参数（仅当useHarrisDetector=true时）
);
```

**使用示例**：

```cpp
#include <opencv2/opencv.hpp>

int main() {
    cv::Mat image = cv::imread("image.jpg");
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;

    // 检测最多100个角点，质量阈值为最大特征值的1%
    // 角点间最小距离为10像素
    cv::goodFeaturesToTrack(
        gray,
        corners,
        100,           // maxCorners
        0.01,          // qualityLevel
        10.0,          // minDistance
        cv::Mat(),     // mask
        3,             // blockSize
        false,         // useHarrisDetector
        0.04            // k
    );

    // 绘制结果
    for (size_t i = 0; i < corners.size(); i++) {
        cv::circle(image, corners[i], 3, cv::Scalar(0, 255, 0), -1);
    }

    cv::imshow("Shi-Tomasi", image);
    cv::waitKey(0);

    return 0;
}
```

---

## 参考文献

1. Harris, C., & Stephens, M. (1988). A combined corner and edge detector. *Alvey Vision Conference*, 147-151.

2. Shi, J., & Tomasi, C. (1994). Good features to track. *Proceedings of IEEE Conference on Computer Vision and Pattern Recognition*, 593-600.

3. OpenCV Documentation: `cornerHarris()`, `goodFeaturesToTrack()`
