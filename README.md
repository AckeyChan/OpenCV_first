# OpenCV 图像处理实验

## Task 1

### 1. 实验说明

使用 OpenCV 对 `resources/sample.jpg` 完成读图、灰度化、HSV 分离、滤波、红色提取、形态学处理、外轮廓筛选、绘制、旋转和裁剪。输入图像尺寸为 `6000 x 4000`，结果保存到 `result/task1_images/`。

### 2. 编译运行

```bash
cmake -S . -B build
cmake --build build
cd build
./OpenCV_first
```

程序会检查图像是否读取成功；读取失败时输出错误信息并退出。正常运行后输出 16 张图和 `contour_areas.txt`。

### 3. 参数

#### 读图与颜色转换

- 输入：`../resources/sample.jpg`
- 灰度：`COLOR_BGR2GRAY`
- HSV：`COLOR_BGR2HSV`
- H 范围：`0--179`；S、V 范围：`0--255`

#### 滤波

- 均值滤波：`blur`，核 `5 x 5`
- 高斯滤波：`GaussianBlur`，核 `5 x 5`，`sigma = 1.5`
- 中值滤波：`medianBlur`，核尺寸 `5`

#### 红色掩膜

采用 HSV 双区间阈值，并将两个掩膜按位或：

- 低色相区间：`H=[0,10]`，`S=[100,255]`，`V=[100,255]`
- 高色相区间：`H=[160,179]`，`S=[100,255]`，`V=[100,255]`
- 代码：`redMask = lowRedMask | highRedMask`

#### 形态学与轮廓

- 结构元素：椭圆形 `MORPH_ELLIPSE`
- 核尺寸：`5 x 5`
- 输出：腐蚀、膨胀、开运算、闭运算
- 轮廓：对闭运算结果使用 `RETR_EXTERNAL` 和 `CHAIN_APPROX_SIMPLE`
- 面积条件：`5000 px < area < 10000000 px`
- 外接矩形宽高比：`0.2 <= width / height <= 5.0`
- 轮廓颜色：红色；外接矩形颜色：绿色


#### 绘制与变换

- 圆：圆心为图像中心，半径为 `min(width, height) / 5`
- 矩形：左上角为 `(width / 10, height / 10)`，尺寸为 `width / 3 x height / 3`
- 文字：`OpenCV`，`FONT_HERSHEY_SIMPLEX`，字号 `1.5`，线宽 `3`
- 旋转：绕图像中心旋转 `35` 度
- 裁剪：原图左上角区域，宽高各取一半

### 4. 输出文件（16 张）

| 编号 | 文件 | 内容 |
|---|---|---|
| 01 | `01_gray.jpg` | 灰度图 |
| 02--04 | `02_h_channel.jpg`、`03_s_channel.jpg`、`04_v_channel.jpg` | H、S、V 单通道图 |
| 05--07 | `05_mean_5x5.jpg`、`06_gaussian_5x5_sigma1.5.jpg`、`07_median_5.jpg` | 三种滤波结果 |
| 08 | `08_red_mask.jpg` | 红色掩膜 |
| 09--12 | `09_erode_red_mask.jpg` 至 `12_close_red_mask.jpg` | 四种形态学结果 |
| 13 | `13_filtered_contours.jpg` | 原图上的筛选轮廓和外接矩形 |
| 14 | `14_drawings.jpg` | 圆、矩形和文字 |
| 15 | `15_rotated_35deg.jpg` | 旋转 35 度结果 |
| 16 | `16_crop_top_left_quarter.jpg` | 左上角四分之一裁剪图 |

### 5. 轮廓面积记录

面积明细见 `result/task1_images/contour_areas.txt`。当前运行筛选出 6 个轮廓：

| 轮廓 | 面积 |
|---|---:|
| 1 | 6493.5 px |
| 2 | 118348 px |
| 3 | 6798.5 px |
| 4 | 18516 px |
| 5 | 1001470 px |
| 6 | 4976450 px |

## Task 2：合成旋转视频参数拟合

### 1. 方法与参数

输入视频为 `resources/task_2.mp4`，视频参数为 `960 x 720`、`60 FPS`、`24 s`、`1440` 帧。使用已知旋转中心 `(480, 360)` 和半径 `220 px`，在 HSV 中提取青色目标（`H=[75,105]`、`S=[80,255]`、`V=[80,255]`），取最大青色连通区域质心作为目标位置。

按题目定义计算角度：

```text
theta = atan2(center_y - target_y, target_x - center_x)
```

随后按相邻帧展开角度，并以第 0 帧为时间原点，直接拟合：

```text
omega(t) = b + A sin(Omega t + phi)
theta(t) = theta0 + b*t + A/Omega * (cos(phi) - cos(Omega*t + phi))
```

先对 `Omega` 做 `0.149--5.0 rad/s` 的频率网格搜索，线性求解初值，再使用 Ceres 优化 `A、b、Omega、phi、theta0`。约束为 `A >= 0`、`b >= 0`、`0.001 <= Omega <= 10`，相位统一到 `[-pi, pi)`。

### 2. 输出与报告

结果保存到 `result/task2_fit/`：

- `task2_fit_result.md`：模型、A、b、Omega、phi、theta0、周期、角度 RMSE、有效样本数、帧范围、方法和 Ceres 状态
- `angle_fit.csv`：帧号、时间、目标坐标、观测角度、拟合角度、角度残差和拟合角速度
- `tracking_overlay.mp4`：标出旋转中心、青色目标和识别参数的视频
- `fit_comparison.png`：观测角度与拟合角度对比图
- `angular_velocity.png`：估计角速度曲线
- `residuals.png`：角度残差曲线

当前运行结果：有效样本 `1440`，帧范围 `0--1439`，角度 RMSE `0.00234146 rad`，Ceres 优化收敛。

### 3. 目录结构

```text
OpenCV_first/
├── CMakeLists.txt
├── README.md
├── resources/sample.jpg
├── src/main.cpp
└── result/
    ├── task1_images/
    │   ├── 01_gray.jpg ... 16_crop_top_left_quarter.jpg
    │   └── contour_areas.txt
    └── task2_fit/
        ├── task2_fit_result.md
        ├── angle_fit.csv
        ├── fit_comparison.png
        ├── angular_velocity.png
        ├── residuals.png
        └── tracking_overlay.mp4
```
