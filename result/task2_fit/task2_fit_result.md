# Task 2 参数拟合结果

## 模型

$$\omega(t)=b+A\sin(\Omega t+\phi)$$

角度模型为对角速度积分后的结果，时间原点为视频第 0 帧。

## 参数与误差

| 参数 | 数值 | 单位 |
|---|---:|---|
| A | 0.5499 | rad/s |
| b | 1.35002 | rad/s |
| Omega | 1.64988 | rad/s |
| phi | 0.702743 | rad，范围 [-pi, pi) |
| theta0 | 0.350327 | rad |
| 周期 T | 3.80827 | s |

- 角度 RMSE：0.00234146 rad
- 有效样本数：1440
- 有效帧范围：0--1439
- 求解状态：Ceres Solver Report: Iterations: 4, Initial cost: 1.213176e+00, Final cost: 3.947365e-03, Termination: CONVERGENCE

## 方法与约束

使用已知旋转中心 (480, 360)，在 HSV 中用 H=[75,105]、S=[80,255]、V=[80,255] 提取青色目标，取最大连通区域质心。按题目定义计算角度并用相邻帧展开。先扫描 Omega=0.149--5.0 rad/s 获取线性最小二乘初值，再使用 Ceres 进行非线性最小二乘优化；约束为 A>=0、b>=0、0.001<=Omega<=10。

## 图像与数据

- [tracking_overlay.mp4](tracking_overlay.mp4)
- [fit_comparison.png](fit_comparison.png)
- [angular_velocity.png](angular_velocity.png)
- [residuals.png](residuals.png)
- [angle_fit.csv](angle_fit.csv)
