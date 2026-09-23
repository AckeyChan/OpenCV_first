#include <fstream>
#include <filesystem>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>


using namespace cv;

struct AngleObservation
{
    int frame;       // 原视频帧号
    double time;     // 相对第 0 帧的时间，单位为秒
    Point2f point;   // 青色目标的质心坐标
    double angle;    // 按题目约定计算并展开后的角度，单位为 rad
};

struct AngleModelResidual
{
    AngleModelResidual(double time, double observed) : time(time), observed(observed) {}

    template <typename T>
    bool operator()(const T* const parameters, T* residual) const
    {
        // 参数顺序：A、b、Omega、phi、theta0。
        const T amplitude = parameters[0];
        const T meanSpeed = parameters[1];
        const T frequency = parameters[2];
        const T phase = parameters[3];
        const T initialAngle = parameters[4];
        // 对 omega(t) 积分得到 theta(t)，用于直接拟合观测角度。
        const T angle = initialAngle + meanSpeed * T(time)
            + amplitude / frequency * (ceres::cos(phase) -
                                      ceres::cos(frequency * T(time) + phase));
        residual[0] = angle - T(observed);
        return true;
    }

    double time;
    double observed;
};

static double unwrapAngle(double angle, double previous)
{
    // atan2 的输出范围是 [-pi, pi)，目标跨过边界时会产生跳变。
    // 通过加减 2*pi，让相邻帧角度保持连续。
    while (angle - previous > CV_PI)
    {
        angle -= 2.0 * CV_PI;
    }
    while (angle - previous < -CV_PI)
    {
        angle += 2.0 * CV_PI;
    }
    return angle;
}

static bool fitInitialParameters(const std::vector<AngleObservation>& observations,
                                 double parameters[5])
{
    if (observations.size() < 10)
    {
        return false;
    }

    double bestError = std::numeric_limits<double>::max();
    double bestFrequency = 1.0;
    double bestConstant = observations.front().angle;
    double bestMeanSpeed = 0.0;
    double bestCosine = 0.0;
    double bestSine = 0.0;

    // 先扫描可能的频率。固定 w 后，角度模型可改写为
    // angle = c + b*t + p*cos(w*t) + q*sin(w*t)，从而用线性最小二乘求初值。
    for (int step = 1; step <= 100; ++step)
    {
        const double frequency = 0.1 + 0.049 * step;
        Mat design(static_cast<int>(observations.size()), 4, CV_64F);
        Mat values(static_cast<int>(observations.size()), 1, CV_64F);
        for (size_t index = 0; index < observations.size(); ++index)
        {
            const double time = observations[index].time;
            design.at<double>(static_cast<int>(index), 0) = 1.0;
            design.at<double>(static_cast<int>(index), 1) = time;
            design.at<double>(static_cast<int>(index), 2) = std::cos(frequency * time);
            design.at<double>(static_cast<int>(index), 3) = std::sin(frequency * time);
            values.at<double>(static_cast<int>(index), 0) = observations[index].angle;
        }
        Mat coefficients;
        solve(design, values, coefficients, DECOMP_QR);
        const Mat errors = design * coefficients - values;
        const double error = norm(errors, NORM_L2SQR);
        if (error < bestError)
        {
            bestError = error;
            bestFrequency = frequency;
            bestConstant = coefficients.at<double>(0);
            bestMeanSpeed = coefficients.at<double>(1);
            bestCosine = coefficients.at<double>(2);
            bestSine = coefficients.at<double>(3);
        }
    }

    // p、q 对应积分模型中的余弦和正弦系数，据此恢复 A 和 phi。
    const double amplitudeOverFrequency = std::hypot(bestCosine, bestSine);
    parameters[0] = std::max(0.001, bestFrequency * amplitudeOverFrequency);
    parameters[1] = bestMeanSpeed;
    parameters[2] = bestFrequency;
    parameters[3] = std::atan2(bestSine, -bestCosine);
    parameters[4] = observations.front().angle;
    return true;
}

static double modelAngle(const double parameters[5], double time)
{
    // 角度模型是角速度模型的积分形式。
    return parameters[4] + parameters[1] * time
        + parameters[0] / parameters[2]
            * (std::cos(parameters[3]) -
               std::cos(parameters[2] * time + parameters[3]));
}

static double modelSpeed(const double parameters[5], double time)
{
    // 拟合完成后，用原始模型计算每个时刻的角速度。
    return parameters[1] + parameters[0]
        * std::sin(parameters[2] * time + parameters[3]);
}

static void writeTask2Plot(const std::string& path,
                           const std::vector<AngleObservation>& observations,
                           const double parameters[5], bool residualPlot)
{
    // 不依赖额外绘图库，直接生成可在浏览器打开的 SVG 曲线。
    const int width = 1200;
    const int height = 650;
    const int left = 75;
    const int right = 30;
    const int top = 45;
    const int bottom = 65;
    const double maxTime = observations.back().time;
    double minValue = std::numeric_limits<double>::max();
    double maxValue = std::numeric_limits<double>::lowest();
    for (const AngleObservation& observation : observations)
    {
        const double value = residualPlot
            ? observation.angle - modelAngle(parameters, observation.time)
            : observation.angle;
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }
    if (std::abs(maxValue - minValue) < 1e-9)
    {
        maxValue += 1.0;
        minValue -= 1.0;
    }
    const double margin = 0.08 * (maxValue - minValue);
    minValue -= margin;
    maxValue += margin;
    std::ofstream output(path);
    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
           << "\" height=\"" << height << "\">\n";
    output << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
    output << "<text x=\"75\" y=\"25\" font-family=\"sans-serif\" font-size=\"20\">"
           << (residualPlot ? "Angle residual (rad)" : "Observed and fitted angle (rad)")
           << "</text>\n";
    output << "<line x1=\"" << left << "\" y1=\"" << top << "\" x2=\""
           << left << "\" y2=\"" << height - bottom << "\" stroke=\"black\"/>\n";
    output << "<line x1=\"" << left << "\" y1=\"" << height - bottom
           << "\" x2=\"" << width - right << "\" y2=\"" << height - bottom
           << "\" stroke=\"black\"/>\n";
    auto pointX = [&](double time) { return left + time / maxTime * (width - left - right); };
    auto pointY = [&](double value) {
        return height - bottom - (value - minValue) / (maxValue - minValue)
            * (height - top - bottom);
    };
    output << "<polyline fill=\"none\" stroke=\"#555555\" points=\"";
    for (const AngleObservation& observation : observations)
    {
        const double value = residualPlot
            ? observation.angle - modelAngle(parameters, observation.time)
            : observation.angle;
        output << pointX(observation.time) << "," << pointY(value) << " ";
    }
    output << "\"/>\n";
    if (!residualPlot)
    {
        output << "<polyline fill=\"none\" stroke=\"#000000\" points=\"";
        for (const AngleObservation& observation : observations)
        {
            output << pointX(observation.time) << ","
                   << pointY(modelAngle(parameters, observation.time)) << " ";
        }
        output << "\"/>\n";
        output << "<text x=\"900\" y=\"45\" font-family=\"sans-serif\" fill=\"#555555\">observed</text>\n";
        output << "<text x=\"900\" y=\"65\" font-family=\"sans-serif\" fill=\"#000000\">fitted</text>\n";
    }
    output << "</svg>\n";
}

static void writeAngularVelocityPlot(const std::string& path,
                                     const std::vector<AngleObservation>& observations,
                                     const double parameters[5])
{
    // 输出题目要求的估计角速度曲线。
    const int width = 1200;
    const int height = 650;
    const int left = 75;
    const int right = 30;
    const int top = 45;
    const int bottom = 65;
    const double maxTime = observations.back().time;
    const double lower = parameters[1] - parameters[0] - 0.1;
    const double upper = parameters[1] + parameters[0] + 0.1;
    std::ofstream output(path);
    output << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
           << "\" height=\"" << height << "\">\n"
           << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n"
           << "<text x=\"75\" y=\"25\" font-family=\"sans-serif\" font-size=\"20\">Estimated angular velocity (rad/s)</text>\n"
           << "<line x1=\"" << left << "\" y1=\"" << top << "\" x2=\""
           << left << "\" y2=\"" << height - bottom << "\" stroke=\"black\"/>\n"
           << "<line x1=\"" << left << "\" y1=\"" << height - bottom
           << "\" x2=\"" << width - right << "\" y2=\"" << height - bottom
           << "\" stroke=\"black\"/>\n";
    auto pointX = [&](double time) { return left + time / maxTime * (width - left - right); };
    auto pointY = [&](double speed) {
        return height - bottom - (speed - lower) / (upper - lower)
            * (height - top - bottom);
    };
    output << "<polyline fill=\"none\" stroke=\"#000000\" points=\"";
    for (const AngleObservation& observation : observations)
    {
        output << pointX(observation.time) << ","
               << pointY(modelSpeed(parameters, observation.time)) << " ";
    }
    output << "\"/>\n</svg>\n";
}

static void writeTask2Png(const std::string& path,
                          const std::vector<AngleObservation>& observations,
                          const double parameters[5], int plotType)
{
    // plotType: 0 为观测/拟合角度，1 为角度残差，2 为估计角速度。
    const int width = 1200;
    const int height = 650;
    const int left = 80;
    const int top = 55;
    const int right = 35;
    const int bottom = 70;
    const double maxTime = observations.back().time;
    double minValue = std::numeric_limits<double>::max();
    double maxValue = std::numeric_limits<double>::lowest();
    for (const AngleObservation& observation : observations)
    {
        double value = 0.0;
        if (plotType == 0)
        {
            value = observation.angle;
        }
        else if (plotType == 1)
        {
            value = observation.angle - modelAngle(parameters, observation.time);
        }
        else
        {
            value = modelSpeed(parameters, observation.time);
        }
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }
    if (std::abs(maxValue - minValue) < 1e-9)
    {
        maxValue += 1.0;
        minValue -= 1.0;
    }
    const double margin = 0.08 * (maxValue - minValue);
    minValue -= margin;
    maxValue += margin;
    Mat plot(height, width, CV_8UC3, Scalar(255, 255, 255));
    line(plot, Point(left, top), Point(left, height - bottom), Scalar(0, 0, 0), 2);
    line(plot, Point(left, height - bottom), Point(width - right, height - bottom),
         Scalar(0, 0, 0), 2);
    auto toPoint = [&](double time, double value) {
        const int x = left + static_cast<int>(time / maxTime * (width - left - right));
        const int y = height - bottom - static_cast<int>(
            (value - minValue) / (maxValue - minValue) * (height - top - bottom));
        return Point(x, y);
    };
    std::vector<Point> observedPoints;
    std::vector<Point> fittedPoints;
    for (const AngleObservation& observation : observations)
    {
        if (plotType == 0)
        {
            observedPoints.push_back(toPoint(observation.time, observation.angle));
            fittedPoints.push_back(toPoint(observation.time,
                                           modelAngle(parameters, observation.time)));
        }
        else if (plotType == 1)
        {
            observedPoints.push_back(toPoint(
                observation.time, observation.angle - modelAngle(parameters, observation.time)));
        }
        else
        {
            observedPoints.push_back(toPoint(
                observation.time, modelSpeed(parameters, observation.time)));
        }
    }
    if (plotType == 0)
    {
        polylines(plot, observedPoints, false, Scalar(140, 140, 140), 1, LINE_AA);
        polylines(plot, fittedPoints, false, Scalar(0, 0, 0), 2, LINE_AA);
        putText(plot, "observed", Point(width - 210, 30), FONT_HERSHEY_SIMPLEX,
            0.7, Scalar(140, 140, 140), 2);
        putText(plot, "fitted", Point(width - 105, 30), FONT_HERSHEY_SIMPLEX,
            0.7, Scalar(0, 0, 0), 2);
        putText(plot, "Angle: observed vs fitted (rad)", Point(left, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    }
    else if (plotType == 1)
    {
        polylines(plot, observedPoints, false, Scalar(0, 0, 0), 2, LINE_AA);
        putText(plot, "Angle residual (rad)", Point(left, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    }
    else
    {
        polylines(plot, observedPoints, false, Scalar(0, 0, 0), 2, LINE_AA);
        putText(plot, "Estimated angular velocity (rad/s)", Point(left, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);
    }
    putText(plot, "time (s)", Point(width - 120, height - 25), FONT_HERSHEY_SIMPLEX,
            0.6, Scalar(0, 0, 0), 1);
    imwrite(path, plot);
}

static bool runTask2(const std::string& inputPath, const std::string& outputDir)
{
    std::filesystem::create_directories(outputDir);
    VideoCapture video(inputPath);
    if (!video.isOpened())
    {
        std::cerr << "无法打开视频: " << inputPath << std::endl;
        return false;
    }
    const double fps = video.get(CAP_PROP_FPS) > 0 ? video.get(CAP_PROP_FPS) : 60.0;
    // 旋转中心和半径由题目给定；半径用于任务说明，角度计算只需中心坐标。
    const Point2f center(480.0F, 360.0F);
    std::vector<AngleObservation> observations;
    VideoWriter writer;
    Mat frame;
    int frameIndex = 0;
    double previousAngle = 0.0;
    while (video.read(frame))
    {
        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        // OpenCV 的 H 范围是 0--179，青色约位于 H=90 附近。
        Mat mask;
        inRange(hsv, Scalar(75, 80, 80), Scalar(105, 255, 255), mask);
        std::vector<std::vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        // 取面积最大的青色连通区域，避免把零散噪声当作目标。
        int bestContour = -1;
        double bestArea = 0.0;
        for (size_t index = 0; index < contours.size(); ++index)
        {
            const double area = contourArea(contours[index]);
            if (area > bestArea)
            {
                bestArea = area;
                bestContour = static_cast<int>(index);
            }
        }
        if (bestContour >= 0 && bestArea >= 5.0)
        {
            const Moments moments = cv::moments(contours[bestContour]);
            if (moments.m00 > 0.0)
            {
                const Point2f target(static_cast<float>(moments.m10 / moments.m00),
                                     static_cast<float>(moments.m01 / moments.m00));
                // x 向右、y 向下；按题目定义从右方起算，逆时针为正。
                const double rawAngle = std::atan2(center.y - target.y, target.x - center.x);
                const double angle = observations.empty()
                    ? rawAngle : unwrapAngle(rawAngle, previousAngle);
                previousAngle = angle;
                observations.push_back({frameIndex, frameIndex / fps, target, angle});
            }
        }
        ++frameIndex;
    }
    video.release();
    if (observations.size() < 10)
    {
        std::cerr << "青色目标有效样本不足: " << observations.size() << std::endl;
        return false;
    }

    // 先用频率网格搜索得到可靠初值，再交给 Ceres 做非线性优化。
    double parameters[5];
    if (!fitInitialParameters(observations, parameters))
    {
        return false;
    }
    ceres::Problem problem;
    for (const AngleObservation& observation : observations)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<AngleModelResidual, 1, 5>(
                new AngleModelResidual(observation.time, observation.angle)),
            nullptr, parameters);
    }
    // 施加物理约束：振幅、平均角速度和频率均取正值。
    problem.SetParameterLowerBound(parameters, 0, 0.0);
    problem.SetParameterLowerBound(parameters, 1, 0.0);
    problem.SetParameterLowerBound(parameters, 2, 0.001);
    problem.SetParameterUpperBound(parameters, 2, 10.0);
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 角度拟合只报告角度误差；角速度曲线由拟合模型直接计算。
    double angleSquaredError = 0.0;
    for (const AngleObservation& observation : observations)
    {
        const double error = observation.angle - modelAngle(parameters, observation.time);
        angleSquaredError += error * error;
    }
    const double angleRmse = std::sqrt(angleSquaredError / observations.size());
    const double phase = std::atan2(std::sin(parameters[3]), std::cos(parameters[3]));
    const double period = 2.0 * CV_PI / parameters[2];
    std::ofstream markdown(outputDir + "/task2_fit_result.md");
    markdown << "# Task 2 参数拟合结果\n\n"
             << "## 模型\n\n"
             << "$$\\omega(t)=b+A\\sin(\\Omega t+\\phi)$$\n\n"
             << "角度模型为对角速度积分后的结果，时间原点为视频第 0 帧。\n\n"
             << "## 参数与误差\n\n"
             << "| 参数 | 数值 | 单位 |\n|---|---:|---|\n"
             << "| A | " << parameters[0] << " | rad/s |\n"
             << "| b | " << parameters[1] << " | rad/s |\n"
             << "| Omega | " << parameters[2] << " | rad/s |\n"
             << "| phi | " << phase << " | rad，范围 [-pi, pi) |\n"
             << "| theta0 | " << parameters[4] << " | rad |\n"
             << "| 周期 T | " << period << " | s |\n\n"
             << "- 角度 RMSE：" << angleRmse << " rad\n"
             << "- 有效样本数：" << observations.size() << "\n"
             << "- 有效帧范围：" << observations.front().frame << "--"
             << observations.back().frame << "\n"
             << "- 求解状态：" << summary.BriefReport() << "\n\n"
             << "## 方法与约束\n\n"
             << "使用已知旋转中心 (480, 360)，在 HSV 中用 H=[75,105]、S=[80,255]、V=[80,255] 提取青色目标，取最大连通区域质心。按题目定义计算角度并用相邻帧展开。先扫描 Omega=0.149--5.0 rad/s 获取线性最小二乘初值，再使用 Ceres 进行非线性最小二乘优化；约束为 A>=0、b>=0、0.001<=Omega<=10。\n\n"
             << "## 图像与数据\n\n"
             << "- [tracking_overlay.mp4](tracking_overlay.mp4)\n"
             << "- [fit_comparison.png](fit_comparison.png)\n"
             << "- [angular_velocity.png](angular_velocity.png)\n"
             << "- [residuals.png](residuals.png)\n"
             << "- [angle_fit.csv](angle_fit.csv)\n";
    markdown.close();

    std::ofstream csv(outputDir + "/angle_fit.csv");
    csv << "frame,time_s,x,y,observed_angle_rad,fitted_angle_rad,angle_residual_rad, fitted_omega_rad_per_s\n";
    for (const AngleObservation& observation : observations)
    {
        csv << observation.frame << "," << observation.time << ","
            << observation.point.x << "," << observation.point.y << ","
            << observation.angle << "," << modelAngle(parameters, observation.time) << ","
            << observation.angle - modelAngle(parameters, observation.time) << ","
            << modelSpeed(parameters, observation.time) << "\n";
    }
    csv.close();
    writeTask2Png(outputDir + "/fit_comparison.png", observations, parameters, 0);
    writeTask2Png(outputDir + "/residuals.png", observations, parameters, 1);
    writeTask2Png(outputDir + "/angular_velocity.png", observations, parameters, 2);

    // 第二次读取原视频，用检测到的目标位置和拟合参数生成标记视频。
    video.open(inputPath);
    if (!video.isOpened())
    {
        return false;
    }
    const int width = static_cast<int>(video.get(CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(video.get(CAP_PROP_FRAME_HEIGHT));
    writer.open(outputDir + "/tracking_overlay.mp4", VideoWriter::fourcc('m', 'p', '4', 'v'),
                fps, Size(width, height));
    size_t observationIndex = 0;
    frameIndex = 0;
    while (video.read(frame))
    {
        circle(frame, center, 8, Scalar(255, 255, 255), 2);
        if (observationIndex < observations.size() && observations[observationIndex].frame == frameIndex)
        {
            circle(frame, observations[observationIndex].point, 8, Scalar(0, 0, 0), 2);
            ++observationIndex;
        }
        putText(frame, "A=" + std::to_string(parameters[0]) + " b="
                + std::to_string(parameters[1]), Point(20, 35),
                FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 0, 0), 2);
        putText(frame, "Omega=" + std::to_string(parameters[2]) + " phi="
                + std::to_string(phase), Point(20, 65),
                FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 0, 0), 2);
        writer.write(frame);
        ++frameIndex;
    }
    writer.release();
    video.release();
    std::cout << "Task 2 完成：有效样本 " << observations.size()
              << "，角度 RMSE " << angleRmse << " rad。" << std::endl;
    return true;
}

int main()
{
    const std::string inputPath = "../resources/sample.jpg";
    const std::string outputDir = "../result/task1_images";
    std::filesystem::create_directories(outputDir);

    Mat image = imread(inputPath);
    if (image.empty())
    {
        std::cerr << "无法读取图像: " << inputPath << std::endl;
        return -1;
    }

    // 1. 灰度图以及 HSV 的 H、S、V 单通道图。
    Mat grayImage, hsvImage;
    cvtColor(image, grayImage, COLOR_BGR2GRAY);
    cvtColor(image, hsvImage, COLOR_BGR2HSV);
    std::vector<Mat> hsvChannels;
    split(hsvImage, hsvChannels);
    imwrite(outputDir + "/01_gray.jpg", grayImage);
    imwrite(outputDir + "/02_h_channel.jpg", hsvChannels[0]);
    imwrite(outputDir + "/03_s_channel.jpg", hsvChannels[1]);
    imwrite(outputDir + "/04_v_channel.jpg", hsvChannels[2]);

    // 2.三种滤波对边缘和细节的影响。
    Mat meanImage, gaussianImage, medianImage;
    blur(image, meanImage, Size(5, 5));
    GaussianBlur(image, gaussianImage, Size(5, 5), 1.5);
    medianBlur(image, medianImage, 5);
    imwrite(outputDir + "/05_mean_5x5.jpg", meanImage);
    imwrite(outputDir + "/06_gaussian_5x5_sigma1.5.jpg", gaussianImage);
    imwrite(outputDir + "/07_median_5.jpg", medianImage);

    // 3. 红色 HSV 双区间阈值
    Mat lowRedMask, highRedMask, redMask;
    inRange(hsvImage, Scalar(0, 100, 100), Scalar(10, 255, 255), lowRedMask);
    inRange(hsvImage, Scalar(160, 100, 100), Scalar(179, 255, 255), highRedMask);
    redMask = lowRedMask | highRedMask;
    imwrite(outputDir + "/08_red_mask.jpg", redMask);

    // 4. 在红色掩膜上做形态学处理，再提取并筛选外轮廓。
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat erodedImage, dilatedImage, openedImage, closedImage;
    erode(redMask, erodedImage, kernel);
    dilate(redMask, dilatedImage, kernel);
    morphologyEx(redMask, openedImage, MORPH_OPEN, kernel);
    morphologyEx(redMask, closedImage, MORPH_CLOSE, kernel);
    imwrite(outputDir + "/09_erode_red_mask.jpg", erodedImage);
    imwrite(outputDir + "/10_dilate_red_mask.jpg", dilatedImage);
    imwrite(outputDir + "/11_open_red_mask.jpg", openedImage);
    imwrite(outputDir + "/12_close_red_mask.jpg", closedImage);

    std::vector<std::vector<Point>> contours;
    findContours(closedImage.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    const double minArea = 5000.0;
    const double maxArea = 10000000.0;
    Mat contourResult = image.clone();

    std::ofstream areaReport(outputDir + "/contour_areas.txt");
    areaReport << "筛选条件：" << minArea << " px < 面积 < " << maxArea
               << " px，且外接矩形宽高比在 [0.2, 5.0] 内\n";

    int selectedCount = 0;
    for (size_t index = 0; index < contours.size(); ++index)
    {
        const double area = contourArea(contours[index]);
        const Rect box = boundingRect(contours[index]);
        const double ratio = static_cast<double>(box.width) / box.height;
        if (area <= minArea || area >= maxArea || ratio > 5.0 || ratio < 0.2)
        {
            continue;
        }
        drawContours(contourResult, contours, static_cast<int>(index), Scalar(0, 0, 255), 2);
        rectangle(contourResult, box, Scalar(0, 255, 0), 2);
        areaReport << "轮廓 " << ++selectedCount << ": " << area << " px\n";
    }
    areaReport.close();
    imwrite(outputDir + "/13_filtered_contours.jpg", contourResult);

    // 5. 绘制、绕中心旋转 35 度，以及原图左上四分之一裁剪。
    Mat drawnImage = image.clone();
    circle(drawnImage, Point(image.cols / 2, image.rows / 2),
                    std::min(image.cols, image.rows) / 5, Scalar(255, 0, 0), 3);
    rectangle(drawnImage, Rect(image.cols / 10, image.rows / 10,
                               image.cols / 3, image.rows / 3), Scalar(0, 255, 0), 3);
    putText(drawnImage, "OpenCV", Point(30, image.rows - 30),
            FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255), 3);
    Mat rotationMatrix = getRotationMatrix2D(
        Point2f(image.cols / 2.0F, image.rows / 2.0F), 35.0, 1.0);
    Mat rotatedImage;
    warpAffine(drawnImage, rotatedImage, rotationMatrix, drawnImage.size());
    Mat croppedImage = image(Rect(0, 0, image.cols / 2, image.rows / 2)).clone();
    imwrite(outputDir + "/14_drawings.jpg", drawnImage);
    imwrite(outputDir + "/15_rotated_35deg.jpg", rotatedImage);
    imwrite(outputDir + "/16_crop_top_left_quarter.jpg", croppedImage);

    std::cout << "处理完成：输入图像 " << image.cols << "x" << image.rows
              << "，输出 16 张图，筛选轮廓 " << selectedCount << " 个。" << std::endl;
    if (!runTask2("../resources/task_2.mp4", "../result/task2_fit"))
    {
        return -1;
    }




    return 0;
}

