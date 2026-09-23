#include <fstream>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

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
    return 0;
}

