#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
/*
    task1
*/
int main(int argc, char** argv)
{
    // 读取图像
    Mat image = imread("../resources/sample.jpg");
    if (image.empty())
    {
        std::cerr << "无法读取图像" << std::endl;
        return -1;
    }

    // 转换为灰度图像
    Mat grayImage;
    cvtColor(image, grayImage, COLOR_BGR2GRAY);

    imwrite("../result/task1_images/gray_test.jpg", grayImage);

    // 图像滤波
    Mat meanImage, gaussianImage, medianImage;

    blur(image, meanImage, Size(5, 5));
    GaussianBlur(image, gaussianImage, Size(5, 5), 1.5);
    medianBlur(image, medianImage, 5);

    imwrite("../result/task1_images/mean_filter.jpg", meanImage);
    imwrite("../result/task1_images/gaussian_filter.jpg", gaussianImage);
    imwrite("../result/task1_images/median_filter.jpg", medianImage);

    //图像二值化
    Mat binaryImage, adaptiveImage;

    threshold(grayImage, binaryImage, 127, 255, THRESH_BINARY);
    adaptiveThreshold(grayImage, adaptiveImage, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);

    imwrite("../result/task1_images/binary_image.jpg", binaryImage);
    imwrite("../result/task1_images/adaptive_binary_image.jpg", adaptiveImage);

    //Canny边缘检测
    Mat cannyImage,gussianGreyImage;

    GaussianBlur(grayImage, gussianGreyImage, Size(5, 5), 1.5);
    Canny(gussianGreyImage, cannyImage, 100, 200);

    imwrite("../result/task1_images/canny_edge.jpg", cannyImage);

    return 0;
}