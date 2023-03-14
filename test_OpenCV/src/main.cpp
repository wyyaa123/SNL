#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>

#include "../include/randomNum.hpp"
using namespace std;

void findCornerPoints (int argc, char** argv);

int main (int argc, char** argv) {

    findCornerPoints(argc, argv);

    return 0;
}

void findCornerPoints (int argc, char** argv) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat gray_img;

    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY); //使用灰度图进行角点检测。
    vector<cv::Point2f> corner_points;

    cv::goodFeaturesToTrack(gray_img, corner_points, 100, 0.1, 10); //查找像素级角点

    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);

    cv::cornerSubPix(gray_img, corner_points, cv::Size(5, 5), cv::Size(-1, -1), criteria);

    for(auto& point : corner_points) {
        auto random1 = getRandomNum(0, 255);
        auto random2 = getRandomNum(0, 255);
        auto random3 = getRandomNum(0, 255);
        cout << random1 << " " << random2 << " " << random3 << endl;
        cv::circle(img, point, 5, (random1, random2, random3), -1); //着点
    }

    cv::imshow(argv[1], img);
    cv::waitKey();

    cv::destroyWindow(argv[1]);
}
