#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "../include/randomNum.hpp"
using namespace std;

void findCornerPoints (int argc, char** argv);

void LK_opticalFlow (int argc, char** argv);

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

    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 40, 0.01);//最大迭代次数为40， 精度为精确到0.01

    cv::cornerSubPix(gray_img, corner_points, cv::Size(5, 5), cv::Size(-1, -1), criteria);///查找亚像素角点

    for(auto& point : corner_points) {
        auto random1 = SNL::getRandomNum(0, 255);
        auto random2 = SNL::getRandomNum(0, 255);
        auto random3 = SNL::getRandomNum(0, 255);
        cout << random1 << " " << random2 << " " << random3 << endl;
        cv::circle(img, point, 5, (random1, random2, random3), -1); //着点
    }

    cv::imshow(argv[1], img);
    cv::waitKey();

    cv::destroyWindow(argv[1]);
}

void LK_opticalFlow (int argc, char** argv) {
    /*
    void calcOpticalFlowPyrLK (cv::InputArray prevImg, 
                               cv::InputArray nextImg, 
                               cv::InputArray prevPts, 
                               cv::InputOutputArray nextPts, 
                               cv::OutputArray status, 
                               cv::OutputArray err, 
                               cv::Size winSize = cv::Size(21, 21), //跟踪窗口大小
                               int maxLevel = 3, //金字塔层数
                               cv::TermCriteria criteria = cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 
                                                                             30, 
                                                                             (0.01)), //算法终止判断标准
                               int flags = 0, 
                               double minEigThreshold = (0.0001))
    */
    if(argc != 3) {
        cout << "Call: " << argv[0] <<  "[image1] [image2]" << endl;
        cout << "Demonstrates Pyramid Lucas-Kanada optical flow. " << endl;
        exit(-1);
    }

    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    cv::Size img_sz = img1.size();
    int win_size = 10;
    cv::Mat img3 = cv::imread(argv[2], cv::IMREAD_UNCHANGED);

    vector<cv::Point2f> corners1, corners2;
    const int MAX_CORNERS = 500;
    cv::goodFeaturesToTrack (img1, corners1, MAX_CORNERS, .01, 5); //计算改善型的Harris角点

    cv::cornerSubPix(img1, 
                     corners1, 
                     cv::Size(win_size, win_size), 
                     cv::Size(-1, -1), 
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03)); //亚像素级别角点

    vector<uchar> features_found;

    cv::calcOpticalFlowPyrLK(img1, img2, 
                             corners1, corners2, 
                             features_found, cv::noArray(), //status的每个元素都会提升是否找到了prevPts中的相应特征
                             cv::Size(win_size* 2 + 1, win_size * 2 + 1), 5, 
                             cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03));

    for(int i = 0; i != corners1.size(); ++i) {
        if(!features_found[i]) continue;
        cv::line(img3, corners1[i], corners2[i], (0, 0, 255), 2, cv::LINE_AA);
    }
    cv::imshow("img1", img1);
    cv::imshow("img2", img2);
    cv::imshow("LK Optical Flow Example", img3);
    cv::waitKey(0);
}
