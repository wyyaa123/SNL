#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "../include/randomNum.hpp"
using namespace std;

void findCornerPoints(cv::InputOutputArray&, cv::InputOutputArray&, const cv::TermCriteria&);

void match(string type, cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);

void detect_and_compute(std::string type, cv::Mat& img, std::vector<cv::KeyPoint>& kpts, cv::Mat& desc);

void LK_opticalFlow(int argc, char** argv);

void calVO(std::vector<cv::KeyPoint>& kpt1, std::vector<cv::KeyPoint>& kpt2, std::vector<cv::DMatch>& matches, cv::Mat& R, cv::Mat& t, vector<cv::Point3f>& points);

void drawPlots(cv::InputOutputArray& img, vector<cv::Point2f>& points);

int main (int argc, char** argv) {

    /*if(argc == 1) {
        cout << "图像个数不对" << endl;
        exit(-1);
    }*/

    cv::Mat img1 = cv::imread("C:\\Users\\南九的橘猫\\Desktop\\SNL\\test_OpenCV\\images\\blur_2.png", cv::ImreadModes::IMREAD_COLOR);
    cv::Mat img2 = cv::imread("C:\\Users\\南九的橘猫\\Desktop\\SNL\\test_OpenCV\\images\\gt_2.png", cv::ImreadModes::IMREAD_COLOR);
    vector<cv::Point2f> _corner_points1;
    vector<cv::Point2f> _corner_points2;
    cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 300, 0.01);//最大迭代次数为40， 精度为精确到0.01

    findCornerPoints(img1, _corner_points1, criteria);
    findCornerPoints(img2, _corner_points2, criteria);

    drawPlots(img1, _corner_points1);
    drawPlots(img2, _corner_points2);

    cv::imshow("corner_points1", img1);
    cv::imshow("corner_points2", img2);
    cv::waitKey();

    cv::destroyAllWindows();

    cv::imwrite("C:\\Users\\南九的橘猫\\Desktop\\SNL\\test_OpenCV\\images\\blur_2_point.png", img1);
    cv::imwrite("C:\\Users\\南九的橘猫\\Desktop\\SNL\\test_OpenCV\\images\\gt_2_point.png", img2);

    return 0;
}

void findCornerPoints (cv::InputOutputArray& _img, cv::InputOutputArray& _corner_points, const cv::TermCriteria& criteria) {
    cv::Mat gray_img;
    cv::cvtColor(_img, gray_img, cv::COLOR_BGR2GRAY); //使用灰度图进行角点检测。

    cv::goodFeaturesToTrack(gray_img, _corner_points, 100, 0.1, 10); //查找像素级角点

    cv::cornerSubPix(gray_img, _corner_points, cv::Size(5, 5), cv::Size(-1, -1), criteria);///查找亚像素角点
}

void LK_opticalFlow (int argc, char** argv) {

    clock_t beg = clock();

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

    clock_t end = clock();

    cout << "total expend " << double(end - beg) / CLOCKS_PER_SEC << "seconds" << endl;

    cv::imshow("img1", img1);
    cv::imshow("img2", img2);
    cv::imshow("LK Optical Flow Example", img3);
    cv::waitKey(0);
}

void calVO(std::vector<cv::KeyPoint>& kpt1, std::vector<cv::KeyPoint>& kpt2, 
           std::vector<cv::DMatch>& matches, 
           cv::Mat& R, cv::Mat& t, 
           vector<cv::Point3f>& points,
           vector<char>& match_mask, 
           cv::Point2f principal_point,
           double focal_length) {


}

void drawPlots(cv::InputOutputArray& img, vector<cv::Point2f>& points) {
    for (auto& point : points) {
        auto random1 = SNL::getRandomNum(0, 255);
        auto random2 = SNL::getRandomNum(0, 255);
        auto random3 = SNL::getRandomNum(0, 255);
        cv::circle(img, point, 5, (random1, random2, random3), -1); //着点
    }
}

/**
 * @brief 计算关键子和描述符
 * 
 * @param type 使用的算法：ORB特征点, FAST角点, BLOB角点, SURF特征点, BRISK特征点, KAZE特征点, AKAZE特征点, FREAK角点, DAISY角点, BRIEF角点。使用时请输入他们的英文缩写
 * @param img 用于提取角点/特征点的图像
 * @param kpts 关键子
 * @param desc 描述符
 */
inline void detect_and_compute(std::string type, cv::Mat& img, std::vector<cv::KeyPoint>& kpts, cv::Mat& desc) {
    std::for_each(type.begin(), type.end(), [](char& c) ->void { c = std::tolower(c); });
    if (type.find("fast") == 0) {
        type = type.substr(4);
        cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(10, true);
        detector->detect(img, kpts);
    }
    if (type.find("blob") == 0) {
        type = type.substr(4);
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create();
        detector->detect(img, kpts);
    }
    if (type == "surf") {
        cv::Ptr<cv::Feature2D> surf = cv::xfeatures2d::SURF::create(800.0);
        surf->detectAndCompute(img, cv::Mat(), kpts, desc);
    }
    // if (type == "sift") {
    //     Ptr<Feature2D> sift = SIFT::create();
    //     sift->detectAndCompute(img, Mat(), kpts, desc);
    // }
    if (type == "orb") {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->detectAndCompute(img, cv::Mat(), kpts, desc);
    }
    if (type == "brisk") {
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->detectAndCompute(img, cv::Mat(), kpts, desc);
    }
    if (type == "kaze") {
        cv::Ptr<cv::KAZE> kaze = cv::KAZE::create();
        kaze->detectAndCompute(img, cv::Mat(), kpts, desc);
    }
    if (type == "akaze") {
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->detectAndCompute(img, cv::Mat(), kpts, desc);
    }
    if (type == "freak") { 
        cv::Ptr<cv::xfeatures2d::FREAK> freak = cv::xfeatures2d::FREAK::create();
        freak->compute(img, kpts, desc);
    }
    if (type == "daisy") {
        cv::Ptr<cv::xfeatures2d::DAISY> daisy = cv::xfeatures2d::DAISY::create();
        daisy->compute(img, kpts, desc);
    }
    if (type == "brief") {
        cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief = cv::xfeatures2d::BriefDescriptorExtractor::create(64);
        brief->compute(img, kpts, desc);
    }
}

/**
 * @brief 用于特征点匹配
 * 
 * @param type 匹配方法：暴力匹配 bf; k-nearest neighbors(fnn)匹配
 * @param desc1 查询图像中的特征点的描述符
 * @param desc2 目标图像中的特征点的描述符
 * @param matches 输出的匹配结果
 */
inline void match(string type, cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches) {

    const double kDistanceCoef = 4.0;
    const int kMaxMatchingSize = 50;

    matches.clear();
    if (type == "bf") {
        cv::BFMatcher desc_matcher(cv::NORM_L2, true); //欧几里得距离
        desc_matcher.match(desc1, desc2, matches, cv::Mat());
    } else if (type == "knn") {
        cv::BFMatcher desc_matcher(cv::NORM_L2, true);
        vector< vector<cv::DMatch> > vmatches;
        desc_matcher.knnMatch(desc1, desc2, vmatches, 1);
        for (int i = 0; i < static_cast<int>(vmatches.size()); ++i) {
            if (!vmatches[i].size()) {
                continue;
            }
            matches.push_back(vmatches[i][0]);
        }
    } else if (type == "flnn") {
        cv::FlannBasedMatcher flnn_matcher;
        flnn_matcher.match(desc1, desc2, matches);
    }

    std::sort(matches.begin(), matches.end());
    while (matches.front().distance * kDistanceCoef < matches.back().distance) {
        matches.pop_back();
    }
    while (matches.size() > kMaxMatchingSize) {
        matches.pop_back();
    }
}
