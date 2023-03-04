#ifndef __SNL_HPP
#define __SNL_HPP 1

//ros相关标准库
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <message_filters/simple_filter.h>
#include <cv_bridge/cv_bridge.h>
//STL标准库
#include <vector>
#include <deque>
#include <queue> //队列
#include <unordered_map>
#include <algorithm>
#include <iostream>
//Eigen库
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
//OpenCV库
#include <opencv4/opencv2/core.hpp> //新式C++风格的结构以及数学运算
#include <opencv4/opencv2/highgui.hpp> //新式C++风格的显示、滑动条、鼠标操作以及输入输出相关

//命名空间SNL ： Standard Navigation Library
namespace SNL {
    /**
     * @brief 用于ros节点通讯的类
     * @param THRESHOLD 时间阈值
     * @param subs 订阅者队列
     * @param pubs 发布者队列
     * @param time ros的Time类
     * @param datas 订阅到的话题数据
     */
    class Comunicate {
        public:
            ros::NodeHandle nh;
            Comunicate(double sec) : THRESHOLD(sec) {}
            std::deque<geometry_msgs::PointStampedConstPtr>::iterator&  AilgnMsgs(const geometry_msgs::PointStampedConstPtr& coorAtCamPtr);
            std::vector<ros::Subscriber>::iterator subBegin();
            std::vector<ros::Publisher>::iterator pubBegin();
            void subEmplace(const ros::Subscriber& sub);
            void pubEmplace(const ros::Publisher& pub);
            size_t pubSize() &&;
            size_t subSize() &&;
        private:
            double THRESHOLD = 0.2;
            std::vector<ros::Subscriber> subs = {5, ros::Subscriber()};
            std::vector<ros::Publisher> pubs = {5, ros::Publisher()};
            std::vector<std::deque<todo>> datas;
            ros::Time time = ros::Time(THRESHOLD);
    };

    /**
     * @brief 返回第一个满足在阈值范围内的话题数据的迭代器，并将其之前的话题数据全部删除
     * 
     * @param coorAtCamPtr 用于配准的话题数据
     * @return std::deque<geometry_msgs::PointStampedConstPtr>::iterator 
     */
    std::deque<geometry_msgs::PointStampedConstPtr>::iterator& Comunicate::AilgnMsgs(const geometry_msgs::PointStampedConstPtr& coorAtCamPtr) {

        auto iter = std::find_if(datas.begin(), datas.end(), [=](nav_msgs::OdometryConstPtr& ptr) -> bool {
                    double dif = coorAtCamPtr->header.stamp.toSec() - ptr->header.stamp.toSec();
                    return  -THRESHOLD <= dif && dif <= THRESHOLD;
                });

        return iter = datas.erase(datas.begin(), iter);
    }
    /**
     * @brief 返回第一个订阅者
     * 
     * @return std::vector<ros::Subscriber>::iterator 
     */
    std::vector<ros::Subscriber>::iterator Comunicate::subBegin() {
        return subs.begin();
    }
    /**
     * @brief 返回第一个发布者
     * 
     * @return std::vector<ros::Publisher>::iterator 
     */
    std::vector<ros::Publisher>::iterator Comunicate::pubBegin() {
        return pubs.begin();
    }
    /**
     * @brief 增长发布队列的订阅者个数
     * 
     * @param sub 需要添加的订阅者
     */
    void Comunicate::subEmplace(const ros::Subscriber& sub) {
        subs.emplace_back(sub);
    }
    /**
     * @brief 增长发布队列的发布者个数
     * 
     * @param pub 需要添加的发布者
     */
    void Comunicate::pubEmplace(const ros::Publisher& pub) {
        pubs.emplace_back(pub);
    }
    /**
     * @brief 返回当前发布队列的发布者个数
     * 
     * @return size_t 
     */
    size_t Comunicate::pubSize() && {
        return pubs.size();
    }

    size_t Comunicate::subSize() && {
        return subs.size();
    }

    class Cam {
    public:
        Cam() { this->Intrinsic << fx, 0, cx, 0, fy, cy, 0, 0, 1; };
        /**
         * @brief Construct a new Cam object
         * 
         * @param fx 内参系数
         * @param fy 内参系数
         * @param cx 内参系数
         * @param cy 内参系数
         */
        Cam(double fx, double fy, double cx, double cy) : fx(fx), fy(fy), cx(cx), cy(cy) { this->Intrinsic << fx, 0, cx, 0, fy, cy, 0, 0, 1; }
        geometry_msgs::PointStamped pixel2Cam(const Eigen::Vector2d& pixelCoor, float depth) &;
        geometry_msgs::PointStamped cam2UAV(const geometry_msgs::PointStampedConstPtr& camCoor);
    private:
        Eigen::Matrix3d Intrinsic;
        const double fx = 376.0;
        const double fy = 376.0;
        const double cx = 376.0;
        const double cy = 240.0;
    };

    /**
     * @brief 返回相机坐标系下的[x, y, z], 返回值只能做右值
     * 
     * @param pixelCoor 像素坐标系下的[x, y]
     * @param depth 像素点的深度，默认1
     * @return nav_msgs::Odometry 
     */
    geometry_msgs::PointStamped Cam::pixel2Cam(const Eigen::Vector2d& pixelCoor, float depth = 1) & {
        // std::cout << "在rgb像素坐标系下的坐标为: (" << pixelCoor[0] << ", " << pixelCoor[1] << ") " << std::endl;
        // std::cout << "目标点深度为:" << depth  << "m" << std::endl;
        // std::cout << "相机内参为:" << this->fx << ", " << this->fy << ", " << this->cx << ", " << this->cy << std::endl;
        Eigen::Vector3d coor = { pixelCoor[0], pixelCoor[1], 1 }; 
        Eigen::Vector3d xyz = this->Intrinsic.inverse() * depth * coor;
        geometry_msgs::PointStamped Point;
        
        Point.point.x = xyz[0];
        Point.point.y = xyz[1];
        Point.point.z = xyz[2];
        std::cout << "在相机坐标系下的坐标为: (" << xyz[0] << ", " 
                                             << xyz[1] << ", "
                                             << xyz[2] << ") "
                                             << std::endl;
        std::cout << "------------------------------" << std::endl;
        return Point;
    } 
    /**
     * @brief 返回机体坐标系下的[x, y, z], 返回值只能做右值
     * 
     * @param camCoor 相机坐标系下的坐标
     * @return nav_msgs::Odometry 
     */
    geometry_msgs::PointStamped Cam::cam2UAV(const geometry_msgs::PointStampedConstPtr& camCoor) {
        geometry_msgs::PointStamped AircaftCoor;
        AircaftCoor.point.x = camCoor->point.z;
        AircaftCoor.point.y = -camCoor->point.x;
        AircaftCoor.point.z = -camCoor->point.y;
        return AircaftCoor;
    }
};

#endif