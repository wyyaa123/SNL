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
#include <typeinfo>
#include <any>
//Eigen库
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
//OpenCV库
#include <opencv2/core.hpp> //新式C++风格的结构以及数学运算
#include <opencv2/highgui.hpp> //新式C++风格的显示、滑动条、鼠标操作以及输入输出相关

//命名空间SNL ： Standard Navigation Library
namespace SNL {
    /**
     * @brief 用于ros节点通讯的类
     * @param THRESHOLD 时间阈值
     * @param subs 订阅者队列
     * @param pubs 发布者队列
     * @param datas 订阅到的话题数据
     */
    class Comunicate  {
        public:
            Comunicate(double sec, int fre) : THRESHOLD(sec) {}
            template<typename _TyPtr1, typename _TyPtr2>
            _TyPtr1 getAilgnedMsg(const _TyPtr2& msgconPtr);
            std::vector<ros::Subscriber>::iterator subBegin();
            std::vector<ros::Publisher>::iterator pubBegin();
            template <typename _TyPtr>
            void emplace(_TyPtr& ptr);
            template <typename _TyPtr>
            size_t size();
            template <typename _TyPtr>
            void pop_front();
            //ros::Publisher& operator[](size_t pos);
            void subEmplace(const ros::Subscriber& sub);
            void pubEmplace(const ros::Publisher& pub);
            size_t pubSize() &&;
            size_t subSize() &&;
        private:
            double THRESHOLD = 0.2;
            std::vector<ros::Subscriber> subs;
            std::vector<ros::Publisher> pubs;
            std::unordered_map<std::string, std::deque<std::any>> datas; 
            template<typename _TyPtr1, typename _TyPtr2>
            std::deque<std::any>::iterator AilgnMsgs(const _TyPtr2& msgconPtr);
    };
    /**
     * @brief 向某一话题消息类型的容器队列尾添加元素
     * 
     * @tparam _TyPtr 消息类型
     * @param msgconPtr 要添加的消息数据
     */
    template <typename _TyPtr>
    void Comunicate::emplace(_TyPtr& msgconPtr) {
        std::string TP = typeid(msgconPtr).name();
        datas[TP].emplace_back(msgconPtr);
    }
    /**
     * @brief 返回对应消息类型队列的元素个数
     * 
     * @tparam _TyPtr 消息类型
     * @return size_t 
     */
    template <typename _TyPtr>
    size_t Comunicate::size() {
        std::string TP = typeid(_TyPtr).name();
        return datas[TP].size();
    }
    /**
     * @brief 删除对应消息类型队列的第一个元素
     * 
     * @tparam _TyPtr 消息类型
     */
    template <typename _TyPtr>
    void Comunicate::pop_front() {
        std::string TP = typeid(_TyPtr).name();
        return datas[TP].pop_front();
    }
    /**
     * @brief 得到与输入元素时间最对应的_TyPtr1类型的话题数据
     * 
     * @tparam _TyPtr1 要与输入消息匹配的话题数据类型
     * @tparam _TyPtr2 输入的消息类型
     * @param msgconPtr 输入的消息
     * @return _TyPtr1 
     */
    template<typename _TyPtr1, typename _TyPtr2>
    _TyPtr1 Comunicate::getAilgnedMsg(const _TyPtr2& msgconPtr) {

        std::string TP = typeid(_TyPtr1).name();
        auto iter = AilgnMsgs<_TyPtr1, _TyPtr2>(msgconPtr);

        if(datas[TP].end() != iter) {
            _TyPtr1 msg = std::any_cast<_TyPtr1>(*iter);
            return msg;
        } 
        return _TyPtr1();
    }
    /**
     * @brief 返回第一个满足在阈值范围内的话题数据的迭代器，并将其之前的话题数据全部删除
     * 
     * @tparam _TyPtr1 要与输入消息匹配的话题数据类型
     * @tparam _TyPtr2 输入的消息类型
     * @param msgconPtr 用于配准的话题数据
     * @return std::deque<std::any>::iterator 
     */
    template<typename _TyPtr1, typename _TyPtr2>
    std::deque<std::any>::iterator Comunicate::AilgnMsgs(const _TyPtr2& msgconPtr) {

        std::string TP = typeid(_TyPtr1).name();
        auto iter = std::find_if(datas[TP].begin(), datas[TP].end(), [&](std::any& any_ptr) -> bool {
                    auto ptr = std::any_cast<_TyPtr1>(any_ptr);
                    double dif = msgconPtr->header.stamp.toSec() - ptr->header.stamp.toSec();
                    return  -THRESHOLD <= dif && dif <= THRESHOLD;
                });
        int cnt = iter - datas[TP].begin();
        while(cnt--) datas[TP].pop_front();
        return iter;
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
    /**
     * @brief 返回当前发布队列的发布者个数
     * @return size_t 
     */
    size_t Comunicate::subSize() && {
        return subs.size();
    }


    /**
     * @brief 相机类
     * 
     */
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
        geometry_msgs::PointStamped cam2UAV(const geometry_msgs::PointStampedConstPtr& camCoor, bool flag);
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
        std::cout << "在rgb像素坐标系下的坐标为: (" << pixelCoor[0] << ", " << pixelCoor[1] << ") " << std::endl;
        std::cout << "目标点深度为:" << depth  << "m" << std::endl;
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
     * @param flag 坐标变换规则
     * @return nav_msgs::Odometry 
     */
    geometry_msgs::PointStamped Cam::cam2UAV(const geometry_msgs::PointStampedConstPtr& camCoor, bool flag = 1) {
        geometry_msgs::PointStamped AircaftCoor;
        if(flag) {
            AircaftCoor.point.x = camCoor->point.z;
            AircaftCoor.point.y = -camCoor->point.x;
            AircaftCoor.point.z = -camCoor->point.y;
        } else {
            AircaftCoor.point.x = camCoor->point.y;
            AircaftCoor.point.y = -camCoor->point.x;
            AircaftCoor.point.z = -camCoor->point.z;  
        }
        return AircaftCoor;
    }
};

#endif