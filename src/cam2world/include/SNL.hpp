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
     * @param time ros的Time类
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

    template <typename _TyPtr>
    void Comunicate::emplace(_TyPtr& msgconPtr) {
        std::string TP = typeid(msgconPtr).name();
        datas[TP].emplace_back(msgconPtr);
    }

    template <typename _TyPtr>
    size_t Comunicate::size() {
        std::string TP = typeid(_TyPtr).name();
        return datas[TP].size();
    }

    template <typename _TyPtr>
    void Comunicate::pop_front() {
        std::string TP = typeid(_TyPtr).name();
        return datas[TP].pop_front();
    }

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
     * @param coorAtCamPtr 用于配准的话题数据
     * @return std::deque<geometry_msgs::PointStampedConstPtr>::iterator 
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
};

#endif