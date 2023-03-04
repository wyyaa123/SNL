#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "../include/SNL.hpp"
#include "../include/nrc.hpp"

#include <deque>
#include <iostream>
#include <algorithm>

#define THRESHOLD 0.2

void initQuaternions(const nav_msgs::OdometryConstPtr& aircraft);
void Alter2Cam(const geometry_msgs::PointStampedConstPtr& coorAtCamPtr);

Eigen::Vector3d t;
Eigen::Quaternion<double> qu;
ros::Publisher coor_pub;
std::deque<nav_msgs::OdometryConstPtr> kf;

SNL::Comunicate com1(0.5, 100);

int main(int argc, char** argv) {

    ros::init(argc, argv, "cam2world");
    
    ros::NodeHandle nh;
    ros::Rate time = ros::Rate(100);
    coor_pub = nh.advertise<geometry_msgs::PointStamped>("cam2world/coor", 10);

    ros::Subscriber kfstate_sub = nh.subscribe<nav_msgs::Odometry>("/kf_state", 10, initQuaternions);
    ros::Subscriber coorAtCam = nh.subscribe<geometry_msgs::PointStamped>("pixel2cam/pointCoor", 10, Alter2Cam);
    while(ros::ok()) {
        ros::spinOnce();
        time.sleep();
    }

    return 0;
}

void initQuaternions(const nav_msgs::OdometryConstPtr& aircraft) {
    com1.emplace(aircraft);
    if(com1.size<nav_msgs::OdometryConstPtr>() > 200) com1.pop_front<nav_msgs::OdometryConstPtr>();
    // ::t = { aircraft->pose.pose.position.x, aircraft->pose.pose.position.y, aircraft->pose.pose.position.z };
    // ::qu = Eigen::Quaterniond(aircraft->pose.pose.orientation.w, aircraft->pose.pose.orientation.x, aircraft->pose.pose.orientation.y, aircraft->pose.pose.orientation.z);
}

void Alter2Cam(const geometry_msgs::PointStampedConstPtr& coorAtCamPtr) {

    bool i = std::system("clear");

    std::cout << "四元数数组大小为：" << com1.size<nav_msgs::OdometryConstPtr>() << std::endl;

    auto iter = com1.getAilgnedMsg<nav_msgs::OdometryConstPtr>(coorAtCamPtr);
    
    if(iter != nav_msgs::OdometryConstPtr()) {
        
        ::qu = Eigen::Quaternion<double>(iter->pose.pose.orientation.w, 
                                       iter->pose.pose.orientation.x, 
                                       iter->pose.pose.orientation.y, 
                                       iter->pose.pose.orientation.z);

        ::t = Eigen::Vector3d(iter->pose.pose.position.x, 
                              iter->pose.pose.position.y, 
                              iter->pose.pose.position.z);

        std::cout << "无人机坐标为：" << "(" 
                  << t[0] << ", " 
                  << t[1] << ", " 
                  << t[2] << ") " 
                  << std::endl;

        std::cout << "四元数：" 
                  << qu.w() << " + "
                  << qu.x() << "i + "
                  << qu.y() << "j + "
                  << qu.z() << "k "
                  << std::endl;

        std::cout << "在相机坐标系下坐标为：(" 
            << coorAtCamPtr->point.x << ", "
            << coorAtCamPtr->point.y << ", "
            << coorAtCamPtr->point.z << ") "
            << std::endl;

        NRC::Cam cam1;
        geometry_msgs::PointStamped AircaftCoor = cam1.cam2UAV(coorAtCamPtr);

        std::cout << "在机体坐标系下坐标为：(" 
            << AircaftCoor.point.x << ", "
            << AircaftCoor.point.y << ", "
            << AircaftCoor.point.z << ") "
            << std::endl;

        Eigen::Vector3d coorAtCam = { AircaftCoor.point.x, AircaftCoor.point.y, AircaftCoor.point.z };

        Eigen::Vector3d coorAtWorld = qu * coorAtCam + t;

        geometry_msgs::PointStamped coor;
        coor.point.x = coorAtWorld[0];
        coor.point.y = coorAtWorld[1];
        coor.point.z = coorAtWorld[2];

        std::cout << "惯性坐标系下坐标为：" << "(" 
                << coorAtWorld[0] << ", " 
                << coorAtWorld[1] << ", " 
                << coorAtWorld[2] << ") " << std::endl;

        std::cout << "---------------------------------------" << std::endl;
        //std::system("clear");

        coor_pub.publish(coor);
    }
}
