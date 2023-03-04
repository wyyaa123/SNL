#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "../../../include/SNL.hpp"

#include <deque>
#include <iostream>
#include <algorithm>

#define THRESHOLD 0.2

void initQuaternions(const nav_msgs::OdometryConstPtr& aircraft);
void Alter2Cam(const geometry_msgs::PointStampedConstPtr& coorAtCamPtr);
//void Alter2World(const nav_msgs::OdometryConstPtr& qu, const geometry_msgs::PointStampedConstPtr& coorAtCamPtr);

//std::deque<geometry_msgs::PointStampedConstPtr> coorAtCam;
std::deque<nav_msgs::OdometryConstPtr> kf;

Eigen::Vector3d t;
Eigen::Quaternion<double> qu;
ros::Publisher coor_pub;

int main(int argc, char** argv) {

    ros::init(argc, argv, "cam2world");

    SNL::Comunicate com(0.02);

    while(ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}