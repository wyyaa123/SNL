#ifndef _P2C_HPP
#define _P2C_HPP 1
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>

namespace NRC {

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
}

#endif
