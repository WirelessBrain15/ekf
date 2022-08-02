#pragma once
// Header files
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>

ros::Time timePrev;

class testEkf
{
    protected:
        // Variables
        ros::NodeHandle nh;
        // Functions
        void listener();

    public:
        // Variables
        // double prevVel = 0;

        // double newReading[3];
        // double newCovariance[9];

        Eigen::Matrix2d odomCovarianceMatrix;
        Eigen::Matrix2d imuCovarianceMatrix;

        double odomReadings[2]; // linear_vel.x, linear_vel.y, ang_vel.z
        double imuReadings[2]; // linear_acc.x, linear_acc.y, ang_vel.z
        // Constructor
        testEkf();
        // Functions
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        double predictVel();
        void correctVel();
        double transformYaw(geometry_msgs::PoseStamped pose);
        void accToVel(double dt);
};