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
        double imuCovariance = 0;
        double odomCovariance = 0;

        double estMean;
        double estVar;
        double meanVal;
        double meanVar;
        double odomReading;
        double imuReading;
        double noiseVal;
        double noiseVar;

        // Constructor
        testEkf();
        // Functions
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        double predict(double val, double var);
        double update(double fusedMean, double fusedVar, double noiseVal);
        double dataFusion(double val1, double var1, double val2, double var2);
};