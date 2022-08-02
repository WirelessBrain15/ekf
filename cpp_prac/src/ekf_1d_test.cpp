#include "ekf_1d_test.hpp"

using namespace std;


testEkf::testEkf()
{
    cout << "[CONSTRUCTOR]" << endl;
    listener();
}

void testEkf::listener()
{
    cout << "[LISTENER]" << endl;

    timePrev = ros::Time::now();
    
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &testEkf::odomCallback, this);
    ros::Subscriber imu_sub = nh.subscribe("/imu_data", 1, &testEkf::imuCallback, this);
    ros::spin();
}

void testEkf::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry temp;
    // cout << "[CALLBACK - ODOM]" << endl;
    temp = *msg;

    tf::Pose pose;
    tf::poseMsgToTF(temp.pose.pose, pose);
    double yaw_angle = tf::getYaw(pose.getRotation());

    odomReading = yaw_angle*180*7/22;
    // odomCovariance = temp.twist.covariance[35];
    odomCovariance = 0.02;
    // odomReadingsMatrix << temp.twist.twist.angular.z;

    // odomCovarianceMatrix << temp.twist.covariance[35];
}

void testEkf::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu temp;
    // cout << "[CALLBACK - IMU]" << endl;
    temp = *msg;

    double yaw_angle = 0;
    geometry_msgs::PoseStamped _temp;
    tf::Pose pose;
    _temp.pose.orientation = temp.orientation;
    _temp.header.frame_id = temp.header.frame_id;
    tf::poseMsgToTF(_temp.pose, pose);

    yaw_angle = tf::getYaw(pose.getRotation());

    double newYaw = yaw_angle + 22.0/14.0;
    if(yaw_angle > 22.0/14)
    {
        if (yaw_angle > 0)
        {
            newYaw = -44.0/7 + newYaw;
        }
        // newYaw = -44.0/7 + yaw_angle;
    }

    imuReading = newYaw*180*7/22;
    // imuCovariance = temp.angular_velocity_covariance[8];
    imuCovariance = 0.01;
    // imuReadingsMatrix << temp.angular_velocity.z;
    
    // imuCovarianceMatrix << temp.angular_velocity_covariance[8];

    testEkf::dataFusion(odomReading, odomCovariance, imuReading, imuCovariance);

    // meanVal, meanVar = testEkf::predict(meanVal, meanVar);

    testEkf::update(meanVal, meanVar, noiseVal);

    // cout << "imu : " << imuReading << endl;
    // cout << "odom : " << odomReading << endl;
    cout << "fused Mean : " << meanVal << endl;
    cout << "fused Var : " << meanVar << endl;
    cout << "Est Mean : " << estMean << endl;
    cout << "Est Var : " << estVar << endl;
    // cout << noiseVal << endl;
}

double testEkf::dataFusion(double val1, double var1, double val2, double var2)
{
    meanVal = (val1*var2 + val2*var1)/(var1 + var2);
    meanVar = 1/((1/var1) + (1/var2));
    noiseVal = 0.0001;
    // cout << newVal << endl;
    // cout << newVar << endl;
    // return newVal, newVar;
}

// Predict 
double testEkf::predict(double val, double var)
{
    // cout << "[PREDICT LOOP]" << endl;
    double newMean = val;
    double newVar = var + noiseVar;

    return newMean,newVar;
}

// Correct 
double testEkf::update(double fusedMean, double fusedVar, double noiseVal)
{
    // cout << "[CORRECTION LOOP]" << endl;

    double kalmanGain = meanVar/(meanVar + fusedVar);
    double z = fusedMean - estMean;
    estMean = estMean + kalmanGain * z;
    estVar = (1 - kalmanGain) * estVar;

    // return finalMean, finalVar;
}

// Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_test");
    
    // ros::NodeHandle nh;

    testEkf ekf;

    cout << "test_ekf" << endl;

    return 0;
}