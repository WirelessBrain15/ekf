#include "ekf_2d_test.hpp"

using namespace std;

Eigen::VectorXd odomReadingsMatrix(2);
Eigen::VectorXd imuReadingsMatrix(2);

Eigen::VectorXd estValMatrix(2);
Eigen::Matrix2d estCovarianceMatrix;

double prevVel = 0;
bool flag = false;


testEkf::testEkf()
{
    cout << "[CONSTRUCTOR]" << endl;
    estCovarianceMatrix << 0.01,0,
    0,0.01;
    estValMatrix << 0,0;
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

    odomReadingsMatrix << temp.twist.twist.linear.x,
    yaw_angle;

    // cout << "odom Yaw : " << yaw_angle*180*7/22 << endl;

    // odomCovarianceMatrix << temp.twist.covariance[0],0,0,
    // 0,temp.twist.covariance[7],0,
    // 0,0,temp.pose.covariance[35];

    odomCovarianceMatrix << 0.05,0,
    0,0.05;

    // cout << odomReadingsMatrix << endl;
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

    if(flag == true)
    {
        yaw_angle = transformYaw(_temp);
    }

    else
    {
        yaw_angle = tf::getYaw(pose.getRotation());
    }

    double newYaw = yaw_angle + 22.0/14.0;
    if(yaw_angle > 22.0/14)
    {
        if (yaw_angle > 0)
        {
            newYaw = -44.0/7 + newYaw;
        }
    }
    
    imuReadingsMatrix << temp.linear_acceleration.z,
    newYaw;

    // cout << "IMU Yaw : " << yaw_angle << endl;
    cout << "new Yaw : " << newYaw*180*7.0/22 << endl;
    // cout << "diff : " << newYaw - yaw_angle << endl;

    // imuCovarianceMatrix << temp.linear_acceleration_covariance[0],0,0,
    // 0,temp.linear_acceleration_covariance[4],0,
    // 0,0,temp.orientation_covariance[8];

    imuCovarianceMatrix << 0.01,0,
    0,0.001;

    double dt = testEkf::predictVel();
    testEkf::correctVel();

    testEkf::accToVel(dt);

    cout << "Est Val : " << estValMatrix(1)*180*7.0/22 << endl;
    // cout << "Est Cov : " << estCovarianceMatrix << endl;
    // cout << "IMU : " <<  dt*imuReadingsMatrix(0) << endl;
    cout << "odom : " << odomReadingsMatrix(1)*180*7.0/22 << endl;
}

double testEkf::transformYaw(geometry_msgs::PoseStamped pose)
{ 
    cout << "banana" << endl;
    double roll,pitch,yaw;
    pose.header.frame_id = "IMU_link";
    tf::TransformListener listener;
    geometry_msgs::PoseStamped _pose;
    _pose.header.frame_id = "base_link";
    try
    {
        listener.waitForTransform("base_link","IMU_link", ros::Time::now(), ros::Duration(3.0));
        listener.transformPose("base_link", pose, _pose);

    }
    catch(tf::TransformException& ex)
    {
        cout << "ERROR lol" << endl;
    }
    // listener.transformPose("base_link", pose, pose);

    tf::Quaternion q(_pose.pose.orientation.x,_pose.pose.orientation.y,
    _pose.pose.orientation.z,
    _pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    return yaw;
}

// Predict 
double testEkf::predictVel()
{
    // cout << "[PREDICT LOOP]" << endl;
    Eigen::Matrix2d A;
    Eigen::Matrix2d B;
    ros::Time timeNow = ros::Time::now();
    double dt = double((timeNow - timePrev).toSec());
    A << -1/dt, 0,
    0, 1;
    B << 1/dt, 0,
    0, 1;
    estValMatrix = A*estValMatrix + B*odomReadingsMatrix;
    estCovarianceMatrix = A*estCovarianceMatrix*A.inverse() + odomCovarianceMatrix;
    timePrev = timeNow;
    return dt;
}

// Correct 
void testEkf::correctVel()
{   
    Eigen::Matrix2d C = Eigen::MatrixXd::Identity(2,2);
    Eigen::Matrix2d temp = (C*estCovarianceMatrix*C.inverse() + imuCovarianceMatrix);
    Eigen::Matrix2d kalmanGain = C*estCovarianceMatrix*temp.inverse();
    Eigen::VectorXd z = imuReadingsMatrix - estValMatrix;
    // Eigen::VectorXd z = estValMatrix - odomReadingsMatrix;
    estValMatrix = (estValMatrix + kalmanGain*z);
    // estValMatrix = (odomReadingsMatrix + kalmanGain*z);
    // cout << "Kalman gain : " << kalmanGain << endl;
    estCovarianceMatrix = (Eigen::MatrixXd::Identity(2,2) - kalmanGain)*estCovarianceMatrix;
}

void testEkf::accToVel(double dt)
{
    prevVel = prevVel + estValMatrix(0)*dt;
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