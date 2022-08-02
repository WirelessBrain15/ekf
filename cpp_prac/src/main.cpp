#include "ekf_test.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_test");
    // ros::NodeHandle nh;

    testEkf ekf;

    cout << "test_ekf" << endl;

    return 0;
}