#include <ros/ros.h>
#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gesture_recognition");
    ros::NodeHandle _nh("gesture_recognition");

    bool use_robot;

    _nh.param<bool>("use_robot", use_robot, true);

    printf("\n");
    ROS_INFO("use_robot flag set to %s", use_robot==true?"true":"false");

    GestureRec gesture_recognition("gesture_recognition", "left");

    ros::spin();
    return 0;
}
