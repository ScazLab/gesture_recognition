#include <ros/ros.h>
#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gesture_recognition");
    ros::NodeHandle _nh("gesture_recognition");

    GestureRec gesture_recognition("gesture_recognition", "left");

    ros::spin();
    return 0;
}
