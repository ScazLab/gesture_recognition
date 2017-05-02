#include "gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb) : _nh(name), _limb(limb),
                                      aruco_ok(false), markers_found(false),
                                         marker_found(false), marker_id(-1)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               3, &GestureRec::ARucoCb, this);
}

void GestureRec::ARucoCb(const aruco_msgs::MarkerArray& msg)
{
    if (msg.markers.size() > 0)
    {
        available_markers.clear();
    }

    for (size_t i = 0; i < msg.markers.size(); ++i)
    {
        // ROS_DEBUG("Processing marker with id %i",msg.markers[i].id);

        available_markers.push_back(int(msg.markers[i].id));
        markers_found = true;

        if (int(msg.markers[i].id) == getMarkerID())
        {
            curr_marker_pos = msg.markers[i].pose.pose.position;
            curr_marker_ori = msg.markers[i].pose.pose.orientation;


            ROS_DEBUG("Marker is in: %g %g %g", curr_marker_pos.x,
                                                curr_marker_pos.y,
                                                curr_marker_pos.z);
            // ROS_INFO("Marker is in: %g %g %g %g", curr_marker_ori.x,
            //                                       curr_marker_ori.y,
            //                                       curr_marker_ori.z,
            //                                       curr_marker_ori.w);

            if (!marker_found)
            {
                marker_found = true;
            }
        }
    }

    if (!aruco_ok)
    {
        aruco_ok = true;
    }
}

bool GestureRec::recordSample(GRT::ClassificationData trainingData, GRT::UINT gestureLabel)
{
    vector<vector<vector<double>>> gesture;
    ros::Time time_start = ros::Time::now();

    while( ros::Time::now().toSec() - time_start.toSec() < 1)
    {
        vector<vector<double>> sample(2);

        vector<double> position(3);
        position[0] = curr_marker_pos.x;
        position[1] = curr_marker_pos.y;
        position[2] = curr_marker_pos.z;

        vector<double> orientation(4);
        orientation[0] = curr_marker_ori.x;
        orientation[1] = curr_marker_ori.y;
        orientation[2] = curr_marker_ori.z;
        orientation[3] = curr_marker_ori.w;

        sample[0] = position;
        sample[1] = orientation;

        gesture.push_back(sample);
    }

    // trainingData.addSample( gestureLabel, gesture);

    return true;
}

bool GestureRec::recTrainingData()
{
    //Create a new instance of the ClassificationData
    GRT::ClassificationData trainingData;
    GRT::UINT gestureLabel = 1;

    if (!recordSample(trainingData, gestureLabel)) return false;

    // bool saveResult = trainingData.save( "TrainingData.grt" );

    return true;

}

GestureRec::~GestureRec()
{

}
