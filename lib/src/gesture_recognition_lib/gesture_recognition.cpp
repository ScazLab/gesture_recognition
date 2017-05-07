#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb, bool _no_robot) : _nh(name), _limb(limb),
                                      aruco_ok(false), markers_found(false),
                                         marker_found(false), marker_id(-1)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               3, &GestureRec::ARucoCb, this);

    std::string topic = "/gesture_recognition/record_sample";
    service = _nh.advertiseService(topic, &GestureRec::recordCb, this);
}

bool GestureRec::recordCb(gesture_recognition::RecordSample::Request  &req,
                          gesture_recognition::RecordSample::Response &res)
{

    GRT::ClassificationDataStream trainingData;
    trainingData.setNumDimensions(3);
    GRT::UINT gestureLabel = req.label;

    setMarkerID(19);

    res.success = false;
    ROS_INFO("Ready to record a gesture!");
    ROS_INFO("Recording in 5 seconds...");
    ros::Duration(1.0).sleep();
    ROS_INFO("Recording in 4 seconds...");
    ros::Duration(1.0).sleep();
    ROS_INFO("Recording in 3 seconds...");
    ros::Duration(1.0).sleep();
    ROS_INFO("Recording in 2 seconds...");
    ros::Duration(1.0).sleep();
    ROS_INFO("Recording in 1 second...");
    ros::Duration(1.0).sleep();

    trainingData = recordSample(trainingData, gestureLabel);

    trainingData.printStats();
    res.response = "Recorded Sample";
    res.success = true;
    return true;
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

GRT::ClassificationDataStream GestureRec::recordSample(GRT::ClassificationDataStream trainingData, GRT::UINT gestureLabel)
{

    GRT::MatrixFloat gesture;
    GRT::Vector< GRT:: VectorFloat > gestureVec;
    ros::Time time_start = ros::Time::now();

    while( ros::Time::now().toSec() - time_start.toSec() < 1)
    {
        GRT::VectorFloat sample(3);
        sample[0] = curr_marker_pos.x;
        sample[1] = curr_marker_pos.y;
        sample[2] = curr_marker_pos.z;

        // sample[3] = curr_marker_ori.x;
        // sample[4] = curr_marker_ori.y;
        // sample[5] = curr_marker_ori.z;
        // sample[6] = curr_marker_ori.w;

        gestureVec.push_back(sample);
    }

    gesture = gestureVec;

    trainingData.addSample(gestureLabel, gesture);

    ROS_INFO("training data should have 1 sample");
    // trainingData.save("TrainingData.csv");

    return trainingData;
}

bool GestureRec::recTrainingData()
{
    //Create a new instance of the ClassificationData
    // GRT::ClassificationData trainingData;
    // GRT::UINT gestureLabel = 1;

    // if (!recordSample(trainingData, gestureLabel)) return false;

    // bool saveResult = trainingData.save( "TrainingData.grt" );

    return true;

}

GestureRec::~GestureRec()
{

}
