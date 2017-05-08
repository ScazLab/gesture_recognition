#ifndef __GESTURE_REC__
#define __GESTURE_REC__

#include <ros/ros.h>
#include <ros/console.h>
#include <aruco_msgs/MarkerArray.h>
#include <gesture_recognition/RecordSample.h>
#include <gesture_recognition/TrainPipeline.h>

#include <GRT/GRT.h>

class GestureRec
{
private:

    ros::NodeHandle _nh;

    std::string _limb; // Limb of the gripper: left or right

    // Subscriber to the ARuco detector,
    ros::Subscriber _aruco_sub;

    // List of available markers
    std::vector<int> available_markers;

        // Bool to check if ARuco is fine or not
    bool              aruco_ok;

    // Bool to check if there are any markers detected
    bool         markers_found;

    // Bool to check if the selected marker was found or not
    bool          marker_found;

    // ID of the marker to detect
    int              marker_id;
    // Marker position and orientation
    geometry_msgs::Point        curr_marker_pos;
    geometry_msgs::Quaternion   curr_marker_ori;

    ros::ServiceServer service;

    GRT::ClassificationDataStream trainingData;


protected:

    bool setUpTrainingData()
    {
        trainingData.setNumDimensions(3);
        trainingData.setDatasetName("TrainingData");
        trainingData.setInfoText("A training data set for the gesture recognition system.");

        return true;
    }

    /**
     * Callback function for the ARuco topic
     * @param msg the topic message
     */
    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    bool recordCb(gesture_recognition::RecordSample::Request  &req,
                  gesture_recognition::RecordSample::Response &res);

    bool trainCb(gesture_recognition::TrainPipeline::Request &req,
                 gesture_recognition::TrainPipeline::Response &res);

    /*
     * Check availability of the ARuco data
     * @return true/false if feedback from ARuco is received
    */
    bool isARucoOK() { return aruco_ok; };


        /* SETTERS */
    void setMarkerID(int _id)                { marker_id = _id; };

    /* GETTERS */
    geometry_msgs::Point      getMarkerPos() { return curr_marker_pos; };
    geometry_msgs::Quaternion getMarkerOri() { return curr_marker_ori; };

    int         getMarkerID()  { return marker_id; };

    /**
     * Returns a list of available markers
     * @return a list of available markers
     */
    std::vector<int> getAvailableMarkers() { return available_markers; };

    /**
     * Looks if a set of markers is present among those available.
     * @return the subset of available markers among those available
     */
    std::vector<int> getAvailableMarkers(std::vector<int> _markers);

    /**
     * @brief records 1 second of ARuco data
     * @return true/false if success/failure
     */
    GRT::ClassificationDataStream recordSample(GRT::ClassificationDataStream trainingData, GRT::UINT gestureLabel);

    /**
     * @brief record & save sample data
     * @return true/false if success/failure
     */
    bool recTrainingData();



public:
    GestureRec(std::string name, std::string limb, bool _no_robot);
    ~GestureRec();

};

#endif
