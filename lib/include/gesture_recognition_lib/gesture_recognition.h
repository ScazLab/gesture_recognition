#ifndef __GESTURE_REC__
#define __GESTURE_REC__

#include <ros/ros.h>
#include <ros/console.h>
#include <aruco_msgs/MarkerArray.h>
#include <gesture_recognition/DoAction.h>
#include <gesture_recognition/GestureState.h>

#include <GRT/GRT.h>

class GestureRec
{
private:

    ros::NodeHandle _nh;

    std::string _limb; // Limb to use for ARuco detection

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

    // service for gesture recognition actions
    ros::ServiceServer service;

    // publisher for gestures recognized in real time
    ros::Publisher gesture_pub;

    // subscriber for gestures recognized in real time
    ros::Subscriber gesture_sub;

    // training data to be used. add samples using recordSample
    GRT::TimeSeriesClassificationDataStream trainingData;

    // pipeline to be used for gesture recognition
    GRT::GestureRecognitionPipeline pipeline;

    // Bool to check if a gesture has been detected
    bool gesture_found;

    // Predicted Class ID of the gesture found
    int gesture_id;


protected:


    /**
     * @brief initialize training data dimensions, name, and info
     * @return true/false if success/failure
     */
    bool setUpTrainingData()
    {
        trainingData.setNumDimensions(3);
        trainingData.setDatasetName("TrainingData");
        trainingData.setInfoText("A training data set for the gesture recognition system.");

        return true;
    }

    /**
     * @brief initialize pipeline features, such as classifier and preprocessing modules
     * @return true/false if success/failure
     */
    bool setUpPipeline()
    {
        // combining MovementTrajectoryFeatures and ANBC is probably the way to go in the long run
        // pipeline.addFeatureExtractionModule( GRT::MovementTrajectoryFeatures(100, 10, 1, 10, 3, false, true) );
        // pipeline.setClassifier( GRT::ANBC() );

        // generally successful classifiers: AdaBoost, GMM for now
        pipeline.setClassifier( GRT::AdaBoost() );
        return true;
    }

     /**
     * @brief records 1 second of ARuco data and adds it to trainingData with the appropriate label
     * @return true/false if success/failure
     */
    bool recordSample(GRT::TimeSeriesClassificationDataStream &trainingData, GRT::UINT gestureLabel, int marker_id, std::string filename);


    /**
     * @brief trains the pipeline with the pipelineData given
     * @return true/false if success/failure
     */
    bool trainPipeline(GRT::GestureRecognitionPipeline &pipeline, GRT::TimeSeriesClassificationDataStream pipelineData);

    /**
     * @brief generates dummy data, trains the pipeline, and tests the pipeline
     * @return true/false if success/failure
     */
    bool testPipeline(GRT::GestureRecognitionPipeline &pipeline);

    /**
     * Callback function for the ARuco topic
     * @param msg the topic message
     */
    void ARucoCb(const aruco_msgs::MarkerArray& msg);

    /**
     * @brief Callback function for gesture recognition actions
     * @return true/false if success/failure
     */
    bool actionCb(gesture_recognition::DoAction::Request &req,
                  gesture_recognition::DoAction::Response &res);

    /**
     * @brief Callback function for the gesture recognition topic
     */
    void gestureCb(const gesture_recognition::GestureState &msg);

    /**
     * @brief publishes status of gesture recognition in real time
     */
    void publishGestures();

    /**
     * @brief will record 1s of ARuco data and use the pipeline to predict the class of the gesture
     * @return true/false if success/failure
     */
    bool predictOnce(GRT::GestureRecognitionPipeline &pipeline, int marker_id);

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



public:
    GestureRec(std::string name, std::string limb, bool _no_robot);
    ~GestureRec();

};

#endif
