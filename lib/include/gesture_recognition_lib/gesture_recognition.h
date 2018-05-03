#ifndef __GESTURE_REC__
#define __GESTURE_REC__

#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_msgs/MarkerArray.h>
#include <gesture_recognition/DoAction.h>
#include <gesture_recognition/GestureState.h>
#include <gesture_recognition/RecState.h>
#include <robot_perception/perception_client_impl.h>
#include <phasespace_publisher/PhasespacePt.h>
#include <phasespace_publisher/PhasespacePtArray.h>
#include <human_robot_collaboration_msgs/DoAction.h>
#include <baxter_core_msgs/DigitalIOState.h>


#include <GRT/GRT.h>

// allowed default states for the system
// #define KILLED         -2
// #define ERROR          -1
#define RECORD          1
// #define TRAIN           2
// #define TEST            3
// #define PUBLISH         4
#define RECORDING       5
#define DONE_RECORDING  6


class GestureRec : public PerceptionClientImpl
{
private:

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner; // AsyncSpinner to handle callbacks

    std::string limb; // Limb to use for ARuco detection

    bool predict_and_add; // setting for predictOnce to add sample to trainingData or not

    // class label to record
    int class_label;

    // internal thread functionality
    std::thread gesture_thread;

    // service for gesture recognition actions
    ros::ServiceServer service;

    // publisher for gestures recognized in real time
    ros::Publisher gesture_pub;

    // subscriber for gestures recognized in real time
    ros::Subscriber gesture_sub;

    // publisher for gesture recognition system state
    ros::Publisher state_pub;
    ros::Subscriber state_sub;

    // training data to be used. add samples using recordSample
    GRT::TimeSeriesClassificationData trainingData;

    // pipeline to be used for gesture recognition
    GRT::GestureRecognitionPipeline pipeline;

    // Bool to check if a gesture has been detected
    bool gesture_found;

    // Predicted Class ID of the gesture found
    int gesture_id;

    // gesture recognition state
    State state;

    // current action of the gesture rec system
    std::string action;

    // flag to recognize gestures in real time
    bool publish;

    ros::Timer rec_timer;
    ros::Timer thread_timer;
    double rec_window;

    GRT::DTW dtw;

    // DISPLAY
    image_transport::ImageTransport it;
    image_transport::Publisher im_pub;

    gesture_recognition::RecState rec_state;

    std::string display_text;
    ros::Timer  text_timer;
    double      text_duration;

    int im_h;
    int im_w;
    int im_w_delim;

    cv::Scalar red;
    cv::Scalar green;
    cv::Scalar yellow;
    cv::Scalar blue;
    cv::Scalar black;

    // flag to use phasespace or not
    bool use_phasespace;

    // phasespace subscriber
    ros::Subscriber ps_sub;

    // phasespace point array for right hand (0-7)
    phasespace_publisher::PhasespacePtArray rh_markers;

    // phasespace point array for left hand (8-15)
    phasespace_publisher::PhasespacePtArray lh_markers;

    // action provider service clients
    bool robot_demo;
    ros::ServiceClient right_client;
    ros::ServiceClient left_client;

    ros::Publisher right_cuff_pub;
    ros::Publisher right_up_cuff_pub;
    ros::Publisher left_cuff_pub;


protected:

    int getClassDistance(int class_name);

    void displayText(cv::Mat& in);

    void callAction(int predicted_class);

    void displayRecState();

    /**
     * @brief initialize training data dimensions, name, and info
     * @return true/false if success/failure
     */
    bool setUpTrainingData()
    {
        if (!use_phasespace)
        {
            trainingData.setNumDimensions(3);
            trainingData.setDatasetName("TrainingData");
            trainingData.setInfoText("A training data set for the gesture recognition system.");
        }
        else
        {
            trainingData.setNumDimensions(3);
            trainingData.setDatasetName("PhasespaceTrainingData");
            trainingData.setInfoText("A training dataset for the gesture recognition system using Phasespace marker data.");
        }

        return true;
    }

    bool initMarkerArrays()
    {
        for (int i = 0; i < 8; i++)
        {
            phasespace_publisher::PhasespacePt pt;
            pt.id = i;
            pt.pt.x = 0.0;
            pt.pt.y = 0.0;
            pt.pt.z = 0.0;
            rh_markers.points.push_back(pt);
        }
        for (int i = 8; i < 16; i++)
        {
            phasespace_publisher::PhasespacePt pt;
            pt.id = i;
            pt.pt.x = 0.0;
            pt.pt.y = 0.0;
            pt.pt.z = 0.0;
            lh_markers.points.push_back(pt);
        }
        return true;
    }

    /**
     * @brief initialize pipeline features, such as classifier and preprocessing modules
     * @return true/false if success/failure
     */
    bool setUpPipeline();


    bool publishState();

    bool setState(int _state);

    bool doAction(std::string action, std::string filename);

    bool setAction(const std::string& _action);


   GRT::MatrixFloat preProcess(GRT::MatrixFloat rawGesture);

     /**
     * @brief records 1 second of ARuco data and adds it to trainingData with the appropriate label
     * @return true/false if success/failure
     */
    bool recordSample(GRT::TimeSeriesClassificationData &trainingData, GRT::UINT gestureLabel, int object_id, std::string filename);


    /**
     * @brief trains the pipeline with the pipelineData given
     * @return true/false if success/failure
     */
    bool trainPipeline(GRT::GestureRecognitionPipeline &pipeline, GRT::TimeSeriesClassificationData pipelineData);

    /**
     * @brief generates dummy data, trains the pipeline, and tests the pipeline
     * @return true/false if success/failure
     */
    bool testPipeline(GRT::GestureRecognitionPipeline &pipeline);


    /**
     * @brief Callback function for gesture recognition actions
     * @return true/false if success/failure
     */
    bool actionCb(gesture_recognition::DoAction::Request &req,
                  gesture_recognition::DoAction::Response &res);

    /**
     * @brief Callback function for the gesture recognition topic
     */
    void gestureRecCb(const gesture_recognition::RecState &msg);

    void gestureStateCb(const gesture_recognition::GestureState& msg);

    void beginPublishThread();

    void beginRecording(GRT::MatrixFloat*);

    void predictPublishCb(GRT::MatrixFloat);

    // void psMarkersCb(const phasespace_publisher::PhasespacePtArray& markers);

    void PsObjectCb(const phasespace_publisher::PhasespacePtArray& markers);


    /**
     * @brief will record 1s of ARuco data and use the pipeline to predict the class of the gesture
     * @return true/false if success/failure
     */
    bool predictOnce(GRT::GestureRecognitionPipeline &pipeline, GRT::UINT gestureLabel, int object_id, std::string filename);

    State getState() {return state; };
    std::string getAction() {return action; };
    bool getPredictAdd() {return predict_and_add; };
    void setPredictAdd(bool predict);
    void setDisplayText(const std::string &s);
    void deleteDisplayText();


public:
    GestureRec(std::string name, std::string limb);
    ~GestureRec();

};

#endif
