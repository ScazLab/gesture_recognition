#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb, bool _no_robot) : _nh(name), _limb(limb),
                                      aruco_ok(false), markers_found(false),
                                         marker_found(false), marker_id(-1)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               3, &GestureRec::ARucoCb, this);

    std::string topic = "/gesture_recognition/action_provider";
    record_service = _nh.advertiseService(topic, &GestureRec::actionCb, this);

    topic = "/gesture_recognition/train_pipeline";
    train_service = _nh.advertiseService(topic, &GestureRec::trainCb, this);

    setUpTrainingData();
    setUpPipeline();

    // string datasetName = trainingData.getDatasetName();
    // string infoText = trainingData.getInfoText();
    // GRT::UINT numSamples = trainingData.getNumSamples();
    // GRT::UINT numDimensions = trainingData.getNumDimensions();
    // GRT::UINT numClasses = trainingData.getNumClasses();

    // cout << "Dataset Name: " << datasetName << endl;
    // cout << "InfoText: " << infoText << endl;
    // cout << "NumberOfSamples: " << numSamples << endl;
    // cout << "NumberOfDimensions: " << numDimensions << endl;
    // cout << "NumberOfClasses: " << numClasses << endl;

    // //You can also get the minimum and maximum ranges of the data
    // GRT::Vector< GRT::MinMax > ranges = trainingData.getRanges();

    // cout << "The ranges of the dataset are: \n";
    // for(GRT::UINT j=0; j<ranges.size(); j++){
    //     cout << "Dimension: " << j << " Min: " << ranges[j].minValue << " Max: " << ranges[j].maxValue << endl;
    // }

}

bool GestureRec::actionCb(gesture_recognition::DoAction::Request &req,
                          gesture_recognition::DoAction::Response &res)
{
    std::string action = req.action;

    if (action == "record")
    {
        if (!recordCb(req, res)) return false;
        return true;
    }

    if (action == "train")
    {
        if (!trainCb(req, res)) return false;
        return true;
    }

    if (action == "test")
    {
        if(!testCb(req, res)) return false;
        return true;
    }

    return true;
}

bool GestureRec::trainCb(gesture_recognition::DoAction::Request &req,
                         gesture_recognition::DoAction::Response &res)
{
    std::string filename = req.filename;
    GRT::TimeSeriesClassificationDataStream pipelineData;

    if( !pipelineData.load(filename) ){
        ROS_ERROR("ERROR: Failed to load training data from file\n");
        return false;
    }


    // if ( !trainPipeline(pipeline, pipelineData)) return false;

    // //Test the pipeline using the test data
    // if( !pipeline.test( pipelineData ) ){
    //     ROS_ERROR("ERROR: Failed to test the pipeline!\n");
    //     return false;
    // }

    //  //Print some stats about the testing
    // cout << "Test Accuracy: " << pipeline.getTestAccuracy() << endl;

    // cout << "Precision: ";
    // for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
    //     GRT::UINT classLabel = pipeline.getClassLabels()[k];
    //     cout << "\t" << pipeline.getTestPrecision(classLabel);
    // }cout << endl;

    // cout << "Recall: ";
    // for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
    //     GRT::UINT classLabel = pipeline.getClassLabels()[k];
    //     cout << "\t" << pipeline.getTestRecall(classLabel);
    // }cout << endl;

    // cout << "FMeasure: ";
    // for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
    //     GRT::UINT classLabel = pipeline.getClassLabels()[k];
    //     cout << "\t" << pipeline.getTestFMeasure(classLabel);
    // }cout << endl;

    // GRT::MatrixDouble confusionMatrix = pipeline.getTestConfusionMatrix();
    // cout << "ConfusionMatrix: \n";
    // for(GRT::UINT i=0; i<confusionMatrix.getNumRows(); i++){
    //     for(GRT::UINT j=0; j<confusionMatrix.getNumCols(); j++){
    //         cout << confusionMatrix[i][j] << "\t";
    //     }cout << endl;
    // }


    res.success = true;
    res.response = "trained and tested data";

    return true;
}

bool GestureRec::testCb(gesture_recognition::DoAction::Request &rec,
                        gesture_recognition::DoAction::Response &res)
{
    // if (!testPipeline(pipeline)) return false;

    res.success = true;
    res.response = "Tested Pipeline";
    return true;
}


bool GestureRec::recordCb(gesture_recognition::DoAction::Request  &req,
                          gesture_recognition::DoAction::Response &res)
{

    GRT::UINT gestureLabel = req.class_label;

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
    ROS_INFO("Recording!");

    trainingData = recordSample(trainingData, gestureLabel);

    trainingData.printStats();


    if( !trainingData.save( "/home/baxter/ros_devel_ws/src/gesture_recognition/data/TrainingData.csv" ) ){
        cout << "ERROR: Failed to save dataset to file!\n";
        return false;
    }


    res.response = "Recorded sample with class label " + std::to_string(req.class_label);
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

GRT::TimeSeriesClassificationDataStream GestureRec::recordSample(GRT::TimeSeriesClassificationDataStream trainingData, GRT::UINT gestureLabel)
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

    return trainingData;
}

bool trainPipeline(GRT::GestureRecognitionPipeline pipeline, GRT::TimeSeriesClassificationDataStream pipelineData)
{
    if( !pipeline.train( pipelineData) )
    {
        ROS_ERROR("ERROR: Failed to train the pipeline!");
        return false;
    }

    return true;
}

bool testPipeline(GRT::GestureRecognitionPipeline pipeline)
{
    // Generate some data
    GRT::TimeSeriesClassificationDataStream testData;
    testData.setNumDimensions(3);
    testData.setDatasetName("TestingData");

    GRT::TimeSeriesClassificationDataStream testingSample;
    testingSample.setNumDimensions(3);
    testingSample.setDatasetName("SampleToTest");

    GRT::UINT gestureLabel = 1;

    GRT::Random random;
    for(GRT::UINT k=0; k<3; k++){//For the number of classes
        gestureLabel = k+1;

        //Get the init random walk position for this gesture
        GRT::VectorDouble startPos( testData.getNumDimensions() );
        for(GRT::UINT j=0; j<startPos.size(); j++){
            startPos[j] = random.getRandomNumberUniform(-1.0,1.0);
        }

        //Generate the 20 time series
        for(GRT::UINT x=0; x<20; x++){
            //Generate the random walk
            GRT::UINT randomWalkLength = random.getRandomNumberInt(90, 110);
            GRT::VectorDouble sample = startPos;
            for(GRT::UINT i=0; i<randomWalkLength; i++){
                for(GRT::UINT j=0; j<sample.size(); j++){
                    sample[j] += random.getRandomNumberUniform(-0.1,0.1);
                }

                //Add the training sample to the dataset
                testData.addSample(gestureLabel, sample );
                testingSample.addSample(gestureLabel, sample);
            }

            //now add some noise to represent a null class
            for(GRT::UINT i=0; i<50; i++){
                for(GRT::UINT j=0; j<sample.size(); j++){
                    sample[j] = random.getRandomNumberUniform(-0.01,0.01);
                }

                //Add the training sample to the dataset, note that we set the gesture label to 0
                testData.addSample(0, sample );
                testingSample.addSample(0, sample);
            }
        }
    }


    //Train the pipeline using the training data
    if( !pipeline.train( testData ) ){
        ROS_ERROR("Failed to train the pipeline!");
        return false;
    }

    //Test the pipeline using the test data
    if( !pipeline.test( testingSample ) ){
        ROS_ERROR("Failed to test the pipeline!");
        return false;
    }

    //Print some stats about the testing
    cout << "Test Accuracy: " << pipeline.getTestAccuracy() << endl;

    cout << "Precision: ";
    for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
        GRT::UINT classLabel = pipeline.getClassLabels()[k];
        cout << "\t" << pipeline.getTestPrecision(classLabel);
    }cout << endl;

    cout << "Recall: ";
    for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
        GRT::UINT classLabel = pipeline.getClassLabels()[k];
        cout << "\t" << pipeline.getTestRecall(classLabel);
    }cout << endl;

    cout << "FMeasure: ";
    for(GRT::UINT k=0; k<pipeline.getNumClassesInModel(); k++){
        GRT::UINT classLabel = pipeline.getClassLabels()[k];
        cout << "\t" << pipeline.getTestFMeasure(classLabel);
    }cout << endl;

    GRT::MatrixDouble confusionMatrix = pipeline.getTestConfusionMatrix();
    cout << "ConfusionMatrix: \n";
    for(GRT::UINT i=0; i<confusionMatrix.getNumRows(); i++){
        for(GRT::UINT j=0; j<confusionMatrix.getNumCols(); j++){
            cout << confusionMatrix[i][j] << "\t";
        }cout << endl;
    }

    return true;
}

GestureRec::~GestureRec()
{

}
