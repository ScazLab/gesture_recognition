#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb) : _nh(name), _limb(limb),
                                      aruco_ok(false), markers_found(false),
                                         marker_found(false), marker_id(-1)
{
    _aruco_sub = _nh.subscribe("/aruco_marker_publisher/markers",
                               3, &GestureRec::ARucoCb, this);

    std::string action_topic = "/gesture_recognition/action_provider";
    service = _nh.advertiseService(action_topic, &GestureRec::actionCb, this);

    setUpTrainingData();
    setUpPipeline();

    std::string result_topic = "/gesture_recognition/result";
    gesture_pub = _nh.advertise<gesture_recognition::GestureState>(result_topic, 1000);
    gesture_sub = _nh.subscribe(result_topic, 1000, &GestureRec::gestureCb, this);

    // publishGestures();

}

void GestureRec::gestureCb(const gesture_recognition::GestureState& msg)
{
    gesture_found = msg.gesture_found;
    gesture_id = msg.predicted_class;
}

void GestureRec::publishGestures()
{
    gesture_recognition::GestureState msg;
    msg.gesture_found = false;
    msg.predicted_class = 0;

    GRT::MatrixFloat gesture;
    GRT::Vector< GRT:: VectorFloat > gestureVec;
    GRT::VectorFloat sample(3);

    if(!pipeline.getTrained())
        {
            ROS_INFO("Pipeline hasn't been trained yet so no gestures will be recognized");
        }

    while (ros::ok())
    {


        sample[0] = curr_marker_pos.x;
        sample[1] = curr_marker_pos.y;
        sample[2] = curr_marker_pos.z;
        gestureVec.push_back(sample);

        gesture = gestureVec;

        if(pipeline.getTrained())
        {
            if(pipeline.predict(gesture)){
                GRT::UINT predicted_label = pipeline.getPredictedClassLabel();
                msg.gesture_found = true;
                msg.predicted_class = predicted_label;
            }
        }

        gesture_pub.publish(msg);
    }
}

bool GestureRec::actionCb(gesture_recognition::DoAction::Request &req,
                          gesture_recognition::DoAction::Response &res)
{
    std::string action = req.action;
    std::string filename = req.filename;
    GRT::UINT class_label = req.class_label;
    int marker_id = req.marker_id;


    res.success = false;

    if (action == "record")
    {
        if (!recordSample(trainingData, class_label, marker_id, filename)) return false;
        res.success = true;
        return true;
    }

    if (action == "train")
    {
        GRT::TimeSeriesClassificationDataStream pipelineData;

        if( filename.empty() ) {
            pipelineData = trainingData;
        }
        else if( !pipelineData.load(filename) ){
            ROS_ERROR("ERROR: Failed to load training data from file\n");
            return false;
        }

        if (!trainPipeline(pipeline, pipelineData)) return false;
        res.success = true;
        return true;
    }

    if (action == "test")
    {
        if(!testPipeline(pipeline)) return false;
        res.success = true;
        return true;
    }


    if (action == "predict")
    {
        if(!predictOnce(pipeline, marker_id)) return false;
        res.success = true;
        return true;
    }

    return false;
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

bool GestureRec::recordSample(GRT::TimeSeriesClassificationDataStream &trainingData, GRT::UINT gestureLabel, int marker_id, std::string filename)
{
    setMarkerID(marker_id);

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

    trainingData.printStats();

    if (!filename.empty())
    {
        if( !trainingData.save( filename ) )
        {
            ROS_ERROR("Failed to save dataset to file!");
            return false;
        }

    ROS_INFO("Sample recorded; dataset saved to file.");

    }

    ROS_INFO("Sample recorded; no save file specified");
    return true;
}

bool GestureRec::trainPipeline(GRT::GestureRecognitionPipeline &pipeline, GRT::TimeSeriesClassificationDataStream pipelineData)
{

    if( !pipeline.train( pipelineData) )
    {
        ROS_ERROR("ERROR: Failed to train the pipeline!");
        return false;
    }

    return true;
}

bool GestureRec::predictOnce(GRT::GestureRecognitionPipeline &pipeline, int marker_id)
{
    setMarkerID(marker_id);

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
    if(pipeline.getTrained())
        {
            if(!pipeline.predict(gesture))
            {
                ROS_INFO("Unable to predict using the pipeline. Check that it has been trained with appropriate samples.");
                return false;
            };

            GRT::UINT predicted_label = pipeline.getPredictedClassLabel();
            cout << "Predicted label: " << predicted_label << endl;
            return true;

        }

    return false;
}

bool GestureRec::testPipeline(GRT::GestureRecognitionPipeline &pipeline)
{
    // Generate some data
    GRT::TimeSeriesClassificationDataStream testData;
    testData.setNumDimensions(3);
    testData.setDatasetName("TestingData");

    GRT::TimeSeriesClassificationDataStream testingSample;
    testingSample.setNumDimensions(3);
    testingSample.setDatasetName("SampleToTest");

    GRT::Classifier* classifier = pipeline.getClassifier();
    cout << "classifier: " << classifier->getClassifierType() << endl;


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
            GRT::VectorDouble train_sample = startPos;
            GRT::VectorDouble test_sample = startPos;

            for(GRT::UINT i=0; i<randomWalkLength; i++){
                for(GRT::UINT j=0; j<train_sample.size(); j++){
                    train_sample[j] += random.getRandomNumberUniform(-0.1,0.1);
                    test_sample[j] += random.getRandomNumberUniform(-0.1,0.1);
                }

                //Add the training sample to the dataset
                testData.addSample(gestureLabel, train_sample );
                testingSample.addSample(gestureLabel, test_sample);
            }

            //now add some noise to represent a null class
            for(GRT::UINT i=0; i<50; i++){
                for(GRT::UINT j=0; j<train_sample.size(); j++){
                    train_sample[j] = random.getRandomNumberUniform(-0.01,0.01);
                    test_sample[j] = random.getRandomNumberUniform(-0.1,0.1);
                }

                //Add the training sample to the dataset, note that we set the gesture label to 0
                testData.addSample(0, train_sample );
                testingSample.addSample(0, test_sample);
            }
        }
    }

    // testData.printStats();
    // testingSample.printStats();

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
