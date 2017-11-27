#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb) : PerceptionClientImpl(name, limb),
                                        nh(name), spinner(4), limb(limb), it(nh)
{

    std::string action_topic = "/gesture_recognition/action_provider";
    service = nh.advertiseService(action_topic, &GestureRec::actionCb, this);

    setUpTrainingData();
    setUpPipeline();

    std::string result_topic = "/gesture_recognition/result";
    gesture_pub = nh.advertise<gesture_recognition::RecState>(result_topic, 1000);
    gesture_sub = nh.subscribe(result_topic, 1000, &GestureRec::gestureRecCb, this);

    std::string state_topic = "/gesture_recognition/state";
    state_pub = nh.advertise<gesture_recognition::GestureState>(state_topic, 1000);

    // just for now, expect marker 198
    object_id = 201;
    publish = false;
    // publishGestures();


    // display
    im_pub = it.advertise("/robot/xdisplay", 1);
    im_h = 600;
    im_w = 1024;
    im_w_delim = 8;

    rec_state.gesture_found = false;
    rec_state.predicted_class = 0;

    red   = cv::Scalar(  44,  48, 201); // BGR color code
    green = cv::Scalar(  60, 160,  60);
    blue  = cv::Scalar( 200, 162,  77);
    black = cv::Scalar(   0,   0,   0);

    displayRecState();


    spinner.start();
}

void GestureRec::displayRecState()
{
    cv::Mat res(im_h,im_w,CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Scalar col = cv::Scalar::all(60);

    int thickness = 3;
    int baseline = 0;
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    int fontScale = 2;

    // place a centered title on top
    std::string title = "GESTURE RECOGNITION STATE";
    cv::Size textSize = cv::getTextSize( title, fontFace, fontScale, thickness, &baseline);
    cv::Point textOrg((res.cols - textSize.width)/2, (res.rows + textSize.height)/6);
    putText(res, title, textOrg, fontFace, fontScale, col, thickness, CV_AA);

    putText(res, "Gesture Found:", cv::Point(20, 300), fontFace, fontScale/2, col, 2, 8);
    if (rec_state.gesture_found)
    {
        putText(res, "YES", cv::Point(250, 350), fontFace, fontScale, col, thickness, CV_AA);
    }
    else
    {
       putText(res, "NONE", cv::Point(250, 350), fontFace, fontScale, col, thickness, CV_AA);
    }

    putText(res, "Class Label:", cv::Point(20, 400), fontFace, fontScale/2, col, 2, 8);
    if (rec_state.gesture_found)
    {
        putText(res, to_string(rec_state.predicted_class), cv::Point(250, 450), fontFace, fontScale/1.25, col, thickness, CV_AA);
    }
    else
    {
        putText(res, "NONE", cv::Point(250, 450), fontFace, fontScale/1.25, col, thickness, CV_AA);
    }

    cv_bridge::CvImage msg;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.image    = res;

    im_pub.publish(msg.toImageMsg());
}

void GestureRec::gestureRecCb(const gesture_recognition::RecState& msg)
{
    gesture_found = msg.gesture_found;
    gesture_id = msg.predicted_class;
}

void GestureRec::publishGestures()
{
    setObjectID(object_id);
    gesture_recognition::RecState msg;
    msg.gesture_found = false;
    msg.predicted_class = 0;

    if(!pipeline.getTrained())
    {
            ROS_INFO("Pipeline hasn't been trained yet so no gestures will be recognized");
    }

    GRT::VectorFloat currPos(trainingData.getNumDimensions());

    ros::Rate r(60);
    while(ros::ok() && waitForData())
    {
        currPos[0] = curr_object_pos.x;
        currPos[1] = curr_object_pos.y;
        currPos[2] = curr_object_pos.z;

        if(publish && pipeline.getTrained())
        {
            if(!pipeline.predict(currPos))
            {
                ROS_INFO("Unable to predict using the pipeline. Check that it has been trained with appropriate samples.");
                publish = false;
            }
            else
            {
                GRT::UINT predicted_label = pipeline.getPredictedClassLabel();
                msg.gesture_found = true;
                msg.predicted_class = predicted_label;
                gesture_pub.publish(msg);
            }
        }
        r.sleep();
    }
}

bool GestureRec::setUpPipeline()
{
    GRT::DTW dtw;
    dtw.enableNullRejection(true);
    // dtw.setNullRejectionCoeff(3);
    // dtw.enableTrimTrainingData(true, 0.1, 90);
    // dtw.setOffsetTimeseriesUsingFirstSample(true);

    pipeline.setClassifier( dtw );
    // pipeline.setClassifier( GRT::HMM() );
    return true;
}

bool GestureRec::setState(int _state)
{
    state.set(_state);

    return publishState();
}

bool GestureRec::publishState()
{
    gesture_recognition::GestureState msg;

    msg.state = string(getState());
    msg.action = getAction();
    msg.object = object_id;

    state_pub.publish(msg);

    return true;
}

bool GestureRec::doAction(std::string action, std::string filename)
{
    setState(START);
    if (action == "record")
    {
        if (!recordSample(trainingData, class_label, object_id, filename)) return false;
        setState(DONE);
        return true;
    }
    else if (action == "train")
    {
        GRT::TimeSeriesClassificationData pipelineData;

        if( filename.empty() ) {
            pipelineData = trainingData;
        }
        else if( !pipelineData.load(filename) ){
            ROS_ERROR("ERROR: Failed to load training data from file\n");
            return false;
        }

        if (!trainPipeline(pipeline, pipelineData)) return false;
        setState(DONE);
        return true;
    }
    else if (action == "test")
    {
        if(!testPipeline(pipeline)) return false;
        setState(DONE);
        return true;
    }
    else if (action == "predict")
    {
        if(!predictOnce(pipeline, class_label, object_id, filename)) return false;
        setState(DONE);
        return true;
    }
    else if (action == "publish")
    {
        publish = true;
        setState(DONE);
        return true;
    }
    else if (action == "print")
    {
        GRT::TimeSeriesClassificationData dataset;
        if (!filename.empty())
        {
            dataset.load(filename);
        }
        else
        {
            dataset = trainingData;
        }
        dataset.printStats();
        setState(DONE);
        return true;
    }

    setState(ERROR);
    ROS_INFO("Action %s is not allowed.", action.c_str());
    return false;
}

bool GestureRec::setAction(const string& _action)
{
    action = _action;
    publishState();
    return true;
}


bool GestureRec::actionCb(gesture_recognition::DoAction::Request &req,
                          gesture_recognition::DoAction::Response &res)
{
    std::string action = req.action;
    setAction(action);
    std::string filename = req.filename;
    class_label = req.class_label;

    int object_id = req.object_id;
    setObjectID(object_id);

    ROS_INFO("%s was requested with filename %s, class label %d, and object ID %d", action.c_str(), filename.c_str(), class_label, object_id);

    if (!doAction(action, filename))
    {
        res.success = false;
        return false;
    }

    res.success = true;
    return true;
}

bool GestureRec::recordSample(GRT::TimeSeriesClassificationData &trainingData, GRT::UINT gestureLabel, int object_id, std::string filename)
{
    setObjectID(object_id);

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
    GRT::VectorFloat currPos(trainingData.getNumDimensions());

    GRT::UINT gestureLength = 0;

    // record a gesture sample for 1 second
    ros::Rate r(60);
    while(ros::ok() && waitForData() && gestureLength < 30)
    {
        currPos[0] = curr_object_pos.x;
        currPos[1] = curr_object_pos.y;
        currPos[2] = curr_object_pos.z;

        ROS_INFO("object is in: %g %g %g", curr_object_pos.x,
                                                curr_object_pos.y,
                                                curr_object_pos.z);
        gesture.push_back(currPos);
        gestureLength++;
        r.sleep();
    }

    GRT::MatrixFloat processedGesture = preProcess(gesture);

    GRT::VectorFloat processedGestureMean(processedGesture.getNumCols());

    trainingData.addSample( gestureLabel, processedGesture);

    trainingData.printStats();

    if (!filename.empty())
    {
        if( !trainingData.save( filename ) )
        {
            ROS_ERROR("Failed to save dataset to file!");
            return false;
        }

        ROS_INFO("Sample recorded; dataset saved to file.");
        return true;
    }

    ROS_INFO("Sample recorded; no save file specified");
    return true;
}

GRT::MatrixFloat GestureRec::preProcess(GRT::MatrixFloat rawGesture)
{
    if (!rawGesture.znorm())
    {
        ROS_ERROR("Unable to normalize data.");
    }
    return rawGesture;

}



bool GestureRec::trainPipeline(GRT::GestureRecognitionPipeline &pipeline, GRT::TimeSeriesClassificationData pipelineData)
{

    if( !pipeline.train( pipelineData) )
    {
        ROS_ERROR("ERROR: Failed to train the pipeline!");
        return false;
    }
    trainingData = pipelineData;
    return true;
}

bool GestureRec::predictOnce(GRT::GestureRecognitionPipeline &pipeline, GRT::UINT gestureLabel, int object_id, std::string filename)
{
    setObjectID(object_id);

    if (!pipeline.getTrained())
    {
        ROS_INFO("Pipeline has not been trained, unable to predict gestures.");
        return false;
    }

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
    GRT::VectorFloat currPos(trainingData.getNumDimensions());

    GRT::UINT gestureLength = 0;

    // record a gesture sample for 1 second
    ros::Rate r(60);
    while(ros::ok() && waitForData() && gestureLength < 30)
    {
        currPos[0] = curr_object_pos.x;
        currPos[1] = curr_object_pos.y;
        currPos[2] = curr_object_pos.z;

        ROS_INFO("object is in: %g %g %g", curr_object_pos.x,
                                                curr_object_pos.y,
                                                curr_object_pos.z);
        gesture.push_back(currPos);
        gestureLength++;
        r.sleep();
    }

    GRT::MatrixFloat processedGesture = preProcess(gesture);

    if(pipeline.getTrained())
        {
            if(!pipeline.predict(processedGesture))
            {
                ROS_INFO("Unable to predict using the pipeline. Check that it has been trained with appropriate samples.");
                return false;
            }

            GRT::UINT predicted_label = pipeline.getPredictedClassLabel();
            if (gestureLabel >= 0)
            {
                ROS_INFO("Predicted label was %d, expected label was %d.", predicted_label, gestureLabel);
            }
            if (predicted_label == gestureLabel)
            {
                trainingData.addSample( gestureLabel, processedGesture);
                ROS_INFO("Added correctly labeled sample to training data.");
                if (!filename.empty())
                {
                    if (!trainingData.save( filename ))
                    {
                        ROS_INFO("Updated training data was not saved to file!");
                    }
                }
            }
            return true;

        }

    return false;
}

bool GestureRec::testPipeline(GRT::GestureRecognitionPipeline &pipeline)
{
    // Generate some data
    GRT::TimeSeriesClassificationData testData;
    testData.setNumDimensions(3);
    testData.setDatasetName("TestingData");

    GRT::TimeSeriesClassificationData testingSample;
    testingSample.setNumDimensions(3);
    testingSample.setDatasetName("SampleToTest");

    GRT::Classifier* classifier = pipeline.getClassifier();
    cout << "classifier: " << classifier->getClassifierType() << endl;


    GRT::UINT gestureLabel = 1;

    GRT::MatrixFloat trainingSample;

    //For now we will just add 10 x 20 random walk data timeseries
    GRT::Random random;
    for(GRT::UINT k=0; k<10; k++){//For the number of classes
        gestureLabel = k+1;

        //Get the init random walk position for this gesture
        GRT::VectorFloat startPos( trainingData.getNumDimensions() );
        for(GRT::UINT j=0; j<startPos.size(); j++){
            startPos[j] = random.getRandomNumberUniform(-1.0,1.0);
        }

        //Generate the 20 time series
        for(GRT::UINT x=0; x<20; x++){

            //Clear any previous timeseries
            trainingSample.clear();

            //Generate the random walk
            GRT::UINT randomWalkLength = random.getRandomNumberInt(90, 110);
            GRT::VectorFloat sample = startPos;
            for(GRT::UINT i=0; i<randomWalkLength; i++){
                for(GRT::UINT j=0; j<startPos.size(); j++){
                    sample[j] += random.getRandomNumberUniform(-0.1,0.1);
                }

                //Add the sample to the training sample
                trainingSample.push_back( sample );
            }

            //Add the training sample to the dataset
            trainingData.addSample( gestureLabel, trainingSample );

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
    if (gesture_thread.joinable()) { gesture_thread.join(); }
}
