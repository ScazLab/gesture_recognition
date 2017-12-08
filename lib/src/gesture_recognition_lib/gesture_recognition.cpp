#include "gesture_recognition_lib/gesture_recognition.h"

using namespace std;

GestureRec::GestureRec(string name, string limb) : PerceptionClientImpl(name, limb),
                                        nh(name), spinner(4), limb(limb), it(nh)
{

    std::string action_topic = "/gesture_recognition/action_provider";
    service = nh.advertiseService(action_topic, &GestureRec::actionCb, this);

    setUpTrainingData();
    setUpPipeline();
    setPredictAdd(true);

    std::string result_topic = "/gesture_recognition/result";
    gesture_pub = nh.advertise<gesture_recognition::RecState>(result_topic, 1000);
    gesture_sub = nh.subscribe(result_topic, 1000, &GestureRec::gestureRecCb, this);

    std::string state_topic = "/gesture_recognition/state";
    state_pub = nh.advertise<gesture_recognition::GestureState>(state_topic, 1000);
    state_sub = nh.subscribe(state_topic, 1000, &GestureRec::gestureStateCb, this);

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
    rec_state.expected_class = 0;
    rec_window = 3.0;

    red     = cv::Scalar( 44,  48, 201); // BGR color code
    green   = cv::Scalar( 60, 160,  60);
    yellow  = cv::Scalar( 60, 200, 200);
    blue    = cv::Scalar(200, 162,  77);
    black   = cv::Scalar(  0,   0,   0);

    displayRecState();


    spinner.start();
}

void GestureRec::displayText(cv::Mat& in)
{
    if (display_text != "")
    {
        int thickness = 3;
        int baseline = 0;
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        int fontScale = 2;

        int border = 20;
        int max_width = 700;
        cv::Size textSize = cv::getTextSize(display_text, fontFace, fontScale, thickness, &baseline);
        int numLines = int(textSize.width/max_width)+1;

        if (numLines > 5)
        {
            fontScale = 1.6;
            thickness = 2;
            textSize = cv::getTextSize(display_text, fontFace, fontScale, thickness, &baseline);
            numLines = int(textSize.width/max_width);
        }

        std::vector<std::string> line;
        std::vector<cv::Size>    size;

        int interline = 20;
        int rec_height = -interline;
        int rec_width = 0;
        int line_length = int(display_text.size()/numLines);

        for (int i = 0; i < numLines; ++i)
        {
            if (i==numLines-1)
            {
                line.push_back(display_text.substr(i*line_length, display_text.size()-i*line_length));
            }
            else
            {
                line.push_back(display_text.substr(i*line_length,line_length));
            }

            size.push_back(cv::getTextSize( line.back(), fontFace, fontScale, thickness, &baseline));
            if (size.back().width>rec_width) rec_width=size.back().width;
            rec_height += interline + size.back().height;
        }

        rec_height += 2*border;
        rec_width  += 2*border;

        cv::Point rectOrg((in.cols - rec_width)/2, (in.rows - rec_height)/2);
        cv::Point rectEnd((in.cols + rec_width)/2, (in.rows + rec_height)/2);
        rectangle(in, rectOrg, rectEnd, blue, -1);

        int textOrgY = rectOrg.y + border;
        for (int i = 0; i < numLines; ++i)
        {
            textOrgY += size[i].height;
            cv::Point textOrg((in.cols - size[i].width)/2, textOrgY);
            putText(in, line[i], textOrg, fontFace, fontScale, cv::Scalar::all(225), thickness, CV_AA);
            textOrgY += interline;
        }

        printf("\n");
    }
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

    putText(res, "Gesture Found:", cv::Point(20, 200), fontFace, fontScale/2, col, 2, 8);
    putText(res, "Class Label:", cv::Point(20, 300), fontFace, fontScale/2, col, 2, 8);
    if (rec_state.gesture_found)
    {
        putText(res, "YES", cv::Point(250, 250), fontFace, fontScale/1.25, col, thickness, CV_AA);
        putText(res, to_string(rec_state.predicted_class), cv::Point(250, 350), fontFace, fontScale/1.25, col, thickness, CV_AA);
    }
    else
    {
        putText(res, "NONE", cv::Point(250, 250), fontFace, fontScale/1.25, col, thickness, CV_AA);
        putText(res, "NONE", cv::Point(250, 350), fontFace, fontScale/1.25, col, thickness, CV_AA);
    }

    if (pipeline.getTrained())
    {
        // int classes = 4;
        int classes = trainingData.getNumClasses();
        int spacing = 500 / classes;
        // int bar_width;
        cv::Scalar color;
        bool predicted_correctly = (rec_state.expected_class == rec_state.predicted_class);
        if (spacing > 6)
        {
            // bar_width = spacing - 5;
            for (int i = 0; i < classes; i++)
            {
                int x = 500 + i * spacing;
                int y_start = 450;
                int distance = getClassDistance(i);
                ROS_INFO("Distance for class %d: %d", i, distance);
                if (distance > 10)
                {
                    distance = 10;
                }
                int y_end = 250 + 20 * distance;
                if (predicted_correctly && i == rec_state.predicted_class)
                {
                    color = green;
                }
                else if (!predicted_correctly && i == rec_state.predicted_class)
                {
                    color = yellow;
                }
                else if (!predicted_correctly && i == rec_state.expected_class)
                {
                    color = green;
                }
                else
                {
                    color = red;
                }
                rectangle(res, cv::Point(x, y_start), cv::Point(x+50, y_end), color, -1);
                putText(res, to_string(100 - 10*distance), cv::Point(x+5, y_end - 20), fontFace, fontScale/2, col, thickness, CV_AA);
                putText(res, to_string(i), cv::Point(x+8, y_start + 55), fontFace, fontScale/1.25, col, thickness, CV_AA);
            }
        }
    }

    displayText(res);

    cv_bridge::CvImage msg;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.image    = res;

    im_pub.publish(msg.toImageMsg());
}

int GestureRec::getClassDistance(int class_name)
{
    GRT::Classifier *classifier = pipeline.getClassifier();
    GRT::DTW *new_dtw = &dtw;
    new_dtw->deepCopyFrom(classifier);
    GRT::VectorFloat classDistances = new_dtw->getClassDistances();
    GRT::UINT numTemplates = new_dtw->getNumTemplates();
    GRT::Vector < GRT::DTWTemplate > templatesBuffer = new_dtw->getModels();

    for (GRT::UINT k = 0; k < numTemplates; k++)
    {
        int template_class = templatesBuffer[k].classLabel;
        if (template_class == class_name)
        {
            return classDistances[k];
        }
    }
    ROS_INFO("Could not find distance for class %d", class_name);
    return 1e8;
}

void GestureRec::gestureRecCb(const gesture_recognition::RecState& msg)
{
    gesture_found = msg.gesture_found;
    gesture_id = msg.predicted_class;

    rec_state.gesture_found = msg.gesture_found;
    rec_state.predicted_class = msg.predicted_class;
    rec_state.expected_class = msg.expected_class;
}


void GestureRec::gestureStateCb(const gesture_recognition::GestureState& msg)
{
    ROS_INFO("Called gestureStateCb");
    if (msg.publishing)
    {
        // beginPublishThread();
    }
}

void GestureRec::beginPublishThread()
{
    if (publish)
    {
        GRT::MatrixFloat gesture;
        beginRecording(&gesture);
        rec_timer = nh.createTimer(ros::Duration(rec_window),
                                    boost::bind(&GestureRec::predictPublishCb, this, &gesture), true);
        // thread_timer = nh.createTimer(ros::Duration(0.5),
        //                                boost::bind(&GestureRec::beginPublishThread, this), true);
    }
    // begin recording a gesture
    // begin timer to stop recording and run prediction on data
    // timer callback should be beginPublishThread
}

void GestureRec::beginRecording(GRT::MatrixFloat *gesture)
{
    GRT::VectorFloat currPos(trainingData.getNumDimensions());

    GRT::UINT gestureLength = 0;

    // record a gesture sample
    ros::Rate r(60);
    while(ros::ok() && waitForData() && gestureLength < 30)
    {
        currPos[0] = curr_object_pos.x;
        currPos[1] = curr_object_pos.y;
        currPos[2] = curr_object_pos.z;

        // ROS_INFO("object is in: %g %g %g", curr_object_pos.x,
        //                                         curr_object_pos.y,
        //                                         curr_object_pos.z);
        gesture->push_back(currPos);
        gestureLength++;
        r.sleep();
    }

    // *gesture = preProcess(*gesture);
}

void GestureRec::predictPublishCb(GRT::MatrixFloat *gesture)
{
    if(pipeline.getTrained())
        {
            // GRT::MatrixFloat predictGesture(*gesture);
            // if(!pipeline.predict(*gesture))
            // {
            //     ROS_INFO("Unable to predict using the pipeline. Check that it has been trained with appropriate samples.");
            //     return;
            // }

            GRT::UINT predicted_label = pipeline.getPredictedClassLabel();

            gesture_recognition::RecState msg;
            msg.gesture_found = true;
            msg.predicted_class = predicted_label;
            gesture_pub.publish(msg);
            displayRecState();
        }
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
    dtw.enableNullRejection(true);
    // dtw.setNullRejectionCoeff(3);
    // dtw.enableTrimTrainingData(true, 0.1, 90);
    // dtw.setOffsetTimeseriesUsingFirstSample(true);

    pipeline.setClassifier( dtw );
    // pipeline.setClassifier( GRT::HMM() );
    return true;
}

void GestureRec::setDisplayText(const std::string &s)
{
    display_text = s;
}

void GestureRec::deleteDisplayText()
{
    setDisplayText("");
    displayRecState();
}

void GestureRec::setPredictAdd(bool predict)
{
    predict_and_add = predict;
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
    msg.publishing = publish;

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
        publishState();
        beginPublishThread();
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
    setDisplayText("Recording in 5...");
    displayRecState();
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
    setDisplayText("Recording!");
    displayRecState();
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
    setDisplayText("Sample Recorded");
    displayRecState();
    ros::Duration(1.0).sleep();
    deleteDisplayText();
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

    setDisplayText("Recording in 5...");
    displayRecState();
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
    setDisplayText("Recording!");
    displayRecState();
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
    setDisplayText("Sample recorded, running prediction");
    displayRecState();
    ros::Duration(1.0).sleep();
    deleteDisplayText();
    GRT::MatrixFloat processedGesture = preProcess(gesture);

    if(pipeline.getTrained())
        {
            if(!pipeline.predict(processedGesture))
            {
                ROS_INFO("Unable to predict using the pipeline. Check that it has been trained with appropriate samples.");
                return false;
            }

            GRT::UINT predicted_label = pipeline.getPredictedClassLabel();

            gesture_recognition::RecState msg;
            msg.gesture_found = true;
            msg.predicted_class = predicted_label;
            msg.expected_class = gestureLabel;
            gesture_pub.publish(msg);

            if (gestureLabel >= 0)
            {
                ROS_INFO("Predicted label was %d, expected label was %d.", predicted_label, gestureLabel);
            }
            if (getPredictAdd())
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
            displayRecState();
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
