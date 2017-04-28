#include "gesture_recognition.h"

using namespace std;

GestureRec::GestureRec()
{

}

bool GestureRec::recTrainingData()
{
    //Create a new instance of the ClassificationData
    GRT::ClassificationData trainingData;

    //Set the dimensionality of the data
    trainingData.setNumDimensions( 3 );

    // For now, this will work for straight-line gestures
    // each gesture must have a different endpoint relative to starting point

    if (!waitForARucoData()) return false;
    double start_x = getMarkerPos().x;
    double start_y = getMarkerPos().y;
    double start_z = getMarkerPos().z;

    // gestures will be 1s long
    ros::Duration(1.0).sleep();

    if (!waitForARucoData()) return false;
    double delta_x = getMarkerPos().x - start_x;
    double delta_y = getMarkerPos().y - start_y;
    double delta_z = getMarkerPos().z - start_z;

    GRT::UINT gestureLabel = 1;
    vector< double > sample(3);
    sample[0] = delta_x;
    sample[1] = delta_y;
    sample[2] = delta_z;

    //Add the sample to the training data
    trainingData.addSample( gestureLabel, sample );

    bool saveResult = trainingData.save( "TrainingData.grt" );

    return true;

}

GestureRec::~GestureRec()
{

}
