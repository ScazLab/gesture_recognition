#ifndef __GESTURE_REC__
#define __GESTURE_REC__

#include <ros/ros.h>
#include <ros/console.h>

#include "robot_utils/utils.h"

#include <GRT/GRT.h>
#include "robot_perception/aruco_client.h"

class GestureRec // : public ARucoClient
{
private:
    /**
     * @brief record & save sample data
     * @return true/false if success/failure
     */
    bool recTrainingData();

public:
    GestureRec();
    ~GestureRec();

};

#endif
