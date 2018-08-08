#ifndef AUTOMATIC_H
#define AUTOMATIC_H
#include <iostream>
#include <memory>
#include "lcm_messages/geometry/pose.hpp"
#include "lcm_messages/exec/task.hpp"
#include "common/MavState.h"
#include "common/common.h"
#include "common/conversions.h"
#include "include/Lander/Lander.h"
#include "Command/TakeOff.hpp"
#include "Command/Land.hpp"
#include "Command/Move.hpp"
#include "Command/Rotate.hpp"

class Automatic
{
   
public:

    Automatic();
    void setState(MavState pose); //set UAV state
    void setPlatformState(MavState pose); //set platform state
    void setVisionFeedback(MavState pose); //set relative position using apriltag.
	void setUltrasonicInfo(MavState pose); //set the information coming from the ultrasonic sensor.	

    void setTask(exec::task task);
    exec::task getTask();
    MavState getState();
    MavState _comm;
    exec::task _actualTask;

    //Command pattern implementation
    void handleCommands();
    void executeCommand();

private:
	MavState _state; //pose of the UAV
	MavState _platformState; //position of the platform
    MavState _visionFeedbackPose; // relative position between UAV and platform   
	MavState _UltrasonicInfo; //information coming from the ultrasonic sensor.	

	std::unique_ptr<Command> _actualCommand; // template of class of type Command and its derived

};

#endif // AUTOMATIC_H
