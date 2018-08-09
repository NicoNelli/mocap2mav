#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "Automatic.h"
#include "common/CallbackHandler.hpp"
#include "poll.h"
#include "Command/TakeOff.hpp"
#include "utils/TimeHelpers.hpp"
#include "common/MavState.h"

using namespace common;

int main(int argc, char** argv){

	lcm::LCM handler, handler2, handler3, handler4, handler5;


	//the following timer are used to check if both the vision and Ultrasonic sensor are not available for some reason.
	Duration VisionData(1); //timer of 1 second
	Duration UltrasonicData(1); //timer of 1 second


	if (!handler.good() && !handler2.good() && !handler3.good() && !handler4.good() && !handler5.good())
		return 1;

	CallbackHandler call;
	Automatic autom;
	Lander lander;

	lcm::Subscription *sub   = handler.subscribe("vision_position_estimate", &CallbackHandler::visionEstimateCallback, &call);//pose of the UAV
	lcm::Subscription *sub2  = handler2.subscribe("platform/pose", &CallbackHandler::positionSetpointCallback, &call); //absolute pose of the platform
	lcm::Subscription *sub3  = handler3.subscribe("actual_task", &CallbackHandler::actualTaskCallback, &call);
	lcm::Subscription *sub4  = handler4.subscribe("apriltag_vision_system", &CallbackHandler::ApriltagCallback, &call); //topic for the vision system.
	lcm::Subscription *sub5  = handler5.subscribe("UltrasonicSensor/platform", &CallbackHandler::UltrasonicCallback, &call); //topic for the ultrasonic sensor.


	sub ->setQueueCapacity(1);
	sub2->setQueueCapacity(1);
	sub3->setQueueCapacity(1);
	sub4->setQueueCapacity(1);
	sub5->setQueueCapacity(1);


	struct pollfd fds[4];

	fds[0].fd = handler3.getFileno(); // Actual task
	fds[0].events = POLLIN;

	fds[1].fd = handler2.getFileno(); // Square pose
	fds[1].events = POLLIN;
	
	fds[2].fd = handler4.getFileno(); 
	fds[2].events = POLLIN;

	fds[3].fd = handler5.getFileno(); 
	fds[3].events = POLLIN;

    bool waiting = true;

	MavState platform;
	MavState visionSystem;
	MavState UltrasonicPos; 


	while(0==handler.handle()){

		autom.setState(call._vision_pos);
        lander.setState(call._vision_pos);

		int ret = poll(fds,4,0);

		if(fds[0].revents & POLLIN){

			handler3.handle();
			autom.setTask(call._task);

			std::cout<<  "New task arrived with action: " << printAction(autom._actualTask.action) << std::endl;

            waiting = false;
            autom.handleCommands();
		}

		if(fds[1].revents & POLLIN){

			handler2.handle();
			platform = call._position_sp;
            autom.setPlatformState(platform);
			//std::cout<<"x:"<<platform.getX()<<std::endl;
			//std::cout<<"y:"<<platform.getY()<<std::endl;
			//std::cout<<"z:"<<platform.getZ()<<std::endl;

		}
	
		if(fds[2].revents & POLLIN){
			//here there are vision data.	
		
			handler4.handle();
			
			VisionData.updateTimer(); //it takes the actual time
			VisionData._start = VisionData._actualTime;
			//every time I entered here there will be data from vision system
			//it is necessary to reset the duration of the timer (the dt will be zero)


			visionSystem = call._relative_pos;
			visionSystem.VisionDataUpdated = true;
            autom.setVisionFeedback(visionSystem);
			//std::cout<<"x:"<<visionSystem.getX()<<std::endl;
			//std::cout<<"y:"<<visionSystem.getY()<<std::endl;
			//std::cout<<"z:"<<visionSystem.getZ()<<std::endl;

		}
		else {
			
			if( VisionData.isExpired() ) {
				visionSystem.VisionDataUpdated = false;
				autom.setVisionFeedback(visionSystem);
			}
		
		}


		if(fds[3].revents & POLLIN){
			//ultrasonic sensor
			handler5.handle();
			
			UltrasonicData.updateTimer();
			UltrasonicData._start = UltrasonicData._actualTime;

			UltrasonicPos = call._Ultrasonic_pos; //copy the information from the callback to the main function.
			UltrasonicPos.UltrasonicDataUpdated = true;			
			autom.setUltrasonicInfo(UltrasonicPos);// pass them to the automatic function.

			//DEBUG:
			//std::cout<<"Z:"<<UltrasonicPos.getZ()<<std::endl;
			//std::cout<<"Vz:"<<UltrasonicPos.getVz()<<std::endl;
			//std::cout<<"valid? "<<UltrasonicPos.IsValid<<std::endl;
	

		}
		else{
			
			if( UltrasonicData.isExpired() ) {
				UltrasonicPos.UltrasonicDataUpdated = false;			
		        autom.setUltrasonicInfo(UltrasonicPos);	
			
			}
		

		}
			



        if(!waiting) {

            //Call commands
            autom.executeCommand();

            //Prepare LCM stuff
            geometry::pose command = call.mavState2LcmPose(autom._comm);
            geometry::pose platRobPos;

            platRobPos.position[0] = platform.getX();
            platRobPos.position[1] = platform.getY();
            platRobPos.position[2] = platform.getZ();

            platRobPos.velocity[0] = platform.getVx();
            platRobPos.velocity[1] = platform.getVy();
            platRobPos.velocity[2] = platform.getVz();

            handler.publish("local_position_sp", &command);

			//For gazebo visualization with the marker plugin
			handler.publish("Marker/pose_cmd", &command);

            handler3.publish("platRob", &platRobPos);
        }
	}

	return 0;

}

