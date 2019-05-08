//
// Created by andreanistico on 08/02/17.
//

#include "include/Lander/StatesClasses.hpp"
#include "param.h"
//Define actual states

void InitState::handle(){
	//If the state of machne is INIT...

    //Got everything (assume it)
    getSignals();
    static int wait = 0;

    //Wait 100 iterations
    if( wait++ > 50 && _VisionPose.VisionDataUpdated ) {
        this->_contextL->setStatePtr(_nextState);
        printStateTransition(); //print the actual state 
        wait=0;
    }
	else if( !_VisionPose.VisionDataUpdated ) {
		
		this->_contextL->setStatePtr(_nextHoveringState);
        printStateTransition(); //print the actual state 
	}


}

void HoveringState::handle() {
	
	getSignals();

	if( _VisionPose.VisionDataUpdated ) {
        this->_contextL->setStatePtr(_nextState);
        printStateTransition(); //print the actual state 
    }

}



void HoldState::handle(){

    //copy the values of the LandMachine class into the ones of the AbstractLandState class.

    getSignals(); 

    //before compensation, this value should be above the minimum platform value.
    //descending phase is valid if the horizontal error is under a given treshold and z distance above zMin.
    
    bool descValid = _holding && (_setPoint.getZ()   > param.zMin + 0.1);
	
	//if the  vision data are not available and the drone reaches the max height coming back to the initState

    bool InitValid = !_VisionPose.VisionDataUpdated && !(_setPoint.getZ()  < param.zMax - 0.1);	
	//std::cout<<"InitValid: "<<InitValid<<std::endl;

    //coming up if the platform is lost and the z distance is under zMax
	
	bool asceValid = _lost    && (_setPoint.getZ()   < param.zMax - 0.1);
	
    //comp is a middle state between land state and hold state.
    //if the UAV is holding ad is centered on the platform and is close to the platform(20 cm), will be accomplish.
	
    bool compValid = _holding && (fabs( _UltraInfo.getZ() + param.zMin) < 0.2 ) && _centered;


    if(descValid){
        this->_contextL->setStatePtr(_nextDesState); //set the next state(DescState)
        printStateTransition();
        return;
    }
    if (asceValid){
        this->_contextL->setStatePtr(_nextAscState); //set the next state(AsceState)
        printStateTransition();
        return;
    }

    if(compValid){
        this->_contextL->setStatePtr(_nextState); //set the next state(RToLandState)
        printStateTransition();
        return;
    }


    if(InitValid){
	this->_contextL->setStatePtr(_nextInitState); //set the next state(InitState)
        printStateTransition();
        return;
    }



}
void DescState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set the next state(HoldState)
    printStateTransition();
}
void AsceState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set the next state(HoldState)
    printStateTransition();
}

void CompState::handle() {

    getSignals();

    /*
    NComp is incremented every time the variable centered is true.
    NframesComp is the number of consecutive frames in which tracking is considered valid
    and robot is ready for compensation.
    */

    bool onTarget = _NComp > param.NFramesComp;

    if (!onTarget || !_centered){ 
        //if is not on the target or if is not centered..coming back!
        
        this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();
    
	}else if ( !_UltraInfo.UltrasonicDataUpdated ){ 
        //if the ultrasonic sensor data are not available..
        
        this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();
    
	}else if ( fabs(_VisionPose.getRoll()) > param.RollThreshold || fabs(_VisionPose.getPitch()) > param.PitchThreshold ){

		this->_contextL->setStatePtr(_nextState); //set the next state(AsceState)
        printStateTransition();

	}


  // std::cout << "VERRRRRRRR: " << _verticalErr << std::endl;

    if(onTarget && _centered && (fabs(_verticalErr) < param.land_threshold)){
	   //if is on the target and is centered and the vertical error is under 25cm, set next state to LandState.

        this->_contextL->setStatePtr(_nextLanState);
        printStateTransition();
    }

}

void RToLandState::handle() {

    getSignals();
    if (_NComp > param.NFramesComp){
        this->_contextL->setStatePtr(_nextComState); //set the next state(CompState)
        printStateTransition();
        return;
    } else if (!_centered){
        this->_contextL->setStatePtr(_nextState); //set the next state(HoldState)
        printStateTransition();
        return;
    }

}

void LandState::handle() {

    getSignals();
    static int wait = 0;
    bool onTarget = _NComp > param.NFramesComp;

    if (!onTarget || !_centered){ //set the next state(AsceState)
        //this->_contextL->setStatePtr(_nextState);
        printStateTransition();
        wait = 0;
    }

    //Restart the procedure
    //if(wait++ > 100) {
     //   this->_contextL->setStatePtr(_restartState); //set the next state(InitState)
       // printStateTransition();
        //wait=0;
    //}

}
