//
// Created by andreanistico on 08/02/17.
//

#include "StatesClasses.hpp"
#include "parameters.h"
//Define actual states

void InitState::handle(){
    //if the state of the StateMachine is INIT...

    //Got everything (assume it)
    getSignals();
    static int wait = 0;

    //Wait 20 iterations
    if(wait++ > 20) {
        this->_contextL->setStatePtr(_nextState);
        printStateTransition(); //print the actual state.
    }
}
void HoldState::handle(){

    getSignals(); //copy the values of the LandMachine class into the ones of the AbstractLandState class.

    bool descValid = _holding && (_setPoint.getZ()   > params_automatic::zMin + 0.1);
    //before compensation,this value should be above the minimum platform value.
    //descending phase is valid if the horizontal error is under a given threshold and z distance above zMin.


    bool asceValid = _lost    && (_setPoint.getZ()   < params_automatic::zMax - 0.1);
    //coming up if the platform is lost and the z distance in under zMax

    bool compValid = _holding && (fabs(_state.getZ() - params_automatic::zMin) < 0.2) && _centered;
    //comp is a middle state between land state and hold state.
    //if the UAV is holding and is centered on the platform and is close to the platform(20 cm), will be accomplish.

    if(descValid){
        this->_contextL->setStatePtr(_nextDesState); //set next state (DescState)
        printStateTransition();
        return;
    }
    if (asceValid){
        this->_contextL->setStatePtr(_nextAscState); //set next state (AsceState)
        printStateTransition();
        return;
    }
    if(compValid){
        this->_contextL->setStatePtr(_nextState); //set next state (RToLandState)
        printStateTransition();
        return;
    }

}
void DescState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set next state (HoldState)
    printStateTransition();
}
void AsceState::handle() {
    getSignals();
    this->_contextL->setStatePtr(_nextState); //set next state (HoldState)
    printStateTransition();
}

void CompState::handle() {

    getSignals();

    bool onTarget = _NComp > params_automatic::NFramesComp;
    /*
    NComp is incremented every time the variable centered is true.
    NFramesComp is the number of consecutive frames in which tracking is considered valid
    and robot is ready for compensation.
    */

    if (!onTarget || !_centered){//if is not on the target or if is not centered..coming back!
        this->_contextL->setStatePtr(_nextState); //set next state (AsceState)
        printStateTransition();
    }

   std::cout << "VERRRRRRRR: " << _verticalErr << std::endl;

    if(onTarget && _centered && (fabs(_verticalErr) < 0.20)){
        
        //if is on the target and is centered and the vertical error is under 20 cm, set next state to LandState.
        
        this->_contextL->setStatePtr(_nextLanState);
        printStateTransition();
    }

}

void RToLandState::handle() {

    getSignals();
    if (_NComp > params_automatic::NFramesComp){
        /*
        NComp is incremented every time the variable centered is true.
        NFramesComp is the number of consecutive frames in which tracking is considered valid
        ==> robot is ready for compensation. 
        */

        this->_contextL->setStatePtr(_nextComState); //set next state (CompState)
        printStateTransition();
        return;
    } else if (!_centered){//if is not centered, coming back to the Hold state.
        this->_contextL->setStatePtr(_nextState); //set next state (HoldState)
        printStateTransition();
        return;
    }

}

void LandState::handle() {

    getSignals();
    bool onTarget = _NComp > params_automatic::NFramesComp;

    if (!onTarget || !_centered){//if is not on the target or if is not centered..coming back!
        this->_contextL->setStatePtr(_nextState);
        printStateTransition();
    }

}