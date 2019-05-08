

#ifndef MOCAP2MAV_STATESCLASSES_HPP
#define MOCAP2MAV_STATESCLASSES_HPP

#include <iostream>
#include "StateMachine/include/Machine.h"
#include "common/MavState.h"
#include "Parameters.h"

/** @ingroup Automatic
 * @brief Derived class of the Machine one.
 * 
 * @details Being the type of access public,
 * the private members of the base class are
 * hidden to the derivated one.
 * Its public members are used to share
 * the actual state of the drone, members of the Lander class,
 * with the ones of the landMachine.
 */

class LandMachine : public Machine {

public:

    /**
     * Horizontal error between drone and platform using vision system
     */

    double *_horizontaErr;

    /**
     *  Vertical error between drone and platform using vision system
     * and ultrasonic sensor 
     */
    
    double *_verticalErr;
    
    /**
     * Threshold value for the horizontal error to allow the hold state
     */
    
    double *_tauHold;
    
    /**
     * Threshold value for the horizontal error to check if the platform is lost
     */

    double *_tauLost;
    
    /**
     * It seems not used
     */

    double *_tauErr;
    
    /**
     * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
     */

    int *_NHold;
    
    /**
     * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
     */

    int *_NLost;
    
    /**
     * It keeps how many times the drone is on place to accomplish the landing
     */

    int *_NComp;
    
    /**
     * The full state of the drone respect to the world frame
     */

    MavState *_state;
    
    /**
     * The position setPoint to accomplish the landing 
     */

    MavState *_setPoint;
	
    /**
     * Relative z position and velocity of the drone respect to the platform
     */

    MavState *_VisionPose;
	
    /**
     * Relative z position and velocity of the drone respect to the platform
     */

    MavState *_UltraInfo;
    
    /**
     * It checks if the drone is holding (horizontal error) the platform
     */
    
    bool *_holding;
    
    /**
     * It checks if the drone has lost the platform
     */

    bool *_lost;
    
    /**
     * It checks if the drone is on place respect to the platform
     */

    bool *_centered;

	Parameters *param;

};

/** @ingroup Automatic
 * @brief Derived class of the AbstractState.
 * @details Being the type of access public,
 * the private members of the base class are
 * hidden to the derivated one. 
 */

class AbstractLandState : public AbstractState {

public:

    /**
     * It is a pointer to the next state.
     * It is useful to link each state and to build the 
     * state machine.
     */

    AbstractState* _nextState;
    
    /**
     * Enumeration with represents the possible state of the land state machine
     */

    enum states{

        INIT,
		HOVERING,
        HOLD,
        DESC,
        ASCE,
        R2LA,
        COMP,
        LAND

    };

    /**
     * @brief Constructor of the derived class. 
     * @param context Pointer to the LandMachine class
     */

    AbstractLandState(LandMachine *context) : AbstractState(context){
        _contextL = context;
    }

    /**
     * @brief Such method allows to connect some members of the LandMachine class
     * with the ones of each state. 
     * @details More in details, the Lander class has some members that it wants to share 
     * with the LandMachine class, then the LandMachine class will share them also with
     * those states that want to know the actual situation. 
     */

    void getSignals(){

         _horizontaErr = *(_contextL->_horizontaErr);
         _verticalErr  = *(_contextL->_verticalErr);
         _tauHold      = *(_contextL->_tauHold);
         _tauLost      = *(_contextL->_tauLost);
         _tauErr       = *(_contextL->_tauErr);
         _NHold        = *(_contextL->_NHold);
         _NLost        = *(_contextL->_NLost);
         _NComp        = *(_contextL->_NComp);
         _state        = *(_contextL->_state);
         _setPoint     = *(_contextL->_setPoint);
		 _VisionPose   = *(_contextL->_VisionPose);
		 _UltraInfo    = *(_contextL->_UltraInfo);		
         param         = *(_contextL->param);
		//New Signals
        _holding       = *(_contextL->_holding);
        _centered      = *(_contextL->_centered);
        _lost          = *(_contextL->_lost);

    }

    /**
     * @brief Such method prints the actual ID of the current state
     */

    void printStateTransition(){
        std::cout << "Actual state: " << _contextL->getActualNodeId() << std::endl;
    }

protected:

    /**
     * Horizontal error between drone and platform using vision system
     */

    double   _horizontaErr;
    
    /**
     * Vertical error between drone and platform using vision system
     * and ultrasonic sensor 
     */
    
    double   _verticalErr;
    
    /**
     * Threshold value for the horizontal error to allow the hold state
     */
    
    double   _tauHold;

    /**
     * Threshold value for the horizontal error to check if the platform is lost
     */

    double   _tauLost;

    /**
     * It seems not used
     */

    double   _tauErr;
    
    /**
     * It keeps how many times the _horizontaErr is under a given threshold (_tauLost)
     */

    int      _NHold;
    
    /**
     * It keeps how many times the _horizontaErr is above a given threshold (_tauLost)
     */

    int      _NLost;

    /**
     * It keeps how many times the drone is on place to accomplish the landing
     */

    int      _NComp;

     /**
     * The full state of the drone respect to the world frame
     */

    MavState _state;

    /**
     * The position setPoint to accomplish the landing 
     */

    MavState _setPoint;

    /**
     * Relative pose between drone and platform using vision system
     */

	MavState _VisionPose;

    /**
     * Relative z position and velocity of the drone respect to the platform
     */

	MavState _UltraInfo;

    /**
     * Pointer to the LandMachine class.
     */

    LandMachine* _contextL;

    /**
     * It checks if the drone is on place respect to the platform
     */

    bool _centered;

    /**
     * It checks if the drone is holding (horizontal error) the platform
     */

    bool _holding;

    /**
     * It checks if the drone has lost the platform
     */

    bool _lost;

	Parameters param;

};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the init state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class InitState : public AbstractLandState {

public:

	AbstractLandState* _nextHoveringState;
    
    /**
     * @brief Constructor of the derived class
     */

    InitState(LandMachine *context) :  AbstractLandState(context){
        setId();
    }

    /**
     * @brief It sets the ID to the INIT one.
     */

    void setId() override {

        _id = INIT;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for InitState sets the next state(hold)
     * of the state machine.
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the hovering state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class HoveringState : public AbstractLandState {

public:
	
	HoveringState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

	/**
     * @brief It sets the ID to the HOVERING one
     */

    void setId() override {

        _id = HOVERING;
    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for Hovering sets the next
     * state of the machine to the init one; the hovering() method has been performed.
     */
    
    void handle();


};


/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the hold state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class HoldState : public AbstractLandState {

public:
    
    /**
     * The hold state is linked with the ascending state
     */

    AbstractLandState* _nextAscState;
    
    /**
     * The hold state is linked with the descending state
     */

    AbstractLandState* _nextDesState;

    /**
     * The hold state is linked with the init state
     */

	AbstractLandState* _nextInitState;

    /**
     * @brief Constructor of the derived class
     */

    HoldState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the HOLD one
     */

    void setId() override {

        _id = HOLD;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for HoldState checks different boolean variables
     * (descValid, InitValid, asceValid, compValid) and based on those it sets the next
     * state of the machine
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the descending state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class DescState : public AbstractLandState {

public:

    /**
     * @brief Constructor of the derived class
     */

    DescState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the DESC one
     */

    void setId() override {

        _id = DESC;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for DescState sets the next
     * state of the machine to the hold one; the desc() method has been performed.
     */
    
    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the ascending state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class AsceState : public AbstractLandState {

public:

    /**
     * @brief Constructor of the derived class
     */

    AsceState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the ASCE one
     */

    void setId() override {

        _id = ASCE;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for DescState sets the next
     * state of the machine to the hold one; the asce() method has been performed.
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the RTo land state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class RToLandState : public AbstractLandState {

public:
    
    /**
     * Such state is linked with the comp state
     */

    AbstractLandState* _nextComState;
    
    /**
     * @brief Constructor of the derived class
     */

    RToLandState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the R2LA one
     */

    void setId() override {

        _id = R2LA;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for RToLandState, based on
     * some variables(_NComp nad _centered), sets the next state to the comp one
     * or to the hold one. 
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the compensation state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class CompState : public AbstractLandState {

public:
    
    /**
     * Such state is linked with the landing state
     */

    AbstractLandState* _nextLanState;
    
    /**
     * @brief Constructor of the derived class
     */

    CompState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the R2LA one
     */

    void setId() override {

        _id = COMP;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for CompState, based on
     * what is happening, sets the next state to the land one or to the asce one. 
     */

    void handle();
};

/** @ingroup Automatic
 * @brief Derived class of the AbstractLandState
 * Such class represents the landing state.
 * This state will be linked with the LandMachine class via
 * the protected member of the AbstractLandState class(_contextL)
 */

class LandState : public AbstractLandState {

public:
    
    /**
     * Such state is linked with the landing state
     */

    AbstractLandState* _restartState;

    /**
     * @brief Constructor of the derived class
     */

    LandState(LandMachine *context) : AbstractLandState(context) {
        setId();
    }

    /**
     * @brief It sets the ID to the LAND one
     */


    void setId() override {

        _id = LAND;

    }

    /**
     * @brief Such method allows to check the current state of the drone
     * and it allows to set the next state of the land state machine.
     * @details In this case, the handle() method for LandState checks 
     * different boolean variables and, based on those, it sets the next
     * state of the machine to the asce one or to the init one 
     */

    void handle();
};



#endif //MOCAP2MAV_STATESCLASSES_HPP
