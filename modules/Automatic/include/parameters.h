//
// Created by andreanistico on 21/02/17.
//

#ifndef MOCAP2MAV_PARAMETERS_H
#define MOCAP2MAV_PARAMETERS_H

/** @ingroup Automatic
 *  List of parameters used for the Landing procedure
 */

namespace params_automatic {

    /**
     * Offset between baricenter of the drone and the ultrasonic sensor 
     */

	static const double OffsetUltraSensor = 0.1;

    /**
     * Number of consecutive frames in which tracking is considered valid 
     */

    static const int    NFramesHold     = 120;

    /**
     * Number of consecutive frames in which tracking is considered not valid 
     */

    static const int    NFramesLost     = 70;

    /**
     * Number of consecutive frames in which tracking is considered valid
     * and robot is ready for compensation
     */

    static const int    NFramesComp     = 20;

    /**
     * Platform dimension 
     */

    static const double platformLenght  = 0.65;

    /**
     * Max altitude for landing procedure 
     */

    static const double zMax            = 1.2;

    /**
     * Minimum altitude for landing procedure (before compensating)     
     */

    static double       zMin            = 0.4;

    /**
     * Proportional gain times platform velocity 
     */

    static const double KpHoldV          = 1;

    /**
     * Proportional gain times horizontal error 
     */

    static const double KpHold           = 0.7;//0.5;//0.78; 

    /**
     * Differential gain times horizontal error 
     */

    static const double KdHold           = 0.00;

    /**
     * Integral gain times integral horizontal error 
     */

    static const double KiHold          = 0.001; //0.1

    /**
     * Proportional gain for velocity tracking 
     */

    static const double KPCompV          = 0.1;

    /**
     * Integral clamping values 
     */

    static const double maxIntValue     = 1;
    
    /**
     * Integral clamping values
     */

    static const double minIntValue     = -maxIntValue;

    /**
     * Max total PID output 
     */

    static const double maxOutput       = 1.5;

}

#endif //MOCAP2MAV_PARAMETERS_H