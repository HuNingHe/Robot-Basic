#ifndef FSM_State_H
#define FSM_State_H

#include <cstdio>

#include "ControlFSMData.h"
#include "TransitionData.h"
#include "GaitScheduler.h"

//#include <BalanceController/BalanceController.hpp>

// Normal robot states
const unsigned int K_PASSIVE = 0;
const unsigned int K_STAND_UP = 1;
const unsigned int K_SIT_DOWN = 2;
const unsigned int K_BALANCE_STAND = 3;
const unsigned int K_LOCOMOTION = 4;
const unsigned int K_RECOVERY_STAND = 5;

/**
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
    INVALID,
    PASSIVE,
    STAND_UP,
    BALANCE_STAND,
    LOCOMOTION,
    RECOVERY_STAND,
    SIT_DOWN
};

class FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Generic constructor for all states
    FSM_State(ControlFSMData* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn);

    // Behavior to be carried out when entering a state
    virtual void onEnter() = 0;

    // Run the normal behavior for the state
    virtual void run() = 0;

    // Manages state specific transitions
    virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

    // Runs the transition behaviors and returns true when done transitioning
    virtual TransitionData transition() { return transitionData; }

    // Behavior to be carried out when exiting a state
    virtual void onExit() = 0;

    void jointPDControl(int leg, int flag, Vec3<double> qDes, Vec3<double> qdDes);

    void runBalanceController();

    void turnOnAllSafetyChecks();
    void turnOffAllSafetyChecks();

    // Holds all of the relevant control data
    ControlFSMData* _data;

    // FSM State info
    FSM_StateName stateName;      // enumerated name of the current state
    FSM_StateName nextStateName;  // enumerated name of the next state
    std::string stateString;      // state name string

    // Transition parameters
    double transitionDuration;  // transition duration time
    double tStartTransition;    // time transition starts
    TransitionData transitionData;

    // Pre controls safety checks
    bool checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    bool checkPDesFoot = false;          // do not command footsetps too far
    bool checkForceFeedForward = false;  // do not command huge forces
    bool checkLegSingularity = false;    // do not let leg

    // Leg controller command placeholders for the whole robot (3x4 matrices)
    Mat34<double> jointFeedForwardTorques;  // feed forward joint torques
    Mat34<double> jointPositions;           // joint angle positions
    Mat34<double> jointVelocities;          // joint angular velocities
    Mat34<double> footFeedForwardForces;    // feedforward forces at the feet
    Mat34<double> footPositions;            // cartesian foot positions
    Mat34<double> footVelocities;           // cartesian foot velocities

    // Footstep locations for next step
    Mat34<double> footstepLocations;

private:
    // Create the cartesian P gain matrix
    Mat3<double> kpMat;

    // Create the cartesian D gain matrix
    Mat3<double> kdMat;
};

#endif  // FSM_State_H
