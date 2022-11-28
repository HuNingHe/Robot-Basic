#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
#include "RobotModel.h"
#include "GamepadCommand.h"
#include "ControlFSMData.h"
#include "SafetyChecker.h"
#include "FSM_State.h"
#include "FSM_State_BalanceStand.h"
#include "FSM_State_Locomotion.h"
#include "FSM_State_Passive.h"
#include "FSM_State_StandUp.h"
#include "FSM_State_RecoveryStand.h"
#include "FSM_State_RecoverySitDown.h"

class GaitController;

enum class FSM_OperatingMode {
    NORMAL, TRANSITIONING, ESTOP
};

struct FSM_StatesList {
    FSM_State *invalid;
    FSM_State_Passive *passive;
    FSM_State_StandUp *standUp;
    FSM_State_BalanceStand *balanceStand;
    FSM_State_Locomotion *locomotion;
    FSM_State_RecoveryStand *recoveryStand;
    FSM_State_RecoverySitDown *recoverySitDown;
};

class ControlFSM {
public:
    ControlFSM(RobotModel *model,
               StateEstimatorContainer *_stateEstimator,
               LegController *_legController,
               GaitController *_gaitScheduler,
               GamepadCommand *_desiredStateCommand,
               RobotControlParameters *controlParameters);
    ~ControlFSM();
    // Initializes the Control FSM instance
    void initialize();

    // Runs the FSM logic and handles the state transitions and normal runs
    void runFSM();

    // This will be removed and put into the SafetyChecker class
    FSM_OperatingMode safetyPreCheck();

    //
    FSM_OperatingMode safetyPostCheck();

    // Gets the next FSM_State from the list of created states when requested
    FSM_State *getNextState(FSM_StateName stateName);

    // Contains all of the control related data
    ControlFSMData data;

    // FSM state information
    FSM_StatesList statesList;  // holds all of the FSM States
    FSM_State *currentState;    // current FSM state
    FSM_State *nextState;       // next FSM state
    FSM_StateName nextStateName;   // next FSM state name

    // Checks all of the inputs and commands for safety
    SafetyChecker *safetyChecker;
    TransitionData transitionData;

private:
    // Operating mode of the FSM
    FSM_OperatingMode operatingMode;

    unsigned long long int iter = 0;
};

#endif  // CONTROLFSM_H
