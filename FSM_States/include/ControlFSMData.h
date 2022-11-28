#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include <RobotControlParameters.h>
#include "RobotModel.h"
#include "GamepadCommand.h"
#include "GaitScheduler.h"
#include "LegController.h"
#include "StateEstimatorContainer.h"

struct ControlFSMData {
    RobotModel *_model;
    StateEstimatorContainer* _stateEstimator;
    LegController* _legController;
    GaitController* _gaitScheduler;
    GamepadCommand* _desiredStateCommand;
    RobotControlParameters* controlParameters;
//    RealRobotState
};

#endif  // CONTROLFSM_H