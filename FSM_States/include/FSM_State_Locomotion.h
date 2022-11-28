#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include <MPC/include/MPCLocomotion.h>
#include "FSM_State.h"
//#include "MPC/include/RFMPCLocomotion.h"

class WBC_Ctrl;
class LocomotionCtrlData;

class FSM_State_Locomotion : public FSM_State {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit FSM_State_Locomotion(ControlFSMData* _controlFSMData);

    // Behavior to be carried out when entering a state
    void onEnter() override;

    // Run the normal behavior for the state
    void run() override;

    // Checks for any transition triggers
    FSM_StateName checkTransition() override;

    // Manages state specific transitions
    TransitionData transition() override;

    // Behavior to be carried out when exiting a state
    void onExit();

private:
    // Keep track of the control iterations
    int iter = 0;
    MPCLocomotion* cMPC;
//    RFMPCLocomotion* rfMPC;
    WBC_Ctrl * _wbc_ctrl;
    LocomotionCtrlData * _wbc_data;
    // Parses contact specific controls to the leg controller
    void LocomotionControlStep();
    bool locomotionSafe();
};

#endif  // FSM_STATE_LOCOMOTION_H
