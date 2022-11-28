#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"


class FSM_State_Passive : public FSM_State {
public:
    explicit FSM_State_Passive(ControlFSMData* _controlFSMData);

    // Behavior to be carried out when entering a state
    void onEnter() override;

    // Run the normal behavior for the state
    void run() override;

    // Checks for any transition triggers
    FSM_StateName checkTransition() override;

    // Manages state specific transitions
    TransitionData transition() override;

    // Behavior to be carried out when exiting a state
    void onExit() override;

    TransitionData testTransition();

private:
    // Keep track of the control iterations
    int iter = 0;
};

#endif  // FSM_STATE_PASSIVE_H
