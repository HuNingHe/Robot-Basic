#ifndef FSM_STATE_STANDUP_H
#define FSM_STATE_STANDUP_H

#include "FSM_State.h"

/**
 *
 */
class FSM_State_StandUp : public FSM_State {
public:
    explicit FSM_State_StandUp(ControlFSMData* _controlFSMData);

    // Behavior to be carried out when entering a state
    void onEnter() override;

    // Run the normal behavior for the state
    void run() override;

    // Checks for any transition triggers
    FSM_StateName checkTransition() override;

    // Manages state specific transitions
    TransitionData transition() override;

    // Behavior to be carried out when exiting a state
    void onExit() override{}

private:
    // Keep track of the control iterations
    int iter = 0;
    std::vector< Vec3<double> > _ini_foot_pos;
};

#endif  // FSM_STATE_STANDUP_H
