#ifndef FSM_STATE_RECOVERY_STANDUP_H
#define FSM_STATE_RECOVERY_STANDUP_H

#include "FSM_State.h"
/*!
 * 机器人倒地后恢复站立的算法
 */
class FSM_State_RecoveryStand : public FSM_State {
public:
    explicit FSM_State_RecoveryStand(ControlFSMData* _controlFSMData);

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
    int _motion_start_iter = 0;

    static constexpr int StandUpFlag = 0;
    static constexpr int FoldLegsFlag = 1;
    static constexpr int RollOverFlag = 2;

    unsigned long long _state_iter;
    int _flag = FoldLegsFlag;

    // JPos
    Vec3<double> fold_jpos[4];
    Vec3<double> stand_jpos[4];
    Vec3<double> rolling_jpos[4];
    Vec3<double> initial_jpos[4];
    Vec3<double> zero_vec3;

    // 0.5 kHz
    const int rollover_ramp_iter = 100; // 100
    const int rollover_settle_iter = 150; // 100

    const int fold_ramp_iter = 100;  // 100
    const int fold_settle_iter = 600;  // 800

    const int standup_ramp_iter = 600; // 400

    void RollOver(const unsigned long long & iter);
    void StandUp(const unsigned long long & iter);
    void FoldLegs(const unsigned long long & iter);

    bool UpsideDown();
    void SetJPosInterPts(const size_t & curr_iter, size_t max_iter, int leg, const Vec3<double> & ini, const Vec3<double> & fin);
};

#endif  // FSM_STATE_RECOVERY_STANDUP_H
