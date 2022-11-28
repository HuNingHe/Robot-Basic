#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"
class WBC_Ctrl;
class LocomotionCtrlData;

class FSM_State_BalanceStand : public FSM_State {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FSM_State_BalanceStand(ControlFSMData* _controlFSMData);

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

 private:
  // Keep track of the control iterations
  int _iter = 0;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();

  WBC_Ctrl * _wbc_ctrl;
  LocomotionCtrlData * _wbc_data;
  double toe_radius = 0.02;
  Vec3<double> _ini_body_pos;
  Vec3<double> _ini_body_ori_rpy;
  Vec3<double> _ini_foot_pos[4];
  double _body_weight;
};

#endif  // FSM_STATE_BALANCESTAND_H
