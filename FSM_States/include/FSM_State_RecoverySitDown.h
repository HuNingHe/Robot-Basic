/*
 * Added by EHL, 4DAGE, Zhuhai
 * developerleen@gmail.com
*/

#ifndef FSM_STATE_RECOVERY_SIT_DOWN_H
#define FSM_STATE_RECOVERY_SIT_DOWN_H

#include "FSM_State.h"

class FSM_State_RecoverySitDown : public FSM_State {
 public:
  explicit FSM_State_RecoverySitDown(ControlFSMData* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition() override;

  // Manages state specific transitions
  TransitionData transition() override;

  // Behavior to be carried out when exiting a state
  void onExit() override{};

 private:
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  unsigned long long _state_iter;

  // JPos
  Vec3<double> sit_jpos[4];
  Vec3<double> initial_jpos[4];
  Vec3<double> zero_vec3;

  const int standup_ramp_iter = 1000;

  void SitDown(const int & iter);

  bool UpsideDown();
  void SetJPosInterPts(const size_t & curr_iter, size_t max_iter, int leg, const Vec3<double> & ini, const Vec3<double> & fin);

};

#endif  // FSM_STATE_RECOVERY_SIT_DOWN_H
