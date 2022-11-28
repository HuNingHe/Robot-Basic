/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"
#include <WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include "save_data_to_file.h"
/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_BalanceStand::FSM_State_BalanceStand(ControlFSMData *_controlFSMData) : FSM_State(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND") {
    // Set the pre controls safety check
    turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    checkPDesFoot = false;

    footFeedForwardForces = Mat34<double>::Zero();
    _wbc_ctrl = new LocomotionCtrl(_controlFSMData->_model);
    _wbc_data = new LocomotionCtrlData();
    _wbc_ctrl->setFloatingBaseWeight(1000.);//1000
    _body_weight = this->_data->_model->quadruped->body_mass * 9.81;
}

void FSM_State_BalanceStand::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Always set the gait to be standing in this state
//    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;

    _ini_body_pos = (this->_data->_stateEstimator->getResult()).position;

    if (_ini_body_pos[2] < 0.2) {
        _ini_body_pos[2] = 0.28;
    }
    _ini_body_ori_rpy = (this->_data->_stateEstimator->getResult()).rpy;
    for (int i = 0; i < 4; ++i) {
        _data->_model->getPos(i+1, _ini_foot_pos[i]);
    }
}

/*!
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_BalanceStand::run() {
    Vec4<double> contactState;
    contactState << 0.5, 0.5, 0.5, 0.5;
    this->_data->_stateEstimator->setContactPhase(contactState);
    BalanceStandStep();
}

/*!
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_BalanceStand::checkTransition() {
    _iter++;
    // Switch FSM control mode
    switch ((int) this->_data->controlParameters->control_mode) {
        case K_BALANCE_STAND:
            break;

        case K_LOCOMOTION:
            // Requested change to balance stand
            this->nextStateName = FSM_StateName::LOCOMOTION;
            // Transition instantaneously to locomotion state on request
            this->transitionDuration = 0.0;
            // Set the next gait in the scheduler to
//            this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
            break;

        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            // Transition time is immediate
            this->transitionDuration = 0.0;
            break;

        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            // Transition time is immediate
            this->transitionDuration = 0.0;
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_BALANCE_STAND << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
            break;
    }
    // Return the next state name to the FSM
    return this->nextStateName;
}

/*！
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 * @return true if transition is complete
 */
TransitionData FSM_State_BalanceStand::transition() {
    // Switch FSM control mode
    switch (this->nextStateName) {
        case FSM_StateName::LOCOMOTION:
            BalanceStandStep();
            _iter++;
            if (_iter >= this->transitionDuration * 1000) {
                this->transitionData.done = true;
                printf("BALANCE_STAND -> LOCOMOTION\n");
            } else {
                this->transitionData.done = false;
            }
            break;

        case FSM_StateName::PASSIVE:
            this->turnOffAllSafetyChecks();
            this->transitionData.done = true;
            break;

        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            printf("BALANCE_STAND -> RECOVERY_STAND\n");
            break;

        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
            break;
    }
    // Return the transition data to the FSM
    return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
void FSM_State_BalanceStand::onExit() {
    _iter = 0;
}

/*!
 * Calculate the commands for the leg controllers for each of the feet.
 */
void FSM_State_BalanceStand::BalanceStandStep() {
    double maxRoll = 0.45;
    double maxPitch = 0.22;
    double maxYaw = 0.4;

    _wbc_data->pBody_des = _ini_body_pos;
    _wbc_data->vBody_des.setZero();
    _wbc_data->aBody_des.setZero();

    _wbc_data->pBody_RPY_des = _ini_body_ori_rpy;
    // Orientation
    _wbc_data->pBody_RPY_des[0] = maxRoll * this->_data->_desiredStateCommand->leftStickAnalog[0];
    _wbc_data->pBody_RPY_des[1] = maxPitch * this->_data->_desiredStateCommand->leftStickAnalog[1];
    _wbc_data->pBody_RPY_des[2] = _ini_body_ori_rpy[2] - maxYaw * this->_data->_desiredStateCommand->rightStickAnalog[0];
    // Height
    _wbc_data->pBody_des[2] = _ini_body_pos[2] - 0.05 * this->_data->_desiredStateCommand->rightStickAnalog[1];
    _wbc_data->vBody_Ori_des.setZero();

    for (int i(0); i < 4; ++i) {
        _wbc_data->pFoot_des[i][2] = 0;
        _wbc_data->pFoot_des[i][0] = _ini_foot_pos[i][0];
        _wbc_data->pFoot_des[i][1] = _ini_foot_pos[i][1];

        _wbc_data->vFoot_des[i].setZero();
        _wbc_data->aFoot_des[i].setZero();
        _wbc_data->Fr_des[i].setZero();
        _wbc_data->Fr_des[i][2] = _body_weight / 4.;
        _wbc_data->contact_state[i] = 1;
    }

    /* data save for graph drawing
    Vec3<double> rpy = _data->_stateEstimator->getResult().rpy;
    Vec3<double> body_pos = _data->_stateEstimator->getResult().position;
    Vec3<double> p_foot;
    _data->_model->getPos(robot_link::LF_Foot, p_foot);
    print_vec3_to_file(rpy, "real_rpy.txt");
    print_vec3_to_file(body_pos, "real_pos.txt");
    print_vec3_to_file(_wbc_data->pBody_RPY_des, "des_rpy.txt");
    print_vec3_to_file(_wbc_data->pBody_des, "des_pos.txt");
    print_vec3_to_file(p_foot, "p_foot.txt");
    print_vec3_to_file(_wbc_data->pFoot_des[0], "p_foot_des.txt");
    */
    _wbc_ctrl->run(_wbc_data, *this->_data);
}