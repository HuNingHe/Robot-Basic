/*============================ Locomotion =============================*/
/*!
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include "Timer.h"
/*!
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_Locomotion::FSM_State_Locomotion(ControlFSMData *_controlFSMData) : FSM_State(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION") {
    cMPC = new MPCLocomotion(0.001, 24, _controlFSMData);
//    rfMPC = new RFMPCLocomotion(0.001, 60, _controlFSMData);
    this->turnOnAllSafetyChecks();
    // Turn off Foot pos command since it is set in WBC as operational task
    this->checkPDesFoot = false;

    // Initialize GRF and footstep locations to 0s
    this->footFeedForwardForces = Mat34<double>::Zero();
    this->footstepLocations = Mat34<double>::Zero();
    _wbc_ctrl = new LocomotionCtrl(_controlFSMData->_model);
    _wbc_data = new LocomotionCtrlData();
}

void FSM_State_Locomotion::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;
    cMPC->initialize();
//    rfMPC->initialize();
    // Reset the transition data
    this->transitionData.zero();
    printf("[FSM LOCOMOTION] On Enter\n");
}

/*!
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_Locomotion::run() {
    LocomotionControlStep();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_Locomotion::checkTransition() {
    // Get the next state
    iter++;

    // Switch FSM control mode
    if (locomotionSafe()) {
        switch ((int) this->_data->controlParameters->control_mode) {
            case K_LOCOMOTION:
                break;

            case K_BALANCE_STAND:
                // Requested change to BALANCE_STAND
                this->nextStateName = FSM_StateName::BALANCE_STAND;

                // Transition time is immediate
                this->transitionDuration = 0.0;

                break;

            case K_PASSIVE:
                // Requested change to BALANCE_STAND
                this->nextStateName = FSM_StateName::PASSIVE;

                // Transition time is immediate
                this->transitionDuration = 0.0;

                break;

            case K_STAND_UP:
                this->nextStateName = FSM_StateName::STAND_UP;
                this->transitionDuration = 0.;
                break;

            case K_RECOVERY_STAND:
                this->nextStateName = FSM_StateName::RECOVERY_STAND;
                this->transitionDuration = 0.;
                break;

            default:
                std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                          << K_LOCOMOTION << " to "
                          << this->_data->controlParameters->control_mode << std::endl;
                break;
        }
    } else {
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        _data->controlParameters->control_mode = K_RECOVERY_STAND;
    }
    // Return the next state name to the FSM
    return this->nextStateName;
}

/*!
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 * @return true if transition is complete
 */
TransitionData FSM_State_Locomotion::transition(){
    // Switch FSM control mode
    switch (this->nextStateName) {
        case FSM_StateName::BALANCE_STAND:
            LocomotionControlStep();
            iter++;
            if (iter >= this->transitionDuration * 1000) {
                this->transitionData.done = true;
            } else {
                this->transitionData.done = false;
            }
            break;

        case FSM_StateName::PASSIVE:
            this->turnOffAllSafetyChecks();
            this->transitionData.done = true;
            break;

        case FSM_StateName::STAND_UP:
            this->transitionData.done = true;
            break;

        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;

        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition"
                      << std::endl;
            break;
    }
    // Return the transition data to the FSM
    return this->transitionData;
}

bool FSM_State_Locomotion::locomotionSafe() {
    auto &seResult = this->_data->_stateEstimator->getResult();

    const double max_roll = 40 * M_PI / 180;
    const double max_pitch = 40 * M_PI / 180;

    if (std::fabs(seResult.rpy[0]) > max_roll) {
        printf("Unsafe locomotion: roll is %.3f rad (max %.3f)\n", seResult.rpy[0], max_roll);
        return false;
    }

    if (std::fabs(seResult.rpy[1]) > max_pitch) {
        printf("Unsafe locomotion: pitch is %.3f rad (max %.3f)\n", seResult.rpy[1], max_pitch);
        return false;
    }

    for (int leg = 0; leg < 4; leg++) {
        auto p_leg = this->_data->_legController->datas[leg].p;
        if (p_leg[2] > 0) {
            printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
            return false;
        }

        if (std::fabs(p_leg[1]) > 0.18) {
            printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
            return false;
        }

        auto v_leg = this->_data->_legController->datas[leg].v.norm();
        if (std::fabs(v_leg) > 9.) { // 9
            printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
            return false;
        }
    }
    return true;
}

/*!
 * Cleans up the state information on exiting the state.
 */
void FSM_State_Locomotion::onExit() {
    // Nothing to clean up when exiting
    iter = 0;
}

/*!
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
void FSM_State_Locomotion::LocomotionControlStep() {
//    Timer solveTimer;
        cMPC->run(*_data);
//    printf("MPC Solve time %f ms\n", solveTimer.getMs());

    if (this->_data->controlParameters->use_wbc) {
        _wbc_data->pBody_des = cMPC->pBody_des;
        _wbc_data->vBody_des = cMPC->vBody_des;
        _wbc_data->aBody_des = cMPC->aBody_des;

        _wbc_data->pBody_RPY_des = cMPC->pBody_RPY_des;
        _wbc_data->vBody_Ori_des = cMPC->vBody_Ori_des;

        for (size_t i(0); i < 4; ++i) {
            _wbc_data->pFoot_des[i] = cMPC->pFoot_des[i];
            _wbc_data->vFoot_des[i] = cMPC->vFoot_des[i];
            _wbc_data->aFoot_des[i] = cMPC->aFoot_des[i];
            _wbc_data->Fr_des[i] = cMPC->Fr_des[i];
        }
        _wbc_data->contact_state = cMPC->contact_state;
        _wbc_ctrl->run(_wbc_data, *this->_data);
    }
}
