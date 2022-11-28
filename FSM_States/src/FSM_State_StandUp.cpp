/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"

/*!
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_StandUp::FSM_State_StandUp(ControlFSMData *_controlFSMData)
        : FSM_State(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"), _ini_foot_pos(4) {
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

void FSM_State_StandUp::onEnter() {
    printf("Enter Stand Up state");
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;

    for (size_t leg(0); leg < 4; ++leg) {
        _ini_foot_pos[leg] = this->_data->_legController->datas[leg].p;
    }
}

/*!
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_StandUp::run() {
    double hMax = _data->controlParameters->body_height;
    double progress = 2 * iter * this->_data->_gaitScheduler->dt;
    if (progress <= 1){
        std::cout << "Standing Up, desire body height is:" << progress * (-hMax) + (1. - progress) * _ini_foot_pos[0][2] <<std::endl;
    } else{
        progress = 1.;
    }

    for (int i = 0; i < 4; i++) {
//        this->_data->_legController->commands[i].zero();
        this->_data->_legController->commands[i].kpCartesian = _data->controlParameters->kp_stand_up.asDiagonal();
        this->_data->_legController->commands[i].kdCartesian = _data->controlParameters->kd_stand_up.asDiagonal();

        this->_data->_legController->commands[i].pDes = _ini_foot_pos[i];
        this->_data->_legController->commands[i].pDes[2] = progress * (-hMax) + (1. - progress) * _ini_foot_pos[i][2];
    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_StandUp::checkTransition() {
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch (this->_data->controlParameters->control_mode) {
        case K_STAND_UP:
            break;

        case K_BALANCE_STAND:
            this->nextStateName = FSM_StateName::BALANCE_STAND;
            printf("STAND_UP -> BALANCE_STAND\n");
            break;

        case K_LOCOMOTION:
            this->nextStateName = FSM_StateName::LOCOMOTION;
            printf("STAND_UP -> LOCOMOTION\n");
            break;

        case K_PASSIVE:  // normal c
            this->nextStateName = FSM_StateName::PASSIVE;
            printf("STAND_UP -> PASSIVE\n");
            break;

        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_STAND_UP << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
    }

    // Get the next state
    return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_StandUp::transition() {
    // Finish Transition
    switch (this->nextStateName) {
        case FSM_StateName::PASSIVE:  // normal
            this->transitionData.done = true;
            break;

        case FSM_StateName::BALANCE_STAND:
            this->transitionData.done = true;
            break;

        case FSM_StateName::LOCOMOTION:
            this->transitionData.done = true;
            break;

        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
            break;
    }
    // Return the transition data to the FSM
    return this->transitionData;
}
