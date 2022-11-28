/*============================= Recovery SitDOwn ==============================*/
/**
 * Added by EHL, 4DAGE, Zhuhai
 * developerleen@gmail.com
 */

#include "FSM_State_RecoverySitDown.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSM_State_RecoverySitDown::FSM_State_RecoverySitDown(ControlFSMData* _controlFSMData) : FSM_State(_controlFSMData, FSM_StateName::SIT_DOWN, "SIT_DOWN"){
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;
    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;

    zero_vec3.setZero();

    sit_jpos[0] << 0.55f, 1.21f, 0.0f;// LF
    sit_jpos[1] << -0.55f, 1.21f, 0.0f; // RF
    sit_jpos[2] << -0.55f, 1.21f, 0.0f; // RH
    sit_jpos[3] << 0.55f, 1.21f, 0.0f;  // LH
}

void FSM_State_RecoverySitDown::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;
    _state_iter = 0;

    // initial configuration, position
    for(size_t i(0); i < 4; ++i) {
        initial_jpos[i] = this->_data->_legController->datas[i].q;
    }

    double body_height = this->_data->_stateEstimator->getResult().position[2];

    if( !UpsideDown() ) { // Proper orientation
        if ( body_height > 0.2 ){
            printf("[Recovery SitDown] body height is %f; Sit Down...\n", body_height);
        }else{
            printf("[Recovery SitDown] body height is %f; already sit down\n", body_height);
        }
    }else{
        printf("[Recovery SitDown] UpsideDown (%d) !!!!!!!!!!!!!!!!!\n", UpsideDown() );
    }
    _motion_start_iter = 0;
}

bool FSM_State_RecoverySitDown::UpsideDown(){
    if(this->_data->_stateEstimator->getResult().rBody(2,2) < 0){
        return true;
    }
    return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
void FSM_State_RecoverySitDown::run() {
    SitDown(_state_iter - _motion_start_iter);
    ++_state_iter;
}

void FSM_State_RecoverySitDown::SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<double> & ini, const Vec3<double> & fin){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
        b = (float)curr_iter/(float)max_iter;
        a = 1.f - b;
    }

    Vec3<double> inter_pos = a * ini + b * fin;

    this->jointPDControl(leg, 1, inter_pos, zero_vec3);

    if(curr_iter == 0){
        printf("sitdown start, curr iter: %lu, state iter: %llu, motion start iter: %d\n", curr_iter, _state_iter, _motion_start_iter);
        printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    }
    if(curr_iter == max_iter){ 
        printf("sitdown end,  curr iter: %lu, state iter: %llu, motion start iter: %d\n", curr_iter, _state_iter, _motion_start_iter);
        printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    }
}

void FSM_State_RecoverySitDown::SitDown(const int & curr_iter){
    double body_height = this->_data->_stateEstimator->getResult().position[2];
    bool something_wrong(false);

    if( UpsideDown() || (body_height > 0.45 ) ) {
        something_wrong = true;
    }

    if( (curr_iter > floor(standup_ramp_iter*0.7) ) && something_wrong){
        // If body height is too low because of some reason
        // even after the stand up motion is almost over
        // (Can happen when E-Stop is engaged in the middle of Other state)
        for(size_t i(0); i < 4; ++i) {
            initial_jpos[i] = this->_data->_legController->datas[i].q;
        }
        _motion_start_iter = _state_iter+1;
        printf("[Recovery SitDown - Warning] body height is still too high (%f) or UpsideDown (%d); Folding legs \n", body_height, UpsideDown() );
    }else{  // normal sit down operation
        for(size_t leg(0); leg<4; ++leg){
            SetJPosInterPts(curr_iter, standup_ramp_iter, leg, initial_jpos[leg], sit_jpos[leg]);
        }
    }
}


/*!
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
FSM_StateName FSM_State_RecoverySitDown::checkTransition() {
    this->nextStateName = this->stateName;
    iter++;
    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode) {
        case K_SIT_DOWN:
          break;
        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            printf("SIT_DOWN -> PASSIVE\n");
            break;
        case K_RECOVERY_STAND:
          // Requested switch to joint PD control
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            printf("SIT_DOWN -> RECOVERY_STAND\n");
            break;
        default:
            std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                      << K_SIT_DOWN << " to "
                      << this->_data->controlParameters->control_mode << std::endl;
            break;
    }
    // Get the next state
    return this->nextStateName;
}

/*!
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
TransitionData FSM_State_RecoverySitDown::transition() {
  // Finish Transition
    switch (this->nextStateName) {
        case FSM_StateName::PASSIVE:  // normal
            this->transitionData.done = true;
            break;

        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;
        default:
            std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
            break;
    }
    // Return the transition data to the FSM
    return this->transitionData;
}
