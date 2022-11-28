#include "FSM_State.h"
/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
FSM_State::FSM_State(ControlFSMData* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn)
    : _data(_controlFSMData),
      stateName(stateNameIn),
      stateString(stateStringIn) {
    transitionData.zero();
    std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn << std::endl;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
void FSM_State::jointPDControl(int leg, int flag, Vec3<double> qDes, Vec3<double> qdDes) {
    if(flag == 0) {
        kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80; // 80
        kdMat << 1.5, 0, 0, 0, 1.5, 0, 0, 0, 1.5; // 0.5
    } else{
        kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80; // 80
        kdMat << 0.5, 0, 0, 0, 0.5, 0, 0, 0, 0; // 0.5 对于期望速度以及期望位置为零的关节Kd对应位置设置为零, 否则出现严重抖动, 因为速度很容易波动
    }
    _data->_legController->commands[leg].kpJoint = kpMat;
    _data->_legController->commands[leg].kdJoint = kdMat;

    _data->_legController->commands[leg].qDes = qDes;
    _data->_legController->commands[leg].qdDes = qdDes;
}

/*
void FSM_State::runBalanceController() {
    double minForce = 25;
    double maxForce = 500;
    double contactStateScheduled[4];  // = {1, 1, 1, 1};
    for (int i = 0; i < 4; i++) {
        contactStateScheduled[i] = _data->_gaitScheduler->gaitData.contactStateScheduled(i);
    }

    double minForces[4];  // = {minForce, minForce, minForce, minForce};
    double maxForces[4];  // = {maxForce, maxForce, maxForce, maxForce};
    for (int leg = 0; leg < 4; leg++) {
        minForces[leg] = contactStateScheduled[leg] * minForce;
        maxForces[leg] = contactStateScheduled[leg] * maxForce;
    }

    double COM_weights_stance[3] = {1, 1, 10};
    double Base_weights_stance[3] = {20, 10, 10};
    double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3], omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

    for (int i = 0; i < 4; i++) {
        se_xfb[i] = (double)_data->_stateEstimator->getResult().orientation(i);
    }

    for (int i = 0; i < 3; i++) {
        rpy[i] = 0;  //(double)_data->_stateEstimator->getResult().rpy(i);
        p_des[i] = (double)_data->_stateEstimator->getResult().position(i);
        p_act[i] = (double)_data->_stateEstimator->getResult().position(i);
        omegaDes[i] = 0;  //(double)_data->_stateEstimator->getResult().omegaBody(i);
        v_act[i] = (double)_data->_stateEstimator->getResult().vBody(i);
        v_des[i] = (double)_data->_stateEstimator->getResult().vBody(i);

        se_xfb[4 + i] = (double)_data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = (double)_data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = (double)_data->_stateEstimator->getResult().vBody(i);

        // Set the translational and orientation gains
        kpCOM[i] = (double)_data->controlParameters->kpCOM(i);
        kdCOM[i] = (double)_data->controlParameters->kdCOM(i);
        kpBase[i] = (double)_data->controlParameters->kpBase(i);
        kdBase[i] = (double)_data->controlParameters->kdBase(i);
    }

    Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;
    // Get the foot locations relative to COM
    for (int leg = 0; leg < 4; leg++) {
        _data->_legController->kine->Forward(_data->_legController->datas[leg].q, pFeetVec);

        pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  (_data->_model->quadruped->hipLocationInKinematic.row(0) + _data->_legController->datas[leg].p);

        pFeet[leg * 3] = (double)pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
    }
  
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.5);
    balanceController.set_mass(_data->_model->quadruped->body_mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act, O_err, 0.0);

    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    // Copy the results to the feed forward forces
    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1], fOpt[leg * 3 + 2];
    }
}
*/

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
void FSM_State::turnOnAllSafetyChecks() {
    // Pre controls safety checks
    checkSafeOrientation = true;  // check roll and pitch

    // Post control safety checks
    checkPDesFoot = true;          // do not command footsetps too far
    checkForceFeedForward = true;  // do not command huge forces
    checkLegSingularity = true;    // do not let leg
}

void FSM_State::turnOffAllSafetyChecks() {
    // Pre controls safety checks
    checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    checkPDesFoot = false;          // do not command footsetps too far
    checkForceFeedForward = false;  // do not command huge forces
    checkLegSingularity = false;    // do not let leg
}