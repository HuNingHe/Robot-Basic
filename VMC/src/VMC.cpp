/*!
 * @file: VMC.cpp
 * @authors: HuNing-He
 * @date: 2022-11-25
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "VMC/include/VMC.h"
#include "qpOASES.hpp"

void VMC::update_command() {
    auto des_command = control_data->_desiredStateCommand;
    des_com_pos(2) = 0.29;
    des_com_vel(2) = 0;

    double filter = 0.01;
    des_angular_vel(2) = -1 * des_command->rightStickAnalog[0];
    des_angular_vel(1) = 1 * des_command->rightStickAnalog[1];
    des_angular_vel(0) = -1.0 * des_command->leftStickAnalog[0];

    double x_vel_cmd_new = -1.2 * des_command->leftStickAnalog[1];
    double y_vel_cmd_new = -1.0 * des_command->leftStickAnalog[0];
    des_com_vel(0) = des_com_vel(0) * (1 - filter) + x_vel_cmd_new * filter;
    des_com_vel(1) = des_com_vel(1) * (1 - filter) + y_vel_cmd_new * filter;

    if (des_command->a) {
        gait_type = GaitType::STAND;
    } else if (des_command->b) {
        gait_type = GaitType::TROT;
    }
}

void VMC::run() {
    double dt = control_data->_gaitScheduler->dt;
    auto leg_ctr = control_data->_legController;
    auto state_est = control_data->_stateEstimator->getResult();
    auto gait = control_data->_gaitScheduler;
    gait->step();

    static Vec3<double> stand_des_com_pos = Vec3<double>::Zero();
    static RotMat<double> stand_des_orientation = RotMat<double>::Identity();
    if (((gait_type == GaitType::STAND) && gait->getCurrentGaitType() != GaitType::STAND) || first_run) {
        stand_des_com_pos << state_est.position[0], state_est.position[1], 0.29;
        stand_des_orientation = state_est.orientation.matrix();
        num_contacts = 4;
    }

    gait->changeGaitType(gait_type);

    Vec4<double> swingTimes = gait->getSwingDuration();
    Vec4<double> stanceTimes = gait->getStanceDuration();
    static Vec4<double> swingTimeRemaining = Vec4<double>::Zero();
    for (int i = 0; i < 4; i++) {
        if (first_swing[i]) {
            swingTimeRemaining[i] = swingTimes[i];
        } else {
            swingTimeRemaining[i] -= dt;
        }
        swing_traj[i].setHeight(0.05);

        double stance_time = stanceTimes[i];

        Vec3<double> Pf = (pYawCorrected + des_com_vel * swingTimeRemaining[i]);

        double p_rel_max = 0.32f;

        double pfx_rel = state_est.vWorld[0] * 0.5 * stance_time +
            0.03f * (state_est.vWorld[0] - v_des_world[0]) +
            (0.5f * state_est.position[2] / 9.81) * (state_est.vWorld[1] * _yaw_turn_rate);

        double pfy_rel = state_est.vWorld[1] * 0.5 * stance_time * dtMPC +
            0.03f * (state_est.vWorld[1] - v_des_world[1]) +
            (0.5f * state_est.position[2] / 9.81) * (-state_est.vWorld[0] * _yaw_turn_rate);
        pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] += pfx_rel;
        Pf[1] += pfy_rel;
        Pf[2] = 0;
        swing_traj[i].setFinalPosition(Pf);
    }

    for (int foot = 0; foot < 4; foot++) {
        double contactState = contactStates[foot];
        double swingState = swingStates[foot];
        if (swingState > 0) {
            if (firstSwing[foot]) {
                firstSwing[foot] = false;
                swing_traj[foot].setInitialPosition(pFoot[foot]);
            }

            swing_traj[foot].computeSwingTrajectoryCycloid(swingState, swingTimes[foot]);

            Vec3<double> pDesFootWorld = swing_traj[foot].getPosition();
            Vec3<double> vDesFootWorld = swing_traj[foot].getVelocity();
            Vec3<double> pDesLeg = state_est.rBody.transpose() * (pDesFootWorld - state_est.position) - data._model->quadruped->hipLocationInKinematic.row(foot).transpose();
            Vec3<double> vDesLeg = state_est.rBody.transpose() * (vDesFootWorld - state_est.vWorld);

            leg_ctr->commands[foot].pDes = pDesLeg;
            leg_ctr->commands[foot].vDes = vDesLeg;
            leg_ctr->commands[foot].kpCartesian = Kp;
            leg_ctr->commands[foot].kdCartesian = Kd;
        } else{ // foot is in stance
            firstSwing[foot] = true;

            Vec3<double> pDesFootWorld = swing_traj[foot].getPosition();
            Vec3<double> vDesFootWorld = swing_traj[foot].getVelocity();
            Vec3<double> pDesLeg = state_est.rBody.transpose() * (pDesFootWorld - state_est.position) - data._model->quadruped->hipLocationInKinematic.row(foot).transpose();
            Vec3<double> vDesLeg = state_est.rBody.transpose() * (vDesFootWorld - state_est.vWorld);

            leg_ctr->commands[foot].pDes = pDesLeg;
            leg_ctr->commands[foot].vDes = vDesLeg;
            leg_ctr->commands[foot].kpCartesian = 0 * Kp_stance;
            leg_ctr->commands[foot].kdCartesian = Kd_stance;
            se_contactState[foot] = contactState;
        }
    }
}

VMC::VMC(ControlFSMData *data) {
    control_data = data;
    mass = control_data->_model->quadruped->body_mass;
    body_inertial = control_data->_model->quadruped->body_inertial;

    for (int i = 0; i < 4; i++) {
        contact_plane[i].normal << 0, 0, 1;
        contact_plane[i].base1 << 1, 0, 0;
        contact_plane[i].base2 << 0, 1, 0;
        first_swing[i] = true;
    }

    first_run = true;

    Eigen::Matrix<double, 6, 1> weight;
    weight << 5, 5, 10, 10, 10, 10;
    S.diagonal() = weight;
    alpha = 0.01;
    beta = 0.001;
    mu = 0.8;

    W << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;

    Kp_sw << 400, 0, 0,
             0, 400, 0,
             0, 0, 400;

    Kd_sw << 60, 0, 0,
             0, 60, 0,
             0, 0, 60;

    des_rpy.setZero();
    des_com_vel = Vec3<double>::Zero();
    des_com_pos = Vec3<double>::Zero();
    des_angular_vel = Vec3<double>::Zero();;
    des_orientation.setIdentity();
    gait_type = GaitType::STAND;
    num_contacts = 0;
}

void VMC::initialize() {
    for (bool & i : first_swing) i = true;
    first_run = true;
}
