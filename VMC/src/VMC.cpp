/*!
 * @file: VMC.cpp
 * @authors: HuNing-He
 * @date: 2022-11-25
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "VMC/include/VMC.h"
#include "qpOASES.hpp"
#include "Orientation.h"
#include <cassert>

void VMC::update_command() {
    auto des_command = control_data->_desiredStateCommand;
    des_com_pos(2) = 0.29;
    des_com_vel(2) = 0;

    double filter = 0.05;
    Vec3<double> des_angular_vel_new = Vec3<double>::Zero();
    Vec3<double> des_com_vel_new = Vec3<double>::Zero();

    des_angular_vel_new(2) = -1 * des_command->rightStickAnalog[0];
    des_angular_vel_new(1) = 1 * des_command->rightStickAnalog[1];
    des_angular_vel_new(0) = -1.0 * des_command->leftStickAnalog[0];

    des_com_vel_new(0) = -1.2 * des_command->leftStickAnalog[1];
    des_com_vel_new(1) = -1.0 * des_command->leftStickAnalog[0];
    des_com_vel = des_com_vel * (1 - filter) + des_com_vel_new * filter;
    des_angular_vel = des_angular_vel * (1 - filter) + des_angular_vel_new * filter;

//    std::cout << "des angular vel:\n" << des_angular_vel_new << std::endl;
//    if (des_command->a) {
//        gait_type = GaitType::STAND;
//    } else if (des_command->b) {
//        gait_type = GaitType::TROT;
//    }
    gait_type = GaitType::TROT;
}

void VMC::run() {
    update_command();
    auto leg_ctr = control_data->_legController;
    auto state_est = control_data->_stateEstimator->getResult();
    auto parameters = control_data->controlParameters;

    mu = parameters->friction_mu;
    S.diagonal() = parameters->vmc_S;
    W(0, 0) = parameters->vmc_W(0);
    W(1, 1) = parameters->vmc_W(1);
    W(2, 2) = parameters->vmc_W(2);

    alpha = parameters->vmc_alpha;
    beta = parameters->vmc_beta;

    Kp_sw(0, 0) = parameters->kp_swing(0);
    Kp_sw(1, 1) = parameters->kp_swing(1);
    Kp_sw(2, 2) = parameters->kp_swing(2);

    Kd_sw(0, 0) = parameters->kd_swing(0);
    Kd_sw(1, 1) = parameters->kd_swing(1);
    Kd_sw(2, 2) = parameters->kd_swing(2);

    Kp_com(0, 0) = parameters->vmc_Kp_com(0);
    Kp_com(1, 1) = parameters->vmc_Kp_com(1);
    Kp_com(2, 2) = parameters->vmc_Kp_com(2);

    Kd_com(0, 0) = parameters->vmc_Kd_com(0);
    Kd_com(1, 1) = parameters->vmc_Kd_com(1);
    Kd_com(2, 2) = parameters->vmc_Kd_com(2);

    Kp_ori(0, 0) = parameters->vmc_Kp_ori(0);
    Kp_ori(1, 1) = parameters->vmc_Kp_ori(1);
    Kp_ori(2, 2) = parameters->vmc_Kp_ori(2);

    Kd_ori(0, 0) = parameters->vmc_Kd_ori(0);
    Kd_ori(1, 1) = parameters->vmc_Kd_ori(1);
    Kd_ori(2, 2) = parameters->vmc_Kd_ori(2);

    auto gait = control_data->_gaitScheduler;
    gait->step();

    if (((gait_type == GaitType::STAND) && gait->getCurrentGaitType() != GaitType::STAND) || first_run) {
        stand_des_com_pos << state_est.position[0], state_est.position[1], 0.29;
        stand_des_rpy = state_est.rpy;
    }

    // some first time initialization
    if(first_run) {
        des_com_pos[0] = state_est.position[0];
        des_com_pos[1] = state_est.position[1];
        des_com_pos[2] = 0.29;

        for(int i = 0; i < 4; i++) {
            swing_traj[i].setHeight(0.06);
            swing_traj[i].setInitialPosition(leg_ctr->datas[i].p);
            swing_traj[i].setFinalPosition(leg_ctr->datas[i].p);
        }
        first_run = false;
    }

    gait->changeGaitType(gait_type);

    Vec4<double> swing_period = gait->getSwingDuration();
    Vec4<double> stance_period = gait->getStanceDuration();
    Vec4<double> stance_sub_phase = gait->getStanceSubPhase();
    Vec4<double> swing_sub_phase = gait->getSwingSubPhase();
    Vec4<double> se_contact_state = Vec4<double>::Zero();

    contact_idx.clear();
    for (int i = 0; i < 4; i++) {
        swing_traj[i].setHeight(0.06);

        Vec3<double> Pf = Vec3<double>::Zero();

        double p_rel_max = 0.32f;
        double pfx_rel = state_est.vBody[0] * 0.5 * stance_period[i] + 0.04f * (state_est.vBody[0] - des_com_vel[0]) ;// +
//                         (0.5f * state_est.position[2] / 9.81) * (state_est.vBody[1] * des_angular_vel(2));

        double pfy_rel = state_est.vBody[1] * 0.5 * stance_period[i] + 0.04f * (state_est.vBody[1] - des_com_vel[1]);// +
//                         (0.5f * state_est.position[2] / 9.81) * (-state_est.vBody[0] * des_angular_vel(2));
        pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
        pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
        Pf[0] = pfx_rel;
        Pf[1] = pfy_rel;
        Pf[2] = -0.25;
        swing_traj[i].setFinalPosition(Pf);

        if (swing_sub_phase[i] <= 0) {
            contact_idx.push_back(i);
        }
    }

    compute_contact_force();

    for (int foot = 0; foot < 4; foot++) {
        if (swing_sub_phase[foot] > 0) {
            if (first_swing[foot]) {
                first_swing[foot] = false;
                swing_traj[foot].setInitialPosition(leg_ctr->datas[foot].p);
            }

            swing_traj[foot].computeSwingTrajectoryCycloid(swing_sub_phase[foot], swing_period[foot]);

            leg_ctr->commands[foot].pDes = swing_traj[foot].getPosition();
            leg_ctr->commands[foot].vDes = swing_traj[foot].getVelocity();
            leg_ctr->commands[foot].kpCartesian = Kp_sw;
            leg_ctr->commands[foot].kdCartesian = Kd_sw;
        } else { // stand leg
            first_swing[foot] = true;
            se_contact_state[foot] = stance_sub_phase[foot];
            leg_ctr->commands[foot].tauFeedForward = f_ff[foot];
        }
    }

    control_data->_stateEstimator->setContactPhase(se_contact_state);
    iter++;
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
        f_ff[i].setZero();
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

    Kp_com << 50, 0, 0,
              0, 50, 0,
              0, 0, 300;
    Kd_com << 10, 0, 0,
              0, 10, 0,
              0, 0, 30;
    Kp_ori << 60, 0, 0,
              0, 80, 0,
              0, 0, 100;
    Kd_ori << 20, 0, 0,
              0, 10, 0,
              0, 0, 10;

    stand_des_rpy.setZero();
    stand_des_com_pos.setZero();
    des_com_vel = Vec3<double>::Zero();
    des_com_pos = Vec3<double>::Zero();
    des_angular_vel = Vec3<double>::Zero();
    des_orientation.setIdentity();
    gait_type = GaitType::STAND;
    iter = 0;
    contact_idx.clear();
}

void VMC::initialize() {
    for (bool & i : first_swing) i = true;
    first_run = true;
    iter = 0;
}

void VMC::compute_contact_force() {
    using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    auto state_est = control_data->_stateEstimator->getResult();
    auto leg_ctr = control_data->_legController;
    auto qua = control_data->_model->quadruped;
    double dt = control_data->_gaitScheduler->dt;
    auto dyn_model = control_data->_model;
    DVec<double> _q = DVec<double>::Zero(dyn_model->num_qdot_real + 1);
    DVec<double> _qd = DVec<double>::Zero(dyn_model->num_qdot_real);

    _q[3] = state_est.orientation.x();
    _q[4] = state_est.orientation.y();
    _q[5] = state_est.orientation.z();
    _q[18] = state_est.orientation.w();
    _qd[3] = state_est.omegaBody[0]; // rbdl eulerZYX joint requires angular velocity in body frame
    _qd[4] = state_est.omegaBody[1];
    _qd[5] = state_est.omegaBody[2];
    for (int i = 0; i < 3; ++i) {
        _q[i] = state_est.position[i];// and floating body position in world frame
        _qd[i] = state_est.vWorld[i]; // and floating body vel in world frame
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            _q[3 * i + j + 6] = leg_ctr->datas[i].q[j];
            _qd[3 * i + j + 6] = leg_ctr->datas[i].qd[j];
        }
    }

    dyn_model->UpdateSystem(_q, _qd);
    DMat<double> J[4];

    dyn_model->getJacobiLinearPart(robot_link::LF_Foot, J[0]);
    dyn_model->getJacobiLinearPart(robot_link::RF_Foot, J[1]);
    dyn_model->getJacobiLinearPart(robot_link::RH_Foot, J[2]);
    dyn_model->getJacobiLinearPart(robot_link::LH_Foot, J[3]);

    int num_contacts = contact_idx.size();
    assert(num_contacts >= 2 && num_contacts <= 4);

    DMat<double> JC = DMat<double>::Zero(3 * num_contacts, dyn_model->num_qdot_real);
    DMat<double> SelectS = DMat<double>::Zero(12, dyn_model->num_qdot_real);
    SelectS.block(0, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();

    DMat<double> A = DMat<double>::Zero(6, 3 * num_contacts);
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    /*!
     * QP Matrices
     */
    RowMajorMatrixXd H = RowMajorMatrixXd::Zero(3 * num_contacts, 3 * num_contacts);
    DVec<double> g = DVec<double>::Zero(3 * num_contacts);
    RowMajorMatrixXd C = RowMajorMatrixXd::Zero(5 * num_contacts, 3 * num_contacts);
    DVec<double> dlb = DVec<double>::Zero(5 * num_contacts);
    DVec<double> dub = DVec<double>::Zero(5 * num_contacts);

    for (int i = 0; i < num_contacts; ++i) {
        JC.block(i * 3, 0, 3, dyn_model->num_qdot_real) = J[contact_idx[i]];
        Eigen::Matrix<double, 5, 3> single_C = Eigen::Matrix<double, 5, 3>::Zero();
        Eigen::Matrix<double, 5, 1> single_d = Eigen::Matrix<double, 5, 1>::Zero();

        single_C.row(0) = -contact_plane[contact_idx[i]].normal * mu + contact_plane[contact_idx[i]].base1;
        single_C.row(1) = -contact_plane[contact_idx[i]].normal * mu + contact_plane[contact_idx[i]].base2;
        single_C.row(2) = contact_plane[contact_idx[i]].normal * mu + contact_plane[contact_idx[i]].base1;
        single_C.row(3) = contact_plane[contact_idx[i]].normal * mu + contact_plane[contact_idx[i]].base2;
        single_C.row(4) = contact_plane[contact_idx[i]].normal;

        Vec3<double> foot_to_com = state_est.rBody * (leg_ctr->datas[contact_idx[i]].p + qua->hipLocationInKinematic.row(contact_idx[i]).transpose());
        A.block(0, i * 3, 3, 3) = Mat3<double>::Identity();
        A.block(3, i * 3, 3, 3) = Ori::crossMatrix(foot_to_com);

        C.block(i * 5, i * 3, 5, 3) = single_C;
        single_d << -5000, -5000, 0, 0, mass * 9.81 / 4;
        dlb.segment(i * 5, 5) = single_d;
        single_d << 0, 0, 5000, 5000, mass * 9.81;
        dub.segment(i * 5, 5) = single_d;
    }

    Mat3<double> inertial_world = state_est.rBody * body_inertial * state_est.rBody.transpose();

    Vec3<double> des_angular_vel_world = Vec3<double>(0, 0, 0);
    Vec3<double> des_com_vel_world = Vec3<double>(0, 0, 0);
    Vec3<double> des_com_pos_world = Vec3<double>(0, 0, 0);

    if (gait_type == GaitType::STAND) {
        des_com_pos_world = stand_des_com_pos;
//        stand_des_rpy += des_angular_vel * dt;
//        std::cout << "des_rpy:\n" << stand_des_rpy << std::endl;

        des_orientation = Ori::rpyToRotMat(stand_des_rpy);
    } else {
        des_com_pos_world(2) = 0.29;
        des_orientation.setIdentity();
        des_com_vel_world = state_est.rBody * des_com_vel;
    }

    RotMat<double> rot_err = des_orientation * state_est.rBody.transpose();
    Vec3<double> so3_err = Ori::SO3Log(rot_err);
    Vec3<double> ori_err = Kp_ori * so3_err + Kd_ori * (des_angular_vel_world - state_est.omegaWorld);
//    double com_h_err = Kp_com(2, 2) * (des_com_pos_world(2) - state_est.position[2]);
    double com_h_err = Kp_com(2, 2) * (des_com_pos_world(2) - state_est.position[2]);

    Vec3<double> com_err = Vec3<double>(0, 0, com_h_err) + Kd_com * (des_com_vel_world - state_est.vWorld);
//    std::cout << "rot error:\n" << rot_err << std::endl;
//    std::cout << "so3_err:\n" << so3_err << std::endl;

    b.segment(0, 3) = mass * com_err + mass * Vec3<double>(0, 0, 9.81);
    b.segment(3, 3) = inertial_world * ori_err;

    DVec<double> des_force = DVec<double>::Zero(3 * num_contacts);

    DMat<double> W_qp = DMat<double>::Zero(3 * num_contacts, 3 * num_contacts);
    double average_force = mass * 9.81 / num_contacts;

    for (int i = 0; i < num_contacts; ++i) {
        W_qp.block(i * 3, i * 3, 3, 3) = W;
        des_force.segment(i * 3, 3) = Vec3<double>(0, 0, average_force);
    }

    H = 2 * A.transpose() * S * A + 2 * (alpha + beta) * W_qp;
    g = -2 * A.transpose() * S * b - 2 * beta * des_force;

//    std::cout <<"A:\n"<< A << std::endl;
//    std::cout <<"H:\n"<< H << std::endl;
//    std::cout <<"g:\n"<< g << std::endl;
//    std::cout <<"C:\n"<< C << std::endl;
//    std::cout <<"dlb:\n"<< dlb << std::endl;
//    std::cout <<"dub:\n"<< dub << std::endl;

    auto qp_problem = qpOASES::QProblem(3 * num_contacts, 5 * num_contacts);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    qp_problem.setOptions(options);
    int max_solver_iter = 100;
    qp_problem.init(H.data(), g.data(), C.data(), nullptr, nullptr, dlb.data(), dub.data(), max_solver_iter);

    DVec<double> qp_sol = DVec<double>::Zero(3 * num_contacts);
    qp_problem.getPrimalSolution(qp_sol.data());

    for (auto & i : f_ff) {
        i.setZero();
    }
    DVec<double> tau = DVec<double>::Zero(12);

    tau = SelectS * JC.transpose() * qp_sol;
    for (int j = 0; j < 4; ++j) {
        f_ff[j] = -tau.segment(j * 3, 3); // J[contact_idx[j]].transpose() * qp_sol.segment(j * 3, 3);//state_est.rBody.transpose()
//        std::cout << "support foot_vel:\n" <<  << std::endl;
    }
    std::cout << "original force:\n" << qp_sol << std::endl;
    std::cout << "Af:\n" << A * qp_sol << std::endl;
    std::cout << "tau:\n" << tau << std::endl;

}
