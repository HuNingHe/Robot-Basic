/*!
 * @file: VMC.h
 * @authors: HuNing-He
 * @date: 2022-11-29
 * @copyright (c) 2022 HuNing-He. This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

/*! reference
 * [R1] High-slope terrain locomotion for torque-controlled quadruped robots by Michele Focchi et,al
 */

#pragma once
#include "StateEstimatorContainer.h"
#include "Kinematic.h"
#include "FootSwingTrajectory.h"
#include "ControlFSMData.h"

struct ContactPlaneEstimate{
    Vec3<double> normal;
    Vec3<double> base1;
    Vec3<double> base2;
};

class VMC {
private:
    double mass;
    Mat3<double> body_inertial;
    double mu;
    Eigen::DiagonalMatrix<double, 6> S;     // weight matrix
    double alpha;
    double beta;

    Eigen::Matrix<double, 3, 3> W;          // a single block of W in paper
    Eigen::Matrix<double, 3, 3> Kp_sw;
    Eigen::Matrix<double, 3, 3> Kd_sw;
    FootSwingTrajectory swing_traj[4]; // foot swing trajectory
    ContactPlaneEstimate contact_plane[4];
    ControlFSMData *control_data;

    Vec3<double> des_com_vel;
    Vec3<double> des_com_pos;
    Vec3<double> des_angular_vel;
    RotMat<double> des_orientation;
    Vec3<double> des_rpy;

    int num_contacts;
    bool first_run;
    bool first_swing[4];

    GaitType gait_type;

public:
    VMC() = default;
    ~VMC() = default;
    explicit VMC(ControlFSMData *data);
    void initialize();
    void update_command();
    void run();
};

