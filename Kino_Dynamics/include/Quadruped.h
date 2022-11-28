//
// Created by HuNing-He on 22-5-22.
//

#ifndef MAIN_CPP_QUADRUPED_H
#define MAIN_CPP_QUADRUPED_H
#include <rbdl/rbdl_math.h>
using namespace RigidBodyDynamics::Math;

namespace robot_link {
    constexpr int Trunk = 0;
    constexpr int LF_Foot = 1;
    constexpr int RF_Foot = 2;
    constexpr int RH_Foot = 3;
    constexpr int LH_Foot = 4;
}

struct Quadruped {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double body_mass, hip_mass, thigh_mass, rotor_mass, link_mass, leg_mass, toe_mass;
    Matrix3d body_inertial, lf_hip_inertial, lh_hip_inertial;
    Matrix3d rf_hip_inertial, rh_hip_inertial, left_leg_inertial, right_leg_inertial;
    Matrix3d left_thigh_inertial, left_rotor_inertial, link_inertial;
    Matrix3d right_thigh_inertial, right_rotor_inertial, toe_inertial;
    Vector3d body_com, lf_hip_com, rf_hip_com, rh_hip_com, lh_hip_com;
    Vector3d left_thigh_com, right_thigh_com, left_rotor_com, right_rotor_com;
    Vector3d link_com, toe_com, leg_com;
    Vector3d gravity;

    double maxLegLength;
    double maxAngle; //60 degree
    double maxLateralForce;
    double maxVerticalForce;
    Matrix43 hipLocationInKinematic;
    Matrix43 hipLocation, thighLocation, rotorLocation, linkLocation;
    Vector3d legLocation, toeLocation;

    Quadruped() {
        body_inertial << 0.022612, 0, -0.001132,
                         0, 0.06388, 0.000001,
                         -0.001132, 0.000001, 0.074688;

        lh_hip_inertial << 0.000494, 0.00001, 0,
                           0.00001, 0.003311, 0,
                           0, 0, 0.003054;
        lf_hip_inertial << 0.000494, -0.00001, 0,
                           -0.00001, 0.003311, 0,
                           0, 0, 0.003054;

        rh_hip_inertial = lf_hip_inertial;
        rf_hip_inertial = lh_hip_inertial;

        left_leg_inertial << 0.00069448, 0, 0.00034033,
                             0, 0.00087332, 0,
                             0.00034033, 0, 0.00018219;
        right_leg_inertial = left_leg_inertial;

        left_thigh_inertial << 0.0047894, 0.0001697, -0.0003733,
                               0.0001697, 0.0040178, 0.00109,
                               -0.0003733, 0.00109, 0.0019091;
        right_thigh_inertial << 0.0047894, -0.0001697, -0.0003733,
                                -0.0001697, 0.0040178, -0.00109,
                                -0.0003733, -0.00109, 0.0019091;

        left_rotor_inertial << 0.00000531, 0.00000088, -0.00000027,
                               0.00000088, 0.00000524, -0.00000055,
                               -0.00000027, -0.00000055, 0.00000492;
        right_rotor_inertial << 0.00000531, -0.00000088, -0.00000027,
                                -0.00000088, 0.00000524, 0.00000055,
                                -0.00000027, 0.00000055, 0.00000492;

        link_inertial << 0.00052681, 0, -0.00000897,
                         0, 0.00052772, 0,
                         -0.00000897, 0, 0.00000112;

        toe_inertial << 1.6854e-05, 0, 0,
                        0, 1.6854e-05, 0,
                        0, 0, 1.6854e-05;

        body_mass = 8.744499;
        hip_mass = 0.794842;
        thigh_mass = 1.028914;
        rotor_mass = 0.03020152;
        link_mass = 0.03515229;
        leg_mass = 0.07978664;
        toe_mass = 0.06;

        body_com << -0.023142, 0, -0.004006;
        lf_hip_com << 0.055207, 0.000796, 0.0;
        rf_hip_com << 0.055207, -0.000796, 0.0;
        rh_hip_com << -0.055207, -0.000796, 0.0;
        lh_hip_com << -0.055207, 0.000796, 0.0;

        left_thigh_com << -0.003147, 0.031419, -0.02041768;
        right_thigh_com << -0.003147, -0.031419, -0.02041768;

        left_rotor_com << -0.0040243, 0.00796815, 0.0031742;
        right_rotor_com << -0.0040243, -0.00796815, 0.0031742;
        link_com << -0.00353189, 0.0, -0.10193327;
        leg_com << 0.03169095, 0.0, -0.0756275;
        toe_com << 0, 0, 0;

        maxLegLength = 0.34;
        maxAngle = 1.0472; //60 degree
        maxLateralForce = 350;
        maxVerticalForce = 350;

        hipLocationInKinematic.row(0) = Vector3d(0.206, 0.128, 0.0) - body_com;
        hipLocationInKinematic.row(1) = Vector3d(0.206, -0.128, 0.0) - body_com;
        hipLocationInKinematic.row(2) = Vector3d(-0.206, -0.128, 0.0) - body_com;
        hipLocationInKinematic.row(3) = Vector3d(-0.206, 0.128, 0.0) - body_com;

        hipLocation.row(0) = Vector3d(0.146, 0.052, 0);
        hipLocation.row(1) = Vector3d(0.146, -0.052, 0);
        hipLocation.row(2) = Vector3d(-0.146, -0.052, 0);
        hipLocation.row(3) = Vector3d(-0.146, 0.052, 0);

        thighLocation.row(0) = Vector3d(0.06, 0.0235, 0);
        thighLocation.row(1) = Vector3d(0.06, -0.0235, 0);
        thighLocation.row(2) = Vector3d(-0.06, -0.0235, 0);
        thighLocation.row(3) = Vector3d(-0.06, 0.0235, 0);

        rotorLocation.row(0) = Vector3d(0, 0.046, 0);
        rotorLocation.row(1) = Vector3d(0, -0.046, 0);
        rotorLocation.row(2) = Vector3d(0, -0.046, 0);
        rotorLocation.row(3) = Vector3d(0, 0.046, 0);

        linkLocation.row(0) = Vector3d(0, 0.0065, -0.015);
        linkLocation.row(1) = Vector3d(0, -0.0065, -0.015);
        linkLocation.row(2) = Vector3d(0, -0.0065, -0.015);
        linkLocation.row(3) = Vector3d(0, 0.0065, -0.015);

        legLocation = Vector3d(0, 0, -0.20923);
        toeLocation = Vector3d(0.11038982, 0, -0.19050112);
        gravity << 0, 0, -9.81;
    }
};
#endif //MAIN_CPP_QUADRUPED_H
