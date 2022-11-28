/*! @file PositionVelocityEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PositionVelocityEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */
#include "Quadruped.h"
#include "PositionVelocityEstimator.h"
/*!
 * Initialize the state estimator
 */
void LinearKFPositionVelocityEstimator::setup() {
    double dt = this->_stateEstimatorData.parameters->controller_dt;
    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    _C.setZero();
    _C.block(0, 0, 3, 6) = C1;
    _C.block(3, 0, 3, 6) = C1;
    _C.block(6, 0, 3, 6) = C1;
    _C.block(9, 0, 3, 6) = C1;
    _C.block(0, 6, 12, 12) = -1 * Eigen::Matrix<double, 12, 12>::Identity();
    _C.block(12, 0, 3, 6) = C2;
    _C.block(15, 0, 3, 6) = C2;
    _C.block(18, 0, 3, 6) = C2;
    _C.block(21, 0, 3, 6) = C2;
    _C(27, 17) = 1;
    _C(26, 14) = 1;
    _C(25, 11) = 1;
    _C(24, 8) = 1;
    _P.setIdentity();
    _P = 100 * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<double, 12, 12>::Identity();
    _R0.setIdentity();
}

LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator() = default;

/*!
 * Run state estimator
 */
void LinearKFPositionVelocityEstimator::run() {
    double process_noise_pimu = this->_stateEstimatorData.parameters->imu_process_noise_position;
    double process_noise_vimu = this->_stateEstimatorData.parameters->imu_process_noise_velocity;
    double process_noise_pfoot = this->_stateEstimatorData.parameters->foot_process_noise_position;
    double sensor_noise_pimu_rel_foot = this->_stateEstimatorData.parameters->foot_sensor_noise_position;
    double sensor_noise_vimu_rel_foot = this->_stateEstimatorData.parameters->foot_sensor_noise_velocity;
    double sensor_noise_zfoot = this->_stateEstimatorData.parameters->foot_height_sensor_noise;

    Eigen::Matrix<double, 18, 18> Q = Eigen::Matrix<double, 18, 18>::Identity();
    Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
    Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
    Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * process_noise_pfoot;

    Eigen::Matrix<double, 28, 28> R = Eigen::Matrix<double, 28, 28>::Identity();
    R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * sensor_noise_pimu_rel_foot;
    R.block(12, 12, 12, 12) = _R0.block(12, 12, 12, 12) * sensor_noise_vimu_rel_foot;
    R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * sensor_noise_zfoot;

    int qindex = 0;
    int rindex1 = 0;
    int rindex2 = 0;
    int rindex3 = 0;

    Quadruped& quadruped = *(this->_stateEstimatorData.quadruped);
    Vec3<double> g = quadruped.gravity;
    Mat3<double> Rbod = this->_stateEstimatorData.result->rBody;
    Vec3<double> a = this->_stateEstimatorData.result->aWorld + g;
    Vec4<double> pzs = Vec4<double>::Zero();
    Vec4<double> trusts = Vec4<double>::Zero();
    Vec3<double> p0, v0;
    p0 << _xhat[0], _xhat[1], _xhat[2];
    v0 << _xhat[3], _xhat[4], _xhat[5];

    for (int i = 0; i < 4; i++) {
        int i1 = 3 * i;
        Vec3<double> ph = quadruped.hipLocationInKinematic.row(i);  // hip positions relative to CoM
        Vec3<double> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
        Vec3<double> dp_rel = this->_stateEstimatorData.legControllerData[i].v;
        Vec3<double> p_f = Rbod * p_rel;
        Vec3<double> dp_f = Rbod * (this->_stateEstimatorData.result->omegaBody.cross(p_rel) + dp_rel);

        qindex = 6 + i1;
        rindex1 = i1;
        rindex2 = 12 + i1;
        rindex3 = 24 + i;

        double trust = 1;
        double phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), 1);

        double trust_window = 0.2;
        // why 不应该摆动相，增大该值吗
        if (phase < trust_window) {
            trust = phase / trust_window;
        } else if (phase > (1 - trust_window)) {
            trust = (1 - phase) / trust_window;
        }

        double high_suspect_number(100);

        Q.block(qindex, qindex, 3, 3) = (1 + (1 - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
        R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
        R.block(rindex2, rindex2, 3, 3) = (1 + (1 - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
        R(rindex3, rindex3) = (1 + (1 - trust) * high_suspect_number) * R(rindex3, rindex3);

        trusts(i) = trust;

        _ps.segment(i1, 3) = -p_f;
        _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
        pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
    }

    Eigen::Matrix<double, 28, 1> y;
    y << _ps, _vs, pzs;
    _xhat = _A * _xhat + _B * a;
    Eigen::Matrix<double, 18, 18> At = _A.transpose();
    Eigen::Matrix<double, 18, 18> Pm = _A * _P * At + Q;
    Eigen::Matrix<double, 18, 28> Ct = _C.transpose();
    Eigen::Matrix<double, 28, 1> yModel = _C * _xhat;
    Eigen::Matrix<double, 28, 1> ey = y - yModel;
    Eigen::Matrix<double, 28, 28> S = _C * Pm * Ct + R;

    // todo compute LU only once
    Eigen::Matrix<double, 28, 1> S_ey = S.lu().solve(ey);
    _xhat += Pm * Ct * S_ey;

    Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C);
    // 误差协方差更新
    _P = (Eigen::Matrix<double, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

    Eigen::Matrix<double, 18, 18> Pt = _P.transpose();
    _P = (_P + Pt) / 2;

    if (_P.block(0, 0, 2, 2).determinant() > 0.000001) {
        _P.block(0, 2, 2, 16).setZero();
        _P.block(2, 0, 16, 2).setZero();
        _P.block(0, 0, 2, 2) /= 10;
    }

    this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
    this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
    this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.result->rBody.transpose() * this->_stateEstimatorData.result->vWorld;
}
