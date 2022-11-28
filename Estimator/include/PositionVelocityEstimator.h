#pragma once
#include "StateEstimatorContainer.h"

/*!
 * Position and velocity estimator based on a Kalman Filter.
 * This is the algorithm used in Mini Cheetah and Cheetah 3.
 */
class LinearKFPositionVelocityEstimator : public GenericEstimator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LinearKFPositionVelocityEstimator();
    void run() override;
    void setup() override;

private:
    Eigen::Matrix<double, 18, 1> _xhat;
    Eigen::Matrix<double, 12, 1> _ps;
    Eigen::Matrix<double, 12, 1> _vs;
    Eigen::Matrix<double, 18, 18> _A;
    Eigen::Matrix<double, 18, 18> _Q0;
    Eigen::Matrix<double, 18, 18> _P;
    Eigen::Matrix<double, 28, 28> _R0;
    Eigen::Matrix<double, 18, 3> _B;
    Eigen::Matrix<double, 28, 18> _C;
};
